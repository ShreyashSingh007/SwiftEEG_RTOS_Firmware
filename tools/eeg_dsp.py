"""
Host-side DSP chain.

This is where filter settings get chosen. It runs on the PC so a change is
visible immediately, and it runs exactly the filters the firmware runs: the
same second-order sections in the same state-variable form - float64 here,
float32 on the device - so a setting that works here goes to the device as
the very same sections (device_stages()).

The chain, in order, and why that order:

  1. high-pass      removes electrode drift. Must come first: everything
                    after it works better on a signal centred near zero.
  2. notch          mains, and optionally its first harmonic.
  3. re-reference   common average. After the high-pass, so per-channel DC
                    offsets do not contaminate the average.
  4. low-pass       removes muscle activity and anything above the band of
                    interest.

Every stage is optional and every corner is settable, because the right
answer depends on what is being measured.

A setting change retunes a stage in place wherever it can. A section keeps
its state as integrators of the signal, and those stay meaningful when a
corner moves, so a new drift cut or smoothing corner applies from the next
sample with no restart transient. Only a stage that gains or loses sections
starts again, primed on its next input.

    python tools/eeg_dsp.py      # self-test
"""

from __future__ import annotations

import math

import numpy as np

# --- what the defaults mean ----------------------------------------------
#
# A 1 Hz high-pass settles the baseline within a few seconds and makes a
# trace readable. It also distorts slow ERP components - a P300 is a ~300 ms
# deflection - so for ERP work use 0.1 Hz and accept a wandering baseline.
# Recordings are raw either way, and can be filtered again later.
#
# The 100 Hz low-pass keeps the whole EEG band, gamma included.

DEFAULT_HIGHPASS_HZ = 1.0
DEFAULT_LOWPASS_HZ = 100.0
DEFAULT_NOTCH_HZ = 50.0
# Q sets how narrow the notch is: width in Hz is roughly f0/Q. Q=30 gives a
# 1.7 Hz notch, which is too sharp to be aimed by hand - measured mains sits
# at 49.6 Hz, and grid frequency wanders continuously. Q=12 is about 4 Hz
# wide, which covers the drift and still leaves alpha and beta untouched.
DEFAULT_NOTCH_Q = 12.0


def butterworth_qs(order: int) -> list[float]:
    """
    Q for each biquad in a Butterworth cascade of the given (even) order.

    A Butterworth response is a product of second-order sections whose poles
    sit evenly on a semicircle; these are the Qs those poles imply.
    """
    if order % 2 or order < 2:
        raise ValueError("order must be even and at least 2")

    n = order // 2
    qs = [1.0 / (2.0 * math.sin(math.pi * (2 * k + 1) / (2 * order)))
          for k in range(n)]

    # Ascending, purely so the list is deterministic - a cascade gives the
    # same response whatever order the sections run in.
    return sorted(qs)


# --- section design ----------------------------------------------------------
# A section is (g, k, m0, m1, m2): g = tan(pi fc / fs), k = 1 / Q, and how the
# output mixes the input, the band-pass and the low-pass. The responses are the
# audio-EQ cookbook's, prewarped at the corner.

def design_notch(fs, f0, q):
    g, k = math.tan(math.pi * f0 / fs), 1.0 / q
    return (g, k, 1.0, -k, 0.0)


def design_lowpass(fs, fc, q):
    g, k = math.tan(math.pi * fc / fs), 1.0 / q
    return (g, k, 0.0, 0.0, 1.0)


def design_highpass(fs, fc, q):
    g, k = math.tan(math.pi * fc / fs), 1.0 / q
    return (g, k, 1.0, -k, -1.0)


def response(sections, fs, freqs):
    """Magnitude of a cascade of sections at the given frequencies."""
    w = 2 * np.pi * np.asarray(freqs, dtype=float) / fs
    z1, z2 = np.exp(-1j * w), np.exp(-2j * w)
    h = np.ones_like(w, dtype=complex)

    for g, k, m0, m1, m2 in sections:
        gg = g * g
        den = (1.0 + g * k + gg) + 2.0 * (gg - 1.0) * z1 + (1.0 - g * k + gg) * z2
        num = m0 * den + m1 * g * (1.0 - z2) + m2 * gg * (1.0 + z1) ** 2
        h *= num / den

    return np.abs(h)


def find_mains(block, fs: float, nominal: float = 50.0) -> float | None:
    """
    Find the actual mains frequency near `nominal`.

    Grid frequency is never exactly 50 or 60 Hz and moves around by a few
    tenths. A notch narrow enough to spare the EEG either side is too narrow
    to hit a moving target by guesswork, so it gets aimed instead.

    Returns None when there is not enough data to be confident, in which case
    the caller should keep whatever it had.
    """
    x = np.asarray(block, dtype=float)
    if x.ndim > 1:
        x = x.mean(axis=1)

    n = len(x)
    # Resolution is fs/n, so a 0.1 Hz answer needs about 10 seconds.
    if n < int(fs * 4):
        return None

    w = np.hanning(n)
    sp = np.abs(np.fft.rfft((x - x.mean()) * w))
    fr = np.fft.rfftfreq(n, 1.0 / fs)

    lo, hi = nominal - 3.0, nominal + 3.0
    m = (fr >= lo) & (fr <= hi)
    if not np.any(m):
        return None

    idx = np.flatnonzero(m)
    k = idx[int(np.argmax(sp[m]))]

    # Parabolic interpolation across the peak: the true frequency sits
    # between bins, and at 4 s of data a bin is 0.25 Hz wide.
    if 0 < k < len(sp) - 1:
        a, b, c = sp[k - 1], sp[k], sp[k + 1]
        denom = a - 2 * b + c
        if denom != 0:
            k_off = 0.5 * (a - c) / denom
            return float(fr[k] + k_off * (fr[1] - fr[0]))

    return float(fr[k])


class Sections:
    """A cascade of sections with per-channel state, filtering sample by sample."""

    def __init__(self, channels: int):
        self.channels = channels
        self.sections: list[tuple] = []
        self._run: list[tuple] = []
        self.reset()

    def set(self, sections) -> str:
        """
        Load sections. Returns "same", "retuned" or "restarted".

        Unchanged sections leave the filter exactly as it was. Changed ones,
        as many as before, are retuned in place: the state is kept, because
        it is integrators of the signal, and those stay right when a corner
        moves. A different number of sections is a different filter, and it
        starts again, primed on its next input.
        """
        new = [tuple(float(v) for v in s) for s in sections]
        if new == self.sections:
            return "same"

        same_count = len(new) == len(self.sections)
        self.sections = new
        self._run = []
        for g, k, m0, m1, m2 in new:
            a1 = 1.0 / (1.0 + g * (g + k))
            a2 = g * a1
            self._run.append((a1, a2, g * a2, m0, m1, m2))

        if same_count:
            return "retuned"
        self.reset()
        return "restarted"

    def reset(self) -> None:
        """Start again from the next sample, primed on it."""
        n = len(self.sections)
        self.ic1 = np.zeros((self.channels, n))
        self.ic2 = np.zeros((self.channels, n))
        self.needs_prime = True

    def prime(self, x0: np.ndarray) -> np.ndarray:
        """
        Set every section's state as if x0 had always been its input, and
        return the cascade's output for it.

        For a steady input the band-pass integrator holds nothing and the
        low-pass one holds the input itself. Started from zero instead, the
        first sample is a step the size of each electrode's DC offset, which
        a 0.1 Hz high-pass takes most of a minute to recover from.
        """
        x = np.array(x0, dtype=np.float64, copy=True)
        for i, (_, _, _, m0, _, m2) in enumerate(self._run):
            self.ic1[:, i] = 0.0
            self.ic2[:, i] = x
            x = (m0 + m2) * x
        self.needs_prime = False
        return x

    def apply(self, block: np.ndarray) -> np.ndarray:
        """block is (samples, channels). Returns the same shape."""
        if not self.sections:
            return block
        if self.needs_prime and len(block):
            self.prime(block[0])

        out = np.array(block, dtype=np.float64, copy=True)

        for i, (a1, a2, a3, m0, m1, m2) in enumerate(self._run):
            ic1 = self.ic1[:, i].copy()
            ic2 = self.ic2[:, i].copy()

            for n in range(out.shape[0]):
                x = out[n]
                v3 = x - ic2
                v1 = a1 * ic1 + a2 * v3
                v2 = ic2 + a2 * ic1 + a3 * v3
                ic1 = 2.0 * v1 - ic1
                ic2 = 2.0 * v2 - ic2
                out[n] = m0 * x + m1 * v1 + m2 * v2

            self.ic1[:, i] = ic1
            self.ic2[:, i] = ic2

        return out


class Chain:
    """The full host chain. Settings can change between blocks."""

    def __init__(self, fs: float, channels: int = 8):
        self.fs = float(fs)
        self.channels = channels

        self.highpass_hz = DEFAULT_HIGHPASS_HZ
        self.lowpass_hz = DEFAULT_LOWPASS_HZ
        self.notch_hz = DEFAULT_NOTCH_HZ
        self.notch_q = DEFAULT_NOTCH_Q
        self.notch_harmonic = True
        self.car = True
        self.order = 4

        # Channels that make the common average. An electrode that is off or
        # barely touching carries drift and mains many times the size of the
        # EEG, and averaged in, a share of it lands on every other channel:
        # simulating one floating electrode, 34 uV RMS reached each of the
        # others at a 0.1 Hz high-pass. Left out, a channel is still
        # re-referenced and still shown.
        self.car_mask = np.ones(channels, dtype=bool)

        # When set, the notch is aimed at the measured mains frequency
        # rather than the nominal one.
        self.notch_track = True
        self.measured_mains: float | None = None

        self._hp = Sections(channels)
        self._notch = Sections(channels)
        self._lp = Sections(channels)
        self.rebuild()

    # -- configuration ----------------------------------------------------

    def _highpass_qs(self) -> list[float]:
        on = self.highpass_hz and self.highpass_hz > 0
        return butterworth_qs(self.order) if on else []

    def _lowpass_qs(self) -> list[float]:
        on = self.lowpass_hz and 0 < self.lowpass_hz < self.fs / 2.0 * 0.95
        return butterworth_qs(self.order) if on else []

    def rebuild(self) -> None:
        """
        Redesign every stage for the current settings and sample rate.

        An unchanged stage is left alone, a changed one is retuned in place,
        and only a stage that gains or loses sections restarts. This used to
        restart and re-prime every stage whenever any one changed: a new
        low-pass put the drift cut through a restart too, and the output was
        90 uV out for five seconds after it.
        """
        self._hp.set([design_highpass(self.fs, self.highpass_hz, q)
                      for q in self._highpass_qs()])
        self._notch.set(self._notch_sections())
        self._lp.set([design_lowpass(self.fs, self.lowpass_hz, q)
                      for q in self._lowpass_qs()])

    def _notch_freqs(self) -> list[float]:
        if not (self.notch_hz and self.notch_hz > 0):
            return []

        # Aim at the measured frequency when there is one. Mains is not
        # where the nameplate says: measured 49.6 Hz against a nominal
        # 50, which a narrow notch misses entirely.
        f0 = self.notch_hz
        if self.notch_track and self.measured_mains:
            if abs(self.measured_mains - self.notch_hz) < 3.0:
                f0 = self.measured_mains

        freqs = [f0]

        # The harmonic tracks the fundamental, so it moves with it.
        if self.notch_harmonic and f0 * 2.0 < self.fs / 2.0 * 0.95:
            freqs.append(f0 * 2.0)

        return freqs

    def _notch_sections(self) -> list:
        return [design_notch(self.fs, f, self.notch_q)
                for f in self._notch_freqs()]

    def device_stages(self) -> tuple[list, list]:
        """
        This chain as the device runs it: the sections before its common
        average and those after, each (g, k, m0, m1, m2).

        The very sections this chain runs. The device keeps its own 0.08 Hz
        DC removal in front of them, which this chain has no need of, so the
        two differ below about 0.3 Hz - by 3 % at 0.3 Hz, and by less than
        1 % from 0.6 Hz up.
        """
        return (self._hp.sections + self._notch.sections,
                list(self._lp.sections))

    @property
    def car_bits(self) -> int:
        """car_mask as the device takes it: bit n for channel n + 1."""
        return sum(1 << i for i, on in enumerate(self.car_mask) if on)

    def update_mains(self, block) -> bool:
        """
        Re-aim the notch from a recent block of data. Returns True if it moved.

        Only the notch changes, and it keeps its state. This used to rebuild
        and restart the whole chain - several times a minute as the grid
        wandered - and every restart put each electrode's whole DC offset
        back through the high-pass, fading over seconds at a 1 Hz corner and
        over most of a minute at 0.1 Hz.

        `block` must be recent, unbroken, and at the current rate: samples from
        before a rate change or a pause would aim the notch at a frequency
        that is not there. The caller keeps it so.
        """
        if not (self.notch_track and self.notch_hz):
            return False

        found = find_mains(block, self.fs, self.notch_hz)
        if found is None:
            return False

        # Only redesign when it has actually moved.
        if self.measured_mains and abs(found - self.measured_mains) < 0.05:
            return False

        self.measured_mains = found
        self._notch.set(self._notch_sections())
        return True

    def set_rate(self, fs: float) -> None:
        if abs(fs - self.fs) < 1e-6:
            return
        self.fs = float(fs)
        self.rebuild()
        self.reset()

    def reset(self) -> None:
        """Start again from the next sample, primed on it."""
        self._hp.reset()
        self._notch.reset()
        self._lp.reset()

    @property
    def settling_seconds(self) -> float:
        """
        Roughly how long before the output is trustworthy.

        Dominated by the high-pass: a corner at f takes a few 1/f to settle.
        On synthetic EEG with electrode offsets and drift, a new corner was
        within 1 uV 2.2 s after a change to 1 Hz, and 33 s after one down to
        0.1 Hz. Reported so a viewer can say so rather than present a
        transient as signal.
        """
        if not self.highpass_hz:
            return 0.0
        return min(60.0, 3.3 / self.highpass_hz)

    # -- processing -------------------------------------------------------

    def process(self, block: np.ndarray) -> np.ndarray:
        """
        block is (samples, channels) in microvolts. Returns the same shape.

        A stage that has restarted primes itself on the first sample it is
        given, so each settles on its own input and a restart of one leaves
        the others as they were.
        """
        out = self._hp.apply(block)
        out = self._notch.apply(out)

        if self.car:
            # Common average reference: subtract the mean across channels,
            # sample by sample. This is what pulls every trace onto a shared
            # zero, and it removes whatever is common to all of them -
            # usually mains and body potential, which no per-channel filter
            # can reach. Only channels in car_mask make the average; every
            # channel has it subtracted.
            m = self.car_mask
            if np.count_nonzero(m) >= 2:
                out = out - out[:, m].mean(axis=1, keepdims=True)

        return self._lp.apply(out)


# --- self-test ------------------------------------------------------------

def _cookbook(kind, fs, f0, q, freqs):
    """The audio-EQ cookbook biquad's magnitude, computed independently."""
    w0 = 2 * math.pi * f0 / fs
    cw, alpha = math.cos(w0), math.sin(w0) / (2 * q)
    b = {"notch": (1.0, -2 * cw, 1.0),
         "lowpass": ((1 - cw) / 2, 1 - cw, (1 - cw) / 2),
         "highpass": ((1 + cw) / 2, -(1 + cw), (1 + cw) / 2)}[kind]
    a = (1 + alpha, -2 * cw, 1 - alpha)
    z = np.exp(-1j * 2 * np.pi * np.asarray(freqs, dtype=float) / fs)
    return np.abs((b[0] + b[1] * z + b[2] * z * z) / (a[0] + a[1] * z + a[2] * z * z))


def _self_test() -> None:
    fs = 250.0

    qs = butterworth_qs(4)
    assert len(qs) == 2
    assert abs(qs[0] - 0.5412) < 1e-3, qs
    assert abs(qs[1] - 1.3066) < 1e-3, qs
    assert abs(butterworth_qs(2)[0] - 0.7071) < 1e-3

    # The sections are the cookbook's filters.
    freqs = np.linspace(0.05, fs / 2 * 0.99, 3000)
    for kind, design, f0, q in (("notch", design_notch, 49.7, 12.0),
                                ("lowpass", design_lowpass, 45.0, 0.5412),
                                ("highpass", design_highpass, 0.1, 1.3066)):
        err = np.abs(response([design(fs, f0, q)], fs, freqs)
                     - _cookbook(kind, fs, f0, q, freqs)).max()
        assert err < 1e-9, f"{kind} section is not the cookbook filter ({err:.1e})"

    # A 4th-order Butterworth is -3 dB at its corner, whatever the order.
    hp = [design_highpass(fs, 1.0, q) for q in butterworth_qs(4)]
    assert abs(response(hp, fs, [1.0])[0] - 0.7071) < 0.02, response(hp, fs, [1.0])
    assert response(hp, fs, [0.05])[0] < 0.01, "high-pass passes drift"
    assert response(hp, fs, [10.0])[0] > 0.99, "high-pass eats the band"

    lp = [design_lowpass(fs, 45.0, q) for q in butterworth_qs(4)]
    assert abs(response(lp, fs, [45.0])[0] - 0.7071) < 0.02
    assert response(lp, fs, [10.0])[0] > 0.99

    n = [design_notch(fs, 50.0, 30.0)]
    assert response(n, fs, [50.0])[0] < 1e-3, "notch does not null"
    assert response(n, fs, [40.0])[0] > 0.9, "notch too wide"
    assert response(n, fs, [10.0])[0] > 0.99, "notch reaches alpha"

    # End to end: 10 uV of alpha on 200 uV of drift and 60 uV of mains.
    t = np.arange(int(fs * 12)) / fs
    alpha = 10.0 * np.sin(2 * np.pi * 10.0 * t)
    drift = 200.0 * np.sin(2 * np.pi * 0.05 * t)
    mains = 60.0 * np.sin(2 * np.pi * 50.0 * t)

    sig = np.zeros((len(t), 8))
    for ch in range(8):
        sig[:, ch] = alpha + drift + mains + ch * 500.0  # each on its own offset

    chain = Chain(fs, 8)
    chain.car = False  # test the filters alone first
    out = chain.process(sig)

    tail = out[int(fs * 8):]
    peak = np.abs(tail).max()
    assert 8.0 < peak < 13.0, f"alpha came out at {peak:.1f} uV, expected ~10"

    # Every channel should now sit on zero despite starting 500 uV apart.
    means = np.abs(tail.mean(axis=0))
    assert means.max() < 1.0, f"channel offsets survived: {means.max():.2f} uV"

    # CAR: a signal common to every channel must vanish, and one that is not
    # common must survive.
    common = np.zeros((len(t), 8))
    for ch in range(8):
        common[:, ch] = mains + drift
    common[:, 3] += alpha

    c2 = Chain(fs, 8)
    c2.car = True
    o2 = c2.process(common)[int(fs * 8):]

    assert np.abs(o2[:, 0]).max() < 3.0, "CAR left a common signal behind"
    assert np.abs(o2[:, 3]).max() > 6.0, "CAR removed a real per-channel signal"

    # A channel left out of the average stays out of everyone's reference,
    # and is still re-referenced and kept itself.
    bad = common.copy()
    bad[:, 5] += 5000.0 * np.sin(2 * np.pi * 7.0 * t)
    c3 = Chain(fs, 8)
    c3.car_mask[5] = False
    o3 = c3.process(bad)[int(fs * 8):]
    assert np.abs(o3[:, 0]).max() < 3.0, "a masked channel leaked into the average"
    assert np.abs(o3[:, 5]).max() > 1000.0, "the masked channel itself was lost"
    assert c3.car_bits == 0xDF, hex(c3.car_bits)

    # Primed on its first sample, a 0.1 Hz high-pass starts settled instead
    # of spending most of a minute recovering from each electrode's offset.
    t2 = np.arange(int(fs * 30)) / fs
    sig2 = np.zeros((len(t2), 8))
    for ch in range(8):
        sig2[:, ch] = (10.0 * np.sin(2 * np.pi * 10.0 * t2)
                       + 20.0 * np.sin(2 * np.pi * 49.9 * t2)
                       + 300.0 * np.sin(2 * np.pi * 0.07 * t2)
                       + (ch - 4) * 15000.0)

    slow = Chain(fs, 8)
    slow.highpass_hz = 0.1
    slow.rebuild()
    start = np.abs(slow.process(sig2[:int(fs * 2)])).max()
    assert start < 50.0, f"start-up transient of {start:.0f} uV"

    def switched(before: dict, after: dict, at: int):
        """The live chain changed at `at`; the reference always had `after`."""
        live, ref = Chain(fs, 8), Chain(fs, 8)
        for c, settings in ((live, before), (ref, after)):
            for key, value in settings.items():
                setattr(c, key, value)
            c.rebuild()
        got, want = [], []
        for i in range(0, len(sig2), 6):
            if i == at:
                for key, value in after.items():
                    setattr(live, key, value)
                live.rebuild()
            got.append(live.process(sig2[i:i + 6]))
            want.append(ref.process(sig2[i:i + 6]))
        return np.abs(np.vstack(got) - np.vstack(want))[at:].max(axis=1)

    at = int(fs * 15) // 6 * 6
    quarter = int(fs * 0.25)

    # A new low-pass must not disturb the high-pass: it used to restart it.
    err = switched(dict(lowpass_hz=45.0), dict(lowpass_hz=100.0), at)
    assert err[quarter:].max() < 1.0, \
        f"a low-pass change left {err[quarter:].max():.1f} uV after 0.25 s"

    # Turning the notch off, likewise.
    err = switched(dict(notch_hz=50.0), dict(notch_hz=0.0), at)
    assert err[quarter:].max() < 1.0, \
        f"turning the notch off left {err[quarter:].max():.1f} uV after 0.25 s"

    # A new drift cut retunes in place rather than restarting.
    err = switched(dict(highpass_hz=0.5), dict(highpass_hz=1.0), at)
    assert err[int(fs * 2):].max() < 1.0, \
        f"a drift cut change left {err[int(fs * 2):].max():.1f} uV after 2 s"

    # Re-aiming the notch mid-stream must leave the rest of the chain alone.
    steady = Chain(fs, 8)
    steady.highpass_hz = 0.1
    steady.rebuild()
    moved = Chain(fs, 8)
    moved.highpass_hz = 0.1
    moved.rebuild()
    half = len(t2) // 2
    ref = steady.process(sig2)
    first = moved.process(sig2[:half])
    assert moved.update_mains(sig2[:half]), "the notch did not re-aim"
    second = moved.process(sig2[half:])
    jump = np.abs(np.vstack((first, second))[half:] - ref[half:]).max()
    assert jump < 5.0, f"re-aiming the notch disturbed the output by {jump:.0f} uV"

    # The device is sent this chain's own sections.
    c = Chain(fs, 8)
    pre, post = c.device_stages()
    assert pre == c._hp.sections + c._notch.sections and post == c._lp.sections

    print("eeg_dsp self-test: OK")
    print(f"  settling at {DEFAULT_HIGHPASS_HZ} Hz high-pass: "
          f"{Chain(fs).settling_seconds:.1f} s")


if __name__ == "__main__":
    _self_test()
