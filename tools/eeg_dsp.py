"""
Host-side DSP chain.

This is where filter settings get chosen. It runs on the PC so a change is
visible immediately, and it is built from the same biquad forms the firmware
uses - transposed direct form II, RBJ cookbook coefficients - so a setting
that works here transfers to the device as coefficients rather than as a
rewrite.

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

    python tools/eeg_dsp.py      # self-test
"""

from __future__ import annotations

import math

import numpy as np

F32 = np.float32

# --- what the defaults mean ----------------------------------------------
#
# 0.5 Hz high-pass makes a trace look clean and settles the baseline. It also
# distorts slow ERP components - a P300 is a ~300 ms deflection, and a corner
# this high visibly changes its shape. For ERP work use 0.1 Hz and accept a
# wandering baseline, or record raw and filter twice.
#
# 45 Hz low-pass sits below the mains notch's neighbourhood and above the
# EEG bands that matter (delta through low gamma).

DEFAULT_HIGHPASS_HZ = 0.5
DEFAULT_LOWPASS_HZ = 45.0
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


# --- coefficient design, matching src/dsp/dsp.c ---------------------------
# Returned as (b0, b1, b2, a1, a2) with the a terms already negated, which is
# the form the firmware's inner loop expects.

def _norm(b0, b1, b2, a0, a1, a2):
    inv = 1.0 / a0
    return (b0 * inv, b1 * inv, b2 * inv, -a1 * inv, -a2 * inv)


def design_notch(fs, f0, q):
    w0 = 2 * math.pi * f0 / fs
    cw, alpha = math.cos(w0), math.sin(w0) / (2 * q)
    return _norm(1.0, -2 * cw, 1.0, 1 + alpha, -2 * cw, 1 - alpha)


def design_lowpass(fs, fc, q):
    w0 = 2 * math.pi * fc / fs
    cw, alpha = math.cos(w0), math.sin(w0) / (2 * q)
    b1 = 1.0 - cw
    return _norm(b1 / 2, b1, b1 / 2, 1 + alpha, -2 * cw, 1 - alpha)


def design_highpass(fs, fc, q):
    w0 = 2 * math.pi * fc / fs
    cw, alpha = math.cos(w0), math.sin(w0) / (2 * q)
    b0 = (1.0 + cw) / 2
    return _norm(b0, -(1.0 + cw), b0, 1 + alpha, -2 * cw, 1 - alpha)


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


def response(sections, fs, freqs):
    """Magnitude of the cascade at the given frequencies."""
    w = 2 * np.pi * np.asarray(freqs, dtype=float) / fs
    z1, z2 = np.exp(-1j * w), np.exp(-2j * w)
    h = np.ones_like(w, dtype=complex)

    for b0, b1, b2, a1, a2 in sections:
        h *= (b0 + b1 * z1 + b2 * z2) / (1.0 - a1 * z1 - a2 * z2)

    return np.abs(h)


class Biquads:
    """A cascade with per-channel state, filtering sample by sample."""

    def __init__(self, sections, channels: int):
        self.sections: list[tuple] = []
        self.channels = 0
        self.set(sections, channels)

    def set(self, sections, channels: int, keep_state: bool = False) -> bool:
        """
        Load coefficients. Returns True if the filter was restarted.

        Unchanged coefficients leave the filter exactly as it was. Changed
        ones restart it from rest - unless keep_state is set, for a small
        retune such as a notch following the mains a tenth of a hertz: the
        old state is nearly right for the new filter, and a restart would put
        a transient into the signal every time.
        """
        new = [tuple(float(v) for v in s) for s in sections]
        same_shape = (channels == self.channels
                      and len(new) == len(self.sections))

        if same_shape and new == self.sections:
            return False

        self.sections = new
        if keep_state and same_shape:
            return False

        self.channels = channels
        self.reset()
        return True

    def reset(self) -> None:
        n = max(1, len(self.sections))
        self.s1 = np.zeros((self.channels, n), dtype=np.float64)
        self.s2 = np.zeros((self.channels, n), dtype=np.float64)

    def prime(self, x0: np.ndarray) -> np.ndarray:
        """
        Set every section's state as if x0 had always been its input.

        A filter started from rest sees the first sample as a step up from
        zero, and a 100 mV electrode offset through a 0.1 Hz high-pass is a
        transient lasting most of a minute. Primed, it starts settled.
        Returns the cascade's output for that steady input.
        """
        x = np.array(x0, dtype=np.float64, copy=True)
        for si, (b0, b1, b2, a1, a2) in enumerate(self.sections):
            y = x * ((b0 + b1 + b2) / (1.0 - a1 - a2))  # DC gain
            self.s2[:, si] = b2 * x + a2 * y
            self.s1[:, si] = b1 * x + a1 * y + self.s2[:, si]
            x = y
        return x

    def apply(self, block: np.ndarray) -> np.ndarray:
        """block is (samples, channels). Returns the same shape."""
        if not self.sections:
            return block

        out = np.array(block, dtype=np.float64, copy=True)

        for si, (b0, b1, b2, a1, a2) in enumerate(self.sections):
            s1 = self.s1[:, si]
            s2 = self.s2[:, si]

            for i in range(out.shape[0]):
                x = out[i]
                y = b0 * x + s1
                s1 = b1 * x + a1 * y + s2
                s2 = b2 * x + a2 * y
                out[i] = y

            self.s1[:, si] = s1
            self.s2[:, si] = s2

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

        self._hp = Biquads([], channels)
        self._notch = Biquads([], channels)
        self._lp = Biquads([], channels)
        self._primed = False
        self.rebuild()

    # -- configuration ----------------------------------------------------

    def rebuild(self) -> None:
        """
        Redesign every stage for the current settings and sample rate.

        A stage whose coefficients come out unchanged keeps its state, so
        changing one setting does not restart the others. Any stage that
        does restart is primed again on the next sample.
        """
        nyq = self.fs / 2.0

        hp = []
        if self.highpass_hz and self.highpass_hz > 0:
            for q in butterworth_qs(self.order):
                hp.append(design_highpass(self.fs, self.highpass_hz, q))

        lp = []
        if self.lowpass_hz and 0 < self.lowpass_hz < nyq * 0.95:
            for q in butterworth_qs(self.order):
                lp.append(design_lowpass(self.fs, self.lowpass_hz, q))

        restarted = self._hp.set(hp, self.channels)
        restarted |= self._notch.set(self._notch_sections(), self.channels)
        restarted |= self._lp.set(lp, self.channels)
        if restarted:
            self._primed = False

    def _notch_sections(self) -> list:
        if not (self.notch_hz and self.notch_hz > 0):
            return []

        # Aim at the measured frequency when there is one. Mains is not
        # where the nameplate says: measured 49.6 Hz against a nominal
        # 50, which a narrow notch misses entirely.
        f0 = self.notch_hz
        if self.notch_track and self.measured_mains:
            if abs(self.measured_mains - self.notch_hz) < 3.0:
                f0 = self.measured_mains

        sections = [design_notch(self.fs, f0, self.notch_q)]

        # The harmonic tracks the fundamental, so it moves with it.
        second = f0 * 2.0
        if self.notch_harmonic and second < self.fs / 2.0 * 0.95:
            sections.append(design_notch(self.fs, second, self.notch_q))

        return sections

    def update_mains(self, block) -> bool:
        """
        Re-aim the notch from a recent block of data. Returns True if it moved.

        Only the notch changes, and it keeps its state. This used to rebuild
        and restart the whole chain - several times a minute as the grid
        wandered - and every restart put each electrode's whole DC offset
        back through the high-pass, fading over seconds at a 1 Hz corner and
        over most of a minute at 0.1 Hz.
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
        self._notch.set(self._notch_sections(), self.channels,
                        keep_state=True)
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
        self._primed = False

    @property
    def settling_seconds(self) -> float:
        """
        Roughly how long before the output is trustworthy.

        Dominated by the high-pass: a corner at f takes on the order of a few
        1/f to settle. Reported so a viewer can grey out the start rather
        than presenting a transient as signal.
        """
        if not self.highpass_hz:
            return 0.0
        return min(10.0, 3.0 / self.highpass_hz)

    # -- processing -------------------------------------------------------

    def process(self, block: np.ndarray) -> np.ndarray:
        """
        block is (samples, channels) in microvolts. Returns the same shape.
        """
        if not self._primed and len(block):
            self._prime(np.asarray(block[0], dtype=np.float64))

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

    def _prime(self, x0: np.ndarray) -> None:
        """
        Start every stage settled on the first sample instead of from zero.

        From zero, the first sample is a step the size of each electrode's
        whole DC offset, and a 0.1 Hz high-pass takes most of a minute to
        recover from it.
        """
        y = self._notch.prime(self._hp.prime(x0))
        if self.car and np.count_nonzero(self.car_mask) >= 2:
            y = y - y[self.car_mask].mean()
        self._lp.prime(y)
        self._primed = True


# --- self-test ------------------------------------------------------------

def _self_test() -> None:
    fs = 250.0

    qs = butterworth_qs(4)
    assert len(qs) == 2
    assert abs(qs[0] - 0.5412) < 1e-3, qs
    assert abs(qs[1] - 1.3066) < 1e-3, qs
    assert abs(butterworth_qs(2)[0] - 0.7071) < 1e-3

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

    # Primed on its first sample, a 0.1 Hz high-pass starts settled instead
    # of spending most of a minute recovering from each electrode's offset.
    t2 = np.arange(int(fs * 30)) / fs
    sig2 = np.zeros((len(t2), 8))
    for ch in range(8):
        sig2[:, ch] = (10.0 * np.sin(2 * np.pi * 10.0 * t2)
                       + 20.0 * np.sin(2 * np.pi * 49.9 * t2)
                       + (ch - 4) * 15000.0)

    slow = Chain(fs, 8)
    slow.highpass_hz = 0.1
    slow.rebuild()
    start = np.abs(slow.process(sig2[:int(fs * 2)])).max()
    assert start < 50.0, f"start-up transient of {start:.0f} uV"

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

    print("eeg_dsp self-test: OK")
    print(f"  settling at {DEFAULT_HIGHPASS_HZ} Hz high-pass: "
          f"{Chain(fs).settling_seconds:.1f} s")


if __name__ == "__main__":
    _self_test()
