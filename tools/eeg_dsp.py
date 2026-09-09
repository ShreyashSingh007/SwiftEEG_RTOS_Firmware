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
DEFAULT_NOTCH_Q = 30.0


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
        self.set(sections, channels)

    def set(self, sections, channels: int) -> None:
        self.sections = [tuple(float(v) for v in s) for s in sections]
        self.channels = channels
        self.reset()

    def reset(self) -> None:
        n = max(1, len(self.sections))
        self.s1 = np.zeros((self.channels, n), dtype=np.float64)
        self.s2 = np.zeros((self.channels, n), dtype=np.float64)

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

        self._hp = Biquads([], channels)
        self._notch = Biquads([], channels)
        self._lp = Biquads([], channels)
        self.rebuild()

    # -- configuration ----------------------------------------------------

    def rebuild(self) -> None:
        """Redesign every stage for the current settings and sample rate."""
        nyq = self.fs / 2.0

        hp = []
        if self.highpass_hz and self.highpass_hz > 0:
            for q in butterworth_qs(self.order):
                hp.append(design_highpass(self.fs, self.highpass_hz, q))

        notch = []
        if self.notch_hz and self.notch_hz > 0:
            notch.append(design_notch(self.fs, self.notch_hz, self.notch_q))
            # The first harmonic is often as strong as the fundamental, but
            # only if it is actually below Nyquist.
            second = self.notch_hz * 2.0
            if self.notch_harmonic and second < nyq * 0.95:
                notch.append(design_notch(self.fs, second, self.notch_q))

        lp = []
        if self.lowpass_hz and 0 < self.lowpass_hz < nyq * 0.95:
            for q in butterworth_qs(self.order):
                lp.append(design_lowpass(self.fs, self.lowpass_hz, q))

        self._hp.set(hp, self.channels)
        self._notch.set(notch, self.channels)
        self._lp.set(lp, self.channels)

    def set_rate(self, fs: float) -> None:
        if abs(fs - self.fs) < 1e-6:
            return
        self.fs = float(fs)
        self.rebuild()

    def reset(self) -> None:
        self._hp.reset()
        self._notch.reset()
        self._lp.reset()

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
        out = self._hp.apply(block)
        out = self._notch.apply(out)

        if self.car:
            # Common average reference: subtract the mean across channels,
            # sample by sample. This is what pulls every trace onto a shared
            # zero, and it removes whatever is common to all of them -
            # usually mains and body potential, which no per-channel filter
            # can reach.
            out = out - out.mean(axis=1, keepdims=True)

        return self._lp.apply(out)


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

    print("eeg_dsp self-test: OK")
    print(f"  settling at {DEFAULT_HIGHPASS_HZ} Hz high-pass: "
          f"{Chain(fs).settling_seconds:.1f} s")


if __name__ == "__main__":
    _self_test()
