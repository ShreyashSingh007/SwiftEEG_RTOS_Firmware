"""
Reference implementation of the SwiftEEG DSP primitives.

Oracle for the C code in src/dsp. Validation here is property-based rather
than a comparison against a second implementation of the same formula: a
notch must actually null at f0 and pass DC, a Butterworth section must be
-3 dB at its corner. That catches a wrong formula, which cross-checking two
copies of the same wrong formula would not.

A section is a state-variable filter with trapezoidal integrators - see
src/dsp/dsp.h for why not a direct form. Filtering is computed in float32,
operation for operation as the firmware does it, so golden vectors compare
meaningfully.

    python tools/dsp_ref.py           # self-test
    python tools/dsp_ref.py --emit    # regenerate tests/dsp/golden_dsp.h
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

F32 = np.float32

MAX_SECTIONS = 8


# --- section design ---------------------------------------------------------
# A section is (g, k, m0, m1, m2) as float32: g = tan(pi fc / fs), k = 1 / Q,
# and how the output mixes the input, the band-pass and the low-pass.

def _section(fs, fc, q, m0, m1_per_k, m2):
    g = math.tan(math.pi * fc / fs)
    k = 1.0 / q
    return (F32(g), F32(k), F32(m0), F32(m1_per_k * k), F32(m2))


def design_notch(fs, f0, q):
    return _section(fs, f0, q, 1.0, -1.0, 0.0)


def design_lowpass(fs, fc, q):
    return _section(fs, fc, q, 0.0, 0.0, 1.0)


def design_highpass(fs, fc, q):
    return _section(fs, fc, q, 1.0, -1.0, -1.0)


def design_notch_f32(fs, f0, q):
    """
    A notch designed as dsp_design_notch designs it on the device, in
    float32. The device designs its own mains notch, and a mirror of that
    chain has to design it the same way, not in float64 and rounded.
    """
    g = np.tan(F32(np.pi) * F32(f0) / F32(fs))
    k = F32(1.0) / F32(q)
    return (F32(g), k, F32(1.0), F32(-1.0) * k, F32(0.0))


def butterworth_qs(order: int) -> list[float]:
    """Q of each section of a Butterworth cascade of the given even order."""
    if order % 2 or order < 2:
        raise ValueError("order must be even and at least 2")
    return sorted(1.0 / (2.0 * math.sin(math.pi * (2 * k + 1) / (2 * order)))
                  for k in range(order // 2))


def transfer(section):
    """The section's H(z) as ((n0, n1, n2), (1, d1, d2)), in float64."""
    g, k, m0, m1, m2 = (float(v) for v in section)
    gg = g * g
    d0 = 1.0 + g * k + gg
    den = (1.0, 2.0 * (gg - 1.0) / d0, (1.0 - g * k + gg) / d0)
    num = ((m0 * d0 + m1 * g + m2 * gg) / d0,
           (2.0 * m0 * (gg - 1.0) + 2.0 * m2 * gg) / d0,
           (m0 * (1.0 - g * k + gg) - m1 * g + m2 * gg) / d0)
    return num, den


def response(sections, fs, freqs):
    """Magnitude of a cascade - or of one section - at the given frequencies."""
    if len(sections) == 5 and np.isscalar(sections[0]):
        sections = [sections]
    w = 2 * np.pi * np.asarray(freqs, dtype=float) / fs
    z1, z2 = np.exp(-1j * w), np.exp(-2j * w)
    h = np.ones_like(w, dtype=complex)
    for s in sections:
        (n0, n1, n2), (_, d1, d2) = transfer(s)
        h *= (n0 + n1 * z1 + n2 * z2) / (1.0 + d1 * z1 + d2 * z2)
    return np.abs(h)


def from_biquad(b0, b1, b2, a1, a2):
    """
    Any stable biquad as a section.

    a1 and a2 are the feedback terms with their signs flipped, as the
    audio-EQ cookbook normalises them:
    H = (b0 + b1 z^-1 + b2 z^-2) / (1 - a1 z^-1 - a2 z^-2).
    """
    d1, d2 = -a1, -a2
    lo, hi = 1.0 + d1 + d2, 1.0 - d1 + d2
    if lo <= 0.0 or hi <= 0.0 or abs(d2) >= 1.0:
        raise ValueError("that biquad is not stable")

    g = math.sqrt(lo / hi)
    d0 = 4.0 / hi
    k = 2.0 * (1.0 - d2) / (g * hi)
    m0 = d0 * (b0 - b1 + b2) / 4.0
    m2 = (d0 * b1 - 2.0 * m0 * (g * g - 1.0)) / (2.0 * g * g)
    m1 = (d0 * (b0 - b2) - 2.0 * g * k * m0) / (2.0 * g)
    return tuple(F32(v) for v in (g, k, m0, m1, m2))


SECTION_MIX_MAX = F32(1e3)
SECTION_G_MAX = F32(1e3)
SECTION_TAU_MAX = F32(1e6)


def slowest_tau(g, k):
    """
    Mirrors slowest_tau in src/dsp/dsp.c, float32 and in the same order: the
    time constant, in samples, of the section's pole nearest the unit circle,
    where the bilinear transform puts it. See dsp.c for the derivation.
    """
    g, k = F32(g), F32(k)
    with np.errstate(over="ignore", divide="ignore"):
        if k < F32(2.0):
            return (F32(1.0) + g * g) / (g * k)
        h = F32(1.0) / g if g > F32(1.0) else g
        return (k + np.sqrt(k * k - F32(4.0))) / (F32(4.0) * h)


def section_is_valid(s) -> bool:
    """
    Mirrors dsp_section_is_valid: all finite, g and k positive, and nothing
    past what float32 runs as designed - see dsp.c for the bounds.
    """
    v = [F32(x) for x in s]
    if not all(np.isfinite(x) for x in v):
        return False
    g, k, m0, m1, m2 = v
    return bool(F32(0.0) < g <= SECTION_G_MAX and k > F32(0.0)
                and abs(m0) <= SECTION_MIX_MAX and abs(m1) <= SECTION_MIX_MAX
                and abs(m2) <= SECTION_MIX_MAX
                and slowest_tau(g, k) <= SECTION_TAU_MAX)


# --- filtering, float32 throughout, as src/dsp/dsp.c does it ---------------

def derive(section):
    """What the firmware's loop runs, derived the way it derives it."""
    g, k, m0, m1, m2 = (F32(v) for v in section)
    a1 = F32(1.0) / (F32(1.0) + g * (g + k))
    a2 = g * a1
    a3 = g * a2
    return a1, a2, a3, m0, m1, m2


class Cascade:
    """Mirror of dsp_cascade_t: sections, and two states per section per channel."""

    TWO = F32(2.0)

    def __init__(self, channels: int):
        self.channels = channels
        self.sections: list[tuple] = []
        self._run: list[tuple] = []
        self.reset()

    def _load(self, sections) -> None:
        self.sections = [tuple(F32(v) for v in s) for s in sections]
        self._run = [derive(s) for s in self.sections]

    def set(self, sections) -> bool:
        if len(sections) > MAX_SECTIONS or not all(map(section_is_valid, sections)):
            return False
        self._load(sections)
        self.reset()
        return True

    def retune(self, sections) -> bool:
        if (len(sections) != len(self.sections)
                or not all(map(section_is_valid, sections))):
            return False
        self._load(sections)
        return True

    def reset(self) -> None:
        zero = F32(0.0)
        self.ic1 = [[zero] * MAX_SECTIONS for _ in range(self.channels)]
        self.ic2 = [[zero] * MAX_SECTIONS for _ in range(self.channels)]
        self.pending = [True] * self.channels

    def apply(self, ch: int, x):
        x = F32(x)
        ic1, ic2 = self.ic1[ch], self.ic2[ch]
        if self.pending[ch]:
            # Primed on this sample, as dsp_cascade_apply primes.
            v = x
            for i, (_, _, _, m0, _, m2) in enumerate(self._run):
                ic1[i] = F32(0.0)
                ic2[i] = v
                v = (m0 + m2) * v
            self.pending[ch] = False
        for i, (a1, a2, a3, m0, m1, m2) in enumerate(self._run):
            v3 = x - ic2[i]
            v1 = a1 * ic1[i] + a2 * v3
            v2 = ic2[i] + a2 * ic1[i] + a3 * v3
            ic1[i] = self.TWO * v1 - ic1[i]
            ic2[i] = self.TWO * v2 - ic2[i]
            x = m0 * x + m1 * v1 + m2 * v2
        return x

    def settle_samples(self) -> int:
        """Mirrors dsp_cascade_settle_samples."""
        total = F32(0.0)
        for g, k, *_ in self.sections:
            d = F32(0.5) * k if k < F32(2.0) else F32(2.0) / k
            total = total + ((F32(4.96) + F32(3.04) * d * d) * slowest_tau(g, k)
                             + F32(2.0))
        if not total < F32(4294967040.0):
            return 0xFFFFFFFF
        return int(np.ceil(total))


def cascade_apply(sections, x):
    """Sections in series over one channel, primed on the first sample."""
    c = Cascade(1)
    if not c.set(sections):
        raise ValueError("invalid section")
    return np.array([c.apply(0, v) for v in x], dtype=F32)


# --- stage 0: integer DC removal ---------------------------------------------

def dc_apply(samples, shift):
    """Mirrors dsp_dc_apply exactly, including priming on the first sample."""
    acc = 0
    primed = False
    out = []
    for s in samples:
        s = int(s)
        if not primed:
            acc = s << shift
            primed = True
        else:
            acc += s - (acc >> shift)
        out.append(s - (acc >> shift))
    return out


def lsb_uv(vref_volts, gain):
    return (2.0 * vref_volts) / (gain * (1 << 24)) * 1e6


def lsb_uv_f32(vref_volts, gain):
    """Mirrors dsp_lsb_uv: float32, and in the same order."""
    if gain == 0:
        return F32(0.0)
    return (F32(2.0) * F32(vref_volts)) / (F32(gain) * F32(16777216.0)) * F32(1e6)


# --- mains tracking ----------------------------------------------------------

class MainsTracker:
    """
    Mirror of dsp_mains_t, operation for operation - see src/dsp/mains.h for
    how it works. float32 on the per-sample path, double beyond it, as the
    firmware does it.
    """

    TWO_PI = 6.283185307179586476925
    LOWPASS_HZ = 1.5
    BASEBAND_HZ = 10
    FINE, COARSE, WINDOW = 10, 2, 40
    LOCK, SEARCH_HZ, AGREE_HZ, REAIM_HZ = 0.6, 3.0, 0.05, 0.005
    RENORMALISE = 8192

    def __init__(self, fs, nominal, start=0.0):
        sps = int(fs)
        if (not 250.0 <= fs <= 64000.0 or not nominal > 0 or float(sps) != fs
                or sps % self.BASEBAND_HZ
                or float(F32(nominal)) + self.SEARCH_HZ >= float(F32(fs)) / 2.0):
            raise ValueError("cannot track mains at that rate")
        self.fs, self.nominal = F32(fs), F32(nominal)
        self.k = F32(1.0 - math.exp(-self.TWO_PI * self.LOWPASS_HZ / float(self.fs)))
        self.decim = sps // self.BASEBAND_HZ
        self.c, self.s = F32(1.0), F32(0.0)
        self.turns = 0
        self.i1 = self.i2 = self.q1 = self.q2 = F32(0.0)
        self.sum_i = self.sum_q = F32(0.0)
        self.phase = 0
        self.ring_i = [F32(0.0)] * self.FINE
        self.ring_q = [F32(0.0)] * self.FINE
        self.ring_len = 0
        self.head = 0
        self._clear()
        self.candidate = F32(0.0)
        self.estimate = F32(0.0)
        start = F32(start)
        near = start > 0 and abs(float(start) - float(self.nominal)) <= self.SEARCH_HZ
        self._aim(float(start) if near else float(self.nominal))

    def _aim(self, hz: float) -> None:
        w = self.TWO_PI * hz / float(self.fs)
        self.mix = F32(hz)
        self.cw = F32(math.cos(w))
        self.sw = F32(math.sin(w))

    def _clear(self) -> None:
        zero = F32(0.0)
        self.fine_i = self.fine_q = self.coarse_i = self.coarse_q = zero
        self.power = zero
        self.count = 0

    def _settle(self, hz) -> bool:
        agreed = (self.candidate != 0
                  and abs(float(hz) - float(self.candidate)) < self.AGREE_HZ)
        if agreed:
            self.estimate = hz
        self.candidate = hz
        return agreed

    def _baseband(self, zi, zq) -> bool:
        if self.ring_len == self.FINE:
            lag = (self.head + self.FINE - self.COARSE) % self.FINE
            fi, fq = self.ring_i[self.head], self.ring_q[self.head]
            ci, cq = self.ring_i[lag], self.ring_q[lag]
            self.fine_i = self.fine_i + (zi * fi + zq * fq)
            self.fine_q = self.fine_q + (zq * fi - zi * fq)
            self.coarse_i = self.coarse_i + (zi * ci + zq * cq)
            self.coarse_q = self.coarse_q + (zq * ci - zi * cq)
            self.power = self.power + (zi * zi + zq * zq)
            self.count += 1

        self.ring_i[self.head] = zi
        self.ring_q[self.head] = zq
        self.head = (self.head + 1) % self.FINE
        if self.ring_len < self.FINE:
            self.ring_len += 1

        if self.count < self.WINDOW:
            return False

        agreed = False
        fine = math.sqrt(float(self.fine_i) * float(self.fine_i)
                         + float(self.fine_q) * float(self.fine_q))
        if self.power > 0 and fine >= self.LOCK * float(self.power):
            df = (-math.atan2(float(self.coarse_q), float(self.coarse_i))
                  / (self.TWO_PI * self.COARSE / self.BASEBAND_HZ))
            if abs(df) < 0.3:
                df = (-math.atan2(float(self.fine_q), float(self.fine_i))
                      / (self.TWO_PI * self.FINE / self.BASEBAND_HZ))
            hz = float(self.mix) + df
            lo = float(self.nominal) - self.SEARCH_HZ
            hi = float(self.nominal) + self.SEARCH_HZ
            hz = lo if hz < lo else hi if hz > hi else hz
            agreed = self._settle(F32(hz))
            if abs(hz - float(self.mix)) > self.REAIM_HZ:
                self._aim(hz)
                self.ring_len = 0
                self.head = 0
        else:
            self.candidate = F32(0.0)

        self._clear()
        return agreed

    def push(self, x) -> bool:
        """One sample in microvolts; True when two answers in a row agree."""
        x = F32(x)
        c = self.c * self.cw - self.s * self.sw
        s = self.s * self.cw + self.c * self.sw
        self.i1 = self.i1 + self.k * (x * c - self.i1)
        self.i2 = self.i2 + self.k * (self.i1 - self.i2)
        self.q1 = self.q1 + self.k * (x * s - self.q1)
        self.q2 = self.q2 + self.k * (self.q1 - self.q2)
        self.turns += 1
        if self.turns == self.RENORMALISE:
            g = F32(1.0 / math.sqrt(float(c) * float(c) + float(s) * float(s)))
            self.c, self.s = c * g, s * g
            self.turns = 0
        else:
            self.c, self.s = c, s
        self.sum_i = self.sum_i + self.i2
        self.sum_q = self.sum_q + self.q2
        self.phase += 1
        if self.phase < self.decim:
            return False
        zi = self.sum_i / F32(self.decim)
        zq = self.sum_q / F32(self.decim)
        self.phase = 0
        self.sum_i = self.sum_q = F32(0.0)
        return self._baseband(zi, zq)


def mains_test_signal(fs, seconds, mains_uv, mains_hz, seed):
    """Random-walk EEG, alpha, slow drift and mains, as float32 microvolts."""
    rng = np.random.default_rng(seed)
    n = int(seconds * fs)
    t = np.arange(n) / fs
    walk = np.cumsum(rng.standard_normal(n))
    walk -= np.convolve(walk, np.ones(int(fs)) / fs, mode="same")
    x = (8.0 * walk / walk.std()
         + 10.0 * np.sin(2 * np.pi * 10.0 * t)
         + 300.0 * np.sin(2 * np.pi * 0.1 * t)
         + mains_uv * np.sin(2 * np.pi * mains_hz * t))
    return x.astype(F32)


def mains_estimates(x, fs, nominal=50.0):
    """Every agreed estimate over a signal, as (sample, Hz)."""
    tracker = MainsTracker(fs, nominal)
    return [(i, float(tracker.estimate)) for i, v in enumerate(x) if tracker.push(v)]


# --- self-test ---------------------------------------------------------------

def _rbj(kind, fs, f0, q):
    """Audio-EQ cookbook biquads in float64, a1/a2 negated - the old design."""
    w0 = 2 * math.pi * f0 / fs
    cw, alpha = math.cos(w0), math.sin(w0) / (2 * q)
    a0, a1, a2 = 1 + alpha, -2 * cw, 1 - alpha
    b = {"notch": (1.0, -2 * cw, 1.0),
         "lowpass": ((1 - cw) / 2, 1 - cw, (1 - cw) / 2),
         "highpass": ((1 + cw) / 2, -(1 + cw), (1 + cw) / 2),
         "peak": (1 + alpha * 2.0, -2 * cw, 1 - alpha * 2.0)}[kind]
    if kind == "peak":
        a0, a2 = 1 + alpha / 2.0, 1 - alpha / 2.0
    return (b[0] / a0, b[1] / a0, b[2] / a0, -a1 / a0, -a2 / a0)


def _rbj_response(c, fs, freqs):
    b0, b1, b2, a1, a2 = c
    w = 2 * np.pi * np.asarray(freqs, dtype=float) / fs
    z1, z2 = np.exp(-1j * w), np.exp(-2j * w)
    return np.abs((b0 + b1 * z1 + b2 * z2) / (1.0 - a1 * z1 - a2 * z2))


def _self_test() -> None:
    fs = 1000.0

    # Notch: nulls at f0, passes DC and Nyquist.
    n = design_notch(fs, 50.0, 30.0)
    assert response(n, fs, [50.0])[0] < 1e-3, "notch does not null at f0"
    assert abs(response(n, fs, [0.0])[0] - 1.0) < 1e-6, "notch alters DC"
    assert abs(response(n, fs, [499.0])[0] - 1.0) < 1e-2, "notch alters HF"
    # Neighbouring EEG must survive: 10 Hz away should be near unity.
    assert response(n, fs, [40.0])[0] > 0.9, "notch far too wide"

    # Butterworth Q gives -3 dB at the corner.
    q = 1.0 / math.sqrt(2.0)
    lp = design_lowpass(fs, 40.0, q)
    assert abs(response(lp, fs, [0.0])[0] - 1.0) < 1e-6, "lowpass DC gain"
    assert abs(response(lp, fs, [40.0])[0] - 0.70710678) < 1e-3, "lowpass -3dB"

    hp = design_highpass(fs, 0.5, q)
    assert response(hp, fs, [0.0])[0] < 1e-6, "highpass passes DC"
    assert abs(response(hp, fs, [0.5])[0] - 0.70710678) < 1e-3, "highpass -3dB"

    assert abs(butterworth_qs(4)[0] - 0.5412) < 1e-3
    assert abs(butterworth_qs(4)[1] - 1.3066) < 1e-3

    # The transfer function the formulas claim is the one the loop runs:
    # an impulse through the float32 loop, against the formula. The impulse
    # comes one sample in: a cascade primes on its first sample, and primed
    # on the impulse itself it would see a step down instead.
    for s in (n, lp, hp, design_notch(250.0, 50.0, 12.0)):
        imp = np.zeros(4096)
        imp[1] = 1.0
        h = np.abs(np.fft.rfft(cascade_apply([s], imp).astype(float)))
        freqs = np.fft.rfftfreq(len(imp), 1.0 / fs)
        err = np.abs(h - response(s, fs, freqs)).max()
        assert err < 1e-3, f"loop and transfer function disagree by {err:.2e}"

    # The same filters as the cookbook designs this replaced, to float32.
    for kind, f0, qq in (("notch", 50.0, 12.0), ("lowpass", 45.0, 0.5412),
                         ("highpass", 0.1, 1.3066), ("peak", 10.0, 2.0)):
        c = _rbj(kind, fs, f0, qq)
        s = from_biquad(*c)
        freqs = np.linspace(0.01, 499.0, 997)
        err = np.abs(response(s, fs, freqs) - _rbj_response(c, fs, freqs)).max()
        assert err < 1e-4, f"{kind}: from_biquad is off by {err:.2e}"
        if kind != "peak":
            direct = {"notch": design_notch, "lowpass": design_lowpass,
                      "highpass": design_highpass}[kind](fs, f0, qq)
            assert np.allclose(s, direct, rtol=1e-5, atol=1e-6), (kind, s, direct)

    # The reason for this form: a 0.1 Hz high-pass at 1000 SPS in float32,
    # on 3 mV of drift, against the float64 cookbook filter.
    t = np.arange(20000) / fs
    x = 3000 * np.sin(2 * np.pi * 0.05 * t) + 20 * np.sin(2 * np.pi * 10.0 * t)
    secs = [design_highpass(fs, 0.1, qq) for qq in butterworth_qs(4)]
    y32 = cascade_apply(secs, x).astype(float)
    y64 = x.copy()
    for qq in butterworth_qs(4):
        b0, b1, b2, a1, a2 = _rbj("highpass", fs, 0.1, qq)
        s1 = s2 = 0.0
        for i, xi in enumerate(y64):
            yi = b0 * xi + s1
            s1 = b1 * xi + a1 * yi + s2
            s2 = b2 * xi + a2 * yi
            y64[i] = yi
    rms = np.sqrt(np.mean((y32[10000:] - y64[10000:]) ** 2))
    assert rms < 0.2, f"float32 0.1 Hz high-pass is {rms:.3f} uV RMS out"

    # Validity: g and k must be positive and everything finite.
    assert section_is_valid(n)
    assert not section_is_valid((0.0, 1.0, 1.0, -1.0, 0.0))
    assert not section_is_valid((0.1, 0.0, 1.0, 0.0, 0.0))
    assert not section_is_valid((0.1, 1.0, float("nan"), 0.0, 0.0))
    assert not section_is_valid((float("inf"), 1.0, 1.0, 0.0, 0.0))

    # Finite is not enough: each of these would load and then overflow, or
    # ring undamped in float32.
    g, k = float(n[0]), float(n[1])
    for huge in ((g, k, 1e30, -k, 0.0), (4096.0, k, 1.0, -k, 0.0),
                 (2e19, k, 1.0, -k, 0.0), (g, 1e-9, 1.0, -1e-9, 0.0),
                 (g, 1e30, 1.0, -1.0, 0.0), (1e-9, k, 1.0, -k, 0.0)):
        assert not section_is_valid(huge), huge

    # And the furthest out the host tools and the device design is inside:
    # the slowest high-pass section, a low-pass just under 0.475 fs, and the
    # narrowest notch the device takes.
    for s in (design_highpass(16000.0, 0.05, butterworth_qs(8)[-1]),
              design_lowpass(250.0, 118.0, butterworth_qs(8)[-1]),
              design_notch_f32(16000.0, 50.0, 255.0)):
        assert section_is_valid(s), s

    # A retune keeps state only with the same number of sections.
    c = Cascade(1)
    assert c.set([n, lp])
    assert not c.retune([n])
    assert c.retune([design_notch(fs, 50.2, 30.0), lp])

    # Settling: nothing to settle is zero samples, and from whatever state a
    # restart leaves, the loop's own output falls below 1 % of its peak
    # inside the estimate - which is not much longer. At 250 SPS the harmonic
    # notch sits where the bilinear transform squeezes its poles up to the
    # unit circle, and an order-2 Butterworth high-pass peaks late: the
    # estimate from the analogue prototype flagged 94 samples of the first,
    # which rings for up to 194, and 518 of the second, which takes 641.
    assert Cascade(1).settle_samples() == 0, "an empty cascade has nothing to settle"
    rng = np.random.default_rng(2)
    for secs in ([design_notch_f32(250.0, 50.0, 12.0), design_notch_f32(250.0, 100.0, 12.0)],
                 [design_highpass(250.0, 0.5, butterworth_qs(2)[0])]):
        c = Cascade(1)
        c.set(secs)
        settle = c.settle_samples()
        worst = 0
        for _ in range(16):
            c.pending[0] = False
            for i in range(len(secs)):
                c.ic1[0][i], c.ic2[0][i] = (F32(v) for v in rng.standard_normal(2))
            y = np.abs([float(c.apply(0, 0.0)) for _ in range(2 * settle)])
            worst = max(worst, int(np.nonzero(y > 0.01 * y.max())[0][-1]) + 1)
        assert worst <= settle < 2 * worst, (secs, settle, worst)

    # A restart primes on its next input: a high-pass fed a constant gives
    # nothing from the very first sample, and a low-pass gives the constant.
    c = Cascade(1)
    c.set([design_highpass(fs, 0.5, q)])
    assert all(abs(float(c.apply(0, 1000.0))) < 1e-3 for _ in range(20))
    c.set([lp])
    assert all(abs(float(c.apply(0, 1000.0)) - 1000.0) < 1e-2 for _ in range(20))

    # DC blocker: priming means the very first output is exactly zero, and a
    # constant input stays at zero rather than ramping.
    const = dc_apply([12345] * 64, 12)
    assert const[0] == 0, f"DC blocker not primed: {const[0]}"
    assert all(v == 0 for v in const), "constant input leaked through"

    # A step away from the primed level must appear, then decay.
    step = dc_apply([1000] * 8 + [2000] * 64, 8)
    assert step[8] > 900, "step suppressed"
    assert abs(step[-1]) < abs(step[8]), "step did not decay"

    # Negative values must behave (arithmetic shift, not logical).
    neg = dc_apply([-5000] * 32, 10)
    assert all(v == 0 for v in neg), f"negative constant leaked: {neg[:4]}"

    # Known LSB for the board's configuration.
    assert abs(lsb_uv(4.5, 24) - 0.02235) < 1e-5, f"{lsb_uv(4.5, 24)}"
    assert abs(float(lsb_uv_f32(4.5, 24)) - lsb_uv(4.5, 24)) < 1e-8

    # The device's float32 notch design is the same filter as the float64 one.
    for f0, qq in ((50.0, 12.0), (99.6, 12.0), (60.0, 30.0)):
        assert np.allclose(design_notch_f32(250.0, f0, qq), design_notch(250.0, f0, qq),
                           rtol=1e-6, atol=1e-7), f0

    # Mains tracking. 20 uV of mains a little off 50 Hz, under EEG and drift:
    # agreed within 15 s, and every estimate within 20 mHz.
    got = mains_estimates(mains_test_signal(250.0, 30, 20.0, 49.62, 3), 250.0)
    assert got and got[0][0] < 15 * 250, f"no estimate in 15 s: {got[:2]}"
    assert all(abs(hz - 49.62) < 0.02 for _, hz in got), got

    # No mains at all: nothing agreed on, in a minute.
    got = mains_estimates(mains_test_signal(250.0, 60, 0.0, 50.0, 4), 250.0)
    assert not got, f"agreed on mains that is not there: {got}"

    # Far from nominal, and 60 Hz mains at 1000 SPS.
    got = mains_estimates(mains_test_signal(250.0, 30, 20.0, 49.2, 5), 250.0)
    assert got and abs(got[-1][1] - 49.2) < 0.05, got
    got = mains_estimates(mains_test_signal(1000.0, 30, 20.0, 60.3, 6), 1000.0, 60.0)
    assert got and abs(got[-1][1] - 60.3) < 0.02, got

    # A restart starts at the last estimate, unless it is too far off.
    assert abs(float(MainsTracker(250.0, 50.0, 49.6).mix) - 49.6) < 1e-4
    assert float(MainsTracker(250.0, 50.0, 40.0).mix) == 50.0
    for fs, nominal in ((255.0, 50.0), (200.0, 50.0), (250.0, 0.0)):
        try:
            MainsTracker(fs, nominal)
            raise AssertionError(f"tracking accepted at {fs} SPS, {nominal} Hz")
        except ValueError:
            pass

    print("dsp_ref self-test: OK")


# --- golden vector emission ------------------------------------------------

OUT = pathlib.Path(__file__).resolve().parent.parent / "tests" / "dsp" / "golden_dsp.h"


def _c_f32(vals):
    return ",\n\t\t".join(
        ", ".join(f"{float(v):.9e}f" for v in vals[i:i + 6])
        for i in range(0, len(vals), 6)
    )


def _c_i32(vals):
    return ",\n\t\t".join(
        ", ".join(str(int(v)) for v in vals[i:i + 10])
        for i in range(0, len(vals), 10)
    )


def _c_section(s):
    return "{ " + ", ".join(f"{float(v):.9e}f" for v in s) + " }"


def _emit() -> None:
    fs = 1000.0
    n = 128
    n_hp = 1024
    q_bw = 1.0 / math.sqrt(2.0)
    q_hp = butterworth_qs(4)[0]

    # Deterministic, non-trivial input: two tones plus mains.
    t = np.arange(n) / fs
    sig = (30.0 * np.sin(2 * np.pi * 10.0 * t)
           + 8.0 * np.sin(2 * np.pi * 22.0 * t)
           + 50.0 * np.sin(2 * np.pi * 50.0 * t))
    sig_f32 = sig.astype(F32)

    notch = design_notch(fs, 50.0, 30.0)
    lp = design_lowpass(fs, 40.0, q_bw)
    hp = design_highpass(fs, 0.1, q_hp)
    notched = cascade_apply([notch], sig_f32)
    cascaded = cascade_apply([notch, lp], sig_f32)

    # A low corner on a large slow signal: the case this form exists for.
    t_hp = np.arange(n_hp) / fs
    drift = (3000.0 * np.sin(2 * np.pi * 0.3 * t_hp)
             + 20.0 * np.sin(2 * np.pi * 10.0 * t_hp)).astype(F32)
    highpassed = cascade_apply([hp], drift)

    # DC path: large offset with a small AC component on top.
    raw = [int(400000 + 500 * math.sin(2 * math.pi * 10.0 * i / fs))
           for i in range(n)]
    dc_out = dc_apply(raw, 10)

    # Mains tracking: 22 s at 250 SPS, 20 uV of mains at 49.73 Hz.
    mains_fs, mains_nominal = 250.0, 50.0
    mains_in = mains_test_signal(mains_fs, 22, 20.0, 49.73, 11)
    mains_out = mains_estimates(mains_in, mains_fs, mains_nominal)
    assert mains_out, "the tracker agreed on nothing to test against"

    L = []
    A = L.append
    A("/*")
    A(" * GENERATED FILE - DO NOT EDIT BY HAND.")
    A(" *")
    A(" * Produced by tools/dsp_ref.py, which is the authoritative reference for")
    A(" * the DSP primitives. If the C disagrees with these values, the C is wrong.")
    A(" *")
    A(" * Regenerate:  python tools/dsp_ref.py --emit")
    A(" */")
    A("#ifndef SWIFTEEG_GOLDEN_DSP_H")
    A("#define SWIFTEEG_GOLDEN_DSP_H")
    A("")
    A("#include <stdint.h>")
    A("")
    A('#include "dsp.h"')
    A("")
    A(f"#define GOLDEN_DSP_FS       {fs:.1f}f")
    A(f"#define GOLDEN_DSP_N        {n}")
    A(f"#define GOLDEN_DSP_N_HP     {n_hp}")
    A("#define GOLDEN_DSP_DC_SHIFT 10")
    A(f"#define GOLDEN_DSP_HP_Q     {q_hp:.9e}f")
    A("")
    A("/* LSB in microvolts for VREF 4.5 V at gain 24. */")
    A(f"#define GOLDEN_DSP_LSB_UV   {lsb_uv(4.5, 24):.9e}f")
    A("")
    for name, what, s in (("notch50", "50 Hz notch, Q 30", notch),
                          ("lp40", "40 Hz low-pass, Q 0.7071", lp),
                          ("hp01", "0.1 Hz high-pass, Q 0.5412", hp)):
        A(f"/* {what}: g, k, m0, m1, m2 */")
        A(f"static const dsp_section_t golden_{name} = {_c_section(s)};")
        A("")
    A("static const float golden_dsp_input[GOLDEN_DSP_N] = {")
    A("\t\t" + _c_f32(sig_f32))
    A("};")
    A("")
    A("static const float golden_dsp_notched[GOLDEN_DSP_N] = {")
    A("\t\t" + _c_f32(notched))
    A("};")
    A("")
    A("static const float golden_dsp_cascaded[GOLDEN_DSP_N] = {")
    A("\t\t" + _c_f32(cascaded))
    A("};")
    A("")
    A("/* 3 mV at 0.3 Hz with 20 uV of 10 Hz on top, and what the 0.1 Hz")
    A(" * high-pass makes of it. */")
    A("static const float golden_dsp_drift[GOLDEN_DSP_N_HP] = {")
    A("\t\t" + _c_f32(drift))
    A("};")
    A("")
    A("static const float golden_dsp_highpassed[GOLDEN_DSP_N_HP] = {")
    A("\t\t" + _c_f32(highpassed))
    A("};")
    A("")
    A("/* Integer DC removal is exact maths - the C must match bit for bit. */")
    A("static const int32_t golden_dc_input[GOLDEN_DSP_N] = {")
    A("\t\t" + _c_i32(raw))
    A("};")
    A("")
    A("static const int32_t golden_dc_output[GOLDEN_DSP_N] = {")
    A("\t\t" + _c_i32(dc_out))
    A("};")
    A("")
    A("/*")
    A(" * Mains tracking: EEG, drift and 20 uV of mains at 49.73 Hz, and the")
    A(" * sample at which the tracker agreed on each estimate, and the estimate.")
    A(" */")
    A(f"#define GOLDEN_MAINS_FS      {mains_fs:.1f}f")
    A(f"#define GOLDEN_MAINS_NOMINAL {mains_nominal:.1f}f")
    A(f"#define GOLDEN_MAINS_N       {len(mains_in)}")
    A(f"#define GOLDEN_MAINS_EVENTS  {len(mains_out)}")
    A("")
    A("static const float golden_mains_input[GOLDEN_MAINS_N] = {")
    A("\t\t" + _c_f32(mains_in))
    A("};")
    A("")
    A("static const int32_t golden_mains_at[GOLDEN_MAINS_EVENTS] = {")
    A("\t\t" + _c_i32([i for i, _ in mains_out]))
    A("};")
    A("")
    A("static const float golden_mains_hz[GOLDEN_MAINS_EVENTS] = {")
    A("\t\t" + _c_f32([F32(hz) for _, hz in mains_out]))
    A("};")
    A("")
    A("#endif /* SWIFTEEG_GOLDEN_DSP_H */")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    _self_test()
    if "--emit" in sys.argv:
        _emit()
