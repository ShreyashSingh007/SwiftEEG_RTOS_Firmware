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


def section_is_valid(s) -> bool:
    """Mirrors dsp_section_is_valid: all finite, g and k positive."""
    v = [F32(x) for x in s]
    return all(np.isfinite(x) for x in v) and v[0] > 0 and v[1] > 0


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

    def apply(self, ch: int, x):
        x = F32(x)
        ic1, ic2 = self.ic1[ch], self.ic2[ch]
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
            if k > F32(2.0):
                tau = (k + np.sqrt(k * k - F32(4.0))) / (F32(4.0) * g)
            else:
                tau = F32(1.0) / (g * k)
            total = total + F32(4.6) * tau
        if not total < F32(4294967040.0):
            return 0xFFFFFFFF
        return int(np.ceil(total))


def cascade_apply(sections, x):
    """Sections in series over one channel, from rest."""
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
    # an impulse through the float32 loop, against the formula.
    for s in (n, lp, hp, design_notch(250.0, 50.0, 12.0)):
        imp = np.zeros(4096)
        imp[0] = 1.0
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

    # A retune keeps state only with the same number of sections.
    c = Cascade(1)
    assert c.set([n, lp])
    assert not c.retune([n])
    assert c.retune([design_notch(fs, 50.2, 30.0), lp])

    # Settling: nothing to settle is zero samples, and a Q 30 notch at 50 Hz
    # rings for about 4.6 Q / (pi f0).
    assert Cascade(1).settle_samples() == 0, "an empty cascade has nothing to settle"
    c = Cascade(1)
    c.set([n])
    expect = 4.6 * 30.0 / (math.pi * 50.0) * fs
    assert abs(c.settle_samples() - expect) / expect < 0.05, c.settle_samples()

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
    A("#endif /* SWIFTEEG_GOLDEN_DSP_H */")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    _self_test()
    if "--emit" in sys.argv:
        _emit()
