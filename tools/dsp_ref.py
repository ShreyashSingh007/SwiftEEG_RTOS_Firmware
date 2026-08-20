"""
Reference implementation of the SwiftEEG DSP primitives.

Oracle for the C code in src/dsp. Validation here is property-based rather
than a comparison against a second implementation of the same formula: a
notch must actually null at f0 and pass DC, a Butterworth section must be
-3 dB at its corner. That catches a wrong formula, which cross-checking two
copies of the same wrong formula would not.

Everything is computed in float32 to match the firmware, so golden vectors
compare meaningfully.

    python tools/dsp_ref.py           # self-test
    python tools/dsp_ref.py --emit    # regenerate tests/dsp/golden_dsp.h
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

F32 = np.float32


# --- coefficient design (RBJ cookbook), returned pre-normalised ------------
# a1/a2 come back NEGATED, matching the C inner loop.

def _normalise(b0, b1, b2, a0, a1, a2):
    inv = 1.0 / a0
    return (F32(b0 * inv), F32(b1 * inv), F32(b2 * inv),
            F32(-a1 * inv), F32(-a2 * inv))


def design_notch(fs, f0, q):
    w0 = 2 * math.pi * f0 / fs
    cw, alpha = math.cos(w0), math.sin(w0) / (2 * q)
    return _normalise(1.0, -2 * cw, 1.0, 1 + alpha, -2 * cw, 1 - alpha)


def design_lowpass(fs, fc, q):
    w0 = 2 * math.pi * fc / fs
    cw, alpha = math.cos(w0), math.sin(w0) / (2 * q)
    b1 = 1.0 - cw
    return _normalise(b1 / 2, b1, b1 / 2, 1 + alpha, -2 * cw, 1 - alpha)


def design_highpass(fs, fc, q):
    w0 = 2 * math.pi * fc / fs
    cw, alpha = math.cos(w0), math.sin(w0) / (2 * q)
    b0 = (1.0 + cw) / 2
    return _normalise(b0, -(1.0 + cw), b0, 1 + alpha, -2 * cw, 1 - alpha)


def response(coeffs, fs, freqs):
    """Magnitude of H(f) for one section. a1/a2 are stored negated."""
    b0, b1, b2, a1, a2 = (float(c) for c in coeffs)
    w = 2 * np.pi * np.asarray(freqs, dtype=float) / fs
    z1, z2 = np.exp(-1j * w), np.exp(-2j * w)
    num = b0 + b1 * z1 + b2 * z2
    den = 1.0 - a1 * z1 - a2 * z2
    return np.abs(num / den)


# --- filtering, transposed direct form II, float32 throughout --------------

def biquad_apply(coeffs, x):
    b0, b1, b2, a1, a2 = coeffs
    s1 = F32(0.0)
    s2 = F32(0.0)
    out = np.empty(len(x), dtype=F32)
    for i, xi in enumerate(x):
        xi = F32(xi)
        y = F32(b0 * xi + s1)
        s1 = F32(b1 * xi + a1 * y + s2)
        s2 = F32(b2 * xi + a2 * y)
        out[i] = y
    return out


def cascade_apply(sections, x):
    for c in sections:
        x = biquad_apply(c, x)
    return x


# --- stage 0: integer DC removal ------------------------------------------

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


# --- self-test -------------------------------------------------------------

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


def _emit() -> None:
    fs = 1000.0
    n = 128

    # Deterministic, non-trivial input: two tones plus mains plus a DC offset.
    t = np.arange(n) / fs
    sig = (30.0 * np.sin(2 * np.pi * 10.0 * t)
           + 8.0 * np.sin(2 * np.pi * 22.0 * t)
           + 50.0 * np.sin(2 * np.pi * 50.0 * t))
    sig_f32 = sig.astype(F32)

    notch = design_notch(fs, 50.0, 30.0)
    lp = design_lowpass(fs, 40.0, 1.0 / math.sqrt(2.0))
    notched = biquad_apply(notch, sig_f32)
    cascaded = cascade_apply([notch, lp], sig_f32)

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
    A(f"#define GOLDEN_DSP_FS       {fs:.1f}f")
    A(f"#define GOLDEN_DSP_N        {n}")
    A("#define GOLDEN_DSP_DC_SHIFT 10")
    A("")
    A("/* LSB in microvolts for VREF 4.5 V at gain 24. */")
    A(f"#define GOLDEN_DSP_LSB_UV   {lsb_uv(4.5, 24):.9e}f")
    A("")
    for name, c in (("notch50", notch), ("lp40", lp)):
        A(f"/* {name}: b0,b1,b2,a1,a2 (a terms negated) */")
        A(f"static const float golden_{name}_coeffs[5] = {{")
        A("\t\t" + ", ".join(f"{float(v):.9e}f" for v in c))
        A("};")
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
