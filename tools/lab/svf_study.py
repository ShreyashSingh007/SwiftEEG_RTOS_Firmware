"""
float32 state-variable (TPT/trapezoidal) biquads against the float64 RBJ
reference, on the cases where float32 TDF2 failed.
"""
import math
import sys

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import eeg_dsp  # noqa: E402


def svf_params(kind, fs, fc, q):
    g = math.tan(math.pi * fc / fs)
    k = 1.0 / q
    mix = {"hp": (1.0, -k, -1.0), "lp": (0.0, 0.0, 1.0), "notch": (1.0, -k, 0.0)}[kind]
    return (g, k) + mix


def rbj(kind, fs, fc, q):
    return {"hp": eeg_dsp.design_highpass, "lp": eeg_dsp.design_lowpass,
            "notch": eeg_dsp.design_notch}[kind](fs, fc, q)


def run_tdf2(sections, x):
    y = x.astype(np.float64)
    for b0, b1, b2, a1, a2 in sections:
        s1 = s2 = 0.0
        out = np.empty_like(y)
        for i in range(len(y)):
            xi = y[i]
            yi = b0 * xi + s1
            s1 = b1 * xi + a1 * yi + s2
            s2 = b2 * xi + a2 * yi
            out[i] = yi
        y = out
    return y


def run_svf(params, x, dt):
    y = x.astype(dt)
    two, one = dt(2), dt(1)
    for g, k, m0, m1, m2 in params:
        g, k, m0, m1, m2 = (dt(v) for v in (g, k, m0, m1, m2))
        a1 = one / (one + g * (g + k))
        a2 = g * a1
        a3 = g * a2
        ic1 = dt(0)
        ic2 = dt(0)
        out = np.empty_like(y)
        for i in range(len(y)):
            v0 = y[i]
            v3 = v0 - ic2
            v1 = a1 * ic1 + a2 * v3
            v2 = ic2 + a2 * ic1 + a3 * v3
            ic1 = two * v1 - ic1
            ic2 = two * v2 - ic2
            out[i] = m0 * v0 + m1 * v1 + m2 * v2
        y = out
    return y


def main():
    seconds = 40
    print(f"{'filter':<20}{'fs':>6}{'svf64 vs rbj64 max':>20}{'svf32 max uV':>14}"
          f"{'svf32 rms uV':>14}")
    for fs in (250.0, 500.0, 1000.0):
        n = int(seconds * fs)
        t = np.arange(n) / fs
        rng = np.random.default_rng(1)
        x = (3000 * np.sin(2 * np.pi * 0.05 * t) + 50 * np.sin(2 * np.pi * 1.0 * t)
             + 20 * np.sin(2 * np.pi * 10.0 * t) + 5 * rng.standard_normal(n))
        qs = eeg_dsp.butterworth_qs(4)
        cases = [(f"HP {fc} Hz x4", [("hp", fc, q) for q in qs]) for fc in (0.1, 0.3, 0.5, 1.0)]
        cases += [("LP 45 Hz x4", [("lp", 45.0, q) for q in qs]),
                  ("notch 50+100 Q12", [("notch", 50.0, 12.0), ("notch", 100.0, 12.0)])]
        for name, spec in cases:
            ref = run_tdf2([rbj(kd, fs, f, q) for kd, f, q in spec], x)
            p = [svf_params(kd, fs, f, q) for kd, f, q in spec]
            s64 = run_svf(p, x, np.float64)
            s32 = run_svf(p, x, np.float32).astype(np.float64)
            k0 = int(15 * fs)
            e = s32[k0:] - ref[k0:]
            print(f"{name:<20}{fs:>6.0f}{np.abs(s64 - ref).max():>20.2e}"
                  f"{np.abs(e).max():>14.4f}{np.sqrt(np.mean(e ** 2)):>14.4f}")


if __name__ == "__main__":
    main()
