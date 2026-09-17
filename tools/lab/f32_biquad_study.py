"""float32 TDF2 biquads at low corners: how far from float64 do they land?"""
import sys

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import eeg_dsp  # noqa: E402


def hp4(fs, fc):
    return [eeg_dsp.design_highpass(fs, fc, q) for q in eeg_dsp.butterworth_qs(4)]


def lp4(fs, fc):
    return [eeg_dsp.design_lowpass(fs, fc, q) for q in eeg_dsp.butterworth_qs(4)]


def notch2(fs, f0, q=12.0):
    return [eeg_dsp.design_notch(fs, f0, q), eeg_dsp.design_notch(fs, 2 * f0, q)]


def jury32(sections):
    worst = np.inf
    for s in sections:
        a1, a2 = np.float32(s[3]), np.float32(s[4])
        worst = min(worst, float((np.float32(1) - a2) - abs(a1)),
                    float(np.float32(1) - abs(a2)))
    return worst


def run(sections, x, coef_dtype, arith_dtype):
    y = x.astype(arith_dtype)
    for s in sections:
        b0, b1, b2, a1, a2 = (arith_dtype(coef_dtype(v)) for v in s)
        s1 = arith_dtype(0)
        s2 = arith_dtype(0)
        out = np.empty_like(y)
        for i in range(len(y)):
            xi = y[i]
            yi = b0 * xi + s1
            s1 = b1 * xi + a1 * yi + s2
            s2 = b2 * xi + a2 * yi
            out[i] = yi
        y = out
    return y


def amp_at(y, fs, f):
    n = len(y)
    t = np.arange(n) / fs
    return 2 * abs(np.mean(y * np.exp(-2j * np.pi * f * t)))


def main():
    seconds = 40
    print(f"{'filter':<22}{'fs':>6}{'jury32':>11}{'1Hz amp err %':>15}"
          f"{'coef-only max uV':>18}{'f32 max uV':>12}{'f32 rms uV':>12}")
    for fs in (250.0, 500.0, 1000.0):
        n = int(seconds * fs)
        t = np.arange(n) / fs
        rng = np.random.default_rng(1)
        x = (3000 * np.sin(2 * np.pi * 0.05 * t) + 50 * np.sin(2 * np.pi * 1.0 * t)
             + 20 * np.sin(2 * np.pi * 10.0 * t) + 5 * rng.standard_normal(n))
        cases = [(f"HP {fc} Hz x4", hp4(fs, fc)) for fc in (0.1, 0.3, 0.5, 1.0)]
        cases += [("LP 45 Hz x4", lp4(fs, 45.0)), ("notch 50+100 Q12", notch2(fs, 50.0))]
        for name, secs in cases:
            ref = run(secs, x, np.float64, np.float64)
            coef = run(secs, x, np.float32, np.float64)
            f32 = run(secs, x, np.float32, np.float32).astype(np.float64)
            k = int(15 * fs)
            a_ref = amp_at(ref[k:], fs, 1.0)
            a_f32 = amp_at(f32[k:], fs, 1.0)
            err_pct = (a_f32 / a_ref - 1) * 100 if a_ref > 1e-9 else float("nan")
            e_coef = np.abs(coef[k:] - ref[k:]).max()
            e = f32[k:] - ref[k:]
            print(f"{name:<22}{fs:>6.0f}{jury32(secs):>11.2e}{err_pct:>15.3f}"
                  f"{e_coef:>18.4f}{np.abs(e).max():>12.4f}"
                  f"{np.sqrt(np.mean(e ** 2)):>12.4f}")


if __name__ == "__main__":
    main()
