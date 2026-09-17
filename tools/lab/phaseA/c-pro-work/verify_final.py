# Final evidence for R3-DSP-10 and R3-DSP-06 against the edited tools/dsp_ref.py mirror.
import math, sys, itertools
import numpy as np
RTOS = r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools"
sys.path.insert(0, RTOS)
import dsp_ref as ref
import eeg_dsp
exec(open("settle_sweep.py").read().split("def case(")[0].replace("MULT = float(sys.argv[1]) if len(sys.argv) > 1 else 4.6", ""))  # section, bw_qs, derive, step, state_space, responses, eeg, realistic_deltas, actual
F32 = np.float32

def settle(secs):
    c = ref.Cascade(1)
    assert c.set(secs), secs
    return c.settle_samples()

def worst_arbitrary(secs, n_dirs=4096, seed=5):
    run = [derive(tuple(F32(v) for v in s)) for s in secs]
    A, C = state_space(run)
    est = settle(secs)
    N = int(2.2 * est) + 64
    Y = responses(A, C, N)
    D = np.random.default_rng(seed).standard_normal((2 * len(secs), n_dirs))
    w = actual(Y, D)
    assert w < N - 8, "simulation window too short"
    return w, est, Y

print("=== R3-DSP-06: every host / device design passes the new bounds ===")
rates = [250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0, 16000.0]
count = 0; worst_tau = (0, None); max_g = 0; max_m = 0
def check(s, what):
    global count, worst_tau, max_g, max_m
    s32 = tuple(F32(v) for v in s)
    assert ref.section_is_valid(s32), (what, s32)
    t = float(ref.slowest_tau(s32[0], s32[1]))
    if t > worst_tau[0]:
        worst_tau = (t, what)
    max_g = max(max_g, float(s32[0])); max_m = max(max_m, max(abs(float(v)) for v in s32[2:]))
    count += 1
for fs in rates:
    for fc in (0.05, 0.07, 0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 3.0, 5.0):
        for order in (2, 4, 6, 8):
            for q in ref.butterworth_qs(order):
                check(eeg_dsp.design_highpass(fs, fc, q), f"eeg_dsp hp {fc} Hz o{order} @ {fs:.0f}")
                check(ref.design_highpass(fs, fc, q), f"dsp_ref hp {fc} Hz o{order} @ {fs:.0f}")
        t = math.tan(math.pi * fc / fs); p = (1 - t) / (1 + t)   # order 1 as a biquad
        check(ref.from_biquad((1 + p) / 2, -(1 + p) / 2, 0.0, p, 0.0), f"order-1 hp {fc} Hz @ {fs:.0f}")
    for fc in np.geomspace(20.0, 0.4749 * fs, 40):
        for order in (2, 4, 6, 8):
            for q in ref.butterworth_qs(order):
                check(eeg_dsp.design_lowpass(fs, fc, q), f"eeg_dsp lp {fc:.1f} Hz o{order} @ {fs:.0f}")
                check(ref.design_lowpass(fs, fc, q), f"dsp_ref lp {fc:.1f} Hz o{order} @ {fs:.0f}")
        t = math.tan(math.pi * fc / fs); p = (1 - t) / (1 + t)
        check(ref.from_biquad((1 - p) / 2, (1 - p) / 2, 0.0, p, 0.0), f"order-1 lp {fc:.1f} Hz @ {fs:.0f}")
    for nominal, off, q in itertools.product((50.0, 60.0), (-3, -1, -0.5, 0, 0.5, 1, 3),
                                             (1, 2, 5, 10, 12, 20, 30, 50, 100, 255)):
        f0 = nominal + off
        for f in [f0] + ([2 * f0] if 2 * f0 < 0.95 * 0.5 * fs else []):
            check(eeg_dsp.design_notch(fs, f, q), f"eeg_dsp notch {f} Q{q} @ {fs:.0f}")
            check(ref.design_notch(fs, f, q), f"dsp_ref notch {f} Q{q} @ {fs:.0f}")
            check(ref.design_notch_f32(fs, f, q), f"device notch {f} Q{q} @ {fs:.0f}")
print(f"{count} sections accepted; slowest tau {worst_tau[0]:.3g} samples ({worst_tau[1]}),"
      f" {1e6 / worst_tau[0]:.1f}x inside the bound; largest g {max_g:.3g}; largest |mix| {max_m:.3g}")

print("=== R3-DSP-06: huge-but-finite sections are refused ===")
g, k = (float(v) for v in ref.design_notch(1000.0, 50.0, 30.0)[:2])
for s, what in (((g, k, 1e30, -k, 0.0), "m0 = 1e30"), ((g, k, 1.0, 2e3, 0.0), "m1 = 2e3"),
                ((4096.0, k, 1.0, -k, 0.0), "g = 4096"), ((2e19, k, 1.0, -k, 0.0), "g = 2e19"),
                ((4096.0, 1e-4, 1.0, -1e-4, 0.0), "g = 4096, k = 1e-4 (a3 == 1.0)"),
                ((g, 1e-9, 1.0, -1e-9, 0.0), "k = 1e-9"), ((g, 1e30, 1.0, -1.0, 0.0), "k = 1e30"),
                ((1e-9, k, 1.0, -k, 0.0), "g = 1e-9"), ((1e3, 1e-3, 1.0, -1e-3, 0.0), "g = 1000, k = 1e-3")):
    print(f"  {what:34s} valid={ref.section_is_valid(s)}  tau={float(ref.slowest_tau(s[0], s[1])):.3g}")
    assert not ref.section_is_valid(s), what

print("=== R3-DSP-10: estimate vs worst ring-down over 4096 states (float32 designs as the device's) ===")
dev = lambda fs, f, q: ref.design_notch_f32(fs, f, q)
tests = {
    "Q30 notch 50 Hz @1k (ztest golden_notch50)": [ref.design_notch(1000.0, 50.0, 30.0)],
    "notch 50+100 Q12 @250": [dev(250.0, 50.0, 12.0), dev(250.0, 100.0, 12.0)],
    "notch 60 Q12 @250": [dev(250.0, 60.0, 12.0)],
    "hp 0.5 Hz order 2 @250": [ref.design_highpass(250.0, 0.5, ref.butterworth_qs(2)[0])],
    "hp 0.1 Hz order 4 @1k": [ref.design_highpass(1000.0, 0.1, q) for q in ref.butterworth_qs(4)],
    "lp 100 Hz order 2 @250": [ref.design_lowpass(250.0, 100.0, ref.butterworth_qs(2)[0])],
}
for name, secs in tests.items():
    w, est, _ = worst_arbitrary(secs)
    print(f"  {name:44s} flagged {est:6d}  worst ring-down {w:6d}  ({est / w:.2f}x)")
    assert est >= w

print("=== sections that share a pole: identical cascades ===")
for name, one in (("Q12 notch 50 Hz @250", dev(250.0, 50.0, 12.0)),
                  ("Q0.5 hp 1 Hz @250 (double pole)", ref.design_highpass(250.0, 1.0, 0.5)),
                  ("Q0.707 hp 1 Hz @250", ref.design_highpass(250.0, 1.0, 0.70710678)),
                  ("Q2.56 lp 40 Hz @250", ref.design_lowpass(250.0, 40.0, 2.5629154))):
    for n in (2, 4, 8):
        w, est, _ = worst_arbitrary([one] * n, n_dirs=2048)
        print(f"  {n} x {name:34s} flagged {est:6d}  worst ring-down {w:6d}  ({est / w:.2f}x)")
        assert est >= w

print("=== old vs new flagged, review cases (realistic 5:1 and 1:1 mains, 48 restarts each) ===")
for fs, f0 in ((250.0, 50.0), (250.0, 60.0), (500.0, 50.0), (1000.0, 50.0)):
    secs = [dev(fs, f0, 12.0)] + ([dev(fs, 2 * f0, 12.0)] if 2 * f0 < 0.95 * 0.5 * fs else [])
    run = [derive(tuple(F32(v) for v in s)) for s in secs]
    A, C = state_space(run)
    est = settle(secs)
    old = int(math.ceil(sum(4.6 * (1 / (float(s[0]) * float(s[1]))) for s in secs)))
    N = int(2.2 * est) + 64
    Y = responses(A, C, N)
    real = np.hstack([realistic_deltas(run, fs, 2 * N, int(4 * fs), 48, f0, 50.0, hu, 11 + j)
                      for j, hu in enumerate((10.0, 50.0))])
    ra = actual(Y, real)
    print(f"  {fs:5.0f} SPS {f0:.0f} Hz Q12{' + harmonic' if len(secs) > 1 else '':11s} old {old:4d}  new {est:4d}"
          f"  realistic ring-down {ra:4d}  ({est / ra:.2f}x)")
    assert est >= ra
print("ALL CHECKS PASSED")
