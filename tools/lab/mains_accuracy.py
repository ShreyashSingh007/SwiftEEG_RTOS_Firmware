"""
The mains tracker as built - tools/dsp_ref.MainsTracker, the firmware's
mirror, with its tenth-second average and two-in-a-row rule - on the
scenarios mains_track_study2.py measured the design on.
"""
import math
import sys

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import dsp_ref  # noqa: E402


def make(fs, seconds, mains_uv, f_fn, seed=2, floor=True):
    rng = np.random.default_rng(seed)
    n = int(seconds * fs)
    t = np.arange(n) / fs
    phase = 2 * np.pi * np.cumsum(f_fn(t)) / fs
    pink = np.cumsum(rng.standard_normal(n))
    pink -= np.convolve(pink, np.ones(int(fs)) / fs, mode="same")
    x = (8 * pink / pink.std()
         + (0.3 * math.sqrt(fs / 2) * rng.standard_normal(n) if floor else 0.0)
         + 10 * np.sin(2 * np.pi * 10 * t)
         + 300 * np.sin(2 * np.pi * 0.1 * t)
         + mains_uv * np.sin(phase))
    return x.astype(np.float32), f_fn(t)


def main():
    scenarios = [
        ("49.60 steady", lambda t: np.full_like(t, 49.60)),
        ("50 wander +/-0.05", lambda t: 50.0 + 0.05 * np.sin(2 * np.pi * t / 40)),
        ("step 49.90->50.08", lambda t: np.where(t < 45, 49.90, 50.08)),
        ("49.20 far off", lambda t: np.full_like(t, 49.20)),
    ]
    print(f"{'fs':>5}{'mains':>7}  {'scenario':<19}{'first':>9}{'err rms':>11}"
          f"{'max':>8}{'agreed':>8}")
    for fs in (250.0, 1000.0):
        for mains_uv in (1.0, 2.0, 5.0, 40.0):
            for name, f_fn in scenarios:
                x, f_true = make(fs, 90.0, mains_uv, f_fn)
                tr = dsp_ref.MainsTracker(fs, 50.0)
                first, errs, agreed = None, [], 0
                for i, v in enumerate(x):
                    if not tr.push(v):
                        continue
                    agreed += 1
                    if first is None:
                        first = i / fs
                    if i / fs > 20:
                        errs.append(float(tr.estimate) - f_true[max(0, i - int(2 * fs))])
                e = np.array(errs) if errs else np.array([np.nan])
                fl = f"{first:6.1f} s" if first is not None else "   never"
                print(f"{fs:>5.0f}{mains_uv:>5.0f}uV  {name:<19}{fl:>9}"
                      f"{np.sqrt(np.nanmean(e ** 2)) * 1000:>8.1f} mHz"
                      f"{np.nanmax(np.abs(e)) * 1000:>6.0f}{agreed:>8}", flush=True)
    for fs in (250.0, 1000.0):
        for floor in (True, False):
            for seed in (9, 10, 11):
                x, _ = make(fs, 90.0, 0.0, lambda t: np.full_like(t, 50.0), seed=seed,
                            floor=floor)
                tr = dsp_ref.MainsTracker(fs, 50.0)
                claims = sum(1 for v in x if tr.push(v))
                print(f"no mains, {fs:.0f} SPS, seed {seed}, "
                      f"{'with' if floor else 'without'} a noise floor: "
                      f"{claims} estimates in 90 s", flush=True)


if __name__ == "__main__":
    main()
