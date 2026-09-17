"""
How long after a filter setting changes is the output clean again?

Runs the app's host chain on EEG-like data - electrode offsets, drift, mains,
alpha, noise - delivered in six-sample batches as over Bluetooth. At the
change, the live chain is rebuilt the way the app rebuilds it; a reference
chain has had the new settings from the start. Reported: the worst
difference after the change, and how long until it stays under 1 uV and
under 0.2 uV.
"""
import sys

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import eeg_dsp  # noqa: E402

FS = 250.0


def signal(seconds, seed=4):
    rng = np.random.default_rng(seed)
    n = int(seconds * FS)
    t = np.arange(n) / FS
    x = np.zeros((n, 8))
    for ch in range(8):
        walk = np.cumsum(rng.standard_normal(n)) * 6.0
        x[:, ch] = (rng.uniform(-60000, 60000)
                    + 2500 * np.sin(2 * np.pi * 0.07 * t + ch)
                    + walk
                    + 40 * np.sin(2 * np.pi * 49.7 * t)
                    + 15 * np.sin(2 * np.pi * 10 * t + ch)
                    + 4 * rng.standard_normal(n))
    return x


def configure(chain, **kw):
    for k, v in kw.items():
        setattr(chain, k, v)
    chain.rebuild()


def run(before, after, change_s, total_s):
    x = signal(total_s)
    k = int(change_s * FS) // 6 * 6

    live = eeg_dsp.Chain(FS, 8)
    live.notch_track = False
    configure(live, **before)
    ref = eeg_dsp.Chain(FS, 8)
    ref.notch_track = False
    configure(ref, **after)

    out_live, out_ref = [], []
    for i in range(0, len(x), 6):
        if i == k:
            configure(live, **after)
        out_live.append(live.process(x[i:i + 6]))
        out_ref.append(ref.process(x[i:i + 6]))
    a = np.vstack(out_live)[k:]
    b = np.vstack(out_ref)[k:]
    err = np.abs(a - b).max(axis=1)

    def settle(limit):
        over = np.flatnonzero(err > limit)
        return 0.0 if len(over) == 0 else (over[-1] + 1) / FS

    return err.max(), settle(1.0), settle(0.2)


def main():
    base = dict(highpass_hz=0.5, lowpass_hz=45.0, notch_hz=50.0, car=True)
    cases = [
        ("drift cut 0.5 -> 1 Hz", base, dict(base, highpass_hz=1.0), 20, 40),
        ("drift cut 1 -> 2 Hz", dict(base, highpass_hz=1.0), dict(base, highpass_hz=2.0), 20, 40),
        ("smooth 45 -> 100 Hz", base, dict(base, lowpass_hz=100.0), 20, 40),
        ("notch 50 -> off", base, dict(base, notch_hz=0.0), 20, 40),
        ("average on -> off", base, dict(base, car=False), 20, 40),
        ("drift cut 1 -> 0.1 Hz", dict(base, highpass_hz=1.0), dict(base, highpass_hz=0.1), 120, 180),
    ]
    print(f"{'change':<24}{'worst uV':>10}{'<1 uV after':>13}{'<0.2 uV after':>15}")
    for name, before, after, change_s, total_s in cases:
        worst, s1, s02 = run(before, after, change_s, total_s)
        print(f"{name:<24}{worst:>10.1f}{s1:>11.2f} s{s02:>13.2f} s")


if __name__ == "__main__":
    main()
