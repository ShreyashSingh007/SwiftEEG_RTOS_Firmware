"""Same scenarios as before the fix, run through the fixed chain."""
import sys

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import eeg_dsp  # noqa: E402

fs = 250.0
n = int(fs * 60)
t = np.arange(n) / fs
f_mains = 49.85 + 0.1 * np.sin(2 * np.pi * t / 23.0)
phase = 2 * np.pi * np.cumsum(f_mains) / fs
alpha = 10.0 * np.sin(2 * np.pi * 10.0 * t)

base = np.zeros((n, 8))
for ch in range(8):
    base[:, ch] = (alpha + 20.0 * np.sin(phase) + (ch - 4) * 15000.0
                   + 30.0 * np.sin(2 * np.pi * 0.3 * t + ch))

walk = np.cumsum(np.random.default_rng(0).standard_normal(n)) * 20.0
floating = base.copy()
floating[:, 5] = 150000.0 + walk + 3000.0 * np.sin(phase)


def run(x, hp, retune, mask_bad=False):
    c = eeg_dsp.Chain(fs, 8)
    c.highpass_hz = hp
    c.rebuild()
    if mask_bad:
        c.car_mask[5] = False
    buf = np.zeros(0)
    nxt = 5.0
    moves = 0
    out = []
    for i in range(0, n, 7):
        blk = x[i:i + 7]
        buf = np.concatenate((buf, blk.mean(axis=1)))[-4000:]
        now = i / fs
        if retune and now >= nxt and len(buf) >= fs * 4:
            nxt = now + 5.0
            if c.update_mains(buf):
                moves += 1
        out.append(c.process(blk))
    return np.vstack(out), moves


def rms_peak(d):
    return np.sqrt(np.mean(d ** 2)), np.abs(d).max()


late = slice(int(20 * fs), n)
for hp in (0.1, 1.0):
    steady, _ = run(base, hp, False)
    live, moves = run(base, hp, True)
    r, p = rms_peak(live[late, 0] - steady[late, 0])
    print(f"HP {hp} Hz: {moves} re-aims -> extra error on CH1 rms {r:7.2f} uV, peak {p:7.2f} uV")

for hp in (0.1, 1.0):
    clean, _ = run(base, hp, False)
    dirty, _ = run(floating, hp, False)
    clean_m, _ = run(base, hp, False, mask_bad=True)
    dirty_m, _ = run(floating, hp, False, mask_bad=True)
    r1, p1 = rms_peak(dirty[late, 0] - clean[late, 0])
    r2, p2 = rms_peak(dirty_m[late, 0] - clean_m[late, 0])
    print(f"HP {hp} Hz: floating CH6 leaks into CH1 - in average rms {r1:6.1f} uV, "
          f"left out rms {r2:6.2f} uV (peak {p2:.2f})")

first = run(base, 0.1, False)[0][: int(fs * 2), 0]
print(f"HP 0.1 Hz start-up: largest CH1 value in the first 2 s {np.abs(first).max():.1f} uV")
