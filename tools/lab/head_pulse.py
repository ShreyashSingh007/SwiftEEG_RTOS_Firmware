"""
The heartbeat the electrodes see, next to the one the IMU sees.

tools/vitals.py owns the maths and reads the pulse from head motion. This asks
the other half of the question: the same beat also moves the spring-steel
electrodes sitting over scalp arteries, so it should appear in the EEG itself.
Two things come out of that - a free check on the heart rate from a completely
separate sensor, and a number for how much of the delta band is heartbeat
rather than brain.

    python head_pulse.py DIR/quality.npz
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
TOOLS = r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools"
sys.path.insert(0, str(HERE))
sys.path.insert(0, TOOLS)
import vitals as V  # noqa: E402
import worn_report as R  # noqa: E402

RAIL = (1 << 23) - 64


def main() -> int:
    path = pathlib.Path(sys.argv[1])
    meta, d = R.load(path)
    m, e = R.imu_part(d), R.eeg_part(d)
    t = m["t"] - m["t"][0]
    fs = 1.0 / float(np.median(np.diff(t)))
    X = np.column_stack([m["accel"], m["gyro"]])
    p, f0, pulse, _ = V.pulse_component(X, fs)
    hr_imu = V.autocorr_hr(pulse, fs)[0]
    print(f"=== {path.name} ===")
    print(f"IMU says {hr_imu:.1f} bpm, {100 * p:.0f} % periodic")

    lsb = R.lsb_of(meta["gains"])
    raw = e["counts"] * lsb
    te = R.eeg_times(e)
    te = te - te[0]
    fe = 1.0 / float(np.median(np.diff(te)))
    live = np.flatnonzero(np.mean(np.abs(e["counts"]) >= RAIL, axis=0) < 0.01)

    print(f"\nthe same beat on the electrodes, raw counts at {fe:.1f} SPS")
    print("  ch    in band   periodic   its own rate    of delta 1-4 Hz")
    best = (-1.0, -1, None)
    for c in live:
        y = V.bandpass(raw[:, c], fe, *V.PULSE)
        pc, fc = V.periodicity(y, fe)
        # How much of the delta band is beat-locked: build the average beat,
        # lay it back down at every beat, and see how much variance that
        # reconstruction explains. This is average artifact subtraction, the
        # standard way the BCG artifact is removed from EEG in a scanner.
        dl = V.bandpass(raw[:, c], fe, 1.0, 4.0)
        idx, q, tpl = V.beats(y, fe, f0)
        share = 0.0
        if tpl is not None:
            half = len(tpl) // 2
            inside = idx[(idx >= half) & (idx + half < len(dl))]
            if len(inside) > 4:
                avg = np.array([dl[i - half:i + half] for i in inside]).mean(axis=0)
                recon = np.zeros_like(dl)
                for i in inside:
                    recon[i - half:i + half] += avg
                share = float(np.var(recon) / max(np.var(dl), 1e-30))
        print(f"  CH{c + 1} {y.std():9.2f} uV {100 * pc:8.0f} % {60 * fc:11.1f} bpm"
              f" {100 * min(share, 1.0):13.0f} %")
        if pc > best[0]:
            best = (pc, int(c), y)
    hr_eeg = V.autocorr_hr(best[2], fe)[0]
    print(f"  clearest is CH{best[1] + 1} at {hr_eeg:.1f} bpm - "
          f"{abs(hr_eeg - hr_imu):.1f} bpm from what the IMU says, on a sensor "
          f"that shares nothing with it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
