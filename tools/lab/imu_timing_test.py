"""
Where do motion timestamps go wrong at 480 Hz?

Sets the motion sensor directly, streams, and follows each frame's
first-sample time against a straight line fitted to the first two seconds.
Every jump over a millisecond is printed with the frames either side - size,
flags, sequence - so a one-off step can be told from a steady slope, and a
step of whole samples from anything else.
"""
import queue
import sys
import time

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_link as L  # noqa: E402


def drain(link, seconds):
    end = time.time() + seconds
    while time.time() < end:
        try:
            link.frames.get(timeout=0.05)
        except queue.Empty:
            pass


def run(link, hz, g, seconds):
    link.send(L.CMD_SET_IMU, 1, hz & 0xFF, hz >> 8, g, 2000 & 0xFF, 2000 >> 8)
    drain(link, 1.5)
    frames = []
    end = time.time() + seconds
    while time.time() < end:
        try:
            f = link.frames.get(timeout=0.2)
        except queue.Empty:
            continue
        if f.type == L.TYPE_IMU:
            d = L.decode_imu(f.payload)
            if d is not None:
                frames.append(d)

    if len(frames) < 20:
        print(f"{hz} Hz: only {len(frames)} frames")
        return
    ts = np.array([d[0] for d in frames], dtype=float)
    seq = np.array([d[1] for d in frames], dtype=float)
    period = np.array([d[2] for d in frames])
    sizes = np.array([len(d[3]) for d in frames])
    flags = np.array([d[6] for d in frames])
    gaps = int(np.sum(seq[1:] != seq[:-1] + sizes[:-1]))

    first = ts < ts[0] + 2e6
    fit = np.polyfit(seq[first], ts[first], 1)
    resid = ts - np.polyval(fit, seq)
    step = np.diff(resid)
    jumps = np.flatnonzero(np.abs(step) > 1000.0)

    print(f"--- {hz} Hz: {len(frames)} frames, {int(sizes.sum())} samples, {gaps} gaps, "
          f"flags seen 0x{int(np.bitwise_or.reduce(flags)):02x}, period "
          f"{period.min():.2f}..{period.max():.2f} us")
    print(f"    residual from the first 2 s: start {resid[0]:.1f} us, end {resid[-1]:.1f} us, "
          f"max |r| {np.abs(resid).max():.1f} us; {len(jumps)} jumps over 1 ms")
    for j in jumps[:12]:
        p = period[j + 1]
        print(f"    frame {j + 1} at +{(ts[j + 1] - ts[0]) / 1e6:6.2f} s: seq {int(seq[j + 1])}, "
              f"jump {step[j]:+.1f} us = {step[j] / p:+.2f} samples; sizes "
              f"{sizes[max(0, j - 2):j + 3].tolist()}, flags {flags[max(0, j - 2):j + 3].tolist()}")


def main():
    link = L.BleLink()
    started = time.time()
    while not link.connected and time.time() - started < 45:
        try:
            print("  status:", link.status.get(timeout=1))
        except queue.Empty:
            pass
    if not link.connected:
        print("could not connect over Bluetooth")
        link.close()
        return
    time.sleep(1.0)
    link.send(L.CMD_SET_ENCODING, L.ENC_RAW_I24)
    link.send(L.CMD_STREAM_START)
    drain(link, 1.0)

    run(link, 480, 4, 20)   # straight to 480 Hz
    run(link, 240, 8, 8)
    run(link, 480, 4, 20)   # 480 Hz after 240 Hz, as imu_hw_test does
    run(link, 960, 16, 8)
    run(link, 480, 4, 20)   # 480 Hz after 960 Hz

    link.send(L.CMD_SET_IMU, 1, 240, 0, 8, 2000 & 0xFF, 2000 >> 8)
    drain(link, 1.0)
    link.send(L.CMD_STREAM_STOP)
    drain(link, 0.5)
    link.close()
    time.sleep(0.5)


if __name__ == "__main__":
    main()
