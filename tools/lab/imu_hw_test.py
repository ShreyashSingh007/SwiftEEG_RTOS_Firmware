"""IMU on the real board, over Bluetooth: config, rates, timing, alignment."""
import queue
import sys
import time

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_link as L  # noqa: E402


def collect(link, seconds):
    eeg, imu, rsp = [], [], []
    end = time.time() + seconds
    while time.time() < end:
        try:
            f = link.frames.get(timeout=0.2)
        except queue.Empty:
            continue
        if f.type == L.TYPE_DATA:
            d = L.decode_data(f.payload)
            if d is not None:
                eeg.append(d)
        elif f.type == L.TYPE_IMU:
            d = L.decode_imu(f.payload)
            if d is not None:
                imu.append(d)
        elif f.type == L.TYPE_RSP:
            rsp.append(bytes(f.payload))
    return eeg, imu, rsp


def show_config(rsp):
    for p in rsp:
        if p[0] == L.CMD_GET_CONFIG and p[1] == 0:
            sps = p[4] | (p[5] << 8)
            if len(p) >= 21:
                print(f"  config: {sps} SPS; IMU flags 0x{p[15]:02x}, "
                      f"{p[16] | (p[17] << 8)} Hz, +/-{p[18]} g, "
                      f"+/-{p[19] | (p[20] << 8)} dps")
            else:
                print(f"  config: {sps} SPS; no IMU fields ({len(p)} bytes)")
            return
    print("  config: no GET_CONFIG reply")


def analyse(eeg, imu, label):
    print(f"--- {label}")
    if eeg:
        gaps = sum(1 for a, b in zip(eeg, eeg[1:]) if b[1] != a[1] + len(a[3]))
        n = sum(len(d[3]) for d in eeg)
        span = (eeg[-1][0] - eeg[0][0]) / 1e6
        rate = (eeg[-1][1] - eeg[0][1]) / span if span > 0 else 0.0
        print(f"  EEG: {len(eeg)} frames, {n} samples, {gaps} gaps, {rate:.2f} SPS")
    else:
        print("  EEG: none")

    if not imu:
        print("  IMU: no frames")
        return

    gaps = 0
    flags = 0
    periods, scales, sizes = set(), set(), []
    t_all, s_all, acc_all, gyr_all = [], [], [], []
    for i, (ts, seq, period, counts, g, dps, fl) in enumerate(imu):
        n = len(counts)
        if i + 1 < len(imu) and imu[i + 1][1] != seq + n:
            gaps += 1
        flags |= fl
        periods.add(round(period, 2))
        scales.add((g, dps))
        sizes.append(n)
        t_all.append(ts + np.arange(n) * period)
        s_all.append(seq + np.arange(n))
        acc_all.append(counts[:, :3].astype(float) * g / 32768)
        gyr_all.append(counts[:, 3:].astype(float) * dps / 32768)

    t = np.concatenate(t_all)
    s = np.concatenate(s_all)
    acc = np.vstack(acc_all)
    gyr = np.vstack(gyr_all)
    fit = np.polyfit(s, t, 1)
    resid = t - np.polyval(fit, s)
    per = sorted(periods)
    hist = np.bincount(sizes)
    common = ", ".join(f"{k}x{hist[k]}" for k in np.argsort(hist)[::-1][:4] if hist[k])

    print(f"  IMU: {len(imu)} frames, {len(s)} samples (sizes {common}), "
          f"{gaps} gaps, flags seen 0x{flags:02x}")
    print(f"  IMU period field: {per[0]}..{per[-1]} us; scales {sorted(scales)}")
    print(f"  IMU timing: fitted {1e6 / fit[0]:.3f} Hz; residual from a straight line "
          f"rms {np.sqrt(np.mean(resid ** 2)):.1f} us, max {np.abs(resid).max():.1f} us")
    mag = np.linalg.norm(acc, axis=1)
    print(f"  accel |a| {mag.mean():.3f} +/- {mag.std():.4f} g; "
          f"gyro sd {np.round(gyr.std(axis=0), 2).tolist()} dps")
    if eeg:
        print(f"  clocks: EEG {eeg[0][0] / 1e6:.3f}..{eeg[-1][0] / 1e6:.3f} s, "
              f"IMU {t[0] / 1e6:.3f}..{t[-1] / 1e6:.3f} s")


def set_imu(link, hz, g, dps=2000):
    link.send(L.CMD_SET_IMU, 1, hz & 0xFF, hz >> 8, g, dps & 0xFF, dps >> 8)


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
    link.send(L.CMD_STREAM_START)
    collect(link, 1.0)

    for i, (hz, g) in enumerate([(240, 8), (480, 4), (960, 16)]):
        if i:
            set_imu(link, hz, g)
            collect(link, 1.5)
        link.send(L.CMD_GET_CONFIG)
        eeg, imu, rsp = collect(link, 8)
        show_config(rsp)
        analyse(eeg, imu, f"{hz} Hz, +/-{g} g")

    set_imu(link, 240, 8)
    collect(link, 1.5)
    link.send(L.CMD_STREAM_STOP)
    link.send(L.CMD_GET_CONFIG)
    _, _, rsp = collect(link, 2)
    show_config(rsp)
    link.close()
    time.sleep(0.5)


if __name__ == "__main__":
    main()
