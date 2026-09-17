"""Motion lanes, IMU decoding and motion recording, on the real app."""
import csv
import os
import struct
import sys
import time
import types

import numpy as np

SCRATCH = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRATCH)
from app_fps_test import FakeLink, appmod, link  # noqa: E402


class MotionLink(FakeLink):
    """EEG as before, plus a 240 Hz motion stream on the same clock."""

    def _run(self):
        seq = 0
        imu_seq = 0
        t0 = time.perf_counter()
        lsb = link.lsb_uv(24)
        f = np.arange(8, 16, dtype=float)
        imu_hz = 240.0
        period_q8 = int(round(1e6 / imu_hz * 256))
        while not self._stop.is_set():
            time.sleep(self.burst_s)
            el = time.perf_counter() - t0

            due = int(el * self.rate)
            while seq + 6 <= due:
                t = np.arange(seq, seq + 6)[:, None] / self.rate
                uv = 40 * np.sin(2 * np.pi * f * t)
                c = (np.round(uv / lsb).astype(np.int64) & 0xFFFFFF).astype(np.uint32)
                body = np.stack([c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF],
                                axis=-1).astype(np.uint8).tobytes()
                payload = struct.pack("<QIBBH", int(seq * 1e6 / self.rate), seq,
                                      8, link.ENC_RAW_I24, 6) + body
                self.frames.put(types.SimpleNamespace(type=link.TYPE_DATA,
                                                      payload=payload, flags=0))
                seq += 6

            imu_due = int(el * imu_hz)
            while imu_seq + 12 <= imu_due:
                t = np.arange(imu_seq, imu_seq + 12) / imu_hz
                # 1 g on z, and a 2 Hz nod: 0.3 g on x, 50 deg/s on y.
                acc = np.stack([0.3 * np.sin(2 * np.pi * 2 * t), 0 * t, 1.0 + 0 * t], axis=1)
                gyr = np.stack([0 * t, 50 * np.cos(2 * np.pi * 2 * t), 0 * t], axis=1)
                counts = np.round(np.hstack([acc / 8 * 32768, gyr / 2000 * 32768])).astype("<i2")
                hdr = struct.pack("<QIIHHBBH", int(round(imu_seq * 1e6 / imu_hz)),
                                  imu_seq, period_q8, 8, 2000, 6, 0, 12)
                self.frames.put(types.SimpleNamespace(type=link.TYPE_IMU,
                                                      payload=hdr + counts.tobytes()))
                imu_seq += 12


def main():
    os.chdir(SCRATCH)
    errors = []
    real_print = appmod.traceback.print_exc

    def count_error(*a, **k):
        errors.append(1)
        real_print(*a, **k)

    appmod.traceback.print_exc = count_error

    app = appmod.App()
    app.update()
    app.imu_present = True
    app.link = MotionLink(250)
    app._was_connected = True
    app.started_at = time.time()

    stamps = []
    real = app._render

    def spy(now):
        real(now)
        stamps.append(now)

    app._render = spy
    app.after(1500, app._toggle_record)
    app.after(4500, app._toggle_record)
    app.after(6000, app.quit)
    app.mainloop()
    del app._render

    lay = app._lay
    print("motion lanes:", lay["motion"], "| lane height", round(lay["lane"]),
          "| plot x", lay["left"], "..", lay["left"] + lay["plot_w"])
    for lane, name in enumerate(("accel", "gyro")):
        top = lay["m_bases"][lane] - lay["lane"] / 2
        parts = []
        for axis in range(3):
            xy = app.canvas.coords(lay["m_lines"][3 * lane + axis])
            parts.append(f"{len(xy) // 2} pts x {min(xy[0::2]):.0f}..{max(xy[0::2]):.0f}"
                         f" y {min(xy[1::2]) - top:.0f}..{max(xy[1::2]) - top:.0f}")
        print(f"{name}: lane top {top:.0f}; " + " | ".join(parts)
              + "; label " + app.canvas.itemcget(lay["m_subs"][lane], "text"))
    print("stats:", app.stats.cget("text").splitlines()[3])
    s = np.array(stamps)
    s = s[s > s[0] + 2]
    print(f"fps {len(s) / (s[-1] - s[0]):.1f}, frame exceptions {len(errors)}")

    eeg = sorted(p for p in os.listdir(SCRATCH) if p.startswith("swifteeg_")
                 and p.endswith(".csv") and not p.endswith("_motion.csv"))[-1]
    mot = eeg[:-4] + "_motion.csv"
    with open(mot, newline="") as fh:
        rows = list(csv.reader(fh))
    print("motion header:", rows[0], rows[1])
    data = np.array([[float(x) for x in r] for r in rows[2:]])
    print("motion rows", len(data), "| ts steps",
          sorted(set(np.round(np.diff(data[:, 0])).astype(int).tolist())),
          "| seq steps", sorted(set(np.diff(data[:, 1]).astype(int).tolist())))
    print(f"ax {data[:, 2].min():.3f}..{data[:, 2].max():.3f} g, az mean {data[:, 4].mean():.3f} g,"
          f" gy peak {np.abs(data[:, 6]).max():.1f} deg/s")
    with open(eeg, newline="") as fh:
        erows = list(csv.reader(fh))
    ets = np.array([int(r[0]) for r in erows[2:]])
    print("EEG ts", ets[0], "..", ets[-1], "| motion ts", int(data[0, 0]), "..", int(data[-1, 0]))
    os.remove(eeg)
    os.remove(mot)

    app.link.close()
    app.pacer.close()
    app.destroy()


if __name__ == "__main__":
    main()
