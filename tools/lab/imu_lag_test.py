"""
Does the motion lane keep up with the EEG on screen?

Drives the real app with EEG (6 samples a frame) and motion (batched as the
firmware batches them) on one simulated device clock, each frame delivered
after a Bluetooth-like delay. Every redraw records how far the newest motion
sample falls short of the plot's right edge. Anything over a few
milliseconds shows as the motion trace stopping short and then catching up.
"""
import heapq
import queue
import struct
import sys
import threading
import time
import types

import numpy as np

TOOLS = r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools"
sys.path.insert(0, TOOLS)
import swifteeg_app as appmod  # noqa: E402
import swifteeg_link as link  # noqa: E402


class DualLink:
    def __init__(self, eeg_rate, imu_rate, imu_batch, seed=1):
        self.frames = queue.Queue()
        self.status = queue.Queue()
        self.connected = True
        self.bad_frames = 0
        self.eeg_rate, self.imu_rate, self.imu_batch = eeg_rate, imu_rate, imu_batch
        self.rng = np.random.default_rng(seed)
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, *args):
        pass

    def close(self):
        self._stop.set()

    def _delay(self):
        # BLE: a connection event every 7.5 ms, plus the odd slow one.
        d = 0.0075 * (1 + int(self.rng.integers(0, 3)))
        return d + (0.03 if self.rng.random() < 0.05 else 0.0)

    def _run(self):
        t0 = time.perf_counter()
        pending = []
        eeg_n = imu_n = 0
        lsb = link.lsb_uv(24)
        while not self._stop.is_set():
            time.sleep(0.002)
            now = time.perf_counter() - t0
            while (eeg_n + 6) / self.eeg_rate <= now:
                t = np.arange(eeg_n, eeg_n + 6)[:, None] / self.eeg_rate
                uv = 30 * np.sin(2 * np.pi * 7 * t + np.arange(8))
                c = (np.round(uv / lsb).astype(np.int64) & 0xFFFFFF).astype(np.uint32)
                body = np.stack([c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF],
                                axis=-1).astype(np.uint8).tobytes()
                p = struct.pack("<QIBBH", int(eeg_n * 1e6 / self.eeg_rate), eeg_n,
                                8, link.ENC_RAW_I24, 6) + body
                heapq.heappush(pending, (now + self._delay(), id(p),
                                         types.SimpleNamespace(type=link.TYPE_DATA,
                                                               payload=p, flags=0)))
                eeg_n += 6
            b = self.imu_batch
            while (imu_n + b) / self.imu_rate <= now:
                period = 1e6 / self.imu_rate
                v = np.zeros((b, 6), dtype="<i2")
                v[:, 0] = (3000 * np.sin(2 * np.pi * 2 * np.arange(imu_n, imu_n + b)
                                         / self.imu_rate)).astype(np.int16)
                p = struct.pack("<QIIHHBBH", int(imu_n * period), imu_n,
                                int(period * 256), 8, 2000, 6, 0, b) + v.tobytes()
                heapq.heappush(pending, (now + self._delay(), id(p),
                                         types.SimpleNamespace(type=link.TYPE_IMU,
                                                               payload=p, flags=0)))
                imu_n += b
            while pending and pending[0][0] <= now:
                self.frames.put(heapq.heappop(pending)[2])


def measure(app, eeg_rate, imu_rate, imu_batch, seconds=8.0, warm=2.0):
    if app.link:
        app.link.close()
    app.rate_var.set(str(eeg_rate))
    app.rate = eeg_rate
    app.chain.set_rate(eeg_rate)
    app._reset_traces()
    app._reset_imu()
    app.imu_present = True
    app._last_data_t = 0.0
    app.link = DualLink(eeg_rate, imu_rate, imu_batch)
    app._was_connected = True

    shortfall = []
    start = time.perf_counter()
    real = app._draw_motion

    def spy(lay, window):
        real(lay, window)
        if time.perf_counter() - start < warm or app.eeg_anchor is None or app.imu_n < 2:
            return
        idx, ts = app.eeg_anchor
        t_right = ts + (app.disp - idx) * 1e6 / app.rate
        _, times = app._imu_recent(1)
        shortfall.append((t_right - times[-1]) / 1000.0)

    app._draw_motion = spy
    app.after(int(seconds * 1000), app.quit)
    app.mainloop()
    del app._draw_motion

    s = np.array(shortfall)
    late = s > 5.0
    print(f"EEG {eeg_rate:4d} SPS, motion {imu_rate} Hz x{imu_batch:2d}: "
          f"motion stops short of the edge on {100 * late.mean():4.1f} % of redraws, "
          f"by up to {s.max():5.1f} ms (display {app._latency * 1000:.0f} ms behind), "
          f"fps {app._fps:.0f}")


def main():
    app = appmod.App()
    app.update()
    batches = [int(a) for a in sys.argv[1:]] or [12]
    for b in batches:
        measure(app, 250, 240, b)
        measure(app, 1000, 240, b)
    app.link.close()
    app.pacer.close()
    app.destroy()


if __name__ == "__main__":
    main()
