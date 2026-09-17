"""Drive the real app with a fake Bluetooth link and measure the plot."""
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


class FakeLink:
    """Synthetic stream delivered in bursts, like a BLE connection interval."""

    def __init__(self, rate, burst_s=0.015):
        self.frames = queue.Queue()
        self.status = queue.Queue()
        self.connected = True
        self.bad_frames = 0
        self.rate = rate
        self.burst_s = burst_s
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, *args):
        pass

    def close(self):
        self._stop.set()

    def _run(self):
        seq = 0
        t0 = time.perf_counter()
        lsb = link.lsb_uv(24)
        f = np.arange(8, 16, dtype=float)
        while not self._stop.is_set():
            time.sleep(self.burst_s)
            due = int((time.perf_counter() - t0) * self.rate)
            while seq + 6 <= due:
                t = np.arange(seq, seq + 6)[:, None] / self.rate
                uv = 40 * np.sin(2 * np.pi * f * t) + 10 * np.sin(2 * np.pi * 50 * t)
                c = (np.round(uv / lsb).astype(np.int64) & 0xFFFFFF).astype(np.uint32)
                body = np.stack([c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF],
                                axis=-1).astype(np.uint8).tobytes()
                payload = struct.pack("<QIBBH", int(seq * 1e6 / self.rate), seq,
                                      8, link.ENC_RAW_I24, 6) + body
                self.frames.put(types.SimpleNamespace(type=link.TYPE_DATA,
                                                      payload=payload, flags=0))
                seq += 6


def test_ring(app):
    app.rate = 250
    app._reset_traces()
    rng = np.random.default_rng(1)
    total = 0
    for _ in range(400):
        n = int(rng.integers(1, 900)) if rng.random() > 0.03 else app.cap + 17
        app._ring_write(np.tile(np.arange(total, total + n, dtype=float), (8, 1)))
        total += n
        assert app.n_total == total
        oldest = max(0, total - app.cap)
        for _ in range(4):
            lo = int(rng.integers(oldest, total))
            hi = int(rng.integers(lo, min(total, lo + app.cap) + 1))
            got = app._ring_read(lo, hi)
            assert got.shape == (8, hi - lo), (got.shape, lo, hi)
            assert np.array_equal(got[3], np.arange(lo, hi)), (lo, hi)
    print("ring buffer: ok")


def run_case(app, rate, window, seconds=6.0, warm=2.0):
    if app.link:
        app.link.close()
    app.rate_var.set(str(rate))
    app.rate = rate
    app.chain.set_rate(rate)
    app.chain.reset()
    app._reset_traces()
    app.window_var.set(str(window))
    app.started_at = time.time()
    app._last_data_t = 0.0
    app.link = FakeLink(rate)
    app._was_connected = True

    stamps, disps = [], []
    real = app._render

    def spy(now):
        real(now)
        stamps.append(now)
        disps.append(app.disp)

    app._render = spy
    app.after(int(seconds * 1000), app.quit)
    app.mainloop()
    del app._render

    s, d = np.array(stamps), np.array(disps)
    keep = s >= s[0] + warm
    s, d = s[keep], d[keep]
    iv = np.diff(s) * 1000
    step = np.diff(d)
    lay = app._lay
    xy = app.canvas.coords(lay["traces"][0])
    xs, ys = xy[0::2], xy[1::2]
    period = 1000 / app.pacer.hz
    print(f"{rate:4d} SPS {window:2d} s: {len(s) / (s[-1] - s[0]):6.1f} fps | "
          f"frame ms p50 {np.median(iv):.2f} p95 {np.percentile(iv, 95):.2f} "
          f"max {iv.max():.2f} | missed {int(np.sum(iv > 1.5 * period))} | "
          f"scroll/frame {step.mean():.3f} sd {step.std():.3f} "
          f"(expect {rate / app.pacer.hz:.3f}) stalls {int(np.sum(step <= 0))} | "
          f"pts {len(xs)} x {min(xs):.0f}..{max(xs):.0f} "
          f"[{lay['left']}..{lay['left'] + lay['plot_w']}] "
          f"y {min(ys):.0f}..{max(ys):.0f} lane0 0..{lay['lane']:.0f} | "
          f"lag {app._latency * 1000:.0f} ms detail {app._pix_per_pt:.2f} | "
          f"gaps {app.gaps} samples {app.samples}")


def main():
    app = appmod.App()
    app.update()
    print("screen", app.pacer.hz, "Hz, canvas", app.canvas.winfo_width(), "x",
          app.canvas.winfo_height())
    test_ring(app)
    run_case(app, 250, 5)
    run_case(app, 1000, 10)
    run_case(app, 250, 1)
    run_case(app, 500, 5)
    app.scale_var.set("50")
    app.perch_var.set(False)
    run_case(app, 1000, 5)
    app.link.close()
    app.pacer.close()
    app.destroy()


if __name__ == "__main__":
    main()
