"""Bad-electrode handling and recording timestamps, on the real app."""
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


class BadLink(FakeLink):
    """CH6 sits at 97 % of full scale and clips for part of every second."""

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
                counts = np.round(40 * np.sin(2 * np.pi * f * t) / lsb).astype(np.int64)
                push = 200000.0 * np.maximum(0.0, np.sin(2 * np.pi * t[:, 0]))
                ch6 = int((1 << 23) * 0.97) + np.round(push / lsb).astype(np.int64)
                counts[:, 5] = np.minimum(ch6, (1 << 23) - 1)
                c = (counts & 0xFFFFFF).astype(np.uint32)
                body = np.stack([c & 0xFF, (c >> 8) & 0xFF, (c >> 16) & 0xFF],
                                axis=-1).astype(np.uint8).tobytes()
                payload = struct.pack("<QIBBH", int(seq * 1e6 / self.rate), seq,
                                      8, link.ENC_RAW_I24, 6) + body
                self.frames.put(types.SimpleNamespace(type=link.TYPE_DATA,
                                                      payload=payload, flags=0))
                seq += 6


def main():
    os.chdir(SCRATCH)
    app = appmod.App()
    app.update()
    # Leaving the average needs the average on, with channels in it.
    app.car_var.set(True)
    for v in app.in_avg:
        v.set(True)
    app._rebuild_chain()
    app._set_car_mask()
    app.link = BadLink(250)
    app._was_connected = True
    app.started_at = time.time()

    seen = {"near": False, "clip": False, "colour_ok": True}
    real = app._render

    def spy(now):
        real(now)
        lay = app._lay
        if not lay:
            return
        r = lay["rows"].index(5)
        warn = app.canvas.itemcget(lay["warns"][r], "text")
        seen["near"] |= warn == "near limit"
        seen["clip"] |= warn == "CLIPPING"
        if app.canvas.itemcget(lay["traces"][r], "fill") != appmod.COLORS[5]:
            seen["colour_ok"] = False

    app._render = spy
    app.after(1500, app._toggle_record)
    app.after(3500, app._toggle_record)
    app.after(5000, app.quit)
    app.mainloop()
    del app._render

    print("CH6 warnings seen:", "near limit" if seen["near"] else "-", "/",
          "CLIPPING" if seen["clip"] else "-")
    print("CH6 trace kept its colour:", seen["colour_ok"])
    print("CH6 in average:", app.in_avg[5].get(), "| car_mask:",
          app.chain.car_mask.astype(int).tolist())
    print("others still in average:",
          all(app.in_avg[i].get() for i in range(8) if i != 5))
    print("status:", app.status.cget("text"))

    rec = sorted(p for p in os.listdir(SCRATCH)
                 if p.startswith("swifteeg_") and p.endswith(".csv")
                 and not p.endswith("_motion.csv"))[-1]
    motion_file = os.path.join(SCRATCH, rec[:-4] + "_motion.csv")
    if os.path.exists(motion_file):
        os.remove(motion_file)
    with open(os.path.join(SCRATCH, rec), newline="") as fh:
        rows = list(csv.reader(fh))
    print("header:", rows[0])
    data = np.array([[int(r[0]), int(r[1])] for r in rows[2:]], dtype=np.int64)
    dts, dseq = np.diff(data[:, 0]), np.diff(data[:, 1])
    print(f"recorded {len(data)} rows; ts step {dts.min()}..{dts.max()} us "
          f"(expect 4000); seq steps {sorted(set(dseq.tolist()))}")
    os.remove(os.path.join(SCRATCH, rec))

    app.link.close()
    app.pacer.close()
    app.destroy()


if __name__ == "__main__":
    main()
