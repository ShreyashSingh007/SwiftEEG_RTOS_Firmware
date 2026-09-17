"""Where does a frame's time go at 500 and 1000 SPS?"""
import sys
import time

import numpy as np

sys.path.insert(0, r"C:\Users\shrey\AppData\Local\Temp\claude\D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware\005c4da0-af1b-4f26-97b9-4ee4c4c7cdce\scratchpad")
from app_fps_test import FakeLink, appmod  # noqa: E402  (runs nothing at import? see guard)
import eeg_dsp  # noqa: E402

app = appmod.App()
app.update()

comp = {k: [] for k in ("wait", "pump", "render", "idle")}


def timed(name, fn):
    def w(*a):
        t = time.perf_counter()
        r = fn(*a)
        comp[name].append(time.perf_counter() - t)
        return r
    return w


app._pump = timed("pump", app._pump)
app._render = timed("render", app._render)
app.update_idletasks = timed("idle", app.update_idletasks)
app.pacer.wait_for_refresh = timed("wait", app.pacer.wait_for_refresh)

dsp = []
_proc = app.chain.process


def proc(block):
    t = time.perf_counter()
    r = _proc(block)
    dsp.append((time.perf_counter() - t, block.shape[0]))
    return r


app.chain.process = proc

fm = []
_fm = eeg_dsp.find_mains


def find(*a, **k):
    t = time.perf_counter()
    r = _fm(*a, **k)
    fm.append(time.perf_counter() - t)
    return r


eeg_dsp.find_mains = find

frames = []
_frame = app._frame


def frame():
    n = {k: len(v) for k, v in comp.items()}
    t0 = time.perf_counter()
    _frame()
    rec = {k: (comp[k][-1] if len(comp[k]) > n[k] else 0.0) for k in comp}
    rec["total"] = time.perf_counter() - t0
    rec["t"] = t0
    frames.append(rec)


app._frame = frame


def case(rate, window, seconds=7.0):
    frames.clear()
    dsp.clear()
    fm.clear()
    if app.link:
        app.link.close()
    app.rate_var.set(str(rate))
    app.rate = rate
    app.chain.set_rate(rate)
    app._reset_traces()
    app.window_var.set(str(window))
    app._pix_per_pt = 1.0
    app.started_at = time.time()
    app._last_data_t = 0.0
    app.link = FakeLink(rate)
    app._was_connected = True
    app.after(int(seconds * 1000), app.quit)
    app.mainloop()

    fr = [f for f in frames if f["t"] > frames[0]["t"] + 2.0 and f["wait"] > 0]
    ms = lambda k: np.array([f[k] for f in fr]) * 1000  # noqa: E731
    work = ms("total") - ms("wait")
    print(f"\n=== {rate} SPS, {window} s window, {len(fr)} frames, "
          f"detail {app._pix_per_pt:.2f}")
    for k in ("pump", "render", "idle"):
        v = ms(k)
        print(f"  {k:6s} p50 {np.median(v):5.2f}  p95 {np.percentile(v, 95):5.2f}  "
              f"max {v.max():6.2f} ms")
    print(f"  work   p50 {np.median(work):5.2f}  p95 {np.percentile(work, 95):5.2f}  "
          f"max {work.max():6.2f} ms")
    if dsp:
        d = np.array(dsp)
        per = d[:, 0] / np.maximum(1, d[:, 1]) * 1e6
        print(f"  dsp    {np.median(per):.1f} us/sample, block p50 {np.median(d[:, 1]):.0f} "
              f"max {d[:, 1].max():.0f} samples, call max {d[:, 0].max() * 1000:.2f} ms")
    if fm:
        print(f"  find_mains calls {len(fm)} max {max(fm) * 1000:.2f} ms")
    order = np.argsort(work)[::-1][:6]
    for i in order:
        f = fr[i]
        print(f"  slow: work {work[i]:6.2f}  pump {f['pump'] * 1000:5.2f}  "
              f"render {f['render'] * 1000:5.2f}  idle {f['idle'] * 1000:5.2f}")


case(500, 5)
case(1000, 5)
app.link.close()
app.pacer.close()
app.destroy()
