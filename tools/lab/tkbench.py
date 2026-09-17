"""How fast can a Tk canvas redraw 8 EEG traces on this machine?"""
import ctypes
import time
import tkinter as tk

import numpy as np

winmm = ctypes.windll.winmm
winmm.timeBeginPeriod(1)
dwm = ctypes.windll.dwmapi
gdi = ctypes.windll.gdi32
user32 = ctypes.windll.user32

hdc = user32.GetDC(0)
print("primary display refresh:", gdi.GetDeviceCaps(hdc, 116), "Hz")
user32.ReleaseDC(0, hdc)

root = tk.Tk()
root.geometry("1500x900+60+40")
side = tk.Frame(root, width=348, bg="#1a1d25")
side.pack(side="left", fill="y")
c = tk.Canvas(root, bg="#0e1014", highlightthickness=0)
c.pack(side="left", fill="both", expand=True)
root.update()

W, H = c.winfo_width(), c.winfo_height()
print("canvas", W, "x", H)
CH = 8
left = 118
plot_w = W - left - 24
lane = H / CH
for s in range(1, 6):
    x = left + plot_w * s / 5
    c.create_line(x, 0, x, H, fill="#171a21")
items = []
for ch in range(CH):
    base = lane * (ch + 0.5)
    c.create_line(left, base, W - 24, base, fill="#1c2029")
    c.create_text(left - 10, base - 7, anchor="e", fill="#4ea1ff",
                  text=f"CH{ch + 1}", font=("Consolas", 10, "bold"))
    c.create_text(left - 10, base + 8, anchor="e", fill="#8a94a6",
                  text="+/-50uV", font=("Consolas", 8))
    items.append(c.create_line(0, 0, 1, 1, fill="#4ea1ff"))

rng = np.random.default_rng(0)
noise = rng.standard_normal((CH, 20000))
bases = (lane * (np.arange(CH) + 0.5))[:, None]


def render(P, t):
    xs = left + np.arange(P) * (plot_w / (P - 1))
    off = int(t * 1000) % 10000
    ys = bases + np.clip(noise[:, off:off + P] * lane * 0.15
                         + np.sin(xs * 0.01 + t * 3) * lane * 0.25,
                         -lane * 0.42, lane * 0.42)
    pts = np.empty((CH, 2 * P))
    pts[:, 0::2] = xs
    pts[:, 1::2] = ys
    call, w = c.tk.call, c._w
    for ch in range(CH):
        call(w, "coords", items[ch], pts[ch].tolist())
    root.update_idletasks()
    gdi.GdiFlush()


def bench(P, secs, paced):
    ts, work = [], []
    end = time.perf_counter() + secs
    while time.perf_counter() < end:
        if paced:
            dwm.DwmFlush()
        t0 = time.perf_counter()
        render(P, t0)
        work.append(time.perf_counter() - t0)
        ts.append(t0)
        root.update()
    iv = np.diff(ts) * 1000
    wk = np.array(work) * 1000
    print(f"P={P:5d} paced={str(paced):5} fps={len(ts) / secs:6.1f}  "
          f"work ms p50={np.median(wk):.2f} p95={np.percentile(wk, 95):.2f} "
          f"max={wk.max():.2f}  interval ms p50={np.median(iv):.2f} "
          f"p95={np.percentile(iv, 95):.2f} max={iv.max():.2f}")


for P in (600, 1200, 2400):
    bench(P, 2.0, False)
for P in (1200, 2400):
    bench(P, 3.0, True)
root.destroy()
winmm.timeEndPeriod(1)
