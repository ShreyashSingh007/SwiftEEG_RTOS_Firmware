"""
SwiftEEG - Windows application.

Connects over Bluetooth or USB, gives every device setting a control, runs a
filter chain on the host, plots the result, and records raw data to disk.

    python tools/swifteeg_app.py

Needs pyserial for USB and bleak for Bluetooth. The window is tkinter, which
ships with Python and is native on Windows.

Three decisions worth knowing:

  * The device streams raw counts. Filtering happens here, where a setting
    can be changed and judged in a second. Recordings are raw for the same
    reason - a session can be re-analysed later with different settings,
    which is impossible once a filter has been baked in.

  * The filters are built from the same biquad forms the firmware uses, so a
    chain that works here transfers to the device as coefficients rather than
    as a rewrite. That is what makes moving the DSP on-chip a configuration
    step and not a second implementation.

  * The plot is paced by the monitor, not by the data. Bluetooth delivers
    samples in bursts, and drawing each burst as it lands makes the trace
    jump. Instead the display trails the newest sample by a few tens of
    milliseconds and scrolls at a steady speed, one redraw per refresh.
"""

from __future__ import annotations

import collections
import csv
import ctypes
import math
import pathlib
import queue
import sys
import time
import tkinter as tk
import traceback
from tkinter import ttk

import numpy as np

sys.path.insert(0, pathlib.Path(__file__).resolve().parent.as_posix())
import eeg_dsp  # noqa: E402
import swifteeg_link as link  # noqa: E402

SITES = ["C4", "P4", "F4", "Oz", "AFz", "F3", "C3", "P3"]
COLORS = ["#4ea1ff", "#ffb454", "#5ed18b", "#ff6b6b",
          "#c792ea", "#ffd866", "#78dce8", "#ff9ec4"]

BG = "#12141a"
PANEL = "#1a1d25"
FG = "#e6e6e6"
DIM = "#8a94a6"
PLOT_BG = "#0e1014"

# History kept for the plot whatever window is showing, so widening the
# window reveals data already received rather than starting empty.
MAX_WINDOW_SECONDS = 10.0

# How far the display may trail the newest sample. It has to trail by more
# than the longest gap between Bluetooth deliveries, or the trace stalls
# waiting for the next one.
LATENCY_MIN_S = 0.05
LATENCY_MAX_S = 0.30

# Converter limits. Output codes stop at +/-2^23, so a clipped sample sits
# right on them.
CLIP_COUNTS = (1 << 23) - 64
# Within 5 % of the limit - about 9 mV of room at gain 24. Still real data,
# but not much electrode drift away from clipping.
NEAR_COUNTS = int((1 << 23) * 0.95)
# How long a warning stays up after the sample that raised it, so it can be
# read instead of flickering with each Bluetooth burst.
LIMIT_HOLD_S = 1.0
# How long a channel sits near its limit before it leaves the average.
AUTO_OUT_S = 2.0


class FramePacer:
    """
    Paces redraws to the monitor's refresh.

    tkinter has no vsync, and on Windows `after(7)` does not give 144 Hz:
    timers round up to the 15.6 ms system tick, which caps a plot near 64
    fps, and the frames that do land drift against the monitor and stutter.
    So the timer resolution is raised to 1 ms, and each frame starts by
    blocking in DwmFlush, which returns when the compositor presents - one
    redraw per refresh, in step with the screen.
    """

    VREFRESH = 116  # GetDeviceCaps index

    def __init__(self) -> None:
        self.hz = 60.0
        self._dwm = None
        self._gdi = None
        self._winmm = None
        self._last = time.perf_counter()

        if sys.platform != "win32":
            return

        try:
            self._winmm = ctypes.windll.winmm
            self._winmm.timeBeginPeriod(1)
        except (OSError, AttributeError):
            self._winmm = None

        try:
            user32 = ctypes.windll.user32
            self._gdi = ctypes.windll.gdi32
            self._dwm = ctypes.windll.dwmapi
            hdc = user32.GetDC(0)
            hz = self._gdi.GetDeviceCaps(hdc, self.VREFRESH)
            user32.ReleaseDC(0, hdc)
            if hz > 1:
                self.hz = float(hz)
        except (OSError, AttributeError):
            self._gdi = None
            self._dwm = None

    def wait_for_refresh(self) -> None:
        """Block until the next screen refresh. Call before drawing."""
        period = 1.0 / self.hz
        if self._dwm is not None:
            self._dwm.DwmFlush()

        # DwmFlush returns at once when nothing is being composed - a locked
        # screen, say. A timed wait stands in then, rather than a busy loop.
        since = time.perf_counter() - self._last
        if since < period * 0.6:
            time.sleep(period - since)
        self._last = time.perf_counter()

    def flush(self) -> None:
        """Push batched GDI drawing out now, so it makes this refresh."""
        if self._gdi is not None:
            self._gdi.GdiFlush()

    def close(self) -> None:
        if self._winmm is not None:
            self._winmm.timeEndPeriod(1)
            self._winmm = None


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("SwiftEEG")
        self.geometry("1500x900")
        self.configure(bg=BG)

        self.link: link.Link | None = None
        self._was_connected = False
        self.rate = 250
        self.chain = eeg_dsp.Chain(self.rate, link.CHANNELS)
        self.gain = 24

        self._reset_traces()
        self.enabled = [tk.BooleanVar(value=True) for _ in range(link.CHANNELS)]
        # Whether each channel helps make the common average - see
        # Chain.car_mask for why a bad electrode must not.
        self.in_avg = [tk.BooleanVar(value=True) for _ in range(link.CHANNELS)]

        # Input limits, per channel. At gain 24 the input range is only
        # +/-187.5 mV, and an electrode offset near that leaves no room.
        # Kept as times rather than flags, so a warning holds long enough to
        # be read.
        self.clip_t = np.full(link.CHANNELS, -np.inf)
        self.near_t = np.full(link.CHANNELS, -np.inf)
        self.near_since = np.full(link.CHANNELS, np.nan)
        self.auto_out = [False] * link.CHANNELS
        self.dc_mv = [0.0] * link.CHANNELS

        # Recent unfiltered samples, kept only so the mains frequency can be
        # measured. Grid frequency is never exactly 50 or 60 Hz - measured
        # 49.6 here - and a notch narrow enough to spare the EEG is too
        # narrow to hit by guesswork.
        self.mains_buf: collections.deque = collections.deque(maxlen=4000)
        self.mains_next = 0.0

        self.samples = 0
        self.frames = 0
        self.last_seq: int | None = None
        self.gaps = 0
        self.leadoff = 0
        self.started_at = 0.0
        self.recorder: csv.writer | None = None
        self._rec_file = None
        self.rec_rows = 0

        # Display state.
        self.pacer = FramePacer()
        self._lay: dict | None = None
        self._item_cfg: dict = {}
        self._span = np.full(link.CHANNELS, 50.0)
        self._active = False
        self._clock_t = time.perf_counter()
        self._frame_dt = 0.0
        self._last_data_t = 0.0
        self._gap_peak = 0.0
        self._latency = 0.1
        self._work = 0.0
        self._pix_per_pt = 1.0
        self._stats_next = 0.0
        self._fps = 0.0
        self._fps_count = 0
        self._fps_t = time.perf_counter()
        self._error_next = 0.0

        self._build()
        self.after(1, self._frame)
        self.protocol("WM_DELETE_WINDOW", self._close)

    # ---------------------------------------------------------------- UI --

    def _label(self, parent, text, **kw):
        return tk.Label(parent, text=text, bg=kw.pop("bg", PANEL),
                        fg=kw.pop("fg", FG), anchor="w",
                        font=kw.pop("font", ("Segoe UI", 9)), **kw)

    def _section(self, parent, title):
        tk.Frame(parent, bg="#2a2f3a", height=1).pack(fill=tk.X, pady=(12, 0))
        self._label(parent, title.upper(), fg=DIM,
                    font=("Segoe UI", 8, "bold")).pack(fill=tk.X, pady=(6, 4))
        f = tk.Frame(parent, bg=PANEL)
        f.pack(fill=tk.X)
        return f

    def _build(self) -> None:
        """
        The control column scrolls.

        There are more controls than fit a laptop screen, and a fixed frame
        silently clips whatever falls off the bottom - which is how the
        statistics line and the scale controls became invisible.
        """
        side = tk.Frame(self, bg=PANEL, width=348)
        side.pack(side=tk.LEFT, fill=tk.Y)
        side.pack_propagate(False)

        canvas = tk.Canvas(side, bg=PANEL, highlightthickness=0, width=330)
        bar = ttk.Scrollbar(side, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)

        bar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        inner = tk.Frame(canvas, bg=PANEL, padx=14, pady=12)
        window = canvas.create_window((0, 0), window=inner, anchor="nw",
                                      width=326)

        def on_resize(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))

        inner.bind("<Configure>", on_resize)

        def on_wheel(event):
            canvas.yview_scroll(int(-event.delta / 120), "units")

        # Bind to the whole window: the pointer is usually over a child
        # widget, not the canvas, and children do not forward the event.
        self.bind_all("<MouseWheel>", lambda e: on_wheel(e)
                      if self._pointer_over(side) else None)

        self._side = side

        # -- connection --
        f = self._section(inner, "connection")
        self.transport = tk.StringVar(value="Bluetooth")
        row = tk.Frame(f, bg=PANEL)
        row.pack(fill=tk.X)
        ttk.Combobox(row, textvariable=self.transport, width=11,
                     state="readonly",
                     values=["Bluetooth", "USB"]).pack(side=tk.LEFT)
        self.btn_conn = tk.Button(row, text="Connect", width=10,
                                  command=self._toggle_conn)
        self.btn_conn.pack(side=tk.LEFT, padx=6)

        # Always available, even when the app thinks it is streaming: if the
        # link wedges there has to be a way back that does not involve
        # restarting the application.
        tk.Button(row, text="Reset", width=6,
                  command=self._reset_link).pack(side=tk.LEFT)

        self.status = self._label(f, "not connected", fg=DIM)
        self.status.pack(fill=tk.X, pady=(6, 0))

        # -- acquisition --
        f = self._section(inner, "acquisition")
        row = tk.Frame(f, bg=PANEL)
        row.pack(fill=tk.X)
        self._label(row, "rate").pack(side=tk.LEFT)
        self.rate_var = tk.StringVar(value="250")
        rc = ttk.Combobox(row, textvariable=self.rate_var, width=7,
                          state="readonly", values=["250", "500", "1000"])
        rc.pack(side=tk.LEFT, padx=6)
        rc.bind("<<ComboboxSelected>>", lambda e: self._set_rate())

        self._label(row, "  format").pack(side=tk.LEFT)
        # "device filtered" is what makes the on-device DSP visible. "raw"
        # sends counts straight off the converter, before the device touches
        # them - so with it selected, the on-device notch correctly appears
        # to do nothing. There is no 32-bit raw here: the converter is
        # 24-bit, and a fourth byte would only repeat the sign.
        self.enc_var = tk.StringVar(value="raw 24-bit")
        ec = ttk.Combobox(row, textvariable=self.enc_var, width=14,
                          state="readonly",
                          values=["raw 24-bit", "device filtered"])
        ec.pack(side=tk.LEFT, padx=6)
        ec.bind("<<ComboboxSelected>>", lambda e: self._set_encoding())

        row = tk.Frame(f, bg=PANEL)
        row.pack(fill=tk.X, pady=(6, 0))
        self.btn_stream = tk.Button(row, text="Start streaming", width=16,
                                    command=self._toggle_stream)
        self.btn_stream.pack(side=tk.LEFT)
        self.btn_rec = tk.Button(row, text="Record", width=10,
                                 command=self._toggle_record)
        self.btn_rec.pack(side=tk.LEFT, padx=6)

        # -- input --
        f = self._section(inner, "input")
        row = tk.Frame(f, bg=PANEL)
        row.pack(fill=tk.X)
        self._label(row, "source").pack(side=tk.LEFT)
        self.src_var = tk.StringVar(value="Electrodes")
        sc = ttk.Combobox(row, textvariable=self.src_var, width=13,
                          state="readonly",
                          values=["Electrodes", "Shorted (noise)",
                                  "Test signal"])
        sc.pack(side=tk.LEFT, padx=6)
        sc.bind("<<ComboboxSelected>>", lambda e: self._set_source())

        row = tk.Frame(f, bg=PANEL)
        row.pack(fill=tk.X, pady=(6, 0))
        self._label(row, "gain").pack(side=tk.LEFT)
        self.gain_var = tk.StringVar(value="24")
        gc = ttk.Combobox(row, textvariable=self.gain_var, width=7,
                          state="readonly",
                          values=["1", "2", "4", "6", "8", "12", "24"])
        gc.pack(side=tk.LEFT, padx=6)
        gc.bind("<<ComboboxSelected>>", lambda e: self._set_gain())

        # -- reference and electrodes --
        f = self._section(inner, "reference")
        self.bias_var = tk.BooleanVar(value=True)
        self.loff_var = tk.BooleanVar(value=False)
        self._check(f, "Bias drive (left mastoid)", self.bias_var,
                    self._set_bias)
        self._check(f, "Lead-off detection", self.loff_var, self._set_leadoff)
        self._label(f, "SRB1 reference: right mastoid", fg=DIM,
                    font=("Segoe UI", 8)).pack(fill=tk.X, pady=(4, 0))

        # -- device notch --
        f = self._section(inner, "on-device filter")
        row = tk.Frame(f, bg=PANEL)
        row.pack(fill=tk.X)
        self._label(row, "mains notch").pack(side=tk.LEFT)
        self.dev_notch = tk.StringVar(value="50 Hz")
        nc = ttk.Combobox(row, textvariable=self.dev_notch, width=8,
                          state="readonly", values=["off", "50 Hz", "60 Hz"])
        nc.pack(side=tk.LEFT, padx=6)
        nc.bind("<<ComboboxSelected>>", lambda e: self._set_dev_notch())

        # -- host filters --
        f = self._section(inner, "host filters (display)")
        self.hp_var = tk.StringVar(value="0.5")
        self.lp_var = tk.StringVar(value="45")
        self.hnotch_var = tk.StringVar(value="50 Hz")
        self._combo_row(f, "drift cut", self.hp_var,
                        ["off", "0.1", "0.3", "0.5", "1.0", "2.0"],
                        self._rebuild_chain, "Hz")
        self._combo_row(f, "smooth above", self.lp_var,
                        ["off", "30", "45", "70", "100"],
                        self._rebuild_chain, "Hz")
        self._combo_row(f, "mains notch", self.hnotch_var,
                        ["off", "50 Hz", "60 Hz"], self._rebuild_chain, "")

        self.q_var = tk.StringVar(value="12")
        self._combo_row(f, "notch width", self.q_var,
                        ["30 (narrow)", "12", "6 (wide)"],
                        self._rebuild_chain, "")

        self.car_var = tk.BooleanVar(value=True)
        self.harm_var = tk.BooleanVar(value=True)
        self.track_var = tk.BooleanVar(value=True)
        self._check(f, "Common average (shared zero)", self.car_var,
                    self._rebuild_chain)
        self._check(f, "Also notch the harmonic", self.harm_var,
                    self._rebuild_chain)
        self._check(f, "Track real mains frequency", self.track_var,
                    self._rebuild_chain)

        self.mains_label = self._label(f, "", fg="#ffd866",
                                       font=("Consolas", 8))
        self.mains_label.pack(fill=tk.X, pady=(2, 0))

        # -- channels --
        f = self._section(inner, "channels")
        # Two ticks per channel: whether it is drawn, and whether it helps
        # make the common average. A bad electrode can stay on screen while
        # being kept out of every other channel's reference.
        for i in range(link.CHANNELS):
            row = tk.Frame(f, bg=PANEL)
            row.pack(fill=tk.X)
            cb = tk.Checkbutton(row, variable=self.enabled[i], bg=PANEL,
                                fg=COLORS[i], selectcolor="#2a2f3a",
                                activebackground=PANEL, highlightthickness=0,
                                text=f"CH{i + 1}  {SITES[i]}",
                                font=("Consolas", 9), anchor="w", width=14)
            cb.pack(side=tk.LEFT)
            tk.Checkbutton(row, variable=self.in_avg[i], bg=PANEL, fg=DIM,
                           selectcolor="#2a2f3a", activebackground=PANEL,
                           activeforeground=FG, highlightthickness=0,
                           text="in average", font=("Segoe UI", 8),
                           command=self._set_car_mask).pack(side=tk.LEFT)
            self.__dict__[f"loff{i}"] = self._label(
                row, "", fg=DIM, font=("Consolas", 8))
            self.__dict__[f"loff{i}"].pack(side=tk.LEFT)

        # -- display --
        f = self._section(inner, "display")

        # "auto" is a value in the same list rather than a separate
        # checkbox. The two-control version was ambiguous: picking a range
        # while auto was still ticked did nothing visible, which reads as a
        # broken control.
        self.scale_var = tk.StringVar(value="auto")
        self._combo_row(f, "range +/-", self.scale_var,
                        ["auto", "10", "25", "50", "100", "200", "500",
                         "2000", "200000"], None, "uV")

        self.window_var = tk.StringVar(value="5")
        self._combo_row(f, "time window", self.window_var,
                        ["1", "2", "3", "5", "10"], None, "s")

        self.perch_var = tk.BooleanVar(value=True)
        self._check(f, "Scale each channel separately", self.perch_var, None)

        self.stats = self._label(inner, "", fg=DIM, font=("Consolas", 8),
                                 justify=tk.LEFT)
        self.stats.pack(fill=tk.X, pady=(14, 0))

        self.canvas = tk.Canvas(self, bg=PLOT_BG, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def _pointer_over(self, widget) -> bool:
        try:
            x, y = self.winfo_pointerxy()
            wx = widget.winfo_rootx()
            return wx <= x < wx + widget.winfo_width()
        except tk.TclError:
            return False

    def _check(self, parent, text, var, cb):
        c = tk.Checkbutton(parent, text=text, variable=var, bg=PANEL, fg=FG,
                           selectcolor="#2a2f3a", activebackground=PANEL,
                           activeforeground=FG, highlightthickness=0,
                           anchor="w", font=("Segoe UI", 9),
                           command=(cb if cb else None))
        c.pack(fill=tk.X)
        return c

    def _combo_row(self, parent, label, var, values, cb, unit):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill=tk.X, pady=1)
        self._label(row, label, width=13).pack(side=tk.LEFT)
        c = ttk.Combobox(row, textvariable=var, width=7, state="readonly",
                         values=values)
        c.pack(side=tk.LEFT, padx=4)
        if cb:
            c.bind("<<ComboboxSelected>>", lambda e: cb())
        if unit:
            self._label(row, unit, fg=DIM).pack(side=tk.LEFT)
        return c

    # ----------------------------------------------------------- device --

    def _send(self, opcode: int, *args: int) -> None:
        if self.link and self.link.connected:
            self.link.send(opcode, *args)

    def _toggle_conn(self) -> None:
        if self.link:
            self.link.close()
            self.link = None
            self.btn_conn.config(text="Connect")
            self.status.config(text="not connected", fg=DIM)
            return

        try:
            if self.transport.get() == "USB":
                self.link = link.UsbLink()
            else:
                self.link = link.BleLink()
        except Exception as exc:  # noqa: BLE001 - shown to the user
            self.status.config(text=str(exc), fg="#ff6b6b")
            return

        self.btn_conn.config(text="Disconnect")
        self.status.config(text="connecting...", fg="#ffd866")
        self._was_connected = False

    def _toggle_stream(self) -> None:
        if self.btn_stream["text"].startswith("Start"):
            self._reset_traces()
            self.samples = 0
            self.frames = 0
            self.last_seq = None
            self.gaps = 0
            self.chain.reset()
            self.started_at = time.time()
            self._send(link.CMD_STREAM_START)
            self.btn_stream.config(text="Stop streaming")
        else:
            self._send(link.CMD_STREAM_STOP)
            self.btn_stream.config(text="Start streaming")

    def _reset_traces(self) -> None:
        """
        Processed samples for the plot, in a ring. Sample n of the stream
        lives at n % cap, so the display addresses samples by their number
        in the stream and can scroll smoothly between them.
        """
        self.cap = int((MAX_WINDOW_SECONDS + 1.0) * self.rate)
        self.ring = np.zeros((link.CHANNELS, self.cap))
        self.n_total = 0
        self.disp = 0.0

    def _ring_write(self, block: np.ndarray) -> None:
        """block is (channels, samples)."""
        n = block.shape[1]
        if n >= self.cap:
            self.n_total += n - self.cap
            block = block[:, n - self.cap:]
            n = self.cap

        i = self.n_total % self.cap
        first = min(n, self.cap - i)
        self.ring[:, i:i + first] = block[:, :first]
        if first < n:
            self.ring[:, :n - first] = block[:, first:]
        self.n_total += n

    def _ring_read(self, a: int, b: int) -> np.ndarray:
        """Samples a to b-1 of the stream, as (channels, b - a)."""
        n = b - a
        i = a % self.cap
        if i + n <= self.cap:
            return self.ring[:, i:i + n]
        return np.concatenate(
            (self.ring[:, i:], self.ring[:, :n - (self.cap - i)]), axis=1)

    def _set_rate(self) -> None:
        """
        Changing rate restarts acquisition on the device, which takes about a
        second. Everything held here belongs to the old rate: the filter
        coefficients, the samples already plotted, and the sequence counter.
        Carrying any of it across is what made a rate change look like the
        stream had died.
        """
        was_streaming = self.btn_stream["text"].startswith("Stop")

        if was_streaming:
            self._send(link.CMD_STREAM_STOP)

        self.rate = int(self.rate_var.get())
        self.chain.set_rate(self.rate)
        self.chain.reset()
        self._reset_traces()
        self.last_seq = None
        self.samples = 0
        self.frames = 0
        self.started_at = time.time()

        self._send(link.CMD_SET_RATE, self.rate & 0xFF, self.rate >> 8)
        self.status.config(text=f"switching to {self.rate} SPS...",
                           fg="#ffd866")

        if was_streaming:
            # The device is stopping a thread and reconfiguring the AFE.
            # Asking it to stream again before that finishes is ignored.
            self.after(2000, self._resume_after_rate)

    def _resume_after_rate(self) -> None:
        self._send(link.CMD_STREAM_START)
        self._send(link.CMD_GET_CONFIG)
        self.started_at = time.time()
        self.status.config(text=f"streaming at {self.rate} SPS", fg="#5ed18b")

    def _reset_link(self) -> None:
        """Stop everything and reconnect, without restarting the app."""
        if self.link:
            try:
                self.link.send(link.CMD_STREAM_STOP)
            except Exception:  # noqa: BLE001
                pass
            self.link.close()
            self.link = None

        self.btn_conn.config(text="Connect")
        self.btn_stream.config(text="Start streaming")
        self._was_connected = False
        self.chain.reset()
        self._reset_traces()
        self.last_seq = None
        self.status.config(text="link reset - press Connect", fg="#ffd866")

    def _set_encoding(self) -> None:
        pick = self.enc_var.get()
        enc = {"raw 24-bit": link.ENC_RAW_I24,
               "device filtered": link.ENC_UV_F32}.get(pick, link.ENC_RAW_I24)
        self._send(link.CMD_SET_ENCODING, enc)
        self.chain.reset()

    def _set_source(self) -> None:
        mux = {"Electrodes": link.MUX_NORMAL,
               "Shorted (noise)": link.MUX_SHORTED,
               "Test signal": link.MUX_TEST}[self.src_var.get()]
        self._send(link.CMD_SET_INPUT, mux, 0)
        self.chain.reset()

    def _set_gain(self) -> None:
        self.gain = int(self.gain_var.get())
        mux = {"Electrodes": link.MUX_NORMAL,
               "Shorted (noise)": link.MUX_SHORTED,
               "Test signal": link.MUX_TEST}[self.src_var.get()]
        self._send(link.CMD_SET_CHANNEL, 0xFF, link.GAIN_CODES[self.gain],
                   mux, 0, 0)

    def _set_bias(self) -> None:
        # Both masks: SRB1 sits on the negative inputs and has to be inside
        # the loop, or the bias amplifier oscillates.
        self._send(link.CMD_SET_BIAS, 1 if self.bias_var.get() else 0,
                   0xFF, 0xFF)

    def _set_leadoff(self) -> None:
        self._send(link.CMD_SET_LEADOFF, 1 if self.loff_var.get() else 0,
                   0xFF, 0x00)

    def _set_dev_notch(self) -> None:
        hz = {"off": 0, "50 Hz": 50, "60 Hz": 60}[self.dev_notch.get()]
        self._send(link.CMD_SET_NOTCH, hz)

    def _rebuild_chain(self) -> None:
        hp = self.hp_var.get()
        lp = self.lp_var.get()
        nz = self.hnotch_var.get()

        self.chain.highpass_hz = 0.0 if hp == "off" else float(hp)
        self.chain.lowpass_hz = 0.0 if lp == "off" else float(lp)
        self.chain.notch_hz = {"off": 0.0, "50 Hz": 50.0, "60 Hz": 60.0}[nz]
        self.chain.notch_harmonic = self.harm_var.get()
        self.chain.car = self.car_var.get()
        self.chain.notch_q = float(self.q_var.get().split()[0])
        self.chain.notch_track = self.track_var.get()
        self.chain.rebuild()

    def _set_car_mask(self) -> None:
        self.chain.car_mask = np.array([v.get() for v in self.in_avg],
                                       dtype=bool)

    # ---------------------------------------------------------- recording --

    def _toggle_record(self) -> None:
        if self.recorder is not None:
            self._rec_file.close()
            self.recorder = None
            self._rec_file = None
            self.btn_rec.config(text="Record")
            return

        name = time.strftime("swifteeg_%Y%m%d_%H%M%S.csv")
        path = pathlib.Path.cwd() / name
        self._rec_file = open(path, "w", newline="", encoding="utf-8")
        self.recorder = csv.writer(self._rec_file)

        # Raw counts, not microvolts: the scale depends on the gain, and a
        # recording that has already been filtered cannot be un-filtered.
        what = ("device filtered uV" if self.enc_var.get() == "device filtered"
                else "raw counts")
        self.recorder.writerow(
            [f"# SwiftEEG {what}", f"rate={self.rate}", f"gain={self.gain}",
             f"lsb_uv={link.lsb_uv(self.gain):.9f}"])
        self.recorder.writerow(
            ["ts_us", "seq"] + [f"ch{i + 1}_{SITES[i]}"
                                for i in range(link.CHANNELS)])
        self.rec_rows = 0
        self.btn_rec.config(text="Stop rec")
        self.status.config(text=f"recording {name}", fg="#5ed18b")

    # -------------------------------------------------------------- loop --

    def _frame(self) -> None:
        """
        One redraw. While samples are arriving this runs once per screen
        refresh; with nothing arriving it idles at 20 a second.
        """
        now = time.perf_counter()
        self._active = now - self._last_data_t < 0.6

        try:
            if self._active:
                self.pacer.wait_for_refresh()
                now = time.perf_counter()

            self._pump(now)
            self._render(now)
            self.update_idletasks()
            self.pacer.flush()

            if self._active:
                self._adapt_detail(time.perf_counter() - now)
        except Exception:  # noqa: BLE001 - one bad frame must not stop the plot
            if now >= self._error_next:
                self._error_next = now + 2.0
                traceback.print_exc()
        finally:
            # 0, not 1: a 1 ms timer really waits 1-2 ms, a fifth of a 144 Hz
            # frame, and frames were missing their refresh for it. Tk still
            # handles pending input between frames, and the wait for the
            # refresh is at the top of the next one.
            self.after(0 if self._active else 50, self._frame)

    def _adapt_detail(self, work: float) -> None:
        """
        Trade plotted points for time when frames get close to a refresh.

        A frame that overruns misses its refresh and the previous one shows
        twice - the stutter all of this exists to remove. A wide window on a
        big screen costs more, so detail backs off until frames fit, and
        comes back when they do.
        """
        self._work = 0.9 * self._work + 0.1 * work
        period = 1.0 / self.pacer.hz
        if self._work > 0.8 * period:
            self._pix_per_pt = min(4.0, self._pix_per_pt * 1.1)
        elif self._work < 0.6 * period:
            self._pix_per_pt = max(1.0, self._pix_per_pt / 1.02)

    def _pump(self, now: float) -> None:
        """Take whatever the link has delivered since the last frame."""
        if not self.link:
            return

        # Ask what the device is actually set to the moment the link comes
        # up, not on a timer: a Bluetooth scan takes longer than any fixed
        # delay worth waiting, and a request sent early is simply lost.
        if self.link.connected and not self._was_connected:
            self._was_connected = True
            self._send(link.CMD_GET_CONFIG)
        elif not self.link.connected:
            self._was_connected = False

        while True:
            try:
                msg = self.link.status.get_nowait()
            except queue.Empty:
                break
            colour = "#ff6b6b" if "fail" in msg or "not found" in msg \
                else "#5ed18b"
            self.status.config(text=msg, fg=colour)

        block_raw = []
        while True:
            try:
                f = self.link.frames.get_nowait()
            except queue.Empty:
                break

            if f.type == link.TYPE_DATA:
                got = link.decode_data(f.payload)
                if got is None:
                    continue
                ts, seq, enc, vals = got
                self.frames += 1

                if self.last_seq is not None and seq != self.last_seq:
                    self.gaps += 1
                self.last_seq = seq + len(vals)

                block_raw.append((ts, seq, vals))
            elif f.type == link.TYPE_RSP:
                self._on_response(bytes(f.payload))

        if block_raw:
            self._consume(block_raw)
            self._note_arrival(now)

    def _note_arrival(self, now: float) -> None:
        """
        Trail the newest sample by a little more than the longest recent gap
        between deliveries. Less, and the trace stalls waiting for the next
        burst; more, and it lags for nothing.
        """
        if self._last_data_t:
            gap = now - self._last_data_t
            if gap < 1.0:  # a pause in streaming is not link jitter
                self._gap_peak = max(gap, self._gap_peak * math.exp(-gap / 3.0))
                self._latency = min(LATENCY_MAX_S, max(
                    LATENCY_MIN_S, self._gap_peak * 1.5 + 1.0 / self.pacer.hz))
        self._last_data_t = now

    def _on_response(self, p: bytes) -> None:
        if len(p) < 2 or p[0] != link.CMD_GET_CONFIG or p[1] != 0:
            return
        if len(p) < 7:
            return

        sps = p[4] | (p[5] << 8)
        if sps in (250, 500, 1000):
            self.rate_var.set(str(sps))
            if sps != self.rate:
                self.rate = sps
                self._reset_traces()
            self.chain.set_rate(sps)

        enc = p[3]
        if enc == link.ENC_RAW_I32:
            # The same samples as 24-bit plus a byte of sign extension. A
            # board flashed before 24-bit became its default still starts here.
            self._send(link.CMD_SET_ENCODING, link.ENC_RAW_I24)
            enc = link.ENC_RAW_I24
        self.enc_var.set({link.ENC_RAW_I24: "raw 24-bit",
                          link.ENC_UV_F32: "device filtered"}.get(
                              enc, "raw 24-bit"))
        self.dev_notch.set({0: "off", 50: "50 Hz", 60: "60 Hz"}.get(p[6], "off"))

        if len(p) >= 15:
            chset = p[7:15]
            code = (chset[0] >> 4) & 0x07
            self.gain = link.GAIN_FROM_CODE.get(code, 24)
            self.gain_var.set(str(self.gain))
            mux = chset[0] & 0x07
            self.src_var.set({link.MUX_NORMAL: "Electrodes",
                              link.MUX_SHORTED: "Shorted (noise)",
                              link.MUX_TEST: "Test signal"}.get(mux,
                                                                "Electrodes"))
        self.status.config(text=f"connected - {sps} SPS, gain {self.gain}",
                           fg="#5ed18b")

    def _consume(self, blocks) -> None:
        counts = np.vstack([v for _, _, v in blocks])

        # "device filtered" arrives already in microvolts, having been
        # through the device's own chain. Scaling it by the LSB again would
        # divide it by 45 million.
        already_uv = self.enc_var.get() == "device filtered"
        scale = 1.0 if already_uv else link.lsb_uv(self.gain)

        self.samples += len(counts)

        if self.recorder is not None:
            # Each batch carries the hardware timestamp of its own first
            # sample, and the rest follow at the sample period. Stamping every
            # row from the first batch of a delivery, a microsecond apart,
            # put samples tens of milliseconds from where they belong -
            # enough to smear an ERP.
            period_us = 1e6 / self.rate
            for ts0, seq0, vals in blocks:
                for i, row in enumerate(vals):
                    self.recorder.writerow(
                        [round(ts0 + i * period_us), seq0 + i, *row.tolist()])
            self.rec_rows += len(counts)

        # Only meaningful on raw counts; the filtered form has had its DC
        # removed on the device and can no longer show the input limit.
        if not already_uv:
            self._check_limits(counts)
        for ch in range(link.CHANNELS):
            self.dc_mv[ch] = float(np.mean(counts[:, ch])) * scale / 1000.0

        uv = counts.astype(np.float64) * scale

        # Re-aim the notch from what the mains is actually doing, using
        # unfiltered samples - the notch has already removed the evidence
        # from anything downstream of it. The chain keeps its state through
        # a re-aim; restarting it here was putting each electrode's whole
        # offset back through the high-pass every few seconds.
        if not already_uv:
            self.mains_buf.extend(uv.mean(axis=1))
            now = time.time()
            if now >= self.mains_next and len(self.mains_buf) >= self.rate * 4:
                self.mains_next = now + 5.0
                self.chain.update_mains(np.array(self.mains_buf))

        out = self.chain.process(uv)
        self._ring_write(out.T)

    def _check_limits(self, counts: np.ndarray) -> None:
        """
        Note which inputs are clipping, or close to it.

        Nothing here hides a channel. Near the limit the data is still real -
        blinks show through an electrode offset of 180 mV - so it is drawn as
        normal and labelled. Only clipped samples are lost, and the label
        says so.
        """
        now = time.perf_counter()
        peak = np.max(np.abs(counts), axis=0)
        near = peak >= NEAR_COUNTS

        self.clip_t[peak >= CLIP_COUNTS] = now
        self.near_t[near] = now
        self.near_since[near & np.isnan(self.near_since)] = now
        self.near_since[~near] = np.nan

        for ch in range(link.CHANNELS):
            if (self.auto_out[ch] or not self.in_avg[ch].get()
                    or np.isnan(self.near_since[ch])
                    or now - self.near_since[ch] < AUTO_OUT_S):
                continue

            # Pinned near its limit: an electrode that is off or barely
            # touching. Its drift and mains would reach every other channel
            # through the average, so it leaves the average - once. Ticked
            # back in by hand, it stays in.
            self.auto_out[ch] = True
            self.in_avg[ch].set(False)
            self._set_car_mask()
            self.status.config(
                text=f"CH{ch + 1} {SITES[ch]} left out of the common average"
                     f" - input near its limit", fg="#ffd866")

    # -------------------------------------------------------------- draw --

    def _cfg(self, item: int, **kw) -> None:
        """itemconfigure, skipped when nothing changed - Tk redraws either way."""
        key = tuple(sorted(kw.items()))
        if self._item_cfg.get(item) != key:
            self._item_cfg[item] = key
            self.canvas.itemconfigure(item, **kw)

    def _render(self, now: float) -> None:
        c = self.canvas
        self._advance_clock(now)
        self._update_stats(now)

        w, h = c.winfo_width(), c.winfo_height()
        shown = [i for i in range(link.CHANNELS) if self.enabled[i].get()]
        if w < 60 or h < 60 or not shown:
            if self._lay is not None:
                c.delete("all")
                self._lay = None
            return

        window = float(self.window_var.get())
        key = (w, h, tuple(shown), window)
        if self._lay is None or self._lay["key"] != key:
            self._lay = self._layout(key)
        lay = self._lay

        self._draw_traces(lay, window, now)

        settled = (time.time() - self.started_at) > self.chain.settling_seconds
        self._cfg(lay["settle"], text=(
            f"filters settling ({self.chain.settling_seconds:.0f} s)"
            if not settled and self.samples else ""))

        clipping = sum(1 for ch in shown
                       if now - self.clip_t[ch] < LIMIT_HOLD_S)
        if clipping:
            # Full-scale differential input is VREF/gain, so 187.5 mV at
            # gain 24. An electrode that is not touching skin floats well
            # past that, which is the usual reason a channel clips.
            fs_mv = 4500.0 / max(1, self.gain)
            text = (f"{clipping} channel(s) clipping at +/-{fs_mv:.0f} mV - "
                    f"those samples are lost; check the electrode")
        else:
            text = ""
        self._cfg(lay["rail"], text=text)

    def _layout(self, key) -> dict:
        """
        Make the canvas items for one size and channel selection.

        Items are made once and then moved. Deleting and recreating every
        line and label each frame cost more than the whole redraw does now.
        """
        w, h, shown, window = key
        c = self.canvas
        c.delete("all")
        self._item_cfg.clear()

        left = 118
        plot_w = w - left - 24
        lane = h / len(shown)

        # A grid line a second, drawn first so the traces sit on top of it.
        for sec in range(1, int(window) + 1):
            x = left + plot_w * (sec / window)
            c.create_line(x, 0, x, h, fill="#171a21")

        bases, subs, warns, traces = [], [], [], []
        for row, ch in enumerate(shown):
            base = lane * (row + 0.5)
            bases.append(base)
            c.create_line(left, base, w - 24, base, fill="#1c2029")
            c.create_text(left - 10, base - 7, anchor="e", fill=COLORS[ch],
                          text=f"CH{ch + 1} {SITES[ch]}",
                          font=("Consolas", 10, "bold"))
            subs.append(c.create_text(left - 10, base + 8, anchor="e",
                                      fill=DIM, text="",
                                      font=("Consolas", 8)))
            warns.append(c.create_text(left - 10, base + 21, anchor="e",
                                       fill=DIM, text="",
                                       font=("Consolas", 8, "bold")))
            traces.append(c.create_line(0, 0, 0, 0, fill=COLORS[ch],
                                        width=1))

        c.create_text(w - 26, h - 10, anchor="e", fill=DIM,
                      font=("Consolas", 8), text=f"{window:.0f} s window")

        return {
            "key": key, "left": left, "plot_w": plot_w, "lane": lane,
            "rows": list(shown), "bases": np.array(bases), "subs": subs,
            "warns": warns, "traces": traces,
            "settle": c.create_text(w // 2, 16, fill="#ffd866",
                                    font=("Segoe UI", 10), text=""),
            "rail": c.create_text(w // 2, h - 26, fill="#ff6b6b",
                                  font=("Segoe UI", 11, "bold"), text=""),
        }

    def _advance_clock(self, now: float) -> None:
        """
        Move the right edge of the plot through the stream.

        It runs at the sample rate and aims to sit `_latency` behind the
        newest sample, pulled there gently: a burst arriving changes its
        speed by a few percent instead of making the trace jump. It never
        runs past the last sample.
        """
        dt = min(0.25, max(0.0, now - self._clock_t))
        self._clock_t = now
        self._frame_dt = dt

        if self.n_total == 0:
            self.disp = 0.0
            return

        target = self.n_total - self._latency * self.rate
        self.disp += dt * self.rate
        err = target - self.disp
        if abs(err) > 0.5 * self.rate:
            self.disp = target  # starting, or recovering from a stall
        else:
            self.disp += err * min(1.0, dt * 3.0)
        self.disp = max(0.0, min(self.disp, float(self.n_total)))

    def _draw_traces(self, lay: dict, window: float, now: float) -> None:
        """
        Put the visible stretch of the stream on the canvas.

        Never much more than a point per pixel. Past that, each bin of
        samples is drawn as its minimum and maximum, which shows the same
        picture - spikes included - for a fraction of the work. Bins are
        pinned to sample numbers rather than to screen columns, so a
        scrolling trace does not shimmer as samples cross column edges.
        """
        c = self.canvas
        rows = lay["rows"]
        left, plot_w = lay["left"], lay["plot_w"]

        n_win = max(2, int(round(window * self.rate)))
        start = self.disp - n_win
        first = max(0, self.n_total - self.cap, math.ceil(start))
        end = min(self.n_total, math.floor(self.disp) + 1)
        pxps = plot_w / n_win
        budget = max(50, int(plot_w / self._pix_per_pt))

        vals = xs = None
        if end - first >= 2 and n_win <= budget:
            vals = self._ring_read(first, end)[rows]
            xs = left + (np.arange(first, end) - start) * pxps
        elif end - first >= 2:
            b = math.ceil(2.0 * n_win / budget)
            k0, k1 = -(-first // b), end // b
            if k1 > k0:
                seg = self._ring_read(k0 * b, k1 * b)[rows]
                seg = seg.reshape(len(rows), k1 - k0, b)
                lo, hi = seg.min(axis=2), seg.max(axis=2)
                lo_first = seg.argmin(axis=2) <= seg.argmax(axis=2)
                vals = np.empty((len(rows), 2 * (k1 - k0)))
                vals[:, 0::2] = np.where(lo_first, lo, hi)
                vals[:, 1::2] = np.where(lo_first, hi, lo)
                xk = left + (np.arange(k0, k1) * b - start) * pxps
                xs = np.empty(2 * (k1 - k0))
                xs[0::2] = xk
                xs[1::2] = xk + (b - 1) * pxps

        spans = self._spans(rows, vals)

        if vals is None:
            for item in lay["traces"]:
                c.coords(item, 0, 0, 0, 0)
        else:
            k = (lay["lane"] * 0.42) / spans
            ys = (lay["bases"][:, None]
                  - np.clip(vals, -spans[:, None], spans[:, None]) * k[:, None])
            pts = np.empty((len(rows), 2 * xs.size))
            pts[:, 0::2] = xs
            pts[:, 1::2] = ys
            # Straight to Tcl, the coordinates as one list: tkinter's coords()
            # flattens and copies every argument first.
            call, name = c.tk.call, c._w
            for r, item in enumerate(lay["traces"]):
                call(name, "coords", item, pts[r].tolist())

        # Every channel keeps its colour whatever its state - a trace near
        # the limit is still data, and blinks show through it. The warning
        # goes underneath the label instead.
        for r, ch in enumerate(rows):
            self._cfg(lay["subs"][r], text=f"+/-{spans[r]:,.0f}uV")
            if now - self.clip_t[ch] < LIMIT_HOLD_S:
                warn, colour = "CLIPPING", "#ff6b6b"
            elif now - self.near_t[ch] < LIMIT_HOLD_S:
                warn, colour = "near limit", "#ffd866"
            elif not self.chain.car_mask[ch]:
                warn, colour = "not in average", DIM
            else:
                warn, colour = "", DIM
            self._cfg(lay["warns"][r], text=warn, fill=colour)

    def _spans(self, rows: list[int], vals) -> np.ndarray:
        """
        Vertical range for each lane.

        Per-channel by default. A shared range is useless the moment one
        electrode is off: that channel sits at 187 500 uV, and a range large
        enough to contain it draws every real trace as a flat line - which
        looks exactly like a device that is not working.

        In auto a range grows at once and shrinks over a second or so, so a
        blink scrolling out of view does not snap its lane to a new scale.
        """
        pick = self.scale_var.get()
        if pick != "auto":
            return np.full(len(rows), float(pick))

        current = self._span[rows]
        if vals is None or vals.shape[1] == 0:
            return current

        peak = np.max(np.abs(vals), axis=1)
        if not self.perch_var.get():
            peak[:] = peak.max()
        target = np.maximum(5.0, peak * 1.15)

        shrink = min(1.0, self._frame_dt * 2.0)
        span = np.where(target > current, target,
                        current + (target - current) * shrink)
        self._span[rows] = span
        return span

    def _update_stats(self, now: float) -> None:
        """Text that would change faster than it can be read: 5 times a second."""
        self._fps_count += 1
        if now - self._fps_t >= 1.0:
            self._fps = self._fps_count / (now - self._fps_t)
            self._fps_count = 0
            self._fps_t = now

        if now < self._stats_next:
            return
        self._stats_next = now + 0.2

        m = self.chain.measured_mains
        if m and self.chain.notch_track:
            self.mains_label.config(
                text=f"mains measured at {m:.2f} Hz")
        elif self.chain.notch_hz:
            self.mains_label.config(text="mains: using nominal")
        else:
            self.mains_label.config(text="")

        el = max(1e-3, time.time() - self.started_at)
        rec = f"  rec {self.rec_rows}" if self.recorder else ""
        bad = self.link.bad_frames if self.link else 0
        dc = " ".join(f"{v:+.0f}" for v in self.dc_mv)
        if self._active:
            display = (f"plot {self._fps:5.1f} fps on {self.pacer.hz:.0f} Hz, "
                       f"{self._latency * 1000:.0f} ms behind")
        else:
            display = "plot idle - no data arriving"
        self.stats.config(text="\n".join([
            f"{self.samples} samples  {self.samples / el:6.1f} SPS",
            f"{self.frames} frames  {bad} bad  {self.gaps} gaps{rec}",
            f"DC mV: {dc}",
            display]))

    def _close(self) -> None:
        if self.recorder is not None:
            self._rec_file.close()
        if self.link:
            try:
                self.link.send(link.CMD_STREAM_STOP)
            except Exception:  # noqa: BLE001
                pass
            self.link.close()
        self.pacer.close()
        self.after(200, self.destroy)


if __name__ == "__main__":
    App().mainloop()
