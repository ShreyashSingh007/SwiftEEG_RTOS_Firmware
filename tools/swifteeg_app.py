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

  * The same filters can run on the device instead. They are designed here
    and sent to it as sections, and it then streams raw and filtered side by
    side: raw for the recording, filtered for the plot. Moving the DSP
    on-chip is a switch, not a second implementation.

  * The plot is paced by the monitor, not by the data. Bluetooth delivers
    samples in bursts, and drawing each burst as it lands makes the trace
    jump. Instead the display trails the newest sample by a few tens of
    milliseconds and scrolls at a steady speed, one redraw per refresh.
"""

from __future__ import annotations

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
AXIS_COLORS = ["#ff6b6b", "#5ed18b", "#4ea1ff"]  # x, y, z

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
# How long to wait for the device to answer a rate change before asking again.
RATE_ANSWER_S = 3.0


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
        # Chain.car_mask for why a bad electrode must not. None to start
        # with, like the average itself.
        self.in_avg = [tk.BooleanVar(value=False) for _ in range(link.CHANNELS)]

        # Input limits, per channel. At gain 24 the input range is only
        # +/-187.5 mV, and an electrode offset near that leaves no room.
        # Kept as times rather than flags, so a warning holds long enough to
        # be read.
        self.clip_t = np.full(link.CHANNELS, -np.inf)
        self.near_t = np.full(link.CHANNELS, -np.inf)
        self.near_since = np.full(link.CHANNELS, np.nan)
        self.auto_out = [False] * link.CHANNELS
        self.dc_mv = [0.0] * link.CHANNELS

        # Motion sensor. Every sample carries a timestamp from the device's
        # TIMER1 - the clock the EEG is stamped on - so motion is kept by
        # time rather than by index, and drawn on the EEG's time axis.
        self.imu_cap = int((MAX_WINDOW_SECONDS + 1.0) * max(link.IMU_RATES))
        self.imu_ring = np.zeros((6, self.imu_cap), dtype=np.float32)
        self.imu_ts = np.zeros(self.imu_cap)
        self.imu_n = 0
        self.imu_last_seq: int | None = None
        self.imu_gaps = 0
        self.imu_flags_seen = 0
        self.imu_period_us = 0.0
        self.imu_present = False
        self.imu_recorder: csv.writer | None = None
        self._imu_file = None
        self.imu_rows = 0
        # (sample number in the EEG stream, device time of that sample)
        self.eeg_anchor: tuple[int, float] | None = None

        # The notch settings the device was last sent. The device designs its
        # notch itself and aims it at the mains frequency it measures; the
        # chain here aims where the device says its notch is.
        self._notch_sent: tuple | None = None

        self.samples = 0
        self.frames = 0
        self.last_seq: int | None = None
        self.gaps = 0
        self.leadoff = 0
        self.started_at = 0.0
        self.recorder: csv.writer | None = None
        self._rec_file = None
        self.rec_rows = 0

        # Where the filters run. On the device, what it was last sent is
        # kept per stage, so an unchanged stage is never sent - and so never
        # restarted - again.
        self.filters_on_device = False
        self._dev_sent: dict = {}
        self._dev_settle_t = -math.inf
        self._dev_applied_seq: int | None = None

        # A rate change in progress, finished by the device's answer; see
        # _set_rate.
        self._rate_change: dict | None = None

        # Display state.
        self.pacer = FramePacer()
        self._lay: dict | None = None
        self._item_cfg: dict = {}
        self._span = np.full(link.CHANNELS, 50.0)
        self._motion_span = [0.1, 10.0]
        self._active = False
        self._clock_t = time.perf_counter()
        self._frame_dt = 0.0
        self._last_data_t = 0.0
        self._gap_peak = 0.0
        self._latency = 0.1
        # How far the newest motion sample trails the newest EEG sample, and
        # the recent peak of that; see _note_arrival.
        self._imu_behind: float | None = None
        self._imu_peak = 0.0
        self._imu_newest_us: float | None = None
        self._imu_seen_t = 0.0
        # When the filters will have settled after the last change to them.
        self._settle_until = 0.0
        self._work = 0.0
        self._pix_per_pt = 1.0
        self._stats_next = 0.0
        self._fps = 0.0
        self._fps_count = 0
        self._fps_t = time.perf_counter()
        self._error_next = 0.0

        self._build()
        # The chain takes its settings from the controls, so the two agree
        # from the start rather than only after the first change.
        self._rebuild_chain()
        self._set_car_mask()
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
        self._check(f, "Bias drive (right mastoid)", self.bias_var,
                    self._set_bias)
        self._check(f, "Lead-off detection", self.loff_var, self._set_leadoff)
        self._label(f, "SRB1 reference: left mastoid", fg=DIM,
                    font=("Segoe UI", 8)).pack(fill=tk.X, pady=(4, 0))

        # -- motion sensor --
        f = self._section(inner, "motion sensor")
        self.imu_rate_var = tk.StringVar(value="240")
        self.imu_acc_var = tk.StringVar(value="8")
        self.imu_gyro_var = tk.StringVar(value="2000")
        self._combo_row(f, "rate", self.imu_rate_var,
                        ["off"] + [str(r) for r in link.IMU_RATES],
                        self._set_imu, "Hz")
        self._combo_row(f, "accel range", self.imu_acc_var,
                        [str(g) for g in link.IMU_ACCEL_G], self._set_imu,
                        "+/- g")
        self._combo_row(f, "gyro range", self.imu_gyro_var,
                        [str(d) for d in link.IMU_GYRO_DPS], self._set_imu,
                        "+/- deg/s")
        self.motion_var = tk.BooleanVar(value=True)
        self._check(f, "Show motion under the EEG", self.motion_var, None)

        # -- filters --
        f = self._section(inner, "filters (display)")

        # Where the chain runs. On the device it is the same chain, sent as
        # sections; the recording stays raw either way.
        self.site_var = tk.StringVar(value="PC")
        self._combo_row(f, "run on", self.site_var, ["PC", "device"],
                        self._set_filter_site, "")
        self.site_label = self._label(f, "", fg=DIM, font=("Consolas", 8))
        self.site_label.pack(fill=tk.X, pady=(0, 4))

        self.hp_var = tk.StringVar(value=f"{eeg_dsp.DEFAULT_HIGHPASS_HZ:.1f}")
        self.lp_var = tk.StringVar(value=f"{eeg_dsp.DEFAULT_LOWPASS_HZ:.0f}")
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

        self.car_var = tk.BooleanVar(value=False)
        self.harm_var = tk.BooleanVar(value=True)
        self.track_var = tk.BooleanVar(value=True)
        self._check(f, "Common average (shared zero)", self.car_var,
                    self._rebuild_chain)
        # The average needs two or more channels ticked "in average", and
        # none are to start with.
        self.car_hint = self._label(f, "", fg="#ffd866", font=("Consolas", 8))
        self.car_hint.pack(fill=tk.X)
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
        self.scale_var = tk.StringVar(value="200")
        self._combo_row(f, "range +/-", self.scale_var,
                        ["auto", "10", "25", "50", "100", "200", "500",
                         "2000", "200000"], None, "uV")

        self.window_var = tk.StringVar(value="3")
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
            self._dev_sent = {}
            self._notch_sent = None
            self._rate_change = None
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
        start = self.btn_stream["text"].startswith("Start")
        self.btn_stream.config(text="Stop streaming" if start
                               else "Start streaming")
        if self._rate_change is not None:
            # The stream is stopped for the change, which starts it again
            # when it finishes - or now does not.
            self._rate_change["resume"] = start
        elif start:
            self._begin_stream()
        else:
            self._send(link.CMD_STREAM_STOP)

    def _begin_stream(self) -> None:
        """Start the stream afresh: nothing from before it is kept."""
        self._forget_samples()
        self.chain.reset()
        self._settle_until = time.time() + self.chain.settling_seconds
        self._send(link.CMD_STREAM_START)

    def _forget_samples(self) -> None:
        """
        Drop everything taken from the stream so far, wherever it breaks - a
        start, a rate change - so nothing from before the break is plotted or
        counted.
        """
        self._reset_traces()
        self._reset_imu()
        self.samples = 0
        self.frames = 0
        self.last_seq = None
        self.gaps = 0
        self.started_at = time.time()

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
        self.eeg_anchor = None

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
        Changing rate restarts acquisition on the device. Everything held here
        belongs to the old rate: the filter coefficients, the samples already
        plotted, the sequence counter, the mains buffer. The device drops the
        sections it was sent too: designed for the old rate, they would be
        different filters at the new one.

        The change finishes on the device's answer, not on a timer. Commands
        run in order there, so a GET_CONFIG sent after SET_RATE is answered
        only once the restart is done. Until then every EEG frame is dropped:
        each was sampled at the old rate, or before the restart. Then the
        filters go back, and only after them the stream, so the first sample
        at the new rate is already filtered. A fixed two-second wait used to
        stand in for all of this: frames from before the change could reach
        the new filters, and two quick changes overlapped.
        """
        new = int(self.rate_var.get())
        change = self._rate_change
        if change is None and new == self.rate:
            return

        self.rate = new
        self.chain.set_rate(new)
        self._forget_samples()
        self._dev_sent = {}

        if not (self.link and self.link.connected):
            return  # the device's own rate is taken when it connects

        streaming = self.btn_stream["text"].startswith("Stop")
        if change is None and streaming:
            self._send(link.CMD_STREAM_STOP)

        self._send(link.CMD_SET_RATE, new & 0xFF, new >> 8)
        self._send(link.CMD_GET_CONFIG)
        self._rate_change = {
            "rate": new,
            "resume": change["resume"] if change else streaming,
            # One answer per GET_CONFIG sent; the last one decides.
            "answers": (change["answers"] if change else 0) + 1,
            "retries": 0,
            "since": time.perf_counter(),
        }
        self.status.config(text=f"switching to {new} SPS...", fg="#ffd866")

    def _rate_change_overdue(self, now: float) -> None:
        """Ask again if the device has not answered; give up after a while."""
        change = self._rate_change
        if change is None or now - change["since"] < RATE_ANSWER_S:
            return
        if change["retries"] < 2:
            change["retries"] += 1
            change["answers"] = 1
            change["since"] = now
            self._send(link.CMD_GET_CONFIG)
            return
        self._rate_change = None
        if change["resume"]:
            self._begin_stream()
        self.status.config(text="no answer to the rate change - press Reset "
                                "if the stream does not come back",
                           fg="#ff6b6b")

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
        self._dev_sent = {}
        self._notch_sent = None
        self._rate_change = None
        self.chain.reset()
        self._forget_samples()
        self.status.config(text="link reset - press Connect", fg="#ffd866")

    def _set_filter_site(self) -> None:
        """
        Run the chain here, or on the device.

        On the device it is this chain, sent as sections, and the stream
        carries raw counts and the device's microvolts side by side: raw for
        the recording and the limit checks, filtered for the plot. Back on
        the PC the device returns to its own default, the mains notch alone,
        and is left as any other host would expect to find it.
        """
        self.filters_on_device = self.site_var.get() == "device"
        self._dev_sent = {}
        self._dev_applied_seq = None
        self.chain.reset()

        if self.filters_on_device:
            self._send(link.CMD_SET_ENCODING, link.ENC_RAW_UV)
            self._push_device_chain()
            self.site_label.config(text="sending the filters to the device...",
                                   fg="#ffd866")
        else:
            self._send(link.CMD_SET_ENCODING, link.ENC_RAW_I24)
            self._restore_device_chain()
            self.site_label.config(text="", fg=DIM)

    def _push_device_chain(self) -> None:
        """
        Send the device whatever part of the chain it does not already have.

        By the rules the chain here follows. A stage whose sections are
        unchanged is not sent, so a new low-pass does not restart the
        high-pass. One with as many sections as the device holds is retuned in
        place, keeping its state - a new drift cut, the notch following the
        mains. Only a stage that gains or loses sections starts again, primed
        on its next sample.
        """
        if not (self.filters_on_device and self.link and self.link.connected):
            return

        pre, post = self.chain.device_stages()
        for stage, sections in ((link.STAGE_PRE, pre), (link.STAGE_POST, post)):
            key = (self.rate, tuple(sections))
            previous = self._dev_sent.get(stage)
            if previous == key:
                continue
            keep = (previous is not None and previous[0] == self.rate
                    and len(previous[1]) == len(sections))
            self._send(link.CMD_SET_FILTER,
                       *link.filter_args(stage, sections, self.rate, keep))
            self._dev_sent[stage] = key

        car = (bool(self.chain.car), self.chain.car_bits)
        if self._dev_sent.get("car") != car:
            self._send(link.CMD_SET_CAR, 1 if car[0] else 0, car[1])
            self._dev_sent["car"] = car

    def _restore_device_chain(self) -> None:
        """The device's own default: its mains notch alone, no average."""
        for stage in (link.STAGE_PRE, link.STAGE_POST):
            self._send(link.CMD_SET_FILTER,
                       *link.filter_args(stage, [], self.rate))
        self._send(link.CMD_SET_CAR, 0, 0xFF)

    def _send_notch(self) -> None:
        """
        The notch settings, to the device, when they differ from what it has
        - wherever the filters run. The device measures the mains either way,
        and the chain here follows where the device aims its notch.
        """
        settings = self.chain.device_notch()
        if self._notch_sent != settings and self.link and self.link.connected:
            self._send(link.CMD_SET_NOTCH, *link.notch_args(*settings))
            self._notch_sent = settings

    def _back_to_pc(self, why: str) -> None:
        """Filters back on the PC, saying why - never a silently wrong plot."""
        self.site_var.set("PC")
        self.filters_on_device = False
        self._dev_sent = {}
        self.chain.reset()
        self._send(link.CMD_SET_ENCODING, link.ENC_RAW_I24)
        self.site_label.config(text=why, fg="#ff6b6b")

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

    def _set_imu(self) -> None:
        rate = self.imu_rate_var.get()
        on = rate != "off"
        hz = int(rate) if on else 240
        g = int(self.imu_acc_var.get())
        dps = int(self.imu_gyro_var.get())
        self._send(link.CMD_SET_IMU, 1 if on else 0, hz & 0xFF, hz >> 8,
                   g, dps & 0xFF, dps >> 8)
        # Motion already received stays: every frame carries its own period,
        # ranges and times, so old and new samples are each drawn right.
        # Clearing it blanked the lane until the new setting filled it again.
        self.imu_flags_seen = 0

    def _rebuild_chain(self) -> None:
        hp = self.hp_var.get()
        lp = self.lp_var.get()
        nz = self.hnotch_var.get()

        was_hp = self.chain.highpass_hz
        was_notch = self.chain.notch_hz
        self.chain.highpass_hz = 0.0 if hp == "off" else float(hp)
        self.chain.lowpass_hz = 0.0 if lp == "off" else float(lp)
        self.chain.notch_hz = {"off": 0.0, "50 Hz": 50.0, "60 Hz": 60.0}[nz]
        self.chain.notch_harmonic = self.harm_var.get()
        self.chain.car = self.car_var.get()
        self.chain.notch_q = float(self.q_var.get().split()[0])
        self.chain.notch_track = self.track_var.get()
        if self.chain.notch_hz != was_notch:
            # A measurement near 50 Hz says nothing about 60, and the device
            # forgets it too.
            self.chain.forget_mains()
        self.chain.rebuild()
        self._push_device_chain()
        self._send_notch()
        self._show_car_hint()

        # Every change applies from the next sample. After a new drift cut,
        # though, the high-pass still has to settle at its new corner - about
        # two seconds at 1 Hz, half a minute at 0.1 Hz - and the plot says so
        # rather than looking slow to respond.
        if self.chain.highpass_hz != was_hp:
            self._settle_until = time.time() + self.chain.settling_seconds

    def _set_car_mask(self) -> None:
        self.chain.car_mask = np.array([v.get() for v in self.in_avg],
                                       dtype=bool)
        self._push_device_chain()
        self._show_car_hint()

    def _show_car_hint(self) -> None:
        few = np.count_nonzero(self.chain.car_mask) < 2
        self.car_hint.config(text="tick 'in average' on 2 or more channels"
                             if self.chain.car and few else "")

    # ---------------------------------------------------------- recording --

    def _toggle_record(self) -> None:
        if self.recorder is not None:
            self._rec_file.close()
            self._imu_file.close()
            self.recorder = None
            self.imu_recorder = None
            self._rec_file = None
            self._imu_file = None
            self.btn_rec.config(text="Record")
            return

        stamp = time.strftime("swifteeg_%Y%m%d_%H%M%S")
        name = stamp + ".csv"
        self._rec_file = open(pathlib.Path.cwd() / name, "w", newline="",
                              encoding="utf-8")
        self.recorder = csv.writer(self._rec_file)

        # Raw counts, not microvolts: the scale depends on the gain, and a
        # recording that has already been filtered cannot be un-filtered.
        # With the filters on the device this still holds - it sends raw
        # alongside, and raw is what is kept.
        self.recorder.writerow(
            ["# SwiftEEG raw counts", f"rate={self.rate}", f"gain={self.gain}",
             f"lsb_uv={link.lsb_uv(self.gain):.9f}"])
        self.recorder.writerow(
            ["ts_us", "seq"] + [f"ch{i + 1}_{SITES[i]}"
                                for i in range(link.CHANNELS)])
        self.rec_rows = 0

        # Motion in a file of its own. ts_us in both files is the device's
        # TIMER1, so the two line up sample for sample.
        self._imu_file = open(pathlib.Path.cwd() / (stamp + "_motion.csv"),
                              "w", newline="", encoding="utf-8")
        self.imu_recorder = csv.writer(self._imu_file)
        self.imu_recorder.writerow(
            ["# SwiftEEG motion", "accel in g, gyro in degrees/s",
             "ts_us on the same clock as the EEG file",
             "flags: 1 time from a poll, not the sensor's interrupt; "
             "2 samples lost just before"])
        self.imu_recorder.writerow(
            ["ts_us", "seq", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps",
             "gz_dps", "flags"])
        self.imu_rows = 0

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

        self._rate_change_overdue(now)

        while True:
            try:
                msg = self.link.status.get_nowait()
            except queue.Empty:
                break
            colour = "#ff6b6b" if "fail" in msg or "not found" in msg \
                else "#5ed18b"
            self.status.config(text=msg, fg=colour)

        block_raw = []
        imu_raw = []
        while True:
            try:
                f = self.link.frames.get_nowait()
            except queue.Empty:
                break

            if f.type == link.TYPE_DATA:
                if self._rate_change is not None:
                    continue  # sampled at the old rate, or before the restart
                got = link.decode_data(f.payload)
                if got is None:
                    continue
                ts, seq, _, counts, uv = got
                if counts is None:
                    # Microvolts alone, left by another host: nothing to
                    # record or check until the config reply fixes it.
                    continue
                self.frames += 1

                if self.last_seq is not None and seq != self.last_seq:
                    self.gaps += 1
                self.last_seq = seq + len(counts)

                if f.flags & link.FLAG_SETTLING:
                    self._dev_settle_t = now

                block_raw.append((ts, seq, counts, uv))
            elif f.type == link.TYPE_IMU:
                got = link.decode_imu(f.payload)
                if got is not None:
                    imu_raw.append(got)
            elif f.type == link.TYPE_EVT:
                event = link.decode_event(bytes(f.payload))
                if event is not None and event["event"] == "mains":
                    self.chain.follow_mains(
                        event["hz"], event["hz"] if event["moved"] else None)
            elif f.type == link.TYPE_RSP:
                self._on_response(bytes(f.payload))

        # Motion first, so the EEG that came with it is measured against it
        # rather than against the batch before.
        if imu_raw:
            self._consume_imu(imu_raw)
        if block_raw:
            self._consume(block_raw)
            self._note_arrival(now)

    def _note_arrival(self, now: float) -> None:
        """
        Trail the newest sample by a little more than the longest recent gap
        between deliveries. Less, and the trace stalls waiting for the next
        burst; more, and it lags for nothing.

        The motion lanes share the time axis but arrive in batches of their
        own, so the newest motion sample can be well behind the newest EEG.
        The display trails that too, while motion is shown and arriving - or
        the motion trace stops short of the edge and catches up in jumps.
        """
        if self._last_data_t:
            gap = now - self._last_data_t
            if gap < 1.0:  # a pause in streaming is not link jitter
                decay = math.exp(-gap / 3.0)
                self._gap_peak = max(gap, self._gap_peak * decay)
                need = self._gap_peak * 1.5
                if (self._imu_behind is not None and self.motion_var.get()
                        and now - self._imu_seen_t < 1.0):
                    self._imu_peak = max(self._imu_behind,
                                         self._imu_peak * decay)
                    need = max(need, self._imu_peak * 1.2)
                self._latency = min(LATENCY_MAX_S, max(
                    LATENCY_MIN_S, need + 1.0 / self.pacer.hz))
        self._last_data_t = now

    def _on_response(self, p: bytes) -> None:
        if len(p) < 2:
            return
        if p[0] in (link.CMD_SET_FILTER, link.CMD_SET_CAR):
            self._on_filter_response(p)
            return

        cfg = link.decode_config(p)
        if cfg is None:
            return

        change = self._rate_change
        if change is not None:
            change["answers"] -= 1
            if change["answers"] > 0:
                return  # from before a later restart; the last answer decides
            if cfg["rate"] != change["rate"] and change["retries"] < 2:
                change["retries"] += 1
                change["answers"] = 1
                change["since"] = time.perf_counter()
                self._send(link.CMD_GET_CONFIG)
                return
            self._rate_change = None

        sps = cfg["rate"]
        if sps in (250, 500, 1000):
            self.rate_var.set(str(sps))
            if sps != self.rate:
                self.rate = sps
                self._forget_samples()
                self._dev_sent = {}
            self.chain.set_rate(sps)

        # The stream this app needs: raw, plus the device's microvolts when
        # the filters run there. A board left on anything else - an older
        # firmware's 32-bit default, another host's choice - is moved to it.
        want = link.ENC_RAW_UV if self.filters_on_device else link.ENC_RAW_I24
        if cfg["encoding"] != want:
            self._send(link.CMD_SET_ENCODING, want)

        if "gains" in cfg:
            self.gain = cfg["gains"][0] or 24
            self.gain_var.set(str(self.gain))
            self.src_var.set({link.MUX_NORMAL: "Electrodes",
                              link.MUX_SHORTED: "Shorted (noise)",
                              link.MUX_TEST: "Test signal"}.get(
                                  cfg["mux"][0], "Electrodes"))

        # Motion sensor, from firmware that has one. Older firmware sends
        # none of it.
        if "imu_rate" in cfg:
            self.imu_present = cfg["imu_fitted"]
            self.imu_rate_var.set(str(cfg["imu_rate"]) if cfg["imu_on"]
                                  else "off")
            self.imu_acc_var.set(str(cfg["imu_accel_g"]))
            self.imu_gyro_var.set(str(cfg["imu_gyro_dps"]))

        # The notch. The device designs it and aims it at the mains it
        # measures; its settings are this app's, and the chain here aims where
        # the device reports its notch is.
        if "notch_q" in cfg:
            self._notch_sent = (cfg["notch"], cfg["notch_q"],
                                cfg["notch_harmonic"], cfg["notch_track"])
            self._send_notch()
            self.chain.follow_mains(cfg["mains_hz"], cfg["notch_aim_hz"])

        if self.filters_on_device:
            if "pre_crc" not in cfg:
                self._back_to_pc("this firmware cannot run the filters")
            else:
                # What the device holds against what it should. A rate change
                # or a restart clears its copy, which is then sent again.
                pre, post = self.chain.device_stages()
                if (cfg["pre_crc"] != link.sections_crc(pre)
                        or cfg["post_crc"] != link.sections_crc(post)
                        or cfg["pre_count"] != len(pre)
                        or cfg["post_count"] != len(post)):
                    self._dev_sent = {}
                    self._push_device_chain()

        if change is not None:
            # The filters before the stream: the device runs commands in
            # order, so its first sample at the new rate is already filtered.
            self._push_device_chain()
            if change["resume"]:
                self._begin_stream()
            state = "streaming" if change["resume"] else "ready"
            self.status.config(text=f"{state} at {sps} SPS", fg="#5ed18b")
            return

        self.status.config(text=f"connected - {sps} SPS, gain {self.gain}",
                           fg="#5ed18b")

    def _on_filter_response(self, p: bytes) -> None:
        if not self.filters_on_device:
            return  # restoring the device's default; nothing to show

        if p[1] == link.STATUS_OK:
            seq = link.response_seq(p)
            if seq is not None:
                self._dev_applied_seq = seq
                self.site_label.config(
                    text=f"on the device from sample {seq}", fg="#5ed18b")
            return

        what = "filters" if p[0] == link.CMD_SET_FILTER else "common average"
        self._back_to_pc(f"the device refused the {what} - back on the PC")

    def _consume(self, blocks) -> None:
        counts = np.vstack([c for _, _, c, _ in blocks])
        scale = link.lsb_uv(self.gain)

        self.samples += len(counts)

        if self.recorder is not None:
            # Each batch carries the hardware timestamp of its own first
            # sample, and the rest follow at the sample period. Stamping every
            # row from the first batch of a delivery, a microsecond apart,
            # put samples tens of milliseconds from where they belong -
            # enough to smear an ERP.
            period_us = 1e6 / self.rate
            for ts0, seq0, vals, _ in blocks:
                for i, row in enumerate(vals):
                    self.recorder.writerow(
                        [round(ts0 + i * period_us), seq0 + i, *row.tolist()])
            self.rec_rows += len(counts)

        self._check_limits(counts)
        for ch in range(link.CHANNELS):
            self.dc_mv[ch] = float(np.mean(counts[:, ch])) * scale / 1000.0

        uv = counts.astype(np.float64) * scale

        # The device's own output when it is filtering; this chain otherwise,
        # and for any batch still arriving in the old form just after a
        # switch.
        device = [u for _, _, _, u in blocks]
        if self.filters_on_device and all(u is not None for u in device):
            out = np.vstack(device).astype(np.float64)
        else:
            out = self.chain.process(uv)

        # Where the newest batch sits on the device clock. The motion lanes
        # are placed by time, and this is what gives them the EEG's axis.
        last_ts, _, last_vals, _ = blocks[-1]
        self.eeg_anchor = (self.n_total + len(counts) - len(last_vals),
                           float(last_ts))
        self._ring_write(out.T)

        if self._imu_newest_us is not None:
            newest = last_ts + (len(last_vals) - 1) * 1e6 / self.rate
            self._imu_behind = (newest - self._imu_newest_us) / 1e6

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

    # ------------------------------------------------------------ motion --

    def _reset_imu(self) -> None:
        self.imu_n = 0
        self.imu_last_seq = None
        self.imu_gaps = 0
        self.imu_flags_seen = 0
        self._imu_newest_us = None
        self._imu_behind = None
        self._imu_peak = 0.0

    def _consume_imu(self, frames) -> None:
        for ts, seq, period_us, counts, accel_g, gyro_dps, flags in frames:
            n = len(counts)
            if self.imu_last_seq is not None and seq != self.imu_last_seq:
                self.imu_gaps += 1
            self.imu_last_seq = seq + n
            self.imu_period_us = period_us
            self.imu_flags_seen |= flags

            # Counts to units: one count is the full scale over 32768.
            scaled = counts.astype(np.float64)
            scaled[:, :3] *= accel_g / 32768.0
            scaled[:, 3:] *= gyro_dps / 32768.0
            times = ts + np.arange(n) * period_us

            if self.imu_recorder is not None:
                for i in range(n):
                    a = scaled[i]
                    self.imu_recorder.writerow(
                        [round(times[i]), seq + i,
                         f"{a[0]:.5f}", f"{a[1]:.5f}", f"{a[2]:.5f}",
                         f"{a[3]:.3f}", f"{a[4]:.3f}", f"{a[5]:.3f}", flags])
                self.imu_rows += n

            self._imu_write(scaled.T, times)
            if n:
                self._imu_newest_us = float(times[-1])
                self._imu_seen_t = time.perf_counter()

    def _imu_write(self, block: np.ndarray, times: np.ndarray) -> None:
        """block is (6, samples); times are device microseconds."""
        n = block.shape[1]
        cap = self.imu_cap
        if n >= cap:
            self.imu_n += n - cap
            block, times, n = block[:, n - cap:], times[n - cap:], cap

        i = self.imu_n % cap
        first = min(n, cap - i)
        self.imu_ring[:, i:i + first] = block[:, :first]
        self.imu_ts[i:i + first] = times[:first]
        if first < n:
            self.imu_ring[:, :n - first] = block[:, first:]
            self.imu_ts[:n - first] = times[first:]
        self.imu_n += n

    def _imu_recent(self, k: int):
        """The newest k motion samples, oldest first, and their times."""
        k = min(k, self.imu_n, self.imu_cap)
        i = (self.imu_n - k) % self.imu_cap
        if i + k <= self.imu_cap:
            return self.imu_ring[:, i:i + k], self.imu_ts[i:i + k]
        j = k - (self.imu_cap - i)
        return (np.concatenate((self.imu_ring[:, i:], self.imu_ring[:, :j]),
                               axis=1),
                np.concatenate((self.imu_ts[i:], self.imu_ts[:j])))

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
        motion = bool(self.motion_var.get()) and (self.imu_present
                                                  or self.imu_n > 0)
        key = (w, h, tuple(shown), window, motion)
        if self._lay is None or self._lay["key"] != key:
            self._lay = self._layout(key)
        lay = self._lay

        self._draw_traces(lay, window, now)
        self._draw_motion(lay, window)

        left = self._settle_until - time.time()
        if self.filters_on_device:
            # The device flags the samples inside its filters' settling time
            # after a restart. A retune it does not flag, so that is timed
            # here, as on the PC.
            settling = now - self._dev_settle_t < 0.5 or left > 0
            text = "device filters settling" if settling and self.samples else ""
        else:
            text = (f"filters settling ({math.ceil(left)} s)"
                    if left > 0 and self.samples else "")
        self._cfg(lay["settle"], text=text)

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
        w, h, shown, window, motion = key
        c = self.canvas
        c.delete("all")
        self._item_cfg.clear()

        left = 118
        plot_w = w - left - 24
        lane = h / (len(shown) + (2 if motion else 0))

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

        m_bases, m_subs, m_lines = [], [], []
        if motion:
            top = lane * len(shown)
            c.create_line(0, top, w, top, fill="#2a2f3a")
            for i, name in enumerate(("ACCEL", "GYRO")):
                base = lane * (len(shown) + i + 0.5)
                m_bases.append(base)
                c.create_line(left, base, w - 24, base, fill="#1c2029")
                c.create_text(left - 10, base - 7, anchor="e", fill=FG,
                              text=name, font=("Consolas", 10, "bold"))
                m_subs.append(c.create_text(left - 10, base + 8, anchor="e",
                                            fill=DIM, text="",
                                            font=("Consolas", 8)))
                for j, axis in enumerate("xyz"):
                    c.create_text(left - 34 + j * 12, base + 21, anchor="e",
                                  fill=AXIS_COLORS[j], text=axis,
                                  font=("Consolas", 8, "bold"))
                for colour in AXIS_COLORS:
                    m_lines.append(c.create_line(0, 0, 0, 0, fill=colour,
                                                 width=1))

        c.create_text(w - 26, h - 10, anchor="e", fill=DIM,
                      font=("Consolas", 8), text=f"{window:.0f} s window")

        return {
            "key": key, "left": left, "plot_w": plot_w, "lane": lane,
            "rows": list(shown), "bases": np.array(bases), "subs": subs,
            "warns": warns, "traces": traces, "motion": motion,
            "m_bases": m_bases, "m_subs": m_subs, "m_lines": m_lines,
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
            elif self.chain.car and not self.chain.car_mask[ch]:
                warn, colour = "not in average", DIM
            else:
                warn, colour = "", DIM
            self._cfg(lay["warns"][r], text=warn, fill=colour)

    def _draw_motion(self, lay: dict, window: float) -> None:
        """
        The motion lanes, on the EEG's time axis.

        Motion samples are placed by their device timestamp - the clock the
        EEG is stamped on - so a head movement lines up with the stretch of
        EEG it disturbed.
        """
        if not lay["motion"]:
            return

        c = self.canvas
        lines = lay["m_lines"]

        def blank():
            for item in lines:
                c.coords(item, 0, 0, 0, 0)

        if self.imu_n < 2 or self.eeg_anchor is None:
            blank()
            return

        idx, ts = self.eeg_anchor
        window_us = window * 1e6
        t_right = ts + (self.disp - idx) * 1e6 / self.rate
        t_left = t_right - window_us

        hz = 1e6 / self.imu_period_us if self.imu_period_us > 0 \
            else max(link.IMU_RATES)
        vals, times = self._imu_recent(int(window * hz * 1.2) + 32)
        keep = (times >= t_left) & (times <= t_right)
        if np.count_nonzero(keep) < 2:
            blank()
            return

        vals, times = vals[:, keep], times[keep]
        stride = max(1, math.ceil(times.size / lay["plot_w"]))
        vals, times = vals[:, ::stride], times[::stride]
        xs = lay["left"] + (times - t_left) * (lay["plot_w"] / window_us)

        call, name = c.tk.call, c._w
        for lane in range(2):
            part = vals[3 * lane:3 * lane + 3].astype(np.float64)
            if lane == 0:
                # Gravity sits on the accelerometer as a steady 1 g spread
                # over the axes by head angle. Taking each axis's mean out
                # shows movement rather than orientation.
                part = part - part.mean(axis=1, keepdims=True)

            floor = 0.02 if lane == 0 else 2.0
            target = max(floor, float(np.abs(part).max()) * 1.15)
            current = self._motion_span[lane]
            span = target if target > current else \
                current + (target - current) * min(1.0, self._frame_dt * 2.0)
            self._motion_span[lane] = span

            k = (lay["lane"] * 0.42) / span
            ys = lay["m_bases"][lane] - np.clip(part, -span, span) * k
            pts = np.empty((3, 2 * xs.size))
            pts[:, 0::2] = xs
            pts[:, 1::2] = ys
            for axis in range(3):
                call(name, "coords", lines[3 * lane + axis],
                     pts[axis].tolist())

            label = (f"+/-{span:.2f} g" if lane == 0
                     else f"+/-{span:,.0f} deg/s")
            self._cfg(lay["m_subs"][lane], text=label)

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
        aim = self.chain.notch_aim
        if not self.chain.notch_hz:
            self.mains_label.config(text="")
        elif m:
            where = (f"notch at {aim:.2f}" if self.chain.notch_track and aim
                     else "notch at nominal")
            self.mains_label.config(
                text=f"mains {m:.2f} Hz on the device, {where}")
        else:
            self.mains_label.config(text="mains not measured yet - notch at nominal")

        el = max(1e-3, time.time() - self.started_at)
        rec = f"  rec {self.rec_rows}" if self.recorder else ""
        bad = self.link.bad_frames if self.link else 0
        dc = " ".join(f"{v:+.0f}" for v in self.dc_mv)
        if self._active:
            display = (f"plot {self._fps:5.1f} fps on {self.pacer.hz:.0f} Hz, "
                       f"{self._latency * 1000:.0f} ms behind")
        else:
            display = "plot idle - no data arriving"

        if self.imu_n:
            hz = 1e6 / self.imu_period_us if self.imu_period_us else 0.0
            polled = ("  timed by poll"
                      if self.imu_flags_seen & link.IMU_FLAG_TIME_ESTIMATED
                      else "")
            motion = (f"motion {self.imu_n} samples  {hz:.2f} Hz  "
                      f"{self.imu_gaps} gaps{polled}")
        elif self.imu_present:
            motion = "motion sensor fitted, no samples yet"
        else:
            motion = "motion sensor not reported"

        site = "filters on the device" if self.filters_on_device \
            else "filters on the PC"

        self.stats.config(text="\n".join([
            f"{self.samples} samples  {self.samples / el:6.1f} SPS",
            f"{self.frames} frames  {bad} bad  {self.gaps} gaps{rec}",
            f"DC mV: {dc}",
            motion,
            site,
            display]))

    def _close(self) -> None:
        if self.recorder is not None:
            self._rec_file.close()
            self._imu_file.close()
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
