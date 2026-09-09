"""
SwiftEEG - Windows application.

Connects over Bluetooth or USB, gives every device setting a control, runs a
filter chain on the host, plots the result, and records raw data to disk.

    python tools/swifteeg_app.py

Needs pyserial for USB and bleak for Bluetooth. The window is tkinter, which
ships with Python and is native on Windows.

Two decisions worth knowing:

  * The device streams raw counts. Filtering happens here, where a setting
    can be changed and judged in a second. Recordings are raw for the same
    reason - a session can be re-analysed later with different settings,
    which is impossible once a filter has been baked in.

  * The filters are built from the same biquad forms the firmware uses, so a
    chain that works here transfers to the device as coefficients rather than
    as a rewrite. That is what makes moving the DSP on-chip a configuration
    step and not a second implementation.
"""

from __future__ import annotations

import collections
import csv
import pathlib
import queue
import sys
import time
import tkinter as tk
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

WINDOW_SECONDS = 5.0


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

        depth = int(WINDOW_SECONDS * 1000)      # resized when the rate changes
        self.traces = [collections.deque(maxlen=depth)
                       for _ in range(link.CHANNELS)]
        self.enabled = [tk.BooleanVar(value=True) for _ in range(link.CHANNELS)]

        # Raw counts sit at +/-2^23 when an input saturates. Tracked per
        # channel because a railed channel is not a small problem: at gain 24
        # the input range is only +/-187.5 mV, and an electrode offset larger
        # than that pins the converter with no signal left to filter.
        self.saturated = [False] * link.CHANNELS
        self.dc_mv = [0.0] * link.CHANNELS

        self.samples = 0
        self.frames = 0
        self.last_seq: int | None = None
        self.gaps = 0
        self.leadoff = 0
        self.started_at = 0.0
        self.recorder: csv.writer | None = None
        self._rec_file = None
        self.rec_rows = 0

        self._build()
        self.after(40, self._tick)
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
        side = tk.Frame(self, bg=PANEL, width=330)
        side.pack(side=tk.LEFT, fill=tk.Y)
        side.pack_propagate(False)

        inner = tk.Frame(side, bg=PANEL, padx=14, pady=12)
        inner.pack(fill=tk.BOTH, expand=True)

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
        self.enc_var = tk.StringVar(value="24-bit")
        ec = ttk.Combobox(row, textvariable=self.enc_var, width=8,
                          state="readonly", values=["24-bit", "32-bit"])
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
        self._check(f, "Bias drive (right mastoid)", self.bias_var,
                    self._set_bias)
        self._check(f, "Lead-off detection", self.loff_var, self._set_leadoff)
        self._label(f, "SRB1 reference: left mastoid", fg=DIM,
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

        self.car_var = tk.BooleanVar(value=True)
        self.harm_var = tk.BooleanVar(value=True)
        self._check(f, "Common average (shared zero)", self.car_var,
                    self._rebuild_chain)
        self._check(f, "Also notch the harmonic", self.harm_var,
                    self._rebuild_chain)

        # -- channels --
        f = self._section(inner, "channels")
        for i in range(link.CHANNELS):
            row = tk.Frame(f, bg=PANEL)
            row.pack(fill=tk.X)
            cb = tk.Checkbutton(row, variable=self.enabled[i], bg=PANEL,
                                fg=COLORS[i], selectcolor="#2a2f3a",
                                activebackground=PANEL, highlightthickness=0,
                                text=f"CH{i + 1}  {SITES[i]}",
                                font=("Consolas", 9), anchor="w", width=14)
            cb.pack(side=tk.LEFT)
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
                        ["1", "2", "3", "5", "10"], self._set_window, "s")

        self.perch_var = tk.BooleanVar(value=True)
        self._check(f, "Scale each channel separately", self.perch_var, None)

        self.stats = self._label(inner, "", fg=DIM, font=("Consolas", 8),
                                 justify=tk.LEFT)
        self.stats.pack(fill=tk.X, pady=(14, 0))

        self.canvas = tk.Canvas(self, bg=PLOT_BG, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

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
            for t in self.traces:
                t.clear()
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

    def _trace_depth(self) -> int:
        return max(2, int(float(self.window_var.get()) * self.rate))

    def _resize_traces(self, keep: bool = True) -> None:
        depth = self._trace_depth()
        self.traces = [
            collections.deque(list(t)[-depth:] if keep else [], maxlen=depth)
            for t in self.traces]

    def _set_window(self) -> None:
        self._resize_traces(keep=True)

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
        self._resize_traces(keep=False)
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
        self._resize_traces(keep=False)
        self.last_seq = None
        self.status.config(text="link reset - press Connect", fg="#ffd866")

    def _set_encoding(self) -> None:
        enc = (link.ENC_RAW_I24 if self.enc_var.get().startswith("24")
               else link.ENC_RAW_I32)
        self._send(link.CMD_SET_ENCODING, enc)

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
        self.chain.rebuild()

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
        self.recorder.writerow(
            ["# SwiftEEG raw counts", f"rate={self.rate}", f"gain={self.gain}",
             f"lsb_uv={link.lsb_uv(self.gain):.9f}"])
        self.recorder.writerow(
            ["ts_us", "seq"] + [f"ch{i + 1}_{SITES[i]}"
                                for i in range(link.CHANNELS)])
        self.rec_rows = 0
        self.btn_rec.config(text="Stop rec")
        self.status.config(text=f"recording {name}", fg="#5ed18b")

    # -------------------------------------------------------------- loop --

    def _tick(self) -> None:
        if self.link:
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

        self._draw()
        self.after(40, self._tick)

    def _on_response(self, p: bytes) -> None:
        if len(p) < 2 or p[0] != link.CMD_GET_CONFIG or p[1] != 0:
            return
        if len(p) < 7:
            return

        sps = p[4] | (p[5] << 8)
        if sps in (250, 500, 1000):
            self.rate_var.set(str(sps))
            self.rate = sps
            self.chain.set_rate(sps)

        self.enc_var.set("24-bit" if p[3] == link.ENC_RAW_I24 else "32-bit")
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
        scale = link.lsb_uv(self.gain)
        counts = np.vstack([v for _, _, v in blocks])

        self.samples += len(counts)

        if self.recorder is not None:
            ts0, seq0, _ = blocks[0]
            for i, row in enumerate(counts):
                self.recorder.writerow([ts0 + i, seq0 + i, *row.tolist()])
            self.rec_rows += len(counts)

        # Saturation is judged on raw counts, before any filter. A railed
        # channel is flat at full scale, and after a high-pass it looks like
        # a quiet channel rather than a broken one.
        full_scale = (1 << 23) * 0.98
        peak = np.max(np.abs(counts), axis=0)
        for ch in range(link.CHANNELS):
            self.saturated[ch] = bool(peak[ch] >= full_scale)
            self.dc_mv[ch] = float(np.mean(counts[:, ch])) * scale / 1000.0

        uv = counts.astype(np.float64) * scale
        out = self.chain.process(uv)

        for ch in range(link.CHANNELS):
            self.traces[ch].extend(out[:, ch])

    # -------------------------------------------------------------- draw --

    def _span_for(self, ch: int, shown) -> float:
        """
        Vertical range for one lane.

        Per-channel by default. A shared range is useless the moment one
        electrode is off: that channel sits at 187 500 uV, and a range large
        enough to contain it draws every real trace as a flat line - which
        looks exactly like a device that is not working.
        """
        pick = self.scale_var.get()

        if pick != "auto":
            return float(pick)

        if self.perch_var.get():
            source = [ch]
        else:
            source = shown

        peak = 0.0
        for i in source:
            t = self.traces[i]
            if t:
                peak = max(peak, max(abs(v) for v in t))

        return max(5.0, peak * 1.15)

    def _draw(self) -> None:
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 60 or h < 60:
            return

        shown = [i for i in range(link.CHANNELS) if self.enabled[i].get()]
        if not shown:
            return

        left = 118
        plot_w = w - left - 24
        lane = h / len(shown)
        window = float(self.window_var.get())

        settled = (time.time() - self.started_at) > self.chain.settling_seconds

        for row, ch in enumerate(shown):
            base = lane * (row + 0.5)
            span = self._span_for(ch, shown)

            c.create_line(left, base, w - 24, base, fill="#1c2029")

            # Lane label, plus what this lane is actually showing.
            c.create_text(left - 10, base - 7, anchor="e", fill=COLORS[ch],
                          text=f"CH{ch + 1} {SITES[ch]}",
                          font=("Consolas", 10, "bold"))

            if self.saturated[ch]:
                c.create_text(left - 10, base + 8, anchor="e", fill="#ff6b6b",
                              text="RAILED", font=("Consolas", 8, "bold"))
            else:
                c.create_text(left - 10, base + 8, anchor="e", fill=DIM,
                              text=f"+/-{span:,.0f}uV", font=("Consolas", 8))

            t = self.traces[ch]
            if len(t) < 2:
                continue

            step = plot_w / (len(t) - 1)
            k = (lane * 0.42) / span

            pts = []
            for i, v in enumerate(t):
                y = base - max(-span, min(span, v)) * k
                pts.extend((left + i * step, y))

            colour = "#4a3038" if self.saturated[ch] else COLORS[ch]
            c.create_line(*pts, fill=colour, width=1)

        # Time axis: a grid line a second, so the window is readable.
        for sec in range(1, int(window) + 1):
            x = left + plot_w * (sec / window)
            c.create_line(x, 0, x, h, fill="#171a21")
        c.create_text(w - 26, h - 10, anchor="e", fill=DIM,
                      font=("Consolas", 8), text=f"{window:.0f} s window")

        if not settled and self.samples:
            c.create_text(w // 2, 16, fill="#ffd866", font=("Segoe UI", 10),
                          text=f"filters settling "
                               f"({self.chain.settling_seconds:.0f} s)")

        n_rail = sum(1 for ch in shown if self.saturated[ch])
        if n_rail:
            # Full-scale differential input is VREF/gain, so 187.5 mV at
            # gain 24. An electrode that is not touching skin floats well
            # past that, which is the usual reason a channel rails.
            fs_mv = 4500.0 / max(1, self.gain)
            c.create_text(
                w // 2, h - 26, fill="#ff6b6b", font=("Segoe UI", 11, "bold"),
                text=f"{n_rail} channel(s) at full scale "
                     f"(+/-{fs_mv:.0f} mV) - electrode not connected, "
                     f"or contact lost")

        el = max(1e-3, time.time() - self.started_at)
        rec = f"  rec {self.rec_rows}" if self.recorder else ""
        bad = self.link.bad_frames if self.link else 0
        dc = " ".join(f"{v:+.0f}" for v in self.dc_mv)
        self.stats.config(
            text=(f"{self.samples} samples  {self.samples / el:6.1f} SPS"
                  + chr(10) +
                  f"{self.frames} frames  {bad} bad  {self.gaps} gaps{rec}"
                  + chr(10) +
                  f"DC mV: {dc}"))

    def _close(self) -> None:
        if self.recorder is not None:
            self._rec_file.close()
        if self.link:
            try:
                self.link.send(link.CMD_STREAM_STOP)
            except Exception:  # noqa: BLE001
                pass
            self.link.close()
        self.after(200, self.destroy)


if __name__ == "__main__":
    App().mainloop()
