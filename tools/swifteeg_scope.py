"""
SwiftEEG live scope.

Opens the device's USB serial port, decodes the binary protocol, and draws
the eight channels. Also drives the ADS1299's own test generator, which is
what turns "the trace moves" into "the trace is correct".

    python tools/swifteeg_scope.py            # find the port automatically
    python tools/swifteeg_scope.py --port COM4

Only needs pyserial. The window is tkinter, which is part of the standard
library and native on Windows, so there is nothing to install.

Protocol decoding is imported from proto_ref.py rather than written again -
that module is the oracle the firmware's own tests are checked against, so
if the two ever disagree this viewer disagrees too, loudly.
"""

from __future__ import annotations

import argparse
import collections
import queue
import struct
import sys
import threading
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

sys.path.insert(0, __import__("pathlib").Path(__file__).resolve().parent.as_posix())
import proto_ref  # noqa: E402

# --- protocol constants, mirroring the firmware ---------------------------

TYPE_CMD, TYPE_RSP, TYPE_EVT, TYPE_DATA = 0x01, 0x02, 0x03, 0x04

CMD_PING = 0x01
CMD_STREAM_START = 0x02
CMD_STREAM_STOP = 0x03
CMD_SET_ENCODING = 0x04
CMD_TEST_SIGNAL = 0x05

ENC_RAW_I32 = 0
ENC_UV_F32 = 1

DATA_HDR = struct.Struct("<QIBBH")  # ts_us, seq, channels, encoding, count

CHANNELS = 8

# Board constants. VREF 4.5 V at gain 24 over a 24-bit converter.
LSB_UV = (2.0 * 4.5) / (24 * (1 << 24)) * 1e6

# The internal generator swings +/-1.875 mV at the inputs, at ~0.98 Hz.
CAL_AMPLITUDE_UV = 4.5 / 2400 * 1e6
CAL_FREQ_HZ = 2.048e6 / (1 << 21)

# Electrode sites, in the order the channels are wired.
SITES = ["C4", "P4", "F4", "Oz", "AFz", "F3", "C3", "P3"]

# Distinct hues that stay legible against a dark plot.
COLORS = [
    "#4ea1ff", "#ffb454", "#5ed18b", "#ff6b6b",
    "#c792ea", "#ffd866", "#78dce8", "#ff9ec4",
]

WINDOW_SECONDS = 4.0
NOMINAL_SPS = 250


def find_port() -> str | None:
    """The firmware's VID/PID, currently Zephyr's test IDs."""
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2FE3 and p.pid == 0x0001:
            return p.device
    return None


class Reader(threading.Thread):
    """Reads the port, decodes frames, and posts samples to the UI."""

    def __init__(self, port: str, out: queue.Queue):
        super().__init__(daemon=True)
        self.port = port
        self.out = out
        self.ser: serial.Serial | None = None
        self.running = True
        self.buf = bytearray()
        self.frames = 0
        self.bad_crc = 0
        self.encoding = ENC_RAW_I32

    def open(self) -> None:
        # Baud is ignored by CDC ACM but pyserial insists on one.
        self.ser = serial.Serial(self.port, 115200, timeout=0.05)

    def send(self, opcode: int, *args: int) -> None:
        if self.ser is None:
            return
        payload = bytes([opcode, *args])
        self.ser.write(proto_ref.encode(TYPE_CMD, 0, 0, payload))

    def run(self) -> None:
        try:
            self.open()
        except Exception as exc:  # noqa: BLE001 - surfaced in the UI
            self.out.put(("error", f"cannot open {self.port}: {exc}"))
            return

        self.out.put(("status", f"connected to {self.port}"))
        self.send(CMD_SET_ENCODING, ENC_RAW_I32)
        self.send(CMD_STREAM_START)

        while self.running:
            try:
                chunk = self.ser.read(4096)
            except Exception as exc:  # noqa: BLE001
                self.out.put(("error", f"read failed: {exc}"))
                return

            if chunk:
                self.buf.extend(chunk)
                self._drain()

        try:
            self.send(CMD_STREAM_STOP)
            self.ser.close()
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass

    def _drain(self) -> None:
        """Pull whole frames out of the byte stream, resyncing on garbage."""
        while True:
            start = self.buf.find(proto_ref.SOF)
            if start < 0:
                self.buf.clear()
                return
            if start:
                del self.buf[:start]
            if len(self.buf) < proto_ref.HEADER_LEN:
                return

            length = struct.unpack_from("<H", self.buf, 4)[0]
            total = proto_ref.HEADER_LEN + length + proto_ref.CRC_LEN
            if len(self.buf) < total:
                return

            raw = bytes(self.buf[:total])
            del self.buf[:total]

            try:
                frame = proto_ref.decode(raw)
            except Exception:  # noqa: BLE001 - a bad frame is data, not a crash
                self.bad_crc += 1
                continue

            self.frames += 1
            if frame.type == TYPE_DATA:
                self._on_data(frame.payload)

    def _on_data(self, payload: bytes) -> None:
        if len(payload) < DATA_HDR.size:
            return

        ts_us, seq, channels, encoding, count = DATA_HDR.unpack_from(payload)
        self.encoding = encoding

        body = payload[DATA_HDR.size:]
        need = count * channels * 4
        if len(body) < need:
            return

        if encoding == ENC_UV_F32:
            vals = struct.unpack(f"<{count * channels}f", body[:need])
            uv = list(vals)
        else:
            vals = struct.unpack(f"<{count * channels}i", body[:need])
            uv = [v * LSB_UV for v in vals]

        block = [uv[i * channels:(i + 1) * channels] for i in range(count)]
        self.out.put(("data", (ts_us, seq, block)))

    def stop(self) -> None:
        self.running = False


class Scope(tk.Tk):
    def __init__(self, port: str):
        super().__init__()
        self.title("SwiftEEG - live scope")
        self.geometry("1180x760")
        self.configure(bg="#14161a")

        self.q: queue.Queue = queue.Queue()
        self.reader = Reader(port, self.q)

        depth = int(WINDOW_SECONDS * NOMINAL_SPS)
        self.traces = [collections.deque([0.0] * depth, maxlen=depth)
                       for _ in range(CHANNELS)]

        self.samples = 0
        self.test_on = tk.BooleanVar(value=False)
        self.autoscale = tk.BooleanVar(value=True)
        self.span_uv = 4000.0

        self._build_ui()
        self.reader.start()
        self.after(33, self._tick)
        self.protocol("WM_DELETE_WINDOW", self._close)

    # -- layout ------------------------------------------------------------

    def _build_ui(self) -> None:
        bar = tk.Frame(self, bg="#1c1f26", pady=8, padx=10)
        bar.pack(fill=tk.X)

        tk.Checkbutton(
            bar, text="ADS1299 test signal", variable=self.test_on,
            command=self._toggle_test, bg="#1c1f26", fg="#e6e6e6",
            selectcolor="#2a2f3a", activebackground="#1c1f26",
            activeforeground="#ffffff", highlightthickness=0,
        ).pack(side=tk.LEFT, padx=(0, 16))

        tk.Checkbutton(
            bar, text="auto scale", variable=self.autoscale,
            bg="#1c1f26", fg="#e6e6e6", selectcolor="#2a2f3a",
            activebackground="#1c1f26", activeforeground="#ffffff",
            highlightthickness=0,
        ).pack(side=tk.LEFT, padx=(0, 16))

        self.expect = tk.Label(
            bar, text="", bg="#1c1f26", fg="#ffb454",
            font=("Consolas", 9),
        )
        self.expect.pack(side=tk.LEFT, padx=(0, 16))

        self.status = tk.Label(
            bar, text="connecting...", bg="#1c1f26", fg="#9aa4b2",
            font=("Consolas", 9),
        )
        self.status.pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(self, bg="#0f1115", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

    # -- control -----------------------------------------------------------

    def _toggle_test(self) -> None:
        on = 1 if self.test_on.get() else 0
        self.reader.send(CMD_TEST_SIGNAL, on, 0)
        if on:
            self.expect.config(
                text=f"expect ~{CAL_AMPLITUDE_UV:,.0f} uV p-p square "
                     f"at ~{CAL_FREQ_HZ:.2f} Hz"
            )
        else:
            self.expect.config(text="")
        # The old window is a different signal; do not average across it.
        for t in self.traces:
            t.clear()
            t.extend([0.0] * t.maxlen)

    def _close(self) -> None:
        self.reader.stop()
        self.after(150, self.destroy)

    # -- update loop -------------------------------------------------------

    def _tick(self) -> None:
        got = 0
        while True:
            try:
                kind, item = self.q.get_nowait()
            except queue.Empty:
                break

            if kind == "data":
                _, _, block = item
                for row in block:
                    for ch in range(CHANNELS):
                        self.traces[ch].append(row[ch])
                    got += 1
            elif kind == "status":
                self.status.config(text=item, fg="#5ed18b")
            elif kind == "error":
                self.status.config(text=item, fg="#ff6b6b")

        self.samples += got
        self._draw()
        self.after(33, self._tick)

    def _draw(self) -> None:
        c = self.canvas
        c.delete("all")

        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        left = 96
        plot_w = w - left - 20
        lane_h = h / CHANNELS

        if self.autoscale:
            peak = max((max(abs(v) for v in t) for t in self.traces), default=0.0)
            # Round up to something stable so the scale does not jitter.
            self.span_uv = max(20.0, peak * 2.2)

        half = self.span_uv / 2.0

        for ch in range(CHANNELS):
            base = lane_h * (ch + 0.5)

            c.create_line(left, base, w - 20, base, fill="#1e222b")
            c.create_text(
                left - 12, base, anchor="e", fill=COLORS[ch],
                text=f"CH{ch + 1} {SITES[ch]}", font=("Consolas", 10, "bold"),
            )

            trace = self.traces[ch]
            n = len(trace)
            if n < 2:
                continue

            step = plot_w / (n - 1)
            scale = (lane_h * 0.42) / half

            pts = []
            for i, v in enumerate(trace):
                y = base - max(-half, min(half, v)) * scale
                pts.extend((left + i * step, y))

            c.create_line(*pts, fill=COLORS[ch], width=1)

        # Scale bar, so the trace height means something.
        c.create_text(
            left - 12, 14, anchor="e", fill="#9aa4b2",
            font=("Consolas", 9),
            text=f"+/-{half:,.0f} uV",
        )

        rate = ""
        if self.reader.frames:
            rate = (f"  frames {self.reader.frames}"
                    f"  bad {self.reader.bad_crc}"
                    f"  samples {self.samples}")
        c.create_text(
            w - 24, 14, anchor="e", fill="#5a6472", font=("Consolas", 9),
            text=f"{WINDOW_SECONDS:.0f} s window{rate}",
        )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. COM4")
    args = ap.parse_args()

    port = args.port or find_port()
    if port is None:
        print("No SwiftEEG found (looked for VID 2FE3 PID 0001).",
              file=sys.stderr)
        print("Ports seen:", file=sys.stderr)
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}", file=sys.stderr)
        return 1

    Scope(port).mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
