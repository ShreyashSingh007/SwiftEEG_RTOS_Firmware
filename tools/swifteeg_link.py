"""
Device links: USB and BLE behind one interface.

The wire format is identical on both, so everything above this file is
transport-agnostic. Both links deliver decoded protocol frames to a queue and
accept command frames; the difference is only in how bytes move.

BLE runs bleak, which is async, on its own thread with its own event loop.
The UI thread never awaits anything - it puts commands on a queue and reads
frames off another.
"""

from __future__ import annotations

import asyncio
import pathlib
import queue
import struct
import sys
import threading
import time

import numpy as np

sys.path.insert(0, pathlib.Path(__file__).resolve().parent.as_posix())
import proto_ref  # noqa: E402

# --- protocol -------------------------------------------------------------

TYPE_CMD, TYPE_RSP, TYPE_EVT, TYPE_DATA = 0x01, 0x02, 0x03, 0x04

CMD_PING = 0x01
CMD_STREAM_START = 0x02
CMD_STREAM_STOP = 0x03
CMD_SET_ENCODING = 0x04
CMD_TEST_SIGNAL = 0x05
CMD_GET_INFO = 0x06
CMD_READ_REG = 0x07
CMD_SET_INPUT = 0x08
CMD_SET_RATE = 0x09
CMD_SET_CHANNEL = 0x0A
CMD_SET_BIAS = 0x0B
CMD_SET_NOTCH = 0x0C
CMD_SET_LEADOFF = 0x0D
CMD_GET_CONFIG = 0x0E

ENC_RAW_I32, ENC_UV_F32, ENC_RAW_I24 = 0, 1, 2

MUX_NORMAL, MUX_SHORTED, MUX_TEST = 0x00, 0x01, 0x05

GAIN_CODES = {1: 0, 2: 1, 4: 2, 6: 3, 8: 4, 12: 5, 24: 6}
GAIN_FROM_CODE = {v: k for k, v in GAIN_CODES.items()}

CHANNELS = 8
DATA_HDR = struct.Struct("<QIBBH")

# VREF 4.5 V over a 24-bit converter. Gain is divided out per channel.
def lsb_uv(gain: int = 24) -> float:
    return (2.0 * 4.5) / (gain * (1 << 24)) * 1e6


# BLE identifiers. The node spells "SWIFTE".
_NODE = "535749465445"
UUID_CONTROL = f"57724502-4700-4000-8000-{_NODE}"
UUID_STREAM = f"57724503-4700-4000-8000-{_NODE}"
UUID_EVENT = f"57724504-4700-4000-8000-{_NODE}"

DEVICE_NAME = "SwiftEEG"
USB_VID, USB_PID = 0x2FE3, 0x0001


def decode_data(payload: bytes):
    """A DATA payload to (ts_us, seq, counts) with counts as (samples, ch)."""
    ts, seq, ch, enc, count = DATA_HDR.unpack_from(payload)
    width = 3 if enc == ENC_RAW_I24 else 4
    need = count * ch * width
    body = payload[DATA_HDR.size:DATA_HDR.size + need]

    if len(body) < need:
        return None

    if enc == ENC_RAW_I24:
        # Little-endian 24-bit two's complement, three bytes per channel.
        a = np.frombuffer(body, dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        v = a[:, 0] | (a[:, 1] << 8) | (a[:, 2] << 16)
        v = np.where(v & 0x800000, v - (1 << 24), v)
        vals = v.reshape(count, ch)
    elif enc == ENC_UV_F32:
        vals = np.frombuffer(body, dtype="<f4").reshape(count, ch)
    else:
        vals = np.frombuffer(body, dtype="<i4").reshape(count, ch)

    return ts, seq, enc, vals


class FrameParser:
    """Pulls whole protocol frames out of a byte stream, resyncing on junk."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []

        while True:
            i = self.buf.find(proto_ref.SOF)
            if i < 0:
                self.buf.clear()
                return out
            if i:
                del self.buf[:i]
            if len(self.buf) < proto_ref.HEADER_LEN:
                return out

            length = struct.unpack_from("<H", self.buf, 4)[0]
            total = proto_ref.HEADER_LEN + length + proto_ref.CRC_LEN
            if len(self.buf) < total:
                return out

            raw = bytes(self.buf[:total])
            del self.buf[:total]

            try:
                out.append(proto_ref.decode(raw))
            except Exception:  # noqa: BLE001 - a bad frame is data, not a crash
                self.bad += 1

        return out


# --- links ----------------------------------------------------------------

class Link:
    """Common surface. `frames` carries decoded frames to the UI thread."""

    def __init__(self) -> None:
        self.frames: queue.Queue = queue.Queue()
        self.status: queue.Queue = queue.Queue()
        self.parser = FrameParser()
        self.connected = False
        self._seq = 0

    def _encode(self, opcode: int, args) -> bytes:
        self._seq = (self._seq + 1) & 0xFFFF
        return proto_ref.encode(TYPE_CMD, 0, self._seq, bytes([opcode, *args]))

    def send(self, opcode: int, *args: int) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    @property
    def bad_frames(self) -> int:
        return self.parser.bad


class UsbLink(Link):
    """CDC ACM. Fast, wired, and the only option above 1 kSPS."""

    def __init__(self, port: str | None = None):
        super().__init__()
        import serial
        import serial.tools.list_ports

        if port is None:
            port = next((p.device for p in serial.tools.list_ports.comports()
                         if p.vid == USB_VID and p.pid == USB_PID), None)
        if port is None:
            raise RuntimeError("no SwiftEEG on USB")

        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.port = port
        self.connected = True
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()
        self.status.put(f"USB {port}")

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.ser.read(8192)
            except Exception as exc:  # noqa: BLE001
                self.status.put(f"USB read failed: {exc}")
                self.connected = False
                return
            if data:
                for f in self.parser.feed(data):
                    self.frames.put(f)

    def send(self, opcode: int, *args: int) -> None:
        if not self.connected:
            return
        try:
            self.ser.write(self._encode(opcode, args))
        except Exception as exc:  # noqa: BLE001
            self.status.put(f"USB write failed: {exc}")
            self.connected = False

    def close(self) -> None:
        self._stop.set()
        self.connected = False
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass


class BleLink(Link):
    """
    Bluetooth LE, via bleak on a dedicated thread.

    bleak is asyncio and tkinter is not, so the event loop lives here and the
    two sides talk over queues. Nothing above this class awaits anything.
    """

    def __init__(self, address: str | None = None):
        super().__init__()
        self.address = address
        self._out: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._thread, daemon=True)
        self._t.start()

    def _thread(self) -> None:
        try:
            asyncio.run(self._main())
        except Exception as exc:  # noqa: BLE001 - surfaced in the UI
            self.status.put(f"BLE failed: {exc}")
        finally:
            self.connected = False

    async def _main(self) -> None:
        from bleak import BleakClient, BleakScanner

        self.status.put("scanning for SwiftEEG...")
        dev = (self.address if self.address else
               await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=20.0))

        if dev is None:
            self.status.put("SwiftEEG not found - is it powered?")
            return

        def on_notify(_, data: bytearray) -> None:
            for f in self.parser.feed(bytes(data)):
                self.frames.put(f)

        dropped = asyncio.Event()

        async with BleakClient(dev, disconnected_callback=lambda _: dropped.set()) as c:
            await c.start_notify(UUID_STREAM, on_notify)
            await c.start_notify(UUID_EVENT, on_notify)
            self.connected = True
            self.status.put(f"BLE connected (MTU {c.mtu_size})")

            while not self._stop.is_set() and not dropped.is_set():
                try:
                    frame = self._out.get_nowait()
                except queue.Empty:
                    await asyncio.sleep(0.01)
                    continue

                try:
                    await c.write_gatt_char(UUID_CONTROL, frame, response=False)
                except Exception as exc:  # noqa: BLE001
                    self.status.put(f"BLE write failed: {exc}")
                    break

            self.connected = False
            if dropped.is_set():
                self.status.put("BLE link dropped")

    def send(self, opcode: int, *args: int) -> None:
        self._out.put(self._encode(opcode, args))

    def close(self) -> None:
        self._stop.set()
        self.connected = False
