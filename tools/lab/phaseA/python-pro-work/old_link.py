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

TYPE_CMD, TYPE_RSP, TYPE_EVT, TYPE_DATA, TYPE_IMU = 0x01, 0x02, 0x03, 0x04, 0x05

# Frame flags. SETTLING marks samples still inside a filter's settling time.
FLAG_SETTLING, FLAG_OVERRUN = 0x01, 0x02

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
CMD_SET_IMU = 0x0F
CMD_SET_FILTER = 0x10
CMD_SET_CAR = 0x11
CMD_RESET_CHAIN = 0x12

# Status byte of a response.
STATUS_OK, STATUS_BADARG, STATUS_FAILED, STATUS_UNKNOWN = 0, 1, 2, 3

# Unsolicited events. The device measures the mains frequency and says so.
EVT_MAINS = 0x01

# The mains notch's flags, in CMD_SET_NOTCH and GET_CONFIG.
NOTCH_HARMONIC, NOTCH_TRACK, NOTCH_MEASURED = 0x01, 0x02, 0x04

# Raw counts in 32 or 24 bits; microvolts after the device's chain; or both
# side by side for every channel, so a host can record the one and show the
# other.
ENC_RAW_I32, ENC_UV_F32, ENC_RAW_I24, ENC_RAW_UV = 0, 1, 2, 3

MUX_NORMAL, MUX_SHORTED, MUX_TEST = 0x00, 0x01, 0x05

GAIN_CODES = {1: 0, 2: 1, 4: 2, 6: 3, 8: 4, 12: 5, 24: 6}
GAIN_FROM_CODE = {v: k for k, v in GAIN_CODES.items()}

CHANNELS = 8
DATA_HDR = struct.Struct("<QIBBH")

# The device chain has two stages of sections, either side of its common
# average reference.
STAGE_PRE, STAGE_POST = 0, 1
MAX_SECTIONS = 8

# Motion sensor: ts, seq, period in 1/256 us, accel g, gyro dps, axes,
# flags, count. Rates and ranges are the ones the LSM6DSV16X offers.
IMU_HDR = struct.Struct("<QIIHHBBH")
IMU_RATES = (60, 120, 240, 480, 960)
IMU_ACCEL_G = (2, 4, 8, 16)
IMU_GYRO_DPS = (125, 250, 500, 1000, 2000, 4000)
IMU_FLAG_TIME_ESTIMATED, IMU_FLAG_OVERRUN = 0x01, 0x02


def decode_imu(payload: bytes):
    """
    An IMU payload to (ts_us, seq, period_us, counts, accel_g, gyro_dps, flags).

    counts is the raw (samples, 6) int16 block - accel x y z, then gyro x y
    z. One count is accel_g / 32768 g, or gyro_dps / 32768 degrees a second.
    """
    if len(payload) < IMU_HDR.size:
        return None

    ts, seq, period_q8, accel_g, gyro_dps, axes, flags, count = \
        IMU_HDR.unpack_from(payload)
    need = count * axes * 2
    body = payload[IMU_HDR.size:IMU_HDR.size + need]

    if axes != 6 or len(body) < need:
        return None

    counts = np.frombuffer(body, dtype="<i2").reshape(count, 6)
    return ts, seq, period_q8 / 256.0, counts, accel_g, gyro_dps, flags

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

# Commands go out as writes without a response. Tried the other way, under a
# 1000 SPS stream: acknowledged writes doubled the reply time, 36 to 73 ms
# median over 252 commands, and replies still went missing - so what loses
# them is not the write.
BLE_WRITE_WITH_RESPONSE = False


def _i24(raw: np.ndarray) -> np.ndarray:
    """Little-endian 24-bit two's complement, three bytes on the last axis."""
    a = raw.astype(np.int32)
    v = a[..., 0] | (a[..., 1] << 8) | (a[..., 2] << 16)
    return np.where(v & 0x800000, v - (1 << 24), v)


def decode_data(payload: bytes):
    """
    A DATA payload to (ts_us, seq, encoding, counts, uv).

    counts is (samples, channels) of raw converter counts and uv the device
    chain's microvolts, the same shape; either is None when the encoding does
    not carry it.
    """
    ts, seq, ch, enc, count = DATA_HDR.unpack_from(payload)
    width = {ENC_RAW_I24: 3, ENC_RAW_UV: 7}.get(enc, 4)
    need = count * ch * width
    body = payload[DATA_HDR.size:DATA_HDR.size + need]

    if len(body) < need:
        return None

    raw = np.frombuffer(body, dtype=np.uint8)

    if enc == ENC_RAW_I24:
        return ts, seq, enc, _i24(raw.reshape(count, ch, 3)), None
    if enc == ENC_RAW_UV:
        rec = raw.reshape(count, ch, 7)
        uv = np.ascontiguousarray(rec[:, :, 3:]).view("<f4").reshape(count, ch)
        return ts, seq, enc, _i24(rec[:, :, :3]), uv
    if enc == ENC_UV_F32:
        return ts, seq, enc, None, np.frombuffer(body, dtype="<f4").reshape(count, ch)
    return ts, seq, enc, np.frombuffer(body, dtype="<i4").reshape(count, ch), None


def pack_sections(sections) -> bytes:
    """Sections as they travel: g, k, m0, m1, m2, each little-endian float32."""
    return b"".join(struct.pack("<5f", *s) for s in sections)


def filter_args(stage: int, sections, fs: float,
                keep_state: bool = False) -> list[int]:
    """
    Arguments for CMD_SET_FILTER.

    `fs` is the rate the sections were designed for. The device refuses them
    at any other rate: a filter designed for one rate is a different filter
    at another.
    """
    if len(sections) > MAX_SECTIONS:
        raise ValueError(f"at most {MAX_SECTIONS} sections per stage")
    fs = int(round(fs))
    return [stage, 1 if keep_state else 0, fs & 0xFF, fs >> 8, len(sections),
            *pack_sections(sections)]


def sections_crc(sections) -> int:
    """The CRC the device reports for a stage holding these sections."""
    return proto_ref.crc16(pack_sections(sections))


def notch_args(hz: int, q: int, harmonic: bool, track: bool) -> list[int]:
    """
    Arguments for CMD_SET_NOTCH: the nominal frequency (0, 50 or 60), Q, and
    whether to notch the harmonic and follow the mains the device measures.
    """
    return [int(hz), int(q),
            (NOTCH_HARMONIC if harmonic else 0) | (NOTCH_TRACK if track else 0)]


def decode_event(payload: bytes) -> dict | None:
    """
    An EVT payload as a dict, or None for one this host does not know.

    EVT_MAINS: the mains frequency the device agreed on, the first sample
    processed after it, and whether its notch moved there.
    """
    if len(payload) >= 10 and payload[0] == EVT_MAINS:
        seq, hz = struct.unpack_from("<If", payload, 2)
        return {"event": "mains", "hz": hz, "seq": seq,
                "moved": bool(payload[1] & 0x01)}
    return None


def response_seq(payload: bytes) -> int | None:
    """The sample a filter change applies from, read from its OK response."""
    if len(payload) >= 6 and payload[1] == STATUS_OK:
        return struct.unpack_from("<I", payload, 2)[0]
    return None


def decode_config(p: bytes) -> dict | None:
    """
    A CMD_GET_CONFIG response - opcode, status, then the state - as a dict.
    Older firmware sends less, and only what arrived is filled in.
    """
    if len(p) < 7 or p[0] != CMD_GET_CONFIG or p[1] != STATUS_OK:
        return None

    cfg = {"channels": p[2], "encoding": p[3], "rate": p[4] | (p[5] << 8),
           "notch": p[6]}

    if len(p) >= 15:
        chset = bytes(p[7:15])
        cfg["chset"] = chset
        cfg["gains"] = [GAIN_FROM_CODE.get((c >> 4) & 0x07) for c in chset]
        cfg["mux"] = [c & 0x07 for c in chset]

    if len(p) >= 21:
        cfg["imu_on"] = bool(p[15] & 0x01)
        cfg["imu_fitted"] = bool(p[15] & 0x02)
        cfg["imu_rate"] = p[16] | (p[17] << 8)
        cfg["imu_accel_g"] = p[18]
        cfg["imu_gyro_dps"] = p[19] | (p[20] << 8)

    if len(p) >= 29:
        cfg["pre_count"] = p[21]
        cfg["post_count"] = p[22]
        cfg["car"] = bool(p[23] & 0x01)
        cfg["car_mask"] = p[24]
        cfg["pre_crc"] = p[25] | (p[26] << 8)
        cfg["post_crc"] = p[27] | (p[28] << 8)

    if len(p) >= 39:
        flags = p[30]
        measured, aim = struct.unpack_from("<ff", p, 31)
        cfg["notch_q"] = p[29]
        cfg["notch_harmonic"] = bool(flags & NOTCH_HARMONIC)
        cfg["notch_track"] = bool(flags & NOTCH_TRACK)
        cfg["mains_hz"] = measured if flags & NOTCH_MEASURED else None
        cfg["notch_aim_hz"] = aim

    return cfg


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
            self._take_notification(bytes(data))

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
                    await c.write_gatt_char(UUID_CONTROL, frame,
                                            response=BLE_WRITE_WITH_RESPONSE)
                except Exception as exc:  # noqa: BLE001
                    self.status.put(f"BLE write failed: {exc}")
                    break

            self.connected = False
            if dropped.is_set():
                self.status.put("BLE link dropped")

    def _take_notification(self, data: bytes) -> None:
        """
        One notification's frames. The device sends whole frames, one to a
        notification, so bytes left over can only be a damaged frame - and
        kept, they would take the next notification, a reply perhaps, as
        their missing part. Both characteristics come through here.
        """
        for f in self.parser.feed(data):
            self.frames.put(f)
        if self.parser.buf:
            self.parser.buf.clear()
            self.parser.bad += 1

    def send(self, opcode: int, *args: int) -> None:
        self._out.put(self._encode(opcode, args))

    def close(self) -> None:
        self._stop.set()
        self.connected = False
