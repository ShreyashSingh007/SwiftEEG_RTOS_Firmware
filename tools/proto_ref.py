"""
Reference implementation of the SwiftEEG binary protocol framing layer.

This is the oracle. The C codec in src/proto must agree with it byte for
byte; golden vectors generated here are what the on-target tests check
against. It is also the starting point for the host library that the GUIs
and the LSL / BrainFlow bridges will use.

Keep this file and src/proto/proto.h in step. If the frame layout changes,
change it here first, regenerate the vectors, and let the C tests fail
loudly.
"""

from __future__ import annotations

import binascii
import struct
from dataclasses import dataclass

SOF = 0xA5
VERSION = 1
HEADER_LEN = 8
CRC_LEN = 2
OVERHEAD = HEADER_LEN + CRC_LEN
MAX_PAYLOAD = 1024

TYPE_CMD = 0x01
TYPE_RSP = 0x02
TYPE_EVT = 0x03
TYPE_DATA = 0x04
VALID_TYPES = (TYPE_CMD, TYPE_RSP, TYPE_EVT, TYPE_DATA)

FLAG_NONE = 0x00
FLAG_SETTLING = 0x01
FLAG_OVERRUN = 0x02


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout."""
    # binascii's CRC-CCITT is exactly this CRC, in C. The bit-by-bit loop
    # below ran on the thread that receives Bluetooth, and at 1 kSPS it was
    # competing with the plot for the interpreter.
    return binascii.crc_hqx(data, 0xFFFF)


def _crc16_bitwise(data: bytes) -> int:
    """The same CRC spelled out bit by bit - the reference crc16 is checked against."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Frame:
    type: int
    flags: int
    seq: int
    payload: bytes


def encode(type_: int, flags: int, seq: int, payload: bytes = b"") -> bytes:
    if type_ not in VALID_TYPES:
        raise ValueError(f"bad type 0x{type_:02x}")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too large: {len(payload)}")

    head = struct.pack("<BBBBHH", SOF, VERSION, type_, flags, len(payload), seq)
    body = head + payload
    # CRC covers everything except the SOF byte.
    return body + struct.pack("<H", crc16(body[1:]))


def decode(buf: bytes) -> Frame:
    if len(buf) < OVERHEAD:
        raise ValueError("short frame")
    if buf[0] != SOF:
        raise ValueError("bad SOF")
    if buf[1] != VERSION:
        raise ValueError(f"bad version {buf[1]}")
    if buf[2] not in VALID_TYPES:
        raise ValueError(f"bad type 0x{buf[2]:02x}")

    payload_len = struct.unpack_from("<H", buf, 4)[0]
    if payload_len > MAX_PAYLOAD:
        raise ValueError("payload too large")
    if len(buf) < OVERHEAD + payload_len:
        raise ValueError("truncated frame")

    want = crc16(buf[1:HEADER_LEN + payload_len])
    got = struct.unpack_from("<H", buf, HEADER_LEN + payload_len)[0]
    if want != got:
        raise ValueError(f"crc mismatch: want 0x{want:04x} got 0x{got:04x}")

    seq = struct.unpack_from("<H", buf, 6)[0]
    return Frame(buf[2], buf[3], seq, bytes(buf[HEADER_LEN:HEADER_LEN + payload_len]))


def _self_test() -> None:
    # Standard check value for CRC-16/CCITT-FALSE.
    assert crc16(b"123456789") == 0x29B1, f"got 0x{crc16(b'123456789'):04x}"
    for n in (0, 1, 2, 7, 64, 255, 300):
        data = bytes((i * 37 + n) & 0xFF for i in range(n))
        assert crc16(data) == _crc16_bitwise(data), f"fast CRC differs at {n} bytes"

    # Round trip, including an empty payload and a maximum-size one.
    for payload in (b"", b"\x00", b"hello", bytes(range(256)) * 4):
        for type_ in VALID_TYPES:
            f = decode(encode(type_, FLAG_NONE, 0x1234, payload))
            assert f.type == type_ and f.seq == 0x1234 and f.payload == payload

    # A single flipped bit must be caught.
    good = bytearray(encode(TYPE_DATA, FLAG_NONE, 1, b"abcd"))
    good[HEADER_LEN] ^= 0x01
    try:
        decode(bytes(good))
    except ValueError as exc:
        assert "crc" in str(exc)
    else:
        raise AssertionError("corrupted frame decoded cleanly")

    print("proto_ref self-test: OK")


if __name__ == "__main__":
    _self_test()
