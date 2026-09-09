"""
Long-run stability check over BLE.

Streams for a while and watches for the things that only show up over time:
dropped connections, sequence gaps, CRC failures, and rate drift. Prints a
line a minute so a run can be watched, and exits non-zero if anything broke.

    python tools/soak.py --minutes 30 --sps 1000
"""

from __future__ import annotations

import argparse
import asyncio
import pathlib
import struct
import sys
import time

sys.path.insert(0, pathlib.Path(__file__).resolve().parent.as_posix())
import proto_ref  # noqa: E402
from bleak import BleakClient, BleakScanner  # noqa: E402

NODE = "535749465445"
CTRL = f"57724502-4700-4000-8000-{NODE}"
STRM = f"57724503-4700-4000-8000-{NODE}"

CMD_STREAM_START, CMD_STREAM_STOP, CMD_SET_RATE = 0x02, 0x03, 0x09

state = {
    "samples": 0, "frames": 0, "bad": 0, "gaps": 0,
    "bytes": 0, "next_seq": None, "disconnects": 0,
}
buf = bytearray()


def on_stream(_, data: bytearray) -> None:
    state["bytes"] += len(data)
    buf.extend(data)

    while True:
        i = buf.find(proto_ref.SOF)
        if i < 0:
            buf.clear()
            return
        if i:
            del buf[:i]
        if len(buf) < proto_ref.HEADER_LEN:
            return

        length = struct.unpack_from("<H", buf, 4)[0]
        total = proto_ref.HEADER_LEN + length + proto_ref.CRC_LEN
        if len(buf) < total:
            return

        raw = bytes(buf[:total])
        del buf[:total]

        try:
            f = proto_ref.decode(raw)
        except Exception:  # noqa: BLE001 - a bad frame is a datum
            state["bad"] += 1
            continue

        if f.type != 0x04:
            continue

        state["frames"] += 1
        _, seq, _, _, count = struct.unpack_from("<QIBBH", f.payload)

        if state["next_seq"] is not None and seq != state["next_seq"]:
            state["gaps"] += 1
        state["next_seq"] = seq + count
        state["samples"] += count


async def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--minutes", type=float, default=30.0)
    ap.add_argument("--sps", type=int, default=1000)
    args = ap.parse_args()

    dev = await BleakScanner.find_device_by_name("SwiftEEG", timeout=20.0)
    if dev is None:
        print("SwiftEEG not found", file=sys.stderr)
        return 2

    print(f"soak: {args.minutes:.0f} min at {args.sps} SPS on {dev.address}")

    disconnected = asyncio.Event()

    async with BleakClient(dev, disconnected_callback=lambda _: disconnected.set()) as c:
        await c.start_notify(STRM, on_stream)
        await c.write_gatt_char(
            CTRL, proto_ref.encode(0x01, 0, 0,
                                   bytes([CMD_SET_RATE, args.sps & 0xFF,
                                          args.sps >> 8])), response=False)
        await asyncio.sleep(2.0)
        await c.write_gatt_char(
            CTRL, proto_ref.encode(0x01, 0, 1, bytes([CMD_STREAM_START])),
            response=False)

        start = time.time()
        end = start + args.minutes * 60.0
        mark = start

        while time.time() < end:
            if disconnected.is_set():
                state["disconnects"] += 1
                print("  LINK DROPPED", file=sys.stderr)
                break

            await asyncio.sleep(1.0)

            now = time.time()
            if now - mark >= 60.0:
                el = now - start
                print(f"  {el / 60:5.1f} min  {state['samples'] / el:7.1f} SPS  "
                      f"{state['bytes'] / el / 1024:5.1f} kB/s  "
                      f"gaps {state['gaps']}  bad {state['bad']}")
                mark = now

        elapsed = time.time() - start
        try:
            await c.write_gatt_char(
                CTRL, proto_ref.encode(0x01, 0, 2, bytes([CMD_STREAM_STOP])),
                response=False)
        except Exception:  # noqa: BLE001 - link may already be gone
            pass

    ok = (state["gaps"] == 0 and state["bad"] == 0
          and state["disconnects"] == 0 and elapsed > args.minutes * 55.0)

    print(f"\n{elapsed / 60:.1f} min, {state['samples']} samples "
          f"({state['samples'] / elapsed:.1f} SPS), {state['frames']} frames")
    print(f"gaps {state['gaps']}, bad CRC {state['bad']}, "
          f"disconnects {state['disconnects']}")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
