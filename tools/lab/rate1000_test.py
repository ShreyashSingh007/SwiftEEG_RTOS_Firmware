"""
Why does 1000 SPS lose samples, when 250 and 500 do not?

On the worn session the device produced about 2.8 % fewer samples than it
should at 1000 SPS: 40 short stalls in 40 s, the sequence numbers unbroken
across them and no overrun flag, so only the hardware timestamps show it.

Every sample the firmware captures is counted in one of three places
(src/pipeline/pipeline.c): frames captured by the transfer-complete
interrupt, frames the DSP ring dropped, and frames rejected for a wrong
status marker - the last un-counts its sequence number, which is exactly
this signature. The health line in the device log prints all three, so this
runs the rate under four conditions while that log is captured alongside.

Nothing needs to be worn: this measures timing, not signal.

    python -u rate1000_test.py --out DIR [--seconds 20]
"""
from __future__ import annotations

import argparse
import pathlib
import queue
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
TOOLS = r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools"
sys.path.insert(0, str(HERE))
sys.path.insert(0, TOOLS)
import swifteeg_link as L  # noqa: E402
from worn_session import (Fail, Lost, Recorder, change_rate, describe,  # noqa: E402
                          jsonable)

IMU_RATE, IMU_G, IMU_DPS = 240, 8, 2000


def stalls(rec: Recorder, rate: int) -> dict:
    """What the device's own timestamps say about samples never produced."""
    ts = np.array([e[2] for e in rec.eeg], dtype=np.float64)
    seq = np.array([e[3] for e in rec.eeg], dtype=np.float64)
    ln = np.array([len(e[4]) if e[4] is not None else len(e[5]) for e in rec.eeg])
    if len(ts) < 10:
        return {"frames": len(ts)}
    step = np.diff(ts) / np.diff(seq)
    good = float(np.median(step))
    bad = np.flatnonzero(step > 1.5 * good)
    lost = float(np.sum(np.diff(ts)[bad] / good - np.diff(seq)[bad]))
    samples = int(ln.sum())
    return {
        "frames": len(ts), "samples": samples, "stalls": len(bad),
        "lost": lost, "lost_pct": 100 * lost / max(samples + lost, 1),
        "period_us": good, "sps_true": 1e6 / good,
        "sps_overall": 1e6 / float(np.polyfit(seq, ts, 1)[0]),
        "worst_ms": float(np.max(np.diff(ts)[bad]) / 1000) if len(bad) else 0.0,
        "flags": int(np.bitwise_or.reduce([e[1] for e in rec.eeg])),
    }


def run_case(rec: Recorder, name: str, rate: int, encoding: int, imu: bool,
             seconds: float) -> dict:
    rec.ok(L.CMD_STREAM_STOP)
    rec.pump(0.3)
    rec.ok(L.CMD_SET_ENCODING, encoding)
    rec.ok(L.CMD_SET_IMU, 1 if imu else 0, IMU_RATE & 0xFF, IMU_RATE >> 8, IMU_G,
           IMU_DPS & 0xFF, IMU_DPS >> 8)
    cfg = rec.config()
    if cfg["rate"] != rate:
        cfg, _ = change_rate(rec, rate)
    rec.begin(name, {"rate": rate, "encoding": encoding, "imu": imu,
                     "gains": cfg["gains"], "config": jsonable(cfg)})
    rec.watch.configure(rate, cfg["gains"])
    rec.ok(L.CMD_STREAM_START)
    rec.pump(seconds)
    rec.ok(L.CMD_STREAM_STOP)
    rec.pump(0.3)
    out = stalls(rec, rate)
    rec.meta["stalls"] = out
    rec.save()
    if "stalls" not in out:
        print(f"  {name}: only {out['frames']} frames", flush=True)
        return out
    print(f"  {name:<22} {out['samples']:6d} samples, {out['stalls']:3d} stalls, "
          f"{out['lost']:6.0f} lost ({out['lost_pct']:4.2f} %), worst "
          f"{out['worst_ms']:6.1f} ms; {out['sps_true']:8.2f} SPS between stalls, "
          f"{out['sps_overall']:8.2f} overall, flags 0x{out['flags']:02x}", flush=True)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    link = L.BleLink()
    end = time.time() + 45
    while not link.connected and time.time() < end:
        try:
            print("  " + link.status.get(timeout=1), flush=True)
        except queue.Empty:
            pass
    if not link.connected:
        print("could not connect", flush=True)
        link.close()
        return 1
    time.sleep(1.0)

    rec = Recorder(link, out)
    code = 0
    original = None
    try:
        rec.ok(L.CMD_STREAM_STOP)
        original = rec.config()
        print("device as found", flush=True)
        describe(original, {})

        print(f"\neach case runs {args.seconds:.0f} s", flush=True)
        cases = (
            ("1000 raw+uv, imu on", 1000, L.ENC_RAW_UV, True),    # the session's case
            ("1000 raw+uv, imu off", 1000, L.ENC_RAW_UV, False),  # is it the motion lane?
            ("1000 counts, imu on", 1000, L.ENC_RAW_I24, True),   # is it the link's load?
            ("1000 counts, imu off", 1000, L.ENC_RAW_I24, False),
            ("500 raw+uv, imu on", 500, L.ENC_RAW_UV, True),      # controls
            ("250 raw+uv, imu on", 250, L.ENC_RAW_UV, True),
        )
        for name, rate, enc, imu in cases:
            run_case(rec, name.replace(" ", "_").replace(",", ""), rate, enc, imu,
                     args.seconds)
    except (Lost, Fail, KeyboardInterrupt) as exc:
        print(f"\nSTOPPED: {exc!r}", flush=True)
        code = 2
    finally:
        if link.connected and original is not None:
            for op, a in ((L.CMD_STREAM_STOP, ()), (L.CMD_SET_ENCODING, (original["encoding"],)),
                          (L.CMD_SET_IMU, (1, IMU_RATE & 0xFF, IMU_RATE >> 8, IMU_G,
                                           IMU_DPS & 0xFF, IMU_DPS >> 8))):
                try:
                    rec.cmd(op, *a)
                except (Lost, Fail):
                    pass
            try:
                if rec.config()["rate"] != original["rate"]:
                    change_rate(rec, original["rate"])
            except (Lost, Fail):
                pass
        link.close()
        time.sleep(0.5)
    print("=== done ===" if code == 0 else "=== stopped ===", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
