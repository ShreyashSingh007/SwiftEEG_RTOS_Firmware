"""
Signal quality on a worn headset, with the device's own filters running.

Four minutes: eyes open, eyes closed, eyes open, cued by beeps. Alpha rising
when the eyes close is the standard proof that a headset records brain
signal rather than noise, and the ratio is what gets compared with what a
clinical amplifier does.

    python -u quality_test.py --out DIR
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
from worn_session import Fail, Lost, Recorder, band_rms, jsonable, stages  # noqa: E402

PHASES = (("eyes open", 60.0), ("eyes closed", 60.0))
HP_HZ, LP_HZ = 1.0, 100.0
RAIL = (1 << 23) - 64


def beep(times: int) -> None:
    try:
        import winsound
        for i in range(times):
            winsound.Beep(880, 220)
            time.sleep(0.12)
    except Exception:  # noqa: BLE001 - a missing beeper is not a failure
        pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
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
        print("could not connect - is the headset on?", flush=True)
        link.close()
        return 1
    time.sleep(1.0)

    rec = Recorder(link, out)
    code = 0
    try:
        rec.ok(L.CMD_STREAM_STOP)
        cfg = rec.config()
        if any(m != L.MUX_NORMAL for m in cfg["mux"]):
            rec.ok(L.CMD_SET_INPUT, L.MUX_NORMAL, 0)
        rec.ok(L.CMD_SET_ENCODING, L.ENC_RAW_UV)
        rec.ok(L.CMD_SET_IMU, 1, 240, 0, 8, 2000 & 0xFF, 2000 >> 8)
        rec.ok(L.CMD_SET_CAR, 0, 0xFF)
        rec.ok(L.CMD_SET_NOTCH, *L.notch_args(50, 12, True, True))
        pre, post = stages(cfg["rate"], HP_HZ, LP_HZ)
        rec.ok(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_PRE, pre, cfg["rate"]))
        rec.ok(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_POST, post, cfg["rate"]))
        cfg = rec.config()
        print(f"  {cfg['rate']} SPS, gains {cfg['gains']}, notch {cfg['notch']} Hz "
              f"aimed at {cfg['notch_aim_hz']:.3f}, filters {cfg['pre_count']}+"
              f"{cfg['post_count']}", flush=True)

        rec.begin("quality", {"rate": cfg["rate"], "gains": cfg["gains"],
                              "config": jsonable(cfg), "hp": HP_HZ, "lp": LP_HZ,
                              "pre": pre, "post": post})
        rec.ok(L.CMD_STREAM_START)
        rec.pump(3.0)  # let the chain settle before the first phase

        marks = []
        for name, seconds in PHASES:
            beep(1 if name == "eyes closed" else 2)
            t0 = time.time()
            print(f"  {name} for {seconds:.0f} s", flush=True)
            rec.pump(seconds)
            marks.append({"phase": name, "start": t0 - rec.t0,
                          "end": time.time() - rec.t0})
        beep(3)

        rec.ok(L.CMD_STREAM_STOP)
        rec.pump(0.3)
        rec.meta["phases"] = marks
        rec.meta["config_after"] = jsonable(rec.config())
        rec.save()
    except (Lost, Fail, KeyboardInterrupt) as exc:
        print(f"STOPPED: {exc!r}", flush=True)
        if rec.eeg:
            rec.meta["phases"] = []
            rec.save()
        code = 2
    finally:
        try:
            if link.connected:
                rec.cmd(L.CMD_STREAM_STOP)
        except (Lost, Fail):
            pass
        link.close()
        time.sleep(0.5)
    return code


if __name__ == "__main__":
    sys.exit(main())
