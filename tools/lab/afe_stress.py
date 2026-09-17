"""
Commands during streaming: do they corrupt samples or wedge the AFE bus?

Streams raw counts at 1000 SPS with the inputs shorted (so every sample is
fresh noise), sends register reads - each one takes the AFE out of
continuous-read mode and back - and looks for two things:

  stale   a sample identical on all eight channels to the one two places
          earlier: an old frame delivered again after the excursion
  wedged  commands that fail or time out
"""
import argparse
import queue
import sys
import time

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_link as L  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=int, default=1000)
    ap.add_argument("--count", type=int, default=60)
    args = ap.parse_args()

    link = L.BleLink()
    end = time.time() + 45
    while not link.connected and time.time() < end:
        try:
            print(" ", link.status.get(timeout=1))
        except queue.Empty:
            pass
    if not link.connected:
        print("could not connect")
        return 1
    time.sleep(1.0)

    data, rsps = {}, []

    def pump(seconds):
        stop = time.time() + seconds
        while time.time() < stop:
            try:
                f = link.frames.get(timeout=0.01)
            except queue.Empty:
                continue
            if f.type == L.TYPE_DATA:
                d = L.decode_data(f.payload)
                if d is not None and d[3] is not None:
                    for i, row in enumerate(d[3]):
                        data[d[1] + i] = row.copy()
            elif f.type == L.TYPE_RSP:
                rsps.append(bytes(f.payload))

    def cmd(op, *a, wait=3.0):
        n = len(rsps)
        link.send(op, *a)
        stop = time.time() + wait
        while time.time() < stop:
            pump(0.01)
            for r in rsps[n:]:
                if r[0] == op:
                    return r
        return None

    cmd(L.CMD_STREAM_STOP)
    cfg = L.decode_config(cmd(L.CMD_GET_CONFIG))
    before = cfg["rate"]
    if before != args.rate:
        cmd(L.CMD_SET_RATE, args.rate & 0xFF, args.rate >> 8)
        time.sleep(2.5)
    cfg = L.decode_config(cmd(L.CMD_GET_CONFIG))
    print(f"rate {cfg['rate']} SPS")

    cmd(L.CMD_SET_INPUT, L.MUX_SHORTED, 0)
    cmd(L.CMD_SET_ENCODING, L.ENC_RAW_I24)
    cmd(L.CMD_STREAM_START)
    pump(1.0)
    data.clear()
    pump(1.0)
    quiet = dict(data)

    ok = failed = lost = 0
    t0 = time.time()
    for _ in range(args.count):
        r = cmd(L.CMD_READ_REG, 0x00)
        if r is None:
            lost += 1
        elif r[1] == L.STATUS_OK:
            ok += 1
        else:
            failed += 1
        pump(0.05)
    elapsed = time.time() - t0
    pump(0.5)
    cmd(L.CMD_STREAM_STOP)
    pump(0.3)

    def analyse(samples, label):
        seqs = sorted(samples)
        if len(seqs) < 3:
            print(f"{label}: no data")
            return
        missing = (seqs[-1] - seqs[0] + 1) - len(seqs)
        stale = sum(1 for s in seqs
                    if s - 2 in samples and np.array_equal(samples[s], samples[s - 2]))
        repeat = sum(1 for s in seqs
                     if s - 1 in samples and np.array_equal(samples[s], samples[s - 1]))
        print(f"{label}: {len(seqs)} samples, {missing} missing, "
              f"{stale} stale (same as two before), {repeat} repeated")

    analyse(quiet, "no commands  ")
    analyse(data, "with commands")
    print(f"register reads: {ok} ok, {failed} failed, {lost} unanswered "
          f"in {elapsed:.1f} s")

    cmd(L.CMD_SET_INPUT, L.MUX_NORMAL, 0)
    if before != cfg["rate"]:
        cmd(L.CMD_SET_RATE, before & 0xFF, before >> 8)
        time.sleep(2.5)
    link.close()
    time.sleep(0.5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
