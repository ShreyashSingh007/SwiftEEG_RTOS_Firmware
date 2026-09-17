"""
A rate change on the real board keeps what was set on the AFE.

Inputs shorted (fresh noise in every sample, no electrodes needed), channel 2
at gain 12 and channel 7 at gain 6, bias drive on. Then the rate is changed
while streaming, the way the app does it - STREAM_STOP, SET_RATE,
GET_CONFIG - and after each change:

  kept     gains and inputs from GET_CONFIG, bias from CONFIG3 bit 2
  answer   how long GET_CONFIG took to come back after SET_RATE
  stream   samples after STREAM_START: gaps, stale frames, measured rate
"""
import queue
import sys
import time

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_link as L  # noqa: E402

GAINS = [24, 12, 24, 24, 24, 24, 6, 24]
REG_CONFIG3, PD_BIAS = 0x03, 0x04


def main():
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
                        data[d[1] + i] = (d[0], row.copy())
            elif f.type == L.TYPE_RSP:
                rsps.append((time.time(), bytes(f.payload)))

    def cmd(op, *a, wait=5.0):
        n = len(rsps)
        link.send(op, *a)
        stop = time.time() + wait
        while time.time() < stop:
            pump(0.01)
            for t, r in rsps[n:]:
                if r[0] == op:
                    return t, r
        return None, None

    def config():
        _, r = cmd(L.CMD_GET_CONFIG)
        return L.decode_config(r) if r else None

    cmd(L.CMD_STREAM_STOP)
    first = config()
    start_rate = first["rate"]
    print(f"start: {start_rate} SPS, gains {first['gains']}")

    cmd(L.CMD_SET_INPUT, L.MUX_SHORTED, 0)
    for ch, g in enumerate(GAINS):
        cmd(L.CMD_SET_CHANNEL, ch, L.GAIN_CODES[g], L.MUX_SHORTED, 0, 0)
    cmd(L.CMD_SET_BIAS, 1, 0xFF, 0xFF)
    cmd(L.CMD_SET_ENCODING, L.ENC_RAW_I24)

    def check_kept(label):
        cfg = config()
        _, r = cmd(L.CMD_READ_REG, REG_CONFIG3)
        bias = r is not None and r[1] == L.STATUS_OK and bool(r[3] & PD_BIAS)
        gains_ok = cfg is not None and cfg["gains"] == GAINS
        mux_ok = cfg is not None and all(m == L.MUX_SHORTED for m in cfg["mux"])
        print(f"  {label}: gains {'kept' if gains_ok else cfg and cfg['gains']}, "
              f"inputs {'kept' if mux_ok else cfg and cfg['mux']}, "
              f"bias {'on' if bias else 'OFF'}"
              + (f" (CONFIG3 0x{r[3]:02x})" if r is not None and len(r) > 3 else ""))
        return gains_ok and mux_ok and bias

    ok = check_kept("before any change")

    def stream(seconds, label):
        data.clear()
        cmd(L.CMD_STREAM_START)
        pump(seconds)
        cmd(L.CMD_STREAM_STOP)
        pump(0.3)
        seqs = sorted(data)
        if len(seqs) < 10:
            print(f"  {label}: no data")
            return False
        missing = (seqs[-1] - seqs[0] + 1) - len(seqs)
        rows = {s: v[1] for s, v in data.items()}
        stale = sum(1 for s in seqs if s - 2 in rows and np.array_equal(rows[s], rows[s - 2]))
        ts = np.array([data[s][0] for s in seqs], dtype=float)
        rate = (seqs[-1] - seqs[0]) / ((ts[-1] - ts[0]) / 1e6) if ts[-1] > ts[0] else 0.0
        print(f"  {label}: {len(seqs)} samples from seq {seqs[0]}, {missing} missing, "
              f"{stale} stale, {rate:.1f} SPS by the device clock")
        return missing == 0 and stale == 0

    ok &= stream(2.0, f"stream at {start_rate}")

    for rate in [r for r in (500, 1000, 250) if r != start_rate] + [start_rate]:
        t0 = time.time()
        link.send(L.CMD_SET_RATE, rate & 0xFF, rate >> 8)
        t_cfg, r = cmd(L.CMD_GET_CONFIG, wait=8.0)
        cfg = L.decode_config(r) if r else None
        if cfg is None:
            print(f"{rate} SPS: no answer")
            ok = False
            continue
        print(f"{rate} SPS: device reports {cfg['rate']}, answered "
              f"{(t_cfg - t0) * 1000:.0f} ms after SET_RATE")
        ok &= cfg["rate"] == rate
        ok &= check_kept("after the change")
        ok &= stream(3.0, "stream")

    cmd(L.CMD_SET_INPUT, L.MUX_NORMAL, 0)
    print("rate keep test:", "OK" if ok else "FAILED")
    link.close()
    time.sleep(0.5)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
