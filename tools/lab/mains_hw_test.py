"""
The mains tracker on the board, nothing worn.

Floating inputs pick up the room's mains strongly - the easy case, but a real
one. Checks that the tracker agrees on a frequency and says so in events,
that the notch moves to it, that the configuration reports both, and that an
FFT of the raw counts streamed alongside finds the same frequency. Then the
same at 1000 SPS, where it should start from the last estimate.
"""
import queue
import sys
import time

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_link as L  # noqa: E402


def fft_mains(x, fs, nominal=50.0):
    x = x - x.mean()
    sp = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), 1 / fs)
    idx = np.flatnonzero((fr >= nominal - 3) & (fr <= nominal + 3))
    k = idx[np.argmax(sp[idx])]
    a, b, c = sp[k - 1], sp[k], sp[k + 1]
    return fr[k] + 0.5 * (a - c) / (a - 2 * b + c) * (fr[1] - fr[0])


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

    rows, events, rsps = {}, [], []
    t_start = [time.time()]

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
                        rows[d[1] + i] = row
            elif f.type == L.TYPE_EVT:
                ev = L.decode_event(bytes(f.payload))
                if ev is not None:
                    events.append((time.time() - t_start[0], ev))
            elif f.type == L.TYPE_RSP:
                rsps.append(bytes(f.payload))

    def cmd(op, *a, wait=5.0):
        n = len(rsps)
        link.send(op, *a)
        stop = time.time() + wait
        while time.time() < stop:
            pump(0.01)
            for r in rsps[n:]:
                if r[0] == op:
                    return r
        return None

    ok = True
    for rate, seconds in ((250, 45.0), (1000, 30.0)):
        cmd(L.CMD_STREAM_STOP)
        cfg = L.decode_config(cmd(L.CMD_GET_CONFIG))
        if cfg["rate"] != rate:
            cmd(L.CMD_SET_RATE, rate & 0xFF, rate >> 8)
            cfg = L.decode_config(cmd(L.CMD_GET_CONFIG, wait=8.0))
        cmd(L.CMD_SET_INPUT, L.MUX_NORMAL, 0)
        cmd(L.CMD_SET_ENCODING, L.ENC_RAW_I24)
        cmd(L.CMD_SET_NOTCH, *L.notch_args(50, 12, True, True))
        cfg = L.decode_config(cmd(L.CMD_GET_CONFIG))
        print(f"\n{rate} SPS: notch {cfg['notch']} Hz Q {cfg['notch_q']}, harmonic "
              f"{cfg['notch_harmonic']}, following {cfg['notch_track']}; measured "
              f"{cfg['mains_hz']}, aimed at {cfg['notch_aim_hz']:.3f} Hz")

        rows.clear()
        events.clear()
        t_start[0] = time.time()
        cmd(L.CMD_STREAM_START)
        pump(seconds)
        cmd(L.CMD_STREAM_STOP)
        pump(0.3)
        cfg = L.decode_config(cmd(L.CMD_GET_CONFIG))

        for t, ev in events:
            print(f"  +{t:5.1f} s  {ev['hz']:.4f} Hz  from sample {ev['seq']}"
                  f"{'  notch moved' if ev['moved'] else ''}")
        seqs = sorted(rows)
        counts = np.array([rows[s] for s in seqs], dtype=float)
        uv = counts.mean(axis=1) * L.lsb_uv(24)
        tail = uv[-int(20 * rate):]
        fft = fft_mains(tail, rate)
        clipped = int(np.sum(np.abs(counts) >= (1 << 23) - 64))
        print(f"  {len(seqs)} samples; FFT of the last 20 s: {fft:.4f} Hz; "
              f"{clipped} clipped values")
        print(f"  config after: measured {cfg['mains_hz']}, aimed at "
              f"{cfg['notch_aim_hz']:.4f} Hz")

        if not events:
            print("  FAIL: no mains event")
            ok = False
            continue
        last = events[-1][1]["hz"]
        ok &= abs(last - fft) < 0.05
        ok &= cfg["mains_hz"] is not None and abs(cfg["mains_hz"] - last) < 1e-3
        ok &= abs(cfg["notch_aim_hz"] - last) < 0.031
        print(f"  last estimate {last:.4f} Hz vs FFT {fft:.4f} Hz: "
              f"{(last - fft) * 1000:+.1f} mHz")

    cmd(L.CMD_SET_RATE, 250, 0)
    time.sleep(1.0)
    print("\nmains hardware test:", "OK" if ok else "FAILED")
    link.close()
    time.sleep(0.5)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
