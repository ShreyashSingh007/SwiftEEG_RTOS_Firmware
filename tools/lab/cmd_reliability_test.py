"""
Do commands get answered while the stream is at its busiest?

Streams raw and filtered at 1000 SPS - the heaviest Bluetooth load the app
makes - and sends commands one at a time, waiting for each reply: pings,
register reads, configuration reads, and every twentieth time a stream stop
and start. Counts replies that never came, and how long the rest took.

    python cmd_reliability_test.py [--no-response]   # write without response
"""
import queue
import sys
import time

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_link as L  # noqa: E402


def main():
    if "--no-response" in sys.argv:
        L.BLE_WRITE_WITH_RESPONSE = False
    print("commands written", "with" if L.BLE_WRITE_WITH_RESPONSE else "without",
          "a response")

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

    rsps = []
    samples = [0]

    def pump(seconds):
        stop = time.time() + seconds
        while time.time() < stop:
            try:
                f = link.frames.get(timeout=0.005)
            except queue.Empty:
                continue
            if f.type == L.TYPE_RSP:
                rsps.append(bytes(f.payload))
            elif f.type == L.TYPE_DATA:
                samples[0] += 1

    def cmd(op, *a, wait=3.0):
        n = len(rsps)
        t0 = time.time()
        link.send(op, *a)
        stop = t0 + wait
        while time.time() < stop:
            pump(0.005)
            for r in rsps[n:]:
                if r[0] == op:
                    return time.time() - t0
        return None

    cmd(L.CMD_STREAM_STOP)
    cmd(L.CMD_SET_RATE, 1000 & 0xFF, 1000 >> 8, wait=8.0)
    cmd(L.CMD_GET_CONFIG, wait=8.0)
    cmd(L.CMD_SET_ENCODING, L.ENC_RAW_UV)
    cmd(L.CMD_STREAM_START)
    pump(2.0)

    lost, times = {}, []
    kinds = [(L.CMD_PING,), (L.CMD_READ_REG, 0x00), (L.CMD_GET_CONFIG,)]
    count = 0
    for i in range(240):
        ops = [kinds[i % 3]]
        if i % 20 == 19:
            ops = [(L.CMD_STREAM_STOP,), (L.CMD_STREAM_START,)]
        for op in ops:
            dt = cmd(*op)
            count += 1
            if dt is None:
                lost[op[0]] = lost.get(op[0], 0) + 1
            else:
                times.append(dt * 1000)
        pump(0.02)

    cmd(L.CMD_STREAM_STOP)
    cmd(L.CMD_SET_ENCODING, L.ENC_RAW_I24)
    cmd(L.CMD_SET_RATE, 250, 0, wait=8.0)
    cmd(L.CMD_GET_CONFIG, wait=8.0)

    t = np.array(times)
    print(f"{count} commands, {sum(lost.values())} unanswered "
          f"{ {f'0x{k:02x}': v for k, v in lost.items()} }; replies in "
          f"{np.median(t):.0f} ms median, {np.percentile(t, 95):.0f} ms p95, "
          f"{t.max():.0f} ms max; {samples[0]} data frames")
    link.close()
    time.sleep(0.5)
    return 0 if not lost else 1


if __name__ == "__main__":
    sys.exit(main())
