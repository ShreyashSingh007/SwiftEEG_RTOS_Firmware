import sys, time, threading
sys.path.insert(0, "fakebleak")
sys.path.insert(0, r"D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS/tools")
import bleak, swifteeg_link as link

# 1: app._reset_link / app._close do link.send(CMD_STREAM_STOP) then link.close()
sent = 0; trials = 100
for _ in range(trials):
    bleak.LOG.clear()
    l = link.BleLink()
    while not l.connected: time.sleep(0.001)
    time.sleep(0.013 * (_ % 7) / 7)   # arbitrary phase against the 10 ms poll
    l.send(link.CMD_STREAM_STOP); l.close()
    l._t.join(1.0)
    sent += any(e[0] == "write" and e[1] == link.CMD_STREAM_STOP for e in bleak.LOG)
print(f"1: STREAM_STOP queued then close(): written in {sent}/{trials} trials")

# 2: close() while scanning (scan takes 0.5 s here; real timeout is 20 s)
bleak.SCAN_S = 0.5; bleak.LOG.clear()
l = link.BleLink(); time.sleep(0.1); t_close = time.perf_counter(); l.close()
l._t.join(2.0)
print("2: close() during scan ->", [(e[0], round(e[-1] - t_close, 3)) for e in bleak.LOG], "; thread alive after:", l._t.is_alive())

# 3: Disconnect during scan then Connect again: two link threads at once
bleak.LOG.clear()
a = link.BleLink(); time.sleep(0.1); a.close(); b = link.BleLink()
time.sleep(0.2)
print("3: threads alive while second scan runs:", a._t.is_alive(), b._t.is_alive())
a._t.join(2); time.sleep(0.6); print("   events:", [e[0] for e in bleak.LOG], "; b.connected:", b.connected)
b.close(); b._t.join(2)
