import sys, time
W = r"C:/Users/shrey/AppData/Local/Temp/claude/D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware/005c4da0-af1b-4f26-97b9-4ee4c4c7cdce/scratchpad/phaseA/python-pro-work"
sys.path.insert(0, W + "/fakebleak2")
sys.path.insert(0, r"D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS/tools")
import bleak, swifteeg_link as link

def wait(cond, s=3.0):
    end = time.time() + s
    while not cond() and time.time() < end: time.sleep(0.005)
    return cond()

def reset(**kw):
    bleak.LOG.clear(); bleak.MODE.update(found=True, connect_s=0.02, drop_after=None, write_fails=False, scan_s=0.0); bleak.MODE.update(kw)

reset(found=False)
l = link.BleLink(); assert wait(lambda: l.finished)
print("not found   ->", l.state, "|", l.reason, "| log", bleak.LOG)
assert l.state == link.LINK_LOST and "not found" in l.reason

reset(drop_after=0.1)
l = link.BleLink(); assert wait(lambda: l.connected); assert wait(lambda: l.finished)
print("dropped     ->", l.state, "|", l.reason, "| log", bleak.LOG)
assert l.state == link.LINK_LOST and l.reason == "BLE link dropped"

reset(write_fails=True)
l = link.BleLink(); assert wait(lambda: l.connected); l.send(link.CMD_PING); assert wait(lambda: l.finished)
print("write fails ->", l.state, "|", l.reason)
assert l.state == link.LINK_LOST and "write failed" in l.reason

reset(connect_s=0.3)
l = link.BleLink(); time.sleep(0.1); l.send(link.CMD_STREAM_STOP)
t0 = time.time(); done = l.close(timeout=2.0); dt = time.time() - t0
print(f"close while connecting -> finished {done} in {dt:.2f} s, state {l.state}, log {bleak.LOG}")
assert done and l.state == link.LINK_CLOSED and "notify" not in bleak.LOG and bleak.LOG[-1] == "disconnect"

reset(scan_s=5.0)
l = link.BleLink(); time.sleep(0.1); t0 = time.time(); done = l.close(timeout=2.0)
print(f"close during 5 s scan -> finished {done} in {time.time()-t0:.2f} s, log {bleak.LOG}")
assert done and bleak.LOG == []

reset()
l = link.BleLink(); assert wait(lambda: l.connected)
for _ in range(20): l.send(link.CMD_PING)
l.send(link.CMD_STREAM_STOP); done = l.close(timeout=2.0); l.send(link.CMD_STREAM_START)
writes = [e[1] for e in bleak.LOG if isinstance(e, tuple)]
print(f"21 queued then close -> {len(writes)} written, last 0x{writes[-1]:02x}, send after close ignored: {link.CMD_STREAM_START not in writes}")
assert done and len(writes) == 21 and writes[-1] == link.CMD_STREAM_STOP
print("BLE state tests: OK")
