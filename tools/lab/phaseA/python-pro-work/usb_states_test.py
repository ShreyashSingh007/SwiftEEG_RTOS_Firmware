import sys, time
W = r"C:/Users/shrey/AppData/Local/Temp/claude/D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware/005c4da0-af1b-4f26-97b9-4ee4c4c7cdce/scratchpad/phaseA/python-pro-work"
sys.path.insert(0, W + "/fakeserial")
sys.path.insert(0, r"D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS/tools")
import serial, proto_ref, swifteeg_link as link

def wait(cond, s=3.0):
    end = time.time() + s
    while not cond() and time.time() < end: time.sleep(0.005)
    return cond()

frame = proto_ref.encode(link.TYPE_RSP, 0, 5, bytes([link.CMD_PING, 0]))
serial.MODE.update(data=b"\x00\xa5\x07" + frame)
l = link.UsbLink()
assert l.connected and l.status.get_nowait() == "USB COM9"
t = time.time(); f = l.frames.get(timeout=1)
print("frame after junk:", f.type, f.payload.hex(), "host_time set:", abs(f.host_time - t) < 1, "bad", l.bad_frames)
l.send(link.CMD_STREAM_STOP); ser = serial.OPEN[-1]
done = l.close(timeout=1.0)
print("close -> finished", done, "state", l.state, "port closed by thread", ser.closed, "stop written", ser.written[-1][8] == link.CMD_STREAM_STOP)
l.send(link.CMD_PING); assert len(ser.written) == 1, "send after close must be ignored"
assert done and l.state == link.LINK_CLOSED and ser.closed and l.status.empty()

serial.MODE.update(data=b"", read_fails_after=0.1)
l = link.UsbLink(); l.status.get_nowait()
assert wait(lambda: l.finished)
print("unplugged -> state", l.state, "reason", l.reason, "| message", l.status.get_nowait())
assert l.state == link.LINK_LOST and "read failed" in l.reason

serial.MODE.update(read_fails_after=None, write_fails=True)
l = link.UsbLink(); l.status.get_nowait(); l.send(link.CMD_PING)
print("write fails -> state", l.state, "reason", l.reason); assert l.state == link.LINK_LOST
l.close(timeout=1.0); assert l.state == link.LINK_CLOSED and l.finished
print("USB state tests: OK")
