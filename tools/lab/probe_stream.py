import struct, sys, time, pathlib
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, serial, serial.tools.list_ports

port = None
for p in serial.tools.list_ports.comports():
    if p.vid == 0x2FE3 and p.pid == 0x0001:
        port = p.device
print("port:", port)
if not port:
    for p in serial.tools.list_ports.comports():
        print("  seen:", p.device, p.description, hex(p.vid or 0), hex(p.pid or 0))
    sys.exit(1)

s = serial.Serial(port, 115200, timeout=0.2)
s.write(proto_ref.encode(0x01, 0, 0, bytes([0x04, 0])))   # encoding = raw
s.write(proto_ref.encode(0x01, 0, 1, bytes([0x02])))      # stream start

buf = bytearray()
frames = bad = samples = 0
types = {}
t_end = time.time() + 3.0
first = None
while time.time() < t_end:
    buf.extend(s.read(4096))
    while True:
        i = buf.find(0xA5)
        if i < 0: buf.clear(); break
        if i: del buf[:i]
        if len(buf) < 8: break
        ln = struct.unpack_from("<H", buf, 4)[0]
        tot = 8 + ln + 2
        if len(buf) < tot: break
        raw = bytes(buf[:tot]); del buf[:tot]
        try:
            f = proto_ref.decode(raw)
        except Exception:
            bad += 1; continue
        frames += 1
        types[f.type] = types.get(f.type, 0) + 1
        if f.type == 0x04:
            ts, seq, ch, enc, cnt = struct.unpack_from("<QIBBH", f.payload)
            samples += cnt
            if first is None:
                first = (ts, seq, ch, enc, cnt)
                body = f.payload[16:16+ch*4]
                print("first sample counts:", struct.unpack(f"<{ch}i", body))
s.write(proto_ref.encode(0x01, 0, 2, bytes([0x03])))
s.close()
print(f"frames={frames} bad={bad} samples={samples} types={types}")
print("hdr:", first)
print(f"rate ~= {samples/3.0:.1f} SPS")
