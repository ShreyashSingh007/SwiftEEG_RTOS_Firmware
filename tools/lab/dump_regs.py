import struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, serial, serial.tools.list_ports

port = next(p.device for p in serial.tools.list_ports.comports()
            if p.vid == 0x2FE3 and p.pid == 0x0001)
s = serial.Serial(port, 115200, timeout=0.3)
seq = 0
def cmd(op, *a):
    global seq
    s.write(proto_ref.encode(0x01, 0, seq, bytes([op, *a]))); seq += 1

buf = bytearray()
def pump(secs=0.6):
    out = []
    t = time.time() + secs
    while time.time() < t:
        buf.extend(s.read(2048))
        while True:
            i = buf.find(0xA5)
            if i < 0: buf.clear(); break
            if i: del buf[:i]
            if len(buf) < 8: break
            ln = struct.unpack_from("<H", buf, 4)[0]; tot = 8+ln+2
            if len(buf) < tot: break
            raw = bytes(buf[:tot]); del buf[:tot]
            try: f = proto_ref.decode(raw)
            except Exception: continue
            if f.type == 0x02: out.append(bytes(f.payload))
    return out

NAMES = {0x00:"ID",0x01:"CONFIG1",0x02:"CONFIG2",0x03:"CONFIG3",
         0x05:"CH1SET",0x06:"CH2SET",0x15:"MISC1"}

def dump(label):
    print(f"\n--- {label} ---")
    for addr in sorted(NAMES):
        cmd(0x07, addr)
        for r in pump(0.35):
            if len(r) >= 4 and r[0] == 0x07 and r[2] == addr:
                print(f"  {NAMES[addr]:8s} (0x{addr:02x}) = 0x{r[3]:02x}")
                break
        else:
            print(f"  {NAMES[addr]:8s} (0x{addr:02x}) = NO RESPONSE")

cmd(0x03); pump(0.3)              # stream stop
dump("test signal OFF")
cmd(0x05, 1, 0)
r = pump(0.8)
print("\ntest-signal cmd responses:", [(x[0], x[1]) for x in r])
dump("test signal ON")
cmd(0x05, 0, 0); pump(0.5)
s.close()
