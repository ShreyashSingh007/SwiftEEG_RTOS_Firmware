import struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, serial, serial.tools.list_ports, numpy as np

LSB_UV = (2.0*4.5)/(24*(1<<24))*1e6
port = next(p.device for p in serial.tools.list_ports.comports()
            if p.vid == 0x2FE3 and p.pid == 0x0001)
s = serial.Serial(port, 115200, timeout=0.2)

def cmd(op, *a, seq=0):
    s.write(proto_ref.encode(0x01, 0, seq, bytes([op, *a])))

cmd(0x04, 0)          # raw counts
cmd(0x05, 1, 0)       # TEST SIGNAL ON, cal_freq 00 (~0.98 Hz)
time.sleep(0.4)
time.sleep(0.3)
cmd(0x02)             # stream start

buf = bytearray(); rows = []; rsp = []
t_end = time.time() + 8.0
while time.time() < t_end:
    buf.extend(s.read(4096))
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
        if f.type == 0x02: rsp.append(tuple(f.payload[:2]))
        if f.type == 0x04:
            ts, seq, ch, enc, cnt = struct.unpack_from("<QIBBH", f.payload)
            body = f.payload[16:16+cnt*ch*4]
            v = np.array(struct.unpack(f"<{cnt*ch}i", body)).reshape(cnt, ch)
            rows.append((ts, v))
cmd(0x05, 0, 0); cmd(0x03); s.close()

print("responses (opcode,status):", rsp)
ts0 = rows[0][0]
data = np.vstack([r[1] for r in rows])
print("samples:", data.shape)

for ch in range(8):
    x = data[:, ch].astype(float)
    lo, hi = np.percentile(x, 5), np.percentile(x, 95)
    pp_counts = hi - lo
    pp_uv = pp_counts * LSB_UV
    # square wave: count sign changes about the midpoint
    mid = (hi + lo) / 2
    sgn = np.sign(x - mid)
    crossings = np.count_nonzero(np.diff(sgn) != 0)
    dur = len(x) / 250.0
    freq = crossings / (2 * dur)
    print(f"CH{ch+1}: {pp_counts:9.0f} counts p-p = {pp_uv/1000:7.3f} mV,  ~{freq:.2f} Hz")

print(f"\nexpected: {2*4.5/2400*1e3:.3f} mV p-p at {2.048e6/(1<<21):.2f} Hz")
