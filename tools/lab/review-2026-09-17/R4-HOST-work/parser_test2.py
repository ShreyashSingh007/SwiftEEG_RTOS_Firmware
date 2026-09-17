import sys, struct
sys.path.insert(0, r"D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS/tools")
import numpy as np, proto_ref
from swifteeg_link import FrameParser, DATA_HDR, TYPE_DATA, ENC_RAW_UV, decode_data
exec(open("parser_test.py").read().split("N = 2000")[0].split("rng = ")[1].join(["rng = ",""]) if False else "")
rng = np.random.default_rng(7)
def data_frame(seq, ts):
    body = bytearray()
    for s in range(3):
        for c in range(8):
            body += (int(rng.normal(0, 3000)) & 0xFFFFFF).to_bytes(3, "little")
            body += struct.pack("<f", float(rng.normal(0, 30)))
    return proto_ref.encode(TYPE_DATA, 0, seq & 0xFFFF, DATA_HDR.pack(ts, seq, 8, ENC_RAW_UV, 3) + bytes(body))
N = 3000
frames = [data_frame(i*3, 1_000_000 + i*3000) for i in range(N)]
flen = len(frames[0]); stream = b"".join(frames)
print("fraction of frames containing a 0xA5 byte after SOF:", np.mean([0xA5 in f[1:] for f in frames]).round(3))

class FixedParser(FrameParser):
    def feed(self, data):
        self.buf.extend(data); out = []
        while True:
            i = self.buf.find(proto_ref.SOF)
            if i < 0: self.buf.clear(); return out
            if i: del self.buf[:i]
            if len(self.buf) < proto_ref.HEADER_LEN: return out
            ln = struct.unpack_from("<H", self.buf, 4)[0]
            if self.buf[1] != proto_ref.VERSION or self.buf[2] not in proto_ref.VALID_TYPES or ln > proto_ref.MAX_PAYLOAD:
                del self.buf[:1]; self.bad += 1; continue
            total = proto_ref.HEADER_LEN + ln + proto_ref.CRC_LEN
            if len(self.buf) < total: return out
            try:
                out.append(proto_ref.decode(bytes(self.buf[:total]))); del self.buf[:total]
            except ValueError:
                del self.buf[:1]; self.bad += 1
def run(cls, buf, chunk=4096):
    p = cls(); got = []
    for i in range(0, len(buf), chunk): got += p.feed(buf[i:i+chunk])
    return got
lostA, lostB = [], []
for trial in range(400):
    j = int(rng.integers(0, 1500)); k = int(rng.integers(1, flen))
    s = stream[j*flen + k: (j+1400)*flen]
    for cls, acc in ((FrameParser, lostA), (FixedParser, lostB)):
        got = run(cls, s)
        first = decode_data(got[0].payload)[1]//3 if got else j+1400
        acc.append(first - (j+1))
for name, a in (("current", lostA), ("header-checked, drop-1-byte", lostB)):
    a = np.array(a); print(f"USB connect mid-stream, {name}: frames lost before lock mean {a.mean():.1f} p90 {np.percentile(a,90):.0f} max {a.max()}; >100 frames in {(a>100).mean()*100:.1f}% of starts")
# single-byte slip with fixed parser
b = stream[:10*flen] + b"\xa5" + stream[10*flen:]
print("fixed parser, lone 0xA5 inserted: lost", N - len(run(FixedParser, b)))
b = stream[:10*flen+30] + stream[10*flen+37:]
print("fixed parser, 7 bytes dropped in frame 10: lost", N - len(run(FixedParser, b)))
b = bytearray(stream); b[10*flen+4:10*flen+6] = b"\xff\xff"
print("fixed parser, 0xFFFF length: lost", N - len(run(FixedParser, bytes(b))))
