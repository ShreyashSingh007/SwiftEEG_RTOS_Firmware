import sys, struct, random
sys.path.insert(0, r"D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS/tools")
import numpy as np
import proto_ref
from swifteeg_link import FrameParser, DATA_HDR, TYPE_DATA, ENC_RAW_UV, decode_data

rng = np.random.default_rng(1)
def data_frame(seq, ts):
    # RAW_UV: 3 samples x 8 ch x (3 B int24 + 4 B f32); realistic small counts
    body = bytearray()
    for s in range(3):
        for c in range(8):
            cnt = int(rng.normal(0, 3000)) & 0xFFFFFF
            body += cnt.to_bytes(3, "little")
            body += struct.pack("<f", float(rng.normal(0, 30)))
    payload = DATA_HDR.pack(ts, seq, 8, ENC_RAW_UV, 3) + bytes(body)
    return proto_ref.encode(TYPE_DATA, 0, seq & 0xFFFF, payload)

N = 2000
frames = [data_frame(i*3, 1_000_000 + i*3000) for i in range(N)]
flen = len(frames[0])
stream = b"".join(frames)
print("frame len", flen, "bytes/s at 1kSPS:", flen*1000//3)

def run(buf, chunk=4096):
    p = FrameParser(); got = []
    for i in range(0, len(buf), chunk):
        got += p.feed(buf[i:i+chunk])
    return got, p

# A: corrupt the length of frame 10 to 0xFFFF
b = bytearray(stream); off = 10*flen; b[off+4:off+6] = b"\xff\xff"
got, p = run(bytes(b))
seqs = [decode_data(f.payload)[1]//3 for f in got]
print("A: 0xFFFF length at frame 10 -> decoded", len(got), "of", N, "; lost", N-len(got), "; first seq after 9:", [s for s in seqs if s > 9][:1], "bad", p.bad)

# B: start mid-stream at every offset within a frame
lost = []
for k in range(1, flen):
    got, p = run(stream[k:])
    first = decode_data(got[0].payload)[1]//3 if got else None
    lost.append((first - 1) if first is not None else N)
lost = np.array(lost)
print("B: start at offset 1..flen-1 -> frames lost before lock: median", np.median(lost), "max", lost.max(), "mean", lost.mean().round(1), "; offsets losing >10 frames:", int((lost > 10).sum()), "of", len(lost))

# C: flip a payload byte (CRC failure) in frame 10 -> only that frame lost?
b = bytearray(stream); b[10*flen + 20] ^= 0x40
got, p = run(bytes(b)); print("C: CRC error in frame 10 -> lost", N-len(got), "bad", p.bad)

# D: 1 junk byte 0xA5 inserted before frame 10
b = stream[:10*flen] + b"\xa5" + stream[10*flen:]
got, p = run(b); print("D: a lone 0xA5 junk byte before frame 10 -> lost", N-len(got), "bad", p.bad)

# E: dropped bytes (USB CDC loss of 7 bytes inside frame 10's payload)
b = stream[:10*flen+30] + stream[10*flen+37:]
got, p = run(b); print("E: 7 bytes dropped inside frame 10 -> lost", N-len(got), "bad", p.bad)
