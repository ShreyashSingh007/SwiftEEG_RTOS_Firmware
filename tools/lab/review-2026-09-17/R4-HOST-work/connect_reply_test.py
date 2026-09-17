import sys
sys.path.insert(0, r"D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS/tools")
import numpy as np, proto_ref
import swifteeg_link as link
rng = np.random.default_rng(11)
POOL = 3000
# electrode offsets 30..190 mV of either sign at 22.35 nV/LSB, plus noise; one offset per channel
offs = (rng.choice([-1, 1], 8) * rng.uniform(1.3e6, 8.3e6, 8)).astype(np.int64)
def pool(sps):
    vals = (offs[None, None, :] + rng.normal(0, 400, (POOL, 6, 8)).astype(np.int64)) & 0xFFFFFF
    b = np.stack([(vals >> 0) & 0xFF, (vals >> 8) & 0xFF, (vals >> 16) & 0xFF], axis=-1).astype(np.uint8)
    t0 = int(rng.integers(0, 10**9))
    return [proto_ref.encode(link.TYPE_DATA, 0, (i * 6) & 0xFFFF,
            link.DATA_HDR.pack(t0 + int(i * 6e6 / sps), i * 6, 8, link.ENC_RAW_I24, 6) + b[i].tobytes())
            for i in range(POOL)]
reply = proto_ref.encode(link.TYPE_RSP, 0, 7, bytes([link.CMD_GET_CONFIG, 0]) + bytes(37))
for sps in (250, 1000):
    frames = pool(sps); fps = sps / 6.0
    print(f"{sps} SPS: fraction of frames with 0xA5 after SOF: {np.mean([0xA5 in f[1:] for f in frames]):.2f}")
    for ms in (30, 100):
        T = 500; lost = 0
        for _ in range(T):
            j = int(rng.integers(0, POOL - 500)); off = int(rng.integers(1, len(frames[j])))
            ins = j + 1 + int(round(ms / 1000 * fps))
            s = frames[j][off:] + b"".join(frames[j + 1:ins]) + reply + b"".join(frames[ins:j + 500])
            p = link.FrameParser(); got = []
            for i in range(0, len(s), 8192):
                got += p.feed(s[i:i + 8192])
            lost += not any(f.type == link.TYPE_RSP for f in got)
        print(f"  reply {ms} ms after port open: connect-time GET_CONFIG reply swallowed in {lost}/{T} ({100 * lost / T:.0f}%)")
