import sys, time, struct, csv, io, os, platform
sys.path.insert(0, r"D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS/tools")
import numpy as np, eeg_dsp, proto_ref
import swifteeg_link as link
print("python", platform.python_version(), "numpy", np.__version__, "round(np.float64) ->", type(round(np.float64(12345.6))).__name__, repr(round(np.float64(12345.6))))
rng = np.random.default_rng(0)

def chain_cost(fs, block):
    c = eeg_dsp.Chain(fs, 8); c.car = True; c.car_mask[:] = True
    c.highpass_hz = 0.1; c.lowpass_hz = 100.0 if fs > 250 else 45.0; c.rebuild()
    x = rng.normal(0, 50, (fs * 5, 8)) + 1e5
    t = time.perf_counter()
    for i in range(0, len(x), block):
        c.process(x[i:i+block])
    dt = time.perf_counter() - t
    return dt / 5.0
for fs in (250, 1000, 4000, 16000):
    print(f"host chain (HP2+notch2+CAR+LP2) at {fs} SPS, 6-sample blocks: {chain_cost(fs, 6)*1000:.0f} ms CPU per s of data")

# decode_data per frame (RAW_I24, 6 samples) and RAW_UV (3 samples)
def mk(enc, count):
    w = {link.ENC_RAW_I24: 3, link.ENC_RAW_UV: 7}[enc]
    return link.DATA_HDR.pack(123456789, 42, 8, enc, count) + bytes(rng.integers(0, 256, count*8*w, dtype=np.uint8))
for enc, cnt in ((link.ENC_RAW_I24, 6), (link.ENC_RAW_UV, 3)):
    p = mk(enc, cnt); n = 20000
    t = time.perf_counter()
    for _ in range(n): link.decode_data(p)
    per = (time.perf_counter() - t) / n
    print(f"decode_data enc={enc} count={cnt}: {per*1e6:.1f} us/frame -> {per*1e6*1000/cnt/1000:.1f} ms per s at 1 kSPS")

# parser feed cost at 1 kSPS RAW_UV
frames = b"".join(proto_ref.encode(4, 0, i & 0xFFFF, mk(link.ENC_RAW_UV, 3)) for i in range(3334))
p = link.FrameParser(); t = time.perf_counter()
for i in range(0, len(frames), 194): p.feed(frames[i:i+194])
print(f"FrameParser.feed, 1 s of 1 kSPS RAW_UV one frame per call: {(time.perf_counter()-t)*1000:.1f} ms")

# EEG CSV loop exactly as app _consume (1000 rows, blocks of 6)
counts = rng.integers(-8388608, 8388607, (6, 8)).astype(np.int32)
blocks = [(1000000 + k*6000, k*6, counts, None) for k in range(167)]
f = open(os.devnull, "w", newline="", encoding="utf-8"); w = csv.writer(f)
t = time.perf_counter()
for rep in range(10):
    period_us = 1e6 / 1000
    for ts0, seq0, vals, _ in blocks:
        for i, row in enumerate(vals):
            w.writerow([round(ts0 + i * period_us), seq0 + i, *row.tolist()])
print(f"EEG CSV rows at 1000 SPS: {(time.perf_counter()-t)/10*1000:.1f} ms per s of data")

# IMU CSV loop exactly as app _consume_imu at 960 Hz (batches of 16)
cnts = rng.integers(-32768, 32767, (16, 6)).astype(np.int16)
t = time.perf_counter()
for rep in range(10):
    for b in range(60):
        n = len(cnts); scaled = cnts.astype(np.float64); scaled[:, :3] *= 8/32768.0; scaled[:, 3:] *= 2000/32768.0
        times = 5_000_000 + b*16666 + np.arange(n) * 1041.666
        for i in range(n):
            a = scaled[i]
            w.writerow([round(times[i]), b + i, f"{a[0]:.5f}", f"{a[1]:.5f}", f"{a[2]:.5f}", f"{a[3]:.3f}", f"{a[4]:.3f}", f"{a[5]:.3f}", 0])
print(f"IMU CSV rows at 960 Hz: {(time.perf_counter()-t)/10*1000:.1f} ms per s of data")
# what round(times[i]) writes
buf = io.StringIO(); csv.writer(buf).writerow([round((5_000_000 + np.arange(2) * 1041.666)[1]), 1]); print("IMU ts cell as written:", buf.getvalue().strip())
