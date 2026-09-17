import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref
from bleak import BleakScanner, BleakClient

NODE = "535749465445"
SVC  = f"57724501-4700-4000-8000-{NODE}"
CTRL = f"57724502-4700-4000-8000-{NODE}"
STRM = f"57724503-4700-4000-8000-{NODE}"

buf = bytearray(); stats = {"data":0,"rsp":0,"bad":0,"samples":0,"bytes":0}

def on_notify(_, data):
    stats["bytes"] += len(data)
    buf.extend(data)
    while True:
        i = buf.find(0xA5)
        if i < 0: buf.clear(); return
        if i: del buf[:i]
        if len(buf) < 8: return
        ln = struct.unpack_from("<H", buf, 4)[0]; tot = 8+ln+2
        if len(buf) < tot: return
        raw = bytes(buf[:tot]); del buf[:tot]
        try: f = proto_ref.decode(raw)
        except Exception: stats["bad"] += 1; continue
        if f.type == 0x04:
            stats["data"] += 1
            _,_,ch,enc,cnt = struct.unpack_from("<QIBBH", f.payload)
            stats["samples"] += cnt
        elif f.type == 0x02:
            stats["rsp"] += 1

async def main():
    print("scanning...")
    dev = await BleakScanner.find_device_by_name("SwiftEEG", timeout=15.0)
    if not dev:
        print("SwiftEEG not found"); return 1
    print("found", dev.address)

    async with BleakClient(dev) as c:
        print("connected, mtu:", c.mtu_size)
        await c.start_notify(STRM, on_notify)
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
        t0 = time.time()
        await asyncio.sleep(10.0)
        dt = time.time() - t0
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,1,bytes([0x03])), response=False)
        await asyncio.sleep(0.3)

    print(f"\n{stats['data']} data frames, {stats['rsp']} responses, {stats['bad']} bad")
    print(f"{stats['samples']} samples in {dt:.1f}s = {stats['samples']/dt:.1f} SPS")
    print(f"{stats['bytes']/dt/1024:.1f} kB/s")
    return 0

sys.exit(asyncio.run(main()))
