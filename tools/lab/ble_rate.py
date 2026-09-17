import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref
from bleak import BleakScanner, BleakClient

NODE="535749465445"
CTRL=f"57724502-4700-4000-8000-{NODE}"
STRM=f"57724503-4700-4000-8000-{NODE}"
EVT =f"57724504-4700-4000-8000-{NODE}"

buf=bytearray(); ebuf=bytearray()
st={"data":0,"samples":0,"bad":0,"bytes":0,"seqgap":0}
rsp=[]; nextseq={"v":None}

def parse(b, sink):
    while True:
        i=b.find(0xA5)
        if i<0: b.clear(); return
        if i: del b[:i]
        if len(b)<8: return
        ln=struct.unpack_from("<H",b,4)[0]; tot=8+ln+2
        if len(b)<tot: return
        raw=bytes(b[:tot]); del b[:tot]
        try: f=proto_ref.decode(raw)
        except Exception: st["bad"]+=1; continue
        sink(f)

def on_stream(_,d):
    st["bytes"]+=len(d); buf.extend(d)
    def h(f):
        if f.type==0x04:
            st["data"]+=1
            _,seq,ch,enc,cnt=struct.unpack_from("<QIBBH",f.payload)
            if nextseq["v"] is not None and seq!=nextseq["v"]: st["seqgap"]+=1
            nextseq["v"]=seq+cnt
            st["samples"]+=cnt
    parse(buf,h)

def on_evt(_,d):
    ebuf.extend(d)
    parse(ebuf, lambda f: rsp.append(bytes(f.payload[:2])) if f.type==0x02 else None)

async def run(c, sps, secs):
    st.update(data=0,samples=0,bad=0,bytes=0,seqgap=0); nextseq["v"]=None; rsp.clear()
    await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x09, sps&0xFF, sps>>8])), response=False)
    await asyncio.sleep(2.0)
    await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,1,bytes([0x02])), response=False)
    t0=time.time(); await asyncio.sleep(secs); dt=time.time()-t0
    await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,2,bytes([0x03])), response=False)
    await asyncio.sleep(0.4)
    print(f"  {sps:>4} SPS -> {st['samples']/dt:7.1f} SPS actual, "
          f"{st['bytes']/dt/1024:5.1f} kB/s, {st['data']} frames, "
          f"{st['bad']} bad, {st['seqgap']} seq gaps")

async def main():
    dev=await BleakScanner.find_device_by_name("SwiftEEG", timeout=15.0)
    if not dev: print("not found"); return 1
    async with BleakClient(dev) as c:
        print("connected, mtu", c.mtu_size)
        await c.start_notify(STRM,on_stream)
        await c.start_notify(EVT,on_evt)
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,9,bytes([0x01])), response=False)
        await asyncio.sleep(1.0)
        print("ping replies:", rsp)
        for sps in (250,500,1000):
            await run(c,sps,8.0)
    return 0
sys.exit(asyncio.run(main()))
