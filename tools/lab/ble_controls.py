import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, numpy as np
from bleak import BleakScanner, BleakClient
NODE="535749465445"
CTRL=f"57724502-4700-4000-8000-{NODE}"; STRM=f"57724503-4700-4000-8000-{NODE}"; EVT=f"57724504-4700-4000-8000-{NODE}"
LSB=(2*4.5)/(24*(1<<24))*1e6
sbuf=bytearray(); ebuf=bytearray(); rsp=[]; blocks=[]; stats={"bytes":0,"bad":0}

def parse(b,sink):
    while True:
        i=b.find(0xA5)
        if i<0: b.clear(); return
        if i: del b[:i]
        if len(b)<8: return
        ln=struct.unpack_from("<H",b,4)[0]; tot=8+ln+2
        if len(b)<tot: return
        raw=bytes(b[:tot]); del b[:tot]
        try: f=proto_ref.decode(raw)
        except Exception: stats["bad"]+=1; continue
        sink(f)

def on_s(_,d):
    stats["bytes"]+=len(d); sbuf.extend(d)
    def h(f):
        if f.type==0x04:
            ts,seq,ch,enc,cnt=struct.unpack_from("<QIBBH",f.payload)
            w=3 if enc==2 else 4
            body=f.payload[16:16+cnt*ch*w]
            if enc==2:
                a=np.frombuffer(body,dtype=np.uint8).reshape(cnt*ch,3).astype(np.int32)
                v=(a[:,0]|(a[:,1]<<8)|(a[:,2]<<16))
                v=np.where(v & 0x800000, v-(1<<24), v).reshape(cnt,ch)
            else:
                v=np.array(struct.unpack(f"<{cnt*ch}i",body)).reshape(cnt,ch)
            blocks.append((f.payload[:16],v,enc))
    parse(sbuf,h)

def on_e(_,d):
    ebuf.extend(d); parse(ebuf, lambda f: rsp.append(bytes(f.payload)) if f.type==0x02 else None)

async def ask(c,op,*a,wait=0.5):
    rsp.clear()
    await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
    await asyncio.sleep(wait)
    return list(rsp)

async def main():
    dev=await BleakScanner.find_device_by_name("SwiftEEG",timeout=15.0)
    if not dev: print("not found"); return 1
    async with BleakClient(dev) as c:
        await c.start_notify(STRM,on_s); await c.start_notify(EVT,on_e)

        r=await ask(c,0x0E)   # GET_CONFIG
        if r:
            p=r[0]
            print(f"config: {p[2]} ch, enc {p[3]}, {p[4]|(p[5]<<8)} SPS, notch {p[6]} Hz")
            print("        CHnSET:", " ".join(f"{b:02x}" for b in p[7:15]))

        print("lead-off on:", (await ask(c,0x0D,1,0xFF,0x00))[0][:2].hex())

        for enc,name in ((0,"int32"),(2,"int24 packed")):
            blocks.clear(); stats["bytes"]=0
            await ask(c,0x04,enc,wait=0.3)
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
            t0=time.time(); await asyncio.sleep(6.0); dt=time.time()-t0
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x03])), response=False)
            await asyncio.sleep(0.4)
            n=sum(len(v) for _,v,_ in blocks)
            st=int(np.mean([struct.unpack("<I",h[0:4])[0] for h,_,_ in blocks])) if blocks else 0
            print(f"{name:>13}: {n/dt:6.1f} SPS, {stats['bytes']/dt/1024:5.1f} kB/s, {stats['bad']} bad")

        # lead-off bits live in the status word of every frame
        if blocks:
            h,_,_=blocks[-1]
            print("\nfirst 3 status words (loff bits [19:12] = P, [11:4] = N):")
            for hh,_,_ in blocks[:1]:
                pass
    return 0
sys.exit(asyncio.run(main()))
