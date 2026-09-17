import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, numpy as np
from bleak import BleakScanner, BleakClient
N="535749465445"
CTRL=f"57724502-4700-4000-8000-{N}"; STRM=f"57724503-4700-4000-8000-{N}"; EVT=f"57724504-4700-4000-8000-{N}"
sb=bytearray(); eb=bytearray(); cnt={"n":0,"f":0}; rsp=[]
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
        except Exception: continue
        sink(f)
def on_s(_,d):
    sb.extend(d)
    def h(f):
        if f.type==0x04:
            _,_,ch,enc,c=struct.unpack_from("<QIBBH",f.payload)
            cnt["n"]+=c; cnt["f"]+=1
    parse(sb,h)
def on_e(_,d):
    eb.extend(d); parse(eb, lambda f: rsp.append(bytes(f.payload)) if f.type==0x02 else None)

async def main():
    d=await BleakScanner.find_device_by_name("SwiftEEG",timeout=20.0)
    async with BleakClient(d) as c:
        await c.start_notify(STRM,on_s); await c.start_notify(EVT,on_e)
        async def w(op,*a):
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
        async def measure(tag, secs=4.0):
            cnt["n"]=0; cnt["f"]=0
            t0=time.time(); await asyncio.sleep(secs); dt=time.time()-t0
            print(f"  {tag:<34} {cnt['n']/dt:7.1f} SPS  ({cnt['f']} frames)")

        await w(0x09,250,0); await asyncio.sleep(2.0)
        await w(0x02); await asyncio.sleep(1.0)
        await measure("streaming at 250")

        print("  -> changing rate to 1000 WHILE streaming")
        rsp.clear()
        await w(0x09,0xE8,0x03); await asyncio.sleep(3.0)
        print(f"     rate reply: {[r[:2].hex() for r in rsp]}")
        await measure("after rate change (no restart)")

        print("  -> re-issuing stream start")
        await w(0x02); await asyncio.sleep(1.0)
        await measure("after re-issuing start")
        await w(0x03)
    return 0
sys.exit(asyncio.run(main()))
