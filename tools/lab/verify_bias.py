import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, numpy as np
from bleak import BleakScanner, BleakClient
N="535749465445"
CTRL=f"57724502-4700-4000-8000-{N}"; STRM=f"57724503-4700-4000-8000-{N}"; EVT=f"57724504-4700-4000-8000-{N}"
LSB=(2*4.5)/(24*(1<<24))*1e6; FS=(1<<23)
sb=bytearray(); eb=bytearray(); data=[]; rsp=[]
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
        if f.type!=0x04: return
        ts,sq,ch,enc,cnt=struct.unpack_from("<QIBBH",f.payload)
        w=3 if enc==2 else 4
        b=f.payload[16:16+cnt*ch*w]
        if len(b)<cnt*ch*w: return
        if enc==2:
            a=np.frombuffer(b,np.uint8).reshape(-1,3).astype(np.int32)
            v=a[:,0]|(a[:,1]<<8)|(a[:,2]<<16); v=np.where(v&0x800000,v-(1<<24),v)
            data.append(v.reshape(cnt,ch))
        else: data.append(np.frombuffer(b,"<i4").reshape(cnt,ch))
    parse(sb,h)
def on_e(_,d):
    eb.extend(d); parse(eb, lambda f: rsp.append(bytes(f.payload)) if f.type==0x02 else None)
async def main():
    d=await BleakScanner.find_device_by_name("SwiftEEG",timeout=25.0)
    if not d: print("board not advertising"); return 1
    async with BleakClient(d) as c:
        await c.start_notify(STRM,on_s); await c.start_notify(EVT,on_e)
        async def w(op,*a,wait=0.6):
            rsp.clear()
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
            await asyncio.sleep(wait); return list(rsp)
        NAMES={0x02:"CONFIG2",0x03:"CONFIG3",0x0D:"BIAS_SENSP",0x0E:"BIAS_SENSN",0x15:"MISC1",0x05:"CH1SET"}
        WANT={0x02:0xC0,0x03:0xE4,0x0D:0xFF,0x0E:0xFF,0x15:0x20,0x05:0x60}
        print("register check after the bias fix:")
        ok=True
        for a in sorted(NAMES):
            r=await w(0x07,a)
            v=[x[3] for x in r if len(x)>=4 and x[0]==0x07 and x[2]==a]
            got=v[0] if v else None
            good = got==WANT[a]
            ok &= good
            print(f"  {NAMES[a]:11s} = " + (f"0x{got:02x}" if got is not None else "NO REPLY")
                  + f"   want 0x{WANT[a]:02x}  {'ok' if good else '<-- WRONG'}")
        await w(0x08,0x01,0,wait=1.5)   # inputs shorted - no electrodes needed
        await w(0x04,0); data.clear()
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
        await asyncio.sleep(6.0)
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x03])), response=False)
        await asyncio.sleep(0.4)
        if data:
            x=np.vstack(data).astype(float)
            print(f"\ninputs shorted, bias ON ({len(x)} samples):")
            print(f"  clipping {100*np.mean(np.abs(x)>=FS*0.98):.2f} %")
            for ch in range(8):
                v=(x[:,ch]-x[:,ch].mean())*LSB
                print(f"  CH{ch+1} {np.std(v):7.3f} uV RMS   DC {x[:,ch].mean()*LSB/1000:+7.2f} mV")
        await w(0x08,0x00,0,wait=1.5)   # back to electrodes
    print("\nregisters", "OK" if ok else "MISMATCH")
    return 0
sys.exit(asyncio.run(main()))
