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
    d=await BleakScanner.find_device_by_name("SwiftEEG",timeout=20.0)
    async with BleakClient(d) as c:
        await c.start_notify(STRM,on_s); await c.start_notify(EVT,on_e)
        async def w(op,*a,wait=0.6):
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
            await asyncio.sleep(wait)
        await w(0x03); await w(0x09,250,0,wait=2.5); await w(0x04,0)
        await w(0x0C,0); await w(0x08,0x00,0,wait=1.5)

        async def grab(tag, secs=6.0):
            data.clear()
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
            t0=time.time(); await asyncio.sleep(secs); dt=time.time()-t0
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x03])), response=False)
            await asyncio.sleep(0.4)
            if not data: print(f"{tag:<28} NO DATA"); return
            x=np.vstack(data).astype(float); fs=len(x)/dt
            clip=100.0*np.mean(np.abs(x)>=FS*0.98)
            com=x.mean(axis=1); comv=(com-com.mean())*LSB
            wnd=np.hanning(len(comv)); sp=np.abs(np.fft.rfft(comv*wnd))
            fr=np.fft.rfftfreq(len(comv),1/fs); sp[fr<0.7]=0
            pk=fr[int(np.argmax(sp))]
            res=x-com[:,None]
            per=np.mean([np.std(res[:,i]*LSB) for i in range(8)])
            print(f"{tag:<28} clip {clip:5.1f}%   common {np.std(comv):9.0f} uV @ {pk:5.2f} Hz   "
                  f"per-ch {per:9.0f} uV")

        print("bias sense mask sweep (electrodes on, device notch off):\n")
        for mask,name in ((0x00,"bias OFF"),(0x01,"1 ch  (CH1)"),(0x03,"2 ch  (CH1,2)"),
                          (0x0F,"4 ch  (CH1-4)"),(0x3F,"6 ch  (CH1-6)"),(0xFF,"8 ch  (all)")):
            if mask==0: await w(0x0B,0,0,0,wait=1.6)
            else:       await w(0x0B,1,mask,0x00,wait=1.6)
            await grab(name)

        print("\nwith BIAS_SENSN also set (negative inputs = SRB1):")
        for sn,name in ((0x00,"sensp=FF sensn=00"),(0xFF,"sensp=FF sensn=FF")):
            await w(0x0B,1,0xFF,sn,wait=1.6); await grab(name)

        await w(0x0B,0,0,0,wait=1.0)
    return 0
sys.exit(asyncio.run(main()))
