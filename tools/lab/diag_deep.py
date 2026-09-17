import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, numpy as np
from bleak import BleakScanner, BleakClient
N="535749465445"
CTRL=f"57724502-4700-4000-8000-{N}"; STRM=f"57724503-4700-4000-8000-{N}"; EVT=f"57724504-4700-4000-8000-{N}"
LSB=(2*4.5)/(24*(1<<24))*1e6
FS_COUNT=(1<<23)
sb=bytearray(); eb=bytearray(); data=[]; rsp=[]; meta={"enc":None,"cnt":None,"frames":0,"samples":0}

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
        meta["enc"]=enc; meta["cnt"]=cnt; meta["frames"]+=1; meta["samples"]+=cnt
        if enc==2:
            a=np.frombuffer(b,np.uint8).reshape(-1,3).astype(np.int32)
            v=a[:,0]|(a[:,1]<<8)|(a[:,2]<<16); v=np.where(v&0x800000,v-(1<<24),v)
            data.append(v.reshape(cnt,ch))
        else: data.append(np.frombuffer(b,"<i4").reshape(cnt,ch))
    parse(sb,h)

def on_e(_,d):
    eb.extend(d); parse(eb, lambda f: rsp.append(bytes(f.payload)) if f.type==0x02 else None)

def analyse(tag, x, fs):
    print(f"\n=== {tag} ===  {len(x)} samples @ {fs:.0f} SPS")
    print(f"{'ch':>3} {'DC mV':>9} {'std uV':>11} {'clip %':>8} {'peak Hz':>9} {'peak uV':>11}")
    for c in range(8):
        raw=x[:,c]
        clip=100.0*np.mean(np.abs(raw)>=FS_COUNT*0.98)
        v=(raw-raw.mean())*LSB
        w=np.hanning(len(v)); sp=np.abs(np.fft.rfft(v*w)); fr=np.fft.rfftfreq(len(v),1/fs)
        sp[fr<0.7]=0
        k=int(np.argmax(sp))
        print(f"{c+1:>3} {raw.mean()*LSB/1000:9.1f} {np.std(v):11.1f} {clip:8.1f} "
              f"{fr[k]:9.2f} {np.ptp(v):11.1f}")
    # how much is common to every channel
    com=x.mean(axis=1); 
    res=x-com[:,None]
    print(f"    common-mode std {np.std((com-com.mean())*LSB):10.1f} uV")
    print(f"    per-channel after removing it: "
          f"{np.mean([np.std(res[:,c]*LSB) for c in range(8)]):.1f} uV")

async def main():
    d=await BleakScanner.find_device_by_name("SwiftEEG",timeout=20.0)
    if not d: print("not found"); return 1
    async with BleakClient(d) as c:
        await c.start_notify(STRM,on_s); await c.start_notify(EVT,on_e)
        async def w(op,*a,wait=0.6):
            rsp.clear()
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
            await asyncio.sleep(wait); return list(rsp)

        await w(0x03); await w(0x09,250,0,wait=2.5); await w(0x04,0)
        await w(0x0C,0)                       # device notch OFF - see raw truth
        await w(0x08,0x00,0,wait=1.5)         # electrodes

        async def grab(tag, secs=8.0):
            data.clear(); meta["frames"]=0; meta["samples"]=0
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
            t0=time.time(); await asyncio.sleep(secs); dt=time.time()-t0
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x03])), response=False)
            await asyncio.sleep(0.5)
            if not data: print(f"{tag}: NO DATA"); return
            x=np.vstack(data).astype(float)
            print(f"\n[{tag}] enc={meta['enc']} samples/frame={meta['cnt']} "
                  f"frames={meta['frames']} samples={meta['samples']} "
                  f"({meta['samples']/meta['frames']:.1f}/frame)")
            analyse(tag, x, len(x)/dt)

        await w(0x0B,1,0xFF,0x00,wait=1.5); await grab("BIAS ON, electrodes")
        await w(0x0B,0,0x00,0x00,wait=1.5);  await grab("BIAS OFF, electrodes")
        await w(0x08,0x01,0,wait=1.5);       await grab("INPUTS SHORTED (chip only)")
        await w(0x08,0x00,0,wait=1.5); await w(0x0B,1,0xFF,0x00,wait=1.5)
    return 0
sys.exit(asyncio.run(main()))
