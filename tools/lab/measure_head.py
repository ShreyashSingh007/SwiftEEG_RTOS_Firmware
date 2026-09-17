import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, numpy as np
from bleak import BleakScanner, BleakClient
N="535749465445"
CTRL=f"57724502-4700-4000-8000-{N}"; STRM=f"57724503-4700-4000-8000-{N}"; EVT=f"57724504-4700-4000-8000-{N}"
LSB=(2*4.5)/(24*(1<<24))*1e6; FSC=(1<<23)
SITES=["C4","P4","F4","Oz","AFz","F3","C3","P3"]
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

def band(v,fs,lo,hi):
    w=np.hanning(len(v)); sp=np.abs(np.fft.rfft((v-v.mean())*w))*2/np.sum(w)
    fr=np.fft.rfftfreq(len(v),1/fs); m=(fr>=lo)&(fr<hi)
    return float(np.sqrt(np.sum(sp[m]**2)))

async def run_once():
    d=await BleakScanner.find_device_by_name("SwiftEEG",timeout=25.0)
    if not d: print("board not found"); return 1
    async with BleakClient(d) as c:
        await asyncio.sleep(1.0)          # Windows needs a beat after connect
        await c.start_notify(STRM,on_s); await c.start_notify(EVT,on_e)
        async def w(op,*a,wait=0.6):
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
            await asyncio.sleep(wait)
        await w(0x03); await w(0x09,250,0,wait=2.5)
        await w(0x04,0)          # RAW counts - no device filtering
        await w(0x0C,0)          # device notch OFF so we see the truth
        await w(0x08,0x00,0,wait=1.5)
        data.clear()
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
        t0=time.time(); await asyncio.sleep(12.0); dt=time.time()-t0
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x03])), response=False)
        await asyncio.sleep(0.4)
    if not data: print("NO DATA"); return 1
    x=np.vstack(data).astype(float); fs=len(x)/dt
    print(f"{len(x)} samples, {fs:.1f} SPS, device notch OFF, raw counts\n")
    print(f"{'ch':<9}{'DC mV':>8}{'clip%':>7}{'RMS uV':>9} |"
          f"{'1-4':>7}{'4-8':>7}{'8-12':>7}{'12-30':>7}{'30-45':>7}{'~50':>8}{'~100':>8}")
    print(f"{'':<9}{'':>8}{'':>7}{'':>9} |{'delta':>7}{'theta':>7}{'alpha':>7}"
          f"{'beta':>7}{'muscle':>7}{'mains':>8}{'harm':>8}")
    tot={}
    for ch in range(8):
        raw=x[:,ch]; v=raw*LSB
        clip=100*np.mean(np.abs(raw)>=FSC*0.98)
        b={k:band(v,fs,lo,hi) for k,(lo,hi) in
           {"d":(1,4),"t":(4,8),"a":(8,12),"b":(12,30),"m":(30,45),
            "50":(48,52),"100":(98,102)}.items()}
        tot[ch]=b
        print(f"CH{ch+1} {SITES[ch]:<5}{raw.mean()*LSB/1000:8.1f}{clip:7.1f}"
              f"{np.std(v):9.1f} |{b['d']:7.1f}{b['t']:7.1f}{b['a']:7.1f}"
              f"{b['b']:7.1f}{b['m']:7.1f}{b['50']:8.1f}{b['100']:8.1f}")
    # dominant frequency of the busiest channel, ignoring drift
    good=[c for c in range(8) if np.max(np.abs(x[:,c]))<FSC*0.98]
    print(f"\nusable channels: {[f'CH{c+1}' for c in good]}")
    if good:
        for c in good[:3]:
            v=x[:,c]*LSB; w=np.hanning(len(v))
            sp=np.abs(np.fft.rfft((v-v.mean())*w)); fr=np.fft.rfftfreq(len(v),1/fs)
            sp[fr<2]=0
            top=np.argsort(sp)[-4:][::-1]
            print(f"  CH{c+1} {SITES[c]:<4} strongest: " +
                  ", ".join(f"{fr[i]:.1f} Hz" for i in top))
    return 0
async def main():
    for attempt in range(4):
        try:
            return await run_once()
        except OSError as exc:
            print(f"  attempt {attempt+1} failed ({exc.__class__.__name__}), retrying")
            await asyncio.sleep(4.0)
    print("could not subscribe after 4 attempts")
    return 1

sys.exit(asyncio.run(main()))
