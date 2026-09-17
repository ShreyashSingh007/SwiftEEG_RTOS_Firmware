import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, eeg_dsp, numpy as np
from bleak import BleakScanner, BleakClient
N="535749465445"
CTRL=f"57724502-4700-4000-8000-{N}"; STRM=f"57724503-4700-4000-8000-{N}"; EVT=f"57724504-4700-4000-8000-{N}"
LSB=(2*4.5)/(24*(1<<24))*1e6; FSC=(1<<23)
SITES=["C4","P4","F4","Oz","AFz","F3","C3","P3"]
sb=bytearray(); data=[]
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
async def once():
    d=await BleakScanner.find_device_by_name("SwiftEEG",timeout=25.0)
    if not d: print("board not found (disconnect the app first?)"); return None
    async with BleakClient(d) as c:
        await asyncio.sleep(1.0)
        await c.start_notify(STRM,on_s)
        async def w(op,*a,wait=0.6):
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
            await asyncio.sleep(wait)
        await w(0x03); await w(0x09,250,0,wait=2.5)
        await w(0x04,0)      # RAW counts
        await w(0x0C,0)      # device notch off
        await w(0x08,0x00,0,wait=1.2)
        data.clear()
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
        t0=time.time(); await asyncio.sleep(16.0); dt=time.time()-t0
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x03])), response=False)
        await asyncio.sleep(0.4)
    return np.vstack(data).astype(float), len(np.vstack(data))/dt
async def main():
    for a in range(4):
        try:
            r=await once()
            if r is None: return 1
            x,fs=r; break
        except OSError:
            print("  retrying..."); await asyncio.sleep(4)
    else: return 1

    good=[c for c in range(8) if np.max(np.abs(x[:,c]))<FSC*0.98]
    print(f"{len(x)} samples @ {fs:.1f} SPS; usable channels "
          f"{[f'CH{c+1}' for c in good]}\n")

    uv=x*LSB
    ch=eeg_dsp.Chain(fs,8)
    ch.update_mains(uv)
    print(f"mains measured: {ch.measured_mains:.3f} Hz\n")
    filt=np.vstack([ch.process(uv[i:i+6]) for i in range(0,len(uv),6)])
    filt=filt[int(fs*6):]                     # let it settle

    def spec(v):
        w=np.hanning(len(v)); sp=np.abs(np.fft.rfft((v-v.mean())*w))*2/np.sum(w)
        return np.fft.rfftfreq(len(v),1/fs), sp
    print("energy left AFTER the host chain (uV rms per band):")
    print(f"{'ch':<9}{'0.5-4':>8}{'4-8':>7}{'8-13':>7}{'13-30':>7}"
          f"{'30-45':>7}{'45-55':>8}{'55-70':>8}{'95-105':>8}")
    for c in good:
        fr,sp=spec(filt[:,c])
        def b(lo,hi):
            m=(fr>=lo)&(fr<hi); return float(np.sqrt(np.sum(sp[m]**2)))
        print(f"CH{c+1} {SITES[c]:<5}{b(0.5,4):8.2f}{b(4,8):7.2f}{b(8,13):7.2f}"
              f"{b(13,30):7.2f}{b(30,45):7.2f}{b(45,55):8.2f}{b(55,70):8.2f}"
              f"{b(95,105):8.2f}")
    # what are the biggest leftover peaks?
    print("\nlargest leftover peaks above 12 Hz:")
    for c in good[:4]:
        fr,sp=spec(filt[:,c]); sp[fr<12]=0
        top=np.argsort(sp)[-3:][::-1]
        print(f"  CH{c+1} {SITES[c]:<4}" +
              "  ".join(f"{fr[i]:6.1f} Hz {sp[i]:6.2f} uV" for i in top))
    return 0
sys.exit(asyncio.run(main()))
