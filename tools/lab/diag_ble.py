import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, numpy as np
from bleak import BleakScanner, BleakClient
N="535749465445"
CTRL=f"57724502-4700-4000-8000-{N}"; STRM=f"57724503-4700-4000-8000-{N}"; EVT=f"57724504-4700-4000-8000-{N}"
LSB=(2*4.5)/(24*(1<<24))*1e6
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
        if f.type==0x04:
            ts,sq,ch,enc,cnt=struct.unpack_from("<QIBBH",f.payload)
            w=3 if enc==2 else 4
            b=f.payload[16:16+cnt*ch*w]
            if enc==2:
                a=np.frombuffer(b,np.uint8).reshape(-1,3).astype(np.int32)
                v=a[:,0]|(a[:,1]<<8)|(a[:,2]<<16); v=np.where(v&0x800000,v-(1<<24),v)
                data.append(v.reshape(cnt,ch))
            else: data.append(np.frombuffer(b,"<i4").reshape(cnt,ch))
    parse(sb,h)

def on_e(_,d):
    eb.extend(d); parse(eb, lambda f: rsp.append(bytes(f.payload)) if f.type==0x02 else None)

NAMES={0x00:"ID",0x01:"CONFIG1",0x02:"CONFIG2",0x03:"CONFIG3",0x04:"LOFF",
       0x05:"CH1SET",0x06:"CH2SET",0x0D:"BIAS_SENSP",0x0F:"LOFF_SENSP",
       0x15:"MISC1",0x17:"CONFIG4"}

async def main():
    d=await BleakScanner.find_device_by_name("SwiftEEG",timeout=20.0)
    async with BleakClient(d) as c:
        await c.start_notify(STRM,on_s); await c.start_notify(EVT,on_e)
        async def cmd(op,*a,w=0.45):
            rsp.clear()
            await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
            await asyncio.sleep(w); return list(rsp)

        await cmd(0x03)                    # stop stream
        r=await cmd(0x08,0x00,0,w=1.5)     # source = ELECTRODES
        print("set source->electrodes reply:", r[0][:2].hex() if r else "none")
        await cmd(0x04,0)                  # raw int32

        print("\nregisters (source = Electrodes):")
        for a in sorted(NAMES):
            rr=await cmd(0x07,a)
            v=[x[3] for x in rr if len(x)>=4 and x[0]==0x07 and x[2]==a]
            print(f"  {NAMES[a]:11s} 0x{a:02x} = " + (f"0x{v[0]:02x}" if v else "NO REPLY"))

        data.clear()
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
        await asyncio.sleep(6.0)
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x03])), response=False)
        await asyncio.sleep(0.4)

    if not data: print("NO DATA"); return 1
    x=np.vstack(data).astype(float)
    print(f"\n{len(x)} samples over ~6 s")
    print(f"{'ch':>3} {'mean uV':>13} {'std uV':>10} {'p-p uV':>11}   note")
    FS=(1<<23)
    for ch in range(8):
        raw=x[:,ch]; v=raw*LSB
        note=""
        if np.max(np.abs(raw)) > FS*0.95: note="RAILED (input open or saturated)"
        elif np.std(raw) < 2: note="dead flat - no signal at all"
        print(f"{ch+1:>3} {np.mean(v):13.1f} {np.std(v):10.2f} {np.ptp(v):11.1f}   {note}")
    return 0
sys.exit(asyncio.run(main()))
