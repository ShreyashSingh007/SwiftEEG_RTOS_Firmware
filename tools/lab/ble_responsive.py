import asyncio, struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref
from bleak import BleakScanner, BleakClient
NODE="535749465445"
CTRL=f"57724502-4700-4000-8000-{NODE}"; STRM=f"57724503-4700-4000-8000-{NODE}"; EVT=f"57724504-4700-4000-8000-{NODE}"

sbuf=bytearray(); ebuf=bytearray()
st={"samples":0,"bad":0,"gap":0}; nxt={"v":None}
pending={}; lat=[]

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
        except Exception: st["bad"]+=1; continue
        sink(f)

def on_s(_,d):
    sbuf.extend(d)
    def h(f):
        if f.type==0x04:
            _,seq,ch,enc,cnt=struct.unpack_from("<QIBBH",f.payload)
            if nxt["v"] is not None and seq!=nxt["v"]: st["gap"]+=1
            nxt["v"]=seq+cnt; st["samples"]+=cnt
    parse(sbuf,h)

def on_e(_,d):
    now=time.perf_counter(); ebuf.extend(d)
    def h(f):
        if f.type==0x02 and len(f.payload)>=1:
            op=f.payload[0]
            if op in pending: lat.append((op,(now-pending.pop(op))*1000))
    parse(ebuf,h)

async def ask(c,op,*a):
    pending[op]=time.perf_counter()
    await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([op,*a])), response=False)
    await asyncio.sleep(0.35)

async def main():
    dev=await BleakScanner.find_device_by_name("SwiftEEG",timeout=15.0)
    if not dev: print("not found"); return 1
    async with BleakClient(dev) as c:
        await c.start_notify(STRM,on_s); await c.start_notify(EVT,on_e)
        await ask(c,0x09,0xE8,0x03)          # 1000 SPS
        await asyncio.sleep(2.0)
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x02])), response=False)
        await asyncio.sleep(1.0)
        st.update(samples=0,bad=0,gap=0); nxt["v"]=None; lat.clear()
        t0=time.perf_counter()

        print("commands issued while streaming at 1 kSPS:")
        await ask(c,0x01)                     # ping
        await ask(c,0x07,0x00)                # read ID
        await ask(c,0x0C,60)                  # notch -> 60 Hz
        await ask(c,0x0A,0xFF,0x05,0x00,0,0)  # all channels gain 12
        await ask(c,0x0B,1,0xFF,0x00)         # bias on
        await ask(c,0x0C,50)                  # notch back to 50
        await ask(c,0x0A,0xFF,0x06,0x00,0,0)  # gain back to 24
        dt=time.perf_counter()-t0
        await c.write_gatt_char(CTRL, proto_ref.encode(0x01,0,0,bytes([0x03])), response=False)
        await asyncio.sleep(0.3)

    names={0x01:"ping",0x07:"read reg",0x0C:"set notch",0x0A:"set channels",0x0B:"set bias"}
    for op,ms in lat: print(f"  {names.get(op,hex(op)):<13} {ms:7.1f} ms")
    print(f"\nreplies {len(lat)}/7, worst {max(m for _,m in lat):.0f} ms" if lat else "no replies")
    print(f"streaming through it: {st['samples']/dt:.0f} SPS, {st['bad']} bad, {st['gap']} gaps")
    return 0
sys.exit(asyncio.run(main()))
