import struct, sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import proto_ref, serial, serial.tools.list_ports, numpy as np
LSB=(2*4.5)/(24*(1<<24))*1e6
port=next((p.device for p in serial.tools.list_ports.comports() if p.vid==0x2FE3), None)
if not port: print("board not on USB"); sys.exit(1)
s=serial.Serial(port,115200,timeout=0.3); buf=bytearray(); seqn=0
def cmd(op,*a):
    global seqn
    s.write(proto_ref.encode(0x01,0,seqn,bytes([op,*a]))); seqn+=1
def pump(t):
    d=[];r=[]; end=time.time()+t
    while time.time()<end:
        buf.extend(s.read(4096))
        while True:
            i=buf.find(0xA5)
            if i<0: buf.clear(); break
            if i: del buf[:i]
            if len(buf)<8: break
            ln=struct.unpack_from("<H",buf,4)[0]; tot=8+ln+2
            if len(buf)<tot: break
            raw=bytes(buf[:tot]); del buf[:tot]
            try: f=proto_ref.decode(raw)
            except Exception: continue
            if f.type==0x02: r.append(bytes(f.payload))
            elif f.type==0x04:
                ts,sq,ch,enc,cnt=struct.unpack_from("<QIBBH",f.payload)
                w=3 if enc==2 else 4
                b=f.payload[16:16+cnt*ch*w]
                if enc==2:
                    a=np.frombuffer(b,np.uint8).reshape(-1,3).astype(np.int32)
                    v=a[:,0]|(a[:,1]<<8)|(a[:,2]<<16); v=np.where(v&0x800000,v-(1<<24),v)
                    d.append(v.reshape(cnt,ch))
                else: d.append(np.frombuffer(b,"<i4").reshape(cnt,ch))
    return d,r

cmd(0x03); pump(0.4)                      # stop
cmd(0x08, 0x00, 0); pump(1.5)             # source = electrodes
cmd(0x04, 0); pump(0.3)                   # raw int32

NAMES={0x00:"ID",0x01:"CONFIG1",0x02:"CONFIG2",0x03:"CONFIG3",0x04:"LOFF",
       0x05:"CH1SET",0x0D:"BIAS_SENSP",0x0F:"LOFF_SENSP",0x15:"MISC1",0x17:"CONFIG4"}
print("registers with source=Electrodes:")
for a in sorted(NAMES):
    cmd(0x07,a); _,r=pump(0.35)
    v=[x[3] for x in r if len(x)>=4 and x[0]==0x07 and x[2]==a]
    print(f"  {NAMES[a]:11s} 0x{a:02x} = " + (f"0x{v[0]:02x}" if v else "NO REPLY"))

cmd(0x02); pump(0.3)
d,_=pump(5.0)
cmd(0x03); pump(0.3); s.close()
if not d: print("NO DATA"); sys.exit(1)
x=np.vstack(d).astype(float)
print(f"\n{len(x)} samples, {len(x)/5.0:.0f} SPS")
print(f"{'ch':>3} {'mean uV':>12} {'std uV':>10} {'p-p uV':>10}  {'saturated?':>10}")
for c in range(8):
    v=x[:,c]*LSB
    sat = "RAILED" if abs(np.mean(x[:,c])) > 8.0e6 else ""
    print(f"{c+1:>3} {np.mean(v):12.1f} {np.std(v):10.2f} {np.ptp(v):10.1f}  {sat:>10}")
