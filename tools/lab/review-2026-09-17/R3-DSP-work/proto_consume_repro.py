# Line-by-line transliteration of src/proto/proto.c stream decoder (push/try_extract/
# consume/resync/decode) to show the payload view is overwritten by consume().
SOF=0xA5; VER=1; HDR=8; OVH=10; MAXP=1024; MAXF=MAXP+OVH
def crc16(b):
    c=0xFFFF
    for x in b:
        c^=x<<8
        for _ in range(8):
            c=((c<<1)^0x1021)&0xFFFF if c&0x8000 else (c<<1)&0xFFFF
    return c
def enc(t,seq,p):
    f=bytearray([SOF,VER,t,0,len(p)&255,len(p)>>8,seq&255,seq>>8])+bytes(p)
    c=crc16(f[1:]); f+=bytes([c&255,c>>8]); return bytes(f)
class St:
    def __init__(s): s.buf=bytearray(MAXF); s.have=0; s.need=0; s.inf=False; s.crc=0
def valid_type(t): return t in (1,2,3,4,5)
def decode(buf,n):
    if n<OVH or buf[0]!=SOF: return -1,None
    if buf[1]!=VER: return -3,None
    if not valid_type(buf[2]): return -5,None
    pl=buf[4]|buf[5]<<8
    if pl>MAXP or n<OVH+pl: return -1,None
    want=crc16(buf[1:HDR+pl]); got=buf[HDR+pl]|buf[HDR+pl+1]<<8
    if want!=got: return -4,None
    return 0,dict(type=buf[2],len=pl,seq=buf[6]|buf[7]<<8,payload_off=HDR)  # pointer into st.buf
def resync(s):
    start=1
    while start<s.have and s.buf[start]!=SOF: start+=1
    if start>=s.have: s.have=0; s.inf=False; s.need=0; return
    rem=s.have-start; s.buf[0:rem]=s.buf[start:start+rem]; s.have=rem; s.inf=True; s.need=0
def consume(s,count):
    if count>=s.have: s.have=0; s.inf=False; s.need=0; return
    rem=s.have-count; s.buf[0:rem]=s.buf[count:count+rem]; s.have=rem
    s.inf=(s.buf[0]==SOF); s.need=0
    if not s.inf: s.have=0
def try_extract(s):
    while s.inf and s.have>=HDR:
        pl=s.buf[4]|s.buf[5]<<8
        if s.buf[1]!=VER or not valid_type(s.buf[2]) or pl>MAXP: resync(s); continue
        s.need=OVH+pl
        if s.have<s.need: return None
        err,f=decode(s.buf,s.need)
        if err==0:
            consume(s,s.need); return f
        if err==-4: s.crc+=1
        resync(s)
    return None
def push(s,b):
    if not s.inf:
        if b!=SOF: return None
        s.inf=True; s.have=0; s.need=0
    if s.have>=len(s.buf):
        resync(s)
        if not s.inf: return None
    s.buf[s.have]=b; s.have+=1
    return try_extract(s)
def view(s,f): return bytes(s.buf[f['payload_off']:f['payload_off']+f['len']])  # what handle() reads

# Host sends: SET_FILTER (cut: BLE ring dropped its tail), SET_CAR, SET_NOTCH, PING
cut   = enc(1,10,bytes([0x10,0,1,0xFA,0x00,8])+bytes(160))[:30]  # 8-section upload, tail dropped
car   = enc(1,11,bytes([0x11,1,0x7F]))
notch = enc(1,12,bytes([0x0C,50,12,0x03]))
chan  = enc(1,13,bytes([0x0A,0xFF,24,0,0,1]))
ping  = enc(1,14,bytes([0x01]))
extra=[enc(1,20+i,bytes([0x07,i])) for i in range(12)]  # READ_REG i
stream = cut+car+notch+chan+ping+b''.join(extra)
s=St(); sent={11:car,12:notch,13:chan,14:ping}
for i,e in enumerate(extra): sent[20+i]=e
for b in stream:
    f=push(s,b)
    while f:
        v=view(s,f); orig=sent[f['seq']][HDR:HDR+f['len']]
        print(f"seq {f['seq']} len {f['len']}: handler sees {v.hex()} sent {orig.hex()} {'OK' if v==orig else 'CORRUPTED'}")
        f=try_extract(s)  # command.c drains with proto_stream_poll
print("crc errors", s.crc)
