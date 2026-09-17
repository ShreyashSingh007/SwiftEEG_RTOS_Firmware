# float32 vs float64 error of the device's SVF loop (dsp.c:146-187, same op order),
# 4th-order Butterworth 0.1 Hz high-pass, at the rates pipeline_start() accepts.
import numpy as np, sys, time
def butter_qs(order):
    return [1.0/(2*np.cos(np.pi*(2*i+1)/(2*order))) for i in range(order//2)]
def run(fs, amps_uv, dtype, seconds, drift_hz=0.3):
    F=dtype; n=int(fs*seconds); C=len(amps_uv)
    secs=[]
    for q in butter_qs(4):
        g=F(np.tan(np.float32(np.pi)*F(0.1)/F(fs))) if dtype==np.float32 else np.tan(np.pi*0.1/fs)
        g=F(np.float32(np.tan(np.float32(np.pi)*np.float32(0.1)/np.float32(fs))))  # device designs in f32 (tanf); host sends f32
        k=F(np.float32(1.0)/np.float32(q))
        a1=F(1)/(F(1)+g*(g+k)); a2=g*a1; a3=g*a2
        secs.append((a1,a2,a3,F(1),F(-k),F(-1)))
    t=np.arange(n)/fs
    A=np.asarray(amps_uv,dtype=np.float64)[:,None]
    x_all=(A*np.sin(2*np.pi*drift_hz*t)[None,:] + 20*np.sin(2*np.pi*10*t)[None,:]).astype(dtype)
    ic1=[np.zeros(C,dtype) for _ in secs]; ic2=[np.zeros(C,dtype) for _ in secs]
    # prime on first sample
    v=x_all[:,0].copy()
    for i,(a1,a2,a3,m0,m1,m2) in enumerate(secs):
        ic1[i][:]=0; ic2[i][:]=v; v=(m0+m2)*v
    out=np.empty((C,n),dtype)
    two=F(2)
    for j in range(n):
        x=x_all[:,j]
        for i,(a1,a2,a3,m0,m1,m2) in enumerate(secs):
            v3=x-ic2[i]; v1=a1*ic1[i]+a2*v3; v2=ic2[i]+a2*ic1[i]+a3*v3
            ic1[i]=two*v1-ic1[i]; ic2[i]=two*v2-ic2[i]
            x=m0*x+m1*v1+m2*v2
        out[:,j]=x
    return out
amps=[3000.0, 30000.0]
for fs,sec in [(1000,40),(4000,40),(16000,40)]:
    t0=time.time()
    y32=run(fs,amps,np.float32,sec).astype(np.float64)
    y64=run(fs,amps,np.float64,sec)
    h=y32.shape[1]//2
    err=np.sqrt(np.mean((y32[:,h:]-y64[:,h:])**2,axis=1)); pk=np.max(np.abs(y32[:,h:]-y64[:,h:]),axis=1)
    for a,e,p in zip(amps,err,pk):
        print(f"fs {fs:5d}  drift {a/1000:4.0f} mV @0.3 Hz: f32 error {e:8.3f} uV RMS, {p:8.3f} uV peak   ({time.time()-t0:.0f}s)")
    sys.stdout.flush()
