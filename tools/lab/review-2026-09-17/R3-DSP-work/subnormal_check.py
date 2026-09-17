# Device SVF loop in float32 on an input that becomes exactly 0 (e.g. a railed channel after DC removal):
# do the states end in subnormals (slow on x86 hosts without FTZ/DAZ)?
import numpy as np
F=np.float32; tiny=np.finfo(F).tiny
def secs(fs, f0, q, kind):
    g=F(np.tan(F(np.pi)*F(f0)/F(fs))); k=F(1)/F(q); a1=F(1)/(F(1)+g*(g+k)); a2=g*a1; a3=g*a2
    m={'notch':(F(1),-k,F(0)),'hp':(F(1),-k,F(-1)),'lp':(F(0),F(0),F(1))}[kind]
    return (a1,a2,a3)+m
for name,chain,fs in (("250 SPS notch 50 + 100 Hz",[secs(250,50,12,'notch'),secs(250,100,12,'notch')],250),
                      ("1 kSPS 0.1 Hz HP x2 + 40 Hz LP",[secs(1000,0.1,0.5412,'hp'),secs(1000,0.1,1.3066,'hp'),secs(1000,40,0.7071,'lp')],1000)):
    ic1=[F(0)]*len(chain); ic2=[F(0)]*len(chain)
    n=int(fs*600); sub_steps=0; first_sub=None
    x0=F(123.4)
    for j in range(n):
        x=x0 if j<50 else F(0)
        for i,(a1,a2,a3,m0,m1,m2) in enumerate(chain):
            v3=x-ic2[i]; v1=a1*ic1[i]+a2*v3; v2=ic2[i]+a2*ic1[i]+a3*v3
            ic1[i]=F(2)*v1-ic1[i]; ic2[i]=F(2)*v2-ic2[i]; x=m0*x+m1*v1+m2*v2
        states=[abs(v) for v in ic1+ic2]
        if any(0<s<tiny for s in states):
            sub_steps+=1
            if first_sub is None: first_sub=j
    print(f"{name}: samples with a subnormal state {sub_steps} of {n}; first at {first_sub}; final states {[float(v) for v in ic1+ic2]}")
