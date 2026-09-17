# dsp_section_group_delay (dsp.c:276-329) transliterated in float32 vs exact float64 group delay.
import numpy as np
F=np.float32; PI=F(3.14159265358979323846)
def gd_f32(g,k,m0,m1,m2,fs,f):
    g,k,m0,m1,m2,fs,f=map(F,(g,k,m0,m1,m2,fs,f))
    gg=g*g; d0=F(1)+g*k+gg; d1=F(2)*(gg-F(1))/d0; d2=(F(1)-g*k+gg)/d0
    n0=(m0*d0+m1*g+m2*gg)/d0; n1=(F(2)*m0*(gg-F(1))+F(2)*m2*gg)/d0; n2=(m0*(F(1)-g*k+gg)-m1*g+m2*gg)/d0
    w=F(2)*PI*f/fs; dw=F(1e-4); ph=[]
    for s in (-dw,dw):
        wi=w+s; c1=np.cos(wi,dtype=F); s1=np.sin(wi,dtype=F); c2=np.cos(F(2)*wi,dtype=F); s2=np.sin(F(2)*wi,dtype=F)
        nr=n0+n1*c1+n2*c2; ni=-(n1*s1+n2*s2); dr=F(1)+d1*c1+d2*c2; di=-(d1*s1+d2*s2)
        ph.append(np.arctan2(ni,nr,dtype=F)-np.arctan2(di,dr,dtype=F))
    d=ph[1]-ph[0]
    while d>PI: d-=F(2)*PI
    while d<-PI: d+=F(2)*PI
    return float(-d/(F(2)*dw))
def gd_exact(g,k,m0,m1,m2,fs,f):
    g,k,m0,m1,m2=map(np.float64,(g,k,m0,m1,m2)); gg=g*g; d0=1+g*k+gg
    b=np.array([m0*d0+m1*g+m2*gg, 2*m0*(gg-1)+2*m2*gg, m0*(1-g*k+gg)-m1*g+m2*gg])/d0
    a=np.array([1,2*(gg-1)/d0,(1-g*k+gg)/d0])
    w=2*np.pi*f/fs; z=np.exp(-1j*w*np.arange(3))
    def gdp(c):  # group delay of polynomial sum c_n z^-n
        num=np.sum(np.arange(3)*c*z); den=np.sum(c*z); return (num/den).real
    return gdp(b)-gdp(a)
def sec(kind,fs,fc,q):
    g=float(np.tan(np.float32(np.pi)*np.float32(fc)/np.float32(fs))); k=1/q
    return {'hp':(g,k,1,-k,-1),'lp':(g,k,0,0,1),'notch':(g,k,1,-k,0)}[kind]
q2=[0.5411961,1.3065630]
for fs in (250,1000,16000):
    for f in (0.5,1,2,5,10,30):
        for kind,fc,q in (('hp',0.1,q2[0]),('hp',0.1,q2[1]),('notch',50,12),('lp',40,0.7071)):
            s=sec(kind,fs,fc,q)
            a=gd_f32(*s,fs,f); b=gd_exact(*s,fs,f)
            if abs(a-b)>0.05*max(1e-3,abs(b)) and abs(a-b)*1e3/fs>0.05:
                print(f"fs {fs:5d} {kind:5s} fc {fc:5.1f} Q {q:.3f} at {f:4.1f} Hz: f32 {a:10.3f} vs exact {b:10.3f} samples  (err {abs(a-b)*1e3/fs:8.3f} ms)")
print("done")
