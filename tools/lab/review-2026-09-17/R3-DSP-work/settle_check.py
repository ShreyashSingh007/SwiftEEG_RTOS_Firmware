# dsp_cascade_settle_samples (dsp.c:189-221) vs the actual decay of a primed restart, device SVF loop.
import numpy as np
def design(fs,f0,q,kind):
    g=float(np.float32(np.tan(np.float32(np.pi)*np.float32(f0)/np.float32(fs)))); k=1.0/q
    return (g,k)+{'notch':(1,-k,0),'lp':(0,0,1),'hp':(1,-k,-1)}[kind]
def code_settle(secs):
    tot=0.0
    for g,k,*_ in secs:
        tau=(k+np.sqrt(k*k-4))/(4*g) if k>2 else 1/(g*k)
        tot+=4.6*tau
    return int(np.ceil(tot))
def run(secs,x):
    ic1=[0.0]*len(secs); ic2=[0.0]*len(secs); v=x[0]
    for i,(g,k,m0,m1,m2) in enumerate(secs): ic1[i]=0.0; ic2[i]=v; v=(m0+m2)*v
    y=np.empty(len(x))
    for n,xn in enumerate(x):
        for i,(g,k,m0,m1,m2) in enumerate(secs):
            a1=1/(1+g*(g+k)); a2=g*a1; a3=g*a2
            v3=xn-ic2[i]; v1=a1*ic1[i]+a2*v3; v2=ic2[i]+a2*ic1[i]+a3*v3
            ic1[i]=2*v1-ic1[i]; ic2[i]=2*v2-ic2[i]; xn=m0*xn+m1*v1+m2*v2
        y[n]=xn
    return y
for fs,mains in ((250,50),(250,60),(500,50),(1000,50)):
    secs=[design(fs,mains,12,'notch')]
    if 2*mains<0.95*0.5*fs: secs.append(design(fs,2*mains,12,'notch'))
    t=np.arange(4000)/fs
    # restart primed at a sample where the input carries mains and its harmonic (worst phase: sweep)
    worst=0
    for ph in np.linspace(0,2*np.pi,24,endpoint=False):
        x=50*np.sin(2*np.pi*mains*t+ph)+10*np.sin(2*np.pi*2*mains*t+2*ph)
        y=run(secs,x); pk=np.max(np.abs(y[:5])) or 1
        above=np.nonzero(np.abs(y)>0.01*np.max(np.abs(y)))[0]
        worst=max(worst,above[-1]+1)
    exact=sum(4.6*(-2/np.log((1-g*k+g*g)/(1+g*k+g*g))) for g,k,*_ in secs)
    print(f"{fs} SPS, {mains} Hz Q12 notch{' + harmonic' if len(secs)>1 else ''}: flagged {code_settle(secs)} samples; "
          f"output above 1% of its peak until sample {worst}; per-section 4.6*tau from the pole radius sums to {exact:.0f}")
