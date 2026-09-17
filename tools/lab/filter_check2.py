import sys, numpy as np
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import eeg_dsp
FS=250.0; N=int(FS*20); t=np.arange(N)/FS

# Realistic: alpha only on CH1 (as real EEG differs per site), mains+drift common.
sig=np.zeros((N,8))
for c in range(8):
    sig[:,c]=200.0*np.sin(2*np.pi*50*t) + 5000.0*np.sin(2*np.pi*0.05*t) + c*20000.0
sig[:,0]+=10.0*np.sin(2*np.pi*10*t)      # 10 uV alpha on CH1 only

def run(**kw):
    c=eeg_dsp.Chain(FS,8)
    for k,v in kw.items(): setattr(c,k,v)
    c.rebuild()
    out=[c.process(sig[i:i+6]) for i in range(0,N,6)]
    return np.vstack(out)[int(FS*12):]

def amp(y,f):
    w=np.hanning(len(y)); fr=np.fft.rfftfreq(len(y),1/FS)
    sp=np.abs(np.fft.rfft(y*w))*2/np.sum(w)
    return sp[np.argmin(np.abs(fr-f))]

print("alpha 10 uV on CH1 only; mains 200 uV and drift common to all\n")
tests=[
 ("no filters",          dict(highpass_hz=0,lowpass_hz=0,notch_hz=0,car=False)),
 ("drift cut only",      dict(highpass_hz=0.5,lowpass_hz=0,notch_hz=0,car=False)),
 ("low-pass 45 only",    dict(highpass_hz=0,lowpass_hz=45,notch_hz=0,car=False)),
 ("notch 50 only",       dict(highpass_hz=0,lowpass_hz=0,notch_hz=50,car=False,notch_harmonic=False)),
 ("CAR only",            dict(highpass_hz=0,lowpass_hz=0,notch_hz=0,car=True)),
 ("drift+notch+lp",      dict(highpass_hz=0.5,lowpass_hz=45,notch_hz=50,car=False,notch_harmonic=False)),
 ("everything + CAR",    dict(highpass_hz=0.5,lowpass_hz=45,notch_hz=50,car=True,notch_harmonic=False)),
]
for name,kw in tests:
    y=run(**kw)
    print(f"{name:<20} CH1: 10Hz {amp(y[:,0],10.0):7.2f} uV   50Hz {amp(y[:,0],50.0):8.3f} uV   "
          f"DC {np.mean(y[:,0]):9.1f} uV")
