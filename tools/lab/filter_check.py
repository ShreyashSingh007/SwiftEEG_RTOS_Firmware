"""Does the app's filter path actually filter? Synthetic input, no hardware."""
import sys, numpy as np
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import eeg_dsp

FS=250.0; N=int(FS*20)
t=np.arange(N)/FS
alpha=10.0*np.sin(2*np.pi*10*t)      # 10 uV of "EEG"
mains=200.0*np.sin(2*np.pi*50*t)     # 200 uV of mains
drift=5000.0*np.sin(2*np.pi*0.05*t)  # 5 mV of drift

sig=np.zeros((N,8))
for c in range(8):
    sig[:,c]=alpha+mains+drift+c*20000.0   # each on a different big DC offset

def measure(chain, x, label):
    # feed in blocks of 6, exactly as the app does
    out=[]
    for i in range(0,len(x),6):
        out.append(chain.process(x[i:i+6]))
    y=np.vstack(out)[int(FS*12):]     # discard settling
    # how much 10 Hz and 50 Hz survived
    w=np.hanning(len(y)); fr=np.fft.rfftfreq(len(y),1/FS)
    def amp(v,f):
        sp=np.abs(np.fft.rfft(v*w))*2/np.sum(w)
        return sp[np.argmin(np.abs(fr-f))]
    a10=amp(y[:,0],10.0); a50=amp(y[:,0],50.0)
    print(f"{label:<34} 10Hz {a10:7.2f} uV   50Hz {a50:8.3f} uV   "
          f"DC {np.mean(y[:,0]):8.2f} uV   spread {np.ptp(np.mean(y,axis=0)):8.2f} uV")
    return a10,a50

print(f"input: 10 Hz = 10.00 uV, 50 Hz = 200.00 uV, drift 5000 uV, "
      f"channel offsets 0..140000 uV\n")

c=eeg_dsp.Chain(FS,8); c.highpass_hz=0; c.lowpass_hz=0; c.notch_hz=0; c.car=False
c.rebuild(); measure(c,sig,"all filters OFF")

c=eeg_dsp.Chain(FS,8); c.highpass_hz=0.5; c.lowpass_hz=0; c.notch_hz=0; c.car=False
c.rebuild(); measure(c,sig,"drift cut 0.5 Hz only")

c=eeg_dsp.Chain(FS,8); c.highpass_hz=0.5; c.lowpass_hz=0; c.notch_hz=50; c.car=False
c.notch_harmonic=False; c.rebuild(); measure(c,sig,"+ mains notch 50 Hz")

c=eeg_dsp.Chain(FS,8); c.highpass_hz=0.5; c.lowpass_hz=45; c.notch_hz=50; c.car=True
c.notch_harmonic=False; c.rebuild(); a10,a50=measure(c,sig,"+ low-pass 45 + common average")

print()
ok = (a10 > 8.0) and (a50 < 1.0)
print("VERDICT:", "filters work - 10 Hz survives, 50 Hz removed" if ok
      else "FILTERS BROKEN")
