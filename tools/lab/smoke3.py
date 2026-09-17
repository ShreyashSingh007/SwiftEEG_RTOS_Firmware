import sys, time, numpy as np
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_app as A
app=A.App(); app.update()
app.transport.set("Bluetooth"); app._toggle_conn()
t=time.time()+35
while time.time()<t:
    app.update_idletasks(); app.update(); time.sleep(0.01)
    if app.link and app.link.connected: break
if not (app.link and app.link.connected):
    print("could not connect"); sys.exit(1)
for _ in range(250): app.update_idletasks(); app.update(); time.sleep(0.01)
app._toggle_stream()
t=time.time()+20
while time.time()<t: app.update_idletasks(); app.update(); time.sleep(0.01)
print("mains measured:", app.chain.measured_mains)
print("notch q:", app.chain.notch_q, "track:", app.chain.notch_track)
print("railed:", [i+1 for i in range(8) if app.saturated[i]])
print("DC mV:", " ".join(f"{v:+.0f}" for v in app.dc_mv))
for ch in (2,3,4,6,7):
    v=np.array(list(app.traces[ch])[-1000:])
    if len(v)>500:
        w=np.hanning(len(v)); fr=np.fft.rfftfreq(len(v),1/app.rate)
        sp=np.abs(np.fft.rfft(v*w))*2/np.sum(w)
        f50=sp[np.argmin(np.abs(fr-49.6))]
        al=np.sqrt(np.sum(sp[(fr>=8)&(fr<12)]**2))
        print(f"  CH{ch+1} after host filters: mains {f50:6.2f} uV   alpha {al:6.2f} uV   "
              f"rms {np.std(v):7.2f} uV")
app._close(); app.update(); print("OK")
