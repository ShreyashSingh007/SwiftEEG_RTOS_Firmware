import sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_app as A

app = A.App()
app.update()
print("window up")

app.transport.set("Bluetooth")
app._toggle_conn()
t = time.time() + 25
while time.time() < t:
    app.update_idletasks(); app.update(); time.sleep(0.01)
    if app.link and app.link.connected: break
print("connected:", bool(app.link and app.link.connected))
print("status:", app.status.cget("text"))

# let GET_CONFIG land
for _ in range(200): app.update_idletasks(); app.update(); time.sleep(0.01)
print(f"config -> rate {app.rate_var.get()} SPS, gain {app.gain_var.get()}, "
      f"enc {app.enc_var.get()}, dev notch {app.dev_notch.get()}, src {app.src_var.get()}")

app._toggle_stream()
t = time.time() + 8
while time.time() < t:
    app.update_idletasks(); app.update(); time.sleep(0.01)
print(f"streaming: {app.samples} samples, {app.frames} frames, "
      f"{app.gaps} gaps, {app.link.bad_frames} bad")
print("trace len ch1:", len(app.traces[0]))
vals = list(app.traces[0])[-200:]
if vals:
    import numpy as np
    print(f"ch1 after host filters: mean {np.mean(vals):8.2f} uV, "
          f"p-p {np.ptp(vals):8.1f} uV")

app._toggle_record(); 
for _ in range(100): app.update_idletasks(); app.update(); time.sleep(0.01)
print("recorded rows:", app.rec_rows)
app._toggle_record()

app.hp_var.set("1.0"); app._rebuild_chain()
app.car_var.set(False); app._rebuild_chain()
print("filter changes accepted")
app._close(); app.update()
print("OK - no exceptions")
