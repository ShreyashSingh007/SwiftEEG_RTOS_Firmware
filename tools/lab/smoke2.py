import sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_app as A
app = A.App(); app.update(); print("window up")
app.transport.set("Bluetooth"); app._toggle_conn()
t=time.time()+30
while time.time()<t:
    app.update_idletasks(); app.update(); time.sleep(0.01)
    if app.link and app.link.connected: break
print("connected:", bool(app.link and app.link.connected))
for _ in range(250): app.update_idletasks(); app.update(); time.sleep(0.01)
print("config rate:", app.rate_var.get(), "gain:", app.gain_var.get())

app._toggle_stream()
for _ in range(400): app.update_idletasks(); app.update(); time.sleep(0.01)
print(f"stream @250: {app.samples} samples, railed={sum(app.saturated)}/8")

# time window
app.window_var.set("1"); app._set_window()
for _ in range(120): app.update_idletasks(); app.update(); time.sleep(0.01)
print("window 1s -> trace len:", len(app.traces[0]), "expected ~", app.rate)
app.window_var.set("5"); app._set_window()

# manual range
app.scale_var.set("50")
app.update_idletasks(); app.update()
print("manual span for ch1:", app._span_for(0, [0]))
app.scale_var.set("auto")
print("auto span for ch1:", round(app._span_for(0, [0]), 1))

# rate change while streaming
print("changing rate 250 -> 1000 while streaming...")
app.rate_var.set("1000"); app._set_rate()
t=time.time()+9
while time.time()<t: app.update_idletasks(); app.update(); time.sleep(0.01)
print(f"after rate change: {app.samples} samples in {time.time()-app.started_at:.1f}s "
      f"= {app.samples/max(0.1,time.time()-app.started_at):.0f} SPS, trace len {len(app.traces[0])}")

app._reset_link()
app.update_idletasks(); app.update()
print("reset ok, status:", app.status.cget("text"))
app._close(); app.update(); print("OK - no exceptions")
