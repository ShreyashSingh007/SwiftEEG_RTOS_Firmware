import sys, time
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_scope as sc

port = sc.find_port()
print("port found:", port)
assert port, "no device"

app = sc.Scope(port)
app.update()                 # realise the window
t = time.time() + 4.0
frames = 0
while time.time() < t:
    app.update_idletasks()
    app.update()             # runs the after() callbacks: read queue + draw
    frames += 1
    time.sleep(0.01)

print(f"ui updates: {frames}")
print(f"reader frames={app.reader.frames} bad_crc={app.reader.bad_crc} samples={app.samples}")
print(f"status text: {app.status.cget('text')}")
print(f"trace0 last 5: {[round(v,1) for v in list(app.traces[0])[-5:]]} uV")
print(f"span: {app.span_uv:.0f} uV")
app.reader.stop()
app.destroy()
print("OK - no exceptions")
