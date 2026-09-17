import sys, tkinter as tk
sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import swifteeg_app as A
app=A.App(); app.geometry("1200x700"); app.update_idletasks(); app.update()

# find the scrolling canvas inside the side panel
cv=[w for w in app._side.winfo_children() if isinstance(w,tk.Canvas)][0]
inner=[w for w in cv.winfo_children() if isinstance(w,tk.Frame)][0]
app.update_idletasks()
content=inner.winfo_reqheight(); view=cv.winfo_height()
print(f"content {content} px, viewport {view} px -> {'scroll needed' if content>view else 'fits'}")
print("scrollregion:", cv.cget("scrollregion"))
cv.yview_moveto(1.0); app.update_idletasks(); app.update()
print("scrolled to bottom, yview:", [round(v,2) for v in cv.yview()])
print("stats visible after scroll:", app.stats.winfo_ismapped()==1)
print("mains label:", repr(app.mains_label.cget("text")))
print("notch width:", app.q_var.get(), "| track:", app.track_var.get(),
      "| format:", app.enc_var.get())
app.destroy(); print("OK")
