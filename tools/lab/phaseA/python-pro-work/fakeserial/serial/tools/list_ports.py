class P: device, vid, pid = "COM9", 0x2FE3, 0x0001
def comports(): return [P()]
