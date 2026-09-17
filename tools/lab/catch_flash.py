"""Retry SWD until the chip answers, then erase and flash immediately.

BLE firmware holds the bus and blocks SWD once it is running, so the only
dependable window is the moment after power-on. This hammers the connection
until it lands in that window.
"""
import subprocess, sys, time
OCD=r"D:\swifteeg-tools\xpack-openocd-0.12.0-7\bin\openocd.exe"
CWD=r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS"
HEX="{D:/SwiftEEG_RTOS/build/zephyr/zephyr.hex}"

def ocd(*cmds, timeout=30):
    a=[OCD,"-f","openocd/swifteeg.cfg"]
    for c in cmds: a += ["-c", c]
    try:
        r=subprocess.run(a, cwd=CWD, capture_output=True, text=True, timeout=timeout)
        return r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return "TIMEOUT"

print("waiting for the chip - power-cycle the board now", flush=True)
deadline = time.time() + 120
n = 0
while time.time() < deadline:
    n += 1
    out = ocd("init", "halt", "exit", timeout=12)
    if "Examination succeed" in out:
        print(f"caught it on attempt {n} - erasing and flashing", flush=True)
        out = ocd("init", "halt", "nrf5 mass_erase",
                  "mww 0xE000ED0C 0x05FA0004", "sleep 300", "halt",
                  "sleep 100", "halt",
                  f"flash write_image erase {HEX}",
                  f"verify_image {HEX}",
                  "mww 0xE000ED0C 0x05FA0004", "exit", timeout=120)
        for line in out.splitlines():
            if any(k in line for k in ("Mass erase","wrote","verified","Error")):
                print("   " + line.strip(), flush=True)
        print("DONE" if "verified" in out else "FLASH FAILED", flush=True)
        sys.exit(0 if "verified" in out else 1)
    if n % 5 == 0:
        print(f"  ...{n} attempts, still locked (keep power-cycling)", flush=True)
print("gave up after 120 s", flush=True)
sys.exit(2)
