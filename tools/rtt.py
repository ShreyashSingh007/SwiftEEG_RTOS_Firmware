"""
Read the firmware's RTT log.

RTT is the only log path on this board - there is no spare UART.

    python tools/rtt.py            # halt the core, dump the buffer, exit
    python tools/rtt.py --live 20  # leave the core running, follow for 20 s

Halted mode is the default because RTT reads race the running target over the
ST-Link HLA transport and fail intermittently. The log text is already sitting
in the RAM ring buffer, so stopping the core first makes retrieval reliable.

Live mode leaves the target running, which is what you want when watching for
something that happens after boot. It is less dependable, and halting the core
also stops the BLE radio - so never use halted mode while testing BLE.

The target is left HALTED after a halted-mode run. Reflash, or reset with:
    openocd -f openocd/swifteeg.cfg -c init -c "mww 0xE000ED0C 0x05FA0004" -c exit
"""

from __future__ import annotations

import argparse
import pathlib
import socket
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent.parent
OPENOCD = pathlib.Path(r"D:\swifteeg-tools\xpack-openocd-0.12.0-7\bin\openocd.exe")
PORT = 9090


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", type=float, metavar="SECONDS",
                    help="follow a running target for this long")
    ap.add_argument("--wait", type=float, default=2.0,
                    help="seconds to collect in halted mode (default 2)")
    args = ap.parse_args()

    cfg = "openocd/rtt.cfg" if args.live else "openocd/rtt_halted.cfg"
    duration = args.live if args.live else args.wait

    # cwd at the repo root keeps every path relative: the repo path contains
    # spaces, which OpenOCD's TCL would split into separate arguments.
    proc = subprocess.Popen(
        [str(OPENOCD), "-f", "openocd/swifteeg.cfg", "-f", cfg],
        cwd=str(ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    try:
        sock = None
        deadline = time.time() + 10.0
        while time.time() < deadline:
            try:
                sock = socket.create_connection(("localhost", PORT), timeout=1.0)
                break
            except OSError:
                if proc.poll() is not None:
                    print("openocd exited early:", file=sys.stderr)
                    print(proc.stdout.read(), file=sys.stderr)
                    return 1
                time.sleep(0.2)

        if sock is None:
            print("timed out waiting for the RTT server", file=sys.stderr)
            return 1

        sock.settimeout(0.5)
        end = time.time() + duration
        while time.time() < end:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            sys.stdout.write(chunk.decode("utf-8", "replace"))
            sys.stdout.flush()
        sock.close()
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    return 0


if __name__ == "__main__":
    sys.exit(main())
