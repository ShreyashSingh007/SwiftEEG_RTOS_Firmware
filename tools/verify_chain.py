"""
The on-device filter chain, checked against the reference, with no
electrodes.

Loads a known chain onto the device, restarts it at a sample number the
device reports, and runs tools/pipeline_ref.py - the model of the firmware's
chain - over the raw counts streamed alongside the device's own output. The
two must agree: the same sections, the same float32 operations, from the
same sample.

The inputs are the ADS1299's own. Channels 1-4 carry its test square wave
and 5-8 are shorted, with channel 2 at gain 12 and channel 7 at gain 6 so
the per-channel scaling is exercised too. Nothing needs to be worn, and the
board is put back on its electrodes afterwards.

    python tools/verify_chain.py                   # Bluetooth, 250 SPS
    python tools/verify_chain.py --usb --rate 1000

Close the Windows application first: the board takes one Bluetooth
connection at a time.

Also reported: what the raw + filtered stream costs the link, as sequence
gaps and the rate actually delivered.
"""

from __future__ import annotations

import argparse
import pathlib
import queue
import sys
import time

import numpy as np

sys.path.insert(0, pathlib.Path(__file__).resolve().parent.as_posix())
import eeg_dsp  # noqa: E402
import pipeline_ref  # noqa: E402
import swifteeg_link as L  # noqa: E402

F32 = np.float32

# Agreement required between the device and the reference, in microvolts.
# Both are float32 doing the same operations, so this is a few ulps of a
# millivolt-sized signal; a real disagreement is orders of magnitude larger.
MATCH_UV = 0.05


class Fail(Exception):
    pass


class Session:
    def __init__(self, link: L.Link):
        self.link = link
        self.capturing = False
        self.data: list = []

    def pump(self, seconds: float) -> list[bytes]:
        """Take frames for a while. Returns the responses that arrived."""
        rsps = []
        end = time.time() + seconds
        while time.time() < end:
            try:
                f = self.link.frames.get(timeout=0.02)
            except queue.Empty:
                continue
            if f.type == L.TYPE_RSP:
                rsps.append(bytes(f.payload))
            elif f.type == L.TYPE_DATA and self.capturing:
                d = L.decode_data(f.payload)
                if d is not None:
                    self.data.append((f.flags, d))
        return rsps

    def cmd(self, op: int, *args: int, timeout: float = 3.0) -> bytes:
        self.link.send(op, *args)
        end = time.time() + timeout
        while time.time() < end:
            for r in self.pump(0.02):
                if r and r[0] == op:
                    return r
        raise Fail(f"no response to command 0x{op:02x}")

    def ok(self, op: int, *args: int) -> bytes:
        r = self.cmd(op, *args)
        if r[1] != L.STATUS_OK:
            raise Fail(f"command 0x{op:02x} failed with status {r[1]}")
        return r

    def config(self) -> dict:
        cfg = L.decode_config(self.ok(L.CMD_GET_CONFIG))
        if cfg is None or "pre_crc" not in cfg:
            raise Fail("this firmware does not report its filter chain - "
                       "flash the current build")
        return cfg


class Report:
    def __init__(self) -> None:
        self.failures = 0

    def check(self, ok: bool, label: str, detail: str = "") -> bool:
        if not ok:
            self.failures += 1
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}"
              + (f"  {detail}" if detail else ""))
        return ok

    @staticmethod
    def info(text: str) -> None:
        print(f"         {text}")


def connect(usb: bool) -> L.Link:
    link = L.UsbLink() if usb else L.BleLink()
    end = time.time() + 45
    while not link.connected and time.time() < end:
        try:
            print("  " + link.status.get(timeout=1))
        except queue.Empty:
            pass
    if not link.connected:
        link.close()
        raise Fail("could not connect")
    time.sleep(1.0)
    return link


def design(rate: int):
    """The chain as the application would send it: 1 Hz, 50 Hz + 100 Hz, 45 Hz."""
    chain = eeg_dsp.Chain(rate, L.CHANNELS)
    chain.highpass_hz = 1.0
    chain.lowpass_hz = 45.0
    chain.notch_hz = 50.0
    chain.notch_track = False
    chain.rebuild()
    return chain.device_stages()


def run(s: Session, rep: Report, rate: int, seconds: float) -> None:
    print("\nsetup")
    s.ok(L.CMD_STREAM_STOP)
    cfg = s.config()
    rep.info(f"device at {cfg['rate']} SPS")

    if cfg["rate"] != rate:
        s.ok(L.CMD_SET_RATE, rate & 0xFF, rate >> 8)
        time.sleep(2.5)
        cfg = s.config()
        if not rep.check(cfg["rate"] == rate, f"rate set to {rate} SPS"):
            raise Fail("rate did not change")

    # Test square wave everywhere, then 5-8 shorted and two odd gains.
    s.ok(L.CMD_SET_INPUT, L.MUX_TEST, 0)
    s.ok(L.CMD_SET_CHANNEL, 1, L.GAIN_CODES[12], L.MUX_TEST, 0, 0)
    for ch in range(4, 8):
        gain = 6 if ch == 6 else 24
        s.ok(L.CMD_SET_CHANNEL, ch, L.GAIN_CODES[gain], L.MUX_SHORTED, 0, 0)

    cfg = s.config()
    gains = cfg["gains"]
    rep.check(gains == [24, 12, 24, 24, 24, 24, 6, 24], "channel gains read back",
              f"{gains}")

    pre, post = design(rate)

    print("\nrefusals")
    r = s.cmd(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_PRE, pre, rate + 1))
    rep.check(r[1] == L.STATUS_BADARG, "sections for another rate are refused")
    bad = [(p[0], 0.0, *p[2:]) for p in pre[:1]]
    r = s.cmd(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_PRE, bad, rate))
    rep.check(r[1] == L.STATUS_BADARG, "an unstable section is refused")

    print("\nload")
    s.ok(L.CMD_SET_ENCODING, L.ENC_RAW_UV)
    s.ok(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_PRE, pre, rate))
    s.ok(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_POST, post, rate))
    s.ok(L.CMD_SET_CAR, 1, 0xFF)

    cfg = s.config()
    rep.check(cfg["pre_count"] == len(pre) and cfg["post_count"] == len(post),
              "section counts read back", f"{cfg['pre_count']} + {cfg['post_count']}")
    rep.check(cfg["pre_crc"] == L.sections_crc(pre)
              and cfg["post_crc"] == L.sections_crc(post),
              "section CRCs match what was sent")
    rep.check(cfg["car"] and cfg["car_mask"] == 0xFF and not cfg["pre_is_notch"],
              "common average on, all channels")

    print(f"\ncapture, {seconds:.0f} s")
    s.data.clear()
    s.capturing = True
    s.ok(L.CMD_STREAM_START)
    s.pump(1.0)
    start = L.response_seq(s.ok(L.CMD_RESET_CHAIN))
    t0 = time.time()
    s.pump(seconds)
    elapsed = time.time() - t0
    s.capturing = False
    s.ok(L.CMD_STREAM_STOP)
    s.pump(0.3)

    rows = {}
    for flags, (_, seq, enc, counts, uv) in s.data:
        if enc != L.ENC_RAW_UV:
            continue
        settling = bool(flags & L.FLAG_SETTLING)
        for i in range(len(counts)):
            rows[seq + i] = (counts[i], uv[i], settling)

    if start is None or start not in rows:
        raise Fail(f"the reset sample {start} never arrived")

    last = max(rows)
    missing = sum(1 for q in range(start, last + 1) if q not in rows)
    n = last - start + 1 - missing
    rep.check(missing == 0, "no sequence gaps", f"{missing} samples missing of {n}")
    rep.check(abs(n / elapsed - rate) / rate < 0.05, "raw + filtered stream keeps up",
              f"{n / elapsed:.1f} SPS delivered at {rate} SPS")

    print("\ncomparison")
    ref = pipeline_ref.Chain(rate, notch_hz=0.0)
    for ch, g in enumerate(gains):
        ref.set_gain(ch, g)
    ref.set_stage(pipeline_ref.STAGE_PRE, [tuple(F32(v) for v in p) for p in pre])
    ref.set_stage(pipeline_ref.STAGE_POST, [tuple(F32(v) for v in p) for p in post])
    ref.set_car(True, 0xFF)

    got, want = [], []
    for q in range(start, last + 1):
        if q not in rows:
            break
        counts, uv, _ = rows[q]
        got.append(uv)
        want.append(ref.process_counts(counts))
    got = np.array(got, dtype=np.float64)
    want = np.array(want, dtype=np.float64)

    err = np.abs(got - want)
    rep.info(f"{len(got)} samples x 8 channels from sample {start}; "
             f"signal to {np.abs(want).max():.0f} uV")
    rep.check(err.max() <= MATCH_UV, "device output matches the reference",
              f"worst {err.max():.2e} uV, RMS {np.sqrt(np.mean(err ** 2)):.2e} uV")
    worst_ch = int(np.argmax(err.max(axis=0)))
    rep.info(f"worst on CH{worst_ch + 1}")

    # The square wave is common to channels 1-4 and absent from 5-8, so the
    # average moves half of it onto the shorted channels. Seeing it there
    # proves the average ran, not just that both sides skipped it.
    shorted = np.abs(got[len(got) // 2:, 4:]).max()
    rep.check(shorted > 100.0, "common average reached the shorted channels",
              f"{shorted:.0f} uV on CH5-8")

    expect = ref.settle_samples()
    flagged = 0
    for q in range(start, last + 1):
        if q not in rows or not rows[q][2]:
            break
        flagged += 1
    rep.check(abs(flagged - expect) <= 6, "restart flagged as settling",
              f"{flagged} samples flagged, about {expect} expected")


def restore(s: Session, rate_before: int | None) -> None:
    for op, *args in ((L.CMD_STREAM_STOP,),
                      (L.CMD_SET_CAR, 0, 0xFF),
                      (L.CMD_SET_NOTCH, 50),
                      (L.CMD_SET_INPUT, L.MUX_NORMAL, 0),
                      (L.CMD_SET_ENCODING, L.ENC_RAW_I24)):
        try:
            s.cmd(op, *args)
        except Fail:
            pass
    try:
        cfg = s.config()
        s.cmd(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_POST, [], cfg["rate"]))
        if rate_before and cfg["rate"] != rate_before:
            s.cmd(L.CMD_SET_RATE, rate_before & 0xFF, rate_before >> 8)
            # The restart takes about a second; stay connected through it.
            time.sleep(2.5)
            if s.config()["rate"] != rate_before:
                print(f"warning: the rate did not go back to {rate_before} SPS",
                      file=sys.stderr)
    except Fail:
        pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usb", action="store_true", help="USB instead of Bluetooth")
    ap.add_argument("--rate", type=int, default=250, choices=(250, 500, 1000))
    ap.add_argument("--seconds", type=float, default=12.0)
    args = ap.parse_args()

    print(f"SwiftEEG on-device chain check, {args.rate} SPS over "
          f"{'USB' if args.usb else 'Bluetooth'}")
    rep = Report()
    link = None
    s = None
    rate_before = None
    try:
        link = connect(args.usb)
        s = Session(link)
        rate_before = s.config()["rate"]
        run(s, rep, args.rate, args.seconds)
    except Fail as exc:
        print(f"\naborted: {exc}", file=sys.stderr)
        rep.failures += 1
    finally:
        if s is not None:
            restore(s, rate_before)
        if link is not None:
            link.close()
            time.sleep(0.5)

    print()
    if rep.failures:
        print(f"{rep.failures} check(s) FAILED")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
