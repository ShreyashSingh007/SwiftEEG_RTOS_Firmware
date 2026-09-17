"""
The app with its filters on a simulated device.

The fake device runs tools/pipeline_ref.py's model of the firmware chain and
answers the filter commands the way command.c does. This checks that the app
sends the right sections, keeps them in step as settings change, records raw
while showing the device's output, and falls back to the PC when refused.
"""
import csv
import os
import pathlib
import queue
import struct
import sys
import threading
import time
import traceback
import types

import numpy as np

SCRATCH = pathlib.Path(__file__).resolve().parent
TOOLS = r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools"
sys.path.insert(0, TOOLS)
import pipeline_ref as ref  # noqa: E402
import swifteeg_app as appmod  # noqa: E402
import swifteeg_link as link  # noqa: E402

F32 = np.float32


def frame(type_, payload, flags=0):
    return types.SimpleNamespace(type=type_, payload=payload, flags=flags)


def i24(v):
    u = int(v) & 0xFFFFFF
    return bytes((u & 0xFF, (u >> 8) & 0xFF, (u >> 16) & 0xFF))


class FakeDevice:
    """Streams like the board; handles filter commands like command.c."""

    def __init__(self, rate=250):
        self.frames = queue.Queue()
        self.status = queue.Queue()
        self.connected = True
        self.bad_frames = 0
        self.lock = threading.Lock()
        self.sent = []
        self.refuse_filter = False
        self.enc = link.ENC_RAW_I24
        self.streaming = False
        self.record = {}
        self.notch = (50, 12, True, True)
        self.mains = None
        self.aim = 50.0
        self._set_rate(rate)
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _set_rate(self, rate):
        self.rate = rate
        self.chain = ref.Chain(rate)
        self.seq = 0
        self.settle_left = 0
        self.t0 = time.perf_counter()
        self.emitted = 0
        # The mains is agreed on 3 s into streaming, as the tracker would.
        self.mains_at = 3 * rate
        self._aim_notch(keep=False)

    def _aim_notch(self, keep):
        """The device's rule: aim at the measurement when following it."""
        hz, q, harmonic, track = self.notch
        aim = (self.mains if track and self.mains and abs(self.mains - hz) <= 1.0
               else float(hz))
        if not self.chain.set_notch(aim, q, harmonic, keep_state=keep):
            self._restart()
        self.aim = aim

    def _measure(self, hz):
        """A mains frequency agreed on: follow it, and say so."""
        self.mains = hz
        moved = bool(self.notch[3] and abs(hz - self.aim) >= 0.03)
        if moved:
            self._aim_notch(keep=True)
        payload = bytes([link.EVT_MAINS, 1 if moved else 0]) + struct.pack("<If", self.seq, hz)
        self.frames.put(frame(link.TYPE_EVT, payload))

    def close(self):
        self._stop.set()

    def _rsp(self, op, status=0, extra=b""):
        self.frames.put(frame(link.TYPE_RSP, bytes([op, status]) + extra))

    def _restart(self):
        self.settle_left = self.chain.settle_samples()

    def send(self, op, *args):
        with self.lock:
            self.sent.append((op, list(args)))
            b = bytes(args)
            if op == link.CMD_STREAM_START:
                self.streaming = True
                self.t0 = time.perf_counter()
                self.emitted = 0
                self._rsp(op)
            elif op == link.CMD_STREAM_STOP:
                self.streaming = False
                self._rsp(op)
            elif op == link.CMD_SET_ENCODING:
                self.enc = b[0]
                self._rsp(op)
            elif op == link.CMD_GET_CONFIG:
                self._rsp(op, 0, self._config())
            elif op == link.CMD_SET_FILTER:
                self._set_filter(op, b)
            elif op == link.CMD_SET_CAR:
                self.chain.set_car(bool(b[0]), b[1])
                self._rsp(op, 0, struct.pack("<I", self.seq))
            elif op == link.CMD_SET_NOTCH:
                if b[0] != self.notch[0]:
                    self.mains = None
                self.notch = (b[0], b[1], bool(b[2] & 1), bool(b[2] & 2))
                self._aim_notch(keep=True)
                self._rsp(op, 0, struct.pack("<I", self.seq))
            elif op == link.CMD_SET_RATE:
                self._rsp(op)
                self._set_rate(b[0] | (b[1] << 8))
            else:
                self._rsp(op)

    def _set_filter(self, op, b):
        stage, flags, fs, count = b[0], b[1], b[2] | (b[3] << 8), b[4]
        body = b[5:]
        if (self.refuse_filter or fs != self.rate or stage > 1
                or len(body) != 20 * count):
            self._rsp(op, link.STATUS_BADARG)
            return
        sections = [struct.unpack_from("<5f", body, 20 * i) for i in range(count)]
        try:
            kept = self.chain.set_stage(stage, sections, keep_state=bool(flags & 1))
        except ValueError:
            self._rsp(op, link.STATUS_BADARG)
            return
        if not kept:
            self._restart()
        self._rsp(op, 0, struct.pack("<I", self.seq))

    def _config(self):
        pre = [tuple(s) for s in self.chain.pre.sections]
        post = [tuple(s) for s in self.chain.post.sections]
        cfg = bytearray(37)
        cfg[0], cfg[1] = 8, self.enc
        cfg[2:4] = struct.pack("<H", self.rate)
        cfg[4] = self.notch[0]
        cfg[5:13] = bytes([0x60] * 8)
        cfg[13] = 0x02
        cfg[14:16] = struct.pack("<H", 240)
        cfg[16] = 8
        cfg[17:19] = struct.pack("<H", 2000)
        cfg[19], cfg[20] = len(pre), len(post)
        cfg[21] = 0x01 if self.chain.car else 0
        cfg[22] = self.chain.mask
        cfg[23:25] = struct.pack("<H", link.sections_crc(pre))
        cfg[25:27] = struct.pack("<H", link.sections_crc(post))
        cfg[27] = self.notch[1]
        cfg[28] = ((link.NOTCH_HARMONIC if self.notch[2] else 0)
                   | (link.NOTCH_TRACK if self.notch[3] else 0)
                   | (link.NOTCH_MEASURED if self.mains else 0))
        cfg[29:33] = struct.pack("<f", self.mains or 0.0)
        cfg[33:37] = struct.pack("<f", self.aim)
        return bytes(cfg)

    def _run(self):
        lsb = link.lsb_uv(24)
        offsets = np.arange(8) * 20000.0 - 70000.0
        while not self._stop.is_set():
            time.sleep(0.012)
            with self.lock:
                if not self.streaming:
                    continue
                batch = 3 if self.enc == link.ENC_RAW_UV else 6
                due = int((time.perf_counter() - self.t0) * self.rate)
                while self.emitted + batch <= due:
                    self._emit(batch, lsb, offsets)
                if self.mains_at is not None and self.emitted >= self.mains_at:
                    self.mains_at = None
                    self._measure(49.69)

    def _emit(self, n, lsb, offsets):
        seq0 = self.seq
        body = bytearray()
        settling = False
        for i in range(n):
            t = (seq0 + i) / self.rate
            uv = (offsets + 30 * np.sin(2 * np.pi * 10 * t + np.arange(8))
                  + 15 * np.sin(2 * np.pi * 49.7 * t)
                  + 200 * np.sin(2 * np.pi * 0.2 * t))
            c = np.round(uv / lsb).astype(np.int64)
            out = self.chain.process_counts(c)
            if self.settle_left:
                self.settle_left -= 1
                settling = True
            self.record[seq0 + i] = (c.copy(), out.copy())
            for ch in range(8):
                body += i24(c[ch])
                if self.enc == link.ENC_RAW_UV:
                    body += struct.pack("<f", float(out[ch]))
        self.seq += n
        self.emitted += n
        payload = struct.pack("<QIBBH", int(seq0 * 1e6 / self.rate), seq0, 8,
                              self.enc, n) + bytes(body)
        self.frames.put(frame(link.TYPE_DATA, payload,
                              link.FLAG_SETTLING if settling else 0))

    def sent_since(self, mark, op=None):
        return [(o, a) for o, a in self.sent[mark:] if op is None or o == op]


class Checks:
    def __init__(self):
        self.failures = []
        self.errors = []

    def __call__(self, ok, label, detail=""):
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f"  {detail}" if detail else ""))
        if not ok:
            self.failures.append(label)


def f32(sections):
    return [tuple(F32(v) for v in s) for s in sections]


def main():
    os.chdir(SCRATCH)
    check = Checks()

    app = appmod.App()
    appmod.traceback.print_exc = lambda: check.errors.append(traceback.format_exc())
    app.report_callback_exception = \
        lambda *exc: check.errors.append("".join(traceback.format_exception(*exc)))

    # The defaults asked for: drift cut 1 Hz, smooth above 100 Hz, no common
    # average and no channel in it, +/-200 uV over 3 s.
    print("defaults")
    check(app.hp_var.get() == "1.0" and app.lp_var.get() == "100",
          "drift cut 1 Hz, smooth above 100 Hz",
          f"{app.hp_var.get()} / {app.lp_var.get()}")
    check(not app.car_var.get() and not app.chain.car, "common average off")
    check(not any(v.get() for v in app.in_avg) and not app.chain.car_mask.any(),
          "no channel in the average")
    check(app.scale_var.get() == "200" and app.window_var.get() == "3",
          "range +/-200 uV, window 3 s")
    check(app.chain.highpass_hz == 1.0 and app.chain.lowpass_hz == 100.0,
          "the chain has the controls' settings")

    # The rest exercises the average on the device, so switch it on.
    app.car_var.set(True)
    for v in app.in_avg:
        v.set(True)
    app._rebuild_chain()
    app._set_car_mask()

    dev = FakeDevice(250)
    app.link = dev
    app._was_connected = False
    marks = {}

    def step(ms, fn):
        def run():
            try:
                fn()
            except Exception:  # noqa: BLE001
                check.errors.append(traceback.format_exc())
        app.after(ms, run)

    def to_device():
        marks["device"] = len(dev.sent)
        app.site_var.set("device")
        app._set_filter_site()

    def device_mode():
        print("device mode")
        pre, post = app.chain.device_stages()
        check(dev.enc == link.ENC_RAW_UV, "stream switched to raw + filtered")
        check(app.filters_on_device, "app reports filters on the device")
        check(dev.chain.pre.sections == f32(pre), "device holds the pre sections",
              f"{len(dev.chain.pre.sections)} sections")
        check(dev.chain.post.sections == f32(post), "device holds the post sections")
        check(dev.chain.car and dev.chain.mask == app.chain.car_bits,
              "device average matches", hex(dev.chain.mask))
        check(app.site_label.cget("text").startswith("on the device from sample"),
              "site label", app.site_label.cget("text"))

        k = 50
        last = app.last_seq
        shown = app._ring_read(app.n_total - k, app.n_total)
        want = np.array([dev.record[q][1] for q in range(last - k, last)]).T
        err = np.abs(shown - want).max()
        check(err < 1e-3, "plot shows the device's own output", f"max diff {err:.2e} uV")

        check(app.chain.measured_mains is not None
              and abs(app.chain.measured_mains - 49.69) < 1e-3,
              "the app shows the mains the device measured",
              f"{app.chain.measured_mains}")
        check(app.chain.notch_aim is not None and abs(app.chain.notch_aim - 49.69) < 1e-3
              and abs(dev.aim - 49.69) < 1e-3,
              "both notches aimed where the device's is",
              f"app {app.chain.notch_aim}, device {dev.aim}")
        check(dev.notch == app.chain.device_notch(), "device notch settings follow the app",
              f"{dev.notch}")
        marks["lp"] = len(dev.sent)
        app.lp_var.set("30")
        app._rebuild_chain()

    def lowpass_only():
        sent = dev.sent_since(marks["lp"], link.CMD_SET_FILTER)
        check(len(sent) == 1 and sent[0][1][0] == link.STAGE_POST,
              "changing the low-pass sends only the post stage",
              f"{[(a[0], a[4]) for _, a in sent]}")
        check(len(sent) == 1 and sent[0][1][1] == 1,
              "and retunes it in place rather than restarting it")
        check(dev.chain.post.sections == f32(app.chain.device_stages()[1]),
              "device low-pass updated")
        marks["car"] = len(dev.sent)
        app.in_avg[3].set(False)
        app._set_car_mask()

    def car():
        sent = dev.sent_since(marks["car"], link.CMD_SET_CAR)
        check(sent and sent[-1][1] == [1, 0xF7], "unticking CH4 sends the mask",
              f"{sent}")
        check(dev.chain.mask == 0xF7, "device average excludes CH4")
        app._toggle_record()
        marks["rec_name"] = app._rec_file.name
        marks["harm"] = len(dev.sent)
        app.harm_var.set(False)
        app._rebuild_chain()

    def recording():
        sent = dev.sent_since(marks["harm"], link.CMD_SET_NOTCH)
        check(len(sent) == 1 and dev.notch == (50, 12, False, True),
              "turning the harmonic off reaches the device notch", f"{dev.notch}")
        check(len(dev.chain.notch.sections) == 1, "which is down to one section")
        app.harm_var.set(True)
        app._rebuild_chain()
        app._toggle_record()
        rows = list(csv.reader(open(marks["rec_name"], newline="")))
        check(rows[0][0] == "# SwiftEEG raw counts", "recording header says raw",
              rows[0][0])
        data = rows[2:]
        bad = 0
        for r in data:
            want = dev.record[int(r[1])][0]
            if [int(v) for v in r[2:]] != [int(v) for v in want]:
                bad += 1
        check(len(data) > 300 and bad == 0, "recording is the raw counts",
              f"{len(data)} rows, {bad} differ")
        dev.refuse_filter = True
        marks["refuse"] = len(dev.sent)
        app.hp_var.set("0.5")
        app._rebuild_chain()

    def fallback():
        check(not app.filters_on_device and app.site_var.get() == "PC",
              "a refusal puts the filters back on the PC")
        check("refused" in app.site_label.cget("text"), "and says so",
              app.site_label.cget("text"))
        check(dev.enc == link.ENC_RAW_I24, "stream back to raw only")
        dev.refuse_filter = False
        to_device()

    def rate_change():
        marks["rate"] = len(dev.sent)
        app.rate_var.set("500")
        app._set_rate()

    def after_rate():
        pre, post = app.chain.device_stages()
        check(dev.rate == 500 and app.rate == 500, "rate changed to 500")
        sent = dev.sent_since(marks["rate"], link.CMD_SET_FILTER)
        check(any(a[2] | (a[3] << 8) == 500 for _, a in sent),
              "sections re-sent designed for 500 SPS")
        check(dev.chain.pre.sections == f32(pre) and dev.chain.post.sections == f32(post),
              "device holds the 500 SPS chain")
        ops = [o for o, _ in dev.sent_since(marks["rate"])]
        start = ops.index(link.CMD_STREAM_START) if link.CMD_STREAM_START in ops else -1
        last_filter = max((i for i, o in enumerate(ops) if o == link.CMD_SET_FILTER),
                          default=-1)
        check(0 <= last_filter < start, "filters sent before the stream restarted",
              f"{ops}")
        check(app.samples > 0 and app.gaps == 0, "samples flowing again, no gaps",
              f"{app.samples} samples, {app.gaps} gaps")
        marks["double"] = len(dev.sent)
        app.rate_var.set("1000")
        app._set_rate()
        app.rate_var.set("500")
        app._set_rate()

    def after_double():
        pre, post = app.chain.device_stages()
        rates = [a[0] | (a[1] << 8)
                 for _, a in dev.sent_since(marks["double"], link.CMD_SET_RATE)]
        check(rates == [1000, 500], "two quick rate changes both sent", f"{rates}")
        check(dev.rate == 500 and app.rate == 500 and app.rate_var.get() == "500",
              "they end at the last one", f"device {dev.rate}, app {app.rate}")
        check(dev.streaming and app._rate_change is None, "and the stream is back")
        check(dev.chain.pre.sections == f32(pre) and dev.chain.post.sections == f32(post),
              "device holds the chain for the last rate")
        check(app.samples > 0 and app.gaps == 0, "samples flowing, no gaps",
              f"{app.samples} samples, {app.gaps} gaps")
        marks["pc"] = len(dev.sent)
        app.site_var.set("PC")
        app._set_filter_site()

    def back_on_pc():
        check(dev.enc == link.ENC_RAW_I24, "back on the PC: raw only")
        check(dev.chain.pre.sections == [] and dev.chain.post.sections == []
              and not dev.chain.car and len(dev.chain.notch.sections) == 2,
              "device returned to its own default: its notch alone")
        check(not app.filters_on_device, "app reports filters on the PC")

    step(300, app._toggle_stream)
    step(1500, to_device)
    step(6500, device_mode)
    step(7600, lowpass_only)
    step(8200, car)
    step(10300, recording)
    step(11400, fallback)
    step(12600, rate_change)
    step(17500, after_rate)
    step(20500, after_double)
    step(21600, back_on_pc)
    step(22200, app.quit)
    app.mainloop()

    dev.close()
    app.pacer.close()
    app.destroy()

    for e in check.errors:
        print("ERROR:\n" + e)
    ok = not check.failures and not check.errors
    print("app device test:", "OK" if ok else
          f"{len(check.failures)} failed, {len(check.errors)} errors")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
