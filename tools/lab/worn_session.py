"""
A session on the worn headset, over Bluetooth.

  sit       the wearer sits; the head moves now and then. Real EEG, motion
            and the mains the device measures. 180 s by default.
  rates     250 -> 500 -> 1000 -> 250 SPS while streaming, the way the
            application changes rate: stop, set the rate, read the
            configuration, send the filters, start.
  settings  the application's settings changed while streaming - drift cut,
            smoothing, notch, common average - each from the sample the device
            reports, after a chain restart, so every change can be replayed
            exactly offline.

Every frame is kept: raw counts and the device's microvolts, motion, mains
events, and each command with its reply. One file per part, written when the
part ends, so the analysis can start while the rest records.

A low battery has shown as noise on every channel that stays. A watch on the
live stream compares the 60-90 Hz noise with the session's opening level and
says so when it holds at four times that for ten seconds.

    python -u worn_session.py --out DIR [--sit 180] [--parts sit,rates,settings]
"""
from __future__ import annotations

import argparse
import json
import pathlib
import queue
import sys
import time

import numpy as np

TOOLS = r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools"
sys.path.insert(0, TOOLS)
import eeg_dsp  # noqa: E402
import swifteeg_link as L  # noqa: E402

HP_HZ, LP_HZ = 1.0, 100.0                 # the application's defaults
IMU_RATE, IMU_G, IMU_DPS = 240, 8, 2000   # the motion defaults

REGISTERS = (("CONFIG3", 0x03), ("BIAS_SENSP", 0x0D), ("BIAS_SENSN", 0x0E),
             ("MISC1", 0x15))

NOISE_BAND = (60.0, 90.0)
NOISE_RATIO, NOISE_HOLD_S, NOISE_BASE_S, NOISE_SKIP_S = 4.0, 10, 20, 5
RAIL = (1 << 23) - 64


class Fail(Exception):
    pass


class Lost(Exception):
    pass


class Tee:
    """Console and a log file, flushed line by line."""

    encoding = "utf-8"

    def __init__(self, path: pathlib.Path):
        self.f = open(path, "a", encoding="utf-8")

    def isatty(self):
        return False

    def write(self, s):
        sys.__stdout__.write(s)
        self.f.write(s)

    def flush(self):
        sys.__stdout__.flush()
        self.f.flush()


def band_rms(x: np.ndarray, fs: float, lo: float, hi: float) -> np.ndarray:
    """RMS per channel of x (samples, channels) between lo and hi Hz."""
    n = len(x)
    t = np.arange(n) - (n - 1) / 2.0
    x = x - x.mean(axis=0)
    x = x - np.outer(t, (t @ x) / (t @ t))
    w = np.hanning(n)
    power = np.abs(np.fft.rfft(x * w[:, None], axis=0)) ** 2
    f = np.fft.rfftfreq(n, 1.0 / fs)
    sel = (f >= lo) & (f <= hi)
    return np.sqrt(2.0 * power[sel].sum(axis=0) / (n * n * np.mean(w * w)))


def stages(rate: int, hp: float = HP_HZ, lp: float = LP_HZ):
    chain = eeg_dsp.Chain(rate, L.CHANNELS)
    chain.highpass_hz, chain.lowpass_hz = hp, lp
    chain.rebuild()
    return chain.device_stages()


def jsonable(cfg: dict) -> dict:
    return {k: (list(v) if isinstance(v, (bytes, bytearray)) else v)
            for k, v in cfg.items()}


class NoiseWatch:
    """Second by second: the median over channels of 60-90 Hz and 1-40 Hz RMS."""

    def __init__(self, t0: float):
        self.t0 = t0
        self.rate = 0
        self.lsb = np.full(L.CHANNELS, L.lsb_uv(24))
        self.parts, self.have = [], 0
        self.first = None
        self.base = None
        self.noise, self.eeg = [], []
        self.run = 0
        self.episodes = []

    def configure(self, rate: int, gains) -> None:
        self.rate = int(rate)
        self.lsb = np.array([L.lsb_uv(g or 24) for g in gains])
        self.parts, self.have = [], 0

    def feed(self, counts, now: float) -> None:
        if counts is None or not self.rate:
            return
        self.parts.append(counts)
        self.have += len(counts)
        while self.have >= self.rate:
            block = np.vstack(self.parts)
            one, rest = block[:self.rate], block[self.rate:]
            self.parts, self.have = ([rest] if len(rest) else []), len(rest)
            self._second(one, now)

    def _second(self, counts: np.ndarray, now: float) -> None:
        ok = np.abs(counts).max(axis=0) < RAIL
        if not ok.any():
            return
        x = counts.astype(np.float64) * self.lsb
        noise = float(np.median(band_rms(x, self.rate, *NOISE_BAND)[ok]))
        eeg = float(np.median(band_rms(x, self.rate, 1.0, 40.0)[ok]))
        self.noise.append((now, noise))
        self.eeg.append((now, eeg))
        if self.first is None:
            self.first = now

        if self.base is None:
            early = [v for t, v in self.noise if t - self.first >= NOISE_SKIP_S]
            if len(early) >= NOISE_BASE_S:
                self.base = float(np.median(early))
                opening_eeg = float(np.median([v for _, v in self.eeg[-NOISE_BASE_S:]]))
                print(f"  noise watch: opening level {self.base:.2f} uV at 60-90 Hz, "
                      f"EEG {opening_eeg:.1f} uV at 1-40 Hz", flush=True)
            return

        ratio = noise / self.base
        if ratio >= NOISE_RATIO:
            self.run += 1
            if self.run == NOISE_HOLD_S:
                self.episodes.append([now - self.t0 - NOISE_HOLD_S, now - self.t0, ratio])
                print(f"  !! NOISE WARNING at {now - self.t0:.0f} s: 60-90 Hz noise "
                      f"{noise:.1f} uV, {ratio:.1f}x the opening level for "
                      f"{NOISE_HOLD_S} s - low battery?", flush=True)
            elif self.run > NOISE_HOLD_S:
                ep = self.episodes[-1]
                ep[1], ep[2] = now - self.t0, max(ep[2], ratio)
        else:
            if self.run > NOISE_HOLD_S:
                print(f"  noise back down to {noise:.1f} uV after {self.run} s", flush=True)
            self.run = 0


class Recorder:
    def __init__(self, link: L.Link, out: pathlib.Path):
        self.link, self.out = link, out
        self.t0 = time.time()
        self.rsps: list = []
        self.watch = NoiseWatch(self.t0)
        self.last_status = time.time()
        self.samples_since = 0
        self.gyro_peak = 0.0
        self.last_mains = None
        self.begin("setup", {})

    def begin(self, name: str, meta: dict) -> None:
        self.name = name
        self.meta = dict(meta, part=name, started=time.time() - self.t0)
        self.eeg, self.imu, self.evt = [], [], []
        self.cmds, self.changes = [], []

    def _check_link(self) -> None:
        while True:
            try:
                print("  link:", self.link.status.get_nowait(), flush=True)
            except queue.Empty:
                break
        if not self.link.connected:
            raise Lost("the Bluetooth link was lost")

    def pump(self, seconds: float) -> None:
        end = time.time() + seconds
        while True:
            left = end - time.time()
            if left <= 0:
                return
            try:
                f = self.link.frames.get(timeout=min(0.02, left))
            except queue.Empty:
                self._check_link()
                continue
            now = time.time()

            if f.type == L.TYPE_DATA:
                d = L.decode_data(f.payload)
                if d is None:
                    continue
                ts, seq, enc, counts, uv = d
                counts = None if counts is None else np.array(counts, dtype=np.int32)
                uv = None if uv is None else np.array(uv, dtype=np.float32)
                self.eeg.append((now, f.flags, ts, seq, counts, uv))
                self.samples_since += len(counts if counts is not None else uv)
                self.watch.feed(counts, now)
            elif f.type == L.TYPE_IMU:
                d = L.decode_imu(f.payload)
                if d is None:
                    continue
                ts, seq, period, c, g, dps, flags = d
                c = np.array(c, dtype=np.int16)
                self.imu.append((now, ts, seq, period, c, g, dps, flags))
                if len(c):
                    gyro = np.sqrt((c[:, 3:].astype(np.float64) ** 2).sum(axis=1)).max()
                    self.gyro_peak = max(self.gyro_peak, gyro * dps / 32768.0)
            elif f.type == L.TYPE_EVT:
                ev = L.decode_event(bytes(f.payload))
                if ev is not None:
                    self.evt.append((now, ev))
                    self.last_mains = ev["hz"]
            elif f.type == L.TYPE_RSP:
                self.rsps.append((now, bytes(f.payload)))

            if now - self.last_status >= 15.0:
                self._status(now)

    def _status(self, now: float) -> None:
        dt = now - self.last_status
        w = self.watch
        noise = w.noise[-1][1] if w.noise else float("nan")
        eeg = w.eeg[-1][1] if w.eeg else float("nan")
        base = w.base if w.base is not None else float("nan")
        mains = self.last_mains if self.last_mains is not None else float("nan")
        print(f"  [{now - self.t0:6.1f} s] {self.name}: {self.samples_since / dt:6.1f} SPS in; "
              f"noise {noise:.2f} uV (opening {base:.2f}); EEG {eeg:.1f} uV; head turn "
              f"peak {self.gyro_peak:.0f} deg/s; mains {mains:.3f} Hz", flush=True)
        self.samples_since, self.gyro_peak, self.last_status = 0, 0.0, now

    def cmd(self, op: int, *args: int, wait: float = 5.0) -> bytes | None:
        n = len(self.rsps)
        sent = time.time()
        self.link.send(op, *args)
        end = sent + wait
        while time.time() < end:
            self.pump(0.02)
            for when, r in self.rsps[n:]:
                if r and r[0] == op:
                    self.cmds.append({"t": sent - self.t0, "answered": when - self.t0,
                                      "op": op, "args": list(args)[:5], "status": r[1],
                                      "seq": L.response_seq(r)})
                    return r
        self.cmds.append({"t": sent - self.t0, "answered": None, "op": op,
                          "args": list(args)[:5]})
        return None

    def ok(self, op: int, *args: int, wait: float = 5.0) -> bytes:
        r = self.cmd(op, *args, wait=wait)
        if r is None:
            raise Fail(f"no reply to command 0x{op:02x}")
        if r[1] != L.STATUS_OK:
            raise Fail(f"command 0x{op:02x} refused with status {r[1]}")
        return r

    def config(self, wait: float = 5.0) -> dict:
        cfg = L.decode_config(self.ok(L.CMD_GET_CONFIG, wait=wait))
        if cfg is None or "notch_q" not in cfg:
            raise Fail("the configuration could not be read")
        return cfg

    def save(self) -> None:
        eeg, imu, evt = self.eeg, self.imu, self.evt
        lens = [len(e[4]) if e[4] is not None else len(e[5]) for e in eeg]
        counts = [e[4] if e[4] is not None else np.zeros((k, L.CHANNELS), np.int32)
                  for e, k in zip(eeg, lens)]
        uv = [e[5] if e[5] is not None else np.full((k, L.CHANNELS), np.nan, np.float32)
              for e, k in zip(eeg, lens)]
        arrays = dict(
            eeg_arrival=np.array([e[0] - self.t0 for e in eeg], dtype=np.float64),
            eeg_flags=np.array([e[1] for e in eeg], dtype=np.uint8),
            eeg_ts=np.array([e[2] for e in eeg], dtype=np.uint64),
            eeg_seq=np.array([e[3] for e in eeg], dtype=np.int64),
            eeg_len=np.array(lens, dtype=np.int32),
            counts=np.vstack(counts) if counts else np.zeros((0, L.CHANNELS), np.int32),
            uv=np.vstack(uv) if uv else np.zeros((0, L.CHANNELS), np.float32),
            imu_arrival=np.array([i[0] - self.t0 for i in imu], dtype=np.float64),
            imu_ts=np.array([i[1] for i in imu], dtype=np.uint64),
            imu_seq=np.array([i[2] for i in imu], dtype=np.int64),
            imu_period=np.array([i[3] for i in imu], dtype=np.float64),
            imu_len=np.array([len(i[4]) for i in imu], dtype=np.int32),
            imu_accel_g=np.array([i[5] for i in imu], dtype=np.uint16),
            imu_gyro_dps=np.array([i[6] for i in imu], dtype=np.uint16),
            imu_flags=np.array([i[7] for i in imu], dtype=np.uint8),
            imu_counts=(np.vstack([i[4] for i in imu]) if imu
                        else np.zeros((0, 6), np.int16)),
            evt_arrival=np.array([e[0] - self.t0 for e in evt], dtype=np.float64),
            evt_seq=np.array([e[1]["seq"] for e in evt], dtype=np.int64),
            evt_hz=np.array([e[1]["hz"] for e in evt], dtype=np.float64),
            evt_moved=np.array([e[1]["moved"] for e in evt], dtype=bool),
        )
        self.meta.update(commands=self.cmds, changes=self.changes,
                         saved=time.time() - self.t0)
        path = self.out / f"{self.name}.npz"
        np.savez_compressed(path, meta=np.array(json.dumps(self.meta)), **arrays)
        print(f"  saved {path.name}: {int(sum(lens))} EEG samples, "
              f"{int(arrays['imu_len'].sum())} motion samples, {len(evt)} mains events",
              flush=True)


def describe(cfg: dict, regs: dict) -> None:
    inputs = {L.MUX_NORMAL: "electrodes", L.MUX_SHORTED: "shorted", L.MUX_TEST: "test"}
    print(f"  rate {cfg['rate']} SPS, encoding {cfg['encoding']}, gains {cfg['gains']}")
    print(f"  inputs {[inputs.get(m, hex(m)) for m in cfg['mux']]}")
    print(f"  motion {'on' if cfg.get('imu_on') else 'off'} {cfg.get('imu_rate')} Hz, "
          f"+/-{cfg.get('imu_accel_g')} g, +/-{cfg.get('imu_gyro_dps')} dps")
    print(f"  filters {cfg.get('pre_count')} + {cfg.get('post_count')} sections; common "
          f"average {'on' if cfg.get('car') else 'off'} (0x{cfg.get('car_mask', 0):02x})")
    print(f"  notch {cfg['notch']} Hz, Q {cfg.get('notch_q')}, harmonic "
          f"{cfg.get('notch_harmonic')}, following {cfg.get('notch_track')}, measured "
          f"{cfg.get('mains_hz')}, aimed at {cfg.get('notch_aim_hz')}")
    c3, sp, sn, m1 = (regs.get(k) for k in ("CONFIG3", "BIAS_SENSP", "BIAS_SENSN", "MISC1"))
    print(f"  bias drive {'on' if c3 is not None and c3 & 0x04 else 'OFF'} "
          f"(CONFIG3 {c3 if c3 is None else hex(c3)}), bias from P "
          f"{sp if sp is None else hex(sp)} N {sn if sn is None else hex(sn)}; SRB1 "
          f"{'on' if m1 is not None and m1 & 0x20 else 'OFF'} "
          f"(MISC1 {m1 if m1 is None else hex(m1)})")


def load_filters(rec: Recorder, rate: int, hp: float = HP_HZ, lp: float = LP_HZ,
                 keep: bool = False):
    pre, post = stages(rate, hp, lp)
    rec.ok(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_PRE, pre, rate, keep_state=keep))
    rec.ok(L.CMD_SET_FILTER, *L.filter_args(L.STAGE_POST, post, rate, keep_state=keep))
    return pre, post


def change_rate(rec: Recorder, rate: int):
    """SET_RATE is answered before the restart; GET_CONFIG after it."""
    t_set = time.time()
    rec.ok(L.CMD_SET_RATE, rate & 0xFF, rate >> 8)
    t_ack = time.time()
    cfg = rec.config(wait=8.0)
    t_cfg = time.time()
    if cfg["rate"] != rate:
        raise Fail(f"the device is at {cfg['rate']} SPS after asking for {rate}")
    return cfg, {"set": t_set - rec.t0, "ack": t_ack - rec.t0, "config": t_cfg - rec.t0}


def part_sit(rec: Recorder, seconds: float) -> None:
    print(f"\nsit: {seconds:.0f} s", flush=True)
    cfg = rec.config()
    rate = cfg["rate"]
    rec.watch.configure(rate, cfg["gains"])
    pre, post = load_filters(rec, rate)
    rec.begin("p1_sit", {"rate": rate, "gains": cfg["gains"], "config": jsonable(cfg),
                         "hp": HP_HZ, "lp": LP_HZ, "pre": pre, "post": post})
    rec.ok(L.CMD_STREAM_START)
    rec.pump(seconds)
    rec.ok(L.CMD_STREAM_STOP)
    rec.pump(0.3)
    rec.meta["config_after"] = jsonable(rec.config())
    rec.save()


def part_rates(rec: Recorder) -> None:
    print("\nrates", flush=True)
    for rate, seconds in ((500, 40.0), (1000, 40.0), (250, 30.0)):
        before = rec.config()
        rec.begin(f"p2_rate{rate}", {"rate": rate, "rate_before": before["rate"],
                                     "gains": before["gains"]})
        # Streaming when the rate changes, as it is in the application.
        rec.ok(L.CMD_STREAM_START)
        rec.pump(5.0)
        t_stop = time.time()
        rec.ok(L.CMD_STREAM_STOP)
        rec.pump(0.3)
        rec.meta["split"] = {"eeg": len(rec.eeg), "imu": len(rec.imu)}
        cfg, times = change_rate(rec, rate)
        rec.watch.configure(rate, cfg["gains"])
        pre, post = load_filters(rec, rate)
        t_start = time.time()
        rec.ok(L.CMD_STREAM_START)
        times.update(stop=t_stop - rec.t0, start=t_start - rec.t0)
        rec.meta.update(times=times, config=jsonable(cfg), pre=pre, post=post,
                        hp=HP_HZ, lp=LP_HZ)
        print(f"  {before['rate']} -> {rate} SPS: reply {1000 * (times['ack'] - times['set']):.0f} ms, "
              f"configuration {1000 * (times['config'] - times['set']):.0f} ms, streaming again "
              f"{1000 * (times['start'] - times['stop']):.0f} ms after the stop", flush=True)
        rec.pump(seconds)
        rec.ok(L.CMD_STREAM_STOP)
        rec.pump(0.3)
        rec.save()


def part_settings(rec: Recorder) -> None:
    print("\nsettings", flush=True)
    rate = 250
    cfg = rec.config()
    if cfg["rate"] != rate:
        cfg, _ = change_rate(rec, rate)
    rec.watch.configure(rate, cfg["gains"])
    base_pre, base_post = load_filters(rec, rate)
    hp_half = stages(rate, 0.5, LP_HZ)[0]
    lp_45 = stages(rate, HP_HZ, 45.0)[1]
    rec.ok(L.CMD_SET_CAR, 0, 0xFF)
    # Held at nominal: a notch that followed the mains partway through would
    # be one more change for the replay to reproduce.
    rec.ok(L.CMD_SET_NOTCH, *L.notch_args(50, 12, True, False))

    rec.begin("p3_settings", {"rate": rate, "gains": cfg["gains"], "config": jsonable(rec.config()),
                              "pre": base_pre, "post": base_post, "hp": HP_HZ, "lp": LP_HZ,
                              "notch": [50, 12, True, False]})
    rec.ok(L.CMD_STREAM_START)
    rec.pump(1.0)
    s0 = L.response_seq(rec.ok(L.CMD_RESET_CHAIN))
    rec.changes.append({"seq": s0, "label": "chain restarted", "kind": "reset",
                        "t": time.time() - rec.t0})
    print(f"  chain restarted at sample {s0}", flush=True)
    rec.pump(12.0)

    steps = (
        ("drift cut 1 -> 0.5 Hz", "pre", hp_half),
        ("drift cut 0.5 -> 1 Hz", "pre", base_pre),
        ("smooth above 100 -> 45 Hz", "post", lp_45),
        ("smooth above 45 -> 100 Hz", "post", base_post),
        ("notch off", "notch", L.notch_args(0, 12, True, False)),
        ("notch on", "notch", L.notch_args(50, 12, True, False)),
        ("common average on", "car", [1, 0xFF]),
        ("common average off", "car", [0, 0xFF]),
    )
    for label, kind, value in steps:
        if kind in ("pre", "post"):
            stage = L.STAGE_PRE if kind == "pre" else L.STAGE_POST
            r = rec.ok(L.CMD_SET_FILTER, *L.filter_args(stage, value, rate, keep_state=True))
        elif kind == "notch":
            r = rec.ok(L.CMD_SET_NOTCH, *value)
        else:
            r = rec.ok(L.CMD_SET_CAR, *value)
        seq = L.response_seq(r)
        rec.changes.append({"seq": seq, "label": label, "kind": kind,
                            "value": [list(s) for s in value] if kind in ("pre", "post")
                            else list(value), "t": time.time() - rec.t0})
        print(f"  {label}: from sample {seq}", flush=True)
        rec.pump(12.0)

    rec.ok(L.CMD_STREAM_STOP)
    rec.pump(0.3)
    rec.save()


def restore(rec: Recorder, original: dict | None) -> None:
    def quiet(op, *args, wait=5.0):
        try:
            return rec.cmd(op, *args, wait=wait)
        except (Lost, Fail):
            return None

    quiet(L.CMD_STREAM_STOP)
    quiet(L.CMD_SET_CAR, 0, 0xFF)
    quiet(L.CMD_SET_NOTCH, *L.notch_args(50, 12, True, True))
    try:
        cfg = rec.config()
        for stage in (L.STAGE_PRE, L.STAGE_POST):
            quiet(L.CMD_SET_FILTER, *L.filter_args(stage, [], cfg["rate"]))
        if original is not None:
            quiet(L.CMD_SET_ENCODING, original["encoding"])
            if cfg["rate"] != original["rate"]:
                change_rate(rec, original["rate"])
            if "imu_rate" in original:
                r, g, d = original["imu_rate"], original["imu_accel_g"], original["imu_gyro_dps"]
                quiet(L.CMD_SET_IMU, 1 if original["imu_on"] else 0, r & 0xFF, r >> 8, g,
                      d & 0xFF, d >> 8)
        print("  device put back as it was", flush=True)
    except (Lost, Fail) as exc:
        print(f"  could not put the device back: {exc}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--sit", type=float, default=180.0)
    ap.add_argument("--parts", default="sit,rates,settings")
    args = ap.parse_args()

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sys.stdout = Tee(out / "session.log")
    sys.stderr = sys.stdout
    parts = args.parts.split(",")
    print(f"=== worn session {time.strftime('%Y-%m-%d %H:%M:%S')}: {', '.join(parts)} ===",
          flush=True)

    link = L.BleLink()
    end = time.time() + 45
    while not link.connected and time.time() < end:
        try:
            print("  " + link.status.get(timeout=1), flush=True)
        except queue.Empty:
            pass
    if not link.connected:
        print("could not connect", flush=True)
        link.close()
        return 1
    time.sleep(1.0)

    rec = Recorder(link, out)
    original = None
    code = 0
    try:
        rec.ok(L.CMD_STREAM_STOP)
        original = rec.config()
        regs = {}
        for name, reg in REGISTERS:
            r = rec.cmd(L.CMD_READ_REG, reg)
            regs[name] = r[3] if r is not None and r[1] == L.STATUS_OK and len(r) >= 4 else None
        print("\ndevice as found", flush=True)
        describe(original, regs)
        with open(out / "device_as_found.json", "w", encoding="utf-8") as f:
            json.dump({"config": jsonable(original), "registers": regs}, f, indent=1)

        if any(m != L.MUX_NORMAL for m in original["mux"]):
            print("  inputs were not on the electrodes: setting them back", flush=True)
            rec.ok(L.CMD_SET_INPUT, L.MUX_NORMAL, 0)
        rec.ok(L.CMD_SET_ENCODING, L.ENC_RAW_UV)
        rec.ok(L.CMD_SET_IMU, 1, IMU_RATE & 0xFF, IMU_RATE >> 8, IMU_G,
               IMU_DPS & 0xFF, IMU_DPS >> 8)
        rec.ok(L.CMD_SET_CAR, 0, 0xFF)
        rec.ok(L.CMD_SET_NOTCH, *L.notch_args(50, 12, True, True))
        if original["rate"] != 250:
            change_rate(rec, 250)

        if "sit" in parts:
            part_sit(rec, args.sit)
        if "rates" in parts:
            part_rates(rec)
        if "settings" in parts:
            part_settings(rec)
    except (Lost, Fail, KeyboardInterrupt) as exc:
        print(f"\nSTOPPED: {exc!r}", flush=True)
        if rec.eeg:
            rec.name += "_partial"
            rec.save()
        code = 2
    finally:
        if link.connected:
            restore(rec, original)
        link.close()
        time.sleep(0.5)

    eps = rec.watch.episodes
    print("\nnoise watch: " + ("no lasting rise" if not eps else
                               "; ".join(f"{a:.0f}-{b:.0f} s at up to {r:.1f}x"
                                         for a, b, r in eps)), flush=True)
    print("=== session done ===" if code == 0 else "=== session stopped ===", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
