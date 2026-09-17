"""
What a worn session's files say.

  sit       every channel's signal, mains and noise; how deep the notch cut
            the mains; the frequency the device measured against an FFT of
            the raw counts; head motion and what it does to the EEG; noise
            that could be a low battery; how late motion arrives next to EEG.
  rates     how long each rate change took, gaps, the first second after it,
            and motion across it.
  settings  the device's own output replayed from the raw counts, and for
            each change how far, and for how long, the output differs from a
            chain that had the new setting all along.

    python worn_report.py DIR [part ...]
"""
from __future__ import annotations

import json
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
TOOLS = r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools"
sys.path.insert(0, str(HERE))
sys.path.insert(0, TOOLS)
import eeg_dsp  # noqa: E402
import pipeline_ref  # noqa: E402
import swifteeg_link as L  # noqa: E402
from worn_session import band_rms, stages  # noqa: E402

RAIL = (1 << 23) - 64
MOVING_DPS = 20.0
NOISE_BAND = (60.0, 90.0)
PARTS = ("p1_sit", "p2_rate500", "p2_rate1000", "p2_rate250", "p3_settings")

OPENING_NOISE: list = []   # the sit part's level, so later parts compare with it


# --- loading --------------------------------------------------------------

def load(path: pathlib.Path):
    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(str(z["meta"]))
        data = {k: z[k] for k in z.files if k != "meta"}
    return meta, data


def eeg_part(d, lo: int = 0, hi: int | None = None):
    lens = d["eeg_len"].astype(np.int64)
    hi = len(lens) if hi is None else hi
    if hi <= lo:
        return None
    starts = np.concatenate(([0], np.cumsum(lens)))
    a, b = int(starts[lo]), int(starts[hi])
    offs = np.concatenate([np.arange(k) for k in lens[lo:hi]])
    return {
        "counts": d["counts"][a:b].astype(np.int64),
        "uv": d["uv"][a:b].astype(np.float64),
        "seq": np.repeat(d["eeg_seq"][lo:hi], lens[lo:hi]) + offs,
        "flags": np.repeat(d["eeg_flags"][lo:hi], lens[lo:hi]),
        "f_ts": d["eeg_ts"][lo:hi].astype(np.float64),
        "f_seq": d["eeg_seq"][lo:hi].astype(np.float64),
        "f_len": lens[lo:hi],
        "f_arrival": d["eeg_arrival"][lo:hi],
    }


def imu_part(d, lo: int = 0, hi: int | None = None):
    lens = d["imu_len"].astype(np.int64)
    hi = len(lens) if hi is None else hi
    if hi <= lo or lens[lo:hi].sum() == 0:
        return None
    starts = np.concatenate(([0], np.cumsum(lens)))
    a, b = int(starts[lo]), int(starts[hi])
    c = d["imu_counts"][a:b].astype(np.float64)
    fl = lens[lo:hi]
    offs = np.concatenate([np.arange(k) for k in fl])
    period = np.repeat(d["imu_period"][lo:hi], fl)
    g = np.repeat(d["imu_accel_g"][lo:hi].astype(np.float64), fl)
    dps = np.repeat(d["imu_gyro_dps"][lo:hi].astype(np.float64), fl)
    return {
        "t": (np.repeat(d["imu_ts"][lo:hi].astype(np.float64), fl) + offs * period) / 1e6,
        "seq": np.repeat(d["imu_seq"][lo:hi], fl) + offs,
        "accel": c[:, :3] * (g / 32768.0)[:, None],
        "gyro": c[:, 3:] * (dps / 32768.0)[:, None],
        "f_ts": d["imu_ts"][lo:hi].astype(np.float64),
        "f_seq": d["imu_seq"][lo:hi].astype(np.float64),
        "f_len": fl,
        "f_period": d["imu_period"][lo:hi],
        "f_arrival": d["imu_arrival"][lo:hi],
        "f_flags": d["imu_flags"][lo:hi],
    }


# --- measurements ---------------------------------------------------------

def sample_us(e) -> float:
    """Microseconds a sample takes, by the device's own clock."""
    return float(np.polyfit(e["f_seq"], e["f_ts"], 1)[0])


def eeg_times(e) -> np.ndarray:
    offs = np.concatenate([np.arange(k) for k in e["f_len"]])
    return (np.repeat(e["f_ts"], e["f_len"]) + offs * sample_us(e)) / 1e6


def health(e, rate: float) -> dict:
    seq = e["seq"]
    uniq = np.unique(seq)
    c = e["counts"]
    same = (np.all(c[2:] == c[:-2], axis=1)
            & (np.abs(c[2:]).max(axis=1) < RAIL)) if len(c) > 2 else np.zeros(0, bool)
    span = e["f_arrival"][-1] - e["f_arrival"][0]
    return {
        "samples": len(seq),
        "missing": int(seq.max() - seq.min() + 1 - len(uniq)),
        "repeated": int(len(seq) - len(uniq)),
        "stale": int(same.sum()),
        "fs_device": 1e6 / sample_us(e),
        "delivered": (len(seq) - e["f_len"][-1]) / span if span > 0 else float("nan"),
    }


def latencies(e, m):
    """
    How late each frame arrives, past the fastest delivery in the file.

    Both are measured against the device's clock, so EEG and motion can be
    compared: a motion lane that runs behind shows up here and nowhere else.
    """
    us = sample_us(e)
    te = (e["f_ts"] + (e["f_len"] - 1) * us) / 1e6
    de = e["f_arrival"] - te
    if m is not None:
        tm = (m["f_ts"] + (m["f_len"] - 1) * m["f_period"]) / 1e6
        dm = m["f_arrival"] - tm
    else:
        tm = dm = np.zeros(0)

    t_all, d_all = np.concatenate((te, tm)), np.concatenate((de, dm))
    bins = np.floor((t_all - t_all.min()) / 5.0)
    xs, ys = [], []
    for b in np.unique(bins):
        k = np.flatnonzero(bins == b)
        j = k[np.argmin(d_all[k])]
        xs.append(t_all[j])
        ys.append(d_all[j])
    floor = np.polyfit(xs, ys, 1) if len(xs) >= 3 else np.array([0.0, float(min(ys))])
    late_e = (de - np.polyval(floor, te)) * 1000.0
    late_m = (dm - np.polyval(floor, tm)) * 1000.0 if len(tm) else None
    return late_e, late_m


def spread(x) -> str:
    return f"{np.median(x):.0f} / {np.percentile(x, 95):.0f} / {x.max():.0f} ms"


def fft_mains(x, fs, nominal=50.0) -> float:
    """The mains frequency in a stretch of signal, by interpolated FFT peak."""
    n = len(x)
    t = np.arange(n) - (n - 1) / 2.0
    x = x - x.mean()
    x = x - t * ((t @ x) / (t @ t))
    sp = np.abs(np.fft.rfft(x * np.hanning(n)))
    fr = np.fft.rfftfreq(n, 1.0 / fs)
    band = np.flatnonzero((fr >= nominal - 3) & (fr <= nominal + 3))
    k = int(band[np.argmax(sp[band])])
    a, b, c = (np.log(max(v, 1e-30)) for v in (sp[k - 1], sp[k], sp[k + 1]))
    return float(fr[k] + 0.5 * (a - c) / (a - 2 * b + c) * (fr[1] - fr[0]))


def lsb_of(gains) -> np.ndarray:
    return np.array([L.lsb_uv(g or 24) for g in gains])


def noise_series(e, fs, lsb) -> np.ndarray:
    n = int(round(fs))
    out = []
    for i in range(len(e["counts"]) // n):
        c = e["counts"][i * n:(i + 1) * n]
        ok = np.abs(c).max(axis=0) < RAIL
        out.append(float(np.median(band_rms(c * lsb, fs, *NOISE_BAND)[ok]))
                   if ok.any() else np.nan)
    return np.array(out)


def noise_report(e, fs, lsb, label="noise") -> None:
    lv = noise_series(e, fs, lsb)
    if not len(lv):
        return
    base = OPENING_NOISE[0] if OPENING_NOISE else float(np.nanmedian(lv[5:25]))
    if not OPENING_NOISE:
        OPENING_NOISE.append(base)
    high = lv >= 4.0 * base
    runs, start = [], None
    for i, flag in enumerate(high):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if i - start >= 10:
                runs.append((start, i, float(np.nanmax(lv[start:i]) / base)))
            start = None
    if start is not None and len(high) - start >= 10:
        runs.append((start, len(high), float(np.nanmax(lv[start:]) / base)))
    print(f"  {label} 60-90 Hz: median {np.nanmedian(lv):.2f} uV, worst second "
          f"{np.nanmax(lv):.2f} uV, against an opening level of {base:.2f} uV")
    if runs:
        for a, b, r in runs:
            print(f"    !! {a}-{b} s at up to {r:.1f}x the opening level - check the battery")
    else:
        print("    no lasting rise: nothing that looks like a flat battery")


# --- the device's chain, in float64, for replaying a recording -------------

def notch_sections(fs, hz, q, harmonic):
    if not hz:
        return []
    out = [eeg_dsp.design_notch(fs, hz, q)]
    if harmonic and 2.0 * hz < 0.95 * fs / 2.0:
        out.append(eeg_dsp.design_notch(fs, 2.0 * hz, q))
    return out


def corner_of(sections, fs) -> float:
    """The corner a stage's sections were designed for: g = tan(pi fc / fs)."""
    return float(np.arctan(sections[0][0]) * fs / np.pi) if sections else 0.0


class FastChain:
    """
    chain.c in float64: integer DC removal, the pre sections, the notch, the
    common average, the post sections. Whole blocks at a time, so a recording
    replays in seconds rather than minutes.
    """

    def __init__(self, fs, gains, pre, post, notch, car):
        self.fs = float(fs)
        self.shift = pipeline_ref.dc_shift_for(fs)
        self.lsb = lsb_of(gains)
        self.acc = None
        self.pre = eeg_dsp.Sections(L.CHANNELS)
        self.pre.set(pre)
        self.notch = eeg_dsp.Sections(L.CHANNELS)
        self.notch.set(notch_sections(fs, *notch))
        self.post = eeg_dsp.Sections(L.CHANNELS)
        self.post.set(post)
        self.car, self.mask = car

    def _dc(self, counts):
        out = np.empty(counts.shape, dtype=np.int64)
        acc, sh = self.acc, self.shift
        for i in range(len(counts)):
            s = counts[i]
            acc = (s << sh) if acc is None else acc + s - (acc >> sh)
            out[i] = s - (acc >> sh)
        self.acc = acc
        return out

    def run(self, counts, events=None):
        events = events or {}
        cuts = sorted({0, len(counts)} | {i for i in events if 0 <= i < len(counts)})
        out = []
        for a, b in zip(cuts[:-1], cuts[1:]):
            for fn in events.get(a, []):
                fn(self)
            x = self._dc(counts[a:b]).astype(np.float64) * self.lsb
            x = self.notch.apply(self.pre.apply(x))
            if self.car:
                m = np.array([(self.mask >> i) & 1 for i in range(L.CHANNELS)], bool)
                if m.sum() >= 2:
                    x = x - x[:, m].mean(axis=1, keepdims=True)
            out.append(self.post.apply(x))
        return np.vstack(out) if out else np.zeros((0, L.CHANNELS))


def chain_for(fs, gains, state) -> FastChain:
    pre, post = stages(fs, state["hp"], state["lp"])
    return FastChain(fs, gains, pre, post, (state["notch"], 12.0, True),
                     (state["car"], 0xFF))


# --- the parts ------------------------------------------------------------

def report_sit(meta, d) -> None:
    fs = float(meta["rate"])
    gains = meta["gains"]
    lsb = lsb_of(gains)
    e, m = eeg_part(d), imu_part(d)
    h = health(e, fs)
    print(f"\n=== sitting, {fs:.0f} SPS, {h['samples'] / fs:.0f} s ===")
    print(f"  stream: {h['samples']} samples, {h['missing']} missing, {h['repeated']} "
          f"repeated, {h['stale']} stale; {h['fs_device']:.2f} SPS by the device clock, "
          f"{h['delivered']:.1f} delivered")

    raw = e["counts"] * lsb
    fm = fft_mains(raw[-int(60 * fs):].mean(axis=1), fs)
    win = int(4 * fs)
    rows = []
    for i in range(len(raw) // win):
        x, y = raw[i * win:(i + 1) * win], e["uv"][i * win:(i + 1) * win]
        rows.append((band_rms(x, fs, 1.0, 40.0), band_rms(x, fs, *NOISE_BAND),
                     np.sqrt(2) * band_rms(x, fs, fm - 0.75, fm + 0.75),
                     band_rms(y, fs, 1.0, 40.0),
                     np.sqrt(2) * band_rms(y, fs, fm - 0.75, fm + 0.75)))
    eeg, hf, mains, dev_eeg, dev_mains = (np.median(np.array([r[k] for r in rows]), axis=0)
                                          for k in range(5))
    offset = np.median(raw, axis=0) / 1000.0
    railed = np.mean(np.abs(e["counts"]) >= RAIL, axis=0)

    print("\n  channel   offset     EEG      mains    mains after   noise    what it looks like")
    print("             (mV)    1-40 uV    raw uV     notch uV    60-90 uV")
    for ch in range(L.CHANNELS):
        notes = []
        if railed[ch] > 0.01:
            notes.append("hitting the limit")
        if eeg[ch] < 0.5:
            notes.append("flat")
        if mains[ch] > max(4 * np.median(mains), 20.0):
            notes.append("mains high - loose?")
        if eeg[ch] > 3 * np.median(eeg):
            notes.append("noisier than the rest")
        print(f"   CH{ch + 1}    {offset[ch]:7.1f} {eeg[ch]:9.1f} {mains[ch]:10.1f} "
              f"{dev_mains[ch]:11.2f} {hf[ch]:10.2f}    {', '.join(notes) or 'ok'}")
    cut = 20 * np.log10(np.maximum(dev_mains, 1e-6) / np.maximum(mains, 1e-6))
    print(f"  the notch cut the mains by {np.median(cut):.0f} dB (worst channel "
          f"{cut.max():.0f} dB)")

    print("\n  mains")
    ev_t, ev_hz, ev_moved = d["evt_arrival"], d["evt_hz"], d["evt_moved"]
    if len(ev_hz):
        errs = []
        te = eeg_times(e)
        for t, hz in zip(ev_t, ev_hz):
            # the twenty seconds of raw signal the device had just measured
            i = int(np.searchsorted(e["f_arrival"], t))
            j = int(np.sum(e["f_len"][:max(i, 1)]))
            a = max(0, j - int(20 * fs))
            if j - a > int(8 * fs):
                errs.append(hz - fft_mains(raw[a:j].mean(axis=1), fs))
        print(f"    {len(ev_hz)} estimates, {int(ev_moved.sum())} of them moved the notch; "
              f"{ev_hz.min():.3f} to {ev_hz.max():.3f} Hz, last {ev_hz[-1]:.4f} Hz")
        if errs:
            errs = np.array(errs) * 1000.0
            print(f"    against an FFT of the raw signal: {np.median(errs):+.1f} mHz "
                  f"median, worst {errs[np.argmax(np.abs(errs))]:+.1f} mHz")
        print(f"    FFT of the last 60 s: {fm:.4f} Hz; the device aims at "
              f"{meta.get('config_after', {}).get('notch_aim_hz')}")
    else:
        print("    no estimate arrived")

    if m is not None:
        print("\n  head motion")
        gyro = np.linalg.norm(m["gyro"], axis=1)
        imu_fs = 1.0 / np.median(np.diff(m["t"]))
        moving = gyro > MOVING_DPS
        w = max(1, int(round(0.25 * imu_fs)))
        moving = np.convolve(moving.astype(int), np.ones(2 * w + 1, int), "same") > 0
        edges = np.flatnonzero(np.diff(np.concatenate(([0], moving.astype(int), [0]))))
        runs = list(zip(edges[::2], edges[1::2]))
        gaps = int(np.sum(m["f_seq"][1:] != m["f_seq"][:-1] + m["f_len"][:-1]))
        print(f"    {len(m['t'])} samples at {imu_fs:.1f} Hz, {gaps} gaps, flags "
              f"0x{int(np.bitwise_or.reduce(m['f_flags'])):02x}; peak turn "
              f"{gyro.max():.0f} deg/s")
        print(f"    {len(runs)} movements, {100 * moving.mean():.0f} % of the time moving")

        te = eeg_times(e)
        n = int(fs)
        still, shift = [], []
        for i in range(len(raw) // n):
            a, b = te[i * n], te[(i + 1) * n - 1]
            j0, j1 = np.searchsorted(m["t"], (a, b))
            level = float(np.median(band_rms(raw[i * n:(i + 1) * n], fs, 1.0, 40.0)))
            (shift if (j1 > j0 and moving[j0:j1].any()) else still).append(level)
        if still and shift:
            print(f"    EEG 1-40 Hz: {np.median(still):.1f} uV sitting still, "
                  f"{np.median(shift):.1f} uV while moving, worst second "
                  f"{max(shift):.0f} uV")

        late_e, late_m = latencies(e, m)
        print(f"    arrival, past the fastest: EEG {spread(late_e)}, motion {spread(late_m)}")

    print()
    noise_report(e, fs, lsb)


def report_rate(name, meta, d) -> None:
    fs = float(meta["rate"])
    gains = meta["gains"]
    lsb = lsb_of(gains)
    split = meta["split"]
    t = meta["times"]
    new = eeg_part(d, split["eeg"])
    if new is None:
        print(f"\n=== {meta['rate_before']} -> {meta['rate']} SPS: no samples after the change ===")
        return
    h = health(new, fs)
    print(f"\n=== {meta['rate_before']} -> {meta['rate']} SPS ===")
    print(f"  the device answered in {1000 * (t['ack'] - t['set']):.0f} ms and its "
          f"configuration {1000 * (t['config'] - t['set']):.0f} ms after that; streaming "
          f"again {1000 * (t['start'] - t['stop']):.0f} ms after the stop")
    print(f"  first new sample {1000 * (new['f_arrival'][0] - t['stop']):.0f} ms after the "
          f"stop, {1000 * (new['f_arrival'][0] - t['start']):.0f} ms after the start")
    print(f"  stream: {h['samples']} samples, {h['missing']} missing, {h['repeated']} "
          f"repeated, {h['stale']} stale; {h['fs_device']:.2f} SPS by the device clock, "
          f"{h['delivered']:.1f} delivered")

    flagged = int(np.argmin(new["flags"] & 1)) if (new["flags"] & 1).any() else 0
    head = np.abs(new["uv"][:int(fs)]).max(axis=0)
    tail = np.percentile(np.abs(new["uv"][-int(20 * fs):]), 99.5, axis=0)
    worst = int(np.argmax(head / np.maximum(tail, 1e-9)))
    print(f"  {flagged} samples flagged as settling ({flagged / fs:.2f} s); first second "
          f"peaks at {head[worst]:.0f} uV against {tail[worst]:.0f} uV later on CH{worst + 1} "
          f"({head[worst] / max(tail[worst], 1e-9):.1f}x)")

    old_m, new_m = imu_part(d, 0, split["imu"]), imu_part(d, split["imu"])
    if new_m is not None:
        gaps = int(np.sum(new_m["f_seq"][1:] != new_m["f_seq"][:-1] + new_m["f_len"][:-1]))
        late_e, late_m = latencies(new, new_m)
        print(f"  motion: {len(new_m['t'])} samples, {gaps} gaps, flags "
              f"0x{int(np.bitwise_or.reduce(new_m['f_flags'])):02x}; arrival past the "
              f"fastest: EEG {spread(late_e)}, motion {spread(late_m)}")
        if old_m is not None:
            step = new_m["seq"][0] - old_m["seq"][-1]
            gap_s = new_m["t"][0] - old_m["t"][-1]
            print(f"    across the change: {step} motion samples and {gap_s:.2f} s of "
                  f"device time - a silence, not a loss, if the two agree")
        end = eeg_times(new)[-1] - new_m["t"][-1]
        print(f"    EEG and motion end {1000 * end:+.0f} ms apart on the device clock")

    if len(d["evt_hz"]):
        after = d["evt_arrival"] > t["start"]
        if after.any():
            first = np.flatnonzero(after)[0]
            print(f"  mains: first estimate {d['evt_arrival'][first] - t['start']:.1f} s "
                  f"after the restart, {d['evt_hz'][first]:.4f} Hz; last "
                  f"{d['evt_hz'][-1]:.4f} Hz")
    noise_report(new, fs, lsb)


def report_settings(meta, d) -> None:
    fs = float(meta["rate"])
    gains = meta["gains"]
    changes = meta["changes"]
    e = eeg_part(d)
    h = health(e, fs)
    print(f"\n=== settings while streaming, {fs:.0f} SPS ===")
    print(f"  stream: {h['samples']} samples, {h['missing']} missing, {h['stale']} stale")

    seq = e["seq"]
    s0 = changes[0]["seq"]
    i0 = int(np.searchsorted(seq, s0))
    if i0 >= len(seq) or seq[i0] != s0:
        print("  the restart sample never arrived - nothing to replay")
        return
    breaks = np.flatnonzero(np.diff(seq[i0:]) != 1)
    end = i0 + (int(breaks[0]) + 1 if len(breaks) else len(seq) - i0)
    counts, dev = e["counts"][i0:end], e["uv"][i0:end]
    where = seq[i0:end]
    aim = meta["config"].get("notch_aim_hz") or 50.0

    state = {"hp": meta["hp"], "lp": meta["lp"], "notch": aim, "car": False}
    steps = []
    for c in changes[1:]:
        i = int(np.searchsorted(where, c["seq"]))
        if i >= len(where) or where[i] != c["seq"]:
            print(f"  {c['label']}: sample {c['seq']} never arrived")
            continue
        after = dict(state)
        if c["kind"] in ("pre", "post"):
            secs = [tuple(v) for v in c["value"]]
            after["hp" if c["kind"] == "pre" else "lp"] = corner_of(secs, fs)
            fn = (lambda s: lambda ch: ch.pre.set(s))(secs) if c["kind"] == "pre" \
                else (lambda s: lambda ch: ch.post.set(s))(secs)
        elif c["kind"] == "notch":
            hz, q, flags = c["value"]
            after["notch"] = aim if hz else 0.0
            fn = (lambda f, q: lambda ch: ch.notch.set(
                notch_sections(fs, f, q, True)))(after["notch"], float(q))
        else:
            on, mask = c["value"]
            after["car"] = bool(on)
            fn = (lambda o, k: lambda ch: (setattr(ch, "car", bool(o)),
                                           setattr(ch, "mask", k)))(on, mask)
        steps.append({"i": i, "label": c["label"], "after": after, "fn": fn})
        state = after

    live = chain_for(fs, gains, {"hp": meta["hp"], "lp": meta["lp"], "notch": aim,
                                 "car": False})
    out = live.run(counts, {s["i"]: [s["fn"]] for s in steps})
    err = np.abs(out - dev).max()
    print(f"  the device's output replayed from the raw counts: worst disagreement "
          f"{err:.2e} uV over {len(dev)} samples "
          f"({'matches' if err < 0.05 else 'DOES NOT MATCH'})")

    print("\n  each change, against a chain that had the new setting all along:")
    cache = {}
    for k, s in enumerate(steps):
        stop = steps[k + 1]["i"] if k + 1 < len(steps) else len(dev)
        key = tuple(sorted(s["after"].items()))
        if key not in cache:
            cache[key] = chain_for(fs, gains, s["after"]).run(counts)
        ref = cache[key]
        diff = np.abs(dev[s["i"]:stop] - ref[s["i"]:stop]).max(axis=1)
        quick = diff[:int(0.25 * fs)].max() if len(diff) > int(0.25 * fs) else diff.max()
        over = np.flatnonzero(diff >= 1.0)
        settle = (over[-1] + 1) / fs if len(over) else 0.0
        flagged = int((e["flags"][i0 + s["i"]:i0 + stop] & 1).sum())
        print(f"    {s['label']:<28} sample {where[s['i']]:>8}: worst {diff.max():7.1f} uV, "
              f"first 0.25 s {quick:7.1f} uV, under 1 uV after {settle:5.2f} s"
              + (f", {flagged} samples flagged settling" if flagged else ""))

    noise_report(e, fs, lsb_of(gains))


def main() -> int:
    folder = pathlib.Path(sys.argv[1])
    wanted = sys.argv[2:] or list(PARTS)
    for name in wanted:
        path = folder / f"{name}.npz"
        if not path.exists():
            print(f"\n{name}: not recorded")
            continue
        meta, d = load(path)
        if name.startswith("p1"):
            report_sit(meta, d)
        elif name.startswith("p2"):
            report_rate(name, meta, d)
        else:
            report_settings(meta, d)
    return 0


if __name__ == "__main__":
    sys.exit(main())
