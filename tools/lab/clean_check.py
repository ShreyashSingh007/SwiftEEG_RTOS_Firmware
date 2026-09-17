"""
Is the worn recording still usable, given yawns, shoulder movement and other
body noise the motion sensor cannot see?

Marks every second as clean or not - no head motion, and no outlier in the
EEG band, the muscle band, or peak to peak - then redoes each number that
depends on the signal using clean seconds only, printed next to what was
reported before. Numbers that do not depend on the signal (samples lost,
timings, the replay match) are not recomputed: nothing about the wearer can
change them.
"""
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import worn_report as R  # noqa: E402

BASE = pathlib.Path("D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/"
                    "RTOS/recordings/worn_2026-09-16")
PTP_UV = 200.0      # a second swinging this far is not resting EEG
OUTLIER = 3.0       # times the channel's own median level


def seconds(e, m, fs, lsb):
    """Which seconds are clean, and why they are not."""
    n = int(round(fs))
    k = len(e["counts"]) // n
    te = R.eeg_times(e)
    raw = e["counts"] * lsb
    eeg = np.array([R.band_rms(raw[i * n:(i + 1) * n], fs, 1.0, 40.0) for i in range(k)])
    emg = np.array([R.band_rms(raw[i * n:(i + 1) * n], fs, 60.0, 90.0) for i in range(k)])
    ptp = np.array([np.ptp(e["uv"][i * n:(i + 1) * n], axis=0) for i in range(k)])
    bad = ((eeg > OUTLIER * np.median(eeg, axis=0)).any(axis=1)
           | (emg > OUTLIER * np.median(emg, axis=0)).any(axis=1)
           | (ptp > PTP_UV).any(axis=1))
    moving = np.zeros(k, bool)
    if m is not None:
        gy = np.linalg.norm(m["gyro"], axis=1)
        for i in range(k):
            j0, j1 = np.searchsorted(m["t"], (te[i * n], te[(i + 1) * n - 1]))
            moving[i] = bool((gy[j0:j1] > R.MOVING_DPS).any()) if j1 > j0 else False
    return ~(bad | moving), moving, bad, raw


def sit():
    meta, d = R.load(BASE / "p1_sit.npz")
    fs = float(meta["rate"])
    lsb = R.lsb_of(meta["gains"])
    e, m = R.eeg_part(d), R.imu_part(d)
    clean, moving, bad, raw = seconds(e, m, fs, lsb)
    n = int(fs)
    print(f"=== sitting, {len(clean)} seconds ===")
    print(f"  {100 * clean.mean():.0f} % clean; head motion {100 * moving.mean():.0f} %, "
          f"body or muscle without head motion {100 * (bad & ~moving).mean():.0f} %")

    fm = R.fft_mains(raw[-int(60 * fs):].mean(axis=1), fs)
    win = 4
    blocks = [i for i in range(len(clean) // win) if clean[i * win:(i + 1) * win].all()]
    every = list(range(len(clean) // win))

    def table(bl):
        rows = []
        for i in bl:
            a, b = i * win * n, (i + 1) * win * n
            x, y = raw[a:b], e["uv"][a:b]
            rows.append((R.band_rms(x, fs, 1.0, 40.0), R.band_rms(x, fs, 60.0, 90.0),
                         np.sqrt(2) * R.band_rms(x, fs, fm - 0.75, fm + 0.75),
                         np.sqrt(2) * R.band_rms(y, fs, fm - 0.75, fm + 0.75)))
        return [np.median(np.array([r[k] for r in rows]), axis=0) for k in range(4)]

    c_eeg, c_hf, c_mains, c_dev = table(blocks)
    a_eeg, a_hf, a_mains, a_dev = table(every)
    print(f"  4 s blocks: {len(blocks)} clean of {len(every)}")
    print("  channel     EEG 1-40 uV       noise 60-90 uV      mains raw uV")
    print("               all    clean       all    clean       all    clean")
    for ch in range(8):
        print(f"   CH{ch + 1}      {a_eeg[ch]:6.1f} {c_eeg[ch]:8.1f}   {a_hf[ch]:7.2f} "
              f"{c_hf[ch]:7.2f}   {a_mains[ch]:7.1f} {c_mains[ch]:7.1f}")
    cut_a = 20 * np.log10(a_dev / a_mains)
    cut_c = 20 * np.log10(c_dev / c_mains)
    print(f"  notch cut: all {np.median(cut_a):.0f} dB, clean {np.median(cut_c):.0f} dB")
    print(f"  noise level for the battery watch: all {np.median(a_hf):.2f} uV, clean "
          f"{np.median(c_hf):.2f} uV (the recorder used 1.73 uV)")

    errs_all, errs_clean = [], []
    for t, hz in zip(d["evt_arrival"], d["evt_hz"]):
        i = int(np.searchsorted(e["f_arrival"], t))
        j = int(np.sum(e["f_len"][:max(i, 1)]))
        a = max(0, j - int(20 * fs))
        if j - a <= int(8 * fs):
            continue
        err = 1000 * (hz - R.fft_mains(raw[a:j].mean(axis=1), fs))
        errs_all.append(err)
        if clean[a // n:j // n].mean() >= 0.9:
            errs_clean.append(err)
    for label, v in (("every estimate", errs_all), ("clean stretches only", errs_clean)):
        v = np.array(v)
        if len(v):
            print(f"  mains against an FFT, {label}: {len(v)} estimates, median "
                  f"{np.median(v):+.1f} mHz, worst {v[np.argmax(np.abs(v))]:+.1f} mHz")


def rates():
    for name in ("p2_rate500", "p2_rate1000", "p2_rate250"):
        meta, d = R.load(BASE / f"{name}.npz")
        fs = float(meta["rate"])
        lsb = R.lsb_of(meta["gains"])
        e = R.eeg_part(d, meta["split"]["eeg"])
        m = R.imu_part(d, meta["split"]["imu"])
        clean, moving, bad, raw = seconds(e, m, fs, lsb)
        n = int(fs)
        tail = e["uv"][-20 * n:]
        keep = np.repeat(clean[-20:], n)[-len(tail):]
        ref = np.percentile(np.abs(tail[keep] if keep.any() else tail), 99.5, axis=0)
        first = np.abs(e["uv"][:n]).max(axis=0)
        w = int(np.argmax(first / ref))
        print(f"\n=== {meta['rate_before']} -> {meta['rate']} SPS ===")
        print(f"  {100 * clean.mean():.0f} % of seconds clean; the first second after "
              f"the restart was {'clean' if clean[0] else 'NOT clean'}; the tail used as "
              f"reference {100 * clean[-20:].mean():.0f} % clean")
        print(f"  first second {first[w]:.0f} uV against a clean-tail reference "
              f"{ref[w]:.0f} uV on CH{w + 1} ({first[w] / ref[w]:.1f}x)")


def settings():
    meta, d = R.load(BASE / "p3_settings.npz")
    fs = float(meta["rate"])
    gains = meta["gains"]
    lsb = R.lsb_of(gains)
    e = R.eeg_part(d)
    clean, moving, bad, raw = seconds(e, None, fs, lsb)
    n = int(fs)
    seq = e["seq"]
    changes = meta["changes"]
    i0 = int(np.searchsorted(seq, changes[0]["seq"]))
    brk = np.flatnonzero(np.diff(seq[i0:]) != 1)
    end = i0 + (int(brk[0]) + 1 if len(brk) else len(seq) - i0)
    counts, dev, where = e["counts"][i0:end], e["uv"][i0:end], seq[i0:end]
    aim = meta["config"].get("notch_aim_hz") or 50.0
    state = {"hp": meta["hp"], "lp": meta["lp"], "notch": aim, "car": False}
    print(f"\n=== settings ===\n  {100 * clean.mean():.0f} % of seconds clean")

    steps = []
    for c in changes[1:]:
        after = dict(state)
        if c["kind"] in ("pre", "post"):
            secs = [tuple(v) for v in c["value"]]
            after["hp" if c["kind"] == "pre" else "lp"] = R.corner_of(secs, fs)
        elif c["kind"] == "notch":
            after["notch"] = aim if c["value"][0] else 0.0
        else:
            after["car"] = bool(c["value"][0])
        steps.append({"i": int(np.searchsorted(where, c["seq"])), "label": c["label"],
                      "after": after})
        state = after

    cache = {}
    for k, s in enumerate(steps):
        stop = steps[k + 1]["i"] if k + 1 < len(steps) else len(dev)
        key = tuple(sorted(s["after"].items()))
        if key not in cache:
            cache[key] = R.chain_for(fs, gains, s["after"]).run(counts)
        diff = np.abs(dev[s["i"]:stop] - cache[key][s["i"]:stop]).max(axis=1)
        over = np.flatnonzero(diff >= 1.0)
        settle = (over[-1] + 1) / fs if len(over) else 0.0
        sec = np.clip((i0 + s["i"] + np.arange(len(diff))) // n, 0, len(clean) - 1)
        over_c = np.flatnonzero((diff >= 1.0) & clean[sec])
        settle_c = (over_c[-1] + 1) / fs if len(over_c) else 0.0
        first3 = clean[np.clip(np.arange(sec[0], sec[0] + 3), 0, len(clean) - 1)]
        print(f"  {s['label']:<28} worst {diff.max():7.1f} uV; under 1 uV after "
              f"{settle:5.2f} s (clean seconds only {settle_c:5.2f} s); first 3 s "
              f"{'clean' if first3.all() else 'had body noise'}")


sit()
rates()
settings()
