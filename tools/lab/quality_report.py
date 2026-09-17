"""
How good is the signal, and where does it fall short?

Everything is measured on clean seconds only - no head motion, no outlier
second - and separately for eyes open and eyes closed, because the one
measurement that distinguishes EEG from a well-behaved noise source is alpha
rising when the eyes close.

    python quality_report.py DIR/quality.npz
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import worn_report as R  # noqa: E402

RAIL = (1 << 23) - 64
MOVING_DPS = 20.0
BANDS = (("delta 1-4", 1.0, 4.0), ("theta 4-8", 4.0, 8.0), ("alpha 8-13", 8.0, 13.0),
         ("beta 13-30", 13.0, 30.0), ("EMG 30-45", 30.0, 45.0))


def welch(x, fs, seconds=4.0):
    """Amplitude spectrum, averaged over Hann windows. Returns (freqs, uV/rtHz)."""
    n = int(seconds * fs)
    if len(x) < n:
        return None, None
    w = np.hanning(n)
    scale = 2.0 / (fs * np.sum(w ** 2))          # one-sided power density
    acc = None
    count = 0
    for i in range(0, len(x) - n + 1, n // 2):
        seg = x[i:i + n]
        seg = seg - seg.mean(axis=0)
        P = np.abs(np.fft.rfft(seg * w[:, None], axis=0)) ** 2 * scale
        acc = P if acc is None else acc + P
        count += 1
    return np.fft.rfftfreq(n, 1.0 / fs), np.sqrt(acc / count)   # uV/rtHz


def band(freqs, dens, lo, hi):
    """RMS in a band from a density spectrum, per channel."""
    sel = (freqs >= lo) & (freqs <= hi)
    df = freqs[1] - freqs[0]
    return np.sqrt(np.sum(dens[sel] ** 2, axis=0) * df)


def clean_seconds(e, m, fs, lsb):
    n = int(round(fs))
    k = len(e["counts"]) // n
    te = R.eeg_times(e)
    raw = e["counts"] * lsb
    lvl = np.array([R.band_rms(raw[i * n:(i + 1) * n], fs, 1.0, 40.0) for i in range(k)])
    hf = np.array([R.band_rms(raw[i * n:(i + 1) * n], fs, 60.0, 90.0) for i in range(k)])
    ptp = np.array([np.ptp(e["uv"][i * n:(i + 1) * n], axis=0) for i in range(k)])
    bad = ((lvl > 3 * np.median(lvl, axis=0)).any(axis=1)
           | (hf > 3 * np.median(hf, axis=0)).any(axis=1)
           | (ptp > 200.0).any(axis=1))
    if m is not None:
        gy = np.linalg.norm(m["gyro"], axis=1)
        for i in range(k):
            j0, j1 = np.searchsorted(m["t"], (te[i * n], te[(i + 1) * n - 1]))
            if j1 > j0 and (gy[j0:j1] > MOVING_DPS).any():
                bad[i] = True
    return ~bad, raw, te


def main() -> int:
    path = pathlib.Path(sys.argv[1])
    meta, d = R.load(path)
    fs = float(meta["rate"])
    lsb = R.lsb_of(meta["gains"])
    e, m = R.eeg_part(d), R.imu_part(d)
    clean, raw, te = clean_seconds(e, m, fs, lsb)
    n = int(round(fs))

    # Phase windows, by arrival time, in whole seconds of the sample stream.
    starts = np.repeat(d["eeg_arrival"], d["eeg_len"].astype(np.int64))
    phase_idx = {}
    for ph in meta["phases"]:
        sel = (starts >= ph["start"] + 2.0) & (starts <= ph["end"])   # skip the cue
        secs = np.unique(np.flatnonzero(sel) // n)
        secs = np.array([s for s in secs if s < len(clean) and clean[s]])
        phase_idx.setdefault(ph["phase"], []).append(secs)

    def gather(name, source):
        out = []
        for secs in phase_idx.get(name, []):
            for s in secs:
                out.append(source[s * n:(s + 1) * n])
        return np.vstack(out) if out else None

    open_raw, closed_raw = gather("eyes open", raw), gather("eyes closed", raw)
    open_dev = gather("eyes open", e["uv"])
    if open_raw is None or closed_raw is None:
        print("not enough clean data in one of the phases")
        return 1

    print(f"=== {len(clean)} s recorded, {100 * clean.mean():.0f} % clean "
          f"(eyes open {len(open_raw) / fs:.0f} s, eyes closed {len(closed_raw) / fs:.0f} s) ===")
    h = R.health(e, fs)
    print(f"stream: {h['samples']} samples, {h['missing']} missing, {h['stale']} stale, "
          f"{h['fs_device']:.2f} SPS")

    fo, do_ = welch(open_raw, fs)
    fc, dc_ = welch(closed_raw, fs)
    fd, dd_ = welch(open_dev, fs)

    offset = np.median(raw, axis=0) / 1000.0
    railed = np.mean(np.abs(e["counts"]) >= RAIL, axis=0)
    # Noise floor: 70-90 Hz is above the EEG band and below the notch harmonic,
    # so what is left there is amplifier, electrode and residual muscle noise.
    floor = np.median(dd_[(fd >= 70) & (fd <= 90)], axis=0)        # uV/rtHz
    wide = floor * np.sqrt(99.5)                                   # 0.5-100 Hz, if flat
    fm = meta.get("config_after", {}).get("notch_aim_hz") or 50.0
    mains_raw = np.sqrt(2) * band(fo, do_, fm - 0.7, fm + 0.7)
    mains_dev = np.sqrt(2) * band(fd, dd_, fm - 0.7, fm + 0.7)

    a_open = band(fo, do_, 8.0, 13.0)
    a_closed = band(fc, dc_, 8.0, 13.0)

    print("\nch   offset  noise      band-limited   50 Hz raw -> out     alpha uV      ")
    print("      (mV)  uV/rtHz   0.5-100 uVrms       (uV)   (dB)    open  closed  x")
    for c in range(8):
        print(f"CH{c+1}  {offset[c]:6.1f} {floor[c]:8.3f} {wide[c]:13.2f}  "
              f"{mains_raw[c]:8.1f} {20*np.log10(max(mains_dev[c],1e-6)/max(mains_raw[c],1e-6)):6.0f}"
              f"  {a_open[c]:7.1f} {a_closed[c]:7.1f} {a_closed[c]/max(a_open[c],1e-9):5.1f}"
              + ("   CLIPPING" if railed[c] > 0.01 else ""))

    print("\nbands, eyes open / eyes closed (uV rms, clean seconds)")
    for name, lo, hi in BANDS:
        bo, bc = band(fo, do_, lo, hi), band(fc, dc_, lo, hi)
        print(f"  {name:<11} {np.median(bo):7.2f} {np.median(bc):8.2f}   "
              f"per-channel closed/open {np.median(bc/np.maximum(bo,1e-9)):.2f}x")

    # Alpha peak prominence over the 1/f background, eyes closed: the clearest
    # single number for "is this brain signal".
    sel = (fc >= 7) & (fc <= 14)
    side = ((fc >= 4) & (fc < 7)) | ((fc > 14) & (fc <= 20))
    peak = dc_[sel].max(axis=0)
    bg = np.median(dc_[side], axis=0)
    prom = 20 * np.log10(peak / np.maximum(bg, 1e-12))
    best = int(np.argmax(prom))
    pf = fc[sel][np.argmax(dc_[sel][:, best])]
    print(f"\nalpha peak, eyes closed: best CH{best+1} at {pf:.1f} Hz, "
          f"{prom[best]:.1f} dB over its own background; median over channels "
          f"{np.median(prom):.1f} dB")

    corr = np.corrcoef(open_dev.T)
    off = corr[~np.eye(8, dtype=bool)]
    print(f"channel correlation, eyes open: median {np.median(off):+.2f}, "
          f"max {off.max():+.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
