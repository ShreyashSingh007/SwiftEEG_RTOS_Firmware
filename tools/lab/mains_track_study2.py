"""
Mains tracking for the MCU, second design: a frequency-locked loop.

Per sample: mix the channel mean down by the current estimate with a
rotating phasor, low-pass both halves (two one-pole stages, 1.5 Hz). Every
1/10 s keep one baseband value. Over a 4 s window, the frequency error is the
phase the baseband tone turns over a lag - 0.2 s while acquiring (+/-2.5 Hz
unambiguous), 1 s once close (fine). The lag is long against the low-pass's
memory, so noise decorrelates and only a real tone stays coherent: that
coherence is the lock test. On an update the mixer moves to the new estimate.

All float32 on the per-sample path, as the firmware would run it. Compared
with the app's current method, a 4000-sample Hann FFT with parabolic peak
interpolation every 5 s.
"""
import collections
import math

import numpy as np

F32 = np.float32
LOCK = 0.6


def make(fs, seconds, mains_uv, f_fn, seed=2):
    rng = np.random.default_rng(seed)
    n = int(seconds * fs)
    t = np.arange(n) / fs
    phase = 2 * np.pi * np.cumsum(f_fn(t)) / fs
    white = rng.standard_normal(n)
    pink = np.cumsum(white)
    pink -= np.convolve(pink, np.ones(int(fs)) / fs, mode="same")
    x = (8 * pink / pink.std()                        # broadband EEG
         + 0.3 * math.sqrt(fs / 2) * rng.standard_normal(n)  # 0.3 uV/rtHz floor
         + 10 * np.sin(2 * np.pi * 10 * t)               # alpha
         + 300 * np.sin(2 * np.pi * 0.1 * t)             # drift
         + mains_uv * np.sin(phase))
    return x, f_fn(t)


class Tracker:
    def __init__(self, fs, nominal=50.0, bb=10.0, corner=1.5, fine_s=1.0,
                 coarse_s=0.2, window_s=4.0):
        self.fs, self.nominal = fs, nominal
        self.k = F32(1 - math.exp(-2 * math.pi * corner / fs))
        self.c, self.s = F32(1.0), F32(0.0)
        self.i1 = self.i2 = self.q1 = self.q2 = F32(0.0)
        self.dec = int(round(fs / bb))
        self.bb = fs / self.dec
        self.fine = int(round(fine_s * self.bb))
        self.coarse = int(round(coarse_s * self.bb))
        self.window = int(round(window_s * self.bb))
        self.ring = collections.deque(maxlen=self.fine)
        self.n = 0
        self.mix = nominal
        self.retune(nominal)
        self._clear()
        self.estimate = None

    def retune(self, f):
        self.mix = f
        w = 2 * math.pi * f / self.fs
        self.cw, self.sw = F32(math.cos(w)), F32(math.sin(w))

    def _clear(self):
        self.rf = complex(0)
        self.rc = complex(0)
        self.p = 0.0
        self.count = 0

    def push(self, x):
        x = F32(x)
        c = self.c * self.cw - self.s * self.sw
        s = self.s * self.cw + self.c * self.sw
        self.c, self.s = c, s
        self.i1 += self.k * (x * c - self.i1)
        self.i2 += self.k * (self.i1 - self.i2)
        self.q1 += self.k * (x * s - self.q1)
        self.q2 += self.k * (self.q1 - self.q2)
        self.n += 1
        if self.n % 8192 == 0:
            g = F32(1.0 / math.sqrt(float(c * c + s * s)))
            self.c, self.s = self.c * g, self.s * g
        if self.n % self.dec:
            return None

        z = complex(float(self.i2), float(self.q2))
        if len(self.ring) == self.fine:
            self.rf += z * self.ring[0].conjugate()
            self.rc += z * self.ring[-self.coarse].conjugate()
            self.p += abs(z) ** 2
            self.count += 1
        self.ring.append(z)
        if self.count < self.window:
            return None

        coarse_coh = abs(self.rc) / max(self.p, 1e-30)
        fine_coh = abs(self.rf) / max(self.p, 1e-30)
        result = None
        # Noise decorrelates over the 1 s lag and a steady tone does not, so
        # the fine-lag coherence is the lock test. Its angle can alias for a
        # large error, so the 0.2 s lag sets the step when far off.
        if fine_coh >= LOCK:
            df = -math.atan2(self.rc.imag, self.rc.real) / (2 * math.pi * self.coarse / self.bb)
            if abs(df) < 0.3:
                df = -math.atan2(self.rf.imag, self.rf.real) / (2 * math.pi * self.fine / self.bb)
            f = min(self.nominal + 3.0, max(self.nominal - 3.0, self.mix + df))
            self.estimate = f
            result = f
            if abs(f - self.mix) > 0.005:
                self.retune(f)
                self.ring.clear()     # old baseband belongs to the old mixer
        self._clear()
        return result


def fft_estimate(block, fs, nominal=50.0):
    x = block - block.mean()
    sp = np.abs(np.fft.rfft(x * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), 1 / fs)
    idx = np.flatnonzero((fr >= nominal - 3) & (fr <= nominal + 3))
    k = idx[np.argmax(sp[idx])]
    a, b, c = sp[k - 1], sp[k], sp[k + 1]
    return fr[k] + 0.5 * (a - c) / (a - 2 * b + c) * (fr[1] - fr[0])


def main():
    scenarios = [
        ("49.60 steady", lambda t: np.full_like(t, 49.60)),
        ("50 wander +/-0.05", lambda t: 50.0 + 0.05 * np.sin(2 * np.pi * t / 40)),
        ("step 49.90->50.08", lambda t: np.where(t < 45, 49.90, 50.08)),
        ("49.20 far off", lambda t: np.full_like(t, 49.20)),
    ]
    print(f"{'fs':>5}{'mains':>7}  {'scenario':<19}{'first lock':>11}{'err rms':>10}"
          f"{'max':>8}{'locked':>8}{'fft rms':>9}")
    for fs in (250.0, 1000.0):
        for mains_uv in (1.0, 2.0, 5.0, 40.0):
            for name, f_fn in scenarios:
                x, f_true = make(fs, 90.0, mains_uv, f_fn)
                tr = Tracker(fs)
                first, errs, evals, locks = None, [], 0, 0
                for i, v in enumerate(x):
                    r = tr.n % tr.dec == tr.dec - 1 and tr.count == tr.window - 1
                    out = tr.push(v)
                    if r:
                        evals += 1
                    if out is None:
                        continue
                    locks += 1
                    if first is None:
                        first = i / fs
                    if i / fs > 20:
                        errs.append(out - f_true[max(0, i - int(2 * fs))])
                e = np.array(errs) if errs else np.array([np.nan])
                fe = np.array([fft_estimate(x[j - 4000:j], fs) - f_true[j - 2000]
                               for j in range(4000, len(x), int(5 * fs))])
                fl = f"{first:8.1f} s" if first is not None else "     never"
                print(f"{fs:>5.0f}{mains_uv:>5.0f}uV  {name:<19}{fl:>11}"
                      f"{np.sqrt(np.nanmean(e ** 2)):>7.4f} Hz{np.nanmax(np.abs(e)):>8.3f}"
                      f"{locks:>5}/{max(evals, locks):<3}{np.sqrt(np.mean(fe ** 2)):>7.4f}")
    for fs in (250.0, 1000.0):
        x, _ = make(fs, 90.0, 0.0, lambda t: np.full_like(t, 50.0), seed=9)
        tr = Tracker(fs)
        claims = sum(1 for v in x if tr.push(v) is not None)
        print(f"no mains at all, {fs:.0f} SPS: {claims} locks claimed in 90 s")
    tr = Tracker(250.0)
    print(f"cost: 18 float ops a sample; 10 a second at baseband; one atan2 every "
          f"4 s. State: {2 * tr.fine} ring floats + 12 others.")


if __name__ == "__main__":
    main()
