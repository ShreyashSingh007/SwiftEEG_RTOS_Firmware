"""
Mains frequency tracking cheap enough for the MCU: how accurate is it?

Estimator, per sample: mix the channel mean down by the nominal mains
frequency with a rotating phasor, low-pass the two halves with two one-pole
stages each, and every 1/25 s keep one baseband value. Over each 2 s window
the frequency offset is the angle of sum(z[k] * conj(z[k-1])) - the phase the
baseband tone turns per step. All float32, as the firmware would run it.

Compared against the app's current method: a Hann-windowed FFT of 4000
samples with parabolic peak interpolation.
"""
import math

import numpy as np

F32 = np.float32


def make(fs, seconds, mains_uv, f_fn, seed=2):
    rng = np.random.default_rng(seed)
    n = int(seconds * fs)
    t = np.arange(n) / fs
    phase = 2 * np.pi * np.cumsum(f_fn(t)) / fs
    # Channel mean of EEG-like activity: pink-ish noise, alpha, drift.
    white = rng.standard_normal(n)
    pink = np.cumsum(white) * 0.05
    pink -= np.convolve(pink, np.ones(int(fs)) / fs, mode="same")
    x = (8 * pink / pink.std() + 3 * rng.standard_normal(n)
         + 10 * np.sin(2 * np.pi * 10 * t) + 300 * np.sin(2 * np.pi * 0.1 * t)
         + mains_uv * np.sin(phase))
    return x, f_fn(t)


class Tracker:
    """What the firmware would run, op for op, in float32."""

    def __init__(self, fs, nominal=50.0, bb_rate=25.0, window_s=2.0, corner_hz=4.0):
        self.fs = fs
        self.nominal = nominal
        w = 2 * math.pi * nominal / fs
        self.cw, self.sw = F32(math.cos(w)), F32(math.sin(w))
        self.c, self.s = F32(1.0), F32(0.0)
        self.k = F32(1.0 - math.exp(-2 * math.pi * corner_hz / fs))
        self.i1 = self.i2 = self.q1 = self.q2 = F32(0.0)
        self.dec = int(round(fs / bb_rate))
        self.bb_rate = fs / self.dec
        self.window = int(round(window_s * self.bb_rate))
        self.n = 0
        self.prev = None
        self.acc_re = self.acc_im = self.power = F32(0.0)
        self.count = 0
        self.estimate = None
        self.ops = 0

    def push(self, x):
        x = F32(x)
        # Rotate the phasor one sample; renormalise now and then.
        c = self.c * self.cw - self.s * self.sw
        s = self.s * self.cw + self.c * self.sw
        self.c, self.s = c, s
        # Mix down and low-pass, two one-pole stages on each half.
        i, q = x * c, x * s
        self.i1 += self.k * (i - self.i1)
        self.i2 += self.k * (self.i1 - self.i2)
        self.q1 += self.k * (q - self.q1)
        self.q2 += self.k * (self.q1 - self.q2)
        self.ops += 16
        self.n += 1
        if self.n % 4096 == 0:
            g = F32(1.0) / F32(math.sqrt(float(c * c + s * s)))
            self.c, self.s = self.c * g, self.s * g
        if self.n % self.dec:
            return None
        zr, zi = self.i2, self.q2
        if self.prev is not None:
            pr, pi = self.prev
            self.acc_re += zr * pr + zi * pi
            self.acc_im += zi * pr - zr * pi
            self.power += zr * zr + zi * zi
            self.ops += 8
            self.count += 1
        self.prev = (zr, zi)
        if self.count < self.window:
            return None
        # Mixing by cos/sin of +w leaves the tone at (f - nominal), turning
        # clockwise for a positive offset.
        turn = math.atan2(float(self.acc_im), float(self.acc_re))
        coherence = math.hypot(float(self.acc_re), float(self.acc_im)) / max(float(self.power), 1e-30)
        amp = 2 * math.sqrt(float(self.power) / self.count)
        self.acc_re = self.acc_im = self.power = F32(0.0)
        self.count = 0
        if coherence < 0.8 or amp < 0.5:
            return ("no lock", amp, coherence)
        self.estimate = self.nominal - turn * self.bb_rate / (2 * math.pi)
        return (self.estimate, amp, coherence)


def fft_estimate(block, fs, nominal=50.0):
    x = block - block.mean()
    w = np.hanning(len(x))
    sp = np.abs(np.fft.rfft(x * w))
    fr = np.fft.rfftfreq(len(x), 1 / fs)
    m = (fr >= nominal - 3) & (fr <= nominal + 3)
    idx = np.flatnonzero(m)
    k = idx[np.argmax(sp[m])]
    a, b, c = sp[k - 1], sp[k], sp[k + 1]
    return fr[k] + 0.5 * (a - c) / (a - 2 * b + c) * (fr[1] - fr[0])


def main():
    scenarios = [
        ("49.70 Hz steady", lambda t: np.full_like(t, 49.70)),
        ("50.00 Hz wandering +/-0.05", lambda t: 50.0 + 0.05 * np.sin(2 * np.pi * t / 40)),
        ("step 49.90 -> 50.08 at 30 s", lambda t: np.where(t < 30, 49.90, 50.08)),
    ]
    print(f"{'fs':>5} {'mains':>6}  {'scenario':<28}{'tracker err rms':>16}{'max':>8}"
          f"{'updates':>9}{'fft err rms':>13}")
    for fs in (250.0, 1000.0):
        for mains_uv in (1.0, 5.0, 40.0):
            for name, f_fn in scenarios:
                x, f_true = make(fs, 60.0, mains_uv, f_fn)
                tr = Tracker(fs)
                errs = []
                locks = 0
                for i, v in enumerate(x):
                    r = tr.push(v)
                    if r is None:
                        continue
                    if r[0] == "no lock":
                        continue
                    locks += 1
                    if i / fs > 6:  # after start-up
                        errs.append(r[0] - f_true[max(0, i - int(fs))])
                e = np.array(errs) if errs else np.array([np.nan])
                fe = []
                for j in range(4000, len(x), int(5 * fs)):
                    fe.append(fft_estimate(x[j - 4000:j], fs) - f_true[j - 2000])
                fe = np.array(fe)
                print(f"{fs:>5.0f} {mains_uv:>5.0f}uV  {name:<28}"
                      f"{np.sqrt(np.nanmean(e ** 2)):>13.4f} Hz{np.nanmax(np.abs(e)):>8.3f}"
                      f"{locks:>9}{np.sqrt(np.mean(fe ** 2)):>10.4f} Hz")
        tr = Tracker(fs)
        print(f"  per-sample float ops at {fs:.0f} SPS: {tr.dec and 16} each sample, "
              f"8 more every {tr.dec} samples")

    # With no mains at all it must not claim a lock.
    x, _ = make(250.0, 60.0, 0.0, lambda t: np.full_like(t, 50.0))
    tr = Tracker(250.0)
    claims = sum(1 for v in x if (r := tr.push(v)) is not None and r[0] != "no lock")
    print(f"no mains present: {claims} false locks in 60 s")


if __name__ == "__main__":
    main()
