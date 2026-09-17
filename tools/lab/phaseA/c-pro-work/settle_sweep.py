# Settling estimate vs the real ring-down of a primed restart, the device's SVF loop.
# Transient = y(primed restart) - y(same filter running for a long time) = zero-input
# response of the cascade from the state difference (linear), so it is computed as
# C A^n delta. "Actual" = last sample above 1 % of the transient's own peak, + 1.
#   realistic: delta from restarts on EEG-like input (walk, alpha, drift, mains + harmonic)
#   arbitrary: delta in random directions (worst case over initial state)
import math, sys
import numpy as np

F32 = np.float32
MULT = float(sys.argv[1]) if len(sys.argv) > 1 else 4.6

def section(fs, fc, q, kind):
    g = F32(math.tan(math.pi * fc / fs)); k = F32(1.0 / q)
    m0, m1k, m2 = {'notch': (1, -1, 0), 'lp': (0, 0, 1), 'hp': (1, -1, -1)}[kind]
    return (g, k, F32(m0), F32(m1k) * k, F32(m2))

def bw_qs(order):
    return sorted(1.0 / (2.0 * math.sin(math.pi * (2 * i + 1) / (2 * order))) for i in range(order // 2))

def tau_old(g, k):
    g, k = float(g), float(k)
    return (k + math.sqrt(k * k - 4)) / (4 * g) if k > 2 else 1 / (g * k)

def tau_new(g, k):
    g, k = float(g), float(k)
    if k < 2:
        return 1 / math.atanh(g * k / (1 + g * g))
    s1 = 2 / (k + math.sqrt(k * k - 4))
    return 0.5 / math.atanh(s1 * min(g, 1 / g))

def flagged(secs, taufn, mult=4.6):
    return int(math.ceil(sum(mult * taufn(s[0], s[1]) for s in secs)))

def derive(s):
    g, k, m0, m1, m2 = s
    a1 = F32(1) / (F32(1) + g * (g + k)); a2 = g * a1; a3 = g * a2
    return tuple(float(v) for v in (a1, a2, a3, m0, m1, m2))

def step(run, ic1, ic2, x):
    for i, (a1, a2, a3, m0, m1, m2) in enumerate(run):
        v3 = x - ic2[i]; v1 = a1 * ic1[i] + a2 * v3; v2 = ic2[i] + a2 * ic1[i] + a3 * v3
        ic1[i] = 2 * v1 - ic1[i]; ic2[i] = 2 * v2 - ic2[i]
        x = m0 * x + m1 * v1 + m2 * v2
    return x

def state_space(run):
    S = len(run); D = 2 * S
    A = np.zeros((D, D)); C = np.zeros(D)
    for j in range(D):
        e = np.zeros(D); e[j] = 1.0
        ic1, ic2 = list(e[0::2]), list(e[1::2])
        C[j] = step(run, ic1, ic2, 0.0)
        A[0::2, j] = ic1; A[1::2, j] = ic2
    return A, C

def responses(A, C, N):
    Y = np.empty((N, len(C))); r = C.copy()
    for n in range(N):
        Y[n] = r; r = r @ A
    return Y

def eeg(fs, n, seed, mains, m_uv, h_uv):
    rng = np.random.default_rng(seed)
    t = np.arange(n) / fs
    walk = np.cumsum(rng.standard_normal(n))
    w = int(fs); cs = np.concatenate(([0.0], np.cumsum(walk)))
    ma = (cs[w:] - cs[:-w]) / w
    walk = walk - np.concatenate((np.full(w // 2, ma[0]), ma, np.full(n - len(ma) - w // 2, ma[-1])))
    return (8 * walk / walk.std() + 10 * np.sin(2 * np.pi * 10.3 * t + rng.uniform(0, 6.3))
            + 300 * np.sin(2 * np.pi * 0.1 * t) + m_uv * np.sin(2 * np.pi * mains * t)
            + h_uv * np.sin(2 * np.pi * 2 * mains * t + rng.uniform(0, 6.3)))

def realistic_deltas(run, fs, warm, span, count, mains, m_uv, h_uv, seed):
    n = warm + span
    x = eeg(fs, n, seed, mains, m_uv, h_uv)
    S = len(run)
    at = set(np.linspace(warm, n - 1, count).astype(int))
    ic1 = [0.0] * S; ic2 = [0.0] * S
    v = x[0]
    for i, r in enumerate(run):
        ic2[i] = v; v = (r[3] + r[5]) * v
    out = []
    for idx in range(n):
        if idx in at:
            d = np.empty(2 * S); v = x[idx]
            for i, r in enumerate(run):
                d[2 * i] = 0.0 - ic1[i]; d[2 * i + 1] = v - ic2[i]; v = (r[3] + r[5]) * v
            out.append(d)
        step(run, ic1, ic2, x[idx])
    return np.array(out).T

def actual(Y, deltas):
    N = Y.shape[0]; worst = 0
    for c in range(0, deltas.shape[1], 64):
        E = np.abs(Y @ deltas[:, c:c + 64])
        pk = E.max(axis=0)
        mask = E > 0.01 * pk
        last = N - np.argmax(mask[::-1], axis=0)
        worst = max(worst, int(last.max()))
    return worst

def case(name, fs, secs, mains=50.0, h_uv=10.0):
    run = [derive(s) for s in secs]
    A, C = state_space(run)
    new = flagged(secs, tau_new, MULT); old = flagged(secs, tau_old)
    tmax = max(tau_new(s[0], s[1]) for s in secs)
    N = int(max(1.6 * new, 14 * tmax)) + 64
    Y = responses(A, C, N)
    rng = np.random.default_rng(1)
    arb = rng.standard_normal((2 * len(secs), 512))
    real = np.hstack([realistic_deltas(run, fs, 2 * N, int(4 * fs), 48, mains, 50.0, hu, 7 + j)
                      for j, hu in enumerate((h_uv, 50.0))])
    ar, ra = actual(Y, arb), actual(Y, real)
    tag = '' if new >= ra else '  <-- UNDER (realistic)'
    tag += '' if new >= ar else '  <-- under (arbitrary state)'
    print(f"{name:34s} {fs:5.0f} | old {old:6d} new {new:6d} | realistic {ra:6d} ({new / ra:4.2f}x)"
          f" arbitrary {ar:6d} ({new / ar:4.2f}x){tag}", flush=True)
    return new, ra, ar

print(f"multiplier {MULT}")
for fs in (250.0, 500.0, 1000.0):
    for f0 in (50.0, 60.0):
        secs = [section(fs, f0, 12, 'notch')]
        if 2 * f0 < 0.95 * 0.5 * fs:
            secs.append(section(fs, 2 * f0, 12, 'notch'))
        case(f"notch {f0:.0f} Q12{' + harmonic' if len(secs) > 1 else ''}", fs, secs, f0)
    case("notch 50 Q30 alone", fs, [section(fs, 50.0, 30, 'notch')])
    for fc in (0.1, 0.5, 1.0):
        for order in (2, 4):
            case(f"high-pass {fc} Hz order {order}", fs, [section(fs, fc, q, 'hp') for q in bw_qs(order)])
    for fc in (40.0, 70.0, 100.0):
        if fc < 0.95 * 0.5 * fs:
            for order in (2, 4):
                case(f"low-pass {fc:.0f} Hz order {order}", fs, [section(fs, fc, q, 'lp') for q in bw_qs(order)])
    chain = ([section(fs, 0.5, q, 'hp') for q in bw_qs(4)] + [section(fs, 50.0, 12, 'notch')]
             + ([section(fs, 100.0, 12, 'notch')] if 100 < 0.95 * 0.5 * fs else [])
             + [section(fs, 45.0, q, 'lp') for q in bw_qs(4)])
    case("chain hp0.5o4 + notch50 + lp45o4", fs, chain)
