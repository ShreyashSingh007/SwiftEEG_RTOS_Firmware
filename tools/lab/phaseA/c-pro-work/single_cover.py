# Coverage of a candidate per-section multiplier M(k) = B + C d^2, d = min(k/2, 2/k), over every initial state.
import math, sys
import numpy as np
def tau_new(g, k):
    if k < 2:
        return 1 / math.atanh(g * k / (1 + g * g))
    s1 = 2 / (k + math.sqrt(k * k - 4))
    return 0.5 / math.atanh(s1 * min(g, 1 / g))
B = float(sys.argv[1]) if len(sys.argv) > 1 else math.log(100 * math.sqrt(2))
C = float(sys.argv[2]) if len(sys.argv) > 2 else 3.04

def tau_safe(g, k):
    try:
        return tau_new(g, k)
    except ValueError:
        return 0.0

def worst(g, k, tau):
    a1 = 1 / (1 + g * (g + k)); a2 = g * a1; a3 = g * a2
    A = np.array([[2 * a1 - 1, -2 * a2], [2 * a2, 1 - 2 * a3]])
    N = int(14 * tau) + 64
    Y = np.empty((N, 2)); r = np.array([a2, 1 - a3])
    for n in range(N):
        Y[n] = r; r = r @ A
    q, _ = np.linalg.qr(Y)
    ph = np.linspace(0, np.pi, 720, endpoint=False)
    E = np.abs(q @ np.vstack((np.cos(ph), np.sin(ph))))
    last = N - np.argmax((E > 0.01 * E.max(axis=0))[::-1], axis=0)
    return int(last.max())

gs = [0.002, 0.01, 0.05, 0.2, 0.5, 0.8, 0.9, 0.939, 0.97, 1.0, 1.03, 1.1, 1.25, 2.0, 5.0, 12.7]
ks = [0.004, 0.02, 0.0833, 0.2, 0.35, 0.5, 0.7, 1.0, 1.2, 1.414, 1.6, 1.7, 1.848, 1.9, 1.96, 1.99, 2.0,
      2.01, 2.05, 2.2, 2.5, 3.0, 4.0, 6.0, 10.0, 20.0, 50.0, 320.0]
print(f"M = {B:.3f} + {C:.3f} d^2")
under = 0
for k in ks:
    d = min(k / 2, 2 / k); M = B + C * d * d
    worst_ratio = 0; worst_T = 0; note = ""
    for g in gs:
        tau = tau_safe(g, k)
        if tau * 14 > 2e6:
            continue
        n = worst(g, k, tau)
        flag = math.ceil(M * tau)
        if tau > 0:
            worst_T = max(worst_T, n / tau)
        worst_ratio = max(worst_ratio, n / max(flag, 1))
        if n > flag:
            under += 1; note += f" UNDER g={g}: {n}>{flag}"
    print(f"k={k:8.4f} M={M:5.2f} worst T/tau={worst_T:5.2f} worst actual/flagged={worst_ratio:4.2f}{note}", flush=True)
print("under-covered cells:", under)
