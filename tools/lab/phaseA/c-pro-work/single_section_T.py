# Worst case, over every initial state, of a single section's ring-down to 1 % of its own peak,
# in units of the slowest pole's time constant tau = -1/ln|z|. Independent of the mix.
import math
import numpy as np

def tau_new(g, k):
    if k < 2:
        return 1 / math.atanh(g * k / (1 + g * g))
    s1 = 2 / (k + math.sqrt(k * k - 4))
    return 0.5 / math.atanh(s1 * min(g, 1 / g))

def worst_samples(g, k):
    a1 = 1 / (1 + g * (g + k)); a2 = g * a1; a3 = g * a2
    A = np.array([[2 * a1 - 1, -2 * a2], [2 * a2, 1 - 2 * a3]])
    tau = tau_new(g, k)
    N = int(14 * tau) + 64
    Y = np.empty((N, 2)); r = np.array([a2, 1 - a3])  # low-pass output row; any generic mix spans the same
    for n in range(N):
        Y[n] = r; r = r @ A
    q, _ = np.linalg.qr(Y)
    ph = np.linspace(0, np.pi, 360, endpoint=False)
    E = np.abs(q @ np.vstack((np.cos(ph), np.sin(ph))))
    pk = E.max(axis=0)
    last = N - np.argmax((E > 0.01 * pk)[::-1], axis=0)
    return int(last.max()), tau

gs = [0.002, 0.01, 0.05, 0.2, 0.5, 1.0, 2.0, 5.0, 12.7]
ks = [0.02, 0.0833, 0.2, 0.5, 1.0, 1.414, 1.7, 1.848, 1.96, 2.0, 2.05, 2.5, 4.0, 10.0, 50.0]
print("rows k, columns g: worst last-sample / tau (samples in brackets for tau < 5)")
print("k \\ g   " + "".join(f"{g:>9g}" for g in gs))
for k in ks:
    cells = []
    for g in gs:
        n, tau = worst_samples(g, k)
        cells.append(f"{n / tau:6.2f}" + (f"[{n}]" if tau < 5 else "   "))
    print(f"{k:7.4f} " + "".join(cells))
