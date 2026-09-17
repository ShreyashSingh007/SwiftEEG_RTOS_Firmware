# Where does the float32-derived SVF loop stop being stable, as a function of g and k?
# Loop state matrix (x = 0): A = [[2a1-1, -2a2], [2a2, 1-2a3]], a1..a3 derived in float32 as dsp.c load().
import numpy as np
F32 = np.float32

def derive32(g, k):
    g = F32(g); k = F32(k)
    with np.errstate(over='ignore'):
        a1 = F32(1.0) / (F32(1.0) + g * (g + k))
    a2 = g * a1
    a3 = g * a2
    return float(a1), float(a2), float(a3)

def ln_radius32(g, k):
    """ln of the spectral radius of the float32-derived loop, computed in float64."""
    a1, a2, a3 = derive32(g, k)
    tr = 2 * a1 - 2 * a3
    det = (2 * a1 - 1) * (1 - 2 * a3) + 4 * a2 * a2
    disc = tr * tr - 4 * det
    if disc < 0:
        return 0.5 * np.log1p(det - 1)
    r = np.sqrt(disc)
    return np.log(max(abs((tr + r) / 2), abs((tr - r) / 2)))

def ln_radius_exact(g, k):
    """ln|z| of the slowest pole of the exact bilinear section."""
    g = float(g); k = float(k)
    if k < 2:
        return -np.arctanh(g * k / (1 + g * g))
    h = min(g, 1 / g)
    s1 = 2 / (k + np.sqrt(k * k - 4))
    return -2 * np.arctanh(s1 * h)

gs = np.logspace(-8, 5, 131)
ks = np.logspace(-6, 4, 101)
rows = []
for g in gs:
    for k in ks:
        ex = ln_radius_exact(g, k)
        f = ln_radius32(g, k)
        tau = -1 / ex if ex < 0 else np.inf
        rows.append((g, k, tau, f / ex if ex < 0 else np.nan, f))
rows = np.array(rows)
tau = rows[:, 2]; ratio = rows[:, 3]; lnr32 = rows[:, 4]
for tmax in (1e5, 1e6, 4e6, 1e7, 1e8):
    sel = tau <= tmax
    bad = sel & (lnr32 >= 0)
    dev = np.nanmax(np.abs(ratio[sel] - 1))
    print(f"tau <= {tmax:.0e}: {sel.sum()} grid points, float32 unstable/undamped at {bad.sum()}, "
          f"worst float32 damping error {100 * dev:.1f} %")
# Box-only regions: legit g range with margin and k range, ignoring tau
for (glo, ghi, klo, khi) in ((1e-6, 1e3, 1e-3, 1e3), (1e-6, 1e3, 1e-4, 1e3), (1e-7, 1e3, 1e-4, 1e4)):
    sel = (rows[:, 0] >= glo) & (rows[:, 0] <= ghi) & (rows[:, 1] >= klo) & (rows[:, 1] <= khi)
    bad = sel & (lnr32 >= 0)
    print(f"box g[{glo:g},{ghi:g}] k[{klo:g},{khi:g}]: {sel.sum()} points, float32 unstable/undamped at {bad.sum()}, "
          f"worst damping error {100 * np.nanmax(np.abs(ratio[sel] - 1)):.0f} %")
# The review's cases
for g, k in ((4096, 1 / 12), (4096, 1e-4), (1e3, 1e-4), (1e3, 1e-3), (100, 1e-4), (2e19, 0.7), (1.0, 1e30)):
    print(f"g={g:g} k={k:g}: a1..a3={derive32(g, k)}, ln r32={ln_radius32(g, k):.3e}, exact {ln_radius_exact(g, k):.3e}")
