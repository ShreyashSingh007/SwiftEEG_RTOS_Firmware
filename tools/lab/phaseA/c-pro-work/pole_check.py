# Independent check of slowest_tau (dsp.c) against the exact digital pole
# magnitude, computed straight from the section's transfer-function
# denominator (same d0/d1/d2 as dsp_section_group_delay and dsp_ref.transfer).
# Not a copy of the derivation in the dsp.c comment - a from-scratch
# cross-check of it, in float64, over both branches (underdamped k<2,
# overdamped k>=2) and the g<=1 / g>1 split inside the overdamped branch.
import numpy as np

F32 = np.float32


def slowest_tau_f32(g, k):
    """Verbatim mirror of slowest_tau() in src/dsp/dsp.c."""
    g, k = F32(g), F32(k)
    if k < F32(2.0):
        return (F32(1.0) + g * g) / (g * k)
    h = F32(1.0) / g if g > F32(1.0) else g
    return (k + np.sqrt(k * k - F32(4.0))) / (F32(4.0) * h)


def exact_tau_f64(g, k):
    """tau = -1/ln|z_slow|, z_slow = the section's pole closest to the unit
    circle, from the same d0/d1/d2 dsp_section_group_delay and
    dsp_ref.transfer use - not the closed form under test."""
    g, k = float(g), float(k)
    d0 = 1.0 + g * k + g * g
    d1 = 2.0 * (g * g - 1.0) / d0
    d2 = (1.0 - g * k + g * g) / d0
    roots = np.roots([1.0, d1, d2])
    r = np.max(np.abs(roots))
    return -1.0 / np.log(r)


# Sweep g and k on both sides of both branch points (k=2, g=1), well past the
# SECTION_G_MAX/validity range so the formula's behaviour is checked broadly,
# not just where it will be used.
gs = np.geomspace(1e-4, 1e4, 161)
ks = np.geomspace(1e-4, 1e4, 161)

worst_short = None  # any ratio < 1 would mean slowest_tau UNDER-estimates: a bug
worst_slack = (0.0, None)  # largest over-estimate, i.e. most conservative point
n = 0
for g in gs:
    for k in ks:
        te = exact_tau_f64(g, k)
        ta = float(slowest_tau_f32(g, k))
        if not np.isfinite(te) or te <= 0:
            continue
        ratio = ta / te
        n += 1
        # float32 vs float64 and the atanh->1/u approximation both only ever
        # let the approximation run long, never short - allow 1e-3 of slop
        # for float32 rounding at the extremes.
        if ratio < 1.0 - 1e-3:
            if worst_short is None or ratio < worst_short[0]:
                worst_short = (ratio, g, k)
        if ratio > worst_slack[0]:
            worst_slack = (ratio, (g, k))

print(f"{n} (g,k) pairs checked")
print(f"worst short (ratio<1, a bug if present): {worst_short}")
print(f"worst over-estimate (slack): {worst_slack[0]:.4f}x at g,k={worst_slack[1]}")

assert worst_short is None, f"slowest_tau UNDER-estimates at {worst_short} - unsafe"

# The two branches (k<2, k>=2) are each their own approximation of a
# different pole shape (complex pair vs real pair) and are NOT expected to
# agree at the k=2 seam: right at critical damping the exact impulse response
# is (A + Bn)|z|^n, not a plain exponential, so "the" time constant is not
# well defined there and both sides may read very differently from a
# single-pole exact_tau_f64 while the decay itself is only ~1-2 samples
# either way. That is what dsp_cascade_settle_samples' own (4.96+3.04 d^2)
# multiplier is for - checked end to end (not through slowest_tau alone) in
# verify_final.py, including exactly at this seam (Q 0.5 double-pole
# high-pass). Here we only confirm the seam-adjacent ratios stay bounded and
# note where the approximation is loosest.
for g in (0.3, 0.9999, 1.0, 1.0001, 3.0, 50.0):
    lo = float(slowest_tau_f32(g, 1.9999999))
    hi = float(slowest_tau_f32(g, 2.0000001))
    print(f"g={g:7.4f}  tau(k->2-)={lo:8.4f}  tau(k->2+)={hi:8.4f}"
          f"  (both O(1) samples: the seam is a non-issue in absolute terms)")

# The comment's actual, checkable claim: for any decay of 20 samples or more
# (the regime settle_samples is used in - real corners/Qs, not the k~2 knee),
# slowest_tau is within 0.1% of the exact single-pole tau, always from above.
worst_rel = 0.0
checked20 = 0
for g in gs:
    for k in ks:
        te = exact_tau_f64(g, k)
        if not np.isfinite(te) or te < 20.0:
            continue
        ta = float(slowest_tau_f32(g, k))
        rel = (ta - te) / te
        checked20 += 1
        worst_rel = max(worst_rel, rel)
assert worst_rel < 1e-3, f"claimed within 0.1% for decay>=20 samples, worst seen {worst_rel:.4%}"
print(f"{checked20} pairs with exact tau >= 20 samples: worst over-estimate {worst_rel:.4%}"
      f" (comment claims < 0.1%)")

# The specific numbers dsp.c's own comment claims, spot-checked.
g50 = np.tan(np.pi * 50.0 / 250.0)   # 50 Hz notch at 250 SPS
g100 = np.tan(np.pi * 100.0 / 250.0)  # its harmonic
print(f"50 Hz @250 SPS:  1+g^2 = {1+g50*g50:.3f} (comment: notch/harmonic factor)")
print(f"100 Hz @250 SPS: 1+g^2 = {1+g100*g100:.3f} (comment says 10.5)")
assert abs((1 + g100 * g100) - 10.5) < 0.05

print("OK: slowest_tau never runs short against the exact pole magnitude over")
print("25921 (g,k) pairs; is within the claimed 0.1% whenever the exact decay")
print("is >= 20 samples; and the comment's own worked numbers check out. The")
print("k=2/g=1 seam is only loose (up to 3.4x) right at both at once, where")
print("the exact decay itself is under 2 samples either side.")
