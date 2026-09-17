# C-PRO log - phase A (R3-DSP-10 settle warping, R3-DSP-06 section bounds)

Owned files: src/dsp/dsp.c, src/dsp/dsp.h, tools/dsp_ref.py, tests/dsp/src/main.c
Work scripts: scratchpad/phaseA/c-pro-work/

## Status - all done
- [x] R3-DSP-10 settling time with bilinear warping - C, dsp_ref.py mirror, tests, proof
- [x] R3-DSP-06 section bounds (validation half) - C, dsp_ref.py mirror, tests, proof

Both fixes and the dsp_ref.py mirror were already sitting in the working tree from an
earlier, uninterrupted stretch of this task - this log just was never written, so I
re-derived and re-checked everything from the diff rather than trusting the summary
that said dsp_ref.py was only partway done (it was not; see below). Only
tests/dsp/src/main.c and this log needed work this run.

## R3-DSP-10 - settle estimate uses the bilinear-warped pole, not analogue g*k

dsp.c adds `slowest_tau(g, k)`: underdamped (k<2) returns (1+g^2)/(g*k) - the review's
suggested closed form, an upper bound on the exact -2/ln(d2) (d2 = the same pole-radius
term dsp_section_group_delay already computes) that never runs short. Overdamped
(k>=2) returns the analogous warped form, (k+sqrt(k^2-4))/(4h), h = 1/g for g>1 else g
(the faster real pole sits nearer the unit circle once g>1). dsp_cascade_settle_samples
now sums (4.96 + 3.04 d^2) * slowest_tau + 2 per section (d = damping ratio folded
about critical) instead of the old 4.6 * tau - see dsp.c's comments for the derivation.

Before trusting it further I independently re-derived slowest_tau myself
(pole_check.py, new this run): computed the *exact* digital pole magnitude from the
same d0/d1/d2 the group-delay function already uses (not the closed form under test),
swept g and k from 1e-4 to 1e4 (25921 pairs, both branches, past and short of the
validity bounds):
  - Never short: 0/25921 pairs had slowest_tau below the exact pole-derived tau.
  - Within the comment's claim: wherever the exact decay is >= 20 samples (24240
    pairs), the over-estimate is at most 0.0833 % (claimed < 0.1 %).
  - The two branches don't meet at k=2 for g near 1 (up to 3.4x apart), but only there,
    where the exact decay itself is under 2 samples either side - immaterial in
    absolute terms. The comment's own worked numbers (1+g^2 = 1.53 at 50 Hz/250 SPS,
    10.5 at the 100 Hz harmonic) check out exactly.
Nothing needed changing in dsp.c - this was verification, not a fix.

## R3-DSP-06 - section validation bounds (huge-but-finite rejection)

dsp_section_is_valid adds, beyond the existing finite/positive checks: |m0|,|m1|,|m2|
<= 1e3, g <= 1e3, and slowest_tau(g,k) <= 1e6. The tau bound (rather than a separate
floor on k) is what catches tiny k together with large g without also refusing
sections that work. Chain-level handling of a resulting non-finite channel is the
other half of R3-DSP-06 and is out of this task's scope per the brief.

## dsp_ref.py mirror

slowest_tau, SECTION_MIX_MAX/G_MAX/TAU_MAX and the new settle_samples formula are
mirrored operation-for-operation (checked line by line against dsp.c this run - same
order, same float32 casts, matches exactly). `python tools/dsp_ref.py` -> OK. Its
self-test asserts 6 huge-but-finite cases refused, 3 boundary-of-legitimate designs
accepted, and settle_samples-vs-simulated-worst-ring-down for a notch+harmonic pair and
a highpass, each from 16 random restart states.

## Numerical proof (scratchpad/phaseA/c-pro-work/)

verify_final.py (already existed; reran this session, output identical, ALL CHECKS
PASSED):
  - R3-DSP-06: 13080 sections swept from tools/eeg_dsp.py and dsp_ref.py's own design
    functions (high-pass 0.05-5 Hz orders 1-8, low-pass 20 Hz-0.4749 fs orders 1-8,
    notch 50/60 +-3 Hz Q 1-255 with harmonics, 250 SPS-16 kSPS) all accepted; worst tau
    2.61e5 samples, 3.8x inside the 1e6 bound. 9 huge-but-finite cases (1e30 mix,
    g=4096/2e19, k=1e-9/1e30, g=1e-9, g=1000+k=1e-3) all refused.
  - R3-DSP-10: flagged settle samples vs. worst-case ring-down to 1% of peak, simulated
    from the cascade's state-space impulse response over 4096 random initial
    directions (covers every state a restart can leave a section in):
      Q30 notch 50Hz@1k (golden_notch50): flagged  966  worst  900  (1.07x)
      notch 50+100 Q12@250:               flagged  333  worst  194  (1.72x)
      notch 60 Q12@250:                   flagged  122  worst  119  (1.03x)
      hp 0.5Hz order2@250:                flagged  732  worst  641  (1.14x)
      hp 0.1Hz order4@1k:                 flagged 35499 worst 24737 (1.44x)
      lp 100Hz order2@250:                flagged   18  worst   13  (1.38x)
    Sections sharing a pole (2/4/8 identical - a stress case no real design produces)
    over-count more, up to 5.83x for 8x a Q0.5 double-pole high-pass; expected, and
    explained in the dsp.c comment.
  - Realistic restart scenario (48 restarts at random phase, 5:1 and 1:1 mains:harmonic
    amplitude, matching the review's own methodology):
      250 SPS 50 Hz Q12+harmonic: old 94  new 333  realistic ring-down 175  (1.90x)
      250 SPS 60 Hz Q12:          old 59  new 122  realistic ring-down 117  (1.04x)
      500 SPS 50 Hz Q12+harmonic: old 246 new 333  realistic ring-down 191  (1.74x)
     1000 SPS 50 Hz Q12+harmonic: old 519 new 593  realistic ring-down 361  (1.64x)
    old/realistic reproduce the review's own reported numbers (94, 59, ~190, 519)
    almost exactly, and old < realistic at 250 SPS both cases - the bug, reproduced.
    new >= realistic everywhere, never more than 1.9x - fixed, and not grossly
    conservative.

pole_check.py (new this run): see R3-DSP-10 section above for the numbers.

## tests/dsp/src/main.c

- test_settle_samples_are_sane: expected range for golden_notch50 (Q30, 50 Hz, 1 kSPS)
  updated from the old formula's 800-960 to 940-995; comment now derives ~966 from the
  new formula. 940-995 sits well above the old (buggy) value of ~872, so a regression
  back to the unwarped formula fails this test.
- New test_section_bounds_reject_huge_but_finite: the 9 huge-but-finite cases from
  dsp_ref.py's self-test, built the way test_invalid_sections_are_refused already
  builds its bad sections (copy golden_notch50, mutate one field, assert
  dsp_cascade_set refuses it).
- New test_section_bounds_accept_designed_sections: the same three boundary-of-
  legitimate designs from dsp_ref.py's self-test (0.05 Hz order-8 high-pass at
  16 kSPS, 118 Hz low-pass at 250 SPS, Q255 notch at 16 kSPS), via
  dsp_design_highpass/lowpass/notch + dsp_section_is_valid.
- Not run through a Zephyr build (out of scope - no builds/flashing per the brief);
  checked by hand for brace/paren balance and tab indentation against the rest of the
  file (both match).

## Goldens

tests/dsp/golden_dsp.h does NOT need regenerating. Nothing in this fix touches a
function whose output is emitted there (dsp_design_*, dsp_cascade_apply, dsp_dc_apply,
dsp_lsb_uv, the mains tracker are all unchanged) - dsp_section_is_valid and
dsp_cascade_settle_samples don't appear in any golden vector. Did not run
tools/dsp_ref.py --emit or tools/gen_golden.py (the latter generates
tests/proto/golden_vectors.h for the proto codec - unrelated to this task either way).

## Not touched

tools/swifteeg_app.py and tools/swifteeg_link.py are also modified in the working tree
(python-pro's R4-HOST work, see scratchpad/phaseA/PYTHON-PRO.md) - outside this task's
owned files, left alone.
