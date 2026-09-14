/*
 * Tests for the pure modules: DSP primitives and the SPSC ring buffer.
 *
 * DSP results are checked against golden vectors from tools/dsp_ref.py,
 * which is the authoritative reference. Integer DC removal must match bit
 * for bit. Float paths allow a small tolerance: the reference designs its
 * sections in float64 and rounds them, where the firmware designs in float32,
 * and the last bit of a mantissa can differ.
 *
 * Build:  .\tools\build.ps1 -Target dsp
 */

#include <zephyr/ztest.h>

#include <math.h>
#include <string.h>

#include "dsp.h"
#include "ringbuf.h"
#include "golden_dsp.h"

/* Signal peaks near 88, so this is roughly 1 part in 10^4. */
#define FLOAT_TOL 5e-3f

/* Relative tolerance on a section designed here against the reference's. */
#define DESIGN_TOL 1e-5f

static void assert_section_close(const dsp_section_t *got,
				 const dsp_section_t *want, const char *what)
{
	const float g[5] = { got->g, got->k, got->m0, got->m1, got->m2 };
	const float w[5] = { want->g, want->k, want->m0, want->m1, want->m2 };

	for (int i = 0; i < 5; i++) {
		const float tol = DESIGN_TOL * fabsf(w[i]) + 1e-7f;

		zassert_within(g[i], w[i], tol, "%s value %d: %.9f vs %.9f",
			       what, i, (double)g[i], (double)w[i]);
	}
}

/* ------------------------------------------------------------------ DSP */

ZTEST(dsp, test_lsb_scaling)
{
	const float lsb = dsp_lsb_uv(4.5f, 24);

	zassert_within(lsb, GOLDEN_DSP_LSB_UV, 1e-9f,
		       "LSB %.9f uV, expected %.9f", (double)lsb,
		       (double)GOLDEN_DSP_LSB_UV);

	/* Gain 0 is nonsense and must not divide by zero. */
	zassert_equal(dsp_lsb_uv(4.5f, 0), 0.0f, "gain 0 not rejected");
}

ZTEST(dsp, test_notch_design_matches_reference)
{
	dsp_section_t s;

	zassert_true(dsp_design_notch(&s, GOLDEN_DSP_FS, 50.0f, 30.0f),
		     "notch design failed");
	assert_section_close(&s, &golden_notch50, "notch");
}

ZTEST(dsp, test_lowpass_design_matches_reference)
{
	dsp_section_t s;

	zassert_true(dsp_design_lowpass(&s, GOLDEN_DSP_FS, 40.0f,
					1.0f / sqrtf(2.0f)),
		     "lowpass design failed");
	assert_section_close(&s, &golden_lp40, "lowpass");
}

ZTEST(dsp, test_highpass_design_matches_reference)
{
	dsp_section_t s;

	zassert_true(dsp_design_highpass(&s, GOLDEN_DSP_FS, 0.1f,
					 GOLDEN_DSP_HP_Q),
		     "highpass design failed");
	assert_section_close(&s, &golden_hp01, "highpass");
}

ZTEST(dsp, test_design_rejects_bad_args)
{
	dsp_section_t s;

	/* At or above Nyquist is not designable. */
	zassert_false(dsp_design_notch(&s, 1000.0f, 500.0f, 30.0f),
		      "f0 at Nyquist accepted");
	zassert_false(dsp_design_notch(&s, 1000.0f, 600.0f, 30.0f),
		      "f0 above Nyquist accepted");
	zassert_false(dsp_design_lowpass(&s, 1000.0f, 40.0f, 0.0f),
		      "zero Q accepted");
	zassert_false(dsp_design_highpass(&s, 0.0f, 1.0f, 0.7f),
		      "zero fs accepted");
	zassert_false(dsp_design_notch(NULL, 1000.0f, 50.0f, 30.0f),
		      "null output accepted");
}

ZTEST(dsp, test_invalid_sections_are_refused)
{
	dsp_cascade_t c;
	dsp_section_t bad;

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_cascade_set(&c, &golden_notch50, 1), NULL);

	/*
	 * g and k must be positive and every value finite. Anything else is
	 * refused without touching what is loaded - a section from a host
	 * that would not be stable is not run.
	 */
	bad = golden_notch50;
	bad.g = 0.0f;
	zassert_false(dsp_cascade_set(&c, &bad, 1), "g = 0 accepted");

	bad = golden_notch50;
	bad.k = -1.0f;
	zassert_false(dsp_cascade_set(&c, &bad, 1), "negative k accepted");

	bad = golden_notch50;
	bad.m1 = NAN;
	zassert_false(dsp_cascade_set(&c, &bad, 1), "NaN accepted");

	bad = golden_notch50;
	bad.g = INFINITY;
	zassert_false(dsp_cascade_retune(&c, &bad, 1), "infinite g accepted");

	zassert_equal(c.count, 1, "a refused load changed the cascade");

	/* No sections at all is a pass-through, not an error. */
	zassert_true(dsp_cascade_set(&c, NULL, 0), "clearing refused");
	zassert_equal(c.count, 0, "cascade not cleared");
	zassert_equal(dsp_cascade_apply(&c, 0, 12.5f), 12.5f,
		      "an empty cascade must pass samples through");
}

ZTEST(dsp, test_integer_dc_removal_is_exact)
{
	dsp_dc_t dc;

	dsp_dc_init(&dc, GOLDEN_DSP_DC_SHIFT);

	for (int i = 0; i < GOLDEN_DSP_N; i++) {
		int32_t got = dsp_dc_apply(&dc, golden_dc_input[i]);

		/* Integer maths: this must be exact, not approximate. */
		zassert_equal(got, golden_dc_output[i],
			      "sample %d: got %d, reference %d",
			      i, got, golden_dc_output[i]);
	}
}

ZTEST(dsp, test_dc_priming_removes_startup_transient)
{
	dsp_dc_t dc;

	dsp_dc_init(&dc, 12);

	/*
	 * The whole point of a first-order section here is that it can be
	 * primed, so a large constant offset produces zero immediately rather
	 * than a multi-second decaying ramp.
	 */
	zassert_equal(dsp_dc_apply(&dc, 400000), 0,
		      "first sample not primed");

	for (int i = 0; i < 64; i++) {
		zassert_equal(dsp_dc_apply(&dc, 400000), 0,
			      "constant input leaked at %d", i);
	}

	/* Negative offsets must work too: arithmetic shift, not logical. */
	dsp_dc_init(&dc, 10);
	zassert_equal(dsp_dc_apply(&dc, -250000), 0, "negative prime failed");
	zassert_equal(dsp_dc_apply(&dc, -250000), 0, "negative constant leaked");
}

ZTEST(dsp, test_single_section_matches_reference)
{
	dsp_cascade_t c;

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_cascade_set(&c, &golden_notch50, 1), NULL);

	for (int i = 0; i < GOLDEN_DSP_N; i++) {
		float y = dsp_cascade_apply(&c, 0, golden_dsp_input[i]);

		zassert_within(y, golden_dsp_notched[i], FLOAT_TOL,
			       "sample %d: got %.6f, reference %.6f",
			       i, (double)y, (double)golden_dsp_notched[i]);
	}
}

ZTEST(dsp, test_two_section_cascade_matches_reference)
{
	dsp_cascade_t c;
	const dsp_section_t sections[2] = { golden_notch50, golden_lp40 };

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_cascade_set(&c, sections, 2), NULL);

	for (int i = 0; i < GOLDEN_DSP_N; i++) {
		float y = dsp_cascade_apply(&c, 0, golden_dsp_input[i]);

		zassert_within(y, golden_dsp_cascaded[i], FLOAT_TOL,
			       "sample %d: got %.6f, reference %.6f",
			       i, (double)y, (double)golden_dsp_cascaded[i]);
	}
}

ZTEST(dsp, test_low_corner_highpass_matches_reference)
{
	/*
	 * The case this section form exists for: a 0.1 Hz high-pass on 3 mV of
	 * slow signal. The direct form it replaced ended microvolts away from
	 * the float64 answer on input like this.
	 */
	dsp_cascade_t c;
	float worst = 0.0f;

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_cascade_set(&c, &golden_hp01, 1), NULL);

	for (int i = 0; i < GOLDEN_DSP_N_HP; i++) {
		const float y = dsp_cascade_apply(&c, 0, golden_dsp_drift[i]);
		const float d = fabsf(y - golden_dsp_highpassed[i]);

		if (d > worst) {
			worst = d;
		}

		zassert_within(y, golden_dsp_highpassed[i], FLOAT_TOL,
			       "sample %d: got %.6f, reference %.6f",
			       i, (double)y, (double)golden_dsp_highpassed[i]);
	}

	TC_PRINT("worst deviation %e uV\n", (double)worst);
}

ZTEST(dsp, test_channels_are_independent)
{
	dsp_cascade_t c;

	dsp_cascade_init(&c, DSP_MAX_CHANNELS);
	zassert_true(dsp_cascade_set(&c, &golden_notch50, 1), NULL);

	/*
	 * Drive channel 0 hard while channel 1 sees only zeros. If state were
	 * shared, channel 1 would produce non-zero output.
	 */
	for (int i = 0; i < 32; i++) {
		(void)dsp_cascade_apply(&c, 0, golden_dsp_input[i]);
		float quiet = dsp_cascade_apply(&c, 1, 0.0f);

		zassert_within(quiet, 0.0f, 1e-9f,
			       "channel 1 disturbed at %d: %.9f",
			       i, (double)quiet);
	}
}

ZTEST(dsp, test_reset_state_clears_history)
{
	dsp_cascade_t c;
	float first_run[16];

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_cascade_set(&c, &golden_notch50, 1), NULL);

	for (int i = 0; i < 16; i++) {
		first_run[i] = dsp_cascade_apply(&c, 0, golden_dsp_input[i]);
	}

	dsp_cascade_reset_state(&c);

	/* After a reset the same input must reproduce the same output. */
	for (int i = 0; i < 16; i++) {
		float y = dsp_cascade_apply(&c, 0, golden_dsp_input[i]);

		zassert_within(y, first_run[i], 1e-9f,
			       "reset did not restore initial state at %d", i);
	}
}

ZTEST(dsp, test_retune_keeps_state)
{
	/*
	 * Retuning to the very same sections must be invisible: the output
	 * carries on exactly as if nothing had happened. A retune that reset
	 * the state would put a transient in instead.
	 */
	dsp_cascade_t steady, retuned;
	const dsp_section_t sections[2] = { golden_notch50, golden_lp40 };

	dsp_cascade_init(&steady, 1);
	dsp_cascade_init(&retuned, 1);
	zassert_true(dsp_cascade_set(&steady, sections, 2), NULL);
	zassert_true(dsp_cascade_set(&retuned, sections, 2), NULL);

	for (int i = 0; i < GOLDEN_DSP_N; i++) {
		if (i == GOLDEN_DSP_N / 2) {
			zassert_true(dsp_cascade_retune(&retuned, sections, 2),
				     "retune refused");
		}

		const float a = dsp_cascade_apply(&steady, 0, golden_dsp_input[i]);
		const float b = dsp_cascade_apply(&retuned, 0, golden_dsp_input[i]);

		zassert_equal(a, b, "retune disturbed sample %d: %f vs %f",
			      i, (double)a, (double)b);
	}

	/* A different number of sections is a different filter: refused. */
	zassert_false(dsp_cascade_retune(&retuned, sections, 1),
		      "retune accepted a different number of sections");
	zassert_equal(retuned.count, 2, "a refused retune changed the cascade");
}

ZTEST(dsp, test_settle_samples_are_sane)
{
	dsp_cascade_t c;

	dsp_cascade_init(&c, 1);
	zassert_equal(dsp_cascade_settle_samples(&c), 0,
		      "an empty cascade has nothing to settle");

	/*
	 * A Q 30 notch at 50 Hz and 1 kSPS rings down to 1 % in about
	 * 4.6 Q / (pi f0) seconds: some 880 samples.
	 */
	zassert_true(dsp_cascade_set(&c, &golden_notch50, 1), NULL);

	const uint32_t n = dsp_cascade_settle_samples(&c);

	zassert_true(n > 800u && n < 960u, "notch settles in %u samples", n);
}

ZTEST(dsp, test_group_delay_is_sane)
{
	/*
	 * A causal filter cannot have negative group delay in its passband,
	 * and the value must be finite - it is what timestamps get corrected
	 * by.
	 */
	const float d = dsp_section_group_delay(&golden_lp40, GOLDEN_DSP_FS,
						10.0f);

	zassert_true(isfinite(d), "group delay is not finite");
	zassert_true(d > 0.0f, "passband group delay %.4f is negative",
		     (double)d);
	zassert_true(d < 100.0f, "group delay %.4f implausibly large",
		     (double)d);
}

ZTEST(dsp, test_cascade_rejects_too_many_sections)
{
	dsp_cascade_t c;
	dsp_section_t sections[DSP_MAX_SECTIONS + 1];

	dsp_cascade_init(&c, 1);

	/* Every section valid, so only the count can be what is refused. */
	for (int i = 0; i < DSP_MAX_SECTIONS + 1; i++) {
		sections[i] = golden_notch50;
	}

	zassert_false(dsp_cascade_set(&c, sections, DSP_MAX_SECTIONS + 1),
		      "oversized cascade accepted");
	zassert_true(dsp_cascade_set(&c, sections, DSP_MAX_SECTIONS),
		     "a full cascade refused");
}

ZTEST_SUITE(dsp, NULL, NULL, NULL, NULL, NULL);

/* ------------------------------------------------------------- ringbuf */

#define RING_CAP 8

struct test_rec {
	uint32_t seq;
	uint8_t  data[12];
};

static struct test_rec ring_storage[RING_CAP];

static void fill(struct test_rec *r, uint32_t seq)
{
	r->seq = seq;
	for (size_t i = 0; i < sizeof(r->data); i++) {
		r->data[i] = (uint8_t)(seq + i);
	}
}

ZTEST(ringbuf, test_init_validates_arguments)
{
	spsc_ring_t r;

	zassert_false(spsc_init(&r, ring_storage, sizeof(struct test_rec), 7),
		      "non-power-of-two capacity accepted");
	zassert_false(spsc_init(&r, ring_storage, sizeof(struct test_rec), 0),
		      "zero capacity accepted");
	zassert_false(spsc_init(&r, NULL, sizeof(struct test_rec), RING_CAP),
		      "null storage accepted");
	zassert_false(spsc_init(&r, ring_storage, 0, RING_CAP),
		      "zero element size accepted");
	zassert_true(spsc_init(&r, ring_storage, sizeof(struct test_rec), RING_CAP),
		     "valid init rejected");
}

ZTEST(ringbuf, test_push_pop_preserves_order_and_content)
{
	spsc_ring_t r;
	struct test_rec in, out;

	zassert_true(spsc_init(&r, ring_storage, sizeof(in), RING_CAP), NULL);
	zassert_true(spsc_is_empty(&r), "fresh ring not empty");

	for (uint32_t i = 0; i < RING_CAP; i++) {
		fill(&in, i);
		zassert_true(spsc_push(&r, &in), "push %u failed", i);
	}

	zassert_equal(spsc_count(&r), RING_CAP, "count wrong");
	zassert_true(spsc_is_full(&r), "ring should be full");

	for (uint32_t i = 0; i < RING_CAP; i++) {
		zassert_true(spsc_pop(&r, &out), "pop %u failed", i);
		zassert_equal(out.seq, i, "FIFO order broken at %u", i);

		fill(&in, i);
		zassert_mem_equal(out.data, in.data, sizeof(in.data),
				  "payload corrupted at %u", i);
	}

	zassert_true(spsc_is_empty(&r), "ring not empty after draining");
	zassert_false(spsc_pop(&r, &out), "pop on empty succeeded");
}

ZTEST(ringbuf, test_overflow_drops_newest_and_counts)
{
	spsc_ring_t r;
	struct test_rec in, out;

	zassert_true(spsc_init(&r, ring_storage, sizeof(in), RING_CAP), NULL);

	for (uint32_t i = 0; i < RING_CAP; i++) {
		fill(&in, i);
		zassert_true(spsc_push(&r, &in), NULL);
	}

	/* Full: further pushes must fail rather than overwrite queued data. */
	for (uint32_t i = 0; i < 5; i++) {
		fill(&in, 999);
		zassert_false(spsc_push(&r, &in), "push on full succeeded");
	}

	zassert_equal(spsc_dropped(&r), 5, "drop count wrong: %u",
		      spsc_dropped(&r));

	/* The originally queued records must be intact. */
	for (uint32_t i = 0; i < RING_CAP; i++) {
		zassert_true(spsc_pop(&r, &out), NULL);
		zassert_equal(out.seq, i, "queued data was overwritten at %u", i);
	}

	spsc_reset_dropped(&r);
	zassert_equal(spsc_dropped(&r), 0, "drop counter did not reset");
}

ZTEST(ringbuf, test_indices_wrap_correctly)
{
	spsc_ring_t r;
	struct test_rec in, out;

	zassert_true(spsc_init(&r, ring_storage, sizeof(in), RING_CAP), NULL);

	/* Far more traffic than capacity, to exercise index wrapping. */
	for (uint32_t i = 0; i < RING_CAP * 50; i++) {
		fill(&in, i);
		zassert_true(spsc_push(&r, &in), "push %u failed", i);
		zassert_true(spsc_pop(&r, &out), "pop %u failed", i);
		zassert_equal(out.seq, i, "wrap corrupted sequence at %u", i);
	}

	zassert_equal(spsc_dropped(&r), 0, "unexpected drops during wrap");
}

ZTEST(ringbuf, test_peek_does_not_consume)
{
	spsc_ring_t r;
	struct test_rec in, out;

	zassert_true(spsc_init(&r, ring_storage, sizeof(in), RING_CAP), NULL);

	zassert_is_null(spsc_peek(&r), "peek on empty returned a record");

	fill(&in, 42);
	zassert_true(spsc_push(&r, &in), NULL);

	const struct test_rec *p = spsc_peek(&r);
	zassert_not_null(p, "peek returned NULL on non-empty ring");
	zassert_equal(p->seq, 42, "peek returned wrong record");
	zassert_equal(spsc_count(&r), 1, "peek consumed the record");

	/* Peeking again must return the same record. */
	p = spsc_peek(&r);
	zassert_equal(p->seq, 42, "second peek differed");

	spsc_release(&r);
	zassert_true(spsc_is_empty(&r), "release did not consume");

	/* Releasing an empty ring must not corrupt the indices. */
	spsc_release(&r);
	zassert_true(spsc_is_empty(&r), "release on empty broke the ring");
	fill(&in, 7);
	zassert_true(spsc_push(&r, &in), "ring unusable after stray release");
	zassert_true(spsc_pop(&r, &out), NULL);
	zassert_equal(out.seq, 7, "ring corrupted by stray release");
}

ZTEST_SUITE(ringbuf, NULL, NULL, NULL, NULL, NULL);
