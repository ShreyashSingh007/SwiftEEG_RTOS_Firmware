/*
 * Tests for the pure modules: DSP primitives and the SPSC ring buffer.
 *
 * DSP results are checked against golden vectors from tools/dsp_ref.py,
 * which is the authoritative reference. Integer DC removal must match bit
 * for bit; float paths allow a small tolerance because the compiler may
 * contract multiply-adds into FMA, which rounds differently from two
 * separate operations.
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

ZTEST(dsp, test_notch_coeffs_match_reference)
{
	dsp_biquad_coeffs_t c;

	zassert_true(dsp_design_notch(&c, GOLDEN_DSP_FS, 50.0f, 30.0f),
		     "notch design failed");

	const float got[5] = { c.b0, c.b1, c.b2, c.a1, c.a2 };
	for (int i = 0; i < 5; i++) {
		zassert_within(got[i], golden_notch50_coeffs[i], 1e-6f,
			       "notch coeff %d: %.9f vs %.9f", i,
			       (double)got[i], (double)golden_notch50_coeffs[i]);
	}
}

ZTEST(dsp, test_lowpass_coeffs_match_reference)
{
	dsp_biquad_coeffs_t c;
	const float q = 1.0f / sqrtf(2.0f);

	zassert_true(dsp_design_lowpass(&c, GOLDEN_DSP_FS, 40.0f, q),
		     "lowpass design failed");

	const float got[5] = { c.b0, c.b1, c.b2, c.a1, c.a2 };
	for (int i = 0; i < 5; i++) {
		zassert_within(got[i], golden_lp40_coeffs[i], 1e-6f,
			       "lp coeff %d: %.9f vs %.9f", i,
			       (double)got[i], (double)golden_lp40_coeffs[i]);
	}
}

ZTEST(dsp, test_design_rejects_bad_args)
{
	dsp_biquad_coeffs_t c;

	/* At or above Nyquist is not designable. */
	zassert_false(dsp_design_notch(&c, 1000.0f, 500.0f, 30.0f),
		      "f0 at Nyquist accepted");
	zassert_false(dsp_design_notch(&c, 1000.0f, 600.0f, 30.0f),
		      "f0 above Nyquist accepted");
	zassert_false(dsp_design_lowpass(&c, 1000.0f, 40.0f, 0.0f),
		      "zero Q accepted");
	zassert_false(dsp_design_highpass(&c, 0.0f, 1.0f, 0.7f),
		      "zero fs accepted");
	zassert_false(dsp_design_notch(NULL, 1000.0f, 50.0f, 30.0f),
		      "null output accepted");
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

ZTEST(dsp, test_single_biquad_matches_reference)
{
	dsp_cascade_t c;
	dsp_biquad_coeffs_t notch;

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_design_notch(&notch, GOLDEN_DSP_FS, 50.0f, 30.0f), NULL);
	zassert_true(dsp_cascade_set(&c, &notch, 1), NULL);

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
	dsp_biquad_coeffs_t sections[2];
	const float q = 1.0f / sqrtf(2.0f);

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_design_notch(&sections[0], GOLDEN_DSP_FS, 50.0f, 30.0f), NULL);
	zassert_true(dsp_design_lowpass(&sections[1], GOLDEN_DSP_FS, 40.0f, q), NULL);
	zassert_true(dsp_cascade_set(&c, sections, 2), NULL);

	for (int i = 0; i < GOLDEN_DSP_N; i++) {
		float y = dsp_cascade_apply(&c, 0, golden_dsp_input[i]);

		zassert_within(y, golden_dsp_cascaded[i], FLOAT_TOL,
			       "sample %d: got %.6f, reference %.6f",
			       i, (double)y, (double)golden_dsp_cascaded[i]);
	}
}

ZTEST(dsp, test_channels_are_independent)
{
	dsp_cascade_t c;
	dsp_biquad_coeffs_t notch;

	dsp_cascade_init(&c, DSP_MAX_CHANNELS);
	zassert_true(dsp_design_notch(&notch, GOLDEN_DSP_FS, 50.0f, 30.0f), NULL);
	zassert_true(dsp_cascade_set(&c, &notch, 1), NULL);

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
	dsp_biquad_coeffs_t notch;
	float first_run[16];

	dsp_cascade_init(&c, 1);
	zassert_true(dsp_design_notch(&notch, GOLDEN_DSP_FS, 50.0f, 30.0f), NULL);
	zassert_true(dsp_cascade_set(&c, &notch, 1), NULL);

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

ZTEST(dsp, test_group_delay_is_sane)
{
	dsp_biquad_coeffs_t lp;
	const float q = 1.0f / sqrtf(2.0f);

	zassert_true(dsp_design_lowpass(&lp, GOLDEN_DSP_FS, 40.0f, q), NULL);

	/*
	 * A causal filter cannot have negative group delay in its passband,
	 * and the value must be finite - it is reported to the host so
	 * timestamps can be corrected.
	 */
	const float d = dsp_biquad_group_delay(&lp, GOLDEN_DSP_FS, 10.0f);

	zassert_true(isfinite(d), "group delay is not finite");
	zassert_true(d > 0.0f, "passband group delay %.4f is negative",
		     (double)d);
	zassert_true(d < 100.0f, "group delay %.4f implausibly large",
		     (double)d);
}

ZTEST(dsp, test_cascade_rejects_too_many_sections)
{
	dsp_cascade_t c;
	dsp_biquad_coeffs_t sections[DSP_MAX_SECTIONS + 1];

	dsp_cascade_init(&c, 1);
	memset(sections, 0, sizeof(sections));

	zassert_false(dsp_cascade_set(&c, sections, DSP_MAX_SECTIONS + 1),
		      "oversized cascade accepted");
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
