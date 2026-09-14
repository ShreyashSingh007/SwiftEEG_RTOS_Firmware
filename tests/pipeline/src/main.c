/*
 * Whole-chain golden vectors.
 *
 * The DSP suite checks each primitive on its own. This checks how they are
 * composed - decode, integer DC removal, per-channel scaling, the pre
 * sections, the common average, the post sections - because a chain built
 * from correct parts is still wrong if a stage runs in the wrong order or
 * the scaling lands on the wrong side of the DC removal.
 *
 * It calls src/pipeline/chain.c, the same code the firmware runs. A golden
 * test against a reimplementation written for the test would only prove that
 * two reimplementations agree.
 *
 * Reference and vectors: tools/pipeline_ref.py.
 */

#include <zephyr/ztest.h>

#include "golden_pipeline.h"
#include "pipeline/chain.h"

#define FRAME_AT(i) (&golden_pipe_frames[(size_t)(i) * FRAME_BYTES])

/*
 * Tolerance on the microvolt output.
 *
 * Both sides are float32 doing the same operations in the same order, so
 * agreement should be near exact. The allowance is for the last bit or two
 * of a mantissa, scaled by the signal: channel 1 runs to a couple of
 * millivolts, where an absolute-only bound would be meaninglessly loose for
 * the microvolt channels.
 */
#define ABS_TOL_UV 1.0e-3f
#define REL_TOL    1.0e-5f

static bool close_enough(float got, float want)
{
	float diff = got - want;

	if (diff < 0.0f) {
		diff = -diff;
	}

	float mag = want < 0.0f ? -want : want;

	return diff <= (ABS_TOL_UV + REL_TOL * mag);
}

/* Compare one output row, tracking the worst channel seen so far. */
static void check_row(const float *got, const float *want, int frame,
		      float *worst)
{
	for (int ch = 0; ch < GOLDEN_PIPE_CHANNELS; ch++) {
		float diff = got[ch] - want[ch];

		if (diff < 0.0f) {
			diff = -diff;
		}
		if (diff > *worst) {
			*worst = diff;
		}

		zassert_true(close_enough(got[ch], want[ch]),
			     "frame %d ch %d: got %f uV, want %f uV",
			     frame, ch + 1, (double)got[ch], (double)want[ch]);
	}
}

static chain_t chain;

/* A: the device's default chain. */
static int build(chain_t *c)
{
	return chain_init(c, GOLDEN_A_FS, GOLDEN_A_DC_SHIFT, GOLDEN_A_NOTCH_HZ,
			  GOLDEN_A_NOTCH_Q, 4.5f, 24);
}

/* B: the host chain, loaded the way the firmware loads it. */
static void build_full(chain_t *c)
{
	zassert_ok(chain_init(c, GOLDEN_B_FS, GOLDEN_B_DC_SHIFT, 0.0f, 12.0f,
			      4.5f, 24));

	for (uint8_t ch = 0; ch < GOLDEN_PIPE_CHANNELS; ch++) {
		zassert_ok(chain_set_gain(c, ch, golden_b_gains[ch]));
	}

	zassert_ok(chain_set_stage(c, CHAIN_STAGE_PRE, golden_b_pre,
				   GOLDEN_B_PRE_COUNT, false, NULL));
	zassert_ok(chain_set_stage(c, CHAIN_STAGE_POST, golden_b_post,
				   GOLDEN_B_POST_COUNT, false, NULL));
	chain_set_car(c, true, GOLDEN_B_CAR_MASK);
}

ZTEST(pipeline, test_lsb_matches_reference)
{
	zassert_ok(build(&chain));
	zassert_within(chain.lsb_uv[0], GOLDEN_PIPE_LSB_UV, 1e-9f,
		       "LSB is the scale everything else rests on");
}

ZTEST(pipeline, test_frame_decode_handles_the_full_range)
{
	/* Channel 7 is full-scale positive, channel 8 full-scale negative. */
	const uint8_t *f = FRAME_AT(0);

	zassert_equal(frame_channel(f, 6), 8388607, "0x7FFFFF must decode to +2^23-1");
	zassert_equal(frame_channel(f, 7), -8388608, "0x800000 must decode to -2^23");
}

ZTEST(pipeline, test_status_marker_is_checked)
{
	uint8_t f[FRAME_BYTES];

	memcpy(f, FRAME_AT(0), sizeof(f));
	zassert_true(frame_is_valid(f));

	/* Only the top nibble is fixed; the rest carries lead-off and GPIO. */
	f[0] = 0xCFu;
	zassert_true(frame_is_valid(f), "low status bits are data, not marker");

	f[0] = 0x00u;
	zassert_false(frame_is_valid(f), "all-zero status must be rejected");

	f[0] = 0xFFu;
	zassert_false(frame_is_valid(f), "all-ones status must be rejected");
}

ZTEST(pipeline, test_default_chain_matches_reference)
{
	zassert_ok(build(&chain));

	float uv[FRAME_CHANNELS];
	float worst = 0.0f;

	for (int i = 0; i < GOLDEN_PIPE_FRAMES; i++) {
		zassert_true(chain_process(&chain, FRAME_AT(i), NULL, uv),
			     "frame %d was rejected", i);
		check_row(uv, golden_a_out[i], i, &worst);
	}

	TC_PRINT("default chain: worst deviation %e uV\n", (double)worst);
}

ZTEST(pipeline, test_full_chain_matches_reference)
{
	build_full(&chain);

	float uv[FRAME_CHANNELS];
	float worst = 0.0f;

	for (int i = 0; i < GOLDEN_PIPE_FRAMES; i++) {
		if (i == GOLDEN_B_RETUNE_AT) {
			bool kept = false;

			zassert_ok(chain_set_stage(&chain, CHAIN_STAGE_PRE,
						   golden_b_pre_retuned,
						   GOLDEN_B_PRE_COUNT, true,
						   &kept));
			zassert_true(kept, "a same-size retune restarted");
		}
		if (i == GOLDEN_B_CAR_AT) {
			chain_set_car(&chain, true, GOLDEN_B_CAR_MASK_LATER);
		}
		if (i == GOLDEN_B_RESET_AT) {
			chain_reset(&chain);
		}

		zassert_true(chain_process(&chain, FRAME_AT(i), NULL, uv),
			     "frame %d was rejected", i);
		check_row(uv, golden_b_out[i], i, &worst);
	}

	TC_PRINT("full chain: worst deviation %e uV\n", (double)worst);
}

ZTEST(pipeline, test_dc_offset_is_removed)
{
	zassert_ok(build(&chain));

	float uv[FRAME_CHANNELS];

	for (int i = 0; i < GOLDEN_PIPE_FRAMES; i++) {
		(void)chain_process(&chain, FRAME_AT(i), NULL, uv);
	}

	/*
	 * Channel 1 carries the cal square wave on a 400000-count offset,
	 * about 8.9 mV. What survives should be the square wave, not the
	 * offset it was riding on.
	 */
	const float last = uv[0] < 0.0f ? -uv[0] : uv[0];

	zassert_true(last < 5000.0f,
		     "DC offset reached the output: %f uV", (double)last);
}

ZTEST(pipeline, test_mains_is_notched_out)
{
	zassert_ok(build(&chain));

	float uv[FRAME_CHANNELS];
	float peak = 0.0f;

	for (int i = 0; i < GOLDEN_PIPE_FRAMES; i++) {
		(void)chain_process(&chain, FRAME_AT(i), NULL, uv);

		/* Skip the settling transient: the notch has to ring down. */
		if (i < 400) {
			continue;
		}

		const float mag = uv[2] < 0.0f ? -uv[2] : uv[2];

		if (mag > peak) {
			peak = mag;
		}
	}

	/* Channel 3 is 50 uV of pure mains and nothing else. */
	zassert_true(peak < 8.0f,
		     "notch left %f uV of a 50 uV input", (double)peak);
}

ZTEST(pipeline, test_alpha_survives_the_notch)
{
	zassert_ok(build(&chain));

	float uv[FRAME_CHANNELS];
	float peak = 0.0f;

	for (int i = 0; i < GOLDEN_PIPE_FRAMES; i++) {
		(void)chain_process(&chain, FRAME_AT(i), NULL, uv);

		if (i < 400) {
			continue;
		}

		const float mag = uv[3] < 0.0f ? -uv[3] : uv[3];

		if (mag > peak) {
			peak = mag;
		}
	}

	/*
	 * Channel 4 is 10 uV at 10 Hz. A notch wide enough to touch it would
	 * be taking out EEG along with the mains.
	 */
	zassert_true(peak > 8.0f && peak < 12.0f,
		     "10 Hz came out at %f uV, expected near 10", (double)peak);
}

ZTEST(pipeline, test_bad_frame_leaves_no_trace)
{
	float uv_ref[FRAME_CHANNELS];
	float uv_bad[FRAME_CHANNELS];

	/* Reference run: 64 clean frames. */
	zassert_ok(build(&chain));
	for (int i = 0; i < 64; i++) {
		(void)chain_process(&chain, FRAME_AT(i), NULL, uv_ref);
	}

	/* Same run, with one corrupted frame offered partway through. */
	chain_t spoiled;

	zassert_ok(build(&spoiled));

	uint8_t bad[FRAME_BYTES];

	memcpy(bad, FRAME_AT(32), sizeof(bad));
	bad[0] = 0x00u;

	for (int i = 0; i < 64; i++) {
		if (i == 32) {
			zassert_false(chain_process(&spoiled, bad, NULL, uv_bad),
				      "a bad frame must be rejected");
		}
		(void)chain_process(&spoiled, FRAME_AT(i), NULL, uv_bad);
	}

	/*
	 * Rejecting it must be free: every valid frame still went through in
	 * the same order, so the result has to be identical bit for bit.
	 */
	for (int ch = 0; ch < FRAME_CHANNELS; ch++) {
		zassert_equal(uv_bad[ch], uv_ref[ch],
			      "ch %d diverged after a rejected frame: %f vs %f",
			      ch + 1, (double)uv_bad[ch], (double)uv_ref[ch]);
	}
}

ZTEST(pipeline, test_invalid_stage_changes_nothing)
{
	build_full(&chain);

	dsp_section_t bad = golden_b_post[0];

	bad.k = 0.0f;

	zassert_equal(chain_set_stage(&chain, CHAIN_STAGE_POST, &bad, 1, false,
				      NULL), -EINVAL, "invalid section accepted");
	zassert_equal(chain_set_stage(&chain, 2u, golden_b_post,
				      GOLDEN_B_POST_COUNT, false, NULL),
		      -EINVAL, "stage 2 accepted");
	zassert_equal(chain.post.count, GOLDEN_B_POST_COUNT,
		      "a refused change altered the stage");
}

ZTEST(pipeline, test_average_needs_two_channels)
{
	chain_t plain, one;
	float a[FRAME_CHANNELS], b[FRAME_CHANNELS];

	zassert_ok(build(&plain));
	zassert_ok(build(&one));

	/* An average of one channel is not an average of anything. */
	chain_set_car(&one, true, 0x08u);

	for (int i = 0; i < 64; i++) {
		(void)chain_process(&plain, FRAME_AT(i), NULL, a);
		(void)chain_process(&one, FRAME_AT(i), NULL, b);

		for (int ch = 0; ch < FRAME_CHANNELS; ch++) {
			zassert_equal(a[ch], b[ch],
				      "frame %d ch %d changed by a one-channel average",
				      i, ch + 1);
		}
	}
}

ZTEST(pipeline, test_gain_rescales_one_channel)
{
	zassert_ok(build(&chain));

	const float lsb24 = chain.lsb_uv[0];

	zassert_ok(chain_set_gain(&chain, 1, 12));
	zassert_within(chain.lsb_uv[1], 2.0f * lsb24, 1e-9f,
		       "gain 12 should double the LSB");
	zassert_equal(chain.lsb_uv[0], lsb24, "another channel's scale moved");
	zassert_equal(chain_set_gain(&chain, 1, 0), -EINVAL, "gain 0 accepted");
}

ZTEST(pipeline, test_reset_is_a_fresh_start)
{
	/*
	 * A reset must be exactly a new chain started at that frame. That is
	 * what lets a host reproduce the device's output from a known sample.
	 */
	chain_t fresh;
	float a[FRAME_CHANNELS], b[FRAME_CHANNELS];

	build_full(&chain);
	for (int i = 0; i < 100; i++) {
		(void)chain_process(&chain, FRAME_AT(i), NULL, a);
	}
	chain_reset(&chain);

	build_full(&fresh);

	for (int i = 100; i < 200; i++) {
		(void)chain_process(&chain, FRAME_AT(i), NULL, a);
		(void)chain_process(&fresh, FRAME_AT(i), NULL, b);

		for (int ch = 0; ch < FRAME_CHANNELS; ch++) {
			zassert_equal(a[ch], b[ch],
				      "frame %d ch %d: reset %f vs fresh %f",
				      i, ch + 1, (double)a[ch], (double)b[ch]);
		}
	}
}

ZTEST_SUITE(pipeline, NULL, NULL, NULL, NULL, NULL);
