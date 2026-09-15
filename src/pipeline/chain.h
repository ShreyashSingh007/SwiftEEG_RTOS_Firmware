/*
 * The per-sample signal chain, with no dependency on Zephyr or hardware.
 *
 * Split out from pipeline.c so the golden-vector test exercises the code
 * that actually runs, rather than a copy of it written for the test. A
 * golden test against a reimplementation proves only that the two
 * reimplementations agree.
 *
 * Order matters and is the point of the whole module:
 *
 *   frame -> 24-bit decode -> integer DC removal -> microvolts
 *         -> pre sections -> common average -> post sections
 *
 * DC comes out in the integer domain, before the float conversion. Full
 * scale at gain 24 is +/-187.5 mV against a 22.35 nV LSB, a ratio of 8.4e6,
 * and float32 carries about 1.7e7 of mantissa - so converting first would
 * leave barely a bit for the microvolt signal riding on the offset.
 *
 * The two stages sit either side of the common average so the host
 * application's chain transfers as it is: its high-pass and notch before the
 * average, so no electrode's drift reaches the others through it, and its
 * low-pass after. By default the pre stage holds the mains notch and the
 * post stage is empty.
 */
#ifndef SWIFTEEG_CHAIN_H
#define SWIFTEEG_CHAIN_H

#include <stdbool.h>
#include <stdint.h>

#include "dsp/dsp.h"
#include "frame.h"

#define CHAIN_STAGE_PRE  0u /* before the common average */
#define CHAIN_STAGE_POST 1u /* after it */

typedef struct {
	dsp_dc_t      dc[FRAME_CHANNELS];
	dsp_cascade_t pre;
	dsp_cascade_t post;
	float         lsb_uv[FRAME_CHANNELS]; /* per channel: gains can differ */
	float         vref_volts;
	uint8_t       car_mask;               /* channels that make the average */
	bool          car;
	uint8_t       channels;
} chain_t;

/*
 * Build the chain: DC corner from `dc_shift`, the mains notch as the pre
 * stage (none if `notch_hz` is 0), an empty post stage, no common average,
 * and every channel scaled for `gain`.
 * Returns 0, or a negative errno if a filter could not be designed.
 */
int chain_init(chain_t *c, float fs_hz, uint8_t dc_shift, float notch_hz,
	       float notch_q, float vref_volts, uint8_t gain);

/*
 * Run one frame through.
 *
 * Returns false, having touched nothing, if the frame's status marker is
 * wrong. That frame is not a sample, and letting it into the filters would
 * corrupt the DC estimate and the filter state for everything after it.
 *
 * `raw_out` and `uv_out` each take FRAME_CHANNELS entries; either may be
 * NULL.
 */
bool chain_process(chain_t *c, const uint8_t *frame, int32_t *raw_out,
		   float *uv_out);

/*
 * Make the pre stage a single mains notch, or a pass-through when
 * `notch_hz` is 0. It starts again, primed on its next sample; the DC state
 * is kept, since the offset has not changed.
 */
int chain_set_notch(chain_t *c, float fs_hz, float notch_hz, float notch_q);

/*
 * Load sections into one stage - CHAIN_STAGE_PRE or CHAIN_STAGE_POST.
 *
 * With `keep_state`, and the same number of sections as the stage holds, the
 * filter is retuned in place; otherwise it starts again, primed on its next
 * sample. `kept` (may be NULL) says which happened. Returns -EINVAL,
 * changing nothing, for a bad stage, too many sections or an invalid one.
 */
int chain_set_stage(chain_t *c, uint8_t stage, const dsp_section_t *sections,
		    uint8_t count, bool keep_state, bool *kept);

/* Common average reference on or off, and which channels make it. */
void chain_set_car(chain_t *c, bool enable, uint8_t mask);

/*
 * Scale one channel for a new gain. A real change re-primes that channel's
 * DC estimate: the same electrode offset is a different number of counts at
 * the new gain, and carrying the old estimate over would put a step the size
 * of the offset into the filters.
 */
int chain_set_gain(chain_t *c, uint8_t ch, uint8_t gain);

/*
 * Start again from the next frame: the DC estimate and every filter
 * re-prime on it. Sections and settings are kept.
 */
void chain_reset(chain_t *c);

/* Samples the filters take to settle after a restart. */
uint32_t chain_settle_samples(const chain_t *c);

#endif /* SWIFTEEG_CHAIN_H */
