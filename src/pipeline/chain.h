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
 *         -> pre sections -> mains notch -> common average -> post sections
 *
 * DC comes out in the integer domain, before the float conversion. Full
 * scale at gain 24 is +/-187.5 mV against a 22.35 nV LSB, a ratio of 8.4e6,
 * and float32 carries about 1.7e7 of mantissa - so converting first would
 * leave barely a bit for the microvolt signal riding on the offset.
 *
 * The pre and post stages sit either side of the common average so the host
 * application's chain transfers as it is: its high-pass before the average,
 * so no electrode's drift reaches the others through it, and its low-pass
 * after. The mains notch between them is the device's own, because the
 * device measures the mains frequency it has to follow; it sits where the
 * host chain has its notch. By default the notch is all that is loaded.
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
	dsp_cascade_t notch;
	dsp_cascade_t post;
	float         lsb_uv[FRAME_CHANNELS]; /* per channel: gains can differ */
	float         vref_volts;
	float         mains_in;               /* see chain_process() */
	uint8_t       mains_mask;             /* channels mains_in is taken over */
	uint8_t       car_mask;               /* channels that make the average */
	bool          car;
	uint8_t       channels;
} chain_t;

/*
 * Build the chain: DC corner from `dc_shift`, every stage empty, no common
 * average, every channel scaled for `gain` and counted in mains_in.
 * Returns 0, or -EINVAL for a gain of 0.
 */
int chain_init(chain_t *c, uint8_t dc_shift, float vref_volts, uint8_t gain);

/*
 * Run one frame through.
 *
 * Returns false, having touched nothing, if the frame's status marker is
 * wrong. That frame is not a sample, and letting it into the filters would
 * corrupt the DC estimate and the filter state for everything after it.
 *
 * Also leaves `mains_in`: the mean over the channels in `mains_mask` of the
 * microvolts after DC removal and before any filter - what the mains tracker
 * needs, since every stage after that point may already have taken the
 * mains out. 0 when the mask is empty.
 *
 * `raw_out` and `uv_out` each take FRAME_CHANNELS entries; either may be
 * NULL.
 */
bool chain_process(chain_t *c, const uint8_t *frame, int32_t *raw_out,
		   float *uv_out);

/*
 * Aim the mains notch at `hz`, `q` wide, with a second section at twice the
 * frequency when `harmonic` and the rate leaves room for it - under 95 % of
 * Nyquist. 0 Hz removes it. With `keep_state` and as many sections as
 * before, the notch is retuned in place, which is how it follows the mains
 * without a transient; otherwise it starts again, primed on its next sample.
 * `kept` (may be NULL) says which happened. Returns -EINVAL, changing
 * nothing, for a notch that cannot be designed at this rate.
 */
int chain_set_notch(chain_t *c, float fs_hz, float hz, float q, bool harmonic,
		    bool keep_state, bool *kept);

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

/* Which channels mains_in is taken over. */
void chain_set_mains_mask(chain_t *c, uint8_t mask);

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
