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
 *   frame -> 24-bit decode -> integer DC removal -> microvolts -> biquads
 *
 * DC comes out in the integer domain, before the float conversion. Full
 * scale at gain 24 is +/-187.5 mV against a 22.35 nV LSB, a ratio of 8.4e6,
 * and float32 carries about 1.7e7 of mantissa - so converting first would
 * leave barely a bit for the microvolt signal riding on the offset.
 */
#ifndef SWIFTEEG_CHAIN_H
#define SWIFTEEG_CHAIN_H

#include <stdbool.h>
#include <stdint.h>

#include "dsp/dsp.h"
#include "frame.h"

typedef struct {
	dsp_dc_t      dc[FRAME_CHANNELS];
	dsp_cascade_t cascade;
	float         lsb_uv;
	uint8_t       channels;
} chain_t;

/*
 * Build the chain: DC corner from `dc_shift`, one mains notch section, and
 * the microvolt scale from the reference and gain in use.
 * Returns 0, or a negative errno if a filter could not be designed.
 */
int chain_init(chain_t *c, float fs_hz, uint8_t dc_shift, float notch_hz,
	       float notch_q, float vref_volts, uint8_t gain);

/*
 * Run one frame through.
 *
 * Returns false, having touched nothing, if the frame's status marker is
 * wrong. That frame is not a sample, and letting it into the filters would
 * corrupt the DC estimate and the biquad state for everything after it.
 *
 * `raw_out` and `uv_out` each take FRAME_CHANNELS entries; either may be
 * NULL.
 */
bool chain_process(chain_t *c, const uint8_t *frame, int32_t *raw_out,
		   float *uv_out);

/*
 * Redesign the notch without disturbing the DC estimate.
 *
 * `notch_hz` of 0 removes the section entirely, leaving the cascade as a
 * pass-through. The biquad state is cleared because it belongs to the old
 * filter; the DC state is kept, since the offset has not changed.
 */
int chain_set_notch(chain_t *c, float fs_hz, float notch_hz, float notch_q);

#endif /* SWIFTEEG_CHAIN_H */
