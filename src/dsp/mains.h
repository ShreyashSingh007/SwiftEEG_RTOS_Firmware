/*
 * Mains frequency tracking, on live samples.
 *
 * Grid frequency is never where the nameplate says - measured at 49.6 Hz
 * here - and it wanders by tenths of a hertz. A notch narrow enough to spare
 * the EEG either side has to be aimed at the real frequency, so it is
 * measured, continuously, from the signal itself.
 *
 * A frequency-locked loop, fed the mean of the channels' unfiltered
 * microvolts:
 *
 *   - mix the signal down by the current estimate with a rotating phasor,
 *     so mains near it lands near 0 Hz;
 *   - low-pass both halves, two one-pole stages at 1.5 Hz, and average them
 *     over each tenth of a second into one value. The average is not
 *     optional: at ten values a second, anything 10, 20 ... 50 Hz from the
 *     mixer folds onto the mains - electrode drift and alpha among them -
 *     and a tenth-second average has a null at each of those. Without it,
 *     a signal with no mains in it at all was found to be 50.00 Hz;
 *   - over 4 s, measure how far that slow tone turns over a lag: 1 s for
 *     the fine answer, and 0.2 s to catch a large error, which the 1 s lag
 *     alone would alias beyond +/-0.5 Hz;
 *   - believe it only when the tone is coherent across the 1 s lag. Noise
 *     decorrelates over that long; a real tone does not;
 *   - re-aim the mixer at each answer, and report an estimate only when two
 *     answers in a row agree.
 *
 * In simulation - random-walk EEG, alpha, drift and mains wandering, stepping
 * or 0.8 Hz off nominal, at 250 and 1000 SPS - the first estimate came about
 * 10 s after a start, and the RMS error after it was 0.3-4 mHz with 40 uV of
 * mains, 3-5 mHz at 5 uV, 7-14 mHz at 2 uV and 9-27 mHz at 1 uV. A Q 12
 * notch 50 mHz off still takes 32 dB off. With no mains at all there was no
 * estimate in any of twelve 90 s runs.
 *
 * The per-sample path is one phasor turn and four one-pole updates in
 * float32, about twenty operations. What happens ten times a second or less
 * is done in double, where a host can reproduce it exactly. Pure maths,
 * mirrored operation for operation by tools/dsp_ref.py.
 */
#ifndef SWIFTEEG_DSP_MAINS_H
#define SWIFTEEG_DSP_MAINS_H

#include <stdbool.h>
#include <stdint.h>

/* Baseband values held for the lagged products: the 1 s lag. */
#define DSP_MAINS_RING 10

typedef struct {
	float    fs_hz;
	float    nominal_hz;
	float    k;          /* one-pole coefficient of the 1.5 Hz low-pass */
	uint16_t decim;      /* input samples per baseband value */

	/* The mixer: aimed at mix_hz, turning by (cw, sw) a sample. */
	float    mix_hz;
	float    cw, sw;
	float    c, s;
	uint16_t turns;      /* since the phasor was last renormalised */

	/* The low-pass, in phase and in quadrature, and its running sums. */
	float    i1, i2, q1, q2;
	float    sum_i, sum_q;
	uint16_t phase;      /* input samples into the current baseband value */

	/* Recent baseband values; once full, the oldest is at `head`. */
	float    ring_i[DSP_MAINS_RING];
	float    ring_q[DSP_MAINS_RING];
	uint8_t  ring_len;
	uint8_t  head;

	/* Sums over the current 4 s. */
	float    fine_i, fine_q;
	float    coarse_i, coarse_q;
	float    power;
	uint8_t  count;

	float    candidate_hz; /* the last answer, waiting for one that agrees */
	float    estimate_hz;  /* the last agreed answer; 0 until there is one */
} dsp_mains_t;

/*
 * Start tracking around `nominal_hz` at `fs_hz`, which must be a whole
 * multiple of 10 from 250 up. The mixer starts at `start_hz` - a previous
 * estimate, which makes a restart quick - or at the nominal frequency when
 * that is 0 or more than 3 Hz away. Returns false, changing nothing, for a
 * rate or a frequency it cannot track.
 */
bool dsp_mains_init(dsp_mains_t *t, float fs_hz, float nominal_hz,
		    float start_hz);

/*
 * One sample, in microvolts. Returns true when two answers in a row have
 * agreed; the frequency is then in `estimate_hz`, and may equal the last.
 */
bool dsp_mains_push(dsp_mains_t *t, float x);

#endif /* SWIFTEEG_DSP_MAINS_H */
