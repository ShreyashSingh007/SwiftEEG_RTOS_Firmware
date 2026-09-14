/*
 * SwiftEEG on-chip DSP primitives.
 *
 * Design rule: firmware does only what MUST happen on-chip, and exposes
 * everything else as a host-programmable cascade of second-order sections.
 * No filtering opinion is baked in. See README section 7.
 *
 * Notably absent by design: a fixed high-pass above ~0.1 Hz. Causal high-pass
 * corners above roughly 0.1-0.3 Hz measurably distort slow ERP components
 * such as P300, which is exactly what this device exists to measure well.
 *
 * Pure maths, depending only on the C standard library, so the whole chain is
 * validated against a Python reference off-target rather than by eye.
 */
#ifndef SWIFTEEG_DSP_H
#define SWIFTEEG_DSP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DSP_MAX_CHANNELS 8

/* --- Stage 0: integer DC tracking and removal ---------------------------- */

/*
 * Removes the electrode half-cell offset BEFORE the float conversion.
 *
 * This ordering is the single most important numeric decision in the pipeline.
 * Full scale at gain 24 is +/-187.5 mV and one LSB is 22.35 nV, a ratio of
 * 8.4e6. float32 carries only ~1.7e7 of mantissa, so converting a DC-laden
 * sample to float leaves barely one bit of headroom for the microvolt EEG
 * riding on top. Subtracting DC in the integer domain first gives the AC
 * signal the full mantissa.
 *
 * A first-order leaky integrator is used deliberately: its state can be primed
 * from the first sample so it starts settled, which removes almost all of the
 * startup transient. A higher-order section cannot be primed so cleanly.
 */
typedef struct {
	int64_t acc;      /* Q(shift) accumulator holding the DC estimate */
	uint8_t shift;    /* larger = slower tracking = lower corner */
	bool    primed;
} dsp_dc_t;

/*
 * shift sets the corner: fc ~= fs / (2*pi * 2^shift).
 * At 1 kSPS, shift 14 gives ~0.01 Hz, shift 11 gives ~0.08 Hz.
 */
void dsp_dc_init(dsp_dc_t *dc, uint8_t shift);
int32_t dsp_dc_apply(dsp_dc_t *dc, int32_t sample);

/* --- Stage 1: scaling to microvolts ------------------------------------- */

/*
 * ADS1299 LSB = (2 * VREF) / (gain * 2^24) volts.
 * With the internal 4.5 V reference and gain 24 this is 22.35 nV.
 */
float dsp_lsb_uv(float vref_volts, uint8_t gain);

/* --- Second-order sections ----------------------------------------------- */

/*
 * One second-order section, run as a state-variable filter with trapezoidal
 * integrators - Zavalishin's topology-preserving transform, in Andrew
 * Simper's form - rather than the direct form the cookbook formulas are
 * usually written for.
 *
 * The reason is float32. A high-pass at 0.1 Hz puts its poles almost on
 * z = 1, where a transposed direct form II loses most of its coefficients'
 * precision and amplifies every rounding error in its state by the filter's
 * own gain near DC. Measured against a float64 reference, on 3 mV of drift
 * with EEG on top, a fourth-order 0.1 Hz high-pass in that form was 2 uV RMS
 * out at 250 SPS and 14 uV at 1000 SPS - as large as the EEG. This form
 * keeps its state as integrators of the signal and its coefficients as small
 * numbers, both of which float32 holds well: 0.02 uV RMS at 250 SPS and
 * 0.08 uV at 1000. It is the same filter - responses agree to 1e-7.
 *
 *   g            tan(pi * fc / fs), the prewarped corner
 *   k            1 / Q
 *   m0, m1, m2   how the output mixes the input, the band-pass and the
 *                low-pass: low-pass (0, 0, 1), high-pass (1, -k, -1),
 *                notch (1, -k, 0). Every biquad has such a mix.
 */
typedef struct {
	float g;
	float k;
	float m0, m1, m2;
} dsp_section_t;

/* What the inner loop runs, derived from a section when it is loaded. */
typedef struct {
	float a1, a2, a3;
	float m0, m1, m2;
} dsp_section_run_t;

/* Two integrator states per section, per channel. */
typedef struct {
	float ic1, ic2;
} dsp_section_state_t;

#define DSP_MAX_SECTIONS 8

typedef struct {
	dsp_section_t       sections[DSP_MAX_SECTIONS]; /* as loaded */
	dsp_section_run_t   run[DSP_MAX_SECTIONS];
	dsp_section_state_t state[DSP_MAX_CHANNELS][DSP_MAX_SECTIONS];
	uint8_t count;
	uint8_t channels;
} dsp_cascade_t;

void dsp_cascade_init(dsp_cascade_t *c, uint8_t channels);

/*
 * Load sections and start them from rest. A count of 0, with `sections`
 * allowed to be NULL, leaves a pass-through. Returns false, changing nothing,
 * for too many sections or an invalid one.
 */
bool dsp_cascade_set(dsp_cascade_t *c, const dsp_section_t *sections,
		     uint8_t count);

/*
 * Load sections but keep the state - for a small retune such as a notch
 * following the mains a tenth of a hertz, where a restart would put a
 * transient into the signal every time. The states are integrators of the
 * signal, so they stay meaningful when a corner moves. Needs the same number
 * of sections as are loaded; returns false, changing nothing, otherwise.
 */
bool dsp_cascade_retune(dsp_cascade_t *c, const dsp_section_t *sections,
			uint8_t count);

void dsp_cascade_reset_state(dsp_cascade_t *c);
float dsp_cascade_apply(dsp_cascade_t *c, uint8_t channel, float x);

/*
 * Whether a section can be run. With g and k both positive a section of this
 * form is stable whatever its mix, so that - with every value finite - is the
 * whole check. Sections arrive from a host, and an unstable one would not
 * fail: it would grow until the output was all infinities.
 */
bool dsp_section_is_valid(const dsp_section_t *s);

/*
 * Roughly how many samples a cascade takes to settle after a restart: each
 * section's slowest decay to 1 %, added up. Samples inside that window are
 * flagged, so a transient is never mistaken for signal.
 */
uint32_t dsp_cascade_settle_samples(const dsp_cascade_t *c);

/* --- Section design (a host may also upload sections directly) ---------- */

/*
 * Second-order notch. `q` trades width against ringing: higher Q is
 * narrower and preserves more neighbouring EEG, but rings for longer.
 */
bool dsp_design_notch(dsp_section_t *out, float fs_hz, float f0_hz, float q);

/* Second-order low- and high-pass sections, Butterworth at Q = 0.7071. */
bool dsp_design_lowpass(dsp_section_t *out, float fs_hz, float fc_hz, float q);
bool dsp_design_highpass(dsp_section_t *out, float fs_hz, float fc_hz, float q);

/*
 * Group delay of one section at a given frequency, in samples.
 *
 * Every active stage delays the signal, and ERP timing is only as good as
 * the correction for it. Combined with hardware-latched DRDY and the measured
 * true sample rate, this is what lets a timestamp be moved to where the
 * brain activity actually happened.
 */
float dsp_section_group_delay(const dsp_section_t *s, float fs_hz, float f_hz);

#endif /* SWIFTEEG_DSP_H */
