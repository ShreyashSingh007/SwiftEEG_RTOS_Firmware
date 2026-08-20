/*
 * SwiftEEG on-chip DSP primitives.
 *
 * Design rule: firmware does only what MUST happen on-chip, and exposes
 * everything else as a host-programmable biquad cascade. No filtering opinion
 * is baked in. See README section 7.
 *
 * Notably absent by design: a fixed high-pass above ~0.1 Hz. Causal high-pass
 * corners above roughly 0.1-0.3 Hz measurably distort slow ERP components
 * such as P300, which is exactly what this device exists to measure well.
 *
 * Pure maths, depending only on the C standard library, so the whole chain is
 * validated against a SciPy reference off-target rather than by eye.
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

/* --- Biquad cascade ------------------------------------------------------ */

/*
 * Transposed direct form II: better numeric behaviour in float than DF1 and
 * it needs only two state words per section.
 *
 * Coefficients are normalised so a0 == 1, and a1/a2 are stored already
 * negated, so the inner loop is all multiply-accumulate with no sign flips:
 *
 *   y   = b0*x + s1
 *   s1  = b1*x + a1*y + s2
 *   s2  = b2*x + a2*y
 */
typedef struct {
	float b0, b1, b2;
	float a1, a2;   /* negated */
} dsp_biquad_coeffs_t;

typedef struct {
	float s1, s2;
} dsp_biquad_state_t;

#define DSP_MAX_SECTIONS 8

typedef struct {
	dsp_biquad_coeffs_t coeffs[DSP_MAX_SECTIONS];
	dsp_biquad_state_t  state[DSP_MAX_CHANNELS][DSP_MAX_SECTIONS];
	uint8_t sections;
	uint8_t channels;
} dsp_cascade_t;

void dsp_cascade_init(dsp_cascade_t *c, uint8_t channels);
bool dsp_cascade_set(dsp_cascade_t *c, const dsp_biquad_coeffs_t *coeffs,
		     uint8_t sections);
void dsp_cascade_reset_state(dsp_cascade_t *c);
float dsp_cascade_apply(dsp_cascade_t *c, uint8_t channel, float x);

/* --- Coefficient design (host may also upload coefficients directly) ----- */

/*
 * Second-order IIR notch. `q` trades width against ringing: higher Q is
 * narrower and preserves more neighbouring EEG, but rings for longer.
 */
bool dsp_design_notch(dsp_biquad_coeffs_t *out, float fs_hz, float f0_hz, float q);

/* Second-order Butterworth sections, for the default presets. */
bool dsp_design_lowpass(dsp_biquad_coeffs_t *out, float fs_hz, float fc_hz, float q);
bool dsp_design_highpass(dsp_biquad_coeffs_t *out, float fs_hz, float fc_hz, float q);

/*
 * Group delay of one section at a given frequency, in samples.
 *
 * Every active stage adds delay, and firmware reports the total so the host
 * can correct sample timestamps exactly. Combined with hardware-latched DRDY
 * and the measured true sample rate, this is what makes ERP timing
 * trustworthy end to end. Most systems leave the user to guess it.
 */
float dsp_biquad_group_delay(const dsp_biquad_coeffs_t *c, float fs_hz, float f_hz);

#endif /* SWIFTEEG_DSP_H */
