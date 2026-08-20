#include "dsp.h"

#include <math.h>
#include <string.h>

/*
 * M_PI is not guaranteed by the C standard and picolibc does not define it
 * under -std=c17, so carry our own rather than depend on a POSIX extension.
 */
#define DSP_PI 3.14159265358979323846f

/* --- Stage 0: integer DC removal ---------------------------------------- */

void dsp_dc_init(dsp_dc_t *dc, uint8_t shift)
{
	if (dc == NULL) {
		return;
	}
	dc->acc = 0;
	dc->shift = shift;
	dc->primed = false;
}

int32_t dsp_dc_apply(dsp_dc_t *dc, int32_t sample)
{
	if (dc == NULL) {
		return sample;
	}

	if (!dc->primed) {
		/*
		 * Start already settled on the first sample. Without this the
		 * estimate would ramp from zero and the output would carry a
		 * large decaying transient for many seconds.
		 */
		dc->acc = (int64_t)sample << dc->shift;
		dc->primed = true;
	} else {
		/* acc += (sample - acc>>shift); a first-order leaky integrator. */
		dc->acc += (int64_t)sample - (dc->acc >> dc->shift);
	}

	return sample - (int32_t)(dc->acc >> dc->shift);
}

/* --- Stage 1: scaling ---------------------------------------------------- */

float dsp_lsb_uv(float vref_volts, uint8_t gain)
{
	if (gain == 0) {
		return 0.0f;
	}
	/* (2*VREF / (gain * 2^24)) volts -> microvolts */
	return (2.0f * vref_volts) / ((float)gain * 16777216.0f) * 1e6f;
}

/* --- Biquad cascade ------------------------------------------------------ */

void dsp_cascade_init(dsp_cascade_t *c, uint8_t channels)
{
	if (c == NULL) {
		return;
	}
	memset(c, 0, sizeof(*c));
	c->channels = (channels > DSP_MAX_CHANNELS) ? DSP_MAX_CHANNELS : channels;
	c->sections = 0;
}

bool dsp_cascade_set(dsp_cascade_t *c, const dsp_biquad_coeffs_t *coeffs,
		     uint8_t sections)
{
	if (c == NULL || coeffs == NULL || sections > DSP_MAX_SECTIONS) {
		return false;
	}

	memcpy(c->coeffs, coeffs, (size_t)sections * sizeof(*coeffs));
	c->sections = sections;
	dsp_cascade_reset_state(c);
	return true;
}

void dsp_cascade_reset_state(dsp_cascade_t *c)
{
	if (c != NULL) {
		memset(c->state, 0, sizeof(c->state));
	}
}

float dsp_cascade_apply(dsp_cascade_t *c, uint8_t channel, float x)
{
	if (c == NULL || channel >= c->channels) {
		return x;
	}

	for (uint8_t s = 0; s < c->sections; s++) {
		const dsp_biquad_coeffs_t *k = &c->coeffs[s];
		dsp_biquad_state_t *st = &c->state[channel][s];

		/* Transposed direct form II. a1/a2 are pre-negated. */
		const float y = k->b0 * x + st->s1;
		st->s1 = k->b1 * x + k->a1 * y + st->s2;
		st->s2 = k->b2 * x + k->a2 * y;
		x = y;
	}

	return x;
}

/* --- Coefficient design -------------------------------------------------- */

/*
 * Standard RBJ audio-EQ cookbook forms. Coefficients are normalised by a0 and
 * the feedback terms negated on the way out, matching the inner loop above.
 */
static void normalise(dsp_biquad_coeffs_t *out,
		      float b0, float b1, float b2,
		      float a0, float a1, float a2)
{
	const float inv = 1.0f / a0;
	out->b0 = b0 * inv;
	out->b1 = b1 * inv;
	out->b2 = b2 * inv;
	out->a1 = -a1 * inv;
	out->a2 = -a2 * inv;
}

static bool design_valid(float fs_hz, float f0_hz, float q)
{
	/* f0 must be below Nyquist, and Q must be sane. */
	return fs_hz > 0.0f && f0_hz > 0.0f && q > 0.0f && f0_hz < fs_hz * 0.5f;
}

bool dsp_design_notch(dsp_biquad_coeffs_t *out, float fs_hz, float f0_hz, float q)
{
	if (out == NULL || !design_valid(fs_hz, f0_hz, q)) {
		return false;
	}

	const float w0 = 2.0f * DSP_PI * f0_hz / fs_hz;
	const float cw = cosf(w0);
	const float alpha = sinf(w0) / (2.0f * q);

	normalise(out, 1.0f, -2.0f * cw, 1.0f,
		       1.0f + alpha, -2.0f * cw, 1.0f - alpha);
	return true;
}

bool dsp_design_lowpass(dsp_biquad_coeffs_t *out, float fs_hz, float fc_hz, float q)
{
	if (out == NULL || !design_valid(fs_hz, fc_hz, q)) {
		return false;
	}

	const float w0 = 2.0f * DSP_PI * fc_hz / fs_hz;
	const float cw = cosf(w0);
	const float alpha = sinf(w0) / (2.0f * q);
	const float b1 = 1.0f - cw;

	normalise(out, b1 * 0.5f, b1, b1 * 0.5f,
		       1.0f + alpha, -2.0f * cw, 1.0f - alpha);
	return true;
}

bool dsp_design_highpass(dsp_biquad_coeffs_t *out, float fs_hz, float fc_hz, float q)
{
	if (out == NULL || !design_valid(fs_hz, fc_hz, q)) {
		return false;
	}

	const float w0 = 2.0f * DSP_PI * fc_hz / fs_hz;
	const float cw = cosf(w0);
	const float alpha = sinf(w0) / (2.0f * q);
	const float b0 = (1.0f + cw) * 0.5f;

	normalise(out, b0, -(1.0f + cw), b0,
		       1.0f + alpha, -2.0f * cw, 1.0f - alpha);
	return true;
}

float dsp_biquad_group_delay(const dsp_biquad_coeffs_t *c, float fs_hz, float f_hz)
{
	if (c == NULL || fs_hz <= 0.0f) {
		return 0.0f;
	}

	/*
	 * Group delay via numerical differentiation of the phase response.
	 *
	 * Evaluating H(e^jw) either side of w and differencing the unwrapped
	 * phase is far less error-prone than the closed form, and this is
	 * computed once at configuration time, not per sample.
	 */
	const float w = 2.0f * DSP_PI * f_hz / fs_hz;
	const float dw = 1e-4f;

	float phase[2];
	for (int i = 0; i < 2; i++) {
		const float wi = w + (i == 0 ? -dw : dw);
		const float cw1 = cosf(wi),  sw1 = sinf(wi);
		const float cw2 = cosf(2.0f * wi), sw2 = sinf(2.0f * wi);

		/* a1/a2 are stored negated, so undo that here. */
		const float nr = c->b0 + c->b1 * cw1 + c->b2 * cw2;
		const float ni = -(c->b1 * sw1 + c->b2 * sw2);
		const float dr = 1.0f - c->a1 * cw1 - c->a2 * cw2;
		const float di = -(-c->a1 * sw1 - c->a2 * sw2);

		phase[i] = atan2f(ni, nr) - atan2f(di, dr);
	}

	float dphi = phase[1] - phase[0];

	/* Unwrap across the +/-pi branch cut. */
	while (dphi > DSP_PI)  { dphi -= 2.0f * DSP_PI; }
	while (dphi < -DSP_PI) { dphi += 2.0f * DSP_PI; }

	return -dphi / (2.0f * dw);
}
