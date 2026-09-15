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

/* --- Second-order sections ----------------------------------------------- */

void dsp_cascade_init(dsp_cascade_t *c, uint8_t channels)
{
	if (c == NULL) {
		return;
	}
	memset(c, 0, sizeof(*c));
	c->channels = (channels > DSP_MAX_CHANNELS) ? DSP_MAX_CHANNELS : channels;
}

bool dsp_section_is_valid(const dsp_section_t *s)
{
	return s != NULL &&
	       isfinite(s->g) && isfinite(s->k) &&
	       isfinite(s->m0) && isfinite(s->m1) && isfinite(s->m2) &&
	       s->g > 0.0f && s->k > 0.0f;
}

static bool sections_valid(const dsp_section_t *sections, uint8_t count)
{
	if (count > DSP_MAX_SECTIONS || (count != 0 && sections == NULL)) {
		return false;
	}

	for (uint8_t i = 0; i < count; i++) {
		if (!dsp_section_is_valid(&sections[i])) {
			return false;
		}
	}

	return true;
}

/*
 * Derive what the loop runs. Done here, in float32, so a host that mirrors
 * the loop can derive exactly the same values from the same section.
 */
static void load(dsp_cascade_t *c, const dsp_section_t *sections, uint8_t count)
{
	for (uint8_t i = 0; i < count; i++) {
		const dsp_section_t s = sections[i];
		dsp_section_run_t *r = &c->run[i];

		c->sections[i] = s;
		r->a1 = 1.0f / (1.0f + s.g * (s.g + s.k));
		r->a2 = s.g * r->a1;
		r->a3 = s.g * r->a2;
		r->m0 = s.m0;
		r->m1 = s.m1;
		r->m2 = s.m2;
	}

	c->count = count;
}

bool dsp_cascade_set(dsp_cascade_t *c, const dsp_section_t *sections,
		     uint8_t count)
{
	if (c == NULL || !sections_valid(sections, count)) {
		return false;
	}

	load(c, sections, count);
	dsp_cascade_reset_state(c);
	return true;
}

bool dsp_cascade_retune(dsp_cascade_t *c, const dsp_section_t *sections,
			uint8_t count)
{
	if (c == NULL || count != c->count || !sections_valid(sections, count)) {
		return false;
	}

	load(c, sections, count);
	return true;
}

_Static_assert(DSP_MAX_CHANNELS <= 8, "prime_mask holds one bit per channel");

void dsp_cascade_reset_state(dsp_cascade_t *c)
{
	if (c != NULL) {
		memset(c->state, 0, sizeof(c->state));
		c->prime_mask = (uint8_t)((1u << c->channels) - 1u);
	}
}

float dsp_cascade_apply(dsp_cascade_t *c, uint8_t channel, float x)
{
	if (c == NULL || channel >= c->channels) {
		return x;
	}

	if ((c->prime_mask & (1u << channel)) != 0u) {
		/*
		 * For a steady input the band-pass integrator holds nothing and
		 * the low-pass one holds the input itself, and a section's output
		 * - the next one's input - is (m0 + m2) times it.
		 */
		float in = x;

		for (uint8_t i = 0; i < c->count; i++) {
			c->state[channel][i].ic1 = 0.0f;
			c->state[channel][i].ic2 = in;
			in = (c->run[i].m0 + c->run[i].m2) * in;
		}
		c->prime_mask &= (uint8_t)~(1u << channel);
	}

	for (uint8_t i = 0; i < c->count; i++) {
		const dsp_section_run_t *r = &c->run[i];
		dsp_section_state_t *st = &c->state[channel][i];

		/*
		 * v1 is the band-pass and v2 the low-pass; ic1 and ic2 are the
		 * two trapezoidal integrators' states.
		 */
		const float v3 = x - st->ic2;
		const float v1 = r->a1 * st->ic1 + r->a2 * v3;
		const float v2 = st->ic2 + r->a2 * st->ic1 + r->a3 * v3;

		st->ic1 = 2.0f * v1 - st->ic1;
		st->ic2 = 2.0f * v2 - st->ic2;

		x = r->m0 * x + r->m1 * v1 + r->m2 * v2;
	}

	return x;
}

uint32_t dsp_cascade_settle_samples(const dsp_cascade_t *c)
{
	if (c == NULL) {
		return 0;
	}

	/*
	 * Each section's slowest pole, taken from the analogue prototype it was
	 * prewarped from (w0 = 2 fs g). An underdamped pair decays at w0 / 2Q,
	 * a time constant of 1 / (g k) samples; an overdamped one at its slower
	 * real pole, (k + sqrt(k^2 - 4)) / 4g. Decaying to 1 % takes ln(100),
	 * about 4.6 time constants.
	 */
	float total = 0.0f;

	for (uint8_t i = 0; i < c->count; i++) {
		const float g = c->sections[i].g;
		const float k = c->sections[i].k;
		const float tau = (k > 2.0f)
				  ? (k + sqrtf(k * k - 4.0f)) / (4.0f * g)
				  : 1.0f / (g * k);

		total += 4.6f * tau;
	}

	/* The largest float32 below 2^32; also catches a NaN. */
	if (!(total < 4294967040.0f)) {
		return UINT32_MAX;
	}

	/* Rounded up, so a cascade with nothing to settle is exactly zero. */
	return (uint32_t)ceilf(total);
}

/* --- Section design ------------------------------------------------------ */

static bool design_valid(float fs_hz, float f0_hz, float q)
{
	/* f0 must be below Nyquist, and Q must be sane. */
	return fs_hz > 0.0f && f0_hz > 0.0f && q > 0.0f && f0_hz < fs_hz * 0.5f;
}

/*
 * The corner is prewarped, so a section designed at fc has exactly the
 * response the bilinear transform of its analogue prototype would - the
 * same as the audio-EQ cookbook biquad at that corner and Q.
 */
static void design(dsp_section_t *out, float fs_hz, float fc_hz, float q,
		   float m0, float m1_per_k, float m2)
{
	out->g = tanf(DSP_PI * fc_hz / fs_hz);
	out->k = 1.0f / q;
	out->m0 = m0;
	out->m1 = m1_per_k * out->k;
	out->m2 = m2;
}

bool dsp_design_notch(dsp_section_t *out, float fs_hz, float f0_hz, float q)
{
	if (out == NULL || !design_valid(fs_hz, f0_hz, q)) {
		return false;
	}

	design(out, fs_hz, f0_hz, q, 1.0f, -1.0f, 0.0f);
	return true;
}

bool dsp_design_lowpass(dsp_section_t *out, float fs_hz, float fc_hz, float q)
{
	if (out == NULL || !design_valid(fs_hz, fc_hz, q)) {
		return false;
	}

	design(out, fs_hz, fc_hz, q, 0.0f, 0.0f, 1.0f);
	return true;
}

bool dsp_design_highpass(dsp_section_t *out, float fs_hz, float fc_hz, float q)
{
	if (out == NULL || !design_valid(fs_hz, fc_hz, q)) {
		return false;
	}

	design(out, fs_hz, fc_hz, q, 1.0f, -1.0f, -1.0f);
	return true;
}

float dsp_section_group_delay(const dsp_section_t *s, float fs_hz, float f_hz)
{
	if (s == NULL || fs_hz <= 0.0f) {
		return 0.0f;
	}

	/*
	 * The section as a transfer function n(z) / d(z), both normalised so
	 * d has a leading 1:
	 *   d = d0 + 2(g^2 - 1) z^-1 + (1 - gk + g^2) z^-2,  d0 = 1 + gk + g^2
	 *   n = m0 d + m1 g (1 - z^-2) + m2 g^2 (1 + z^-1)^2
	 */
	const float g = s->g;
	const float k = s->k;
	const float gg = g * g;
	const float d0 = 1.0f + g * k + gg;
	const float d1 = 2.0f * (gg - 1.0f) / d0;
	const float d2 = (1.0f - g * k + gg) / d0;
	const float n0 = (s->m0 * d0 + s->m1 * g + s->m2 * gg) / d0;
	const float n1 = (2.0f * s->m0 * (gg - 1.0f) + 2.0f * s->m2 * gg) / d0;
	const float n2 = (s->m0 * (1.0f - g * k + gg) - s->m1 * g + s->m2 * gg) / d0;

	/*
	 * Group delay via numerical differentiation of the phase response.
	 *
	 * Evaluating H(e^jw) either side of w and differencing the unwrapped
	 * phase is far less error-prone than the closed form, and this is
	 * computed at configuration time, not per sample.
	 */
	const float w = 2.0f * DSP_PI * f_hz / fs_hz;
	const float dw = 1e-4f;

	float phase[2];
	for (int i = 0; i < 2; i++) {
		const float wi = w + (i == 0 ? -dw : dw);
		const float cw1 = cosf(wi),  sw1 = sinf(wi);
		const float cw2 = cosf(2.0f * wi), sw2 = sinf(2.0f * wi);

		const float nr = n0 + n1 * cw1 + n2 * cw2;
		const float ni = -(n1 * sw1 + n2 * sw2);
		const float dr = 1.0f + d1 * cw1 + d2 * cw2;
		const float di = -(d1 * sw1 + d2 * sw2);

		phase[i] = atan2f(ni, nr) - atan2f(di, dr);
	}

	float dphi = phase[1] - phase[0];

	/* Unwrap across the +/-pi branch cut. */
	while (dphi > DSP_PI)  { dphi -= 2.0f * DSP_PI; }
	while (dphi < -DSP_PI) { dphi += 2.0f * DSP_PI; }

	return -dphi / (2.0f * dw);
}
