#include "chain.h"

#include <errno.h>

int chain_init(chain_t *c, float fs_hz, uint8_t dc_shift, float notch_hz,
	       float notch_q, float vref_volts, uint8_t gain)
{
	if (c == NULL || gain == 0) {
		return -EINVAL;
	}

	c->channels = FRAME_CHANNELS;
	c->vref_volts = vref_volts;
	c->car = false;
	c->car_mask = 0xFFu;

	for (uint8_t ch = 0; ch < FRAME_CHANNELS; ch++) {
		dsp_dc_init(&c->dc[ch], dc_shift);
		c->lsb_uv[ch] = dsp_lsb_uv(vref_volts, gain);
	}

	dsp_cascade_init(&c->pre, FRAME_CHANNELS);
	dsp_cascade_init(&c->post, FRAME_CHANNELS);

	return chain_set_notch(c, fs_hz, notch_hz, notch_q);
}

bool chain_process(chain_t *c, const uint8_t *frame, int32_t *raw_out,
		   float *uv_out)
{
	if (!frame_is_valid(frame)) {
		return false;
	}

	float v[FRAME_CHANNELS];

	for (uint8_t ch = 0; ch < c->channels; ch++) {
		const int32_t raw = frame_channel(frame, ch);

		if (raw_out != NULL) {
			raw_out[ch] = raw;
		}

		/* Stage 0: integer DC removal, before any float appears. */
		const int32_t ac = dsp_dc_apply(&c->dc[ch], raw);

		/* Stage 1: microvolts, at this channel's own gain. */
		const float uv = (float)ac * c->lsb_uv[ch];

		/* Stage 2: the pre sections - high-pass and notch. */
		v[ch] = dsp_cascade_apply(&c->pre, ch, uv);
	}

	/*
	 * Stage 3: common average reference. Only masked channels make the
	 * average, and every channel has it subtracted - so a bad electrode is
	 * kept out of the others' reference but is still re-referenced itself.
	 * One channel is not an average of anything: fewer than two leaves the
	 * signal as it was.
	 */
	if (c->car) {
		float sum = 0.0f;
		uint8_t n = 0;

		for (uint8_t ch = 0; ch < c->channels; ch++) {
			if ((c->car_mask & (1u << ch)) != 0u) {
				sum += v[ch];
				n++;
			}
		}

		if (n >= 2) {
			const float mean = sum / (float)n;

			for (uint8_t ch = 0; ch < c->channels; ch++) {
				v[ch] -= mean;
			}
		}
	}

	/* Stage 4: the post sections - low-pass. */
	for (uint8_t ch = 0; ch < c->channels; ch++) {
		const float y = dsp_cascade_apply(&c->post, ch, v[ch]);

		if (uv_out != NULL) {
			uv_out[ch] = y;
		}
	}

	return true;
}

int chain_set_notch(chain_t *c, float fs_hz, float notch_hz, float notch_q)
{
	if (c == NULL) {
		return -EINVAL;
	}

	if (notch_hz <= 0.0f) {
		/* No sections: the pre stage passes samples straight through. */
		(void)dsp_cascade_set(&c->pre, NULL, 0);
		return 0;
	}

	dsp_section_t notch;

	if (!dsp_design_notch(&notch, fs_hz, notch_hz, notch_q)) {
		return -EINVAL;
	}

	return dsp_cascade_set(&c->pre, &notch, 1) ? 0 : -EINVAL;
}

int chain_set_stage(chain_t *c, uint8_t stage, const dsp_section_t *sections,
		    uint8_t count, bool keep_state, bool *kept)
{
	if (kept != NULL) {
		*kept = false;
	}

	if (c == NULL || stage > CHAIN_STAGE_POST) {
		return -EINVAL;
	}

	dsp_cascade_t *cas = (stage == CHAIN_STAGE_PRE) ? &c->pre : &c->post;

	/*
	 * A retune keeps the state only when it can - the same number of
	 * sections. Anything else is a different filter and starts again,
	 * primed on its next sample.
	 */
	if (keep_state && dsp_cascade_retune(cas, sections, count)) {
		if (kept != NULL) {
			*kept = true;
		}
		return 0;
	}

	return dsp_cascade_set(cas, sections, count) ? 0 : -EINVAL;
}

void chain_set_car(chain_t *c, bool enable, uint8_t mask)
{
	if (c != NULL) {
		c->car = enable;
		c->car_mask = mask;
	}
}

int chain_set_gain(chain_t *c, uint8_t ch, uint8_t gain)
{
	if (c == NULL || ch >= FRAME_CHANNELS || gain == 0) {
		return -EINVAL;
	}

	const float lsb = dsp_lsb_uv(c->vref_volts, gain);

	if (lsb != c->lsb_uv[ch]) {
		c->lsb_uv[ch] = lsb;
		dsp_dc_init(&c->dc[ch], c->dc[ch].shift);
	}

	return 0;
}

void chain_reset(chain_t *c)
{
	if (c == NULL) {
		return;
	}

	for (uint8_t ch = 0; ch < FRAME_CHANNELS; ch++) {
		dsp_dc_init(&c->dc[ch], c->dc[ch].shift);
	}

	dsp_cascade_reset_state(&c->pre);
	dsp_cascade_reset_state(&c->post);
}

uint32_t chain_settle_samples(const chain_t *c)
{
	if (c == NULL) {
		return 0;
	}

	const uint32_t a = dsp_cascade_settle_samples(&c->pre);
	const uint32_t b = dsp_cascade_settle_samples(&c->post);

	return (a > UINT32_MAX - b) ? UINT32_MAX : a + b;
}
