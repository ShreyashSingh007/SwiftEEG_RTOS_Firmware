#include "chain.h"

#include <errno.h>
#include <math.h>

int chain_init(chain_t *c, uint8_t dc_shift, float vref_volts, uint8_t gain)
{
	if (c == NULL || gain == 0) {
		return -EINVAL;
	}

	c->channels = FRAME_CHANNELS;
	c->vref_volts = vref_volts;
	c->car = false;
	c->car_mask = 0xFFu;
	c->car_runtime_mask = 0xFFu;
	c->mains_mask = 0xFFu;
	c->mains_in = 0.0f;

	for (uint8_t ch = 0; ch < FRAME_CHANNELS; ch++) {
		dsp_dc_init(&c->dc[ch], dc_shift);
		c->lsb_uv[ch] = dsp_lsb_uv(vref_volts, gain);
		c->channel[ch] = (struct chain_channel_config){
			.gain = gain,
			.mux = 0u,
		};
	}

	dsp_cascade_init(&c->pre, FRAME_CHANNELS);
	dsp_cascade_init(&c->notch, FRAME_CHANNELS);
	dsp_cascade_init(&c->post, FRAME_CHANNELS);

	return 0;
}

bool chain_process(chain_t *c, const uint8_t *frame, int32_t *raw_out,
		   float *uv_out)
{
	return chain_process_flags(c, frame, raw_out, uv_out, NULL);
}

bool chain_process_flags(chain_t *c, const uint8_t *frame, int32_t *raw_out,
			 float *uv_out, uint8_t *flags)
{
	if (flags != NULL) {
		*flags = 0u;
	}

	if (c == NULL || frame == NULL || !frame_is_valid(frame)) {
		return false;
	}

	float v[FRAME_CHANNELS];
	float in_sum = 0.0f;
	uint8_t in_n = 0;
	uint8_t valid_mask = 0u;
	uint8_t invalid_mask = 0u;

	for (uint8_t ch = 0; ch < c->channels; ch++) {
		const int32_t raw = frame_channel(frame, ch);

		if (raw_out != NULL) {
			raw_out[ch] = raw;
		}

		/* Stage 0: integer DC removal, before any float appears. */
		const int32_t ac = dsp_dc_apply(&c->dc[ch], raw);

		/* Stage 1: microvolts, at this channel's own gain. */
		const float uv = (float)ac * c->lsb_uv[ch];

		if (!isfinite(uv)) {
			invalid_mask |= (uint8_t)(1u << ch);
			dsp_dc_init(&c->dc[ch], c->dc[ch].shift);
			dsp_cascade_reset_channel(&c->pre, ch);
			dsp_cascade_reset_channel(&c->notch, ch);
			dsp_cascade_reset_channel(&c->post, ch);
			v[ch] = 0.0f;
			continue;
		}

		if ((c->mains_mask & (1u << ch)) != 0u) {
			in_sum += uv;
			in_n++;
		}

		/* Stages 2 and 3: the pre sections - high-pass - then the notch. */
		const float hp = dsp_cascade_apply(&c->pre, ch, uv);

		if (!isfinite(hp)) {
			invalid_mask |= (uint8_t)(1u << ch);
			dsp_dc_init(&c->dc[ch], c->dc[ch].shift);
			dsp_cascade_reset_channel(&c->pre, ch);
			dsp_cascade_reset_channel(&c->notch, ch);
			dsp_cascade_reset_channel(&c->post, ch);
			v[ch] = 0.0f;
			continue;
		}

		v[ch] = dsp_cascade_apply(&c->notch, ch, hp);
		if (!isfinite(v[ch])) {
			invalid_mask |= (uint8_t)(1u << ch);
			dsp_dc_init(&c->dc[ch], c->dc[ch].shift);
			dsp_cascade_reset_channel(&c->pre, ch);
			dsp_cascade_reset_channel(&c->notch, ch);
			dsp_cascade_reset_channel(&c->post, ch);
			v[ch] = 0.0f;
			continue;
		}

		valid_mask |= (uint8_t)(1u << ch);
	}

	c->mains_in = (in_n != 0u) ? in_sum / (float)in_n : 0.0f;

	/*
	 * Stage 4: common average reference. Only masked channels make the
	 * average, and every channel has it subtracted - so a bad electrode is
	 * kept out of the others' reference but is still re-referenced itself.
	 * One channel is not an average of anything: fewer than two leaves the
	 * signal as it was.
	 */
	if (c->car) {
		float sum = 0.0f;
		uint8_t n = 0;

		for (uint8_t ch = 0; ch < c->channels; ch++) {
			const struct chain_channel_config *cfg = &c->channel[ch];

			if ((c->car_mask & c->car_runtime_mask & valid_mask &
			     (1u << ch)) != 0u &&
			    !cfg->power_down && cfg->mux == CHAIN_MUX_NORMAL) {
				sum += v[ch];
				n++;
			}
		}

		if (n >= 2) {
			const float mean = sum / (float)n;

			for (uint8_t ch = 0; ch < c->channels; ch++) {
				if ((valid_mask & (1u << ch)) != 0u) {
					v[ch] -= mean;
				}
			}
		}
	}

	/* Stage 5: the post sections - low-pass. */
	for (uint8_t ch = 0; ch < c->channels; ch++) {
		if ((invalid_mask & (1u << ch)) != 0u) {
			if (uv_out != NULL) {
				uv_out[ch] = NAN;
			}
			continue;
		}

		const float y = dsp_cascade_apply(&c->post, ch, v[ch]);

		if (!isfinite(y)) {
			invalid_mask |= (uint8_t)(1u << ch);
			dsp_dc_init(&c->dc[ch], c->dc[ch].shift);
			dsp_cascade_reset_channel(&c->pre, ch);
			dsp_cascade_reset_channel(&c->notch, ch);
			dsp_cascade_reset_channel(&c->post, ch);
		}

		if (uv_out != NULL) {
			uv_out[ch] = isfinite(y) ? y : NAN;
		}
	}

	if (invalid_mask != 0u && flags != NULL) {
		*flags |= CHAIN_FLAG_DSP_INVALID;
	}

	return true;
}

int chain_set_notch(chain_t *c, float fs_hz, float hz, float q, bool harmonic,
		    bool keep_state, bool *kept)
{
	if (kept != NULL) {
		*kept = false;
	}

	if (c == NULL) {
		return -EINVAL;
	}

	dsp_section_t sections[2];
	uint8_t count = 0;

	if (hz > 0.0f) {
		if (!dsp_design_notch(&sections[0], fs_hz, hz, q)) {
			return -EINVAL;
		}
		count = 1;

		/* The first harmonic, where the rate leaves room for it. */
		const float twice = 2.0f * hz;

		if (harmonic && twice < 0.95f * 0.5f * fs_hz) {
			if (!dsp_design_notch(&sections[1], fs_hz, twice, q)) {
				return -EINVAL;
			}
			count = 2;
		}
	}

	if (keep_state && dsp_cascade_retune(&c->notch, sections, count)) {
		if (kept != NULL) {
			*kept = true;
		}
		return 0;
	}

	return dsp_cascade_set(&c->notch, sections, count) ? 0 : -EINVAL;
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

void chain_set_mains_mask(chain_t *c, uint8_t mask)
{
	if (c != NULL) {
		c->mains_mask = mask;
	}
}

int chain_set_gain(chain_t *c, uint8_t ch, uint8_t gain)
{
	if (c == NULL || ch >= FRAME_CHANNELS) {
		return -EINVAL;
	}

	return chain_set_channel(c, ch, gain, c->channel[ch].mux,
				 c->channel[ch].power_down, c->channel[ch].srb2,
				 NULL);
}

int chain_set_channel(chain_t *c, uint8_t ch, uint8_t gain, uint8_t mux,
		      bool power_down, bool srb2, bool *changed)
{
	if (changed != NULL) {
		*changed = false;
	}

	if (c == NULL || ch >= FRAME_CHANNELS || gain == 0u || mux > 7u) {
		return -EINVAL;
	}

	const struct chain_channel_config next = {
		.gain = gain,
		.mux = mux,
		.power_down = power_down,
		.srb2 = srb2,
	};

	if (c->channel[ch].gain == next.gain &&
	    c->channel[ch].mux == next.mux &&
	    c->channel[ch].power_down == next.power_down &&
	    c->channel[ch].srb2 == next.srb2) {
		return 0;
	}

	c->channel[ch] = next;
	c->lsb_uv[ch] = dsp_lsb_uv(c->vref_volts, gain);
	dsp_dc_init(&c->dc[ch], c->dc[ch].shift);
	dsp_cascade_reset_channel(&c->pre, ch);
	dsp_cascade_reset_channel(&c->notch, ch);
	dsp_cascade_reset_channel(&c->post, ch);

	if (changed != NULL) {
		*changed = true;
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
	dsp_cascade_reset_state(&c->notch);
	dsp_cascade_reset_state(&c->post);
}

uint32_t chain_settle_samples(const chain_t *c)
{
	if (c == NULL) {
		return 0;
	}

	uint32_t total = 0;
	const uint32_t parts[3] = {
		dsp_cascade_settle_samples(&c->pre),
		dsp_cascade_settle_samples(&c->notch),
		dsp_cascade_settle_samples(&c->post),
	};

	for (int i = 0; i < 3; i++) {
		total = (parts[i] > UINT32_MAX - total) ? UINT32_MAX
							 : total + parts[i];
	}

	return total;
}
