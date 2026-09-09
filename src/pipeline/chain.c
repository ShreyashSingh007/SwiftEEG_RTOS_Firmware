#include "chain.h"

#include <errno.h>

int chain_init(chain_t *c, float fs_hz, uint8_t dc_shift, float notch_hz,
	       float notch_q, float vref_volts, uint8_t gain)
{
	if (c == NULL) {
		return -EINVAL;
	}

	c->channels = FRAME_CHANNELS;
	c->lsb_uv = dsp_lsb_uv(vref_volts, gain);

	for (uint8_t ch = 0; ch < FRAME_CHANNELS; ch++) {
		dsp_dc_init(&c->dc[ch], dc_shift);
	}

	dsp_cascade_init(&c->cascade, FRAME_CHANNELS);

	dsp_biquad_coeffs_t notch;

	if (!dsp_design_notch(&notch, fs_hz, notch_hz, notch_q)) {
		return -EINVAL;
	}

	if (!dsp_cascade_set(&c->cascade, &notch, 1)) {
		return -EINVAL;
	}

	return 0;
}

bool chain_process(chain_t *c, const uint8_t *frame, int32_t *raw_out,
		   float *uv_out)
{
	if (!frame_is_valid(frame)) {
		return false;
	}

	for (uint8_t ch = 0; ch < c->channels; ch++) {
		const int32_t raw = frame_channel(frame, ch);

		if (raw_out != NULL) {
			raw_out[ch] = raw;
		}

		/* Stage 0: integer DC removal, before any float appears. */
		const int32_t ac = dsp_dc_apply(&c->dc[ch], raw);

		/* Stage 1: microvolts. */
		float uv = (float)ac * c->lsb_uv;

		/* Stage 2: the biquad cascade, currently the mains notch. */
		uv = dsp_cascade_apply(&c->cascade, ch, uv);

		if (uv_out != NULL) {
			uv_out[ch] = uv;
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
		/* No sections: the cascade passes samples straight through. */
		dsp_cascade_set(&c->cascade, NULL, 0);
		return 0;
	}

	dsp_biquad_coeffs_t notch;

	if (!dsp_design_notch(&notch, fs_hz, notch_hz, notch_q)) {
		return -EINVAL;
	}

	if (!dsp_cascade_set(&c->cascade, &notch, 1)) {
		return -EINVAL;
	}

	return 0;
}
