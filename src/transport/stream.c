#include "stream.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "proto/proto.h"
#include "usb.h"

LOG_MODULE_REGISTER(stream, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Samples per DATA frame.
 *
 * 16 samples of 8 channels at 4 bytes each is 512 bytes, plus a 16-byte
 * header, which sits inside the protocol's 1024-byte payload limit with room
 * to spare. At 250 SPS that is about 16 frames a second - frequent enough
 * that a viewer looks live, large enough that headers are not the bulk of
 * the traffic.
 */
#define BATCH_SAMPLES 16

/*
 * DATA payload header, little-endian, ahead of the sample block:
 *
 *   u64 ts_us      hardware timestamp of the first sample in the batch
 *   u32 seq        sample index of the first sample
 *   u8  channels
 *   u8  encoding   STREAM_ENC_*
 *   u16 count      samples in this batch
 */
#define DATA_HDR_LEN 16

static uint8_t batch[DATA_HDR_LEN + BATCH_SAMPLES * ADS1299_CHANNELS * 4];
static uint16_t batch_count;
static uint64_t batch_ts;
static uint32_t batch_seq;
static uint8_t batch_flags;

static uint8_t frame_buf[PROTO_MAX_FRAME];
static uint16_t frame_seq;

static uint8_t encoding = STREAM_ENC_RAW_I32;
static bool enabled;

static struct stream_stats stats;

static inline void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)(v >> 8);
}

static inline void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void flush(void)
{
	if (batch_count == 0) {
		return;
	}

	/* Fill in the header now that the batch is complete. */
	put_u32(&batch[0], (uint32_t)(batch_ts & 0xFFFFFFFFu));
	put_u32(&batch[4], (uint32_t)(batch_ts >> 32));
	put_u32(&batch[8], batch_seq);
	batch[12] = ADS1299_CHANNELS;
	batch[13] = encoding;
	put_u16(&batch[14], batch_count);

	const size_t payload_len =
		DATA_HDR_LEN + (size_t)batch_count * ADS1299_CHANNELS * 4u;

	const int n = proto_encode(PROTO_TYPE_DATA, batch_flags, frame_seq++,
				   batch, payload_len, frame_buf,
				   sizeof(frame_buf));

	if (n > 0) {
		const size_t sent = usb_transport_write(frame_buf, (size_t)n);

		if (sent < (size_t)n) {
			stats.bytes_dropped += (uint32_t)((size_t)n - sent);
		} else {
			stats.frames_sent++;
			stats.samples_sent += batch_count;
		}
	}

	batch_count = 0;
	batch_flags = 0;
}

void stream_on_sample(const struct eeg_sample *s)
{
	if (!enabled) {
		return;
	}

	if (batch_count == 0) {
		batch_ts = s->ts_us;
		batch_seq = s->seq;
	}

	uint8_t *p = &batch[DATA_HDR_LEN +
			    (size_t)batch_count * ADS1299_CHANNELS * 4u];

	for (uint8_t ch = 0; ch < ADS1299_CHANNELS; ch++) {
		if (encoding == STREAM_ENC_UV_F32) {
			/*
			 * Copy the float's bytes rather than casting through
			 * a pointer: the destination is unaligned inside the
			 * batch, and the host reads it back as IEEE 754.
			 */
			float v = s->ch_uv[ch];
			uint32_t bits;

			memcpy(&bits, &v, sizeof(bits));
			put_u32(p, bits);
		} else {
			put_u32(p, (uint32_t)s->ch_raw[ch]);
		}
		p += 4;
	}

	batch_flags |= s->flags;
	batch_count++;

	if (batch_count >= BATCH_SAMPLES) {
		flush();
	}
}

void stream_set_encoding(uint8_t enc)
{
	encoding = (enc == STREAM_ENC_UV_F32) ? STREAM_ENC_UV_F32
					      : STREAM_ENC_RAW_I32;
}

void stream_enable(bool on)
{
	if (!on) {
		flush();
	}
	batch_count = 0;
	enabled = on;
}

bool stream_enabled(void)
{
	return enabled;
}

void stream_get_stats(struct stream_stats *out)
{
	*out = stats;
	out->bytes_dropped += usb_transport_dropped();
}

void stream_reset_stats(void)
{
	memset(&stats, 0, sizeof(stats));
}
