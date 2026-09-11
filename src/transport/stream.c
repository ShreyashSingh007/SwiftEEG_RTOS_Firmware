#include "stream.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "proto/proto.h"
#include "ble.h"
#include "usb.h"

LOG_MODULE_REGISTER(stream, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Samples per DATA frame.
 *
 * Sized so one frame fits in a single BLE notification. A notification
 * larger than the negotiated ATT MTU is dropped by the stack without
 * complaint, which would look like a device that streams over USB and goes
 * quiet over BLE.
 *
 * With a 247-byte MTU there are 244 bytes to spend, minus 10 of protocol
 * overhead and 16 of data header, leaving 218 - six samples of eight
 * channels at four bytes. USB carries the same frames; the extra header
 * share costs it nothing it cannot afford.
 */
#define BATCH_SAMPLES 6

/* Bytes per channel for each encoding. */
static inline uint8_t enc_width(uint8_t enc)
{
	return (enc == STREAM_ENC_RAW_I24) ? 3u : 4u;
}

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
static uint16_t batch_stride; /* bytes per sample, fixed when a batch opens */
static uint16_t batch_count;
static uint64_t batch_ts;
static uint32_t batch_seq;
static uint8_t batch_flags;

static uint8_t frame_buf[PROTO_MAX_FRAME];
static uint16_t frame_seq;

/*
 * Packed 24-bit by default. The converter is 24-bit, so a 32-bit sample adds
 * only a byte of sign extension - a third more radio traffic for nothing.
 */
static uint8_t encoding = STREAM_ENC_RAW_I24;
static bool enabled;

/*
 * Frame and sample counts belong to the DSP thread. Link drops are counted
 * by whichever thread is sending - the IMU thread shares the links - so
 * those are atomic.
 */
static struct stream_stats stats;
static atomic_t ble_dropped;
static atomic_t ble_too_big;

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

bool stream_send(const uint8_t *frame, size_t len)
{
	bool delivered = false;

	/*
	 * Both links carry byte-identical frames, and both are fed: a host on
	 * either one sees the same stream, and unplugging USB mid-session does
	 * not interrupt BLE. The USB transport counts its own drops.
	 */
	if (usb_transport_is_connected() &&
	    usb_transport_write(frame, len) == len) {
		delivered = true;
	}

	if (ble_transport_is_streaming()) {
		const uint16_t room = ble_transport_max_payload();

		if (room != 0 && len > room) {
			/* Would be dropped by the stack; count it here instead
			 * of losing it silently. */
			atomic_inc(&ble_too_big);
		} else if (ble_transport_send_stream(frame, (uint16_t)len) == 0) {
			delivered = true;
		} else {
			atomic_inc(&ble_dropped);
		}
	}

	return delivered;
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
		DATA_HDR_LEN + (size_t)batch_count * batch_stride;

	const int n = proto_encode(PROTO_TYPE_DATA, batch_flags, frame_seq++,
				   batch, payload_len, frame_buf,
				   sizeof(frame_buf));

	if (n > 0 && stream_send(frame_buf, (size_t)n)) {
		stats.frames_sent++;
		stats.samples_sent += batch_count;
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
		/* Fixed for the batch: the header declares one encoding. */
		batch_stride = (uint16_t)(ADS1299_CHANNELS * enc_width(encoding));
	}

	uint8_t *p = &batch[DATA_HDR_LEN + (size_t)batch_count * batch_stride];

	for (uint8_t ch = 0; ch < ADS1299_CHANNELS; ch++) {
		switch (encoding) {
		case STREAM_ENC_UV_F32: {
			/*
			 * Copy the float's bytes rather than casting through
			 * a pointer: the destination is unaligned inside the
			 * batch, and the host reads it back as IEEE 754.
			 */
			float v = s->ch_uv[ch];
			uint32_t bits;

			memcpy(&bits, &v, sizeof(bits));
			put_u32(p, bits);
			p += 4;
			break;
		}

		case STREAM_ENC_RAW_I24: {
			/*
			 * The converter is 24-bit, so the low three bytes are
			 * the whole sample; the fourth carried only sign
			 * extension the host can rebuild.
			 */
			const uint32_t v = (uint32_t)s->ch_raw[ch];

			p[0] = (uint8_t)(v & 0xFFu);
			p[1] = (uint8_t)((v >> 8) & 0xFFu);
			p[2] = (uint8_t)((v >> 16) & 0xFFu);
			p += 3;
			break;
		}

		default:
			put_u32(p, (uint32_t)s->ch_raw[ch]);
			p += 4;
			break;
		}
	}

	batch_flags |= s->flags;
	batch_count++;

	if (batch_count >= BATCH_SAMPLES) {
		flush();
	}
}

void stream_set_encoding(uint8_t enc)
{
	if (enc > STREAM_ENC_RAW_I24) {
		return;
	}

	/* Flush first: the header declares one encoding for the whole batch. */
	if (enc != encoding) {
		flush();
	}

	encoding = enc;
}

uint8_t stream_encoding(void)
{
	return encoding;
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
	out->bytes_dropped = usb_transport_dropped();
	out->ble_dropped = (uint32_t)atomic_get(&ble_dropped);
	out->ble_too_big = (uint32_t)atomic_get(&ble_too_big);
}

void stream_reset_stats(void)
{
	memset(&stats, 0, sizeof(stats));
	atomic_set(&ble_dropped, 0);
	atomic_set(&ble_too_big, 0);
}
