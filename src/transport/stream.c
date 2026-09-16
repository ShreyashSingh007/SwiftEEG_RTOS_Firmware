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
 * channels at four bytes, or three at seven when every channel carries both
 * its raw counts and its filtered microvolts. USB carries the same frames;
 * the extra header share costs it nothing it cannot afford.
 */
#define BATCH_SAMPLES      6
#define BATCH_SAMPLES_BOTH 3
#define BATCH_BYTES        (BATCH_SAMPLES * ADS1299_CHANNELS * 4)

BUILD_ASSERT(BATCH_SAMPLES_BOTH * ADS1299_CHANNELS * 7 <= BATCH_BYTES,
	     "a raw + microvolt batch does not fit the batch buffer");

/* Bytes per channel for each encoding. */
static inline uint8_t enc_width(uint8_t enc)
{
	switch (enc) {
	case STREAM_ENC_RAW_I24:
		return 3u;
	case STREAM_ENC_RAW_UV:
		return 7u;
	default:
		return 4u;
	}
}

static inline uint16_t enc_batch(uint8_t enc)
{
	return (enc == STREAM_ENC_RAW_UV) ? BATCH_SAMPLES_BOTH : BATCH_SAMPLES;
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

/*
 * The batch belongs to whichever thread holds this lock: the DSP thread
 * adding a sample, or the command thread stopping the stream and sending
 * what was half built. Without it a STREAM_STOP landing mid-sample could send
 * a frame the DSP thread was still writing.
 */
static K_MUTEX_DEFINE(batch_lock);

static uint8_t batch[DATA_HDR_LEN + BATCH_BYTES];
static uint16_t batch_stride; /* bytes per sample, fixed when a batch opens */
static uint16_t batch_limit;  /* samples per batch, likewise */
static uint16_t batch_count;
static uint64_t batch_ts;
static uint32_t batch_seq;
static uint8_t batch_flags;
static uint8_t batch_encoding;

static uint8_t frame_buf[PROTO_MAX_FRAME];
static uint16_t frame_seq;

/* A mains event: id, flags, u32 seq, f32 Hz. */
#define EVT_MAINS_LEN 10u

static uint8_t event_frame[PROTO_OVERHEAD + EVT_MAINS_LEN];
static uint16_t event_seq;

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

/* The 24 bits of a sample; the converter produces no more. */
static inline void put_i24(uint8_t *p, int32_t v)
{
	const uint32_t u = (uint32_t)v;

	p[0] = (uint8_t)(u & 0xFFu);
	p[1] = (uint8_t)((u >> 8) & 0xFFu);
	p[2] = (uint8_t)((u >> 16) & 0xFFu);
}

/*
 * A float's bytes, copied rather than cast through a pointer: the
 * destination is unaligned inside the batch, and the host reads it back as
 * IEEE 754.
 */
static inline void put_f32(uint8_t *p, float v)
{
	uint32_t bits;

	memcpy(&bits, &v, sizeof(bits));
	put_u32(p, bits);
}

/*
 * Frames waiting for the radio.
 *
 * bt_gatt_notify() waits for a transmit buffer when the link is saturated,
 * and it was the DSP thread that called it: at 1000 SPS with raw and
 * microvolt samples that wait let the acquisition ring fill until the
 * interrupt began dropping frames - 107 samples in 20 s, with nothing to say
 * so (2026-09-16). Refusing to wait instead only moved the loss: a refused
 * frame is a frame thrown away, and delivery fell at every rate.
 *
 * So a finished frame is copied here, and a thread of its own does the
 * waiting. Acquisition never blocks, and the radio stays as full as the link
 * allows. A queue that fills drops the newest frame and counts it, which
 * costs a batch rather than a sample and shows up as a gap in the sequence
 * numbers a host is already checking.
 */
#define TX_QUEUE_FRAMES 32
#define TX_FRAME_BYTES  244 /* one notification at a 247-byte ATT MTU */
#define TX_STACK_SIZE   2048
#define TX_PRIORITY     4   /* below the DSP thread, above the command one */

struct tx_frame {
	uint16_t len;
	uint8_t  data[TX_FRAME_BYTES];
};

BUILD_ASSERT(PROTO_OVERHEAD + DATA_HDR_LEN + BATCH_BYTES <= TX_FRAME_BYTES,
	     "a data frame does not fit the transmit queue");

K_MSGQ_DEFINE(tx_queue, sizeof(struct tx_frame), TX_QUEUE_FRAMES, 4);

static atomic_t tx_dropped;

/*
 * Both links carry byte-identical frames, and both are fed: a host on either
 * one sees the same stream, and unplugging USB mid-session does not
 * interrupt BLE. The USB transport counts its own drops.
 */
static void deliver(const uint8_t *frame, size_t len)
{
	if (usb_transport_is_connected()) {
		(void)usb_transport_write(frame, len);
	}

	if (ble_transport_is_streaming()) {
		const uint16_t room = ble_transport_max_payload();

		if (room != 0 && len > room) {
			/* Would be dropped by the stack; count it here instead
			 * of losing it silently. */
			atomic_inc(&ble_too_big);
		} else if (ble_transport_send_stream(frame, (uint16_t)len) != 0) {
			atomic_inc(&ble_dropped);
		}
	}
}

static void tx_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (1) {
		struct tx_frame tx;

		if (k_msgq_get(&tx_queue, &tx, K_FOREVER) == 0) {
			deliver(tx.data, tx.len);
		}
	}
}

K_THREAD_DEFINE(stream_tx, TX_STACK_SIZE, tx_entry, NULL, NULL, NULL,
		TX_PRIORITY, 0, 0);

bool stream_send(const uint8_t *frame, size_t len)
{
	struct tx_frame tx;

	if (len > sizeof(tx.data)) {
		atomic_inc(&ble_too_big);
		return false;
	}

	tx.len = (uint16_t)len;
	memcpy(tx.data, frame, len);

	if (k_msgq_put(&tx_queue, &tx, K_NO_WAIT) != 0) {
		atomic_inc(&tx_dropped);
		return false;
	}

	return true;
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
	batch[13] = batch_encoding;
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

	k_mutex_lock(&batch_lock, K_FOREVER);

	/* Stopped while this thread waited for the lock. */
	if (!enabled) {
		k_mutex_unlock(&batch_lock);
		return;
	}

	if (batch_count == 0) {
		/* Fixed for the batch: the header declares one encoding. */
		batch_ts = s->ts_us;
		batch_seq = s->seq;
		batch_encoding = encoding;
		batch_stride = (uint16_t)(ADS1299_CHANNELS * enc_width(encoding));
		batch_limit = enc_batch(encoding);
	}

	uint8_t *p = &batch[DATA_HDR_LEN + (size_t)batch_count * batch_stride];

	for (uint8_t ch = 0; ch < ADS1299_CHANNELS; ch++) {
		switch (batch_encoding) {
		case STREAM_ENC_UV_F32:
			put_f32(p, s->ch_uv[ch]);
			p += 4;
			break;

		case STREAM_ENC_RAW_I24:
			/*
			 * The converter is 24-bit, so the low three bytes are
			 * the whole sample; the fourth carried only sign
			 * extension the host can rebuild.
			 */
			put_i24(p, s->ch_raw[ch]);
			p += 3;
			break;

		case STREAM_ENC_RAW_UV:
			/*
			 * Both views of the same sample: what the converter
			 * said, and what the chain made of it. A host records
			 * the first and shows the second.
			 */
			put_i24(p, s->ch_raw[ch]);
			put_f32(&p[3], s->ch_uv[ch]);
			p += 7;
			break;

		default:
			put_u32(p, (uint32_t)s->ch_raw[ch]);
			p += 4;
			break;
		}
	}

	batch_flags |= s->flags;
	batch_count++;

	if (batch_count >= batch_limit) {
		flush();
	}

	k_mutex_unlock(&batch_lock);
}

void stream_on_mains(float hz, uint32_t seq, bool moved)
{
	if (!enabled) {
		return;
	}

	uint8_t payload[EVT_MAINS_LEN];

	payload[0] = STREAM_EVT_MAINS;
	payload[1] = moved ? 0x01u : 0x00u;
	put_u32(&payload[2], seq);
	put_f32(&payload[6], hz);

	const int n = proto_encode(PROTO_TYPE_EVT, PROTO_FLAG_NONE, event_seq++,
				   payload, sizeof(payload), event_frame,
				   sizeof(event_frame));

	if (n > 0) {
		(void)stream_send(event_frame, (size_t)n);
	}
}

void stream_set_encoding(uint8_t enc)
{
	if (enc > STREAM_ENC_RAW_UV) {
		return;
	}

	/*
	 * A batch already open keeps the encoding it opened with, so a change
	 * takes effect cleanly at the next one.
	 */
	encoding = enc;
}

uint8_t stream_encoding(void)
{
	return encoding;
}

void stream_enable(bool on)
{
	k_mutex_lock(&batch_lock, K_FOREVER);
	if (!on) {
		flush();
	}
	batch_count = 0;
	enabled = on;
	k_mutex_unlock(&batch_lock);
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
	out->queue_dropped = (uint32_t)atomic_get(&tx_dropped);
}

void stream_reset_stats(void)
{
	memset(&stats, 0, sizeof(stats));
	atomic_set(&ble_dropped, 0);
	atomic_set(&ble_too_big, 0);
	atomic_set(&tx_dropped, 0);
}
