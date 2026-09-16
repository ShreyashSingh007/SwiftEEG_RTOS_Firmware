/*
 * Sample streaming over the binary protocol.
 *
 * Takes completed samples from the DSP thread, batches them, and sends them
 * out as protocol DATA frames. Batching matters: a frame carries 10 bytes of
 * protocol overhead plus a 16-byte data header, so sending one sample at a
 * time would spend more on headers than on signal.
 *
 * Encoding is raw ADC counts by default. That is what the host needs to
 * check the AFE against its own test generator - a known amplitude in counts
 * is only known before anything filters it.
 */
#ifndef SWIFTEEG_TRANSPORT_STREAM_H
#define SWIFTEEG_TRANSPORT_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pipeline/pipeline.h"

/* What the samples in a DATA frame look like. */
#define STREAM_ENC_RAW_I32 0u /* ADC counts, before the DSP chain */
#define STREAM_ENC_UV_F32  1u /* microvolts, after the DSP chain */
#define STREAM_ENC_RAW_I24 2u /* ADC counts packed to three bytes */
#define STREAM_ENC_RAW_UV  3u /* both, per channel: counts in three bytes,
                               * then microvolts after the chain */

struct stream_stats {
	uint32_t frames_sent;
	uint32_t samples_sent;
	uint32_t bytes_dropped; /* USB buffer was full */
	uint32_t ble_dropped;   /* notification refused, usually no buffers */
	uint32_t ble_too_big;   /* frame exceeded the negotiated MTU */
	uint32_t queue_dropped; /* the transmit queue was full */
};

/*
 * Choose what gets sent. Takes effect on the next batch.
 *
 * Packed 24-bit is the native width of the converter, so it loses nothing
 * and costs a quarter less bandwidth than the 32-bit form - at 1 kSPS that
 * is 24 kB/s rather than 32. Worth having on a radio link.
 */
void stream_set_encoding(uint8_t encoding);

/* The encoding currently in use. */
uint8_t stream_encoding(void);

/* Start and stop transmitting. */
void stream_enable(bool on);
bool stream_enabled(void);

/*
 * Send one encoded protocol frame on every connected link. Safe from any
 * thread: the IMU sends its motion frames through here too. Returns true if
 * at least one link took the frame.
 */
bool stream_send(const uint8_t *frame, size_t len);

/* Install as the pipeline's sink. Runs in the DSP thread. */
void stream_on_sample(const struct eeg_sample *s);

/* Unsolicited event ids: the first byte of an EVT payload. */
#define STREAM_EVT_MAINS 0x01u

/*
 * Install as the pipeline's mains sink. While the stream is on, sends an EVT
 * frame, little-endian:
 *
 *   u8  STREAM_EVT_MAINS
 *   u8  flags   bit 0: the notch moved to this frequency
 *   u32 seq     the first sample processed after the measurement
 *   f32 hz      the mains frequency measured
 *
 * Runs in the DSP thread.
 */
void stream_on_mains(float hz, uint32_t seq, bool moved);

void stream_get_stats(struct stream_stats *out);
void stream_reset_stats(void);

#endif /* SWIFTEEG_TRANSPORT_STREAM_H */
