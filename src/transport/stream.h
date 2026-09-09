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
#include <stdint.h>

#include "pipeline/pipeline.h"

/* What the samples in a DATA frame look like. */
#define STREAM_ENC_RAW_I32 0u /* ADC counts, before the DSP chain */
#define STREAM_ENC_UV_F32  1u /* microvolts, after the DSP chain */
#define STREAM_ENC_RAW_I24 2u /* ADC counts packed to three bytes */

struct stream_stats {
	uint32_t frames_sent;
	uint32_t samples_sent;
	uint32_t bytes_dropped; /* USB buffer was full */
	uint32_t ble_dropped;   /* notification refused, usually no buffers */
	uint32_t ble_too_big;   /* frame exceeded the negotiated MTU */
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

/* Install as the pipeline's sink. Runs in the DSP thread. */
void stream_on_sample(const struct eeg_sample *s);

void stream_get_stats(struct stream_stats *out);
void stream_reset_stats(void);

#endif /* SWIFTEEG_TRANSPORT_STREAM_H */
