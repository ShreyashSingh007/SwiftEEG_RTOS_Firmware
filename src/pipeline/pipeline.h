/*
 * Acquisition pipeline: interrupt -> ring -> DSP thread.
 *
 *   DRDY --PPI--> SPI DMA --END ISR--> raw ring --> DSP thread --> sinks
 *
 * The interrupt does as little as possible: it copies the finished frame and
 * its hardware timestamp into a lock-free ring and returns. Everything that
 * can take time - decoding, DC removal, filtering - happens in a thread, so
 * a slow consumer can never delay acquisition. If the thread falls behind,
 * the ring drops the newest frames and counts them, rather than stalling the
 * interrupt or corrupting samples already queued.
 */
#ifndef SWIFTEEG_PIPELINE_H
#define SWIFTEEG_PIPELINE_H

#include <stdbool.h>
#include <stdint.h>

#include "afe/ads1299.h"

/* Sample flags, mirroring the protocol's DATA header. */
#define EEG_FLAG_SETTLING 0x01 /* filters have not settled yet */
#define EEG_FLAG_OVERRUN  0x02 /* frames were dropped before this one */

/* One processed sample set, in microvolts. */
struct eeg_sample {
	uint64_t ts_us;                     /* latched in hardware at DRDY */
	uint32_t seq;
	uint32_t status;                    /* AFE status word, lead-off bits */
	float    ch_uv[ADS1299_CHANNELS];
	uint8_t  flags;
};

struct pipeline_stats {
	uint32_t frames;      /* frames the interrupt handed over */
	uint32_t processed;   /* samples the DSP thread completed */
	uint32_t ring_drops;  /* frames lost because the thread fell behind */
	uint32_t bad_status;  /* frames whose status word was malformed */
	uint32_t dsp_mean_us; /* mean time to process one frame */
	uint32_t dsp_max_us;  /* worst case */
	float    ch1_min_uv;
	float    ch1_max_uv;
};

/*
 * Start acquiring at the given ADS1299_DR_* rate. Configures the AFE, wires
 * DRDY to the DMA transfer, and starts the DSP thread.
 */
int pipeline_start(uint8_t rate);

/* Stop acquiring. Safe to call when already stopped. */
void pipeline_stop(void);

/* Zero the counters and begin a fresh measurement window. */
void pipeline_reset_stats(void);

void pipeline_get_stats(struct pipeline_stats *out);

#endif /* SWIFTEEG_PIPELINE_H */
