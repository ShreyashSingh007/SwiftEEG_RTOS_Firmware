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
	int32_t  ch_raw[ADS1299_CHANNELS];  /* ADC counts, before any DSP */
	float    ch_uv[ADS1299_CHANNELS];   /* microvolts, after the chain */
	uint8_t  flags;
};

/*
 * Called from the DSP thread for every completed sample. Keep it short: it
 * runs in the acquisition path, ahead of the next frame.
 */
typedef void (*pipeline_sink_t)(const struct eeg_sample *s);

/* Install the sink. Pass NULL to detach. Set before pipeline_start(). */
void pipeline_set_sink(pipeline_sink_t sink);

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

/*
 * Change the sample rate, restarting acquisition around it. The filters are
 * redesigned for the new rate - a notch is only at 50 Hz for the rate it was
 * designed at, so carrying the old coefficients over would quietly move it.
 *
 * `sps` is the real rate, not a register code; 250 to 1000 are supported
 * over BLE, higher needs USB. Returns 0, or a negative errno.
 */
int pipeline_set_rate(uint16_t sps);

/* The rate currently running, in samples per second. */
uint16_t pipeline_rate(void);

/* Zero the counters and begin a fresh measurement window. */
void pipeline_reset_stats(void);

void pipeline_get_stats(struct pipeline_stats *out);

#endif /* SWIFTEEG_PIPELINE_H */
