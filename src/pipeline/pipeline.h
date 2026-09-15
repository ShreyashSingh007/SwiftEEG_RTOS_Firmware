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
#include "dsp/dsp.h"

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
 * Sections a host loaded are dropped for the same reason: the pre stage goes
 * back to the device's own notch and the post stage empties. The common
 * average settings are kept, since they do not depend on the rate, and so is
 * everything set on the AFE: channel gains and inputs, bias drive, lead-off.
 *
 * `sps` is the real rate, not a register code; 250 to 1000 are supported
 * over BLE, higher needs USB. Returns 0, or a negative errno.
 */
int pipeline_set_rate(uint16_t sps);

/* The rate currently running, in samples per second. */
uint16_t pipeline_rate(void);

/*
 * Filter changes land between two samples, never inside one: the DSP thread
 * applies each at the top of the next sample. Every call below reports, in
 * `applied_seq` (may be NULL), the sequence number of the first sample
 * processed with the change, so a host can line its own records up with the
 * device's exactly. They return -ENODEV when there is no chain to change.
 */

/*
 * Set the device's own mains notch: 50, 60, or 0 to remove it.
 *
 * Replaces the pre stage with that single notch, including any sections a
 * host loaded there, and starts it again, primed on its next sample.
 */
int pipeline_set_notch(uint8_t hz);

/* The device notch's frequency, or 0 when the pre stage holds something else. */
uint8_t pipeline_notch(void);

#define PIPELINE_STAGE_PRE  0u /* before the common average */
#define PIPELINE_STAGE_POST 1u /* after it */

/*
 * Load a stage with sections a host designed. With `keep_state` and the
 * same number of sections, the filter is retuned in place; otherwise it
 * starts again, primed on its next sample. Returns -EINVAL, changing
 * nothing, for an invalid section.
 */
int pipeline_set_stage(uint8_t stage, const dsp_section_t *sections,
		       uint8_t count, bool keep_state, uint32_t *applied_seq);

/* Common average reference on or off, and which channels make the average. */
int pipeline_set_car(bool enable, uint8_t mask, uint32_t *applied_seq);

/*
 * Restart the chain from the next sample: the DC estimate and every filter
 * re-prime on it. What a host needs to run its own copy of the chain
 * from exactly the same starting point.
 */
int pipeline_reset_chain(uint32_t *applied_seq);

/*
 * Take every channel's gain from what the AFE driver last wrote, and scale
 * the chain to match. Call after anything that changes a gain, or microvolts
 * from the chain are wrong by the ratio of the old gain to the new.
 */
int pipeline_sync_gains(void);

/* What the chain is doing, for reporting to a host. */
struct pipeline_filters {
	dsp_section_t pre[DSP_MAX_SECTIONS];
	dsp_section_t post[DSP_MAX_SECTIONS];
	uint8_t pre_count;
	uint8_t post_count;
	bool    pre_is_notch; /* the pre stage is the device's own notch */
	bool    car;
	uint8_t car_mask;
};

void pipeline_get_filters(struct pipeline_filters *out);

/* Zero the counters and begin a fresh measurement window. */
void pipeline_reset_stats(void);

void pipeline_get_stats(struct pipeline_stats *out);

#endif /* SWIFTEEG_PIPELINE_H */
