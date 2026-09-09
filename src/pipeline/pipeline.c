#include "pipeline.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "capture.h"
#include "chain.h"
#include "dsp/dsp.h"
#include "sys/ringbuf.h"
#include "timebase/timebase.h"

LOG_MODULE_REGISTER(pipeline, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Raw frames buffered between the interrupt and the DSP thread.
 *
 * 128 frames is half a second at 250 SPS and 8 ms at 16 kSPS. The thread
 * only has to keep up on average; this absorbs the bursts when something
 * higher priority - the radio, most likely - holds the CPU for a while.
 */
#define RAW_RING_FRAMES 128

/*
 * DC removal corner. The estimator is a leaky integrator whose corner sits
 * near fs / (2*pi * 2^shift): about 0.08 Hz at 250 SPS.
 *
 * Deliberately far below the 0.1-0.3 Hz where high-pass filtering starts
 * distorting slow ERP components - the P300 features a speller depends on.
 * Its job is only to strip the electrode half-cell offset so the signal fits
 * in a float32 mantissa, not to shape the band.
 */
#define DC_SHIFT 9

/* Mains notch. 50 Hz here; 60 Hz becomes a runtime option with the commands. */
#define NOTCH_HZ 50.0f
#define NOTCH_Q  30.0f

#define DSP_STACK_SIZE 2048
#define DSP_PRIORITY   2

struct raw_frame {
	uint64_t ts_us;
	uint8_t  data[ADS1299_FRAME_BYTES];
};

static struct raw_frame raw_storage[RAW_RING_FRAMES];
static spsc_ring_t raw_ring;

static chain_t chain;
static float sample_rate_hz;

static K_THREAD_STACK_DEFINE(dsp_stack, DSP_STACK_SIZE);
static struct k_thread dsp_thread;
static k_tid_t dsp_tid;

static volatile bool running;
static uint8_t current_rate_code;
static struct k_sem frame_ready;

/* Statistics. Written by the DSP thread, read by whoever asks. */
static volatile uint32_t st_frames;
static volatile uint32_t st_processed;
static volatile uint32_t st_bad_status;
static volatile uint32_t st_dsp_total_us;
static volatile uint32_t st_dsp_max_us;
static volatile float st_ch1_min;
static volatile float st_ch1_max;
static uint32_t st_seq;
static pipeline_sink_t sink;

/*
 * Cost of the two timebase reads that bracket process(), measured once at
 * start-up. timebase_now_us() takes a spinlock and does a capture-task round
 * trip, so it is not free, and at these per-sample scales it is a large
 * share of what the naive measurement reports.
 */
static uint32_t timing_overhead_us;

static void measure_timing_overhead(void)
{
	uint32_t best = UINT32_MAX;

	/* Best of several: anything longer was interrupted. */
	for (int i = 0; i < 16; i++) {
		const uint32_t a = (uint32_t)timebase_now_us();
		const uint32_t b = (uint32_t)timebase_now_us();
		const uint32_t d = b - a;

		if (d < best) {
			best = d;
		}
	}

	timing_overhead_us = best;
}

/*
 * Runs in the SPI transfer-complete interrupt. Copy and leave - anything
 * slower here eats into the budget before the next sample arrives, which at
 * 16 kSPS is only 62.5 us.
 */
static void on_frame(const uint8_t *frame, uint64_t ts_us)
{
	struct raw_frame rf;

	rf.ts_us = ts_us;
	memcpy(rf.data, frame, ADS1299_FRAME_BYTES);

	st_frames++;

	if (spsc_push(&raw_ring, &rf)) {
		k_sem_give(&frame_ready);
	}
	/* On failure the ring has already counted the drop. */
}

static void process(const struct raw_frame *rf)
{
	const uint32_t t0 = (uint32_t)timebase_now_us();

	struct eeg_sample out = {
		.ts_us = rf->ts_us,
		.seq = st_seq++,
		.status = frame_status(rf->data),
		.flags = 0,
	};

	/*
	 * chain_process() rejects a frame whose status marker is wrong. That
	 * is not a sample: letting it through would corrupt the DC estimate
	 * and the biquad state for everything after it.
	 */
	if (!chain_process(&chain, rf->data, out.ch_raw, out.ch_uv)) {
		st_bad_status++;
		st_seq--; /* it never became a sample */
		return;
	}

	if (out.ch_uv[0] < st_ch1_min) {
		st_ch1_min = out.ch_uv[0];
	}
	if (out.ch_uv[0] > st_ch1_max) {
		st_ch1_max = out.ch_uv[0];
	}

	if (sink != NULL) {
		sink(&out);
	}

	const uint32_t dt = (uint32_t)timebase_now_us() - t0;

	st_dsp_total_us += dt;
	if (dt > st_dsp_max_us) {
		st_dsp_max_us = dt;
	}

	st_processed++;
}

static void dsp_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	while (running) {
		if (k_sem_take(&frame_ready, K_MSEC(100)) != 0) {
			continue;
		}

		/*
		 * Drain rather than handling one per wake-up: the semaphore
		 * count and the ring can disagree after a drop, and draining
		 * keeps latency down when a burst arrives.
		 */
		const struct raw_frame *rf;

		while ((rf = spsc_peek(&raw_ring)) != NULL) {
			process(rf);
			spsc_release(&raw_ring);
		}
	}
}

static int build_chain(void)
{
	/* VREF is the part's internal 4.5 V; gain 24 is what configure() sets. */
	const int err = chain_init(&chain, sample_rate_hz, DC_SHIFT, NOTCH_HZ,
				   NOTCH_Q, 4.5f, 24);

	if (err) {
		LOG_ERR("filter design failed for fs %d Hz (%d)",
			(int)sample_rate_hz, err);
	}

	return err;
}

int pipeline_start(uint8_t rate)
{
	if (running) {
		return -EALREADY;
	}

	current_rate_code = rate;

	/* Nominal, for filter design. The true rate is measured separately. */
	switch (rate) {
	case ADS1299_DR_250SPS:  sample_rate_hz = 250.0f; break;
	case ADS1299_DR_500SPS:  sample_rate_hz = 500.0f; break;
	case ADS1299_DR_1KSPS:   sample_rate_hz = 1000.0f; break;
	case ADS1299_DR_2KSPS:   sample_rate_hz = 2000.0f; break;
	case ADS1299_DR_4KSPS:   sample_rate_hz = 4000.0f; break;
	case ADS1299_DR_8KSPS:   sample_rate_hz = 8000.0f; break;
	case ADS1299_DR_16KSPS:  sample_rate_hz = 16000.0f; break;
	default: return -EINVAL;
	}

	if (!spsc_init(&raw_ring, raw_storage, sizeof(struct raw_frame),
		       RAW_RING_FRAMES)) {
		return -EINVAL;
	}

	k_sem_init(&frame_ready, 0, K_SEM_MAX_LIMIT);

	int err = ads1299_configure(rate);

	if (err) {
		return err;
	}

	err = build_chain();
	if (err) {
		return err;
	}

	err = capture_init();
	if (err) {
		return err;
	}

	err = capture_attach_task(ads1299_start_task_addr());
	if (err) {
		return err;
	}

	/* Let the AFE driver suspend the hardware trigger when it needs the
	 * bus to itself. */
	ads1299_set_trigger_gate(capture_set_trigger);

	measure_timing_overhead();

	pipeline_reset_stats();
	running = true;

	dsp_tid = k_thread_create(&dsp_thread, dsp_stack, DSP_STACK_SIZE,
				  dsp_entry, NULL, NULL, NULL,
				  DSP_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(dsp_tid, "eeg_dsp");

	err = ads1299_stream_start(on_frame);
	if (err) {
		pipeline_stop();
		return err;
	}

	LOG_INF("pipeline running: %d SPS, LSB %d nV, notch %d Hz "
		"(timing overhead %u us)",
		(int)sample_rate_hz, (int)(chain.lsb_uv * 1000.0f), (int)NOTCH_HZ,
		timing_overhead_us);

	return 0;
}

void pipeline_stop(void)
{
	if (!running) {
		return;
	}

	ads1299_stream_stop();
	running = false;

	/* Wake the thread so it sees the flag rather than waiting out its
	 * timeout. */
	k_sem_give(&frame_ready);

	if (dsp_tid != NULL) {
		(void)k_thread_join(&dsp_thread, K_MSEC(500));
		dsp_tid = NULL;
	}
}

void pipeline_set_sink(pipeline_sink_t s)
{
	sink = s;
}

/* Real rate to the register code the AFE wants. 0xFF if unsupported. */
static uint8_t rate_code_for(uint16_t sps)
{
	switch (sps) {
	case 250:   return ADS1299_DR_250SPS;
	case 500:   return ADS1299_DR_500SPS;
	case 1000:  return ADS1299_DR_1KSPS;
	case 2000:  return ADS1299_DR_2KSPS;
	case 4000:  return ADS1299_DR_4KSPS;
	case 8000:  return ADS1299_DR_8KSPS;
	case 16000: return ADS1299_DR_16KSPS;
	default:    return 0xFFu;
	}
}

uint16_t pipeline_rate(void)
{
	return (uint16_t)sample_rate_hz;
}

int pipeline_set_rate(uint16_t sps)
{
	const uint8_t code = rate_code_for(sps);

	if (code == 0xFFu) {
		return -EINVAL;
	}

	if (running && code == current_rate_code) {
		return 0;
	}

	const bool was_running = running;

	if (was_running) {
		pipeline_stop();
	}

	if (!was_running) {
		return 0; /* takes effect at the next start */
	}

	const int err = pipeline_start(code);

	if (err) {
		LOG_ERR("could not restart at %u SPS (%d)", sps, err);
	} else {
		LOG_INF("sample rate now %u SPS", sps);
	}

	return err;
}

void pipeline_reset_stats(void)
{
	st_frames = 0;
	st_processed = 0;
	st_bad_status = 0;
	st_dsp_total_us = 0;
	st_dsp_max_us = 0;
	st_ch1_min = 1e30f;
	st_ch1_max = -1e30f;
	st_seq = 0;
}

void pipeline_get_stats(struct pipeline_stats *out)
{
	const uint32_t n = st_processed;

	out->frames = st_frames;
	out->processed = n;
	out->ring_drops = spsc_dropped(&raw_ring);
	out->bad_status = st_bad_status;
	/*
	 * Subtract the measurement's own cost so the figure is the work, not
	 * the instrumentation.
	 */
	const uint32_t mean = n ? (st_dsp_total_us / n) : 0;

	out->dsp_mean_us = (mean > timing_overhead_us)
				   ? (mean - timing_overhead_us) : 0;
	out->dsp_max_us = (st_dsp_max_us > timing_overhead_us)
				  ? (st_dsp_max_us - timing_overhead_us) : 0;
	out->ch1_min_uv = (n != 0) ? st_ch1_min : 0.0f;
	out->ch1_max_uv = (n != 0) ? st_ch1_max : 0.0f;
}
