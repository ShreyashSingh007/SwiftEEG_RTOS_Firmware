#include "pipeline.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "capture.h"
#include "chain.h"
#include "dsp/dsp.h"
#include "sys/ringbuf.h"
#include "timebase/timebase.h"

LOG_MODULE_REGISTER(pipeline, CONFIG_LOG_DEFAULT_LEVEL);

BUILD_ASSERT(PIPELINE_STAGE_PRE == CHAIN_STAGE_PRE &&
	     PIPELINE_STAGE_POST == CHAIN_STAGE_POST,
	     "pipeline and chain disagree on stage numbers");

/*
 * Raw frames buffered between the interrupt and the DSP thread.
 *
 * 128 frames is half a second at 250 SPS and 8 ms at 16 kSPS. The thread
 * only has to keep up on average; this absorbs the bursts when something
 * higher priority - the radio, most likely - holds the CPU for a while.
 */
#define RAW_RING_FRAMES 128

/*
 * Mains notch, the pre stage's default. Q 12 is about 4 Hz wide. Mains is
 * never where the nameplate says - measured at 49.6 Hz here - and a Q 30
 * notch aimed at 50.0 takes only 7 dB off that. The host application
 * defaults to Q 12 for the same reason.
 */
#define NOTCH_Q 12.0f

/* The part's internal reference. */
#define VREF_VOLTS 4.5f

#define DSP_STACK_SIZE 2048
#define DSP_PRIORITY   2

/*
 * How long a filter change waits for the DSP thread to take it. A sample
 * arrives every 4 ms at 250 SPS, so this only runs out when acquisition has
 * stalled.
 */
#define REQ_TIMEOUT_MS 500

/* The longest a restart is flagged as settling, in seconds. */
#define SETTLE_MAX_S 60u

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

/*
 * Filter settings that outlive a restart. The device notch and the common
 * average do not depend on the rate; sections a host loaded do, and are
 * dropped with it.
 */
static float notch_hz = 50.0f;
static bool car_on;
static uint8_t car_mask = 0xFFu;

/* Whether the pre stage holds the device notch rather than host sections. */
static bool pre_is_notch = true;

/* A chain has been built, so there is something to change. */
static bool chain_ready;

/* Samples still to be flagged as settling. */
static uint32_t settle_left;

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

/* --- filter changes -------------------------------------------------------- */

/*
 * A filter change, handed from the command thread to the DSP thread.
 *
 * It has to land between two samples. Applied from the command thread
 * directly, it could land halfway through one - sections half copied while a
 * channel is being filtered - so it is posted here instead, and the DSP
 * thread takes it at the top of its next sample and records which sample
 * that was. One change at a time; whoever posts it waits for it.
 */
enum req_kind {
	REQ_STAGE,
	REQ_CAR,
	REQ_NOTCH,
	REQ_GAINS,
	REQ_RESET,
};

struct chain_req {
	enum req_kind kind;
	uint8_t stage;
	uint8_t count;
	bool keep_state;
	dsp_section_t sections[DSP_MAX_SECTIONS];
	bool car;
	uint8_t mask;
	float notch_hz;
	uint8_t gains[FRAME_CHANNELS];

	/* Filled in by whichever thread applied it. */
	int result;
	uint32_t seq;
};

#define REQ_IDLE   0
#define REQ_POSTED 1
#define REQ_TAKEN  2

static struct chain_req req;
static atomic_t req_state = ATOMIC_INIT(REQ_IDLE);
static K_SEM_DEFINE(req_done, 0, 1);
static K_MUTEX_DEFINE(req_lock);

static void arm_settling(void)
{
	const uint32_t cap = (uint32_t)sample_rate_hz * SETTLE_MAX_S;
	const uint32_t n = chain_settle_samples(&chain);

	settle_left = (n < cap) ? n : cap;
}

static void apply_request(struct chain_req *r)
{
	bool restarted = false;

	switch (r->kind) {
	case REQ_STAGE: {
		bool kept = false;

		r->result = chain_set_stage(&chain, r->stage, r->sections,
					    r->count, r->keep_state, &kept);
		if (r->result == 0) {
			restarted = !kept;
			if (r->stage == CHAIN_STAGE_PRE) {
				pre_is_notch = false;
			}
		}
		break;
	}

	case REQ_CAR:
		chain_set_car(&chain, r->car, r->mask);
		r->result = 0;
		break;

	case REQ_NOTCH:
		r->result = chain_set_notch(&chain, sample_rate_hz, r->notch_hz,
					    NOTCH_Q);
		if (r->result == 0) {
			pre_is_notch = true;
			restarted = true;
		}
		break;

	case REQ_GAINS:
		r->result = 0;
		for (uint8_t ch = 0; ch < FRAME_CHANNELS; ch++) {
			if (chain_set_gain(&chain, ch, r->gains[ch]) != 0) {
				r->result = -EINVAL;
			}
		}
		break;

	case REQ_RESET:
		chain_reset(&chain);
		r->result = 0;
		restarted = true;
		break;

	default:
		r->result = -EINVAL;
		break;
	}

	/* The next sample processed is the first to see the change. */
	r->seq = st_seq;

	if (restarted) {
		arm_settling();
	}
}

static int submit(const struct chain_req *r, uint32_t *applied_seq)
{
	if (!chain_ready) {
		return -ENODEV;
	}

	k_mutex_lock(&req_lock, K_FOREVER);

	req = *r;

	int err;

	if (!running) {
		/* No DSP thread to race with: apply it here. */
		apply_request(&req);
		err = req.result;
	} else {
		k_sem_reset(&req_done);
		atomic_set(&req_state, REQ_POSTED);

		if (k_sem_take(&req_done, K_MSEC(REQ_TIMEOUT_MS)) == 0) {
			err = req.result;
		} else if (atomic_cas(&req_state, REQ_POSTED, REQ_IDLE)) {
			/* Withdrawn before the DSP thread saw it: never applied. */
			err = -ETIMEDOUT;
		} else {
			/* Taken just as the wait ran out; it is being applied. */
			(void)k_sem_take(&req_done, K_FOREVER);
			err = req.result;
		}
	}

	if (err == 0 && applied_seq != NULL) {
		*applied_seq = req.seq;
	}

	k_mutex_unlock(&req_lock);
	return err;
}

/* --- acquisition ------------------------------------------------------------ */

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
	/*
	 * A filter change from the command thread goes in before this sample,
	 * so every sample is filtered wholly one way or the other.
	 */
	if (atomic_cas(&req_state, REQ_POSTED, REQ_TAKEN)) {
		apply_request(&req);
		atomic_set(&req_state, REQ_IDLE);
		k_sem_give(&req_done);
	}

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
	 * and the filter state for everything after it.
	 */
	if (!chain_process(&chain, rf->data, out.ch_raw, out.ch_uv)) {
		st_bad_status++;
		st_seq--; /* it never became a sample */
		return;
	}

	if (settle_left != 0u) {
		settle_left--;
		out.flags |= EEG_FLAG_SETTLING;
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

/*
 * DC removal corner. The estimator is a leaky integrator whose corner sits
 * near fs / (2*pi * 2^shift), so the shift grows with the rate to hold the
 * corner near 0.08 Hz at every rate. A fixed shift put it at 0.31 Hz at
 * 1000 SPS - inside the 0.1-0.3 Hz range where high-pass filtering starts
 * distorting slow ERP components.
 *
 * Its job is only to strip the electrode half-cell offset so the signal fits
 * in a float32 mantissa, not to shape the band.
 */
static uint8_t dc_shift_for(uint16_t sps)
{
	uint8_t shift = 9; /* 250 SPS */

	for (uint32_t r = 250u; r < sps && shift < 15u; r <<= 1) {
		shift++;
	}

	return shift;
}

/* CHnSET bits 6:4 to the gain they select; 0 for the reserved code. */
static uint8_t gain_from_chset(uint8_t chset)
{
	static const uint8_t gains[8] = { 1, 2, 4, 6, 8, 12, 24, 0 };

	return gains[(chset >> 4) & 0x07u];
}

static int build_chain(void)
{
	chain_ready = false;

	const uint8_t shift = dc_shift_for((uint16_t)sample_rate_hz);
	int err = chain_init(&chain, sample_rate_hz, shift, notch_hz, NOTCH_Q,
			     VREF_VOLTS, 24);

	if (err) {
		LOG_ERR("filter design failed for fs %d Hz (%d)",
			(int)sample_rate_hz, err);
		return err;
	}

	/* The gains the channels are set to, not the ones assumed. */
	uint8_t chset[ADS1299_CHANNELS];

	ads1299_get_channels_cached(chset, ADS1299_CHANNELS);
	for (uint8_t ch = 0; ch < ADS1299_CHANNELS; ch++) {
		(void)chain_set_gain(&chain, ch, gain_from_chset(chset[ch]));
	}

	chain_set_car(&chain, car_on, car_mask);
	pre_is_notch = true;
	arm_settling();
	chain_ready = true;

	return 0;
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

	LOG_INF("pipeline running: %d SPS, LSB %d nV, notch %d Hz, DC shift %u "
		"(timing overhead %u us)",
		(int)sample_rate_hz, (int)(chain.lsb_uv[0] * 1000.0f),
		(int)notch_hz, dc_shift_for((uint16_t)sample_rate_hz),
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

int pipeline_set_notch(uint8_t hz)
{
	if (hz != 0 && hz != 50 && hz != 60) {
		return -EINVAL;
	}

	notch_hz = (float)hz;

	if (!chain_ready) {
		return 0; /* takes effect when the chain is built */
	}

	/*
	 * Rebuilt in place rather than by restarting acquisition: a notch
	 * change is a filter change, not a hardware one, and there is no
	 * reason to drop samples for it.
	 */
	const struct chain_req r = { .kind = REQ_NOTCH, .notch_hz = notch_hz };
	const int err = submit(&r, NULL);

	if (err == 0) {
		LOG_INF("notch now %u Hz", hz);
	}

	return err;
}

uint8_t pipeline_notch(void)
{
	return pre_is_notch ? (uint8_t)notch_hz : 0u;
}

int pipeline_set_stage(uint8_t stage, const dsp_section_t *sections,
		       uint8_t count, bool keep_state, uint32_t *applied_seq)
{
	if (stage > PIPELINE_STAGE_POST || count > DSP_MAX_SECTIONS ||
	    (count != 0u && sections == NULL)) {
		return -EINVAL;
	}

	struct chain_req r = {
		.kind = REQ_STAGE,
		.stage = stage,
		.count = count,
		.keep_state = keep_state,
	};

	if (count != 0u) {
		memcpy(r.sections, sections, (size_t)count * sizeof(*sections));
	}

	return submit(&r, applied_seq);
}

int pipeline_set_car(bool enable, uint8_t mask, uint32_t *applied_seq)
{
	const struct chain_req r = { .kind = REQ_CAR, .car = enable, .mask = mask };
	const int err = submit(&r, applied_seq);

	if (err == 0) {
		car_on = enable;
		car_mask = mask;
	}

	return err;
}

int pipeline_reset_chain(uint32_t *applied_seq)
{
	const struct chain_req r = { .kind = REQ_RESET };

	return submit(&r, applied_seq);
}

int pipeline_sync_gains(void)
{
	uint8_t chset[ADS1299_CHANNELS];

	ads1299_get_channels_cached(chset, ADS1299_CHANNELS);

	struct chain_req r = { .kind = REQ_GAINS };

	for (uint8_t ch = 0; ch < ADS1299_CHANNELS; ch++) {
		r.gains[ch] = gain_from_chset(chset[ch]);
	}

	return submit(&r, NULL);
}

void pipeline_get_filters(struct pipeline_filters *out)
{
	memset(out, 0, sizeof(*out));

	if (!chain_ready) {
		return;
	}

	/*
	 * Only the command thread changes filters, and it waits for each change
	 * to land, so nothing here can be mid-change while it is read.
	 */
	out->pre_count = chain.pre.count;
	out->post_count = chain.post.count;
	memcpy(out->pre, chain.pre.sections, sizeof(out->pre));
	memcpy(out->post, chain.post.sections, sizeof(out->post));
	out->pre_is_notch = pre_is_notch;
	out->car = chain.car;
	out->car_mask = chain.car_mask;
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

	/*
	 * Not running here means the last restart failed: nothing else stops
	 * acquisition once it has begun. So start regardless. Returning early
	 * for it - "takes effect at the next start" - left a board whose
	 * restart had failed ignoring every rate change after it until reset,
	 * because no next start ever came.
	 */
	pipeline_stop();

	int err = pipeline_start(code);

	if (err) {
		LOG_WRN("restart at %u SPS failed (%d), trying once more", sps, err);
		err = pipeline_start(code);
	}

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
