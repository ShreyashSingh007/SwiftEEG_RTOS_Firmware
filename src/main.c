/*
 * SwiftEEG firmware - M1 bring-up.
 *
 * Proves the toolchain, board port, flashing and RTT logging, and reports
 * what hardware is actually fitted. Acquisition, DSP and transports arrive
 * in M2.
 *
 * The AFE is deliberately allowed to be missing: two of the three boards are
 * built without an ADS1299, and the firmware must run identically on all of
 * them rather than halting on absent hardware.
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "afe/ads1299.h"
#include "board/leds.h"
#include "board/supply.h"
#include "pipeline/capture.h"
#include "pipeline/pipeline.h"
#include "timebase/timebase.h"
#include "transport/ble.h"
#include "transport/usb.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Blink the two LEDs out of phase so a stuck pin is obvious at a glance. */
#define BLINK_PERIOD_MS 500

static void report_supply(void)
{
	int mv = supply_read_vdd_mv();

	if (mv < 0) {
		LOG_WRN("VDD read failed (%d)", mv);
		return;
	}

	LOG_INF("VDD rail: %d mV", mv);

	/* Prove the reading rather than trusting a single gain setting. */
	supply_selftest();

	/*
	 * LP5907 is a 3.3 V part. A reading well under that means either the
	 * cell has sagged into the regulator's dropout or the rail is loaded
	 * down - both worth knowing, and both affect LED brightness.
	 */
	if (mv < 3000) {
		LOG_WRN("VDD is low - expected ~3300 mV from the LP5907");
	}
}

static bool afe_present;

static void report_afe(void)
{
	struct afe_probe probe;

	if (ads1299_probe(&probe) != 0) {
		LOG_ERR("AFE probe failed to run");
		return;
	}

	switch (probe.result) {
	case AFE_PRESENT:
		afe_present = true;
		LOG_INF("AFE ADS1299: present, ID 0x%02x (8 channels)",
			probe.raw_id);
		if (ads1299_start_pin_stuck_high()) {
			LOG_ERR("AFE START pin reads HIGH - conversions cannot "
				"be stopped. Tie U5 pin 38 to DGND.");
		} else {
			LOG_INF("AFE START pin OK (STOP honoured)");
		}
		break;

	case AFE_ABSENT:
		/* Expected on the boards built without an ADS1299. */
		LOG_INF("AFE ADS1299: %s", afe_probe_str(probe.result));
		break;

	default:
		LOG_WRN("AFE ADS1299: %s (raw ID 0x%02x)",
			afe_probe_str(probe.result), probe.raw_id);
		break;
	}
}

/*
 * Bring up the sample timebase and prove it counts at the right rate.
 *
 * The check is against the kernel clock, which runs from the 32.768 kHz
 * crystal, while the timebase counts HFCLK. Two independent crystals: if
 * they agree the pair is trustworthy, and a disagreement points at whichever
 * one is running off its internal RC oscillator instead.
 */
static void report_timebase(void)
{
	if (timebase_init() != 0) {
		LOG_WRN("timebase unavailable");
		return;
	}

	const uint32_t c0 = k_cycle_get_32();
	const uint64_t t0 = timebase_now_us();

	k_msleep(200);

	const uint64_t t1 = timebase_now_us();
	const uint32_t c1 = k_cycle_get_32();

	const uint64_t tb_us = t1 - t0;
	const uint64_t ref_us = k_cyc_to_us_near64(c1 - c0);

	if (ref_us == 0) {
		LOG_WRN("timebase check: reference clock did not advance");
		return;
	}

	const int64_t delta_us = (int64_t)tb_us - (int64_t)ref_us;

	LOG_INF("timebase: %llu us vs %llu us reference, delta %lld us, HFXO %s",
		tb_us, ref_us, delta_us,
		timebase_hfxo_running() ? "on" : "off");

	/*
	 * This is a coarse check, not a calibration. The reference ticks at
	 * 32.768 kHz, so it quantises to 30.5 us - about 150 ppm over this
	 * window, which swamps the ~40 ppm a pair of crystals would show.
	 *
	 * What it does catch is a clock running off an internal RC oscillator
	 * instead of its crystal, which is off by 1-2 % - two orders of
	 * magnitude larger. Flag at 1 %. The true sample rate gets measured
	 * properly later, by fitting many DRDY captures.
	 */
	if (delta_us > (int64_t)ref_us / 100 || -delta_us > (int64_t)ref_us / 100) {
		LOG_WRN("timebase disagrees with the reference by more than 1 %% - "
			"one of the two is running on its internal RC oscillator");
	}
}

/*
 * Prove the hardware capture path end to end.
 *
 * Configures the AFE, lets it convert for a second, and reports what the
 * DRDY intervals looked like. Nothing is read over SPI here on purpose: this
 * isolates the timestamp path, so a failure points at DRDY, GPIOTE or PPI
 * rather than at the data transfer that comes next.
 *
 * The AFE runs from its own oscillator, so the measured rate is expected to
 * sit near the nominal one rather than on it. That offset is the thing the
 * drift estimator will track.
 */
#define CAPTURE_TEST_MS   1000
#define CAPTURE_NOMINAL_SPS 250

static void report_capture(void)
{
	if (!afe_present) {
		LOG_INF("capture test skipped: no AFE on this board");
		return;
	}

	if (ads1299_configure(ADS1299_DR_250SPS) != 0) {
		LOG_ERR("AFE configuration failed, skipping capture test");
		return;
	}

	if (capture_init() != 0) {
		LOG_ERR("capture path unavailable");
		return;
	}

	if (ads1299_start_conversions() != 0) {
		LOG_ERR("AFE START failed");
		return;
	}

	/*
	 * Poll rather than take interrupts. The capture is done in hardware,
	 * so watching the register move tests DRDY, GPIOTE and PPI on their
	 * own - and it does not care who owns the GPIOTE interrupt.
	 */
	struct capture_stats st;

	capture_measure(CAPTURE_TEST_MS, &st);

	(void)ads1299_stop_conversions();

	if (st.count < 2) {
		LOG_ERR("capture test: %u DRDY edges in %d ms - the AFE is not "
			"converting, or DRDY is not reaching P0.04",
			st.count, CAPTURE_TEST_MS);
		return;
	}

	/* Mean interval over the window, in microseconds. */
	const uint32_t mean_us = (uint32_t)(st.total_us / (st.count - 1));
	const uint32_t sps = (mean_us != 0) ? (TIMEBASE_HZ / mean_us) : 0;

	LOG_INF("capture: %u edges in %d ms -> %u SPS "
		"(interval mean %u us, min %u, max %u)",
		st.count, CAPTURE_TEST_MS, sps, mean_us, st.min_us, st.max_us);

	/*
	 * Jitter is the interesting number. The timestamp is latched in
	 * hardware, so spread here is the AFE's own oscillator plus any
	 * missed edges - not interrupt latency.
	 */
	const uint32_t spread = st.max_us - st.min_us;

	if (spread > mean_us / 10) {
		LOG_WRN("DRDY interval spread %u us is over 10 %% of the mean - "
			"edges are being missed or the AFE is not steady",
			spread);
	}

	if (sps < CAPTURE_NOMINAL_SPS * 9 / 10 ||
	    sps > CAPTURE_NOMINAL_SPS * 11 / 10) {
		LOG_WRN("measured %u SPS is far from the nominal %d SPS",
			sps, CAPTURE_NOMINAL_SPS);
	}
}

/*
 * Run the acquisition pipeline and report what came out.
 *
 * This is the whole chain: DRDY starts the DMA transfer in hardware, the
 * transfer-complete interrupt drops the frame into a lock-free ring, and a
 * thread decodes, removes DC in the integer domain, scales to microvolts
 * and runs the mains notch.
 */
#define PIPELINE_TEST_MS 1000

static void report_pipeline(void)
{
	if (!afe_present) {
		LOG_INF("pipeline test skipped: no AFE on this board");
		return;
	}

	if (pipeline_start(ADS1299_DR_250SPS) != 0) {
		LOG_ERR("pipeline failed to start");
		return;
	}

	k_msleep(PIPELINE_TEST_MS);

	struct pipeline_stats st;

	pipeline_get_stats(&st);
	pipeline_stop();

	if (st.processed == 0) {
		LOG_ERR("pipeline: no samples processed");
		return;
	}

	LOG_INF("pipeline: %u frames, %u processed, %u dropped, %u bad status",
		st.frames, st.processed, st.ring_drops, st.bad_status);

	/*
	 * Cost per sample decides which rates are reachable, so report it as
	 * a load rather than leaving it to be worked out. Tenths of a percent
	 * because at 250 SPS whole percent rounds to zero and reads as free.
	 *
	 * The 16 kSPS projection is the number that actually constrains the
	 * design: a 62.5 us sample period leaves far less room.
	 */
	const uint32_t period_us = TIMEBASE_HZ / 250U;
	const uint32_t load_tenths = st.dsp_mean_us * 1000U / period_us;
	/* 16 kSPS is a 62.5 us period, so tenths of a percent is us * 16. */
	const uint32_t load_16k = st.dsp_mean_us * 16U;

	LOG_INF("pipeline: DSP %u us/sample mean, %u us worst -> %u.%u %% at "
		"250 SPS, would be %u.%u %% at 16 kSPS",
		st.dsp_mean_us, st.dsp_max_us,
		load_tenths / 10U, load_tenths % 10U,
		load_16k / 10U, load_16k % 10U);

	/* Integer nanovolts: float formatting is not built into the log. */
	LOG_INF("pipeline: ch1 %d to %d nV after DC removal and notch",
		(int)(st.ch1_min_uv * 1000.0f), (int)(st.ch1_max_uv * 1000.0f));

	if (st.ring_drops != 0) {
		LOG_WRN("%u frames dropped - the DSP thread is not keeping up",
			st.ring_drops);
	}
}

int main(void)
{
	LOG_INF("SwiftEEG firmware starting (M1 bring-up)");

	int err = leds_init();
	if (err) {
		LOG_ERR("LED init failed (%d) - halting", err);
		return err;
	}

	report_supply();
	report_afe();
	report_timebase();
	report_capture();
	report_pipeline();

	/*
	 * Transports are brought up but carry no protocol yet - the codec
	 * lands in M2. A failure here is logged and tolerated: a device that
	 * cannot advertise is still useful over USB, and vice versa.
	 */
	if (usb_transport_init() != 0) {
		LOG_WRN("USB transport unavailable");
	}

	/*
	 * Report the USB regulator state. Sampled after a short delay: the
	 * SoC's internal USB regulator takes a moment to come ready after
	 * VBUS is detected, and reading immediately after usbd_enable()
	 * reports OUTPUTRDY=0 and logs a scary warning about a supply that is
	 * in fact perfectly healthy.
	 */
	k_msleep(50);
	supply_log_usb_status();
	if (ble_transport_init() != 0) {
		LOG_WRN("BLE transport unavailable");
	}

	/* Start out of phase. */
	(void)leds_set(LED_YELLOW, true);
	(void)leds_set(LED_BLUE, false);

	LOG_INF("Entering blink loop");

	while (1) {
		(void)leds_toggle(LED_YELLOW);
		(void)leds_toggle(LED_BLUE);
		k_msleep(BLINK_PERIOD_MS);
	}

	return 0;
}
