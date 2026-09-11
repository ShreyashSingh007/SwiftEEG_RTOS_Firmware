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
#include "imu/imu.h"
#include "pipeline/capture.h"
#include "pipeline/pipeline.h"
#include "transport/command.h"
#include "transport/stream.h"
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
 * The motion sensor. Needs the timebase running first: its samples are
 * timestamped on TIMER1, like the EEG's.
 */
static void report_imu(void)
{
	const int err = imu_init();

	if (err == -ENODEV) {
		LOG_INF("IMU: not fitted on this board");
	} else if (err != 0) {
		LOG_ERR("IMU init failed (%d)", err);
	}
}

/*
 * Start acquisition and leave it running.
 *
 * Samples go out over USB as protocol DATA frames, in raw ADC counts by
 * default - which is what the host needs to check the AFE against its own
 * test generator, since a known amplitude in counts is only known before
 * anything filters it.
 *
 * Streaming starts switched off. The host turns it on, so a device sitting
 * on a bench with nobody listening is not filling a buffer nobody reads.
 */
static void start_acquisition(void)
{
	if (!afe_present) {
		LOG_INF("acquisition skipped: no AFE on this board");
		return;
	}

	pipeline_set_sink(stream_on_sample);

	if (pipeline_start(ADS1299_DR_250SPS) != 0) {
		LOG_ERR("pipeline failed to start");
		return;
	}

	if (command_init() != 0) {
		LOG_WRN("command handler unavailable");
	}

	LOG_INF("acquiring at 250 SPS; waiting for the host to start the stream");
}

/* Periodic health line, so a long run leaves some evidence in the log. */
static void report_health(void)
{
	if (afe_present) {
		struct pipeline_stats ps;
		struct stream_stats ss;

		pipeline_get_stats(&ps);
		stream_get_stats(&ss);

		LOG_INF("health: %u samples, %u dropped, %u bad; stream %u frames, "
			"%u samples, %u bytes lost; DSP %u us",
			ps.processed, ps.ring_drops, ps.bad_status,
			ss.frames_sent, ss.samples_sent, ss.bytes_dropped,
			ps.dsp_mean_us);
	}

	if (imu_present()) {
		struct imu_stats is;

		imu_get_stats(&is);

		/* The period in microseconds, to three decimals. */
		LOG_INF("imu: %u samples, %u frames, %u overruns, %u unpaired, "
			"%u timed by poll; period %u.%03u us",
			is.samples, is.frames, is.overruns, is.unpaired,
			is.estimated, is.period_us_q8 >> 8,
			((is.period_us_q8 & 0xFFu) * 1000u) >> 8);
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
	report_imu();

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

	start_acquisition();

	LOG_INF("Entering blink loop");

	uint32_t ticks = 0;

	while (1) {
		(void)leds_toggle(LED_YELLOW);
		(void)leds_toggle(LED_BLUE);
		k_msleep(BLINK_PERIOD_MS);

		/* Roughly every 10 s at a 500 ms blink. */
		if ((afe_present || imu_present()) && (++ticks % 20u) == 0u) {
			report_health();
		}
	}

	return 0;
}
