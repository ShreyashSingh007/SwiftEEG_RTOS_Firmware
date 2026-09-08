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
 * Stream real samples through the DMA path.
 *
 * One PPI channel now carries the DRDY edge to two tasks: the timer capture
 * that timestamps the sample, and the SPI transfer that fetches it. The CPU
 * does nothing until the 27 bytes are already in RAM.
 */
#define STREAM_TEST_MS 1000

/*
 * Frames to discard before measuring. The first transfer is started by the
 * first DRDY after RDATAC is entered, and the part has not necessarily put a
 * complete sample on the bus by then - that frame reads back as zeros and
 * would otherwise sit in the noise figure as a large fake excursion.
 */
#define STREAM_WARMUP 10

static volatile uint32_t sf_count;
static volatile uint32_t sf_total;
static volatile uint32_t sf_bad_status;
static volatile uint64_t sf_first_us;
static volatile uint64_t sf_last_us;
static volatile uint32_t sf_min_gap;
static volatile uint32_t sf_max_gap;
static volatile int32_t sf_ch1_min;
static volatile int32_t sf_ch1_max;
static volatile uint32_t sf_last_status;

/* Sign-extend one 24-bit big-endian channel word. */
static int32_t decode_ch(const uint8_t *p)
{
	const uint32_t raw = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];

	return (raw & 0x800000u) ? (int32_t)(raw | 0xFF000000u) : (int32_t)raw;
}

static void on_frame(const uint8_t *frame, uint64_t ts_us)
{
	sf_last_status = ((uint32_t)frame[0] << 16) | ((uint32_t)frame[1] << 8) |
			 frame[2];

	sf_total++;

	if (sf_total <= STREAM_WARMUP) {
		sf_last_us = ts_us;
		return;
	}

	if (sf_count == 0) {
		sf_first_us = ts_us;
		sf_ch1_min = INT32_MAX;
		sf_ch1_max = INT32_MIN;
	} else {
		const uint32_t gap = (uint32_t)(ts_us - sf_last_us);

		if (gap < sf_min_gap) {
			sf_min_gap = gap;
		}
		if (gap > sf_max_gap) {
			sf_max_gap = gap;
		}
	}

	/*
	 * The status word's top four bits are hard-wired to 1100. Anything
	 * else means the frame is misaligned, which is the failure mode to
	 * watch for when a transfer is started by hardware rather than code.
	 */
	if ((frame[0] & 0xF0u) != 0xC0u) {
		sf_bad_status++;
	}

	const int32_t ch1 = decode_ch(&frame[ADS1299_STATUS_BYTES]);

	if (ch1 < sf_ch1_min) {
		sf_ch1_min = ch1;
	}
	if (ch1 > sf_ch1_max) {
		sf_ch1_max = ch1;
	}

	sf_last_us = ts_us;
	sf_count++;
}

static void report_stream(void)
{
	if (!afe_present) {
		return;
	}

	if (capture_attach_task(ads1299_start_task_addr()) != 0) {
		LOG_ERR("could not attach the SPI start task to DRDY");
		return;
	}

	sf_count = 0;
	sf_total = 0;
	sf_bad_status = 0;
	sf_min_gap = UINT32_MAX;
	sf_max_gap = 0;

	if (ads1299_stream_start(on_frame) != 0) {
		LOG_ERR("stream start failed");
		return;
	}

	k_msleep(STREAM_TEST_MS);
	ads1299_stream_stop();

	const uint32_t n = sf_count;

	if (n < 2) {
		LOG_ERR("stream: %u frames in %d ms - DRDY is not starting the "
			"SPI transfer", n, STREAM_TEST_MS);
		return;
	}

	const uint32_t mean_us = (uint32_t)((sf_last_us - sf_first_us) / (n - 1));

	LOG_INF("stream: %u frames in %d ms (%u measured) -> %u SPS "
		"(gap mean %u us, min %u, max %u)",
		sf_total, STREAM_TEST_MS, n,
		mean_us ? (TIMEBASE_HZ / mean_us) : 0,
		mean_us, sf_min_gap, sf_max_gap);

	LOG_INF("stream: status word 0x%06x, bad %u, overruns %u",
		sf_last_status, sf_bad_status, ads1299_overruns());

	/*
	 * Channels sit at their reset setting, which is gain 24 with the
	 * inputs shorted internally - so this is a noise-floor reading, not
	 * EEG. LSB is 4.5 V / (24 * 2^23), about 22.35 nV.
	 */
	const int32_t span = sf_ch1_max - sf_ch1_min;

	LOG_INF("stream: ch1 shorted-input noise %d counts p-p (~%d nV), "
		"min %d max %d", span, span * 2235 / 100, sf_ch1_min, sf_ch1_max);

	if (sf_bad_status != 0) {
		LOG_WRN("%u frames had a bad status word - transfers are "
			"misaligned with DRDY", sf_bad_status);
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
	report_stream();

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
