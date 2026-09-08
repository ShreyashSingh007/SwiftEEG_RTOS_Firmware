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

static void report_afe(void)
{
	struct afe_probe probe;

	if (ads1299_probe(&probe) != 0) {
		LOG_ERR("AFE probe failed to run");
		return;
	}

	switch (probe.result) {
	case AFE_PRESENT:
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

	/*
	 * Error in parts per thousand, signed. Crystals should land within a
	 * count or two of zero; the internal RC would show tens.
	 */
	const int32_t err_ppt =
		(int32_t)(((int64_t)tb_us - (int64_t)ref_us) * 1000 / (int64_t)ref_us);

	LOG_INF("timebase: %llu us elapsed vs %llu us reference (%d ppt), HFXO %s",
		tb_us, ref_us, err_ppt,
		timebase_hfxo_running() ? "on" : "off");
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
