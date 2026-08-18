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
