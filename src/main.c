/*
 * SwiftEEG firmware — M1 bring-up.
 *
 * Goal of this milestone: prove the toolchain, board port, flashing and RTT
 * logging all work, and that both LEDs are driven correctly. Acquisition,
 * DSP and transports arrive in M2.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "board/leds.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Blink the two LEDs out of phase so a stuck pin is obvious at a glance. */
#define BLINK_PERIOD_MS 500

int main(void)
{
	LOG_INF("SwiftEEG firmware starting (M1 bring-up)");

	int err = leds_init();
	if (err) {
		LOG_ERR("LED init failed (%d) — halting", err);
		return err;
	}

	/* Start out of phase. */
	(void)leds_set(LED_YELLOW, true);
	(void)leds_set(LED_BLUE, false);

	LOG_INF("LEDs initialised, entering blink loop");

	while (1) {
		(void)leds_toggle(LED_YELLOW);
		(void)leds_toggle(LED_BLUE);
		k_msleep(BLINK_PERIOD_MS);
	}

	return 0;
}
