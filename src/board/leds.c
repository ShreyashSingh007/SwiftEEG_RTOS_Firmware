#include "leds.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(leds, CONFIG_LOG_DEFAULT_LEVEL);

static const struct gpio_dt_spec leds[LED_COUNT] = {
	[LED_YELLOW] = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
	[LED_BLUE]   = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
};

int leds_init(void)
{
	for (size_t i = 0; i < LED_COUNT; i++) {
		if (!gpio_is_ready_dt(&leds[i])) {
			LOG_ERR("LED %u GPIO not ready", (unsigned)i);
			return -ENODEV;
		}

		int err = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
		if (err) {
			LOG_ERR("LED %u configure failed (%d)", (unsigned)i, err);
			return err;
		}
	}

	return 0;
}

int leds_set(led_id_t id, bool on)
{
	if (id >= LED_COUNT) {
		return -EINVAL;
	}
	return gpio_pin_set_dt(&leds[id], on ? 1 : 0);
}

int leds_toggle(led_id_t id)
{
	if (id >= LED_COUNT) {
		return -EINVAL;
	}
	return gpio_pin_toggle_dt(&leds[id]);
}
