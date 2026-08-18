/* Two status LEDs. Both active high — the MCU drives the LED anode. */
#ifndef SWIFTEEG_BOARD_LEDS_H
#define SWIFTEEG_BOARD_LEDS_H

#include <stdbool.h>
#include <zephyr/kernel.h>

typedef enum {
	LED_YELLOW = 0,
	LED_BLUE   = 1,
	LED_COUNT
} led_id_t;

/* Returns 0 on success, negative errno if a LED GPIO is not ready. */
int leds_init(void);

int leds_set(led_id_t id, bool on);
int leds_toggle(led_id_t id);

#endif /* SWIFTEEG_BOARD_LEDS_H */
