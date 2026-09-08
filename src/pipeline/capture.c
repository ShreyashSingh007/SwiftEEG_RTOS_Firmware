#include "capture.h"

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <helpers/nrfx_gppi.h>
#include <hal/nrf_gpio.h>
#include <nrfx_gpiote.h>

#include "timebase/timebase.h"

LOG_MODULE_REGISTER(capture, CONFIG_LOG_DEFAULT_LEVEL);

#define AFE_NODE DT_ALIAS(eeg_afe)

/* DRDY is active low, so a new sample is the falling edge. */
#define DRDY_PIN  DT_GPIO_PIN(AFE_NODE, drdy_gpios)
#define DRDY_PORT DT_PROP(DT_GPIO_CTLR(AFE_NODE, drdy_gpios), port)

/*
 * nRF52840 has a single GPIOTE. The macro takes the peripheral's register
 * base, not an instance index - passing 0 compiles fine and leaves p_reg
 * NULL, which faults on the first register write.
 */
static nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(NRF_GPIOTE);

static uint8_t drdy_ch;
static nrfx_gppi_handle_t ppi_handle;
static bool capture_ready;

static nrfx_gpiote_pin_t drdy_pin(void)
{
	return NRF_GPIO_PIN_MAP(DRDY_PORT, DRDY_PIN);
}

int capture_init(void)
{
	if (capture_ready) {
		return 0;
	}

	/*
	 * Zephyr's GPIO driver shares this peripheral, so initialise only if
	 * that has not already happened, and allocate the channel rather than
	 * picking one, so the two cannot collide.
	 */
	if (!nrfx_gpiote_init_check(&gpiote)) {
		int err = nrfx_gpiote_init(&gpiote, 0);

		if (err != 0) {
			LOG_ERR("GPIOTE init failed (%d)", err);
			return -EIO;
		}
	}

	/*
	 * GPIOTE can only see an edge if the pad's input buffer is connected.
	 * Do it here rather than relying on some earlier caller having
	 * configured the pin, which is an easy dependency to break.
	 */
	nrf_gpio_cfg_input(drdy_pin(), NRF_GPIO_PIN_NOPULL);

	int err = nrfx_gpiote_channel_alloc(&gpiote, &drdy_ch);
	if (err != 0) {
		LOG_ERR("no free GPIOTE channel (%d)", err);
		return -ENOMEM;
	}

	/*
	 * No handler: this configures the pin to raise a GPIOTE IN event, and
	 * nothing more. The event feeds PPI directly.
	 */
	const nrfx_gpiote_trigger_config_t trigger = {
		.trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
		.p_in_channel = &drdy_ch,
	};
	const nrfx_gpiote_input_pin_config_t pin_cfg = {
		.p_trigger_config = &trigger,
	};

	err = nrfx_gpiote_input_configure(&gpiote, drdy_pin(), &pin_cfg);
	if (err != 0) {
		LOG_ERR("DRDY input configure failed (%d)", err);
		return -EIO;
	}

	/*
	 * One PPI channel carries the DRDY event to the timer's capture task.
	 * The allocator masks out the channels MPSL reserves, so this cannot
	 * silently steal one the radio depends on.
	 *
	 * On nRF52 a channel drives one event and up to two tasks, so the SPI
	 * start task attaches to this same channel later.
	 */
	const uint32_t domain = nrfx_gppi_domain_id_get((uint32_t)NRF_TIMER1);

	err = nrfx_gppi_domain_conn_alloc(domain, domain, &ppi_handle);
	if (err != 0) {
		LOG_ERR("no free PPI channel (%d)", err);
		return -ENOMEM;
	}

	err = nrfx_gppi_ep_attach(
		nrfx_gpiote_in_event_address_get(&gpiote, drdy_pin()), ppi_handle);
	if (err != 0) {
		LOG_ERR("DRDY event attach failed (%d)", err);
		return -EIO;
	}

	err = nrfx_gppi_ep_attach(timebase_capture_task_addr(), ppi_handle);
	if (err != 0) {
		LOG_ERR("capture task attach failed (%d)", err);
		return -EIO;
	}

	nrfx_gppi_conn_enable(ppi_handle);

	/* Raise the event, but do not interrupt the CPU for it. */
	nrfx_gpiote_trigger_enable(&gpiote, drdy_pin(), false);

	capture_ready = true;

	LOG_INF("DRDY capture wired: P%u.%02u -> TIMER1 CC[0] (GPIOTE ch %u)",
		DRDY_PORT, DRDY_PIN, drdy_ch);

	return 0;
}

int capture_attach_task(uint32_t task_addr)
{
	if (!capture_ready) {
		return -ENODEV;
	}

	int err = nrfx_gppi_ep_attach(task_addr, ppi_handle);

	if (err != 0) {
		LOG_ERR("second task attach failed (%d)", err);
		return -EIO;
	}

	return 0;
}

void capture_measure(uint32_t ms, struct capture_stats *out)
{
	uint32_t count = 0;
	uint32_t min_us = UINT32_MAX;
	uint32_t max_us = 0;
	uint32_t first = 0, last = 0;

	uint32_t prev = timebase_capture_get();
	const uint64_t deadline = timebase_now_us() + (uint64_t)ms * 1000U;

	while (timebase_now_us() < deadline) {
		const uint32_t cc = timebase_capture_get();

		if (cc == prev) {
			continue;
		}

		/*
		 * Unsigned subtraction gives the right interval even across
		 * the counter's 32-bit rollover.
		 */
		const uint32_t gap = cc - prev;

		prev = cc;

		if (count == 0) {
			first = cc;
		} else {
			if (gap < min_us) {
				min_us = gap;
			}
			if (gap > max_us) {
				max_us = gap;
			}
		}

		last = cc;
		count++;
	}

	out->count = count;
	out->min_us = (min_us == UINT32_MAX) ? 0 : min_us;
	out->max_us = max_us;
	out->total_us = (count > 1) ? (last - first) : 0;
}

uint64_t capture_last_us(void)
{
	return timebase_stamp_us(timebase_capture_get());
}
