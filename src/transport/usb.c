#include "usb.h"

#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
/*
 * usbd_msg.h has an always-true comparison that trips -Werror=type-limits.
 * That is a bug in the SDK header, not here, so the suppression is scoped to
 * this include only rather than weakening the flag for our own code.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#include <zephyr/usb/usbd.h>
#pragma GCC diagnostic pop

LOG_MODULE_REGISTER(usb, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Uses the current USB device stack (usbd), not the legacy one. The legacy
 * stack still exists in this SDK but is marked DEPRECATED and will be
 * removed, which is not a foundation worth building on.
 *
 * nRF52840 is full-speed only, so only an FS configuration is defined.
 *
 * VID/PID below are Zephyr's test IDs. They are fine for development but
 * MUST be replaced before anything ships - a real product needs its own
 * vendor ID.
 */
#define SWIFTEEG_USB_VID 0x2FE3
#define SWIFTEEG_USB_PID 0x0001

USBD_DEVICE_DEFINE(swifteeg_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(usbd)),
		   SWIFTEEG_USB_VID, SWIFTEEG_USB_PID);

USBD_DESC_LANG_DEFINE(swifteeg_lang);
USBD_DESC_MANUFACTURER_DEFINE(swifteeg_mfr, "SwiftEEG");
USBD_DESC_PRODUCT_DEFINE(swifteeg_product, "SwiftEEG 8-Channel EEG");
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(swifteeg_sn)));

USBD_DESC_CONFIG_DEFINE(swifteeg_fs_cfg_desc, "SwiftEEG FS Configuration");

/* Bus-powered, 250 mA. No remote wakeup: nothing here needs to wake a host. */
USBD_CONFIGURATION_DEFINE(swifteeg_fs_config, 0, 250, &swifteeg_fs_cfg_desc);

/* Transmit path; defined at the bottom of this file. */
#define USB_TX_BUF_BYTES 8192
static uint8_t usb_tx_storage[USB_TX_BUF_BYTES];
static struct ring_buf usb_tx_rb;
static atomic_t usb_tx_dropped;
static bool usb_tx_ready;
static usb_rx_notify_t usb_rx_notify;
static void cdc_irq_handler(const struct device *dev, void *user_data);

/* Inbound buffer. Commands are small and rare; 512 bytes is generous. */
#define USB_RX_BUF_BYTES 512
static uint8_t usb_rx_storage[USB_RX_BUF_BYTES];
static struct ring_buf usb_rx_rb;

static const struct device *const cdc_dev =
	DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

int usb_transport_init(void)
{
	int err;

	if (!device_is_ready(cdc_dev)) {
		LOG_ERR("CDC ACM device not ready");
		return -ENODEV;
	}

	err = usbd_add_descriptor(&swifteeg_usbd, &swifteeg_lang);
	if (err) {
		LOG_ERR("usbd lang descriptor failed (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&swifteeg_usbd, &swifteeg_mfr);
	if (err) {
		LOG_ERR("usbd manufacturer descriptor failed (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&swifteeg_usbd, &swifteeg_product);
	if (err) {
		LOG_ERR("usbd product descriptor failed (%d)", err);
		return err;
	}

	IF_ENABLED(CONFIG_HWINFO, ({
		/* Serial number derived from the SoC's unique device ID. */
		err = usbd_add_descriptor(&swifteeg_usbd, &swifteeg_sn);
		if (err) {
			LOG_WRN("usbd serial descriptor failed (%d)", err);
		}
	}));

	err = usbd_add_configuration(&swifteeg_usbd, USBD_SPEED_FS,
				     &swifteeg_fs_config);
	if (err) {
		LOG_ERR("usbd add configuration failed (%d)", err);
		return err;
	}

	err = usbd_register_all_classes(&swifteeg_usbd, USBD_SPEED_FS, 1, NULL);
	if (err) {
		LOG_ERR("usbd register classes failed (%d)", err);
		return err;
	}

	err = usbd_init(&swifteeg_usbd);
	if (err) {
		LOG_ERR("usbd init failed (%d)", err);
		return err;
	}

	err = usbd_enable(&swifteeg_usbd);
	if (err) {
		LOG_ERR("usbd enable failed (%d)", err);
		return err;
	}

	ring_buf_init(&usb_tx_rb, sizeof(usb_tx_storage), usb_tx_storage);
	ring_buf_init(&usb_rx_rb, sizeof(usb_rx_storage), usb_rx_storage);
	uart_irq_callback_user_data_set(cdc_dev, cdc_irq_handler, NULL);
	uart_irq_rx_enable(cdc_dev);
	usb_tx_ready = true;

	LOG_INF("USB CDC ACM up (VID 0x%04x PID 0x%04x)",
		SWIFTEEG_USB_VID, SWIFTEEG_USB_PID);
	return 0;
}

bool usb_transport_is_connected(void)
{
	uint32_t dtr = 0;

	if (!device_is_ready(cdc_dev)) {
		return false;
	}

	(void)uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr);
	return dtr != 0;
}


/* ---- transmit path -------------------------------------------------- */

/*
 * The outbound buffer is 8 kB, about 30 ms of 8-channel data at 1 kSPS -
 * enough to ride out the host pausing briefly without losing anything.
 */
static void cdc_irq_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t rx[64];
			const int n = uart_fifo_read(dev, rx, sizeof(rx));

			if (n > 0) {
				/* A full buffer drops commands rather than
				 * stalling the interrupt. */
				(void)ring_buf_put(&usb_rx_rb, rx, (uint32_t)n);

				if (usb_rx_notify != NULL) {
					usb_rx_notify();
				}
			}
		}

		if (!uart_irq_tx_ready(dev)) {
			continue;
		}

		uint8_t *data;
		const uint32_t claimed =
			ring_buf_get_claim(&usb_tx_rb, &data, USB_TX_BUF_BYTES);

		if (claimed == 0) {
			/* Nothing left: stop asking to be interrupted. */
			uart_irq_tx_disable(dev);
			(void)ring_buf_get_finish(&usb_tx_rb, 0);
			continue;
		}

		const int sent = uart_fifo_fill(dev, data, (int)claimed);

		(void)ring_buf_get_finish(&usb_tx_rb, (sent > 0) ? sent : 0);
	}
}

/*
 * Several threads write: the DSP thread's samples, the IMU thread's motion
 * frames and the command thread's replies. The ring buffer is only safe for
 * one writer at a time, so writers take turns. And a frame goes in whole or
 * not at all - half a frame corrupts the byte stream for whatever follows.
 */
static K_MUTEX_DEFINE(usb_tx_lock);

size_t usb_transport_write(const uint8_t *buf, size_t len)
{
	if (!usb_tx_ready || !usb_transport_is_connected()) {
		return 0;
	}

	uint32_t put = 0;

	k_mutex_lock(&usb_tx_lock, K_FOREVER);
	if (ring_buf_space_get(&usb_tx_rb) >= len) {
		put = ring_buf_put(&usb_tx_rb, buf, (uint32_t)len);
	}
	k_mutex_unlock(&usb_tx_lock);

	if (put < len) {
		atomic_add(&usb_tx_dropped, (atomic_val_t)(len - put));
	}

	if (put != 0) {
		uart_irq_tx_enable(cdc_dev);
	}

	return put;
}

uint32_t usb_transport_dropped(void)
{
	return (uint32_t)atomic_get(&usb_tx_dropped);
}

size_t usb_transport_read(uint8_t *buf, size_t len)
{
	if (!usb_tx_ready) {
		return 0;
	}

	return ring_buf_get(&usb_rx_rb, buf, (uint32_t)len);
}

void usb_transport_set_rx_notify(usb_rx_notify_t cb)
{
	usb_rx_notify = cb;
}
