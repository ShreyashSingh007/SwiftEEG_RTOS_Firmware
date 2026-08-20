#include "usb.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
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
