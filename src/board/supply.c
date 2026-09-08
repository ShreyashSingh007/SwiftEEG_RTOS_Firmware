#include "supply.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_power.h>

LOG_MODULE_REGISTER(supply, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * SAADC channels wired in devicetree to the SoC's internal VDD input, so no
 * external pin is involved. Using the Zephyr ADC driver rather than the raw
 * HAL keeps calibration, EasyDMA buffer handling and the reference maths in
 * one place.
 *
 * Three channels sample the same VDD at different gains. See the devicetree
 * comment: agreement across gains means the reading is real, saturation on
 * the narrower ranges would mean the scaling is wrong.
 */
static const struct adc_dt_spec vdd_ch[] = {
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
};

/* Full-scale millivolts for each channel, i.e. 600 mV / gain. */
static const int fs_mv[] = { 3600, 3000, 2400 };
static const char *const gain_name[] = { "1/6", "1/5", "1/4" };

/*
 * 14-bit single-ended tops out at 16383 in theory, but gain and reference
 * tolerance mean a saturated channel reads slightly under that - 16380 was
 * observed on this part. Compare against a threshold rather than the exact
 * maximum, or genuine saturation goes unflagged.
 */
#define ADC_SATURATED_COUNT 16300

static int read_channel(const struct adc_dt_spec *spec, int16_t *raw_out)
{
	int16_t raw = 0;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
		/* One-shot diagnostics can afford a calibration each time. */
		.calibrate = true,
	};

	if (!adc_is_ready_dt(spec)) {
		return -ENODEV;
	}

	int err = adc_channel_setup_dt(spec);
	if (err) {
		return err;
	}

	err = adc_sequence_init_dt(spec, &seq);
	if (err) {
		return err;
	}

	err = adc_read_dt(spec, &seq);
	if (err) {
		return err;
	}

	*raw_out = raw;
	return 0;
}

int supply_read_vdd_mv(void)
{
	int16_t raw = 0;

	int err = read_channel(&vdd_ch[0], &raw);
	if (err) {
		return err;
	}

	int32_t mv = raw;

	err = adc_raw_to_millivolts_dt(&vdd_ch[0], &mv);
	if (err) {
		return err;
	}

	return (int)mv;
}

void supply_selftest(void)
{
	LOG_INF("VDD cross-check across SAADC gains:");

	for (size_t i = 0; i < ARRAY_SIZE(vdd_ch); i++) {
		int16_t raw = 0;

		int err = read_channel(&vdd_ch[i], &raw);
		if (err) {
			LOG_WRN("  gain %s: read failed (%d)", gain_name[i], err);
			continue;
		}

		int32_t mv = raw;
		(void)adc_raw_to_millivolts_dt(&vdd_ch[i], &mv);

		/*
		 * Saturation is the informative case: a rail above this
		 * channel's full scale pins the code at maximum, which a
		 * genuinely lower rail can never do.
		 */
		const bool saturated = (raw >= ADC_SATURATED_COUNT);

		LOG_INF("  gain %s (full scale %d mV): raw %5d -> %4d mV%s",
			gain_name[i], fs_mv[i], raw, mv,
			saturated ? "  <-- SATURATED, VDD exceeds this range" : "");
	}

}

void supply_log_usb_status(void)
{
	/*
	 * The nRF52840 will not bring up its USB PHY until both bits are set.
	 * VBUSDETECT alone means the cable is present; OUTPUTRDY means the
	 * internal USB regulator has actually come up, and the USB device
	 * controller stays disabled without it.
	 */
	const uint32_t status = nrf_power_usbregstatus_get(NRF_POWER);
	const bool vbus = (status & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
	const bool ready = (status & POWER_USBREGSTATUS_OUTPUTRDY_Msk) != 0;

	LOG_INF("USBREGSTATUS 0x%08x: VBUSDETECT=%d OUTPUTRDY=%d",
		status, vbus ? 1 : 0, ready ? 1 : 0);

	if (vbus && !ready) {
		LOG_WRN("VBUS present but the USB regulator is not ready - "
			"the USB PHY cannot start");
	} else if (!vbus) {
		LOG_INF("No VBUS: USB cable not connected, or VBUS is not "
			"reaching the MCU");
	}
}
