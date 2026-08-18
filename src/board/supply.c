#include "supply.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(supply, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * SAADC channel 0 is wired in devicetree to the SoC's internal VDD input,
 * so no external pin is involved. Using the Zephyr ADC driver rather than the
 * raw HAL keeps calibration, EasyDMA buffer handling and the reference maths
 * in one place.
 */
static const struct adc_dt_spec vdd_ch = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

int supply_read_vdd_mv(void)
{
	int16_t raw = 0;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
		/* Ask the driver to calibrate; a one-shot diagnostic can afford it. */
		.calibrate = true,
	};

	if (!adc_is_ready_dt(&vdd_ch)) {
		return -ENODEV;
	}

	int err = adc_channel_setup_dt(&vdd_ch);
	if (err) {
		return err;
	}

	err = adc_sequence_init_dt(&vdd_ch, &seq);
	if (err) {
		return err;
	}

	err = adc_read_dt(&vdd_ch, &seq);
	if (err) {
		return err;
	}

	/*
	 * Log the raw count too. Full scale is 3.6 V over 2^14 counts, so a
	 * healthy 3.3 V rail should read about 15000. A much lower count means
	 * the rail really is low rather than the scaling being wrong.
	 */
	LOG_INF("VDD raw count: %d (expect ~15000 at 3.3 V)", raw);

	int32_t mv = raw;

	/* Converts the raw count to millivolts using the DT gain/reference. */
	err = adc_raw_to_millivolts_dt(&vdd_ch, &mv);
	if (err) {
		return err;
	}

	return (int)mv;
}
