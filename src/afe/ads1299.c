#include "ads1299.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(afe, CONFIG_LOG_DEFAULT_LEVEL);

#define AFE_NODE DT_ALIAS(eeg_afe)

/*
 * ADS1299 is SPI mode 1 (CPOL=0, CPHA=1).
 *
 * Register access runs at 1 MHz rather than the 16 MHz in the devicetree.
 * The part needs ~4 tCLK (about 2 us at its 2.048 MHz internal oscillator)
 * between a command and the data that follows it. At 1 MHz one byte takes
 * 8 us, which satisfies that naturally and lets each register operation be a
 * single transfer. Streaming in M2 uses the full 16 MHz, where that timing is
 * handled by the DMA framing instead.
 */
#define AFE_REG_FREQ_HZ 1000000U
#define AFE_SPI_OP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

static struct spi_dt_spec afe_spi = SPI_DT_SPEC_GET(AFE_NODE, AFE_SPI_OP);
static const struct gpio_dt_spec afe_drdy =
	GPIO_DT_SPEC_GET(AFE_NODE, drdy_gpios);

static int afe_cmd(uint8_t cmd)
{
	const struct spi_buf tx = { .buf = &cmd, .len = 1 };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

	return spi_write_dt(&afe_spi, &tx_set);
}

static int afe_read_reg(uint8_t addr, uint8_t *val)
{
	/* RREG: [0x20|addr][n-1][data]. Single transfer, CS held throughout. */
	uint8_t tx_buf[3] = { ADS1299_CMD_RREG | addr, 0x00, 0x00 };
	uint8_t rx_buf[3] = { 0 };

	const struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	const struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
	const struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	const struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

	int err = spi_transceive_dt(&afe_spi, &tx_set, &rx_set);
	if (err) {
		return err;
	}

	*val = rx_buf[2];
	return 0;
}

const char *afe_probe_str(afe_probe_result_t r)
{
	switch (r) {
	case AFE_PRESENT:       return "present";
	case AFE_ABSENT:        return "absent (not fitted)";
	case AFE_UNEXPECTED_ID: return "unexpected ID";
	case AFE_BUS_ERROR:     return "SPI bus error";
	default:                return "?";
	}
}

int ads1299_probe(struct afe_probe *out)
{
	out->result = AFE_BUS_ERROR;
	out->raw_id = 0;

	if (!spi_is_ready_dt(&afe_spi)) {
		LOG_ERR("AFE SPI bus not ready");
		return -ENODEV;
	}

	/* Slow down for register access - see AFE_REG_FREQ_HZ above. */
	afe_spi.config.frequency = AFE_REG_FREQ_HZ;

	/*
	 * RESET opcode, not a pin: RESET is tied high through a pull-up on
	 * this board. Datasheet asks for 18 tCLK after reset; 10 ms is
	 * generous and this path is not time critical.
	 */
	(void)afe_cmd(ADS1299_CMD_RESET);
	k_msleep(10);

	/*
	 * Leave continuous-read mode before touching registers. After reset
	 * the part is in RDATAC, where RREG is ignored.
	 */
	(void)afe_cmd(ADS1299_CMD_SDATAC);
	k_busy_wait(10);

	uint8_t id = 0;
	int err = afe_read_reg(ADS1299_REG_ID, &id);
	if (err) {
		LOG_ERR("AFE ID read failed (%d)", err);
		out->result = AFE_BUS_ERROR;
		return err;
	}

	out->raw_id = id;

	/*
	 * With no AFE fitted, MISO floats or is driven by nothing, which reads
	 * as all-zeros or all-ones. Treat both as "not present" rather than
	 * misreporting a bogus ID.
	 */
	if (id == 0x00 || id == 0xFF) {
		out->result = AFE_ABSENT;
		return 0;
	}

	/* Bit 4 always reads 1, and NU_CH must say 8 channels. */
	if ((id & ADS1299_ID_RESERVED_BIT4) &&
	    (id & ADS1299_ID_NU_CH_MSK) == ADS1299_ID_NU_CH_8) {
		out->result = AFE_PRESENT;
	} else {
		out->result = AFE_UNEXPECTED_ID;
	}

	return 0;
}

bool ads1299_start_pin_stuck_high(void)
{
	if (!gpio_is_ready_dt(&afe_drdy)) {
		LOG_WRN("DRDY GPIO not ready, skipping START check");
		return false;
	}

	if (gpio_pin_configure_dt(&afe_drdy, GPIO_INPUT) != 0) {
		LOG_WRN("DRDY configure failed, skipping START check");
		return false;
	}

	/* Ask the part to stop converting. If START is asserted it will not. */
	(void)afe_cmd(ADS1299_CMD_STOP);
	k_msleep(5);

	/*
	 * DRDY is active low and pulses once per conversion. Sample for a few
	 * milliseconds; any transition means conversions are still running,
	 * which means START is being held high in hardware.
	 */
	int first = gpio_pin_get_dt(&afe_drdy);
	for (int i = 0; i < 200; i++) {
		if (gpio_pin_get_dt(&afe_drdy) != first) {
			return true;
		}
		k_busy_wait(50);
	}

	return false;
}
