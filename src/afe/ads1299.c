#include "ads1299.h"

#include "timebase/timebase.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_spim.h>

LOG_MODULE_REGISTER(afe, CONFIG_LOG_DEFAULT_LEVEL);

#define AFE_NODE     DT_ALIAS(eeg_afe)
#define AFE_SPI_NODE DT_NODELABEL(spi3)
#define AFE_SPIM     NRF_SPIM3

/*
 * SPIM3 is driven through the HAL rather than Zephyr's SPI API.
 *
 * The acquisition path needs the transfer started by PPI on the DRDY edge,
 * with the CPU asleep. Zephyr's API has no way to express that - it starts
 * transfers from a function call - so this driver owns the peripheral
 * directly and the spi3 node is left disabled for Zephyr's driver.
 *
 * SPIM3 specifically because it is the only instance on this part with
 * hardware chip select, which the PPI-triggered transfer needs: nothing can
 * toggle CS in software when no code runs between DRDY and the transfer.
 *
 * Pins still come from devicetree - pinctrl routes SCK, MOSI and MISO, and
 * CS comes from the node's cs-gpios.
 */
PINCTRL_DT_DEFINE(AFE_SPI_NODE);
static const struct pinctrl_dev_config *afe_pcfg =
	PINCTRL_DT_DEV_CONFIG_GET(AFE_SPI_NODE);

#define AFE_CS_PORT DT_PROP(DT_SPI_DEV_CS_GPIOS_CTLR(AFE_NODE), port)
#define AFE_CS_PIN  DT_SPI_DEV_CS_GPIOS_PIN(AFE_NODE)

static const struct gpio_dt_spec afe_drdy =
	GPIO_DT_SPEC_GET(AFE_NODE, drdy_gpios);

/*
 * ADS1299 is SPI mode 1 (CPOL=0, CPHA=1).
 *
 * Register access runs at 1 MHz rather than the 16 MHz the part can take.
 * It needs ~4 tCLK (about 2 us at its 2.048 MHz internal oscillator) between
 * a command and the data that follows. At 1 MHz one byte takes 8 us, which
 * satisfies that naturally and lets each register operation be a single
 * transfer. Streaming switches to 16 MHz, where the gap is handled by the
 * framing instead.
 */
#define AFE_FREQ_REGS   NRF_SPIM_FREQ_1M
#define AFE_FREQ_STREAM NRF_SPIM_FREQ_8M

/*
 * CSN guard time, in 15.625 ns units. The ADS1299 wants CS settled well
 * before the first clock edge; 16 gives 250 ns, comfortably over its
 * requirement and costing nothing at these rates.
 */
#define AFE_CSN_DURATION 16

/*
 * EasyDMA can only reach RAM, so command bytes cannot be sent from a const
 * array in flash. Everything goes through these.
 */
static uint8_t afe_tx[8];
static uint8_t afe_rx[8];

static bool afe_bus_ready;

/*
 * True while the transfer-complete interrupt is armed, and while CS is being
 * held low for a streaming session. Blocking transfers have to know both:
 * see afe_xfer().
 */
static bool afe_irq_on;
static bool afe_cs_held;

static uint32_t afe_cs_pin(void)
{
	return NRF_GPIO_PIN_MAP(AFE_CS_PORT, AFE_CS_PIN);
}

static int afe_bus_init(void)
{
	if (afe_bus_ready) {
		return 0;
	}

	int err = pinctrl_apply_state(afe_pcfg, PINCTRL_STATE_DEFAULT);

	if (err < 0) {
		LOG_ERR("AFE pinctrl failed (%d)", err);
		return err;
	}

	nrf_spim_disable(AFE_SPIM);
	nrf_spim_configure(AFE_SPIM, NRF_SPIM_MODE_1, NRF_SPIM_BIT_ORDER_MSB_FIRST);
	nrf_spim_frequency_set(AFE_SPIM, AFE_FREQ_REGS);

	/*
	 * Chip select is driven as a plain GPIO for register access. The pad
	 * has to be an output either way: csn_configure() only routes CSN
	 * inside the peripheral and never touches GPIO, and CS is not in the
	 * pinctrl group because devicetree models it as cs-gpios.
	 */
	nrf_gpio_pin_set(afe_cs_pin()); /* idle high */
	nrf_gpio_cfg_output(afe_cs_pin());

	/* Bytes clocked out once the TX buffer runs dry during a longer read. */
	nrf_spim_orc_set(AFE_SPIM, 0x00);

	nrf_spim_enable(AFE_SPIM);

	afe_bus_ready = true;
	return 0;
}

static void afe_xfer_done(void)
{
	if (!afe_cs_held) {
		nrf_gpio_pin_set(afe_cs_pin());
	}
	if (afe_irq_on) {
		nrf_spim_int_enable(AFE_SPIM, NRF_SPIM_INT_END_MASK);
	}
}

/* Blocking transfer. Used for register access only; streaming is DMA. */
static int afe_xfer(size_t len)
{
	if (!afe_bus_ready) {
		return -ENODEV;
	}

	/*
	 * Take the END interrupt down for the duration. This function decides
	 * the transfer is finished by polling that same event, and the ISR
	 * clears it - so with the interrupt armed the poll never sees it and
	 * every register access times out.
	 */
	if (afe_irq_on) {
		nrf_spim_int_disable(AFE_SPIM, NRF_SPIM_INT_END_MASK);
	}

	nrf_spim_tx_buffer_set(AFE_SPIM, afe_tx, len);
	nrf_spim_rx_buffer_set(AFE_SPIM, afe_rx, len);

	/* During streaming CS is already low and must stay there. */
	if (!afe_cs_held) {
		nrf_gpio_pin_clear(afe_cs_pin());
	}

	nrf_spim_event_clear(AFE_SPIM, NRF_SPIM_EVENT_END);
	nrf_spim_task_trigger(AFE_SPIM, NRF_SPIM_TASK_START);

	/*
	 * Bounded wait. At 1 MHz an 8-byte transfer takes 64 us, so anything
	 * approaching this limit means the peripheral is not running.
	 */
	for (int i = 0; i < 10000; i++) {
		if (nrf_spim_event_check(AFE_SPIM, NRF_SPIM_EVENT_END)) {
			nrf_spim_event_clear(AFE_SPIM, NRF_SPIM_EVENT_END);
			afe_xfer_done();
			return 0;
		}
		k_busy_wait(1);
	}

	afe_xfer_done();
	LOG_ERR("AFE SPI transfer timed out");
	return -ETIMEDOUT;
}

static int afe_cmd(uint8_t cmd)
{
	afe_tx[0] = cmd;
	return afe_xfer(1);
}

static int afe_read_reg(uint8_t addr, uint8_t *val)
{
	/* RREG: [0x20|addr][n-1][data]. One transfer, CS held throughout. */
	afe_tx[0] = ADS1299_CMD_RREG | addr;
	afe_tx[1] = 0x00;
	afe_tx[2] = 0x00;

	int err = afe_xfer(3);

	if (err) {
		return err;
	}

	*val = afe_rx[2];
	return 0;
}

static int afe_write_reg(uint8_t addr, uint8_t val)
{
	/* WREG: [0x40|addr][n-1][data]. */
	afe_tx[0] = ADS1299_CMD_WREG | addr;
	afe_tx[1] = 0x00;
	afe_tx[2] = val;

	return afe_xfer(3);
}

int ads1299_read_reg(uint8_t addr, uint8_t *val)
{
	return afe_read_reg(addr, val);
}

int ads1299_start_conversions(void)
{
	return afe_cmd(ADS1299_CMD_START);
}

int ads1299_stop_conversions(void)
{
	return afe_cmd(ADS1299_CMD_STOP);
}

int ads1299_configure(uint8_t rate)
{
	/* Registers are only writable outside continuous-read mode. */
	int err = afe_cmd(ADS1299_CMD_SDATAC);
	if (err) {
		return err;
	}
	k_busy_wait(10);

	err = afe_write_reg(ADS1299_REG_CONFIG1,
			    ADS1299_CONFIG1_BASE | (rate & 0x07));
	if (err) {
		return err;
	}

	/*
	 * Internal reference on. VREFP carries decoupling only on this board,
	 * so without this the part converts against nothing.
	 */
	err = afe_write_reg(ADS1299_REG_CONFIG3,
			    ADS1299_CONFIG3_BASE | ADS1299_CONFIG3_PD_REFBUF);
	if (err) {
		return err;
	}

	/* The reference needs time to settle before conversions mean anything. */
	k_msleep(150);

	/* SRB1 referential montage - every channel measured against SRB1. */
	err = afe_write_reg(ADS1299_REG_MISC1, ADS1299_MISC1_SRB1);
	if (err) {
		return err;
	}

	/*
	 * Read back the two registers that decide whether the data is valid at
	 * all. A silent SPI failure here would otherwise look like real EEG.
	 */
	uint8_t cfg1 = 0, cfg3 = 0;

	(void)afe_read_reg(ADS1299_REG_CONFIG1, &cfg1);
	(void)afe_read_reg(ADS1299_REG_CONFIG3, &cfg3);

	const uint8_t want1 = ADS1299_CONFIG1_BASE | (rate & 0x07);
	const uint8_t want3 = ADS1299_CONFIG3_BASE | ADS1299_CONFIG3_PD_REFBUF;

	if (cfg1 != want1 || cfg3 != want3) {
		LOG_ERR("AFE config readback wrong: CONFIG1 %02x (want %02x), "
			"CONFIG3 %02x (want %02x)", cfg1, want1, cfg3, want3);
		return -EIO;
	}

	LOG_INF("AFE configured: CONFIG1 %02x, CONFIG3 %02x, SRB1 on", cfg1, cfg3);
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

	int bus_err = afe_bus_init();

	if (bus_err) {
		return bus_err;
	}

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


/* ---- continuous acquisition ---------------------------------------- */

/*
 * Two frame buffers. EasyDMA writes one while the callback reads the other,
 * so a frame is never being overwritten while it is being consumed. They
 * must live in RAM - EasyDMA cannot reach flash.
 */
static uint8_t afe_frame[2][ADS1299_FRAME_BYTES];
static uint8_t afe_dummy[ADS1299_FRAME_BYTES];
static uint8_t afe_active;

static ads1299_frame_cb_t afe_cb;
static volatile uint32_t afe_overrun;
static bool afe_streaming;

static void afe_spim_isr(const void *arg)
{
	ARG_UNUSED(arg);

	if (!nrf_spim_event_check(AFE_SPIM, NRF_SPIM_EVENT_END)) {
		return;
	}
	nrf_spim_event_clear(AFE_SPIM, NRF_SPIM_EVENT_END);

	/*
	 * The timestamp was latched when DRDY fell, before this interrupt was
	 * ever raised, so none of its latency is in the number.
	 */
	const uint64_t ts = timebase_stamp_us(timebase_capture_get());

	const uint8_t done = afe_active;

	/*
	 * Point the DMA at the other buffer before doing anything else. The
	 * next DRDY can start a transfer at any moment and nothing in software
	 * gates it.
	 */
	afe_active ^= 1;
	nrf_spim_rx_buffer_set(AFE_SPIM, afe_frame[afe_active],
			       ADS1299_FRAME_BYTES);

	if (afe_cb != NULL) {
		afe_cb(afe_frame[done], ts);
	}
}

uint32_t ads1299_start_task_addr(void)
{
	return nrf_spim_task_address_get(AFE_SPIM, NRF_SPIM_TASK_START);
}

uint32_t ads1299_overruns(void)
{
	return afe_overrun;
}

int ads1299_stream_start(ads1299_frame_cb_t cb)
{
	if (afe_streaming) {
		return -EALREADY;
	}
	if (!afe_bus_ready) {
		return -ENODEV;
	}

	afe_cb = cb;
	afe_active = 0;
	afe_overrun = 0;

	/* Enter continuous-read mode: the part clocks out a frame per DRDY. */
	int err = afe_cmd(ADS1299_CMD_RDATAC);

	if (err) {
		return err;
	}

	/*
	 * Faster clock for the payload. 27 bytes at 8 MHz take 27 us, which
	 * fits inside even the 62.5 us budget of the highest sample rate.
	 */
	nrf_spim_frequency_set(AFE_SPIM, AFE_FREQ_STREAM);

	/*
	 * Chip select stays low for the whole session rather than being
	 * toggled per transfer.
	 *
	 * Nothing runs between the DRDY edge and the transfer it triggers, so
	 * software cannot drive CS, and the peripheral's hardware chip select
	 * does not work here: its guard time maxes out at about 4 us and the
	 * ADS1299 needs longer between CS falling and the first clock. Holding
	 * CS low throughout is the arrangement the datasheet describes for
	 * continuous read, and it removes the timing question entirely.
	 */
	nrf_gpio_pin_clear(afe_cs_pin());
	afe_cs_held = true;

	/*
	 * TX is a buffer of zeros: the part ignores DIN during RDATAC, but
	 * SPIM needs something to clock out to generate SCK.
	 */
	memset(afe_dummy, 0, sizeof(afe_dummy));
	nrf_spim_tx_buffer_set(AFE_SPIM, afe_dummy, ADS1299_FRAME_BYTES);
	nrf_spim_rx_buffer_set(AFE_SPIM, afe_frame[afe_active],
			       ADS1299_FRAME_BYTES);

	nrf_spim_event_clear(AFE_SPIM, NRF_SPIM_EVENT_END);

	IRQ_CONNECT(DT_IRQN(AFE_SPI_NODE), DT_IRQ(AFE_SPI_NODE, priority),
		    afe_spim_isr, NULL, 0);
	irq_enable(DT_IRQN(AFE_SPI_NODE));
	nrf_spim_int_enable(AFE_SPIM, NRF_SPIM_INT_END_MASK);
	afe_irq_on = true;

	afe_streaming = true;

	/* Conversions last: everything must be ready before the first DRDY. */
	err = ads1299_start_conversions();
	if (err) {
		ads1299_stream_stop();
		return err;
	}

	return 0;
}

void ads1299_stream_stop(void)
{
	if (!afe_streaming) {
		return;
	}

	(void)ads1299_stop_conversions();

	nrf_spim_int_disable(AFE_SPIM, NRF_SPIM_INT_END_MASK);
	irq_disable(DT_IRQN(AFE_SPI_NODE));
	afe_irq_on = false;

	afe_cs_held = false;
	nrf_gpio_pin_set(afe_cs_pin());
	nrf_spim_frequency_set(AFE_SPIM, AFE_FREQ_REGS);

	afe_streaming = false;
	afe_cb = NULL;

	/* Back to command mode so registers can be written again. */
	(void)afe_cmd(ADS1299_CMD_SDATAC);
}
