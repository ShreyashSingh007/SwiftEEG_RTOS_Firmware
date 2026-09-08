/*
 * TI ADS1299 - 8-channel 24-bit EEG analog front end.
 *
 * M1 scope: presence detection and identification only. Acquisition,
 * RDATAC streaming and the PPI/DMA fast path arrive in M2.
 *
 * Board notes that shape this driver:
 *  - RESET and PWDN are tied high through pull-ups, so reset is by SPI
 *    opcode only and the part cannot be power-cycled by the MCU.
 *  - START is unconnected, so conversions are opcode-driven. A floating
 *    START can read high, in which case STOP has no effect - see
 *    ads1299_start_pin_stuck_high().
 *  - CLKSEL is tied high: the part runs its own internal oscillator.
 */
#ifndef SWIFTEEG_AFE_ADS1299_H
#define SWIFTEEG_AFE_ADS1299_H

#include <stdbool.h>
#include <stdint.h>

/* SPI opcodes (ADS1299 datasheet, "SPI Command Definitions"). */
#define ADS1299_CMD_WAKEUP  0x02
#define ADS1299_CMD_STANDBY 0x04
#define ADS1299_CMD_RESET   0x06
#define ADS1299_CMD_START   0x08
#define ADS1299_CMD_STOP    0x0A
#define ADS1299_CMD_RDATAC  0x10
#define ADS1299_CMD_SDATAC  0x11
#define ADS1299_CMD_RDATA   0x12
#define ADS1299_CMD_RREG    0x20	/* OR with register address */
#define ADS1299_CMD_WREG    0x40	/* OR with register address */

/* Register map (subset used at this stage). */
#define ADS1299_REG_ID      0x00
#define ADS1299_REG_CONFIG1 0x01
#define ADS1299_REG_CONFIG2 0x02
#define ADS1299_REG_CONFIG3 0x03
#define ADS1299_REG_CH1SET  0x05
#define ADS1299_REG_MISC1   0x15

/*
 * CONFIG1 data rate, the low three bits. The part divides its own internal
 * oscillator, so these are nominal - the true rate is measured, not assumed.
 */
#define ADS1299_DR_16KSPS 0x00
#define ADS1299_DR_8KSPS  0x01
#define ADS1299_DR_4KSPS  0x02
#define ADS1299_DR_2KSPS  0x03
#define ADS1299_DR_1KSPS  0x04
#define ADS1299_DR_500SPS 0x05
#define ADS1299_DR_250SPS 0x06

/* CONFIG1 bit 7 and bit 4 read back as 1; the rest we set. */
#define ADS1299_CONFIG1_BASE 0x90

/*
 * CONFIG3 bits 6:5 must be written as 1. Bit 7 is PD_REFBUF: this board
 * leaves VREFP with decoupling only, so the internal reference has to be
 * switched on or every conversion is meaningless.
 */
#define ADS1299_CONFIG3_BASE      0x60
#define ADS1299_CONFIG3_PD_REFBUF 0x80

/*
 * CHnSET: [7] power down, [6:4] gain, [3] SRB2, [2:0] mux.
 *
 * Reset is 0x61 - gain 24 with the inputs shorted, which measures the part's
 * own noise rather than anything on the electrodes. Normal input is mux 000.
 */
#define ADS1299_MUX_NORMAL  0x00
#define ADS1299_MUX_SHORTED 0x01
#define ADS1299_MUX_TEST    0x05	/* internal square wave, for M3 */

#define ADS1299_GAIN_1  0x00
#define ADS1299_GAIN_2  0x01
#define ADS1299_GAIN_4  0x02
#define ADS1299_GAIN_6  0x03
#define ADS1299_GAIN_8  0x04
#define ADS1299_GAIN_12 0x05
#define ADS1299_GAIN_24 0x06

/* MISC1 bit 5 ties every channel's negative input to SRB1. */
#define ADS1299_MISC1_SRB1 0x20

/*
 * ID register layout:
 *   [7:5] REV_ID   silicon revision, varies between parts
 *   [4]   always 1
 *   [3:2] DEV_ID
 *   [1:0] NU_CH    00 = 4ch, 01 = 6ch, 10 = 8ch
 */
#define ADS1299_ID_NU_CH_MSK  0x03
#define ADS1299_ID_NU_CH_8    0x02
#define ADS1299_ID_RESERVED_BIT4 0x10

typedef enum {
	AFE_PRESENT,       /* responded with a plausible 8-channel ID */
	AFE_ABSENT,        /* bus read all-zeros or all-ones: not fitted */
	AFE_UNEXPECTED_ID, /* something answered, but not an 8-channel ADS1299 */
	AFE_BUS_ERROR,     /* SPI transfer itself failed */
} afe_probe_result_t;

/* Result of a probe. raw_id is only meaningful when the bus read succeeded. */
struct afe_probe {
	afe_probe_result_t result;
	uint8_t raw_id;
};

/*
 * Reset the part and read its ID register.
 *
 * Never fails hard: a missing AFE is a normal condition on boards built
 * without one, and the caller is expected to keep running.
 */
int ads1299_probe(struct afe_probe *out);

/*
 * Detect a START pin stuck high.
 *
 * START is unconnected on this board. If it floats high the part free-runs
 * and the STOP opcode has no effect, which silently corrupts register writes
 * and start timing. Issues STOP, then watches DRDY: continued pulses mean
 * START is asserted in hardware.
 *
 * Only meaningful when the AFE is present. Returns true if stuck.
 */
bool ads1299_start_pin_stuck_high(void);

/*
 * Put the part into a known, converting state:
 *   - internal reference on (mandatory on this board)
 *   - the requested data rate
 *   - SRB1 referential montage, matching how the electrodes are wired
 *
 * Leaves the part in SDATAC with conversions running, so DRDY pulses but no
 * data is read. `rate` is one of the ADS1299_DR_* values.
 */
int ads1299_configure(uint8_t rate);

/*
 * Set every channel's gain and input mux. SRB2 is left open: the negative
 * inputs come from SRB1 instead, which is how this board is wired.
 */
int ads1299_set_channels(uint8_t gain, uint8_t mux);

/* START and STOP opcodes. Conversions run between them. */
int ads1299_start_conversions(void);
int ads1299_stop_conversions(void);

/* Read one register. Exposed so a caller can verify what was written. */
int ads1299_read_reg(uint8_t addr, uint8_t *val);

/*
 * One RDATAC frame: 3 status bytes then 8 channels of 24-bit data.
 * The status word carries the lead-off and GPIO bits.
 */
#define ADS1299_FRAME_BYTES 27
#define ADS1299_STATUS_BYTES 3
#define ADS1299_CHANNELS 8

/*
 * Called from the SPI transfer-complete interrupt, once per sample.
 * `ts_us` was latched by hardware when DRDY fell, so it carries none of the
 * interrupt's latency. `frame` stays valid only until the next callback.
 */
typedef void (*ads1299_frame_cb_t)(const uint8_t *frame, uint64_t ts_us);

/*
 * Begin continuous acquisition.
 *
 * Puts the part into RDATAC and hands the SPI start task to the caller, so
 * the DRDY edge can trigger the transfer through PPI with no CPU in the
 * loop. `cb` runs from the transfer-complete interrupt.
 */
int ads1299_stream_start(ads1299_frame_cb_t cb);

/* Stop acquisition and return the part to command mode. */
void ads1299_stream_stop(void);

/* Address of SPI TASKS_START, to hang off the same PPI channel as DRDY. */
uint32_t ads1299_start_task_addr(void);

/*
 * Frames whose transfer had not finished when the next DRDY arrived. Any
 * non-zero value means samples were lost.
 */
uint32_t ads1299_overruns(void);

/* Human-readable form of a probe result, for logging. */
const char *afe_probe_str(afe_probe_result_t r);

#endif /* SWIFTEEG_AFE_ADS1299_H */
