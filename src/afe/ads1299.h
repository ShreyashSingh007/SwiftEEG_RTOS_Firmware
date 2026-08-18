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

/* Human-readable form of a probe result, for logging. */
const char *afe_probe_str(afe_probe_result_t r);

#endif /* SWIFTEEG_AFE_ADS1299_H */
