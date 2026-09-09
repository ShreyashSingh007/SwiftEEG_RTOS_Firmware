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
#define ADS1299_REG_BIAS_SENSP 0x0D
#define ADS1299_REG_BIAS_SENSN 0x0E
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

/*
 * CONFIG2 drives the built-in test generator. Bits 7:6 read back as 1.
 *
 *   bit 4  INT_CAL    1 = generate the signal internally
 *   bit 2  CAL_AMP    0 = 1 x (VREFP-VREFN)/2400, 1 = twice that
 *   bits 1:0 CAL_FREQ 00 = fCLK/2^21, 01 = fCLK/2^20, 11 = DC
 *
 * With the internal 2.048 MHz oscillator and the internal 4.5 V reference,
 * CAL_FREQ 00 gives a ~0.98 Hz square wave of +/-1.875 mV at the inputs.
 * That is a known amplitude and a known frequency, which is what makes it
 * worth anything as a check.
 */
#define ADS1299_CONFIG2_BASE     0xC0
#define ADS1299_CONFIG2_INT_CAL  0x10
#define ADS1299_CONFIG2_CAL_AMP  0x04
#define ADS1299_CAL_FREQ_DIV21   0x00	/* ~0.98 Hz */
#define ADS1299_CAL_FREQ_DIV20   0x01	/* ~1.95 Hz */
#define ADS1299_CAL_FREQ_DC      0x03

/*
 * Test signal amplitude at the input, in nanovolts: (4.5 / 2400) volts.
 * The square wave swings either side of zero, so peak-to-peak is twice this.
 */
#define ADS1299_CAL_AMPLITUDE_NV 1875000

/*
 * CONFIG3 bit 2, PD_BIAS: powers the bias amplifier - the driven right leg.
 *
 * On this board BIASOUT goes to the right mastoid and SRB1 to the left, so
 * the amplifier senses the common-mode across the scalp electrodes and
 * drives its inverse into the head. That is what keeps mains out of the
 * signal; a notch afterwards only removes what is still separable, and
 * whatever the front end could not reject as common-mode is already gone.
 *
 * BIASREF is grounded on this board, which is correct - mid-supply is 0 V
 * between the +/-2.5 V rails - so BIASREF_INT (bit 3) stays 0.
 */
#define ADS1299_CONFIG3_PD_BIAS 0x04

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

/*
 * Switch every channel to the internal test generator, or back to the
 * electrodes. The generated square wave has a known amplitude and frequency,
 * so what comes out the far end of the DSP chain can be checked rather than
 * eyeballed.
 */
int ads1299_test_signal(bool on, uint8_t cal_freq);

/*
 * Point every channel at one of the ADS1299_MUX_* sources, handling the mode
 * switch the part needs for a register write. Selecting the test source also
 * enables the generator in CONFIG2; anything else disables it.
 *
 * MUX_SHORTED is how the noise floor is measured: the inputs are tied
 * together internally, so what comes out is the front end's own noise and
 * nothing from the electrodes.
 */
int ads1299_set_input(uint8_t mux, uint8_t cal_freq);

/*
 * Configure the bias drive.
 *
 * `sensp` and `sensn` are bitmasks choosing which channels the amplifier
 * derives the common-mode from. The default senses all eight positive
 * inputs, the scalp electrodes, and none of the negative ones - those are
 * all tied to SRB1, so including them would just weight the reference
 * electrode eight times over.
 */
int ads1299_set_bias(bool enable, uint8_t sensp, uint8_t sensn);

/*
 * Configure one channel, or every channel when `ch` is 0xFF.
 * `power_down` parks an unused channel; `srb2` routes its negative input to
 * SRB2 instead of SRB1.
 */
int ads1299_set_channel(uint8_t ch, uint8_t gain, uint8_t mux, bool power_down,
			bool srb2);

/* START and STOP opcodes. Conversions run between them. */
int ads1299_start_conversions(void);
int ads1299_stop_conversions(void);

/* Read one register. Only valid when the part is already in command mode. */
int ads1299_read_reg(uint8_t addr, uint8_t *val);

/*
 * Read one register from anywhere, including mid-acquisition: drops the part
 * into command mode, reads, and puts it back. Registers read back as zero in
 * continuous-read mode, so the plain version silently lies while streaming.
 */
int ads1299_read_reg_safe(uint8_t addr, uint8_t *val);

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

/*
 * Hook the driver uses to stop DRDY starting transfers while it talks to the
 * part directly. Whoever wired the trigger installs this; without it, any
 * register access during acquisition races a hardware-started transfer.
 */
typedef void (*ads1299_trigger_gate_t)(bool enable);
void ads1299_set_trigger_gate(ads1299_trigger_gate_t gate);

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
