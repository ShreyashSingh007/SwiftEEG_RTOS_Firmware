/*
 * LSM6DSV16X motion sensor: accelerometer and gyroscope, streamed alongside
 * the EEG and timestamped on the same clock.
 *
 * Motion-artifact removal needs to know when each motion sample happened
 * relative to each EEG sample, to well under a millisecond. So the sensor's
 * samples are placed on TIMER1, the clock DRDY is latched on:
 *
 *   - the sensor batches samples in its FIFO and raises INT2 when the FIFO
 *     reaches a watermark;
 *   - PPI latches that edge into TIMER1 in hardware, exactly as for DRDY;
 *   - the watermark falls on a known sample of the batch, so every sample is
 *     placed relative to it at the sensor's sample period;
 *   - that period is measured edge to edge against TIMER1's crystal, not
 *     taken from the sensor's own oscillator.
 *
 * Samples go out as PROTO_TYPE_IMU frames while the stream is enabled.
 */
#ifndef SWIFTEEG_IMU_H
#define SWIFTEEG_IMU_H

#include <stdbool.h>
#include <stdint.h>

/* IMU frame flags. */
#define IMU_FLAG_TIME_ESTIMATED 0x01 /* no watermark edge; timed from the poll */
#define IMU_FLAG_OVERRUN        0x02 /* the sensor FIFO overflowed before this */

struct imu_config {
	bool     enabled;
	uint16_t rate_hz;  /* 60, 120, 240, 480 or 960 */
	uint8_t  accel_g;  /* full scale: 2, 4, 8 or 16 */
	uint16_t gyro_dps; /* full scale: 125, 250, 500, 1000, 2000 or 4000 */
};

struct imu_stats {
	uint32_t samples;      /* read from the sensor */
	uint32_t frames;       /* handed to a link */
	uint32_t overruns;     /* times the sensor FIFO overflowed */
	uint32_t unpaired;     /* FIFO words without a partner, dropped */
	uint32_t extrapolated; /* batches placed from an earlier edge */
	uint32_t estimated;    /* batches timed from the poll, no edge yet */
	uint32_t period_us_q8; /* measured sample period, 1/256 us */
};

/*
 * Probe the sensor and start its thread. Returns 0, or -ENODEV when it is
 * not fitted - which is not an error for the rest of the firmware.
 */
int imu_init(void);

bool imu_present(void);

/*
 * Check and queue a configuration; the IMU thread applies it within one
 * poll. Returns 0, -EINVAL for a rate or range the sensor does not have, or
 * -ENODEV when there is no sensor.
 */
int imu_configure(const struct imu_config *cfg);

void imu_get_config(struct imu_config *out);
void imu_get_stats(struct imu_stats *out);

#endif /* SWIFTEEG_IMU_H */
