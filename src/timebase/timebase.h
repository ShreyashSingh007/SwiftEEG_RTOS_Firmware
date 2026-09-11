/*
 * Sample timebase: a free-running 64-bit microsecond clock, plus the
 * hardware capture endpoint that timestamps DRDY without CPU involvement.
 *
 * TIMER1 at 1 MHz is the source. It is 32-bit in hardware and wraps every
 * 71.6 minutes; a wrap ISR extends it to 64 bits (see timebase_math.h).
 *
 * TIMER0, RTC0 and PPI channels 19/30/31 belong to MPSL and must not be
 * touched - TIMER1 is free on nRF52 (it is only reserved on nRF53).
 */
#ifndef SWIFTEEG_TIMEBASE_H
#define SWIFTEEG_TIMEBASE_H

#include <stdbool.h>
#include <stdint.h>

#include "timebase_math.h"

/*
 * Start the timebase. Requests HFXO, because the timer counts HFCLK: on the
 * internal RC oscillator the rate is off by ~1 %, on the crystal by ~20 ppm.
 * Returns 0, or a negative errno.
 */
int timebase_init(void);

/* Current time in microseconds since timebase_init(). */
uint64_t timebase_now_us(void);

/*
 * Extend the value latched in the capture register into a 64-bit timestamp.
 * Call from the ISR that runs after a captured event.
 */
uint64_t timebase_stamp_us(uint32_t capture);

/* Value latched at the last captured event. */
uint32_t timebase_capture_get(void);

/*
 * Address of TASKS_CAPTURE for the capture channel, for a PPI endpoint.
 * Wire the DRDY event to this and every sample is timestamped in hardware.
 */
uint32_t timebase_capture_task_addr(void);

/*
 * A second capture channel, for the IMU's interrupt line. Same clock as the
 * EEG samples, so the two streams line up to the timer's microsecond.
 */
uint32_t timebase_imu_capture_task_addr(void);
uint32_t timebase_imu_capture_get(void);

/*
 * Extend a capture latched a while ago - anything under 71 minutes - into a
 * 64-bit timestamp. Unlike timebase_stamp_us(), this stays right when the
 * counter wrapped between the capture and the call.
 */
uint64_t timebase_stamp_past_us(uint32_t capture);

/* True once HFXO has taken over, so the rate is crystal-accurate. */
bool timebase_hfxo_running(void);

#endif /* SWIFTEEG_TIMEBASE_H */
