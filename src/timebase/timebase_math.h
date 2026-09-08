/*
 * Timebase maths, kept free of Zephyr and nrfx so it can be unit tested on
 * the host. Hardware lives in timebase.c.
 */
#ifndef SWIFTEEG_TIMEBASE_MATH_H
#define SWIFTEEG_TIMEBASE_MATH_H

#include <stdbool.h>
#include <stdint.h>

/* TIMER1 runs at 1 MHz, so one tick is one microsecond. */
#define TIMEBASE_HZ 1000000U

/* Half the 32-bit range. See timebase_extend(). */
#define TIMEBASE_HALF UINT32_C(0x80000000)

/*
 * Combine the software wrap count with a 32-bit hardware counter reading.
 *
 * The hardware counter wraps every 2^32 us, about 71.6 minutes. A wrap
 * raises an event whose ISR increments `wraps`. `wrap_pending` says that
 * event has fired but the ISR has not run yet, so `wraps` is one behind.
 *
 * In that window the counter value tells us which period the reading belongs
 * to: near the top of the range it was taken before the wrap and is old, near
 * the bottom it was taken after and is new. Splitting at half the range means
 * only an ISR delayed by 35 minutes could confuse the two.
 */
static inline uint64_t timebase_extend(uint32_t wraps, uint32_t counter,
				       bool wrap_pending)
{
	if (wrap_pending && counter < TIMEBASE_HALF) {
		wraps++;
	}

	return ((uint64_t)wraps << 32) | counter;
}

#endif /* SWIFTEEG_TIMEBASE_MATH_H */
