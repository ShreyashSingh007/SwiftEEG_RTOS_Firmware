/*
 * Timebase wrap maths.
 *
 * The hardware counter is 32-bit and rolls over every 71.6 minutes. The one
 * property that matters is that timestamps never go backwards across that
 * rollover, including in the window where the wrap event has fired but its
 * ISR has not run yet.
 */

#include <zephyr/ztest.h>

#include "timebase/timebase_math.h"

#define HI(n) ((uint64_t)(n) << 32)

ZTEST(timebase, test_plain_value_passes_through)
{
	zassert_equal(timebase_extend(0, 0, false), 0);
	zassert_equal(timebase_extend(0, 12345, false), 12345);
	zassert_equal(timebase_extend(0, UINT32_MAX, false), UINT32_MAX);
}

ZTEST(timebase, test_wrap_count_becomes_the_high_word)
{
	zassert_equal(timebase_extend(1, 0, false), HI(1));
	zassert_equal(timebase_extend(7, 42, false), HI(7) + 42);
	zassert_equal(timebase_extend(UINT32_MAX, 0, false), HI(UINT32_MAX));
}

ZTEST(timebase, test_pending_wrap_promotes_a_low_counter)
{
	/* Read taken after the rollover, before the ISR caught up. */
	zassert_equal(timebase_extend(0, 0, true), HI(1));
	zassert_equal(timebase_extend(0, 1000, true), HI(1) + 1000);
	zassert_equal(timebase_extend(3, 500, true), HI(4) + 500);
}

ZTEST(timebase, test_pending_wrap_leaves_a_high_counter_alone)
{
	/* Read taken before the rollover; it belongs to the old period. */
	zassert_equal(timebase_extend(0, UINT32_MAX, true), UINT32_MAX);
	zassert_equal(timebase_extend(2, 0xFFFF0000, true), HI(2) + 0xFFFF0000);
}

ZTEST(timebase, test_half_range_is_the_split_point)
{
	/* Exactly half counts as old, one below as new. */
	zassert_equal(timebase_extend(0, TIMEBASE_HALF, true), TIMEBASE_HALF);
	zassert_equal(timebase_extend(0, TIMEBASE_HALF - 1, true),
		      HI(1) + TIMEBASE_HALF - 1);
}

ZTEST(timebase, test_both_orderings_agree)
{
	/*
	 * The ISR may run before or after the counter is read. Either way the
	 * same instant must produce the same timestamp - that equivalence is
	 * what makes the pending flag safe to use.
	 */
	const uint32_t counter = 250;

	const uint64_t isr_late = timebase_extend(4, counter, true);
	const uint64_t isr_early = timebase_extend(5, counter, false);

	zassert_equal(isr_late, isr_early,
		      "orderings disagree: %llu vs %llu", isr_late, isr_early);
}

ZTEST(timebase, test_extend_past_resolves_what_extend_alone_cannot)
{
	/*
	 * R1-ACQ-01: unlike test_both_orderings_agree above, a HIGH counter
	 * (pre-wrap) does NOT agree once the ISR has already run - pending is
	 * false either way, so timebase_extend() cannot tell "genuinely in the
	 * new period" from "stale, from just before the wrap" apart:
	 */
	const uint32_t counter = 0xFFFFFF00;

	const uint64_t isr_not_run_yet = timebase_extend(4, counter, true);
	const uint64_t isr_already_ran = timebase_extend(5, counter, false);

	zassert_not_equal(isr_not_run_yet, isr_already_ran,
			  "if this starts passing, timebase_extend() alone has "
			  "become sufficient and the _from_isr past-rule path "
			  "may no longer be needed");

	/*
	 * timebase_extend_past() is what the SPIM3 END ISR now uses
	 * (timebase_stamp_past_us_from_isr()) to resolve exactly this case: a
	 * capture latched ~32 us before a wrap (a live DRDY timestamp), read out
	 * after the wrap ISR has already bumped the high word and moved on.
	 */
	const uint64_t now = timebase_extend(5, 0x00000060, false);

	zassert_equal(timebase_extend_past(now, counter), HI(4) + counter,
		      "a capture from just before the wrap must stay in the "
		      "old epoch, not jump 2^32 us into the future");
}

ZTEST(timebase, test_extend_past_leaves_the_same_epoch_alone)
{
	/* No wrap in the window: an ordinary capture is unaffected. */
	const uint64_t now = timebase_extend(4, 500000, false);

	zassert_equal(timebase_extend_past(now, 100), HI(4) + 100);
	zassert_equal(timebase_extend_past(now, 500000), now,
		      "a capture equal to now must not be pushed into the past");
}

ZTEST(timebase, test_never_goes_backwards_across_a_wrap)
{
	/*
	 * Walk a realistic sequence through a rollover: late in the old
	 * period, then the pending window, then after the ISR has run.
	 */
	const struct {
		uint32_t wraps;
		uint32_t counter;
		bool pending;
	} seq[] = {
		{ 0, UINT32_MAX - 2000, false },
		{ 0, UINT32_MAX - 1000, false },
		{ 0, UINT32_MAX,        false },
		{ 0, 0,                 true  },  /* wrapped, ISR not yet run */
		{ 0, 500,               true  },
		{ 1, 1000,              false },  /* ISR has run */
		{ 1, 2000,              false },
	};

	uint64_t prev = 0;

	for (size_t i = 0; i < ARRAY_SIZE(seq); i++) {
		uint64_t now = timebase_extend(seq[i].wraps, seq[i].counter,
					       seq[i].pending);

		zassert_true(now > prev,
			     "step %zu went backwards: %llu after %llu",
			     i, now, prev);
		prev = now;
	}
}

ZTEST(timebase, test_wrap_period_is_what_we_claim)
{
	/*
	 * One wrap is 2^32 ticks. At 1 MHz that is 4295 seconds, the 71.6
	 * minutes quoted in the header - assert it rather than trusting the
	 * comment.
	 */
	const uint64_t period_us = HI(1);
	const uint64_t period_s = period_us / TIMEBASE_HZ;

	zassert_equal(TIMEBASE_HZ, 1000000U);
	zassert_within(period_s, 4295, 1, "wrap period is %llu s", period_s);
}

ZTEST_SUITE(timebase, NULL, NULL, NULL, NULL, NULL);
