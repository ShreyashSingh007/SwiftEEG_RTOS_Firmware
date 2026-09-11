#include "timebase.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_timer.h>

LOG_MODULE_REGISTER(timebase, CONFIG_LOG_DEFAULT_LEVEL);

#define TB_NODE DT_NODELABEL(timer1)
#define TB_TIMER NRF_TIMER1

/*
 * Compare/capture channel roles:
 *   0  latched from DRDY by PPI - the EEG sample timestamp
 *   1  wrap detector, fires when the counter rolls over to 0
 *   2  scratch for timebase_now_us()
 *   3  latched from the IMU's INT2 by PPI - motion sample timestamps
 */
#define CC_CAPTURE 0
#define CC_WRAP    1
#define CC_NOW     2
#define CC_IMU     3

#define WRAP_EVENT NRF_TIMER_EVENT_COMPARE1
#define WRAP_INT   NRF_TIMER_INT_COMPARE1_MASK

/* 16 MHz HFCLK / 2^4 = 1 MHz. */
#define TB_PRESCALER 4

/* Written only by the wrap ISR, read everywhere. */
static volatile uint32_t tb_wraps;

/* timebase_now_us() shares CC_NOW between callers. */
static struct k_spinlock tb_lock;

static void tb_isr(const void *arg)
{
	ARG_UNUSED(arg);

	if (nrf_timer_event_check(TB_TIMER, WRAP_EVENT)) {
		nrf_timer_event_clear(TB_TIMER, WRAP_EVENT);
		tb_wraps++;
	}
}

/*
 * Combine a counter reading with the wrap count.
 *
 * The wrap count and the pending flag must agree with each other, and the ISR
 * can land between the two reads. Reading the count either side of the flag
 * detects that: if it changed, the pair is inconsistent, so try again.
 */
static uint64_t tb_stamp(uint32_t counter)
{
	uint32_t w1, w2;
	bool pending;

	do {
		w1 = tb_wraps;
		pending = nrf_timer_event_check(TB_TIMER, WRAP_EVENT);
		w2 = tb_wraps;
	} while (w1 != w2);

	return timebase_extend(w1, counter, pending);
}

int timebase_init(void)
{
	const struct device *clk = DEVICE_DT_GET_ONE(nordic_nrf_clock);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock controller not ready");
		return -ENODEV;
	}

	/*
	 * Ask for HFXO. This is a request, not a switch: MPSL also drives the
	 * crystal for the radio, and the clock controller reference counts.
	 */
	int err = clock_control_on(clk, CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (err < 0 && err != -EALREADY && err != -EINPROGRESS) {
		LOG_ERR("HFXO request failed (%d)", err);
		return err;
	}

	nrf_timer_task_trigger(TB_TIMER, NRF_TIMER_TASK_STOP);
	nrf_timer_task_trigger(TB_TIMER, NRF_TIMER_TASK_CLEAR);
	nrf_timer_mode_set(TB_TIMER, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(TB_TIMER, NRF_TIMER_BIT_WIDTH_32);
	nrf_timer_prescaler_set(TB_TIMER, TB_PRESCALER);

	/*
	 * Wrap detector. COMPARE fires when the counter increments *to* the
	 * value, so a compare of 0 fires on rollover from 0xFFFFFFFF and not
	 * when the counter is cleared here.
	 */
	nrf_timer_cc_set(TB_TIMER, CC_WRAP, 0);
	nrf_timer_event_clear(TB_TIMER, WRAP_EVENT);
	tb_wraps = 0;

	IRQ_CONNECT(DT_IRQN(TB_NODE), DT_IRQ(TB_NODE, priority), tb_isr, NULL, 0);
	irq_enable(DT_IRQN(TB_NODE));
	nrf_timer_int_enable(TB_TIMER, WRAP_INT);

	nrf_timer_task_trigger(TB_TIMER, NRF_TIMER_TASK_START);

	LOG_INF("timebase up: TIMER1 at %u Hz, HFXO %s", TIMEBASE_HZ,
		timebase_hfxo_running() ? "running" : "starting");

	return 0;
}

uint64_t timebase_now_us(void)
{
	k_spinlock_key_t key = k_spin_lock(&tb_lock);

	nrf_timer_task_trigger(TB_TIMER, nrf_timer_capture_task_get(CC_NOW));
	uint32_t counter = nrf_timer_cc_get(TB_TIMER, CC_NOW);

	k_spin_unlock(&tb_lock, key);

	return tb_stamp(counter);
}

uint64_t timebase_stamp_us(uint32_t capture)
{
	return tb_stamp(capture);
}

uint64_t timebase_stamp_past_us(uint32_t capture)
{
	const uint64_t now = timebase_now_us();
	uint64_t t = (now & ~(uint64_t)UINT32_MAX) | capture;

	/* Later than now means the capture came before the most recent wrap. */
	if (t > now) {
		t -= (uint64_t)1 << 32;
	}

	return t;
}

uint32_t timebase_capture_get(void)
{
	return nrf_timer_cc_get(TB_TIMER, CC_CAPTURE);
}

uint32_t timebase_capture_task_addr(void)
{
	return nrf_timer_task_address_get(TB_TIMER,
					  nrf_timer_capture_task_get(CC_CAPTURE));
}

uint32_t timebase_imu_capture_get(void)
{
	return nrf_timer_cc_get(TB_TIMER, CC_IMU);
}

uint32_t timebase_imu_capture_task_addr(void)
{
	return nrf_timer_task_address_get(TB_TIMER,
					  nrf_timer_capture_task_get(CC_IMU));
}

bool timebase_hfxo_running(void)
{
	const struct device *clk = DEVICE_DT_GET_ONE(nordic_nrf_clock);
	enum clock_control_status st =
		clock_control_get_status(clk, CLOCK_CONTROL_NRF_SUBSYS_HF);

	return st == CLOCK_CONTROL_STATUS_ON;
}
