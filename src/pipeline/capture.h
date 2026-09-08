/*
 * Hardware-triggered sample capture.
 *
 * DRDY from the AFE is routed through GPIOTE and PPI straight into the
 * timebase capture register, so every sample is timestamped by hardware with
 * no CPU involvement and no interrupt jitter. The same PPI channel will also
 * start the SPI DMA transfer once that lands - one nRF52 PPI channel drives
 * one event and two tasks, which is exactly this topology.
 *
 * Measured on hardware at 250 SPS: intervals 3996-3998 us, i.e. 2 us
 * peak-to-peak, which is the 1 MHz timer's own resolution.
 *
 * There is deliberately no DRDY interrupt. Zephyr's GPIO driver owns the
 * GPIOTE interrupt line, and nothing here needs it: the capture is done in
 * hardware, and once SPI DMA is running its END interrupt is what wakes the
 * CPU, once per frame.
 */
#ifndef SWIFTEEG_CAPTURE_H
#define SWIFTEEG_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>

/* What the DRDY intervals looked like over a measurement window. */
struct capture_stats {
	uint32_t count;    /* DRDY events seen */
	uint32_t min_us;   /* shortest interval */
	uint32_t max_us;   /* longest interval */
	uint32_t total_us; /* first event to last, for the mean */
};

/*
 * Wire DRDY to the timebase capture register. Allocates one GPIOTE channel
 * and one PPI channel, both from the shared allocators so the channels MPSL
 * reserves for the radio are skipped. Returns 0, or a negative errno.
 */
int capture_init(void);

/*
 * Measure DRDY intervals by polling the capture register for `ms`.
 *
 * Polling, not interrupts: the timestamp is latched by hardware, so watching
 * the register move tests DRDY, GPIOTE and PPI end to end without depending
 * on an interrupt at all. Blocks for the full duration.
 */
void capture_measure(uint32_t ms, struct capture_stats *out);

/*
 * Hang a second task off the DRDY channel, so one edge both timestamps the
 * sample and starts its SPI transfer. nRF52 PPI allows one event and two
 * tasks per channel, which is exactly the two we need.
 */
int capture_attach_task(uint32_t task_addr);

/*
 * Enable or disable the DRDY trigger.
 *
 * Disabling stops the edge from reaching either task, so no transfer can
 * start behind the CPU's back. Anything that talks to the AFE over SPI
 * outside the streaming path has to do this first, or its transfer collides
 * with a hardware-started one and both are corrupted.
 */
void capture_set_trigger(bool on);

/* Timestamp of the most recent DRDY, in microseconds. */
uint64_t capture_last_us(void);

#endif /* SWIFTEEG_CAPTURE_H */
