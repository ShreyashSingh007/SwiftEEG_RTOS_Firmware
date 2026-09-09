/*
 * RDATAC frame layout, kept free of Zephyr and nrfx so it can be tested on
 * the host.
 *
 * The ADS1299 sends 27 bytes per sample: three status bytes then eight
 * channels of 24-bit two's-complement data, most significant byte first.
 * This is the most data-critical code in the firmware - a sign-extension or
 * byte-order mistake here does not crash anything, it silently produces
 * plausible-looking EEG that is wrong.
 */
#ifndef SWIFTEEG_FRAME_H
#define SWIFTEEG_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define FRAME_STATUS_BYTES 3
#define FRAME_CHANNELS     8
#define FRAME_BYTES        (FRAME_STATUS_BYTES + FRAME_CHANNELS * 3)

/*
 * The top four status bits are hard-wired to 1100. They are the only
 * check available that a frame is aligned with the sample, which matters
 * because the transfer is started by hardware with no software in the loop.
 */
#define FRAME_STATUS_MARKER      0xC0u
#define FRAME_STATUS_MARKER_MASK 0xF0u

static inline bool frame_is_valid(const uint8_t *frame)
{
	return (frame[0] & FRAME_STATUS_MARKER_MASK) == FRAME_STATUS_MARKER;
}

/* The 24-bit status word: marker, then the lead-off and GPIO bits. */
static inline uint32_t frame_status(const uint8_t *frame)
{
	return ((uint32_t)frame[0] << 16) | ((uint32_t)frame[1] << 8) | frame[2];
}

/*
 * One channel, sign-extended from 24 to 32 bits.
 *
 * The shift-based sign extension is written out rather than relying on a
 * signed right shift, whose behaviour on negative values is
 * implementation-defined.
 */
static inline int32_t frame_channel(const uint8_t *frame, uint8_t ch)
{
	const uint8_t *p = &frame[FRAME_STATUS_BYTES + (size_t)ch * 3];
	const uint32_t raw = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];

	return (raw & 0x800000u) ? (int32_t)(raw | 0xFF000000u) : (int32_t)raw;
}

#endif /* SWIFTEEG_FRAME_H */
