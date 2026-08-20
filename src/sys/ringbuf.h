/*
 * Lock-free single-producer / single-consumer ring buffer.
 *
 * This is the join between stages of the acquisition pipeline:
 *
 *   SPIM3 END ISR  --push-->  raw ring  --pop-->  DSP thread
 *   DSP thread     --push-->  processed ring  --pop-->  transport
 *
 * Exactly one producer and one consumer per instance. That constraint is what
 * makes it lock-free without a CAS loop: the producer owns head, the consumer
 * owns tail, and neither ever writes the other's index. No mutex is taken on
 * the acquisition path, so the ISR cannot be delayed by a thread holding a
 * lock.
 *
 * Records are fixed size, which suits sample frames and avoids the partial
 * read/write problems a byte-oriented ring would bring.
 *
 * Overflow drops the NEWEST record and counts it, rather than blocking or
 * overwriting unread data. Acquisition must never stall waiting for a slow
 * consumer, and silently corrupting already-queued samples would be worse
 * than losing a known, counted number of new ones.
 *
 * Depends only on the C standard library so it is testable off-target.
 */
#ifndef SWIFTEEG_SYS_RINGBUF_H
#define SWIFTEEG_SYS_RINGBUF_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint8_t *buf;        /* capacity * elem_size bytes */
	size_t   elem_size;
	size_t   capacity;   /* record count, MUST be a power of two */
	size_t   mask;       /* capacity - 1 */

	/*
	 * Free-running counters, masked on use. Letting them wrap naturally
	 * means a full buffer is (head - tail == capacity), so all capacity
	 * slots are usable - the usual "keep one slot empty" trick is not
	 * needed.
	 */
	atomic_size_t head;  /* producer only */
	atomic_size_t tail;  /* consumer only */

	atomic_uint_fast32_t dropped; /* records refused because it was full */
} spsc_ring_t;

/*
 * Binds storage to a ring. `capacity` must be a power of two, and `storage`
 * must be at least capacity * elem_size bytes.
 * Returns false on a bad argument.
 */
bool spsc_init(spsc_ring_t *r, void *storage, size_t elem_size, size_t capacity);

/* Producer side. Returns false and counts a drop if full. */
bool spsc_push(spsc_ring_t *r, const void *elem);

/* Consumer side. Returns false if empty. */
bool spsc_pop(spsc_ring_t *r, void *out);

/*
 * Consumer side, zero-copy: returns a pointer to the oldest record without
 * releasing it, or NULL if empty. Call spsc_release() when finished with it.
 * Avoids a memcpy on the hot path.
 */
const void *spsc_peek(const spsc_ring_t *r);

/* Releases the record most recently returned by spsc_peek(). */
void spsc_release(spsc_ring_t *r);

/* Snapshot counts. Safe to call from either side; inherently racy by nature. */
size_t spsc_count(const spsc_ring_t *r);
bool spsc_is_empty(const spsc_ring_t *r);
bool spsc_is_full(const spsc_ring_t *r);

/* Total records refused because the ring was full. */
uint32_t spsc_dropped(const spsc_ring_t *r);
void spsc_reset_dropped(spsc_ring_t *r);

#endif /* SWIFTEEG_SYS_RINGBUF_H */
