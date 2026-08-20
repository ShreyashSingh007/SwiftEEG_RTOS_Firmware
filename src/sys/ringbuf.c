#include "ringbuf.h"

#include <string.h>

static inline bool is_power_of_two(size_t v)
{
	return v != 0 && (v & (v - 1)) == 0;
}

bool spsc_init(spsc_ring_t *r, void *storage, size_t elem_size, size_t capacity)
{
	if (r == NULL || storage == NULL || elem_size == 0) {
		return false;
	}
	if (!is_power_of_two(capacity)) {
		return false;
	}

	r->buf = (uint8_t *)storage;
	r->elem_size = elem_size;
	r->capacity = capacity;
	r->mask = capacity - 1;

	atomic_init(&r->head, 0);
	atomic_init(&r->tail, 0);
	atomic_init(&r->dropped, 0);

	return true;
}

bool spsc_push(spsc_ring_t *r, const void *elem)
{
	if (r == NULL || elem == NULL) {
		return false;
	}

	/* Producer owns head, so a relaxed load of it is fine. */
	const size_t head = atomic_load_explicit(&r->head, memory_order_relaxed);
	/*
	 * Acquire on tail: pairs with the consumer's release in pop/release,
	 * so slots it has freed are visible before we reuse them.
	 */
	const size_t tail = atomic_load_explicit(&r->tail, memory_order_acquire);

	if ((head - tail) >= r->capacity) {
		atomic_fetch_add_explicit(&r->dropped, 1, memory_order_relaxed);
		return false;
	}

	memcpy(&r->buf[(head & r->mask) * r->elem_size], elem, r->elem_size);

	/*
	 * Release: the record above must be visible to the consumer before it
	 * can observe the new head. Without this the consumer could read a
	 * slot the producer has not finished writing.
	 */
	atomic_store_explicit(&r->head, head + 1, memory_order_release);
	return true;
}

bool spsc_pop(spsc_ring_t *r, void *out)
{
	if (r == NULL || out == NULL) {
		return false;
	}

	const size_t tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
	const size_t head = atomic_load_explicit(&r->head, memory_order_acquire);

	if (head == tail) {
		return false;
	}

	memcpy(out, &r->buf[(tail & r->mask) * r->elem_size], r->elem_size);

	atomic_store_explicit(&r->tail, tail + 1, memory_order_release);
	return true;
}

const void *spsc_peek(const spsc_ring_t *r)
{
	if (r == NULL) {
		return NULL;
	}

	const size_t tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
	const size_t head = atomic_load_explicit(&r->head, memory_order_acquire);

	if (head == tail) {
		return NULL;
	}

	return &r->buf[(tail & r->mask) * r->elem_size];
}

void spsc_release(spsc_ring_t *r)
{
	if (r == NULL) {
		return;
	}

	const size_t tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
	const size_t head = atomic_load_explicit(&r->head, memory_order_acquire);

	if (head == tail) {
		return; /* nothing was held */
	}

	atomic_store_explicit(&r->tail, tail + 1, memory_order_release);
}

size_t spsc_count(const spsc_ring_t *r)
{
	if (r == NULL) {
		return 0;
	}

	const size_t head = atomic_load_explicit(&r->head, memory_order_acquire);
	const size_t tail = atomic_load_explicit(&r->tail, memory_order_acquire);

	return head - tail;
}

bool spsc_is_empty(const spsc_ring_t *r)
{
	return spsc_count(r) == 0;
}

bool spsc_is_full(const spsc_ring_t *r)
{
	return r != NULL && spsc_count(r) >= r->capacity;
}

uint32_t spsc_dropped(const spsc_ring_t *r)
{
	if (r == NULL) {
		return 0;
	}
	return (uint32_t)atomic_load_explicit(&r->dropped, memory_order_relaxed);
}

void spsc_reset_dropped(spsc_ring_t *r)
{
	if (r != NULL) {
		atomic_store_explicit(&r->dropped, 0, memory_order_relaxed);
	}
}
