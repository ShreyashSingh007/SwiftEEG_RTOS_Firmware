#include "proto.h"

#include <string.h>

/* --- little endian helpers, written out so the codec stays endian-safe --- */

static inline void put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)(v >> 8);
}

static inline uint16_t get_le16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint16_t proto_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFFu;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int b = 0; b < 8; b++) {
			if (crc & 0x8000u) {
				crc = (uint16_t)((crc << 1) ^ 0x1021u);
			} else {
				crc = (uint16_t)(crc << 1);
			}
		}
	}

	return crc;
}

static bool type_is_valid(uint8_t type)
{
	switch (type) {
	case PROTO_TYPE_CMD:
	case PROTO_TYPE_RSP:
	case PROTO_TYPE_EVT:
	case PROTO_TYPE_DATA:
		return true;
	default:
		return false;
	}
}

int proto_encode(uint8_t type, uint8_t flags, uint16_t seq,
		 const void *payload, uint16_t payload_len,
		 uint8_t *dst, size_t dst_cap)
{
	if (dst == NULL) {
		return PROTO_ERR_ARG;
	}
	if (payload_len > PROTO_MAX_PAYLOAD) {
		return PROTO_ERR_ARG;
	}
	if (payload_len > 0 && payload == NULL) {
		return PROTO_ERR_ARG;
	}
	if (!type_is_valid(type)) {
		return PROTO_ERR_TYPE;
	}

	const size_t total = (size_t)PROTO_OVERHEAD + payload_len;
	if (dst_cap < total) {
		return PROTO_ERR_SPACE;
	}

	dst[0] = PROTO_SOF;
	dst[1] = PROTO_VERSION;
	dst[2] = type;
	dst[3] = flags;
	put_le16(&dst[4], payload_len);
	put_le16(&dst[6], seq);

	if (payload_len > 0) {
		memcpy(&dst[PROTO_HEADER_LEN], payload, payload_len);
	}

	/* CRC covers everything except the SOF byte. */
	const uint16_t crc = proto_crc16(&dst[1],
					 (size_t)(PROTO_HEADER_LEN - 1) + payload_len);
	put_le16(&dst[PROTO_HEADER_LEN + payload_len], crc);

	return (int)total;
}

int proto_decode(const uint8_t *src, size_t len, proto_frame_t *out)
{
	if (src == NULL || out == NULL) {
		return PROTO_ERR_ARG;
	}
	if (len < PROTO_OVERHEAD) {
		return PROTO_ERR_ARG;
	}
	if (src[0] != PROTO_SOF) {
		return PROTO_ERR_ARG;
	}
	if (src[1] != PROTO_VERSION) {
		return PROTO_ERR_VERSION;
	}
	if (!type_is_valid(src[2])) {
		return PROTO_ERR_TYPE;
	}

	const uint16_t payload_len = get_le16(&src[4]);
	if (payload_len > PROTO_MAX_PAYLOAD) {
		return PROTO_ERR_ARG;
	}
	if (len < (size_t)PROTO_OVERHEAD + payload_len) {
		return PROTO_ERR_ARG;
	}

	const uint16_t want = proto_crc16(&src[1],
					  (size_t)(PROTO_HEADER_LEN - 1) + payload_len);
	const uint16_t got = get_le16(&src[PROTO_HEADER_LEN + payload_len]);
	if (want != got) {
		return PROTO_ERR_CRC;
	}

	out->version = src[1];
	out->type    = src[2];
	out->flags   = src[3];
	out->len     = payload_len;
	out->seq     = get_le16(&src[6]);
	out->payload = payload_len ? &src[PROTO_HEADER_LEN] : NULL;

	return PROTO_OK;
}

/* --- incremental stream decoder ---------------------------------------- */

void proto_stream_reset(proto_stream_t *st)
{
	if (st == NULL) {
		return;
	}
	memset(st, 0, sizeof(*st));
}

/*
 * Drops the first byte of a bad frame and rescans what is left for a new SOF.
 *
 * Rescanning the remainder rather than discarding the whole buffer matters:
 * a real SOF can legitimately appear inside the bytes we already consumed,
 * and throwing them away would lose a good frame that followed a bad one.
 */
static void resync(proto_stream_t *st)
{
	st->resyncs++;

	size_t start = 1;
	while (start < st->have && st->buf[start] != PROTO_SOF) {
		start++;
	}

	if (start >= st->have) {
		st->have = 0;
		st->in_frame = false;
		st->need = 0;
		return;
	}

	const uint16_t remaining = (uint16_t)(st->have - start);
	memmove(st->buf, &st->buf[start], remaining);
	st->have = remaining;
	st->in_frame = true;
	st->need = 0;
}

bool proto_stream_push(proto_stream_t *st, uint8_t byte, proto_frame_t *out)
{
	if (st == NULL || out == NULL) {
		return false;
	}

	if (!st->in_frame) {
		if (byte != PROTO_SOF) {
			return false; /* still hunting for a frame start */
		}
		st->in_frame = true;
		st->have = 0;
		st->need = 0;
	}

	if (st->have >= sizeof(st->buf)) {
		resync(st);
		return false;
	}

	st->buf[st->have++] = byte;

	/* Once the header is in, the total length is known. */
	if (st->need == 0 && st->have >= PROTO_HEADER_LEN) {
		const uint16_t payload_len = get_le16(&st->buf[4]);

		if (st->buf[1] != PROTO_VERSION ||
		    !type_is_valid(st->buf[2]) ||
		    payload_len > PROTO_MAX_PAYLOAD) {
			/* Header is not plausible - this was not a real SOF. */
			resync(st);
			return false;
		}

		st->need = (uint16_t)(PROTO_OVERHEAD + payload_len);
	}

	if (st->need == 0 || st->have < st->need) {
		return false; /* still collecting */
	}

	const int err = proto_decode(st->buf, st->have, out);

	if (err == PROTO_OK) {
		st->have = 0;
		st->need = 0;
		st->in_frame = false;
		return true;
	}

	if (err == PROTO_ERR_CRC) {
		st->crc_errors++;
	}
	resync(st);
	return false;
}
