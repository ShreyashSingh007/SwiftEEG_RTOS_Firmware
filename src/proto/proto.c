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

/*
 * CRC-16/CCITT-FALSE (poly 0x1021), one entry per possible byte value.
 * table[b] is what the bit-by-bit algorithm produces for (crc >> 8) ^ b
 * shifted through all 8 bits with crc's low byte held at 0 - see
 * tools/proto_ref.py's crc16() (binascii.crc_hqx, the same polynomial and
 * table) for the reference this was generated and cross-checked against.
 * This runs on the DSP thread once per DATA frame (R3-DSP-12), where the
 * previous bit-at-a-time loop cost ~60-80 us of CRC alone per RAW_UV sample;
 * the table trades 512 B of flash for one lookup a byte.
 */
static const uint16_t crc16_table[256] = {
	0x0000u, 0x1021u, 0x2042u, 0x3063u, 0x4084u, 0x50a5u, 0x60c6u, 0x70e7u,
	0x8108u, 0x9129u, 0xa14au, 0xb16bu, 0xc18cu, 0xd1adu, 0xe1ceu, 0xf1efu,
	0x1231u, 0x0210u, 0x3273u, 0x2252u, 0x52b5u, 0x4294u, 0x72f7u, 0x62d6u,
	0x9339u, 0x8318u, 0xb37bu, 0xa35au, 0xd3bdu, 0xc39cu, 0xf3ffu, 0xe3deu,
	0x2462u, 0x3443u, 0x0420u, 0x1401u, 0x64e6u, 0x74c7u, 0x44a4u, 0x5485u,
	0xa56au, 0xb54bu, 0x8528u, 0x9509u, 0xe5eeu, 0xf5cfu, 0xc5acu, 0xd58du,
	0x3653u, 0x2672u, 0x1611u, 0x0630u, 0x76d7u, 0x66f6u, 0x5695u, 0x46b4u,
	0xb75bu, 0xa77au, 0x9719u, 0x8738u, 0xf7dfu, 0xe7feu, 0xd79du, 0xc7bcu,
	0x48c4u, 0x58e5u, 0x6886u, 0x78a7u, 0x0840u, 0x1861u, 0x2802u, 0x3823u,
	0xc9ccu, 0xd9edu, 0xe98eu, 0xf9afu, 0x8948u, 0x9969u, 0xa90au, 0xb92bu,
	0x5af5u, 0x4ad4u, 0x7ab7u, 0x6a96u, 0x1a71u, 0x0a50u, 0x3a33u, 0x2a12u,
	0xdbfdu, 0xcbdcu, 0xfbbfu, 0xeb9eu, 0x9b79u, 0x8b58u, 0xbb3bu, 0xab1au,
	0x6ca6u, 0x7c87u, 0x4ce4u, 0x5cc5u, 0x2c22u, 0x3c03u, 0x0c60u, 0x1c41u,
	0xedaeu, 0xfd8fu, 0xcdecu, 0xddcdu, 0xad2au, 0xbd0bu, 0x8d68u, 0x9d49u,
	0x7e97u, 0x6eb6u, 0x5ed5u, 0x4ef4u, 0x3e13u, 0x2e32u, 0x1e51u, 0x0e70u,
	0xff9fu, 0xefbeu, 0xdfddu, 0xcffcu, 0xbf1bu, 0xaf3au, 0x9f59u, 0x8f78u,
	0x9188u, 0x81a9u, 0xb1cau, 0xa1ebu, 0xd10cu, 0xc12du, 0xf14eu, 0xe16fu,
	0x1080u, 0x00a1u, 0x30c2u, 0x20e3u, 0x5004u, 0x4025u, 0x7046u, 0x6067u,
	0x83b9u, 0x9398u, 0xa3fbu, 0xb3dau, 0xc33du, 0xd31cu, 0xe37fu, 0xf35eu,
	0x02b1u, 0x1290u, 0x22f3u, 0x32d2u, 0x4235u, 0x5214u, 0x6277u, 0x7256u,
	0xb5eau, 0xa5cbu, 0x95a8u, 0x8589u, 0xf56eu, 0xe54fu, 0xd52cu, 0xc50du,
	0x34e2u, 0x24c3u, 0x14a0u, 0x0481u, 0x7466u, 0x6447u, 0x5424u, 0x4405u,
	0xa7dbu, 0xb7fau, 0x8799u, 0x97b8u, 0xe75fu, 0xf77eu, 0xc71du, 0xd73cu,
	0x26d3u, 0x36f2u, 0x0691u, 0x16b0u, 0x6657u, 0x7676u, 0x4615u, 0x5634u,
	0xd94cu, 0xc96du, 0xf90eu, 0xe92fu, 0x99c8u, 0x89e9u, 0xb98au, 0xa9abu,
	0x5844u, 0x4865u, 0x7806u, 0x6827u, 0x18c0u, 0x08e1u, 0x3882u, 0x28a3u,
	0xcb7du, 0xdb5cu, 0xeb3fu, 0xfb1eu, 0x8bf9u, 0x9bd8u, 0xabbbu, 0xbb9au,
	0x4a75u, 0x5a54u, 0x6a37u, 0x7a16u, 0x0af1u, 0x1ad0u, 0x2ab3u, 0x3a92u,
	0xfd2eu, 0xed0fu, 0xdd6cu, 0xcd4du, 0xbdaau, 0xad8bu, 0x9de8u, 0x8dc9u,
	0x7c26u, 0x6c07u, 0x5c64u, 0x4c45u, 0x3ca2u, 0x2c83u, 0x1ce0u, 0x0cc1u,
	0xef1fu, 0xff3eu, 0xcf5du, 0xdf7cu, 0xaf9bu, 0xbfbau, 0x8fd9u, 0x9ff8u,
	0x6e17u, 0x7e36u, 0x4e55u, 0x5e74u, 0x2e93u, 0x3eb2u, 0x0ed1u, 0x1ef0u,
};

uint16_t proto_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFFu;

	for (size_t i = 0; i < len; i++) {
		crc = (uint16_t)((crc << 8) ^ crc16_table[(uint8_t)((crc >> 8) ^ data[i])]);
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
	case PROTO_TYPE_IMU:
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

/* Removes `count` leading bytes, keeping anything that follows them. */
static void consume(proto_stream_t *st, uint16_t count)
{
	if (count >= st->have) {
		st->have = 0;
		st->in_frame = false;
		st->need = 0;
		return;
	}

	const uint16_t remaining = (uint16_t)(st->have - count);
	memmove(st->buf, &st->buf[count], remaining);
	st->have = remaining;

	/*
	 * Trailing bytes are the start of the next frame, so stay in-frame if
	 * they begin with a SOF. Discarding them would lose a frame that
	 * arrived in the same burst as the one just decoded.
	 */
	st->in_frame = (st->buf[0] == PROTO_SOF);
	st->need = 0;
	if (!st->in_frame) {
		st->have = 0;
	}
}

/*
 * Applies a consume() deferred by a previous try_extract(), if any. Must run
 * before anything else touches st->buf/st->have, so it goes first in both
 * push and poll.
 */
static void apply_pending_consume(proto_stream_t *st)
{
	if (st->pending_consume != 0) {
		const uint16_t count = st->pending_consume;

		st->pending_consume = 0;
		consume(st, count);
	}
}

/*
 * Tries to pull one complete frame out of whatever is already buffered.
 *
 * Loops rather than returning after a resync: once resync() has re-anchored
 * on a later SOF, the buffer may ALREADY hold a complete valid frame, and
 * waiting for another byte before parsing it would stall - or lose it
 * entirely if the stream went quiet.
 */
static bool try_extract(proto_stream_t *st, proto_frame_t *out)
{
	while (st->in_frame && st->have >= PROTO_HEADER_LEN) {
		const uint16_t payload_len = get_le16(&st->buf[4]);

		if (st->buf[1] != PROTO_VERSION ||
		    !type_is_valid(st->buf[2]) ||
		    payload_len > PROTO_MAX_PAYLOAD) {
			/* Not a plausible header - that was not a real SOF. */
			resync(st);
			continue;
		}

		st->need = (uint16_t)(PROTO_OVERHEAD + payload_len);
		if (st->have < st->need) {
			return false; /* still collecting */
		}

		const int err = proto_decode(st->buf, st->need, out);
		if (err == PROTO_OK) {
			/*
			 * out->payload points into st->buf. Consuming the frame here
			 * would memmove the buffer and clobber it before the caller
			 * can read it, so defer the consume to the start of the next
			 * push/poll instead - out->payload stays valid until then,
			 * as documented in the header.
			 */
			st->pending_consume = st->need;
			return true;
		}

		if (err == PROTO_ERR_CRC) {
			st->crc_errors++;
		}
		resync(st);
	}

	return false;
}

bool proto_stream_poll(proto_stream_t *st, proto_frame_t *out)
{
	if (st == NULL || out == NULL) {
		return false;
	}
	apply_pending_consume(st);
	return try_extract(st, out);
}

bool proto_stream_push(proto_stream_t *st, uint8_t byte, proto_frame_t *out)
{
	if (st == NULL || out == NULL) {
		return false;
	}

	apply_pending_consume(st);

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
		if (!st->in_frame) {
			return false;
		}
	}

	st->buf[st->have++] = byte;

	return try_extract(st, out);
}
