/*
 * SwiftEEG binary protocol - framing layer.
 *
 * One format, byte-identical over BLE and USB. The host library is written
 * once and works on both links.
 *
 * This header and its implementation deliberately depend on nothing but the
 * C standard library - no Zephyr, no HAL. That keeps the codec compilable
 * and testable on a host, which is how it gets validated against golden
 * vectors rather than only on target.
 *
 * Frame layout (little endian):
 *
 *   off  size  field
 *   0    1     SOF        0xA5
 *   1    1     version    PROTO_VERSION
 *   2    1     type       proto_type_t
 *   3    1     flags
 *   4    2     len        payload length in bytes
 *   6    2     seq        sequence number, wraps
 *   8    N     payload
 *   8+N  2     CRC-16/CCITT-FALSE over bytes [1 .. 8+N-1]
 *
 * SOF is excluded from the CRC so a resyncing stream decoder can scan for it
 * cheaply without having to special-case the checksum.
 *
 * BLE preserves message boundaries and USB CDC does not, so the same framing
 * is used on both: redundant on BLE, essential on USB, and one parser.
 */
#ifndef SWIFTEEG_PROTO_H
#define SWIFTEEG_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTO_SOF          0xA5u
#define PROTO_VERSION      1u
#define PROTO_HEADER_LEN   8u
#define PROTO_CRC_LEN      2u
#define PROTO_OVERHEAD     (PROTO_HEADER_LEN + PROTO_CRC_LEN)

/* Bounded so a corrupt length field cannot make the decoder allocate wildly. */
#define PROTO_MAX_PAYLOAD  1024u
#define PROTO_MAX_FRAME    (PROTO_MAX_PAYLOAD + PROTO_OVERHEAD)

typedef enum {
	PROTO_TYPE_CMD  = 0x01, /* host -> device, expects a RSP */
	PROTO_TYPE_RSP  = 0x02, /* device -> host, answers a CMD */
	PROTO_TYPE_EVT  = 0x03, /* device -> host, unsolicited */
	PROTO_TYPE_DATA = 0x04, /* device -> host, sample stream */
	PROTO_TYPE_IMU  = 0x05, /* device -> host, motion samples */
} proto_type_t;

/* Frame flags. */
#define PROTO_FLAG_NONE      0x00u
/* Set on DATA frames whose samples are still inside a filter settling window. */
#define PROTO_FLAG_SETTLING  0x01u
/* Set when samples were dropped before this frame; seq shows how many. */
#define PROTO_FLAG_OVERRUN   0x02u

typedef struct {
	uint8_t  version;
	uint8_t  type;
	uint8_t  flags;
	uint16_t len;
	uint16_t seq;
	const uint8_t *payload; /* points into the caller's buffer */
} proto_frame_t;

typedef enum {
	PROTO_OK = 0,
	PROTO_ERR_ARG      = -1, /* null pointer or payload too large */
	PROTO_ERR_SPACE    = -2, /* destination buffer too small */
	PROTO_ERR_VERSION  = -3,
	PROTO_ERR_CRC      = -4,
	PROTO_ERR_TYPE     = -5,
} proto_err_t;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor. */
uint16_t proto_crc16(const uint8_t *data, size_t len);

/*
 * Encodes one frame into dst.
 * Returns the number of bytes written, or a negative proto_err_t.
 */
int proto_encode(uint8_t type, uint8_t flags, uint16_t seq,
		 const void *payload, uint16_t payload_len,
		 uint8_t *dst, size_t dst_cap);

/*
 * Decodes a complete frame already sitting in a buffer.
 * Used on BLE, where message boundaries survive. `out->payload` points into
 * `src`, so it stays valid only as long as `src` does.
 */
int proto_decode(const uint8_t *src, size_t len, proto_frame_t *out);

/* --- Incremental decoder, for byte streams such as USB CDC --------------- */

typedef struct {
	uint8_t  buf[PROTO_MAX_FRAME];
	uint16_t have;      /* bytes currently in buf */
	uint16_t need;      /* total frame length once the header is known */
	bool     in_frame;  /* SOF seen, still collecting */
	uint32_t resyncs;   /* diagnostics: times we discarded and rescanned */
	uint32_t crc_errors;
} proto_stream_t;

void proto_stream_reset(proto_stream_t *st);

/*
 * Feeds one byte. Returns true when `out` holds a complete, verified frame.
 *
 * On a bad CRC or an implausible header the decoder drops the frame, counts
 * it, and resumes scanning for the next SOF - a stream transport must be able
 * to recover from a desync rather than wedging.
 */
bool proto_stream_push(proto_stream_t *st, uint8_t byte, proto_frame_t *out);

/*
 * Tries to pull a frame out of already-buffered bytes without feeding a new
 * one. A single push can leave a second complete frame buffered (they often
 * arrive in one burst), so drain with this until it returns false if you need
 * every frame before the stream goes quiet.
 *
 * As with proto_decode, out->payload points into the decoder's own buffer and
 * is only valid until the next push or poll.
 */
bool proto_stream_poll(proto_stream_t *st, proto_frame_t *out);

#endif /* SWIFTEEG_PROTO_H */
