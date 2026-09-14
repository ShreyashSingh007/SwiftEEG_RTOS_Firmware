#include "command.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "afe/ads1299.h"
#include "dsp/dsp.h"
#include "imu/imu.h"
#include "pipeline/pipeline.h"
#include "proto/proto.h"
#include "ble.h"
#include "stream.h"
#include "usb.h"

LOG_MODULE_REGISTER(command, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * Immediate-mode logging formats on the calling thread's stack, and a filter
 * upload decodes its sections there too. The Bluetooth receive thread
 * overflowed on less; this one is given room.
 */
#define CMD_STACK_SIZE 3072
#define CMD_PRIORITY   6 /* below the DSP thread: commands can wait */

/*
 * The thread sleeps on this and is woken by whichever link received bytes,
 * so a command is picked up as soon as it lands rather than at the next
 * poll. The timeout is only a backstop in case a wake-up is ever missed.
 */
#define CMD_IDLE_MS 200

/* One section on the wire: g, k, m0, m1, m2, little-endian float32 each. */
#define SECTION_BYTES 20u

static struct k_sem cmd_ready;

static K_THREAD_STACK_DEFINE(cmd_stack, CMD_STACK_SIZE);
static struct k_thread cmd_thread;

static proto_stream_t rx_stream;   /* USB */
static proto_stream_t ble_stream;  /* BLE, separate so a partial frame on
                                    * one link cannot corrupt the other */

/*
 * Bytes handed over by the Bluetooth thread, drained by the command thread.
 * A filter upload is up to 176 bytes, one per stage, and the Bluetooth thread
 * can deliver both before this thread next runs - so this holds several.
 */
#define BLE_RX_BYTES 1024
static uint8_t ble_rx_storage[BLE_RX_BYTES];
static struct ring_buf ble_rx_rb;
static uint8_t rsp_buf[64];
static uint16_t rsp_seq;

/*
 * Which link the command being handled arrived on. A reply has to go back
 * the way the request came: a host on BLE never sees an answer sent to USB,
 * and cannot tell that from a command that was ignored.
 */
static bool reply_via_ble;

static inline uint16_t get_u16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t get_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)(v >> 8);
}

static inline void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static inline float get_f32(const uint8_t *p)
{
	const uint32_t bits = get_u32(p);
	float v;

	memcpy(&v, &bits, sizeof(v));
	return v;
}

static dsp_section_t get_section(const uint8_t *p)
{
	return (dsp_section_t){
		.g = get_f32(&p[0]),
		.k = get_f32(&p[4]),
		.m0 = get_f32(&p[8]),
		.m1 = get_f32(&p[12]),
		.m2 = get_f32(&p[16]),
	};
}

static void put_section(uint8_t *p, const dsp_section_t *s)
{
	const float v[5] = { s->g, s->k, s->m0, s->m1, s->m2 };

	for (size_t i = 0; i < 5; i++) {
		uint32_t bits;

		memcpy(&bits, &v[i], sizeof(bits));
		put_u32(&p[4 * i], bits);
	}
}

/*
 * CRC of a stage's sections in their wire form. Reported with the
 * configuration so a host can tell whether the device still holds exactly
 * what it sent - after a reconnect, say - without reading every value back.
 */
static uint16_t sections_crc(const dsp_section_t *sections, uint8_t count)
{
	uint8_t buf[DSP_MAX_SECTIONS * SECTION_BYTES];

	for (uint8_t i = 0; i < count; i++) {
		put_section(&buf[(size_t)i * SECTION_BYTES], &sections[i]);
	}

	return proto_crc16(buf, (size_t)count * SECTION_BYTES);
}

static void respond(uint8_t opcode, uint8_t status, const uint8_t *extra,
		    size_t extra_len)
{
	uint8_t payload[32];

	payload[0] = opcode;
	payload[1] = status;

	size_t len = 2;

	if (extra != NULL && extra_len <= sizeof(payload) - 2) {
		memcpy(&payload[2], extra, extra_len);
		len += extra_len;
	}

	const int n = proto_encode(PROTO_TYPE_RSP, PROTO_FLAG_NONE, rsp_seq++,
				   payload, len, rsp_buf, sizeof(rsp_buf));

	if (n <= 0) {
		return;
	}

	if (reply_via_ble) {
		(void)ble_transport_send_event(rsp_buf, (uint16_t)n);
	} else {
		(void)usb_transport_write(rsp_buf, (size_t)n);
	}
}

/* An OK that says from which sample a filter change applies. */
static void respond_seq(uint8_t opcode, uint32_t seq)
{
	uint8_t extra[4];

	put_u32(extra, seq);
	respond(opcode, CMD_OK, extra, sizeof(extra));
}

static uint8_t status_for(int err)
{
	if (err == 0) {
		return CMD_OK;
	}
	return (err == -EINVAL) ? CMD_EBADARG : CMD_EFAILED;
}

/*
 * After anything that can change a gain. The chain scales each channel by
 * its own gain, and a stale one makes every microvolt it reports wrong by
 * the ratio of the two.
 */
static uint8_t sync_gains(uint8_t status)
{
	if (status == CMD_OK && pipeline_sync_gains() != 0) {
		LOG_WRN("a gain changed but the chain could not follow it");
		return CMD_EFAILED;
	}
	return status;
}

/*
 * [1] stage, [2] flags (bit 0 keep state), [3..4] the rate the sections were
 * designed for, [5] count, then count sections of SECTION_BYTES each.
 */
static void handle_set_filter(const proto_frame_t *f)
{
	const uint8_t op = f->payload[0];

	if (f->len < 6u) {
		respond(op, CMD_EBADARG, NULL, 0);
		return;
	}

	const uint8_t stage = f->payload[1];
	const bool keep = (f->payload[2] & 0x01u) != 0u;
	const uint16_t designed_for = get_u16(&f->payload[3]);
	const uint8_t count = f->payload[5];

	if (stage > PIPELINE_STAGE_POST || count > DSP_MAX_SECTIONS ||
	    f->len != 6u + SECTION_BYTES * count) {
		respond(op, CMD_EBADARG, NULL, 0);
		return;
	}

	/*
	 * A section is only the filter it was designed to be at the rate it
	 * was designed for; run at another rate it quietly moves. That class of
	 * mistake once put 3390 uV of noise on a quiet channel, so a mismatch
	 * is refused rather than run.
	 */
	if (designed_for != pipeline_rate()) {
		respond(op, CMD_EBADARG, NULL, 0);
		return;
	}

	dsp_section_t sections[DSP_MAX_SECTIONS];

	for (uint8_t i = 0; i < count; i++) {
		sections[i] = get_section(&f->payload[6u + SECTION_BYTES * i]);
	}

	uint32_t seq = 0;
	const int err = pipeline_set_stage(stage, sections, count, keep, &seq);

	if (err == 0) {
		respond_seq(op, seq);
	} else {
		respond(op, status_for(err), NULL, 0);
	}
}

static void handle(const proto_frame_t *f)
{
	if (f->type != PROTO_TYPE_CMD || f->len < 1) {
		return;
	}

	const uint8_t op = f->payload[0];
	uint8_t status = CMD_OK;

	switch (op) {
	case CMD_PING:
		break;

	case CMD_STREAM_START:
		stream_enable(true);
		break;

	case CMD_STREAM_STOP:
		stream_enable(false);
		break;

	case CMD_SET_ENCODING:
		if (f->len < 2 || f->payload[1] > STREAM_ENC_RAW_UV) {
			status = CMD_EBADARG;
		} else {
			stream_set_encoding(f->payload[1]);
		}
		break;

	case CMD_TEST_SIGNAL:
		if (f->len < 2) {
			status = CMD_EBADARG;
		} else {
			const bool on = f->payload[1] != 0;
			const uint8_t freq = (f->len >= 3) ? f->payload[2]
							   : ADS1299_CAL_FREQ_DIV21;

			/*
			 * The driver handles dropping the AFE into command
			 * mode and back; pausing transmission here just keeps
			 * a half-built batch from spanning the change.
			 */
			const bool was_streaming = stream_enabled();

			stream_enable(false);
			status = (ads1299_test_signal(on, freq) == 0)
					 ? CMD_OK : CMD_EFAILED;
			status = sync_gains(status);
			stream_enable(was_streaming);
		}
		break;

	case CMD_GET_INFO: {
		const uint16_t sps = pipeline_rate();
		const uint8_t info[4] = {
			ADS1299_CHANNELS,
			0,
			(uint8_t)(sps & 0xFFu),
			(uint8_t)(sps >> 8),
		};

		respond(op, CMD_OK, info, sizeof(info));
		return;
	}

	case CMD_SET_INPUT:
		if (f->len < 2) {
			status = CMD_EBADARG;
		} else {
			const uint8_t mux = f->payload[1];
			const uint8_t freq = (f->len >= 3) ? f->payload[2]
							   : ADS1299_CAL_FREQ_DIV21;
			const bool was_streaming = stream_enabled();

			stream_enable(false);
			status = (ads1299_set_input(mux, freq) == 0)
					 ? CMD_OK : CMD_EFAILED;
			status = sync_gains(status);
			stream_enable(was_streaming);
		}
		break;

	case CMD_SET_RATE: {
		if (f->len < 3) {
			status = CMD_EBADARG;
			break;
		}

		const uint16_t sps = get_u16(&f->payload[1]);

		/*
		 * Answer before restarting. The restart tears down the DSP
		 * thread and reconfigures the AFE, which takes long enough
		 * that a host waiting on the reply would time out.
		 */
		respond(op, CMD_OK, NULL, 0);
		(void)pipeline_set_rate(sps);
		return;
	}

	case CMD_SET_CHANNEL:
		if (f->len < 4) {
			status = CMD_EBADARG;
		} else {
			const bool pd = (f->len >= 5) && f->payload[4];
			const bool srb2 = (f->len >= 6) && f->payload[5];

			status = (ads1299_set_channel(f->payload[1],
						      f->payload[2],
						      f->payload[3], pd,
						      srb2) == 0)
					 ? CMD_OK : CMD_EFAILED;
			status = sync_gains(status);
		}
		break;

	case CMD_SET_BIAS:
		if (f->len < 2) {
			status = CMD_EBADARG;
		} else {
			const uint8_t sensp = (f->len >= 3) ? f->payload[2] : 0xFFu;
			const uint8_t sensn = (f->len >= 4) ? f->payload[3] : 0x00u;

			status = (ads1299_set_bias(f->payload[1] != 0, sensp,
						   sensn) == 0)
					 ? CMD_OK : CMD_EFAILED;
		}
		break;

	case CMD_SET_NOTCH:
		if (f->len < 2) {
			status = CMD_EBADARG;
		} else {
			status = status_for(pipeline_set_notch(f->payload[1]));
		}
		break;

	case CMD_SET_LEADOFF:
		if (f->len < 2) {
			status = CMD_EBADARG;
		} else {
			const uint8_t sensp = (f->len >= 3) ? f->payload[2] : 0xFFu;
			const uint8_t sensn = (f->len >= 4) ? f->payload[3] : 0x00u;

			status = (ads1299_set_leadoff(f->payload[1] != 0, sensp,
						      sensn) == 0)
					 ? CMD_OK : CMD_EFAILED;
		}
		break;

	case CMD_SET_IMU:
		if (f->len < 7) {
			status = CMD_EBADARG;
		} else {
			const struct imu_config cfg = {
				.enabled = f->payload[1] != 0,
				.rate_hz = get_u16(&f->payload[2]),
				.accel_g = f->payload[4],
				.gyro_dps = get_u16(&f->payload[5]),
			};
			const int err = imu_configure(&cfg);

			status = (err == 0) ? CMD_OK
				 : (err == -EINVAL) ? CMD_EBADARG : CMD_EFAILED;
		}
		break;

	case CMD_SET_FILTER:
		handle_set_filter(f);
		return;

	case CMD_SET_CAR: {
		if (f->len < 3) {
			status = CMD_EBADARG;
			break;
		}

		uint32_t seq = 0;
		const int err = pipeline_set_car(f->payload[1] != 0,
						 f->payload[2], &seq);

		if (err == 0) {
			respond_seq(op, seq);
			return;
		}
		status = status_for(err);
		break;
	}

	case CMD_RESET_CHAIN: {
		uint32_t seq = 0;
		const int err = pipeline_reset_chain(&seq);

		if (err == 0) {
			respond_seq(op, seq);
			return;
		}
		status = status_for(err);
		break;
	}

	case CMD_GET_CONFIG: {
		/*
		 * Everything a host needs to draw its controls in the right
		 * position, in one exchange. Reading it back from the AFE
		 * rather than reporting what we believe we set means a
		 * failed write shows up as a wrong control, not a lie.
		 */
		uint8_t chset[ADS1299_CHANNELS] = { 0 };
		struct imu_config imu;
		struct pipeline_filters filt;

		(void)ads1299_get_channels(chset, ADS1299_CHANNELS);
		imu_get_config(&imu);
		pipeline_get_filters(&filt);

		const uint16_t sps = pipeline_rate();
		uint8_t cfg[27];

		cfg[0] = ADS1299_CHANNELS;
		cfg[1] = stream_encoding();
		put_u16(&cfg[2], sps);
		cfg[4] = pipeline_notch();
		memcpy(&cfg[5], chset, sizeof(chset));

		/* Motion sensor: bit 0 on, bit 1 fitted; then rate and ranges. */
		cfg[13] = (uint8_t)((imu.enabled ? 0x01u : 0u) |
				    (imu_present() ? 0x02u : 0u));
		put_u16(&cfg[14], imu.rate_hz);
		cfg[16] = imu.accel_g;
		put_u16(&cfg[17], imu.gyro_dps);

		/*
		 * The chain: sections per stage; flags (bit 0 common average
		 * on, bit 1 the pre stage is the device's own notch); the
		 * average's channel mask; a CRC of each stage's sections.
		 */
		cfg[19] = filt.pre_count;
		cfg[20] = filt.post_count;
		cfg[21] = (uint8_t)((filt.car ? 0x01u : 0u) |
				    (filt.pre_is_notch ? 0x02u : 0u));
		cfg[22] = filt.car_mask;
		put_u16(&cfg[23], sections_crc(filt.pre, filt.pre_count));
		put_u16(&cfg[25], sections_crc(filt.post, filt.post_count));

		respond(op, CMD_OK, cfg, sizeof(cfg));
		return;
	}

	case CMD_READ_REG: {
		if (f->len < 2) {
			status = CMD_EBADARG;
			break;
		}

		uint8_t val = 0;

		/*
		 * Registers only read back in command mode, so this borrows
		 * the same stop/restore the driver uses for writes.
		 */
		if (ads1299_read_reg_safe(f->payload[1], &val) != 0) {
			status = CMD_EFAILED;
			break;
		}

		const uint8_t extra[2] = { f->payload[1], val };

		respond(op, CMD_OK, extra, sizeof(extra));
		return;
	}

	default:
		status = CMD_EUNKNOWN;
		break;
	}

	respond(op, status, NULL, 0);
}

/*
 * Commands arriving over BLE.
 *
 * This runs on the Bluetooth thread, so it only copies the bytes and
 * returns. Handling them here would be a mistake: some commands restart the
 * pipeline, which stops the DSP thread and reconfigures the AFE and takes
 * the better part of a second. Blocking the Bluetooth thread that long
 * starves the link layer and the central drops the connection - which is
 * exactly what a rate change did before this queue existed.
 */
static void on_ble_control(const uint8_t *data, uint16_t len)
{
	const uint32_t put = ring_buf_put(&ble_rx_rb, data, len);

	if (put < len) {
		LOG_WRN("BLE command buffer full, %u bytes dropped", len - put);
	}

	k_sem_give(&cmd_ready);
}

/* Runs in the USB receive interrupt. */
static void on_usb_rx(void)
{
	k_sem_give(&cmd_ready);
}

static void cmd_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	proto_stream_reset(&rx_stream);

	while (1) {
		uint8_t buf[64];
		proto_frame_t f;
		bool did_work = false;

		/* USB */
		size_t n = usb_transport_read(buf, sizeof(buf));

		if (n != 0) {
			did_work = true;
			reply_via_ble = false;

			for (size_t i = 0; i < n; i++) {
				if (proto_stream_push(&rx_stream, buf[i], &f)) {
					handle(&f);
				}
			}

			/*
			 * push() surfaces one frame per byte at most, so a
			 * buffer holding several complete frames leaves the
			 * rest queued.
			 */
			while (proto_stream_poll(&rx_stream, &f)) {
				handle(&f);
			}
		}

		/* BLE, queued by the Bluetooth thread. */
		n = ring_buf_get(&ble_rx_rb, buf, sizeof(buf));

		if (n != 0) {
			did_work = true;
			reply_via_ble = true;

			for (size_t i = 0; i < n; i++) {
				if (proto_stream_push(&ble_stream, buf[i], &f)) {
					handle(&f);
				}
			}

			while (proto_stream_poll(&ble_stream, &f)) {
				handle(&f);
			}

			reply_via_ble = false;
		}

		if (!did_work) {
			(void)k_sem_take(&cmd_ready, K_MSEC(CMD_IDLE_MS));
		}
	}
}

int command_init(void)
{
	k_sem_init(&cmd_ready, 0, K_SEM_MAX_LIMIT);

	k_tid_t tid = k_thread_create(&cmd_thread, cmd_stack, CMD_STACK_SIZE,
				      cmd_entry, NULL, NULL, NULL,
				      CMD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(tid, "eeg_cmd");

	proto_stream_reset(&ble_stream);
	ring_buf_init(&ble_rx_rb, sizeof(ble_rx_storage), ble_rx_storage);
	ble_transport_set_control_handler(on_ble_control);
	usb_transport_set_rx_notify(on_usb_rx);

	LOG_INF("command handler up (USB and BLE)");
	return 0;
}
