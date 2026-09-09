#include "command.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "afe/ads1299.h"
#include "pipeline/pipeline.h"
#include "proto/proto.h"
#include "ble.h"
#include "stream.h"
#include "usb.h"

LOG_MODULE_REGISTER(command, CONFIG_LOG_DEFAULT_LEVEL);

#define CMD_STACK_SIZE 1536
#define CMD_PRIORITY   6 /* below the DSP thread: commands can wait */

/*
 * The thread sleeps on this and is woken by whichever link received bytes,
 * so a command is picked up as soon as it lands rather than at the next
 * poll. The timeout is only a backstop in case a wake-up is ever missed.
 */
#define CMD_IDLE_MS 200

static struct k_sem cmd_ready;

static K_THREAD_STACK_DEFINE(cmd_stack, CMD_STACK_SIZE);
static struct k_thread cmd_thread;

static proto_stream_t rx_stream;   /* USB */
static proto_stream_t ble_stream;  /* BLE, separate so a partial frame on
                                    * one link cannot corrupt the other */

/*
 * Bytes handed over by the Bluetooth thread, drained by the command thread.
 * Commands are small and infrequent; this only has to absorb a burst.
 */
#define BLE_RX_BYTES 256
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

static void respond(uint8_t opcode, uint8_t status, const uint8_t *extra,
		    size_t extra_len)
{
	uint8_t payload[16];

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
		if (f->len < 2) {
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
			stream_enable(was_streaming);
		}
		break;

	case CMD_SET_RATE: {
		if (f->len < 3) {
			status = CMD_EBADARG;
			break;
		}

		const uint16_t sps = (uint16_t)f->payload[1] |
				     ((uint16_t)f->payload[2] << 8);

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
			status = (pipeline_set_notch(f->payload[1]) == 0)
					 ? CMD_OK : CMD_EBADARG;
		}
		break;

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
