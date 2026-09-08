#include "command.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "afe/ads1299.h"
#include "pipeline/pipeline.h"
#include "proto/proto.h"
#include "stream.h"
#include "usb.h"

LOG_MODULE_REGISTER(command, CONFIG_LOG_DEFAULT_LEVEL);

#define CMD_STACK_SIZE 1536
#define CMD_PRIORITY   6 /* below the DSP thread: commands can wait */

/* How often to look for host input when none is arriving. */
#define CMD_POLL_MS 20

static K_THREAD_STACK_DEFINE(cmd_stack, CMD_STACK_SIZE);
static struct k_thread cmd_thread;

static proto_stream_t rx_stream;
static uint8_t rsp_buf[64];
static uint16_t rsp_seq;

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

	if (n > 0) {
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
		/* channels, encoding-in-use, nominal rate as a u16 */
		const uint8_t info[4] = {
			ADS1299_CHANNELS,
			0,
			250u & 0xFFu,
			250u >> 8,
		};

		respond(op, CMD_OK, info, sizeof(info));
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

static void cmd_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	proto_stream_reset(&rx_stream);

	while (1) {
		uint8_t buf[64];
		const size_t n = usb_transport_read(buf, sizeof(buf));

		if (n == 0) {
			k_msleep(CMD_POLL_MS);
			continue;
		}

		proto_frame_t f;

		for (size_t i = 0; i < n; i++) {
			if (proto_stream_push(&rx_stream, buf[i], &f)) {
				handle(&f);
			}
		}

		/*
		 * push() surfaces one frame per byte at most, so a buffer
		 * holding several complete frames leaves the rest queued.
		 */
		while (proto_stream_poll(&rx_stream, &f)) {
			handle(&f);
		}
	}
}

int command_init(void)
{
	k_tid_t tid = k_thread_create(&cmd_thread, cmd_stack, CMD_STACK_SIZE,
				      cmd_entry, NULL, NULL, NULL,
				      CMD_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(tid, "eeg_cmd");
	LOG_INF("command handler up");
	return 0;
}
