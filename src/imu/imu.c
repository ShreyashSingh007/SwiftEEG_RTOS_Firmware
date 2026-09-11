#include "imu.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <hal/nrf_gpio.h>

#include "pipeline/capture.h"
#include "proto/proto.h"
#include "timebase/timebase.h"
#include "transport/stream.h"

LOG_MODULE_REGISTER(imu, CONFIG_LOG_DEFAULT_LEVEL);

#define IMU_NODE  DT_ALIAS(imu)
#define INT2_PIN  DT_GPIO_PIN(IMU_NODE, int2_gpios)
#define INT2_PORT DT_PROP(DT_GPIO_CTLR(IMU_NODE, int2_gpios), port)

/* Registers and fields, as defined in ST's lsm6dsv16x_reg.h. */
#define REG_IF_CFG        0x03
#define REG_FIFO_CTRL1    0x07 /* watermark, in FIFO words */
#define REG_FIFO_CTRL3    0x09 /* batch rate: accel [3:0], gyro [7:4] */
#define REG_FIFO_CTRL4    0x0A /* fifo_mode [2:0] */
#define REG_INT2_CTRL     0x0E
#define REG_WHO_AM_I      0x0F
#define REG_CTRL1         0x10 /* accel ODR [3:0], operating mode [6:4] */
#define REG_CTRL2         0x11 /* gyro ODR [3:0], operating mode [6:4] */
#define REG_CTRL3         0x12
#define REG_CTRL6         0x15 /* gyro full scale [3:0] */
#define REG_CTRL8         0x17 /* accel full scale [1:0] */
#define REG_FIFO_STATUS1  0x1B /* unread words, bits 7:0 */
#define REG_FIFO_STATUS2  0x1C /* bit 0: unread words bit 8; bit 3: overrun */
#define REG_INTERNAL_FREQ 0x4F /* the sensor's clock error, 0.13 % a count */
#define REG_FIFO_DATA_TAG 0x78 /* tag byte, then six data bytes */

#define WHO_AM_I_VALUE           0x70
#define IF_CFG_I2C_I3C_DISABLE   0x01
#define CTRL3_SW_RESET           0x01
#define CTRL3_IF_INC             0x04
#define CTRL3_BDU                0x40
#define INT2_FIFO_TH             0x08
#define FIFO_MODE_BYPASS         0x00
#define FIFO_MODE_STREAM         0x06
#define FIFO_STATUS2_OVR_LATCHED 0x08
#define TAG_GYRO                 0x01
#define TAG_ACCEL                0x02

#define WORD_BYTES 7

/*
 * IMU frame payload, little-endian:
 *
 *   u64 ts_us         TIMER1 timestamp of the first sample
 *   u32 seq           sensor sample index of the first sample
 *   u32 period_us_q8  sample period, 1/256 us
 *   u16 accel_g       accelerometer full scale; one count is accel_g/32768 g
 *   u16 gyro_dps      gyroscope full scale; one count is gyro_dps/32768 dps
 *   u8  axes          6: accel x y z, then gyro x y z, int16 each
 *   u8  flags         IMU_FLAG_*
 *   u16 count         samples that follow
 */
#define HDR_LEN      24
#define SAMPLE_BYTES 12

/*
 * At most this many samples a frame, so a frame is one BLE notification:
 * (244 - 10 protocol - 24 header) / 12 = 17.
 */
#define FRAME_SAMPLES_MAX 17

/* Words read per poll at most. A late poll at 960 Hz finds about 30. */
#define DRAIN_WORDS_MAX 96

#define IMU_STACK_SIZE 2048
#define IMU_PRIORITY   4 /* below the EEG DSP thread, above commands */

static const struct spi_dt_spec bus = SPI_DT_SPEC_GET(
	IMU_NODE, SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA);

static K_THREAD_STACK_DEFINE(imu_stack, IMU_STACK_SIZE);
static struct k_thread imu_thread;

static bool present;

/*
 * `active` is written only by the IMU thread and read elsewhere under the
 * lock; `requested` is the reverse.
 */
static struct imu_config active = {
	.enabled = true,
	.rate_hz = 240,
	.accel_g = 8,
	.gyro_dps = 2000,
};
static struct imu_config requested;
static atomic_t config_pending;
static struct k_spinlock config_lock;

static struct imu_stats stats;

/* Timing state, owned by the IMU thread. */
static uint8_t  batch;        /* samples per watermark */
static uint32_t last_capture; /* capture register when the last read ended */
static bool     have_edge;
static uint64_t edge_us;      /* time of the last trusted watermark edge */
static uint64_t edge_index;   /* sample index that edge belongs to */
static uint32_t edges;        /* period measurements so far */
static uint64_t period_q8;    /* sample period in use, 1/256 us */
static uint64_t nominal_q8;   /* what the rate says it should be */
static uint32_t seq;          /* index of the next sample */
static uint8_t  next_flags;

struct motion {
	int16_t v[6]; /* accel x y z, gyro x y z */
};

static uint8_t fifo_buf[DRAIN_WORDS_MAX * WORD_BYTES];
static struct motion samples[DRAIN_WORDS_MAX / 2];
static uint8_t payload[HDR_LEN + FRAME_SAMPLES_MAX * SAMPLE_BYTES];
static uint8_t frame[PROTO_OVERHEAD + sizeof(payload)];
static uint16_t frame_seq;

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
	p[3] = (uint8_t)(v >> 24);
}

/* --- bus ------------------------------------------------------------------ */

static int reg_read(uint8_t reg, uint8_t *data, size_t len)
{
	uint8_t addr = (uint8_t)(reg | 0x80);
	const struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf rx_bufs[] = {
		{ .buf = NULL, .len = 1 }, /* clocked in while the address goes out */
		{ .buf = data, .len = len },
	};
	const struct spi_buf_set rx = { .buffers = rx_bufs,
					.count = ARRAY_SIZE(rx_bufs) };

	return spi_transceive_dt(&bus, &tx, &rx);
}

static int reg_write(uint8_t reg, uint8_t value)
{
	uint8_t buf[2] = { (uint8_t)(reg & 0x7F), value };
	const struct spi_buf tx_buf = { .buf = buf, .len = sizeof(buf) };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	return spi_write_dt(&bus, &tx);
}

static int reg_update(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t v;
	const int err = reg_read(reg, &v, 1);

	if (err) {
		return err;
	}

	return reg_write(reg, (uint8_t)((v & ~mask) | (value & mask)));
}

/* --- settings ------------------------------------------------------------- */

static int odr_code(uint16_t hz)
{
	switch (hz) {
	case 60:  return 0x5;
	case 120: return 0x6;
	case 240: return 0x7;
	case 480: return 0x8;
	case 960: return 0x9;
	default:  return -EINVAL;
	}
}

static int accel_fs_code(uint8_t g)
{
	switch (g) {
	case 2:  return 0x0;
	case 4:  return 0x1;
	case 8:  return 0x2;
	case 16: return 0x3;
	default: return -EINVAL;
	}
}

static int gyro_fs_code(uint16_t dps)
{
	switch (dps) {
	case 125:  return 0x0;
	case 250:  return 0x1;
	case 500:  return 0x2;
	case 1000: return 0x3;
	case 2000: return 0x4;
	case 4000: return 0xC;
	default:   return -EINVAL;
	}
}

/*
 * Samples per watermark, and how often to look for it. A late look adds
 * samples to the batch, and a batch has to stay within one frame's 17.
 */
static uint8_t batch_for(uint16_t hz)
{
	return (hz >= 960) ? 8 : (hz >= 480) ? 10 : 12;
}

static int32_t poll_ms_for(uint16_t hz)
{
	return (hz >= 960) ? 4 : 8;
}

static void timing_reset(uint16_t hz)
{
	have_edge = false;
	edges = 0;
	nominal_q8 = ((uint64_t)1000000 << 8) / hz;
	period_q8 = nominal_q8;
	last_capture = timebase_imu_capture_get();
}

static int apply(const struct imu_config *c)
{
	/*
	 * Stop and empty the FIFO first - bypass mode discards it. Samples
	 * batched at the old rate or range must not be read back as the new.
	 */
	int err = reg_write(REG_FIFO_CTRL4, FIFO_MODE_BYPASS);

	err |= reg_update(REG_CTRL1, 0x0F, 0);
	err |= reg_update(REG_CTRL2, 0x0F, 0);

	if (err || !c->enabled) {
		return err;
	}

	const uint8_t odr = (uint8_t)odr_code(c->rate_hz);

	batch = batch_for(c->rate_hz);
	timing_reset(c->rate_hz);

	err |= reg_update(REG_CTRL8, 0x03, (uint8_t)accel_fs_code(c->accel_g));
	err |= reg_update(REG_CTRL6, 0x0F, (uint8_t)gyro_fs_code(c->gyro_dps));

	/* Watermark in FIFO words: an accelerometer and a gyroscope word per sample. */
	err |= reg_write(REG_FIFO_CTRL1, (uint8_t)(2 * batch));
	err |= reg_write(REG_FIFO_CTRL3, (uint8_t)((odr << 4) | odr));
	err |= reg_write(REG_INT2_CTRL, INT2_FIFO_TH);
	err |= reg_update(REG_FIFO_CTRL4, 0x07, FIFO_MODE_STREAM);

	/* Operating mode 0 is high performance. */
	err |= reg_update(REG_CTRL1, 0x7F, odr);
	err |= reg_update(REG_CTRL2, 0x7F, odr);

	return err;
}

/* --- samples -------------------------------------------------------------- */

static void emit(uint64_t ts, uint32_t first, const struct motion *s,
		 uint16_t count, uint8_t flags)
{
	uint8_t *p = payload;

	put_u32(&p[0], (uint32_t)(ts & 0xFFFFFFFFu));
	put_u32(&p[4], (uint32_t)(ts >> 32));
	put_u32(&p[8], first);
	put_u32(&p[12], (uint32_t)period_q8);
	put_u16(&p[16], active.accel_g);
	put_u16(&p[18], active.gyro_dps);
	p[20] = 6;
	p[21] = flags;
	put_u16(&p[22], count);

	p += HDR_LEN;
	for (uint16_t i = 0; i < count; i++) {
		for (int a = 0; a < 6; a++) {
			put_u16(p, (uint16_t)s[i].v[a]);
			p += 2;
		}
	}

	const int n = proto_encode(PROTO_TYPE_IMU, PROTO_FLAG_NONE, frame_seq++,
				   payload, (uint16_t)(HDR_LEN + count * SAMPLE_BYTES),
				   frame, sizeof(frame));

	if (n > 0 && stream_send(frame, (size_t)n)) {
		stats.frames++;
	}
}

/*
 * Learn the sensor's real sample period from two watermark edges.
 *
 * An edge paired with the wrong sample would show up as a period far from
 * nominal, so anything more than 5 % out is ignored rather than learned.
 */
static void learn_period(uint64_t at_us, uint64_t index)
{
	if (have_edge && index > edge_index && at_us > edge_us) {
		const uint64_t measured = ((at_us - edge_us) << 8) / (index - edge_index);
		const uint64_t slack = nominal_q8 / 20;

		if (measured > nominal_q8 - slack && measured < nominal_q8 + slack) {
			if (edges < 16) {
				period_q8 = (edges == 0) ? measured
					  : (period_q8 * edges + measured) / (edges + 1);
			} else {
				period_q8 = (period_q8 * 15 + measured) / 16;
			}
			edges++;
		}
	}

	have_edge = true;
	edge_us = at_us;
	edge_index = index;
}

static int drain(void)
{
	uint8_t status[2];

	if (reg_read(REG_FIFO_STATUS1, status, sizeof(status)) != 0) {
		return -EIO;
	}

	if (status[1] & FIFO_STATUS2_OVR_LATCHED) {
		/* Samples were lost, so the chain of edges no longer holds. */
		stats.overruns++;
		timing_reset(active.rate_hz);
		next_flags |= IMU_FLAG_OVERRUN;
	}

	const uint16_t wtm = (uint16_t)(2 * batch);
	uint16_t words = (uint16_t)(status[0] | ((status[1] & 0x01) << 8));

	if (words < wtm) {
		return 0;
	}
	if (words > DRAIN_WORDS_MAX) {
		words = DRAIN_WORDS_MAX;
	}

	/*
	 * The edge for this batch was latched when the watermark word went in,
	 * which was before the status read above could count it.
	 */
	const uint32_t capture = timebase_imu_capture_get();
	const uint64_t polled_us = timebase_now_us();

	/*
	 * Every word in one transfer; the address wraps from the last data
	 * byte back to the tag. Read word by word, a 480 Hz batch took long
	 * enough for new words to push the FIFO back over the watermark while
	 * it was still being emptied - and that edge was then taken for the
	 * next batch's own, putting samples up to 25 ms out.
	 */
	if (reg_read(REG_FIFO_DATA_TAG, fifo_buf, (size_t)words * WORD_BYTES) != 0) {
		return -EIO;
	}

	const uint32_t after = timebase_imu_capture_get();

	uint16_t n = 0;
	int edge_sample = -1;
	bool edge_word_read = false;
	bool have_a = false;
	bool have_g = false;
	uint8_t cnt_a = 0;
	uint8_t cnt_g = 0;
	int16_t acc[3] = { 0 };
	int16_t gyr[3] = { 0 };

	for (uint16_t w = 0; w < words; w++) {
		const uint8_t *word = &fifo_buf[w * WORD_BYTES];
		const uint8_t tag = (uint8_t)(word[0] >> 3);
		const uint8_t cnt = (uint8_t)((word[0] >> 1) & 0x03);
		int16_t xyz[3];

		for (int a = 0; a < 3; a++) {
			xyz[a] = (int16_t)(word[1 + 2 * a] | (word[2 + 2 * a] << 8));
		}

		if (tag == TAG_ACCEL) {
			memcpy(acc, xyz, sizeof(acc));
			cnt_a = cnt;
			have_a = true;
		} else if (tag == TAG_GYRO) {
			memcpy(gyr, xyz, sizeof(gyr));
			cnt_g = cnt;
			have_g = true;
		}

		if (w == (uint16_t)(wtm - 1)) {
			edge_word_read = true;
		}

		if (!(have_a && have_g)) {
			continue;
		}

		/* Both words of one time slot share a tag counter. */
		if (cnt_a == cnt_g) {
			memcpy(samples[n].v, acc, sizeof(acc));
			memcpy(&samples[n].v[3], gyr, sizeof(gyr));
			n++;

			if (edge_word_read && edge_sample < 0) {
				edge_sample = (int)n - 1;
			}
		} else {
			stats.unpaired++;
		}

		have_a = false;
		have_g = false;
	}

	uint8_t flags = next_flags;
	uint64_t first_us;

	next_flags = 0;

	if (capture != last_capture && edge_sample >= 0) {
		/* An edge of this batch's own: anchor on it. */
		const uint64_t at_us = timebase_stamp_past_us(capture);

		learn_period(at_us, (uint64_t)seq + (uint64_t)edge_sample);
		first_us = at_us - (((uint64_t)edge_sample * period_q8) >> 8);
	} else if (have_edge) {
		/*
		 * No edge to call its own - lost to a late poll, say. Carry on
		 * from the last one at the measured period, which by now is
		 * known far better than a poll time could be.
		 */
		first_us = edge_us + (((uint64_t)seq - edge_index) * period_q8 >> 8);
		stats.extrapolated++;
	} else {
		/* Nothing to go on yet: the newest sample is at most a poll old. */
		first_us = polled_us - ((uint64_t)(n ? n - 1 : 0) * period_q8 >> 8);
		flags |= IMU_FLAG_TIME_ESTIMATED;
		stats.estimated++;
	}

	/*
	 * An edge that landed while the words were being read cannot be
	 * placed: it may belong to one of them. Consume it, so the next batch
	 * does not take it for its own watermark.
	 */
	last_capture = after;

	if (n == 0) {
		return 0;
	}

	if (stream_enabled()) {
		for (uint16_t off = 0; off < n; off = (uint16_t)(off + FRAME_SAMPLES_MAX)) {
			uint16_t count = (uint16_t)(n - off);

			if (count > FRAME_SAMPLES_MAX) {
				count = FRAME_SAMPLES_MAX;
			}

			emit(first_us + (((uint64_t)off * period_q8) >> 8),
			     seq + off, &samples[off], count, flags);
		}
	}

	seq += n;
	stats.samples += n;
	stats.period_us_q8 = (uint32_t)period_q8;

	return 0;
}

/* --- thread --------------------------------------------------------------- */

static void imu_entry(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	uint32_t bus_errors = 0;

	while (1) {
		if (atomic_cas(&config_pending, 1, 0)) {
			k_spinlock_key_t key = k_spin_lock(&config_lock);
			struct imu_config next = requested;

			k_spin_unlock(&config_lock, key);

			const int err = apply(&next);

			key = k_spin_lock(&config_lock);
			active = next;
			k_spin_unlock(&config_lock, key);

			if (err) {
				LOG_ERR("IMU configuration failed (%d)", err);
			} else if (next.enabled) {
				LOG_INF("IMU on: %u Hz, +/-%u g, +/-%u dps, %u samples a batch",
					next.rate_hz, next.accel_g, next.gyro_dps, batch);
			} else {
				LOG_INF("IMU off");
			}
		}

		if (!active.enabled) {
			k_msleep(50);
			continue;
		}

		k_msleep(poll_ms_for(active.rate_hz));

		if (drain() != 0 && (bus_errors++ % 1000u) == 0u) {
			LOG_WRN("IMU read failed (%u so far)", bus_errors);
		}
	}
}

/* --- interface ------------------------------------------------------------ */

int imu_init(void)
{
	if (!spi_is_ready_dt(&bus)) {
		LOG_ERR("IMU SPI bus not ready");
		return -ENODEV;
	}

	uint8_t id = 0;

	if (reg_read(REG_WHO_AM_I, &id, 1) != 0 || id != WHO_AM_I_VALUE) {
		LOG_WRN("IMU LSM6DSV16X not found (WHO_AM_I 0x%02x, expected 0x%02x)",
			id, WHO_AM_I_VALUE);
		return -ENODEV;
	}

	/* Reset, so nothing survives from before a warm restart. */
	(void)reg_write(REG_CTRL3, CTRL3_SW_RESET);

	uint8_t ctrl3 = CTRL3_SW_RESET;

	for (int i = 0; i < 50 && (ctrl3 & CTRL3_SW_RESET); i++) {
		k_msleep(2);
		if (reg_read(REG_CTRL3, &ctrl3, 1) != 0) {
			ctrl3 = CTRL3_SW_RESET;
		}
	}

	if (ctrl3 & CTRL3_SW_RESET) {
		LOG_ERR("IMU did not come out of reset");
		return -EIO;
	}

	/*
	 * SPI only, so switch off the I2C/I3C front end, which would otherwise
	 * watch clock and data activity on the shared pins as bus traffic.
	 */
	int err = reg_update(REG_IF_CFG, IF_CFG_I2C_I3C_DISABLE,
			     IF_CFG_I2C_I3C_DISABLE);

	err |= reg_write(REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC);
	if (err) {
		LOG_ERR("IMU setup failed (%d)", err);
		return -EIO;
	}

	/*
	 * The sensor's own estimate of its clock error. Logged so the period
	 * measured against TIMER1 has something independent to be checked
	 * against.
	 */
	uint8_t freq = 0;

	if (reg_read(REG_INTERNAL_FREQ, &freq, 1) == 0) {
		const int hundredths = (int)(int8_t)freq * 13;

		LOG_INF("IMU clock %s%d.%02d %% from nominal, by its own estimate",
			(hundredths < 0) ? "-" : "+", abs(hundredths) / 100,
			abs(hundredths) % 100);
	}

	err = capture_edge_init(NRF_GPIO_PIN_MAP(INT2_PORT, INT2_PIN), true,
				timebase_imu_capture_task_addr());
	if (err) {
		LOG_WRN("IMU INT2 capture unavailable (%d): motion timestamps "
			"will come from polling", err);
	}

	present = true;
	batch = batch_for(active.rate_hz);
	timing_reset(active.rate_hz);

	requested = active;
	atomic_set(&config_pending, 1);

	k_tid_t tid = k_thread_create(&imu_thread, imu_stack, IMU_STACK_SIZE,
				      imu_entry, NULL, NULL, NULL,
				      IMU_PRIORITY, 0, K_NO_WAIT);

	k_thread_name_set(tid, "imu");

	LOG_INF("IMU LSM6DSV16X present (WHO_AM_I 0x%02x), INT2 on P%u.%02u",
		id, INT2_PORT, INT2_PIN);

	return 0;
}

bool imu_present(void)
{
	return present;
}

int imu_configure(const struct imu_config *cfg)
{
	if (!present) {
		return -ENODEV;
	}

	if (cfg->enabled && (odr_code(cfg->rate_hz) < 0 ||
			     accel_fs_code(cfg->accel_g) < 0 ||
			     gyro_fs_code(cfg->gyro_dps) < 0)) {
		return -EINVAL;
	}

	struct imu_config next = *cfg;

	if (!next.enabled) {
		/* Keep the last rate and ranges, so turning it back on needs
		 * only the switch. */
		k_spinlock_key_t key = k_spin_lock(&config_lock);

		next.rate_hz = active.rate_hz;
		next.accel_g = active.accel_g;
		next.gyro_dps = active.gyro_dps;
		k_spin_unlock(&config_lock, key);
	}

	k_spinlock_key_t key = k_spin_lock(&config_lock);

	requested = next;
	k_spin_unlock(&config_lock, key);

	atomic_set(&config_pending, 1);
	return 0;
}

void imu_get_config(struct imu_config *out)
{
	k_spinlock_key_t key = k_spin_lock(&config_lock);

	*out = active;
	k_spin_unlock(&config_lock, key);

	if (!present) {
		out->enabled = false;
	}
}

void imu_get_stats(struct imu_stats *out)
{
	*out = stats;
}
