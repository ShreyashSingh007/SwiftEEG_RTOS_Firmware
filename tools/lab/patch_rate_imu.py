"""Guarded firmware edits: rate changes keep the AFE setup; ~20 ms motion batches."""
import pathlib

ROOT = pathlib.Path(r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS")

FILES = {
"src/afe/ads1299.h": [
('''int ads1299_configure(uint8_t rate);
''',
'''int ads1299_configure(uint8_t rate);

/*
 * Change only the data rate, leaving every other setting as it is: channel
 * gains and inputs, bias drive, lead-off. For a restart at a new rate on a
 * part already configured. ads1299_configure() would put every channel back
 * to gain 24 on the electrodes and switch the bias drive off for over 150 ms
 * while the reference settles - on a head, a common-mode step that every
 * electrode sees.
 *
 * Expects the part stopped, as ads1299_stream_stop() leaves it.
 */
int ads1299_set_data_rate(uint8_t rate);
'''),
],

"src/afe/ads1299.c": [
('''/*
 * Registers are only writable in command mode. While acquiring, the part is
''',
'''int ads1299_set_data_rate(uint8_t rate)
{
	/* Registers are only writable outside continuous-read mode. */
	int err = afe_cmd(ADS1299_CMD_SDATAC);

	if (err) {
		return err;
	}
	k_busy_wait(10);

	const uint8_t want = ADS1299_CONFIG1_BASE | (rate & 0x07u);

	err = afe_write_reg(ADS1299_REG_CONFIG1, want);
	if (err) {
		return err;
	}

	uint8_t back = 0;

	err = afe_read_reg(ADS1299_REG_CONFIG1, &back);
	if (err) {
		return err;
	}
	if (back != want) {
		LOG_ERR("CONFIG1 readback %02x, wanted %02x", back, want);
		return -EIO;
	}

	LOG_INF("AFE data rate changed (CONFIG1 %02x), other settings kept", back);
	return 0;
}

/*
 * Registers are only writable in command mode. While acquiring, the part is
'''),
],

"src/pipeline/pipeline.c": [
('''static volatile bool running;
static uint8_t current_rate_code;
''',
'''static volatile bool running;
static uint8_t current_rate_code;

/* The AFE has had its full configuration once. */
static bool afe_configured;
'''),

('''	int err = ads1299_configure(rate);

	if (err) {
		return err;
	}

	err = build_chain();
''',
'''	/*
	 * A restart at a new rate changes the rate and nothing else. The full
	 * configuration would put every channel back to gain 24 on the
	 * electrodes, lose the bias and lead-off settings, and switch the bias
	 * drive off for over 150 ms - on a head, a common-mode step that every
	 * electrode sees. It runs at the first start, and a restart falls back
	 * on it if the lighter change fails.
	 */
	int err = afe_configured ? ads1299_set_data_rate(rate) : -EAGAIN;

	if (err) {
		err = ads1299_configure(rate);
	}
	if (err) {
		return err;
	}
	afe_configured = true;

	err = build_chain();
'''),
],

"src/pipeline/pipeline.h": [
('''back to the device's own notch and the post stage empties. The common
 * average settings are kept, since they do not depend on the rate.
''',
'''back to the device's own notch and the post stage empties. The common
 * average settings are kept, since they do not depend on the rate, and so is
 * everything set on the AFE: channel gains and inputs, bias drive, lead-off.
'''),
],

"src/imu/imu.c": [
('''/*
 * Samples per watermark, and how often to look for it. A late look adds
 * samples to the batch, and a batch has to stay within one frame's 17.
 */
static uint8_t batch_for(uint16_t hz)
{
	return (hz >= 960) ? 8 : (hz >= 480) ? 10 : 12;
}
''',
'''/*
 * Samples per watermark, and how often to look for it.
 *
 * About 20 ms of samples a batch at every rate. A batch is also how long its
 * newest sample waits before it is sent, and at 12 a batch - 50 ms at 240 Hz
 * - the motion trace in the app fell short of the EEG's and caught up in
 * jumps. A late look adds samples to a batch, which has to stay within one
 * frame's 17, so 960 Hz keeps 8 and a faster look.
 */
static uint8_t batch_for(uint16_t hz)
{
	switch (hz) {
	case 60:  return 2;  /* 33 ms */
	case 120: return 3;  /* 25 ms */
	case 240: return 5;  /* 21 ms */
	case 480: return 10; /* 21 ms */
	default:  return 8;  /* 960 Hz, 8 ms */
	}
}
'''),

('''/*
 * `active` is written only by the IMU thread and read elsewhere under the
 * lock; `requested` is the reverse.
 */
static struct imu_config active = {
''',
'''/*
 * `active` is written only by the IMU thread and read elsewhere under the
 * lock; `requested` is the reverse.
 *
 * The defaults, from the datasheet (DS13510) and the job - a head-worn
 * reference for motion artifacts, and later the machine learning core:
 *
 *   240 Hz      the machine learning core's highest rate (MLC_ODR), so one
 *               stream can feed both. High-performance mode keeps the sensor's
 *               anti-aliasing filter at ODR/2, so 120 Hz of motion is sampled
 *               honestly.
 *   +/-8 g      head acceleration walking, running and jumping, with room for
 *               knocks. Noise density is the same at every range (60 ug/rtHz),
 *               so the wider range costs nothing: noise is still ~3 counts.
 *   +/-2000 dps fast head shakes reach several hundred degrees a second.
 *               Rate noise (2.8 mdps/rtHz) is range-independent up to here.
 */
static struct imu_config active = {
'''),
],
}


def main():
    # All or nothing: every edit is checked before any file is written.
    done = []
    for rel, edits in FILES.items():
        p = ROOT / rel
        s = p.read_bytes().decode("utf-8")
        assert "\r\n" not in s, f"{rel} has CRLF line endings"
        for i, (old, new) in enumerate(edits, 1):
            n = s.count(old)
            assert n == 1, f"{rel} edit {i}: found {n} times"
            s = s.replace(old, new)
        done.append((rel, p, s, len(edits)))
    for rel, p, s, n in done:
        p.write_bytes(s.encode("utf-8"))
        print(f"{rel}: {n} edits")


if __name__ == "__main__":
    main()
