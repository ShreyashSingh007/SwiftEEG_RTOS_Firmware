"""Guarded edits: AFE register access during streaming, and restart recovery."""
import pathlib

ROOT = pathlib.Path(r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS")

FILES = {
"src/afe/ads1299.c": [
# frame buffers move up, beside the command buffers the resume path now needs
('''/*
 * Two frame buffers. EasyDMA writes one while the callback reads the other,
 * so a frame is never being overwritten while it is being consumed. They
 * must live in RAM - EasyDMA cannot reach flash.
 */
static uint8_t afe_frame[2][ADS1299_FRAME_BYTES];
static uint8_t afe_dummy[ADS1299_FRAME_BYTES];
static uint8_t afe_active;

static ads1299_frame_cb_t afe_cb;
''',
'''static ads1299_frame_cb_t afe_cb;
'''),

('''static uint8_t afe_tx[8];
static uint8_t afe_rx[8];
''',
'''static uint8_t afe_tx[8];
static uint8_t afe_rx[8];

/*
 * Two frame buffers. EasyDMA writes one while the callback reads the other,
 * so a frame is never being overwritten while it is being consumed. They
 * must live in RAM - EasyDMA cannot reach flash.
 */
static uint8_t afe_frame[2][ADS1299_FRAME_BYTES];
static uint8_t afe_dummy[ADS1299_FRAME_BYTES];
static uint8_t afe_active;

/*
 * What was last written to each CHnSET. Every write goes through this
 * driver, so this is what the part holds - from its reset value, gain 24
 * with the inputs shorted, onwards.
 */
static uint8_t afe_chset[ADS1299_CHANNELS] = {
	0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61,
};

/*
 * How long a transfer DRDY had already started may still be running once
 * the trigger is gated off: 27 bytes at 8 MHz take ~30 us.
 */
#define AFE_XFER_SETTLE_US 500
'''),

# recovery from a stuck transfer
('''/* Blocking transfer. Used for register access only; streaming is DMA. */
static int afe_xfer(size_t len)
''',
'''/*
 * Put a stuck SPIM3 back in working order. STOP abandons whatever transfer
 * it believes is running, and a disable-enable cycle clears the rest; the
 * configuration registers survive both. Without this, one stuck transfer
 * made every one after it time out as well, and acquisition could not be
 * restarted short of a reset.
 */
static void afe_recover(void)
{
	nrf_spim_task_trigger(AFE_SPIM, NRF_SPIM_TASK_STOP);

	for (int i = 0; i < 100; i++) {
		if (nrf_spim_event_check(AFE_SPIM, NRF_SPIM_EVENT_STOPPED)) {
			break;
		}
		k_busy_wait(100);
	}

	nrf_spim_event_clear(AFE_SPIM, NRF_SPIM_EVENT_STOPPED);
	nrf_spim_disable(AFE_SPIM);
	nrf_spim_enable(AFE_SPIM);
	nrf_spim_event_clear(AFE_SPIM, NRF_SPIM_EVENT_END);
}

/* Blocking transfer. Used for register access only; streaming is DMA. */
static int afe_xfer(size_t len)
'''),

('''	afe_xfer_done();
	LOG_ERR("AFE SPI transfer timed out");
	return -ETIMEDOUT;
''',
'''	afe_recover();
	afe_xfer_done();
	LOG_ERR("AFE SPI transfer timed out");
	return -ETIMEDOUT;
'''),

# let an in-flight hardware transfer finish before touching the peripheral
('''	if (afe_gate != NULL) {
		afe_gate(false);
	}

	/*
	 * Drop back to the register clock.''',
'''	if (afe_gate != NULL) {
		afe_gate(false);
	}

	/*
	 * An edge just before the gate closed may already have started a
	 * transfer, which runs on for ~30 us. Touching the peripheral before it
	 * ends - a new clock rate, new buffers, a START while it is busy - left
	 * SPIM3 stuck: at 1000 SPS a register access now and then timed out,
	 * every transfer after it did too, and the next rate change could not
	 * restart acquisition. Waiting it out costs a register access nothing,
	 * and the streaming interrupt takes that last frame as normal.
	 */
	k_busy_wait(AFE_XFER_SETTLE_US);

	/*
	 * Drop back to the register clock.'''),

# the DMA goes back to the frame buffers on resume
('''	err = afe_cmd(ADS1299_CMD_START);

	/* Payload rate again now the commands are done. */
	nrf_spim_frequency_set(AFE_SPIM, AFE_FREQ_STREAM);
''',
'''	err = afe_cmd(ADS1299_CMD_START);

	/*
	 * Point the DMA back at the frame buffers. Register access leaves it on
	 * the one-byte command buffers, so without this the first frame after
	 * every register access was read one byte long, and the interrupt
	 * passed on an old frame as if it were new.
	 */
	nrf_spim_tx_buffer_set(AFE_SPIM, afe_dummy, ADS1299_FRAME_BYTES);
	nrf_spim_rx_buffer_set(AFE_SPIM, afe_frame[afe_active],
			       ADS1299_FRAME_BYTES);

	/* Payload rate again now the commands are done. */
	nrf_spim_frequency_set(AFE_SPIM, AFE_FREQ_STREAM);
'''),

# the CHnSET shadow follows every write
('''	if (ch == 0xFFu) {
		for (uint8_t i = 0; i < ADS1299_CHANNELS && err == 0; i++) {
			err = afe_write_reg(ADS1299_REG_CH1SET + i, val);
		}
	} else {
		err = afe_write_reg(ADS1299_REG_CH1SET + ch, val);
	}
''',
'''	if (ch == 0xFFu) {
		for (uint8_t i = 0; i < ADS1299_CHANNELS && err == 0; i++) {
			err = afe_write_reg(ADS1299_REG_CH1SET + i, val);
			if (err == 0) {
				afe_chset[i] = val;
			}
		}
	} else {
		err = afe_write_reg(ADS1299_REG_CH1SET + ch, val);
		if (err == 0) {
			afe_chset[ch] = val;
		}
	}
'''),

('''		if (err) {
			LOG_ERR("CH%uSET write failed (%d)", i + 1, err);
			return err;
		}
	}
''',
'''		if (err) {
			LOG_ERR("CH%uSET write failed (%d)", i + 1, err);
			return err;
		}

		afe_chset[i] = val;
	}
'''),

('''const char *afe_probe_str(afe_probe_result_t r)
''',
'''void ads1299_get_channels_cached(uint8_t *out, uint8_t count)
{
	if (out != NULL && count <= ADS1299_CHANNELS) {
		memcpy(out, afe_chset, count);
	}
}

const char *afe_probe_str(afe_probe_result_t r)
'''),
],

"src/afe/ads1299.h": [
('''/* Current CHnSET contents, for reporting configuration to a host. */
int ads1299_get_channels(uint8_t *out, uint8_t count);
''',
'''/* Current CHnSET contents, for reporting configuration to a host. */
int ads1299_get_channels(uint8_t *out, uint8_t count);

/*
 * The CHnSET values this driver last wrote, without touching the bus. Every
 * write goes through the driver, so this is what the part holds. The chain
 * takes its gains from here: a register read pauses acquisition for a few
 * milliseconds, a price worth paying to report state to a host but not to
 * rescale after every command.
 */
void ads1299_get_channels_cached(uint8_t *out, uint8_t count);
'''),
],

"src/pipeline/pipeline.c": [
('''	/* The gains the AFE is really set to, not the ones assumed. */
	uint8_t chset[ADS1299_CHANNELS];

	if (ads1299_get_channels(chset, ADS1299_CHANNELS) == 0) {
		for (uint8_t ch = 0; ch < ADS1299_CHANNELS; ch++) {
			(void)chain_set_gain(&chain, ch, gain_from_chset(chset[ch]));
		}
	} else {
		LOG_WRN("could not read the channel gains; assuming 24");
	}
''',
'''	/* The gains the channels are set to, not the ones assumed. */
	uint8_t chset[ADS1299_CHANNELS];

	ads1299_get_channels_cached(chset, ADS1299_CHANNELS);
	for (uint8_t ch = 0; ch < ADS1299_CHANNELS; ch++) {
		(void)chain_set_gain(&chain, ch, gain_from_chset(chset[ch]));
	}
'''),

('''	uint8_t chset[ADS1299_CHANNELS];
	const int err = ads1299_get_channels(chset, ADS1299_CHANNELS);

	if (err) {
		return err;
	}

	struct chain_req r = { .kind = REQ_GAINS };
''',
'''	uint8_t chset[ADS1299_CHANNELS];

	ads1299_get_channels_cached(chset, ADS1299_CHANNELS);

	struct chain_req r = { .kind = REQ_GAINS };
'''),

('''	const bool was_running = running;

	if (was_running) {
		pipeline_stop();
	}

	if (!was_running) {
		return 0; /* takes effect at the next start */
	}

	const int err = pipeline_start(code);

	if (err) {
		LOG_ERR("could not restart at %u SPS (%d)", sps, err);
''',
'''	/*
	 * Not running here means the last restart failed: nothing else stops
	 * acquisition once it has begun. So start regardless. Returning early
	 * for it - "takes effect at the next start" - left a board whose
	 * restart had failed ignoring every rate change after it until reset,
	 * because no next start ever came.
	 */
	pipeline_stop();

	int err = pipeline_start(code);

	if (err) {
		LOG_WRN("restart at %u SPS failed (%d), trying once more", sps, err);
		err = pipeline_start(code);
	}

	if (err) {
		LOG_ERR("could not restart at %u SPS (%d)", sps, err);
'''),
],

"src/pipeline/pipeline.h": [
('''/*
 * Re-read every channel's gain from the AFE and scale the chain to match.
 * Call after anything that changes a gain, or microvolts from the chain are
 * wrong by the ratio of the old gain to the new.
 */''',
'''/*
 * Take every channel's gain from what the AFE driver last wrote, and scale
 * the chain to match. Call after anything that changes a gain, or microvolts
 * from the chain are wrong by the ratio of the old gain to the new.
 */'''),
],
}


def main():
    for rel, edits in FILES.items():
        p = ROOT / rel
        raw = p.read_bytes()
        crlf = b"\r\n" in raw
        s = raw.decode("utf-8").replace("\r\n", "\n")
        for i, (old, new) in enumerate(edits, 1):
            n = s.count(old)
            assert n == 1, f"{rel} edit {i}: found {n} times"
            s = s.replace(old, new)
        if crlf:
            s = s.replace("\n", "\r\n")
        p.write_bytes(s.encode("utf-8"))
        print(f"{rel}: {len(edits)} edits")


if __name__ == "__main__":
    main()
