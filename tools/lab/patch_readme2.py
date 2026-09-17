"""README: on-chip results for M6, and the two register-access bugs."""
import pathlib

P = pathlib.Path(r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\README.md")

EDITS = [
('''> **Status: M1-M4 done, M5 Windows app working, motion sensor streaming on
> the EEG's clock. M6 under way: the app's filter chain now runs on the
> device too (verified on the PC; the on-chip check is pending).
> Motion-artifact recordings wait for an electrode solder fix.**
''',
'''> **Status: M1-M4 done, M5 Windows app working, motion sensor streaming on
> the EEG's clock. M6: the app's filter chain runs on the device too, and
> matches the reference bit for bit on the chip. Motion-artifact recordings
> wait for an electrode solder fix.**
'''),

('''> | Golden vectors vs reference | worst 3.4 nV over 512 frames x 8 ch |
> | Unit tests on target | 42/42 |
''',
'''> | Golden vectors vs reference | worst 3.4 nV over 512 frames x 8 ch |
> | On-device filter chain vs reference | **0.00 uV** at 250 / 500 / 1000 SPS over BLE, 0 gaps |
> | Register reads while streaming, 1 kSPS | 60 reads, **0 stale samples** (60 before the fix) |
> | Unit tests on target | 52/52 |
'''),

('''Nothing was dropped while the AFE was reconfigured mid-stream. Three things
make that true:
''',
'''Nothing was dropped while the AFE was reconfigured mid-stream - no sequence
gaps. Each register access did slip one old sample in, though, unnoticed
until M6; see the mistakes below. Three things keep the device responsive:
'''),

('''**More than one thread writes to USB.** The DSP thread's samples, the IMU
thread's motion frames and the command thread's replies all go into one
ring buffer that is only safe for a single writer. Writers now take a lock,
and a frame goes in whole or not at all.
''',
'''**More than one thread writes to USB.** The DSP thread's samples, the IMU
thread's motion frames and the command thread's replies all go into one
ring buffer that is only safe for a single writer. Writers now take a lock,
and a frame goes in whole or not at all.

**Every register access during streaming slipped in an old sample.** A
setting change takes the AFE out of continuous-read mode and back, and the
DMA was left pointing at the one-byte command buffers. The first frame
afterwards was read one byte long, and the interrupt passed an old frame on
as if it were new. Sequence numbers had no gap, so nothing noticed. Measured
at 1 kSPS with the inputs shorted: 60 register reads, 60 samples identical to
the one two before. The resume path now points the DMA back at the frame
buffers - 0 in 60. (`CMD_GET_CONFIG` reads the channel registers, so even
asking the device its state did this.)

**A register access could wedge the SPI bus.** A DRDY edge just before the
trigger was gated off had already started a transfer, and the driver changed
the clock and buffers underneath it. Now and then at 1 kSPS a transfer timed
out, every transfer after it timed out too, and the next rate change failed
to restart acquisition - after which the firmware ignored every rate change
until reset, waiting for a "next start" nothing would request. The driver now
lets that last transfer finish first, recovers the peripheral if a transfer
ever sticks, and a failed restart is retried rather than parked. The chain
also takes channel gains from what the driver wrote, instead of reading them
back after every change.
'''),

('''- **On the chip: pending.** `python tools/verify_chain.py` loads a chain,
  restarts it at a sample the device reports, and checks the device's output
  against the reference model run on the raw counts - on the ADS1299's own
  test signal, with no electrodes.
''',
'''- **On the chip,** over Bluetooth, with `python tools/verify_chain.py`: a
  chain loaded, restarted at a sample the device reported, and the device's
  output compared with the reference model run on the raw counts streamed
  beside it - on the ADS1299's own test signal, channel 2 at gain 12 and
  channel 7 at gain 6, no electrodes.

  ```
   250 SPS   3004 samples x 8 ch   worst 0.00 uV   0 gaps   250.3 SPS delivered
   500 SPS   4004 samples x 8 ch   worst 0.00 uV   0 gaps   500.2 SPS delivered
  1000 SPS   8136 samples x 8 ch   worst 0.00 uV   0 gaps  1014.7 SPS delivered
  ```

  Bit for bit. Also checked there: sections for the wrong rate and unstable
  sections refused; section CRCs read back; the common average moving 1.4 mV
  of test square wave onto the shorted channels; settling flagged for 787
  samples against 785 expected. On-target suites: dsp 22/22, with the 0.1 Hz
  high-pass 0.00 uV from its reference, and pipeline 13/13, with the whole
  host chain 0.00 uV.
'''),

('''- **The device notch was Q 30,** 1.7 Hz wide, aimed at 50.0 Hz against mains
  measured at 49.6 - about 7 dB of rejection. It is Q 12 now, like the app's.
''',
'''- **The device notch was Q 30,** 1.7 Hz wide, aimed at 50.0 Hz against mains
  measured at 49.6 - about 7 dB of rejection. It is Q 12 now, like the app's.
- **Register access during streaming** slipped an old sample in each time,
  and could wedge the SPI bus. Both are older than M6; see *Mistakes worth
  keeping* under M4.
'''),
]


def main():
    raw = P.read_bytes()
    crlf = b"\r\n" in raw
    s = raw.decode("utf-8").replace("\r\n", "\n")
    for i, (old, new) in enumerate(EDITS, 1):
        n = s.count(old)
        assert n == 1, f"edit {i}: found {n} times"
        s = s.replace(old, new)
    if crlf:
        s = s.replace("\n", "\r\n")
    P.write_bytes(s.encode("utf-8"))
    print(f"applied {len(EDITS)} README edits")


if __name__ == "__main__":
    main()
