"""Guarded README edits for M6 (filters on the device)."""
import pathlib

P = pathlib.Path(r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\README.md")

M6_OLD = '''### M6 — push the validated chain into the firmware

The settings found in M5, and the motion cleanup, become the device's own,
which is what the original plan always called for.

The mechanism already exists in the design: the DSP chain's last stage is a
**host-programmable biquad cascade**. The host uploads coefficients; firmware
runs them. Nothing needs redesigning - the host application becomes the tool
that designs the coefficients it then uploads.

**Accept:** with the host chain bypassed, on-device output matches what the
host chain produced from the same raw input, within tolerance.
'''

M6_NEW = '''### M6 — push the validated chain into the firmware  (filters: built)

The settings found in M5, and the motion cleanup, become the device's own,
which is what the original plan always called for. The filter half is built;
the motion half waits for the cleanup to exist.

**In the app:** *filters (display) - run on: device.* The app sends the chain
it designed and switches the stream to raw and filtered side by side: the
plot shows what the device computed, and the recording stays raw. Settings,
the "in average" ticks and the mains tracking follow onto the device as they
change, and only a stage that changed is sent, so moving the low-pass does
not restart the high-pass. If the device refuses anything, the filters go
back to the PC and the panel says why.

#### The chain on the device

```
raw counts -> integer DC removal, 0.08 Hz -> microvolts at each channel's gain
           -> pre sections: high-pass, notch and its harmonic
           -> common average over the ticked channels
           -> post sections: low-pass
```

Up to eight sections a stage. Left alone, the pre stage is the device's own
50 Hz notch and the post stage is empty, as before.

A filter change lands between two samples, never inside one: the command
thread hands it to the DSP thread, which applies it at the top of the next
sample and reports which sample that was. Samples inside a filter's settling
time carry the protocol's SETTLING flag.

The one intended difference from the PC chain is the DC removal in front,
which exists for float32 headroom. It is first order at 0.08 Hz, so the two
differ by 3 % at 0.3 Hz and by under 1 % from 0.6 Hz up.

#### Why the sections are state-variable filters

Measured before anything was built: a fourth-order Butterworth high-pass in
float32, against the same filter in float64, on 3 mV of drift with EEG on top.

```
                        transposed direct form II   state-variable (TPT)
0.1 Hz at  250 SPS      2.0 uV RMS                  0.02 uV RMS
0.1 Hz at 1000 SPS      13.6 uV RMS, 28 uV peak     0.08 uV RMS
0.3 Hz at 1000 SPS      0.59 uV RMS                 0.012 uV RMS
45 Hz low-pass, notch   under 0.004 uV RMS          under 0.001 uV RMS
```

The direct form - what the audio-EQ cookbook formulas are written for, and
what the firmware first had - is fine for a notch and unusable for the 0.1 Hz
drift cut ERP work needs. Its poles sit almost on z = 1, float32 rounds away
most of its coefficients' precision, and every rounding error in its state is
amplified by the filter's own gain near DC. A state-variable filter with
trapezoidal integrators (Zavalishin's topology-preserving transform, in
Simper's form) keeps its state as integrators of the signal and its
coefficients as small numbers. It is the same filter: the two forms agree to
under 1e-6 in float64.

A section is five float32 numbers: `g = tan(pi fc / fs)`, `k = 1/Q`, and how
the output mixes the input, band-pass and low-pass (`m0 m1 m2`). With `g` and
`k` positive a section is stable whatever its mix, so the device can check
what it is sent. Any stable biquad converts: `dsp_ref.from_biquad`.

#### On the wire

`CMD_SET_FILTER` (`0x10`): stage (0 before the average, 1 after), flags (bit 0
keep state, for a retune in place), the rate the sections were designed for,
the count, then five float32 per section. Refused if the rate is not the
running one - a filter designed for one rate is a different filter at
another - or if a section is invalid.

`CMD_SET_CAR` (`0x11`): enable, channel mask. `CMD_RESET_CHAIN` (`0x12`):
restart every filter and re-prime the DC removal from the next sample. All
three answer with the sequence number of the first sample they apply to.

Encoding `3`, raw and filtered: for every channel, three bytes of counts then
four of microvolts. Three samples a frame, so a frame still fits one
notification.

`CMD_GET_CONFIG` appends sections per stage; flags (bit 0 average on, bit 1
the pre stage is the device's notch); the average's mask; and a CRC-16 of
each stage's sections, so a host can tell whether the device still holds
what it sent. A rate change drops sections designed for the old rate.

#### Verified

- **Golden vectors.** Both references were rewritten for the sections
  (`tools/dsp_ref.py`, `tools/pipeline_ref.py`), including the whole host
  chain with a notch retune, a change of average and a restart partway
  through. The on-target suites run the firmware's own code against them.
- **The app against a simulated device** running the reference chain: the
  plot is the device's output to 0.00 uV, the recording is the raw counts, a
  low-pass change sends only the post stage, a mains re-aim retunes in place,
  a rate change re-sends for the new rate, and a refusal falls back to the
  PC. The older app checks still pass: 144 fps at 250-1000 SPS, bad-channel
  handling, motion lanes.
- **On the chip: pending.** `python tools/verify_chain.py` loads a chain,
  restarts it at a sample the device reports, and checks the device's output
  against the reference model run on the raw counts - on the ADS1299's own
  test signal, with no electrodes.

#### Found along the way

- **The notch could not be switched off.** `CMD_SET_NOTCH 0` reported success
  and left it running, and a rate change after it would have failed to
  design a 0 Hz notch and stopped acquisition.
- **Filter changes raced the DSP thread.** The notch was rebuilt from the
  command thread while samples were being filtered.
- **Device microvolts ignored channel gains.** The chain scaled every channel
  for gain 24, whatever it was set to.
- **The DC corner moved with the rate:** 0.08 Hz at 250 SPS but 0.31 Hz at
  1000 SPS, inside the range that distorts slow ERP components. It now holds
  0.08 Hz at every rate.
- **The device notch was Q 30,** 1.7 Hz wide, aimed at 50.0 Hz against mains
  measured at 49.6 - about 7 dB of rejection. It is Q 12 now, like the app's.

**Accept:** with the host chain bypassed, on-device output matches what the
host chain produced from the same raw input, within tolerance.
'''

EDITS = [
('''> **Status: M1-M4 done, M5 Windows app working, motion sensor streaming on
> the EEG's clock. Next: recordings of a moving subject, then motion-artifact
> cleanup.**
''',
'''> **Status: M1-M4 done, M5 Windows app working, motion sensor streaming on
> the EEG's clock. M6 under way: the app's filter chain now runs on the
> device too (verified on the PC; the on-chip check is pending).
> Motion-artifact recordings wait for an electrode solder fix.**
'''),

('''- Chain: 24-bit decode, integer DC removal, microvolt scaling, mains notch
''',
'''- Chain: 24-bit decode, integer DC removal, per-channel microvolt scaling,
  then two host-programmable stages either side of a common average - the
  Windows app's whole filter chain can run on the device (M6)
'''),

('''- The on-device filter chain is not yet matched to the host chain (M6).
''',
'''- Motion cleanup is not on the device - it has to be built first.
'''),

('''src/dsp/                    DC removal, biquads, filter design
''',
'''src/dsp/                    DC removal, second-order sections, filter design
'''),

('''tools/verify.py             hardware acceptance checks, pass/fail
''',
'''tools/verify.py             hardware acceptance checks, pass/fail
tools/verify_chain.py       on-device filter chain vs the reference, no electrodes
'''),

('''tests/dsp/                  17 tests  (dsp + ringbuf suites)
tests/timebase/              8 tests
tests/pipeline/              8 tests  (whole-chain golden vectors)
''',
'''tests/dsp/                  22 tests  (dsp + ringbuf suites)
tests/timebase/              8 tests
tests/pipeline/             13 tests  (whole-chain golden vectors)
'''),

(M6_OLD, M6_NEW),

('''Firmware does only what *must* happen on-chip - DC removal for numeric
headroom, mains notch, anti-alias decimation, IMU artifact removal - and
exposes everything else as a **host-programmable biquad cascade**. No
filtering opinion is baked in.

Total group delay of the active configuration is computed and reported, so
the host can correct sample timestamps exactly.
''',
'''Firmware does only what *must* happen on-chip - DC removal for numeric
headroom, mains notch, anti-alias decimation, IMU artifact removal - and
exposes everything else as **host-programmable second-order sections**. No
filtering opinion is baked in.

The group delay of any section can be computed (`dsp_section_group_delay`),
which is what correcting timestamps for the filters in use needs. It is not
yet reported in the stream.
'''),

('''M5 tunes the chain on the host, where a change is visible in a second. M6
moves the settled chain onto the device, where it belongs.

That transfer needs no new architecture. The chain's last stage is a
programmable biquad cascade: the host designs coefficients and uploads them.
The host application becomes the tool that designs what it then installs, and
the acceptance test for M6 is that the device reproduces, from the same raw
input, what the host chain produced.
''',
'''M5 tunes the chain on the host, where a change is visible in a second. M6
moves the settled chain onto the device, where it belongs - and for the
filters that is now a switch in the app (see M6).

The host designs and the device runs: the app sends the sections it designed,
and the device reproduces, from the same raw input, what the host chain
produced - apart from the DC removal it keeps in front for numeric headroom.
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
