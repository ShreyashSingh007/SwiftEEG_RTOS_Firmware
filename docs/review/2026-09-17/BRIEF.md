# SwiftEEG deep review — shared brief (read fully before starting)

## 1. What this is

SwiftEEG: a wireless 8-channel EEG headset. nRF52840 (Cortex-M4F, 64 MHz, 1 MB
flash, 256 KB RAM) + TI ADS1299 (24-bit, 8 ch) + ST LSM6DSV16X IMU (on the
head). Firmware on nRF Connect SDK v3.4.0 (Zephyr 4.4, SoftDevice Controller),
written in C. Host software so far: a Windows desktop app in Python/Tk.

- Repo (git): `D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS`
  Branch `m1-bringup` at commit `6a094b6`. Review THIS tree.
- SDK sources (read-only, for checking API semantics against the real code,
  never from memory): `C:\ncs\v3.4.0\zephyr`, `C:\ncs\v3.4.0\nrf`,
  `C:\ncs\v3.4.0\modules\hal\nordic` (nrfx). A previous build may exist under
  `RTOS\build\zephyr\` (`.config` = effective Kconfig, `zephyr.map`).
- Documentation: `RTOS\README.md` (the single project doc; ~1130 lines) and the
  original plan at `C:\Users\shrey\.claude\plans\d-electronics-projects-eeg-project-swif-peppy-peach.md`.

## 2. The owner's goals (review against these)

1. "Really capable, flexible firmware that gives full granular control over
   the hardware wirelessly, and clean, state-of-the-art EEG signal — the best
   possible from the device."
2. Signal integrity is sacred: recordings stay raw; ERP work with deep
   learning is planned, so timing must be trustworthy (hardware-latched DRDY
   timestamps, measured true sample rate, reported group delay) and a 0.1 Hz
   drift cut must remain usable. IMU samples are never repeated, interpolated
   or averaged to match EEG.
3. **Native multiplatform apps soon: Windows, macOS, iOS, Android.** They must
   share as much as possible — business logic, protocol/integration pipelines,
   DSP, filter design, timing model — as shared native libraries, for maximum
   consistency and performance on every platform, with no bottleneck or
   performance compromise anywhere in the chain (firmware -> radio -> host).
4. All processing ultimately on the MCU (the host chain is where DSP is tuned
   and validated before it moves on-device, milestone M6).

## 3. Architecture as built (verify, do not trust)

```
DRDY -GPIOTE-PPI-+-> TIMER1 CAPTURE (sample timestamp, 0 CPU)
                 +-> SPIM3 TASKS_START (27-byte DMA read, 0 CPU)
SPIM3 END ISR -> SPSC raw ring (128 frames) -> DSP thread (prio 2)
  chain: int24 decode -> integer DC removal -> per-channel uV scaling ->
         "pre" stage (host-uploaded state-variable filter sections) ->
         common average reference (mask) -> "post" stage -> mains notch
         (+ harmonic) aimed by an on-device mains-frequency tracker (FLL)
  -> stream_send(): copy into a 32-frame k_msgq -> stream TX thread (prio 4)
     -> bt_gatt_notify (blocking) and/or USB CDC write
IMU: LSM6DSV16X FIFO, INT2 -> PPI capture on TIMER1 (same clock as EEG),
     accel+gyro pairs, per-batch timestamp + measured period.
Commands: CMD frames over BLE (write) or USB, handled off the BT thread,
     RSP frames back; EVT frames async.
```

- Frame: `SOF | ver | type | flags | len(LE16) | seq | payload | CRC`.
  Types CMD 0x01, RSP 0x02, EVT 0x03, DATA 0x04, IMU 0x05.
- DATA encodings: ENC_RAW_I24 (3 B/ch), ENC_UV_F32 (4 B/ch), ENC_RAW_UV
  (raw + uV, 7 B/ch = 56 B/sample). 6 samples a DATA frame (3 for RAW_UV).
- Flags: SETTLING 0x01, OVERRUN 0x02.
- Rates in use: 250 / 500 / 1000 SPS (plan targeted up to 16 kSPS over USB).
- BLE: custom 128-bit service (control write, stream notify, event notify),
  2M PHY, DLE, MTU 247; firmware re-asks for a 7.5-15 ms interval (max 3 x).
- Host: `tools/swifteeg_link.py` (bleak on its own asyncio thread; pyserial
  for USB; FrameParser), `tools/swifteeg_app.py` (Tk UI, host filter chain,
  recording to CSV, rate-change state machine), `tools/eeg_dsp.py` (host chain),
  reference models `proto_ref.py`, `dsp_ref.py`, `pipeline_ref.py`,
  `gen_golden.py` -> `tests/*/golden_*.h` (ztest suites run on target).

## 4. Hardware facts that shape the code

- ADS1299 `START` pin floats (driven by SPI opcode; boot self-test exists).
  RESET/PWDN pulled up (no hardware reset/power-down). CLKSEL=internal osc:
  the sample clock is asynchronous to the MCU (drift measured, ~+0.13 %).
- Full scale +/-187.5 mV at gain 24; LSB 22.35 nV. Electrode offsets of
  30-190 mV seen on dry electrodes; one channel railed in the last session.
- SPIM3 16 MHz for the AFE (PPI-triggerable, DMA); IMU on SPIM2 8 MHz.
- Only IMU INT2 is wired. No battery sense (VDD rail only). No user button.
- Montage: SRB1 referential, reference on LEFT mastoid, bias on RIGHT.
- Electrodes are dry spring-steel wires.

## 5. Already verified on hardware (do not re-report as unverified)

Noise floor 130-149 nV RMS shorted (datasheet ~140). 0 CRC failures. Golden
vectors within 3.4 nV of Python reference. Device chain output bit-exact vs
reference at 250/500/1000 SPS over BLE. 30-min soak at 1 kSPS over BLE (M4).
252 commands while streaming, 0 unanswered, 37 ms median reply. IMU 240/480/
960 Hz with EEG: 0 gaps, sample times within 1 us of a line. 1000 SPS sample
loss (DSP thread blocked in bt_gatt_notify) fixed by the TX queue + thread and
the connection-interval re-ask: six-case matrix 0 lost, full delivery.

## 6. Known and already fixed (README "Mistakes worth keeping" / "Found along
the way") — only report if the fix is wrong, incomplete, or regressed

Verbose USB logging cost; slow work on the BT thread; PPI task slot attached
once; BT RX workqueue stack overflow; multiple threads writing USB; register
access mid-stream slipped an old sample; register access could wedge SPI;
rate change reset the AFE (gains/inputs lost); replies lost around stream
start/stop; acquisition waited for the radio; central moving the interval to
45 ms; notch could not be switched off; filter changes raced the DSP thread;
device uV ignored channel gains; DC corner moved with rate; notch Q 30;
restarted stage started from rest instead of primed.

## 7. Known open items (context, not findings)

No battery sense. If the AFE fails to come up at boot (e.g. low battery),
acquisition is skipped AND command_init() never runs -> device connects but
answers nothing. No impedance measurement (lead-off only). No motion-artifact
cleanup yet. No SD card. ERP stimulus-marker method undecided (software
marker vs trigger wire). Three commits unpushed.

## 8. RULES (hard)

- **READ-ONLY on the repo.** Do not edit, create, move or delete anything
  under `RTOS\`. No git commands that change state (no checkout, stash,
  commit, reset). `git log/show/diff/blame` are fine.
- **No builds, no flashing, no Bluetooth/USB connections, no RTT.** Several
  reviewers run at once; a build would collide in `build/` and hardware may
  be in use.
- You MAY run read-only host self-tests (`python tools/proto_ref.py`,
  `python tools/eeg_dsp.py`, `python tools/dsp_ref.py`,
  `python tools/pipeline_ref.py`) and small throwaway Python/numpy scripts
  that write only inside your own folder under the review directory below.
  scipy is NOT installed; numpy is.
- Evidence over opinion: every finding cites file:line and quotes the code.
  Check SDK semantics in the SDK sources above, not from memory. If you
  cannot confirm something by reading, mark it `speculative`.
- No style nits, no "consider adding comments", no generic best-practice
  lists. Only things with a concrete consequence.
- Do not re-report section 5/6 items unless you show the fix is broken.

## 8b. WORK EFFICIENTLY (the last run was cut off by a usage limit)

- Every tool call re-sends your whole context. Read each owned file ONCE, in
  full or in large chunks; do not re-read, do not page through with many tiny
  reads. Use grep to locate things in files you do not own, then read only
  those ranges. SDK sources: grep for the one function you need, read ~60
  lines around it.
- Do NOT read the whole README. Read only the sections your task names, by
  these line ranges (README.md at HEAD f135b9a):
  0 status 33-99 | 1.2 board quirks 133-168 | 1.3 montage 169-196 |
  2 repo layout 197-249 | 3 flash map 250-266 | M4 + responsive 415-463 |
  Mistakes worth keeping 464-575 | M5 app 576-703 (config on connect 598,
  host chain 615, low drift cut 635, settings apply 651, rate change 676,
  plot pacing 695) | Motion 704-858 (sample time 726, rates 766, on the wire
  795, first session on a head 819) | M6 859-1059 (chain on device 875, SVF
  897, mains tracker 925, on the wire 964, verified 996, found along the way
  1036) | M7/SD 1060-1068 | 7 DSP principle 1069-end.

## 8c. SAVE AS YOU GO (hard)

- Create your report file as your FIRST write, with the header and an empty
  `## Findings` section, before reading any source.
- Append each finding to the file the moment you have confirmed it. Never
  hold findings back for the end. An interruption must lose at most one
  finding.
- Keep a `## Progress` line at the top of the file listing the files you have
  finished reading, updated every few files.
- If your report file already exists with content (an interrupted earlier
  run), read it first and continue from where it stopped.

## 9. Output

Write your full report to
`C:\Users\shrey\AppData\Local\Temp\claude\D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware\005c4da0-af1b-4f26-97b9-4ee4c4c7cdce\scratchpad\review\<YOUR-ID>.md`
(your ID is in your task). Structure:

```
# <ID> — <area>
## Findings
### <ID>-01 [critical|high|medium|low] <one-line title>
- Category: bug | race | robustness | performance | memory | architecture |
  security | contract-drift | docs-drift | test-gap
- Where: path:line[-line] (and any other sites)
- Evidence: the exact code lines (short quotes)
- Failure scenario: concrete state/inputs -> wrong result
- Confidence: confirmed-by-reading | likely | speculative
- Fix sketch: 1-3 lines
(... at most 15 findings, most severe first ...)
## Checked and correct
- short bullets of what you verified is right (so coverage is known)
## Multiplatform / shared-core implications for this area
- short bullets
```

Severity: **critical** = silent data corruption/loss, crash, hang or lockup in
normal use, or a design flaw that blocks the multiplatform goal. **high** =
wrong behaviour under realistic conditions (reconnect, disconnect mid-stream,
rate change, malformed input from any BLE central, long runs), timing errors,
leaks, hard performance ceilings. **medium** = real edge cases and robustness
gaps with a concrete consequence. **low** = minor but real.

Your final reply (the message you return) must be SHORT: at most 300 words —
counts by severity and one line per finding (ID, severity, title). The detail
lives in your file.
