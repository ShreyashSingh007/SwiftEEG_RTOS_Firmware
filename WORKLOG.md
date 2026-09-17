# SwiftEEG work log

The running record: what was done, why, where it lives in git, and what is
waiting on whom. Work can stop at any point and pick up from the top of this
file without re-deriving anything. Newest entry first.

`README.md` stays the reference documentation - hardware, design, what was
measured and why. This file is the journal. (The plan's "one .md file" rule
was relaxed by the owner on 2026-09-16 for exactly this.)

---

## Resume here

- **Work on:** `m1-bringup` - the milestone line.
- **Review status:** wave 1 done and verified (44 findings, below). Wave 2 (architecture / multiplatform, protocol contract) running. Full reports with file:line evidence: `docs/review/2026-09-17/`.
- **In progress:** deep review of the firmware, the desktop app and the link
  between them, plus whether the foundation supports native apps on Windows,
  macOS, iOS and Android with shared native libraries. Verified findings land
  in the log below; nothing is fixed until the owner picks what to fix.
- **Parked:** `feature/bcg-vitals` - heart rate, breathing and HRV from head
  motion. Works, not on the plan.
- **Waiting on the owner:** see *Open decisions*.

## Branches

| Branch | Holds | State |
|---|---|---|
| `m1-bringup` | M1-M6 milestone line | 3 commits ahead of the `SwiftEEG_RTOS` remote; the push has to be run by the owner |
| `feature/bcg-vitals` | BCG vitals, `10ec2ab`, branched from `6a094b6` | parked |

## Open decisions (owner)

1. **Re-seat CH8, CH2, CH4** before the next recording. CH8 sat on the rail
   (187.5 mV, full scale at gain 24) and carried nothing; CH2 and CH4 were
   close and picked up the most mains.
2. **A still recording**, 2-3 minutes, no yawning - to settle the rate-change
   restart transient and the settings transients.
3. **Motion recordings**, then the motion-artifact cleanup. Approach already
   approved: resample only the reference copy used as a hint; EEG samples are
   never touched and recordings stay raw.
4. **ERP stimulus markers:** a software marker from the PC, or a trigger wire
   into a spare pin. Undecided.
5. **Push** the three commits on `m1-bringup` (blocked on this side).
6. Battery: nothing to be done unless asked. The 3.3 V rail reading could be
   exposed over the protocol if wanted.
7. Electrode impedance meter: parked until the input network values can be
   read from the schematic.

---

## Review findings (2026-09-17)

Severity as the reviewer set it. **V** = re-checked against the code by the
lead after the review (all critical and high findings, plus the mediums that
touch data integrity). Evidence, failure scenarios and fix sketches for every
row are in `docs/review/2026-09-17/<prefix>.md`. Nothing is fixed yet.

### Acquisition and timing core (R1-ACQ, Embedded Firmware Engineer)

| ID | Sev | V | Finding |
|---|---|---|---|
| 01 | high | V | Sample timestamp jumps 71.6 min ahead if TIMER1 wraps between DRDY and the end of that sample's SPI transfer (a few % of wraps; one wrap every 71.6 min). |
| 02 | high | V | A late SPI-end interrupt silently merges or duplicates samples; the overrun counter exists but nothing ever increments it. Mostly a ceiling above 1 kSPS. |
| 03 | medium | V | OVERRUN flag lands on the oldest queued sample, up to 127 samples before the real hole. |
| 04 | medium | V | A frame with a bad status byte vanishes with no flag and no sequence gap. |
| 05 | medium | | Timestamps mark DRDY, not the sampling instant: the ADS1299's own ~1.5-period filter delay (~6 ms at 250 SPS, ~1.5 ms at 1 kSPS) is neither corrected nor reported. |
| 06 | medium | | Connected but useless, silently: an AFE that fails at boot means commands are never handled; a failed restart after SET_RATE still reports the new rate. |
| 07 | medium | V | A failed register access mid-stream leaves acquisition switched off until some later command happens to succeed. |
| 08 | medium | V | First sample after every stream start or rate change is stale (previous session) or zero. |
| 09 | medium | | No watchdog; a fatal error halts forever while the radio may keep the link up. |
| 10 | medium | | Ceilings well below the 16 kSPS plan: 8 ms raw ring at 16 kSPS behind cooperative work, per-sample kernel round trips, blocking RTT logging. |
| 11 | medium | | Direct SPIM3 driver skips Nordic's anomaly-198 workaround; channel and lead-off writes are never read back. |
| 12 | low | | START-pin self-test cannot reliably detect a free-running AFE. |
| 13 | low | | A timebase that fails to start leaves main spinning forever. |
| 14 | low | | MCUboot is not integrated: no firmware update without SWD; the partition table is decorative. |

### IMU, Bluetooth, USB, transmit queue (R2-LINK, Embedded Firmware Engineer)

| ID | Sev | V | Finding |
|---|---|---|---|
| 01 | **critical** | V | Frames never adapt to the negotiated MTU. At MTU 185 (iOS/macOS) the microvolt and raw+microvolt streams send nothing over BLE, silently; a central left at MTU 23 gets no data at all. |
| 02 | high | V | The 7.5-15 ms interval request breaks Apple's rules (min >= 15 ms, max >= min + 15 ms); the 3 re-asks are spent on rejections (the stack reports failed updates too); the first request goes out 5 s after connect. |
| 03 | high | V | Preferred parameters published to every central are 30-50 ms with a 420 ms supervision timeout, and the stack requests them itself. |
| 04 | high | V | One transmit thread feeds USB and BLE, so a slow BLE link stalls and drops the USB stream. Only matters with both links active. |
| 05 | medium | | USB keeps stale frames across host sessions (DTR latched after unplug, no flush on reopen). |
| 06 | medium | | Replies overtake queued data frames: a host can take old-rate data for new-rate data. |
| 07 | medium | | No pairing, bonding or encryption: any central in range reads EEG and reconfigures; with one connection slot it can lock the owner out. |
| 08 | low | | Reply retry loop cannot work as its comment says; a stalled link blocks the command thread. |
| 09 | low | | Mains events travel on the Stream characteristic, against the documented contract. |
| 10 | low | | A failed IMU configuration is reported as applied. |
| 11 | low | | TX thread stack unmeasured on the disconnect-under-load path (speculative). |

### DSP, protocol codec, command handling (R3-DSP, c-pro)

| ID | Sev | V | Finding |
|---|---|---|---|
| 01 | high | V | After a resync the command decoder shifts its buffer before the caller reads the payload: the device can execute a different command, with arguments nobody sent (reproduced: a SET_CAR ran as SET_CHANNEL). |
| 02 | medium | V | SET_RATE replies OK before the rate is validated or applied. |
| 03 | high | V | Any central can select 2-16 kSPS; the DSP thread saturates and starves the command and transmit threads; only a power cycle recovers. |
| 04 | medium | | Switching a channel's input (electrode / shorted / test) does not re-prime DC removal or flag settling: a ~10 s exponential, spread to all channels by the average. |
| 05 | medium | | float32 high-pass error grows with drift and rate: ~0.7 uV RMS at 1 kSPS on 30 mV of drift, ~9 uV at 16 kSPS (reproduced). |
| 06 | low | | Section check accepts huge finite values; one non-finite channel spreads NaN through the average and survives retunes. |
| 07 | low | | SET_CHANNEL gain and input bytes are masked, not range-checked; a reserved gain code reaches the AFE. |
| 08 | low | | Group delay is not reported, and the only implementation is wrong at the low frequencies ERP work needs (reproduced). |
| 09 | low | V | GET_CONFIG answers OK with all-zero channel settings when the register read fails. |
| 10 | low | | Settling flag too short for the notch at 250 SPS: 94 samples flagged, rings for ~143 (reproduced). |
| 11 | low | | Tests miss all of the above: no command-handler tests, no payload check after a resync, chain goldens at 250 SPS only. |
| 12 | medium | | Bitwise CRC-16 on the DSP thread, estimated 60-80 us per sample in raw+uV mode (not measured). |

### Windows app and link layer (R4-HOST, python-pro) - partial: host DSP correctness and performance not reviewed

| ID | Sev | V | Finding |
|---|---|---|---|
| 01 | medium | V | Frame parser trusts the length field: one false SOF can swallow up to 65 KB (measured losses when opening USB on a streaming device). |
| 02 | high | V | Recording header is written once: gain, rate or input changes mid-recording silently mis-describe the raw data; per-channel gains, input mode, versions and filters are never recorded. |
| 03 | high | V | Acquisition losses are invisible in recordings: seq stays continuous, the CSV drops the flags, timestamps are never checked. |
| 04 | medium | | BLE close drops the queued STREAM_STOP (device keeps streaming); close is ignored during a scan (simulated: 0 of 100 delivered). |
| 05 | medium | | One exception in the pump throws away every drained batch, repeatedly; the error goes only to stderr. |
| 06 | medium | | A dropped link shows green status, no reconnect, recording stays open. |
| 07 | medium | V | App times and filters by a rate it never confirmed: one GET_CONFIG with no retry, rates above 1 kSPS ignored, the give-up path resumes anyway. |

### Corrections made during verification

- **R3-DSP-03:** the Bluetooth host threads are cooperative and keep running;
  it is the command and transmit threads that starve. The DSP cost measured
  on 2026-09-16 (171-209 us per sample at 1 kSPS, with the sink) supports
  saturation at 8-16 kSPS.
- **R1-ACQ-08:** the stray START byte left on DIN is harmless - in
  continuous-read mode the part honours only SDATAC (the firmware's own
  measured comment). The stale first frame is real.
- **R2-LINK-04:** real, but only bites when USB and Bluetooth stream at once.

---

## Log

### 2026-09-17 - wave 1 results verified

- Usage limit hit again during wave 1, but save-as-you-go kept 44 findings.
  All critical and high ones, and the mediums touching data integrity, were
  re-checked against the code; three corrections above.
- Wave 2 relaunched lean: architecture + multiplatform path (R6,
  architect-review, which also rolls up performance and security from wave 1)
  and the byte-level protocol contract (R5, code-reviewer on Sonnet).
  Verification done by the lead instead of more agents, to stay inside the
  usage window.

### 2026-09-17 - review relaunched

- The first review run was lost: the usage limit cut every reviewer off
  before any report was written. The two runs of `arm-cortex-expert` never
  worked at all - its plugin definition has `tools: []`, so it cannot read a
  file, and one run printed several hundred fake tool calls as plain text,
  which likely ate a large share of the usage. Fix on the owner's side:
  delete that line in
  `~/.claude/plugins/cache/claude-code-workflows/arm-cortex-microcontrollers/1.2.1/agents/arm-cortex-expert.md`.
  Firmware reviews use **Embedded Firmware Engineer** (nRF Connect SDK / Zephyr).
- Relaunched in two waves of four instead of eight at once. Every reviewer
  now writes its report file first and appends each finding the moment it is
  confirmed, so an interruption loses at most one finding. Reports live in
  the session scratchpad `review/` folder next to the shared `BRIEF.md`.
  - Wave 1 (running): R1 acquisition, R2 IMU + transports, R3 DSP / codec /
    commands, R4 desktop app.
  - Wave 2 (next): R5 contract, R6 architecture, R7 performance, R8 security.

### 2026-09-16 - review before resuming the plan

- Moved the BCG vitals work off the milestone line onto `feature/bcg-vitals`
  (`10ec2ab`). `m1-bringup` is back at `6a094b6` with none of it.
- Kept the eyes-open / eyes-closed quality recording in
  `recordings/quality_2026-09-16/` (git-ignored, as agreed for recordings).
- Signal quality from that recording, for reference: amplifier-class noise
  (0.11-0.14 uV/rtHz, about 1.2 uVrms over 0.5-100 Hz, electrode-limited);
  notch -44 to -51 dB; alpha 9.5 Hz peak 13.7 dB over background, 1.56x eyes
  closed - weaker than a wet clinical setup because of contact, not the
  amplifier. On the good channels roughly half the 1-4 Hz band was the
  heartbeat (measured on the parked branch).
- Deep review started. Pipeline:

  | Stage | Agent | Area |
  |---|---|---|
  | review | arm-cortex-expert | acquisition core: ADS1299 driver, capture, ring, pipeline, timebase, boot |
  | review | arm-cortex-expert | IMU driver and the BLE / USB transports |
  | review | c-pro | on-device DSP, mains tracker, protocol codec, command handling, tests |
  | review | python-pro | desktop app, link layer, host DSP |
  | review | code-reviewer | firmware <-> host contract, reference-model parity |
  | review | architect-review | architecture and the shared-core multiplatform path |
  | review | performance-engineer | CPU, memory and radio budgets, bottlenecks per platform |
  | review | firmware-analyst | security of the radio and command surface, DFU |
  | verify | independent specialists | every critical and high finding re-checked against the code |
  | synthesis | lead | this log: validated findings, fix plan, path review |
