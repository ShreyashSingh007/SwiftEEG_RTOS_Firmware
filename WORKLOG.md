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
- **Review:** COMPLETE (2026-09-17). 58 findings, every critical and high one
  verified against the code. Verdict, fix plan and the owner's decisions are
  in *Verdict and path* below; evidence in `docs/review/2026-09-17/`.
- **Next:** the owner picks from *Open decisions (owner)*. Nothing is fixed
  until then.
- **Parked:** `feature/bcg-vitals` - heart rate, breathing and HRV from head
  motion. Works, not on the plan.

## Branches

| Branch | Holds | State |
|---|---|---|
| `m1-bringup` | M1-M6 milestone line | 3 commits ahead of the `SwiftEEG_RTOS` remote; the push has to be run by the owner |
| `feature/bcg-vitals` | BCG vitals, `10ec2ab`, branched from `6a094b6` | parked |

## Open decisions (owner)

**From the review (see *Verdict and path*):**

A. Fix the data-integrity bugs (phase A) before any more recordings?
   Recommended: yes.
B. Shared core language: the firmware's own C for the codec and DSP, plus a
   Rust session layer with generated Swift/Kotlin/Python bindings
   (recommended), or all C.
C. Apple at 1 kSPS cannot carry raw + filtered side by side. Stream raw only
   and have the apps replay the device's exact filters (recommended), or keep
   the device-filtered stream and accept lower rates on Apple.
D. Protocol v2, pairing/bonding and MCUboot in ONE firmware release, so each
   board needs a single SWD reflash. Recommended: yes.

**Carried over:**


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

### Firmware <-> host contract (R5-CONTRACT, code-reviewer)

Byte layouts otherwise match exactly: frame header, CRC, every opcode and
status value, DATA and IMU headers, all four sample encodings, the full
GET_CONFIG reply and the SET_FILTER upload.

| ID | Sev | V | Finding |
|---|---|---|---|
| 01 | **critical** | V | No version or capability negotiation. GET_INFO carries channels, a hard-coded 0 and the rate - no firmware version, protocol version, device identity or feature list - and both sides silently drop any frame whose version byte differs. Nothing breaks today; the first protocol change makes old apps and new firmware mutually deaf. |
| 02 | medium | V | A reply payload over 46 bytes is dropped entirely and still answered OK. GET_CONFIG is 9 bytes under that limit. |
| 03 | medium | | GET_CONFIG readback drops each channel's power-down and SRB2 bits on the host side (only gain and input are decoded). |
| 04 | medium | | The host library has no generic reply-status decoder: bad-argument, failed and unknown-command replies look like garbled ones. |
| 05 | low | | The test-signal frequency enum has no host-side definition. |
| 06 | low | | A command with an empty payload gets no reply at all instead of bad-argument. |
| 07 | low | | Sample flags are defined twice (pipeline.h, proto.h) with equal values but no compile-time check tying them. |

### Architecture and the multiplatform path (R6-ARCH, architect-review)

| ID | Sev | V | Finding |
|---|---|---|---|
| 01 | high | V | Host logic has no UI-independent home: the filter design exists in both `eeg_dsp.py` (app) and `dsp_ref.py` (oracle), the chain in `eeg_dsp.py` and `pipeline_ref.py`, the LSB in three files. The device is tested against one copy while the app designs with another. |
| 02 | high | V | DATA frames are not self-describing: the header is time, seq, channels, encoding, count - no rate, no configuration generation, no per-sample flags, though the plan specified them. |
| 03 | high | V | No identity, firmware version, capabilities, boot id or link state on the wire (GET_INFO is 4 bytes; no Device Information Service). Apple apps cannot even tell two headsets apart. |
| 04 | high | | Raw + filtered side by side cannot fit Apple at 1 kSPS (64.7 kB/s; ~10 notifications a connection event at 30 ms). |
| 05 | medium | | Sharing C source is not enough for bit-identical output: compilers may fuse float operations and each platform's maths library rounds differently. Needs explicit float flags and a vendored libm. |
| 06 | medium | | The protocol has no single definition and has already drifted (event characteristic, gain code vs value, planned vs built header). |
| 07 | medium | V | No host time-sync or marker command, though the plan committed to one: stimulus timing can only use notification arrival, off by up to a connection interval. |

## Verdict and path

**Can today's foundation carry native apps on Windows, macOS, iOS and Android,
sharing one native core with no bottleneck? Not as it stands - yes after a
protocol v2 and a core extraction.** The base is right: DRDY-latched timing,
one codec over BLE and USB, bit-exact on-device DSP, and codec/DSP/chain C
(`proto.c`, `dsp.c`, `mains.c`, `chain.c`) that already compiles without
Zephyr. What is not ready is the wire contract four apps would freeze, and the
host logic, which lives inside a Tk script. An app built before both are fixed
gets rewritten.

**Bluetooth budget** (details and assumptions: R6-ARCH.md section 2). Windows
and Android carry every mode today except 1000 SPS raw+uV, which is marginal
(measured OK on Windows). Apple today carries only raw 24-bit, up to 500 SPS
with motion. Apple at 1000 SPS + motion needs MTU-sized frames, an interval
request Apple accepts, raw-only streaming, and lossless packing for headroom.
The Apple figures rest on assumed packets per connection event: measure a real
iPhone before freezing v2.

**Target architecture** (R6-ARCH.md section 3):
- **L0 `lib/swifteeg` - C17**, no heap/I/O/threads, compiled from the SAME
  files into the firmware and every host: codec + schema, DSP sections, chain,
  mains tracker, filter design (moved out of numpy), timing maths, recording
  encoder.
- **L1 `swifteeg-core` - Rust**, no I/O of its own, C ABI: request matching,
  device mirror, connect / rate change / reconnect, loss accounting, clock
  sync, replay chain, recorder, LSL. Bindings generated (UniFFI) for Swift,
  Kotlin and Python; a C header for Windows.
- **Per platform:** BLE/USB I/O (CoreBluetooth, BluetoothGatt, WinRT),
  pairing UI, permissions, storage, UI.
- **Conformance:** golden vectors become shared data files replayed by every
  binding's tests.

**Protocol v2** (13 changes, R6-ARCH.md section 4), from one schema file that
generates the C header, Python constants and README tables: versioned GET_INFO
with identity and capabilities; replies echo the command seq and heavy
commands report completion by event; frames sized to the MTU; seq counts
conversions so every loss is a gap; config generation, rate and per-sample
flags in DATA/IMU; delays and stats readable; every event on the event
characteristic; time sync and markers; raw 24-bit for production, lossless
packing capability-gated; stream stops on disconnect; USB flush + hello.

**Security baseline before anyone else wears it:** pairing with LE Secure
Connections inside a pairing window, encrypted writes, per-link rate caps,
watchdog, codec fix and range checks; MCUboot + SMP firmware update; lock SWD
only after updates work.

### Proposed fix order

**Phase A - data integrity, current protocol, then resume recordings.**
Small, testable, no protocol change:
- R3-DSP-01 codec payload overwrite (+ distinct-frames test); R1-ACQ-01
  timestamp wrap (+ pre-wrap test).
- R1-ACQ-07/08 register access all-or-nothing, start-path buffer order.
- R3-DSP-02/07/09, R5-CONTRACT-02/06: validate before OK, range checks,
  failed readback, reply size, empty payload.
- R3-DSP-03 + R1-ACQ-09: rate cap per link, DSP thread yields when behind,
  watchdog, reset on fatal with the cause reported.
- R3-DSP-04/06/10/12, R1-ACQ-11: re-prime on input change, section bounds,
  settling formula, table CRC, anomaly-198 workaround + readback.
- App: R4-HOST-01..07 - parser hardening, recording sidecar with full
  metadata and a new segment on any config change, loss column from
  timestamp continuity, per-frame error isolation, link-lost state, rate
  confirmation, BLE close ordering.
- Then the original plan resumes: still recording, motion recordings, motion
  cleanup.

**Phase B - shared core groundwork, no device change** (can run beside the
motion work): extract `lib/swifteeg/` with a host test runner and explicit
float flags; write the v1 schema + generator; start the core behind
`swifteeg_link.Link`, switched over only when its output matches today's on
captured byte streams.

**Phase C - one firmware release, one SWD pass per board:** protocol v2,
MTU-adaptive batching, Apple-compliant link parameters and PPCP, USB/BLE
transmit decoupling and USB flush, pairing/bonding, MCUboot + SMP. Measure an
iPhone first.

**Phase D - native apps:** macOS + iOS first (hardest radio budget, one
CoreBluetooth adapter), then Android, then native Windows; Tk retired last.

**Deferred unless rates above 1 kSPS are needed:** R1-ACQ-02/10 (acquisition
ceilings), R3-DSP-05 (float32 high-pass at high rates).

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

### 2026-09-17 - review complete

- Wave 2 done: R5 contract (7 findings, 2 verified) and R6 architecture
  (7 findings, 4 verified; its claims about duplicated logic, the DATA header,
  missing sync/marker commands and Zephyr-free shared C all checked).
- Synthesis written: *Verdict and path*, proposed phases A-D, four decisions
  for the owner. Reports R5 and R6 added to `docs/review/2026-09-17/`.
- Totals: 58 findings - 2 critical, 13 high, 27 medium, 16 low.

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
