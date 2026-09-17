# SwiftEEG - handoff

Written 2026-09-17 (afternoon, IST) so another engineer or AI assistant
(e.g. opencode) can pick this up with nothing else. It links to the existing
documents instead of repeating them. **If this file and `WORKLOG.md` or git
disagree, `WORKLOG.md` and git are newer and win.**

---

## 0. Read order (about 20 minutes)

1. **This file** - what the project is, where it stands, what is running, what
   to do next.
2. **[WORKLOG.md](WORKLOG.md)** - the running journal: *Resume here*, branch
   table, *Open decisions (owner)*, all 58 review findings with a verified
   column, *Verdict and path*, *Proposed fix order* (phases A-D), dated log.
3. **[README.md](README.md)** - the reference: hardware, build/flash, every
   milestone, measured results, and *Mistakes worth keeping* (section map in
   §10 below).
4. **[docs/review/2026-09-17/](docs/review/2026-09-17/)** - full evidence
   (file:line, failure scenarios, fix sketches) for every finding.
5. **The original plan** (outside the repo):
   `C:\Users\shrey\.claude\plans\d-electronics-projects-eeg-project-swif-peppy-peach.md`
6. **Phase A live logs** - see §3.3. Snapshot in
   [docs/handoff/phaseA/](docs/handoff/phaseA/).

---

## 1. What SwiftEEG is, and what the owner wants

An 8-channel wireless EEG headset: **nRF52840** (Cortex-M4F) + **TI ADS1299**
(24-bit AFE) + **ST LSM6DSV16X** IMU on the head. Firmware in C on **nRF
Connect SDK v3.4.0 (Zephyr 4.4)** in this repo; a **Windows desktop app in
Python/Tk** (`tools/swifteeg_app.py`) streams, filters, plots and records over
Bluetooth LE or USB.

**The owner's goals, in their words and decisions:**

- "Really capable, flexible firmware that gives full granular control over my
  hardware wirelessly, and clean, state-of-the-art EEG signal - the best
  possible from the device."
- **Signal integrity is sacred.** Recordings stay **raw**. ERP analysis with
  deep-learning models is planned, so timing must be trustworthy
  (hardware-latched DRDY timestamps, true sample rate, known delays) and a
  0.1 Hz drift cut must stay usable. IMU samples are never repeated,
  interpolated or averaged to match EEG. A failure must never look like
  success.
- **Most DSP happens on the device** ("consistency is the key"). The host app
  is where DSP is tuned before it moves on-device (milestone M6).
- **Native apps soon on Windows, macOS, iOS and Android**, sharing as much as
  possible - business logic, protocol/integration pipelines, DSP - as shared
  native libraries, with no bottleneck or performance compromise.
- Decisions made 2026-09-17 after the review (details: WORKLOG *Open decisions
  (owner)*):
  - **A.** Fix the data-integrity bugs (phase A) before any more recordings.
  - **B.** Shared core = the firmware's own C for codec and DSP (L0) + a
    **Rust** session layer with generated Swift/Kotlin/Python bindings (L1).
  - **C.** Filtering stays **on the device**; each platform/link gets only the
    rates it can carry without loss (e.g. 1000 SPS disabled on links that
    throttle BLE). No host replay as the display path.
  - **D.** Protocol v2 + pairing/bonding + MCUboot/SMP ship in **one**
    firmware release (one SWD reflash per board).

---

## 2. How to work with the owner (important)

- **Short, plain answers.** Lead with the answer; technical depth only when
  asked.
- **Re-surface skipped warnings and unanswered critical questions.** If a
  question affects data quality, architecture, or something that cannot be
  undone and the owner replies "continue" without answering, ask again before
  going further. Ordinary judgement calls you make yourself. The owner has
  said repeated reminders are welcome.
- **When lost, re-orient in three lines:** goal, where we are, what is
  blocked and on whom.
- **Blocked on access/tools/permissions?** Name the blocker and ask. No
  improvised workarounds.
- **Privacy:** code, data and project details are the owner's IP. Never send
  them to outside services unasked.
- **Agents:** the owner likes specialised sub-agents (the wshobson/agents
  catalog is installed) and parallel orchestration - but see §11 on usage
  limits.
- **Keep `WORKLOG.md` current and commit as you go.** Park off-plan features
  on their own branches (like `feature/bcg-vitals`).
- **Hardware etiquette:** ask before any test that needs the headset **worn**
  and wait for a yes. Tell the owner when to power the device **on (not
  worn)** and when to connect the ST-Link. The debugger stops BLE (README
  §5.6).
- **Low battery:** output turns noisy on every channel and stays noisy. At
  ~2.16 V VDD the ADS1299 does not come up at all ("AFE absent") and the
  device connects but answers no commands. Never analyse such data; tell the
  owner to check and recharge. **Do not build a battery warning unless the
  owner asks.**
- **Git push:** the owner pushes. An assistant's push was blocked once.
- Full personal working-style notes live in the local memory folder (§10),
  deliberately not copied into git because the repo syncs to GitHub.

---

## 3. State right now

### 3.1 Git

Repo: `D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS`

| Branch | Head | What |
|---|---|---|
| `m1-bringup` | `830e21a` (+ this handoff commit) | The milestone line. **Work here.** Ahead of the remote by 11 commits before the current phase-A edits; the owner has not pushed yet. |
| `feature/bcg-vitals` | `10ec2ab` | Parked: heart rate, breathing, HRV from head motion (`tools/vitals.py`, app box, README section). Local only. |

Remotes: `SwiftEEG_RTOS` = `https://github.com/ShreyashSingh007/SwiftEEG_RTOS_Firmware.git`
(this repo; remote `m1-bringup` is at `5315c00`). `origin` =
`https://github.com/ShreyashSingh007/SwiftEEG.git` (a different repo).

Unpushed commits on `m1-bringup` (oldest first):

| Commit | Meaning |
|---|---|
| `72f007a` fw | Never wait for the radio on the acquisition path: 32-frame transmit queue + `stream_tx` thread, OVERRUN flag on ring drops, BLE interval re-ask, BT buffer Kconfig. Fixed the silent 1000 SPS sample loss. |
| `82b2b7e` host | The app counts and shows samples the device lost. |
| `6a094b6` docs | README: the silent sample loss, and the first session on a head. |
| `f135b9a` docs | `WORKLOG.md` started; BCG vitals parked on their branch. |
| `4c34102` docs | The lost first review run, relaunch in two waves. |
| `1c30c56` docs | Wave-1 review findings, verified; reports into `docs/review/`. |
| `7faefea` docs | Protocol contract review (R5). |
| `a0229bc` docs | Review complete: verdict, multiplatform path, fix phases. |
| `4b764ae` docs | Wording. |
| `830e21a` docs | Owner decisions A-D; phase A starts. |

`receipts/` and `review-receipts/` (untracked) are artifacts of installed
Claude Code plugins, not project files. Leave them uncommitted.

### 3.2 Phase A - in progress, NOT committed

Phase A = the data-integrity fixes chosen in decision A (full list: WORKLOG
*Proposed fix order*). At the time of writing three specialist agents were
working in the **uncommitted working tree**. Run `git status` and
`git diff --stat` to see the real state before doing anything.

| Item | Status when written | Files | Evidence |
|---|---|---|---|
| R3-DSP-10 settling flag uses the warped pole | **done** (not yet built into test suites) | `src/dsp/dsp.c/.h`, `tools/dsp_ref.py`, `tests/dsp/src/main.c` | `docs/handoff/phaseA/C-PRO.md`: never short over 25 921 (g,k) pairs; flagged 1.03-1.9x the simulated ring-down |
| R3-DSP-06 section bounds (validation half) | **done** | same | all 13 080 designed sections accepted; 9 huge-but-finite cases refused |
| R4-HOST-01 parser, -04 BLE close, link side of -05/-06 | **done** | `tools/swifteeg_link.py` | `python tools/swifteeg_link.py` self-test OK; parser losses 338->1 frames; STREAM_STOP written 100/100 (was 0/100); config reply swallowed 0/500 (was ~10 %) |
| R4-HOST-02/03/05/06/07 app side (sync, measured period, loss columns, error isolation, segments + JSON sidecar, link-lost state) | **implemented**; runtime check blocked by missing NumPy | `tools/swifteeg_app.py` | `docs/handoff/phaseA/PYTHON-PRO.md` |
| Firmware 1. R3-DSP-01 codec payload overwrite (+ test) | **done**, builds | `src/proto/proto.c/.h`, `tests/proto` | `docs/handoff/phaseA/FIRMWARE.md` |
| Firmware 2. R3-DSP-12 table CRC | **done**, builds (+512 B) | `src/proto/proto.c` | check value 0x29B1 |
| Firmware 3. R1-ACQ-01 timestamp wrap race (+ test) | **done**, builds | `src/timebase/*`, `src/afe/ads1299.c`, `tests/timebase` | **follow-up:** `src/pipeline/capture.c:224` still calls `timebase_stamp_us()` - same latent issue, not yet fixed |
| Firmware 4. R1-ACQ-07 register access all-or-nothing | **done**, builds | `src/afe/ads1299.c` | |
| Firmware 5. R1-ACQ-08 START before frame buffers | **done**, builds | `src/afe/ads1299.c` | |
| Firmware 6. R1-ACQ-11 anomaly-198 workaround + readback | **done**, builds (+260 B) | `src/afe/ads1299.c` | |
| Firmware 7. Commands: SET_RATE validated first + **cap at 1000 SPS**, `pipeline_rate()` 0 when stopped, gain/mux range checks, GET_CONFIG EFAILED on bad readback, no silent reply drop, empty-payload reply | **done**, builds | `src/transport/command.c/.h`, `src/pipeline/pipeline.c/.h` | FLASH 251 396 B, RAM 101 370 B |
| Firmware 8. R3-DSP-03 DSP thread yields when behind | **done**, builds | `src/pipeline/pipeline.c` | |
| Firmware 9. R1-ACQ-09 watchdog + reset on fatal + reset cause | **done**, builds | `prj.conf`, `src/main.c`, threads | hardware timing still pending |
| Firmware 10. R3-DSP-04 re-prime DC + SETTLING on input/power change | **done**, builds | `pipeline.c`, `chain.c` | reset and DSP recovery include DC settling |
| Firmware 11. R3-DSP-06 chain half (non-finite channel reset + flag) | **done**, builds | `chain.c` | invalid channels are isolated from CAR |
| Firmware 12. Mirror 10-11 in `tools/pipeline_ref.py`; host self-tests; regenerate goldens if outputs change | **done**, builds | `tools/pipeline_ref.py`, `tests/pipeline` | NumPy host self-test still pending |

Nothing of phase A has been flashed or run on hardware yet.

### 3.3 Where the live progress logs are

Agents write these as they go (the session scratchpad is temporary and may be
cleaned up; the snapshot in the repo is from the moment this file was
written):

- Live: `C:\Users\shrey\AppData\Local\Temp\claude\D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware\005c4da0-af1b-4f26-97b9-4ee4c4c7cdce\scratchpad\phaseA\` -
  `FIRMWARE.md`, `PYTHON-PRO.md`, `C-PRO.md`
- Snapshot: [docs/handoff/phaseA/](docs/handoff/phaseA/)

### 3.4 How to finish phase A

1. `git status`, `git diff --stat`; read the three logs (live if present,
   otherwise the snapshot). Trust the diff over the logs.
2. **Firmware items 8-12** (§3.2): specs in WORKLOG *Proposed fix order*,
   evidence and fix sketches in `docs/review/2026-09-17/R1-ACQ.md` and
   `R3-DSP.md`, the plan at the top of `FIRMWARE.md`. Build after every fix:
   `powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1`
   (success prints a `FLASH:` line; fix any warning you introduced).
   Also fix the `capture.c:224` follow-up.
3. **App steps** still open in `PYTHON-PRO.md`: edit `tools/swifteeg_app.py`
   in small hunks (see §11 - a whole-file rewrite hits the output limit).
4. Host checks, from a **fresh shell** (the build shell's Python lacks
   `bleak`): `python tools/proto_ref.py`, `python tools/dsp_ref.py`,
   `python tools/pipeline_ref.py`, `python tools/eeg_dsp.py`,
   `python tools/swifteeg_link.py` - all must print OK.
5. Build the test suites: `tools/build.ps1 -Target proto|dsp|timebase|pipeline`.
6. **Commit in groups** in the repo's style (`fw: ...`, `host: ...`,
   `docs: ...`, plain-English subject, body explaining why): codec + timebase;
   AFE access; commands + rate cap; DSP + chain; watchdog + pipeline; link;
   app.
7. **Hardware verification with the owner** - device ON, NOT worn, ST-Link on
   J4: flash (`powershell -File tools/flash.ps1`; it exits 0 even on failure,
   so read the output for `verified`), run each test suite on target, then
   `python tools/verify.py --quick`, `python tools/verify_chain.py` at 250,
   500 and 1000 SPS (`--rate`), `tools/lab/cmd_reliability_test.py`,
   `tools/lab/rate1000_test.py`, `tools/lab/afe_stress.py`, and watch the RTT
   health line (`python tools/rtt.py --live 20`). Try SET_RATE 2000 (must be
   refused) and a register access mid-stream.
8. Update README (*Mistakes worth keeping* / *Found along the way* /
   *Verified*) and WORKLOG, commit, then resume the plan (§4).

---

## 4. The plan after phase A

Order (full reasoning: WORKLOG *Verdict and path*;
architecture: `docs/review/2026-09-17/R6-ARCH.md`):

1. **Resume the original milestone path** (memory `swifteeg-milestone-order`):
   - Owner re-seats CH8, CH2, CH4 (§5).
   - **Still recording**, 2-3 minutes, no yawning - to settle the rate-change
     restart transient and the settings transients.
   - **Motion recordings** of a moving subject (ask before wearing).
   - **Motion-artifact cleanup on the host.** Approved approach: resample only
     the IMU reference copy used as a regression hint; EEG samples are never
     touched; recordings stay raw.
   - Then the motion half of **M6** (cleanup on the device).
2. **Phase B - shared-core groundwork** (no device change, can run alongside):
   extract `lib/swifteeg/` (proto, dsp, mains, chain, frame.h,
   timebase_math.h - already Zephyr-free) with a host test runner and explicit
   float flags (`-ffp-contract=off`); write the protocol **v1 schema** exactly
   as built plus a generator (C header, Python constants, README tables); start
   the **Rust** core behind `swifteeg_link.Link`, switched over only when its
   output matches today's on captured byte streams.
3. **Phase C - one firmware release, one SWD pass per board:** protocol v2 (13
   changes, R6-ARCH §4: versioned GET_INFO with identity and capabilities,
   replies echo the command seq, MTU-sized frames, seq counts conversions,
   config generation + per-sample flags in DATA/IMU, delays and stats, events
   on the event characteristic, time sync + markers, ...) **with per-link
   rate/encoding capabilities (decision C)**, Apple-compliant connection
   parameters and PPCP, USB/BLE transmit decoupling and USB flush,
   pairing/bonding (LE Secure Connections, pairing window), MCUboot + SMP.
   **Measure a real iPhone/Mac (MTU, interval, packets per event) first** -
   the Apple budget numbers are assumptions.
4. **Phase D - native apps:** macOS + iOS first (hardest radio budget), then
   Android, then native Windows; the Tk app retires last.

Deferred: rates above 1 kSPS (R1-ACQ-02/10, R3-DSP-05), the electrode
impedance meter, SD card (last, may not happen).

---

## 5. Open questions and pending items for the owner

1. **Re-seat CH8, CH2, CH4** before the next recording. CH8 sat on the rail
   (187.5 mV = full scale at gain 24) and carried nothing; CH2 (184 mV) and
   CH4 (155 mV) were close and picked up the most mains.
2. **Still recording** (2-3 min, completely still) - after phase A.
3. **Motion recordings** - after that; ask before wearing.
4. **ERP stimulus markers - undecided since 2026-09-11:** software markers
   from the PC (v2 plans `CMD_MARKER` + time sync) or a hardware trigger wire
   into a spare pin (sub-millisecond). Re-ask when ERP work comes up.
5. **Push** the unpushed commits (owner runs
   `git -C "D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS" push SwiftEEG_RTOS m1-bringup`;
   `feature/bcg-vitals` is local only).
6. **Battery:** nothing unless asked. Option on record: expose the MCU's VDD
   rail reading (`supply_read_vdd_mv`) over the protocol so apps can warn.
7. **Impedance meter:** parked. It needs the input resistor/capacitor values
   from the schematic, which the owner asked not to read for now
   (`D:\Electronics Projects\EEG Project\SwiftEEG\SwiftEEG\SwiftEEG.kicad_sch`).
   The owner has ~100 k and 1 M resistors for an accuracy check.
8. **Broken agent plugin:** `arm-cortex-microcontrollers:arm-cortex-expert` has
   `tools: []` in
   `C:\Users\shrey\.claude\plugins\cache\claude-code-workflows\arm-cortex-microcontrollers\1.2.1\agents\arm-cortex-expert.md`,
   so it cannot read files. Owner's fix: delete that line.
9. **Signal quality to revisit:** alpha reactivity was weak (1.56x eyes closed)
   because of contact; roughly half the 1-4 Hz band on good channels is the
   heartbeat (measured on the parked branch) - cardiac artifact removal is a
   possible later item.

---

## 6. History - what was done

Older detail lives in README §6 (milestones, measured numbers, mistakes) and
the memory files (§10). Commit ranges from `git log`.

| When | What | Where |
|---|---|---|
| Aug 2026 | Plan agreed: Zephyr/NCS firmware, C, DMA acquisition, on-chip DSP, one binary protocol over BLE and USB. Working-style preferences recorded. | plan file; memory |
| M1 | Bring-up: board port, USB + BLE, sensor IDs, LEDs. | `8c5e955`..`a5ffff1` |
| M2 | 64-bit TIMER1 timebase, DRDY timestamped in hardware (2 us jitter), DMA sample fetch, ring buffer + DSP thread, USB streaming, AFE verified to 0.2 %. | `b8433b3`..`6ec814c` |
| M3 | Golden vectors for the whole chain; noise floor 130-149 nV RMS (datasheet ~140). | `2cb192a` |
| 2026-09-09 | Plan revised by the owner: wireless control first, Windows app in scope. | `3ed7ee7`; memory `swifteeg-milestone-order` |
| M4 | BLE streaming, configurable rate, bias, per-channel control, lead-off, packed 24-bit, config readback, 30-min wireless soak. | `92b0543`..`a61a515` |
| M5 | Windows app: scaling, rate changes, mains tracking, redraw at monitor rate, bad electrodes out of the average; SRB1 bias-loop fix. | `a61a515`..`ee8fd7d`, `9587c0c` |
| IMU | LSM6DSV16X streamed on the EEG clock (240/480/960 Hz, sample times within 1 us); FIFO read in one transfer; accel/gyro pairing. | `cb177ff`, `e72084e`, `d1d28b8` |
| M6 (filters) | The app's filter chain runs on the device, bit-exact vs the reference; mains frequency measured on the device and the notch aimed by it. | `e8b901d`, `8fa41ee`, `df25c2a`, `5315c00` |
| 2026-09-15 | App defaults, settings apply at once, rate change waits for the device, AFE settings kept across rate change, filters primed on restart, 20 ms motion batches. | `6561ec1`, `2a7bf36`, `3dc987d`; memory `swifteeg-wip-2026-09-15` |
| 2026-09-16 | **First worn session** (sitting): real EEG, 0 missing of 45 083 samples, mains 36 dB down, IMU true rate 235.6 Hz for nominal 240, motion artifacts sized. Only 54 % of seconds clean (yawns, shoulders) - re-checked with `clean_check.py`. | README *First session on a head*; memory `swifteeg-worn-2026-09-16` |
| 2026-09-16 | **Flat battery found** (VDD 2158 mV, AFE absent, silent device). | memory `swifteeg-low-battery-noise` |
| 2026-09-16 | **Silent 1000 SPS sample loss** (0.53 % charged, 2.79 % flat): DSP thread blocked in `bt_gatt_notify`. A first "refuse to wait" fix cut delivery and was reverted. Final fix: transmit queue + thread, OVERRUN flag, interval re-ask. Verified: six-case matrix 0 lost with full delivery; verify_chain 250/500/1000 pass; 252 commands 0 unanswered (37 ms median); DSP cost 171-209 us/sample at 1000 SPS. | `72f007a`, `82b2b7e`, `6a094b6`; README *Mistakes worth keeping* |
| 2026-09-16 | **Signal-quality benchmark** (1 min eyes open + 1 min closed, device filters on): amplifier-class noise 0.11-0.14 uV/rtHz (~1.2 uVrms, electrode-limited); notch -44 to -51 dB; raw mains 24-165 uV; offsets up to 187.5 mV (CH8 railed); alpha 9.5 Hz, 13.7 dB over background, 1.56x eyes closed (weak: contact). | `recordings/quality_2026-09-16/`; `tools/lab/quality_*.py`; memory `swifteeg-worn-2026-09-16` |
| 2026-09-16 | **Heartbeat seen on accel Y** -> head ballistocardiogram: HR 87.9 bpm from the IMU, 84-86.5 bpm independently from EEG pulse artifact; breathing ~6.5/min (Mayer-wave ambiguity); no blood pressure from the head alone. Live app box on a worker thread. **Parked at the owner's request.** | branch `feature/bcg-vitals` |
| 2026-09-16/17 | **Deep multi-agent review** of firmware, app and integration + multiplatform assessment: 58 findings (2 critical, 13 high, 27 medium, 16 low), all critical/high verified against code. Two runs lost to usage limits (§11). | `docs/review/2026-09-17/`; WORKLOG |
| 2026-09-17 | Owner decisions A-D; **phase A started** (§3.2). | WORKLOG |

Earlier owner instructions still in force: unworn headset noise (especially
SRB1 montage, floating inputs) is expected and not a fault; recordings live in
`recordings/` inside the project and out of git; motion cleanup must use a
proven method that cannot harm the EEG; the montage is SRB1 reference on the
**left** mastoid and bias on the **right** (the app label and README once had
it swapped - check new text); electrodes are dry spring-steel wires.

---

## 7. Environment, build, flash, hardware

- **Machine:** Windows 11. Python 3.14 (`C:\Python314`) with numpy, bleak,
  pyserial. **scipy is not installed.**
- **SDK:** `C:\ncs\v3.4.0` (zephyr, nrf, modules\hal\nordic); toolchain
  `C:\ncs\toolchains\dcbdc366a1`. Board `boards/shreyash/swifteeg`.
- **Build:** `powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1`
  (`-Target app|proto|dsp|timebase|pipeline`, `-Pristine`, `-NoBle`). Never
  build through the nrfutil toolchain launcher (README §4.1). Run host tools
  from a fresh shell afterwards.
- **Flash:** `powershell -File tools/flash.ps1` (default
  `build\zephyr\zephyr.hex`, mass-erases first). Probe: ST-Link V2 on header
  J4 (1 GND, 2 nRESET, 3 SWDIO, 4 SWDCLK); OpenOCD xPack 0.12.0 at
  `D:\swifteeg-tools\xpack-openocd-0.12.0-7\bin\openocd.exe`. Exit code is 0
  even on failure - read the output. README §5 covers HLA, SWD speed, reset.
- **Logs:** RTT only. `python tools/rtt.py` (halts the core, dumps, leaves it
  halted) or `python tools/rtt.py --live 20` (follows a running target; less
  dependable). RTT is unreliable while BLE is active (`tools/build.ps1 -NoBle`
  for USB-only diagnosis). The health line prints every 10 s (samples, drops,
  bad status, DSP us, queue/link drops, VDD).
- **Current size:** app FLASH ~251 KB (24 %), RAM ~101 KB (39 %). MCUboot is
  not integrated yet (the partition table in the DTS is unused).
- **Hardware facts:** README §1. Short version: DRDY -> PPI -> TIMER1 capture +
  SPIM3 DMA start (16 MHz); IMU on SPIM2, only INT2 wired; ADS1299 START pin
  floats, RESET/PWDN pulled up, internal oscillator (~+0.13 % fast); full scale
  +/-187.5 mV at gain 24, LSB 22.35 nV; no battery sense; no user button;
  three boards exist, one has the ADS1299 fitted. Montage (README §1.3): CH1
  C4, CH2 P4, CH3 F4, CH4 Oz, CH5 AFz, CH6 F3, CH7 C3, CH8 P3.
- **Protocol (v1 as built):** frame `SOF | ver | type | flags | len | seq |
  payload | CRC16`; types CMD 0x01, RSP 0x02, EVT 0x03, DATA 0x04, IMU 0x05;
  encodings RAW_I32 0, UV_F32 1, RAW_I24 2, RAW_UV 3; 6 samples a DATA frame
  (3 in RAW_UV); flags SETTLING 0x01, OVERRUN 0x02. Byte-level table:
  `docs/review/2026-09-17/R5-CONTRACT.md` appendix. Opcodes:
  `src/transport/command.h`.

---

## 8. Tools and scripts

### 8.1 In the repo, `tools/`

| Tool | Purpose |
|---|---|
| `build.ps1`, `flash.ps1` | Build and flash (§7). |
| `rtt.py` | Firmware log over RTT. |
| `swifteeg_app.py` | The Windows application. |
| `swifteeg_link.py` | BLE (bleak) and USB (pyserial) links, frame parser, decoders; self-test. |
| `swifteeg_scope.py` | Minimal live scope. |
| `eeg_dsp.py` | Host DSP chain used by the app (state-variable sections, mains FFT tracker). |
| `proto_ref.py`, `dsp_ref.py`, `pipeline_ref.py` | Independent Python reference models of the codec, DSP primitives and whole chain; each has a self-test. |
| `gen_golden.py` | Regenerates golden vectors for the C test suites. |
| `verify.py` | Hardware acceptance (test signal, noise floor, timing). |
| `verify_chain.py` | On-device filter chain vs the reference, over BLE or USB. |
| `soak.py` | Long-run BLE stability. |

### 8.2 Session tools, copied to `tools/lab/`

Made during the sessions to work faster. They are lab scripts, not product
code: most need hardware, several hard-code the old scratchpad or `TOOLS`
paths at the top (adjust before running), and some depend on others in the
same folder.

| Area | Scripts |
|---|---|
| Worn sessions and signal quality | `worn_session.py` (BLE recorder: parts `sit` / `rates` / `settings`, saves npz, live 60-90 Hz noise watch for low battery), `worn_report.py` (analysis + loaders `load`, `eeg_part`, `imu_part`), `clean_check.py` (marks clean seconds: no head motion, no outliers), `quality_test.py` + `quality_report.py` (eyes open/closed benchmark), `head_pulse.py` (EEG pulse artifact vs IMU; needs `tools/vitals.py` from the BCG branch), `measure_head.py`, `spectrum.py` |
| Radio throughput and command reliability | `rate1000_test.py` (six-case loss matrix from hardware timestamps), `cmd_reliability_test.py`, `ble_test.py`, `ble_rate.py`, `ble_controls.py`, `ble_responsive.py`, `diag.py`, `diag_ble.py`, `diag_deep.py`, `diag_rate.py`, `probe_stream.py` |
| AFE and board | `afe_stress.py` (commands during streaming), `rate_keep_test.py`, `dump_regs.py`, `bias_sweep.py`, `verify_bias.py`, `probe_cal.py`, `catch_flash.py` (retry SWD until the chip answers) |
| IMU | `imu_hw_test.py`, `imu_timing_test.py`, `imu_lag_test.py` |
| Mains tracker | `mains_track_study.py`, `mains_track_study2.py` (FLL design), `mains_accuracy.py`, `mains_hw_test.py` |
| DSP studies | `svf_study.py`, `f32_biquad_study.py`, `settle_test.py`, `dsp_demo2.py`, `filter_check.py`, `filter_check2.py` |
| App tests (headless, fake links) | `app_device_test.py`, `app_badch_test.py`, `app_fps_test.py`, `app_imu_test.py`, `app_profile.py`, `tkbench.py`, `link_layout_test.py`, `smoke*.py`, `smoke_app.py`, `smoke_gui.py` |
| Past guarded patches (already applied; examples of the edit style) | `patch_afe.py`, `patch_app.py`, `patch_prime.py`, `patch_rate_imu.py`, `patch_readme.py`, `patch_readme2.py` |
| Review reproductions | `review-2026-09-17/R3-DSP-work/` (`proto_consume_repro.py` - the wrong-command bug; `svf_f32_rates.py`, `gd_f32.py`, `settle_check.py`, `subnormal_check.py`), `review-2026-09-17/R4-HOST-work/` (parser loss tests, BLE close test with a fake `bleak`, connect-reply test) |
| Phase A proofs | `phaseA/c-pro-work/` (`verify_final.py`, `pole_check.py`, settle sweeps), `phaseA/python-pro-work/` (link state tests, fake `bleak` and `serial` packages, `old_link.py` for comparison) |
| Handoff | `handoff_scan.py` - regenerates the fact scan this file was built from |

Not copied: build/RTT logs, a 684 KB text extract of the LSM6DSV16X datasheet
(copyrighted; the PDF is `C:\Users\shrey\Downloads\DS_lsm6dsv16x.pdf`), and a
few small CSV test recordings.

---

## 9. Data and recordings

`recordings/` is git-ignored (owner's choice) and lives only on this machine:

| Folder | Content |
|---|---|
| `recordings/worn_2026-09-16/` | First worn session: `p1_sit.npz` (3 min sitting), `p2_rate500/1000/250.npz` (rate changes), `p3_settings.npz` (settings changes), `session.log`, `device_as_found.json`. |
| `recordings/quality_2026-09-16/quality.npz` | 1 min eyes open + 1 min eyes closed, device filters on (HP 1 Hz, LP 100 Hz, notch 50 Hz Q12 tracking, RAW_UV, IMU 240 Hz). |
| `recordings/rate1000_2026-09-16/` | The loss-matrix captures (`charged`, `fixed`, `txthread`, `final`) behind the 1000 SPS fix. |

npz format (written by `worn_session.py` / `quality_test.py`): `meta` (JSON:
config, commands, changes, phases) plus per-frame arrays `eeg_len`, `eeg_seq`,
`eeg_flags`, `eeg_ts`, `eeg_arrival`, `counts` (int32 raw), `uv` (float32 device
microvolts), `imu_len`, `imu_seq`, `imu_ts`, `imu_period`, `imu_counts`
(int16 x6), `imu_accel_g`, `imu_gyro_dps`, `imu_flags`, `imu_arrival`. Load
with `worn_report.load` / `eeg_part` / `imu_part`.

The app's own CSV recordings: raw counts per channel with `ts_us` and `seq`,
plus a `_motion.csv`; phase A adds `flags`, `missing_before`, segments and a
JSON sidecar.

---

## 10. Documents and files index

**In the repo**

- [README.md](README.md) (1 133 lines) - section map: 0 status 33-99; 1
  hardware 100-196 (1.2 board quirks 133, 1.3 montage 169); 2 repo layout 197;
  3 flash map 250; 4 building 267; 5 flashing and logs 295-396; 6 milestones
  397-1068 (M4 415, *Mistakes worth keeping* 464, M5 app 576, motion 704, first
  session on a head 819, M6 859, *Found along the way* 1036, M7 1060, SD 1065);
  7 DSP design principle 1069.
- [WORKLOG.md](WORKLOG.md) - journal and decisions (§0).
- [HANDOFF.md](HANDOFF.md) - this file.
- [docs/review/2026-09-17/](docs/review/2026-09-17/) - `BRIEF.md` (shared
  review brief: architecture, rules), `R1-ACQ.md` (acquisition/timing),
  `R2-LINK.md` (IMU, BLE, USB), `R3-DSP.md` (DSP, codec, commands),
  `R4-HOST.md` (Windows app), `R5-CONTRACT.md` (byte-level contract +
  appendix table), `R6-ARCH.md` (verdict, Bluetooth budget per platform,
  target architecture, protocol v2, security, migration path).
- [docs/handoff/phaseA/](docs/handoff/phaseA/) - phase A log snapshot.
- [tools/lab/](tools/lab/) - session scripts (§8.2).

**Outside the repo, on this machine**

- Original plan:
  `C:\Users\shrey\.claude\plans\d-electronics-projects-eeg-project-swif-peppy-peach.md`
- Assistant memory (working style, project state across sessions):
  `C:\Users\shrey\.claude\projects\D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware\memory\`
  - `MEMORY.md` index; `swifteeg-review-2026-09-16.md` (latest resume point);
    `swifteeg-worn-2026-09-16.md` (worn sessions, 1000 SPS loss, quality
    benchmark); `swifteeg-wip-2026-09-15.md` (2026-09-15 work: defaults,
    settings lag, rate changes, IMU defaults from the datasheet, mains-tracker
    research with numbers); `swifteeg-milestone-order.md` (milestones,
    montage, parked items); `swifteeg-low-battery-noise.md`;
    `use-specialized-agents.md`; `worklog-and-git.md`; and working-style notes
    (`answer-style-short-simple.md`, `adhd-failsafe-reminders.md`,
    `reorient-when-lost.md`, `ask-for-access-dont-work-around.md`,
    `keep-his-work-private.md`).
- Full conversation transcripts (JSONL, same folder as `memory\`'s parent):
  `005c4da0-af1b-4f26-97b9-4ee4c4c7cdce.jsonl` (this long session, ~35 MB:
  worn sessions through phase A), `553864c7-...jsonl`, `d380114e-...jsonl`,
  `44540ce0-...jsonl`, `bde8bfa5-...jsonl`, `4238bf04-...jsonl` (earlier
  sessions).
- Session scratchpad (temporary):
  `C:\Users\shrey\AppData\Local\Temp\claude\D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware\005c4da0-af1b-4f26-97b9-4ee4c4c7cdce\scratchpad\`
- Schematic (owner asked not to read it for now):
  `D:\Electronics Projects\EEG Project\SwiftEEG\SwiftEEG\SwiftEEG.kicad_sch`
- LSM6DSV16X datasheet: `C:\Users\shrey\Downloads\DS_lsm6dsv16x.pdf`

---

## 11. Lessons and gotchas

**Working with AI tooling on this project**

- **Usage limits bite fast.** Four to eight parallel Opus agents used up the
  owner's 5-hour window in about 30 minutes, three times. Use few agents,
  lighter models for implementation, and make every agent **save as it goes**
  (a log file appended after each step) - that is what saved the second review
  run. Long conversations make every step expensive: start fresh sessions
  often and resume from `WORKLOG.md` and this file.
- **A single response is capped at 64 000 output tokens.** Never rewrite a
  large file in one go; edit in small hunks.
- **Check an agent definition before relying on it:** `arm-cortex-expert` had
  `tools: []` and silently read nothing (it printed fake tool calls as text).
  *Embedded Firmware Engineer* worked well for nRF/Zephyr.
- **Verify agent findings against the code** before acting; three review
  claims needed correcting (WORKLOG *Corrections made during verification*).
- **Shell:** in the Bash tool, heredocs containing apostrophes fail - write
  scripts to files and run them. Python's `write_text` on Windows writes CRLF;
  `.gitattributes` normalises to LF, but prefer `write_bytes` with `\n`.

**Firmware and hardware** - read README *Mistakes worth keeping* first. The
ones that cost the most:

- Verbose USB logging is not free; do not do slow work on the Bluetooth
  thread; the PPI task slot is attached once, not per start; Bluetooth RX
  stack overflow on connect; one writer per USB endpoint.
- Every register access during streaming used to slip in an old sample and
  could wedge the SPI bus; a rate change used to reset the AFE settings.
- `bt_gatt_notify()` blocks when the link is saturated - never call it from
  the acquisition path. Refusing to wait is not the same as not waiting (a
  refuse-and-drop fix cut delivery to 70 %).
- Windows moved the connection interval to 45 ms mid-session, too slow for
  1000 SPS raw+uV; iOS/macOS reject the current 7.5-15 ms request (fix in
  phase C).
- The IMU's own clock runs 1.8 % slow (235.6 Hz for 240): time every sample
  from the measured period, never assume nominal.
- The debugger halts BLE; `flash.ps1` exits 0 on failure; RTT live mode races
  the target.
- Low battery looks like a broken board (§2).
