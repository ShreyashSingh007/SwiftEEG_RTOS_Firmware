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
- **Review status:** wave 1 of 2 running (see the 2026-09-17 log entry).
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

## Log

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
