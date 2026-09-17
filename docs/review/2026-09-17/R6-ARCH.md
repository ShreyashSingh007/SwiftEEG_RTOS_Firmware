# R6-ARCH — Architecture for the shared native core (Windows / macOS / iOS / Android)

## Progress
- Read: BRIEF, R1-ACQ, R2-LINK, R3-DSP, R4-HOST, plan 3-5 + 10, README 33-99 / 197-266 / 859-1068; greps: includes, proto.h, stream.c/imu.c headers, command.c GET_INFO/respond, command.h opcodes, BLE .config, compile flags, tools defs. All six sections done, condensed.

## Findings
New architecture-level findings R6-ARCH-01..07 are under section 1.

## 1. Verdict

**Not as it stands. Yes, after a protocol v2 and a core extraction.** The base is right: DRDY-latched timing, one codec over BLE and USB, and codec/DSP/chain C that already builds on any host (`proto.c`, `dsp.c`, `mains.c`, `chain.c`, `frame.h` include only libc headers), checked by golden vectors. Two things are not ready: the wire contract four apps would freeze, and the host logic, which lives inside a Tk script. An app built before both are fixed will be rewritten.

| # | Blocker, in order | IDs |
|---|---|---|
| 1 | Frames fixed for MTU 247: at Apple's 185, UV_F32 / RAW_UV / RAW_I32 carry nothing, silently | R2-LINK-01 |
| 2 | Contract: DATA not self-describing; seq blind to acquisition loss; per-batch flags; replies unmatched and overtaking data; OK before SET_RATE applies; events on the data path; no identity | R6-ARCH-02/03; R1-ACQ-02/03/04; R2-LINK-05/06/09; R3-DSP-02 |
| 3 | No shared core: contract logic in the UI and three times in Python; the codec meant to be shared corrupts payloads after a resync; the parser trusts lengths | R6-ARCH-01/06; R3-DSP-01; R4-HOST-01 |
| 4 | Link policy blind to platform: interval Apple rejects, PPCP 30-50 ms, USB stalled by BLE | R2-LINK-02/03/04 |
| 5 | Raw+uV mode cannot fit Apple at 1 kSPS; host replay needs guaranteed parity | R6-ARCH-04/05 |
| 6 | Open GATT; any central can set 16 kSPS and starve the device; no watchdog; no MCUboot (adding it later = SWD reflash of every unit) | R2-LINK-07; R3-DSP-03; R1-ACQ-09/14 |
| 7 | Timing inputs: wrap stamp error, AFE delay, group delay, settle length, time sync | R1-ACQ-01/05; R3-DSP-08/10; R6-ARCH-07 |
| 8 | Shared file format and session states: recordings mis-describe changes and hide losses; "link lost", "connected but useless" | R4-HOST-02/03/06/07; R1-ACQ-06/07 |

### R6-ARCH-01 [high] Host contract logic has no UI-independent home and exists three times in Python
- Category: architecture
- Where: tools/swifteeg_app.py (1751 lines: rate change ~636-700, recording 905-910, losses 1026-1039); tools/eeg_dsp.py:61-100, 204; tools/dsp_ref.py:42-85, 232; tools/pipeline_ref.py:64, 122; tools/swifteeg_link.py:112
- Evidence: `butterworth_qs`, `design_notch/_lowpass/_highpass`, `response` are defined in both eeg_dsp.py and dsp_ref.py; `class Chain` in eeg_dsp.py and pipeline_ref.py; `lsb_uv` in three files.
- Failure scenario: four apps copy this again and fix R4-HOST-02/03/06/07 separately. The device is tested against dsp_ref.py while the app designs with eeg_dsp.py, so a design change in the app's copy passes every test.
- Confidence: confirmed-by-reading
- Fix sketch: one core owns codec, session, timing, filter design, chains and recording (section 3); the Python oracles stay only as independent test references.

### R6-ARCH-02 [high] DATA frames are not self-describing, contrary to the plan
- Category: contract-drift
- Where: src/transport/stream.c:55-63; plan 5.4 (plan:366-369); README 678-685; tools/swifteeg_app.py:1112-1119
- Evidence: built header `u64 ts_us, u32 seq, u8 channels, u8 encoding, u16 count`; planned "rate id, channel mask ... flags (SETTLING, OVERRUN, LEADOFF)". No rate or gain appears anywhere in the stream.
- Failure scenario: each app must drop frames until a GET_CONFIG reply and assume nothing older follows it. R2-LINK-06 shows ~30 old-rate frames can follow; R4-HOST-07 shows the existing app already mis-times rows. Four apps are four chances to record the wrong rate or LSB.
- Confidence: confirmed-by-reading
- Fix sketch: add `config_gen`, a rate code and per-sample flags to DATA and IMU headers; whatever applies a change returns its `config_gen`.

### R6-ARCH-03 [high] No identity, version, capabilities, boot id or link state on the wire
- Category: architecture
- Where: src/transport/command.c:343-353; src/proto/proto.h:38; build/zephyr/.config:412
- Evidence: `const uint8_t info[4] = { ADS1299_CHANNELS, 0, (uint8_t)(sps & 0xFFu), (uint8_t)(sps >> 8) };` `PROTO_VERSION 1u`; `# CONFIG_BT_DIS is not set`.
- Failure scenario: CoreBluetooth hides the device address, so Apple apps cannot tell two headsets apart or tag a file with a serial. No app can see a device reset mid-file (R4-HOST-06), tell "no AFE" from "hung" (R1-ACQ-06), or check firmware versions once DFU mixes builds. iOS and Android apps cannot read the connection interval, so only the device can tell them which encoding fits.
- Confidence: confirmed-by-reading (platform API limits: likely)
- Fix sketch: GET_INFO v2 TLVs (section 4, row 1), EVT_LINK, DIS.

### R6-ARCH-04 [high] The raw+uV display mode cannot fit Apple at 1 kSPS; device-chain display needs host replay
- Category: performance
- Where: src/transport/stream.c:37-51; README 865-868, 976-978
- Evidence: RAW_UV is 56 B a sample against 24 for RAW_I24; README: "raw and filtered side by side ... the recording stays raw".
- Failure scenario: 1000 SPS RAW_UV is 64.7 kB/s. At MTU 185 even adaptive frames hold only 2 samples: 500 notifications/s, 7.5 per event at 15 ms (section 2). Sending uV only would break raw recordings.
- Confidence: confirmed (arithmetic; Apple packets per event is an assumption)
- Fix sketch: the production stream is raw only. The core replays the device chain from raw plus apply-seqs, mains EVTs (R2-LINK-09) and `config_gen`, and flags SETTLING after a gap. RAW_UV stays for verify_chain.

### R6-ARCH-05 [medium] Sharing the C source alone does not make device and host output bit-identical
- Category: architecture (parity)
- Where: build/compile_commands.json (`-std=c17 -Os -mfpu=fpv4-sp-d16 -specs=picolibc.specs`, no `-ffp-contract`); src/dsp/dsp.c:239 `out->g = tanf(DSP_PI * fc_hz / fs_hz);`; src/dsp/mains.c:26-27, 48, 119, 123; README 1016-1018, 1033-1034
- Evidence: today "worst 0.0009 uV" at 250 SPS and "within 0.002 uV", both on the test signal with no electrodes.
- Failure scenario: GCC in ISO C17 does not contract, but host compilers may fuse `a*b + c` on AArch64, and `tanf/atan2/exp` differ by ULPs across picolibc, Apple libm, UCRT and bionic. With 30 mV of electrode drift, the float32 0.1 Hz sections already round at 0.7 uV RMS (R3-DSP-05), so replay and cross-app plots can disagree by a fraction of a microvolt on a head.
- Confidence: likely
- Fix sketch: explicit `-ffp-contract=off` (MSVC `/fp:precise`), vendored correctly rounded libm in L0, bit-exact conformance on every target.

### R6-ARCH-06 [medium] The protocol has no single source of truth and has already drifted
- Category: contract-drift
- Where: src/transport/command.h:14-48; stream.c:55-63; imu.c:58-70; tools/swifteeg_link.py:70-266; tools/proto_ref.py; README 964-994, 1062-1063
- Evidence: event-characteristic drift (R2-LINK-09); the gain byte is a code, not a value (R3-DSP-07); planned vs built header (R6-ARCH-02); the README says "any platform can speak it" while MTU 185 disables three encodings.
- Failure scenario: apps hand-copy offsets from comments. One misses a new field and misparses behind a valid CRC.
- Confidence: confirmed-by-reading
- Fix sketch: one schema plus a generator (section 4).

### R6-ARCH-07 [medium] No host time sync or marker path, though the plan locked one
- Category: architecture (timing)
- Where: src/transport/command.h:14-41; plan:113, 233-234
- Evidence: plan: "host sync protocol", "ping/pong ... solves for offset and skew"; no such opcode; no drift estimator anywhere (R1-ACQ).
- Failure scenario: a stimulus app can map device time to its own clock only by notification arrival, which moves by up to one interval (7.5-30 ms) plus queue backlog. ERP latencies then carry a per-platform error larger than the AFE delay in R1-ACQ-05.
- Confidence: confirmed-by-reading (absence); magnitude likely
- Fix sketch: CMD_TIME_SYNC with min-RTT filtering, CMD_MARKER stamped on TIMER1; the core fits offset and drift. A trigger wire stays the sub-ms option.

## 2. Bluetooth budget per platform

**Result:** Windows and Android carry every mode today except 1000 SPS RAW_UV, which is marginal. Apple today carries only RAW_I24, up to 500 SPS with IMU. 1000 SPS + IMU on Apple needs A + B + C below, and D for headroom at 30 ms.

Assumptions:
- Frames (R2-LINK-01) have 26 B overhead. RAW_I24: 6 x 24 B = 170 B; UV_F32: 6 x 32 = 218 B; RAW_UV: 3 x 56 = 194 B. IMU at 240 Hz: batch 5 (imu.c:262), 48 frames/s x 94 B = 4.5 kB/s.
- Notification room = MTU - 3: 244 at 247, 182 at 185. Firmware caps the MTU at 247 (`CONFIG_BT_L2CAP_TX_MTU=247`), so Android's 517 changes nothing.
- 2M + DLE 251: a notification costs 4 us x (value + 17 B) + 340 us; encryption adds 16 us. Airtime peaks at 43 % (1000 SPS RAW_UV + IMU). The real limits are MTU, granted interval and packets per event. Device: event extension on, 10 TX buffers (.config:51, 257, 466).
- Platforms (assumed, not measurable from the repo): Windows 7.5-15 ms, at least 6 packets per event (1000 SPS RAW_UV delivered with 0 missing, README 1021-1023). Android 7.5-15 ms, 4-6. iOS/macOS 15-30 ms, MTU 185, 4-6 (Apple's historic limit is 6); Apple sits at 30 ms today because it refuses the firmware's request (R2-LINK-02). 20 % retransmission margin.

Goodput (80 % of packets x room / interval): Windows 130 kB/s at 7.5 ms (airtime caps it at 5 packets), 78 at 15 ms. Android 104-130 / 52-78. iOS/macOS 39-58 at 15 ms, 19-29 at 30 ms.

Verdict, by notifications per connection event: **fits** = at most 4 at the worst interval; **marg** = 4-6, or at most 4 only at the best; **NO** = more, or frame larger than the room. Cells show EEG / EEG + IMU 240 Hz.

| SPS | Encoding | kB/s | Notif/s | Per event @15 ms | @30 ms | Windows, Android today | Apple today (30 ms, 185) | Apple after A + B |
|---|---|---|---|---|---|---|---|---|
| 250 | RAW_I24 | 7.1 / 11.6 | 42 / 90 | 0.6 / 1.3 | 1.3 / 2.7 | fits / fits | fits / fits | fits / fits |
| 250 | UV_F32 | 9.1 / 13.6 | 42 / 90 | 0.6 / 1.3 | 1.3 / 2.7 | fits / fits | NO (MTU) | fits / fits |
| 250 | RAW_UV | 16.2 / 20.7 | 83 / 131 | 1.3 / 2.0 | 2.5 / 3.9 | fits / fits | NO (MTU) | fits / marg |
| 500 | RAW_I24 | 14.2 / 18.7 | 83 / 131 | 1.3 / 2.0 | 2.5 / 3.9 | fits / fits | fits / fits | fits / fits |
| 500 | UV_F32 | 18.2 / 22.7 | 83 / 131 | 1.3 / 2.0 | 2.5 / 3.9 | fits / fits | NO (MTU) | fits / marg |
| 500 | RAW_UV | 32.3 / 36.8 | 167 / 215 | 2.5 / 3.2 | 5.0 / 6.4 | fits / fits | NO (MTU) | marg / NO |
| 1000 | RAW_I24 | 28.3 / 32.8 | 167 / 215 | 2.5 / 3.2 | 5.0 / 6.4 | fits / fits | marg / NO | marg / marg |
| 1000 | UV_F32 | 36.3 / 40.8 | 167 / 215 | 2.5 / 3.2 | 5.0 / 6.4 | fits / fits | NO (MTU) | marg / NO |
| 1000 | RAW_UV | 64.7 / 69.2 | 333 / 381 | 5.0 / 5.7 | 10.0 / 11.4 | marg / marg (Windows measured OK) | NO (MTU) | NO / NO |

After A, a 185-byte frame holds 6 / 4 / 2 samples, so 1000 SPS needs 167 / 250 / 500 notifications/s.

| Firmware change | Effect |
|---|---|
| A. MTU-adaptive batching (R2-LINK-01) | UV_F32 and RAW_UV work on Apple; RAW_I24 at 247 packs 9 a frame: 167 -> 111 notifications/s at 1 kSPS |
| B. Apple-compliant 15-30 ms request, PPCP fix (R2-LINK-02/03) | halves Apple's per-event load when 15 ms is granted |
| C. Raw-only stream + host replay (R6-ARCH-04/05) | RAW_UV rows become RAW_I24 rows: Apple 1000 SPS + IMU goes NO -> marg |
| D. Lossless packing: delta + bit width per frame, 24-bit fallback (~12 B a sample assumed) | Apple 1000 SPS + IMU: 125 notifications/s, 3.75 per event at 30 ms: fits |
| E. IMU frames filled to the room | IMU 48 -> 14-20 notifications/s, at 50-70 ms display latency |
| F. Rate and encoding capped per link from EVT_LINK (R3-DSP-03) | no app can ask for a NO row |

USB (not on iOS): 16 kSPS RAW_I24 is ~453 kB/s, within full-speed CDC. The device-side ceilings bind first (R1-ACQ-02/10, R3-DSP-03/12).

## 3. Target shared-core architecture

**Recommendation: two layers.** L0 `libswifteeg` is C17 with no heap, I/O or threads, built from the same files into the firmware and every host. L1 `swifteeg-core` is Rust: sans-I/O, C ABI, links L0. The rule: anything that could run on the MCU or must match it bit for bit goes in L0; host lifecycle, I/O and concurrency go in L1.

| Option | Parity with firmware | Host-logic safety | Bindings | Verdict |
|---|---|---|---|---|
| All C | by construction | weakest: R3-DSP-01 and R4-HOST-01/04/05 are bug classes C permits | Swift imports C; JNI, P/Invoke, cffi by hand | fallback |
| C++ around firmware C | by construction | RAII, same undefined behaviour | C ABI and hand-written JNI anyway | no gain |
| **Rust L1 + firmware C as L0** | by construction | ownership covers parsers, state machines, threads | UniFFI for Swift/Kotlin/Python; cbindgen header for Windows | **choose** |
| Rust rewrite of DSP/codec | by tests only | strong | same | reject |

| In the core | Layer | Absorbs |
|---|---|---|
| Codec, table CRC, resync, generated schema | L0 | R3-DSP-01/12, R4-HOST-01, R6-ARCH-06 |
| DATA/IMU decode, DSP sections, chain, mains tracker | L0 | replay (R6-ARCH-04) |
| Filter design, response, analytic group delay, settle (moved out of numpy) | L0, double, vendored libm | R3-DSP-08/10, R6-ARCH-05 |
| Timing maths: 64-bit extension, (seq, ts) fit, true rate, drift, AFE + chain delay | L0 | R1-ACQ-01/05 |
| Recording format encoder (pure; reusable for SD) | L0 | R4-HOST-02 |
| Request ids, echo matching, timeouts, retries | L1 | R2-LINK-05/06, R3-DSP-02 |
| Device mirror keyed by `config_gen`; v1/v2 negotiation | L1 | R6-ARCH-02/03 |
| Connect, rate change, start/stop, reconnect | L1 | R4-HOST-04/06/07, R1-ACQ-06/07 |
| Seq, gap, overrun, settling accounting | L1 | R4-HOST-03, R1-ACQ-02/03/04 |
| Clock map, time sync, markers | L1 | R6-ARCH-07 |
| Display and replay chains; recorder thread (segments, JSON sidecar); LSL via liblsl | L1 | R4-HOST-02/05 |
| **Stays per platform:** BLE/USB I/O (CoreBluetooth; BluetoothGatt + USB host; WinRT GATT + serial), pairing UI, permissions, background modes, storage, UI | app | |

```
 nRF52840 firmware                         host (Win / macOS / iOS / Android)
+--------------------------+   BLE / USB  +----------------------------------------+
| afe imu timebase transp. |<============>| UI: SwiftUI | Compose | WinUI | Tk now   |
| +----------------------+ |              | I/O: CoreBluetooth | Gatt | WinRT      |
| | L0 libswifteeg (C17) | |              +-------- bytes in / out, MTU -----------+
| | codec+schema, dsp,   | |  same files, | L1 swifteeg-core (Rust, C ABI)         |
| | chain, mains, timing | |  same flags  |  requests, mirror, gaps, clock, replay,|
| +----------------------+ |------------->|  recorder, LSL  +-- L0 (C17) --+       |
+--------------------------+              +----------------------------------------+
 conformance/ (Python oracles) -> ztest | cargo test | XCTest | Android | xUnit | pytest
```

**Threads and memory.** One session per device, called only from one serial queue the platform owns; the core starts only a recorder thread. Samples go to preallocated lock-free SPSC rings and events to a bounded queue. Nothing is allocated on the data path, and there is one FFI call per notification (under 550/s), never one per sample.

```
sweeg_session_new(cfg)  sweeg_link_up(s, transport, mtu)  sweeg_link_down(s, why)
sweeg_feed(s, src, bytes, n, host_ns)   sweeg_take_tx(s, buf, cap, &dst)   sweeg_tick(s, host_ns)
sweeg_request(s, &cmd) -> req_id        sweeg_poll_event(s, &ev)  /* reply, state, gap, link, mains, marker */
sweeg_read_eeg(s, &block, max)          sweeg_read_imu(s, &block, max)     /* structure of arrays */
sweeg_chain_set(s, stage, &design)      sweeg_record_start(s, dir, meta)   sweeg_mark(s, label, host_ns)
```

| Target | Package | Binding |
|---|---|---|
| iOS, macOS | XCFramework (ios-arm64, simulator, macOS universal) via SwiftPM | UniFFI Swift + async/AsyncStream wrapper |
| Android | AAR (arm64-v8a, x86_64) | UniFFI Kotlin; one JNI call with a direct ByteBuffer for sample blocks |
| Windows | x64/ARM64 DLL + cbindgen header | P/Invoke (C#) or C++/WinRT |
| Python tools, current app | wheel (maturin) | numpy views |

**Build.** L0 is a CMake library the firmware also compiles, with `-std=c17 -ffp-contract=off` (MSVC `/fp:precise`). L1 is a Cargo workspace whose `build.rs` compiles L0 with the same flags; cargo-ndk and an `xtask` produce the XCFramework, AAR, DLL and wheel. CI runs on Windows, macOS (+ iOS simulator) and Linux (Android emulator, Zephyr `native_sim`).

**Conformance.** Golden vectors become data files in `conformance/`, generated by the independent Python oracles and replayed by every binding's tests: codec edge cases (R3-DSP-01, R4-HOST-01), reply transcripts (R2-LINK-05/06, R3-DSP-02), bit-exact DSP at 250/500/1000 SPS with a gain change, a retune and 30 mV of drift (R3-DSP-05/11), timer wrap and overrun placement (R1-ACQ-01/03), recordings.

## 4. Protocol v2, before four apps depend on it

| # | Change | From |
|---|---|---|
| 1 | Frame `ver` 2. GET_INFO as TLVs (unknown tags ignored): protocol minor, firmware semver + git hash, hardware rev, serial, boot_id, reset cause, AFE/IMU state, capabilities | R6-ARCH-03; R1-ACQ-06/09 |
| 2 | RSP echoes the command's frame seq. Validate before acting. SET_RATE and restarts answer ACCEPTED, then EVT_DONE {request seq, status, config_gen, first sample seq} | R2-LINK-05/06; R3-DSP-02 |
| 3 | Status codes ETOOBIG, EBUSY, ENOTREADY, ELINK (over the link budget) | R2-LINK-08; R3-DSP-03/07 |
| 4 | Frames sized to the negotiated MTU; `count` stays authoritative; EVT_LINK {mtu, interval, phy, data length} | R2-LINK-01 |
| 5 | `seq` = DRDY conversion index assigned in the END ISR, so every loss is a seq gap; OVERRUN marks the first sample after the hole | R1-ACQ-02/03/04; R4-HOST-03 |
| 6 | DATA/IMU header gains `config_gen`, a rate code, and per-sample flags (SETTLING, OVERRUN, LEADOFF) | R6-ARCH-02 |
| 7 | GET_CONFIG adds AFE delay, DC shift and full sections, so the core computes total group delay | R1-ACQ-05; R3-DSP-08 |
| 8 | GET_STATS, plus EVT_STATS at 1 Hz: ring, status, AFE, TX/BLE/USB drops, ble_too_big, DSP max us, stack high-water | R2-LINK-01; R1-ACQ-02/10 |
| 9 | Every EVT on the event characteristic, with its own queue and seq: mains, config applied, stream state, AFE stall, IMU fault, link | R2-LINK-09/10; R1-ACQ-07 |
| 10 | CMD_TIME_SYNC and CMD_MARKER | R6-ARCH-07 |
| 11 | RAW_I24 for production; ENC_PACKED (lossless, capability-gated); RAW_UV for validation only | R6-ARCH-04 |
| 12 | Declared policies: stream stops on disconnect, rate limits per transport, encrypted writes | R4-HOST-04; R3-DSP-03; R2-LINK-07 |
| 13 | USB: flush on the DTR edge, then a HELLO frame to sync on | R2-LINK-05; R4-HOST-01 |

**Single source of truth.** `protocol/swifteeg.yaml` (opcodes, argument and reply layouts, EVT ids, headers, flags, TLV tags, the version of each field) generates: the C header of constants and pack/unpack helpers for the firmware and L0 (Rust reads it through bindgen), constants for the Python oracles (whose decode logic stays independent), and the README tables. CI fails on a stale file. At runtime one codec, L0 `proto.c`, runs everywhere, and apps never parse bytes. A device sent an unknown major version answers EUNKNOWN but still serves GET_INFO, so apps can offer DFU. DATA stays fixed binary; zcbor + CDDL (in NCS) is an option for the TLVs.

## 5. Security and update baseline

| Minimum before anyone else wears it | From |
|---|---|
| `CONFIG_BT_SMP`, LE Secure Connections, bonds in the `storage` partition; Control writes and Stream/Event CCC writes need encryption | R2-LINK-07 |
| Bond only inside a pairing window (e.g. 60 s after power-on, or while no bond exists), then an accept list; bond reset over USB. With no display and no buttons only Just Works is possible: it stops sniffing but not a MITM during pairing, so the window matters | R2-LINK-07 |
| Per-link rate caps, DSP thread yields, task watchdog, reset on fatal error with the cause reported | R3-DSP-03; R1-ACQ-09 |
| Codec fix and argument range checks | R3-DSP-01/06/07 |
| Lock SWD (APPROTECT) only after DFU works | R1-ACQ-14 |

| Platform | Connect flow with bonding |
|---|---|
| iOS, macOS | The OS pairs on first access to an encrypted attribute (system prompt). Apps cannot delete bonds; the user must choose "Forget This Device". Identifiers differ per phone, so identity comes from the GET_INFO serial. A device reflash that wipes bonds makes encryption fail until the user forgets it, and the app must say so |
| Android | Call `createBond()` before GATT work (auto-pairing varies by OEM), watch bond-state broadcasts, request BLUETOOTH_CONNECT (API 31+). CCC subscriptions persist only if the device stores them for bonded peers |
| Windows | Pair via `DeviceInformation.Pairing` (custom, ConfirmOnly) or Settings; today's bleak app needs its pairing call; after a device reflash, remove the stale device |

**MCUboot + SMP.** Sysbuild `SB_CONFIG_BOOTLOADER_MCUBOOT` on the partitions already defined (README 250-259), signed images (key kept out of the repo), swap with revert, confirmed only after AFE, BLE and USB come up. SMP runs over encrypted BLE and over a second CDC ACM interface. The relink needs one SWD pass per unit: do it once, with v2. Hosts: iOSMcuManagerLibrary, nRF Connect Device Manager (Android) and smpmgr (Windows) now, an SMP client in L1 later. A 245 KB image takes ~2-6 s on Windows/Android and ~4-13 s on Apple.

## 6. Migration path

| Phase | Work | Windows app |
|---|---|---|
| 0 | Extract L0 to `lib/swifteeg/` and build the firmware from it; run the golden suites on the PC too; fix R3-DSP-01, R1-ACQ-01; set FP flags | unchanged |
| 1 | Schema of v1 as built, generator, generated README tables | unchanged |
| 2 | L1 session speaking v1 (parser, matching by opcode until v2, ts-continuity gaps, recorder + sidecar) as a Python wheel under `swifteeg_link.Link`; old and new run side by side on captured bytes | same UI, core underneath |
| 3 | Filter design and host chain move from eeg_dsp.py into L0 | same UI |
| 4 | One firmware release with v2, security and MCUboot/SMP (one SWD pass); core negotiates v1/v2 | same UI, v2 features |
| 5 | macOS + iOS first (one CoreBluetooth adapter, hardest budget), then Android, then native Windows replaces Tk | Tk retired last |

First three steps (on a branch, WORKLOG updated):
1. Create `lib/swifteeg/` from `src/proto`, `src/dsp`, `src/pipeline/chain.c` + `frame.h` and `src/timebase/timebase_math.h`. Point the firmware CMakeLists at it and add a host CMake test runner, so tests/proto, dsp and pipeline also run on the PC. Fix R3-DSP-01 there, with the truncated-then-distinct-frames test.
2. Write `protocol/swifteeg.yaml` for v1 exactly as built, with the generator for the C header, Python constants and README tables. Then draft v2 in the same file and check it against section 4 before writing any app code.
3. Start `swifteeg-core` (Rust, linking lib/swifteeg) as a Python wheel behind `swifteeg_link.Link`: parser, reply matching, ts/seq loss accounting and the recording sidecar first. Capture raw BLE and USB bytes from today's app as the first conformance transcripts, and switch over only when the decoded output is identical.
