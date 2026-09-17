# Phase A firmware fixes — progress log

Branch: m1-bringup. Repo: D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS
Owner: signal integrity/consistency first, failure must never look like success, recordings stay raw.
Build cmd: `powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1` (success prints "FLASH:").
No git, no flash, no hardware.

## Plan (in order)
1. R3-DSP-01 proto.c try_extract payload-clobber (+ ztest)
2. R3-DSP-12 table-driven CRC-16/CCITT-FALSE
3. R1-ACQ-01 timebase past-rule stamping in SPIM3 END ISR + timebase_now_us lock extend (+ test)
4. R1-ACQ-07 afe_enter_command_mode/afe_resume_streaming all-or-nothing
5. R1-ACQ-08 stream_start: START before DMA frame buffers
6. R1-ACQ-11 anomaly-198 workaround + CHnSET/LOFF readback
7. command.c: R3-DSP-02 SET_RATE validate+cap 1000SPS, R3-DSP-07 gain/mux range check, R3-DSP-09 GET_CONFIG EFAILED, R5-CONTRACT-02 respond() no silent truncate + BUILD_ASSERT, R5-CONTRACT-06 empty CMD payload -> EBADARG
8. R3-DSP-03 pipeline.c DSP thread bounded batch + sleep
9. R1-ACQ-09 watchdog + CONFIG_RESET_ON_FATAL_ERROR + hwinfo reset cause log
10. R3-DSP-04 re-prime DC + SETTLING on mux/power-down change
11. R3-DSP-06 chain.c non-finite channel reset+flag
12. Mirror 10-11 in tools/pipeline_ref.py; run proto_ref.py, dsp_ref.py, pipeline_ref.py, eeg_dsp.py; regen goldens if needed

## Status

### 1. R3-DSP-01 — DONE
Files: src/proto/proto.h (added `pending_consume` field to proto_stream_t), src/proto/proto.c
(new `apply_pending_consume()`, try_extract() now sets `st->pending_consume = st->need` instead of
calling consume() immediately, proto_stream_poll()/proto_stream_push() call apply_pending_consume()
first thing). Payload pointer now stays valid until the next push/poll, as the header already
documented.
Test: tests/proto/src/main.c `test_stream_truncated_frame_preserves_distinct_payloads` — truncated
(header-only) frame followed by 3 DIFFERENT golden frames pushed back to back so they pile up
behind the resync; each frame's payload is copied out immediately on its own push()/poll() and
compared byte-for-byte at the end. Traced by hand: under the old immediate-consume code frame[0]'s
payload reads back as frame[1]'s bytes; with the fix it's correct. Cannot run on hardware (no
flash), so verified by build only.
Build: app FLASH 250400 B (23.88%) RAM 101370 B (38.67%); proto test suite FLASH 51772 B RAM 22272 B.
Both clean, no new warnings.

### 2. R3-DSP-12 — DONE
Files: src/proto/proto.c — `proto_crc16()` replaced the bit-at-a-time loop with a 256-entry
`static const uint16_t crc16_table[256]` (CRC-16/CCITT-FALSE, poly 0x1021) and a one-lookup-a-byte
update. Table generated with a standalone Python script (same generation algorithm as
tools/proto_ref.py's `binascii.crc_hqx` reference) and cross-checked against the bitwise algorithm
for several lengths incl. 0/1/512 bytes, and against the "123456789"->0x29B1 check value, BEFORE
pasting into C. Caught and fixed my own transcription error here: first paste had ~25 extra
hand-continued rows past the real 256-entry table (pattern-completed, not from the generator) —
diffed the in-file array byte-for-byte against the regenerated reference file to confirm the fix;
now exact. Existing `test_crc_check_value` ztest covers the check value, unchanged.
Build: app FLASH 250912 B (23.93%, +512 B = exactly the table, RAM unchanged); proto suite FLASH
52284 B RAM 22272 B. Both clean.

### 3. R1-ACQ-01 — DONE
Files: src/timebase/timebase_math.h (new pure `timebase_extend_past(now, capture)` helper, peer to
`timebase_extend`, host-testable — factors out the "t>now -> t-=2^32" past rule), src/timebase/timebase.c
(`timebase_now_us()` now extends via tb_stamp() BEFORE `k_spin_unlock` instead of after;
`timebase_stamp_past_us()` refactored to call the new shared helper, behavior-preserving; new
`timebase_stamp_past_us_from_isr()` — triggers CC_NOW, reads it, tb_stamp()'s it, applies the past
rule, all without taking tb_lock, safe specifically because TIMER1's wrap ISR and the SPIM3 END ISR
share NVIC priority 1 (confirmed myself in build/zephyr/zephyr.dts: timer1 and spi3 both
`interrupts = < ... 0x1 >`) so same-priority Cortex-M IRQs can't preempt each other), src/timebase/timebase.h
(declares the new function), src/afe/ads1299.c (END ISR ~931 now calls
`timebase_stamp_past_us_from_isr()` instead of the unsafe `timebase_stamp_us()`).
Left src/pipeline/capture.c:224 (also calls timebase_stamp_us(), same latent issue) untouched —
out of my file ownership for this task.
Test: tests/timebase/src/main.c — `test_extend_past_resolves_what_extend_alone_cannot` reproduces
the exact R1-ACQ-01 numbers (capture 0xFFFFFF00 just before a wrap, "now" read after the wrap ISR
already ran) and asserts timebase_extend_past() puts it back in the old epoch (also asserts
timebase_extend() alone genuinely can't, i.e. the disagreement test_both_orderings_agree's
low-counter-only coverage missed); `test_extend_past_leaves_the_same_epoch_alone` sanity-checks the
ordinary no-wrap case. Pure-math tests (timebase_math.h only), matches this suite's existing
host-testable style.
Build: app FLASH 250944 B (23.93%) RAM 101370 B; timebase suite FLASH 47280 B RAM 18176 B. Both clean.

### 4. R1-ACQ-07 — DONE
Files: src/afe/ads1299.c. Reordered `afe_resume_streaming` before `afe_enter_command_mode` (the
latter now calls the former, needed forward order since this file uses no prototypes). 
`afe_resume_streaming`: RDATAC and START are now BOTH always attempted and CS/frame-buffers/clock/
gate are ALWAYS restored regardless of either command's result (previously returned early on RDATAC
failure, skipping all of it); returns the first error. `afe_enter_command_mode`: STOP now only
attempted if SDATAC succeeded (unchanged behavior there), but on ANY error now calls
`afe_resume_streaming(true)` before returning, so the trigger gate/CS/clock get put back instead of
staying gated off until an unrelated later access happens to succeed. Callers (read_reg_safe,
set_input, set_bias, set_leadoff, get_channels, set_channel) all needed no changes - they already
return enter's error as-is, which now carries a restored state with it.
Build: app FLASH 251024 B (23.94%) RAM 101370 B. Clean (one intermediate build caught a missing
forward-declaration ordering issue, fixed by reordering the two functions rather than adding a
prototype, matching this file's style of no forward declarations).

### 5. R1-ACQ-08 — DONE
Files: src/afe/ads1299.c `ads1299_stream_start()`. Reordered so `ads1299_start_conversions()`
(START) runs BEFORE the frame buffers are pointed at (afe_dummy/afe_frame[active]), matching
afe_resume_streaming()'s order — previously START's own afe_xfer() call clobbered the just-set
buffer pointers with its 1-byte command buffers and nothing restored them, so the first frame after
every start/restart was read into the small command RX buffer while afe_frame[0] stayed stale/zero.
Also moved the trigger-gate enable to the very end (after buffers + IRQ are both set up, matching
the "gate last" invariant afe_resume_streaming already uses) instead of before START — belt-and-
suspenders so the first DRDY that gets through always sees correct buffers, not just "safe because
conversions can't run yet". Moved `afe_streaming = true` earlier (right before START) so a START
failure still unwinds through the existing `ads1299_stream_stop()` teardown (which early-returns if
`!afe_streaming`) instead of leaving RDATAC/CS/clock half set up; verified stream_stop's teardown
steps are all safe/idempotent to run this early (IRQ never connected/enabled yet, gate never opened).
Build: app FLASH 251024 B (23.94%, unchanged — pure reordering) RAM 101370 B. Clean.

### 6. R1-ACQ-11 — DONE
Files: src/afe/ads1299.c. Added `afe_anomaly_198_enable()`/`afe_anomaly_198_disable()` (address
math and the 0x40000E00 register copied verbatim from
C:\ncs\v3.4.0\modules\hal\nordic\nrfx\drivers\src\nrfx_spim.c's static anomaly_198_enable/disable —
those symbols are private to nrfx_spim.c so couldn't be called directly, hence local copies), wired
around the START-trigger/END-wait in `afe_xfer()` (both the success and timeout return paths),
gated on `NRF_ERRATA_STATIC_CHECK(52,198) && NRF_ERRATA_DYNAMIC_CHECK(52,198)` (the latter reads the
FICR variant/revision at runtime, same macros nrfx itself uses — already reachable via the existing
`<hal/nrf_spim.h>` include, confirmed nrf_spim.h itself calls NRF_ERRATA_DYNAMIC_CHECK for a
different erratum). Only applied to afe_xfer (register access) not the streaming DMA path, matching
review scope — streaming ignores MOSI so is unaffected.
Added `afe_write_reg_verified()` (write + read back + EIO on mismatch, same pattern CONFIG1/CONFIG3
already use in ads1299_configure/set_data_rate) and `afe_write_chset_verified()` (wraps it, updates
`afe_chset` cache only after the readback confirms the write landed). Switched
`ads1299_set_leadoff()`'s 4 writes (LOFF, LOFF_SENSP, LOFF_SENSN, CONFIG4) and
`ads1299_set_channel()`'s CHnSET write(s) to the verified versions. Left `ads1299_set_channels()`
(the bulk internal one used only by ads1299_configure, has its own spot-check already) untouched —
not the function the review flagged (728-773 is set_channel, singular).
Build: app FLASH 251284 B (23.96%, +260 B) RAM 101370 B. Clean.

### 7. Command handling (R3-DSP-02, phase-A rate cap, R3-DSP-07, R3-DSP-09, R5-CONTRACT-02, R5-CONTRACT-06) — DONE
Files: src/transport/command.c, src/transport/command.h, src/pipeline/pipeline.c, src/pipeline/pipeline.h.
- R3-DSP-02 + rate cap: new `pipeline_rate_supported(sps)` (pipeline.h/.c, wraps existing
  `rate_code_for`). `pipeline_rate()` now returns 0 unless `running` (previously reported
  `sample_rate_hz` even after a failed restart, since that's written before the AFE is touched).
  CMD_SET_RATE now rejects `sps > 1000` (phase-A cap, owner decision, every link) or an
  AFE-unsupported rate with CMD_EBADARG BEFORE the early "OK, restarting" reply.
- R3-DSP-07: CMD_SET_CHANNEL rejects gain code > 6 or mux > 7 with CMD_EBADARG before calling
  ads1299_set_channel(); command.h's doc comment updated ("[2]=gain code 0-6, [3]=mux 0-7").
- R3-DSP-09: CMD_GET_CONFIG now checks `ads1299_get_channels()`'s return and answers CMD_EFAILED
  (instead of CMD_OK with an all-zero chset) on a failed readback.
- R5-CONTRACT-02: `respond()` rewritten — payload size is now the named `RSP_PAYLOAD_BYTES`/
  `RSP_EXTRA_MAX` (48/46, same numbers as before, just named); when `extra_len > RSP_EXTRA_MAX` it
  now logs and sends `[opcode, CMD_EFAILED]` instead of silently sending `[opcode, CMD_OK]` with the
  extra data dropped. Added `BUILD_ASSERT(sizeof(cfg) <= RSP_EXTRA_MAX, ...)` at GET_CONFIG's `cfg[37]`
  so a future field added there that overflows fails the build instead of hitting this path silently.
- R5-CONTRACT-06: `handle()` now answers `respond(CMD_OP_NONE, CMD_EBADARG, NULL, 0)` for a
  zero-length CMD payload instead of returning with no reply at all; new `CMD_OP_NONE` (0x00, never a
  real opcode) in command.h.
Build: app FLASH 251396 B (23.97%) RAM 101370 B. Clean. tests/pipeline confirmed unaffected (it
links chain.c + dsp.c only, not pipeline.c) — no rebuild needed for this fix specifically.

### 8. R3-DSP-03 — DONE
Files: src/pipeline/pipeline.c. `dsp_entry()`'s drain loop (`while ((rf = spsc_peek(...)) != NULL)`)
was unbounded — at priority 2 (above command@6, TX@4) a rate/filter load the thread can't sustain
meant the ring never went empty, so it never reached the `k_sem_take()` that's the only place this
thread yields to lower-priority threads: total lockup, power cycle only. Added `DSP_BATCH_FRAMES=64`
cap on the drain loop; if the cap is hit (still frames waiting), `k_sleep(K_TICKS(1))` before the
outer loop re-checks. Ring drop counting (`spsc_dropped`, read in `process()`) is untouched, so a
fall-behind still shows up as counted/flagged drops — it just no longer wedges the device.
Build: app FLASH 251412 B (23.98%) RAM 101370 B. Clean.


