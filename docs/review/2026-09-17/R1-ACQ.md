# R1-ACQ — Acquisition and timing core (DRDY edge to DSP thread, boot/stream lifecycle)

Repo HEAD f135b9a on m1-bringup (4c34102 on top changes WORKLOG.md only). Read-only review.
Counts: 2 high, 9 medium, 3 low (findings sorted by severity).

## Progress
- Files finished (all owned files): src/pipeline/capture.c/.h, src/sys/ringbuf.c/.h,
  src/timebase/timebase.c/.h, timebase_math.h, src/pipeline/frame.h, src/pipeline/pipeline.c/.h,
  src/afe/ads1299.c/.h, src/main.c, src/board/supply.c, src/board/leds.c, prj.conf, app.overlay,
  CMakeLists.txt, boards/shreyash/swifteeg/*; build/zephyr/.config (effective Kconfig), zephyr.dts
  IRQ priorities, zephyr.map sizes/addresses; README 133-168, 250-266, 415-575, 875-896, 1036-1059.
- Crossed into: stream.c (sink, batch lock, DATA header), chain.c (validation), command.c
  (SET_RATE/SET_INPUT/GET_CONFIG), tests/timebase, swifteeg_app.py (how the host uses ts/seq),
  Zephyr log_core.c / log_backend_rtt.c / kernel/fatal.c / clock_control_nrf.c, nrfx_spim.c,
  mpsl_init.c, hci_core.c.
- Status: complete.

## Findings

### R1-ACQ-01 [high] EEG sample stamped 2^32 us (71.6 min) in the future when TIMER1 wraps between DRDY and the END ISR
- Category: bug (timing)
- Where: src/afe/ads1299.c:931; src/timebase/timebase.c:41-49, 58-70, 118-128; src/timebase/timebase_math.h:29-37; build/zephyr/zephyr.dts timer1 `interrupts = < 0x9 0x1 >`, spi3 `interrupts = < 0x2f 0x1 >`; tests/timebase/src/main.c:53-67
- Evidence:
  - END ISR: `const uint64_t ts = timebase_stamp_us(timebase_capture_get());`
  - `tb_stamp()`: `w1 = tb_wraps; pending = nrf_timer_event_check(TB_TIMER, WRAP_EVENT); ... return timebase_extend(w1, counter, pending);`
  - `timebase_extend()`: `if (wrap_pending && counter < TIMEBASE_HALF) { wraps++; }` - only corrects the *pending* case.
  - The IMU path already uses the safe variant: imu.c:507 `timebase_stamp_past_us(capture)`.
  - Test gap: `test_both_orderings_agree` checks the two ISR orderings only for a *post*-wrap counter (`counter = 250`). With a pre-wrap counter the orderings disagree: `timebase_extend(4, 0xFFFFFF00, true)` = HI(4)+... but `timebase_extend(5, 0xFFFFFF00, false)` = HI(5)+.... Once the ISR has run, the function cannot tell them apart.
- Failure scenario: DRDY latches CC[0] = 0xFFFFFFE0. The counter wraps ~32 us later, before the 27-byte transfer ends. TIMER1 (IRQ 9) and SPIM3 (IRQ 47) are both priority 1, so the wrap ISR runs at the wrap, clears the event and does `tb_wraps++`. The END ISR then reads pending = false, the new wrap count, and counter 0xFFFFFFE0: `ts = (new_wraps << 32) | 0xFFFFFFE0`, 4294.97 s too late. The window is (transfer ~27 us + END latency) / period per wrap: ~0.75 % at 250 SPS, ~3 % at 1 kSPS, ~50 % at 16 kSPS. The first wrap comes 71.6 min after boot, so the 30-min soaks could never hit it. The host writes CSV rows as `ts0 + i * period_us` (swifteeg_app.py:1212), so a whole 6-sample batch gets recorded 71.6 min out of place, with no warning (the host checks seq, not ts).
- Same flaw, smaller window: `timebase_now_us()` releases the spinlock *before* `tb_stamp(counter)`. If the wrap ISR runs between the CC_NOW capture and the `tb_wraps` read, "now" is +2^32 too. That also defeats `timebase_stamp_past_us()`, the IMU path, whose rule trusts `now`. The window is a few us per call, so it is rare, but it has the same root cause.
- Confidence: confirmed-by-reading (the priorities come from the generated DTS; the probability is arithmetic)
- Fix sketch: in the END ISR, extend with the "past" rule against the live counter (as `timebase_stamp_past_us` does, but without the spinlock round trip). Or in `tb_stamp`: if `!pending && counter >= HALF` and the current counter is `< HALF`, use `w1 - 1`. In `timebase_now_us`, do the extend inside the locked section. Add the pre-wrap/ISR-early case to tests/timebase.

### R1-ACQ-02 [high] A late SPIM3 END ISR silently merges samples, and can deliver a torn or stale-duplicate frame; nothing detects it (`afe_overrun` is never incremented)
- Category: race / robustness (silent data loss, hard ceiling at higher rates)
- Where: src/afe/ads1299.c:915-947, 954-957, 970; src/pipeline/pipeline.c:426-439; ads1299.h:341-345
- Evidence:
  - ISR: `afe_active ^= 1; nrf_spim_rx_buffer_set(AFE_SPIM, afe_frame[afe_active], ...); ... afe_cb(afe_frame[done], ts);`
  - `static volatile uint32_t afe_overrun;` is only ever set to 0 (line 970) and returned (956). No code increments it, and nothing calls `ads1299_overruns()`. The header promises "Frames whose transfer had not finished when the next DRDY arrived. Any non-zero value means samples were lost."
  - PPI starts the transfer on every DRDY with no software gate. END is a single event flag, so two ENDs merge into one interrupt.
- Failure scenario (END ISR latency L, period T, transfer ~27 us at 8 MHz):
  - (a) L >= T: DRDY n+1 restarts SPIM into the same, unswapped buffer and overwrites sample n. One ISR delivers n+1 with ts(n+1). Sample n is gone, seq stays continuous, no OVERRUN flag, and the host's `ts0 + i*period` shifts every later row in that batch one period early.
  - (b) T-27 us <= L < T: the ISR runs while transfer n+1 is filling buffer A, so `on_frame` memcpy's A mid-DMA and gets a torn mix of n+1 and n, stamped ts(n+1). The byte-0 marker check passes, because byte 0 already holds the new status byte. The END of transfer n+1 then delivers buffer B, which still holds sample n-1, again stamped ts(n+1). Result: one torn sample plus one stale duplicate, both fed through the filters as real data.
  - Budget: L must stay under T-27 us, which is ~970 us at 1 kSPS but ~220 us at 4 kSPS, ~95 us at 8 kSPS and ~35 us at 16 kSPS. Priority 1 is shared with USBD (IRQ 39, the 16 kSPS transport), TIMER1 and SPIM2. MPSL radio ISRs run above it as zero-latency IRQs (`CONFIG_ZERO_LATENCY_IRQS=y`). At the planned USB rates this is expected, not rare. At <=1 kSPS the 30-min soak says it is rare, but nothing would report it if it happened.
- Confidence: confirmed-by-reading for (a) and for the missing detection. Likely for (b): it assumes RXD.PTR is latched at TASKS_START, which is what the ISR comment itself relies on.
- Fix sketch: in the ISR, keep the previous capture. If `cc - prev_cc > 1.5*T`, count `afe_overrun` and pass an overrun flag with the frame. If `cc == prev_cc`, drop the frame (duplicate). To stop the torn read, check RXD.AMOUNT, or copy the frame and then verify CC[0] did not move during the copy. Raise SPIM3 above the other priority-1 peripherals (move USBD/SPIM2 to 2).

### R1-ACQ-03 [medium] The OVERRUN flag is put on the wrong sample: it marks the oldest queued frame, up to 127 samples *before* the hole
- Category: contract-drift (timing)
- Where: src/pipeline/pipeline.c:482-487 (and 426-439); pipeline.h:24 `#define EEG_FLAG_OVERRUN 0x02 /* frames were dropped before this one */`
- Evidence: the drop counter is sampled when a frame is *processed*: `const uint32_t drops = spsc_dropped(&raw_ring); if (drops != ring_drops_seen) { ... out.flags |= EEG_FLAG_OVERRUN; }`. The ring drops the *newest* frame (`spsc_push` refuses when full).
- Failure scenario: the DSP thread stalls and the ring fills with frames k..k+127. Frames k+128.. are refused. When the thread resumes, it processes frame k first, sees the counter moved, and flags k. The real discontinuity is between k+127 and the next accepted frame, 127 samples (127 ms at 1 kSPS) later, and that sample carries no flag. Sequence numbers are continuous across the hole, so a host that marks, splits or NaN-pads at the flagged sample misplaces it. With stream.c's per-batch flags, the host cannot even narrow it to 6 samples.
- Confidence: confirmed-by-reading
- Fix sketch: record the drop on the producer side. Keep a `pending_overrun` bit in the ISR, set when a push fails and written into the next `raw_frame` that *is* pushed. Or carry the ISR's running drop count in `raw_frame` and compare consecutive values in the DSP thread.

### R1-ACQ-04 [medium] A frame rejected for a bad status word disappears without trace: `st_seq--`, no flag, so sequence numbers and timestamps silently de-synchronise
- Category: contract-drift / robustness
- Where: src/pipeline/pipeline.c:455-471; host consequence at swifteeg_app.py:1026-1028, 1208-1212
- Evidence: `.seq = st_seq++, ... if (!chain_process(...)) { st_bad_status++; st_seq--; /* it never became a sample */ return; }`. No `EEG_FLAG_OVERRUN` or other flag is carried to the next good sample. This is the same silent-loss class the M4 fix addressed for ring drops (pipeline.c:474-481), left open on this path.
- Failure scenario: a DRDY whose frame is misaligned or corrupt (a torn read from -02, the zero first frame from -08, an SPI glitch) is a real conversion that happened. The next good sample takes its sequence number, so the host sees no gap and no flag. It writes rows as `ts0 + i*period_us`, placing every later sample in that batch one period early. The loss is visible only in the device's health counter, which no host reads.
- Confidence: confirmed-by-reading
- Fix sketch: keep the seq increment for rejected frames (a seq gap *is* the signal) and set a flag on the next good sample. Better, derive seq from the DRDY count or timestamp in the ISR, so any loss upstream of the host shows as a gap.

### R1-ACQ-05 [medium] The timestamp is the DRDY edge, not the sampling instant: the AFE's own rate-dependent filter delay (~1.5 conversion periods) is neither corrected nor reported
- Category: architecture (timing model)
- Where: src/afe/ads1299.c:927-931; src/pipeline/pipeline.h:28 (`ts_us; /* latched in hardware at DRDY */`); stream.c:57 DATA header; no mention of AFE latency in README or any header (grep "group delay" finds only host sections, README:1076)
- Evidence: `const uint64_t ts = timebase_stamp_us(timebase_capture_get());` becomes the sample's time as-is. GET_INFO/GET_CONFIG carry the rate but no latency.
- Failure scenario: the ADS1299's sinc3 decimator puts DRDY after the averaging window, so each value represents the input about 1.5 x t_DR earlier. That is ~6 ms at 250 SPS, ~3 ms at 500, ~1.5 ms at 1 kSPS and ~0.1 ms at 16 kSPS. An ERP analysis that lines a TIMER1-stamped stimulus marker up against these timestamps (either candidate method, software marker or trigger wire, lands on that clock) biases every latency by that amount, and the bias changes with the rate. Data recorded at 250 and 1000 SPS disagree by ~4.5 ms, well inside the P300/N170 latency effects people measure. The goal list asks for "reported group delay", and this is the one delay the firmware knows exactly and does not report.
- Confidence: confirmed-by-reading that no correction/report exists; likely for the magnitudes (standard sinc3 math; ADS1299 datasheet not in the SDK)
- Fix sketch: add a per-rate `afe_delay_us` table from the datasheet's filter spec. Verify it once on the bench: drive a step into one input from a GPIO whose edge is captured on TIMER1, then fit the step's midpoint against DRDY stamps at each rate. Report it in GET_INFO and apply it in the shared host timing model (keep the raw DRDY stamps on the wire).

### R1-ACQ-06 [medium] Two failure paths leave a device that is connected but useless, and nothing tells the host
- Category: robustness / contract-drift
- Where: src/main.c:256-276, 326-357; src/pipeline/pipeline.c:606-624, 866-869, 871-906; src/transport/command.c:374-389, 519
- Evidence:
  - Boot: `if (!afe_present) { ... return; }` and `if (pipeline_start(ADS1299_DR_250SPS) != 0) { LOG_ERR(...); return; }` both come before `command_init()`. `usb_transport_init()` and `ble_transport_init()` have already run.
  - Rate change: command.c `respond(op, CMD_OK, NULL, 0); (void)pipeline_set_rate(sps); return;`
  - `pipeline_start` assigns `current_rate_code = rate;` and `sample_rate_hz = ...` *before* anything that can fail. `pipeline_rate()` returns `sample_rate_hz` whether or not `running` is true, and GET_CONFIG reports it (command.c:519).
- Failure scenario:
  - (a) This extends a known open item. On the boards built without an ADS1299 (main.c:8-10 says two of three), the IMU still initialises and the device advertises and enumerates, but no command is ever handled. Motion can never be streamed or configured, and the host cannot tell "no AFE" from "firmware hung". A *transient* AFE failure at boot (a readback mismatch during a supply dip, an SPI timeout) does the same until power-cycled: there is no retry and no error report.
  - (b) A restart that fails twice leaves `running == false`. The host already got CMD_OK. Its follow-up GET_CONFIG reports the *new* rate, so the app's rate-change state machine (swifteeg_app.py `_set_rate`/`_rate_change_overdue`, ~636-700) completes and resumes a stream that never produces data. No EVT is sent, and nothing retries.
- Confidence: confirmed-by-reading
- Fix sketch: always run `command_init()` and report AFE/IMU presence and pipeline state in GET_INFO. Retry `pipeline_start` from a delayed work item. Send an EVT (or delay the RSP) with the restart result, and make `pipeline_rate()` return 0 when not running.

### R1-ACQ-07 [medium] A failed register access mid-stream leaves the DRDY trigger gated off: acquisition silently stops until some later register command succeeds, a *different* rate is set, or the device is reset
- Category: robustness
- Where: src/afe/ads1299.c:427-491 (enter), 493-535 (resume), and callers 546-549, 563-567, 616-620, 668-672, 713-717, 737-741; src/pipeline/pipeline.c:879-881
- Evidence:
  - Every mid-stream accessor does `int err = afe_enter_command_mode(&was_streaming); if (err) { return err; }`, so it never calls `afe_resume_streaming`. By that point enter has already done `afe_gate(false)`, `afe_cs_held = false`, CS high, and 1 MHz.
  - `afe_resume_streaming`: `int err = afe_cmd(ADS1299_CMD_RDATAC); if (err) { return err; }`. It returns before CS low, the buffer restore, 8 MHz and `afe_gate(true)`.
  - `afe_streaming` stays true, and the pipeline's `running` stays true.
  - `pipeline_set_rate`: `if (running && code == current_rate_code) { return 0; }`
- Failure scenario: a register command (CMD_GET_CONFIG, set bias, and so on) hits an SPI timeout in SDATAC/STOP/RDATAC. `afe_xfer` then runs `afe_recover()` and returns -ETIMEDOUT. The README records such timeouts at 1 kSPS, and `afe_recover` exists because they happen. The host gets an error reply, but PPI stays disabled, so no transfer and no timestamp occur again. The DSP thread idles, the stream goes silent while connected, and re-sending the current rate is a no-op. Data only comes back by accident: the next register command that *succeeds* re-enters with `was_streaming = true` and its resume restores the gate. A different rate or a power cycle also works. Nothing tells the host that it needs any of these. The same happens if START fails inside resume: the gate is re-enabled, but conversions may be stopped.
- Confidence: confirmed-by-reading
- Fix sketch: make enter/resume all-or-nothing. On any error in enter, run the resume sequence (or `afe_recover` + RDATAC/START + gate on) before returning. In resume, always restore CS, buffers, clock and gate, even if a command failed, and return the first error. Add a stall detector (no frame for more than 5 periods while `running`) that restarts acquisition and raises an EVT.

### R1-ACQ-08 [medium] Fix incomplete: `ads1299_stream_start` leaves EasyDMA on the 1-byte command buffers, so the first "sample" of every start and restart is stale (or zero)
- Category: bug (regression of the README "every register access slipped in an old sample" fix, on the start path)
- Where: src/afe/ads1299.c:1003-1006 then 1024; afe_xfer 205-206; ISR 940-945; compare afe_resume_streaming 514-524, which does it in the right order
- Evidence:
  - stream_start sets `nrf_spim_tx_buffer_set(AFE_SPIM, afe_dummy, 27); nrf_spim_rx_buffer_set(AFE_SPIM, afe_frame[afe_active], 27);`, enables the gate, *then* `err = ads1299_start_conversions();` → `afe_cmd(START)` → `afe_xfer(1)`: `nrf_spim_tx_buffer_set(AFE_SPIM, afe_tx, len); nrf_spim_rx_buffer_set(AFE_SPIM, afe_rx, len);`. Nothing restores the frame buffers afterwards.
- Failure scenario: the first DRDY after START clocks a 1-byte transfer into `afe_rx`. The ISR swaps to `afe_frame[1]` and hands on `afe_frame[0]`, which this session never wrote.
  - At the first start after boot (BSS zeros), the frame fails the marker check. `bad_status` becomes 1 and the first real sample is silently dropped (see -04).
  - After every rate change, `afe_frame[0]` still holds a valid frame from the *previous* session. It is processed as sample 0 of the new session with the new first timestamp: it primes the freshly built DC estimator and notch, and reaches any host that is already streaming. The current app hides it by sending STREAM_STOP first.
  - TX stays on `afe_tx` (MAXCNT 1) until the next register access, so every frame's first DIN byte is 0x08 (START). The comment at 1000 assumes zeros. Whether the part decodes it mid-RDATAC is `speculative`. At boot, with no host, acquisition runs in this state indefinitely.
- Confidence: confirmed-by-reading (the stale/zero first frame); speculative (the effect of the 0x08 DIN byte)
- Fix sketch: send START before setting the frame buffers, the same order as `afe_resume_streaming`. Or re-set TX/RX after `ads1299_start_conversions()`. Check in the ISR that RXD.AMOUNT == 27, and count the frame as bad otherwise.

### R1-ACQ-09 [medium] No watchdog, and a fatal error halts forever; the radio link probably stays up, so the headset can look connected while dead
- Category: robustness
- Where: prj.conf:37-40 (`CONFIG_RESET_ON_FATAL_ERROR=n`, `CONFIG_ASSERT=y`); no WDT use anywhere in src/ (only the DT alias `watchdog0 = &wdt0`, swifteeg.dts:60); build/zephyr/.config `CONFIG_ZERO_LATENCY_IRQS=y`; zephyr/kernel/fatal.c:21-31, 37-46; nrf/subsys/mpsl/init/mpsl_init.c:188, 280-284
- Evidence:
  - The default handler runs `LOG_PANIC(); LOG_ERR("Halting system"); arch_system_halt(reason);`, which does `(void)arch_irq_lock(); for (;;) {}`.
  - MPSL connects TIMER0/RTC0/RADIO with `IRQ_ZERO_LATENCY`, and irq_lock does not mask those.
- Failure scenario: an MPU stack-guard hit or an assert mid-recording halts every thread. This has happened on this codebase more than once (README: BT RX WQ overflow "the moment a central connected", udc_nrfx overflow). The link layer runs in zero-latency ISRs and likely keeps the connection alive with empty PDUs, so the host sees a connected device that sends and answers nothing, with no supervision timeout to tell it so. Only SW1 (nRESET) or a power cycle recovers it. Separately, a thread hang with no fault has no net at all: a wedged command thread, or a busy loop (see -13).
- Confidence: confirmed-by-reading for the halt/no-watchdog part; likely that the link stays up
- Fix sketch: `CONFIG_RESET_ON_FATAL_ERROR=y` (NCS lib) plus a retained-RAM or `hwinfo` reset cause reported in GET_INFO. Add a hardware WDT (e.g. 4 s) fed from the DSP thread only while frames arrive, plus the idle/main loop. Keep `CONFIG_ASSERT` for bench builds only.

### R1-ACQ-10 [medium] Acquisition ceilings below the 16 kSPS plan, besides -02: an 8 ms ring behind cooperative and priority-0 work, and several per-sample kernel round trips
- Category: performance (hard ceiling for the planned USB rates)
- Where: src/pipeline/pipeline.c:27-31 (ring), 436 (`k_sem_give` per frame), 453 and 509 (two `timebase_now_us()` per sample); src/transport/stream.c:275 (`k_mutex_lock(&batch_lock, K_FOREVER)` per sample), stream.c:29 and 160 (6-sample batches, 32-frame queue); main.c:363-371; build/zephyr/.config `CONFIG_MAIN_THREAD_PRIORITY=0`, `CONFIG_SYSTEM_WORKQUEUE_PRIORITY=-1`, `CONFIG_BT_RX_PRIO=8` (zephyr hci_core.c:4738 `K_PRIO_COOP(CONFIG_BT_RX_PRIO)`), `CONFIG_MPSL_THREAD_COOP_PRIO=6` (mpsl_init.c:525 `K_PRIO_COOP`), `CONFIG_LOG_MODE_IMMEDIATE=y`, `CONFIG_LOG_BACKEND_RTT_MODE_BLOCK=y`, `RETRY_CNT=4`, `RETRY_DELAY_MS=5`; zephyr log_backend_rtt.c:178-186 (`k_busy_wait(USEC_PER_MSEC * CONFIG_LOG_BACKEND_RTT_RETRY_DELAY_MS)` in sync mode)
- Evidence: pipeline.c's own comment says "128 frames is ... 8 ms at 16 kSPS".
  - The DSP thread (preemptible 2) is outranked by the BT RX workqueue, the MPSL work queue and the system workqueue, all *cooperative*, so it cannot preempt them. It is also outranked by main (priority 0), which runs `report_health()` with immediate-mode formatting every 10 s.
  - The first time the RTT up-buffer fills with no reader (`host_present` is initialised true), the logging thread busy-waits 3 x 5 ms = 15 ms.
- Failure scenario: at 16 kSPS, any 8 ms stretch of BT RX callback work, controller work, workqueue items, or a health-line format plus the one-off 15 ms RTT stall on main overflows the ring. Frames are then dropped, and flagged on the wrong sample (-03). Per sample, the path costs a semaphore give from the ISR (one wake/context switch per sample when the thread drains to empty), two TIMER1 capture round trips under a spinlock, and a mutex lock/unlock in the sink. At 62.5 us per sample these fixed costs, not the ~1 us filter maths, set the budget. None of it is measured above 1 kSPS; `st_dsp_total_us` (a uint32 sum of per-sample us) also wraps in hours at high rates (~2.5 h at 16 kSPS if a sample costs 30 us), which garbles `dsp_mean_us`.
- Confidence: confirmed-by-reading (priorities, ring size, busy-wait); the per-sample costs are unmeasured
- Fix sketch: size the ring in time (e.g. >=100 ms at the running rate: 2048 frames x 40 B = 80 KB at 16 kSPS, or allocate per rate). Give the semaphore only on empty→non-empty. Take the processing-time stamps with DWT CYCCNT rather than TIMER1. Batch in the sink without a per-sample mutex. Set `CONFIG_LOG_BACKEND_RTT_MODE_DROP=y` or deferred logging for streaming builds. Measure `dsp_max_us` and ring high-water at 4/8/16 kSPS over USB before claiming those rates.

### R1-ACQ-11 [medium] The direct-HAL SPIM3 driver skips the nRF52840 anomaly-198 workaround that nrfx applies to every SPIM3 transfer, and several register writes are never read back
- Category: robustness (silent misconfiguration)
- Where: src/afe/ads1299.c:189-233 (afe_xfer), 664-703 (set_leadoff: no readback), 728-773 (set_channel: no readback), 612-662 (set_bias: only CONFIG3 read back); modules/hal/nordic/nrfx/drivers/src/nrfx_spim.c:62-97, 776-779, 875-876
- Evidence:
  - nrfx does `if (NRF_ERRATA_DYNAMIC_CHECK(52, 198) && p_spim == NRF_SPIM3) { anomaly_198_enable(p_xfer_desc->p_tx_buffer, ...); }`, which writes the TX buffer's 8 KB RAM-block mask to `0x40000E00` around each transfer.
  - ads1299.c drives `NRF_SPIM3` through `hal/nrf_spim.h` only and never does this. `afe_tx` is at 0x2000913D (zephyr.map), in the same 8 KB block (0x20008000-0x20009FFF) as udc_nrf.c's EasyDMA buffer `m_tx_buffer` at 0x20008670. The nrfx workaround code is linked (`.bss.m_anomaly_198_preserved_value`), just not applied here.
- Failure scenario: nRF52840 anomaly 198 is "SPIM3 transmit data might be corrupted", which nrfx guards against unconditionally. A WREG sent while USB or the radio is doing DMA in the same RAM block could put a corrupted byte on MOSI. With `set_channel`, the chain's gain comes from the driver *cache* (`afe_chset`), not the part, so a corrupted CHnSET makes device microvolts silently wrong by the gain ratio, or leaves a channel shorted or powered down. With `set_leadoff`, it could leave lead-off current on. GET_CONFIG reads CHnSET back, so a host that compares would notice. Streaming reads are unaffected, because MOSI is ignored.
- Confidence: confirmed-by-reading that the workaround is bypassed and those writes have no readback; speculative that it corrupts under this load (the erratum's conditions are not written in the SDK sources)
- Fix sketch: copy nrfx's `anomaly_198_enable/disable` around `afe_xfer`'s START and END (a few lines), or place `afe_tx`/`afe_dummy` in a dedicated linker section/block. Read back every CHnSET/LOFF write the way CONFIG1/CONFIG3 already are, and update `afe_chset` from the readback.

### R1-ACQ-12 [low] The START-pin self-test cannot see a free-running AFE; the README's "measured: not stuck high" rests on it
- Category: robustness / docs-drift
- Where: src/afe/ads1299.c:880-910; README.md:135-143; main.c:74-79; first-start gate ordering capture.c:118 vs ads1299.c:973
- Evidence: `(void)afe_cmd(ADS1299_CMD_STOP); k_msleep(5); int first = gpio_pin_get_dt(&afe_drdy); for (i < 200) { if (gpio_pin_get_dt(&afe_drdy) != first) return true; k_busy_wait(50); }`
- Failure scenario: in SDATAC with nobody reading data, a converting ADS1299 holds DRDY *low* and only pulses it high for a few tCLK (~2 us at 2.048 MHz) before each new result. Level-polling every >=50 us catches such a pulse ~4 % of the time, so over 10 ms at 250 SPS (2-3 pulses) a stuck-high START is reported "OK" roughly 90 % of the time. If START does float high, the damage is limited in this design, because register access gates PPI off first. But the first `ads1299_stream_start` sends its RDATAC and START transfers with the PPI path already live: the connection was left enabled by `capture_init()` at boot, and the SPIM task was attached just before. Each of those transfers can then collide with a DRDY-started one (~1 % each at 250 SPS), which leads to a timeout and -06(a). The README's hardware "measurement" does not establish the pin state.
- Confidence: likely (depends on the datasheet DRDY-without-retrieval behaviour, not readable here)
- Fix sketch: detect edges in hardware instead. With PPI capture enabled, sample `timebase_capture_get()` before and after 20 ms (as `capture_measure` already does); any change means conversions are running. Gate PPI off in `capture_init()` until stream start.

### R1-ACQ-13 [low] A timebase that fails to start is logged and ignored; the boot capture test then spins forever on its frozen deadline
- Category: robustness
- Where: src/main.c:102-107, 162-191; src/timebase/timebase.c:85-89; src/pipeline/capture.c:183-186
- Evidence:
  - `if (timebase_init() != 0) { LOG_WRN("timebase unavailable"); return; }` returns before TIMER1 is started.
  - `const uint64_t deadline = timebase_now_us() + (uint64_t)ms * 1000U; while (timebase_now_us() < deadline) {`
- Failure scenario: `clock_control_on()` returns an error. `api_blocking_start` gives -EAGAIN if HFXO has not started within 500 ms, which points at a crystal or clock fault. TIMER1 never starts, so `timebase_now_us()` returns 0 forever, and `report_capture()` busy-loops on main (priority 0) for good. BLE and USB are never initialised, the LEDs never blink, and with no watchdog (-09) the board is simply dead with one log line. If the capture test were skipped, acquisition and the IMU would stream with frozen timestamps, and nothing would check.
- Confidence: confirmed-by-reading (the trigger is a rare hardware/clock fault)
- Fix sketch: bound `capture_measure` by `k_uptime_get()`. Make timebase failure fatal to acquisition (`pipeline_start` returns -ENODEV) and report it to the host.

### R1-ACQ-14 [low] MCUboot is not integrated: the dual-slot partition table is decorative, and the app links at 0x0 over the `mcuboot` partition
- Category: architecture
- Where: boards/shreyash/swifteeg/swifteeg.dts:214-236 (`boot_partition` 0x0-0xC000, `slot0` 0x74000); build/zephyr/.config `# CONFIG_BOOTLOADER_MCUBOOT is not set`, `# CONFIG_USE_DT_CODE_PARTITION is not set`, `CONFIG_FLASH_LOAD_OFFSET=0`; README.md:252-265
- Evidence: `zephyr,code-partition = &slot0_partition;` has no effect without `USE_DT_CODE_PARTITION` (zephyr Kconfig.zephyr:131-135: `default 0`). zephyr.map `_flash_used = 0x3d130` (250,160 B) from address 0.
- Failure scenario: no firmware update is possible except over SWD, and adopting MCUboot later means relinking at 0xC000 and SWD-reflashing every unit: the exact cost the README flash-map section says it wanted to avoid by tracking this from M1. The margin itself is fine: image 244 KB vs a 464 KB slot (53 %), static RAM 99 KB of 256 KB (39 %).
- Confidence: confirmed-by-reading
- Fix sketch: enable `SB_CONFIG_BOOTLOADER_MCUBOOT` (sysbuild) with this DTS partitioning, plus SMP over BLE/USB for DFU. Keep a CI size check against 0x74000 minus the MCUboot trailer.

## Checked and correct
- SPSC ring: exactly one producer (`on_frame` in the SPIM3 END ISR) and one consumer (the DSP thread, peek/process/release). `spsc_init` runs only in `pipeline_start`, after `ads1299_stream_stop()` has done `irq_disable(SPIM3)` and the DSP thread has been joined (or before either exists). `pipeline_reset_stats` has no caller besides `pipeline_start` (before thread creation), so the `st_seq`/`ring_drops_seen` resets cannot race. On overflow the push is refused, never overwritten. A peeked slot cannot be reused while held (the full test is `head - tail >= capacity`). Semaphore/ring disagreement is handled by draining. The ISR's `atomic_fetch_add` on `dropped` against the thread's load loses nothing.
- frame.h: byte order and 24→32-bit sign extension are correct. `chain_process` rejects before touching any state. Caveat: the marker is only 4 bits, so ~1/16 of misaligned frames pass it.
- TIMER1: 32-bit, prescaler 4 = 1 MHz from HFCLK. The COMPARE1 = 0 wrap detector fires only on rollover. HFXO is requested once through the ref-counted onoff manager (`clock_control_on` → `api_blocking_start` → `onoff_request`) and never released, so the crystal holds for the whole run. `timebase_stamp_past_us` math is right *given* a correct `now` (see -01). The capture-task-then-read in `timebase_now_us` is the same pattern nrfx uses.
- Drift: there is no drift or true-rate estimator on the device or the host (only the boot capture log). With ~+0.13 % AFE clock error, the host's nominal spacing inside a 6-sample batch is off by <=6.5 us at 1 kSPS and <=26 us at 250 SPS, because each batch header carries the full 64-bit first-sample stamp. That is negligible.
- PPI/GPIOTE: one GPPI channel carries the DRDY event to TIMER1 CAPTURE0 and SPIM3 START. The attach is idempotent, so the fixed item holds. GPIOTE and PPI channels come from the shared allocators, which mask out MPSL's. There is no GPIOTE CPU interrupt. `capture_edge_init` (IMU) is called once, so nothing leaks.
- Register access: the gate disables capture and SPI start together. `AFE_XFER_SETTLE_US` (500 us) is enough, because a transfer lasts ~27 us, and a thread cannot resume while an unmasked priority-1 END ISR is pending, so the last frame is always consumed first. CS is framed per command at 1 MHz, and resume re-points DMA at the frame buffers, so the M4 stale-sample fix holds on that path (not on stream start, -08). Stream stop gates off and waits before SDATAC/STOP.
- Rate change: only CONFIG1 is rewritten (with readback); the full configure is a fallback; one retry. Filters are redesigned for the new rate. The DC shift scales from 9 to 15 with rate (0.08 Hz corner). `SETTLE_MAX_S * rate` fits in uint32. The fixed items "rate change reset the AFE" and "PPI slot attached once" hold.
- Filter hand-off (`submit`/`process`): the POSTED→TAKEN→IDLE CAS plus `req_done` has no lost wakeup and no double apply, and the timeout/late-take branch is correct. `applied_seq` stays right even when the next frame is rejected (`st_seq--` restores it).
- Concurrency: every pipeline_*/ads1299_* mutator runs on the command thread only (priority 6, below DSP 2, TX 4, IMU 4). The only blocking call on the DSP path is `batch_lock`, which has short holders and priority inheritance. `stream_send` is K_NO_WAIT. `k_thread_join(500 ms)` cannot realistically time out today. Caveat: its result is ignored, so a sink that ever blocked longer would make `pipeline_start` call `k_thread_create` on a live thread.
- FPU and stacks: floats appear only in threads (DSP, main, command), never in the ISRs; `CONFIG_FPU_SHARING=y`. chain.c, dsp.c and mains.c contain no LOG_ calls, so the 2048-byte DSP stack is not exposed to immediate-mode formatting. ISR stack 2048.
- Logging vs ISR latency: immediate mode here takes no interrupt lock (`CONFIG_LOG_IMMEDIATE_CLEAN_OUTPUT` unset, so no `process_lock`; sync-mode RTT writes use `WriteSkipNoLock`), so logging delays threads, never the END ISR.
- Boot: an absent IMU (-ENODEV) and an absent AFE are both tolerated. The VDD read and self-test run at boot only (the no-battery-sense item is known). LEDs and supply code are fine.
- Flash/RAM: 250,160 B flash (53 % of the 464 KB slot0) and 99 KB static RAM (39 %), so the MCUboot dual-slot budget has room (see -14 for the integration itself).

## Multiplatform / shared-core implications for this area
- Sequence numbers are assigned in the DSP thread, after the ring, so they count *processed* frames, not conversions. Every loss before that point (-02, -03, -04, -08) is invisible in seq. If seq came from the DRDY count in the ISR, every loss would be a plain seq gap, and every app could share one simple rule.
- The shared core should own the timing model: a running fit of (seq, DRDY ts) that rejects half-range jumps (-01), estimates the true rate and drift (no one does today, despite the goal), and applies the AFE decimation delay (-05) plus the on-device chain group delay. The device must publish those delays (GET_INFO) so all four apps apply the same correction.
- Flags are per DATA batch (6 samples). A shared parser cannot place an OVERRUN/SETTLING transition inside a batch. Add a per-sample flag byte or a "first flagged index" field before multiple apps depend on the current format.
- Batch size (6), ring depth (128) and TX queue depth (32) are hard-coded for BLE at <=1 kSPS. At 16 kSPS over USB they must scale with rate, so the protocol should let the host negotiate or discover batch size rather than assume 6.
- timebase_math.h and ringbuf.c are host-testable C with ztest suites. They fit the shared core's test corpus, once the missing pre-wrap/ISR-early case (-01) is added.
