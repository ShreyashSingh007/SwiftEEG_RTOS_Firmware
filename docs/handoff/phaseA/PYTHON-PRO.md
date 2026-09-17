# Phase A - python-pro: R4-HOST-01..07

Scope: `tools/swifteeg_app.py`, `tools/swifteeg_link.py` (branch m1-bringup).
Work folder: `scratchpad/phaseA/python-pro-work/`.

## Progress log

- 2026-09-17 started. Reading R4-HOST.md, then the two files.
- Read R4-HOST.md, swifteeg_link.py, swifteeg_app.py, proto_ref decode, README 576-703, reviewer harnesses.
  Contract facts used: GET_INFO reply = op,status,channels,0,rate16 (command.c:343); CHnSET bits PD/gain/SRB2/mux;
  verify_chain.py reads link.connected, link.status strings, decode_data (kept compatible).

## Plan (before edits)
- link: FrameParser validates header at 8 bytes, drops 1 byte on bad header/CRC; `bad` counts damaged stretches;
  decode_data length/encoding checks; decode_config adds power_down/srb2; decode_info; explicit link state
  (connecting/connected/lost/closed) + reason; frames stamped host_time; BLE drains queue before stop, scan cancellable,
  stop checked after connect; close(timeout) joins; `finished`; _self_test().
- app: alert label (persistent red); sync state (GET_CONFIG epoch FIFO, 1 s retry, alert after 5); DATA dropped until
  synced; unsupported rate = error; measured period (median of batch periods) + >2 % alert; rows timed by it;
  flags + missing_before columns (seq and ts continuity); lost = samples; per-frame isolation; recording I/O error
  stops recording loudly; segments + JSON sidecar; config commands / host filter changes / stream break end the
  segment, next opens on a current GET_CONFIG answer; link loss closes segment with note, resets buttons;
  Connect waits for the old link thread; window close waits (bounded) for the link.

## swifteeg_link.py - done (R4-HOST-01, 04, 05 decode side, 06 link side)
Changes:
- FrameParser: header (version, type, len <= MAX_PAYLOAD) judged at 8 bytes; bad header or CRC failure drops ONE byte
  and rescans. `bad` now counts damaged stretches (once per run of skipped bytes). `discard()` for BLE leftovers.
- decode_data: None for payload shorter than header, short body, or unknown encoding (was: struct.error / read as i32).
- decode_config: adds power_down and srb2 per channel (CHnSET bits 7 and 3). New decode_info (GET_INFO reply).
- Link state: LINK_CONNECTING / CONNECTED / LOST / CLOSED + `reason`; `connected` is now a read-only property;
  state set before the status message; closed stays closed, lost stays lost. Frames carry `host_time` (time.time()).
- UsbLink: reader thread closes the port itself on exit; read/write failure -> LOST (not when closing).
- BleLink: queued commands drained before stop is honoured; scan runs as a task and is cancelled on close;
  stop checked after connect (no subscribe, still writes what was queued); send ignored once lost/closed.
- Link.close(timeout=2.0) joins the thread (0 = don't wait); `finished` property.
- _self_test() (python tools/swifteeg_link.py): clean stream in 1..4096-byte pieces, 0xFFFF length, stray SOF (+plausible
  version/type), 24 truncations (inside/end, several different frames after, payloads compared), bad CRC,
  400 random start offsets, BLE leftover discard, decode_data short/unknown payloads, link state rules.

Evidence:
- `python tools/swifteeg_link.py` -> OK (0.4 s); proto_ref self-test OK.
- Same self-test with the OLD parser from git swapped in -> fails ("length 0xFFFF: 10 frames, want 119").
- Reviewer harnesses on new parser: parser_test.py A 0xFFFF lost 1 (was 338), B mid-frame starts lost 0 (was mean 35),
  C CRC lost 1, D stray 0xA5 lost 0 (was 243), E 7 bytes lost 1 (was 148); parser_test2 >100 lost in 0 % of starts
  (was 12.5 %); connect_reply_test GET_CONFIG reply swallowed 0/500 at 250 and 1000 SPS, 30 and 100 ms (was ~10 %).
- ble_close_test.py (fake bleak): STREAM_STOP written 100/100 (was 0/100); close during scan -> no connect, thread
  ended (was connected 0.44 s later); disconnect+connect -> old thread not alive while new scan runs (was both).
- python-pro-work/ble_states_test.py: not found -> LOST; drop -> LOST "BLE link dropped"; write failure -> LOST;
  close while connecting -> CLOSED in 0.21 s, no subscribe, queued STREAM_STOP written, then disconnect;
  close during 5 s scan -> finished in 0.03 s; 21 queued then close -> all 21 written, send after close ignored.
- python-pro-work/usb_states_test.py (fake serial): junk then frame -> frame with host_time, bad 1; close -> port closed
  by reader thread, STREAM_STOP written first; unplug -> LOST with reason; write failure -> LOST.
