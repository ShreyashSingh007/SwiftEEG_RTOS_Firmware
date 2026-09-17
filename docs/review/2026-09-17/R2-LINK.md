# R2-LINK — IMU driver, BLE, USB CDC, stream TX queue/thread

Repo HEAD f135b9a (m1-bringup). READ-ONLY review.

## Progress
Files finished: stream.c/.h, ble.c/.h, usb.c/.h, imu.c/.h, prj.conf, usb_only.conf, command.c (reply/RX/CMD ranges), timebase.c (capture), README 415-575/676-694/726-818/964-995; SDK att.c, gatt.c, conn.c, hci_core.c, l2cap.c, gap_svc.c, usbd_cdc_acm.c, ring_buffer, lsm6dsv16x_reg.h, SDC Kconfig. Findings 01-11 written.

## Findings

### R2-LINK-01 [critical] Frames never adapt to the negotiated ATT MTU: at iOS/macOS MTU 185, three of four encodings stream nothing over BLE, and nobody is told
- Category: architecture / robustness (multiplatform blocker)
- Where: src/transport/stream.c:29-30, 160-161, 182-199; src/imu/imu.c:73-77, 537-547; src/transport/ble.c:298-308; src/main.c:295-296
- Evidence:
  - `#define BATCH_SAMPLES 6` / `#define BATCH_SAMPLES_BOTH 3` (fixed, sized for "a 247-byte MTU")
  - deliver(): `if (room != 0 && len > room) { atomic_inc(&ble_too_big); }` - the frame is discarded for BLE, no fallback/split.
  - imu.c: `#define FRAME_SAMPLES_MAX 17` "(244 - 10 protocol - 24 header) / 12 = 17".
  - main.c:295-296 logs `ss.queue_dropped, ss.ble_dropped` only; `ble_too_big` is never logged, and stream_get_stats() has no caller other than that RTT line (no command exposes it). usb_only.conf:3-6 says RTT is unusable while BLE is on.
- Arithmetic (PROTO_OVERHEAD = 8 header + 2 CRC = 10, proto.h:41; notification room = MTU-3):
  - DATA RAW_I24 = 10+16+6*24 = 170 B; UV_F32 / RAW_I32 = 10+16+6*32 = 218 B; RAW_UV = 10+16+3*56 = 194 B.
  - IMU = 10+24+12n B -> fits 182 B only for n <= 12. drain() routinely emits 11-12 samples at 480 Hz and 960 Hz (reads land ~24 ms / ~12 ms after the previous one) and 13-17 on any late poll or catch-up read.
- Failure scenario: iPhone/Mac central negotiates ATT MTU 185 (room 182). Host selects ENC_UV_F32, ENC_RAW_UV (the M6 on-device-chain mode) or RAW_I32: every DATA frame is counted in `ble_too_big` and dropped -> zero EEG over BLE while USB works; IMU at 480/960 Hz loses every frame with n >= 13. A central that never raises the MTU (e.g. an Android app that never calls requestMtu() -> MTU 23, room 20) gets no DATA and no IMU frames at all, and any RSP > 20 B fails too. The host sees silence, not a gap (a fully dropped stream has no gaps to count), and the only counter of the cause is never surfaced.
- Confidence: confirmed-by-reading (sizes, drop path, missing reporting); platform MTU values per brief (iOS 185, macOS sometimes lower) - likely.
- Fix sketch: register `bt_gatt_cb.att_mtu_updated` and keep an atomic `ble_room`; when a batch opens, `batch_limit = MIN(enc_batch(enc), (ble_room - 26) / stride)` (>=1), and split IMU frames at `(ble_room - 34) / 12`. Report ble_too_big / ble_dropped / queue_dropped / usb bytes_dropped to the host (GET_STATS or a periodic EVT).

### R2-LINK-02 [high] The 3-ask interval budget is spent on rejections and on the host's 5 s deferral; iOS/macOS are only ever asked for a value outside Apple's limits
- Category: bug / robustness (multiplatform)
- Where: src/transport/ble.c:139-144, 153-156, 179-180, 225-238. SDK: zephyr/subsys/bluetooth/host/conn.c:3821-3833, conn.c:2294-2310 + 2336, hci_core.c:2041-2083
- Evidence:
  - ble.c: `.interval_min = 6, /* 7.5 ms */ .interval_max = 12, /* 15 ms */`; `if (interval > FAST_INTERVAL_UNITS && slow_link_asks < SLOW_LINK_MAX_ASKS) { slow_link_asks++; ... bt_conn_le_param_update(conn, &fast_param); }`
  - conn.c:3823-3832: until the peripheral update timer has fired (`BT_CONN_PERIPHERAL_PARAM_UPDATE`), `bt_conn_le_param_update()` only stores the params and returns 0; deferred_work sends them after CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT = 5000 ms (build .config:485). So the "request on connect" goes out 5 s after connect.
  - hci_core.c:2041-2082: `bt_conn_notify_le_param_updated(conn)` is called for every LE Connection Update Complete, including a failed one (`if (!evt->status) {...}` closes before it), with the unchanged interval.
- Failure scenario: (a) iPhone/Mac central at its default ~30 ms. At t=5 s the stack sends 7.5-15 ms. Apple's accessory guidelines require Interval Min >= 15 ms and Interval Max >= Interval Min + 15 ms, so iOS rejects -> Update Complete with error -> `le_param_updated(24)` -> re-ask (same non-compliant request) -> rejected, three times within about a second. The link stays at 30 ms for the whole session, and a later genuine slowdown is never re-asked. At 30 ms, 1000 SPS RAW_I24 + IMU 240 Hz needs ~6.3 notifications per connection event (167 + ~42 per s) and RAW_UV + IMU ~11.3, against ~3.1 / ~5.6 at 15 ms. (b) Windows/Android: any central-initiated update inside the first 5 s (common around service discovery) consumes an ask while nothing is sent; a procedure collision (0x23/0x2A) also consumes one. The budget meant for the documented mid-session 45 ms case (README 567-574) can be gone before streaming starts.
- Confidence: confirmed-by-reading (deferral, callback on failure, ask accounting); iOS rejection of min < 15 ms is likely (Apple Accessory Design Guidelines, not checkable in the SDK).
- Fix sketch: count an ask only when the callback reports a successful change (track the interval that was in force when the request went out and ignore an identical report), reset the budget on a fast interval, and after a rejection fall back to {min 12, max 24} (15-30 ms), which Apple accepts; do not re-ask before the 5 s timer (or set CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT low, e.g. 1000).

### R2-LINK-03 [high] GAP PPCP and the stack's own automatic update still advertise 30-50 ms with a 420 ms supervision timeout
- Category: contract-drift / robustness
- Where: prj.conf:67-104 (no CONFIG_BT_PERIPHERAL_PREF_*); build/zephyr/.config:475-480; SDK zephyr/subsys/bluetooth/services/gap_svc.c:173-176; conn.c:2022-2035, conn.c:2311-2324
- Evidence:
  - effective config: `CONFIG_BT_GAP_AUTO_UPDATE_CONN_PARAMS=y`, `CONFIG_BT_GAP_PERIPHERAL_PREF_PARAMS=y`, `CONFIG_BT_PERIPHERAL_PREF_MIN_INT=24`, `MAX_INT=40`, `LATENCY=0`, `TIMEOUT=42` (30-50 ms, 420 ms).
  - gap_svc.c:174: `BT_GATT_CHARACTERISTIC(BT_UUID_GAP_PPCP, BT_GATT_CHRC_READ, ..., read_ppcp, ...)` publishes those values to every central.
  - conn.c:2029-2034 clears `BT_CONN_PERIPHERAL_PARAM_SET` when the central already satisfies the app's pending request (interval in 6..12, latency 0, timeout == 400); deferred_work then takes the `else if (CONFIG_BT_GAP_AUTO_UPDATE_CONN_PARAMS)` branch and requests the PPCP values (conn.c:2315-2321).
- Failure scenario: (1) A central that adopts PPCP (BlueZ stores it as the device's connection parameters for later connections; possibly Windows) runs the link at 30-50 ms with a 420 ms supervision timeout: at 45 ms the 1000 SPS raw+uV stream sheds batches (the README's Windows episode sat at exactly 45 ms), and a 420 ms timeout drops a head-worn link on a brief body-shadowing fade. (2) A central that grants 7.5-15 ms / latency 0 / 4.00 s within 5 s of connecting gets a stack-initiated request for 30-50 ms / 420 ms at t=5 s, which the firmware then has to undo with a re-ask (and R2-LINK-02's budget).
- Confidence: confirmed-by-reading (values and code paths); central adoption of PPCP likely (BlueZ) / speculative (Windows).
- Fix sketch: in prj.conf set `CONFIG_BT_PERIPHERAL_PREF_MIN_INT=12`, `CONFIG_BT_PERIPHERAL_PREF_MAX_INT=24` (or 6/12 for non-Apple), `CONFIG_BT_PERIPHERAL_PREF_LATENCY=0`, `CONFIG_BT_PERIPHERAL_PREF_TIMEOUT=400`, consistent with fast_param; or turn off CONFIG_BT_GAP_AUTO_UPDATE_CONN_PARAMS so the only request sent is the firmware's.

### R2-LINK-04 [high] One TX thread feeds both links, so BLE backpressure stalls and drops the USB stream
- Category: architecture / performance
- Where: src/transport/stream.c:182-199 (deliver), 201-214 (tx_entry), 219-237 (stream_send); SDK zephyr/subsys/bluetooth/host/att.c:814-830
- Evidence:
  - `deliver()`: `(void)usb_transport_write(frame, len);` then `ble_transport_send_stream(frame, ...)`, both on the single `stream_tx` thread.
  - att.c:823-830: for any caller that is not the system workqueue, `timeout = K_FOREVER;` then `bt_l2cap_create_pdu_timeout(&att_pool, 0, timeout)`. bt_gatt_notify() therefore blocks until an ATT buffer (CONFIG_BT_ATT_TX_COUNT=10) frees, or until the link is torn down.
  - `k_msgq_put(&tx_queue, &tx, K_NO_WAIT) != 0 -> atomic_inc(&tx_dropped); return false;` - a full queue drops the frame for every link.
- Failure scenario: recording over USB (the "reliable" link) while a BLE central is also subscribed, a combination the README advertises ("both links carry byte-identical frames"). (a) BLE link slower than production (1000 SPS raw+uV at a 30-45 ms interval, iOS/Windows cases in R2-LINK-02/03): the TX thread spends its time blocked in notify, the 32-frame queue stays full, and the USB recording loses the same batches BLE loses, although USB has ~10x the capacity. (b) RF fade: notify blocks until the supervision timeout (4 s requested, central may choose more). USB receives nothing for that time and everything after the first 32 frames (~190 ms at 1000 SPS RAW_I24) is dropped for both links.
- Confidence: confirmed-by-reading.
- Fix sketch: decouple the links. Write USB directly from stream_send() (usb_transport_write is already non-blocking and serialised by its own mutex), or give BLE its own queue/thread so only BLE sheds frames when BLE is slow.

### R2-LINK-05 [medium] USB frames outlive the host session: DTR stays latched after unplug and nothing flushes on reopen, so the next session starts with stale DATA/IMU/RSP frames
- Category: bug / robustness
- Where: src/transport/usb.c:137-147, 205-228; src/transport/stream.c:184-186; SDK zephyr/subsys/usb/device_next/class/usbd_cdc_acm.c:472-483, 536-537, 363-371, 374-382
- Evidence:
  - usb.c: `(void)uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr); return dtr != 0;` is the only "connected" test.
  - usbd_cdc_acm.c: `line_state_dtr` is written only in `cdc_acm_update_linestate()` from SET_CONTROL_LINE_STATE (lines 480-483); `usbd_cdc_acm_disable()` (374-382) and suspend do not clear it. On re-enable, `usbd_cdc_acm_enable()` immediately schedules any pending TX FIFO data (368-371).
  - No flush of `usb_tx_rb` on a DTR edge or bus reset anywhere in usb.c.
- Failure scenario: (1) Cable pulled while the port is open and the stream on: DTR stays 1, deliver() keeps writing until the 8 kB ring and the class FIFO fill, then counts every later byte as dropped (a meaningless, unbounded `bytes_dropped`). On re-plug, the first bytes the host reads are those stale frames, minutes or hours old. (2) Host app closes the port mid-stream without reading to the end: up to 8 kB (~280 ms of RAW_I24 at 1 kSPS, ~47 frames) stays queued and is delivered first to the next opener. Both cases include RSP frames. A reply carries a device-side `rsp_seq`, not the request's seq (command.c:170), so a stale reply to e.g. GET_CONFIG cannot be told from the answer to the new request. DATA/IMU `seq` and `ts` jump backwards at session start.
- Confidence: confirmed-by-reading.
- Fix sketch: track DTR in usb_transport_is_connected(); on a 0->1 edge (and on unplug/bus reset via a usbd message callback) drop the ring contents from the CDC work-queue context before re-enabling TX. Make RSP frames echo the command frame's seq so any host can match replies.

### R2-LINK-06 [medium] Replies bypass the TX queue and overtake data frames produced before the command
- Category: contract-drift / race
- Where: src/transport/command.c:177-204 (respond, sent directly from the command thread); src/transport/stream.c:219-237, 239-267, 379-388; README.md:678-685 (host uses a reply as the boundary)
- Evidence:
  - respond(): `ble_transport_send_event(rsp_buf, n)` / `usb_transport_write(rsp_buf, n)` straight from the command thread, while DATA/IMU/EVT frames wait in `tx_queue` (up to 32) for the stream TX thread.
  - stream_enable(false): `flush()` puts the half batch into the queue; the STREAM_STOP reply is sent immediately after, ahead of it.
  - README 682-684: the app drops every EEG frame "until" the GET_CONFIG answer after a rate command, then treats what follows as new-rate data.
- Failure scenario: the link is backlogged (queue holding ~32 frames, R2-LINK-04 conditions) and the host changes rate 1000 -> 500 SPS. SET_RATE takes 40-110 ms, and GET_CONFIG is answered at once. On USB the RSP bytes enter the ring ahead of the queued old frames; on BLE the notification enters the single ATT bearer queue ahead of them. The host sees the boundary reply and then up to ~30 frames sampled at 1000 SPS, which it can admit as 500 SPS data (wrong rate for filtering, plotting and recording). The same inversion hides the tail of a stream after the STREAM_STOP reply.
- Confidence: confirmed-by-reading (device ordering); host impact likely (depends on whether the app also checks seq/ts continuity).
- Fix sketch: before replying to STREAM_STOP/SET_RATE/any acquisition restart, wait (bounded, e.g. 500 ms) until `k_msgq_num_used_get(&tx_queue) == 0` and the TX thread is idle, or send those replies through the same queue. Echo the command seq in RSP.

### R2-LINK-07 [medium] No pairing, bonding or encryption: any central in range can connect, read the EEG in plaintext and reconfigure the device, and with one connection slot it can lock the owner out
- Category: security
- Where: prj.conf:67-104 (no CONFIG_BT_SMP; effective .config has none); src/transport/ble.c:104-125 (`BT_GATT_PERM_WRITE`, CCC `BT_GATT_PERM_READ | BT_GATT_PERM_WRITE`), 158-181, 204-223 (re-advertises connectable after every disconnect), prj.conf:96 `CONFIG_BT_MAX_CONN=1`
- Evidence: no `_ENCRYPT`/`_AUTHEN` permissions on any attribute; `control_write` passes any bytes to the command decoder; connectable advertising resumes whenever no central is attached.
- Failure scenario: the owner's host drops the link (walks away, app restart). Any nearby phone or script connects first (MAX_CONN=1, so the owner's host cannot reconnect while it stays), subscribes, and receives the raw EEG, or issues SET_RATE / SET_FILTER / SET_INPUT / test-signal commands that silently change the recording configuration. A passive sniffer following the connection also reads the EEG. For a head-worn biosignal device this is personal health data.
- Confidence: confirmed-by-reading.
- Fix sketch: enable CONFIG_BT_SMP (+ CONFIG_BT_SETTINGS for bonds), LE Secure Connections, require `BT_GATT_PERM_WRITE_ENCRYPT` on Control and the CCCs, and use a filter accept list after bonding. Decide this before the native apps are written, because bonding changes the connect flow on iOS/Android/Windows (pairing prompts on first encrypted access, CCC persistence across reconnects).

### R2-LINK-08 [low] The BLE reply retry loop cannot do what its comment says: buffer exhaustion blocks forever inside bt_gatt_notify, and the only -ENOMEM it sees (reply larger than the MTU) is retried pointlessly
- Category: robustness / docs-drift
- Where: src/transport/command.c:39-45, 177-201; src/transport/ble.c:329-336; SDK att.c:802-806, 814-825, 3106-3116; gatt.c:2423-2428
- Evidence:
  - command.c: `if (err != -ENOMEM && err != -ENOBUFS && err != -EAGAIN) break; k_msleep(RSP_RETRY_MS);` with the comment "a notification refused for want of a buffer waits for room - and says so if it never gets it".
  - att.c:823-825: from the command thread `timeout = K_FOREVER` for the att_pool allocation, so a missing buffer never returns -ENOMEM. -ENOMEM comes only from `bt_att_create_pdu()` finding no channel whose MTU fits (att.c:3106-3116), which no retry can fix. -EAGAIN needs change-unaware tracking (BT_GATT_CACHING, not enabled). -ENOBUFS is never returned.
- Failure scenario: (a) link fade while a reply is pending: the command thread blocks until the supervision timeout. Meanwhile no command from either link, USB included, is processed. When the stream TX thread (prio 4) is also waiting, it wins each freed buffer, and att_pool buffers are returned one per system-workqueue item (att.c:323-354), so replies under saturation wait for bursts. (b) MTU 23 central: GET_CONFIG (39-byte payload -> 49-byte frame) fails 25 times over 100 ms and is logged, while the host sees an ignored command.
- Confidence: confirmed-by-reading.
- Fix sketch: drop the retry loop; bound the wait instead (send replies from a context with K_NO_WAIT semantics and a short retry, or pre-check `len <= ble_transport_max_payload()` and return a CMD_ETOOBIG status); correct the comment.

### R2-LINK-09 [low] Mains EVT frames travel on the Stream characteristic and through the data queue, contrary to the BLE contract
- Category: contract-drift
- Where: src/transport/ble.h:40-44; src/transport/stream.c:339-358, 182-199; src/transport/ble.c:315-327
- Evidence: ble.h says the Event characteristic carries "command replies and telemetry. Kept off the Stream characteristic so a host can subscribe to replies without having to receive sample data". But `stream_on_mains()` -> `stream_send()` -> `deliver()` -> `ble_transport_send_stream()` -> `attrs[3]` (Stream). Only RSP frames use `ble_transport_send_event()`.
- Failure scenario: a host (or a second, lightweight client such as a watch/phone status view) that subscribes only to Event never receives mains events. EVT frames are also dropped with data when the 32-frame queue is full or the link is slow, so the host's copy of "where the notch is aimed" can go stale without anything but an EVT seq gap to show it.
- Confidence: confirmed-by-reading.
- Fix sketch: send EVT frames with `ble_transport_send_event()` (own small queue or the command thread), or change the documented contract so every host library parses EVT on the Stream characteristic.

### R2-LINK-10 [low] A failed IMU configuration is reported as applied: `active` is overwritten on error and the command already answered OK
- Category: robustness
- Where: src/imu/imu.c:286-320, 568-588, 696-727; src/transport/command.c:453-468, 528-533
- Evidence: imu_entry: `const int err = apply(&next); ... active = next;` unconditionally; `if (err) LOG_ERR(...)` only. imu_configure() returns 0 once the request is queued, so CMD_SET_IMU answers CMD_OK before apply() runs. apply() also changes `batch` and timing state before any register write can fail, and ORs errno values (`err |= reg_update(...)`).
- Failure scenario: an SPI error after `reg_write(REG_FIFO_CTRL4, FIFO_MODE_BYPASS)` / ODR-off leaves the sensor stopped. `active.enabled` is still true, GET_CONFIG reports "on, 240 Hz", drain() returns early forever (`words < wtm`), and motion frames silently stop with only an RTT line (unreadable while BLE is on).
- Confidence: confirmed-by-reading (failure needs a bus error, so rare).
- Fix sketch: on error keep the previous `active` (or mark enabled=false and report it), retry apply() a bounded number of times, and surface the failure (EVT or a status bit in GET_CONFIG's IMU flags).

### R2-LINK-11 [low] stream_tx stack (2048) is a guess and its deepest path is exactly the common disconnect-while-streaming case with immediate-mode logging (speculative)
- Category: memory
- Where: src/transport/stream.c:162, 201-214 (a 246-byte `struct tx_frame` on the stack); prj.conf:8 `CONFIG_LOG_MODE_IMMEDIATE=y`, prj.conf:38 `CONFIG_RESET_ON_FATAL_ERROR=n`; SDK att.c:3075-3077 (`LOG_WRN("Not connected")` in att_get), .config `CONFIG_BT_ATT_LOG_LEVEL=3`
- Evidence: when a disconnect lands while the TX thread is inside `bt_gatt_notify()` (blocked on att_pool or about to send), the call continues into `bt_att_send()` -> `att_get()` on a non-connected conn and formats a WRN log on the stream_tx stack: tx_frame (246 B) + deliver/notify/gatt/att frames + immediate-mode log formatting. The README already records two stack overflows from immediate-mode logging (BT RX WQ at 1024, UDC at 512). There is no measurement of this thread's high-water mark (THREAD_STACK_INFO is on, but nothing reads it).
- Failure scenario (unverified magnitude): an MPU stack-guard fault on disconnect under load; with RESET_ON_FATAL_ERROR=n the headset halts until power-cycled (no button).
- Confidence: speculative.
- Fix sketch: log `k_thread_stack_space_get()` for stream_tx / imu in the health line after a forced disconnect at 1000 SPS; move `struct tx_frame tx` off the stack (static, the thread is the sole consumer); size from the measurement with margin.

