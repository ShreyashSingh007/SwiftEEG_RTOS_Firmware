# R5-CONTRACT — firmware/host byte-level wire contract

## Progress
- Read in full: src/proto/proto.h, proto.c; src/transport/command.h, command.c,
  stream.c; src/imu/imu.c, imu.h; src/pipeline/pipeline.h; tools/proto_ref.py,
  tools/swifteeg_link.py.
- Targeted greps (not owned, values needed for the contract table): stream.h
  (STREAM_ENC_*, STREAM_EVT_MAINS), dsp.h (DSP_MAX_SECTIONS), ads1299.h
  (ADS1299_CHANNELS, ADS1299_MUX_*, ADS1299_GAIN_*, ADS1299_CAL_FREQ_*,
  ads1299_get_channels doc).
- Review complete.

## Exclusion list (already reported by other reviewers — do not repeat)
- R3-DSP-01: stream decoder payload pointer overwritten before caller reads (codec payload overwrite)
- R2-LINK-05/06: USB frames outlive session / replies bypass TX queue (incl. command-seq echo issues)
- R2-LINK-09: mains EVT frames on Stream characteristic instead of Event characteristic
- R1-ACQ-03/04: OVERRUN flag on wrong sample; seq-- silently desyncs on bad-status frame rejection
- R4-HOST-01: FrameParser trusts unvalidated length field (USB false-SOF -> 65KB swallow)
- R3-DSP-02: CMD_SET_RATE replies OK before validating/applying rate

## Findings

### R5-CONTRACT-01 [critical] No working protocol-version or capability negotiation; a version mismatch is indistinguishable from a dead device
- Category: contract-drift / architecture
- Where: src/proto/proto.c:210-219 (`try_extract`), proto.c:102-104 (dead `PROTO_ERR_VERSION` path); src/transport/command.h:19, command.c:343-354 (CMD_GET_INFO); tools/swifteeg_link.py `FrameParser.feed` (~296-300), proto_ref.py:82-83
- Evidence:
  - proto.c: `if (st->buf[1] != PROTO_VERSION || !type_is_valid(st->buf[2]) || payload_len > PROTO_MAX_PAYLOAD) { resync(st); continue; }` — a wrong `ver` byte is treated as "not a plausible header" and silently resynced. `proto_decode()`'s own `PROTO_ERR_VERSION` (proto.c:102-104) can only be reached by a caller that skips this pre-filter; command.c's `cmd_entry` never does (it only calls `proto_stream_push`/`poll`), so that return code is dead on the one path actually used.
  - command.c CMD_GET_INFO: `const uint8_t info[4] = { ADS1299_CHANNELS, 0, (uint8_t)(sps & 0xFFu), (uint8_t)(sps >> 8) };` — byte[1] is a bare literal `0`, not documented as reserved, not a version or capability field. No other opcode/reply carries a firmware build, protocol version, or feature list either.
  - swifteeg_link.py: `try: out.append(proto_ref.decode(raw)) except Exception: self.bad += 1` — `proto_ref.decode`'s distinct `ValueError(f"bad version {buf[1]}")` (proto_ref.py:83) is caught generically and merged into the same `bad` counter as a CRC failure or a truncated frame; nothing reports "version mismatch" specifically.
- Failure scenario: any one of the four planned native apps built against a later protocol revision (or an older app talking to updated firmware) sends or receives frames whose `ver` byte differs from the peer's `PROTO_VERSION`. Firmware answers with nothing at all (no RSP, no EVT, only an internal `resyncs` counter no host can see); the host sees frames vanish and `bad_frames` climb, exactly as corruption would. Nothing in the wire contract lets an app ask "what version/features does this device speak" before it sends anything, or interpret silence as "wrong version" instead of "dead/wedged" (which is already a real failure mode per section 7 of the brief).
- Confidence: confirmed-by-reading
- Fix sketch: give CMD_GET_INFO a real version/capability payload (protocol version, firmware build, feature bitmap) in place of the wasted byte[1]; on a `ver` mismatch, reply with a distinct error (or at least log/count it separately from CRC errors) instead of silent resync; have the host surface version mismatches distinctly from generic bad frames.

### R5-CONTRACT-02 [medium] `respond()` silently truncates any RSP payload over 46 bytes and still reports CMD_OK
- Category: bug / contract-drift
- Where: src/transport/command.c:155-176 (`respond`), command.c:504-561 (CMD_GET_CONFIG, 37-byte `extra`)
- Evidence: `uint8_t payload[48]; payload[0]=opcode; payload[1]=status; size_t len = 2; if (extra != NULL && extra_len <= sizeof(payload) - 2) { memcpy(&payload[2], extra, extra_len); len += extra_len; }` — when `extra_len > 46` the whole `if` is skipped: no memcpy, `len` stays 2, no log, no status change. `proto_encode(PROTO_TYPE_RSP, ..., payload, len, ...)` then emits a perfectly valid, CRC-correct RSP `[opcode, CMD_OK]` with the caller's data silently gone.
- Failure scenario: CMD_GET_CONFIG already sends 37 bytes via this path — 9 bytes below the 46-byte ceiling. tools/swifteeg_link.py's `decode_config()` is explicitly written to keep growing (`if len(p) >= 15: ... if len(p) >= 39: ...`, comment "Older firmware sends less"). The next field added to `cfg[]` that pushes it past 46 bytes total is not rejected at compile time or caught at runtime: the host gets a bare `[GET_CONFIG, OK]` reply, `decode_config()` quietly returns the oldest, smallest dict, and nothing signals the device tried to say more.
- Confidence: confirmed-by-reading (mechanism verified directly; not yet triggered by current call sites, whose largest `extra_len` is 37)
- Fix sketch: size `payload[]` from `PROTO_MAX_PAYLOAD`/`rsp_buf` instead of a fixed 48, or turn the `extra_len` bound into an assert/`LOG_ERR` — a truncated reply must never carry `CMD_OK`.

### R5-CONTRACT-03 [medium] The one per-channel state readback drops the power-down and SRB2 bits
- Category: contract-drift
- Where: src/transport/command.h:23-24 (CMD_SET_CHANNEL doc), command.c:391-405 (handler), command.c:511,515,526 (GET_CONFIG `chset`); src/afe/ads1299.h:269-270 (`ads1299_get_channels` = "Current CHnSET contents"); tools/swifteeg_link.py `decode_config` gains/mux lines (~238-239)
- Evidence: CMD_SET_CHANNEL independently sets power-down and SRB2: `ads1299_set_channel(f->payload[1], f->payload[2], f->payload[3], pd, srb2)`. The only readback of per-channel state, CMD_GET_CONFIG, copies the raw ADS1299 CHnSET register byte verbatim — ads1299.h documents `ads1299_get_channels()` as "Current CHnSET contents," the real register, whose standard layout is PD(bit7):GAIN(bits6:4):SRB2(bit3):MUX(bits2:0) (confirmed by the exact match of `ADS1299_GAIN_*` = 0x00-0x06 against the host's `(c >> 4) & 0x07`, and `ADS1299_MUX_*` against `c & 0x07`). `decode_config()` computes only `cfg["gains"]` and `cfg["mux"]` from each `chset` byte; bit 7 and bit 3 are never named.
- Failure scenario: a host sets a channel to power-down or SRB2 reference, then calls GET_CONFIG later (e.g. after reconnect) to redraw its UI from device truth — the documented purpose of this command (command.c:505-510: "a failed write shows up as a wrong control, not a lie"). The shared decoder hands back gain and mux only; PD/SRB2 state must be re-derived by hand from the raw `cfg["chset"]` bytes, whose bit layout is undocumented in the Python reference itself.
- Confidence: confirmed-by-reading
- Fix sketch: decode `pd`/`srb2` (bit7/bit3) in `decode_config()` alongside gain/mux, and document the CHnSET bit layout once, next to `DATA_HDR`/`IMU_HDR`, instead of leaving it to be re-derived from the datasheet per platform.

### R5-CONTRACT-04 [medium] The shared host library has no generic reply-status decoder; three of four status codes are indistinguishable from a garbled reply
- Category: contract-drift / robustness
- Where: tools/swifteeg_link.py `response_seq` (~217-221), `decode_config` (~229); src/transport/command.h:45-48 (status codes)
- Evidence: firmware defines and faithfully sends four distinct status codes (`CMD_OK`/`EBADARG`/`EFAILED`/`EUNKNOWN`, command.h:45-48; assigned throughout command.c via `status_for()` and explicit branches). In swifteeg_link.py the only functions that look at a status byte are `response_seq()` (`if len(payload) >= 6 and payload[1] == STATUS_OK: return struct.unpack_from("<I", payload, 2)[0]` else `None`) and `decode_config()` (`if ... p[1] != STATUS_OK: return None`). Both collapse EBADARG, EFAILED, EUNKNOWN, and a too-short/corrupt frame into the identical `None`/absent result. No function in the file returns "which status code came back" for a plain RSP.
- Failure scenario: a native app sends e.g. CMD_SET_CHANNEL with a value the AFE write rejects (EFAILED) versus an out-of-range argument (EBADARG) — the shared library gives identical nothing either way, so it cannot surface to the user *why* a command failed even though firmware already computed the distinction.
- Confidence: confirmed-by-reading
- Fix sketch: add a small `decode_response(payload) -> (opcode, status)` to swifteeg_link.py that every future app uses, instead of leaving status-byte handling to be reinvented per opcode and per platform.

### R5-CONTRACT-05 [low] The CMD_TEST_SIGNAL / CMD_SET_INPUT calibration-frequency byte has no host-side representation at all
- Category: contract-drift
- Where: src/afe/ads1299.h:103-105 (`ADS1299_CAL_FREQ_*`); src/transport/command.h:18,21 (payload docs), command.c:326,362 (firmware-side default); tools/swifteeg_link.py (whole file — no `CAL_FREQ` symbol)
- Evidence: firmware defines `ADS1299_CAL_FREQ_DIV21 0x00`, `..._DIV20 0x01`, `..._DC 0x03` (0x02 is undefined — a non-contiguous enum) and both CMD_TEST_SIGNAL and CMD_SET_INPUT take this byte as their optional third argument. swifteeg_link.py defines named constants for every other enumerated CMD argument in the file (`MUX_*`, `GAIN_CODES`, `NOTCH_*`, `ENC_*`) but none for calibration frequency.
- Failure scenario: any future app wanting a calibration frequency other than firmware's silent fallback (DIV21) has nothing shared to reference; it hand-copies raw byte values from firmware source, with no indication that 0x02 is not a legal value.
- Confidence: confirmed-by-reading
- Fix sketch: add `CAL_FREQ_DIV21/DIV20/DC = 0x00/0x01/0x03` to swifteeg_link.py next to `MUX_*`.

### R5-CONTRACT-06 [low] A zero-length CMD payload gets total silence instead of CMD_EBADARG, unlike every other malformed-argument case
- Category: robustness
- Where: src/transport/command.c:289-293 (`handle`), command.c:592 (the shared `CMD_EBADARG` fallthrough every other case uses)
- Evidence: `static void handle(const proto_frame_t *f) { if (f->type != PROTO_TYPE_CMD || f->len < 1) { return; } const uint8_t op = f->payload[0]; ...}` — every other argument-length failure in this function falls through to `respond(op, CMD_EBADARG, NULL, 0)` at line 592; a zero-length payload returns before `op` can even be read, so no RSP is sent at all.
- Failure scenario: a CMD frame with a 0-byte payload is legal at the framing layer (`proto_encode`/`proto_decode` both accept `payload_len == 0`). Sending one produces exactly the same silence as a dropped/corrupted frame or a version mismatch (R5-CONTRACT-01) — a host cannot distinguish "malformed command" from "never arrived."
- Confidence: confirmed-by-reading
- Fix sketch: answer with a generic bad-request reply (opcode 0 or similar sentinel) instead of dropping silently, for consistency with the rest of the function's error contract.

### R5-CONTRACT-07 [low] EEG_FLAG_* and PROTO_FLAG_* are hand-duplicated with no static check tying them together
- Category: contract-drift
- Where: src/pipeline/pipeline.h:23-24; src/proto/proto.h:56-60; src/transport/stream.c:256,329 (`batch_flags |= s->flags` passed straight into `proto_encode`)
- Evidence: pipeline.h independently defines `EEG_FLAG_SETTLING 0x01` / `EEG_FLAG_OVERRUN 0x02` ("mirroring the protocol's DATA header" — a comment, not an enforced link); proto.h independently defines `PROTO_FLAG_SETTLING 0x01u` / `PROTO_FLAG_OVERRUN 0x02u`. stream.c never translates between the two namespaces: `struct eeg_sample.flags` (an `EEG_FLAG_*` value) is OR'd straight into `batch_flags`, which becomes the wire frame's `flags` byte. No `BUILD_ASSERT` or shared header ties the two enums' numeric values together.
- Failure scenario: the values match today, so the pass-through is currently correct. If either header is renumbered independently (they live in different subsystems, maintained separately) the wire `flags` byte silently starts carrying the wrong meaning — e.g. SETTLING reported as OVERRUN — with no compiler or runtime diagnostic.
- Confidence: confirmed-by-reading
- Fix sketch: `BUILD_ASSERT(EEG_FLAG_SETTLING == PROTO_FLAG_SETTLING, ...)` (and OVERRUN) next to the pass-through in stream.c, or define pipeline.h's flags as aliases of proto.h's.

## Checked and correct
- Frame header (SOF/ver/type/flags/len/seq/payload/CRC), byte order, and which
  byte range the CRC covers are byte-identical in proto.c and proto_ref.py.
- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, no xorout) is the
  same algorithm on both sides; proto_ref.py's self-test cross-checks it
  against the standard `"123456789" -> 0x29B1` vector and a bitwise reference.
- All 18 CMD opcodes (0x01-0x12) and all 4 status codes (0-3) have identical
  values and names in command.h and swifteeg_link.py.
- STREAM_ENC_RAW_I32/UV_F32/RAW_I24/RAW_UV = 0/1/2/3 (grepped from stream.h,
  which is not in this reviewer's file list but is on the wire) match
  swifteeg_link.py's ENC_* exactly — note the brief's own architecture summary
  (section 3) only mentions 3 encodings; there are really 4 (RAW_I32 exists,
  is the wire default, and is the `default:` case in stream.c's width/encode
  switches).
- ADS1299_MUX_* (0x00/0x01/0x05), ADS1299_GAIN_* (0x00-0x06 for 1/2/4/6/8/12/24),
  ADS1299_CHANNELS (8), and DSP_MAX_SECTIONS (8) all match the host's
  MUX_*/GAIN_CODES/CHANNELS/MAX_SECTIONS exactly.
- DATA header (16 bytes: u64 ts_us as two LE u32 halves, u32 seq, u8 channels,
  u8 encoding, u16 count) and IMU header (24 bytes: u64 ts_us, u32 seq, u32
  period_q8, u16 accel_g, u16 gyro_dps, u8 axes, u8 flags, u16 count) match
  `DATA_HDR`/`IMU_HDR` field-for-field, offset-for-offset.
- All four DATA sample encodings match: per-channel widths (3/4/4/7 bytes),
  and RAW_UV's internal layout (3-byte raw then 4-byte float, not the other
  way round). Raw-sample signedness (24-bit and 32-bit two's complement) and
  sign-extension direction match exactly (`put_i24`/`_i24`).
- CMD_GET_CONFIG's entire 37-byte payload was verified field-by-field against
  `decode_config()`'s four length-gated tiers (>=15/21/29/39 bytes of the full
  RSP): every offset, width, and bit position lines up exactly across all 39
  bytes of opcode+status+cfg.
- CMD_SET_FILTER's variable-length section upload matches `filter_args()`/
  `pack_sections()` field-for-field, including the identical `6 + 20*count`
  total-length formula enforced on both sides.
- CMD_SET_NOTCH argument layout and the NOTCH_HARMONIC/NOTCH_TRACK/
  NOTCH_MEASURED bit flags match.
- The "applies from sample N" encoding used by SET_FILTER/SET_NOTCH/SET_CAR/
  RESET_CHAIN replies (`respond_seq` / `response_seq`: opcode, OK, then a
  little-endian u32) matches.
- EVT_MAINS = 0x01 is the only event ID either side defines (grepped
  stream.h), matches swifteeg_link.py's `EVT_MAINS`; its 10-byte payload
  (id, moved flag, u32 seq, f32 Hz) matches `decode_event()` exactly.
- IMU_FLAG_TIME_ESTIMATED (0x01) / IMU_FLAG_OVERRUN (0x02) match.
- Most opcodes bound-check argument length with `f->len < N` and fail with
  CMD_EBADARG on a too-short frame rather than reading past the payload.

## Multiplatform / shared-core implications for this area
- proto_ref.py + swifteeg_link.py are the only cross-checked specification of
  this wire format that exists. Every gap found here (no version/capability
  discovery, no CAL_FREQ enum, PD/SRB2 dropped on readback, coarse status
  decoding) will be independently reproduced by each of the four native ports
  unless it is fixed in the shared reference before those ports are written.
- Where the contract is fully specified end to end (frame header, DATA/IMU
  headers, GET_CONFIG, SET_FILTER), the two existing implementations agree
  precisely, down to the byte and the bit — a good sign that the format
  itself is mechanically simple to port (plain little-endian, fixed offsets,
  no bit-packing surprises found). The risk is not the wire mechanics; it is
  undocumented knowledge that currently lives only in comments and naming
  conventions split across two languages (e.g. the ADS1299 CHnSET bit layout
  that swifteeg_link.py half-decodes and firmware documents in one line).
- GET_CONFIG's tiered `len(p) >= N` decoding is a real, already-exercised
  forward-compatibility mechanism, but it is pure convention with no IDL or
  schema behind it — every future native client re-derives the same offsets
  by hand from this one Python file, forever, once per platform.
- The complete absence of a version/capability query (R5-CONTRACT-01) is the
  single biggest risk to "four native apps, sharing as much as possible": it
  means there is currently no way to build a client that degrades gracefully
  against a firmware it does not exactly match, which is guaranteed to happen
  the moment there is more than one firmware build in the field.

## Appendix: field-by-field contract table

### Frame header (all types) — proto.h/.c vs proto_ref.py
| Off | Size | Field | FW type/enc | Host type/enc | Notes |
|---|---|---|---|---|---|
| 0 | 1 | SOF | u8 = 0xA5 | 0xA5 | excluded from CRC |
| 1 | 1 | version | u8 = 1 | 1 | mismatch silently resynced (R5-CONTRACT-01) |
| 2 | 1 | type | u8 enum | int enum | CMD1/RSP2/EVT3/DATA4/IMU5, match |
| 3 | 1 | flags | u8 bitmask | int bitmask | SETTLING 0x01, OVERRUN 0x02, match |
| 4 | 2 | len | u16 LE | `<H` | payload length only |
| 6 | 2 | seq | u16 LE, wraps | `<H` | frame seq, not correlated to CMD seq (R2-LINK, excluded) |
| 8 | N | payload | bytes | bytes | |
| 8+N | 2 | CRC16 | u16 LE, CCITT-FALSE over [1..8+N-1] | same | match, self-tested |

### CMD opcode values and argument layout — command.h/.c vs swifteeg_link.py
| Op | Name | Args (payload[1..], after opcode byte) | Match |
|---|---|---|---|
| 0x01 | PING | none | yes |
| 0x02 | STREAM_START | none | yes |
| 0x03 | STREAM_STOP | none | yes |
| 0x04 | SET_ENCODING | [1]=enc (0-3, bound-checked) | yes |
| 0x05 | TEST_SIGNAL | [1]=on, [2]=cal_freq (opt, default DIV21) | layout yes; cal_freq enum host-side missing (R5-CONTRACT-05) |
| 0x06 | GET_INFO | none | yes; reply wastes a byte (R5-CONTRACT-01) |
| 0x07 | READ_REG | [1]=addr | yes |
| 0x08 | SET_INPUT | [1]=mux, [2]=cal_freq (opt) | layout yes; cal_freq enum missing (R5-CONTRACT-05) |
| 0x09 | SET_RATE | [1..2]=sps u16 LE | yes (early-OK semantics excluded, R3-DSP-02) |
| 0x0A | SET_CHANNEL | [1]=ch/0xFF,[2]=gain,[3]=mux,[4]=pd(opt),[5]=srb2(opt) | yes; not fully readable back (R5-CONTRACT-03) |
| 0x0B | SET_BIAS | [1]=enable,[2]=sensp(opt,def 0xFF),[3]=sensn(opt,def 0) | yes |
| 0x0C | SET_NOTCH | [1]=hz,[2]=Q,[3]=flags(bit0 harmonic,bit1 track) | yes |
| 0x0D | SET_LEADOFF | [1]=enable,[2]=sensp(opt),[3]=sensn(opt) | yes |
| 0x0E | GET_CONFIG | none | yes (see table below) |
| 0x0F | SET_IMU | [1]=enable,[2..3]=rate u16,[4]=accel_g,[5..6]=gyro_dps u16 | yes |
| 0x10 | SET_FILTER | [1]=stage,[2]=flags(bit0 keep),[3..4]=designed_for u16,[5]=count,[6..]=count*20B sections | yes, verified byte-exact |
| 0x11 | SET_CAR | [1]=enable,[2]=mask | yes |
| 0x12 | RESET_CHAIN | none | yes |

### RSP payload layout — generic and per-opcode
| Form | Bytes | Layout | Notes |
|---|---|---|---|
| generic | 2 | [0]=opcode,[1]=status | status 0=OK,1=EBADARG,2=EFAILED,3=EUNKNOWN; only OK/not-OK distinguished by shared host code (R5-CONTRACT-04) |
| "applies from" (FILTER/NOTCH/CAR/RESET_CHAIN, OK only) | 6 | [0]=opcode,[1]=OK,[2..6]=seq u32 LE | `respond_seq`/`response_seq`, match |
| GET_INFO | 6 | [0]=op,[1]=status,[2]=channels,[3]=0(unused),[4..6]=sps u16 LE | byte[3] wasted (R5-CONTRACT-01) |
| READ_REG | 4 | [0]=op,[1]=status,[2]=addr,[3]=value | match |
| GET_CONFIG | 39 (grows) | see below | verified byte-exact through all 4 tiers |
| any RSP w/ extra>46B | 2 | truncates to generic form, status still OK | R5-CONTRACT-02 |

### GET_CONFIG payload (opcode+status+37 = 39 bytes), full-payload offsets
| Off | Size | Field | Host tier gate | Notes |
|---|---|---|---|---|
| 0-1 | 2 | opcode, status | always | |
| 2 | 1 | channels | always | = ADS1299_CHANNELS |
| 3 | 1 | encoding | always | STREAM_ENC_* |
| 4-5 | 2 | rate (sps) u16 LE | always | |
| 6 | 1 | notch_hz (nominal) | always | 0/50/60 |
| 7-14 | 8 | chset[8] (raw CHnSET regs) | len>=15 | host decodes gain(bits6:4)/mux(bits2:0) only; PD(bit7)/SRB2(bit3) dropped (R5-CONTRACT-03) |
| 15 | 1 | imu flags (bit0 on, bit1 fitted) | len>=21 | |
| 16-17 | 2 | imu rate_hz u16 LE | len>=21 | |
| 18 | 1 | imu accel_g | len>=21 | |
| 19-20 | 2 | imu gyro_dps u16 LE | len>=21 | |
| 21 | 1 | pre_count | len>=29 | |
| 22 | 1 | post_count | len>=29 | |
| 23 | 1 | car enable (bit0) | len>=29 | |
| 24 | 1 | car_mask | len>=29 | |
| 25-26 | 2 | pre stage CRC16 LE | len>=29 | over packed sections, `sections_crc`/`sections_crc` match |
| 27-28 | 2 | post stage CRC16 LE | len>=29 | |
| 29 | 1 | notch_q | len>=39 | |
| 30 | 1 | notch flags (bit0 harmonic,bit1 track,bit2 measured) | len>=39 | |
| 31-34 | 4 | mains_hz f32 LE | len>=39 | host uses bit2 to gate None vs value, not a 0.0 sentinel check |
| 35-38 | 4 | notch_aim_hz f32 LE | len>=39 | 0 when off |

### EVT payload — stream.c vs swifteeg_link.py
| Off | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | event id | only EVT_MAINS=0x01 defined either side |
| 1 | 1 | flags (bit0 moved) | |
| 2-5 | 4 | seq u32 LE | first sample after the measurement |
| 6-9 | 4 | hz f32 LE | measured mains frequency |
Routed on the Stream characteristic, not Event (R2-LINK-09, excluded).

### DATA header + body — stream.c vs swifteeg_link.py `DATA_HDR`/`decode_data`
| Off | Size | Field | Notes |
|---|---|---|---|
| 0-7 | 8 | ts_us u64 LE (built from 2 LE u32 halves) | hardware DRDY timestamp of first sample |
| 8-11 | 4 | seq u32 LE | first sample's index |
| 12 | 1 | channels | = ADS1299_CHANNELS, host reads it (not hardcoded) |
| 13 | 1 | encoding | STREAM_ENC_*: 0=RAW_I32(i32 LE,signed),1=UV_F32(f32 LE),2=RAW_I24(i24 LE 2's-compl),3=RAW_UV(i24+f32, 7B) |
| 14-15 | 2 | count | samples in this batch (<=6, or <=3 for RAW_UV) |
| 16+ | count*ch*width | sample block | per-channel width by encoding: 4/4/3/7 bytes |

### IMU header + body — imu.c vs swifteeg_link.py `IMU_HDR`/`decode_imu`
| Off | Size | Field | Notes |
|---|---|---|---|
| 0-7 | 8 | ts_us u64 LE (2 LE u32 halves) | TIMER1-latched watermark edge, or extrapolated/estimated (flags say which) |
| 8-11 | 4 | seq (first sample index) u32 LE | |
| 12-15 | 4 | period_us_q8 u32 LE | measured sample period, 1/256 us units |
| 16-17 | 2 | accel_g (full scale) u16 LE | 2/4/8/16 |
| 18-19 | 2 | gyro_dps (full scale) u16 LE | 125/250/500/1000/2000/4000 |
| 20 | 1 | axes | always 6 (accel xyz, gyro xyz); host rejects otherwise |
| 21 | 1 | flags | bit0 TIME_ESTIMATED, bit1 OVERRUN, match |
| 22-23 | 2 | count | <=17 (FRAME_SAMPLES_MAX), host does not assume a max, reads count |
| 24+ | count*12 | samples | 6x i16 LE per sample: accel x,y,z then gyro x,y,z; 1 count = full_scale/32768 units |
