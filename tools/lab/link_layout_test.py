"""
The Python decoders against byte layouts built the way the firmware builds
them (stream.c for DATA, command.c for GET_CONFIG and filter responses).
"""
import struct
import sys

import numpy as np

sys.path.insert(0, r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools")
import eeg_dsp  # noqa: E402
import proto_ref  # noqa: E402
import swifteeg_link as L  # noqa: E402


def i24(v):
    u = v & 0xFFFFFF
    return bytes((u & 0xFF, (u >> 8) & 0xFF, (u >> 16) & 0xFF))


def data_payload(enc, counts, uv, ts=123456789, seq=4242):
    n, ch = counts.shape
    body = bytearray()
    for i in range(n):
        for c in range(ch):
            if enc == L.ENC_RAW_UV:
                body += i24(int(counts[i, c])) + struct.pack("<f", float(uv[i, c]))
            elif enc == L.ENC_RAW_I24:
                body += i24(int(counts[i, c]))
            elif enc == L.ENC_UV_F32:
                body += struct.pack("<f", float(uv[i, c]))
            else:
                body += struct.pack("<i", int(counts[i, c]))
    return struct.pack("<QIBBH", ts, seq, ch, enc, n) + bytes(body)


def main():
    rng = np.random.default_rng(3)
    counts = rng.integers(-(1 << 23), 1 << 23, size=(3, 8))
    counts[0, 0], counts[0, 1] = (1 << 23) - 1, -(1 << 23)
    uv = rng.normal(0, 300, size=(3, 8)).astype(np.float32)

    for enc in (L.ENC_RAW_I32, L.ENC_UV_F32, L.ENC_RAW_I24, L.ENC_RAW_UV):
        p = data_payload(enc, counts, uv)
        ts, seq, e, c, u = L.decode_data(p)
        assert (ts, seq, e) == (123456789, 4242, enc)
        if enc == L.ENC_UV_F32:
            assert c is None and np.array_equal(u, uv)
        else:
            assert np.array_equal(c, counts), (enc, c, counts)
        if enc == L.ENC_RAW_UV:
            assert np.array_equal(u, uv), (u, uv)
        if enc in (L.ENC_RAW_I32, L.ENC_RAW_I24):
            assert u is None
    # A raw + microvolt batch of three must fit one notification at MTU 247.
    frame = proto_ref.encode(L.TYPE_DATA, 0, 1, data_payload(L.ENC_RAW_UV, counts, uv))
    assert len(frame) <= 244, len(frame)

    # Filter upload: 6 header bytes + 20 per section, and the CRC over the
    # packed sections the way command.c computes it.
    chain = eeg_dsp.Chain(250.0, 8)
    pre, post = chain.device_stages()
    args = L.filter_args(L.STAGE_PRE, pre, 250.0, keep_state=True)
    assert args[:5] == [0, 1, 250, 0, len(pre)], args[:5]
    # The opcode byte goes in front when sent: 6 + 20 per section in all.
    assert len(args) == 5 + 20 * len(pre), len(args)
    body = bytes(args[5:])
    assert len(body) == 20 * len(pre)
    back = [struct.unpack_from("<5f", body, 20 * i) for i in range(len(pre))]
    for sent, got in zip(pre, back):
        assert np.allclose(sent, got, rtol=1e-6), (sent, got)
    assert L.sections_crc([]) == 0xFFFF
    frame = proto_ref.encode(L.TYPE_CMD, 0, 1, bytes([L.CMD_SET_FILTER, *args]))
    assert len(frame) <= 244, f"upload of {len(pre)} sections is {len(frame)} bytes"
    big = L.filter_args(L.STAGE_PRE, pre * 2, 250.0)
    assert len(proto_ref.encode(L.TYPE_CMD, 0, 1, bytes([L.CMD_SET_FILTER, *big]))) <= 244

    # The notch command and the mains event.
    assert L.notch_args(50, 12, True, False) == [50, 12, 0x01]
    assert L.notch_args(60, 30, False, True) == [60, 30, 0x02]
    ev = L.decode_event(bytes([L.EVT_MAINS, 0x01]) + struct.pack("<If", 70001, 49.69))
    assert ev["event"] == "mains" and ev["moved"] and ev["seq"] == 70001
    assert abs(ev["hz"] - 49.69) < 1e-4, ev
    assert not L.decode_event(bytes([L.EVT_MAINS, 0x00]) + struct.pack("<If", 5, 50.0))["moved"]
    assert L.decode_event(bytes([0x7F, 0]) + bytes(8)) is None
    frame = proto_ref.encode(L.TYPE_EVT, 0, 1, bytes([L.EVT_MAINS, 1]) + bytes(8))
    assert len(frame) <= 244

    # GET_CONFIG as command.c lays it out.
    cfg = bytearray(37)
    cfg[0], cfg[1] = 8, L.ENC_RAW_UV
    cfg[2:4] = struct.pack("<H", 500)
    cfg[4] = 0
    cfg[5:13] = bytes([0x60, 0x50, 0x60, 0x65, 0x61, 0x61, 0x31, 0x61])
    cfg[13] = 0x03
    cfg[14:16] = struct.pack("<H", 240)
    cfg[16] = 8
    cfg[17:19] = struct.pack("<H", 2000)
    cfg[19], cfg[20], cfg[21], cfg[22] = len(pre), len(post), 0x01, 0xDF
    cfg[23:25] = struct.pack("<H", L.sections_crc(pre))
    cfg[25:27] = struct.pack("<H", L.sections_crc(post))
    cfg[27] = 12
    cfg[28] = L.NOTCH_HARMONIC | L.NOTCH_TRACK | L.NOTCH_MEASURED
    cfg[29:33] = struct.pack("<f", 49.68)
    cfg[33:37] = struct.pack("<f", 49.68)
    d = L.decode_config(bytes([L.CMD_GET_CONFIG, 0]) + bytes(cfg))
    assert d["rate"] == 500 and d["encoding"] == L.ENC_RAW_UV and d["notch"] == 0
    assert d["gains"] == [24, 12, 24, 24, 24, 24, 6, 24], d["gains"]
    assert d["mux"][3] == L.MUX_TEST and d["mux"][4] == L.MUX_SHORTED
    assert d["imu_on"] and d["imu_fitted"] and d["imu_rate"] == 240
    assert d["imu_accel_g"] == 8 and d["imu_gyro_dps"] == 2000
    assert d["pre_count"] == len(pre) and d["post_count"] == len(post)
    assert d["car"] and d["car_mask"] == 0xDF
    assert d["pre_crc"] == L.sections_crc(pre)
    assert d["post_crc"] == L.sections_crc(post)
    assert d["notch_q"] == 12 and d["notch_harmonic"] and d["notch_track"]
    assert abs(d["mains_hz"] - 49.68) < 1e-4 and abs(d["notch_aim_hz"] - 49.68) < 1e-4
    cfg[28] = L.NOTCH_HARMONIC
    d = L.decode_config(bytes([L.CMD_GET_CONFIG, 0]) + bytes(cfg))
    assert d["mains_hz"] is None and not d["notch_track"], d

    # A filter response: opcode, status, u32 sequence number.
    assert L.response_seq(bytes([L.CMD_SET_FILTER, 0]) + struct.pack("<I", 70000)) == 70000
    assert L.response_seq(bytes([L.CMD_SET_FILTER, 1])) is None

    # A damaged notification must not take the next one with it: a data
    # frame cut short, then a whole reply in the notification after.
    import queue
    import types
    ble = types.SimpleNamespace(parser=L.FrameParser(), frames=queue.Queue())
    data = proto_ref.encode(L.TYPE_DATA, 0, 7, data_payload(L.ENC_RAW_UV, counts, uv))
    reply = proto_ref.encode(L.TYPE_RSP, 0, 8, bytes([L.CMD_STREAM_STOP, 0]))
    L.BleLink._take_notification(ble, data[:40])
    L.BleLink._take_notification(ble, reply)
    got = []
    while not ble.frames.empty():
        got.append(ble.frames.get())
    assert len(got) == 1 and got[0].type == L.TYPE_RSP, got
    assert ble.parser.bad == 1 and not ble.parser.buf

    print(f"link layout test: OK  (pre {len(pre)} sections, post {len(post)})")


if __name__ == "__main__":
    main()
