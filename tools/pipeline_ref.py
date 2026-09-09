"""
Reference implementation of the whole acquisition chain.

dsp_ref.py covers the DSP primitives one at a time. This covers how they are
composed in src/pipeline/pipeline.c, which is where the remaining mistakes
live: a chain can be built from correct parts and still be wrong, if the
stages run in the wrong order, or the scaling is applied on the wrong side of
the DC removal, or a frame is decoded with the wrong byte order.

The chain, in order:

    27-byte RDATAC frame
      -> 24-bit big-endian two's complement, per channel   (frame.h)
      -> integer DC removal, leaky integrator, shift 9     (dsp_dc_apply)
      -> multiply by the LSB to get microvolts             (float32 from here)
      -> mains notch biquad, transposed direct form II     (dsp_cascade_apply)

The DC removal deliberately happens in the integer domain, before the float
conversion: full scale at gain 24 is +/-187.5 mV against a 22.35 nV LSB, a
ratio of 8.4e6, and float32 carries about 1.7e7 of mantissa. Converting a
DC-laden sample first would leave almost nothing for the signal on top.

    python tools/pipeline_ref.py           # self-test
    python tools/pipeline_ref.py --emit    # write tests/pipeline/golden_pipeline.h
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, pathlib.Path(__file__).resolve().parent.as_posix())
import dsp_ref  # noqa: E402  - the primitives are defined once, over there

F32 = np.float32

# --- constants that must match the firmware ------------------------------

CHANNELS = 8
STATUS_BYTES = 3
FRAME_BYTES = STATUS_BYTES + CHANNELS * 3

STATUS_MARKER = 0xC0

FS_HZ = 250.0
DC_SHIFT = 9
NOTCH_HZ = 50.0
NOTCH_Q = 30.0

VREF_V = 4.5
GAIN = 24

# The ADS1299's own generator: (VREFP-VREFN)/2400, at fCLK/2^21.
CAL_AMPLITUDE_V = VREF_V / 2400.0
CAL_FREQ_HZ = 2.048e6 / (1 << 21)


def lsb_uv() -> float:
    return dsp_ref.lsb_uv(VREF_V, GAIN)


# --- frame encode/decode, mirroring src/pipeline/frame.h ------------------

def encode_frame(counts, status: int = 0xC00000) -> bytes:
    """Build a 27-byte RDATAC frame from eight signed 24-bit values."""
    out = bytearray()
    out.append((status >> 16) & 0xFF)
    out.append((status >> 8) & 0xFF)
    out.append(status & 0xFF)

    for v in counts:
        v = int(v) & 0xFFFFFF  # two's complement, 24-bit
        out.append((v >> 16) & 0xFF)
        out.append((v >> 8) & 0xFF)
        out.append(v & 0xFF)

    return bytes(out)


def decode_frame(frame: bytes):
    """Inverse of encode_frame. Mirrors frame_channel() exactly."""
    vals = []
    for ch in range(CHANNELS):
        p = STATUS_BYTES + ch * 3
        raw = (frame[p] << 16) | (frame[p + 1] << 8) | frame[p + 2]
        vals.append(raw - (1 << 24) if raw & 0x800000 else raw)
    return vals


def frame_is_valid(frame: bytes) -> bool:
    return (frame[0] & 0xF0) == STATUS_MARKER


# --- the chain ------------------------------------------------------------

def run_chain(frames):
    """
    Frames in, microvolts out. One DC state and one biquad state per channel,
    carried across the whole run exactly as the firmware carries them.

    An invalid frame produces no output row at all, which is what the
    firmware does: it counts the frame and returns before the filters, so a
    misaligned sample neither reaches the output nor advances any state.
    The result therefore has one row per *valid* frame, not per frame.
    """
    notch = dsp_ref.design_notch(FS_HZ, NOTCH_HZ, NOTCH_Q)
    scale = F32(lsb_uv())

    dc_acc = [0] * CHANNELS
    dc_primed = [False] * CHANNELS
    s1 = [F32(0.0)] * CHANNELS
    s2 = [F32(0.0)] * CHANNELS

    b0, b1, b2, a1, a2 = notch
    rows = []

    for frame in frames:
        if not frame_is_valid(frame):
            continue

        counts = decode_frame(frame)
        row = np.zeros(CHANNELS, dtype=F32)

        for ch in range(CHANNELS):
            s = counts[ch]

            # Stage 0: integer DC removal, primed on the first sample.
            if not dc_primed[ch]:
                dc_acc[ch] = s << DC_SHIFT
                dc_primed[ch] = True
            else:
                dc_acc[ch] += s - (dc_acc[ch] >> DC_SHIFT)
            ac = s - (dc_acc[ch] >> DC_SHIFT)

            # Stage 1: to microvolts, float32 from here on.
            x = F32(F32(ac) * scale)

            # Stage 2: mains notch, transposed direct form II.
            y = F32(b0 * x + s1[ch])
            s1[ch] = F32(b1 * x + a1 * y + s2[ch])
            s2[ch] = F32(b2 * x + a2 * y)

            row[ch] = y

        rows.append(row)

    return np.array(rows, dtype=F32) if rows else np.zeros((0, CHANNELS), F32)


# --- a synthetic recording with something to check ------------------------

def make_frames(n: int):
    """
    Eight channels, each carrying something the chain should handle:

      1  the cal square wave, on a large DC offset
      2  the same, inverted
      3  50 Hz mains only - the notch must remove it
      4  10 Hz alpha, which must survive
      5  10 Hz alpha plus 50 Hz mains
      6  a DC step partway through
      7  full-scale positive, to exercise sign handling at the top
      8  full-scale negative, likewise at the bottom
    """
    t = np.arange(n) / FS_HZ

    cal_counts = CAL_AMPLITUDE_V / (dsp_ref.lsb_uv(VREF_V, GAIN) * 1e-6)
    square = np.sign(np.sin(2 * np.pi * CAL_FREQ_HZ * t)) * cal_counts

    # A big electrode offset, the thing stage 0 exists to remove.
    offset = 400_000

    alpha = 10.0 / (dsp_ref.lsb_uv(VREF_V, GAIN) * 1e-6) * 1e-6  # 10 uV
    mains = 50.0 / (dsp_ref.lsb_uv(VREF_V, GAIN) * 1e-6) * 1e-6  # 50 uV

    step = np.where(t < t[n // 2], 0, 250_000)

    ch = [
        square + offset,
        -square - offset,
        mains * np.sin(2 * np.pi * 50.0 * t),
        alpha * np.sin(2 * np.pi * 10.0 * t),
        alpha * np.sin(2 * np.pi * 10.0 * t) + mains * np.sin(2 * np.pi * 50.0 * t),
        step,
        np.full(n, (1 << 23) - 1),
        np.full(n, -(1 << 23)),
    ]

    frames = []
    for i in range(n):
        frames.append(encode_frame([int(c[i]) for c in ch]))
    return frames


# --- self-test ------------------------------------------------------------

def _self_test() -> None:
    # Round-tripping every corner of the 24-bit range.
    corners = [0, 1, -1, (1 << 23) - 1, -(1 << 23), 0x7FFFFF, -0x800000, 12345, -12345]
    f = encode_frame(corners[:CHANNELS])
    assert decode_frame(f) == corners[:CHANNELS], decode_frame(f)
    assert frame_is_valid(f)
    assert len(f) == FRAME_BYTES, len(f)

    # The status marker is what catches a misaligned frame.
    assert not frame_is_valid(b"\x00" + f[1:])
    assert not frame_is_valid(b"\xff" + f[1:])

    # The LSB the whole scale rests on.
    assert abs(lsb_uv() - 0.02235174) < 1e-7, lsb_uv()

    # The cal signal in counts, which is what the hardware test checks.
    cal_counts = CAL_AMPLITUDE_V / (lsb_uv() * 1e-6)
    assert abs(cal_counts - 83886) < 2, cal_counts

    frames = make_frames(600)
    out = run_chain(frames)

    # A large DC offset must not reach the output.
    assert abs(out[-1, 0]) < 5000, f"DC leaked through: {out[-1, 0]}"

    # 50 Hz alone should be most of the way gone once the notch has settled.
    tail = out[400:, 2]
    assert np.abs(tail).max() < 8.0, f"notch left {np.abs(tail).max()} uV of mains"

    # 10 Hz must survive it, at roughly the amplitude put in.
    alpha_out = out[400:, 3]
    assert 8.0 < np.abs(alpha_out).max() < 12.0, np.abs(alpha_out).max()

    # Full-scale inputs are constant, so after DC removal they are zero.
    assert np.abs(out[400:, 6]).max() < 1.0
    assert np.abs(out[400:, 7]).max() < 1.0

    # A discarded frame must leave no trace: everything before it is
    # identical, and what follows carries on cleanly rather than ringing.
    bad = list(frames)
    bad[300] = bytes([0x00]) + bad[300][1:]
    out_bad = run_chain(bad)

    assert out_bad.shape[0] == out.shape[0] - 1, out_bad.shape
    assert np.array_equal(out[:300], out_bad[:300]),         "output before the bad frame changed"

    after = out_bad[350:, 3]
    assert np.isfinite(after).all(), "chain produced non-finite values"
    assert np.abs(after).max() < 12.0,         f"chain rang after a discarded frame: {np.abs(after).max()} uV"

    print("pipeline_ref self-test: OK")


# --- golden vector emission -----------------------------------------------

OUT = (pathlib.Path(__file__).resolve().parent.parent
       / "tests" / "pipeline" / "golden_pipeline.h")

N_GOLDEN = 512


def _c_bytes(data: bytes) -> str:
    rows = []
    for i in range(0, len(data), 16):
        rows.append(", ".join(f"0x{b:02x}" for b in data[i:i + 16]))
    return ",\n\t\t".join(rows)


def _c_f32(vals) -> str:
    flat = np.asarray(vals).reshape(-1)
    rows = []
    for i in range(0, len(flat), 6):
        rows.append(", ".join(f"{float(v):.9e}f" for v in flat[i:i + 6]))
    return ",\n\t\t".join(rows)


def _emit() -> None:
    frames = make_frames(N_GOLDEN)
    out = run_chain(frames)

    blob = b"".join(frames)

    L = []
    A = L.append
    A("/*")
    A(" * GENERATED FILE - DO NOT EDIT BY HAND.")
    A(" *")
    A(" * Produced by tools/pipeline_ref.py, the reference for the whole")
    A(" * acquisition chain: frame decode, integer DC removal, microvolt")
    A(" * scaling and the mains notch, composed exactly as pipeline.c")
    A(" * composes them. If the C disagrees with these values, the C is wrong.")
    A(" *")
    A(" * Regenerate:  python tools/pipeline_ref.py --emit")
    A(" */")
    A("#ifndef SWIFTEEG_GOLDEN_PIPELINE_H")
    A("#define SWIFTEEG_GOLDEN_PIPELINE_H")
    A("")
    A("#include <stdint.h>")
    A("")
    A(f"#define GOLDEN_PIPE_FRAMES   {N_GOLDEN}")
    A(f"#define GOLDEN_PIPE_CHANNELS {CHANNELS}")
    A(f"#define GOLDEN_PIPE_FS       {FS_HZ:.1f}f")
    A(f"#define GOLDEN_PIPE_DC_SHIFT {DC_SHIFT}")
    A(f"#define GOLDEN_PIPE_NOTCH_HZ {NOTCH_HZ:.1f}f")
    A(f"#define GOLDEN_PIPE_NOTCH_Q  {NOTCH_Q:.1f}f")
    A(f"#define GOLDEN_PIPE_LSB_UV   {lsb_uv():.9e}f")
    A("")
    A("/* Raw RDATAC frames, 27 bytes each, exactly as they arrive by DMA. */")
    A("static const uint8_t golden_pipe_frames"
      f"[GOLDEN_PIPE_FRAMES * {FRAME_BYTES}] = {{")
    A("\t\t" + _c_bytes(blob))
    A("};")
    A("")
    A("/* Microvolts out, after the full chain. */")
    A("static const float golden_pipe_out"
      "[GOLDEN_PIPE_FRAMES][GOLDEN_PIPE_CHANNELS] = {")
    for row in out:
        A("\t\t{ " + ", ".join(f"{float(v):.9e}f" for v in row) + " },")
    A("};")
    A("")
    A("#endif /* SWIFTEEG_GOLDEN_PIPELINE_H */")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {OUT}  ({OUT.stat().st_size // 1024} kB)")


if __name__ == "__main__":
    _self_test()
    if "--emit" in sys.argv:
        _emit()
