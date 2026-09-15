"""
Reference implementation of the whole acquisition chain.

dsp_ref.py covers the DSP primitives one at a time. This covers how they are
composed in src/pipeline/chain.c, which is where the remaining mistakes
live: a chain can be built from correct parts and still be wrong, if the
stages run in the wrong order, or the scaling is applied on the wrong side of
the DC removal, or a frame is decoded with the wrong byte order.

The chain, in order:

    27-byte RDATAC frame
      -> 24-bit big-endian two's complement, per channel   (frame.h)
      -> integer DC removal, leaky integrator              (dsp_dc_apply)
      -> times the channel's own LSB, to microvolts        (float32 from here)
      -> pre sections: high-pass and notch                 (dsp_cascade_apply)
      -> common average over the masked channels
      -> post sections: low-pass

The DC removal deliberately happens in the integer domain, before the float
conversion: full scale at gain 24 is +/-187.5 mV against a 22.35 nV LSB, a
ratio of 8.4e6, and float32 carries about 1.7e7 of mantissa. Converting a
DC-laden sample first would leave almost nothing for the signal on top.

    python tools/pipeline_ref.py           # self-test
    python tools/pipeline_ref.py --emit    # write tests/pipeline/golden_pipeline.h
"""

from __future__ import annotations

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
NOTCH_HZ = 50.0
NOTCH_Q = 12.0          # the device notch, pipeline.c

VREF_V = 4.5
GAIN = 24

STAGE_PRE, STAGE_POST = 0, 1

# The ADS1299's own generator: (VREFP-VREFN)/2400, at fCLK/2^21.
CAL_AMPLITUDE_V = VREF_V / 2400.0
CAL_FREQ_HZ = 2.048e6 / (1 << 21)


def lsb_uv() -> float:
    return dsp_ref.lsb_uv(VREF_V, GAIN)


def dc_shift_for(sps: float) -> int:
    """Mirrors pipeline.c: the shift that holds the DC corner near 0.08 Hz."""
    shift, r = 9, 250
    while r < sps and shift < 15:
        r <<= 1
        shift += 1
    return shift


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

class Chain:
    """
    Mirror of chain_t, operation for operation. Settings change between
    samples exactly as the firmware changes them, so a host that replays the
    same changes at the same sample numbers gets the same output.
    """

    def __init__(self, fs: float = FS_HZ, dc_shift: int | None = None,
                 notch_hz: float = NOTCH_HZ, notch_q: float = NOTCH_Q,
                 vref: float = VREF_V, gain: int = GAIN):
        self.fs = float(fs)
        self.shift = dc_shift_for(fs) if dc_shift is None else dc_shift
        self.vref = vref
        self.lsb = [dsp_ref.lsb_uv_f32(vref, gain)] * CHANNELS
        self.dc_acc = [0] * CHANNELS
        self.dc_primed = [False] * CHANNELS
        self.pre = dsp_ref.Cascade(CHANNELS)
        self.post = dsp_ref.Cascade(CHANNELS)
        self.car = False
        self.mask = 0xFF
        self.set_notch(notch_hz, notch_q)

    def set_notch(self, hz: float, q: float = NOTCH_Q) -> None:
        self.pre.set([] if hz <= 0 else [dsp_ref.design_notch(self.fs, hz, q)])

    def set_stage(self, stage: int, sections, keep_state: bool = False) -> bool:
        """Load a stage. Returns whether the state was kept."""
        cas = self.pre if stage == STAGE_PRE else self.post
        if keep_state and cas.retune(sections):
            return True
        if not cas.set(sections):
            raise ValueError("invalid section")
        return False

    def set_car(self, enable: bool, mask: int) -> None:
        self.car = bool(enable)
        self.mask = int(mask) & 0xFF

    def set_gain(self, ch: int, gain: int) -> None:
        lsb = dsp_ref.lsb_uv_f32(self.vref, gain)
        if lsb != self.lsb[ch]:
            self.lsb[ch] = lsb
            self.dc_primed[ch] = False

    def reset(self) -> None:
        self.dc_primed = [False] * CHANNELS
        self.pre.reset()
        self.post.reset()

    def settle_samples(self) -> int:
        return min(0xFFFFFFFF,
                   self.pre.settle_samples() + self.post.settle_samples())

    def process_counts(self, counts):
        """One sample's eight raw counts in, eight float32 microvolts out."""
        v = []
        for ch in range(CHANNELS):
            s = int(counts[ch])

            # Stage 0: integer DC removal, primed on the first sample.
            if not self.dc_primed[ch]:
                self.dc_acc[ch] = s << self.shift
                self.dc_primed[ch] = True
            else:
                self.dc_acc[ch] += s - (self.dc_acc[ch] >> self.shift)
            ac = s - (self.dc_acc[ch] >> self.shift)

            # Stage 1: microvolts, at this channel's own gain.
            uv = F32(ac) * self.lsb[ch]

            # Stage 2: the pre sections.
            v.append(self.pre.apply(ch, uv))

        # Stage 3: common average over the masked channels.
        if self.car:
            total = F32(0.0)
            n = 0
            for ch in range(CHANNELS):
                if self.mask & (1 << ch):
                    total = total + v[ch]
                    n += 1
            if n >= 2:
                mean = total / F32(n)
                v = [x - mean for x in v]

        # Stage 4: the post sections.
        return np.array([self.post.apply(ch, v[ch]) for ch in range(CHANNELS)],
                        dtype=F32)

    def process_frame(self, frame: bytes):
        """A frame in; None for one the firmware would reject."""
        if not frame_is_valid(frame):
            return None
        return self.process_counts(decode_frame(frame))


def run_chain(frames, chain: Chain | None = None):
    """
    Frames in, microvolts out, through the device's default chain unless
    another is given.

    An invalid frame produces no output row at all, which is what the
    firmware does: it counts the frame and returns before the filters, so a
    misaligned sample neither reaches the output nor advances any state.
    The result therefore has one row per *valid* frame, not per frame.
    """
    chain = chain or Chain()
    rows = [r for r in (chain.process_frame(f) for f in frames) if r is not None]
    return np.array(rows, dtype=F32) if rows else np.zeros((0, CHANNELS), F32)


# --- a synthetic recording with something to check ------------------------

def make_frames(n: int, fs: float = FS_HZ):
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
    t = np.arange(n) / fs

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

    return [encode_frame([int(c[i]) for c in ch]) for i in range(n)]


# --- the host chain on the device: the second golden configuration --------

def full_config() -> dict:
    """
    What the host application loads: a 0.5 Hz high-pass and a mains notch
    with its harmonic before the average, a 45 Hz low-pass after it. Partway
    through, the notch follows the mains 0.2 Hz down keeping its state, the
    average takes in another channel, and the chain restarts.
    """
    fs = 250.0
    q1, q2 = dsp_ref.butterworth_qs(4)
    hp = [dsp_ref.design_highpass(fs, 0.5, q1), dsp_ref.design_highpass(fs, 0.5, q2)]
    return {
        "fs": fs,
        "gains": [24, 12, 24, 24, 24, 24, 24, 24],
        "pre": hp + [dsp_ref.design_notch(fs, 50.0, 12.0),
                     dsp_ref.design_notch(fs, 100.0, 12.0)],
        "pre_retuned": hp + [dsp_ref.design_notch(fs, 49.8, 12.0),
                             dsp_ref.design_notch(fs, 99.6, 12.0)],
        "post": [dsp_ref.design_lowpass(fs, 45.0, q1),
                 dsp_ref.design_lowpass(fs, 45.0, q2)],
        "mask": 0x3F,
        "mask_later": 0x7F,
        "retune_at": 256,
        "car_at": 384,
        "reset_at": 448,
        "restart_post_at": 320,
    }


def build_full(cfg: dict) -> Chain:
    c = Chain(cfg["fs"], notch_hz=0.0)
    for ch, g in enumerate(cfg["gains"]):
        c.set_gain(ch, g)
    c.set_stage(STAGE_PRE, cfg["pre"])
    c.set_stage(STAGE_POST, cfg["post"])
    c.set_car(True, cfg["mask"])
    return c


def run_full(frames, cfg: dict, events: bool = True):
    c = build_full(cfg)
    rows = []
    for i, f in enumerate(frames):
        if events and i == cfg["retune_at"]:
            assert c.set_stage(STAGE_PRE, cfg["pre_retuned"], keep_state=True)
        if events and i == cfg["restart_post_at"]:
            assert not c.set_stage(STAGE_POST, cfg["post"])
        if events and i == cfg["car_at"]:
            c.set_car(True, cfg["mask_later"])
        if events and i == cfg["reset_at"]:
            c.reset()
        rows.append(c.process_frame(f))
    return np.array(rows, dtype=F32)


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

    # The DC corner holds near 0.08 Hz at every rate.
    for sps in (250, 500, 1000, 2000, 4000, 8000, 16000):
        fc = sps / (2 * np.pi * (1 << dc_shift_for(sps)))
        assert 0.07 < fc < 0.09, (sps, fc)

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
    assert np.array_equal(out[:300], out_bad[:300]), \
        "output before the bad frame changed"

    after = out_bad[350:, 3]
    assert np.isfinite(after).all(), "chain produced non-finite values"
    assert np.abs(after).max() < 12.0, \
        f"chain rang after a discarded frame: {np.abs(after).max()} uV"

    # A channel at half the gain carries the same microvolts in twice the
    # counts' worth of scale: its LSB doubles, and so does its output here,
    # where the counts are unchanged.
    half = Chain()
    half.set_gain(1, 12)
    ratio = np.abs(run_chain(frames[:200], half)[100:, 1]).max() / \
        np.abs(out[100:200, 1]).max()
    assert abs(ratio - 2.0) < 1e-4, f"gain 12 scaled channel 2 by {ratio}"

    # The common average removes what every channel shares, and a mask of
    # one channel is not an average of anything.
    t = np.arange(400) / FS_HZ
    shared = (10.0 / (lsb_uv() * 1e-6) * 1e-6) * np.sin(2 * np.pi * 10.0 * t)
    common = [encode_frame([int(v)] * CHANNELS) for v in shared]
    car = Chain(notch_hz=0.0)
    car.set_car(True, 0xFF)
    assert np.abs(run_chain(common, car)).max() < 1e-3, "a shared signal survived"
    one = Chain(notch_hz=0.0)
    one.set_car(True, 0x01)
    assert np.array_equal(run_chain(common, one), run_chain(common, Chain(notch_hz=0.0))), \
        "a single-channel average changed the signal"

    # A retune that keeps its state barely disturbs a signal the filter
    # passes, where a restart rings: 20 uV of alpha through the notch, moved
    # 0.2 Hz partway through.
    alpha20 = (20.0 / (lsb_uv() * 1e-6) * 1e-6) * np.sin(2 * np.pi * 10.0 * t)
    tone = [encode_frame([int(v)] * CHANNELS) for v in alpha20]
    moved = [dsp_ref.design_notch(FS_HZ, 49.8, NOTCH_Q)]

    def retuned(keep: bool):
        c = Chain()
        rows = []
        for i, fr in enumerate(tone):
            if i == 200:
                c.set_stage(STAGE_PRE, moved, keep_state=keep)
            rows.append(c.process_frame(fr))
        return np.array(rows, dtype=F32)

    base = run_chain(tone)
    kept = np.abs(retuned(True)[200:260, 0] - base[200:260, 0]).max()
    restarted = np.abs(retuned(False)[200:260, 0] - base[200:260, 0]).max()
    assert kept < 1.0 and restarted > 3 * kept, \
        f"retune kept state: {kept:.3f} uV off; restarted: {restarted:.3f} uV"

    # The full chain, with its events.
    cfg = full_config()
    frames_b = make_frames(512)
    with_events = run_full(frames_b, cfg)

    # A reset is exactly a fresh chain started at that frame. This is what
    # lets a host reproduce the device's output from a known sample.
    r = cfg["reset_at"]
    fresh = build_full(cfg)
    fresh.set_stage(STAGE_PRE, cfg["pre_retuned"])
    fresh.set_car(True, cfg["mask_later"])
    replay = np.array([fresh.process_frame(fr) for fr in frames_b[r:]], dtype=F32)
    assert np.array_equal(replay, with_events[r:]), "a reset is not a fresh start"

    # Invalid sections are refused and change nothing.
    c = build_full(cfg)
    try:
        c.set_stage(STAGE_PRE, [(0.0, 1.0, 1.0, -1.0, 0.0)])
        raise AssertionError("an invalid section was accepted")
    except ValueError:
        pass
    assert len(c.pre.sections) == len(cfg["pre"])

    assert 0 < c.settle_samples() < 60 * cfg["fs"], c.settle_samples()

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


def _c_section(s) -> str:
    return "{ " + ", ".join(f"{float(v):.9e}f" for v in s) + " }"


def _c_rows(out) -> list[str]:
    return ["\t\t{ " + ", ".join(f"{float(v):.9e}f" for v in row) + " },"
            for row in out]


def _emit() -> None:
    frames = make_frames(N_GOLDEN)
    out_a = run_chain(frames)
    cfg = full_config()
    out_b = run_full(frames, cfg)

    blob = b"".join(frames)

    L = []
    A = L.append
    A("/*")
    A(" * GENERATED FILE - DO NOT EDIT BY HAND.")
    A(" *")
    A(" * Produced by tools/pipeline_ref.py, the reference for the whole")
    A(" * acquisition chain: frame decode, integer DC removal, per-channel")
    A(" * scaling, the pre sections, the common average and the post sections,")
    A(" * composed exactly as chain.c composes them. If the C disagrees with")
    A(" * these values, the C is wrong.")
    A(" *")
    A(" * Regenerate:  python tools/pipeline_ref.py --emit")
    A(" */")
    A("#ifndef SWIFTEEG_GOLDEN_PIPELINE_H")
    A("#define SWIFTEEG_GOLDEN_PIPELINE_H")
    A("")
    A("#include <stdint.h>")
    A("")
    A('#include "dsp/dsp.h"')
    A("")
    A(f"#define GOLDEN_PIPE_FRAMES   {N_GOLDEN}")
    A(f"#define GOLDEN_PIPE_CHANNELS {CHANNELS}")
    A(f"#define GOLDEN_PIPE_LSB_UV   {lsb_uv():.9e}f")
    A("")
    A("/* Raw RDATAC frames, 27 bytes each, exactly as they arrive by DMA. */")
    A("static const uint8_t golden_pipe_frames"
      f"[GOLDEN_PIPE_FRAMES * {FRAME_BYTES}] = {{")
    A("\t\t" + _c_bytes(blob))
    A("};")
    A("")
    A("/* A: the device's default chain - the mains notch, nothing else. */")
    A(f"#define GOLDEN_A_FS       {FS_HZ:.1f}f")
    A(f"#define GOLDEN_A_DC_SHIFT {dc_shift_for(FS_HZ)}")
    A(f"#define GOLDEN_A_NOTCH_HZ {NOTCH_HZ:.1f}f")
    A(f"#define GOLDEN_A_NOTCH_Q  {NOTCH_Q:.1f}f")
    A("")
    A("static const float golden_a_out"
      "[GOLDEN_PIPE_FRAMES][GOLDEN_PIPE_CHANNELS] = {")
    L.extend(_c_rows(out_a))
    A("};")
    A("")
    A("/*")
    A(" * B: the host application's chain loaded onto the device, channel 2 at")
    A(" * gain 12. At RETUNE_AT the notch pair moves 0.2 Hz keeping its state;")
    A(" * at RESTART_POST_AT the low-pass restarts, priming on its next input;")
    A(" * at CAR_AT the average takes in channel 7; at RESET_AT the chain")
    A(" * restarts.")
    A(" */")
    A(f"#define GOLDEN_B_FS            {cfg['fs']:.1f}f")
    A(f"#define GOLDEN_B_DC_SHIFT      {dc_shift_for(cfg['fs'])}")
    A(f"#define GOLDEN_B_PRE_COUNT     {len(cfg['pre'])}")
    A(f"#define GOLDEN_B_POST_COUNT    {len(cfg['post'])}")
    A(f"#define GOLDEN_B_CAR_MASK      0x{cfg['mask']:02x}u")
    A(f"#define GOLDEN_B_CAR_MASK_LATER 0x{cfg['mask_later']:02x}u")
    A(f"#define GOLDEN_B_RETUNE_AT     {cfg['retune_at']}")
    A(f"#define GOLDEN_B_CAR_AT        {cfg['car_at']}")
    A(f"#define GOLDEN_B_RESET_AT      {cfg['reset_at']}")
    A(f"#define GOLDEN_B_RESTART_POST_AT {cfg['restart_post_at']}")
    A("")
    A("static const uint8_t golden_b_gains[GOLDEN_PIPE_CHANNELS] = { "
      + ", ".join(str(g) for g in cfg["gains"]) + " };")
    A("")
    for name in ("pre", "pre_retuned", "post"):
        count = "GOLDEN_B_POST_COUNT" if name == "post" else "GOLDEN_B_PRE_COUNT"
        A(f"static const dsp_section_t golden_b_{name}[{count}] = {{")
        for s in cfg[name]:
            A("\t\t" + _c_section(s) + ",")
        A("};")
        A("")
    A("static const float golden_b_out"
      "[GOLDEN_PIPE_FRAMES][GOLDEN_PIPE_CHANNELS] = {")
    L.extend(_c_rows(out_b))
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
