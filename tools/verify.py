"""
Hardware acceptance checks.

Runs the M3 measurements against a connected board and reports pass or fail
on each. Everything here is a comparison against a number that comes from
the datasheet or from the protocol definition - nothing is judged by eye.

    python tools/verify.py              # everything
    python tools/verify.py --quick      # skip the long noise measurement
    python tools/verify.py --port COM4

Exits non-zero if any check fails, so it can gate a release.

What is checked, and why each one is worth the time:

  registers    The AFE is configured the way the firmware believes. Every
               other measurement is meaningless if this is wrong, and it is
               the check that catches a silent SPI failure.

  test signal  Amplitude and frequency against the ADS1299's own generator.
               This is the only end-to-end check of absolute scale: it
               exercises the internal reference, the gain, the 24-bit
               conversion, the LSB constant, DMA, framing, USB and the host
               decoder in one measurement.

  noise floor  Shorted inputs, against the datasheet's input-referred noise.
               Catches a supply or reference problem that the test signal
               would ride straight over.

  timing       Sample interval and jitter from the hardware timestamps, plus
               sequence continuity. ERP work depends on these being true.

  protocol     CRC failures and sequence gaps over the whole run.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import struct
import sys
import time

import numpy as np
import serial
import serial.tools.list_ports

sys.path.insert(0, pathlib.Path(__file__).resolve().parent.as_posix())
import pipeline_ref  # noqa: E402
import proto_ref  # noqa: E402

# --- protocol -------------------------------------------------------------

TYPE_CMD, TYPE_RSP, TYPE_DATA = 0x01, 0x02, 0x04

CMD_STREAM_START = 0x02
CMD_STREAM_STOP = 0x03
CMD_SET_ENCODING = 0x04
CMD_READ_REG = 0x07
CMD_SET_INPUT = 0x08

ENC_RAW_I32 = 0

MUX_NORMAL, MUX_SHORTED, MUX_TEST = 0x00, 0x01, 0x05

DATA_HDR = struct.Struct("<QIBBH")

CHANNELS = 8
LSB_UV = pipeline_ref.lsb_uv()

# Register: (address, expected, name). Expected is what configure() writes.
EXPECTED_REGS = [
    (0x00, 0x3E, "ID       (ADS1299, 8 channels)"),
    (0x01, 0x96, "CONFIG1  (250 SPS)"),
    (0x03, 0xE0, "CONFIG3  (internal reference on)"),
    (0x15, 0x20, "MISC1    (SRB1 referential)"),
]

# ADS1299 datasheet, input-referred noise at gain 24, 250 SPS: ~0.14 uV RMS.
# Allow generous headroom - this is a "the front end is broken" check, not a
# characterisation.
NOISE_RMS_LIMIT_UV = 0.5

NOMINAL_SPS = 250.0


class Fail(Exception):
    pass


class Device:
    def __init__(self, port: str):
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.buf = bytearray()
        self.seq = 0
        self.bad_crc = 0
        self.frames = 0

    def close(self) -> None:
        try:
            self.cmd(CMD_STREAM_STOP)
            self.ser.close()
        except Exception:  # noqa: BLE001 - shutting down
            pass

    def cmd(self, op: int, *args: int) -> None:
        self.ser.write(proto_ref.encode(TYPE_CMD, 0, self.seq & 0xFFFF,
                                        bytes([op, *args])))
        self.seq += 1

    def _pump(self, seconds: float):
        """Read for a while, returning (data_blocks, responses)."""
        blocks, responses = [], []
        end = time.time() + seconds

        while time.time() < end:
            self.buf.extend(self.ser.read(4096))

            while True:
                i = self.buf.find(proto_ref.SOF)
                if i < 0:
                    self.buf.clear()
                    break
                if i:
                    del self.buf[:i]
                if len(self.buf) < proto_ref.HEADER_LEN:
                    break

                length = struct.unpack_from("<H", self.buf, 4)[0]
                total = proto_ref.HEADER_LEN + length + proto_ref.CRC_LEN
                if len(self.buf) < total:
                    break

                raw = bytes(self.buf[:total])
                del self.buf[:total]

                try:
                    frame = proto_ref.decode(raw)
                except Exception:  # noqa: BLE001 - a bad frame is a datum
                    self.bad_crc += 1
                    continue

                self.frames += 1
                if frame.type == TYPE_RSP:
                    responses.append(bytes(frame.payload))
                elif frame.type == TYPE_DATA:
                    blocks.append(self._unpack(frame.payload))

        return blocks, responses

    @staticmethod
    def _unpack(payload: bytes):
        ts, seq, ch, enc, count = DATA_HDR.unpack_from(payload)
        body = payload[DATA_HDR.size:DATA_HDR.size + count * ch * 4]
        vals = np.array(struct.unpack(f"<{count * ch}i", body)).reshape(count, ch)
        return ts, seq, vals

    def read_reg(self, addr: int, tries: int = 3):
        for _ in range(tries):
            self.cmd(CMD_READ_REG, addr)
            _, responses = self._pump(0.4)
            for r in responses:
                if len(r) >= 4 and r[0] == CMD_READ_REG and r[2] == addr:
                    return r[3]
        return None

    def set_input(self, mux: int) -> bool:
        self.cmd(CMD_SET_INPUT, mux, 0)
        _, responses = self._pump(0.8)
        for r in responses:
            if len(r) >= 2 and r[0] == CMD_SET_INPUT:
                return r[1] == 0
        return False

    def capture(self, seconds: float):
        """Stream for a while. Returns (counts, timestamps, first_seq)."""
        self.cmd(CMD_SET_ENCODING, ENC_RAW_I32)
        self._pump(0.2)
        self.cmd(CMD_STREAM_START)
        self._pump(0.3)          # let it get going before we start counting
        self.buf.clear()

        blocks, _ = self._pump(seconds)
        self.cmd(CMD_STREAM_STOP)
        self._pump(0.2)

        if not blocks:
            raise Fail("no data frames arrived")

        counts = np.vstack([b[2] for b in blocks])
        stamps = [(b[0], b[1], len(b[2])) for b in blocks]
        return counts, stamps


# --- checks ---------------------------------------------------------------

class Report:
    def __init__(self) -> None:
        self.failures = 0

    def check(self, ok: bool, label: str, detail: str = "") -> bool:
        mark = "PASS" if ok else "FAIL"
        if not ok:
            self.failures += 1
        print(f"  [{mark}] {label}" + (f"  {detail}" if detail else ""))
        return ok

    def info(self, text: str) -> None:
        print(f"         {text}")


def check_registers(dev: Device, rep: Report) -> None:
    print("\nregisters")
    for addr, want, name in EXPECTED_REGS:
        got = dev.read_reg(addr)
        if got is None:
            rep.check(False, name, "no response")
            continue
        rep.check(got == want, name, f"0x{got:02x} (expected 0x{want:02x})")


def check_test_signal(dev: Device, rep: Report, seconds: float) -> None:
    print("\ntest signal (ADS1299 internal generator)")

    if not rep.check(dev.set_input(MUX_TEST), "select test source"):
        return

    counts, _ = dev.capture(seconds)
    rep.info(f"{counts.shape[0]} samples")

    want_uv = pipeline_ref.CAL_AMPLITUDE_V * 1e6 * 2.0  # peak-to-peak
    want_hz = pipeline_ref.CAL_FREQ_HZ

    amps, freqs = [], []
    for ch in range(CHANNELS):
        x = counts[:, ch].astype(float)

        # Percentiles rather than min/max: a square wave spends almost no
        # time in transition, so this is the flat-top separation and is not
        # moved by a single noisy sample.
        lo, hi = np.percentile(x, 5), np.percentile(x, 95)
        pp_uv = (hi - lo) * LSB_UV
        amps.append(pp_uv)

        mid = (hi + lo) / 2.0
        crossings = np.count_nonzero(np.diff(np.sign(x - mid)) != 0)
        freqs.append(crossings / (2.0 * len(x) / NOMINAL_SPS))

    for ch in range(CHANNELS):
        err = abs(amps[ch] - want_uv) / want_uv * 100.0
        rep.check(err < 2.0, f"CH{ch + 1} amplitude",
                  f"{amps[ch] / 1000:.3f} mV p-p ({err:.2f} % off)")

    f_err = max(abs(f - want_hz) for f in freqs)
    rep.check(f_err < 0.1, "frequency, all channels",
              f"{min(freqs):.2f}-{max(freqs):.2f} Hz "
              f"(expected {want_hz:.2f})")

    spread = (max(amps) - min(amps)) / np.mean(amps) * 100.0
    rep.check(spread < 1.0, "channel matching", f"{spread:.2f} % spread")


def check_noise_floor(dev: Device, rep: Report, seconds: float) -> None:
    print("\nnoise floor (inputs shorted internally)")

    if not rep.check(dev.set_input(MUX_SHORTED), "select shorted source"):
        return

    counts, _ = dev.capture(seconds)
    rep.info(f"{counts.shape[0]} samples, gain 24")

    for ch in range(CHANNELS):
        x = counts[:, ch].astype(float) * LSB_UV
        rms = float(np.std(x))
        pp = float(np.percentile(x, 99.5) - np.percentile(x, 0.5))
        rep.check(rms < NOISE_RMS_LIMIT_UV, f"CH{ch + 1} noise",
                  f"{rms * 1000:.0f} nV RMS, {pp * 1000:.0f} nV p-p")

    rep.info(f"datasheet is ~140 nV RMS at gain 24, 250 SPS; "
             f"limit here {NOISE_RMS_LIMIT_UV * 1000:.0f} nV")


def check_timing(dev: Device, rep: Report, seconds: float) -> None:
    print("\ntiming and continuity")

    dev.set_input(MUX_NORMAL)
    counts, stamps = dev.capture(seconds)

    # Timestamps are per batch; the gap between batch starts covers the
    # samples in between, so divide by the batch size.
    per_sample = []
    for (ts_a, _, n_a), (ts_b, _, _) in zip(stamps, stamps[1:]):
        if n_a:
            per_sample.append((ts_b - ts_a) / n_a)

    gaps = np.array(per_sample, dtype=float)
    rep.check(len(gaps) > 0, "batches received", f"{len(stamps)}")
    if not len(gaps):
        return

    mean_us = float(np.mean(gaps))
    sps = 1e6 / mean_us

    rep.check(abs(sps - NOMINAL_SPS) / NOMINAL_SPS < 0.02, "sample rate",
              f"{sps:.2f} SPS (nominal {NOMINAL_SPS:.0f}, "
              f"{(sps / NOMINAL_SPS - 1) * 100:+.2f} %)")

    jitter = float(np.max(gaps) - np.min(gaps))
    rep.check(jitter < 50.0, "inter-batch jitter",
              f"{jitter:.1f} us peak to peak")

    # Sequence numbers are assigned per sample by the DSP thread, so a gap
    # means a frame was dropped somewhere between DMA and the transport.
    expected = 0
    gaps_found = 0
    for i, (_, seq, n) in enumerate(stamps):
        if i and seq != expected:
            gaps_found += 1
        expected = seq + n

    rep.check(gaps_found == 0, "no sequence gaps",
              f"{gaps_found} discontinuities in {len(stamps)} batches")


def check_protocol(dev: Device, rep: Report) -> None:
    print("\nprotocol")
    rep.check(dev.bad_crc == 0, "no CRC failures",
              f"{dev.bad_crc} bad of {dev.frames} frames")


# --- entry point ----------------------------------------------------------

def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2FE3 and p.pid == 0x0001:
            return p.device
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--quick", action="store_true",
                    help="shorter captures")
    args = ap.parse_args()

    port = args.port or find_port()
    if port is None:
        print("No SwiftEEG found (VID 2FE3 PID 0001). Is the board on?",
              file=sys.stderr)
        return 2

    span = 3.0 if args.quick else 8.0

    print(f"SwiftEEG hardware verification on {port}")
    print(f"LSB {LSB_UV * 1000:.3f} nV, capture {span:.0f} s per stage")

    dev = Device(port)
    rep = Report()

    try:
        check_registers(dev, rep)
        check_test_signal(dev, rep, span)
        check_noise_floor(dev, rep, span)
        check_timing(dev, rep, span)
        check_protocol(dev, rep)
    except Fail as exc:
        print(f"\naborted: {exc}", file=sys.stderr)
        rep.failures += 1
    finally:
        # Leave the board on the electrodes, not on a test source.
        try:
            dev.set_input(MUX_NORMAL)
        except Exception:  # noqa: BLE001
            pass
        dev.close()

    print()
    if rep.failures:
        print(f"{rep.failures} check(s) FAILED")
        return 1

    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
