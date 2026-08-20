"""
Generates golden test vectors for the C protocol codec.

Vectors come from proto_ref.py, which is the oracle. The generated header is
committed so the on-target tests are reproducible and a change in the C codec
that silently alters the wire format fails the build rather than quietly
shipping.

Regenerate with:  python tools/gen_golden.py
"""

from __future__ import annotations

import pathlib

import proto_ref as ref

OUT = pathlib.Path(__file__).resolve().parent.parent / "tests" / "proto" / "golden_vectors.h"

CASES = [
    ("empty_cmd",      ref.TYPE_CMD,  ref.FLAG_NONE,     0x0000, b""),
    ("empty_rsp",      ref.TYPE_RSP,  ref.FLAG_NONE,     0x0001, b""),
    ("evt_short",      ref.TYPE_EVT,  ref.FLAG_NONE,     0x00FF, b"\x01\x02\x03"),
    ("data_settling",  ref.TYPE_DATA, ref.FLAG_SETTLING, 0x1234, b"\xde\xad\xbe\xef"),
    ("data_overrun",   ref.TYPE_DATA, ref.FLAG_OVERRUN,  0xFFFF, bytes(range(16))),
    ("data_seq_wrap",  ref.TYPE_DATA, ref.FLAG_NONE,     0xFFFF, b"\x00"),
    ("data_27b_frame", ref.TYPE_DATA, ref.FLAG_NONE,     0x0042, bytes(range(27))),
    ("data_large",     ref.TYPE_DATA, ref.FLAG_NONE,     0x0100, bytes((i * 7) & 0xFF for i in range(512))),
]


def c_array(data: bytes) -> str:
    if not data:
        return "{0}"
    rows = []
    for i in range(0, len(data), 12):
        rows.append(", ".join(f"0x{b:02x}" for b in data[i:i + 12]))
    return "{\n\t\t" + ",\n\t\t".join(rows) + "\n\t}"


def main() -> None:
    lines = [
        "/*",
        " * GENERATED FILE - DO NOT EDIT BY HAND.",
        " *",
        " * Produced by tools/gen_golden.py from tools/proto_ref.py, which is the",
        " * authoritative definition of the wire format. If the C codec disagrees",
        " * with these bytes, the C codec is wrong.",
        " *",
        " * Regenerate:  python tools/gen_golden.py",
        " */",
        "#ifndef SWIFTEEG_GOLDEN_VECTORS_H",
        "#define SWIFTEEG_GOLDEN_VECTORS_H",
        "",
        "#include <stdint.h>",
        "",
        "struct golden_vector {",
        "\tconst char *name;",
        "\tuint8_t  type;",
        "\tuint8_t  flags;",
        "\tuint16_t seq;",
        "\tconst uint8_t *payload;",
        "\tuint16_t payload_len;",
        "\tconst uint8_t *frame;",
        "\tuint16_t frame_len;",
        "};",
        "",
    ]

    for name, type_, flags, seq, payload in CASES:
        frame = ref.encode(type_, flags, seq, payload)
        # round-trip through the reference decoder as a sanity check
        back = ref.decode(frame)
        assert back.payload == payload and back.seq == seq and back.type == type_

        if payload:
            lines.append(f"static const uint8_t gv_{name}_payload[] = {c_array(payload)};")
        lines.append(f"static const uint8_t gv_{name}_frame[] = {c_array(frame)};")
        lines.append("")

    lines.append("static const struct golden_vector golden_vectors[] = {")
    for name, type_, flags, seq, payload in CASES:
        frame = ref.encode(type_, flags, seq, payload)
        pay_ptr = f"gv_{name}_payload" if payload else "NULL"
        lines.append(
            f'\t{{ "{name}", 0x{type_:02x}, 0x{flags:02x}, 0x{seq:04x}, '
            f"{pay_ptr}, {len(payload)}, gv_{name}_frame, {len(frame)} }},"
        )
    lines.append("};")
    lines.append("")
    lines.append(f"#define GOLDEN_VECTOR_COUNT {len(CASES)}")
    lines.append("")
    lines.append("/* CRC-16/CCITT-FALSE standard check value for \"123456789\". */")
    lines.append(f"#define GOLDEN_CRC_CHECK 0x{ref.crc16(b'123456789'):04x}u")
    lines.append("")
    lines.append("#endif /* SWIFTEEG_GOLDEN_VECTORS_H */")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    total = sum(len(ref.encode(t, f, s, p)) for _, t, f, s, p in CASES)
    print(f"wrote {OUT} ({len(CASES)} vectors, {total} frame bytes)")


if __name__ == "__main__":
    main()
