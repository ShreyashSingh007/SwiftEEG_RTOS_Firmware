"""Copy session tools and phase A logs into the repo; link HANDOFF.md from WORKLOG and README."""
import pathlib
import shutil

REPO = pathlib.Path(r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS")
SCRATCH = pathlib.Path(__file__).resolve().parent


def copy_text(src, dst):
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_bytes(src.read_bytes().replace(b"\r\n", b"\n"))


copied = 0
# Session scripts at the scratchpad root.
for src in sorted(SCRATCH.glob("*.py")):
    copy_text(src, REPO / "tools" / "lab" / src.name)
    copied += 1

# Review reproductions and phase A proofs, keeping their folder layout.
for sub, dest in (("review/R3-DSP-work", "review-2026-09-17/R3-DSP-work"),
                  ("review/R4-HOST-work", "review-2026-09-17/R4-HOST-work"),
                  ("phaseA/c-pro-work", "phaseA/c-pro-work"),
                  ("phaseA/python-pro-work", "phaseA/python-pro-work")):
    base = SCRATCH / sub
    for src in sorted(base.rglob("*.py")):
        if "__pycache__" in src.parts:
            continue
        copy_text(src, REPO / "tools" / "lab" / dest / src.relative_to(base))
        copied += 1

# Phase A progress logs, snapshot.
for name in ("FIRMWARE.md", "PYTHON-PRO.md", "C-PRO.md"):
    src = SCRATCH / "phaseA" / name
    if src.exists():
        copy_text(src, REPO / "docs" / "handoff" / "phaseA" / name)

# Loss-matrix captures behind the 1000 SPS fix: recordings/ is git-ignored.
for variant in ("charged", "fixed", "txthread", "final"):
    for src in sorted((SCRATCH / f"rate1000_{variant}").glob("*.npz")):
        dst = REPO / "recordings" / "rate1000_2026-09-16" / variant / src.name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

# Links to the handoff.
w = REPO / "WORKLOG.md"
s = w.read_bytes().decode("utf-8").replace("\r\n", "\n")
old = "- **Work on:** `m1-bringup` - the milestone line.\n"
new = ("- **Work on:** `m1-bringup` - the milestone line.\n"
       "- **Handoff:** `HANDOFF.md` - everything a new engineer or assistant needs to\n"
       "  resume, including phase A's in-progress state and the session tools.\n")
assert s.count(old) == 1
w.write_bytes(s.replace(old, new, 1).encode("utf-8"))

r = REPO / "README.md"
s = r.read_bytes().decode("utf-8").replace("\r\n", "\n")
old = ("The running work log - what was done last, which branch, what is waiting\n"
       "on whom - is `WORKLOG.md`. This file is the reference.\n")
new = ("The running work log - what was done last, which branch, what is waiting\n"
       "on whom - is `WORKLOG.md`. A full handoff for a new engineer or assistant\n"
       "is `HANDOFF.md`. This file is the reference.\n")
assert s.count(old) == 1
r.write_bytes(s.replace(old, new, 1).encode("utf-8"))

print("copied", copied, "scripts")
