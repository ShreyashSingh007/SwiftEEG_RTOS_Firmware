"""Collect the facts a handoff document needs, in one pass."""
import ast
import json
import pathlib
import subprocess

REPO = pathlib.Path(r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS")
SCRATCH = pathlib.Path(__file__).resolve().parent
MEM = pathlib.Path(r"C:\Users\shrey\.claude\projects\D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware\memory")
PROJ = MEM.parent
PLAN = pathlib.Path(r"C:\Users\shrey\.claude\plans\d-electronics-projects-eeg-project-swif-peppy-peach.md")


def git(*args):
    r = subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True)
    return (r.stdout + r.stderr).strip()


def first_doc_line(path):
    try:
        mod = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
        doc = ast.get_docstring(mod)
        if doc:
            for line in doc.strip().splitlines():
                if line.strip():
                    return line.strip()
    except Exception as exc:  # noqa: BLE001
        return f"(unparsed: {exc.__class__.__name__})"
    return "(no docstring)"


print("### GIT")
print(git("branch", "-a", "-vv"))
print(git("remote", "-v"))
print("-- unpushed on m1-bringup vs SwiftEEG_RTOS/m1-bringup:")
print(git("log", "--oneline", "SwiftEEG_RTOS/m1-bringup..m1-bringup"))
print("-- feature/bcg-vitals only:")
print(git("log", "--oneline", "m1-bringup..feature/bcg-vitals"))
print("-- last 45 on m1-bringup:")
print(git("log", "--oneline", "-45", "m1-bringup"))
print("-- status:")
print(git("status", "--short"))
print(git("diff", "--stat"))

print("\n### REPO TOOLS")
for p in sorted((REPO / "tools").glob("*")):
    if p.is_file():
        desc = first_doc_line(p) if p.suffix == ".py" else ""
        print(f"{p.name:28s} {desc}")

print("\n### SCRATCHPAD FILES")
for p in sorted(SCRATCH.rglob("*")):
    if p.is_dir() or "__pycache__" in p.parts:
        continue
    rel = p.relative_to(SCRATCH).as_posix()
    desc = first_doc_line(p) if p.suffix == ".py" else ""
    print(f"{rel:60s} {p.stat().st_size:>8d}  {desc}")

print("\n### MEMORY")
for p in sorted(MEM.glob("*.md")):
    text = p.read_text(encoding="utf-8", errors="replace")
    desc = ""
    for line in text.splitlines():
        if line.startswith("description:"):
            desc = line[len("description:"):].strip().strip('"')
            break
    print(f"{p.name:40s} {desc}")

print("\n### TRANSCRIPTS")
for p in sorted(PROJ.glob("*.jsonl"), key=lambda q: q.stat().st_mtime):
    import datetime
    t = datetime.datetime.fromtimestamp(p.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
    print(f"{p.name}  {p.stat().st_size // 1024} KB  modified {t}")

print("\n### PLAN FILE", PLAN, "exists" if PLAN.exists() else "MISSING")

print("\n### RECORDINGS")
for p in sorted((REPO / "recordings").rglob("*")):
    if p.is_file():
        print(f"{p.relative_to(REPO).as_posix():60s} {p.stat().st_size // 1024} KB")

print("\n### DOCS")
for p in sorted(REPO.rglob("*.md")):
    if "build" in p.parts or "receipts" in p.parts[-2:]:
        continue
    print(p.relative_to(REPO).as_posix(), sum(1 for _ in p.open(encoding="utf-8", errors="replace")), "lines")

for name in ("FIRMWARE.md", "C-PRO.md"):
    f = SCRATCH / "phaseA" / name
    print(f"\n### PHASE A LOG {name}")
    print(f.read_text(encoding="utf-8", errors="replace") if f.exists() else "(missing)")

f = SCRATCH / "phaseA" / "PYTHON-PRO.md"
print("\n### PHASE A LOG PYTHON-PRO.md (after the link section)")
if f.exists():
    t = f.read_text(encoding="utf-8", errors="replace")
    i = t.find("- python-pro-work/usb_states_test.py")
    print(t[i:] if i >= 0 else t[-3000:])
