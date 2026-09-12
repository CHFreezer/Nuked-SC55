#!/usr/bin/env python3
"""Default fast regression gate for mk2cpp hand/voice changes (cross-platform).

Runs the designated external-MIDI song (037 by default) in both modes
(stock ``-mk2`` vs ``-mk2 -mk2cpp``) concurrently and compares the full WAV
byte-for-byte (SHA256). Hard timeout, dummy SDL audio/video drivers.

``--demo`` switches to the built-in demo playlist with a 60 s budget
(window [144M, 1440M) cycles, 144M skips the boot animation; the full 5-song
playlist is never run by default).

Scope policy: 037 is the only external-MIDI regression song -- do not run the
song corpus or the stress matrix unless the user explicitly asks.

Portability: Python 3 standard library only (no bundled interpreter), no
platform-specific paths; GT executable is auto-detected per platform and can
be overridden with ``--exe``.

Usage::

    python3 mk2cpp/tests/m4_quick_gate.py
    python3 mk2cpp/tests/m4_quick_gate.py --demo
    python3 mk2cpp/tests/m4_quick_gate.py --sched path/to/midi037.sched
    python3 mk2cpp/tests/m4_quick_gate.py --exe build/Release/nuked-sc55

Exit codes: 0 = PASS (hashes equal), 1 = FAIL, 2 = setup error.
"""

import argparse
import hashlib
import os
import pathlib
import subprocess
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


def default_exe() -> pathlib.Path:
    name = "nuked-sc55.exe" if os.name == "nt" else "nuked-sc55"
    return REPO_ROOT / "build" / name


def sha256_file(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def gt_env() -> dict:
    env = dict(os.environ)
    env["SDL_AUDIODRIVER"] = "dummy"
    env["SDL_VIDEODRIVER"] = "dummy"
    env["SDL_RENDER_DRIVER"] = "software"
    return env


def wait_meta(proc: subprocess.Popen, meta: pathlib.Path, deadline: float) -> bool:
    while time.time() < deadline:
        if meta.exists() and meta.stat().st_size > 0:
            return True
        if proc.poll() is not None:
            return False
        time.sleep(0.2)
    return False


def tail_cycle(sched: pathlib.Path) -> int:
    last = ""
    with open(sched, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                last = line
    return int(last.split()[0])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sched", default=str(REPO_ROOT / "mk2cpp" / "out" / "m4" / "corpus" / "midi037.sched"),
                    help="schedule file (default: corpus/midi037.sched)")
    ap.add_argument("--demo", action="store_true", help="run the built-in demo instead of an external MIDI schedule")
    ap.add_argument("--start", type=int, default=200_000_000, help="audio window start cycle (MIDI mode)")
    ap.add_argument("--demo-start", type=int, default=144_000_000, help="demo window start (default 144M)")
    ap.add_argument("--demo-end", type=int, default=1_440_000_000, help="demo window end (default 1440M = 60 s)")
    ap.add_argument("--timeout", type=int, default=300, help="per-process hard timeout in seconds")
    ap.add_argument("--exe", default=None, help="GT executable (default: build/nuked-sc55[.exe])")
    ap.add_argument("--outdir", default=str(REPO_ROOT / "mk2cpp" / "out" / "quick_gate"),
                    help="output directory for wav/meta/logs")
    args = ap.parse_args()

    exe = pathlib.Path(args.exe) if args.exe else default_exe()
    if not exe.exists():
        print(f"ERROR: GT executable missing: {exe}", file=sys.stderr)
        return 2

    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    if args.demo:
        start, end = args.demo_start, args.demo_end
        src_args = ["-demo"]
        print(f"gate: demo win=[{start},{end}) ({(end - start) / 24e6:.1f}s emulated)")
    else:
        sched = pathlib.Path(args.sched)
        if not sched.exists():
            print(f"ERROR: schedule missing: {sched}", file=sys.stderr)
            print("       generate it with mk2cpp/tools/midisched from the 037 SMF, or pass --sched.", file=sys.stderr)
            return 2
        last = tail_cycle(sched)
        start, end = args.start, args.start + last + 2_000_000
        src_args = ["-midiseq", str(sched), str(start)]
        print(f"gate: sched={sched.name} last={last} win=[{start},{end})")

    modes = [
        ("stock", ["-mk2", "-nomidi"]),
        ("tr", ["-mk2", "-mk2cpp", "-nomidi"]),
    ]

    t0 = time.time()
    procs = {}
    for name, mode_args in modes:
        wav = outdir / f"{name}.wav"
        meta = outdir / f"{name}.wav.meta"
        for p in (wav, meta):
            try:
                p.unlink()
            except FileNotFoundError:
                pass
        argv = [str(exe)] + mode_args + src_args + [
            f"-wav:{wav}", "-audiowin", str(start), str(end),
        ]
        so = open(outdir / f"{name}.stdout.txt", "wb")
        se = open(outdir / f"{name}.stderr.txt", "wb")
        proc = subprocess.Popen(argv, cwd=str(REPO_ROOT), env=gt_env(), stdout=so, stderr=se)
        procs[name] = (proc, wav, meta, so, se)

    deadline = time.time() + args.timeout
    ok = True
    for name, (proc, _wav, meta, _so, _se) in procs.items():
        if not wait_meta(proc, meta, deadline):
            ok = False
        if proc.poll() is None:
            proc.kill()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
    for _name, (_proc, _wav, _meta, so, se) in procs.items():
        so.close()
        se.close()

    wall = time.time() - t0
    wavs = {name: procs[name][1] for name in procs}
    hashes = {}
    for name, wav in wavs.items():
        hashes[name] = sha256_file(wav) if wav.exists() and wav.stat().st_size > 0 else ""
    stderr_bytes = 0
    for name in procs:
        se_path = outdir / f"{name}.stderr.txt"
        if se_path.exists():
            stderr_bytes += se_path.stat().st_size

    if not ok:
        print(f"RESULT: FAIL (timeout/missing output, wall={wall:.1f}s)")
        for name, (_proc, _wav, meta, _so, _se) in procs.items():
            print(f"  {name}: meta={'ok' if meta.exists() else 'MISSING'}")
        return 1
    if hashes["stock"] and hashes["stock"] == hashes["tr"]:
        print(f"RESULT: PASS  sha256={hashes['stock']}  wall={wall:.1f}s  stderr={stderr_bytes}B")
        return 0
    print(f"RESULT: FAIL  stock={hashes['stock']} tr={hashes['tr']}  wall={wall:.1f}s")
    return 1


if __name__ == "__main__":
    sys.exit(main())
