#!/usr/bin/env python3
"""Two-mode regression check for the mk2cpp integration (cross-platform).

Default GT interpreter vs ``-mk2cpp`` translated core, same scenario/window:
traces must be identical, state hashes must be equal, and (for the frozen
scenarios) the stock trace must match ``tools/baselines``.

GT never self-exits: the runner polls for the ``-hashdump`` file and a closed
trace window, then force-kills the process. ``-nomidi`` is always passed so
host MIDI input cannot inject external bytes (see tests/README.md).

Portability: Python 3 standard library only, no platform-specific paths.

Usage::

    python3 mk2cpp/tests/two_mode_check.py                     # boot [0,3M)
    python3 mk2cpp/tests/two_mode_check.py --scenario demo200  # [200M,202M)
    python3 mk2cpp/tests/two_mode_check.py --scenario custom \\
        --args -demo --window-from 0 --window-to 3000000 --hash-at 3000000

Exit codes: 0 = PASS, 1 = FAIL, 2 = setup error.
"""

import argparse
import hashlib
import os
import pathlib
import re
import subprocess
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
TRACE_TAIL_RE = re.compile(r"^[ms]\s+(\d+)\s")

SCENARIOS = {
    "boot": dict(window=(0, 3_000_000), hash_at=3_000_000, extra=[],
                 baseline="trace_boot3m_base.txt"),
    "demo200": dict(window=(200_000_000, 202_000_000), hash_at=202_000_000,
                    extra=["-demo"], baseline="trace_200m_base.txt"),
}


def default_exe() -> pathlib.Path:
    name = "nuked-sc55.exe" if os.name == "nt" else "nuked-sc55"
    return REPO_ROOT / "build" / name


def gt_env() -> dict:
    env = dict(os.environ)
    env["SDL_AUDIODRIVER"] = "dummy"
    env["SDL_VIDEODRIVER"] = "dummy"
    env["SDL_RENDER_DRIVER"] = "software"
    return env


def sha256_file(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def trace_tail_cycle(path: pathlib.Path):
    try:
        size = path.stat().st_size
        with open(path, "rb") as f:
            f.seek(max(0, size - 8192))
            tail = f.read().decode("ascii", "ignore").splitlines()
    except OSError:
        return None
    for line in reversed(tail):
        m = TRACE_TAIL_RE.match(line)
        if m:
            return int(m.group(1))
    return None


def first_diff_line(a: pathlib.Path, b: pathlib.Path):
    try:
        with open(a, "r", encoding="ascii", errors="replace") as fa, \
             open(b, "r", encoding="ascii", errors="replace") as fb:
            for n, (la, lb) in enumerate(zip(fa, fb), 1):
                if la != lb:
                    return n, la.strip(), lb.strip()
            ra, rb = fa.readline(), fb.readline()
            if ra != rb:
                return n + 1 if 'n' in dir() else 1, ra.strip(), rb.strip()
    except OSError:
        return None
    return None


def wait_outputs(proc: subprocess.Popen, trace: pathlib.Path, hashf: pathlib.Path,
                 timeout: int, trace_end: int) -> str:
    tail_floor = trace_end - 12 if trace_end > 12 else 0
    deadline = time.time() + timeout
    last_size, stable = -1, 0
    while time.time() < deadline:
        hash_ok = hashf.exists() and hashf.stat().st_size > 0
        trace_ok = False
        if trace.exists():
            size = trace.stat().st_size
            stable = stable + 1 if size == last_size else 0
            last_size = size
            tc = trace_tail_cycle(trace)
            trace_ok = (tc is not None and tc >= tail_floor) or (size > 0 and stable >= 4)
        else:
            stable = 0
        if hash_ok and trace_ok:
            return "ready"
        if proc.poll() is not None:
            return "exited"
        time.sleep(0.4)
    return "timeout"


def run_mode(exe: pathlib.Path, mode_args, trace: pathlib.Path, hashf: pathlib.Path,
             outdir: pathlib.Path, label: str, win_from: int, win_to: int,
             hash_at: int, timeout: int):
    for p in (trace, hashf):
        try:
            p.unlink()
        except FileNotFoundError:
            pass
    argv = [str(exe)] + mode_args + [
        "-tracepc", str(trace), str(win_from), str(win_to),
        "-hashdump", str(hash_at), str(hashf),
    ]
    so = open(outdir / f"{label}.stdout.txt", "wb")
    se = open(outdir / f"{label}.stderr.txt", "wb")
    proc = subprocess.Popen(argv, cwd=str(REPO_ROOT), env=gt_env(), stdout=so, stderr=se)
    try:
        status = wait_outputs(proc, trace, hashf, timeout, win_to)
    finally:
        if proc.poll() is None:
            proc.kill()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
        so.close()
        se.close()
    return {
        "label": label, "status": status,
        "trace": trace, "hash": hashf,
        "trace_ok": trace.exists() and trace.stat().st_size > 0,
        "hash_ok": hashf.exists() and hashf.stat().st_size > 0,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=sorted(SCENARIOS) + ["custom"], default="boot")
    ap.add_argument("--args", nargs=argparse.REMAINDER, default=[],
                    help="extra GT args for --scenario custom")
    ap.add_argument("--window-from", type=int, default=0)
    ap.add_argument("--window-to", type=int, default=0)
    ap.add_argument("--hash-at", type=int, default=0)
    ap.add_argument("--timeout", type=int, default=120, help="per-process hard timeout (s)")
    ap.add_argument("--exe", default=None)
    ap.add_argument("--outdir", default=str(REPO_ROOT / "mk2cpp" / "out" / "twomode"))
    args = ap.parse_args()

    exe = pathlib.Path(args.exe) if args.exe else default_exe()
    if not exe.exists():
        print(f"ERROR: GT executable missing: {exe}", file=sys.stderr)
        return 2

    baseline_name = None
    if args.scenario == "custom":
        if args.window_to <= args.window_from:
            print("ERROR: custom scenario needs --window-to > --window-from", file=sys.stderr)
            return 2
        win_from, win_to = args.window_from, args.window_to
        hash_at = args.hash_at or win_to
        extra = list(args.args)
        label = "custom"
    else:
        spec = SCENARIOS[args.scenario]
        win_from, win_to = spec["window"]
        hash_at = spec["hash_at"]
        extra = spec["extra"]
        baseline_name = spec["baseline"]
        label = args.scenario

    outdir = pathlib.Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    def trace_path(name):
        return outdir / name

    def hash_path(name):
        return outdir / name

    def_label = "def"
    tr_label = "tr.mk2cpp"
    print(f"scenario={label} window=[{win_from},{win_to}) hash_at={hash_at} timeout={args.timeout}s")

    stock = run_mode(exe, ["-mk2", "-nomidi"] + extra,
                     trace_path(f"{def_label}.trace"), hash_path(f"{def_label}.hash"),
                     outdir, def_label, win_from, win_to, hash_at, args.timeout)
    tr = run_mode(exe, ["-mk2", "-nomidi", "-mk2cpp"] + extra,
                  trace_path(f"{tr_label}.trace"), hash_path(f"{tr_label}.hash"),
                  outdir, tr_label, win_from, win_to, hash_at, args.timeout)

    failures = 0

    for run in (stock, tr):
        if run["status"] != "ready" or not run["trace_ok"] or not run["hash_ok"]:
            print(f"run:{run['label']:<14} FAIL  status={run['status']} trace={run['trace_ok']} hash={run['hash_ok']}")
            failures += 1
        else:
            print(f"run:{run['label']:<14} PASS")

    if stock["trace_ok"] and tr["trace_ok"]:
        same = sha256_file(stock["trace"]) == sha256_file(tr["trace"])
        if same:
            print("trace:def-vs-tr        PASS   identical")
        else:
            d = first_diff_line(stock["trace"], tr["trace"])
            print(f"trace:def-vs-tr        FAIL   first diff: {d}")
            failures += 1

    if stock["hash_ok"] and tr["hash_ok"]:
        hs, ht = sha256_file(stock["hash"]), sha256_file(tr["hash"])
        if hs == ht:
            print(f"hash:def-vs-tr         PASS   sha256={hs}")
        else:
            print(f"hash:def-vs-tr         FAIL   def={hs} tr={ht}")
            failures += 1

    if baseline_name and stock["trace_ok"]:
        baseline = REPO_ROOT / "tools" / "baselines" / baseline_name
        if not baseline.exists():
            print(f"trace:def-vs-baseline  SKIP   missing {baseline_name}")
        elif sha256_file(baseline) == sha256_file(stock["trace"]):
            print(f"trace:def-vs-baseline  PASS   {baseline_name}")
        else:
            d = first_diff_line(baseline, stock["trace"])
            print(f"trace:def-vs-baseline  FAIL   first diff: {d}")
            failures += 1

    print()
    if failures:
        print(f"RESULT: FAIL ({failures} check(s); outputs in {outdir})")
        return 1
    print(f"RESULT: PASS (outputs in {outdir})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
