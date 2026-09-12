#!/usr/bin/env python3
"""M4 oracle -- n=28 audio null test (cross-platform Python port).

Stock interpreter vs M4 native engine (``-mk2cpp -voices:28``) over a fixed
cycle window. Dry-run by default: validates arguments, probes the GT build for
the frozen oracle options and prints the exact GT commands, timeouts and the
comparison plan. It starts NO GT process.

With ``--execute`` (and ``--user-present``) it starts the real GT per mode --
LCD window and audio ON, never SDL dummy -- polls for the audio window's
``.meta`` marker plus ``.audiohash``/``.state.hash``, then force-kills the
process. It compares stock vs M4 captures:

1. ``-audiohash`` (deterministic FNV-1a64 at a fixed cycle) -- preferred.
2. WAV PCM payload SHA256 / ``pcmdiff --tolerance 0`` byte compare.
3. ``.meta`` window/rate/length/payload-fnv equality.
4. layout-independent state scalars from ``-hashdump`` (W3 gate 3 helper).
5. ``LCDEN 0`` reset-loop heuristic from stdout.
6. CPU duty report (process CPU / wall; INFO only).

With ``--hand-off`` it adds the same-binary A/B run
(``-mk2cpp -voices:28 -mk2cpp-hand:0``) and compares it against the hand-on run
(W3 gate 5).

With ``--check-baseline`` it additionally gates on the frozen stock capture in
``tools/baselines/m4_audio`` (G6): first ``SHA256SUMS.txt`` is re-verified
(mismatch/missing listed file = FAIL), then the stock run's WAV payload SHA256
and ``-audiohash`` are compared against the frozen files (differences = FAIL).
A missing baseline directory/manifest/files = SKIP, never FAIL.

Portability: Python 3 standard library only, no platform-specific paths; the
GT executable is auto-detected (``build/nuked-sc55[.exe]``) and can be
overridden with ``--exe``. Real runs always pass ``-nomidi`` so host MIDI
input cannot inject external bytes (see tests/README.md).

Usage::

    python3 mk2cpp/tests/m4_audio_null.py                     # dry-run
    python3 mk2cpp/tests/m4_audio_null.py --require-ready     # exit 2 until GT ready
    python3 mk2cpp/tests/m4_audio_null.py --execute --user-present
    python3 mk2cpp/tests/m4_audio_null.py --check-baseline --execute --user-present
    python3 mk2cpp/tests/m4_audio_null.py --scenario midi \
        --midi-schedule mk2cpp/out/m4/corpus/midi037.sched --hand-off \
        --execute --user-present

Exit codes: 0 = PASS/SKIP (automated part), 1 = FAIL, 2 = setup error.

NOTE: the A/B listen is manual and cannot be automated; the user must be
present for real runs.
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
SCALAR_KEYS = ["mcu.cp", "mcu.pc", "mcu.sr", "mcu.cycles",
               "pcm.config_reg_3c", "pcm.config_reg_3d", "pcm.irq_assert"]
META_FIELDS = ["start_cycles", "end_cycles", "rate", "channels", "format", "sample_pairs"]


def default_exe() -> pathlib.Path:
    name = "nuked-sc55.exe" if os.name == "nt" else "nuked-sc55"
    return REPO_ROOT / "build" / name


def default_pcmdiff() -> pathlib.Path:
    name = "pcmdiff.exe" if os.name == "nt" else "pcmdiff"
    return REPO_ROOT / "mk2cpp" / "tools" / "pcmdiff" / name


def sha256_file(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().lower()


def short(h: str) -> str:
    return h[:16] if h else "?"


def gt_timeout_sec(cycles: int) -> int:
    return max(10, int(-(-(cycles / 24e6 * 2.0 + 15.0) // 1)))


def read_kv(path: pathlib.Path) -> dict:
    out = {}
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                m = re.match(r"^\s*([A-Za-z0-9_.]+)\s*=\s*(\S+)\s*$", line)
                if m:
                    out[m.group(1)] = m.group(2)
    except OSError:
        pass
    return out


def read_audio_hash(path: pathlib.Path) -> str:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                m = re.search(r"audio_fnv1a\s*=\s*([0-9a-fA-F]+)", line)
                if m:
                    return m.group(1).lower()
    except OSError:
        pass
    return ""


def wav_data_info(path: pathlib.Path):
    """Return (offset, declared, actual, bytes) of the RIFF data chunk, or None."""
    try:
        data = path.read_bytes()
    except OSError:
        return None
    if len(data) < 44:
        return None
    pos = 12
    while pos + 8 <= len(data):
        cid = data[pos:pos + 4]
        size = int.from_bytes(data[pos + 4:pos + 8], "little")
        remaining = len(data) - (pos + 8)
        if cid == b"data":
            return pos + 8, size, remaining, data
        if size > remaining:
            return None
        pos += 8 + size + (size & 1)
    return None


def wav_payload_hash(info) -> str:
    if info is None:
        return ""
    offset, declared, actual, data = info
    if declared == 0 and actual > 0:
        return ""
    length = min(declared, actual)
    if length <= 0:
        return ""
    return hashlib.sha256(data[offset:offset + length]).hexdigest().lower()


class Oracle:
    def __init__(self, args):
        self.args = args
        self.exe = pathlib.Path(args.exe) if args.exe else default_exe()
        self.pcmdiff = default_pcmdiff()
        rooted = lambda p: pathlib.Path(p) if pathlib.Path(p).is_absolute() else REPO_ROOT / p
        self.outdir = rooted(args.outdir)
        self.baseline_dir = rooted(args.baseline_dir)
        self.checks = []
        self.missing = []
        self.caps = {}

    # ---- setup ----
    def fail_setup(self, msg):
        print(f"ERROR: {msg}")
        sys.exit(2)

    def probe(self):
        try:
            p = subprocess.run([str(self.exe), "-h"], capture_output=True, text=True, timeout=30)
            help_text = (p.stdout or "") + (p.stderr or "")
        except Exception:
            help_text = ""
        self.caps = {
            "wav": "-wav:" in help_text,
            "audiowin": "-audiowin" in help_text,
            "audiohash": "-audiohash" in help_text,
            "midiseq": "-midiseq" in help_text,
            "hand": "-mk2cpp-hand" in help_text,
        }
        req = []
        if not self.caps["wav"]:
            req.append("-wav:<file>")
        if not self.caps["audiowin"]:
            req.append("-audiowin <start> <end>")
        if not self.caps["audiohash"]:
            req.append("-audiohash <cycles> <file>")
        if self.args.scenario == "midi" and not self.caps["midiseq"]:
            req.append("-midiseq <file> [start]")
        if self.args.hand_off and not self.caps["hand"]:
            req.append("-mk2cpp-hand:0|1")
        self.missing = req

    def validate(self):
        if not self.exe.exists():
            self.fail_setup(f"GT exe not found: {self.exe}")
        if self.args.audio_end <= self.args.audio_start:
            self.fail_setup(f"invalid audio window [{self.args.audio_start},{self.args.audio_end})")
        if self.args.scenario == "midi":
            if not self.args.midi_schedule:
                self.fail_setup("--scenario midi requires --midi-schedule <file>")
            if not pathlib.Path(self.args.midi_schedule).exists():
                self.fail_setup(f"MIDI schedule not found: {self.args.midi_schedule}")

    # ---- artifacts ----
    def paths(self):
        o = self.outdir
        return {
            "stock_wav": o / "stock.wav",
            "m4_wav": o / "m4.wav",
            "handoff_wav": o / "handoff.wav",
            "stock_hash": o / "stock.audiohash",
            "m4_hash": o / "m4.audiohash",
            "handoff_hash": o / "handoff.audiohash",
            "stock_state": o / "stock.state.hash",
            "m4_state": o / "m4.state.hash",
            "handoff_state": o / "handoff.state.hash",
            "summary": o / "summary.txt",
            "plan": o / "plan.txt",
        }

    def baseline_paths(self):
        stem = self.args.baseline_name
        return {
            "dir": self.baseline_dir,
            "wav": self.baseline_dir / f"{stem}.wav",
            "hash": self.baseline_dir / f"{stem}.audiohash",
            "sums": self.baseline_dir / "SHA256SUMS.txt",
        }

    # ---- command builders ----
    def mode_args(self, mode, wav, hash_path, state_path):
        a = ["-mk2", "-nomidi"]
        if mode in ("m4", "m4handoff"):
            a += ["-mk2cpp", "-voices:28"]
        if mode == "m4handoff":
            a += ["-mk2cpp-hand:0"]
        if self.args.scenario == "demo":
            a += ["-demo"]
        else:
            a += ["-midiseq", self.args.midi_schedule, "200000000"]
        if self.args.gain:
            a += [f"-gain:{self.args.gain}"]
        a += [f"-wav:{wav}", "-audiowin", str(self.args.audio_start), str(self.args.audio_end),
              "-audiohash", str(self.args.audio_end), str(hash_path),
              "-hashdump", str(self.args.state_at), str(state_path)]
        return a

    def build_commands(self):
        p = self.paths()
        cmds = {
            "stock": self.mode_args("stock", p["stock_wav"], p["stock_hash"], p["stock_state"]),
            "m4": self.mode_args("m4", p["m4_wav"], p["m4_hash"], p["m4_state"]),
        }
        if self.args.hand_off:
            cmds["handoff"] = self.mode_args("m4handoff", p["handoff_wav"],
                                             p["handoff_hash"], p["handoff_state"])
        return cmds

    # ---- baseline ----
    def baseline_status(self):
        bp = self.baseline_paths()
        if not bp["dir"].exists():
            return "SKIP", f"baseline dir missing: {bp['dir']}"
        if not bp["sums"].exists():
            return "SKIP", f"SHA256SUMS.txt missing: {bp['sums']}"
        bad, count = [], 0
        try:
            with open(bp["sums"], "r", encoding="utf-8", errors="replace") as f:
                for line in f:
                    m = re.match(r"^\s*([0-9a-fA-F]{64})\s+(.+?)\s*$", line)
                    if not m:
                        continue
                    count += 1
                    expected, name = m.group(1).lower(), m.group(2)
                    target = bp["dir"] / name
                    if not target.exists():
                        bad.append(f"{name}: missing")
                        continue
                    if sha256_file(target) != expected:
                        bad.append(f"{name}: sha256 mismatch")
        except OSError as e:
            return "SKIP", f"SHA256SUMS unreadable: {e}"
        if count == 0:
            return "SKIP", "SHA256SUMS.txt has no parsable entries"
        if bad:
            return "FAIL", "SHA256SUMS mismatch: " + "; ".join(bad)
        return "PASS", f"SHA256SUMS verified ({count} files)"

    # ---- run ----
    def run_mode(self, label, argv):
        p = self.paths()
        key = {"stock": "stock", "m4": "m4", "handoff": "handoff"}[label]
        wav = p[f"{key}_wav"]
        hash_path = p[f"{key}_hash"]
        state = p[f"{key}_state"]
        meta = pathlib.Path(str(wav) + ".meta")
        stdout = self.outdir / f"{label}.stdout.txt"
        stderr = self.outdir / f"{label}.stderr.txt"
        for f in (wav, meta, hash_path, state, stdout, stderr):
            try:
                f.unlink()
            except FileNotFoundError:
                pass
        print(f"[{label}] " + " ".join(argv))
        env = dict(os.environ)
        env.pop("SDL_VIDEODRIVER", None)   # real window
        env.pop("SDL_AUDIODRIVER", None)   # real audio
        so = open(stdout, "wb")
        se = open(stderr, "wb")
        t0 = time.time()
        proc = subprocess.Popen(argv, cwd=str(REPO_ROOT / "build"), env=env, stdout=so, stderr=se)
        status = "timeout"
        cpu_sec = None
        try:
            deadline = time.time() + self.args.timeout
            while True:
                if meta.exists() and hash_path.exists() and hash_path.stat().st_size > 0 \
                        and state.exists() and state.stat().st_size > 0 \
                        and wav.exists() and wav.stat().st_size > 0:
                    status = "ready"
                    break
                if proc.poll() is not None:
                    status = "exited"
                    break
                if time.time() >= deadline:
                    status = "timeout"
                    break
                time.sleep(0.5)
            try:
                cpu_sec = sum(proc.cpu_times()[:2], 0.0)
            except Exception:
                cpu_sec = None
        finally:
            if proc.poll() is None:
                proc.kill()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
            so.close()
            se.close()
        wall = time.time() - t0
        duty = f"{cpu_sec / wall * 100:.1f}%" if cpu_sec and wall > 0 else "n/a"
        return {"label": label, "status": status, "wav": wav, "hash": hash_path,
                "state": state, "stdout": stdout, "stderr": stderr,
                "wall": wall, "cpu": cpu_sec, "duty": duty}

    # ---- checks ----
    def add(self, check, result, detail=""):
        self.checks.append((check, result, detail))
        print(f"{check:<22} {result:<5} {detail}")

    def compare_wavs(self, ref, dut, report):
        ri, di = wav_data_info(ref), wav_data_info(dut)
        if ri is None or di is None:
            return "FAIL", "missing/unparsable WAV"
        if ri[1] == 0 and ri[2] > 0:
            return "FAIL", "ref WAV header not finalized (data size 0)"
        if di[1] == 0 and di[2] > 0:
            return "FAIL", "dut WAV header not finalized (data size 0)"
        rh, dh = wav_payload_hash(ri), wav_payload_hash(di)
        if rh and rh == dh:
            return "PASS", f"payload bit-exact sha256={short(rh)}"
        if self.pcmdiff.exists():
            try:
                p = subprocess.run([str(self.pcmdiff), "--ref", str(ref), "--dut", str(dut),
                                    "--tolerance", "0", "--report", str(report)],
                                   capture_output=True, text=True, timeout=300)
                code = p.returncode
                detail = f"pcmdiff exit={code}; see {report}"
                return ("PASS" if code == 0 else "FAIL"), detail
            except Exception as e:
                return "FAIL", f"pcmdiff failed: {e}"
        return "FAIL", f"WAV payload differs (ref={short(rh)} dut={short(dh)})"

    def run_checks(self, runs, cmds):
        p = self.paths()
        for run in runs.values():
            if run["status"] != "ready":
                self.add(f"run:{run['label']}", "FAIL", f"status={run['status']}")
            else:
                self.add(f"run:{run['label']}", "PASS",
                         f"meta/hash/state ready; wall={run['wall']:.1f}s cpu={run['cpu']}s duty={run['duty']}")

        h_stock = read_audio_hash(p["stock_hash"])
        h_m4 = read_audio_hash(p["m4_hash"])
        if not h_stock or not h_m4:
            self.add("audio:hash", "FAIL", "missing audio_fnv1a line")
        elif h_stock == h_m4:
            self.add("audio:hash", "PASS", f"fnv1a={short(h_stock)}")
        else:
            self.add("audio:hash", "FAIL", f"stock={short(h_stock)} m4={short(h_m4)}")

        meta_s = read_kv(pathlib.Path(str(p["stock_wav"]) + ".meta"))
        meta_m = read_kv(pathlib.Path(str(p["m4_wav"]) + ".meta"))
        diffs = [f"{k}: {meta_s.get(k)} != {meta_m.get(k)}" for k in META_FIELDS
                 if meta_s.get(k) != meta_m.get(k)]
        if diffs:
            self.add("audio:meta", "FAIL", "; ".join(diffs))
        else:
            self.add("audio:meta", "PASS",
                     f"pairs={meta_s.get('sample_pairs')} rate={meta_s.get('rate')} "
                     f"[{meta_s.get('start_cycles')},{meta_s.get('end_cycles')})")
        if meta_s.get("payload_fnv1a") and meta_m.get("payload_fnv1a"):
            if meta_s["payload_fnv1a"] != meta_m["payload_fnv1a"]:
                self.add("audio:meta-fnv", "FAIL", "payload_fnv1a differs")
            else:
                self.add("audio:meta-fnv", "PASS", f"payload_fnv1a={short(meta_s['payload_fnv1a'])}")

        res, detail = self.compare_wavs(p["stock_wav"], p["m4_wav"], self.outdir / "audio_diff.md")
        self.add("audio:null", res, detail)

        if self.args.check_baseline:
            self.baseline_checks(runs["stock"])

        s_stock = read_kv(p["stock_state"])
        s_m4 = read_kv(p["m4_state"])
        if not s_stock or not s_m4:
            self.add("state:scalars", "FAIL", "missing hashdump")
        else:
            sd = [f"{k}: {s_stock.get(k)} != {s_m4.get(k)}" for k in SCALAR_KEYS
                  if s_stock.get(k) != s_m4.get(k)]
            if sd:
                self.add("state:scalars", "FAIL", "; ".join(sd))
            else:
                self.add("state:scalars", "PASS",
                         f"cycles={s_stock.get('mcu.cycles')} cfg3d={s_stock.get('pcm.config_reg_3d')}")

        for run in runs.values():
            try:
                text = run["stdout"].read_text(encoding="utf-8", errors="replace")
            except OSError:
                self.add(f"lcd:{run['label']}", "FAIL", "stdout missing")
                continue
            n = text.count("LCDEN 0")
            if n <= 2:
                self.add(f"lcd:{run['label']}", "PASS", f"LCDEN 0 count={n}")
            else:
                self.add(f"lcd:{run['label']}", "FAIL", f"LCDEN 0 count={n} suspects reset loop")

        if self.args.hand_off:
            h_off = read_audio_hash(p["handoff_hash"])
            if h_off and h_m4 and h_off == h_m4:
                self.add("audio:handoff", "PASS", f"fnv1a={short(h_off)}")
            else:
                self.add("audio:handoff", "FAIL", "hand-off vs hand-on audiohash differs")
            res, detail = self.compare_wavs(p["m4_wav"], p["handoff_wav"],
                                            self.outdir / "audio_diff_handoff.md")
            self.add("audio:handoff-wav", res, detail)
            s_off = read_kv(p["handoff_state"])
            if not s_off:
                self.add("state:handoff", "FAIL", "missing hashdump")
            else:
                hd = [f"{k}: {s_off.get(k)} != {s_m4.get(k)}" for k in SCALAR_KEYS
                      if s_off.get(k) != s_m4.get(k)]
                if hd:
                    self.add("state:handoff", "FAIL", "; ".join(hd))
                else:
                    self.add("state:handoff", "PASS", "handoff scalars == hand-on scalars")

    def baseline_checks(self, stock_run):
        bp = self.baseline_paths()
        result, detail = self.baseline_status()
        if result == "SKIP":
            self.add("baseline:integrity", "SKIP", detail)
            self.add("baseline:wav", "SKIP", "frozen baseline absent; nothing to compare")
            self.add("baseline:audiohash", "SKIP", "frozen baseline absent; nothing to compare")
            return
        self.add("baseline:integrity", result, detail)

        base_info = wav_data_info(bp["wav"])
        if base_info is None:
            self.add("baseline:wav", "SKIP", f"frozen WAV missing/unparsable: {bp['wav']}")
        else:
            base_hash = wav_payload_hash(base_info)
            stock_hash = wav_payload_hash(wav_data_info(self.paths()["stock_wav"]))
            if base_hash and base_hash == stock_hash:
                self.add("baseline:wav", "PASS", f"payload bit-exact sha256={short(base_hash)}")
            else:
                self.add("baseline:wav", "FAIL",
                         f"payload differs (frozen={short(base_hash)} stock={short(stock_hash)})")

        base_ah = read_audio_hash(bp["hash"])
        if not base_ah:
            self.add("baseline:audiohash", "SKIP", f"frozen audiohash missing/unparsable: {bp['hash']}")
        else:
            stock_ah = read_audio_hash(stock_run["hash"])
            if base_ah == stock_ah:
                self.add("baseline:audiohash", "PASS", f"fnv1a={short(base_ah)}")
            else:
                self.add("baseline:audiohash", "FAIL",
                         f"frozen={short(base_ah)} stock={short(stock_ah)}")

    # ---- plan / caps ----
    def write_plan(self, cmds):
        lines = [
            "m4_audio_null.py -- plan (dry-run unless --execute)",
            f"scenario     : {self.args.scenario}",
            f"audio window : [{self.args.audio_start},{self.args.audio_end})",
            f"state sample : hashdump @ {self.args.state_at}",
            f"timeout      : {self.args.timeout}s  (ceil(End/24e6*2)+15)",
            f"outdir       : {self.outdir}",
            f"pcmdiff      : {'present' if self.pcmdiff.exists() else 'not built (SHA256 fallback)'}",
        ]
        if self.args.check_baseline:
            bp = self.baseline_paths()
            result, detail = self.baseline_status()
            lines += [f"baseline     : {bp['dir']} [{self.args.baseline_name}]",
                      f"baseline chk : {result} - {detail}"]
        lines.append("")
        for name, argv in cmds.items():
            lines.append(f"{name:<7}: " + " ".join(argv))
        lines += [
            "",
            "checks (automated, 10_m4_oracle.md 2.1-2.4 / W3 5.3):",
            "  1. -audiohash files exist and are equal (deterministic FNV-1a64)",
            "  2. WAV payload SHA256 / pcmdiff --tolerance 0 bit-exact (gate: 0 tolerance)",
            "  3. .meta windows/rate/length identical; WAV header finalized",
            "  4. layout-independent state scalars equal (mcu.pc/sr/cycles, pcm cfg 3c/3d)",
            "  5. stdout LCDEN 0 <= 2 (boot + demo power-cycle)",
            "  6. CPU duty report (process CPU / wall; INFO only)",
        ]
        if self.args.check_baseline:
            lines += [
                "  B0. frozen baseline integrity via SHA256SUMS.txt (mismatch = FAIL, missing = SKIP)",
                "  B1. stock WAV payload SHA256 == frozen WAV payload (bit-exact)",
                "  B2. stock -audiohash == frozen audio_fnv1a",
            ]
        if self.args.hand_off:
            lines.append("  7. W3 gate 5: handoff vs hand-on audiohash + WAV payload byte-identical")
        lines += [
            "checks (manual, user present):",
            "  A. LCD window renders; audio device is real (no SDL dummy)",
            "  B. A/B listen: stock vs M4 must be indistinguishable",
        ]
        text = "\n".join(lines) + "\n"
        (self.outdir / "plan.txt").write_text(text, encoding="utf-8")
        print(text)

    def capability_report(self):
        print(f'GT capability probe ("{self.exe}" -h):')
        rows = [
            ("wav", "-wav:<file>", "G1  (10_m4_oracle.md 2.2)"),
            ("audiowin", "-audiowin <start> <end>", "G1  (10_m4_oracle.md 2.2)"),
            ("audiohash", "-audiohash <cycles> <file>", "G1  (10_m4_oracle.md 2.2)"),
            ("midiseq", "-midiseq <file> [start]", "G5  (10_m4_oracle.md 3.1)"),
            ("hand", "-mk2cpp-hand:0|1", "W3  (09_m4_integration.md 5.4)"),
        ]
        for key, opt, ref in rows:
            state = "ok" if self.caps.get(key) else "MISSING"
            print(f"  {opt:<28} {state:<8} {ref}")

    def missing_report(self):
        if not self.missing:
            return
        print()
        print(f"ERROR: this GT build lacks {len(self.missing)} option(s) required by m4_audio_null.py:")
        for m in self.missing:
            print(f"  ERROR: missing GT option {m}")
        print("       implementer (Wave 0a/0b): land the frozen options listed in")
        print("       mk2cpp/docs/10_m4_oracle.md 2.2/5.1 (G1/G5) and advertise them in -h;")
        print("       -audiowin must backfill the RIFF/data sizes and fclose the WAV,")
        print("       then write <file>.meta last as the completion marker.")
        print("       Dry-run continues; --execute will refuse until these options exist.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", choices=["demo", "midi"], default="demo")
    ap.add_argument("--audio-start", type=int, default=300_000_000)
    ap.add_argument("--audio-end", type=int, default=320_000_000)
    ap.add_argument("--state-at", type=int, default=0, help="hashdump cycle (0 -> audio-end, must be >= 200M)")
    ap.add_argument("--timeout", type=int, default=0, help="hard timeout (0 -> ceil(End/24e6*2)+15)")
    ap.add_argument("--outdir", default=str(REPO_ROOT / "mk2cpp" / "out" / "m4" / "audio_null"))
    ap.add_argument("--midi-schedule", default="")
    ap.add_argument("--gain", default="")
    ap.add_argument("--check-baseline", action="store_true")
    ap.add_argument("--baseline-dir", default=str(REPO_ROOT / "tools" / "baselines" / "m4_audio"))
    ap.add_argument("--baseline-name", default="stock_300M_320M")
    ap.add_argument("--hand-off", action="store_true")
    ap.add_argument("--require-ready", action="store_true")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--user-present", action="store_true")
    ap.add_argument("--exe", default=None)
    args = ap.parse_args()

    oracle = Oracle(args)
    oracle.validate()
    oracle.probe()

    if args.state_at == 0:
        args.state_at = args.audio_end
    if args.state_at < 200_000_000:
        oracle.fail_setup(f"StateAt={args.state_at} is below the 200M-cycle state sampling floor")
    if args.timeout <= 0:
        args.timeout = gt_timeout_sec(args.audio_end)

    oracle.outdir.mkdir(parents=True, exist_ok=True)
    cmds = oracle.build_commands()
    oracle.write_plan(cmds)
    print()
    oracle.capability_report()
    oracle.missing_report()

    if not args.execute:
        if args.require_ready and oracle.missing:
            print(f"ERROR: --require-ready: {len(oracle.missing)} required GT option(s) missing.")
            return 2
        print()
        print(f"DRY-RUN OK ({len(oracle.missing)} required GT option(s) missing). "
              "Re-run with --execute --user-present when the GT side is ready.")
        return 0

    if not args.user_present:
        print("ERROR: real GT runs must be observed on site (LCD + audio).")
        print("       Re-run with --execute --user-present only when the user is present.")
        return 2
    if oracle.missing:
        print(f"ERROR: refusing to launch GT: {len(oracle.missing)} required option(s) missing (see above).")
        return 2

    runs = {}
    for label in ("stock", "m4"):
        runs[label] = oracle.run_mode(label, [str(oracle.exe)] + cmds[label])
    if args.hand_off:
        runs["handoff"] = oracle.run_mode("handoff", [str(oracle.exe)] + cmds["handoff"])

    oracle.run_checks(runs, cmds)

    print()
    print("== summary ==")
    width = max(len(c) for c, _, _ in oracle.checks) if oracle.checks else 0
    lines = [f"{c:<{width}}  {r:<4} {d}" for c, r, d in oracle.checks]
    table = "Check".ljust(width) + "  Result Detail\n" + "\n".join(lines)
    print(table)
    (oracle.outdir / "summary.txt").write_text(table + "\n", encoding="utf-8")

    fails = sum(1 for _, r, _ in oracle.checks if r == "FAIL")
    if fails:
        print(f"RESULT: FAIL ({fails} check(s); outputs in {oracle.outdir})")
        print("REMINDER: user must confirm the A/B listen before accepting the run.")
        return 1
    print(f"RESULT: PASS (automated part; outputs in {oracle.outdir})")
    print("REMINDER: A/B listen by the user is still required (plan_256.md 6).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
