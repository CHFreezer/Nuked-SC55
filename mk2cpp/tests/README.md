# mk2cpp/tests

Oracle scripts for the mk2cpp integration. `two_mode_check.ps1` runs stock GT
twice for one scenario and window — default interpreter vs `-mk2cpp` translated
core — then compares traces, state hashes and (when applicable) the frozen
`tools/baselines` fixture.

GT never self-exits, so the script polls for the `-hashdump` file and a
completed trace window and then force-kills the process. It is pure PowerShell
5.1 (no Python) and writes everything under `-OutDir` only.

## M4 oracle scripts (Wave 0c)

> **回滚说明（2026-09-11）**：256 复音扩展（`-voices:<n>` 等）已从 GT 回滚到原版
> 28 复音实现。`m4_stress_voices.ps1`（n=32/64/128/255）所依赖的 `-voices:` 现已无效，
> 该脚本**暂缓**（当前运行会静默按 28 复音跑，结果不代表压力测试）。
> `m4_audio_null.ps1` 的 n=28 音频 null 仍有效（`-voices:28` 现为 no-op，等价默认 28）。

| Script | Purpose | Real GT run? |
|---|---|---|
| `m4_audio_null.ps1` | n=28 audio null: stock vs `-mk2cpp`（`-voices:28` 已回滚为 no-op，等价默认 28；`[300M,320M)` WAV payload / `-audiohash` / state scalars）；可选 `-HandOff` 同二进制 A/B | only with `-Execute -UserPresent` |
| `m4_stress_voices.ps1` | ~~n=32/64/128/255 矩阵（255 = 妥协上限，目标 256）~~ **暂缓**（`-voices:` 已回滚）；原 S1-S7 解析与 O1-O10 映射（`cfg3d=0x7b` + `pcm.ext_voices=n`）随 256 扩展暂缓 | 暂缓 |
| `two_mode_check.ps1` | M1-M3 default-path regression (trace/hash, `SDL_*_DRIVER=dummy`) | headless by design, unchanged |

Hard boundaries for the M4 scripts:

- **Dry-run by default.** Without `-Execute` the scripts only validate paths,
  probe the GT build (`-h` only) and print the exact commands, timeout and
  comparison plan. Dry-run exits `0`; missing GT options are printed as
  `ERROR:` lines and `-RequireReady` turns them into exit `2`.
- **`-Execute` requires `-UserPresent`** (otherwise exit `2`) and starts GT
  with LCD window + real audio. They never set `SDL_VIDEODRIVER`/`SDL_AUDIODRIVER`.
- Timeout is `ceil(end_cycles/24e6 * 2) + 15` s (320M -> 42 s, 400M -> 49 s);
  the process is `Stop-Process -Force`d in a `finally` block.
- Automated = file/stream comparison and judgement parsing. Manual = LCD
  observation, n=28 A/B listen, ~~255-voice dropped-note/crackle listen~~
  （256 复音监听随扩展回滚暂缓）; SKIP is
  not PASS and a green summary never replaces the on-site listen.

### m4_audio_null.ps1

```powershell
# dry-run (default, no GT process)
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\m4_audio_null.ps1

# readiness gate: exit 2 until the GT options/corpus exist
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\m4_audio_null.ps1 -RequireReady

# real run (user present, LCD + audio)
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\m4_audio_null.ps1 -Execute -UserPresent

# MIDI-driven variant and W3 gate-5 A/B (needs a corpus schedule)
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\m4_audio_null.ps1 -Scenario midi `
    -MidiSchedule mk2cpp\out\m4\corpus\poly28.sched -HandOff -Execute -UserPresent
```

| Parameter | Default | Meaning |
|---|---|---|
| `-Scenario` | `demo` | `demo` or `midi` (`-midiseq <schedule> 200000000`) |
| `-AudioStart` / `-AudioEnd` | `300M` / `320M` | audio window; default per `plan_256.md` |
| `-StateAt` | `0` (= `AudioEnd`) | `-hashdump` cycle; must be `>= 200M` |
| `-TimeoutSec` | `0` (auto) | per-run kill timeout |
| `-OutDir` | `mk2cpp\out\m4\audio_null` | outputs (gitignored) |
| `-MidiSchedule` | empty | required for `-Scenario midi` |
| `-Gain` | empty | optional `-gain:<x|db>`; must match across runs |
| `-HandOff` | off | adds `-mk2cpp-hand:0` A/B run (W3 gate 5) |
| `-RequireReady` | off | dry-run exits `2` when required GT options are missing |
| `-Execute` / `-UserPresent` | off | the only way to launch GT; both are required |

Checks: `-audiohash` equality, WAV payload SHA256 (`pcmdiff --tolerance 0`
report when built), finalized WAV header, `.meta` window/rate/length equality,
layout-independent state scalars (`mcu.pc/sr/cycles`, `pcm.config_reg_3c/3d`),
`LCDEN 0 <= 2`, CPU duty report (INFO), and the hand on/off A/B.

### m4_stress_voices.ps1

```powershell
# dry-run matrix; per-level SKIP when the corpus schedule is missing
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\m4_stress_voices.ps1

# demo-scenario long run (no MIDI material needed)
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\m4_stress_voices.ps1 -Scenario demo -Execute -UserPresent

# subset (Windows PowerShell 5.1 -File binds comma lists as one token, so the
# script splits them itself; both "32,64" and "32 64" work)
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\m4_stress_voices.ps1 -VoiceLevels 32,64
```

| Parameter | Default | Meaning |
|---|---|---|
| `-VoiceLevels` | `32,64,128,255` | levels outside `28..255` are warned and skipped (`256` is a CLI cap violation, D1) |
| `-Scenario` | `midi` | `midi` (`-midiseq`) or `demo` |
| `-MidiSchedule` | empty | single schedule override; empty -> `<CorpusDir>\poly<n>.sched` |
| `-CorpusDir` | `mk2cpp\out\m4\corpus` | not committed; missing schedule = SKIP + midisched hint |
| `-RunTo` | `400M` | simulation end; timeout from this value |
| `-AudioStart` / `-AudioEnd` | `300M` / `320M` | audio window |
| `-HashAt` | `0` (= `RunTo`) | `-hashdump` cycle; must be `>= 200M` |
| `-TraceFrom` / `-TraceTo` | `0`/`0` -> `RunTo-2M`/`RunTo` | trace window used for S1/O10 |
| `-TimeoutSec` | `0` (auto) | per-level kill timeout |
| `-OutDir` | `mk2cpp\out\m4\stress` | outputs (gitignored) |
| `-RequireReady` | off | dry-run exits `2` when GT options or schedules are missing |
| `-Execute` / `-UserPresent` | off | the only way to launch GT; both are required |

Per-level checks: `S0` schedule presence; `S1` run/trace/`LCDEN`; `S2` non-zero
WAV payload; `S3` CPU duty/real-time factor (INFO); `S4` `pcm.config_reg_3d ==
0x7b` **and** `pcm.ext_voices == n` (plan A, 2026-09-11, per
`out/m4/12_cfg3d_voice_count.md`; `ext_voices` is read from `-snapinfo` only
because `-hashdump` stays v1 for M1-M3 baseline compatibility, and a
missing/unparsable `ext_voices` is a SKIP with the reason
printed, never a PASS); `S5`
`select_channel` coverage; `S6` low-32 mask popcount (ext writes need the G2
pcmtrace extension); `S7` IRQ slot evidence (G2/G3, else SKIP). The `O1-O10`
mapping is printed in `plan.txt` and parsed where evidence exists: O2/O3 need
`-snapinfo` (the `-hashdump` text stays v1 and does not grow scalars), O5 uses pcmtrace PCs
`00:5527`/`00:5664`, O6 needs ext write logging, O9 stays with
`two_mode_check.ps1`, O10 is the `00:037A` stall signature.

### MIDI corpus (not committed)

`midisched` (see `../tools/README.md`) converts SMF to the `-midiseq` schedule
format. Put the result under `mk2cpp\out\m4\corpus\` (gitignored):

```powershell
clang -O2 -o mk2cpp\tools\midisched\midisched.exe mk2cpp\tools\midisched\midisched.c
mk2cpp\tools\midisched\midisched.exe poly255.smf -o mk2cpp\out\m4\corpus\poly255.sched
```

Schedule cycles are relative to the `-midiseq` start argument (the scripts pass
`200000000`). `midisched` splits each message into per-byte lines with a
`--byte-gap` (default 7680 cycles = 320 us) so the 8192-byte GT UART ring is not
overrun; `--byte-gap 0` groups the bytes of one event on one line.

## two_mode_check.ps1 usage (M1-M3, unchanged)

```powershell
# boot [0,3M), expected PASS
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\two_mode_check.ps1 -Scenario boot

# 200M demo window [200M,202M), ~40s per run; raise the timeout
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\two_mode_check.ps1 -Scenario demo200 -TimeoutSec 90

# custom window/mode arguments
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\two_mode_check.ps1 `
    -Scenario custom -WindowFrom 0 -WindowTo 1000000 -HashCycle 1000000 -ScenarioArgs @('-demo')
```

## two_mode_check.ps1 parameters

| Parameter | Default | Meaning |
|---|---|---|
| `-Scenario` | `boot` | `boot` (`-mk2`), `demo200` (`-mk2 -demo`), `custom` (`-mk2` + `-ScenarioArgs`) |
| `-Window` | `auto` | Named window preset: `auto`, `boot3m`, `200m`, `custom` |
| `-WindowFrom` | `0` | Trace window start (overrides preset when > 0) |
| `-WindowTo` | `0` | Trace window end, exclusive (overrides preset when > 0) |
| `-HashCycle` | `0` | `-hashdump` trigger cycle (0 → `-WindowTo`; overrides preset when > 0) |
| `-TimeoutSec` | `60` | Per-run poll timeout; GT is killed afterwards either way |
| `-OutDir` | `mk2cpp\out\twomode` | Output directory (relative paths resolve against the repo root) |
| `-ScenarioArgs` | empty | Extra GT args for `-Scenario custom` only |
| `-KeepGoing` | off | Continue to the summary and exit 0 even when checks fail |

Scenario presets: `boot` = `[0,3000000)` hash@3000000; `demo200` =
`[200000000,202000000)` hash@202000000.

## two_mode_check.ps1 comparison

1. `def.trace` vs `tr.mk2cpp.trace` — `mk2cpp\tools\tracediff\tracediff.exe`
   (exit 0 required; exit 1 = first divergence, 2/3 = prefix).
2. `def.hash` vs `tr.mk2cpp.hash` — `Get-FileHash` SHA256.
3. `def.trace` vs `tools\baselines\trace_boot3m_base.txt` (boot, 0/3M only) or
    `trace_200m_base.txt` (demo200, 200M/202M only). If the fixture is absent the
    check is `SKIP`, not a failure.

Since M3 the SM (sub MCU) is part of this check automatically: the unified trace
carries the `s <sm_cycles> <pc>` SM lines (boot ~125K, demo200 ~208K), and the
state hash includes `hash.sm`, `sm_ram`, `sm_shared_ram`, `sm_device_mode` and
`hash.lcd_state` — so a PASS also proves the translated SM (`-mk2cpp`) drives the
SM state and LCD identically to the GT interpreter.

The script also checks that both runs produced a non-empty trace and hash; if a
file never appears it marks `FAIL` and prints the tail of the run's stdout and
stderr.

## Outputs (under `-OutDir`)

```
def.trace / def.hash                 default interpreter run
tr.mk2cpp.trace / tr.mk2cpp.hash     -mk2cpp run
diff_trace.md / diff_trace_div.txt   tracediff def vs -mk2cpp
diff_baseline.md / diff_baseline_div.txt
summary.txt                          PASS/FAIL table
logs\*.stdout.txt / *.stderr.txt     redirected run logs
```

## Frozen GT interface (Wave 0a/0b minimum commitments)

The M4 scripts are written against these frozen interfaces even though the
current GT build does not implement them yet. The capability probe reads `-h`
only; every implemented option must be advertised there.

1. `-wav:<file>`: producer-side tap in `MCU_PostSample` after `master_gain` and
   the int16 clamp; standard RIFF/WAVE header (PCM=1, ch=2, S16,
   rate=`spec.freq`); default off, zero behavior change without the flag.
2. `-audiowin <start> <end>`: only write samples with `mcu.cycles` in
   `[start,end)`. At `end` backfill the RIFF/data sizes, flush and close the
   WAV, then write `<file>.meta` **last** as the completion marker (`wavdump v1`
   with `rate/channels/format/start_cycles/end_cycles/first_cycle/last_cycle/
   sample_pairs/data_bytes/payload_fnv1a`). Force-kill must not be required to
   finalize the header.
3. `-audiohash <cycles> <file>`: FNV-1a64 over the int16 PostSample stream up to
   the target cycle, one `audio_fnv1a = <16hex>` line; may repeat for checkpoints.
4. `-midiseq <file> [start]`: text schedule `<cycle> <hexbyte>...` with `#`
   comments; cycles are relative to `start` (scripts pass `200000000`);
   `MCU_PostUART` with ring backpressure (8192 B), no overwrite.
5. `-mk2cpp-hand:0|1`: default `1`; `0` skips the hand table for the same-binary
   A/B gate (W3 §5.3 gate 5).
6. `-snapinfo <cycles> <file>`: `isr4fe`
   heartbeat, `sleep`/`iml`/`pend`, `pcm.select_channel`, `pcm.irq_channel`,
   `voice_mask` popcount, `pcm.ext_voices` (S4/O4, plan A per
   `out/m4/12_cfg3d_voice_count.md` §4.3). `-hashdump` stays v1 (no new
   fields) so the M1-M3 baselines remain comparable; `pcm.ext_voices` is not
   added there. Optional: missing evidence is SKIP, not PASS.
7. `-pcmtrace` extended coverage (G2): `pcm ext reg=.. val=.. pc=.. cyc=..` for
   `PCM_WriteExt` and ext reads (`0xE820`). Optional: O5 works without it, but
   O6/S6-ext/S7 are then SKIP.

The project goal is **256 sounding voices**; the accepted oracle voice count is
`-voices:255` (CLI range `28..255`) - the compromise cap forced by the `0xff`
slot sentinel and the 8-bit pool counter, not the goal itself. The oracle never
asks for `-voices:256` until that design change (D1) is approved.

## Exit codes

- `0` — all checks PASS/SKIP, or `-KeepGoing` was given.
- `1` — at least one FAIL (without `-KeepGoing`).
- `2` — setup error (missing exe/tracediff, invalid window).
- M4 scripts: `0` = automated PASS or dry-run OK; `1` = at least one FAIL;
  `2` = setup/gate error (`-RequireReady` with missing options, `-Execute`
  without `-UserPresent`, or a required GT option missing).

## Notes / caveats

- The script sets `SDL_VIDEODRIVER=dummy`, `SDL_AUDIODRIVER=dummy`,
  `SDL_RENDER_DRIVER=software` for headless determinism. The M4 scripts
  deliberately do **not** reuse that mode.
- With no generated code linked (no `-DMK2CPP_GEN_DIR`), `-mk2cpp` is pure
  interpreter fallback and the expected result is PASS on every check.
- Baseline traces are local fixtures; they live under `tools\baselines\` and
  may be absent after cleanup. A missing fixture is reported as SKIP.
- A completed trace is detected via the last `m <cycles>` line reaching the
  window end; the `-hashdump` file must additionally exist and be non-empty.
- Redirected streams are closed and the process is `Stop-Process -Force`d in a
  `finally` block, so a hung GT cannot block the check indefinitely.

## Known gaps (Wave 0c)

- No real GT run was performed in this wave: the M4 scripts are dry-run
  validated only. They need GT options G1/G2/G3/G5 (above) and the local corpus
  before `-Execute` can pass its gate.
- `-voices:256` remains a CLI cap decision (D1): the goal is 256 voices, but
  the current compromise cap is `255` (`0xff` sentinel + 8-bit counter,
  `out/m4/12_cfg3d_voice_count.md` §5.2); the oracle uses `255` until the
  true-256 design change lands. If the cap is ever raised, `-VoiceLevels` and
  the S4 expectation need revisiting.
- D6 (`config_reg_3d` bit5 vs voice count) is resolved by plan A
  (2026-09-11, `out/m4/12_cfg3d_voice_count.md`): extension levels must keep
  `cfg3d=0x7b` and report `pcm.ext_voices=n` via `-snapinfo` (`-hashdump` stays
  v1, unchanged for M1-M3 baseline compatibility). The scalar is parsed from
  `-snapinfo`; when absent, S4/O4 parse as SKIP (never PASS).
- The stock 300M-320M audio baseline freeze (G6) is still pending: it needs a
  user-present stock run with `-wav:`/`-audiowin`/`-audiohash`, otherwise every
  M4 audio null is only "same-version two-mode", not a frozen reference.
- The `mk2cpp/README.md` tools directory table was not updated (write scope of
  this wave was limited to `mk2cpp/tests/**` and `mk2cpp/tools/**`); add the
  pcmdiff/midisched rows there separately.
