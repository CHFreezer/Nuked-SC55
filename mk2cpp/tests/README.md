# mk2cpp/tests

Oracle scripts for the mk2cpp integration. **Canonical, cross-platform
implementations are the Python 3 scripts** (standard library only, no bundled
interpreter):

- `m4_quick_gate.py` — default regression: 037 in both modes, full-WAV SHA256.
- `two_mode_check.py` — scenario regression: stock vs `-mk2cpp` traces/hashes
  plus the frozen `tools/baselines` fixtures.

Run them with your own Python 3 interpreter: `python3 mk2cpp/tests/m4_quick_gate.py`
(never commit an embedded interpreter). GT never self-exits, so the scripts poll
for the completion marker and force-kill the process; outputs go under
`mk2cpp/out/`.

本机相关设置（解释器、本地素材路径等）因人而异，不入库；按需自建仓库根的
`local.md` 并以其为准（README 不写具体值）。

No `.ps1` gate scripts remain: `two_mode_check.ps1`, `gt_run.ps1` and
`m4_audio_null.py` were removed after their logic was ported to Python;
`m4_stress_voices.ps1` was deleted with the M5 rollback (rewrite in Python when
M5 restarts). The Python scripts are the only gate.

## 默认回归口径（2026-09-12 起，用户要求）

- **指定回归曲 = 037**（外部 MIDI 测试冻结在这一首；曲子足够复杂，覆盖绝大
  部分行为）。**不再默认或半默认测其他曲子**。默认回归 =
  `python3 mk2cpp/tests/m4_quick_gate.py`（037 两模式并行 + WAV 逐字节，约 65 s）；
  改动主链路/非 voice 路径时再跑
  `python3 mk2cpp/tests/two_mode_check.py --scenario boot|demo200`（各 1–2 分钟）。
- **demo 预算 = 60 秒模拟时间**：`m4_quick_gate.py --demo`（窗口
  `[144M, 1440M)` cycles，144M 起跳过开机动画，墙钟约 61 s）。内置 demo 有 5 首曲，
  **永远不默认跑完整播放列表**；`two_mode_check.py --scenario demo200` 的
  `[200M,202M)` 只是 2M-cycle 冻结 fixture（秒级），两者用途不同、都不长跑。
- **不默认跑 82 曲语料**（`out/m4/corpus/midi*.sched`）与压力矩阵
  （cov_mix/stress_*）：它们是代理侧扩展覆盖，不是用户要求；仅在用户明确
  要求"全量"时运行（8 并发约 30–40 分钟），且必须分批并汇报总耗时。
- **单次前台测试预算 ≤3 分钟**；超过必须后台/分片，并先告知用户。长跑会
  阻塞会话，用户以此计算机会成本——不要用"更保险"扩大默认测试范围。

### 确定性与 `-nomidi`（必须）

- 所有 oracle GT 运行都要加 `-nomidi`：Windows 构建默认打开主机第一个 MIDI
  输入口，外部字节由回调线程经 `MCU_PostUART` 在任意 cycle 注入，造成运行间
  hash 漂移（2026-09-12 实测：gate 两实例相对历史同点漂移，端口安静后复跑恢复；
  `src/mcu.cpp` 已加 `-nomidi` 跳过 `MIDI_Init`）。
- `two_mode_check.py`、`m4_quick_gate.py` 已默认带 `-nomidi`；手工 oracle
  运行也必须带。

## M4 oracle scripts (Wave 0c)

> **回滚说明（2026-09-11）**：256 复音扩展（`-voices:<n>` 等）已从 GT 回滚到原版
> 28 复音实现。`m4_stress_voices.ps1`（n=32/64/128/255）所依赖的 `-voices:` 现已无效，
> 该脚本**暂缓**（当前运行会静默按 28 复音跑，结果不代表压力测试）。
> `m4_audio_null.py` 的 n=28 音频 null 仍有效（`-voices:28` 现为 no-op，等价默认 28）。

| Script | Purpose | Real GT run? |
|---|---|---|
| `m4_audio_null.py` | n=28 audio null: stock vs `-mk2cpp`（`-voices:28` 已回滚为 no-op，等价默认 28；`[300M,320M)` WAV payload / `-audiohash` / state scalars）；可选 `--hand-off` 同二进制 A/B、`--check-baseline` 冻结基准门禁（G6） | only with `--execute --user-present` |
| （256 压力矩阵）| n=32/64/128/255（255 = 妥协上限，目标 256）**随 M5 暂缓**；原 `m4_stress_voices.ps1` 已删除，M5 重启时以 Python 重写 | 暂缓 |
| `two_mode_check.py` | M1-M3 default-path regression (trace/hash, `SDL_*_DRIVER=dummy`, `-nomidi`); cross-platform | headless by design, unchanged |
| `m4_quick_gate.py` | **默认回归**：037 两模式并行 WAV 逐字节（`-nomidi`，硬超时，约 65 s）；`--demo` 跑内置 demo 60 s 预算（`[144M,1440M)`，约 61 s）；`--sched` 可指定其他 schedule（仅调试用，默认冻结 037） | headless, no user needed |

Hard boundaries for the M4 scripts:

- **Dry-run by default.** Without `--execute` the scripts only validate paths,
  probe the GT build (`-h` only) and print the exact commands, timeout and
  comparison plan. Dry-run exits `0`; missing GT options are printed as
  `ERROR:` lines and `--require-ready` turns them into exit `2`.
- **`--execute` requires `--user-present`** (otherwise exit `2`) and starts GT
  with LCD window + real audio. They never set `SDL_VIDEODRIVER`/`SDL_AUDIODRIVER`.
- Timeout is `ceil(end_cycles/24e6 * 2) + 15` s (320M -> 42 s, 400M -> 49 s);
  the process is force-killed after the completion markers appear.
- Automated = file/stream comparison and judgement parsing. Manual = LCD
  observation and the n=28 A/B listen; SKIP is not PASS and a green summary
  never replaces the on-site listen.

### m4_audio_null.py

```bash
# dry-run (default, no GT process)
python3 mk2cpp/tests/m4_audio_null.py

# readiness gate: exit 2 until the GT options exist
python3 mk2cpp/tests/m4_audio_null.py --require-ready

# real run (user present, LCD + audio; host MIDI disabled via -nomidi)
python3 mk2cpp/tests/m4_audio_null.py --execute --user-present

# frozen stock 300M-320M baseline gate (G6; local fixture, missing baseline = SKIP)
python3 mk2cpp/tests/m4_audio_null.py --check-baseline --execute --user-present

# MIDI-driven variant and W3 gate-5 A/B (schedule path from local.md)
python3 mk2cpp/tests/m4_audio_null.py --scenario midi \
    --midi-schedule mk2cpp/out/m4/corpus/midi037.sched --hand-off \
    --execute --user-present
```

| Parameter | Default | Meaning |
|---|---|---|
| `--scenario` | `demo` | `demo` or `midi` (`-midiseq <schedule> 200000000`) |
| `--audio-start` / `--audio-end` | `300M` / `320M` | audio window |
| `--state-at` | `0` (= end) | `-hashdump` cycle; must be `>= 200M` |
| `--timeout` | `0` (auto) | per-run kill timeout |
| `--outdir` | `mk2cpp/out/m4/audio_null` | outputs (gitignored) |
| `--midi-schedule` | empty | required for `--scenario midi` |
| `--gain` | empty | optional `-gain:<x|db>`; must match across runs |
| `--check-baseline` | off | G6 gate: verify `tools/baselines/m4_audio/SHA256SUMS.txt` (mismatch/missing listed file = FAIL) and require the stock WAV payload/`-audiohash` to match the frozen capture; missing baseline dir/manifest = SKIP |
| `--baseline-dir` | `tools/baselines/m4_audio` | frozen baseline directory |
| `--baseline-name` | `stock_300M_320M` | frozen file stem inside `--baseline-dir` |
| `--hand-off` | off | adds `-mk2cpp-hand:0` A/B run (W3 gate 5) |
| `--require-ready` | off | dry-run exits `2` when required GT options are missing |
| `--execute` / `--user-present` | off | the only way to launch GT; both are required |
| `--exe` | `build/nuked-sc55[.exe]` | GT executable override |

Checks: `-audiohash` equality, WAV payload SHA256 (`pcmdiff --tolerance 0`
report when built), finalized WAV header, `.meta` window/rate/length equality,
layout-independent state scalars (`mcu.pc/sr/cycles`, `pcm.config_reg_3c/3d`),
`LCDEN 0 <= 2`, CPU duty report (INFO), and the hand on/off A/B. With
`--check-baseline` it adds the G6 checks: `baseline:integrity` (SHA256SUMS.txt
re-verification; mismatch/missing listed file = FAIL), `baseline:wav` (stock
payload == frozen payload) and `baseline:audiohash` (stock `audio_fnv1a` ==
frozen). A missing baseline directory, manifest or file is reported as `SKIP`
with the reason; the dry-run prints the baseline status but keeps exit `0`.

### 256-voice stress matrix（M5 暂缓）

n=32/64/128/255 压力矩阵随 256 复音扩展（M5）暂缓；原 `m4_stress_voices.ps1`
已删除，M5 重启时以 Python 重写（`-voices:` CLI 已回滚，当前不可用）。O1–O10
映射与 S1–S7 设计保留在 `docs/10_m4_oracle.md`。

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

## two_mode_check.py usage (cross-platform)

```bash
# boot [0,3M), expected PASS
python3 mk2cpp/tests/two_mode_check.py

# demo window [200M,202M)
python3 mk2cpp/tests/two_mode_check.py --scenario demo200

# custom window / extra GT args
python3 mk2cpp/tests/two_mode_check.py --scenario custom \
    --args -demo --window-from 0 --window-to 1000000 --hash-at 1000000
```

| Parameter | Default | Meaning |
|---|---|---|
| `--scenario` | `boot` | `boot` (`-mk2`), `demo200` (`-mk2 -demo`), `custom` (`-mk2` + `--args`) |
| `--args` | empty | Extra GT args for `--scenario custom` (nargs after `--args`) |
| `--window-from` / `--window-to` | `0` | custom trace window, `--window-to > --window-from` required |
| `--hash-at` | window end | `-hashdump` trigger cycle |
| `--timeout` | `120` | per-run hard timeout (s); GT is force-killed afterwards either way |
| `--exe` | `build/nuked-sc55[.exe]` | GT executable override |
| `--outdir` | `mk2cpp/out/twomode` | output directory |

Scenario presets: `boot` = `[0,3000000)` hash@3000000; `demo200` =
`[200000000,202000000)` hash@202000000.

Comparison: `def.trace` SHA256 == `tr.mk2cpp.trace` SHA256; `def.hash` ==
`tr.mk2cpp.hash`; `def.trace` == `tools/baselines` fixture for the frozen
scenarios (SKIP when the local fixture is absent). On mismatch the first
differing line is printed.

Since M3 the SM (sub MCU) is part of this check automatically: the unified trace
carries the `s <sm_cycles> <pc>` SM lines (boot ~125K, demo200 ~208K), and the
state hash includes `hash.sm`, `sm_ram`, `sm_shared_ram`, `sm_device_mode` and
`hash.lcd_state` — so a PASS also proves the translated SM (`-mk2cpp`) drives the
SM state and LCD identically to the GT interpreter.

## Outputs (under `--outdir`)

```
def.trace / def.hash                 default interpreter run
tr.mk2cpp.trace / tr.mk2cpp.hash     -mk2cpp run
def.stdout.txt / def.stderr.txt      redirected run logs (same for tr.mk2cpp)
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
- The stock 300M-320M audio baseline freeze (G6) is **done**: the frozen dump
  lives in `tools\baselines\m4_audio\` (local fixture, gitignored, SHA256SUMS.txt
  verified; see `tools\baselines\README.md`). `m4_audio_null.py -CheckBaseline`
  turns the n=28 null into a frozen-reference comparison. Refreeze (user
  present) when the stock PCM/ROM path or the audio tap changes.
- The `mk2cpp/README.md` tools directory table was not updated (write scope of
  this wave was limited to `mk2cpp/tests/**` and `mk2cpp/tools/**`); add the
  pcmdiff/midisched rows there separately.
