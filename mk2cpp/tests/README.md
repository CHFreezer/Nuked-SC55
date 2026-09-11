# mk2cpp/tests

Oracle scripts for the mk2cpp integration. `two_mode_check.ps1` runs stock GT
twice for one scenario and window — default interpreter vs `-mk2cpp` translated
core — then compares traces, state hashes and (when applicable) the frozen
`tools/baselines` fixture.

GT never self-exits, so the script polls for the `-hashdump` file and a
completed trace window and then force-kills the process. It is pure PowerShell
5.1 (no Python) and writes everything under `-OutDir` only.

## Usage

```powershell
# boot [0,3M), expected PASS
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\two_mode_check.ps1 -Scenario boot

# 200M demo window [200M,202M), ~40s per run; raise the timeout
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\two_mode_check.ps1 -Scenario demo200 -TimeoutSec 90

# custom window/mode arguments
powershell -ExecutionPolicy Bypass -File mk2cpp\tests\two_mode_check.ps1 `
    -Scenario custom -WindowFrom 0 -WindowTo 1000000 -HashCycle 1000000 -ScenarioArgs @('-demo')
```

## Parameters

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

## What is compared

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

## Exit codes

- `0` — all checks PASS/SKIP, or `-KeepGoing` was given.
- `1` — at least one FAIL (without `-KeepGoing`).
- `2` — setup error (missing exe/tracediff, invalid window).

## Notes / caveats

- The script sets `SDL_VIDEODRIVER=dummy`, `SDL_AUDIODRIVER=dummy`,
  `SDL_RENDER_DRIVER=software` for headless determinism.
- With no generated code linked (no `-DMK2CPP_GEN_DIR`), `-mk2cpp` is pure
  interpreter fallback and the expected result is PASS on every check.
- Baseline traces are local fixtures; they live under `tools\baselines\` and
  may be absent after cleanup. A missing fixture is reported as SKIP.
- A completed trace is detected via the last `m <cycles>` line reaching the
  window end; the `-hashdump` file must additionally exist and be non-empty.
- Redirected streams are closed and the process is `Stop-Process -Force`d in a
  `finally` block, so a hung GT cannot block the check indefinitely.
