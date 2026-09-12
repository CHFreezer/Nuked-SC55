# tools/ — H8/532 VM and Nuked-SC55 verification toolset

Standalone reimplementation of the Nuked-SC55 main CPU (H8/532) / SM sub-CPU /
PCM engine, plus disassemblers, verification tools and research docs used to
check everything against the reference emulator (`../src/`, `../build/nuked-sc55.exe`).

All tools are **C** and built with clang (no Python toolchain; `python/` is a
bundled CPython kept *only* to satisfy LLDB's `python311.dll` dependency when
debugging — project tooling must stay in C).

## Layout

```
vm/        H8/532 VM (main CPU + SM sub-CPU + PCM audio engine)
disasm/    h8dasm (H8/55 disassembler), smdasm (SM disassembler), logproc
verify/    verify_dasm — disassembly + semantic VM verification (Phase A/B)
probe/     memory/page probes used during the polyphony research (C sources)
baselines/ PC sets, flow edges, disassembly, reference traces and snapshots
docs/      research + evidence docs (see docs/evidence_protocol.md)
python/    embedded CPython for LLDB only (do not use for project tooling)
build.ps1  builds every tool above (clang)
```

## Build

```
powershell -ExecutionPolicy Bypass -File build.ps1
```

Requires clang (defaults to `C:\Program Files\LLVM\bin\clang.exe`, falls back
to `clang` on PATH). Executables are produced next to their sources.

## Git policy (hard rule)

Only **docs and code** are committed. ROM data and anything derived from it
(disassembly, PC/flow sets, traces, snapshots, logs) must never enter git — see
`docs/evidence_protocol.md` §0.0. `tools/` and `mk2cpp/` are already public
trees, so ignore rules for artifacts inside them live in the committed
`.gitignore` (`tools/baselines/*`, `*.exe`, `mk2cpp/out/`,
`mk2cpp/src/gen/`, ...) and are shared by every contributor. Never write local
private path names or local directory layouts into any committed file (docs,
comments, commit messages); local-only material is excluded locally and stays
out of the repo. This rule cannot be enforced by the repository itself, so it
is on every contributor.

## Directory hygiene

- `../build/` holds only build system + runtime assets (exe/pdb, rom1/rom2/
  rom_sm, waverom1/2, `back.data`, SDL2.dll). GT uses the exe directory as the
  ROM BasePath (prints `Base path is: ...`), so executables must stay there.
- All debug outputs (traces, logs, `.out`, snapshots) go to `%TEMP%\opencode\`
  (or a scratch dir), never into `build/` or `tools/`.
- Baseline fixtures live in `baselines/`; see `baselines/README.md`.

## VM — `vm/h8vm.exe`

```
h8vm.exe rom1.bin rom2.bin rom_sm.bin [waverom1.bin waverom2.bin] [limit] [noregs] [verbose]
         [savesnap file cycle] [loadsnap file] [tracepc file [start end]] [demo]
```

- `limit` — instruction count (default 500000). Each instruction = 12 cycles.
- `noregs` — compact `m` line (`m <mcu_cycles> <cp:pc>`); default keeps the full
  `sr r0-r7 br dp ep tp` register dump. `verbose` adds side-channel lines to
  `vm.log` (memory reads, SRAM writes, timer/interrupt changes, probe dumps).
- `demo` — apply the built-in Q/R/T/W key sequence from boot. Off by default so
  a `loadsnap` of a mid-demo snapshot starts exactly in the saved state.
- `savesnap file cycle` / `loadsnap file` — full machine state
  (main CPU + sram + device regs + FRT/timer + SM sub-CPU + PCM device).
  Verified round-trip: trace after reload is byte-identical over 200k instrs.
- `tracepc file [start end]` — unified main+SM PC trace in the half-open window
  `[start, end)` (defaults `200000000 210000000`): `m <cycles> <cp:pc>` (logged
  after execution) interleaved with `s <sm_cycles> <pc>` at their actual
  positions. GT and VM produce byte-identical files on the same window.
- Wave ROMs are unscrambled after load (same permutation as the reference).

The VM also backs extension pages: page 6 and page 7 (64KB each) are real RAM
(`b_ram`/`b_ram2`), matching GT's extension mapping (`src/mcu.cpp`).

## verify — `verify/verify_dasm.exe`

Phase A: re-decodes the disassembly baselines and checks instruction boundaries,
operand semantics and annotations (vector-entry supplemental decode included).
Phase B: runs the built-in H8 VM over boot + snapshot traces and compares the
executed instruction stream against the recorded baseline (0 divergences).

## disasm

- `h8dasm.exe rom1 rom2 pcs flows out` — H8/55 disassembler. Note: the `->`
  annotation at end of line is the **origin** recorded in the trace, not the
  caller; use `baselines/flow_main.txt` for real predecessor edges.
- `smdasm.exe` — SM sub-CPU disassembler (see `baselines/sm_full.txt`).
- `logproc.exe <vm.log> <outdir>` — extracts unique PC sets + flow edges + hot
  stats from a VM log.

## probe

Research probes for the polyphony extension: `memprobe_gen` (page6/page7
backing generation), `memprobe_map` (SRAM usage map), `memprobe_read`
(read-path logging), `smvec_check` (SM vector check). Historical; kept for
reproducibility.

## baselines/

See `baselines/README.md` for the fixture inventory and regeneration commands.
Reference traces moved here from `build/`:
`trace_200m_base.txt`, `trace_boot3m_base.txt`; probe snapshots:
`probe_snap_page0.bin`, `probe_snap_page6.bin`.
