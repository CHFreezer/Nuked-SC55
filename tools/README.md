# tools/ — H8/532 VM and Nuked-SC55 verification toolset

Standalone reimplementation of the Nuked-SC55 main CPU (H8/532) plus the
disassembler and log-diff utilities used to verify it against the reference
emulator (`../src/`, `../build/nuked-sc55.exe`).

Verified state (SC-55mk2, `../roms/SC-55mk2-v1.01/`):
- Read-log path identity with the reference emulator over ~3M reads (~35M cycles),
  zero divergences (addr + value + pc).
- Unique main-PC coverage: 7021 at a 100M-instruction `demo_postW.bin` run
  (saved as `baselines/pc_main.txt`); SM sub-CPU stays in the DSP core loop.

## Layout

```
vm/        H8/532 VM (main CPU + SM sub-CPU + PCM audio engine)
disasm/    h8dasm — ground-truth H8/55 disassembler
diff/      trace/log comparison utilities (all argv-driven)
misc/      one-off analysis scripts from the investigation (hardcoded paths)
baselines/ PC sets + disassembly from our VM (`demo_postW.bin` 100M run)
docs/      research notes (SC-55 pipeline, filter ports, boundaries)
python/    embedded CPython (used by ../build/dasm.py)
build.ps1  builds everything (clang)
```

## Build

```
powershell -ExecutionPolicy Bypass -File build.ps1
```

Requires clang (defaults to `C:\Program Files\LLVM\bin\clang.exe`, falls back to
`clang` on PATH). Executables are produced next to their sources.

## VM — `vm/h8vm.exe`

```
h8vm.exe rom1.bin rom2.bin rom_sm.bin [waverom1.bin waverom2.bin] [limit] [noregs] [verbose]
        [savesnap file cycle] [loadsnap file] [tracepc file] [demo]
```

- `limit` — instruction count (default 500000). Each instruction = 12 cycles.
- `noregs` — compact trace: the `m` line is `m <mcu_cycles> <cp:pc>` with no
  register dump (default keeps the full `sr r0-r7 br dp ep tp` dump).
- `verbose` — extra side-channel debug lines in `vm.log` (off by default, and
  independent of `noregs`): memory reads (`rc ...`), SRAM writes (`sramw ...`),
  FRT/timer/interrupt state on every change (`c ...`), plus one-shot dumps (burst
  sram window, divergence register probe, read-window instruction step). `noregs`
  trims the `m` line; `verbose` adds whole lines. See the side-channel table below.
- `demo` — apply the built-in Q/R/T/W key sequence that starts the built-in demo
  from boot. **Off by default** so that a `loadsnap` of a mid-demo snapshot
  (e.g. `demo_postW.bin`) starts in the exact saved state without re-driving the
  boot keys. When loading a snapshot, "demo" = start from that snapshot, not the
  key sequence.
- Wave ROMs are unscrambled after load (same bit-permutation as the reference).

### Snapshot save/load

Full machine state (main H8/532 CPU + sram + device regs + FRT/timer +
`timer_cycles`/`timer_tempreg` + analog, SM sub-CPU incl. its RAM/shared RAM/
UART buffer/GA lines, and the PCM audio device) serialized to a binary file.
Used to resume the VM from an exact cycle and reproduce the reference trace:

- `savesnap file cycle` — write the snapshot once, when `mcu.cycles >= cycle`.
- `loadsnap file` — read the snapshot at the very start (before any instruction),
  the zero-key equivalent of the reference emulator's `-loadsnap`/`-demo2`.

Verified round-trip: save at a mid-run cycle, reload at start, and the trace
from the save point is byte-identical (0 diffs over 200k instructions).

### PC trace (reference-compatible)

`tracepc file [start end]` — unified main+SM PC trace for the half-open
`[start, end)` window (default `200000000 210000000`). `end` is the write
boundary: once `mcu.cycles` reaches `end`, writing stops even if the run
continues (VM `limit` / GT real-time), so the file is bounded by the window,
not by how long the process runs. One file, one line per event, with a type
prefix, then the cycle position, then the PC:

  `m <mcu_cycles> <cp:pc>` — main-CPU instruction (logged after it executes)
  `s <sm_cycles>  <pc>`    — SM instruction (logged inside `SM_Update`)

The SM is clocked at 5x the main rate (48 sm-cycles/instruction) and is
asynchronous, so `s` lines are interleaved with `m` lines at their actual
sm-cycle position. This captures every SM PC (per-main-cycle sampling of `sm.pc`
misses the intermediate instruction when the SM runs two in one main cycle).
Because the SM keeps running during the main instruction that crosses `end`,
the file can end with a couple of trailing `s` lines after the last `m` line;
this is expected and identical on both sides.

`demo_postW.bin` is captured at `mcu.cycles ≈ 200000000`, so to trace a window
further along you must run that far: the run starts at ~200M, so VM `limit` must
be at least `(end − 200000000) / 12` instructions (each = 12 cycles) to reach
the `end` boundary and close the file; GT simply runs real-time until it passes
`end`. GT and VM produce byte-for-byte identical files (0 diffs — default window
over 833332 lines; `[300000000, 310000000)` over 37,291,706 bytes, `fc /B`):

```
nuked-sc55.exe -mk2 -loadsnap demo_postW.bin -tracepc gt.txt
h8vm.exe rom1.bin rom2.bin rom_sm.bin waverom1.bin waverom2.bin 833333 noregs loadsnap demo_postW.bin tracepc vm.txt

nuked-sc55.exe -mk2 -loadsnap demo_postW.bin -tracepc gt300.txt 300000000 310000000
h8vm.exe rom1.bin rom2.bin rom_sm.bin waverom1.bin waverom2.bin 12000000 noregs loadsnap demo_postW.bin tracepc vm300.txt 300000000 310000000
```

`vm.log` (when `tracepc` is off) is one line per event: `m <mcu_cycles> <cp:pc>`
(main), `s <sm_cycles> <pc>` (SM), plus the `verbose` side-channel lines below.

`verbose` side-channel lines (all go to `vm.log`, off by default; **not**
separate files):

| line type              | content                                              |
|------------------------|------------------------------------------------------|
| `rc <n> c<cycles> ...` | first 3,000,000 MCU reads: `addr=XXXXXXXX val=XX pc=XX:YYYY` |
| `c <cycles> f0=... ...`| FRT/timer/SR/interrupt-pending state on every change |
| `sramw off=... ...`    | SRAM writes (0x5c00–0x5d30 range)                    |
| `burst<n>` / `===`     | burst sram window + recent PCs at each burst entry (pc=0x7c3f) |
| `=== divergence ...`   | register snapshot at the 0xfd48 divergence probe     |
| `instr step<n> ...`    | read-window instruction-step probe (read 144000–145500) |

## Reference emulator (for producing the "orig" side)

Build `../src/` into `../build/` (ninja). Run `../build/nuked-sc55.exe` from
`../build/` so it finds the ROMs. Its debug hooks write `obs.log` (pc-change
trace), `read_orig.log`, `ram_orig.log`, `frtstate_orig.log`, `stack_orig.log`,
`sramw_orig.log` into the current directory.

## Verification workflow

1. Run both emulators for a long run and collect their logs.
2. Path identity (cheapest and most sensitive):
   ```
   diff\pcdiff.exe   read_vm.log <orig_read_log>    # addr+pc columns only
   diff\readdiff.exe read_vm.log <orig_read_log>    # addr+value+pc full lines
   ```
3. PC coverage:
    ```
    disasm\logproc.exe build\vm.log .   # VM: PC set + flow + hot stats
    diff\extract_pc.exe <trace> out_main.txt out_sm.txt   # orig obs.log
    ```
    Compare `out_main.txt` against `baselines\pc_main.txt` (7021 PCs, VM baseline).
4. Disassembly:
   ```
   diff\extract_flow.exe obs.log flow.txt
   disasm\h8dasm.exe rom1.bin rom2.bin <out_main.txt> flow.txt dasm.txt
   ```

## diff/ utilities

| tool            | usage | purpose |
|-----------------|-------|---------|
| `pcdiff`        | `pcdiff <vm_log> <orig_log>` | first addr+pc path divergence in read logs |
| `readdiff`      | `readdiff <vm_log> <orig_log>` | full-line diff of read logs |
| `ramdiff`       | `ramdiff <vm_log> <orig_log>` | per-burst sram snapshot diff |
| `extract_pc`    | `extract_pc <trace> out_main out_sm` | unique executed PCs (main + SM) |
| `extract_flow`  | `extract_flow <obs.log> out.txt` | deduped main-PC transition edges |
| `seq`           | `seq <vm\|orig> <infile> <outfile>` | ordered deduped main-PC sequence |
| `seqdiff`       | `seqdiff <orig_seq> <vm_seq>` | first sequence divergences with resync search |
| `segdump`       | `segdump <trace> <out>` | collapse trace into per-PC segments |
| `segdiff`       | `segdiff <orig_seg> <vm_seg>` | first structural segment divergence |
| `collapse`      | `collapse <in> <out>` | collapse consecutive duplicate lines |

## baselines/

PC-coverage baselines from our H8/532 VM (`demo_postW.bin` load, 100M-instruction
run; see `baselines/dasm_annotated.txt`):

- `pc_main.txt` — 7021 unique main-MCU PCs (`XXXXXXXX`, one per line, sorted).
- `flow_main.txt` — 19848 main-MCU PC transition edges (`src dst`, dedup, sorted).
- `dasm_full.txt` — `h8dasm.exe` output over the VM PC set (linear-drifted).
- `dasm_annotated.txt` — hand-annotated analysis of the hot functions.

Regenerate: `disasm\logproc.exe build\vm.log .` then
`disasm\h8dasm.exe rom1.bin rom2.bin pc_main.txt flow_main.txt dasm_full.txt`.

## misc/

One-off scripts written against specific artifacts of the investigation
(hardcoded input paths such as `..\build\ram_orig.log`). Kept for reference;
not part of the regular workflow.

## docs/

- `scgs_pipeline.md`, `scgs_operational_boundary.md`, `scgs_filter.md` —
  reverse-engineering notes on the SC-55/SCGS DSP pipeline.
- `filter_float_port.md` — notes on the float resonant filter variant.
