# tools/ — H8/532 VM and Nuked-SC55 verification toolset

Standalone reimplementation of the Nuked-SC55 main CPU (H8/532) plus the
disassembler and log-diff utilities used to verify it against the reference
emulator (`../src/`, `../build/nuked-sc55.exe`).

Verified state (SC-55mk2, `../roms/SC-55mk2-v1.01/`):
- Read-log path identity with the reference emulator over ~3M reads (~35M cycles),
  zero divergences (addr + value + pc).
- Unique main-PC coverage: 6796 at a 40M-instruction run, superset of the
  6682-PC reference baseline (`baselines/pc_main.txt`), 188/188 SM-PC baseline.

## Layout

```
vm/        H8/532 VM (main CPU + SM sub-CPU + PCM audio engine)
disasm/    h8dasm2 — ground-truth H8/55 disassembler
diff/      trace/log comparison utilities (all argv-driven)
misc/      one-off analysis scripts from the investigation (hardcoded paths)
baselines/ PC sets extracted from a long reference-emulator run
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
h8vm.exe rom1.bin rom2.bin rom_sm.bin [waverom1.bin waverom2.bin] [limit] [noregs]
```

- `limit` — instruction count (default 500000). Each instruction = 12 cycles.
- `noregs` — compact trace (pc only) instead of full register dump.
- Wave ROMs are unscrambled after load (same bit-permutation as the reference).
- A MIDI note-on (ch0, note 60, vel 100) is pre-loaded into the shared UART,
  matching the reference emulator's warm-up.

stdout: one line per executed instruction (`main pc=.. | sm pc=..`).

Side-channel logs (written to the current working directory):

| file               | content                                              |
|--------------------|------------------------------------------------------|
| `read_vm.log`      | first 3,000,000 MCU reads: `rcN cCYCLES addr=XXXXXXXX val=XX pc=XX:YYYY` |
| `frtstate_vm.log`  | FRT/timer/SR/interrupt-pending state on every change |
| `sramw_vm.log`     | SRAM writes                                          |
| `stack_vm.log`     | 32-byte stack window at each burst entry (pc=0x7c3f) |
| `ram_vm.log`       | full 32KB sram snapshot for the first 32 bursts      |
| `divg_vm.log`      | register snapshot for a specific divergence probe    |
| `instr_vm.log`     | read-window probe (read count 144000–145500)         |

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
   diff\extract_pc.exe <trace> out_main.txt out_sm.txt   # works on orig obs.log and VM traces
   ```
   Compare `out_main.txt` against `baselines\pc_main.txt` (6682 PCs) and
   `out_sm.txt` against `baselines\pc_sm.txt` (188 PCs).
4. Disassembly:
   ```
   diff\extract_flow.exe obs.log flow.txt
   disasm\h8dasm2.exe rom1.bin rom2.bin <out_main.txt> flow.txt dasm.txt
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

- `pc_main.txt` — 6682 unique main-MCU PCs from a long reference run
  (format: `XXXXXXXX`, one per line, sorted).
- `pc_sm.txt` — 188 unique SM sub-CPU PCs (`XXXX`, sorted).

## misc/

One-off scripts written against specific artifacts of the investigation
(hardcoded input paths such as `..\build\ram_orig.log`). Kept for reference;
not part of the regular workflow.

## docs/

- `scgs_pipeline.md`, `scgs_operational_boundary.md`, `scgs_filter.md` —
  reverse-engineering notes on the SC-55/SCGS DSP pipeline.
- `filter_float_port.md` — notes on the float resonant filter variant.
