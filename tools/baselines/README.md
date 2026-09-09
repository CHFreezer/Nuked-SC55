# tools/baselines

PC-coverage baselines from our H8/532 VM (`../vm/h8vm.exe`). The main-CPU PC set
is the **union of three trace runs** (source counts + commands in the "Provenance"
section below; the SM set is the 251-PC from-reset demo+mocknote run, see
`sm_full.txt`):

- `pc_main.txt`        — 9217 unique main-MCU PCs, `XXXXXXXX` per line, sorted
  (union of the 7021 demo-snapshot baseline + run1 demo 200M–300M + run2
  from-reset mocknote 0–260M; the 2196 non-baseline PCs are boot/init, note-on
  and extra rom2 DSP paths the snapshot run skipped)
- `flow_main.txt`      — 22969 main-MCU PC transition edges, `XXXXXXXX XXXXXXXX`
  (src dst), dedup + sorted by src (union of the same three runs)
- `pc_sm.txt`          — 45 unique SM (m740) PCs, `XXXX` per line, sorted.
  This is the *complete* SM set: the `s <cycle> <pc>` lines in `vm.log` capture
  every SM instruction, so logproc recovers all of them (per-main-cycle sampling
  of `sm.pc` only saw 40).
- `sm_disasm.txt`      — `smdasm.exe` output over the SM PC set, with [xref]
- `sm_full.txt`        — `smdasm.exe … linear`: full-ROM (4096-byte) multi-entry
  linear disassembly. 10 sections, one per vector (RESET + 9 interrupt handlers),
  each section aligned at its entry and linear up to the next entry; `*` marks the
  **251 VM-verified PCs** (cross-checked by running the VM *from reset*, no snapshot,
  20M instr — a superset of the demo's 45; the demo snapshot at ~200M skipped the init).
  The VM posts a MIDI note-on (0x90,60,100) into the shared UART at 144M cycles
  (6s), aligned with the `demo` start (`uart_post` in `h8vm_main.c`), so the
  UART RX path is exercised too.
  Verified: init f003–f064, main loop, TIMER_X, IPCM0, COLLISION, **UART2_RX
  (f8d2, 21 PCs)**. **Not verified**: UART TX (f856/f871/f88c) and UART1_RX/UART3_RX
  (f88d/f8ff) — those fire only on the other host/MIDI UART ports. Drifts in data
  tables / gaps between anchors. Layout: RESET f003 (init + main DSP loop f065–f2ab),
  TIMER_X f451, IPCM0 f4b7, COLLISION f4ed, UART1/2/3_TX f856/f871/f88c,
  UART1/2/3_RX f88d/f8d2/f8ff (+ vector table f0fec–f0fff at ROM tail).
  Each section header is tagged `[VM-verified]` or `[linear/static, unverified]`
  (auto-computed by `smdasm` from whether the section contains any executed PC):
  5 verified (RESET, TIMER_X, IPCM0, COLLISION, UART2_RX) + 5 linear-only
  (UART1/2/3_TX, UART1_RX, UART3_RX).
  Regenerate the 251-PC marking: run the VM from reset (`h8vm … 20000000 noregs
  demo mocknote`, no `loadsnap`) → `logproc` → pass its `pc_sm.txt` to `smdasm …
  linear`. Both `demo` and `mocknote` are required: `mocknote` alone reaches only
  244 SM PCs; the demo key sequence adds the remaining 7.
- `dasm_full.txt`      — `h8dasm.exe` output over the 9217-PC set (9217 instr
   lines; 0 undecoded operands; cross-validated by `tools/verify/verify_dasm.py`
   — Phase A length/target 0-fail on 833332 instr, Phase B register/SP/sr
   0-fail on 44213 instr). Decodes the counter-branch opcodes (`01`/`06`/`07`,
   3-byte `cntjmp rN <disp> [Z|!Z]`) and the `11` register-indirect family
   (`ret`/`ret via rN:rN+1`/`jmp rN`/`jsr rN`) per GT `MCU_Jump_JMP`.
- `dasm_annotated.txt` — hand-annotated analysis of the hot functions (main
  sleep loop, rom2 per-frame loop, SM 6502 DSP loop, vector tables)

Provenance (main 9217 PC set — evidence):
```
source    command (from reset unless noted)                          PCs
baseline  h8vm … loadsnap demo_postW.bin (100M demo, 200M window)     7021
run1      h8vm … loadsnap demo_postW.bin tracepc t1 200000000 300000000  5833
run2      h8vm … mocknote tracepc t2 0 260000000                        6705
-------------------------------------------------------------------------
union     sort -u (baseline+run1+run2 pc_main / flow_main)            9217
```
- All 7021 baseline PCs are present in the union (verified: 0 missing).
- The 2196 non-baseline PCs are boot/init (rom1 0x016f–0x02ab), the note-on
  path, and extra rom2 DSP regions (0x40462+, 0x41220+, 0x42b8a+, …) — code the
  demo-snapshot baseline (which loads at ~200M, skipping boot) never reached.
- SM union is 244 (run2 mocknote-only) ⊂ the 251 from-reset demo+mocknote set,
  so the SM baseline stays 251 (no SM gain from the two main runs).

Regenerate (main union):
```
# each run -> logproc (pc_main.txt + flow_main.txt); then union:
sort -u baseline/pc_main.txt run1/pc_main.txt run2/pc_main.txt > pc_main.txt     # 9217
sort -u baseline/flow_main.txt run1/flow_main.txt run2/flow_main.txt > flow_main.txt  # 22969
../disasm/h8dasm.exe rom1.bin rom2.bin pc_main.txt flow_main.txt dasm_full.txt
../disasm/smdasm.exe rom_sm.bin pc_sm.txt sm_disasm.txt
../disasm/smdasm.exe rom_sm.bin pc_sm.txt sm_full.txt linear    # full-ROM multi-entry linear
```

The directory is gitignored except this note (tracked so git keeps the folder
in place); drop the `.txt` files here to make the verification step runnable.
