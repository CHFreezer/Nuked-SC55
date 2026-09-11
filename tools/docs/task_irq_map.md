# Nuked-SC55 mkII — main firmware task/IRQ map and PCM voice-enable contract

Scope: SC-55mk2 (`ROM_SET_MK2`) main H8/532 firmware (`build/rom1.bin` + `build/rom2.bin`)
and the GT emulator (`src/`). Research/documentation only; no source was modified.

Evidence tiers (per `evidence_protocol.md`):
- **T0** = `src/` (mcu.cpp, mcu_opcodes.cpp, mcu_interrupt.cpp, mcu_timer.cpp, pcm.cpp/h) + raw ROM bytes.
- **T1** = runtime observations (`-savesnap` console `SNAP ...`, `LCDEN ...`, `-pcmtrace` log).
- **T2** = disassembler output (`tools/baselines/dasm_full.txt`, `h8dasm`) — fallible, cross-checked against raw bytes.
- **T3** = older docs (`dasm_annotated.txt`, `mk2_polyphony_256.md`, ...) — hypotheses only.

Address convention: `0xNNNNNN` = **flat** (cp:pc for main). rom2 flat = `0x40000 + file offset`.
All ROM byte citations were re-read from `build/rom1.bin`/`build/rom2.bin` (T0).

## 0. TL;DR

- The main firmware runs a single idle context that sleeps at flat `0x0491` and is driven by a
  **periodic FRT2 OCIA heartbeat** (vector 0xA4 → flat `0x0342`). The handler polls the SM/host
  queue and, when work is pending, jumps into the main event dispatcher, which returns through the
  epilogue at flat `0x04FE–0x0508`. `g_isr4fe` (`src/mcu.cpp:1084-1087`, printed by `-savesnap` as
  `isr=`) counts those dispatches — it is a system-liveness oracle (stock ≈1382 / 20M mcu cycles;
  stalled builds ≈75).
- **Voice-enable registers 0..3 carry exactly 32 bits**: `reg0`=voices 24-31, `reg1`=16-23,
  `reg2`=8-15, `reg3`=0-7, bit b = voice (base+b) within each byte. The firmware keeps this as two
  16-bit words `d150` (voices 16-31) and `d152` (voices 0-7/8-15) plus pending words `d154`/`d156`,
  and writes nothing else to the PCM mask window. It never writes registers 4..7 (those are the
  voice-parameter write latch) and never touches `0xE800`.
- **>32 voices is NOT feasible without changing the firmware mask path.** GT cannot invent
  enable bits (`src/pcm.cpp:589-590,1169` gates each slot on `voice_mask`), the mask writer is a
  hard-wired 32-bit accumulator, and `BSET (dp,0xd154) r1` uses `bit = r1 & 0xf`
  (`src/mcu_opcodes.cpp:1011-1025`), so voice indices ≥32 alias into voices 16-31.
- A minimal in-place patch for **≤64 voices** is possible (new mask words + 7 call-site redirects +
  3 replacement routines in the free rom1 cave `0x7DC4+`); the full 28→255 route remains the
  B-block design already planned in `tools/docs/plan_256.md` / `polyphony_256_todo.md` (R2/R9/R15/R17).

---

## 1. Main firmware task / interrupt architecture

### 1.1 Execution model (GT)

- `MCU_ReadInstruction` reads one instruction, `mcu.cycles += 12` per instruction
  (`src/mcu.cpp:1378-1380`); `MCU_Interrupt_Handle()` runs **before** each instruction
  (`src/mcu.cpp:1372-1378`).
- Interrupt dispatch order: TRAPA (vectors 16..31) → CPU exceptions → NMI → hardware IRQs by
  priority (`src/mcu_interrupt.cpp:65-209`). A hardware IRQ is taken when
  `(sr >> 8) & 7 < level`; the handler frame pushes pc, cp, sr
  (`src/mcu_interrupt.cpp:21-33`).
- The main CPU idles at `sleep` (flat `0x0491`) / `BRA` to itself (`0x0492`). Only an interrupt
  leaves the loop; `MCU_Interrupt_Start` clears `mcu.sleep` (`src/mcu_interrupt.cpp:32`).
- `br` is the H8 base register; `(br,$xx)` = `(br<<8)|xx` on page 0
  (`src/mcu_opcodes.cpp:589-596`). Firmware sets `br=0xE0` (PCM window `0xE000`) at
  `0x41270` / `0x41339` and keeps it there.
- Page registers: `r0-r3→dp`, `r4-r5→ep`, `r6-r7→tp` (`src/mcu.h:208-215`); `dp=0` reaches
  SRAM `0x8000-0xDFFF` and global ram `0xFB80-0xFF7F`.

### 1.2 Vector table (`rom1[0x0000..0x00FF]`, 4-byte big-endian entries)

Raw bytes re-read from T0. Unused/reserved vectors point to flat `0x00E4`, which is a 1-byte
`rte` stub (`rom1[0xE4] = 0x0A`).

| Vector | Entry | Target (flat) | Meaning / handler |
|---|---|---|---|
| 0 | `rom1[0x00..03] = 00 00 01 6C` | `0x016C` | RESET: `LDC #0 r7`; `0x016F` boot |
| 16 | `rom1[0x40..43] = 00 00 04 12` | `0x0412` | TRAPA #0x10 → clear/ack event flag |
| 17 | `0x44` | `0x02CC` | TRAPA #0x11 → enqueue timestamped event |
| 18 | `0x48` | `0x03D0` | TRAPA #0x12 → set event flag + dispatch |
| 27 | `0x6C` | `0x02AC` | TRAPA #0x1B → ack/clear event bit7 |
| 32 | `rom1[0x80..83] = 00 00 05 22` | `0x0522` | IRQ0 = PCM voice IRQ |
| 33 | `0x84` | `0x7B4C` | IRQ1 |
| 37 (0x94) | `0x94` | `0x414B0` → `0x7B6A` | FRT1 OCIA |
| 38 (0x98) | `0x98` | `0x414B4` → `0x46591` | FRT1 OCIB |
| 41 (0xA4) | `0xA4` | `0x0342` | FRT2 OCIA (**heartbeat**) |
| 42 (0xA8) | `0xA8` | `0x414B7` → `0x46595` | FRT2 OCIB |
| 45 (0xB4) | `0xB4` | `0x414AB` → `0x461AC` | FRT3 OCIA |
| 46 (0xB8) | `0xB8` | `0x7D88` → `0x7D8A` | FRT3 OCIB (**per-frame DSP**) |
| 48/49 (0xC0/C4) | `0xC0/C4` | `0x414BA`/`0x414BD` → `0x46599`/`0x4659D` | TMR CMIA/CMIB |
| 56 (0xE0) | `0xE0` | `0x7B5A` | ADI (analog) |

### 1.3 Task map

| Entry (flat) | Trigger | What it does (T0/T2 evidence) | Data touched | Periodicity |
|---|---|---|---|---|
| `0x016C`/`0x016F` | RESET | Boot init; `movi r7 #0xFDC0`, SR/ports, copy init tables to ram, `pjsr 0x41444`, `pjsr 0x41333`, clear `fe18[64]`, program FRT2 (`0x294-0x2A3`: `0xFFA0=0x22`, `0xFFA1=0x01`, `0xFFA2/3=0`, `0xFFA4/5=0x0177`), `jmp 0x0462` | dev regs, ram 0xFB80+, SRAM | once |
| `0x0462`–`0x0492` | boot / returns | Main loop: manage event count `fdc4`, `IML=0`, check event slots, sleep at `0x491` | `fdc0/fdc2/fdc4` | event-driven |
| `0x04F0`–`0x0508` | dispatcher tail (not an entry) | Save `fdc0=r6`, `fdc2=r3`; if `fe58==0`: pop r3/r4/r5, `ldm #0x7F`, pop dp, `rte` | `fdc0/fdc2`, stack | per dispatched event |
| `0x0509`–`0x0515` | `fe58!=0` only | Stack unwind: limit from `rom2[0x1434+r6]`, pop until `r7 >= limit`; dead in traced runs | stack | rare/mode-specific |
| `0x0342` (2B `BF 9D`)→`0x0344-0x03BA` | FRT2 OCIA (vector 0xA4) | Heartbeat. Reads `0xFFA1` (FRT2 TCSR), clears OCB/OCIA flag bit5; `fe16 = (fe16+1)&0x7FFF`; scans 6-entry table at `fe08` and 64-entry counters at `fe18`; on pending work jumps to dispatcher (`0x3BB→0x4AF`), else `rte` at `0x3BA` | `fe08..fe18`, `fdc*`, `fdd4`, `fde2` | periodic (programmed at boot) |
| `0x0522` (2B `B7 9D`)→`0x0524-0x053D` | IRQ0 (PCM voice end) | `r0 = PCM status (0xE03E) & 0x1F`; `d15c[r0] = 0xFF` (completion flag); set `r0=2,r1=1`; `jmp 0x03DB` (set event) | `d15c[]`, PCM 0xE03E | voice-event driven |
| `0x02AC`–`0x02CB` | TRAPA #0x1B (v27) | ack: clear bit7 at `fdd4 + fdc0` (queue slot), clear `fdf2` | `fdc0`, `fdd4`, `fdf2` | event-driven |
| `0x02CC`–`0x030F` | TRAPA #0x11 (v17) | enqueue a timestamped event; writes ring counters at `fe16/fe18`, event slots `fdd4/fdfa` | `fe16`, `fe18`, `fdfa` | event-driven |
| `0x03D0`→`0x03DB`–`0x03FC` | TRAPA #0x12 (v18) | set event flag `BSET fdd4+r0, r1`; if masked-in, queue index in `fde2[2*fdc0]`, advance `fdc2`; `rte` | `fdd4`, `fde2`, `fdc2` | event-driven |
| `0x0412`→`0x0416`–`0x043D` | TRAPA #0x10 (v16) | clear event flag; count trailing zero → index; `BCLR fdd4+idx`; `rte` (plus 0x043E path that pops the saved frame → `0x04AF`) | `fdd4`, `fdc0`, `fde2` | event-driven |
| `0x7B6A` | FRT1 OCIA | clear `0xFF90/0xFF91` bit5; set event `r0=4,r1=0` → `jmp 0x03DB` | FRT1 regs, `fdd4` | periodic |
| `0x46591` | FRT1 OCIB | empty stub: `LDC @r7++ r0` (pop SR) + `ret` (T0 bytes `CF 88 11 19`) | — | no-op |
| `0x46595` | FRT2 OCIB | empty stub (same 4 bytes) | — | no-op |
| `0x461AC` | FRT3 OCIA | clear `0xFFB1` bit5; `0xFFB4 += 0x0BB8`; decrement `d37f/d37e/d424/d425[]`; optionally `pjsr 0x41498/0x41493` | `ffb*`, `d37e..d425` | periodic |
| `0x7D8A` | FRT3 OCIB | clear `0xFFB1` bit6; `0xFFB6 += d68c`; `pjsr 0x414D8` → `jmp 0x4ACA9` (per-frame DSP) | `ffb*`, `d68c` | per frame |
| `0x46599`/`0x4659D` | TMR CMIA/CMIB | empty stubs (same `CF 88 11 19`) | — | no-op |
| `0x7B5A` | ADI (analog) | analog/ADC event | `dev_register` | event-driven |
| `0x00E4` | unused vectors | `rte` | — | — |

Trap vector note: `trapa #0x1n` → vector `16+n` (`src/mcu_opcodes.cpp:185-196`). `trapa #0x17/0x18`
hit vector 23/24 → `0x00E4` rte stub (used at `0x41268`, `0x4131D`).

### 1.4 Boot / init fragments

| Flat | Evidence | Role |
|---|---|---|
| `0x016C-0x02A9` | dasm lines 3-100; T0 bytes | Reset init: `br=0xFF` device setup, ram globals, `0xE400` GA/LCD regs, FRT2 programming, `jmp 0x0462` |
| `0x41220`–`0x41262` | dasm 13219-13239; T0 | Clear `d150/d152/d154/d156` (PCM mask state), then per-voice loop `r0=27..0`: `CLR ad0e[i]`, write `0x16` to `@(0x64d6+2*i)` +0/+2/+4, `CLR d15c[i]`; `trapa #0x11`; `ret` |
| `0x41266`–`0x41324` | dasm 13242-13304 | init `d1a0/d18e/d194/d19c`, copy `0x802B..0x8039` into `d181..d18d`, channel loop `r1=27..0` writing `0xFF` via `@r2+0x64d6`, `bsr16 0x2B8A`, `trapa #0x18/#0x11`; `ret` |
| `0x41333`–`0x4142D` | dasm 13307-13411; PCM trace | PCM/channel establishment: reads config table `rom2[0x41432]` (word = `C3 7B`), `config 0x3C/0x3D = 0xC3/0x7B`, zeroes mask regs 0/2, selects channels `27..0` via `(br,$3E)`, then `0xFFFF` to regs 0/1+2/3 (all-ones mask test) followed by zeroes, initializes channels 30/31; returns. Called by boot `pjsr 0x04:1333` (`0x0217`) and on demo power-cycle (~215.37M) |
| `0x41444`→`0x46621`→`0x41447` | dasm 13417-13421, 16546 | GA/LCD init: `r6=0x17`, `0xE401 = r6`, `0xD467`, clears `0xE402`, sets `0xFE70=8`, `0xD45C`, `0xDC2B/0xDC99`, `0xFFFE=0x1C`, `0xE406=0x40`; returns |
| `0x4329F`–`0x433C2` | dasm 13840-13968 | Maintenance/aging path: reset `d1d6..d1dd`, clear 32-byte arrays `d435` and fill 32 bytes `d1ac` with 4 (`0x4331E`/`0x43329`), set `d311/d6a4/d1d3/d1d4/d1d5`, adjust `0xFFF3/0xFFF7`, then a trap-paced loop (`trapa #0x11/#0x10`) calling `0x433D1/0x43641/0x434CB/0x4342A`; loop `BRA 0x43370`. Reached via stub `0x414A2` |

### 1.5 The `0x04FE-0x0508` path, health and 75 vs 1382

The only entries to the dispatcher (`0x4AF`) are `0x03CD` and `0x040F`, both inside the FRT2
handler (`flow_main.txt` confirms: `000003cd 000004af`, `0000040f 000004af`). Therefore:

- Every FRT2 OCIA tick either returns immediately (`rte 0x03BA`) or dispatches one queued event and
  exits through `0x04F0 → 0x04FC → 0x04FE..0x0508` (`fe58==0` in all traced stock runs; the `0x0509`
  unwind is never reached — `pc_main.txt` has no `0x0509`).
- `g_isr4fe` (`src/mcu.cpp:1087`) counts fetches at `pc==0x04FE`, i.e. **completed event
  dispatches**, and is printed as `isr=` by `-savesnap` (`src/mcu.cpp:1454`).
- Stock (T1, reproduced): `-savesnap 20000000` → `isr=1382`, `pc=00:0492`, `sleep=1`, `cfg3d=7b`.
- Broken builds (T1, reproduced `-voices:48/:64`) stall **inside** the FRT2 handler at `pc=0x037A`
  with `IML=7` (`sr=0705/0701`); the dispatcher is never reached and `isr` stays ≈75–76 after 20M.
  So the number measures whether the periodic task keeps completing — not the raw timer period.
  (The boot constants imply a much lower raw FRT2 rate under GT's timer model; the exact divisor
  between ticks and dispatches is unresolved and not required for the oracle.)

### 1.6 Data structures touched (global ram, page 0)

| Address | Use (from code) |
|---|---|
| `0xFB80-0xFF7F` | 1KB global ram (`ram[]`, needs `DEV_RAME` bit7; set at boot) |
| `0xFDC0/FDC2/FDC4` | dispatcher queue state: current class, index/count, work counter |
| `0xFDD4..0xFE17` | event flag structures (7 bytes each; `BSET/BCLR @r2 r1`, mask at `+7`); `fdd7` bit7 ack |
| `0xFDE2+` | saved frame pointers per event class (`MOVG3 r7 -> @r6+0xFDE2`) |
| `0xFE08..0xFE15` | 6-entry poll table (FRT2 handler `r2=0xFE08`, `r3=6`) |
| `0xFE16` | 16-bit heartbeat counter (`+1`, `&0x7FFF`) |
| `0xFE18..0xFE57` | 64 byte counters (`ADDQ #1 @r1+0xFE18`, index `&0x3F`) |
| `0xFE58` | dispatcher unwind selector |
| `0xFB80+0xD0..0xD16x` | voice engine state (`d0e0` busy/kind, `d150..d156` PCM mask, `d15c` IRQ-pending); these are SRAM (`0x8000-0xDFFF`), not `ram[]`; the reloc patch moves `d15c` (`0x531/0x51E0/0x51EF/0x553A/0x516C/0x41257` → newbase `0x9D40`) |
| `0xFE5A..0xFE70` | misc globals (init/queue) |

---

## 2. Voice-enable mask contract

### 2.1 GT truth (T0)

Writer side (`src/pcm.cpp:82-103`):
- `PCM_Write(addr)` masks `addr &= 0x3f`.
- `addr 0..3` update `voice_mask_pending[3], [2], [1], [0]` respectively:
  - `reg 0` → `pending[3]` = **voices 24..31** (stock mode keeps only low nibble: voices 24..27)
  - `reg 1` → `pending[2]` = **voices 16..23**
  - `reg 2` → `pending[1]` = **voices 8..15**
  - `reg 3` → `pending[0]` = **voices 0..7**
- Sets `voice_mask_updating = 1`.

Latch/consumer side:
- On the next read of any main-window address `< 4`, `voice_mask = voice_mask_pending`
  (`src/pcm.cpp:215-220`). **The readback is the commit point.**
- `voice_active[i] = voice_mask[i] & voice_mask_pending[i]` and
  `key = (voice_active[slot>>3] >> (slot&7)) & 1` (`src/pcm.cpp:589-590, 1169`).
- Bit/byte rule (`src/pcm.h:35-37`): **bit N = voice N; byte i = voices 8i..8i+7** (LSB = lowest
  voice of the byte).

Word-write semantics: `movsw rN @(br,$xx)` is a 16-bit write (`src/mcu_opcodes.cpp:779-799`) that
`MCU_Write16` splits big-endian (`src/mcu.cpp:1075-1080`): high byte → lower PCM address.
Therefore a firmware word write to `(br,$00)` writes `reg0 = rN>>8`, `reg1 = rN&0xFF`.

### 2.2 Firmware producer side (T0 disassembly + T1 trace)

Mask state (all 16-bit words, page 0):
| Variable | Meaning |
|---|---|
| `d150` | current mask, voices 16-31: bit i = voice 16+i (low byte = 16-23, high byte = 24-31) |
| `d152` | current mask, voices 0-15: bit i = voice i |
| `d154` | pending bits for voices 16-31 |
| `d156` | pending bits for voices 0-15 |

Writer routines:
- **Set pending bit** `0x546E–0x5484`:
  `r1 = @r0-2`; if `r1 <= 15` → `BSET (dp,0xd156) r1`; else `r1 -= 16` → `BSET (dp,0xd154) r1`.
  (`BSET` with a register bit uses `bit = r1 & 0xF`, `src/mcu_opcodes.cpp:1011-1025`.)
  Call sites (T0 + `flow_main.txt`): `0x5346`, `0x534D`, `0x53BD` (`bsr16 0x546E`).
- **Disable/clear flush** `0x54FB–0x552B`: gated on `@(d0e0+idx)==0`;
  `r3 = d150 & ~d154`, `r4 = d152 & ~d156`, store back, then
  `movsw r3 @(br,$00)` (regs 0/1), `movsw r4 @(br,$02)` (regs 2/3), `movl r6 @(br,$00)` readback.
  Call sites: `0x5366`, `0x53CF`.
- **Enable/set flush** `0x564A–0x5670`: `r5 = d150 | d154` → regs 0/1; `r6 = d152 | d156` → regs
  2/3; store `d150/d152`; readback `movl r6 @(br,$00)`; clear `d154/d156`.
  Call sites: `0x5373`, `0x53D5`.
- Boot/test writes: `0x4134C` (word config 0x3C/0x3D from `rom2[0x1432]`),
  `0x4134F/0x41354` (zero mask), `0x4139D/0x413A2` (`0xFFFF` all-ones), `0x413A7` readback,
  `0x413A9/0x413AD` (zero).

T1 confirmation (`-mocknote 144000000 -pcmtrace -savesnap 150000000`, stock):
```
pcm reg=00 val=00 pc=00:5527 cyc=144051996   ; disable flush: reg0=hi(r3)=0, reg1=lo(r3)=0
pcm reg=01 val=00 pc=00:5527 ...
pcm reg=02 val=00 pc=00:5529 ...             ; regs 2/3 written by 0x5527
pcm reg=03 val=00 pc=00:5529 ...
pcm reg=00 val=08 pc=00:5664 cyc=144052980   ; enable flush: reg0=0x08 -> voice 24+3 = 27
pcm reg=01 val=00 pc=00:5664 ...
pcm reg=02 val=00 pc=00:5666 ...
pcm reg=03 val=00 pc=00:5666 ...
```
(The trace logs the **next PC**; `0x5525/0x5527` and `0x5662/0x5664` are the two word writes.)

### 2.3 Exact bit mapping (main window)

| PCM reg | `voice_mask` byte | voices | firmware word | firmware bits |
|---|---|---|---|---|
| 0 | 3 | 24..31 | `d150` high byte | bit 8..15 of `d150` |
| 1 | 2 | 16..23 | `d150` low byte | bit 0..7 of `d150` |
| 2 | 1 | 8..15 | `d152` high byte | bit 8..15 of `d152` |
| 3 | 0 | 0..7 | `d152` low byte | bit 0..7 of `d152` |

So in GT terms, `voice_mask = (d150 << 16) | d152` with bit N = voice N (for N=0..31), and
`voice_mask_pending = (d154 << 16) | d156`.

### 2.4 What happens for slots ≥ 32

1. **No registers 4..7 exist for the mask.** A complete `grep` of `dasm_full.txt` for writes to
   `(br,$00..$07)` finds only:
   `$00/$02` at `0x5525/0x5527/0x5662/0x5664/0x4134F/0x41354/0x4139D/0x413A2/0x413A9/0x413AD`
   (voice enable / all-ones test) and `$05/$06` at `0x5559/0x555B`, `0x629E/0x62A0`,
   `0x413FA/0x413FC`, `0x41402/0x41404` (voice-parameter write latch).
   In GT those `4..0xF` addresses are the parameter write latch (`src/pcm.cpp:153-181`).
2. **The mask state is exactly 32 bits.** `d150/d152` are 16-bit words; there is no storage for
   voices ≥32 and no code path that computes one.
3. **Index aliasing.** The only bit-set path (`0x547A`/`0x5480`) subtracts 16 and lets `BSET` take
   `r1 & 0xF`; a voice index of 32..47 sets a bit in voices 16..31, 48..63 aliases again, etc.
4. **Nothing ever addresses `0xE800`.** No ROM bytes reference the extension window
   (`grep` over both ROMs for `0xE8xx` window accesses returns the GA/LCD block `0xE400`
   only). The 256-bit `voice_mask` in GT therefore keeps bits 32..255 at 0 forever, and
   `PCM_Update` can never key those slots (`src/pcm.cpp:589-590,1169`).
5. **IRQ channel truncation.** Voice-end IRQ stores `pcm.irq_channel = slot` (full index,
   `src/pcm.cpp:1533`) but the status read exposes only `irq_channel & 0x1f`
   (`src/pcm.cpp:233`), and the firmware masks it again (`AND #0x1F`, `0x052D`). A slot ≥32 would
   be misattributed to slot `slot-32`.

### 2.5 GT extension window mapping (authoritative)

`PCM_WriteExt`/`PCM_ReadExt` are mapped at `0xE800-0xE83F` when `pcm_ext_active && !mcu_jv880`
(`src/mcu.cpp:696-699, 955-958`):
- ext offset `a` with `a < 0x1C` writes/reads `voice_mask_pending[4+a]` (`src/pcm.cpp:288-296`).
  So **ext byte `a` = mask byte `4+a` = voices `32+8a .. 39+8a`**, LSB = lowest voice.
- ext `0x20` = full IRQ slot (`irq_channel & 0xFF`, read-only, `src/pcm.cpp:303-304`);
  ext `0x21` = extension-active probe (`src/pcm.cpp:305-307`).
- Ext writes set `voice_mask_updating=1`; the latch is still the main-window read (see 2.1).

> **Correction of the working shorthand.** "route mask writes for slots 32..N-1 into `0xE800+4..7`"
> is off by four: on this GT, voices 32..63 are mask bytes 4..7, which live at **`0xE800+0x00..0x03`**.
> `0xE800+4` would be voices 64..71. All patch proposals below use the T0 mapping.

---

## 3. What must change to support >32 voices

### 3.1 Verdict

**>32 voices is not feasible by changing GT alone, and the stock firmware mask path cannot carry it.**
Any working design must extend the firmware side (extra mask state + ext-window writes + IRQ slot
fix). The GT side for the mask mapping is already in place (stage 1 in `plan_256.md`).

### 3.2 Current in-tree status

- `MCU_PatchROM()` calls only `PCMEXT_BuildReloc()` (`src/mcu.cpp:1583-1592`), which:
  relocates every referenced per-voice array to free SRAM (`src/reloc_data.h`), stamps 12 bound
  immediates (`kRelocImms`, incl. the mask-accumulator `0x1AD7` and IRQ scans `0x51D9/0x51FC`),
  writes `rom2[0x1433] = n-1`, and sets `pcm_ext_active=1` (`src/patch_reloc.cpp:23-80`).
  It does **not** add any ext-window write, does **not** touch the `0x3E` effect select, and does
  **not** touch the IRQ channel.
- `src/patch_256.cpp` (B block, 255-voice rewrite) is present but **not wired in**
  (`g_all_enabled = 0` at line 623; `MCU_PatchROM` never calls `PCMEXT_BuildPatch`). Its planned
  IRQ patch `kIPatch` (`0x529 → 0xE820`, `0x52D → 0x00FF`, lines 218-221) alone is insufficient:
  replacing the `0xE03E` read removes the PCM IRQ ack (see 3.5).
- Empirical status (T1, `-savesnap 20000000`, this machine):
  | run | `isr=` @20M | `pc` | `cfg3d` | verdict |
  |---|---|---|---|---|
  | stock (no flag) | 1382 | `00:0492` | `7b` (=28) | healthy |
  | `-voices:32` | 1395 | `00:0492` | `1f` (=32) | healthy |
  | `-voices:33` | 1439 | `00:0492` | `20` (=33) | boots; mask cannot represent voice 32 |
  | `-voices:40` | 1473 | `00:0492` | `27` (=40) | boots; mask cannot represent voices 32-39 |
  | `-voices:48` | 76 | `00:037A` (FRT2 ISR), `IML=7` | `2f` (=48) | stalled |
  | `-voices:64` | 75 | `00:037A`, `IML=7` | `3f` (=64) | stalled |

  So the shipped relocation provides a healthy **≤32** extension (no mask change needed), but its
  "up to 64" claim is wrong twice: the mask path cannot represent >32, and the build stalls ≥48.

### 3.3 Option A (recommended): finish the B block (255 voices = compromise cap, goal 256)

Follow `plan_256.md` and `polyphony_256_todo.md` R2/R9/R12/R15/R17:
- 32-byte current/pending mask arrays in page 6 (`MHI`/`PHI` in the plan: `0x3300`/`0x3320`),
  submit table of 14 `movsw` writes covering ext offsets `0x00..0x1A` (voices 32..247) plus the
  main-window words.
- **Ordering:** write the ext bytes **before** the main-window readback that latches
  (`0x5529`/`0x5666` equivalent), exactly as R9 requires.
- `reg_slots` config byte `rom2[0x1433]=n-1`; GT already switches formulas when `pcm_ext_active`.
- IRQ: read full slot from `0xE820`, then ack (see 3.5).
- Effects: for `n>28`, 1-byte patch all effect-programming `0x3E` select writes to `0x3F`
  (`src/pcm.cpp:147-152` alias already exists; R17).

### 3.4 Option B (minimal): in-place 64-voice mask extension

This is the smallest *complete* mask contract fix for `n ≤ 64`; it is an alternative to the B
block, using the free rom1 code cave at `0x7DC4` (T0: `0x7DC4-0x7FFF` is 0xFF padding).

**New state (free in both stock and reloc builds: the reloc table puts `d15c` at `0x9D40`
(64-byte region `0x9D40-0x9D7F` for 64 slots) and never uses `0x9D80-0x9DFF`):**

| Address | Word | Meaning |
|---|---|---|
| `0x9D80` | `CH0` | current, voices 32-47 (hi byte=32-39, lo=40-47) |
| `0x9D82` | `CH1` | current, voices 48-63 |
| `0x9D84` | `PH0` | pending, voices 32-47 |
| `0x9D86` | `PH1` | pending, voices 48-63 |

**Redirect 7 call sites** (bsr16: `1E <disp16>`, branch target = `pc+3+disp16`; all 3-byte forms,
verified against ROM bytes):

| PC | old bytes / target | new bytes / target |
|---|---|---|
| `0x5346` | `1E 01 25` → `0x546E` | `1E 2A 7B` → `0x7DC4` |
| `0x534D` | `1E 01 1E` → `0x546E` | `1E 2A 74` → `0x7DC4` |
| `0x53BD` | `1E 00 AE` → `0x546E` | `1E 2A 04` → `0x7DC4` |
| `0x5366` | `1E 01 92` → `0x54FB` | `1E 2A FA` → `0x7E63` |
| `0x53CF` | `1E 01 29` → `0x54FB` | `1E 2A 91` → `0x7E63` |
| `0x5373` | `1E 02 D4` → `0x564A` | `1E 2A 9C` → `0x7E12` |
| `0x53D5` | `1E 02 72` → `0x564A` | `1E 2A 3A` → `0x7E12` |

(Verify with a full-ROM static scan for bsr/jsr/flow into `0x546E/0x54FB/0x564A`, not only
`dasm_full` PCs.)

**Replacement A - set pending bit, `0x7DC4`** (input `r0` = per-voice pointer, index at `@r0-2`;
preserves all registers except r1):

```
7DC4: E8 FE 81     MOVG2 @r0+-2 r1
7DC7: 49 00 0F     cmp r1,w #0x000F
7DCA: 23 41        BLS  -> 7E0D        ; idx 0..15  -> d156
7DCC: 49 00 1F     cmp r1,w #0x001F
7DCF: 23 33        BLS  -> 7E04        ; idx 16..31 -> d154
7DD1: 49 00 27     cmp r1,w #0x0027
7DD4: 23 25        BLS  -> 7DFB        ; idx 32..39 -> PH0 (bit +8)
7DD6: 49 00 2F     cmp r1,w #0x002F
7DD9: 23 17        BLS  -> 7DF2        ; idx 40..47 -> PH0
7DDB: 49 00 37     cmp r1,w #0x0037
7DDE: 23 09        BLS  -> 7DE9        ; idx 48..55 -> PH1 (bit +8)
7DE0: 0C 00 38 31  SUB #0x0038 r1      ; idx 56..63 -> PH1
7DE4: 1D 9D 86 49  BSET (dp,0x9D86) r1
7DE8: 19           rts
7DE9: 0C 00 28 31  SUB #0x0028 r1
7DED: 1D 9D 86 49  BSET (dp,0x9D86) r1
7DF1: 19           rts
7DF2: 0C 00 28 31  SUB #0x0028 r1
7DF6: 1D 9D 84 49  BSET (dp,0x9D84) r1
7DFA: 19           rts
7DFB: 0C 00 18 31  SUB #0x0018 r1
7DFF: 1D 9D 84 49  BSET (dp,0x9D84) r1
7E03: 19           rts
7E04: 0C 00 10 31  SUB #0x0010 r1
7E08: 1D D1 54 49  BSET (dp,0xD154) r1
7E0C: 19           rts
7E0D: 1D D1 56 49  BSET (dp,0xD156) r1
7E11: 19           rts
```

**Replacement B - enable flush, `0x7E12`** (mirrors `0x564A` + high words; restores `br=0xE0`):

```
7E12: 1D D1 50 85  MOVG2 (dp,0xD150) r5
7E16: 1D D1 52 86  MOVG2 (dp,0xD152) r6
7E1A: 1D D1 54 45  OR    (dp,0xD154) r5
7E1E: 1D D1 56 46  OR    (dp,0xD156) r6
7E22: 1D D1 50 95  MOVG3 r5 -> (dp,0xD150)
7E26: 1D D1 52 96  MOVG3 r6 -> (dp,0xD152)
7E2A: 7D 00        movsw r5 @(br,$00)     ; main regs 0/1 (voices 16-31)
7E2C: 7E 02        movsw r6 @(br,$02)     ; main regs 2/3 (voices 0-15)
7E2E: 1D 9D 80 85  MOVG2 (dp,0x9D80) r5
7E32: 1D 9D 82 86  MOVG2 (dp,0x9D82) r6
7E36: 1D 9D 84 45  OR    (dp,0x9D84) r5
7E3A: 1D 9D 86 46  OR    (dp,0x9D86) r6
7E3E: 1D 9D 80 95  MOVG3 r5 -> (dp,0x9D80)
7E42: 1D 9D 82 96  MOVG3 r6 -> (dp,0x9D82)
7E46: 04 E8 8B     LDC #0xE8 r3            ; br = 0xE8 (control reg 3)
7E49: 7D 00        movsw r5 @(br,$00)      ; 0xE800/1 = voices 32-47
7E4B: 7E 02        movsw r6 @(br,$02)      ; 0xE802/3 = voices 48-63
7E4D: 04 E0 8B     LDC #0xE0 r3            ; restore br
7E50: 66 00        movl r6 @(br,$00)       ; readback: latches pending -> current in GT
7E52: 1D D1 54 13  CLR (dp,0xD154)
7E56: 1D D1 56 13  CLR (dp,0xD156)
7E5A: 1D 9D 84 13  CLR (dp,0x9D84)
7E5E: 1D 9D 86 13  CLR (dp,0x9D86)
7E62: 19           rts
```

**Replacement C - disable flush, `0x7E63`** (mirrors `0x54FB` gate + high words; no pending clear,
as stock):

```
7E63: E8 FE 81        MOVG2 @r0+-2 r1
7E66: F1 D0 E0 04 00  SUB @r1+0xD0E0 #0x0000   ; note: relocated build -> @r1+0x9C40
7E6B: 26 50           BNE -> 7EBD (rts)
7E6D: 1D D1 54 85     MOVG2 (dp,0xD154) r5
7E71: 1D D1 56 86     MOVG2 (dp,0xD156) r6
7E75: AD 15           NOT r5
7E77: AE 15           NOT r6
7E79: 1D D1 50 83     MOVG2 (dp,0xD150) r3
7E7D: 1D D1 52 84     MOVG2 (dp,0xD152) r4
7E81: AD 53           AND r5 r3
7E83: AE 54           AND r6 r4
7E85: 1D D1 50 93     MOVG3 r3 -> (dp,0xD150)
7E89: 1D D1 52 94     MOVG3 r4 -> (dp,0xD152)
7E8D: 7B 00           movsw r3 @(br,$00)
7E8F: 7C 02           movsw r4 @(br,$02)
7E91: 1D 9D 84 85     MOVG2 (dp,0x9D84) r5
7E95: 1D 9D 86 86     MOVG2 (dp,0x9D86) r6
7E99: AD 15           NOT r5
7E9B: AE 15           NOT r6
7E9D: 1D 9D 80 83     MOVG2 (dp,0x9D80) r3
7EA1: 1D 9D 82 84     MOVG2 (dp,0x9D82) r4
7EA5: AD 53           AND r5 r3
7EA7: AE 54           AND r6 r4
7EA9: 1D 9D 80 93     MOVG3 r3 -> (dp,0x9D80)
7EAD: 1D 9D 82 94     MOVG3 r4 -> (dp,0x9D82)
7EB1: 04 E8 8B        LDC #0xE8 r3
7EB4: 7B 00           movsw r3 @(br,$00)   ; 0xE800/1
7EB6: 7C 02           movsw r4 @(br,$02)   ; 0xE802/3
7EB8: 04 E0 8B        LDC #0xE0 r3
7EBB: 66 00           movl r6 @(br,$00)    ; readback/latch
7EBD: 19              rts
```

Constraints / required generator behavior:
- These bytes assume **relocation off**. If `PCMEXT_BuildReloc` is active, substitute every
  relocated base (e.g. `d0e0 → 0x9C40`, `d15c → 0x9D40`) in the new routines; `d150..d156` are not
  in the reloc table and stay.
- `br` must be `0xE0` on entry and is restored to `0xE0`; the low writes themselves rely on the
  firmware invariant `br=0xE0`.
- The **readback after the ext writes** is mandatory: GT latches pending→current only on a
  main-window read (`src/pcm.cpp:215-220`).
- Apply only when `n > 32`; `n ≤ 32` needs no mask patch (32-bit state is exact).
- Assemble with the project's C emitter + `tools/verify` (evidence protocol §14); hand-assembled
  bytes in this table were checked against GT operand/opcode semantics but have not been executed.

### 3.5 Additional changes needed beyond the mask (all n > 28/32)

1. **PCM IRQ slot (n > 32, also correct for n>28).** `0xE03E` status is 5-bit; the handler masks
   `AND #0x1F` at `0x052D`. Fix options:
   - firmware: read ext `0xE820` (full slot) **and still read `0xE03E` to ack**, then write
     `d15c[slot]`; or
   - GT: make `PCM_ReadExt(0x20)` also clear `irq_assert` and deassert IRQ0, so the 2-word patch
     `0x529: 15 E0 3E 80 → 15 E8 20 80` + `0x52D: 0C 00 1F 50 → 0C 00 FF 50` is sufficient.
   The current `patch_256.cpp` `kIPatch` does the substitution but not the ack — incomplete as is.
2. **Effect channels (n > 28).** Effect programming selects channels 28..31 via `0x3E`
   (effect cluster `0x5DDC-0x64C3` plus boot/effect selects such as `0x41377`/`0x41385` and the
   `0x5E06/0x5FE5/0x61E9/0x633F` sites in the PCM trace). In ext mode GT's `0x3E` means literal
   slot 28..31, so without the R17 patch effects program note voices. Fix: 1-byte `0x3E→0x3F` at
   the effect write/readback sites; GT's `0x3F` alias already maps to `PCM_EFF_BASE+(v&3)`
   (`src/pcm.cpp:147-152`).
3. **Relocation stall for n ≥ 48.** Reproduced (3.2); the relocation generator's per-array capacity
   must be fixed (or folded into the B block), otherwise no `n ≥ 48` build can run even with the
   mask patch.

### 3.6 GT-side changes

- Mask mapping: **none** for Option A/B; ext window, latch, 256-bit mask, EFF_BASE, reg_slots
  formula and `cycles` clamp are already implemented (`src/pcm.cpp`, `src/mcu.cpp:696-699,955-958`).
- Optional but recommended:
  - `PCM_ReadExt(0x20)` = IRQ ack variant (3.5.1) if firmware does not dual-read.
  - Save/restore `b_ram` (page 6) in `state_save/state_load` when the B block ships
    (`src/mcu.cpp:1254-1332`) — currently page6 state is lost across `-savesnap`.
  - Tie `pcm_ext_active` to the actual mask-patch installation, not just `PCMEXT_BuildReloc`
    (`src/patch_reloc.cpp:79`), so a broken build does not silently present ext semantics.
  - Lift the `-voices` clamp `>64 → 64` (`src/mcu.cpp:1586-1590`) when B/255 lands.
  - Extend `-pcmtrace` to also log `PCM_WriteExt` (0xE800) writes; today only `<=3/0x3C/0x3D/0x3E`
    are logged (`src/mcu.cpp:940-953`).

---

## 4. Verification oracles

All runs are from reset with the GT window + audio (per `plan_256.md` §6); use timeouts derived
from 24 MHz (e.g. 20M ≈ 1s of emulated time). Capture stdout to a file.

| # | Signal | Expected (stock / correct build) | Capture |
|---|---|---|---|
| O1 | LCD enable | `LCDEN 0 pc=04:662b cyc≈588` then `LCDEN 1 pc=04:668d cyc≈4.22M` (`4222836`..`4223244`). With `-demo`: `LCDEN 0` ≈145.72M then `LCDEN 1` ≈163.94M | stdout (`src/lcd.cpp:39-49` prints every transition) |
| O2 | Heartbeat/liveness | `isr≈1382` per 20M (`isr=1382` observed). Must keep growing; `-demo -savesnap 320000000` → `isr=21460` | `-savesnap <cycles>`, read `SNAP ... isr=` |
| O3 | Idle state | `pc=00:0492`, `sleep=1`, `iml=0`, `pend=0` at 20M | same as O2 |
| O4 | Config byte | stock `cfg3d=7b` (28 voices); extended build `cfg3d=n-1` (e.g. `-voices:32 → 1f`) | same as O2 |
| O5 | Note-driven main mask | `-mocknote 144000000 -pcmtrace -savesnap 150000000`: writes at `pc=00:5527` (disable flush) and `pc=00:5664` (enable flush, nonzero byte) around `cyc=144.05M` | `pcm_trace.log` |
| O6 | Extended ext writes (`n>32`) | after the note, nonzero writes to `0xE800..0xE803` (mask bytes 4..7) **before** the `0x5529/0x5666` readback. `-pcmtrace` does not log ext today (3.6); inspect `pcm.voice_mask[4..]` in `demo_snap.bin` (state_save writes `pcm`), or temporarily log `PCM_WriteExt` | snapshot / instrumented build |
| O7 | IRQ slot (`n>32`) | PCM IRQ for a slot ≥32 must set `d15c[slot]` with the full index; no aliasing to `slot-32`. Observable via `0xE820` read path and the `d15c` array after voices cross their loop points | disasm + `-pcmtrace`/snapshot |
| O8 | Survival past power-cycle | no reset/stall through the `-demo` power cycle at ≈215.37M: config re-init trace (`0x4134C` writes) and `isr` keeps increasing past the 200M level | `-demo -savesnap 220000000` (compare with `-savesnap 200000000`), or `-tracepc` window `214M..216M` |
| O9 | Regression | no flag: 200M-202M PC trace byte-identical to the frozen baseline (`tools/baselines`) | `-tracepc` diff |
| O10 | Failure signature | stall with `isr≈75`, `pc=00:037A`, `iml=7` (FRT2 handler), or MCU reset loop | `-savesnap 20000000` |

Recommended acceptance set for the Option-B build: O1, O2, O3, O4 (`n-1`), O5 (for `n≤32`) and O6
(for `n>32`), O9 (no-flag), plus a musical listen at ≥300M per the docs.

---

## 5. Evidence index

- Raw ROM: `build/rom1.bin` (vectors `0x00..0xFF`, `0xE4=0x0A`, code cave `0x7DC4+`=0xFF),
  `build/rom2.bin[0x1432..0x1433] = C3 7B`.
- Disassembly (T2, cross-checked): `tools/baselines/dasm_full.txt` lines cited inline;
  `pc_vec.txt`, `flow_main.txt`, `mach_vec.txt`.
- GT (T0): `src/pcm.cpp:82-103,147-152,215-220,233,288-308,587-590,1169,1527-1538`;
  `src/pcm.h:27-37,72-77`; `src/mcu.cpp:696-699,955-958,1064-1068,1075-1080,1084-1087,1583-1592,1915-1927`;
  `src/mcu_opcodes.cpp:185-196,460-539,779-800,1011-1025,1043-1097`;
  `src/mcu_interrupt.cpp:21-33,65-209`; `src/mcu_timer.cpp:206-335`; `src/mcu.h:23-98,100-158,203-215`;
  `src/patch_reloc.cpp:23-80`; `src/reloc_data.h:6-433`; `src/patch_256.cpp:126,218-221,623`.
- Plans/prior work (T3, used only as cross-reference): `tools/docs/plan_256.md`,
  `tools/docs/polyphony_256_todo.md` (R2/R9/R12/R15/R17), `tools/docs/mk2_polyphony_256.md`.
- T1 runs: commands and outputs quoted in §1.5, §2.2, §3.2, §4.

Open items (non-blocking for the contract):
- Exact divisor between FRT2 ticks and dispatches (raw period vs `isr` rate).
- Root cause of the `n≥48` relocation stall (`0x37A`).
- Semantic names for `d434/d435/d1ac` and the `0x4329F` maintenance loop.
