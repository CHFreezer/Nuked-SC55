# Voice memory map — H8 main firmware per-voice structures (stock SC-55mk2 / rom1+rom2)

Status: research/documentation (T3 doc), 2026-09-11. Every claim cites a PC in
`tools/baselines/dasm_full.txt` (T1 trace disasm over the 9217-PC stock set) or raw
ROM/SRAM bytes (T0). Confidence tags follow `evidence_protocol.md` §12:
**C**=Confirmed (ROM+src), **S**=Strongly supported (GT + trace/multi-site), **I**=Inferred.

Notation: `PC` = flat address as printed by `dasm_full.txt` (`0000xxxx`=rom1,
`0004xxxx`=rom2). `sram[a]` = H8 page-0 address `a`, physical `sram[a & 0x7fff]`
(`src/mcu.cpp:711-713`). Snapshot SRAM base: file offset `0x450`
(`sizeof(mcu_t)=0x50` + `RAM_SIZE=0x400`, `src/mcu.cpp:158-161,1256-1258`), so
`sram[a]` is at file `0x450 + (a & 0x7fff)`.

Operand width in the disasm is decided by bit3 of the operand byte (`operand & 0x08`,
`src/mcu_opcodes.cpp:558-561`): `e0..e7/f0..f7` = byte, `e8..ef/f8..ff` = word.
This matters below (e.g. `MOVG2 @r0+46` is `[e8 2e 82]` = **word**, not byte).

---

## 1. Summary (answers to the required questions)

1. **Per-voice structure base/stride.** There are **28 AoS voice structs**, each
   **0x12a (298) bytes**, occupying SRAM **[0xad2e, 0xcdc6)** (0x2098 = 28×0x12a bytes).
   The firmware does **not** compute the base by multiplication; a **pointer table in
   ROM1 at flat `0x64d6`** holds 28 big-endian words, one interior pointer `P(v)` per
   voice slot `v`:
   `P(v) = rom1[0x64d6 + 2*v]` — values `0xadae, 0xaed8, 0xb002, … 0xcd1c`.
   Within each struct the word `P-2` holds the slot index `v` `[e8 fe 81]`.
   Struct `v` spans `[P(v)-0x80, P(v)+0xaa)`; consecutive structs are adjacent
   (`P(v+1) = P(v)+0x12a`). Slot 0 is `[0xad2e, 0xaed8)`, slot 27 is `[0xcc9c, 0xcdc6)`.
   Evidence: `0x41239-0x4125b` (init loop reads the table and writes `P+0/2/4`),
   `0x53eb-0x53f3` (pointer resolution + slot write-back), raw `rom1.bin[0x64d6..0x650d]`.
2. **The `r1=0xcc9c` observation is not a struct base.** `0xcc9c = P(27) - 0x80`, the
   *sub-block* base of the last slot. `r1` is computed at `0x37f7` (`ADD #0xff80 r1`,
   `[0c ff 80 21]`) or `0x38ac` (`ADD #0xffa2 r1`, `[0c ff a2 21]`) before `0x3970`.
   The stock crash log line `CRASH@3970 r1=cc9c …` (`build/cr_stock.out`,
   `build/ib_4.out`) therefore identifies the active voice, not a base constant.
   The debug hook that printed `struct@r1:` (`src/mcu.cpp:1088-1105`) read memory
   through `mcu.ep` while `r1` (reg 1) is paged by `dp` (`src/mcu.h:208-215`); its
   bytes match **rom2** (`build/ib_4.out` → `01 8a 01 8d 01 8f 01 90` ==
   `rom2.bin[0x1cc9c..]`; `build/case_nocfg.out` → `20 03 5d da 8c 1e ff 1a` ==
   `rom2.bin[0x25ad..]`), **not** the SRAM struct. Do not use those bytes as struct data.
3. **Field table.** See §4. The struct splits into a 0x80-byte sub-block
   `S = P-0x80` (two 0x22-byte interpolator records + flags/PCM bytes) and a 0xaa-byte
   main block `P..P+0xa9` (tone-pointer triples, DSP param mirrors, counters).
4. **The 49 candidate bases are all category (b): standalone SoA arrays**, not fields
   of the AoS struct. They are slot/note-indexed arrays accessed with the literal base
   (`@rN+0xa368`, etc.); every one is ≥0x8000, so page-0 SRAM, never ROM. None lies
   inside `[0xad2e,0xcdc6)`. The earlier model was incomplete, not wrong about these
   49: it missed the 0x2098-byte AoS region and the `0x64d6` pointer table (see §6).
5. **Capacity.** **28 native voice slots** (28 pointer-table words; 28 init iterations).
   There is no spare struct: slot 28 would start at `0xcdc6`, which is exactly where
   the SoA arrays `0xcdc6/0xcdfe/…` begin. 32-voice support needs relocation/extension
   of the struct region (the existing page-6 plan's layout table does not list this
   region — see §8).

---

## 2. Memory map

| Range (flat) | Size | Contents | Decisive evidence |
|---|---|---|---|
| `0x64d6-0x650d` (rom1) | 0x38 | **Voice pointer table**, 28 big-endian words `P(v)`, `P(0)=0xadae … P(27)=0xcd1c`, stride `0x12a` | read at `0x53ef` `MOVG2 @r3+0x64d6 r0 [fb 64 d6 80]` after `0x53ed ADD r3 r3 [ab 23]`; raw `rom1.bin[0x64d6]` = `ad ae ae d8 b0 02 … cb f2 cd 1c` |
| `0xa1xx-0xacf2` | — | SoA slot/note arrays (the 49-list group A); see §6 | `0x40469-0x404c2` clears/initialises many of them with `@r1+…` where `r1` = slot 27…0 |
| `0xad0e-0xad29` | 0x1c | `ad0e[28]` byte array (slot→PCM channel; `0xff`=free) | `CMP @r1+0xad0e r0 [f1 ad 0e 70]` `0x17c0`; clear `[f1 ad 0e 13]` `0x1839` |
| `0xad2a-0xad2d` | 4 | Globals: `(dp,0xad2a)` = rate/scale, `(dp,0xad2c)` temp | `MULXU (dp,0xad2a) r4:r5 [1d ad 2a ac]` `0x38bb`; `MOVG2 (dp,0xad2a) r6` / `MOVG3 r6 -> (dp,0xad2c)` `0x36da/0x36de`; `MOVG #1 -> (dp,0xad2a)` `0x36e2` |
| **`0xad2e-0xcdc5`** | **0x2098** | **28 AoS voice structs × 0x12a**, `[P(v)-0x80, P(v)+0xaa)` | table + `0x41248-0x41252` (`P+0/2/4 = 0x0016` for all 28); s28 snapshot §7 |
| `0xcdc6-0xd15b` | ~0x596 | Secondary SoA arrays (word `cdc6`, `cdfe`; byte `ce3f…d15c` etc.); §6 group B | `CLR @r1+0xcdc6 [f9 cd c6 13]` `0x3622` (r1 doubled at `0x3620`); `CLR @r1+0xcdfe [f9 cd fe 13]` `0x2748` (doubled at `0x2746`) |
| `0xd150-0xd156` | 8 | 32-bit voice mask: current `d150/d152`, pending `d154/d156` | `movsw r3 @(br,$00)` `0x5525`; `BSET (dp,0xd154) r1 [1d d1 54 49]` `0x547a`; `BSET (dp,0xd156) r1` `0x5480` |
| `0xd158-0xd15b` | 4 | Two 16-bit **live voice pointers** set from `P(slot)` | `MOVG3 r0 -> (dp,0xd158)` `0x52f7`; `MOVG3 r0 -> (dp,0xd15a)` `0x5300`; `MOVG #0xffff -> (dp,0xd15a)` `0x5396` |
| `0xd15c` | 28 | B byte array; PCM-IRQ pending / per-voice busy flags | `CLR @r1+0xd15c [f1 d1 5c 13]` `0x516c`; scan `TST @r1+0xd15c` `0x51e0` (r1=27..0) |

ROM1 table bytes (from `build/rom1.bin`, big-endian words):

```
64d6: ad ae ae d8 b0 02 b1 2c b2 56 b3 80 b4 aa b5 d4
64e6: b6 fe b8 28 b9 52 ba 7c bb a6 bc d0 bd fa bf 24
64f6: c0 4e c1 78 c2 a2 c3 cc c4 f6 c6 20 c7 4a c8 74
6506: c9 9e ca c8 cb f2 cd 1c
```

`P(v)` list: `adae aed8 b002 b12c b256 b380 b4aa b5d4 b6fe b828 b952 ba7c bba6
bcd0 bdfa bf24 c04e c178 c2a2 c3cc c4f6 c620 c74a c874 c99e cac8 cbf2 cd1c`.
Diff of any two neighbours = `0x12a` (e.g. `0xb002-0xaed8 = 0x12a`). The word after
the table (`0x650e`) is `0x0006` — not a pointer, so exactly 28 entries. **C**

Cross-check with the snapshots: for every one of the 28 values, `sram[P(v)+0]`,
`+2`, `+4` == `0x0016` in both `build/s28.bin` and `build/s256b.bin` (28/28), matching
the reset writes at `0x41248/0x4124d/0x41252`. `sram[P(27)-2..P(27)+5]` =
`00 00 | 00 16 00 16 00 16`. **C**

---

## 3. How a voice pointer is obtained (slot → P)

The table is read with `rX = 2*slot` and disp16 `0x64d6` (dp=0; `MCU_GetPageForRegister`
maps r0-r3→dp, r4-r5→ep, r6-r7→tp, `src/mcu.h:208-215`; `dp` is written 0 all over the
firmware, e.g. `LDC #0x00 r5` `[04 00 8d]` at `0x7d8a`, `0x41220`, `0x5313`):

| Site | Sequence | Raw bytes |
|---|---|---|
| `0x25f0` | `MOVG2 r1 r2; ADD r2 r2; MOVG2 @r2+0x64d6 r0; SUB @r0+0 #0x000e` | `[a9 82][aa 22][fa 64 d6 80][e8 00 05 00 0e]` |
| `0x4123c` | `MOVG2 r0 r2; ADD r2 r2; MOVG2 @r2+0x64d6 r1; CLR @r0+0xad0e` | `[a8 82][aa 22][fa 64 d6 81][f0 ad 0e 13]` |
| `0x41308` | `MOVG2 r1 r2; SHLL r2; MOVG2 @r2+0x64d6 r0; MOVG #0xff -> @r0+-26` | `[a9 82][aa 1a][fa 64 d6 80][e0 e6 07 00 ff]` |
| `0x5184` | `ADD r1 r1; MOVG2 @r1+0x64d6 r2; …MOVG3 r6 -> @r2+0` | `[a9 21][f9 64 d6 82][ea 00 96]` |
| `0x523e` | `MOVG2 r1 r2; ADD r2 r2; MOVG2 @r2+0x64d6 r2; SUB @r2+0 #0x12` | `[a9 82][aa 22][fa 64 d6 82][ea 00 04 12]` |
| `0x53eb` | `MOVG2 r1 r3; ADD r3 r3; MOVG2 @r3+0x64d6 r0; MOVG3 r1 -> @r0+-2` | `[a9 83][ab 23][fb 64 d6 80][e8 fe 91]` |
| `0x5889` | `MOVG2 r1 r2; ADD r2 r2; MOVG2 @r2+0x64d6 r0; SUB @r0+0 #0x12` | `[a9 82][aa 22][fa 64 d6 80][e8 00 04 12]` |
| `0x58bc` | `MOVG2 r3 r2; ADD r2 r2; MOVG2 @r2+0x64d6 r2` | `[ab 82][aa 22][fa 64 d6 82]` |
| `0x58ce` | `MOVG2 r1 r0; ADD r0 r0; MOVG2 @r0+0x64d6 r0` | `[a9 80][a8 20][f8 64 d6 80]` |
| `0x58de` | `MOVG2 r1 r2; ADD r2 r2; MOVG2 @r2+0x64d6 r0; CLR @r0+-26` | `[a9 82][aa 22][fa 64 d6 80][e0 e6 13]` |

The slot integer is written into the struct at `P-2` by `0x53f3` and read back at
`0x361d [e8 fe 81]`, `0x3aa7 [e8 fe 81]`, `0x531e`, `0x546e [e8 fe 81]`, `0x5540`,
`0x5626`, `0x2d7a`, `0x30f2`. Every voice-scope routine therefore recovers the slot
from its `P` argument and can index the 49 SoA arrays. **C**

Two "live voice pointer" globals are then used by the PCM/envelope update path:
`MOVG3 r0 -> (dp,0xd158)` `0x52f7` and `MOVG3 r0 -> (dp,0xd15a)` `0x5300` (both `r0`
just returned by `bsr 0x53eb`); the "kill" path sets `(dp,0xd158)` from `0x53eb` and
`(dp,0xd15a)=0xffff` at `0x5392/0x5396`. Readers: `0x5308-0x53e8`, plus the
cross-voice copy routine `0x3a9e` (`MOVG2 @r2+-2 r1; ADD r1 r1;
MOVG2 @r1+0xcdc6 r6; MOVG2 @r0+-2 r1; … MOVG3 r6 -> @r1+0xcdc6`, `0x3aa1-0x3aac`). **S**

---

## 4. Struct field table

Terminology: `P(v) = rom1[0x64d6+2*v]` (interior pointer), `S = P-0x80` (start of the
0x12a-byte struct). Sub-block = `[S, P)`, main block = `[P, P+0xaa)`.
Offsets are hexadecimal; `w`=16-bit, `b`=8-bit (from operand byte, §0).

### 4.1 Sub-block `S+0x00 .. S+0x7f` (= `P-0x80 .. P-1`)

The `0x38b0-0x39b1` interpolator runs with `r1 = P-0x80` (path `0x37f2`→`0x37f7`)
**or** `r1 = P-0x5e` (path `0x37fe`→`0x38aa`), i.e. it treats `S+0x00` and `S+0x22`
as two identical `0x22`-byte records (REC0/REC1). Evidence for record length: highest
REC field is `+32` word (`0x39b1 MOVG3 r4 -> @r1+32 [e9 20 94]`), ending at `+0x22`.

| Off (S) | P-rel | W | Hypothesis | Evidence (PC, bytes) |
|---|---|---|---|---|
| `+0x00` | `P-0x80` | w | REC0+0 signed value, set from tone `r5[72]` | `0x35af MOVG3 r2 -> @r0+-128 [e8 80 92]`; `R` `0x3b13 [e8 80 82]`, `0x3ba2 [e8 80 86]` |
| `+0x02` | `P-0x7e` | w | REC0+2 | `0x35e9 [e8 82 92]`; `R` `0x3b25 [e8 82 82]`, `0x3ba8` |
| `+0x04` | `P-0x7c` | w | REC0+4 output of `0x3790` clamp | `0x37f2 MOVG3 r2 -> @r0+-124 [e8 84 92]`; `0x3b99 [e8 84 92]` |
| `+0x06` | `P-0x7a` | w | REC0+6 cleared at note setup | `0x368a [e8 86 13]`; `0x3b00 [e8 86 13]`; `0x3b22 [e8 86 92]` |
| `+0x08` | `P-0x78` | w | REC0+8 = MULXU product | `0x368d [e8 88 13]`; `0x3b03 [e8 88 13]`; `0x3b34 [e8 88 92]` |
| `+0x0a` | `P-0x76` | w | REC0+0x0a = MULXU product | `0x3690 [e8 8a 13]`; `0x3b06 [e8 8a 13]`; `0x3b9e [e8 8a 92]` |
| `+0x0c` | `P-0x74` | w | REC0+0x0c: index into rom1 table `0x6d86` (`ADD r3 r3; MOVG2 @r3+0x6d86 r3`) | `0x3796 MOVG3 r3 -> @r0+-116 [e0 8c 93]`; use `0x3942/0x3947 [ab 23][fb 6d 86 83]` |
| `+0x0d` | `P-0x73` | b | gate/enable byte (high half of `+0x0c`) | `CLR @r0+-115 [e0 8d 13]` `0x3626`; `TST` `0x3709 [e0 8d 16]`; copy `0x3aef [e0 8d 96]` |
| `+0x0e` | `P-0x72` | w | REC0+0x0e signed addend | `0x3af5 MOVG3 r6 -> @r0+-114 [e8 8e 96]`; `R` `0x394b MOVG2 @r1+14 r4 [e9 0e 84]` |
| `+0x10` | `P-0x70` | w | REC0+0x10 (MULXU operand, `(dp,0xad2a)`) | `0x38b8 MOVG2 @r1+16 r4 [e9 10 84]`; write `0x36c3 [e8 90 93]`, copy `0x3ab9 [e8 90 96]` |
| `+0x12` | `P-0x6e` | w | REC0+0x12 | `0x38de MOVG2 @r1+18 r6 [e9 12 84]`; write `0x36d1 [e8 92 93]`, copy `0x3abf` |
| `+0x14` | `P-0x6c` | w | REC0+0x14 | write `0x367b MOVG3 r3 -> @r0+-108 [e8 94 93]`; copy `0x3ac5 [e8 94 96]` |
| `+0x16` | `P-0x6a` | w | REC0+0x16 from `(r6&0xc0)` | `0x366e MOVG3 r3 -> @r0+-106 [e8 96 93]`; copy `0x3acb [e8 96 96]` |
| `+0x18` | `P-0x68` | w | REC0+0x18 | `CLR @r0+-104 [e8 98 13]` `0x3681`; write `0x3ad1`, copy `0x3aef` chain |
| `+0x1a` | `P-0x66` | w | REC0+0x1a | `0x3b0a MOVG2 @r0+-102 r6 [e8 9a 86]`; `CLR` `0x3684 [e8 9a 13]` |
| `+0x1c` | `P-0x64` | w | REC0+0x1c | `0x36f4 MOVG3 r4 -> @r0+-100 [e8 9c 94]`; copy `0x3add` |
| `+0x1e` | `P-0x62` | w | REC0+0x1e | `CLR @r0+-98 [e8 9e 13]` `0x367e`; write `0x36f7 [e8 9e 94]`; copy `0x3ae3` |
| `+0x20` | `P-0x60` | w | REC0+0x20 result of interpolator (`r4`) | `0x39b1 MOVG3 r4 -> @r1+32 [e9 20 94]`; `0x3a50 [e9 20 94]`; tests `0x2e00 TST [e8 a0 16]` |
| `+0x22` | `P-0x5e` | w | REC1+0 (= `r1` of the `0x38aa` path); from tone `r5[73]` | `0x359a MOVG3 r2 -> @r0+-94 [e8 a2 92]`; `0x38ac ADD #0xffa2 r1 [0c ff a2 21]` |
| `+0x24` | `P-0x5c` | w | REC1+2 from rom1 table `0x6f86` | `0x35cc MOVG3 r2 -> @r0+-92 [e8 a4 92]` |
| `+0x26` | `P-0x5a` | w | REC1+4 from rom1 table `0x7086` | `0x3608 MOVG3 r2 -> @r0+-90 [e8 a6 92]` |
| `+0x28` | `P-0x58` | w | REC1+6 | `0x2812`; `0x2e05` (first PCs) |
| `+0x2a` | `P-0x56` | w | REC1+8 | `0x2815`; `0x4442` |
| `+0x2c` | `P-0x54` | w | REC1+0x0a | `0x2818`; `0x4e9e` |
| `+0x2e` | `P-0x52` | w | REC1+0x0c (table index) | `0x2842` |
| `+0x2f` | `P-0x51` | b | REC1+0x0d gate byte | `TST @r0+-81 [e0 af 16]` `0x37fe`; `CLR [e0 af 13]` `0x274c` |
| `+0x32` | `P-0x4e` | w | REC1+0x10 | `TST @r0+-78 [e8 b2 16]` `0x2fda`; `0x282c` |
| `+0x34` | `P-0x4c` | w | REC1+0x12 | `0x283a` |
| `+0x36` | `P-0x4a` | w | REC1+0x14 table 0x7207 value | `0x27ff/0x2803 MOVG3 r3 -> @r0+-74 [e8 b6 93]` |
| `+0x38` | `P-0x48` | w | REC1+0x16 | `0x27f6 [e8 b8 93]` |
| `+0x3a` | `P-0x46` | w | REC1 tail | `0x2809` (agg.) |
| `+0x3c` | `P-0x44` | w | REC1 tail | `0x280c` (agg.) |
| `+0x3e` | `P-0x42` | w | REC1 tail | `0x285f` (agg.) |
| `+0x40` | `P-0x40` | w | REC1 tail | `0x2806`, `0x2862` |
| `+0x42` | `P-0x3e` | w | REC1 tail (`+0x20` of REC1) | `0x280f`, `0x2e0c` |
| `+0x44` | `P-0x3c` | w | first word after REC1 | `0x46ea`, `0x49e3` |
| `+0x45` | `P-0x3b` | b | **flag, bit7 = steal/cleanup pending** | `BTSTI @r0+-59 #7 [e0 c5 f7]` `0x5321`, `0x537a`, `0x53ae`; `MOVG3 r6 -> @r0+-59 [e0 c5 96]` `0x542a` |
| `+0x46` | `P-0x3a` | w | sub-block scalar | `0x4cf8` (agg.) |
| `+0x48` | `P-0x38` | w | sub-block scalar | `0x485e`, `0x48aa` |
| `+0x4a` | `P-0x36` | w | sub-block scalar | `0x4aa3`, `0x4bbe` |
| `+0x4c` | `P-0x34` | w | sub-block scalar | `0x4b0e`, `0x4c9b` |
| `+0x4e` | `P-0x32` | w | sub-block scalar | `0x4b65`, `0x4bd1` |
| `+0x50` | `P-0x30` | w | sub-block scalar | `0x420f`, `0x42d6`, `0x4531` |
| `+0x52` | `P-0x2e` | w | sub-block scalar (decimal `-46` twin of `P+0x2e`; distinct field) | `0x3c86 MOVG3 r4 -> @r0+-46 [e8 d2 94]`; `0x3c89 MOVG2 [e8 d2 82]` |
| `+0x54` | `P-0x2c` | w | | `0x402d`, `0x42a1` |
| `+0x56` | `P-0x2a` | w | | `0x4095`, `0x4360` |
| `+0x58` | `P-0x28` | w | | `0x40f1`, `0x425e` |
| `+0x5a` | `P-0x26` | w | | `0x4146`, `0x42b4` |
| `+0x5c` | `P-0x24` | w | | `0x3c97`, `0x3d32` |
| `+0x5e` | `P-0x22` | w | | `0x2a8a`, `0x31f4` |
| `+0x60` | `P-0x20` | w | | `0x2aee`, `0x3329` |
| `+0x62` | `P-0x1e` | w | | `0x2b4a`, `0x3207` |
| `+0x64` | `P-0x1c` | w | | `0x2b9f`, `0x325d` |
| `+0x66` | `P-0x1a` | w | cleared at init and key-on | `CLR @r0+-26 [e0 e6 13]` `0x2e89`, `0x58e2`; `MOVG #0xff -> @r0+-26 [e0 e6 07 00 ff]` `0x2e89`, `0x41310`; test `0x5897 [e0 e6 16]` |
| `+0x67` | `P-0x19` | b | PCM param | `MOVG3 r3 -> @r0+-25 [e0 e7 93]` `0x36d7`; `MOVG2 @r0+-25 r3 [e0 e7 83]` `0x3780`; copy `0x3ab3 [e0 e7 96]` |
| `+0x68` | `P-0x18` | w | PCM reg `br+1a` (pitch?) | `MOVG2 @r0+-24 r6 [e8 e8 86]` `0x55c3`; `CLR` `0x3bcb [e8 e8 13]` |
| `+0x6a` | `P-0x16` | w | PCM reg `br+36` | `MOVG2 @r0+-22 r5 [e8 ea 85]` `0x563d`; `CLR` `0x3bce [e8 ea 13]` |
| `+0x6c` | `P-0x14` | b | PCM reg `br+05` | `MOVG2 @r0+-20 r5 [e0 ec 85]` `0x5553` |
| `+0x6d` | `P-0x13` | b | PCM reg `br+09` | `MOVG2 @r0+-19 r5 [e0 ed 85]` `0x5583` |
| `+0x6e` | `P-0x12` | b | PCM reg `br+0d` | `MOVG2 @r0+-18 r5 [e0 ee 85]` `0x556b` |
| `+0x6f` | `P-0x11` | b | voice-busy / event flag | `TST @r0+-17 [e0 ef 16]` `0x2600`; `0x28d8` |
| `+0x70` | `P-0x10` | w | PCM reg `br+06` | `MOVG2 @r0+-16 r6 [e8 f0 86]` `0x5556` |
| `+0x72` | `P-0x0e` | w | PCM reg `br+0a` | `MOVG2 @r0+-14 r6 [e8 f2 86]` `0x5586` |
| `+0x74` | `P-0x0c` | w | PCM reg `br+0e` | `MOVG2 @r0+-12 r6 [e8 f4 86]` `0x556e` |
| `+0x76` | `P-0x0a` | w | PCM reg `br+1e` | `MOVG2 @r0+-10 r6 [e8 f6 86]` `0x5540`; `0x28d2` |
| `+0x78` | `P-0x08` | b | LFO/algorithm selector | `MOVG2 @r0+-8 r5 [e0 f8 85]`/`MOVG3 r5 -> @r0+-8 [e0 f8 95]` `0x2f83/0x2f86`; route `0x3142 [e0 f8 94]` |
| `+0x79` | `P-0x07` | b | LFO selector B | `MOVG2 @r0+-7 r4 [e0 f9 84]` `0x3123` |
| `+0x7a` | `P-0x06` | b | LFO selector C | `MOVG2 @r0+-6 r4 [e0 fa 84]` `0x312e`; `0x2bdf` |
| `+0x7b` | `P-0x05` | b | LFO selector D | `MOVG2 @r0+-5 r4 [e0 fb 84]` `0x3139`; `0x2bf1` |
| `+0x7c` | `P-0x04` | b | waveform/flag | `MOVG2 @r0+-4 r5 [e0 fc 85]` `0x2f83`; `0x2c0f` |
| `+0x7d` | `P-0x03` | b | 0/2 selector | `MOVG #0x02 -> @r0+-3 [e0 fd 06 02]` `0x2fc3`; `MOVG #0x00 -> @r0+-3 [e0 fd 06 00]` `0x2fc9` |
| `+0x7e` | `P-0x02` | w | **slot index v (0..27)** | write `0x53f3 MOVG3 r1 -> @r0+-2 [e8 fe 91]`; reads `0x361d`, `0x3aa7`, `0x531e`, `0x546e`, `0x5540`, `0x5626` |

**Offset notation.** `dasm_full.txt` prints disp8 offsets in **decimal** and disp16
offsets in **hex**:
`@r0+-46` is `P-46` (decimal) = `S+0x52`, a different field from `@r0+46`
(= `P+0x2e`, the note/part pointer used by `0x377a`/`0x369a`). The directly
`P`-negative accesses at `-60…-36` and `-25…-10` address the tail of the sub-block;
the REC0/REC1 partition is **S** (Strongly supported), not **C**.

### 4.2 Main block `P+0x00 .. P+0xa9`

| Off (P) | W | Hypothesis | Evidence (PC, bytes) |
|---|---|---|---|
| `+0x00` | w | **state/phase word** — init `0x16`; set `0x0e/0x10/0x12/0x0c/0x02`; compared `0x0e/0x12` | `0x41248 MOVG #0x0016 -> @r1+0 [e9 00 07 00 16]`; `0x262f MOVG3 r6 -> @r0+0 [e8 00 96]` with `r6=0x0e` (`0x2613`) or `0x10` (`0x2622`); `0x2f5d MOVG #0x000c -> @r0+0 [e8 00 07 00 0c]`; `0x55ed MOVG3 r4 -> @r0+0 [e8 00 94]`; `0x25f8 SUB @r0+0 #0x000e [e8 00 05 00 0e]`; `0x5891 SUB @r0+0 #0x12 [e8 00 04 12]` |
| `+0x02` | w | state word | `0x4124d [e9 02 07 00 16]`; `0x2632 MOVG3 r6 -> @r0+2 [e8 02 96]`; `0x2f62 [e8 02 07 00 0c]`; `0x55f0 [e8 02 94]` |
| `+0x04` | w | state word | `0x41252 [e9 04 07 00 16]`; `0x2635 [e8 04 96]`; `0x2f67 [e8 04 07 00 0c]`; `0x55f3 [e8 04 94]` |
| `+0x06` | w | timer/param | `0x2925`, `0x3478` (agg.) |
| `+0x08` | w | note step / dpcm flag cleared on key-on | `0x2f77 CLR @r0+8 [e8 08 13]`; `0x30f8 SUB @r0+8 #0xffff [e8 08 05 ff ff]`; `0x310d CLR` |
| `+0x0a` | w | | `0x2f89 CLR @r0+10 [e8 0a 13]` |
| `+0x0c` | w | | `0x2fd4 CLR @r0+12 [e8 0c 13]`; `0x4ce9` (agg.) |
| `+0x0e` | w | | `0x2c72` (agg.); `0x55d2 TST @r0+14 [e8 0e 16]` |
| `+0x10` | w | | `0x2c77` (agg.) |
| `+0x12` | w | | `0x2f7a CLR @r0+18 [e8 12 13]`; `0x31a0 CLR` |
| `+0x14` | w | | `0x2f8c CLR @r0+20 [e8 14 13]`; `0x41c4` (agg.) |
| `+0x16` | w | | `0x2fd7 CLR @r0+22 [e8 16 13]`; `0x41d1` (agg.) |
| `+0x18` | w | | `0x2c98` (agg.); `0x2d7f MOVG3 r5 -> @r0+24 [e8 18 95]` |
| `+0x1a` | w | written `0xba` after `bsr 0x2e82` | `0x2d84 MOVG3 r5 -> @r0+26 [e8 1a 95]`; `0x261b MOVG #0x00b5 -> @r0+26 [e8 1a 07 00 b5]`; `0x5197 [ea 1a 07 00 b5]` |
| `+0x1c` | w | `(dp,0xad2a)`-scaled pulse width | `0x2ed7 MOVG3 r5 -> @r0+28 [e8 1c 95]`; `0x31bf MOVG3 r6 -> @r0+28 [e8 1c 96]` |
| `+0x1e` | w | | `0x262a MOVG #0x00b5 -> @r0+30 [e8 1e 07 00 b5]`; `0x31c6 [e8 1e 96]` |
| `+0x20` | w | | `0x2f95 MOVG2 @r0+32 r5 [e8 20 85]`; `0x4215` (agg.) |
| `+0x22` | w | | `0x4419` (agg.) |
| `+0x24` | w | loop start | `0x2f08 MOVG3 r5 -> @r0+36 [e8 24 95]`; `0x3bd4 CLR [e8 24 13]` |
| `+0x26` | w | loop end | `0x2ef6 SUB @r0+38 #0xff00 [e8 26 05 ff 00]`; `0x3bd1 CLR [e8 26 13]`; `0x4563 [e8 26 07 ff 00]` |
| `+0x28` | b | DPCM pointer low byte | `0x45d0 MOVG2 @r0+40 r6 [e0 28 86]`; `0x4606 MOVG3 r4 -> @r0+40 [e0 28 94]` |
| `+0x29` | b | | `0x266f MOVG3 r4 -> @r0+41 [e0 29 94]` |
| `+0x2a` | b | | `0x2669 MOVG2 @r0+42 r4 [e0 2a 84]`; `0x4651 MOVG3 r2 -> @r0+42 [e0 2a 92]` |
| `+0x2b` | b | | `0x4744` (agg.) |
| `+0x2c` | b | | `0x2fa1 MOVG2 @r0+44 r4 [e0 2c 84]`; `0x2fb9 CMP @r0+44 r4 [e0 2c 74]` |
| `+0x2d` | b | | `0x2675 MOVG2 @r0+45 r2 [e0 2d 82]` |
| `+0x2e` | w | **pointer to a 0x70-byte record** (`0x8048+n*0x70`; from `rom1[0x7218+2*ce78[slot]]`) | `0x5442 MOVG2 @r3+0x7218 r3; 0x5446 MOVG3 r3 -> @r0+46 [e8 2e 93]`; use `0x377a MOVG2 @r0+46 r3 [e8 2e 83]`, `0x2d9f [e8 2e 83]` |
| `+0x30` | w | `0x8748 + cecc[slot]` | `0x545a movi r3 #0x8748 [5b 87 48]`; `0x5469 MOVG3 r3 -> @r0+48 [e8 30 93]`; `0x2cc9` (agg.) |
| `+0x34` | w | | `0x2d77 MOVG3 r5 -> @r0+52 [e8 34 95]`; `0x3532` (agg.) |
| `+0x36` | w | | `0x2d46 MOVG3 r3 -> @r0+54 [e8 36 93]` |
| `+0x38` | w | | `0x2c54` (agg.); `0x2d17 MOVG2 @r0+56 r3 [e8 38 83]` |
| `+0x3a` | w | detune/ratio (compared + swapped) | `0x2cf2` (agg.); `0x355b MOVG2 @r0+58 r2 [e8 3a 82]`; `0x357c MOVG3 r2 -> @r0+58 [e8 3a 92]` |
| `+0x3c` | w | | `0x45d7 MOVG2 @r0+60 r6 [e8 3c 86]`; `0x4609 MOVG3 r5 -> @r0+60 [e8 3c 95]` |
| `+0x3e` | w | | `0x2672 MOVG3 r5 -> @r0+62 [e8 3e 95]` |
| `+0x40` | w | | `0x266c MOVG2 @r0+64 r5 [e8 40 85]` |
| `+0x42` | w | | `0x4747` (agg.) |
| `+0x44` | w | | `0x2fa4 MOVG2 @r0+68 r5 [e8 44 85]`; `0x2fbe CMP @r0+68 r5 [e8 44 75]` |
| `+0x46` | w | | `0x2cc6` (agg.); `0x2678 MOVG2 @r0+70 r3 [e8 46 83]` |
| `+0x48` | w | cutoff/pan param | `0x50eb` (agg.); `0x55dd MOVG2 @r0+72 r6 [e8 48 86]` |
| `+0x4a` | b | | `0x2f92 MOVG2 @r0+74 r5 [e0 4a 85]` |
| `+0x4b` | b | | `0x415a` (agg.) |
| `+0x4c` | b | | `0x4164` (agg.) |
| `+0x4d` | b | | `0x416e` (agg.) |
| `+0x4e` | b | | `0x2f8f MOVG2 @r0+78 r5 [e0 4e 85]` |
| `+0x4f` | b | | `0x2bb7` (agg.); `0x2f80 MOVG3 r5 -> @r0+79 [e0 4f 95]` |
| `+0x50` | b | | `0x2bcf` (agg.); `0x3120 MOVG2 @r0+80 r3 [e0 50 83]` |
| `+0x51` | b | | `0x2be7` (agg.); `0x312b [e0 51 83]` |
| `+0x52` | b | | `0x2bff` (agg.); `0x3136 [e0 52 83]` |
| `+0x53` | b | | `0x2c17` (agg.); `0x2f7d MOVG2 @r0+83 r5 [e0 53 85]` |
| `+0x54` | w | filter/pan coefficient | `0x2f98 MOVG3 r5 -> @r0+84 [e8 54 95]`; `0x3c57 MOVG3 r3 -> @r0+84 [e8 54 93]` |
| `+0x56` | w | | `0x2f9e MOVG3 r5 -> @r0+86 [e8 56 95]` |
| `+0x58` | w | | `0x3d90` (agg.) |
| `+0x5a` | w | | `0x3e0f` (agg.) |
| `+0x5c` | w | | `0x3e8e` (agg.) |
| `+0x5e` | w | | `0x2f9b MOVG2 @r0+94 r5 [e8 5e 85]` |
| `+0x60` | b | | `0x2a2c` (agg.); `0x2f71 MOVG3 r5 -> @r0+96 [e0 60 95]` |
| `+0x61` | b | | `0x29ed` (agg.); `0x2f74 CLR @r0+97 [e0 61 13]` |
| `+0x62` | b | | `0x2a01` (agg.); `0x3126 MOVG2 @r0+98 r6 [e0 62 86]` |
| `+0x63` | b | | `0x2a15` (agg.); `0x3131 [e0 63 86]` |
| `+0x64` | b | | `0x2a29` (agg.); `0x313c MOVG2 @r0+100 r6 [e0 64 86]` |
| `+0x65` | b | flag (bit test / `0xff`) | `0x3bc6 MOVG #0x00ff -> @r0+101 [e0 65 07 00 ff]`; `0x5629 TST [e0 65 16]` |
| `+0x66` | b | mode/type (set 3 on key-on) | `0x3bdb MOVG #0x03 -> @r0+102 [e0 66 06 03]`; `0x55be [e0 66 86]` |
| `+0x67` | b | copied from tone `r5[38]` | `0x3bf0 MOVG3 r4 -> @r0+103 [e0 67 94]` |
| `+0x68` | b | envelope step (init 0x40) | `0x3bd7 MOVG #0x40 -> @r0+104 [e0 68 06 40]`; `0x55b9 [e0 68 86]` |
| `+0x69` | b | | `0x3f99` (agg.) |
| `+0x6a` | b | | `0x2fa7 MOVG3 r4 -> @r0+106 [e0 6a 94]` |
| `+0x6b` | b | | `0x2fb3 MOVG3 r4 -> @r0+107 [e0 6b 94]` |
| `+0x6c` | b | | `0x47fa` (agg.) |
| `+0x6d` | b | | `0x4802` (agg.) |
| `+0x6e` | b | | `0x472b` (agg.) |
| `+0x6f` | b | | `0x2fad MOVG2 @r0+111 r4 [e0 6f 84]` |
| `+0x70` | w | | `0x2faa MOVG3 r5 -> @r0+112 [e8 70 95]` |
| `+0x72` | w | | `0x2fb6 MOVG3 r5 -> @r0+114 [e8 72 95]` |
| `+0x74` | w | | `0x493f` (agg.) |
| `+0x76` | w | | `0x498b` (agg.) |
| `+0x78` | w | | `0x472e` (agg.) |
| `+0x7a` | w | | `0x2fb0 MOVG2 @r0+122 r5 [e8 7a 85]` |
| `+0x7c` | w | | `0x2fd1 MOVG3 r5 -> @r0+124 [e8 7c 95]` |
| `+0x7e` | w | | `0x4bf7` (agg.) |
| `+0x80` | w | mirror coefficient | `0x4c40`, `0x4d8a` (disp16 agg.) |
| `+0x82` | w | mirror coefficient | `0x4c8a`, `0x4d96` (disp16 agg.) |
| `+0x84` | w | | `0x2fcd MOVG2 @r0+0x0084 r5 [f8 00 84 85]` |
| `+0x86` | w | filter coefficient (computed at `0x5998`) | `0x5d72 MOVG3 r6 -> @r0+0x0086 [f8 00 86 96]`; `0x5a15 MOVG3 r4 -> @r0+0x0086 [f8 00 86 94]` |
| `+0x88` | w | coefficient copied by `0x5d6e` | `0x5d7e MOVG2 @r2+0x0088` / `0x5d82 MOVG3 r6 -> @r0+0x0088`; `0x441c` |
| `+0x8a` | w | coefficient computed at `0x5998` | `0x5d76 [f8]`, `0x5d7a MOVG3 r6 -> @r0+0x008a [f8 00 8a 96]`; `0x5a7b [f8 00 8a 94]` |
| `+0x8c` | w | coefficient copied by `0x5d6e` | `0x5dae [f8]`; `0x4438` |
| `+0x8e` | w | coefficient copied by `0x5d6e` | `0x5d9e [f8]`; `0x2dfc` |
| `+0x90` | w | gain from `MULXU #0xbe7a` | `0x5d69 MOVG3 r4 -> @r0+0x0090 [f8 00 90 94]` |
| `+0x92` | w | coefficient copied by `0x5d6e` | `0x5db6 [f8]`; `0x4ea1` |
| `+0x94` | w | coefficient copied by `0x5d6e` | `0x5da6 [f8]`; `0x4445` |
| `+0x96` | w | coefficient copied by `0x5d6e` | `0x5d96 [f8]`; `0x2e08` |
| `+0x98` | b | **EP page for the `+0x9c` pointer** (loaded via `LDC … r4` = ep) | `0x3615 LDC @r0+0x0098 r4 [f0 00 98 8c]`; store from `cfe4[slot]` `0x53fa MOVG3 r6 -> @r0+0x0098 [f0 00 98 96]` |
| `+0x99` | b | **EP page for the `+0x9e` pointer** | `0x3580 LDC @r0+0x0099 r4 [f0 00 99 8c]`; store from `d038[slot]` `0x5402 MOVG3 r6 -> @r0+0x0099 [f0 00 99 96]`; `0x2734` |
| `+0x9a` | b | **EP page for the `+0xa0` pointer** | `0x460c LDC @r0+0x009a r4 [f0 00 9a 8c]`; store from `d08c[slot]` `0x540a MOVG3 r6 -> @r0+0x009a [f0 00 9a 96]` |
| `+0x9b` | b | **tone index** → `rom1[0x7218+2*idx]` stored at `+0x2e` | `0x5438 MOVG2 @r1+0xce78 r3; 0x543c MOVG3 r3 -> @r0+0x009b [f0 00 9b 93]` |
| `+0x9c` | w | **pointer (word) into page `+0x98`** (sample/loop table) | read `0x3619 MOVG2 @r0+0x009c r5 [f8 00 9c 85]`; store from `cfac[2*slot]` `0x5412 MOVG3 r6 -> @r0+0x009c [f8 00 9c 96]`; use `0x29c5` |
| `+0x9e` | w | **pointer (word) into page `+0x99`** (tone data: `r5[8],[14],[15],[42],[43],[61],[72],[73]` used) | read `0x3584 MOVG2 @r0+0x009e r5 [f8 00 9e 85]`; store from `d000[2*slot]` `0x541a [f8 00 9e 96]` |
| `+0xa0` | w | **pointer (word) into page `+0x9a`** (DPCM table) | read `0x4610 MOVG2 @r0+0x00a0 r5 [f8 00 a0 85]`; store from `d054[2*slot]` `0x5422 [f8 00 a0 96]` |
| `+0xa2` | b | tone byte (`r5[8]`) | `0x273f MOVG3 r6 -> @r0+0x00a2 [f0 00 a2 96]`; `0x4220` |
| `+0xa3` | b | operator/sample type | `0x3bf9 MOVG3 r2 -> @r0+0x00a3 [f0 00 a3 92]`; `0x3f3d`, `0x43e5` |
| `+0xa4` | b | nonzero ⇒ fractional running accumulator enabled | `0x2681 CLR @r0+0x00a4 [f0 00 a4 13]`; `0x4cec`, `0x500b` |
| `+0xa6` | w | **fractional accumulator** (added to integer count) | `0x2685 MOVG2 @r0+0x00a6 r1 [f8 00 a6 81]`; `0x50ce`/`0x50d2` |
| `+0xa8` | w | signed note pitch/offset from tone byte (`r5[14]`) | store `0x3610 MOVG3 r2 -> @r0+0x00a8 [f8 00 a8 92]`; read `0x37a1 MOVG2 @r0+0x00a8 r2 [f8 00 a8 82]`, `0x3b3f` |

Confidence: `-2` (slot backref), `+0x00..+0x04` (state), `+0x2e`/`+0x30` (tone pointer/
value), `+0x98..+0xa8` (page+pointer triples and pitch offset) are **C** (named
init/copy routines and PCM register writes). The bulk `+0x06..+0x96` is **S** (many
independent sites consistent with a per-voice DSP state). Exact semantic names for
individual counters are **I**.

---

## 5. Pointer construction / pool (§4 of the request)

1. **Static slot→pointer table.** `rom1[0x64d6 + 2*v]` (§2). No runtime arithmetic
   stride (`×0x12a`) was found anywhere in the executed set; `0x64d6` is the only
   slot→pointer mapping.
2. **All readers of `0x64d6`** (10 instruction sites): `0x25f4`, `0x5186`, `0x5240`,
   `0x53ef`, `0x588d`, `0x58bc`, `0x58ce`, `0x58de`, `0x41240`, `0x4130c`.
3. **Back-reference.** `P-2 = slot` written only at `0x53f3`; read at `0x361d`,
   `0x3aa7`, `0x3bf3`, `0x531e`, `0x546e`, `0x5540`, `0x5626`, `0x2d7a`, `0x2f14`,
   `0x30f2` (all `[e8 fe 81]`).
4. **Struct field fill (SoA → AoS).** `0x53eb-0x546c` is the "materialize voice"
   routine called with `r1 = slot`; it resolves `P` and copies:
   `cfe4[slot]→P+0x98`, `d038[slot]→P+0x99`, `d08c[slot]→P+0x9a`,
   `cfac[2*slot]→P+0x9c`, `d000[2*slot]→P+0x9e`, `d054[2*slot]→P+0xa0`,
   `cf90[slot]→P-59`, `ce78[slot]→P+0x9b`, `rom1[0x7218+2*ce78[slot]]→P+0x2e`, and
   `P+0x30 = 0x8748 + (d0fc[slot]<0 ? cecc[slot] : 0)`.
   PC map: `0x53f6,0x53fa,0x53fe,0x5402,0x5406,0x540a,0x540e,0x5412,0x5416,0x541a,
   0x541e,0x5422,0x5426,0x542a,0x5438,0x543c,0x5442,0x5446,0x544b-0x5469`.
5. **Deferred writers of `+0`/`+0x1a`/`+0x1e`.** `0x516c-0x51b5` (called with
   `r1=slot`, `r2=P` from `0x5186`) sets `@r2+26/+30 = 0xb5` and `@r2+0/2/4 = 0x12/0x14`;
   `0x25f0-0x2635` (r0=P) sets `@r0+0/2/4 = 0x0e/0x10` and `@r0+26/+30 = 0xb5`.
6. **Free-slot pool (indices, not pointers).** `0x40508-0x40536` builds the
   `a3d8[]` "next" chain and the head `(dp,0xa430)` over slots 27..0
   (`0x4050d MOVG2 (dp,0xa430) r0 [15 a4 30 80]`, `0x40517 MOVG3 r0 -> @r1+0xa3d8`,
   `0x40526 MOVG3 r1 -> (dp,0xa430)`); `(dp,0xa42d)` counts (`ADDQ #1 (dp,0xa42d)`
   `0x4052f`), `(dp,0xa42e)`/`(dp,0xa42f)` are chain ends. Allocation pops the chain at
   `0x184a-0x187d` (`MOVG2 (dp,0xa430) r0` `0x184a`, `MOVG3 r1 -> (dp,0xa430)` `0x1863`),
   free pushes back at `0x19c4-0x1a20`. **These operate purely on slot numbers**;
   the `P` pointer is only obtained later via `0x64d6` (e.g. `0x53eb`).
7. **Live-pointer globals.** `(dp,0xd158)`/`(dp,0xd15a)` = two `P` values used by the
   PCM-event path (`0x52f7`, `0x5300`, `0x5392/0x5396`; readers `0x5308-0x53e8`,
   `0x5d6e` copies `+0x86/+0x88/+0x8a` between them, `0x3a9e` swaps the `0xcdc6`
   word entry between their slots).
8. **Voice-mask bit set by slot.** `0x546e-0x5484`: `r1 = @r0-2` (slot); if `r1<=15`
   `BSET (dp,0xd156) r1 [1d d1 56 49]`, else `r1-=16; BSET (dp,0xd154) r1
   [1d d1 54 49]` (then `0x51f6`).

---

## 6. Classification of the 49 listed bases

**Result: all 49 are category (b) — standalone SoA arrays** indexed by a small 8-bit
index (voice slot 0..27, note/channel/sequence index), addressed as
`@rN + <literal base>` where the literal is a 16-bit displacement and `rN` is the
index. None is a field of the AoS struct: the AoS region is `[0xad2e,0xcdc6)` and all
49 addresses fall outside it; conversely every AoS field is accessed with a *small*
displacement against `P` from `0x64d6`, never with these literals. Also, all 49 are
`≥0x8000`, i.e. page-0 SRAM (`src/mcu.cpp:711-713`), so none is a ROM table. The only
ROM pointer table found in this analysis is `0x64d6` itself (category (c), not in the
list). **C**

Width legend: byte/word decided by operand byte bit3 (`e0/f0`=b, `e8/f8`=w). "Index"
lists the register that carries the element number; evidence is one representative PC
(raw operand bytes in brackets) plus the `ADD rX rX`/`SHLL` that scales word arrays.

| Base | W | Index | Evidence (PC, bytes) | Verdict |
|---|---|---|---|---|
| `0xa34c` | b | slot / sequence `r3=(dp,0xa4a9)` | `0x1087 MOVG3 r1 -> @r3+0xa34c [f3 a3 4c 91]`; `0x19e7 BCLR @r2+0xa34c #1 [f2 a3 4c d1]`; init `0x404aa [f1 a3 4c 92]` | (b) |
| `0xa368` | b | slot | `0x19a4 MOVG3 r3 -> @r1+0xa368 [f1 a3 68 93]`; search `0x45cc2 MOVG2 @r0+0xa368 r6 [f0 a3 68 86]` | (b) |
| `0xa384` | b | slot | `0x1748 MOVG2 @r1+0xa384 r2 [f1 a3 84 82]`; `0x19a8 store [f1 a3 84 92]` | (b) |
| `0xa3a0` | b | slot | `0x1867 MOVG #0x94 -> @r1+0xa3a0 [f1 a3 a0 06 94]`; `0x19c4 BTSTI @r1+0xa3a0 #7 [f1 a3 a0 f7]` | (b) |
| `0xa3bc` | b | slot | `0x148c MOVG #0x01 -> @r1+0xa3bc [f1 a3 bc 06 01]`; `0x183d CLR [f1 a3 bc 13]` | (b) |
| `0xa3d8` | b | slot | `0x1854 MOVG3 r0 -> @r1+0xa3d8 [f1 a3 d8 90]`; `0x18af [f1 a3 d8 90]` | (b) |
| `0xa3f4` | b | slot | `0x1b48 MOVG2 @r1+0xa3f4 r3 [f1 a3 f4 83]`; `0x197c [f0 a3 f4 91]` | (b) |
| `0xa410` | b | slot | `0x1496 MOVG2 @r1+0xa410 r1 [f1 a4 10 81]`; `0x196c [f1 a4 10 06 ff]` | (b) |
| `0xa46c` | w | slot (×2) | `0xfc4 SHLL r1 [a9 1a]`, `0xfc6 MOVG3 r6 -> @r1+0xa46c [f9 a4 6c 96]`; `0x404bc [f9 a4 6c 92]` | (b) |
| `0xa4aa` | b | sequence | `0x1928 MOVG #0xff -> @r0+0xa4aa [f0 a4 aa 06 ff]`; read `0x1469 MOVG2 (dp,0xa4aa) r1 [15 a4 aa 81]` | (b) |
| `0xa4b4` | b | slot | `0x1491 MOVG #0xff -> @r1+0xa4b4 [f1 a4 b4 06 ff]`; `0x1ade OR @r1+0xacf2` pair | (b) |
| `0xacf2` | b | slot | `0x1ade OR @r1+0xacf2 r0 [f1 ac f2 40]`; `0x1ae2 MOVG3 r0 -> @r1+0xacf2 [f1 ac f2 90]` | (b) |
| `0xad0e` | b | slot | `0x17c0 CMP @r1+0xad0e r0 [f1 ad 0e 70]`; `0x1839 CLR [f1 ad 0e 13]` | (b) |
| `0xce3f` | b | slot | `0x4049e MOVG3 r2 -> @r1+0xce3f [f1 ce 3f 92]`; `0x5536 CLR @r3+0xce3f [f3 ce 3f 13]` | (b) |
| `0xce5c` | b | slot | `0x40469 MOVG3 r3 -> @r1+0xce5c [f1 ce 5c 93]` | (b) |
| `0xce78` | b | slot | `0xfb8 MOVG3 r0 -> @r1+0xce78 [f1 ce 78 90]`; used as index at `0x5438` | (b) |
| `0xce94` | b | slot | `0xfd0 MOVG3 r0 -> @r1+0xce94 [f1 ce 94 90]`; `0x40471 [f1 ce 94 93]` | (b) |
| `0xceb0` | b | slot | `0xffa MOVG3 r0 -> @r1+0xceb0 [f1 ce b0 90]`; read `0x480f [f1 ce b0 82]` | (b) |
| `0xcecc` | b | slot | `0x100a MOVG3 r0 -> @r1+0xcecc [f1 ce cc 90]`; read `0x5461 [f1 ce cc 86]` | (b) |
| `0xcee8` | b | slot | `0x29a2 MOVG2 @r1+0xcee8 r3 [f1 ce e8 83]`; `0x2afb SUB [f1 ce e8 32]` | (b) |
| `0xcf04` | b | slot | `0x3c5d SUB @r1+0xcf04 r2 [f1 cf 04 32]`; `0x40a2` | (b) |
| `0xcf20` | b | slot | `0x1102 MOVG3 r0 -> @r1+0xcf20 [f1 cf 20 90]`; `0x40481 [f1 cf 20 93]` | (b) |
| `0xcf3c` | b | slot | `0x40485 MOVG #0x3c -> @r1+0xcf3c [f1 cf 3c 06 3c]`; read `0x45e0 MOVG2 @r1+0xcf3c r4 [f1 cf 3c 84]` | (b) |
| `0xcf58` | w | slot (×2) | `0x10e2 SHLL r1`, `0x10e8 MOVG3 r0 -> @r1+0xcf58 [f9 cf 58 90]`; `0x45ec ADD @r2+0xcf58 r5 [fa cf 58 25]` | (b) |
| `0xcf90` | b | slot | `0xff2 MOVG3 r0 -> @r1+0xcf90 [f1 cf 90 90]`; `0x5426 read` | (b) |
| `0xcfac` | w | slot (×2) | `0xfa4 MOVG3 r0 -> @r1+0xcfac [f9 cf ac 90]`; `0x540e MOVG2 @r3+0xcfac r6 [fb cf ac 86]` | (b) |
| `0xcfe4` | b | slot | `0x1012 MOVG3 r0 -> @r1+0xcfe4 [f1 cf e4 90]`; `0x53f6 read` | (b) |
| `0xd000` | w | slot (×2) | `0xfac MOVG3 r0 -> @r1+0xd000 [f9 d0 00 90]`; `0x5416 [fb d0 00 86]` | (b) |
| `0xd038` | b | slot | `0x101a MOVG3 r0 -> @r1+0xd038 [f1 d0 38 90]`; `0x53fe read` | (b) |
| `0xd054` | w | slot (×2) | `0xf9c MOVG3 r5 -> @r1+0xd054 [f9 d0 54 95]`; `0x541e [fb d0 54 86]` | (b) |
| `0xd08c` | b | slot | `0x1022 MOVG3 r0 -> @r1+0xd08c [f1 d0 8c 90]`; `0x5406 read` | (b) |
| `0xd0a8` | b | chain index | `0x2638 SUB @r1+0xd0a8 #0xff [f1 d0 a8 04 ff]`; `0x58a6` | (b) |
| `0xd0c4` | b | chain index | `0x263f SUB @r1+0xd0c4 #0xff [f1 d0 c4 04 ff]`; `0x58ad` | (b) |
| `0xd0e0` | b | slot | `0x1026 MOVG #0x02 -> @r1+0xd0e0 [f1 d0 e0 06 02]`; `0x521d MOVG2 @r1+0xd0e0 r2 [f1 d0 e0 82]` | (b) |
| `0xd0fc` | b | slot | `0x1002 MOVG3 r0 -> @r1+0xd0fc [f1 d0 fc 90]`; `0x544b TST [f1 d0 fc 16]` | (b) |
| `0xd118` | b | slot | `0x40475 MOVG3 r3 -> @r1+0xd118 [f1 d1 18 93]` | (b) |
| `0xd134` | b | slot | `0xfe0 MOVG3 r0 -> @r1+0xd134 [f1 d1 34 90]`; read `0x59b0 [f1 d1 34 83]` | (b) |
| `0xd15c` | b | slot | `0x516c CLR @r1+0xd15c [f1 d1 5c 13]`; scan `0x51e0 TST [f1 d1 5c 16]` | (b) |
| `0xcdc6` | w | slot (×2) | `0x3620 ADD r1 r1`, `0x3622 CLR @r1+0xcdc6 [f9 cd c6 13]`; `0x3aa3 MOVG2 [f9 cd c6 86]` | (b) |
| `0xcdfe` | w | slot (×2) | `0x2746 ADD r1 r1`, `0x2748 CLR @r1+0xcdfe [f9 cd fe 13]` | (b) |
| `0xa250` | b | note/channel (linked-list next) | `0x13c3 MOVG2 @r2+0xa250 r2 [f2 a2 50 82]`; `0x18f0 MOVG3 r2 -> @r0+0xa250 [f0 a2 50 92]` | (b) |
| `0xa26c` | b | note/channel (prev) | `0x1b27 MOVG2 @r2+0xa26c r1 [f2 a2 6c 81]`; `0x18f8 store [f2 a2 6c 90]` | (b) |
| `0xa288` | b | note/channel | `0x13ab SUB @r2+0xa288 #0x00 [f2 a2 88 04 00]`; `0x1bc2 MOVG #0x94 [f2 a2 88 06 94]` | (b) |
| `0xa2a4` | b | note/channel (bits) | `0x1475 BSET @r2+0xa2a4 #0 [f2 a2 a4 c0]`; `0x1530 BCLR #0 [f2 a2 a4 d0]` | (b) |
| `0xa2c0` | b | note/channel | `0x1952 MOVG3 r1 -> @r2+0xa2c0 [f2 a2 c0 91]`; `0x19e1 CMP [f2 a2 c0 71]` | (b) |
| `0xa2dc` | b | note/channel | `0x1488 MOVG2 @r2+0xa2dc r1 [f2 a2 dc 81]`; `0x1996 MOVG3 r1 -> @r2+0xa2dc [f2 a2 dc 91]` | (b) |
| `0xa2f8` | b | note/channel | `0x1508 CMP @r2+0xa2f8 r4 [f2 a2 f8 74]`; `0x1659 [f2 a2 f8 75]` | (b) |
| `0xa314` | b | note/channel | `0x13b2 CMP @r2+0xa314 r1 [f2 a3 14 71]`; `0x190d store [f2 a3 14 90]` | (b) |
| `0xa330` | b | note/channel | `0x13b8 BTSTI @r2+0xa330 #0 [f2 a3 30 f0]`; `0x1915 store [f2 a3 30 90]` | (b) |

Group A (`0xa34c..0xad0e`) = allocator/bookkeeping; group B (`0xcdc6..0xd15c`) =
secondary per-voice arrays, several of which are copied into the AoS struct by `0x53eb`
(`cfe4,d038,d08c,cfac,d000,d054,cf90,ce78,cecc`) plus scan flags; group C
(`0xa250..0xa330`) = per-note/per-channel descriptors (linked list + part fields), not
voice-slot arrays. All three groups are (b).

> **Reconciliation with the prior doc.** `mk2_polyphony_256.md` §6.1 listed these 49 as
> "parallel arrays"; that reading is **confirmed** for the 49. What was missing is the
> *second layer*: the 28 AoS structs `[0xad2e,0xcdc6)` (8,344 bytes) and the rom1
> pointer table `0x64d6`, which are the actual per-voice working state written to the
> PCM interface. The earlier `r0=0xcc9c / stride 0x80` hypothesis is an artifact of
> reading `S=P-0x80` as a base (see §1.2).

---

## 7. Snapshot cross-check (`build/s28.bin`, `build/s256b.bin`)

Read formula: struct address `A` → file offset `0x450 + (A & 0x7fff)`
(`src/mcu.cpp:1256-1258`; `sram[0]=0x8000`).

* All 28 `P(v)` from `rom1[0x64d6]`, in **both** snapshots, satisfy
  `word[sram[P+0]] = word[sram[P+2]] = word[sram[P+4]] = 0x0016`, matching the reset
  loop `0x41248/0x4124d/0x41252`. This is an independent runtime confirmation that
  the 28 pointers are live voice structs and that the layout is unchanged in the
  28-voice snapshot.
* `sram[0xcd1a..0xcd21]` (slot 27): `00 00 | 00 16 00 16 00 16` — `P-2 = 0`
  (slot backref only written when the voice is materialized at `0x53f3`), state words
  `0x0016`.
* `sram[0xcc9c..]` (the `r1=0xcc9c` address) is zero in the snapshot; the bytes printed
  by the `src/mcu.cpp:1088` hook are rom2 tone data, not SRAM (see §1.2).
* Both snapshots identical over `[0xad2e,0xcdc6)` for the checked fields; the WIP
  `-voices:256` build has not (yet) relocated this region.

---

## 8. Capacity, and consequences for the 256-voice plan

* **Native capacity = 28** (table has 28 words; `0x41239 movi r0 #0x1b` = 27→0 loop =
  28 iterations; `0x40469`, `0x404dc`, `0x40508`, `0x4053b`, `0x51d9`, `0x51fc`,
  `0x5886`, `0x58d7`, `0x41239`, `0x41305`, `0x4135c` all use 27 as the top index).
* **No spare struct.** Slot 28 would need `P(28)=0xce46`, `S(28)=0xcdc6` — exactly
  the first byte of the `0xcdc6` word array (which is cleared at `0x2748`/`0x3622`).
  The AoS region ends flush against the SoA secondary arrays. `0x2098` bytes would
  have to be relocated/expanded for every additional voice.
* **The 4 extra PCM slots (28-31) are not voice structs.** Firmware's PCM channel
  selection (`br+0x3e`) uses 28-31 for effect channels (`0x5e20`, `0x5ee7`), and no
  `0x64d6` entries exist for them. A 32-voice config therefore cannot reuse them
  without new struct backing.
* **Action item for the existing plan (doc-only observation):** the page-6 layout in
  `mk2_polyphony_256.md` §6.5 R8 (0x3400 bytes of arrays) does **not** include the
  28×0x12a AoS region or the fact that `P(v)` is fetched from **rom1 0x64d6** (a ROM
  table, not an SRAM array). Any 256 implementation must additionally (a) extend
  `0x64d6` or replace the fetch sites with a computed base, and (b) allocate the
  struct region (`0x12a × N`) in extended memory, or the PCM write path (`0x54fb`,
  `0x5533`, `0x5626`) will read stale 28-slot state.

---

## 9. Open questions / unresolved contradictions

1. **Premise vs GT (resolved in favor of GT):** the "base 0xcc9c, stride 0x80, 32
   slots" model is contradicted by `rom1[0x64d6]` (28 words, stride 0x12a) and by the
   init loops. `0xcc9c` is `P(27)-0x80`. If any tool/trace still assumes 0x80 stride,
   it must be corrected; no 32-entry pointer table is used (or referenced) by the
   executed stock firmware.
2. **Debug-hook page bug:** `src/mcu.cpp:1096-1105` reads `struct@r1` through
   `mcu.ep`, while `r1`'s page is `dp` (`src/mcu.h:208-215`). The printed bytes match
   rom2 (proofs in §1.2). This diagnostic should be fixed or ignored before it is
   used as evidence.
3. **REC0/REC1 interpretation (S):** that `S+0x00` and `S+0x22` are two independent
   0x22-byte interpolator records is inferred from the two entry paths
   (`0x37f7` and `0x38ac`) sharing `0x38b0-0x39b1`. It cannot be ruled out that the
   `0x37fe` path is meant for a different struct layout. The exact pitch/envelope
   meaning of the record's six 16-bit positions is not established.
4. **Index-space of secondary arrays (S):** `0xcee8/0xcf04/0xcf20/0xcf58` are written
   with `r1 = slot` at `0x53eb-0x5469`, but also with `r1 = (dp,0xa4aa)` at
   `0x10c3-0x1106`. Whether `(dp,0xa4aa)` is a note counter that equals the slot
   (i.e. both are 0..27) or a different index space needs tracing.
5. **Unobserved struct fields:** the middle block `P+0x06..+0x7e` is a mix of verified
   sites and rows tagged `(agg.)` (only the aggregate disasm scan); their semantic names
   are hypotheses. A per-field dynamic trace (write/read at the PCM materialization
   path) would settle names.
6. **Other 0x7x-stride tables:** the rom2 copy loop at `0x42b9a-0x42c96+` uses
   `movi r5 #0xcc08/#0xcc78/#0xcce8` (stride 0x70) to copy data into
   `0x8208/0x8278/0x82e8`; `P+0x2e` points at `0x8048+n*0x70`. These are unrelated to
   the voice structs but are a plausible source of the "0x80-stride per-voice"
   misreading; they are **not** voice state.

---

## Appendix A — instrumented evidence files

* `tools/baselines/dasm_full.txt` — all PCs cited above.
* `tools/baselines/flow_main.txt` — control-flow edges used for `0x37f2/0x37fe`
  entries (`000037ea→000037f2`, `00002865→000037fe`, `00003001→000037fe`).
* `build/rom1.bin` — pointer table bytes `0x64d6`.
* `build/rom2.bin` — bytes `0x1cc9c`/`0x25ad` proving the hook read rom2.
* `build/s28.bin`, `build/s256b.bin` — SRAM snapshots (offset `0x450`).
* `build/cr_stock.out`, `build/ib_4.out`, `build/case_nocfg.out` — `CRASH@3970`
  lines (`r1=cc9c` / `r1=25ad`).
* `src/mcu.cpp:1088-1105` — temporary crash hook (`struct@r1` line).
