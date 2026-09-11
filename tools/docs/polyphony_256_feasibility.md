# Nuked-SC55 mkII — polyphony feasibility study (target 256; 255 = compromise cap; research-only, no source modified)

> **口径修订（2026-09-11）**：项目目标 = **256 声同时发音**；本文研究的 **N=255**
> 是 `0xff` 哨兵 + 8-bit 池计数妥协下的阶段性上限，**非最终目标**。文中所有
> “255-voice / 目标 N=255”表述均按“妥协上限”理解；真 256 需额外的哨兵表示/
> 计数宽度变更（见 `plan_256.md`、`polyphony_256_todo.md` R12/R14）。

Status: research deliverable, 2026-09-11. All claims cite T0 ROM bytes / GT `src/`
lines, T1 snapshots, or T2/T3 docs (`voice_memory_map.md`, `voice_bounds_inventory.md`,
`task_irq_map.md`, `mk2_polyphony_256.md`, `polyphony_256_todo.md`). Where a claim is
my own scan or an inference it is marked **[scan]** / **[I]** with the confirming test.

Address conventions (same as the cited docs): PC `0000xxxx` = rom1 flat, `0004xxxx` =
rom2 flat (`0x40000 + file offset`). `sram[a] = sram[a & 0x7fff]` at file base `0x450`
(`src/mcu.cpp:1256-1258`). Operand widths from operand-byte bit3 (`src/mcu_opcodes.cpp:558-561`).

---

## 0. TL;DR — max N per architecture

| Arch | Mechanism | Max N (arithmetic) | Verdict for 255 |
|---|---|---|---|
| **A** | stay in page-0 24 KB window; relocate SoA+AoS, rewrite literal bases/bounds | **46** conservative / 53 optimistic, then hard **127 cap** (sign tests) | no |
| **B** | B-copy code, all per-voice data in **one** page-6 `b_ram` (current plan) | **166** with current `kMoved` layout; **183** if tightly packed | no (fails 255) |
| **C** | compress AoS stride 0x12a→≤0x100, then page-6 holds AoS | 255 **iff** 0x2a (42) bytes/struct are provably dead | unproven; high risk |
| **D1** | B-copy + **two** extension pages: main block M=[P,P+0xaa) page6, sub-block S=[P-0x80,P) page7 (SoA page7) | **255** (8-bit cap; ~39 KB slack) | **go (recommended)** |
| **D2** | compressed AoS page6 + SoA relocated in page-0 | 255 iff C's liveness proof holds | fallback |

The documented 255 plan (`mk2_polyphony_256.md` §6.5 R8) budgets only **15,872 B of SoA**
in page 6 and **does not allocate the AoS structs at all** (already noted in
`voice_memory_map.md` §8). Since `0x12a × 255 = 75,990 B > 64 KB`, 255 is **not
implementable on one page-6**; it requires either a second 64 KB backing page or a
~14 % struct compaction plus moving the SoA elsewhere. The base+disp16 addressing model
itself survives (a page register per data class), but the per-voice code, the pointer
table representation and the page budget must be rewritten/extended. Details in §2–§6.

---

## 1. The two per-voice representations (recap, with fresh cross-checks)

### 1.1 AoS (array of structs)
* 28 structs × 0x12a (298) B = 0x2098 = 8,344 B at SRAM **[0xad2e, 0xcdc6)**
  (`voice_memory_map.md` §1.1/§2; re-verified: `0xcdc6-0xad2e = 0x2098 = 28*0x12a`).
* Interior pointer table in rom1: `rom1[0x64d6 + 2*v]`, P(0)=0xadae, stride 0x12a,
  28 entries; word after the table (`0x650e`) = `0x0006` (pitch table), not a pointer
  (`voice_memory_map.md` §2; raw rom1 read reproduced below).
  Raw table (`build/rom1.bin[0x64d6..0x650d]`):
  `ad ae ae d8 b0 02 b1 2c b2 56 b3 80 b4 aa b5 d4 b6 fe b8 28 b9 52 ba 7c bb a6
   bc d0 bd fa bf 24 c0 4e c1 78 c2 a2 c3 cc c4 f6 c6 20 c7 4a c8 74 c9 9e ca c8
   cb f2 cd 1c`.
* Struct span `[P-0x80, P+0xaa)` (= S..M), 0x12a total; slot backref at `P-2`
  (`0x53f3 MOVG3 r1 -> @r0+-2 [e8 fe 91]`); sub-block S=P-0x80 used by the interpolator
  `0x38b0` (entry `0x37f7 ADD #0xff80 r1 [0c ff 80 21]`; accesses `@r1+0..+32`,
  `dasm_full.txt:6964-7107`), REC1 path `0x38ac ADD #0xffa2 r1 [0c ff a2 21]`; main
  block M=P..P+0xa9 (`voice_memory_map.md` §4.1/§4.2, ~72 + ~100 field rows, evidence
  PCs there).
* The 12 static `0x64d6` read sites **[scan/u16]**: `0x25f4`, `0x2764`, `0x363d`,
  `0x5186`, `0x5240`, `0x53ef`, `0x588d`, `0x58bc`, `0x58ce`, `0x58de`, `0x41240`,
  `0x4130c` (`r16_hits.txt` / `r16_final.txt` labels "char table W" hits=12, exec=10);
  the 10 executed ones are exactly `voice_memory_map.md` §3's list. No writer exists —
  the table is ROM (voice_bounds_inventory.md §1.6).

### 1.2 SoA (arrays of structure members)
49 literal-base arrays (`voice_memory_map.md` §6 classification, lines 393-444). For the
B route the in-tree generator's moved list (`src/patch_256.cpp:22-40`, `kMoved`) is the
authoritative per-voice subset: **50 entries = 42 byte arrays (1 B/slot) + 8 word arrays
(2 B/slot)**:

| kind | bases | per-voice bytes |
|---|---|---|
| byte (42) | `a34c a368 a384 a3a0 a3bc a3d8 a3f4 a410 a4aa a4b4 acf2 ad0e` `ce3f ce5c ce78 ce94 ceb0 cecc cee8 cf04 cf20 cf3c cf90 cfe4` `d038 d08c d0a8 d0c4 d0e0 d0fc d118 d134 d15c` + descriptor pool `a250 a26c a288 a2a4 a2c0 a2dc a2f8 a314 a330` | `42 × N` |
| word (8) | `a46c 64d6 cdc6 cdfe cf58 cfac d000 d054` | `8 × 2N = 16N` |
| **SoA total** | | **`58N`** |

Representative evidence PCs per base are tabulated in `voice_memory_map.md` §6 (e.g.
`0xa368` `0x19a4 [f1 a3 68 93]`, `0xad0e` `0x17c0 [f1 ad 0e 70]`, `0xcfac` `0x540e
[fb cf ac 86]`, `0xcdc6` `0x3622 [f9 cd c6 13]`); the `0x53eb` materializer copies
`cfe4/d038/d08c/cfac/d000/d054/cf90/ce78/cecc` into the AoS (`0x53f6-0x5469`,
`voice_memory_map.md` §5.4). The 9 descriptor-pool bases were added to `kMoved` after
R18 (`polyphony_256_todo.md:146`).

### 1.3 Other per-voice / voice-adjacent state
| item | stock size | evidence | scales with N? |
|---|---|---|---|
| PCM mask current+pending `d150/d152/d154/d156` | 4 words = 8 B | `task_irq_map.md` §2.2; `0x41229-0x41235` clears them | yes: 256-bit ⇒ 64 B + main 8 B (MHI/PHI, `mk2 §6.5 R9`) |
| IRQ-pending `d15c[slot]` | 28 B (byte array) | `0x516c CLR`, `0x51e0 TST`, `0x52d/0x531` write (`dasm_full.txt:375-377`) | yes, already in the 42 byte arrays |
| live pointers `d158/d15a` | 4 B | `voice_memory_map.md` §5.7 | no (2 pointers) |
| `d1a6` | ≥1 B (byte-array style `@r1+0xd1a6`) | `0x1da4 [f1 d1 a6 90]`, `0x1ddf [f1 d1 a6 13]`, `(dp,0xd1a6)` `0x406fb/0x46640` | **[I]** per-voice; width unproven. Accessors are outside the B closure (`polyphony_256_todo.md:143`) |
| `d1ac[32]` | 32 B | init `0x43323 movi r1 #0xd1ac; 0x43326 movi r0 #0x1f; 0x43329 MOVG #4 -> r1++` (`dasm_full.txt:13871-13876`), scans `0x433ed`/`0x43644`, helpers `0x43695/0x436a3/0x436c3/0x436d5` + `0x436b5 EXTU r0; @r0+0xd1ac` (`dasm_full.txt:14129-14135`) | **[I]** 32 per-slot flags; must widen to N if it really is per-voice-slot, else voices ≥32 alias |
| `d435[32]` | 32 B | clear `0x43318-0x43320`; **no other reference** (`voice_bounds_inventory.md` §1.2) | no (dead, write-only) |
| pool scalars `a42c/a42d/a42e/a42f/a430` | 5 B | `voice_bounds_inventory.md` §2.2; `mk2 §6.5 R12` | no (a42d is 8-bit count, holds ≤255) |

### 1.4 Exact RAM budget (all four classes)

`per-voice = 0x12a (AoS) + 42 (SoA bytes) + 16 (SoA words) = 356 B` (+ 2N if the
d1a6/d1ac arrays are widened ⇒ 358N).

| N | AoS `298N` | SoA `58N` | pointer table `2N` | mask | d1a6+d1ac | misc | **total** |
|---|---|---|---|---|---|---|---|
| 28 (stock) | 8,344 | 1,624 | 56 | 8 | ~56 | ~10 | **~10.0 KB** |
| 64 | 19,072 (0x4A80) | 3,712 (0xE80) | 128 | 16 | 128 | 10 | **23,066 (0x5A1A)** |
| 128 | 38,144 (0x9500) | 7,424 (0x1D00) | 256 | 32 | 256 | 10 | **46,122 (0xB42A)** |
| 255 | **75,990 (0x128D6)** | 14,790 (0x39C6) | 510 | 72 | 510 | 10 | **91,882 (0x166EA) = 89.7 KB** |

The 255 total is `> 64 KB` by ~26 KB; the AoS alone exceeds one 64 KB page for
`N ≥ 220` (`65536/298 = 219.9`). The current page-6 layout payload is **15,872 B**
(`kMoved` extent 0x3E00 = 0x3A00 arrays + holes; `mk2 §6.5 R8` table; 14,848 B of it
array payload), so with that layout one page 6 fits at most
`floor((65536-15872)/298) = 166` AoS structs; with everything tightly packed
(`356N + 64 ≤ 65536`) the single-page ceiling is **183**.

### 1.5 d1ac/d1a6 confirmation tests (needed before >32 is safe)
* Instrument GT to log the index register fed to the `@r0+0xd1ac`/`@r1+0xd1a6` sites
  (`0x436b5`, `0x43647`, `0x1da4`) with PC; if values ever exceed 27 on a stock demo
  the arrays are per-voice-slot and must be widened, else they are per-PCM-channel and
  may be left. **(T0+T1, cheap.)**

---

## 2. Address-space budget

### 2.1 What dp=0 can reach (no operand page change)
* `MCU_Read_impl` page 0 maps the SRAM window `0x8000-0xdfff` to `sram[a & 0x7fff]`
  (`src/mcu.cpp:711-713`); `MCU_Write` same (`src/mcu.cpp:972-988`). That is **24,576 B**,
  not 32 KB (`sram[]` itself is 32 KB, `src/mcu.cpp:161`, but the upper 8 KB is shadowed
  by the PCM/GA/device windows `0xe000-0xffff`). Operand forms with a 16-bit displacement
  (`@rN+disp16`, `(dp,disp16)`) cannot produce a page bit: `effective =
  (page<<16)|((rN+disp16)&0xffff)` (`src/mcu_opcodes.cpp:631/646`), so moving data to
  another page **requires a page register** (dp/ep/tp, `src/mcu.h:208-215`).
* Snapshot occupancy `build/snap28.bin` (sram at file `0x450`; my block scan **[scan]**):

```
0x8000-0x8fff used (boot/global data, 16 blocks; 0x9000 56 nz)
0x9100-0x9fff all-zero (15 blocks = 3,840 B)
0xa000-0xa4ff used (part/note data)
0xa500-0xa7ff all-zero (768 B)
0xa800-0xacff used, 0xad00-0xbfff AoS region (sparse: state words)
0xc000-0xccff AoS region (sparse/free holes), 0xce00-0xd4ff used
0xd500 free, 0xd700-0xdbff free (1,536 B), 0xdc00-0xdfff used
```

  All-zero bytes in the 24 KB window: **7,168 B** [scan]. The conservative reference-density
  analysis (`polyphony_256_todo.md:31`, R1a) counts only **18 free 0x100-blocks = 4,608 B,
  max contiguous 2,048 B (0x9800-0x9f00)** because some zero blocks are code-referenced
  (coefficient tables 0x9000-0x9740 etc.). The current `≤64` relocation already uses
  0x9500-0xa5c0 (`src/reloc_data.h` new bases; 428 sites), corroborating that budget.
* Reclaimable if all voice state is packed: the stock voice/pool span
  `[0xa250,0xd1cb]` = **12,156 B** (descriptor pool + 42/8 arrays + AoS; minus a few
  non-voice globals wedged in: `ad2a/ad2c` 4 B, `d150-d15b` 12 B, `d1a6-d1cb`
  ~38 B).
* Net in-window capacity for packed voice state ≈ `12,156 + 4,608 = 16,764 B`
  (conservative) / `12,156 + 7,168 = 19,324 B` (snapshot-zero). With 358 B/voice
  + 64 B mask: **N ≤ 46** (conservative) / **N ≤ 53** (optimistic). Add the hard
  sign-test ceiling 127 (`voice_bounds_inventory.md` §4.1 item 7: `BPL/BMI` on byte
  slots break at 0x80; widening the 2-byte tests in place usually has no room) ⇒
  architecture A tops out at **≈46**.

### 2.2 Extension pages (only if accesses are rewritten to a page register)
* Page 6 = `b_ram[0x10000]`, 64 KB, already in GT (`src/mcu.cpp:657`,
  read `:853-856`, write `:1064-1067`, gated by `pcm_ext_enabled`) and in the VM
  (`mk2` 附2). It is reached only via a page register (`tp=6` ⇒ `r6`/`r7` operands).
* Pages 5/12/13 read 0xff on mk2; **page 7 falls to `default: 0x00`**
  (`src/mcu.cpp:847-852,858-860`) and writes print "Unknown write". Pages 10/11 alias
  the same 32 KB `sram[]` for non-mk1 (`src/mcu.cpp:833-839,1052-1055`) — a GT FIXME,
  and it adds no *new* bytes (same backing 32 KB).
* Adding page 7 as a second 64 KB backing is the same one-line class of change as page 6
  (`case 7: ret = b_ram2[address]`, write branch) — it adds **64 KB** of addressable
  per-voice RAM. Two pages = 128 KB ≥ 89.7 KB at N=255 (with ~30 KB slack even after
  code/data overheads).

### 2.3 Is GT "shadow translation" of page 0 to a larger backing store well-defined?
**No, not for the stock base+slot access pattern.** The CPU computes one 16-bit
effective address per access: `(base + index) mod 0x10000`. An emulator that only
observes the final address cannot recover which logical (array, slot) pair was meant:
* SoA: stock bases are 0x1c apart while the index grows to N−1 (e.g. `@r1+0xa368`
  vs `@r1+0xa384`): at slot 30, `0xa368+30 = 0xa386 = 0xa384+2`. Array-A-slot-30 and
  Array-B-slot-2 are literally the same physical byte. No injective shadow map exists.
* AoS: the firmware itself materialises P from the 16-bit ROM table. For N=255,
  `P(0)+254*0x12a = 0x1D55A > 0xffff`; the word wraps at `v=71`
  (`(0xffff-0xadae)/0x12a = 70.6`). A shadow map cannot see the intended slot either.
* Condition under which a shadow map *would* be well-defined: the union of all
  effective byte addresses used for every live (object,index) must be **injective and
  inside the mapping window**. That is only true after the bases/pointers have been
  rewritten to disjoint ranges — i.e. after doing the relocation, which is exactly the
  work architecture A/B does. Shadow translation is therefore **not a shortcut**; it
  is an emulator implementation detail *after* the firmware is rewritten.
* Corollary: any plan that says "just back page 0 with 128 KB" cannot work, because
  page 0 already has only 64 KB of address space and the overlap is in the address
  arithmetic, not in the backing size.

---

## 3. Code that must be rewritten to move SoA and/or AoS to page 6/7

### 3.1 SoA static inventory (T0-derived scan)
Source: `%TEMP%\opencode\r16_scan\r16_hits.txt` / `r16_final.txt` (raw-ROM scan for
every decode-valid `[f0-fb reg] disp16` and `[15] disp16` whose base is one of the 43
tracked per-voice bases; classification A = in the R11 baseline, E = executed but
missed, B = unexecuted but decode-valid):

* **306 hits** total; `run.log`: EXEC=237, UNEXEC=69; `r16_final.txt`:
  **A=222, E=15, B=70** (307 incl. one header artifact).
* Grouped by closure fragment (`src/patch_256.cpp:124-168`, ranges [start,end)) **[scan]**:

| fragment (PC range) | sites | fragment | sites |
|---|---|---|---|
| IRQ0 0x522-0x540 | 1 | `25f0` 0x25f0-0x26a2 | 9 (+`0x2764` B) |
| note-fill 0xf86-0x1034 | 19 | mega 0x272e-0x5492 (smallest-fit) | 60 |
| note-fill b 0x107d-0x1190 | 10 | `2d95` 0x2d95-0x2e21 | 1 |
| `13a9` 0x13a9-0x1459 | 0 | `2e83` 0x2e83-0x30f0 | 13 |
| `1459` 0x1459-0x14a9 | 5 | `3615` 0x3615-0x3709 | 2 |
| `151e` 0x151e-0x157d | 5 | `3a9e` 0x3a9e-0x3bb2 | 2 |
| dispatcher 0x15e0-0x15fb | 0 | `4de7` 0x4de7-0x50ef | 1 |
| handler 0x173e-0x179c | 3 | PCM enable 0x516c-0x51b6 | 2 |
| slot-select 0x179c-0x17ed | 5 | scan 0x51d9-0x51fb | 2 |
| note-helper 0x17ed-0x1806 | 0 | `53eb` 0x53eb-0x546d | 12 |
| allocA 0x1823-0x187e | 8 | `546e` 0x546e-0x548f | 1 |
| allocB 0x187e-0x18ce | 6 | `54cc`/write1 0x54cc-0x552c | 2 |
| `18ce`/`194c`/`19ad` | 20 | `5533` 0x5533-0x5608 | 2 |
| free 0x19c4-0x1a24 | 10 | `5998` 0x5998-0x5d6e | 2 |
| `1a4d` 0x1a4d-0x1a6c | 3 | pool 0x40462-0x4062b | 28 |
| mask-acc 0x1ad3-0x1af2 | 4 | init 0x41220-0x41262 | 3 |
| H1/H2 0x1b44/0x1bad | 10 | search 0x45c7c-0x45d9a | 2 |
| **inside kFrags total** | **253** | outside | 54 |

  (Assignment rule: each hit to the smallest containing fragment, so the 42 sites in
  `0x2d95..0x5d6e` that lie inside the mega span `0x272e-0x5492` appear only in their
  own rows; the raw "any fragment" count for mega would be 96 and double-counts.)
  The 54 outside-kFrags sites are documented gaps or deliberately excluded code:
  `0x588d-0x58e5` table scan/clear (missing from `kFrags` despite R11 listing
  `0x5869`), `0x4130c`, `0x1d4f/1da4/1ddf` (`d1a6`), `0x43697-0x436d7`
  (`d1ac` helper family, R16), `0x406b7-0x406fb`, and the unexecuted arms at
  `0x7cf-0xdd1`, `0x16de/0x16e6`, `0x54aa/0x54c5`.

### 3.2 Pointer-table (AoS) sites
12 static `0x64d6` reads (§1.1); 10 executed. A 255-voice build must additionally
generate a 255-entry table (510 B) whose values are page-relative — the current
`patch_256.cpp` moves the table to page 6 `0x2500` (`kMoved`) but **no code writes its
contents**, and no AoS backing exists (see §4 B).

### 3.3 AoS P-relative sites — the inventory that is missing
The existing static scan tracks only the 43 literal-base *SoA* arrays. The AoS is
addressed through P (small displacements), so it never appears in `r16_*`. **[scan]**
crude bounds over the AoS-touching closures:
* ≥ ~150 distinct PCs are already listed as field evidence in `voice_memory_map.md`
  §4.1/§4.2 (~172 documented field rows).
* A range scan (closure fragments, `@r0..@r3` with disp8/`0x00xx`) finds **842**
  r0-r3-operand sites in the rom1 fragments, of which the mega fragment
  `0x272e-0x5492` alone has 780 — an *upper* bound (many are pointer-free scratch).
* Exact count is a generator deliverable; the test is a decode/dataflow pass that marks
  each operand whose base register currently holds P (extend `tools/r16scan` or the
  `patch_256.cpp` decoder) and asserts 100 % coverage. Until that exists, any "255"
  plan that counts only the 307 SoA sites omits the entire AoS access surface.

### 3.4 Non-array rewrite sites (all architectures >28)
| class | count/sites | source |
|---|---|---|
| `cntjmp` bound `#0x1b` → `#n-1` | 12 sites: `0x1ad7, 0x51d9, 0x51fc, 0x5886, 0x58d7, 0x40462/404c8/404d9/40508, 0x41239, 0x41305, 0x4135c` | `voice_bounds_inventory.md` §1.1/§5 |
| `cmp r0,b #0x1c` → `#n` | `0x45cde` (`[40 1c]`, `dasm_full.txt:15944`) | same, §4.1 item 1 |
| 32-wide `movi #0x1f` reset/scan | `0x4331b, 0x43326, 0x433ed, 0x43644` (`d1ac`, `d435`) | §1.2 |
| sign tests on slot bytes | 26 sites in `kSent` (`src/patch_256.cpp:225-233`) | mk2 §6.5 R12 |
| mask set `0x546e-0x5484` (`BSET d154/d156`) | 1 routine, 3 byte paths | `task_irq_map.md` §2.2 |
| mask flush `0x54fb-0x552b` / `0x564a-0x5670` | 2 clusters, 4 `movsw` + 2 `movl` readback | `dasm_full.txt:11053-11055,11160-11162` |
| IRQ capture `0x529/0x52d/0x531` | 3 instructions | `dasm_full.txt:375-377`; `task_irq_map.md` §3.5.1 |
| effect select `0x3e`→`0x3f` | 42 E + 6 readback sites | mk2 §6.6 R17 |
| GT/VM: page7, `irq` full slot, `reg_slots` formula, `cycles` clamp | `src/pcm.cpp:587,1672`; page map | `task_irq_map.md` §3.6 |

### 3.5 Size of a B-copy implementation
* `kFrags` currently has **42 fragments** (`src/patch_256.cpp:124-168`); the closure is
  documented as **26 logical routines / 41 fragments** (mk2 §6.5 R11) plus the d1ac
  family and 5869 scan/clear (mk2 §6.6 R16; `polyphony_256_todo.md:143-151`).
* Raw source bytes: sum of fragment lengths = **17,246 B**; **union = 14,918 B**
  (overlap 2,328 B, dominated by the 11,620 B mega fragment `0x272e-0x5492`), plus
  rom2 fragments **1,273 B** ⇒ **16,191 B** of stock code to translate [scan].
  The generator grows each moved-array access by 4–6 B (253 sites ⇒ ~1.2 KB), plus
  wrappers (7 B × 42) and branch expansions, so a realistic B code size is
  **~18–22 KB**. The free rom2 window is `0x60000-0x6ffff` = **66.9 KB**
  (mk2 §6; `src/mcu.cpp:826-832` maps cp=0x0E to it), so code space is ample.
* Risks (from the previous B-block experiments): `rts` does not carry cp ⇒ wrappers +
  rom1 trampolines (`src/patch_256.cpp:385-403`); relative branches must be re-targeted
  two-pass; `cntjmp` cannot span pages (expanded to tramp+`pjmp`, `:503-531`); the B
  region runs with IML=7 (interrupt latency = routine duration, R5/R19); the mega
  fragment overlaps 6 other fragments (R11) so it must be deduplicated before install.

---

## 4. Architecture comparison

### A) In-place page-0 relocation (no new page)
Layout math: window 24,576 B; reclaimable voice span 12,156 B + free 4,608 B
(conservative) = 16,764 B; 358 B/voice + 64 B mask ⇒ **N = 46**
(`(16764-64)/358 = 46.6`). Snapshot-zero free gives N=53; sign-test ceiling 127.
Implementation: rewrite the disp16 field of every SoA site (307 static + gaps), extend
the pointer table in the rom1 cave (`0x7DC4-0x7FFF` = 572 B; 2×46 B is fine), widen the
12 bounds + `0x45cde`, widen `d1ac`, extend mask. No page registers touched.
Oracles: every (array,slot) address range disjoint and `< 0xE000` (static checker);
O1/O2/O3/O4/O5/O9 (`task_irq_map.md` §4).
Failure modes: any missed literal silently overlaps two arrays; the 0x64d6 table must
be mirrored out of ROM; the shipped reloc route (`src/patch_reloc.cpp`, clamp 64,
`src/mcu.cpp:1586-1589`) already **stalls at n≥48** (`task_irq_map.md` §3.2/§3.5.3,
unresolved root cause at `pc=0x37A`, IML=7) — so even 46 is not demonstrated today.

### B) Single-page B-copy (the documented plan: all per-voice data in page 6)
Max N: AoS `298N` + SoA (current `kMoved` layout fixed 15,872 B incl. MHI/PHI) ≤ 65,536
⇒ **N = 166**; tightly packed (`356N + 64 ≤ 65,536`) ⇒ **N = 183**.
Implementation (in tree, incomplete): 42 fragments/`patch_256.cpp`, wrappers, A→B
redirects; code ~16 KB source. Data layout `kMoved`/R8.
Oracles: O1–O7, O9, O10; plus a page-6 layout disjointness check.
Failure modes: **as specified it cannot hold the AoS** — `kMoved` has no `0xad2e`
struct region and no writer for the relocated `0x64d6` table; a slot ≥28 pointer reads
stale/pitch data (the `r0=0x25ad` crash, `voice_bounds_inventory.md` §3.1). N=64/128
fit comfortably (23 KB / 46 KB), N=255 does not.

### C) AoS stride compaction (0x12a → ≤0x100)
At stride 0x100, AoS = `256×255 = 65,280 B`, which just fits page 6; the SoA (14,848 B)
must move to page 7 or back to page-0 (14,790 ≤ 16,764 reclaimable).
Max N = 255 **iff** 42 B/struct are provably reclaimable. Evidence today:
`voice_memory_map.md` §4 marks only `P-2`, state words `P+0..+4`, `P+0x2e/0x30`,
`P+0x98..+0xa8` as Confirmed; the `P+0x06..+0x96` bulk is "S" with many rows tagged
`(agg.)`, and `P+0x58..+0x5e`/`P+0x74..+0x7e` etc. have no named consumer. There is
currently **no proof any byte is dead**; the confirming test is a full write/read
coverage trace of all 298 bytes across a long demo + MIDI stress. Because every field
offset must be rewritten anyway, the compaction risk is additive to the B rewrite.
Failure modes: dropping a live coefficient ⇒ envelopes/filters silently wrong.

### D1) Two extension pages, struct split (recommended for 255)
Idea: keep the base+disp16 model but route the two halves of the AoS to **different,
statically known pages**; no per-slot page switching:
* main block `M = [P, P+0xaa)` (170 B/voice) → page 6; at 255: 43,350 B
* sub-block `S = [P-0x80, P)` (128 B/voice) → page 7; at 255: 32,640 B
* SoA (14,848 B at 255, or 14,790 packed) → page 7 (32,640+14,848 = 47,488 ≤ 64 KB)
* mask MHI/PHI 64 B → page 6 (43,414 B) or page 7
Both pages fit with >18 KB slack per page (~39 KB total). Pointer table: one 16-bit
word per slot (E.g. store M offset `m`; S offset = `m-0x80`, with M starting at offset
0x80 in page 6 and S at 0 in page 7). Every access is statically M or S by the sign of
its displacement, so the generator emits `@r6+disp` (tp=6) for M and `@r4/r5+disp`
(ep=7) for S.
Implementation size: the §3.5 B rewrite **plus** (a) page-7 backing in GT/VM, (b) AoS
site remap and pointer-pair/layout generation, (c) B wrapper saves/restores ep in
addition to tp, (d) 64 KB second page in `state_save/load`. N is then bound by the
8-bit counters/sentinels ⇒ **255** (N=256 collides with the 0xff sentinel and a42d
overflow; `voice_bounds_inventory.md` §4.1 item 7, mk2 R12).
Oracles: O1–O10 plus (D1-specific) "no generated M offset beyond 0xAA16",
"all AoS/S accesses inside their page", IRQ handler reads full slot before ack.
Failure modes: page-register discipline (ep is dynamic in A code; B must mask
interrupts and restore both page regs — R5 wrapper already does SR/tp); register
pressure in routines needing S and SoA simultaneously; CPU load ~9× (per-frame DSP
loop 28→255, `src/pcm.cpp:1164/1672`).

### D2) Hybrid: compacted AoS page 6 + SoA page 0
Same as C but keeps the SoA in page 0 using the 16.7 KB reclaimable window; then only
one extension page is needed. It still needs the full B rewrite for AoS and the whole
SoA literal rewrite for page-0 relocation. Fallback if page 7 is rejected.

---

## 5. The hard question, answered

**Is 255 implementable without rewriting the firmware's addressing model?**
No. Even the most conservative accounting shows the AoS alone (`75,990 B`) exceeds any
single 64 KB extension page, and the 24 KB page-0 window holds at most ~46 voices.
Moreover the overlap analysis in §2.3 shows GT cannot hide this behind a
shadow-translated backing store: base+slot arithmetic aliases different logical fields
onto the same physical address. The minimum changes are structural:

1. **Pointer table**: 28-word ROM table → 255-entry generated table; values must be
   page-aware (either pair of offsets S/M as in D1, or a page:offset convention), and
   all 12 read sites plus every P-relative access must be re-based.
2. **Address space**: add a second 64 KB page (page 7) or compact the struct below
   256 B. Pages 5/7/12/13 are unbacked on mk2; extending page 6 to page 7 is a local
   GT/VM change (`src/mcu.cpp:657` pattern) but a firmware-wide access rewrite.
3. **Per-voice arrays (SoA)**: 307 known literal-base sites (253 inside the current
   kFrags + 54 outside gaps) + the un-inventoried AoS P-relative sites (bounds
   150–842) must be regenerated with page-relative bases.
4. **Counters/mask/IRQ/effects**: 12 bound immediates + `0x45cde`, 4×32-wide arrays,
   26 sign tests, the 32-bit mask path (set/flush/readback + MHI/PHI), the 5-bit IRQ
   slot truncation, and the 0x3e→0x3f effect alias.

The *model* (base register + 16-bit displacement) is preserved; what changes is that
"the" per-voice page becomes "the per-data-class page" (tp=6 for M, ep=7 for S/SoA),
and the pointer representation changes from a single 16-bit P to a page-aware pair.

---

## 6. Recommendation and next steps

**255: conditionally GO, via D1** (two pages, M page6 / S page7, static page routing).
It is the only option with full slack at N=255 and no unproven liveness assumption.
For a staged delivery, B-single-page already gives a genuine **N=128** (and up to 166
with the current layout) — but note the in-tree `≤64` path is still broken at n≥48.

Concrete next 3 implementation steps:
1. **Fix the plan's memory budget and add page 7.** Extend `mk2_polyphony_256.md` §6.5
   R8 with the AoS (M page6/S page7) and add a second 64 KB backing page to GT/VM
   (`src/mcu.cpp` page-map, `state_save/load`) behind `pcm_ext_enabled`; lift the
   `>64 → 64` clamp (`src/mcu.cpp:1586-1589`) only after the generator covers AoS.
   Oracle: page-7 probe (memprobe-style) + no-flag 200M–202M trace byte-identical.
2. **Extend the generator to the AoS and prove site coverage.** In `src/patch_256.cpp`
   add a P-tracking decode pass that enumerates every P-relative site in the closure
   (asserting 100 % coverage), generate the 255-entry pointer table (M offset; S =
   M-0x80), route M→tp=6, S→ep=7, and add the missing fragments (`0x5889` scan/clear,
   `0x4130c`, descriptor pool, `d1ac` family if the §1.5 test shows it is per-voice).
   Oracle: `b_dump.bin` disassembly + a `bmap.txt` site ledger with zero unmapped
   accesses; N=32 smoke run (isr≈1382/20M, idle `pc=00:0492`).
3. **Run the acceptance matrix.** Default (byte-identical), N=64 and N=128 on single
   page, then N=255 on D1; MIDI stress ≥255 notes; O1–O10 (`task_irq_map.md` §4) plus
   the D1 page-disjointness checks; a ≥300M audio listen per the user's run rules.

Confidence: budget arithmetic and page limits **C** (raw bytes + src + snapshots);
site counts for SoA **S** (static scan, decode-validated); AoS site count and d1ac/d1a6
semantics **I** (needs the §1.5/§3.3 tests); D1 layout is a design, not yet built.

---

## Appendix — evidence index
* `tools/baselines/dasm_full.txt`: `000037f2/37f7/38aa/38ac/38b0-39b1` (interpolator,
  substruct); `000025f4`, `00005186`, `00005240`, `000053ef`, `0000588d/bc/ce/de`,
  `00041240`, `0004130c` (0x64d6); `00045cc2/cca/cdc/cde/ce0` (search loop);
  `00000529-531` (IRQ), `00005525-529/5662-666` (mask flush); `00043318-3329`
  (d435/d1ac reset); `000436b5-d7` (d1ac helper).
* `src/mcu.cpp`: 158-161 (sizes), 657/853-856/1064-1067 (page 6), 696-699/955-958
  (0xe800 ext), 711-713/972-988 (page-0 SRAM window), 1256-1258 (snapshot), 1576-1592
  (MCU_PatchROM/reloc clamp), 1915-1925 (`-voices`); `src/pcm.h:27-37`;
  `src/pcm.cpp:89-99,217-219,233,288-308,587-590,1164,1672`.
* `src/patch_256.cpp`: 22-40 (`kMoved`), 124-168 (`kFrags`), 225-233 (`kSent`),
  253-618 (rewrite, no AoS case), 674-855 (build/install).
* `src/reloc_data.h` (428 sites / 12 bound imms) + `src/patch_reloc.cpp`.
* `tools/docs/voice_memory_map.md` §1-§8; `voice_bounds_inventory.md` §1-§5;
  `task_irq_map.md` §0-§5; `mk2_polyphony_256.md` §6.3-§6.6, §8;
  `polyphony_256_todo.md` R1a/R11/R12/R16/R18.
* `%TEMP%\opencode\r16_scan\{r16_hits.txt,r16_final.txt,run.log}` (static scans),
  `build/snap28.bin` (T1 occupancy), `build/rom1.bin` (pointer table).
