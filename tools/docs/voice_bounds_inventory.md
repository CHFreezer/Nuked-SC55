# Voice-count dependency inventory — main firmware (SC-55 mkII v1.01)

Research-only inventory (no code changed). Evidence base:

- `tools/baselines/dasm_full.txt` — trace disassembly (flat addresses; rom1 = PC,
  rom2 = `0x40000 + fileoff`). All PC/bytes below are quoted from it.
- `tools/baselines/flow_main.txt` — executed control-flow edges (call sites).
- `roms/SC-55mk2-v1.01/rom1.bin`, `rom2.bin` — raw bytes.
- GT (`src/`) for instruction semantics: `MCU_Jump_JMP` cntjmp at
  `src/mcu_opcodes.cpp:377-385`, PCM reg decode `src/pcm.cpp:123-145,587`,
  memory map `src/mcu.cpp:674-714`.

Loop-count semantics used throughout:

- `cntjmp rX, rel` (`0x01/0x06/0x07`) decrements rX and branches while the result
  is not `0xffff` → **body runs `initial+1` times** (`mcu_opcodes.cpp:380-384`).
  Therefore an N-entry loop is encoded as `movi rX #(N-1)`.
  Verified against the firmware itself: pool-init loop C starts with
  `movi r1 #0x1b` and increments `(dp,0xa42d)` once per pass; the post-boot
  snapshot value is `a42d = 0x1c = 28`.
- `cmp rX,b #K; BNE` do-while loops run **K** times (K is the exit value).

The stock config gives N = 28; the constants that encode it are **0x1b (=N-1)**
for `cntjmp` loops and **0x1c (=N)** for the one `cmp`-terminated voice search.
There is **no `movi rX #0x001c` anywhere** in the traced firmware.

---

## 1. Master inventory table

### 1.1 VOICE-COUNT — must scale with N

| PC | instruction | classification | correct value for N | evidence / notes |
|---|---|---|---|---|
| `0x001ad7` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Routine `0x1ad3` (called from `0x00621`, `0x00827`; edges in flow_main), IML-protected. Body `0x1ada-0x1aea`: `r0 = a4b4[r1]; r0 |= acf2[r1]; acf2[r1] = r0; a4b4[r1] = 0`; `0x1aea cntjmp r1 -19 -> 0x1ada`. Per-voice **voice-enable accumulator** (a4b4 = new bits from note path, acf2 = accumulated bits). |
| `0x0051d9` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Dispatcher `0x51d0` (trapa #0x10). Body `0x51e0-0x51e6`: `TST @r1+0xd15c; BNE 0x51ef; cntjmp r1 -9 -> 0x51e0`. Scans the per-voice **PCM IRQ-pending flags** d15c[0..27]; first non-zero → `0x51ef CLR @r1+0xd15c; BRA 0x25f0`. |
| `0x0051fc` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Same dispatcher (`r0==0` path). Body `0x520b-0x5212`: `SUB @r1+0xd0e0 #0x00; BNE 0x521b; cntjmp r1 -10`. Scans per-voice **pending-command slots** d0e0[0..27], then dispatches through `@r2+0x522c` jump table. |
| `0x005886` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Routine `0x5869` region (reached from `0x56c0`). Body `0x5889-0x58d4`: `r2=2*r1; r0=word[0x64d6+r2]; SUB @r0+0,#0x12; BCC skip; TST @r0-26; BNE skip; …; 0x58d4 cntjmp r1 -78 -> 0x5889`. Scans all N voices via the **rom1 0x64d6 pointer table** (see §4.1). |
| `0x0058d7` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Body `0x58da-0x58e5`: `r2=2*r1; r0=word[0x64d6+r2]; CLR @r0-26; cntjmp r1 -14`. Bulk-clears a per-voice byte through the same table. |
| `0x040462` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Pool-init loop A (routine `0x40462`, entered by `0x565: pjsr #0x04:0462`). Body `0x40469-0x404c2`: initializes per-voice arrays `ce5c/ce78/ce94/d118/d134/ceb0/cf20/cf3c/cf90/d0fc/d0a8/d0c4/ad0e/ce3f/d0e0/a4b4/a34c`, then `SHLL r1` and word arrays `cfac/d000/d054/a46c`, `SHLR r1`; `0x404c2 cntjmp r1 -92 -> 0x469`. |
| `0x0404c8` | `movi r2 #0x001b  [5a 00 1b]` | VOICE-COUNT (**multi-purpose seed, not a loop bound**) | **N-1** | Not a `cntjmp` bound itself. It (a) seeds `r2`, which is the cursor of the descriptor-pool loop D at `0x4053b` (`cntjmp r2 -40`, init bodies `a250/a288/a26c/a314/a2c0/a2dc`), (b) `0x404cb MOVG2 r2 r1` copies it into r1, (c) `0x404cd/d1/d5 MOVG3` store `a432=0x0f`, `a434=0x1b`, `a436=0x1b` (both write-only, never read in the traced firmware). |
| `0x0404d9` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Pool-init loop B. Body `0x404dc-0x404ef`: `a3f4[r1]=0xff; a410[r1]=0xff; a3bc[r1]=0; a3d8[r1]=0xff`; `0x404ef cntjmp r1 -22 -> 0x4dc`. Per-voice sentinel init. |
| `0x040508` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Pool-init loop C (**free/pool list build**). `0x40504 CLR (dp,0xa42d)`; body `0x4050d-0x40533`: links r1 into the `a3d8` chain (`a42f`/`a430` head/tail), sets `a3a0[r1]=0x94`, `ADDQ #1 (dp,0xa42d)`; `0x40533 cntjmp r1 -41`. **After the loop `a42d = N`** (snapshot evidence 0x1c). |
| `0x041239` | `movi r0 #0x001b  [58 00 1b]` | VOICE-COUNT | **N-1** | Routine `0x41220` (called by `0x51cc: pjsr #0x04:1220`). Body `0x4123c-0x4125b`: `r2=2*r0; r1=word[0x64d6+r2]; CLR ad0e[r0]; word[r1+0/2/4]=0x16; CLR d15c[r0]; cntjmp r0 -34`. Per-voice **descriptor re-init** through the rom1 table. |
| `0x041305` | `movi r1 #0x001b  [59 00 1b]` | VOICE-COUNT | **N-1** | Routine `0x41266` (called by `0x56b6: pjsr #0x04:1266`). Body `0x41308-0x41318`: `r2=2*r1; r0=word[0x64d6+r2]; @r0-26=0xff; bsr 0x42b8a` (clears `@r0-108`, `@r0-74`, `(dp,ce36/ce38)`); `cntjmp r1 -19`. |
| `0x04135c` | `movi r0 #0x001b  [58 00 1b]` | VOICE-COUNT (**PCM channel loop**) | **N-1** stock; only valid ≤ 31 without GT remap | Routine `0x41333` (called by `0x217: pjsr #0x04:1333`). `r0` is written to PCM `select_channel` (`0x41362 movs r0 @(br,$3e)`) and regs 0x12/0x14=0, 0x16/0x18/0x1a=0xba are written per channel; `0x41374 cntjmp r0 -21`. This is a **hardware slot** loop, not a memory-array loop: see §3.2. |
| `0x045cde` | `cmp r0,b #0x1c  [40 1c]` | VOICE-COUNT | **N** (immediate = N, exit value; for 255 → `0xff`) | **NOT in the current patch list.** Routine `0x45c2`/`0x45cbe` DSP search. Body `0x45cc2-0x45ce0`: `r6 = a368[i]`; if `r6==r4`, `r6=ad0e[i]` (0xff→0), keep the minimum age in r5; `0x45cdc ADDQ #1 r0`; `0x45cde cmp r0,b #0x1c; 0x45ce0 BNE -32`. Runs i = 0..27 (28 entries), i.e. exit bound = N. Byte compare caps N ≤ 255. |

### 1.2 FIXED-32 — stock width 32, per-voice, must become N only when N > 32

| PC | instruction | classification | correct value for N | evidence / notes |
|---|---|---|---|---|
| `0x04331b` | `movi r0 #0x001f  [58 00 1f]` | FIXED-32 (per-voice array, write-only) | 32 stock; N if N>32 | `0x43318 movi r1 #0xd435`; loop `0x4331e CLR r1++; cntjmp r0 -5` → clears `d435..d454` (32 bytes). `d435` has **no other reference** in the traced firmware (write-only). Harmless at 28–32; widen for N>32 for safety/consistency. |
| `0x043326` | `movi r0 #0x001f  [58 00 1f]` | FIXED-32 (**per-voice**) | **N if N>32** | `0x43323 movi r1 #0xd1ac`; loop `0x43329 MOVG #0x04 -> r1++; cntjmp r0 -6` → fills `d1ac..d1cb` (32 bytes) with 4. `d1ac` is per-voice by runtime index: `0x436b5 EXTU r0; 0x436b7 MOVG2 @r0+0xd1ac r1; 0x436be MOVG3 r1 -> @r0+0xd1ac` (bit-clear helper). Overflows for N>32. |
| `0x0433ed` | `movi r0 #0x001f  [58 00 1f]` | FIXED-32 (**per-voice scan**) | **N if N>32** | `0x433ea movi r1 #0xd1cb`; loop `0x433f0 BTSTI @r1 #2 / #1`, `0x43419 ADDQ #-1 r1`, `cntjmp r0 -46` → scans `d1cb` down to `d1ac` (32 bytes = per-voice flags). |
| `0x043644` | `movi r0 #0x001f  [58 00 1f]` | FIXED-32 (**per-voice scan**) | **N if N>32** | `0x43641 movi r1 #0xd1cc`; loop `0x43647 BTSTI --r1 #0`, `0x43691 cntjmp r0 -77` → scans `d1cb..d1ac` (32 bytes, bit0 per voice). Executed from `0x3ba` and `0x7c3f` fan-outs (flow_main). |

### 1.3 Hardware FIXED slots (must NOT scale; alias with voices if N>28/31)

| PC | instruction | classification | correct value for N | evidence / notes |
|---|---|---|---|---|
| `0x0005e20` | `move r0 #0x1c  [50 1c]` | FIXED hardware slot (**PCM channel 28 = effect ch**) | 0x1c always | Routine `0x5e1b`: `r2=30, r1=29` then `movs r2 @(br,$3e)`, writes effect regs. Channel 28/29/30 are the effect channels; 0x1c is a slot number, not a voice count. GT stock remaps 28–31 to EFF_BASE (`pcm.cpp:141-145`). |
| `0x0005ee7` | `move r1 #0x1c  [51 1c]` | FIXED hardware slot (effect ch) | 0x1c always | Same routine family (`0x5ea3`), paired with `r2=0x1d`, `r3=0x1e`. |
| `0x00061e1` | `move r6 #0x1f  [56 1f]` | FIXED hardware slot (**PCM channel 31 = global**) | 0x1f always | `movs r6 @(br,$3e)`; channel 31 programs the global register block (regs 0x14-0x1a). |
| `0x00061f9` | `move r3 #0x1f  [53 1f]` | FIXED hardware slot (global) | 0x1f always | `0x6201 movs r3 @(br,$3e)`, then `movsw r5 @(br,$12)`. |
| `0x0006241` | `move r6 #0x1f  [56 1f]` | FIXED hardware slot (global) | 0x1f always | `0x6247 movs r6 @(br,$3e)`, `movsw r1 @(br,$1e)`. |
| `0x00062fc` | `move r6 #0x1f  [56 1f]` | FIXED hardware slot (global) | 0x1f always | `0x6302 movs r6 @(br,$3e)`, writes 0x14/0x16/0x18/0x1a. |
| `0x000633b` | `MOVG #0x1f -> (br,$3e)  [05 3e 06 1f]` | FIXED hardware slot (global) | 0x1f always | `0x633f movsw r5 @(br,$12)`. |
| `0x00041385` | `MOVG #0x1f -> (br,$3e)  [05 3e 06 1f]` | FIXED hardware slot (global) | 0x1f always | In the boot channel-init routine `0x41333`, after the `0x4135c` voice loop. |
| `0x000413cf` | `move r6 #0x1f  [56 1f]` | FIXED hardware slot (global) | 0x1f always | `0x413d1 movs r6 @(br,$3e)` (boot global init). |

Operational note: these are *not* voice-count constants, but they collide with
voice slots 28–31 as soon as N > 28. GT handles stock by remapping channels
28–31 to effect slots 28–31 (`pcm.cpp:141-145`); an extension needs an alias
(reg 0x3f or EFF_BASE) and must not "widen" these immediates.

### 1.4 UNRELATED — same numeric values, do NOT patch

| PC | instruction | classification | value | evidence / notes |
|---|---|---|---|---|
| `0x0002b38` | `movi r2 #0x001f  [5a 00 1f]` | UNRELATED (pitch math) | 0x1f | `0x2b2a movi r4 #0x1fc0; 0x2b2d MULXU @r3+0x650e r2; SUB r2 r4; 0x2b33 cmp r4,w #0x0020; BCS; 0x2b38 movi r2 #0x001f; 0x2b3b movi r3 #0xc000; DIVXU r4 r2:r3` → 32-bit `0x001f_c000 / r4` pitch normaliser. 0x1f is the high word of the dividend, not a count. |
| `0x004134` | `movi r2 #0x001f  [5a 00 1f]` | UNRELATED (pitch math) | 0x1f | rom1 duplicate of the same normaliser (`0x412f cmp r4,w #0x0020`). |
| `0x0001a4` | `MOVG #0x1f -> (br,$fe)` | UNRELATED (port init) | 0x1f | Reset port/P7 init at boot. |
| `0x0001c1` | `MOVG #0x1f -> @r4` (`r4=0xe402`) | UNRELATED (GA/LCD IRQ enable) | 0x1f | Boot init of 0xe402 (GA interrupt enable). |
| `0x000069f`/`0x0001099` | `ADDS #0x20 r5` | UNRELATED (arithmetic) | 0x20 | sign/extension arithmetic. |
| `0x0000e2d` | `ADD #0x0020 r5` | UNRELATED (struct stride) | 0x20 | Advances a `0xd8`-stride part record (`0x0e1c MULXU #0x00d8`). |
| `0x0021b5`,`0x21f5`,`0x2226`,`0x258c`,`0x25ae`,`0x25bf` | `ADD #0x0020 r1` | UNRELATED (record stride) | 0x20 | Pointer bump inside 0x20-byte records. |
| `0x000448a9` | `MOVG #0x20 -> (dp,0xffb0)` | UNRELATED (device reg) | 0x20 | Writes device register 0xffb0 (timer/PWM area). |
| `0x00046656` | `MOVG #0x1c -> (dp,0xfffe)` | UNRELATED (device reg) | 0x1c | Boot init writes 28 to device register 0xfffe (ADC/port area), followed by `BTSTI (dp,0xffff)`. Not a loop bound, no consumer in traced code. |
| `0x0004676a` | `MOVG #0x20 -> @r0` | UNRELATED (ASCII space) | 0x20 | Stores 0x20 into `d25f+…` (name/patch buffer). |
| `0x00043967` | `move r0 #0x20` then `jsr 0x3dc7` | UNRELATED (ASCII space fill) | 0x20 | `0x43dc7` fills `0x8f+1=144` bytes at `d22f` with 0x20 (text buffer). |
| `0x00048f95` | `move r5 #0x20  [55 20]` | UNRELATED (default value) | 0x20 | Default velocity/value in a note-event path. |
| `0x00044835` / `0x00044d1d` | `MOVG #0x1b -> (dp,0xd37c)  [15 d3 7c 06 1b]` | DATA-LENGTH / write-only | 0x1b always | Written in init `0x447df` and reset `0x44cd2`. `d37c` is **never read** in the traced firmware (only two writes); its neighbour `d37d` is read at `0x45bfd` as a table index. No evidence it feeds any voice structure. Leave unchanged; residual risk only if an untraced consumer exists. |
| `0x0046699` | `cmp r0,b #0x1c  [40 1c]`, then `cmp r0,b #0x24` | UNRELATED (threshold) | 0x1c / 0x24 always | `0x46696 jsr #0xbb36` returns a timer/analog value; `0x46699/0x4669d` compare it against 28/36. Nothing to do with voice slots. |

### 1.5 PART-16 and other loop counts in the same routines (context, do NOT patch)

| PC | instruction | classification | value | notes |
|---|---|---|---|---|
| `0x0404c5` | `movi r3 #0x000f  [5b 00 0f]` | PART-16 | 15 | Pool-init loop E (`0x40565-0x40581`, `ADDQ #-1 r3; BPL`) clears 16 part records `a210/a220/a230/a240/a200`. Deliberately **not** in the bound patch list. |
| `0x0043c7f` | `movi r5 #0x000f` | PART-16 | 15 | 16-iteration inner loop of the display/LED swap routine (`0x43c4f` region). |
| `0x0043de2` / `0x0043dee` | `movi r2 #0x001f  [5a 00 1f]` | FIXED-32 **display buffer** | 32 stock, keep 32 | `0x43ddf/0x43deb` fill `d2cf..d2ee` and `d2f0..d30f` with 0xff. Evidence this is display/part state, not voice: `0x43c58` swaps exactly **16 bytes** `d2cf..d2de` with `d2df..d2ee`, then writes to LCD data register 0xe405 (`0x43d2b/0x43d2d`). The 32-byte fills cover two 16-byte display buffers. |
| `0x00043e0a`/`0x43e13` | `movi r2 #0x003f` | DATA-LENGTH | 64 | Clears/fills 64-byte envelope tables at `0xfec0` stride 0x40. |

### 1.6 Per-voice pointer table stride sites (no bound constant, but capacity-bound)

The rom1 table at **0x64d6** is read in 10 places; these accesses all assume a
valid per-voice descriptor pointer:

`0x0025f4`, `0x005186`, `0x005240`, `0x0053ef`, `0x00588d`, `0x0058bc`,
`0x0058ce`, `0x0058de`, `0x041240`, `0x04130c` (plus the two loop headers
`0x5889/0x58da` and the two rom2 init loops). There is no write to 0x64d6
anywhere in rom1/rom2: it is a **read-only ROM table**.

---

## 2. Runtime propagation of the voice count

### 2.1 The ROM config word (only source of the "28" value outside code)

```
0x0041333: LDC #0x00 r5 / LDC #0x00 r4 / LDC #0xe0 r3 / CLR r0 / CLR r1
0x004133e: LDC #0x04 r5                       ; CP=4 → rom2
0x0041343: MOVG2 @r1+0x1432 r1  [f9 14 32 81] ; r1 = word @ flat 0x41432
0x0041347: OR r1 r0
0x0041349: LDC #0x00 r5
0x004134c: MOVG3 r0 -> (br,$3c) [0d 3c 90]    ; word → PCM regs 0x3c/0x3d
0x004134f: MOVG #0x0000 -> (br,$00)           ; zero voice-enable banks 0/2
0x0041354: MOVG #0x0000 -> (br,$02)
0x0041359: MOVG2 (br,$3e) r1                  ; read select_channel
0x004135c: movi r0 #0x001b                    ; N-1
0x0041362: movs r0 @(br,$3e)                  ; select_channel = 0..27
   … per-channel init, cntjmp r0
0x0041377: MOVG #0x1e -> (br,$3e)             ; global/effect window
0x0041385: MOVG #0x1f -> (br,$3e)             ; global window
```

Raw bytes: `rom2.bin[0x1432..0x1433] = c3 7b` → H8 big-endian word `0xc37b`.
MOVG3 writes the word to PCM page: `config_reg_3c = 0xc3`,
`config_reg_3d = 0x7b`. GT: `reg_slots = (config_reg_3d & 31) + 1 = 28`
(`src/pcm.cpp:587`); the same function remaps stock channels 28–31 to
EFF_BASE (`src/pcm.cpp:141-145`).

**Key negative result:** the firmware never reads config regs 0x3c/0x3d back
(the only `(br,$3c)` reference is the write at `0x4134c`). The config word is
consumed **only by the PCM**, once, at boot. Firmware loop bounds are hardcoded
literals. Consequently:

- changing only the config byte (`rom2[0x1433] = N-1`) changes the PCM slot
  count but **not** firmware indexing;
- changing only the `movi` literals changes firmware indexing but **not** the
  PCM slot count;
- both must be changed together (as `patch_256.cpp:839` + bound stamps do).

Call chain: `0x217 pjsr #0x04:1333` (boot) → `0x41333`.

### 2.2 Internal RAM copies derived from N

| RAM | written at | value | read at | meaning |
|---|---|---|---|---|
| `a42d` | cleared `0x40504`; `ADDQ #1` inside loop C (`0x4052f`), and on pool transitions `0x186c/0x18bc/0x1a1c`; `ADDQ #-1` at `0x19bf` | **N at boot** | `0x1587`, `0x1873`, `0x18c3` | The only RAM variable that equals the voice count at reset. Pool/list bookkeeping for the `a3d8` chain (`a42f`/`a430` head/tail, snapshot `a42d=0x1c`). For N voices the loop C bound must be N-1 so this counter is N. |
| `a434` = byte `0x1b`, `a436` = byte `0x1b` | `0x404d1`, `0x404d5` | N-1 | **never read** in the traced firmware | Write-only leftovers of pool init. Safe to stamp N-1, but nothing depends on them. |
| `a432` = byte `0x0f` | `0x404cd` | 15 | **never read** | Write-only; part-count-like constant; do not confuse with the voice count. |
| `a42c` | `0x158d` (`a4a8 - a42d`, only if > 0), decremented/clamped at `0x1873-0x1879`, `0x18c3-0x18c9`, tested `0x15cc` | derived | `0x15cc` | A **signed shortfall** value, not a capacity. |
| `a4a8` | `0x0e85` (routine `0x0e1c`) | 1 or 2 in trace | `0x1583`, `0x1922` | **Not the voice count.** Routine `0x0e1c` computes it from a `0xd8`-stride part/tone record (`0x0e1c MULXU #0x00d8`, `0x0e2a MOVG2 @r5+18 r0`), and `0x1922` uses it as the per-note allocation loop count. Treat as a per-note element count, independent of N. |

### 2.3 Per-frame consumers of the (hardcoded) count

- `0x1ad3` mask accumulator (N-1) → `acf2[voice]` → consumed in the PCM program
  path (`0x2f17 TST @r1+0xacf2`) → voice-enable bit built into `d150/d152`
  (+ pending `d154/d156`) and flushed to PCM regs 0..3:
  `0x5525/0x5527 movsw r3/r4 @(br,$00/02)` and
  `0x5662/0x5664 movsw r5/r6 @(br,$00/02)`.
  Stock mask window is **32 bits (4×16)**, so slots ≥32 have no bit.
- `0x51d0` dispatcher (N-1 scans of `d15c` pending flags and `d0e0` commands).
- `0x45c2/0x45cbe` per-frame voice search (`cmp #0x1c`, bound = N).
- alloc/free note paths (`0x1823/0x187e/0x19c4`, `0x13a9/0x1459/0x151e/0x173e/
  0x179c/0x17ed`) traverse `a3d8`/`a250` lists and touch per-voice arrays by
  slot index; they contain **no hard N bound** — they auto-extend only if the
  pool is initialized larger, the arrays exist for those slots, and
  the rom1 0x64d6 table can resolve them.
- PCM IRQ path: `0x529 MOVG2 (dp,0xe03e) r0; 0x52d AND #0x001f r0;
  0x531 MOVG #0xff -> @r0+0xd15c`. The channel captured from the PCM is
  truncated to 5 bits: for N > 32 the wrong `d15c` entry (or none) is set.

---

## 3. Structural capacity dependencies not captured by any `movi`

### 3.1 rom1 0x64d6 — per-voice descriptor pointer table (hard DATA-LENGTH)

Raw `rom1.bin[0x64d6..0x650d]` (big-endian words):

```
0xadae 0xaed8 0xb002 0xb12c 0xb256 0xb380 0xb4aa 0xb5d4
0xb6fe 0xb828 0xb952 0xba7c 0xbba6 0xbcd0 0xbdfa 0xbf24
0xc04e 0xc178 0xc2a2 0xc3cc 0xc4f6 0xc620 0xc74a 0xc874
0xc99e 0xcac8 0xcbf2 0xcd1c
```

Exactly **28 entries** (pointer stride `0x12a`), ending at `0x650d`. The very
next word, `0x650e = 0x0006`, is the start of the pitch/log table
(`0x650e: 00 06 0d 13 1a 20 …`). Any loop that walks `@r?+0x64d6` past
index 27 will read pitch-table bytes as if they were voice pointers. This is
the most likely origin of a bogus per-voice pointer such as `r0 = 0x25ad` after
widening bounds while leaving the config/table at 28: `patch_256.cpp` already
lists `0x64d6` as a "moved array" (→ page6 0x2500), but the table is **ROM
data**; a valid N-entry replacement must be synthesised and its struct targets
(`0x12a` stride) allocated.

### 3.2 PCM channel select is a hardware field

`0x4135c` programs `select_channel = 0..N-1`; GT stock masks to 5 bits
(`pcm.cpp:141`) and remaps 28–31. Widening `0x4135c` alone (or together with
the other bounds while GT/effects remain stock) makes channels 28–31 address
effect/global registers instead of voices — corruption with no crash point,
or duplicated writes to `(val & 0x1f)` when GT is extended but `0x3e` is still
masked downstream.

### 3.3 Voice-enable mask is 32 bits

The enable bits are accumulated per voice (`acf2`) and flushed into regs 0–3
(`d150/d152` + `d154/d156` pending; `0x5525/0x5527/0x5662/0x5664`). No firmware
loop scales this beyond 32 bits; N > 32 needs the extended mask window.

### 3.4 Effect/global slot aliasing at N > 28

The firmware always programs effect registers through PCM channels 28–30 and
globals through 31 (`0x5e20/0x5ee7/0x61e1/0x61f9/0x6241/0x62fc/0x633b/0x41385/
0x413cf`). As long as the PCM remaps those to the effect block this is fine;
after extension they must not be treated as voices (see §1.3).

---

## 4. Review of the current patch sites (12 listed)

Verdict legend: **OK** = genuine N-1/N loop; **MISCLASSIFIED** = numeric value
is right but the site is not what the label says; **CONDITIONAL** = only valid
together with the structural changes; **MISSING** = no longer in the list.

| Site | Current treatment | Verdict | Explanation |
|---|---|---|---|
| `0x040462` | bound stamp N-1 | **OK** | Loop A, clears 17 byte arrays + 4 word arrays per voice. Needs arrays relocated for N > 28, else it overwrites adjacent SRAM. |
| `0x0404c8` | bound stamp N-1 | **MISCLASSIFIED (and dangerous alone)** | Seeds loop D (`0x4053b`, descriptor pool `a250…a330`) and writes write-only `a432/a434/a436`. It is not a loop header; patching it changes a different array family than A/B/C. Alone it widens a pool whose entries are later indexed by voice slots that the rom1 0x64d6 table cannot resolve. |
| `0x0404d9` | bound stamp N-1 | **OK** | Loop B sentinel init. |
| `0x040508` | bound stamp N-1 | **OK** | Loop C pool build; defines `a42d = N`. |
| `0x001ad7` | bound stamp N-1 | **CONDITIONAL** | `acf2` accumulator is 1 byte/voice and safe, but acf2 only reaches the PCM through the 32-bit enable mask. Widening without extending the mask silently drops voices ≥32. |
| `0x0051d9` | bound stamp N-1 | **CONDITIONAL** | The IRQ side that populates `d15c` truncates the PCM channel with `AND #0x1f` at `0x52d`. Widening the scan without fixing the capture either misses IRQs for slots ≥32 or sets the wrong entry. `patch_256.cpp` already carries `kIPatch {0x529→0xe820, 0x52d→0x00ff}` inside fragment `0x522` (currently disabled). |
| `0x0051fc` | A-only bound stamp N-1 | **OK** | d0e0 command scan; no companion requirement beyond arrays. |
| `0x005886` | A-only bound stamp N-1 | **CONDITIONAL** | Walks the 28-entry rom1 0x64d6 table. With no table extension this reads the pitch table from index 28 onward. |
| `0x0058d7` | A-only bound stamp N-1 | **CONDITIONAL** | Same table. |
| `0x041239` | bound stamp N-1 | **CONDITIONAL** | Same table (initializer writes through pointers from it). |
| `0x041305` | bound stamp N-1 | **CONDITIONAL** | Same table. |
| `0x04135c` | bound stamp N-1 | **MISCLASSIFIED (hardware loop)** | PCM `select_channel` init 0..27. "N-1" is only meaningful up to 31; beyond that it needs the GT select/effect remap. Full N (0xfe) is not usable with a stock PCM model. |

### 4.1 Sites that are MISSING from the current list

1. **`0x045cde cmp r0,b #0x1c`** — the DSP voice search; correct stamp is `N`
   (`0xff` for 255), not `N-1`. It iterates 0..27 in stock. No entry exists in
   `kBoundMovi`/`kABoundMovi`/`kIPatch`.
2. **32-wide per-voice arrays** `0x04331b/0x043326/0x0433ed/0x43644`
   (`d435`, `d1ac`). These do not need changes for 28 < N ≤ 32, but overflow
   for N > 32. None of these PCs is in the patch tables, and `d1ac`/`d435` are
   not in `kMoved` either.
3. **rom1 0x64d6 table length** — no generated N-entry replacement exists;
   `kMoved` merely relocates the access base. Without synthesised pointer
   entries (and `0x12a`-stride targets) every per-voice routine breaks for
   slots ≥28. This is the prime suspect for the observed crash
   (`r0=0x25ad`) when only the `movi` bounds were widened.
4. **IRQ channel capture** (`0x52d AND #0x1f`) — present as a per-site
   instruction patch but only inside the disabled `0x522` fragment; it must be
   considered a required companion of `0x51d9`.
5. **32-bit voice-enable mask** (`0x5525/0x5527/0x5662/0x5664`) — no firmware
   hook in the current bound lists; required for any N > 32.
6. **Effect/global aliasing** (`0x5e20/0x5ee7` ch 28; `#0x1f` global selects) —
   required for any N > 28 when GT is extended.
7. **Per-voice index registers** — most loops use an 8-bit index/compare
   (`cmp rX,b`, byte stores). N is therefore capped at 255 (0xff is also the
   `a3f4/a410/a3d8/ce3f` sentinel). N ≥ 128 additionally breaks signed tests
   (`BPL/BMI`) on those byte indices unless rewritten (matches the existing
   255-voice design decision).

### 4.2 Do-not-patch list (confirmed non-voice)

`0x0002b38`, `0x004134` (pitch normaliser), `0x0043de2`, `0x0043dee` (display
buffers), `0x0046699` (timer/analog threshold), `0x00044835`/`0x00044d1d`
(write-only `d37c`), `0x0004676a`/`0x43967`/`0x48f95` (ASCII/display),
`0x0001a4`/`0x1c1` (port/GA init), `0x000448a9`/`0x46656` (device registers),
all `ADDS/ADD #0x20` (strides/arithmetic), `0x0404c5` (PART-16 loop E).

---

## 5. Concise summary

- **Correct value for N:** `N-1` for the 12 real `cntjmp`/seed sites
  (`0x1ad7, 0x51d9, 0x51fc, 0x5886, 0x58d7, 0x40462, 0x404c8, 0x404d9,
  0x40508, 0x41239, 0x41305, 0x4135c`); `N` (`0xff` at 255) for the missed
  byte-compare site `0x45cde`.
- **Wrong / misclassified current sites:** `0x0404c8` (descriptor-pool seed +
  write-only stores, not a loop bound) and `0x04135c` (PCM hardware channel
  loop, only valid ≤ 31 without a GT remap). The remaining ten are genuine
  loops, but four of them (`0x5886/0x58d7/0x41239/0x41305`) are only safe once
  the rom1 0x64d6 table is replaced; `0x1ad7`/`0x51d9` additionally need the
  mask and IRQ-capture fixes.
- **Missing sites:** `0x045cde` (bound = N); `0x4331b/0x43326/0x433ed/0x43644`
  (32-wide per-voice `d1ac`/`d435`, N for N>32); the **28-entry rom1 0x64d6
  pointer table** (hard DATA-LENGTH, next word is the pitch table at 0x650e);
  IRQ mask `0x52d`; 32-bit enable mask flush; effect/global slot aliasing.
- **Runtime propagation:** only the boot path `0x41343` reads the config word
  (rom2 0x1432 = `c3 7b`) and writes PCM regs 0x3c/0x3d; the firmware never
  reads it back. The only RAM value equal to N is `a42d` (pool counter,
  initialised by loop C); `a434/a436` get N-1 but are write-only; `a4a8/a42c`
  are per-note element/shortfall values and are **not** voice-count copies.
