# SC-GS (Sound Canvas VSTi) resonant filter — decoded & compared to Nuked-SC55 pcm.cpp

Reverse-engineered from `C:\Program Files\Roland VS\SOUND Canvas VA\SCCore.dll`
(27,347,456 B, x64 PE32+, ImageBase 0x180000000, stripped). Full disasm in `re/sc.asm`.

## Engine identity

`SCCore.dll` is Roland's **SC-GS / SC-8820 / XP-GS "Tone Generator"** (product banner
strings in `.rdata`). The 3 KB `SOUND Canvas VA.dll` is only the VST stub that loads it.
Self-contained C++ x64: imports only KERNEL32 / VCRUNTIME140 / UCRT (no `dlsplay.dll`).
Plays **RKS/DLS** sound data (Roland's DLS variant). DSP is **float32 SSE**
(`mulss` x7619, `addss` x8016 dominant). This is a *modern reimplementation*, not the
SC-55 chip's 20-bit fixed-point — hence a **soft reference**.

## The filter

Two implementations of the *same* 2-integrator (State-II) resonant 2nd-order low-pass.
Each scalar call processes a **batch of consecutive samples for one voice** — the two
integrator states are carried serially in memory between steps (inputs stride 0x10,
outputs packed 0x04).

- **Scalar** — `0x18008ce70` dispatches by a 2-bit per-voice state (`r15d & 3`) to one of
  four variants. All are the *same* State-II lowpass; they differ **only in output tap and
  coefficient wiring** (re-verified against the disasm, 2026-09-06):
  - `0x18008d0a0`  out = **state2** (low-pass);  `s1'=S1+S0·B; err=in−(S0·A+s1'); s0'=S0+err·B`
  - `0x18008d2d0`  out = **err** (residual);      same recurrence as 0x…d0a0
  - `0x18008d520`  out = **state1**;              same recurrence as 0x…d0a0
  - `0x18008d740`  out = **state2 − err**;        integrators use **A** (not B); B only in feedforward err
- **SIMD (packed SSE, 4 voices/iter)** — `0x18008d9a0`. Same recurrence (`mulps/addps/subps`),
  per-lane enable mask via `andps/orps` from the `0x181a04040` table.

Canonical scalar recurrence (`0x18008d0a0`, decoded):

```c
// per voice, per sample.  A = coeffA, B = coeffB (per-voice, set in param path).
state2 = state1 * B + state2;
out    = state2;                        // low-pass tap
err    = input  - (state2 + state1 * A);
state1 = state1 + err * B;
```

Constants (`.rdata`, readable):
- `0x181a04030` = [1,1,1,1]   - blend default
- `0x181a04040` = [0,0,0,0]   - per-lane enable mask table, 16 B/voice
- `0x181a04130` = [−NaN x4]   - (0xFFFFFFFF) select/invert constant
Per-voice coeff/input/output tables at `0x181a1c…`/`0x181a1d…` live in **.data BSS**
(zero-init, filled at runtime).

### Saturation (the mk2-relevant finding)

Across the **entire** `sc.asm` (.text): **0** `minps/maxps/minss/maxss/blendps/blendvps`
and **0** `comips/ucomips`. The only select-ops in the filter range are the `andps/orps`
per-lane enable masks (SIMD), **not** a bound clamp. => the filter is **pure float32 with
NO per-sample saturation**. The two scalar `comiss`+branch clamps that *do* exist are
**outside the filter**: `0x180006bd0` clamps a param to ±1.0 (`0x3F800000`/`0xBF800000`),
and `0x180005ce4` is a float→int range clamp (`0x7FFFFFFF`/`0x80000000`) around a bit-extract
decode — both generic helpers, not the resonant state.

## Comparison with `src/pcm.cpp`

**Chip mk1 (20-bit fixed-point, SATURATED every add):**
```c
A1 = filter >> 8;  A2 = (filter >> 1) & 127;  B = (ram2[6] >> 8) & 127;
s2 = addclip20(reg3, s1*A1, …);              // + s1*A2 (addclip20), + s1*B (addclip20)
err = input - s2;                            // sign-flip via ^0xfffff
s1 = addclip20(reg1, err*A1, …);             // + err*A2 (addclip20)
ram1[3] = s2;  ram1[1] = s1;                 // out = s2
```

**Chip mk2 (`// hack: use 32-bit math`, UNSATURATED):** same structure, plain 32-bit
int, `multi`→`reg1*(int8_t)(…)` and `addclip20`→`+`.

**SC-GS VSTi (float32, UNSATURATED):** the `state2 = state1*B + state2; …` form above.

Differences that matter for the mk2 crackle:

| | numeric | saturation | state2 terms | state1 terms |
|---|---|---|---|---|
| chip mk1 | 20-bit (±524288) | 20-bit clamp every add | s1·A1+s1·A2+s1·B | (in−s2)·A1+(in−s2)·A2 |
| chip mk2 hack | 32-bit int (±2¹³¹→±2^31) | none | (same, plain int) | (same, plain int) |
| SC-GS VSTi | float32 (±3.4e38) | none | s1·B | (in−s2−s1·A)·B |

- **Topology matches**: two integrator states, `state2` = low-pass output, `state1`
  integrates the error. => pcm.cpp's filter *structure* is validated by Roland's own engine.
- **Coefficient split differs** (chip: 3+2 terms; VSTi: 2-coeff State-II with a
  feedforward zero). Both are 2nd-order resonant low-passes; the exact constant mapping
  needs the chip's filter-coeff table (`ram2[11]` + MCU filter param).
- **The headline (re-verified 2026-09-06; the "word-width" theory is refuted)**: Roland's
  reference filter is **unsaturated** (no per-sample clamp in any variant or the SIMD path).
  The mk2 hack drops saturation *and* caps at 32-bit int. The 4-quadrant A/B shows word
  width is **not** the cause — `-float` (float range, no saturation) **still crackles**,
  while `-floatsat`/`-mk1` (saturated) are clean. => the crackle is the **missing per-add
  saturation on a marginally-unstable resonant loop**, not 32-bit overflow.
- **Why the unsaturated VSTi stays clean but the unsaturated chip does not**: the chip's
  `g1 = A1/64 + A2/8192` can reach **≈±2.0** (A1 ∈ ±128, A2 ∈ 0..127). Schur-stability of
  the 2-integrator lowpass needs `g1² + 2·g1·g2 < 4` and `0 < g1·g2 < 2`; with the chip's
  `g2 = B/64 ≥ 0` and `g1` near ±2, many patches are **unstable** → the state rings to
  full scale every sample. The VSTi's A/B are DLS-derived and kept in the stable range, so
  it needs no clamp. Per-add saturation (mk1) tames even the unstable case into a bounded
  limit cycle — exactly what `-float` now does.

## Reference value / limits

- Use to: confirm filter topology (done), confirm the wave ROM (real Roland GM/RKS samples),
  isolated **dry-voice** A/B (single voice, no chorus/reverb).
- Not for: sample-accurate mk2 A/B — float32 vs 20-bit int differ in the tail/edge cases.
- Chorus/reverb in the VSTi are DLS-standard, NOT the SC-55 chip's — the "wrong vibe".
