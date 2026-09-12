/*
 * HAND voice/native_pool -- voice pool initialization, one host step per H8
 * instruction (M4, round 7).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 6
 *
 * Replaces the rom2 pool-init routine 0x40462-0x4062a (pjsr from 0x565,
 * ret from 0x40586) with per-PC entries. Every registered PC executes exactly
 * one H8 instruction and returns 1, so the host keeps its per-instruction poll,
 * TIMER_Clock, trace and SM_Update cadence. The previous form aggregated the
 * routine into four L1 blocks (A 0x40462..0x404b0 = 22 instructions,
 * B 0x404b4..0x4061e = 103 PCs, C 0x0621, D 0x40624/27/2a); a multi-
 * instruction block defers the host tail, so -midiseq / real-time MIDI bytes
 * queued inside the block are consumed by the SM at the block end instead of
 * at each instruction boundary. That timing drift is why A and B are now
 * per-PC (the alloc/free and materialize slices were converted for the same
 * reason); C and D already were. M4 closure step 2 (2026-09-13) turned all six
 * ranges A-F into named per-PC entries with {pc, fn} registration tables.
 *
 *   range           PCs  notes
 *   A 0x40462..0x404b0  22  seed + loop A first half
 *   B 0x404b4..0x4061e 103  loop A tail, free/desc/part tables, helpers
 *   C 0x40621            1  MOVG #0xff -> r2++ (16x16 fill store)
 *   D 0x40624/27/2a      3  cntjmp r1 -6 / cntjmp r3 -48 / rts
 *   E 0x4062b..0x40672  29  part descriptor scan (pjsr from 0x625)
 *   F 0x40674..0x406e2  39  kill/release, rom2 variant of kill_a (pjsr 0x62a/0x660)
 *
 * E/F are the continuation of the pool-init family: the note/part command ring
 * handlers 0x625 (state 08) / 0x62a (0a) / 0x660 (16) pjsr #0x04:0x062b /
 * #0x04:0x0674. Both routines live at rom2 off 0x62b..0x6e2 and are taken over
 * whole. 0x40673 and 0x4068d are mid-instruction bytes of the two-byte `ret`
 * at 0x40672 and the `BMI 73` at 0x4068c, so the valid start set is
 * 0x4062b..0x40672 + 0x40674..0x406e2. The tail 0x4068e..0x406e2 used to be
 * owned by gen, but the 0x4068c BMI targets 0x406d7 inside it and the rest is
 * its loop body; leaving it to gen would split routine F in the middle, so the
 * whole segment is registered here (hand wins over gen in MK2CPP_Step). Both
 * exits are `ret (pop cp,pc)` at 0x40672 / 0x406e2, matching the pjsr stack
 * order. E walks the descriptor chain a220 -> a250 and the per-part a288/a314
 * words; F is the rom2 twin of note_path.cpp B3 kill_a 0x1459..0x14a8
 * (a288/a2a4/a2dc + a3bc/a4b4/a410 + 0xff fill).
 *
 * 0x40586 (ret, pops cp+pc) stays with the generated code, exactly like
 * before; 0x4062a's rts pops to it and the next dispatch runs it per-PC.
 *
 * The stock boot executes 2866 instructions (2868 traced PCs minus the two
 * interrupt-poll lines); per-PC stepping reproduces the same sequence, and the
 * cycle count comes from the host's own +12 per entry.
 *
 * Instruction semantics come from GT (src/mcu_opcodes.cpp); flag effects are
 * mirrored with GT's own helpers. The ROM's LDC #1/#2 r4 (0x4059c/0x405a6)
 * writes ep and sets ex_ignore like the stock LDC (src/mcu_opcodes.cpp:996),
 * so the following host poll is skipped exactly as stock skips it. All state
 * lives in page-0 SRAM and is accessed with MCU_Read/MCU_Write, so the state
 * hashdump observes exactly the stock bytes.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"

#include "native_pool.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as native_allocfree.cpp / pcm_enable.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* ---- GT instruction effects (src/mcu_opcodes.cpp) ------------------------ */

/* CLR flags: N=0, Z=1, V=0, C=0. */
void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG3 / MOVG_Immediate -> mem: N/Z/V from the value, C preserved. */
void store8(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
    MCU_SetStatusCommon(value, 0);
}

void store16(uint32_t addr, uint16_t value)
{
    MCU_Write16(addr, value);
    MCU_SetStatusCommon(value, 1);
}

void mov_imm8(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
    MCU_SetStatusCommon(value, 0);
}

void mov_imm16(uint32_t addr, uint16_t value)
{
    MCU_Write16(addr, value);
    MCU_SetStatusCommon(value, 1);
}

/* CLR mem (byte/word): GT also clears C. */
void clear8(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

void clear16(uint32_t addr)
{
    MCU_Write16(addr, 0);
    flags_clr();
}

/* MOVG2 mem -> rN; byte loads keep the register's high byte. */
void load8(uint16_t &reg, uint32_t addr)
{
    uint8_t value = MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

void load16(uint16_t &reg, uint32_t addr)
{
    uint16_t value = MCU_Read16(addr);
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* movi rN #imm16 (Short_MOVI) / move rN #imm8 (Short_MOVE). */
void movi(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

void move8(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

/* SHLL/SHLR word: C = shifted-out bit, N/Z/V from the result. */
void shll16(uint16_t &reg)
{
    uint16_t data = reg;
    uint16_t carry = (uint16_t)((data & 0x8000u) != 0);
    data = (uint16_t)(data << 1);
    reg = data;
    MCU_SetStatus(carry, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

void shlr16(uint16_t &reg)
{
    uint16_t data = reg;
    uint16_t carry = (uint16_t)(data & 1u);
    data = (uint16_t)(data >> 1);
    reg = data;
    MCU_SetStatus(carry, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

/* SHLL/SHLR on the low byte only (operand byte has bit3 clear). */
void shll8(uint16_t &reg)
{
    uint16_t data = (uint16_t)(reg & 0xffu);
    uint16_t carry = (uint16_t)((data & 0x80u) != 0);
    data = (uint16_t)((data << 1) & 0xffu);
    reg = (uint16_t)((reg & 0xff00u) | data);
    MCU_SetStatus(carry, STATUS_C);
    MCU_SetStatusCommon(data, 0);
}

void shlr8(uint16_t &reg)
{
    uint16_t data = (uint16_t)(reg & 0xffu);
    uint16_t carry = (uint16_t)(data & 1u);
    data = (uint16_t)(data >> 1);
    reg = (uint16_t)((reg & 0xff00u) | data);
    MCU_SetStatus(carry, STATUS_C);
    MCU_SetStatusCommon(data, 0);
}

/* ADDQ on a register's low byte: high byte preserved, flags from the byte. */
void addq8_low(uint16_t &reg, int delta)
{
    int32_t value = MCU_ADD_Common((int32_t)(reg & 0xffu), delta, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | (uint8_t)value);
}

/* ADDQ #1 / #-1 on a memory byte: GT computes the flags from the byte. */
void addq8(uint32_t addr, int delta)
{
    int32_t value = MCU_ADD_Common((int32_t)MCU_Read(addr), delta, 0, 0);
    MCU_Write(addr, (uint8_t)value);
}

/* cmp r4,w #imm: flags only, no write (word). */
void sub16_nowrite(uint16_t value, uint16_t imm)
{
    (void)MCU_SUB_Common((int32_t)value, (int32_t)imm, 0, 1);
}

/* MULXU #0xd8 rH:rL: H = high word, L = low word, N/Z from the 32-bit product. */
void mulxu_d8(uint16_t &high, uint16_t &low, uint16_t value)
{
    uint32_t product = 0xd8u * (uint32_t)value;
    high = (uint16_t)(product >> 16);
    low = (uint16_t)product;
    MCU_SetStatus((product & 0x80000000u) != 0, STATUS_N);
    MCU_SetStatus(product == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* BCLR @addr #0: Z = (bit == 0), then clear bit 0; other flags untouched. */
void bclr0(uint32_t addr)
{
    uint8_t data = MCU_Read(addr);
    MCU_SetStatus((data & 1u) == 0, STATUS_Z);
    MCU_Write(addr, (uint8_t)(data & 0xfeu));
}

/* BTSTI @addr #0: Z = (bit == 0); other flags untouched. */
void btsti0(uint32_t addr)
{
    MCU_SetStatus((MCU_Read(addr) & 1u) == 0, STATUS_Z);
}

/* TST @addr (byte) / TST rN (word): N/Z/V from the value, C = 0. */
void tst8(uint32_t addr)
{
    MCU_SetStatusCommon(MCU_Read(addr), 0);
    MCU_SetStatus(0, STATUS_C);
}

void tst16(uint16_t value)
{
    MCU_SetStatusCommon(value, 1);
    MCU_SetStatus(0, STATUS_C);
}

/* EXTU rN (aN 12): zero-extend the low byte; N/V/C = 0, Z from the result. */
void extu8(uint16_t &reg)
{
    uint8_t data = (uint8_t)reg;
    reg = data;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(data == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* `ret (pop cp,pc)` (0x11 0x19): pop cp first, then pc, like GT
 * MCU_Jump_JMP (src/mcu_opcodes.cpp:345). */
void ret_cp_pc(void)
{
    mcu.cp = (uint8_t)MCU_PopStack();
    mcu.pc = MCU_PopStack();
}

/* Conditional branch targets (GT MCU_Jump_Bcc, src/mcu_opcodes.cpp:231). */
uint16_t bpl_pool(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? fall : taken; }
uint16_t bcc_pool(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_C) ? fall : taken; }
uint16_t beq_pool(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? taken : fall; }
uint16_t bmi_pool(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? taken : fall; }
uint16_t bne_pool(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? fall : taken; }

/* Effective addresses: @rN+disp through dp (r0-r3), ep (r4/r5), tp (r6/r7);
 * (dp,addr16) absolute. Same pages the interpreter uses. */
uint32_t ind_addr(uint32_t reg, uint16_t disp)
{
    uint8_t page = (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
    return ((uint32_t)page << 16) | (uint16_t)(mcu.r[reg] + disp);
}

uint32_t dp_addr(uint16_t disp) { return ((uint32_t)mcu.dp << 16) | disp; }

/* ---- A 0x40462..0x404b0: loop-A first half (per-PC named entries) -------- */

/* 0x40462 movi r1,#0x001b [59 00 1b] -- slot cursor 27..0. */
void step_a_load_count(void)
{
    movi(mcu.r[1], 0x001b);
    mcu.pc = 0x0465;
}

/* 0x40465 movi r2,#0x0000 [5a 00 00]. */
void step_a_clear_value(void)
{
    mcu.r[2] = 0;
    MCU_SetStatusCommon(0, 1);
    mcu.pc = 0x0467;
}

/* 0x40467 move r3,#0xff [53 ff]. */
void step_a_set_ff(void)
{
    move8(mcu.r[3], 0xff);
    mcu.pc = 0x0469;
}

/* 0x40469 movg #0xff,@r1+0xce5c -- clear the per-slot flag arrays. */
void step_a_clear_ce5c(void)
{
    store8(ind_addr(1, 0xce5c), (uint8_t)mcu.r[3]);
    mcu.pc = 0x046d;
}

/* 0x4046D movg #0xff,@r1+0xce78. */
void step_a_clear_ce78(void)
{
    store8(ind_addr(1, 0xce78), (uint8_t)mcu.r[3]);
    mcu.pc = 0x0471;
}

/* 0x40471 movg #0xff,@r1+0xce94. */
void step_a_clear_ce94(void)
{
    store8(ind_addr(1, 0xce94), (uint8_t)mcu.r[3]);
    mcu.pc = 0x0475;
}

/* 0x40475 movg #0xff,@r1+0xd118. */
void step_a_clear_d118(void)
{
    store8(ind_addr(1, 0xd118), (uint8_t)mcu.r[3]);
    mcu.pc = 0x0479;
}

/* 0x40479 movg #0xff,@r1+0xd134. */
void step_a_clear_d134(void)
{
    store8(ind_addr(1, 0xd134), (uint8_t)mcu.r[3]);
    mcu.pc = 0x047d;
}

/* 0x4047D movg #0xff,@r1+0xceb0. */
void step_a_clear_ceb0(void)
{
    store8(ind_addr(1, 0xceb0), (uint8_t)mcu.r[3]);
    mcu.pc = 0x0481;
}

/* 0x40481 movg #0xff,@r1+0xcf20. */
void step_a_clear_cf20(void)
{
    store8(ind_addr(1, 0xcf20), (uint8_t)mcu.r[3]);
    mcu.pc = 0x0485;
}

/* 0x40485 movg #0x3c,@r1+0xcf3c. */
void step_a_set_cf3c(void)
{
    mov_imm8(ind_addr(1, 0xcf3c), 0x3c);
    mcu.pc = 0x048a;
}

/* 0x4048A movg #0xff,@r1+0xcf90. */
void step_a_clear_cf90(void)
{
    store8(ind_addr(1, 0xcf90), (uint8_t)mcu.r[3]);
    mcu.pc = 0x048e;
}

/* 0x4048E movg #0xff,@r1+0xd0fc. */
void step_a_clear_d0fc(void)
{
    store8(ind_addr(1, 0xd0fc), (uint8_t)mcu.r[3]);
    mcu.pc = 0x0492;
}

/* 0x40492 movg #0xff,@r1+0xd0a8. */
void step_a_clear_d0a8(void)
{
    store8(ind_addr(1, 0xd0a8), (uint8_t)mcu.r[3]);
    mcu.pc = 0x0496;
}

/* 0x40496 movg #0xff,@r1+0xd0c4. */
void step_a_clear_d0c4(void)
{
    store8(ind_addr(1, 0xd0c4), (uint8_t)mcu.r[3]);
    mcu.pc = 0x049a;
}

/* 0x4049A movg #0x00,@r1+0xad0e -- zero the activity bytes. */
void step_a_clear_ad0e(void)
{
    store8(ind_addr(1, 0xad0e), (uint8_t)mcu.r[2]);
    mcu.pc = 0x049e;
}

/* 0x4049E movg #0x00,@r1+0xce3f. */
void step_a_clear_ce3f(void)
{
    store8(ind_addr(1, 0xce3f), (uint8_t)mcu.r[2]);
    mcu.pc = 0x04a2;
}

/* 0x404A2 movg #0x00,@r1+0xd0e0. */
void step_a_clear_d0e0(void)
{
    store8(ind_addr(1, 0xd0e0), (uint8_t)mcu.r[2]);
    mcu.pc = 0x04a6;
}

/* 0x404A6 movg #0x00,@r1+0xa4b4. */
void step_a_clear_a4b4(void)
{
    store8(ind_addr(1, 0xa4b4), (uint8_t)mcu.r[2]);
    mcu.pc = 0x04aa;
}

/* 0x404AA movg #0x00,@r1+0xa34c. */
void step_a_clear_a34c(void)
{
    store8(ind_addr(1, 0xa34c), (uint8_t)mcu.r[2]);
    mcu.pc = 0x04ae;
}

/* 0x404AE shll r1 -- cursor index * 2 for the word arrays. */
void step_a_double_cursor(void)
{
    shll16(mcu.r[1]);
    mcu.pc = 0x04b0;
}

/* 0x404B0 mov.w r2,@r1+0xcfac -- first word array. */
void step_a_store_cfac(void)
{
    store16(ind_addr(1, 0xcfac), mcu.r[2]);
    mcu.pc = 0x04b4;
}

/* ---- B 0x404b4..0x4061e: pool-init body (per-PC named entries) ----------- */

/* 0x404B4 mov.w r2,@r1+0xd000 -- loop-A tail word arrays. */
void step_b_store_d000(void)
{
    store16(ind_addr(1, 0xd000), mcu.r[2]);
    mcu.pc = 0x04b8;
}

/* 0x404B8 mov.w r2,@r1+0xd054. */
void step_b_store_d054(void)
{
    store16(ind_addr(1, 0xd054), mcu.r[2]);
    mcu.pc = 0x04bc;
}

/* 0x404BC mov.w r2,@r1+0xa46c. */
void step_b_store_a46c(void)
{
    store16(ind_addr(1, 0xa46c), mcu.r[2]);
    mcu.pc = 0x04c0;
}

/* 0x404C0 shlr r1 -- back to the byte cursor. */
void step_b_halve_cursor(void)
{
    shlr16(mcu.r[1]);
    mcu.pc = 0x04c2;
}

/* 0x404C2 cntjmp r1,-92 -> 0x40469 -- loop A over 28 slots. */
void step_b_loop_a(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x0469u : 0x04c5u;
}

/* 0x404C5 movi r3,#0x000f -- part count. */
void step_b_load_part_count(void)
{
    movi(mcu.r[3], 0x000f);
    mcu.pc = 0x04c8;
}

/* 0x404C8 movi r2,#0x001b -- slot count seed. */
void step_b_load_slot_count(void)
{
    movi(mcu.r[2], 0x001b);
    mcu.pc = 0x04cb;
}

/* 0x404CB mov.w r2,r1 -- mirror the count. */
void step_b_copy_count(void)
{
    mcu.r[1] = mcu.r[2];
    MCU_SetStatusCommon(mcu.r[2], 1);
    mcu.pc = 0x04cd;
}

/* 0x404CD mov.w r3,@a432. */
void step_b_store_a432(void)
{
    store16(dp_addr(0xa432), mcu.r[3]);
    mcu.pc = 0x04d1;
}

/* 0x404D1 mov.w r2,@a434. */
void step_b_store_a434(void)
{
    store16(dp_addr(0xa434), mcu.r[2]);
    mcu.pc = 0x04d5;
}

/* 0x404D5 mov.w r1,@a436. */
void step_b_store_a436(void)
{
    store16(dp_addr(0xa436), mcu.r[1]);
    mcu.pc = 0x04d9;
}

/* 0x404D9 movi r1,#0x001b -- sentinel loop count. */
void step_b_reload_slot_count(void)
{
    movi(mcu.r[1], 0x001b);
    mcu.pc = 0x04dc;
}

/* 0x404DC movg #0xff,@r1+0xa3f4. */
void step_b_sentinel_a3f4(void)
{
    mov_imm8(ind_addr(1, 0xa3f4), 0xff);
    mcu.pc = 0x04e1;
}

/* 0x404E1 movg #0xff,@r1+0xa410. */
void step_b_sentinel_a410(void)
{
    mov_imm8(ind_addr(1, 0xa410), 0xff);
    mcu.pc = 0x04e6;
}

/* 0x404E6 clr @r1+0xa3bc. */
void step_b_clear_a3bc(void)
{
    clear8(ind_addr(1, 0xa3bc));
    mcu.pc = 0x04ea;
}

/* 0x404EA movg #0xff,@r1+0xa3d8 -- free-list chain head. */
void step_b_sentinel_a3d8(void)
{
    mov_imm8(ind_addr(1, 0xa3d8), 0xff);
    mcu.pc = 0x04ef;
}

/* 0x404EF cntjmp r1,-22 -> 0x404dc -- loop B sentinels. */
void step_b_loop_sentinels(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x04dcu : 0x04f2u;
}

/* 0x404F2 clr.w (dp,0xa1f0). */
void step_b_clear_a1f0(void)
{
    clear16(dp_addr(0xa1f0));
    mcu.pc = 0x04f6;
}

/* 0x404F6 clr.b (dp,0xa1df). */
void step_b_clear_a1df(void)
{
    clear8(dp_addr(0xa1df));
    mcu.pc = 0x04fa;
}

/* 0x404FA movg #0xff,(dp,0xa42f) -- free-list tail. */
void step_b_seed_a42f(void)
{
    mov_imm8(dp_addr(0xa42f), 0xff);
    mcu.pc = 0x04ff;
}

/* 0x404FF movg #0xff,(dp,0xa430) -- free-list head. */
void step_b_seed_a430(void)
{
    mov_imm8(dp_addr(0xa430), 0xff);
    mcu.pc = 0x0504;
}

/* 0x40504 clr.b (dp,0xa42d) -- free count. */
void step_b_clear_a42d(void)
{
    clear8(dp_addr(0xa42d));
    mcu.pc = 0x0508;
}

/* 0x40508 movi r1,#0x001b -- free-list build count. */
void step_b_reload_free_count(void)
{
    movi(mcu.r[1], 0x001b);
    mcu.pc = 0x050b;
}

/* 0x4050B clr r0 -- list cursor. */
void step_b_clear_r0(void)
{
    mcu.r[0] = 0;
    flags_clr();
    mcu.pc = 0x050d;
}

/* 0x4050D mov.b (dp,0xa430),r0 -- head. */
void step_b_load_free_head(void)
{
    load8(mcu.r[0], dp_addr(0xa430));
    mcu.pc = 0x0511;
}

/* 0x40511 bpl -> 0x4051d -- first entry seeds the head. */
void step_b_test_free_head(void)
{
    mcu.pc = bpl_pool(0x051d, 0x0513);
}

/* 0x40513 mov.b r1,(dp,0xa42f) -- append at the tail. */
void step_b_link_tail(void)
{
    store8(dp_addr(0xa42f), (uint8_t)mcu.r[1]);
    mcu.pc = 0x0517;
}

/* 0x40517 mov.b r0,@r1+0xa3d8 -- link to the previous head. */
void step_b_link_next(void)
{
    store8(ind_addr(1, 0xa3d8), (uint8_t)mcu.r[0]);
    mcu.pc = 0x051b;
}

/* 0x4051B bra -> 0x40526. */
void step_b_link_done(void)
{
    mcu.pc = 0x0526;
}

/* 0x4051D mov.b r1,@r0+0xa3d8 -- link back from the old head. */
void step_b_link_head(void)
{
    store8(ind_addr(0, 0xa3d8), (uint8_t)mcu.r[1]);
    mcu.pc = 0x0521;
}

/* 0x40521 movg #0xff,@r1+0xa3d8 -- terminate the chain. */
void step_b_link_terminate(void)
{
    mov_imm8(ind_addr(1, 0xa3d8), 0xff);
    mcu.pc = 0x0526;
}

/* 0x40526 mov.b r1,(dp,0xa430) -- new head. */
void step_b_store_free_head(void)
{
    store8(dp_addr(0xa430), (uint8_t)mcu.r[1]);
    mcu.pc = 0x052a;
}

/* 0x4052A movg #0x94,@r1+0xa3a0 -- mark the slot free. */
void step_b_mark_free(void)
{
    mov_imm8(ind_addr(1, 0xa3a0), 0x94);
    mcu.pc = 0x052f;
}

/* 0x4052F addq.b #1,(dp,0xa42d) -- free count++. */
void step_b_count_free(void)
{
    addq8(dp_addr(0xa42d), 1);
    mcu.pc = 0x0533;
}

/* 0x40533 cntjmp r1,-41 -> 0x4050d -- loop C free-list build. */
void step_b_loop_free(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x050du : 0x0536u;
}

/* 0x40536 movg #0xff,(dp,0xa42e) -- descriptor-pool head. */
void step_b_seed_a42e(void)
{
    mov_imm8(dp_addr(0xa42e), 0xff);
    mcu.pc = 0x053b;
}

/* 0x4053B mov.b (dp,0xa42e),r0 -- previous head. */
void step_b_load_desc_head(void)
{
    load8(mcu.r[0], dp_addr(0xa42e));
    mcu.pc = 0x053f;
}

/* 0x4053F mov.b r0,@r2+0xa250 -- link the descriptor. */
void step_b_link_desc(void)
{
    store8(ind_addr(2, 0xa250), (uint8_t)mcu.r[0]);
    mcu.pc = 0x0543;
}

/* 0x40543 mov.b r2,(dp,0xa42e) -- new head. */
void step_b_store_desc_head(void)
{
    store8(dp_addr(0xa42e), (uint8_t)mcu.r[2]);
    mcu.pc = 0x0547;
}

/* 0x40547 movg #0x94,@r2+0xa288 -- descriptor active marker. */
void step_b_mark_desc(void)
{
    mov_imm8(ind_addr(2, 0xa288), 0x94);
    mcu.pc = 0x054c;
}

/* 0x4054C movg #0xff,@r2+0xa26c. */
void step_b_desc_a26c(void)
{
    mov_imm8(ind_addr(2, 0xa26c), 0xff);
    mcu.pc = 0x0551;
}

/* 0x40551 movg #0xff,@r2+0xa314. */
void step_b_desc_a314(void)
{
    mov_imm8(ind_addr(2, 0xa314), 0xff);
    mcu.pc = 0x0556;
}

/* 0x40556 movg #0xff,@r2+0xa2c0. */
void step_b_desc_a2c0(void)
{
    mov_imm8(ind_addr(2, 0xa2c0), 0xff);
    mcu.pc = 0x055b;
}

/* 0x4055B movg #0xff,@r2+0xa2dc. */
void step_b_desc_a2dc(void)
{
    mov_imm8(ind_addr(2, 0xa2dc), 0xff);
    mcu.pc = 0x0560;
}

/* 0x40560 cntjmp r2,-40 -> 0x4053b -- loop D descriptors. */
void step_b_loop_desc(void)
{
    mcu.r[2] = (uint16_t)(mcu.r[2] - 1);
    mcu.pc = (mcu.r[2] != 0xffff) ? 0x053bu : 0x0563u;
}

/* 0x40563 clr r2 -- part cursor. */
void step_b_clear_r2(void)
{
    mcu.r[2] = 0;
    flags_clr();
    mcu.pc = 0x0565;
}

/* 0x40565 clr @r3+0xa210. */
void step_b_part_a210(void)
{
    clear8(ind_addr(3, 0xa210));
    mcu.pc = 0x0569;
}

/* 0x40569 movg #0xff,@r3+0xa220 -- part gate. */
void step_b_part_a220(void)
{
    mov_imm8(ind_addr(3, 0xa220), 0xff);
    mcu.pc = 0x056e;
}

/* 0x4056E movg #0xff,@r3+0xa230. */
void step_b_part_a230(void)
{
    mov_imm8(ind_addr(3, 0xa230), 0xff);
    mcu.pc = 0x0573;
}

/* 0x40573 bclr @r3+0xa240,#0. */
void step_b_part_a240(void)
{
    bclr0(ind_addr(3, 0xa240));
    mcu.pc = 0x0577;
}

/* 0x40577 movg #0xff,@r3+0xa200. */
void step_b_part_a200(void)
{
    mov_imm8(ind_addr(3, 0xa200), 0xff);
    mcu.pc = 0x057d;
}

/* 0x4057D mov.b r3,r0 -- seed the previous-part byte. */
void step_b_part_copy(void)
{
    mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | (uint8_t)mcu.r[3]);
    MCU_SetStatusCommon((uint8_t)mcu.r[3], 0);
    mcu.pc = 0x057f;
}

/* 0x4057F addq.b #-1,r3 -- next part. */
void step_b_part_dec(void)
{
    addq8_low(mcu.r[3], -1);
    mcu.pc = 0x0581;
}

/* 0x40581 bpl -> 0x40565 -- loop E over 16 parts. */
void step_b_loop_part(void)
{
    mcu.pc = bpl_pool(0x0565, 0x0583);
}

/* 0x40583 bsr #0x40588 -- part record helper; return via 0x40586. */
void step_b_call_helper(void)
{
    MCU_PushStack(0x0586);
    mcu.pc = 0x0588;
}

/* 0x40588 movi r3,#0x000f -- helper loop counter. */
void step_b_helper_count(void)
{
    movi(mcu.r[3], 0x000f);
    mcu.pc = 0x058b;
}

/* 0x4058B shll.b r3 -- part index * 2. */
void step_b_helper_double(void)
{
    shll8(mcu.r[3]);
    mcu.pc = 0x058d;
}

/* 0x4058D mov.w @r3+0xabde,r4 -- part record word. */
void step_b_load_abde(void)
{
    load16(mcu.r[4], ind_addr(3, 0xabde));
    mcu.pc = 0x0591;
}

/* 0x40591 mov.w r4,@r3+0xa1b0. */
void step_b_store_a1b0(void)
{
    store16(ind_addr(3, 0xa1b0), mcu.r[4]);
    mcu.pc = 0x0595;
}

/* 0x40595 shlr.b r3 -- back to the part index. */
void step_b_helper_halve(void)
{
    shlr8(mcu.r[3]);
    mcu.pc = 0x0597;
}

/* 0x40597 cmp r4,#0x00e0 -- bank boundary. */
void step_b_compare_e0(void)
{
    sub16_nowrite(mcu.r[4], 0x00e0);
    mcu.pc = 0x059a;
}

/* 0x4059A bcc -> 0x405a6 -- word >= 0xe0: second bank. */
void step_b_branch_e0(void)
{
    mcu.pc = bcc_pool(0x05a6, 0x059c);
}

/* 0x4059C ldc #1,r4 -- ep = 1 (bank 1), poll skipped. */
void step_b_ep1(void)
{
    mcu.ep = 1;
    mcu.ex_ignore = 1;
    mcu.pc = 0x059f;
}

/* 0x4059F movg #1,(dp,0xa1f4) -- record bank 1. */
void step_b_set_bank1(void)
{
    store8(dp_addr(0xa1f4), 1);
    mcu.pc = 0x05a4;
}

/* 0x405A4 bra -> 0x405b2. */
void step_b_bank_done(void)
{
    mcu.pc = 0x05b2;
}

/* 0x405A6 ldc #2,r4 -- ep = 2 (bank 2), poll skipped. */
void step_b_ep2(void)
{
    mcu.ep = 2;
    mcu.ex_ignore = 1;
    mcu.pc = 0x05a9;
}

/* 0x405A9 movg #2,(dp,0xa1f4) -- record bank 2. */
void step_b_set_bank2(void)
{
    store8(dp_addr(0xa1f4), 2);
    mcu.pc = 0x05ae;
}

/* 0x405AE sub r4,#0x00e0 -- rebase the record offset. */
void step_b_sub_e0(void)
{
    mcu.r[4] = (uint16_t)MCU_SUB_Common(mcu.r[4], 0x00e0, 0, 1);
    mcu.pc = 0x05b2;
}

/* 0x405B2 mov.b (dp,0xa1f4),r0 -- bank id. */
void step_b_load_bank(void)
{
    load8(mcu.r[0], dp_addr(0xa1f4));
    mcu.pc = 0x05b6;
}

/* 0x405B6 mov.b r0,@r3+0xa43c. */
void step_b_store_a43c(void)
{
    store8(ind_addr(3, 0xa43c), (uint8_t)mcu.r[0]);
    mcu.pc = 0x05ba;
}

/* 0x405BA mov.w r4,r0 -- offset for the scaling multiply. */
void step_b_copy_r4(void)
{
    mcu.r[0] = mcu.r[4];
    MCU_SetStatusCommon(mcu.r[4], 1);
    mcu.pc = 0x05bc;
}

/* 0x405BC mulxu #0xd8,r0:r1 -- offset * 0xd8. */
void step_b_mulxu(void)
{
    mulxu_d8(mcu.r[0], mcu.r[1], mcu.r[0]);
    mcu.pc = 0x05c0;
}

/* 0x405C0 addx #0,r1 -- fold the carry. */
void step_b_carry(void)
{
    mcu.r[1] = (uint16_t)MCU_ADD_Common(mcu.r[1], 0, 0, 1);
    mcu.pc = 0x05c4;
}

/* 0x405C4 shll.b r3 -- part index * 2. */
void step_b_helper_double2(void)
{
    shll8(mcu.r[3]);
    mcu.pc = 0x05c6;
}

/* 0x405C6 mov.w r1,@r3+0xa44c -- scaled base. */
void step_b_store_a44c(void)
{
    store16(ind_addr(3, 0xa44c), mcu.r[1]);
    mcu.pc = 0x05ca;
}

/* 0x405CA shlr.b r3. */
void step_b_helper_halve2(void)
{
    shlr8(mcu.r[3]);
    mcu.pc = 0x05cc;
}

/* 0x405CC mulxu #0xd8,r4:r5 -- rebased offset * 0xd8. */
void step_b_mulxu2(void)
{
    mulxu_d8(mcu.r[4], mcu.r[5], mcu.r[4]);
    mcu.pc = 0x05d0;
}

/* 0x405D0 addx #0,r5 -- fold the carry. */
void step_b_carry2(void)
{
    mcu.r[5] = (uint16_t)MCU_ADD_Common(mcu.r[5], 0, 0, 1);
    mcu.pc = 0x05d4;
}

/* 0x405D4 move r0,#0 -- default mix. */
void step_b_clear_r0b(void)
{
    move8(mcu.r[0], 0);
    mcu.pc = 0x05d6;
}

/* 0x405D6 btsti @(ep:r5+13),#0 -- record flag. */
void step_b_test_bit(void)
{
    btsti0(((uint32_t)mcu.ep << 16) | (uint16_t)(mcu.r[5] + 13));
    mcu.pc = 0x05d9;
}

/* 0x405D9 beq -> 0x405de -- flag clear keeps mix 0. */
void step_b_branch_bit(void)
{
    mcu.pc = beq_pool(0x05de, 0x05dc);
}

/* 0x405DC move r0,#2 -- flagged mix. */
void step_b_set_two(void)
{
    move8(mcu.r[0], 2);
    mcu.pc = 0x05de;
}

/* 0x405DE mov.b r0,@r3+0xa040 -- per-part mix. */
void step_b_store_a040(void)
{
    store8(ind_addr(3, 0xa040), (uint8_t)mcu.r[0]);
    mcu.pc = 0x05e2;
}

/* 0x405E2 movg #0x40,@r3+0xa080. */
void step_b_set_a080(void)
{
    mov_imm8(ind_addr(3, 0xa080), 0x40);
    mcu.pc = 0x05e7;
}

/* 0x405E7 shll.w r3 -- word-index the per-part arrays. */
void step_b_helper_double3(void)
{
    shll16(mcu.r[3]);
    mcu.pc = 0x05e9;
}

/* 0x405E9 movg #0x3c3c,@r3+0xa190. */
void step_b_set_a190(void)
{
    mov_imm16(ind_addr(3, 0xa190), 0x3c3c);
    mcu.pc = 0x05ef;
}

/* 0x405EF shlr.w r3. */
void step_b_helper_halve3(void)
{
    shlr16(mcu.r[3]);
    mcu.pc = 0x05f1;
}

/* 0x405F1 cntjmp r3,-102 -> 0x4058b -- helper loop 1 over 16 parts. */
void step_b_loop_helper(void)
{
    mcu.r[3] = (uint16_t)(mcu.r[3] - 1);
    mcu.pc = (mcu.r[3] != 0xffff) ? 0x058bu : 0x05f4u;
}

/* 0x405F4 movi r3,#0x000f -- helper loop 2 counter. */
void step_b_part2_count(void)
{
    movi(mcu.r[3], 0x000f);
    mcu.pc = 0x05f7;
}

/* 0x405F7 movi r2,#0xa090 -- per-part base. */
void step_b_part2_base(void)
{
    movi(mcu.r[2], 0xa090);
    mcu.pc = 0x05fa;
}

/* 0x405FA clr @r3+0xa060. */
void step_b_part2_a060(void)
{
    clear8(ind_addr(3, 0xa060));
    mcu.pc = 0x05fe;
}

/* 0x405FE movg #0xff,@r3+0xa050. */
void step_b_part2_a050(void)
{
    mov_imm8(ind_addr(3, 0xa050), 0xff);
    mcu.pc = 0x0603;
}

/* 0x40603 movg #0x3c,@r3+0xa070. */
void step_b_part2_a070(void)
{
    mov_imm8(ind_addr(3, 0xa070), 0x3c);
    mcu.pc = 0x0608;
}

/* 0x40608 mov.w r3,r0 -- part index. */
void step_b_part2_copy(void)
{
    mcu.r[0] = mcu.r[3];
    MCU_SetStatusCommon(mcu.r[3], 1);
    mcu.pc = 0x060a;
}

/* 0x4060A shll.w r0 (x4) -- index * 16. */
void step_b_part2_shift1(void)
{
    shll16(mcu.r[0]);
    mcu.pc = 0x060c;
}

/* 0x4060C shll.w r0. */
void step_b_part2_shift2(void)
{
    shll16(mcu.r[0]);
    mcu.pc = 0x060e;
}

/* 0x4060E shll.w r0. */
void step_b_part2_shift3(void)
{
    shll16(mcu.r[0]);
    mcu.pc = 0x0610;
}

/* 0x40610 shll.w r0. */
void step_b_part2_shift4(void)
{
    shll16(mcu.r[0]);
    mcu.pc = 0x0612;
}

/* 0x40612 add #0x9f50,r0 -- per-part clear base. */
void step_b_part2_baseptr(void)
{
    mcu.r[0] = (uint16_t)MCU_ADD_Common(mcu.r[0], 0x9f50, 0, 1);
    mcu.pc = 0x0616;
}

/* 0x40616 movi r1,#7 -- 8 words per part. */
void step_b_part2_inner(void)
{
    movi(mcu.r[1], 7);
    mcu.pc = 0x0619;
}

/* 0x40619 mov.w r0,--r0; clr.w @r0 -- clear one word going down. */
void step_b_part2_clear(void)
{
    mcu.r[0] = (uint16_t)(mcu.r[0] - 2);
    clear16(dp_addr(mcu.r[0]));
    mcu.pc = 0x061b;
}

/* 0x4061B cntjmp r1,-3 -> 0x40619 -- inner clear loop. */
void step_b_part2_loop(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x0619u : 0x061eu;
}

/* 0x4061E movi r1,#0x000f -- hand off to the shared helper tail. */
void step_b_part2_count2(void)
{
    movi(mcu.r[1], 0x000f);
    mcu.pc = 0x0621;
}

/* ---- C 0x40621: fill store (per-PC named entry) --------------------------- */

/* 0x40621 movg #0xff,@r2++ -- 16-byte fill used by the helper loops. */
void step_c_fill_ff(void)
{
    MCU_Write(((uint32_t)mcu.dp << 16) | mcu.r[2], 0xff);
    MCU_SetStatusCommon(0xff, 0);
    mcu.r[2] = (uint16_t)(mcu.r[2] + 1);
    mcu.pc = 0x0624u;
}

/* ---- D 0x40624/0x40627/0x4062a: counter tail (per-PC named entries) ------- */

/* 0x40624 cntjmp r1,-6 -> 0x40621 -- fill loop. */
void step_d_loop_fill(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x0621u : 0x0627u;
}

/* 0x40627 cntjmp r3,-48 -> 0x405fa -- per-part loop. */
void step_d_loop_part(void)
{
    mcu.r[3] = (uint16_t)(mcu.r[3] - 1);
    mcu.pc = (mcu.r[3] != 0xffff) ? 0x05fau : 0x062au;
}

/* 0x4062A rts -- returns to the 0x40586 ret handled by gen. */
void step_d_return(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- E 0x4062b..0x40672: part descriptor scan (per-PC named entries) ------ */

/* 0x4062B clr r2 -- descriptor cursor. */
void step_e_clear_index(void)
{
    mcu.r[2] = 0;
    flags_clr();
    mcu.pc = 0x062d;
}

/* 0x4062D mov.b @r3+0xa220,r2 -- part gate. */
void step_e_load_gate(void)
{
    load8(mcu.r[2], ind_addr(3, 0xa220));
    mcu.pc = 0x0631;
}

/* 0x40631 bmi -> 0x40672 -- gate set: done. */
void step_e_test_gate(void)
{
    mcu.pc = bmi_pool(0x0672, 0x0633);
}

/* 0x40633 mov.w r3,r0 -- part index. */
void step_e_copy_part(void)
{
    mcu.r[0] = mcu.r[3];
    MCU_SetStatusCommon(mcu.r[3], 1);
    mcu.pc = 0x0635;
}

/* 0x40635 shll.b r0 (x4) -- part index * 16. */
void step_e_shift1(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x0637;
}

/* 0x40637 shll.b r0. */
void step_e_shift2(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x0639;
}

/* 0x40639 shll.b r0. */
void step_e_shift3(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x063b;
}

/* 0x4063B shll.b r0. */
void step_e_shift4(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x063d;
}

/* 0x4063D add #0xa090,r0 -- per-part descriptor base. */
void step_e_base(void)
{
    mcu.r[0] = (uint16_t)MCU_ADD_Common(mcu.r[0], 0xa090, 0, 1);
    mcu.pc = 0x0641;
}

/* 0x40641 mov.w r0,r5 -- keep the base. */
void step_e_save_base(void)
{
    mcu.r[5] = mcu.r[0];
    MCU_SetStatusCommon(mcu.r[0], 1);
    mcu.pc = 0x0643;
}

/* 0x40643 cmp @r2+0xa288,#0 -- descriptor active marker. */
void step_e_compare_desc(void)
{
    (void)MCU_SUB_Common((int32_t)MCU_Read(ind_addr(2, 0xa288)), 0, 0, 0);
    mcu.pc = 0x0648;
}

/* 0x40648 bne -> 0x4066c -- inactive descriptor: next. */
void step_e_branch_desc(void)
{
    mcu.pc = bne_pool(0x066c, 0x064a);
}

/* 0x4064A mov.b @r2+0xa314,r1 -- match value. */
void step_e_load_match(void)
{
    load8(mcu.r[1], ind_addr(2, 0xa314));
    mcu.pc = 0x064e;
}

/* 0x4064E movi r4,#0x000f -- 16 entries. */
void step_e_load_count(void)
{
    movi(mcu.r[4], 0x000f);
    mcu.pc = 0x0651;
}

/* 0x40651 mov.w r5,r0 -- descriptor cursor. */
void step_e_copy_base(void)
{
    mcu.r[0] = mcu.r[5];
    MCU_SetStatusCommon(mcu.r[5], 1);
    mcu.pc = 0x0653;
}

/* 0x40653 tst @r0 -- entry active? */
void step_e_test_entry(void)
{
    tst8(ind_addr(0, 0));
    mcu.pc = 0x0655;
}

/* 0x40655 bmi -> 0x40665 -- free entry: insert here. */
void step_e_branch_entry(void)
{
    mcu.pc = bmi_pool(0x0665, 0x0657);
}

/* 0x40657 addq #1,r0; cmp r1,@r0 -- does this entry match? */
void step_e_compare_match(void)
{
    uint32_t addr = ind_addr(0, 0);
    mcu.r[0] = (uint16_t)(mcu.r[0] + 1);
    (void)MCU_SUB_Common((int32_t)mcu.r[1], (int32_t)MCU_Read(addr), 0, 0);
    mcu.pc = 0x0659;
}

/* 0x40659 beq -> 0x40667 -- match: consume. */
void step_e_branch_match(void)
{
    mcu.pc = beq_pool(0x0667, 0x065b);
}

/* 0x4065B sub #1,r4 -- next entry. */
void step_e_dec_count(void)
{
    mcu.r[4] = (uint16_t)MCU_SUB_Common(mcu.r[4], 0x0001, 0, 1);
    mcu.pc = 0x065f;
}

/* 0x4065F tst r4. */
void step_e_test_count(void)
{
    tst16(mcu.r[4]);
    mcu.pc = 0x0661;
}

/* 0x40661 bmi -> 0x4066c -- exhausted: next descriptor. */
void step_e_branch_count(void)
{
    mcu.pc = bmi_pool(0x066c, 0x0663);
}

/* 0x40663 bra -> 0x40653 -- scan the next entry. */
void step_e_loop_entry(void)
{
    mcu.pc = 0x0653;
}

/* 0x40665 addq #1,r0; mov.b r1,@r0 -- write into the free entry. */
void step_e_store_match(void)
{
    uint32_t addr = ind_addr(0, 0);
    mcu.r[0] = (uint16_t)(mcu.r[0] + 1);
    MCU_Write(addr, (uint8_t)mcu.r[1]);
    MCU_SetStatusCommon((uint8_t)mcu.r[1], 0);
    mcu.pc = 0x0667;
}

/* 0x40667 cntjmp r4,-9 -> 0x40665 -- clear the rest. */
void step_e_loop_clear(void)
{
    mcu.r[4] = (uint16_t)(mcu.r[4] - 1);
    mcu.pc = (mcu.r[4] != 0xffff) ? 0x066cu : 0x066au;
}

/* 0x4066A bra -> 0x40672 -- done. */
void step_e_branch_return(void)
{
    mcu.pc = 0x0672;
}

/* 0x4066C mov.b @r2+0xa250,r2 -- next descriptor. */
void step_e_next_desc(void)
{
    load8(mcu.r[2], ind_addr(2, 0xa250));
    mcu.pc = 0x0670;
}

/* 0x40670 bpl -> 0x40643 -- next active descriptor. */
void step_e_loop_desc(void)
{
    mcu.pc = bpl_pool(0x0643, 0x0672);
}

/* 0x40672 ret (pop cp,pc). */
void step_e_return(void)
{
    ret_cp_pc();
}

/* ---- F 0x40674..0x406e2: kill/release (rom2 kill_a twin) ------------------ */

/* 0x40674 extu r3 -- slot index. */
void step_f_extend_slot(void)
{
    extu8(mcu.r[3]);
    mcu.pc = 0x0676;
}

/* 0x40676 mov.w r3,r0. */
void step_f_copy_slot(void)
{
    mcu.r[0] = mcu.r[3];
    MCU_SetStatusCommon(mcu.r[3], 1);
    mcu.pc = 0x0678;
}

/* 0x40678 shll.b r0 (x4) -- slot * 16. */
void step_f_shift1(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x067a;
}

/* 0x4067A shll.b r0. */
void step_f_shift2(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x067c;
}

/* 0x4067C shll.b r0. */
void step_f_shift3(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x067e;
}

/* 0x4067E shll.b r0. */
void step_f_shift4(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x0680;
}

/* 0x40680 add #0xa090,r0 -- per-slot record base. */
void step_f_base(void)
{
    mcu.r[0] = (uint16_t)MCU_ADD_Common(mcu.r[0], 0xa090, 0, 1);
    mcu.pc = 0x0684;
}

/* 0x40684 mov.w r0,r4 -- keep the base. */
void step_f_save_base(void)
{
    mcu.r[4] = mcu.r[0];
    MCU_SetStatusCommon(mcu.r[0], 1);
    mcu.pc = 0x0686;
}

/* 0x40686 clr r2 -- descriptor cursor. */
void step_f_clear_index(void)
{
    mcu.r[2] = 0;
    flags_clr();
    mcu.pc = 0x0688;
}

/* 0x40688 mov.b @r3+0xa220,r2 -- part gate. */
void step_f_load_gate(void)
{
    load8(mcu.r[2], ind_addr(3, 0xa220));
    mcu.pc = 0x068c;
}

/* 0x4068C bmi -> 0x406d7 -- gate set: skip to the fill. */
void step_f_test_gate(void)
{
    mcu.pc = bmi_pool(0x06d7, 0x068e);
}

/* 0x4068E cmp @r2+0xa288,#2 -- descriptor active? */
void step_f_compare_desc(void)
{
    (void)MCU_SUB_Common((int32_t)MCU_Read(ind_addr(2, 0xa288)), 0x02, 0, 0);
    mcu.pc = 0x0693;
}

/* 0x40693 bne -> 0x406d1 -- not active: next descriptor. */
void step_f_branch_desc(void)
{
    mcu.pc = bne_pool(0x06d1, 0x0695);
}

/* 0x40695 movi r1,#0x000f -- 16 entries. */
void step_f_load_count(void)
{
    movi(mcu.r[1], 0x000f);
    mcu.pc = 0x0698;
}

/* 0x40698 mov.w r4,r0 -- entry cursor. */
void step_f_copy_base(void)
{
    mcu.r[0] = mcu.r[4];
    MCU_SetStatusCommon(mcu.r[4], 1);
    mcu.pc = 0x069a;
}

/* 0x4069A tst @r0 -- entry active? */
void step_f_test_entry(void)
{
    tst8(ind_addr(0, 0));
    mcu.pc = 0x069c;
}

/* 0x4069C bmi -> 0x406d1 -- free entry: next descriptor. */
void step_f_branch_entry(void)
{
    mcu.pc = bmi_pool(0x06d1, 0x069e);
}

/* 0x4069E addq #1,r0; mov.b @r0,r5 -- entry value. */
void step_f_load_match(void)
{
    uint32_t addr = ind_addr(0, 0);
    mcu.r[0] = (uint16_t)(mcu.r[0] + 1);
    load8(mcu.r[5], addr);
    mcu.pc = 0x06a0;
}

/* 0x406A0 cmp @r2+0xa314,r5 -- matches the descriptor? */
void step_f_compare_match(void)
{
    (void)MCU_SUB_Common((int32_t)mcu.r[5],
                         (int32_t)MCU_Read(ind_addr(2, 0xa314)), 0, 0);
    mcu.pc = 0x06a4;
}

/* 0x406A4 beq -> 0x406ab -- match: evaluate the kill. */
void step_f_branch_match(void)
{
    mcu.pc = beq_pool(0x06ab, 0x06a6);
}

/* 0x406A6 cntjmp r1,-12 -> 0x4069a -- next entry. */
void step_f_dec_count(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x069au : 0x06a9u;
}

/* 0x406A9 bra -> 0x406d7 -- no match: fill. */
void step_f_branch_kill(void)
{
    mcu.pc = 0x06d7;
}

/* 0x406AB btsti @r2+0xa2a4,#0. */
void step_f_test_flag(void)
{
    btsti0(ind_addr(2, 0xa2a4));
    mcu.pc = 0x06af;
}

/* 0x406AF bne -> 0x406d1 -- flagged: next descriptor. */
void step_f_branch_flag(void)
{
    mcu.pc = bne_pool(0x06d1, 0x06b1);
}

/* 0x406B1 mov.b @r2+0xa2dc,r1 -- voice id. */
void step_f_load_voice(void)
{
    load8(mcu.r[1], ind_addr(2, 0xa2dc));
    mcu.pc = 0x06b5;
}

/* 0x406B5 extu r1. */
void step_f_extend_voice(void)
{
    extu8(mcu.r[1]);
    mcu.pc = 0x06b7;
}

/* 0x406B7 movg #1,@r1+0xa3bc -- mark released. */
void step_f_mark_a3bc(void)
{
    mov_imm8(ind_addr(1, 0xa3bc), 0x01);
    mcu.pc = 0x06bc;
}

/* 0x406BC movg #0xff,@r1+0xa4b4 -- clear the note id. */
void step_f_clear_a4b4(void)
{
    mov_imm8(ind_addr(1, 0xa4b4), 0xff);
    mcu.pc = 0x06c1;
}

/* 0x406C1 mov.b @r1+0xa410,r1 -- next link. */
void step_f_load_state(void)
{
    load8(mcu.r[1], ind_addr(1, 0xa410));
    mcu.pc = 0x06c5;
}

/* 0x406C5 bmi -> 0x406d1 -- chain end: next descriptor. */
void step_f_test_state(void)
{
    mcu.pc = bmi_pool(0x06d1, 0x06c7);
}

/* 0x406C7 movg #1,@r1+0xa3bc -- mark the next voice too. */
void step_f_mark_a3bc2(void)
{
    mov_imm8(ind_addr(1, 0xa3bc), 0x01);
    mcu.pc = 0x06cc;
}

/* 0x406CC movg #0xff,@r1+0xa4b4. */
void step_f_clear_a4b4b(void)
{
    mov_imm8(ind_addr(1, 0xa4b4), 0xff);
    mcu.pc = 0x06d1;
}

/* 0x406D1 mov.b @r2+0xa250,r2 -- next descriptor. */
void step_f_next_desc(void)
{
    load8(mcu.r[2], ind_addr(2, 0xa250));
    mcu.pc = 0x06d5;
}

/* 0x406D5 bpl -> 0x4068e -- next active descriptor. */
void step_f_loop_desc(void)
{
    mcu.pc = bpl_pool(0x068e, 0x06d7);
}

/* 0x406D7 movi r1,#0x000f -- fill count. */
void step_f_load_count2(void)
{
    movi(mcu.r[1], 0x000f);
    mcu.pc = 0x06da;
}

/* 0x406DA mov.w r4,r0 -- record base. */
void step_f_copy_base2(void)
{
    mcu.r[0] = mcu.r[4];
    MCU_SetStatusCommon(mcu.r[4], 1);
    mcu.pc = 0x06dc;
}

/* 0x406DC addq #1,r0; movg #0xff,@r0 -- fill the record. */
void step_f_fill_ff(void)
{
    uint32_t addr = ind_addr(0, 0);
    mcu.r[0] = (uint16_t)(mcu.r[0] + 1);
    MCU_Write(addr, 0xff);
    MCU_SetStatusCommon(0xff, 0);
    mcu.pc = 0x06df;
}

/* 0x406DF cntjmp r1,-3 -> 0x406dc. */
void step_f_loop_fill(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x06dcu : 0x06e2u;
}

/* 0x406E2 ret (pop cp,pc). */
void step_f_return(void)
{
    ret_cp_pc();
}

/* ---- PC tables (rom2 offsets; the flat key adds cp 4) --------------------- */

/* {pc, fn} entry for the per-PC named routines (A/C/D). */
struct PoolPcEnt
{
    uint16_t pc;
    mk2cpp_fn fn;
};

const PoolPcEnt kPoolInitA[] = {
    { 0x0462, &step_a_load_count },    { 0x0465, &step_a_clear_value },
    { 0x0467, &step_a_set_ff },        { 0x0469, &step_a_clear_ce5c },
    { 0x046d, &step_a_clear_ce78 },    { 0x0471, &step_a_clear_ce94 },
    { 0x0475, &step_a_clear_d118 },    { 0x0479, &step_a_clear_d134 },
    { 0x047d, &step_a_clear_ceb0 },    { 0x0481, &step_a_clear_cf20 },
    { 0x0485, &step_a_set_cf3c },      { 0x048a, &step_a_clear_cf90 },
    { 0x048e, &step_a_clear_d0fc },    { 0x0492, &step_a_clear_d0a8 },
    { 0x0496, &step_a_clear_d0c4 },    { 0x049a, &step_a_clear_ad0e },
    { 0x049e, &step_a_clear_ce3f },    { 0x04a2, &step_a_clear_d0e0 },
    { 0x04a6, &step_a_clear_a4b4 },    { 0x04aa, &step_a_clear_a34c },
    { 0x04ae, &step_a_double_cursor }, { 0x04b0, &step_a_store_cfac },
};

const PoolPcEnt kPoolInitB[] = {
    { 0x04b4, &step_b_store_d000 },     { 0x04b8, &step_b_store_d054 },
    { 0x04bc, &step_b_store_a46c },     { 0x04c0, &step_b_halve_cursor },
    { 0x04c2, &step_b_loop_a },         { 0x04c5, &step_b_load_part_count },
    { 0x04c8, &step_b_load_slot_count },{ 0x04cb, &step_b_copy_count },
    { 0x04cd, &step_b_store_a432 },     { 0x04d1, &step_b_store_a434 },
    { 0x04d5, &step_b_store_a436 },     { 0x04d9, &step_b_reload_slot_count },
    { 0x04dc, &step_b_sentinel_a3f4 },  { 0x04e1, &step_b_sentinel_a410 },
    { 0x04e6, &step_b_clear_a3bc },     { 0x04ea, &step_b_sentinel_a3d8 },
    { 0x04ef, &step_b_loop_sentinels }, { 0x04f2, &step_b_clear_a1f0 },
    { 0x04f6, &step_b_clear_a1df },     { 0x04fa, &step_b_seed_a42f },
    { 0x04ff, &step_b_seed_a430 },      { 0x0504, &step_b_clear_a42d },
    { 0x0508, &step_b_reload_free_count},{ 0x050b, &step_b_clear_r0 },
    { 0x050d, &step_b_load_free_head }, { 0x0511, &step_b_test_free_head },
    { 0x0513, &step_b_link_tail },      { 0x0517, &step_b_link_next },
    { 0x051b, &step_b_link_done },      { 0x051d, &step_b_link_head },
    { 0x0521, &step_b_link_terminate }, { 0x0526, &step_b_store_free_head },
    { 0x052a, &step_b_mark_free },      { 0x052f, &step_b_count_free },
    { 0x0533, &step_b_loop_free },      { 0x0536, &step_b_seed_a42e },
    { 0x053b, &step_b_load_desc_head }, { 0x053f, &step_b_link_desc },
    { 0x0543, &step_b_store_desc_head },{ 0x0547, &step_b_mark_desc },
    { 0x054c, &step_b_desc_a26c },      { 0x0551, &step_b_desc_a314 },
    { 0x0556, &step_b_desc_a2c0 },      { 0x055b, &step_b_desc_a2dc },
    { 0x0560, &step_b_loop_desc },      { 0x0563, &step_b_clear_r2 },
    { 0x0565, &step_b_part_a210 },      { 0x0569, &step_b_part_a220 },
    { 0x056e, &step_b_part_a230 },      { 0x0573, &step_b_part_a240 },
    { 0x0577, &step_b_part_a200 },      { 0x057d, &step_b_part_copy },
    { 0x057f, &step_b_part_dec },       { 0x0581, &step_b_loop_part },
    { 0x0583, &step_b_call_helper },    { 0x0588, &step_b_helper_count },
    { 0x058b, &step_b_helper_double },  { 0x058d, &step_b_load_abde },
    { 0x0591, &step_b_store_a1b0 },     { 0x0595, &step_b_helper_halve },
    { 0x0597, &step_b_compare_e0 },     { 0x059a, &step_b_branch_e0 },
    { 0x059c, &step_b_ep1 },            { 0x059f, &step_b_set_bank1 },
    { 0x05a4, &step_b_bank_done },      { 0x05a6, &step_b_ep2 },
    { 0x05a9, &step_b_set_bank2 },      { 0x05ae, &step_b_sub_e0 },
    { 0x05b2, &step_b_load_bank },      { 0x05b6, &step_b_store_a43c },
    { 0x05ba, &step_b_copy_r4 },        { 0x05bc, &step_b_mulxu },
    { 0x05c0, &step_b_carry },          { 0x05c4, &step_b_helper_double2 },
    { 0x05c6, &step_b_store_a44c },     { 0x05ca, &step_b_helper_halve2 },
    { 0x05cc, &step_b_mulxu2 },         { 0x05d0, &step_b_carry2 },
    { 0x05d4, &step_b_clear_r0b },      { 0x05d6, &step_b_test_bit },
    { 0x05d9, &step_b_branch_bit },     { 0x05dc, &step_b_set_two },
    { 0x05de, &step_b_store_a040 },     { 0x05e2, &step_b_set_a080 },
    { 0x05e7, &step_b_helper_double3 }, { 0x05e9, &step_b_set_a190 },
    { 0x05ef, &step_b_helper_halve3 },  { 0x05f1, &step_b_loop_helper },
    { 0x05f4, &step_b_part2_count },    { 0x05f7, &step_b_part2_base },
    { 0x05fa, &step_b_part2_a060 },     { 0x05fe, &step_b_part2_a050 },
    { 0x0603, &step_b_part2_a070 },     { 0x0608, &step_b_part2_copy },
    { 0x060a, &step_b_part2_shift1 },   { 0x060c, &step_b_part2_shift2 },
    { 0x060e, &step_b_part2_shift3 },   { 0x0610, &step_b_part2_shift4 },
    { 0x0612, &step_b_part2_baseptr },  { 0x0616, &step_b_part2_inner },
    { 0x0619, &step_b_part2_clear },    { 0x061b, &step_b_part2_loop },
    { 0x061e, &step_b_part2_count2 },
};

const PoolPcEnt kPoolInitC[] = {
    { 0x0621, &step_c_fill_ff },
};

const PoolPcEnt kPoolInitD[] = {
    { 0x0624, &step_d_loop_fill },
    { 0x0627, &step_d_loop_part },
    { 0x062a, &step_d_return },
};

const PoolPcEnt kPoolScanE[] = {
    { 0x062b, &step_e_clear_index },  { 0x062d, &step_e_load_gate },
    { 0x0631, &step_e_test_gate },    { 0x0633, &step_e_copy_part },
    { 0x0635, &step_e_shift1 },       { 0x0637, &step_e_shift2 },
    { 0x0639, &step_e_shift3 },       { 0x063b, &step_e_shift4 },
    { 0x063d, &step_e_base },         { 0x0641, &step_e_save_base },
    { 0x0643, &step_e_compare_desc }, { 0x0648, &step_e_branch_desc },
    { 0x064a, &step_e_load_match },   { 0x064e, &step_e_load_count },
    { 0x0651, &step_e_copy_base },    { 0x0653, &step_e_test_entry },
    { 0x0655, &step_e_branch_entry }, { 0x0657, &step_e_compare_match },
    { 0x0659, &step_e_branch_match }, { 0x065b, &step_e_dec_count },
    { 0x065f, &step_e_test_count },   { 0x0661, &step_e_branch_count },
    { 0x0663, &step_e_loop_entry },   { 0x0665, &step_e_store_match },
    { 0x0667, &step_e_loop_clear },   { 0x066a, &step_e_branch_return },
    { 0x066c, &step_e_next_desc },    { 0x0670, &step_e_loop_desc },
    { 0x0672, &step_e_return },
};

/* 0x4068e..0x406e2 are also in the gen table; the hand entry wins in
 * MK2CPP_Step, which keeps routine F from being split at 0x4068d. */
const PoolPcEnt kPoolKillF[] = {
    { 0x0674, &step_f_extend_slot },  { 0x0676, &step_f_copy_slot },
    { 0x0678, &step_f_shift1 },       { 0x067a, &step_f_shift2 },
    { 0x067c, &step_f_shift3 },       { 0x067e, &step_f_shift4 },
    { 0x0680, &step_f_base },         { 0x0684, &step_f_save_base },
    { 0x0686, &step_f_clear_index },  { 0x0688, &step_f_load_gate },
    { 0x068c, &step_f_test_gate },    { 0x068e, &step_f_compare_desc },
    { 0x0693, &step_f_branch_desc },  { 0x0695, &step_f_load_count },
    { 0x0698, &step_f_copy_base },    { 0x069a, &step_f_test_entry },
    { 0x069c, &step_f_branch_entry }, { 0x069e, &step_f_load_match },
    { 0x06a0, &step_f_compare_match },{ 0x06a4, &step_f_branch_match },
    { 0x06a6, &step_f_dec_count },    { 0x06a9, &step_f_branch_kill },
    { 0x06ab, &step_f_test_flag },    { 0x06af, &step_f_branch_flag },
    { 0x06b1, &step_f_load_voice },   { 0x06b5, &step_f_extend_voice },
    { 0x06b7, &step_f_mark_a3bc },    { 0x06bc, &step_f_clear_a4b4 },
    { 0x06c1, &step_f_load_state },   { 0x06c5, &step_f_test_state },
    { 0x06c7, &step_f_mark_a3bc2 },   { 0x06cc, &step_f_clear_a4b4b },
    { 0x06d1, &step_f_next_desc },    { 0x06d5, &step_f_loop_desc },
    { 0x06d7, &step_f_load_count2 },  { 0x06da, &step_f_copy_base2 },
    { 0x06dc, &step_f_fill_ff },      { 0x06df, &step_f_loop_fill },
    { 0x06e2, &step_f_return },
};

} /* anonymous namespace */

void pool_post_reset(void)
{
    /* Pool-init is translated from ROM and writes page-0 SRAM directly; there
     * is no native pool copy left to reset. Kept as the HandPostReset hook. */
}

} /* namespace mk2c */

/* ---- hand integration hooks ----------------------------------------------- */

void MK2CPP_PoolPostReset(void)
{
    mk2c::pool_post_reset();
}

void MK2CPP_PoolFillTables(void)
{
    for (uint32_t i = 0; i < (uint32_t)(sizeof(mk2c::kPoolInitA) / sizeof(mk2c::kPoolInitA[0])); i++)
        MK2CPP_HandRegister(0x00040000u | mk2c::kPoolInitA[i].pc, mk2c::kPoolInitA[i].fn);
    for (uint32_t i = 0; i < (uint32_t)(sizeof(mk2c::kPoolInitB) / sizeof(mk2c::kPoolInitB[0])); i++)
        MK2CPP_HandRegister(0x00040000u | mk2c::kPoolInitB[i].pc, mk2c::kPoolInitB[i].fn);
    for (uint32_t i = 0; i < (uint32_t)(sizeof(mk2c::kPoolInitC) / sizeof(mk2c::kPoolInitC[0])); i++)
        MK2CPP_HandRegister(0x00040000u | mk2c::kPoolInitC[i].pc, mk2c::kPoolInitC[i].fn);
    for (uint32_t i = 0; i < (uint32_t)(sizeof(mk2c::kPoolInitD) / sizeof(mk2c::kPoolInitD[0])); i++)
        MK2CPP_HandRegister(0x00040000u | mk2c::kPoolInitD[i].pc, mk2c::kPoolInitD[i].fn);
    for (uint32_t i = 0; i < (uint32_t)(sizeof(mk2c::kPoolScanE) / sizeof(mk2c::kPoolScanE[0])); i++)
        MK2CPP_HandRegister(0x00040000u | mk2c::kPoolScanE[i].pc, mk2c::kPoolScanE[i].fn);
    for (uint32_t i = 0; i < (uint32_t)(sizeof(mk2c::kPoolKillF) / sizeof(mk2c::kPoolKillF[0])); i++)
        MK2CPP_HandRegister(0x00040000u | mk2c::kPoolKillF[i].pc, mk2c::kPoolKillF[i].fn);

    /* Whole-routine alloc/free hooks (native_allocfree.cpp). */
    MK2CPP_AllocFreeFillTables();
}
