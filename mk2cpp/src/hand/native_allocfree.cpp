/*
 * HAND voice/native_allocfree -- alloc/free as per-instruction hand entries.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 2
 *
 * Five rom1 routines (pool_pop 0x19ad, link 0x194c, release A 0x1823,
 * release B 0x187e, free_voice 0x19c4) and their callees (H1 0x1b44,
 * H2 0x1bad, H3 0x1b90, unlink 0x1b23, pcm_start 0x516c) are translated
 * instruction by instruction. Every instruction PC is one MK2CPP_Step entry
 * returning 1; the host polls interrupts, runs the timer and traces between
 * every pair of instructions, exactly like the interpreter.
 *
 * Why no L1 blocks: a multi-instruction block defers an interrupt that stock
 * takes inside it, so the handler's saved resume PC and stack residue differ
 * (and, over a long run, the audio timeline drifts). That was the root cause
 * of the demo340/500M and [350M,550M) audio divergence: only the few block
 * boundaries seen in one run had been split; stock can be interrupted at any
 * IML=0 instruction. Per-instruction entries remove the deferral entirely.
 *
 * ROM topology is preserved: bsr/jsr push the return address and set pc to the
 * target, and the callee's rts pops it. Helpers are shared by every caller and
 * a caller's continuation PC is just the next registered entry (release A's
 * 0x1847 pushes 0x184a and jumps to H1; H1's 0x1b8f rts pops back to 0x184a).
 * The IML=7 window (release A 0x1823..0x1833, release B 0x187e..0x1890) is
 * stepped per instruction too: no maskable interrupt can be taken there either
 * way, and per-instruction stepping reproduces the ex_ignore/TRAPA polling
 * points exactly.
 *
 * All state is the page-0 SRAM the stock code uses (MCU_Read/MCU_Write); the
 * cases are named list/SoA manipulations, not a shadow copy. Semantics and
 * field names: mk2cpp/out/m4/17_alloc_free_semantics.md.
 *
 * PC ranges / static counts (tools/baselines/dasm_full.txt):
 *   pool_pop  0x19ad..0x19c3      7
 *   link      0x194c..0x19ac     25 (0x195c/0x1967 are unexecuted arms)
 *   release A 0x1823..0x187d     27
 *   release B 0x187e..0x18cd     24
 *   free      0x19c4..0x1a23     28
 *   H1        0x1b44..0x1b8f     21
 *   H2        0x1bad..0x1bfd     26
 *   H3        0x1b90..0x1bac      9
 *   unlink    0x1b23..0x1b43     11
 *   pcm_start 0x516c..0x51b5     21
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as pcm_enable.cpp / voice_materialize.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* ---- general-operand effective addresses (src/mcu_opcodes.cpp:543) -------- */

/* @rN+disp: page from dp (r0-r3), ep (r4/r5) or tp (r6/r7), sum mod 0x10000. */
uint32_t ind_addr(uint32_t reg, uint16_t disp)
{
    uint8_t page = (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
    return ((uint32_t)page << 16) | (uint16_t)(mcu.r[reg] + disp);
}

/* (dp,addr16): absolute through the DP page register. */
uint32_t dp_addr(uint16_t disp) { return ((uint32_t)mcu.dp << 16) | disp; }

/* ---- single-instruction operations (GT semantics) ------------------------- */

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG2 @addr -> rN */
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

/* MOVG3 rN -> @addr (write) */
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

/* MOVG #imm -> @addr (MOVG_Immediate ore 6/7) */
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

/* SUB @addr #imm (MOVG_Immediate ore 4 byte): flags only, no write */
void sub8_nowrite(uint32_t addr, uint8_t imm)
{
    MCU_SUB_Common(MCU_Read(addr), imm, 0, 0);
}

/* ADDQ #delta @addr (byte) */
void addq_byte(uint32_t addr, int delta)
{
    int32_t value = MCU_ADD_Common(MCU_Read(addr), delta, 0, 0);
    MCU_Write(addr, (uint8_t)value);
}

/* CLR @addr (byte) / CLR rN (word: operands a8/ab have bit3 set) */
void clr8_mem(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

void clr_reg(uint16_t &reg)
{
    reg = 0;
    flags_clr();
}

/* TST @addr (byte): N/Z from value, C cleared, V=0, no write */
void tst8_mem(uint32_t addr)
{
    MCU_SetStatusCommon(MCU_Read(addr), 0);
    MCU_SetStatus(0, STATUS_C);
}

/* BTSTI @addr #bit: Z = (bit == 0), other flags untouched */
void btsti(uint32_t addr, uint8_t bit)
{
    MCU_SetStatus((MCU_Read(addr) & (1u << bit)) == 0, STATUS_Z);
}

/* BCLR @addr #bit: Z = (bit == 0), then clear the bit */
void bclr_mem(uint32_t addr, uint8_t bit)
{
    uint8_t data = MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    MCU_Write(addr, (uint8_t)(data & ~(1u << bit)));
}

/* CMP @addr rN: t1 = rN, t2 = memory */
void cmp8_mem(uint16_t reg, uint32_t addr)
{
    MCU_SUB_Common((uint8_t)reg, MCU_Read(addr), 0, 0);
}

/* CMP rS rD: t1 = rD (opcode register), t2 = rS (operand) */
void cmp16_reg(uint16_t dst, uint16_t src)
{
    MCU_SUB_Common(dst, src, 0, 1);
}

/* ADD rD rD (word) */
void add16_self(uint16_t &reg)
{
    reg = (uint16_t)MCU_ADD_Common(reg, reg, 0, 1);
}

/* EXTU rN: zero-extend the low byte, N=0, Z=(value==0), V=0, C=0 */
void extu(uint16_t &reg)
{
    uint16_t value = (uint8_t)reg;
    reg = value;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(value == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* Short_MOVI / Short_MOVE helpers */
void movi16(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

void move8(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

/* BSET_ORC #0x0700 / BCLR_ANDC #0xf8ff on CCR (IML window + ex_ignore) */
void bset_orc_iml7(void)
{
    mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
    mcu.ex_ignore = 1;
}

void bclr_andc_iml0(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
}

/* STM #0x3e / LDM #0x3e: push r5,r4,r3,r2,r1 / pop r1..r5 */
void stm_3e(void)
{
    MCU_PushStack(mcu.r[5]);
    MCU_PushStack(mcu.r[4]);
    MCU_PushStack(mcu.r[3]);
    MCU_PushStack(mcu.r[2]);
    MCU_PushStack(mcu.r[1]);
}

void ldm_3e(void)
{
    mcu.r[1] = MCU_PopStack();
    mcu.r[2] = MCU_PopStack();
    mcu.r[3] = MCU_PopStack();
    mcu.r[4] = MCU_PopStack();
    mcu.r[5] = MCU_PopStack();
}

/* Conditional branch targets (GT MCU_Jump_Bcc, src/mcu_opcodes.cpp:231). */
uint16_t bpl(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? fall : taken; }
uint16_t bmi(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? taken : fall; }
uint16_t bne(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? fall : taken; }
uint16_t beq(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? taken : fall; }
uint16_t bcc(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_C) ? fall : taken; }
uint16_t bhi(uint16_t taken, uint16_t fall)
{
    return (mcu.sr & (STATUS_C | STATUS_Z)) ? fall : taken;
}

/* Safety net: a PC missing from the tables below is executed by the stock
 * interpreter for that one instruction (should never happen). */
void stock_instruction(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
}

/* ---- pool_pop 0x19ad..0x19c3 --------------------------------------------- */

uint32_t step_pool_pop(void)
{
    switch (mcu.pc)
    {
    case 0x19ad: /* MOVG2 (dp,0xa42f) r1 */
        load8(mcu.r[1], dp_addr(0xa42f));
        mcu.pc = 0x19b1;
        return 1;
    case 0x19b1: /* MOVG2 @r1+0xa3d8 r0 */
        load8(mcu.r[0], ind_addr(1, 0xa3d8));
        mcu.pc = 0x19b5;
        return 1;
    case 0x19b5: /* MOVG3 r0 -> (dp,0xa42f) */
        store8(dp_addr(0xa42f), (uint8_t)mcu.r[0]);
        mcu.pc = 0x19b9;
        return 1;
    case 0x19b9: /* BPL 0x19bf: list not empty */
        mcu.pc = bpl(0x19bf, 0x19bb);
        return 1;
    case 0x19bb: /* MOVG3 r0 -> (dp,0xa430) */
        store8(dp_addr(0xa430), (uint8_t)mcu.r[0]);
        mcu.pc = 0x19bf;
        return 1;
    case 0x19bf: /* ADDQ #-1 (dp,0xa42d) */
        addq_byte(dp_addr(0xa42d), -1);
        mcu.pc = 0x19c3;
        return 1;
    case 0x19c3: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- link 0x194c..0x19ac -------------------------------------------------- */

uint32_t step_link(void)
{
    switch (mcu.pc)
    {
    case 0x194c: /* MOVG2 @r2+0xa2dc r0 (chain tail) */
        load8(mcu.r[0], ind_addr(2, 0xa2dc));
        mcu.pc = 0x1950;
        return 1;
    case 0x1950: /* BPL 0x1978: chain empty */
        mcu.pc = bpl(0x1978, 0x1952);
        return 1;
    case 0x1952: /* MOVG3 r1 -> @r2+0xa2c0 (new head) */
        store8(ind_addr(2, 0xa2c0), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1956;
        return 1;
    case 0x1956: /* MOVG2 @r1+0xd0a8 r0 (start-prev) */
        load8(mcu.r[0], ind_addr(1, 0xd0a8));
        mcu.pc = 0x195a;
        return 1;
    case 0x195a: /* BMI 0x1961: no start-prev neighbour */
        mcu.pc = bmi(0x1961, 0x195c);
        return 1;
    case 0x195c: /* MOVG #0xff -> @r0+0xd0c4 (unexecuted arm) */
        mov_imm8(ind_addr(0, 0xd0c4), 0xff);
        mcu.pc = 0x1961;
        return 1;
    case 0x1961: /* MOVG2 @r1+0xd0c4 r0 (start-next) */
        load8(mcu.r[0], ind_addr(1, 0xd0c4));
        mcu.pc = 0x1965;
        return 1;
    case 0x1965: /* BMI 0x196c: no start-next neighbour */
        mcu.pc = bmi(0x196c, 0x1967);
        return 1;
    case 0x1967: /* MOVG #0xff -> @r0+0xd0a8 (unexecuted arm) */
        mov_imm8(ind_addr(0, 0xd0a8), 0xff);
        mcu.pc = 0x196c;
        return 1;
    case 0x196c: /* MOVG #0xff -> @r1+0xa410 */
        mov_imm8(ind_addr(1, 0xa410), 0xff);
        mcu.pc = 0x1971;
        return 1;
    case 0x1971: /* MOVG #0xff -> @r1+0xd0a8 */
        mov_imm8(ind_addr(1, 0xd0a8), 0xff);
        mcu.pc = 0x1976;
        return 1;
    case 0x1976: /* BRA 0x1996 */
        mcu.pc = 0x1996;
        return 1;
    case 0x1978: /* MOVG3 r0 -> @r2+0xa2c0 (head = old tail) */
        store8(ind_addr(2, 0xa2c0), (uint8_t)mcu.r[0]);
        mcu.pc = 0x197c;
        return 1;
    case 0x197c: /* MOVG3 r1 -> @r0+0xa3f4 */
        store8(ind_addr(0, 0xa3f4), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1980;
        return 1;
    case 0x1980: /* MOVG3 r1 -> @r0+0xd0c4 */
        store8(ind_addr(0, 0xd0c4), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1984;
        return 1;
    case 0x1984: /* MOVG3 r0 -> @r1+0xa410 */
        store8(ind_addr(1, 0xa410), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1988;
        return 1;
    case 0x1988: /* MOVG3 r0 -> @r1+0xd0a8 */
        store8(ind_addr(1, 0xd0a8), (uint8_t)mcu.r[0]);
        mcu.pc = 0x198c;
        return 1;
    case 0x198c: /* MOVG #0xff -> @r0+0xa410 */
        mov_imm8(ind_addr(0, 0xa410), 0xff);
        mcu.pc = 0x1991;
        return 1;
    case 0x1991: /* MOVG #0xff -> @r0+0xd0a8 */
        mov_imm8(ind_addr(0, 0xd0a8), 0xff);
        mcu.pc = 0x1996;
        return 1;
    case 0x1996: /* MOVG3 r1 -> @r2+0xa2dc (new tail) */
        store8(ind_addr(2, 0xa2dc), (uint8_t)mcu.r[1]);
        mcu.pc = 0x199a;
        return 1;
    case 0x199a: /* MOVG #0xff -> @r1+0xa3f4 */
        mov_imm8(ind_addr(1, 0xa3f4), 0xff);
        mcu.pc = 0x199f;
        return 1;
    case 0x199f: /* MOVG #0xff -> @r1+0xd0c4 */
        mov_imm8(ind_addr(1, 0xd0c4), 0xff);
        mcu.pc = 0x19a4;
        return 1;
    case 0x19a4: /* MOVG3 r3 -> @r1+0xa368 (slot -> part) */
        store8(ind_addr(1, 0xa368), (uint8_t)mcu.r[3]);
        mcu.pc = 0x19a8;
        return 1;
    case 0x19a8: /* MOVG3 r2 -> @r1+0xa384 (slot -> desc) */
        store8(ind_addr(1, 0xa384), (uint8_t)mcu.r[2]);
        mcu.pc = 0x19ac;
        return 1;
    case 0x19ac: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- unlink 0x1b23..0x1b43 (desc out of the part chain) ------------------- */

uint32_t step_unlink(void)
{
    switch (mcu.pc)
    {
    case 0x1b23: /* MOVG2 @r2+0xa250 r0 (next) */
        load8(mcu.r[0], ind_addr(2, 0xa250));
        mcu.pc = 0x1b27;
        return 1;
    case 0x1b27: /* MOVG2 @r2+0xa26c r1 (prev) */
        load8(mcu.r[1], ind_addr(2, 0xa26c));
        mcu.pc = 0x1b2b;
        return 1;
    case 0x1b2b: /* BPL 0x1b33: has prev */
        mcu.pc = bpl(0x1b33, 0x1b2d);
        return 1;
    case 0x1b2d: /* MOVG3 r0 -> @r3+0xa220 (part head = next) */
        store8(ind_addr(3, 0xa220), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1b31;
        return 1;
    case 0x1b31: /* BRA 0x1b37 */
        mcu.pc = 0x1b37;
        return 1;
    case 0x1b33: /* MOVG3 r0 -> @r1+0xa250 (prev.next = next) */
        store8(ind_addr(1, 0xa250), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1b37;
        return 1;
    case 0x1b37: /* BPL 0x1b3f: second branch tests `next` (store flags) */
        mcu.pc = bpl(0x1b3f, 0x1b39);
        return 1;
    case 0x1b39: /* MOVG3 r1 -> @r3+0xa230 (part tail = prev) */
        store8(ind_addr(3, 0xa230), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1b3d;
        return 1;
    case 0x1b3d: /* BRA 0x1b43 */
        mcu.pc = 0x1b43;
        return 1;
    case 0x1b3f: /* MOVG3 r1 -> @r0+0xa26c (next.prev = prev) */
        store8(ind_addr(0, 0xa26c), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1b43;
        return 1;
    case 0x1b43: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- H1 0x1b44..0x1b8f (voice out of its descriptor chain) ---------------- */

uint32_t step_h1(void)
{
    switch (mcu.pc)
    {
    case 0x1b44: /* MOVG3 r3 -> --r7 (save r3) */
        MCU_PushStack(mcu.r[3]);
        mcu.pc = 0x1b46;
        return 1;
    case 0x1b46: /* CLR r3 */
        clr_reg(mcu.r[3]);
        mcu.pc = 0x1b48;
        return 1;
    case 0x1b48: /* MOVG2 @r1+0xa3f4 r3 (prev) */
        load8(mcu.r[3], ind_addr(1, 0xa3f4));
        mcu.pc = 0x1b4c;
        return 1;
    case 0x1b4c: /* BMI 0x1b68: no prev */
        mcu.pc = bmi(0x1b68, 0x1b4e);
        return 1;
    case 0x1b4e: /* MOVG3 r3 -> @r2+0xa2c0 (vhead = prev) */
        store8(ind_addr(2, 0xa2c0), (uint8_t)mcu.r[3]);
        mcu.pc = 0x1b52;
        return 1;
    case 0x1b52: /* MOVG #0xff -> @r3+0xa410 */
        mov_imm8(ind_addr(3, 0xa410), 0xff);
        mcu.pc = 0x1b57;
        return 1;
    case 0x1b57: /* MOVG #0xff -> @r3+0xd0a8 */
        mov_imm8(ind_addr(3, 0xd0a8), 0xff);
        mcu.pc = 0x1b5c;
        return 1;
    case 0x1b5c: /* MOVG #0xff -> @r1+0xa3f4 */
        mov_imm8(ind_addr(1, 0xa3f4), 0xff);
        mcu.pc = 0x1b61;
        return 1;
    case 0x1b61: /* MOVG #0xff -> @r1+0xd0c4 */
        mov_imm8(ind_addr(1, 0xd0c4), 0xff);
        mcu.pc = 0x1b66;
        return 1;
    case 0x1b66: /* BRA 0x1b8d (a2dc stays untouched) */
        mcu.pc = 0x1b8d;
        return 1;
    case 0x1b68: /* MOVG2 @r1+0xa410 r3 (next) */
        load8(mcu.r[3], ind_addr(1, 0xa410));
        mcu.pc = 0x1b6c;
        return 1;
    case 0x1b6c: /* BPL 0x1b75: has next */
        mcu.pc = bpl(0x1b75, 0x1b6e);
        return 1;
    case 0x1b6e: /* MOVG #0xff -> @r2+0xa2c0 (sole node: clear head) */
        mov_imm8(ind_addr(2, 0xa2c0), 0xff);
        mcu.pc = 0x1b73;
        return 1;
    case 0x1b73: /* BRA 0x1b89 */
        mcu.pc = 0x1b89;
        return 1;
    case 0x1b75: /* MOVG #0xff -> @r1+0xa410 */
        mov_imm8(ind_addr(1, 0xa410), 0xff);
        mcu.pc = 0x1b7a;
        return 1;
    case 0x1b7a: /* MOVG #0xff -> @r1+0xd0a8 */
        mov_imm8(ind_addr(1, 0xd0a8), 0xff);
        mcu.pc = 0x1b7f;
        return 1;
    case 0x1b7f: /* MOVG #0xff -> @r3+0xa3f4 */
        mov_imm8(ind_addr(3, 0xa3f4), 0xff);
        mcu.pc = 0x1b84;
        return 1;
    case 0x1b84: /* MOVG #0xff -> @r3+0xd0c4 */
        mov_imm8(ind_addr(3, 0xd0c4), 0xff);
        mcu.pc = 0x1b89;
        return 1;
    case 0x1b89: /* MOVG3 r3 -> @r2+0xa2dc (vtail = next / 0xff) */
        store8(ind_addr(2, 0xa2dc), (uint8_t)mcu.r[3]);
        mcu.pc = 0x1b8d;
        return 1;
    case 0x1b8d: /* MOVG2 r7++ r3 (restore r3) */
        mcu.r[3] = MCU_PopStack();
        mcu.pc = 0x1b8f;
        return 1;
    case 0x1b8f: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- release A 0x1823..0x187d / release B 0x187e..0x18cd ------------------ */

uint32_t step_release(void)
{
    switch (mcu.pc)
    {
    /* ---- release A ---- */
    case 0x1823: /* BSET_ORC #0x0700 r0: IML=7 */
        bset_orc_iml7();
        mcu.pc = 0x1827;
        return 1;
    case 0x1827: /* stm #0x3e (save r1..r5) */
        stm_3e();
        mcu.pc = 0x1829;
        return 1;
    case 0x1829: /* jsr #0x516c (pcm_start) */
        MCU_PushStack(0x182c);
        mcu.pc = 0x516c;
        return 1;
    case 0x182c: /* ldm #0x3e (restore r1..r5) */
        ldm_3e();
        mcu.pc = 0x182e;
        return 1;
    case 0x182e: /* MOVG #0x04 -> @r1+0xd0e0 (cmd = stop) */
        mov_imm8(ind_addr(1, 0xd0e0), 0x04);
        mcu.pc = 0x1833;
        return 1;
    case 0x1833: /* BCLR_ANDC #0xf8ff r0: IML=0 */
        bclr_andc_iml0();
        mcu.pc = 0x1837;
        return 1;
    case 0x1837: /* EXTU r1 */
        extu(mcu.r[1]);
        mcu.pc = 0x1839;
        return 1;
    case 0x1839: /* CLR @r1+0xad0e */
        clr8_mem(ind_addr(1, 0xad0e));
        mcu.pc = 0x183d;
        return 1;
    case 0x183d: /* CLR @r1+0xa3bc */
        clr8_mem(ind_addr(1, 0xa3bc));
        mcu.pc = 0x1841;
        return 1;
    case 0x1841: /* CLR @r1+0xa4b4 */
        clr8_mem(ind_addr(1, 0xa4b4));
        mcu.pc = 0x1845;
        return 1;
    case 0x1845: /* CLR r0 */
        clr_reg(mcu.r[0]);
        mcu.pc = 0x1847;
        return 1;
    case 0x1847: /* bsr16 -> 0x1b44 (H1) */
        MCU_PushStack(0x184a);
        mcu.pc = 0x1b44;
        return 1;
    case 0x184a: /* MOVG2 (dp,0xa430) r0 (free tail) */
        load8(mcu.r[0], dp_addr(0xa430));
        mcu.pc = 0x184e;
        return 1;
    case 0x184e: /* BPL 0x185a: tail valid */
        mcu.pc = bpl(0x185a, 0x1850);
        return 1;
    case 0x1850: /* MOVG3 r1 -> (dp,0xa42f) (empty list: head = slot) */
        store8(dp_addr(0xa42f), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1854;
        return 1;
    case 0x1854: /* MOVG3 r0 -> @r1+0xa3d8 */
        store8(ind_addr(1, 0xa3d8), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1858;
        return 1;
    case 0x1858: /* BRA 0x1863 */
        mcu.pc = 0x1863;
        return 1;
    case 0x185a: /* MOVG3 r1 -> @r0+0xa3d8 (old tail.next = slot) */
        store8(ind_addr(0, 0xa3d8), (uint8_t)mcu.r[1]);
        mcu.pc = 0x185e;
        return 1;
    case 0x185e: /* MOVG #0xff -> @r1+0xa3d8 */
        mov_imm8(ind_addr(1, 0xa3d8), 0xff);
        mcu.pc = 0x1863;
        return 1;
    case 0x1863: /* MOVG3 r1 -> (dp,0xa430) (tail = slot) */
        store8(dp_addr(0xa430), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1867;
        return 1;
    case 0x1867: /* MOVG #0x94 -> @r1+0xa3a0 (free) */
        mov_imm8(ind_addr(1, 0xa3a0), 0x94);
        mcu.pc = 0x186c;
        return 1;
    case 0x186c: /* ADDQ #1 (dp,0xa42d) (count++) */
        addq_byte(dp_addr(0xa42d), 1);
        mcu.pc = 0x1870;
        return 1;
    case 0x1870: /* bsr16 -> 0x1bad (H2) */
        MCU_PushStack(0x1873);
        mcu.pc = 0x1bad;
        return 1;
    case 0x1873: /* ADDQ #-1 (dp,0xa42c) (shortfall--) */
        addq_byte(dp_addr(0xa42c), -1);
        mcu.pc = 0x1877;
        return 1;
    case 0x1877: /* BPL 0x187d: shortfall >= 0 */
        mcu.pc = bpl(0x187d, 0x1879);
        return 1;
    case 0x1879: /* CLR (dp,0xa42c) (clamp to 0) */
        clr8_mem(dp_addr(0xa42c));
        mcu.pc = 0x187d;
        return 1;
    case 0x187d: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;

    /* ---- release B ---- */
    case 0x187e: /* EXTU r1 */
        extu(mcu.r[1]);
        mcu.pc = 0x1880;
        return 1;
    case 0x1880: /* BSET_ORC #0x0700 r0: IML=7 */
        bset_orc_iml7();
        mcu.pc = 0x1884;
        return 1;
    case 0x1884: /* stm #0x3e */
        stm_3e();
        mcu.pc = 0x1886;
        return 1;
    case 0x1886: /* jsr #0x516c (pcm_start) */
        MCU_PushStack(0x1889);
        mcu.pc = 0x516c;
        return 1;
    case 0x1889: /* ldm #0x3e */
        ldm_3e();
        mcu.pc = 0x188b;
        return 1;
    case 0x188b: /* MOVG #0x04 -> @r1+0xd0e0 */
        mov_imm8(ind_addr(1, 0xd0e0), 0x04);
        mcu.pc = 0x1890;
        return 1;
    case 0x1890: /* BCLR_ANDC #0xf8ff r0: IML=0 */
        bclr_andc_iml0();
        mcu.pc = 0x1894;
        return 1;
    case 0x1894: /* CLR @r1+0xad0e */
        clr8_mem(ind_addr(1, 0xad0e));
        mcu.pc = 0x1898;
        return 1;
    case 0x1898: /* CLR @r1+0xa3bc */
        clr8_mem(ind_addr(1, 0xa3bc));
        mcu.pc = 0x189c;
        return 1;
    case 0x189c: /* CLR @r1+0xa4b4 */
        clr8_mem(ind_addr(1, 0xa4b4));
        mcu.pc = 0x18a0;
        return 1;
    case 0x18a0: /* CLR r0 */
        clr_reg(mcu.r[0]);
        mcu.pc = 0x18a2;
        return 1;
    case 0x18a2: /* bsr16 -> 0x1b44 (H1) */
        MCU_PushStack(0x18a5);
        mcu.pc = 0x1b44;
        return 1;
    case 0x18a5: /* MOVG2 (dp,0xa42f) r0 (free head) */
        load8(mcu.r[0], dp_addr(0xa42f));
        mcu.pc = 0x18a9;
        return 1;
    case 0x18a9: /* BPL 0x18af: head valid */
        mcu.pc = bpl(0x18af, 0x18ab);
        return 1;
    case 0x18ab: /* MOVG3 r1 -> (dp,0xa430) (empty list: tail = slot) */
        store8(dp_addr(0xa430), (uint8_t)mcu.r[1]);
        mcu.pc = 0x18af;
        return 1;
    case 0x18af: /* MOVG3 r0 -> @r1+0xa3d8 (slot.next = old head) */
        store8(ind_addr(1, 0xa3d8), (uint8_t)mcu.r[0]);
        mcu.pc = 0x18b3;
        return 1;
    case 0x18b3: /* MOVG3 r1 -> (dp,0xa42f) (head = slot) */
        store8(dp_addr(0xa42f), (uint8_t)mcu.r[1]);
        mcu.pc = 0x18b7;
        return 1;
    case 0x18b7: /* MOVG #0x94 -> @r1+0xa3a0 (free) */
        mov_imm8(ind_addr(1, 0xa3a0), 0x94);
        mcu.pc = 0x18bc;
        return 1;
    case 0x18bc: /* ADDQ #1 (dp,0xa42d) */
        addq_byte(dp_addr(0xa42d), 1);
        mcu.pc = 0x18c0;
        return 1;
    case 0x18c0: /* bsr16 -> 0x1bad (H2) */
        MCU_PushStack(0x18c3);
        mcu.pc = 0x1bad;
        return 1;
    case 0x18c3: /* ADDQ #-1 (dp,0xa42c) */
        addq_byte(dp_addr(0xa42c), -1);
        mcu.pc = 0x18c7;
        return 1;
    case 0x18c7: /* BPL 0x18cd */
        mcu.pc = bpl(0x18cd, 0x18c9);
        return 1;
    case 0x18c9: /* CLR (dp,0xa42c) */
        clr8_mem(dp_addr(0xa42c));
        mcu.pc = 0x18cd;
        return 1;
    case 0x18cd: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- free_voice 0x19c4..0x1a23 -------------------------------------------- */

uint32_t step_free(void)
{
    switch (mcu.pc)
    {
    case 0x19c4: /* BTSTI @r1+0xa3a0 #7 (already free?) */
        btsti(ind_addr(1, 0xa3a0), 7);
        mcu.pc = 0x19c8;
        return 1;
    case 0x19c8: /* BNE 0x1a23: already free */
        mcu.pc = bne(0x1a23, 0x19ca);
        return 1;
    case 0x19ca: /* SUB @r1+0xad0e #0xff (channel assigned?) */
        sub8_nowrite(ind_addr(1, 0xad0e), 0xff);
        mcu.pc = 0x19cf;
        return 1;
    case 0x19cf: /* BEQ 0x1a23: no channel */
        mcu.pc = beq(0x1a23, 0x19d1);
        return 1;
    case 0x19d1: /* MOVG2 @r1+0xa384 r2 (desc) */
        load8(mcu.r[2], ind_addr(1, 0xa384));
        mcu.pc = 0x19d5;
        return 1;
    case 0x19d5: /* MOVG2 @r1+0xa368 r3 (part) */
        load8(mcu.r[3], ind_addr(1, 0xa368));
        mcu.pc = 0x19d9;
        return 1;
    case 0x19d9: /* CLR @r1+0xa3bc */
        clr8_mem(ind_addr(1, 0xa3bc));
        mcu.pc = 0x19dd;
        return 1;
    case 0x19dd: /* CLR @r1+0xa4b4 */
        clr8_mem(ind_addr(1, 0xa4b4));
        mcu.pc = 0x19e1;
        return 1;
    case 0x19e1: /* CMP @r2+0xa2c0 r1 (slot == vhead?) */
        cmp8_mem(mcu.r[1], ind_addr(2, 0xa2c0));
        mcu.pc = 0x19e5;
        return 1;
    case 0x19e5: /* BNE 0x19eb */
        mcu.pc = bne(0x19eb, 0x19e7);
        return 1;
    case 0x19e7: /* BCLR @r2+0xa34c #1 */
        bclr_mem(ind_addr(2, 0xa34c), 1);
        mcu.pc = 0x19eb;
        return 1;
    case 0x19eb: /* CMP @r2+0xa2dc r1 (slot == vtail?) */
        cmp8_mem(mcu.r[1], ind_addr(2, 0xa2dc));
        mcu.pc = 0x19ef;
        return 1;
    case 0x19ef: /* BNE 0x19f5 */
        mcu.pc = bne(0x19f5, 0x19f1);
        return 1;
    case 0x19f1: /* BCLR @r2+0xa34c #0 */
        bclr_mem(ind_addr(2, 0xa34c), 0);
        mcu.pc = 0x19f5;
        return 1;
    case 0x19f5: /* CLR r0 */
        clr_reg(mcu.r[0]);
        mcu.pc = 0x19f7;
        return 1;
    case 0x19f7: /* bsr16 -> 0x1b44 (H1) */
        MCU_PushStack(0x19fa);
        mcu.pc = 0x1b44;
        return 1;
    case 0x19fa: /* MOVG2 (dp,0xa430) r0 (free tail) */
        load8(mcu.r[0], dp_addr(0xa430));
        mcu.pc = 0x19fe;
        return 1;
    case 0x19fe: /* BPL 0x1a0a */
        mcu.pc = bpl(0x1a0a, 0x1a00);
        return 1;
    case 0x1a00: /* MOVG3 r1 -> (dp,0xa42f) (empty list) */
        store8(dp_addr(0xa42f), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1a04;
        return 1;
    case 0x1a04: /* MOVG3 r0 -> @r1+0xa3d8 */
        store8(ind_addr(1, 0xa3d8), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1a08;
        return 1;
    case 0x1a08: /* BRA 0x1a13 */
        mcu.pc = 0x1a13;
        return 1;
    case 0x1a0a: /* MOVG3 r1 -> @r0+0xa3d8 */
        store8(ind_addr(0, 0xa3d8), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1a0e;
        return 1;
    case 0x1a0e: /* MOVG #0xff -> @r1+0xa3d8 */
        mov_imm8(ind_addr(1, 0xa3d8), 0xff);
        mcu.pc = 0x1a13;
        return 1;
    case 0x1a13: /* MOVG3 r1 -> (dp,0xa430) */
        store8(dp_addr(0xa430), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1a17;
        return 1;
    case 0x1a17: /* MOVG #0x94 -> @r1+0xa3a0 */
        mov_imm8(ind_addr(1, 0xa3a0), 0x94);
        mcu.pc = 0x1a1c;
        return 1;
    case 0x1a1c: /* ADDQ #1 (dp,0xa42d) */
        addq_byte(dp_addr(0xa42d), 1);
        mcu.pc = 0x1a20;
        return 1;
    case 0x1a20: /* bsr16 -> 0x1bad (H2) */
        MCU_PushStack(0x1a23);
        mcu.pc = 0x1bad;
        return 1;
    case 0x1a23: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- H2 0x1bad..0x1bfd (descriptor finish) -------------------------------- */

uint32_t step_h2(void)
{
    switch (mcu.pc)
    {
    case 0x1bad: /* TST @r2+0xa2c0 (voice chain empty?) */
        tst8_mem(ind_addr(2, 0xa2c0));
        mcu.pc = 0x1bb1;
        return 1;
    case 0x1bb1: /* BPL 0x1bf9: chain still busy, only count */
        mcu.pc = bpl(0x1bf9, 0x1bb3);
        return 1;
    case 0x1bb3: /* bsr16 -> 0x1b23 (unlink desc) */
        MCU_PushStack(0x1bb6);
        mcu.pc = 0x1b23;
        return 1;
    case 0x1bb6: /* MOVG2 (dp,0xa42e) r0 (old desc head) */
        load8(mcu.r[0], dp_addr(0xa42e));
        mcu.pc = 0x1bba;
        return 1;
    case 0x1bba: /* MOVG3 r0 -> @r2+0xa250 (desc.next = old head) */
        store8(ind_addr(2, 0xa250), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1bbe;
        return 1;
    case 0x1bbe: /* MOVG3 r2 -> (dp,0xa42e) (head = desc) */
        store8(dp_addr(0xa42e), (uint8_t)mcu.r[2]);
        mcu.pc = 0x1bc2;
        return 1;
    case 0x1bc2: /* MOVG #0x94 -> @r2+0xa288 (desc free) */
        mov_imm8(ind_addr(2, 0xa288), 0x94);
        mcu.pc = 0x1bc7;
        return 1;
    case 0x1bc7: /* TST @r3+0xa200 (part current desc) */
        tst8_mem(ind_addr(3, 0xa200));
        mcu.pc = 0x1bcb;
        return 1;
    case 0x1bcb: /* BMI 0x1bf9: no current desc */
        mcu.pc = bmi(0x1bf9, 0x1bcd);
        return 1;
    case 0x1bcd: /* CMP @r3+0xa200 r2 (a200 == desc?) */
        cmp8_mem(mcu.r[2], ind_addr(3, 0xa200));
        mcu.pc = 0x1bd1;
        return 1;
    case 0x1bd1: /* BNE 0x1bf9 */
        mcu.pc = bne(0x1bf9, 0x1bd3);
        return 1;
    case 0x1bd3: /* MOVG3 r2 -> --r7 (save desc) */
        MCU_PushStack(mcu.r[2]);
        mcu.pc = 0x1bd5;
        return 1;
    case 0x1bd5: /* move r0 #0x7f (candidate age) */
        move8(mcu.r[0], 0x7f);
        mcu.pc = 0x1bd7;
        return 1;
    case 0x1bd7: /* MOVG2 @r3+0xa220 r2 (part chain head) */
        load8(mcu.r[2], ind_addr(3, 0xa220));
        mcu.pc = 0x1bdb;
        return 1;
    case 0x1bdb: /* BMI 0x1bf3: empty chain -> a200 = 0xff */
        mcu.pc = bmi(0x1bf3, 0x1bdd);
        return 1;
    case 0x1bdd: /* CMP @r2+0xa314 r0 */
        cmp8_mem(mcu.r[0], ind_addr(2, 0xa314));
        mcu.pc = 0x1be1;
        return 1;
    case 0x1be1: /* BLS 0x1beb: candidate <= r0, keep */
        mcu.pc = (mcu.sr & (STATUS_C | STATUS_Z)) ? 0x1beb : 0x1be3;
        return 1;
    case 0x1be3: /* MOVG2 @r2+0xa314 r0 (new candidate) */
        load8(mcu.r[0], ind_addr(2, 0xa314));
        mcu.pc = 0x1be7;
        return 1;
    case 0x1be7: /* MOVG3 r2 -> @r3+0xa200 */
        store8(ind_addr(3, 0xa200), (uint8_t)mcu.r[2]);
        mcu.pc = 0x1beb;
        return 1;
    case 0x1beb: /* MOVG2 @r2+0xa250 r2 (next) */
        load8(mcu.r[2], ind_addr(2, 0xa250));
        mcu.pc = 0x1bef;
        return 1;
    case 0x1bef: /* BPL 0x1bdd: loop over part chain */
        mcu.pc = bpl(0x1bdd, 0x1bf1);
        return 1;
    case 0x1bf1: /* BRA 0x1bf7 */
        mcu.pc = 0x1bf7;
        return 1;
    case 0x1bf3: /* MOVG3 r2 -> @r3+0xa200 (write 0xff) */
        store8(ind_addr(3, 0xa200), (uint8_t)mcu.r[2]);
        mcu.pc = 0x1bf7;
        return 1;
    case 0x1bf7: /* MOVG2 r7++ r2 (restore desc) */
        mcu.r[2] = MCU_PopStack();
        mcu.pc = 0x1bf9;
        return 1;
    case 0x1bf9: /* ADDQ #-1 @r3+0xa210 (active voices--) */
        addq_byte(ind_addr(3, 0xa210), -1);
        mcu.pc = 0x1bfd;
        return 1;
    case 0x1bfd: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- H3 0x1b90..0x1bac (desc_setup helper: reset state, pick a200) -------- */

uint32_t step_h3(void)
{
    switch (mcu.pc)
    {
    case 0x1b90: /* CLR @r2+0xa288 */
        clr8_mem(ind_addr(2, 0xa288));
        mcu.pc = 0x1b94;
        return 1;
    case 0x1b94: /* CLR @r2+0xa2a4 */
        clr8_mem(ind_addr(2, 0xa2a4));
        mcu.pc = 0x1b98;
        return 1;
    case 0x1b98: /* MOVG2 @r3+0xa200 r0 */
        load8(mcu.r[0], ind_addr(3, 0xa200));
        mcu.pc = 0x1b9c;
        return 1;
    case 0x1b9c: /* BMI 0x1ba8: no current desc -> take this one */
        mcu.pc = bmi(0x1ba8, 0x1b9e);
        return 1;
    case 0x1b9e: /* MOVG2 @r2+0xa314 r1 (desc age) */
        load8(mcu.r[1], ind_addr(2, 0xa314));
        mcu.pc = 0x1ba2;
        return 1;
    case 0x1ba2: /* CMP @r0+0xa314 r1 (desc age vs current age) */
        cmp8_mem(mcu.r[1], ind_addr(0, 0xa314));
        mcu.pc = 0x1ba6;
        return 1;
    case 0x1ba6: /* BHI 0x1bac: current is smaller, keep it */
        mcu.pc = bhi(0x1bac, 0x1ba8);
        return 1;
    case 0x1ba8: /* MOVG3 r2 -> @r3+0xa200 */
        store8(ind_addr(3, 0xa200), (uint8_t)mcu.r[2]);
        mcu.pc = 0x1bac;
        return 1;
    case 0x1bac: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- pcm_start 0x516c..0x51b5 (release A/B and note_fill callee) ---------- */

uint32_t step_pcm_start(void)
{
    switch (mcu.pc)
    {
    case 0x516c: /* CLR @r1+0xd15c (IRQ pending = 0) */
        clr8_mem(ind_addr(1, 0xd15c));
        mcu.pc = 0x5170;
        return 1;
    case 0x5170: /* MOVG3 r1 -> (dp,0xe03e) (select channel) */
        store8(dp_addr(0xe03e), (uint8_t)mcu.r[1]);
        mcu.pc = 0x5174;
        return 1;
    case 0x5174: /* MOVG2 (dp,0xe032) r4 (latches ram2[sel][9]) */
        load8(mcu.r[4], dp_addr(0xe032));
        mcu.pc = 0x5178;
        return 1;
    case 0x5178: /* MOVG2 (dp,0xe03a) r4 (word read_latch) */
        load16(mcu.r[4], dp_addr(0xe03a));
        mcu.pc = 0x517c;
        return 1;
    case 0x517c: /* MOVG2 (dp,0xe034) r5 (latches ram2[sel][10]) */
        load8(mcu.r[5], dp_addr(0xe034));
        mcu.pc = 0x5180;
        return 1;
    case 0x5180: /* MOVG2 (dp,0xe03a) r5 (word read_latch) */
        load16(mcu.r[5], dp_addr(0xe03a));
        mcu.pc = 0x5184;
        return 1;
    case 0x5184: /* ADD r1 r1 */
        add16_self(mcu.r[1]);
        mcu.pc = 0x5186;
        return 1;
    case 0x5186: /* MOVG2 @r1+0x64d6 r2 (AoS record pointer) */
        load16(mcu.r[2], ind_addr(1, 0x64d6));
        mcu.pc = 0x518a;
        return 1;
    case 0x518a: /* CMP r4 r5 (GT compares r5 - r4) */
        cmp16_reg(mcu.r[5], mcu.r[4]);
        mcu.pc = 0x518c;
        return 1;
    case 0x518c: /* BCC 0x519e: r5 >= r4 -> 0x14 port */
        mcu.pc = bcc(0x519e, 0x518e);
        return 1;
    case 0x518e: /* movi r6 #0x0012 */
        movi16(mcu.r[6], 0x0012);
        mcu.pc = 0x5191;
        return 1;
    case 0x5191: /* MOVG #0x00b5 -> (dp,0xe016) */
        mov_imm16(dp_addr(0xe016), 0x00b5);
        mcu.pc = 0x5197;
        return 1;
    case 0x5197: /* MOVG #0x00b5 -> @r2+0x1a */
        mov_imm16(ind_addr(2, 0x1a), 0x00b5);
        mcu.pc = 0x519c;
        return 1;
    case 0x519c: /* BRA 0x51ac */
        mcu.pc = 0x51ac;
        return 1;
    case 0x519e: /* movi r6 #0x0014 */
        movi16(mcu.r[6], 0x0014);
        mcu.pc = 0x51a1;
        return 1;
    case 0x51a1: /* MOVG #0x00b5 -> (dp,0xe018) */
        mov_imm16(dp_addr(0xe018), 0x00b5);
        mcu.pc = 0x51a7;
        return 1;
    case 0x51a7: /* MOVG #0x00b5 -> @r2+0x1e */
        mov_imm16(ind_addr(2, 0x1e), 0x00b5);
        mcu.pc = 0x51ac;
        return 1;
    case 0x51ac: /* MOVG3 r6 -> @r2+0 (state[0]) */
        store16(ind_addr(2, 0), mcu.r[6]);
        mcu.pc = 0x51af;
        return 1;
    case 0x51af: /* MOVG3 r6 -> @r2+2 */
        store16(ind_addr(2, 2), mcu.r[6]);
        mcu.pc = 0x51b2;
        return 1;
    case 0x51b2: /* MOVG3 r6 -> @r2+4 */
        store16(ind_addr(2, 4), mcu.r[6]);
        mcu.pc = 0x51b5;
        return 1;
    case 0x51b5: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

} /* anonymous namespace */

/* ---- PC tables: one host step per instruction ----------------------------- */

const uint16_t kPoolPopPcs[] = {
    0x19ad, 0x19b1, 0x19b5, 0x19b9, 0x19bb, 0x19bf, 0x19c3,
};

const uint16_t kLinkPcs[] = {
    0x194c, 0x1950, 0x1952, 0x1956, 0x195a, 0x195c, 0x1961, 0x1965,
    0x1967, 0x196c, 0x1971, 0x1976, 0x1978, 0x197c, 0x1980, 0x1984,
    0x1988, 0x198c, 0x1991, 0x1996, 0x199a, 0x199f, 0x19a4, 0x19a8,
    0x19ac,
};

const uint16_t kReleasePcs[] = {
    0x1823, 0x1827, 0x1829, 0x182c, 0x182e, 0x1833, 0x1837, 0x1839,
    0x183d, 0x1841, 0x1845, 0x1847, 0x184a, 0x184e, 0x1850, 0x1854,
    0x1858, 0x185a, 0x185e, 0x1863, 0x1867, 0x186c, 0x1870, 0x1873,
    0x1877, 0x1879, 0x187d,
    0x187e, 0x1880, 0x1884, 0x1886, 0x1889, 0x188b, 0x1890, 0x1894,
    0x1898, 0x189c, 0x18a0, 0x18a2, 0x18a5, 0x18a9, 0x18ab, 0x18af,
    0x18b3, 0x18b7, 0x18bc, 0x18c0, 0x18c3, 0x18c7, 0x18c9, 0x18cd,
};

const uint16_t kFreePcs[] = {
    0x19c4, 0x19c8, 0x19ca, 0x19cf, 0x19d1, 0x19d5, 0x19d9, 0x19dd,
    0x19e1, 0x19e5, 0x19e7, 0x19eb, 0x19ef, 0x19f1, 0x19f5, 0x19f7,
    0x19fa, 0x19fe, 0x1a00, 0x1a04, 0x1a08, 0x1a0a, 0x1a0e, 0x1a13,
    0x1a17, 0x1a1c, 0x1a20, 0x1a23,
};

const uint16_t kH1Pcs[] = {
    0x1b44, 0x1b46, 0x1b48, 0x1b4c, 0x1b4e, 0x1b52, 0x1b57, 0x1b5c,
    0x1b61, 0x1b66, 0x1b68, 0x1b6c, 0x1b6e, 0x1b73, 0x1b75, 0x1b7a,
    0x1b7f, 0x1b84, 0x1b89, 0x1b8d, 0x1b8f,
};

const uint16_t kUnlinkPcs[] = {
    0x1b23, 0x1b27, 0x1b2b, 0x1b2d, 0x1b31, 0x1b33, 0x1b37, 0x1b39,
    0x1b3d, 0x1b3f, 0x1b43,
};

const uint16_t kH2Pcs[] = {
    0x1bad, 0x1bb1, 0x1bb3, 0x1bb6, 0x1bba, 0x1bbe, 0x1bc2, 0x1bc7,
    0x1bcb, 0x1bcd, 0x1bd1, 0x1bd3, 0x1bd5, 0x1bd7, 0x1bdb, 0x1bdd,
    0x1be1, 0x1be3, 0x1be7, 0x1beb, 0x1bef, 0x1bf1, 0x1bf3, 0x1bf7,
    0x1bf9, 0x1bfd,
};

const uint16_t kH3Pcs[] = {
    0x1b90, 0x1b94, 0x1b98, 0x1b9c, 0x1b9e, 0x1ba2, 0x1ba6, 0x1ba8,
    0x1bac,
};

const uint16_t kPcmStartPcs[] = {
    0x516c, 0x5170, 0x5174, 0x5178, 0x517c, 0x5180, 0x5184, 0x5186,
    0x518a, 0x518c, 0x518e, 0x5191, 0x5197, 0x519c, 0x519e, 0x51a1,
    0x51a7, 0x51ac, 0x51af, 0x51b2, 0x51b5,
};

} /* namespace mk2c */

static void register_step_pcs(const uint16_t *pcs, uint32_t count, mk2cpp_hand_routine_fn fn)
{
    for (uint32_t i = 0; i < count; i++)
        MK2CPP_HandRegisterRoutine(pcs[i], fn);
}

void MK2CPP_AllocFreeFillTables(void)
{
    register_step_pcs(mk2c::kPoolPopPcs,
                      (uint32_t)(sizeof(mk2c::kPoolPopPcs) / sizeof(mk2c::kPoolPopPcs[0])),
                      &mk2c::step_pool_pop);
    register_step_pcs(mk2c::kLinkPcs,
                      (uint32_t)(sizeof(mk2c::kLinkPcs) / sizeof(mk2c::kLinkPcs[0])),
                      &mk2c::step_link);
    register_step_pcs(mk2c::kReleasePcs,
                      (uint32_t)(sizeof(mk2c::kReleasePcs) / sizeof(mk2c::kReleasePcs[0])),
                      &mk2c::step_release);
    register_step_pcs(mk2c::kFreePcs,
                      (uint32_t)(sizeof(mk2c::kFreePcs) / sizeof(mk2c::kFreePcs[0])),
                      &mk2c::step_free);
    register_step_pcs(mk2c::kH1Pcs,
                      (uint32_t)(sizeof(mk2c::kH1Pcs) / sizeof(mk2c::kH1Pcs[0])),
                      &mk2c::step_h1);
    register_step_pcs(mk2c::kUnlinkPcs,
                      (uint32_t)(sizeof(mk2c::kUnlinkPcs) / sizeof(mk2c::kUnlinkPcs[0])),
                      &mk2c::step_unlink);
    register_step_pcs(mk2c::kH2Pcs,
                      (uint32_t)(sizeof(mk2c::kH2Pcs) / sizeof(mk2c::kH2Pcs[0])),
                      &mk2c::step_h2);
    register_step_pcs(mk2c::kH3Pcs,
                      (uint32_t)(sizeof(mk2c::kH3Pcs) / sizeof(mk2c::kH3Pcs[0])),
                      &mk2c::step_h3);
    register_step_pcs(mk2c::kPcmStartPcs,
                      (uint32_t)(sizeof(mk2c::kPcmStartPcs) / sizeof(mk2c::kPcmStartPcs[0])),
                      &mk2c::step_pcm_start);
}

