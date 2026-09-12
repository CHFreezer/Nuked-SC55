/*
 * HAND voice/native_allocfree -- alloc/free as per-instruction hand entries.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 2
 * M4 closure step 2 (semantic rewrite): one named void step function per PC, registered flat=pc/cp0 via MK2CPP_HandRegister.
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

#include "hand_registry.h"

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

/* ======================================================================== */
/* pool_pop 0x19ad..0x19c3, 7 PCs */
/* ======================================================================== */

/* 0x19ad MOVG2 (dp,0xa42f) r1 (free head). */
void step_pool_pop_movg2_dp_0xa42f_r1(void)
{
    load8(mcu.r[1], dp_addr(0xa42f));
    mcu.pc = 0x19b1;
}

/* 0x19b1 MOVG2 @r1+0xa3d8 r0 (next slot). */
void step_pool_pop_movg2_r1_0xa3d8_r0(void)
{
    load8(mcu.r[0], ind_addr(1, 0xa3d8));
    mcu.pc = 0x19b5;
}

/* 0x19b5 MOVG3 r0 -> (dp,0xa42f) (head = next). */
void step_pool_pop_movg3_r0_to_dp_0xa42f(void)
{
    store8(dp_addr(0xa42f), (uint8_t)mcu.r[0]);
    mcu.pc = 0x19b9;
}

/* 0x19b9 BPL 0x19bf: list not empty. */
void step_pool_pop_bpl_0x19bf(void)
{
    mcu.pc = bpl(0x19bf, 0x19bb);
}

/* 0x19bb MOVG3 r0 -> (dp,0xa430) (emptied list: tail too). */
void step_pool_pop_movg3_r0_to_dp_0xa430(void)
{
    store8(dp_addr(0xa430), (uint8_t)mcu.r[0]);
    mcu.pc = 0x19bf;
}

/* 0x19bf ADDQ #-1 (dp,0xa42d) (count--). */
void step_pool_pop_addq_1_dp_0xa42d(void)
{
    addq_byte(dp_addr(0xa42d), -1);
    mcu.pc = 0x19c3;
}

/* 0x19c3 rts. */
void step_pool_pop_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* link 0x194c..0x19ac, 25 PCs */
/* ======================================================================== */

/* 0x194c MOVG2 @r2+0xa2dc r0 (chain tail). */
void step_link_movg2_r2_0xa2dc_r0_chain_tail(void)
{
    load8(mcu.r[0], ind_addr(2, 0xa2dc));
    mcu.pc = 0x1950;
}

/* 0x1950 BPL 0x1978: chain empty. */
void step_link_bpl_0x1978_chain_empty(void)
{
    mcu.pc = bpl(0x1978, 0x1952);
}

/* 0x1952 MOVG3 r1 -> @r2+0xa2c0 (new head). */
void step_link_movg3_r1_to_r2_0xa2c0_new_head(void)
{
    store8(ind_addr(2, 0xa2c0), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1956;
}

/* 0x1956 MOVG2 @r1+0xd0a8 r0 (start-prev). */
void step_link_movg2_r1_0xd0a8_r0_start_prev(void)
{
    load8(mcu.r[0], ind_addr(1, 0xd0a8));
    mcu.pc = 0x195a;
}

/* 0x195a BMI 0x1961: no start-prev neighbour. */
void step_link_bmi_0x1961_no_start_prev(void)
{
    mcu.pc = bmi(0x1961, 0x195c);
}

/* 0x195c MOVG #0xff -> @r0+0xd0c4 (unexecuted arm). */
void step_link_movg_0xff_to_r0_0xd0c4_arm(void)
{
    mov_imm8(ind_addr(0, 0xd0c4), 0xff);
    mcu.pc = 0x1961;
}

/* 0x1961 MOVG2 @r1+0xd0c4 r0 (start-next). */
void step_link_movg2_r1_0xd0c4_r0_start_next(void)
{
    load8(mcu.r[0], ind_addr(1, 0xd0c4));
    mcu.pc = 0x1965;
}

/* 0x1965 BMI 0x196c: no start-next neighbour. */
void step_link_bmi_0x196c_no_start_next(void)
{
    mcu.pc = bmi(0x196c, 0x1967);
}

/* 0x1967 MOVG #0xff -> @r0+0xd0a8 (unexecuted arm). */
void step_link_movg_0xff_to_r0_0xd0a8_arm(void)
{
    mov_imm8(ind_addr(0, 0xd0a8), 0xff);
    mcu.pc = 0x196c;
}

/* 0x196c MOVG #0xff -> @r1+0xa410. */
void step_link_movg_0xff_to_r1_0xa410(void)
{
    mov_imm8(ind_addr(1, 0xa410), 0xff);
    mcu.pc = 0x1971;
}

/* 0x1971 MOVG #0xff -> @r1+0xd0a8. */
void step_link_movg_0xff_to_r1_0xd0a8(void)
{
    mov_imm8(ind_addr(1, 0xd0a8), 0xff);
    mcu.pc = 0x1976;
}

/* 0x1976 BRA 0x1996. */
void step_link_bra_0x1996(void)
{
    mcu.pc = 0x1996;
}

/* 0x1978 MOVG3 r0 -> @r2+0xa2c0 (head = old tail). */
void step_link_movg3_r0_to_r2_0xa2c0_old_tail(void)
{
    store8(ind_addr(2, 0xa2c0), (uint8_t)mcu.r[0]);
    mcu.pc = 0x197c;
}

/* 0x197c MOVG3 r1 -> @r0+0xa3f4. */
void step_link_movg3_r1_to_r0_0xa3f4(void)
{
    store8(ind_addr(0, 0xa3f4), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1980;
}

/* 0x1980 MOVG3 r1 -> @r0+0xd0c4. */
void step_link_movg3_r1_to_r0_0xd0c4(void)
{
    store8(ind_addr(0, 0xd0c4), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1984;
}

/* 0x1984 MOVG3 r0 -> @r1+0xa410. */
void step_link_movg3_r0_to_r1_0xa410(void)
{
    store8(ind_addr(1, 0xa410), (uint8_t)mcu.r[0]);
    mcu.pc = 0x1988;
}

/* 0x1988 MOVG3 r0 -> @r1+0xd0a8. */
void step_link_movg3_r0_to_r1_0xd0a8(void)
{
    store8(ind_addr(1, 0xd0a8), (uint8_t)mcu.r[0]);
    mcu.pc = 0x198c;
}

/* 0x198c MOVG #0xff -> @r0+0xa410. */
void step_link_movg_0xff_to_r0_0xa410(void)
{
    mov_imm8(ind_addr(0, 0xa410), 0xff);
    mcu.pc = 0x1991;
}

/* 0x1991 MOVG #0xff -> @r0+0xd0a8. */
void step_link_movg_0xff_to_r0_0xd0a8(void)
{
    mov_imm8(ind_addr(0, 0xd0a8), 0xff);
    mcu.pc = 0x1996;
}

/* 0x1996 MOVG3 r1 -> @r2+0xa2dc (new tail). */
void step_link_movg3_r1_to_r2_0xa2dc_new_tail(void)
{
    store8(ind_addr(2, 0xa2dc), (uint8_t)mcu.r[1]);
    mcu.pc = 0x199a;
}

/* 0x199a MOVG #0xff -> @r1+0xa3f4. */
void step_link_movg_0xff_to_r1_0xa3f4(void)
{
    mov_imm8(ind_addr(1, 0xa3f4), 0xff);
    mcu.pc = 0x199f;
}

/* 0x199f MOVG #0xff -> @r1+0xd0c4. */
void step_link_movg_0xff_to_r1_0xd0c4(void)
{
    mov_imm8(ind_addr(1, 0xd0c4), 0xff);
    mcu.pc = 0x19a4;
}

/* 0x19a4 MOVG3 r3 -> @r1+0xa368 (slot -> part). */
void step_link_movg3_r3_to_r1_0xa368_slot_part(void)
{
    store8(ind_addr(1, 0xa368), (uint8_t)mcu.r[3]);
    mcu.pc = 0x19a8;
}

/* 0x19a8 MOVG3 r2 -> @r1+0xa384 (slot -> desc). */
void step_link_movg3_r2_to_r1_0xa384_slot_desc(void)
{
    store8(ind_addr(1, 0xa384), (uint8_t)mcu.r[2]);
    mcu.pc = 0x19ac;
}

/* 0x19ac rts. */
void step_link_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* release A 0x1823..0x187d, 27 PCs (IML=7 window 0x1823..0x1833) */
/* ======================================================================== */

/* 0x1823 BSET_ORC #0x0700 r0: IML=7. */
void step_release_a_bset_orc_0x0700_r0_iml7(void)
{
    bset_orc_iml7();
    mcu.pc = 0x1827;
}

/* 0x1827 stm #0x3e (save r1..r5). */
void step_release_a_stm_0x3e(void)
{
    stm_3e();
    mcu.pc = 0x1829;
}

/* 0x1829 jsr #0x516c (pcm_start). */
void step_release_a_jsr_0x516c_pcm_start(void)
{
    MCU_PushStack(0x182c);
    mcu.pc = 0x516c;
}

/* 0x182c ldm #0x3e (restore r1..r5). */
void step_release_a_ldm_0x3e(void)
{
    ldm_3e();
    mcu.pc = 0x182e;
}

/* 0x182e MOVG #0x04 -> @r1+0xd0e0 (cmd = stop). */
void step_release_a_movg_0x04_to_r1_0xd0e0_cmd_stop(void)
{
    mov_imm8(ind_addr(1, 0xd0e0), 0x04);
    mcu.pc = 0x1833;
}

/* 0x1833 BCLR_ANDC #0xf8ff r0: IML=0. */
void step_release_a_bclr_andc_0xf8ff_r0_iml0(void)
{
    bclr_andc_iml0();
    mcu.pc = 0x1837;
}

/* 0x1837 EXTU r1. */
void step_release_a_extu_r1(void)
{
    extu(mcu.r[1]);
    mcu.pc = 0x1839;
}

/* 0x1839 CLR @r1+0xad0e. */
void step_release_a_clr_r1_0xad0e(void)
{
    clr8_mem(ind_addr(1, 0xad0e));
    mcu.pc = 0x183d;
}

/* 0x183d CLR @r1+0xa3bc. */
void step_release_a_clr_r1_0xa3bc(void)
{
    clr8_mem(ind_addr(1, 0xa3bc));
    mcu.pc = 0x1841;
}

/* 0x1841 CLR @r1+0xa4b4. */
void step_release_a_clr_r1_0xa4b4(void)
{
    clr8_mem(ind_addr(1, 0xa4b4));
    mcu.pc = 0x1845;
}

/* 0x1845 CLR r0. */
void step_release_a_clr_r0(void)
{
    clr_reg(mcu.r[0]);
    mcu.pc = 0x1847;
}

/* 0x1847 bsr16 -> 0x1b44 (H1). */
void step_release_a_bsr16_to_0x1b44_h1(void)
{
    MCU_PushStack(0x184a);
    mcu.pc = 0x1b44;
}

/* 0x184a MOVG2 (dp,0xa430) r0 (free tail). */
void step_release_a_movg2_dp_0xa430_r0_free_tail(void)
{
    load8(mcu.r[0], dp_addr(0xa430));
    mcu.pc = 0x184e;
}

/* 0x184e BPL 0x185a: tail valid. */
void step_release_a_bpl_0x185a_tail_valid(void)
{
    mcu.pc = bpl(0x185a, 0x1850);
}

/* 0x1850 MOVG3 r1 -> (dp,0xa42f) (empty list: head = slot). */
void step_release_a_movg3_r1_to_dp_0xa42f_head(void)
{
    store8(dp_addr(0xa42f), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1854;
}

/* 0x1854 MOVG3 r0 -> @r1+0xa3d8. */
void step_release_a_movg3_r0_to_r1_0xa3d8(void)
{
    store8(ind_addr(1, 0xa3d8), (uint8_t)mcu.r[0]);
    mcu.pc = 0x1858;
}

/* 0x1858 BRA 0x1863. */
void step_release_a_bra_0x1863(void)
{
    mcu.pc = 0x1863;
}

/* 0x185a MOVG3 r1 -> @r0+0xa3d8 (old tail.next = slot). */
void step_release_a_movg3_r1_to_r0_0xa3d8_old_tail_next(void)
{
    store8(ind_addr(0, 0xa3d8), (uint8_t)mcu.r[1]);
    mcu.pc = 0x185e;
}

/* 0x185e MOVG #0xff -> @r1+0xa3d8. */
void step_release_a_movg_0xff_to_r1_0xa3d8(void)
{
    mov_imm8(ind_addr(1, 0xa3d8), 0xff);
    mcu.pc = 0x1863;
}

/* 0x1863 MOVG3 r1 -> (dp,0xa430) (tail = slot). */
void step_release_a_movg3_r1_to_dp_0xa430_tail(void)
{
    store8(dp_addr(0xa430), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1867;
}

/* 0x1867 MOVG #0x94 -> @r1+0xa3a0 (free). */
void step_release_a_movg_0x94_to_r1_0xa3a0_free(void)
{
    mov_imm8(ind_addr(1, 0xa3a0), 0x94);
    mcu.pc = 0x186c;
}

/* 0x186c ADDQ #1 (dp,0xa42d) (count++). */
void step_release_a_addq_1_dp_0xa42d_count(void)
{
    addq_byte(dp_addr(0xa42d), 1);
    mcu.pc = 0x1870;
}

/* 0x1870 bsr16 -> 0x1bad (H2). */
void step_release_a_bsr16_to_0x1bad_h2(void)
{
    MCU_PushStack(0x1873);
    mcu.pc = 0x1bad;
}

/* 0x1873 ADDQ #-1 (dp,0xa42c) (shortfall--). */
void step_release_a_addq_1_dp_0xa42c_shortfall(void)
{
    addq_byte(dp_addr(0xa42c), -1);
    mcu.pc = 0x1877;
}

/* 0x1877 BPL 0x187d: shortfall >= 0. */
void step_release_a_bpl_0x187d(void)
{
    mcu.pc = bpl(0x187d, 0x1879);
}

/* 0x1879 CLR (dp,0xa42c) (clamp to 0). */
void step_release_a_clr_dp_0xa42c_clamp(void)
{
    clr8_mem(dp_addr(0xa42c));
    mcu.pc = 0x187d;
}

/* 0x187d rts. */
void step_release_a_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* release B 0x187e..0x18cd, 24 PCs (IML=7 window 0x187e..0x1890) */
/* ======================================================================== */

/* 0x187e EXTU r1. */
void step_release_b_extu_r1(void)
{
    extu(mcu.r[1]);
    mcu.pc = 0x1880;
}

/* 0x1880 BSET_ORC #0x0700 r0: IML=7. */
void step_release_b_bset_orc_0x0700_r0_iml7(void)
{
    bset_orc_iml7();
    mcu.pc = 0x1884;
}

/* 0x1884 stm #0x3e. */
void step_release_b_stm_0x3e(void)
{
    stm_3e();
    mcu.pc = 0x1886;
}

/* 0x1886 jsr #0x516c (pcm_start). */
void step_release_b_jsr_0x516c_pcm_start(void)
{
    MCU_PushStack(0x1889);
    mcu.pc = 0x516c;
}

/* 0x1889 ldm #0x3e. */
void step_release_b_ldm_0x3e(void)
{
    ldm_3e();
    mcu.pc = 0x188b;
}

/* 0x188b MOVG #0x04 -> @r1+0xd0e0. */
void step_release_b_movg_0x04_to_r1_0xd0e0_cmd_stop(void)
{
    mov_imm8(ind_addr(1, 0xd0e0), 0x04);
    mcu.pc = 0x1890;
}

/* 0x1890 BCLR_ANDC #0xf8ff r0: IML=0. */
void step_release_b_bclr_andc_0xf8ff_r0_iml0(void)
{
    bclr_andc_iml0();
    mcu.pc = 0x1894;
}

/* 0x1894 CLR @r1+0xad0e. */
void step_release_b_clr_r1_0xad0e(void)
{
    clr8_mem(ind_addr(1, 0xad0e));
    mcu.pc = 0x1898;
}

/* 0x1898 CLR @r1+0xa3bc. */
void step_release_b_clr_r1_0xa3bc(void)
{
    clr8_mem(ind_addr(1, 0xa3bc));
    mcu.pc = 0x189c;
}

/* 0x189c CLR @r1+0xa4b4. */
void step_release_b_clr_r1_0xa4b4(void)
{
    clr8_mem(ind_addr(1, 0xa4b4));
    mcu.pc = 0x18a0;
}

/* 0x18a0 CLR r0. */
void step_release_b_clr_r0(void)
{
    clr_reg(mcu.r[0]);
    mcu.pc = 0x18a2;
}

/* 0x18a2 bsr16 -> 0x1b44 (H1). */
void step_release_b_bsr16_to_0x1b44_h1(void)
{
    MCU_PushStack(0x18a5);
    mcu.pc = 0x1b44;
}

/* 0x18a5 MOVG2 (dp,0xa42f) r0 (free head). */
void step_release_b_movg2_dp_0xa42f_r0_free_head(void)
{
    load8(mcu.r[0], dp_addr(0xa42f));
    mcu.pc = 0x18a9;
}

/* 0x18a9 BPL 0x18af: head valid. */
void step_release_b_bpl_0x18af_head_valid(void)
{
    mcu.pc = bpl(0x18af, 0x18ab);
}

/* 0x18ab MOVG3 r1 -> (dp,0xa430) (empty list: tail = slot). */
void step_release_b_movg3_r1_to_dp_0xa430_tail(void)
{
    store8(dp_addr(0xa430), (uint8_t)mcu.r[1]);
    mcu.pc = 0x18af;
}

/* 0x18af MOVG3 r0 -> @r1+0xa3d8 (slot.next = old head). */
void step_release_b_movg3_r0_to_r1_0xa3d8_slot_next(void)
{
    store8(ind_addr(1, 0xa3d8), (uint8_t)mcu.r[0]);
    mcu.pc = 0x18b3;
}

/* 0x18b3 MOVG3 r1 -> (dp,0xa42f) (head = slot). */
void step_release_b_movg3_r1_to_dp_0xa42f_head(void)
{
    store8(dp_addr(0xa42f), (uint8_t)mcu.r[1]);
    mcu.pc = 0x18b7;
}

/* 0x18b7 MOVG #0x94 -> @r1+0xa3a0 (free). */
void step_release_b_movg_0x94_to_r1_0xa3a0_free(void)
{
    mov_imm8(ind_addr(1, 0xa3a0), 0x94);
    mcu.pc = 0x18bc;
}

/* 0x18bc ADDQ #1 (dp,0xa42d). */
void step_release_b_addq_1_dp_0xa42d_count(void)
{
    addq_byte(dp_addr(0xa42d), 1);
    mcu.pc = 0x18c0;
}

/* 0x18c0 bsr16 -> 0x1bad (H2). */
void step_release_b_bsr16_to_0x1bad_h2(void)
{
    MCU_PushStack(0x18c3);
    mcu.pc = 0x1bad;
}

/* 0x18c3 ADDQ #-1 (dp,0xa42c). */
void step_release_b_addq_1_dp_0xa42c_shortfall(void)
{
    addq_byte(dp_addr(0xa42c), -1);
    mcu.pc = 0x18c7;
}

/* 0x18c7 BPL 0x18cd. */
void step_release_b_bpl_0x18cd(void)
{
    mcu.pc = bpl(0x18cd, 0x18c9);
}

/* 0x18c9 CLR (dp,0xa42c). */
void step_release_b_clr_dp_0xa42c_clamp(void)
{
    clr8_mem(dp_addr(0xa42c));
    mcu.pc = 0x18cd;
}

/* 0x18cd rts. */
void step_release_b_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* free_voice 0x19c4..0x1a23, 28 PCs */
/* ======================================================================== */

/* 0x19c4 BTSTI @r1+0xa3a0 #7 (already free?). */
void step_free_btsti_r1_0xa3a0_7_already_free(void)
{
    btsti(ind_addr(1, 0xa3a0), 7);
    mcu.pc = 0x19c8;
}

/* 0x19c8 BNE 0x1a23: already free. */
void step_free_bne_0x1a23_already_free(void)
{
    mcu.pc = bne(0x1a23, 0x19ca);
}

/* 0x19ca SUB @r1+0xad0e #0xff (channel assigned?). */
void step_free_sub_r1_0xad0e_0xff_channel_assigned(void)
{
    sub8_nowrite(ind_addr(1, 0xad0e), 0xff);
    mcu.pc = 0x19cf;
}

/* 0x19cf BEQ 0x1a23: no channel. */
void step_free_beq_0x1a23_no_channel(void)
{
    mcu.pc = beq(0x1a23, 0x19d1);
}

/* 0x19d1 MOVG2 @r1+0xa384 r2 (desc). */
void step_free_movg2_r1_0xa384_r2_desc(void)
{
    load8(mcu.r[2], ind_addr(1, 0xa384));
    mcu.pc = 0x19d5;
}

/* 0x19d5 MOVG2 @r1+0xa368 r3 (part). */
void step_free_movg2_r1_0xa368_r3_part(void)
{
    load8(mcu.r[3], ind_addr(1, 0xa368));
    mcu.pc = 0x19d9;
}

/* 0x19d9 CLR @r1+0xa3bc. */
void step_free_clr_r1_0xa3bc(void)
{
    clr8_mem(ind_addr(1, 0xa3bc));
    mcu.pc = 0x19dd;
}

/* 0x19dd CLR @r1+0xa4b4. */
void step_free_clr_r1_0xa4b4(void)
{
    clr8_mem(ind_addr(1, 0xa4b4));
    mcu.pc = 0x19e1;
}

/* 0x19e1 CMP @r2+0xa2c0 r1 (slot == vhead?). */
void step_free_cmp_r2_0xa2c0_r1_slot_vhead(void)
{
    cmp8_mem(mcu.r[1], ind_addr(2, 0xa2c0));
    mcu.pc = 0x19e5;
}

/* 0x19e5 BNE 0x19eb. */
void step_free_bne_0x19eb(void)
{
    mcu.pc = bne(0x19eb, 0x19e7);
}

/* 0x19e7 BCLR @r2+0xa34c #1. */
void step_free_bclr_r2_0xa34c_1(void)
{
    bclr_mem(ind_addr(2, 0xa34c), 1);
    mcu.pc = 0x19eb;
}

/* 0x19eb CMP @r2+0xa2dc r1 (slot == vtail?). */
void step_free_cmp_r2_0xa2dc_r1_slot_vtail(void)
{
    cmp8_mem(mcu.r[1], ind_addr(2, 0xa2dc));
    mcu.pc = 0x19ef;
}

/* 0x19ef BNE 0x19f5. */
void step_free_bne_0x19f5(void)
{
    mcu.pc = bne(0x19f5, 0x19f1);
}

/* 0x19f1 BCLR @r2+0xa34c #0. */
void step_free_bclr_r2_0xa34c_0(void)
{
    bclr_mem(ind_addr(2, 0xa34c), 0);
    mcu.pc = 0x19f5;
}

/* 0x19f5 CLR r0. */
void step_free_clr_r0(void)
{
    clr_reg(mcu.r[0]);
    mcu.pc = 0x19f7;
}

/* 0x19f7 bsr16 -> 0x1b44 (H1). */
void step_free_bsr16_to_0x1b44_h1(void)
{
    MCU_PushStack(0x19fa);
    mcu.pc = 0x1b44;
}

/* 0x19fa MOVG2 (dp,0xa430) r0 (free tail). */
void step_free_movg2_dp_0xa430_r0_free_tail(void)
{
    load8(mcu.r[0], dp_addr(0xa430));
    mcu.pc = 0x19fe;
}

/* 0x19fe BPL 0x1a0a. */
void step_free_bpl_0x1a0a(void)
{
    mcu.pc = bpl(0x1a0a, 0x1a00);
}

/* 0x1a00 MOVG3 r1 -> (dp,0xa42f) (empty list). */
void step_free_movg3_r1_to_dp_0xa42f_head(void)
{
    store8(dp_addr(0xa42f), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1a04;
}

/* 0x1a04 MOVG3 r0 -> @r1+0xa3d8. */
void step_free_movg3_r0_to_r1_0xa3d8(void)
{
    store8(ind_addr(1, 0xa3d8), (uint8_t)mcu.r[0]);
    mcu.pc = 0x1a08;
}

/* 0x1a08 BRA 0x1a13. */
void step_free_bra_0x1a13(void)
{
    mcu.pc = 0x1a13;
}

/* 0x1a0a MOVG3 r1 -> @r0+0xa3d8. */
void step_free_movg3_r1_to_r0_0xa3d8(void)
{
    store8(ind_addr(0, 0xa3d8), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1a0e;
}

/* 0x1a0e MOVG #0xff -> @r1+0xa3d8. */
void step_free_movg_0xff_to_r1_0xa3d8(void)
{
    mov_imm8(ind_addr(1, 0xa3d8), 0xff);
    mcu.pc = 0x1a13;
}

/* 0x1a13 MOVG3 r1 -> (dp,0xa430). */
void step_free_movg3_r1_to_dp_0xa430_tail(void)
{
    store8(dp_addr(0xa430), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1a17;
}

/* 0x1a17 MOVG #0x94 -> @r1+0xa3a0. */
void step_free_movg_0x94_to_r1_0xa3a0(void)
{
    mov_imm8(ind_addr(1, 0xa3a0), 0x94);
    mcu.pc = 0x1a1c;
}

/* 0x1a1c ADDQ #1 (dp,0xa42d). */
void step_free_addq_1_dp_0xa42d_count(void)
{
    addq_byte(dp_addr(0xa42d), 1);
    mcu.pc = 0x1a20;
}

/* 0x1a20 bsr16 -> 0x1bad (H2). */
void step_free_bsr16_to_0x1bad_h2(void)
{
    MCU_PushStack(0x1a23);
    mcu.pc = 0x1bad;
}

/* 0x1a23 rts. */
void step_free_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* H1 0x1b44..0x1b8f, 21 PCs (voice out of its descriptor chain) */
/* ======================================================================== */

/* 0x1b44 MOVG3 r3 -> --r7 (save r3). */
void step_h1_movg3_r3_to_r7_save(void)
{
    MCU_PushStack(mcu.r[3]);
    mcu.pc = 0x1b46;
}

/* 0x1b46 CLR r3. */
void step_h1_clr_r3(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x1b48;
}

/* 0x1b48 MOVG2 @r1+0xa3f4 r3 (prev). */
void step_h1_movg2_r1_0xa3f4_r3_prev(void)
{
    load8(mcu.r[3], ind_addr(1, 0xa3f4));
    mcu.pc = 0x1b4c;
}

/* 0x1b4c BMI 0x1b68: no prev. */
void step_h1_bmi_0x1b68_no_prev(void)
{
    mcu.pc = bmi(0x1b68, 0x1b4e);
}

/* 0x1b4e MOVG3 r3 -> @r2+0xa2c0 (vhead = prev). */
void step_h1_movg3_r3_to_r2_0xa2c0_vhead(void)
{
    store8(ind_addr(2, 0xa2c0), (uint8_t)mcu.r[3]);
    mcu.pc = 0x1b52;
}

/* 0x1b52 MOVG #0xff -> @r3+0xa410. */
void step_h1_movg_0xff_to_r3_0xa410(void)
{
    mov_imm8(ind_addr(3, 0xa410), 0xff);
    mcu.pc = 0x1b57;
}

/* 0x1b57 MOVG #0xff -> @r3+0xd0a8. */
void step_h1_movg_0xff_to_r3_0xd0a8(void)
{
    mov_imm8(ind_addr(3, 0xd0a8), 0xff);
    mcu.pc = 0x1b5c;
}

/* 0x1b5c MOVG #0xff -> @r1+0xa3f4. */
void step_h1_movg_0xff_to_r1_0xa3f4(void)
{
    mov_imm8(ind_addr(1, 0xa3f4), 0xff);
    mcu.pc = 0x1b61;
}

/* 0x1b61 MOVG #0xff -> @r1+0xd0c4. */
void step_h1_movg_0xff_to_r1_0xd0c4(void)
{
    mov_imm8(ind_addr(1, 0xd0c4), 0xff);
    mcu.pc = 0x1b66;
}

/* 0x1b66 BRA 0x1b8d (a2dc stays untouched). */
void step_h1_bra_0x1b8d(void)
{
    mcu.pc = 0x1b8d;
}

/* 0x1b68 MOVG2 @r1+0xa410 r3 (next). */
void step_h1_movg2_r1_0xa410_r3_next(void)
{
    load8(mcu.r[3], ind_addr(1, 0xa410));
    mcu.pc = 0x1b6c;
}

/* 0x1b6c BPL 0x1b75: has next. */
void step_h1_bpl_0x1b75_has_next(void)
{
    mcu.pc = bpl(0x1b75, 0x1b6e);
}

/* 0x1b6e MOVG #0xff -> @r2+0xa2c0 (sole node: clear head). */
void step_h1_movg_0xff_to_r2_0xa2c0_clear_head(void)
{
    mov_imm8(ind_addr(2, 0xa2c0), 0xff);
    mcu.pc = 0x1b73;
}

/* 0x1b73 BRA 0x1b89. */
void step_h1_bra_0x1b89(void)
{
    mcu.pc = 0x1b89;
}

/* 0x1b75 MOVG #0xff -> @r1+0xa410. */
void step_h1_movg_0xff_to_r1_0xa410(void)
{
    mov_imm8(ind_addr(1, 0xa410), 0xff);
    mcu.pc = 0x1b7a;
}

/* 0x1b7a MOVG #0xff -> @r1+0xd0a8. */
void step_h1_movg_0xff_to_r1_0xd0a8(void)
{
    mov_imm8(ind_addr(1, 0xd0a8), 0xff);
    mcu.pc = 0x1b7f;
}

/* 0x1b7f MOVG #0xff -> @r3+0xa3f4. */
void step_h1_movg_0xff_to_r3_0xa3f4(void)
{
    mov_imm8(ind_addr(3, 0xa3f4), 0xff);
    mcu.pc = 0x1b84;
}

/* 0x1b84 MOVG #0xff -> @r3+0xd0c4. */
void step_h1_movg_0xff_to_r3_0xd0c4(void)
{
    mov_imm8(ind_addr(3, 0xd0c4), 0xff);
    mcu.pc = 0x1b89;
}

/* 0x1b89 MOVG3 r3 -> @r2+0xa2dc (vtail = next / 0xff). */
void step_h1_movg3_r3_to_r2_0xa2dc_vtail(void)
{
    store8(ind_addr(2, 0xa2dc), (uint8_t)mcu.r[3]);
    mcu.pc = 0x1b8d;
}

/* 0x1b8d MOVG2 r7++ r3 (restore r3). */
void step_h1_movg2_r7_r3_restore(void)
{
    mcu.r[3] = MCU_PopStack();
    mcu.pc = 0x1b8f;
}

/* 0x1b8f rts. */
void step_h1_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* unlink 0x1b23..0x1b43, 11 PCs (desc out of the part chain) */
/* ======================================================================== */

/* 0x1b23 MOVG2 @r2+0xa250 r0 (next). */
void step_unlink_movg2_r2_0xa250_r0_next(void)
{
    load8(mcu.r[0], ind_addr(2, 0xa250));
    mcu.pc = 0x1b27;
}

/* 0x1b27 MOVG2 @r2+0xa26c r1 (prev). */
void step_unlink_movg2_r2_0xa26c_r1_prev(void)
{
    load8(mcu.r[1], ind_addr(2, 0xa26c));
    mcu.pc = 0x1b2b;
}

/* 0x1b2b BPL 0x1b33: has prev. */
void step_unlink_bpl_0x1b33_has_prev(void)
{
    mcu.pc = bpl(0x1b33, 0x1b2d);
}

/* 0x1b2d MOVG3 r0 -> @r3+0xa220 (part head = next). */
void step_unlink_movg3_r0_to_r3_0xa220_part_head(void)
{
    store8(ind_addr(3, 0xa220), (uint8_t)mcu.r[0]);
    mcu.pc = 0x1b31;
}

/* 0x1b31 BRA 0x1b37. */
void step_unlink_bra_0x1b37(void)
{
    mcu.pc = 0x1b37;
}

/* 0x1b33 MOVG3 r0 -> @r1+0xa250 (prev.next = next). */
void step_unlink_movg3_r0_to_r1_0xa250_prev_next(void)
{
    store8(ind_addr(1, 0xa250), (uint8_t)mcu.r[0]);
    mcu.pc = 0x1b37;
}

/* 0x1b37 BPL 0x1b3f: second branch tests `next` (store flags). */
void step_unlink_bpl_0x1b3f_next_store_flags(void)
{
    mcu.pc = bpl(0x1b3f, 0x1b39);
}

/* 0x1b39 MOVG3 r1 -> @r3+0xa230 (part tail = prev). */
void step_unlink_movg3_r1_to_r3_0xa230_part_tail(void)
{
    store8(ind_addr(3, 0xa230), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1b3d;
}

/* 0x1b3d BRA 0x1b43. */
void step_unlink_bra_0x1b43(void)
{
    mcu.pc = 0x1b43;
}

/* 0x1b3f MOVG3 r1 -> @r0+0xa26c (next.prev = prev). */
void step_unlink_movg3_r1_to_r0_0xa26c_next_prev(void)
{
    store8(ind_addr(0, 0xa26c), (uint8_t)mcu.r[1]);
    mcu.pc = 0x1b43;
}

/* 0x1b43 rts. */
void step_unlink_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* H2 0x1bad..0x1bfd, 26 PCs (descriptor finish) */
/* ======================================================================== */

/* 0x1bad TST @r2+0xa2c0 (voice chain empty?). */
void step_h2_tst_r2_0xa2c0_chain_empty(void)
{
    tst8_mem(ind_addr(2, 0xa2c0));
    mcu.pc = 0x1bb1;
}

/* 0x1bb1 BPL 0x1bf9: chain still busy, only count. */
void step_h2_bpl_0x1bf9_chain_busy(void)
{
    mcu.pc = bpl(0x1bf9, 0x1bb3);
}

/* 0x1bb3 bsr16 -> 0x1b23 (unlink desc). */
void step_h2_bsr16_to_0x1b23_unlink(void)
{
    MCU_PushStack(0x1bb6);
    mcu.pc = 0x1b23;
}

/* 0x1bb6 MOVG2 (dp,0xa42e) r0 (old desc head). */
void step_h2_movg2_dp_0xa42e_r0_old_desc_head(void)
{
    load8(mcu.r[0], dp_addr(0xa42e));
    mcu.pc = 0x1bba;
}

/* 0x1bba MOVG3 r0 -> @r2+0xa250 (desc.next = old head). */
void step_h2_movg3_r0_to_r2_0xa250_desc_next(void)
{
    store8(ind_addr(2, 0xa250), (uint8_t)mcu.r[0]);
    mcu.pc = 0x1bbe;
}

/* 0x1bbe MOVG3 r2 -> (dp,0xa42e) (head = desc). */
void step_h2_movg3_r2_to_dp_0xa42e_head(void)
{
    store8(dp_addr(0xa42e), (uint8_t)mcu.r[2]);
    mcu.pc = 0x1bc2;
}

/* 0x1bc2 MOVG #0x94 -> @r2+0xa288 (desc free). */
void step_h2_movg_0x94_to_r2_0xa288_desc_free(void)
{
    mov_imm8(ind_addr(2, 0xa288), 0x94);
    mcu.pc = 0x1bc7;
}

/* 0x1bc7 TST @r3+0xa200 (part current desc). */
void step_h2_tst_r3_0xa200_current_desc(void)
{
    tst8_mem(ind_addr(3, 0xa200));
    mcu.pc = 0x1bcb;
}

/* 0x1bcb BMI 0x1bf9: no current desc. */
void step_h2_bmi_0x1bf9_no_current_desc(void)
{
    mcu.pc = bmi(0x1bf9, 0x1bcd);
}

/* 0x1bcd CMP @r3+0xa200 r2 (a200 == desc?). */
void step_h2_cmp_r3_0xa200_r2(void)
{
    cmp8_mem(mcu.r[2], ind_addr(3, 0xa200));
    mcu.pc = 0x1bd1;
}

/* 0x1bd1 BNE 0x1bf9. */
void step_h2_bne_0x1bf9(void)
{
    mcu.pc = bne(0x1bf9, 0x1bd3);
}

/* 0x1bd3 MOVG3 r2 -> --r7 (save desc). */
void step_h2_movg3_r2_to_r7_save_desc(void)
{
    MCU_PushStack(mcu.r[2]);
    mcu.pc = 0x1bd5;
}

/* 0x1bd5 move r0 #0x7f (candidate age). */
void step_h2_move_r0_0x7f_candidate_age(void)
{
    move8(mcu.r[0], 0x7f);
    mcu.pc = 0x1bd7;
}

/* 0x1bd7 MOVG2 @r3+0xa220 r2 (part chain head). */
void step_h2_movg2_r3_0xa220_r2_part_head(void)
{
    load8(mcu.r[2], ind_addr(3, 0xa220));
    mcu.pc = 0x1bdb;
}

/* 0x1bdb BMI 0x1bf3: empty chain -> a200 = 0xff. */
void step_h2_bmi_0x1bf3_empty_chain(void)
{
    mcu.pc = bmi(0x1bf3, 0x1bdd);
}

/* 0x1bdd CMP @r2+0xa314 r0. */
void step_h2_cmp_r2_0xa314_r0(void)
{
    cmp8_mem(mcu.r[0], ind_addr(2, 0xa314));
    mcu.pc = 0x1be1;
}

/* 0x1be1 BLS 0x1beb: candidate <= r0, keep. */
void step_h2_bls_0x1beb_candidate_le_r0(void)
{
    mcu.pc = (mcu.sr & (STATUS_C | STATUS_Z)) ? 0x1beb : 0x1be3;
}

/* 0x1be3 MOVG2 @r2+0xa314 r0 (new candidate). */
void step_h2_movg2_r2_0xa314_r0_new_candidate(void)
{
    load8(mcu.r[0], ind_addr(2, 0xa314));
    mcu.pc = 0x1be7;
}

/* 0x1be7 MOVG3 r2 -> @r3+0xa200. */
void step_h2_movg3_r2_to_r3_0xa200(void)
{
    store8(ind_addr(3, 0xa200), (uint8_t)mcu.r[2]);
    mcu.pc = 0x1beb;
}

/* 0x1beb MOVG2 @r2+0xa250 r2 (next). */
void step_h2_movg2_r2_0xa250_r2_next(void)
{
    load8(mcu.r[2], ind_addr(2, 0xa250));
    mcu.pc = 0x1bef;
}

/* 0x1bef BPL 0x1bdd: loop over part chain. */
void step_h2_bpl_0x1bdd_loop(void)
{
    mcu.pc = bpl(0x1bdd, 0x1bf1);
}

/* 0x1bf1 BRA 0x1bf7. */
void step_h2_bra_0x1bf7(void)
{
    mcu.pc = 0x1bf7;
}

/* 0x1bf3 MOVG3 r2 -> @r3+0xa200 (write 0xff). */
void step_h2_movg3_r2_to_r3_0xa200_write_0xff(void)
{
    store8(ind_addr(3, 0xa200), (uint8_t)mcu.r[2]);
    mcu.pc = 0x1bf7;
}

/* 0x1bf7 MOVG2 r7++ r2 (restore desc). */
void step_h2_movg2_r7_r2_restore_desc(void)
{
    mcu.r[2] = MCU_PopStack();
    mcu.pc = 0x1bf9;
}

/* 0x1bf9 ADDQ #-1 @r3+0xa210 (active voices--). */
void step_h2_addq_1_r3_0xa210_active_voices(void)
{
    addq_byte(ind_addr(3, 0xa210), -1);
    mcu.pc = 0x1bfd;
}

/* 0x1bfd rts. */
void step_h2_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* H3 0x1b90..0x1bac, 9 PCs (desc_setup helper: reset state, pick a200) */
/* ======================================================================== */

/* 0x1b90 CLR @r2+0xa288. */
void step_h3_clr_r2_0xa288(void)
{
    clr8_mem(ind_addr(2, 0xa288));
    mcu.pc = 0x1b94;
}

/* 0x1b94 CLR @r2+0xa2a4. */
void step_h3_clr_r2_0xa2a4(void)
{
    clr8_mem(ind_addr(2, 0xa2a4));
    mcu.pc = 0x1b98;
}

/* 0x1b98 MOVG2 @r3+0xa200 r0. */
void step_h3_movg2_r3_0xa200_r0(void)
{
    load8(mcu.r[0], ind_addr(3, 0xa200));
    mcu.pc = 0x1b9c;
}

/* 0x1b9c BMI 0x1ba8: no current desc -> take this one. */
void step_h3_bmi_0x1ba8_no_current_desc(void)
{
    mcu.pc = bmi(0x1ba8, 0x1b9e);
}

/* 0x1b9e MOVG2 @r2+0xa314 r1 (desc age). */
void step_h3_movg2_r2_0xa314_r1_desc_age(void)
{
    load8(mcu.r[1], ind_addr(2, 0xa314));
    mcu.pc = 0x1ba2;
}

/* 0x1ba2 CMP @r0+0xa314 r1 (desc age vs current age). */
void step_h3_cmp_r0_0xa314_r1(void)
{
    cmp8_mem(mcu.r[1], ind_addr(0, 0xa314));
    mcu.pc = 0x1ba6;
}

/* 0x1ba6 BHI 0x1bac: current is smaller, keep it. */
void step_h3_bhi_0x1bac_current_smaller(void)
{
    mcu.pc = bhi(0x1bac, 0x1ba8);
}

/* 0x1ba8 MOVG3 r2 -> @r3+0xa200. */
void step_h3_movg3_r2_to_r3_0xa200(void)
{
    store8(ind_addr(3, 0xa200), (uint8_t)mcu.r[2]);
    mcu.pc = 0x1bac;
}

/* 0x1bac rts. */
void step_h3_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* pcm_start 0x516c..0x51b5, 21 PCs (release A/B and note_fill callee) */
/* ======================================================================== */

/* 0x516c CLR @r1+0xd15c (IRQ pending = 0). */
void step_pcm_start_clr_r1_0xd15c_irq_pending(void)
{
    clr8_mem(ind_addr(1, 0xd15c));
    mcu.pc = 0x5170;
}

/* 0x5170 MOVG3 r1 -> (dp,0xe03e) (select channel). */
void step_pcm_start_movg3_r1_to_dp_0xe03e_select_channel(void)
{
    store8(dp_addr(0xe03e), (uint8_t)mcu.r[1]);
    mcu.pc = 0x5174;
}

/* 0x5174 MOVG2 (dp,0xe032) r4 (latches ram2[sel][9]). */
void step_pcm_start_movg2_dp_0xe032_r4_latch9(void)
{
    load8(mcu.r[4], dp_addr(0xe032));
    mcu.pc = 0x5178;
}

/* 0x5178 MOVG2 (dp,0xe03a) r4 (word read_latch). */
void step_pcm_start_movg2_dp_0xe03a_r4_word_read_latch(void)
{
    load16(mcu.r[4], dp_addr(0xe03a));
    mcu.pc = 0x517c;
}

/* 0x517c MOVG2 (dp,0xe034) r5 (latches ram2[sel][10]). */
void step_pcm_start_movg2_dp_0xe034_r5_latch10(void)
{
    load8(mcu.r[5], dp_addr(0xe034));
    mcu.pc = 0x5180;
}

/* 0x5180 MOVG2 (dp,0xe03a) r5 (word read_latch). */
void step_pcm_start_movg2_dp_0xe03a_r5_word_read_latch(void)
{
    load16(mcu.r[5], dp_addr(0xe03a));
    mcu.pc = 0x5184;
}

/* 0x5184 ADD r1 r1. */
void step_pcm_start_add_r1_r1(void)
{
    add16_self(mcu.r[1]);
    mcu.pc = 0x5186;
}

/* 0x5186 MOVG2 @r1+0x64d6 r2 (AoS record pointer). */
void step_pcm_start_movg2_r1_0x64d6_r2_record_ptr(void)
{
    load16(mcu.r[2], ind_addr(1, 0x64d6));
    mcu.pc = 0x518a;
}

/* 0x518a CMP r4 r5 (GT compares r5 - r4). */
void step_pcm_start_cmp_r4_r5(void)
{
    cmp16_reg(mcu.r[5], mcu.r[4]);
    mcu.pc = 0x518c;
}

/* 0x518c BCC 0x519e: r5 >= r4 -> 0x14 port. */
void step_pcm_start_bcc_0x519e_r5_ge_r4(void)
{
    mcu.pc = bcc(0x519e, 0x518e);
}

/* 0x518e movi r6 #0x0012. */
void step_pcm_start_movi_r6_0x0012(void)
{
    movi16(mcu.r[6], 0x0012);
    mcu.pc = 0x5191;
}

/* 0x5191 MOVG #0x00b5 -> (dp,0xe016). */
void step_pcm_start_movg_0x00b5_to_dp_0xe016(void)
{
    mov_imm16(dp_addr(0xe016), 0x00b5);
    mcu.pc = 0x5197;
}

/* 0x5197 MOVG #0x00b5 -> @r2+0x1a. */
void step_pcm_start_movg_0x00b5_to_r2_0x1a(void)
{
    mov_imm16(ind_addr(2, 0x1a), 0x00b5);
    mcu.pc = 0x519c;
}

/* 0x519c BRA 0x51ac. */
void step_pcm_start_bra_0x51ac(void)
{
    mcu.pc = 0x51ac;
}

/* 0x519e movi r6 #0x0014. */
void step_pcm_start_movi_r6_0x0014(void)
{
    movi16(mcu.r[6], 0x0014);
    mcu.pc = 0x51a1;
}

/* 0x51a1 MOVG #0x00b5 -> (dp,0xe018). */
void step_pcm_start_movg_0x00b5_to_dp_0xe018(void)
{
    mov_imm16(dp_addr(0xe018), 0x00b5);
    mcu.pc = 0x51a7;
}

/* 0x51a7 MOVG #0x00b5 -> @r2+0x1e. */
void step_pcm_start_movg_0x00b5_to_r2_0x1e(void)
{
    mov_imm16(ind_addr(2, 0x1e), 0x00b5);
    mcu.pc = 0x51ac;
}

/* 0x51ac MOVG3 r6 -> @r2+0 (state[0]). */
void step_pcm_start_movg3_r6_to_r2_0_state0(void)
{
    store16(ind_addr(2, 0), mcu.r[6]);
    mcu.pc = 0x51af;
}

/* 0x51af MOVG3 r6 -> @r2+2. */
void step_pcm_start_movg3_r6_to_r2_2_state1(void)
{
    store16(ind_addr(2, 2), mcu.r[6]);
    mcu.pc = 0x51b2;
}

/* 0x51b2 MOVG3 r6 -> @r2+4. */
void step_pcm_start_movg3_r6_to_r2_4_state2(void)
{
    store16(ind_addr(2, 4), mcu.r[6]);
    mcu.pc = 0x51b5;
}

/* 0x51b5 rts. */
void step_pcm_start_rts(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */
} /* namespace mk2c */

/* ---- registration: explicit flat = pc (cp0) for all 199 PCs -------------- */

void MK2CPP_AllocFreeFillTables(void)
{
    /* Called directly from MK2CPP_PoolFillTables (native_pool.cpp) and again
     * from the self-registered module below; duplicate registration is fatal
     * (mk2cpp.cpp), so publish the table only once. */
    static bool registered = false;
    if (registered)
        return;
    registered = true;

    /* pool_pop 0x19ad..0x19c3, 7 PCs */
    MK2CPP_HandRegister(0x000019adu, &mk2c::step_pool_pop_movg2_dp_0xa42f_r1);
    MK2CPP_HandRegister(0x000019b1u, &mk2c::step_pool_pop_movg2_r1_0xa3d8_r0);
    MK2CPP_HandRegister(0x000019b5u, &mk2c::step_pool_pop_movg3_r0_to_dp_0xa42f);
    MK2CPP_HandRegister(0x000019b9u, &mk2c::step_pool_pop_bpl_0x19bf);
    MK2CPP_HandRegister(0x000019bbu, &mk2c::step_pool_pop_movg3_r0_to_dp_0xa430);
    MK2CPP_HandRegister(0x000019bfu, &mk2c::step_pool_pop_addq_1_dp_0xa42d);
    MK2CPP_HandRegister(0x000019c3u, &mk2c::step_pool_pop_rts);

    /* link 0x194c..0x19ac, 25 PCs */
    MK2CPP_HandRegister(0x0000194cu, &mk2c::step_link_movg2_r2_0xa2dc_r0_chain_tail);
    MK2CPP_HandRegister(0x00001950u, &mk2c::step_link_bpl_0x1978_chain_empty);
    MK2CPP_HandRegister(0x00001952u, &mk2c::step_link_movg3_r1_to_r2_0xa2c0_new_head);
    MK2CPP_HandRegister(0x00001956u, &mk2c::step_link_movg2_r1_0xd0a8_r0_start_prev);
    MK2CPP_HandRegister(0x0000195au, &mk2c::step_link_bmi_0x1961_no_start_prev);
    MK2CPP_HandRegister(0x0000195cu, &mk2c::step_link_movg_0xff_to_r0_0xd0c4_arm);
    MK2CPP_HandRegister(0x00001961u, &mk2c::step_link_movg2_r1_0xd0c4_r0_start_next);
    MK2CPP_HandRegister(0x00001965u, &mk2c::step_link_bmi_0x196c_no_start_next);
    MK2CPP_HandRegister(0x00001967u, &mk2c::step_link_movg_0xff_to_r0_0xd0a8_arm);
    MK2CPP_HandRegister(0x0000196cu, &mk2c::step_link_movg_0xff_to_r1_0xa410);
    MK2CPP_HandRegister(0x00001971u, &mk2c::step_link_movg_0xff_to_r1_0xd0a8);
    MK2CPP_HandRegister(0x00001976u, &mk2c::step_link_bra_0x1996);
    MK2CPP_HandRegister(0x00001978u, &mk2c::step_link_movg3_r0_to_r2_0xa2c0_old_tail);
    MK2CPP_HandRegister(0x0000197cu, &mk2c::step_link_movg3_r1_to_r0_0xa3f4);
    MK2CPP_HandRegister(0x00001980u, &mk2c::step_link_movg3_r1_to_r0_0xd0c4);
    MK2CPP_HandRegister(0x00001984u, &mk2c::step_link_movg3_r0_to_r1_0xa410);
    MK2CPP_HandRegister(0x00001988u, &mk2c::step_link_movg3_r0_to_r1_0xd0a8);
    MK2CPP_HandRegister(0x0000198cu, &mk2c::step_link_movg_0xff_to_r0_0xa410);
    MK2CPP_HandRegister(0x00001991u, &mk2c::step_link_movg_0xff_to_r0_0xd0a8);
    MK2CPP_HandRegister(0x00001996u, &mk2c::step_link_movg3_r1_to_r2_0xa2dc_new_tail);
    MK2CPP_HandRegister(0x0000199au, &mk2c::step_link_movg_0xff_to_r1_0xa3f4);
    MK2CPP_HandRegister(0x0000199fu, &mk2c::step_link_movg_0xff_to_r1_0xd0c4);
    MK2CPP_HandRegister(0x000019a4u, &mk2c::step_link_movg3_r3_to_r1_0xa368_slot_part);
    MK2CPP_HandRegister(0x000019a8u, &mk2c::step_link_movg3_r2_to_r1_0xa384_slot_desc);
    MK2CPP_HandRegister(0x000019acu, &mk2c::step_link_rts);

    /* release A 0x1823..0x187d, 27 PCs */
    MK2CPP_HandRegister(0x00001823u, &mk2c::step_release_a_bset_orc_0x0700_r0_iml7);
    MK2CPP_HandRegister(0x00001827u, &mk2c::step_release_a_stm_0x3e);
    MK2CPP_HandRegister(0x00001829u, &mk2c::step_release_a_jsr_0x516c_pcm_start);
    MK2CPP_HandRegister(0x0000182cu, &mk2c::step_release_a_ldm_0x3e);
    MK2CPP_HandRegister(0x0000182eu, &mk2c::step_release_a_movg_0x04_to_r1_0xd0e0_cmd_stop);
    MK2CPP_HandRegister(0x00001833u, &mk2c::step_release_a_bclr_andc_0xf8ff_r0_iml0);
    MK2CPP_HandRegister(0x00001837u, &mk2c::step_release_a_extu_r1);
    MK2CPP_HandRegister(0x00001839u, &mk2c::step_release_a_clr_r1_0xad0e);
    MK2CPP_HandRegister(0x0000183du, &mk2c::step_release_a_clr_r1_0xa3bc);
    MK2CPP_HandRegister(0x00001841u, &mk2c::step_release_a_clr_r1_0xa4b4);
    MK2CPP_HandRegister(0x00001845u, &mk2c::step_release_a_clr_r0);
    MK2CPP_HandRegister(0x00001847u, &mk2c::step_release_a_bsr16_to_0x1b44_h1);
    MK2CPP_HandRegister(0x0000184au, &mk2c::step_release_a_movg2_dp_0xa430_r0_free_tail);
    MK2CPP_HandRegister(0x0000184eu, &mk2c::step_release_a_bpl_0x185a_tail_valid);
    MK2CPP_HandRegister(0x00001850u, &mk2c::step_release_a_movg3_r1_to_dp_0xa42f_head);
    MK2CPP_HandRegister(0x00001854u, &mk2c::step_release_a_movg3_r0_to_r1_0xa3d8);
    MK2CPP_HandRegister(0x00001858u, &mk2c::step_release_a_bra_0x1863);
    MK2CPP_HandRegister(0x0000185au, &mk2c::step_release_a_movg3_r1_to_r0_0xa3d8_old_tail_next);
    MK2CPP_HandRegister(0x0000185eu, &mk2c::step_release_a_movg_0xff_to_r1_0xa3d8);
    MK2CPP_HandRegister(0x00001863u, &mk2c::step_release_a_movg3_r1_to_dp_0xa430_tail);
    MK2CPP_HandRegister(0x00001867u, &mk2c::step_release_a_movg_0x94_to_r1_0xa3a0_free);
    MK2CPP_HandRegister(0x0000186cu, &mk2c::step_release_a_addq_1_dp_0xa42d_count);
    MK2CPP_HandRegister(0x00001870u, &mk2c::step_release_a_bsr16_to_0x1bad_h2);
    MK2CPP_HandRegister(0x00001873u, &mk2c::step_release_a_addq_1_dp_0xa42c_shortfall);
    MK2CPP_HandRegister(0x00001877u, &mk2c::step_release_a_bpl_0x187d);
    MK2CPP_HandRegister(0x00001879u, &mk2c::step_release_a_clr_dp_0xa42c_clamp);
    MK2CPP_HandRegister(0x0000187du, &mk2c::step_release_a_rts);

    /* release B 0x187e..0x18cd, 24 PCs */
    MK2CPP_HandRegister(0x0000187eu, &mk2c::step_release_b_extu_r1);
    MK2CPP_HandRegister(0x00001880u, &mk2c::step_release_b_bset_orc_0x0700_r0_iml7);
    MK2CPP_HandRegister(0x00001884u, &mk2c::step_release_b_stm_0x3e);
    MK2CPP_HandRegister(0x00001886u, &mk2c::step_release_b_jsr_0x516c_pcm_start);
    MK2CPP_HandRegister(0x00001889u, &mk2c::step_release_b_ldm_0x3e);
    MK2CPP_HandRegister(0x0000188bu, &mk2c::step_release_b_movg_0x04_to_r1_0xd0e0_cmd_stop);
    MK2CPP_HandRegister(0x00001890u, &mk2c::step_release_b_bclr_andc_0xf8ff_r0_iml0);
    MK2CPP_HandRegister(0x00001894u, &mk2c::step_release_b_clr_r1_0xad0e);
    MK2CPP_HandRegister(0x00001898u, &mk2c::step_release_b_clr_r1_0xa3bc);
    MK2CPP_HandRegister(0x0000189cu, &mk2c::step_release_b_clr_r1_0xa4b4);
    MK2CPP_HandRegister(0x000018a0u, &mk2c::step_release_b_clr_r0);
    MK2CPP_HandRegister(0x000018a2u, &mk2c::step_release_b_bsr16_to_0x1b44_h1);
    MK2CPP_HandRegister(0x000018a5u, &mk2c::step_release_b_movg2_dp_0xa42f_r0_free_head);
    MK2CPP_HandRegister(0x000018a9u, &mk2c::step_release_b_bpl_0x18af_head_valid);
    MK2CPP_HandRegister(0x000018abu, &mk2c::step_release_b_movg3_r1_to_dp_0xa430_tail);
    MK2CPP_HandRegister(0x000018afu, &mk2c::step_release_b_movg3_r0_to_r1_0xa3d8_slot_next);
    MK2CPP_HandRegister(0x000018b3u, &mk2c::step_release_b_movg3_r1_to_dp_0xa42f_head);
    MK2CPP_HandRegister(0x000018b7u, &mk2c::step_release_b_movg_0x94_to_r1_0xa3a0_free);
    MK2CPP_HandRegister(0x000018bcu, &mk2c::step_release_b_addq_1_dp_0xa42d_count);
    MK2CPP_HandRegister(0x000018c0u, &mk2c::step_release_b_bsr16_to_0x1bad_h2);
    MK2CPP_HandRegister(0x000018c3u, &mk2c::step_release_b_addq_1_dp_0xa42c_shortfall);
    MK2CPP_HandRegister(0x000018c7u, &mk2c::step_release_b_bpl_0x18cd);
    MK2CPP_HandRegister(0x000018c9u, &mk2c::step_release_b_clr_dp_0xa42c_clamp);
    MK2CPP_HandRegister(0x000018cdu, &mk2c::step_release_b_rts);

    /* free_voice 0x19c4..0x1a23, 28 PCs */
    MK2CPP_HandRegister(0x000019c4u, &mk2c::step_free_btsti_r1_0xa3a0_7_already_free);
    MK2CPP_HandRegister(0x000019c8u, &mk2c::step_free_bne_0x1a23_already_free);
    MK2CPP_HandRegister(0x000019cau, &mk2c::step_free_sub_r1_0xad0e_0xff_channel_assigned);
    MK2CPP_HandRegister(0x000019cfu, &mk2c::step_free_beq_0x1a23_no_channel);
    MK2CPP_HandRegister(0x000019d1u, &mk2c::step_free_movg2_r1_0xa384_r2_desc);
    MK2CPP_HandRegister(0x000019d5u, &mk2c::step_free_movg2_r1_0xa368_r3_part);
    MK2CPP_HandRegister(0x000019d9u, &mk2c::step_free_clr_r1_0xa3bc);
    MK2CPP_HandRegister(0x000019ddu, &mk2c::step_free_clr_r1_0xa4b4);
    MK2CPP_HandRegister(0x000019e1u, &mk2c::step_free_cmp_r2_0xa2c0_r1_slot_vhead);
    MK2CPP_HandRegister(0x000019e5u, &mk2c::step_free_bne_0x19eb);
    MK2CPP_HandRegister(0x000019e7u, &mk2c::step_free_bclr_r2_0xa34c_1);
    MK2CPP_HandRegister(0x000019ebu, &mk2c::step_free_cmp_r2_0xa2dc_r1_slot_vtail);
    MK2CPP_HandRegister(0x000019efu, &mk2c::step_free_bne_0x19f5);
    MK2CPP_HandRegister(0x000019f1u, &mk2c::step_free_bclr_r2_0xa34c_0);
    MK2CPP_HandRegister(0x000019f5u, &mk2c::step_free_clr_r0);
    MK2CPP_HandRegister(0x000019f7u, &mk2c::step_free_bsr16_to_0x1b44_h1);
    MK2CPP_HandRegister(0x000019fau, &mk2c::step_free_movg2_dp_0xa430_r0_free_tail);
    MK2CPP_HandRegister(0x000019feu, &mk2c::step_free_bpl_0x1a0a);
    MK2CPP_HandRegister(0x00001a00u, &mk2c::step_free_movg3_r1_to_dp_0xa42f_head);
    MK2CPP_HandRegister(0x00001a04u, &mk2c::step_free_movg3_r0_to_r1_0xa3d8);
    MK2CPP_HandRegister(0x00001a08u, &mk2c::step_free_bra_0x1a13);
    MK2CPP_HandRegister(0x00001a0au, &mk2c::step_free_movg3_r1_to_r0_0xa3d8);
    MK2CPP_HandRegister(0x00001a0eu, &mk2c::step_free_movg_0xff_to_r1_0xa3d8);
    MK2CPP_HandRegister(0x00001a13u, &mk2c::step_free_movg3_r1_to_dp_0xa430_tail);
    MK2CPP_HandRegister(0x00001a17u, &mk2c::step_free_movg_0x94_to_r1_0xa3a0);
    MK2CPP_HandRegister(0x00001a1cu, &mk2c::step_free_addq_1_dp_0xa42d_count);
    MK2CPP_HandRegister(0x00001a20u, &mk2c::step_free_bsr16_to_0x1bad_h2);
    MK2CPP_HandRegister(0x00001a23u, &mk2c::step_free_rts);

    /* H1 0x1b44..0x1b8f, 21 PCs */
    MK2CPP_HandRegister(0x00001b44u, &mk2c::step_h1_movg3_r3_to_r7_save);
    MK2CPP_HandRegister(0x00001b46u, &mk2c::step_h1_clr_r3);
    MK2CPP_HandRegister(0x00001b48u, &mk2c::step_h1_movg2_r1_0xa3f4_r3_prev);
    MK2CPP_HandRegister(0x00001b4cu, &mk2c::step_h1_bmi_0x1b68_no_prev);
    MK2CPP_HandRegister(0x00001b4eu, &mk2c::step_h1_movg3_r3_to_r2_0xa2c0_vhead);
    MK2CPP_HandRegister(0x00001b52u, &mk2c::step_h1_movg_0xff_to_r3_0xa410);
    MK2CPP_HandRegister(0x00001b57u, &mk2c::step_h1_movg_0xff_to_r3_0xd0a8);
    MK2CPP_HandRegister(0x00001b5cu, &mk2c::step_h1_movg_0xff_to_r1_0xa3f4);
    MK2CPP_HandRegister(0x00001b61u, &mk2c::step_h1_movg_0xff_to_r1_0xd0c4);
    MK2CPP_HandRegister(0x00001b66u, &mk2c::step_h1_bra_0x1b8d);
    MK2CPP_HandRegister(0x00001b68u, &mk2c::step_h1_movg2_r1_0xa410_r3_next);
    MK2CPP_HandRegister(0x00001b6cu, &mk2c::step_h1_bpl_0x1b75_has_next);
    MK2CPP_HandRegister(0x00001b6eu, &mk2c::step_h1_movg_0xff_to_r2_0xa2c0_clear_head);
    MK2CPP_HandRegister(0x00001b73u, &mk2c::step_h1_bra_0x1b89);
    MK2CPP_HandRegister(0x00001b75u, &mk2c::step_h1_movg_0xff_to_r1_0xa410);
    MK2CPP_HandRegister(0x00001b7au, &mk2c::step_h1_movg_0xff_to_r1_0xd0a8);
    MK2CPP_HandRegister(0x00001b7fu, &mk2c::step_h1_movg_0xff_to_r3_0xa3f4);
    MK2CPP_HandRegister(0x00001b84u, &mk2c::step_h1_movg_0xff_to_r3_0xd0c4);
    MK2CPP_HandRegister(0x00001b89u, &mk2c::step_h1_movg3_r3_to_r2_0xa2dc_vtail);
    MK2CPP_HandRegister(0x00001b8du, &mk2c::step_h1_movg2_r7_r3_restore);
    MK2CPP_HandRegister(0x00001b8fu, &mk2c::step_h1_rts);

    /* unlink 0x1b23..0x1b43, 11 PCs */
    MK2CPP_HandRegister(0x00001b23u, &mk2c::step_unlink_movg2_r2_0xa250_r0_next);
    MK2CPP_HandRegister(0x00001b27u, &mk2c::step_unlink_movg2_r2_0xa26c_r1_prev);
    MK2CPP_HandRegister(0x00001b2bu, &mk2c::step_unlink_bpl_0x1b33_has_prev);
    MK2CPP_HandRegister(0x00001b2du, &mk2c::step_unlink_movg3_r0_to_r3_0xa220_part_head);
    MK2CPP_HandRegister(0x00001b31u, &mk2c::step_unlink_bra_0x1b37);
    MK2CPP_HandRegister(0x00001b33u, &mk2c::step_unlink_movg3_r0_to_r1_0xa250_prev_next);
    MK2CPP_HandRegister(0x00001b37u, &mk2c::step_unlink_bpl_0x1b3f_next_store_flags);
    MK2CPP_HandRegister(0x00001b39u, &mk2c::step_unlink_movg3_r1_to_r3_0xa230_part_tail);
    MK2CPP_HandRegister(0x00001b3du, &mk2c::step_unlink_bra_0x1b43);
    MK2CPP_HandRegister(0x00001b3fu, &mk2c::step_unlink_movg3_r1_to_r0_0xa26c_next_prev);
    MK2CPP_HandRegister(0x00001b43u, &mk2c::step_unlink_rts);

    /* H2 0x1bad..0x1bfd, 26 PCs */
    MK2CPP_HandRegister(0x00001badu, &mk2c::step_h2_tst_r2_0xa2c0_chain_empty);
    MK2CPP_HandRegister(0x00001bb1u, &mk2c::step_h2_bpl_0x1bf9_chain_busy);
    MK2CPP_HandRegister(0x00001bb3u, &mk2c::step_h2_bsr16_to_0x1b23_unlink);
    MK2CPP_HandRegister(0x00001bb6u, &mk2c::step_h2_movg2_dp_0xa42e_r0_old_desc_head);
    MK2CPP_HandRegister(0x00001bbau, &mk2c::step_h2_movg3_r0_to_r2_0xa250_desc_next);
    MK2CPP_HandRegister(0x00001bbeu, &mk2c::step_h2_movg3_r2_to_dp_0xa42e_head);
    MK2CPP_HandRegister(0x00001bc2u, &mk2c::step_h2_movg_0x94_to_r2_0xa288_desc_free);
    MK2CPP_HandRegister(0x00001bc7u, &mk2c::step_h2_tst_r3_0xa200_current_desc);
    MK2CPP_HandRegister(0x00001bcbu, &mk2c::step_h2_bmi_0x1bf9_no_current_desc);
    MK2CPP_HandRegister(0x00001bcdu, &mk2c::step_h2_cmp_r3_0xa200_r2);
    MK2CPP_HandRegister(0x00001bd1u, &mk2c::step_h2_bne_0x1bf9);
    MK2CPP_HandRegister(0x00001bd3u, &mk2c::step_h2_movg3_r2_to_r7_save_desc);
    MK2CPP_HandRegister(0x00001bd5u, &mk2c::step_h2_move_r0_0x7f_candidate_age);
    MK2CPP_HandRegister(0x00001bd7u, &mk2c::step_h2_movg2_r3_0xa220_r2_part_head);
    MK2CPP_HandRegister(0x00001bdbu, &mk2c::step_h2_bmi_0x1bf3_empty_chain);
    MK2CPP_HandRegister(0x00001bddu, &mk2c::step_h2_cmp_r2_0xa314_r0);
    MK2CPP_HandRegister(0x00001be1u, &mk2c::step_h2_bls_0x1beb_candidate_le_r0);
    MK2CPP_HandRegister(0x00001be3u, &mk2c::step_h2_movg2_r2_0xa314_r0_new_candidate);
    MK2CPP_HandRegister(0x00001be7u, &mk2c::step_h2_movg3_r2_to_r3_0xa200);
    MK2CPP_HandRegister(0x00001bebu, &mk2c::step_h2_movg2_r2_0xa250_r2_next);
    MK2CPP_HandRegister(0x00001befu, &mk2c::step_h2_bpl_0x1bdd_loop);
    MK2CPP_HandRegister(0x00001bf1u, &mk2c::step_h2_bra_0x1bf7);
    MK2CPP_HandRegister(0x00001bf3u, &mk2c::step_h2_movg3_r2_to_r3_0xa200_write_0xff);
    MK2CPP_HandRegister(0x00001bf7u, &mk2c::step_h2_movg2_r7_r2_restore_desc);
    MK2CPP_HandRegister(0x00001bf9u, &mk2c::step_h2_addq_1_r3_0xa210_active_voices);
    MK2CPP_HandRegister(0x00001bfdu, &mk2c::step_h2_rts);

    /* H3 0x1b90..0x1bac, 9 PCs */
    MK2CPP_HandRegister(0x00001b90u, &mk2c::step_h3_clr_r2_0xa288);
    MK2CPP_HandRegister(0x00001b94u, &mk2c::step_h3_clr_r2_0xa2a4);
    MK2CPP_HandRegister(0x00001b98u, &mk2c::step_h3_movg2_r3_0xa200_r0);
    MK2CPP_HandRegister(0x00001b9cu, &mk2c::step_h3_bmi_0x1ba8_no_current_desc);
    MK2CPP_HandRegister(0x00001b9eu, &mk2c::step_h3_movg2_r2_0xa314_r1_desc_age);
    MK2CPP_HandRegister(0x00001ba2u, &mk2c::step_h3_cmp_r0_0xa314_r1);
    MK2CPP_HandRegister(0x00001ba6u, &mk2c::step_h3_bhi_0x1bac_current_smaller);
    MK2CPP_HandRegister(0x00001ba8u, &mk2c::step_h3_movg3_r2_to_r3_0xa200);
    MK2CPP_HandRegister(0x00001bacu, &mk2c::step_h3_rts);

    /* pcm_start 0x516c..0x51b5, 21 PCs */
    MK2CPP_HandRegister(0x0000516cu, &mk2c::step_pcm_start_clr_r1_0xd15c_irq_pending);
    MK2CPP_HandRegister(0x00005170u, &mk2c::step_pcm_start_movg3_r1_to_dp_0xe03e_select_channel);
    MK2CPP_HandRegister(0x00005174u, &mk2c::step_pcm_start_movg2_dp_0xe032_r4_latch9);
    MK2CPP_HandRegister(0x00005178u, &mk2c::step_pcm_start_movg2_dp_0xe03a_r4_word_read_latch);
    MK2CPP_HandRegister(0x0000517cu, &mk2c::step_pcm_start_movg2_dp_0xe034_r5_latch10);
    MK2CPP_HandRegister(0x00005180u, &mk2c::step_pcm_start_movg2_dp_0xe03a_r5_word_read_latch);
    MK2CPP_HandRegister(0x00005184u, &mk2c::step_pcm_start_add_r1_r1);
    MK2CPP_HandRegister(0x00005186u, &mk2c::step_pcm_start_movg2_r1_0x64d6_r2_record_ptr);
    MK2CPP_HandRegister(0x0000518au, &mk2c::step_pcm_start_cmp_r4_r5);
    MK2CPP_HandRegister(0x0000518cu, &mk2c::step_pcm_start_bcc_0x519e_r5_ge_r4);
    MK2CPP_HandRegister(0x0000518eu, &mk2c::step_pcm_start_movi_r6_0x0012);
    MK2CPP_HandRegister(0x00005191u, &mk2c::step_pcm_start_movg_0x00b5_to_dp_0xe016);
    MK2CPP_HandRegister(0x00005197u, &mk2c::step_pcm_start_movg_0x00b5_to_r2_0x1a);
    MK2CPP_HandRegister(0x0000519cu, &mk2c::step_pcm_start_bra_0x51ac);
    MK2CPP_HandRegister(0x0000519eu, &mk2c::step_pcm_start_movi_r6_0x0014);
    MK2CPP_HandRegister(0x000051a1u, &mk2c::step_pcm_start_movg_0x00b5_to_dp_0xe018);
    MK2CPP_HandRegister(0x000051a7u, &mk2c::step_pcm_start_movg_0x00b5_to_r2_0x1e);
    MK2CPP_HandRegister(0x000051acu, &mk2c::step_pcm_start_movg3_r6_to_r2_0_state0);
    MK2CPP_HandRegister(0x000051afu, &mk2c::step_pcm_start_movg3_r6_to_r2_2_state1);
    MK2CPP_HandRegister(0x000051b2u, &mk2c::step_pcm_start_movg3_r6_to_r2_4_state2);
    MK2CPP_HandRegister(0x000051b5u, &mk2c::step_pcm_start_rts);
}

/* Self-registration (parallel safe): no shared aggregator file is edited. */
namespace
{
struct AllocFreeSelfRegister
{
    AllocFreeSelfRegister() { mk2c::hand_register_module(&MK2CPP_AllocFreeFillTables); }
};

AllocFreeSelfRegister g_allocfree_self_register;
} /* anonymous namespace */
