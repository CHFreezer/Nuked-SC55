/*
 * HAND voice/note_chain_tail -- note/descriptor chain tail and PCM stop.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * M4 closure gap slice (out/m4/18_closure_gap.md 4.2 items 18-24 and 5 P1-6):
 * the second half of the note descriptor chain, per-instruction L0 entries
 * (one host step per H8 instruction, each returns 1):
 *
 *   B8  0x16f4 steal        0x16f4..0x173d   27 PCs
 *   B9  0x173e key_on       0x173e..0x179b   34 PCs
 *   B10 0x179c slot_select  0x179c..0x17ec   27 PCs
 *   B11 0x17ed note_helper  0x17ed..0x1805    9 PCs
 *   B12 0x180d aux_alloc    0x180d..0x1822    7 PCs
 *   B15 0x18ce desc_setup   0x18ce..0x194b   38 PCs
 *   B19 0x1a4d cleanup_seq  0x1a4d..0x1a6b   10 PCs
 *   C3  0x5238 pcm_stop     0x5238..0x52c8   48 PCs
 *
 * 200 registered PCs total. No L1 blocks: a multi-instruction step defers the
 * host tail (interrupt poll/TIMER_Clock/SM_Update/midiseq_poll) and breaks the
 * -midiseq MIDI byte timing on real songs (voice_materialize.cpp header,
 * MATERIALIZE_STATUS.md "root cause 2"), so everything is one entry per PC.
 *
 * Semantic rewrite (2026-09-13, M4 closure step 2): all eight routines are now
 * named per-PC entries registered through {pc, fn} tables (no switch fallback).
 *
 * Source of truth is the ROM bytes, not only the executed trace: 0x16f4 B8,
 * 0x173e B9, 0x179c B10, 0x18ce B15 and 0x5238 C3 each contain arms that the
 * frozen dasm_full run never executed (union trace). Their decodes come from
 * build/rom1.bin (verified sha256 above) plus the GT opcode implementation
 * (src/mcu_opcodes.cpp); confidence C (ROM bytes + interpreter). In particular:
 *   B8   0x1706/0x170a/0x170e/0x1710/0x1712 and 0x1733/0x1735/0x1739;
 *   B9   0x178f/0x1793/0x1797;
 *   pcm_stop 0x5293/0x5297 (the BNE arm over 0x528a) and 0x5299..0x52b2.
 * The routines body is transcribed instruction by instruction; the only
 * callees outside this file are already-hand routines (note_helper 0x17ed is
 * ours; release A 0x1823 and release B 0x187e, H1 0x1b44, H2 0x1bad, H3
 * 0x1b90, pool_pop 0x19ad, link 0x194c are native_allocfree.cpp). bsr/jsr
 * push the continuation PC and the callee's rts pops it, so the stack bytes
 * are byte-identical to stock. pcm_stop's BRA exits land on 0x51fc (the C2
 * dispatcher, another slice) and are written as plain pc writes (no stack).
 *
 * Data model: all state stays in page-0 SRAM / device registers and is
 * accessed through MCU_Read/MCU_Write exactly like the interpreter; word
 * operand accesses keep the stock odd-address ADDRESS_ERROR check
 * (src/mcu_opcodes.cpp MCU_Operand_Read/Write). pcm_stop's (br,disp8)
 * MOVS/MOVL accesses go through MCU_Read/MCU_Write so the -pcmtrace log and
 * the PCM/ext device routing stay identical to the stock instruction path.
 *
 * Flags: every helper mirrors the GT opcode body it replaces (MCU_ADD_Common /
 * MCU_SUB_Common / MCU_SetStatusCommon), including which flag a given
 * instruction leaves untouched (e.g. MOVG2/MOVG3 do not write C). IML windows:
 * MOVG2's BSET_ORC/BCLR_ANDC pair (not present here except pcm_stop 0x526f/
 * 0x52be) is replayed through the CCR with ex_ignore, one step each.
 *
 * 0x1806..0x180c (bsr 0x1823; TST 0xa42c; rts), 0x1a24..0x1a4c (the ungated
 * twin of cleanup_seq) and 0x52ca (rts of the dispatcher arm) are NOT
 * registered here: they are outside the assigned entry list and stay on
 * gen/interpreter (a later owner can take them without collision).
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_interrupt.h"
#include "mcu_opcodes.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as native_allocfree.cpp / pcm_enable.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

/* Single-entry build switch: 0 makes this slice not register at all (pure
 * gen/interpreter fallback) without touching other files. */
#define MK2CPP_HAND_NOTE_CHAIN_TAIL 1

namespace mk2c {
namespace {

/* ---- effective addresses (src/mcu_opcodes.cpp:543) ------------------------ */

/* @rN+disp: page from dp (r0-r3), ep (r4/r5) or tp (r6/r7), sum mod 0x10000. */
uint32_t ind_addr(uint32_t reg, uint16_t disp)
{
    uint8_t page = (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
    return ((uint32_t)page << 16) | (uint16_t)(mcu.r[reg] + disp);
}

/* (dp,disp): absolute through the DP page register. */
uint32_t dp_addr(uint16_t disp) { return ((uint32_t)mcu.dp << 16) | disp; }

/* (br,disp8): absolute through the BR register (pcm_stop MOVS/MOVL form). */
uint16_t br_addr(uint8_t disp) { return (uint16_t)((mcu.br << 8) | disp); }

/* ---- single-instruction operations (GT semantics) ------------------------- */

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG2 (opcode 0x10) memory/register -> Rd */
void load8(uint16_t &reg, uint32_t addr)
{
    uint8_t value = MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

void load16(uint16_t &reg, uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t value = MCU_Read16(addr);
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* MOVG3 (opcode 0x12) Rs -> memory */
void store8(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
    MCU_SetStatusCommon(value, 0);
}

void store16(uint32_t addr, uint16_t value)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, value);
    MCU_SetStatusCommon(value, 1);
}

/* MOVG #imm -> memory (opcode 0x06: sign-extended byte; 0x07: word). */
void mov_imm8(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
    MCU_SetStatusCommon(value, 0);
}

/* MOVG2 register-register (byte keeps the destination high byte). */
void mov_reg8(uint16_t &dst, uint16_t src)
{
    uint8_t data = (uint8_t)src;
    dst = (uint16_t)((dst & 0xff00u) | data);
    MCU_SetStatusCommon(data, 0);
}

void mov_reg16(uint16_t &dst, uint16_t src)
{
    dst = src;
    MCU_SetStatusCommon(src, 1);
}

/* SUB (opcode 0x06) Rd - <EA>, written back to Rd. */
void sub8_reg_mem(uint16_t &reg, uint32_t addr)
{
    uint16_t old = reg;
    int32_t value = MCU_SUB_Common(old, MCU_Read(addr), 0, 0);
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)value));
}

void sub16_reg_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_SUB_Common(reg, imm, 0, 1);
}

/* SUB @addr #imm (MOVG_Immediate ore 4/5): flags only, no write. */
void sub8_mem_imm_flags(uint32_t addr, uint8_t imm)
{
    MCU_SUB_Common(MCU_Read(addr), imm, 0, 0);
}

void sub16_mem_imm_flags(uint32_t addr, uint16_t imm)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_SUB_Common(MCU_Read16(addr), imm, 0, 1);
}

/* CMP <EA> Rd / CMP Rs Rd: flags only. */
void cmp8_reg_mem(uint16_t reg, uint32_t addr)
{
    MCU_SUB_Common(reg, MCU_Read(addr), 0, 0);
}

void cmp8_regs(uint16_t t1, uint16_t t2)
{
    MCU_SUB_Common(t1, t2, 0, 0);
}

/* Short CMP rN,#imm (opcode 0x40..0x4f, byte form). */
void cmp8_short_imm(uint16_t reg, uint8_t imm)
{
    MCU_SUB_Common(reg, imm, 0, 0);
}

/* ADDQ (opcode 0x08..0x0f) byte/word operand. */
void addq8_mem(uint32_t addr, int delta)
{
    int32_t value = MCU_ADD_Common(MCU_Read(addr), delta, 0, 0);
    MCU_Write(addr, (uint8_t)value);
}

void addq8_reg(uint16_t &reg, int delta)
{
    uint16_t old = reg;
    int32_t value = MCU_ADD_Common(old & 0xffu, delta, 0, 0);
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)value));
}

/* ADD rD rD (word). */
void add16_self(uint16_t &reg)
{
    reg = (uint16_t)MCU_ADD_Common(reg, reg, 0, 1);
}

/* CLR @addr / CLR rN / TST @addr (opcode block 0x10..0x1f). */
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

void tst8_mem(uint32_t addr)
{
    MCU_SetStatusCommon(MCU_Read(addr), 0);
    MCU_SetStatus(0, STATUS_C);
}

/* EXTU rN: zero-extend the low byte, N=0, Z=(value==0), V=0, C=0. */
void extu(uint16_t &reg)
{
    uint16_t value = (uint8_t)reg;
    reg = value;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(value == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* SWAP rN: swap bytes, status = common(data, WORD). */
void swap_reg(uint16_t &reg)
{
    uint32_t data = reg;
    data = ((data & 0xffu) << 8) | ((data >> 8) & 0xffu);
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* BTSTI @addr #bit: Z = (bit == 0), other flags untouched. */
void btsti(uint32_t addr, uint8_t bit)
{
    MCU_SetStatus((MCU_Read(addr) & (1u << bit)) == 0, STATUS_Z);
}

/* BCLR_ANDC #0xf8ff on CCR (IML window + ex_ignore). */
void bclr_andc_iml0(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
}

/* ---- conditional branches (GT MCU_Jump_Bcc, src/mcu_opcodes.cpp:231) ------ */

uint16_t bpl(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? fall : taken; }
uint16_t bmi(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? taken : fall; }
uint16_t bne(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? fall : taken; }
uint16_t beq(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? taken : fall; }
uint16_t bls(uint16_t taken, uint16_t fall)
{
    return (mcu.sr & (STATUS_C | STATUS_Z)) ? taken : fall;
}

/* BGT: taken when Z=0 and N==V (unsigned/ordered form used by key_on). */
uint16_t bgt(uint16_t taken, uint16_t fall)
{
    int nv = ((mcu.sr & STATUS_N) != 0) ^ ((mcu.sr & STATUS_V) != 0);
    return ((mcu.sr & STATUS_Z) == 0 && nv == 0) ? taken : fall;
}

uint16_t ble(uint16_t taken, uint16_t fall)
{
    int nv = ((mcu.sr & STATUS_N) != 0) ^ ((mcu.sr & STATUS_V) != 0);
    return ((mcu.sr & STATUS_Z) != 0 || nv != 0) ? taken : fall;
}

/* ---- pcm_stop (br,disp8) MOVS/MOVL forms (src/mcu_opcodes.cpp:755-800) ---- */

/* mov.b rN,@(br,disp8) */
void movs8_br(uint8_t reg, uint8_t disp)
{
    uint8_t data = (uint8_t)mcu.r[reg];
    MCU_Write(br_addr(disp), data);
    MCU_SetStatusCommon(data, 0);
}

/* mov.b @(br,disp8),rN */
void movl8_br(uint8_t reg, uint8_t disp)
{
    uint8_t data = MCU_Read(br_addr(disp));
    mcu.r[reg] = (uint16_t)((mcu.r[reg] & 0xff00u) | data);
    MCU_SetStatusCommon(data, 0);
}

/* mov.w @(br,disp8),rN */
void movlw16_br(uint8_t reg, uint8_t disp)
{
    uint16_t addr = br_addr(disp);
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t data = MCU_Read16(addr);
    mcu.r[reg] = data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVG #imm16 -> (br,disp8) (opcode 0x07, br absolute word). */
void mov_imm16_br(uint8_t disp, uint16_t value)
{
    uint16_t addr = br_addr(disp);
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, value);
    MCU_SetStatusCommon(value, 1);
}

/* ---- B8 steal 0x16f4..0x173d (27 PCs) ------------------------------------- */
/* ROM 0x16f4..0x173d:
 *   f3 a2 20 82 2b 41 f2 a2 88 04 00 27 13 1e 00 e9 2f 37 f3 a2 10 80
 *   f3 80 18 30 2f 0a a5 82 2b 06 f2 a2 50 82 2a e0 f3 a2 20 82 2b 1b
 *   1e 00 ca 2f 18 f3 a2 10 80 f3 80 18 30 2f 0c a5 82 2a ed 20 06
 *   f2 a2 50 82 2a e5 50 01 19
 * 0x1706/0x170a/0x170e/0x1710/0x1712 and 0x1733/0x1735/0x1739 are the
 * unexecuted arms of the frozen run (C: ROM bytes). */
/* 0x16F4 mov.b @r3+0xa220,r2 -- part descriptor chain head. */
void step_steal_load_head(void) { load8(mcu.r[2], ind_addr(3, 0xa220)); mcu.pc = 0x16f8; }
/* 0x16F8 bmi -> 0x173b -- chain empty. */
void step_steal_branch_empty(void) { mcu.pc = bmi(0x173b, 0x16fa); }
/* 0x16FA sub.b @r2+0xa288,#0 -- descriptor free? (flags only). */
void step_steal_compare_free(void) { sub8_mem_imm_flags(ind_addr(2, 0xa288), 0x00); mcu.pc = 0x16ff; }
/* 0x16FF beq -> 0x1714 -- free descriptor. */
void step_steal_branch_free(void) { mcu.pc = beq(0x1714, 0x1701); }
/* 0x1701 bsr #0x17ed -- note_helper releases the descriptor voices. */
void step_steal_call_helper(void) { MCU_PushStack(0x1704); mcu.pc = 0x17ed; }
/* 0x1704 ble -> 0x173d -- shortfall >= 0. */
void step_steal_branch_shortfall(void) { mcu.pc = ble(0x173d, 0x1706); }
/* 0x1706 mov.b @r3+0xa210,r0 -- active voice count (unexecuted arm). */
void step_steal_load_active(void) { load8(mcu.r[0], ind_addr(3, 0xa210)); mcu.pc = 0x170a; }
/* 0x170A sub.b @r3+0x8018,r0 -- vs the voice limit (unexecuted arm). */
void step_steal_sub_limit(void) { sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018)); mcu.pc = 0x170e; }
/* 0x170E ble -> 0x171a. */
void step_steal_branch_limit(void) { mcu.pc = ble(0x171a, 0x1710); }
/* 0x1710 mov.b r5,r2 (unexecuted arm). */
void step_steal_copy_voice(void) { mov_reg8(mcu.r[2], mcu.r[5]); mcu.pc = 0x1712; }
/* 0x1712 bmi -> 0x171a. */
void step_steal_branch_sign(void) { mcu.pc = bmi(0x171a, 0x1714); }
/* 0x1714 mov.b @r2+0xa250,r2 -- next descriptor. */
void step_steal_next_desc(void) { load8(mcu.r[2], ind_addr(2, 0xa250)); mcu.pc = 0x1718; }
/* 0x1718 bpl -> 0x16fa -- walk the chain. */
void step_steal_loop_chain(void) { mcu.pc = bpl(0x16fa, 0x171a); }
/* 0x171A mov.b @r3+0xa220,r2 -- restart from head. */
void step_steal_restart_head(void) { load8(mcu.r[2], ind_addr(3, 0xa220)); mcu.pc = 0x171e; }
/* 0x171E bmi -> 0x173b. */
void step_steal_branch_restart_empty(void) { mcu.pc = bmi(0x173b, 0x1720); }
/* 0x1720 bsr #0x17ed -- note_helper. */
void step_steal_call_helper2(void) { MCU_PushStack(0x1723); mcu.pc = 0x17ed; }
/* 0x1723 ble -> 0x173d. */
void step_steal_branch_shortfall2(void) { mcu.pc = ble(0x173d, 0x1725); }
/* 0x1725 mov.b @r3+0xa210,r0. */
void step_steal_load_active2(void) { load8(mcu.r[0], ind_addr(3, 0xa210)); mcu.pc = 0x1729; }
/* 0x1729 sub.b @r3+0x8018,r0. */
void step_steal_sub_limit2(void) { sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018)); mcu.pc = 0x172d; }
/* 0x172D ble -> 0x173b. */
void step_steal_branch_limit2(void) { mcu.pc = ble(0x173b, 0x172f); }
/* 0x172F mov.b r5,r2. */
void step_steal_copy_voice2(void) { mov_reg8(mcu.r[2], mcu.r[5]); mcu.pc = 0x1731; }
/* 0x1731 bpl -> 0x1720 -- retry until the sign is set. */
void step_steal_loop_retry(void) { mcu.pc = bpl(0x1720, 0x1733); }
/* 0x1733 bra -> 0x173b (unexecuted arm). */
void step_steal_branch_done(void) { mcu.pc = 0x173b; }
/* 0x1735 mov.b @r2+0xa250,r2 (unexecuted arm). */
void step_steal_next_desc2(void) { load8(mcu.r[2], ind_addr(2, 0xa250)); mcu.pc = 0x1739; }
/* 0x1739 bpl -> 0x1720 (unexecuted arm). */
void step_steal_loop_retry2(void) { mcu.pc = bpl(0x1720, 0x173b); }
/* 0x173B move r0,#0x01 -- chain exhausted. */
void step_steal_set_one(void) { mov_reg8(mcu.r[0], 0x01); mcu.pc = 0x173d; }
/* 0x173D rts. */
void step_steal_return(void) { mcu.pc = MCU_PopStack(); }

/* ---- B9 key_on 0x173e..0x179b (34 PCs) ------------------------------------ */
/* ROM 0x173e..0x179b:
 *   f3 a2 00 84 56 00 0e 56 2b 15 f1 a3 84 82 1e 00 9e 2f 4a f3 a2 10 80
 *   f3 80 18 30 2e e7 20 3c 56 01 1e 00 3a 2b 15 f1 a3 84 82 1e 00 82
 *   2f 2e f3 a2 10 80 f3 80 18 30 2e e6 20 20 f3 a2 00 16 2b 1a 56 02
 *   1e 00 18 2b 13 f1 a3 84 82 1e 00 60 2f 0c f3 a2 10 80 f3 80 18 30
 *   2e e6 50 01 19
 * Three passes r6 = 0/1/2 (normal/effect/low); 0x178f/0x1793/0x1797 are the
 * unexecuted third-pass tail (C: ROM bytes). */
/* 0x173E mov.b @r3+0xa200,r4 -- current descriptor. */
void step_key_on_load_current(void) { load8(mcu.r[4], ind_addr(3, 0xa200)); mcu.pc = 0x1742; }
/* 0x1742 move r6,#0x00 -- pass 0 (normal). */
void step_key_on_pass0(void) { mov_reg8(mcu.r[6], 0x00); mcu.pc = 0x1744; }
/* 0x1744 bsr #0x179c -- slot_select. */
void step_key_on_call_slot0(void) { MCU_PushStack(0x1746); mcu.pc = 0x179c; }
/* 0x1746 bmi -> 0x175d -- no slot, try pass 1. */
void step_key_on_branch_no_slot0(void) { mcu.pc = bmi(0x175d, 0x1748); }
/* 0x1748 mov.b @r1+0xa384,r2 -- selected descriptor. */
void step_key_on_load_selected0(void) { load8(mcu.r[2], ind_addr(1, 0xa384)); mcu.pc = 0x174c; }
/* 0x174C bsr #0x17ed -- note_helper. */
void step_key_on_call_helper0(void) { MCU_PushStack(0x174f); mcu.pc = 0x17ed; }
/* 0x174F ble -> 0x179b -- shortfall >= 0: done. */
void step_key_on_branch_shortfall0(void) { mcu.pc = ble(0x179b, 0x1751); }
/* 0x1751 mov.b @r3+0xa210,r0. */
void step_key_on_load_active0(void) { load8(mcu.r[0], ind_addr(3, 0xa210)); mcu.pc = 0x1755; }
/* 0x1755 sub.b @r3+0x8018,r0. */
void step_key_on_sub_limit0(void) { sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018)); mcu.pc = 0x1759; }
/* 0x1759 bgt -> 0x1742 -- above the limit, retry pass 0. */
void step_key_on_branch_above0(void) { mcu.pc = bgt(0x1742, 0x175b); }
/* 0x175B bra -> 0x1799 -- success. */
void step_key_on_branch_success0(void) { mcu.pc = 0x1799; }
/* 0x175D move r6,#0x01 -- pass 1 (effect). */
void step_key_on_pass1(void) { mov_reg8(mcu.r[6], 0x01); mcu.pc = 0x175f; }
/* 0x175F bsr #0x179c. */
void step_key_on_call_slot1(void) { MCU_PushStack(0x1762); mcu.pc = 0x179c; }
/* 0x1762 bmi -> 0x1779 -- no slot, try pass 2. */
void step_key_on_branch_no_slot1(void) { mcu.pc = bmi(0x1779, 0x1764); }
/* 0x1764 mov.b @r1+0xa384,r2. */
void step_key_on_load_selected1(void) { load8(mcu.r[2], ind_addr(1, 0xa384)); mcu.pc = 0x1768; }
/* 0x1768 bsr #0x17ed. */
void step_key_on_call_helper1(void) { MCU_PushStack(0x176b); mcu.pc = 0x17ed; }
/* 0x176B ble -> 0x179b. */
void step_key_on_branch_shortfall1(void) { mcu.pc = ble(0x179b, 0x176d); }
/* 0x176D mov.b @r3+0xa210,r0. */
void step_key_on_load_active1(void) { load8(mcu.r[0], ind_addr(3, 0xa210)); mcu.pc = 0x1771; }
/* 0x1771 sub.b @r3+0x8018,r0. */
void step_key_on_sub_limit1(void) { sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018)); mcu.pc = 0x1775; }
/* 0x1775 bgt -> 0x175d -- retry pass 1. */
void step_key_on_branch_above1(void) { mcu.pc = bgt(0x175d, 0x1777); }
/* 0x1777 bra -> 0x1799. */
void step_key_on_branch_success1(void) { mcu.pc = 0x1799; }
/* 0x1779 tst.b @r3+0xa200 -- current descriptor valid? */
void step_key_on_test_current(void) { tst8_mem(ind_addr(3, 0xa200)); mcu.pc = 0x177d; }
/* 0x177D bmi -> 0x1799 -- no current descriptor: success anyway. */
void step_key_on_branch_no_current(void) { mcu.pc = bmi(0x1799, 0x177f); }
/* 0x177F move r6,#0x02 -- pass 2 (low voices). */
void step_key_on_pass2(void) { mov_reg8(mcu.r[6], 0x02); mcu.pc = 0x1781; }
/* 0x1781 bsr #0x179c. */
void step_key_on_call_slot2(void) { MCU_PushStack(0x1784); mcu.pc = 0x179c; }
/* 0x1784 bmi -> 0x1799 -- no slot: success. */
void step_key_on_branch_no_slot2(void) { mcu.pc = bmi(0x1799, 0x1786); }
/* 0x1786 mov.b @r1+0xa384,r2. */
void step_key_on_load_selected2(void) { load8(mcu.r[2], ind_addr(1, 0xa384)); mcu.pc = 0x178a; }
/* 0x178A bsr #0x17ed. */
void step_key_on_call_helper2(void) { MCU_PushStack(0x178d); mcu.pc = 0x17ed; }
/* 0x178D ble -> 0x179b. */
void step_key_on_branch_shortfall2(void) { mcu.pc = ble(0x179b, 0x178f); }
/* 0x178F mov.b @r3+0xa210,r0 (unexecuted arm). */
void step_key_on_load_active2(void) { load8(mcu.r[0], ind_addr(3, 0xa210)); mcu.pc = 0x1793; }
/* 0x1793 sub.b @r3+0x8018,r0 (unexecuted arm). */
void step_key_on_sub_limit2(void) { sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018)); mcu.pc = 0x1797; }
/* 0x1797 bgt -> 0x177f -- retry pass 2 (unexecuted arm). */
void step_key_on_branch_above2(void) { mcu.pc = bgt(0x177f, 0x1799); }
/* 0x1799 move r0,#0x01 -- success. */
void step_key_on_set_one(void) { mov_reg8(mcu.r[0], 0x01); mcu.pc = 0x179b; }
/* 0x179B rts. */
void step_key_on_return(void) { mcu.pc = MCU_PopStack(); }

/* ---- B10 slot_select 0x179c..0x17ec (27 PCs) ------------------------------ */
/* ROM 0x179c..0x17ec:
 *   15 a4 38 06 ff 50 ff f3 a2 20 82 2b 3f 46 02 27 0f 46 01 27 07
 *   f2 a2 88 04 00 27 2a a2 74 27 26 f2 a2 dc 81 f1 ad 0e 70 23 08
 *   f1 ad 0e 80 15 a4 38 91 f1 a4 10 81 2b 0e f1 ad 0e 70 23 08
 *   f1 ad 0e 80 15 a4 38 91 f2 a2 50 82 2a c1 15 a4 38 81 19
 * Selects the PCM channel (a438 := min ad0e voice index) over the part's
 * descriptor chain; r6 selects the pass (0/1/2), r0 is the running minimum. */
/* 0x179C movg #0xff,(dp,0xa438) -- selected slot = none. */
void step_slot_clear_result(void) { mov_imm8(dp_addr(0xa438), 0xff); mcu.pc = 0x17a1; }
/* 0x17A1 move r0,#0xff -- running minimum. */
void step_slot_init_min(void) { mov_reg8(mcu.r[0], 0xff); mcu.pc = 0x17a3; }
/* 0x17A3 mov.b @r3+0xa220,r2 -- descriptor head. */
void step_slot_load_head(void) { load8(mcu.r[2], ind_addr(3, 0xa220)); mcu.pc = 0x17a7; }
/* 0x17A7 bmi -> 0x17e8 -- chain empty. */
void step_slot_branch_empty(void) { mcu.pc = bmi(0x17e8, 0x17a9); }
/* 0x17A9 cmp r6,b #0x02. */
void step_slot_compare_pass2(void) { cmp8_short_imm(mcu.r[6], 0x02); mcu.pc = 0x17ab; }
/* 0x17AB beq -> 0x17bc -- pass 2 ignores the free/current filters. */
void step_slot_branch_pass2(void) { mcu.pc = beq(0x17bc, 0x17ad); }
/* 0x17AD cmp r6,b #0x01. */
void step_slot_compare_pass1(void) { cmp8_short_imm(mcu.r[6], 0x01); mcu.pc = 0x17af; }
/* 0x17AF beq -> 0x17b8 -- pass 1 skips the free test. */
void step_slot_branch_pass1(void) { mcu.pc = beq(0x17b8, 0x17b1); }
/* 0x17B1 sub.b @r2+0xa288,#0 -- pass 0 needs a free descriptor. */
void step_slot_compare_free(void) { sub8_mem_imm_flags(ind_addr(2, 0xa288), 0x00); mcu.pc = 0x17b6; }
/* 0x17B6 beq -> 0x17e2 -- skip non-free descriptor. */
void step_slot_branch_skip_nonfree(void) { mcu.pc = beq(0x17e2, 0x17b8); }
/* 0x17B8 cmp r2,r4 -- skip the current descriptor. */
void step_slot_compare_current(void) { cmp8_regs(mcu.r[4], mcu.r[2]); mcu.pc = 0x17ba; }
/* 0x17BA beq -> 0x17e2. */
void step_slot_branch_skip_current(void) { mcu.pc = beq(0x17e2, 0x17bc); }
/* 0x17BC mov.b @r2+0xa2dc,r1 -- voice chain tail. */
void step_slot_load_tail(void) { load8(mcu.r[1], ind_addr(2, 0xa2dc)); mcu.pc = 0x17c0; }
/* 0x17C0 cmp.b @r1+0xad0e,r0 -- age vs minimum. */
void step_slot_compare_age(void) { cmp8_reg_mem(mcu.r[0], ind_addr(1, 0xad0e)); mcu.pc = 0x17c4; }
/* 0x17C4 bls -> 0x17ce -- r0 <= age: keep the minimum. */
void step_slot_branch_keep_age(void) { mcu.pc = bls(0x17ce, 0x17c6); }
/* 0x17C6 mov.b @r1+0xad0e,r0 -- new minimum. */
void step_slot_load_age(void) { load8(mcu.r[0], ind_addr(1, 0xad0e)); mcu.pc = 0x17ca; }
/* 0x17CA mov.b r1,(dp,0xa438). */
void step_slot_store_result(void) { store8(dp_addr(0xa438), (uint8_t)mcu.r[1]); mcu.pc = 0x17ce; }
/* 0x17CE mov.b @r1+0xa410,r1 -- next voice in the descriptor chain. */
void step_slot_load_next_voice(void) { load8(mcu.r[1], ind_addr(1, 0xa410)); mcu.pc = 0x17d2; }
/* 0x17D2 bmi -> 0x17e2 -- chain end. */
void step_slot_branch_chain_end(void) { mcu.pc = bmi(0x17e2, 0x17d4); }
/* 0x17D4 cmp.b @r1+0xad0e,r0. */
void step_slot_compare_age2(void) { cmp8_reg_mem(mcu.r[0], ind_addr(1, 0xad0e)); mcu.pc = 0x17d8; }
/* 0x17D8 bls -> 0x17e2. */
void step_slot_branch_keep_age2(void) { mcu.pc = bls(0x17e2, 0x17da); }
/* 0x17DA mov.b @r1+0xad0e,r0. */
void step_slot_load_age2(void) { load8(mcu.r[0], ind_addr(1, 0xad0e)); mcu.pc = 0x17de; }
/* 0x17DE mov.b r1,(dp,0xa438). */
void step_slot_store_result2(void) { store8(dp_addr(0xa438), (uint8_t)mcu.r[1]); mcu.pc = 0x17e2; }
/* 0x17E2 mov.b @r2+0xa250,r2 -- next descriptor. */
void step_slot_next_desc(void) { load8(mcu.r[2], ind_addr(2, 0xa250)); mcu.pc = 0x17e6; }
/* 0x17E6 bpl -> 0x17a9 -- walk the descriptor chain. */
void step_slot_loop_desc(void) { mcu.pc = bpl(0x17a9, 0x17e8); }
/* 0x17E8 mov.b (dp,0xa438),r1 -- result. */
void step_slot_load_result(void) { load8(mcu.r[1], dp_addr(0xa438)); mcu.pc = 0x17ec; }
/* 0x17EC rts. */
void step_slot_return(void) { mcu.pc = MCU_PopStack(); }

/* ---- B11 note_helper 0x17ed..0x1805 (9 PCs) ------------------------------- */
/* ROM 0x17ed..0x1805: a9 13 ad 13 f2 a2 50 85 f2 a2 dc 81 0e 28
 *                     f2 a2 dc 81 2a f8 15 a4 2c 16 19
 * Walks the voice chain of desc r2 (r5 keeps the chain head across the
 * release calls) and drains the shortfall through release A (0x1823). */
/* 0x17ED clr r1. */
void step_nh_clear_r1(void) { clr_reg(mcu.r[1]); mcu.pc = 0x17ef; }
/* 0x17EF clr r5. */
void step_nh_clear_r5(void) { clr_reg(mcu.r[5]); mcu.pc = 0x17f1; }
/* 0x17F1 mov.b @r2+0xa250,r5 -- next descriptor, kept across calls. */
void step_nh_load_next(void) { load8(mcu.r[5], ind_addr(2, 0xa250)); mcu.pc = 0x17f5; }
/* 0x17F5 mov.b @r2+0xa2dc,r1 -- voice chain tail. */
void step_nh_load_tail(void) { load8(mcu.r[1], ind_addr(2, 0xa2dc)); mcu.pc = 0x17f9; }
/* 0x17F9 bsr #0x1823 -- release A. */
void step_nh_call_release(void) { MCU_PushStack(0x17fb); mcu.pc = 0x1823; }
/* 0x17FB mov.b @r2+0xa2dc,r1 -- new chain tail. */
void step_nh_load_tail2(void) { load8(mcu.r[1], ind_addr(2, 0xa2dc)); mcu.pc = 0x17ff; }
/* 0x17FF bpl -> 0x17f9 -- while a voice remains. */
void step_nh_loop_release(void) { mcu.pc = bpl(0x17f9, 0x1801); }
/* 0x1801 tst.b (dp,0xa42c) -- shortfall flag. */
void step_nh_test_shortfall(void) { tst8_mem(dp_addr(0xa42c)); mcu.pc = 0x1805; }
/* 0x1805 rts. */
void step_nh_return(void) { mcu.pc = MCU_PopStack(); }

/* ---- B12 aux_alloc 0x180d..0x1822 (7 PCs) --------------------------------- */
/* ROM 0x180d..0x1822: f2 a2 50 85 f2 a2 dc 81 1e 00 66 f2 a2 dc 81
 *                     2a f7 15 a4 2c 16 19
 * Same shape as B11 but drains through release B (0x187e). */
/* 0x180D mov.b @r2+0xa250,r5 -- next descriptor. */
void step_ax_load_next(void) { load8(mcu.r[5], ind_addr(2, 0xa250)); mcu.pc = 0x1811; }
/* 0x1811 mov.b @r2+0xa2dc,r1 -- voice chain tail. */
void step_ax_load_tail(void) { load8(mcu.r[1], ind_addr(2, 0xa2dc)); mcu.pc = 0x1815; }
/* 0x1815 bsr #0x187e -- release B. */
void step_ax_call_release(void) { MCU_PushStack(0x1818); mcu.pc = 0x187e; }
/* 0x1818 mov.b @r2+0xa2dc,r1 -- new chain tail. */
void step_ax_load_tail2(void) { load8(mcu.r[1], ind_addr(2, 0xa2dc)); mcu.pc = 0x181c; }
/* 0x181C bpl -> 0x1815. */
void step_ax_loop_release(void) { mcu.pc = bpl(0x1815, 0x181e); }
/* 0x181E tst.b (dp,0xa42c). */
void step_ax_test_shortfall(void) { tst8_mem(dp_addr(0xa42c)); mcu.pc = 0x1822; }
/* 0x1822 rts. */
void step_ax_return(void) { mcu.pc = MCU_PopStack(); }

/* ---- B15 desc_setup 0x18ce..0x194b (38 PCs) ------------------------------- */
/* ROM 0x18ce..0x194b:
 *   ab 13 15 a4 a4 83 aa 13 15 a4 2e 82 f2 a2 50 80 15 a4 2e 90 a8 13
 *   f3 a2 30 80 2a 06 f3 a2 20 92 20 04 f0 a2 50 92 f3 a2 30 92
 *   f2 a2 6c 90 f2 a2 50 06 ff 15 a4 a5 80 f2 a2 f8 90 15 a4 a6 80
 *   f2 a3 14 90 15 a1 e0 80 f2 a3 30 90 15 a4 a9 92 1e 02 70 ac 13
 *   15 a4 a8 84 ac 80 f0 a4 aa 06 ff a4 0c 0e 7c ac 80 f0 a4 aa 91
 *   15 a4 a6 86 f1 d1 18 96 0e 0b f3 a2 10 08 0c 00 01 34 2a e4 19
 * Builds one descriptor: part a4a4 = r3, unlinks from the a42e desc free
 * list, links into the part's a220/a230 chain, copies a4a5/a4a6/a1e0 into
 * the desc, then fills a4a8 descriptors via pool_pop (0x19ad) + link
 * (0x194c), incrementing the part voice count (a210) and decrementing the
 * shortfall counter r4 (word at 0x1945, byte ADDQ at 0x192d). */
/* 0x18CE clr r3. */
void step_ds_clear_r3(void) { clr_reg(mcu.r[3]); mcu.pc = 0x18d0; }
/* 0x18D0 mov.b (dp,0xa4a4),r3 -- part index. */
void step_ds_load_part(void) { load8(mcu.r[3], dp_addr(0xa4a4)); mcu.pc = 0x18d4; }
/* 0x18D4 clr r2. */
void step_ds_clear_r2(void) { clr_reg(mcu.r[2]); mcu.pc = 0x18d6; }
/* 0x18D6 mov.b (dp,0xa42e),r2 -- descriptor free head. */
void step_ds_load_free_head(void) { load8(mcu.r[2], dp_addr(0xa42e)); mcu.pc = 0x18da; }
/* 0x18DA mov.b @r2+0xa250,r0 -- descriptor next. */
void step_ds_load_desc_next(void) { load8(mcu.r[0], ind_addr(2, 0xa250)); mcu.pc = 0x18de; }
/* 0x18DE mov.b r0,(dp,0xa42e) -- new free head. */
void step_ds_store_free_head(void) { store8(dp_addr(0xa42e), (uint8_t)mcu.r[0]); mcu.pc = 0x18e2; }
/* 0x18E2 clr r0. */
void step_ds_clear_r0(void) { clr_reg(mcu.r[0]); mcu.pc = 0x18e4; }
/* 0x18E4 mov.b @r3+0xa230,r0 -- part descriptor tail. */
void step_ds_load_part_tail(void) { load8(mcu.r[0], ind_addr(3, 0xa230)); mcu.pc = 0x18e8; }
/* 0x18E8 bpl -> 0x18f0 -- non-empty chain. */
void step_ds_branch_chain_empty(void) { mcu.pc = bpl(0x18f0, 0x18ea); }
/* 0x18EA mov.b r2,@r3+0xa220 -- part head = descriptor. */
void step_ds_link_head(void) { store8(ind_addr(3, 0xa220), (uint8_t)mcu.r[2]); mcu.pc = 0x18ee; }
/* 0x18EE bra -> 0x18f4. */
void step_ds_branch_join_head(void) { mcu.pc = 0x18f4; }
/* 0x18F0 mov.b r2,@r0+0xa250 -- old tail.next = descriptor. */
void step_ds_link_tail(void) { store8(ind_addr(0, 0xa250), (uint8_t)mcu.r[2]); mcu.pc = 0x18f4; }
/* 0x18F4 mov.b r2,@r3+0xa230 -- part tail = descriptor. */
void step_ds_store_part_tail(void) { store8(ind_addr(3, 0xa230), (uint8_t)mcu.r[2]); mcu.pc = 0x18f8; }
/* 0x18F8 mov.b r0,@r2+0xa26c -- descriptor.prev = old tail. */
void step_ds_store_desc_prev(void) { store8(ind_addr(2, 0xa26c), (uint8_t)mcu.r[0]); mcu.pc = 0x18fc; }
/* 0x18FC movg #0xff,@r2+0xa250 -- descriptor.next = none. */
void step_ds_terminate_desc(void) { mov_imm8(ind_addr(2, 0xa250), 0xff); mcu.pc = 0x1901; }
/* 0x1901 mov.b (dp,0xa4a5),r0. */
void step_ds_load_a4a5(void) { load8(mcu.r[0], dp_addr(0xa4a5)); mcu.pc = 0x1905; }
/* 0x1905 mov.b r0,@r2+0xa2f8. */
void step_ds_store_a2f8(void) { store8(ind_addr(2, 0xa2f8), (uint8_t)mcu.r[0]); mcu.pc = 0x1909; }
/* 0x1909 mov.b (dp,0xa4a6),r0. */
void step_ds_load_a4a6(void) { load8(mcu.r[0], dp_addr(0xa4a6)); mcu.pc = 0x190d; }
/* 0x190D mov.b r0,@r2+0xa314 -- descriptor age. */
void step_ds_store_age(void) { store8(ind_addr(2, 0xa314), (uint8_t)mcu.r[0]); mcu.pc = 0x1911; }
/* 0x1911 mov.b (dp,0xa1e0),r0. */
void step_ds_load_a1e0(void) { load8(mcu.r[0], dp_addr(0xa1e0)); mcu.pc = 0x1915; }
/* 0x1915 mov.b r0,@r2+0xa330. */
void step_ds_store_a330(void) { store8(ind_addr(2, 0xa330), (uint8_t)mcu.r[0]); mcu.pc = 0x1919; }
/* 0x1919 mov.b r2,(dp,0xa4a9) -- setup descriptor register. */
void step_ds_store_setup_desc(void) { store8(dp_addr(0xa4a9), (uint8_t)mcu.r[2]); mcu.pc = 0x191d; }
/* 0x191D bsr #0x1b90 -- H3 descriptor state. */
void step_ds_call_h3(void) { MCU_PushStack(0x1920); mcu.pc = 0x1b90; }
/* 0x1920 clr r4. */
void step_ds_clear_r4(void) { clr_reg(mcu.r[4]); mcu.pc = 0x1922; }
/* 0x1922 mov.b (dp,0xa4a8),r4 -- voices to allocate. */
void step_ds_load_alloc_count(void) { load8(mcu.r[4], dp_addr(0xa4a8)); mcu.pc = 0x1926; }
/* 0x1926 mov.w r4,r0. */
void step_ds_copy_r4(void) { mov_reg16(mcu.r[0], mcu.r[4]); mcu.pc = 0x1928; }
/* 0x1928 movg #0xff,@r0+0xa4aa -- clear the per-part voice list. */
void step_ds_clear_voice_list(void) { mov_imm8(ind_addr(0, 0xa4aa), 0xff); mcu.pc = 0x192d; }
/* 0x192D addq #-1,r4 (byte). */
void step_ds_dec_count(void) { addq8_reg(mcu.r[4], -1); mcu.pc = 0x192f; }
/* 0x192F bsr #0x19ad -- pool_pop: free voice -> r1. */
void step_ds_call_pool_pop(void) { MCU_PushStack(0x1931); mcu.pc = 0x19ad; }
/* 0x1931 mov.w r4,r0. */
void step_ds_copy_r4_b(void) { mov_reg16(mcu.r[0], mcu.r[4]); mcu.pc = 0x1933; }
/* 0x1933 mov.b r1,@r0+0xa4aa -- voice list append. */
void step_ds_append_voice(void) { store8(ind_addr(0, 0xa4aa), (uint8_t)mcu.r[1]); mcu.pc = 0x1937; }
/* 0x1937 mov.b (dp,0xa4a6),r6 -- descriptor age. */
void step_ds_load_age(void) { load8(mcu.r[6], dp_addr(0xa4a6)); mcu.pc = 0x193b; }
/* 0x193B mov.b r6,@r1+0xd118 -- voice age. */
void step_ds_store_voice_age(void) { store8(ind_addr(1, 0xd118), (uint8_t)mcu.r[6]); mcu.pc = 0x193f; }
/* 0x193F bsr #0x194c -- link voice into the descriptor/part chains. */
void step_ds_call_link(void) { MCU_PushStack(0x1941); mcu.pc = 0x194c; }
/* 0x1941 addq #1,@r3+0xa210 -- part voice count++. */
void step_ds_inc_part_count(void) { addq8_mem(ind_addr(3, 0xa210), 1); mcu.pc = 0x1945; }
/* 0x1945 sub #0x0001,r4 -- shortfall--. */
void step_ds_dec_shortfall(void) { sub16_reg_imm(mcu.r[4], 0x0001); mcu.pc = 0x1949; }
/* 0x1949 bpl -> 0x192f -- more voices to allocate. */
void step_ds_loop_alloc(void) { mcu.pc = bpl(0x192f, 0x194b); }
/* 0x194B rts. */
void step_ds_return(void) { mcu.pc = MCU_PopStack(); }

/* ---- B19 cleanup_seq 0x1a4d..0x1a6b (10 PCs) ------------------------------ */
/* ROM 0x1a4d..0x1a6b: 15 a1 d1 f7 27 18 a9 13 15 a4 aa 81 2b 05
 *                     f1 ce 3f 06 ff 15 a4 ab 81 2b 05 f1 ce 3f 06 ff 19
 * If (dp,0xa1d1) bit7 is set, clears both per-part voice list heads
 * (a4aa/a4ab) by writing 0xff into each voice's ce3f byte. */
/* 0x1A4D btsti (dp,0xa1d1),#7. */
void step_cq_test_flag(void) { btsti(dp_addr(0xa1d1), 7); mcu.pc = 0x1a51; }
/* 0x1A51 beq -> 0x1a6b -- nothing to clean. */
void step_cq_branch_skip(void) { mcu.pc = beq(0x1a6b, 0x1a53); }
/* 0x1A53 clr r1. */
void step_cq_clear_r1(void) { clr_reg(mcu.r[1]); mcu.pc = 0x1a55; }
/* 0x1A55 mov.b (dp,0xa4aa),r1 -- list A head. */
void step_cq_load_list_a(void) { load8(mcu.r[1], dp_addr(0xa4aa)); mcu.pc = 0x1a59; }
/* 0x1A59 bmi -> 0x1a60 -- list empty. */
void step_cq_branch_a_empty(void) { mcu.pc = bmi(0x1a60, 0x1a5b); }
/* 0x1A5B movg #0xff,@r1+0xce3f. */
void step_cq_clear_a(void) { mov_imm8(ind_addr(1, 0xce3f), 0xff); mcu.pc = 0x1a60; }
/* 0x1A60 mov.b (dp,0xa4ab),r1 -- list B head. */
void step_cq_load_list_b(void) { load8(mcu.r[1], dp_addr(0xa4ab)); mcu.pc = 0x1a64; }
/* 0x1A64 bmi -> 0x1a6b -- list empty. */
void step_cq_branch_b_empty(void) { mcu.pc = bmi(0x1a6b, 0x1a66); }
/* 0x1A66 movg #0xff,@r1+0xce3f. */
void step_cq_clear_b(void) { mov_imm8(ind_addr(1, 0xce3f), 0xff); mcu.pc = 0x1a6b; }
/* 0x1A6B rts. */
void step_cq_return(void) { mcu.pc = MCU_PopStack(); }

/* ---- C3 pcm_stop 0x5238..0x52c8 (48 PCs) ---------------------------------- */
/* ROM 0x5238..0x52c8:
 *   f1 ad 0e 13 a9 82 aa 22 fa 64 d6 82 ea 00 04 12 26 09 71 3e 65 32
 *   5e 00 0e 20 07 71 3e 65 34 5e 00 10 6d 3a 27 17 ad 25 a5 10
 *   f1 ad 0e 95 ea 00 96 ea 02 96 ea 04 96 0c f8 ff 58 20 87
 *   f1 ad 0e 13 5e 00 16 ea 00 96 ea 02 96 ea 04 96 f1 d0 a8 04 ff
 *   26 0d f1 d0 c4 04 ff 27 20 [f1 d0 c4 83 20 04] [f1 d0 a8 83]
 *   a3 12 f1 d0 a8 06 ff f1 d0 c4 06 ff f3 d0 a8 06 ff f3 d0 c4 06 ff
 *   71 3e 0d 18 07 00 b5 1d a1 f6 91 0c f8 ff 58 50 01 51 01 08 12
 *   30 ff 31
 * The two bracketed groups 0x5293/0x5297 and 0x5299 are the never-executed
 * arms of 0x528a BNE (C: ROM bytes). Both BRA exits (0x5273, 0x52c8) target
 * 0x51fc, the C2 dispatcher, another slice: plain pc writes, no stack pop.
 * 0x52c6 trapa #0x12: MCU_TRAPA uses opcode & 0x0f, i.e. vector 2
 * (src/mcu_opcodes.cpp:185-196). */
/* 0x5238 clr @r1+0xad0e -- release the old channel. */
void step_ps_clear_channel(void) { clr8_mem(ind_addr(1, 0xad0e)); mcu.pc = 0x523c; }
/* 0x523C mov.w r1,r2. */
void step_ps_copy_slot(void) { mov_reg16(mcu.r[2], mcu.r[1]); mcu.pc = 0x523e; }
/* 0x523E add r2,r2 -- slot * 2. */
void step_ps_double_slot(void) { add16_self(mcu.r[2]); mcu.pc = 0x5240; }
/* 0x5240 mov.w @r2+0x64d6,r2 -- AoS record pointer. */
void step_ps_load_record(void) { load16(mcu.r[2], ind_addr(2, 0x64d6)); mcu.pc = 0x5244; }
/* 0x5244 sub.w @r2,#0x0012 -- record state == 0x12? */
void step_ps_compare_state(void) { sub16_mem_imm_flags(ind_addr(2, 0), 0x0012); mcu.pc = 0x5248; }
/* 0x5248 bne -> 0x5253 -- other state. */
void step_ps_branch_state(void) { mcu.pc = bne(0x5253, 0x524a); }
/* 0x524A movs r1,(br,0x3e) -- write the select byte. */
void step_ps_store_key_a(void) { movs8_br(1, 0x3e); mcu.pc = 0x524c; }
/* 0x524C movl r5,(br,0x32) -- PCM readback byte. */
void step_ps_load_value_a(void) { movl8_br(5, 0x32); mcu.pc = 0x524e; }
/* 0x524E movi r6,#0x000e. */
void step_ps_set_code_a(void) { mov_reg16(mcu.r[6], 0x000e); mcu.pc = 0x5251; }
/* 0x5251 bra -> 0x525a. */
void step_ps_branch_apply_a(void) { mcu.pc = 0x525a; }
/* 0x5253 movs r1,(br,0x3e). */
void step_ps_store_key_b(void) { movs8_br(1, 0x3e); mcu.pc = 0x5255; }
/* 0x5255 movl r5,(br,0x34). */
void step_ps_load_value_b(void) { movl8_br(5, 0x34); mcu.pc = 0x5257; }
/* 0x5257 movi r6,#0x0010. */
void step_ps_set_code_b(void) { mov_reg16(mcu.r[6], 0x0010); mcu.pc = 0x525a; }
/* 0x525A movlw r5,(br,0x3a) -- word readback. */
void step_ps_load_readback(void) { movlw16_br(5, 0x3a); mcu.pc = 0x525c; }
/* 0x525C beq -> 0x5275 -- readback zero. */
void step_ps_branch_readback(void) { mcu.pc = beq(0x5275, 0x525e); }
/* 0x525E add r5,r5 -- byte offset * 2. */
void step_ps_double_readback(void) { add16_self(mcu.r[5]); mcu.pc = 0x5260; }
/* 0x5260 swap r5 -- high byte becomes the channel index. */
void step_ps_swap_readback(void) { swap_reg(mcu.r[5]); mcu.pc = 0x5262; }
/* 0x5262 mov.b r5,@r1+0xad0e -- channel = readback. */
void step_ps_store_channel(void) { store8(ind_addr(1, 0xad0e), (uint8_t)mcu.r[5]); mcu.pc = 0x5266; }
/* 0x5266 mov.w r6,@r2 -- state. */
void step_ps_store_state0(void) { store16(ind_addr(2, 0), mcu.r[6]); mcu.pc = 0x5269; }
/* 0x5269 mov.w r6,@r2+2. */
void step_ps_store_state2(void) { store16(ind_addr(2, 2), mcu.r[6]); mcu.pc = 0x526c; }
/* 0x526C mov.w r6,@r2+4. */
void step_ps_store_state4(void) { store16(ind_addr(2, 4), mcu.r[6]); mcu.pc = 0x526f; }
/* 0x526F bclr_andc #0xf8ff,r0 -- IML=0. */
void step_ps_lower_iml(void) { bclr_andc_iml0(); mcu.pc = 0x5273; }
/* 0x5273 bra -> 0x51fc -- dispatcher (other slice). */
void step_ps_branch_dispatcher(void) { mcu.pc = 0x51fc; }
/* 0x5275 clr @r1+0xad0e. */
void step_ps_clear_channel_b(void) { clr8_mem(ind_addr(1, 0xad0e)); mcu.pc = 0x5279; }
/* 0x5279 movi r6,#0x0016 -- idle state. */
void step_ps_set_idle(void) { mov_reg16(mcu.r[6], 0x0016); mcu.pc = 0x527c; }
/* 0x527C mov.w r6,@r2. */
void step_ps_store_idle0(void) { store16(ind_addr(2, 0), mcu.r[6]); mcu.pc = 0x527f; }
/* 0x527F mov.w r6,@r2+2. */
void step_ps_store_idle2(void) { store16(ind_addr(2, 2), mcu.r[6]); mcu.pc = 0x5282; }
/* 0x5282 mov.w r6,@r2+4. */
void step_ps_store_idle4(void) { store16(ind_addr(2, 4), mcu.r[6]); mcu.pc = 0x5285; }
/* 0x5285 sub.b @r1+0xd0a8,#0xff -- start-prev valid? */
void step_ps_compare_prev(void) { sub8_mem_imm_flags(ind_addr(1, 0xd0a8), 0xff); mcu.pc = 0x528a; }
/* 0x528A bne -> 0x5299 -- neighbour exists. */
void step_ps_branch_prev(void) { mcu.pc = bne(0x5299, 0x528c); }
/* 0x528C sub.b @r1+0xd0c4,#0xff -- start-next valid? */
void step_ps_compare_next(void) { sub8_mem_imm_flags(ind_addr(1, 0xd0c4), 0xff); mcu.pc = 0x5291; }
/* 0x5291 beq -> 0x52b3 -- neither neighbour exists. */
void step_ps_branch_next(void) { mcu.pc = beq(0x52b3, 0x5293); }
/* 0x5293 mov.b @r1+0xd0c4,r3 (unexecuted arm). */
void step_ps_load_next_arm(void) { load8(mcu.r[3], ind_addr(1, 0xd0c4)); mcu.pc = 0x5297; }
/* 0x5297 bra -> 0x529d (unexecuted arm). */
void step_ps_branch_unlink(void) { mcu.pc = 0x529d; }
/* 0x5299 mov.b @r1+0xd0a8,r3 -- the existing neighbour. */
void step_ps_load_prev(void) { load8(mcu.r[3], ind_addr(1, 0xd0a8)); mcu.pc = 0x529d; }
/* 0x529D extu r3 -- zero-extend the neighbour index. */
void step_ps_extend_neighbour(void) { extu(mcu.r[3]); mcu.pc = 0x529f; }
/* 0x529F movg #0xff,@r1+0xd0a8 -- clear own prev link. */
void step_ps_clear_own_prev(void) { mov_imm8(ind_addr(1, 0xd0a8), 0xff); mcu.pc = 0x52a4; }
/* 0x52A4 movg #0xff,@r1+0xd0c4 -- clear own next link. */
void step_ps_clear_own_next(void) { mov_imm8(ind_addr(1, 0xd0c4), 0xff); mcu.pc = 0x52a9; }
/* 0x52A9 movg #0xff,@r3+0xd0a8 -- neighbour prev = none. */
void step_ps_clear_nbr_prev(void) { mov_imm8(ind_addr(3, 0xd0a8), 0xff); mcu.pc = 0x52ae; }
/* 0x52AE movg #0xff,@r3+0xd0c4 -- neighbour next = none. */
void step_ps_clear_nbr_next(void) { mov_imm8(ind_addr(3, 0xd0c4), 0xff); mcu.pc = 0x52b3; }
/* 0x52B3 movs r1,(br,0x3e). */
void step_ps_store_key_c(void) { movs8_br(1, 0x3e); mcu.pc = 0x52b5; }
/* 0x52B5 movg #0x00b5,(br,0x18) -- PCM main volume. */
void step_ps_store_volume(void) { mov_imm16_br(0x18, 0x00b5); mcu.pc = 0x52ba; }
/* 0x52BA mov.w r1,(dp,0xa1f6) -- PCM select shadow. */
void step_ps_store_shadow(void) { store16(dp_addr(0xa1f6), mcu.r[1]); mcu.pc = 0x52be; }
/* 0x52BE bclr_andc #0xf8ff,r0 -- IML=0. */
void step_ps_lower_iml_b(void) { bclr_andc_iml0(); mcu.pc = 0x52c2; }
/* 0x52C2 move r0,#0x01. */
void step_ps_set_r0(void) { mov_reg8(mcu.r[0], 0x01); mcu.pc = 0x52c4; }
/* 0x52C4 move r1,#0x01. */
void step_ps_set_r1(void) { mov_reg8(mcu.r[1], 0x01); mcu.pc = 0x52c6; }
/* 0x52C6 trapa -- vector 2. */
void step_ps_trapa(void) { MCU_Interrupt_TRAPA(2); mcu.pc = 0x52c8; }
/* 0x52C8 bra -> 0x51fc -- dispatcher (other slice). */
void step_ps_branch_dispatcher2(void) { mcu.pc = 0x51fc; }

/* ---- PC tables: one host step per instruction ----------------------------- */

struct NotePcEnt
{
    uint16_t pc;
    mk2cpp_fn fn;
};

const NotePcEnt kSteal[] = {
    { 0x16f4, &step_steal_load_head },       { 0x16f8, &step_steal_branch_empty },
    { 0x16fa, &step_steal_compare_free },    { 0x16ff, &step_steal_branch_free },
    { 0x1701, &step_steal_call_helper },     { 0x1704, &step_steal_branch_shortfall },
    { 0x1706, &step_steal_load_active },     { 0x170a, &step_steal_sub_limit },
    { 0x170e, &step_steal_branch_limit },    { 0x1710, &step_steal_copy_voice },
    { 0x1712, &step_steal_branch_sign },     { 0x1714, &step_steal_next_desc },
    { 0x1718, &step_steal_loop_chain },      { 0x171a, &step_steal_restart_head },
    { 0x171e, &step_steal_branch_restart_empty },
    { 0x1720, &step_steal_call_helper2 },    { 0x1723, &step_steal_branch_shortfall2 },
    { 0x1725, &step_steal_load_active2 },    { 0x1729, &step_steal_sub_limit2 },
    { 0x172d, &step_steal_branch_limit2 },   { 0x172f, &step_steal_copy_voice2 },
    { 0x1731, &step_steal_loop_retry },      { 0x1733, &step_steal_branch_done },
    { 0x1735, &step_steal_next_desc2 },      { 0x1739, &step_steal_loop_retry2 },
    { 0x173b, &step_steal_set_one },         { 0x173d, &step_steal_return },
};

const NotePcEnt kKeyOn[] = {
    { 0x173e, &step_key_on_load_current },   { 0x1742, &step_key_on_pass0 },
    { 0x1744, &step_key_on_call_slot0 },     { 0x1746, &step_key_on_branch_no_slot0 },
    { 0x1748, &step_key_on_load_selected0 }, { 0x174c, &step_key_on_call_helper0 },
    { 0x174f, &step_key_on_branch_shortfall0 },
    { 0x1751, &step_key_on_load_active0 },   { 0x1755, &step_key_on_sub_limit0 },
    { 0x1759, &step_key_on_branch_above0 },  { 0x175b, &step_key_on_branch_success0 },
    { 0x175d, &step_key_on_pass1 },          { 0x175f, &step_key_on_call_slot1 },
    { 0x1762, &step_key_on_branch_no_slot1 },{ 0x1764, &step_key_on_load_selected1 },
    { 0x1768, &step_key_on_call_helper1 },   { 0x176b, &step_key_on_branch_shortfall1 },
    { 0x176d, &step_key_on_load_active1 },   { 0x1771, &step_key_on_sub_limit1 },
    { 0x1775, &step_key_on_branch_above1 },  { 0x1777, &step_key_on_branch_success1 },
    { 0x1779, &step_key_on_test_current },   { 0x177d, &step_key_on_branch_no_current },
    { 0x177f, &step_key_on_pass2 },          { 0x1781, &step_key_on_call_slot2 },
    { 0x1784, &step_key_on_branch_no_slot2 },{ 0x1786, &step_key_on_load_selected2 },
    { 0x178a, &step_key_on_call_helper2 },   { 0x178d, &step_key_on_branch_shortfall2 },
    { 0x178f, &step_key_on_load_active2 },   { 0x1793, &step_key_on_sub_limit2 },
    { 0x1797, &step_key_on_branch_above2 },  { 0x1799, &step_key_on_set_one },
    { 0x179b, &step_key_on_return },
};

const NotePcEnt kSlotSelect[] = {
    { 0x179c, &step_slot_clear_result },     { 0x17a1, &step_slot_init_min },
    { 0x17a3, &step_slot_load_head },        { 0x17a7, &step_slot_branch_empty },
    { 0x17a9, &step_slot_compare_pass2 },    { 0x17ab, &step_slot_branch_pass2 },
    { 0x17ad, &step_slot_compare_pass1 },    { 0x17af, &step_slot_branch_pass1 },
    { 0x17b1, &step_slot_compare_free },     { 0x17b6, &step_slot_branch_skip_nonfree },
    { 0x17b8, &step_slot_compare_current },  { 0x17ba, &step_slot_branch_skip_current },
    { 0x17bc, &step_slot_load_tail },        { 0x17c0, &step_slot_compare_age },
    { 0x17c4, &step_slot_branch_keep_age },  { 0x17c6, &step_slot_load_age },
    { 0x17ca, &step_slot_store_result },     { 0x17ce, &step_slot_load_next_voice },
    { 0x17d2, &step_slot_branch_chain_end }, { 0x17d4, &step_slot_compare_age2 },
    { 0x17d8, &step_slot_branch_keep_age2 }, { 0x17da, &step_slot_load_age2 },
    { 0x17de, &step_slot_store_result2 },    { 0x17e2, &step_slot_next_desc },
    { 0x17e6, &step_slot_loop_desc },        { 0x17e8, &step_slot_load_result },
    { 0x17ec, &step_slot_return },
};

const NotePcEnt kNoteHelper[] = {
    { 0x17ed, &step_nh_clear_r1 },      { 0x17ef, &step_nh_clear_r5 },
    { 0x17f1, &step_nh_load_next },     { 0x17f5, &step_nh_load_tail },
    { 0x17f9, &step_nh_call_release },  { 0x17fb, &step_nh_load_tail2 },
    { 0x17ff, &step_nh_loop_release },  { 0x1801, &step_nh_test_shortfall },
    { 0x1805, &step_nh_return },
};

const NotePcEnt kAuxAlloc[] = {
    { 0x180d, &step_ax_load_next },     { 0x1811, &step_ax_load_tail },
    { 0x1815, &step_ax_call_release },  { 0x1818, &step_ax_load_tail2 },
    { 0x181c, &step_ax_loop_release },  { 0x181e, &step_ax_test_shortfall },
    { 0x1822, &step_ax_return },
};

const NotePcEnt kDescSetup[] = {
    { 0x18ce, &step_ds_clear_r3 },         { 0x18d0, &step_ds_load_part },
    { 0x18d4, &step_ds_clear_r2 },         { 0x18d6, &step_ds_load_free_head },
    { 0x18da, &step_ds_load_desc_next },   { 0x18de, &step_ds_store_free_head },
    { 0x18e2, &step_ds_clear_r0 },         { 0x18e4, &step_ds_load_part_tail },
    { 0x18e8, &step_ds_branch_chain_empty },{ 0x18ea, &step_ds_link_head },
    { 0x18ee, &step_ds_branch_join_head }, { 0x18f0, &step_ds_link_tail },
    { 0x18f4, &step_ds_store_part_tail },  { 0x18f8, &step_ds_store_desc_prev },
    { 0x18fc, &step_ds_terminate_desc },   { 0x1901, &step_ds_load_a4a5 },
    { 0x1905, &step_ds_store_a2f8 },       { 0x1909, &step_ds_load_a4a6 },
    { 0x190d, &step_ds_store_age },        { 0x1911, &step_ds_load_a1e0 },
    { 0x1915, &step_ds_store_a330 },       { 0x1919, &step_ds_store_setup_desc },
    { 0x191d, &step_ds_call_h3 },          { 0x1920, &step_ds_clear_r4 },
    { 0x1922, &step_ds_load_alloc_count }, { 0x1926, &step_ds_copy_r4 },
    { 0x1928, &step_ds_clear_voice_list }, { 0x192d, &step_ds_dec_count },
    { 0x192f, &step_ds_call_pool_pop },    { 0x1931, &step_ds_copy_r4_b },
    { 0x1933, &step_ds_append_voice },     { 0x1937, &step_ds_load_age },
    { 0x193b, &step_ds_store_voice_age },  { 0x193f, &step_ds_call_link },
    { 0x1941, &step_ds_inc_part_count },   { 0x1945, &step_ds_dec_shortfall },
    { 0x1949, &step_ds_loop_alloc },       { 0x194b, &step_ds_return },
};

const NotePcEnt kCleanupSeq[] = {
    { 0x1a4d, &step_cq_test_flag },      { 0x1a51, &step_cq_branch_skip },
    { 0x1a53, &step_cq_clear_r1 },       { 0x1a55, &step_cq_load_list_a },
    { 0x1a59, &step_cq_branch_a_empty }, { 0x1a5b, &step_cq_clear_a },
    { 0x1a60, &step_cq_load_list_b },    { 0x1a64, &step_cq_branch_b_empty },
    { 0x1a66, &step_cq_clear_b },        { 0x1a6b, &step_cq_return },
};

const NotePcEnt kPcmStop[] = {
    { 0x5238, &step_ps_clear_channel },   { 0x523c, &step_ps_copy_slot },
    { 0x523e, &step_ps_double_slot },      { 0x5240, &step_ps_load_record },
    { 0x5244, &step_ps_compare_state },    { 0x5248, &step_ps_branch_state },
    { 0x524a, &step_ps_store_key_a },      { 0x524c, &step_ps_load_value_a },
    { 0x524e, &step_ps_set_code_a },       { 0x5251, &step_ps_branch_apply_a },
    { 0x5253, &step_ps_store_key_b },      { 0x5255, &step_ps_load_value_b },
    { 0x5257, &step_ps_set_code_b },       { 0x525a, &step_ps_load_readback },
    { 0x525c, &step_ps_branch_readback },  { 0x525e, &step_ps_double_readback },
    { 0x5260, &step_ps_swap_readback },    { 0x5262, &step_ps_store_channel },
    { 0x5266, &step_ps_store_state0 },     { 0x5269, &step_ps_store_state2 },
    { 0x526c, &step_ps_store_state4 },     { 0x526f, &step_ps_lower_iml },
    { 0x5273, &step_ps_branch_dispatcher },{ 0x5275, &step_ps_clear_channel_b },
    { 0x5279, &step_ps_set_idle },         { 0x527c, &step_ps_store_idle0 },
    { 0x527f, &step_ps_store_idle2 },      { 0x5282, &step_ps_store_idle4 },
    { 0x5285, &step_ps_compare_prev },     { 0x528a, &step_ps_branch_prev },
    { 0x528c, &step_ps_compare_next },     { 0x5291, &step_ps_branch_next },
    { 0x5293, &step_ps_load_next_arm },    { 0x5297, &step_ps_branch_unlink },
    { 0x5299, &step_ps_load_prev },        { 0x529d, &step_ps_extend_neighbour },
    { 0x529f, &step_ps_clear_own_prev },   { 0x52a4, &step_ps_clear_own_next },
    { 0x52a9, &step_ps_clear_nbr_prev },   { 0x52ae, &step_ps_clear_nbr_next },
    { 0x52b3, &step_ps_store_key_c },      { 0x52b5, &step_ps_store_volume },
    { 0x52ba, &step_ps_store_shadow },     { 0x52be, &step_ps_lower_iml_b },
    { 0x52c2, &step_ps_set_r0 },           { 0x52c4, &step_ps_set_r1 },
    { 0x52c6, &step_ps_trapa },            { 0x52c8, &step_ps_branch_dispatcher2 },
};

} /* anonymous namespace */
} /* namespace mk2c */

void MK2CPP_NoteChainTailFillTables(void)
{
#if MK2CPP_HAND_NOTE_CHAIN_TAIL
    const mk2c::NotePcEnt *named[] = {
        mk2c::kSteal, mk2c::kKeyOn, mk2c::kSlotSelect,
        mk2c::kNoteHelper, mk2c::kAuxAlloc, mk2c::kDescSetup,
        mk2c::kCleanupSeq, mk2c::kPcmStop,
    };
    const uint32_t named_sizes[] = {
        (uint32_t)(sizeof(mk2c::kSteal) / sizeof(mk2c::kSteal[0])),
        (uint32_t)(sizeof(mk2c::kKeyOn) / sizeof(mk2c::kKeyOn[0])),
        (uint32_t)(sizeof(mk2c::kSlotSelect) / sizeof(mk2c::kSlotSelect[0])),
        (uint32_t)(sizeof(mk2c::kNoteHelper) / sizeof(mk2c::kNoteHelper[0])),
        (uint32_t)(sizeof(mk2c::kAuxAlloc) / sizeof(mk2c::kAuxAlloc[0])),
        (uint32_t)(sizeof(mk2c::kDescSetup) / sizeof(mk2c::kDescSetup[0])),
        (uint32_t)(sizeof(mk2c::kCleanupSeq) / sizeof(mk2c::kCleanupSeq[0])),
        (uint32_t)(sizeof(mk2c::kPcmStop) / sizeof(mk2c::kPcmStop[0])),
    };
    for (uint32_t t = 0; t < (uint32_t)(sizeof(named) / sizeof(named[0])); t++)
        for (uint32_t i = 0; i < named_sizes[t]; i++)
            MK2CPP_HandRegister(named[t][i].pc, named[t][i].fn);
#endif
}

/* Self-registration (hand_registry.h): pcm_enable.cpp's MK2CPP_HandFillTables
 * calls hand_fill_modules() after the built-in modules, so this file never
 * edits a shared aggregator. Static ctor + function-local registry storage
 * keeps initialization order safe. */
namespace
{
struct note_chain_tail_autoreg
{
    note_chain_tail_autoreg()
    {
        mk2c::hand_register_module(&MK2CPP_NoteChainTailFillTables);
    }
};
note_chain_tail_autoreg g_note_chain_tail_autoreg;
} /* anonymous namespace */
