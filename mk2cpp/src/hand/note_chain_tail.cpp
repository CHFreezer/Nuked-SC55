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

/* Safety net: a PC missing from the tables below is executed by the stock
 * interpreter for that one instruction (should never happen). */
void stock_instruction(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
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
uint32_t step_steal(void)
{
    switch (mcu.pc)
    {
    case 0x16f4: /* MOVG2.B @r3+0xa220 r2 (part descriptor chain head) */
        load8(mcu.r[2], ind_addr(3, 0xa220));
        mcu.pc = 0x16f8;
        return 1;
    case 0x16f8: /* BMI 0x173b: chain empty */
        mcu.pc = bmi(0x173b, 0x16fa);
        return 1;
    case 0x16fa: /* SUB.B @r2+0xa288 #0x00 (desc free? flags only) */
        sub8_mem_imm_flags(ind_addr(2, 0xa288), 0x00);
        mcu.pc = 0x16ff;
        return 1;
    case 0x16ff: /* BEQ 0x1714: free descriptor */
        mcu.pc = beq(0x1714, 0x1701);
        return 1;
    case 0x1701: /* bsr16 0x17ed (note_helper, releases desc voices) */
        MCU_PushStack(0x1704);
        mcu.pc = 0x17ed;
        return 1;
    case 0x1704: /* BLE 0x173d: shortfall >= 0 */
        mcu.pc = ble(0x173d, 0x1706);
        return 1;
    case 0x1706: /* MOVG2.B @r3+0xa210 r0 (active voices, unexecuted arm) */
        load8(mcu.r[0], ind_addr(3, 0xa210));
        mcu.pc = 0x170a;
        return 1;
    case 0x170a: /* SUB.B @r3+0x8018 r0 (vs voice limit, unexecuted arm) */
        sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018));
        mcu.pc = 0x170e;
        return 1;
    case 0x170e: /* BLE 0x171a */
        mcu.pc = ble(0x171a, 0x1710);
        return 1;
    case 0x1710: /* MOVG2.B r5 r2 (unexecuted arm) */
        mov_reg8(mcu.r[2], mcu.r[5]);
        mcu.pc = 0x1712;
        return 1;
    case 0x1712: /* BMI 0x171a */
        mcu.pc = bmi(0x171a, 0x1714);
        return 1;
    case 0x1714: /* MOVG2.B @r2+0xa250 r2 (next descriptor) */
        load8(mcu.r[2], ind_addr(2, 0xa250));
        mcu.pc = 0x1718;
        return 1;
    case 0x1718: /* BPL 0x16fa: walk the chain */
        mcu.pc = bpl(0x16fa, 0x171a);
        return 1;
    case 0x171a: /* MOVG2.B @r3+0xa220 r2 (restart from head) */
        load8(mcu.r[2], ind_addr(3, 0xa220));
        mcu.pc = 0x171e;
        return 1;
    case 0x171e: /* BMI 0x173b */
        mcu.pc = bmi(0x173b, 0x1720);
        return 1;
    case 0x1720: /* bsr16 0x17ed (note_helper) */
        MCU_PushStack(0x1723);
        mcu.pc = 0x17ed;
        return 1;
    case 0x1723: /* BLE 0x173d */
        mcu.pc = ble(0x173d, 0x1725);
        return 1;
    case 0x1725: /* MOVG2.B @r3+0xa210 r0 */
        load8(mcu.r[0], ind_addr(3, 0xa210));
        mcu.pc = 0x1729;
        return 1;
    case 0x1729: /* SUB.B @r3+0x8018 r0 */
        sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018));
        mcu.pc = 0x172d;
        return 1;
    case 0x172d: /* BLE 0x173b */
        mcu.pc = ble(0x173b, 0x172f);
        return 1;
    case 0x172f: /* MOVG2.B r5 r2 */
        mov_reg8(mcu.r[2], mcu.r[5]);
        mcu.pc = 0x1731;
        return 1;
    case 0x1731: /* BPL 0x1720: retry until r2 sign set */
        mcu.pc = bpl(0x1720, 0x1733);
        return 1;
    case 0x1733: /* BRA 0x173b (unexecuted arm) */
        mcu.pc = 0x173b;
        return 1;
    case 0x1735: /* MOVG2.B @r2+0xa250 r2 (unexecuted arm) */
        load8(mcu.r[2], ind_addr(2, 0xa250));
        mcu.pc = 0x1739;
        return 1;
    case 0x1739: /* BPL 0x1720 (unexecuted arm) */
        mcu.pc = bpl(0x1720, 0x173b);
        return 1;
    case 0x173b: /* move r0 #0x01: chain exhausted */
        mov_reg8(mcu.r[0], 0x01);
        mcu.pc = 0x173d;
        return 1;
    case 0x173d: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- B9 key_on 0x173e..0x179b (34 PCs) ------------------------------------ */
/* ROM 0x173e..0x179b:
 *   f3 a2 00 84 56 00 0e 56 2b 15 f1 a3 84 82 1e 00 9e 2f 4a f3 a2 10 80
 *   f3 80 18 30 2e e7 20 3c 56 01 1e 00 3a 2b 15 f1 a3 84 82 1e 00 82
 *   2f 2e f3 a2 10 80 f3 80 18 30 2e e6 20 20 f3 a2 00 16 2b 1a 56 02
 *   1e 00 18 2b 13 f1 a3 84 82 1e 00 60 2f 0c f3 a2 10 80 f3 80 18 30
 *   2e e6 50 01 19
 * Three passes r6 = 0/1/2 (normal/effect/low); 0x178f/0x1793/0x1797 are the
 * unexecuted third-pass tail (C: ROM bytes). */
uint32_t step_key_on(void)
{
    switch (mcu.pc)
    {
    case 0x173e: /* MOVG2.B @r3+0xa200 r4 (current desc) */
        load8(mcu.r[4], ind_addr(3, 0xa200));
        mcu.pc = 0x1742;
        return 1;
    case 0x1742: /* move r6 #0x00: pass 0 */
        mov_reg8(mcu.r[6], 0x00);
        mcu.pc = 0x1744;
        return 1;
    case 0x1744: /* bsr 0x179c (slot_select) */
        MCU_PushStack(0x1746);
        mcu.pc = 0x179c;
        return 1;
    case 0x1746: /* BMI 0x175d: no slot, try pass 1 */
        mcu.pc = bmi(0x175d, 0x1748);
        return 1;
    case 0x1748: /* MOVG2.B @r1+0xa384 r2 (selected desc) */
        load8(mcu.r[2], ind_addr(1, 0xa384));
        mcu.pc = 0x174c;
        return 1;
    case 0x174c: /* bsr16 0x17ed (note_helper) */
        MCU_PushStack(0x174f);
        mcu.pc = 0x17ed;
        return 1;
    case 0x174f: /* BLE 0x179b: shortfall >= 0, done */
        mcu.pc = ble(0x179b, 0x1751);
        return 1;
    case 0x1751: /* MOVG2.B @r3+0xa210 r0 */
        load8(mcu.r[0], ind_addr(3, 0xa210));
        mcu.pc = 0x1755;
        return 1;
    case 0x1755: /* SUB.B @r3+0x8018 r0 */
        sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018));
        mcu.pc = 0x1759;
        return 1;
    case 0x1759: /* BGT 0x1742: still above limit, retry pass 0 */
        mcu.pc = bgt(0x1742, 0x175b);
        return 1;
    case 0x175b: /* BRA 0x1799: success */
        mcu.pc = 0x1799;
        return 1;
    case 0x175d: /* move r6 #0x01: pass 1 */
        mov_reg8(mcu.r[6], 0x01);
        mcu.pc = 0x175f;
        return 1;
    case 0x175f: /* bsr16 0x179c (slot_select) */
        MCU_PushStack(0x1762);
        mcu.pc = 0x179c;
        return 1;
    case 0x1762: /* BMI 0x1779: no slot, try pass 2 */
        mcu.pc = bmi(0x1779, 0x1764);
        return 1;
    case 0x1764: /* MOVG2.B @r1+0xa384 r2 */
        load8(mcu.r[2], ind_addr(1, 0xa384));
        mcu.pc = 0x1768;
        return 1;
    case 0x1768: /* bsr16 0x17ed (note_helper) */
        MCU_PushStack(0x176b);
        mcu.pc = 0x17ed;
        return 1;
    case 0x176b: /* BLE 0x179b */
        mcu.pc = ble(0x179b, 0x176d);
        return 1;
    case 0x176d: /* MOVG2.B @r3+0xa210 r0 */
        load8(mcu.r[0], ind_addr(3, 0xa210));
        mcu.pc = 0x1771;
        return 1;
    case 0x1771: /* SUB.B @r3+0x8018 r0 */
        sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018));
        mcu.pc = 0x1775;
        return 1;
    case 0x1775: /* BGT 0x175d: retry pass 1 */
        mcu.pc = bgt(0x175d, 0x1777);
        return 1;
    case 0x1777: /* BRA 0x1799 */
        mcu.pc = 0x1799;
        return 1;
    case 0x1779: /* TST.B @r3+0xa200 (current desc valid?) */
        tst8_mem(ind_addr(3, 0xa200));
        mcu.pc = 0x177d;
        return 1;
    case 0x177d: /* BMI 0x1799: no current desc, success anyway */
        mcu.pc = bmi(0x1799, 0x177f);
        return 1;
    case 0x177f: /* move r6 #0x02: pass 2 */
        mov_reg8(mcu.r[6], 0x02);
        mcu.pc = 0x1781;
        return 1;
    case 0x1781: /* bsr16 0x179c (slot_select) */
        MCU_PushStack(0x1784);
        mcu.pc = 0x179c;
        return 1;
    case 0x1784: /* BMI 0x1799: no slot, success */
        mcu.pc = bmi(0x1799, 0x1786);
        return 1;
    case 0x1786: /* MOVG2.B @r1+0xa384 r2 */
        load8(mcu.r[2], ind_addr(1, 0xa384));
        mcu.pc = 0x178a;
        return 1;
    case 0x178a: /* bsr16 0x17ed (note_helper) */
        MCU_PushStack(0x178d);
        mcu.pc = 0x17ed;
        return 1;
    case 0x178d: /* BLE 0x179b */
        mcu.pc = ble(0x179b, 0x178f);
        return 1;
    case 0x178f: /* MOVG2.B @r3+0xa210 r0 (unexecuted arm) */
        load8(mcu.r[0], ind_addr(3, 0xa210));
        mcu.pc = 0x1793;
        return 1;
    case 0x1793: /* SUB.B @r3+0x8018 r0 (unexecuted arm) */
        sub8_reg_mem(mcu.r[0], ind_addr(3, 0x8018));
        mcu.pc = 0x1797;
        return 1;
    case 0x1797: /* BGT 0x177f: retry pass 2 (unexecuted arm) */
        mcu.pc = bgt(0x177f, 0x1799);
        return 1;
    case 0x1799: /* move r0 #0x01: success */
        mov_reg8(mcu.r[0], 0x01);
        mcu.pc = 0x179b;
        return 1;
    case 0x179b: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- B10 slot_select 0x179c..0x17ec (27 PCs) ------------------------------ */
/* ROM 0x179c..0x17ec:
 *   15 a4 38 06 ff 50 ff f3 a2 20 82 2b 3f 46 02 27 0f 46 01 27 07
 *   f2 a2 88 04 00 27 2a a2 74 27 26 f2 a2 dc 81 f1 ad 0e 70 23 08
 *   f1 ad 0e 80 15 a4 38 91 f1 a4 10 81 2b 0e f1 ad 0e 70 23 08
 *   f1 ad 0e 80 15 a4 38 91 f2 a2 50 82 2a c1 15 a4 38 81 19
 * Selects the PCM channel (a438 := min ad0e voice index) over the part's
 * descriptor chain; r6 selects the pass (0/1/2), r0 is the running minimum. */
uint32_t step_slot_select(void)
{
    switch (mcu.pc)
    {
    case 0x179c: /* MOVG #0xff -> (dp,0xa438) (selected slot = none) */
        mov_imm8(dp_addr(0xa438), 0xff);
        mcu.pc = 0x17a1;
        return 1;
    case 0x17a1: /* move r0 #0xff (running minimum) */
        mov_reg8(mcu.r[0], 0xff);
        mcu.pc = 0x17a3;
        return 1;
    case 0x17a3: /* MOVG2.B @r3+0xa220 r2 (desc head) */
        load8(mcu.r[2], ind_addr(3, 0xa220));
        mcu.pc = 0x17a7;
        return 1;
    case 0x17a7: /* BMI 0x17e8: chain empty, return a438 */
        mcu.pc = bmi(0x17e8, 0x17a9);
        return 1;
    case 0x17a9: /* cmp r6,b #0x02 */
        cmp8_short_imm(mcu.r[6], 0x02);
        mcu.pc = 0x17ab;
        return 1;
    case 0x17ab: /* BEQ 0x17bc: pass 2 ignores a288/r4 filters */
        mcu.pc = beq(0x17bc, 0x17ad);
        return 1;
    case 0x17ad: /* cmp r6,b #0x01 */
        cmp8_short_imm(mcu.r[6], 0x01);
        mcu.pc = 0x17af;
        return 1;
    case 0x17af: /* BEQ 0x17b8: pass 1 skips the a288 test */
        mcu.pc = beq(0x17b8, 0x17b1);
        return 1;
    case 0x17b1: /* SUB.B @r2+0xa288 #0x00 (pass 0: need free desc) */
        sub8_mem_imm_flags(ind_addr(2, 0xa288), 0x00);
        mcu.pc = 0x17b6;
        return 1;
    case 0x17b6: /* BEQ 0x17e2: skip non-free desc */
        mcu.pc = beq(0x17e2, 0x17b8);
        return 1;
    case 0x17b8: /* CMP.B r2 r4 (skip the current desc) */
        cmp8_regs(mcu.r[4], mcu.r[2]);
        mcu.pc = 0x17ba;
        return 1;
    case 0x17ba: /* BEQ 0x17e2 */
        mcu.pc = beq(0x17e2, 0x17bc);
        return 1;
    case 0x17bc: /* MOVG2.B @r2+0xa2dc r1 (voice chain tail) */
        load8(mcu.r[1], ind_addr(2, 0xa2dc));
        mcu.pc = 0x17c0;
        return 1;
    case 0x17c0: /* CMP.B @r1+0xad0e r0 (age vs minimum) */
        cmp8_reg_mem(mcu.r[0], ind_addr(1, 0xad0e));
        mcu.pc = 0x17c4;
        return 1;
    case 0x17c4: /* BLS 0x17ce: r0 <= age, keep minimum */
        mcu.pc = bls(0x17ce, 0x17c6);
        return 1;
    case 0x17c6: /* MOVG2.B @r1+0xad0e r0 (new minimum) */
        load8(mcu.r[0], ind_addr(1, 0xad0e));
        mcu.pc = 0x17ca;
        return 1;
    case 0x17ca: /* MOVG3.B r1 -> (dp,0xa438) */
        store8(dp_addr(0xa438), (uint8_t)mcu.r[1]);
        mcu.pc = 0x17ce;
        return 1;
    case 0x17ce: /* MOVG2.B @r1+0xa410 r1 (next voice in desc chain) */
        load8(mcu.r[1], ind_addr(1, 0xa410));
        mcu.pc = 0x17d2;
        return 1;
    case 0x17d2: /* BMI 0x17e2: chain end */
        mcu.pc = bmi(0x17e2, 0x17d4);
        return 1;
    case 0x17d4: /* CMP.B @r1+0xad0e r0 */
        cmp8_reg_mem(mcu.r[0], ind_addr(1, 0xad0e));
        mcu.pc = 0x17d8;
        return 1;
    case 0x17d8: /* BLS 0x17e2 */
        mcu.pc = bls(0x17e2, 0x17da);
        return 1;
    case 0x17da: /* MOVG2.B @r1+0xad0e r0 */
        load8(mcu.r[0], ind_addr(1, 0xad0e));
        mcu.pc = 0x17de;
        return 1;
    case 0x17de: /* MOVG3.B r1 -> (dp,0xa438) */
        store8(dp_addr(0xa438), (uint8_t)mcu.r[1]);
        mcu.pc = 0x17e2;
        return 1;
    case 0x17e2: /* MOVG2.B @r2+0xa250 r2 (next desc) */
        load8(mcu.r[2], ind_addr(2, 0xa250));
        mcu.pc = 0x17e6;
        return 1;
    case 0x17e6: /* BPL 0x17a9: walk the desc chain */
        mcu.pc = bpl(0x17a9, 0x17e8);
        return 1;
    case 0x17e8: /* MOVG2.B (dp,0xa438) r1 (result) */
        load8(mcu.r[1], dp_addr(0xa438));
        mcu.pc = 0x17ec;
        return 1;
    case 0x17ec: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- B11 note_helper 0x17ed..0x1805 (9 PCs) ------------------------------- */
/* ROM 0x17ed..0x1805: a9 13 ad 13 f2 a2 50 85 f2 a2 dc 81 0e 28
 *                     f2 a2 dc 81 2a f8 15 a4 2c 16 19
 * Walks the voice chain of desc r2 (r5 keeps the chain head across the
 * release calls) and drains the shortfall through release A (0x1823). */
uint32_t step_note_helper(void)
{
    switch (mcu.pc)
    {
    case 0x17ed: /* CLR r1 */
        clr_reg(mcu.r[1]);
        mcu.pc = 0x17ef;
        return 1;
    case 0x17ef: /* CLR r5 */
        clr_reg(mcu.r[5]);
        mcu.pc = 0x17f1;
        return 1;
    case 0x17f1: /* MOVG2.B @r2+0xa250 r5 (desc next, kept across calls) */
        load8(mcu.r[5], ind_addr(2, 0xa250));
        mcu.pc = 0x17f5;
        return 1;
    case 0x17f5: /* MOVG2.B @r2+0xa2dc r1 (voice chain tail) */
        load8(mcu.r[1], ind_addr(2, 0xa2dc));
        mcu.pc = 0x17f9;
        return 1;
    case 0x17f9: /* bsr 0x1823 (release A) */
        MCU_PushStack(0x17fb);
        mcu.pc = 0x1823;
        return 1;
    case 0x17fb: /* MOVG2.B @r2+0xa2dc r1 (new chain tail) */
        load8(mcu.r[1], ind_addr(2, 0xa2dc));
        mcu.pc = 0x17ff;
        return 1;
    case 0x17ff: /* BPL 0x17f9: while a voice remains */
        mcu.pc = bpl(0x17f9, 0x1801);
        return 1;
    case 0x1801: /* TST.B (dp,0xa42c) (shortfall flag for the caller) */
        tst8_mem(dp_addr(0xa42c));
        mcu.pc = 0x1805;
        return 1;
    case 0x1805: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- B12 aux_alloc 0x180d..0x1822 (7 PCs) --------------------------------- */
/* ROM 0x180d..0x1822: f2 a2 50 85 f2 a2 dc 81 1e 00 66 f2 a2 dc 81
 *                     2a f7 15 a4 2c 16 19
 * Same shape as B11 but drains through release B (0x187e). */
uint32_t step_aux_alloc(void)
{
    switch (mcu.pc)
    {
    case 0x180d: /* MOVG2.B @r2+0xa250 r5 */
        load8(mcu.r[5], ind_addr(2, 0xa250));
        mcu.pc = 0x1811;
        return 1;
    case 0x1811: /* MOVG2.B @r2+0xa2dc r1 */
        load8(mcu.r[1], ind_addr(2, 0xa2dc));
        mcu.pc = 0x1815;
        return 1;
    case 0x1815: /* bsr16 0x187e (release B) */
        MCU_PushStack(0x1818);
        mcu.pc = 0x187e;
        return 1;
    case 0x1818: /* MOVG2.B @r2+0xa2dc r1 */
        load8(mcu.r[1], ind_addr(2, 0xa2dc));
        mcu.pc = 0x181c;
        return 1;
    case 0x181c: /* BPL 0x1815 */
        mcu.pc = bpl(0x1815, 0x181e);
        return 1;
    case 0x181e: /* TST.B (dp,0xa42c) */
        tst8_mem(dp_addr(0xa42c));
        mcu.pc = 0x1822;
        return 1;
    case 0x1822: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

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
uint32_t step_desc_setup(void)
{
    switch (mcu.pc)
    {
    case 0x18ce: /* CLR r3 */
        clr_reg(mcu.r[3]);
        mcu.pc = 0x18d0;
        return 1;
    case 0x18d0: /* MOVG2.B (dp,0xa4a4) r3 (part index) */
        load8(mcu.r[3], dp_addr(0xa4a4));
        mcu.pc = 0x18d4;
        return 1;
    case 0x18d4: /* CLR r2 */
        clr_reg(mcu.r[2]);
        mcu.pc = 0x18d6;
        return 1;
    case 0x18d6: /* MOVG2.B (dp,0xa42e) r2 (desc free head) */
        load8(mcu.r[2], dp_addr(0xa42e));
        mcu.pc = 0x18da;
        return 1;
    case 0x18da: /* MOVG2.B @r2+0xa250 r0 (desc.next) */
        load8(mcu.r[0], ind_addr(2, 0xa250));
        mcu.pc = 0x18de;
        return 1;
    case 0x18de: /* MOVG3.B r0 -> (dp,0xa42e) (free head = next) */
        store8(dp_addr(0xa42e), (uint8_t)mcu.r[0]);
        mcu.pc = 0x18e2;
        return 1;
    case 0x18e2: /* CLR r0 */
        clr_reg(mcu.r[0]);
        mcu.pc = 0x18e4;
        return 1;
    case 0x18e4: /* MOVG2.B @r3+0xa230 r0 (part desc tail) */
        load8(mcu.r[0], ind_addr(3, 0xa230));
        mcu.pc = 0x18e8;
        return 1;
    case 0x18e8: /* BPL 0x18f0: non-empty chain */
        mcu.pc = bpl(0x18f0, 0x18ea);
        return 1;
    case 0x18ea: /* MOVG3.B r2 -> @r3+0xa220 (part head = desc) */
        store8(ind_addr(3, 0xa220), (uint8_t)mcu.r[2]);
        mcu.pc = 0x18ee;
        return 1;
    case 0x18ee: /* BRA 0x18f4 */
        mcu.pc = 0x18f4;
        return 1;
    case 0x18f0: /* MOVG3.B r2 -> @r0+0xa250 (old tail.next = desc) */
        store8(ind_addr(0, 0xa250), (uint8_t)mcu.r[2]);
        mcu.pc = 0x18f4;
        return 1;
    case 0x18f4: /* MOVG3.B r2 -> @r3+0xa230 (part tail = desc) */
        store8(ind_addr(3, 0xa230), (uint8_t)mcu.r[2]);
        mcu.pc = 0x18f8;
        return 1;
    case 0x18f8: /* MOVG3.B r0 -> @r2+0xa26c (desc.prev = old tail) */
        store8(ind_addr(2, 0xa26c), (uint8_t)mcu.r[0]);
        mcu.pc = 0x18fc;
        return 1;
    case 0x18fc: /* MOVG #0xff -> @r2+0xa250 (desc.next = none) */
        mov_imm8(ind_addr(2, 0xa250), 0xff);
        mcu.pc = 0x1901;
        return 1;
    case 0x1901: /* MOVG2.B (dp,0xa4a5) r0 */
        load8(mcu.r[0], dp_addr(0xa4a5));
        mcu.pc = 0x1905;
        return 1;
    case 0x1905: /* MOVG3.B r0 -> @r2+0xa2f8 */
        store8(ind_addr(2, 0xa2f8), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1909;
        return 1;
    case 0x1909: /* MOVG2.B (dp,0xa4a6) r0 */
        load8(mcu.r[0], dp_addr(0xa4a6));
        mcu.pc = 0x190d;
        return 1;
    case 0x190d: /* MOVG3.B r0 -> @r2+0xa314 (desc age) */
        store8(ind_addr(2, 0xa314), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1911;
        return 1;
    case 0x1911: /* MOVG2.B (dp,0xa1e0) r0 */
        load8(mcu.r[0], dp_addr(0xa1e0));
        mcu.pc = 0x1915;
        return 1;
    case 0x1915: /* MOVG3.B r0 -> @r2+0xa330 */
        store8(ind_addr(2, 0xa330), (uint8_t)mcu.r[0]);
        mcu.pc = 0x1919;
        return 1;
    case 0x1919: /* MOVG3.B r2 -> (dp,0xa4a9) (setup desc register) */
        store8(dp_addr(0xa4a9), (uint8_t)mcu.r[2]);
        mcu.pc = 0x191d;
        return 1;
    case 0x191d: /* bsr16 0x1b90 (H3: desc state + a200 candidate) */
        MCU_PushStack(0x1920);
        mcu.pc = 0x1b90;
        return 1;
    case 0x1920: /* CLR r4 */
        clr_reg(mcu.r[4]);
        mcu.pc = 0x1922;
        return 1;
    case 0x1922: /* MOVG2.B (dp,0xa4a8) r4 (voices to allocate) */
        load8(mcu.r[4], dp_addr(0xa4a8));
        mcu.pc = 0x1926;
        return 1;
    case 0x1926: /* MOVG2.W r4 r0 */
        mov_reg16(mcu.r[0], mcu.r[4]);
        mcu.pc = 0x1928;
        return 1;
    case 0x1928: /* MOVG #0xff -> @r0+0xa4aa (clear per-part voice list) */
        mov_imm8(ind_addr(0, 0xa4aa), 0xff);
        mcu.pc = 0x192d;
        return 1;
    case 0x192d: /* ADDQ #-1 r4 (byte) */
        addq8_reg(mcu.r[4], -1);
        mcu.pc = 0x192f;
        return 1;
    case 0x192f: /* bsr 0x19ad (pool_pop: free voice -> r1) */
        MCU_PushStack(0x1931);
        mcu.pc = 0x19ad;
        return 1;
    case 0x1931: /* MOVG2.W r4 r0 */
        mov_reg16(mcu.r[0], mcu.r[4]);
        mcu.pc = 0x1933;
        return 1;
    case 0x1933: /* MOVG3.B r1 -> @r0+0xa4aa (voice list append) */
        store8(ind_addr(0, 0xa4aa), (uint8_t)mcu.r[1]);
        mcu.pc = 0x1937;
        return 1;
    case 0x1937: /* MOVG2.B (dp,0xa4a6) r6 (desc age) */
        load8(mcu.r[6], dp_addr(0xa4a6));
        mcu.pc = 0x193b;
        return 1;
    case 0x193b: /* MOVG3.B r6 -> @r1+0xd118 (voice age) */
        store8(ind_addr(1, 0xd118), (uint8_t)mcu.r[6]);
        mcu.pc = 0x193f;
        return 1;
    case 0x193f: /* bsr 0x194c (link: voice -> desc/part chains) */
        MCU_PushStack(0x1941);
        mcu.pc = 0x194c;
        return 1;
    case 0x1941: /* ADDQ #1 @r3+0xa210 (part voice count++) */
        addq8_mem(ind_addr(3, 0xa210), 1);
        mcu.pc = 0x1945;
        return 1;
    case 0x1945: /* SUB #0x0001 r4 (word shortfall--) */
        sub16_reg_imm(mcu.r[4], 0x0001);
        mcu.pc = 0x1949;
        return 1;
    case 0x1949: /* BPL 0x192f: more voices to allocate */
        mcu.pc = bpl(0x192f, 0x194b);
        return 1;
    case 0x194b: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- B19 cleanup_seq 0x1a4d..0x1a6b (10 PCs) ------------------------------ */
/* ROM 0x1a4d..0x1a6b: 15 a1 d1 f7 27 18 a9 13 15 a4 aa 81 2b 05
 *                     f1 ce 3f 06 ff 15 a4 ab 81 2b 05 f1 ce 3f 06 ff 19
 * If (dp,0xa1d1) bit7 is set, clears both per-part voice list heads
 * (a4aa/a4ab) by writing 0xff into each voice's ce3f byte. */
uint32_t step_cleanup_seq(void)
{
    switch (mcu.pc)
    {
    case 0x1a4d: /* BTSTI (dp,0xa1d1) #7 */
        btsti(dp_addr(0xa1d1), 7);
        mcu.pc = 0x1a51;
        return 1;
    case 0x1a51: /* BEQ 0x1a6b: nothing to clean */
        mcu.pc = beq(0x1a6b, 0x1a53);
        return 1;
    case 0x1a53: /* CLR r1 */
        clr_reg(mcu.r[1]);
        mcu.pc = 0x1a55;
        return 1;
    case 0x1a55: /* MOVG2.B (dp,0xa4aa) r1 (list A head) */
        load8(mcu.r[1], dp_addr(0xa4aa));
        mcu.pc = 0x1a59;
        return 1;
    case 0x1a59: /* BMI 0x1a60: list empty */
        mcu.pc = bmi(0x1a60, 0x1a5b);
        return 1;
    case 0x1a5b: /* MOVG #0xff -> @r1+0xce3f */
        mov_imm8(ind_addr(1, 0xce3f), 0xff);
        mcu.pc = 0x1a60;
        return 1;
    case 0x1a60: /* MOVG2.B (dp,0xa4ab) r1 (list B head) */
        load8(mcu.r[1], dp_addr(0xa4ab));
        mcu.pc = 0x1a64;
        return 1;
    case 0x1a64: /* BMI 0x1a6b: list empty */
        mcu.pc = bmi(0x1a6b, 0x1a66);
        return 1;
    case 0x1a66: /* MOVG #0xff -> @r1+0xce3f */
        mov_imm8(ind_addr(1, 0xce3f), 0xff);
        mcu.pc = 0x1a6b;
        return 1;
    case 0x1a6b: /* rts */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

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
uint32_t step_pcm_stop(void)
{
    switch (mcu.pc)
    {
    case 0x5238: /* CLR @r1+0xad0e (release the old channel) */
        clr8_mem(ind_addr(1, 0xad0e));
        mcu.pc = 0x523c;
        return 1;
    case 0x523c: /* MOVG2.W r1 r2 */
        mov_reg16(mcu.r[2], mcu.r[1]);
        mcu.pc = 0x523e;
        return 1;
    case 0x523e: /* ADD r2 r2 (slot * 2) */
        add16_self(mcu.r[2]);
        mcu.pc = 0x5240;
        return 1;
    case 0x5240: /* MOVG2.W @r2+0x64d6 r2 (AoS record pointer) */
        load16(mcu.r[2], ind_addr(2, 0x64d6));
        mcu.pc = 0x5244;
        return 1;
    case 0x5244: /* SUB.W @r2+0 #0x12 (record state == 0x12?) */
        sub16_mem_imm_flags(ind_addr(2, 0), 0x0012);
        mcu.pc = 0x5248;
        return 1;
    case 0x5248: /* BNE 0x5253: other state */
        mcu.pc = bne(0x5253, 0x524a);
        return 1;
    case 0x524a: /* movs r1 @(br,0x3e) (write select byte) */
        movs8_br(1, 0x3e);
        mcu.pc = 0x524c;
        return 1;
    case 0x524c: /* movl r5 @(br,0x32) (PCM readback byte) */
        movl8_br(5, 0x32);
        mcu.pc = 0x524e;
        return 1;
    case 0x524e: /* movi r6 #0x000e */
        mov_reg16(mcu.r[6], 0x000e);
        mcu.pc = 0x5251;
        return 1;
    case 0x5251: /* BRA 0x525a */
        mcu.pc = 0x525a;
        return 1;
    case 0x5253: /* movs r1 @(br,0x3e) */
        movs8_br(1, 0x3e);
        mcu.pc = 0x5255;
        return 1;
    case 0x5255: /* movl r5 @(br,0x34) */
        movl8_br(5, 0x34);
        mcu.pc = 0x5257;
        return 1;
    case 0x5257: /* movi r6 #0x0010 */
        mov_reg16(mcu.r[6], 0x0010);
        mcu.pc = 0x525a;
        return 1;
    case 0x525a: /* movlw r5 @(br,0x3a) (word readback) */
        movlw16_br(5, 0x3a);
        mcu.pc = 0x525c;
        return 1;
    case 0x525c: /* BEQ 0x5275: readback zero */
        mcu.pc = beq(0x5275, 0x525e);
        return 1;
    case 0x525e: /* ADD r5 r5 (byte offset * 2) */
        add16_self(mcu.r[5]);
        mcu.pc = 0x5260;
        return 1;
    case 0x5260: /* SWAP r5 (high byte becomes the channel index) */
        swap_reg(mcu.r[5]);
        mcu.pc = 0x5262;
        return 1;
    case 0x5262: /* MOVG3.B r5 -> @r1+0xad0e (channel = readback) */
        store8(ind_addr(1, 0xad0e), (uint8_t)mcu.r[5]);
        mcu.pc = 0x5266;
        return 1;
    case 0x5266: /* MOVG3.W r6 -> @r2+0 (state) */
        store16(ind_addr(2, 0), mcu.r[6]);
        mcu.pc = 0x5269;
        return 1;
    case 0x5269: /* MOVG3.W r6 -> @r2+2 */
        store16(ind_addr(2, 2), mcu.r[6]);
        mcu.pc = 0x526c;
        return 1;
    case 0x526c: /* MOVG3.W r6 -> @r2+4 */
        store16(ind_addr(2, 4), mcu.r[6]);
        mcu.pc = 0x526f;
        return 1;
    case 0x526f: /* BCLR_ANDC #0xf8ff r0: IML=0 */
        bclr_andc_iml0();
        mcu.pc = 0x5273;
        return 1;
    case 0x5273: /* BRA 0x51fc (dispatcher, other slice) */
        mcu.pc = 0x51fc;
        return 1;
    case 0x5275: /* CLR @r1+0xad0e */
        clr8_mem(ind_addr(1, 0xad0e));
        mcu.pc = 0x5279;
        return 1;
    case 0x5279: /* movi r6 #0x0016 (idle state) */
        mov_reg16(mcu.r[6], 0x0016);
        mcu.pc = 0x527c;
        return 1;
    case 0x527c: /* MOVG3.W r6 -> @r2+0 */
        store16(ind_addr(2, 0), mcu.r[6]);
        mcu.pc = 0x527f;
        return 1;
    case 0x527f: /* MOVG3.W r6 -> @r2+2 */
        store16(ind_addr(2, 2), mcu.r[6]);
        mcu.pc = 0x5282;
        return 1;
    case 0x5282: /* MOVG3.W r6 -> @r2+4 */
        store16(ind_addr(2, 4), mcu.r[6]);
        mcu.pc = 0x5285;
        return 1;
    case 0x5285: /* SUB.B @r1+0xd0a8 #0xff (start-prev valid?) */
        sub8_mem_imm_flags(ind_addr(1, 0xd0a8), 0xff);
        mcu.pc = 0x528a;
        return 1;
    case 0x528a: /* BNE 0x5299: exists, unlink it */
        mcu.pc = bne(0x5299, 0x528c);
        return 1;
    case 0x528c: /* SUB.B @r1+0xd0c4 #0xff (start-next valid?) */
        sub8_mem_imm_flags(ind_addr(1, 0xd0c4), 0xff);
        mcu.pc = 0x5291;
        return 1;
    case 0x5291: /* BEQ 0x52b3: neither neighbour exists */
        mcu.pc = beq(0x52b3, 0x5293);
        return 1;
    case 0x5293: /* MOVG2.B @r1+0xd0c4 r3 (unexecuted arm) */
        load8(mcu.r[3], ind_addr(1, 0xd0c4));
        mcu.pc = 0x5297;
        return 1;
    case 0x5297: /* BRA 0x529d (unexecuted arm) */
        mcu.pc = 0x529d;
        return 1;
    case 0x5299: /* MOVG2.B @r1+0xd0a8 r3 */
        load8(mcu.r[3], ind_addr(1, 0xd0a8));
        mcu.pc = 0x529d;
        return 1;
    case 0x529d: /* EXTU r3 (zero-extend the neighbour index) */
        extu(mcu.r[3]);
        mcu.pc = 0x529f;
        return 1;
    case 0x529f: /* MOVG #0xff -> @r1+0xd0a8 (clear own prev link) */
        mov_imm8(ind_addr(1, 0xd0a8), 0xff);
        mcu.pc = 0x52a4;
        return 1;
    case 0x52a4: /* MOVG #0xff -> @r1+0xd0c4 (clear own next link) */
        mov_imm8(ind_addr(1, 0xd0c4), 0xff);
        mcu.pc = 0x52a9;
        return 1;
    case 0x52a9: /* MOVG #0xff -> @r3+0xd0a8 (neighbour prev = none) */
        mov_imm8(ind_addr(3, 0xd0a8), 0xff);
        mcu.pc = 0x52ae;
        return 1;
    case 0x52ae: /* MOVG #0xff -> @r3+0xd0c4 (neighbour next = none) */
        mov_imm8(ind_addr(3, 0xd0c4), 0xff);
        mcu.pc = 0x52b3;
        return 1;
    case 0x52b3: /* movs r1 @(br,0x3e) */
        movs8_br(1, 0x3e);
        mcu.pc = 0x52b5;
        return 1;
    case 0x52b5: /* MOVG #0x00b5 -> (br,0x18) (PCM main volume) */
        mov_imm16_br(0x18, 0x00b5);
        mcu.pc = 0x52ba;
        return 1;
    case 0x52ba: /* MOVG3.W r1 -> (dp,0xa1f6) (PCM select shadow) */
        store16(dp_addr(0xa1f6), mcu.r[1]);
        mcu.pc = 0x52be;
        return 1;
    case 0x52be: /* BCLR_ANDC #0xf8ff r0: IML=0 */
        bclr_andc_iml0();
        mcu.pc = 0x52c2;
        return 1;
    case 0x52c2: /* move r0 #0x01 */
        mov_reg8(mcu.r[0], 0x01);
        mcu.pc = 0x52c4;
        return 1;
    case 0x52c4: /* move r1 #0x01 */
        mov_reg8(mcu.r[1], 0x01);
        mcu.pc = 0x52c6;
        return 1;
    case 0x52c6: /* trapa (opcode 0x12 -> vector 2) */
        MCU_Interrupt_TRAPA(2);
        mcu.pc = 0x52c8;
        return 1;
    case 0x52c8: /* BRA 0x51fc (dispatcher, other slice) */
        mcu.pc = 0x51fc;
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- PC tables: one host step per instruction ----------------------------- */

const uint16_t kStealPcs[] = {
    0x16f4, 0x16f8, 0x16fa, 0x16ff, 0x1701, 0x1704, 0x1706, 0x170a,
    0x170e, 0x1710, 0x1712, 0x1714, 0x1718, 0x171a, 0x171e, 0x1720,
    0x1723, 0x1725, 0x1729, 0x172d, 0x172f, 0x1731, 0x1733, 0x1735,
    0x1739, 0x173b, 0x173d,
};

const uint16_t kKeyOnPcs[] = {
    0x173e, 0x1742, 0x1744, 0x1746, 0x1748, 0x174c, 0x174f, 0x1751,
    0x1755, 0x1759, 0x175b, 0x175d, 0x175f, 0x1762, 0x1764, 0x1768,
    0x176b, 0x176d, 0x1771, 0x1775, 0x1777, 0x1779, 0x177d, 0x177f,
    0x1781, 0x1784, 0x1786, 0x178a, 0x178d, 0x178f, 0x1793, 0x1797,
    0x1799, 0x179b,
};

const uint16_t kSlotSelectPcs[] = {
    0x179c, 0x17a1, 0x17a3, 0x17a7, 0x17a9, 0x17ab, 0x17ad, 0x17af,
    0x17b1, 0x17b6, 0x17b8, 0x17ba, 0x17bc, 0x17c0, 0x17c4, 0x17c6,
    0x17ca, 0x17ce, 0x17d2, 0x17d4, 0x17d8, 0x17da, 0x17de, 0x17e2,
    0x17e6, 0x17e8, 0x17ec,
};

const uint16_t kNoteHelperPcs[] = {
    0x17ed, 0x17ef, 0x17f1, 0x17f5, 0x17f9, 0x17fb, 0x17ff, 0x1801,
    0x1805,
};

const uint16_t kAuxAllocPcs[] = {
    0x180d, 0x1811, 0x1815, 0x1818, 0x181c, 0x181e, 0x1822,
};

const uint16_t kDescSetupPcs[] = {
    0x18ce, 0x18d0, 0x18d4, 0x18d6, 0x18da, 0x18de, 0x18e2, 0x18e4,
    0x18e8, 0x18ea, 0x18ee, 0x18f0, 0x18f4, 0x18f8, 0x18fc, 0x1901,
    0x1905, 0x1909, 0x190d, 0x1911, 0x1915, 0x1919, 0x191d, 0x1920,
    0x1922, 0x1926, 0x1928, 0x192d, 0x192f, 0x1931, 0x1933, 0x1937,
    0x193b, 0x193f, 0x1941, 0x1945, 0x1949, 0x194b,
};

const uint16_t kCleanupSeqPcs[] = {
    0x1a4d, 0x1a51, 0x1a53, 0x1a55, 0x1a59, 0x1a5b, 0x1a60, 0x1a64,
    0x1a66, 0x1a6b,
};

const uint16_t kPcmStopPcs[] = {
    0x5238, 0x523c, 0x523e, 0x5240, 0x5244, 0x5248, 0x524a, 0x524c,
    0x524e, 0x5251, 0x5253, 0x5255, 0x5257, 0x525a, 0x525c, 0x525e,
    0x5260, 0x5262, 0x5266, 0x5269, 0x526c, 0x526f, 0x5273, 0x5275,
    0x5279, 0x527c, 0x527f, 0x5282, 0x5285, 0x528a, 0x528c, 0x5291,
    0x5293, 0x5297, 0x5299, 0x529d, 0x529f, 0x52a4, 0x52a9, 0x52ae,
    0x52b3, 0x52b5, 0x52ba, 0x52be, 0x52c2, 0x52c4, 0x52c6, 0x52c8,
};

} /* anonymous namespace */
} /* namespace mk2c */

static void register_step_pcs(const uint16_t *pcs, uint32_t count,
                              mk2cpp_hand_routine_fn fn)
{
    for (uint32_t i = 0; i < count; i++)
        MK2CPP_HandRegisterRoutine(pcs[i], fn);
}

void MK2CPP_NoteChainTailFillTables(void)
{
#if MK2CPP_HAND_NOTE_CHAIN_TAIL
    register_step_pcs(mk2c::kStealPcs,
                      (uint32_t)(sizeof(mk2c::kStealPcs) / sizeof(mk2c::kStealPcs[0])),
                      &mk2c::step_steal);
    register_step_pcs(mk2c::kKeyOnPcs,
                      (uint32_t)(sizeof(mk2c::kKeyOnPcs) / sizeof(mk2c::kKeyOnPcs[0])),
                      &mk2c::step_key_on);
    register_step_pcs(mk2c::kSlotSelectPcs,
                      (uint32_t)(sizeof(mk2c::kSlotSelectPcs) / sizeof(mk2c::kSlotSelectPcs[0])),
                      &mk2c::step_slot_select);
    register_step_pcs(mk2c::kNoteHelperPcs,
                      (uint32_t)(sizeof(mk2c::kNoteHelperPcs) / sizeof(mk2c::kNoteHelperPcs[0])),
                      &mk2c::step_note_helper);
    register_step_pcs(mk2c::kAuxAllocPcs,
                      (uint32_t)(sizeof(mk2c::kAuxAllocPcs) / sizeof(mk2c::kAuxAllocPcs[0])),
                      &mk2c::step_aux_alloc);
    register_step_pcs(mk2c::kDescSetupPcs,
                      (uint32_t)(sizeof(mk2c::kDescSetupPcs) / sizeof(mk2c::kDescSetupPcs[0])),
                      &mk2c::step_desc_setup);
    register_step_pcs(mk2c::kCleanupSeqPcs,
                      (uint32_t)(sizeof(mk2c::kCleanupSeqPcs) / sizeof(mk2c::kCleanupSeqPcs[0])),
                      &mk2c::step_cleanup_seq);
    register_step_pcs(mk2c::kPcmStopPcs,
                      (uint32_t)(sizeof(mk2c::kPcmStopPcs) / sizeof(mk2c::kPcmStopPcs[0])),
                      &mk2c::step_pcm_stop);
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
