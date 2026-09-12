/*
 * HAND voice/note_path -- note_fill (B1) and the first half of the note
 * descriptor chain (B2..B7), one host step per H8 instruction.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * Routines / PC ranges / static count (tools/baselines/dasm_full.txt and the
 * generated map; a dedicated linear h8dasm run over the same PC set decodes
 * every byte arm, including the unexecuted ones):
 *
 *   B1 note_fill   0x0f86..0x118f  163  part a1xx -> SoA materialize + jsr
 *                                        pcm_start; also the 0x1034/0x104d/
 *                                        0x1063 entry blocks and the 0x107d
 *                                        recursion body (07 1.2 B1)
 *   B2 scan_a      0x13a9..0x13c9   12  a220/a250 descriptor walk -> kill_a
 *   B3 kill_a      0x1459..0x14a8   28  mark release, propagate a410 chain
 *   B4 scan_b      0x151e..0x157c   33  clear a240 bit0, arm a3bc/a4b4
 *   B5 alloc_scan  0x157d..0x15d0   28  a42c shortfall, per-part note_disp
 *   B6 note_disp   0x15d1..0x15fa   18  rom1 0x1bfe part table dispatch
 *   B7 note_cmd    0x15fb..0x16a1   64  note command loop, desc_setup call
 *   ------------------------------------------------------------- 346 PCs
 *
 * Why per instruction: every instruction is one MK2CPP_HandRegisterRoutine
 * entry returning 1. The host keeps its per-step interrupt poll, TIMER_Clock,
 * -tracepc and SM_Update cadence; a multi-instruction block defers a due
 * interrupt / queued -midiseq byte to the block end and diverges (the same
 * reason the pool/alloc/materialize slices are per-PC). All of B1/B2/B3/B5/B7
 * run at IML=0 (the only IML=7 pair in B1, 0x10cf..0x10dd, is stepped per
 * instruction too and reproduces the ex_ignore/stack-bank writes exactly).
 *
 * Data model: page-0 SRAM stays authoritative; the cases read/write the same
 * addresses as the stock instructions through MCU_Read/MCU_Read16/MCU_Write/
 * MCU_Write16, so the ROM readers (C9 acf2, C15 P(v), mask_acc) keep working
 * unchanged. `ea_r`/`ea_dp` are the GT general-operand effective addresses
 * (r0-r3 -> dp, r4/r5 -> ep, r6/r7 -> tp; src/mcu_opcodes.cpp). No native
 * shadow copies. Every operand (address, immediate, branch target) is decoded
 * from the ROM bytes; there is no runtime instruction fetch.
 *
 * Confidence: C/S per out/m4/18_closure_gap.md 1.2; the unexecuted arms
 * implemented here (0x0f90..0x0f97 free path, 0x104d/0x1063 entry blocks,
 * 0x1479/0x147f/0x1481/0x1483, 0x154f/0x1551/0x1553, 0x156c/0x1571,
 * 0x159b/0x159f/0x15a5, 0x15ba..0x15ca, 0x1615/0x1616, 0x1626/0x1627,
 * 0x1644/0x1646/0x1648, 0x1698/0x169a) and the overlap decodes
 * (0x1007/0x100c/0x100f/0x10a8/0x1110) are decoded from the ROM bytes and
 * match the generated map's semantics (S).
 *
 * Out of scope and intentionally left to the generated code / interpreter
 * (never hand-shadowed by this module): 0x1190/0x11b8 (B1 callees), 0x1af2,
 * 0x1b09, 0x16f4 (steal, B8), 0x173e (key_on, B9), 0x17ed (note_helper,
 * B11), 0x180d (aux_alloc, B12), 0x18ce (desc_setup, B15), 0x516c
 * (pcm_start, native_allocfree). The bsr/jsr/pc transitions into them are
 * replayed exactly (push return address, set pc).
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_interrupt.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as native_allocfree.cpp / pcm_enable.cpp). */
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);
/* Defined in src/mcu.cpp; used only by the defensive default case. */
void MCU_ReadInstruction(void);

/* Single-entry build switch (09 5.4): 0 makes this slice not register at all
 * without touching other files. */
#define MK2CPP_HAND_NOTE_PATH 1

namespace mk2c {
namespace {

/* ---- page/effective-address helpers ------------------------------------ */

uint32_t page_of(uint32_t reg)
{
    if (reg >= 6)
        return mcu.tp;
    if (reg >= 4)
        return mcu.ep;
    return mcu.dp;
}

/* @rN+disp16: low 16 bits add and wrap, page from the register file. */
uint32_t ea_r(uint32_t reg, uint16_t disp)
{
    return ((uint32_t)page_of(reg) << 16) | (uint16_t)(mcu.r[reg] + disp);
}

uint32_t ea_dp(uint16_t disp)
{
    return ((uint32_t)mcu.dp << 16) | disp;
}

/* ---- flags ------------------------------------------------------------- */

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

bool f_n(void) { return (mcu.sr & STATUS_N) != 0; }
bool f_z(void) { return (mcu.sr & STATUS_Z) != 0; }
bool f_v(void) { return (mcu.sr & STATUS_V) != 0; }
bool f_c(void) { return (mcu.sr & STATUS_C) != 0; }

/* ---- byte/word load and store (GT MOVG2/MOVG3 semantics) --------------- */

void ld8(uint16_t &reg, uint32_t addr)
{
    uint8_t value = MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

void ld16(uint16_t &reg, uint32_t addr)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t value = MCU_Read16(addr);
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

void st8(uint32_t addr, uint32_t value)
{
    MCU_Write(addr, (uint8_t)value);
    MCU_SetStatusCommon(value, 0);
}

void st16(uint32_t addr, uint32_t value)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, (uint16_t)value);
    MCU_SetStatusCommon(value, 1);
}

/* ---- CLR / TST / EXTU --------------------------------------------------- */

void clr16(uint16_t &reg)
{
    reg = 0;
    flags_clr();
}

void clr8_reg(uint16_t &reg)
{
    reg = (uint16_t)(reg & 0xff00u);
    flags_clr();
}

void clr8_mem(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

void tst8_mem(uint32_t addr)
{
    uint32_t data = MCU_Read(addr);
    MCU_SetStatusCommon(data, 0);
    MCU_SetStatus(0, STATUS_C);
}

void tst16_mem(uint32_t addr)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = MCU_Read16(addr);
    MCU_SetStatusCommon(data, 1);
    MCU_SetStatus(0, STATUS_C);
}

void tst8_reg(uint16_t reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    MCU_SetStatusCommon(data, 0);
    MCU_SetStatus(0, STATUS_C);
}

/* EXTU rN: low byte -> full register, N=0 V=0 C=0, Z=1 if byte zero. */
void extu16(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    reg = (uint16_t)data;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(data == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* ---- bit operations (BTSTI / BSET / BCLR) ------------------------------ */

void btst8_mem(uint32_t addr, uint32_t bit)
{
    uint32_t data = MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
}

void bset8_mem(uint32_t addr, uint32_t bit)
{
    uint32_t data = MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    data |= 1u << bit;
    MCU_Write(addr, (uint8_t)data);
}

void bclr8_mem(uint32_t addr, uint32_t bit)
{
    uint32_t data = MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    data &= ~(1u << bit);
    MCU_Write(addr, (uint8_t)data);
}

void btst8_reg(uint16_t reg, uint32_t bit)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
}

void bclr8_reg(uint16_t &reg, uint32_t bit)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    data &= ~(1u << bit);
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
}

/* ---- shifts ------------------------------------------------------------- */

void shll16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t c = (data & 0x8000u) != 0;
    data <<= 1;
    reg = (uint16_t)data;
    MCU_SetStatus(c, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

void shlr16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t c = data & 1u;
    data >>= 1;
    reg = (uint16_t)data;
    MCU_SetStatus(c, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

void shll8(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    uint32_t c = (data & 0x0080u) != 0;
    data <<= 1;
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatus(c, STATUS_C);
    MCU_SetStatusCommon(data, 0);
}

void shlr8(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    uint32_t c = data & 1u;
    data >>= 1;
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatus(c, STATUS_C);
    MCU_SetStatusCommon(data, 0);
}

/* ---- arithmetic / logic -------------------------------------------------- */

/* ADDS #imm8, rN: sign-extended add, no status update (GT short ADDS). */
void adds8(uint16_t &reg, int32_t imm)
{
    reg = (uint16_t)(reg + (uint16_t)(int8_t)imm);
}

void add_imm16_r0(uint16_t imm)
{
    int32_t t1 = (int32_t)mcu.r[0];
    t1 = MCU_ADD_Common(t1, (int32_t)(uint32_t)imm, 0, 1);
    mcu.r[0] = (uint16_t)t1;
}

/* ADDQ #imm8: low-byte add, high byte kept, byte-size flags. */
void addq8(uint16_t &reg, int32_t imm)
{
    uint32_t t1 = (uint32_t)(reg & 0xff);
    t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, imm, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | (t1 & 0xffu));
}

/* ADD rS,rD byte form (GT opcode 4, siz 0): t1 = full Rd, t2 = Rs&0xff. */
void add8_reg(uint16_t &dst, uint16_t src)
{
    int32_t t1 = (int32_t)dst;
    uint32_t t2 = (uint32_t)(src & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    dst = (uint16_t)((dst & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

/* SUB Rd, @mem byte form: t1 = full r0, t2 = mem byte, low byte written back. */
void sub8_r0_mem(uint32_t addr)
{
    int32_t t1 = (int32_t)mcu.r[0];
    uint32_t t2 = (uint32_t)MCU_Read(addr);
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

/* SUB @mem, #imm8: flags only (GT opcode 4 with immediate). */
void sub8_mem_imm8(uint32_t addr, uint8_t imm)
{
    uint32_t t1 = (uint32_t)MCU_Read(addr);
    MCU_SUB_Common((int32_t)t1, (int32_t)imm, 0, 0);
}

/* SUB @mem, #imm16 word form. */
void sub16_mem_imm16(uint32_t addr, uint16_t imm)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t1 = (uint32_t)MCU_Read16(addr);
    MCU_SUB_Common((int32_t)t1, (int32_t)(uint32_t)imm, 0, 1);
}

/* CMP @mem, rN: SUB rN - mem, flags only. */
void cmp8_mem_reg(uint32_t addr, uint16_t reg)
{
    int32_t t1 = (int32_t)reg;
    uint32_t t2 = (uint32_t)MCU_Read(addr);
    MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
}

/* CMP r0++ r1: post-increment r0 by 1, compare r1 with the byte read. */
void cmp8_postinc_r0_r1(void)
{
    uint32_t oea = (uint32_t)mcu.r[0];
    mcu.r[0] = (uint16_t)(mcu.r[0] + 1);
    oea &= 0xffff;
    uint32_t addr = ((uint32_t)page_of(0) << 16) | (uint16_t)oea;
    int32_t t1 = (int32_t)mcu.r[1];
    uint32_t t2 = (uint32_t)MCU_Read(addr);
    MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
}

/* cmp r0,b #imm8: flags only. */
void cmp_imm8_r0(uint8_t imm)
{
    int32_t t2 = (int32_t)imm;
    int32_t t1 = (int32_t)mcu.r[0];
    MCU_SUB_Common(t1, t2, 0, 0);
}

void and8_imm_r0(uint8_t imm)
{
    uint32_t data = (uint32_t)mcu.r[0];
    data &= (uint32_t)imm;
    mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(mcu.r[0], 0);
}

/* ---- immediate / register moves ---------------------------------------- */

void movi16(uint16_t &reg, uint16_t imm)
{
    reg = imm;
    MCU_SetStatusCommon(imm, 1);
}

void move8_imm(uint16_t &reg, uint8_t imm)
{
    reg = (uint16_t)((reg & 0xff00u) | imm);
    MCU_SetStatusCommon(imm, 0);
}

/* MOVG #imm8 -> @rN+disp16 (GT opcode 6, siz 0). */
void mov_imm8_mem(uint32_t addr, int32_t imm)
{
    uint32_t data = (uint32_t)imm;
    MCU_Write(addr, (uint8_t)data);
    MCU_SetStatusCommon(data, 0);
}

/* ---- control flow -------------------------------------------------------- */

/* bsr16/jsr/bsr: push the return address, jump. */
void call(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

void ret(void)
{
    mcu.pc = MCU_PopStack();
}

/* cntjmp rN, disp: rN--; branch back while rN != 0xffff. */
void cntjmp_reg(uint16_t &reg, uint16_t next, uint16_t target)
{
    reg = (uint16_t)(reg - 1);
    mcu.pc = (reg != 0xffff) ? target : next;
}

/* stm/ldm #0x3e exactly as GT implements the bank save/restore. */
void stm_rlist(uint8_t rlist)
{
    for (int i = 7; i >= 0; i--)
    {
        if (rlist & (1 << i))
        {
            uint16_t data = mcu.r[i];
            if (i == 7)
                data = (uint16_t)(data - 2);
            MCU_PushStack(data);
        }
    }
}

void ldm_rlist(uint8_t rlist)
{
    for (int i = 0; i < 8; i++)
    {
        if (rlist & (1 << i))
        {
            uint16_t data = MCU_PopStack();
            if (i != 7)
                mcu.r[i] = data;
        }
    }
}

/* ORC #imm16 / ANDC #imm16 on control register 0, sets ex_ignore. */
void orc_imm16(uint16_t imm)
{
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val |= (uint32_t)imm;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

void andc_imm16(uint16_t imm)
{
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val &= (uint32_t)imm;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

/* ---- odd-format moves used by these routines ---------------------------- */

/* MOVSW r2, @(br,disp8): 16-bit store at (br<<8)|disp. */
void movsw16_r2(uint8_t disp)
{
    uint16_t addr = (uint16_t)((mcu.br << 8) | disp);
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)mcu.r[2];
    MCU_Write16((uint32_t)addr, (uint16_t)data);
    MCU_SetStatusCommon(data, 1);
}

/* MOVF word store: @(tp, r6+disp8) <- r0. */
void movf_store16_r0(int8_t disp)
{
    uint32_t addr = ((uint32_t)mcu.tp << 16) | (uint16_t)(mcu.r[6] + disp);
    uint32_t data = (uint32_t)mcu.r[0];
    MCU_Write16(addr, (uint16_t)data);
    MCU_SetStatusCommon(data, 1);
}

/* MOVF byte load quirk (GT MCU_Opcode_Short_MOVF): byte from @(tp,r6+disp)
 * overwrites the full register and the status is the word form. */
void movf_load_quirk_r0(int8_t disp)
{
    uint32_t addr = ((uint32_t)mcu.tp << 16) | (uint16_t)(mcu.r[6] + disp);
    uint16_t data = MCU_Read(addr);
    mcu.r[0] = data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVG2 rS -> rD word (register direct), e.g. ab 86. */
void move16_reg(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)src;
    dst = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* ======================================================================== */
/* B1 note_fill 0x0f86..0x118f                                              */
/* ======================================================================== */

uint32_t note_fill_step(void)
{
    switch (mcu.pc)
    {
    case 0x0f86: /* CLR @r1+0xa3a0 (byte) */
        clr8_mem(ea_r(1, 0xa3a0));
        mcu.pc = 0x0f8a;
        break;
    case 0x0f8a: /* TST (dp,0xa1f2) word */
        tst16_mem(ea_dp(0xa1f2));
        mcu.pc = 0x0f8e;
        break;
    case 0x0f8e: /* BPL 0x0f9a */
        mcu.pc = f_n() ? 0x0f90 : 0x0f9a;
        break;
    case 0x0f90: /* CLR r2 word (free path) */
        clr16(mcu.r[2]);
        mcu.pc = 0x0f92;
        break;
    case 0x0f92: /* CLR r3 word (free path) */
        clr16(mcu.r[3]);
        mcu.pc = 0x0f94;
        break;
    case 0x0f94: /* bsr16 -> 0x19c4 free_voice (B18) */
        call(0x0f97, 0x19c4);
        break;
    case 0x0f97: /* BRA -> 0x1033 */
        mcu.pc = 0x1033;
        break;
    case 0x0f9a: /* SHLL r1 word */
        shll16(mcu.r[1]);
        mcu.pc = 0x0f9c;
        break;
    case 0x0f9c: /* MOVG3 r5 -> @r1+0xd054 (word) */
        st16(ea_r(1, 0xd054), mcu.r[5]);
        mcu.pc = 0x0fa0;
        break;
    case 0x0fa0: /* MOVG2 (dp,0xa1e4) r0 (word) */
        ld16(mcu.r[0], ea_dp(0xa1e4));
        mcu.pc = 0x0fa4;
        break;
    case 0x0fa4: /* MOVG3 r0 -> @r1+0xcfac (word) */
        st16(ea_r(1, 0xcfac), mcu.r[0]);
        mcu.pc = 0x0fa8;
        break;
    case 0x0fa8: /* MOVG2 (dp,0xa1e6) r0 (word) */
        ld16(mcu.r[0], ea_dp(0xa1e6));
        mcu.pc = 0x0fac;
        break;
    case 0x0fac: /* MOVG3 r0 -> @r1+0xd000 (word) */
        st16(ea_r(1, 0xd000), mcu.r[0]);
        mcu.pc = 0x0fb0;
        break;
    case 0x0fb0: /* SHLR r1 word */
        shlr16(mcu.r[1]);
        mcu.pc = 0x0fb2;
        break;
    case 0x0fb2: /* CLR r0 word */
        clr16(mcu.r[0]);
        mcu.pc = 0x0fb4;
        break;
    case 0x0fb4: /* MOVG2 (dp,0xa4a4) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa4a4));
        mcu.pc = 0x0fb8;
        break;
    case 0x0fb8: /* MOVG3 r0 -> @r1+0xce78 (byte) */
        st8(ea_r(1, 0xce78), mcu.r[0]);
        mcu.pc = 0x0fbc;
        break;
    case 0x0fbc: /* SHLL r0 word */
        shll16(mcu.r[0]);
        mcu.pc = 0x0fbe;
        break;
    case 0x0fbe: /* MOVG2 @r0+0xa1b0 r6 (word) */
        ld16(mcu.r[6], ea_r(0, 0xa1b0));
        mcu.pc = 0x0fc2;
        break;
    case 0x0fc2: /* SHLR r0 word */
        shlr16(mcu.r[0]);
        mcu.pc = 0x0fc4;
        break;
    case 0x0fc4: /* SHLL r1 word */
        shll16(mcu.r[1]);
        mcu.pc = 0x0fc6;
        break;
    case 0x0fc6: /* MOVG3 r6 -> @r1+0xa46c (word) */
        st16(ea_r(1, 0xa46c), mcu.r[6]);
        mcu.pc = 0x0fca;
        break;
    case 0x0fca: /* SHLR r1 word */
        shlr16(mcu.r[1]);
        mcu.pc = 0x0fcc;
        break;
    case 0x0fcc: /* MOVG2 (dp,0xa1d2) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1d2));
        mcu.pc = 0x0fd0;
        break;
    case 0x0fd0: /* MOVG3 r0 -> @r1+0xce94 (byte) */
        st8(ea_r(1, 0xce94), mcu.r[0]);
        mcu.pc = 0x0fd4;
        break;
    case 0x0fd4: /* MOVG2 (dp,0xa1d3) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1d3));
        mcu.pc = 0x0fd8;
        break;
    case 0x0fd8: /* MOVG3 r0 -> @r1+0xcf3c (byte) */
        st8(ea_r(1, 0xcf3c), mcu.r[0]);
        mcu.pc = 0x0fdc;
        break;
    case 0x0fdc: /* MOVG2 (dp,0xa1d4) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1d4));
        mcu.pc = 0x0fe0;
        break;
    case 0x0fe0: /* MOVG3 r0 -> @r1+0xd134 (byte) */
        st8(ea_r(1, 0xd134), mcu.r[0]);
        mcu.pc = 0x0fe4;
        break;
    case 0x0fe4: /* BTSTI (dp,0xa1df) #7 */
        btst8_mem(ea_dp(0xa1df), 7);
        mcu.pc = 0x0fe8;
        break;
    case 0x0fe8: /* BEQ 0x0fee */
        mcu.pc = f_z() ? 0x0fee : 0x0fea;
        break;
    case 0x0fea: /* BSET (dp,0xa1d1) #7 */
        bset8_mem(ea_dp(0xa1d1), 7);
        mcu.pc = 0x0fee;
        break;
    case 0x0fee: /* MOVG2 (dp,0xa1d1) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1d1));
        mcu.pc = 0x0ff2;
        break;
    case 0x0ff2: /* MOVG3 r0 -> @r1+0xcf90 (byte) */
        st8(ea_r(1, 0xcf90), mcu.r[0]);
        mcu.pc = 0x0ff6;
        break;
    case 0x0ff6: /* MOVG2 (dp,0xa4a7) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa4a7));
        mcu.pc = 0x0ffa;
        break;
    case 0x0ffa: /* MOVG3 r0 -> @r1+0xceb0 (byte) */
        st8(ea_r(1, 0xceb0), mcu.r[0]);
        mcu.pc = 0x0ffe;
        break;
    case 0x0ffe: /* MOVG2 (dp,0xa1d6) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1d6));
        mcu.pc = 0x1002;
        break;
    case 0x1002: /* MOVG3 r0 -> @r1+0xd0fc (byte) */
        st8(ea_r(1, 0xd0fc), mcu.r[0]);
        mcu.pc = 0x1006;
        break;
    case 0x1006: /* MOVG2 (dp,0xa1d7) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1d7));
        mcu.pc = 0x100a;
        break;
    case 0x1007: /* BCLR r1 #7 (overlap decode) */
        bclr8_reg(mcu.r[1], 7);
        mcu.pc = 0x1009;
        break;
    case 0x100a: /* MOVG3 r0 -> @r1+0xcecc (byte) */
        st8(ea_r(1, 0xcecc), mcu.r[0]);
        mcu.pc = 0x100e;
        break;
    case 0x100c: /* MOVG3 r0 -> r4++ (word, overlap decode) */
    {
        uint32_t oea = (uint32_t)mcu.r[4];
        mcu.r[4] = (uint16_t)(mcu.r[4] + 2);
        st16(((uint32_t)mcu.ep << 16) | (uint16_t)oea, mcu.r[0]);
        mcu.pc = 0x100e;
        break;
    }
    case 0x100e: /* MOVG2 (dp,0xa1f4) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1f4));
        mcu.pc = 0x1012;
        break;
    case 0x100f: /* BTSTI r1 #4 (overlap decode) */
        btst8_reg(mcu.r[1], 4);
        mcu.pc = 0x1011;
        break;
    case 0x1012: /* MOVG3 r0 -> @r1+0xcfe4 (byte) */
        st8(ea_r(1, 0xcfe4), mcu.r[0]);
        mcu.pc = 0x1016;
        break;
    case 0x1016: /* MOVG2 (dp,0xa1f4) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1f4));
        mcu.pc = 0x101a;
        break;
    case 0x101a: /* MOVG3 r0 -> @r1+0xd038 (byte) */
        st8(ea_r(1, 0xd038), mcu.r[0]);
        mcu.pc = 0x101e;
        break;
    case 0x101e: /* MOVG2 (dp,0xa1f5) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1f5));
        mcu.pc = 0x1022;
        break;
    case 0x1022: /* MOVG3 r0 -> @r1+0xd08c (byte) */
        st8(ea_r(1, 0xd08c), mcu.r[0]);
        mcu.pc = 0x1026;
        break;
    case 0x1026: /* MOVG #0x02 -> @r1+0xd0e0 */
        mov_imm8_mem(ea_r(1, 0xd0e0), 0x02);
        mcu.pc = 0x102b;
        break;
    case 0x102b: /* CLR @r1+0xa4b4 (byte) */
        clr8_mem(ea_r(1, 0xa4b4));
        mcu.pc = 0x102f;
        break;
    case 0x102f: /* CLR @r1+0xacf2 (byte) */
        clr8_mem(ea_r(1, 0xacf2));
        mcu.pc = 0x1033;
        break;
    case 0x1033: /* rts */
        ret();
        break;
    case 0x1034: /* MOVG2 (dp,0xa1e2) r2 (word) */
        ld16(mcu.r[2], ea_dp(0xa1e2));
        mcu.pc = 0x1038;
        break;
    case 0x1038: /* MOVG2 (dp,0xa1d2) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa1d2));
        mcu.pc = 0x103c;
        break;
    case 0x103c: /* MOVG3 r1 -> (dp,0xa1d5) (byte) */
        st8(ea_dp(0xa1d5), mcu.r[1]);
        mcu.pc = 0x1040;
        break;
    case 0x1040: /* bsr16 -> 0x1af2 */
        call(0x1043, 0x1af2);
        break;
    case 0x1043: /* bsr16 -> 0x1b09 */
        call(0x1046, 0x1b09);
        break;
    case 0x1046: /* MOVG3 r0 -> (dp,0xa1d3) (byte) */
        st8(ea_dp(0xa1d3), mcu.r[0]);
        mcu.pc = 0x104a;
        break;
    case 0x104a: /* bsr -> 0x107d */
        call(0x104c, 0x107d);
        break;
    case 0x104c: /* rts */
        ret();
        break;
    case 0x104d: /* MOVG2 (dp,0xa1e2) r2 (word) */
        ld16(mcu.r[2], ea_dp(0xa1e2));
        mcu.pc = 0x1051;
        break;
    case 0x1051: /* MOVG2 (dp,0xa1d2) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa1d2));
        mcu.pc = 0x1055;
        break;
    case 0x1055: /* MOVG3 r1 -> (dp,0xa1d5) (byte) */
        st8(ea_dp(0xa1d5), mcu.r[1]);
        mcu.pc = 0x1059;
        break;
    case 0x1056: /* BCLR r1 #5 (overlap decode; next byte not a decoded PC) */
        bclr8_reg(mcu.r[1], 5);
        mcu.pc = 0x1058;
        break;
    case 0x1059: /* bsr16 -> 0x1af2 */
        call(0x105c, 0x1af2);
        break;
    case 0x105c: /* MOVG3 r1 -> (dp,0xa1d3) (byte) */
        st8(ea_dp(0xa1d3), mcu.r[1]);
        mcu.pc = 0x1060;
        break;
    case 0x1060: /* bsr -> 0x107d */
        call(0x1062, 0x107d);
        break;
    case 0x1062: /* rts */
        ret();
        break;
    case 0x1063: /* MOVG2 (dp,0xa1e2) r2 (word) */
        ld16(mcu.r[2], ea_dp(0xa1e2));
        mcu.pc = 0x1067;
        break;
    case 0x1067: /* MOVG2 (dp,0xa1e8) r1 (word) */
        ld16(mcu.r[1], ea_dp(0xa1e8));
        mcu.pc = 0x106b;
        break;
    case 0x106b: /* MOVG2 @r1+0x0180 r1 (byte) */
        ld8(mcu.r[1], ea_r(1, 0x0180));
        mcu.pc = 0x106f;
        break;
    case 0x106f: /* bsr16 -> 0x1af2 */
        call(0x1072, 0x1af2);
        break;
    case 0x1072: /* MOVG3 r0 -> (dp,0xa1d5) (byte) */
        st8(ea_dp(0xa1d5), mcu.r[0]);
        mcu.pc = 0x1076;
        break;
    case 0x1076: /* MOVG3 r0 -> (dp,0xa1d3) (byte) */
        st8(ea_dp(0xa1d3), mcu.r[0]);
        mcu.pc = 0x107a;
        break;
    case 0x107a: /* bsr -> 0x107d */
        call(0x107c, 0x107d);
        break;
    case 0x107c: /* rts */
        ret();
        break;
    case 0x107d: /* MOVG3 r0 -> --r7 (word) */
    {
        mcu.r[7] = (uint16_t)(mcu.r[7] - 2);
        st16(((uint32_t)mcu.tp << 16) | mcu.r[7], mcu.r[0]);
        mcu.pc = 0x107f;
        break;
    }
    case 0x107f: /* MOVG2 (dp,0xa4a9) r3 (byte) */
        ld8(mcu.r[3], ea_dp(0xa4a9));
        mcu.pc = 0x1083;
        break;
    case 0x1083: /* MOVG2 (dp,0xa1d0) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa1d0));
        mcu.pc = 0x1087;
        break;
    case 0x1087: /* MOVG3 r1 -> @r3+0xa34c (byte) */
        st8(ea_r(3, 0xa34c), mcu.r[1]);
        mcu.pc = 0x108b;
        break;
    case 0x108b: /* MOVG2 (dp,0xa4a4) r3 (byte) */
        ld8(mcu.r[3], ea_dp(0xa4a4));
        mcu.pc = 0x108f;
        break;
    case 0x108f: /* BTSTI (dp,0xa1d0) #0 */
        btst8_mem(ea_dp(0xa1d0), 0);
        mcu.pc = 0x1093;
        break;
    case 0x1093: /* BEQ 0x1109 */
        mcu.pc = f_z() ? 0x1109 : 0x1095;
        break;
    case 0x1095: /* MOVG2 (dp,0xa1e4) r5 (word) */
        ld16(mcu.r[5], ea_dp(0xa1e4));
        mcu.pc = 0x1099;
        break;
    case 0x1099: /* ADDS #0x20 r5 */
        adds8(mcu.r[5], 0x20);
        mcu.pc = 0x109c;
        break;
    case 0x109c: /* MOVG3 r5 -> (dp,0xa1e6) (word) */
        st16(ea_dp(0xa1e6), mcu.r[5]);
        mcu.pc = 0x10a0;
        break;
    case 0x10a0: /* SUB @r5+2 #0xffff */
        sub16_mem_imm16(ea_r(5, 0x0002), 0xffff);
        mcu.pc = 0x10a5;
        break;
    case 0x10a5: /* BEQ 0x1109 */
        mcu.pc = f_z() ? 0x1109 : 0x10a7;
        break;
    case 0x10a7: /* bsr16 -> 0x1190 */
        call(0x10aa, 0x1190);
        break;
    case 0x10a8: /* nop (overlap decode; next byte not a decoded PC) */
        mcu.pc = 0x10a9;
        break;
    case 0x10aa: /* MOVG2 (dp,0xa1ee) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa1ee));
        mcu.pc = 0x10ae;
        break;
    case 0x10ae: /* bsr16 -> 0x11b8 */
        call(0x10b1, 0x11b8);
        break;
    case 0x10b1: /* MOVG2 (dp,0xa4a4) r3 (byte) */
        ld8(mcu.r[3], ea_dp(0xa4a4));
        mcu.pc = 0x10b5;
        break;
    case 0x10b5: /* SHLL r3 word */
        shll16(mcu.r[3]);
        mcu.pc = 0x10b7;
        break;
    case 0x10b7: /* MOVG2 (dp,0xa1d3) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa1d3));
        mcu.pc = 0x10bb;
        break;
    case 0x10bb: /* MOVG3 r1 -> @r3+0xa190 (byte) */
        st8(ea_r(3, 0xa190), mcu.r[1]);
        mcu.pc = 0x10bf;
        break;
    case 0x10bf: /* SHLR r3 word */
        shlr16(mcu.r[3]);
        mcu.pc = 0x10c1;
        break;
    case 0x10c1: /* CLR r1 word */
        clr16(mcu.r[1]);
        mcu.pc = 0x10c3;
        break;
    case 0x10c3: /* MOVG2 (dp,0xa4aa) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa4aa));
        mcu.pc = 0x10c7;
        break;
    case 0x10c7: /* BMI 0x1109 */
        mcu.pc = f_n() ? 0x1109 : 0x10c9;
        break;
    case 0x10c9: /* BTSTI (dp,0xa1d1) #7 */
        btst8_mem(ea_dp(0xa1d1), 7);
        mcu.pc = 0x10cd;
        break;
    case 0x10cd: /* BEQ 0x10e2 */
        mcu.pc = f_z() ? 0x10e2 : 0x10cf;
        break;
    case 0x10cf: /* BSET_ORC #0x0700 r0 (IML=7) */
        orc_imm16(0x0700);
        mcu.pc = 0x10d3;
        break;
    case 0x10d3: /* stm #0x3e */
        stm_rlist(0x3e);
        mcu.pc = 0x10d5;
        break;
    case 0x10d5: /* jsr #0x516c pcm_start (C1) */
        call(0x10d8, 0x516c);
        break;
    case 0x10d8: /* ldm #0x3e */
        ldm_rlist(0x3e);
        mcu.pc = 0x10da;
        break;
    case 0x10da: /* BCLR_ANDC #0xf8ff r0 (IML=0) */
        andc_imm16(0xf8ff);
        mcu.pc = 0x10de;
        break;
    case 0x10de: /* BSET (dp,0xa1df) #7 */
        bset8_mem(ea_dp(0xa1df), 7);
        mcu.pc = 0x10e2;
        break;
    case 0x10e2: /* SHLL r1 byte */
        shll8(mcu.r[1]);
        mcu.pc = 0x10e4;
        break;
    case 0x10e4: /* MOVG2 (dp,0xa1ec) r0 (word) */
        ld16(mcu.r[0], ea_dp(0xa1ec));
        mcu.pc = 0x10e8;
        break;
    case 0x10e8: /* MOVG3 r0 -> @r1+0xcf58 (word) */
        st16(ea_r(1, 0xcf58), mcu.r[0]);
        mcu.pc = 0x10ec;
        break;
    case 0x10ec: /* SHLR r1 byte */
        shlr8(mcu.r[1]);
        mcu.pc = 0x10ee;
        break;
    case 0x10ee: /* MOVG2 (dp,0xa1da) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1da));
        mcu.pc = 0x10f2;
        break;
    case 0x10f2: /* MOVG3 r0 -> @r1+0xcee8 (byte) */
        st8(ea_r(1, 0xcee8), mcu.r[0]);
        mcu.pc = 0x10f6;
        break;
    case 0x10f6: /* MOVG2 (dp,0xa1dd) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1dd));
        mcu.pc = 0x10fa;
        break;
    case 0x10fa: /* MOVG3 r0 -> @r1+0xcf04 (byte) */
        st8(ea_r(1, 0xcf04), mcu.r[0]);
        mcu.pc = 0x10fe;
        break;
    case 0x10fe: /* MOVG2 (dp,0xa1ee) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1ee));
        mcu.pc = 0x1102;
        break;
    case 0x1102: /* MOVG3 r0 -> @r1+0xcf20 (byte) */
        st8(ea_r(1, 0xcf20), mcu.r[0]);
        mcu.pc = 0x1106;
        break;
    case 0x1106: /* bsr16 -> 0x0f86 (self recursion) */
        call(0x1109, 0x0f86);
        break;
    case 0x1109: /* MOVG2 r7++ r0 (word) */
    {
        uint32_t oea = (uint32_t)mcu.r[7];
        mcu.r[7] = (uint16_t)(mcu.r[7] + 2);
        ld16(mcu.r[0], ((uint32_t)mcu.tp << 16) | (uint16_t)oea);
        mcu.pc = 0x110b;
        break;
    }
    case 0x110b: /* BTSTI (dp,0xa1d0) #1 */
        btst8_mem(ea_dp(0xa1d0), 1);
        mcu.pc = 0x110f;
        break;
    case 0x110f: /* BEQ 0x118b (fallthrough 0x1111, not the overlap 0x1110) */
        mcu.pc = f_z() ? 0x118b : 0x1111;
        break;
    case 0x1110: /* movsw r2 @(br,$1d) (overlap decode) */
        movsw16_r2(0x1d);
        mcu.pc = 0x1112;
        break;
    case 0x1111: /* MOVG2 (dp,0xa1e4) r5 (word) */
        ld16(mcu.r[5], ea_dp(0xa1e4));
        mcu.pc = 0x1115;
        break;
    case 0x1115: /* ADDS #0x7c r5 */
        adds8(mcu.r[5], 0x7c);
        mcu.pc = 0x1118;
        break;
    case 0x1118: /* MOVG3 r5 -> (dp,0xa1e6) (word) */
        st16(ea_dp(0xa1e6), mcu.r[5]);
        mcu.pc = 0x111c;
        break;
    case 0x111c: /* SUB @r5+2 #0xffff */
        sub16_mem_imm16(ea_r(5, 0x0002), 0xffff);
        mcu.pc = 0x1121;
        break;
    case 0x1121: /* BEQ 0x118b */
        mcu.pc = f_z() ? 0x118b : 0x1123;
        break;
    case 0x1123: /* bsr16 -> 0x1190 */
        call(0x1126, 0x1190);
        break;
    case 0x1126: /* MOVG2 (dp,0xa1ef) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa1ef));
        mcu.pc = 0x112a;
        break;
    case 0x112a: /* bsr16 -> 0x11b8 */
        call(0x112d, 0x11b8);
        break;
    case 0x112d: /* MOVG2 (dp,0xa4a4) r3 (byte) */
        ld8(mcu.r[3], ea_dp(0xa4a4));
        mcu.pc = 0x1131;
        break;
    case 0x1131: /* SHLL r3 word */
        shll16(mcu.r[3]);
        mcu.pc = 0x1133;
        break;
    case 0x1133: /* MOVG2 (dp,0xa1d3) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa1d3));
        mcu.pc = 0x1137;
        break;
    case 0x1137: /* MOVG3 r1 -> @r3+0xa191 (byte) */
        st8(ea_r(3, 0xa191), mcu.r[1]);
        mcu.pc = 0x113b;
        break;
    case 0x113b: /* SHLR r3 word */
        shlr16(mcu.r[3]);
        mcu.pc = 0x113d;
        break;
    case 0x113d: /* CLR r1 word */
        clr16(mcu.r[1]);
        mcu.pc = 0x113f;
        break;
    case 0x113f: /* MOVG2 (dp,0xa4ab) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa4ab));
        mcu.pc = 0x1143;
        break;
    case 0x1143: /* BPL 0x114b */
        mcu.pc = f_n() ? 0x1145 : 0x114b;
        break;
    case 0x1145: /* MOVG2 (dp,0xa4aa) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa4aa));
        mcu.pc = 0x1149;
        break;
    case 0x1149: /* BMI 0x118b */
        mcu.pc = f_n() ? 0x118b : 0x114b;
        break;
    case 0x114b: /* BTSTI (dp,0xa1d1) #7 */
        btst8_mem(ea_dp(0xa1d1), 7);
        mcu.pc = 0x114f;
        break;
    case 0x114f: /* BEQ 0x1164 */
        mcu.pc = f_z() ? 0x1164 : 0x1151;
        break;
    case 0x1151: /* BSET_ORC #0x0700 r0 */
        orc_imm16(0x0700);
        mcu.pc = 0x1155;
        break;
    case 0x1155: /* stm #0x3e */
        stm_rlist(0x3e);
        mcu.pc = 0x1157;
        break;
    case 0x1157: /* jsr #0x516c pcm_start (C1) */
        call(0x115a, 0x516c);
        break;
    case 0x115a: /* ldm #0x3e */
        ldm_rlist(0x3e);
        mcu.pc = 0x115c;
        break;
    case 0x115c: /* BCLR_ANDC #0xf8ff r0 */
        andc_imm16(0xf8ff);
        mcu.pc = 0x1160;
        break;
    case 0x1160: /* BSET (dp,0xa1df) #7 */
        bset8_mem(ea_dp(0xa1df), 7);
        mcu.pc = 0x1164;
        break;
    case 0x1164: /* SHLL r1 byte */
        shll8(mcu.r[1]);
        mcu.pc = 0x1166;
        break;
    case 0x1166: /* MOVG2 (dp,0xa1ec) r0 (word) */
        ld16(mcu.r[0], ea_dp(0xa1ec));
        mcu.pc = 0x116a;
        break;
    case 0x116a: /* MOVG3 r0 -> @r1+0xcf58 (word) */
        st16(ea_r(1, 0xcf58), mcu.r[0]);
        mcu.pc = 0x116e;
        break;
    case 0x116e: /* SHLR r1 byte */
        shlr8(mcu.r[1]);
        mcu.pc = 0x1170;
        break;
    case 0x1170: /* MOVG2 (dp,0xa1db) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1db));
        mcu.pc = 0x1174;
        break;
    case 0x1174: /* MOVG3 r0 -> @r1+0xcee8 (byte) */
        st8(ea_r(1, 0xcee8), mcu.r[0]);
        mcu.pc = 0x1178;
        break;
    case 0x1178: /* MOVG2 (dp,0xa1de) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1de));
        mcu.pc = 0x117c;
        break;
    case 0x117c: /* MOVG3 r0 -> @r1+0xcf04 (byte) */
        st8(ea_r(1, 0xcf04), mcu.r[0]);
        mcu.pc = 0x1180;
        break;
    case 0x1180: /* MOVG2 (dp,0xa1ef) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa1ef));
        mcu.pc = 0x1184;
        break;
    case 0x1184: /* MOVG3 r0 -> @r1+0xcf20 (byte) */
        st8(ea_r(1, 0xcf20), mcu.r[0]);
        mcu.pc = 0x1188;
        break;
    case 0x1188: /* bsr16 -> 0x0f86 (self recursion) */
        call(0x118b, 0x0f86);
        break;
    case 0x118b: /* BCLR (dp,0xa1df) #7 */
        bclr8_mem(ea_dp(0xa1df), 7);
        mcu.pc = 0x118f;
        break;
    case 0x118f: /* rts */
        ret();
        break;
    default:
        /* Not one of ours: execute the stock instruction once (defensive;
         * registration below covers every PC of the decoded range). */
        MCU_ReadInstruction();
        break;
    }
    return 1;
}

/* ======================================================================== */
/* B2 scan_a 0x13a9..0x13c9                                                 */
/* ======================================================================== */

uint32_t scan_a_step(void)
{
    switch (mcu.pc)
    {
    case 0x13a9: /* BMI 0x13c9 */
        mcu.pc = f_n() ? 0x13c9 : 0x13ab;
        break;
    case 0x13ab: /* SUB @r2+0xa288 #0x00 (a288 == 0?) */
        sub8_mem_imm8(ea_r(2, 0xa288), 0x00);
        mcu.pc = 0x13b0;
        break;
    case 0x13b0: /* BNE 0x13c3 */
        mcu.pc = f_z() ? 0x13b2 : 0x13c3;
        break;
    case 0x13b2: /* CMP @r2+0xa314 r1 */
        cmp8_mem_reg(ea_r(2, 0xa314), mcu.r[1]);
        mcu.pc = 0x13b6;
        break;
    case 0x13b6: /* BNE 0x13c3 */
        mcu.pc = f_z() ? 0x13b8 : 0x13c3;
        break;
    case 0x13b8: /* BTSTI @r2+0xa330 #0 */
        btst8_mem(ea_r(2, 0xa330), 0);
        mcu.pc = 0x13bc;
        break;
    case 0x13bc: /* BEQ 0x13c3 */
        mcu.pc = f_z() ? 0x13c3 : 0x13be;
        break;
    case 0x13be: /* bsr16 -> 0x1459 kill_a (B3) */
        call(0x13c1, 0x1459);
        break;
    case 0x13c1: /* BRA 0x13c9 */
        mcu.pc = 0x13c9;
        break;
    case 0x13c3: /* MOVG2 @r2+0xa250 r2 (byte) */
        ld8(mcu.r[2], ea_r(2, 0xa250));
        mcu.pc = 0x13c7;
        break;
    case 0x13c7: /* BPL 0x13ab */
        mcu.pc = f_n() ? 0x13c9 : 0x13ab;
        break;
    case 0x13c9: /* rts */
        ret();
        break;
    default:
        MCU_ReadInstruction();
        break;
    }
    return 1;
}

/* ======================================================================== */
/* B3 kill_a 0x1459..0x14a8                                                 */
/* ======================================================================== */

uint32_t kill_a_step(void)
{
    switch (mcu.pc)
    {
    case 0x1459: /* MOVG #0x02 -> @r2+0xa288 */
        mov_imm8_mem(ea_r(2, 0xa288), 0x02);
        mcu.pc = 0x145e;
        break;
    case 0x145e: /* BTSTI @r3+0xa240 #0 */
        btst8_mem(ea_r(3, 0xa240), 0);
        mcu.pc = 0x1462;
        break;
    case 0x1462: /* BEQ 0x146a */
        mcu.pc = f_z() ? 0x146a : 0x1464;
        break;
    case 0x1464: /* BSET @r2+0xa2a4 #0 */
        bset8_mem(ea_r(2, 0xa2a4), 0);
        mcu.pc = 0x1468;
        break;
    case 0x1468: /* BRA 0x14a6 */
        mcu.pc = 0x14a6;
        break;
    case 0x146a: /* movi r6 #0x000f */
        movi16(mcu.r[6], 0x000f);
        mcu.pc = 0x146d;
        break;
    case 0x146d: /* MOVG2 r3 r0 (word) */
        move16_reg(mcu.r[0], mcu.r[3]);
        mcu.pc = 0x146f;
        break;
    case 0x146f: /* SHLL r0 x4 (a090 + 16*part) */
        shll8(mcu.r[0]);
        mcu.pc = 0x1471;
        break;
    case 0x1471: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x1473;
        break;
    case 0x1473: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x1475;
        break;
    case 0x1475: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x1477;
        break;
    case 0x1477: /* ADD #0xa090 r0 */
        add_imm16_r0(0xa090);
        mcu.pc = 0x147b;
        break;
    case 0x1479: /* movf r0 -> @r6+32 (overlap decode, word store) */
        movf_store16_r0(32);
        mcu.pc = 0x147b;
        break;
    case 0x147b: /* TST @r0 (byte) */
        tst8_mem(ea_r(0, 0x0000));
        mcu.pc = 0x147d;
        break;
    case 0x147d: /* BMI 0x1486 */
        mcu.pc = f_n() ? 0x1486 : 0x147f;
        break;
    case 0x147f: /* CMP r0++ r1 */
        cmp8_postinc_r0_r1();
        mcu.pc = 0x1481;
        break;
    case 0x1481: /* BEQ 0x14a6 */
        mcu.pc = f_z() ? 0x14a6 : 0x1483;
        break;
    case 0x1483: /* cntjmp r6 -11 -> 0x147b */
        cntjmp_reg(mcu.r[6], 0x1486, 0x147b);
        break;
    case 0x1486: /* CLR r1 word */
        clr16(mcu.r[1]);
        mcu.pc = 0x1488;
        break;
    case 0x1488: /* MOVG2 @r2+0xa2dc r1 (byte) */
        ld8(mcu.r[1], ea_r(2, 0xa2dc));
        mcu.pc = 0x148c;
        break;
    case 0x148c: /* MOVG #0x01 -> @r1+0xa3bc */
        mov_imm8_mem(ea_r(1, 0xa3bc), 0x01);
        mcu.pc = 0x1491;
        break;
    case 0x1491: /* MOVG #0xff -> @r1+0xa4b4 */
        mov_imm8_mem(ea_r(1, 0xa4b4), 0xff);
        mcu.pc = 0x1496;
        break;
    case 0x1496: /* MOVG2 @r1+0xa410 r1 (byte) */
        ld8(mcu.r[1], ea_r(1, 0xa410));
        mcu.pc = 0x149a;
        break;
    case 0x149a: /* BMI 0x14a6 */
        mcu.pc = f_n() ? 0x14a6 : 0x149c;
        break;
    case 0x149c: /* MOVG #0x01 -> @r1+0xa3bc (chain propagate) */
        mov_imm8_mem(ea_r(1, 0xa3bc), 0x01);
        mcu.pc = 0x14a1;
        break;
    case 0x14a1: /* MOVG #0xff -> @r1+0xa4b4 */
        mov_imm8_mem(ea_r(1, 0xa4b4), 0xff);
        mcu.pc = 0x14a6;
        break;
    case 0x14a6: /* EXTU r0 */
        extu16(mcu.r[0]);
        mcu.pc = 0x14a8;
        break;
    case 0x14a8: /* rts */
        ret();
        break;
    default:
        MCU_ReadInstruction();
        break;
    }
    return 1;
}

/* ======================================================================== */
/* B4 scan_b 0x151e..0x157c                                                 */
/* ======================================================================== */

uint32_t scan_b_step(void)
{
    switch (mcu.pc)
    {
    case 0x151e: /* CLR r1 word */
        clr16(mcu.r[1]);
        mcu.pc = 0x1520;
        break;
    case 0x1520: /* EXTU r3 */
        extu16(mcu.r[3]);
        mcu.pc = 0x1522;
        break;
    case 0x1522: /* BCLR @r3+0xa240 #0 */
        bclr8_mem(ea_r(3, 0xa240), 0);
        mcu.pc = 0x1526;
        break;
    case 0x1526: /* BEQ 0x157c */
        mcu.pc = f_z() ? 0x157c : 0x1528;
        break;
    case 0x1528: /* CLR r2 word */
        clr16(mcu.r[2]);
        mcu.pc = 0x152a;
        break;
    case 0x152a: /* MOVG2 @r3+0xa220 r2 (byte) */
        ld8(mcu.r[2], ea_r(3, 0xa220));
        mcu.pc = 0x152e;
        break;
    case 0x152e: /* BMI 0x157c */
        mcu.pc = f_n() ? 0x157c : 0x1530;
        break;
    case 0x1530: /* BCLR @r2+0xa2a4 #0 */
        bclr8_mem(ea_r(2, 0xa2a4), 0);
        mcu.pc = 0x1534;
        break;
    case 0x1534: /* BEQ 0x1576 */
        mcu.pc = f_z() ? 0x1576 : 0x1536;
        break;
    case 0x1536: /* MOVG2 @r2+0xa314 r1 (byte) */
        ld8(mcu.r[1], ea_r(2, 0xa314));
        mcu.pc = 0x153a;
        break;
    case 0x153a: /* movi r6 #0x000f */
        movi16(mcu.r[6], 0x000f);
        mcu.pc = 0x153d;
        break;
    case 0x153d: /* MOVG2 r3 r0 (word) */
        move16_reg(mcu.r[0], mcu.r[3]);
        mcu.pc = 0x153f;
        break;
    case 0x153f: /* SHLL r0 (x4, a090 + 16*part) */
        shll8(mcu.r[0]);
        mcu.pc = 0x1541;
        break;
    case 0x1541: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x1543;
        break;
    case 0x1543: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x1545;
        break;
    case 0x1545: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x1547;
        break;
    case 0x1547: /* ADD #0xa090 r0 */
        add_imm16_r0(0xa090);
        mcu.pc = 0x154b;
        break;
    case 0x154b: /* TST @r0 (byte) */
        tst8_mem(ea_r(0, 0x0000));
        mcu.pc = 0x154d;
        break;
    case 0x154d: /* BMI 0x1556 */
        mcu.pc = f_n() ? 0x1556 : 0x154f;
        break;
    case 0x154f: /* CMP r0++ r1 */
        cmp8_postinc_r0_r1();
        mcu.pc = 0x1551;
        break;
    case 0x1551: /* BEQ 0x1576 */
        mcu.pc = f_z() ? 0x1576 : 0x1553;
        break;
    case 0x1553: /* cntjmp r6 -11 -> 0x154b */
        cntjmp_reg(mcu.r[6], 0x1556, 0x154b);
        break;
    case 0x1556: /* MOVG2 @r2+0xa2dc r1 (byte) */
        ld8(mcu.r[1], ea_r(2, 0xa2dc));
        mcu.pc = 0x155a;
        break;
    case 0x155a: /* EXTU r1 */
        extu16(mcu.r[1]);
        mcu.pc = 0x155c;
        break;
    case 0x155c: /* MOVG #0x01 -> @r1+0xa3bc */
        mov_imm8_mem(ea_r(1, 0xa3bc), 0x01);
        mcu.pc = 0x1561;
        break;
    case 0x1561: /* MOVG #0xff -> @r1+0xa4b4 */
        mov_imm8_mem(ea_r(1, 0xa4b4), 0xff);
        mcu.pc = 0x1566;
        break;
    case 0x1566: /* MOVG2 @r1+0xa410 r1 (byte) */
        ld8(mcu.r[1], ea_r(1, 0xa410));
        mcu.pc = 0x156a;
        break;
    case 0x156a: /* BMI 0x1576 */
        mcu.pc = f_n() ? 0x1576 : 0x156c;
        break;
    case 0x156c: /* MOVG #0x01 -> @r1+0xa3bc (chain propagate) */
        mov_imm8_mem(ea_r(1, 0xa3bc), 0x01);
        mcu.pc = 0x1571;
        break;
    case 0x1571: /* MOVG #0xff -> @r1+0xa4b4 */
        mov_imm8_mem(ea_r(1, 0xa4b4), 0xff);
        mcu.pc = 0x1576;
        break;
    case 0x1576: /* MOVG2 @r2+0xa250 r2 (byte) */
        ld8(mcu.r[2], ea_r(2, 0xa250));
        mcu.pc = 0x157a;
        break;
    case 0x157a: /* BPL 0x1530 */
        mcu.pc = f_n() ? 0x157c : 0x1530;
        break;
    case 0x157c: /* rts */
        ret();
        break;
    default:
        MCU_ReadInstruction();
        break;
    }
    return 1;
}

/* ======================================================================== */
/* B5 alloc_scan 0x157d..0x15d0                                             */
/* ======================================================================== */

uint32_t alloc_scan_step(void)
{
    switch (mcu.pc)
    {
    case 0x157d: /* CLR (dp,0xa42c) */
        clr8_mem(ea_dp(0xa42c));
        mcu.pc = 0x1581;
        break;
    case 0x1581: /* CLR r0 word */
        clr16(mcu.r[0]);
        mcu.pc = 0x1583;
        break;
    case 0x1583: /* MOVG2 (dp,0xa4a8) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa4a8));
        mcu.pc = 0x1587;
        break;
    case 0x1587: /* SUB (dp,0xa42d) r0: shortfall = a4a8 - a42d */
        sub8_r0_mem(ea_dp(0xa42d));
        mcu.pc = 0x158b;
        break;
    case 0x158b: /* BLE 0x15cc */
        mcu.pc = (f_z() || (f_n() != f_v())) ? 0x15cc : 0x158d;
        break;
    case 0x158d: /* MOVG3 r0 -> (dp,0xa42c) (shortfall) */
        st8(ea_dp(0xa42c), mcu.r[0]);
        mcu.pc = 0x1591;
        break;
    case 0x1591: /* MOVG2 (dp,0xa4a4) r3 (byte) */
        ld8(mcu.r[3], ea_dp(0xa4a4));
        mcu.pc = 0x1595;
        break;
    case 0x1595: /* TST (dp,0x8028) (byte) */
        tst8_mem(ea_dp(0x8028));
        mcu.pc = 0x1599;
        break;
    case 0x1599: /* BEQ 0x15a1 */
        mcu.pc = f_z() ? 0x15a1 : 0x159b;
        break;
    case 0x159b: /* CMP (dp,0x8028) r3 */
        cmp8_mem_reg(ea_dp(0x8028), mcu.r[3]);
        mcu.pc = 0x159f;
        break;
    case 0x159f: /* BLS 0x15a5 */
        mcu.pc = (f_c() || f_z()) ? 0x15a5 : 0x15a1;
        break;
    case 0x15a1: /* move r3 #0x0f */
        move8_imm(mcu.r[3], 0x0f);
        mcu.pc = 0x15a3;
        break;
    case 0x15a3: /* BRA 0x15a9 */
        mcu.pc = 0x15a9;
        break;
    case 0x15a5: /* MOVG2 (dp,0x8028) r3 (byte) */
        ld8(mcu.r[3], ea_dp(0x8028));
        mcu.pc = 0x15a9;
        break;
    case 0x15a9: /* MOVG2 @r3+0xa210 r0 (byte) */
        ld8(mcu.r[0], ea_r(3, 0xa210));
        mcu.pc = 0x15ad;
        break;
    case 0x15ad: /* SUB @r3+0x8018 r0 */
        sub8_r0_mem(ea_r(3, 0x8018));
        mcu.pc = 0x15b1;
        break;
    case 0x15b1: /* BLE 0x15b7 */
        mcu.pc = (f_z() || (f_n() != f_v())) ? 0x15b7 : 0x15b3;
        break;
    case 0x15b3: /* bsr -> 0x15d1 note_disp (B6) */
        call(0x15b5, 0x15d1);
        break;
    case 0x15b5: /* BLE 0x15cc */
        mcu.pc = (f_z() || (f_n() != f_v())) ? 0x15cc : 0x15b7;
        break;
    case 0x15b7: /* cntjmp r3 -17 -> 0x15a9 */
        cntjmp_reg(mcu.r[3], 0x15ba, 0x15a9);
        break;
    case 0x15ba: /* MOVG2 (dp,0xa4a4) r3 (byte) */
        ld8(mcu.r[3], ea_dp(0xa4a4));
        mcu.pc = 0x15be;
        break;
    case 0x15be: /* MOVG2 (dp,0xa42c) r0 (byte) */
        ld8(mcu.r[0], ea_dp(0xa42c));
        mcu.pc = 0x15c2;
        break;
    case 0x15c2: /* BLS 0x15cc */
        mcu.pc = (f_c() || f_z()) ? 0x15cc : 0x15c4;
        break;
    case 0x15c4: /* CMP @r3+0xa210 r0 */
        cmp8_mem_reg(ea_r(3, 0xa210), mcu.r[0]);
        mcu.pc = 0x15c8;
        break;
    case 0x15c8: /* BGT 0x15cc */
        mcu.pc = !(f_z() || (f_n() != f_v())) ? 0x15cc : 0x15ca;
        break;
    case 0x15ca: /* bsr -> 0x15d1 note_disp (B6) */
        call(0x15cc, 0x15d1);
        break;
    case 0x15cc: /* TST (dp,0xa42c) (byte) */
        tst8_mem(ea_dp(0xa42c));
        mcu.pc = 0x15d0;
        break;
    case 0x15d0: /* rts */
        ret();
        break;
    default:
        MCU_ReadInstruction();
        break;
    }
    return 1;
}

/* ======================================================================== */
/* B6 note_disp 0x15d1..0x15fa                                              */
/* ======================================================================== */

uint32_t note_disp_step(void)
{
    switch (mcu.pc)
    {
    case 0x15d1: /* MOVG2 r3 r6 (word) */
        move16_reg(mcu.r[6], mcu.r[3]);
        mcu.pc = 0x15d3;
        break;
    case 0x15d3: /* SHLL r6 byte (part*2) */
        shll8(mcu.r[6]);
        mcu.pc = 0x15d5;
        break;
    case 0x15d5: /* MOVG2 @r6+0x1bfe r2 (word, rom1 part table) */
        ld16(mcu.r[2], ea_r(6, 0x1bfe));
        mcu.pc = 0x15d9;
        break;
    case 0x15d9: /* move r0 #0x00 */
        move8_imm(mcu.r[0], 0x00);
        mcu.pc = 0x15db;
        break;
    case 0x15db: /* BTSTI @r2+5 #4 */
        btst8_mem(ea_r(2, 0x0005), 4);
        mcu.pc = 0x15de;
        break;
    case 0x15de: /* BNE 0x15e4 */
        mcu.pc = f_z() ? 0x15e0 : 0x15e4;
        break;
    case 0x15e0: /* MOVG2 @r3+0xa040 r0 (byte, a040[part]) */
        ld8(mcu.r[0], ea_r(3, 0xa040));
        mcu.pc = 0x15e4;
        break;
    case 0x15e4: /* CLR r2 word */
        clr16(mcu.r[2]);
        mcu.pc = 0x15e6;
        break;
    case 0x15e6: /* cmp r0,b #0x00 */
        cmp_imm8_r0(0x00);
        mcu.pc = 0x15e8;
        break;
    case 0x15e8: /* BNE 0x15ef */
        mcu.pc = f_z() ? 0x15ea : 0x15ef;
        break;
    case 0x15ea: /* bsr16 -> 0x16f4 steal (B8) */
        call(0x15ed, 0x16f4);
        break;
    case 0x15ed: /* BRA 0x15fa */
        mcu.pc = 0x15fa;
        break;
    case 0x15ef: /* cmp r0,b #0x02 */
        cmp_imm8_r0(0x02);
        mcu.pc = 0x15f1;
        break;
    case 0x15f1: /* BNE 0x15f8 */
        mcu.pc = f_z() ? 0x15f3 : 0x15f8;
        break;
    case 0x15f3: /* bsr16 -> 0x173e key_on (B9) */
        call(0x15f6, 0x173e);
        break;
    case 0x15f6: /* BRA 0x15fa */
        mcu.pc = 0x15fa;
        break;
    case 0x15f8: /* move r0 #0x01 */
        move8_imm(mcu.r[0], 0x01);
        mcu.pc = 0x15fa;
        break;
    case 0x15fa: /* rts */
        ret();
        break;
    default:
        MCU_ReadInstruction();
        break;
    }
    return 1;
}

/* ======================================================================== */
/* B7 note_cmd 0x15fb..0x16a1                                               */
/* ======================================================================== */

uint32_t note_cmd_step(void)
{
    switch (mcu.pc)
    {
    case 0x15fb: /* CLR r0 word */
        clr16(mcu.r[0]);
        mcu.pc = 0x15fd;
        break;
    case 0x15fd: /* CLR r1 word */
        clr16(mcu.r[1]);
        mcu.pc = 0x15ff;
        break;
    case 0x15ff: /* CLR r3 word */
        clr16(mcu.r[3]);
        mcu.pc = 0x1601;
        break;
    case 0x1601: /* MOVG2 (dp,0xa4a4) r3 (byte) */
        ld8(mcu.r[3], ea_dp(0xa4a4));
        mcu.pc = 0x1605;
        break;
    case 0x1605: /* MOVG2 (dp,0xa1e2) r2 (word) */
        ld16(mcu.r[2], ea_dp(0xa1e2));
        mcu.pc = 0x1609;
        break;
    case 0x1609: /* BTSTI @r2+5 #7 */
        btst8_mem(ea_r(2, 0x0005), 7);
        mcu.pc = 0x160c;
        break;
    case 0x160c: /* BEQ 0x1693 */
        mcu.pc = f_z() ? 0x1693 : 0x160f;
        break;
    case 0x160f: /* MOVG2 (dp,0xa1e2) r2 (word) */
        ld16(mcu.r[2], ea_dp(0xa1e2));
        mcu.pc = 0x1613;
        break;
    case 0x1613: /* MOVG2 @r2+5 r0 (byte) */
        ld8(mcu.r[0], ea_r(2, 0x0005));
        mcu.pc = 0x1616;
        break;
    case 0x1615: /* movf r0 @r6+4 (overlap decode, GT quirk) */
        movf_load_quirk_r0(4);
        mcu.pc = 0x1617;
        break;
    case 0x1616: /* AND #0x03 r0 (overlap decode) */
        and8_imm_r0(0x03);
        mcu.pc = 0x1619;
        break;
    case 0x1619: /* CLR r2 word */
        clr16(mcu.r[2]);
        mcu.pc = 0x161b;
        break;
    case 0x161b: /* TST r0 (byte) */
        tst8_reg(mcu.r[0]);
        mcu.pc = 0x161d;
        break;
    case 0x161d: /* BEQ 0x1674 */
        mcu.pc = f_z() ? 0x1674 : 0x1620;
        break;
    case 0x1620: /* ADDQ #-1 r0 */
        addq8(mcu.r[0], -1);
        mcu.pc = 0x1622;
        break;
    case 0x1622: /* BNE 0x1693 */
        mcu.pc = f_z() ? 0x1625 : 0x1693;
        break;
    case 0x1625: /* MOVG2 @r3+0xa220 r2 (byte) */
        ld8(mcu.r[2], ea_r(3, 0xa220));
        mcu.pc = 0x1629;
        break;
    case 0x1626: /* ADD r2 r0 (overlap decode) */
        add8_reg(mcu.r[0], mcu.r[2]);
        mcu.pc = 0x1628;
        break;
    case 0x1627: /* BRA -126 -> 0x15ab (inside alloc_scan range) */
        mcu.pc = 0x15ab;
        break;
    case 0x1629: /* BMI 0x1672 */
        mcu.pc = f_n() ? 0x1672 : 0x162b;
        break;
    case 0x162b: /* MOVG2 (dp,0xa4a6) r1 (byte) */
        ld8(mcu.r[1], ea_dp(0xa4a6));
        mcu.pc = 0x162f;
        break;
    case 0x162f: /* movi r6 #0x000f */
        movi16(mcu.r[6], 0x000f);
        mcu.pc = 0x1632;
        break;
    case 0x1632: /* MOVG2 r3 r0 (word) */
        move16_reg(mcu.r[0], mcu.r[3]);
        mcu.pc = 0x1634;
        break;
    case 0x1634: /* SHLL r0 (x4, a090 + 16*part) */
        shll8(mcu.r[0]);
        mcu.pc = 0x1636;
        break;
    case 0x1636: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x1638;
        break;
    case 0x1638: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x163a;
        break;
    case 0x163a: /* SHLL r0 */
        shll8(mcu.r[0]);
        mcu.pc = 0x163c;
        break;
    case 0x163c: /* ADD #0xa090 r0 */
        add_imm16_r0(0xa090);
        mcu.pc = 0x1640;
        break;
    case 0x1640: /* TST @r0 (byte) */
        tst8_mem(ea_r(0, 0x0000));
        mcu.pc = 0x1642;
        break;
    case 0x1642: /* BMI 0x164b */
        mcu.pc = f_n() ? 0x164b : 0x1644;
        break;
    case 0x1644: /* CMP r0++ r1 */
        cmp8_postinc_r0_r1();
        mcu.pc = 0x1646;
        break;
    case 0x1646: /* BEQ 0x1693 */
        mcu.pc = f_z() ? 0x1693 : 0x1648;
        break;
    case 0x1648: /* cntjmp r6 -11 -> 0x1640 */
        cntjmp_reg(mcu.r[6], 0x164b, 0x1640);
        break;
    case 0x164b: /* MOVG2 (dp,0xa4a6) r4 (byte) */
        ld8(mcu.r[4], ea_dp(0xa4a6));
        mcu.pc = 0x164f;
        break;
    case 0x164f: /* MOVG2 (dp,0xa4a5) r5 (byte) */
        ld8(mcu.r[5], ea_dp(0xa4a5));
        mcu.pc = 0x1653;
        break;
    case 0x1653: /* CMP @r2+0xa314 r4 */
        cmp8_mem_reg(ea_r(2, 0xa314), mcu.r[4]);
        mcu.pc = 0x1657;
        break;
    case 0x1657: /* BNE 0x166c */
        mcu.pc = f_z() ? 0x1659 : 0x166c;
        break;
    case 0x1659: /* CMP @r2+0xa2f8 r5 */
        cmp8_mem_reg(ea_r(2, 0xa2f8), mcu.r[5]);
        mcu.pc = 0x165d;
        break;
    case 0x165d: /* BNE 0x166c */
        mcu.pc = f_z() ? 0x165f : 0x166c;
        break;
    case 0x165f: /* BSET @r2+0xa2a4 #2 */
        bset8_mem(ea_r(2, 0xa2a4), 2);
        mcu.pc = 0x1663;
        break;
    case 0x1663: /* BNE 0x1667 (Z from BSET: bit was 1) */
        mcu.pc = f_z() ? 0x1665 : 0x1667;
        break;
    case 0x1665: /* BRA 0x166c */
        mcu.pc = 0x166c;
        break;
    case 0x1667: /* bsr16 -> 0x17ed note_helper (B11) */
        call(0x166a, 0x17ed);
        break;
    case 0x166a: /* BRA 0x1672 */
        mcu.pc = 0x1672;
        break;
    case 0x166c: /* MOVG2 @r2+0xa250 r2 (byte) */
        ld8(mcu.r[2], ea_r(2, 0xa250));
        mcu.pc = 0x1670;
        break;
    case 0x1670: /* BPL 0x1653 */
        mcu.pc = f_n() ? 0x1672 : 0x1653;
        break;
    case 0x1672: /* BRA 0x1693 */
        mcu.pc = 0x1693;
        break;
    case 0x1674: /* MOVG2 @r3+0xa220 r2 (byte) */
        ld8(mcu.r[2], ea_r(3, 0xa220));
        mcu.pc = 0x1678;
        break;
    case 0x1678: /* BMI 0x1693 */
        mcu.pc = f_n() ? 0x1693 : 0x167a;
        break;
    case 0x167a: /* MOVG2 (dp,0xa4a6) r4 (byte) */
        ld8(mcu.r[4], ea_dp(0xa4a6));
        mcu.pc = 0x167e;
        break;
    case 0x167e: /* MOVG2 (dp,0xa4a5) r5 (byte) */
        ld8(mcu.r[5], ea_dp(0xa4a5));
        mcu.pc = 0x1682;
        break;
    case 0x1682: /* CMP @r2+0xa314 r4 */
        cmp8_mem_reg(ea_r(2, 0xa314), mcu.r[4]);
        mcu.pc = 0x1686;
        break;
    case 0x1686: /* BNE 0x168d */
        mcu.pc = f_z() ? 0x1688 : 0x168d;
        break;
    case 0x1688: /* bsr16 -> 0x180d aux_alloc (B12) */
        call(0x168b, 0x180d);
        break;
    case 0x168b: /* BRA 0x1693 */
        mcu.pc = 0x1693;
        break;
    case 0x168d: /* MOVG2 @r2+0xa250 r2 (byte) */
        ld8(mcu.r[2], ea_r(2, 0xa250));
        mcu.pc = 0x1691;
        break;
    case 0x1691: /* BPL 0x1682 */
        mcu.pc = f_n() ? 0x1693 : 0x1682;
        break;
    case 0x1693: /* bsr16 -> 0x157d alloc_scan (B5, recursion) */
        call(0x1696, 0x157d);
        break;
    case 0x1696: /* BLE 0x169c */
        mcu.pc = (f_z() || (f_n() != f_v())) ? 0x169c : 0x1698;
        break;
    case 0x1698: /* move r0 #0xff */
        move8_imm(mcu.r[0], 0xff);
        mcu.pc = 0x169a;
        break;
    case 0x169a: /* BRA 0x16a1 */
        mcu.pc = 0x16a1;
        break;
    case 0x169c: /* bsr16 -> 0x18ce desc_setup (B15) */
        call(0x169f, 0x18ce);
        break;
    case 0x169f: /* CLR r0 byte */
        clr8_reg(mcu.r[0]);
        mcu.pc = 0x16a1;
        break;
    case 0x16a1: /* rts */
        ret();
        break;
    default:
        MCU_ReadInstruction();
        break;
    }
    return 1;
}

/* ---- PC lists (one entry per decoded instruction; 346 total) ----------- */

const uint16_t kNoteFillPcs[] = {
    0x0f86, 0x0f8a, 0x0f8e, 0x0f90, 0x0f92, 0x0f94, 0x0f97, 0x0f9a,
    0x0f9c, 0x0fa0, 0x0fa4, 0x0fa8, 0x0fac, 0x0fb0, 0x0fb2, 0x0fb4,
    0x0fb8, 0x0fbc, 0x0fbe, 0x0fc2, 0x0fc4, 0x0fc6, 0x0fca, 0x0fcc,
    0x0fd0, 0x0fd4, 0x0fd8, 0x0fdc, 0x0fe0, 0x0fe4, 0x0fe8, 0x0fea,
    0x0fee, 0x0ff2, 0x0ff6, 0x0ffa, 0x0ffe, 0x1002, 0x1006, 0x1007,
    0x100a, 0x100c, 0x100e, 0x100f, 0x1012, 0x1016, 0x101a, 0x101e,
    0x1022, 0x1026, 0x102b, 0x102f, 0x1033, 0x1034, 0x1038, 0x103c,
    0x1040, 0x1043, 0x1046, 0x104a, 0x104c, 0x104d, 0x1051, 0x1055,
    0x1056, 0x1059, 0x105c, 0x1060, 0x1062, 0x1063, 0x1067, 0x106b,
    0x106f, 0x1072, 0x1076, 0x107a, 0x107c, 0x107d, 0x107f, 0x1083,
    0x1087, 0x108b, 0x108f, 0x1093, 0x1095, 0x1099, 0x109c, 0x10a0,
    0x10a5, 0x10a7, 0x10a8, 0x10aa, 0x10ae, 0x10b1, 0x10b5, 0x10b7,
    0x10bb, 0x10bf, 0x10c1, 0x10c3, 0x10c7, 0x10c9, 0x10cd, 0x10cf,
    0x10d3, 0x10d5, 0x10d8, 0x10da, 0x10de, 0x10e2, 0x10e4, 0x10e8,
    0x10ec, 0x10ee, 0x10f2, 0x10f6, 0x10fa, 0x10fe, 0x1102, 0x1106,
    0x1109, 0x110b, 0x110f, 0x1110, 0x1111, 0x1115, 0x1118, 0x111c,
    0x1121, 0x1123, 0x1126, 0x112a, 0x112d, 0x1131, 0x1133, 0x1137,
    0x113b, 0x113d, 0x113f, 0x1143, 0x1145, 0x1149, 0x114b, 0x114f,
    0x1151, 0x1155, 0x1157, 0x115a, 0x115c, 0x1160, 0x1164, 0x1166,
    0x116a, 0x116e, 0x1170, 0x1174, 0x1178, 0x117c, 0x1180, 0x1184,
    0x1188, 0x118b, 0x118f,
};

const uint16_t kScanAPcs[] = {
    0x13a9, 0x13ab, 0x13b0, 0x13b2, 0x13b6, 0x13b8, 0x13bc, 0x13be,
    0x13c1, 0x13c3, 0x13c7, 0x13c9,
};

const uint16_t kKillAPcs[] = {
    0x1459, 0x145e, 0x1462, 0x1464, 0x1468, 0x146a, 0x146d, 0x146f,
    0x1471, 0x1473, 0x1475, 0x1477, 0x1479, 0x147b, 0x147d, 0x147f,
    0x1481, 0x1483, 0x1486, 0x1488, 0x148c, 0x1491, 0x1496, 0x149a,
    0x149c, 0x14a1, 0x14a6, 0x14a8,
};

const uint16_t kScanBPcs[] = {
    0x151e, 0x1520, 0x1522, 0x1526, 0x1528, 0x152a, 0x152e, 0x1530,
    0x1534, 0x1536, 0x153a, 0x153d, 0x153f, 0x1541, 0x1543, 0x1545,
    0x1547, 0x154b, 0x154d, 0x154f, 0x1551, 0x1553, 0x1556, 0x155a,
    0x155c, 0x1561, 0x1566, 0x156a, 0x156c, 0x1571, 0x1576, 0x157a,
    0x157c,
};

const uint16_t kAllocScanPcs[] = {
    0x157d, 0x1581, 0x1583, 0x1587, 0x158b, 0x158d, 0x1591, 0x1595,
    0x1599, 0x159b, 0x159f, 0x15a1, 0x15a3, 0x15a5, 0x15a9, 0x15ad,
    0x15b1, 0x15b3, 0x15b5, 0x15b7, 0x15ba, 0x15be, 0x15c2, 0x15c4,
    0x15c8, 0x15ca, 0x15cc, 0x15d0,
};

const uint16_t kNoteDispPcs[] = {
    0x15d1, 0x15d3, 0x15d5, 0x15d9, 0x15db, 0x15de, 0x15e0, 0x15e4,
    0x15e6, 0x15e8, 0x15ea, 0x15ed, 0x15ef, 0x15f1, 0x15f3, 0x15f6,
    0x15f8, 0x15fa,
};

const uint16_t kNoteCmdPcs[] = {
    0x15fb, 0x15fd, 0x15ff, 0x1601, 0x1605, 0x1609, 0x160c, 0x160f,
    0x1613, 0x1615, 0x1616, 0x1619, 0x161b, 0x161d, 0x1620, 0x1622,
    0x1625, 0x1626, 0x1627, 0x1629, 0x162b, 0x162f, 0x1632, 0x1634,
    0x1636, 0x1638, 0x163a, 0x163c, 0x1640, 0x1642, 0x1644, 0x1646,
    0x1648, 0x164b, 0x164f, 0x1653, 0x1657, 0x1659, 0x165d, 0x165f,
    0x1663, 0x1665, 0x1667, 0x166a, 0x166c, 0x1670, 0x1672, 0x1674,
    0x1678, 0x167a, 0x167e, 0x1682, 0x1686, 0x1688, 0x168b, 0x168d,
    0x1691, 0x1693, 0x1696, 0x1698, 0x169a, 0x169c, 0x169f, 0x16a1,
};

} /* anonymous namespace */
} /* namespace mk2c */

static void register_step_pcs(const uint16_t *pcs, uint32_t count, mk2cpp_hand_routine_fn fn)
{
    for (uint32_t i = 0; i < count; i++)
        MK2CPP_HandRegisterRoutine(pcs[i], fn);
}

void MK2CPP_NotePathFillTables(void)
{
#if MK2CPP_HAND_NOTE_PATH
    register_step_pcs(mk2c::kNoteFillPcs,
                      (uint32_t)(sizeof(mk2c::kNoteFillPcs) / sizeof(mk2c::kNoteFillPcs[0])),
                      &mk2c::note_fill_step);
    register_step_pcs(mk2c::kScanAPcs,
                      (uint32_t)(sizeof(mk2c::kScanAPcs) / sizeof(mk2c::kScanAPcs[0])),
                      &mk2c::scan_a_step);
    register_step_pcs(mk2c::kKillAPcs,
                      (uint32_t)(sizeof(mk2c::kKillAPcs) / sizeof(mk2c::kKillAPcs[0])),
                      &mk2c::kill_a_step);
    register_step_pcs(mk2c::kScanBPcs,
                      (uint32_t)(sizeof(mk2c::kScanBPcs) / sizeof(mk2c::kScanBPcs[0])),
                      &mk2c::scan_b_step);
    register_step_pcs(mk2c::kAllocScanPcs,
                      (uint32_t)(sizeof(mk2c::kAllocScanPcs) / sizeof(mk2c::kAllocScanPcs[0])),
                      &mk2c::alloc_scan_step);
    register_step_pcs(mk2c::kNoteDispPcs,
                      (uint32_t)(sizeof(mk2c::kNoteDispPcs) / sizeof(mk2c::kNoteDispPcs[0])),
                      &mk2c::note_disp_step);
    register_step_pcs(mk2c::kNoteCmdPcs,
                      (uint32_t)(sizeof(mk2c::kNoteCmdPcs) / sizeof(mk2c::kNoteCmdPcs[0])),
                      &mk2c::note_cmd_step);
#endif
}

/* Self-registration (parallel safe): pcm_enable.cpp calls
 * mk2c::hand_fill_modules() from MK2CPP_HandFillTables, after the built-in
 * tables. No shared aggregator file is touched. */
namespace
{
struct NotePathModule
{
    NotePathModule() { mk2c::hand_register_module(&MK2CPP_NotePathFillTables); }
};
NotePathModule g_note_path_module;
} /* anonymous namespace */
