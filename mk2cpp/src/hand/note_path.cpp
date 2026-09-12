/*
 * HAND voice/note_path -- note_fill (B1) and the first half of the note
 * descriptor chain (B2..B7), one host step per H8 instruction.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 * M4 closure step 2 (semantic rewrite): one named void step function per PC,
 * registered flat=pc/cp0 via MK2CPP_HandRegister.
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
 * Style: every old switch arm is now its own void step_<routine>_<semantics>
 * (void) that sets mcu.pc exactly as the old arm did; the bodies are the
 * literal case bodies (only helper aliases and the bpl/bmi/bne/beq/bls/ble/bgt
 * shims differ). The host keeps its per-step interrupt poll, TIMER_Clock,
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

void load8(uint16_t &reg, uint32_t addr)
{
    uint8_t value = MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

void load16(uint16_t &reg, uint32_t addr)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t value = MCU_Read16(addr);
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

void store8(uint32_t addr, uint32_t value)
{
    MCU_Write(addr, (uint8_t)value);
    MCU_SetStatusCommon(value, 0);
}

void store16(uint32_t addr, uint32_t value)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, (uint16_t)value);
    MCU_SetStatusCommon(value, 1);
}

/* ---- CLR / TST / EXTU --------------------------------------------------- */

void clr_reg(uint16_t &reg)
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
void extu8(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    reg = (uint16_t)data;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(data == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* ---- bit operations (BTSTI / BSET / BCLR) ------------------------------ */

void btsti(uint32_t addr, uint32_t bit)
{
    uint32_t data = MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
}

void bset(uint32_t addr, uint32_t bit)
{
    uint32_t data = MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    data |= 1u << bit;
    MCU_Write(addr, (uint8_t)data);
}

void bclr(uint32_t addr, uint32_t bit)
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
void cntjmp(uint16_t &reg, uint16_t next, uint16_t target)
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
void mov16_reg(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)src;
    dst = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* ---- conditional branches (disp8 relative to the next PC) --------------- */

void bpl(uint16_t taken, uint16_t fall)
{
    mcu.pc = f_n() ? fall : taken;
}

void bmi(uint16_t taken, uint16_t fall)
{
    mcu.pc = f_n() ? taken : fall;
}

void bne(uint16_t taken, uint16_t fall)
{
    mcu.pc = f_z() ? fall : taken;
}

void beq(uint16_t taken, uint16_t fall)
{
    mcu.pc = f_z() ? taken : fall;
}

void bls(uint16_t taken, uint16_t fall)
{
    mcu.pc = (f_c() || f_z()) ? taken : fall;
}

void ble(uint16_t taken, uint16_t fall)
{
    mcu.pc = (f_z() || (f_n() != f_v())) ? taken : fall;
}

void bgt(uint16_t taken, uint16_t fall)
{
    mcu.pc = (f_z() || (f_n() != f_v())) ? fall : taken;
}

/* ======================================================================== */
/* B1 note_fill 0x0f86..0x118f, 163 PCs */
/* ======================================================================== */

/* 0x0f86 CLR @r1+0xa3a0 (byte). */
void step_note_fill_clr_r1_0xa3a0_byte(void)
{
    clr8_mem(ea_r(1, 0xa3a0));
    mcu.pc = 0x0f8a;
}

/* 0x0f8a TST (dp,0xa1f2) word. */
void step_note_fill_tst_dp_0xa1f2_word(void)
{
    tst16_mem(ea_dp(0xa1f2));
    mcu.pc = 0x0f8e;
}

/* 0x0f8e BPL 0x0f9a. */
void step_note_fill_bpl_0x0f9a(void)
{
    bpl(0x0f9a, 0x0f90);
}

/* 0x0f90 CLR r2 word (free path). */
void step_note_fill_clr_r2_word_free_path(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x0f92;
}

/* 0x0f92 CLR r3 word (free path). */
void step_note_fill_clr_r3_word_free_path(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x0f94;
}

/* 0x0f94 bsr16 -> 0x19c4 free_voice (B18). */
void step_note_fill_bsr16_to_0x19c4_free_voice_b18(void)
{
    call(0x0f97, 0x19c4);
}

/* 0x0f97 BRA -> 0x1033. */
void step_note_fill_bra_to_0x1033(void)
{
    mcu.pc = 0x1033;
}

/* 0x0f9a SHLL r1 word. */
void step_note_fill_shll_r1_word(void)
{
    shll16(mcu.r[1]);
    mcu.pc = 0x0f9c;
}

/* 0x0f9c MOVG3 r5 -> @r1+0xd054 (word). */
void step_note_fill_movg3_r5_to_r1_0xd054_word(void)
{
    store16(ea_r(1, 0xd054), mcu.r[5]);
    mcu.pc = 0x0fa0;
}

/* 0x0fa0 MOVG2 (dp,0xa1e4) r0 (word). */
void step_note_fill_movg2_dp_0xa1e4_r0_word(void)
{
    load16(mcu.r[0], ea_dp(0xa1e4));
    mcu.pc = 0x0fa4;
}

/* 0x0fa4 MOVG3 r0 -> @r1+0xcfac (word). */
void step_note_fill_movg3_r0_to_r1_0xcfac_word(void)
{
    store16(ea_r(1, 0xcfac), mcu.r[0]);
    mcu.pc = 0x0fa8;
}

/* 0x0fa8 MOVG2 (dp,0xa1e6) r0 (word). */
void step_note_fill_movg2_dp_0xa1e6_r0_word(void)
{
    load16(mcu.r[0], ea_dp(0xa1e6));
    mcu.pc = 0x0fac;
}

/* 0x0fac MOVG3 r0 -> @r1+0xd000 (word). */
void step_note_fill_movg3_r0_to_r1_0xd000_word(void)
{
    store16(ea_r(1, 0xd000), mcu.r[0]);
    mcu.pc = 0x0fb0;
}

/* 0x0fb0 SHLR r1 word. */
void step_note_fill_shlr_r1_word(void)
{
    shlr16(mcu.r[1]);
    mcu.pc = 0x0fb2;
}

/* 0x0fb2 CLR r0 word. */
void step_note_fill_clr_r0_word(void)
{
    clr_reg(mcu.r[0]);
    mcu.pc = 0x0fb4;
}

/* 0x0fb4 MOVG2 (dp,0xa4a4) r0 (byte). */
void step_note_fill_movg2_dp_0xa4a4_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa4a4));
    mcu.pc = 0x0fb8;
}

/* 0x0fb8 MOVG3 r0 -> @r1+0xce78 (byte). */
void step_note_fill_movg3_r0_to_r1_0xce78_byte(void)
{
    store8(ea_r(1, 0xce78), mcu.r[0]);
    mcu.pc = 0x0fbc;
}

/* 0x0fbc SHLL r0 word. */
void step_note_fill_shll_r0_word(void)
{
    shll16(mcu.r[0]);
    mcu.pc = 0x0fbe;
}

/* 0x0fbe MOVG2 @r0+0xa1b0 r6 (word). */
void step_note_fill_movg2_r0_0xa1b0_r6_word(void)
{
    load16(mcu.r[6], ea_r(0, 0xa1b0));
    mcu.pc = 0x0fc2;
}

/* 0x0fc2 SHLR r0 word. */
void step_note_fill_shlr_r0_word(void)
{
    shlr16(mcu.r[0]);
    mcu.pc = 0x0fc4;
}

/* 0x0fc4 SHLL r1 word. */
void step_note_fill_shll_r1_word_a(void)
{
    shll16(mcu.r[1]);
    mcu.pc = 0x0fc6;
}

/* 0x0fc6 MOVG3 r6 -> @r1+0xa46c (word). */
void step_note_fill_movg3_r6_to_r1_0xa46c_word(void)
{
    store16(ea_r(1, 0xa46c), mcu.r[6]);
    mcu.pc = 0x0fca;
}

/* 0x0fca SHLR r1 word. */
void step_note_fill_shlr_r1_word_a(void)
{
    shlr16(mcu.r[1]);
    mcu.pc = 0x0fcc;
}

/* 0x0fcc MOVG2 (dp,0xa1d2) r0 (byte). */
void step_note_fill_movg2_dp_0xa1d2_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1d2));
    mcu.pc = 0x0fd0;
}

/* 0x0fd0 MOVG3 r0 -> @r1+0xce94 (byte). */
void step_note_fill_movg3_r0_to_r1_0xce94_byte(void)
{
    store8(ea_r(1, 0xce94), mcu.r[0]);
    mcu.pc = 0x0fd4;
}

/* 0x0fd4 MOVG2 (dp,0xa1d3) r0 (byte). */
void step_note_fill_movg2_dp_0xa1d3_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1d3));
    mcu.pc = 0x0fd8;
}

/* 0x0fd8 MOVG3 r0 -> @r1+0xcf3c (byte). */
void step_note_fill_movg3_r0_to_r1_0xcf3c_byte(void)
{
    store8(ea_r(1, 0xcf3c), mcu.r[0]);
    mcu.pc = 0x0fdc;
}

/* 0x0fdc MOVG2 (dp,0xa1d4) r0 (byte). */
void step_note_fill_movg2_dp_0xa1d4_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1d4));
    mcu.pc = 0x0fe0;
}

/* 0x0fe0 MOVG3 r0 -> @r1+0xd134 (byte). */
void step_note_fill_movg3_r0_to_r1_0xd134_byte(void)
{
    store8(ea_r(1, 0xd134), mcu.r[0]);
    mcu.pc = 0x0fe4;
}

/* 0x0fe4 BTSTI (dp,0xa1df) #7. */
void step_note_fill_btsti_dp_0xa1df_7(void)
{
    btsti(ea_dp(0xa1df), 7);
    mcu.pc = 0x0fe8;
}

/* 0x0fe8 BEQ 0x0fee. */
void step_note_fill_beq_0x0fee(void)
{
    beq(0x0fee, 0x0fea);
}

/* 0x0fea BSET (dp,0xa1d1) #7. */
void step_note_fill_bset_dp_0xa1d1_7(void)
{
    bset(ea_dp(0xa1d1), 7);
    mcu.pc = 0x0fee;
}

/* 0x0fee MOVG2 (dp,0xa1d1) r0 (byte). */
void step_note_fill_movg2_dp_0xa1d1_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1d1));
    mcu.pc = 0x0ff2;
}

/* 0x0ff2 MOVG3 r0 -> @r1+0xcf90 (byte). */
void step_note_fill_movg3_r0_to_r1_0xcf90_byte(void)
{
    store8(ea_r(1, 0xcf90), mcu.r[0]);
    mcu.pc = 0x0ff6;
}

/* 0x0ff6 MOVG2 (dp,0xa4a7) r0 (byte). */
void step_note_fill_movg2_dp_0xa4a7_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa4a7));
    mcu.pc = 0x0ffa;
}

/* 0x0ffa MOVG3 r0 -> @r1+0xceb0 (byte). */
void step_note_fill_movg3_r0_to_r1_0xceb0_byte(void)
{
    store8(ea_r(1, 0xceb0), mcu.r[0]);
    mcu.pc = 0x0ffe;
}

/* 0x0ffe MOVG2 (dp,0xa1d6) r0 (byte). */
void step_note_fill_movg2_dp_0xa1d6_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1d6));
    mcu.pc = 0x1002;
}

/* 0x1002 MOVG3 r0 -> @r1+0xd0fc (byte). */
void step_note_fill_movg3_r0_to_r1_0xd0fc_byte(void)
{
    store8(ea_r(1, 0xd0fc), mcu.r[0]);
    mcu.pc = 0x1006;
}

/* 0x1006 MOVG2 (dp,0xa1d7) r0 (byte). */
void step_note_fill_movg2_dp_0xa1d7_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1d7));
    mcu.pc = 0x100a;
}

/* 0x1007 BCLR r1 #7 (overlap decode). */
void step_note_fill_bclr_r1_7_overlap_decode(void)
{
    bclr8_reg(mcu.r[1], 7);
    mcu.pc = 0x1009;
}

/* 0x100a MOVG3 r0 -> @r1+0xcecc (byte). */
void step_note_fill_movg3_r0_to_r1_0xcecc_byte(void)
{
    store8(ea_r(1, 0xcecc), mcu.r[0]);
    mcu.pc = 0x100e;
}

/* 0x100c MOVG3 r0 -> r4++ (word, overlap decode). */
void step_note_fill_movg3_r0_to_r4_word_overlap_decode(void)
{
    uint32_t oea = (uint32_t)mcu.r[4];
    mcu.r[4] = (uint16_t)(mcu.r[4] + 2);
    store16(((uint32_t)mcu.ep << 16) | (uint16_t)oea, mcu.r[0]);
    mcu.pc = 0x100e;
}

/* 0x100e MOVG2 (dp,0xa1f4) r0 (byte). */
void step_note_fill_movg2_dp_0xa1f4_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1f4));
    mcu.pc = 0x1012;
}

/* 0x100f BTSTI r1 #4 (overlap decode). */
void step_note_fill_btsti_r1_4_overlap_decode(void)
{
    btst8_reg(mcu.r[1], 4);
    mcu.pc = 0x1011;
}

/* 0x1012 MOVG3 r0 -> @r1+0xcfe4 (byte). */
void step_note_fill_movg3_r0_to_r1_0xcfe4_byte(void)
{
    store8(ea_r(1, 0xcfe4), mcu.r[0]);
    mcu.pc = 0x1016;
}

/* 0x1016 MOVG2 (dp,0xa1f4) r0 (byte). */
void step_note_fill_movg2_dp_0xa1f4_r0_byte_a(void)
{
    load8(mcu.r[0], ea_dp(0xa1f4));
    mcu.pc = 0x101a;
}

/* 0x101a MOVG3 r0 -> @r1+0xd038 (byte). */
void step_note_fill_movg3_r0_to_r1_0xd038_byte(void)
{
    store8(ea_r(1, 0xd038), mcu.r[0]);
    mcu.pc = 0x101e;
}

/* 0x101e MOVG2 (dp,0xa1f5) r0 (byte). */
void step_note_fill_movg2_dp_0xa1f5_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1f5));
    mcu.pc = 0x1022;
}

/* 0x1022 MOVG3 r0 -> @r1+0xd08c (byte). */
void step_note_fill_movg3_r0_to_r1_0xd08c_byte(void)
{
    store8(ea_r(1, 0xd08c), mcu.r[0]);
    mcu.pc = 0x1026;
}

/* 0x1026 MOVG #0x02 -> @r1+0xd0e0. */
void step_note_fill_movg_0x02_to_r1_0xd0e0(void)
{
    mov_imm8_mem(ea_r(1, 0xd0e0), 0x02);
    mcu.pc = 0x102b;
}

/* 0x102b CLR @r1+0xa4b4 (byte). */
void step_note_fill_clr_r1_0xa4b4_byte(void)
{
    clr8_mem(ea_r(1, 0xa4b4));
    mcu.pc = 0x102f;
}

/* 0x102f CLR @r1+0xacf2 (byte). */
void step_note_fill_clr_r1_0xacf2_byte(void)
{
    clr8_mem(ea_r(1, 0xacf2));
    mcu.pc = 0x1033;
}

/* 0x1033 rts. */
void step_note_fill_rts(void)
{
    ret();
}

/* 0x1034 MOVG2 (dp,0xa1e2) r2 (word). */
void step_note_fill_movg2_dp_0xa1e2_r2_word(void)
{
    load16(mcu.r[2], ea_dp(0xa1e2));
    mcu.pc = 0x1038;
}

/* 0x1038 MOVG2 (dp,0xa1d2) r1 (byte). */
void step_note_fill_movg2_dp_0xa1d2_r1_byte(void)
{
    load8(mcu.r[1], ea_dp(0xa1d2));
    mcu.pc = 0x103c;
}

/* 0x103c MOVG3 r1 -> (dp,0xa1d5) (byte). */
void step_note_fill_movg3_r1_to_dp_0xa1d5_byte(void)
{
    store8(ea_dp(0xa1d5), mcu.r[1]);
    mcu.pc = 0x1040;
}

/* 0x1040 bsr16 -> 0x1af2. */
void step_note_fill_bsr16_to_0x1af2(void)
{
    call(0x1043, 0x1af2);
}

/* 0x1043 bsr16 -> 0x1b09. */
void step_note_fill_bsr16_to_0x1b09(void)
{
    call(0x1046, 0x1b09);
}

/* 0x1046 MOVG3 r0 -> (dp,0xa1d3) (byte). */
void step_note_fill_movg3_r0_to_dp_0xa1d3_byte(void)
{
    store8(ea_dp(0xa1d3), mcu.r[0]);
    mcu.pc = 0x104a;
}

/* 0x104a bsr -> 0x107d. */
void step_note_fill_bsr_to_0x107d(void)
{
    call(0x104c, 0x107d);
}

/* 0x104c rts. */
void step_note_fill_rts_a(void)
{
    ret();
}

/* 0x104d MOVG2 (dp,0xa1e2) r2 (word). */
void step_note_fill_movg2_dp_0xa1e2_r2_word_a(void)
{
    load16(mcu.r[2], ea_dp(0xa1e2));
    mcu.pc = 0x1051;
}

/* 0x1051 MOVG2 (dp,0xa1d2) r1 (byte). */
void step_note_fill_movg2_dp_0xa1d2_r1_byte_a(void)
{
    load8(mcu.r[1], ea_dp(0xa1d2));
    mcu.pc = 0x1055;
}

/* 0x1055 MOVG3 r1 -> (dp,0xa1d5) (byte). */
void step_note_fill_movg3_r1_to_dp_0xa1d5_byte_a(void)
{
    store8(ea_dp(0xa1d5), mcu.r[1]);
    mcu.pc = 0x1059;
}

/* 0x1056 BCLR r1 #5 (overlap decode; next byte not a decoded PC). */
void step_note_fill_bclr_r1_5_overlap_decode_next_byte_not_a_decoded_pc(void)
{
    bclr8_reg(mcu.r[1], 5);
    mcu.pc = 0x1058;
}

/* 0x1059 bsr16 -> 0x1af2. */
void step_note_fill_bsr16_to_0x1af2_a(void)
{
    call(0x105c, 0x1af2);
}

/* 0x105c MOVG3 r1 -> (dp,0xa1d3) (byte). */
void step_note_fill_movg3_r1_to_dp_0xa1d3_byte(void)
{
    store8(ea_dp(0xa1d3), mcu.r[1]);
    mcu.pc = 0x1060;
}

/* 0x1060 bsr -> 0x107d. */
void step_note_fill_bsr_to_0x107d_a(void)
{
    call(0x1062, 0x107d);
}

/* 0x1062 rts. */
void step_note_fill_rts_b(void)
{
    ret();
}

/* 0x1063 MOVG2 (dp,0xa1e2) r2 (word). */
void step_note_fill_movg2_dp_0xa1e2_r2_word_b(void)
{
    load16(mcu.r[2], ea_dp(0xa1e2));
    mcu.pc = 0x1067;
}

/* 0x1067 MOVG2 (dp,0xa1e8) r1 (word). */
void step_note_fill_movg2_dp_0xa1e8_r1_word(void)
{
    load16(mcu.r[1], ea_dp(0xa1e8));
    mcu.pc = 0x106b;
}

/* 0x106b MOVG2 @r1+0x0180 r1 (byte). */
void step_note_fill_movg2_r1_0x0180_r1_byte(void)
{
    load8(mcu.r[1], ea_r(1, 0x0180));
    mcu.pc = 0x106f;
}

/* 0x106f bsr16 -> 0x1af2. */
void step_note_fill_bsr16_to_0x1af2_b(void)
{
    call(0x1072, 0x1af2);
}

/* 0x1072 MOVG3 r0 -> (dp,0xa1d5) (byte). */
void step_note_fill_movg3_r0_to_dp_0xa1d5_byte(void)
{
    store8(ea_dp(0xa1d5), mcu.r[0]);
    mcu.pc = 0x1076;
}

/* 0x1076 MOVG3 r0 -> (dp,0xa1d3) (byte). */
void step_note_fill_movg3_r0_to_dp_0xa1d3_byte_a(void)
{
    store8(ea_dp(0xa1d3), mcu.r[0]);
    mcu.pc = 0x107a;
}

/* 0x107a bsr -> 0x107d. */
void step_note_fill_bsr_to_0x107d_b(void)
{
    call(0x107c, 0x107d);
}

/* 0x107c rts. */
void step_note_fill_rts_c(void)
{
    ret();
}

/* 0x107d MOVG3 r0 -> --r7 (word). */
void step_note_fill_movg3_r0_to_r7_word(void)
{
    mcu.r[7] = (uint16_t)(mcu.r[7] - 2);
    store16(((uint32_t)mcu.tp << 16) | mcu.r[7], mcu.r[0]);
    mcu.pc = 0x107f;
}

/* 0x107f MOVG2 (dp,0xa4a9) r3 (byte). */
void step_note_fill_movg2_dp_0xa4a9_r3_byte(void)
{
    load8(mcu.r[3], ea_dp(0xa4a9));
    mcu.pc = 0x1083;
}

/* 0x1083 MOVG2 (dp,0xa1d0) r1 (byte). */
void step_note_fill_movg2_dp_0xa1d0_r1_byte(void)
{
    load8(mcu.r[1], ea_dp(0xa1d0));
    mcu.pc = 0x1087;
}

/* 0x1087 MOVG3 r1 -> @r3+0xa34c (byte). */
void step_note_fill_movg3_r1_to_r3_0xa34c_byte(void)
{
    store8(ea_r(3, 0xa34c), mcu.r[1]);
    mcu.pc = 0x108b;
}

/* 0x108b MOVG2 (dp,0xa4a4) r3 (byte). */
void step_note_fill_movg2_dp_0xa4a4_r3_byte(void)
{
    load8(mcu.r[3], ea_dp(0xa4a4));
    mcu.pc = 0x108f;
}

/* 0x108f BTSTI (dp,0xa1d0) #0. */
void step_note_fill_btsti_dp_0xa1d0_0(void)
{
    btsti(ea_dp(0xa1d0), 0);
    mcu.pc = 0x1093;
}

/* 0x1093 BEQ 0x1109. */
void step_note_fill_beq_0x1109(void)
{
    beq(0x1109, 0x1095);
}

/* 0x1095 MOVG2 (dp,0xa1e4) r5 (word). */
void step_note_fill_movg2_dp_0xa1e4_r5_word(void)
{
    load16(mcu.r[5], ea_dp(0xa1e4));
    mcu.pc = 0x1099;
}

/* 0x1099 ADDS #0x20 r5. */
void step_note_fill_adds_0x20_r5(void)
{
    adds8(mcu.r[5], 0x20);
    mcu.pc = 0x109c;
}

/* 0x109c MOVG3 r5 -> (dp,0xa1e6) (word). */
void step_note_fill_movg3_r5_to_dp_0xa1e6_word(void)
{
    store16(ea_dp(0xa1e6), mcu.r[5]);
    mcu.pc = 0x10a0;
}

/* 0x10a0 SUB @r5+2 #0xffff. */
void step_note_fill_sub_r5_2_0xffff(void)
{
    sub16_mem_imm16(ea_r(5, 0x0002), 0xffff);
    mcu.pc = 0x10a5;
}

/* 0x10a5 BEQ 0x1109. */
void step_note_fill_beq_0x1109_a(void)
{
    beq(0x1109, 0x10a7);
}

/* 0x10a7 bsr16 -> 0x1190. */
void step_note_fill_bsr16_to_0x1190(void)
{
    call(0x10aa, 0x1190);
}

/* 0x10a8 nop (overlap decode; next byte not a decoded PC). */
void step_note_fill_nop_overlap_decode_next_byte_not_a_decoded_pc(void)
{
    mcu.pc = 0x10a9;
}

/* 0x10aa MOVG2 (dp,0xa1ee) r1 (byte). */
void step_note_fill_movg2_dp_0xa1ee_r1_byte(void)
{
    load8(mcu.r[1], ea_dp(0xa1ee));
    mcu.pc = 0x10ae;
}

/* 0x10ae bsr16 -> 0x11b8. */
void step_note_fill_bsr16_to_0x11b8(void)
{
    call(0x10b1, 0x11b8);
}

/* 0x10b1 MOVG2 (dp,0xa4a4) r3 (byte). */
void step_note_fill_movg2_dp_0xa4a4_r3_byte_a(void)
{
    load8(mcu.r[3], ea_dp(0xa4a4));
    mcu.pc = 0x10b5;
}

/* 0x10b5 SHLL r3 word. */
void step_note_fill_shll_r3_word(void)
{
    shll16(mcu.r[3]);
    mcu.pc = 0x10b7;
}

/* 0x10b7 MOVG2 (dp,0xa1d3) r1 (byte). */
void step_note_fill_movg2_dp_0xa1d3_r1_byte(void)
{
    load8(mcu.r[1], ea_dp(0xa1d3));
    mcu.pc = 0x10bb;
}

/* 0x10bb MOVG3 r1 -> @r3+0xa190 (byte). */
void step_note_fill_movg3_r1_to_r3_0xa190_byte(void)
{
    store8(ea_r(3, 0xa190), mcu.r[1]);
    mcu.pc = 0x10bf;
}

/* 0x10bf SHLR r3 word. */
void step_note_fill_shlr_r3_word(void)
{
    shlr16(mcu.r[3]);
    mcu.pc = 0x10c1;
}

/* 0x10c1 CLR r1 word. */
void step_note_fill_clr_r1_word(void)
{
    clr_reg(mcu.r[1]);
    mcu.pc = 0x10c3;
}

/* 0x10c3 MOVG2 (dp,0xa4aa) r1 (byte). */
void step_note_fill_movg2_dp_0xa4aa_r1_byte(void)
{
    load8(mcu.r[1], ea_dp(0xa4aa));
    mcu.pc = 0x10c7;
}

/* 0x10c7 BMI 0x1109. */
void step_note_fill_bmi_0x1109(void)
{
    bmi(0x1109, 0x10c9);
}

/* 0x10c9 BTSTI (dp,0xa1d1) #7. */
void step_note_fill_btsti_dp_0xa1d1_7(void)
{
    btsti(ea_dp(0xa1d1), 7);
    mcu.pc = 0x10cd;
}

/* 0x10cd BEQ 0x10e2. */
void step_note_fill_beq_0x10e2(void)
{
    beq(0x10e2, 0x10cf);
}

/* 0x10cf BSET_ORC #0x0700 r0 (IML=7). */
void step_note_fill_bset_orc_0x0700_r0_iml_7(void)
{
    orc_imm16(0x0700);
    mcu.pc = 0x10d3;
}

/* 0x10d3 stm #0x3e. */
void step_note_fill_stm_0x3e(void)
{
    stm_rlist(0x3e);
    mcu.pc = 0x10d5;
}

/* 0x10d5 jsr #0x516c pcm_start (C1). */
void step_note_fill_jsr_0x516c_pcm_start_c1(void)
{
    call(0x10d8, 0x516c);
}

/* 0x10d8 ldm #0x3e. */
void step_note_fill_ldm_0x3e(void)
{
    ldm_rlist(0x3e);
    mcu.pc = 0x10da;
}

/* 0x10da BCLR_ANDC #0xf8ff r0 (IML=0). */
void step_note_fill_bclr_andc_0xf8ff_r0_iml_0(void)
{
    andc_imm16(0xf8ff);
    mcu.pc = 0x10de;
}

/* 0x10de BSET (dp,0xa1df) #7. */
void step_note_fill_bset_dp_0xa1df_7(void)
{
    bset(ea_dp(0xa1df), 7);
    mcu.pc = 0x10e2;
}

/* 0x10e2 SHLL r1 byte. */
void step_note_fill_shll_r1_byte(void)
{
    shll8(mcu.r[1]);
    mcu.pc = 0x10e4;
}

/* 0x10e4 MOVG2 (dp,0xa1ec) r0 (word). */
void step_note_fill_movg2_dp_0xa1ec_r0_word(void)
{
    load16(mcu.r[0], ea_dp(0xa1ec));
    mcu.pc = 0x10e8;
}

/* 0x10e8 MOVG3 r0 -> @r1+0xcf58 (word). */
void step_note_fill_movg3_r0_to_r1_0xcf58_word(void)
{
    store16(ea_r(1, 0xcf58), mcu.r[0]);
    mcu.pc = 0x10ec;
}

/* 0x10ec SHLR r1 byte. */
void step_note_fill_shlr_r1_byte(void)
{
    shlr8(mcu.r[1]);
    mcu.pc = 0x10ee;
}

/* 0x10ee MOVG2 (dp,0xa1da) r0 (byte). */
void step_note_fill_movg2_dp_0xa1da_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1da));
    mcu.pc = 0x10f2;
}

/* 0x10f2 MOVG3 r0 -> @r1+0xcee8 (byte). */
void step_note_fill_movg3_r0_to_r1_0xcee8_byte(void)
{
    store8(ea_r(1, 0xcee8), mcu.r[0]);
    mcu.pc = 0x10f6;
}

/* 0x10f6 MOVG2 (dp,0xa1dd) r0 (byte). */
void step_note_fill_movg2_dp_0xa1dd_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1dd));
    mcu.pc = 0x10fa;
}

/* 0x10fa MOVG3 r0 -> @r1+0xcf04 (byte). */
void step_note_fill_movg3_r0_to_r1_0xcf04_byte(void)
{
    store8(ea_r(1, 0xcf04), mcu.r[0]);
    mcu.pc = 0x10fe;
}

/* 0x10fe MOVG2 (dp,0xa1ee) r0 (byte). */
void step_note_fill_movg2_dp_0xa1ee_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1ee));
    mcu.pc = 0x1102;
}

/* 0x1102 MOVG3 r0 -> @r1+0xcf20 (byte). */
void step_note_fill_movg3_r0_to_r1_0xcf20_byte(void)
{
    store8(ea_r(1, 0xcf20), mcu.r[0]);
    mcu.pc = 0x1106;
}

/* 0x1106 bsr16 -> 0x0f86 (self recursion). */
void step_note_fill_bsr16_to_0x0f86_self_recursion(void)
{
    call(0x1109, 0x0f86);
}

/* 0x1109 MOVG2 r7++ r0 (word). */
void step_note_fill_movg2_r7_r0_word(void)
{
    uint32_t oea = (uint32_t)mcu.r[7];
    mcu.r[7] = (uint16_t)(mcu.r[7] + 2);
    load16(mcu.r[0], ((uint32_t)mcu.tp << 16) | (uint16_t)oea);
    mcu.pc = 0x110b;
}

/* 0x110b BTSTI (dp,0xa1d0) #1. */
void step_note_fill_btsti_dp_0xa1d0_1(void)
{
    btsti(ea_dp(0xa1d0), 1);
    mcu.pc = 0x110f;
}

/* 0x110f BEQ 0x118b (fallthrough 0x1111, not the overlap 0x1110). */
void step_note_fill_beq_0x118b_fallthrough_0x1111_not_the_overlap_0x1110(void)
{
    beq(0x118b, 0x1111);
}

/* 0x1110 movsw r2 @(br,$1d) (overlap decode). */
void step_note_fill_movsw_r2_br_1d_overlap_decode(void)
{
    movsw16_r2(0x1d);
    mcu.pc = 0x1112;
}

/* 0x1111 MOVG2 (dp,0xa1e4) r5 (word). */
void step_note_fill_movg2_dp_0xa1e4_r5_word_a(void)
{
    load16(mcu.r[5], ea_dp(0xa1e4));
    mcu.pc = 0x1115;
}

/* 0x1115 ADDS #0x7c r5. */
void step_note_fill_adds_0x7c_r5(void)
{
    adds8(mcu.r[5], 0x7c);
    mcu.pc = 0x1118;
}

/* 0x1118 MOVG3 r5 -> (dp,0xa1e6) (word). */
void step_note_fill_movg3_r5_to_dp_0xa1e6_word_a(void)
{
    store16(ea_dp(0xa1e6), mcu.r[5]);
    mcu.pc = 0x111c;
}

/* 0x111c SUB @r5+2 #0xffff. */
void step_note_fill_sub_r5_2_0xffff_a(void)
{
    sub16_mem_imm16(ea_r(5, 0x0002), 0xffff);
    mcu.pc = 0x1121;
}

/* 0x1121 BEQ 0x118b. */
void step_note_fill_beq_0x118b(void)
{
    beq(0x118b, 0x1123);
}

/* 0x1123 bsr16 -> 0x1190. */
void step_note_fill_bsr16_to_0x1190_a(void)
{
    call(0x1126, 0x1190);
}

/* 0x1126 MOVG2 (dp,0xa1ef) r1 (byte). */
void step_note_fill_movg2_dp_0xa1ef_r1_byte(void)
{
    load8(mcu.r[1], ea_dp(0xa1ef));
    mcu.pc = 0x112a;
}

/* 0x112a bsr16 -> 0x11b8. */
void step_note_fill_bsr16_to_0x11b8_a(void)
{
    call(0x112d, 0x11b8);
}

/* 0x112d MOVG2 (dp,0xa4a4) r3 (byte). */
void step_note_fill_movg2_dp_0xa4a4_r3_byte_b(void)
{
    load8(mcu.r[3], ea_dp(0xa4a4));
    mcu.pc = 0x1131;
}

/* 0x1131 SHLL r3 word. */
void step_note_fill_shll_r3_word_a(void)
{
    shll16(mcu.r[3]);
    mcu.pc = 0x1133;
}

/* 0x1133 MOVG2 (dp,0xa1d3) r1 (byte). */
void step_note_fill_movg2_dp_0xa1d3_r1_byte_a(void)
{
    load8(mcu.r[1], ea_dp(0xa1d3));
    mcu.pc = 0x1137;
}

/* 0x1137 MOVG3 r1 -> @r3+0xa191 (byte). */
void step_note_fill_movg3_r1_to_r3_0xa191_byte(void)
{
    store8(ea_r(3, 0xa191), mcu.r[1]);
    mcu.pc = 0x113b;
}

/* 0x113b SHLR r3 word. */
void step_note_fill_shlr_r3_word_a(void)
{
    shlr16(mcu.r[3]);
    mcu.pc = 0x113d;
}

/* 0x113d CLR r1 word. */
void step_note_fill_clr_r1_word_a(void)
{
    clr_reg(mcu.r[1]);
    mcu.pc = 0x113f;
}

/* 0x113f MOVG2 (dp,0xa4ab) r1 (byte). */
void step_note_fill_movg2_dp_0xa4ab_r1_byte(void)
{
    load8(mcu.r[1], ea_dp(0xa4ab));
    mcu.pc = 0x1143;
}

/* 0x1143 BPL 0x114b. */
void step_note_fill_bpl_0x114b(void)
{
    bpl(0x114b, 0x1145);
}

/* 0x1145 MOVG2 (dp,0xa4aa) r1 (byte). */
void step_note_fill_movg2_dp_0xa4aa_r1_byte_a(void)
{
    load8(mcu.r[1], ea_dp(0xa4aa));
    mcu.pc = 0x1149;
}

/* 0x1149 BMI 0x118b. */
void step_note_fill_bmi_0x118b(void)
{
    bmi(0x118b, 0x114b);
}

/* 0x114b BTSTI (dp,0xa1d1) #7. */
void step_note_fill_btsti_dp_0xa1d1_7_a(void)
{
    btsti(ea_dp(0xa1d1), 7);
    mcu.pc = 0x114f;
}

/* 0x114f BEQ 0x1164. */
void step_note_fill_beq_0x1164(void)
{
    beq(0x1164, 0x1151);
}

/* 0x1151 BSET_ORC #0x0700 r0. */
void step_note_fill_bset_orc_0x0700_r0(void)
{
    orc_imm16(0x0700);
    mcu.pc = 0x1155;
}

/* 0x1155 stm #0x3e. */
void step_note_fill_stm_0x3e_a(void)
{
    stm_rlist(0x3e);
    mcu.pc = 0x1157;
}

/* 0x1157 jsr #0x516c pcm_start (C1). */
void step_note_fill_jsr_0x516c_pcm_start_c1_a(void)
{
    call(0x115a, 0x516c);
}

/* 0x115a ldm #0x3e. */
void step_note_fill_ldm_0x3e_a(void)
{
    ldm_rlist(0x3e);
    mcu.pc = 0x115c;
}

/* 0x115c BCLR_ANDC #0xf8ff r0. */
void step_note_fill_bclr_andc_0xf8ff_r0(void)
{
    andc_imm16(0xf8ff);
    mcu.pc = 0x1160;
}

/* 0x1160 BSET (dp,0xa1df) #7. */
void step_note_fill_bset_dp_0xa1df_7_a(void)
{
    bset(ea_dp(0xa1df), 7);
    mcu.pc = 0x1164;
}

/* 0x1164 SHLL r1 byte. */
void step_note_fill_shll_r1_byte_a(void)
{
    shll8(mcu.r[1]);
    mcu.pc = 0x1166;
}

/* 0x1166 MOVG2 (dp,0xa1ec) r0 (word). */
void step_note_fill_movg2_dp_0xa1ec_r0_word_a(void)
{
    load16(mcu.r[0], ea_dp(0xa1ec));
    mcu.pc = 0x116a;
}

/* 0x116a MOVG3 r0 -> @r1+0xcf58 (word). */
void step_note_fill_movg3_r0_to_r1_0xcf58_word_a(void)
{
    store16(ea_r(1, 0xcf58), mcu.r[0]);
    mcu.pc = 0x116e;
}

/* 0x116e SHLR r1 byte. */
void step_note_fill_shlr_r1_byte_a(void)
{
    shlr8(mcu.r[1]);
    mcu.pc = 0x1170;
}

/* 0x1170 MOVG2 (dp,0xa1db) r0 (byte). */
void step_note_fill_movg2_dp_0xa1db_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1db));
    mcu.pc = 0x1174;
}

/* 0x1174 MOVG3 r0 -> @r1+0xcee8 (byte). */
void step_note_fill_movg3_r0_to_r1_0xcee8_byte_a(void)
{
    store8(ea_r(1, 0xcee8), mcu.r[0]);
    mcu.pc = 0x1178;
}

/* 0x1178 MOVG2 (dp,0xa1de) r0 (byte). */
void step_note_fill_movg2_dp_0xa1de_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1de));
    mcu.pc = 0x117c;
}

/* 0x117c MOVG3 r0 -> @r1+0xcf04 (byte). */
void step_note_fill_movg3_r0_to_r1_0xcf04_byte_a(void)
{
    store8(ea_r(1, 0xcf04), mcu.r[0]);
    mcu.pc = 0x1180;
}

/* 0x1180 MOVG2 (dp,0xa1ef) r0 (byte). */
void step_note_fill_movg2_dp_0xa1ef_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa1ef));
    mcu.pc = 0x1184;
}

/* 0x1184 MOVG3 r0 -> @r1+0xcf20 (byte). */
void step_note_fill_movg3_r0_to_r1_0xcf20_byte_a(void)
{
    store8(ea_r(1, 0xcf20), mcu.r[0]);
    mcu.pc = 0x1188;
}

/* 0x1188 bsr16 -> 0x0f86 (self recursion). */
void step_note_fill_bsr16_to_0x0f86_self_recursion_a(void)
{
    call(0x118b, 0x0f86);
}

/* 0x118b BCLR (dp,0xa1df) #7. */
void step_note_fill_bclr_dp_0xa1df_7(void)
{
    bclr(ea_dp(0xa1df), 7);
    mcu.pc = 0x118f;
}

/* 0x118f rts. */
void step_note_fill_rts_d(void)
{
    ret();
}

/* ======================================================================== */
/* B2 scan_a 0x13a9..0x13c9, 12 PCs */
/* ======================================================================== */

/* 0x13a9 BMI 0x13c9. */
void step_scan_a_bmi_0x13c9(void)
{
    bmi(0x13c9, 0x13ab);
}

/* 0x13ab SUB @r2+0xa288 #0x00 (a288 == 0?). */
void step_scan_a_sub_r2_0xa288_0x00_a288_0(void)
{
    sub8_mem_imm8(ea_r(2, 0xa288), 0x00);
    mcu.pc = 0x13b0;
}

/* 0x13b0 BNE 0x13c3. */
void step_scan_a_bne_0x13c3(void)
{
    bne(0x13c3, 0x13b2);
}

/* 0x13b2 CMP @r2+0xa314 r1. */
void step_scan_a_cmp_r2_0xa314_r1(void)
{
    cmp8_mem_reg(ea_r(2, 0xa314), mcu.r[1]);
    mcu.pc = 0x13b6;
}

/* 0x13b6 BNE 0x13c3. */
void step_scan_a_bne_0x13c3_a(void)
{
    bne(0x13c3, 0x13b8);
}

/* 0x13b8 BTSTI @r2+0xa330 #0. */
void step_scan_a_btsti_r2_0xa330_0(void)
{
    btsti(ea_r(2, 0xa330), 0);
    mcu.pc = 0x13bc;
}

/* 0x13bc BEQ 0x13c3. */
void step_scan_a_beq_0x13c3(void)
{
    beq(0x13c3, 0x13be);
}

/* 0x13be bsr16 -> 0x1459 kill_a (B3). */
void step_scan_a_bsr16_to_0x1459_kill_a_b3(void)
{
    call(0x13c1, 0x1459);
}

/* 0x13c1 BRA 0x13c9. */
void step_scan_a_bra_0x13c9(void)
{
    mcu.pc = 0x13c9;
}

/* 0x13c3 MOVG2 @r2+0xa250 r2 (byte). */
void step_scan_a_movg2_r2_0xa250_r2_byte(void)
{
    load8(mcu.r[2], ea_r(2, 0xa250));
    mcu.pc = 0x13c7;
}

/* 0x13c7 BPL 0x13ab. */
void step_scan_a_bpl_0x13ab(void)
{
    bpl(0x13ab, 0x13c9);
}

/* 0x13c9 rts. */
void step_scan_a_rts(void)
{
    ret();
}

/* ======================================================================== */
/* B3 kill_a 0x1459..0x14a8, 28 PCs */
/* ======================================================================== */

/* 0x1459 MOVG #0x02 -> @r2+0xa288. */
void step_kill_a_movg_0x02_to_r2_0xa288(void)
{
    mov_imm8_mem(ea_r(2, 0xa288), 0x02);
    mcu.pc = 0x145e;
}

/* 0x145e BTSTI @r3+0xa240 #0. */
void step_kill_a_btsti_r3_0xa240_0(void)
{
    btsti(ea_r(3, 0xa240), 0);
    mcu.pc = 0x1462;
}

/* 0x1462 BEQ 0x146a. */
void step_kill_a_beq_0x146a(void)
{
    beq(0x146a, 0x1464);
}

/* 0x1464 BSET @r2+0xa2a4 #0. */
void step_kill_a_bset_r2_0xa2a4_0(void)
{
    bset(ea_r(2, 0xa2a4), 0);
    mcu.pc = 0x1468;
}

/* 0x1468 BRA 0x14a6. */
void step_kill_a_bra_0x14a6(void)
{
    mcu.pc = 0x14a6;
}

/* 0x146a movi r6 #0x000f. */
void step_kill_a_movi_r6_0x000f(void)
{
    movi16(mcu.r[6], 0x000f);
    mcu.pc = 0x146d;
}

/* 0x146d MOVG2 r3 r0 (word). */
void step_kill_a_movg2_r3_r0_word(void)
{
    mov16_reg(mcu.r[0], mcu.r[3]);
    mcu.pc = 0x146f;
}

/* 0x146f SHLL r0 x4 (a090 + 16*part). */
void step_kill_a_shll_r0_x4_a090_16_part(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1471;
}

/* 0x1471 SHLL r0. */
void step_kill_a_shll_r0(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1473;
}

/* 0x1473 SHLL r0. */
void step_kill_a_shll_r0_a(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1475;
}

/* 0x1475 SHLL r0. */
void step_kill_a_shll_r0_b(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1477;
}

/* 0x1477 ADD #0xa090 r0. */
void step_kill_a_add_0xa090_r0(void)
{
    add_imm16_r0(0xa090);
    mcu.pc = 0x147b;
}

/* 0x1479 movf r0 -> @r6+32 (overlap decode, word store). */
void step_kill_a_movf_r0_to_r6_32_overlap_decode_word_store(void)
{
    movf_store16_r0(32);
    mcu.pc = 0x147b;
}

/* 0x147b TST @r0 (byte). */
void step_kill_a_tst_r0_byte(void)
{
    tst8_mem(ea_r(0, 0x0000));
    mcu.pc = 0x147d;
}

/* 0x147d BMI 0x1486. */
void step_kill_a_bmi_0x1486(void)
{
    bmi(0x1486, 0x147f);
}

/* 0x147f CMP r0++ r1. */
void step_kill_a_cmp_r0_r1(void)
{
    cmp8_postinc_r0_r1();
    mcu.pc = 0x1481;
}

/* 0x1481 BEQ 0x14a6. */
void step_kill_a_beq_0x14a6(void)
{
    beq(0x14a6, 0x1483);
}

/* 0x1483 cntjmp r6 -11 -> 0x147b. */
void step_kill_a_cntjmp_r6_11_to_0x147b(void)
{
    cntjmp(mcu.r[6], 0x1486, 0x147b);
}

/* 0x1486 CLR r1 word. */
void step_kill_a_clr_r1_word(void)
{
    clr_reg(mcu.r[1]);
    mcu.pc = 0x1488;
}

/* 0x1488 MOVG2 @r2+0xa2dc r1 (byte). */
void step_kill_a_movg2_r2_0xa2dc_r1_byte(void)
{
    load8(mcu.r[1], ea_r(2, 0xa2dc));
    mcu.pc = 0x148c;
}

/* 0x148c MOVG #0x01 -> @r1+0xa3bc. */
void step_kill_a_movg_0x01_to_r1_0xa3bc(void)
{
    mov_imm8_mem(ea_r(1, 0xa3bc), 0x01);
    mcu.pc = 0x1491;
}

/* 0x1491 MOVG #0xff -> @r1+0xa4b4. */
void step_kill_a_movg_0xff_to_r1_0xa4b4(void)
{
    mov_imm8_mem(ea_r(1, 0xa4b4), 0xff);
    mcu.pc = 0x1496;
}

/* 0x1496 MOVG2 @r1+0xa410 r1 (byte). */
void step_kill_a_movg2_r1_0xa410_r1_byte(void)
{
    load8(mcu.r[1], ea_r(1, 0xa410));
    mcu.pc = 0x149a;
}

/* 0x149a BMI 0x14a6. */
void step_kill_a_bmi_0x14a6(void)
{
    bmi(0x14a6, 0x149c);
}

/* 0x149c MOVG #0x01 -> @r1+0xa3bc (chain propagate). */
void step_kill_a_movg_0x01_to_r1_0xa3bc_chain_propagate(void)
{
    mov_imm8_mem(ea_r(1, 0xa3bc), 0x01);
    mcu.pc = 0x14a1;
}

/* 0x14a1 MOVG #0xff -> @r1+0xa4b4. */
void step_kill_a_movg_0xff_to_r1_0xa4b4_a(void)
{
    mov_imm8_mem(ea_r(1, 0xa4b4), 0xff);
    mcu.pc = 0x14a6;
}

/* 0x14a6 EXTU r0. */
void step_kill_a_extu_r0(void)
{
    extu8(mcu.r[0]);
    mcu.pc = 0x14a8;
}

/* 0x14a8 rts. */
void step_kill_a_rts(void)
{
    ret();
}

/* ======================================================================== */
/* B4 scan_b 0x151e..0x157c, 33 PCs */
/* ======================================================================== */

/* 0x151e CLR r1 word. */
void step_scan_b_clr_r1_word(void)
{
    clr_reg(mcu.r[1]);
    mcu.pc = 0x1520;
}

/* 0x1520 EXTU r3. */
void step_scan_b_extu_r3(void)
{
    extu8(mcu.r[3]);
    mcu.pc = 0x1522;
}

/* 0x1522 BCLR @r3+0xa240 #0. */
void step_scan_b_bclr_r3_0xa240_0(void)
{
    bclr(ea_r(3, 0xa240), 0);
    mcu.pc = 0x1526;
}

/* 0x1526 BEQ 0x157c. */
void step_scan_b_beq_0x157c(void)
{
    beq(0x157c, 0x1528);
}

/* 0x1528 CLR r2 word. */
void step_scan_b_clr_r2_word(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x152a;
}

/* 0x152a MOVG2 @r3+0xa220 r2 (byte). */
void step_scan_b_movg2_r3_0xa220_r2_byte(void)
{
    load8(mcu.r[2], ea_r(3, 0xa220));
    mcu.pc = 0x152e;
}

/* 0x152e BMI 0x157c. */
void step_scan_b_bmi_0x157c(void)
{
    bmi(0x157c, 0x1530);
}

/* 0x1530 BCLR @r2+0xa2a4 #0. */
void step_scan_b_bclr_r2_0xa2a4_0(void)
{
    bclr(ea_r(2, 0xa2a4), 0);
    mcu.pc = 0x1534;
}

/* 0x1534 BEQ 0x1576. */
void step_scan_b_beq_0x1576(void)
{
    beq(0x1576, 0x1536);
}

/* 0x1536 MOVG2 @r2+0xa314 r1 (byte). */
void step_scan_b_movg2_r2_0xa314_r1_byte(void)
{
    load8(mcu.r[1], ea_r(2, 0xa314));
    mcu.pc = 0x153a;
}

/* 0x153a movi r6 #0x000f. */
void step_scan_b_movi_r6_0x000f(void)
{
    movi16(mcu.r[6], 0x000f);
    mcu.pc = 0x153d;
}

/* 0x153d MOVG2 r3 r0 (word). */
void step_scan_b_movg2_r3_r0_word(void)
{
    mov16_reg(mcu.r[0], mcu.r[3]);
    mcu.pc = 0x153f;
}

/* 0x153f SHLL r0 (x4, a090 + 16*part). */
void step_scan_b_shll_r0_x4_a090_16_part(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1541;
}

/* 0x1541 SHLL r0. */
void step_scan_b_shll_r0(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1543;
}

/* 0x1543 SHLL r0. */
void step_scan_b_shll_r0_a(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1545;
}

/* 0x1545 SHLL r0. */
void step_scan_b_shll_r0_b(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1547;
}

/* 0x1547 ADD #0xa090 r0. */
void step_scan_b_add_0xa090_r0(void)
{
    add_imm16_r0(0xa090);
    mcu.pc = 0x154b;
}

/* 0x154b TST @r0 (byte). */
void step_scan_b_tst_r0_byte(void)
{
    tst8_mem(ea_r(0, 0x0000));
    mcu.pc = 0x154d;
}

/* 0x154d BMI 0x1556. */
void step_scan_b_bmi_0x1556(void)
{
    bmi(0x1556, 0x154f);
}

/* 0x154f CMP r0++ r1. */
void step_scan_b_cmp_r0_r1(void)
{
    cmp8_postinc_r0_r1();
    mcu.pc = 0x1551;
}

/* 0x1551 BEQ 0x1576. */
void step_scan_b_beq_0x1576_a(void)
{
    beq(0x1576, 0x1553);
}

/* 0x1553 cntjmp r6 -11 -> 0x154b. */
void step_scan_b_cntjmp_r6_11_to_0x154b(void)
{
    cntjmp(mcu.r[6], 0x1556, 0x154b);
}

/* 0x1556 MOVG2 @r2+0xa2dc r1 (byte). */
void step_scan_b_movg2_r2_0xa2dc_r1_byte(void)
{
    load8(mcu.r[1], ea_r(2, 0xa2dc));
    mcu.pc = 0x155a;
}

/* 0x155a EXTU r1. */
void step_scan_b_extu_r1(void)
{
    extu8(mcu.r[1]);
    mcu.pc = 0x155c;
}

/* 0x155c MOVG #0x01 -> @r1+0xa3bc. */
void step_scan_b_movg_0x01_to_r1_0xa3bc(void)
{
    mov_imm8_mem(ea_r(1, 0xa3bc), 0x01);
    mcu.pc = 0x1561;
}

/* 0x1561 MOVG #0xff -> @r1+0xa4b4. */
void step_scan_b_movg_0xff_to_r1_0xa4b4(void)
{
    mov_imm8_mem(ea_r(1, 0xa4b4), 0xff);
    mcu.pc = 0x1566;
}

/* 0x1566 MOVG2 @r1+0xa410 r1 (byte). */
void step_scan_b_movg2_r1_0xa410_r1_byte(void)
{
    load8(mcu.r[1], ea_r(1, 0xa410));
    mcu.pc = 0x156a;
}

/* 0x156a BMI 0x1576. */
void step_scan_b_bmi_0x1576(void)
{
    bmi(0x1576, 0x156c);
}

/* 0x156c MOVG #0x01 -> @r1+0xa3bc (chain propagate). */
void step_scan_b_movg_0x01_to_r1_0xa3bc_chain_propagate(void)
{
    mov_imm8_mem(ea_r(1, 0xa3bc), 0x01);
    mcu.pc = 0x1571;
}

/* 0x1571 MOVG #0xff -> @r1+0xa4b4. */
void step_scan_b_movg_0xff_to_r1_0xa4b4_a(void)
{
    mov_imm8_mem(ea_r(1, 0xa4b4), 0xff);
    mcu.pc = 0x1576;
}

/* 0x1576 MOVG2 @r2+0xa250 r2 (byte). */
void step_scan_b_movg2_r2_0xa250_r2_byte(void)
{
    load8(mcu.r[2], ea_r(2, 0xa250));
    mcu.pc = 0x157a;
}

/* 0x157a BPL 0x1530. */
void step_scan_b_bpl_0x1530(void)
{
    bpl(0x1530, 0x157c);
}

/* 0x157c rts. */
void step_scan_b_rts(void)
{
    ret();
}

/* ======================================================================== */
/* B5 alloc_scan 0x157d..0x15d0, 28 PCs */
/* ======================================================================== */

/* 0x157d CLR (dp,0xa42c). */
void step_alloc_scan_clr_dp_0xa42c(void)
{
    clr8_mem(ea_dp(0xa42c));
    mcu.pc = 0x1581;
}

/* 0x1581 CLR r0 word. */
void step_alloc_scan_clr_r0_word(void)
{
    clr_reg(mcu.r[0]);
    mcu.pc = 0x1583;
}

/* 0x1583 MOVG2 (dp,0xa4a8) r0 (byte). */
void step_alloc_scan_movg2_dp_0xa4a8_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa4a8));
    mcu.pc = 0x1587;
}

/* 0x1587 SUB (dp,0xa42d) r0: shortfall = a4a8 - a42d. */
void step_alloc_scan_sub_dp_0xa42d_r0_shortfall_a4a8_a42d(void)
{
    sub8_r0_mem(ea_dp(0xa42d));
    mcu.pc = 0x158b;
}

/* 0x158b BLE 0x15cc. */
void step_alloc_scan_ble_0x15cc(void)
{
    ble(0x15cc, 0x158d);
}

/* 0x158d MOVG3 r0 -> (dp,0xa42c) (shortfall). */
void step_alloc_scan_movg3_r0_to_dp_0xa42c_shortfall(void)
{
    store8(ea_dp(0xa42c), mcu.r[0]);
    mcu.pc = 0x1591;
}

/* 0x1591 MOVG2 (dp,0xa4a4) r3 (byte). */
void step_alloc_scan_movg2_dp_0xa4a4_r3_byte(void)
{
    load8(mcu.r[3], ea_dp(0xa4a4));
    mcu.pc = 0x1595;
}

/* 0x1595 TST (dp,0x8028) (byte). */
void step_alloc_scan_tst_dp_0x8028_byte(void)
{
    tst8_mem(ea_dp(0x8028));
    mcu.pc = 0x1599;
}

/* 0x1599 BEQ 0x15a1. */
void step_alloc_scan_beq_0x15a1(void)
{
    beq(0x15a1, 0x159b);
}

/* 0x159b CMP (dp,0x8028) r3. */
void step_alloc_scan_cmp_dp_0x8028_r3(void)
{
    cmp8_mem_reg(ea_dp(0x8028), mcu.r[3]);
    mcu.pc = 0x159f;
}

/* 0x159f BLS 0x15a5. */
void step_alloc_scan_bls_0x15a5(void)
{
    bls(0x15a5, 0x15a1);
}

/* 0x15a1 move r3 #0x0f. */
void step_alloc_scan_move_r3_0x0f(void)
{
    move8_imm(mcu.r[3], 0x0f);
    mcu.pc = 0x15a3;
}

/* 0x15a3 BRA 0x15a9. */
void step_alloc_scan_bra_0x15a9(void)
{
    mcu.pc = 0x15a9;
}

/* 0x15a5 MOVG2 (dp,0x8028) r3 (byte). */
void step_alloc_scan_movg2_dp_0x8028_r3_byte(void)
{
    load8(mcu.r[3], ea_dp(0x8028));
    mcu.pc = 0x15a9;
}

/* 0x15a9 MOVG2 @r3+0xa210 r0 (byte). */
void step_alloc_scan_movg2_r3_0xa210_r0_byte(void)
{
    load8(mcu.r[0], ea_r(3, 0xa210));
    mcu.pc = 0x15ad;
}

/* 0x15ad SUB @r3+0x8018 r0. */
void step_alloc_scan_sub_r3_0x8018_r0(void)
{
    sub8_r0_mem(ea_r(3, 0x8018));
    mcu.pc = 0x15b1;
}

/* 0x15b1 BLE 0x15b7. */
void step_alloc_scan_ble_0x15b7(void)
{
    ble(0x15b7, 0x15b3);
}

/* 0x15b3 bsr -> 0x15d1 note_disp (B6). */
void step_alloc_scan_bsr_to_0x15d1_note_disp_b6(void)
{
    call(0x15b5, 0x15d1);
}

/* 0x15b5 BLE 0x15cc. */
void step_alloc_scan_ble_0x15cc_a(void)
{
    ble(0x15cc, 0x15b7);
}

/* 0x15b7 cntjmp r3 -17 -> 0x15a9. */
void step_alloc_scan_cntjmp_r3_17_to_0x15a9(void)
{
    cntjmp(mcu.r[3], 0x15ba, 0x15a9);
}

/* 0x15ba MOVG2 (dp,0xa4a4) r3 (byte). */
void step_alloc_scan_movg2_dp_0xa4a4_r3_byte_a(void)
{
    load8(mcu.r[3], ea_dp(0xa4a4));
    mcu.pc = 0x15be;
}

/* 0x15be MOVG2 (dp,0xa42c) r0 (byte). */
void step_alloc_scan_movg2_dp_0xa42c_r0_byte(void)
{
    load8(mcu.r[0], ea_dp(0xa42c));
    mcu.pc = 0x15c2;
}

/* 0x15c2 BLS 0x15cc. */
void step_alloc_scan_bls_0x15cc(void)
{
    bls(0x15cc, 0x15c4);
}

/* 0x15c4 CMP @r3+0xa210 r0. */
void step_alloc_scan_cmp_r3_0xa210_r0(void)
{
    cmp8_mem_reg(ea_r(3, 0xa210), mcu.r[0]);
    mcu.pc = 0x15c8;
}

/* 0x15c8 BGT 0x15cc. */
void step_alloc_scan_bgt_0x15cc(void)
{
    bgt(0x15cc, 0x15ca);
}

/* 0x15ca bsr -> 0x15d1 note_disp (B6). */
void step_alloc_scan_bsr_to_0x15d1_note_disp_b6_a(void)
{
    call(0x15cc, 0x15d1);
}

/* 0x15cc TST (dp,0xa42c) (byte). */
void step_alloc_scan_tst_dp_0xa42c_byte(void)
{
    tst8_mem(ea_dp(0xa42c));
    mcu.pc = 0x15d0;
}

/* 0x15d0 rts. */
void step_alloc_scan_rts(void)
{
    ret();
}

/* ======================================================================== */
/* B6 note_disp 0x15d1..0x15fa, 18 PCs */
/* ======================================================================== */

/* 0x15d1 MOVG2 r3 r6 (word). */
void step_note_disp_movg2_r3_r6_word(void)
{
    mov16_reg(mcu.r[6], mcu.r[3]);
    mcu.pc = 0x15d3;
}

/* 0x15d3 SHLL r6 byte (part*2). */
void step_note_disp_shll_r6_byte_part_2(void)
{
    shll8(mcu.r[6]);
    mcu.pc = 0x15d5;
}

/* 0x15d5 MOVG2 @r6+0x1bfe r2 (word, rom1 part table). */
void step_note_disp_movg2_r6_0x1bfe_r2_word_rom1_part_table(void)
{
    load16(mcu.r[2], ea_r(6, 0x1bfe));
    mcu.pc = 0x15d9;
}

/* 0x15d9 move r0 #0x00. */
void step_note_disp_move_r0_0x00(void)
{
    move8_imm(mcu.r[0], 0x00);
    mcu.pc = 0x15db;
}

/* 0x15db BTSTI @r2+5 #4. */
void step_note_disp_btsti_r2_5_4(void)
{
    btsti(ea_r(2, 0x0005), 4);
    mcu.pc = 0x15de;
}

/* 0x15de BNE 0x15e4. */
void step_note_disp_bne_0x15e4(void)
{
    bne(0x15e4, 0x15e0);
}

/* 0x15e0 MOVG2 @r3+0xa040 r0 (byte, a040[part]). */
void step_note_disp_movg2_r3_0xa040_r0_byte_a040_part(void)
{
    load8(mcu.r[0], ea_r(3, 0xa040));
    mcu.pc = 0x15e4;
}

/* 0x15e4 CLR r2 word. */
void step_note_disp_clr_r2_word(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x15e6;
}

/* 0x15e6 cmp r0,b #0x00. */
void step_note_disp_cmp_r0_b_0x00(void)
{
    cmp_imm8_r0(0x00);
    mcu.pc = 0x15e8;
}

/* 0x15e8 BNE 0x15ef. */
void step_note_disp_bne_0x15ef(void)
{
    bne(0x15ef, 0x15ea);
}

/* 0x15ea bsr16 -> 0x16f4 steal (B8). */
void step_note_disp_bsr16_to_0x16f4_steal_b8(void)
{
    call(0x15ed, 0x16f4);
}

/* 0x15ed BRA 0x15fa. */
void step_note_disp_bra_0x15fa(void)
{
    mcu.pc = 0x15fa;
}

/* 0x15ef cmp r0,b #0x02. */
void step_note_disp_cmp_r0_b_0x02(void)
{
    cmp_imm8_r0(0x02);
    mcu.pc = 0x15f1;
}

/* 0x15f1 BNE 0x15f8. */
void step_note_disp_bne_0x15f8(void)
{
    bne(0x15f8, 0x15f3);
}

/* 0x15f3 bsr16 -> 0x173e key_on (B9). */
void step_note_disp_bsr16_to_0x173e_key_on_b9(void)
{
    call(0x15f6, 0x173e);
}

/* 0x15f6 BRA 0x15fa. */
void step_note_disp_bra_0x15fa_a(void)
{
    mcu.pc = 0x15fa;
}

/* 0x15f8 move r0 #0x01. */
void step_note_disp_move_r0_0x01(void)
{
    move8_imm(mcu.r[0], 0x01);
    mcu.pc = 0x15fa;
}

/* 0x15fa rts. */
void step_note_disp_rts(void)
{
    ret();
}

/* ======================================================================== */
/* B7 note_cmd 0x15fb..0x16a1, 64 PCs */
/* ======================================================================== */

/* 0x15fb CLR r0 word. */
void step_note_cmd_clr_r0_word(void)
{
    clr_reg(mcu.r[0]);
    mcu.pc = 0x15fd;
}

/* 0x15fd CLR r1 word. */
void step_note_cmd_clr_r1_word(void)
{
    clr_reg(mcu.r[1]);
    mcu.pc = 0x15ff;
}

/* 0x15ff CLR r3 word. */
void step_note_cmd_clr_r3_word(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x1601;
}

/* 0x1601 MOVG2 (dp,0xa4a4) r3 (byte). */
void step_note_cmd_movg2_dp_0xa4a4_r3_byte(void)
{
    load8(mcu.r[3], ea_dp(0xa4a4));
    mcu.pc = 0x1605;
}

/* 0x1605 MOVG2 (dp,0xa1e2) r2 (word). */
void step_note_cmd_movg2_dp_0xa1e2_r2_word(void)
{
    load16(mcu.r[2], ea_dp(0xa1e2));
    mcu.pc = 0x1609;
}

/* 0x1609 BTSTI @r2+5 #7. */
void step_note_cmd_btsti_r2_5_7(void)
{
    btsti(ea_r(2, 0x0005), 7);
    mcu.pc = 0x160c;
}

/* 0x160c BEQ 0x1693. */
void step_note_cmd_beq_0x1693(void)
{
    beq(0x1693, 0x160f);
}

/* 0x160f MOVG2 (dp,0xa1e2) r2 (word). */
void step_note_cmd_movg2_dp_0xa1e2_r2_word_a(void)
{
    load16(mcu.r[2], ea_dp(0xa1e2));
    mcu.pc = 0x1613;
}

/* 0x1613 MOVG2 @r2+5 r0 (byte). */
void step_note_cmd_movg2_r2_5_r0_byte(void)
{
    load8(mcu.r[0], ea_r(2, 0x0005));
    mcu.pc = 0x1616;
}

/* 0x1615 movf r0 @r6+4 (overlap decode, GT quirk). */
void step_note_cmd_movf_r0_r6_4_overlap_decode_gt_quirk(void)
{
    movf_load_quirk_r0(4);
    mcu.pc = 0x1617;
}

/* 0x1616 AND #0x03 r0 (overlap decode). */
void step_note_cmd_and_0x03_r0_overlap_decode(void)
{
    and8_imm_r0(0x03);
    mcu.pc = 0x1619;
}

/* 0x1619 CLR r2 word. */
void step_note_cmd_clr_r2_word(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x161b;
}

/* 0x161b TST r0 (byte). */
void step_note_cmd_tst_r0_byte(void)
{
    tst8_reg(mcu.r[0]);
    mcu.pc = 0x161d;
}

/* 0x161d BEQ 0x1674. */
void step_note_cmd_beq_0x1674(void)
{
    beq(0x1674, 0x1620);
}

/* 0x1620 ADDQ #-1 r0. */
void step_note_cmd_addq_1_r0(void)
{
    addq8(mcu.r[0], -1);
    mcu.pc = 0x1622;
}

/* 0x1622 BNE 0x1693. */
void step_note_cmd_bne_0x1693(void)
{
    bne(0x1693, 0x1625);
}

/* 0x1625 MOVG2 @r3+0xa220 r2 (byte). */
void step_note_cmd_movg2_r3_0xa220_r2_byte(void)
{
    load8(mcu.r[2], ea_r(3, 0xa220));
    mcu.pc = 0x1629;
}

/* 0x1626 ADD r2 r0 (overlap decode). */
void step_note_cmd_add_r2_r0_overlap_decode(void)
{
    add8_reg(mcu.r[0], mcu.r[2]);
    mcu.pc = 0x1628;
}

/* 0x1627 BRA -126 -> 0x15ab (inside alloc_scan range). */
void step_note_cmd_bra_126_to_0x15ab_inside_alloc_scan_range(void)
{
    mcu.pc = 0x15ab;
}

/* 0x1629 BMI 0x1672. */
void step_note_cmd_bmi_0x1672(void)
{
    bmi(0x1672, 0x162b);
}

/* 0x162b MOVG2 (dp,0xa4a6) r1 (byte). */
void step_note_cmd_movg2_dp_0xa4a6_r1_byte(void)
{
    load8(mcu.r[1], ea_dp(0xa4a6));
    mcu.pc = 0x162f;
}

/* 0x162f movi r6 #0x000f. */
void step_note_cmd_movi_r6_0x000f(void)
{
    movi16(mcu.r[6], 0x000f);
    mcu.pc = 0x1632;
}

/* 0x1632 MOVG2 r3 r0 (word). */
void step_note_cmd_movg2_r3_r0_word(void)
{
    mov16_reg(mcu.r[0], mcu.r[3]);
    mcu.pc = 0x1634;
}

/* 0x1634 SHLL r0 (x4, a090 + 16*part). */
void step_note_cmd_shll_r0_x4_a090_16_part(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1636;
}

/* 0x1636 SHLL r0. */
void step_note_cmd_shll_r0(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x1638;
}

/* 0x1638 SHLL r0. */
void step_note_cmd_shll_r0_a(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x163a;
}

/* 0x163a SHLL r0. */
void step_note_cmd_shll_r0_b(void)
{
    shll8(mcu.r[0]);
    mcu.pc = 0x163c;
}

/* 0x163c ADD #0xa090 r0. */
void step_note_cmd_add_0xa090_r0(void)
{
    add_imm16_r0(0xa090);
    mcu.pc = 0x1640;
}

/* 0x1640 TST @r0 (byte). */
void step_note_cmd_tst_r0_byte_a(void)
{
    tst8_mem(ea_r(0, 0x0000));
    mcu.pc = 0x1642;
}

/* 0x1642 BMI 0x164b. */
void step_note_cmd_bmi_0x164b(void)
{
    bmi(0x164b, 0x1644);
}

/* 0x1644 CMP r0++ r1. */
void step_note_cmd_cmp_r0_r1(void)
{
    cmp8_postinc_r0_r1();
    mcu.pc = 0x1646;
}

/* 0x1646 BEQ 0x1693. */
void step_note_cmd_beq_0x1693_a(void)
{
    beq(0x1693, 0x1648);
}

/* 0x1648 cntjmp r6 -11 -> 0x1640. */
void step_note_cmd_cntjmp_r6_11_to_0x1640(void)
{
    cntjmp(mcu.r[6], 0x164b, 0x1640);
}

/* 0x164b MOVG2 (dp,0xa4a6) r4 (byte). */
void step_note_cmd_movg2_dp_0xa4a6_r4_byte(void)
{
    load8(mcu.r[4], ea_dp(0xa4a6));
    mcu.pc = 0x164f;
}

/* 0x164f MOVG2 (dp,0xa4a5) r5 (byte). */
void step_note_cmd_movg2_dp_0xa4a5_r5_byte(void)
{
    load8(mcu.r[5], ea_dp(0xa4a5));
    mcu.pc = 0x1653;
}

/* 0x1653 CMP @r2+0xa314 r4. */
void step_note_cmd_cmp_r2_0xa314_r4(void)
{
    cmp8_mem_reg(ea_r(2, 0xa314), mcu.r[4]);
    mcu.pc = 0x1657;
}

/* 0x1657 BNE 0x166c. */
void step_note_cmd_bne_0x166c(void)
{
    bne(0x166c, 0x1659);
}

/* 0x1659 CMP @r2+0xa2f8 r5. */
void step_note_cmd_cmp_r2_0xa2f8_r5(void)
{
    cmp8_mem_reg(ea_r(2, 0xa2f8), mcu.r[5]);
    mcu.pc = 0x165d;
}

/* 0x165d BNE 0x166c. */
void step_note_cmd_bne_0x166c_a(void)
{
    bne(0x166c, 0x165f);
}

/* 0x165f BSET @r2+0xa2a4 #2. */
void step_note_cmd_bset_r2_0xa2a4_2(void)
{
    bset(ea_r(2, 0xa2a4), 2);
    mcu.pc = 0x1663;
}

/* 0x1663 BNE 0x1667 (Z from BSET: bit was 1). */
void step_note_cmd_bne_0x1667_z_from_bset_bit_was_1(void)
{
    bne(0x1667, 0x1665);
}

/* 0x1665 BRA 0x166c. */
void step_note_cmd_bra_0x166c(void)
{
    mcu.pc = 0x166c;
}

/* 0x1667 bsr16 -> 0x17ed note_helper (B11). */
void step_note_cmd_bsr16_to_0x17ed_note_helper_b11(void)
{
    call(0x166a, 0x17ed);
}

/* 0x166a BRA 0x1672. */
void step_note_cmd_bra_0x1672(void)
{
    mcu.pc = 0x1672;
}

/* 0x166c MOVG2 @r2+0xa250 r2 (byte). */
void step_note_cmd_movg2_r2_0xa250_r2_byte(void)
{
    load8(mcu.r[2], ea_r(2, 0xa250));
    mcu.pc = 0x1670;
}

/* 0x1670 BPL 0x1653. */
void step_note_cmd_bpl_0x1653(void)
{
    bpl(0x1653, 0x1672);
}

/* 0x1672 BRA 0x1693. */
void step_note_cmd_bra_0x1693(void)
{
    mcu.pc = 0x1693;
}

/* 0x1674 MOVG2 @r3+0xa220 r2 (byte). */
void step_note_cmd_movg2_r3_0xa220_r2_byte_a(void)
{
    load8(mcu.r[2], ea_r(3, 0xa220));
    mcu.pc = 0x1678;
}

/* 0x1678 BMI 0x1693. */
void step_note_cmd_bmi_0x1693(void)
{
    bmi(0x1693, 0x167a);
}

/* 0x167a MOVG2 (dp,0xa4a6) r4 (byte). */
void step_note_cmd_movg2_dp_0xa4a6_r4_byte_a(void)
{
    load8(mcu.r[4], ea_dp(0xa4a6));
    mcu.pc = 0x167e;
}

/* 0x167e MOVG2 (dp,0xa4a5) r5 (byte). */
void step_note_cmd_movg2_dp_0xa4a5_r5_byte_a(void)
{
    load8(mcu.r[5], ea_dp(0xa4a5));
    mcu.pc = 0x1682;
}

/* 0x1682 CMP @r2+0xa314 r4. */
void step_note_cmd_cmp_r2_0xa314_r4_a(void)
{
    cmp8_mem_reg(ea_r(2, 0xa314), mcu.r[4]);
    mcu.pc = 0x1686;
}

/* 0x1686 BNE 0x168d. */
void step_note_cmd_bne_0x168d(void)
{
    bne(0x168d, 0x1688);
}

/* 0x1688 bsr16 -> 0x180d aux_alloc (B12). */
void step_note_cmd_bsr16_to_0x180d_aux_alloc_b12(void)
{
    call(0x168b, 0x180d);
}

/* 0x168b BRA 0x1693. */
void step_note_cmd_bra_0x1693_a(void)
{
    mcu.pc = 0x1693;
}

/* 0x168d MOVG2 @r2+0xa250 r2 (byte). */
void step_note_cmd_movg2_r2_0xa250_r2_byte_a(void)
{
    load8(mcu.r[2], ea_r(2, 0xa250));
    mcu.pc = 0x1691;
}

/* 0x1691 BPL 0x1682. */
void step_note_cmd_bpl_0x1682(void)
{
    bpl(0x1682, 0x1693);
}

/* 0x1693 bsr16 -> 0x157d alloc_scan (B5, recursion). */
void step_note_cmd_bsr16_to_0x157d_alloc_scan_b5_recursion(void)
{
    call(0x1696, 0x157d);
}

/* 0x1696 BLE 0x169c. */
void step_note_cmd_ble_0x169c(void)
{
    ble(0x169c, 0x1698);
}

/* 0x1698 move r0 #0xff. */
void step_note_cmd_move_r0_0xff(void)
{
    move8_imm(mcu.r[0], 0xff);
    mcu.pc = 0x169a;
}

/* 0x169a BRA 0x16a1. */
void step_note_cmd_bra_0x16a1(void)
{
    mcu.pc = 0x16a1;
}

/* 0x169c bsr16 -> 0x18ce desc_setup (B15). */
void step_note_cmd_bsr16_to_0x18ce_desc_setup_b15(void)
{
    call(0x169f, 0x18ce);
}

/* 0x169f CLR r0 byte. */
void step_note_cmd_clr_r0_byte(void)
{
    clr8_reg(mcu.r[0]);
    mcu.pc = 0x16a1;
}

/* 0x16a1 rts. */
void step_note_cmd_rts(void)
{
    ret();
}

} /* anonymous namespace */
} /* namespace mk2c */

void MK2CPP_NotePathFillTables(void)
{
#if MK2CPP_HAND_NOTE_PATH
    /* B1 note_fill 0x0f86..0x118f, 163 PCs */
    MK2CPP_HandRegister(0x00000f86u, &mk2c::step_note_fill_clr_r1_0xa3a0_byte);
    MK2CPP_HandRegister(0x00000f8au, &mk2c::step_note_fill_tst_dp_0xa1f2_word);
    MK2CPP_HandRegister(0x00000f8eu, &mk2c::step_note_fill_bpl_0x0f9a);
    MK2CPP_HandRegister(0x00000f90u, &mk2c::step_note_fill_clr_r2_word_free_path);
    MK2CPP_HandRegister(0x00000f92u, &mk2c::step_note_fill_clr_r3_word_free_path);
    MK2CPP_HandRegister(0x00000f94u, &mk2c::step_note_fill_bsr16_to_0x19c4_free_voice_b18);
    MK2CPP_HandRegister(0x00000f97u, &mk2c::step_note_fill_bra_to_0x1033);
    MK2CPP_HandRegister(0x00000f9au, &mk2c::step_note_fill_shll_r1_word);
    MK2CPP_HandRegister(0x00000f9cu, &mk2c::step_note_fill_movg3_r5_to_r1_0xd054_word);
    MK2CPP_HandRegister(0x00000fa0u, &mk2c::step_note_fill_movg2_dp_0xa1e4_r0_word);
    MK2CPP_HandRegister(0x00000fa4u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcfac_word);
    MK2CPP_HandRegister(0x00000fa8u, &mk2c::step_note_fill_movg2_dp_0xa1e6_r0_word);
    MK2CPP_HandRegister(0x00000facu, &mk2c::step_note_fill_movg3_r0_to_r1_0xd000_word);
    MK2CPP_HandRegister(0x00000fb0u, &mk2c::step_note_fill_shlr_r1_word);
    MK2CPP_HandRegister(0x00000fb2u, &mk2c::step_note_fill_clr_r0_word);
    MK2CPP_HandRegister(0x00000fb4u, &mk2c::step_note_fill_movg2_dp_0xa4a4_r0_byte);
    MK2CPP_HandRegister(0x00000fb8u, &mk2c::step_note_fill_movg3_r0_to_r1_0xce78_byte);
    MK2CPP_HandRegister(0x00000fbcu, &mk2c::step_note_fill_shll_r0_word);
    MK2CPP_HandRegister(0x00000fbeu, &mk2c::step_note_fill_movg2_r0_0xa1b0_r6_word);
    MK2CPP_HandRegister(0x00000fc2u, &mk2c::step_note_fill_shlr_r0_word);
    MK2CPP_HandRegister(0x00000fc4u, &mk2c::step_note_fill_shll_r1_word_a);
    MK2CPP_HandRegister(0x00000fc6u, &mk2c::step_note_fill_movg3_r6_to_r1_0xa46c_word);
    MK2CPP_HandRegister(0x00000fcau, &mk2c::step_note_fill_shlr_r1_word_a);
    MK2CPP_HandRegister(0x00000fccu, &mk2c::step_note_fill_movg2_dp_0xa1d2_r0_byte);
    MK2CPP_HandRegister(0x00000fd0u, &mk2c::step_note_fill_movg3_r0_to_r1_0xce94_byte);
    MK2CPP_HandRegister(0x00000fd4u, &mk2c::step_note_fill_movg2_dp_0xa1d3_r0_byte);
    MK2CPP_HandRegister(0x00000fd8u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcf3c_byte);
    MK2CPP_HandRegister(0x00000fdcu, &mk2c::step_note_fill_movg2_dp_0xa1d4_r0_byte);
    MK2CPP_HandRegister(0x00000fe0u, &mk2c::step_note_fill_movg3_r0_to_r1_0xd134_byte);
    MK2CPP_HandRegister(0x00000fe4u, &mk2c::step_note_fill_btsti_dp_0xa1df_7);
    MK2CPP_HandRegister(0x00000fe8u, &mk2c::step_note_fill_beq_0x0fee);
    MK2CPP_HandRegister(0x00000feau, &mk2c::step_note_fill_bset_dp_0xa1d1_7);
    MK2CPP_HandRegister(0x00000feeu, &mk2c::step_note_fill_movg2_dp_0xa1d1_r0_byte);
    MK2CPP_HandRegister(0x00000ff2u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcf90_byte);
    MK2CPP_HandRegister(0x00000ff6u, &mk2c::step_note_fill_movg2_dp_0xa4a7_r0_byte);
    MK2CPP_HandRegister(0x00000ffau, &mk2c::step_note_fill_movg3_r0_to_r1_0xceb0_byte);
    MK2CPP_HandRegister(0x00000ffeu, &mk2c::step_note_fill_movg2_dp_0xa1d6_r0_byte);
    MK2CPP_HandRegister(0x00001002u, &mk2c::step_note_fill_movg3_r0_to_r1_0xd0fc_byte);
    MK2CPP_HandRegister(0x00001006u, &mk2c::step_note_fill_movg2_dp_0xa1d7_r0_byte);
    MK2CPP_HandRegister(0x00001007u, &mk2c::step_note_fill_bclr_r1_7_overlap_decode);
    MK2CPP_HandRegister(0x0000100au, &mk2c::step_note_fill_movg3_r0_to_r1_0xcecc_byte);
    MK2CPP_HandRegister(0x0000100cu, &mk2c::step_note_fill_movg3_r0_to_r4_word_overlap_decode);
    MK2CPP_HandRegister(0x0000100eu, &mk2c::step_note_fill_movg2_dp_0xa1f4_r0_byte);
    MK2CPP_HandRegister(0x0000100fu, &mk2c::step_note_fill_btsti_r1_4_overlap_decode);
    MK2CPP_HandRegister(0x00001012u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcfe4_byte);
    MK2CPP_HandRegister(0x00001016u, &mk2c::step_note_fill_movg2_dp_0xa1f4_r0_byte_a);
    MK2CPP_HandRegister(0x0000101au, &mk2c::step_note_fill_movg3_r0_to_r1_0xd038_byte);
    MK2CPP_HandRegister(0x0000101eu, &mk2c::step_note_fill_movg2_dp_0xa1f5_r0_byte);
    MK2CPP_HandRegister(0x00001022u, &mk2c::step_note_fill_movg3_r0_to_r1_0xd08c_byte);
    MK2CPP_HandRegister(0x00001026u, &mk2c::step_note_fill_movg_0x02_to_r1_0xd0e0);
    MK2CPP_HandRegister(0x0000102bu, &mk2c::step_note_fill_clr_r1_0xa4b4_byte);
    MK2CPP_HandRegister(0x0000102fu, &mk2c::step_note_fill_clr_r1_0xacf2_byte);
    MK2CPP_HandRegister(0x00001033u, &mk2c::step_note_fill_rts);
    MK2CPP_HandRegister(0x00001034u, &mk2c::step_note_fill_movg2_dp_0xa1e2_r2_word);
    MK2CPP_HandRegister(0x00001038u, &mk2c::step_note_fill_movg2_dp_0xa1d2_r1_byte);
    MK2CPP_HandRegister(0x0000103cu, &mk2c::step_note_fill_movg3_r1_to_dp_0xa1d5_byte);
    MK2CPP_HandRegister(0x00001040u, &mk2c::step_note_fill_bsr16_to_0x1af2);
    MK2CPP_HandRegister(0x00001043u, &mk2c::step_note_fill_bsr16_to_0x1b09);
    MK2CPP_HandRegister(0x00001046u, &mk2c::step_note_fill_movg3_r0_to_dp_0xa1d3_byte);
    MK2CPP_HandRegister(0x0000104au, &mk2c::step_note_fill_bsr_to_0x107d);
    MK2CPP_HandRegister(0x0000104cu, &mk2c::step_note_fill_rts_a);
    MK2CPP_HandRegister(0x0000104du, &mk2c::step_note_fill_movg2_dp_0xa1e2_r2_word_a);
    MK2CPP_HandRegister(0x00001051u, &mk2c::step_note_fill_movg2_dp_0xa1d2_r1_byte_a);
    MK2CPP_HandRegister(0x00001055u, &mk2c::step_note_fill_movg3_r1_to_dp_0xa1d5_byte_a);
    MK2CPP_HandRegister(0x00001056u, &mk2c::step_note_fill_bclr_r1_5_overlap_decode_next_byte_not_a_decoded_pc);
    MK2CPP_HandRegister(0x00001059u, &mk2c::step_note_fill_bsr16_to_0x1af2_a);
    MK2CPP_HandRegister(0x0000105cu, &mk2c::step_note_fill_movg3_r1_to_dp_0xa1d3_byte);
    MK2CPP_HandRegister(0x00001060u, &mk2c::step_note_fill_bsr_to_0x107d_a);
    MK2CPP_HandRegister(0x00001062u, &mk2c::step_note_fill_rts_b);
    MK2CPP_HandRegister(0x00001063u, &mk2c::step_note_fill_movg2_dp_0xa1e2_r2_word_b);
    MK2CPP_HandRegister(0x00001067u, &mk2c::step_note_fill_movg2_dp_0xa1e8_r1_word);
    MK2CPP_HandRegister(0x0000106bu, &mk2c::step_note_fill_movg2_r1_0x0180_r1_byte);
    MK2CPP_HandRegister(0x0000106fu, &mk2c::step_note_fill_bsr16_to_0x1af2_b);
    MK2CPP_HandRegister(0x00001072u, &mk2c::step_note_fill_movg3_r0_to_dp_0xa1d5_byte);
    MK2CPP_HandRegister(0x00001076u, &mk2c::step_note_fill_movg3_r0_to_dp_0xa1d3_byte_a);
    MK2CPP_HandRegister(0x0000107au, &mk2c::step_note_fill_bsr_to_0x107d_b);
    MK2CPP_HandRegister(0x0000107cu, &mk2c::step_note_fill_rts_c);
    MK2CPP_HandRegister(0x0000107du, &mk2c::step_note_fill_movg3_r0_to_r7_word);
    MK2CPP_HandRegister(0x0000107fu, &mk2c::step_note_fill_movg2_dp_0xa4a9_r3_byte);
    MK2CPP_HandRegister(0x00001083u, &mk2c::step_note_fill_movg2_dp_0xa1d0_r1_byte);
    MK2CPP_HandRegister(0x00001087u, &mk2c::step_note_fill_movg3_r1_to_r3_0xa34c_byte);
    MK2CPP_HandRegister(0x0000108bu, &mk2c::step_note_fill_movg2_dp_0xa4a4_r3_byte);
    MK2CPP_HandRegister(0x0000108fu, &mk2c::step_note_fill_btsti_dp_0xa1d0_0);
    MK2CPP_HandRegister(0x00001093u, &mk2c::step_note_fill_beq_0x1109);
    MK2CPP_HandRegister(0x00001095u, &mk2c::step_note_fill_movg2_dp_0xa1e4_r5_word);
    MK2CPP_HandRegister(0x00001099u, &mk2c::step_note_fill_adds_0x20_r5);
    MK2CPP_HandRegister(0x0000109cu, &mk2c::step_note_fill_movg3_r5_to_dp_0xa1e6_word);
    MK2CPP_HandRegister(0x000010a0u, &mk2c::step_note_fill_sub_r5_2_0xffff);
    MK2CPP_HandRegister(0x000010a5u, &mk2c::step_note_fill_beq_0x1109_a);
    MK2CPP_HandRegister(0x000010a7u, &mk2c::step_note_fill_bsr16_to_0x1190);
    MK2CPP_HandRegister(0x000010a8u, &mk2c::step_note_fill_nop_overlap_decode_next_byte_not_a_decoded_pc);
    MK2CPP_HandRegister(0x000010aau, &mk2c::step_note_fill_movg2_dp_0xa1ee_r1_byte);
    MK2CPP_HandRegister(0x000010aeu, &mk2c::step_note_fill_bsr16_to_0x11b8);
    MK2CPP_HandRegister(0x000010b1u, &mk2c::step_note_fill_movg2_dp_0xa4a4_r3_byte_a);
    MK2CPP_HandRegister(0x000010b5u, &mk2c::step_note_fill_shll_r3_word);
    MK2CPP_HandRegister(0x000010b7u, &mk2c::step_note_fill_movg2_dp_0xa1d3_r1_byte);
    MK2CPP_HandRegister(0x000010bbu, &mk2c::step_note_fill_movg3_r1_to_r3_0xa190_byte);
    MK2CPP_HandRegister(0x000010bfu, &mk2c::step_note_fill_shlr_r3_word);
    MK2CPP_HandRegister(0x000010c1u, &mk2c::step_note_fill_clr_r1_word);
    MK2CPP_HandRegister(0x000010c3u, &mk2c::step_note_fill_movg2_dp_0xa4aa_r1_byte);
    MK2CPP_HandRegister(0x000010c7u, &mk2c::step_note_fill_bmi_0x1109);
    MK2CPP_HandRegister(0x000010c9u, &mk2c::step_note_fill_btsti_dp_0xa1d1_7);
    MK2CPP_HandRegister(0x000010cdu, &mk2c::step_note_fill_beq_0x10e2);
    MK2CPP_HandRegister(0x000010cfu, &mk2c::step_note_fill_bset_orc_0x0700_r0_iml_7);
    MK2CPP_HandRegister(0x000010d3u, &mk2c::step_note_fill_stm_0x3e);
    MK2CPP_HandRegister(0x000010d5u, &mk2c::step_note_fill_jsr_0x516c_pcm_start_c1);
    MK2CPP_HandRegister(0x000010d8u, &mk2c::step_note_fill_ldm_0x3e);
    MK2CPP_HandRegister(0x000010dau, &mk2c::step_note_fill_bclr_andc_0xf8ff_r0_iml_0);
    MK2CPP_HandRegister(0x000010deu, &mk2c::step_note_fill_bset_dp_0xa1df_7);
    MK2CPP_HandRegister(0x000010e2u, &mk2c::step_note_fill_shll_r1_byte);
    MK2CPP_HandRegister(0x000010e4u, &mk2c::step_note_fill_movg2_dp_0xa1ec_r0_word);
    MK2CPP_HandRegister(0x000010e8u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcf58_word);
    MK2CPP_HandRegister(0x000010ecu, &mk2c::step_note_fill_shlr_r1_byte);
    MK2CPP_HandRegister(0x000010eeu, &mk2c::step_note_fill_movg2_dp_0xa1da_r0_byte);
    MK2CPP_HandRegister(0x000010f2u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcee8_byte);
    MK2CPP_HandRegister(0x000010f6u, &mk2c::step_note_fill_movg2_dp_0xa1dd_r0_byte);
    MK2CPP_HandRegister(0x000010fau, &mk2c::step_note_fill_movg3_r0_to_r1_0xcf04_byte);
    MK2CPP_HandRegister(0x000010feu, &mk2c::step_note_fill_movg2_dp_0xa1ee_r0_byte);
    MK2CPP_HandRegister(0x00001102u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcf20_byte);
    MK2CPP_HandRegister(0x00001106u, &mk2c::step_note_fill_bsr16_to_0x0f86_self_recursion);
    MK2CPP_HandRegister(0x00001109u, &mk2c::step_note_fill_movg2_r7_r0_word);
    MK2CPP_HandRegister(0x0000110bu, &mk2c::step_note_fill_btsti_dp_0xa1d0_1);
    MK2CPP_HandRegister(0x0000110fu, &mk2c::step_note_fill_beq_0x118b_fallthrough_0x1111_not_the_overlap_0x1110);
    MK2CPP_HandRegister(0x00001110u, &mk2c::step_note_fill_movsw_r2_br_1d_overlap_decode);
    MK2CPP_HandRegister(0x00001111u, &mk2c::step_note_fill_movg2_dp_0xa1e4_r5_word_a);
    MK2CPP_HandRegister(0x00001115u, &mk2c::step_note_fill_adds_0x7c_r5);
    MK2CPP_HandRegister(0x00001118u, &mk2c::step_note_fill_movg3_r5_to_dp_0xa1e6_word_a);
    MK2CPP_HandRegister(0x0000111cu, &mk2c::step_note_fill_sub_r5_2_0xffff_a);
    MK2CPP_HandRegister(0x00001121u, &mk2c::step_note_fill_beq_0x118b);
    MK2CPP_HandRegister(0x00001123u, &mk2c::step_note_fill_bsr16_to_0x1190_a);
    MK2CPP_HandRegister(0x00001126u, &mk2c::step_note_fill_movg2_dp_0xa1ef_r1_byte);
    MK2CPP_HandRegister(0x0000112au, &mk2c::step_note_fill_bsr16_to_0x11b8_a);
    MK2CPP_HandRegister(0x0000112du, &mk2c::step_note_fill_movg2_dp_0xa4a4_r3_byte_b);
    MK2CPP_HandRegister(0x00001131u, &mk2c::step_note_fill_shll_r3_word_a);
    MK2CPP_HandRegister(0x00001133u, &mk2c::step_note_fill_movg2_dp_0xa1d3_r1_byte_a);
    MK2CPP_HandRegister(0x00001137u, &mk2c::step_note_fill_movg3_r1_to_r3_0xa191_byte);
    MK2CPP_HandRegister(0x0000113bu, &mk2c::step_note_fill_shlr_r3_word_a);
    MK2CPP_HandRegister(0x0000113du, &mk2c::step_note_fill_clr_r1_word_a);
    MK2CPP_HandRegister(0x0000113fu, &mk2c::step_note_fill_movg2_dp_0xa4ab_r1_byte);
    MK2CPP_HandRegister(0x00001143u, &mk2c::step_note_fill_bpl_0x114b);
    MK2CPP_HandRegister(0x00001145u, &mk2c::step_note_fill_movg2_dp_0xa4aa_r1_byte_a);
    MK2CPP_HandRegister(0x00001149u, &mk2c::step_note_fill_bmi_0x118b);
    MK2CPP_HandRegister(0x0000114bu, &mk2c::step_note_fill_btsti_dp_0xa1d1_7_a);
    MK2CPP_HandRegister(0x0000114fu, &mk2c::step_note_fill_beq_0x1164);
    MK2CPP_HandRegister(0x00001151u, &mk2c::step_note_fill_bset_orc_0x0700_r0);
    MK2CPP_HandRegister(0x00001155u, &mk2c::step_note_fill_stm_0x3e_a);
    MK2CPP_HandRegister(0x00001157u, &mk2c::step_note_fill_jsr_0x516c_pcm_start_c1_a);
    MK2CPP_HandRegister(0x0000115au, &mk2c::step_note_fill_ldm_0x3e_a);
    MK2CPP_HandRegister(0x0000115cu, &mk2c::step_note_fill_bclr_andc_0xf8ff_r0);
    MK2CPP_HandRegister(0x00001160u, &mk2c::step_note_fill_bset_dp_0xa1df_7_a);
    MK2CPP_HandRegister(0x00001164u, &mk2c::step_note_fill_shll_r1_byte_a);
    MK2CPP_HandRegister(0x00001166u, &mk2c::step_note_fill_movg2_dp_0xa1ec_r0_word_a);
    MK2CPP_HandRegister(0x0000116au, &mk2c::step_note_fill_movg3_r0_to_r1_0xcf58_word_a);
    MK2CPP_HandRegister(0x0000116eu, &mk2c::step_note_fill_shlr_r1_byte_a);
    MK2CPP_HandRegister(0x00001170u, &mk2c::step_note_fill_movg2_dp_0xa1db_r0_byte);
    MK2CPP_HandRegister(0x00001174u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcee8_byte_a);
    MK2CPP_HandRegister(0x00001178u, &mk2c::step_note_fill_movg2_dp_0xa1de_r0_byte);
    MK2CPP_HandRegister(0x0000117cu, &mk2c::step_note_fill_movg3_r0_to_r1_0xcf04_byte_a);
    MK2CPP_HandRegister(0x00001180u, &mk2c::step_note_fill_movg2_dp_0xa1ef_r0_byte);
    MK2CPP_HandRegister(0x00001184u, &mk2c::step_note_fill_movg3_r0_to_r1_0xcf20_byte_a);
    MK2CPP_HandRegister(0x00001188u, &mk2c::step_note_fill_bsr16_to_0x0f86_self_recursion_a);
    MK2CPP_HandRegister(0x0000118bu, &mk2c::step_note_fill_bclr_dp_0xa1df_7);
    MK2CPP_HandRegister(0x0000118fu, &mk2c::step_note_fill_rts_d);

    /* B2 scan_a 0x13a9..0x13c9, 12 PCs */
    MK2CPP_HandRegister(0x000013a9u, &mk2c::step_scan_a_bmi_0x13c9);
    MK2CPP_HandRegister(0x000013abu, &mk2c::step_scan_a_sub_r2_0xa288_0x00_a288_0);
    MK2CPP_HandRegister(0x000013b0u, &mk2c::step_scan_a_bne_0x13c3);
    MK2CPP_HandRegister(0x000013b2u, &mk2c::step_scan_a_cmp_r2_0xa314_r1);
    MK2CPP_HandRegister(0x000013b6u, &mk2c::step_scan_a_bne_0x13c3_a);
    MK2CPP_HandRegister(0x000013b8u, &mk2c::step_scan_a_btsti_r2_0xa330_0);
    MK2CPP_HandRegister(0x000013bcu, &mk2c::step_scan_a_beq_0x13c3);
    MK2CPP_HandRegister(0x000013beu, &mk2c::step_scan_a_bsr16_to_0x1459_kill_a_b3);
    MK2CPP_HandRegister(0x000013c1u, &mk2c::step_scan_a_bra_0x13c9);
    MK2CPP_HandRegister(0x000013c3u, &mk2c::step_scan_a_movg2_r2_0xa250_r2_byte);
    MK2CPP_HandRegister(0x000013c7u, &mk2c::step_scan_a_bpl_0x13ab);
    MK2CPP_HandRegister(0x000013c9u, &mk2c::step_scan_a_rts);

    /* B3 kill_a 0x1459..0x14a8, 28 PCs */
    MK2CPP_HandRegister(0x00001459u, &mk2c::step_kill_a_movg_0x02_to_r2_0xa288);
    MK2CPP_HandRegister(0x0000145eu, &mk2c::step_kill_a_btsti_r3_0xa240_0);
    MK2CPP_HandRegister(0x00001462u, &mk2c::step_kill_a_beq_0x146a);
    MK2CPP_HandRegister(0x00001464u, &mk2c::step_kill_a_bset_r2_0xa2a4_0);
    MK2CPP_HandRegister(0x00001468u, &mk2c::step_kill_a_bra_0x14a6);
    MK2CPP_HandRegister(0x0000146au, &mk2c::step_kill_a_movi_r6_0x000f);
    MK2CPP_HandRegister(0x0000146du, &mk2c::step_kill_a_movg2_r3_r0_word);
    MK2CPP_HandRegister(0x0000146fu, &mk2c::step_kill_a_shll_r0_x4_a090_16_part);
    MK2CPP_HandRegister(0x00001471u, &mk2c::step_kill_a_shll_r0);
    MK2CPP_HandRegister(0x00001473u, &mk2c::step_kill_a_shll_r0_a);
    MK2CPP_HandRegister(0x00001475u, &mk2c::step_kill_a_shll_r0_b);
    MK2CPP_HandRegister(0x00001477u, &mk2c::step_kill_a_add_0xa090_r0);
    MK2CPP_HandRegister(0x00001479u, &mk2c::step_kill_a_movf_r0_to_r6_32_overlap_decode_word_store);
    MK2CPP_HandRegister(0x0000147bu, &mk2c::step_kill_a_tst_r0_byte);
    MK2CPP_HandRegister(0x0000147du, &mk2c::step_kill_a_bmi_0x1486);
    MK2CPP_HandRegister(0x0000147fu, &mk2c::step_kill_a_cmp_r0_r1);
    MK2CPP_HandRegister(0x00001481u, &mk2c::step_kill_a_beq_0x14a6);
    MK2CPP_HandRegister(0x00001483u, &mk2c::step_kill_a_cntjmp_r6_11_to_0x147b);
    MK2CPP_HandRegister(0x00001486u, &mk2c::step_kill_a_clr_r1_word);
    MK2CPP_HandRegister(0x00001488u, &mk2c::step_kill_a_movg2_r2_0xa2dc_r1_byte);
    MK2CPP_HandRegister(0x0000148cu, &mk2c::step_kill_a_movg_0x01_to_r1_0xa3bc);
    MK2CPP_HandRegister(0x00001491u, &mk2c::step_kill_a_movg_0xff_to_r1_0xa4b4);
    MK2CPP_HandRegister(0x00001496u, &mk2c::step_kill_a_movg2_r1_0xa410_r1_byte);
    MK2CPP_HandRegister(0x0000149au, &mk2c::step_kill_a_bmi_0x14a6);
    MK2CPP_HandRegister(0x0000149cu, &mk2c::step_kill_a_movg_0x01_to_r1_0xa3bc_chain_propagate);
    MK2CPP_HandRegister(0x000014a1u, &mk2c::step_kill_a_movg_0xff_to_r1_0xa4b4_a);
    MK2CPP_HandRegister(0x000014a6u, &mk2c::step_kill_a_extu_r0);
    MK2CPP_HandRegister(0x000014a8u, &mk2c::step_kill_a_rts);

    /* B4 scan_b 0x151e..0x157c, 33 PCs */
    MK2CPP_HandRegister(0x0000151eu, &mk2c::step_scan_b_clr_r1_word);
    MK2CPP_HandRegister(0x00001520u, &mk2c::step_scan_b_extu_r3);
    MK2CPP_HandRegister(0x00001522u, &mk2c::step_scan_b_bclr_r3_0xa240_0);
    MK2CPP_HandRegister(0x00001526u, &mk2c::step_scan_b_beq_0x157c);
    MK2CPP_HandRegister(0x00001528u, &mk2c::step_scan_b_clr_r2_word);
    MK2CPP_HandRegister(0x0000152au, &mk2c::step_scan_b_movg2_r3_0xa220_r2_byte);
    MK2CPP_HandRegister(0x0000152eu, &mk2c::step_scan_b_bmi_0x157c);
    MK2CPP_HandRegister(0x00001530u, &mk2c::step_scan_b_bclr_r2_0xa2a4_0);
    MK2CPP_HandRegister(0x00001534u, &mk2c::step_scan_b_beq_0x1576);
    MK2CPP_HandRegister(0x00001536u, &mk2c::step_scan_b_movg2_r2_0xa314_r1_byte);
    MK2CPP_HandRegister(0x0000153au, &mk2c::step_scan_b_movi_r6_0x000f);
    MK2CPP_HandRegister(0x0000153du, &mk2c::step_scan_b_movg2_r3_r0_word);
    MK2CPP_HandRegister(0x0000153fu, &mk2c::step_scan_b_shll_r0_x4_a090_16_part);
    MK2CPP_HandRegister(0x00001541u, &mk2c::step_scan_b_shll_r0);
    MK2CPP_HandRegister(0x00001543u, &mk2c::step_scan_b_shll_r0_a);
    MK2CPP_HandRegister(0x00001545u, &mk2c::step_scan_b_shll_r0_b);
    MK2CPP_HandRegister(0x00001547u, &mk2c::step_scan_b_add_0xa090_r0);
    MK2CPP_HandRegister(0x0000154bu, &mk2c::step_scan_b_tst_r0_byte);
    MK2CPP_HandRegister(0x0000154du, &mk2c::step_scan_b_bmi_0x1556);
    MK2CPP_HandRegister(0x0000154fu, &mk2c::step_scan_b_cmp_r0_r1);
    MK2CPP_HandRegister(0x00001551u, &mk2c::step_scan_b_beq_0x1576_a);
    MK2CPP_HandRegister(0x00001553u, &mk2c::step_scan_b_cntjmp_r6_11_to_0x154b);
    MK2CPP_HandRegister(0x00001556u, &mk2c::step_scan_b_movg2_r2_0xa2dc_r1_byte);
    MK2CPP_HandRegister(0x0000155au, &mk2c::step_scan_b_extu_r1);
    MK2CPP_HandRegister(0x0000155cu, &mk2c::step_scan_b_movg_0x01_to_r1_0xa3bc);
    MK2CPP_HandRegister(0x00001561u, &mk2c::step_scan_b_movg_0xff_to_r1_0xa4b4);
    MK2CPP_HandRegister(0x00001566u, &mk2c::step_scan_b_movg2_r1_0xa410_r1_byte);
    MK2CPP_HandRegister(0x0000156au, &mk2c::step_scan_b_bmi_0x1576);
    MK2CPP_HandRegister(0x0000156cu, &mk2c::step_scan_b_movg_0x01_to_r1_0xa3bc_chain_propagate);
    MK2CPP_HandRegister(0x00001571u, &mk2c::step_scan_b_movg_0xff_to_r1_0xa4b4_a);
    MK2CPP_HandRegister(0x00001576u, &mk2c::step_scan_b_movg2_r2_0xa250_r2_byte);
    MK2CPP_HandRegister(0x0000157au, &mk2c::step_scan_b_bpl_0x1530);
    MK2CPP_HandRegister(0x0000157cu, &mk2c::step_scan_b_rts);

    /* B5 alloc_scan 0x157d..0x15d0, 28 PCs */
    MK2CPP_HandRegister(0x0000157du, &mk2c::step_alloc_scan_clr_dp_0xa42c);
    MK2CPP_HandRegister(0x00001581u, &mk2c::step_alloc_scan_clr_r0_word);
    MK2CPP_HandRegister(0x00001583u, &mk2c::step_alloc_scan_movg2_dp_0xa4a8_r0_byte);
    MK2CPP_HandRegister(0x00001587u, &mk2c::step_alloc_scan_sub_dp_0xa42d_r0_shortfall_a4a8_a42d);
    MK2CPP_HandRegister(0x0000158bu, &mk2c::step_alloc_scan_ble_0x15cc);
    MK2CPP_HandRegister(0x0000158du, &mk2c::step_alloc_scan_movg3_r0_to_dp_0xa42c_shortfall);
    MK2CPP_HandRegister(0x00001591u, &mk2c::step_alloc_scan_movg2_dp_0xa4a4_r3_byte);
    MK2CPP_HandRegister(0x00001595u, &mk2c::step_alloc_scan_tst_dp_0x8028_byte);
    MK2CPP_HandRegister(0x00001599u, &mk2c::step_alloc_scan_beq_0x15a1);
    MK2CPP_HandRegister(0x0000159bu, &mk2c::step_alloc_scan_cmp_dp_0x8028_r3);
    MK2CPP_HandRegister(0x0000159fu, &mk2c::step_alloc_scan_bls_0x15a5);
    MK2CPP_HandRegister(0x000015a1u, &mk2c::step_alloc_scan_move_r3_0x0f);
    MK2CPP_HandRegister(0x000015a3u, &mk2c::step_alloc_scan_bra_0x15a9);
    MK2CPP_HandRegister(0x000015a5u, &mk2c::step_alloc_scan_movg2_dp_0x8028_r3_byte);
    MK2CPP_HandRegister(0x000015a9u, &mk2c::step_alloc_scan_movg2_r3_0xa210_r0_byte);
    MK2CPP_HandRegister(0x000015adu, &mk2c::step_alloc_scan_sub_r3_0x8018_r0);
    MK2CPP_HandRegister(0x000015b1u, &mk2c::step_alloc_scan_ble_0x15b7);
    MK2CPP_HandRegister(0x000015b3u, &mk2c::step_alloc_scan_bsr_to_0x15d1_note_disp_b6);
    MK2CPP_HandRegister(0x000015b5u, &mk2c::step_alloc_scan_ble_0x15cc_a);
    MK2CPP_HandRegister(0x000015b7u, &mk2c::step_alloc_scan_cntjmp_r3_17_to_0x15a9);
    MK2CPP_HandRegister(0x000015bau, &mk2c::step_alloc_scan_movg2_dp_0xa4a4_r3_byte_a);
    MK2CPP_HandRegister(0x000015beu, &mk2c::step_alloc_scan_movg2_dp_0xa42c_r0_byte);
    MK2CPP_HandRegister(0x000015c2u, &mk2c::step_alloc_scan_bls_0x15cc);
    MK2CPP_HandRegister(0x000015c4u, &mk2c::step_alloc_scan_cmp_r3_0xa210_r0);
    MK2CPP_HandRegister(0x000015c8u, &mk2c::step_alloc_scan_bgt_0x15cc);
    MK2CPP_HandRegister(0x000015cau, &mk2c::step_alloc_scan_bsr_to_0x15d1_note_disp_b6_a);
    MK2CPP_HandRegister(0x000015ccu, &mk2c::step_alloc_scan_tst_dp_0xa42c_byte);
    MK2CPP_HandRegister(0x000015d0u, &mk2c::step_alloc_scan_rts);

    /* B6 note_disp 0x15d1..0x15fa, 18 PCs */
    MK2CPP_HandRegister(0x000015d1u, &mk2c::step_note_disp_movg2_r3_r6_word);
    MK2CPP_HandRegister(0x000015d3u, &mk2c::step_note_disp_shll_r6_byte_part_2);
    MK2CPP_HandRegister(0x000015d5u, &mk2c::step_note_disp_movg2_r6_0x1bfe_r2_word_rom1_part_table);
    MK2CPP_HandRegister(0x000015d9u, &mk2c::step_note_disp_move_r0_0x00);
    MK2CPP_HandRegister(0x000015dbu, &mk2c::step_note_disp_btsti_r2_5_4);
    MK2CPP_HandRegister(0x000015deu, &mk2c::step_note_disp_bne_0x15e4);
    MK2CPP_HandRegister(0x000015e0u, &mk2c::step_note_disp_movg2_r3_0xa040_r0_byte_a040_part);
    MK2CPP_HandRegister(0x000015e4u, &mk2c::step_note_disp_clr_r2_word);
    MK2CPP_HandRegister(0x000015e6u, &mk2c::step_note_disp_cmp_r0_b_0x00);
    MK2CPP_HandRegister(0x000015e8u, &mk2c::step_note_disp_bne_0x15ef);
    MK2CPP_HandRegister(0x000015eau, &mk2c::step_note_disp_bsr16_to_0x16f4_steal_b8);
    MK2CPP_HandRegister(0x000015edu, &mk2c::step_note_disp_bra_0x15fa);
    MK2CPP_HandRegister(0x000015efu, &mk2c::step_note_disp_cmp_r0_b_0x02);
    MK2CPP_HandRegister(0x000015f1u, &mk2c::step_note_disp_bne_0x15f8);
    MK2CPP_HandRegister(0x000015f3u, &mk2c::step_note_disp_bsr16_to_0x173e_key_on_b9);
    MK2CPP_HandRegister(0x000015f6u, &mk2c::step_note_disp_bra_0x15fa_a);
    MK2CPP_HandRegister(0x000015f8u, &mk2c::step_note_disp_move_r0_0x01);
    MK2CPP_HandRegister(0x000015fau, &mk2c::step_note_disp_rts);

    /* B7 note_cmd 0x15fb..0x16a1, 64 PCs */
    MK2CPP_HandRegister(0x000015fbu, &mk2c::step_note_cmd_clr_r0_word);
    MK2CPP_HandRegister(0x000015fdu, &mk2c::step_note_cmd_clr_r1_word);
    MK2CPP_HandRegister(0x000015ffu, &mk2c::step_note_cmd_clr_r3_word);
    MK2CPP_HandRegister(0x00001601u, &mk2c::step_note_cmd_movg2_dp_0xa4a4_r3_byte);
    MK2CPP_HandRegister(0x00001605u, &mk2c::step_note_cmd_movg2_dp_0xa1e2_r2_word);
    MK2CPP_HandRegister(0x00001609u, &mk2c::step_note_cmd_btsti_r2_5_7);
    MK2CPP_HandRegister(0x0000160cu, &mk2c::step_note_cmd_beq_0x1693);
    MK2CPP_HandRegister(0x0000160fu, &mk2c::step_note_cmd_movg2_dp_0xa1e2_r2_word_a);
    MK2CPP_HandRegister(0x00001613u, &mk2c::step_note_cmd_movg2_r2_5_r0_byte);
    MK2CPP_HandRegister(0x00001615u, &mk2c::step_note_cmd_movf_r0_r6_4_overlap_decode_gt_quirk);
    MK2CPP_HandRegister(0x00001616u, &mk2c::step_note_cmd_and_0x03_r0_overlap_decode);
    MK2CPP_HandRegister(0x00001619u, &mk2c::step_note_cmd_clr_r2_word);
    MK2CPP_HandRegister(0x0000161bu, &mk2c::step_note_cmd_tst_r0_byte);
    MK2CPP_HandRegister(0x0000161du, &mk2c::step_note_cmd_beq_0x1674);
    MK2CPP_HandRegister(0x00001620u, &mk2c::step_note_cmd_addq_1_r0);
    MK2CPP_HandRegister(0x00001622u, &mk2c::step_note_cmd_bne_0x1693);
    MK2CPP_HandRegister(0x00001625u, &mk2c::step_note_cmd_movg2_r3_0xa220_r2_byte);
    MK2CPP_HandRegister(0x00001626u, &mk2c::step_note_cmd_add_r2_r0_overlap_decode);
    MK2CPP_HandRegister(0x00001627u, &mk2c::step_note_cmd_bra_126_to_0x15ab_inside_alloc_scan_range);
    MK2CPP_HandRegister(0x00001629u, &mk2c::step_note_cmd_bmi_0x1672);
    MK2CPP_HandRegister(0x0000162bu, &mk2c::step_note_cmd_movg2_dp_0xa4a6_r1_byte);
    MK2CPP_HandRegister(0x0000162fu, &mk2c::step_note_cmd_movi_r6_0x000f);
    MK2CPP_HandRegister(0x00001632u, &mk2c::step_note_cmd_movg2_r3_r0_word);
    MK2CPP_HandRegister(0x00001634u, &mk2c::step_note_cmd_shll_r0_x4_a090_16_part);
    MK2CPP_HandRegister(0x00001636u, &mk2c::step_note_cmd_shll_r0);
    MK2CPP_HandRegister(0x00001638u, &mk2c::step_note_cmd_shll_r0_a);
    MK2CPP_HandRegister(0x0000163au, &mk2c::step_note_cmd_shll_r0_b);
    MK2CPP_HandRegister(0x0000163cu, &mk2c::step_note_cmd_add_0xa090_r0);
    MK2CPP_HandRegister(0x00001640u, &mk2c::step_note_cmd_tst_r0_byte_a);
    MK2CPP_HandRegister(0x00001642u, &mk2c::step_note_cmd_bmi_0x164b);
    MK2CPP_HandRegister(0x00001644u, &mk2c::step_note_cmd_cmp_r0_r1);
    MK2CPP_HandRegister(0x00001646u, &mk2c::step_note_cmd_beq_0x1693_a);
    MK2CPP_HandRegister(0x00001648u, &mk2c::step_note_cmd_cntjmp_r6_11_to_0x1640);
    MK2CPP_HandRegister(0x0000164bu, &mk2c::step_note_cmd_movg2_dp_0xa4a6_r4_byte);
    MK2CPP_HandRegister(0x0000164fu, &mk2c::step_note_cmd_movg2_dp_0xa4a5_r5_byte);
    MK2CPP_HandRegister(0x00001653u, &mk2c::step_note_cmd_cmp_r2_0xa314_r4);
    MK2CPP_HandRegister(0x00001657u, &mk2c::step_note_cmd_bne_0x166c);
    MK2CPP_HandRegister(0x00001659u, &mk2c::step_note_cmd_cmp_r2_0xa2f8_r5);
    MK2CPP_HandRegister(0x0000165du, &mk2c::step_note_cmd_bne_0x166c_a);
    MK2CPP_HandRegister(0x0000165fu, &mk2c::step_note_cmd_bset_r2_0xa2a4_2);
    MK2CPP_HandRegister(0x00001663u, &mk2c::step_note_cmd_bne_0x1667_z_from_bset_bit_was_1);
    MK2CPP_HandRegister(0x00001665u, &mk2c::step_note_cmd_bra_0x166c);
    MK2CPP_HandRegister(0x00001667u, &mk2c::step_note_cmd_bsr16_to_0x17ed_note_helper_b11);
    MK2CPP_HandRegister(0x0000166au, &mk2c::step_note_cmd_bra_0x1672);
    MK2CPP_HandRegister(0x0000166cu, &mk2c::step_note_cmd_movg2_r2_0xa250_r2_byte);
    MK2CPP_HandRegister(0x00001670u, &mk2c::step_note_cmd_bpl_0x1653);
    MK2CPP_HandRegister(0x00001672u, &mk2c::step_note_cmd_bra_0x1693);
    MK2CPP_HandRegister(0x00001674u, &mk2c::step_note_cmd_movg2_r3_0xa220_r2_byte_a);
    MK2CPP_HandRegister(0x00001678u, &mk2c::step_note_cmd_bmi_0x1693);
    MK2CPP_HandRegister(0x0000167au, &mk2c::step_note_cmd_movg2_dp_0xa4a6_r4_byte_a);
    MK2CPP_HandRegister(0x0000167eu, &mk2c::step_note_cmd_movg2_dp_0xa4a5_r5_byte_a);
    MK2CPP_HandRegister(0x00001682u, &mk2c::step_note_cmd_cmp_r2_0xa314_r4_a);
    MK2CPP_HandRegister(0x00001686u, &mk2c::step_note_cmd_bne_0x168d);
    MK2CPP_HandRegister(0x00001688u, &mk2c::step_note_cmd_bsr16_to_0x180d_aux_alloc_b12);
    MK2CPP_HandRegister(0x0000168bu, &mk2c::step_note_cmd_bra_0x1693_a);
    MK2CPP_HandRegister(0x0000168du, &mk2c::step_note_cmd_movg2_r2_0xa250_r2_byte_a);
    MK2CPP_HandRegister(0x00001691u, &mk2c::step_note_cmd_bpl_0x1682);
    MK2CPP_HandRegister(0x00001693u, &mk2c::step_note_cmd_bsr16_to_0x157d_alloc_scan_b5_recursion);
    MK2CPP_HandRegister(0x00001696u, &mk2c::step_note_cmd_ble_0x169c);
    MK2CPP_HandRegister(0x00001698u, &mk2c::step_note_cmd_move_r0_0xff);
    MK2CPP_HandRegister(0x0000169au, &mk2c::step_note_cmd_bra_0x16a1);
    MK2CPP_HandRegister(0x0000169cu, &mk2c::step_note_cmd_bsr16_to_0x18ce_desc_setup_b15);
    MK2CPP_HandRegister(0x0000169fu, &mk2c::step_note_cmd_clr_r0_byte);
    MK2CPP_HandRegister(0x000016a1u, &mk2c::step_note_cmd_rts);
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
