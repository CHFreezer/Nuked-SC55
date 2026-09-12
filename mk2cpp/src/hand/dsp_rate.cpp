/*
 * HAND dsp/rate -- per-voice DSP rate update, one host step per H8 instruction
 * (M4 wave, out/m4/18_closure_gap.md 4.2 item 27, "0x4DE7 family").
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 * M4 closure step 2 (semantic rewrite): one named void step function per PC, registered via MK2CPP_HandRegister.
 *
 * Routine boundary (derived from the ROM bytes + tools/baselines/dasm_full.txt,
 * not assumed from the fragment table): the family is the single routine
 * 0x4DE7..0x50EE, 302 instruction PCs, terminated by the rts at 0x50EE.  The
 * routines physical range in R11 (`0x4de7-0x50ef`, half-open) matches exactly:
 * 0x4DE7 is the entry (callers 0x4D0C/0x4DCE/0x7C3F per flow_main.txt) and
 * 0x50EF is the first byte after the rts, where a *shared* helper starts.  That
 * helper (0x50EF..0x516B, rts at 0x516B) is called from 0x4E9B/0x4EA8 inside
 * this routine but also from outside the family (flow_main edges 0x0508 ->
 * 0x50EF, 0x7C3F/0x7DC3 return paths), so it is NOT registered here: it is a
 * different fragment/owner and stays on gen/interpreter.  Registering beyond
 * the rts would shadow another slice.  Likewise 0x4DE2/0x4DE5 (before the
 * entry) and 0x516C+ (C1 pcm_start, native_allocfree.cpp) are not touched.
 *
 * Semantics: each case is a 1:1 transcription of the ROM decode and mirrors the
 * per-PC body of the generated table (mk2cpp/src/gen/mk2c_r1.cpp, itself a 1:1
 * transcription of src/mcu_opcodes.cpp).  Decode-time constants (addressing
 * mode, size, register numbers, displacements, immediates, branch targets) are
 * folded exactly like h8emit folds them; no semantic "improvement" is applied.
 * The routine role name (per-voice DSP/rate update: reads dp:0xad2a, divides by
 * 0x2EE0, scales by the 0x78EE/0x7AEE tables, updates the @r0+12/22/44/45/
 * 66/68/70/72 and +0x86/+0xA4/+0xA6 voice fields) is INFERRED (confidence I);
 * the instruction-level semantics are confidence C (ROM bytes + GT emitter).
 *
 * Data model: all state stays in page-0 SRAM / device registers and goes
 * through MCU_Read/MCU_Read16/MCU_Write/MCU_Write16 so device routing and the
 * odd-address ADDRESS_ERROR trap stay identical to the stock interpreter.
 * Flags: every helper mirrors the GT body it replaces (MCU_ADD_Common /
 * MCU_SUB_Common / MCU_SetStatusCommon / MCU_SetStatus), including which flag a
 * given instruction leaves untouched (MOVG2/MOVG3 do not write C; ADDX keeps
 * Z sticky via the `if (!Z) clear` replay while SUBX leaves Z alone, matching
 * mcu_opcodes.cpp:1441 vs :1453; MULXU/DIVXU write all four).
 *
 * bsr16 at 0x4E9B/0x4EA8 pushes the stock continuation (0x4E9E/0x4EAB) and
 * jumps to 0x50EF; the helper's rts pops that same return address, so the stack
 * bytes and the -tracepc stream are byte-identical to stock even though the
 * helper itself is executed by gen/interpreter.
 *
 * No symbols from src/gen (a hand-only build must link), GT helpers only.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"
#include "mcu_interrupt.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as pcm_dispatch.cpp / note_chain_tail.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

/* Single-entry build switch (09 5.4): 0 makes this slice not register at all
 * (pure gen/interpreter fallback) without touching other files. */
#define MK2CPP_HAND_DSP_RATE 1

namespace mk2c {
namespace {

/* ---- effective addresses (src/mcu_opcodes.cpp general operand) ------------ */

/* @rN+disp: page from dp (r0-r3), ep (r4/r5), tp (r6/r7); 16-bit wrap. */
uint32_t ind_addr(uint32_t reg, int disp)
{
    uint8_t page = (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
    return ((uint32_t)page << 16) | (uint16_t)(mcu.r[reg] + (uint16_t)disp);
}

/* (dp,disp16): absolute through the DP page register. */
uint32_t dp_addr(uint16_t disp) { return ((uint32_t)mcu.dp << 16) | disp; }

/* ---- status helpers ------------------------------------------------------- */

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG2 (opcode 0x10) memory/register/immediate -> Rd (byte keeps high byte) */
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

void load_imm16(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* MOVG3 (opcode 0x12) Rs -> memory */
void store8(uint32_t addr, uint16_t value)
{
    uint8_t data = (uint8_t)value;
    MCU_Write(addr, data);
    MCU_SetStatusCommon(data, 0);
}

void store16(uint32_t addr, uint16_t value)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, value);
    MCU_SetStatusCommon(value, 1);
}

/* CLR / TST */
void clr16_mem(uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, 0);
    flags_clr();
}
void clr8_reg(uint16_t &reg) { reg = (uint16_t)(reg & 0xff00u); flags_clr(); }
void clr16_reg(uint16_t &reg) { reg = 0; flags_clr(); }

void tst8_reg(uint16_t reg)
{
    MCU_SetStatusCommon((uint32_t)(reg & 0xffu), 0);
    MCU_SetStatus(0, STATUS_C);
}
void tst16_reg(uint16_t reg)
{
    MCU_SetStatusCommon((uint32_t)reg, 1);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVE / MOVI */
void move8(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}
void movi16(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* MOVG2 register-register (byte keeps destination high byte). */
void mov_reg8(uint16_t &dst, uint16_t src)
{
    uint8_t data = (uint8_t)src;
    dst = (uint16_t)((dst & 0xff00u) | data);
    MCU_SetStatusCommon(data, 0);
}

/* ---- ADD/SUB and carry forms --------------------------------------------- */

/* ADD rS rD / SUB rS rD / ADDX rS rD / SUBX rS rD (destination is second). */
void add16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_ADD_Common(dst, src, 0, 1);
}
void sub16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_SUB_Common(dst, src, 0, 1);
}

/* ADDX keeps Z sticky: GT only clears Z, never sets it (mcu_opcodes.cpp:1441).
 * SUBX does NOT touch Z at all (mcu_opcodes.cpp:1453-1466). */
void addx16_imm(uint16_t &reg, uint16_t imm)
{
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    reg = (uint16_t)MCU_ADD_Common(reg, imm, C, 1);
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
}
void subx16_imm(uint16_t &reg, uint16_t imm)
{
    int32_t C = (mcu.sr & STATUS_C) != 0;
    reg = (uint16_t)MCU_SUB_Common(reg, imm, C, 1);
}
void addx8_imm(uint16_t &reg, uint8_t imm)
{
    uint16_t old = reg;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t value = MCU_ADD_Common(old & 0xffu, imm, C, 0);
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)value));
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
}
void subx8_imm(uint16_t &reg, uint8_t imm)
{
    uint16_t old = reg;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t value = MCU_SUB_Common(old & 0xffu, imm, C, 0);
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)value));
}
void addx8_reg(uint16_t &dst, uint16_t src)
{
    addx8_imm(dst, (uint8_t)src);
}
void subx8_reg(uint16_t &dst, uint16_t src)
{
    subx8_imm(dst, (uint8_t)src);
}

/* Memory forms. */
void add16_mem(uint16_t &reg, uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    reg = (uint16_t)MCU_ADD_Common(reg, MCU_Read16(addr), 0, 1);
}
void sub16_mem(uint16_t &reg, uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    reg = (uint16_t)MCU_SUB_Common(reg, MCU_Read16(addr), 0, 1);
}
void addx8_mem(uint16_t &reg, uint32_t addr)
{
    uint16_t old = reg;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t value = MCU_ADD_Common(old & 0xffu, MCU_Read(addr), C, 0);
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)value));
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
}
void subx8_mem(uint16_t &reg, uint32_t addr)
{
    uint16_t old = reg;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t value = MCU_SUB_Common(old & 0xffu, MCU_Read(addr), C, 0);
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)value));
}
/* MOVG_Immediate SUB @addr #imm: flags only, no write (byte, imm zero ext). */
void sub8_mem_imm_flags(uint32_t addr, uint8_t imm)
{
    MCU_SUB_Common(MCU_Read(addr), imm, 0, 0);
}
/* SUB #imm8 rN (byte, imm zero-extended, low byte written). */
void sub8_imm(uint16_t &reg, uint8_t imm)
{
    uint16_t old = reg;
    int32_t value = MCU_SUB_Common(old & 0xffu, imm, 0, 0);
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)value));
}
void sub16_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_SUB_Common(reg, imm, 0, 1);
}
void add16_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_ADD_Common(reg, imm, 0, 1);
}
void addq16(uint16_t &reg, int delta)
{
    reg = (uint16_t)MCU_ADD_Common(reg, delta, 0, 1);
}

/* CMP forms: flags only. */
void cmp8_mem(uint16_t reg, uint32_t addr)
{
    MCU_SUB_Common(reg & 0xffu, MCU_Read(addr), 0, 0);
}
void cmp8_imm(uint16_t reg, uint8_t imm)
{
    MCU_SUB_Common(reg & 0xffu, imm, 0, 0);
}
void cmp16_imm(uint16_t reg, uint16_t imm)
{
    MCU_SUB_Common(reg, imm, 0, 1);
}

/* AND / NEG / NOT / EXTS / SWAP / ROTL / SHLR */
void and8_imm(uint16_t &reg, uint8_t imm)
{
    uint16_t old = reg;
    uint32_t data = (old & 0xffu) & (uint32_t)imm;
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)data));
    MCU_SetStatusCommon(reg, 0);
}
void neg8(uint16_t &reg)
{
    uint16_t old = reg;
    int32_t value = MCU_SUB_Common(0, old & 0xffu, 0, 0);
    reg = (uint16_t)((old & 0xff00u) | ((uint8_t)value));
}
void neg16(uint16_t &reg)
{
    reg = (uint16_t)MCU_SUB_Common(0, reg, 0, 1);
}
void not16(uint16_t &reg)
{
    uint32_t data = ~(uint32_t)reg;
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}
void exts(uint16_t &reg)
{
    uint32_t data = reg;
    reg = (uint16_t)(int8_t)data;
    MCU_SetStatusCommon(data, 1);
}
void swap_reg(uint16_t &reg)
{
    uint32_t data = ((reg & 0xffu) << 8) | (reg >> 8);
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}
void rotl(uint16_t &reg)
{
    uint32_t data = reg;
    uint32_t C = (data & 0x8000u) != 0;
    data = (data << 1) | C;
    reg = (uint16_t)data;
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}
void shlr(uint16_t &reg)
{
    uint32_t data = reg;
    uint32_t C = data & 1u;
    data >>= 1;
    reg = (uint16_t)data;
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

/* MULXU/DIVXU (word forms; src/mcu_opcodes.cpp:1325/1354). */
void mulxu_finish(uint16_t &hi, uint16_t &lo, uint32_t product)
{
    hi = (uint16_t)(product >> 16);
    lo = (uint16_t)product;
    MCU_SetStatus((product & 0x80000000u) != 0, STATUS_N);
    MCU_SetStatus(product == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}
void mulxu16_mem(uint16_t &hi, uint16_t &lo, uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    mulxu_finish(hi, lo, (uint32_t)MCU_Read16(addr) * (uint32_t)hi);
}
void mulxu16_reg(uint16_t &hi, uint16_t &lo, uint16_t src)
{
    mulxu_finish(hi, lo, (uint32_t)src * (uint32_t)hi);
}
void divxu_finish(uint16_t &hi, uint16_t &lo, uint32_t divisor)
{
    if (divisor == 0)
    {
        MCU_ErrorTrap();
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        return;
    }
    uint32_t dividend = ((uint32_t)hi << 16) | (uint32_t)lo;
    uint32_t R = dividend % divisor;
    uint32_t Q = dividend / divisor;
    if (Q > 0xffffu)
    {
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(0, STATUS_Z);
        MCU_SetStatus(1, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    else
    {
        hi = (uint16_t)R;
        lo = (uint16_t)Q;
        MCU_SetStatusCommon(Q, 1);
        MCU_SetStatus(0, STATUS_C);
    }
}
void divxu16_mem(uint16_t &hi, uint16_t &lo, uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    divxu_finish(hi, lo, MCU_Read16(addr));
}
void divxu16_imm(uint16_t &hi, uint16_t &lo, uint16_t imm)
{
    divxu_finish(hi, lo, imm);
}
void divxu16_reg(uint16_t &hi, uint16_t &lo, uint16_t divisor)
{
    divxu_finish(hi, lo, divisor);
}

/* ---- conditional branches (GT MCU_Jump_Bcc, mcu_opcodes.cpp:231) ---------- */

uint16_t beq(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? taken : fall; }
uint16_t bne(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? fall : taken; }
uint16_t bpl(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? fall : taken; }
uint16_t bmi(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? taken : fall; }
uint16_t bcc(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_C) ? fall : taken; }
uint16_t bcs(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_C) ? taken : fall; }
uint16_t bls(uint16_t taken, uint16_t fall)
{
    return (mcu.sr & (STATUS_C | STATUS_Z)) ? taken : fall;
}
uint16_t bge(uint16_t taken, uint16_t fall)
{
    int nv = ((mcu.sr & STATUS_N) != 0) ^ ((mcu.sr & STATUS_V) != 0);
    return (nv == 0) ? taken : fall;
}

/* ======================================================================== */
/* the 0x4DE7 family, 0x4DE7..0x50EE, 302 PCs: one named step per instruction */
/* ======================================================================== */

/* 0x4de7 MOVG2 (dp,0xad2a) r2. */
void step_dsp_rate_movg2_dp_0xad2a_r2(void)
{
    load16(mcu.r[2], dp_addr(0xad2a));
    mcu.pc = 0x4deb;
}

/* 0x4deb ADD @r0+22 r2. */
void step_dsp_rate_add_r0_22_r2(void)
{
    add16_mem(mcu.r[2], ind_addr(0, 22));
    mcu.pc = 0x4dee;
}

/* 0x4dee CLR @r0+22. */
void step_dsp_rate_clr_r0_22(void)
{
    clr16_mem(ind_addr(0, 22));
    mcu.pc = 0x4df1;
}

/* 0x4df1 MULXU @r0+124 r2:r3. */
void step_dsp_rate_mulxu_r0_124_r2_r3(void)
{
    mulxu16_mem(mcu.r[2], mcu.r[3], ind_addr(0, 124));
    mcu.pc = 0x4df4;
}

/* 0x4df4 ADD @r0+12 r3. */
void step_dsp_rate_add_r0_12_r3(void)
{
    add16_mem(mcu.r[3], ind_addr(0, 12));
    mcu.pc = 0x4df7;
}

/* 0x4df7 ADDX #0x0000 r2. */
void step_dsp_rate_addx_0x0000_r2(void)
{
    addx16_imm(mcu.r[2], 0x0000);
    mcu.pc = 0x4dfb;
}

/* 0x4dfb TST r2. */
void step_dsp_rate_tst_r2(void)
{
    tst16_reg(mcu.r[2]);
    mcu.pc = 0x4dfd;
}

/* 0x4dfd BEQ 17 -> 0x4e10. */
void step_dsp_rate_beq_17_to_0x4e10(void)
{
    mcu.pc = beq(0x4e10, 0x4dff);
}

/* 0x4dff SUB #0xffff r3. */
void step_dsp_rate_sub_0xffff_r3(void)
{
    sub16_imm(mcu.r[3], 0xffff);
    mcu.pc = 0x4e03;
}

/* 0x4e03 SUBX #0x0000 r2. */
void step_dsp_rate_subx_0x0000_r2(void)
{
    subx16_imm(mcu.r[2], 0x0000);
    mcu.pc = 0x4e07;
}

/* 0x4e07 DIVXU @r0+124 r2:r3. */
void step_dsp_rate_divxu_r0_124_r2_r3(void)
{
    divxu16_mem(mcu.r[2], mcu.r[3], ind_addr(0, 124));
    mcu.pc = 0x4e0a;
}

/* 0x4e0a MOVG3 r3 -> @r0+22. */
void step_dsp_rate_movg3_r3_to_r0_22(void)
{
    store16(ind_addr(0, 22), mcu.r[3]);
    mcu.pc = 0x4e0d;
}

/* 0x4e0d movi r3 #0xffff. */
void step_dsp_rate_movi_r3_0xffff(void)
{
    movi16(mcu.r[3], 0xffff);
    mcu.pc = 0x4e10;
}

/* 0x4e10 MOVG3 r3 -> @r0+12. */
void step_dsp_rate_movg3_r3_to_r0_12(void)
{
    store16(ind_addr(0, 12), mcu.r[3]);
    mcu.pc = 0x4e13;
}

/* 0x4e13 SUB @r0+-3 #0x00. */
void step_dsp_rate_sub_r0_neg_3_0x00(void)
{
    sub8_mem_imm_flags(ind_addr(0, -3), 0x00);
    mcu.pc = 0x4e17;
}

/* 0x4e17 BNE 30 -> 0x4e37. */
void step_dsp_rate_bne_30_to_0x4e37(void)
{
    mcu.pc = bne(0x4e37, 0x4e19);
}

/* 0x4e19 MOVG2 @r0+114 r4. */
void step_dsp_rate_movg2_r0_114_r4(void)
{
    load16(mcu.r[4], ind_addr(0, 114));
    mcu.pc = 0x4e1c;
}

/* 0x4e1c SUB @r0+112 r4. */
void step_dsp_rate_sub_r0_112_r4(void)
{
    sub16_mem(mcu.r[4], ind_addr(0, 112));
    mcu.pc = 0x4e1f;
}

/* 0x4e1f MULXU r3 r4:r5. */
void step_dsp_rate_mulxu_r3_r4_r5(void)
{
    mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[3]);
    mcu.pc = 0x4e21;
}

/* 0x4e21 CLR r3. */
void step_dsp_rate_clr_r3(void)
{
    clr16_reg(mcu.r[3]);
    mcu.pc = 0x4e23;
}

/* 0x4e23 ADD @r0+112 r4. */
void step_dsp_rate_add_r0_112_r4(void)
{
    add16_mem(mcu.r[4], ind_addr(0, 112));
    mcu.pc = 0x4e26;
}

/* 0x4e26 ADDX @r0+106 r3. */
void step_dsp_rate_addx_r0_106_r3(void)
{
    addx8_mem(mcu.r[3], ind_addr(0, 106));
    mcu.pc = 0x4e29;
}

/* 0x4e29 MOVG3 r3 -> @r0+44. */
void step_dsp_rate_movg3_r3_to_r0_44(void)
{
    store8(ind_addr(0, 44), mcu.r[3]);
    mcu.pc = 0x4e2c;
}

/* 0x4e2c MOVG3 r4 -> @r0+68. */
void step_dsp_rate_movg3_r4_to_r0_68(void)
{
    store16(ind_addr(0, 68), mcu.r[4]);
    mcu.pc = 0x4e2f;
}

/* 0x4e2f MOVG3 r3 -> @r0+45. */
void step_dsp_rate_movg3_r3_to_r0_45(void)
{
    store8(ind_addr(0, 45), mcu.r[3]);
    mcu.pc = 0x4e32;
}

/* 0x4e32 MOVG3 r4 -> @r0+70. */
void step_dsp_rate_movg3_r4_to_r0_70(void)
{
    store16(ind_addr(0, 70), mcu.r[4]);
    mcu.pc = 0x4e35;
}

/* 0x4e35 BRA 31 -> 0x4e56. */
void step_dsp_rate_bra_31_to_0x4e56(void)
{
    mcu.pc = 0x4e56;
}

/* 0x4e37 MOVG2 @r0+112 r4. */
void step_dsp_rate_movg2_r0_112_r4(void)
{
    load16(mcu.r[4], ind_addr(0, 112));
    mcu.pc = 0x4e3a;
}

/* 0x4e3a SUB @r0+114 r4. */
void step_dsp_rate_sub_r0_114_r4(void)
{
    sub16_mem(mcu.r[4], ind_addr(0, 114));
    mcu.pc = 0x4e3d;
}

/* 0x4e3d MULXU r3 r4:r5. */
void step_dsp_rate_mulxu_r3_r4_r5_a(void)
{
    mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[3]);
    mcu.pc = 0x4e3f;
}

/* 0x4e3f MOVG2 @r0+112 r6. */
void step_dsp_rate_movg2_r0_112_r6(void)
{
    load16(mcu.r[6], ind_addr(0, 112));
    mcu.pc = 0x4e42;
}

/* 0x4e42 MOVG2 @r0+106 r5. */
void step_dsp_rate_movg2_r0_106_r5(void)
{
    load8(mcu.r[5], ind_addr(0, 106));
    mcu.pc = 0x4e45;
}

/* 0x4e45 SUB r4 r6. */
void step_dsp_rate_sub_r4_r6(void)
{
    sub16_reg(mcu.r[6], mcu.r[4]);
    mcu.pc = 0x4e47;
}

/* 0x4e47 SUBX #0x00 r5. */
void step_dsp_rate_subx_0x00_r5(void)
{
    subx8_imm(mcu.r[5], 0x00);
    mcu.pc = 0x4e4a;
}

/* 0x4e4a MOVG3 r5 -> @r0+44. */
void step_dsp_rate_movg3_r5_to_r0_44(void)
{
    store8(ind_addr(0, 44), mcu.r[5]);
    mcu.pc = 0x4e4d;
}

/* 0x4e4d MOVG3 r6 -> @r0+68. */
void step_dsp_rate_movg3_r6_to_r0_68(void)
{
    store16(ind_addr(0, 68), mcu.r[6]);
    mcu.pc = 0x4e50;
}

/* 0x4e50 MOVG3 r5 -> @r0+45. */
void step_dsp_rate_movg3_r5_to_r0_45(void)
{
    store8(ind_addr(0, 45), mcu.r[5]);
    mcu.pc = 0x4e53;
}

/* 0x4e53 MOVG3 r6 -> @r0+70. */
void step_dsp_rate_movg3_r6_to_r0_70(void)
{
    store16(ind_addr(0, 70), mcu.r[6]);
    mcu.pc = 0x4e56;
}

/* 0x4e56 MOVG2 @r0+45 r5. */
void step_dsp_rate_movg2_r0_45_r5(void)
{
    load8(mcu.r[5], ind_addr(0, 45));
    mcu.pc = 0x4e59;
}

/* 0x4e59 MOVG2 @r0+70 r6. */
void step_dsp_rate_movg2_r0_70_r6(void)
{
    load16(mcu.r[6], ind_addr(0, 70));
    mcu.pc = 0x4e5c;
}

/* 0x4e5c MOVG2 @r0+0x0086 r2. */
void step_dsp_rate_movg2_r0_0x0086_r2(void)
{
    load16(mcu.r[2], ind_addr(0, 134));
    mcu.pc = 0x4e60;
}

/* 0x4e60 BPL 15 -> 0x4e71. */
void step_dsp_rate_bpl_15_to_0x4e71(void)
{
    mcu.pc = bpl(0x4e71, 0x4e62);
}

/* 0x4e62 NEG r2. */
void step_dsp_rate_neg_r2(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x4e64;
}

/* 0x4e64 SUB r2 r6. */
void step_dsp_rate_sub_r2_r6(void)
{
    sub16_reg(mcu.r[6], mcu.r[2]);
    mcu.pc = 0x4e66;
}

/* 0x4e66 SUBX #0x00 r5. */
void step_dsp_rate_subx_0x00_r5_a(void)
{
    subx8_imm(mcu.r[5], 0x00);
    mcu.pc = 0x4e69;
}

/* 0x4e69 BCC 32 -> 0x4e8b. */
void step_dsp_rate_bcc_32_to_0x4e8b(void)
{
    mcu.pc = bcc(0x4e8b, 0x4e6b);
}

/* 0x4e6b CLR r6. */
void step_dsp_rate_clr_r6(void)
{
    clr16_reg(mcu.r[6]);
    mcu.pc = 0x4e6d;
}

/* 0x4e6d CLR r5. */
void step_dsp_rate_clr_r5(void)
{
    clr8_reg(mcu.r[5]);
    mcu.pc = 0x4e6f;
}

/* 0x4e6f BRA 26 -> 0x4e8b. */
void step_dsp_rate_bra_26_to_0x4e8b(void)
{
    mcu.pc = 0x4e8b;
}

/* 0x4e71 ADD r2 r6. */
void step_dsp_rate_add_r2_r6(void)
{
    add16_reg(mcu.r[6], mcu.r[2]);
    mcu.pc = 0x4e73;
}

/* 0x4e73 ADDX #0x00 r5. */
void step_dsp_rate_addx_0x00_r5(void)
{
    addx8_imm(mcu.r[5], 0x00);
    mcu.pc = 0x4e76;
}

/* 0x4e76 cmp r5,b #0x01. */
void step_dsp_rate_cmp_r5_b_0x01(void)
{
    cmp8_imm(mcu.r[5], 0x01);
    mcu.pc = 0x4e78;
}

/* 0x4e78 BEQ 9 -> 0x4e83. */
void step_dsp_rate_beq_9_to_0x4e83(void)
{
    mcu.pc = beq(0x4e83, 0x4e7a);
}

/* 0x4e7a BCS 15 -> 0x4e8b. */
void step_dsp_rate_bcs_15_to_0x4e8b(void)
{
    mcu.pc = bcs(0x4e8b, 0x4e7c);
}

/* 0x4e7c movi r6 #0xf018. */
void step_dsp_rate_movi_r6_0xf018(void)
{
    movi16(mcu.r[6], 0xf018);
    mcu.pc = 0x4e7f;
}

/* 0x4e7f move r5 #0x01. */
void step_dsp_rate_move_r5_0x01(void)
{
    move8(mcu.r[5], 0x01);
    mcu.pc = 0x4e81;
}

/* 0x4e81 BRA 8 -> 0x4e8b. */
void step_dsp_rate_bra_8_to_0x4e8b(void)
{
    mcu.pc = 0x4e8b;
}

/* 0x4e83 cmp r6,w #0xf018. */
void step_dsp_rate_cmp_r6_w_0xf018(void)
{
    cmp16_imm(mcu.r[6], 0xf018);
    mcu.pc = 0x4e86;
}

/* 0x4e86 BLS 3 -> 0x4e8b. */
void step_dsp_rate_bls_3_to_0x4e8b(void)
{
    mcu.pc = bls(0x4e8b, 0x4e88);
}

/* 0x4e88 movi r6 #0xf018. */
void step_dsp_rate_movi_r6_0xf018_a(void)
{
    movi16(mcu.r[6], 0xf018);
    mcu.pc = 0x4e8b;
}

/* 0x4e8b MOVG3 r5 -> @r0+45. */
void step_dsp_rate_movg3_r5_to_r0_45_a(void)
{
    store8(ind_addr(0, 45), mcu.r[5]);
    mcu.pc = 0x4e8e;
}

/* 0x4e8e MOVG3 r6 -> @r0+70. */
void step_dsp_rate_movg3_r6_to_r0_70_a(void)
{
    store16(ind_addr(0, 70), mcu.r[6]);
    mcu.pc = 0x4e91;
}

/* 0x4e91 MOVG2 @r0+-118 r2. */
void step_dsp_rate_movg2_r0_neg_118_r2(void)
{
    load16(mcu.r[2], ind_addr(0, -118));
    mcu.pc = 0x4e94;
}

/* 0x4e94 MOVG2 @r0+0x0090 r3. */
void step_dsp_rate_movg2_r0_0x0090_r3(void)
{
    load16(mcu.r[3], ind_addr(0, 144));
    mcu.pc = 0x4e98;
}

/* 0x4e98 MOVG2 @r0+-96 r6. */
void step_dsp_rate_movg2_r0_neg_96_r6(void)
{
    load16(mcu.r[6], ind_addr(0, -96));
    mcu.pc = 0x4e9b;
}

/* 0x4e9b bsr16 -> 0x50ef. */
void step_dsp_rate_bsr16_to_0x50ef(void)
{
    MCU_PushStack(0x4e9e);
    mcu.pc = 0x50ef;
}

/* 0x4e9e MOVG2 @r0+-84 r2. */
void step_dsp_rate_movg2_r0_neg_84_r2(void)
{
    load16(mcu.r[2], ind_addr(0, -84));
    mcu.pc = 0x4ea1;
}

/* 0x4ea1 MOVG2 @r0+0x0092 r3. */
void step_dsp_rate_movg2_r0_0x0092_r3(void)
{
    load16(mcu.r[3], ind_addr(0, 146));
    mcu.pc = 0x4ea5;
}

/* 0x4ea5 MOVG2 @r0+-62 r6. */
void step_dsp_rate_movg2_r0_neg_62_r6(void)
{
    load16(mcu.r[6], ind_addr(0, -62));
    mcu.pc = 0x4ea8;
}

/* 0x4ea8 bsr16 -> 0x50ef. */
void step_dsp_rate_bsr16_to_0x50ef_a(void)
{
    MCU_PushStack(0x4eab);
    mcu.pc = 0x50ef;
}

/* 0x4eab MOVG2 @r0+45 r5. */
void step_dsp_rate_movg2_r0_45_r5_a(void)
{
    load8(mcu.r[5], ind_addr(0, 45));
    mcu.pc = 0x4eae;
}

/* 0x4eae MOVG2 @r0+70 r6. */
void step_dsp_rate_movg2_r0_70_r6_a(void)
{
    load16(mcu.r[6], ind_addr(0, 70));
    mcu.pc = 0x4eb1;
}

/* 0x4eb1 MOVG2 (dp,0x8000) r3. */
void step_dsp_rate_movg2_dp_0x8000_r3(void)
{
    load16(mcu.r[3], dp_addr(0x8000));
    mcu.pc = 0x4eb5;
}

/* 0x4eb5 SUB #0x0400 r3. */
void step_dsp_rate_sub_0x0400_r3(void)
{
    sub16_imm(mcu.r[3], 0x0400);
    mcu.pc = 0x4eb9;
}

/* 0x4eb9 BMI 7 -> 0x4ec2. */
void step_dsp_rate_bmi_7_to_0x4ec2(void)
{
    mcu.pc = bmi(0x4ec2, 0x4ebb);
}

/* 0x4ebb ADD r3 r6. */
void step_dsp_rate_add_r3_r6(void)
{
    add16_reg(mcu.r[6], mcu.r[3]);
    mcu.pc = 0x4ebd;
}

/* 0x4ebd ADDX #0x00 r5. */
void step_dsp_rate_addx_0x00_r5_a(void)
{
    addx8_imm(mcu.r[5], 0x00);
    mcu.pc = 0x4ec0;
}

/* 0x4ec0 BRA 15 -> 0x4ed1. */
void step_dsp_rate_bra_15_to_0x4ed1(void)
{
    mcu.pc = 0x4ed1;
}

/* 0x4ec2 NEG r3. */
void step_dsp_rate_neg_r3(void)
{
    neg16(mcu.r[3]);
    mcu.pc = 0x4ec4;
}

/* 0x4ec4 SUB r3 r6. */
void step_dsp_rate_sub_r3_r6(void)
{
    sub16_reg(mcu.r[6], mcu.r[3]);
    mcu.pc = 0x4ec6;
}

/* 0x4ec6 SUBX #0x00 r5. */
void step_dsp_rate_subx_0x00_r5_b(void)
{
    subx8_imm(mcu.r[5], 0x00);
    mcu.pc = 0x4ec9;
}

/* 0x4ec9 TST r5. */
void step_dsp_rate_tst_r5(void)
{
    tst8_reg(mcu.r[5]);
    mcu.pc = 0x4ecb;
}

/* 0x4ecb BPL 4 -> 0x4ed1. */
void step_dsp_rate_bpl_4_to_0x4ed1(void)
{
    mcu.pc = bpl(0x4ed1, 0x4ecd);
}

/* 0x4ecd CLR r6. */
void step_dsp_rate_clr_r6_a(void)
{
    clr16_reg(mcu.r[6]);
    mcu.pc = 0x4ecf;
}

/* 0x4ecf CLR r5. */
void step_dsp_rate_clr_r5_a(void)
{
    clr8_reg(mcu.r[5]);
    mcu.pc = 0x4ed1;
}

/* 0x4ed1 MOVG2 @r0+-2 r3. */
void step_dsp_rate_movg2_r0_neg_2_r3(void)
{
    load16(mcu.r[3], ind_addr(0, -2));
    mcu.pc = 0x4ed4;
}

/* 0x4ed4 MOVG2 @r3+0xce78 r3. */
void step_dsp_rate_movg2_r3_0xce78_r3(void)
{
    load8(mcu.r[3], ind_addr(3, 52856));
    mcu.pc = 0x4ed8;
}

/* 0x4ed8 ADD r3 r3. */
void step_dsp_rate_add_r3_r3(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x4eda;
}

/* 0x4eda MOVG2 @r3+0xac4e r3. */
void step_dsp_rate_movg2_r3_0xac4e_r3(void)
{
    load16(mcu.r[3], ind_addr(3, 44110));
    mcu.pc = 0x4ede;
}

/* 0x4ede BMI 7 -> 0x4ee7. */
void step_dsp_rate_bmi_7_to_0x4ee7(void)
{
    mcu.pc = bmi(0x4ee7, 0x4ee0);
}

/* 0x4ee0 ADD r3 r6. */
void step_dsp_rate_add_r3_r6_a(void)
{
    add16_reg(mcu.r[6], mcu.r[3]);
    mcu.pc = 0x4ee2;
}

/* 0x4ee2 ADDX #0x00 r5. */
void step_dsp_rate_addx_0x00_r5_b(void)
{
    addx8_imm(mcu.r[5], 0x00);
    mcu.pc = 0x4ee5;
}

/* 0x4ee5 BRA 15 -> 0x4ef6. */
void step_dsp_rate_bra_15_to_0x4ef6(void)
{
    mcu.pc = 0x4ef6;
}

/* 0x4ee7 NEG r3. */
void step_dsp_rate_neg_r3_a(void)
{
    neg16(mcu.r[3]);
    mcu.pc = 0x4ee9;
}

/* 0x4ee9 SUB r3 r6. */
void step_dsp_rate_sub_r3_r6_a(void)
{
    sub16_reg(mcu.r[6], mcu.r[3]);
    mcu.pc = 0x4eeb;
}

/* 0x4eeb SUBX #0x00 r5. */
void step_dsp_rate_subx_0x00_r5_c(void)
{
    subx8_imm(mcu.r[5], 0x00);
    mcu.pc = 0x4eee;
}

/* 0x4eee TST r5. */
void step_dsp_rate_tst_r5_a(void)
{
    tst8_reg(mcu.r[5]);
    mcu.pc = 0x4ef0;
}

/* 0x4ef0 BPL 4 -> 0x4ef6. */
void step_dsp_rate_bpl_4_to_0x4ef6(void)
{
    mcu.pc = bpl(0x4ef6, 0x4ef2);
}

/* 0x4ef2 CLR r6. */
void step_dsp_rate_clr_r6_b(void)
{
    clr16_reg(mcu.r[6]);
    mcu.pc = 0x4ef4;
}

/* 0x4ef4 CLR r5. */
void step_dsp_rate_clr_r5_b(void)
{
    clr8_reg(mcu.r[5]);
    mcu.pc = 0x4ef6;
}

/* 0x4ef6 MOVG3 r5 -> @r0+45. */
void step_dsp_rate_movg3_r5_to_r0_45_b(void)
{
    store8(ind_addr(0, 45), mcu.r[5]);
    mcu.pc = 0x4ef9;
}

/* 0x4ef9 MOVG3 r6 -> @r0+70. */
void step_dsp_rate_movg3_r6_to_r0_70_b(void)
{
    store16(ind_addr(0, 70), mcu.r[6]);
    mcu.pc = 0x4efc;
}

/* 0x4efc MOVG2 @r0+66 r5. */
void step_dsp_rate_movg2_r0_66_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 66));
    mcu.pc = 0x4eff;
}

/* 0x4eff MOVG2 @r0+43 r4. */
void step_dsp_rate_movg2_r0_43_r4(void)
{
    load8(mcu.r[4], ind_addr(0, 43));
    mcu.pc = 0x4f02;
}

/* 0x4f02 BNE 14 -> 0x4f12. */
void step_dsp_rate_bne_14_to_0x4f12(void)
{
    mcu.pc = bne(0x4f12, 0x4f04);
}

/* 0x4f04 TST r5. */
void step_dsp_rate_tst_r5_b(void)
{
    tst16_reg(mcu.r[5]);
    mcu.pc = 0x4f06;
}

/* 0x4f06 BNE 10 -> 0x4f12. */
void step_dsp_rate_bne_10_to_0x4f12(void)
{
    mcu.pc = bne(0x4f12, 0x4f08);
}

/* 0x4f08 CLR r2. */
void step_dsp_rate_clr_r2(void)
{
    clr16_reg(mcu.r[2]);
    mcu.pc = 0x4f0a;
}

/* 0x4f0a MOVG2 @r0+45 r2. */
void step_dsp_rate_movg2_r0_45_r2(void)
{
    load8(mcu.r[2], ind_addr(0, 45));
    mcu.pc = 0x4f0d;
}

/* 0x4f0d MOVG2 @r0+70 r3. */
void step_dsp_rate_movg2_r0_70_r3(void)
{
    load16(mcu.r[3], ind_addr(0, 70));
    mcu.pc = 0x4f10;
}

/* 0x4f10 BRA 92 -> 0x4f6e. */
void step_dsp_rate_bra_92_to_0x4f6e(void)
{
    mcu.pc = 0x4f6e;
}

/* 0x4f12 MOVG2 @r0+-2 r1. */
void step_dsp_rate_movg2_r0_neg_2_r1(void)
{
    load16(mcu.r[1], ind_addr(0, -2));
    mcu.pc = 0x4f15;
}

/* 0x4f15 CLR r3. */
void step_dsp_rate_clr_r3_a(void)
{
    clr16_reg(mcu.r[3]);
    mcu.pc = 0x4f17;
}

/* 0x4f17 MOVG2 @r0+-58 r2. */
void step_dsp_rate_movg2_r0_neg_58_r2(void)
{
    load16(mcu.r[2], ind_addr(0, -58));
    mcu.pc = 0x4f1a;
}

/* 0x4f1a MOVG2 @r2 r3. */
void step_dsp_rate_movg2_r2_r3(void)
{
    load8(mcu.r[3], ind_addr(2, 0));
    mcu.pc = 0x4f1c;
}

/* 0x4f1c ADD r3 r3. */
void step_dsp_rate_add_r3_r3_a(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x4f1e;
}

/* 0x4f1e MOVG2 @r3+0x77a6 r6. */
void step_dsp_rate_movg2_r3_0x77a6_r6(void)
{
    load16(mcu.r[6], ind_addr(3, 30630));
    mcu.pc = 0x4f22;
}

/* 0x4f22 MOVG2 (dp,0xad2a) r2. */
void step_dsp_rate_movg2_dp_0xad2a_r2_a(void)
{
    load16(mcu.r[2], dp_addr(0xad2a));
    mcu.pc = 0x4f26;
}

/* 0x4f26 MULXU r6 r2:r3. */
void step_dsp_rate_mulxu_r6_r2_r3(void)
{
    mulxu16_reg(mcu.r[2], mcu.r[3], mcu.r[6]);
    mcu.pc = 0x4f28;
}

/* 0x4f28 TST r4. */
void step_dsp_rate_tst_r4(void)
{
    tst8_reg(mcu.r[4]);
    mcu.pc = 0x4f2a;
}

/* 0x4f2a BPL 30 -> 0x4f4a. */
void step_dsp_rate_bpl_30_to_0x4f4a(void)
{
    mcu.pc = bpl(0x4f4a, 0x4f2c);
}

/* 0x4f2c NEG r4. */
void step_dsp_rate_neg_r4(void)
{
    neg8(mcu.r[4]);
    mcu.pc = 0x4f2e;
}

/* 0x4f2e NEG r5. */
void step_dsp_rate_neg_r5(void)
{
    neg16(mcu.r[5]);
    mcu.pc = 0x4f30;
}

/* 0x4f30 SUBX #0x00 r4. */
void step_dsp_rate_subx_0x00_r4(void)
{
    subx8_imm(mcu.r[4], 0x00);
    mcu.pc = 0x4f33;
}

/* 0x4f33 SUB r3 r5. */
void step_dsp_rate_sub_r3_r5(void)
{
    sub16_reg(mcu.r[5], mcu.r[3]);
    mcu.pc = 0x4f35;
}

/* 0x4f35 SUBX r2 r4. */
void step_dsp_rate_subx_r2_r4(void)
{
    subx8_reg(mcu.r[4], mcu.r[2]);
    mcu.pc = 0x4f37;
}

/* 0x4f37 TST r4. */
void step_dsp_rate_tst_r4_a(void)
{
    tst8_reg(mcu.r[4]);
    mcu.pc = 0x4f39;
}

/* 0x4f39 BPL 6 -> 0x4f41. */
void step_dsp_rate_bpl_6_to_0x4f41(void)
{
    mcu.pc = bpl(0x4f41, 0x4f3b);
}

/* 0x4f3b CLR r4. */
void step_dsp_rate_clr_r4(void)
{
    clr8_reg(mcu.r[4]);
    mcu.pc = 0x4f3d;
}

/* 0x4f3d CLR r5. */
void step_dsp_rate_clr_r5_c(void)
{
    clr16_reg(mcu.r[5]);
    mcu.pc = 0x4f3f;
}

/* 0x4f3f BRA 21 -> 0x4f56. */
void step_dsp_rate_bra_21_to_0x4f56(void)
{
    mcu.pc = 0x4f56;
}

/* 0x4f41 NEG r4. */
void step_dsp_rate_neg_r4_a(void)
{
    neg8(mcu.r[4]);
    mcu.pc = 0x4f43;
}

/* 0x4f43 NEG r5. */
void step_dsp_rate_neg_r5_a(void)
{
    neg16(mcu.r[5]);
    mcu.pc = 0x4f45;
}

/* 0x4f45 SUBX #0x00 r4. */
void step_dsp_rate_subx_0x00_r4_a(void)
{
    subx8_imm(mcu.r[4], 0x00);
    mcu.pc = 0x4f48;
}

/* 0x4f48 BRA 12 -> 0x4f56. */
void step_dsp_rate_bra_12_to_0x4f56(void)
{
    mcu.pc = 0x4f56;
}

/* 0x4f4a SUB r3 r5. */
void step_dsp_rate_sub_r3_r5_a(void)
{
    sub16_reg(mcu.r[5], mcu.r[3]);
    mcu.pc = 0x4f4c;
}

/* 0x4f4c SUBX r2 r4. */
void step_dsp_rate_subx_r2_r4_a(void)
{
    subx8_reg(mcu.r[4], mcu.r[2]);
    mcu.pc = 0x4f4e;
}

/* 0x4f4e TST r4. */
void step_dsp_rate_tst_r4_b(void)
{
    tst8_reg(mcu.r[4]);
    mcu.pc = 0x4f50;
}

/* 0x4f50 BPL 4 -> 0x4f56. */
void step_dsp_rate_bpl_4_to_0x4f56(void)
{
    mcu.pc = bpl(0x4f56, 0x4f52);
}

/* 0x4f52 CLR r4. */
void step_dsp_rate_clr_r4_a(void)
{
    clr8_reg(mcu.r[4]);
    mcu.pc = 0x4f54;
}

/* 0x4f54 CLR r5. */
void step_dsp_rate_clr_r5_d(void)
{
    clr16_reg(mcu.r[5]);
    mcu.pc = 0x4f56;
}

/* 0x4f56 MOVG3 r5 -> @r0+66. */
void step_dsp_rate_movg3_r5_to_r0_66(void)
{
    store16(ind_addr(0, 66), mcu.r[5]);
    mcu.pc = 0x4f59;
}

/* 0x4f59 MOVG3 r4 -> @r0+43. */
void step_dsp_rate_movg3_r4_to_r0_43(void)
{
    store8(ind_addr(0, 43), mcu.r[4]);
    mcu.pc = 0x4f5c;
}

/* 0x4f5c CLR r2. */
void step_dsp_rate_clr_r2_a(void)
{
    clr16_reg(mcu.r[2]);
    mcu.pc = 0x4f5e;
}

/* 0x4f5e MOVG2 @r0+70 r3. */
void step_dsp_rate_movg2_r0_70_r3_a(void)
{
    load16(mcu.r[3], ind_addr(0, 70));
    mcu.pc = 0x4f61;
}

/* 0x4f61 MOVG2 @r0+45 r2. */
void step_dsp_rate_movg2_r0_45_r2_a(void)
{
    load8(mcu.r[2], ind_addr(0, 45));
    mcu.pc = 0x4f64;
}

/* 0x4f64 ADD r5 r3. */
void step_dsp_rate_add_r5_r3(void)
{
    add16_reg(mcu.r[3], mcu.r[5]);
    mcu.pc = 0x4f66;
}

/* 0x4f66 ADDX r4 r2. */
void step_dsp_rate_addx_r4_r2(void)
{
    addx8_reg(mcu.r[2], mcu.r[4]);
    mcu.pc = 0x4f68;
}

/* 0x4f68 MOVG3 r2 -> @r0+45. */
void step_dsp_rate_movg3_r2_to_r0_45(void)
{
    store8(ind_addr(0, 45), mcu.r[2]);
    mcu.pc = 0x4f6b;
}

/* 0x4f6b MOVG3 r3 -> @r0+70. */
void step_dsp_rate_movg3_r3_to_r0_70(void)
{
    store16(ind_addr(0, 70), mcu.r[3]);
    mcu.pc = 0x4f6e;
}

/* 0x4f6e SUB @r0+62 r3. */
void step_dsp_rate_sub_r0_62_r3(void)
{
    sub16_mem(mcu.r[3], ind_addr(0, 62));
    mcu.pc = 0x4f71;
}

/* 0x4f71 SUBX @r0+41 r2. */
void step_dsp_rate_subx_r0_41_r2(void)
{
    subx8_mem(mcu.r[2], ind_addr(0, 41));
    mcu.pc = 0x4f74;
}

/* 0x4f74 SUB #0x2ee0 r3. */
void step_dsp_rate_sub_0x2ee0_r3(void)
{
    sub16_imm(mcu.r[3], 0x2ee0);
    mcu.pc = 0x4f78;
}

/* 0x4f78 SUBX #0x00 r2. */
void step_dsp_rate_subx_0x00_r2(void)
{
    subx8_imm(mcu.r[2], 0x00);
    mcu.pc = 0x4f7b;
}

/* 0x4f7b BPL 79 -> 0x4fcc. */
void step_dsp_rate_bpl_79_to_0x4fcc(void)
{
    mcu.pc = bpl(0x4fcc, 0x4f7d);
}

/* 0x4f7d EXTS r2. */
void step_dsp_rate_exts_r2(void)
{
    exts(mcu.r[2]);
    mcu.pc = 0x4f7f;
}

/* 0x4f7f NOT r2. */
void step_dsp_rate_not_r2(void)
{
    not16(mcu.r[2]);
    mcu.pc = 0x4f81;
}

/* 0x4f81 NOT r3. */
void step_dsp_rate_not_r3(void)
{
    not16(mcu.r[3]);
    mcu.pc = 0x4f83;
}

/* 0x4f83 ADDQ #1 r3. */
void step_dsp_rate_addq_1_r3(void)
{
    addq16(mcu.r[3], 1);
    mcu.pc = 0x4f85;
}

/* 0x4f85 ADDX #0x0000 r2. */
void step_dsp_rate_addx_0x0000_r2_a(void)
{
    addx16_imm(mcu.r[2], 0x0000);
    mcu.pc = 0x4f89;
}

/* 0x4f89 DIVXU #0x2ee0 r2:r3. */
void step_dsp_rate_divxu_0x2ee0_r2_r3(void)
{
    divxu16_imm(mcu.r[2], mcu.r[3], 0x2ee0);
    mcu.pc = 0x4f8d;
}

/* 0x4f8d cmp r2,w #0x0000. */
void step_dsp_rate_cmp_r2_w_0x0000(void)
{
    cmp16_imm(mcu.r[2], 0x0000);
    mcu.pc = 0x4f90;
}

/* 0x4f90 BEQ 8 -> 0x4f9a. */
void step_dsp_rate_beq_8_to_0x4f9a(void)
{
    mcu.pc = beq(0x4f9a, 0x4f92);
}

/* 0x4f92 ADDQ #1 r3. */
void step_dsp_rate_addq_1_r3_a(void)
{
    addq16(mcu.r[3], 1);
    mcu.pc = 0x4f94;
}

/* 0x4f94 NEG r2. */
void step_dsp_rate_neg_r2_a(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x4f96;
}

/* 0x4f96 ADD #0x2ee0 r2. */
void step_dsp_rate_add_0x2ee0_r2(void)
{
    add16_imm(mcu.r[2], 0x2ee0);
    mcu.pc = 0x4f9a;
}

/* 0x4f9a CLR r1. */
void step_dsp_rate_clr_r1(void)
{
    clr16_reg(mcu.r[1]);
    mcu.pc = 0x4f9c;
}

/* 0x4f9c MOVG2 r2 r1. */
void step_dsp_rate_movg2_r2_r1(void)
{
    mov_reg8(mcu.r[1], mcu.r[2]);
    mcu.pc = 0x4f9e;
}

/* 0x4f9e ADD r1 r1. */
void step_dsp_rate_add_r1_r1(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x4fa0;
}

/* 0x4fa0 MOVG2 @r1+0x78ee r4. */
void step_dsp_rate_movg2_r1_0x78ee_r4(void)
{
    load16(mcu.r[4], ind_addr(1, 30958));
    mcu.pc = 0x4fa4;
}

/* 0x4fa4 CLR r1. */
void step_dsp_rate_clr_r1_a(void)
{
    clr16_reg(mcu.r[1]);
    mcu.pc = 0x4fa6;
}

/* 0x4fa6 SWAP r2. */
void step_dsp_rate_swap_r2(void)
{
    swap_reg(mcu.r[2]);
    mcu.pc = 0x4fa8;
}

/* 0x4fa8 MOVG2 r2 r1. */
void step_dsp_rate_movg2_r2_r1_a(void)
{
    mov_reg8(mcu.r[1], mcu.r[2]);
    mcu.pc = 0x4faa;
}

/* 0x4faa ADD r1 r1. */
void step_dsp_rate_add_r1_r1_a(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x4fac;
}

/* 0x4fac MOVG2 @r1+0x7aee r1. */
void step_dsp_rate_movg2_r1_0x7aee_r1(void)
{
    load16(mcu.r[1], ind_addr(1, 31470));
    mcu.pc = 0x4fb0;
}

/* 0x4fb0 MULXU r1 r4:r5. */
void step_dsp_rate_mulxu_r1_r4_r5(void)
{
    mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[1]);
    mcu.pc = 0x4fb2;
}

/* 0x4fb2 ROTL r4. */
void step_dsp_rate_rotl_r4(void)
{
    rotl(mcu.r[4]);
    mcu.pc = 0x4fb4;
}

/* 0x4fb4 ROTL r4. */
void step_dsp_rate_rotl_r4_a(void)
{
    rotl(mcu.r[4]);
    mcu.pc = 0x4fb6;
}

/* 0x4fb6 AND #0x03 r4. */
void step_dsp_rate_and_0x03_r4(void)
{
    and8_imm(mcu.r[4], 0x03);
    mcu.pc = 0x4fb9;
}

/* 0x4fb9 SWAP r4. */
void step_dsp_rate_swap_r4(void)
{
    swap_reg(mcu.r[4]);
    mcu.pc = 0x4fbb;
}

/* 0x4fbb ADD r1 r4. */
void step_dsp_rate_add_r1_r4(void)
{
    add16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x4fbd;
}

/* 0x4fbd TST r3. */
void step_dsp_rate_tst_r3(void)
{
    tst16_reg(mcu.r[3]);
    mcu.pc = 0x4fbf;
}

/* 0x4fbf BEQ 62 -> 0x4fff. */
void step_dsp_rate_beq_62_to_0x4fff(void)
{
    mcu.pc = beq(0x4fff, 0x4fc1);
}

/* 0x4fc1 SUB #0x0001 r3. */
void step_dsp_rate_sub_0x0001_r3(void)
{
    sub16_imm(mcu.r[3], 0x0001);
    mcu.pc = 0x4fc5;
}

/* 0x4fc5 SHLR r4. */
void step_dsp_rate_shlr_r4(void)
{
    shlr(mcu.r[4]);
    mcu.pc = 0x4fc7;
}

/* 0x4fc7 cntjmp r3 -5 -> 0x4fc5. */
void step_dsp_rate_cntjmp_r3_5_to_0x4fc5(void)
{
    mcu.r[3] = (uint16_t)(mcu.r[3] - 1);
    mcu.pc = (mcu.r[3] != 0xffff) ? 0x4fc5 : 0x4fca;
}

/* 0x4fca BRA 51 -> 0x4fff. */
void step_dsp_rate_bra_51_to_0x4fff(void)
{
    mcu.pc = 0x4fff;
}

/* 0x4fcc EXTS r2. */
void step_dsp_rate_exts_r2_a(void)
{
    exts(mcu.r[2]);
    mcu.pc = 0x4fce;
}

/* 0x4fce DIVXU #0x2ee0 r2:r3. */
void step_dsp_rate_divxu_0x2ee0_r2_r3_a(void)
{
    divxu16_imm(mcu.r[2], mcu.r[3], 0x2ee0);
    mcu.pc = 0x4fd2;
}

/* 0x4fd2 TST r3. */
void step_dsp_rate_tst_r3_a(void)
{
    tst16_reg(mcu.r[3]);
    mcu.pc = 0x4fd4;
}

/* 0x4fd4 BEQ 6 -> 0x4fdc. */
void step_dsp_rate_beq_6_to_0x4fdc(void)
{
    mcu.pc = beq(0x4fdc, 0x4fd6);
}

/* 0x4fd6 MOVG2 #0xffff r4. */
void step_dsp_rate_movg2_0xffff_r4(void)
{
    load_imm16(mcu.r[4], 0xffff);
    mcu.pc = 0x4fda;
}

/* 0x4fda BRA 35 -> 0x4fff. */
void step_dsp_rate_bra_35_to_0x4fff(void)
{
    mcu.pc = 0x4fff;
}

/* 0x4fdc CLR r1. */
void step_dsp_rate_clr_r1_b(void)
{
    clr16_reg(mcu.r[1]);
    mcu.pc = 0x4fde;
}

/* 0x4fde MOVG2 r2 r1. */
void step_dsp_rate_movg2_r2_r1_b(void)
{
    mov_reg8(mcu.r[1], mcu.r[2]);
    mcu.pc = 0x4fe0;
}

/* 0x4fe0 ADD r1 r1. */
void step_dsp_rate_add_r1_r1_b(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x4fe2;
}

/* 0x4fe2 MOVG2 @r1+0x78ee r4. */
void step_dsp_rate_movg2_r1_0x78ee_r4_a(void)
{
    load16(mcu.r[4], ind_addr(1, 30958));
    mcu.pc = 0x4fe6;
}

/* 0x4fe6 CLR r1. */
void step_dsp_rate_clr_r1_c(void)
{
    clr16_reg(mcu.r[1]);
    mcu.pc = 0x4fe8;
}

/* 0x4fe8 SWAP r2. */
void step_dsp_rate_swap_r2_a(void)
{
    swap_reg(mcu.r[2]);
    mcu.pc = 0x4fea;
}

/* 0x4fea MOVG2 r2 r1. */
void step_dsp_rate_movg2_r2_r1_c(void)
{
    mov_reg8(mcu.r[1], mcu.r[2]);
    mcu.pc = 0x4fec;
}

/* 0x4fec ADD r1 r1. */
void step_dsp_rate_add_r1_r1_c(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x4fee;
}

/* 0x4fee MOVG2 @r1+0x7aee r1. */
void step_dsp_rate_movg2_r1_0x7aee_r1_a(void)
{
    load16(mcu.r[1], ind_addr(1, 31470));
    mcu.pc = 0x4ff2;
}

/* 0x4ff2 MULXU r1 r4:r5. */
void step_dsp_rate_mulxu_r1_r4_r5_a(void)
{
    mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[1]);
    mcu.pc = 0x4ff4;
}

/* 0x4ff4 ROTL r4. */
void step_dsp_rate_rotl_r4_b(void)
{
    rotl(mcu.r[4]);
    mcu.pc = 0x4ff6;
}

/* 0x4ff6 ROTL r4. */
void step_dsp_rate_rotl_r4_c(void)
{
    rotl(mcu.r[4]);
    mcu.pc = 0x4ff8;
}

/* 0x4ff8 AND #0x03 r4. */
void step_dsp_rate_and_0x03_r4_a(void)
{
    and8_imm(mcu.r[4], 0x03);
    mcu.pc = 0x4ffb;
}

/* 0x4ffb SWAP r4. */
void step_dsp_rate_swap_r4_a(void)
{
    swap_reg(mcu.r[4]);
    mcu.pc = 0x4ffd;
}

/* 0x4ffd ADD r1 r4. */
void step_dsp_rate_add_r1_r4_a(void)
{
    add16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x4fff;
}

/* 0x4fff MOVG3 r4 -> (dp,0xce3c). */
void step_dsp_rate_movg3_r4_to_dp_0xce3c(void)
{
    store16(dp_addr(0xce3c), mcu.r[4]);
    mcu.pc = 0x5003;
}

/* 0x5003 MOVG2 @r0+46 r1. */
void step_dsp_rate_movg2_r0_46_r1(void)
{
    load16(mcu.r[1], ind_addr(0, 46));
    mcu.pc = 0x5006;
}

/* 0x5006 CLR r2. */
void step_dsp_rate_clr_r2_b(void)
{
    clr16_reg(mcu.r[2]);
    mcu.pc = 0x5008;
}

/* 0x5008 MOVG2 @r1+7 r2. */
void step_dsp_rate_movg2_r1_7_r2(void)
{
    load8(mcu.r[2], ind_addr(1, 7));
    mcu.pc = 0x500b;
}

/* 0x500b CMP @r0+0x00a4 r2. */
void step_dsp_rate_cmp_r0_0x00a4_r2(void)
{
    cmp8_mem(mcu.r[2], ind_addr(0, 164));
    mcu.pc = 0x500f;
}

/* 0x500f BEQ 0x00c0 -> 0x50d2. */
void step_dsp_rate_beq_0x00c0_to_0x50d2(void)
{
    mcu.pc = beq(0x50d2, 0x5012);
}

/* 0x5012 MOVG3 r2 -> @r0+0x00a4. */
void step_dsp_rate_movg3_r2_to_r0_0x00a4(void)
{
    store8(ind_addr(0, 164), mcu.r[2]);
    mcu.pc = 0x5016;
}

/* 0x5016 MOVG2 @r0+62 r3. */
void step_dsp_rate_movg2_r0_62_r3(void)
{
    load16(mcu.r[3], ind_addr(0, 62));
    mcu.pc = 0x5019;
}

/* 0x5019 MOVG2 @r0+41 r2. */
void step_dsp_rate_movg2_r0_41_r2(void)
{
    load8(mcu.r[2], ind_addr(0, 41));
    mcu.pc = 0x501c;
}

/* 0x501c SUB #0x3c68 r3. */
void step_dsp_rate_sub_0x3c68_r3(void)
{
    sub16_imm(mcu.r[3], 0x3c68);
    mcu.pc = 0x5020;
}

/* 0x5020 SUBX #0x01 r2. */
void step_dsp_rate_subx_0x01_r2(void)
{
    subx8_imm(mcu.r[2], 0x01);
    mcu.pc = 0x5023;
}

/* 0x5023 BPL 85 -> 0x507a. */
void step_dsp_rate_bpl_85_to_0x507a(void)
{
    mcu.pc = bpl(0x507a, 0x5025);
}

/* 0x5025 EXTS r2. */
void step_dsp_rate_exts_r2_b(void)
{
    exts(mcu.r[2]);
    mcu.pc = 0x5027;
}

/* 0x5027 NOT r2. */
void step_dsp_rate_not_r2_a(void)
{
    not16(mcu.r[2]);
    mcu.pc = 0x5029;
}

/* 0x5029 NOT r3. */
void step_dsp_rate_not_r3_a(void)
{
    not16(mcu.r[3]);
    mcu.pc = 0x502b;
}

/* 0x502b ADDQ #1 r3. */
void step_dsp_rate_addq_1_r3_b(void)
{
    addq16(mcu.r[3], 1);
    mcu.pc = 0x502d;
}

/* 0x502d ADDX #0x0000 r2. */
void step_dsp_rate_addx_0x0000_r2_b(void)
{
    addx16_imm(mcu.r[2], 0x0000);
    mcu.pc = 0x5031;
}

/* 0x5031 DIVXU #0x2ee0 r2:r3. */
void step_dsp_rate_divxu_0x2ee0_r2_r3_b(void)
{
    divxu16_imm(mcu.r[2], mcu.r[3], 0x2ee0);
    mcu.pc = 0x5035;
}

/* 0x5035 TST r2. */
void step_dsp_rate_tst_r2_a(void)
{
    tst16_reg(mcu.r[2]);
    mcu.pc = 0x5037;
}

/* 0x5037 BEQ 8 -> 0x5041. */
void step_dsp_rate_beq_8_to_0x5041(void)
{
    mcu.pc = beq(0x5041, 0x5039);
}

/* 0x5039 ADDQ #1 r3. */
void step_dsp_rate_addq_1_r3_c(void)
{
    addq16(mcu.r[3], 1);
    mcu.pc = 0x503b;
}

/* 0x503b NEG r2. */
void step_dsp_rate_neg_r2_b(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x503d;
}

/* 0x503d ADD #0x2ee0 r2. */
void step_dsp_rate_add_0x2ee0_r2_a(void)
{
    add16_imm(mcu.r[2], 0x2ee0);
    mcu.pc = 0x5041;
}

/* 0x5041 CLR r1. */
void step_dsp_rate_clr_r1_d(void)
{
    clr16_reg(mcu.r[1]);
    mcu.pc = 0x5043;
}

/* 0x5043 MOVG2 r2 r1. */
void step_dsp_rate_movg2_r2_r1_d(void)
{
    mov_reg8(mcu.r[1], mcu.r[2]);
    mcu.pc = 0x5045;
}

/* 0x5045 ADD r1 r1. */
void step_dsp_rate_add_r1_r1_d(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x5047;
}

/* 0x5047 MOVG2 @r1+0x78ee r4. */
void step_dsp_rate_movg2_r1_0x78ee_r4_b(void)
{
    load16(mcu.r[4], ind_addr(1, 30958));
    mcu.pc = 0x504b;
}

/* 0x504b CLR r1. */
void step_dsp_rate_clr_r1_e(void)
{
    clr16_reg(mcu.r[1]);
    mcu.pc = 0x504d;
}

/* 0x504d SWAP r2. */
void step_dsp_rate_swap_r2_b(void)
{
    swap_reg(mcu.r[2]);
    mcu.pc = 0x504f;
}

/* 0x504f MOVG2 r2 r1. */
void step_dsp_rate_movg2_r2_r1_e(void)
{
    mov_reg8(mcu.r[1], mcu.r[2]);
    mcu.pc = 0x5051;
}

/* 0x5051 ADD r1 r1. */
void step_dsp_rate_add_r1_r1_e(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x5053;
}

/* 0x5053 MOVG2 @r1+0x7aee r1. */
void step_dsp_rate_movg2_r1_0x7aee_r1_b(void)
{
    load16(mcu.r[1], ind_addr(1, 31470));
    mcu.pc = 0x5057;
}

/* 0x5057 MULXU r1 r4:r5. */
void step_dsp_rate_mulxu_r1_r4_r5_b(void)
{
    mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[1]);
    mcu.pc = 0x5059;
}

/* 0x5059 ROTL r4. */
void step_dsp_rate_rotl_r4_d(void)
{
    rotl(mcu.r[4]);
    mcu.pc = 0x505b;
}

/* 0x505b ROTL r4. */
void step_dsp_rate_rotl_r4_e(void)
{
    rotl(mcu.r[4]);
    mcu.pc = 0x505d;
}

/* 0x505d AND #0x03 r4. */
void step_dsp_rate_and_0x03_r4_b(void)
{
    and8_imm(mcu.r[4], 0x03);
    mcu.pc = 0x5060;
}

/* 0x5060 SWAP r4. */
void step_dsp_rate_swap_r4_b(void)
{
    swap_reg(mcu.r[4]);
    mcu.pc = 0x5062;
}

/* 0x5062 ADD r1 r4. */
void step_dsp_rate_add_r1_r4_b(void)
{
    add16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x5064;
}

/* 0x5064 TST r3. */
void step_dsp_rate_tst_r3_b(void)
{
    tst16_reg(mcu.r[3]);
    mcu.pc = 0x5066;
}

/* 0x5066 BEQ 69 -> 0x50ad. */
void step_dsp_rate_beq_69_to_0x50ad(void)
{
    mcu.pc = beq(0x50ad, 0x5068);
}

/* 0x5068 SUB #0x0001 r3. */
void step_dsp_rate_sub_0x0001_r3_a(void)
{
    sub16_imm(mcu.r[3], 0x0001);
    mcu.pc = 0x506c;
}

/* 0x506c SHLR r4. */
void step_dsp_rate_shlr_r4_a(void)
{
    shlr(mcu.r[4]);
    mcu.pc = 0x506e;
}

/* 0x506e cntjmp r3 -5 -> 0x506c. */
void step_dsp_rate_cntjmp_r3_5_to_0x506c(void)
{
    mcu.r[3] = (uint16_t)(mcu.r[3] - 1);
    mcu.pc = (mcu.r[3] != 0xffff) ? 0x506c : 0x5071;
}

/* 0x5071 TST r4. */
void step_dsp_rate_tst_r4_c(void)
{
    tst16_reg(mcu.r[4]);
    mcu.pc = 0x5073;
}

/* 0x5073 BNE 56 -> 0x50ad. */
void step_dsp_rate_bne_56_to_0x50ad(void)
{
    mcu.pc = bne(0x50ad, 0x5075);
}

/* 0x5075 movi r4 #0x0001. */
void step_dsp_rate_movi_r4_0x0001(void)
{
    movi16(mcu.r[4], 0x0001);
    mcu.pc = 0x5078;
}

/* 0x5078 BRA 51 -> 0x50ad. */
void step_dsp_rate_bra_51_to_0x50ad(void)
{
    mcu.pc = 0x50ad;
}

/* 0x507a EXTS r2. */
void step_dsp_rate_exts_r2_c(void)
{
    exts(mcu.r[2]);
    mcu.pc = 0x507c;
}

/* 0x507c DIVXU #0x2ee0 r2:r3. */
void step_dsp_rate_divxu_0x2ee0_r2_r3_c(void)
{
    divxu16_imm(mcu.r[2], mcu.r[3], 0x2ee0);
    mcu.pc = 0x5080;
}

/* 0x5080 TST r3. */
void step_dsp_rate_tst_r3_c(void)
{
    tst16_reg(mcu.r[3]);
    mcu.pc = 0x5082;
}

/* 0x5082 BEQ 6 -> 0x508a. */
void step_dsp_rate_beq_6_to_0x508a(void)
{
    mcu.pc = beq(0x508a, 0x5084);
}

/* 0x5084 MOVG2 #0xffff r4. */
void step_dsp_rate_movg2_0xffff_r4_a(void)
{
    load_imm16(mcu.r[4], 0xffff);
    mcu.pc = 0x5088;
}

/* 0x5088 BRA 35 -> 0x50ad. */
void step_dsp_rate_bra_35_to_0x50ad(void)
{
    mcu.pc = 0x50ad;
}

/* 0x508a CLR r1. */
void step_dsp_rate_clr_r1_f(void)
{
    clr16_reg(mcu.r[1]);
    mcu.pc = 0x508c;
}

/* 0x508c MOVG2 r2 r1. */
void step_dsp_rate_movg2_r2_r1_f(void)
{
    mov_reg8(mcu.r[1], mcu.r[2]);
    mcu.pc = 0x508e;
}

/* 0x508e ADD r1 r1. */
void step_dsp_rate_add_r1_r1_f(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x5090;
}

/* 0x5090 MOVG2 @r1+0x78ee r4. */
void step_dsp_rate_movg2_r1_0x78ee_r4_c(void)
{
    load16(mcu.r[4], ind_addr(1, 30958));
    mcu.pc = 0x5094;
}

/* 0x5094 CLR r1. */
void step_dsp_rate_clr_r1_g(void)
{
    clr16_reg(mcu.r[1]);
    mcu.pc = 0x5096;
}

/* 0x5096 SWAP r2. */
void step_dsp_rate_swap_r2_c(void)
{
    swap_reg(mcu.r[2]);
    mcu.pc = 0x5098;
}

/* 0x5098 MOVG2 r2 r1. */
void step_dsp_rate_movg2_r2_r1_g(void)
{
    mov_reg8(mcu.r[1], mcu.r[2]);
    mcu.pc = 0x509a;
}

/* 0x509a ADD r1 r1. */
void step_dsp_rate_add_r1_r1_g(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x509c;
}

/* 0x509c MOVG2 @r1+0x7aee r1. */
void step_dsp_rate_movg2_r1_0x7aee_r1_c(void)
{
    load16(mcu.r[1], ind_addr(1, 31470));
    mcu.pc = 0x50a0;
}

/* 0x50a0 MULXU r1 r4:r5. */
void step_dsp_rate_mulxu_r1_r4_r5_c(void)
{
    mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[1]);
    mcu.pc = 0x50a2;
}

/* 0x50a2 ROTL r4. */
void step_dsp_rate_rotl_r4_f(void)
{
    rotl(mcu.r[4]);
    mcu.pc = 0x50a4;
}

/* 0x50a4 ROTL r4. */
void step_dsp_rate_rotl_r4_g(void)
{
    rotl(mcu.r[4]);
    mcu.pc = 0x50a6;
}

/* 0x50a6 AND #0x03 r4. */
void step_dsp_rate_and_0x03_r4_c(void)
{
    and8_imm(mcu.r[4], 0x03);
    mcu.pc = 0x50a9;
}

/* 0x50a9 SWAP r4. */
void step_dsp_rate_swap_r4_c(void)
{
    swap_reg(mcu.r[4]);
    mcu.pc = 0x50ab;
}

/* 0x50ab ADD r1 r4. */
void step_dsp_rate_add_r1_r4_c(void)
{
    add16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x50ad;
}

/* 0x50ad CLR r3. */
void step_dsp_rate_clr_r3_b(void)
{
    clr16_reg(mcu.r[3]);
    mcu.pc = 0x50af;
}

/* 0x50af CLR r2. */
void step_dsp_rate_clr_r2_c(void)
{
    clr16_reg(mcu.r[2]);
    mcu.pc = 0x50b1;
}

/* 0x50b1 MOVG2 @r0+0x00a4 r2. */
void step_dsp_rate_movg2_r0_0x00a4_r2(void)
{
    load8(mcu.r[2], ind_addr(0, 164));
    mcu.pc = 0x50b5;
}

/* 0x50b5 SUB #0x80 r2. */
void step_dsp_rate_sub_0x80_r2(void)
{
    sub8_imm(mcu.r[2], 0x80);
    mcu.pc = 0x50b8;
}

/* 0x50b8 BMI 9 -> 0x50c3. */
void step_dsp_rate_bmi_9_to_0x50c3(void)
{
    mcu.pc = bmi(0x50c3, 0x50ba);
}

/* 0x50ba DIVXU r4 r2:r3. */
void step_dsp_rate_divxu_r4_r2_r3(void)
{
    divxu16_reg(mcu.r[2], mcu.r[3], mcu.r[4]);
    mcu.pc = 0x50bc;
}

/* 0x50bc BGE 16 -> 0x50ce. */
void step_dsp_rate_bge_16_to_0x50ce(void)
{
    mcu.pc = bge(0x50ce, 0x50be);
}

/* 0x50be movi r3 #0x7fff. */
void step_dsp_rate_movi_r3_0x7fff(void)
{
    movi16(mcu.r[3], 0x7fff);
    mcu.pc = 0x50c1;
}

/* 0x50c1 BRA 11 -> 0x50ce. */
void step_dsp_rate_bra_11_to_0x50ce(void)
{
    mcu.pc = 0x50ce;
}

/* 0x50c3 NEG r2. */
void step_dsp_rate_neg_r2_c(void)
{
    neg8(mcu.r[2]);
    mcu.pc = 0x50c5;
}

/* 0x50c5 DIVXU r4 r2:r3. */
void step_dsp_rate_divxu_r4_r2_r3_a(void)
{
    divxu16_reg(mcu.r[2], mcu.r[3], mcu.r[4]);
    mcu.pc = 0x50c7;
}

/* 0x50c7 BGE 3 -> 0x50cc. */
void step_dsp_rate_bge_3_to_0x50cc(void)
{
    mcu.pc = bge(0x50cc, 0x50c9);
}

/* 0x50c9 movi r3 #0x7fff. */
void step_dsp_rate_movi_r3_0x7fff_a(void)
{
    movi16(mcu.r[3], 0x7fff);
    mcu.pc = 0x50cc;
}

/* 0x50cc NEG r3. */
void step_dsp_rate_neg_r3_b(void)
{
    neg16(mcu.r[3]);
    mcu.pc = 0x50ce;
}

/* 0x50ce MOVG3 r3 -> @r0+0x00a6. */
void step_dsp_rate_movg3_r3_to_r0_0x00a6(void)
{
    store16(ind_addr(0, 166), mcu.r[3]);
    mcu.pc = 0x50d2;
}

/* 0x50d2 MOVG2 @r0+0x00a6 r4. */
void step_dsp_rate_movg2_r0_0x00a6_r4(void)
{
    load16(mcu.r[4], ind_addr(0, 166));
    mcu.pc = 0x50d6;
}

/* 0x50d6 BMI 11 -> 0x50e3. */
void step_dsp_rate_bmi_11_to_0x50e3(void)
{
    mcu.pc = bmi(0x50e3, 0x50d8);
}

/* 0x50d8 ADD (dp,0xce3c) r4. */
void step_dsp_rate_add_dp_0xce3c_r4(void)
{
    add16_mem(mcu.r[4], dp_addr(0xce3c));
    mcu.pc = 0x50dc;
}

/* 0x50dc BCC 13 -> 0x50eb. */
void step_dsp_rate_bcc_13_to_0x50eb(void)
{
    mcu.pc = bcc(0x50eb, 0x50de);
}

/* 0x50de movi r4 #0xffff. */
void step_dsp_rate_movi_r4_0xffff(void)
{
    movi16(mcu.r[4], 0xffff);
    mcu.pc = 0x50e1;
}

/* 0x50e1 BRA 8 -> 0x50eb. */
void step_dsp_rate_bra_8_to_0x50eb(void)
{
    mcu.pc = 0x50eb;
}

/* 0x50e3 ADD (dp,0xce3c) r4. */
void step_dsp_rate_add_dp_0xce3c_r4_a(void)
{
    add16_mem(mcu.r[4], dp_addr(0xce3c));
    mcu.pc = 0x50e7;
}

/* 0x50e7 BCS 2 -> 0x50eb. */
void step_dsp_rate_bcs_2_to_0x50eb(void)
{
    mcu.pc = bcs(0x50eb, 0x50e9);
}

/* 0x50e9 CLR r4. */
void step_dsp_rate_clr_r4_b(void)
{
    clr16_reg(mcu.r[4]);
    mcu.pc = 0x50eb;
}

/* 0x50eb MOVG3 r4 -> @r0+72. */
void step_dsp_rate_movg3_r4_to_r0_72(void)
{
    store16(ind_addr(0, 72), mcu.r[4]);
    mcu.pc = 0x50ee;
}

/* 0x50ee rts. */
void step_dsp_rate_rts(void)
{
    mcu.pc = MCU_PopStack();
}

void dsp_rate_fill(void)
{
    MK2CPP_HandRegister(0x00004de7u, &step_dsp_rate_movg2_dp_0xad2a_r2);
    MK2CPP_HandRegister(0x00004debu, &step_dsp_rate_add_r0_22_r2);
    MK2CPP_HandRegister(0x00004deeu, &step_dsp_rate_clr_r0_22);
    MK2CPP_HandRegister(0x00004df1u, &step_dsp_rate_mulxu_r0_124_r2_r3);
    MK2CPP_HandRegister(0x00004df4u, &step_dsp_rate_add_r0_12_r3);
    MK2CPP_HandRegister(0x00004df7u, &step_dsp_rate_addx_0x0000_r2);
    MK2CPP_HandRegister(0x00004dfbu, &step_dsp_rate_tst_r2);
    MK2CPP_HandRegister(0x00004dfdu, &step_dsp_rate_beq_17_to_0x4e10);
    MK2CPP_HandRegister(0x00004dffu, &step_dsp_rate_sub_0xffff_r3);
    MK2CPP_HandRegister(0x00004e03u, &step_dsp_rate_subx_0x0000_r2);
    MK2CPP_HandRegister(0x00004e07u, &step_dsp_rate_divxu_r0_124_r2_r3);
    MK2CPP_HandRegister(0x00004e0au, &step_dsp_rate_movg3_r3_to_r0_22);
    MK2CPP_HandRegister(0x00004e0du, &step_dsp_rate_movi_r3_0xffff);
    MK2CPP_HandRegister(0x00004e10u, &step_dsp_rate_movg3_r3_to_r0_12);
    MK2CPP_HandRegister(0x00004e13u, &step_dsp_rate_sub_r0_neg_3_0x00);
    MK2CPP_HandRegister(0x00004e17u, &step_dsp_rate_bne_30_to_0x4e37);
    MK2CPP_HandRegister(0x00004e19u, &step_dsp_rate_movg2_r0_114_r4);
    MK2CPP_HandRegister(0x00004e1cu, &step_dsp_rate_sub_r0_112_r4);
    MK2CPP_HandRegister(0x00004e1fu, &step_dsp_rate_mulxu_r3_r4_r5);
    MK2CPP_HandRegister(0x00004e21u, &step_dsp_rate_clr_r3);
    MK2CPP_HandRegister(0x00004e23u, &step_dsp_rate_add_r0_112_r4);
    MK2CPP_HandRegister(0x00004e26u, &step_dsp_rate_addx_r0_106_r3);
    MK2CPP_HandRegister(0x00004e29u, &step_dsp_rate_movg3_r3_to_r0_44);
    MK2CPP_HandRegister(0x00004e2cu, &step_dsp_rate_movg3_r4_to_r0_68);
    MK2CPP_HandRegister(0x00004e2fu, &step_dsp_rate_movg3_r3_to_r0_45);
    MK2CPP_HandRegister(0x00004e32u, &step_dsp_rate_movg3_r4_to_r0_70);
    MK2CPP_HandRegister(0x00004e35u, &step_dsp_rate_bra_31_to_0x4e56);
    MK2CPP_HandRegister(0x00004e37u, &step_dsp_rate_movg2_r0_112_r4);
    MK2CPP_HandRegister(0x00004e3au, &step_dsp_rate_sub_r0_114_r4);
    MK2CPP_HandRegister(0x00004e3du, &step_dsp_rate_mulxu_r3_r4_r5_a);
    MK2CPP_HandRegister(0x00004e3fu, &step_dsp_rate_movg2_r0_112_r6);
    MK2CPP_HandRegister(0x00004e42u, &step_dsp_rate_movg2_r0_106_r5);
    MK2CPP_HandRegister(0x00004e45u, &step_dsp_rate_sub_r4_r6);
    MK2CPP_HandRegister(0x00004e47u, &step_dsp_rate_subx_0x00_r5);
    MK2CPP_HandRegister(0x00004e4au, &step_dsp_rate_movg3_r5_to_r0_44);
    MK2CPP_HandRegister(0x00004e4du, &step_dsp_rate_movg3_r6_to_r0_68);
    MK2CPP_HandRegister(0x00004e50u, &step_dsp_rate_movg3_r5_to_r0_45);
    MK2CPP_HandRegister(0x00004e53u, &step_dsp_rate_movg3_r6_to_r0_70);
    MK2CPP_HandRegister(0x00004e56u, &step_dsp_rate_movg2_r0_45_r5);
    MK2CPP_HandRegister(0x00004e59u, &step_dsp_rate_movg2_r0_70_r6);
    MK2CPP_HandRegister(0x00004e5cu, &step_dsp_rate_movg2_r0_0x0086_r2);
    MK2CPP_HandRegister(0x00004e60u, &step_dsp_rate_bpl_15_to_0x4e71);
    MK2CPP_HandRegister(0x00004e62u, &step_dsp_rate_neg_r2);
    MK2CPP_HandRegister(0x00004e64u, &step_dsp_rate_sub_r2_r6);
    MK2CPP_HandRegister(0x00004e66u, &step_dsp_rate_subx_0x00_r5_a);
    MK2CPP_HandRegister(0x00004e69u, &step_dsp_rate_bcc_32_to_0x4e8b);
    MK2CPP_HandRegister(0x00004e6bu, &step_dsp_rate_clr_r6);
    MK2CPP_HandRegister(0x00004e6du, &step_dsp_rate_clr_r5);
    MK2CPP_HandRegister(0x00004e6fu, &step_dsp_rate_bra_26_to_0x4e8b);
    MK2CPP_HandRegister(0x00004e71u, &step_dsp_rate_add_r2_r6);
    MK2CPP_HandRegister(0x00004e73u, &step_dsp_rate_addx_0x00_r5);
    MK2CPP_HandRegister(0x00004e76u, &step_dsp_rate_cmp_r5_b_0x01);
    MK2CPP_HandRegister(0x00004e78u, &step_dsp_rate_beq_9_to_0x4e83);
    MK2CPP_HandRegister(0x00004e7au, &step_dsp_rate_bcs_15_to_0x4e8b);
    MK2CPP_HandRegister(0x00004e7cu, &step_dsp_rate_movi_r6_0xf018);
    MK2CPP_HandRegister(0x00004e7fu, &step_dsp_rate_move_r5_0x01);
    MK2CPP_HandRegister(0x00004e81u, &step_dsp_rate_bra_8_to_0x4e8b);
    MK2CPP_HandRegister(0x00004e83u, &step_dsp_rate_cmp_r6_w_0xf018);
    MK2CPP_HandRegister(0x00004e86u, &step_dsp_rate_bls_3_to_0x4e8b);
    MK2CPP_HandRegister(0x00004e88u, &step_dsp_rate_movi_r6_0xf018_a);
    MK2CPP_HandRegister(0x00004e8bu, &step_dsp_rate_movg3_r5_to_r0_45_a);
    MK2CPP_HandRegister(0x00004e8eu, &step_dsp_rate_movg3_r6_to_r0_70_a);
    MK2CPP_HandRegister(0x00004e91u, &step_dsp_rate_movg2_r0_neg_118_r2);
    MK2CPP_HandRegister(0x00004e94u, &step_dsp_rate_movg2_r0_0x0090_r3);
    MK2CPP_HandRegister(0x00004e98u, &step_dsp_rate_movg2_r0_neg_96_r6);
    MK2CPP_HandRegister(0x00004e9bu, &step_dsp_rate_bsr16_to_0x50ef);
    MK2CPP_HandRegister(0x00004e9eu, &step_dsp_rate_movg2_r0_neg_84_r2);
    MK2CPP_HandRegister(0x00004ea1u, &step_dsp_rate_movg2_r0_0x0092_r3);
    MK2CPP_HandRegister(0x00004ea5u, &step_dsp_rate_movg2_r0_neg_62_r6);
    MK2CPP_HandRegister(0x00004ea8u, &step_dsp_rate_bsr16_to_0x50ef_a);
    MK2CPP_HandRegister(0x00004eabu, &step_dsp_rate_movg2_r0_45_r5_a);
    MK2CPP_HandRegister(0x00004eaeu, &step_dsp_rate_movg2_r0_70_r6_a);
    MK2CPP_HandRegister(0x00004eb1u, &step_dsp_rate_movg2_dp_0x8000_r3);
    MK2CPP_HandRegister(0x00004eb5u, &step_dsp_rate_sub_0x0400_r3);
    MK2CPP_HandRegister(0x00004eb9u, &step_dsp_rate_bmi_7_to_0x4ec2);
    MK2CPP_HandRegister(0x00004ebbu, &step_dsp_rate_add_r3_r6);
    MK2CPP_HandRegister(0x00004ebdu, &step_dsp_rate_addx_0x00_r5_a);
    MK2CPP_HandRegister(0x00004ec0u, &step_dsp_rate_bra_15_to_0x4ed1);
    MK2CPP_HandRegister(0x00004ec2u, &step_dsp_rate_neg_r3);
    MK2CPP_HandRegister(0x00004ec4u, &step_dsp_rate_sub_r3_r6);
    MK2CPP_HandRegister(0x00004ec6u, &step_dsp_rate_subx_0x00_r5_b);
    MK2CPP_HandRegister(0x00004ec9u, &step_dsp_rate_tst_r5);
    MK2CPP_HandRegister(0x00004ecbu, &step_dsp_rate_bpl_4_to_0x4ed1);
    MK2CPP_HandRegister(0x00004ecdu, &step_dsp_rate_clr_r6_a);
    MK2CPP_HandRegister(0x00004ecfu, &step_dsp_rate_clr_r5_a);
    MK2CPP_HandRegister(0x00004ed1u, &step_dsp_rate_movg2_r0_neg_2_r3);
    MK2CPP_HandRegister(0x00004ed4u, &step_dsp_rate_movg2_r3_0xce78_r3);
    MK2CPP_HandRegister(0x00004ed8u, &step_dsp_rate_add_r3_r3);
    MK2CPP_HandRegister(0x00004edau, &step_dsp_rate_movg2_r3_0xac4e_r3);
    MK2CPP_HandRegister(0x00004edeu, &step_dsp_rate_bmi_7_to_0x4ee7);
    MK2CPP_HandRegister(0x00004ee0u, &step_dsp_rate_add_r3_r6_a);
    MK2CPP_HandRegister(0x00004ee2u, &step_dsp_rate_addx_0x00_r5_b);
    MK2CPP_HandRegister(0x00004ee5u, &step_dsp_rate_bra_15_to_0x4ef6);
    MK2CPP_HandRegister(0x00004ee7u, &step_dsp_rate_neg_r3_a);
    MK2CPP_HandRegister(0x00004ee9u, &step_dsp_rate_sub_r3_r6_a);
    MK2CPP_HandRegister(0x00004eebu, &step_dsp_rate_subx_0x00_r5_c);
    MK2CPP_HandRegister(0x00004eeeu, &step_dsp_rate_tst_r5_a);
    MK2CPP_HandRegister(0x00004ef0u, &step_dsp_rate_bpl_4_to_0x4ef6);
    MK2CPP_HandRegister(0x00004ef2u, &step_dsp_rate_clr_r6_b);
    MK2CPP_HandRegister(0x00004ef4u, &step_dsp_rate_clr_r5_b);
    MK2CPP_HandRegister(0x00004ef6u, &step_dsp_rate_movg3_r5_to_r0_45_b);
    MK2CPP_HandRegister(0x00004ef9u, &step_dsp_rate_movg3_r6_to_r0_70_b);
    MK2CPP_HandRegister(0x00004efcu, &step_dsp_rate_movg2_r0_66_r5);
    MK2CPP_HandRegister(0x00004effu, &step_dsp_rate_movg2_r0_43_r4);
    MK2CPP_HandRegister(0x00004f02u, &step_dsp_rate_bne_14_to_0x4f12);
    MK2CPP_HandRegister(0x00004f04u, &step_dsp_rate_tst_r5_b);
    MK2CPP_HandRegister(0x00004f06u, &step_dsp_rate_bne_10_to_0x4f12);
    MK2CPP_HandRegister(0x00004f08u, &step_dsp_rate_clr_r2);
    MK2CPP_HandRegister(0x00004f0au, &step_dsp_rate_movg2_r0_45_r2);
    MK2CPP_HandRegister(0x00004f0du, &step_dsp_rate_movg2_r0_70_r3);
    MK2CPP_HandRegister(0x00004f10u, &step_dsp_rate_bra_92_to_0x4f6e);
    MK2CPP_HandRegister(0x00004f12u, &step_dsp_rate_movg2_r0_neg_2_r1);
    MK2CPP_HandRegister(0x00004f15u, &step_dsp_rate_clr_r3_a);
    MK2CPP_HandRegister(0x00004f17u, &step_dsp_rate_movg2_r0_neg_58_r2);
    MK2CPP_HandRegister(0x00004f1au, &step_dsp_rate_movg2_r2_r3);
    MK2CPP_HandRegister(0x00004f1cu, &step_dsp_rate_add_r3_r3_a);
    MK2CPP_HandRegister(0x00004f1eu, &step_dsp_rate_movg2_r3_0x77a6_r6);
    MK2CPP_HandRegister(0x00004f22u, &step_dsp_rate_movg2_dp_0xad2a_r2_a);
    MK2CPP_HandRegister(0x00004f26u, &step_dsp_rate_mulxu_r6_r2_r3);
    MK2CPP_HandRegister(0x00004f28u, &step_dsp_rate_tst_r4);
    MK2CPP_HandRegister(0x00004f2au, &step_dsp_rate_bpl_30_to_0x4f4a);
    MK2CPP_HandRegister(0x00004f2cu, &step_dsp_rate_neg_r4);
    MK2CPP_HandRegister(0x00004f2eu, &step_dsp_rate_neg_r5);
    MK2CPP_HandRegister(0x00004f30u, &step_dsp_rate_subx_0x00_r4);
    MK2CPP_HandRegister(0x00004f33u, &step_dsp_rate_sub_r3_r5);
    MK2CPP_HandRegister(0x00004f35u, &step_dsp_rate_subx_r2_r4);
    MK2CPP_HandRegister(0x00004f37u, &step_dsp_rate_tst_r4_a);
    MK2CPP_HandRegister(0x00004f39u, &step_dsp_rate_bpl_6_to_0x4f41);
    MK2CPP_HandRegister(0x00004f3bu, &step_dsp_rate_clr_r4);
    MK2CPP_HandRegister(0x00004f3du, &step_dsp_rate_clr_r5_c);
    MK2CPP_HandRegister(0x00004f3fu, &step_dsp_rate_bra_21_to_0x4f56);
    MK2CPP_HandRegister(0x00004f41u, &step_dsp_rate_neg_r4_a);
    MK2CPP_HandRegister(0x00004f43u, &step_dsp_rate_neg_r5_a);
    MK2CPP_HandRegister(0x00004f45u, &step_dsp_rate_subx_0x00_r4_a);
    MK2CPP_HandRegister(0x00004f48u, &step_dsp_rate_bra_12_to_0x4f56);
    MK2CPP_HandRegister(0x00004f4au, &step_dsp_rate_sub_r3_r5_a);
    MK2CPP_HandRegister(0x00004f4cu, &step_dsp_rate_subx_r2_r4_a);
    MK2CPP_HandRegister(0x00004f4eu, &step_dsp_rate_tst_r4_b);
    MK2CPP_HandRegister(0x00004f50u, &step_dsp_rate_bpl_4_to_0x4f56);
    MK2CPP_HandRegister(0x00004f52u, &step_dsp_rate_clr_r4_a);
    MK2CPP_HandRegister(0x00004f54u, &step_dsp_rate_clr_r5_d);
    MK2CPP_HandRegister(0x00004f56u, &step_dsp_rate_movg3_r5_to_r0_66);
    MK2CPP_HandRegister(0x00004f59u, &step_dsp_rate_movg3_r4_to_r0_43);
    MK2CPP_HandRegister(0x00004f5cu, &step_dsp_rate_clr_r2_a);
    MK2CPP_HandRegister(0x00004f5eu, &step_dsp_rate_movg2_r0_70_r3_a);
    MK2CPP_HandRegister(0x00004f61u, &step_dsp_rate_movg2_r0_45_r2_a);
    MK2CPP_HandRegister(0x00004f64u, &step_dsp_rate_add_r5_r3);
    MK2CPP_HandRegister(0x00004f66u, &step_dsp_rate_addx_r4_r2);
    MK2CPP_HandRegister(0x00004f68u, &step_dsp_rate_movg3_r2_to_r0_45);
    MK2CPP_HandRegister(0x00004f6bu, &step_dsp_rate_movg3_r3_to_r0_70);
    MK2CPP_HandRegister(0x00004f6eu, &step_dsp_rate_sub_r0_62_r3);
    MK2CPP_HandRegister(0x00004f71u, &step_dsp_rate_subx_r0_41_r2);
    MK2CPP_HandRegister(0x00004f74u, &step_dsp_rate_sub_0x2ee0_r3);
    MK2CPP_HandRegister(0x00004f78u, &step_dsp_rate_subx_0x00_r2);
    MK2CPP_HandRegister(0x00004f7bu, &step_dsp_rate_bpl_79_to_0x4fcc);
    MK2CPP_HandRegister(0x00004f7du, &step_dsp_rate_exts_r2);
    MK2CPP_HandRegister(0x00004f7fu, &step_dsp_rate_not_r2);
    MK2CPP_HandRegister(0x00004f81u, &step_dsp_rate_not_r3);
    MK2CPP_HandRegister(0x00004f83u, &step_dsp_rate_addq_1_r3);
    MK2CPP_HandRegister(0x00004f85u, &step_dsp_rate_addx_0x0000_r2_a);
    MK2CPP_HandRegister(0x00004f89u, &step_dsp_rate_divxu_0x2ee0_r2_r3);
    MK2CPP_HandRegister(0x00004f8du, &step_dsp_rate_cmp_r2_w_0x0000);
    MK2CPP_HandRegister(0x00004f90u, &step_dsp_rate_beq_8_to_0x4f9a);
    MK2CPP_HandRegister(0x00004f92u, &step_dsp_rate_addq_1_r3_a);
    MK2CPP_HandRegister(0x00004f94u, &step_dsp_rate_neg_r2_a);
    MK2CPP_HandRegister(0x00004f96u, &step_dsp_rate_add_0x2ee0_r2);
    MK2CPP_HandRegister(0x00004f9au, &step_dsp_rate_clr_r1);
    MK2CPP_HandRegister(0x00004f9cu, &step_dsp_rate_movg2_r2_r1);
    MK2CPP_HandRegister(0x00004f9eu, &step_dsp_rate_add_r1_r1);
    MK2CPP_HandRegister(0x00004fa0u, &step_dsp_rate_movg2_r1_0x78ee_r4);
    MK2CPP_HandRegister(0x00004fa4u, &step_dsp_rate_clr_r1_a);
    MK2CPP_HandRegister(0x00004fa6u, &step_dsp_rate_swap_r2);
    MK2CPP_HandRegister(0x00004fa8u, &step_dsp_rate_movg2_r2_r1_a);
    MK2CPP_HandRegister(0x00004faau, &step_dsp_rate_add_r1_r1_a);
    MK2CPP_HandRegister(0x00004facu, &step_dsp_rate_movg2_r1_0x7aee_r1);
    MK2CPP_HandRegister(0x00004fb0u, &step_dsp_rate_mulxu_r1_r4_r5);
    MK2CPP_HandRegister(0x00004fb2u, &step_dsp_rate_rotl_r4);
    MK2CPP_HandRegister(0x00004fb4u, &step_dsp_rate_rotl_r4_a);
    MK2CPP_HandRegister(0x00004fb6u, &step_dsp_rate_and_0x03_r4);
    MK2CPP_HandRegister(0x00004fb9u, &step_dsp_rate_swap_r4);
    MK2CPP_HandRegister(0x00004fbbu, &step_dsp_rate_add_r1_r4);
    MK2CPP_HandRegister(0x00004fbdu, &step_dsp_rate_tst_r3);
    MK2CPP_HandRegister(0x00004fbfu, &step_dsp_rate_beq_62_to_0x4fff);
    MK2CPP_HandRegister(0x00004fc1u, &step_dsp_rate_sub_0x0001_r3);
    MK2CPP_HandRegister(0x00004fc5u, &step_dsp_rate_shlr_r4);
    MK2CPP_HandRegister(0x00004fc7u, &step_dsp_rate_cntjmp_r3_5_to_0x4fc5);
    MK2CPP_HandRegister(0x00004fcau, &step_dsp_rate_bra_51_to_0x4fff);
    MK2CPP_HandRegister(0x00004fccu, &step_dsp_rate_exts_r2_a);
    MK2CPP_HandRegister(0x00004fceu, &step_dsp_rate_divxu_0x2ee0_r2_r3_a);
    MK2CPP_HandRegister(0x00004fd2u, &step_dsp_rate_tst_r3_a);
    MK2CPP_HandRegister(0x00004fd4u, &step_dsp_rate_beq_6_to_0x4fdc);
    MK2CPP_HandRegister(0x00004fd6u, &step_dsp_rate_movg2_0xffff_r4);
    MK2CPP_HandRegister(0x00004fdau, &step_dsp_rate_bra_35_to_0x4fff);
    MK2CPP_HandRegister(0x00004fdcu, &step_dsp_rate_clr_r1_b);
    MK2CPP_HandRegister(0x00004fdeu, &step_dsp_rate_movg2_r2_r1_b);
    MK2CPP_HandRegister(0x00004fe0u, &step_dsp_rate_add_r1_r1_b);
    MK2CPP_HandRegister(0x00004fe2u, &step_dsp_rate_movg2_r1_0x78ee_r4_a);
    MK2CPP_HandRegister(0x00004fe6u, &step_dsp_rate_clr_r1_c);
    MK2CPP_HandRegister(0x00004fe8u, &step_dsp_rate_swap_r2_a);
    MK2CPP_HandRegister(0x00004feau, &step_dsp_rate_movg2_r2_r1_c);
    MK2CPP_HandRegister(0x00004fecu, &step_dsp_rate_add_r1_r1_c);
    MK2CPP_HandRegister(0x00004feeu, &step_dsp_rate_movg2_r1_0x7aee_r1_a);
    MK2CPP_HandRegister(0x00004ff2u, &step_dsp_rate_mulxu_r1_r4_r5_a);
    MK2CPP_HandRegister(0x00004ff4u, &step_dsp_rate_rotl_r4_b);
    MK2CPP_HandRegister(0x00004ff6u, &step_dsp_rate_rotl_r4_c);
    MK2CPP_HandRegister(0x00004ff8u, &step_dsp_rate_and_0x03_r4_a);
    MK2CPP_HandRegister(0x00004ffbu, &step_dsp_rate_swap_r4_a);
    MK2CPP_HandRegister(0x00004ffdu, &step_dsp_rate_add_r1_r4_a);
    MK2CPP_HandRegister(0x00004fffu, &step_dsp_rate_movg3_r4_to_dp_0xce3c);
    MK2CPP_HandRegister(0x00005003u, &step_dsp_rate_movg2_r0_46_r1);
    MK2CPP_HandRegister(0x00005006u, &step_dsp_rate_clr_r2_b);
    MK2CPP_HandRegister(0x00005008u, &step_dsp_rate_movg2_r1_7_r2);
    MK2CPP_HandRegister(0x0000500bu, &step_dsp_rate_cmp_r0_0x00a4_r2);
    MK2CPP_HandRegister(0x0000500fu, &step_dsp_rate_beq_0x00c0_to_0x50d2);
    MK2CPP_HandRegister(0x00005012u, &step_dsp_rate_movg3_r2_to_r0_0x00a4);
    MK2CPP_HandRegister(0x00005016u, &step_dsp_rate_movg2_r0_62_r3);
    MK2CPP_HandRegister(0x00005019u, &step_dsp_rate_movg2_r0_41_r2);
    MK2CPP_HandRegister(0x0000501cu, &step_dsp_rate_sub_0x3c68_r3);
    MK2CPP_HandRegister(0x00005020u, &step_dsp_rate_subx_0x01_r2);
    MK2CPP_HandRegister(0x00005023u, &step_dsp_rate_bpl_85_to_0x507a);
    MK2CPP_HandRegister(0x00005025u, &step_dsp_rate_exts_r2_b);
    MK2CPP_HandRegister(0x00005027u, &step_dsp_rate_not_r2_a);
    MK2CPP_HandRegister(0x00005029u, &step_dsp_rate_not_r3_a);
    MK2CPP_HandRegister(0x0000502bu, &step_dsp_rate_addq_1_r3_b);
    MK2CPP_HandRegister(0x0000502du, &step_dsp_rate_addx_0x0000_r2_b);
    MK2CPP_HandRegister(0x00005031u, &step_dsp_rate_divxu_0x2ee0_r2_r3_b);
    MK2CPP_HandRegister(0x00005035u, &step_dsp_rate_tst_r2_a);
    MK2CPP_HandRegister(0x00005037u, &step_dsp_rate_beq_8_to_0x5041);
    MK2CPP_HandRegister(0x00005039u, &step_dsp_rate_addq_1_r3_c);
    MK2CPP_HandRegister(0x0000503bu, &step_dsp_rate_neg_r2_b);
    MK2CPP_HandRegister(0x0000503du, &step_dsp_rate_add_0x2ee0_r2_a);
    MK2CPP_HandRegister(0x00005041u, &step_dsp_rate_clr_r1_d);
    MK2CPP_HandRegister(0x00005043u, &step_dsp_rate_movg2_r2_r1_d);
    MK2CPP_HandRegister(0x00005045u, &step_dsp_rate_add_r1_r1_d);
    MK2CPP_HandRegister(0x00005047u, &step_dsp_rate_movg2_r1_0x78ee_r4_b);
    MK2CPP_HandRegister(0x0000504bu, &step_dsp_rate_clr_r1_e);
    MK2CPP_HandRegister(0x0000504du, &step_dsp_rate_swap_r2_b);
    MK2CPP_HandRegister(0x0000504fu, &step_dsp_rate_movg2_r2_r1_e);
    MK2CPP_HandRegister(0x00005051u, &step_dsp_rate_add_r1_r1_e);
    MK2CPP_HandRegister(0x00005053u, &step_dsp_rate_movg2_r1_0x7aee_r1_b);
    MK2CPP_HandRegister(0x00005057u, &step_dsp_rate_mulxu_r1_r4_r5_b);
    MK2CPP_HandRegister(0x00005059u, &step_dsp_rate_rotl_r4_d);
    MK2CPP_HandRegister(0x0000505bu, &step_dsp_rate_rotl_r4_e);
    MK2CPP_HandRegister(0x0000505du, &step_dsp_rate_and_0x03_r4_b);
    MK2CPP_HandRegister(0x00005060u, &step_dsp_rate_swap_r4_b);
    MK2CPP_HandRegister(0x00005062u, &step_dsp_rate_add_r1_r4_b);
    MK2CPP_HandRegister(0x00005064u, &step_dsp_rate_tst_r3_b);
    MK2CPP_HandRegister(0x00005066u, &step_dsp_rate_beq_69_to_0x50ad);
    MK2CPP_HandRegister(0x00005068u, &step_dsp_rate_sub_0x0001_r3_a);
    MK2CPP_HandRegister(0x0000506cu, &step_dsp_rate_shlr_r4_a);
    MK2CPP_HandRegister(0x0000506eu, &step_dsp_rate_cntjmp_r3_5_to_0x506c);
    MK2CPP_HandRegister(0x00005071u, &step_dsp_rate_tst_r4_c);
    MK2CPP_HandRegister(0x00005073u, &step_dsp_rate_bne_56_to_0x50ad);
    MK2CPP_HandRegister(0x00005075u, &step_dsp_rate_movi_r4_0x0001);
    MK2CPP_HandRegister(0x00005078u, &step_dsp_rate_bra_51_to_0x50ad);
    MK2CPP_HandRegister(0x0000507au, &step_dsp_rate_exts_r2_c);
    MK2CPP_HandRegister(0x0000507cu, &step_dsp_rate_divxu_0x2ee0_r2_r3_c);
    MK2CPP_HandRegister(0x00005080u, &step_dsp_rate_tst_r3_c);
    MK2CPP_HandRegister(0x00005082u, &step_dsp_rate_beq_6_to_0x508a);
    MK2CPP_HandRegister(0x00005084u, &step_dsp_rate_movg2_0xffff_r4_a);
    MK2CPP_HandRegister(0x00005088u, &step_dsp_rate_bra_35_to_0x50ad);
    MK2CPP_HandRegister(0x0000508au, &step_dsp_rate_clr_r1_f);
    MK2CPP_HandRegister(0x0000508cu, &step_dsp_rate_movg2_r2_r1_f);
    MK2CPP_HandRegister(0x0000508eu, &step_dsp_rate_add_r1_r1_f);
    MK2CPP_HandRegister(0x00005090u, &step_dsp_rate_movg2_r1_0x78ee_r4_c);
    MK2CPP_HandRegister(0x00005094u, &step_dsp_rate_clr_r1_g);
    MK2CPP_HandRegister(0x00005096u, &step_dsp_rate_swap_r2_c);
    MK2CPP_HandRegister(0x00005098u, &step_dsp_rate_movg2_r2_r1_g);
    MK2CPP_HandRegister(0x0000509au, &step_dsp_rate_add_r1_r1_g);
    MK2CPP_HandRegister(0x0000509cu, &step_dsp_rate_movg2_r1_0x7aee_r1_c);
    MK2CPP_HandRegister(0x000050a0u, &step_dsp_rate_mulxu_r1_r4_r5_c);
    MK2CPP_HandRegister(0x000050a2u, &step_dsp_rate_rotl_r4_f);
    MK2CPP_HandRegister(0x000050a4u, &step_dsp_rate_rotl_r4_g);
    MK2CPP_HandRegister(0x000050a6u, &step_dsp_rate_and_0x03_r4_c);
    MK2CPP_HandRegister(0x000050a9u, &step_dsp_rate_swap_r4_c);
    MK2CPP_HandRegister(0x000050abu, &step_dsp_rate_add_r1_r4_c);
    MK2CPP_HandRegister(0x000050adu, &step_dsp_rate_clr_r3_b);
    MK2CPP_HandRegister(0x000050afu, &step_dsp_rate_clr_r2_c);
    MK2CPP_HandRegister(0x000050b1u, &step_dsp_rate_movg2_r0_0x00a4_r2);
    MK2CPP_HandRegister(0x000050b5u, &step_dsp_rate_sub_0x80_r2);
    MK2CPP_HandRegister(0x000050b8u, &step_dsp_rate_bmi_9_to_0x50c3);
    MK2CPP_HandRegister(0x000050bau, &step_dsp_rate_divxu_r4_r2_r3);
    MK2CPP_HandRegister(0x000050bcu, &step_dsp_rate_bge_16_to_0x50ce);
    MK2CPP_HandRegister(0x000050beu, &step_dsp_rate_movi_r3_0x7fff);
    MK2CPP_HandRegister(0x000050c1u, &step_dsp_rate_bra_11_to_0x50ce);
    MK2CPP_HandRegister(0x000050c3u, &step_dsp_rate_neg_r2_c);
    MK2CPP_HandRegister(0x000050c5u, &step_dsp_rate_divxu_r4_r2_r3_a);
    MK2CPP_HandRegister(0x000050c7u, &step_dsp_rate_bge_3_to_0x50cc);
    MK2CPP_HandRegister(0x000050c9u, &step_dsp_rate_movi_r3_0x7fff_a);
    MK2CPP_HandRegister(0x000050ccu, &step_dsp_rate_neg_r3_b);
    MK2CPP_HandRegister(0x000050ceu, &step_dsp_rate_movg3_r3_to_r0_0x00a6);
    MK2CPP_HandRegister(0x000050d2u, &step_dsp_rate_movg2_r0_0x00a6_r4);
    MK2CPP_HandRegister(0x000050d6u, &step_dsp_rate_bmi_11_to_0x50e3);
    MK2CPP_HandRegister(0x000050d8u, &step_dsp_rate_add_dp_0xce3c_r4);
    MK2CPP_HandRegister(0x000050dcu, &step_dsp_rate_bcc_13_to_0x50eb);
    MK2CPP_HandRegister(0x000050deu, &step_dsp_rate_movi_r4_0xffff);
    MK2CPP_HandRegister(0x000050e1u, &step_dsp_rate_bra_8_to_0x50eb);
    MK2CPP_HandRegister(0x000050e3u, &step_dsp_rate_add_dp_0xce3c_r4_a);
    MK2CPP_HandRegister(0x000050e7u, &step_dsp_rate_bcs_2_to_0x50eb);
    MK2CPP_HandRegister(0x000050e9u, &step_dsp_rate_clr_r4_b);
    MK2CPP_HandRegister(0x000050ebu, &step_dsp_rate_movg3_r4_to_r0_72);
    MK2CPP_HandRegister(0x000050eeu, &step_dsp_rate_rts);
}

} /* anonymous namespace */
} /* namespace mk2c */

namespace {

struct DspRateRegistration
{
    DspRateRegistration()
    {
        mk2c::hand_register_module(&mk2c::dsp_rate_fill);
    }
};

DspRateRegistration g_dsp_rate_registration;

} /* anonymous namespace */
