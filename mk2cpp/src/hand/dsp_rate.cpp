/*
 * HAND dsp/rate -- per-voice DSP rate update, one host step per H8 instruction
 * (M4 wave, out/m4/18_closure_gap.md 4.2 item 27, "0x4DE7 family").
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
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

/* Safety net: a PC missing from the table is executed by the stock interpreter
 * for that one instruction (should never happen). */
void stock_instruction(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
}

/* ---- the 0x4DE7 family: one case per instruction PC ----------------------- */

uint32_t step_dsp_rate(void)
{
    switch (mcu.pc)
    {
    case 0x4de7: /* MOVG2 (dp,0xad2a) r2 [1d ad 2a 82] */
        load16(mcu.r[2], dp_addr(0xad2a));
        mcu.pc = 0x4deb;
        return 1;
    case 0x4deb: /* ADD @r0+22 r2 [e8 16 22] */
        add16_mem(mcu.r[2], ind_addr(0, 22));
        mcu.pc = 0x4dee;
        return 1;
    case 0x4dee: /* CLR @r0+22 [e8 16 13] */
        clr16_mem(ind_addr(0, 22));
        mcu.pc = 0x4df1;
        return 1;
    case 0x4df1: /* MULXU @r0+124 r2:r3 [e8 7c aa] */
        mulxu16_mem(mcu.r[2], mcu.r[3], ind_addr(0, 124));
        mcu.pc = 0x4df4;
        return 1;
    case 0x4df4: /* ADD @r0+12 r3 [e8 0c 23] */
        add16_mem(mcu.r[3], ind_addr(0, 12));
        mcu.pc = 0x4df7;
        return 1;
    case 0x4df7: /* ADDX #0x0000 r2 [0c 00 00 a2] */
        addx16_imm(mcu.r[2], 0x0000);
        mcu.pc = 0x4dfb;
        return 1;
    case 0x4dfb: /* TST r2 [aa 16] */
        tst16_reg(mcu.r[2]);
        mcu.pc = 0x4dfd;
        return 1;
    case 0x4dfd: /* BEQ 17 -> 0x4e10 [27 11] */
        mcu.pc = beq(0x4e10, 0x4dff);
        return 1;
    case 0x4dff: /* SUB #0xffff r3 [0c ff ff 33] */
        sub16_imm(mcu.r[3], 0xffff);
        mcu.pc = 0x4e03;
        return 1;
    case 0x4e03: /* SUBX #0x0000 r2 [0c 00 00 b2] */
        subx16_imm(mcu.r[2], 0x0000);
        mcu.pc = 0x4e07;
        return 1;
    case 0x4e07: /* DIVXU @r0+124 r2:r3 [e8 7c ba] */
        divxu16_mem(mcu.r[2], mcu.r[3], ind_addr(0, 124));
        mcu.pc = 0x4e0a;
        return 1;
    case 0x4e0a: /* MOVG3 r3 -> @r0+22 [e8 16 93] */
        store16(ind_addr(0, 22), mcu.r[3]);
        mcu.pc = 0x4e0d;
        return 1;
    case 0x4e0d: /* movi r3 #0xffff [5b ff ff] */
        movi16(mcu.r[3], 0xffff);
        mcu.pc = 0x4e10;
        return 1;
    case 0x4e10: /* MOVG3 r3 -> @r0+12 [e8 0c 93] */
        store16(ind_addr(0, 12), mcu.r[3]);
        mcu.pc = 0x4e13;
        return 1;
    case 0x4e13: /* SUB @r0+-3 #0x00 [e0 fd 04 00] */
        sub8_mem_imm_flags(ind_addr(0, -3), 0x00);
        mcu.pc = 0x4e17;
        return 1;
    case 0x4e17: /* BNE 30 -> 0x4e37 [26 1e] */
        mcu.pc = bne(0x4e37, 0x4e19);
        return 1;
    case 0x4e19: /* MOVG2 @r0+114 r4 [e8 72 84] */
        load16(mcu.r[4], ind_addr(0, 114));
        mcu.pc = 0x4e1c;
        return 1;
    case 0x4e1c: /* SUB @r0+112 r4 [e8 70 34] */
        sub16_mem(mcu.r[4], ind_addr(0, 112));
        mcu.pc = 0x4e1f;
        return 1;
    case 0x4e1f: /* MULXU r3 r4:r5 [ab ac] */
        mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[3]);
        mcu.pc = 0x4e21;
        return 1;
    case 0x4e21: /* CLR r3 [ab 13] */
        clr16_reg(mcu.r[3]);
        mcu.pc = 0x4e23;
        return 1;
    case 0x4e23: /* ADD @r0+112 r4 [e8 70 24] */
        add16_mem(mcu.r[4], ind_addr(0, 112));
        mcu.pc = 0x4e26;
        return 1;
    case 0x4e26: /* ADDX @r0+106 r3 [e0 6a a3] */
        addx8_mem(mcu.r[3], ind_addr(0, 106));
        mcu.pc = 0x4e29;
        return 1;
    case 0x4e29: /* MOVG3 r3 -> @r0+44 [e0 2c 93] */
        store8(ind_addr(0, 44), mcu.r[3]);
        mcu.pc = 0x4e2c;
        return 1;
    case 0x4e2c: /* MOVG3 r4 -> @r0+68 [e8 44 94] */
        store16(ind_addr(0, 68), mcu.r[4]);
        mcu.pc = 0x4e2f;
        return 1;
    case 0x4e2f: /* MOVG3 r3 -> @r0+45 [e0 2d 93] */
        store8(ind_addr(0, 45), mcu.r[3]);
        mcu.pc = 0x4e32;
        return 1;
    case 0x4e32: /* MOVG3 r4 -> @r0+70 [e8 46 94] */
        store16(ind_addr(0, 70), mcu.r[4]);
        mcu.pc = 0x4e35;
        return 1;
    case 0x4e35: /* BRA 31 -> 0x4e56 [20 1f] */
        mcu.pc = 0x4e56;
        return 1;
    case 0x4e37: /* MOVG2 @r0+112 r4 [e8 70 84] */
        load16(mcu.r[4], ind_addr(0, 112));
        mcu.pc = 0x4e3a;
        return 1;
    case 0x4e3a: /* SUB @r0+114 r4 [e8 72 34] */
        sub16_mem(mcu.r[4], ind_addr(0, 114));
        mcu.pc = 0x4e3d;
        return 1;
    case 0x4e3d: /* MULXU r3 r4:r5 [ab ac] */
        mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[3]);
        mcu.pc = 0x4e3f;
        return 1;
    case 0x4e3f: /* MOVG2 @r0+112 r6 [e8 70 86] */
        load16(mcu.r[6], ind_addr(0, 112));
        mcu.pc = 0x4e42;
        return 1;
    case 0x4e42: /* MOVG2 @r0+106 r5 [e0 6a 85] */
        load8(mcu.r[5], ind_addr(0, 106));
        mcu.pc = 0x4e45;
        return 1;
    case 0x4e45: /* SUB r4 r6 [ac 36] */
        sub16_reg(mcu.r[6], mcu.r[4]);
        mcu.pc = 0x4e47;
        return 1;
    case 0x4e47: /* SUBX #0x00 r5 [04 00 b5] */
        subx8_imm(mcu.r[5], 0x00);
        mcu.pc = 0x4e4a;
        return 1;
    case 0x4e4a: /* MOVG3 r5 -> @r0+44 [e0 2c 95] */
        store8(ind_addr(0, 44), mcu.r[5]);
        mcu.pc = 0x4e4d;
        return 1;
    case 0x4e4d: /* MOVG3 r6 -> @r0+68 [e8 44 96] */
        store16(ind_addr(0, 68), mcu.r[6]);
        mcu.pc = 0x4e50;
        return 1;
    case 0x4e50: /* MOVG3 r5 -> @r0+45 [e0 2d 95] */
        store8(ind_addr(0, 45), mcu.r[5]);
        mcu.pc = 0x4e53;
        return 1;
    case 0x4e53: /* MOVG3 r6 -> @r0+70 [e8 46 96] */
        store16(ind_addr(0, 70), mcu.r[6]);
        mcu.pc = 0x4e56;
        return 1;
    case 0x4e56: /* MOVG2 @r0+45 r5 [e0 2d 85] */
        load8(mcu.r[5], ind_addr(0, 45));
        mcu.pc = 0x4e59;
        return 1;
    case 0x4e59: /* MOVG2 @r0+70 r6 [e8 46 86] */
        load16(mcu.r[6], ind_addr(0, 70));
        mcu.pc = 0x4e5c;
        return 1;
    case 0x4e5c: /* MOVG2 @r0+0x0086 r2 [f8 00 86 82] */
        load16(mcu.r[2], ind_addr(0, 134));
        mcu.pc = 0x4e60;
        return 1;
    case 0x4e60: /* BPL 15 -> 0x4e71 [2a 0f] */
        mcu.pc = bpl(0x4e71, 0x4e62);
        return 1;
    case 0x4e62: /* NEG r2 [aa 14] */
        neg16(mcu.r[2]);
        mcu.pc = 0x4e64;
        return 1;
    case 0x4e64: /* SUB r2 r6 [aa 36] */
        sub16_reg(mcu.r[6], mcu.r[2]);
        mcu.pc = 0x4e66;
        return 1;
    case 0x4e66: /* SUBX #0x00 r5 [04 00 b5] */
        subx8_imm(mcu.r[5], 0x00);
        mcu.pc = 0x4e69;
        return 1;
    case 0x4e69: /* BCC 32 -> 0x4e8b [24 20] */
        mcu.pc = bcc(0x4e8b, 0x4e6b);
        return 1;
    case 0x4e6b: /* CLR r6 [ae 13] */
        clr16_reg(mcu.r[6]);
        mcu.pc = 0x4e6d;
        return 1;
    case 0x4e6d: /* CLR r5 [a5 13] */
        clr8_reg(mcu.r[5]);
        mcu.pc = 0x4e6f;
        return 1;
    case 0x4e6f: /* BRA 26 -> 0x4e8b [20 1a] */
        mcu.pc = 0x4e8b;
        return 1;
    case 0x4e71: /* ADD r2 r6 [aa 26] */
        add16_reg(mcu.r[6], mcu.r[2]);
        mcu.pc = 0x4e73;
        return 1;
    case 0x4e73: /* ADDX #0x00 r5 [04 00 a5] */
        addx8_imm(mcu.r[5], 0x00);
        mcu.pc = 0x4e76;
        return 1;
    case 0x4e76: /* cmp r5,b #0x01 [45 01] */
        cmp8_imm(mcu.r[5], 0x01);
        mcu.pc = 0x4e78;
        return 1;
    case 0x4e78: /* BEQ 9 -> 0x4e83 [27 09] */
        mcu.pc = beq(0x4e83, 0x4e7a);
        return 1;
    case 0x4e7a: /* BCS 15 -> 0x4e8b [25 0f] */
        mcu.pc = bcs(0x4e8b, 0x4e7c);
        return 1;
    case 0x4e7c: /* movi r6 #0xf018 [5e f0 18] */
        movi16(mcu.r[6], 0xf018);
        mcu.pc = 0x4e7f;
        return 1;
    case 0x4e7f: /* move r5 #0x01 [55 01] */
        move8(mcu.r[5], 0x01);
        mcu.pc = 0x4e81;
        return 1;
    case 0x4e81: /* BRA 8 -> 0x4e8b [20 08] */
        mcu.pc = 0x4e8b;
        return 1;
    case 0x4e83: /* cmp r6,w #0xf018 [4e f0 18] */
        cmp16_imm(mcu.r[6], 0xf018);
        mcu.pc = 0x4e86;
        return 1;
    case 0x4e86: /* BLS 3 -> 0x4e8b [23 03] */
        mcu.pc = bls(0x4e8b, 0x4e88);
        return 1;
    case 0x4e88: /* movi r6 #0xf018 [5e f0 18] */
        movi16(mcu.r[6], 0xf018);
        mcu.pc = 0x4e8b;
        return 1;
    case 0x4e8b: /* MOVG3 r5 -> @r0+45 [e0 2d 95] */
        store8(ind_addr(0, 45), mcu.r[5]);
        mcu.pc = 0x4e8e;
        return 1;
    case 0x4e8e: /* MOVG3 r6 -> @r0+70 [e8 46 96] */
        store16(ind_addr(0, 70), mcu.r[6]);
        mcu.pc = 0x4e91;
        return 1;
    case 0x4e91: /* MOVG2 @r0+-118 r2 [e8 8a 82] */
        load16(mcu.r[2], ind_addr(0, -118));
        mcu.pc = 0x4e94;
        return 1;
    case 0x4e94: /* MOVG2 @r0+0x0090 r3 [f8 00 90 83] */
        load16(mcu.r[3], ind_addr(0, 144));
        mcu.pc = 0x4e98;
        return 1;
    case 0x4e98: /* MOVG2 @r0+-96 r6 [e8 a0 86] */
        load16(mcu.r[6], ind_addr(0, -96));
        mcu.pc = 0x4e9b;
        return 1;
    case 0x4e9b: /* bsr16 -> 0x50ef [1e 02 51] */
        MCU_PushStack(0x4e9e);
        mcu.pc = 0x50ef;
        return 1;
    case 0x4e9e: /* MOVG2 @r0+-84 r2 [e8 ac 82] */
        load16(mcu.r[2], ind_addr(0, -84));
        mcu.pc = 0x4ea1;
        return 1;
    case 0x4ea1: /* MOVG2 @r0+0x0092 r3 [f8 00 92 83] */
        load16(mcu.r[3], ind_addr(0, 146));
        mcu.pc = 0x4ea5;
        return 1;
    case 0x4ea5: /* MOVG2 @r0+-62 r6 [e8 c2 86] */
        load16(mcu.r[6], ind_addr(0, -62));
        mcu.pc = 0x4ea8;
        return 1;
    case 0x4ea8: /* bsr16 -> 0x50ef [1e 02 44] */
        MCU_PushStack(0x4eab);
        mcu.pc = 0x50ef;
        return 1;
    case 0x4eab: /* MOVG2 @r0+45 r5 [e0 2d 85] */
        load8(mcu.r[5], ind_addr(0, 45));
        mcu.pc = 0x4eae;
        return 1;
    case 0x4eae: /* MOVG2 @r0+70 r6 [e8 46 86] */
        load16(mcu.r[6], ind_addr(0, 70));
        mcu.pc = 0x4eb1;
        return 1;
    case 0x4eb1: /* MOVG2 (dp,0x8000) r3 [1d 80 00 83] */
        load16(mcu.r[3], dp_addr(0x8000));
        mcu.pc = 0x4eb5;
        return 1;
    case 0x4eb5: /* SUB #0x0400 r3 [0c 04 00 33] */
        sub16_imm(mcu.r[3], 0x0400);
        mcu.pc = 0x4eb9;
        return 1;
    case 0x4eb9: /* BMI 7 -> 0x4ec2 [2b 07] */
        mcu.pc = bmi(0x4ec2, 0x4ebb);
        return 1;
    case 0x4ebb: /* ADD r3 r6 [ab 26] */
        add16_reg(mcu.r[6], mcu.r[3]);
        mcu.pc = 0x4ebd;
        return 1;
    case 0x4ebd: /* ADDX #0x00 r5 [04 00 a5] */
        addx8_imm(mcu.r[5], 0x00);
        mcu.pc = 0x4ec0;
        return 1;
    case 0x4ec0: /* BRA 15 -> 0x4ed1 [20 0f] */
        mcu.pc = 0x4ed1;
        return 1;
    case 0x4ec2: /* NEG r3 [ab 14] */
        neg16(mcu.r[3]);
        mcu.pc = 0x4ec4;
        return 1;
    case 0x4ec4: /* SUB r3 r6 [ab 36] */
        sub16_reg(mcu.r[6], mcu.r[3]);
        mcu.pc = 0x4ec6;
        return 1;
    case 0x4ec6: /* SUBX #0x00 r5 [04 00 b5] */
        subx8_imm(mcu.r[5], 0x00);
        mcu.pc = 0x4ec9;
        return 1;
    case 0x4ec9: /* TST r5 [a5 16] */
        tst8_reg(mcu.r[5]);
        mcu.pc = 0x4ecb;
        return 1;
    case 0x4ecb: /* BPL 4 -> 0x4ed1 [2a 04] */
        mcu.pc = bpl(0x4ed1, 0x4ecd);
        return 1;
    case 0x4ecd: /* CLR r6 [ae 13] */
        clr16_reg(mcu.r[6]);
        mcu.pc = 0x4ecf;
        return 1;
    case 0x4ecf: /* CLR r5 [a5 13] */
        clr8_reg(mcu.r[5]);
        mcu.pc = 0x4ed1;
        return 1;
    case 0x4ed1: /* MOVG2 @r0+-2 r3 [e8 fe 83] */
        load16(mcu.r[3], ind_addr(0, -2));
        mcu.pc = 0x4ed4;
        return 1;
    case 0x4ed4: /* MOVG2 @r3+0xce78 r3 [f3 ce 78 83] */
        load8(mcu.r[3], ind_addr(3, 52856));
        mcu.pc = 0x4ed8;
        return 1;
    case 0x4ed8: /* ADD r3 r3 [ab 23] */
        add16_reg(mcu.r[3], mcu.r[3]);
        mcu.pc = 0x4eda;
        return 1;
    case 0x4eda: /* MOVG2 @r3+0xac4e r3 [fb ac 4e 83] */
        load16(mcu.r[3], ind_addr(3, 44110));
        mcu.pc = 0x4ede;
        return 1;
    case 0x4ede: /* BMI 7 -> 0x4ee7 [2b 07] */
        mcu.pc = bmi(0x4ee7, 0x4ee0);
        return 1;
    case 0x4ee0: /* ADD r3 r6 [ab 26] */
        add16_reg(mcu.r[6], mcu.r[3]);
        mcu.pc = 0x4ee2;
        return 1;
    case 0x4ee2: /* ADDX #0x00 r5 [04 00 a5] */
        addx8_imm(mcu.r[5], 0x00);
        mcu.pc = 0x4ee5;
        return 1;
    case 0x4ee5: /* BRA 15 -> 0x4ef6 [20 0f] */
        mcu.pc = 0x4ef6;
        return 1;
    case 0x4ee7: /* NEG r3 [ab 14] */
        neg16(mcu.r[3]);
        mcu.pc = 0x4ee9;
        return 1;
    case 0x4ee9: /* SUB r3 r6 [ab 36] */
        sub16_reg(mcu.r[6], mcu.r[3]);
        mcu.pc = 0x4eeb;
        return 1;
    case 0x4eeb: /* SUBX #0x00 r5 [04 00 b5] */
        subx8_imm(mcu.r[5], 0x00);
        mcu.pc = 0x4eee;
        return 1;
    case 0x4eee: /* TST r5 [a5 16] */
        tst8_reg(mcu.r[5]);
        mcu.pc = 0x4ef0;
        return 1;
    case 0x4ef0: /* BPL 4 -> 0x4ef6 [2a 04] */
        mcu.pc = bpl(0x4ef6, 0x4ef2);
        return 1;
    case 0x4ef2: /* CLR r6 [ae 13] */
        clr16_reg(mcu.r[6]);
        mcu.pc = 0x4ef4;
        return 1;
    case 0x4ef4: /* CLR r5 [a5 13] */
        clr8_reg(mcu.r[5]);
        mcu.pc = 0x4ef6;
        return 1;
    case 0x4ef6: /* MOVG3 r5 -> @r0+45 [e0 2d 95] */
        store8(ind_addr(0, 45), mcu.r[5]);
        mcu.pc = 0x4ef9;
        return 1;
    case 0x4ef9: /* MOVG3 r6 -> @r0+70 [e8 46 96] */
        store16(ind_addr(0, 70), mcu.r[6]);
        mcu.pc = 0x4efc;
        return 1;
    case 0x4efc: /* MOVG2 @r0+66 r5 [e8 42 85] */
        load16(mcu.r[5], ind_addr(0, 66));
        mcu.pc = 0x4eff;
        return 1;
    case 0x4eff: /* MOVG2 @r0+43 r4 [e0 2b 84] */
        load8(mcu.r[4], ind_addr(0, 43));
        mcu.pc = 0x4f02;
        return 1;
    case 0x4f02: /* BNE 14 -> 0x4f12 [26 0e] */
        mcu.pc = bne(0x4f12, 0x4f04);
        return 1;
    case 0x4f04: /* TST r5 [ad 16] */
        tst16_reg(mcu.r[5]);
        mcu.pc = 0x4f06;
        return 1;
    case 0x4f06: /* BNE 10 -> 0x4f12 [26 0a] */
        mcu.pc = bne(0x4f12, 0x4f08);
        return 1;
    case 0x4f08: /* CLR r2 [aa 13] */
        clr16_reg(mcu.r[2]);
        mcu.pc = 0x4f0a;
        return 1;
    case 0x4f0a: /* MOVG2 @r0+45 r2 [e0 2d 82] */
        load8(mcu.r[2], ind_addr(0, 45));
        mcu.pc = 0x4f0d;
        return 1;
    case 0x4f0d: /* MOVG2 @r0+70 r3 [e8 46 83] */
        load16(mcu.r[3], ind_addr(0, 70));
        mcu.pc = 0x4f10;
        return 1;
    case 0x4f10: /* BRA 92 -> 0x4f6e [20 5c] */
        mcu.pc = 0x4f6e;
        return 1;
    case 0x4f12: /* MOVG2 @r0+-2 r1 [e8 fe 81] */
        load16(mcu.r[1], ind_addr(0, -2));
        mcu.pc = 0x4f15;
        return 1;
    case 0x4f15: /* CLR r3 [ab 13] */
        clr16_reg(mcu.r[3]);
        mcu.pc = 0x4f17;
        return 1;
    case 0x4f17: /* MOVG2 @r0+-58 r2 [e8 c6 82] */
        load16(mcu.r[2], ind_addr(0, -58));
        mcu.pc = 0x4f1a;
        return 1;
    case 0x4f1a: /* MOVG2 @r2 r3 [d2 83] */
        load8(mcu.r[3], ind_addr(2, 0));
        mcu.pc = 0x4f1c;
        return 1;
    case 0x4f1c: /* ADD r3 r3 [ab 23] */
        add16_reg(mcu.r[3], mcu.r[3]);
        mcu.pc = 0x4f1e;
        return 1;
    case 0x4f1e: /* MOVG2 @r3+0x77a6 r6 [fb 77 a6 86] */
        load16(mcu.r[6], ind_addr(3, 30630));
        mcu.pc = 0x4f22;
        return 1;
    case 0x4f22: /* MOVG2 (dp,0xad2a) r2 [1d ad 2a 82] */
        load16(mcu.r[2], dp_addr(0xad2a));
        mcu.pc = 0x4f26;
        return 1;
    case 0x4f26: /* MULXU r6 r2:r3 [ae aa] */
        mulxu16_reg(mcu.r[2], mcu.r[3], mcu.r[6]);
        mcu.pc = 0x4f28;
        return 1;
    case 0x4f28: /* TST r4 [a4 16] */
        tst8_reg(mcu.r[4]);
        mcu.pc = 0x4f2a;
        return 1;
    case 0x4f2a: /* BPL 30 -> 0x4f4a [2a 1e] */
        mcu.pc = bpl(0x4f4a, 0x4f2c);
        return 1;
    case 0x4f2c: /* NEG r4 [a4 14] */
        neg8(mcu.r[4]);
        mcu.pc = 0x4f2e;
        return 1;
    case 0x4f2e: /* NEG r5 [ad 14] */
        neg16(mcu.r[5]);
        mcu.pc = 0x4f30;
        return 1;
    case 0x4f30: /* SUBX #0x00 r4 [04 00 b4] */
        subx8_imm(mcu.r[4], 0x00);
        mcu.pc = 0x4f33;
        return 1;
    case 0x4f33: /* SUB r3 r5 [ab 35] */
        sub16_reg(mcu.r[5], mcu.r[3]);
        mcu.pc = 0x4f35;
        return 1;
    case 0x4f35: /* SUBX r2 r4 [a2 b4] */
        subx8_reg(mcu.r[4], mcu.r[2]);
        mcu.pc = 0x4f37;
        return 1;
    case 0x4f37: /* TST r4 [a4 16] */
        tst8_reg(mcu.r[4]);
        mcu.pc = 0x4f39;
        return 1;
    case 0x4f39: /* BPL 6 -> 0x4f41 [2a 06] */
        mcu.pc = bpl(0x4f41, 0x4f3b);
        return 1;
    case 0x4f3b: /* CLR r4 [a4 13] */
        clr8_reg(mcu.r[4]);
        mcu.pc = 0x4f3d;
        return 1;
    case 0x4f3d: /* CLR r5 [ad 13] */
        clr16_reg(mcu.r[5]);
        mcu.pc = 0x4f3f;
        return 1;
    case 0x4f3f: /* BRA 21 -> 0x4f56 [20 15] */
        mcu.pc = 0x4f56;
        return 1;
    case 0x4f41: /* NEG r4 [a4 14] */
        neg8(mcu.r[4]);
        mcu.pc = 0x4f43;
        return 1;
    case 0x4f43: /* NEG r5 [ad 14] */
        neg16(mcu.r[5]);
        mcu.pc = 0x4f45;
        return 1;
    case 0x4f45: /* SUBX #0x00 r4 [04 00 b4] */
        subx8_imm(mcu.r[4], 0x00);
        mcu.pc = 0x4f48;
        return 1;
    case 0x4f48: /* BRA 12 -> 0x4f56 [20 0c] */
        mcu.pc = 0x4f56;
        return 1;
    case 0x4f4a: /* SUB r3 r5 [ab 35] */
        sub16_reg(mcu.r[5], mcu.r[3]);
        mcu.pc = 0x4f4c;
        return 1;
    case 0x4f4c: /* SUBX r2 r4 [a2 b4] */
        subx8_reg(mcu.r[4], mcu.r[2]);
        mcu.pc = 0x4f4e;
        return 1;
    case 0x4f4e: /* TST r4 [a4 16] */
        tst8_reg(mcu.r[4]);
        mcu.pc = 0x4f50;
        return 1;
    case 0x4f50: /* BPL 4 -> 0x4f56 [2a 04] */
        mcu.pc = bpl(0x4f56, 0x4f52);
        return 1;
    case 0x4f52: /* CLR r4 [a4 13] */
        clr8_reg(mcu.r[4]);
        mcu.pc = 0x4f54;
        return 1;
    case 0x4f54: /* CLR r5 [ad 13] */
        clr16_reg(mcu.r[5]);
        mcu.pc = 0x4f56;
        return 1;
    case 0x4f56: /* MOVG3 r5 -> @r0+66 [e8 42 95] */
        store16(ind_addr(0, 66), mcu.r[5]);
        mcu.pc = 0x4f59;
        return 1;
    case 0x4f59: /* MOVG3 r4 -> @r0+43 [e0 2b 94] */
        store8(ind_addr(0, 43), mcu.r[4]);
        mcu.pc = 0x4f5c;
        return 1;
    case 0x4f5c: /* CLR r2 [aa 13] */
        clr16_reg(mcu.r[2]);
        mcu.pc = 0x4f5e;
        return 1;
    case 0x4f5e: /* MOVG2 @r0+70 r3 [e8 46 83] */
        load16(mcu.r[3], ind_addr(0, 70));
        mcu.pc = 0x4f61;
        return 1;
    case 0x4f61: /* MOVG2 @r0+45 r2 [e0 2d 82] */
        load8(mcu.r[2], ind_addr(0, 45));
        mcu.pc = 0x4f64;
        return 1;
    case 0x4f64: /* ADD r5 r3 [ad 23] */
        add16_reg(mcu.r[3], mcu.r[5]);
        mcu.pc = 0x4f66;
        return 1;
    case 0x4f66: /* ADDX r4 r2 [a4 a2] */
        addx8_reg(mcu.r[2], mcu.r[4]);
        mcu.pc = 0x4f68;
        return 1;
    case 0x4f68: /* MOVG3 r2 -> @r0+45 [e0 2d 92] */
        store8(ind_addr(0, 45), mcu.r[2]);
        mcu.pc = 0x4f6b;
        return 1;
    case 0x4f6b: /* MOVG3 r3 -> @r0+70 [e8 46 93] */
        store16(ind_addr(0, 70), mcu.r[3]);
        mcu.pc = 0x4f6e;
        return 1;
    case 0x4f6e: /* SUB @r0+62 r3 [e8 3e 33] */
        sub16_mem(mcu.r[3], ind_addr(0, 62));
        mcu.pc = 0x4f71;
        return 1;
    case 0x4f71: /* SUBX @r0+41 r2 [e0 29 b2] */
        subx8_mem(mcu.r[2], ind_addr(0, 41));
        mcu.pc = 0x4f74;
        return 1;
    case 0x4f74: /* SUB #0x2ee0 r3 [0c 2e e0 33] */
        sub16_imm(mcu.r[3], 0x2ee0);
        mcu.pc = 0x4f78;
        return 1;
    case 0x4f78: /* SUBX #0x00 r2 [04 00 b2] */
        subx8_imm(mcu.r[2], 0x00);
        mcu.pc = 0x4f7b;
        return 1;
    case 0x4f7b: /* BPL 79 -> 0x4fcc [2a 4f] */
        mcu.pc = bpl(0x4fcc, 0x4f7d);
        return 1;
    case 0x4f7d: /* EXTS r2 [a2 11] */
        exts(mcu.r[2]);
        mcu.pc = 0x4f7f;
        return 1;
    case 0x4f7f: /* NOT r2 [aa 15] */
        not16(mcu.r[2]);
        mcu.pc = 0x4f81;
        return 1;
    case 0x4f81: /* NOT r3 [ab 15] */
        not16(mcu.r[3]);
        mcu.pc = 0x4f83;
        return 1;
    case 0x4f83: /* ADDQ #1 r3 [ab 08] */
        addq16(mcu.r[3], 1);
        mcu.pc = 0x4f85;
        return 1;
    case 0x4f85: /* ADDX #0x0000 r2 [0c 00 00 a2] */
        addx16_imm(mcu.r[2], 0x0000);
        mcu.pc = 0x4f89;
        return 1;
    case 0x4f89: /* DIVXU #0x2ee0 r2:r3 [0c 2e e0 ba] */
        divxu16_imm(mcu.r[2], mcu.r[3], 0x2ee0);
        mcu.pc = 0x4f8d;
        return 1;
    case 0x4f8d: /* cmp r2,w #0x0000 [4a 00 00] */
        cmp16_imm(mcu.r[2], 0x0000);
        mcu.pc = 0x4f90;
        return 1;
    case 0x4f90: /* BEQ 8 -> 0x4f9a [27 08] */
        mcu.pc = beq(0x4f9a, 0x4f92);
        return 1;
    case 0x4f92: /* ADDQ #1 r3 [ab 08] */
        addq16(mcu.r[3], 1);
        mcu.pc = 0x4f94;
        return 1;
    case 0x4f94: /* NEG r2 [aa 14] */
        neg16(mcu.r[2]);
        mcu.pc = 0x4f96;
        return 1;
    case 0x4f96: /* ADD #0x2ee0 r2 [0c 2e e0 22] */
        add16_imm(mcu.r[2], 0x2ee0);
        mcu.pc = 0x4f9a;
        return 1;
    case 0x4f9a: /* CLR r1 [a9 13] */
        clr16_reg(mcu.r[1]);
        mcu.pc = 0x4f9c;
        return 1;
    case 0x4f9c: /* MOVG2 r2 r1 [a2 81] */
        mov_reg8(mcu.r[1], mcu.r[2]);
        mcu.pc = 0x4f9e;
        return 1;
    case 0x4f9e: /* ADD r1 r1 [a9 21] */
        add16_reg(mcu.r[1], mcu.r[1]);
        mcu.pc = 0x4fa0;
        return 1;
    case 0x4fa0: /* MOVG2 @r1+0x78ee r4 [f9 78 ee 84] */
        load16(mcu.r[4], ind_addr(1, 30958));
        mcu.pc = 0x4fa4;
        return 1;
    case 0x4fa4: /* CLR r1 [a9 13] */
        clr16_reg(mcu.r[1]);
        mcu.pc = 0x4fa6;
        return 1;
    case 0x4fa6: /* SWAP r2 [a2 10] */
        swap_reg(mcu.r[2]);
        mcu.pc = 0x4fa8;
        return 1;
    case 0x4fa8: /* MOVG2 r2 r1 [a2 81] */
        mov_reg8(mcu.r[1], mcu.r[2]);
        mcu.pc = 0x4faa;
        return 1;
    case 0x4faa: /* ADD r1 r1 [a9 21] */
        add16_reg(mcu.r[1], mcu.r[1]);
        mcu.pc = 0x4fac;
        return 1;
    case 0x4fac: /* MOVG2 @r1+0x7aee r1 [f9 7a ee 81] */
        load16(mcu.r[1], ind_addr(1, 31470));
        mcu.pc = 0x4fb0;
        return 1;
    case 0x4fb0: /* MULXU r1 r4:r5 [a9 ac] */
        mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[1]);
        mcu.pc = 0x4fb2;
        return 1;
    case 0x4fb2: /* ROTL r4 [ac 1c] */
        rotl(mcu.r[4]);
        mcu.pc = 0x4fb4;
        return 1;
    case 0x4fb4: /* ROTL r4 [ac 1c] */
        rotl(mcu.r[4]);
        mcu.pc = 0x4fb6;
        return 1;
    case 0x4fb6: /* AND #0x03 r4 [04 03 54] */
        and8_imm(mcu.r[4], 0x03);
        mcu.pc = 0x4fb9;
        return 1;
    case 0x4fb9: /* SWAP r4 [a4 10] */
        swap_reg(mcu.r[4]);
        mcu.pc = 0x4fbb;
        return 1;
    case 0x4fbb: /* ADD r1 r4 [a9 24] */
        add16_reg(mcu.r[4], mcu.r[1]);
        mcu.pc = 0x4fbd;
        return 1;
    case 0x4fbd: /* TST r3 [ab 16] */
        tst16_reg(mcu.r[3]);
        mcu.pc = 0x4fbf;
        return 1;
    case 0x4fbf: /* BEQ 62 -> 0x4fff [27 3e] */
        mcu.pc = beq(0x4fff, 0x4fc1);
        return 1;
    case 0x4fc1: /* SUB #0x0001 r3 [0c 00 01 33] */
        sub16_imm(mcu.r[3], 0x0001);
        mcu.pc = 0x4fc5;
        return 1;
    case 0x4fc5: /* SHLR r4 [ac 1b] */
        shlr(mcu.r[4]);
        mcu.pc = 0x4fc7;
        return 1;
    case 0x4fc7: /* cntjmp r3 -5 -> 0x4fc5 [01 bb fb] */
        mcu.r[3] = (uint16_t)(mcu.r[3] - 1);
        mcu.pc = (mcu.r[3] != 0xffff) ? 0x4fc5 : 0x4fca;
        return 1;
    case 0x4fca: /* BRA 51 -> 0x4fff [20 33] */
        mcu.pc = 0x4fff;
        return 1;
    case 0x4fcc: /* EXTS r2 [a2 11] */
        exts(mcu.r[2]);
        mcu.pc = 0x4fce;
        return 1;
    case 0x4fce: /* DIVXU #0x2ee0 r2:r3 [0c 2e e0 ba] */
        divxu16_imm(mcu.r[2], mcu.r[3], 0x2ee0);
        mcu.pc = 0x4fd2;
        return 1;
    case 0x4fd2: /* TST r3 [ab 16] */
        tst16_reg(mcu.r[3]);
        mcu.pc = 0x4fd4;
        return 1;
    case 0x4fd4: /* BEQ 6 -> 0x4fdc [27 06] */
        mcu.pc = beq(0x4fdc, 0x4fd6);
        return 1;
    case 0x4fd6: /* MOVG2 #0xffff r4 [0c ff ff 84] */
        load_imm16(mcu.r[4], 0xffff);
        mcu.pc = 0x4fda;
        return 1;
    case 0x4fda: /* BRA 35 -> 0x4fff [20 23] */
        mcu.pc = 0x4fff;
        return 1;
    case 0x4fdc: /* CLR r1 [a9 13] */
        clr16_reg(mcu.r[1]);
        mcu.pc = 0x4fde;
        return 1;
    case 0x4fde: /* MOVG2 r2 r1 [a2 81] */
        mov_reg8(mcu.r[1], mcu.r[2]);
        mcu.pc = 0x4fe0;
        return 1;
    case 0x4fe0: /* ADD r1 r1 [a9 21] */
        add16_reg(mcu.r[1], mcu.r[1]);
        mcu.pc = 0x4fe2;
        return 1;
    case 0x4fe2: /* MOVG2 @r1+0x78ee r4 [f9 78 ee 84] */
        load16(mcu.r[4], ind_addr(1, 30958));
        mcu.pc = 0x4fe6;
        return 1;
    case 0x4fe6: /* CLR r1 [a9 13] */
        clr16_reg(mcu.r[1]);
        mcu.pc = 0x4fe8;
        return 1;
    case 0x4fe8: /* SWAP r2 [a2 10] */
        swap_reg(mcu.r[2]);
        mcu.pc = 0x4fea;
        return 1;
    case 0x4fea: /* MOVG2 r2 r1 [a2 81] */
        mov_reg8(mcu.r[1], mcu.r[2]);
        mcu.pc = 0x4fec;
        return 1;
    case 0x4fec: /* ADD r1 r1 [a9 21] */
        add16_reg(mcu.r[1], mcu.r[1]);
        mcu.pc = 0x4fee;
        return 1;
    case 0x4fee: /* MOVG2 @r1+0x7aee r1 [f9 7a ee 81] */
        load16(mcu.r[1], ind_addr(1, 31470));
        mcu.pc = 0x4ff2;
        return 1;
    case 0x4ff2: /* MULXU r1 r4:r5 [a9 ac] */
        mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[1]);
        mcu.pc = 0x4ff4;
        return 1;
    case 0x4ff4: /* ROTL r4 [ac 1c] */
        rotl(mcu.r[4]);
        mcu.pc = 0x4ff6;
        return 1;
    case 0x4ff6: /* ROTL r4 [ac 1c] */
        rotl(mcu.r[4]);
        mcu.pc = 0x4ff8;
        return 1;
    case 0x4ff8: /* AND #0x03 r4 [04 03 54] */
        and8_imm(mcu.r[4], 0x03);
        mcu.pc = 0x4ffb;
        return 1;
    case 0x4ffb: /* SWAP r4 [a4 10] */
        swap_reg(mcu.r[4]);
        mcu.pc = 0x4ffd;
        return 1;
    case 0x4ffd: /* ADD r1 r4 [a9 24] */
        add16_reg(mcu.r[4], mcu.r[1]);
        mcu.pc = 0x4fff;
        return 1;
    case 0x4fff: /* MOVG3 r4 -> (dp,0xce3c) [1d ce 3c 94] */
        store16(dp_addr(0xce3c), mcu.r[4]);
        mcu.pc = 0x5003;
        return 1;
    case 0x5003: /* MOVG2 @r0+46 r1 [e8 2e 81] */
        load16(mcu.r[1], ind_addr(0, 46));
        mcu.pc = 0x5006;
        return 1;
    case 0x5006: /* CLR r2 [aa 13] */
        clr16_reg(mcu.r[2]);
        mcu.pc = 0x5008;
        return 1;
    case 0x5008: /* MOVG2 @r1+7 r2 [e1 07 82] */
        load8(mcu.r[2], ind_addr(1, 7));
        mcu.pc = 0x500b;
        return 1;
    case 0x500b: /* CMP @r0+0x00a4 r2 [f0 00 a4 72] */
        cmp8_mem(mcu.r[2], ind_addr(0, 164));
        mcu.pc = 0x500f;
        return 1;
    case 0x500f: /* BEQ 0x00c0 -> 0x50d2 [37 00 c0] */
        mcu.pc = beq(0x50d2, 0x5012);
        return 1;
    case 0x5012: /* MOVG3 r2 -> @r0+0x00a4 [f0 00 a4 92] */
        store8(ind_addr(0, 164), mcu.r[2]);
        mcu.pc = 0x5016;
        return 1;
    case 0x5016: /* MOVG2 @r0+62 r3 [e8 3e 83] */
        load16(mcu.r[3], ind_addr(0, 62));
        mcu.pc = 0x5019;
        return 1;
    case 0x5019: /* MOVG2 @r0+41 r2 [e0 29 82] */
        load8(mcu.r[2], ind_addr(0, 41));
        mcu.pc = 0x501c;
        return 1;
    case 0x501c: /* SUB #0x3c68 r3 [0c 3c 68 33] */
        sub16_imm(mcu.r[3], 0x3c68);
        mcu.pc = 0x5020;
        return 1;
    case 0x5020: /* SUBX #0x01 r2 [04 01 b2] */
        subx8_imm(mcu.r[2], 0x01);
        mcu.pc = 0x5023;
        return 1;
    case 0x5023: /* BPL 85 -> 0x507a [2a 55] */
        mcu.pc = bpl(0x507a, 0x5025);
        return 1;
    case 0x5025: /* EXTS r2 [a2 11] */
        exts(mcu.r[2]);
        mcu.pc = 0x5027;
        return 1;
    case 0x5027: /* NOT r2 [aa 15] */
        not16(mcu.r[2]);
        mcu.pc = 0x5029;
        return 1;
    case 0x5029: /* NOT r3 [ab 15] */
        not16(mcu.r[3]);
        mcu.pc = 0x502b;
        return 1;
    case 0x502b: /* ADDQ #1 r3 [ab 08] */
        addq16(mcu.r[3], 1);
        mcu.pc = 0x502d;
        return 1;
    case 0x502d: /* ADDX #0x0000 r2 [0c 00 00 a2] */
        addx16_imm(mcu.r[2], 0x0000);
        mcu.pc = 0x5031;
        return 1;
    case 0x5031: /* DIVXU #0x2ee0 r2:r3 [0c 2e e0 ba] */
        divxu16_imm(mcu.r[2], mcu.r[3], 0x2ee0);
        mcu.pc = 0x5035;
        return 1;
    case 0x5035: /* TST r2 [aa 16] */
        tst16_reg(mcu.r[2]);
        mcu.pc = 0x5037;
        return 1;
    case 0x5037: /* BEQ 8 -> 0x5041 [27 08] */
        mcu.pc = beq(0x5041, 0x5039);
        return 1;
    case 0x5039: /* ADDQ #1 r3 [ab 08] */
        addq16(mcu.r[3], 1);
        mcu.pc = 0x503b;
        return 1;
    case 0x503b: /* NEG r2 [aa 14] */
        neg16(mcu.r[2]);
        mcu.pc = 0x503d;
        return 1;
    case 0x503d: /* ADD #0x2ee0 r2 [0c 2e e0 22] */
        add16_imm(mcu.r[2], 0x2ee0);
        mcu.pc = 0x5041;
        return 1;
    case 0x5041: /* CLR r1 [a9 13] */
        clr16_reg(mcu.r[1]);
        mcu.pc = 0x5043;
        return 1;
    case 0x5043: /* MOVG2 r2 r1 [a2 81] */
        mov_reg8(mcu.r[1], mcu.r[2]);
        mcu.pc = 0x5045;
        return 1;
    case 0x5045: /* ADD r1 r1 [a9 21] */
        add16_reg(mcu.r[1], mcu.r[1]);
        mcu.pc = 0x5047;
        return 1;
    case 0x5047: /* MOVG2 @r1+0x78ee r4 [f9 78 ee 84] */
        load16(mcu.r[4], ind_addr(1, 30958));
        mcu.pc = 0x504b;
        return 1;
    case 0x504b: /* CLR r1 [a9 13] */
        clr16_reg(mcu.r[1]);
        mcu.pc = 0x504d;
        return 1;
    case 0x504d: /* SWAP r2 [a2 10] */
        swap_reg(mcu.r[2]);
        mcu.pc = 0x504f;
        return 1;
    case 0x504f: /* MOVG2 r2 r1 [a2 81] */
        mov_reg8(mcu.r[1], mcu.r[2]);
        mcu.pc = 0x5051;
        return 1;
    case 0x5051: /* ADD r1 r1 [a9 21] */
        add16_reg(mcu.r[1], mcu.r[1]);
        mcu.pc = 0x5053;
        return 1;
    case 0x5053: /* MOVG2 @r1+0x7aee r1 [f9 7a ee 81] */
        load16(mcu.r[1], ind_addr(1, 31470));
        mcu.pc = 0x5057;
        return 1;
    case 0x5057: /* MULXU r1 r4:r5 [a9 ac] */
        mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[1]);
        mcu.pc = 0x5059;
        return 1;
    case 0x5059: /* ROTL r4 [ac 1c] */
        rotl(mcu.r[4]);
        mcu.pc = 0x505b;
        return 1;
    case 0x505b: /* ROTL r4 [ac 1c] */
        rotl(mcu.r[4]);
        mcu.pc = 0x505d;
        return 1;
    case 0x505d: /* AND #0x03 r4 [04 03 54] */
        and8_imm(mcu.r[4], 0x03);
        mcu.pc = 0x5060;
        return 1;
    case 0x5060: /* SWAP r4 [a4 10] */
        swap_reg(mcu.r[4]);
        mcu.pc = 0x5062;
        return 1;
    case 0x5062: /* ADD r1 r4 [a9 24] */
        add16_reg(mcu.r[4], mcu.r[1]);
        mcu.pc = 0x5064;
        return 1;
    case 0x5064: /* TST r3 [ab 16] */
        tst16_reg(mcu.r[3]);
        mcu.pc = 0x5066;
        return 1;
    case 0x5066: /* BEQ 69 -> 0x50ad [27 45] */
        mcu.pc = beq(0x50ad, 0x5068);
        return 1;
    case 0x5068: /* SUB #0x0001 r3 [0c 00 01 33] */
        sub16_imm(mcu.r[3], 0x0001);
        mcu.pc = 0x506c;
        return 1;
    case 0x506c: /* SHLR r4 [ac 1b] */
        shlr(mcu.r[4]);
        mcu.pc = 0x506e;
        return 1;
    case 0x506e: /* cntjmp r3 -5 -> 0x506c [01 bb fb] */
        mcu.r[3] = (uint16_t)(mcu.r[3] - 1);
        mcu.pc = (mcu.r[3] != 0xffff) ? 0x506c : 0x5071;
        return 1;
    case 0x5071: /* TST r4 [ac 16] */
        tst16_reg(mcu.r[4]);
        mcu.pc = 0x5073;
        return 1;
    case 0x5073: /* BNE 56 -> 0x50ad [26 38] */
        mcu.pc = bne(0x50ad, 0x5075);
        return 1;
    case 0x5075: /* movi r4 #0x0001 [5c 00 01] */
        movi16(mcu.r[4], 0x0001);
        mcu.pc = 0x5078;
        return 1;
    case 0x5078: /* BRA 51 -> 0x50ad [20 33] */
        mcu.pc = 0x50ad;
        return 1;
    case 0x507a: /* EXTS r2 [a2 11] */
        exts(mcu.r[2]);
        mcu.pc = 0x507c;
        return 1;
    case 0x507c: /* DIVXU #0x2ee0 r2:r3 [0c 2e e0 ba] */
        divxu16_imm(mcu.r[2], mcu.r[3], 0x2ee0);
        mcu.pc = 0x5080;
        return 1;
    case 0x5080: /* TST r3 [ab 16] */
        tst16_reg(mcu.r[3]);
        mcu.pc = 0x5082;
        return 1;
    case 0x5082: /* BEQ 6 -> 0x508a [27 06] */
        mcu.pc = beq(0x508a, 0x5084);
        return 1;
    case 0x5084: /* MOVG2 #0xffff r4 [0c ff ff 84] */
        load_imm16(mcu.r[4], 0xffff);
        mcu.pc = 0x5088;
        return 1;
    case 0x5088: /* BRA 35 -> 0x50ad [20 23] */
        mcu.pc = 0x50ad;
        return 1;
    case 0x508a: /* CLR r1 [a9 13] */
        clr16_reg(mcu.r[1]);
        mcu.pc = 0x508c;
        return 1;
    case 0x508c: /* MOVG2 r2 r1 [a2 81] */
        mov_reg8(mcu.r[1], mcu.r[2]);
        mcu.pc = 0x508e;
        return 1;
    case 0x508e: /* ADD r1 r1 [a9 21] */
        add16_reg(mcu.r[1], mcu.r[1]);
        mcu.pc = 0x5090;
        return 1;
    case 0x5090: /* MOVG2 @r1+0x78ee r4 [f9 78 ee 84] */
        load16(mcu.r[4], ind_addr(1, 30958));
        mcu.pc = 0x5094;
        return 1;
    case 0x5094: /* CLR r1 [a9 13] */
        clr16_reg(mcu.r[1]);
        mcu.pc = 0x5096;
        return 1;
    case 0x5096: /* SWAP r2 [a2 10] */
        swap_reg(mcu.r[2]);
        mcu.pc = 0x5098;
        return 1;
    case 0x5098: /* MOVG2 r2 r1 [a2 81] */
        mov_reg8(mcu.r[1], mcu.r[2]);
        mcu.pc = 0x509a;
        return 1;
    case 0x509a: /* ADD r1 r1 [a9 21] */
        add16_reg(mcu.r[1], mcu.r[1]);
        mcu.pc = 0x509c;
        return 1;
    case 0x509c: /* MOVG2 @r1+0x7aee r1 [f9 7a ee 81] */
        load16(mcu.r[1], ind_addr(1, 31470));
        mcu.pc = 0x50a0;
        return 1;
    case 0x50a0: /* MULXU r1 r4:r5 [a9 ac] */
        mulxu16_reg(mcu.r[4], mcu.r[5], mcu.r[1]);
        mcu.pc = 0x50a2;
        return 1;
    case 0x50a2: /* ROTL r4 [ac 1c] */
        rotl(mcu.r[4]);
        mcu.pc = 0x50a4;
        return 1;
    case 0x50a4: /* ROTL r4 [ac 1c] */
        rotl(mcu.r[4]);
        mcu.pc = 0x50a6;
        return 1;
    case 0x50a6: /* AND #0x03 r4 [04 03 54] */
        and8_imm(mcu.r[4], 0x03);
        mcu.pc = 0x50a9;
        return 1;
    case 0x50a9: /* SWAP r4 [a4 10] */
        swap_reg(mcu.r[4]);
        mcu.pc = 0x50ab;
        return 1;
    case 0x50ab: /* ADD r1 r4 [a9 24] */
        add16_reg(mcu.r[4], mcu.r[1]);
        mcu.pc = 0x50ad;
        return 1;
    case 0x50ad: /* CLR r3 [ab 13] */
        clr16_reg(mcu.r[3]);
        mcu.pc = 0x50af;
        return 1;
    case 0x50af: /* CLR r2 [aa 13] */
        clr16_reg(mcu.r[2]);
        mcu.pc = 0x50b1;
        return 1;
    case 0x50b1: /* MOVG2 @r0+0x00a4 r2 [f0 00 a4 82] */
        load8(mcu.r[2], ind_addr(0, 164));
        mcu.pc = 0x50b5;
        return 1;
    case 0x50b5: /* SUB #0x80 r2 [04 80 32] */
        sub8_imm(mcu.r[2], 0x80);
        mcu.pc = 0x50b8;
        return 1;
    case 0x50b8: /* BMI 9 -> 0x50c3 [2b 09] */
        mcu.pc = bmi(0x50c3, 0x50ba);
        return 1;
    case 0x50ba: /* DIVXU r4 r2:r3 [ac ba] */
        divxu16_reg(mcu.r[2], mcu.r[3], mcu.r[4]);
        mcu.pc = 0x50bc;
        return 1;
    case 0x50bc: /* BGE 16 -> 0x50ce [2c 10] */
        mcu.pc = bge(0x50ce, 0x50be);
        return 1;
    case 0x50be: /* movi r3 #0x7fff [5b 7f ff] */
        movi16(mcu.r[3], 0x7fff);
        mcu.pc = 0x50c1;
        return 1;
    case 0x50c1: /* BRA 11 -> 0x50ce [20 0b] */
        mcu.pc = 0x50ce;
        return 1;
    case 0x50c3: /* NEG r2 [a2 14] */
        neg8(mcu.r[2]);
        mcu.pc = 0x50c5;
        return 1;
    case 0x50c5: /* DIVXU r4 r2:r3 [ac ba] */
        divxu16_reg(mcu.r[2], mcu.r[3], mcu.r[4]);
        mcu.pc = 0x50c7;
        return 1;
    case 0x50c7: /* BGE 3 -> 0x50cc [2c 03] */
        mcu.pc = bge(0x50cc, 0x50c9);
        return 1;
    case 0x50c9: /* movi r3 #0x7fff [5b 7f ff] */
        movi16(mcu.r[3], 0x7fff);
        mcu.pc = 0x50cc;
        return 1;
    case 0x50cc: /* NEG r3 [ab 14] */
        neg16(mcu.r[3]);
        mcu.pc = 0x50ce;
        return 1;
    case 0x50ce: /* MOVG3 r3 -> @r0+0x00a6 [f8 00 a6 93] */
        store16(ind_addr(0, 166), mcu.r[3]);
        mcu.pc = 0x50d2;
        return 1;
    case 0x50d2: /* MOVG2 @r0+0x00a6 r4 [f8 00 a6 84] */
        load16(mcu.r[4], ind_addr(0, 166));
        mcu.pc = 0x50d6;
        return 1;
    case 0x50d6: /* BMI 11 -> 0x50e3 [2b 0b] */
        mcu.pc = bmi(0x50e3, 0x50d8);
        return 1;
    case 0x50d8: /* ADD (dp,0xce3c) r4 [1d ce 3c 24] */
        add16_mem(mcu.r[4], dp_addr(0xce3c));
        mcu.pc = 0x50dc;
        return 1;
    case 0x50dc: /* BCC 13 -> 0x50eb [24 0d] */
        mcu.pc = bcc(0x50eb, 0x50de);
        return 1;
    case 0x50de: /* movi r4 #0xffff [5c ff ff] */
        movi16(mcu.r[4], 0xffff);
        mcu.pc = 0x50e1;
        return 1;
    case 0x50e1: /* BRA 8 -> 0x50eb [20 08] */
        mcu.pc = 0x50eb;
        return 1;
    case 0x50e3: /* ADD (dp,0xce3c) r4 [1d ce 3c 24] */
        add16_mem(mcu.r[4], dp_addr(0xce3c));
        mcu.pc = 0x50e7;
        return 1;
    case 0x50e7: /* BCS 2 -> 0x50eb [25 02] */
        mcu.pc = bcs(0x50eb, 0x50e9);
        return 1;
    case 0x50e9: /* CLR r4 [ac 13] */
        clr16_reg(mcu.r[4]);
        mcu.pc = 0x50eb;
        return 1;
    case 0x50eb: /* MOVG3 r4 -> @r0+72 [e8 48 94] */
        store16(ind_addr(0, 72), mcu.r[4]);
        mcu.pc = 0x50ee;
        return 1;
    case 0x50ee: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* One entry per instruction PC of 0x4DE7..0x50EE (302 PCs, L0 returns 1). */
const uint16_t kDspRatePcs[] = {
    0x4de7, 0x4deb, 0x4dee, 0x4df1, 0x4df4, 0x4df7, 0x4dfb, 0x4dfd,
    0x4dff, 0x4e03, 0x4e07, 0x4e0a, 0x4e0d, 0x4e10, 0x4e13, 0x4e17,
    0x4e19, 0x4e1c, 0x4e1f, 0x4e21, 0x4e23, 0x4e26, 0x4e29, 0x4e2c,
    0x4e2f, 0x4e32, 0x4e35, 0x4e37, 0x4e3a, 0x4e3d, 0x4e3f, 0x4e42,
    0x4e45, 0x4e47, 0x4e4a, 0x4e4d, 0x4e50, 0x4e53, 0x4e56, 0x4e59,
    0x4e5c, 0x4e60, 0x4e62, 0x4e64, 0x4e66, 0x4e69, 0x4e6b, 0x4e6d,
    0x4e6f, 0x4e71, 0x4e73, 0x4e76, 0x4e78, 0x4e7a, 0x4e7c, 0x4e7f,
    0x4e81, 0x4e83, 0x4e86, 0x4e88, 0x4e8b, 0x4e8e, 0x4e91, 0x4e94,
    0x4e98, 0x4e9b, 0x4e9e, 0x4ea1, 0x4ea5, 0x4ea8, 0x4eab, 0x4eae,
    0x4eb1, 0x4eb5, 0x4eb9, 0x4ebb, 0x4ebd, 0x4ec0, 0x4ec2, 0x4ec4,
    0x4ec6, 0x4ec9, 0x4ecb, 0x4ecd, 0x4ecf, 0x4ed1, 0x4ed4, 0x4ed8,
    0x4eda, 0x4ede, 0x4ee0, 0x4ee2, 0x4ee5, 0x4ee7, 0x4ee9, 0x4eeb,
    0x4eee, 0x4ef0, 0x4ef2, 0x4ef4, 0x4ef6, 0x4ef9, 0x4efc, 0x4eff,
    0x4f02, 0x4f04, 0x4f06, 0x4f08, 0x4f0a, 0x4f0d, 0x4f10, 0x4f12,
    0x4f15, 0x4f17, 0x4f1a, 0x4f1c, 0x4f1e, 0x4f22, 0x4f26, 0x4f28,
    0x4f2a, 0x4f2c, 0x4f2e, 0x4f30, 0x4f33, 0x4f35, 0x4f37, 0x4f39,
    0x4f3b, 0x4f3d, 0x4f3f, 0x4f41, 0x4f43, 0x4f45, 0x4f48, 0x4f4a,
    0x4f4c, 0x4f4e, 0x4f50, 0x4f52, 0x4f54, 0x4f56, 0x4f59, 0x4f5c,
    0x4f5e, 0x4f61, 0x4f64, 0x4f66, 0x4f68, 0x4f6b, 0x4f6e, 0x4f71,
    0x4f74, 0x4f78, 0x4f7b, 0x4f7d, 0x4f7f, 0x4f81, 0x4f83, 0x4f85,
    0x4f89, 0x4f8d, 0x4f90, 0x4f92, 0x4f94, 0x4f96, 0x4f9a, 0x4f9c,
    0x4f9e, 0x4fa0, 0x4fa4, 0x4fa6, 0x4fa8, 0x4faa, 0x4fac, 0x4fb0,
    0x4fb2, 0x4fb4, 0x4fb6, 0x4fb9, 0x4fbb, 0x4fbd, 0x4fbf, 0x4fc1,
    0x4fc5, 0x4fc7, 0x4fca, 0x4fcc, 0x4fce, 0x4fd2, 0x4fd4, 0x4fd6,
    0x4fda, 0x4fdc, 0x4fde, 0x4fe0, 0x4fe2, 0x4fe6, 0x4fe8, 0x4fea,
    0x4fec, 0x4fee, 0x4ff2, 0x4ff4, 0x4ff6, 0x4ff8, 0x4ffb, 0x4ffd,
    0x4fff, 0x5003, 0x5006, 0x5008, 0x500b, 0x500f, 0x5012, 0x5016,
    0x5019, 0x501c, 0x5020, 0x5023, 0x5025, 0x5027, 0x5029, 0x502b,
    0x502d, 0x5031, 0x5035, 0x5037, 0x5039, 0x503b, 0x503d, 0x5041,
    0x5043, 0x5045, 0x5047, 0x504b, 0x504d, 0x504f, 0x5051, 0x5053,
    0x5057, 0x5059, 0x505b, 0x505d, 0x5060, 0x5062, 0x5064, 0x5066,
    0x5068, 0x506c, 0x506e, 0x5071, 0x5073, 0x5075, 0x5078, 0x507a,
    0x507c, 0x5080, 0x5082, 0x5084, 0x5088, 0x508a, 0x508c, 0x508e,
    0x5090, 0x5094, 0x5096, 0x5098, 0x509a, 0x509c, 0x50a0, 0x50a2,
    0x50a4, 0x50a6, 0x50a9, 0x50ab, 0x50ad, 0x50af, 0x50b1, 0x50b5,
    0x50b8, 0x50ba, 0x50bc, 0x50be, 0x50c1, 0x50c3, 0x50c5, 0x50c7,
    0x50c9, 0x50cc, 0x50ce, 0x50d2, 0x50d6, 0x50d8, 0x50dc, 0x50de,
    0x50e1, 0x50e3, 0x50e7, 0x50e9, 0x50eb, 0x50ee,
};

void dsp_rate_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kDspRatePcs) / sizeof(kDspRatePcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kDspRatePcs[i], &step_dsp_rate);
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
