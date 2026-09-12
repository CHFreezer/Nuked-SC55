/*
 * HAND pcm/note_on_setup — note-on voice setup (class C7, rom1 cp=0,
 * flat 0x272e..0x2d92).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). Semantically rewritten in M4 closure step 2
 * (2026-09-13, named operations): one named void step function per PC,
 * registered flat=pc/cp0 via MK2CPP_HandRegister.
 *
 * Semantics: entered by mask_set 0x548e BRA 0x272e (voice_materialize.cpp).
 * Reads the tone record, clears cdfe, initializes P+0xa2 / P-0x51 and the
 * gate path; two entry arms share the body: 0x272e (BTSTI P-0x3b #7, BEQ
 * 0x2874) and 0x2874. The range ends at 0x2d92 BRA 0x3bb4 (C8 begins 0x2d95).
 * Evidence: out/m4/18_closure_gap.md 1.3 C7 / 4.2 row 8 (Q10); docs/07:141.
 * Confidence: C (ROM bytes + dasm).
 *
 * Registration: all PCs self-register via MK2CPP_HandRegister from a
 * file-static initializer (hand_registry.h); no shared aggregator file is
 * edited and duplicate registration is fatal (mk2cpp.cpp).
 */

#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_interrupt.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as note_path.cpp / native_allocfree.cpp). */
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

/* Single-entry build switch (09 5.4): 0 makes this slice not register at all
 * without touching other files. */
#define MK2CPP_HAND_NOTE_ON_SETUP 1

namespace mk2c {
namespace {

/* ---- local helpers (bodies copied from the previous generated form) ------- */

/* Page register for rN: dp (r0-r3), ep (r4/r5), tp (r6/r7). */
uint32_t page_of(uint32_t reg)
{
    if (reg >= 6)
        return mcu.tp;
    if (reg >= 4)
        return mcu.ep;
    return mcu.dp;
}

/* @rN+disp: the rN+disp sum wraps inside 16 bits. */
uint32_t ea_r(uint32_t reg, uint16_t disp)
{
    return ((uint32_t)page_of(reg) << 16) | (uint16_t)(mcu.r[reg] + disp);
}

/* (dp,disp16): absolute through the DP page register. */
uint32_t ea_dp(uint16_t disp)
{
    return ((uint32_t)mcu.dp << 16) | disp;
}

/* (br,disp8): the BR page register, low byte offset. */
uint32_t ea_br(uint16_t disp)
{
    return (uint32_t)(uint16_t)((mcu.br << 8) | disp);
}

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG2 @addr -> rN (byte): low byte replaced, high byte kept. */
void load8(uint16_t &reg, uint32_t addr)
{
    uint32_t data = (uint32_t)MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* MOVG2 @addr -> rN (word): odd-address check first, then flags from data. */
void load16(uint16_t &reg, uint32_t addr)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(addr);
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVG3 rN -> @addr (byte). */
void store8(uint32_t addr, uint32_t value)
{
    MCU_Write(addr, (uint8_t)value);
    MCU_SetStatusCommon(value, 0);
}

/* MOVG3 rN -> @addr (word): odd-address check first, then write + flags. */
void store16(uint32_t addr, uint32_t value)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, (uint16_t)value);
    MCU_SetStatusCommon(value, 1);
}

/* MOVG2 rS -> rD (byte): low byte replaced, high byte kept. */
void mov8_reg(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)(src & 0xff);
    dst &= ~0xff;
    dst |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

/* MOVG2 rS -> rD (word). */
void mov16_reg(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)src;
    dst = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVG #imm8 -> @addr (byte). */
void mov_imm8_mem(uint32_t addr, uint32_t imm)
{
    uint32_t d = (uint32_t)(int8_t)imm;
    MCU_Write(addr, (uint8_t)d);
    MCU_SetStatusCommon(d, 0);
}

/* MOVG #imm16 -> @addr (word). */
void mov_imm16_mem(uint32_t addr, uint16_t imm)
{
    uint32_t d = (uint32_t)imm;
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, (uint16_t)d);
    MCU_SetStatusCommon(d, 1);
}

void clr_reg(uint16_t &reg)
{
    reg = 0;
    flags_clr();
}

void clr8_reg(uint16_t &reg)
{
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)(0) & 0xffu));
    flags_clr();
}

void clr8_mem(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

void clr16_mem(uint32_t addr)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, 0);
    flags_clr();
}

void tst16_reg(uint16_t reg)
{
    uint32_t data = (uint32_t)reg;
    MCU_SetStatusCommon(data, 1);
    MCU_SetStatus(0, STATUS_C);
}

void btsti(uint32_t addr, uint32_t bit)
{
    uint32_t data = (uint32_t)MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
}

void btst8_reg(uint16_t reg, uint32_t bit)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
}

/* BSET_ORC / BCLR_ANDC on control register 0, sets ex_ignore. */
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

void shll16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t C = (data & 0x8000u) != 0;
    data <<= 1;
    reg = (uint16_t)data;
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

void shlr16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t C = data & 1;
    data >>= 1;
    reg = (uint16_t)data;
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

void swap16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t data_h = data >> 8;
    uint32_t data_l = data & 0xff;
    data = (data_l << 8) | data_h;
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
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

/* NEG rN byte: two's complement of the low byte, high byte kept. */
void neg8_reg(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)(data) & 0xffu));
}

void movi16(uint16_t &reg, uint16_t imm)
{
    uint16_t data = imm;
    reg = data;
    MCU_SetStatusCommon(data, 1);
}

void move8_imm(uint16_t &reg, uint8_t imm)
{
    uint8_t data = imm;
    reg &= ~0xff;
    reg |= data;
    MCU_SetStatusCommon(data, 0);
}

void add16_reg(uint16_t &dst, uint16_t src)
{
    int32_t t1 = (int32_t)dst;
    uint32_t t2 = (uint32_t)src;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    dst = (uint16_t)t1;
}

void add8_reg(uint16_t &dst, uint16_t src)
{
    int32_t t1 = (int32_t)dst;
    uint32_t t2 = (uint32_t)(src & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    dst = (uint16_t)((dst & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

void sub16_reg(uint16_t &dst, uint16_t src)
{
    int32_t t1 = (int32_t)dst;
    uint32_t t2 = (uint32_t)src;
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    dst = (uint16_t)t1;
}

void sub8_reg(uint16_t &dst, uint16_t src)
{
    int32_t t1 = (int32_t)dst;
    uint32_t t2 = (uint32_t)(src & 0xff);
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    dst = (uint16_t)((dst & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

void cmp16_reg(uint16_t a, uint16_t b)
{
    int32_t t1 = (int32_t)a;
    uint32_t t2 = (uint32_t)b;
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void add_imm16_reg(uint16_t &reg, uint16_t imm)
{
    int32_t t1 = (int32_t)reg;
    uint32_t t2 = (uint32_t)imm;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    reg = (uint16_t)t1;
}

void sub_imm8_reg(uint16_t &reg, uint32_t imm)
{
    int32_t t1 = (int32_t)reg;
    uint32_t t2 = imm;
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    reg &= ~0xff;
    reg |= (uint16_t)(t1 & 0xff);
}

void cmp_imm8_reg(uint16_t reg, uint32_t imm)
{
    int32_t t2 = (int32_t)imm;
    int32_t t1 = (int32_t)reg;
    MCU_SUB_Common(t1, t2, 0, 0);
}

void cmp_imm16_reg(uint16_t reg, uint16_t imm)
{
    int32_t t2 = (int32_t)imm;
    int32_t t1 = (int32_t)reg;
    MCU_SUB_Common(t1, t2, 0, 1);
}

void add8_mem(uint16_t &reg, uint32_t addr)
{
    int32_t t1 = (int32_t)reg;
    uint32_t t2 = (uint32_t)MCU_Read(addr);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

void add16_mem(uint16_t &reg, uint32_t addr)
{
    int32_t t1 = (int32_t)reg;
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(addr);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    reg = (uint16_t)t1;
}

void sub8_mem(uint16_t &reg, uint32_t addr)
{
    int32_t t1 = (int32_t)reg;
    uint32_t t2 = (uint32_t)MCU_Read(addr);
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

void sub16_mem(uint16_t &reg, uint32_t addr)
{
    int32_t t1 = (int32_t)reg;
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(addr);
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    reg = (uint16_t)t1;
}

void cmp8_mem_reg(uint32_t addr, uint16_t reg)
{
    int32_t t1 = (int32_t)reg;
    uint32_t t2 = (uint32_t)MCU_Read(addr);
    MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
}

void cmp16_mem_reg(uint32_t addr, uint16_t reg)
{
    int32_t t1 = (int32_t)reg;
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(addr);
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

/* SUB @addr,#imm8 / SUB @addr,#imm16: flags only, no write. */
void sub8_mem_imm8(uint32_t addr, uint32_t imm)
{
    uint32_t t1 = (uint32_t)MCU_Read(addr);
    uint32_t t2 = imm;
    MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
}

void sub16_mem_imm16(uint32_t addr, uint16_t imm)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t1 = (uint32_t)MCU_Read16(addr);
    uint32_t t2 = (uint32_t)imm;
    MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
}

void addx8_reg(uint16_t &dst, uint16_t src)
{
    int32_t t1 = (int32_t)dst;
    uint32_t t2 = (uint32_t)(src & 0xff);
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 0);
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
    dst &= ~0xff;
    dst |= (uint16_t)(t1 & 0xff);
}

void addx8_imm(uint16_t &dst, uint32_t imm)
{
    int32_t t1 = (int32_t)dst;
    uint32_t t2 = imm;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 0);
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
    dst &= ~0xff;
    dst |= (uint16_t)(t1 & 0xff);
}

void subx8_imm(uint16_t &dst, uint32_t imm)
{
    int32_t t1 = (int32_t)dst;
    uint32_t t2 = imm;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    t1 = MCU_SUB_Common(t1, (int32_t)t2, C, 0);
    dst &= ~0xff;
    dst |= (uint16_t)(t1 & 0xff);
}

void and_imm16_reg(uint16_t &reg, uint16_t imm)
{
    uint32_t data = (uint32_t)reg;
    uint32_t t2 = (uint32_t)imm;
    data &= t2;
    reg = (uint16_t)data;
    MCU_SetStatusCommon(reg, 1);
}

void and_imm8_reg(uint16_t &reg, uint32_t imm)
{
    uint32_t data = (uint32_t)reg;
    uint32_t t2 = imm;
    data &= t2;
    reg &= ~0xff;
    reg |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(reg, 0);
}

void or8_reg(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)(src & 0xff);
    dst |= (uint16_t)data;
    MCU_SetStatusCommon(dst, 0);
}

void adds16_mem(uint16_t &reg, uint32_t addr)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(addr);
    reg += (uint16_t)data;
}

void mulxu8_mem(uint16_t &acc, uint32_t addr)
{
    uint32_t t1 = (uint32_t)MCU_Read(addr);
    uint32_t t2 = (uint32_t)acc;
    t2 &= 0xff;
    t1 *= t2;
    t1 &= 0xffff;
    acc = (uint16_t)t1;
    uint32_t N = (t1 & 0x8000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void mulxu16_imm(uint16_t imm, uint16_t &high, uint16_t &low)
{
    uint32_t t1 = (uint32_t)imm;
    uint32_t t2 = (uint32_t)high;
    t1 *= t2;
    high = (uint16_t)(t1 >> 16);
    low = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* DIVXU rX rH:rL: H = remainder, L = quotient; divide-by-zero traps. */
void divxu16(uint16_t divisor, uint16_t &rem, uint16_t &quo)
{
    uint32_t t1 = (uint32_t)divisor;
    uint32_t t2 = 0;
    uint32_t r = 0, q = 0;
    if (t1 == 0)
    {
        MCU_ErrorTrap();
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        return;
    }
    t2 = ((uint32_t)rem << 16) | (uint32_t)quo;
    r = t2 % t1;
    q = t2 / t1;
    if (q > 0xffffu)
    {
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(0, STATUS_Z);
        MCU_SetStatus(1, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    else
    {
        rem = (uint16_t)r;
        quo = (uint16_t)q;
        MCU_SetStatusCommon(q, 1);
        MCU_SetStatus(0, STATUS_C);
    }
}

void ldc8_mem(uint32_t cr, uint32_t addr)
{
    uint32_t data = (uint32_t)MCU_Read(addr);
    MCU_ControlRegisterWrite(cr, 0, data);
    mcu.ex_ignore = 1;
}

void ldc_imm8(uint32_t cr, uint32_t imm)
{
    uint32_t data = imm;
    MCU_ControlRegisterWrite(cr, 0, data);
    mcu.ex_ignore = 1;
}

void stc8_reg(uint32_t cr, uint16_t &reg)
{
    uint32_t data = MCU_ControlRegisterRead(cr, 0);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)(data) & 0xffu));
}

void movl_br(uint16_t &reg, uint16_t disp)
{
    uint16_t addr = (uint16_t)((mcu.br << 8) | disp);
    uint32_t data = (uint32_t)MCU_Read(addr);
    reg &= ~0xff;
    reg |= (uint16_t)data;
    MCU_SetStatusCommon(data, 0);
}

void movlw_br(uint16_t &reg, uint16_t disp)
{
    uint16_t addr = (uint16_t)((mcu.br << 8) | disp);
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t data = MCU_Read16(addr);
    reg = data;
    MCU_SetStatusCommon(data, 1);
}

void movs_br(uint16_t reg, uint16_t disp)
{
    uint16_t addr = (uint16_t)((mcu.br << 8) | disp);
    uint32_t data = (uint32_t)(reg & 0xff);
    MCU_Write(addr, (uint8_t)data);
    MCU_SetStatusCommon(data, 0);
}

/* bsr/jsr: push the return address, jump. */
void call(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

/* cntjmp rN, disp: rN--; branch back while rN != 0xffff. */
void cntjmp(uint16_t &reg, uint16_t taken, uint16_t fall)
{
    reg = (uint16_t)(reg - 1);
    mcu.pc = (reg != 0xffff) ? taken : fall;
}

void bcc(uint16_t taken, uint16_t fall)
{
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    mcu.pc = (C == 0) ? taken : fall;
}

void bcs(uint16_t taken, uint16_t fall)
{
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    mcu.pc = (C == 1) ? taken : fall;
}

void bvc(uint16_t taken, uint16_t fall)
{
    uint32_t V = (mcu.sr & STATUS_V) != 0;
    mcu.pc = (V == 0) ? taken : fall;
}

void bhi(uint16_t taken, uint16_t fall)
{
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    mcu.pc = ((C | Z) == 0) ? taken : fall;
}

void bpl(uint16_t taken, uint16_t fall)
{
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    mcu.pc = (N == 0) ? taken : fall;
}

void bmi(uint16_t taken, uint16_t fall)
{
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    mcu.pc = (N == 1) ? taken : fall;
}

void bne(uint16_t taken, uint16_t fall)
{
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    mcu.pc = (Z == 0) ? taken : fall;
}

void beq(uint16_t taken, uint16_t fall)
{
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    mcu.pc = (Z == 1) ? taken : fall;
}

/* ======================================================================== */
/* C7 note_on_setup 0x272e..0x2d92, 583 PCs */
/* ======================================================================== */

/* 0x272e BTSTI @r0+-59 #7. */
void step_note_on_setup_btsti_r0_m59_7(void)
{
    btsti(ea_r(0, (uint16_t)(int8_t)0xc5), 7);
    mcu.pc = 0x2731;
}

/* 0x2731 BEQ 0x0140 -> 0x2874. */
void step_note_on_setup_beq_0x0140_0x2874(void)
{
    beq(0x2874, 0x2734);
}

/* 0x2734 LDC @r0+0x0099 r4. */
void step_note_on_setup_ldc_r0_0x0099_r4(void)
{
    ldc8_mem(4, ea_r(0, 0x0099));
    mcu.pc = 0x2738;
}

/* 0x2738 MOVG2 @r0+0x009e r5. */
void step_note_on_setup_movg2_r0_0x009e_r5(void)
{
    load16(mcu.r[5], ea_r(0, 0x009e));
    mcu.pc = 0x273c;
}

/* 0x273c MOVG2 @r5+8 r6. */
void step_note_on_setup_movg2_r5_8_r6(void)
{
    load8(mcu.r[6], ea_r(5, (uint16_t)(int8_t)0x08));
    mcu.pc = 0x273f;
}

/* 0x273f MOVG3 r6 -> @r0+0x00a2. */
void step_note_on_setup_movg3_r6_r0_0x00a2(void)
{
    store8(ea_r(0, 0x00a2), mcu.r[6]);
    mcu.pc = 0x2743;
}

/* 0x2743 MOVG2 @r0+-2 r1. */
void step_note_on_setup_movg2_r0_m2_r1(void)
{
    load16(mcu.r[1], ea_r(0, (uint16_t)(int8_t)0xfe));
    mcu.pc = 0x2746;
}

/* 0x2746 ADD r1 r1. */
void step_note_on_setup_add_r1_r1(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x2748;
}

/* 0x2748 CLR @r1+0xcdfe. */
void step_note_on_setup_clr_r1_0xcdfe(void)
{
    clr16_mem(ea_r(1, 0xcdfe));
    mcu.pc = 0x274c;
}

/* 0x274c CLR @r0+-81. */
void step_note_on_setup_clr_r0_m81(void)
{
    clr8_mem(ea_r(0, (uint16_t)(int8_t)0xaf));
    mcu.pc = 0x274f;
}

/* 0x274f MOVG2 @r5+4 r6. */
void step_note_on_setup_movg2_r5_4_r6(void)
{
    load8(mcu.r[6], ea_r(5, (uint16_t)(int8_t)0x04));
    mcu.pc = 0x2752;
}

/* 0x2752 BTSTI r6 #4. */
void step_note_on_setup_btsti_r6_4(void)
{
    btst8_reg(mcu.r[6], 4);
    mcu.pc = 0x2754;
}

/* 0x2754 BEQ 0x0097 -> 0x27ee. */
void step_note_on_setup_beq_0x0097_0x27ee(void)
{
    beq(0x27ee, 0x2757);
}

/* 0x2757 MOVG2 @r0+0x009b r3. */
void step_note_on_setup_movg2_r0_0x009b_r3(void)
{
    load8(mcu.r[3], ea_r(0, 0x009b));
    mcu.pc = 0x275b;
}

/* 0x275b STC r4 -> r4. */
void step_note_on_setup_stc_r4_r4(void)
{
    stc8_reg(4, mcu.r[4]);
    mcu.pc = 0x275d;
}

/* 0x275d movi r1 #0x001b. */
void step_note_on_setup_movi_r1_0x001b(void)
{
    movi16(mcu.r[1], 0x001b);
    mcu.pc = 0x2760;
}

/* 0x2760 MOVG2 r1 r2. */
void step_note_on_setup_movg2_r1_r2(void)
{
    mov16_reg(mcu.r[2], mcu.r[1]);
    mcu.pc = 0x2762;
}

/* 0x2762 ADD r2 r2. */
void step_note_on_setup_add_r2_r2(void)
{
    add16_reg(mcu.r[2], mcu.r[2]);
    mcu.pc = 0x2764;
}

/* 0x2764 MOVG2 @r2+0x64d6 r2. */
void step_note_on_setup_movg2_r2_0x64d6_r2(void)
{
    load16(mcu.r[2], ea_r(2, 0x64d6));
    mcu.pc = 0x2768;
}

/* 0x2768 CMP r0 r2. */
void step_note_on_setup_cmp_r0_r2(void)
{
    cmp16_reg(mcu.r[2], mcu.r[0]);
    mcu.pc = 0x276a;
}

/* 0x276a BEQ 25 -> 0x2785. */
void step_note_on_setup_beq_25_0x2785(void)
{
    beq(0x2785, 0x276c);
}

/* 0x276c SUB @r2+0 #0x000c. */
void step_note_on_setup_sub_r2_0_0x000c(void)
{
    sub16_mem_imm16(ea_r(2, (uint16_t)(int8_t)0x00), 0x000c);
    mcu.pc = 0x2771;
}

/* 0x2771 BHI 18 -> 0x2785. */
void step_note_on_setup_bhi_18_0x2785(void)
{
    bhi(0x2785, 0x2773);
}

/* 0x2773 CMP @r2+0x009b r3. */
void step_note_on_setup_cmp_r2_0x009b_r3(void)
{
    cmp8_mem_reg(ea_r(2, 0x009b), mcu.r[3]);
    mcu.pc = 0x2777;
}

/* 0x2777 BNE 12 -> 0x2785. */
void step_note_on_setup_bne_12_0x2785(void)
{
    bne(0x2785, 0x2779);
}

/* 0x2779 CMP @r2+0x0099 r4. */
void step_note_on_setup_cmp_r2_0x0099_r4(void)
{
    cmp8_mem_reg(ea_r(2, 0x0099), mcu.r[4]);
    mcu.pc = 0x277d;
}

/* 0x277d BNE 6 -> 0x2785. */
void step_note_on_setup_bne_6_0x2785(void)
{
    bne(0x2785, 0x277f);
}

/* 0x277f CMP @r2+0x009e r5. */
void step_note_on_setup_cmp_r2_0x009e_r5(void)
{
    cmp16_mem_reg(ea_r(2, 0x009e), mcu.r[5]);
    mcu.pc = 0x2783;
}

/* 0x2783 BEQ 5 -> 0x278a. */
void step_note_on_setup_beq_5_0x278a(void)
{
    beq(0x278a, 0x2785);
}

/* 0x2785 cntjmp r1 -40 -> 0x2760. */
void step_note_on_setup_cntjmp_r1_m40_0x2760(void)
{
    cntjmp(mcu.r[1], 0x2760, 0x2788);
}

/* 0x2788 BRA 100 -> 0x27ee. */
void step_note_on_setup_bra_100_0x27ee(void)
{
    mcu.pc = 0x27ee;
}

/* 0x278a MOVG2 @r2+-88 r6. */
void step_note_on_setup_movg2_r2_m88_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xa8));
    mcu.pc = 0x278d;
}

/* 0x278d MOVG3 r6 -> @r0+-88. */
void step_note_on_setup_movg3_r6_r0_m88(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xa8), mcu.r[6]);
    mcu.pc = 0x2790;
}

/* 0x2790 MOVG2 @r2+-86 r6. */
void step_note_on_setup_movg2_r2_m86_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xaa));
    mcu.pc = 0x2793;
}

/* 0x2793 MOVG3 r6 -> @r0+-86. */
void step_note_on_setup_movg3_r6_r0_m86(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xaa), mcu.r[6]);
    mcu.pc = 0x2796;
}

/* 0x2796 MOVG2 @r2+-84 r6. */
void step_note_on_setup_movg2_r2_m84_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xac));
    mcu.pc = 0x2799;
}

/* 0x2799 MOVG3 r6 -> @r0+-84. */
void step_note_on_setup_movg3_r6_r0_m84(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xac), mcu.r[6]);
    mcu.pc = 0x279c;
}

/* 0x279c MOVG2 @r2+-80 r6. */
void step_note_on_setup_movg2_r2_m80_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xb0));
    mcu.pc = 0x279f;
}

/* 0x279f MOVG3 r6 -> @r0+-80. */
void step_note_on_setup_movg3_r6_r0_m80(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb0), mcu.r[6]);
    mcu.pc = 0x27a2;
}

/* 0x27a2 MOVG2 @r2+-82 r6. */
void step_note_on_setup_movg2_r2_m82_r6(void)
{
    load8(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xae));
    mcu.pc = 0x27a5;
}

/* 0x27a5 MOVG3 r6 -> @r0+-82. */
void step_note_on_setup_movg3_r6_r0_m82(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0xae), mcu.r[6]);
    mcu.pc = 0x27a8;
}

/* 0x27a8 MOVG2 @r2+-78 r6. */
void step_note_on_setup_movg2_r2_m78_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xb2));
    mcu.pc = 0x27ab;
}

/* 0x27ab MOVG3 r6 -> @r0+-78. */
void step_note_on_setup_movg3_r6_r0_m78(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb2), mcu.r[6]);
    mcu.pc = 0x27ae;
}

/* 0x27ae MOVG2 @r2+-76 r6. */
void step_note_on_setup_movg2_r2_m76_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xb4));
    mcu.pc = 0x27b1;
}

/* 0x27b1 MOVG3 r6 -> @r0+-76. */
void step_note_on_setup_movg3_r6_r0_m76(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb4), mcu.r[6]);
    mcu.pc = 0x27b4;
}

/* 0x27b4 MOVG2 @r2+-74 r6. */
void step_note_on_setup_movg2_r2_m74_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xb6));
    mcu.pc = 0x27b7;
}

/* 0x27b7 MOVG3 r6 -> @r0+-74. */
void step_note_on_setup_movg3_r6_r0_m74(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb6), mcu.r[6]);
    mcu.pc = 0x27ba;
}

/* 0x27ba MOVG2 @r2+-72 r6. */
void step_note_on_setup_movg2_r2_m72_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xb8));
    mcu.pc = 0x27bd;
}

/* 0x27bd MOVG3 r6 -> @r0+-72. */
void step_note_on_setup_movg3_r6_r0_m72(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb8), mcu.r[6]);
    mcu.pc = 0x27c0;
}

/* 0x27c0 MOVG2 @r2+-70 r6. */
void step_note_on_setup_movg2_r2_m70_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xba));
    mcu.pc = 0x27c3;
}

/* 0x27c3 MOVG3 r6 -> @r0+-70. */
void step_note_on_setup_movg3_r6_r0_m70(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xba), mcu.r[6]);
    mcu.pc = 0x27c6;
}

/* 0x27c6 MOVG2 @r2+-68 r6. */
void step_note_on_setup_movg2_r2_m68_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xbc));
    mcu.pc = 0x27c9;
}

/* 0x27c9 MOVG3 r6 -> @r0+-68. */
void step_note_on_setup_movg3_r6_r0_m68(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xbc), mcu.r[6]);
    mcu.pc = 0x27cc;
}

/* 0x27cc MOVG2 @r2+-66 r6. */
void step_note_on_setup_movg2_r2_m66_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xbe));
    mcu.pc = 0x27cf;
}

/* 0x27cf MOVG3 r6 -> @r0+-66. */
void step_note_on_setup_movg3_r6_r0_m66(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xbe), mcu.r[6]);
    mcu.pc = 0x27d2;
}

/* 0x27d2 MOVG2 @r2+-64 r6. */
void step_note_on_setup_movg2_r2_m64_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xc0));
    mcu.pc = 0x27d5;
}

/* 0x27d5 MOVG3 r6 -> @r0+-64. */
void step_note_on_setup_movg3_r6_r0_m64(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xc0), mcu.r[6]);
    mcu.pc = 0x27d8;
}

/* 0x27d8 MOVG2 @r2+-62 r6. */
void step_note_on_setup_movg2_r2_m62_r6(void)
{
    load16(mcu.r[6], ea_r(2, (uint16_t)(int8_t)0xc2));
    mcu.pc = 0x27db;
}

/* 0x27db MOVG3 r6 -> @r0+-62. */
void step_note_on_setup_movg3_r6_r0_m62(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xc2), mcu.r[6]);
    mcu.pc = 0x27de;
}

/* 0x27de MOVG2 @r0+-2 r1. */
void step_note_on_setup_movg2_r0_m2_r1_b(void)
{
    load16(mcu.r[1], ea_r(0, (uint16_t)(int8_t)0xfe));
    mcu.pc = 0x27e1;
}

/* 0x27e1 ADD r1 r1. */
void step_note_on_setup_add_r1_r1_b(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x27e3;
}

/* 0x27e3 MOVG3 r2 -> @r1+0xcdfe. */
void step_note_on_setup_movg3_r2_r1_0xcdfe(void)
{
    store16(ea_r(1, 0xcdfe), mcu.r[2]);
    mcu.pc = 0x27e7;
}

/* 0x27e7 MOVG #0xff -> @r0+-81. */
void step_note_on_setup_movg_0xff_r0_m81(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xaf), 0xff);
    mcu.pc = 0x27eb;
}

/* 0x27eb BRA 0x0086 -> 0x2874. */
void step_note_on_setup_bra_0x0086_0x2874(void)
{
    mcu.pc = 0x2874;
}

/* 0x27ee MOVG2 r6 r3. */
void step_note_on_setup_movg2_r6_r3(void)
{
    mov8_reg(mcu.r[3], mcu.r[6]);
    mcu.pc = 0x27f0;
}

/* 0x27f0 AND #0x00c0 r3. */
void step_note_on_setup_and_0x00c0_r3(void)
{
    and_imm16_reg(mcu.r[3], 0x00c0);
    mcu.pc = 0x27f4;
}

/* 0x27f4 SWAP r3. */
void step_note_on_setup_swap_r3(void)
{
    swap16(mcu.r[3]);
    mcu.pc = 0x27f6;
}

/* 0x27f6 MOVG3 r3 -> @r0+-72. */
void step_note_on_setup_movg3_r3_r0_m72(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb8), mcu.r[3]);
    mcu.pc = 0x27f9;
}

/* 0x27f9 MOVG2 r6 r3. */
void step_note_on_setup_movg2_r6_r3_b(void)
{
    mov8_reg(mcu.r[3], mcu.r[6]);
    mcu.pc = 0x27fb;
}

/* 0x27fb AND #0x000f r3. */
void step_note_on_setup_and_0x000f_r3(void)
{
    and_imm16_reg(mcu.r[3], 0x000f);
    mcu.pc = 0x27ff;
}

/* 0x27ff MOVG2 @r3+0x7207 r3. */
void step_note_on_setup_movg2_r3_0x7207_r3(void)
{
    load8(mcu.r[3], ea_r(3, 0x7207));
    mcu.pc = 0x2803;
}

/* 0x2803 MOVG3 r3 -> @r0+-74. */
void step_note_on_setup_movg3_r3_r0_m74(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb6), mcu.r[3]);
    mcu.pc = 0x2806;
}

/* 0x2806 CLR @r0+-64. */
void step_note_on_setup_clr_r0_m64(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0xc0));
    mcu.pc = 0x2809;
}

/* 0x2809 CLR @r0+-70. */
void step_note_on_setup_clr_r0_m70(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0xba));
    mcu.pc = 0x280c;
}

/* 0x280c CLR @r0+-68. */
void step_note_on_setup_clr_r0_m68(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0xbc));
    mcu.pc = 0x280f;
}

/* 0x280f CLR @r0+-62. */
void step_note_on_setup_clr_r0_m62(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0xc2));
    mcu.pc = 0x2812;
}

/* 0x2812 CLR @r0+-88. */
void step_note_on_setup_clr_r0_m88(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0xa8));
    mcu.pc = 0x2815;
}

/* 0x2815 CLR @r0+-86. */
void step_note_on_setup_clr_r0_m86(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0xaa));
    mcu.pc = 0x2818;
}

/* 0x2818 CLR @r0+-84. */
void step_note_on_setup_clr_r0_m84(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0xac));
    mcu.pc = 0x281b;
}

/* 0x281b CLR r3. */
void step_note_on_setup_clr_r3(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x281d;
}

/* 0x281d MOVG2 @r5+6 r3. */
void step_note_on_setup_movg2_r5_6_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x06));
    mcu.pc = 0x2820;
}

/* 0x2820 BMI 8 -> 0x282a. */
void step_note_on_setup_bmi_8_0x282a(void)
{
    bmi(0x282a, 0x2822);
}

/* 0x2822 ADD r3 r3. */
void step_note_on_setup_add_r3_r3(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2824;
}

/* 0x2824 MOVG2 @r3+0x6e86 r3. */
void step_note_on_setup_movg2_r3_0x6e86_r3(void)
{
    load16(mcu.r[3], ea_r(3, 0x6e86));
    mcu.pc = 0x2828;
}

/* 0x2828 BRA 2 -> 0x282c. */
void step_note_on_setup_bra_2_0x282c(void)
{
    mcu.pc = 0x282c;
}

/* 0x282a CLR r3. */
void step_note_on_setup_clr_r3_b(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x282c;
}

/* 0x282c MOVG3 r3 -> @r0+-78. */
void step_note_on_setup_movg3_r3_r0_m78(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb2), mcu.r[3]);
    mcu.pc = 0x282f;
}

/* 0x282f CLR r3. */
void step_note_on_setup_clr_r3_c(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2831;
}

/* 0x2831 MOVG2 @r5+7 r3. */
void step_note_on_setup_movg2_r5_7_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x07));
    mcu.pc = 0x2834;
}

/* 0x2834 ADD r3 r3. */
void step_note_on_setup_add_r3_r3_b(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2836;
}

/* 0x2836 MOVG2 @r3+0x6e86 r3. */
void step_note_on_setup_movg2_r3_0x6e86_r3_b(void)
{
    load16(mcu.r[3], ea_r(3, 0x6e86));
    mcu.pc = 0x283a;
}

/* 0x283a MOVG3 r3 -> @r0+-76. */
void step_note_on_setup_movg3_r3_r0_m76(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xb4), mcu.r[3]);
    mcu.pc = 0x283d;
}

/* 0x283d CLR r3. */
void step_note_on_setup_clr_r3_d(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x283f;
}

/* 0x283f MOVG2 @r5+5 r3. */
void step_note_on_setup_movg2_r5_5_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x05));
    mcu.pc = 0x2842;
}

/* 0x2842 MOVG3 r3 -> @r0+-82. */
void step_note_on_setup_movg3_r3_r0_m82(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0xae), mcu.r[3]);
    mcu.pc = 0x2845;
}

/* 0x2845 MOVG2 (dp,0xad2a) r6. */
void step_note_on_setup_movg2_dp_0xad2a_r6(void)
{
    load16(mcu.r[6], ea_dp(0xad2a));
    mcu.pc = 0x2849;
}

/* 0x2849 MOVG3 r6 -> (dp,0xad2c). */
void step_note_on_setup_movg3_r6_dp_0xad2c(void)
{
    store16(ea_dp(0xad2c), mcu.r[6]);
    mcu.pc = 0x284d;
}

/* 0x284d MOVG #0x0001 -> (dp,0xad2a). */
void step_note_on_setup_movg_0x0001_dp_0xad2a(void)
{
    mov_imm16_mem(ea_dp(0xad2a), 0x0001);
    mcu.pc = 0x2853;
}

/* 0x2853 BSET_ORC #0x0700 r0. */
void step_note_on_setup_bset_orc_0x0700_r0(void)
{
    orc_imm16(0x0700);
    mcu.pc = 0x2857;
}

/* 0x2857 MOVG #0x1e -> (br,$3e). */
void step_note_on_setup_movg_0x1e_br_3e(void)
{
    mov_imm8_mem(ea_br(0x3e), 0x1e);
    mcu.pc = 0x285b;
}

/* 0x285b movl r4 @(br,$34). */
void step_note_on_setup_movl_r4_br_34(void)
{
    movl_br(mcu.r[4], 0x34);
    mcu.pc = 0x285d;
}

/* 0x285d movlw r4 @(br,$3a). */
void step_note_on_setup_movlw_r4_br_3a(void)
{
    movlw_br(mcu.r[4], 0x3a);
    mcu.pc = 0x285f;
}

/* 0x285f MOVG3 r4 -> @r0+-66. */
void step_note_on_setup_movg3_r4_r0_m66(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xbe), mcu.r[4]);
    mcu.pc = 0x2862;
}

/* 0x2862 MOVG3 r4 -> @r0+-64. */
void step_note_on_setup_movg3_r4_r0_m64(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xc0), mcu.r[4]);
    mcu.pc = 0x2865;
}

/* 0x2865 bsr16 -> 0x37fe. */
void step_note_on_setup_bsr16_0x37fe(void)
{
    call(0x2868, 0x37fe);
}

/* 0x2868 BCLR_ANDC #0xf8ff r0. */
void step_note_on_setup_bclr_andc_0xf8ff_r0(void)
{
    andc_imm16(0xf8ff);
    mcu.pc = 0x286c;
}

/* 0x286c MOVG2 (dp,0xad2c) r6. */
void step_note_on_setup_movg2_dp_0xad2c_r6(void)
{
    load16(mcu.r[6], ea_dp(0xad2c));
    mcu.pc = 0x2870;
}

/* 0x2870 MOVG3 r6 -> (dp,0xad2a). */
void step_note_on_setup_movg3_r6_dp_0xad2a(void)
{
    store16(ea_dp(0xad2a), mcu.r[6]);
    mcu.pc = 0x2874;
}

/* 0x2874 LDC @r0+0x009a r4. */
void step_note_on_setup_ldc_r0_0x009a_r4(void)
{
    ldc8_mem(4, ea_r(0, 0x009a));
    mcu.pc = 0x2878;
}

/* 0x2878 MOVG2 @r0+0x00a0 r5. */
void step_note_on_setup_movg2_r0_0x00a0_r5(void)
{
    load16(mcu.r[5], ea_r(0, 0x00a0));
    mcu.pc = 0x287c;
}

/* 0x287c MOVG2 @r5+1 r3. */
void step_note_on_setup_movg2_r5_1_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x01));
    mcu.pc = 0x287f;
}

/* 0x287f MOVG3 r3 -> @r0+-20. */
void step_note_on_setup_movg3_r3_r0_m20(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0xec), mcu.r[3]);
    mcu.pc = 0x2882;
}

/* 0x2882 MOVG2 @r5+2 r4. */
void step_note_on_setup_movg2_r5_2_r4(void)
{
    load16(mcu.r[4], ea_r(5, (uint16_t)(int8_t)0x02));
    mcu.pc = 0x2885;
}

/* 0x2885 MOVG3 r4 -> @r0+-16. */
void step_note_on_setup_movg3_r4_r0_m16(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xf0), mcu.r[4]);
    mcu.pc = 0x2888;
}

/* 0x2888 MOVG2 @r0+-2 r1. */
void step_note_on_setup_movg2_r0_m2_r1_c(void)
{
    load16(mcu.r[1], ea_r(0, (uint16_t)(int8_t)0xfe));
    mcu.pc = 0x288b;
}

/* 0x288b BTSTI @r0+-59 #7. */
void step_note_on_setup_btsti_r0_m59_7_b(void)
{
    btsti(ea_r(0, (uint16_t)(int8_t)0xc5), 7);
    mcu.pc = 0x288e;
}

/* 0x288e BNE 15 -> 0x289f. */
void step_note_on_setup_bne_15_0x289f(void)
{
    bne(0x289f, 0x2890);
}

/* 0x2890 MOVG2 @r5+4 r6. */
void step_note_on_setup_movg2_r5_4_r6_b(void)
{
    load16(mcu.r[6], ea_r(5, (uint16_t)(int8_t)0x04));
    mcu.pc = 0x2893;
}

/* 0x2893 CLR r2. */
void step_note_on_setup_clr_r2(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x2895;
}

/* 0x2895 ADD r4 r6. */
void step_note_on_setup_add_r4_r6(void)
{
    add16_reg(mcu.r[6], mcu.r[4]);
    mcu.pc = 0x2897;
}

/* 0x2897 ADDX r3 r2. */
void step_note_on_setup_addx_r3_r2(void)
{
    addx8_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2899;
}

/* 0x2899 MOVG3 r2 -> @r0+-20. */
void step_note_on_setup_movg3_r2_r0_m20(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0xec), mcu.r[2]);
    mcu.pc = 0x289c;
}

/* 0x289c MOVG3 r6 -> @r0+-16. */
void step_note_on_setup_movg3_r6_r0_m16(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xf0), mcu.r[6]);
    mcu.pc = 0x289f;
}

/* 0x289f ADD @r5+6 r4. */
void step_note_on_setup_add_r5_6_r4(void)
{
    add16_mem(mcu.r[4], ea_r(5, (uint16_t)(int8_t)0x06));
    mcu.pc = 0x28a2;
}

/* 0x28a2 ADDX #0x00 r3. */
void step_note_on_setup_addx_0x00_r3(void)
{
    addx8_imm(mcu.r[3], 0x00);
    mcu.pc = 0x28a5;
}

/* 0x28a5 MOVG3 r3 -> @r0+-19. */
void step_note_on_setup_movg3_r3_r0_m19(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0xed), mcu.r[3]);
    mcu.pc = 0x28a8;
}

/* 0x28a8 MOVG3 r4 -> @r0+-14. */
void step_note_on_setup_movg3_r4_r0_m14(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xf2), mcu.r[4]);
    mcu.pc = 0x28ab;
}

/* 0x28ab SUB @r5+8 r4. */
void step_note_on_setup_sub_r5_8_r4(void)
{
    sub16_mem(mcu.r[4], ea_r(5, (uint16_t)(int8_t)0x08));
    mcu.pc = 0x28ae;
}

/* 0x28ae SUBX #0x00 r3. */
void step_note_on_setup_subx_0x00_r3(void)
{
    subx8_imm(mcu.r[3], 0x00);
    mcu.pc = 0x28b1;
}

/* 0x28b1 MOVG3 r3 -> @r0+-18. */
void step_note_on_setup_movg3_r3_r0_m18(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0xee), mcu.r[3]);
    mcu.pc = 0x28b4;
}

/* 0x28b4 MOVG3 r4 -> @r0+-12. */
void step_note_on_setup_movg3_r4_r0_m12(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xf4), mcu.r[4]);
    mcu.pc = 0x28b7;
}

/* 0x28b7 ADD r3 r3. */
void step_note_on_setup_add_r3_r3_c(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x28b9;
}

/* 0x28b9 ADD r3 r3. */
void step_note_on_setup_add_r3_r3_d(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x28bb;
}

/* 0x28bb ADD r3 r3. */
void step_note_on_setup_add_r3_r3_e(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x28bd;
}

/* 0x28bd ADD r3 r3. */
void step_note_on_setup_add_r3_r3_f(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x28bf;
}

/* 0x28bf MOVG2 @r5+10 r4. */
void step_note_on_setup_movg2_r5_10_r4(void)
{
    load8(mcu.r[4], ea_r(5, (uint16_t)(int8_t)0x0a));
    mcu.pc = 0x28c2;
}

/* 0x28c2 MOVG2 r4 r5. */
void step_note_on_setup_movg2_r4_r5(void)
{
    mov8_reg(mcu.r[5], mcu.r[4]);
    mcu.pc = 0x28c4;
}

/* 0x28c4 AND #0x0001 r4. */
void step_note_on_setup_and_0x0001_r4(void)
{
    and_imm16_reg(mcu.r[4], 0x0001);
    mcu.pc = 0x28c8;
}

/* 0x28c8 SWAP r4. */
void step_note_on_setup_swap_r4(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x28ca;
}

/* 0x28ca SHLR r4. */
void step_note_on_setup_shlr_r4(void)
{
    shlr16(mcu.r[4]);
    mcu.pc = 0x28cc;
}

/* 0x28cc SHLR r4. */
void step_note_on_setup_shlr_r4_b(void)
{
    shlr16(mcu.r[4]);
    mcu.pc = 0x28ce;
}

/* 0x28ce MOVG2 r4 r3. */
void step_note_on_setup_movg2_r4_r3(void)
{
    mov8_reg(mcu.r[3], mcu.r[4]);
    mcu.pc = 0x28d0;
}

/* 0x28d0 OR r1 r3. */
void step_note_on_setup_or_r1_r3(void)
{
    or8_reg(mcu.r[3], mcu.r[1]);
    mcu.pc = 0x28d2;
}

/* 0x28d2 MOVG3 r3 -> @r0+-10. */
void step_note_on_setup_movg3_r3_r0_m10(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xf6), mcu.r[3]);
    mcu.pc = 0x28d5;
}

/* 0x28d5 AND #0x02 r5. */
void step_note_on_setup_and_0x02_r5(void)
{
    and_imm8_reg(mcu.r[5], 0x02);
    mcu.pc = 0x28d8;
}

/* 0x28d8 MOVG3 r5 -> @r0+-17. */
void step_note_on_setup_movg3_r5_r0_m17(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0xef), mcu.r[5]);
    mcu.pc = 0x28db;
}

/* 0x28db BTSTI @r0+-59 #7. */
void step_note_on_setup_btsti_r0_m59_7_c(void)
{
    btsti(ea_r(0, (uint16_t)(int8_t)0xc5), 7);
    mcu.pc = 0x28de;
}

/* 0x28de BNE 69 -> 0x2925. */
void step_note_on_setup_bne_69_0x2925(void)
{
    bne(0x2925, 0x28e0);
}

/* 0x28e0 BSET_ORC #0x0700 r0. */
void step_note_on_setup_bset_orc_0x0700_r0_b(void)
{
    orc_imm16(0x0700);
    mcu.pc = 0x28e4;
}

/* 0x28e4 movs r1 @(br,$3e). */
void step_note_on_setup_movs_r1_br_3e(void)
{
    movs_br(mcu.r[1], 0x3e);
    mcu.pc = 0x28e6;
}

/* 0x28e6 MOVG #0x00b4 -> (br,$16). */
void step_note_on_setup_movg_0x00b4_br_16(void)
{
    mov_imm16_mem(ea_br(0x16), 0x00b4);
    mcu.pc = 0x28eb;
}

/* 0x28eb MOVG #0x00b4 -> @r0+26. */
void step_note_on_setup_movg_0x00b4_r0_26(void)
{
    mov_imm16_mem(ea_r(0, (uint16_t)(int8_t)0x1a), 0x00b4);
    mcu.pc = 0x28f0;
}

/* 0x28f0 MOVG2 @r0+28 r6. */
void step_note_on_setup_movg2_r0_28_r6(void)
{
    load16(mcu.r[6], ea_r(0, (uint16_t)(int8_t)0x1c));
    mcu.pc = 0x28f3;
}

/* 0x28f3 move r6 #0xaf. */
void step_note_on_setup_move_r6_0xaf(void)
{
    move8_imm(mcu.r[6], 0xaf);
    mcu.pc = 0x28f5;
}

/* 0x28f5 MOVG3 r6 -> @r0+30. */
void step_note_on_setup_movg3_r6_r0_30(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x1e), mcu.r[6]);
    mcu.pc = 0x28f8;
}

/* 0x28f8 MOVG2 @r0+36 r6. */
void step_note_on_setup_movg2_r0_36_r6(void)
{
    load16(mcu.r[6], ea_r(0, (uint16_t)(int8_t)0x24));
    mcu.pc = 0x28fb;
}

/* 0x28fb MOVG3 r6 -> @r0+-22. */
void step_note_on_setup_movg3_r6_r0_m22(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xea), mcu.r[6]);
    mcu.pc = 0x28fe;
}

/* 0x28fe move r6 #0xaf. */
void step_note_on_setup_move_r6_0xaf_b(void)
{
    move8_imm(mcu.r[6], 0xaf);
    mcu.pc = 0x2900;
}

/* 0x2900 MOVG3 r6 -> @r0+38. */
void step_note_on_setup_movg3_r6_r0_38(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x26), mcu.r[6]);
    mcu.pc = 0x2903;
}

/* 0x2903 MOVG3 r6 -> @r0+-24. */
void step_note_on_setup_movg3_r6_r0_m24(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xe8), mcu.r[6]);
    mcu.pc = 0x2906;
}

/* 0x2906 MOVG2 @r0+0 r6. */
void step_note_on_setup_movg2_r0_0_r6(void)
{
    load16(mcu.r[6], ea_r(0, (uint16_t)(int8_t)0x00));
    mcu.pc = 0x2909;
}

/* 0x2909 cmp r6,w #0x0018. */
void step_note_on_setup_cmp_r6_w_0x0018(void)
{
    cmp_imm16_reg(mcu.r[6], 0x0018);
    mcu.pc = 0x290c;
}

/* 0x290c BEQ 8 -> 0x2916. */
void step_note_on_setup_beq_8_0x2916(void)
{
    beq(0x2916, 0x290e);
}

/* 0x290e MOVG #0x0018 -> @r0+0. */
void step_note_on_setup_movg_0x0018_r0_0(void)
{
    mov_imm16_mem(ea_r(0, (uint16_t)(int8_t)0x00), 0x0018);
    mcu.pc = 0x2913;
}

/* 0x2913 MOVG3 r6 -> @r0+6. */
void step_note_on_setup_movg3_r6_r0_6(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x06), mcu.r[6]);
    mcu.pc = 0x2916;
}

/* 0x2916 BCLR_ANDC #0xf8ff r0. */
void step_note_on_setup_bclr_andc_0xf8ff_r0_b(void)
{
    andc_imm16(0xf8ff);
    mcu.pc = 0x291a;
}

/* 0x291a SUB @r1+0xd0e0 #0x00. */
void step_note_on_setup_sub_r1_0xd0e0_0x00(void)
{
    sub8_mem_imm8(ea_r(1, 0xd0e0), 0x00);
    mcu.pc = 0x291f;
}

/* 0x291f BNE 0x2b70 -> 0x5492. */
void step_note_on_setup_bne_0x2b70_0x5492(void)
{
    bne(0x5492, 0x2922);
}

/* 0x2922 BRA 0x1ca7 -> 0x45cc. */
void step_note_on_setup_bra_0x1ca7_0x45cc(void)
{
    mcu.pc = 0x45cc;
}

/* 0x2925 MOVG #0xffff -> @r0+6. */
void step_note_on_setup_movg_0xffff_r0_6(void)
{
    mov_imm16_mem(ea_r(0, (uint16_t)(int8_t)0x06), 0xffff);
    mcu.pc = 0x292a;
}

/* 0x292a LDC @r0+0x0099 r4. */
void step_note_on_setup_ldc_r0_0x0099_r4_b(void)
{
    ldc8_mem(4, ea_r(0, 0x0099));
    mcu.pc = 0x292e;
}

/* 0x292e MOVG2 @r0+0x009e r5. */
void step_note_on_setup_movg2_r0_0x009e_r5_b(void)
{
    load16(mcu.r[5], ea_r(0, 0x009e));
    mcu.pc = 0x2932;
}

/* 0x2932 movi r2 #0x00ff. */
void step_note_on_setup_movi_r2_0x00ff(void)
{
    movi16(mcu.r[2], 0x00ff);
    mcu.pc = 0x2935;
}

/* 0x2935 CLR r3. */
void step_note_on_setup_clr_r3_e(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2937;
}

/* 0x2937 MOVG2 @r5+69 r3. */
void step_note_on_setup_movg2_r5_69_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x45));
    mcu.pc = 0x293a;
}

/* 0x293a SUB @r3+0x6883 r2. */
void step_note_on_setup_sub_r3_0x6883_r2(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x6883));
    mcu.pc = 0x293e;
}

/* 0x293e BHI 2 -> 0x2942. */
void step_note_on_setup_bhi_2_0x2942(void)
{
    bhi(0x2942, 0x2940);
}

/* 0x2940 move r2 #0x01. */
void step_note_on_setup_move_r2_0x01(void)
{
    move8_imm(mcu.r[2], 0x01);
    mcu.pc = 0x2942;
}

/* 0x2942 CLR r4. */
void step_note_on_setup_clr_r4(void)
{
    clr_reg(mcu.r[4]);
    mcu.pc = 0x2944;
}

/* 0x2944 MOVG2 @r0+-2 r1. */
void step_note_on_setup_movg2_r0_m2_r1_d(void)
{
    load16(mcu.r[1], ea_r(0, (uint16_t)(int8_t)0xfe));
    mcu.pc = 0x2947;
}

/* 0x2947 MOVG2 @r1+0xd134 r4. */
void step_note_on_setup_movg2_r1_0xd134_r4(void)
{
    load8(mcu.r[4], ea_r(1, 0xd134));
    mcu.pc = 0x294b;
}

/* 0x294b LDC #0x03 r5. */
void step_note_on_setup_ldc_0x03_r5(void)
{
    ldc_imm8(5, 0x03);
    mcu.pc = 0x294e;
}

/* 0x294e MOVG2 @r5+70 r3. */
void step_note_on_setup_movg2_r5_70_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x46));
    mcu.pc = 0x2951;
}

/* 0x2951 ADD r3 r3. */
void step_note_on_setup_add_r3_r3_g(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2953;
}

/* 0x2953 MOVG2 @r3+0xdd7c r3. */
void step_note_on_setup_movg2_r3_0xdd7c_r3(void)
{
    load16(mcu.r[3], ea_r(3, 0xdd7c));
    mcu.pc = 0x2957;
}

/* 0x2957 ADD r4 r3. */
void step_note_on_setup_add_r4_r3(void)
{
    add16_reg(mcu.r[3], mcu.r[4]);
    mcu.pc = 0x2959;
}

/* 0x2959 MOVG2 @r3 r4. */
void step_note_on_setup_movg2_r3_r4(void)
{
    load8(mcu.r[4], ea_r(3, 0));
    mcu.pc = 0x295b;
}

/* 0x295b LDC #0x00 r5. */
void step_note_on_setup_ldc_0x00_r5(void)
{
    ldc_imm8(5, 0x00);
    mcu.pc = 0x295e;
}

/* 0x295e CLR r3. */
void step_note_on_setup_clr_r3_f(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2960;
}

/* 0x2960 MOVG2 @r5+71 r3. */
void step_note_on_setup_movg2_r5_71_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x47));
    mcu.pc = 0x2963;
}

/* 0x2963 SUB #0x40 r3. */
void step_note_on_setup_sub_0x40_r3(void)
{
    sub_imm8_reg(mcu.r[3], 0x40);
    mcu.pc = 0x2966;
}

/* 0x2966 BCC 11 -> 0x2973. */
void step_note_on_setup_bcc_11_0x2973(void)
{
    bcc(0x2973, 0x2968);
}

/* 0x2968 NEG r3. */
void step_note_on_setup_neg_r3(void)
{
    neg8_reg(mcu.r[3]);
    mcu.pc = 0x296a;
}

/* 0x296a SUB #0x80 r4. */
void step_note_on_setup_sub_0x80_r4(void)
{
    sub_imm8_reg(mcu.r[4], 0x80);
    mcu.pc = 0x296d;
}

/* 0x296d BCC 11 -> 0x297a. */
void step_note_on_setup_bcc_11_0x297a(void)
{
    bcc(0x297a, 0x296f);
}

/* 0x296f NEG r4. */
void step_note_on_setup_neg_r4(void)
{
    neg8_reg(mcu.r[4]);
    mcu.pc = 0x2971;
}

/* 0x2971 BRA 28 -> 0x298f. */
void step_note_on_setup_bra_28_0x298f(void)
{
    mcu.pc = 0x298f;
}

/* 0x2973 SUB #0x80 r4. */
void step_note_on_setup_sub_0x80_r4_b(void)
{
    sub_imm8_reg(mcu.r[4], 0x80);
    mcu.pc = 0x2976;
}

/* 0x2976 BCC 23 -> 0x298f. */
void step_note_on_setup_bcc_23_0x298f(void)
{
    bcc(0x298f, 0x2978);
}

/* 0x2978 NEG r4. */
void step_note_on_setup_neg_r4_b(void)
{
    neg8_reg(mcu.r[4]);
    mcu.pc = 0x297a;
}

/* 0x297a MULXU @r3+0x650e r4. */
void step_note_on_setup_mulxu_r3_0x650e_r4(void)
{
    mulxu8_mem(mcu.r[4], ea_r(3, 0x650e));
    mcu.pc = 0x297e;
}

/* 0x297e ADD r4 r4. */
void step_note_on_setup_add_r4_r4(void)
{
    add16_reg(mcu.r[4], mcu.r[4]);
    mcu.pc = 0x2980;
}

/* 0x2980 SWAP r4. */
void step_note_on_setup_swap_r4_b(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x2982;
}

/* 0x2982 MOVG2 r4 r3. */
void step_note_on_setup_movg2_r4_r3_b(void)
{
    mov8_reg(mcu.r[3], mcu.r[4]);
    mcu.pc = 0x2984;
}

/* 0x2984 SUB @r3+0x673a r2. */
void step_note_on_setup_sub_r3_0x673a_r2(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x673a));
    mcu.pc = 0x2988;
}

/* 0x2988 BHI 24 -> 0x29a2. */
void step_note_on_setup_bhi_24_0x29a2(void)
{
    bhi(0x29a2, 0x298a);
}

/* 0x298a movi r2 #0x0001. */
void step_note_on_setup_movi_r2_0x0001(void)
{
    movi16(mcu.r[2], 0x0001);
    mcu.pc = 0x298d;
}

/* 0x298d BRA 19 -> 0x29a2. */
void step_note_on_setup_bra_19_0x29a2(void)
{
    mcu.pc = 0x29a2;
}

/* 0x298f MULXU @r3+0x650e r4. */
void step_note_on_setup_mulxu_r3_0x650e_r4_b(void)
{
    mulxu8_mem(mcu.r[4], ea_r(3, 0x650e));
    mcu.pc = 0x2993;
}

/* 0x2993 ADD r4 r4. */
void step_note_on_setup_add_r4_r4_b(void)
{
    add16_reg(mcu.r[4], mcu.r[4]);
    mcu.pc = 0x2995;
}

/* 0x2995 SWAP r4. */
void step_note_on_setup_swap_r4_c(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x2997;
}

/* 0x2997 MOVG2 r4 r3. */
void step_note_on_setup_movg2_r4_r3_c(void)
{
    mov8_reg(mcu.r[3], mcu.r[4]);
    mcu.pc = 0x2999;
}

/* 0x2999 ADD @r3+0x673a r2. */
void step_note_on_setup_add_r3_0x673a_r2(void)
{
    add8_mem(mcu.r[2], ea_r(3, 0x673a));
    mcu.pc = 0x299d;
}

/* 0x299d BCC 3 -> 0x29a2. */
void step_note_on_setup_bcc_3_0x29a2(void)
{
    bcc(0x29a2, 0x299f);
}

/* 0x299f movi r2 #0x00ff. */
void step_note_on_setup_movi_r2_0x00ff_b(void)
{
    movi16(mcu.r[2], 0x00ff);
    mcu.pc = 0x29a2;
}

/* 0x29a2 MOVG2 @r1+0xcee8 r3. */
void step_note_on_setup_movg2_r1_0xcee8_r3(void)
{
    load8(mcu.r[3], ea_r(1, 0xcee8));
    mcu.pc = 0x29a6;
}

/* 0x29a6 SUB @r3+0x6883 r2. */
void step_note_on_setup_sub_r3_0x6883_r2_b(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x6883));
    mcu.pc = 0x29aa;
}

/* 0x29aa BHI 2 -> 0x29ae. */
void step_note_on_setup_bhi_2_0x29ae(void)
{
    bhi(0x29ae, 0x29ac);
}

/* 0x29ac move r2 #0x01. */
void step_note_on_setup_move_r2_0x01_b(void)
{
    move8_imm(mcu.r[2], 0x01);
    mcu.pc = 0x29ae;
}

/* 0x29ae LDC @r0+0x009a r4. */
void step_note_on_setup_ldc_r0_0x009a_r4_b(void)
{
    ldc8_mem(4, ea_r(0, 0x009a));
    mcu.pc = 0x29b2;
}

/* 0x29b2 MOVG2 @r0+0x00a0 r5. */
void step_note_on_setup_movg2_r0_0x00a0_r5_b(void)
{
    load16(mcu.r[5], ea_r(0, 0x00a0));
    mcu.pc = 0x29b6;
}

/* 0x29b6 MOVG2 @r5+0 r3. */
void step_note_on_setup_movg2_r5_0_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x00));
    mcu.pc = 0x29b9;
}

/* 0x29b9 SUB @r3+0x6883 r2. */
void step_note_on_setup_sub_r3_0x6883_r2_c(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x6883));
    mcu.pc = 0x29bd;
}

/* 0x29bd BHI 2 -> 0x29c1. */
void step_note_on_setup_bhi_2_0x29c1(void)
{
    bhi(0x29c1, 0x29bf);
}

/* 0x29bf move r2 #0x01. */
void step_note_on_setup_move_r2_0x01_c(void)
{
    move8_imm(mcu.r[2], 0x01);
    mcu.pc = 0x29c1;
}

/* 0x29c1 LDC @r0+0x0098 r4. */
void step_note_on_setup_ldc_r0_0x0098_r4(void)
{
    ldc8_mem(4, ea_r(0, 0x0098));
    mcu.pc = 0x29c5;
}

/* 0x29c5 MOVG2 @r0+0x009c r5. */
void step_note_on_setup_movg2_r0_0x009c_r5(void)
{
    load16(mcu.r[5], ea_r(0, 0x009c));
    mcu.pc = 0x29c9;
}

/* 0x29c9 MOVG2 @r5+12 r3. */
void step_note_on_setup_movg2_r5_12_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x0c));
    mcu.pc = 0x29cc;
}

/* 0x29cc SUB @r3+0x6883 r2. */
void step_note_on_setup_sub_r3_0x6883_r2_d(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x6883));
    mcu.pc = 0x29d0;
}

/* 0x29d0 BHI 2 -> 0x29d4. */
void step_note_on_setup_bhi_2_0x29d4(void)
{
    bhi(0x29d4, 0x29d2);
}

/* 0x29d2 move r2 #0x01. */
void step_note_on_setup_move_r2_0x01_d(void)
{
    move8_imm(mcu.r[2], 0x01);
    mcu.pc = 0x29d4;
}

/* 0x29d4 LDC @r0+0x0099 r4. */
void step_note_on_setup_ldc_r0_0x0099_r4_c(void)
{
    ldc8_mem(4, ea_r(0, 0x0099));
    mcu.pc = 0x29d8;
}

/* 0x29d8 MOVG2 @r0+0x009e r5. */
void step_note_on_setup_movg2_r0_0x009e_r5_c(void)
{
    load16(mcu.r[5], ea_r(0, 0x009e));
    mcu.pc = 0x29dc;
}

/* 0x29dc MOVG2 r2 r6. */
void step_note_on_setup_movg2_r2_r6(void)
{
    mov16_reg(mcu.r[6], mcu.r[2]);
    mcu.pc = 0x29de;
}

/* 0x29de MOVG2 @r5+74 r3. */
void step_note_on_setup_movg2_r5_74_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x4a));
    mcu.pc = 0x29e1;
}

/* 0x29e1 SUB @r3+0x6883 r2. */
void step_note_on_setup_sub_r3_0x6883_r2_e(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x6883));
    mcu.pc = 0x29e5;
}

/* 0x29e5 BCC 2 -> 0x29e9. */
void step_note_on_setup_bcc_2_0x29e9(void)
{
    bcc(0x29e9, 0x29e7);
}

/* 0x29e7 CLR r2. */
void step_note_on_setup_clr_r2_b(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x29e9;
}

/* 0x29e9 MOVG2 @r2+0x6903 r3. */
void step_note_on_setup_movg2_r2_0x6903_r3(void)
{
    load8(mcu.r[3], ea_r(2, 0x6903));
    mcu.pc = 0x29ed;
}

/* 0x29ed MOVG3 r3 -> @r0+97. */
void step_note_on_setup_movg3_r3_r0_97(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x61), mcu.r[3]);
    mcu.pc = 0x29f0;
}

/* 0x29f0 MOVG2 r6 r2. */
void step_note_on_setup_movg2_r6_r2(void)
{
    mov16_reg(mcu.r[2], mcu.r[6]);
    mcu.pc = 0x29f2;
}

/* 0x29f2 MOVG2 @r5+75 r3. */
void step_note_on_setup_movg2_r5_75_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x4b));
    mcu.pc = 0x29f5;
}

/* 0x29f5 SUB @r3+0x6883 r2. */
void step_note_on_setup_sub_r3_0x6883_r2_f(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x6883));
    mcu.pc = 0x29f9;
}

/* 0x29f9 BCC 2 -> 0x29fd. */
void step_note_on_setup_bcc_2_0x29fd(void)
{
    bcc(0x29fd, 0x29fb);
}

/* 0x29fb CLR r2. */
void step_note_on_setup_clr_r2_c(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x29fd;
}

/* 0x29fd MOVG2 @r2+0x6903 r3. */
void step_note_on_setup_movg2_r2_0x6903_r3_b(void)
{
    load8(mcu.r[3], ea_r(2, 0x6903));
    mcu.pc = 0x2a01;
}

/* 0x2a01 MOVG3 r3 -> @r0+98. */
void step_note_on_setup_movg3_r3_r0_98(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x62), mcu.r[3]);
    mcu.pc = 0x2a04;
}

/* 0x2a04 MOVG2 r6 r2. */
void step_note_on_setup_movg2_r6_r2_b(void)
{
    mov16_reg(mcu.r[2], mcu.r[6]);
    mcu.pc = 0x2a06;
}

/* 0x2a06 MOVG2 @r5+76 r3. */
void step_note_on_setup_movg2_r5_76_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x4c));
    mcu.pc = 0x2a09;
}

/* 0x2a09 SUB @r3+0x6883 r2. */
void step_note_on_setup_sub_r3_0x6883_r2_g(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x6883));
    mcu.pc = 0x2a0d;
}

/* 0x2a0d BCC 2 -> 0x2a11. */
void step_note_on_setup_bcc_2_0x2a11(void)
{
    bcc(0x2a11, 0x2a0f);
}

/* 0x2a0f CLR r2. */
void step_note_on_setup_clr_r2_d(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x2a11;
}

/* 0x2a11 MOVG2 @r2+0x6903 r3. */
void step_note_on_setup_movg2_r2_0x6903_r3_c(void)
{
    load8(mcu.r[3], ea_r(2, 0x6903));
    mcu.pc = 0x2a15;
}

/* 0x2a15 MOVG3 r3 -> @r0+99. */
void step_note_on_setup_movg3_r3_r0_99(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x63), mcu.r[3]);
    mcu.pc = 0x2a18;
}

/* 0x2a18 MOVG2 r6 r2. */
void step_note_on_setup_movg2_r6_r2_c(void)
{
    mov16_reg(mcu.r[2], mcu.r[6]);
    mcu.pc = 0x2a1a;
}

/* 0x2a1a MOVG2 @r5+77 r3. */
void step_note_on_setup_movg2_r5_77_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x4d));
    mcu.pc = 0x2a1d;
}

/* 0x2a1d SUB @r3+0x6883 r2. */
void step_note_on_setup_sub_r3_0x6883_r2_h(void)
{
    sub8_mem(mcu.r[2], ea_r(3, 0x6883));
    mcu.pc = 0x2a21;
}

/* 0x2a21 BCC 2 -> 0x2a25. */
void step_note_on_setup_bcc_2_0x2a25(void)
{
    bcc(0x2a25, 0x2a23);
}

/* 0x2a23 CLR r2. */
void step_note_on_setup_clr_r2_e(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x2a25;
}

/* 0x2a25 MOVG2 @r2+0x6903 r3. */
void step_note_on_setup_movg2_r2_0x6903_r3_d(void)
{
    load8(mcu.r[3], ea_r(2, 0x6903));
    mcu.pc = 0x2a29;
}

/* 0x2a29 MOVG3 r3 -> @r0+100. */
void step_note_on_setup_movg3_r3_r0_100(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x64), mcu.r[3]);
    mcu.pc = 0x2a2c;
}

/* 0x2a2c CLR @r0+96. */
void step_note_on_setup_clr_r0_96(void)
{
    clr8_mem(ea_r(0, (uint16_t)(int8_t)0x60));
    mcu.pc = 0x2a2f;
}

/* 0x2a2f CLR r4. */
void step_note_on_setup_clr_r4_b(void)
{
    clr_reg(mcu.r[4]);
    mcu.pc = 0x2a31;
}

/* 0x2a31 MOVG2 @r0+-2 r1. */
void step_note_on_setup_movg2_r0_m2_r1_e(void)
{
    load16(mcu.r[1], ea_r(0, (uint16_t)(int8_t)0xfe));
    mcu.pc = 0x2a34;
}

/* 0x2a34 MOVG2 @r1+0xd134 r4. */
void step_note_on_setup_movg2_r1_0xd134_r4_b(void)
{
    load8(mcu.r[4], ea_r(1, 0xd134));
    mcu.pc = 0x2a38;
}

/* 0x2a38 MOVG2 @r5+85 r2. */
void step_note_on_setup_movg2_r5_85_r2(void)
{
    load8(mcu.r[2], ea_r(5, (uint16_t)(int8_t)0x55));
    mcu.pc = 0x2a3b;
}

/* 0x2a3b ADD r2 r2. */
void step_note_on_setup_add_r2_r2_b(void)
{
    add16_reg(mcu.r[2], mcu.r[2]);
    mcu.pc = 0x2a3d;
}

/* 0x2a3d LDC #0x03 r5. */
void step_note_on_setup_ldc_0x03_r5_b(void)
{
    ldc_imm8(5, 0x03);
    mcu.pc = 0x2a40;
}

/* 0x2a40 MOVG2 @r2+0xdd9c r2. */
void step_note_on_setup_movg2_r2_0xdd9c_r2(void)
{
    load16(mcu.r[2], ea_r(2, 0xdd9c));
    mcu.pc = 0x2a44;
}

/* 0x2a44 ADD r4 r2. */
void step_note_on_setup_add_r4_r2(void)
{
    add16_reg(mcu.r[2], mcu.r[4]);
    mcu.pc = 0x2a46;
}

/* 0x2a46 MOVG2 @r2 r4. */
void step_note_on_setup_movg2_r2_r4(void)
{
    load8(mcu.r[4], ea_r(2, 0));
    mcu.pc = 0x2a48;
}

/* 0x2a48 LDC #0x00 r5. */
void step_note_on_setup_ldc_0x00_r5_b(void)
{
    ldc_imm8(5, 0x00);
    mcu.pc = 0x2a4b;
}

/* 0x2a4b CLR r3. */
void step_note_on_setup_clr_r3_g(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2a4d;
}

/* 0x2a4d MOVG2 @r5+87 r3. */
void step_note_on_setup_movg2_r5_87_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x57));
    mcu.pc = 0x2a50;
}

/* 0x2a50 SUB #0x40 r3. */
void step_note_on_setup_sub_0x40_r3_b(void)
{
    sub_imm8_reg(mcu.r[3], 0x40);
    mcu.pc = 0x2a53;
}

/* 0x2a53 BEQ 58 -> 0x2a8f. */
void step_note_on_setup_beq_58_0x2a8f(void)
{
    beq(0x2a8f, 0x2a55);
}

/* 0x2a55 BCC 8 -> 0x2a5f. */
void step_note_on_setup_bcc_8_0x2a5f(void)
{
    bcc(0x2a5f, 0x2a57);
}

/* 0x2a57 NEG r3. */
void step_note_on_setup_neg_r3_b(void)
{
    neg8_reg(mcu.r[3]);
    mcu.pc = 0x2a59;
}

/* 0x2a59 NEG r4. */
void step_note_on_setup_neg_r4_c(void)
{
    neg8_reg(mcu.r[4]);
    mcu.pc = 0x2a5b;
}

/* 0x2a5b BNE 2 -> 0x2a5f. */
void step_note_on_setup_bne_2_0x2a5f(void)
{
    bne(0x2a5f, 0x2a5d);
}

/* 0x2a5d move r4 #0xff. */
void step_note_on_setup_move_r4_0xff(void)
{
    move8_imm(mcu.r[4], 0xff);
    mcu.pc = 0x2a5f;
}

/* 0x2a5f SUB #0x80 r4. */
void step_note_on_setup_sub_0x80_r4_c(void)
{
    sub_imm8_reg(mcu.r[4], 0x80);
    mcu.pc = 0x2a62;
}

/* 0x2a62 BEQ 43 -> 0x2a8f. */
void step_note_on_setup_beq_43_0x2a8f(void)
{
    beq(0x2a8f, 0x2a64);
}

/* 0x2a64 BCC 17 -> 0x2a77. */
void step_note_on_setup_bcc_17_0x2a77(void)
{
    bcc(0x2a77, 0x2a66);
}

/* 0x2a66 NEG r4. */
void step_note_on_setup_neg_r4_d(void)
{
    neg8_reg(mcu.r[4]);
    mcu.pc = 0x2a68;
}

/* 0x2a68 MULXU @r3+0x650e r4. */
void step_note_on_setup_mulxu_r3_0x650e_r4_c(void)
{
    mulxu8_mem(mcu.r[4], ea_r(3, 0x650e));
    mcu.pc = 0x2a6c;
}

/* 0x2a6c ADD r4 r4. */
void step_note_on_setup_add_r4_r4_c(void)
{
    add16_reg(mcu.r[4], mcu.r[4]);
    mcu.pc = 0x2a6e;
}

/* 0x2a6e SWAP r4. */
void step_note_on_setup_swap_r4_d(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x2a70;
}

/* 0x2a70 movi r3 #0x0080. */
void step_note_on_setup_movi_r3_0x0080(void)
{
    movi16(mcu.r[3], 0x0080);
    mcu.pc = 0x2a73;
}

/* 0x2a73 SUB r4 r3. */
void step_note_on_setup_sub_r4_r3(void)
{
    sub8_reg(mcu.r[3], mcu.r[4]);
    mcu.pc = 0x2a75;
}

/* 0x2a75 BRA 13 -> 0x2a84. */
void step_note_on_setup_bra_13_0x2a84(void)
{
    mcu.pc = 0x2a84;
}

/* 0x2a77 MULXU @r3+0x650e r4. */
void step_note_on_setup_mulxu_r3_0x650e_r4_d(void)
{
    mulxu8_mem(mcu.r[4], ea_r(3, 0x650e));
    mcu.pc = 0x2a7b;
}

/* 0x2a7b ADD r4 r4. */
void step_note_on_setup_add_r4_r4_d(void)
{
    add16_reg(mcu.r[4], mcu.r[4]);
    mcu.pc = 0x2a7d;
}

/* 0x2a7d SWAP r4. */
void step_note_on_setup_swap_r4_e(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x2a7f;
}

/* 0x2a7f movi r3 #0x0080. */
void step_note_on_setup_movi_r3_0x0080_b(void)
{
    movi16(mcu.r[3], 0x0080);
    mcu.pc = 0x2a82;
}

/* 0x2a82 ADD r4 r3. */
void step_note_on_setup_add_r4_r3_b(void)
{
    add8_reg(mcu.r[3], mcu.r[4]);
    mcu.pc = 0x2a84;
}

/* 0x2a84 ADD r3 r3. */
void step_note_on_setup_add_r3_r3_h(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2a86;
}

/* 0x2a86 MOVG2 @r3+0x653a r3. */
void step_note_on_setup_movg2_r3_0x653a_r3(void)
{
    load16(mcu.r[3], ea_r(3, 0x653a));
    mcu.pc = 0x2a8a;
}

/* 0x2a8a MOVG3 r3 -> @r0+-34. */
void step_note_on_setup_movg3_r3_r0_m34(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xde), mcu.r[3]);
    mcu.pc = 0x2a8d;
}

/* 0x2a8d BRA 5 -> 0x2a94. */
void step_note_on_setup_bra_5_0x2a94(void)
{
    mcu.pc = 0x2a94;
}

/* 0x2a8f MOVG #0x0100 -> @r0+-34. */
void step_note_on_setup_movg_0x0100_r0_m34(void)
{
    mov_imm16_mem(ea_r(0, (uint16_t)(int8_t)0xde), 0x0100);
    mcu.pc = 0x2a94;
}

/* 0x2a94 CLR r4. */
void step_note_on_setup_clr_r4_c(void)
{
    clr_reg(mcu.r[4]);
    mcu.pc = 0x2a96;
}

/* 0x2a96 MOVG2 @r1+0xd134 r4. */
void step_note_on_setup_movg2_r1_0xd134_r4_c(void)
{
    load8(mcu.r[4], ea_r(1, 0xd134));
    mcu.pc = 0x2a9a;
}

/* 0x2a9a CLR r2. */
void step_note_on_setup_clr_r2_f(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x2a9c;
}

/* 0x2a9c MOVG2 @r5+86 r2. */
void step_note_on_setup_movg2_r5_86_r2(void)
{
    load8(mcu.r[2], ea_r(5, (uint16_t)(int8_t)0x56));
    mcu.pc = 0x2a9f;
}

/* 0x2a9f ADD r2 r2. */
void step_note_on_setup_add_r2_r2_c(void)
{
    add16_reg(mcu.r[2], mcu.r[2]);
    mcu.pc = 0x2aa1;
}

/* 0x2aa1 LDC #0x03 r5. */
void step_note_on_setup_ldc_0x03_r5_c(void)
{
    ldc_imm8(5, 0x03);
    mcu.pc = 0x2aa4;
}

/* 0x2aa4 MOVG2 @r2+0xddbc r2. */
void step_note_on_setup_movg2_r2_0xddbc_r2(void)
{
    load16(mcu.r[2], ea_r(2, 0xddbc));
    mcu.pc = 0x2aa8;
}

/* 0x2aa8 ADD r4 r2. */
void step_note_on_setup_add_r4_r2_b(void)
{
    add16_reg(mcu.r[2], mcu.r[4]);
    mcu.pc = 0x2aaa;
}

/* 0x2aaa MOVG2 @r2 r4. */
void step_note_on_setup_movg2_r2_r4_b(void)
{
    load8(mcu.r[4], ea_r(2, 0));
    mcu.pc = 0x2aac;
}

/* 0x2aac LDC #0x00 r5. */
void step_note_on_setup_ldc_0x00_r5_c(void)
{
    ldc_imm8(5, 0x00);
    mcu.pc = 0x2aaf;
}

/* 0x2aaf CLR r3. */
void step_note_on_setup_clr_r3_h(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2ab1;
}

/* 0x2ab1 MOVG2 @r5+88 r3. */
void step_note_on_setup_movg2_r5_88_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x58));
    mcu.pc = 0x2ab4;
}

/* 0x2ab4 SUB #0x40 r3. */
void step_note_on_setup_sub_0x40_r3_c(void)
{
    sub_imm8_reg(mcu.r[3], 0x40);
    mcu.pc = 0x2ab7;
}

/* 0x2ab7 BEQ 58 -> 0x2af3. */
void step_note_on_setup_beq_58_0x2af3(void)
{
    beq(0x2af3, 0x2ab9);
}

/* 0x2ab9 BCC 8 -> 0x2ac3. */
void step_note_on_setup_bcc_8_0x2ac3(void)
{
    bcc(0x2ac3, 0x2abb);
}

/* 0x2abb NEG r3. */
void step_note_on_setup_neg_r3_c(void)
{
    neg8_reg(mcu.r[3]);
    mcu.pc = 0x2abd;
}

/* 0x2abd NEG r4. */
void step_note_on_setup_neg_r4_e(void)
{
    neg8_reg(mcu.r[4]);
    mcu.pc = 0x2abf;
}

/* 0x2abf BNE 2 -> 0x2ac3. */
void step_note_on_setup_bne_2_0x2ac3(void)
{
    bne(0x2ac3, 0x2ac1);
}

/* 0x2ac1 move r4 #0xff. */
void step_note_on_setup_move_r4_0xff_b(void)
{
    move8_imm(mcu.r[4], 0xff);
    mcu.pc = 0x2ac3;
}

/* 0x2ac3 SUB #0x80 r4. */
void step_note_on_setup_sub_0x80_r4_d(void)
{
    sub_imm8_reg(mcu.r[4], 0x80);
    mcu.pc = 0x2ac6;
}

/* 0x2ac6 BEQ 43 -> 0x2af3. */
void step_note_on_setup_beq_43_0x2af3(void)
{
    beq(0x2af3, 0x2ac8);
}

/* 0x2ac8 BCC 17 -> 0x2adb. */
void step_note_on_setup_bcc_17_0x2adb(void)
{
    bcc(0x2adb, 0x2aca);
}

/* 0x2aca NEG r4. */
void step_note_on_setup_neg_r4_f(void)
{
    neg8_reg(mcu.r[4]);
    mcu.pc = 0x2acc;
}

/* 0x2acc MULXU @r3+0x650e r4. */
void step_note_on_setup_mulxu_r3_0x650e_r4_e(void)
{
    mulxu8_mem(mcu.r[4], ea_r(3, 0x650e));
    mcu.pc = 0x2ad0;
}

/* 0x2ad0 ADD r4 r4. */
void step_note_on_setup_add_r4_r4_e(void)
{
    add16_reg(mcu.r[4], mcu.r[4]);
    mcu.pc = 0x2ad2;
}

/* 0x2ad2 SWAP r4. */
void step_note_on_setup_swap_r4_f(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x2ad4;
}

/* 0x2ad4 movi r3 #0x0080. */
void step_note_on_setup_movi_r3_0x0080_c(void)
{
    movi16(mcu.r[3], 0x0080);
    mcu.pc = 0x2ad7;
}

/* 0x2ad7 SUB r4 r3. */
void step_note_on_setup_sub_r4_r3_b(void)
{
    sub8_reg(mcu.r[3], mcu.r[4]);
    mcu.pc = 0x2ad9;
}

/* 0x2ad9 BRA 13 -> 0x2ae8. */
void step_note_on_setup_bra_13_0x2ae8(void)
{
    mcu.pc = 0x2ae8;
}

/* 0x2adb MULXU @r3+0x650e r4. */
void step_note_on_setup_mulxu_r3_0x650e_r4_f(void)
{
    mulxu8_mem(mcu.r[4], ea_r(3, 0x650e));
    mcu.pc = 0x2adf;
}

/* 0x2adf ADD r4 r4. */
void step_note_on_setup_add_r4_r4_f(void)
{
    add16_reg(mcu.r[4], mcu.r[4]);
    mcu.pc = 0x2ae1;
}

/* 0x2ae1 SWAP r4. */
void step_note_on_setup_swap_r4_g(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x2ae3;
}

/* 0x2ae3 movi r3 #0x0080. */
void step_note_on_setup_movi_r3_0x0080_d(void)
{
    movi16(mcu.r[3], 0x0080);
    mcu.pc = 0x2ae6;
}

/* 0x2ae6 ADD r4 r3. */
void step_note_on_setup_add_r4_r3_c(void)
{
    add8_reg(mcu.r[3], mcu.r[4]);
    mcu.pc = 0x2ae8;
}

/* 0x2ae8 ADD r3 r3. */
void step_note_on_setup_add_r3_r3_i(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2aea;
}

/* 0x2aea MOVG2 @r3+0x653a r3. */
void step_note_on_setup_movg2_r3_0x653a_r3_b(void)
{
    load16(mcu.r[3], ea_r(3, 0x653a));
    mcu.pc = 0x2aee;
}

/* 0x2aee MOVG3 r3 -> @r0+-32. */
void step_note_on_setup_movg3_r3_r0_m32(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xe0), mcu.r[3]);
    mcu.pc = 0x2af1;
}

/* 0x2af1 BRA 5 -> 0x2af8. */
void step_note_on_setup_bra_5_0x2af8(void)
{
    mcu.pc = 0x2af8;
}

/* 0x2af3 MOVG #0x0100 -> @r0+-32. */
void step_note_on_setup_movg_0x0100_r0_m32(void)
{
    mov_imm16_mem(ea_r(0, (uint16_t)(int8_t)0xe0), 0x0100);
    mcu.pc = 0x2af8;
}

/* 0x2af8 movi r2 #0x007f. */
void step_note_on_setup_movi_r2_0x007f(void)
{
    movi16(mcu.r[2], 0x007f);
    mcu.pc = 0x2afb;
}

/* 0x2afb SUB @r1+0xcee8 r2. */
void step_note_on_setup_sub_r1_0xcee8_r2(void)
{
    sub8_mem(mcu.r[2], ea_r(1, 0xcee8));
    mcu.pc = 0x2aff;
}

/* 0x2aff CLR r3. */
void step_note_on_setup_clr_r3_i(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2b01;
}

/* 0x2b01 MOVG2 @r5+89 r3. */
void step_note_on_setup_movg2_r5_89_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x59));
    mcu.pc = 0x2b04;
}

/* 0x2b04 SUB #0x40 r3. */
void step_note_on_setup_sub_0x40_r3_d(void)
{
    sub_imm8_reg(mcu.r[3], 0x40);
    mcu.pc = 0x2b07;
}

/* 0x2b07 BEQ 62 -> 0x2b47. */
void step_note_on_setup_beq_62_0x2b47(void)
{
    beq(0x2b47, 0x2b09);
}

/* 0x2b09 BCC 31 -> 0x2b2a. */
void step_note_on_setup_bcc_31_0x2b2a(void)
{
    bcc(0x2b2a, 0x2b0b);
}

/* 0x2b0b NEG r3. */
void step_note_on_setup_neg_r3_d(void)
{
    neg8_reg(mcu.r[3]);
    mcu.pc = 0x2b0d;
}

/* 0x2b0d MULXU @r3+0x650e r2. */
void step_note_on_setup_mulxu_r3_0x650e_r2(void)
{
    mulxu8_mem(mcu.r[2], ea_r(3, 0x650e));
    mcu.pc = 0x2b11;
}

/* 0x2b11 cmp r2,w #0x1f41. */
void step_note_on_setup_cmp_r2_w_0x1f41(void)
{
    cmp_imm16_reg(mcu.r[2], 0x1f41);
    mcu.pc = 0x2b14;
}

/* 0x2b14 BCS 5 -> 0x2b1b. */
void step_note_on_setup_bcs_5_0x2b1b(void)
{
    bcs(0x2b1b, 0x2b16);
}

/* 0x2b16 movi r3 #0x0004. */
void step_note_on_setup_movi_r3_0x0004(void)
{
    movi16(mcu.r[3], 0x0004);
    mcu.pc = 0x2b19;
}

/* 0x2b19 BRA 47 -> 0x2b4a. */
void step_note_on_setup_bra_47_0x2b4a(void)
{
    mcu.pc = 0x2b4a;
}

/* 0x2b1b movi r3 #0x1fc0. */
void step_note_on_setup_movi_r3_0x1fc0(void)
{
    movi16(mcu.r[3], 0x1fc0);
    mcu.pc = 0x2b1e;
}

/* 0x2b1e SUB r2 r3. */
void step_note_on_setup_sub_r2_r3(void)
{
    sub16_reg(mcu.r[3], mcu.r[2]);
    mcu.pc = 0x2b20;
}

/* 0x2b20 MOVG2 r3 r2. */
void step_note_on_setup_movg2_r3_r2(void)
{
    mov16_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2b22;
}

/* 0x2b22 MULXU #0x0810 r2:r3. */
void step_note_on_setup_mulxu_0x0810_r2_r3(void)
{
    mulxu16_imm(0x0810, mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2b26;
}

/* 0x2b26 MOVG2 r2 r3. */
void step_note_on_setup_movg2_r2_r3(void)
{
    mov16_reg(mcu.r[3], mcu.r[2]);
    mcu.pc = 0x2b28;
}

/* 0x2b28 BRA 32 -> 0x2b4a. */
void step_note_on_setup_bra_32_0x2b4a(void)
{
    mcu.pc = 0x2b4a;
}

/* 0x2b2a movi r4 #0x1fc0. */
void step_note_on_setup_movi_r4_0x1fc0(void)
{
    movi16(mcu.r[4], 0x1fc0);
    mcu.pc = 0x2b2d;
}

/* 0x2b2d MULXU @r3+0x650e r2. */
void step_note_on_setup_mulxu_r3_0x650e_r2_b(void)
{
    mulxu8_mem(mcu.r[2], ea_r(3, 0x650e));
    mcu.pc = 0x2b31;
}

/* 0x2b31 SUB r2 r4. */
void step_note_on_setup_sub_r2_r4(void)
{
    sub16_reg(mcu.r[4], mcu.r[2]);
    mcu.pc = 0x2b33;
}

/* 0x2b33 cmp r4,w #0x0020. */
void step_note_on_setup_cmp_r4_w_0x0020(void)
{
    cmp_imm16_reg(mcu.r[4], 0x0020);
    mcu.pc = 0x2b36;
}

/* 0x2b36 BCS 10 -> 0x2b42. */
void step_note_on_setup_bcs_10_0x2b42(void)
{
    bcs(0x2b42, 0x2b38);
}

/* 0x2b38 movi r2 #0x001f. */
void step_note_on_setup_movi_r2_0x001f(void)
{
    movi16(mcu.r[2], 0x001f);
    mcu.pc = 0x2b3b;
}

/* 0x2b3b movi r3 #0xc000. */
void step_note_on_setup_movi_r3_0xc000(void)
{
    movi16(mcu.r[3], 0xc000);
    mcu.pc = 0x2b3e;
}

/* 0x2b3e DIVXU r4 r2:r3. */
void step_note_on_setup_divxu_r4_r2_r3(void)
{
    divxu16(mcu.r[4], mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2b40;
}

/* 0x2b40 BRA 8 -> 0x2b4a. */
void step_note_on_setup_bra_8_0x2b4a(void)
{
    mcu.pc = 0x2b4a;
}

/* 0x2b42 movi r3 #0xffff. */
void step_note_on_setup_movi_r3_0xffff(void)
{
    movi16(mcu.r[3], 0xffff);
    mcu.pc = 0x2b45;
}

/* 0x2b45 BRA 3 -> 0x2b4a. */
void step_note_on_setup_bra_3_0x2b4a(void)
{
    mcu.pc = 0x2b4a;
}

/* 0x2b47 movi r3 #0x0100. */
void step_note_on_setup_movi_r3_0x0100(void)
{
    movi16(mcu.r[3], 0x0100);
    mcu.pc = 0x2b4a;
}

/* 0x2b4a MOVG3 r3 -> @r0+-30. */
void step_note_on_setup_movg3_r3_r0_m30(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xe2), mcu.r[3]);
    mcu.pc = 0x2b4d;
}

/* 0x2b4d movi r2 #0x007f. */
void step_note_on_setup_movi_r2_0x007f_b(void)
{
    movi16(mcu.r[2], 0x007f);
    mcu.pc = 0x2b50;
}

/* 0x2b50 SUB @r1+0xcee8 r2. */
void step_note_on_setup_sub_r1_0xcee8_r2_b(void)
{
    sub8_mem(mcu.r[2], ea_r(1, 0xcee8));
    mcu.pc = 0x2b54;
}

/* 0x2b54 CLR r3. */
void step_note_on_setup_clr_r3_j(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2b56;
}

/* 0x2b56 MOVG2 @r5+90 r3. */
void step_note_on_setup_movg2_r5_90_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x5a));
    mcu.pc = 0x2b59;
}

/* 0x2b59 SUB #0x40 r3. */
void step_note_on_setup_sub_0x40_r3_e(void)
{
    sub_imm8_reg(mcu.r[3], 0x40);
    mcu.pc = 0x2b5c;
}

/* 0x2b5c BEQ 62 -> 0x2b9c. */
void step_note_on_setup_beq_62_0x2b9c(void)
{
    beq(0x2b9c, 0x2b5e);
}

/* 0x2b5e BCC 31 -> 0x2b7f. */
void step_note_on_setup_bcc_31_0x2b7f(void)
{
    bcc(0x2b7f, 0x2b60);
}

/* 0x2b60 NEG r3. */
void step_note_on_setup_neg_r3_e(void)
{
    neg8_reg(mcu.r[3]);
    mcu.pc = 0x2b62;
}

/* 0x2b62 MULXU @r3+0x650e r2. */
void step_note_on_setup_mulxu_r3_0x650e_r2_c(void)
{
    mulxu8_mem(mcu.r[2], ea_r(3, 0x650e));
    mcu.pc = 0x2b66;
}

/* 0x2b66 cmp r2,w #0x1f41. */
void step_note_on_setup_cmp_r2_w_0x1f41_b(void)
{
    cmp_imm16_reg(mcu.r[2], 0x1f41);
    mcu.pc = 0x2b69;
}

/* 0x2b69 BCS 5 -> 0x2b70. */
void step_note_on_setup_bcs_5_0x2b70(void)
{
    bcs(0x2b70, 0x2b6b);
}

/* 0x2b6b movi r3 #0x0004. */
void step_note_on_setup_movi_r3_0x0004_b(void)
{
    movi16(mcu.r[3], 0x0004);
    mcu.pc = 0x2b6e;
}

/* 0x2b6e BRA 47 -> 0x2b9f. */
void step_note_on_setup_bra_47_0x2b9f(void)
{
    mcu.pc = 0x2b9f;
}

/* 0x2b70 movi r3 #0x1fc0. */
void step_note_on_setup_movi_r3_0x1fc0_b(void)
{
    movi16(mcu.r[3], 0x1fc0);
    mcu.pc = 0x2b73;
}

/* 0x2b73 SUB r2 r3. */
void step_note_on_setup_sub_r2_r3_b(void)
{
    sub16_reg(mcu.r[3], mcu.r[2]);
    mcu.pc = 0x2b75;
}

/* 0x2b75 MOVG2 r3 r2. */
void step_note_on_setup_movg2_r3_r2_b(void)
{
    mov16_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2b77;
}

/* 0x2b77 MULXU #0x0810 r2:r3. */
void step_note_on_setup_mulxu_0x0810_r2_r3_b(void)
{
    mulxu16_imm(0x0810, mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2b7b;
}

/* 0x2b7b MOVG2 r2 r3. */
void step_note_on_setup_movg2_r2_r3_b(void)
{
    mov16_reg(mcu.r[3], mcu.r[2]);
    mcu.pc = 0x2b7d;
}

/* 0x2b7d BRA 32 -> 0x2b9f. */
void step_note_on_setup_bra_32_0x2b9f(void)
{
    mcu.pc = 0x2b9f;
}

/* 0x2b7f movi r4 #0x1fc0. */
void step_note_on_setup_movi_r4_0x1fc0_b(void)
{
    movi16(mcu.r[4], 0x1fc0);
    mcu.pc = 0x2b82;
}

/* 0x2b82 MULXU @r3+0x650e r2. */
void step_note_on_setup_mulxu_r3_0x650e_r2_d(void)
{
    mulxu8_mem(mcu.r[2], ea_r(3, 0x650e));
    mcu.pc = 0x2b86;
}

/* 0x2b86 SUB r2 r4. */
void step_note_on_setup_sub_r2_r4_b(void)
{
    sub16_reg(mcu.r[4], mcu.r[2]);
    mcu.pc = 0x2b88;
}

/* 0x2b88 cmp r4,w #0x0020. */
void step_note_on_setup_cmp_r4_w_0x0020_b(void)
{
    cmp_imm16_reg(mcu.r[4], 0x0020);
    mcu.pc = 0x2b8b;
}

/* 0x2b8b BCS 10 -> 0x2b97. */
void step_note_on_setup_bcs_10_0x2b97(void)
{
    bcs(0x2b97, 0x2b8d);
}

/* 0x2b8d movi r2 #0x001f. */
void step_note_on_setup_movi_r2_0x001f_b(void)
{
    movi16(mcu.r[2], 0x001f);
    mcu.pc = 0x2b90;
}

/* 0x2b90 movi r3 #0xc000. */
void step_note_on_setup_movi_r3_0xc000_b(void)
{
    movi16(mcu.r[3], 0xc000);
    mcu.pc = 0x2b93;
}

/* 0x2b93 DIVXU r4 r2:r3. */
void step_note_on_setup_divxu_r4_r2_r3_b(void)
{
    divxu16(mcu.r[4], mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2b95;
}

/* 0x2b95 BRA 8 -> 0x2b9f. */
void step_note_on_setup_bra_8_0x2b9f(void)
{
    mcu.pc = 0x2b9f;
}

/* 0x2b97 movi r3 #0xffff. */
void step_note_on_setup_movi_r3_0xffff_b(void)
{
    movi16(mcu.r[3], 0xffff);
    mcu.pc = 0x2b9a;
}

/* 0x2b9a BRA 3 -> 0x2b9f. */
void step_note_on_setup_bra_3_0x2b9f(void)
{
    mcu.pc = 0x2b9f;
}

/* 0x2b9c movi r3 #0x0100. */
void step_note_on_setup_movi_r3_0x0100_b(void)
{
    movi16(mcu.r[3], 0x0100);
    mcu.pc = 0x2b9f;
}

/* 0x2b9f MOVG3 r3 -> @r0+-28. */
void step_note_on_setup_movg3_r3_r0_m28(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0xe4), mcu.r[3]);
    mcu.pc = 0x2ba2;
}

/* 0x2ba2 MOVG2 @r5+78 r2. */
void step_note_on_setup_movg2_r5_78_r2(void)
{
    load8(mcu.r[2], ea_r(5, (uint16_t)(int8_t)0x4e));
    mcu.pc = 0x2ba5;
}

/* 0x2ba5 BTSTI r2 #7. */
void step_note_on_setup_btsti_r2_7(void)
{
    btst8_reg(mcu.r[2], 7);
    mcu.pc = 0x2ba7;
}

/* 0x2ba7 BEQ 6 -> 0x2baf. */
void step_note_on_setup_beq_6_0x2baf(void)
{
    beq(0x2baf, 0x2ba9);
}

/* 0x2ba9 MOVG #0x00 -> @r0+-8. */
void step_note_on_setup_movg_0x00_r0_m8(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xf8), 0x00);
    mcu.pc = 0x2bad;
}

/* 0x2bad BRA 4 -> 0x2bb3. */
void step_note_on_setup_bra_4_0x2bb3(void)
{
    mcu.pc = 0x2bb3;
}

/* 0x2baf MOVG #0x04 -> @r0+-8. */
void step_note_on_setup_movg_0x04_r0_m8(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xf8), 0x04);
    mcu.pc = 0x2bb3;
}

/* 0x2bb3 AND #0x007f r2. */
void step_note_on_setup_and_0x007f_r2(void)
{
    and_imm16_reg(mcu.r[2], 0x007f);
    mcu.pc = 0x2bb7;
}

/* 0x2bb7 MOVG3 r2 -> @r0+79. */
void step_note_on_setup_movg3_r2_r0_79(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x4f), mcu.r[2]);
    mcu.pc = 0x2bba;
}

/* 0x2bba MOVG2 @r5+79 r2. */
void step_note_on_setup_movg2_r5_79_r2(void)
{
    load8(mcu.r[2], ea_r(5, (uint16_t)(int8_t)0x4f));
    mcu.pc = 0x2bbd;
}

/* 0x2bbd BTSTI r2 #7. */
void step_note_on_setup_btsti_r2_7_b(void)
{
    btst8_reg(mcu.r[2], 7);
    mcu.pc = 0x2bbf;
}

/* 0x2bbf BEQ 6 -> 0x2bc7. */
void step_note_on_setup_beq_6_0x2bc7(void)
{
    beq(0x2bc7, 0x2bc1);
}

/* 0x2bc1 MOVG #0x00 -> @r0+-7. */
void step_note_on_setup_movg_0x00_r0_m7(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xf9), 0x00);
    mcu.pc = 0x2bc5;
}

/* 0x2bc5 BRA 4 -> 0x2bcb. */
void step_note_on_setup_bra_4_0x2bcb(void)
{
    mcu.pc = 0x2bcb;
}

/* 0x2bc7 MOVG #0x04 -> @r0+-7. */
void step_note_on_setup_movg_0x04_r0_m7(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xf9), 0x04);
    mcu.pc = 0x2bcb;
}

/* 0x2bcb AND #0x007f r2. */
void step_note_on_setup_and_0x007f_r2_b(void)
{
    and_imm16_reg(mcu.r[2], 0x007f);
    mcu.pc = 0x2bcf;
}

/* 0x2bcf MOVG3 r2 -> @r0+80. */
void step_note_on_setup_movg3_r2_r0_80(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x50), mcu.r[2]);
    mcu.pc = 0x2bd2;
}

/* 0x2bd2 MOVG2 @r5+80 r2. */
void step_note_on_setup_movg2_r5_80_r2(void)
{
    load8(mcu.r[2], ea_r(5, (uint16_t)(int8_t)0x50));
    mcu.pc = 0x2bd5;
}

/* 0x2bd5 BTSTI r2 #7. */
void step_note_on_setup_btsti_r2_7_c(void)
{
    btst8_reg(mcu.r[2], 7);
    mcu.pc = 0x2bd7;
}

/* 0x2bd7 BEQ 6 -> 0x2bdf. */
void step_note_on_setup_beq_6_0x2bdf(void)
{
    beq(0x2bdf, 0x2bd9);
}

/* 0x2bd9 MOVG #0x00 -> @r0+-6. */
void step_note_on_setup_movg_0x00_r0_m6(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xfa), 0x00);
    mcu.pc = 0x2bdd;
}

/* 0x2bdd BRA 4 -> 0x2be3. */
void step_note_on_setup_bra_4_0x2be3(void)
{
    mcu.pc = 0x2be3;
}

/* 0x2bdf MOVG #0x04 -> @r0+-6. */
void step_note_on_setup_movg_0x04_r0_m6(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xfa), 0x04);
    mcu.pc = 0x2be3;
}

/* 0x2be3 AND #0x007f r2. */
void step_note_on_setup_and_0x007f_r2_c(void)
{
    and_imm16_reg(mcu.r[2], 0x007f);
    mcu.pc = 0x2be7;
}

/* 0x2be7 MOVG3 r2 -> @r0+81. */
void step_note_on_setup_movg3_r2_r0_81(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x51), mcu.r[2]);
    mcu.pc = 0x2bea;
}

/* 0x2bea MOVG2 @r5+81 r2. */
void step_note_on_setup_movg2_r5_81_r2(void)
{
    load8(mcu.r[2], ea_r(5, (uint16_t)(int8_t)0x51));
    mcu.pc = 0x2bed;
}

/* 0x2bed BTSTI r2 #7. */
void step_note_on_setup_btsti_r2_7_d(void)
{
    btst8_reg(mcu.r[2], 7);
    mcu.pc = 0x2bef;
}

/* 0x2bef BEQ 6 -> 0x2bf7. */
void step_note_on_setup_beq_6_0x2bf7(void)
{
    beq(0x2bf7, 0x2bf1);
}

/* 0x2bf1 MOVG #0x00 -> @r0+-5. */
void step_note_on_setup_movg_0x00_r0_m5(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xfb), 0x00);
    mcu.pc = 0x2bf5;
}

/* 0x2bf5 BRA 4 -> 0x2bfb. */
void step_note_on_setup_bra_4_0x2bfb(void)
{
    mcu.pc = 0x2bfb;
}

/* 0x2bf7 MOVG #0x04 -> @r0+-5. */
void step_note_on_setup_movg_0x04_r0_m5(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xfb), 0x04);
    mcu.pc = 0x2bfb;
}

/* 0x2bfb AND #0x007f r2. */
void step_note_on_setup_and_0x007f_r2_d(void)
{
    and_imm16_reg(mcu.r[2], 0x007f);
    mcu.pc = 0x2bff;
}

/* 0x2bff MOVG3 r2 -> @r0+82. */
void step_note_on_setup_movg3_r2_r0_82(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x52), mcu.r[2]);
    mcu.pc = 0x2c02;
}

/* 0x2c02 MOVG2 @r5+82 r2. */
void step_note_on_setup_movg2_r5_82_r2(void)
{
    load8(mcu.r[2], ea_r(5, (uint16_t)(int8_t)0x52));
    mcu.pc = 0x2c05;
}

/* 0x2c05 BTSTI r2 #7. */
void step_note_on_setup_btsti_r2_7_e(void)
{
    btst8_reg(mcu.r[2], 7);
    mcu.pc = 0x2c07;
}

/* 0x2c07 BEQ 6 -> 0x2c0f. */
void step_note_on_setup_beq_6_0x2c0f(void)
{
    beq(0x2c0f, 0x2c09);
}

/* 0x2c09 MOVG #0x00 -> @r0+-4. */
void step_note_on_setup_movg_0x00_r0_m4(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xfc), 0x00);
    mcu.pc = 0x2c0d;
}

/* 0x2c0d BRA 4 -> 0x2c13. */
void step_note_on_setup_bra_4_0x2c13(void)
{
    mcu.pc = 0x2c13;
}

/* 0x2c0f MOVG #0x04 -> @r0+-4. */
void step_note_on_setup_movg_0x04_r0_m4(void)
{
    mov_imm8_mem(ea_r(0, (uint16_t)(int8_t)0xfc), 0x04);
    mcu.pc = 0x2c13;
}

/* 0x2c13 AND #0x007f r2. */
void step_note_on_setup_and_0x007f_r2_e(void)
{
    and_imm16_reg(mcu.r[2], 0x007f);
    mcu.pc = 0x2c17;
}

/* 0x2c17 MOVG3 r2 -> @r0+83. */
void step_note_on_setup_movg3_r2_r0_83(void)
{
    store8(ea_r(0, (uint16_t)(int8_t)0x53), mcu.r[2]);
    mcu.pc = 0x2c1a;
}

/* 0x2c1a LDC @r0+0x0098 r4. */
void step_note_on_setup_ldc_r0_0x0098_r4_b(void)
{
    ldc8_mem(4, ea_r(0, 0x0098));
    mcu.pc = 0x2c1e;
}

/* 0x2c1e MOVG2 @r0+0x009c r5. */
void step_note_on_setup_movg2_r0_0x009c_r5_b(void)
{
    load16(mcu.r[5], ea_r(0, 0x009c));
    mcu.pc = 0x2c22;
}

/* 0x2c22 CLR r3. */
void step_note_on_setup_clr_r3_k(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2c24;
}

/* 0x2c24 MOVG2 @r5+31 r3. */
void step_note_on_setup_movg2_r5_31_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x1f));
    mcu.pc = 0x2c27;
}

/* 0x2c27 LDC @r0+0x0099 r4. */
void step_note_on_setup_ldc_r0_0x0099_r4_d(void)
{
    ldc8_mem(4, ea_r(0, 0x0099));
    mcu.pc = 0x2c2b;
}

/* 0x2c2b MOVG2 @r0+0x009e r5. */
void step_note_on_setup_movg2_r0_0x009e_r5_d(void)
{
    load16(mcu.r[5], ea_r(0, 0x009e));
    mcu.pc = 0x2c2f;
}

/* 0x2c2f AND #0x03 r3. */
void step_note_on_setup_and_0x03_r3(void)
{
    and_imm8_reg(mcu.r[3], 0x03);
    mcu.pc = 0x2c32;
}

/* 0x2c32 BEQ 27 -> 0x2c4f. */
void step_note_on_setup_beq_27_0x2c4f(void)
{
    beq(0x2c4f, 0x2c34);
}

/* 0x2c34 CLR r2. */
void step_note_on_setup_clr_r2_g(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x2c36;
}

/* 0x2c36 MOVG2 @r1+0xd134 r2. */
void step_note_on_setup_movg2_r1_0xd134_r2(void)
{
    load8(mcu.r[2], ea_r(1, 0xd134));
    mcu.pc = 0x2c3a;
}

/* 0x2c3a LDC #0x03 r5. */
void step_note_on_setup_ldc_0x03_r5_d(void)
{
    ldc_imm8(5, 0x03);
    mcu.pc = 0x2c3d;
}

/* 0x2c3d SHLL r3. */
void step_note_on_setup_shll_r3(void)
{
    shll16(mcu.r[3]);
    mcu.pc = 0x2c3f;
}

/* 0x2c3f ADDS @r3+0xf80c r2. */
void step_note_on_setup_adds_r3_0xf80c_r2(void)
{
    adds16_mem(mcu.r[2], ea_r(3, 0xf80c));
    mcu.pc = 0x2c43;
}

/* 0x2c43 MOVG2 @r2 r2. */
void step_note_on_setup_movg2_r2_r2(void)
{
    load8(mcu.r[2], ea_r(2, 0));
    mcu.pc = 0x2c45;
}

/* 0x2c45 LDC #0x00 r5. */
void step_note_on_setup_ldc_0x00_r5_d(void)
{
    ldc_imm8(5, 0x00);
    mcu.pc = 0x2c48;
}

/* 0x2c48 EXTU r2. */
void step_note_on_setup_extu_r2(void)
{
    extu8(mcu.r[2]);
    mcu.pc = 0x2c4a;
}

/* 0x2c4a MOVG3 r2 -> @r0+56. */
void step_note_on_setup_movg3_r2_r0_56(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x38), mcu.r[2]);
    mcu.pc = 0x2c4d;
}

/* 0x2c4d BRA 8 -> 0x2c57. */
void step_note_on_setup_bra_8_0x2c57(void)
{
    mcu.pc = 0x2c57;
}

/* 0x2c4f CLR r3. */
void step_note_on_setup_clr_r3_l(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2c51;
}

/* 0x2c51 MOVG2 @r5+9 r3. */
void step_note_on_setup_movg2_r5_9_r3(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x09));
    mcu.pc = 0x2c54;
}

/* 0x2c54 MOVG3 r3 -> @r0+56. */
void step_note_on_setup_movg3_r3_r0_56(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x38), mcu.r[3]);
    mcu.pc = 0x2c57;
}

/* 0x2c57 MOVG2 @r5+0 r3. */
void step_note_on_setup_movg2_r5_0_r3_b(void)
{
    load8(mcu.r[3], ea_r(5, (uint16_t)(int8_t)0x00));
    mcu.pc = 0x2c5a;
}

/* 0x2c5a BPL 4 -> 0x2c60. */
void step_note_on_setup_bpl_4_0x2c60(void)
{
    bpl(0x2c60, 0x2c5c);
}

/* 0x2c5c CLR r3. */
void step_note_on_setup_clr_r3_m(void)
{
    clr_reg(mcu.r[3]);
    mcu.pc = 0x2c5e;
}

/* 0x2c5e BRA 35 -> 0x2c83. */
void step_note_on_setup_bra_35_0x2c83(void)
{
    mcu.pc = 0x2c83;
}

/* 0x2c60 ADD r3 r3. */
void step_note_on_setup_add_r3_r3_j(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2c62;
}

/* 0x2c62 MOVG2 @r3+0x6c86 r4. */
void step_note_on_setup_movg2_r3_0x6c86_r4(void)
{
    load16(mcu.r[4], ea_r(3, 0x6c86));
    mcu.pc = 0x2c66;
}

/* 0x2c66 ADD #0x0004 r4. */
void step_note_on_setup_add_0x0004_r4(void)
{
    add_imm16_reg(mcu.r[4], 0x0004);
    mcu.pc = 0x2c6a;
}

/* 0x2c6a SHLR r4. */
void step_note_on_setup_shlr_r4_c(void)
{
    shlr16(mcu.r[4]);
    mcu.pc = 0x2c6c;
}

/* 0x2c6c SHLR r4. */
void step_note_on_setup_shlr_r4_d(void)
{
    shlr16(mcu.r[4]);
    mcu.pc = 0x2c6e;
}

/* 0x2c6e SHLR r4. */
void step_note_on_setup_shlr_r4_e(void)
{
    shlr16(mcu.r[4]);
    mcu.pc = 0x2c70;
}

/* 0x2c70 BNE 10 -> 0x2c7c. */
void step_note_on_setup_bne_10_0x2c7c(void)
{
    bne(0x2c7c, 0x2c72);
}

/* 0x2c72 MOVG #0xffff -> @r0+14. */
void step_note_on_setup_movg_0xffff_r0_14(void)
{
    mov_imm16_mem(ea_r(0, (uint16_t)(int8_t)0x0e), 0xffff);
    mcu.pc = 0x2c77;
}

/* 0x2c77 CLR @r0+16. */
void step_note_on_setup_clr_r0_16(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x10));
    mcu.pc = 0x2c7a;
}

/* 0x2c7a BRA 22 -> 0x2c92. */
void step_note_on_setup_bra_22_0x2c92(void)
{
    mcu.pc = 0x2c92;
}

/* 0x2c7c movi r3 #0xffff. */
void step_note_on_setup_movi_r3_0xffff_c(void)
{
    movi16(mcu.r[3], 0xffff);
    mcu.pc = 0x2c7f;
}

/* 0x2c7f CLR r2. */
void step_note_on_setup_clr_r2_h(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x2c81;
}

/* 0x2c81 DIVXU r4 r2:r3. */
void step_note_on_setup_divxu_r4_r2_r3_c(void)
{
    divxu16(mcu.r[4], mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2c83;
}

/* 0x2c83 CLR @r0+14. */
void step_note_on_setup_clr_r0_14(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x0e));
    mcu.pc = 0x2c86;
}

/* 0x2c86 MOVG3 r3 -> @r0+16. */
void step_note_on_setup_movg3_r3_r0_16(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x10), mcu.r[3]);
    mcu.pc = 0x2c89;
}

/* 0x2c89 CLR @r0+24. */
void step_note_on_setup_clr_r0_24(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x18));
    mcu.pc = 0x2c8c;
}

/* 0x2c8c CLR @r0+28. */
void step_note_on_setup_clr_r0_28(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x1c));
    mcu.pc = 0x2c8f;
}

/* 0x2c8f CLR @r0+36. */
void step_note_on_setup_clr_r0_36(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x24));
    mcu.pc = 0x2c92;
}

/* 0x2c92 CLR @r0+18. */
void step_note_on_setup_clr_r0_18(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x12));
    mcu.pc = 0x2c95;
}

/* 0x2c95 CLR @r0+8. */
void step_note_on_setup_clr_r0_8(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x08));
    mcu.pc = 0x2c98;
}

/* 0x2c98 CLR @r0+24. */
void step_note_on_setup_clr_r0_24_b(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x18));
    mcu.pc = 0x2c9b;
}

/* 0x2c9b CLR @r0+28. */
void step_note_on_setup_clr_r0_28_b(void)
{
    clr16_mem(ea_r(0, (uint16_t)(int8_t)0x1c));
    mcu.pc = 0x2c9e;
}

/* 0x2c9e MOVG2 (dp,0xad2a) r6. */
void step_note_on_setup_movg2_dp_0xad2a_r6_b(void)
{
    load16(mcu.r[6], ea_dp(0xad2a));
    mcu.pc = 0x2ca2;
}

/* 0x2ca2 MOVG3 r6 -> (dp,0xad2c). */
void step_note_on_setup_movg3_r6_dp_0xad2c_b(void)
{
    store16(ea_dp(0xad2c), mcu.r[6]);
    mcu.pc = 0x2ca6;
}

/* 0x2ca6 MOVG #0x0001 -> (dp,0xad2a). */
void step_note_on_setup_movg_0x0001_dp_0xad2a_b(void)
{
    mov_imm16_mem(ea_dp(0xad2a), 0x0001);
    mcu.pc = 0x2cac;
}

/* 0x2cac MOVG2 @r0+-2 r1. */
void step_note_on_setup_movg2_r0_m2_r1_f(void)
{
    load16(mcu.r[1], ea_r(0, (uint16_t)(int8_t)0xfe));
    mcu.pc = 0x2caf;
}

/* 0x2caf bsr16 -> 0x31ca. */
void step_note_on_setup_bsr16_0x31ca(void)
{
    call(0x2cb2, 0x31ca);
}

/* 0x2cb2 MOVG2 @r0+30 r2. */
void step_note_on_setup_movg2_r0_30_r2(void)
{
    load16(mcu.r[2], ea_r(0, (uint16_t)(int8_t)0x1e));
    mcu.pc = 0x2cb5;
}

/* 0x2cb5 cmp r2,b #0xaf. */
void step_note_on_setup_cmp_r2_b_0xaf(void)
{
    cmp_imm8_reg(mcu.r[2], 0xaf);
    mcu.pc = 0x2cb7;
}

/* 0x2cb7 BNE 5 -> 0x2cbe. */
void step_note_on_setup_bne_5_0x2cbe(void)
{
    bne(0x2cbe, 0x2cb9);
}

/* 0x2cb9 move r2 #0xba. */
void step_note_on_setup_move_r2_0xba(void)
{
    move8_imm(mcu.r[2], 0xba);
    mcu.pc = 0x2cbb;
}

/* 0x2cbb MOVG3 r2 -> @r0+30. */
void step_note_on_setup_movg3_r2_r0_30(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x1e), mcu.r[2]);
    mcu.pc = 0x2cbe;
}

/* 0x2cbe MOVG2 (dp,0xad2c) r6. */
void step_note_on_setup_movg2_dp_0xad2c_r6_b(void)
{
    load16(mcu.r[6], ea_dp(0xad2c));
    mcu.pc = 0x2cc2;
}

/* 0x2cc2 MOVG3 r6 -> (dp,0xad2a). */
void step_note_on_setup_movg3_r6_dp_0xad2a_b(void)
{
    store16(ea_dp(0xad2a), mcu.r[6]);
    mcu.pc = 0x2cc6;
}

/* 0x2cc6 MOVG2 @r0+46 r2. */
void step_note_on_setup_movg2_r0_46_r2(void)
{
    load16(mcu.r[2], ea_r(0, (uint16_t)(int8_t)0x2e));
    mcu.pc = 0x2cc9;
}

/* 0x2cc9 MOVG2 @r0+48 r3. */
void step_note_on_setup_movg2_r0_48_r3(void)
{
    load16(mcu.r[3], ea_r(0, (uint16_t)(int8_t)0x30));
    mcu.pc = 0x2ccc;
}

/* 0x2ccc MOVG2 @r2+14 r4. */
void step_note_on_setup_movg2_r2_14_r4(void)
{
    load8(mcu.r[4], ea_r(2, (uint16_t)(int8_t)0x0e));
    mcu.pc = 0x2ccf;
}

/* 0x2ccf MOVG2 @r2+15 r5. */
void step_note_on_setup_movg2_r2_15_r5(void)
{
    load8(mcu.r[5], ea_r(2, (uint16_t)(int8_t)0x0f));
    mcu.pc = 0x2cd2;
}

/* 0x2cd2 TST r3. */
void step_note_on_setup_tst_r3(void)
{
    tst16_reg(mcu.r[3]);
    mcu.pc = 0x2cd4;
}

/* 0x2cd4 BEQ 24 -> 0x2cee. */
void step_note_on_setup_beq_24_0x2cee(void)
{
    beq(0x2cee, 0x2cd6);
}

/* 0x2cd6 MULXU @r3+0x0380 r4. */
void step_note_on_setup_mulxu_r3_0x0380_r4(void)
{
    mulxu8_mem(mcu.r[4], ea_r(3, 0x0380));
    mcu.pc = 0x2cda;
}

/* 0x2cda MULXU @r3+0x0300 r5. */
void step_note_on_setup_mulxu_r3_0x0300_r5(void)
{
    mulxu8_mem(mcu.r[5], ea_r(3, 0x0300));
    mcu.pc = 0x2cde;
}

/* 0x2cde ADD r4 r4. */
void step_note_on_setup_add_r4_r4_g(void)
{
    add16_reg(mcu.r[4], mcu.r[4]);
    mcu.pc = 0x2ce0;
}

/* 0x2ce0 ADD r5 r5. */
void step_note_on_setup_add_r5_r5(void)
{
    add16_reg(mcu.r[5], mcu.r[5]);
    mcu.pc = 0x2ce2;
}

/* 0x2ce2 ADD #0x00ff r4. */
void step_note_on_setup_add_0x00ff_r4(void)
{
    add_imm16_reg(mcu.r[4], 0x00ff);
    mcu.pc = 0x2ce6;
}

/* 0x2ce6 ADD #0x00ff r5. */
void step_note_on_setup_add_0x00ff_r5(void)
{
    add_imm16_reg(mcu.r[5], 0x00ff);
    mcu.pc = 0x2cea;
}

/* 0x2cea SWAP r4. */
void step_note_on_setup_swap_r4_h(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x2cec;
}

/* 0x2cec SWAP r5. */
void step_note_on_setup_swap_r5(void)
{
    swap16(mcu.r[5]);
    mcu.pc = 0x2cee;
}

/* 0x2cee SWAP r5. */
void step_note_on_setup_swap_r5_b(void)
{
    swap16(mcu.r[5]);
    mcu.pc = 0x2cf0;
}

/* 0x2cf0 MOVG2 r4 r5. */
void step_note_on_setup_movg2_r4_r5_b(void)
{
    mov8_reg(mcu.r[5], mcu.r[4]);
    mcu.pc = 0x2cf2;
}

/* 0x2cf2 MOVG3 r5 -> @r0+58. */
void step_note_on_setup_movg3_r5_r0_58(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x3a), mcu.r[5]);
    mcu.pc = 0x2cf5;
}

/* 0x2cf5 MOVG2 @r2+9 r2. */
void step_note_on_setup_movg2_r2_9_r2(void)
{
    load8(mcu.r[2], ea_r(2, (uint16_t)(int8_t)0x09));
    mcu.pc = 0x2cf8;
}

/* 0x2cf8 BEQ 81 -> 0x2d4b. */
void step_note_on_setup_beq_81_0x2d4b(void)
{
    beq(0x2d4b, 0x2cfa);
}

/* 0x2cfa TST r3. */
void step_note_on_setup_tst_r3_b(void)
{
    tst16_reg(mcu.r[3]);
    mcu.pc = 0x2cfc;
}

/* 0x2cfc BEQ 25 -> 0x2d17. */
void step_note_on_setup_beq_25_0x2d17(void)
{
    beq(0x2d17, 0x2cfe);
}

/* 0x2cfe MOVG2 @r3+0x0280 r5. */
void step_note_on_setup_movg2_r3_0x0280_r5(void)
{
    load8(mcu.r[5], ea_r(3, 0x0280));
    mcu.pc = 0x2d02;
}

/* 0x2d02 BEQ 71 -> 0x2d4b. */
void step_note_on_setup_beq_71_0x2d4b(void)
{
    beq(0x2d4b, 0x2d04);
}

/* 0x2d04 SUB #0x40 r5. */
void step_note_on_setup_sub_0x40_r5(void)
{
    sub_imm8_reg(mcu.r[5], 0x40);
    mcu.pc = 0x2d07;
}

/* 0x2d07 BCC 8 -> 0x2d11. */
void step_note_on_setup_bcc_8_0x2d11(void)
{
    bcc(0x2d11, 0x2d09);
}

/* 0x2d09 ADD r5 r2. */
void step_note_on_setup_add_r5_r2(void)
{
    add8_reg(mcu.r[2], mcu.r[5]);
    mcu.pc = 0x2d0b;
}

/* 0x2d0b BPL 10 -> 0x2d17. */
void step_note_on_setup_bpl_10_0x2d17(void)
{
    bpl(0x2d17, 0x2d0d);
}

/* 0x2d0d CLR r2. */
void step_note_on_setup_clr_r2_i(void)
{
    clr8_reg(mcu.r[2]);
    mcu.pc = 0x2d0f;
}

/* 0x2d0f BRA 6 -> 0x2d17. */
void step_note_on_setup_bra_6_0x2d17(void)
{
    mcu.pc = 0x2d17;
}

/* 0x2d11 ADD r5 r2. */
void step_note_on_setup_add_r5_r2_b(void)
{
    add8_reg(mcu.r[2], mcu.r[5]);
    mcu.pc = 0x2d13;
}

/* 0x2d13 BVC 2 -> 0x2d17. */
void step_note_on_setup_bvc_2_0x2d17(void)
{
    bvc(0x2d17, 0x2d15);
}

/* 0x2d15 move r2 #0x7f. */
void step_note_on_setup_move_r2_0x7f(void)
{
    move8_imm(mcu.r[2], 0x7f);
    mcu.pc = 0x2d17;
}

/* 0x2d17 MOVG2 @r0+56 r3. */
void step_note_on_setup_movg2_r0_56_r3(void)
{
    load16(mcu.r[3], ea_r(0, (uint16_t)(int8_t)0x38));
    mcu.pc = 0x2d1a;
}

/* 0x2d1a SUB #0x40 r2. */
void step_note_on_setup_sub_0x40_r2(void)
{
    sub_imm8_reg(mcu.r[2], 0x40);
    mcu.pc = 0x2d1d;
}

/* 0x2d1d BCC 8 -> 0x2d27. */
void step_note_on_setup_bcc_8_0x2d27(void)
{
    bcc(0x2d27, 0x2d1f);
}

/* 0x2d1f ADD r2 r3. */
void step_note_on_setup_add_r2_r3(void)
{
    add8_reg(mcu.r[3], mcu.r[2]);
    mcu.pc = 0x2d21;
}

/* 0x2d21 BPL 10 -> 0x2d2d. */
void step_note_on_setup_bpl_10_0x2d2d(void)
{
    bpl(0x2d2d, 0x2d23);
}

/* 0x2d23 CLR r3. */
void step_note_on_setup_clr_r3_n(void)
{
    clr8_reg(mcu.r[3]);
    mcu.pc = 0x2d25;
}

/* 0x2d25 BRA 6 -> 0x2d2d. */
void step_note_on_setup_bra_6_0x2d2d(void)
{
    mcu.pc = 0x2d2d;
}

/* 0x2d27 ADD r2 r3. */
void step_note_on_setup_add_r2_r3_b(void)
{
    add8_reg(mcu.r[3], mcu.r[2]);
    mcu.pc = 0x2d29;
}

/* 0x2d29 BVC 2 -> 0x2d2d. */
void step_note_on_setup_bvc_2_0x2d2d(void)
{
    bvc(0x2d2d, 0x2d2b);
}

/* 0x2d2b move r3 #0x7f. */
void step_note_on_setup_move_r3_0x7f(void)
{
    move8_imm(mcu.r[3], 0x7f);
    mcu.pc = 0x2d2d;
}

/* 0x2d2d MOVG2 (dp,0x8006) r2. */
void step_note_on_setup_movg2_dp_0x8006_r2(void)
{
    load8(mcu.r[2], ea_dp(0x8006));
    mcu.pc = 0x2d31;
}

/* 0x2d31 BEQ 19 -> 0x2d46. */
void step_note_on_setup_beq_19_0x2d46(void)
{
    beq(0x2d46, 0x2d33);
}

/* 0x2d33 SUB #0x40 r2. */
void step_note_on_setup_sub_0x40_r2_b(void)
{
    sub_imm8_reg(mcu.r[2], 0x40);
    mcu.pc = 0x2d36;
}

/* 0x2d36 BCC 8 -> 0x2d40. */
void step_note_on_setup_bcc_8_0x2d40(void)
{
    bcc(0x2d40, 0x2d38);
}

/* 0x2d38 ADD r2 r3. */
void step_note_on_setup_add_r2_r3_c(void)
{
    add8_reg(mcu.r[3], mcu.r[2]);
    mcu.pc = 0x2d3a;
}

/* 0x2d3a BPL 10 -> 0x2d46. */
void step_note_on_setup_bpl_10_0x2d46(void)
{
    bpl(0x2d46, 0x2d3c);
}

/* 0x2d3c CLR r3. */
void step_note_on_setup_clr_r3_o(void)
{
    clr8_reg(mcu.r[3]);
    mcu.pc = 0x2d3e;
}

/* 0x2d3e BRA 6 -> 0x2d46. */
void step_note_on_setup_bra_6_0x2d46(void)
{
    mcu.pc = 0x2d46;
}

/* 0x2d40 ADD r2 r3. */
void step_note_on_setup_add_r2_r3_d(void)
{
    add8_reg(mcu.r[3], mcu.r[2]);
    mcu.pc = 0x2d42;
}

/* 0x2d42 BVC 2 -> 0x2d46. */
void step_note_on_setup_bvc_2_0x2d46(void)
{
    bvc(0x2d46, 0x2d44);
}

/* 0x2d44 move r3 #0x7f. */
void step_note_on_setup_move_r3_0x7f_b(void)
{
    move8_imm(mcu.r[3], 0x7f);
    mcu.pc = 0x2d46;
}

/* 0x2d46 MOVG3 r3 -> @r0+54. */
void step_note_on_setup_movg3_r3_r0_54(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x36), mcu.r[3]);
    mcu.pc = 0x2d49;
}

/* 0x2d49 BRA 27 -> 0x2d66. */
void step_note_on_setup_bra_27_0x2d66(void)
{
    mcu.pc = 0x2d66;
}

/* 0x2d4b BSET_ORC #0x0700 r0. */
void step_note_on_setup_bset_orc_0x0700_r0_c(void)
{
    orc_imm16(0x0700);
    mcu.pc = 0x2d4f;
}

/* 0x2d4f MOVG #0x1e -> (br,$3e). */
void step_note_on_setup_movg_0x1e_br_3e_b(void)
{
    mov_imm8_mem(ea_br(0x3e), 0x1e);
    mcu.pc = 0x2d53;
}

/* 0x2d53 movl r3 @(br,$34). */
void step_note_on_setup_movl_r3_br_34(void)
{
    movl_br(mcu.r[3], 0x34);
    mcu.pc = 0x2d55;
}

/* 0x2d55 movlw r3 @(br,$3a). */
void step_note_on_setup_movlw_r3_br_3a(void)
{
    movlw_br(mcu.r[3], 0x3a);
    mcu.pc = 0x2d57;
}

/* 0x2d57 BCLR_ANDC #0xf8ff r0. */
void step_note_on_setup_bclr_andc_0xf8ff_r0_c(void)
{
    andc_imm16(0xf8ff);
    mcu.pc = 0x2d5b;
}

/* 0x2d5b CLR r3. */
void step_note_on_setup_clr_r3_p(void)
{
    clr8_reg(mcu.r[3]);
    mcu.pc = 0x2d5d;
}

/* 0x2d5d SWAP r3. */
void step_note_on_setup_swap_r3_b(void)
{
    swap16(mcu.r[3]);
    mcu.pc = 0x2d5f;
}

/* 0x2d5f SHLR r3. */
void step_note_on_setup_shlr_r3(void)
{
    shlr16(mcu.r[3]);
    mcu.pc = 0x2d61;
}

/* 0x2d61 MOVG #0xffff -> @r0+54. */
void step_note_on_setup_movg_0xffff_r0_54(void)
{
    mov_imm16_mem(ea_r(0, (uint16_t)(int8_t)0x36), 0xffff);
    mcu.pc = 0x2d66;
}

/* 0x2d66 MOVG2 @r3+0x6a03 r4. */
void step_note_on_setup_movg2_r3_0x6a03_r4(void)
{
    load8(mcu.r[4], ea_r(3, 0x6a03));
    mcu.pc = 0x2d6a;
}

/* 0x2d6a movi r2 #0x0080. */
void step_note_on_setup_movi_r2_0x0080(void)
{
    movi16(mcu.r[2], 0x0080);
    mcu.pc = 0x2d6d;
}

/* 0x2d6d SUB r3 r2. */
void step_note_on_setup_sub_r3_r2(void)
{
    sub8_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2d6f;
}

/* 0x2d6f MOVG2 @r2+0x6a03 r5. */
void step_note_on_setup_movg2_r2_0x6a03_r5(void)
{
    load8(mcu.r[5], ea_r(2, 0x6a03));
    mcu.pc = 0x2d73;
}

/* 0x2d73 SWAP r5. */
void step_note_on_setup_swap_r5_c(void)
{
    swap16(mcu.r[5]);
    mcu.pc = 0x2d75;
}

/* 0x2d75 MOVG2 r4 r5. */
void step_note_on_setup_movg2_r4_r5_c(void)
{
    mov8_reg(mcu.r[5], mcu.r[4]);
    mcu.pc = 0x2d77;
}

/* 0x2d77 MOVG3 r5 -> @r0+52. */
void step_note_on_setup_movg3_r5_r0_52(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x34), mcu.r[5]);
    mcu.pc = 0x2d7a;
}

/* 0x2d7a MOVG2 @r0+-2 r1. */
void step_note_on_setup_movg2_r0_m2_r1_g(void)
{
    load16(mcu.r[1], ea_r(0, (uint16_t)(int8_t)0xfe));
    mcu.pc = 0x2d7d;
}

/* 0x2d7d bsr 22 -> 0x2d95. */
void step_note_on_setup_bsr_22_0x2d95(void)
{
    call(0x2d7f, 0x2d95);
}

/* 0x2d7f MOVG3 r5 -> @r0+24. */
void step_note_on_setup_movg3_r5_r0_24(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x18), mcu.r[5]);
    mcu.pc = 0x2d82;
}

/* 0x2d82 move r5 #0xba. */
void step_note_on_setup_move_r5_0xba(void)
{
    move8_imm(mcu.r[5], 0xba);
    mcu.pc = 0x2d84;
}

/* 0x2d84 MOVG3 r5 -> @r0+26. */
void step_note_on_setup_movg3_r5_r0_26(void)
{
    store16(ea_r(0, (uint16_t)(int8_t)0x1a), mcu.r[5]);
    mcu.pc = 0x2d87;
}

/* 0x2d87 MOVG2 @r0+-2 r1. */
void step_note_on_setup_movg2_r0_m2_r1_h(void)
{
    load16(mcu.r[1], ea_r(0, (uint16_t)(int8_t)0xfe));
    mcu.pc = 0x2d8a;
}

/* 0x2d8a SUB @r1+0xd0e0 #0x00. */
void step_note_on_setup_sub_r1_0xd0e0_0x00_b(void)
{
    sub8_mem_imm8(ea_r(1, 0xd0e0), 0x00);
    mcu.pc = 0x2d8f;
}

/* 0x2d8f BNE 0x2700 -> 0x5492. */
void step_note_on_setup_bne_0x2700_0x5492(void)
{
    bne(0x5492, 0x2d92);
}

/* 0x2d92 BRA 0x0e1f -> 0x3bb4. */
void step_note_on_setup_bra_0x0e1f_0x3bb4(void)
{
    mcu.pc = 0x3bb4;
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void note_on_setup_fill(void)
{
#if MK2CPP_HAND_NOTE_ON_SETUP
    MK2CPP_HandRegister(0x0000272eu, &step_note_on_setup_btsti_r0_m59_7);
    MK2CPP_HandRegister(0x00002731u, &step_note_on_setup_beq_0x0140_0x2874);
    MK2CPP_HandRegister(0x00002734u, &step_note_on_setup_ldc_r0_0x0099_r4);
    MK2CPP_HandRegister(0x00002738u, &step_note_on_setup_movg2_r0_0x009e_r5);
    MK2CPP_HandRegister(0x0000273cu, &step_note_on_setup_movg2_r5_8_r6);
    MK2CPP_HandRegister(0x0000273fu, &step_note_on_setup_movg3_r6_r0_0x00a2);
    MK2CPP_HandRegister(0x00002743u, &step_note_on_setup_movg2_r0_m2_r1);
    MK2CPP_HandRegister(0x00002746u, &step_note_on_setup_add_r1_r1);
    MK2CPP_HandRegister(0x00002748u, &step_note_on_setup_clr_r1_0xcdfe);
    MK2CPP_HandRegister(0x0000274cu, &step_note_on_setup_clr_r0_m81);
    MK2CPP_HandRegister(0x0000274fu, &step_note_on_setup_movg2_r5_4_r6);
    MK2CPP_HandRegister(0x00002752u, &step_note_on_setup_btsti_r6_4);
    MK2CPP_HandRegister(0x00002754u, &step_note_on_setup_beq_0x0097_0x27ee);
    MK2CPP_HandRegister(0x00002757u, &step_note_on_setup_movg2_r0_0x009b_r3);
    MK2CPP_HandRegister(0x0000275bu, &step_note_on_setup_stc_r4_r4);
    MK2CPP_HandRegister(0x0000275du, &step_note_on_setup_movi_r1_0x001b);
    MK2CPP_HandRegister(0x00002760u, &step_note_on_setup_movg2_r1_r2);
    MK2CPP_HandRegister(0x00002762u, &step_note_on_setup_add_r2_r2);
    MK2CPP_HandRegister(0x00002764u, &step_note_on_setup_movg2_r2_0x64d6_r2);
    MK2CPP_HandRegister(0x00002768u, &step_note_on_setup_cmp_r0_r2);
    MK2CPP_HandRegister(0x0000276au, &step_note_on_setup_beq_25_0x2785);
    MK2CPP_HandRegister(0x0000276cu, &step_note_on_setup_sub_r2_0_0x000c);
    MK2CPP_HandRegister(0x00002771u, &step_note_on_setup_bhi_18_0x2785);
    MK2CPP_HandRegister(0x00002773u, &step_note_on_setup_cmp_r2_0x009b_r3);
    MK2CPP_HandRegister(0x00002777u, &step_note_on_setup_bne_12_0x2785);
    MK2CPP_HandRegister(0x00002779u, &step_note_on_setup_cmp_r2_0x0099_r4);
    MK2CPP_HandRegister(0x0000277du, &step_note_on_setup_bne_6_0x2785);
    MK2CPP_HandRegister(0x0000277fu, &step_note_on_setup_cmp_r2_0x009e_r5);
    MK2CPP_HandRegister(0x00002783u, &step_note_on_setup_beq_5_0x278a);
    MK2CPP_HandRegister(0x00002785u, &step_note_on_setup_cntjmp_r1_m40_0x2760);
    MK2CPP_HandRegister(0x00002788u, &step_note_on_setup_bra_100_0x27ee);
    MK2CPP_HandRegister(0x0000278au, &step_note_on_setup_movg2_r2_m88_r6);
    MK2CPP_HandRegister(0x0000278du, &step_note_on_setup_movg3_r6_r0_m88);
    MK2CPP_HandRegister(0x00002790u, &step_note_on_setup_movg2_r2_m86_r6);
    MK2CPP_HandRegister(0x00002793u, &step_note_on_setup_movg3_r6_r0_m86);
    MK2CPP_HandRegister(0x00002796u, &step_note_on_setup_movg2_r2_m84_r6);
    MK2CPP_HandRegister(0x00002799u, &step_note_on_setup_movg3_r6_r0_m84);
    MK2CPP_HandRegister(0x0000279cu, &step_note_on_setup_movg2_r2_m80_r6);
    MK2CPP_HandRegister(0x0000279fu, &step_note_on_setup_movg3_r6_r0_m80);
    MK2CPP_HandRegister(0x000027a2u, &step_note_on_setup_movg2_r2_m82_r6);
    MK2CPP_HandRegister(0x000027a5u, &step_note_on_setup_movg3_r6_r0_m82);
    MK2CPP_HandRegister(0x000027a8u, &step_note_on_setup_movg2_r2_m78_r6);
    MK2CPP_HandRegister(0x000027abu, &step_note_on_setup_movg3_r6_r0_m78);
    MK2CPP_HandRegister(0x000027aeu, &step_note_on_setup_movg2_r2_m76_r6);
    MK2CPP_HandRegister(0x000027b1u, &step_note_on_setup_movg3_r6_r0_m76);
    MK2CPP_HandRegister(0x000027b4u, &step_note_on_setup_movg2_r2_m74_r6);
    MK2CPP_HandRegister(0x000027b7u, &step_note_on_setup_movg3_r6_r0_m74);
    MK2CPP_HandRegister(0x000027bau, &step_note_on_setup_movg2_r2_m72_r6);
    MK2CPP_HandRegister(0x000027bdu, &step_note_on_setup_movg3_r6_r0_m72);
    MK2CPP_HandRegister(0x000027c0u, &step_note_on_setup_movg2_r2_m70_r6);
    MK2CPP_HandRegister(0x000027c3u, &step_note_on_setup_movg3_r6_r0_m70);
    MK2CPP_HandRegister(0x000027c6u, &step_note_on_setup_movg2_r2_m68_r6);
    MK2CPP_HandRegister(0x000027c9u, &step_note_on_setup_movg3_r6_r0_m68);
    MK2CPP_HandRegister(0x000027ccu, &step_note_on_setup_movg2_r2_m66_r6);
    MK2CPP_HandRegister(0x000027cfu, &step_note_on_setup_movg3_r6_r0_m66);
    MK2CPP_HandRegister(0x000027d2u, &step_note_on_setup_movg2_r2_m64_r6);
    MK2CPP_HandRegister(0x000027d5u, &step_note_on_setup_movg3_r6_r0_m64);
    MK2CPP_HandRegister(0x000027d8u, &step_note_on_setup_movg2_r2_m62_r6);
    MK2CPP_HandRegister(0x000027dbu, &step_note_on_setup_movg3_r6_r0_m62);
    MK2CPP_HandRegister(0x000027deu, &step_note_on_setup_movg2_r0_m2_r1_b);
    MK2CPP_HandRegister(0x000027e1u, &step_note_on_setup_add_r1_r1_b);
    MK2CPP_HandRegister(0x000027e3u, &step_note_on_setup_movg3_r2_r1_0xcdfe);
    MK2CPP_HandRegister(0x000027e7u, &step_note_on_setup_movg_0xff_r0_m81);
    MK2CPP_HandRegister(0x000027ebu, &step_note_on_setup_bra_0x0086_0x2874);
    MK2CPP_HandRegister(0x000027eeu, &step_note_on_setup_movg2_r6_r3);
    MK2CPP_HandRegister(0x000027f0u, &step_note_on_setup_and_0x00c0_r3);
    MK2CPP_HandRegister(0x000027f4u, &step_note_on_setup_swap_r3);
    MK2CPP_HandRegister(0x000027f6u, &step_note_on_setup_movg3_r3_r0_m72);
    MK2CPP_HandRegister(0x000027f9u, &step_note_on_setup_movg2_r6_r3_b);
    MK2CPP_HandRegister(0x000027fbu, &step_note_on_setup_and_0x000f_r3);
    MK2CPP_HandRegister(0x000027ffu, &step_note_on_setup_movg2_r3_0x7207_r3);
    MK2CPP_HandRegister(0x00002803u, &step_note_on_setup_movg3_r3_r0_m74);
    MK2CPP_HandRegister(0x00002806u, &step_note_on_setup_clr_r0_m64);
    MK2CPP_HandRegister(0x00002809u, &step_note_on_setup_clr_r0_m70);
    MK2CPP_HandRegister(0x0000280cu, &step_note_on_setup_clr_r0_m68);
    MK2CPP_HandRegister(0x0000280fu, &step_note_on_setup_clr_r0_m62);
    MK2CPP_HandRegister(0x00002812u, &step_note_on_setup_clr_r0_m88);
    MK2CPP_HandRegister(0x00002815u, &step_note_on_setup_clr_r0_m86);
    MK2CPP_HandRegister(0x00002818u, &step_note_on_setup_clr_r0_m84);
    MK2CPP_HandRegister(0x0000281bu, &step_note_on_setup_clr_r3);
    MK2CPP_HandRegister(0x0000281du, &step_note_on_setup_movg2_r5_6_r3);
    MK2CPP_HandRegister(0x00002820u, &step_note_on_setup_bmi_8_0x282a);
    MK2CPP_HandRegister(0x00002822u, &step_note_on_setup_add_r3_r3);
    MK2CPP_HandRegister(0x00002824u, &step_note_on_setup_movg2_r3_0x6e86_r3);
    MK2CPP_HandRegister(0x00002828u, &step_note_on_setup_bra_2_0x282c);
    MK2CPP_HandRegister(0x0000282au, &step_note_on_setup_clr_r3_b);
    MK2CPP_HandRegister(0x0000282cu, &step_note_on_setup_movg3_r3_r0_m78);
    MK2CPP_HandRegister(0x0000282fu, &step_note_on_setup_clr_r3_c);
    MK2CPP_HandRegister(0x00002831u, &step_note_on_setup_movg2_r5_7_r3);
    MK2CPP_HandRegister(0x00002834u, &step_note_on_setup_add_r3_r3_b);
    MK2CPP_HandRegister(0x00002836u, &step_note_on_setup_movg2_r3_0x6e86_r3_b);
    MK2CPP_HandRegister(0x0000283au, &step_note_on_setup_movg3_r3_r0_m76);
    MK2CPP_HandRegister(0x0000283du, &step_note_on_setup_clr_r3_d);
    MK2CPP_HandRegister(0x0000283fu, &step_note_on_setup_movg2_r5_5_r3);
    MK2CPP_HandRegister(0x00002842u, &step_note_on_setup_movg3_r3_r0_m82);
    MK2CPP_HandRegister(0x00002845u, &step_note_on_setup_movg2_dp_0xad2a_r6);
    MK2CPP_HandRegister(0x00002849u, &step_note_on_setup_movg3_r6_dp_0xad2c);
    MK2CPP_HandRegister(0x0000284du, &step_note_on_setup_movg_0x0001_dp_0xad2a);
    MK2CPP_HandRegister(0x00002853u, &step_note_on_setup_bset_orc_0x0700_r0);
    MK2CPP_HandRegister(0x00002857u, &step_note_on_setup_movg_0x1e_br_3e);
    MK2CPP_HandRegister(0x0000285bu, &step_note_on_setup_movl_r4_br_34);
    MK2CPP_HandRegister(0x0000285du, &step_note_on_setup_movlw_r4_br_3a);
    MK2CPP_HandRegister(0x0000285fu, &step_note_on_setup_movg3_r4_r0_m66);
    MK2CPP_HandRegister(0x00002862u, &step_note_on_setup_movg3_r4_r0_m64);
    MK2CPP_HandRegister(0x00002865u, &step_note_on_setup_bsr16_0x37fe);
    MK2CPP_HandRegister(0x00002868u, &step_note_on_setup_bclr_andc_0xf8ff_r0);
    MK2CPP_HandRegister(0x0000286cu, &step_note_on_setup_movg2_dp_0xad2c_r6);
    MK2CPP_HandRegister(0x00002870u, &step_note_on_setup_movg3_r6_dp_0xad2a);
    MK2CPP_HandRegister(0x00002874u, &step_note_on_setup_ldc_r0_0x009a_r4);
    MK2CPP_HandRegister(0x00002878u, &step_note_on_setup_movg2_r0_0x00a0_r5);
    MK2CPP_HandRegister(0x0000287cu, &step_note_on_setup_movg2_r5_1_r3);
    MK2CPP_HandRegister(0x0000287fu, &step_note_on_setup_movg3_r3_r0_m20);
    MK2CPP_HandRegister(0x00002882u, &step_note_on_setup_movg2_r5_2_r4);
    MK2CPP_HandRegister(0x00002885u, &step_note_on_setup_movg3_r4_r0_m16);
    MK2CPP_HandRegister(0x00002888u, &step_note_on_setup_movg2_r0_m2_r1_c);
    MK2CPP_HandRegister(0x0000288bu, &step_note_on_setup_btsti_r0_m59_7_b);
    MK2CPP_HandRegister(0x0000288eu, &step_note_on_setup_bne_15_0x289f);
    MK2CPP_HandRegister(0x00002890u, &step_note_on_setup_movg2_r5_4_r6_b);
    MK2CPP_HandRegister(0x00002893u, &step_note_on_setup_clr_r2);
    MK2CPP_HandRegister(0x00002895u, &step_note_on_setup_add_r4_r6);
    MK2CPP_HandRegister(0x00002897u, &step_note_on_setup_addx_r3_r2);
    MK2CPP_HandRegister(0x00002899u, &step_note_on_setup_movg3_r2_r0_m20);
    MK2CPP_HandRegister(0x0000289cu, &step_note_on_setup_movg3_r6_r0_m16);
    MK2CPP_HandRegister(0x0000289fu, &step_note_on_setup_add_r5_6_r4);
    MK2CPP_HandRegister(0x000028a2u, &step_note_on_setup_addx_0x00_r3);
    MK2CPP_HandRegister(0x000028a5u, &step_note_on_setup_movg3_r3_r0_m19);
    MK2CPP_HandRegister(0x000028a8u, &step_note_on_setup_movg3_r4_r0_m14);
    MK2CPP_HandRegister(0x000028abu, &step_note_on_setup_sub_r5_8_r4);
    MK2CPP_HandRegister(0x000028aeu, &step_note_on_setup_subx_0x00_r3);
    MK2CPP_HandRegister(0x000028b1u, &step_note_on_setup_movg3_r3_r0_m18);
    MK2CPP_HandRegister(0x000028b4u, &step_note_on_setup_movg3_r4_r0_m12);
    MK2CPP_HandRegister(0x000028b7u, &step_note_on_setup_add_r3_r3_c);
    MK2CPP_HandRegister(0x000028b9u, &step_note_on_setup_add_r3_r3_d);
    MK2CPP_HandRegister(0x000028bbu, &step_note_on_setup_add_r3_r3_e);
    MK2CPP_HandRegister(0x000028bdu, &step_note_on_setup_add_r3_r3_f);
    MK2CPP_HandRegister(0x000028bfu, &step_note_on_setup_movg2_r5_10_r4);
    MK2CPP_HandRegister(0x000028c2u, &step_note_on_setup_movg2_r4_r5);
    MK2CPP_HandRegister(0x000028c4u, &step_note_on_setup_and_0x0001_r4);
    MK2CPP_HandRegister(0x000028c8u, &step_note_on_setup_swap_r4);
    MK2CPP_HandRegister(0x000028cau, &step_note_on_setup_shlr_r4);
    MK2CPP_HandRegister(0x000028ccu, &step_note_on_setup_shlr_r4_b);
    MK2CPP_HandRegister(0x000028ceu, &step_note_on_setup_movg2_r4_r3);
    MK2CPP_HandRegister(0x000028d0u, &step_note_on_setup_or_r1_r3);
    MK2CPP_HandRegister(0x000028d2u, &step_note_on_setup_movg3_r3_r0_m10);
    MK2CPP_HandRegister(0x000028d5u, &step_note_on_setup_and_0x02_r5);
    MK2CPP_HandRegister(0x000028d8u, &step_note_on_setup_movg3_r5_r0_m17);
    MK2CPP_HandRegister(0x000028dbu, &step_note_on_setup_btsti_r0_m59_7_c);
    MK2CPP_HandRegister(0x000028deu, &step_note_on_setup_bne_69_0x2925);
    MK2CPP_HandRegister(0x000028e0u, &step_note_on_setup_bset_orc_0x0700_r0_b);
    MK2CPP_HandRegister(0x000028e4u, &step_note_on_setup_movs_r1_br_3e);
    MK2CPP_HandRegister(0x000028e6u, &step_note_on_setup_movg_0x00b4_br_16);
    MK2CPP_HandRegister(0x000028ebu, &step_note_on_setup_movg_0x00b4_r0_26);
    MK2CPP_HandRegister(0x000028f0u, &step_note_on_setup_movg2_r0_28_r6);
    MK2CPP_HandRegister(0x000028f3u, &step_note_on_setup_move_r6_0xaf);
    MK2CPP_HandRegister(0x000028f5u, &step_note_on_setup_movg3_r6_r0_30);
    MK2CPP_HandRegister(0x000028f8u, &step_note_on_setup_movg2_r0_36_r6);
    MK2CPP_HandRegister(0x000028fbu, &step_note_on_setup_movg3_r6_r0_m22);
    MK2CPP_HandRegister(0x000028feu, &step_note_on_setup_move_r6_0xaf_b);
    MK2CPP_HandRegister(0x00002900u, &step_note_on_setup_movg3_r6_r0_38);
    MK2CPP_HandRegister(0x00002903u, &step_note_on_setup_movg3_r6_r0_m24);
    MK2CPP_HandRegister(0x00002906u, &step_note_on_setup_movg2_r0_0_r6);
    MK2CPP_HandRegister(0x00002909u, &step_note_on_setup_cmp_r6_w_0x0018);
    MK2CPP_HandRegister(0x0000290cu, &step_note_on_setup_beq_8_0x2916);
    MK2CPP_HandRegister(0x0000290eu, &step_note_on_setup_movg_0x0018_r0_0);
    MK2CPP_HandRegister(0x00002913u, &step_note_on_setup_movg3_r6_r0_6);
    MK2CPP_HandRegister(0x00002916u, &step_note_on_setup_bclr_andc_0xf8ff_r0_b);
    MK2CPP_HandRegister(0x0000291au, &step_note_on_setup_sub_r1_0xd0e0_0x00);
    MK2CPP_HandRegister(0x0000291fu, &step_note_on_setup_bne_0x2b70_0x5492);
    MK2CPP_HandRegister(0x00002922u, &step_note_on_setup_bra_0x1ca7_0x45cc);
    MK2CPP_HandRegister(0x00002925u, &step_note_on_setup_movg_0xffff_r0_6);
    MK2CPP_HandRegister(0x0000292au, &step_note_on_setup_ldc_r0_0x0099_r4_b);
    MK2CPP_HandRegister(0x0000292eu, &step_note_on_setup_movg2_r0_0x009e_r5_b);
    MK2CPP_HandRegister(0x00002932u, &step_note_on_setup_movi_r2_0x00ff);
    MK2CPP_HandRegister(0x00002935u, &step_note_on_setup_clr_r3_e);
    MK2CPP_HandRegister(0x00002937u, &step_note_on_setup_movg2_r5_69_r3);
    MK2CPP_HandRegister(0x0000293au, &step_note_on_setup_sub_r3_0x6883_r2);
    MK2CPP_HandRegister(0x0000293eu, &step_note_on_setup_bhi_2_0x2942);
    MK2CPP_HandRegister(0x00002940u, &step_note_on_setup_move_r2_0x01);
    MK2CPP_HandRegister(0x00002942u, &step_note_on_setup_clr_r4);
    MK2CPP_HandRegister(0x00002944u, &step_note_on_setup_movg2_r0_m2_r1_d);
    MK2CPP_HandRegister(0x00002947u, &step_note_on_setup_movg2_r1_0xd134_r4);
    MK2CPP_HandRegister(0x0000294bu, &step_note_on_setup_ldc_0x03_r5);
    MK2CPP_HandRegister(0x0000294eu, &step_note_on_setup_movg2_r5_70_r3);
    MK2CPP_HandRegister(0x00002951u, &step_note_on_setup_add_r3_r3_g);
    MK2CPP_HandRegister(0x00002953u, &step_note_on_setup_movg2_r3_0xdd7c_r3);
    MK2CPP_HandRegister(0x00002957u, &step_note_on_setup_add_r4_r3);
    MK2CPP_HandRegister(0x00002959u, &step_note_on_setup_movg2_r3_r4);
    MK2CPP_HandRegister(0x0000295bu, &step_note_on_setup_ldc_0x00_r5);
    MK2CPP_HandRegister(0x0000295eu, &step_note_on_setup_clr_r3_f);
    MK2CPP_HandRegister(0x00002960u, &step_note_on_setup_movg2_r5_71_r3);
    MK2CPP_HandRegister(0x00002963u, &step_note_on_setup_sub_0x40_r3);
    MK2CPP_HandRegister(0x00002966u, &step_note_on_setup_bcc_11_0x2973);
    MK2CPP_HandRegister(0x00002968u, &step_note_on_setup_neg_r3);
    MK2CPP_HandRegister(0x0000296au, &step_note_on_setup_sub_0x80_r4);
    MK2CPP_HandRegister(0x0000296du, &step_note_on_setup_bcc_11_0x297a);
    MK2CPP_HandRegister(0x0000296fu, &step_note_on_setup_neg_r4);
    MK2CPP_HandRegister(0x00002971u, &step_note_on_setup_bra_28_0x298f);
    MK2CPP_HandRegister(0x00002973u, &step_note_on_setup_sub_0x80_r4_b);
    MK2CPP_HandRegister(0x00002976u, &step_note_on_setup_bcc_23_0x298f);
    MK2CPP_HandRegister(0x00002978u, &step_note_on_setup_neg_r4_b);
    MK2CPP_HandRegister(0x0000297au, &step_note_on_setup_mulxu_r3_0x650e_r4);
    MK2CPP_HandRegister(0x0000297eu, &step_note_on_setup_add_r4_r4);
    MK2CPP_HandRegister(0x00002980u, &step_note_on_setup_swap_r4_b);
    MK2CPP_HandRegister(0x00002982u, &step_note_on_setup_movg2_r4_r3_b);
    MK2CPP_HandRegister(0x00002984u, &step_note_on_setup_sub_r3_0x673a_r2);
    MK2CPP_HandRegister(0x00002988u, &step_note_on_setup_bhi_24_0x29a2);
    MK2CPP_HandRegister(0x0000298au, &step_note_on_setup_movi_r2_0x0001);
    MK2CPP_HandRegister(0x0000298du, &step_note_on_setup_bra_19_0x29a2);
    MK2CPP_HandRegister(0x0000298fu, &step_note_on_setup_mulxu_r3_0x650e_r4_b);
    MK2CPP_HandRegister(0x00002993u, &step_note_on_setup_add_r4_r4_b);
    MK2CPP_HandRegister(0x00002995u, &step_note_on_setup_swap_r4_c);
    MK2CPP_HandRegister(0x00002997u, &step_note_on_setup_movg2_r4_r3_c);
    MK2CPP_HandRegister(0x00002999u, &step_note_on_setup_add_r3_0x673a_r2);
    MK2CPP_HandRegister(0x0000299du, &step_note_on_setup_bcc_3_0x29a2);
    MK2CPP_HandRegister(0x0000299fu, &step_note_on_setup_movi_r2_0x00ff_b);
    MK2CPP_HandRegister(0x000029a2u, &step_note_on_setup_movg2_r1_0xcee8_r3);
    MK2CPP_HandRegister(0x000029a6u, &step_note_on_setup_sub_r3_0x6883_r2_b);
    MK2CPP_HandRegister(0x000029aau, &step_note_on_setup_bhi_2_0x29ae);
    MK2CPP_HandRegister(0x000029acu, &step_note_on_setup_move_r2_0x01_b);
    MK2CPP_HandRegister(0x000029aeu, &step_note_on_setup_ldc_r0_0x009a_r4_b);
    MK2CPP_HandRegister(0x000029b2u, &step_note_on_setup_movg2_r0_0x00a0_r5_b);
    MK2CPP_HandRegister(0x000029b6u, &step_note_on_setup_movg2_r5_0_r3);
    MK2CPP_HandRegister(0x000029b9u, &step_note_on_setup_sub_r3_0x6883_r2_c);
    MK2CPP_HandRegister(0x000029bdu, &step_note_on_setup_bhi_2_0x29c1);
    MK2CPP_HandRegister(0x000029bfu, &step_note_on_setup_move_r2_0x01_c);
    MK2CPP_HandRegister(0x000029c1u, &step_note_on_setup_ldc_r0_0x0098_r4);
    MK2CPP_HandRegister(0x000029c5u, &step_note_on_setup_movg2_r0_0x009c_r5);
    MK2CPP_HandRegister(0x000029c9u, &step_note_on_setup_movg2_r5_12_r3);
    MK2CPP_HandRegister(0x000029ccu, &step_note_on_setup_sub_r3_0x6883_r2_d);
    MK2CPP_HandRegister(0x000029d0u, &step_note_on_setup_bhi_2_0x29d4);
    MK2CPP_HandRegister(0x000029d2u, &step_note_on_setup_move_r2_0x01_d);
    MK2CPP_HandRegister(0x000029d4u, &step_note_on_setup_ldc_r0_0x0099_r4_c);
    MK2CPP_HandRegister(0x000029d8u, &step_note_on_setup_movg2_r0_0x009e_r5_c);
    MK2CPP_HandRegister(0x000029dcu, &step_note_on_setup_movg2_r2_r6);
    MK2CPP_HandRegister(0x000029deu, &step_note_on_setup_movg2_r5_74_r3);
    MK2CPP_HandRegister(0x000029e1u, &step_note_on_setup_sub_r3_0x6883_r2_e);
    MK2CPP_HandRegister(0x000029e5u, &step_note_on_setup_bcc_2_0x29e9);
    MK2CPP_HandRegister(0x000029e7u, &step_note_on_setup_clr_r2_b);
    MK2CPP_HandRegister(0x000029e9u, &step_note_on_setup_movg2_r2_0x6903_r3);
    MK2CPP_HandRegister(0x000029edu, &step_note_on_setup_movg3_r3_r0_97);
    MK2CPP_HandRegister(0x000029f0u, &step_note_on_setup_movg2_r6_r2);
    MK2CPP_HandRegister(0x000029f2u, &step_note_on_setup_movg2_r5_75_r3);
    MK2CPP_HandRegister(0x000029f5u, &step_note_on_setup_sub_r3_0x6883_r2_f);
    MK2CPP_HandRegister(0x000029f9u, &step_note_on_setup_bcc_2_0x29fd);
    MK2CPP_HandRegister(0x000029fbu, &step_note_on_setup_clr_r2_c);
    MK2CPP_HandRegister(0x000029fdu, &step_note_on_setup_movg2_r2_0x6903_r3_b);
    MK2CPP_HandRegister(0x00002a01u, &step_note_on_setup_movg3_r3_r0_98);
    MK2CPP_HandRegister(0x00002a04u, &step_note_on_setup_movg2_r6_r2_b);
    MK2CPP_HandRegister(0x00002a06u, &step_note_on_setup_movg2_r5_76_r3);
    MK2CPP_HandRegister(0x00002a09u, &step_note_on_setup_sub_r3_0x6883_r2_g);
    MK2CPP_HandRegister(0x00002a0du, &step_note_on_setup_bcc_2_0x2a11);
    MK2CPP_HandRegister(0x00002a0fu, &step_note_on_setup_clr_r2_d);
    MK2CPP_HandRegister(0x00002a11u, &step_note_on_setup_movg2_r2_0x6903_r3_c);
    MK2CPP_HandRegister(0x00002a15u, &step_note_on_setup_movg3_r3_r0_99);
    MK2CPP_HandRegister(0x00002a18u, &step_note_on_setup_movg2_r6_r2_c);
    MK2CPP_HandRegister(0x00002a1au, &step_note_on_setup_movg2_r5_77_r3);
    MK2CPP_HandRegister(0x00002a1du, &step_note_on_setup_sub_r3_0x6883_r2_h);
    MK2CPP_HandRegister(0x00002a21u, &step_note_on_setup_bcc_2_0x2a25);
    MK2CPP_HandRegister(0x00002a23u, &step_note_on_setup_clr_r2_e);
    MK2CPP_HandRegister(0x00002a25u, &step_note_on_setup_movg2_r2_0x6903_r3_d);
    MK2CPP_HandRegister(0x00002a29u, &step_note_on_setup_movg3_r3_r0_100);
    MK2CPP_HandRegister(0x00002a2cu, &step_note_on_setup_clr_r0_96);
    MK2CPP_HandRegister(0x00002a2fu, &step_note_on_setup_clr_r4_b);
    MK2CPP_HandRegister(0x00002a31u, &step_note_on_setup_movg2_r0_m2_r1_e);
    MK2CPP_HandRegister(0x00002a34u, &step_note_on_setup_movg2_r1_0xd134_r4_b);
    MK2CPP_HandRegister(0x00002a38u, &step_note_on_setup_movg2_r5_85_r2);
    MK2CPP_HandRegister(0x00002a3bu, &step_note_on_setup_add_r2_r2_b);
    MK2CPP_HandRegister(0x00002a3du, &step_note_on_setup_ldc_0x03_r5_b);
    MK2CPP_HandRegister(0x00002a40u, &step_note_on_setup_movg2_r2_0xdd9c_r2);
    MK2CPP_HandRegister(0x00002a44u, &step_note_on_setup_add_r4_r2);
    MK2CPP_HandRegister(0x00002a46u, &step_note_on_setup_movg2_r2_r4);
    MK2CPP_HandRegister(0x00002a48u, &step_note_on_setup_ldc_0x00_r5_b);
    MK2CPP_HandRegister(0x00002a4bu, &step_note_on_setup_clr_r3_g);
    MK2CPP_HandRegister(0x00002a4du, &step_note_on_setup_movg2_r5_87_r3);
    MK2CPP_HandRegister(0x00002a50u, &step_note_on_setup_sub_0x40_r3_b);
    MK2CPP_HandRegister(0x00002a53u, &step_note_on_setup_beq_58_0x2a8f);
    MK2CPP_HandRegister(0x00002a55u, &step_note_on_setup_bcc_8_0x2a5f);
    MK2CPP_HandRegister(0x00002a57u, &step_note_on_setup_neg_r3_b);
    MK2CPP_HandRegister(0x00002a59u, &step_note_on_setup_neg_r4_c);
    MK2CPP_HandRegister(0x00002a5bu, &step_note_on_setup_bne_2_0x2a5f);
    MK2CPP_HandRegister(0x00002a5du, &step_note_on_setup_move_r4_0xff);
    MK2CPP_HandRegister(0x00002a5fu, &step_note_on_setup_sub_0x80_r4_c);
    MK2CPP_HandRegister(0x00002a62u, &step_note_on_setup_beq_43_0x2a8f);
    MK2CPP_HandRegister(0x00002a64u, &step_note_on_setup_bcc_17_0x2a77);
    MK2CPP_HandRegister(0x00002a66u, &step_note_on_setup_neg_r4_d);
    MK2CPP_HandRegister(0x00002a68u, &step_note_on_setup_mulxu_r3_0x650e_r4_c);
    MK2CPP_HandRegister(0x00002a6cu, &step_note_on_setup_add_r4_r4_c);
    MK2CPP_HandRegister(0x00002a6eu, &step_note_on_setup_swap_r4_d);
    MK2CPP_HandRegister(0x00002a70u, &step_note_on_setup_movi_r3_0x0080);
    MK2CPP_HandRegister(0x00002a73u, &step_note_on_setup_sub_r4_r3);
    MK2CPP_HandRegister(0x00002a75u, &step_note_on_setup_bra_13_0x2a84);
    MK2CPP_HandRegister(0x00002a77u, &step_note_on_setup_mulxu_r3_0x650e_r4_d);
    MK2CPP_HandRegister(0x00002a7bu, &step_note_on_setup_add_r4_r4_d);
    MK2CPP_HandRegister(0x00002a7du, &step_note_on_setup_swap_r4_e);
    MK2CPP_HandRegister(0x00002a7fu, &step_note_on_setup_movi_r3_0x0080_b);
    MK2CPP_HandRegister(0x00002a82u, &step_note_on_setup_add_r4_r3_b);
    MK2CPP_HandRegister(0x00002a84u, &step_note_on_setup_add_r3_r3_h);
    MK2CPP_HandRegister(0x00002a86u, &step_note_on_setup_movg2_r3_0x653a_r3);
    MK2CPP_HandRegister(0x00002a8au, &step_note_on_setup_movg3_r3_r0_m34);
    MK2CPP_HandRegister(0x00002a8du, &step_note_on_setup_bra_5_0x2a94);
    MK2CPP_HandRegister(0x00002a8fu, &step_note_on_setup_movg_0x0100_r0_m34);
    MK2CPP_HandRegister(0x00002a94u, &step_note_on_setup_clr_r4_c);
    MK2CPP_HandRegister(0x00002a96u, &step_note_on_setup_movg2_r1_0xd134_r4_c);
    MK2CPP_HandRegister(0x00002a9au, &step_note_on_setup_clr_r2_f);
    MK2CPP_HandRegister(0x00002a9cu, &step_note_on_setup_movg2_r5_86_r2);
    MK2CPP_HandRegister(0x00002a9fu, &step_note_on_setup_add_r2_r2_c);
    MK2CPP_HandRegister(0x00002aa1u, &step_note_on_setup_ldc_0x03_r5_c);
    MK2CPP_HandRegister(0x00002aa4u, &step_note_on_setup_movg2_r2_0xddbc_r2);
    MK2CPP_HandRegister(0x00002aa8u, &step_note_on_setup_add_r4_r2_b);
    MK2CPP_HandRegister(0x00002aaau, &step_note_on_setup_movg2_r2_r4_b);
    MK2CPP_HandRegister(0x00002aacu, &step_note_on_setup_ldc_0x00_r5_c);
    MK2CPP_HandRegister(0x00002aafu, &step_note_on_setup_clr_r3_h);
    MK2CPP_HandRegister(0x00002ab1u, &step_note_on_setup_movg2_r5_88_r3);
    MK2CPP_HandRegister(0x00002ab4u, &step_note_on_setup_sub_0x40_r3_c);
    MK2CPP_HandRegister(0x00002ab7u, &step_note_on_setup_beq_58_0x2af3);
    MK2CPP_HandRegister(0x00002ab9u, &step_note_on_setup_bcc_8_0x2ac3);
    MK2CPP_HandRegister(0x00002abbu, &step_note_on_setup_neg_r3_c);
    MK2CPP_HandRegister(0x00002abdu, &step_note_on_setup_neg_r4_e);
    MK2CPP_HandRegister(0x00002abfu, &step_note_on_setup_bne_2_0x2ac3);
    MK2CPP_HandRegister(0x00002ac1u, &step_note_on_setup_move_r4_0xff_b);
    MK2CPP_HandRegister(0x00002ac3u, &step_note_on_setup_sub_0x80_r4_d);
    MK2CPP_HandRegister(0x00002ac6u, &step_note_on_setup_beq_43_0x2af3);
    MK2CPP_HandRegister(0x00002ac8u, &step_note_on_setup_bcc_17_0x2adb);
    MK2CPP_HandRegister(0x00002acau, &step_note_on_setup_neg_r4_f);
    MK2CPP_HandRegister(0x00002accu, &step_note_on_setup_mulxu_r3_0x650e_r4_e);
    MK2CPP_HandRegister(0x00002ad0u, &step_note_on_setup_add_r4_r4_e);
    MK2CPP_HandRegister(0x00002ad2u, &step_note_on_setup_swap_r4_f);
    MK2CPP_HandRegister(0x00002ad4u, &step_note_on_setup_movi_r3_0x0080_c);
    MK2CPP_HandRegister(0x00002ad7u, &step_note_on_setup_sub_r4_r3_b);
    MK2CPP_HandRegister(0x00002ad9u, &step_note_on_setup_bra_13_0x2ae8);
    MK2CPP_HandRegister(0x00002adbu, &step_note_on_setup_mulxu_r3_0x650e_r4_f);
    MK2CPP_HandRegister(0x00002adfu, &step_note_on_setup_add_r4_r4_f);
    MK2CPP_HandRegister(0x00002ae1u, &step_note_on_setup_swap_r4_g);
    MK2CPP_HandRegister(0x00002ae3u, &step_note_on_setup_movi_r3_0x0080_d);
    MK2CPP_HandRegister(0x00002ae6u, &step_note_on_setup_add_r4_r3_c);
    MK2CPP_HandRegister(0x00002ae8u, &step_note_on_setup_add_r3_r3_i);
    MK2CPP_HandRegister(0x00002aeau, &step_note_on_setup_movg2_r3_0x653a_r3_b);
    MK2CPP_HandRegister(0x00002aeeu, &step_note_on_setup_movg3_r3_r0_m32);
    MK2CPP_HandRegister(0x00002af1u, &step_note_on_setup_bra_5_0x2af8);
    MK2CPP_HandRegister(0x00002af3u, &step_note_on_setup_movg_0x0100_r0_m32);
    MK2CPP_HandRegister(0x00002af8u, &step_note_on_setup_movi_r2_0x007f);
    MK2CPP_HandRegister(0x00002afbu, &step_note_on_setup_sub_r1_0xcee8_r2);
    MK2CPP_HandRegister(0x00002affu, &step_note_on_setup_clr_r3_i);
    MK2CPP_HandRegister(0x00002b01u, &step_note_on_setup_movg2_r5_89_r3);
    MK2CPP_HandRegister(0x00002b04u, &step_note_on_setup_sub_0x40_r3_d);
    MK2CPP_HandRegister(0x00002b07u, &step_note_on_setup_beq_62_0x2b47);
    MK2CPP_HandRegister(0x00002b09u, &step_note_on_setup_bcc_31_0x2b2a);
    MK2CPP_HandRegister(0x00002b0bu, &step_note_on_setup_neg_r3_d);
    MK2CPP_HandRegister(0x00002b0du, &step_note_on_setup_mulxu_r3_0x650e_r2);
    MK2CPP_HandRegister(0x00002b11u, &step_note_on_setup_cmp_r2_w_0x1f41);
    MK2CPP_HandRegister(0x00002b14u, &step_note_on_setup_bcs_5_0x2b1b);
    MK2CPP_HandRegister(0x00002b16u, &step_note_on_setup_movi_r3_0x0004);
    MK2CPP_HandRegister(0x00002b19u, &step_note_on_setup_bra_47_0x2b4a);
    MK2CPP_HandRegister(0x00002b1bu, &step_note_on_setup_movi_r3_0x1fc0);
    MK2CPP_HandRegister(0x00002b1eu, &step_note_on_setup_sub_r2_r3);
    MK2CPP_HandRegister(0x00002b20u, &step_note_on_setup_movg2_r3_r2);
    MK2CPP_HandRegister(0x00002b22u, &step_note_on_setup_mulxu_0x0810_r2_r3);
    MK2CPP_HandRegister(0x00002b26u, &step_note_on_setup_movg2_r2_r3);
    MK2CPP_HandRegister(0x00002b28u, &step_note_on_setup_bra_32_0x2b4a);
    MK2CPP_HandRegister(0x00002b2au, &step_note_on_setup_movi_r4_0x1fc0);
    MK2CPP_HandRegister(0x00002b2du, &step_note_on_setup_mulxu_r3_0x650e_r2_b);
    MK2CPP_HandRegister(0x00002b31u, &step_note_on_setup_sub_r2_r4);
    MK2CPP_HandRegister(0x00002b33u, &step_note_on_setup_cmp_r4_w_0x0020);
    MK2CPP_HandRegister(0x00002b36u, &step_note_on_setup_bcs_10_0x2b42);
    MK2CPP_HandRegister(0x00002b38u, &step_note_on_setup_movi_r2_0x001f);
    MK2CPP_HandRegister(0x00002b3bu, &step_note_on_setup_movi_r3_0xc000);
    MK2CPP_HandRegister(0x00002b3eu, &step_note_on_setup_divxu_r4_r2_r3);
    MK2CPP_HandRegister(0x00002b40u, &step_note_on_setup_bra_8_0x2b4a);
    MK2CPP_HandRegister(0x00002b42u, &step_note_on_setup_movi_r3_0xffff);
    MK2CPP_HandRegister(0x00002b45u, &step_note_on_setup_bra_3_0x2b4a);
    MK2CPP_HandRegister(0x00002b47u, &step_note_on_setup_movi_r3_0x0100);
    MK2CPP_HandRegister(0x00002b4au, &step_note_on_setup_movg3_r3_r0_m30);
    MK2CPP_HandRegister(0x00002b4du, &step_note_on_setup_movi_r2_0x007f_b);
    MK2CPP_HandRegister(0x00002b50u, &step_note_on_setup_sub_r1_0xcee8_r2_b);
    MK2CPP_HandRegister(0x00002b54u, &step_note_on_setup_clr_r3_j);
    MK2CPP_HandRegister(0x00002b56u, &step_note_on_setup_movg2_r5_90_r3);
    MK2CPP_HandRegister(0x00002b59u, &step_note_on_setup_sub_0x40_r3_e);
    MK2CPP_HandRegister(0x00002b5cu, &step_note_on_setup_beq_62_0x2b9c);
    MK2CPP_HandRegister(0x00002b5eu, &step_note_on_setup_bcc_31_0x2b7f);
    MK2CPP_HandRegister(0x00002b60u, &step_note_on_setup_neg_r3_e);
    MK2CPP_HandRegister(0x00002b62u, &step_note_on_setup_mulxu_r3_0x650e_r2_c);
    MK2CPP_HandRegister(0x00002b66u, &step_note_on_setup_cmp_r2_w_0x1f41_b);
    MK2CPP_HandRegister(0x00002b69u, &step_note_on_setup_bcs_5_0x2b70);
    MK2CPP_HandRegister(0x00002b6bu, &step_note_on_setup_movi_r3_0x0004_b);
    MK2CPP_HandRegister(0x00002b6eu, &step_note_on_setup_bra_47_0x2b9f);
    MK2CPP_HandRegister(0x00002b70u, &step_note_on_setup_movi_r3_0x1fc0_b);
    MK2CPP_HandRegister(0x00002b73u, &step_note_on_setup_sub_r2_r3_b);
    MK2CPP_HandRegister(0x00002b75u, &step_note_on_setup_movg2_r3_r2_b);
    MK2CPP_HandRegister(0x00002b77u, &step_note_on_setup_mulxu_0x0810_r2_r3_b);
    MK2CPP_HandRegister(0x00002b7bu, &step_note_on_setup_movg2_r2_r3_b);
    MK2CPP_HandRegister(0x00002b7du, &step_note_on_setup_bra_32_0x2b9f);
    MK2CPP_HandRegister(0x00002b7fu, &step_note_on_setup_movi_r4_0x1fc0_b);
    MK2CPP_HandRegister(0x00002b82u, &step_note_on_setup_mulxu_r3_0x650e_r2_d);
    MK2CPP_HandRegister(0x00002b86u, &step_note_on_setup_sub_r2_r4_b);
    MK2CPP_HandRegister(0x00002b88u, &step_note_on_setup_cmp_r4_w_0x0020_b);
    MK2CPP_HandRegister(0x00002b8bu, &step_note_on_setup_bcs_10_0x2b97);
    MK2CPP_HandRegister(0x00002b8du, &step_note_on_setup_movi_r2_0x001f_b);
    MK2CPP_HandRegister(0x00002b90u, &step_note_on_setup_movi_r3_0xc000_b);
    MK2CPP_HandRegister(0x00002b93u, &step_note_on_setup_divxu_r4_r2_r3_b);
    MK2CPP_HandRegister(0x00002b95u, &step_note_on_setup_bra_8_0x2b9f);
    MK2CPP_HandRegister(0x00002b97u, &step_note_on_setup_movi_r3_0xffff_b);
    MK2CPP_HandRegister(0x00002b9au, &step_note_on_setup_bra_3_0x2b9f);
    MK2CPP_HandRegister(0x00002b9cu, &step_note_on_setup_movi_r3_0x0100_b);
    MK2CPP_HandRegister(0x00002b9fu, &step_note_on_setup_movg3_r3_r0_m28);
    MK2CPP_HandRegister(0x00002ba2u, &step_note_on_setup_movg2_r5_78_r2);
    MK2CPP_HandRegister(0x00002ba5u, &step_note_on_setup_btsti_r2_7);
    MK2CPP_HandRegister(0x00002ba7u, &step_note_on_setup_beq_6_0x2baf);
    MK2CPP_HandRegister(0x00002ba9u, &step_note_on_setup_movg_0x00_r0_m8);
    MK2CPP_HandRegister(0x00002badu, &step_note_on_setup_bra_4_0x2bb3);
    MK2CPP_HandRegister(0x00002bafu, &step_note_on_setup_movg_0x04_r0_m8);
    MK2CPP_HandRegister(0x00002bb3u, &step_note_on_setup_and_0x007f_r2);
    MK2CPP_HandRegister(0x00002bb7u, &step_note_on_setup_movg3_r2_r0_79);
    MK2CPP_HandRegister(0x00002bbau, &step_note_on_setup_movg2_r5_79_r2);
    MK2CPP_HandRegister(0x00002bbdu, &step_note_on_setup_btsti_r2_7_b);
    MK2CPP_HandRegister(0x00002bbfu, &step_note_on_setup_beq_6_0x2bc7);
    MK2CPP_HandRegister(0x00002bc1u, &step_note_on_setup_movg_0x00_r0_m7);
    MK2CPP_HandRegister(0x00002bc5u, &step_note_on_setup_bra_4_0x2bcb);
    MK2CPP_HandRegister(0x00002bc7u, &step_note_on_setup_movg_0x04_r0_m7);
    MK2CPP_HandRegister(0x00002bcbu, &step_note_on_setup_and_0x007f_r2_b);
    MK2CPP_HandRegister(0x00002bcfu, &step_note_on_setup_movg3_r2_r0_80);
    MK2CPP_HandRegister(0x00002bd2u, &step_note_on_setup_movg2_r5_80_r2);
    MK2CPP_HandRegister(0x00002bd5u, &step_note_on_setup_btsti_r2_7_c);
    MK2CPP_HandRegister(0x00002bd7u, &step_note_on_setup_beq_6_0x2bdf);
    MK2CPP_HandRegister(0x00002bd9u, &step_note_on_setup_movg_0x00_r0_m6);
    MK2CPP_HandRegister(0x00002bddu, &step_note_on_setup_bra_4_0x2be3);
    MK2CPP_HandRegister(0x00002bdfu, &step_note_on_setup_movg_0x04_r0_m6);
    MK2CPP_HandRegister(0x00002be3u, &step_note_on_setup_and_0x007f_r2_c);
    MK2CPP_HandRegister(0x00002be7u, &step_note_on_setup_movg3_r2_r0_81);
    MK2CPP_HandRegister(0x00002beau, &step_note_on_setup_movg2_r5_81_r2);
    MK2CPP_HandRegister(0x00002bedu, &step_note_on_setup_btsti_r2_7_d);
    MK2CPP_HandRegister(0x00002befu, &step_note_on_setup_beq_6_0x2bf7);
    MK2CPP_HandRegister(0x00002bf1u, &step_note_on_setup_movg_0x00_r0_m5);
    MK2CPP_HandRegister(0x00002bf5u, &step_note_on_setup_bra_4_0x2bfb);
    MK2CPP_HandRegister(0x00002bf7u, &step_note_on_setup_movg_0x04_r0_m5);
    MK2CPP_HandRegister(0x00002bfbu, &step_note_on_setup_and_0x007f_r2_d);
    MK2CPP_HandRegister(0x00002bffu, &step_note_on_setup_movg3_r2_r0_82);
    MK2CPP_HandRegister(0x00002c02u, &step_note_on_setup_movg2_r5_82_r2);
    MK2CPP_HandRegister(0x00002c05u, &step_note_on_setup_btsti_r2_7_e);
    MK2CPP_HandRegister(0x00002c07u, &step_note_on_setup_beq_6_0x2c0f);
    MK2CPP_HandRegister(0x00002c09u, &step_note_on_setup_movg_0x00_r0_m4);
    MK2CPP_HandRegister(0x00002c0du, &step_note_on_setup_bra_4_0x2c13);
    MK2CPP_HandRegister(0x00002c0fu, &step_note_on_setup_movg_0x04_r0_m4);
    MK2CPP_HandRegister(0x00002c13u, &step_note_on_setup_and_0x007f_r2_e);
    MK2CPP_HandRegister(0x00002c17u, &step_note_on_setup_movg3_r2_r0_83);
    MK2CPP_HandRegister(0x00002c1au, &step_note_on_setup_ldc_r0_0x0098_r4_b);
    MK2CPP_HandRegister(0x00002c1eu, &step_note_on_setup_movg2_r0_0x009c_r5_b);
    MK2CPP_HandRegister(0x00002c22u, &step_note_on_setup_clr_r3_k);
    MK2CPP_HandRegister(0x00002c24u, &step_note_on_setup_movg2_r5_31_r3);
    MK2CPP_HandRegister(0x00002c27u, &step_note_on_setup_ldc_r0_0x0099_r4_d);
    MK2CPP_HandRegister(0x00002c2bu, &step_note_on_setup_movg2_r0_0x009e_r5_d);
    MK2CPP_HandRegister(0x00002c2fu, &step_note_on_setup_and_0x03_r3);
    MK2CPP_HandRegister(0x00002c32u, &step_note_on_setup_beq_27_0x2c4f);
    MK2CPP_HandRegister(0x00002c34u, &step_note_on_setup_clr_r2_g);
    MK2CPP_HandRegister(0x00002c36u, &step_note_on_setup_movg2_r1_0xd134_r2);
    MK2CPP_HandRegister(0x00002c3au, &step_note_on_setup_ldc_0x03_r5_d);
    MK2CPP_HandRegister(0x00002c3du, &step_note_on_setup_shll_r3);
    MK2CPP_HandRegister(0x00002c3fu, &step_note_on_setup_adds_r3_0xf80c_r2);
    MK2CPP_HandRegister(0x00002c43u, &step_note_on_setup_movg2_r2_r2);
    MK2CPP_HandRegister(0x00002c45u, &step_note_on_setup_ldc_0x00_r5_d);
    MK2CPP_HandRegister(0x00002c48u, &step_note_on_setup_extu_r2);
    MK2CPP_HandRegister(0x00002c4au, &step_note_on_setup_movg3_r2_r0_56);
    MK2CPP_HandRegister(0x00002c4du, &step_note_on_setup_bra_8_0x2c57);
    MK2CPP_HandRegister(0x00002c4fu, &step_note_on_setup_clr_r3_l);
    MK2CPP_HandRegister(0x00002c51u, &step_note_on_setup_movg2_r5_9_r3);
    MK2CPP_HandRegister(0x00002c54u, &step_note_on_setup_movg3_r3_r0_56);
    MK2CPP_HandRegister(0x00002c57u, &step_note_on_setup_movg2_r5_0_r3_b);
    MK2CPP_HandRegister(0x00002c5au, &step_note_on_setup_bpl_4_0x2c60);
    MK2CPP_HandRegister(0x00002c5cu, &step_note_on_setup_clr_r3_m);
    MK2CPP_HandRegister(0x00002c5eu, &step_note_on_setup_bra_35_0x2c83);
    MK2CPP_HandRegister(0x00002c60u, &step_note_on_setup_add_r3_r3_j);
    MK2CPP_HandRegister(0x00002c62u, &step_note_on_setup_movg2_r3_0x6c86_r4);
    MK2CPP_HandRegister(0x00002c66u, &step_note_on_setup_add_0x0004_r4);
    MK2CPP_HandRegister(0x00002c6au, &step_note_on_setup_shlr_r4_c);
    MK2CPP_HandRegister(0x00002c6cu, &step_note_on_setup_shlr_r4_d);
    MK2CPP_HandRegister(0x00002c6eu, &step_note_on_setup_shlr_r4_e);
    MK2CPP_HandRegister(0x00002c70u, &step_note_on_setup_bne_10_0x2c7c);
    MK2CPP_HandRegister(0x00002c72u, &step_note_on_setup_movg_0xffff_r0_14);
    MK2CPP_HandRegister(0x00002c77u, &step_note_on_setup_clr_r0_16);
    MK2CPP_HandRegister(0x00002c7au, &step_note_on_setup_bra_22_0x2c92);
    MK2CPP_HandRegister(0x00002c7cu, &step_note_on_setup_movi_r3_0xffff_c);
    MK2CPP_HandRegister(0x00002c7fu, &step_note_on_setup_clr_r2_h);
    MK2CPP_HandRegister(0x00002c81u, &step_note_on_setup_divxu_r4_r2_r3_c);
    MK2CPP_HandRegister(0x00002c83u, &step_note_on_setup_clr_r0_14);
    MK2CPP_HandRegister(0x00002c86u, &step_note_on_setup_movg3_r3_r0_16);
    MK2CPP_HandRegister(0x00002c89u, &step_note_on_setup_clr_r0_24);
    MK2CPP_HandRegister(0x00002c8cu, &step_note_on_setup_clr_r0_28);
    MK2CPP_HandRegister(0x00002c8fu, &step_note_on_setup_clr_r0_36);
    MK2CPP_HandRegister(0x00002c92u, &step_note_on_setup_clr_r0_18);
    MK2CPP_HandRegister(0x00002c95u, &step_note_on_setup_clr_r0_8);
    MK2CPP_HandRegister(0x00002c98u, &step_note_on_setup_clr_r0_24_b);
    MK2CPP_HandRegister(0x00002c9bu, &step_note_on_setup_clr_r0_28_b);
    MK2CPP_HandRegister(0x00002c9eu, &step_note_on_setup_movg2_dp_0xad2a_r6_b);
    MK2CPP_HandRegister(0x00002ca2u, &step_note_on_setup_movg3_r6_dp_0xad2c_b);
    MK2CPP_HandRegister(0x00002ca6u, &step_note_on_setup_movg_0x0001_dp_0xad2a_b);
    MK2CPP_HandRegister(0x00002cacu, &step_note_on_setup_movg2_r0_m2_r1_f);
    MK2CPP_HandRegister(0x00002cafu, &step_note_on_setup_bsr16_0x31ca);
    MK2CPP_HandRegister(0x00002cb2u, &step_note_on_setup_movg2_r0_30_r2);
    MK2CPP_HandRegister(0x00002cb5u, &step_note_on_setup_cmp_r2_b_0xaf);
    MK2CPP_HandRegister(0x00002cb7u, &step_note_on_setup_bne_5_0x2cbe);
    MK2CPP_HandRegister(0x00002cb9u, &step_note_on_setup_move_r2_0xba);
    MK2CPP_HandRegister(0x00002cbbu, &step_note_on_setup_movg3_r2_r0_30);
    MK2CPP_HandRegister(0x00002cbeu, &step_note_on_setup_movg2_dp_0xad2c_r6_b);
    MK2CPP_HandRegister(0x00002cc2u, &step_note_on_setup_movg3_r6_dp_0xad2a_b);
    MK2CPP_HandRegister(0x00002cc6u, &step_note_on_setup_movg2_r0_46_r2);
    MK2CPP_HandRegister(0x00002cc9u, &step_note_on_setup_movg2_r0_48_r3);
    MK2CPP_HandRegister(0x00002cccu, &step_note_on_setup_movg2_r2_14_r4);
    MK2CPP_HandRegister(0x00002ccfu, &step_note_on_setup_movg2_r2_15_r5);
    MK2CPP_HandRegister(0x00002cd2u, &step_note_on_setup_tst_r3);
    MK2CPP_HandRegister(0x00002cd4u, &step_note_on_setup_beq_24_0x2cee);
    MK2CPP_HandRegister(0x00002cd6u, &step_note_on_setup_mulxu_r3_0x0380_r4);
    MK2CPP_HandRegister(0x00002cdau, &step_note_on_setup_mulxu_r3_0x0300_r5);
    MK2CPP_HandRegister(0x00002cdeu, &step_note_on_setup_add_r4_r4_g);
    MK2CPP_HandRegister(0x00002ce0u, &step_note_on_setup_add_r5_r5);
    MK2CPP_HandRegister(0x00002ce2u, &step_note_on_setup_add_0x00ff_r4);
    MK2CPP_HandRegister(0x00002ce6u, &step_note_on_setup_add_0x00ff_r5);
    MK2CPP_HandRegister(0x00002ceau, &step_note_on_setup_swap_r4_h);
    MK2CPP_HandRegister(0x00002cecu, &step_note_on_setup_swap_r5);
    MK2CPP_HandRegister(0x00002ceeu, &step_note_on_setup_swap_r5_b);
    MK2CPP_HandRegister(0x00002cf0u, &step_note_on_setup_movg2_r4_r5_b);
    MK2CPP_HandRegister(0x00002cf2u, &step_note_on_setup_movg3_r5_r0_58);
    MK2CPP_HandRegister(0x00002cf5u, &step_note_on_setup_movg2_r2_9_r2);
    MK2CPP_HandRegister(0x00002cf8u, &step_note_on_setup_beq_81_0x2d4b);
    MK2CPP_HandRegister(0x00002cfau, &step_note_on_setup_tst_r3_b);
    MK2CPP_HandRegister(0x00002cfcu, &step_note_on_setup_beq_25_0x2d17);
    MK2CPP_HandRegister(0x00002cfeu, &step_note_on_setup_movg2_r3_0x0280_r5);
    MK2CPP_HandRegister(0x00002d02u, &step_note_on_setup_beq_71_0x2d4b);
    MK2CPP_HandRegister(0x00002d04u, &step_note_on_setup_sub_0x40_r5);
    MK2CPP_HandRegister(0x00002d07u, &step_note_on_setup_bcc_8_0x2d11);
    MK2CPP_HandRegister(0x00002d09u, &step_note_on_setup_add_r5_r2);
    MK2CPP_HandRegister(0x00002d0bu, &step_note_on_setup_bpl_10_0x2d17);
    MK2CPP_HandRegister(0x00002d0du, &step_note_on_setup_clr_r2_i);
    MK2CPP_HandRegister(0x00002d0fu, &step_note_on_setup_bra_6_0x2d17);
    MK2CPP_HandRegister(0x00002d11u, &step_note_on_setup_add_r5_r2_b);
    MK2CPP_HandRegister(0x00002d13u, &step_note_on_setup_bvc_2_0x2d17);
    MK2CPP_HandRegister(0x00002d15u, &step_note_on_setup_move_r2_0x7f);
    MK2CPP_HandRegister(0x00002d17u, &step_note_on_setup_movg2_r0_56_r3);
    MK2CPP_HandRegister(0x00002d1au, &step_note_on_setup_sub_0x40_r2);
    MK2CPP_HandRegister(0x00002d1du, &step_note_on_setup_bcc_8_0x2d27);
    MK2CPP_HandRegister(0x00002d1fu, &step_note_on_setup_add_r2_r3);
    MK2CPP_HandRegister(0x00002d21u, &step_note_on_setup_bpl_10_0x2d2d);
    MK2CPP_HandRegister(0x00002d23u, &step_note_on_setup_clr_r3_n);
    MK2CPP_HandRegister(0x00002d25u, &step_note_on_setup_bra_6_0x2d2d);
    MK2CPP_HandRegister(0x00002d27u, &step_note_on_setup_add_r2_r3_b);
    MK2CPP_HandRegister(0x00002d29u, &step_note_on_setup_bvc_2_0x2d2d);
    MK2CPP_HandRegister(0x00002d2bu, &step_note_on_setup_move_r3_0x7f);
    MK2CPP_HandRegister(0x00002d2du, &step_note_on_setup_movg2_dp_0x8006_r2);
    MK2CPP_HandRegister(0x00002d31u, &step_note_on_setup_beq_19_0x2d46);
    MK2CPP_HandRegister(0x00002d33u, &step_note_on_setup_sub_0x40_r2_b);
    MK2CPP_HandRegister(0x00002d36u, &step_note_on_setup_bcc_8_0x2d40);
    MK2CPP_HandRegister(0x00002d38u, &step_note_on_setup_add_r2_r3_c);
    MK2CPP_HandRegister(0x00002d3au, &step_note_on_setup_bpl_10_0x2d46);
    MK2CPP_HandRegister(0x00002d3cu, &step_note_on_setup_clr_r3_o);
    MK2CPP_HandRegister(0x00002d3eu, &step_note_on_setup_bra_6_0x2d46);
    MK2CPP_HandRegister(0x00002d40u, &step_note_on_setup_add_r2_r3_d);
    MK2CPP_HandRegister(0x00002d42u, &step_note_on_setup_bvc_2_0x2d46);
    MK2CPP_HandRegister(0x00002d44u, &step_note_on_setup_move_r3_0x7f_b);
    MK2CPP_HandRegister(0x00002d46u, &step_note_on_setup_movg3_r3_r0_54);
    MK2CPP_HandRegister(0x00002d49u, &step_note_on_setup_bra_27_0x2d66);
    MK2CPP_HandRegister(0x00002d4bu, &step_note_on_setup_bset_orc_0x0700_r0_c);
    MK2CPP_HandRegister(0x00002d4fu, &step_note_on_setup_movg_0x1e_br_3e_b);
    MK2CPP_HandRegister(0x00002d53u, &step_note_on_setup_movl_r3_br_34);
    MK2CPP_HandRegister(0x00002d55u, &step_note_on_setup_movlw_r3_br_3a);
    MK2CPP_HandRegister(0x00002d57u, &step_note_on_setup_bclr_andc_0xf8ff_r0_c);
    MK2CPP_HandRegister(0x00002d5bu, &step_note_on_setup_clr_r3_p);
    MK2CPP_HandRegister(0x00002d5du, &step_note_on_setup_swap_r3_b);
    MK2CPP_HandRegister(0x00002d5fu, &step_note_on_setup_shlr_r3);
    MK2CPP_HandRegister(0x00002d61u, &step_note_on_setup_movg_0xffff_r0_54);
    MK2CPP_HandRegister(0x00002d66u, &step_note_on_setup_movg2_r3_0x6a03_r4);
    MK2CPP_HandRegister(0x00002d6au, &step_note_on_setup_movi_r2_0x0080);
    MK2CPP_HandRegister(0x00002d6du, &step_note_on_setup_sub_r3_r2);
    MK2CPP_HandRegister(0x00002d6fu, &step_note_on_setup_movg2_r2_0x6a03_r5);
    MK2CPP_HandRegister(0x00002d73u, &step_note_on_setup_swap_r5_c);
    MK2CPP_HandRegister(0x00002d75u, &step_note_on_setup_movg2_r4_r5_c);
    MK2CPP_HandRegister(0x00002d77u, &step_note_on_setup_movg3_r5_r0_52);
    MK2CPP_HandRegister(0x00002d7au, &step_note_on_setup_movg2_r0_m2_r1_g);
    MK2CPP_HandRegister(0x00002d7du, &step_note_on_setup_bsr_22_0x2d95);
    MK2CPP_HandRegister(0x00002d7fu, &step_note_on_setup_movg3_r5_r0_24);
    MK2CPP_HandRegister(0x00002d82u, &step_note_on_setup_move_r5_0xba);
    MK2CPP_HandRegister(0x00002d84u, &step_note_on_setup_movg3_r5_r0_26);
    MK2CPP_HandRegister(0x00002d87u, &step_note_on_setup_movg2_r0_m2_r1_h);
    MK2CPP_HandRegister(0x00002d8au, &step_note_on_setup_sub_r1_0xd0e0_0x00_b);
    MK2CPP_HandRegister(0x00002d8fu, &step_note_on_setup_bne_0x2700_0x5492);
    MK2CPP_HandRegister(0x00002d92u, &step_note_on_setup_bra_0x0e1f_0x3bb4);
#endif
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct NoteOnSetupSelfRegister
{
    NoteOnSetupSelfRegister() { hand_register_module(&note_on_setup_fill); }
};

NoteOnSetupSelfRegister g_note_on_setup_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
