#ifndef MK2CPP_HAND_PRIMS_H
#define MK2CPP_HAND_PRIMS_H

/*
 * HAND shared H8 primitives (split out of shared_misc.cpp, 2026-09-12).
 *
 * Small, context-free helpers used by the hand modules split from the former
 * shared_misc.cpp: effective-address builders, flag/move/arith shims wrapping
 * the GT core (MCU_Read/Read16/Write/Write16, MCU_Add/Sub/SetStatus*, device
 * routing), and the stock-instruction safety net. Each mirrors exactly the
 * generated body semantics (mk2cpp/src/gen), including which flags an
 * instruction leaves untouched; there is no private state and no caching, so
 * every helper is a pure function of the architectural register/SRAM state.
 *
 * All functions are inline; include this header from split hand modules only.
 * The identifier namespace is mk2c::hand_prim to stay clear of the local
 * anonymous-namespace helpers of the other hand modules.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"
#include "mcu_interrupt.h"

/* Defined in src/mcu_opcodes.cpp; declared locally like the other hand
 * modules (not exported through a header). Used by the split hand modules
 * and by the primitives below. */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace hand_prim {
/* ---- effective addresses -------------------------------------------------- */

/* Page register for rN: dp (r0-r3), ep (r4/r5), tp (r6/r7). */
inline uint32_t page_of_reg(uint32_t reg)
{
    return (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
}

/* (dp,disp16): absolute through the DP page register. */
inline uint32_t dp_addr(uint16_t disp)
{
    return ((uint32_t)mcu.dp << 16) | disp;
}

/* @rN: page of rN, offset rN. */
inline uint32_t reg_addr(uint32_t reg)
{
    return ((uint32_t)page_of_reg(reg) << 16) | mcu.r[reg];
}

/* @rN+disp: the rN+disp sum wraps inside 16 bits. */
inline uint32_t ind_addr(uint32_t reg, int disp)
{
    return ((uint32_t)page_of_reg(reg) << 16) |
           (uint16_t)(mcu.r[reg] + (uint16_t)(int16_t)disp);
}

/* ---- status --------------------------------------------------------------- */

inline void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* TST (byte): N/Z from the low byte, C=0, V untouched. */
inline void tst8(uint32_t data)
{
    MCU_SetStatusCommon(data, 0);
    MCU_SetStatus(0, STATUS_C);
}

/* TST (word): N/Z from the word, C=0, V untouched. */
inline void tst16(uint16_t value)
{
    MCU_SetStatusCommon((uint32_t)value, 1);
    MCU_SetStatus(0, STATUS_C);
}

/* ---- byte/word moves (MOVG2/MOVG3/MOVI) ----------------------------------- */

/* MOVG2 @addr -> rN (byte): low byte replaced, flags from the byte. */
inline void load8(uint16_t &reg, uint32_t addr)
{
    uint32_t data = (uint32_t)MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* MOVG2 @addr -> rN (word): odd-address check first, then flags from data. */
inline void load16(uint16_t &reg, uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(addr);
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVG3 rN -> @addr (byte). */
inline void store8(uint32_t addr, uint16_t value)
{
    MCU_Write(addr, (uint8_t)value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* MOVG3 rN -> @addr (word): odd-address check first, then write + flags. */
inline void store16(uint32_t addr, uint16_t value)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 1);
}

/* MOVG2 rS -> rD (byte): low byte copied; flags from the copied byte. */
inline void mov8(uint16_t &dst, uint32_t data)
{
    dst = (uint16_t)((dst & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* MOVG2 rS -> rD (word): full 16-bit copy; flags from the copied word. */
inline void mov16(uint16_t &dst, uint32_t data)
{
    dst = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVI #imm -> rN */
inline void movi8(uint16_t &reg, uint8_t imm)
{
    reg = imm;
    MCU_SetStatusCommon((uint32_t)imm, 0);
}
inline void movi16(uint16_t &reg, uint16_t imm)
{
    reg = imm;
    MCU_SetStatusCommon((uint32_t)imm, 1);
}

/* MOVG #imm16 -> (dp,disp) (byte): only the low byte is written. */
inline void store8_imm_dp(uint16_t disp, uint16_t imm)
{
    MCU_Write(dp_addr(disp), (uint8_t)imm);
    MCU_SetStatusCommon((uint32_t)imm, 0);
}

/* LDC #imm rN: control register write + ex_ignore (gen does the same). */
inline void ldc_cr(uint32_t reg, uint8_t imm)
{
    MCU_ControlRegisterWrite(reg, 0, imm);
    mcu.ex_ignore = 1;
}

/* ---- arithmetic ----------------------------------------------------------- */

/* MULXU rS rH:rL (word): 32-bit product into H:L, flags from the product. */
inline void mulxu(uint16_t &hi, uint16_t &lo, uint16_t a, uint16_t b)
{
    uint32_t p = (uint32_t)a * (uint32_t)b;
    hi = (uint16_t)(p >> 16);
    lo = (uint16_t)p;
    MCU_SetStatus((p & 0x80000000u) != 0, STATUS_N);
    MCU_SetStatus(p == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* CMP.B #imm, rN: subtract-only, flags only. */
inline void cmp8_imm(uint16_t reg, uint8_t imm)
{
    MCU_SUB_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 0);
}

/* CMP.W #imm, rN: subtract-only, flags only. */
inline void cmp16_imm(uint16_t reg, uint16_t imm)
{
    MCU_SUB_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 1);
}

/* NEG rN (word). */
inline void neg16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    reg = (uint16_t)data;
}

/* ---- control flow --------------------------------------------------------- */

/* cntjmp rN disp8: rN--; branch to taken while rN != 0xffff. */
inline void cntjmp(uint16_t &reg, uint16_t taken, uint16_t fall)
{
    reg = (uint16_t)(reg - 1);
    mcu.pc = (reg != 0xffff) ? taken : fall;
}

/* bsr/jsr: push the return PC, then jump. */
inline void call(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

/* PUSH.W rN as gen emits it: r7 -= 2 first, then the odd check.  (MCU_PushStack
 * checks oddness before decrementing, so it cannot be used verbatim here.) */
inline void pushw(uint16_t value)
{
    mcu.r[7] -= 2;
    uint32_t oea = (uint32_t)mcu.r[7] & 0xffffu;
    uint32_t oep = (uint32_t)(page_of_reg(7) & 0xff);
    if (oea & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), value);
    MCU_SetStatusCommon((uint32_t)value, 1);
}

/* POP.W rN as gen emits it: r7 += 2 first, then the odd check. */
inline void popw(uint16_t &reg)
{
    uint32_t oea = (uint32_t)mcu.r[7] & 0xffffu;
    mcu.r[7] += 2;
    uint32_t oep = (uint32_t)(page_of_reg(7) & 0xff);
    if (oea & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* Safety net: a PC missing from the case tables is executed by the stock
 * interpreter for that one instruction (should never happen). */
inline void stock_instruction(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
}

} /* namespace hand_prim */
} /* namespace mk2c */

#endif /* MK2CPP_HAND_PRIMS_H */
