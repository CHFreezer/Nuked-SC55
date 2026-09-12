/*
 * HAND shared_misc -- remaining shared leaf routines: the cp0 dsp/rate helper
 * 0x50EF..0x516B and the reset_init fragment tails in cp4
 * (0x433C4, 0x433D1, 0x433EA, 0x4342A, 0x434CB, 0x43641) plus the shared
 * 0x4142F nop;rts stub.  One hand entry per H8 instruction (L0: every entry
 * executes exactly one instruction and returns 1, so the host keeps its
 * per-instruction interrupt poll / TIMER_Clock / midiseq+SM cadence / trace /
 * PCM catch-up).
 *
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * Scope and boundaries (all evidence: raw ROM bytes + linear decode in
 * tools/baselines/dasm_full.txt + the per-PC generated reference bodies in
 * mk2cpp/src/gen/mk2c_r1.cpp / mk2c_r2.cpp; the current hand partition files
 * were grepped for disjointness before registering):
 *
 *   cp0 helper  0x50EF-0x516B  54 PC  (rts at 0x516B)
 *     Callers: bsr16 at 0x4E9B/0x4EA8 inside the dsp_rate family (hand module
 *     dsp_rate.cpp, which stops at its 0x50EE rts).  flow_main also lists
 *     0x0508 (rte), 0x7C3F (ret) and 0x7DC3 (rte) as sources reaching 0x50EF:
 *     those are interrupt/return resume edges, not calls -- the PC can be an
 *     exception return address, so the helper must stay per-instruction
 *     correct for every caller/resume context, which it is: semantics are a
 *     pure function of registers/SRAM.  Static call scan of rom1 finds only
 *     the two bsr16 sites.
 *     Includes the two gen-registered overlap decodes 0x5106/0x5107
 *     (jmp #0xAA14 / NEG r2), transcribed from their gen bodies.
 *
 *   cp4 0x433C4-0x433D0   3 PC  sets dp:0xd202/dp:0xd204 = 0xd1de; rts
 *     Callers (jsr abs16 scan + flow): 0x432D5 (A5), 0x3F8E, 0x3FAC, 0x3FC0,
 *     0x495F, 0x67A2, 0x67D4, 0xB610, 0xB98D (rom2 offsets) -- routine is
 *     leaf and caller-independent.
 *
 *   cp4 0x433D1-0x433E9  10 PC  bit merge on dp:0xd1cd/dp:0xd1cf; rts
 *     Caller: bsr at 0x4338C (A5).  No dynamic hit in the recorded boot/demo
 *     windows (C bytes / I dynamic).
 *
 *   cp4 0x433EA-0x43429  26 PC  dp:0xd1cb bit2/1/3 gate + dp:0xd1d5
 *     0x1f-entry decrement loop; calls 0x3532 with r0/r1 saved; rts at 0x43429
 *     Caller: bsr8 at 0x433B7 (A5), the only call site in the 15999-PC h8reach
 *     static closure.  The fall-through PCs 0x433F8..0x43418 are statically
 *     reachable from 0x433F6 but absent from the exec-only h8part map (never
 *     executed in the recorded windows); the fragment can also be an
 *     exception-resume PC (flow_main rte/ret resume edges 0x3BA/0x508/0x7C3F/
 *     0x7DC3), so it stays L0 per instruction like the other tails.
 *
 *   cp4 0x4342A-0x434CA  65 PC  d1ff/d421 voice-on/off gate; rts at 0x434CA
 *     Caller: jsr at 0x433BF (A5).  Internal arms at 0x343B/0x347B, single
 *     return via `jmp #0x34CA`.  Calls 0x43532/0x43574 (other owners).
 *
 *   cp4 0x434CB-0x43531  41 PC  d1d6..d1da table walk; rts at 0x43531
 *     Caller: jsr at 0x433BC (A5).  Internal calls 0x43574/0x43532, movm
 *     push/pop, movi/bsr/cntjmp.
 *
 *   cp4 0x43641-0x43694  35 PC  d1cc bit-0 scan / d1da write; rts at 0x43694
 *     Caller: jsr at 0x433B9 (A5).  Calls 0x43532.
 *
 *   cp4 0x4142F-0x41430   2 PC  shared `nop; rts` stub
 *     Callers: bsr8 at 0x413D5/0x413D7/0x413FE/0x41400/0x41416/0x41418 (A4
 *     pcm_chan_init, reset_init.cpp pushes those returns itself).
 *
 * Confidence: instruction-level C (ROM bytes + GT emitter bodies); routine
 * role names are inferred (I) and do not affect the transcription.  No case
 * depends on caller context beyond the architectural register/SRAM state, so
 * every PC is equivalent under all callers and none is left on gen.
 *
 * Data model: state stays in page-0 SRAM / device registers and goes through
 * MCU_Read/Read16/Write/Write16 so device routing and the odd-address
 * ADDRESS_ERROR trap order stay identical to the generated code.  Flags mirror
 * the gen body of each PC exactly (MCU_ADD_Common / MCU_SUB_Common /
 * MCU_SetStatusCommon / MCU_SetStatus), including which flags an instruction
 * leaves untouched (TST does not write V; ADDX keeps Z sticky by replaying the
 * old Z; SUBX leaves Z alone; CLR/EXTU write all four).
 *
 * Registration: per-PC MK2CPP_HandRegisterRoutine from a file-static
 * initializer through mk2c::hand_register_module (hand_registry.h); no shared
 * aggregator file is edited.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"
#include "mcu_interrupt.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; declared locally like the other hand
 * modules (not exported through a header). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* ---- effective addresses -------------------------------------------------- */

/* Page register for rN: dp (r0-r3), ep (r4/r5), tp (r6/r7). */
uint32_t page_of_reg(uint32_t reg)
{
    return (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
}

/* (dp,disp16): absolute through the DP page register. */
uint32_t dp_addr(uint16_t disp)
{
    return ((uint32_t)mcu.dp << 16) | disp;
}

/* @rN: page of rN, offset rN. */
uint32_t reg_addr(uint32_t reg)
{
    return ((uint32_t)page_of_reg(reg) << 16) | mcu.r[reg];
}

/* @rN+disp: the rN+disp sum wraps inside 16 bits. */
uint32_t ind_addr(uint32_t reg, int disp)
{
    return ((uint32_t)page_of_reg(reg) << 16) |
           (uint16_t)(mcu.r[reg] + (uint16_t)(int16_t)disp);
}

/* ---- status --------------------------------------------------------------- */

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* TST (byte): N/Z from the low byte, C=0, V untouched. */
void tst8(uint32_t data)
{
    MCU_SetStatusCommon(data, 0);
    MCU_SetStatus(0, STATUS_C);
}

/* TST (word): N/Z from the word, C=0, V untouched. */
void tst16(uint16_t value)
{
    MCU_SetStatusCommon((uint32_t)value, 1);
    MCU_SetStatus(0, STATUS_C);
}

/* ---- byte/word moves (MOVG2/MOVG3/MOVI) ----------------------------------- */

/* MOVG2 @addr -> rN (byte): low byte replaced, flags from the byte. */
void load8(uint16_t &reg, uint32_t addr)
{
    uint32_t data = (uint32_t)MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* MOVG2 @addr -> rN (word): odd-address check first, then flags from data. */
void load16(uint16_t &reg, uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(addr);
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVG3 rN -> @addr (byte). */
void store8(uint32_t addr, uint16_t value)
{
    MCU_Write(addr, (uint8_t)value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* MOVG3 rN -> @addr (word): odd-address check first, then write + flags. */
void store16(uint32_t addr, uint16_t value)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 1);
}

/* MOVG2 rS -> rD (byte): low byte copied; flags from the copied byte. */
void mov8(uint16_t &dst, uint32_t data)
{
    dst = (uint16_t)((dst & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* MOVG2 rS -> rD (word): full 16-bit copy; flags from the copied word. */
void mov16(uint16_t &dst, uint32_t data)
{
    dst = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVI #imm -> rN */
void movi8(uint16_t &reg, uint8_t imm)
{
    reg = imm;
    MCU_SetStatusCommon((uint32_t)imm, 0);
}
void movi16(uint16_t &reg, uint16_t imm)
{
    reg = imm;
    MCU_SetStatusCommon((uint32_t)imm, 1);
}

/* MOVG #imm16 -> (dp,disp) (byte): only the low byte is written. */
void store8_imm_dp(uint16_t disp, uint16_t imm)
{
    MCU_Write(dp_addr(disp), (uint8_t)imm);
    MCU_SetStatusCommon((uint32_t)imm, 0);
}

/* LDC #imm rN: control register write + ex_ignore (gen does the same). */
void ldc_cr(uint32_t reg, uint8_t imm)
{
    MCU_ControlRegisterWrite(reg, 0, imm);
    mcu.ex_ignore = 1;
}

/* ---- arithmetic ----------------------------------------------------------- */

/* MULXU rS rH:rL (word): 32-bit product into H:L, flags from the product. */
void mulxu(uint16_t &hi, uint16_t &lo, uint16_t a, uint16_t b)
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
void cmp8_imm(uint16_t reg, uint8_t imm)
{
    MCU_SUB_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 0);
}

/* CMP.W #imm, rN: subtract-only, flags only. */
void cmp16_imm(uint16_t reg, uint16_t imm)
{
    MCU_SUB_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 1);
}

/* NEG rN (word). */
void neg16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    reg = (uint16_t)data;
}

/* ---- control flow --------------------------------------------------------- */

/* cntjmp rN disp8: rN--; branch to taken while rN != 0xffff. */
void cntjmp(uint16_t &reg, uint16_t taken, uint16_t fall)
{
    reg = (uint16_t)(reg - 1);
    mcu.pc = (reg != 0xffff) ? taken : fall;
}

/* bsr/jsr: push the return PC, then jump. */
void call(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

/* PUSH.W rN as gen emits it: r7 -= 2 first, then the odd check.  (MCU_PushStack
 * checks oddness before decrementing, so it cannot be used verbatim here.) */
void pushw(uint16_t value)
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
void popw(uint16_t &reg)
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
void stock_instruction(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
}

/* ======================================================================
 * cp0 shared helper 0x50EF..0x516B (rts at 0x516B), 54 PC.
 * Called by bsr16 from dsp_rate 0x4E9B/0x4EA8; may also be an exception
 * return PC (rte 0x7DC3 / ret 0x7C3F resume edges in flow_main).
 * Takes r0 = voice base, scales the value in r2 by the 0x78EE/0x7AEE table
 * at r1, accumulates into r6 with 16-bit carry folding in r5 (the exact
 * arithmetic is transcribed per instruction below).
 * ====================================================================== */

uint32_t step_shared_helper(void)
{
    switch (mcu.pc)
    {
    case 0x50ef: /* TST r2 [aa 16] */
        tst16(mcu.r[2]);
        mcu.pc = 0x50f1;
        return 1;
    case 0x50f1: /* BMI +0x10 -> 0x5103 [2b 10] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5103 : 0x50f3;
        return 1;
    case 0x50f3: /* TST r3 [ab 16] */
        tst16(mcu.r[3]);
        mcu.pc = 0x50f5;
        return 1;
    case 0x50f5: /* BMI +0x20 -> 0x5117 [2b 20] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5117 : 0x50f7;
        return 1;
    case 0x50f7: /* ADD r3 r2 [ab 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            (int32_t)(uint32_t)mcu.r[3], 0, 1);
        mcu.pc = 0x50f9;
        return 1;
    case 0x50f9: /* CMP.W #0x1770 r2 [4a 17 70] */
        cmp16_imm(mcu.r[2], 0x1770);
        mcu.pc = 0x50fc;
        return 1;
    case 0x50fc: /* BLS +0x21 -> 0x511f [23 21] */
    {
        uint32_t branch = ((mcu.sr & (STATUS_C | STATUS_Z)) != 0);
        mcu.pc = branch ? 0x511f : 0x50fe;
        return 1;
    }
    case 0x50fe: /* MOVI r2 #0x1770 [5a 17 70] */
        movi16(mcu.r[2], 0x1770);
        mcu.pc = 0x5101;
        return 1;
    case 0x5101: /* BRA +0x1c -> 0x511f [20 1c] */
        mcu.pc = 0x511f;
        return 1;
    case 0x5103: /* TST r3 [ab 16] */
        tst16(mcu.r[3]);
        mcu.pc = 0x5105;
        return 1;
    case 0x5105: /* BPL +0x10 -> 0x5117 [2a 10] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5107 : 0x5117;
        return 1;
    case 0x5106: /* JMP #0xAA14 [10 aa 14] (overlap decode) */
        mcu.pc = 0xaa14;
        return 1;
    case 0x5107: /* NEG r2 [aa 14] (overlap decode) */
        neg16(mcu.r[2]);
        mcu.pc = 0x5109;
        return 1;
    case 0x5109: /* NEG r3 [ab 14] */
        neg16(mcu.r[3]);
        mcu.pc = 0x510b;
        return 1;
    case 0x510b: /* ADD r3 r2 [ab 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            (int32_t)(uint32_t)mcu.r[3], 0, 1);
        mcu.pc = 0x510d;
        return 1;
    case 0x510d: /* CMP.W #0x1770 r2 [4a 17 70] */
        cmp16_imm(mcu.r[2], 0x1770);
        mcu.pc = 0x5110;
        return 1;
    case 0x5110: /* BLS +0x15 -> 0x5127 [23 15] */
    {
        uint32_t branch = ((mcu.sr & (STATUS_C | STATUS_Z)) != 0);
        mcu.pc = branch ? 0x5127 : 0x5112;
        return 1;
    }
    case 0x5112: /* MOVI r2 #0x1770 [5a 17 70] */
        movi16(mcu.r[2], 0x1770);
        mcu.pc = 0x5115;
        return 1;
    case 0x5115: /* BRA +0x10 -> 0x5127 [20 10] */
        mcu.pc = 0x5127;
        return 1;
    case 0x5117: /* ADD r3 r2 [ab 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            (int32_t)(uint32_t)mcu.r[3], 0, 1);
        mcu.pc = 0x5119;
        return 1;
    case 0x5119: /* BPL +4 -> 0x511f [2a 04] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x511b : 0x511f;
        return 1;
    case 0x511b: /* NEG r2 [aa 14] */
        neg16(mcu.r[2]);
        mcu.pc = 0x511d;
        return 1;
    case 0x511d: /* BRA +8 -> 0x5127 [20 08] */
        mcu.pc = 0x5127;
        return 1;
    case 0x511f: /* TST r6 [ae 16] */
        tst16(mcu.r[6]);
        mcu.pc = 0x5121;
        return 1;
    case 0x5121: /* BPL +0x0a -> 0x512d [2a 0a] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5123 : 0x512d;
        return 1;
    case 0x5123: /* NEG r6 [ae 14] */
        neg16(mcu.r[6]);
        mcu.pc = 0x5125;
        return 1;
    case 0x5125: /* BRA +0x1f -> 0x5146 [20 1f] */
        mcu.pc = 0x5146;
        return 1;
    case 0x5127: /* TST r6 [ae 16] */
        tst16(mcu.r[6]);
        mcu.pc = 0x5129;
        return 1;
    case 0x5129: /* BPL +0x1b -> 0x5146 [2a 1b] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x512b : 0x5146;
        return 1;
    case 0x512b: /* NEG r6 [ae 14] */
        neg16(mcu.r[6]);
        mcu.pc = 0x512d;
        return 1;
    case 0x512d: /* ADD r6 r6 [ae 26] */
        mcu.r[6] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[6],
                                            (int32_t)(uint32_t)mcu.r[6], 0, 1);
        mcu.pc = 0x512f;
        return 1;
    case 0x512f: /* MULXU r6 r2:r3 [ae aa] */
        mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
        mcu.pc = 0x5131;
        return 1;
    case 0x5131: /* ADD.W #0x8000 r3 [0c 80 00 23] */
        mcu.r[3] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[3],
                                            0x8000, 0, 1);
        mcu.pc = 0x5135;
        return 1;
    case 0x5135: /* ADDX #0x0000 r2 [0c 00 00 a2] */
    {
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2], 0, C, 1);
        if (!Z)
            MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
        mcu.pc = 0x5139;
        return 1;
    }
    case 0x5139: /* MOVG2 @r0+45 r5 (byte) [e0 2d 85] */
        load8(mcu.r[5], ind_addr(0, 45));
        mcu.pc = 0x513c;
        return 1;
    case 0x513c: /* MOVG2 @r0+70 r6 (word) [e8 46 86] */
        load16(mcu.r[6], ind_addr(0, 70));
        mcu.pc = 0x513f;
        return 1;
    case 0x513f: /* ADD r2 r6 [aa 26] */
        mcu.r[6] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[6],
                                            (int32_t)(uint32_t)mcu.r[2], 0, 1);
        mcu.pc = 0x5141;
        return 1;
    case 0x5141: /* ADDX #0x00 r5 [04 00 a5] */
    {
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)mcu.r[5], 0, C, 0);
        if (!Z)
            MCU_SetStatus(0, STATUS_Z);
        mcu.r[5] = (uint16_t)((mcu.r[5] & 0xff00u) | ((uint32_t)t1 & 0xffu));
        mcu.pc = 0x5144;
        return 1;
    }
    case 0x5144: /* BRA +0x1f -> 0x5165 [20 1f] */
        mcu.pc = 0x5165;
        return 1;
    case 0x5146: /* ADD r6 r6 [ae 26] */
        mcu.r[6] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[6],
                                            (int32_t)(uint32_t)mcu.r[6], 0, 1);
        mcu.pc = 0x5148;
        return 1;
    case 0x5148: /* MULXU r6 r2:r3 [ae aa] */
        mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
        mcu.pc = 0x514a;
        return 1;
    case 0x514a: /* ADD.W #0x8000 r3 [0c 80 00 23] */
        mcu.r[3] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[3],
                                            0x8000, 0, 1);
        mcu.pc = 0x514e;
        return 1;
    case 0x514e: /* ADDX #0x0000 r2 [0c 00 00 a2] */
    {
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2], 0, C, 1);
        if (!Z)
            MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
        mcu.pc = 0x5152;
        return 1;
    }
    case 0x5152: /* MOVG2 @r0+45 r5 (byte) [e0 2d 85] */
        load8(mcu.r[5], ind_addr(0, 45));
        mcu.pc = 0x5155;
        return 1;
    case 0x5155: /* MOVG2 @r0+70 r6 (word) [e8 46 86] */
        load16(mcu.r[6], ind_addr(0, 70));
        mcu.pc = 0x5158;
        return 1;
    case 0x5158: /* SUB r2 r6 [aa 36] */
        mcu.r[6] = (uint16_t)MCU_SUB_Common((int32_t)(uint32_t)mcu.r[6],
                                            (int32_t)(uint32_t)mcu.r[2], 0, 1);
        mcu.pc = 0x515a;
        return 1;
    case 0x515a: /* SUBX #0x00 r5 [04 00 b5] */
    {
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)mcu.r[5], 0, C, 0);
        mcu.r[5] = (uint16_t)((mcu.r[5] & 0xff00u) | ((uint32_t)t1 & 0xffu));
        mcu.pc = 0x515d;
        return 1;
    }
    case 0x515d: /* TST r5 (byte) [a5 16] */
        tst8((uint32_t)mcu.r[5]);
        mcu.pc = 0x515f;
        return 1;
    case 0x515f: /* BPL +4 -> 0x5165 [2a 04] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5161 : 0x5165;
        return 1;
    case 0x5161: /* CLR r6 [ae 13] */
        mcu.r[6] = 0;
        flags_clr();
        mcu.pc = 0x5163;
        return 1;
    case 0x5163: /* CLR r5 (byte) [a5 13] */
        mcu.r[5] = (uint16_t)(mcu.r[5] & 0xff00u);
        flags_clr();
        mcu.pc = 0x5165;
        return 1;
    case 0x5165: /* MOVG3 r5 -> @r0+45 (byte) [e0 2d 95] */
        store8(ind_addr(0, 45), mcu.r[5]);
        mcu.pc = 0x5168;
        return 1;
    case 0x5168: /* MOVG3 r6 -> @r0+70 (word) [e8 46 96] */
        store16(ind_addr(0, 70), mcu.r[6]);
        mcu.pc = 0x516b;
        return 1;
    case 0x516b: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ======================================================================
 * cp4 0x433C4..0x433D0, 3 PC: write 0xd1de to dp:0xd202 and dp:0xd204.
 * Entry jsr; callers 0x432D5/0x3F8E/0x3FAC/0x3FC0/0x495F/0x67A2/0x67D4/
 * 0xB610/0xB98D (rom2 offsets); leaf, no context dependency.
 * ====================================================================== */

uint32_t step_r433c4(void)
{
    switch (mcu.pc)
    {
    case 0x33c4: /* MOVG #0xd1de -> (dp,0xd202) (word) [1d d2 02 07 d1 de] */
        mcu.pc = 0x33ca;
        store16(dp_addr(0xd202), 0xd1de);
        return 1;
    case 0x33ca: /* MOVG #0xd1de -> (dp,0xd204) (word) [1d d2 04 07 d1 de] */
        mcu.pc = 0x33d0;
        store16(dp_addr(0xd204), 0xd1de);
        return 1;
    case 0x33d0: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ======================================================================
 * cp4 0x433D1..0x433E9, 10 PC: read d1cf, merge bits with d1cd:
 * d1cd = (d1cd & (d1cf | ~d1cf)) ... exact per-instruction transcription
 * below (AND/OR/NOT/XOR on low bytes).  Caller bsr at 0x4338C (A5).
 * C bytes / I dynamic (not executed in the recorded boot/demo windows).
 * ====================================================================== */

uint32_t step_r433d1(void)
{
    switch (mcu.pc)
    {
    case 0x33d1: /* MOVG2 (dp,0xd1cf) r0 (byte) [15 d1 cf 80] */
        load8(mcu.r[0], dp_addr(0xd1cf));
        mcu.pc = 0x33d5;
        return 1;
    case 0x33d5: /* MOVG2 (dp,0xd1cd) r1 (byte) [15 d1 cd 81] */
        load8(mcu.r[1], dp_addr(0xd1cd));
        mcu.pc = 0x33d9;
        return 1;
    case 0x33d9: /* MOVG2 r1 r2 (byte) [a1 82] */
        mov8(mcu.r[2], (uint32_t)mcu.r[1]);
        mcu.pc = 0x33db;
        return 1;
    case 0x33db: /* AND r0 r1 (byte) [a0 51] */
    {
        uint32_t data = (uint32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)(mcu.r[0] & 0xffu);
        data &= t2;
        mcu.r[1] = (uint16_t)((mcu.r[1] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(mcu.r[1], 0);
        mcu.pc = 0x33dd;
        return 1;
    }
    case 0x33dd: /* NOT r0 (byte) [a0 15] */
    {
        uint32_t data = (uint32_t)(mcu.r[0] & 0xffu);
        data = ~data;
        mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(data, 0);
        mcu.pc = 0x33df;
        return 1;
    }
    case 0x33df: /* XOR r0 r2 (byte) [a0 62] */
    {
        uint32_t data = (uint32_t)(mcu.r[0] & 0xffu);
        mcu.r[2] ^= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[2], 0);
        mcu.pc = 0x33e1;
        return 1;
    }
    case 0x33e1: /* AND r0 r2 (byte) [a0 52] */
    {
        uint32_t data = (uint32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[0] & 0xffu);
        data &= t2;
        mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(mcu.r[2], 0);
        mcu.pc = 0x33e3;
        return 1;
    }
    case 0x33e3: /* OR r1 r2 (byte) [a1 42] */
    {
        uint32_t data = (uint32_t)(mcu.r[1] & 0xffu);
        mcu.r[2] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[2], 0);
        mcu.pc = 0x33e5;
        return 1;
    }
    case 0x33e5: /* MOVG3 r2 -> (dp,0xd1cd) (byte) [15 d1 cd 92] */
        store8(dp_addr(0xd1cd), mcu.r[2]);
        mcu.pc = 0x33e9;
        return 1;
    case 0x33e9: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ======================================================================
 * cp4 0x433EA..0x43429, 26 PC: dp:0xd1cb bit2/bit1/bit3 gate over the
 * dp:0xd1d5 counter, then an 0x1f-entry loop that walks dp:0xd435 + r0
 * and calls 0x3532 (gen-owned) for entries that pass.  Caller bsr8 at
 * 0x433B7 (A5); single return via the rts at 0x43429.  The fall-through
 * PCs 0x433F8..0x43418 are statically reachable but dynamically unproven
 * in the recorded windows (C bytes).
 * ====================================================================== */

uint32_t step_r433ea(void)
{
    switch (mcu.pc)
    {
    case 0x33ea: /* MOVI r1 #0xd1cb [59 d1 cb] */
        movi16(mcu.r[1], 0xd1cb);
        mcu.pc = 0x33ed;
        return 1;
    case 0x33ed: /* MOVI r0 #0x001f [58 00 1f] */
        movi16(mcu.r[0], 0x001f);
        mcu.pc = 0x33f0;
        return 1;
    case 0x33f0: /* BTSTI @r1 #2 [d1 f2] */
    {
        uint32_t data = (uint32_t)MCU_Read(reg_addr(1));
        MCU_SetStatus((data & (1u << 2)) == 0, STATUS_Z);
        mcu.pc = 0x33f2;
        return 1;
    }
    case 0x33f2: /* BEQ +0x25 -> 0x3419 [27 25] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3419 : 0x33f4;
        return 1;
    case 0x33f4: /* BTSTI @r1 #1 [d1 f1] */
    {
        uint32_t data = (uint32_t)MCU_Read(reg_addr(1));
        MCU_SetStatus((data & (1u << 1)) == 0, STATUS_Z);
        mcu.pc = 0x33f6;
        return 1;
    }
    case 0x33f6: /* BEQ +0x21 -> 0x3419 [27 21] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3419 : 0x33f8;
        return 1;
    case 0x33f8: /* BTSTI @r1 #3 [d1 f3] */
    {
        uint32_t data = (uint32_t)MCU_Read(reg_addr(1));
        MCU_SetStatus((data & (1u << 3)) == 0, STATUS_Z);
        mcu.pc = 0x33fa;
        return 1;
    }
    case 0x33fa: /* BNE +6 -> 0x3402 [26 06] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x33fc : 0x3402;
        return 1;
    case 0x33fc: /* TST (dp,0xd1d5) (byte) [15 d1 d5 16] */
        tst8((uint32_t)MCU_Read(dp_addr(0xd1d5)));
        mcu.pc = 0x3400;
        return 1;
    case 0x3400: /* BNE +0x17 -> 0x3419 [26 17] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3402 : 0x3419;
        return 1;
    case 0x3402: /* MOVG2 r0 r2 (byte) [a0 82] */
        mov8(mcu.r[2], (uint32_t)mcu.r[0]);
        mcu.pc = 0x3404;
        return 1;
    case 0x3404: /* EXTU r2 [a2 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[2] & 0xffu);
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x3406;
        return 1;
    }
    case 0x3406: /* ADD.W #0xd435 r2 [0c d4 35 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            0xd435, 0, 1);
        mcu.pc = 0x340a;
        return 1;
    case 0x340a: /* TST @r2 (byte) [d2 16] */
        tst8((uint32_t)MCU_Read(reg_addr(2)));
        mcu.pc = 0x340c;
        return 1;
    case 0x340c: /* BNE +0x0b -> 0x3419 [26 0b] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x340e : 0x3419;
        return 1;
    case 0x340e: /* PUSH.W r0 [bf 90] */
        pushw(mcu.r[0]);
        mcu.pc = 0x3410;
        return 1;
    case 0x3410: /* PUSH.W r1 [bf 91] */
        pushw(mcu.r[1]);
        mcu.pc = 0x3412;
        return 1;
    case 0x3412: /* JSR @0x3532 [18 35 32] */
        call(0x3415, 0x3532);
        return 1;
    case 0x3415: /* POP.W r1 [cf 81] */
        popw(mcu.r[1]);
        mcu.pc = 0x3417;
        return 1;
    case 0x3417: /* POP.W r0 [cf 80] */
        popw(mcu.r[0]);
        mcu.pc = 0x3419;
        return 1;
    case 0x3419: /* ADDQ #-1 r1 [a9 0c] */
        mcu.r[1] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[1],
                                            -1, 0, 1);
        mcu.pc = 0x341b;
        return 1;
    case 0x341b: /* cntjmp r0 -46 -> 0x33f0 [01 b8 d2] */
        cntjmp(mcu.r[0], 0x33f0, 0x341e);
        return 1;
    case 0x341e: /* ADDQ #-1 (dp,0xd1d5) (byte) [15 d1 d5 0c] */
    {
        uint32_t addr = dp_addr(0xd1d5);
        uint32_t t1 = (uint32_t)MCU_Read(addr);
        t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, -1, 0, 0);
        MCU_Write(addr, (uint8_t)t1);
        mcu.pc = 0x3422;
        return 1;
    }
    case 0x3422: /* BCS +5 -> 0x3429 [25 05] */
        mcu.pc = (mcu.sr & STATUS_C) ? 0x3429 : 0x3424;
        return 1;
    case 0x3424: /* MOVG #0x03 -> (dp,0xd1d5) (byte) [15 d1 d5 06 03] */
        store8_imm_dp(0xd1d5, 0x03);
        mcu.pc = 0x3429;
        return 1;
    case 0x3429: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ======================================================================
 * cp4 0x4342A..0x434CA, 65 PC: voice-on/off gate keyed on dp:0xd1ff,
 * dp:0xd421 and dp:0xd364; writes dp:0xd1ff, scans dp:0xd1ac.. and
 * toggles bit 3 at @r0.  Caller jsr at 0x433BF (A5); internal arms
 * 0x343B/0x347B; single return through the shared rts at 0x434CA.
 * Calls 0x43532/0x43574 (other owners, left to gen/hand as-is).
 * ====================================================================== */

uint32_t step_r4342a(void)
{
    switch (mcu.pc)
    {
    case 0x342a: /* MOVG2 (dp,0xd1ff) r0 (byte) [15 d1 ff 80] */
        load8(mcu.r[0], dp_addr(0xd1ff));
        mcu.pc = 0x342e;
        return 1;
    case 0x342e: /* CMP.B #0xff r0 [40 ff] */
        cmp8_imm(mcu.r[0], 0xff);
        mcu.pc = 0x3430;
        return 1;
    case 0x3430: /* BNE +0x49 -> 0x347b [26 49] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3432 : 0x347b;
        return 1;
    case 0x3432: /* TST (dp,0xd421) (byte) [15 d4 21 16] */
        tst8((uint32_t)MCU_Read(dp_addr(0xd421)));
        mcu.pc = 0x3436;
        return 1;
    case 0x3436: /* BNE +3 -> 0x343b [26 03] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3438 : 0x343b;
        return 1;
    case 0x3438: /* JMP #0x34ca [10 34 ca] */
        mcu.pc = 0x34ca;
        return 1;
    case 0x343b: /* MOVG2 (dp,0xd207) r0 (byte) [15 d2 07 80] */
        load8(mcu.r[0], dp_addr(0xd207));
        mcu.pc = 0x343f;
        return 1;
    case 0x343f: /* CMP.B #0x14 r0 [40 14] */
        cmp8_imm(mcu.r[0], 0x14);
        mcu.pc = 0x3441;
        return 1;
    case 0x3441: /* BCS +0x1f -> 0x3462 [25 1f] */
        mcu.pc = (mcu.sr & STATUS_C) ? 0x3462 : 0x3443;
        return 1;
    case 0x3443: /* MOVI r1 #0x40 [51 40] */
        movi8(mcu.r[1], 0x40);
        mcu.pc = 0x3445;
        return 1;
    case 0x3445: /* CMP.B #0x04, (dp,0xd364) [15 d3 64 04 04] */
    {
        uint32_t t1 = (uint32_t)MCU_Read(dp_addr(0xd364));
        MCU_SUB_Common((int32_t)t1, 0x04, 0, 0);
        mcu.pc = 0x344a;
        return 1;
    }
    case 0x344a: /* BNE +0x24 -> 0x3470 [26 24] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x344c : 0x3470;
        return 1;
    case 0x344c: /* MOVI r1 #0x20 [51 20] */
        movi8(mcu.r[1], 0x20);
        mcu.pc = 0x344e;
        return 1;
    case 0x344e: /* CMP.B #0x45 r0 [40 45] */
        cmp8_imm(mcu.r[0], 0x45);
        mcu.pc = 0x3450;
        return 1;
    case 0x3450: /* BEQ +0x1e -> 0x3470 [27 1e] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x3452;
        return 1;
    case 0x3452: /* MOVI r1 #0x21 [51 21] */
        movi8(mcu.r[1], 0x21);
        mcu.pc = 0x3454;
        return 1;
    case 0x3454: /* CMP.B #0x2a r0 [40 2a] */
        cmp8_imm(mcu.r[0], 0x2a);
        mcu.pc = 0x3456;
        return 1;
    case 0x3456: /* BEQ +0x18 -> 0x3470 [27 18] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x3458;
        return 1;
    case 0x3458: /* MOVI r1 #0x22 [51 22] */
        movi8(mcu.r[1], 0x22);
        mcu.pc = 0x345a;
        return 1;
    case 0x345a: /* CMP.B #0x50 r0 [40 50] */
        cmp8_imm(mcu.r[0], 0x50);
        mcu.pc = 0x345c;
        return 1;
    case 0x345c: /* BEQ +0x12 -> 0x3470 [27 12] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x345e;
        return 1;
    case 0x345e: /* MOVI r1 #0x40 [51 40] */
        movi8(mcu.r[1], 0x40);
        mcu.pc = 0x3460;
        return 1;
    case 0x3460: /* BRA +0x0e -> 0x3470 [20 0e] */
        mcu.pc = 0x3470;
        return 1;
    case 0x3462: /* EXTU r0 [a0 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[0] & 0xffu);
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x3464;
        return 1;
    }
    case 0x3464: /* ADD.W #0x320b r0 [0c 32 0b 20] */
        mcu.r[0] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[0],
                                            0x320b, 0, 1);
        mcu.pc = 0x3468;
        return 1;
    case 0x3468: /* LDC #0x04 r5 [04 04 8d] */
        mcu.pc = 0x346b;
        ldc_cr(5, 0x04);
        return 1;
    case 0x346b: /* MOVG2 @r0 r1 (byte) [d0 81] */
        load8(mcu.r[1], reg_addr(0));
        mcu.pc = 0x346d;
        return 1;
    case 0x346d: /* LDC #0x00 r5 [04 00 8d] */
        mcu.pc = 0x3470;
        ldc_cr(5, 0x00);
        return 1;
    case 0x3470: /* OR #0x40 r1 [04 40 41] */
    {
        uint32_t data = 0x40;
        mcu.r[1] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[1], 0);
        mcu.pc = 0x3473;
        return 1;
    }
    case 0x3473: /* MOVG2 r1 r0 (byte) [a1 80] */
        mov8(mcu.r[0], (uint32_t)mcu.r[1]);
        mcu.pc = 0x3475;
        return 1;
    case 0x3475: /* MOVG3 r0 -> (dp,0xd1ff) (byte) [15 d1 ff 90] */
        store8(dp_addr(0xd1ff), mcu.r[0]);
        mcu.pc = 0x3479;
        return 1;
    case 0x3479: /* BRA +0x0f -> 0x348a [20 0f] */
        mcu.pc = 0x348a;
        return 1;
    case 0x347b: /* TST (dp,0xd421) (byte) [15 d4 21 16] */
        tst8((uint32_t)MCU_Read(dp_addr(0xd421)));
        mcu.pc = 0x347f;
        return 1;
    case 0x347f: /* BNE +0x49 -> 0x34ca [26 49] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3481 : 0x34ca;
        return 1;
    case 0x3481: /* MOVG #0x00ff -> (dp,0xd1ff) (byte) [15 d1 ff 07 00 ff] */
        mcu.pc = 0x3487;
        store8_imm_dp(0xd1ff, 0x00ff);
        return 1;
    case 0x3487: /* OR #0x80 r0 [04 80 40] */
    {
        uint32_t data = 0x80;
        mcu.r[0] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[0], 0);
        mcu.pc = 0x348a;
        return 1;
    }
    case 0x348a: /* MOVG2 r0 r1 (byte) [a0 81] */
        mov8(mcu.r[1], (uint32_t)mcu.r[0]);
        mcu.pc = 0x348c;
        return 1;
    case 0x348c: /* CMP.B #0x04, (dp,0xd364) [15 d3 64 04 04] */
    {
        uint32_t t1 = (uint32_t)MCU_Read(dp_addr(0xd364));
        MCU_SUB_Common((int32_t)t1, 0x04, 0, 0);
        mcu.pc = 0x3491;
        return 1;
    }
    case 0x3491: /* BEQ +0x0b -> 0x349e [27 0b] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x349e : 0x3493;
        return 1;
    case 0x3493: /* PUSH.W r1 [bf 91] */
        pushw(mcu.r[1]);
        mcu.pc = 0x3495;
        return 1;
    case 0x3495: /* JSR @0x3574 [18 35 74] */
        call(0x3498, 0x3574);
        return 1;
    case 0x3498: /* POP.W r1 [cf 81] */
        popw(mcu.r[1]);
        mcu.pc = 0x349a;
        return 1;
    case 0x349a: /* CMP.B #0xff r0 [40 ff] */
        cmp8_imm(mcu.r[0], 0xff);
        mcu.pc = 0x349c;
        return 1;
    case 0x349c: /* BEQ +7 -> 0x34a5 [27 07] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34a5 : 0x349e;
        return 1;
    case 0x349e: /* PUSH.W r1 [bf 91] */
        pushw(mcu.r[1]);
        mcu.pc = 0x34a0;
        return 1;
    case 0x34a0: /* JSR @0x3532 [18 35 32] */
        call(0x34a3, 0x3532);
        return 1;
    case 0x34a3: /* POP.W r1 [cf 81] */
        popw(mcu.r[1]);
        mcu.pc = 0x34a5;
        return 1;
    case 0x34a5: /* MOVG2 r1 r0 (byte) [a1 80] */
        mov8(mcu.r[0], (uint32_t)mcu.r[1]);
        mcu.pc = 0x34a7;
        return 1;
    case 0x34a7: /* AND #0x3f r0 [04 3f 50] */
    {
        uint32_t data = (uint32_t)mcu.r[0];
        data &= 0x3f;
        mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(mcu.r[0], 0);
        mcu.pc = 0x34aa;
        return 1;
    }
    case 0x34aa: /* CMP.B #0x14 r0 [40 14] */
        cmp8_imm(mcu.r[0], 0x14);
        mcu.pc = 0x34ac;
        return 1;
    case 0x34ac: /* BEQ +0x0c -> 0x34ba [27 0c] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34ae;
        return 1;
    case 0x34ae: /* CMP.B #0x15 r0 [40 15] */
        cmp8_imm(mcu.r[0], 0x15);
        mcu.pc = 0x34b0;
        return 1;
    case 0x34b0: /* BEQ +8 -> 0x34ba [27 08] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34b2;
        return 1;
    case 0x34b2: /* CMP.B #0x12 r0 [40 12] */
        cmp8_imm(mcu.r[0], 0x12);
        mcu.pc = 0x34b4;
        return 1;
    case 0x34b4: /* BEQ +4 -> 0x34ba [27 04] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34b6;
        return 1;
    case 0x34b6: /* CMP.B #0x13 r0 [40 13] */
        cmp8_imm(mcu.r[0], 0x13);
        mcu.pc = 0x34b8;
        return 1;
    case 0x34b8: /* BNE +0x10 -> 0x34ca [26 10] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34ca;
        return 1;
    case 0x34ba: /* EXTU r0 [a0 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[0] & 0xffu);
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x34bc;
        return 1;
    }
    case 0x34bc: /* ADD.W #0xd1ac r0 [0c d1 ac 20] */
        mcu.r[0] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[0],
                                            0xd1ac, 0, 1);
        mcu.pc = 0x34c0;
        return 1;
    case 0x34c0: /* BTSTI r1 #7 [a1 f7] */
    {
        uint32_t data = (uint32_t)(mcu.r[1] & 0xffu);
        MCU_SetStatus((data & (1u << 7)) == 0, STATUS_Z);
        mcu.pc = 0x34c2;
        return 1;
    }
    case 0x34c2: /* BNE +4 -> 0x34c8 [26 04] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34c4 : 0x34c8;
        return 1;
    case 0x34c4: /* BSET @r0 #3 [d0 c3] */
    {
        uint32_t addr = reg_addr(0);
        uint32_t data = (uint32_t)MCU_Read(addr);
        MCU_SetStatus((data & (1u << 3)) == 0, STATUS_Z);
        data |= 1u << 3;
        MCU_Write(addr, (uint8_t)data);
        mcu.pc = 0x34c6;
        return 1;
    }
    case 0x34c6: /* BRA +2 -> 0x34ca [20 02] */
        mcu.pc = 0x34ca;
        return 1;
    case 0x34c8: /* BCLR @r0 #3 [d0 d3] */
    {
        uint32_t addr = reg_addr(0);
        uint32_t data = (uint32_t)MCU_Read(addr);
        MCU_SetStatus((data & (1u << 3)) == 0, STATUS_Z);
        data &= ~(1u << 3);
        MCU_Write(addr, (uint8_t)data);
        mcu.pc = 0x34ca;
        return 1;
    }
    case 0x34ca: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ======================================================================
 * cp4 0x434CB..0x43531, 41 PC: walks dp:0xd1d6 (4 entries, stride 7?)...
 * exact per-instruction transcription; writes dp:0xecf5/0xecf6 and
 * dp:0xd1cd/dp:0xd1d6 tables; calls 0x43574 (bsr) and 0x43532 (jsr).
 * Caller jsr at 0x433BC (A5).
 * ====================================================================== */

uint32_t step_r434cb(void)
{
    switch (mcu.pc)
    {
    case 0x34cb: /* MOVI r0 #0x31f4 [58 31 f4] */
        movi16(mcu.r[0], 0x31f4);
        mcu.pc = 0x34ce;
        return 1;
    case 0x34ce: /* MOVI r1 #0xd1da [59 d1 da] */
        movi16(mcu.r[1], 0xd1da);
        mcu.pc = 0x34d1;
        return 1;
    case 0x34d1: /* MOVI r2 #0xd1d6 [5a d1 d6] */
        movi16(mcu.r[2], 0xd1d6);
        mcu.pc = 0x34d4;
        return 1;
    case 0x34d4: /* MOVI r4 #0x0003 [5c 00 03] */
        movi16(mcu.r[4], 0x0003);
        mcu.pc = 0x34d7;
        return 1;
    case 0x34d7: /* LDC #0x04 r5 [04 04 8d] */
        mcu.pc = 0x34da;
        ldc_cr(5, 0x04);
        return 1;
    case 0x34da: /* MOVG2 r0++ r3 (byte) [c0 83] */
    {
        uint32_t oea = reg_addr(0);
        mcu.r[0] += 1;
        uint32_t data = (uint32_t)MCU_Read(oea);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(data, 0);
        mcu.pc = 0x34dc;
        return 1;
    }
    case 0x34dc: /* LDC #0x00 r5 [04 00 8d] */
        mcu.pc = 0x34df;
        ldc_cr(5, 0x00);
        return 1;
    case 0x34df: /* AND (dp,0xd1cd) r3 (byte) [15 d1 cd 53] */
    {
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)MCU_Read(dp_addr(0xd1cd));
        data &= t2;
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(mcu.r[3], 0);
        mcu.pc = 0x34e3;
        return 1;
    }
    case 0x34e3: /* MOVG3 r3 -> (dp,0xecf6) (byte) [15 ec f6 93] */
        store8(dp_addr(0xecf6), mcu.r[3]);
        mcu.pc = 0x34e7;
        return 1;
    case 0x34e7: /* MOVG2 (dp,0xecf5) r5 (byte) [15 ec f5 85] */
        load8(mcu.r[5], dp_addr(0xecf5));
        mcu.pc = 0x34eb;
        return 1;
    case 0x34eb: /* OR #0x8f r3 [04 8f 43] */
    {
        uint32_t data = 0x8f;
        mcu.r[3] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[3], 0);
        mcu.pc = 0x34ee;
        return 1;
    }
    case 0x34ee: /* MOVG3 r3 -> (dp,0xecf6) (byte) [15 ec f6 93] */
        store8(dp_addr(0xecf6), mcu.r[3]);
        mcu.pc = 0x34f2;
        return 1;
    case 0x34f2: /* MOVG2 r2++ r3 (byte) [c2 83] */
    {
        uint32_t oea = reg_addr(2);
        mcu.r[2] += 1;
        uint32_t data = (uint32_t)MCU_Read(oea);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(data, 0);
        mcu.pc = 0x34f4;
        return 1;
    }
    case 0x34f4: /* MOVG2 @r1 r6 (byte) [d1 86] */
        load8(mcu.r[6], reg_addr(1));
        mcu.pc = 0x34f6;
        return 1;
    case 0x34f6: /* MOVG3 r5 -> r1++ (byte) [c1 95] */
    {
        uint32_t oea = reg_addr(1);
        mcu.r[1] += 1;
        MCU_Write(oea, (uint8_t)mcu.r[5]);
        MCU_SetStatusCommon((uint32_t)mcu.r[5], 0);
        mcu.pc = 0x34f8;
        return 1;
    }
    case 0x34f8: /* CMP r5 r6 (byte) [a5 76] */
    {
        uint32_t t1 = (uint32_t)mcu.r[6];
        uint32_t t2 = (uint32_t)(mcu.r[5] & 0xffu);
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
        mcu.pc = 0x34fa;
        return 1;
    }
    case 0x34fa: /* BNE +4 -> 0x3500 [26 04] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34fc : 0x3500;
        return 1;
    case 0x34fc: /* XOR r5 r3 (byte) [a5 63] */
    {
        uint32_t data = (uint32_t)(mcu.r[5] & 0xffu);
        mcu.r[3] ^= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[3], 0);
        mcu.pc = 0x34fe;
        return 1;
    }
    case 0x34fe: /* BNE +5 -> 0x3505 [26 05] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3500 : 0x3505;
        return 1;
    case 0x3500: /* cntjmp r4 -44 -> 0x34d7 [01 bc d4] */
        cntjmp(mcu.r[4], 0x34d7, 0x3503);
        return 1;
    case 0x3503: /* BRA +0x2c -> 0x3531 [20 2c] */
        mcu.pc = 0x3531;
        return 1;
    case 0x3505: /* MOVI r1 #0x0003 [59 00 03] */
        movi16(mcu.r[1], 0x0003);
        mcu.pc = 0x3508;
        return 1;
    case 0x3508: /* SUB r4 r1 (byte) [a4 31] */
    {
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xffu);
        int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)mcu.r[1],
                                    (int32_t)t2, 0, 0);
        mcu.r[1] = (uint16_t)((mcu.r[1] & 0xff00u) | ((uint32_t)t1 & 0xffu));
        mcu.pc = 0x350a;
        return 1;
    }
    case 0x350a: /* MULXU #0x08 r1 [04 08 a9] */
    {
        uint32_t t1 = 0x08;
        uint32_t t2 = (uint32_t)(mcu.r[1] & 0xffu);
        t1 *= t2;
        t1 &= 0xffffu;
        mcu.r[1] = (uint16_t)t1;
        MCU_SetStatus((t1 & 0x8000u) != 0, STATUS_N);
        MCU_SetStatus(t1 == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x350d;
        return 1;
    }
    case 0x350d: /* MOVI r4 #0x0007 [5c 00 07] */
        movi16(mcu.r[4], 0x0007);
        mcu.pc = 0x3510;
        return 1;
    case 0x3510: /* BTST r4 r3 (byte) [a3 7c] */
    {
        uint32_t data = (uint32_t)(mcu.r[3] & 0xffu);
        uint32_t bit = (uint32_t)(mcu.r[4] & 0x0fu);
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
        mcu.pc = 0x3512;
        return 1;
    }
    case 0x3512: /* BEQ +0x18 -> 0x352c [27 18] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x352c : 0x3514;
        return 1;
    case 0x3514: /* MOVG2 r1 r0 (byte) [a1 80] */
        mov8(mcu.r[0], (uint32_t)mcu.r[1]);
        mcu.pc = 0x3516;
        return 1;
    case 0x3516: /* ADD r4 r0 (byte) [a4 20] */
    {
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xffu);
        int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)mcu.r[0],
                                    (int32_t)t2, 0, 0);
        mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | ((uint32_t)t1 & 0xffu));
        mcu.pc = 0x3518;
        return 1;
    }
    case 0x3518: /* BTST r4 r5 (byte) [a5 7c] */
    {
        uint32_t data = (uint32_t)(mcu.r[5] & 0xffu);
        uint32_t bit = (uint32_t)(mcu.r[4] & 0x0fu);
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
        mcu.pc = 0x351a;
        return 1;
    }
    case 0x351a: /* BEQ +3 -> 0x351f [27 03] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x351f : 0x351c;
        return 1;
    case 0x351c: /* OR #0x80 r0 [04 80 40] */
    {
        uint32_t data = 0x80;
        mcu.r[0] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[0], 0);
        mcu.pc = 0x351f;
        return 1;
    }
    case 0x351f: /* MOVM rlist, @-SP (r0-r6) [12 7f] */
    {
        uint8_t rlist = 0x7f;
        for (int i = 7; i >= 0; i--)
        {
            if (rlist & (1 << i))
            {
                uint16_t data = mcu.r[i];
                if (i == 7)
                    data -= 2;
                MCU_PushStack(data);
            }
        }
        mcu.pc = 0x3521;
        return 1;
    }
    case 0x3521: /* BSR +0x51 -> 0x3574 [0e 51] */
        call(0x3523, 0x3574);
        return 1;
    case 0x3523: /* CMP.B #0xff r0 [40 ff] */
        cmp8_imm(mcu.r[0], 0xff);
        mcu.pc = 0x3525;
        return 1;
    case 0x3525: /* BEQ +3 -> 0x352a [27 03] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x352a : 0x3527;
        return 1;
    case 0x3527: /* JSR @0x3532 [18 35 32] */
        call(0x352a, 0x3532);
        return 1;
    case 0x352a: /* MOVM @SP+, rlist (r0-r6) [02 7f] */
    {
        uint8_t rlist = 0x7f;
        for (int i = 0; i < 8; i++)
        {
            if (rlist & (1 << i))
            {
                uint16_t data = MCU_PopStack();
                if (i != 7)
                    mcu.r[i] = data;
            }
        }
        mcu.pc = 0x352c;
        return 1;
    }
    case 0x352c: /* cntjmp r4 -31 -> 0x3510 [01 bc e1] */
        cntjmp(mcu.r[4], 0x3510, 0x352f);
        return 1;
    case 0x352f: /* MOVG3 r5 -> @-r2 (byte) [b2 95] */
    {
        mcu.r[2] -= 1;
        uint32_t oea = (uint32_t)mcu.r[2] & 0xffffu;
        uint32_t oep = (uint32_t)(page_of_reg(2) & 0xff);
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
        mcu.pc = 0x3531;
        return 1;
    }
    case 0x3531: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ======================================================================
 * cp4 0x43641..0x43694, 35 PC: scan dp:0xd1cc bit 0 over 0x1f entries,
 * then write the 0xd1d6 table at 0xd1da+stride; sets bits in @r3 and
 * clears @r2.  Caller jsr at 0x433B9 (A5); calls 0x43532.
 * ====================================================================== */

uint32_t step_r43641(void)
{
    switch (mcu.pc)
    {
    case 0x3641: /* MOVI r1 #0xd1cc [59 d1 cc] */
        movi16(mcu.r[1], 0xd1cc);
        mcu.pc = 0x3644;
        return 1;
    case 0x3644: /* MOVI r0 #0x001f [58 00 1f] */
        movi16(mcu.r[0], 0x001f);
        mcu.pc = 0x3647;
        return 1;
    case 0x3647: /* BTSTI --r1 #0 [b1 f0] */
    {
        mcu.r[1] -= 1;
        uint32_t data = (uint32_t)MCU_Read(reg_addr(1));
        MCU_SetStatus((data & (1u << 0)) == 0, STATUS_Z);
        mcu.pc = 0x3649;
        return 1;
    }
    case 0x3649: /* BEQ +0x46 -> 0x3691 [27 46] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3691 : 0x364b;
        return 1;
    case 0x364b: /* MOVG2 r0 r2 (byte) [a0 82] */
        mov8(mcu.r[2], (uint32_t)mcu.r[0]);
        mcu.pc = 0x364d;
        return 1;
    case 0x364d: /* EXTU r2 [a2 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[2] & 0xffu);
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x364f;
        return 1;
    }
    case 0x364f: /* ADD.W #0xd435 r2 [0c d4 35 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            0xd435, 0, 1);
        mcu.pc = 0x3653;
        return 1;
    case 0x3653: /* SUB #0x32, @r2 (byte) [d2 04 32] */
    {
        uint32_t t1 = (uint32_t)MCU_Read(reg_addr(2));
        MCU_SUB_Common((int32_t)t1, 0x32, 0, 0);
        mcu.pc = 0x3656;
        return 1;
    }
    case 0x3656: /* BCC +0x39 -> 0x3691 [24 39] */
        mcu.pc = (mcu.sr & STATUS_C) ? 0x3658 : 0x3691;
        return 1;
    case 0x3658: /* BCLR @r1 #0 [d1 d0] */
    {
        uint32_t addr = reg_addr(1);
        uint32_t data = (uint32_t)MCU_Read(addr);
        MCU_SetStatus((data & (1u << 0)) == 0, STATUS_Z);
        data &= ~(1u << 0);
        MCU_Write(addr, (uint8_t)data);
        mcu.pc = 0x365a;
        return 1;
    }
    case 0x365a: /* MOVG2 r0 r2 (byte) [a0 82] */
        mov8(mcu.r[2], (uint32_t)mcu.r[0]);
        mcu.pc = 0x365c;
        return 1;
    case 0x365c: /* EXTU r2 [a2 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[2] & 0xffu);
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x365e;
        return 1;
    }
    case 0x365e: /* ADD r2 r2 [aa 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            (int32_t)(uint32_t)mcu.r[2], 0, 1);
        mcu.pc = 0x3660;
        return 1;
    case 0x3660: /* ADD.W #0x321f r2 [0c 32 1f 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            0x321f, 0, 1);
        mcu.pc = 0x3664;
        return 1;
    case 0x3664: /* LDC #0x04 r5 [04 04 8d] */
        mcu.pc = 0x3667;
        ldc_cr(5, 0x04);
        return 1;
    case 0x3667: /* MOVG2 @r2 r3 (byte) [d2 83] */
        load8(mcu.r[3], reg_addr(2));
        mcu.pc = 0x3669;
        return 1;
    case 0x3669: /* LDC #0x00 r5 [04 00 8d] */
        mcu.pc = 0x366c;
        ldc_cr(5, 0x00);
        return 1;
    case 0x366c: /* CMP.B #0xff r3 [43 ff] */
        cmp8_imm(mcu.r[3], 0xff);
        mcu.pc = 0x366e;
        return 1;
    case 0x366e: /* BEQ +0x16 -> 0x3686 [27 16] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3686 : 0x3670;
        return 1;
    case 0x3670: /* EXTU r3 [a3 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[3] & 0xffu);
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x3672;
        return 1;
    }
    case 0x3672: /* MOVG2 r3 r2 (word) [ab 82] */
        mov16(mcu.r[2], (uint32_t)mcu.r[3]);
        mcu.pc = 0x3674;
        return 1;
    case 0x3674: /* ADD.W #0xd1ac r3 [0c d1 ac 23] */
        mcu.r[3] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[3],
                                            0xd1ac, 0, 1);
        mcu.pc = 0x3678;
        return 1;
    case 0x3678: /* BTSTI @r3 #1 [d3 f1] */
    {
        uint32_t data = (uint32_t)MCU_Read(reg_addr(3));
        MCU_SetStatus((data & (1u << 1)) == 0, STATUS_Z);
        mcu.pc = 0x367a;
        return 1;
    }
    case 0x367a: /* BEQ +0x0a -> 0x3686 [27 0a] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3686 : 0x367c;
        return 1;
    case 0x367c: /* BSET @r3 #3 [d3 c3] */
    {
        uint32_t addr = reg_addr(3);
        uint32_t data = (uint32_t)MCU_Read(addr);
        MCU_SetStatus((data & (1u << 3)) == 0, STATUS_Z);
        data |= 1u << 3;
        MCU_Write(addr, (uint8_t)data);
        mcu.pc = 0x367e;
        return 1;
    }
    case 0x367e: /* ADD.W #0xd435 r2 [0c d4 35 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            0xd435, 0, 1);
        mcu.pc = 0x3682;
        return 1;
    case 0x3682: /* CLR @r2 (byte) [d2 13] */
        MCU_Write(reg_addr(2), (uint8_t)0);
        flags_clr();
        mcu.pc = 0x3684;
        return 1;
    case 0x3684: /* BRA +0x0b -> 0x3691 [20 0b] */
        mcu.pc = 0x3691;
        return 1;
    case 0x3686: /* PUSH.W r0 [bf 90] */
        pushw(mcu.r[0]);
        mcu.pc = 0x3688;
        return 1;
    case 0x3688: /* PUSH.W r1 [bf 91] */
        pushw(mcu.r[1]);
        mcu.pc = 0x368a;
        return 1;
    case 0x368a: /* JSR @0x3532 [18 35 32] */
        call(0x368d, 0x3532);
        return 1;
    case 0x368d: /* POP.W r1 [cf 81] */
        popw(mcu.r[1]);
        mcu.pc = 0x368f;
        return 1;
    case 0x368f: /* POP.W r0 [cf 80] */
        popw(mcu.r[0]);
        mcu.pc = 0x3691;
        return 1;
    case 0x3691: /* cntjmp r0 -77 -> 0x3647 [01 b8 b3] */
        cntjmp(mcu.r[0], 0x3647, 0x3694);
        return 1;
    case 0x3694: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ======================================================================
 * cp4 0x4142F..0x41430, 2 PC: shared `nop; rts` stub called by the six A4
 * bsr sites (0x413D5/D7/FE, 0x41400/16/18).  nop/rts, no context.
 * ====================================================================== */

uint32_t step_r4142f(void)
{
    switch (mcu.pc)
    {
    case 0x142f: /* nop [00] */
        mcu.pc = 0x1430;
        return 1;
    case 0x1430: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- PC tables ------------------------------------------------------------ */

/* cp0 shared helper 0x50EF..0x516B (rts included), 54 PC. */
const uint16_t kSharedHelperPcs[] = {
    0x50ef, 0x50f1, 0x50f3, 0x50f5, 0x50f7, 0x50f9, 0x50fc, 0x50fe,
    0x5101, 0x5103, 0x5105, 0x5106, 0x5107, 0x5109, 0x510b, 0x510d,
    0x5110, 0x5112, 0x5115, 0x5117, 0x5119, 0x511b, 0x511d, 0x511f,
    0x5121, 0x5123, 0x5125, 0x5127, 0x5129, 0x512b, 0x512d, 0x512f,
    0x5131, 0x5135, 0x5139, 0x513c, 0x513f, 0x5141, 0x5144, 0x5146,
    0x5148, 0x514a, 0x514e, 0x5152, 0x5155, 0x5158, 0x515a, 0x515d,
    0x515f, 0x5161, 0x5163, 0x5165, 0x5168, 0x516b,
};

/* cp4 0x433C4..0x433D0, 3 PC. */
const uint16_t kR433c4Pcs[] = {
    0x33c4, 0x33ca, 0x33d0,
};

/* cp4 0x433D1..0x433E9, 10 PC. */
const uint16_t kR433d1Pcs[] = {
    0x33d1, 0x33d5, 0x33d9, 0x33db, 0x33dd, 0x33df, 0x33e1, 0x33e3,
    0x33e5, 0x33e9,
};

/* cp4 0x433EA..0x43429, 26 PC. */
const uint16_t kR433eaPcs[] = {
    0x33ea, 0x33ed, 0x33f0, 0x33f2, 0x33f4, 0x33f6, 0x33f8, 0x33fa,
    0x33fc, 0x3400, 0x3402, 0x3404, 0x3406, 0x340a, 0x340c, 0x340e,
    0x3410, 0x3412, 0x3415, 0x3417, 0x3419, 0x341b, 0x341e, 0x3422,
    0x3424, 0x3429,
};

/* cp4 0x4342A..0x434CA, 65 PC. */
const uint16_t kR4342aPcs[] = {
    0x342a, 0x342e, 0x3430, 0x3432, 0x3436, 0x3438, 0x343b, 0x343f,
    0x3441, 0x3443, 0x3445, 0x344a, 0x344c, 0x344e, 0x3450, 0x3452,
    0x3454, 0x3456, 0x3458, 0x345a, 0x345c, 0x345e, 0x3460, 0x3462,
    0x3464, 0x3468, 0x346b, 0x346d, 0x3470, 0x3473, 0x3475, 0x3479,
    0x347b, 0x347f, 0x3481, 0x3487, 0x348a, 0x348c, 0x3491, 0x3493,
    0x3495, 0x3498, 0x349a, 0x349c, 0x349e, 0x34a0, 0x34a3, 0x34a5,
    0x34a7, 0x34aa, 0x34ac, 0x34ae, 0x34b0, 0x34b2, 0x34b4, 0x34b6,
    0x34b8, 0x34ba, 0x34bc, 0x34c0, 0x34c2, 0x34c4, 0x34c6, 0x34c8,
    0x34ca,
};

/* cp4 0x434CB..0x43531, 41 PC. */
const uint16_t kR434cbPcs[] = {
    0x34cb, 0x34ce, 0x34d1, 0x34d4, 0x34d7, 0x34da, 0x34dc, 0x34df,
    0x34e3, 0x34e7, 0x34eb, 0x34ee, 0x34f2, 0x34f4, 0x34f6, 0x34f8,
    0x34fa, 0x34fc, 0x34fe, 0x3500, 0x3503, 0x3505, 0x3508, 0x350a,
    0x350d, 0x3510, 0x3512, 0x3514, 0x3516, 0x3518, 0x351a, 0x351c,
    0x351f, 0x3521, 0x3523, 0x3525, 0x3527, 0x352a, 0x352c, 0x352f,
    0x3531,
};

/* cp4 0x43641..0x43694, 35 PC. */
const uint16_t kR43641Pcs[] = {
    0x3641, 0x3644, 0x3647, 0x3649, 0x364b, 0x364d, 0x364f, 0x3653,
    0x3656, 0x3658, 0x365a, 0x365c, 0x365e, 0x3660, 0x3664, 0x3667,
    0x3669, 0x366c, 0x366e, 0x3670, 0x3672, 0x3674, 0x3678, 0x367a,
    0x367c, 0x367e, 0x3682, 0x3684, 0x3686, 0x3688, 0x368a, 0x368d,
    0x368f, 0x3691, 0x3694,
};

void shared_misc_fill(void)
{
    uint32_t i;

    for (i = 0; i < sizeof(kSharedHelperPcs) / sizeof(kSharedHelperPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kSharedHelperPcs[i], &step_shared_helper);

    for (i = 0; i < sizeof(kR433c4Pcs) / sizeof(kR433c4Pcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kR433c4Pcs[i], &step_r433c4);

    for (i = 0; i < sizeof(kR433d1Pcs) / sizeof(kR433d1Pcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kR433d1Pcs[i], &step_r433d1);

    for (i = 0; i < sizeof(kR433eaPcs) / sizeof(kR433eaPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kR433eaPcs[i], &step_r433ea);

    for (i = 0; i < sizeof(kR4342aPcs) / sizeof(kR4342aPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kR4342aPcs[i], &step_r4342a);

    for (i = 0; i < sizeof(kR434cbPcs) / sizeof(kR434cbPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kR434cbPcs[i], &step_r434cb);

    for (i = 0; i < sizeof(kR43641Pcs) / sizeof(kR43641Pcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kR43641Pcs[i], &step_r43641);

    MK2CPP_HandRegisterRoutine(0x00040000u | 0x142fu, &step_r4142f);
    MK2CPP_HandRegisterRoutine(0x00040000u | 0x1430u, &step_r4142f);
}

} /* anonymous namespace */
} /* namespace mk2c */

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct SharedMiscSelfRegister
{
    SharedMiscSelfRegister() { mk2c::hand_register_module(&mk2c::shared_misc_fill); }
};

SharedMiscSelfRegister g_shared_misc_self_register;

} /* anonymous namespace */
