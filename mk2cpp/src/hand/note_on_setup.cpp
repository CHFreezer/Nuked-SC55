/*
 * HAND pcm/note_on_setup — note-on voice setup (class C7, rom1 cp=0,
 * flat 0x272e..0x2d92).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). The per-PC case bodies below are an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC; see pcm_irq_service.cpp for the L0 rationale and the
 * pending semantic rewrite (M4 closure step 2).
 *
 * Semantics: entered by mask_set 0x548e BRA 0x272e (voice_materialize.cpp).
 * Reads the tone record, clears cdfe, initializes P+0xa2 / P-0x51 and the
 * gate path; two entry arms share the body: 0x272e (BTSTI P-0x3b #7, BEQ
 * 0x2874) and 0x2874. The range ends at 0x2d92 BRA 0x3bb4 (C8 begins 0x2d95).
 * Evidence: out/m4/18_closure_gap.md 1.3 C7 / 4.2 row 8 (Q10); docs/07:141.
 * Confidence: C (ROM bytes + dasm).
 *
 * Registration: all PCs self-register via MK2CPP_HandRegisterRoutine from a
 * file-static initializer (hand_registry.h); no shared aggregator file is
 * edited and duplicate registration is fatal (mk2cpp.cpp).
 */

#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_interrupt.h"
#include "mcu_opcodes.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; declared locally like the other hand modules. */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* Fallback if a stray PC inside a registered block is ever reached: execute
 * the stock instruction through the interpreter operand table. */
void stock_instruction(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
}

uint32_t step_note_on_setup(void)
{
    switch (mcu.pc)
    {

    /* ---- note ---- */
    case 0x272e: /* BTSTI @r0+-59 #7 */
    {
        mcu.pc = 0x2731;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc5;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0xf7;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t bit = 7;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x2731: /* BEQ 0x0140 -> 0x2874 */
    {
        mcu.pc = 0x2734;
        uint16_t disp = (uint16_t)(0x01 << 8);
        disp |= 0x40;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2734: /* LDC @r0+0x0099 r4 */
    {
        mcu.pc = 0x2738;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x99;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x8c;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2738: /* MOVG2 @r0+0x009e r5 */
    {
        mcu.pc = 0x273c;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9e;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x273c: /* MOVG2 @r5+8 r6 */
    {
        mcu.pc = 0x273f;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x08;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] &= ~0xff;
        mcu.r[6] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x273f: /* MOVG3 r6 -> @r0+0x00a2 */
    {
        mcu.pc = 0x2743;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0xa2;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2743: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2746;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2746: /* ADD r1 r1 */
    {
        mcu.pc = 0x2748;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;
    case 0x2748: /* CLR @r1+0xcdfe */
    {
        mcu.pc = 0x274c;
        uint32_t odisp = (uint32_t)0xcd;
        odisp = (odisp << 8) | (uint32_t)0xfe;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x274c: /* CLR @r0+-81 */
    {
        mcu.pc = 0x274f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xaf;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x274f: /* MOVG2 @r5+4 r6 */
    {
        mcu.pc = 0x2752;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x04;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] &= ~0xff;
        mcu.r[6] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2752: /* BTSTI r6 #4 */
    {
        mcu.pc = 0x2754;
        uint8_t op2 = 0xf4;
        uint32_t data = (uint32_t)(mcu.r[6] & 0xff);
        uint32_t bit = 4;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x2754: /* BEQ 0x0097 -> 0x27ee */
    {
        mcu.pc = 0x2757;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0x97;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2757: /* MOVG2 @r0+0x009b r3 */
    {
        mcu.pc = 0x275b;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9b;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x275b: /* STC r4 -> r4 */
    {
        mcu.pc = 0x275d;
        uint8_t op2 = 0x9c;
        uint32_t data = MCU_ControlRegisterRead(4, 0);
        mcu.r[4] = (uint16_t)((mcu.r[4] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x275d: /* movi r1 #0x001b */
    {
        mcu.pc = 0x2760;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x1b;
        mcu.r[1] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2760: /* MOVG2 r1 r2 */
    {
        mcu.pc = 0x2762;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2762: /* ADD r2 r2 */
    {
        mcu.pc = 0x2764;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2764: /* MOVG2 @r2+0x64d6 r2 */
    {
        mcu.pc = 0x2768;
        uint32_t odisp = (uint32_t)0x64;
        odisp = (odisp << 8) | (uint32_t)0xd6;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x82;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2768: /* CMP r0 r2 */
    {
        mcu.pc = 0x276a;
        uint8_t op2 = 0x72;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[0];
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x276a: /* BEQ 25 -> 0x2785 */
    {
        mcu.pc = 0x276c;
        uint16_t disp = (uint16_t)(int8_t)0x19;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x276c: /* SUB @r2+0 #0x000c */
    {
        mcu.pc = 0x2771;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x0c;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2771: /* BHI 18 -> 0x2785 */
    {
        mcu.pc = 0x2773;
        uint16_t disp = (uint16_t)(int8_t)0x12;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2773: /* CMP @r2+0x009b r3 */
    {
        mcu.pc = 0x2777;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9b;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x73;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    }
    break;
    case 0x2777: /* BNE 12 -> 0x2785 */
    {
        mcu.pc = 0x2779;
        uint16_t disp = (uint16_t)(int8_t)0x0c;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2779: /* CMP @r2+0x0099 r4 */
    {
        mcu.pc = 0x277d;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x99;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x74;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    }
    break;
    case 0x277d: /* BNE 6 -> 0x2785 */
    {
        mcu.pc = 0x277f;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x277f: /* CMP @r2+0x009e r5 */
    {
        mcu.pc = 0x2783;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9e;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x75;
        int32_t t1 = (int32_t)mcu.r[5];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2783: /* BEQ 5 -> 0x278a */
    {
        mcu.pc = 0x2785;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2785: /* cntjmp r1 -40 -> 0x2760 */
    {
        mcu.pc = 0x2788;
        uint8_t op2 = 0xb9;
        uint8_t reg = op2 & 0x07;
        if ((op2 >> 3) == 0x17)
        {
        uint16_t disp = (uint16_t)(int8_t)0xd8;
        mcu.r[reg]--;
        if (mcu.r[reg] != 0xffff)
        mcu.pc += disp;
        }
        else
        {
        MCU_ErrorTrap();
        }
    }
    break;
    case 0x2788: /* BRA 100 -> 0x27ee */
    {
        mcu.pc = 0x278a;
        uint16_t disp = (uint16_t)(int8_t)0x64;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x278a: /* MOVG2 @r2+-88 r6 */
    {
        mcu.pc = 0x278d;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xa8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x278d: /* MOVG3 r6 -> @r0+-88 */
    {
        mcu.pc = 0x2790;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xa8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2790: /* MOVG2 @r2+-86 r6 */
    {
        mcu.pc = 0x2793;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xaa;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2793: /* MOVG3 r6 -> @r0+-86 */
    {
        mcu.pc = 0x2796;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xaa;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2796: /* MOVG2 @r2+-84 r6 */
    {
        mcu.pc = 0x2799;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xac;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2799: /* MOVG3 r6 -> @r0+-84 */
    {
        mcu.pc = 0x279c;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xac;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x279c: /* MOVG2 @r2+-80 r6 */
    {
        mcu.pc = 0x279f;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xb0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x279f: /* MOVG3 r6 -> @r0+-80 */
    {
        mcu.pc = 0x27a2;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27a2: /* MOVG2 @r2+-82 r6 */
    {
        mcu.pc = 0x27a5;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xae;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] &= ~0xff;
        mcu.r[6] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x27a5: /* MOVG3 r6 -> @r0+-82 */
    {
        mcu.pc = 0x27a8;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xae;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x27a8: /* MOVG2 @r2+-78 r6 */
    {
        mcu.pc = 0x27ab;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xb2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27ab: /* MOVG3 r6 -> @r0+-78 */
    {
        mcu.pc = 0x27ae;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27ae: /* MOVG2 @r2+-76 r6 */
    {
        mcu.pc = 0x27b1;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xb4;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27b1: /* MOVG3 r6 -> @r0+-76 */
    {
        mcu.pc = 0x27b4;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb4;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27b4: /* MOVG2 @r2+-74 r6 */
    {
        mcu.pc = 0x27b7;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xb6;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27b7: /* MOVG3 r6 -> @r0+-74 */
    {
        mcu.pc = 0x27ba;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb6;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27ba: /* MOVG2 @r2+-72 r6 */
    {
        mcu.pc = 0x27bd;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xb8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27bd: /* MOVG3 r6 -> @r0+-72 */
    {
        mcu.pc = 0x27c0;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27c0: /* MOVG2 @r2+-70 r6 */
    {
        mcu.pc = 0x27c3;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xba;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27c3: /* MOVG3 r6 -> @r0+-70 */
    {
        mcu.pc = 0x27c6;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xba;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27c6: /* MOVG2 @r2+-68 r6 */
    {
        mcu.pc = 0x27c9;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xbc;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27c9: /* MOVG3 r6 -> @r0+-68 */
    {
        mcu.pc = 0x27cc;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xbc;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27cc: /* MOVG2 @r2+-66 r6 */
    {
        mcu.pc = 0x27cf;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xbe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27cf: /* MOVG3 r6 -> @r0+-66 */
    {
        mcu.pc = 0x27d2;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xbe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27d2: /* MOVG2 @r2+-64 r6 */
    {
        mcu.pc = 0x27d5;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xc0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27d5: /* MOVG3 r6 -> @r0+-64 */
    {
        mcu.pc = 0x27d8;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27d8: /* MOVG2 @r2+-62 r6 */
    {
        mcu.pc = 0x27db;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xc2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27db: /* MOVG3 r6 -> @r0+-62 */
    {
        mcu.pc = 0x27de;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27de: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x27e1;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27e1: /* ADD r1 r1 */
    {
        mcu.pc = 0x27e3;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;
    case 0x27e3: /* MOVG3 r2 -> @r1+0xcdfe */
    {
        mcu.pc = 0x27e7;
        uint32_t odisp = (uint32_t)0xcd;
        odisp = (odisp << 8) | (uint32_t)0xfe;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27e7: /* MOVG #0xff -> @r0+-81 */
    {
        mcu.pc = 0x27eb;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xaf;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0xff;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x27eb: /* BRA 0x0086 -> 0x2874 */
    {
        mcu.pc = 0x27ee;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0x86;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x27ee: /* MOVG2 r6 r3 */
    {
        mcu.pc = 0x27f0;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)(mcu.r[6] & 0xff);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x27f0: /* AND #0x00c0 r3 */
    {
        mcu.pc = 0x27f4;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0xc0;
        uint8_t op2 = 0x53;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[3], 1);
    }
    break;
    case 0x27f4: /* SWAP r3 */
    {
        mcu.pc = 0x27f6;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27f6: /* MOVG3 r3 -> @r0+-72 */
    {
        mcu.pc = 0x27f9;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x27f9: /* MOVG2 r6 r3 */
    {
        mcu.pc = 0x27fb;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)(mcu.r[6] & 0xff);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x27fb: /* AND #0x000f r3 */
    {
        mcu.pc = 0x27ff;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x0f;
        uint8_t op2 = 0x53;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[3], 1);
    }
    break;
    case 0x27ff: /* MOVG2 @r3+0x7207 r3 */
    {
        mcu.pc = 0x2803;
        uint32_t odisp = (uint32_t)0x72;
        odisp = (odisp << 8) | (uint32_t)0x07;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2803: /* MOVG3 r3 -> @r0+-74 */
    {
        mcu.pc = 0x2806;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb6;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2806: /* CLR @r0+-64 */
    {
        mcu.pc = 0x2809;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2809: /* CLR @r0+-70 */
    {
        mcu.pc = 0x280c;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xba;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x280c: /* CLR @r0+-68 */
    {
        mcu.pc = 0x280f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xbc;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x280f: /* CLR @r0+-62 */
    {
        mcu.pc = 0x2812;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2812: /* CLR @r0+-88 */
    {
        mcu.pc = 0x2815;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xa8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2815: /* CLR @r0+-86 */
    {
        mcu.pc = 0x2818;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xaa;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2818: /* CLR @r0+-84 */
    {
        mcu.pc = 0x281b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xac;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x281b: /* CLR r3 */
    {
        mcu.pc = 0x281d;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x281d: /* MOVG2 @r5+6 r3 */
    {
        mcu.pc = 0x2820;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x06;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2820: /* BMI 8 -> 0x282a */
    {
        mcu.pc = 0x2822;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2822: /* ADD r3 r3 */
    {
        mcu.pc = 0x2824;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2824: /* MOVG2 @r3+0x6e86 r3 */
    {
        mcu.pc = 0x2828;
        uint32_t odisp = (uint32_t)0x6e;
        odisp = (odisp << 8) | (uint32_t)0x86;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x83;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2828: /* BRA 2 -> 0x282c */
    {
        mcu.pc = 0x282a;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x282a: /* CLR r3 */
    {
        mcu.pc = 0x282c;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x282c: /* MOVG3 r3 -> @r0+-78 */
    {
        mcu.pc = 0x282f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x282f: /* CLR r3 */
    {
        mcu.pc = 0x2831;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2831: /* MOVG2 @r5+7 r3 */
    {
        mcu.pc = 0x2834;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x07;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2834: /* ADD r3 r3 */
    {
        mcu.pc = 0x2836;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2836: /* MOVG2 @r3+0x6e86 r3 */
    {
        mcu.pc = 0x283a;
        uint32_t odisp = (uint32_t)0x6e;
        odisp = (odisp << 8) | (uint32_t)0x86;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x83;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x283a: /* MOVG3 r3 -> @r0+-76 */
    {
        mcu.pc = 0x283d;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb4;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x283d: /* CLR r3 */
    {
        mcu.pc = 0x283f;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x283f: /* MOVG2 @r5+5 r3 */
    {
        mcu.pc = 0x2842;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x05;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2842: /* MOVG3 r3 -> @r0+-82 */
    {
        mcu.pc = 0x2845;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xae;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2845: /* MOVG2 (dp,0xad2a) r6 */
    {
        mcu.pc = 0x2849;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2849: /* MOVG3 r6 -> (dp,0xad2c) */
    {
        mcu.pc = 0x284d;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x284d: /* MOVG #0x0001 -> (dp,0xad2a) */
    {
        mcu.pc = 0x2853;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x01;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2853: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x2857;
        uint32_t odata = (uint32_t)0x07;
        odata = (odata << 8) | (uint32_t)0x00;
        uint8_t op2 = 0x48;
        uint32_t data = odata;
        uint32_t val = MCU_ControlRegisterRead(0, 1);
        val |= data;
        MCU_ControlRegisterWrite(0, 1, val);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2857: /* MOVG #0x1e -> (br,$3e) */
    {
        mcu.pc = 0x285b;
        uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x3e;
        oea &= 0xffff;
        uint32_t oep = 0;
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x1e;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x285b: /* movl r4 @(br,$34) */
    {
        mcu.pc = 0x285d;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x34;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x285d: /* movlw r4 @(br,$3a) */
    {
        mcu.pc = 0x285f;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3a;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint16_t data = MCU_Read16(addr);
        mcu.r[4] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x285f: /* MOVG3 r4 -> @r0+-66 */
    {
        mcu.pc = 0x2862;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xbe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x94;
        uint32_t data = (uint32_t)mcu.r[4];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2862: /* MOVG3 r4 -> @r0+-64 */
    {
        mcu.pc = 0x2865;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x94;
        uint32_t data = (uint32_t)mcu.r[4];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2865: /* bsr16 -> 0x37fe */
    {
        mcu.pc = 0x2868;
        uint16_t disp = (uint16_t)(0x0f << 8);
        disp |= 0x96;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2868: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x286c;
        uint32_t odata = (uint32_t)0xf8;
        odata = (odata << 8) | (uint32_t)0xff;
        uint8_t op2 = 0x58;
        uint32_t data = odata;
        uint32_t val = MCU_ControlRegisterRead(0, 1);
        val &= data;
        MCU_ControlRegisterWrite(0, 1, val);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x286c: /* MOVG2 (dp,0xad2c) r6 */
    {
        mcu.pc = 0x2870;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2870: /* MOVG3 r6 -> (dp,0xad2a) */
    {
        mcu.pc = 0x2874;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2874: /* LDC @r0+0x009a r4 */
    {
        mcu.pc = 0x2878;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9a;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x8c;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2878: /* MOVG2 @r0+0x00a0 r5 */
    {
        mcu.pc = 0x287c;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0xa0;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x287c: /* MOVG2 @r5+1 r3 */
    {
        mcu.pc = 0x287f;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x01;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x287f: /* MOVG3 r3 -> @r0+-20 */
    {
        mcu.pc = 0x2882;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xec;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2882: /* MOVG2 @r5+2 r4 */
    {
        mcu.pc = 0x2885;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x02;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x84;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2885: /* MOVG3 r4 -> @r0+-16 */
    {
        mcu.pc = 0x2888;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x94;
        uint32_t data = (uint32_t)mcu.r[4];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2888: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x288b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x288b: /* BTSTI @r0+-59 #7 */
    {
        mcu.pc = 0x288e;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc5;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0xf7;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t bit = 7;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x288e: /* BNE 15 -> 0x289f */
    {
        mcu.pc = 0x2890;
        uint16_t disp = (uint16_t)(int8_t)0x0f;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2890: /* MOVG2 @r5+4 r6 */
    {
        mcu.pc = 0x2893;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x04;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2893: /* CLR r2 */
    {
        mcu.pc = 0x2895;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2895: /* ADD r4 r6 */
    {
        mcu.pc = 0x2897;
        uint8_t op2 = 0x26;
        int32_t t1 = (int32_t)mcu.r[6];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[6] = (uint16_t)t1;
    }
    break;
    case 0x2897: /* ADDX r3 r2 */
    {
        mcu.pc = 0x2899;
        uint8_t op2 = 0xa2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 0);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2899: /* MOVG3 r2 -> @r0+-20 */
    {
        mcu.pc = 0x289c;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xec;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x289c: /* MOVG3 r6 -> @r0+-16 */
    {
        mcu.pc = 0x289f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x289f: /* ADD @r5+6 r4 */
    {
        mcu.pc = 0x28a2;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x06;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x28a2: /* ADDX #0x00 r3 */
    {
        mcu.pc = 0x28a5;
        uint32_t odata = (uint32_t)0x00;
        uint8_t op2 = 0xa3;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 0);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x28a5: /* MOVG3 r3 -> @r0+-19 */
    {
        mcu.pc = 0x28a8;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xed;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x28a8: /* MOVG3 r4 -> @r0+-14 */
    {
        mcu.pc = 0x28ab;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x94;
        uint32_t data = (uint32_t)mcu.r[4];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28ab: /* SUB @r5+8 r4 */
    {
        mcu.pc = 0x28ae;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x08;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x28ae: /* SUBX #0x00 r3 */
    {
        mcu.pc = 0x28b1;
        uint32_t odata = (uint32_t)0x00;
        uint8_t op2 = 0xb3;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        int32_t C = (mcu.sr & STATUS_C) != 0;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, C, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x28b1: /* MOVG3 r3 -> @r0+-18 */
    {
        mcu.pc = 0x28b4;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xee;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x28b4: /* MOVG3 r4 -> @r0+-12 */
    {
        mcu.pc = 0x28b7;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf4;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x94;
        uint32_t data = (uint32_t)mcu.r[4];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28b7: /* ADD r3 r3 */
    {
        mcu.pc = 0x28b9;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x28b9: /* ADD r3 r3 */
    {
        mcu.pc = 0x28bb;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x28bb: /* ADD r3 r3 */
    {
        mcu.pc = 0x28bd;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x28bd: /* ADD r3 r3 */
    {
        mcu.pc = 0x28bf;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x28bf: /* MOVG2 @r5+10 r4 */
    {
        mcu.pc = 0x28c2;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x0a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x28c2: /* MOVG2 r4 r5 */
    {
        mcu.pc = 0x28c4;
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x28c4: /* AND #0x0001 r4 */
    {
        mcu.pc = 0x28c8;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x01;
        uint8_t op2 = 0x54;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[4], 1);
    }
    break;
    case 0x28c8: /* SWAP r4 */
    {
        mcu.pc = 0x28ca;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28ca: /* SHLR r4 */
    {
        mcu.pc = 0x28cc;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28cc: /* SHLR r4 */
    {
        mcu.pc = 0x28ce;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28ce: /* MOVG2 r4 r3 */
    {
        mcu.pc = 0x28d0;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x28d0: /* OR r1 r3 */
    {
        mcu.pc = 0x28d2;
        uint8_t op2 = 0x43;
        uint32_t data = (uint32_t)(mcu.r[1] & 0xff);
        mcu.r[3] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[3], 0);
    }
    break;
    case 0x28d2: /* MOVG3 r3 -> @r0+-10 */
    {
        mcu.pc = 0x28d5;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf6;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28d5: /* AND #0x02 r5 */
    {
        mcu.pc = 0x28d8;
        uint32_t odata = (uint32_t)0x02;
        uint8_t op2 = 0x55;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(mcu.r[5], 0);
    }
    break;
    case 0x28d8: /* MOVG3 r5 -> @r0+-17 */
    {
        mcu.pc = 0x28db;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xef;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x28db: /* BTSTI @r0+-59 #7 */
    {
        mcu.pc = 0x28de;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc5;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0xf7;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t bit = 7;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x28de: /* BNE 69 -> 0x2925 */
    {
        mcu.pc = 0x28e0;
        uint16_t disp = (uint16_t)(int8_t)0x45;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x28e0: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x28e4;
        uint32_t odata = (uint32_t)0x07;
        odata = (odata << 8) | (uint32_t)0x00;
        uint8_t op2 = 0x48;
        uint32_t data = odata;
        uint32_t val = MCU_ControlRegisterRead(0, 1);
        val |= data;
        MCU_ControlRegisterWrite(0, 1, val);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x28e4: /* movs r1 @(br,$3e) */
    {
        mcu.pc = 0x28e6;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3e;
        uint32_t data = (uint32_t)(mcu.r[1] & 0xff);
        MCU_Write(addr, (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x28e6: /* MOVG #0x00b4 -> (br,$16) */
    {
        mcu.pc = 0x28eb;
        uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x16;
        oea &= 0xffff;
        uint32_t oep = 0;
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0xb4;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x28eb: /* MOVG #0x00b4 -> @r0+26 */
    {
        mcu.pc = 0x28f0;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0xb4;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x28f0: /* MOVG2 @r0+28 r6 */
    {
        mcu.pc = 0x28f3;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28f3: /* move r6 #0xaf */
    {
        mcu.pc = 0x28f5;
        uint8_t data = 0xaf;
        mcu.r[6] &= ~0xff;
        mcu.r[6] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x28f5: /* MOVG3 r6 -> @r0+30 */
    {
        mcu.pc = 0x28f8;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28f8: /* MOVG2 @r0+36 r6 */
    {
        mcu.pc = 0x28fb;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x24;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28fb: /* MOVG3 r6 -> @r0+-22 */
    {
        mcu.pc = 0x28fe;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xea;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x28fe: /* move r6 #0xaf */
    {
        mcu.pc = 0x2900;
        uint8_t data = 0xaf;
        mcu.r[6] &= ~0xff;
        mcu.r[6] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2900: /* MOVG3 r6 -> @r0+38 */
    {
        mcu.pc = 0x2903;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x26;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2903: /* MOVG3 r6 -> @r0+-24 */
    {
        mcu.pc = 0x2906;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2906: /* MOVG2 @r0+0 r6 */
    {
        mcu.pc = 0x2909;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2909: /* cmp r6,w #0x0018 */
    {
        mcu.pc = 0x290c;
        int32_t t2 = (int32_t)0x00;
        t2 = (t2 << 8) | (int32_t)0x18;
        int32_t t1 = (int32_t)mcu.r[6];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x290c: /* BEQ 8 -> 0x2916 */
    {
        mcu.pc = 0x290e;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x290e: /* MOVG #0x0018 -> @r0+0 */
    {
        mcu.pc = 0x2913;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x18;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2913: /* MOVG3 r6 -> @r0+6 */
    {
        mcu.pc = 0x2916;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x06;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2916: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x291a;
        uint32_t odata = (uint32_t)0xf8;
        odata = (odata << 8) | (uint32_t)0xff;
        uint8_t op2 = 0x58;
        uint32_t data = odata;
        uint32_t val = MCU_ControlRegisterRead(0, 1);
        val &= data;
        MCU_ControlRegisterWrite(0, 1, val);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x291a: /* SUB @r1+0xd0e0 #0x00 */
    {
        mcu.pc = 0x291f;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xe0;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x04;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
    }
    break;
    case 0x291f: /* BNE 0x2b70 -> 0x5492 */
    {
        mcu.pc = 0x2922;
        uint16_t disp = (uint16_t)(0x2b << 8);
        disp |= 0x70;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2922: /* BRA 0x1ca7 -> 0x45cc */
    {
        mcu.pc = 0x2925;
        uint16_t disp = (uint16_t)(0x1c << 8);
        disp |= 0xa7;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2925: /* MOVG #0xffff -> @r0+6 */
    {
        mcu.pc = 0x292a;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x06;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0xff;
        d = (d << 8) | (uint32_t)0xff;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x292a: /* LDC @r0+0x0099 r4 */
    {
        mcu.pc = 0x292e;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x99;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x8c;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x292e: /* MOVG2 @r0+0x009e r5 */
    {
        mcu.pc = 0x2932;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9e;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2932: /* movi r2 #0x00ff */
    {
        mcu.pc = 0x2935;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0xff;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2935: /* CLR r3 */
    {
        mcu.pc = 0x2937;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2937: /* MOVG2 @r5+69 r3 */
    {
        mcu.pc = 0x293a;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x45;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x293a: /* SUB @r3+0x6883 r2 */
    {
        mcu.pc = 0x293e;
        uint32_t odisp = (uint32_t)0x68;
        odisp = (odisp << 8) | (uint32_t)0x83;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x293e: /* BHI 2 -> 0x2942 */
    {
        mcu.pc = 0x2940;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2940: /* move r2 #0x01 */
    {
        mcu.pc = 0x2942;
        uint8_t data = 0x01;
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2942: /* CLR r4 */
    {
        mcu.pc = 0x2944;
        uint8_t op2 = 0x13;
        mcu.r[4] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2944: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2947;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2947: /* MOVG2 @r1+0xd134 r4 */
    {
        mcu.pc = 0x294b;
        uint32_t odisp = (uint32_t)0xd1;
        odisp = (odisp << 8) | (uint32_t)0x34;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x294b: /* LDC #0x03 r5 */
    {
        mcu.pc = 0x294e;
        uint32_t odata = (uint32_t)0x03;
        uint8_t op2 = 0x8d;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(5, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x294e: /* MOVG2 @r5+70 r3 */
    {
        mcu.pc = 0x2951;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x46;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2951: /* ADD r3 r3 */
    {
        mcu.pc = 0x2953;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2953: /* MOVG2 @r3+0xdd7c r3 */
    {
        mcu.pc = 0x2957;
        uint32_t odisp = (uint32_t)0xdd;
        odisp = (odisp << 8) | (uint32_t)0x7c;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x83;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2957: /* ADD r4 r3 */
    {
        mcu.pc = 0x2959;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2959: /* MOVG2 @r3 r4 */
    {
        mcu.pc = 0x295b;
        uint32_t oea = (uint32_t)mcu.r[3];
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x295b: /* LDC #0x00 r5 */
    {
        mcu.pc = 0x295e;
        uint32_t odata = (uint32_t)0x00;
        uint8_t op2 = 0x8d;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(5, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x295e: /* CLR r3 */
    {
        mcu.pc = 0x2960;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2960: /* MOVG2 @r5+71 r3 */
    {
        mcu.pc = 0x2963;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x47;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2963: /* SUB #0x40 r3 */
    {
        mcu.pc = 0x2966;
        uint32_t odata = (uint32_t)0x40;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2966: /* BCC 11 -> 0x2973 */
    {
        mcu.pc = 0x2968;
        uint16_t disp = (uint16_t)(int8_t)0x0b;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2968: /* NEG r3 */
    {
        mcu.pc = 0x296a;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x296a: /* SUB #0x80 r4 */
    {
        mcu.pc = 0x296d;
        uint32_t odata = (uint32_t)0x80;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x296d: /* BCC 11 -> 0x297a */
    {
        mcu.pc = 0x296f;
        uint16_t disp = (uint16_t)(int8_t)0x0b;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x296f: /* NEG r4 */
    {
        mcu.pc = 0x2971;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[4] = (uint16_t)((mcu.r[4] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2971: /* BRA 28 -> 0x298f */
    {
        mcu.pc = 0x2973;
        uint16_t disp = (uint16_t)(int8_t)0x1c;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2973: /* SUB #0x80 r4 */
    {
        mcu.pc = 0x2976;
        uint32_t odata = (uint32_t)0x80;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2976: /* BCC 23 -> 0x298f */
    {
        mcu.pc = 0x2978;
        uint16_t disp = (uint16_t)(int8_t)0x17;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2978: /* NEG r4 */
    {
        mcu.pc = 0x297a;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[4] = (uint16_t)((mcu.r[4] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x297a: /* MULXU @r3+0x650e r4 */
    {
        mcu.pc = 0x297e;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[4];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[4] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x297e: /* ADD r4 r4 */
    {
        mcu.pc = 0x2980;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2980: /* SWAP r4 */
    {
        mcu.pc = 0x2982;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2982: /* MOVG2 r4 r3 */
    {
        mcu.pc = 0x2984;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2984: /* SUB @r3+0x673a r2 */
    {
        mcu.pc = 0x2988;
        uint32_t odisp = (uint32_t)0x67;
        odisp = (odisp << 8) | (uint32_t)0x3a;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2988: /* BHI 24 -> 0x29a2 */
    {
        mcu.pc = 0x298a;
        uint16_t disp = (uint16_t)(int8_t)0x18;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x298a: /* movi r2 #0x0001 */
    {
        mcu.pc = 0x298d;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x01;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x298d: /* BRA 19 -> 0x29a2 */
    {
        mcu.pc = 0x298f;
        uint16_t disp = (uint16_t)(int8_t)0x13;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x298f: /* MULXU @r3+0x650e r4 */
    {
        mcu.pc = 0x2993;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[4];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[4] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2993: /* ADD r4 r4 */
    {
        mcu.pc = 0x2995;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2995: /* SWAP r4 */
    {
        mcu.pc = 0x2997;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2997: /* MOVG2 r4 r3 */
    {
        mcu.pc = 0x2999;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2999: /* ADD @r3+0x673a r2 */
    {
        mcu.pc = 0x299d;
        uint32_t odisp = (uint32_t)0x67;
        odisp = (odisp << 8) | (uint32_t)0x3a;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x299d: /* BCC 3 -> 0x29a2 */
    {
        mcu.pc = 0x299f;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x299f: /* movi r2 #0x00ff */
    {
        mcu.pc = 0x29a2;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0xff;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x29a2: /* MOVG2 @r1+0xcee8 r3 */
    {
        mcu.pc = 0x29a6;
        uint32_t odisp = (uint32_t)0xce;
        odisp = (odisp << 8) | (uint32_t)0xe8;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29a6: /* SUB @r3+0x6883 r2 */
    {
        mcu.pc = 0x29aa;
        uint32_t odisp = (uint32_t)0x68;
        odisp = (odisp << 8) | (uint32_t)0x83;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x29aa: /* BHI 2 -> 0x29ae */
    {
        mcu.pc = 0x29ac;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x29ac: /* move r2 #0x01 */
    {
        mcu.pc = 0x29ae;
        uint8_t data = 0x01;
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29ae: /* LDC @r0+0x009a r4 */
    {
        mcu.pc = 0x29b2;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9a;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x8c;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x29b2: /* MOVG2 @r0+0x00a0 r5 */
    {
        mcu.pc = 0x29b6;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0xa0;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x29b6: /* MOVG2 @r5+0 r3 */
    {
        mcu.pc = 0x29b9;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29b9: /* SUB @r3+0x6883 r2 */
    {
        mcu.pc = 0x29bd;
        uint32_t odisp = (uint32_t)0x68;
        odisp = (odisp << 8) | (uint32_t)0x83;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x29bd: /* BHI 2 -> 0x29c1 */
    {
        mcu.pc = 0x29bf;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x29bf: /* move r2 #0x01 */
    {
        mcu.pc = 0x29c1;
        uint8_t data = 0x01;
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29c1: /* LDC @r0+0x0098 r4 */
    {
        mcu.pc = 0x29c5;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x98;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x8c;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x29c5: /* MOVG2 @r0+0x009c r5 */
    {
        mcu.pc = 0x29c9;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9c;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x29c9: /* MOVG2 @r5+12 r3 */
    {
        mcu.pc = 0x29cc;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x0c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29cc: /* SUB @r3+0x6883 r2 */
    {
        mcu.pc = 0x29d0;
        uint32_t odisp = (uint32_t)0x68;
        odisp = (odisp << 8) | (uint32_t)0x83;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x29d0: /* BHI 2 -> 0x29d4 */
    {
        mcu.pc = 0x29d2;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x29d2: /* move r2 #0x01 */
    {
        mcu.pc = 0x29d4;
        uint8_t data = 0x01;
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29d4: /* LDC @r0+0x0099 r4 */
    {
        mcu.pc = 0x29d8;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x99;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x8c;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x29d8: /* MOVG2 @r0+0x009e r5 */
    {
        mcu.pc = 0x29dc;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9e;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x29dc: /* MOVG2 r2 r6 */
    {
        mcu.pc = 0x29de;
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)mcu.r[2];
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x29de: /* MOVG2 @r5+74 r3 */
    {
        mcu.pc = 0x29e1;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x4a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29e1: /* SUB @r3+0x6883 r2 */
    {
        mcu.pc = 0x29e5;
        uint32_t odisp = (uint32_t)0x68;
        odisp = (odisp << 8) | (uint32_t)0x83;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x29e5: /* BCC 2 -> 0x29e9 */
    {
        mcu.pc = 0x29e7;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x29e7: /* CLR r2 */
    {
        mcu.pc = 0x29e9;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x29e9: /* MOVG2 @r2+0x6903 r3 */
    {
        mcu.pc = 0x29ed;
        uint32_t odisp = (uint32_t)0x69;
        odisp = (odisp << 8) | (uint32_t)0x03;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29ed: /* MOVG3 r3 -> @r0+97 */
    {
        mcu.pc = 0x29f0;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x61;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29f0: /* MOVG2 r6 r2 */
    {
        mcu.pc = 0x29f2;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[6];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x29f2: /* MOVG2 @r5+75 r3 */
    {
        mcu.pc = 0x29f5;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x4b;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x29f5: /* SUB @r3+0x6883 r2 */
    {
        mcu.pc = 0x29f9;
        uint32_t odisp = (uint32_t)0x68;
        odisp = (odisp << 8) | (uint32_t)0x83;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x29f9: /* BCC 2 -> 0x29fd */
    {
        mcu.pc = 0x29fb;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x29fb: /* CLR r2 */
    {
        mcu.pc = 0x29fd;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x29fd: /* MOVG2 @r2+0x6903 r3 */
    {
        mcu.pc = 0x2a01;
        uint32_t odisp = (uint32_t)0x69;
        odisp = (odisp << 8) | (uint32_t)0x03;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a01: /* MOVG3 r3 -> @r0+98 */
    {
        mcu.pc = 0x2a04;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x62;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a04: /* MOVG2 r6 r2 */
    {
        mcu.pc = 0x2a06;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[6];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a06: /* MOVG2 @r5+76 r3 */
    {
        mcu.pc = 0x2a09;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x4c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a09: /* SUB @r3+0x6883 r2 */
    {
        mcu.pc = 0x2a0d;
        uint32_t odisp = (uint32_t)0x68;
        odisp = (odisp << 8) | (uint32_t)0x83;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2a0d: /* BCC 2 -> 0x2a11 */
    {
        mcu.pc = 0x2a0f;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a0f: /* CLR r2 */
    {
        mcu.pc = 0x2a11;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a11: /* MOVG2 @r2+0x6903 r3 */
    {
        mcu.pc = 0x2a15;
        uint32_t odisp = (uint32_t)0x69;
        odisp = (odisp << 8) | (uint32_t)0x03;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a15: /* MOVG3 r3 -> @r0+99 */
    {
        mcu.pc = 0x2a18;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x63;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a18: /* MOVG2 r6 r2 */
    {
        mcu.pc = 0x2a1a;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[6];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a1a: /* MOVG2 @r5+77 r3 */
    {
        mcu.pc = 0x2a1d;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x4d;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a1d: /* SUB @r3+0x6883 r2 */
    {
        mcu.pc = 0x2a21;
        uint32_t odisp = (uint32_t)0x68;
        odisp = (odisp << 8) | (uint32_t)0x83;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2a21: /* BCC 2 -> 0x2a25 */
    {
        mcu.pc = 0x2a23;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a23: /* CLR r2 */
    {
        mcu.pc = 0x2a25;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a25: /* MOVG2 @r2+0x6903 r3 */
    {
        mcu.pc = 0x2a29;
        uint32_t odisp = (uint32_t)0x69;
        odisp = (odisp << 8) | (uint32_t)0x03;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a29: /* MOVG3 r3 -> @r0+100 */
    {
        mcu.pc = 0x2a2c;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x64;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a2c: /* CLR @r0+96 */
    {
        mcu.pc = 0x2a2f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x60;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a2f: /* CLR r4 */
    {
        mcu.pc = 0x2a31;
        uint8_t op2 = 0x13;
        mcu.r[4] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a31: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2a34;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a34: /* MOVG2 @r1+0xd134 r4 */
    {
        mcu.pc = 0x2a38;
        uint32_t odisp = (uint32_t)0xd1;
        odisp = (odisp << 8) | (uint32_t)0x34;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a38: /* MOVG2 @r5+85 r2 */
    {
        mcu.pc = 0x2a3b;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x55;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a3b: /* ADD r2 r2 */
    {
        mcu.pc = 0x2a3d;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2a3d: /* LDC #0x03 r5 */
    {
        mcu.pc = 0x2a40;
        uint32_t odata = (uint32_t)0x03;
        uint8_t op2 = 0x8d;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(5, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2a40: /* MOVG2 @r2+0xdd9c r2 */
    {
        mcu.pc = 0x2a44;
        uint32_t odisp = (uint32_t)0xdd;
        odisp = (odisp << 8) | (uint32_t)0x9c;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x82;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a44: /* ADD r4 r2 */
    {
        mcu.pc = 0x2a46;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2a46: /* MOVG2 @r2 r4 */
    {
        mcu.pc = 0x2a48;
        uint32_t oea = (uint32_t)mcu.r[2];
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a48: /* LDC #0x00 r5 */
    {
        mcu.pc = 0x2a4b;
        uint32_t odata = (uint32_t)0x00;
        uint8_t op2 = 0x8d;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(5, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2a4b: /* CLR r3 */
    {
        mcu.pc = 0x2a4d;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a4d: /* MOVG2 @r5+87 r3 */
    {
        mcu.pc = 0x2a50;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x57;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a50: /* SUB #0x40 r3 */
    {
        mcu.pc = 0x2a53;
        uint32_t odata = (uint32_t)0x40;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2a53: /* BEQ 58 -> 0x2a8f */
    {
        mcu.pc = 0x2a55;
        uint16_t disp = (uint16_t)(int8_t)0x3a;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a55: /* BCC 8 -> 0x2a5f */
    {
        mcu.pc = 0x2a57;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a57: /* NEG r3 */
    {
        mcu.pc = 0x2a59;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2a59: /* NEG r4 */
    {
        mcu.pc = 0x2a5b;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[4] = (uint16_t)((mcu.r[4] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2a5b: /* BNE 2 -> 0x2a5f */
    {
        mcu.pc = 0x2a5d;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a5d: /* move r4 #0xff */
    {
        mcu.pc = 0x2a5f;
        uint8_t data = 0xff;
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a5f: /* SUB #0x80 r4 */
    {
        mcu.pc = 0x2a62;
        uint32_t odata = (uint32_t)0x80;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2a62: /* BEQ 43 -> 0x2a8f */
    {
        mcu.pc = 0x2a64;
        uint16_t disp = (uint16_t)(int8_t)0x2b;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a64: /* BCC 17 -> 0x2a77 */
    {
        mcu.pc = 0x2a66;
        uint16_t disp = (uint16_t)(int8_t)0x11;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a66: /* NEG r4 */
    {
        mcu.pc = 0x2a68;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[4] = (uint16_t)((mcu.r[4] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2a68: /* MULXU @r3+0x650e r4 */
    {
        mcu.pc = 0x2a6c;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[4];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[4] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a6c: /* ADD r4 r4 */
    {
        mcu.pc = 0x2a6e;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2a6e: /* SWAP r4 */
    {
        mcu.pc = 0x2a70;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a70: /* movi r3 #0x0080 */
    {
        mcu.pc = 0x2a73;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x80;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a73: /* SUB r4 r3 */
    {
        mcu.pc = 0x2a75;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2a75: /* BRA 13 -> 0x2a84 */
    {
        mcu.pc = 0x2a77;
        uint16_t disp = (uint16_t)(int8_t)0x0d;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a77: /* MULXU @r3+0x650e r4 */
    {
        mcu.pc = 0x2a7b;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[4];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[4] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a7b: /* ADD r4 r4 */
    {
        mcu.pc = 0x2a7d;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2a7d: /* SWAP r4 */
    {
        mcu.pc = 0x2a7f;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a7f: /* movi r3 #0x0080 */
    {
        mcu.pc = 0x2a82;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x80;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a82: /* ADD r4 r3 */
    {
        mcu.pc = 0x2a84;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2a84: /* ADD r3 r3 */
    {
        mcu.pc = 0x2a86;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2a86: /* MOVG2 @r3+0x653a r3 */
    {
        mcu.pc = 0x2a8a;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x3a;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x83;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a8a: /* MOVG3 r3 -> @r0+-34 */
    {
        mcu.pc = 0x2a8d;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xde;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2a8d: /* BRA 5 -> 0x2a94 */
    {
        mcu.pc = 0x2a8f;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2a8f: /* MOVG #0x0100 -> @r0+-34 */
    {
        mcu.pc = 0x2a94;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xde;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x01;
        d = (d << 8) | (uint32_t)0x00;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2a94: /* CLR r4 */
    {
        mcu.pc = 0x2a96;
        uint8_t op2 = 0x13;
        mcu.r[4] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a96: /* MOVG2 @r1+0xd134 r4 */
    {
        mcu.pc = 0x2a9a;
        uint32_t odisp = (uint32_t)0xd1;
        odisp = (odisp << 8) | (uint32_t)0x34;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a9a: /* CLR r2 */
    {
        mcu.pc = 0x2a9c;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2a9c: /* MOVG2 @r5+86 r2 */
    {
        mcu.pc = 0x2a9f;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x56;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2a9f: /* ADD r2 r2 */
    {
        mcu.pc = 0x2aa1;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2aa1: /* LDC #0x03 r5 */
    {
        mcu.pc = 0x2aa4;
        uint32_t odata = (uint32_t)0x03;
        uint8_t op2 = 0x8d;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(5, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2aa4: /* MOVG2 @r2+0xddbc r2 */
    {
        mcu.pc = 0x2aa8;
        uint32_t odisp = (uint32_t)0xdd;
        odisp = (odisp << 8) | (uint32_t)0xbc;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x82;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2aa8: /* ADD r4 r2 */
    {
        mcu.pc = 0x2aaa;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2aaa: /* MOVG2 @r2 r4 */
    {
        mcu.pc = 0x2aac;
        uint32_t oea = (uint32_t)mcu.r[2];
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2aac: /* LDC #0x00 r5 */
    {
        mcu.pc = 0x2aaf;
        uint32_t odata = (uint32_t)0x00;
        uint8_t op2 = 0x8d;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(5, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2aaf: /* CLR r3 */
    {
        mcu.pc = 0x2ab1;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2ab1: /* MOVG2 @r5+88 r3 */
    {
        mcu.pc = 0x2ab4;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x58;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2ab4: /* SUB #0x40 r3 */
    {
        mcu.pc = 0x2ab7;
        uint32_t odata = (uint32_t)0x40;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2ab7: /* BEQ 58 -> 0x2af3 */
    {
        mcu.pc = 0x2ab9;
        uint16_t disp = (uint16_t)(int8_t)0x3a;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2ab9: /* BCC 8 -> 0x2ac3 */
    {
        mcu.pc = 0x2abb;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2abb: /* NEG r3 */
    {
        mcu.pc = 0x2abd;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2abd: /* NEG r4 */
    {
        mcu.pc = 0x2abf;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[4] = (uint16_t)((mcu.r[4] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2abf: /* BNE 2 -> 0x2ac3 */
    {
        mcu.pc = 0x2ac1;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2ac1: /* move r4 #0xff */
    {
        mcu.pc = 0x2ac3;
        uint8_t data = 0xff;
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2ac3: /* SUB #0x80 r4 */
    {
        mcu.pc = 0x2ac6;
        uint32_t odata = (uint32_t)0x80;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2ac6: /* BEQ 43 -> 0x2af3 */
    {
        mcu.pc = 0x2ac8;
        uint16_t disp = (uint16_t)(int8_t)0x2b;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2ac8: /* BCC 17 -> 0x2adb */
    {
        mcu.pc = 0x2aca;
        uint16_t disp = (uint16_t)(int8_t)0x11;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2aca: /* NEG r4 */
    {
        mcu.pc = 0x2acc;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[4] = (uint16_t)((mcu.r[4] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2acc: /* MULXU @r3+0x650e r4 */
    {
        mcu.pc = 0x2ad0;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[4];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[4] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2ad0: /* ADD r4 r4 */
    {
        mcu.pc = 0x2ad2;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2ad2: /* SWAP r4 */
    {
        mcu.pc = 0x2ad4;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ad4: /* movi r3 #0x0080 */
    {
        mcu.pc = 0x2ad7;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x80;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ad7: /* SUB r4 r3 */
    {
        mcu.pc = 0x2ad9;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2ad9: /* BRA 13 -> 0x2ae8 */
    {
        mcu.pc = 0x2adb;
        uint16_t disp = (uint16_t)(int8_t)0x0d;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2adb: /* MULXU @r3+0x650e r4 */
    {
        mcu.pc = 0x2adf;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[4];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[4] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2adf: /* ADD r4 r4 */
    {
        mcu.pc = 0x2ae1;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2ae1: /* SWAP r4 */
    {
        mcu.pc = 0x2ae3;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ae3: /* movi r3 #0x0080 */
    {
        mcu.pc = 0x2ae6;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x80;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ae6: /* ADD r4 r3 */
    {
        mcu.pc = 0x2ae8;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2ae8: /* ADD r3 r3 */
    {
        mcu.pc = 0x2aea;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2aea: /* MOVG2 @r3+0x653a r3 */
    {
        mcu.pc = 0x2aee;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x3a;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x83;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2aee: /* MOVG3 r3 -> @r0+-32 */
    {
        mcu.pc = 0x2af1;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2af1: /* BRA 5 -> 0x2af8 */
    {
        mcu.pc = 0x2af3;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2af3: /* MOVG #0x0100 -> @r0+-32 */
    {
        mcu.pc = 0x2af8;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe0;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x01;
        d = (d << 8) | (uint32_t)0x00;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2af8: /* movi r2 #0x007f */
    {
        mcu.pc = 0x2afb;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x7f;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2afb: /* SUB @r1+0xcee8 r2 */
    {
        mcu.pc = 0x2aff;
        uint32_t odisp = (uint32_t)0xce;
        odisp = (odisp << 8) | (uint32_t)0xe8;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2aff: /* CLR r3 */
    {
        mcu.pc = 0x2b01;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2b01: /* MOVG2 @r5+89 r3 */
    {
        mcu.pc = 0x2b04;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x59;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2b04: /* SUB #0x40 r3 */
    {
        mcu.pc = 0x2b07;
        uint32_t odata = (uint32_t)0x40;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2b07: /* BEQ 62 -> 0x2b47 */
    {
        mcu.pc = 0x2b09;
        uint16_t disp = (uint16_t)(int8_t)0x3e;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b09: /* BCC 31 -> 0x2b2a */
    {
        mcu.pc = 0x2b0b;
        uint16_t disp = (uint16_t)(int8_t)0x1f;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b0b: /* NEG r3 */
    {
        mcu.pc = 0x2b0d;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2b0d: /* MULXU @r3+0x650e r2 */
    {
        mcu.pc = 0x2b11;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xaa;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[2];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[2] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2b11: /* cmp r2,w #0x1f41 */
    {
        mcu.pc = 0x2b14;
        int32_t t2 = (int32_t)0x1f;
        t2 = (t2 << 8) | (int32_t)0x41;
        int32_t t1 = (int32_t)mcu.r[2];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x2b14: /* BCS 5 -> 0x2b1b */
    {
        mcu.pc = 0x2b16;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b16: /* movi r3 #0x0004 */
    {
        mcu.pc = 0x2b19;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x04;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b19: /* BRA 47 -> 0x2b4a */
    {
        mcu.pc = 0x2b1b;
        uint16_t disp = (uint16_t)(int8_t)0x2f;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b1b: /* movi r3 #0x1fc0 */
    {
        mcu.pc = 0x2b1e;
        uint16_t data = (uint16_t)(0x1f << 8);
        data |= 0xc0;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b1e: /* SUB r2 r3 */
    {
        mcu.pc = 0x2b20;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2b20: /* MOVG2 r3 r2 */
    {
        mcu.pc = 0x2b22;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[3];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b22: /* MULXU #0x0810 r2:r3 */
    {
        mcu.pc = 0x2b26;
        uint32_t odata = (uint32_t)0x08;
        odata = (odata << 8) | (uint32_t)0x10;
        uint8_t op2 = 0xaa;
        uint32_t t1 = odata;
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 *= t2;
        mcu.r[2] = (uint16_t)(t1 >> 16);
        mcu.r[3] = (uint16_t)t1;
        uint32_t N = (t1 & 0x80000000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2b26: /* MOVG2 r2 r3 */
    {
        mcu.pc = 0x2b28;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)mcu.r[2];
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b28: /* BRA 32 -> 0x2b4a */
    {
        mcu.pc = 0x2b2a;
        uint16_t disp = (uint16_t)(int8_t)0x20;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b2a: /* movi r4 #0x1fc0 */
    {
        mcu.pc = 0x2b2d;
        uint16_t data = (uint16_t)(0x1f << 8);
        data |= 0xc0;
        mcu.r[4] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b2d: /* MULXU @r3+0x650e r2 */
    {
        mcu.pc = 0x2b31;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xaa;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[2];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[2] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2b31: /* SUB r2 r4 */
    {
        mcu.pc = 0x2b33;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2b33: /* cmp r4,w #0x0020 */
    {
        mcu.pc = 0x2b36;
        int32_t t2 = (int32_t)0x00;
        t2 = (t2 << 8) | (int32_t)0x20;
        int32_t t1 = (int32_t)mcu.r[4];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x2b36: /* BCS 10 -> 0x2b42 */
    {
        mcu.pc = 0x2b38;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b38: /* movi r2 #0x001f */
    {
        mcu.pc = 0x2b3b;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x1f;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b3b: /* movi r3 #0xc000 */
    {
        mcu.pc = 0x2b3e;
        uint16_t data = (uint16_t)(0xc0 << 8);
        data |= 0x00;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b3e: /* DIVXU r4 r2:r3 */
    {
        mcu.pc = 0x2b40;
        uint8_t op2 = 0xba;
        uint32_t t1 = (uint32_t)mcu.r[4];
        uint32_t t2 = 0;
        uint32_t R = 0, Q = 0;
        if (t1 == 0)
        {
        MCU_ErrorTrap();
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        break;
        }
        t2 = ((uint32_t)mcu.r[2] << 16) | (uint32_t)mcu.r[3];
        R = t2 % t1;
        Q = t2 / t1;
        if (Q > 0xffffu)
        {
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(0, STATUS_Z);
        MCU_SetStatus(1, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        }
        else
        {
        mcu.r[2] = (uint16_t)R;
        mcu.r[3] = (uint16_t)Q;
        MCU_SetStatusCommon(Q, 1);
        MCU_SetStatus(0, STATUS_C);
        }
    }
    break;
    case 0x2b40: /* BRA 8 -> 0x2b4a */
    {
        mcu.pc = 0x2b42;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b42: /* movi r3 #0xffff */
    {
        mcu.pc = 0x2b45;
        uint16_t data = (uint16_t)(0xff << 8);
        data |= 0xff;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b45: /* BRA 3 -> 0x2b4a */
    {
        mcu.pc = 0x2b47;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b47: /* movi r3 #0x0100 */
    {
        mcu.pc = 0x2b4a;
        uint16_t data = (uint16_t)(0x01 << 8);
        data |= 0x00;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b4a: /* MOVG3 r3 -> @r0+-30 */
    {
        mcu.pc = 0x2b4d;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b4d: /* movi r2 #0x007f */
    {
        mcu.pc = 0x2b50;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x7f;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b50: /* SUB @r1+0xcee8 r2 */
    {
        mcu.pc = 0x2b54;
        uint32_t odisp = (uint32_t)0xce;
        odisp = (odisp << 8) | (uint32_t)0xe8;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2b54: /* CLR r3 */
    {
        mcu.pc = 0x2b56;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2b56: /* MOVG2 @r5+90 r3 */
    {
        mcu.pc = 0x2b59;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x5a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2b59: /* SUB #0x40 r3 */
    {
        mcu.pc = 0x2b5c;
        uint32_t odata = (uint32_t)0x40;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2b5c: /* BEQ 62 -> 0x2b9c */
    {
        mcu.pc = 0x2b5e;
        uint16_t disp = (uint16_t)(int8_t)0x3e;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b5e: /* BCC 31 -> 0x2b7f */
    {
        mcu.pc = 0x2b60;
        uint16_t disp = (uint16_t)(int8_t)0x1f;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b60: /* NEG r3 */
    {
        mcu.pc = 0x2b62;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;
    case 0x2b62: /* MULXU @r3+0x650e r2 */
    {
        mcu.pc = 0x2b66;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xaa;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[2];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[2] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2b66: /* cmp r2,w #0x1f41 */
    {
        mcu.pc = 0x2b69;
        int32_t t2 = (int32_t)0x1f;
        t2 = (t2 << 8) | (int32_t)0x41;
        int32_t t1 = (int32_t)mcu.r[2];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x2b69: /* BCS 5 -> 0x2b70 */
    {
        mcu.pc = 0x2b6b;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b6b: /* movi r3 #0x0004 */
    {
        mcu.pc = 0x2b6e;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x04;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b6e: /* BRA 47 -> 0x2b9f */
    {
        mcu.pc = 0x2b70;
        uint16_t disp = (uint16_t)(int8_t)0x2f;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b70: /* movi r3 #0x1fc0 */
    {
        mcu.pc = 0x2b73;
        uint16_t data = (uint16_t)(0x1f << 8);
        data |= 0xc0;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b73: /* SUB r2 r3 */
    {
        mcu.pc = 0x2b75;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2b75: /* MOVG2 r3 r2 */
    {
        mcu.pc = 0x2b77;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[3];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b77: /* MULXU #0x0810 r2:r3 */
    {
        mcu.pc = 0x2b7b;
        uint32_t odata = (uint32_t)0x08;
        odata = (odata << 8) | (uint32_t)0x10;
        uint8_t op2 = 0xaa;
        uint32_t t1 = odata;
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 *= t2;
        mcu.r[2] = (uint16_t)(t1 >> 16);
        mcu.r[3] = (uint16_t)t1;
        uint32_t N = (t1 & 0x80000000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2b7b: /* MOVG2 r2 r3 */
    {
        mcu.pc = 0x2b7d;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)mcu.r[2];
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b7d: /* BRA 32 -> 0x2b9f */
    {
        mcu.pc = 0x2b7f;
        uint16_t disp = (uint16_t)(int8_t)0x20;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b7f: /* movi r4 #0x1fc0 */
    {
        mcu.pc = 0x2b82;
        uint16_t data = (uint16_t)(0x1f << 8);
        data |= 0xc0;
        mcu.r[4] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b82: /* MULXU @r3+0x650e r2 */
    {
        mcu.pc = 0x2b86;
        uint32_t odisp = (uint32_t)0x65;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xaa;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[2];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[2] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2b86: /* SUB r2 r4 */
    {
        mcu.pc = 0x2b88;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2b88: /* cmp r4,w #0x0020 */
    {
        mcu.pc = 0x2b8b;
        int32_t t2 = (int32_t)0x00;
        t2 = (t2 << 8) | (int32_t)0x20;
        int32_t t1 = (int32_t)mcu.r[4];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x2b8b: /* BCS 10 -> 0x2b97 */
    {
        mcu.pc = 0x2b8d;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b8d: /* movi r2 #0x001f */
    {
        mcu.pc = 0x2b90;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x1f;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b90: /* movi r3 #0xc000 */
    {
        mcu.pc = 0x2b93;
        uint16_t data = (uint16_t)(0xc0 << 8);
        data |= 0x00;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b93: /* DIVXU r4 r2:r3 */
    {
        mcu.pc = 0x2b95;
        uint8_t op2 = 0xba;
        uint32_t t1 = (uint32_t)mcu.r[4];
        uint32_t t2 = 0;
        uint32_t R = 0, Q = 0;
        if (t1 == 0)
        {
        MCU_ErrorTrap();
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        break;
        }
        t2 = ((uint32_t)mcu.r[2] << 16) | (uint32_t)mcu.r[3];
        R = t2 % t1;
        Q = t2 / t1;
        if (Q > 0xffffu)
        {
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(0, STATUS_Z);
        MCU_SetStatus(1, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        }
        else
        {
        mcu.r[2] = (uint16_t)R;
        mcu.r[3] = (uint16_t)Q;
        MCU_SetStatusCommon(Q, 1);
        MCU_SetStatus(0, STATUS_C);
        }
    }
    break;
    case 0x2b95: /* BRA 8 -> 0x2b9f */
    {
        mcu.pc = 0x2b97;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b97: /* movi r3 #0xffff */
    {
        mcu.pc = 0x2b9a;
        uint16_t data = (uint16_t)(0xff << 8);
        data |= 0xff;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b9a: /* BRA 3 -> 0x2b9f */
    {
        mcu.pc = 0x2b9c;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2b9c: /* movi r3 #0x0100 */
    {
        mcu.pc = 0x2b9f;
        uint16_t data = (uint16_t)(0x01 << 8);
        data |= 0x00;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2b9f: /* MOVG3 r3 -> @r0+-28 */
    {
        mcu.pc = 0x2ba2;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe4;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ba2: /* MOVG2 @r5+78 r2 */
    {
        mcu.pc = 0x2ba5;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x4e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2ba5: /* BTSTI r2 #7 */
    {
        mcu.pc = 0x2ba7;
        uint8_t op2 = 0xf7;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        uint32_t bit = 7;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x2ba7: /* BEQ 6 -> 0x2baf */
    {
        mcu.pc = 0x2ba9;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2ba9: /* MOVG #0x00 -> @r0+-8 */
    {
        mcu.pc = 0x2bad;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x00;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2bad: /* BRA 4 -> 0x2bb3 */
    {
        mcu.pc = 0x2baf;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2baf: /* MOVG #0x04 -> @r0+-8 */
    {
        mcu.pc = 0x2bb3;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x04;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2bb3: /* AND #0x007f r2 */
    {
        mcu.pc = 0x2bb7;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x7f;
        uint8_t op2 = 0x52;
        uint32_t data = (uint32_t)mcu.r[2];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[2], 1);
    }
    break;
    case 0x2bb7: /* MOVG3 r2 -> @r0+79 */
    {
        mcu.pc = 0x2bba;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x4f;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2bba: /* MOVG2 @r5+79 r2 */
    {
        mcu.pc = 0x2bbd;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x4f;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2bbd: /* BTSTI r2 #7 */
    {
        mcu.pc = 0x2bbf;
        uint8_t op2 = 0xf7;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        uint32_t bit = 7;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x2bbf: /* BEQ 6 -> 0x2bc7 */
    {
        mcu.pc = 0x2bc1;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2bc1: /* MOVG #0x00 -> @r0+-7 */
    {
        mcu.pc = 0x2bc5;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf9;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x00;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2bc5: /* BRA 4 -> 0x2bcb */
    {
        mcu.pc = 0x2bc7;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2bc7: /* MOVG #0x04 -> @r0+-7 */
    {
        mcu.pc = 0x2bcb;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf9;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x04;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2bcb: /* AND #0x007f r2 */
    {
        mcu.pc = 0x2bcf;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x7f;
        uint8_t op2 = 0x52;
        uint32_t data = (uint32_t)mcu.r[2];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[2], 1);
    }
    break;
    case 0x2bcf: /* MOVG3 r2 -> @r0+80 */
    {
        mcu.pc = 0x2bd2;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x50;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2bd2: /* MOVG2 @r5+80 r2 */
    {
        mcu.pc = 0x2bd5;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x50;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2bd5: /* BTSTI r2 #7 */
    {
        mcu.pc = 0x2bd7;
        uint8_t op2 = 0xf7;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        uint32_t bit = 7;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x2bd7: /* BEQ 6 -> 0x2bdf */
    {
        mcu.pc = 0x2bd9;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2bd9: /* MOVG #0x00 -> @r0+-6 */
    {
        mcu.pc = 0x2bdd;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfa;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x00;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2bdd: /* BRA 4 -> 0x2be3 */
    {
        mcu.pc = 0x2bdf;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2bdf: /* MOVG #0x04 -> @r0+-6 */
    {
        mcu.pc = 0x2be3;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfa;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x04;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2be3: /* AND #0x007f r2 */
    {
        mcu.pc = 0x2be7;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x7f;
        uint8_t op2 = 0x52;
        uint32_t data = (uint32_t)mcu.r[2];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[2], 1);
    }
    break;
    case 0x2be7: /* MOVG3 r2 -> @r0+81 */
    {
        mcu.pc = 0x2bea;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x51;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2bea: /* MOVG2 @r5+81 r2 */
    {
        mcu.pc = 0x2bed;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x51;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2bed: /* BTSTI r2 #7 */
    {
        mcu.pc = 0x2bef;
        uint8_t op2 = 0xf7;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        uint32_t bit = 7;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x2bef: /* BEQ 6 -> 0x2bf7 */
    {
        mcu.pc = 0x2bf1;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2bf1: /* MOVG #0x00 -> @r0+-5 */
    {
        mcu.pc = 0x2bf5;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfb;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x00;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2bf5: /* BRA 4 -> 0x2bfb */
    {
        mcu.pc = 0x2bf7;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2bf7: /* MOVG #0x04 -> @r0+-5 */
    {
        mcu.pc = 0x2bfb;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfb;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x04;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2bfb: /* AND #0x007f r2 */
    {
        mcu.pc = 0x2bff;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x7f;
        uint8_t op2 = 0x52;
        uint32_t data = (uint32_t)mcu.r[2];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[2], 1);
    }
    break;
    case 0x2bff: /* MOVG3 r2 -> @r0+82 */
    {
        mcu.pc = 0x2c02;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x52;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2c02: /* MOVG2 @r5+82 r2 */
    {
        mcu.pc = 0x2c05;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x52;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2c05: /* BTSTI r2 #7 */
    {
        mcu.pc = 0x2c07;
        uint8_t op2 = 0xf7;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        uint32_t bit = 7;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    }
    break;
    case 0x2c07: /* BEQ 6 -> 0x2c0f */
    {
        mcu.pc = 0x2c09;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2c09: /* MOVG #0x00 -> @r0+-4 */
    {
        mcu.pc = 0x2c0d;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfc;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x00;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2c0d: /* BRA 4 -> 0x2c13 */
    {
        mcu.pc = 0x2c0f;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2c0f: /* MOVG #0x04 -> @r0+-4 */
    {
        mcu.pc = 0x2c13;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfc;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x04;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2c13: /* AND #0x007f r2 */
    {
        mcu.pc = 0x2c17;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x7f;
        uint8_t op2 = 0x52;
        uint32_t data = (uint32_t)mcu.r[2];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[2], 1);
    }
    break;
    case 0x2c17: /* MOVG3 r2 -> @r0+83 */
    {
        mcu.pc = 0x2c1a;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x53;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2c1a: /* LDC @r0+0x0098 r4 */
    {
        mcu.pc = 0x2c1e;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x98;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x8c;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2c1e: /* MOVG2 @r0+0x009c r5 */
    {
        mcu.pc = 0x2c22;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9c;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c22: /* CLR r3 */
    {
        mcu.pc = 0x2c24;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c24: /* MOVG2 @r5+31 r3 */
    {
        mcu.pc = 0x2c27;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x1f;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2c27: /* LDC @r0+0x0099 r4 */
    {
        mcu.pc = 0x2c2b;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x99;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x8c;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2c2b: /* MOVG2 @r0+0x009e r5 */
    {
        mcu.pc = 0x2c2f;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x9e;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c2f: /* AND #0x03 r3 */
    {
        mcu.pc = 0x2c32;
        uint32_t odata = (uint32_t)0x03;
        uint8_t op2 = 0x53;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(mcu.r[3], 0);
    }
    break;
    case 0x2c32: /* BEQ 27 -> 0x2c4f */
    {
        mcu.pc = 0x2c34;
        uint16_t disp = (uint16_t)(int8_t)0x1b;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2c34: /* CLR r2 */
    {
        mcu.pc = 0x2c36;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c36: /* MOVG2 @r1+0xd134 r2 */
    {
        mcu.pc = 0x2c3a;
        uint32_t odisp = (uint32_t)0xd1;
        odisp = (odisp << 8) | (uint32_t)0x34;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2c3a: /* LDC #0x03 r5 */
    {
        mcu.pc = 0x2c3d;
        uint32_t odata = (uint32_t)0x03;
        uint8_t op2 = 0x8d;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(5, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2c3d: /* SHLL r3 */
    {
        mcu.pc = 0x2c3f;
        uint8_t op2 = 0x1a;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t C = (data & 0x8000u) != 0;
        data <<= 1;
        mcu.r[3] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c3f: /* ADDS @r3+0xf80c r2 */
    {
        mcu.pc = 0x2c43;
        uint32_t odisp = (uint32_t)0xf8;
        odisp = (odisp << 8) | (uint32_t)0x0c;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x2a;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] += (uint16_t)data;
    }
    break;
    case 0x2c43: /* MOVG2 @r2 r2 */
    {
        mcu.pc = 0x2c45;
        uint32_t oea = (uint32_t)mcu.r[2];
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2c45: /* LDC #0x00 r5 */
    {
        mcu.pc = 0x2c48;
        uint32_t odata = (uint32_t)0x00;
        uint8_t op2 = 0x8d;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(5, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2c48: /* EXTU r2 */
    {
        mcu.pc = 0x2c4a;
        uint8_t op2 = 0x12;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c4a: /* MOVG3 r2 -> @r0+56 */
    {
        mcu.pc = 0x2c4d;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x38;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c4d: /* BRA 8 -> 0x2c57 */
    {
        mcu.pc = 0x2c4f;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2c4f: /* CLR r3 */
    {
        mcu.pc = 0x2c51;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c51: /* MOVG2 @r5+9 r3 */
    {
        mcu.pc = 0x2c54;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x09;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2c54: /* MOVG3 r3 -> @r0+56 */
    {
        mcu.pc = 0x2c57;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x38;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c57: /* MOVG2 @r5+0 r3 */
    {
        mcu.pc = 0x2c5a;
        uint32_t oea = (uint32_t)mcu.r[5] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(5) & 0xff);
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2c5a: /* BPL 4 -> 0x2c60 */
    {
        mcu.pc = 0x2c5c;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2c5c: /* CLR r3 */
    {
        mcu.pc = 0x2c5e;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c5e: /* BRA 35 -> 0x2c83 */
    {
        mcu.pc = 0x2c60;
        uint16_t disp = (uint16_t)(int8_t)0x23;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2c60: /* ADD r3 r3 */
    {
        mcu.pc = 0x2c62;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2c62: /* MOVG2 @r3+0x6c86 r4 */
    {
        mcu.pc = 0x2c66;
        uint32_t odisp = (uint32_t)0x6c;
        odisp = (odisp << 8) | (uint32_t)0x86;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x84;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c66: /* ADD #0x0004 r4 */
    {
        mcu.pc = 0x2c6a;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x04;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = odata;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2c6a: /* SHLR r4 */
    {
        mcu.pc = 0x2c6c;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c6c: /* SHLR r4 */
    {
        mcu.pc = 0x2c6e;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c6e: /* SHLR r4 */
    {
        mcu.pc = 0x2c70;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c70: /* BNE 10 -> 0x2c7c */
    {
        mcu.pc = 0x2c72;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2c72: /* MOVG #0xffff -> @r0+14 */
    {
        mcu.pc = 0x2c77;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x0e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0xff;
        d = (d << 8) | (uint32_t)0xff;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2c77: /* CLR @r0+16 */
    {
        mcu.pc = 0x2c7a;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x10;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c7a: /* BRA 22 -> 0x2c92 */
    {
        mcu.pc = 0x2c7c;
        uint16_t disp = (uint16_t)(int8_t)0x16;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2c7c: /* movi r3 #0xffff */
    {
        mcu.pc = 0x2c7f;
        uint16_t data = (uint16_t)(0xff << 8);
        data |= 0xff;
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c7f: /* CLR r2 */
    {
        mcu.pc = 0x2c81;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c81: /* DIVXU r4 r2:r3 */
    {
        mcu.pc = 0x2c83;
        uint8_t op2 = 0xba;
        uint32_t t1 = (uint32_t)mcu.r[4];
        uint32_t t2 = 0;
        uint32_t R = 0, Q = 0;
        if (t1 == 0)
        {
        MCU_ErrorTrap();
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        break;
        }
        t2 = ((uint32_t)mcu.r[2] << 16) | (uint32_t)mcu.r[3];
        R = t2 % t1;
        Q = t2 / t1;
        if (Q > 0xffffu)
        {
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(0, STATUS_Z);
        MCU_SetStatus(1, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        }
        else
        {
        mcu.r[2] = (uint16_t)R;
        mcu.r[3] = (uint16_t)Q;
        MCU_SetStatusCommon(Q, 1);
        MCU_SetStatus(0, STATUS_C);
        }
    }
    break;
    case 0x2c83: /* CLR @r0+14 */
    {
        mcu.pc = 0x2c86;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x0e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c86: /* MOVG3 r3 -> @r0+16 */
    {
        mcu.pc = 0x2c89;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x10;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2c89: /* CLR @r0+24 */
    {
        mcu.pc = 0x2c8c;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x18;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c8c: /* CLR @r0+28 */
    {
        mcu.pc = 0x2c8f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c8f: /* CLR @r0+36 */
    {
        mcu.pc = 0x2c92;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x24;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c92: /* CLR @r0+18 */
    {
        mcu.pc = 0x2c95;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x12;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c95: /* CLR @r0+8 */
    {
        mcu.pc = 0x2c98;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x08;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c98: /* CLR @r0+24 */
    {
        mcu.pc = 0x2c9b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x18;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c9b: /* CLR @r0+28 */
    {
        mcu.pc = 0x2c9e;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x13;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2c9e: /* MOVG2 (dp,0xad2a) r6 */
    {
        mcu.pc = 0x2ca2;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ca2: /* MOVG3 r6 -> (dp,0xad2c) */
    {
        mcu.pc = 0x2ca6;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ca6: /* MOVG #0x0001 -> (dp,0xad2a) */
    {
        mcu.pc = 0x2cac;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x01;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2cac: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2caf;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2caf: /* bsr16 -> 0x31ca */
    {
        mcu.pc = 0x2cb2;
        uint16_t disp = (uint16_t)(0x05 << 8);
        disp |= 0x18;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2cb2: /* MOVG2 @r0+30 r2 */
    {
        mcu.pc = 0x2cb5;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x82;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cb5: /* cmp r2,b #0xaf */
    {
        mcu.pc = 0x2cb7;
        int32_t t2 = (int32_t)0xaf;
        int32_t t1 = (int32_t)mcu.r[2];
        MCU_SUB_Common(t1, t2, 0, 0);
    }
    break;
    case 0x2cb7: /* BNE 5 -> 0x2cbe */
    {
        mcu.pc = 0x2cb9;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2cb9: /* move r2 #0xba */
    {
        mcu.pc = 0x2cbb;
        uint8_t data = 0xba;
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2cbb: /* MOVG3 r2 -> @r0+30 */
    {
        mcu.pc = 0x2cbe;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cbe: /* MOVG2 (dp,0xad2c) r6 */
    {
        mcu.pc = 0x2cc2;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x86;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cc2: /* MOVG3 r6 -> (dp,0xad2a) */
    {
        mcu.pc = 0x2cc6;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x96;
        uint32_t data = (uint32_t)mcu.r[6];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cc6: /* MOVG2 @r0+46 r2 */
    {
        mcu.pc = 0x2cc9;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x2e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x82;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cc9: /* MOVG2 @r0+48 r3 */
    {
        mcu.pc = 0x2ccc;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x30;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x83;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ccc: /* MOVG2 @r2+14 r4 */
    {
        mcu.pc = 0x2ccf;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0x0e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2ccf: /* MOVG2 @r2+15 r5 */
    {
        mcu.pc = 0x2cd2;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0x0f;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2cd2: /* TST r3 */
    {
        mcu.pc = 0x2cd4;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2cd4: /* BEQ 24 -> 0x2cee */
    {
        mcu.pc = 0x2cd6;
        uint16_t disp = (uint16_t)(int8_t)0x18;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2cd6: /* MULXU @r3+0x0380 r4 */
    {
        mcu.pc = 0x2cda;
        uint32_t odisp = (uint32_t)0x03;
        odisp = (odisp << 8) | (uint32_t)0x80;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[4];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[4] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2cda: /* MULXU @r3+0x0300 r5 */
    {
        mcu.pc = 0x2cde;
        uint32_t odisp = (uint32_t)0x03;
        odisp = (odisp << 8) | (uint32_t)0x00;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xad;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)mcu.r[5];
        t2 &= 0xff;
        t1 *= t2;
        t1 &= 0xffff;
        mcu.r[5] = (uint16_t)t1;
        uint32_t N = (t1 & 0x8000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2cde: /* ADD r4 r4 */
    {
        mcu.pc = 0x2ce0;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2ce0: /* ADD r5 r5 */
    {
        mcu.pc = 0x2ce2;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[5];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;
    case 0x2ce2: /* ADD #0x00ff r4 */
    {
        mcu.pc = 0x2ce6;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0xff;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = odata;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2ce6: /* ADD #0x00ff r5 */
    {
        mcu.pc = 0x2cea;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0xff;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = odata;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;
    case 0x2cea: /* SWAP r4 */
    {
        mcu.pc = 0x2cec;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cec: /* SWAP r5 */
    {
        mcu.pc = 0x2cee;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cee: /* SWAP r5 */
    {
        mcu.pc = 0x2cf0;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cf0: /* MOVG2 r4 r5 */
    {
        mcu.pc = 0x2cf2;
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2cf2: /* MOVG3 r5 -> @r0+58 */
    {
        mcu.pc = 0x2cf5;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x3a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2cf5: /* MOVG2 @r2+9 r2 */
    {
        mcu.pc = 0x2cf8;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0x09;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2cf8: /* BEQ 81 -> 0x2d4b */
    {
        mcu.pc = 0x2cfa;
        uint16_t disp = (uint16_t)(int8_t)0x51;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2cfa: /* TST r3 */
    {
        mcu.pc = 0x2cfc;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2cfc: /* BEQ 25 -> 0x2d17 */
    {
        mcu.pc = 0x2cfe;
        uint16_t disp = (uint16_t)(int8_t)0x19;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2cfe: /* MOVG2 @r3+0x0280 r5 */
    {
        mcu.pc = 0x2d02;
        uint32_t odisp = (uint32_t)0x02;
        odisp = (odisp << 8) | (uint32_t)0x80;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d02: /* BEQ 71 -> 0x2d4b */
    {
        mcu.pc = 0x2d04;
        uint16_t disp = (uint16_t)(int8_t)0x47;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d04: /* SUB #0x40 r5 */
    {
        mcu.pc = 0x2d07;
        uint32_t odata = (uint32_t)0x40;
        uint8_t op2 = 0x35;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d07: /* BCC 8 -> 0x2d11 */
    {
        mcu.pc = 0x2d09;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d09: /* ADD r5 r2 */
    {
        mcu.pc = 0x2d0b;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[5] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d0b: /* BPL 10 -> 0x2d17 */
    {
        mcu.pc = 0x2d0d;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d0d: /* CLR r2 */
    {
        mcu.pc = 0x2d0f;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(0) & 0xffu));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2d0f: /* BRA 6 -> 0x2d17 */
    {
        mcu.pc = 0x2d11;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d11: /* ADD r5 r2 */
    {
        mcu.pc = 0x2d13;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[5] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d13: /* BVC 2 -> 0x2d17 */
    {
        mcu.pc = 0x2d15;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t V = (mcu.sr & STATUS_V) != 0;
        uint32_t branch = V == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d15: /* move r2 #0x7f */
    {
        mcu.pc = 0x2d17;
        uint8_t data = 0x7f;
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d17: /* MOVG2 @r0+56 r3 */
    {
        mcu.pc = 0x2d1a;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x38;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x83;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d1a: /* SUB #0x40 r2 */
    {
        mcu.pc = 0x2d1d;
        uint32_t odata = (uint32_t)0x40;
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d1d: /* BCC 8 -> 0x2d27 */
    {
        mcu.pc = 0x2d1f;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d1f: /* ADD r2 r3 */
    {
        mcu.pc = 0x2d21;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[2] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d21: /* BPL 10 -> 0x2d2d */
    {
        mcu.pc = 0x2d23;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d23: /* CLR r3 */
    {
        mcu.pc = 0x2d25;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(0) & 0xffu));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2d25: /* BRA 6 -> 0x2d2d */
    {
        mcu.pc = 0x2d27;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d27: /* ADD r2 r3 */
    {
        mcu.pc = 0x2d29;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[2] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d29: /* BVC 2 -> 0x2d2d */
    {
        mcu.pc = 0x2d2b;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t V = (mcu.sr & STATUS_V) != 0;
        uint32_t branch = V == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d2b: /* move r3 #0x7f */
    {
        mcu.pc = 0x2d2d;
        uint8_t data = 0x7f;
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d2d: /* MOVG2 (dp,0x8006) r2 */
    {
        mcu.pc = 0x2d31;
        uint32_t oea = (uint32_t)0x80;
        oea = (oea << 8) | (uint32_t)0x06;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d31: /* BEQ 19 -> 0x2d46 */
    {
        mcu.pc = 0x2d33;
        uint16_t disp = (uint16_t)(int8_t)0x13;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d33: /* SUB #0x40 r2 */
    {
        mcu.pc = 0x2d36;
        uint32_t odata = (uint32_t)0x40;
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d36: /* BCC 8 -> 0x2d40 */
    {
        mcu.pc = 0x2d38;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d38: /* ADD r2 r3 */
    {
        mcu.pc = 0x2d3a;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[2] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d3a: /* BPL 10 -> 0x2d46 */
    {
        mcu.pc = 0x2d3c;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d3c: /* CLR r3 */
    {
        mcu.pc = 0x2d3e;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(0) & 0xffu));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2d3e: /* BRA 6 -> 0x2d46 */
    {
        mcu.pc = 0x2d40;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d40: /* ADD r2 r3 */
    {
        mcu.pc = 0x2d42;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[2] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d42: /* BVC 2 -> 0x2d46 */
    {
        mcu.pc = 0x2d44;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t V = (mcu.sr & STATUS_V) != 0;
        uint32_t branch = V == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d44: /* move r3 #0x7f */
    {
        mcu.pc = 0x2d46;
        uint8_t data = 0x7f;
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d46: /* MOVG3 r3 -> @r0+54 */
    {
        mcu.pc = 0x2d49;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x36;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d49: /* BRA 27 -> 0x2d66 */
    {
        mcu.pc = 0x2d4b;
        uint16_t disp = (uint16_t)(int8_t)0x1b;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d4b: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x2d4f;
        uint32_t odata = (uint32_t)0x07;
        odata = (odata << 8) | (uint32_t)0x00;
        uint8_t op2 = 0x48;
        uint32_t data = odata;
        uint32_t val = MCU_ControlRegisterRead(0, 1);
        val |= data;
        MCU_ControlRegisterWrite(0, 1, val);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2d4f: /* MOVG #0x1e -> (br,$3e) */
    {
        mcu.pc = 0x2d53;
        uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x3e;
        oea &= 0xffff;
        uint32_t oep = 0;
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x1e;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2d53: /* movl r3 @(br,$34) */
    {
        mcu.pc = 0x2d55;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x34;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d55: /* movlw r3 @(br,$3a) */
    {
        mcu.pc = 0x2d57;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3a;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint16_t data = MCU_Read16(addr);
        mcu.r[3] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d57: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x2d5b;
        uint32_t odata = (uint32_t)0xf8;
        odata = (odata << 8) | (uint32_t)0xff;
        uint8_t op2 = 0x58;
        uint32_t data = odata;
        uint32_t val = MCU_ControlRegisterRead(0, 1);
        val &= data;
        MCU_ControlRegisterWrite(0, 1, val);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x2d5b: /* CLR r3 */
    {
        mcu.pc = 0x2d5d;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(0) & 0xffu));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2d5d: /* SWAP r3 */
    {
        mcu.pc = 0x2d5f;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d5f: /* SHLR r3 */
    {
        mcu.pc = 0x2d61;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[3] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d61: /* MOVG #0xffff -> @r0+54 */
    {
        mcu.pc = 0x2d66;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x36;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0xff;
        d = (d << 8) | (uint32_t)0xff;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2d66: /* MOVG2 @r3+0x6a03 r4 */
    {
        mcu.pc = 0x2d6a;
        uint32_t odisp = (uint32_t)0x6a;
        odisp = (odisp << 8) | (uint32_t)0x03;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d6a: /* movi r2 #0x0080 */
    {
        mcu.pc = 0x2d6d;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x80;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d6d: /* SUB r3 r2 */
    {
        mcu.pc = 0x2d6f;
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x2d6f: /* MOVG2 @r2+0x6a03 r5 */
    {
        mcu.pc = 0x2d73;
        uint32_t odisp = (uint32_t)0x6a;
        odisp = (odisp << 8) | (uint32_t)0x03;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d73: /* SWAP r5 */
    {
        mcu.pc = 0x2d75;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d75: /* MOVG2 r4 r5 */
    {
        mcu.pc = 0x2d77;
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d77: /* MOVG3 r5 -> @r0+52 */
    {
        mcu.pc = 0x2d7a;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x34;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d7a: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2d7d;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d7d: /* bsr 22 -> 0x2d95 */
    {
        mcu.pc = 0x2d7f;
        uint16_t disp = (uint16_t)(int8_t)0x16;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2d7f: /* MOVG3 r5 -> @r0+24 */
    {
        mcu.pc = 0x2d82;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x18;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d82: /* move r5 #0xba */
    {
        mcu.pc = 0x2d84;
        uint8_t data = 0xba;
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d84: /* MOVG3 r5 -> @r0+26 */
    {
        mcu.pc = 0x2d87;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d87: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2d8a;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2d8a: /* SUB @r1+0xd0e0 #0x00 */
    {
        mcu.pc = 0x2d8f;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xe0;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x04;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
    }
    break;
    case 0x2d8f: /* BNE 0x2700 -> 0x5492 */
    {
        mcu.pc = 0x2d92;
        uint16_t disp = (uint16_t)(0x27 << 8);
        disp |= 0x00;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2d92: /* BRA 0x0e1f -> 0x3bb4 */
    {
        mcu.pc = 0x2d95;
        uint16_t disp = (uint16_t)(0x0e << 8);
        disp |= 0x1f;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    default:
        stock_instruction();
        break;
    }
    return 1;
}

/* note_on_setup C7: 583 PCs -> step_note_on_setup */
const uint16_t kNoteOnSetupPcs[] = {
    0x272e, 0x2731, 0x2734, 0x2738, 0x273c, 0x273f, 0x2743, 0x2746, 0x2748, 0x274c,
    0x274f, 0x2752, 0x2754, 0x2757, 0x275b, 0x275d, 0x2760, 0x2762, 0x2764, 0x2768,
    0x276a, 0x276c, 0x2771, 0x2773, 0x2777, 0x2779, 0x277d, 0x277f, 0x2783, 0x2785,
    0x2788, 0x278a, 0x278d, 0x2790, 0x2793, 0x2796, 0x2799, 0x279c, 0x279f, 0x27a2,
    0x27a5, 0x27a8, 0x27ab, 0x27ae, 0x27b1, 0x27b4, 0x27b7, 0x27ba, 0x27bd, 0x27c0,
    0x27c3, 0x27c6, 0x27c9, 0x27cc, 0x27cf, 0x27d2, 0x27d5, 0x27d8, 0x27db, 0x27de,
    0x27e1, 0x27e3, 0x27e7, 0x27eb, 0x27ee, 0x27f0, 0x27f4, 0x27f6, 0x27f9, 0x27fb,
    0x27ff, 0x2803, 0x2806, 0x2809, 0x280c, 0x280f, 0x2812, 0x2815, 0x2818, 0x281b,
    0x281d, 0x2820, 0x2822, 0x2824, 0x2828, 0x282a, 0x282c, 0x282f, 0x2831, 0x2834,
    0x2836, 0x283a, 0x283d, 0x283f, 0x2842, 0x2845, 0x2849, 0x284d, 0x2853, 0x2857,
    0x285b, 0x285d, 0x285f, 0x2862, 0x2865, 0x2868, 0x286c, 0x2870, 0x2874, 0x2878,
    0x287c, 0x287f, 0x2882, 0x2885, 0x2888, 0x288b, 0x288e, 0x2890, 0x2893, 0x2895,
    0x2897, 0x2899, 0x289c, 0x289f, 0x28a2, 0x28a5, 0x28a8, 0x28ab, 0x28ae, 0x28b1,
    0x28b4, 0x28b7, 0x28b9, 0x28bb, 0x28bd, 0x28bf, 0x28c2, 0x28c4, 0x28c8, 0x28ca,
    0x28cc, 0x28ce, 0x28d0, 0x28d2, 0x28d5, 0x28d8, 0x28db, 0x28de, 0x28e0, 0x28e4,
    0x28e6, 0x28eb, 0x28f0, 0x28f3, 0x28f5, 0x28f8, 0x28fb, 0x28fe, 0x2900, 0x2903,
    0x2906, 0x2909, 0x290c, 0x290e, 0x2913, 0x2916, 0x291a, 0x291f, 0x2922, 0x2925,
    0x292a, 0x292e, 0x2932, 0x2935, 0x2937, 0x293a, 0x293e, 0x2940, 0x2942, 0x2944,
    0x2947, 0x294b, 0x294e, 0x2951, 0x2953, 0x2957, 0x2959, 0x295b, 0x295e, 0x2960,
    0x2963, 0x2966, 0x2968, 0x296a, 0x296d, 0x296f, 0x2971, 0x2973, 0x2976, 0x2978,
    0x297a, 0x297e, 0x2980, 0x2982, 0x2984, 0x2988, 0x298a, 0x298d, 0x298f, 0x2993,
    0x2995, 0x2997, 0x2999, 0x299d, 0x299f, 0x29a2, 0x29a6, 0x29aa, 0x29ac, 0x29ae,
    0x29b2, 0x29b6, 0x29b9, 0x29bd, 0x29bf, 0x29c1, 0x29c5, 0x29c9, 0x29cc, 0x29d0,
    0x29d2, 0x29d4, 0x29d8, 0x29dc, 0x29de, 0x29e1, 0x29e5, 0x29e7, 0x29e9, 0x29ed,
    0x29f0, 0x29f2, 0x29f5, 0x29f9, 0x29fb, 0x29fd, 0x2a01, 0x2a04, 0x2a06, 0x2a09,
    0x2a0d, 0x2a0f, 0x2a11, 0x2a15, 0x2a18, 0x2a1a, 0x2a1d, 0x2a21, 0x2a23, 0x2a25,
    0x2a29, 0x2a2c, 0x2a2f, 0x2a31, 0x2a34, 0x2a38, 0x2a3b, 0x2a3d, 0x2a40, 0x2a44,
    0x2a46, 0x2a48, 0x2a4b, 0x2a4d, 0x2a50, 0x2a53, 0x2a55, 0x2a57, 0x2a59, 0x2a5b,
    0x2a5d, 0x2a5f, 0x2a62, 0x2a64, 0x2a66, 0x2a68, 0x2a6c, 0x2a6e, 0x2a70, 0x2a73,
    0x2a75, 0x2a77, 0x2a7b, 0x2a7d, 0x2a7f, 0x2a82, 0x2a84, 0x2a86, 0x2a8a, 0x2a8d,
    0x2a8f, 0x2a94, 0x2a96, 0x2a9a, 0x2a9c, 0x2a9f, 0x2aa1, 0x2aa4, 0x2aa8, 0x2aaa,
    0x2aac, 0x2aaf, 0x2ab1, 0x2ab4, 0x2ab7, 0x2ab9, 0x2abb, 0x2abd, 0x2abf, 0x2ac1,
    0x2ac3, 0x2ac6, 0x2ac8, 0x2aca, 0x2acc, 0x2ad0, 0x2ad2, 0x2ad4, 0x2ad7, 0x2ad9,
    0x2adb, 0x2adf, 0x2ae1, 0x2ae3, 0x2ae6, 0x2ae8, 0x2aea, 0x2aee, 0x2af1, 0x2af3,
    0x2af8, 0x2afb, 0x2aff, 0x2b01, 0x2b04, 0x2b07, 0x2b09, 0x2b0b, 0x2b0d, 0x2b11,
    0x2b14, 0x2b16, 0x2b19, 0x2b1b, 0x2b1e, 0x2b20, 0x2b22, 0x2b26, 0x2b28, 0x2b2a,
    0x2b2d, 0x2b31, 0x2b33, 0x2b36, 0x2b38, 0x2b3b, 0x2b3e, 0x2b40, 0x2b42, 0x2b45,
    0x2b47, 0x2b4a, 0x2b4d, 0x2b50, 0x2b54, 0x2b56, 0x2b59, 0x2b5c, 0x2b5e, 0x2b60,
    0x2b62, 0x2b66, 0x2b69, 0x2b6b, 0x2b6e, 0x2b70, 0x2b73, 0x2b75, 0x2b77, 0x2b7b,
    0x2b7d, 0x2b7f, 0x2b82, 0x2b86, 0x2b88, 0x2b8b, 0x2b8d, 0x2b90, 0x2b93, 0x2b95,
    0x2b97, 0x2b9a, 0x2b9c, 0x2b9f, 0x2ba2, 0x2ba5, 0x2ba7, 0x2ba9, 0x2bad, 0x2baf,
    0x2bb3, 0x2bb7, 0x2bba, 0x2bbd, 0x2bbf, 0x2bc1, 0x2bc5, 0x2bc7, 0x2bcb, 0x2bcf,
    0x2bd2, 0x2bd5, 0x2bd7, 0x2bd9, 0x2bdd, 0x2bdf, 0x2be3, 0x2be7, 0x2bea, 0x2bed,
    0x2bef, 0x2bf1, 0x2bf5, 0x2bf7, 0x2bfb, 0x2bff, 0x2c02, 0x2c05, 0x2c07, 0x2c09,
    0x2c0d, 0x2c0f, 0x2c13, 0x2c17, 0x2c1a, 0x2c1e, 0x2c22, 0x2c24, 0x2c27, 0x2c2b,
    0x2c2f, 0x2c32, 0x2c34, 0x2c36, 0x2c3a, 0x2c3d, 0x2c3f, 0x2c43, 0x2c45, 0x2c48,
    0x2c4a, 0x2c4d, 0x2c4f, 0x2c51, 0x2c54, 0x2c57, 0x2c5a, 0x2c5c, 0x2c5e, 0x2c60,
    0x2c62, 0x2c66, 0x2c6a, 0x2c6c, 0x2c6e, 0x2c70, 0x2c72, 0x2c77, 0x2c7a, 0x2c7c,
    0x2c7f, 0x2c81, 0x2c83, 0x2c86, 0x2c89, 0x2c8c, 0x2c8f, 0x2c92, 0x2c95, 0x2c98,
    0x2c9b, 0x2c9e, 0x2ca2, 0x2ca6, 0x2cac, 0x2caf, 0x2cb2, 0x2cb5, 0x2cb7, 0x2cb9,
    0x2cbb, 0x2cbe, 0x2cc2, 0x2cc6, 0x2cc9, 0x2ccc, 0x2ccf, 0x2cd2, 0x2cd4, 0x2cd6,
    0x2cda, 0x2cde, 0x2ce0, 0x2ce2, 0x2ce6, 0x2cea, 0x2cec, 0x2cee, 0x2cf0, 0x2cf2,
    0x2cf5, 0x2cf8, 0x2cfa, 0x2cfc, 0x2cfe, 0x2d02, 0x2d04, 0x2d07, 0x2d09, 0x2d0b,
    0x2d0d, 0x2d0f, 0x2d11, 0x2d13, 0x2d15, 0x2d17, 0x2d1a, 0x2d1d, 0x2d1f, 0x2d21,
    0x2d23, 0x2d25, 0x2d27, 0x2d29, 0x2d2b, 0x2d2d, 0x2d31, 0x2d33, 0x2d36, 0x2d38,
    0x2d3a, 0x2d3c, 0x2d3e, 0x2d40, 0x2d42, 0x2d44, 0x2d46, 0x2d49, 0x2d4b, 0x2d4f,
    0x2d53, 0x2d55, 0x2d57, 0x2d5b, 0x2d5d, 0x2d5f, 0x2d61, 0x2d66, 0x2d6a, 0x2d6d,
    0x2d6f, 0x2d73, 0x2d75, 0x2d77, 0x2d7a, 0x2d7d, 0x2d7f, 0x2d82, 0x2d84, 0x2d87,
    0x2d8a, 0x2d8f, 0x2d92,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void note_on_setup_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kNoteOnSetupPcs) / sizeof(kNoteOnSetupPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kNoteOnSetupPcs[i], &step_note_on_setup);
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
