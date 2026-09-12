/*
 * HAND pcm/ts_scan — time-slot scan (class C15, rom1 cp=0,
 * flat 0x5869..0x5995).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). The per-PC case bodies below are an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC; see pcm_irq_service.cpp for the L0 rationale and the
 * pending semantic rewrite (M4 closure step 2).
 *
 * Semantics: loops over voices P(v) looking for an active slot, writes ad2a
 * and clears P-0x26; TRAPA #0x1b; reached from 0x56c0 and linked with the C9 /
 * coeff chain (C16/C17).
 * Evidence: out/m4/18_closure_gap.md 1.3 C15 / 4.2 row 11; docs/07:149;
 * voice_bounds_inventory:38-39. Confidence: C (ROM bytes + dasm).
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

uint32_t step_ts_scan(void)
{
    switch (mcu.pc)
    {

    /* ---- ts ---- */
    case 0x5869: /* trapa #0x1b */
    {
        mcu.pc = 0x586b;
        uint8_t opcode = 0x1b;
        if ((opcode & 0xf0) == 0x10)
        MCU_Interrupt_TRAPA(opcode & 0x0f);
        else
        MCU_ErrorTrap();
    }
    break;
    case 0x586b: /* EXTU r0 */
    {
        mcu.pc = 0x586d;
        uint8_t op2 = 0x12;
        uint32_t data = (uint32_t)(mcu.r[0] & 0xff);
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x586d: /* MOVG3 r0 -> (dp,0xad2a) */
    {
        mcu.pc = 0x5871;
        uint32_t oea = (uint32_t)0xad;
        oea = (oea << 8) | (uint32_t)0x2a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x90;
        uint32_t data = (uint32_t)mcu.r[0];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5871: /* TST (dp,0xd19c) */
    {
        mcu.pc = 0x5875;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x9c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x5875: /* BEQ 3 -> 0x587a */
    {
        mcu.pc = 0x5877;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x5877: /* bsr16 -> 0x5dc3 */
    {
        mcu.pc = 0x587a;
        uint16_t disp = (uint16_t)(0x05 << 8);
        disp |= 0x49;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x587a: /* TST (dp,0xd19e) */
    {
        mcu.pc = 0x587e;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x9e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x587e: /* BEQ 3 -> 0x5883 */
    {
        mcu.pc = 0x5880;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x5880: /* bsr16 -> 0x618c */
    {
        mcu.pc = 0x5883;
        uint16_t disp = (uint16_t)(0x09 << 8);
        disp |= 0x09;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5883: /* LDC #0x00 r4 */
    {
        mcu.pc = 0x5886;
        uint32_t odata = (uint32_t)0x00;
        uint8_t op2 = 0x8c;
        uint32_t data = odata;
        MCU_ControlRegisterWrite(4, 0, data);
        mcu.ex_ignore = 1;
    }
    break;
    case 0x5886: /* movi r1 #0x001b */
    {
        mcu.pc = 0x5889;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x1b;
        mcu.r[1] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5889: /* MOVG2 r1 r2 */
    {
        mcu.pc = 0x588b;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x588b: /* ADD r2 r2 */
    {
        mcu.pc = 0x588d;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x588d: /* MOVG2 @r2+0x64d6 r0 */
    {
        mcu.pc = 0x5891;
        uint32_t odisp = (uint32_t)0x64;
        odisp = (odisp << 8) | (uint32_t)0xd6;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5895: /* BCC 61 -> 0x58d4 */
    {
        mcu.pc = 0x5897;
        uint16_t disp = (uint16_t)(int8_t)0x3d;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x5897: /* TST @r0+-26 */
    {
        mcu.pc = 0x589a;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe6;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SetStatusCommon(data, 0);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x589a: /* BNE 56 -> 0x58d4 */
    {
        mcu.pc = 0x589c;
        uint16_t disp = (uint16_t)(int8_t)0x38;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x589c: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x58a0;
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
    case 0x58a0: /* MOVG3 r1 -> (dp,0xd178) */
    {
        mcu.pc = 0x58a4;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x78;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x91;
        uint32_t data = (uint32_t)mcu.r[1];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58a4: /* CLR r3 */
    {
        mcu.pc = 0x58a6;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x58a6: /* SUB @r1+0xd0a8 #0xff */
    {
        mcu.pc = 0x58ab;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xa8;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x04;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0xff;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
    }
    break;
    case 0x58ab: /* BNE 21 -> 0x58c2 */
    {
        mcu.pc = 0x58ad;
        uint16_t disp = (uint16_t)(int8_t)0x15;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x58ad: /* SUB @r1+0xd0c4 #0xff */
    {
        mcu.pc = 0x58b2;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xc4;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x04;
        uint32_t t1 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0xff;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
    }
    break;
    case 0x58b2: /* BEQ 55 -> 0x58eb */
    {
        mcu.pc = 0x58b4;
        uint16_t disp = (uint16_t)(int8_t)0x37;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x58b4: /* MOVG2 @r1+0xd0c4 r3 */
    {
        mcu.pc = 0x58b8;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xc4;
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
    case 0x58b8: /* MOVG2 r3 r2 */
    {
        mcu.pc = 0x58ba;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[3];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58ba: /* ADD r2 r2 */
    {
        mcu.pc = 0x58bc;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x58bc: /* MOVG2 @r2+0x64d6 r2 */
    {
        mcu.pc = 0x58c0;
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
    case 0x58c0: /* BRA 106 -> 0x592c */
    {
        mcu.pc = 0x58c2;
        uint16_t disp = (uint16_t)(int8_t)0x6a;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x58c2: /* MOVG2 @r1+0xd0a8 r3 */
    {
        mcu.pc = 0x58c6;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xa8;
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
    case 0x58c6: /* XCH r3 r1 */
    {
        mcu.pc = 0x58c8;
        uint8_t op2 = 0x93;
        uint32_t r1 = (uint32_t)mcu.r[3];
        uint32_t r2 = (uint32_t)mcu.r[1];
        mcu.r[3] = (uint16_t)r2;
        mcu.r[1] = (uint16_t)r1;
    }
    break;
    case 0x58c8: /* MOVG2 r0 r2 */
    {
        mcu.pc = 0x58ca;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[0];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58ca: /* MOVG2 r1 r0 */
    {
        mcu.pc = 0x58cc;
        uint8_t op2 = 0x80;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58cc: /* ADD r0 r0 */
    {
        mcu.pc = 0x58ce;
        uint8_t op2 = 0x20;
        int32_t t1 = (int32_t)mcu.r[0];
        uint32_t t2 = (uint32_t)mcu.r[0];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[0] = (uint16_t)t1;
    }
    break;
    case 0x58ce: /* MOVG2 @r0+0x64d6 r0 */
    {
        mcu.pc = 0x58d2;
        uint32_t odisp = (uint32_t)0x64;
        odisp = (odisp << 8) | (uint32_t)0xd6;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58d2: /* BRA 88 -> 0x592c */
    {
        mcu.pc = 0x58d4;
        uint16_t disp = (uint16_t)(int8_t)0x58;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x58d4: /* cntjmp r1 -78 -> 0x5889 */
    {
        mcu.pc = 0x58d7;
        uint8_t op2 = 0xb9;
        uint8_t reg = op2 & 0x07;
        if ((op2 >> 3) == 0x17)
        {
        uint16_t disp = (uint16_t)(int8_t)0xb2;
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
    case 0x58d7: /* movi r1 #0x001b */
    {
        mcu.pc = 0x58da;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x1b;
        mcu.r[1] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58da: /* MOVG2 r1 r2 */
    {
        mcu.pc = 0x58dc;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58dc: /* ADD r2 r2 */
    {
        mcu.pc = 0x58de;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x58de: /* MOVG2 @r2+0x64d6 r0 */
    {
        mcu.pc = 0x58e2;
        uint32_t odisp = (uint32_t)0x64;
        odisp = (odisp << 8) | (uint32_t)0xd6;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58e2: /* CLR @r0+-26 */
    {
        mcu.pc = 0x58e5;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe6;
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
    case 0x58e5: /* cntjmp r1 -14 -> 0x58da */
    {
        mcu.pc = 0x58e8;
        uint8_t op2 = 0xb9;
        uint8_t reg = op2 & 0x07;
        if ((op2 >> 3) == 0x17)
        {
        uint16_t disp = (uint16_t)(int8_t)0xf2;
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
    case 0x58e8: /* BRA 0xfffffdcf -> 0x56ba */
    {
        mcu.pc = 0x58eb;
        uint16_t disp = (uint16_t)(0xfd << 8);
        disp |= 0xcf;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x58eb: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x58ef;
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
    case 0x58ef: /* MOVG3 r0 -> (dp,0xd17a) */
    {
        mcu.pc = 0x58f3;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x90;
        uint32_t data = (uint32_t)mcu.r[0];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58f3: /* MOVG3 r1 -> @r0+-2 */
    {
        mcu.pc = 0x58f6;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x91;
        uint32_t data = (uint32_t)mcu.r[1];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58f6: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x58fa;
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
    case 0x58fa: /* MOVG2 (dp,0xd17a) r0 */
    {
        mcu.pc = 0x58fe;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x58fe: /* bsr16 -> 0x5998 */
    {
        mcu.pc = 0x5901;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0x97;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5905: /* nop */
    {
        mcu.pc = 0x5906;
        /* nop */
    }
    break;
    case 0x5906: /* nop */
    {
        mcu.pc = 0x5907;
        /* nop */
    }
    break;
    case 0x5907: /* nop */
    {
        mcu.pc = 0x5908;
        /* nop */
    }
    break;
    case 0x5908: /* nop */
    {
        mcu.pc = 0x5909;
        /* nop */
    }
    break;
    case 0x5909: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x590d;
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
    case 0x590d: /* MOVG2 (dp,0xd17a) r0 */
    {
        mcu.pc = 0x5911;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5911: /* bsr16 -> 0x3709 */
    {
        mcu.pc = 0x5914;
        uint16_t disp = (uint16_t)(0xdd << 8);
        disp |= 0xf5;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5914: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x5918;
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
    case 0x5918: /* MOVG2 (dp,0xd17a) r0 */
    {
        mcu.pc = 0x591c;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x591c: /* bsr16 -> 0x2e83 */
    {
        mcu.pc = 0x591f;
        uint16_t disp = (uint16_t)(0xd5 << 8);
        disp |= 0x64;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x591f: /* MOVG2 (dp,0xd17a) r0 */
    {
        mcu.pc = 0x5923;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5923: /* bsr16 -> 0x5671 */
    {
        mcu.pc = 0x5926;
        uint16_t disp = (uint16_t)(0xfd << 8);
        disp |= 0x4b;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5926: /* MOVG2 (dp,0xd178) r1 */
    {
        mcu.pc = 0x592a;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x78;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x592a: /* BRA -88 -> 0x58d4 */
    {
        mcu.pc = 0x592c;
        uint16_t disp = (uint16_t)(int8_t)0xa8;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x592c: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x5930;
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
    case 0x5930: /* MOVG3 r0 -> (dp,0xd17a) */
    {
        mcu.pc = 0x5934;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x90;
        uint32_t data = (uint32_t)mcu.r[0];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5934: /* MOVG3 r1 -> @r0+-2 */
    {
        mcu.pc = 0x5937;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x91;
        uint32_t data = (uint32_t)mcu.r[1];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5937: /* MOVG3 r2 -> (dp,0xd17c) */
    {
        mcu.pc = 0x593b;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x92;
        uint32_t data = (uint32_t)mcu.r[2];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x593b: /* MOVG3 r3 -> @r2+-2 */
    {
        mcu.pc = 0x593e;
        uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xfe;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x93;
        uint32_t data = (uint32_t)mcu.r[3];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x593e: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x5942;
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
    case 0x5942: /* MOVG2 (dp,0xd17a) r0 */
    {
        mcu.pc = 0x5946;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5946: /* bsr 80 -> 0x5998 */
    {
        mcu.pc = 0x5948;
        uint16_t disp = (uint16_t)(int8_t)0x50;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5948: /* MOVG2 (dp,0xd17c) r0 */
    {
        mcu.pc = 0x594c;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x594c: /* MOVG2 (dp,0xd17a) r2 */
    {
        mcu.pc = 0x5950;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x82;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5950: /* bsr16 -> 0x5d6e */
    {
        mcu.pc = 0x5953;
        uint16_t disp = (uint16_t)(0x04 << 8);
        disp |= 0x1b;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5953: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x5957;
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
    case 0x5957: /* nop */
    {
        mcu.pc = 0x5958;
        /* nop */
    }
    break;
    case 0x5958: /* nop */
    {
        mcu.pc = 0x5959;
        /* nop */
    }
    break;
    case 0x5959: /* nop */
    {
        mcu.pc = 0x595a;
        /* nop */
    }
    break;
    case 0x595a: /* nop */
    {
        mcu.pc = 0x595b;
        /* nop */
    }
    break;
    case 0x595b: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x595f;
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
    case 0x595f: /* MOVG2 (dp,0xd17a) r0 */
    {
        mcu.pc = 0x5963;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5963: /* bsr16 -> 0x3709 */
    {
        mcu.pc = 0x5966;
        uint16_t disp = (uint16_t)(0xdd << 8);
        disp |= 0xa3;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5966: /* MOVG2 (dp,0xd17c) r0 */
    {
        mcu.pc = 0x596a;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x596a: /* MOVG2 (dp,0xd17a) r2 */
    {
        mcu.pc = 0x596e;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x82;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x596e: /* bsr16 -> 0x3ac8 */
    {
        mcu.pc = 0x5971;
        uint16_t disp = (uint16_t)(0xe1 << 8);
        disp |= 0x57;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5971: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x5975;
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
    case 0x5975: /* MOVG2 (dp,0xd17a) r0 */
    {
        mcu.pc = 0x5979;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5979: /* bsr16 -> 0x2e83 */
    {
        mcu.pc = 0x597c;
        uint16_t disp = (uint16_t)(0xd5 << 8);
        disp |= 0x07;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x597c: /* MOVG2 (dp,0xd17c) r0 */
    {
        mcu.pc = 0x5980;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5980: /* bsr16 -> 0x2e83 */
    {
        mcu.pc = 0x5983;
        uint16_t disp = (uint16_t)(0xd5 << 8);
        disp |= 0x00;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5983: /* MOVG2 (dp,0xd17a) r0 */
    {
        mcu.pc = 0x5987;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5987: /* bsr16 -> 0x5671 */
    {
        mcu.pc = 0x598a;
        uint16_t disp = (uint16_t)(0xfc << 8);
        disp |= 0xe7;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x598a: /* MOVG2 (dp,0xd17c) r0 */
    {
        mcu.pc = 0x598e;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x7c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x80;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x598e: /* bsr16 -> 0x5671 */
    {
        mcu.pc = 0x5991;
        uint16_t disp = (uint16_t)(0xfc << 8);
        disp |= 0xe0;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x5991: /* MOVG2 (dp,0xd178) r1 */
    {
        mcu.pc = 0x5995;
        uint32_t oea = (uint32_t)0xd1;
        oea = (oea << 8) | (uint32_t)0x78;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x5995: /* BRA 0xffffff3c -> 0x58d4 */
    {
        mcu.pc = 0x5998;
        uint16_t disp = (uint16_t)(0xff << 8);
        disp |= 0x3c;
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

/* ts_scan C15: 97 PCs -> step_ts_scan */
const uint16_t kTsScanPcs[] = {
    0x5869, 0x586b, 0x586d, 0x5871, 0x5875, 0x5877, 0x587a, 0x587e, 0x5880, 0x5883,
    0x5886, 0x5889, 0x588b, 0x588d, 0x5895, 0x5897, 0x589a, 0x589c, 0x58a0, 0x58a4,
    0x58a6, 0x58ab, 0x58ad, 0x58b2, 0x58b4, 0x58b8, 0x58ba, 0x58bc, 0x58c0, 0x58c2,
    0x58c6, 0x58c8, 0x58ca, 0x58cc, 0x58ce, 0x58d2, 0x58d4, 0x58d7, 0x58da, 0x58dc,
    0x58de, 0x58e2, 0x58e5, 0x58e8, 0x58eb, 0x58ef, 0x58f3, 0x58f6, 0x58fa, 0x58fe,
    0x5905, 0x5906, 0x5907, 0x5908, 0x5909, 0x590d, 0x5911, 0x5914, 0x5918, 0x591c,
    0x591f, 0x5923, 0x5926, 0x592a, 0x592c, 0x5930, 0x5934, 0x5937, 0x593b, 0x593e,
    0x5942, 0x5946, 0x5948, 0x594c, 0x5950, 0x5953, 0x5957, 0x5958, 0x5959, 0x595a,
    0x595b, 0x595f, 0x5963, 0x5966, 0x596a, 0x596e, 0x5971, 0x5975, 0x5979, 0x597c,
    0x5980, 0x5983, 0x5987, 0x598a, 0x598e, 0x5991, 0x5995,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void ts_scan_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kTsScanPcs) / sizeof(kTsScanPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kTsScanPcs[i], &step_ts_scan);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct TsScanSelfRegister
{
    TsScanSelfRegister() { hand_register_module(&ts_scan_fill); }
};

TsScanSelfRegister g_ts_scan_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
