/*
 * HAND pcm/pcm_fraction_div — 32/32 fixed-point fraction division ("svc_math",
 * class C6, rom1 cp=0, flat 0x2669..0x272d).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). The per-PC case bodies below are an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC; see pcm_irq_service.cpp for the L0 rationale and the
 * pending semantic rewrite (M4 closure step 2).
 *
 * Semantics: reached by fallthrough from C5 irq_service. Computes the
 * frequency fraction with 32/32 fixed-point division (word tables
 * 0x78ee/0x7aee, DIVXU #0x2ee0); the quotient is latched into (br,0x10) and
 * consumed by the caller's PCM reg10 write.
 * Evidence: out/m4/18_closure_gap.md 1.3 C6 / 4.2 row 7; dasm:4360+.
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

uint32_t step_fraction_div(void)
{
    switch (mcu.pc)
    {

    /* ---- svc ---- */
    case 0x2669: /* MOVG2 @r0+42 r4 */
    {
        mcu.pc = 0x266c;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x2a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x266c: /* MOVG2 @r0+64 r5 */
    {
        mcu.pc = 0x266f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x40;
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
    case 0x266f: /* MOVG3 r4 -> @r0+41 */
    {
        mcu.pc = 0x2672;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x29;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x94;
        uint32_t data = (uint32_t)mcu.r[4];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2672: /* MOVG3 r5 -> @r0+62 */
    {
        mcu.pc = 0x2675;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x3e;
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
    case 0x2675: /* MOVG2 @r0+45 r2 */
    {
        mcu.pc = 0x2678;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x2d;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2678: /* MOVG2 @r0+70 r3 */
    {
        mcu.pc = 0x267b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x46;
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
    case 0x267b: /* SUB r5 r3 */
    {
        mcu.pc = 0x267d;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[5];
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x267d: /* SUBX r4 r2 */
    {
        mcu.pc = 0x267f;
        uint8_t op2 = 0xb2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
        int32_t C = (mcu.sr & STATUS_C) != 0;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, C, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x267f: /* bsr 35 -> 0x26a4 */
    {
        mcu.pc = 0x2681;
        uint16_t disp = (uint16_t)(int8_t)0x23;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2681: /* CLR @r0+0x00a4 */
    {
        mcu.pc = 0x2685;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0xa4;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
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
    case 0x2685: /* MOVG2 @r0+0x00a6 r1 */
    {
        mcu.pc = 0x2689;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0xa6;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
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
    case 0x2689: /* BMI 9 -> 0x2694 */
    {
        mcu.pc = 0x268b;
        uint16_t disp = (uint16_t)(int8_t)0x09;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x268b: /* ADD r1 r4 */
    {
        mcu.pc = 0x268d;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x268d: /* BCC 11 -> 0x269a */
    {
        mcu.pc = 0x268f;
        uint16_t disp = (uint16_t)(int8_t)0x0b;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x268f: /* movi r4 #0xffff */
    {
        mcu.pc = 0x2692;
        uint16_t data = (uint16_t)(0xff << 8);
        data |= 0xff;
        mcu.r[4] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2692: /* BRA 6 -> 0x269a */
    {
        mcu.pc = 0x2694;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2694: /* ADD r1 r4 */
    {
        mcu.pc = 0x2696;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2696: /* BCS 2 -> 0x269a */
    {
        mcu.pc = 0x2698;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2698: /* CLR r4 */
    {
        mcu.pc = 0x269a;
        uint8_t op2 = 0x13;
        mcu.r[4] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x269a: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x269d;
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
    case 0x269d: /* movs r1 @(br,$3e) */
    {
        mcu.pc = 0x269f;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3e;
        uint32_t data = (uint32_t)(mcu.r[1] & 0xff);
        MCU_Write(addr, (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x269f: /* movsw r4 @(br,$10) */
    {
        mcu.pc = 0x26a1;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x10;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)mcu.r[4];
        MCU_Write16(addr, (uint16_t)data);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26a1: /* BRA 0x2b52 -> 0x51f6 */
    {
        mcu.pc = 0x26a4;
        uint16_t disp = (uint16_t)(0x2b << 8);
        disp |= 0x52;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x26a4: /* SUB #0x2ee0 r3 */
    {
        mcu.pc = 0x26a8;
        uint32_t odata = (uint32_t)0x2e;
        odata = (odata << 8) | (uint32_t)0xe0;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x26a8: /* SUBX #0x00 r2 */
    {
        mcu.pc = 0x26ab;
        uint32_t odata = (uint32_t)0x00;
        uint8_t op2 = 0xb2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = odata;
        int32_t C = (mcu.sr & STATUS_C) != 0;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, C, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;
    case 0x26ab: /* BPL 78 -> 0x26fb */
    {
        mcu.pc = 0x26ad;
        uint16_t disp = (uint16_t)(int8_t)0x4e;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x26ad: /* EXTS r2 */
    {
        mcu.pc = 0x26af;
        uint8_t op2 = 0x11;
        uint32_t data = (uint32_t)mcu.r[2];
        mcu.r[2] = (uint16_t)(int8_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26af: /* NOT r2 */
    {
        mcu.pc = 0x26b1;
        uint8_t op2 = 0x15;
        uint32_t data = (uint32_t)mcu.r[2];
        data = ~data;
        mcu.r[2] = (uint16_t)(data);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26b1: /* NOT r3 */
    {
        mcu.pc = 0x26b3;
        uint8_t op2 = 0x15;
        uint32_t data = (uint32_t)mcu.r[3];
        data = ~data;
        mcu.r[3] = (uint16_t)(data);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26b3: /* ADDQ #1 r3 */
    {
        mcu.pc = 0x26b5;
        uint8_t op2 = 0x08;
        uint32_t t1 = (uint32_t)mcu.r[3];
        int32_t t2 = 1;
        t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, t2, 0, 1);
        mcu.r[3] = (uint16_t)(t1);
    }
    break;
    case 0x26b5: /* ADDX #0x0000 r2 */
    {
        mcu.pc = 0x26b9;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x00;
        uint8_t op2 = 0xa2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = odata;
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x26b9: /* DIVXU #0x2ee0 r2:r3 */
    {
        mcu.pc = 0x26bd;
        uint32_t odata = (uint32_t)0x2e;
        odata = (odata << 8) | (uint32_t)0xe0;
        uint8_t op2 = 0xba;
        uint32_t t1 = odata;
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
    case 0x26bd: /* cmp r2,w #0x0000 */
    {
        mcu.pc = 0x26c0;
        int32_t t2 = (int32_t)0x00;
        t2 = (t2 << 8) | (int32_t)0x00;
        int32_t t1 = (int32_t)mcu.r[2];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x26c0: /* BEQ 8 -> 0x26ca */
    {
        mcu.pc = 0x26c2;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x26c2: /* ADDQ #1 r3 */
    {
        mcu.pc = 0x26c4;
        uint8_t op2 = 0x08;
        uint32_t t1 = (uint32_t)mcu.r[3];
        int32_t t2 = 1;
        t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, t2, 0, 1);
        mcu.r[3] = (uint16_t)(t1);
    }
    break;
    case 0x26c4: /* NEG r2 */
    {
        mcu.pc = 0x26c6;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;
    case 0x26c6: /* ADD #0x2ee0 r2 */
    {
        mcu.pc = 0x26ca;
        uint32_t odata = (uint32_t)0x2e;
        odata = (odata << 8) | (uint32_t)0xe0;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = odata;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x26ca: /* CLR r1 */
    {
        mcu.pc = 0x26cc;
        uint8_t op2 = 0x13;
        mcu.r[1] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x26cc: /* MOVG2 r2 r1 */
    {
        mcu.pc = 0x26ce;
        uint8_t op2 = 0x81;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        mcu.r[1] &= ~0xff;
        mcu.r[1] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x26ce: /* ADD r1 r1 */
    {
        mcu.pc = 0x26d0;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;
    case 0x26d0: /* MOVG2 @r1+0x78ee r4 */
    {
        mcu.pc = 0x26d4;
        uint32_t odisp = (uint32_t)0x78;
        odisp = (odisp << 8) | (uint32_t)0xee;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x84;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26d4: /* CLR r1 */
    {
        mcu.pc = 0x26d6;
        uint8_t op2 = 0x13;
        mcu.r[1] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x26d6: /* SWAP r2 */
    {
        mcu.pc = 0x26d8;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[2];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26d8: /* MOVG2 r2 r1 */
    {
        mcu.pc = 0x26da;
        uint8_t op2 = 0x81;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        mcu.r[1] &= ~0xff;
        mcu.r[1] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x26da: /* ADD r1 r1 */
    {
        mcu.pc = 0x26dc;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;
    case 0x26dc: /* MOVG2 @r1+0x7aee r1 */
    {
        mcu.pc = 0x26e0;
        uint32_t odisp = (uint32_t)0x7a;
        odisp = (odisp << 8) | (uint32_t)0xee;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26e0: /* MULXU r1 r4:r5 */
    {
        mcu.pc = 0x26e2;
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 *= t2;
        mcu.r[4] = (uint16_t)(t1 >> 16);
        mcu.r[5] = (uint16_t)t1;
        uint32_t N = (t1 & 0x80000000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x26e2: /* ROTL r4 */
    {
        mcu.pc = 0x26e4;
        uint8_t op2 = 0x1c;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = (data & 0x8000u) != 0;
        data <<= 1;
        data |= C;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26e4: /* ROTL r4 */
    {
        mcu.pc = 0x26e6;
        uint8_t op2 = 0x1c;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = (data & 0x8000u) != 0;
        data <<= 1;
        data |= C;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26e6: /* AND #0x03 r4 */
    {
        mcu.pc = 0x26e9;
        uint32_t odata = (uint32_t)0x03;
        uint8_t op2 = 0x54;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(mcu.r[4], 0);
    }
    break;
    case 0x26e9: /* SWAP r4 */
    {
        mcu.pc = 0x26eb;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26eb: /* ADD r1 r4 */
    {
        mcu.pc = 0x26ed;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x26ed: /* TST r3 */
    {
        mcu.pc = 0x26ef;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x26ef: /* BEQ 60 -> 0x272d */
    {
        mcu.pc = 0x26f1;
        uint16_t disp = (uint16_t)(int8_t)0x3c;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x26f1: /* SUB #0x0001 r3 */
    {
        mcu.pc = 0x26f5;
        uint32_t odata = (uint32_t)0x00;
        odata = (odata << 8) | (uint32_t)0x01;
        uint8_t op2 = 0x33;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x26f5: /* SHLR r4 */
    {
        mcu.pc = 0x26f7;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26f7: /* cntjmp r3 -5 -> 0x26f5 */
    {
        mcu.pc = 0x26fa;
        uint8_t op2 = 0xbb;
        uint8_t reg = op2 & 0x07;
        if ((op2 >> 3) == 0x17)
        {
        uint16_t disp = (uint16_t)(int8_t)0xfb;
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
    case 0x26fa: /* rts */
    {
        mcu.pc = 0x26fb;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x26fb: /* EXTS r2 */
    {
        mcu.pc = 0x26fd;
        uint8_t op2 = 0x11;
        uint32_t data = (uint32_t)mcu.r[2];
        mcu.r[2] = (uint16_t)(int8_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x26fd: /* DIVXU #0x2ee0 r2:r3 */
    {
        mcu.pc = 0x2701;
        uint32_t odata = (uint32_t)0x2e;
        odata = (odata << 8) | (uint32_t)0xe0;
        uint8_t op2 = 0xba;
        uint32_t t1 = odata;
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
    case 0x2701: /* TST r3 */
    {
        mcu.pc = 0x2703;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2703: /* BEQ 5 -> 0x270a */
    {
        mcu.pc = 0x2705;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2705: /* MOVG2 #0xffff r4 */
    {
        mcu.pc = 0x2709;
        uint32_t odata = (uint32_t)0xff;
        odata = (odata << 8) | (uint32_t)0xff;
        uint8_t op2 = 0x84;
        uint32_t data = odata;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2709: /* rts */
    {
        mcu.pc = 0x270a;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x270a: /* CLR r1 */
    {
        mcu.pc = 0x270c;
        uint8_t op2 = 0x13;
        mcu.r[1] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x270c: /* MOVG2 r2 r1 */
    {
        mcu.pc = 0x270e;
        uint8_t op2 = 0x81;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        mcu.r[1] &= ~0xff;
        mcu.r[1] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x270e: /* ADD r1 r1 */
    {
        mcu.pc = 0x2710;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;
    case 0x2710: /* MOVG2 @r1+0x78ee r4 */
    {
        mcu.pc = 0x2714;
        uint32_t odisp = (uint32_t)0x78;
        odisp = (odisp << 8) | (uint32_t)0xee;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x84;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2714: /* CLR r1 */
    {
        mcu.pc = 0x2716;
        uint8_t op2 = 0x13;
        mcu.r[1] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2716: /* SWAP r2 */
    {
        mcu.pc = 0x2718;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[2];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2718: /* MOVG2 r2 r1 */
    {
        mcu.pc = 0x271a;
        uint8_t op2 = 0x81;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        mcu.r[1] &= ~0xff;
        mcu.r[1] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x271a: /* ADD r1 r1 */
    {
        mcu.pc = 0x271c;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;
    case 0x271c: /* MOVG2 @r1+0x7aee r1 */
    {
        mcu.pc = 0x2720;
        uint32_t odisp = (uint32_t)0x7a;
        odisp = (odisp << 8) | (uint32_t)0xee;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x81;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2720: /* MULXU r1 r4:r5 */
    {
        mcu.pc = 0x2722;
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[4];
        t1 *= t2;
        mcu.r[4] = (uint16_t)(t1 >> 16);
        mcu.r[5] = (uint16_t)t1;
        uint32_t N = (t1 & 0x80000000u) != 0;
        uint32_t Z = (t1 == 0);
        MCU_SetStatus(N, STATUS_N);
        MCU_SetStatus(Z, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2722: /* ROTL r4 */
    {
        mcu.pc = 0x2724;
        uint8_t op2 = 0x1c;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = (data & 0x8000u) != 0;
        data <<= 1;
        data |= C;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2724: /* ROTL r4 */
    {
        mcu.pc = 0x2726;
        uint8_t op2 = 0x1c;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t C = (data & 0x8000u) != 0;
        data <<= 1;
        data |= C;
        mcu.r[4] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2726: /* AND #0x03 r4 */
    {
        mcu.pc = 0x2729;
        uint32_t odata = (uint32_t)0x03;
        uint8_t op2 = 0x54;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t t2 = odata;
        data &= t2;
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(mcu.r[4], 0);
    }
    break;
    case 0x2729: /* SWAP r4 */
    {
        mcu.pc = 0x272b;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[4];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x272b: /* ADD r1 r4 */
    {
        mcu.pc = 0x272d;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x272d: /* rts */
    {
        mcu.pc = 0x272e;
        mcu.pc = MCU_PopStack();
    }
    break;

    default:
        stock_instruction();
        break;
    }
    return 1;
}

/* fraction_div C6: 80 PCs -> step_fraction_div */
const uint16_t kFractionDivPcs[] = {
    0x2669, 0x266c, 0x266f, 0x2672, 0x2675, 0x2678, 0x267b, 0x267d, 0x267f, 0x2681,
    0x2685, 0x2689, 0x268b, 0x268d, 0x268f, 0x2692, 0x2694, 0x2696, 0x2698, 0x269a,
    0x269d, 0x269f, 0x26a1, 0x26a4, 0x26a8, 0x26ab, 0x26ad, 0x26af, 0x26b1, 0x26b3,
    0x26b5, 0x26b9, 0x26bd, 0x26c0, 0x26c2, 0x26c4, 0x26c6, 0x26ca, 0x26cc, 0x26ce,
    0x26d0, 0x26d4, 0x26d6, 0x26d8, 0x26da, 0x26dc, 0x26e0, 0x26e2, 0x26e4, 0x26e6,
    0x26e9, 0x26eb, 0x26ed, 0x26ef, 0x26f1, 0x26f5, 0x26f7, 0x26fa, 0x26fb, 0x26fd,
    0x2701, 0x2703, 0x2705, 0x2709, 0x270a, 0x270c, 0x270e, 0x2710, 0x2714, 0x2716,
    0x2718, 0x271a, 0x271c, 0x2720, 0x2722, 0x2724, 0x2726, 0x2729, 0x272b, 0x272d,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void pcm_fraction_div_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kFractionDivPcs) / sizeof(kFractionDivPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kFractionDivPcs[i], &step_fraction_div);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct PcmFractionDivSelfRegister
{
    PcmFractionDivSelfRegister() { hand_register_module(&pcm_fraction_div_fill); }
};

PcmFractionDivSelfRegister g_pcm_fraction_div_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
