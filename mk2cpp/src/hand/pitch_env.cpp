/*
 * HAND pcm/pitch_env — pitch/envelope-related per-voice processing
 * (class C8 "r2d95", rom1 cp=0, flat 0x2d95..0x2e82).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). The per-PC case bodies below are an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC; see pcm_irq_service.cpp for the L0 rationale and the
 * pending semantic rewrite (M4 closure step 2).
 *
 * Semantics: pitch/envelope-related per-voice processing; the exact field
 * semantics are NOT fully proven (docs/07:142; 18_closure_gap row 9 marks
 * C bytes / I semantics). The filename is a provisional label, not a proven
 * identity; the body is a byte-exact transcription, never a re-derivation.
 * Evidence: out/m4/18_closure_gap.md 1.3 C8 / 4.2 row 9; frag 2d95.
 * Confidence: C bytes / I semantics.
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

uint32_t step_pitch_env(void)
{
    switch (mcu.pc)
    {

    /* ---- c8 ---- */
    case 0x2d95: /* CLR r2 */
    {
        mcu.pc = 0x2d97;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2d97: /* MOVG2 @r1+0xce78 r2 */
    {
        mcu.pc = 0x2d9b;
        uint32_t odisp = (uint32_t)0xce;
        odisp = (odisp << 8) | (uint32_t)0x78;
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
    case 0x2d9b: /* MOVG2 @r2+0xac0e r2 */
    {
        mcu.pc = 0x2d9f;
        uint32_t odisp = (uint32_t)0xac;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[2] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2d9f: /* MOVG2 @r0+46 r3 */
    {
        mcu.pc = 0x2da2;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x2e;
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
    case 0x2da2: /* MULXU @r3+8 r2 */
    {
        mcu.pc = 0x2da5;
        uint32_t oea = (uint32_t)mcu.r[3] + (uint32_t)(int8_t)0x08;
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
    case 0x2da5: /* CLR r6 */
    {
        mcu.pc = 0x2da7;
        uint8_t op2 = 0x13;
        mcu.r[6] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2da7: /* MOVG2 (dp,0x8002) r6 */
    {
        mcu.pc = 0x2dab;
        uint32_t oea = (uint32_t)0x80;
        oea = (oea << 8) | (uint32_t)0x02;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(mcu.dp & 0xff);
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] &= ~0xff;
        mcu.r[6] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2dab: /* MULXU r6 r2:r3 */
    {
        mcu.pc = 0x2dad;
        uint8_t op2 = 0xaa;
        uint32_t t1 = (uint32_t)mcu.r[6];
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
    case 0x2dad: /* ADD r3 r3 */
    {
        mcu.pc = 0x2daf;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2daf: /* ADDX r2 r2 */
    {
        mcu.pc = 0x2db1;
        uint8_t op2 = 0xa2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2db1: /* ADD r3 r3 */
    {
        mcu.pc = 0x2db3;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2db3: /* ADDX r2 r2 */
    {
        mcu.pc = 0x2db5;
        uint8_t op2 = 0xa2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2db5: /* MOVG2 r2 r3 */
    {
        mcu.pc = 0x2db7;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2db7: /* SWAP r3 */
    {
        mcu.pc = 0x2db9;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2db9: /* MOVG2 r3 r2 */
    {
        mcu.pc = 0x2dbb;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[3];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2dbb: /* MOVG2 @r0+48 r3 */
    {
        mcu.pc = 0x2dbe;
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
    case 0x2dbe: /* BEQ 22 -> 0x2dd6 */
    {
        mcu.pc = 0x2dc0;
        uint16_t disp = (uint16_t)(int8_t)0x16;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2dc0: /* MOVG2 @r3+0x0100 r6 */
    {
        mcu.pc = 0x2dc4;
        uint32_t odisp = (uint32_t)0x01;
        odisp = (odisp << 8) | (uint32_t)0x00;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[6] &= ~0xff;
        mcu.r[6] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2dc4: /* MULXU r6 r2:r3 */
    {
        mcu.pc = 0x2dc6;
        uint8_t op2 = 0xaa;
        uint32_t t1 = (uint32_t)mcu.r[6];
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
    case 0x2dc6: /* ADD r3 r3 */
    {
        mcu.pc = 0x2dc8;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2dc8: /* ADDX r2 r2 */
    {
        mcu.pc = 0x2dca;
        uint8_t op2 = 0xa2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2dca: /* MOVG2 r2 r3 */
    {
        mcu.pc = 0x2dcc;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2dcc: /* SWAP r3 */
    {
        mcu.pc = 0x2dce;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2dce: /* MOVG2 r3 r2 */
    {
        mcu.pc = 0x2dd0;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[3];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2dd0: /* MULXU #0x830e r2:r3 */
    {
        mcu.pc = 0x2dd4;
        uint32_t odata = (uint32_t)0x83;
        odata = (odata << 8) | (uint32_t)0x0e;
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
    case 0x2dd4: /* BRA 4 -> 0x2dda */
    {
        mcu.pc = 0x2dd6;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2dd6: /* MULXU #0x8208 r2:r3 */
    {
        mcu.pc = 0x2dda;
        uint32_t odata = (uint32_t)0x82;
        odata = (odata << 8) | (uint32_t)0x08;
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
    case 0x2dda: /* ADD r3 r3 */
    {
        mcu.pc = 0x2ddc;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2ddc: /* ADDX r2 r2 */
    {
        mcu.pc = 0x2dde;
        uint8_t op2 = 0xa2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2dde: /* MOVG2 r2 r4 */
    {
        mcu.pc = 0x2de0;
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)mcu.r[2];
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2de0: /* BNE 3 -> 0x2de5 */
    {
        mcu.pc = 0x2de2;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2de2: /* CLR r5 */
    {
        mcu.pc = 0x2de4;
        uint8_t op2 = 0x13;
        mcu.r[5] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2de4: /* rts */
    {
        mcu.pc = 0x2de5;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x2de5: /* MOVG2 @r0+0x008a r2 */
    {
        mcu.pc = 0x2de9;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x8a;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
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
    case 0x2de9: /* BEQ 14 -> 0x2df9 */
    {
        mcu.pc = 0x2deb;
        uint16_t disp = (uint16_t)(int8_t)0x0e;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2deb: /* BPL 10 -> 0x2df7 */
    {
        mcu.pc = 0x2ded;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2ded: /* NEG r2 */
    {
        mcu.pc = 0x2def;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;
    case 0x2def: /* SUB r2 r4 */
    {
        mcu.pc = 0x2df1;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2df1: /* BCC 6 -> 0x2df9 */
    {
        mcu.pc = 0x2df3;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2df3: /* CLR r4 */
    {
        mcu.pc = 0x2df5;
        uint8_t op2 = 0x13;
        mcu.r[4] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2df5: /* BRA 2 -> 0x2df9 */
    {
        mcu.pc = 0x2df7;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2df7: /* ADD r2 r4 */
    {
        mcu.pc = 0x2df9;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2df9: /* MOVG2 @r0+-122 r2 */
    {
        mcu.pc = 0x2dfc;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x86;
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
    case 0x2dfc: /* MOVG2 @r0+0x008e r3 */
    {
        mcu.pc = 0x2e00;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x8e;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
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
    case 0x2e00: /* MOVG2 @r0+-96 r6 */
    {
        mcu.pc = 0x2e03;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xa0;
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
    case 0x2e03: /* bsr 32 -> 0x2e25 */
    {
        mcu.pc = 0x2e05;
        uint16_t disp = (uint16_t)(int8_t)0x20;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2e05: /* MOVG2 @r0+-88 r2 */
    {
        mcu.pc = 0x2e08;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xa8;
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
    case 0x2e08: /* MOVG2 @r0+0x0096 r3 */
    {
        mcu.pc = 0x2e0c;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x96;
        uint32_t oea = (uint32_t)mcu.r[0] + odisp;
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
    case 0x2e0c: /* MOVG2 @r0+-62 r6 */
    {
        mcu.pc = 0x2e0f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc2;
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
    case 0x2e0f: /* bsr 20 -> 0x2e25 */
    {
        mcu.pc = 0x2e11;
        uint16_t disp = (uint16_t)(int8_t)0x14;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2e11: /* MULXU r4 r4:r5 */
    {
        mcu.pc = 0x2e13;
        uint8_t op2 = 0xac;
        uint32_t t1 = (uint32_t)mcu.r[4];
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
    case 0x2e13: /* MULXU #0x0208 r4:r5 */
    {
        mcu.pc = 0x2e17;
        uint32_t odata = (uint32_t)0x02;
        odata = (odata << 8) | (uint32_t)0x08;
        uint8_t op2 = 0xac;
        uint32_t t1 = odata;
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
    case 0x2e17: /* cmp r4,w #0x00ff */
    {
        mcu.pc = 0x2e1a;
        int32_t t2 = (int32_t)0x00;
        t2 = (t2 << 8) | (int32_t)0xff;
        int32_t t1 = (int32_t)mcu.r[4];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x2e1a: /* BCC 5 -> 0x2e21 */
    {
        mcu.pc = 0x2e1c;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e1c: /* MOVG2 r4 r5 */
    {
        mcu.pc = 0x2e1e;
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)(mcu.r[4] & 0xff);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2e1e: /* SWAP r5 */
    {
        mcu.pc = 0x2e20;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2e20: /* rts */
    {
        mcu.pc = 0x2e21;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x2e21: /* movi r5 #0xffff */
    {
        mcu.pc = 0x2e24;
        uint16_t data = (uint16_t)(0xff << 8);
        data |= 0xff;
        mcu.r[5] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2e24: /* rts */
    {
        mcu.pc = 0x2e25;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x2e25: /* TST r2 */
    {
        mcu.pc = 0x2e27;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[2];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2e27: /* BMI 16 -> 0x2e39 */
    {
        mcu.pc = 0x2e29;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e29: /* TST r3 */
    {
        mcu.pc = 0x2e2b;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2e2b: /* BMI 32 -> 0x2e4d */
    {
        mcu.pc = 0x2e2d;
        uint16_t disp = (uint16_t)(int8_t)0x20;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e2d: /* ADD r3 r2 */
    {
        mcu.pc = 0x2e2f;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2e2f: /* cmp r2,w #0x7f00 */
    {
        mcu.pc = 0x2e32;
        int32_t t2 = (int32_t)0x7f;
        t2 = (t2 << 8) | (int32_t)0x00;
        int32_t t1 = (int32_t)mcu.r[2];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x2e32: /* BLS 33 -> 0x2e55 */
    {
        mcu.pc = 0x2e34;
        uint16_t disp = (uint16_t)(int8_t)0x21;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e34: /* movi r2 #0x7f00 */
    {
        mcu.pc = 0x2e37;
        uint16_t data = (uint16_t)(0x7f << 8);
        data |= 0x00;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2e37: /* BRA 28 -> 0x2e55 */
    {
        mcu.pc = 0x2e39;
        uint16_t disp = (uint16_t)(int8_t)0x1c;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e39: /* TST r3 */
    {
        mcu.pc = 0x2e3b;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[3];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2e3b: /* BPL 16 -> 0x2e4d */
    {
        mcu.pc = 0x2e3d;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e3d: /* NEG r3 */
    {
        mcu.pc = 0x2e3f;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[3];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[3] = (uint16_t)(data);
    }
    break;
    case 0x2e3f: /* NEG r2 */
    {
        mcu.pc = 0x2e41;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;
    case 0x2e41: /* ADD r3 r2 */
    {
        mcu.pc = 0x2e43;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2e43: /* cmp r2,w #0x7f00 */
    {
        mcu.pc = 0x2e46;
        int32_t t2 = (int32_t)0x7f;
        t2 = (t2 << 8) | (int32_t)0x00;
        int32_t t1 = (int32_t)mcu.r[2];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;
    case 0x2e46: /* BLS 21 -> 0x2e5d */
    {
        mcu.pc = 0x2e48;
        uint16_t disp = (uint16_t)(int8_t)0x15;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e48: /* movi r2 #0x7f00 */
    {
        mcu.pc = 0x2e4b;
        uint16_t data = (uint16_t)(0x7f << 8);
        data |= 0x00;
        mcu.r[2] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2e4b: /* BRA 16 -> 0x2e5d */
    {
        mcu.pc = 0x2e4d;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e4d: /* ADD r3 r2 */
    {
        mcu.pc = 0x2e4f;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2e4f: /* BPL 4 -> 0x2e55 */
    {
        mcu.pc = 0x2e51;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e51: /* NEG r2 */
    {
        mcu.pc = 0x2e53;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;
    case 0x2e53: /* BRA 8 -> 0x2e5d */
    {
        mcu.pc = 0x2e55;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e55: /* TST r6 */
    {
        mcu.pc = 0x2e57;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[6];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2e57: /* BPL 10 -> 0x2e63 */
    {
        mcu.pc = 0x2e59;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e59: /* NEG r6 */
    {
        mcu.pc = 0x2e5b;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[6];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[6] = (uint16_t)(data);
    }
    break;
    case 0x2e5b: /* BRA 20 -> 0x2e71 */
    {
        mcu.pc = 0x2e5d;
        uint16_t disp = (uint16_t)(int8_t)0x14;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e5d: /* TST r6 */
    {
        mcu.pc = 0x2e5f;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[6];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2e5f: /* BPL 16 -> 0x2e71 */
    {
        mcu.pc = 0x2e61;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e61: /* NEG r6 */
    {
        mcu.pc = 0x2e63;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[6];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[6] = (uint16_t)(data);
    }
    break;
    case 0x2e63: /* MULXU r6 r2:r3 */
    {
        mcu.pc = 0x2e65;
        uint8_t op2 = 0xaa;
        uint32_t t1 = (uint32_t)mcu.r[6];
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
    case 0x2e65: /* ADD r3 r3 */
    {
        mcu.pc = 0x2e67;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2e67: /* ADDX r2 r2 */
    {
        mcu.pc = 0x2e69;
        uint8_t op2 = 0xa2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2e69: /* ADD #0xffff r3 */
    {
        mcu.pc = 0x2e6d;
        uint32_t odata = (uint32_t)0xff;
        odata = (odata << 8) | (uint32_t)0xff;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2e6d: /* ADDX r2 r4 */
    {
        mcu.pc = 0x2e6f;
        uint8_t op2 = 0xa4;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[2];
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2e6f: /* BRA 16 -> 0x2e81 */
    {
        mcu.pc = 0x2e71;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e71: /* MULXU r6 r2:r3 */
    {
        mcu.pc = 0x2e73;
        uint8_t op2 = 0xaa;
        uint32_t t1 = (uint32_t)mcu.r[6];
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
    case 0x2e73: /* ADD r3 r3 */
    {
        mcu.pc = 0x2e75;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2e75: /* ADDX r2 r2 */
    {
        mcu.pc = 0x2e77;
        uint8_t op2 = 0xa2;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
        if (!Z)
        MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x2e77: /* ADD #0xffff r3 */
    {
        mcu.pc = 0x2e7b;
        uint32_t odata = (uint32_t)0xff;
        odata = (odata << 8) | (uint32_t)0xff;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = odata;
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;
    case 0x2e7b: /* SUBX r2 r4 */
    {
        mcu.pc = 0x2e7d;
        uint8_t op2 = 0xb4;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[2];
        int32_t C = (mcu.sr & STATUS_C) != 0;
        t1 = MCU_SUB_Common(t1, (int32_t)t2, C, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;
    case 0x2e7d: /* BCC 2 -> 0x2e81 */
    {
        mcu.pc = 0x2e7f;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e7f: /* CLR r4 */
    {
        mcu.pc = 0x2e81;
        uint8_t op2 = 0x13;
        mcu.r[4] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2e81: /* rts */
    {
        mcu.pc = 0x2e82;
        mcu.pc = MCU_PopStack();
    }
    break;
    default:
        stock_instruction();
        break;
    }
    return 1;
}

/* pitch_env C8: 102 PCs -> step_pitch_env */
const uint16_t kPitchEnvPcs[] = {
    0x2d95, 0x2d97, 0x2d9b, 0x2d9f, 0x2da2, 0x2da5, 0x2da7, 0x2dab, 0x2dad, 0x2daf,
    0x2db1, 0x2db3, 0x2db5, 0x2db7, 0x2db9, 0x2dbb, 0x2dbe, 0x2dc0, 0x2dc4, 0x2dc6,
    0x2dc8, 0x2dca, 0x2dcc, 0x2dce, 0x2dd0, 0x2dd4, 0x2dd6, 0x2dda, 0x2ddc, 0x2dde,
    0x2de0, 0x2de2, 0x2de4, 0x2de5, 0x2de9, 0x2deb, 0x2ded, 0x2def, 0x2df1, 0x2df3,
    0x2df5, 0x2df7, 0x2df9, 0x2dfc, 0x2e00, 0x2e03, 0x2e05, 0x2e08, 0x2e0c, 0x2e0f,
    0x2e11, 0x2e13, 0x2e17, 0x2e1a, 0x2e1c, 0x2e1e, 0x2e20, 0x2e21, 0x2e24, 0x2e25,
    0x2e27, 0x2e29, 0x2e2b, 0x2e2d, 0x2e2f, 0x2e32, 0x2e34, 0x2e37, 0x2e39, 0x2e3b,
    0x2e3d, 0x2e3f, 0x2e41, 0x2e43, 0x2e46, 0x2e48, 0x2e4b, 0x2e4d, 0x2e4f, 0x2e51,
    0x2e53, 0x2e55, 0x2e57, 0x2e59, 0x2e5b, 0x2e5d, 0x2e5f, 0x2e61, 0x2e63, 0x2e65,
    0x2e67, 0x2e69, 0x2e6d, 0x2e6f, 0x2e71, 0x2e73, 0x2e75, 0x2e77, 0x2e7b, 0x2e7d,
    0x2e7f, 0x2e81,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void pitch_env_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kPitchEnvPcs) / sizeof(kPitchEnvPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kPitchEnvPcs[i], &step_pitch_env);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct PitchEnvSelfRegister
{
    PitchEnvSelfRegister() { hand_register_module(&pitch_env_fill); }
};

PitchEnvSelfRegister g_pitch_env_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
