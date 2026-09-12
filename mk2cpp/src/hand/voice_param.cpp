/*
 * HAND pcm/voice_param — per-voice parameter processing (class C9,
 * rom1 cp=0, flat 0x2e83..0x30f0).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). The per-PC case bodies below are an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC; see pcm_irq_service.cpp for the L0 rationale and the
 * pending semantic rewrite (M4 closure step 2).
 *
 * Semantics: per-voice parameter processing; entry arms 0x2e83/0x2e85
 * (trampoline + body), reads/clears the acf2 fields at 0x2f17/0x2f1e and
 * pairs with mask_acc (B7); the tail 0x3061..0x30f0 BRA returns to ts_scan
 * (0x591c). Called from ts_scan.
 * Evidence: out/m4/18_closure_gap.md 1.3 C9 / 4.2 row 10; out/m4/16 §0-1.2;
 * docs/07:143. Confidence: C (ROM bytes + dasm).
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

uint32_t step_voice_param(void)
{
    switch (mcu.pc)
    {

    /* ---- c9 ---- */
    case 0x2e82: /* rts */
    {
        mcu.pc = 0x2e83;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x2e83: /* bsr -3 -> 0x2e82 */
    {
        mcu.pc = 0x2e85;
        uint16_t disp = (uint16_t)(int8_t)0xfd;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2e85: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x2e89;
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
    case 0x2e89: /* MOVG #0x00ff -> @r0+-26 */
    {
        mcu.pc = 0x2e8e;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe6;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0xff;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2e8e: /* SUB @r0+0 #0x000e */
    {
        mcu.pc = 0x2e93;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x0e;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2e93: /* BCC 0x01cb -> 0x3061 */
    {
        mcu.pc = 0x2e96;
        uint16_t disp = (uint16_t)(0x01 << 8);
        disp |= 0xcb;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e96: /* SUB @r0+0 #0x0000 */
    {
        mcu.pc = 0x2e9b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x00;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2e9b: /* BEQ 0x0076 -> 0x2f14 */
    {
        mcu.pc = 0x2e9e;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0x76;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2e9e: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2ea1;
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
    case 0x2ea1: /* movs r1 @(br,$3e) */
    {
        mcu.pc = 0x2ea3;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3e;
        uint32_t data = (uint32_t)(mcu.r[1] & 0xff);
        MCU_Write(addr, (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2ea3: /* SUB @r0+26 #0xff00 */
    {
        mcu.pc = 0x2ea8;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0xff;
        t2 = (t2 << 8) | (uint32_t)0x00;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2ea8: /* BEQ 16 -> 0x2eba */
    {
        mcu.pc = 0x2eaa;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2eaa: /* MOVG #0xff00 -> (br,$16) */
    {
        mcu.pc = 0x2eaf;
        uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x16;
        oea &= 0xffff;
        uint32_t oep = 0;
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0xff;
        d = (d << 8) | (uint32_t)0x00;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2eaf: /* movl r5 @(br,$32) */
    {
        mcu.pc = 0x2eb1;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x32;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2eb1: /* movlw r5 @(br,$3a) */
    {
        mcu.pc = 0x2eb3;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3a;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint16_t data = MCU_Read16(addr);
        mcu.r[5] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2eb3: /* ADD r5 r5 */
    {
        mcu.pc = 0x2eb5;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[5];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;
    case 0x2eb5: /* MOVG3 r5 -> @r0+24 */
    {
        mcu.pc = 0x2eb8;
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
    case 0x2eb8: /* BRA 11 -> 0x2ec5 */
    {
        mcu.pc = 0x2eba;
        uint16_t disp = (uint16_t)(int8_t)0x0b;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2eba: /* MOVG2 @r0+24 r5 */
    {
        mcu.pc = 0x2ebd;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x18;
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
    case 0x2ebd: /* SHLR r5 */
    {
        mcu.pc = 0x2ebf;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[5] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ebf: /* movsw r5 @(br,$32) */
    {
        mcu.pc = 0x2ec1;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x32;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write16(addr, (uint16_t)data);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ec1: /* bsr -65 -> 0x2e82 */
    {
        mcu.pc = 0x2ec3;
        uint16_t disp = (uint16_t)(int8_t)0xbf;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2ec3: /* movsw r5 @(br,$32) */
    {
        mcu.pc = 0x2ec5;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x32;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write16(addr, (uint16_t)data);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ec5: /* SUB @r0+30 #0xff00 */
    {
        mcu.pc = 0x2eca;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0xff;
        t2 = (t2 << 8) | (uint32_t)0x00;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2eca: /* BEQ 16 -> 0x2edc */
    {
        mcu.pc = 0x2ecc;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2ecc: /* MOVG #0xff00 -> (br,$18) */
    {
        mcu.pc = 0x2ed1;
        uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x18;
        oea &= 0xffff;
        uint32_t oep = 0;
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0xff;
        d = (d << 8) | (uint32_t)0x00;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2ed1: /* movl r5 @(br,$34) */
    {
        mcu.pc = 0x2ed3;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x34;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2ed3: /* movlw r5 @(br,$3a) */
    {
        mcu.pc = 0x2ed5;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3a;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint16_t data = MCU_Read16(addr);
        mcu.r[5] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ed5: /* ADD r5 r5 */
    {
        mcu.pc = 0x2ed7;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[5];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;
    case 0x2ed7: /* MOVG3 r5 -> @r0+28 */
    {
        mcu.pc = 0x2eda;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1c;
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
    case 0x2eda: /* BRA 11 -> 0x2ee7 */
    {
        mcu.pc = 0x2edc;
        uint16_t disp = (uint16_t)(int8_t)0x0b;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2edc: /* MOVG2 @r0+28 r5 */
    {
        mcu.pc = 0x2edf;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1c;
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
    case 0x2edf: /* SHLR r5 */
    {
        mcu.pc = 0x2ee1;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[5] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ee1: /* movsw r5 @(br,$34) */
    {
        mcu.pc = 0x2ee3;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x34;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write16(addr, (uint16_t)data);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ee3: /* bsr -99 -> 0x2e82 */
    {
        mcu.pc = 0x2ee5;
        uint16_t disp = (uint16_t)(int8_t)0x9d;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x2ee5: /* movsw r5 @(br,$34) */
    {
        mcu.pc = 0x2ee7;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x34;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write16(addr, (uint16_t)data);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2ee7: /* MOVG2 @r0+28 r5 */
    {
        mcu.pc = 0x2eea;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1c;
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
    case 0x2eea: /* SWAP r5 */
    {
        mcu.pc = 0x2eec;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2eec: /* cmp r5,b #0xff */
    {
        mcu.pc = 0x2eee;
        int32_t t2 = (int32_t)0xff;
        int32_t t1 = (int32_t)mcu.r[5];
        MCU_SUB_Common(t1, t2, 0, 0);
    }
    break;
    case 0x2eee: /* BNE 2 -> 0x2ef2 */
    {
        mcu.pc = 0x2ef0;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2ef0: /* move r5 #0xfe */
    {
        mcu.pc = 0x2ef2;
        uint8_t data = 0xfe;
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2ef2: /* MOVG3 r5 -> @r1+0xad0e */
    {
        mcu.pc = 0x2ef6;
        uint32_t odisp = (uint32_t)0xad;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2ef6: /* SUB @r0+38 #0xff00 */
    {
        mcu.pc = 0x2efb;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x26;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0xff;
        t2 = (t2 << 8) | (uint32_t)0x00;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2efb: /* BEQ 16 -> 0x2f0d */
    {
        mcu.pc = 0x2efd;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2efd: /* MOVG #0xff00 -> (br,$1a) */
    {
        mcu.pc = 0x2f02;
        uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x1a;
        oea &= 0xffff;
        uint32_t oep = 0;
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0xff;
        d = (d << 8) | (uint32_t)0x00;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f02: /* movl r5 @(br,$36) */
    {
        mcu.pc = 0x2f04;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x36;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2f04: /* movlw r5 @(br,$3a) */
    {
        mcu.pc = 0x2f06;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3a;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint16_t data = MCU_Read16(addr);
        mcu.r[5] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2f06: /* ADD r5 r5 */
    {
        mcu.pc = 0x2f08;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[5];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;
    case 0x2f08: /* MOVG3 r5 -> @r0+36 */
    {
        mcu.pc = 0x2f0b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x24;
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
    case 0x2f0b: /* BRA 7 -> 0x2f14 */
    {
        mcu.pc = 0x2f0d;
        uint16_t disp = (uint16_t)(int8_t)0x07;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2f0d: /* MOVG2 @r0+36 r5 */
    {
        mcu.pc = 0x2f10;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x24;
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
    case 0x2f10: /* SHLR r5 */
    {
        mcu.pc = 0x2f12;
        uint8_t op2 = 0x1b;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t C = data & 1;
        data >>= 1;
        mcu.r[5] = (uint16_t)(data);
        MCU_SetStatus(C, STATUS_C);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2f12: /* movsw r5 @(br,$36) */
    {
        mcu.pc = 0x2f14;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x36;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write16(addr, (uint16_t)data);
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2f14: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2f17;
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
    case 0x2f17: /* TST @r1+0xacf2 */
    {
        mcu.pc = 0x2f1b;
        uint32_t odisp = (uint32_t)0xac;
        odisp = (odisp << 8) | (uint32_t)0xf2;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SetStatusCommon(data, 0);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2f1b: /* BEQ 0x00d0 -> 0x2fee */
    {
        mcu.pc = 0x2f1e;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0xd0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2f1e: /* CLR @r1+0xacf2 */
    {
        mcu.pc = 0x2f22;
        uint32_t odisp = (uint32_t)0xac;
        odisp = (odisp << 8) | (uint32_t)0xf2;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x13;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2f22: /* SUB @r0+0 #0x000c */
    {
        mcu.pc = 0x2f27;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x0c;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2f27: /* BCC 0x00c4 -> 0x2fee */
    {
        mcu.pc = 0x2f2a;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0xc4;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2f2a: /* SUB @r0+0 #0x0000 */
    {
        mcu.pc = 0x2f2f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x00;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2f2f: /* BNE 44 -> 0x2f5d */
    {
        mcu.pc = 0x2f31;
        uint16_t disp = (uint16_t)(int8_t)0x2c;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2f31: /* TST @r0+16 */
    {
        mcu.pc = 0x2f34;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x10;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2f34: /* BNE 18 -> 0x2f48 */
    {
        mcu.pc = 0x2f36;
        uint16_t disp = (uint16_t)(int8_t)0x12;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2f36: /* MOVG #0x0002 -> @r0+0 */
    {
        mcu.pc = 0x2f3b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x02;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f3b: /* MOVG #0x0002 -> @r0+2 */
    {
        mcu.pc = 0x2f40;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x02;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x02;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f40: /* MOVG #0x0002 -> @r0+4 */
    {
        mcu.pc = 0x2f45;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x04;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x02;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f45: /* BRA 0x00a6 -> 0x2fee */
    {
        mcu.pc = 0x2f48;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0xa6;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2f48: /* MOVG #0x0016 -> @r0+0 */
    {
        mcu.pc = 0x2f4d;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f4d: /* MOVG #0x0016 -> @r0+2 */
    {
        mcu.pc = 0x2f52;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x02;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f52: /* MOVG #0x0016 -> @r0+4 */
    {
        mcu.pc = 0x2f57;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x04;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f57: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x2f5a;
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
    case 0x2f5a: /* BRA 0x0134 -> 0x3091 */
    {
        mcu.pc = 0x2f5d;
        uint16_t disp = (uint16_t)(0x01 << 8);
        disp |= 0x34;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2f5d: /* MOVG #0x000c -> @r0+0 */
    {
        mcu.pc = 0x2f62;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x0c;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f62: /* MOVG #0x000c -> @r0+2 */
    {
        mcu.pc = 0x2f67;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x02;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x0c;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f67: /* MOVG #0x000c -> @r0+4 */
    {
        mcu.pc = 0x2f6c;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x04;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0x0c;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2f6c: /* MOVG2 @r0+28 r5 */
    {
        mcu.pc = 0x2f6f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1c;
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
    case 0x2f6f: /* SWAP r5 */
    {
        mcu.pc = 0x2f71;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2f71: /* MOVG3 r5 -> @r0+96 */
    {
        mcu.pc = 0x2f74;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x60;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2f74: /* CLR @r0+97 */
    {
        mcu.pc = 0x2f77;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x61;
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
    case 0x2f77: /* CLR @r0+8 */
    {
        mcu.pc = 0x2f7a;
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
    case 0x2f7a: /* CLR @r0+18 */
    {
        mcu.pc = 0x2f7d;
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
    case 0x2f7d: /* MOVG2 @r0+83 r5 */
    {
        mcu.pc = 0x2f80;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x53;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2f80: /* MOVG3 r5 -> @r0+79 */
    {
        mcu.pc = 0x2f83;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x4f;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2f83: /* MOVG2 @r0+-4 r5 */
    {
        mcu.pc = 0x2f86;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfc;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2f86: /* MOVG3 r5 -> @r0+-8 */
    {
        mcu.pc = 0x2f89;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xf8;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2f89: /* CLR @r0+10 */
    {
        mcu.pc = 0x2f8c;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x0a;
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
    case 0x2f8c: /* CLR @r0+20 */
    {
        mcu.pc = 0x2f8f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x14;
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
    case 0x2f8f: /* MOVG2 @r0+78 r5 */
    {
        mcu.pc = 0x2f92;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x4e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x85;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2f92: /* MOVG3 r5 -> @r0+74 */
    {
        mcu.pc = 0x2f95;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x4a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2f95: /* MOVG2 @r0+32 r5 */
    {
        mcu.pc = 0x2f98;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x20;
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
    case 0x2f98: /* MOVG3 r5 -> @r0+84 */
    {
        mcu.pc = 0x2f9b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x54;
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
    case 0x2f9b: /* MOVG2 @r0+94 r5 */
    {
        mcu.pc = 0x2f9e;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x5e;
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
    case 0x2f9e: /* MOVG3 r5 -> @r0+86 */
    {
        mcu.pc = 0x2fa1;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x56;
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
    case 0x2fa1: /* MOVG2 @r0+44 r4 */
    {
        mcu.pc = 0x2fa4;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x2c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2fa4: /* MOVG2 @r0+68 r5 */
    {
        mcu.pc = 0x2fa7;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x44;
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
    case 0x2fa7: /* MOVG3 r4 -> @r0+106 */
    {
        mcu.pc = 0x2faa;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x6a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x94;
        uint32_t data = (uint32_t)mcu.r[4];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2faa: /* MOVG3 r5 -> @r0+112 */
    {
        mcu.pc = 0x2fad;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x70;
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
    case 0x2fad: /* MOVG2 @r0+111 r4 */
    {
        mcu.pc = 0x2fb0;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x6f;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)(data & 0xff);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2fb0: /* MOVG2 @r0+122 r5 */
    {
        mcu.pc = 0x2fb3;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x7a;
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
    case 0x2fb3: /* MOVG3 r4 -> @r0+107 */
    {
        mcu.pc = 0x2fb6;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x6b;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x94;
        uint32_t data = (uint32_t)mcu.r[4];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2fb6: /* MOVG3 r5 -> @r0+114 */
    {
        mcu.pc = 0x2fb9;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x72;
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
    case 0x2fb9: /* CMP @r0+44 r4 */
    {
        mcu.pc = 0x2fbc;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x2c;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x74;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    }
    break;
    case 0x2fbc: /* BNE 3 -> 0x2fc1 */
    {
        mcu.pc = 0x2fbe;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2fbe: /* CMP @r0+68 r5 */
    {
        mcu.pc = 0x2fc1;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x44;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x75;
        int32_t t1 = (int32_t)mcu.r[5];
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2fc1: /* BCC 6 -> 0x2fc9 */
    {
        mcu.pc = 0x2fc3;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2fc3: /* MOVG #0x02 -> @r0+-3 */
    {
        mcu.pc = 0x2fc7;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfd;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x02;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2fc7: /* BRA 4 -> 0x2fcd */
    {
        mcu.pc = 0x2fc9;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2fc9: /* MOVG #0x00 -> @r0+-3 */
    {
        mcu.pc = 0x2fcd;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfd;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x00;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x2fcd: /* MOVG2 @r0+0x0084 r5 */
    {
        mcu.pc = 0x2fd1;
        uint32_t odisp = (uint32_t)0x00;
        odisp = (odisp << 8) | (uint32_t)0x84;
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
    case 0x2fd1: /* MOVG3 r5 -> @r0+124 */
    {
        mcu.pc = 0x2fd4;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x7c;
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
    case 0x2fd4: /* CLR @r0+12 */
    {
        mcu.pc = 0x2fd7;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x0c;
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
    case 0x2fd7: /* CLR @r0+22 */
    {
        mcu.pc = 0x2fda;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x16;
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
    case 0x2fda: /* TST @r0+-78 */
    {
        mcu.pc = 0x2fdd;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb2;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2fdd: /* BNE 5 -> 0x2fe4 */
    {
        mcu.pc = 0x2fdf;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2fdf: /* MOVG #0xffff -> @r0+-78 */
    {
        mcu.pc = 0x2fe4;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb2;
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
    case 0x2fe4: /* TST @r0+-112 */
    {
        mcu.pc = 0x2fe7;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x90;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2fe7: /* BNE 5 -> 0x2fee */
    {
        mcu.pc = 0x2fe9;
        uint16_t disp = (uint16_t)(int8_t)0x05;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2fe9: /* MOVG #0xffff -> @r0+-112 */
    {
        mcu.pc = 0x2fee;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x90;
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
    case 0x2fee: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x2ff2;
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
    case 0x2ff2: /* nop */
    {
        mcu.pc = 0x2ff3;
        /* nop */
    }
    break;
    case 0x2ff3: /* nop */
    {
        mcu.pc = 0x2ff4;
        /* nop */
    }
    break;
    case 0x2ff4: /* nop */
    {
        mcu.pc = 0x2ff5;
        /* nop */
    }
    break;
    case 0x2ff5: /* nop */
    {
        mcu.pc = 0x2ff6;
        /* nop */
    }
    break;
    case 0x2ff6: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x2ffa;
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
    case 0x2ffa: /* SUB @r0+0 #0x000e */
    {
        mcu.pc = 0x2fff;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x0e;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2fff: /* BCC 96 -> 0x3061 */
    {
        mcu.pc = 0x3001;
        uint16_t disp = (uint16_t)(int8_t)0x60;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3001: /* bsr16 -> 0x37fe */
    {
        mcu.pc = 0x3004;
        uint16_t disp = (uint16_t)(0x07 << 8);
        disp |= 0xfa;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x3004: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x3008;
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
    case 0x3008: /* nop */
    {
        mcu.pc = 0x3009;
        /* nop */
    }
    break;
    case 0x3009: /* nop */
    {
        mcu.pc = 0x300a;
        /* nop */
    }
    break;
    case 0x300a: /* nop */
    {
        mcu.pc = 0x300b;
        /* nop */
    }
    break;
    case 0x300b: /* nop */
    {
        mcu.pc = 0x300c;
        /* nop */
    }
    break;
    case 0x300c: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x3010;
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
    case 0x3010: /* SUB @r0+0 #0x000e */
    {
        mcu.pc = 0x3015;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x0e;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x3015: /* BCC 74 -> 0x3061 */
    {
        mcu.pc = 0x3017;
        uint16_t disp = (uint16_t)(int8_t)0x4a;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3017: /* bsr16 -> 0x30f2 */
    {
        mcu.pc = 0x301a;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0xd8;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x301a: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x301e;
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
    case 0x301e: /* nop */
    {
        mcu.pc = 0x301f;
        /* nop */
    }
    break;
    case 0x301f: /* nop */
    {
        mcu.pc = 0x3020;
        /* nop */
    }
    break;
    case 0x3020: /* nop */
    {
        mcu.pc = 0x3021;
        /* nop */
    }
    break;
    case 0x3021: /* nop */
    {
        mcu.pc = 0x3022;
        /* nop */
    }
    break;
    case 0x3022: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x3026;
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
    case 0x3026: /* SUB @r0+0 #0x000e */
    {
        mcu.pc = 0x302b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x0e;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x302b: /* BCC 52 -> 0x3061 */
    {
        mcu.pc = 0x302d;
        uint16_t disp = (uint16_t)(int8_t)0x34;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x302d: /* bsr16 -> 0x41bb */
    {
        mcu.pc = 0x3030;
        uint16_t disp = (uint16_t)(0x11 << 8);
        disp |= 0x8b;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x3030: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x3034;
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
    case 0x3034: /* nop */
    {
        mcu.pc = 0x3035;
        /* nop */
    }
    break;
    case 0x3035: /* nop */
    {
        mcu.pc = 0x3036;
        /* nop */
    }
    break;
    case 0x3036: /* nop */
    {
        mcu.pc = 0x3037;
        /* nop */
    }
    break;
    case 0x3037: /* nop */
    {
        mcu.pc = 0x3038;
        /* nop */
    }
    break;
    case 0x3038: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x303c;
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
    case 0x303c: /* SUB @r0+0 #0x000e */
    {
        mcu.pc = 0x3041;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x0e;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x3041: /* BCC 30 -> 0x3061 */
    {
        mcu.pc = 0x3043;
        uint16_t disp = (uint16_t)(int8_t)0x1e;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3043: /* bsr16 -> 0x4d62 */
    {
        mcu.pc = 0x3046;
        uint16_t disp = (uint16_t)(0x1d << 8);
        disp |= 0x1c;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x3046: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x304a;
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
    case 0x304a: /* nop */
    {
        mcu.pc = 0x304b;
        /* nop */
    }
    break;
    case 0x304b: /* nop */
    {
        mcu.pc = 0x304c;
        /* nop */
    }
    break;
    case 0x304c: /* nop */
    {
        mcu.pc = 0x304d;
        /* nop */
    }
    break;
    case 0x304d: /* nop */
    {
        mcu.pc = 0x304e;
        /* nop */
    }
    break;
    case 0x304e: /* BSET_ORC #0x0700 r0 */
    {
        mcu.pc = 0x3052;
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
    case 0x3052: /* SUB @r0+0 #0x000e */
    {
        mcu.pc = 0x3057;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x05;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint32_t)0x00;
        t2 = (t2 << 8) | (uint32_t)0x0e;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x3057: /* BCC 8 -> 0x3061 */
    {
        mcu.pc = 0x3059;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3059: /* bsr16 -> 0x3465 */
    {
        mcu.pc = 0x305c;
        uint16_t disp = (uint16_t)(0x04 << 8);
        disp |= 0x09;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x305c: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x3060;
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
    case 0x3060: /* rts */
    {
        mcu.pc = 0x3061;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x3061: /* SUB @r0+0 #0x10 */
    {
        mcu.pc = 0x3065;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x04;
        // TODO(gt): MOVG_Immediate ore=4 word: GT reads one imm8 sign-extended (src/mcu_opcodes.cpp:847 FIXME)
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint16_t)(int8_t)0x10;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x3065: /* BEQ 15 -> 0x3076 */
    {
        mcu.pc = 0x3067;
        uint16_t disp = (uint16_t)(int8_t)0x0f;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3067: /* SUB @r0+0 #0x0e */
    {
        mcu.pc = 0x306b;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x04;
        // TODO(gt): MOVG_Immediate ore=4 word: GT reads one imm8 sign-extended (src/mcu_opcodes.cpp:847 FIXME)
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t t2 = (uint16_t)(int8_t)0x0e;
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x306b: /* BNE 120 -> 0x30e5 */
    {
        mcu.pc = 0x306d;
        uint16_t disp = (uint16_t)(int8_t)0x78;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x306d: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x3070;
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
    case 0x3070: /* movs r1 @(br,$3e) */
    {
        mcu.pc = 0x3072;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3e;
        uint32_t data = (uint32_t)(mcu.r[1] & 0xff);
        MCU_Write(addr, (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x3072: /* movl r5 @(br,$32) */
    {
        mcu.pc = 0x3074;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x32;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x3074: /* BRA 7 -> 0x307d */
    {
        mcu.pc = 0x3076;
        uint16_t disp = (uint16_t)(int8_t)0x07;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3076: /* MOVG2 @r0+-2 r1 */
    {
        mcu.pc = 0x3079;
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
    case 0x3079: /* movs r1 @(br,$3e) */
    {
        mcu.pc = 0x307b;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3e;
        uint32_t data = (uint32_t)(mcu.r[1] & 0xff);
        MCU_Write(addr, (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x307b: /* movl r5 @(br,$34) */
    {
        mcu.pc = 0x307d;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x34;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x307d: /* movlw r5 @(br,$3a) */
    {
        mcu.pc = 0x307f;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3a;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint16_t data = MCU_Read16(addr);
        mcu.r[5] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x307f: /* BEQ 16 -> 0x3091 */
    {
        mcu.pc = 0x3081;
        uint16_t disp = (uint16_t)(int8_t)0x10;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3081: /* ADD r5 r5 */
    {
        mcu.pc = 0x3083;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[5];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;
    case 0x3083: /* SWAP r5 */
    {
        mcu.pc = 0x3085;
        uint8_t op2 = 0x10;
        uint32_t data = (uint32_t)mcu.r[5];
        uint32_t data_h = data >> 8;
        uint32_t data_l = data & 0xff;
        data = (data_l << 8) | data_h;
        mcu.r[5] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x3085: /* cmp r5,b #0xff */
    {
        mcu.pc = 0x3087;
        int32_t t2 = (int32_t)0xff;
        int32_t t1 = (int32_t)mcu.r[5];
        MCU_SUB_Common(t1, t2, 0, 0);
    }
    break;
    case 0x3087: /* BNE 2 -> 0x308b */
    {
        mcu.pc = 0x3089;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3089: /* move r5 #0xfe */
    {
        mcu.pc = 0x308b;
        uint8_t data = 0xfe;
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x308b: /* MOVG3 r5 -> @r1+0xad0e */
    {
        mcu.pc = 0x308f;
        uint32_t odisp = (uint32_t)0xad;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x95;
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x308f: /* BRA 84 -> 0x30e5 */
    {
        mcu.pc = 0x3091;
        uint16_t disp = (uint16_t)(int8_t)0x54;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x3091: /* CLR @r1+0xad0e */
    {
        mcu.pc = 0x3095;
        uint32_t odisp = (uint32_t)0xad;
        odisp = (odisp << 8) | (uint32_t)0x0e;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x13;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x3095: /* MOVG #0x16 -> @r0+0 */
    {
        mcu.pc = 0x3099;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0x16;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x3099: /* SUB @r1+0xd0a8 #0xff */
    {
        mcu.pc = 0x309e;
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
    case 0x309e: /* BNE 13 -> 0x30ad */
    {
        mcu.pc = 0x30a0;
        uint16_t disp = (uint16_t)(int8_t)0x0d;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x30a0: /* SUB @r1+0xd0c4 #0xff */
    {
        mcu.pc = 0x30a5;
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
    case 0x30a5: /* BEQ 32 -> 0x30c7 */
    {
        mcu.pc = 0x30a7;
        uint16_t disp = (uint16_t)(int8_t)0x20;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x30a7: /* MOVG2 @r1+0xd0c4 r3 */
    {
        mcu.pc = 0x30ab;
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
    case 0x30ab: /* BRA 4 -> 0x30b1 */
    {
        mcu.pc = 0x30ad;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x30ad: /* MOVG2 @r1+0xd0a8 r3 */
    {
        mcu.pc = 0x30b1;
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
    case 0x30b1: /* EXTU r3 */
    {
        mcu.pc = 0x30b3;
        uint8_t op2 = 0x12;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x30b3: /* MOVG #0xff -> @r1+0xd0a8 */
    {
        mcu.pc = 0x30b8;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xa8;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0xff;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x30b8: /* MOVG #0xff -> @r1+0xd0c4 */
    {
        mcu.pc = 0x30bd;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xc4;
        uint32_t oea = (uint32_t)mcu.r[1] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0xff;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x30bd: /* MOVG #0xff -> @r3+0xd0a8 */
    {
        mcu.pc = 0x30c2;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xa8;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0xff;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x30c2: /* MOVG #0xff -> @r3+0xd0c4 */
    {
        mcu.pc = 0x30c7;
        uint32_t odisp = (uint32_t)0xd0;
        odisp = (odisp << 8) | (uint32_t)0xc4;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0xff;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x30c7: /* movs r1 @(br,$3e) */
    {
        mcu.pc = 0x30c9;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3e;
        uint32_t data = (uint32_t)(mcu.r[1] & 0xff);
        MCU_Write(addr, (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x30c9: /* MOVG #0x00b5 -> (br,$18) */
    {
        mcu.pc = 0x30ce;
        uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x18;
        oea &= 0xffff;
        uint32_t oep = 0;
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0xb5;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x30ce: /* MOVG3 r1 -> (dp,0xa1f6) */
    {
        mcu.pc = 0x30d2;
        uint32_t oea = (uint32_t)0xa1;
        oea = (oea << 8) | (uint32_t)0xf6;
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
    case 0x30d2: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x30d6;
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
    case 0x30d6: /* move r0 #0x01 */
    {
        mcu.pc = 0x30d8;
        uint8_t data = 0x01;
        mcu.r[0] &= ~0xff;
        mcu.r[0] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x30d8: /* move r1 #0x01 */
    {
        mcu.pc = 0x30da;
        uint8_t data = 0x01;
        mcu.r[1] &= ~0xff;
        mcu.r[1] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x30da: /* trapa #0x12 */
    {
        mcu.pc = 0x30dc;
        uint8_t opcode = 0x12;
        if ((opcode & 0xf0) == 0x10)
        MCU_Interrupt_TRAPA(opcode & 0x0f);
        else
        MCU_ErrorTrap();
    }
    break;
    case 0x30dc: /* ADDQ #2 r7 */
    {
        mcu.pc = 0x30de;
        uint8_t op2 = 0x09;
        uint32_t t1 = (uint32_t)mcu.r[7];
        int32_t t2 = 2;
        t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, t2, 0, 1);
        mcu.r[7] = (uint16_t)(t1);
    }
    break;
    case 0x30de: /* MOVG2 (dp,0xd178) r1 */
    {
        mcu.pc = 0x30e2;
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
    case 0x30e2: /* BRA 0x27ef -> 0x58d4 */
    {
        mcu.pc = 0x30e5;
        uint16_t disp = (uint16_t)(0x27 << 8);
        disp |= 0xef;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x30e5: /* BCLR_ANDC #0xf8ff r0 */
    {
        mcu.pc = 0x30e9;
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
    case 0x30e9: /* MOVG2 (dp,0xd178) r1 */
    {
        mcu.pc = 0x30ed;
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
    case 0x30ed: /* ADDQ #2 r7 */
    {
        mcu.pc = 0x30ef;
        uint8_t op2 = 0x09;
        uint32_t t1 = (uint32_t)mcu.r[7];
        int32_t t2 = 2;
        t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, t2, 0, 1);
        mcu.r[7] = (uint16_t)(t1);
    }
    break;
    case 0x30ef: /* BRA 0x27e2 -> 0x58d4 */
    {
        mcu.pc = 0x30f2;
        uint16_t disp = (uint16_t)(0x27 << 8);
        disp |= 0xe2;
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

/* voice_param C9: 213 PCs -> step_voice_param */
const uint16_t kVoiceParamPcs[] = {
    0x2e82, 0x2e83, 0x2e85, 0x2e89, 0x2e8e, 0x2e93, 0x2e96, 0x2e9b, 0x2e9e, 0x2ea1,
    0x2ea3, 0x2ea8, 0x2eaa, 0x2eaf, 0x2eb1, 0x2eb3, 0x2eb5, 0x2eb8, 0x2eba, 0x2ebd,
    0x2ebf, 0x2ec1, 0x2ec3, 0x2ec5, 0x2eca, 0x2ecc, 0x2ed1, 0x2ed3, 0x2ed5, 0x2ed7,
    0x2eda, 0x2edc, 0x2edf, 0x2ee1, 0x2ee3, 0x2ee5, 0x2ee7, 0x2eea, 0x2eec, 0x2eee,
    0x2ef0, 0x2ef2, 0x2ef6, 0x2efb, 0x2efd, 0x2f02, 0x2f04, 0x2f06, 0x2f08, 0x2f0b,
    0x2f0d, 0x2f10, 0x2f12, 0x2f14, 0x2f17, 0x2f1b, 0x2f1e, 0x2f22, 0x2f27, 0x2f2a,
    0x2f2f, 0x2f31, 0x2f34, 0x2f36, 0x2f3b, 0x2f40, 0x2f45, 0x2f48, 0x2f4d, 0x2f52,
    0x2f57, 0x2f5a, 0x2f5d, 0x2f62, 0x2f67, 0x2f6c, 0x2f6f, 0x2f71, 0x2f74, 0x2f77,
    0x2f7a, 0x2f7d, 0x2f80, 0x2f83, 0x2f86, 0x2f89, 0x2f8c, 0x2f8f, 0x2f92, 0x2f95,
    0x2f98, 0x2f9b, 0x2f9e, 0x2fa1, 0x2fa4, 0x2fa7, 0x2faa, 0x2fad, 0x2fb0, 0x2fb3,
    0x2fb6, 0x2fb9, 0x2fbc, 0x2fbe, 0x2fc1, 0x2fc3, 0x2fc7, 0x2fc9, 0x2fcd, 0x2fd1,
    0x2fd4, 0x2fd7, 0x2fda, 0x2fdd, 0x2fdf, 0x2fe4, 0x2fe7, 0x2fe9, 0x2fee, 0x2ff2,
    0x2ff3, 0x2ff4, 0x2ff5, 0x2ff6, 0x2ffa, 0x2fff, 0x3001, 0x3004, 0x3008, 0x3009,
    0x300a, 0x300b, 0x300c, 0x3010, 0x3015, 0x3017, 0x301a, 0x301e, 0x301f, 0x3020,
    0x3021, 0x3022, 0x3026, 0x302b, 0x302d, 0x3030, 0x3034, 0x3035, 0x3036, 0x3037,
    0x3038, 0x303c, 0x3041, 0x3043, 0x3046, 0x304a, 0x304b, 0x304c, 0x304d, 0x304e,
    0x3052, 0x3057, 0x3059, 0x305c, 0x3060, 0x3061, 0x3065, 0x3067, 0x306b, 0x306d,
    0x3070, 0x3072, 0x3074, 0x3076, 0x3079, 0x307b, 0x307d, 0x307f, 0x3081, 0x3083,
    0x3085, 0x3087, 0x3089, 0x308b, 0x308f, 0x3091, 0x3095, 0x3099, 0x309e, 0x30a0,
    0x30a5, 0x30a7, 0x30ab, 0x30ad, 0x30b1, 0x30b3, 0x30b8, 0x30bd, 0x30c2, 0x30c7,
    0x30c9, 0x30ce, 0x30d2, 0x30d6, 0x30d8, 0x30da, 0x30dc, 0x30de, 0x30e2, 0x30e5,
    0x30e9, 0x30ed, 0x30ef,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void voice_param_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kVoiceParamPcs) / sizeof(kVoiceParamPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kVoiceParamPcs[i], &step_voice_param);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct VoiceParamSelfRegister
{
    VoiceParamSelfRegister() { hand_register_module(&voice_param_fill); }
};

VoiceParamSelfRegister g_voice_param_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
