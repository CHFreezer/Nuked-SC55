/*
 * HAND pcm/pcm_irq_service — PCM dispatcher IRQ service (class C5, rom1 cp=0,
 * flat 0x25f0..0x2666).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). The per-PC case bodies below are an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). The semantic rewrite
 * (nukeykt-style named operations) is M4 closure step 2 (docs/00_plan.md,
 * section "semantic gap") and changes neither addresses nor registration.
 *
 * Semantics: entries arrive from the C2 pcm_dispatcher via BRA 0x51f3 ->
 * 0x25f0; the routine judges the PCM status word (0x0e), reads the device and
 * writes per-voice bytes P+0/P+2/P+4/P+0x26/P+0x30 before returning to the
 * dispatcher at 0x51f6; it falls through into C6 svc_math (0x2669).
 * Evidence: out/m4/18_closure_gap.md 1.3 C5 / 4.2 row 6; dasm:4259-4360;
 * pc_main:1973-2065. Confidence: C (ROM bytes + dasm).
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

uint32_t step_irq_service(void)
{
    switch (mcu.pc)
    {

    /* ---- irq ---- */
    case 0x25f0: /* MOVG2 r1 r2 */
    {
        mcu.pc = 0x25f2;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x25f2: /* ADD r2 r2 */
    {
        mcu.pc = 0x25f4;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;
    case 0x25f4: /* MOVG2 @r2+0x64d6 r0 */
    {
        mcu.pc = 0x25f8;
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
    case 0x25f8: /* SUB @r0+0 #0x000e */
    {
        mcu.pc = 0x25fd;
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
    case 0x25fd: /* BCC 0x2bf6 -> 0x51f6 */
    {
        mcu.pc = 0x2600;
        uint16_t disp = (uint16_t)(0x2b << 8);
        disp |= 0xf6;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2600: /* TST @r0+-17 */
    {
        mcu.pc = 0x2603;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xef;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        MCU_SetStatusCommon(data, 0);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2603: /* BEQ 100 -> 0x2669 */
    {
        mcu.pc = 0x2605;
        uint16_t disp = (uint16_t)(int8_t)0x64;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2605: /* movs r1 @(br,$3e) */
    {
        mcu.pc = 0x2607;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3e;
        uint32_t data = (uint32_t)(mcu.r[1] & 0xff);
        MCU_Write(addr, (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2607: /* movl r4 @(br,$32) */
    {
        mcu.pc = 0x2609;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x32;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x2609: /* movlw r4 @(br,$3a) */
    {
        mcu.pc = 0x260b;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3a;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint16_t data = MCU_Read16(addr);
        mcu.r[4] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x260b: /* movl r5 @(br,$34) */
    {
        mcu.pc = 0x260d;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x34;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[5] &= ~0xff;
        mcu.r[5] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;
    case 0x260d: /* movlw r5 @(br,$3a) */
    {
        mcu.pc = 0x260f;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x3a;
        if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        uint16_t data = MCU_Read16(addr);
        mcu.r[5] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x260f: /* CMP r4 r5 */
    {
        mcu.pc = 0x2611;
        uint8_t op2 = 0x75;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[4];
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;
    case 0x2611: /* BCC 15 -> 0x2622 */
    {
        mcu.pc = 0x2613;
        uint16_t disp = (uint16_t)(int8_t)0x0f;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2613: /* movi r6 #0x000e */
    {
        mcu.pc = 0x2616;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x0e;
        mcu.r[6] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2616: /* MOVG #0x00b5 -> (br,$16) */
    {
        mcu.pc = 0x261b;
        uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x16;
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
    case 0x261b: /* MOVG #0x00b5 -> @r0+26 */
    {
        mcu.pc = 0x2620;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1a;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0xb5;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x2620: /* BRA 13 -> 0x262f */
    {
        mcu.pc = 0x2622;
        uint16_t disp = (uint16_t)(int8_t)0x0d;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2622: /* movi r6 #0x0010 */
    {
        mcu.pc = 0x2625;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x10;
        mcu.r[6] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;
    case 0x2625: /* MOVG #0x00b5 -> (br,$18) */
    {
        mcu.pc = 0x262a;
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
    case 0x262a: /* MOVG #0x00b5 -> @r0+30 */
    {
        mcu.pc = 0x262f;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x1e;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
        uint8_t op2 = 0x07;
        uint32_t d = (uint32_t)0x00;
        d = (d << 8) | (uint32_t)0xb5;
        if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
        MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
        MCU_SetStatusCommon(d, 1);
    }
    break;
    case 0x262f: /* MOVG3 r6 -> @r0+0 */
    {
        mcu.pc = 0x2632;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
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
    case 0x2632: /* MOVG3 r6 -> @r0+2 */
    {
        mcu.pc = 0x2635;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x02;
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
    case 0x2635: /* MOVG3 r6 -> @r0+4 */
    {
        mcu.pc = 0x2638;
        uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x04;
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
    case 0x2638: /* SUB @r1+0xd0a8 #0xff */
    {
        mcu.pc = 0x263d;
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
    case 0x263d: /* BNE 13 -> 0x264c */
    {
        mcu.pc = 0x263f;
        uint16_t disp = (uint16_t)(int8_t)0x0d;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x263f: /* SUB @r1+0xd0c4 #0xff */
    {
        mcu.pc = 0x2644;
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
    case 0x2644: /* BEQ 32 -> 0x2666 */
    {
        mcu.pc = 0x2646;
        uint16_t disp = (uint16_t)(int8_t)0x20;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x2646: /* MOVG2 @r1+0xd0c4 r3 */
    {
        mcu.pc = 0x264a;
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
    case 0x264a: /* BRA 4 -> 0x2650 */
    {
        mcu.pc = 0x264c;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;
    case 0x264c: /* MOVG2 @r1+0xd0a8 r3 */
    {
        mcu.pc = 0x2650;
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
    case 0x2650: /* EXTU r3 */
    {
        mcu.pc = 0x2652;
        uint8_t op2 = 0x12;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x2652: /* MOVG #0xff -> @r1+0xd0a8 */
    {
        mcu.pc = 0x2657;
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
    case 0x2657: /* MOVG #0xff -> @r1+0xd0c4 */
    {
        mcu.pc = 0x265c;
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
    case 0x265c: /* MOVG #0xff -> @r3+0xd0a8 */
    {
        mcu.pc = 0x2661;
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
    case 0x2661: /* MOVG #0xff -> @r3+0xd0c4 */
    {
        mcu.pc = 0x2666;
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
    case 0x2666: /* BRA 0x2b8d -> 0x51f6 */
    {
        mcu.pc = 0x2669;
        uint16_t disp = (uint16_t)(0x2b << 8);
        disp |= 0x8d;
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

/* irq_service C5: 37 PCs -> step_irq_service */
const uint16_t kIrqServicePcs[] = {
    0x25f0, 0x25f2, 0x25f4, 0x25f8, 0x25fd, 0x2600, 0x2603, 0x2605, 0x2607, 0x2609,
    0x260b, 0x260d, 0x260f, 0x2611, 0x2613, 0x2616, 0x261b, 0x2620, 0x2622, 0x2625,
    0x262a, 0x262f, 0x2632, 0x2635, 0x2638, 0x263d, 0x263f, 0x2644, 0x2646, 0x264a,
    0x264c, 0x2650, 0x2652, 0x2657, 0x265c, 0x2661, 0x2666,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void pcm_irq_service_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kIrqServicePcs) / sizeof(kIrqServicePcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kIrqServicePcs[i], &step_irq_service);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct PcmIrqServiceSelfRegister
{
    PcmIrqServiceSelfRegister() { hand_register_module(&pcm_irq_service_fill); }
};

PcmIrqServiceSelfRegister g_pcm_irq_service_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
