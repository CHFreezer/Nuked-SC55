/*
 * HAND pcm/cmd_ring — cp0 command-ring state handlers (rom1 cp=0,
 * flat 0x0625..0x0673).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). The per-PC case bodies below are an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC; see pcm_irq_service.cpp for the L0 rationale and the
 * pending semantic rewrite (M4 closure step 2).
 *
 * Semantics: the ring dispatcher at 0x5f0 (jsr r6) selects six per-state
 * handlers through the table at 0x5f4; the handlers drive voice/part work
 * through the pool and mask_acc paths. The same entry PCs are also used from
 * cp4 0x4062b/0x40674 (pool tail); cp0 and cp4 flat values differ and do not
 * collide.
 * Evidence: out/m4/33_dynamic_class.md 3.3; out/m4/35_pool_tail_status.md;
 * cov c=227,231,988. Confidence: C for bytes / S for state semantics.
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

uint32_t step_cmd_ring(void)
{
    switch (mcu.pc)
    {

    /* ---- ring ---- */
    case 0x0625: /* pjsr #0x04:062b -- push 0x629,cp; pool 0x4062b */
    {
        MCU_PushStack(0x0629);
        MCU_PushStack(mcu.cp);
        mcu.cp = 0x04;
        mcu.pc = 0x062b;
    }
    break;
    case 0x0629: /* rts */
    {
        mcu.pc = 0x062a;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x062a: /* pjsr #0x04:0674 -- push 0x62e,cp; pool 0x40674 */
    {
        MCU_PushStack(0x062e);
        MCU_PushStack(mcu.cp);
        mcu.cp = 0x04;
        mcu.pc = 0x0674;
    }
    break;
    case 0x062e: /* bsr16 -> 0x1ad3 (mask_acc) */
    {
        mcu.pc = 0x0631;
        uint16_t disp = (uint16_t)(0x14 << 8);
        disp |= (uint16_t)0xa2;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x0631: /* rts */
    {
        mcu.pc = 0x0632;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x0632: /* MOVG #0xff -> @r3+0xa050 */
    {
        mcu.pc = 0x0637;
        uint32_t odisp = (uint32_t)0xa0;
        odisp = (odisp << 8) | (uint32_t)0x50;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0xff;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x0637: /* BSET @r3+0xa060 #0 */
    {
        mcu.pc = 0x063b;
        uint32_t odisp = (uint32_t)0xa0;
        odisp = (odisp << 8) | (uint32_t)0x60;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0xc0;
        uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
        uint32_t bit = 0;
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
        data |= 1u << bit;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
    }
    break;
    case 0x063b: /* rts */
    {
        mcu.pc = 0x063c;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x063c: /* CLR @r3+0xa060 */
    {
        mcu.pc = 0x0640;
        uint32_t odisp = (uint32_t)0xa0;
        odisp = (odisp << 8) | (uint32_t)0x60;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x13;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x0640: /* rts */
    {
        mcu.pc = 0x0641;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x0653: /* bsr16 -> 0x14ad */
    {
        mcu.pc = 0x0656;
        uint16_t disp = (uint16_t)(0x0e << 8);
        disp |= (uint16_t)0x57;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x0656: /* rts */
    {
        mcu.pc = 0x0657;
        mcu.pc = MCU_PopStack();
    }
    break;
    case 0x0660: /* CLR @r3+0xa060 */
    {
        mcu.pc = 0x0664;
        uint32_t odisp = (uint32_t)0xa0;
        odisp = (odisp << 8) | (uint32_t)0x60;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x13;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(0));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;
    case 0x0664: /* MOVG #0xff -> @r3+0xa050 */
    {
        mcu.pc = 0x0669;
        uint32_t odisp = (uint32_t)0xa0;
        odisp = (odisp << 8) | (uint32_t)0x50;
        uint32_t oea = (uint32_t)mcu.r[3] + odisp;
        oea &= 0xffff;
        uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
        uint8_t op2 = 0x06;
        uint32_t d = (uint32_t)(int8_t)0xff;
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
        MCU_SetStatusCommon(d, 0);
    }
    break;
    case 0x0669: /* bsr16 -> 0x151e (scan_b) */
    {
        mcu.pc = 0x066c;
        uint16_t disp = (uint16_t)(0x0e << 8);
        disp |= (uint16_t)0xb2;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x066c: /* pjsr #0x04:0674 -- push 0x670,cp; pool 0x40674 */
    {
        MCU_PushStack(0x0670);
        MCU_PushStack(mcu.cp);
        mcu.cp = 0x04;
        mcu.pc = 0x0674;
    }
    break;
    case 0x0670: /* bsr16 -> 0x1ad3 (mask_acc) */
    {
        mcu.pc = 0x0673;
        uint16_t disp = (uint16_t)(0x14 << 8);
        disp |= (uint16_t)0x60;
        MCU_PushStack(mcu.pc);
        mcu.pc += disp;
    }
    break;
    case 0x0673: /* rts */
    {
        mcu.pc = 0x0674;
        mcu.pc = MCU_PopStack();
    }
    break;
    default:
        stock_instruction();
        break;
    }
    return 1;
}

/* cmd_ring: 18 PCs -> step_cmd_ring */
const uint16_t kCmdRingPcs[] = {
    0x625, 0x629, 0x62a, 0x62e, 0x631, 0x632, 0x637, 0x63b, 0x63c, 0x640,
    0x653, 0x656, 0x660, 0x664, 0x669, 0x66c, 0x670, 0x673,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void cmd_ring_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kCmdRingPcs) / sizeof(kCmdRingPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kCmdRingPcs[i], &step_cmd_ring);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct CmdRingSelfRegister
{
    CmdRingSelfRegister() { hand_register_module(&cmd_ring_fill); }
};

CmdRingSelfRegister g_cmd_ring_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
