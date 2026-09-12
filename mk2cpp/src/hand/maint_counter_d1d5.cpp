/*
 * HAND maint_counter_d1d5.cpp - maintenance counter loop over dp:d1d5 (cp4 0x433EA..0x43429).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1); the per-PC case body below is an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). The semantic rewrite is
 * M4 closure step 2 and changes neither addresses nor registration.
 *
 * Semantics: dp:0xd1cb bit2/1/3 gate plus a decrement loop over the 0x1f entries at dp:0xd1d5; calls 0x3532 for entries that pass. Caller: bsr8 at 0x433B7 (A5); single return via rts at 0x43429. Fall-through PCs 0x433F8..0x43418 are statically reachable but dynamically unproven in the recorded windows (C bytes).
 * Evidence: out/m4/28_shared_status.md 1.2; out/m4/30_tail_status.md. Confidence: C bytes (role naming I where noted).
 *
 * Shared H8 primitives live in hand_prims.h (mk2c::hand_prim). Registration:
 * all PCs self-register via MK2CPP_HandRegisterRoutine from a file-static
 * initializer (hand_registry.h); no shared aggregator file is edited and
 * duplicate registration is fatal (mk2cpp.cpp).
 */
#include "hand_prims.h"
#include "hand_registry.h"

namespace mk2c {
namespace {
using namespace mk2c::hand_prim;

/* ======================================================================
 * cp4 0x433EA..0x43429, 26 PC: dp:0xd1cb bit2/bit1/bit3 gate over the
 * dp:0xd1d5 counter, then an 0x1f-entry loop that walks dp:0xd435 + r0
 * and calls 0x3532 (gen-owned) for entries that pass.  Caller bsr8 at
 * 0x433B7 (A5); single return via the rts at 0x43429.  The fall-through
 * PCs 0x433F8..0x43418 are statically reachable but dynamically unproven
 * in the recorded windows (C bytes).
 * ====================================================================== */

uint32_t step_maint_counter_d1d5(void)
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

/* cp4 0x433EA..0x43429 (26 PC) -> step_maint_counter_d1d5 */
const uint16_t kMaintCounterD1d5Pcs[] = {
    0x33ea, 0x33ed, 0x33f0, 0x33f2, 0x33f4, 0x33f6, 0x33f8, 0x33fa,
    0x33fc, 0x3400, 0x3402, 0x3404, 0x3406, 0x340a, 0x340c, 0x340e,
    0x3410, 0x3412, 0x3415, 0x3417, 0x3419, 0x341b, 0x341e, 0x3422,
    0x3424, 0x3429,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_counter_d1d5_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kMaintCounterD1d5Pcs) / sizeof(kMaintCounterD1d5Pcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kMaintCounterD1d5Pcs[i], &step_maint_counter_d1d5);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct MaintCounterD1d5SelfRegister
{
    MaintCounterD1d5SelfRegister() { hand_register_module(&maint_counter_d1d5_fill); }
};

MaintCounterD1d5SelfRegister g_maint_counter_d1d5_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
