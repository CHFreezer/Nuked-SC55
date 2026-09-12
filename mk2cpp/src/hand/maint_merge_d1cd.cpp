/*
 * HAND maint_merge_d1cd.cpp - maintenance bit merge on dp:d1cd/dp:d1cf (cp4 0x433D1..0x433E9).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1); the per-PC case body below is an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). The semantic rewrite is
 * M4 closure step 2 and changes neither addresses nor registration.
 *
 * Semantics: Reads dp:0xd1cf and merges bits with dp:0xd1cd (AND/OR/NOT/XOR on low bytes). Caller: bsr at 0x4338C (A5). C bytes / I dynamic (not executed in the recorded windows in 28).
 * Evidence: out/m4/28_shared_status.md 1.2. Confidence: C bytes (role naming I where noted).
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
 * cp4 0x433D1..0x433E9, 10 PC: read d1cf, merge bits with d1cd:
 * d1cd = (d1cd & (d1cf | ~d1cf)) ... exact per-instruction transcription
 * below (AND/OR/NOT/XOR on low bytes).  Caller bsr at 0x4338C (A5).
 * C bytes / I dynamic (not executed in the recorded boot/demo windows).
 * ====================================================================== */

uint32_t step_maint_merge_d1cd(void)
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

/* cp4 0x433D1..0x433E9 (10 PC) -> step_maint_merge_d1cd */
const uint16_t kMaintMergeD1cdPcs[] = {
    0x33d1, 0x33d5, 0x33d9, 0x33db, 0x33dd, 0x33df, 0x33e1, 0x33e3,
    0x33e5, 0x33e9,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_merge_d1cd_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kMaintMergeD1cdPcs) / sizeof(kMaintMergeD1cdPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kMaintMergeD1cdPcs[i], &step_maint_merge_d1cd);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct MaintMergeD1cdSelfRegister
{
    MaintMergeD1cdSelfRegister() { hand_register_module(&maint_merge_d1cd_fill); }
};

MaintMergeD1cdSelfRegister g_maint_merge_d1cd_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
