/*
 * HAND maint_fill_d1de.cpp - maintenance fill: dp:d202/d204 = d1de (cp4 0x433C4..0x433D0).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1); the per-PC case body below is an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). The semantic rewrite is
 * M4 closure step 2 and changes neither addresses nor registration.
 *
 * Semantics: Writes dp:0xd1de to dp:0xd202 and dp:0xd204, then rts. Nine rom2 callers (0x432D5 A5, 0x3F8E, 0x3FAC, 0x3FC0, 0x495F, 0x67A2, 0x67D4, 0xB610, 0xB98D); leaf and caller-independent.
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
 * cp4 0x433C4..0x433D0, 3 PC: write 0xd1de to dp:0xd202 and dp:0xd204.
 * Entry jsr; callers 0x432D5/0x3F8E/0x3FAC/0x3FC0/0x495F/0x67A2/0x67D4/
 * 0xB610/0xB98D (rom2 offsets); leaf, no context dependency.
 * ====================================================================== */

uint32_t step_maint_fill_d1de(void)
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

/* cp4 0x433C4..0x433D0 (3 PC) -> step_maint_fill_d1de */
const uint16_t kMaintFillD1dePcs[] = {
    0x33c4, 0x33ca, 0x33d0,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_fill_d1de_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kMaintFillD1dePcs) / sizeof(kMaintFillD1dePcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kMaintFillD1dePcs[i], &step_maint_fill_d1de);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct MaintFillD1deSelfRegister
{
    MaintFillD1deSelfRegister() { hand_register_module(&maint_fill_d1de_fill); }
};

MaintFillD1deSelfRegister g_maint_fill_d1de_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
