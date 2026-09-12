/*
 * HAND shared_nop_rts.cpp - shared nop;rts stub (cp4 0x4142F..0x41430).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1); the per-PC case body below is an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). Semantically rewritten
 * in M4 closure step 2 (2026-09-13); addresses and registration unchanged.
 *
 * Semantics: Two-instruction shared stub: 0x4142F nop, 0x41430 rts. Called by the six A4 bsr sites (0x413D5/D7/FE, 0x41400/16/18) owned by reset_init.cpp.
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
 * cp4 0x4142F..0x41430, 2 PC: shared `nop; rts` stub called by the six A4
 * bsr sites (0x413D5/D7/FE, 0x41400/16/18).  nop/rts, no context.
 * ====================================================================== */

/* 0x4142F nop [00]: consume the opcode byte and fall through to rts. */
void step_nop(void)
{
    mcu.pc = 0x1430;
}

/* 0x41430 rts [19]: pop the return address into pc. */
void step_rts(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void shared_nop_rts_fill(void)
{
    MK2CPP_HandRegister(0x0004142fu, &step_nop);
    MK2CPP_HandRegister(0x00041430u, &step_rts);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct SharedNopRtsSelfRegister
{
    SharedNopRtsSelfRegister() { hand_register_module(&shared_nop_rts_fill); }
};

SharedNopRtsSelfRegister g_shared_nop_rts_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
