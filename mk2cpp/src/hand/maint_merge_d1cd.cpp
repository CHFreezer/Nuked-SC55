/*
 * HAND maint_merge_d1cd.cpp - maintenance bit-equality fold:
 * dp:0xd1cd = ~(dp:0xd1cd ^ dp:0xd1cf) (low byte) (cp4 0x433D1..0x433E9).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13); addresses and registration unchanged. One L0 hand entry per
 * instruction PC so the host keeps its per-instruction interrupt poll,
 * cycles += 12, trace and MIDI/SM cadence (docs/09_m4_integration.md 4.1).
 *
 * Semantics (Confirmed): with A = dp:0xd1cf and B = dp:0xd1cd, the ten stock
 * instructions compute B & A, then (B ^ ~A) & ~A = ~A & ~B, then OR with A & B,
 * i.e. the per-bit equality mask ~(A ^ B), and store it back to dp:0xd1cd.
 * Raw bytes at rom2 file offset 0x33D1 (re-read 2026-09-13): 15 d1 cf 80 /
 * 15 d1 cd 81 / a1 82 / a0 51 / a0 15 / a0 62 / a0 52 / a1 42 / 15 d1 cd 92 / 19.
 * Caller: bsr at 0x4338C inside the rom2 A5 maintenance routine, right after
 * the d1d3 setup. Role naming I (bit business meaning not established) and
 * dynamic arm I (not executed in the recorded boot/demo windows), so the
 * trace/WAV gates cannot exercise this module; equivalence rests on the
 * helpers being literal copies of the previous per-case bodies.
 * Evidence: out/m4/28_shared_status.md 1.2.
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
 * cp4 0x433D1..0x433E9, 10 PC: d1cd = XNOR(d1cd, d1cf) low byte.
 * Caller bsr at 0x4338C (A5); dynamically unexecuted in the recorded
 * boot/demo windows, so the gates verify only registration/build.
 * ====================================================================== */

/* Byte logical ops with the exact bodies the case table used to inline:
 * low-byte result, N/Z from the result, C = 0, V untouched. */
void and8(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)dst;
    data &= (uint32_t)(src & 0xffu);
    dst = (uint16_t)((dst & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(dst, 0);
}

void or8(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)(src & 0xffu);
    dst = (uint16_t)(dst | (uint16_t)data);
    MCU_SetStatusCommon(dst, 0);
}

void xor8(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)(src & 0xffu);
    dst ^= (uint16_t)data;
    MCU_SetStatusCommon(dst, 0);
}

void not8(uint16_t &dst)
{
    uint32_t data = ~(uint32_t)(dst & 0xffu);
    dst = (uint16_t)((dst & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* 0x433D1 mov.b (dp,0xd1cf),r0 -- A = d1cf. */
void step_load_compare(void)
{
    load8(mcu.r[0], dp_addr(0xd1cf));
    mcu.pc = 0x33d5;
}

/* 0x433D5 mov.b (dp,0xd1cd),r1 -- B = d1cd. */
void step_load_target(void)
{
    load8(mcu.r[1], dp_addr(0xd1cd));
    mcu.pc = 0x33d9;
}

/* 0x433D9 mov.b r1,r2 -- seed the result with B. */
void step_seed_result(void)
{
    mov8(mcu.r[2], (uint32_t)mcu.r[1]);
    mcu.pc = 0x33db;
}

/* 0x433DB and.b r0,r1 -- r1 = A & B (bits set in both). */
void step_intersect(void)
{
    and8(mcu.r[1], mcu.r[0]);
    mcu.pc = 0x33dd;
}

/* 0x433DD not.b r0 -- r0 = ~A. */
void step_invert_compare(void)
{
    not8(mcu.r[0]);
    mcu.pc = 0x33df;
}

/* 0x433DF xor.b r0,r2 -- r2 = B ^ ~A. */
void step_xor_inverted(void)
{
    xor8(mcu.r[2], mcu.r[0]);
    mcu.pc = 0x33e1;
}

/* 0x433E1 and.b r0,r2 -- r2 = (B ^ ~A) & ~A = ~A & ~B (bits clear in both). */
void step_intersect_inverted(void)
{
    and8(mcu.r[2], mcu.r[0]);
    mcu.pc = 0x33e3;
}

/* 0x433E3 or.b r1,r2 -- r2 = (A & B) | (~A & ~B) = ~(A ^ B): bit set iff
 * A and B agree. */
void step_fold_equality(void)
{
    or8(mcu.r[2], mcu.r[1]);
    mcu.pc = 0x33e5;
}

/* 0x433E5 mov.b r2,(dp,0xd1cd) -- store the per-bit equality mask. */
void step_store_result(void)
{
    store8(dp_addr(0xd1cd), mcu.r[2]);
    mcu.pc = 0x33e9;
}

/* 0x433E9 rts. */
void step_return(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_merge_d1cd_fill(void)
{
    MK2CPP_HandRegister(0x000433d1u, &step_load_compare);
    MK2CPP_HandRegister(0x000433d5u, &step_load_target);
    MK2CPP_HandRegister(0x000433d9u, &step_seed_result);
    MK2CPP_HandRegister(0x000433dbu, &step_intersect);
    MK2CPP_HandRegister(0x000433ddu, &step_invert_compare);
    MK2CPP_HandRegister(0x000433dfu, &step_xor_inverted);
    MK2CPP_HandRegister(0x000433e1u, &step_intersect_inverted);
    MK2CPP_HandRegister(0x000433e3u, &step_fold_equality);
    MK2CPP_HandRegister(0x000433e5u, &step_store_result);
    MK2CPP_HandRegister(0x000433e9u, &step_return);
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
