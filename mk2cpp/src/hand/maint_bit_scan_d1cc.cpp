/*
 * HAND maint_bit_scan_d1cc.cpp - maintenance bit-0 scan over dp:d1cc, write dp:d1da (cp4 0x43641..0x43694).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1); the per-PC case body below is an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). The semantic rewrite is
 * M4 closure step 2 and changes neither addresses nor registration.
 *
 * Semantics: Scans dp:0xd1cc bit 0 over 0x1f entries, then writes the dp:0xd1d6 table at dp:0xd1da+stride; sets bits in @r3 and clears @r2. Caller: jsr at 0x433B9 (A5); calls 0x43532. The immediate-byte bug at 0x43653 was fixed here in 28 (d2 04 32 -> SUB immediate 0x32).
 * Evidence: out/m4/28_shared_status.md 1.2, 2, 4. Confidence: C bytes (role naming I where noted).
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
 * cp4 0x43641..0x43694, 35 PC: scan dp:0xd1cc bit 0 over 0x1f entries,
 * then write the 0xd1d6 table at 0xd1da+stride; sets bits in @r3 and
 * clears @r2.  Caller jsr at 0x433B9 (A5); calls 0x43532.
 * ====================================================================== */

uint32_t step_maint_bit_scan_d1cc(void)
{
    switch (mcu.pc)
    {
    case 0x3641: /* MOVI r1 #0xd1cc [59 d1 cc] */
        movi16(mcu.r[1], 0xd1cc);
        mcu.pc = 0x3644;
        return 1;
    case 0x3644: /* MOVI r0 #0x001f [58 00 1f] */
        movi16(mcu.r[0], 0x001f);
        mcu.pc = 0x3647;
        return 1;
    case 0x3647: /* BTSTI --r1 #0 [b1 f0] */
    {
        mcu.r[1] -= 1;
        uint32_t data = (uint32_t)MCU_Read(reg_addr(1));
        MCU_SetStatus((data & (1u << 0)) == 0, STATUS_Z);
        mcu.pc = 0x3649;
        return 1;
    }
    case 0x3649: /* BEQ +0x46 -> 0x3691 [27 46] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3691 : 0x364b;
        return 1;
    case 0x364b: /* MOVG2 r0 r2 (byte) [a0 82] */
        mov8(mcu.r[2], (uint32_t)mcu.r[0]);
        mcu.pc = 0x364d;
        return 1;
    case 0x364d: /* EXTU r2 [a2 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[2] & 0xffu);
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x364f;
        return 1;
    }
    case 0x364f: /* ADD.W #0xd435 r2 [0c d4 35 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            0xd435, 0, 1);
        mcu.pc = 0x3653;
        return 1;
    case 0x3653: /* SUB #0x32, @r2 (byte) [d2 04 32] */
    {
        uint32_t t1 = (uint32_t)MCU_Read(reg_addr(2));
        MCU_SUB_Common((int32_t)t1, 0x32, 0, 0);
        mcu.pc = 0x3656;
        return 1;
    }
    case 0x3656: /* BCC +0x39 -> 0x3691 [24 39] */
        mcu.pc = (mcu.sr & STATUS_C) ? 0x3658 : 0x3691;
        return 1;
    case 0x3658: /* BCLR @r1 #0 [d1 d0] */
    {
        uint32_t addr = reg_addr(1);
        uint32_t data = (uint32_t)MCU_Read(addr);
        MCU_SetStatus((data & (1u << 0)) == 0, STATUS_Z);
        data &= ~(1u << 0);
        MCU_Write(addr, (uint8_t)data);
        mcu.pc = 0x365a;
        return 1;
    }
    case 0x365a: /* MOVG2 r0 r2 (byte) [a0 82] */
        mov8(mcu.r[2], (uint32_t)mcu.r[0]);
        mcu.pc = 0x365c;
        return 1;
    case 0x365c: /* EXTU r2 [a2 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[2] & 0xffu);
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x365e;
        return 1;
    }
    case 0x365e: /* ADD r2 r2 [aa 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            (int32_t)(uint32_t)mcu.r[2], 0, 1);
        mcu.pc = 0x3660;
        return 1;
    case 0x3660: /* ADD.W #0x321f r2 [0c 32 1f 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            0x321f, 0, 1);
        mcu.pc = 0x3664;
        return 1;
    case 0x3664: /* LDC #0x04 r5 [04 04 8d] */
        mcu.pc = 0x3667;
        ldc_cr(5, 0x04);
        return 1;
    case 0x3667: /* MOVG2 @r2 r3 (byte) [d2 83] */
        load8(mcu.r[3], reg_addr(2));
        mcu.pc = 0x3669;
        return 1;
    case 0x3669: /* LDC #0x00 r5 [04 00 8d] */
        mcu.pc = 0x366c;
        ldc_cr(5, 0x00);
        return 1;
    case 0x366c: /* CMP.B #0xff r3 [43 ff] */
        cmp8_imm(mcu.r[3], 0xff);
        mcu.pc = 0x366e;
        return 1;
    case 0x366e: /* BEQ +0x16 -> 0x3686 [27 16] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3686 : 0x3670;
        return 1;
    case 0x3670: /* EXTU r3 [a3 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[3] & 0xffu);
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x3672;
        return 1;
    }
    case 0x3672: /* MOVG2 r3 r2 (word) [ab 82] */
        mov16(mcu.r[2], (uint32_t)mcu.r[3]);
        mcu.pc = 0x3674;
        return 1;
    case 0x3674: /* ADD.W #0xd1ac r3 [0c d1 ac 23] */
        mcu.r[3] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[3],
                                            0xd1ac, 0, 1);
        mcu.pc = 0x3678;
        return 1;
    case 0x3678: /* BTSTI @r3 #1 [d3 f1] */
    {
        uint32_t data = (uint32_t)MCU_Read(reg_addr(3));
        MCU_SetStatus((data & (1u << 1)) == 0, STATUS_Z);
        mcu.pc = 0x367a;
        return 1;
    }
    case 0x367a: /* BEQ +0x0a -> 0x3686 [27 0a] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3686 : 0x367c;
        return 1;
    case 0x367c: /* BSET @r3 #3 [d3 c3] */
    {
        uint32_t addr = reg_addr(3);
        uint32_t data = (uint32_t)MCU_Read(addr);
        MCU_SetStatus((data & (1u << 3)) == 0, STATUS_Z);
        data |= 1u << 3;
        MCU_Write(addr, (uint8_t)data);
        mcu.pc = 0x367e;
        return 1;
    }
    case 0x367e: /* ADD.W #0xd435 r2 [0c d4 35 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            0xd435, 0, 1);
        mcu.pc = 0x3682;
        return 1;
    case 0x3682: /* CLR @r2 (byte) [d2 13] */
        MCU_Write(reg_addr(2), (uint8_t)0);
        flags_clr();
        mcu.pc = 0x3684;
        return 1;
    case 0x3684: /* BRA +0x0b -> 0x3691 [20 0b] */
        mcu.pc = 0x3691;
        return 1;
    case 0x3686: /* PUSH.W r0 [bf 90] */
        pushw(mcu.r[0]);
        mcu.pc = 0x3688;
        return 1;
    case 0x3688: /* PUSH.W r1 [bf 91] */
        pushw(mcu.r[1]);
        mcu.pc = 0x368a;
        return 1;
    case 0x368a: /* JSR @0x3532 [18 35 32] */
        call(0x368d, 0x3532);
        return 1;
    case 0x368d: /* POP.W r1 [cf 81] */
        popw(mcu.r[1]);
        mcu.pc = 0x368f;
        return 1;
    case 0x368f: /* POP.W r0 [cf 80] */
        popw(mcu.r[0]);
        mcu.pc = 0x3691;
        return 1;
    case 0x3691: /* cntjmp r0 -77 -> 0x3647 [01 b8 b3] */
        cntjmp(mcu.r[0], 0x3647, 0x3694);
        return 1;
    case 0x3694: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* cp4 0x43641..0x43694 (35 PC) -> step_maint_bit_scan_d1cc */
const uint16_t kMaintBitScanD1ccPcs[] = {
    0x3641, 0x3644, 0x3647, 0x3649, 0x364b, 0x364d, 0x364f, 0x3653,
    0x3656, 0x3658, 0x365a, 0x365c, 0x365e, 0x3660, 0x3664, 0x3667,
    0x3669, 0x366c, 0x366e, 0x3670, 0x3672, 0x3674, 0x3678, 0x367a,
    0x367c, 0x367e, 0x3682, 0x3684, 0x3686, 0x3688, 0x368a, 0x368d,
    0x368f, 0x3691, 0x3694,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_bit_scan_d1cc_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kMaintBitScanD1ccPcs) / sizeof(kMaintBitScanD1ccPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kMaintBitScanD1ccPcs[i], &step_maint_bit_scan_d1cc);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct MaintBitScanD1ccSelfRegister
{
    MaintBitScanD1ccSelfRegister() { hand_register_module(&maint_bit_scan_d1cc_fill); }
};

MaintBitScanD1ccSelfRegister g_maint_bit_scan_d1cc_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
