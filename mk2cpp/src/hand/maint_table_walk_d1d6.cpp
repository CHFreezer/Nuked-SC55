/*
 * HAND maint_table_walk_d1d6.cpp - maintenance table walk over dp:d1d6..dp:d1da (cp4 0x434CB..0x43531).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1); the per-PC case body below is an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). The semantic rewrite is
 * M4 closure step 2 and changes neither addresses nor registration.
 *
 * Semantics: Walks dp:0xd1d6 (four entries), writes dp:0xecf5/dp:0xecf6 and the dp:0xd1cd/dp:0xd1d6 tables; calls 0x43574 (bsr) and 0x43532 (jsr); movm push/pop, movi/bsr/cntjmp. Caller: jsr at 0x433BC (A5). Alternate arms transcribed (C bytes).
 * Evidence: out/m4/28_shared_status.md 1.2 and 4. Confidence: C bytes (role naming I where noted).
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
 * cp4 0x434CB..0x43531, 41 PC: walks dp:0xd1d6 (4 entries, stride 7?)...
 * exact per-instruction transcription; writes dp:0xecf5/0xecf6 and
 * dp:0xd1cd/dp:0xd1d6 tables; calls 0x43574 (bsr) and 0x43532 (jsr).
 * Caller jsr at 0x433BC (A5).
 * ====================================================================== */

uint32_t step_maint_table_walk_d1d6(void)
{
    switch (mcu.pc)
    {
    case 0x34cb: /* MOVI r0 #0x31f4 [58 31 f4] */
        movi16(mcu.r[0], 0x31f4);
        mcu.pc = 0x34ce;
        return 1;
    case 0x34ce: /* MOVI r1 #0xd1da [59 d1 da] */
        movi16(mcu.r[1], 0xd1da);
        mcu.pc = 0x34d1;
        return 1;
    case 0x34d1: /* MOVI r2 #0xd1d6 [5a d1 d6] */
        movi16(mcu.r[2], 0xd1d6);
        mcu.pc = 0x34d4;
        return 1;
    case 0x34d4: /* MOVI r4 #0x0003 [5c 00 03] */
        movi16(mcu.r[4], 0x0003);
        mcu.pc = 0x34d7;
        return 1;
    case 0x34d7: /* LDC #0x04 r5 [04 04 8d] */
        mcu.pc = 0x34da;
        ldc_cr(5, 0x04);
        return 1;
    case 0x34da: /* MOVG2 r0++ r3 (byte) [c0 83] */
    {
        uint32_t oea = reg_addr(0);
        mcu.r[0] += 1;
        uint32_t data = (uint32_t)MCU_Read(oea);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(data, 0);
        mcu.pc = 0x34dc;
        return 1;
    }
    case 0x34dc: /* LDC #0x00 r5 [04 00 8d] */
        mcu.pc = 0x34df;
        ldc_cr(5, 0x00);
        return 1;
    case 0x34df: /* AND (dp,0xd1cd) r3 (byte) [15 d1 cd 53] */
    {
        uint32_t data = (uint32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)MCU_Read(dp_addr(0xd1cd));
        data &= t2;
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(mcu.r[3], 0);
        mcu.pc = 0x34e3;
        return 1;
    }
    case 0x34e3: /* MOVG3 r3 -> (dp,0xecf6) (byte) [15 ec f6 93] */
        store8(dp_addr(0xecf6), mcu.r[3]);
        mcu.pc = 0x34e7;
        return 1;
    case 0x34e7: /* MOVG2 (dp,0xecf5) r5 (byte) [15 ec f5 85] */
        load8(mcu.r[5], dp_addr(0xecf5));
        mcu.pc = 0x34eb;
        return 1;
    case 0x34eb: /* OR #0x8f r3 [04 8f 43] */
    {
        uint32_t data = 0x8f;
        mcu.r[3] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[3], 0);
        mcu.pc = 0x34ee;
        return 1;
    }
    case 0x34ee: /* MOVG3 r3 -> (dp,0xecf6) (byte) [15 ec f6 93] */
        store8(dp_addr(0xecf6), mcu.r[3]);
        mcu.pc = 0x34f2;
        return 1;
    case 0x34f2: /* MOVG2 r2++ r3 (byte) [c2 83] */
    {
        uint32_t oea = reg_addr(2);
        mcu.r[2] += 1;
        uint32_t data = (uint32_t)MCU_Read(oea);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(data, 0);
        mcu.pc = 0x34f4;
        return 1;
    }
    case 0x34f4: /* MOVG2 @r1 r6 (byte) [d1 86] */
        load8(mcu.r[6], reg_addr(1));
        mcu.pc = 0x34f6;
        return 1;
    case 0x34f6: /* MOVG3 r5 -> r1++ (byte) [c1 95] */
    {
        uint32_t oea = reg_addr(1);
        mcu.r[1] += 1;
        MCU_Write(oea, (uint8_t)mcu.r[5]);
        MCU_SetStatusCommon((uint32_t)mcu.r[5], 0);
        mcu.pc = 0x34f8;
        return 1;
    }
    case 0x34f8: /* CMP r5 r6 (byte) [a5 76] */
    {
        uint32_t t1 = (uint32_t)mcu.r[6];
        uint32_t t2 = (uint32_t)(mcu.r[5] & 0xffu);
        MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
        mcu.pc = 0x34fa;
        return 1;
    }
    case 0x34fa: /* BNE +4 -> 0x3500 [26 04] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34fc : 0x3500;
        return 1;
    case 0x34fc: /* XOR r5 r3 (byte) [a5 63] */
    {
        uint32_t data = (uint32_t)(mcu.r[5] & 0xffu);
        mcu.r[3] ^= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[3], 0);
        mcu.pc = 0x34fe;
        return 1;
    }
    case 0x34fe: /* BNE +5 -> 0x3505 [26 05] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3500 : 0x3505;
        return 1;
    case 0x3500: /* cntjmp r4 -44 -> 0x34d7 [01 bc d4] */
        cntjmp(mcu.r[4], 0x34d7, 0x3503);
        return 1;
    case 0x3503: /* BRA +0x2c -> 0x3531 [20 2c] */
        mcu.pc = 0x3531;
        return 1;
    case 0x3505: /* MOVI r1 #0x0003 [59 00 03] */
        movi16(mcu.r[1], 0x0003);
        mcu.pc = 0x3508;
        return 1;
    case 0x3508: /* SUB r4 r1 (byte) [a4 31] */
    {
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xffu);
        int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)mcu.r[1],
                                    (int32_t)t2, 0, 0);
        mcu.r[1] = (uint16_t)((mcu.r[1] & 0xff00u) | ((uint32_t)t1 & 0xffu));
        mcu.pc = 0x350a;
        return 1;
    }
    case 0x350a: /* MULXU #0x08 r1 [04 08 a9] */
    {
        uint32_t t1 = 0x08;
        uint32_t t2 = (uint32_t)(mcu.r[1] & 0xffu);
        t1 *= t2;
        t1 &= 0xffffu;
        mcu.r[1] = (uint16_t)t1;
        MCU_SetStatus((t1 & 0x8000u) != 0, STATUS_N);
        MCU_SetStatus(t1 == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x350d;
        return 1;
    }
    case 0x350d: /* MOVI r4 #0x0007 [5c 00 07] */
        movi16(mcu.r[4], 0x0007);
        mcu.pc = 0x3510;
        return 1;
    case 0x3510: /* BTST r4 r3 (byte) [a3 7c] */
    {
        uint32_t data = (uint32_t)(mcu.r[3] & 0xffu);
        uint32_t bit = (uint32_t)(mcu.r[4] & 0x0fu);
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
        mcu.pc = 0x3512;
        return 1;
    }
    case 0x3512: /* BEQ +0x18 -> 0x352c [27 18] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x352c : 0x3514;
        return 1;
    case 0x3514: /* MOVG2 r1 r0 (byte) [a1 80] */
        mov8(mcu.r[0], (uint32_t)mcu.r[1]);
        mcu.pc = 0x3516;
        return 1;
    case 0x3516: /* ADD r4 r0 (byte) [a4 20] */
    {
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xffu);
        int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)mcu.r[0],
                                    (int32_t)t2, 0, 0);
        mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | ((uint32_t)t1 & 0xffu));
        mcu.pc = 0x3518;
        return 1;
    }
    case 0x3518: /* BTST r4 r5 (byte) [a5 7c] */
    {
        uint32_t data = (uint32_t)(mcu.r[5] & 0xffu);
        uint32_t bit = (uint32_t)(mcu.r[4] & 0x0fu);
        MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
        mcu.pc = 0x351a;
        return 1;
    }
    case 0x351a: /* BEQ +3 -> 0x351f [27 03] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x351f : 0x351c;
        return 1;
    case 0x351c: /* OR #0x80 r0 [04 80 40] */
    {
        uint32_t data = 0x80;
        mcu.r[0] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[0], 0);
        mcu.pc = 0x351f;
        return 1;
    }
    case 0x351f: /* MOVM rlist, @-SP (r0-r6) [12 7f] */
    {
        uint8_t rlist = 0x7f;
        for (int i = 7; i >= 0; i--)
        {
            if (rlist & (1 << i))
            {
                uint16_t data = mcu.r[i];
                if (i == 7)
                    data -= 2;
                MCU_PushStack(data);
            }
        }
        mcu.pc = 0x3521;
        return 1;
    }
    case 0x3521: /* BSR +0x51 -> 0x3574 [0e 51] */
        call(0x3523, 0x3574);
        return 1;
    case 0x3523: /* CMP.B #0xff r0 [40 ff] */
        cmp8_imm(mcu.r[0], 0xff);
        mcu.pc = 0x3525;
        return 1;
    case 0x3525: /* BEQ +3 -> 0x352a [27 03] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x352a : 0x3527;
        return 1;
    case 0x3527: /* JSR @0x3532 [18 35 32] */
        call(0x352a, 0x3532);
        return 1;
    case 0x352a: /* MOVM @SP+, rlist (r0-r6) [02 7f] */
    {
        uint8_t rlist = 0x7f;
        for (int i = 0; i < 8; i++)
        {
            if (rlist & (1 << i))
            {
                uint16_t data = MCU_PopStack();
                if (i != 7)
                    mcu.r[i] = data;
            }
        }
        mcu.pc = 0x352c;
        return 1;
    }
    case 0x352c: /* cntjmp r4 -31 -> 0x3510 [01 bc e1] */
        cntjmp(mcu.r[4], 0x3510, 0x352f);
        return 1;
    case 0x352f: /* MOVG3 r5 -> @-r2 (byte) [b2 95] */
    {
        mcu.r[2] -= 1;
        uint32_t oea = (uint32_t)mcu.r[2] & 0xffffu;
        uint32_t oep = (uint32_t)(page_of_reg(2) & 0xff);
        uint32_t data = (uint32_t)mcu.r[5];
        MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)data);
        MCU_SetStatusCommon(data, 0);
        mcu.pc = 0x3531;
        return 1;
    }
    case 0x3531: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* cp4 0x434CB..0x43531 (41 PC) -> step_maint_table_walk_d1d6 */
const uint16_t kMaintTableWalkD1d6Pcs[] = {
    0x34cb, 0x34ce, 0x34d1, 0x34d4, 0x34d7, 0x34da, 0x34dc, 0x34df,
    0x34e3, 0x34e7, 0x34eb, 0x34ee, 0x34f2, 0x34f4, 0x34f6, 0x34f8,
    0x34fa, 0x34fc, 0x34fe, 0x3500, 0x3503, 0x3505, 0x3508, 0x350a,
    0x350d, 0x3510, 0x3512, 0x3514, 0x3516, 0x3518, 0x351a, 0x351c,
    0x351f, 0x3521, 0x3523, 0x3525, 0x3527, 0x352a, 0x352c, 0x352f,
    0x3531,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_table_walk_d1d6_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kMaintTableWalkD1d6Pcs) / sizeof(kMaintTableWalkD1d6Pcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kMaintTableWalkD1d6Pcs[i], &step_maint_table_walk_d1d6);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct MaintTableWalkD1d6SelfRegister
{
    MaintTableWalkD1d6SelfRegister() { hand_register_module(&maint_table_walk_d1d6_fill); }
};

MaintTableWalkD1d6SelfRegister g_maint_table_walk_d1d6_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
