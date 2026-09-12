/*
 * HAND maint_voice_gate_d1ff.cpp - maintenance voice-on/off gate on dp:d1ff/dp:d421 (cp4 0x4342A..0x434CA).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1); the per-PC case body below is an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). The semantic rewrite is
 * M4 closure step 2 and changes neither addresses nor registration.
 *
 * Semantics: Reads dp:0xd421/dp:0xd364, writes dp:0xd1ff, scans dp:0xd1ac.. and toggles bit 3 at @r0. Caller: jsr at 0x433BF (A5); internal arms 0x343B/0x347B; single return through the shared rts at 0x434CA. Calls 0x43532/0x43574 (other owners). All alternate arms are transcribed per ROM bytes (C).
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
 * cp4 0x4342A..0x434CA, 65 PC: voice-on/off gate keyed on dp:0xd1ff,
 * dp:0xd421 and dp:0xd364; writes dp:0xd1ff, scans dp:0xd1ac.. and
 * toggles bit 3 at @r0.  Caller jsr at 0x433BF (A5); internal arms
 * 0x343B/0x347B; single return through the shared rts at 0x434CA.
 * Calls 0x43532/0x43574 (other owners, left to gen/hand as-is).
 * ====================================================================== */

uint32_t step_maint_voice_gate_d1ff(void)
{
    switch (mcu.pc)
    {
    case 0x342a: /* MOVG2 (dp,0xd1ff) r0 (byte) [15 d1 ff 80] */
        load8(mcu.r[0], dp_addr(0xd1ff));
        mcu.pc = 0x342e;
        return 1;
    case 0x342e: /* CMP.B #0xff r0 [40 ff] */
        cmp8_imm(mcu.r[0], 0xff);
        mcu.pc = 0x3430;
        return 1;
    case 0x3430: /* BNE +0x49 -> 0x347b [26 49] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3432 : 0x347b;
        return 1;
    case 0x3432: /* TST (dp,0xd421) (byte) [15 d4 21 16] */
        tst8((uint32_t)MCU_Read(dp_addr(0xd421)));
        mcu.pc = 0x3436;
        return 1;
    case 0x3436: /* BNE +3 -> 0x343b [26 03] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3438 : 0x343b;
        return 1;
    case 0x3438: /* JMP #0x34ca [10 34 ca] */
        mcu.pc = 0x34ca;
        return 1;
    case 0x343b: /* MOVG2 (dp,0xd207) r0 (byte) [15 d2 07 80] */
        load8(mcu.r[0], dp_addr(0xd207));
        mcu.pc = 0x343f;
        return 1;
    case 0x343f: /* CMP.B #0x14 r0 [40 14] */
        cmp8_imm(mcu.r[0], 0x14);
        mcu.pc = 0x3441;
        return 1;
    case 0x3441: /* BCS +0x1f -> 0x3462 [25 1f] */
        mcu.pc = (mcu.sr & STATUS_C) ? 0x3462 : 0x3443;
        return 1;
    case 0x3443: /* MOVI r1 #0x40 [51 40] */
        movi8(mcu.r[1], 0x40);
        mcu.pc = 0x3445;
        return 1;
    case 0x3445: /* CMP.B #0x04, (dp,0xd364) [15 d3 64 04 04] */
    {
        uint32_t t1 = (uint32_t)MCU_Read(dp_addr(0xd364));
        MCU_SUB_Common((int32_t)t1, 0x04, 0, 0);
        mcu.pc = 0x344a;
        return 1;
    }
    case 0x344a: /* BNE +0x24 -> 0x3470 [26 24] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x344c : 0x3470;
        return 1;
    case 0x344c: /* MOVI r1 #0x20 [51 20] */
        movi8(mcu.r[1], 0x20);
        mcu.pc = 0x344e;
        return 1;
    case 0x344e: /* CMP.B #0x45 r0 [40 45] */
        cmp8_imm(mcu.r[0], 0x45);
        mcu.pc = 0x3450;
        return 1;
    case 0x3450: /* BEQ +0x1e -> 0x3470 [27 1e] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x3452;
        return 1;
    case 0x3452: /* MOVI r1 #0x21 [51 21] */
        movi8(mcu.r[1], 0x21);
        mcu.pc = 0x3454;
        return 1;
    case 0x3454: /* CMP.B #0x2a r0 [40 2a] */
        cmp8_imm(mcu.r[0], 0x2a);
        mcu.pc = 0x3456;
        return 1;
    case 0x3456: /* BEQ +0x18 -> 0x3470 [27 18] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x3458;
        return 1;
    case 0x3458: /* MOVI r1 #0x22 [51 22] */
        movi8(mcu.r[1], 0x22);
        mcu.pc = 0x345a;
        return 1;
    case 0x345a: /* CMP.B #0x50 r0 [40 50] */
        cmp8_imm(mcu.r[0], 0x50);
        mcu.pc = 0x345c;
        return 1;
    case 0x345c: /* BEQ +0x12 -> 0x3470 [27 12] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x345e;
        return 1;
    case 0x345e: /* MOVI r1 #0x40 [51 40] */
        movi8(mcu.r[1], 0x40);
        mcu.pc = 0x3460;
        return 1;
    case 0x3460: /* BRA +0x0e -> 0x3470 [20 0e] */
        mcu.pc = 0x3470;
        return 1;
    case 0x3462: /* EXTU r0 [a0 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[0] & 0xffu);
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x3464;
        return 1;
    }
    case 0x3464: /* ADD.W #0x320b r0 [0c 32 0b 20] */
        mcu.r[0] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[0],
                                            0x320b, 0, 1);
        mcu.pc = 0x3468;
        return 1;
    case 0x3468: /* LDC #0x04 r5 [04 04 8d] */
        mcu.pc = 0x346b;
        ldc_cr(5, 0x04);
        return 1;
    case 0x346b: /* MOVG2 @r0 r1 (byte) [d0 81] */
        load8(mcu.r[1], reg_addr(0));
        mcu.pc = 0x346d;
        return 1;
    case 0x346d: /* LDC #0x00 r5 [04 00 8d] */
        mcu.pc = 0x3470;
        ldc_cr(5, 0x00);
        return 1;
    case 0x3470: /* OR #0x40 r1 [04 40 41] */
    {
        uint32_t data = 0x40;
        mcu.r[1] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[1], 0);
        mcu.pc = 0x3473;
        return 1;
    }
    case 0x3473: /* MOVG2 r1 r0 (byte) [a1 80] */
        mov8(mcu.r[0], (uint32_t)mcu.r[1]);
        mcu.pc = 0x3475;
        return 1;
    case 0x3475: /* MOVG3 r0 -> (dp,0xd1ff) (byte) [15 d1 ff 90] */
        store8(dp_addr(0xd1ff), mcu.r[0]);
        mcu.pc = 0x3479;
        return 1;
    case 0x3479: /* BRA +0x0f -> 0x348a [20 0f] */
        mcu.pc = 0x348a;
        return 1;
    case 0x347b: /* TST (dp,0xd421) (byte) [15 d4 21 16] */
        tst8((uint32_t)MCU_Read(dp_addr(0xd421)));
        mcu.pc = 0x347f;
        return 1;
    case 0x347f: /* BNE +0x49 -> 0x34ca [26 49] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x3481 : 0x34ca;
        return 1;
    case 0x3481: /* MOVG #0x00ff -> (dp,0xd1ff) (byte) [15 d1 ff 07 00 ff] */
        mcu.pc = 0x3487;
        store8_imm_dp(0xd1ff, 0x00ff);
        return 1;
    case 0x3487: /* OR #0x80 r0 [04 80 40] */
    {
        uint32_t data = 0x80;
        mcu.r[0] |= (uint16_t)data;
        MCU_SetStatusCommon(mcu.r[0], 0);
        mcu.pc = 0x348a;
        return 1;
    }
    case 0x348a: /* MOVG2 r0 r1 (byte) [a0 81] */
        mov8(mcu.r[1], (uint32_t)mcu.r[0]);
        mcu.pc = 0x348c;
        return 1;
    case 0x348c: /* CMP.B #0x04, (dp,0xd364) [15 d3 64 04 04] */
    {
        uint32_t t1 = (uint32_t)MCU_Read(dp_addr(0xd364));
        MCU_SUB_Common((int32_t)t1, 0x04, 0, 0);
        mcu.pc = 0x3491;
        return 1;
    }
    case 0x3491: /* BEQ +0x0b -> 0x349e [27 0b] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x349e : 0x3493;
        return 1;
    case 0x3493: /* PUSH.W r1 [bf 91] */
        pushw(mcu.r[1]);
        mcu.pc = 0x3495;
        return 1;
    case 0x3495: /* JSR @0x3574 [18 35 74] */
        call(0x3498, 0x3574);
        return 1;
    case 0x3498: /* POP.W r1 [cf 81] */
        popw(mcu.r[1]);
        mcu.pc = 0x349a;
        return 1;
    case 0x349a: /* CMP.B #0xff r0 [40 ff] */
        cmp8_imm(mcu.r[0], 0xff);
        mcu.pc = 0x349c;
        return 1;
    case 0x349c: /* BEQ +7 -> 0x34a5 [27 07] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34a5 : 0x349e;
        return 1;
    case 0x349e: /* PUSH.W r1 [bf 91] */
        pushw(mcu.r[1]);
        mcu.pc = 0x34a0;
        return 1;
    case 0x34a0: /* JSR @0x3532 [18 35 32] */
        call(0x34a3, 0x3532);
        return 1;
    case 0x34a3: /* POP.W r1 [cf 81] */
        popw(mcu.r[1]);
        mcu.pc = 0x34a5;
        return 1;
    case 0x34a5: /* MOVG2 r1 r0 (byte) [a1 80] */
        mov8(mcu.r[0], (uint32_t)mcu.r[1]);
        mcu.pc = 0x34a7;
        return 1;
    case 0x34a7: /* AND #0x3f r0 [04 3f 50] */
    {
        uint32_t data = (uint32_t)mcu.r[0];
        data &= 0x3f;
        mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | (data & 0xffu));
        MCU_SetStatusCommon(mcu.r[0], 0);
        mcu.pc = 0x34aa;
        return 1;
    }
    case 0x34aa: /* CMP.B #0x14 r0 [40 14] */
        cmp8_imm(mcu.r[0], 0x14);
        mcu.pc = 0x34ac;
        return 1;
    case 0x34ac: /* BEQ +0x0c -> 0x34ba [27 0c] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34ae;
        return 1;
    case 0x34ae: /* CMP.B #0x15 r0 [40 15] */
        cmp8_imm(mcu.r[0], 0x15);
        mcu.pc = 0x34b0;
        return 1;
    case 0x34b0: /* BEQ +8 -> 0x34ba [27 08] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34b2;
        return 1;
    case 0x34b2: /* CMP.B #0x12 r0 [40 12] */
        cmp8_imm(mcu.r[0], 0x12);
        mcu.pc = 0x34b4;
        return 1;
    case 0x34b4: /* BEQ +4 -> 0x34ba [27 04] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34b6;
        return 1;
    case 0x34b6: /* CMP.B #0x13 r0 [40 13] */
        cmp8_imm(mcu.r[0], 0x13);
        mcu.pc = 0x34b8;
        return 1;
    case 0x34b8: /* BNE +0x10 -> 0x34ca [26 10] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34ca;
        return 1;
    case 0x34ba: /* EXTU r0 [a0 12] */
    {
        uint32_t data = (uint32_t)(mcu.r[0] & 0xffu);
        mcu.r[0] = (uint16_t)data;
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(data == 0, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x34bc;
        return 1;
    }
    case 0x34bc: /* ADD.W #0xd1ac r0 [0c d1 ac 20] */
        mcu.r[0] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[0],
                                            0xd1ac, 0, 1);
        mcu.pc = 0x34c0;
        return 1;
    case 0x34c0: /* BTSTI r1 #7 [a1 f7] */
    {
        uint32_t data = (uint32_t)(mcu.r[1] & 0xffu);
        MCU_SetStatus((data & (1u << 7)) == 0, STATUS_Z);
        mcu.pc = 0x34c2;
        return 1;
    }
    case 0x34c2: /* BNE +4 -> 0x34c8 [26 04] */
        mcu.pc = (mcu.sr & STATUS_Z) ? 0x34c4 : 0x34c8;
        return 1;
    case 0x34c4: /* BSET @r0 #3 [d0 c3] */
    {
        uint32_t addr = reg_addr(0);
        uint32_t data = (uint32_t)MCU_Read(addr);
        MCU_SetStatus((data & (1u << 3)) == 0, STATUS_Z);
        data |= 1u << 3;
        MCU_Write(addr, (uint8_t)data);
        mcu.pc = 0x34c6;
        return 1;
    }
    case 0x34c6: /* BRA +2 -> 0x34ca [20 02] */
        mcu.pc = 0x34ca;
        return 1;
    case 0x34c8: /* BCLR @r0 #3 [d0 d3] */
    {
        uint32_t addr = reg_addr(0);
        uint32_t data = (uint32_t)MCU_Read(addr);
        MCU_SetStatus((data & (1u << 3)) == 0, STATUS_Z);
        data &= ~(1u << 3);
        MCU_Write(addr, (uint8_t)data);
        mcu.pc = 0x34ca;
        return 1;
    }
    case 0x34ca: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* cp4 0x4342A..0x434CA (65 PC) -> step_maint_voice_gate_d1ff */
const uint16_t kMaintVoiceGateD1ffPcs[] = {
    0x342a, 0x342e, 0x3430, 0x3432, 0x3436, 0x3438, 0x343b, 0x343f,
    0x3441, 0x3443, 0x3445, 0x344a, 0x344c, 0x344e, 0x3450, 0x3452,
    0x3454, 0x3456, 0x3458, 0x345a, 0x345c, 0x345e, 0x3460, 0x3462,
    0x3464, 0x3468, 0x346b, 0x346d, 0x3470, 0x3473, 0x3475, 0x3479,
    0x347b, 0x347f, 0x3481, 0x3487, 0x348a, 0x348c, 0x3491, 0x3493,
    0x3495, 0x3498, 0x349a, 0x349c, 0x349e, 0x34a0, 0x34a3, 0x34a5,
    0x34a7, 0x34aa, 0x34ac, 0x34ae, 0x34b0, 0x34b2, 0x34b4, 0x34b6,
    0x34b8, 0x34ba, 0x34bc, 0x34c0, 0x34c2, 0x34c4, 0x34c6, 0x34c8,
    0x34ca,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_voice_gate_d1ff_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kMaintVoiceGateD1ffPcs) / sizeof(kMaintVoiceGateD1ffPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kMaintVoiceGateD1ffPcs[i], &step_maint_voice_gate_d1ff);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct MaintVoiceGateD1ffSelfRegister
{
    MaintVoiceGateD1ffSelfRegister() { hand_register_module(&maint_voice_gate_d1ff_fill); }
};

MaintVoiceGateD1ffSelfRegister g_maint_voice_gate_d1ff_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
