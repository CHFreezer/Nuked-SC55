/*
 * HAND dsp_rate_common.cpp - dsp_rate shared sub-helper (cp0 0x50EF..0x516B).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1); the per-PC case body below is an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC (each entry executes exactly one H8 instruction and returns
 * 1) so the host keeps its per-instruction interrupt poll, cycles += 12, trace
 * and MIDI/SM cadence (docs/09_m4_integration.md 4.1). The semantic rewrite is
 * M4 closure step 2 and changes neither addresses nor registration.
 *
 * Semantics: Shared sub-helper of the dsp_rate family: called by bsr16 from 0x4E9B/0x4EA8; scales the value in r2 by the 0x78EE/0x7AEE table at r1, folds carries through r5 and accumulates into r6. May also be an exception-return PC (rte 0x7DC3 / ret 0x7C3F resume edges). Role naming is inferred (I); the body is a per-instruction transcription (C bytes).
 * Evidence: out/m4/28_shared_status.md 1.1; out/m4/24_dsprate_status.md. Confidence: C bytes (role naming I where noted).
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
 * cp0 shared helper 0x50EF..0x516B (rts at 0x516B), 54 PC.
 * Called by bsr16 from dsp_rate 0x4E9B/0x4EA8; may also be an exception
 * return PC (rte 0x7DC3 / ret 0x7C3F resume edges in flow_main).
 * Takes r0 = voice base, scales the value in r2 by the 0x78EE/0x7AEE table
 * at r1, accumulates into r6 with 16-bit carry folding in r5 (the exact
 * arithmetic is transcribed per instruction below).
 * ====================================================================== */

uint32_t step_dsp_rate_common(void)
{
    switch (mcu.pc)
    {
    case 0x50ef: /* TST r2 [aa 16] */
        tst16(mcu.r[2]);
        mcu.pc = 0x50f1;
        return 1;
    case 0x50f1: /* BMI +0x10 -> 0x5103 [2b 10] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5103 : 0x50f3;
        return 1;
    case 0x50f3: /* TST r3 [ab 16] */
        tst16(mcu.r[3]);
        mcu.pc = 0x50f5;
        return 1;
    case 0x50f5: /* BMI +0x20 -> 0x5117 [2b 20] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5117 : 0x50f7;
        return 1;
    case 0x50f7: /* ADD r3 r2 [ab 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            (int32_t)(uint32_t)mcu.r[3], 0, 1);
        mcu.pc = 0x50f9;
        return 1;
    case 0x50f9: /* CMP.W #0x1770 r2 [4a 17 70] */
        cmp16_imm(mcu.r[2], 0x1770);
        mcu.pc = 0x50fc;
        return 1;
    case 0x50fc: /* BLS +0x21 -> 0x511f [23 21] */
    {
        uint32_t branch = ((mcu.sr & (STATUS_C | STATUS_Z)) != 0);
        mcu.pc = branch ? 0x511f : 0x50fe;
        return 1;
    }
    case 0x50fe: /* MOVI r2 #0x1770 [5a 17 70] */
        movi16(mcu.r[2], 0x1770);
        mcu.pc = 0x5101;
        return 1;
    case 0x5101: /* BRA +0x1c -> 0x511f [20 1c] */
        mcu.pc = 0x511f;
        return 1;
    case 0x5103: /* TST r3 [ab 16] */
        tst16(mcu.r[3]);
        mcu.pc = 0x5105;
        return 1;
    case 0x5105: /* BPL +0x10 -> 0x5117 [2a 10] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5107 : 0x5117;
        return 1;
    case 0x5106: /* JMP #0xAA14 [10 aa 14] (overlap decode) */
        mcu.pc = 0xaa14;
        return 1;
    case 0x5107: /* NEG r2 [aa 14] (overlap decode) */
        neg16(mcu.r[2]);
        mcu.pc = 0x5109;
        return 1;
    case 0x5109: /* NEG r3 [ab 14] */
        neg16(mcu.r[3]);
        mcu.pc = 0x510b;
        return 1;
    case 0x510b: /* ADD r3 r2 [ab 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            (int32_t)(uint32_t)mcu.r[3], 0, 1);
        mcu.pc = 0x510d;
        return 1;
    case 0x510d: /* CMP.W #0x1770 r2 [4a 17 70] */
        cmp16_imm(mcu.r[2], 0x1770);
        mcu.pc = 0x5110;
        return 1;
    case 0x5110: /* BLS +0x15 -> 0x5127 [23 15] */
    {
        uint32_t branch = ((mcu.sr & (STATUS_C | STATUS_Z)) != 0);
        mcu.pc = branch ? 0x5127 : 0x5112;
        return 1;
    }
    case 0x5112: /* MOVI r2 #0x1770 [5a 17 70] */
        movi16(mcu.r[2], 0x1770);
        mcu.pc = 0x5115;
        return 1;
    case 0x5115: /* BRA +0x10 -> 0x5127 [20 10] */
        mcu.pc = 0x5127;
        return 1;
    case 0x5117: /* ADD r3 r2 [ab 22] */
        mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2],
                                            (int32_t)(uint32_t)mcu.r[3], 0, 1);
        mcu.pc = 0x5119;
        return 1;
    case 0x5119: /* BPL +4 -> 0x511f [2a 04] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x511b : 0x511f;
        return 1;
    case 0x511b: /* NEG r2 [aa 14] */
        neg16(mcu.r[2]);
        mcu.pc = 0x511d;
        return 1;
    case 0x511d: /* BRA +8 -> 0x5127 [20 08] */
        mcu.pc = 0x5127;
        return 1;
    case 0x511f: /* TST r6 [ae 16] */
        tst16(mcu.r[6]);
        mcu.pc = 0x5121;
        return 1;
    case 0x5121: /* BPL +0x0a -> 0x512d [2a 0a] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5123 : 0x512d;
        return 1;
    case 0x5123: /* NEG r6 [ae 14] */
        neg16(mcu.r[6]);
        mcu.pc = 0x5125;
        return 1;
    case 0x5125: /* BRA +0x1f -> 0x5146 [20 1f] */
        mcu.pc = 0x5146;
        return 1;
    case 0x5127: /* TST r6 [ae 16] */
        tst16(mcu.r[6]);
        mcu.pc = 0x5129;
        return 1;
    case 0x5129: /* BPL +0x1b -> 0x5146 [2a 1b] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x512b : 0x5146;
        return 1;
    case 0x512b: /* NEG r6 [ae 14] */
        neg16(mcu.r[6]);
        mcu.pc = 0x512d;
        return 1;
    case 0x512d: /* ADD r6 r6 [ae 26] */
        mcu.r[6] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[6],
                                            (int32_t)(uint32_t)mcu.r[6], 0, 1);
        mcu.pc = 0x512f;
        return 1;
    case 0x512f: /* MULXU r6 r2:r3 [ae aa] */
        mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
        mcu.pc = 0x5131;
        return 1;
    case 0x5131: /* ADD.W #0x8000 r3 [0c 80 00 23] */
        mcu.r[3] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[3],
                                            0x8000, 0, 1);
        mcu.pc = 0x5135;
        return 1;
    case 0x5135: /* ADDX #0x0000 r2 [0c 00 00 a2] */
    {
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2], 0, C, 1);
        if (!Z)
            MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
        mcu.pc = 0x5139;
        return 1;
    }
    case 0x5139: /* MOVG2 @r0+45 r5 (byte) [e0 2d 85] */
        load8(mcu.r[5], ind_addr(0, 45));
        mcu.pc = 0x513c;
        return 1;
    case 0x513c: /* MOVG2 @r0+70 r6 (word) [e8 46 86] */
        load16(mcu.r[6], ind_addr(0, 70));
        mcu.pc = 0x513f;
        return 1;
    case 0x513f: /* ADD r2 r6 [aa 26] */
        mcu.r[6] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[6],
                                            (int32_t)(uint32_t)mcu.r[2], 0, 1);
        mcu.pc = 0x5141;
        return 1;
    case 0x5141: /* ADDX #0x00 r5 [04 00 a5] */
    {
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)mcu.r[5], 0, C, 0);
        if (!Z)
            MCU_SetStatus(0, STATUS_Z);
        mcu.r[5] = (uint16_t)((mcu.r[5] & 0xff00u) | ((uint32_t)t1 & 0xffu));
        mcu.pc = 0x5144;
        return 1;
    }
    case 0x5144: /* BRA +0x1f -> 0x5165 [20 1f] */
        mcu.pc = 0x5165;
        return 1;
    case 0x5146: /* ADD r6 r6 [ae 26] */
        mcu.r[6] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[6],
                                            (int32_t)(uint32_t)mcu.r[6], 0, 1);
        mcu.pc = 0x5148;
        return 1;
    case 0x5148: /* MULXU r6 r2:r3 [ae aa] */
        mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
        mcu.pc = 0x514a;
        return 1;
    case 0x514a: /* ADD.W #0x8000 r3 [0c 80 00 23] */
        mcu.r[3] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[3],
                                            0x8000, 0, 1);
        mcu.pc = 0x514e;
        return 1;
    case 0x514e: /* ADDX #0x0000 r2 [0c 00 00 a2] */
    {
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t Z = (mcu.sr & STATUS_Z) != 0;
        int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2], 0, C, 1);
        if (!Z)
            MCU_SetStatus(0, STATUS_Z);
        mcu.r[2] = (uint16_t)t1;
        mcu.pc = 0x5152;
        return 1;
    }
    case 0x5152: /* MOVG2 @r0+45 r5 (byte) [e0 2d 85] */
        load8(mcu.r[5], ind_addr(0, 45));
        mcu.pc = 0x5155;
        return 1;
    case 0x5155: /* MOVG2 @r0+70 r6 (word) [e8 46 86] */
        load16(mcu.r[6], ind_addr(0, 70));
        mcu.pc = 0x5158;
        return 1;
    case 0x5158: /* SUB r2 r6 [aa 36] */
        mcu.r[6] = (uint16_t)MCU_SUB_Common((int32_t)(uint32_t)mcu.r[6],
                                            (int32_t)(uint32_t)mcu.r[2], 0, 1);
        mcu.pc = 0x515a;
        return 1;
    case 0x515a: /* SUBX #0x00 r5 [04 00 b5] */
    {
        int32_t C = (mcu.sr & STATUS_C) != 0;
        int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)mcu.r[5], 0, C, 0);
        mcu.r[5] = (uint16_t)((mcu.r[5] & 0xff00u) | ((uint32_t)t1 & 0xffu));
        mcu.pc = 0x515d;
        return 1;
    }
    case 0x515d: /* TST r5 (byte) [a5 16] */
        tst8((uint32_t)mcu.r[5]);
        mcu.pc = 0x515f;
        return 1;
    case 0x515f: /* BPL +4 -> 0x5165 [2a 04] */
        mcu.pc = (mcu.sr & STATUS_N) ? 0x5161 : 0x5165;
        return 1;
    case 0x5161: /* CLR r6 [ae 13] */
        mcu.r[6] = 0;
        flags_clr();
        mcu.pc = 0x5163;
        return 1;
    case 0x5163: /* CLR r5 (byte) [a5 13] */
        mcu.r[5] = (uint16_t)(mcu.r[5] & 0xff00u);
        flags_clr();
        mcu.pc = 0x5165;
        return 1;
    case 0x5165: /* MOVG3 r5 -> @r0+45 (byte) [e0 2d 95] */
        store8(ind_addr(0, 45), mcu.r[5]);
        mcu.pc = 0x5168;
        return 1;
    case 0x5168: /* MOVG3 r6 -> @r0+70 (word) [e8 46 96] */
        store16(ind_addr(0, 70), mcu.r[6]);
        mcu.pc = 0x516b;
        return 1;
    case 0x516b: /* rts [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* cp0 0x50EF..0x516B (54 PC) -> step_dsp_rate_common */
const uint16_t kDspRateCommonPcs[] = {
    0x50ef, 0x50f1, 0x50f3, 0x50f5, 0x50f7, 0x50f9, 0x50fc, 0x50fe,
    0x5101, 0x5103, 0x5105, 0x5106, 0x5107, 0x5109, 0x510b, 0x510d,
    0x5110, 0x5112, 0x5115, 0x5117, 0x5119, 0x511b, 0x511d, 0x511f,
    0x5121, 0x5123, 0x5125, 0x5127, 0x5129, 0x512b, 0x512d, 0x512f,
    0x5131, 0x5135, 0x5139, 0x513c, 0x513f, 0x5141, 0x5144, 0x5146,
    0x5148, 0x514a, 0x514e, 0x5152, 0x5155, 0x5158, 0x515a, 0x515d,
    0x515f, 0x5161, 0x5163, 0x5165, 0x5168, 0x516b,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void dsp_rate_common_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kDspRateCommonPcs) / sizeof(kDspRateCommonPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kDspRateCommonPcs[i], &step_dsp_rate_common);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct DspRateCommonSelfRegister
{
    DspRateCommonSelfRegister() { hand_register_module(&dsp_rate_common_fill); }
};

DspRateCommonSelfRegister g_dsp_rate_common_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
