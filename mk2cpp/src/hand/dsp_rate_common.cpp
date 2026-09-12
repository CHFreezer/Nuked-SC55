/*
 * HAND dsp_rate_common.cpp - dsp_rate shared sub-helper (cp0 0x50EF..0x516B).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13); addresses and registration unchanged. One L0 hand entry per
 * instruction PC so the host keeps its per-instruction interrupt poll,
 * cycles += 12, trace and MIDI/SM cadence (docs/09_m4_integration.md 4.1).
 *
 * Semantics (Confirmed bytes, role naming I): shared sub-helper of the
 * dsp_rate family, called by bsr16 from 0x4E9B/0x4EA8; it may also be an
 * exception-return PC (rte 0x7DC3 / ret 0x7C3F resume edges). It saturates a
 * signed sum of r2+r3 to +/-0x1770, then performs a 16x16 multiply-accumulate
 * on the 32-bit product: double the accumulator r6, multiply by the saturated
 * value into r2:r3, round the high half with +0x8000 and fold the carry
 * through the byte fraction r5, adding the product into the word at r0+70 and
 * the carry into the byte at r0+45 (the second arm subtracts instead when r5
 * goes negative, clearing both). The overlap decode at 0x5106 (JMP #0xAA14)
 * and 0x5107 (NEG r2) are both kept.
 * Evidence: out/m4/28_shared_status.md 1.1; out/m4/24_dsprate_status.md.
 *
 * Shared H8 primitives live in hand_prims.h (mk2c::hand_prim). Registration:
 * all PCs self-register from a file-static initializer (hand_registry.h); no
 * shared aggregator file is edited and duplicate registration is fatal
 * (mk2cpp.cpp).
 */
#include "hand_prims.h"
#include "hand_registry.h"

namespace mk2c {
namespace {
using namespace mk2c::hand_prim;

/* ---- local composites (bodies copied from the case table) ---------------- */

/* ADD rS,rD (word). */
void add16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, 0, 1);
}

/* SUB rS,rD (word). */
void sub16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_SUB_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, 0, 1);
}

/* ADD.W #imm,rD (word). */
void add16_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 1);
}

/* ADDX #0,rD (word): rD += C; Z is kept unless the incoming Z was clear. */
void addx16_zero(uint16_t &reg)
{
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)reg, 0, C, 1);
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
    reg = (uint16_t)t1;
}

/* ADDX #0,rD (byte). */
void addx8_zero(uint16_t &reg)
{
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)reg, 0, C, 0);
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

/* SUBX #0,rD (byte): rD -= C. */
void subx8_zero(uint16_t &reg)
{
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)reg, 0, C, 0);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

/* CLR rD (word). */
void clr16(uint16_t &reg)
{
    reg = 0;
    flags_clr();
}

/* CLR rD (byte): keep the high byte, clear the low byte. */
void clr8_low(uint16_t &reg)
{
    reg = (uint16_t)(reg & 0xff00u);
    flags_clr();
}

/* ======================================================================
 * cp0 0x50EF..0x516B (rts at 0x516B), 54 PC.
 * ====================================================================== */

/* 0x50EF tst r2. */
void step_test_value(void)
{
    tst16(mcu.r[2]);
    mcu.pc = 0x50f1;
}

/* 0x50F1 bmi -> 0x5103 -- r2 negative. */
void step_branch_if_value_neg(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5103 : 0x50f3;
}

/* 0x50F3 tst r3. */
void step_test_offset(void)
{
    tst16(mcu.r[3]);
    mcu.pc = 0x50f5;
}

/* 0x50F5 bmi -> 0x5117 -- r3 negative. */
void step_branch_if_offset_neg(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5117 : 0x50f7;
}

/* 0x50F7 add r3,r2 -- same-sign positive sum. */
void step_add_offset(void)
{
    add16_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x50f9;
}

/* 0x50F9 cmp.w #0x1770,r2. */
void step_compare_limit(void)
{
    cmp16_imm(mcu.r[2], 0x1770);
    mcu.pc = 0x50fc;
}

/* 0x50FC bls -> 0x511f -- within the limit. */
void step_branch_within_limit(void)
{
    uint32_t branch = ((mcu.sr & (STATUS_C | STATUS_Z)) != 0);
    mcu.pc = branch ? 0x511f : 0x50fe;
}

/* 0x50FE movi r2,#0x1770 -- saturate. */
void step_saturate_pos(void)
{
    movi16(mcu.r[2], 0x1770);
    mcu.pc = 0x5101;
}

/* 0x5101 bra -> 0x511f. */
void step_branch_use(void)
{
    mcu.pc = 0x511f;
}

/* 0x5103 tst r3 (r2 negative). */
void step_test_offset_b(void)
{
    tst16(mcu.r[3]);
    mcu.pc = 0x5105;
}

/* 0x5105 bpl -> 0x5117 -- r3 positive: mixed signs. */
void step_branch_if_offset_pos(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5107 : 0x5117;
}

/* 0x5106 jmp #0xAA14 -- overlap decode of the bpl bytes. */
void step_jump_helper(void)
{
    mcu.pc = 0xaa14;
}

/* 0x5107 neg r2 -- both negative: negate r2. */
void step_negate_value(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x5109;
}

/* 0x5109 neg r3. */
void step_negate_offset(void)
{
    neg16(mcu.r[3]);
    mcu.pc = 0x510b;
}

/* 0x510B add r3,r2 -- negative sum. */
void step_add_offsets(void)
{
    add16_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x510d;
}

/* 0x510D cmp.w #0x1770,r2. */
void step_compare_limit_b(void)
{
    cmp16_imm(mcu.r[2], 0x1770);
    mcu.pc = 0x5110;
}

/* 0x5110 bls -> 0x5127 -- within the limit. */
void step_branch_within_limit_b(void)
{
    uint32_t branch = ((mcu.sr & (STATUS_C | STATUS_Z)) != 0);
    mcu.pc = branch ? 0x5127 : 0x5112;
}

/* 0x5112 movi r2,#0x1770 -- saturate. */
void step_saturate_pos_b(void)
{
    movi16(mcu.r[2], 0x1770);
    mcu.pc = 0x5115;
}

/* 0x5115 bra -> 0x5127. */
void step_branch_use_b(void)
{
    mcu.pc = 0x5127;
}

/* 0x5117 add r3,r2 -- mixed signs: combine. */
void step_add_opposite(void)
{
    add16_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x5119;
}

/* 0x5119 bpl -> 0x511f -- non-negative result. */
void step_branch_if_result_pos(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x511b : 0x511f;
}

/* 0x511B neg r2 -- make it positive. */
void step_negate_result(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x511d;
}

/* 0x511D bra -> 0x5127. */
void step_branch_use_c(void)
{
    mcu.pc = 0x5127;
}

/* 0x511F tst r6. */
void step_test_accumulator(void)
{
    tst16(mcu.r[6]);
    mcu.pc = 0x5121;
}

/* 0x5121 bpl -> 0x512d -- non-negative accumulator. */
void step_branch_if_accum_pos(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5123 : 0x512d;
}

/* 0x5123 neg r6. */
void step_negate_accum(void)
{
    neg16(mcu.r[6]);
    mcu.pc = 0x5125;
}

/* 0x5125 bra -> 0x5146 -- accumulator was negative: subtract arm. */
void step_branch_accum_neg(void)
{
    mcu.pc = 0x5146;
}

/* 0x5127 tst r6 (saturated arm). */
void step_test_accumulator_b(void)
{
    tst16(mcu.r[6]);
    mcu.pc = 0x5129;
}

/* 0x5129 bpl -> 0x5146 -- non-negative accumulator: add arm. */
void step_branch_if_accum_pos_b(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x512b : 0x5146;
}

/* 0x512B neg r6. */
void step_negate_accum_b(void)
{
    neg16(mcu.r[6]);
    mcu.pc = 0x512d;
}

/* 0x512D add r6,r6 -- double the accumulator (add arm). */
void step_double_accum(void)
{
    add16_reg(mcu.r[6], mcu.r[6]);
    mcu.pc = 0x512f;
}

/* 0x512F mulxu r6,r2:r3 -- 32-bit product. */
void step_multiply(void)
{
    mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
    mcu.pc = 0x5131;
}

/* 0x5131 add.w #0x8000,r3 -- round the low half. */
void step_round_low(void)
{
    add16_imm(mcu.r[3], 0x8000);
    mcu.pc = 0x5135;
}

/* 0x5135 addx #0,r2 -- fold the carry into the high half. */
void step_carry_high(void)
{
    addx16_zero(mcu.r[2]);
    mcu.pc = 0x5139;
}

/* 0x5139 mov.b @r0+45,r5 -- fraction byte. */
void step_load_frac(void)
{
    load8(mcu.r[5], ind_addr(0, 45));
    mcu.pc = 0x513c;
}

/* 0x513C mov.w @r0+70,r6 -- accumulator word. */
void step_load_accum(void)
{
    load16(mcu.r[6], ind_addr(0, 70));
    mcu.pc = 0x513f;
}

/* 0x513F add r2,r6 -- accumulate the high half. */
void step_add_product(void)
{
    add16_reg(mcu.r[6], mcu.r[2]);
    mcu.pc = 0x5141;
}

/* 0x5141 addx #0,r5 -- fold the carry into the fraction byte. */
void step_carry_frac(void)
{
    addx8_zero(mcu.r[5]);
    mcu.pc = 0x5144;
}

/* 0x5144 bra -> 0x5165 -- store. */
void step_branch_store(void)
{
    mcu.pc = 0x5165;
}

/* 0x5146 add r6,r6 -- double the accumulator (subtract arm). */
void step_double_accum_b(void)
{
    add16_reg(mcu.r[6], mcu.r[6]);
    mcu.pc = 0x5148;
}

/* 0x5148 mulxu r6,r2:r3 -- 32-bit product. */
void step_multiply_b(void)
{
    mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
    mcu.pc = 0x514a;
}

/* 0x514A add.w #0x8000,r3 -- round the low half. */
void step_round_low_b(void)
{
    add16_imm(mcu.r[3], 0x8000);
    mcu.pc = 0x514e;
}

/* 0x514E addx #0,r2 -- fold the carry into the high half. */
void step_carry_high_b(void)
{
    addx16_zero(mcu.r[2]);
    mcu.pc = 0x5152;
}

/* 0x5152 mov.b @r0+45,r5 -- fraction byte. */
void step_load_frac_b(void)
{
    load8(mcu.r[5], ind_addr(0, 45));
    mcu.pc = 0x5155;
}

/* 0x5155 mov.w @r0+70,r6 -- accumulator word. */
void step_load_accum_b(void)
{
    load16(mcu.r[6], ind_addr(0, 70));
    mcu.pc = 0x5158;
}

/* 0x5158 sub r2,r6 -- subtract the high half. */
void step_sub_product(void)
{
    sub16_reg(mcu.r[6], mcu.r[2]);
    mcu.pc = 0x515a;
}

/* 0x515A subx #0,r5 -- borrow from the fraction byte. */
void step_borrow_frac(void)
{
    subx8_zero(mcu.r[5]);
    mcu.pc = 0x515d;
}

/* 0x515D tst r5 (byte). */
void step_test_frac(void)
{
    tst8((uint32_t)mcu.r[5]);
    mcu.pc = 0x515f;
}

/* 0x515F bpl -> 0x5165 -- fraction non-negative: store. */
void step_branch_if_frac_pos(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5161 : 0x5165;
}

/* 0x5161 clr r6 -- negative guard: clear the accumulator. */
void step_clr_accum(void)
{
    clr16(mcu.r[6]);
    mcu.pc = 0x5163;
}

/* 0x5163 clr r5 (byte) -- clear the fraction. */
void step_clr_frac(void)
{
    clr8_low(mcu.r[5]);
    mcu.pc = 0x5165;
}

/* 0x5165 mov.b r5,@r0+45 -- store the fraction byte. */
void step_store_frac(void)
{
    store8(ind_addr(0, 45), mcu.r[5]);
    mcu.pc = 0x5168;
}

/* 0x5168 mov.w r6,@r0+70 -- store the accumulator word. */
void step_store_accum(void)
{
    store16(ind_addr(0, 70), mcu.r[6]);
    mcu.pc = 0x516b;
}

/* 0x516B rts. */
void step_return(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void dsp_rate_common_fill(void)
{
    MK2CPP_HandRegister(0x000050efu, &step_test_value);
    MK2CPP_HandRegister(0x000050f1u, &step_branch_if_value_neg);
    MK2CPP_HandRegister(0x000050f3u, &step_test_offset);
    MK2CPP_HandRegister(0x000050f5u, &step_branch_if_offset_neg);
    MK2CPP_HandRegister(0x000050f7u, &step_add_offset);
    MK2CPP_HandRegister(0x000050f9u, &step_compare_limit);
    MK2CPP_HandRegister(0x000050fcu, &step_branch_within_limit);
    MK2CPP_HandRegister(0x000050feu, &step_saturate_pos);
    MK2CPP_HandRegister(0x00005101u, &step_branch_use);
    MK2CPP_HandRegister(0x00005103u, &step_test_offset_b);
    MK2CPP_HandRegister(0x00005105u, &step_branch_if_offset_pos);
    MK2CPP_HandRegister(0x00005106u, &step_jump_helper);
    MK2CPP_HandRegister(0x00005107u, &step_negate_value);
    MK2CPP_HandRegister(0x00005109u, &step_negate_offset);
    MK2CPP_HandRegister(0x0000510bu, &step_add_offsets);
    MK2CPP_HandRegister(0x0000510du, &step_compare_limit_b);
    MK2CPP_HandRegister(0x00005110u, &step_branch_within_limit_b);
    MK2CPP_HandRegister(0x00005112u, &step_saturate_pos_b);
    MK2CPP_HandRegister(0x00005115u, &step_branch_use_b);
    MK2CPP_HandRegister(0x00005117u, &step_add_opposite);
    MK2CPP_HandRegister(0x00005119u, &step_branch_if_result_pos);
    MK2CPP_HandRegister(0x0000511bu, &step_negate_result);
    MK2CPP_HandRegister(0x0000511du, &step_branch_use_c);
    MK2CPP_HandRegister(0x0000511fu, &step_test_accumulator);
    MK2CPP_HandRegister(0x00005121u, &step_branch_if_accum_pos);
    MK2CPP_HandRegister(0x00005123u, &step_negate_accum);
    MK2CPP_HandRegister(0x00005125u, &step_branch_accum_neg);
    MK2CPP_HandRegister(0x00005127u, &step_test_accumulator_b);
    MK2CPP_HandRegister(0x00005129u, &step_branch_if_accum_pos_b);
    MK2CPP_HandRegister(0x0000512bu, &step_negate_accum_b);
    MK2CPP_HandRegister(0x0000512du, &step_double_accum);
    MK2CPP_HandRegister(0x0000512fu, &step_multiply);
    MK2CPP_HandRegister(0x00005131u, &step_round_low);
    MK2CPP_HandRegister(0x00005135u, &step_carry_high);
    MK2CPP_HandRegister(0x00005139u, &step_load_frac);
    MK2CPP_HandRegister(0x0000513cu, &step_load_accum);
    MK2CPP_HandRegister(0x0000513fu, &step_add_product);
    MK2CPP_HandRegister(0x00005141u, &step_carry_frac);
    MK2CPP_HandRegister(0x00005144u, &step_branch_store);
    MK2CPP_HandRegister(0x00005146u, &step_double_accum_b);
    MK2CPP_HandRegister(0x00005148u, &step_multiply_b);
    MK2CPP_HandRegister(0x0000514au, &step_round_low_b);
    MK2CPP_HandRegister(0x0000514eu, &step_carry_high_b);
    MK2CPP_HandRegister(0x00005152u, &step_load_frac_b);
    MK2CPP_HandRegister(0x00005155u, &step_load_accum_b);
    MK2CPP_HandRegister(0x00005158u, &step_sub_product);
    MK2CPP_HandRegister(0x0000515au, &step_borrow_frac);
    MK2CPP_HandRegister(0x0000515du, &step_test_frac);
    MK2CPP_HandRegister(0x0000515fu, &step_branch_if_frac_pos);
    MK2CPP_HandRegister(0x00005161u, &step_clr_accum);
    MK2CPP_HandRegister(0x00005163u, &step_clr_frac);
    MK2CPP_HandRegister(0x00005165u, &step_store_frac);
    MK2CPP_HandRegister(0x00005168u, &step_store_accum);
    MK2CPP_HandRegister(0x0000516bu, &step_return);
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
