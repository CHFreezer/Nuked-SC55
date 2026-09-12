/*
 * HAND pcm/pcm_fraction_div - 32/32 fixed-point fraction division
 * ("svc_math", class C6, rom1 cp=0, flat 0x2669..0x272d).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13, named operations); addresses and registration unchanged. One
 * L0 hand entry per instruction PC; see pcm_irq_service.cpp for the L0
 * rationale (docs/09_m4_integration.md 4.1).
 *
 * Semantics (Confirmed, C6): reached by fallthrough from C5 irq_service. It
 * forms a 32-bit numerator from the voice fields (r4=[r0+42], r5=[r0+64],
 * r2=[r0+45], r3=[r0+70]), subtracts r5:r4 from r3:r2, then calls the local
 * helper at 0x26a4 with r2:r3. The helper subtracts 0x2ee0 (12000, the
 * sample-rate divisor) and, using DIVXU, produces a quotient r3 and remainder
 * r2; the quotient selects the 0x78ee coefficient and the (swapped) high byte
 * selects the 0x7aee coefficient, whose 16x16 product is rotated into the
 * fraction. A negative quotient shifts the result right r3 times, an overflow
 * clamps r4 to 0xffff, and the outcome is adjusted by [r0+0xa6], clamped to
 * 0xffff/0, and latched: key byte to (br,0x3e), fraction word to (br,0x10),
 * then back to the dispatcher at 0x51f6.
 * Evidence: out/m4/18_closure_gap.md 1.3 C6 / 4.2 row 7; dasm:4360+.
 * Confidence: C (ROM bytes + dasm).
 *
 * Registration: all PCs self-register from a file-static initializer
 * (hand_registry.h); no shared aggregator file is edited and duplicate
 * registration is fatal (mk2cpp.cpp).
 */

#include "hand_prims.h"
#include "hand_registry.h"

namespace mk2c {
namespace {
using namespace mk2c::hand_prim;

/* (br,disp8): the base-register-relative control window (page 0). */
uint32_t br_addr(uint16_t disp)
{
    return (uint32_t)((mcu.br << 8) | (disp & 0xffu));
}

/* EXTS rN (byte): sign-extend the low byte; status comes from the old word. */
void exts8(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    reg = (uint16_t)(int8_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* NOT rN (word). */
void not16(uint16_t &reg)
{
    uint32_t data = ~(uint32_t)reg;
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

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

/* ADD.W #imm,rD. */
void add16_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 1);
}

/* SUB.W #imm,rD. */
void sub16_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_SUB_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 1);
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

/* SUBX #0,rD (byte): rD -= C. */
void subx8_zero(uint16_t &reg)
{
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)reg, 0, C, 0);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

/* SUBX rS,rD (byte): rD -= rS + C. */
void subx8_reg(uint16_t &dst, uint16_t src)
{
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)dst, (int32_t)(src & 0xffu), C, 0);
    dst = (uint16_t)((dst & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

/* AND #imm,rD (byte form used here). */
void and8_imm(uint16_t &reg, uint8_t imm)
{
    uint32_t data = (uint32_t)reg & (uint32_t)imm;
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(reg, 0);
}

/* DIVXU #imm,rH:rL (word divisor): rH = remainder, rL = quotient. */
void divxu16(uint16_t &hi, uint16_t &lo, uint16_t divisor)
{
    if (divisor == 0)
    {
        MCU_ErrorTrap();
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        return;
    }
    uint32_t dividend = ((uint32_t)hi << 16) | (uint32_t)lo;
    uint32_t R = dividend % divisor;
    uint32_t Q = dividend / divisor;
    if (Q > 0xffffu)
    {
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(0, STATUS_Z);
        MCU_SetStatus(1, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    else
    {
        hi = (uint16_t)R;
        lo = (uint16_t)Q;
        MCU_SetStatusCommon(Q, 1);
        MCU_SetStatus(0, STATUS_C);
    }
}

/* SWAP rN (word). */
void swap16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t swapped = ((data & 0xffu) << 8) | (data >> 8);
    reg = (uint16_t)swapped;
    MCU_SetStatusCommon(swapped, 1);
}

/* ROTL rN (word): rotate left through carry. */
void rotl16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t C = (data & 0x8000u) != 0;
    data <<= 1;
    data |= C;
    reg = (uint16_t)data;
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

/* SHLR rN (word): shift right, LSB into C. */
void shlr16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t C = data & 1u;
    data >>= 1;
    reg = (uint16_t)data;
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

/* ======================================================================
 * cp0 0x2669..0x272D, 80 PC: C6 fixed-point fraction division.
 * ====================================================================== */

/* 0x2669 mov.b @r0+42,r4 -- numerator low byte source. */
void step_load_env4(void)
{
    load8(mcu.r[4], ind_addr(0, 42));
    mcu.pc = 0x266c;
}

/* 0x266C mov.w @r0+64,r5 -- numerator word source. */
void step_load_env5(void)
{
    load16(mcu.r[5], ind_addr(0, 64));
    mcu.pc = 0x266f;
}

/* 0x266F mov.b r4,@r0+41 -- publish the low byte. */
void step_store_env4(void)
{
    store8(ind_addr(0, 41), mcu.r[4]);
    mcu.pc = 0x2672;
}

/* 0x2672 mov.w r5,@r0+62 -- publish the word. */
void step_store_env5(void)
{
    store16(ind_addr(0, 62), mcu.r[5]);
    mcu.pc = 0x2675;
}

/* 0x2675 mov.b @r0+45,r2 -- numerator low. */
void step_load_num_lo(void)
{
    load8(mcu.r[2], ind_addr(0, 45));
    mcu.pc = 0x2678;
}

/* 0x2678 mov.w @r0+70,r3 -- numerator high. */
void step_load_num_hi(void)
{
    load16(mcu.r[3], ind_addr(0, 70));
    mcu.pc = 0x267b;
}

/* 0x267B sub r5,r3 -- low half. */
void step_sub_num_lo(void)
{
    sub16_reg(mcu.r[3], mcu.r[5]);
    mcu.pc = 0x267d;
}

/* 0x267D subx r4,r2 -- high half with borrow. */
void step_sub_num_hi(void)
{
    subx8_reg(mcu.r[2], mcu.r[4]);
    mcu.pc = 0x267f;
}

/* 0x267F bsr #0x26a4 -- divide helper; returns to 0x2681. */
void step_call_div_helper(void)
{
    call(0x2681, 0x26a4);
}

/* 0x2681 clr @r0+0xa4 -- clear the result flag byte. */
void step_clear_result_flag(void)
{
    store8(ind_addr(0, 0xa4), 0);
    mcu.pc = 0x2685;
}

/* 0x2685 mov.w @r0+0xa6,r1 -- adjust value. */
void step_load_adjust(void)
{
    load16(mcu.r[1], ind_addr(0, 0xa6));
    mcu.pc = 0x2689;
}

/* 0x2689 bmi -> 0x2694 -- negative adjust. */
void step_branch_if_adjust_neg(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x2694 : 0x268b;
}

/* 0x268B add r1,r4 -- positive adjust. */
void step_add_adjust(void)
{
    add16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x268d;
}

/* 0x268D bcc -> 0x269a -- no overflow: keep the sum. */
void step_branch_after_high(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x268f : 0x269a;
}

/* 0x268F movi r4,#0xffff -- clamp. */
void step_saturate_high(void)
{
    movi16(mcu.r[4], 0xffff);
    mcu.pc = 0x2692;
}

/* 0x2692 bra -> 0x269a. */
void step_branch_high_done(void)
{
    mcu.pc = 0x269a;
}

/* 0x2694 add r1,r4 -- negative adjust. */
void step_add_adjust_neg(void)
{
    add16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x2696;
}

/* 0x2696 bcs -> 0x269a -- borrow: keep the difference. */
void step_branch_after_neg(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x269a : 0x2698;
}

/* 0x2698 clr r4 -- clamp to zero. */
void step_clear_result(void)
{
    mcu.r[4] = 0;
    flags_clr();
    mcu.pc = 0x269a;
}

/* 0x269A mov.w @r0-2,r1 -- caller key. */
void step_load_key(void)
{
    load16(mcu.r[1], ind_addr(0, -2));
    mcu.pc = 0x269d;
}

/* 0x269D movs r1,(br,$3e) -- key byte to the control window. */
void step_store_key(void)
{
    store8(br_addr(0x3e), mcu.r[1]);
    mcu.pc = 0x269f;
}

/* 0x269F movsw r4,(br,$10) -- latched fraction word. */
void step_store_fraction(void)
{
    store16(br_addr(0x10), mcu.r[4]);
    mcu.pc = 0x26a1;
}

/* 0x26A1 bra -> 0x51f6 -- back to the dispatcher. */
void step_branch_return(void)
{
    mcu.pc = 0x51f6;
}

/* 0x26A4 sub #0x2ee0,r3 -- below the sample-rate divisor? */
void step_sub_divisor_lo(void)
{
    sub16_imm(mcu.r[3], 0x2ee0);
    mcu.pc = 0x26a8;
}

/* 0x26A8 subx #0,r2 -- high half with borrow. */
void step_sub_divisor_hi(void)
{
    subx8_zero(mcu.r[2]);
    mcu.pc = 0x26ab;
}

/* 0x26AB bpl -> 0x26fb -- non-negative: forward division. */
void step_branch_if_positive(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x26ad : 0x26fb;
}

/* 0x26AD exts r2 -- negative path: sign-extend the high byte. */
void step_exts_hi(void)
{
    exts8(mcu.r[2]);
    mcu.pc = 0x26af;
}

/* 0x26AF not r2. */
void step_not_hi(void)
{
    not16(mcu.r[2]);
    mcu.pc = 0x26b1;
}

/* 0x26B1 not r3. */
void step_not_lo(void)
{
    not16(mcu.r[3]);
    mcu.pc = 0x26b3;
}

/* 0x26B3 addq #1,r3 -- two's-complement negate r3. */
void step_inc_lo(void)
{
    add16_imm(mcu.r[3], 0x0001);
    mcu.pc = 0x26b5;
}

/* 0x26B5 addx #0,r2 -- carry into the high half. */
void step_carry_hi(void)
{
    addx16_zero(mcu.r[2]);
    mcu.pc = 0x26b9;
}

/* 0x26B9 divxu #0x2ee0,r2:r3 -- divide the absolute value. */
void step_divide(void)
{
    divxu16(mcu.r[2], mcu.r[3], 0x2ee0);
    mcu.pc = 0x26bd;
}

/* 0x26BD cmp r2,w #0 -- remainder zero? */
void step_compare_remainder(void)
{
    cmp16_imm(mcu.r[2], 0x0000);
    mcu.pc = 0x26c0;
}

/* 0x26C0 beq -> 0x26ca -- exact division. */
void step_branch_remainder_zero(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x26ca : 0x26c2;
}

/* 0x26C2 addq #1,r3 -- round the quotient up. */
void step_inc_quotient(void)
{
    add16_imm(mcu.r[3], 0x0001);
    mcu.pc = 0x26c4;
}

/* 0x26C4 neg r2 -- remainder to the other sign. */
void step_neg_remainder(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x26c6;
}

/* 0x26C6 add #0x2ee0,r2 -- restore the divisor scale. */
void step_restore_remainder(void)
{
    add16_imm(mcu.r[2], 0x2ee0);
    mcu.pc = 0x26ca;
}

/* 0x26CA clr r1 -- table index. */
void step_clear_index(void)
{
    mcu.r[1] = 0;
    flags_clr();
    mcu.pc = 0x26cc;
}

/* 0x26CC mov.b r2,r1 -- seed the low index byte. */
void step_seed_index_lo(void)
{
    mov8(mcu.r[1], (uint32_t)mcu.r[2]);
    mcu.pc = 0x26ce;
}

/* 0x26CE add r1,r1 -- index * 2. */
void step_double_index(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x26d0;
}

/* 0x26D0 mov.w @r1+0x78ee,r4 -- first coefficient. */
void step_load_coeff_hi(void)
{
    load16(mcu.r[4], ind_addr(1, 0x78ee));
    mcu.pc = 0x26d4;
}

/* 0x26D4 clr r1. */
void step_clear_index2(void)
{
    mcu.r[1] = 0;
    flags_clr();
    mcu.pc = 0x26d6;
}

/* 0x26D6 swap r2 -- high byte into the low half. */
void step_swap_remainder(void)
{
    swap16(mcu.r[2]);
    mcu.pc = 0x26d8;
}

/* 0x26D8 mov.b r2,r1 -- seed the high index byte. */
void step_seed_index_hi(void)
{
    mov8(mcu.r[1], (uint32_t)mcu.r[2]);
    mcu.pc = 0x26da;
}

/* 0x26DA add r1,r1. */
void step_double_index2(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x26dc;
}

/* 0x26DC mov.w @r1+0x7aee,r1 -- second coefficient. */
void step_load_coeff_lo(void)
{
    load16(mcu.r[1], ind_addr(1, 0x7aee));
    mcu.pc = 0x26e0;
}

/* 0x26E0 mulxu r1,r4:r5 -- 32-bit coefficient product. */
void step_multiply_coeff(void)
{
    mulxu(mcu.r[4], mcu.r[5], mcu.r[1], mcu.r[4]);
    mcu.pc = 0x26e2;
}

/* 0x26E2 rotl r4. */
void step_rotate_a(void)
{
    rotl16(mcu.r[4]);
    mcu.pc = 0x26e4;
}

/* 0x26E4 rotl r4. */
void step_rotate_b(void)
{
    rotl16(mcu.r[4]);
    mcu.pc = 0x26e6;
}

/* 0x26E6 and #0x03,r4 -- keep two fractional bits. */
void step_mask_rotate(void)
{
    and8_imm(mcu.r[4], 0x03);
    mcu.pc = 0x26e9;
}

/* 0x26E9 swap r4. */
void step_swap_result(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x26eb;
}

/* 0x26EB add r1,r4 -- fold the coefficient back. */
void step_add_base(void)
{
    add16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x26ed;
}

/* 0x26ED tst r3 -- remaining shift count. */
void step_test_shift(void)
{
    tst16(mcu.r[3]);
    mcu.pc = 0x26ef;
}

/* 0x26EF beq -> 0x272d -- no shift: return. */
void step_branch_no_shift(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x272d : 0x26f1;
}

/* 0x26F1 sub #0x0001,r3 -- one shift consumed. */
void step_dec_shift(void)
{
    sub16_imm(mcu.r[3], 0x0001);
    mcu.pc = 0x26f5;
}

/* 0x26F5 shlr r4 -- shift the fraction right. */
void step_shift_right(void)
{
    shlr16(mcu.r[4]);
    mcu.pc = 0x26f7;
}

/* 0x26F7 cntjmp r3,-5 -> 0x26f5 -- loop the shift r3 times. */
void step_loop_shift(void)
{
    cntjmp(mcu.r[3], 0x26f5, 0x26fa);
}

/* 0x26FA rts -- negative path done. */
void step_return(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x26FB exts r2 -- positive path: sign-extend the high byte. */
void step_exts_remainder(void)
{
    exts8(mcu.r[2]);
    mcu.pc = 0x26fd;
}

/* 0x26FD divxu #0x2ee0,r2:r3 -- divide. */
void step_divide_pos(void)
{
    divxu16(mcu.r[2], mcu.r[3], 0x2ee0);
    mcu.pc = 0x2701;
}

/* 0x2701 tst r3 -- quotient. */
void step_test_quotient(void)
{
    tst16(mcu.r[3]);
    mcu.pc = 0x2703;
}

/* 0x2703 beq -> 0x270a -- zero quotient: still resolve the fraction. */
void step_branch_quotient_zero(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x270a : 0x2705;
}

/* 0x2705 movg #0xffff,r4 -- overflow clamp. */
void step_set_ones(void)
{
    mcu.pc = 0x2709;
    movi16(mcu.r[4], 0xffff);
}

/* 0x2709 rts -- clamped. */
void step_return_ones(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x270A clr r1 -- table index. */
void step_clear_index_pos(void)
{
    mcu.r[1] = 0;
    flags_clr();
    mcu.pc = 0x270c;
}

/* 0x270C mov.b r2,r1. */
void step_seed_index_pos(void)
{
    mov8(mcu.r[1], (uint32_t)mcu.r[2]);
    mcu.pc = 0x270e;
}

/* 0x270E add r1,r1. */
void step_double_index_pos(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x2710;
}

/* 0x2710 mov.w @r1+0x78ee,r4 -- first coefficient. */
void step_load_coeff_hi_pos(void)
{
    load16(mcu.r[4], ind_addr(1, 0x78ee));
    mcu.pc = 0x2714;
}

/* 0x2714 clr r1. */
void step_clear_index2_pos(void)
{
    mcu.r[1] = 0;
    flags_clr();
    mcu.pc = 0x2716;
}

/* 0x2716 swap r2. */
void step_swap_remainder_pos(void)
{
    swap16(mcu.r[2]);
    mcu.pc = 0x2718;
}

/* 0x2718 mov.b r2,r1. */
void step_seed_index_hi_pos(void)
{
    mov8(mcu.r[1], (uint32_t)mcu.r[2]);
    mcu.pc = 0x271a;
}

/* 0x271A add r1,r1. */
void step_double_index2_pos(void)
{
    add16_reg(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x271c;
}

/* 0x271C mov.w @r1+0x7aee,r1 -- second coefficient. */
void step_load_coeff_lo_pos(void)
{
    load16(mcu.r[1], ind_addr(1, 0x7aee));
    mcu.pc = 0x2720;
}

/* 0x2720 mulxu r1,r4:r5 -- 32-bit coefficient product. */
void step_multiply_coeff_pos(void)
{
    mulxu(mcu.r[4], mcu.r[5], mcu.r[1], mcu.r[4]);
    mcu.pc = 0x2722;
}

/* 0x2722 rotl r4. */
void step_rotate_a_pos(void)
{
    rotl16(mcu.r[4]);
    mcu.pc = 0x2724;
}

/* 0x2724 rotl r4. */
void step_rotate_b_pos(void)
{
    rotl16(mcu.r[4]);
    mcu.pc = 0x2726;
}

/* 0x2726 and #0x03,r4. */
void step_mask_rotate_pos(void)
{
    and8_imm(mcu.r[4], 0x03);
    mcu.pc = 0x2729;
}

/* 0x2729 swap r4. */
void step_swap_result_pos(void)
{
    swap16(mcu.r[4]);
    mcu.pc = 0x272b;
}

/* 0x272B add r1,r4 -- fold the coefficient back. */
void step_add_base_pos(void)
{
    add16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x272d;
}

/* 0x272D rts -- positive path done. */
void step_return_done(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void pcm_fraction_div_fill(void)
{
    MK2CPP_HandRegister(0x00002669u, &step_load_env4);
    MK2CPP_HandRegister(0x0000266cu, &step_load_env5);
    MK2CPP_HandRegister(0x0000266fu, &step_store_env4);
    MK2CPP_HandRegister(0x00002672u, &step_store_env5);
    MK2CPP_HandRegister(0x00002675u, &step_load_num_lo);
    MK2CPP_HandRegister(0x00002678u, &step_load_num_hi);
    MK2CPP_HandRegister(0x0000267bu, &step_sub_num_lo);
    MK2CPP_HandRegister(0x0000267du, &step_sub_num_hi);
    MK2CPP_HandRegister(0x0000267fu, &step_call_div_helper);
    MK2CPP_HandRegister(0x00002681u, &step_clear_result_flag);
    MK2CPP_HandRegister(0x00002685u, &step_load_adjust);
    MK2CPP_HandRegister(0x00002689u, &step_branch_if_adjust_neg);
    MK2CPP_HandRegister(0x0000268bu, &step_add_adjust);
    MK2CPP_HandRegister(0x0000268du, &step_branch_after_high);
    MK2CPP_HandRegister(0x0000268fu, &step_saturate_high);
    MK2CPP_HandRegister(0x00002692u, &step_branch_high_done);
    MK2CPP_HandRegister(0x00002694u, &step_add_adjust_neg);
    MK2CPP_HandRegister(0x00002696u, &step_branch_after_neg);
    MK2CPP_HandRegister(0x00002698u, &step_clear_result);
    MK2CPP_HandRegister(0x0000269au, &step_load_key);
    MK2CPP_HandRegister(0x0000269du, &step_store_key);
    MK2CPP_HandRegister(0x0000269fu, &step_store_fraction);
    MK2CPP_HandRegister(0x000026a1u, &step_branch_return);
    MK2CPP_HandRegister(0x000026a4u, &step_sub_divisor_lo);
    MK2CPP_HandRegister(0x000026a8u, &step_sub_divisor_hi);
    MK2CPP_HandRegister(0x000026abu, &step_branch_if_positive);
    MK2CPP_HandRegister(0x000026adu, &step_exts_hi);
    MK2CPP_HandRegister(0x000026afu, &step_not_hi);
    MK2CPP_HandRegister(0x000026b1u, &step_not_lo);
    MK2CPP_HandRegister(0x000026b3u, &step_inc_lo);
    MK2CPP_HandRegister(0x000026b5u, &step_carry_hi);
    MK2CPP_HandRegister(0x000026b9u, &step_divide);
    MK2CPP_HandRegister(0x000026bdu, &step_compare_remainder);
    MK2CPP_HandRegister(0x000026c0u, &step_branch_remainder_zero);
    MK2CPP_HandRegister(0x000026c2u, &step_inc_quotient);
    MK2CPP_HandRegister(0x000026c4u, &step_neg_remainder);
    MK2CPP_HandRegister(0x000026c6u, &step_restore_remainder);
    MK2CPP_HandRegister(0x000026cau, &step_clear_index);
    MK2CPP_HandRegister(0x000026ccu, &step_seed_index_lo);
    MK2CPP_HandRegister(0x000026ceu, &step_double_index);
    MK2CPP_HandRegister(0x000026d0u, &step_load_coeff_hi);
    MK2CPP_HandRegister(0x000026d4u, &step_clear_index2);
    MK2CPP_HandRegister(0x000026d6u, &step_swap_remainder);
    MK2CPP_HandRegister(0x000026d8u, &step_seed_index_hi);
    MK2CPP_HandRegister(0x000026dau, &step_double_index2);
    MK2CPP_HandRegister(0x000026dcu, &step_load_coeff_lo);
    MK2CPP_HandRegister(0x000026e0u, &step_multiply_coeff);
    MK2CPP_HandRegister(0x000026e2u, &step_rotate_a);
    MK2CPP_HandRegister(0x000026e4u, &step_rotate_b);
    MK2CPP_HandRegister(0x000026e6u, &step_mask_rotate);
    MK2CPP_HandRegister(0x000026e9u, &step_swap_result);
    MK2CPP_HandRegister(0x000026ebu, &step_add_base);
    MK2CPP_HandRegister(0x000026edu, &step_test_shift);
    MK2CPP_HandRegister(0x000026efu, &step_branch_no_shift);
    MK2CPP_HandRegister(0x000026f1u, &step_dec_shift);
    MK2CPP_HandRegister(0x000026f5u, &step_shift_right);
    MK2CPP_HandRegister(0x000026f7u, &step_loop_shift);
    MK2CPP_HandRegister(0x000026fau, &step_return);
    MK2CPP_HandRegister(0x000026fbu, &step_exts_remainder);
    MK2CPP_HandRegister(0x000026fdu, &step_divide_pos);
    MK2CPP_HandRegister(0x00002701u, &step_test_quotient);
    MK2CPP_HandRegister(0x00002703u, &step_branch_quotient_zero);
    MK2CPP_HandRegister(0x00002705u, &step_set_ones);
    MK2CPP_HandRegister(0x00002709u, &step_return_ones);
    MK2CPP_HandRegister(0x0000270au, &step_clear_index_pos);
    MK2CPP_HandRegister(0x0000270cu, &step_seed_index_pos);
    MK2CPP_HandRegister(0x0000270eu, &step_double_index_pos);
    MK2CPP_HandRegister(0x00002710u, &step_load_coeff_hi_pos);
    MK2CPP_HandRegister(0x00002714u, &step_clear_index2_pos);
    MK2CPP_HandRegister(0x00002716u, &step_swap_remainder_pos);
    MK2CPP_HandRegister(0x00002718u, &step_seed_index_hi_pos);
    MK2CPP_HandRegister(0x0000271au, &step_double_index2_pos);
    MK2CPP_HandRegister(0x0000271cu, &step_load_coeff_lo_pos);
    MK2CPP_HandRegister(0x00002720u, &step_multiply_coeff_pos);
    MK2CPP_HandRegister(0x00002722u, &step_rotate_a_pos);
    MK2CPP_HandRegister(0x00002724u, &step_rotate_b_pos);
    MK2CPP_HandRegister(0x00002726u, &step_mask_rotate_pos);
    MK2CPP_HandRegister(0x00002729u, &step_swap_result_pos);
    MK2CPP_HandRegister(0x0000272bu, &step_add_base_pos);
    MK2CPP_HandRegister(0x0000272du, &step_return_done);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct PcmFractionDivSelfRegister
{
    PcmFractionDivSelfRegister() { hand_register_module(&pcm_fraction_div_fill); }
};

PcmFractionDivSelfRegister g_pcm_fraction_div_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
