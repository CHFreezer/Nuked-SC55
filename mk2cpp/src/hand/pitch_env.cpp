/*
 * HAND pcm/pitch_env — pitch/envelope-related per-voice processing
 * (class C8 "r2d95", rom1 cp=0, flat 0x2d95..0x2e82).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13, named operations); addresses and registration unchanged. One
 * L0 hand entry per instruction PC; see pcm_irq_service.cpp for the L0
 * rationale (docs/09_m4_integration.md 4.1).
 *
 * Semantics: pitch/envelope-related per-voice processing; the exact field
 * semantics are NOT fully proven (docs/07:142; 18_closure_gap row 9 marks
 * C bytes / I semantics). The filename is a provisional label, not a proven
 * identity; the body is a byte-exact transcription, never a re-derivation.
 * Evidence: out/m4/18_closure_gap.md 1.3 C8 / 4.2 row 9; frag 2d95.
 * Confidence: C bytes / I semantics.
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

/* ---- local helpers (bodies copied from the previous generated form) ------- */

/* CLR rN (word): rN=0, N=0 Z=1 V=0 C=0. */
void clr_reg(uint16_t &reg)
{
    reg = (uint16_t)0;
    flags_clr();
}

/* SWAP rN (word). */
void swap16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t data_h = data >> 8;
    uint32_t data_l = data & 0xff;
    data = (data_l << 8) | data_h;
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MULXU @addr,rD (byte): rD = byte * (rD & 0xff), 16-bit product. */
void mulxu8_mem(uint16_t &reg, uint32_t addr)
{
    uint32_t t1 = (uint32_t)MCU_Read(addr);
    uint32_t t2 = (uint32_t)reg;
    t2 &= 0xff;
    t1 *= t2;
    t1 &= 0xffff;
    reg = (uint16_t)t1;
    uint32_t N = (t1 & 0x8000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* ADD rS,rD (word). */
void add16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, 0, 1);
}

/* ADD.W #imm,rD. */
void add16_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 1);
}

/* ADDX rD,rD (word): rD += rD + C; Z is kept unless the incoming Z was clear. */
void addx16_self(uint16_t &reg)
{
    int32_t t1 = (int32_t)(uint32_t)reg;
    uint32_t t2 = (uint32_t)reg;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
    reg = (uint16_t)t1;
}

/* ADDX rS,rD (word): rD += rS + C; Z is kept unless the incoming Z was clear. */
void addx16_reg(uint16_t &dst, uint16_t src)
{
    int32_t t1 = (int32_t)(uint32_t)dst;
    uint32_t t2 = (uint32_t)src;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
    if (!Z)
        MCU_SetStatus(0, STATUS_Z);
    dst = (uint16_t)t1;
}

/* SUB rS,rD (word). */
void sub16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_SUB_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, 0, 1);
}

/* SUBX rS,rD (word): rD -= rS + C. */
void subx16_reg(uint16_t &dst, uint16_t src)
{
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, C, 1);
    dst = (uint16_t)t1;
}

/* ---- conditional branches (disp8 relative to the next PC) ----------------- */

void bpl(uint16_t taken, uint16_t fall)
{
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    mcu.pc = (N == 0) ? taken : fall;
}

void bmi(uint16_t taken, uint16_t fall)
{
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    mcu.pc = (N == 1) ? taken : fall;
}

void bne(uint16_t taken, uint16_t fall)
{
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    mcu.pc = (Z == 0) ? taken : fall;
}

void beq(uint16_t taken, uint16_t fall)
{
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    mcu.pc = (Z == 1) ? taken : fall;
}

void bls(uint16_t taken, uint16_t fall)
{
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    mcu.pc = ((C | Z) == 1) ? taken : fall;
}

void bcc(uint16_t taken, uint16_t fall)
{
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    mcu.pc = (C == 0) ? taken : fall;
}

/* ======================================================================
 * cp0 0x2D95..0x2E82, 102 PC: C8 pitch/envelope per-voice processing.
 * ====================================================================== */

/* 0x2D95 clr r2. */
void step_clr_r2(void)
{
    clr_reg(mcu.r[2]);
    mcu.pc = 0x2d97;
}

/* 0x2D97 mov.b @r1+0xce78,r2. */
void step_load_voice_lo_a(void)
{
    load8(mcu.r[2], ind_addr(1, 0xce78));
    mcu.pc = 0x2d9b;
}

/* 0x2D9B mov.b @r2+0xac0e,r2. */
void step_load_voice_lo_b(void)
{
    load8(mcu.r[2], ind_addr(2, 0xac0e));
    mcu.pc = 0x2d9f;
}

/* 0x2D9F mov.w @r0+46,r3. */
void step_load_ptr_a(void)
{
    load16(mcu.r[3], ind_addr(0, 0x2e));
    mcu.pc = 0x2da2;
}

/* 0x2DA2 mulxu @r3+8,r2 (byte). */
void step_mulxu_byte_a(void)
{
    mulxu8_mem(mcu.r[2], ind_addr(3, 0x08));
    mcu.pc = 0x2da5;
}

/* 0x2DA5 clr r6. */
void step_clr_r6(void)
{
    clr_reg(mcu.r[6]);
    mcu.pc = 0x2da7;
}

/* 0x2DA7 mov.b (dp,0x8002),r6. */
void step_load_ctl_byte(void)
{
    load8(mcu.r[6], dp_addr(0x8002));
    mcu.pc = 0x2dab;
}

/* 0x2DAB mulxu r6,r2:r3. */
void step_mulxu_reg_a(void)
{
    mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
    mcu.pc = 0x2dad;
}

/* 0x2DAD add r3,r3. */
void step_double_r3_a(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2daf;
}

/* 0x2DAF addx r2,r2. */
void step_addx_r2_a(void)
{
    addx16_self(mcu.r[2]);
    mcu.pc = 0x2db1;
}

/* 0x2DB1 add r3,r3. */
void step_double_r3_b(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2db3;
}

/* 0x2DB3 addx r2,r2. */
void step_addx_r2_b(void)
{
    addx16_self(mcu.r[2]);
    mcu.pc = 0x2db5;
}

/* 0x2DB5 mov.b r2,r3. */
void step_mov_r2_r3_a(void)
{
    mov8(mcu.r[3], (uint32_t)mcu.r[2]);
    mcu.pc = 0x2db7;
}

/* 0x2DB7 swap r3. */
void step_swap_r3_a(void)
{
    swap16(mcu.r[3]);
    mcu.pc = 0x2db9;
}

/* 0x2DB9 mov.w r3,r2. */
void step_mov_r3_r2_a(void)
{
    mov16(mcu.r[2], (uint32_t)mcu.r[3]);
    mcu.pc = 0x2dbb;
}

/* 0x2DBB mov.w @r0+48,r3. */
void step_load_ptr_b(void)
{
    load16(mcu.r[3], ind_addr(0, 0x30));
    mcu.pc = 0x2dbe;
}

/* 0x2DBE beq 22 -> 0x2dd6. */
void step_beq_zero_a(void)
{
    beq(0x2dd6, 0x2dc0);
}

/* 0x2DC0 mov.b @r3+0x0100,r6. */
void step_load_frac_byte_a(void)
{
    load8(mcu.r[6], ind_addr(3, 0x0100));
    mcu.pc = 0x2dc4;
}

/* 0x2DC4 mulxu r6,r2:r3. */
void step_mulxu_reg_b(void)
{
    mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
    mcu.pc = 0x2dc6;
}

/* 0x2DC6 add r3,r3. */
void step_double_r3_c(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2dc8;
}

/* 0x2DC8 addx r2,r2. */
void step_addx_r2_c(void)
{
    addx16_self(mcu.r[2]);
    mcu.pc = 0x2dca;
}

/* 0x2DCA mov.b r2,r3. */
void step_mov_r2_r3_b(void)
{
    mov8(mcu.r[3], (uint32_t)mcu.r[2]);
    mcu.pc = 0x2dcc;
}

/* 0x2DCC swap r3. */
void step_swap_r3_b(void)
{
    swap16(mcu.r[3]);
    mcu.pc = 0x2dce;
}

/* 0x2DCE mov.w r3,r2. */
void step_mov_r3_r2_b(void)
{
    mov16(mcu.r[2], (uint32_t)mcu.r[3]);
    mcu.pc = 0x2dd0;
}

/* 0x2DD0 mulxu #0x830e,r2:r3. */
void step_mulxu_imm_a(void)
{
    mulxu(mcu.r[2], mcu.r[3], 0x830e, mcu.r[2]);
    mcu.pc = 0x2dd4;
}

/* 0x2DD4 bra 4 -> 0x2dda. */
void step_bra_a(void)
{
    mcu.pc = 0x2dda;
}

/* 0x2DD6 mulxu #0x8208,r2:r3. */
void step_mulxu_imm_b(void)
{
    mulxu(mcu.r[2], mcu.r[3], 0x8208, mcu.r[2]);
    mcu.pc = 0x2dda;
}

/* 0x2DDA add r3,r3. */
void step_double_r3_d(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2ddc;
}

/* 0x2DDC addx r2,r2. */
void step_addx_r2_d(void)
{
    addx16_self(mcu.r[2]);
    mcu.pc = 0x2dde;
}

/* 0x2DDE mov.w r2,r4. */
void step_mov_r2_r4(void)
{
    mov16(mcu.r[4], (uint32_t)mcu.r[2]);
    mcu.pc = 0x2de0;
}

/* 0x2DE0 bne 3 -> 0x2de5. */
void step_bne_nonzero(void)
{
    bne(0x2de5, 0x2de2);
}

/* 0x2DE2 clr r5. */
void step_clr_r5(void)
{
    clr_reg(mcu.r[5]);
    mcu.pc = 0x2de4;
}

/* 0x2DE4 rts. */
void step_return_a(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x2DE5 mov.w @r0+0x008a,r2. */
void step_load_env_word_a(void)
{
    load16(mcu.r[2], ind_addr(0, 0x008a));
    mcu.pc = 0x2de9;
}

/* 0x2DE9 beq 14 -> 0x2df9. */
void step_beq_zero_b(void)
{
    beq(0x2df9, 0x2deb);
}

/* 0x2DEB bpl 10 -> 0x2df7. */
void step_bpl_a(void)
{
    bpl(0x2df7, 0x2ded);
}

/* 0x2DED neg r2. */
void step_neg_r2_a(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x2def;
}

/* 0x2DEF sub r2,r4. */
void step_sub_r2_r4(void)
{
    sub16_reg(mcu.r[4], mcu.r[2]);
    mcu.pc = 0x2df1;
}

/* 0x2DF1 bcc 6 -> 0x2df9. */
void step_bcc_a(void)
{
    bcc(0x2df9, 0x2df3);
}

/* 0x2DF3 clr r4. */
void step_clr_r4_a(void)
{
    clr_reg(mcu.r[4]);
    mcu.pc = 0x2df5;
}

/* 0x2DF5 bra 2 -> 0x2df9. */
void step_bra_b(void)
{
    mcu.pc = 0x2df9;
}

/* 0x2DF7 add r2,r4. */
void step_add_r2_r4(void)
{
    add16_reg(mcu.r[4], mcu.r[2]);
    mcu.pc = 0x2df9;
}

/* 0x2DF9 mov.w @r0-122,r2. */
void step_load_env_word_b(void)
{
    load16(mcu.r[2], ind_addr(0, -122));
    mcu.pc = 0x2dfc;
}

/* 0x2DFC mov.w @r0+0x008e,r3. */
void step_load_env_word_c(void)
{
    load16(mcu.r[3], ind_addr(0, 0x008e));
    mcu.pc = 0x2e00;
}

/* 0x2E00 mov.w @r0-96,r6. */
void step_load_env_word_d(void)
{
    load16(mcu.r[6], ind_addr(0, -96));
    mcu.pc = 0x2e03;
}

/* 0x2E03 bsr 32 -> 0x2e25. */
void step_call_helper_a(void)
{
    call(0x2e05, 0x2e25);
}

/* 0x2E05 mov.w @r0-88,r2. */
void step_load_env_word_e(void)
{
    load16(mcu.r[2], ind_addr(0, -88));
    mcu.pc = 0x2e08;
}

/* 0x2E08 mov.w @r0+0x0096,r3. */
void step_load_env_word_f(void)
{
    load16(mcu.r[3], ind_addr(0, 0x0096));
    mcu.pc = 0x2e0c;
}

/* 0x2E0C mov.w @r0-62,r6. */
void step_load_env_word_g(void)
{
    load16(mcu.r[6], ind_addr(0, -62));
    mcu.pc = 0x2e0f;
}

/* 0x2E0F bsr 20 -> 0x2e25. */
void step_call_helper_b(void)
{
    call(0x2e11, 0x2e25);
}

/* 0x2E11 mulxu r4,r4:r5. */
void step_mulxu_r4(void)
{
    mulxu(mcu.r[4], mcu.r[5], mcu.r[4], mcu.r[4]);
    mcu.pc = 0x2e13;
}

/* 0x2E13 mulxu #0x0208,r4:r5. */
void step_mulxu_imm_r4(void)
{
    mulxu(mcu.r[4], mcu.r[5], 0x0208, mcu.r[4]);
    mcu.pc = 0x2e17;
}

/* 0x2E17 cmp r4,w #0x00ff. */
void step_cmp_r4(void)
{
    cmp16_imm(mcu.r[4], 0x00ff);
    mcu.pc = 0x2e1a;
}

/* 0x2E1A bcc 5 -> 0x2e21. */
void step_bcc_b(void)
{
    bcc(0x2e21, 0x2e1c);
}

/* 0x2E1C mov.b r4,r5. */
void step_mov_r4_r5(void)
{
    mov8(mcu.r[5], (uint32_t)mcu.r[4]);
    mcu.pc = 0x2e1e;
}

/* 0x2E1E swap r5. */
void step_swap_r5(void)
{
    swap16(mcu.r[5]);
    mcu.pc = 0x2e20;
}

/* 0x2E20 rts. */
void step_return_b(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x2E21 movi r5,#0xffff. */
void step_set_ones_r5(void)
{
    mcu.pc = 0x2e24;
    movi16(mcu.r[5], 0xffff);
}

/* 0x2E24 rts. */
void step_return_c(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x2E25 tst r2. */
void step_tst_r2(void)
{
    tst16(mcu.r[2]);
    mcu.pc = 0x2e27;
}

/* 0x2E27 bmi 16 -> 0x2e39. */
void step_bmi_a(void)
{
    bmi(0x2e39, 0x2e29);
}

/* 0x2E29 tst r3. */
void step_tst_r3_a(void)
{
    tst16(mcu.r[3]);
    mcu.pc = 0x2e2b;
}

/* 0x2E2B bmi 32 -> 0x2e4d. */
void step_bmi_b(void)
{
    bmi(0x2e4d, 0x2e2d);
}

/* 0x2E2D add r3,r2. */
void step_add_r3_r2_a(void)
{
    add16_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2e2f;
}

/* 0x2E2F cmp r2,w #0x7f00. */
void step_cmp_r2_a(void)
{
    cmp16_imm(mcu.r[2], 0x7f00);
    mcu.pc = 0x2e32;
}

/* 0x2E32 bls 33 -> 0x2e55. */
void step_bls_a(void)
{
    bls(0x2e55, 0x2e34);
}

/* 0x2E34 movi r2,#0x7f00. */
void step_set_limit_r2_a(void)
{
    movi16(mcu.r[2], 0x7f00);
    mcu.pc = 0x2e37;
}

/* 0x2E37 bra 28 -> 0x2e55. */
void step_bra_c(void)
{
    mcu.pc = 0x2e55;
}

/* 0x2E39 tst r3. */
void step_tst_r3_b(void)
{
    tst16(mcu.r[3]);
    mcu.pc = 0x2e3b;
}

/* 0x2E3B bpl 16 -> 0x2e4d. */
void step_bpl_b(void)
{
    bpl(0x2e4d, 0x2e3d);
}

/* 0x2E3D neg r3. */
void step_neg_r3(void)
{
    neg16(mcu.r[3]);
    mcu.pc = 0x2e3f;
}

/* 0x2E3F neg r2. */
void step_neg_r2_b(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x2e41;
}

/* 0x2E41 add r3,r2. */
void step_add_r3_r2_b(void)
{
    add16_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2e43;
}

/* 0x2E43 cmp r2,w #0x7f00. */
void step_cmp_r2_b(void)
{
    cmp16_imm(mcu.r[2], 0x7f00);
    mcu.pc = 0x2e46;
}

/* 0x2E46 bls 21 -> 0x2e5d. */
void step_bls_b(void)
{
    bls(0x2e5d, 0x2e48);
}

/* 0x2E48 movi r2,#0x7f00. */
void step_set_limit_r2_b(void)
{
    movi16(mcu.r[2], 0x7f00);
    mcu.pc = 0x2e4b;
}

/* 0x2E4B bra 16 -> 0x2e5d. */
void step_bra_d(void)
{
    mcu.pc = 0x2e5d;
}

/* 0x2E4D add r3,r2. */
void step_add_r3_r2_c(void)
{
    add16_reg(mcu.r[2], mcu.r[3]);
    mcu.pc = 0x2e4f;
}

/* 0x2E4F bpl 4 -> 0x2e55. */
void step_bpl_c(void)
{
    bpl(0x2e55, 0x2e51);
}

/* 0x2E51 neg r2. */
void step_neg_r2_c(void)
{
    neg16(mcu.r[2]);
    mcu.pc = 0x2e53;
}

/* 0x2E53 bra 8 -> 0x2e5d. */
void step_bra_e(void)
{
    mcu.pc = 0x2e5d;
}

/* 0x2E55 tst r6. */
void step_tst_r6_a(void)
{
    tst16(mcu.r[6]);
    mcu.pc = 0x2e57;
}

/* 0x2E57 bpl 10 -> 0x2e63. */
void step_bpl_d(void)
{
    bpl(0x2e63, 0x2e59);
}

/* 0x2E59 neg r6. */
void step_neg_r6_a(void)
{
    neg16(mcu.r[6]);
    mcu.pc = 0x2e5b;
}

/* 0x2E5B bra 20 -> 0x2e71. */
void step_bra_f(void)
{
    mcu.pc = 0x2e71;
}

/* 0x2E5D tst r6. */
void step_tst_r6_b(void)
{
    tst16(mcu.r[6]);
    mcu.pc = 0x2e5f;
}

/* 0x2E5F bpl 16 -> 0x2e71. */
void step_bpl_e(void)
{
    bpl(0x2e71, 0x2e61);
}

/* 0x2E61 neg r6. */
void step_neg_r6_b(void)
{
    neg16(mcu.r[6]);
    mcu.pc = 0x2e63;
}

/* 0x2E63 mulxu r6,r2:r3. */
void step_mulxu_reg_c(void)
{
    mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
    mcu.pc = 0x2e65;
}

/* 0x2E65 add r3,r3. */
void step_double_r3_e(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2e67;
}

/* 0x2E67 addx r2,r2. */
void step_addx_r2_e(void)
{
    addx16_self(mcu.r[2]);
    mcu.pc = 0x2e69;
}

/* 0x2E69 add #0xffff,r3. */
void step_add_imm_a(void)
{
    add16_imm(mcu.r[3], 0xffff);
    mcu.pc = 0x2e6d;
}

/* 0x2E6D addx r2,r4. */
void step_addx_r2_r4(void)
{
    addx16_reg(mcu.r[4], mcu.r[2]);
    mcu.pc = 0x2e6f;
}

/* 0x2E6F bra 16 -> 0x2e81. */
void step_bra_g(void)
{
    mcu.pc = 0x2e81;
}

/* 0x2E71 mulxu r6,r2:r3. */
void step_mulxu_reg_d(void)
{
    mulxu(mcu.r[2], mcu.r[3], mcu.r[6], mcu.r[2]);
    mcu.pc = 0x2e73;
}

/* 0x2E73 add r3,r3. */
void step_double_r3_f(void)
{
    add16_reg(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x2e75;
}

/* 0x2E75 addx r2,r2. */
void step_addx_r2_f(void)
{
    addx16_self(mcu.r[2]);
    mcu.pc = 0x2e77;
}

/* 0x2E77 add #0xffff,r3. */
void step_add_imm_b(void)
{
    add16_imm(mcu.r[3], 0xffff);
    mcu.pc = 0x2e7b;
}

/* 0x2E7B subx r2,r4. */
void step_subx_r2_r4(void)
{
    subx16_reg(mcu.r[4], mcu.r[2]);
    mcu.pc = 0x2e7d;
}

/* 0x2E7D bcc 2 -> 0x2e81. */
void step_bcc_c(void)
{
    bcc(0x2e81, 0x2e7f);
}

/* 0x2E7F clr r4. */
void step_clr_r4_b(void)
{
    clr_reg(mcu.r[4]);
    mcu.pc = 0x2e81;
}

/* 0x2E81 rts. */
void step_return_d(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void pitch_env_fill(void)
{
    MK2CPP_HandRegister(0x00002d95u, &step_clr_r2);
    MK2CPP_HandRegister(0x00002d97u, &step_load_voice_lo_a);
    MK2CPP_HandRegister(0x00002d9bu, &step_load_voice_lo_b);
    MK2CPP_HandRegister(0x00002d9fu, &step_load_ptr_a);
    MK2CPP_HandRegister(0x00002da2u, &step_mulxu_byte_a);
    MK2CPP_HandRegister(0x00002da5u, &step_clr_r6);
    MK2CPP_HandRegister(0x00002da7u, &step_load_ctl_byte);
    MK2CPP_HandRegister(0x00002dabu, &step_mulxu_reg_a);
    MK2CPP_HandRegister(0x00002dadu, &step_double_r3_a);
    MK2CPP_HandRegister(0x00002dafu, &step_addx_r2_a);
    MK2CPP_HandRegister(0x00002db1u, &step_double_r3_b);
    MK2CPP_HandRegister(0x00002db3u, &step_addx_r2_b);
    MK2CPP_HandRegister(0x00002db5u, &step_mov_r2_r3_a);
    MK2CPP_HandRegister(0x00002db7u, &step_swap_r3_a);
    MK2CPP_HandRegister(0x00002db9u, &step_mov_r3_r2_a);
    MK2CPP_HandRegister(0x00002dbbu, &step_load_ptr_b);
    MK2CPP_HandRegister(0x00002dbeu, &step_beq_zero_a);
    MK2CPP_HandRegister(0x00002dc0u, &step_load_frac_byte_a);
    MK2CPP_HandRegister(0x00002dc4u, &step_mulxu_reg_b);
    MK2CPP_HandRegister(0x00002dc6u, &step_double_r3_c);
    MK2CPP_HandRegister(0x00002dc8u, &step_addx_r2_c);
    MK2CPP_HandRegister(0x00002dcau, &step_mov_r2_r3_b);
    MK2CPP_HandRegister(0x00002dccu, &step_swap_r3_b);
    MK2CPP_HandRegister(0x00002dceu, &step_mov_r3_r2_b);
    MK2CPP_HandRegister(0x00002dd0u, &step_mulxu_imm_a);
    MK2CPP_HandRegister(0x00002dd4u, &step_bra_a);
    MK2CPP_HandRegister(0x00002dd6u, &step_mulxu_imm_b);
    MK2CPP_HandRegister(0x00002ddau, &step_double_r3_d);
    MK2CPP_HandRegister(0x00002ddcu, &step_addx_r2_d);
    MK2CPP_HandRegister(0x00002ddeu, &step_mov_r2_r4);
    MK2CPP_HandRegister(0x00002de0u, &step_bne_nonzero);
    MK2CPP_HandRegister(0x00002de2u, &step_clr_r5);
    MK2CPP_HandRegister(0x00002de4u, &step_return_a);
    MK2CPP_HandRegister(0x00002de5u, &step_load_env_word_a);
    MK2CPP_HandRegister(0x00002de9u, &step_beq_zero_b);
    MK2CPP_HandRegister(0x00002debu, &step_bpl_a);
    MK2CPP_HandRegister(0x00002dedu, &step_neg_r2_a);
    MK2CPP_HandRegister(0x00002defu, &step_sub_r2_r4);
    MK2CPP_HandRegister(0x00002df1u, &step_bcc_a);
    MK2CPP_HandRegister(0x00002df3u, &step_clr_r4_a);
    MK2CPP_HandRegister(0x00002df5u, &step_bra_b);
    MK2CPP_HandRegister(0x00002df7u, &step_add_r2_r4);
    MK2CPP_HandRegister(0x00002df9u, &step_load_env_word_b);
    MK2CPP_HandRegister(0x00002dfcu, &step_load_env_word_c);
    MK2CPP_HandRegister(0x00002e00u, &step_load_env_word_d);
    MK2CPP_HandRegister(0x00002e03u, &step_call_helper_a);
    MK2CPP_HandRegister(0x00002e05u, &step_load_env_word_e);
    MK2CPP_HandRegister(0x00002e08u, &step_load_env_word_f);
    MK2CPP_HandRegister(0x00002e0cu, &step_load_env_word_g);
    MK2CPP_HandRegister(0x00002e0fu, &step_call_helper_b);
    MK2CPP_HandRegister(0x00002e11u, &step_mulxu_r4);
    MK2CPP_HandRegister(0x00002e13u, &step_mulxu_imm_r4);
    MK2CPP_HandRegister(0x00002e17u, &step_cmp_r4);
    MK2CPP_HandRegister(0x00002e1au, &step_bcc_b);
    MK2CPP_HandRegister(0x00002e1cu, &step_mov_r4_r5);
    MK2CPP_HandRegister(0x00002e1eu, &step_swap_r5);
    MK2CPP_HandRegister(0x00002e20u, &step_return_b);
    MK2CPP_HandRegister(0x00002e21u, &step_set_ones_r5);
    MK2CPP_HandRegister(0x00002e24u, &step_return_c);
    MK2CPP_HandRegister(0x00002e25u, &step_tst_r2);
    MK2CPP_HandRegister(0x00002e27u, &step_bmi_a);
    MK2CPP_HandRegister(0x00002e29u, &step_tst_r3_a);
    MK2CPP_HandRegister(0x00002e2bu, &step_bmi_b);
    MK2CPP_HandRegister(0x00002e2du, &step_add_r3_r2_a);
    MK2CPP_HandRegister(0x00002e2fu, &step_cmp_r2_a);
    MK2CPP_HandRegister(0x00002e32u, &step_bls_a);
    MK2CPP_HandRegister(0x00002e34u, &step_set_limit_r2_a);
    MK2CPP_HandRegister(0x00002e37u, &step_bra_c);
    MK2CPP_HandRegister(0x00002e39u, &step_tst_r3_b);
    MK2CPP_HandRegister(0x00002e3bu, &step_bpl_b);
    MK2CPP_HandRegister(0x00002e3du, &step_neg_r3);
    MK2CPP_HandRegister(0x00002e3fu, &step_neg_r2_b);
    MK2CPP_HandRegister(0x00002e41u, &step_add_r3_r2_b);
    MK2CPP_HandRegister(0x00002e43u, &step_cmp_r2_b);
    MK2CPP_HandRegister(0x00002e46u, &step_bls_b);
    MK2CPP_HandRegister(0x00002e48u, &step_set_limit_r2_b);
    MK2CPP_HandRegister(0x00002e4bu, &step_bra_d);
    MK2CPP_HandRegister(0x00002e4du, &step_add_r3_r2_c);
    MK2CPP_HandRegister(0x00002e4fu, &step_bpl_c);
    MK2CPP_HandRegister(0x00002e51u, &step_neg_r2_c);
    MK2CPP_HandRegister(0x00002e53u, &step_bra_e);
    MK2CPP_HandRegister(0x00002e55u, &step_tst_r6_a);
    MK2CPP_HandRegister(0x00002e57u, &step_bpl_d);
    MK2CPP_HandRegister(0x00002e59u, &step_neg_r6_a);
    MK2CPP_HandRegister(0x00002e5bu, &step_bra_f);
    MK2CPP_HandRegister(0x00002e5du, &step_tst_r6_b);
    MK2CPP_HandRegister(0x00002e5fu, &step_bpl_e);
    MK2CPP_HandRegister(0x00002e61u, &step_neg_r6_b);
    MK2CPP_HandRegister(0x00002e63u, &step_mulxu_reg_c);
    MK2CPP_HandRegister(0x00002e65u, &step_double_r3_e);
    MK2CPP_HandRegister(0x00002e67u, &step_addx_r2_e);
    MK2CPP_HandRegister(0x00002e69u, &step_add_imm_a);
    MK2CPP_HandRegister(0x00002e6du, &step_addx_r2_r4);
    MK2CPP_HandRegister(0x00002e6fu, &step_bra_g);
    MK2CPP_HandRegister(0x00002e71u, &step_mulxu_reg_d);
    MK2CPP_HandRegister(0x00002e73u, &step_double_r3_f);
    MK2CPP_HandRegister(0x00002e75u, &step_addx_r2_f);
    MK2CPP_HandRegister(0x00002e77u, &step_add_imm_b);
    MK2CPP_HandRegister(0x00002e7bu, &step_subx_r2_r4);
    MK2CPP_HandRegister(0x00002e7du, &step_bcc_c);
    MK2CPP_HandRegister(0x00002e7fu, &step_clr_r4_b);
    MK2CPP_HandRegister(0x00002e81u, &step_return_d);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct PitchEnvSelfRegister
{
    PitchEnvSelfRegister() { hand_register_module(&pitch_env_fill); }
};

PitchEnvSelfRegister g_pitch_env_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
