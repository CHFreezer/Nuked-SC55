/*
 * HAND pcm/voice_param — per-voice parameter processing (class C9,
 * rom1 cp=0, flat 0x2e83..0x30f0).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13, named operations); addresses and registration unchanged. One
 * L0 hand entry per instruction PC; see pcm_irq_service.cpp for the L0
 * rationale and registration policy.
 *
 * Semantics: per-voice parameter processing; entry arms 0x2e83/0x2e85
 * (trampoline + body), reads/clears the acf2 fields at 0x2f17/0x2f1e and
 * pairs with mask_acc (B7); the tail 0x3061..0x30f0 BRA returns to ts_scan
 * (0x591c). Called from ts_scan.
 * Evidence: out/m4/18_closure_gap.md 1.3 C9 / 4.2 row 10; out/m4/16 §0-1.2;
 * docs/07:143. Confidence: C (ROM bytes + dasm).
 *
 * Registration: all PCs self-register via MK2CPP_HandRegister from a
 * file-static initializer (hand_registry.h); no shared aggregator file is
 * edited and duplicate registration is fatal (mk2cpp.cpp).
 */

#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_interrupt.h"
#include "mcu_opcodes.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; declared locally like the other hand modules. */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* ---- local helpers (bodies copied from the previous generated form) ------- */

/* Page register for rN: dp (r0-r3), ep (r4/r5), tp (r6/r7). */
uint32_t page_of_reg(uint32_t reg)
{
    return (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
}

/* @rN+disp: the rN+disp sum wraps inside 16 bits. */
uint32_t ind_addr(uint32_t reg, int disp)
{
    return ((uint32_t)page_of_reg(reg) << 16) |
           (uint16_t)(mcu.r[reg] + (uint16_t)(int16_t)disp);
}

/* (dp,disp16): absolute through the DP page register. */
uint32_t dp_addr(uint16_t disp)
{
    return ((uint32_t)mcu.dp << 16) | disp;
}

/* (br,disp8): the BR page register, low byte offset. */
uint32_t br_addr(uint8_t disp)
{
    return ((uint32_t)mcu.br << 8) | disp;
}

/* CLR flags: N=0 Z=1 V=0 C=0. */
void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG2 @addr -> rN (byte): low byte replaced, high byte kept. */
void load8(uint16_t &reg, uint32_t addr)
{
    uint32_t data = (uint32_t)MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* MOVG2 @addr -> rN (word): odd-address check first, then flags from data. */
void load16(uint16_t &reg, uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(addr);
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVG3 rN -> @addr (byte). */
void store8(uint32_t addr, uint32_t value)
{
    MCU_Write(addr, (uint8_t)value);
    MCU_SetStatusCommon(value, 0);
}

/* MOVG3 rN -> @addr (word): odd-address check first, then write + flags. */
void store16(uint32_t addr, uint32_t value)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, (uint16_t)value);
    MCU_SetStatusCommon(value, 1);
}

/* CLR @addr (byte). */
void clr8_mem(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

/* CLR @addr (word): odd-address check first. */
void clr16_mem(uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, 0);
    flags_clr();
}

/* TST @addr (byte): N/Z from the value, C=0, V untouched. */
void tst8_mem(uint32_t addr)
{
    MCU_SetStatusCommon((uint32_t)MCU_Read(addr), 0);
    MCU_SetStatus(0, STATUS_C);
}

/* TST @addr (word): N/Z from the value, C=0, V untouched. */
void tst16_mem(uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_SetStatusCommon((uint32_t)MCU_Read16(addr), 1);
    MCU_SetStatus(0, STATUS_C);
}

/* SUB @addr #imm (word): flags only, no write. */
void sub16_mem_nowrite(uint32_t addr, uint16_t imm)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t1 = (uint32_t)MCU_Read16(addr);
    uint32_t t2 = (uint32_t)imm;
    MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
}

/* SUB @addr #imm (byte): flags only, no write. */
void sub8_mem_nowrite(uint32_t addr, uint8_t imm)
{
    uint32_t t1 = (uint32_t)MCU_Read(addr);
    uint32_t t2 = (uint32_t)imm;
    MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
}

/* CMP @addr,rN (byte): t1 = full rN, t2 = byte memory. */
void cmp8_mem_reg(uint32_t addr, uint16_t reg)
{
    int32_t t1 = (int32_t)reg;
    uint32_t t2 = (uint32_t)MCU_Read(addr);
    MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
}

/* CMP @addr,rN (word): t1 = full rN, t2 = word memory. */
void cmp16_mem_reg(uint32_t addr, uint16_t reg)
{
    int32_t t1 = (int32_t)reg;
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(addr);
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

/* cmp rN,b #imm: t1 = full rN, t2 = imm8, byte flags. */
void cmp8_reg_imm(uint16_t reg, uint8_t imm)
{
    int32_t t2 = (int32_t)(uint32_t)imm;
    int32_t t1 = (int32_t)reg;
    MCU_SUB_Common(t1, t2, 0, 0);
}

/* ADD rS,rD (word). */
void add16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, 0, 1);
}

/* ADDQ #delta,rN (word). */
void addq16(uint16_t &reg, int delta)
{
    reg = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)reg, delta, 0, 1);
}

/* SHLR rN (word). */
void shlr16(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t C = data & 1;
    data >>= 1;
    reg = (uint16_t)data;
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
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

/* EXTU rN: zero-extend the low byte. */
void extu8(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    reg = (uint16_t)data;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(data == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* move rN #imm8: low byte replaced, high byte kept. */
void move8(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* movs rN @(br,disp): store the low byte through the BR page. */
void movs_br(uint16_t &reg, uint8_t disp)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    MCU_Write(br_addr(disp), (uint8_t)data);
    MCU_SetStatusCommon(data, 0);
}

/* movl rN @(br,disp): load a byte into the low half (high kept). */
void movl_br(uint16_t &reg, uint8_t disp)
{
    uint32_t data = (uint32_t)MCU_Read(br_addr(disp));
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* movlw rN @(br,disp): load a word. */
void movlw_br(uint16_t &reg, uint8_t disp)
{
    uint32_t addr = br_addr(disp);
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t data = MCU_Read16(addr);
    reg = data;
    MCU_SetStatusCommon((uint32_t)data, 1);
}

/* movsw rN @(br,disp): store a word. */
void movsw_br(uint16_t &reg, uint8_t disp)
{
    uint32_t addr = br_addr(disp);
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)reg;
    MCU_Write16(addr, (uint16_t)data);
    MCU_SetStatusCommon(data, 1);
}

/* BSET_ORC #imm,r0: OR into the control register, hold the poll. */
void bset_orc_cr0(uint16_t imm)
{
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val |= (uint32_t)imm;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

/* BCLR_ANDC #imm,r0: AND into the control register, hold the poll. */
void bclr_andc_cr0(uint16_t imm)
{
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val &= (uint32_t)imm;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

/* trapa #imm. */
void trapa(uint8_t opcode)
{
    if ((opcode & 0xf0) == 0x10)
        MCU_Interrupt_TRAPA(opcode & 0x0f);
    else
        MCU_ErrorTrap();
}

/* ---- conditional branches (target/fall are absolute) ---------------------- */

void bcc(uint16_t taken, uint16_t fall)
{
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    mcu.pc = (C == 0) ? taken : fall;
}

void beq(uint16_t taken, uint16_t fall)
{
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    mcu.pc = (Z == 1) ? taken : fall;
}

void bne(uint16_t taken, uint16_t fall)
{
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    mcu.pc = (Z == 0) ? taken : fall;
}

/* bsr/jsr: push the return PC, then jump. */
void call(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

/* ======================================================================
 * cp0 0x2E82..0x30EF, 213 PC: C9 per-voice parameter processing.
 * ====================================================================== */

/* 0x2E82 rts. */
void step_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x2E83 bsr -3 -> 0x2e82. */
void step_bsr_3_0x2e82(void)
{
    call(0x2e85, 0x2e82);
}

/* 0x2E85 BSET_ORC #0x0700 r0. */
void step_bset_orc_0x0700_r0(void)
{
    bset_orc_cr0(0x0700);
    mcu.pc = 0x2e89;
}

/* 0x2E89 MOVG #0x00ff -> @r0+-26. */
void step_movg_0x00ff_r0_26(void)
{
    store8(ind_addr(0, -26), 0xff);
    mcu.pc = 0x2e8e;
}

/* 0x2E8E SUB @r0+0 #0x000e. */
void step_sub_r0_0_0x000e(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x000e);
    mcu.pc = 0x2e93;
}

/* 0x2E93 BCC 0x01cb -> 0x3061. */
void step_bcc_0x01cb_0x3061(void)
{
    bcc(0x3061, 0x2e96);
}

/* 0x2E96 SUB @r0+0 #0x0000. */
void step_sub_r0_0_0x0000(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x0000);
    mcu.pc = 0x2e9b;
}

/* 0x2E9B BEQ 0x0076 -> 0x2f14. */
void step_beq_0x0076_0x2f14(void)
{
    beq(0x2f14, 0x2e9e);
}

/* 0x2E9E MOVG2 @r0+-2 r1. */
void step_movg2_r0_2_r1(void)
{
    load16(mcu.r[1], ind_addr(0, -2));
    mcu.pc = 0x2ea1;
}

/* 0x2EA1 movs r1 @(br,$3e). */
void step_movs_r1_br_3e(void)
{
    movs_br(mcu.r[1], 0x3e);
    mcu.pc = 0x2ea3;
}

/* 0x2EA3 SUB @r0+26 #0xff00. */
void step_sub_r0_26_0xff00(void)
{
    sub16_mem_nowrite(ind_addr(0, 26), 0xff00);
    mcu.pc = 0x2ea8;
}

/* 0x2EA8 BEQ 16 -> 0x2eba. */
void step_beq_16_0x2eba(void)
{
    beq(0x2eba, 0x2eaa);
}

/* 0x2EAA MOVG #0xff00 -> (br,$16). */
void step_movg_0xff00_br_16(void)
{
    store16(br_addr(0x16), 0xff00);
    mcu.pc = 0x2eaf;
}

/* 0x2EAF movl r5 @(br,$32). */
void step_movl_r5_br_32(void)
{
    movl_br(mcu.r[5], 0x32);
    mcu.pc = 0x2eb1;
}

/* 0x2EB1 movlw r5 @(br,$3a). */
void step_movlw_r5_br_3a(void)
{
    movlw_br(mcu.r[5], 0x3a);
    mcu.pc = 0x2eb3;
}

/* 0x2EB3 ADD r5 r5. */
void step_add_r5_r5(void)
{
    add16_reg(mcu.r[5], mcu.r[5]);
    mcu.pc = 0x2eb5;
}

/* 0x2EB5 MOVG3 r5 -> @r0+24. */
void step_movg3_r5_r0_24(void)
{
    store16(ind_addr(0, 24), mcu.r[5]);
    mcu.pc = 0x2eb8;
}

/* 0x2EB8 BRA 11 -> 0x2ec5. */
void step_bra_11_0x2ec5(void)
{
    mcu.pc = 0x2ec5;
}

/* 0x2EBA MOVG2 @r0+24 r5. */
void step_movg2_r0_24_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 24));
    mcu.pc = 0x2ebd;
}

/* 0x2EBD SHLR r5. */
void step_shlr_r5(void)
{
    shlr16(mcu.r[5]);
    mcu.pc = 0x2ebf;
}

/* 0x2EBF movsw r5 @(br,$32). */
void step_movsw_r5_br_32(void)
{
    movsw_br(mcu.r[5], 0x32);
    mcu.pc = 0x2ec1;
}

/* 0x2EC1 bsr -65 -> 0x2e82. */
void step_bsr_65_0x2e82(void)
{
    call(0x2ec3, 0x2e82);
}

/* 0x2EC3 movsw r5 @(br,$32). */
void step_movsw_r5_br_32_b(void)
{
    movsw_br(mcu.r[5], 0x32);
    mcu.pc = 0x2ec5;
}

/* 0x2EC5 SUB @r0+30 #0xff00. */
void step_sub_r0_30_0xff00(void)
{
    sub16_mem_nowrite(ind_addr(0, 30), 0xff00);
    mcu.pc = 0x2eca;
}

/* 0x2ECA BEQ 16 -> 0x2edc. */
void step_beq_16_0x2edc(void)
{
    beq(0x2edc, 0x2ecc);
}

/* 0x2ECC MOVG #0xff00 -> (br,$18). */
void step_movg_0xff00_br_18(void)
{
    store16(br_addr(0x18), 0xff00);
    mcu.pc = 0x2ed1;
}

/* 0x2ED1 movl r5 @(br,$34). */
void step_movl_r5_br_34(void)
{
    movl_br(mcu.r[5], 0x34);
    mcu.pc = 0x2ed3;
}

/* 0x2ED3 movlw r5 @(br,$3a). */
void step_movlw_r5_br_3a_b(void)
{
    movlw_br(mcu.r[5], 0x3a);
    mcu.pc = 0x2ed5;
}

/* 0x2ED5 ADD r5 r5. */
void step_add_r5_r5_b(void)
{
    add16_reg(mcu.r[5], mcu.r[5]);
    mcu.pc = 0x2ed7;
}

/* 0x2ED7 MOVG3 r5 -> @r0+28. */
void step_movg3_r5_r0_28(void)
{
    store16(ind_addr(0, 28), mcu.r[5]);
    mcu.pc = 0x2eda;
}

/* 0x2EDA BRA 11 -> 0x2ee7. */
void step_bra_11_0x2ee7(void)
{
    mcu.pc = 0x2ee7;
}

/* 0x2EDC MOVG2 @r0+28 r5. */
void step_movg2_r0_28_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 28));
    mcu.pc = 0x2edf;
}

/* 0x2EDF SHLR r5. */
void step_shlr_r5_b(void)
{
    shlr16(mcu.r[5]);
    mcu.pc = 0x2ee1;
}

/* 0x2EE1 movsw r5 @(br,$34). */
void step_movsw_r5_br_34(void)
{
    movsw_br(mcu.r[5], 0x34);
    mcu.pc = 0x2ee3;
}

/* 0x2EE3 bsr -99 -> 0x2e82. */
void step_bsr_99_0x2e82(void)
{
    call(0x2ee5, 0x2e82);
}

/* 0x2EE5 movsw r5 @(br,$34). */
void step_movsw_r5_br_34_b(void)
{
    movsw_br(mcu.r[5], 0x34);
    mcu.pc = 0x2ee7;
}

/* 0x2EE7 MOVG2 @r0+28 r5. */
void step_movg2_r0_28_r5_b(void)
{
    load16(mcu.r[5], ind_addr(0, 28));
    mcu.pc = 0x2eea;
}

/* 0x2EEA SWAP r5. */
void step_swap_r5(void)
{
    swap16(mcu.r[5]);
    mcu.pc = 0x2eec;
}

/* 0x2EEC cmp r5,b #0xff. */
void step_cmp_r5_b_0xff(void)
{
    cmp8_reg_imm(mcu.r[5], 0xff);
    mcu.pc = 0x2eee;
}

/* 0x2EEE BNE 2 -> 0x2ef2. */
void step_bne_2_0x2ef2(void)
{
    bne(0x2ef2, 0x2ef0);
}

/* 0x2EF0 move r5 #0xfe. */
void step_move_r5_0xfe(void)
{
    move8(mcu.r[5], 0xfe);
    mcu.pc = 0x2ef2;
}

/* 0x2EF2 MOVG3 r5 -> @r1+0xad0e. */
void step_movg3_r5_r1_0xad0e(void)
{
    store8(ind_addr(1, 0xad0e), mcu.r[5]);
    mcu.pc = 0x2ef6;
}

/* 0x2EF6 SUB @r0+38 #0xff00. */
void step_sub_r0_38_0xff00(void)
{
    sub16_mem_nowrite(ind_addr(0, 38), 0xff00);
    mcu.pc = 0x2efb;
}

/* 0x2EFB BEQ 16 -> 0x2f0d. */
void step_beq_16_0x2f0d(void)
{
    beq(0x2f0d, 0x2efd);
}

/* 0x2EFD MOVG #0xff00 -> (br,$1a). */
void step_movg_0xff00_br_1a(void)
{
    store16(br_addr(0x1a), 0xff00);
    mcu.pc = 0x2f02;
}

/* 0x2F02 movl r5 @(br,$36). */
void step_movl_r5_br_36(void)
{
    movl_br(mcu.r[5], 0x36);
    mcu.pc = 0x2f04;
}

/* 0x2F04 movlw r5 @(br,$3a). */
void step_movlw_r5_br_3a_c(void)
{
    movlw_br(mcu.r[5], 0x3a);
    mcu.pc = 0x2f06;
}

/* 0x2F06 ADD r5 r5. */
void step_add_r5_r5_c(void)
{
    add16_reg(mcu.r[5], mcu.r[5]);
    mcu.pc = 0x2f08;
}

/* 0x2F08 MOVG3 r5 -> @r0+36. */
void step_movg3_r5_r0_36(void)
{
    store16(ind_addr(0, 36), mcu.r[5]);
    mcu.pc = 0x2f0b;
}

/* 0x2F0B BRA 7 -> 0x2f14. */
void step_bra_7_0x2f14(void)
{
    mcu.pc = 0x2f14;
}

/* 0x2F0D MOVG2 @r0+36 r5. */
void step_movg2_r0_36_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 36));
    mcu.pc = 0x2f10;
}

/* 0x2F10 SHLR r5. */
void step_shlr_r5_c(void)
{
    shlr16(mcu.r[5]);
    mcu.pc = 0x2f12;
}

/* 0x2F12 movsw r5 @(br,$36). */
void step_movsw_r5_br_36(void)
{
    movsw_br(mcu.r[5], 0x36);
    mcu.pc = 0x2f14;
}

/* 0x2F14 MOVG2 @r0+-2 r1. */
void step_movg2_r0_2_r1_b(void)
{
    load16(mcu.r[1], ind_addr(0, -2));
    mcu.pc = 0x2f17;
}

/* 0x2F17 TST @r1+0xacf2. */
void step_tst_r1_0xacf2(void)
{
    tst8_mem(ind_addr(1, 0xacf2));
    mcu.pc = 0x2f1b;
}

/* 0x2F1B BEQ 0x00d0 -> 0x2fee. */
void step_beq_0x00d0_0x2fee(void)
{
    beq(0x2fee, 0x2f1e);
}

/* 0x2F1E CLR @r1+0xacf2. */
void step_clr_r1_0xacf2(void)
{
    clr8_mem(ind_addr(1, 0xacf2));
    mcu.pc = 0x2f22;
}

/* 0x2F22 SUB @r0+0 #0x000c. */
void step_sub_r0_0_0x000c(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x000c);
    mcu.pc = 0x2f27;
}

/* 0x2F27 BCC 0x00c4 -> 0x2fee. */
void step_bcc_0x00c4_0x2fee(void)
{
    bcc(0x2fee, 0x2f2a);
}

/* 0x2F2A SUB @r0+0 #0x0000. */
void step_sub_r0_0_0x0000_b(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x0000);
    mcu.pc = 0x2f2f;
}

/* 0x2F2F BNE 44 -> 0x2f5d. */
void step_bne_44_0x2f5d(void)
{
    bne(0x2f5d, 0x2f31);
}

/* 0x2F31 TST @r0+16. */
void step_tst_r0_16(void)
{
    tst16_mem(ind_addr(0, 16));
    mcu.pc = 0x2f34;
}

/* 0x2F34 BNE 18 -> 0x2f48. */
void step_bne_18_0x2f48(void)
{
    bne(0x2f48, 0x2f36);
}

/* 0x2F36 MOVG #0x0002 -> @r0+0. */
void step_movg_0x0002_r0_0(void)
{
    store16(ind_addr(0, 0), 0x0002);
    mcu.pc = 0x2f3b;
}

/* 0x2F3B MOVG #0x0002 -> @r0+2. */
void step_movg_0x0002_r0_2(void)
{
    store16(ind_addr(0, 2), 0x0002);
    mcu.pc = 0x2f40;
}

/* 0x2F40 MOVG #0x0002 -> @r0+4. */
void step_movg_0x0002_r0_4(void)
{
    store16(ind_addr(0, 4), 0x0002);
    mcu.pc = 0x2f45;
}

/* 0x2F45 BRA 0x00a6 -> 0x2fee. */
void step_bra_0x00a6_0x2fee(void)
{
    mcu.pc = 0x2fee;
}

/* 0x2F48 MOVG #0x0016 -> @r0+0. */
void step_movg_0x0016_r0_0(void)
{
    store16(ind_addr(0, 0), 0x0016);
    mcu.pc = 0x2f4d;
}

/* 0x2F4D MOVG #0x0016 -> @r0+2. */
void step_movg_0x0016_r0_2(void)
{
    store16(ind_addr(0, 2), 0x0016);
    mcu.pc = 0x2f52;
}

/* 0x2F52 MOVG #0x0016 -> @r0+4. */
void step_movg_0x0016_r0_4(void)
{
    store16(ind_addr(0, 4), 0x0016);
    mcu.pc = 0x2f57;
}

/* 0x2F57 MOVG2 @r0+-2 r1. */
void step_movg2_r0_2_r1_c(void)
{
    load16(mcu.r[1], ind_addr(0, -2));
    mcu.pc = 0x2f5a;
}

/* 0x2F5A BRA 0x0134 -> 0x3091. */
void step_bra_0x0134_0x3091(void)
{
    mcu.pc = 0x3091;
}

/* 0x2F5D MOVG #0x000c -> @r0+0. */
void step_movg_0x000c_r0_0(void)
{
    store16(ind_addr(0, 0), 0x000c);
    mcu.pc = 0x2f62;
}

/* 0x2F62 MOVG #0x000c -> @r0+2. */
void step_movg_0x000c_r0_2(void)
{
    store16(ind_addr(0, 2), 0x000c);
    mcu.pc = 0x2f67;
}

/* 0x2F67 MOVG #0x000c -> @r0+4. */
void step_movg_0x000c_r0_4(void)
{
    store16(ind_addr(0, 4), 0x000c);
    mcu.pc = 0x2f6c;
}

/* 0x2F6C MOVG2 @r0+28 r5. */
void step_movg2_r0_28_r5_c(void)
{
    load16(mcu.r[5], ind_addr(0, 28));
    mcu.pc = 0x2f6f;
}

/* 0x2F6F SWAP r5. */
void step_swap_r5_b(void)
{
    swap16(mcu.r[5]);
    mcu.pc = 0x2f71;
}

/* 0x2F71 MOVG3 r5 -> @r0+96. */
void step_movg3_r5_r0_96(void)
{
    store8(ind_addr(0, 96), mcu.r[5]);
    mcu.pc = 0x2f74;
}

/* 0x2F74 CLR @r0+97. */
void step_clr_r0_97(void)
{
    clr8_mem(ind_addr(0, 97));
    mcu.pc = 0x2f77;
}

/* 0x2F77 CLR @r0+8. */
void step_clr_r0_8(void)
{
    clr16_mem(ind_addr(0, 8));
    mcu.pc = 0x2f7a;
}

/* 0x2F7A CLR @r0+18. */
void step_clr_r0_18(void)
{
    clr16_mem(ind_addr(0, 18));
    mcu.pc = 0x2f7d;
}

/* 0x2F7D MOVG2 @r0+83 r5. */
void step_movg2_r0_83_r5(void)
{
    load8(mcu.r[5], ind_addr(0, 83));
    mcu.pc = 0x2f80;
}

/* 0x2F80 MOVG3 r5 -> @r0+79. */
void step_movg3_r5_r0_79(void)
{
    store8(ind_addr(0, 79), mcu.r[5]);
    mcu.pc = 0x2f83;
}

/* 0x2F83 MOVG2 @r0+-4 r5. */
void step_movg2_r0_4_r5(void)
{
    load8(mcu.r[5], ind_addr(0, -4));
    mcu.pc = 0x2f86;
}

/* 0x2F86 MOVG3 r5 -> @r0+-8. */
void step_movg3_r5_r0_8(void)
{
    store8(ind_addr(0, -8), mcu.r[5]);
    mcu.pc = 0x2f89;
}

/* 0x2F89 CLR @r0+10. */
void step_clr_r0_10(void)
{
    clr16_mem(ind_addr(0, 10));
    mcu.pc = 0x2f8c;
}

/* 0x2F8C CLR @r0+20. */
void step_clr_r0_20(void)
{
    clr16_mem(ind_addr(0, 20));
    mcu.pc = 0x2f8f;
}

/* 0x2F8F MOVG2 @r0+78 r5. */
void step_movg2_r0_78_r5(void)
{
    load8(mcu.r[5], ind_addr(0, 78));
    mcu.pc = 0x2f92;
}

/* 0x2F92 MOVG3 r5 -> @r0+74. */
void step_movg3_r5_r0_74(void)
{
    store8(ind_addr(0, 74), mcu.r[5]);
    mcu.pc = 0x2f95;
}

/* 0x2F95 MOVG2 @r0+32 r5. */
void step_movg2_r0_32_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 32));
    mcu.pc = 0x2f98;
}

/* 0x2F98 MOVG3 r5 -> @r0+84. */
void step_movg3_r5_r0_84(void)
{
    store16(ind_addr(0, 84), mcu.r[5]);
    mcu.pc = 0x2f9b;
}

/* 0x2F9B MOVG2 @r0+94 r5. */
void step_movg2_r0_94_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 94));
    mcu.pc = 0x2f9e;
}

/* 0x2F9E MOVG3 r5 -> @r0+86. */
void step_movg3_r5_r0_86(void)
{
    store16(ind_addr(0, 86), mcu.r[5]);
    mcu.pc = 0x2fa1;
}

/* 0x2FA1 MOVG2 @r0+44 r4. */
void step_movg2_r0_44_r4(void)
{
    load8(mcu.r[4], ind_addr(0, 44));
    mcu.pc = 0x2fa4;
}

/* 0x2FA4 MOVG2 @r0+68 r5. */
void step_movg2_r0_68_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 68));
    mcu.pc = 0x2fa7;
}

/* 0x2FA7 MOVG3 r4 -> @r0+106. */
void step_movg3_r4_r0_106(void)
{
    store8(ind_addr(0, 106), mcu.r[4]);
    mcu.pc = 0x2faa;
}

/* 0x2FAA MOVG3 r5 -> @r0+112. */
void step_movg3_r5_r0_112(void)
{
    store16(ind_addr(0, 112), mcu.r[5]);
    mcu.pc = 0x2fad;
}

/* 0x2FAD MOVG2 @r0+111 r4. */
void step_movg2_r0_111_r4(void)
{
    load8(mcu.r[4], ind_addr(0, 111));
    mcu.pc = 0x2fb0;
}

/* 0x2FB0 MOVG2 @r0+122 r5. */
void step_movg2_r0_122_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 122));
    mcu.pc = 0x2fb3;
}

/* 0x2FB3 MOVG3 r4 -> @r0+107. */
void step_movg3_r4_r0_107(void)
{
    store8(ind_addr(0, 107), mcu.r[4]);
    mcu.pc = 0x2fb6;
}

/* 0x2FB6 MOVG3 r5 -> @r0+114. */
void step_movg3_r5_r0_114(void)
{
    store16(ind_addr(0, 114), mcu.r[5]);
    mcu.pc = 0x2fb9;
}

/* 0x2FB9 CMP @r0+44 r4. */
void step_cmp_r0_44_r4(void)
{
    cmp8_mem_reg(ind_addr(0, 44), mcu.r[4]);
    mcu.pc = 0x2fbc;
}

/* 0x2FBC BNE 3 -> 0x2fc1. */
void step_bne_3_0x2fc1(void)
{
    bne(0x2fc1, 0x2fbe);
}

/* 0x2FBE CMP @r0+68 r5. */
void step_cmp_r0_68_r5(void)
{
    cmp16_mem_reg(ind_addr(0, 68), mcu.r[5]);
    mcu.pc = 0x2fc1;
}

/* 0x2FC1 BCC 6 -> 0x2fc9. */
void step_bcc_6_0x2fc9(void)
{
    bcc(0x2fc9, 0x2fc3);
}

/* 0x2FC3 MOVG #0x02 -> @r0+-3. */
void step_movg_0x02_r0_3(void)
{
    store8(ind_addr(0, -3), 0x02);
    mcu.pc = 0x2fc7;
}

/* 0x2FC7 BRA 4 -> 0x2fcd. */
void step_bra_4_0x2fcd(void)
{
    mcu.pc = 0x2fcd;
}

/* 0x2FC9 MOVG #0x00 -> @r0+-3. */
void step_movg_0x00_r0_3(void)
{
    store8(ind_addr(0, -3), 0x00);
    mcu.pc = 0x2fcd;
}

/* 0x2FCD MOVG2 @r0+0x0084 r5. */
void step_movg2_r0_0x0084_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 0x0084));
    mcu.pc = 0x2fd1;
}

/* 0x2FD1 MOVG3 r5 -> @r0+124. */
void step_movg3_r5_r0_124(void)
{
    store16(ind_addr(0, 124), mcu.r[5]);
    mcu.pc = 0x2fd4;
}

/* 0x2FD4 CLR @r0+12. */
void step_clr_r0_12(void)
{
    clr16_mem(ind_addr(0, 12));
    mcu.pc = 0x2fd7;
}

/* 0x2FD7 CLR @r0+22. */
void step_clr_r0_22(void)
{
    clr16_mem(ind_addr(0, 22));
    mcu.pc = 0x2fda;
}

/* 0x2FDA TST @r0+-78. */
void step_tst_r0_78(void)
{
    tst16_mem(ind_addr(0, -78));
    mcu.pc = 0x2fdd;
}

/* 0x2FDD BNE 5 -> 0x2fe4. */
void step_bne_5_0x2fe4(void)
{
    bne(0x2fe4, 0x2fdf);
}

/* 0x2FDF MOVG #0xffff -> @r0+-78. */
void step_movg_0xffff_r0_78(void)
{
    store16(ind_addr(0, -78), 0xffff);
    mcu.pc = 0x2fe4;
}

/* 0x2FE4 TST @r0+-112. */
void step_tst_r0_112(void)
{
    tst16_mem(ind_addr(0, -112));
    mcu.pc = 0x2fe7;
}

/* 0x2FE7 BNE 5 -> 0x2fee. */
void step_bne_5_0x2fee(void)
{
    bne(0x2fee, 0x2fe9);
}

/* 0x2FE9 MOVG #0xffff -> @r0+-112. */
void step_movg_0xffff_r0_112(void)
{
    store16(ind_addr(0, -112), 0xffff);
    mcu.pc = 0x2fee;
}

/* 0x2FEE BCLR_ANDC #0xf8ff r0. */
void step_bclr_andc_0xf8ff_r0(void)
{
    bclr_andc_cr0(0xf8ff);
    mcu.pc = 0x2ff2;
}

/* 0x2FF2 nop. */
void step_nop(void)
{
    mcu.pc = 0x2ff3;
}

/* 0x2FF3 nop. */
void step_nop_b(void)
{
    mcu.pc = 0x2ff4;
}

/* 0x2FF4 nop. */
void step_nop_c(void)
{
    mcu.pc = 0x2ff5;
}

/* 0x2FF5 nop. */
void step_nop_d(void)
{
    mcu.pc = 0x2ff6;
}

/* 0x2FF6 BSET_ORC #0x0700 r0. */
void step_bset_orc_0x0700_r0_b(void)
{
    bset_orc_cr0(0x0700);
    mcu.pc = 0x2ffa;
}

/* 0x2FFA SUB @r0+0 #0x000e. */
void step_sub_r0_0_0x000e_b(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x000e);
    mcu.pc = 0x2fff;
}

/* 0x2FFF BCC 96 -> 0x3061. */
void step_bcc_96_0x3061(void)
{
    bcc(0x3061, 0x3001);
}

/* 0x3001 bsr16 -> 0x37fe. */
void step_bsr16_0x37fe(void)
{
    call(0x3004, 0x37fe);
}

/* 0x3004 BCLR_ANDC #0xf8ff r0. */
void step_bclr_andc_0xf8ff_r0_b(void)
{
    bclr_andc_cr0(0xf8ff);
    mcu.pc = 0x3008;
}

/* 0x3008 nop. */
void step_nop_e(void)
{
    mcu.pc = 0x3009;
}

/* 0x3009 nop. */
void step_nop_f(void)
{
    mcu.pc = 0x300a;
}

/* 0x300A nop. */
void step_nop_g(void)
{
    mcu.pc = 0x300b;
}

/* 0x300B nop. */
void step_nop_h(void)
{
    mcu.pc = 0x300c;
}

/* 0x300C BSET_ORC #0x0700 r0. */
void step_bset_orc_0x0700_r0_c(void)
{
    bset_orc_cr0(0x0700);
    mcu.pc = 0x3010;
}

/* 0x3010 SUB @r0+0 #0x000e. */
void step_sub_r0_0_0x000e_c(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x000e);
    mcu.pc = 0x3015;
}

/* 0x3015 BCC 74 -> 0x3061. */
void step_bcc_74_0x3061(void)
{
    bcc(0x3061, 0x3017);
}

/* 0x3017 bsr16 -> 0x30f2. */
void step_bsr16_0x30f2(void)
{
    call(0x301a, 0x30f2);
}

/* 0x301A BCLR_ANDC #0xf8ff r0. */
void step_bclr_andc_0xf8ff_r0_c(void)
{
    bclr_andc_cr0(0xf8ff);
    mcu.pc = 0x301e;
}

/* 0x301E nop. */
void step_nop_i(void)
{
    mcu.pc = 0x301f;
}

/* 0x301F nop. */
void step_nop_j(void)
{
    mcu.pc = 0x3020;
}

/* 0x3020 nop. */
void step_nop_k(void)
{
    mcu.pc = 0x3021;
}

/* 0x3021 nop. */
void step_nop_l(void)
{
    mcu.pc = 0x3022;
}

/* 0x3022 BSET_ORC #0x0700 r0. */
void step_bset_orc_0x0700_r0_d(void)
{
    bset_orc_cr0(0x0700);
    mcu.pc = 0x3026;
}

/* 0x3026 SUB @r0+0 #0x000e. */
void step_sub_r0_0_0x000e_d(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x000e);
    mcu.pc = 0x302b;
}

/* 0x302B BCC 52 -> 0x3061. */
void step_bcc_52_0x3061(void)
{
    bcc(0x3061, 0x302d);
}

/* 0x302D bsr16 -> 0x41bb. */
void step_bsr16_0x41bb(void)
{
    call(0x3030, 0x41bb);
}

/* 0x3030 BCLR_ANDC #0xf8ff r0. */
void step_bclr_andc_0xf8ff_r0_d(void)
{
    bclr_andc_cr0(0xf8ff);
    mcu.pc = 0x3034;
}

/* 0x3034 nop. */
void step_nop_m(void)
{
    mcu.pc = 0x3035;
}

/* 0x3035 nop. */
void step_nop_n(void)
{
    mcu.pc = 0x3036;
}

/* 0x3036 nop. */
void step_nop_o(void)
{
    mcu.pc = 0x3037;
}

/* 0x3037 nop. */
void step_nop_p(void)
{
    mcu.pc = 0x3038;
}

/* 0x3038 BSET_ORC #0x0700 r0. */
void step_bset_orc_0x0700_r0_e(void)
{
    bset_orc_cr0(0x0700);
    mcu.pc = 0x303c;
}

/* 0x303C SUB @r0+0 #0x000e. */
void step_sub_r0_0_0x000e_e(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x000e);
    mcu.pc = 0x3041;
}

/* 0x3041 BCC 30 -> 0x3061. */
void step_bcc_30_0x3061(void)
{
    bcc(0x3061, 0x3043);
}

/* 0x3043 bsr16 -> 0x4d62. */
void step_bsr16_0x4d62(void)
{
    call(0x3046, 0x4d62);
}

/* 0x3046 BCLR_ANDC #0xf8ff r0. */
void step_bclr_andc_0xf8ff_r0_e(void)
{
    bclr_andc_cr0(0xf8ff);
    mcu.pc = 0x304a;
}

/* 0x304A nop. */
void step_nop_q(void)
{
    mcu.pc = 0x304b;
}

/* 0x304B nop. */
void step_nop_r(void)
{
    mcu.pc = 0x304c;
}

/* 0x304C nop. */
void step_nop_s(void)
{
    mcu.pc = 0x304d;
}

/* 0x304D nop. */
void step_nop_t(void)
{
    mcu.pc = 0x304e;
}

/* 0x304E BSET_ORC #0x0700 r0. */
void step_bset_orc_0x0700_r0_f(void)
{
    bset_orc_cr0(0x0700);
    mcu.pc = 0x3052;
}

/* 0x3052 SUB @r0+0 #0x000e. */
void step_sub_r0_0_0x000e_f(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x000e);
    mcu.pc = 0x3057;
}

/* 0x3057 BCC 8 -> 0x3061. */
void step_bcc_8_0x3061(void)
{
    bcc(0x3061, 0x3059);
}

/* 0x3059 bsr16 -> 0x3465. */
void step_bsr16_0x3465(void)
{
    call(0x305c, 0x3465);
}

/* 0x305C BCLR_ANDC #0xf8ff r0. */
void step_bclr_andc_0xf8ff_r0_f(void)
{
    bclr_andc_cr0(0xf8ff);
    mcu.pc = 0x3060;
}

/* 0x3060 rts. */
void step_rts_b(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x3061 SUB @r0+0 #0x10. */
void step_sub_r0_0_0x10(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x0010);
    mcu.pc = 0x3065;
}

/* 0x3065 BEQ 15 -> 0x3076. */
void step_beq_15_0x3076(void)
{
    beq(0x3076, 0x3067);
}

/* 0x3067 SUB @r0+0 #0x0e. */
void step_sub_r0_0_0x0e(void)
{
    sub16_mem_nowrite(ind_addr(0, 0), 0x000e);
    mcu.pc = 0x306b;
}

/* 0x306B BNE 120 -> 0x30e5. */
void step_bne_120_0x30e5(void)
{
    bne(0x30e5, 0x306d);
}

/* 0x306D MOVG2 @r0+-2 r1. */
void step_movg2_r0_2_r1_d(void)
{
    load16(mcu.r[1], ind_addr(0, -2));
    mcu.pc = 0x3070;
}

/* 0x3070 movs r1 @(br,$3e). */
void step_movs_r1_br_3e_b(void)
{
    movs_br(mcu.r[1], 0x3e);
    mcu.pc = 0x3072;
}

/* 0x3072 movl r5 @(br,$32). */
void step_movl_r5_br_32_b(void)
{
    movl_br(mcu.r[5], 0x32);
    mcu.pc = 0x3074;
}

/* 0x3074 BRA 7 -> 0x307d. */
void step_bra_7_0x307d(void)
{
    mcu.pc = 0x307d;
}

/* 0x3076 MOVG2 @r0+-2 r1. */
void step_movg2_r0_2_r1_e(void)
{
    load16(mcu.r[1], ind_addr(0, -2));
    mcu.pc = 0x3079;
}

/* 0x3079 movs r1 @(br,$3e). */
void step_movs_r1_br_3e_c(void)
{
    movs_br(mcu.r[1], 0x3e);
    mcu.pc = 0x307b;
}

/* 0x307B movl r5 @(br,$34). */
void step_movl_r5_br_34_b(void)
{
    movl_br(mcu.r[5], 0x34);
    mcu.pc = 0x307d;
}

/* 0x307D movlw r5 @(br,$3a). */
void step_movlw_r5_br_3a_d(void)
{
    movlw_br(mcu.r[5], 0x3a);
    mcu.pc = 0x307f;
}

/* 0x307F BEQ 16 -> 0x3091. */
void step_beq_16_0x3091(void)
{
    beq(0x3091, 0x3081);
}

/* 0x3081 ADD r5 r5. */
void step_add_r5_r5_d(void)
{
    add16_reg(mcu.r[5], mcu.r[5]);
    mcu.pc = 0x3083;
}

/* 0x3083 SWAP r5. */
void step_swap_r5_c(void)
{
    swap16(mcu.r[5]);
    mcu.pc = 0x3085;
}

/* 0x3085 cmp r5,b #0xff. */
void step_cmp_r5_b_0xff_b(void)
{
    cmp8_reg_imm(mcu.r[5], 0xff);
    mcu.pc = 0x3087;
}

/* 0x3087 BNE 2 -> 0x308b. */
void step_bne_2_0x308b(void)
{
    bne(0x308b, 0x3089);
}

/* 0x3089 move r5 #0xfe. */
void step_move_r5_0xfe_b(void)
{
    move8(mcu.r[5], 0xfe);
    mcu.pc = 0x308b;
}

/* 0x308B MOVG3 r5 -> @r1+0xad0e. */
void step_movg3_r5_r1_0xad0e_b(void)
{
    store8(ind_addr(1, 0xad0e), mcu.r[5]);
    mcu.pc = 0x308f;
}

/* 0x308F BRA 84 -> 0x30e5. */
void step_bra_84_0x30e5(void)
{
    mcu.pc = 0x30e5;
}

/* 0x3091 CLR @r1+0xad0e. */
void step_clr_r1_0xad0e(void)
{
    clr8_mem(ind_addr(1, 0xad0e));
    mcu.pc = 0x3095;
}

/* 0x3095 MOVG #0x16 -> @r0+0. */
void step_movg_0x16_r0_0(void)
{
    store16(ind_addr(0, 0), 0x0016);
    mcu.pc = 0x3099;
}

/* 0x3099 SUB @r1+0xd0a8 #0xff. */
void step_sub_r1_0xd0a8_0xff(void)
{
    sub8_mem_nowrite(ind_addr(1, 0xd0a8), 0xff);
    mcu.pc = 0x309e;
}

/* 0x309E BNE 13 -> 0x30ad. */
void step_bne_13_0x30ad(void)
{
    bne(0x30ad, 0x30a0);
}

/* 0x30A0 SUB @r1+0xd0c4 #0xff. */
void step_sub_r1_0xd0c4_0xff(void)
{
    sub8_mem_nowrite(ind_addr(1, 0xd0c4), 0xff);
    mcu.pc = 0x30a5;
}

/* 0x30A5 BEQ 32 -> 0x30c7. */
void step_beq_32_0x30c7(void)
{
    beq(0x30c7, 0x30a7);
}

/* 0x30A7 MOVG2 @r1+0xd0c4 r3. */
void step_movg2_r1_0xd0c4_r3(void)
{
    load8(mcu.r[3], ind_addr(1, 0xd0c4));
    mcu.pc = 0x30ab;
}

/* 0x30AB BRA 4 -> 0x30b1. */
void step_bra_4_0x30b1(void)
{
    mcu.pc = 0x30b1;
}

/* 0x30AD MOVG2 @r1+0xd0a8 r3. */
void step_movg2_r1_0xd0a8_r3(void)
{
    load8(mcu.r[3], ind_addr(1, 0xd0a8));
    mcu.pc = 0x30b1;
}

/* 0x30B1 EXTU r3. */
void step_extu_r3(void)
{
    extu8(mcu.r[3]);
    mcu.pc = 0x30b3;
}

/* 0x30B3 MOVG #0xff -> @r1+0xd0a8. */
void step_movg_0xff_r1_0xd0a8(void)
{
    store8(ind_addr(1, 0xd0a8), 0xff);
    mcu.pc = 0x30b8;
}

/* 0x30B8 MOVG #0xff -> @r1+0xd0c4. */
void step_movg_0xff_r1_0xd0c4(void)
{
    store8(ind_addr(1, 0xd0c4), 0xff);
    mcu.pc = 0x30bd;
}

/* 0x30BD MOVG #0xff -> @r3+0xd0a8. */
void step_movg_0xff_r3_0xd0a8(void)
{
    store8(ind_addr(3, 0xd0a8), 0xff);
    mcu.pc = 0x30c2;
}

/* 0x30C2 MOVG #0xff -> @r3+0xd0c4. */
void step_movg_0xff_r3_0xd0c4(void)
{
    store8(ind_addr(3, 0xd0c4), 0xff);
    mcu.pc = 0x30c7;
}

/* 0x30C7 movs r1 @(br,$3e). */
void step_movs_r1_br_3e_d(void)
{
    movs_br(mcu.r[1], 0x3e);
    mcu.pc = 0x30c9;
}

/* 0x30C9 MOVG #0x00b5 -> (br,$18). */
void step_movg_0x00b5_br_18(void)
{
    store16(br_addr(0x18), 0x00b5);
    mcu.pc = 0x30ce;
}

/* 0x30CE MOVG3 r1 -> (dp,0xa1f6). */
void step_movg3_r1_dp_0xa1f6(void)
{
    store16(dp_addr(0xa1f6), mcu.r[1]);
    mcu.pc = 0x30d2;
}

/* 0x30D2 BCLR_ANDC #0xf8ff r0. */
void step_bclr_andc_0xf8ff_r0_g(void)
{
    bclr_andc_cr0(0xf8ff);
    mcu.pc = 0x30d6;
}

/* 0x30D6 move r0 #0x01. */
void step_move_r0_0x01(void)
{
    move8(mcu.r[0], 0x01);
    mcu.pc = 0x30d8;
}

/* 0x30D8 move r1 #0x01. */
void step_move_r1_0x01(void)
{
    move8(mcu.r[1], 0x01);
    mcu.pc = 0x30da;
}

/* 0x30DA trapa #0x12. */
void step_trapa_0x12(void)
{
    trapa(0x12);
    mcu.pc = 0x30dc;
}

/* 0x30DC ADDQ #2 r7. */
void step_addq_2_r7(void)
{
    addq16(mcu.r[7], 2);
    mcu.pc = 0x30de;
}

/* 0x30DE MOVG2 (dp,0xd178) r1. */
void step_movg2_dp_0xd178_r1(void)
{
    load16(mcu.r[1], dp_addr(0xd178));
    mcu.pc = 0x30e2;
}

/* 0x30E2 BRA 0x27ef -> 0x58d4. */
void step_bra_0x27ef_0x58d4(void)
{
    mcu.pc = 0x58d4;
}

/* 0x30E5 BCLR_ANDC #0xf8ff r0. */
void step_bclr_andc_0xf8ff_r0_h(void)
{
    bclr_andc_cr0(0xf8ff);
    mcu.pc = 0x30e9;
}

/* 0x30E9 MOVG2 (dp,0xd178) r1. */
void step_movg2_dp_0xd178_r1_b(void)
{
    load16(mcu.r[1], dp_addr(0xd178));
    mcu.pc = 0x30ed;
}

/* 0x30ED ADDQ #2 r7. */
void step_addq_2_r7_b(void)
{
    addq16(mcu.r[7], 2);
    mcu.pc = 0x30ef;
}

/* 0x30EF BRA 0x27e2 -> 0x58d4. */
void step_bra_0x27e2_0x58d4(void)
{
    mcu.pc = 0x58d4;
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void voice_param_fill(void)
{
    MK2CPP_HandRegister(0x00002e82u, &step_rts);
    MK2CPP_HandRegister(0x00002e83u, &step_bsr_3_0x2e82);
    MK2CPP_HandRegister(0x00002e85u, &step_bset_orc_0x0700_r0);
    MK2CPP_HandRegister(0x00002e89u, &step_movg_0x00ff_r0_26);
    MK2CPP_HandRegister(0x00002e8eu, &step_sub_r0_0_0x000e);
    MK2CPP_HandRegister(0x00002e93u, &step_bcc_0x01cb_0x3061);
    MK2CPP_HandRegister(0x00002e96u, &step_sub_r0_0_0x0000);
    MK2CPP_HandRegister(0x00002e9bu, &step_beq_0x0076_0x2f14);
    MK2CPP_HandRegister(0x00002e9eu, &step_movg2_r0_2_r1);
    MK2CPP_HandRegister(0x00002ea1u, &step_movs_r1_br_3e);
    MK2CPP_HandRegister(0x00002ea3u, &step_sub_r0_26_0xff00);
    MK2CPP_HandRegister(0x00002ea8u, &step_beq_16_0x2eba);
    MK2CPP_HandRegister(0x00002eaau, &step_movg_0xff00_br_16);
    MK2CPP_HandRegister(0x00002eafu, &step_movl_r5_br_32);
    MK2CPP_HandRegister(0x00002eb1u, &step_movlw_r5_br_3a);
    MK2CPP_HandRegister(0x00002eb3u, &step_add_r5_r5);
    MK2CPP_HandRegister(0x00002eb5u, &step_movg3_r5_r0_24);
    MK2CPP_HandRegister(0x00002eb8u, &step_bra_11_0x2ec5);
    MK2CPP_HandRegister(0x00002ebau, &step_movg2_r0_24_r5);
    MK2CPP_HandRegister(0x00002ebdu, &step_shlr_r5);
    MK2CPP_HandRegister(0x00002ebfu, &step_movsw_r5_br_32);
    MK2CPP_HandRegister(0x00002ec1u, &step_bsr_65_0x2e82);
    MK2CPP_HandRegister(0x00002ec3u, &step_movsw_r5_br_32_b);
    MK2CPP_HandRegister(0x00002ec5u, &step_sub_r0_30_0xff00);
    MK2CPP_HandRegister(0x00002ecau, &step_beq_16_0x2edc);
    MK2CPP_HandRegister(0x00002eccu, &step_movg_0xff00_br_18);
    MK2CPP_HandRegister(0x00002ed1u, &step_movl_r5_br_34);
    MK2CPP_HandRegister(0x00002ed3u, &step_movlw_r5_br_3a_b);
    MK2CPP_HandRegister(0x00002ed5u, &step_add_r5_r5_b);
    MK2CPP_HandRegister(0x00002ed7u, &step_movg3_r5_r0_28);
    MK2CPP_HandRegister(0x00002edau, &step_bra_11_0x2ee7);
    MK2CPP_HandRegister(0x00002edcu, &step_movg2_r0_28_r5);
    MK2CPP_HandRegister(0x00002edfu, &step_shlr_r5_b);
    MK2CPP_HandRegister(0x00002ee1u, &step_movsw_r5_br_34);
    MK2CPP_HandRegister(0x00002ee3u, &step_bsr_99_0x2e82);
    MK2CPP_HandRegister(0x00002ee5u, &step_movsw_r5_br_34_b);
    MK2CPP_HandRegister(0x00002ee7u, &step_movg2_r0_28_r5_b);
    MK2CPP_HandRegister(0x00002eeau, &step_swap_r5);
    MK2CPP_HandRegister(0x00002eecu, &step_cmp_r5_b_0xff);
    MK2CPP_HandRegister(0x00002eeeu, &step_bne_2_0x2ef2);
    MK2CPP_HandRegister(0x00002ef0u, &step_move_r5_0xfe);
    MK2CPP_HandRegister(0x00002ef2u, &step_movg3_r5_r1_0xad0e);
    MK2CPP_HandRegister(0x00002ef6u, &step_sub_r0_38_0xff00);
    MK2CPP_HandRegister(0x00002efbu, &step_beq_16_0x2f0d);
    MK2CPP_HandRegister(0x00002efdu, &step_movg_0xff00_br_1a);
    MK2CPP_HandRegister(0x00002f02u, &step_movl_r5_br_36);
    MK2CPP_HandRegister(0x00002f04u, &step_movlw_r5_br_3a_c);
    MK2CPP_HandRegister(0x00002f06u, &step_add_r5_r5_c);
    MK2CPP_HandRegister(0x00002f08u, &step_movg3_r5_r0_36);
    MK2CPP_HandRegister(0x00002f0bu, &step_bra_7_0x2f14);
    MK2CPP_HandRegister(0x00002f0du, &step_movg2_r0_36_r5);
    MK2CPP_HandRegister(0x00002f10u, &step_shlr_r5_c);
    MK2CPP_HandRegister(0x00002f12u, &step_movsw_r5_br_36);
    MK2CPP_HandRegister(0x00002f14u, &step_movg2_r0_2_r1_b);
    MK2CPP_HandRegister(0x00002f17u, &step_tst_r1_0xacf2);
    MK2CPP_HandRegister(0x00002f1bu, &step_beq_0x00d0_0x2fee);
    MK2CPP_HandRegister(0x00002f1eu, &step_clr_r1_0xacf2);
    MK2CPP_HandRegister(0x00002f22u, &step_sub_r0_0_0x000c);
    MK2CPP_HandRegister(0x00002f27u, &step_bcc_0x00c4_0x2fee);
    MK2CPP_HandRegister(0x00002f2au, &step_sub_r0_0_0x0000_b);
    MK2CPP_HandRegister(0x00002f2fu, &step_bne_44_0x2f5d);
    MK2CPP_HandRegister(0x00002f31u, &step_tst_r0_16);
    MK2CPP_HandRegister(0x00002f34u, &step_bne_18_0x2f48);
    MK2CPP_HandRegister(0x00002f36u, &step_movg_0x0002_r0_0);
    MK2CPP_HandRegister(0x00002f3bu, &step_movg_0x0002_r0_2);
    MK2CPP_HandRegister(0x00002f40u, &step_movg_0x0002_r0_4);
    MK2CPP_HandRegister(0x00002f45u, &step_bra_0x00a6_0x2fee);
    MK2CPP_HandRegister(0x00002f48u, &step_movg_0x0016_r0_0);
    MK2CPP_HandRegister(0x00002f4du, &step_movg_0x0016_r0_2);
    MK2CPP_HandRegister(0x00002f52u, &step_movg_0x0016_r0_4);
    MK2CPP_HandRegister(0x00002f57u, &step_movg2_r0_2_r1_c);
    MK2CPP_HandRegister(0x00002f5au, &step_bra_0x0134_0x3091);
    MK2CPP_HandRegister(0x00002f5du, &step_movg_0x000c_r0_0);
    MK2CPP_HandRegister(0x00002f62u, &step_movg_0x000c_r0_2);
    MK2CPP_HandRegister(0x00002f67u, &step_movg_0x000c_r0_4);
    MK2CPP_HandRegister(0x00002f6cu, &step_movg2_r0_28_r5_c);
    MK2CPP_HandRegister(0x00002f6fu, &step_swap_r5_b);
    MK2CPP_HandRegister(0x00002f71u, &step_movg3_r5_r0_96);
    MK2CPP_HandRegister(0x00002f74u, &step_clr_r0_97);
    MK2CPP_HandRegister(0x00002f77u, &step_clr_r0_8);
    MK2CPP_HandRegister(0x00002f7au, &step_clr_r0_18);
    MK2CPP_HandRegister(0x00002f7du, &step_movg2_r0_83_r5);
    MK2CPP_HandRegister(0x00002f80u, &step_movg3_r5_r0_79);
    MK2CPP_HandRegister(0x00002f83u, &step_movg2_r0_4_r5);
    MK2CPP_HandRegister(0x00002f86u, &step_movg3_r5_r0_8);
    MK2CPP_HandRegister(0x00002f89u, &step_clr_r0_10);
    MK2CPP_HandRegister(0x00002f8cu, &step_clr_r0_20);
    MK2CPP_HandRegister(0x00002f8fu, &step_movg2_r0_78_r5);
    MK2CPP_HandRegister(0x00002f92u, &step_movg3_r5_r0_74);
    MK2CPP_HandRegister(0x00002f95u, &step_movg2_r0_32_r5);
    MK2CPP_HandRegister(0x00002f98u, &step_movg3_r5_r0_84);
    MK2CPP_HandRegister(0x00002f9bu, &step_movg2_r0_94_r5);
    MK2CPP_HandRegister(0x00002f9eu, &step_movg3_r5_r0_86);
    MK2CPP_HandRegister(0x00002fa1u, &step_movg2_r0_44_r4);
    MK2CPP_HandRegister(0x00002fa4u, &step_movg2_r0_68_r5);
    MK2CPP_HandRegister(0x00002fa7u, &step_movg3_r4_r0_106);
    MK2CPP_HandRegister(0x00002faau, &step_movg3_r5_r0_112);
    MK2CPP_HandRegister(0x00002fadu, &step_movg2_r0_111_r4);
    MK2CPP_HandRegister(0x00002fb0u, &step_movg2_r0_122_r5);
    MK2CPP_HandRegister(0x00002fb3u, &step_movg3_r4_r0_107);
    MK2CPP_HandRegister(0x00002fb6u, &step_movg3_r5_r0_114);
    MK2CPP_HandRegister(0x00002fb9u, &step_cmp_r0_44_r4);
    MK2CPP_HandRegister(0x00002fbcu, &step_bne_3_0x2fc1);
    MK2CPP_HandRegister(0x00002fbeu, &step_cmp_r0_68_r5);
    MK2CPP_HandRegister(0x00002fc1u, &step_bcc_6_0x2fc9);
    MK2CPP_HandRegister(0x00002fc3u, &step_movg_0x02_r0_3);
    MK2CPP_HandRegister(0x00002fc7u, &step_bra_4_0x2fcd);
    MK2CPP_HandRegister(0x00002fc9u, &step_movg_0x00_r0_3);
    MK2CPP_HandRegister(0x00002fcdu, &step_movg2_r0_0x0084_r5);
    MK2CPP_HandRegister(0x00002fd1u, &step_movg3_r5_r0_124);
    MK2CPP_HandRegister(0x00002fd4u, &step_clr_r0_12);
    MK2CPP_HandRegister(0x00002fd7u, &step_clr_r0_22);
    MK2CPP_HandRegister(0x00002fdau, &step_tst_r0_78);
    MK2CPP_HandRegister(0x00002fddu, &step_bne_5_0x2fe4);
    MK2CPP_HandRegister(0x00002fdfu, &step_movg_0xffff_r0_78);
    MK2CPP_HandRegister(0x00002fe4u, &step_tst_r0_112);
    MK2CPP_HandRegister(0x00002fe7u, &step_bne_5_0x2fee);
    MK2CPP_HandRegister(0x00002fe9u, &step_movg_0xffff_r0_112);
    MK2CPP_HandRegister(0x00002feeu, &step_bclr_andc_0xf8ff_r0);
    MK2CPP_HandRegister(0x00002ff2u, &step_nop);
    MK2CPP_HandRegister(0x00002ff3u, &step_nop_b);
    MK2CPP_HandRegister(0x00002ff4u, &step_nop_c);
    MK2CPP_HandRegister(0x00002ff5u, &step_nop_d);
    MK2CPP_HandRegister(0x00002ff6u, &step_bset_orc_0x0700_r0_b);
    MK2CPP_HandRegister(0x00002ffau, &step_sub_r0_0_0x000e_b);
    MK2CPP_HandRegister(0x00002fffu, &step_bcc_96_0x3061);
    MK2CPP_HandRegister(0x00003001u, &step_bsr16_0x37fe);
    MK2CPP_HandRegister(0x00003004u, &step_bclr_andc_0xf8ff_r0_b);
    MK2CPP_HandRegister(0x00003008u, &step_nop_e);
    MK2CPP_HandRegister(0x00003009u, &step_nop_f);
    MK2CPP_HandRegister(0x0000300au, &step_nop_g);
    MK2CPP_HandRegister(0x0000300bu, &step_nop_h);
    MK2CPP_HandRegister(0x0000300cu, &step_bset_orc_0x0700_r0_c);
    MK2CPP_HandRegister(0x00003010u, &step_sub_r0_0_0x000e_c);
    MK2CPP_HandRegister(0x00003015u, &step_bcc_74_0x3061);
    MK2CPP_HandRegister(0x00003017u, &step_bsr16_0x30f2);
    MK2CPP_HandRegister(0x0000301au, &step_bclr_andc_0xf8ff_r0_c);
    MK2CPP_HandRegister(0x0000301eu, &step_nop_i);
    MK2CPP_HandRegister(0x0000301fu, &step_nop_j);
    MK2CPP_HandRegister(0x00003020u, &step_nop_k);
    MK2CPP_HandRegister(0x00003021u, &step_nop_l);
    MK2CPP_HandRegister(0x00003022u, &step_bset_orc_0x0700_r0_d);
    MK2CPP_HandRegister(0x00003026u, &step_sub_r0_0_0x000e_d);
    MK2CPP_HandRegister(0x0000302bu, &step_bcc_52_0x3061);
    MK2CPP_HandRegister(0x0000302du, &step_bsr16_0x41bb);
    MK2CPP_HandRegister(0x00003030u, &step_bclr_andc_0xf8ff_r0_d);
    MK2CPP_HandRegister(0x00003034u, &step_nop_m);
    MK2CPP_HandRegister(0x00003035u, &step_nop_n);
    MK2CPP_HandRegister(0x00003036u, &step_nop_o);
    MK2CPP_HandRegister(0x00003037u, &step_nop_p);
    MK2CPP_HandRegister(0x00003038u, &step_bset_orc_0x0700_r0_e);
    MK2CPP_HandRegister(0x0000303cu, &step_sub_r0_0_0x000e_e);
    MK2CPP_HandRegister(0x00003041u, &step_bcc_30_0x3061);
    MK2CPP_HandRegister(0x00003043u, &step_bsr16_0x4d62);
    MK2CPP_HandRegister(0x00003046u, &step_bclr_andc_0xf8ff_r0_e);
    MK2CPP_HandRegister(0x0000304au, &step_nop_q);
    MK2CPP_HandRegister(0x0000304bu, &step_nop_r);
    MK2CPP_HandRegister(0x0000304cu, &step_nop_s);
    MK2CPP_HandRegister(0x0000304du, &step_nop_t);
    MK2CPP_HandRegister(0x0000304eu, &step_bset_orc_0x0700_r0_f);
    MK2CPP_HandRegister(0x00003052u, &step_sub_r0_0_0x000e_f);
    MK2CPP_HandRegister(0x00003057u, &step_bcc_8_0x3061);
    MK2CPP_HandRegister(0x00003059u, &step_bsr16_0x3465);
    MK2CPP_HandRegister(0x0000305cu, &step_bclr_andc_0xf8ff_r0_f);
    MK2CPP_HandRegister(0x00003060u, &step_rts_b);
    MK2CPP_HandRegister(0x00003061u, &step_sub_r0_0_0x10);
    MK2CPP_HandRegister(0x00003065u, &step_beq_15_0x3076);
    MK2CPP_HandRegister(0x00003067u, &step_sub_r0_0_0x0e);
    MK2CPP_HandRegister(0x0000306bu, &step_bne_120_0x30e5);
    MK2CPP_HandRegister(0x0000306du, &step_movg2_r0_2_r1_d);
    MK2CPP_HandRegister(0x00003070u, &step_movs_r1_br_3e_b);
    MK2CPP_HandRegister(0x00003072u, &step_movl_r5_br_32_b);
    MK2CPP_HandRegister(0x00003074u, &step_bra_7_0x307d);
    MK2CPP_HandRegister(0x00003076u, &step_movg2_r0_2_r1_e);
    MK2CPP_HandRegister(0x00003079u, &step_movs_r1_br_3e_c);
    MK2CPP_HandRegister(0x0000307bu, &step_movl_r5_br_34_b);
    MK2CPP_HandRegister(0x0000307du, &step_movlw_r5_br_3a_d);
    MK2CPP_HandRegister(0x0000307fu, &step_beq_16_0x3091);
    MK2CPP_HandRegister(0x00003081u, &step_add_r5_r5_d);
    MK2CPP_HandRegister(0x00003083u, &step_swap_r5_c);
    MK2CPP_HandRegister(0x00003085u, &step_cmp_r5_b_0xff_b);
    MK2CPP_HandRegister(0x00003087u, &step_bne_2_0x308b);
    MK2CPP_HandRegister(0x00003089u, &step_move_r5_0xfe_b);
    MK2CPP_HandRegister(0x0000308bu, &step_movg3_r5_r1_0xad0e_b);
    MK2CPP_HandRegister(0x0000308fu, &step_bra_84_0x30e5);
    MK2CPP_HandRegister(0x00003091u, &step_clr_r1_0xad0e);
    MK2CPP_HandRegister(0x00003095u, &step_movg_0x16_r0_0);
    MK2CPP_HandRegister(0x00003099u, &step_sub_r1_0xd0a8_0xff);
    MK2CPP_HandRegister(0x0000309eu, &step_bne_13_0x30ad);
    MK2CPP_HandRegister(0x000030a0u, &step_sub_r1_0xd0c4_0xff);
    MK2CPP_HandRegister(0x000030a5u, &step_beq_32_0x30c7);
    MK2CPP_HandRegister(0x000030a7u, &step_movg2_r1_0xd0c4_r3);
    MK2CPP_HandRegister(0x000030abu, &step_bra_4_0x30b1);
    MK2CPP_HandRegister(0x000030adu, &step_movg2_r1_0xd0a8_r3);
    MK2CPP_HandRegister(0x000030b1u, &step_extu_r3);
    MK2CPP_HandRegister(0x000030b3u, &step_movg_0xff_r1_0xd0a8);
    MK2CPP_HandRegister(0x000030b8u, &step_movg_0xff_r1_0xd0c4);
    MK2CPP_HandRegister(0x000030bdu, &step_movg_0xff_r3_0xd0a8);
    MK2CPP_HandRegister(0x000030c2u, &step_movg_0xff_r3_0xd0c4);
    MK2CPP_HandRegister(0x000030c7u, &step_movs_r1_br_3e_d);
    MK2CPP_HandRegister(0x000030c9u, &step_movg_0x00b5_br_18);
    MK2CPP_HandRegister(0x000030ceu, &step_movg3_r1_dp_0xa1f6);
    MK2CPP_HandRegister(0x000030d2u, &step_bclr_andc_0xf8ff_r0_g);
    MK2CPP_HandRegister(0x000030d6u, &step_move_r0_0x01);
    MK2CPP_HandRegister(0x000030d8u, &step_move_r1_0x01);
    MK2CPP_HandRegister(0x000030dau, &step_trapa_0x12);
    MK2CPP_HandRegister(0x000030dcu, &step_addq_2_r7);
    MK2CPP_HandRegister(0x000030deu, &step_movg2_dp_0xd178_r1);
    MK2CPP_HandRegister(0x000030e2u, &step_bra_0x27ef_0x58d4);
    MK2CPP_HandRegister(0x000030e5u, &step_bclr_andc_0xf8ff_r0_h);
    MK2CPP_HandRegister(0x000030e9u, &step_movg2_dp_0xd178_r1_b);
    MK2CPP_HandRegister(0x000030edu, &step_addq_2_r7_b);
    MK2CPP_HandRegister(0x000030efu, &step_bra_0x27e2_0x58d4);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct VoiceParamSelfRegister
{
    VoiceParamSelfRegister() { hand_register_module(&voice_param_fill); }
};

VoiceParamSelfRegister g_voice_param_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
