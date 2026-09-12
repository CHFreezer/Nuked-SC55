/*
 * HAND pcm/ts_scan — time-slot scan (class C15, rom1 cp=0,
 * flat 0x5869..0x5995).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13, named operations); addresses and registration unchanged. One
 * L0 hand entry per instruction PC; see pcm_irq_service.cpp for the L0
 * rationale (docs/09_m4_integration.md 4.1).
 *
 * Semantics: loops over voices P(v) looking for an active slot, writes ad2a
 * and clears P-0x26; TRAPA #0x1b; reached from 0x56c0 and linked with the C9 /
 * coeff chain (C16/C17).
 * Evidence: out/m4/18_closure_gap.md 1.3 C15 / 4.2 row 11; docs/07:149;
 * voice_bounds_inventory:38-39. Confidence: C (ROM bytes + dasm).
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

/* TRAPA #imm copied verbatim: only the 0x1x encoding reaches the trap. */
void trapa(uint8_t opcode)
{
    if ((opcode & 0xf0) == 0x10)
        MCU_Interrupt_TRAPA(opcode & 0x0f);
    else
        MCU_ErrorTrap();
}

/* EXTU rN: zero-extend the low byte, N=0, Z from the result, V=0, C=0. */
void extu8(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    reg = (uint16_t)data;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(data == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* CLR rN (word): rN=0, N=0 Z=1 V=0 C=0. */
void clr_reg(uint16_t &reg)
{
    reg = (uint16_t)0;
    flags_clr();
}

/* CLR @addr (byte): memory only, N=0 Z=1 V=0 C=0. */
void clr8_mem(uint32_t addr)
{
    MCU_Write(addr, (uint8_t)(0));
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* ADD rS,rD (word). */
void add16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, 0, 1);
}

/* SUB #imm,@addr (byte): flags only, the memory value is not written back. */
void sub8_mem_imm(uint32_t addr, uint8_t imm)
{
    uint32_t t1 = (uint32_t)MCU_Read(addr);
    uint32_t t2 = (uint32_t)imm;
    MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
}

/* XCH rA,rB (word): exchange, no flags. */
void xch16(uint16_t &a, uint16_t &b)
{
    uint32_t r1 = (uint32_t)a;
    uint32_t r2 = (uint32_t)b;
    a = (uint16_t)r2;
    b = (uint16_t)r1;
}

/* BSET_ORC #0x0700,r0: raise IML to 7 through the CCR window + ex_ignore. */
void bset_orc_iml7(void)
{
    uint32_t data = 0x0700;
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val |= data;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

/* BCLR_ANDC #0xf8ff,r0: lower IML to 0 through the CCR window + ex_ignore. */
void bclr_andc_iml0(void)
{
    uint32_t data = 0xf8ff;
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val &= data;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

/* ---- conditional branches (disp8 relative to the next PC) ----------------- */

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

void bcc(uint16_t taken, uint16_t fall)
{
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    mcu.pc = (C == 0) ? taken : fall;
}

/* ======================================================================
 * cp0 0x5869..0x5995, 97 PC: C15 time-slot scan.
 * ====================================================================== */

/* 0x5869 trapa #0x1b. */
void step_trapa(void)
{
    mcu.pc = 0x586b;
    trapa(0x1b);
}

/* 0x586B extu r0. */
void step_extu_r0(void)
{
    mcu.pc = 0x586d;
    extu8(mcu.r[0]);
}

/* 0x586D movg3 r0,(dp,0xad2a). */
void step_store_dp_ad2a_r0(void)
{
    mcu.pc = 0x5871;
    store16(dp_addr(0xad2a), mcu.r[0]);
}

/* 0x5871 tst (dp,0xd19c). */
void step_tst_dp_d19c(void)
{
    mcu.pc = 0x5875;
    tst16((uint16_t)MCU_Read16(dp_addr(0xd19c)));
}

/* 0x5875 beq 3 -> 0x587a. */
void step_beq_587a(void)
{
    beq(0x587a, 0x5877);
}

/* 0x5877 bsr16 -> 0x5dc3. */
void step_call_5dc3(void)
{
    call(0x587a, 0x5dc3);
}

/* 0x587A tst (dp,0xd19e). */
void step_tst_dp_d19e(void)
{
    mcu.pc = 0x587e;
    tst16((uint16_t)MCU_Read16(dp_addr(0xd19e)));
}

/* 0x587E beq 3 -> 0x5883. */
void step_beq_5883(void)
{
    beq(0x5883, 0x5880);
}

/* 0x5880 bsr16 -> 0x618c. */
void step_call_618c(void)
{
    call(0x5883, 0x618c);
}

/* 0x5883 ldc #0x00,ep. */
void step_ldc_ep0(void)
{
    mcu.pc = 0x5886;
    ldc_cr(4, 0x00);
}

/* 0x5886 movi r1,#0x001b. */
void step_movi_r1_001b(void)
{
    mcu.pc = 0x5889;
    movi16(mcu.r[1], 0x001b);
}

/* 0x5889 mov.w r1,r2. */
void step_mov_r1_r2(void)
{
    mcu.pc = 0x588b;
    mov16(mcu.r[2], (uint32_t)mcu.r[1]);
}

/* 0x588B add r2,r2. */
void step_add_r2_r2_a(void)
{
    mcu.pc = 0x588d;
    add16_reg(mcu.r[2], mcu.r[2]);
}

/* 0x588D mov.w @r2+0x64d6,r0. */
void step_load_ptr_r2_r0_a(void)
{
    mcu.pc = 0x5891;
    load16(mcu.r[0], ind_addr(2, 0x64d6));
}

/* 0x5895 bcc 61 -> 0x58d4. */
void step_bcc_58d4_a(void)
{
    bcc(0x58d4, 0x5897);
}

/* 0x5897 tst.b @r0-26. */
void step_tst_voice_active(void)
{
    mcu.pc = 0x589a;
    tst8(MCU_Read(ind_addr(0, -26)));
}

/* 0x589A bne 56 -> 0x58d4. */
void step_bne_58d4(void)
{
    bne(0x58d4, 0x589c);
}

/* 0x589C bset_orc #0x0700,r0. */
void step_raise_iml_a(void)
{
    mcu.pc = 0x58a0;
    bset_orc_iml7();
}

/* 0x58A0 movg3 r1,(dp,0xd178). */
void step_store_dp_d178_r1_a(void)
{
    mcu.pc = 0x58a4;
    store16(dp_addr(0xd178), mcu.r[1]);
}

/* 0x58A4 clr r3. */
void step_clr_r3_a(void)
{
    mcu.pc = 0x58a6;
    clr_reg(mcu.r[3]);
}

/* 0x58A6 sub #0xff,@r1+0xd0a8. */
void step_sub_flags_d0a8(void)
{
    mcu.pc = 0x58ab;
    sub8_mem_imm(ind_addr(1, 0xd0a8), 0xff);
}

/* 0x58AB bne 21 -> 0x58c2. */
void step_bne_58c2(void)
{
    bne(0x58c2, 0x58ad);
}

/* 0x58AD sub #0xff,@r1+0xd0c4. */
void step_sub_flags_d0c4(void)
{
    mcu.pc = 0x58b2;
    sub8_mem_imm(ind_addr(1, 0xd0c4), 0xff);
}

/* 0x58B2 beq 55 -> 0x58eb. */
void step_beq_58eb(void)
{
    beq(0x58eb, 0x58b4);
}

/* 0x58B4 mov.b @r1+0xd0c4,r3. */
void step_load_byte_r1_d0c4(void)
{
    mcu.pc = 0x58b8;
    load8(mcu.r[3], ind_addr(1, 0xd0c4));
}

/* 0x58B8 mov.w r3,r2. */
void step_mov_r3_r2(void)
{
    mcu.pc = 0x58ba;
    mov16(mcu.r[2], (uint32_t)mcu.r[3]);
}

/* 0x58BA add r2,r2. */
void step_add_r2_r2_b(void)
{
    mcu.pc = 0x58bc;
    add16_reg(mcu.r[2], mcu.r[2]);
}

/* 0x58BC mov.w @r2+0x64d6,r2. */
void step_load_ptr_r2_r2(void)
{
    mcu.pc = 0x58c0;
    load16(mcu.r[2], ind_addr(2, 0x64d6));
}

/* 0x58C0 bra 106 -> 0x592c. */
void step_bra_592c_a(void)
{
    mcu.pc = 0x592c;
}

/* 0x58C2 mov.b @r1+0xd0a8,r3. */
void step_load_byte_r1_d0a8(void)
{
    mcu.pc = 0x58c6;
    load8(mcu.r[3], ind_addr(1, 0xd0a8));
}

/* 0x58C6 xch r3,r1. */
void step_xch_r3_r1(void)
{
    mcu.pc = 0x58c8;
    xch16(mcu.r[3], mcu.r[1]);
}

/* 0x58C8 mov.w r0,r2. */
void step_mov_r0_r2(void)
{
    mcu.pc = 0x58ca;
    mov16(mcu.r[2], (uint32_t)mcu.r[0]);
}

/* 0x58CA mov.w r1,r0. */
void step_mov_r1_r0(void)
{
    mcu.pc = 0x58cc;
    mov16(mcu.r[0], (uint32_t)mcu.r[1]);
}

/* 0x58CC add r0,r0. */
void step_add_r0_r0(void)
{
    mcu.pc = 0x58ce;
    add16_reg(mcu.r[0], mcu.r[0]);
}

/* 0x58CE mov.w @r0+0x64d6,r0. */
void step_load_ptr_r0_r0(void)
{
    mcu.pc = 0x58d2;
    load16(mcu.r[0], ind_addr(0, 0x64d6));
}

/* 0x58D2 bra 88 -> 0x592c. */
void step_bra_592c_b(void)
{
    mcu.pc = 0x592c;
}

/* 0x58D4 cntjmp r1,-78 -> 0x5889. */
void step_cntjmp_5889(void)
{
    cntjmp(mcu.r[1], 0x5889, 0x58d7);
}

/* 0x58D7 movi r1,#0x001b. */
void step_movi_r1_001b_b(void)
{
    mcu.pc = 0x58da;
    movi16(mcu.r[1], 0x001b);
}

/* 0x58DA mov.w r1,r2. */
void step_mov_r1_r2_b(void)
{
    mcu.pc = 0x58dc;
    mov16(mcu.r[2], (uint32_t)mcu.r[1]);
}

/* 0x58DC add r2,r2. */
void step_add_r2_r2_c(void)
{
    mcu.pc = 0x58de;
    add16_reg(mcu.r[2], mcu.r[2]);
}

/* 0x58DE mov.w @r2+0x64d6,r0. */
void step_load_ptr_r2_r0_b(void)
{
    mcu.pc = 0x58e2;
    load16(mcu.r[0], ind_addr(2, 0x64d6));
}

/* 0x58E2 clr.b @r0-26. */
void step_clr_voice_active(void)
{
    mcu.pc = 0x58e5;
    clr8_mem(ind_addr(0, -26));
}

/* 0x58E5 cntjmp r1,-14 -> 0x58da. */
void step_cntjmp_58da(void)
{
    cntjmp(mcu.r[1], 0x58da, 0x58e8);
}

/* 0x58E8 bra -> 0x56ba. */
void step_bra_56ba(void)
{
    mcu.pc = 0x56ba;
}

/* 0x58EB bclr_andc #0xf8ff,r0. */
void step_lower_iml_a(void)
{
    mcu.pc = 0x58ef;
    bclr_andc_iml0();
}

/* 0x58EF movg3 r0,(dp,0xd17a). */
void step_store_dp_d17a_r0_a(void)
{
    mcu.pc = 0x58f3;
    store16(dp_addr(0xd17a), mcu.r[0]);
}

/* 0x58F3 movg3 r1,@r0-2. */
void step_store_at_r0_m2_r1_a(void)
{
    mcu.pc = 0x58f6;
    store16(ind_addr(0, -2), mcu.r[1]);
}

/* 0x58F6 bset_orc #0x0700,r0. */
void step_raise_iml_b(void)
{
    mcu.pc = 0x58fa;
    bset_orc_iml7();
}

/* 0x58FA mov.w (dp,0xd17a),r0. */
void step_load_dp_d17a_r0_a(void)
{
    mcu.pc = 0x58fe;
    load16(mcu.r[0], dp_addr(0xd17a));
}

/* 0x58FE bsr16 -> 0x5998. */
void step_call_5998_a(void)
{
    call(0x5901, 0x5998);
}

/* 0x5905 nop. */
void step_nop_5905(void)
{
    mcu.pc = 0x5906;
}

/* 0x5906 nop. */
void step_nop_5906(void)
{
    mcu.pc = 0x5907;
}

/* 0x5907 nop. */
void step_nop_5907(void)
{
    mcu.pc = 0x5908;
}

/* 0x5908 nop. */
void step_nop_5908(void)
{
    mcu.pc = 0x5909;
}

/* 0x5909 bset_orc #0x0700,r0. */
void step_raise_iml_c(void)
{
    mcu.pc = 0x590d;
    bset_orc_iml7();
}

/* 0x590D mov.w (dp,0xd17a),r0. */
void step_load_dp_d17a_r0_b(void)
{
    mcu.pc = 0x5911;
    load16(mcu.r[0], dp_addr(0xd17a));
}

/* 0x5911 bsr16 -> 0x3709. */
void step_call_3709_a(void)
{
    call(0x5914, 0x3709);
}

/* 0x5914 bclr_andc #0xf8ff,r0. */
void step_lower_iml_b(void)
{
    mcu.pc = 0x5918;
    bclr_andc_iml0();
}

/* 0x5918 mov.w (dp,0xd17a),r0. */
void step_load_dp_d17a_r0_c(void)
{
    mcu.pc = 0x591c;
    load16(mcu.r[0], dp_addr(0xd17a));
}

/* 0x591C bsr16 -> 0x2e83. */
void step_call_2e83_a(void)
{
    call(0x591f, 0x2e83);
}

/* 0x591F mov.w (dp,0xd17a),r0. */
void step_load_dp_d17a_r0_d(void)
{
    mcu.pc = 0x5923;
    load16(mcu.r[0], dp_addr(0xd17a));
}

/* 0x5923 bsr16 -> 0x5671. */
void step_call_5671_a(void)
{
    call(0x5926, 0x5671);
}

/* 0x5926 mov.w (dp,0xd178),r1. */
void step_load_dp_d178_r1_a(void)
{
    mcu.pc = 0x592a;
    load16(mcu.r[1], dp_addr(0xd178));
}

/* 0x592A bra -88 -> 0x58d4. */
void step_bra_58d4(void)
{
    mcu.pc = 0x58d4;
}

/* 0x592C bclr_andc #0xf8ff,r0. */
void step_lower_iml_c(void)
{
    mcu.pc = 0x5930;
    bclr_andc_iml0();
}

/* 0x5930 movg3 r0,(dp,0xd17a). */
void step_store_dp_d17a_r0_b(void)
{
    mcu.pc = 0x5934;
    store16(dp_addr(0xd17a), mcu.r[0]);
}

/* 0x5934 movg3 r1,@r0-2. */
void step_store_at_r0_m2_r1_b(void)
{
    mcu.pc = 0x5937;
    store16(ind_addr(0, -2), mcu.r[1]);
}

/* 0x5937 movg3 r2,(dp,0xd17c). */
void step_store_dp_d17c_r2(void)
{
    mcu.pc = 0x593b;
    store16(dp_addr(0xd17c), mcu.r[2]);
}

/* 0x593B movg3 r3,@r2-2. */
void step_store_at_r2_m2_r3(void)
{
    mcu.pc = 0x593e;
    store16(ind_addr(2, -2), mcu.r[3]);
}

/* 0x593E bset_orc #0x0700,r0. */
void step_raise_iml_d(void)
{
    mcu.pc = 0x5942;
    bset_orc_iml7();
}

/* 0x5942 mov.w (dp,0xd17a),r0. */
void step_load_dp_d17a_r0_e(void)
{
    mcu.pc = 0x5946;
    load16(mcu.r[0], dp_addr(0xd17a));
}

/* 0x5946 bsr 80 -> 0x5998. */
void step_call_5998_b(void)
{
    call(0x5948, 0x5998);
}

/* 0x5948 mov.w (dp,0xd17c),r0. */
void step_load_dp_d17c_r0_a(void)
{
    mcu.pc = 0x594c;
    load16(mcu.r[0], dp_addr(0xd17c));
}

/* 0x594C mov.w (dp,0xd17a),r2. */
void step_load_dp_d17a_r2(void)
{
    mcu.pc = 0x5950;
    load16(mcu.r[2], dp_addr(0xd17a));
}

/* 0x5950 bsr16 -> 0x5d6e. */
void step_call_5d6e(void)
{
    call(0x5953, 0x5d6e);
}

/* 0x5953 bclr_andc #0xf8ff,r0. */
void step_lower_iml_d(void)
{
    mcu.pc = 0x5957;
    bclr_andc_iml0();
}

/* 0x5957 nop. */
void step_nop_5957(void)
{
    mcu.pc = 0x5958;
}

/* 0x5958 nop. */
void step_nop_5958(void)
{
    mcu.pc = 0x5959;
}

/* 0x5959 nop. */
void step_nop_5959(void)
{
    mcu.pc = 0x595a;
}

/* 0x595A nop. */
void step_nop_595a(void)
{
    mcu.pc = 0x595b;
}

/* 0x595B bset_orc #0x0700,r0. */
void step_raise_iml_e(void)
{
    mcu.pc = 0x595f;
    bset_orc_iml7();
}

/* 0x595F mov.w (dp,0xd17a),r0. */
void step_load_dp_d17a_r0_f(void)
{
    mcu.pc = 0x5963;
    load16(mcu.r[0], dp_addr(0xd17a));
}

/* 0x5963 bsr16 -> 0x3709. */
void step_call_3709_b(void)
{
    call(0x5966, 0x3709);
}

/* 0x5966 mov.w (dp,0xd17c),r0. */
void step_load_dp_d17c_r0_b(void)
{
    mcu.pc = 0x596a;
    load16(mcu.r[0], dp_addr(0xd17c));
}

/* 0x596A mov.w (dp,0xd17a),r2. */
void step_load_dp_d17a_r2_b(void)
{
    mcu.pc = 0x596e;
    load16(mcu.r[2], dp_addr(0xd17a));
}

/* 0x596E bsr16 -> 0x3ac8. */
void step_call_3ac8(void)
{
    call(0x5971, 0x3ac8);
}

/* 0x5971 bclr_andc #0xf8ff,r0. */
void step_lower_iml_e(void)
{
    mcu.pc = 0x5975;
    bclr_andc_iml0();
}

/* 0x5975 mov.w (dp,0xd17a),r0. */
void step_load_dp_d17a_r0_g(void)
{
    mcu.pc = 0x5979;
    load16(mcu.r[0], dp_addr(0xd17a));
}

/* 0x5979 bsr16 -> 0x2e83. */
void step_call_2e83_b(void)
{
    call(0x597c, 0x2e83);
}

/* 0x597C mov.w (dp,0xd17c),r0. */
void step_load_dp_d17c_r0_c(void)
{
    mcu.pc = 0x5980;
    load16(mcu.r[0], dp_addr(0xd17c));
}

/* 0x5980 bsr16 -> 0x2e83. */
void step_call_2e83_c(void)
{
    call(0x5983, 0x2e83);
}

/* 0x5983 mov.w (dp,0xd17a),r0. */
void step_load_dp_d17a_r0_h(void)
{
    mcu.pc = 0x5987;
    load16(mcu.r[0], dp_addr(0xd17a));
}

/* 0x5987 bsr16 -> 0x5671. */
void step_call_5671_b(void)
{
    call(0x598a, 0x5671);
}

/* 0x598A mov.w (dp,0xd17c),r0. */
void step_load_dp_d17c_r0_d(void)
{
    mcu.pc = 0x598e;
    load16(mcu.r[0], dp_addr(0xd17c));
}

/* 0x598E bsr16 -> 0x5671. */
void step_call_5671_c(void)
{
    call(0x5991, 0x5671);
}

/* 0x5991 mov.w (dp,0xd178),r1. */
void step_load_dp_d178_r1_b(void)
{
    mcu.pc = 0x5995;
    load16(mcu.r[1], dp_addr(0xd178));
}

/* 0x5995 bra -> 0x58d4. */
void step_bra_58d4_b(void)
{
    mcu.pc = 0x58d4;
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void ts_scan_fill(void)
{
    MK2CPP_HandRegister(0x00005869u, &step_trapa);
    MK2CPP_HandRegister(0x0000586bu, &step_extu_r0);
    MK2CPP_HandRegister(0x0000586du, &step_store_dp_ad2a_r0);
    MK2CPP_HandRegister(0x00005871u, &step_tst_dp_d19c);
    MK2CPP_HandRegister(0x00005875u, &step_beq_587a);
    MK2CPP_HandRegister(0x00005877u, &step_call_5dc3);
    MK2CPP_HandRegister(0x0000587au, &step_tst_dp_d19e);
    MK2CPP_HandRegister(0x0000587eu, &step_beq_5883);
    MK2CPP_HandRegister(0x00005880u, &step_call_618c);
    MK2CPP_HandRegister(0x00005883u, &step_ldc_ep0);
    MK2CPP_HandRegister(0x00005886u, &step_movi_r1_001b);
    MK2CPP_HandRegister(0x00005889u, &step_mov_r1_r2);
    MK2CPP_HandRegister(0x0000588bu, &step_add_r2_r2_a);
    MK2CPP_HandRegister(0x0000588du, &step_load_ptr_r2_r0_a);
    MK2CPP_HandRegister(0x00005895u, &step_bcc_58d4_a);
    MK2CPP_HandRegister(0x00005897u, &step_tst_voice_active);
    MK2CPP_HandRegister(0x0000589au, &step_bne_58d4);
    MK2CPP_HandRegister(0x0000589cu, &step_raise_iml_a);
    MK2CPP_HandRegister(0x000058a0u, &step_store_dp_d178_r1_a);
    MK2CPP_HandRegister(0x000058a4u, &step_clr_r3_a);
    MK2CPP_HandRegister(0x000058a6u, &step_sub_flags_d0a8);
    MK2CPP_HandRegister(0x000058abu, &step_bne_58c2);
    MK2CPP_HandRegister(0x000058adu, &step_sub_flags_d0c4);
    MK2CPP_HandRegister(0x000058b2u, &step_beq_58eb);
    MK2CPP_HandRegister(0x000058b4u, &step_load_byte_r1_d0c4);
    MK2CPP_HandRegister(0x000058b8u, &step_mov_r3_r2);
    MK2CPP_HandRegister(0x000058bau, &step_add_r2_r2_b);
    MK2CPP_HandRegister(0x000058bcu, &step_load_ptr_r2_r2);
    MK2CPP_HandRegister(0x000058c0u, &step_bra_592c_a);
    MK2CPP_HandRegister(0x000058c2u, &step_load_byte_r1_d0a8);
    MK2CPP_HandRegister(0x000058c6u, &step_xch_r3_r1);
    MK2CPP_HandRegister(0x000058c8u, &step_mov_r0_r2);
    MK2CPP_HandRegister(0x000058cau, &step_mov_r1_r0);
    MK2CPP_HandRegister(0x000058ccu, &step_add_r0_r0);
    MK2CPP_HandRegister(0x000058ceu, &step_load_ptr_r0_r0);
    MK2CPP_HandRegister(0x000058d2u, &step_bra_592c_b);
    MK2CPP_HandRegister(0x000058d4u, &step_cntjmp_5889);
    MK2CPP_HandRegister(0x000058d7u, &step_movi_r1_001b_b);
    MK2CPP_HandRegister(0x000058dau, &step_mov_r1_r2_b);
    MK2CPP_HandRegister(0x000058dcu, &step_add_r2_r2_c);
    MK2CPP_HandRegister(0x000058deu, &step_load_ptr_r2_r0_b);
    MK2CPP_HandRegister(0x000058e2u, &step_clr_voice_active);
    MK2CPP_HandRegister(0x000058e5u, &step_cntjmp_58da);
    MK2CPP_HandRegister(0x000058e8u, &step_bra_56ba);
    MK2CPP_HandRegister(0x000058ebu, &step_lower_iml_a);
    MK2CPP_HandRegister(0x000058efu, &step_store_dp_d17a_r0_a);
    MK2CPP_HandRegister(0x000058f3u, &step_store_at_r0_m2_r1_a);
    MK2CPP_HandRegister(0x000058f6u, &step_raise_iml_b);
    MK2CPP_HandRegister(0x000058fau, &step_load_dp_d17a_r0_a);
    MK2CPP_HandRegister(0x000058feu, &step_call_5998_a);
    MK2CPP_HandRegister(0x00005905u, &step_nop_5905);
    MK2CPP_HandRegister(0x00005906u, &step_nop_5906);
    MK2CPP_HandRegister(0x00005907u, &step_nop_5907);
    MK2CPP_HandRegister(0x00005908u, &step_nop_5908);
    MK2CPP_HandRegister(0x00005909u, &step_raise_iml_c);
    MK2CPP_HandRegister(0x0000590du, &step_load_dp_d17a_r0_b);
    MK2CPP_HandRegister(0x00005911u, &step_call_3709_a);
    MK2CPP_HandRegister(0x00005914u, &step_lower_iml_b);
    MK2CPP_HandRegister(0x00005918u, &step_load_dp_d17a_r0_c);
    MK2CPP_HandRegister(0x0000591cu, &step_call_2e83_a);
    MK2CPP_HandRegister(0x0000591fu, &step_load_dp_d17a_r0_d);
    MK2CPP_HandRegister(0x00005923u, &step_call_5671_a);
    MK2CPP_HandRegister(0x00005926u, &step_load_dp_d178_r1_a);
    MK2CPP_HandRegister(0x0000592au, &step_bra_58d4);
    MK2CPP_HandRegister(0x0000592cu, &step_lower_iml_c);
    MK2CPP_HandRegister(0x00005930u, &step_store_dp_d17a_r0_b);
    MK2CPP_HandRegister(0x00005934u, &step_store_at_r0_m2_r1_b);
    MK2CPP_HandRegister(0x00005937u, &step_store_dp_d17c_r2);
    MK2CPP_HandRegister(0x0000593bu, &step_store_at_r2_m2_r3);
    MK2CPP_HandRegister(0x0000593eu, &step_raise_iml_d);
    MK2CPP_HandRegister(0x00005942u, &step_load_dp_d17a_r0_e);
    MK2CPP_HandRegister(0x00005946u, &step_call_5998_b);
    MK2CPP_HandRegister(0x00005948u, &step_load_dp_d17c_r0_a);
    MK2CPP_HandRegister(0x0000594cu, &step_load_dp_d17a_r2);
    MK2CPP_HandRegister(0x00005950u, &step_call_5d6e);
    MK2CPP_HandRegister(0x00005953u, &step_lower_iml_d);
    MK2CPP_HandRegister(0x00005957u, &step_nop_5957);
    MK2CPP_HandRegister(0x00005958u, &step_nop_5958);
    MK2CPP_HandRegister(0x00005959u, &step_nop_5959);
    MK2CPP_HandRegister(0x0000595au, &step_nop_595a);
    MK2CPP_HandRegister(0x0000595bu, &step_raise_iml_e);
    MK2CPP_HandRegister(0x0000595fu, &step_load_dp_d17a_r0_f);
    MK2CPP_HandRegister(0x00005963u, &step_call_3709_b);
    MK2CPP_HandRegister(0x00005966u, &step_load_dp_d17c_r0_b);
    MK2CPP_HandRegister(0x0000596au, &step_load_dp_d17a_r2_b);
    MK2CPP_HandRegister(0x0000596eu, &step_call_3ac8);
    MK2CPP_HandRegister(0x00005971u, &step_lower_iml_e);
    MK2CPP_HandRegister(0x00005975u, &step_load_dp_d17a_r0_g);
    MK2CPP_HandRegister(0x00005979u, &step_call_2e83_b);
    MK2CPP_HandRegister(0x0000597cu, &step_load_dp_d17c_r0_c);
    MK2CPP_HandRegister(0x00005980u, &step_call_2e83_c);
    MK2CPP_HandRegister(0x00005983u, &step_load_dp_d17a_r0_h);
    MK2CPP_HandRegister(0x00005987u, &step_call_5671_b);
    MK2CPP_HandRegister(0x0000598au, &step_load_dp_d17c_r0_d);
    MK2CPP_HandRegister(0x0000598eu, &step_call_5671_c);
    MK2CPP_HandRegister(0x00005991u, &step_load_dp_d178_r1_b);
    MK2CPP_HandRegister(0x00005995u, &step_bra_58d4_b);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct TsScanSelfRegister
{
    TsScanSelfRegister() { hand_register_module(&ts_scan_fill); }
};

TsScanSelfRegister g_ts_scan_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
