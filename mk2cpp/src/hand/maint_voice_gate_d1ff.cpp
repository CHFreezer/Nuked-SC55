/*
 * HAND maint_voice_gate_d1ff.cpp - maintenance voice on/off gate
 * (cp4 0x4342A..0x434CA).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13); addresses and registration unchanged. One L0 hand entry per
 * instruction PC so the host keeps its per-instruction interrupt poll,
 * cycles += 12, trace and MIDI/SM cadence (docs/09_m4_integration.md 4.1).
 *
 * Semantics (Confirmed bytes): gate byte dp:0xd1ff drives the voice-enable
 * handshake. When it is not 0xff and dp:0xd421 is zero, it is retired to
 * 0xff and bit7 of the incoming value is set; when it is 0xff and dp:0xd421
 * is non-zero, a new gate is derived: notes below 0x14 are looked up in the
 * rom2 table at 0x320b+note (paged with LDC #4,cr5 -> dp=4), notes 0x14 and
 * above map 0x45/0x2a/0x50 to 0x20/0x21/0x22 when the mode dp:0xd364 is 4
 * (default 0x40), then bit 0x40 is forced and dp:0xd1ff is written. The new
 * gate is then passed through 0x3574 unless dp:0xd364 is 4 and through
 * 0x3532 unless the lookup returned 0xff; if the masked channel (r0 & 0x3f)
 * is one of 0x12..0x15, bit3 of dp:0xd1ac[channel] is set when bit7 of the
 * gate is clear and cleared when it is set. Caller: jsr at 0x433BF (A5);
 * single return through the rts at 0x434CA. Calls 0x3532/0x3574 (other
 * owners). All alternate arms transcribed per ROM bytes (C).
 * Evidence: out/m4/28_shared_status.md 1.2 and 4.
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

/* ---- local forms (bodies copied from the case table) --------------------- */

/* EXTU rN (byte): zero-extend the low byte; N=0, Z from result, V=0, C=0. */
void extu8(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xffu);
    reg = (uint16_t)data;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(data == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* ADD.W #imm,rN (word). */
void add16_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 1);
}

/* OR #imm,rN (byte form used here): rN |= imm. */
void or8_imm(uint16_t &reg, uint8_t imm)
{
    reg = (uint16_t)(reg | (uint16_t)imm);
    MCU_SetStatusCommon(reg, 0);
}

/* AND #imm,rN (byte form used here): rN low byte &= imm. */
void and8_imm(uint16_t &reg, uint8_t imm)
{
    uint32_t data = (uint32_t)reg;
    data &= (uint32_t)imm;
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(reg, 0);
}

/* BTSTI rN #bit: Z = the register's low-byte bit is clear. */
void btsti_reg(uint16_t reg, unsigned bit)
{
    MCU_SetStatus((((uint32_t)reg & 0xffu) & (1u << bit)) == 0, STATUS_Z);
}

/* CMP.B #imm,(dp,disp): flags only. */
void cmp8_mem_dp(uint16_t disp, uint8_t imm)
{
    uint32_t t1 = (uint32_t)MCU_Read(dp_addr(disp));
    MCU_SUB_Common((int32_t)t1, (int32_t)(uint32_t)imm, 0, 0);
}

/* ======================================================================
 * cp4 0x4342A..0x434CA, 65 PC: gate byte dp:0xd1ff, enable dp:0xd421,
 * note dp:0xd207, mode dp:0xd364, rom2 table 0x320b+note (dp=4) and the
 * dp:0xd1ac bit3 toggle for channels 0x12..0x15.
 * ====================================================================== */

/* 0x4342A mov.b (dp,0xd1ff),r0 -- current gate byte. */
void step_load_gate(void)
{
    load8(mcu.r[0], dp_addr(0xd1ff));
    mcu.pc = 0x342e;
}

/* 0x4342E cmp.b #0xff,r0. */
void step_compare_gate_ff(void)
{
    cmp8_imm(mcu.r[0], 0xff);
    mcu.pc = 0x3430;
}

/* 0x43430 bne -> 0x347b -- gate not 0xff: take the retire arm. */
void step_branch_if_gate_not_ff(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3432 : 0x347b;
}

/* 0x43432 tst (dp,0xd421) -- enable latch. */
void step_test_enable_a(void)
{
    tst8((uint32_t)MCU_Read(dp_addr(0xd421)));
    mcu.pc = 0x3436;
}

/* 0x43436 bne -> 0x343b -- enabled: derive a new gate. */
void step_branch_if_enable_a(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3438 : 0x343b;
}

/* 0x43438 jmp #0x34ca -- not enabled: return. */
void step_return_direct(void)
{
    mcu.pc = 0x34ca;
}

/* 0x4343B mov.b (dp,0xd207),r0 -- note number. */
void step_load_note(void)
{
    load8(mcu.r[0], dp_addr(0xd207));
    mcu.pc = 0x343f;
}

/* 0x4343F cmp.b #0x14,r0. */
void step_compare_note_low(void)
{
    cmp8_imm(mcu.r[0], 0x14);
    mcu.pc = 0x3441;
}

/* 0x43441 bcs -> 0x3462 -- note < 0x14: rom2 table lookup. */
void step_branch_if_note_below(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x3462 : 0x3443;
}

/* 0x43443 movi r1,#0x40 -- default gate. */
void step_default_gate(void)
{
    movi8(mcu.r[1], 0x40);
    mcu.pc = 0x3445;
}

/* 0x43445 cmp.b #4,(dp,0xd364) -- voice mode. */
void step_compare_mode(void)
{
    cmp8_mem_dp(0xd364, 0x04);
    mcu.pc = 0x344a;
}

/* 0x4344A bne -> 0x3470 -- mode != 4: keep the default. */
void step_branch_if_mode_not4(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x344c : 0x3470;
}

/* 0x4344C movi r1,#0x20 -- mode 4, note 0x45 gate. */
void step_gate_20(void)
{
    movi8(mcu.r[1], 0x20);
    mcu.pc = 0x344e;
}

/* 0x4344E cmp.b #0x45,r0. */
void step_compare_note_45(void)
{
    cmp8_imm(mcu.r[0], 0x45);
    mcu.pc = 0x3450;
}

/* 0x43450 beq -> 0x3470 -- note 0x45. */
void step_branch_if_note_45(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x3452;
}

/* 0x43452 movi r1,#0x21 -- mode 4, note 0x2a gate. */
void step_gate_21(void)
{
    movi8(mcu.r[1], 0x21);
    mcu.pc = 0x3454;
}

/* 0x43454 cmp.b #0x2a,r0. */
void step_compare_note_2a(void)
{
    cmp8_imm(mcu.r[0], 0x2a);
    mcu.pc = 0x3456;
}

/* 0x43456 beq -> 0x3470 -- note 0x2a. */
void step_branch_if_note_2a(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x3458;
}

/* 0x43458 movi r1,#0x22 -- mode 4, note 0x50 gate. */
void step_gate_22(void)
{
    movi8(mcu.r[1], 0x22);
    mcu.pc = 0x345a;
}

/* 0x4345A cmp.b #0x50,r0. */
void step_compare_note_50(void)
{
    cmp8_imm(mcu.r[0], 0x50);
    mcu.pc = 0x345c;
}

/* 0x4345C beq -> 0x3470 -- note 0x50. */
void step_branch_if_note_50(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3470 : 0x345e;
}

/* 0x4345E movi r1,#0x40 -- fallback default. */
void step_gate_40(void)
{
    movi8(mcu.r[1], 0x40);
    mcu.pc = 0x3460;
}

/* 0x43460 bra -> 0x3470. */
void step_branch_apply(void)
{
    mcu.pc = 0x3470;
}

/* 0x43462 extu r0 -- zero-extend the low note. */
void step_zero_extend_note(void)
{
    extu8(mcu.r[0]);
    mcu.pc = 0x3464;
}

/* 0x43464 add.w #0x320b,r0 -- rom2 table offset. */
void step_add_table_base(void)
{
    add16_imm(mcu.r[0], 0x320b);
    mcu.pc = 0x3468;
}

/* 0x43468 ldc #4,cr5 -- page the next @r0 access to rom2 (dp = 4). */
void step_select_page4(void)
{
    mcu.pc = 0x346b;
    ldc_cr(5, 0x04);
}

/* 0x4346B mov.b @r0,r1 -- table gate for a low note. */
void step_load_table_gate(void)
{
    load8(mcu.r[1], reg_addr(0));
    mcu.pc = 0x346d;
}

/* 0x4346D ldc #0,cr5 -- restore page 0 (dp = 0). */
void step_restore_page0(void)
{
    mcu.pc = 0x3470;
    ldc_cr(5, 0x00);
}

/* 0x43470 or #0x40,r1 -- force the gate flag. */
void step_or_gate_flag(void)
{
    or8_imm(mcu.r[1], 0x40);
    mcu.pc = 0x3473;
}

/* 0x43473 mov.b r1,r0. */
void step_copy_gate(void)
{
    mov8(mcu.r[0], (uint32_t)mcu.r[1]);
    mcu.pc = 0x3475;
}

/* 0x43475 mov.b r0,(dp,0xd1ff) -- publish the new gate. */
void step_store_gate(void)
{
    store8(dp_addr(0xd1ff), mcu.r[0]);
    mcu.pc = 0x3479;
}

/* 0x43479 bra -> 0x348a. */
void step_branch_apply2(void)
{
    mcu.pc = 0x348a;
}

/* 0x4347B tst (dp,0xd421) -- enable latch (retire arm). */
void step_test_enable_b(void)
{
    tst8((uint32_t)MCU_Read(dp_addr(0xd421)));
    mcu.pc = 0x347f;
}

/* 0x4347F bne -> 0x34ca -- enabled: leave the retire alone. */
void step_branch_if_enable_b(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3481 : 0x34ca;
}

/* 0x43481 movg #0x00ff,(dp,0xd1ff) -- retire the gate. */
void step_clear_gate(void)
{
    mcu.pc = 0x3487;
    store8_imm_dp(0xd1ff, 0x00ff);
}

/* 0x43487 or #0x80,r0 -- mark the disable bit. */
void step_set_disable_bit(void)
{
    or8_imm(mcu.r[0], 0x80);
    mcu.pc = 0x348a;
}

/* 0x4348A mov.b r0,r1. */
void step_copy_to_r1(void)
{
    mov8(mcu.r[1], (uint32_t)mcu.r[0]);
    mcu.pc = 0x348c;
}

/* 0x4348C cmp.b #4,(dp,0xd364) -- voice mode. */
void step_compare_mode_again(void)
{
    cmp8_mem_dp(0xd364, 0x04);
    mcu.pc = 0x3491;
}

/* 0x43491 beq -> 0x349e -- mode 4: skip the 0x3574 lookup. */
void step_branch_if_mode4(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x349e : 0x3493;
}

/* 0x43493 push.w r1 -- save the gate around the lookup. */
void step_save_gate(void)
{
    pushw(mcu.r[1]);
    mcu.pc = 0x3495;
}

/* 0x43495 jsr #0x3574 -- lookup helper (other owner). */
void step_call_lookup(void)
{
    call(0x3498, 0x3574);
}

/* 0x43498 pop.w r1 -- restore the gate. */
void step_restore_gate(void)
{
    popw(mcu.r[1]);
    mcu.pc = 0x349a;
}

/* 0x4349A cmp.b #0xff,r0 -- did the lookup answer? */
void step_compare_lookup_ff(void)
{
    cmp8_imm(mcu.r[0], 0xff);
    mcu.pc = 0x349c;
}

/* 0x4349C beq -> 0x34a5 -- 0xff: no handler call. */
void step_branch_if_lookup_ff(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x34a5 : 0x349e;
}

/* 0x4349E push.w r1 -- save the gate for the handler. */
void step_save_gate2(void)
{
    pushw(mcu.r[1]);
    mcu.pc = 0x34a0;
}

/* 0x434A0 jsr #0x3532 -- slot handler (other owner). */
void step_call_handler(void)
{
    call(0x34a3, 0x3532);
}

/* 0x434A3 pop.w r1 -- restore the gate. */
void step_restore_gate2(void)
{
    popw(mcu.r[1]);
    mcu.pc = 0x34a5;
}

/* 0x434A5 mov.b r1,r0. */
void step_copy_result(void)
{
    mov8(mcu.r[0], (uint32_t)mcu.r[1]);
    mcu.pc = 0x34a7;
}

/* 0x434A7 and #0x3f,r0 -- channel number. */
void step_mask_channel(void)
{
    and8_imm(mcu.r[0], 0x3f);
    mcu.pc = 0x34aa;
}

/* 0x434AA cmp.b #0x14,r0. */
void step_compare_ch14(void)
{
    cmp8_imm(mcu.r[0], 0x14);
    mcu.pc = 0x34ac;
}

/* 0x434AC beq -> 0x34ba -- channel 0x14. */
void step_branch_ch14(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34ae;
}

/* 0x434AE cmp.b #0x15,r0. */
void step_compare_ch15(void)
{
    cmp8_imm(mcu.r[0], 0x15);
    mcu.pc = 0x34b0;
}

/* 0x434B0 beq -> 0x34ba -- channel 0x15. */
void step_branch_ch15(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34b2;
}

/* 0x434B2 cmp.b #0x12,r0. */
void step_compare_ch12(void)
{
    cmp8_imm(mcu.r[0], 0x12);
    mcu.pc = 0x34b4;
}

/* 0x434B4 beq -> 0x34ba -- channel 0x12. */
void step_branch_ch12(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34b6;
}

/* 0x434B6 cmp.b #0x13,r0. */
void step_compare_ch13(void)
{
    cmp8_imm(mcu.r[0], 0x13);
    mcu.pc = 0x34b8;
}

/* 0x434B8 bne -> 0x34ca -- not one of the gated channels: return. */
void step_branch_if_not_ch13(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x34ba : 0x34ca;
}

/* 0x434BA extu r0 -- zero-extend the channel. */
void step_zero_extend_channel(void)
{
    extu8(mcu.r[0]);
    mcu.pc = 0x34bc;
}

/* 0x434BC add.w #0xd1ac,r0 -- gate table entry for the channel. */
void step_index_gate_table(void)
{
    add16_imm(mcu.r[0], 0xd1ac);
    mcu.pc = 0x34c0;
}

/* 0x434C0 btsti r1,#7 -- Z = gate bit7 clear. */
void step_test_gate_bit(void)
{
    btsti_reg(mcu.r[1], 7);
    mcu.pc = 0x34c2;
}

/* 0x434C2 bne -> 0x34c8 -- bit7 set: clear the entry (disable). */
void step_branch_on_gate_bit(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x34c4 : 0x34c8;
}

/* 0x434C4 bset @r0,#3 -- bit7 clear: set the entry (enable). */
void step_set_gate_entry(void)
{
    bset_reg_bit(0, 3);
    mcu.pc = 0x34c6;
}

/* 0x434C6 bra -> 0x34ca. */
void step_branch_return(void)
{
    mcu.pc = 0x34ca;
}

/* 0x434C8 bclr @r0,#3 -- bit7 set: clear the entry. */
void step_clear_gate_entry(void)
{
    bclr_reg_bit(0, 3);
    mcu.pc = 0x34ca;
}

/* 0x434CA rts. */
void step_return(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_voice_gate_d1ff_fill(void)
{
    MK2CPP_HandRegister(0x0004342au, &step_load_gate);
    MK2CPP_HandRegister(0x0004342eu, &step_compare_gate_ff);
    MK2CPP_HandRegister(0x00043430u, &step_branch_if_gate_not_ff);
    MK2CPP_HandRegister(0x00043432u, &step_test_enable_a);
    MK2CPP_HandRegister(0x00043436u, &step_branch_if_enable_a);
    MK2CPP_HandRegister(0x00043438u, &step_return_direct);
    MK2CPP_HandRegister(0x0004343bu, &step_load_note);
    MK2CPP_HandRegister(0x0004343fu, &step_compare_note_low);
    MK2CPP_HandRegister(0x00043441u, &step_branch_if_note_below);
    MK2CPP_HandRegister(0x00043443u, &step_default_gate);
    MK2CPP_HandRegister(0x00043445u, &step_compare_mode);
    MK2CPP_HandRegister(0x0004344au, &step_branch_if_mode_not4);
    MK2CPP_HandRegister(0x0004344cu, &step_gate_20);
    MK2CPP_HandRegister(0x0004344eu, &step_compare_note_45);
    MK2CPP_HandRegister(0x00043450u, &step_branch_if_note_45);
    MK2CPP_HandRegister(0x00043452u, &step_gate_21);
    MK2CPP_HandRegister(0x00043454u, &step_compare_note_2a);
    MK2CPP_HandRegister(0x00043456u, &step_branch_if_note_2a);
    MK2CPP_HandRegister(0x00043458u, &step_gate_22);
    MK2CPP_HandRegister(0x0004345au, &step_compare_note_50);
    MK2CPP_HandRegister(0x0004345cu, &step_branch_if_note_50);
    MK2CPP_HandRegister(0x0004345eu, &step_gate_40);
    MK2CPP_HandRegister(0x00043460u, &step_branch_apply);
    MK2CPP_HandRegister(0x00043462u, &step_zero_extend_note);
    MK2CPP_HandRegister(0x00043464u, &step_add_table_base);
    MK2CPP_HandRegister(0x00043468u, &step_select_page4);
    MK2CPP_HandRegister(0x0004346bu, &step_load_table_gate);
    MK2CPP_HandRegister(0x0004346du, &step_restore_page0);
    MK2CPP_HandRegister(0x00043470u, &step_or_gate_flag);
    MK2CPP_HandRegister(0x00043473u, &step_copy_gate);
    MK2CPP_HandRegister(0x00043475u, &step_store_gate);
    MK2CPP_HandRegister(0x00043479u, &step_branch_apply2);
    MK2CPP_HandRegister(0x0004347bu, &step_test_enable_b);
    MK2CPP_HandRegister(0x0004347fu, &step_branch_if_enable_b);
    MK2CPP_HandRegister(0x00043481u, &step_clear_gate);
    MK2CPP_HandRegister(0x00043487u, &step_set_disable_bit);
    MK2CPP_HandRegister(0x0004348au, &step_copy_to_r1);
    MK2CPP_HandRegister(0x0004348cu, &step_compare_mode_again);
    MK2CPP_HandRegister(0x00043491u, &step_branch_if_mode4);
    MK2CPP_HandRegister(0x00043493u, &step_save_gate);
    MK2CPP_HandRegister(0x00043495u, &step_call_lookup);
    MK2CPP_HandRegister(0x00043498u, &step_restore_gate);
    MK2CPP_HandRegister(0x0004349au, &step_compare_lookup_ff);
    MK2CPP_HandRegister(0x0004349cu, &step_branch_if_lookup_ff);
    MK2CPP_HandRegister(0x0004349eu, &step_save_gate2);
    MK2CPP_HandRegister(0x000434a0u, &step_call_handler);
    MK2CPP_HandRegister(0x000434a3u, &step_restore_gate2);
    MK2CPP_HandRegister(0x000434a5u, &step_copy_result);
    MK2CPP_HandRegister(0x000434a7u, &step_mask_channel);
    MK2CPP_HandRegister(0x000434aau, &step_compare_ch14);
    MK2CPP_HandRegister(0x000434acu, &step_branch_ch14);
    MK2CPP_HandRegister(0x000434aeu, &step_compare_ch15);
    MK2CPP_HandRegister(0x000434b0u, &step_branch_ch15);
    MK2CPP_HandRegister(0x000434b2u, &step_compare_ch12);
    MK2CPP_HandRegister(0x000434b4u, &step_branch_ch12);
    MK2CPP_HandRegister(0x000434b6u, &step_compare_ch13);
    MK2CPP_HandRegister(0x000434b8u, &step_branch_if_not_ch13);
    MK2CPP_HandRegister(0x000434bau, &step_zero_extend_channel);
    MK2CPP_HandRegister(0x000434bcu, &step_index_gate_table);
    MK2CPP_HandRegister(0x000434c0u, &step_test_gate_bit);
    MK2CPP_HandRegister(0x000434c2u, &step_branch_on_gate_bit);
    MK2CPP_HandRegister(0x000434c4u, &step_set_gate_entry);
    MK2CPP_HandRegister(0x000434c6u, &step_branch_return);
    MK2CPP_HandRegister(0x000434c8u, &step_clear_gate_entry);
    MK2CPP_HandRegister(0x000434cau, &step_return);
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
