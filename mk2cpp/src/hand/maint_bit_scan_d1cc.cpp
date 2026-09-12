/*
 * HAND maint_bit_scan_d1cc.cpp - maintenance slot scan for gate bit0 and
 * promotion of low status bytes (cp4 0x43641..0x43694).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13); addresses and registration unchanged. One L0 hand entry per
 * instruction PC so the host keeps its per-instruction interrupt poll,
 * cycles += 12, trace and MIDI/SM cadence (docs/09_m4_integration.md 4.1).
 *
 * Semantics (Confirmed bytes): 32 slots (r1 = 0xd1cc, pre-decremented before
 * the test, so the gate bytes are 0xd1cb..0xd1ac; r0 = 0x1f counts 32 passes
 * via cntjmp). For each slot whose gate byte has bit0 set:
 *   1. read the slot status byte at dp:0xd435+idx; if it is >= 0x32 skip;
 *   2. clear the gate bit0;
 *   3. page in rom2 (LDC #4,cr5 sets dp=4) and read the byte table at
 *      0x321f+2*idx, then restore dp=0;
 *   4. if that byte is 0xff, or bit1 of dp:0xd1ac[byte] is clear, call the
 *      handler at 0x3532 (r0/r1 saved on the stack);
 *   5. otherwise set bit3 of dp:0xd1ac[byte] and clear dp:0xd435[byte].
 * Caller: jsr at 0x433B9 (A5). The immediate-byte bug at 0x43653 was fixed
 * here in 28 (d2 04 32 -> SUB immediate 0x32). Dynamic arm I: some slot arms
 * are statically reachable but not proven in the recorded windows.
 * Evidence: out/m4/28_shared_status.md 1.2, 2, 4.
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

/* 0x43641 movi r1,#0xd1cc -- gate-byte cursor (pre-decremented first). */
void step_load_gate_cursor(void)
{
    movi16(mcu.r[1], 0xd1cc);
    mcu.pc = 0x3644;
}

/* 0x43644 movi r0,#0x001f -- slot index; cntjmp makes it 32 passes. */
void step_load_slot_index(void)
{
    movi16(mcu.r[0], 0x001f);
    mcu.pc = 0x3647;
}

/* 0x43647 btsti --r1,#0 -- decrement the cursor, then Z = gate bit0 clear. */
void step_test_gate_bit0(void)
{
    mcu.r[1] -= 1;
    btsti_reg_bit(1, 0);
    mcu.pc = 0x3649;
}

/* 0x43649 beq -> 0x3691 -- gate bit0 clear: next slot. */
void step_skip_if_bit0_clear(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3691 : 0x364b;
}

/* 0x4364B mov.b r0,r2 -- seed the slot index. */
void step_seed_slot_index(void)
{
    mov8(mcu.r[2], (uint32_t)mcu.r[0]);
    mcu.pc = 0x364d;
}

/* 0x4364D extu r2 -- zero-extend the index byte. */
void step_zero_extend_index(void)
{
    extu8(mcu.r[2]);
    mcu.pc = 0x364f;
}

/* 0x4364F add.w #0xd435,r2 -- slot status byte address. */
void step_add_slot_base(void)
{
    add16_imm(mcu.r[2], 0xd435);
    mcu.pc = 0x3653;
}

/* 0x43653 sub #0x32,@r2 (flags only, no write) -- C=1 means status < 0x32. */
void step_compare_slot_limit(void)
{
    uint32_t t1 = (uint32_t)MCU_Read(reg_addr(2));
    MCU_SUB_Common((int32_t)t1, 0x32, 0, 0);
    mcu.pc = 0x3656;
}

/* 0x43656 bcc -> 0x3691 -- status >= 0x32: next slot. */
void step_skip_if_at_limit(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x3658 : 0x3691;
}

/* 0x43658 bclr @r1,#0 -- consume the gate: clear bit0. */
void step_clear_gate_bit0(void)
{
    bclr_reg_bit(1, 0);
    mcu.pc = 0x365a;
}

/* 0x4365A mov.b r0,r2 -- re-seed the index. */
void step_reseed_slot_index(void)
{
    mov8(mcu.r[2], (uint32_t)mcu.r[0]);
    mcu.pc = 0x365c;
}

/* 0x4365C extu r2 -- zero-extend the index byte. */
void step_zero_extend_index_2(void)
{
    extu8(mcu.r[2]);
    mcu.pc = 0x365e;
}

/* 0x4365E add r2,r2 -- index * 2 (table stride). */
void step_double_index(void)
{
    add16_imm(mcu.r[2], mcu.r[2]);
    mcu.pc = 0x3660;
}

/* 0x43660 add.w #0x321f,r2 -- rom2 table offset. */
void step_add_table_base(void)
{
    add16_imm(mcu.r[2], 0x321f);
    mcu.pc = 0x3664;
}

/* 0x43664 ldc #4,cr5 -- page the next @r2 access to rom2 (dp = 4). */
void step_select_page4(void)
{
    mcu.pc = 0x3667;
    ldc_cr(5, 0x04);
}

/* 0x43667 mov.b @r2,r3 -- table byte for this slot. */
void step_load_table_byte(void)
{
    load8(mcu.r[3], reg_addr(2));
    mcu.pc = 0x3669;
}

/* 0x43669 ldc #0,cr5 -- restore page 0 (dp = 0). */
void step_restore_page0(void)
{
    mcu.pc = 0x366c;
    ldc_cr(5, 0x00);
}

/* 0x4366C cmp.b #0xff,r3. */
void step_compare_table_byte(void)
{
    cmp8_imm(mcu.r[3], 0xff);
    mcu.pc = 0x366e;
}

/* 0x4366E beq -> 0x3686 -- table byte 0xff: call the handler. */
void step_branch_if_table_ff(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3686 : 0x3670;
}

/* 0x43670 extu r3 -- zero-extend the table byte. */
void step_zero_extend_value(void)
{
    extu8(mcu.r[3]);
    mcu.pc = 0x3672;
}

/* 0x43672 mov.w r3,r2 -- keep the value for the status byte address. */
void step_copy_value_to_r2(void)
{
    mov16(mcu.r[2], (uint32_t)mcu.r[3]);
    mcu.pc = 0x3674;
}

/* 0x43674 add.w #0xd1ac,r3 -- gate table entry for this value. */
void step_index_gate_table(void)
{
    add16_imm(mcu.r[3], 0xd1ac);
    mcu.pc = 0x3678;
}

/* 0x43678 btsti @r3,#1 -- Z = gate flag clear. */
void step_test_gate_flag(void)
{
    btsti_reg_bit(3, 1);
    mcu.pc = 0x367a;
}

/* 0x4367A beq -> 0x3686 -- gate flag clear: call the handler. */
void step_branch_if_flag_clear(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3686 : 0x367c;
}

/* 0x4367C bset @r3,#3 -- promote: mark the gate entry. */
void step_set_gate_flag(void)
{
    bset_reg_bit(3, 3);
    mcu.pc = 0x367e;
}

/* 0x4367E add.w #0xd435,r2 -- status byte for the mapped value. */
void step_index_slot_status(void)
{
    add16_imm(mcu.r[2], 0xd435);
    mcu.pc = 0x3682;
}

/* 0x43682 clr @r2 (byte) -- consume the mapped status byte. */
void step_clear_slot_status(void)
{
    MCU_Write(reg_addr(2), (uint8_t)0);
    flags_clr();
    mcu.pc = 0x3684;
}

/* 0x43684 bra -> 0x3691 -- next slot. */
void step_branch_next_slot(void)
{
    mcu.pc = 0x3691;
}

/* 0x43686 push.w r0 -- save the slot index for the handler. */
void step_save_index(void)
{
    pushw(mcu.r[0]);
    mcu.pc = 0x3688;
}

/* 0x43688 push.w r1 -- save the gate cursor for the handler. */
void step_save_cursor(void)
{
    pushw(mcu.r[1]);
    mcu.pc = 0x368a;
}

/* 0x4368A jsr #0x3532 -- run the slot handler (gen-owned). */
void step_call_handler(void)
{
    call(0x368d, 0x3532);
}

/* 0x4368D pop.w r1 -- restore the gate cursor. */
void step_restore_cursor(void)
{
    popw(mcu.r[1]);
    mcu.pc = 0x368f;
}

/* 0x4368F pop.w r0 -- restore the slot index. */
void step_restore_index(void)
{
    popw(mcu.r[0]);
    mcu.pc = 0x3691;
}

/* 0x43691 cntjmp r0 -77 -> 0x3647 -- next slot while r0 has not wrapped. */
void step_loop_next_slot(void)
{
    cntjmp(mcu.r[0], 0x3647, 0x3694);
}

/* 0x43694 rts. */
void step_return(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_bit_scan_d1cc_fill(void)
{
    MK2CPP_HandRegister(0x00043641u, &step_load_gate_cursor);
    MK2CPP_HandRegister(0x00043644u, &step_load_slot_index);
    MK2CPP_HandRegister(0x00043647u, &step_test_gate_bit0);
    MK2CPP_HandRegister(0x00043649u, &step_skip_if_bit0_clear);
    MK2CPP_HandRegister(0x0004364bu, &step_seed_slot_index);
    MK2CPP_HandRegister(0x0004364du, &step_zero_extend_index);
    MK2CPP_HandRegister(0x0004364fu, &step_add_slot_base);
    MK2CPP_HandRegister(0x00043653u, &step_compare_slot_limit);
    MK2CPP_HandRegister(0x00043656u, &step_skip_if_at_limit);
    MK2CPP_HandRegister(0x00043658u, &step_clear_gate_bit0);
    MK2CPP_HandRegister(0x0004365au, &step_reseed_slot_index);
    MK2CPP_HandRegister(0x0004365cu, &step_zero_extend_index_2);
    MK2CPP_HandRegister(0x0004365eu, &step_double_index);
    MK2CPP_HandRegister(0x00043660u, &step_add_table_base);
    MK2CPP_HandRegister(0x00043664u, &step_select_page4);
    MK2CPP_HandRegister(0x00043667u, &step_load_table_byte);
    MK2CPP_HandRegister(0x00043669u, &step_restore_page0);
    MK2CPP_HandRegister(0x0004366cu, &step_compare_table_byte);
    MK2CPP_HandRegister(0x0004366eu, &step_branch_if_table_ff);
    MK2CPP_HandRegister(0x00043670u, &step_zero_extend_value);
    MK2CPP_HandRegister(0x00043672u, &step_copy_value_to_r2);
    MK2CPP_HandRegister(0x00043674u, &step_index_gate_table);
    MK2CPP_HandRegister(0x00043678u, &step_test_gate_flag);
    MK2CPP_HandRegister(0x0004367au, &step_branch_if_flag_clear);
    MK2CPP_HandRegister(0x0004367cu, &step_set_gate_flag);
    MK2CPP_HandRegister(0x0004367eu, &step_index_slot_status);
    MK2CPP_HandRegister(0x00043682u, &step_clear_slot_status);
    MK2CPP_HandRegister(0x00043684u, &step_branch_next_slot);
    MK2CPP_HandRegister(0x00043686u, &step_save_index);
    MK2CPP_HandRegister(0x00043688u, &step_save_cursor);
    MK2CPP_HandRegister(0x0004368au, &step_call_handler);
    MK2CPP_HandRegister(0x0004368du, &step_restore_cursor);
    MK2CPP_HandRegister(0x0004368fu, &step_restore_index);
    MK2CPP_HandRegister(0x00043691u, &step_loop_next_slot);
    MK2CPP_HandRegister(0x00043694u, &step_return);
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
