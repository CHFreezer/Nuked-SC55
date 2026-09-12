/*
 * HAND maint_table_walk_d1d6.cpp - maintenance 4-entry table walk with
 * bit-field promotion (cp4 0x434CB..0x43531).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13); addresses and registration unchanged. One L0 hand entry per
 * instruction PC so the host keeps its per-instruction interrupt poll,
 * cycles += 12, trace and MIDI/SM cadence (docs/09_m4_integration.md 4.1).
 *
 * Semantics (Confirmed bytes): walks four rom2 bytes at 0x31f4 (paged with
 * LDC #4,cr5 -> dp=4, restored afterwards), masks each with dp:0xd1cd and
 * writes dp:0xecf6; then forces bits 0x8f and writes dp:0xecf6 again, reads
 * dp:0xecf5 and compares it against the dp:0xd1da stream (writing the new
 * value back and advancing both cursors). When a value differs or the XOR
 * against the walked byte is non-zero, the mismatch arm rebuilds an address
 * from the outer count (3-r4, *8) and scans bits 7..0 of the walked byte,
 * setting bit 0x80 in the candidate address when the target has the bit,
 * pushing r0-r6, calling 0x3574 and then 0x3532 unless the call returned
 * 0xff, popping the registers and writing the value to the pre-decremented
 * dp:0xd1d6 cursor. Caller: jsr at 0x433BC (A5). Alternate arms transcribed
 * (C bytes).
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

/* ---- local byte-size forms (bodies copied from the case table) ------------ */

/* AND.B rS,rD: low byte of rD &= low byte of rS, high byte preserved. */
void and8(uint16_t &dst, uint32_t src)
{
    uint32_t data = (uint32_t)dst;
    data &= (uint32_t)(src & 0xffu);
    dst = (uint16_t)((dst & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(dst, 0);
}

/* OR.B #imm,rD. */
void or8(uint16_t &dst, uint32_t src)
{
    dst = (uint16_t)(dst | (uint16_t)(src & 0xffu));
    MCU_SetStatusCommon(dst, 0);
}

/* XOR.B rS,rD: low byte of rD ^= low byte of rS. */
void xor8(uint16_t &dst, uint32_t src)
{
    uint32_t data = (uint32_t)(src & 0xffu);
    dst ^= (uint16_t)data;
    MCU_SetStatusCommon(dst, 0);
}

/* ADD.B rS,rD. */
void add8(uint16_t &dst, uint32_t src)
{
    int32_t t1 = MCU_ADD_Common((int32_t)(uint32_t)dst, (int32_t)(src & 0xffu), 0, 0);
    dst = (uint16_t)((dst & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

/* SUB.B rS,rD. */
void sub8(uint16_t &dst, uint32_t src)
{
    int32_t t1 = MCU_SUB_Common((int32_t)(uint32_t)dst, (int32_t)(src & 0xffu), 0, 0);
    dst = (uint16_t)((dst & 0xff00u) | ((uint32_t)t1 & 0xffu));
}

/* BTST rS,rD (byte): Z = bit (rS & 0xf) of rD is clear. */
void btst_reg(uint32_t src, uint16_t dst)
{
    uint32_t data = (uint32_t)(dst & 0xffu);
    uint32_t bit = (uint32_t)(src & 0x0fu);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
}

/* MULXU #imm,rD (byte): rD = imm * low byte, N/Z from the 16-bit product. */
void mulxu8_imm(uint16_t &reg, uint8_t imm)
{
    uint32_t product = (uint32_t)imm * (uint32_t)(reg & 0xffu);
    reg = (uint16_t)product;
    MCU_SetStatus((product & 0x8000u) != 0, STATUS_N);
    MCU_SetStatus(product == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG2 rN++ rD (byte): read @rN, then increment rN. */
void load8_postinc(uint32_t reg, uint16_t &dst)
{
    uint32_t oea = reg_addr(reg);
    mcu.r[reg] += 1;
    uint32_t data = (uint32_t)MCU_Read(oea);
    dst = (uint16_t)((dst & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* MOVG3 rS -> rN++ (byte): write rS to @rN, then increment rN. */
void store8_postinc(uint32_t reg, uint16_t src)
{
    uint32_t oea = reg_addr(reg);
    mcu.r[reg] += 1;
    MCU_Write(oea, (uint8_t)src);
    MCU_SetStatusCommon((uint32_t)src, 0);
}

/* MOVG3 rS -> @-rN (byte): pre-decrement rN, then write rS. */
void store8_predec(uint32_t reg, uint16_t src)
{
    mcu.r[reg] -= 1;
    uint32_t oea = (uint32_t)mcu.r[reg] & 0xffffu;
    uint32_t oep = (uint32_t)(page_of_reg(reg) & 0xff);
    MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)src);
    MCU_SetStatusCommon((uint32_t)src, 0);
}

/* MOVM rlist,@-SP (r0-r6): push r6..r0. */
void push_rlist_0_6(void)
{
    for (int i = 6; i >= 0; i--)
        MCU_PushStack(mcu.r[i]);
}

/* MOVM @SP+,rlist (r0-r6): pop r0..r6. */
void pop_rlist_0_6(void)
{
    for (int i = 0; i <= 6; i++)
        mcu.r[i] = MCU_PopStack();
}

/* ======================================================================
 * cp4 0x434CB..0x43531, 41 PC: 4-entry rom2 table walk (0x31f4, dp=4),
 * dp:0xecf6/dp:0xecf5 promotion and the dp:0xd1da/dp:0xd1d6 mismatch arm.
 * Caller jsr at 0x433BC (A5).
 * ====================================================================== */

/* 0x434CB movi r0,#0x31f4 -- rom2 table cursor. */
void step_load_table_cursor(void)
{
    movi16(mcu.r[0], 0x31f4);
    mcu.pc = 0x34ce;
}

/* 0x434CE movi r1,#0xd1da -- dp:0xd1da stream cursor. */
void step_load_write_cursor(void)
{
    movi16(mcu.r[1], 0xd1da);
    mcu.pc = 0x34d1;
}

/* 0x434D1 movi r2,#0xd1d6 -- dp:0xd1d6 stream cursor. */
void step_load_read_cursor(void)
{
    movi16(mcu.r[2], 0xd1d6);
    mcu.pc = 0x34d4;
}

/* 0x434D4 movi r4,#3 -- outer count; cntjmp makes it 4 entries. */
void step_load_outer_count(void)
{
    movi16(mcu.r[4], 0x0003);
    mcu.pc = 0x34d7;
}

/* 0x434D7 ldc #4,cr5 -- page the next @r0 access to rom2 (dp = 4). */
void step_select_page4(void)
{
    mcu.pc = 0x34da;
    ldc_cr(5, 0x04);
}

/* 0x434DA mov.b r0++,r3 -- table byte. */
void step_load_table_byte(void)
{
    load8_postinc(0, mcu.r[3]);
    mcu.pc = 0x34dc;
}

/* 0x434DC ldc #0,cr5 -- restore page 0 (dp = 0). */
void step_restore_page0(void)
{
    mcu.pc = 0x34df;
    ldc_cr(5, 0x00);
}

/* 0x434DF and (dp,0xd1cd),r3 -- keep only bits enabled in d1cd. */
void step_mask_with_d1cd(void)
{
    and8(mcu.r[3], (uint32_t)MCU_Read(dp_addr(0xd1cd)));
    mcu.pc = 0x34e3;
}

/* 0x434E3 mov.b r3,(dp,0xecf6) -- publish the masked byte. */
void step_store_ecf6(void)
{
    store8(dp_addr(0xecf6), mcu.r[3]);
    mcu.pc = 0x34e7;
}

/* 0x434E7 mov.b (dp,0xecf5),r5 -- current value. */
void step_load_ecf5(void)
{
    load8(mcu.r[5], dp_addr(0xecf5));
    mcu.pc = 0x34eb;
}

/* 0x434EB or #0x8f,r3 -- force the low flags. */
void step_force_flags(void)
{
    or8(mcu.r[3], 0x8f);
    mcu.pc = 0x34ee;
}

/* 0x434EE mov.b r3,(dp,0xecf6) -- publish again with the forced bits. */
void step_store_ecf6_again(void)
{
    store8(dp_addr(0xecf6), mcu.r[3]);
    mcu.pc = 0x34f2;
}

/* 0x434F2 mov.b r2++,r3 -- next dp:0xd1d6 stream byte. */
void step_load_read_byte(void)
{
    load8_postinc(2, mcu.r[3]);
    mcu.pc = 0x34f4;
}

/* 0x434F4 mov.b @r1,r6 -- previous dp:0xd1da stream byte. */
void step_load_seen_byte(void)
{
    load8(mcu.r[6], reg_addr(1));
    mcu.pc = 0x34f6;
}

/* 0x434F6 mov.b r5,r1++ -- store the current value, advance the cursor. */
void step_store_seen_byte(void)
{
    store8_postinc(1, mcu.r[5]);
    mcu.pc = 0x34f8;
}

/* 0x434F8 cmp.b r5,r6 -- did the stream change? */
void step_compare_seen(void)
{
    uint32_t t1 = (uint32_t)mcu.r[6];
    uint32_t t2 = (uint32_t)(mcu.r[5] & 0xffu);
    MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);
    mcu.pc = 0x34fa;
}

/* 0x434FA bne -> 0x3500 -- changed: next outer entry. */
void step_branch_if_changed(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x34fc : 0x3500;
}

/* 0x434FC xor.b r5,r3 -- differing bits against the walked byte. */
void step_xor_walked(void)
{
    xor8(mcu.r[3], (uint32_t)mcu.r[5]);
    mcu.pc = 0x34fe;
}

/* 0x434FE bne -> 0x3505 -- some bit differs: take the mismatch arm. */
void step_branch_if_xor_nonzero(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3500 : 0x3505;
}

/* 0x43500 cntjmp r4 -44 -> 0x34d7 -- next outer entry (4 total). */
void step_loop_outer(void)
{
    cntjmp(mcu.r[4], 0x34d7, 0x3503);
}

/* 0x43503 bra -> 0x3531 -- all entries matched: return. */
void step_branch_done(void)
{
    mcu.pc = 0x3531;
}

/* 0x43505 movi r1,#3 -- mismatch arm: outer count for the address build. */
void step_reload_count(void)
{
    movi16(mcu.r[1], 0x0003);
    mcu.pc = 0x3508;
}

/* 0x43508 sub r4,r1 (byte) -- r1 = 3 - r4. */
void step_count_delta(void)
{
    sub8(mcu.r[1], (uint32_t)mcu.r[4]);
    mcu.pc = 0x350a;
}

/* 0x4350A mulxu #8,r1 -- byte offset * 8. */
void step_scale_index(void)
{
    mulxu8_imm(mcu.r[1], 0x08);
    mcu.pc = 0x350d;
}

/* 0x4350D movi r4,#7 -- bit scan starts at bit 7. */
void step_load_bit(void)
{
    movi16(mcu.r[4], 0x0007);
    mcu.pc = 0x3510;
}

/* 0x43510 btst r4,r3 -- is the walked byte's bit r4 clear? */
void step_test_walked_bit(void)
{
    btst_reg((uint32_t)mcu.r[4], mcu.r[3]);
    mcu.pc = 0x3512;
}

/* 0x43512 beq -> 0x352c -- bit clear: next bit. */
void step_skip_if_walked_bit_clear(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x352c : 0x3514;
}

/* 0x43514 mov.b r1,r0 -- seed the candidate address from the scaled index. */
void step_seed_address(void)
{
    mov8(mcu.r[0], (uint32_t)mcu.r[1]);
    mcu.pc = 0x3516;
}

/* 0x43516 add r4,r0 (byte) -- add the bit position. */
void step_add_bit_offset(void)
{
    add8(mcu.r[0], (uint32_t)mcu.r[4]);
    mcu.pc = 0x3518;
}

/* 0x43518 btst r4,r5 -- does the current value have this bit set? */
void step_test_value_bit(void)
{
    btst_reg((uint32_t)mcu.r[4], mcu.r[5]);
    mcu.pc = 0x351a;
}

/* 0x4351A beq -> 0x351f -- value bit clear: keep the plain address. */
void step_branch_if_value_bit_clear(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x351f : 0x351c;
}

/* 0x4351C or #0x80,r0 -- mark the high bit of the candidate address. */
void step_set_high_bit(void)
{
    or8(mcu.r[0], 0x80);
    mcu.pc = 0x351f;
}

/* 0x4351F movm rlist,@-sp (r0-r6) -- save the registers for the callee. */
void step_save_regs(void)
{
    push_rlist_0_6();
    mcu.pc = 0x3521;
}

/* 0x43521 bsr #0x3574 -- helper call, returns to 0x3523. */
void step_call_helper(void)
{
    call(0x3523, 0x3574);
}

/* 0x43523 cmp.b #0xff,r0 -- did the helper answer? */
void step_compare_result(void)
{
    cmp8_imm(mcu.r[0], 0xff);
    mcu.pc = 0x3525;
}

/* 0x43525 beq -> 0x352a -- 0xff: no handler call. */
void step_branch_if_no_result(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x352a : 0x3527;
}

/* 0x43527 jsr #0x3532 -- run the slot handler. */
void step_call_handler(void)
{
    call(0x352a, 0x3532);
}

/* 0x4352A movm @sp+,rlist (r0-r6) -- restore the registers. */
void step_restore_regs(void)
{
    pop_rlist_0_6();
    mcu.pc = 0x352c;
}

/* 0x4352C cntjmp r4 -31 -> 0x3510 -- next bit (8 bits). */
void step_loop_bits(void)
{
    cntjmp(mcu.r[4], 0x3510, 0x352f);
}

/* 0x4352F mov.b r5,@-r2 -- push the value onto the dp:0xd1d6 stream. */
void step_store_back(void)
{
    store8_predec(2, mcu.r[5]);
    mcu.pc = 0x3531;
}

/* 0x43531 rts. */
void step_return(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_table_walk_d1d6_fill(void)
{
    MK2CPP_HandRegister(0x000434cbu, &step_load_table_cursor);
    MK2CPP_HandRegister(0x000434ceu, &step_load_write_cursor);
    MK2CPP_HandRegister(0x000434d1u, &step_load_read_cursor);
    MK2CPP_HandRegister(0x000434d4u, &step_load_outer_count);
    MK2CPP_HandRegister(0x000434d7u, &step_select_page4);
    MK2CPP_HandRegister(0x000434dau, &step_load_table_byte);
    MK2CPP_HandRegister(0x000434dcu, &step_restore_page0);
    MK2CPP_HandRegister(0x000434dfu, &step_mask_with_d1cd);
    MK2CPP_HandRegister(0x000434e3u, &step_store_ecf6);
    MK2CPP_HandRegister(0x000434e7u, &step_load_ecf5);
    MK2CPP_HandRegister(0x000434ebu, &step_force_flags);
    MK2CPP_HandRegister(0x000434eeu, &step_store_ecf6_again);
    MK2CPP_HandRegister(0x000434f2u, &step_load_read_byte);
    MK2CPP_HandRegister(0x000434f4u, &step_load_seen_byte);
    MK2CPP_HandRegister(0x000434f6u, &step_store_seen_byte);
    MK2CPP_HandRegister(0x000434f8u, &step_compare_seen);
    MK2CPP_HandRegister(0x000434fau, &step_branch_if_changed);
    MK2CPP_HandRegister(0x000434fcu, &step_xor_walked);
    MK2CPP_HandRegister(0x000434feu, &step_branch_if_xor_nonzero);
    MK2CPP_HandRegister(0x00043500u, &step_loop_outer);
    MK2CPP_HandRegister(0x00043503u, &step_branch_done);
    MK2CPP_HandRegister(0x00043505u, &step_reload_count);
    MK2CPP_HandRegister(0x00043508u, &step_count_delta);
    MK2CPP_HandRegister(0x0004350au, &step_scale_index);
    MK2CPP_HandRegister(0x0004350du, &step_load_bit);
    MK2CPP_HandRegister(0x00043510u, &step_test_walked_bit);
    MK2CPP_HandRegister(0x00043512u, &step_skip_if_walked_bit_clear);
    MK2CPP_HandRegister(0x00043514u, &step_seed_address);
    MK2CPP_HandRegister(0x00043516u, &step_add_bit_offset);
    MK2CPP_HandRegister(0x00043518u, &step_test_value_bit);
    MK2CPP_HandRegister(0x0004351au, &step_branch_if_value_bit_clear);
    MK2CPP_HandRegister(0x0004351cu, &step_set_high_bit);
    MK2CPP_HandRegister(0x0004351fu, &step_save_regs);
    MK2CPP_HandRegister(0x00043521u, &step_call_helper);
    MK2CPP_HandRegister(0x00043523u, &step_compare_result);
    MK2CPP_HandRegister(0x00043525u, &step_branch_if_no_result);
    MK2CPP_HandRegister(0x00043527u, &step_call_handler);
    MK2CPP_HandRegister(0x0004352au, &step_restore_regs);
    MK2CPP_HandRegister(0x0004352cu, &step_loop_bits);
    MK2CPP_HandRegister(0x0004352fu, &step_store_back);
    MK2CPP_HandRegister(0x00043531u, &step_return);
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
