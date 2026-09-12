/*
 * HAND maint_counter_d1d5.cpp - maintenance slot scan over dp:0xd1d5 (cp4 0x433EA..0x43429).
 *
 * Split out of the former catch-all shared_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13); addresses and registration unchanged. One L0 hand entry per
 * instruction PC so the host keeps its per-instruction interrupt poll,
 * cycles += 12, trace and MIDI/SM cadence (docs/09_m4_integration.md 4.1).
 *
 * Semantics (Confirmed bytes): the routine scans 32 maintenance slots.
 * r1 walks the gate bytes dp:0xd1cb downwards while r0 counts 31..0 and
 * indexes dp:0xd435+r0 (cntjmp decrements first, so 0x1f seeds 32 passes).
 * A slot runs the handler at 0x3532 only when gate bit2 and bit1 are set,
 * gate bit3 is set or the global countdown dp:0xd1d5 is zero, and the indexed
 * byte is zero; the handler runs with r0/r1 saved on the stack. After the
 * loop dp:0xd1d5 is decremented and reloaded with 3 unless it underflowed.
 * Caller: bsr8 at 0x433B7 (A5); single exit via the rts at 0x43429. Dynamic
 * arm I: the fall-through PCs 0x433F8..0x43418 are statically reachable but
 * not proven in the recorded windows.
 * Evidence: out/m4/28_shared_status.md 1.2; out/m4/30_tail_status.md.
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

/* 0x433EA movi r1,#0xd1cb -- gate-byte cursor, walks downwards. */
void step_load_gate_cursor(void)
{
    movi16(mcu.r[1], 0xd1cb);
    mcu.pc = 0x33ed;
}

/* 0x433ED movi r0,#0x001f -- slot index; cntjmp makes it 32 passes. */
void step_load_slot_index(void)
{
    movi16(mcu.r[0], 0x001f);
    mcu.pc = 0x33f0;
}

/* 0x433F0 btsti @r1,#2 -- Z = gate bit2 clear. */
void step_test_gate_bit2(void)
{
    btsti_reg_bit(1, 2);
    mcu.pc = 0x33f2;
}

/* 0x433F2 beq -> 0x3419 -- gate bit2 clear: skip this slot. */
void step_skip_if_bit2_clear(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3419 : 0x33f4;
}

/* 0x433F4 btsti @r1,#1 -- Z = gate bit1 clear. */
void step_test_gate_bit1(void)
{
    btsti_reg_bit(1, 1);
    mcu.pc = 0x33f6;
}

/* 0x433F6 beq -> 0x3419 -- gate bit1 clear: skip this slot. */
void step_skip_if_bit1_clear(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3419 : 0x33f8;
}

/* 0x433F8 btsti @r1,#3 -- Z = gate bit3 clear. */
void step_test_gate_bit3(void)
{
    btsti_reg_bit(1, 3);
    mcu.pc = 0x33fa;
}

/* 0x433FA bne -> 0x3402 -- bit3 set: run the slot regardless of d1d5. */
void step_branch_on_bit3(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x33fc : 0x3402;
}

/* 0x433FC tst (dp,0xd1d5) -- global countdown byte. */
void step_test_countdown(void)
{
    tst8((uint32_t)MCU_Read(dp_addr(0xd1d5)));
    mcu.pc = 0x3400;
}

/* 0x43400 bne -> 0x3419 -- bit3 clear and countdown nonzero: skip. */
void step_skip_if_countdown(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3402 : 0x3419;
}

/* 0x43402 mov.b r0,r2 -- seed the slot index. */
void step_seed_slot_index(void)
{
    mov8(mcu.r[2], (uint32_t)mcu.r[0]);
    mcu.pc = 0x3404;
}

/* 0x43404 extu r2 -- zero-extend the index byte. */
void step_zero_extend_index(void)
{
    uint32_t data = (uint32_t)(mcu.r[2] & 0xffu);
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(data == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
    mcu.pc = 0x3406;
}

/* 0x43406 add.w #0xd435,r2 -- slot status byte address. */
void step_add_slot_base(void)
{
    mcu.r[2] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[2], 0xd435, 0, 1);
    mcu.pc = 0x340a;
}

/* 0x4340A tst @r2 -- slot status byte. */
void step_test_slot_byte(void)
{
    tst8((uint32_t)MCU_Read(reg_addr(2)));
    mcu.pc = 0x340c;
}

/* 0x4340C bne -> 0x3419 -- slot byte nonzero: skip. */
void step_skip_if_slot_set(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x340e : 0x3419;
}

/* 0x4340E push.w r0 -- save the slot index for the handler. */
void step_save_index(void)
{
    pushw(mcu.r[0]);
    mcu.pc = 0x3410;
}

/* 0x43410 push.w r1 -- save the gate cursor for the handler. */
void step_save_cursor(void)
{
    pushw(mcu.r[1]);
    mcu.pc = 0x3412;
}

/* 0x43412 jsr #0x3532 -- run the slot handler (gen-owned). */
void step_call_slot_handler(void)
{
    call(0x3415, 0x3532);
}

/* 0x43415 pop.w r1 -- restore the gate cursor. */
void step_restore_cursor(void)
{
    popw(mcu.r[1]);
    mcu.pc = 0x3417;
}

/* 0x43417 pop.w r0 -- restore the slot index. */
void step_restore_index(void)
{
    popw(mcu.r[0]);
    mcu.pc = 0x3419;
}

/* 0x43419 addq #-1,r1 -- next gate byte (downwards). */
void step_next_gate_byte(void)
{
    mcu.r[1] = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)mcu.r[1], -1, 0, 1);
    mcu.pc = 0x341b;
}

/* 0x4341B cntjmp r0 -- r0--; loop to 0x33f0 while it has not wrapped. */
void step_loop_next_slot(void)
{
    cntjmp(mcu.r[0], 0x33f0, 0x341e);
}

/* 0x4341E addq.b #-1,(dp,0xd1d5) -- count this pass down. */
void step_decrement_countdown(void)
{
    uint32_t addr = dp_addr(0xd1d5);
    uint32_t value = (uint32_t)MCU_Read(addr);
    value = (uint32_t)MCU_ADD_Common((int32_t)value, -1, 0, 0);
    MCU_Write(addr, (uint8_t)value);
    mcu.pc = 0x3422;
}

/* 0x43422 bcs -> 0x3429 -- underflow (0 -> 0xff): leave it and return. */
void step_branch_on_underflow(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x3429 : 0x3424;
}

/* 0x43424 movg #3 -> (dp,0xd1d5) -- reload the countdown. */
void step_reload_countdown(void)
{
    store8_imm_dp(0xd1d5, 0x03);
    mcu.pc = 0x3429;
}

/* 0x43429 rts. */
void step_return(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void maint_counter_d1d5_fill(void)
{
    MK2CPP_HandRegister(0x000433eau, &step_load_gate_cursor);
    MK2CPP_HandRegister(0x000433edu, &step_load_slot_index);
    MK2CPP_HandRegister(0x000433f0u, &step_test_gate_bit2);
    MK2CPP_HandRegister(0x000433f2u, &step_skip_if_bit2_clear);
    MK2CPP_HandRegister(0x000433f4u, &step_test_gate_bit1);
    MK2CPP_HandRegister(0x000433f6u, &step_skip_if_bit1_clear);
    MK2CPP_HandRegister(0x000433f8u, &step_test_gate_bit3);
    MK2CPP_HandRegister(0x000433fau, &step_branch_on_bit3);
    MK2CPP_HandRegister(0x000433fcu, &step_test_countdown);
    MK2CPP_HandRegister(0x00043400u, &step_skip_if_countdown);
    MK2CPP_HandRegister(0x00043402u, &step_seed_slot_index);
    MK2CPP_HandRegister(0x00043404u, &step_zero_extend_index);
    MK2CPP_HandRegister(0x00043406u, &step_add_slot_base);
    MK2CPP_HandRegister(0x0004340au, &step_test_slot_byte);
    MK2CPP_HandRegister(0x0004340cu, &step_skip_if_slot_set);
    MK2CPP_HandRegister(0x0004340eu, &step_save_index);
    MK2CPP_HandRegister(0x00043410u, &step_save_cursor);
    MK2CPP_HandRegister(0x00043412u, &step_call_slot_handler);
    MK2CPP_HandRegister(0x00043415u, &step_restore_cursor);
    MK2CPP_HandRegister(0x00043417u, &step_restore_index);
    MK2CPP_HandRegister(0x00043419u, &step_next_gate_byte);
    MK2CPP_HandRegister(0x0004341bu, &step_loop_next_slot);
    MK2CPP_HandRegister(0x0004341eu, &step_decrement_countdown);
    MK2CPP_HandRegister(0x00043422u, &step_branch_on_underflow);
    MK2CPP_HandRegister(0x00043424u, &step_reload_countdown);
    MK2CPP_HandRegister(0x00043429u, &step_return);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct MaintCounterD1d5SelfRegister
{
    MaintCounterD1d5SelfRegister() { hand_register_module(&maint_counter_d1d5_fill); }
};

MaintCounterD1d5SelfRegister g_maint_counter_d1d5_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
