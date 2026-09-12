/*
 * HAND pcm/pcm_irq_service - PCM dispatcher IRQ service
 * (class C5, rom1 cp=0, flat 0x25f0..0x2666).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1) and semantically rewritten in M4 closure step 2
 * (2026-09-13, named operations); addresses and registration unchanged. One
 * L0 hand entry per instruction PC so the host keeps its per-instruction
 * interrupt poll, cycles += 12, trace and MIDI/SM cadence
 * (docs/09_m4_integration.md 4.1).
 *
 * Semantics (Confirmed, C5): entries arrive from the C2 pcm_dispatcher via
 * BRA 0x51f3 -> 0x25f0 with r1 = voice slot. The routine indexes the PCM
 * status word at dp:r1*2+0x64d6, returns to the dispatcher at 0x51f6 when the
 * status is below 0x0e, and otherwise falls through into C6 svc_math at
 * 0x2669 when the pending byte at r0-0x11 is zero. It publishes the key byte
 * r1 to (br,0x3e), compares the (br,0x32/0x3a) and (br,0x34/0x3a) words and
 * selects the enable/disable code 0x0e/0x10, writing 0x00b5 to (br,0x16|0x18)
 * and to the voice words at r0+0x1a/0x1e, then r6 to r0+0/2/4. Finally it
 * acks the active flags dp:0xd0a8/dp:0xd0c4 (for r1, or for the r3 it read)
 * by writing 0xff, or returns when both are already idle.
 * Evidence: out/m4/18_closure_gap.md 1.3 C5 / 4.2 row 6; dasm:4259-4360;
 * pc_main:1973-2065. Confidence: C (ROM bytes + dasm).
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

/* (br,disp8): the base-register-relative control window (page 0). */
uint32_t br_addr(uint16_t disp)
{
    return (uint32_t)((mcu.br << 8) | (disp & 0xffu));
}

/* Odd-address-checked word read (stock MOVG2/MOVL word path). */
uint16_t read16_checked(uint32_t addr)
{
    if (addr & 1u)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    return MCU_Read16(addr);
}

/* ADD rS,rD (word). */
void add16_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, 0, 1);
}

/* CMP rS,rD (word): flags only. */
void cmp16_reg(uint16_t a, uint16_t b)
{
    MCU_SUB_Common((int32_t)(uint32_t)a, (int32_t)(uint32_t)b, 0, 1);
}

/* SUB @addr,#imm (byte): flags only (no write). */
void sub8_mem_flags(uint32_t addr, uint8_t imm)
{
    MCU_SUB_Common((int32_t)(uint32_t)MCU_Read(addr), (int32_t)(uint32_t)imm, 0, 0);
}

/* SUB @addr,#imm16 (word): flags only (no write). */
void sub16_mem_flags(uint32_t addr, uint16_t imm)
{
    MCU_SUB_Common((int32_t)(uint32_t)read16_checked(addr), (int32_t)(uint32_t)imm, 0, 1);
}

/* MOVG #imm -> @addr (byte immediate). */
void store8_mem(uint32_t addr, uint8_t imm)
{
    MCU_Write(addr, imm);
    MCU_SetStatusCommon((uint32_t)imm, 0);
}

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

/* ======================================================================
 * cp0 0x25F0..0x2666, 37 PC: C5 IRQ service, entry from C2 via 0x51f3,
 * return to the dispatcher at 0x51f6, fall-through to C6 svc_math 0x2669.
 * ====================================================================== */

/* 0x25F0 mov.w r1,r2 -- slot index. */
void step_copy_slot(void)
{
    mov16(mcu.r[2], (uint32_t)mcu.r[1]);
    mcu.pc = 0x25f2;
}

/* 0x25F2 add r2,r2 -- word offset. */
void step_double_slot(void)
{
    add16_reg(mcu.r[2], mcu.r[2]);
    mcu.pc = 0x25f4;
}

/* 0x25F4 mov.w @r2+0x64d6,r0 -- PCM status word pointer. */
void step_load_status_ptr(void)
{
    load16(mcu.r[0], ind_addr(2, 0x64d6));
    mcu.pc = 0x25f8;
}

/* 0x25F8 sub @r0,#0x000e -- status vs 0x0e (flags only). */
void step_compare_status(void)
{
    sub16_mem_flags(ind_addr(0, 0), 0x000e);
    mcu.pc = 0x25fd;
}

/* 0x25FD bcc -> 0x51f6 -- status below 0x0e: back to the dispatcher. */
void step_branch_status_low(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x2600 : 0x51f6;
}

/* 0x2600 tst @r0,-17 -- pending byte. */
void step_test_pending(void)
{
    tst8((uint32_t)MCU_Read(ind_addr(0, -17)));
    mcu.pc = 0x2603;
}

/* 0x2603 beq -> 0x2669 -- pending zero: C6 svc_math. */
void step_branch_pending_zero(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x2669 : 0x2605;
}

/* 0x2605 movs r1,(br,$3e) -- publish the key byte. */
void step_store_key(void)
{
    store8(br_addr(0x3e), mcu.r[1]);
    mcu.pc = 0x2607;
}

/* 0x2607 movl r4,(br,$32) -- value low byte. */
void step_load_value_lo(void)
{
    load8(mcu.r[4], br_addr(0x32));
    mcu.pc = 0x2609;
}

/* 0x2609 movlw r4,(br,$3a) -- value word. */
void step_load_value(void)
{
    load16(mcu.r[4], br_addr(0x3a));
    mcu.pc = 0x260b;
}

/* 0x260B movl r5,(br,$34) -- target low byte. */
void step_load_target_lo(void)
{
    load8(mcu.r[5], br_addr(0x34));
    mcu.pc = 0x260d;
}

/* 0x260D movlw r5,(br,$3a) -- target word. */
void step_load_target(void)
{
    load16(mcu.r[5], br_addr(0x3a));
    mcu.pc = 0x260f;
}

/* 0x260F cmp r4,r5 -- target vs value (flags only). */
void step_compare_value_target(void)
{
    cmp16_reg(mcu.r[5], mcu.r[4]);
    mcu.pc = 0x2611;
}

/* 0x2611 bcc -> 0x2622 -- target >= value: use the 0x10 code. */
void step_branch_below_target(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x2613 : 0x2622;
}

/* 0x2613 movi r6,#0x000e -- enable code. */
void step_status_enable(void)
{
    movi16(mcu.r[6], 0x000e);
    mcu.pc = 0x2616;
}

/* 0x2616 movg #0x00b5,(br,$16) -- control window. */
void step_write_ctrl_16(void)
{
    store16(br_addr(0x16), 0x00b5);
    mcu.pc = 0x261b;
}

/* 0x261B movg #0x00b5,@r0+26 -- voice word. */
void step_write_voice_26(void)
{
    store16(ind_addr(0, 26), 0x00b5);
    mcu.pc = 0x2620;
}

/* 0x2620 bra -> 0x262f -- publish the code. */
void step_branch_publish(void)
{
    mcu.pc = 0x262f;
}

/* 0x2622 movi r6,#0x0010 -- disable code. */
void step_status_disable(void)
{
    movi16(mcu.r[6], 0x0010);
    mcu.pc = 0x2625;
}

/* 0x2625 movg #0x00b5,(br,$18) -- control window. */
void step_write_ctrl_18(void)
{
    store16(br_addr(0x18), 0x00b5);
    mcu.pc = 0x262a;
}

/* 0x262A movg #0x00b5,@r0+30 -- voice word. */
void step_write_voice_30(void)
{
    store16(ind_addr(0, 30), 0x00b5);
    mcu.pc = 0x262f;
}

/* 0x262F mov.w r6,@r0 -- voice status 0. */
void step_store_status_0(void)
{
    store16(ind_addr(0, 0), mcu.r[6]);
    mcu.pc = 0x2632;
}

/* 0x2632 mov.w r6,@r0+2 -- voice status 2. */
void step_store_status_2(void)
{
    store16(ind_addr(0, 2), mcu.r[6]);
    mcu.pc = 0x2635;
}

/* 0x2635 mov.w r6,@r0+4 -- voice status 4. */
void step_store_status_4(void)
{
    store16(ind_addr(0, 4), mcu.r[6]);
    mcu.pc = 0x2638;
}

/* 0x2638 sub @r1+0xd0a8,#0xff -- active flag A (flags only). */
void step_compare_active_a(void)
{
    sub8_mem_flags(ind_addr(1, 0xd0a8), 0xff);
    mcu.pc = 0x263d;
}

/* 0x263D bne -> 0x264c -- A not idle. */
void step_branch_active_a(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x263f : 0x264c;
}

/* 0x263F sub @r1+0xd0c4,#0xff -- active flag B (flags only). */
void step_compare_active_b(void)
{
    sub8_mem_flags(ind_addr(1, 0xd0c4), 0xff);
    mcu.pc = 0x2644;
}

/* 0x2644 beq -> 0x2666 -- both idle: back to the dispatcher. */
void step_branch_both_idle(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x2666 : 0x2646;
}

/* 0x2646 mov.b @r1+0xd0c4,r3 -- active flag B value. */
void step_load_active_b(void)
{
    load8(mcu.r[3], ind_addr(1, 0xd0c4));
    mcu.pc = 0x264a;
}

/* 0x264A bra -> 0x2650 -- ack through flag B's owner. */
void step_branch_publish_b(void)
{
    mcu.pc = 0x2650;
}

/* 0x264C mov.b @r1+0xd0a8,r3 -- active flag A value. */
void step_load_active_a(void)
{
    load8(mcu.r[3], ind_addr(1, 0xd0a8));
    mcu.pc = 0x2650;
}

/* 0x2650 extu r3 -- zero-extend the flag value. */
void step_zero_extend_active(void)
{
    extu8(mcu.r[3]);
    mcu.pc = 0x2652;
}

/* 0x2652 movg #0xff,@r1+0xd0a8 -- ack flag A. */
void step_ack_active_a(void)
{
    store8_mem(ind_addr(1, 0xd0a8), 0xff);
    mcu.pc = 0x2657;
}

/* 0x2657 movg #0xff,@r1+0xd0c4 -- ack flag B. */
void step_ack_active_b(void)
{
    store8_mem(ind_addr(1, 0xd0c4), 0xff);
    mcu.pc = 0x265c;
}

/* 0x265C movg #0xff,@r3+0xd0a8 -- ack the peer's flag A. */
void step_ack_peer_a(void)
{
    store8_mem(ind_addr(3, 0xd0a8), 0xff);
    mcu.pc = 0x2661;
}

/* 0x2661 movg #0xff,@r3+0xd0c4 -- ack the peer's flag B. */
void step_ack_peer_b(void)
{
    store8_mem(ind_addr(3, 0xd0c4), 0xff);
    mcu.pc = 0x2666;
}

/* 0x2666 bra -> 0x51f6 -- back to the dispatcher. */
void step_branch_return(void)
{
    mcu.pc = 0x51f6;
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void pcm_irq_service_fill(void)
{
    MK2CPP_HandRegister(0x000025f0u, &step_copy_slot);
    MK2CPP_HandRegister(0x000025f2u, &step_double_slot);
    MK2CPP_HandRegister(0x000025f4u, &step_load_status_ptr);
    MK2CPP_HandRegister(0x000025f8u, &step_compare_status);
    MK2CPP_HandRegister(0x000025fdu, &step_branch_status_low);
    MK2CPP_HandRegister(0x00002600u, &step_test_pending);
    MK2CPP_HandRegister(0x00002603u, &step_branch_pending_zero);
    MK2CPP_HandRegister(0x00002605u, &step_store_key);
    MK2CPP_HandRegister(0x00002607u, &step_load_value_lo);
    MK2CPP_HandRegister(0x00002609u, &step_load_value);
    MK2CPP_HandRegister(0x0000260bu, &step_load_target_lo);
    MK2CPP_HandRegister(0x0000260du, &step_load_target);
    MK2CPP_HandRegister(0x0000260fu, &step_compare_value_target);
    MK2CPP_HandRegister(0x00002611u, &step_branch_below_target);
    MK2CPP_HandRegister(0x00002613u, &step_status_enable);
    MK2CPP_HandRegister(0x00002616u, &step_write_ctrl_16);
    MK2CPP_HandRegister(0x0000261bu, &step_write_voice_26);
    MK2CPP_HandRegister(0x00002620u, &step_branch_publish);
    MK2CPP_HandRegister(0x00002622u, &step_status_disable);
    MK2CPP_HandRegister(0x00002625u, &step_write_ctrl_18);
    MK2CPP_HandRegister(0x0000262au, &step_write_voice_30);
    MK2CPP_HandRegister(0x0000262fu, &step_store_status_0);
    MK2CPP_HandRegister(0x00002632u, &step_store_status_2);
    MK2CPP_HandRegister(0x00002635u, &step_store_status_4);
    MK2CPP_HandRegister(0x00002638u, &step_compare_active_a);
    MK2CPP_HandRegister(0x0000263du, &step_branch_active_a);
    MK2CPP_HandRegister(0x0000263fu, &step_compare_active_b);
    MK2CPP_HandRegister(0x00002644u, &step_branch_both_idle);
    MK2CPP_HandRegister(0x00002646u, &step_load_active_b);
    MK2CPP_HandRegister(0x0000264au, &step_branch_publish_b);
    MK2CPP_HandRegister(0x0000264cu, &step_load_active_a);
    MK2CPP_HandRegister(0x00002650u, &step_zero_extend_active);
    MK2CPP_HandRegister(0x00002652u, &step_ack_active_a);
    MK2CPP_HandRegister(0x00002657u, &step_ack_active_b);
    MK2CPP_HandRegister(0x0000265cu, &step_ack_peer_a);
    MK2CPP_HandRegister(0x00002661u, &step_ack_peer_b);
    MK2CPP_HandRegister(0x00002666u, &step_branch_return);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct PcmIrqServiceSelfRegister
{
    PcmIrqServiceSelfRegister() { hand_register_module(&pcm_irq_service_fill); }
};

PcmIrqServiceSelfRegister g_pcm_irq_service_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
