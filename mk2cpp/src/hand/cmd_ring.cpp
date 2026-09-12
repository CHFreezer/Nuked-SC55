/*
 * HAND pcm/cmd_ring — cp0 command-ring state handlers (rom1 cp=0,
 * flat 0x0625..0x0673).
 *
 * Split out of the former catch-all pcm_misc.cpp by ROM routine (2026-09-12,
 * M4 closure step 1). The per-PC case bodies below are an unchanged mechanical
 * move, so behavior is bit-identical. Current form: one L0 hand entry per
 * instruction PC; see pcm_irq_service.cpp for the L0 rationale.
 * M4 closure step 2 (semantic rewrite): one named void step function per PC,
 * registered flat=pc/cp0 via MK2CPP_HandRegister.
 *
 * Semantics: the ring dispatcher at 0x5f0 (jsr r6) selects six per-state
 * handlers through the table at 0x5f4; the handlers drive voice/part work
 * through the pool and mask_acc paths. The same entry PCs are also used from
 * cp4 0x4062b/0x40674 (pool tail); cp0 and cp4 flat values differ and do not
 * collide.
 * Evidence: out/m4/33_dynamic_class.md 3.3; out/m4/35_pool_tail_status.md;
 * cov c=227,231,988. Confidence: C for bytes / S for state semantics.
 *
 * Registration: explicit MK2CPP_HandRegister entries in cmd_ring_fill, which
 * self-registers from a file-static initializer (hand_registry.h); no shared
 * aggregator file is edited and duplicate registration is fatal (mk2cpp.cpp).
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

/* ---- effective addresses / flags ---------------------------------------- */

uint32_t page_of(uint32_t reg)
{
    if (reg >= 6)
        return mcu.tp;
    if (reg >= 4)
        return mcu.ep;
    return mcu.dp;
}

/* @rN+disp16: low 16 bits add and wrap, page from the register file. */
uint32_t ea_r(uint32_t reg, uint16_t disp)
{
    return ((uint32_t)page_of(reg) << 16) | (uint16_t)(mcu.r[reg] + disp);
}

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* ---- byte / stack / control-flow primitives ------------------------------ */

/* MOVG #imm8 -> @rN+disp16 (GT opcode 6, siz 0). */
void mov_imm8_mem(uint32_t addr, int32_t imm)
{
    uint32_t data = (uint32_t)imm;
    MCU_Write(addr, (uint8_t)data);
    MCU_SetStatusCommon(data, 0);
}

/* BSET @mem #bit: Z = bit was clear, then set it. */
void bset(uint32_t addr, uint32_t bit)
{
    uint32_t data = MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    data |= 1u << bit;
    MCU_Write(addr, (uint8_t)data);
}

/* CLR @mem byte: write 0; N=0 Z=1 V=0 C=0. */
void clr8_mem(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

/* pjsr #page:target: push the return pc and the current cp, then switch. */
void pjsr(uint8_t page, uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    MCU_PushStack(mcu.cp);
    mcu.cp = page;
    mcu.pc = target;
}

/* bsr16: push the return pc, jump to the target. */
void call(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

void ret(void)
{
    mcu.pc = MCU_PopStack();
}

/* ======================================================================== */
/* command-ring state handlers, 18 PCs (cp0, flat = pc; states 08/0A/0C/0E/  */
/* 12/16 from the table at 0x5f4)                                           */
/* ======================================================================== */

/* 0x0625 state 08: pjsr #0x04:0x062b (pool scan E); returns to 0x0629. */
void step_cmd_ring_state08_pjsr_pool_scan(void)
{
    pjsr(0x04, 0x0629, 0x062b);
}

/* 0x0629 state 08: rts. */
void step_cmd_ring_state08_rts(void)
{
    ret();
}

/* 0x062a state 0A: pjsr #0x04:0x0674 (pool kill/release F); returns 0x062e. */
void step_cmd_ring_state0a_pjsr_pool_kill(void)
{
    pjsr(0x04, 0x062e, 0x0674);
}

/* 0x062e state 0A: bsr16 -> 0x1ad3 mask_acc; returns to 0x0631. */
void step_cmd_ring_state0a_bsr_mask_acc(void)
{
    call(0x0631, 0x1ad3);
}

/* 0x0631 state 0A: rts. */
void step_cmd_ring_state0a_rts(void)
{
    ret();
}

/* 0x0632 state 0C: MOVG #0xff -> @r3+0xa050. */
void step_cmd_ring_state0c_mov_ff_to_r3_0xa050(void)
{
    mov_imm8_mem(ea_r(3, 0xa050), (int8_t)0xff);
    mcu.pc = 0x0637;
}

/* 0x0637 state 0C: BSET @r3+0xa060 #0. */
void step_cmd_ring_state0c_bset_r3_0xa060_bit0(void)
{
    bset(ea_r(3, 0xa060), 0);
    mcu.pc = 0x063b;
}

/* 0x063b state 0C: rts. */
void step_cmd_ring_state0c_rts(void)
{
    ret();
}

/* 0x063c state 0E: CLR @r3+0xa060. */
void step_cmd_ring_state0e_clr_r3_0xa060(void)
{
    clr8_mem(ea_r(3, 0xa060));
    mcu.pc = 0x0640;
}

/* 0x0640 state 0E: rts. */
void step_cmd_ring_state0e_rts(void)
{
    ret();
}

/* 0x0653 state 12: bsr16 -> 0x14ad; returns to 0x0656. */
void step_cmd_ring_state12_bsr_to_0x14ad(void)
{
    call(0x0656, 0x14ad);
}

/* 0x0656 state 12: rts. */
void step_cmd_ring_state12_rts(void)
{
    ret();
}

/* 0x0660 state 16: CLR @r3+0xa060. */
void step_cmd_ring_state16_clr_r3_0xa060(void)
{
    clr8_mem(ea_r(3, 0xa060));
    mcu.pc = 0x0664;
}

/* 0x0664 state 16: MOVG #0xff -> @r3+0xa050. */
void step_cmd_ring_state16_mov_ff_to_r3_0xa050(void)
{
    mov_imm8_mem(ea_r(3, 0xa050), (int8_t)0xff);
    mcu.pc = 0x0669;
}

/* 0x0669 state 16: bsr16 -> 0x151e scan_b; returns to 0x066c. */
void step_cmd_ring_state16_bsr_to_scan_b(void)
{
    call(0x066c, 0x151e);
}

/* 0x066c state 16: pjsr #0x04:0x0674 (pool kill/release F); returns 0x0670. */
void step_cmd_ring_state16_pjsr_pool_kill(void)
{
    pjsr(0x04, 0x0670, 0x0674);
}

/* 0x0670 state 16: bsr16 -> 0x1ad3 mask_acc; returns to 0x0673. */
void step_cmd_ring_state16_bsr_to_mask_acc(void)
{
    call(0x0673, 0x1ad3);
}

/* 0x0673 state 16: rts. */
void step_cmd_ring_state16_rts(void)
{
    ret();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from the built-in
 * MK2CPP_HandFillTables aggregator (pcm_enable.cpp). */
void cmd_ring_fill(void)
{
    MK2CPP_HandRegister(0x00000625u, &step_cmd_ring_state08_pjsr_pool_scan);
    MK2CPP_HandRegister(0x00000629u, &step_cmd_ring_state08_rts);
    MK2CPP_HandRegister(0x0000062au, &step_cmd_ring_state0a_pjsr_pool_kill);
    MK2CPP_HandRegister(0x0000062eu, &step_cmd_ring_state0a_bsr_mask_acc);
    MK2CPP_HandRegister(0x00000631u, &step_cmd_ring_state0a_rts);
    MK2CPP_HandRegister(0x00000632u, &step_cmd_ring_state0c_mov_ff_to_r3_0xa050);
    MK2CPP_HandRegister(0x00000637u, &step_cmd_ring_state0c_bset_r3_0xa060_bit0);
    MK2CPP_HandRegister(0x0000063bu, &step_cmd_ring_state0c_rts);
    MK2CPP_HandRegister(0x0000063cu, &step_cmd_ring_state0e_clr_r3_0xa060);
    MK2CPP_HandRegister(0x00000640u, &step_cmd_ring_state0e_rts);
    MK2CPP_HandRegister(0x00000653u, &step_cmd_ring_state12_bsr_to_0x14ad);
    MK2CPP_HandRegister(0x00000656u, &step_cmd_ring_state12_rts);
    MK2CPP_HandRegister(0x00000660u, &step_cmd_ring_state16_clr_r3_0xa060);
    MK2CPP_HandRegister(0x00000664u, &step_cmd_ring_state16_mov_ff_to_r3_0xa050);
    MK2CPP_HandRegister(0x00000669u, &step_cmd_ring_state16_bsr_to_scan_b);
    MK2CPP_HandRegister(0x0000066cu, &step_cmd_ring_state16_pjsr_pool_kill);
    MK2CPP_HandRegister(0x00000670u, &step_cmd_ring_state16_bsr_to_mask_acc);
    MK2CPP_HandRegister(0x00000673u, &step_cmd_ring_state16_rts);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct CmdRingSelfRegister
{
    CmdRingSelfRegister() { hand_register_module(&cmd_ring_fill); }
};

CmdRingSelfRegister g_cmd_ring_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
