/*
 * HAND voice/mask_acc -- per-voice enable accumulation, one host step per H8
 * instruction (M4 B7, out/m4/16; round 7).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 2
 * M4 closure step 2 (semantic rewrite): one named void step function per PC,
 * registered flat=pc/cp0 via MK2CPP_HandRegister.
 *
 * Replaces the whole mask_acc block 0x1ad3..0x1af1 (IML gate, loop seed, loop
 * body, IML restore, rts) with one hand entry per instruction. Each entry
 * executes exactly one H8 instruction,
 * so the host keeps its per-instruction interrupt poll, TIMER_Clock, trace and
 * SM_Update cadence. The previous form was one L1 block returning 5*N (140):
 * a multi-instruction block defers the host tail, so -midiseq / real-time MIDI
 * bytes queued inside the block are consumed by the SM at the block end
 * instead of at each instruction boundary (the same timing drift that made the
 * materialize/pool-init/alloc-free slices per-PC).
 *
 * ROM segment (tools/baselines/dasm_full.txt:2893-2903);
 * rom1 bytes 0x1ad3..0x1af1 =
 *   0c 07 00 48 59 00 1b f1 a4 b4 80 f1 ac f2 40 f1 ac f2 90
 *   f1 a4 b4 13 01 b9 ed 0c f8 ff 58 19
 *
 *   0x1ad3  BSET_ORC #0x0700 r0    SR |= 0x0700 (IML=7); ex_ignore=1
 *   0x1ad7  movi r1 #0x001b        r1 = 0x1b; N=0 Z=0 V=0 C=0
 *   0x1ada  MOVG2 @r1+0xa4b4 r0    r0.lo = new[r1]        (byte; high byte kept)
 *   0x1ade  OR    @r1+0xacf2 r0    r0.lo |= acc[r1]       (GT OR <EA>,Rd)
 *   0x1ae2  MOVG3 r0 -> @r1+0xacf2 acc[r1] = r0.lo
 *   0x1ae6  CLR   @r1+0xa4b4       new[r1] = 0; N=0 Z=1 V=0 C=0
 *   0x1aea  cntjmp r1 -19          r1--; branch back while r1 != 0xffff
 *   0x1aed  BCLR_ANDC #0xf8ff r0   SR &= 0xf8ff (IML=0); ex_ignore=1
 *   0x1af1  rts                    pc = pop
 *
 * The head/tail PCs are registered as of 18_closure_gap 4.1 (P3). 0x1ad3 and
 * 0x1aed are the only SR / ex_ignore boundaries; per-PC stepping is exact
 * because each entry is one instruction and the flow_main three-run union has
 * zero IRQ/exception edges inside 0x1ad3-0x1af1 (out/m4/16 3.1). The two SR
 * writes are the gen transcription: MCU_ControlRegisterRead/Write(0,1) plus
 * mcu.ex_ignore = 1, so the IML window and interrupt poll cadence are
 * unchanged. The loop body itself contains no SR write, no push/pop, no
 * TRAPA/exception source (no T).
 *
 * Data model: acc (acf2) and new (a4b4) stay authoritative in page-0 SRAM, so
 * the ROM readers keep working unchanged -- C9 0x2e83 reads/clears acf2 at
 * 0x2f17/0x2f1e and note_fill clears it at 0x102f. The entries read and write
 * the same bytes with MCU_Read/MCU_Write; there is no native copy. The loop
 * walks exactly the slots the ROM seed selects (27 -> 28 iterations), so no
 * byte outside a4b4[0..27]/acf2[0..27] is ever touched (acf2[28] would be
 * ad0e[0], out/m4/16 1.7).
 *
 * Terminal state (13 2.1, C): after 0x1aea, r1 = 0xffff; r0 low byte =
 * new[0] | acc[0], high byte preserved through the byte operations; flags are
 * the last CLR's (N=0 Z=1 V=0 C=0). pc = 0x1aed (cntjmp not taken).
 *
 * All nine PCs are registered here; generated code no longer owns any of
 * 0x1ad3..0x1af1 (this module would otherwise shadow it, and duplicate
 * registration is fatal in mk2cpp.cpp:113-117).
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as pcm_enable.cpp / native_pool.cpp). */
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

/* Single-entry build switch (09 5.4): 0 makes this slice not register at all
 * (pure gen/interpreter fallback) without touching other files. */
#define MK2CPP_HAND_MASKACC 1

namespace mk2c {
namespace {

/* Effective address @rN+disp through the page register for rN (dp for r1). */
uint32_t ind_addr(uint32_t reg, uint16_t disp)
{
    uint8_t page = (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
    return ((uint32_t)page << 16) | (uint16_t)(mcu.r[reg] + disp);
}

/* ======================================================================== */
/* mask_acc 0x1ad3..0x1af1, 9 PCs (cp0, flat = pc)                          */
/* ======================================================================== */

/* 0x1ad3 BSET_ORC #0x0700 r0: SR |= 0x0700 (IML=7); ex_ignore=1. */
void step_mask_acc_set_iml7(void)
{
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val |= 0x0700u;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
    mcu.pc = 0x1ad7;
}

/* 0x1ad7 movi r1 #0x001b: loop seed (27 down to 0). */
void step_mask_acc_seed_loop(void)
{
    mcu.r[1] = 0x001b;
    MCU_SetStatusCommon(0x001b, 1);
    mcu.pc = 0x1ada;
}

/* 0x1ada MOVG2 @r1+0xa4b4 r0: r0.lo = new[r1], high byte kept. */
void step_mask_acc_load_new(void)
{
    uint8_t value = MCU_Read(ind_addr(1, 0xa4b4));
    mcu.r[0] = (uint16_t)((mcu.r[0] & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
    mcu.pc = 0x1ade;
}

/* 0x1ade OR @r1+0xacf2 r0 (GT OR <EA>,Rd: flags from r0). */
void step_mask_acc_or_accum(void)
{
    uint32_t data = MCU_Read(ind_addr(1, 0xacf2));
    mcu.r[0] = (uint16_t)(mcu.r[0] | data);
    MCU_SetStatusCommon(mcu.r[0], 0);
    mcu.pc = 0x1ae2;
}

/* 0x1ae2 MOVG3 r0 -> @r1+0xacf2: acc[r1] = r0.lo. */
void step_mask_acc_store_accum(void)
{
    uint8_t value = (uint8_t)mcu.r[0];
    MCU_Write(ind_addr(1, 0xacf2), value);
    MCU_SetStatusCommon(value, 0);
    mcu.pc = 0x1ae6;
}

/* 0x1ae6 CLR @r1+0xa4b4: new[r1] = 0; N=0 Z=1 V=0 C=0. */
void step_mask_acc_clear_new(void)
{
    MCU_Write(ind_addr(1, 0xa4b4), 0);
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
    mcu.pc = 0x1aea;
}

/* 0x1aea cntjmp r1 -19: r1--; branch back while r1 != 0xffff. */
void step_mask_acc_count_loop(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x1adau : 0x1aedu;
}

/* 0x1aed BCLR_ANDC #0xf8ff r0: SR &= 0xf8ff (IML=0); ex_ignore=1. */
void step_mask_acc_restore_iml(void)
{
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val &= 0xf8ffu;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
    mcu.pc = 0x1af1;
}

/* 0x1af1 rts. */
void step_mask_acc_rts(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */
} /* namespace mk2c */

void MK2CPP_MaskAccFillTables(void)
{
#if MK2CPP_HAND_MASKACC
    MK2CPP_HandRegister(0x00001ad3u, &mk2c::step_mask_acc_set_iml7);
    MK2CPP_HandRegister(0x00001ad7u, &mk2c::step_mask_acc_seed_loop);
    MK2CPP_HandRegister(0x00001adau, &mk2c::step_mask_acc_load_new);
    MK2CPP_HandRegister(0x00001adeu, &mk2c::step_mask_acc_or_accum);
    MK2CPP_HandRegister(0x00001ae2u, &mk2c::step_mask_acc_store_accum);
    MK2CPP_HandRegister(0x00001ae6u, &mk2c::step_mask_acc_clear_new);
    MK2CPP_HandRegister(0x00001aeau, &mk2c::step_mask_acc_count_loop);
    MK2CPP_HandRegister(0x00001aedu, &mk2c::step_mask_acc_restore_iml);
    MK2CPP_HandRegister(0x00001af1u, &mk2c::step_mask_acc_rts);
#endif
}
