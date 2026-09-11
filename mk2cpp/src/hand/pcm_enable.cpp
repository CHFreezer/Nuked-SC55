/*
 * HAND pcm/pcm_enable flats=0005525,0005527,0005529,0005662,0005664,0005666
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * M4 Wave 1, first hand slice (mk2cpp/docs/09_m4_integration.md 5.2-5.4):
 * PCM voice-enable flush. One L0 hand function per PC, each executing exactly
 * one H8 instruction and writing back pc/flags itself; cycles, interrupt
 * polling and device catch-up stay in the host (09 4.1). Semantics are the
 * stock MOVS word / MOVL byte paths (src/mcu_opcodes.cpp:755-800), not a
 * "better" reinterpretation. The device side effects of the emitted
 * MOVS/MOVL accesses stay on GT's PCM path:
 *
 *   0x5525  7b 00  mov.w r3,@(br,$00)  disable flush: pending mask bytes 0/1
 *   0x5527  7c 02  mov.w r4,@(br,$02)  disable flush: pending mask bytes 2/3
 *   0x5529  66 00  mov.b @(br,$00),r6  readback -> PCM_Read(0) latching commit
 *   0x5662  7d 00  mov.w r5,@(br,$00)  enable flush: pending mask bytes 0/1
 *   0x5664  7e 02  mov.w r6,@(br,$02)  enable flush: pending mask bytes 2/3
 *   0x5666  66 00  mov.b @(br,$00),r6  readback -> PCM_Read(0) latching commit
 *
 * Both byte writes of the MOVS words go through MCU_Write, whose stock shim
 * logs -pcmtrace and routes the main window to PCM_Write and the 0xe800
 * window (when pcm_ext_active) to PCM_WriteExt (src/mcu.cpp:970-985). Keeping
 * the shim in the write path is what makes pcm_trace.log byte-identical, so
 * the hand layer must not bypass it (08 5/R8). The readback calls PCM_Read
 * directly for the main window: reads below 4 perform the pending->committed
 * mask latch (src/pcm.cpp:215-220) and main-window reads have no -pcmtrace
 * side effect; every other address keeps GT's MCU_Read routing.
 *
 * No symbols from src/gen (a hand-only build must link), GT helpers only.
 */

#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_interrupt.h"
#include "pcm.h"

#include "native_pool.h"

/* Declared locally like the generated code does (defined in
 * src/mcu_opcodes.cpp); not part of mcu.h. */
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

/* Single-entry build switch (09 5.4): 0 makes this slice not register at all
 * (transparent gen/interpreter fallback) without touching other files. */
#define MK2CPP_HAND_PCM_ENABLE 1

namespace mk2c {

/* Device write for one byte of a page-0 PCM access. MCU_Write keeps the
 * -pcmtrace log and the stock main/ext window routing (PCM_Write /
 * PCM_WriteExt), exactly what the interpreter does for the same byte. */
static void pcm_write8(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
}

/* Device read for one byte of a page-0 PCM access. Main window (base
 * 0xe000, or 0xf000 on JV880; 0xe000..0xe03f on MK1) -> PCM_Read directly;
 * anything else (including the 0xe800 extension window and its read trace)
 * -> stock MCU_Read routing. Mirrors src/mcu.cpp:705-815. */
static uint8_t pcm_read8(uint32_t addr)
{
    uint32_t base = mcu_jv880 ? 0xf000u : 0xe000u;
    if (!mcu_mk1)
    {
        if (addr >= base && addr < (base | 0x400u))
            return PCM_Read(addr & 0x3f);
    }
    else if (addr >= 0xe000u && addr < 0xe040u)
    {
        return PCM_Read(addr & 0x3f);
    }
    return MCU_Read(addr);
}

/* mov.w rN,@(br,disp8) -- MCU_Opcode_Short_MOVS word path
 * (src/mcu_opcodes.cpp:779-800): opcode byte + disp8, addr = (br<<8)|disp,
 * odd address raises ADDRESS_ERROR, the 16-bit rN is written high byte
 * first (MCU_Write16 order), status = common(data, 1). */
static void pcm_movs_word(uint32_t reg)
{
    (void)MCU_ReadCodeAdvance(); /* opcode byte */
    uint16_t addr = (uint16_t)(mcu.br << 8);
    addr |= MCU_ReadCodeAdvance();
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)mcu.r[reg];
    pcm_write8((uint32_t)addr, (uint8_t)(data >> 8));
    pcm_write8((uint32_t)addr + 1u, (uint8_t)(data & 0xff));
    MCU_SetStatusCommon(data, 1);
}

/* mov.b @(br,disp8),r6 -- MCU_Opcode_Short_MOVL byte path
 * (src/mcu_opcodes.cpp:755-777): opcode byte + disp8, addr = (br<<8)|disp
 * (byte access: no odd-address check), r6 low byte = data,
 * status = common(data, 0). */
static void pcm_movl_byte_r6(void)
{
    (void)MCU_ReadCodeAdvance(); /* opcode byte */
    uint16_t addr = (uint16_t)(mcu.br << 8);
    addr |= MCU_ReadCodeAdvance();
    uint32_t data = (uint32_t)pcm_read8((uint32_t)addr);
    mcu.r[6] &= ~0xff;
    mcu.r[6] |= (uint16_t)data;
    MCU_SetStatusCommon(data, 0);
}

/* ---- per-PC L0 entries (one H8 instruction each, 09 4.3) -------------- */

void pcm_disable_flush_w0(void) { pcm_movs_word(3); }        /* 0x5525: 7b 00 */
void pcm_disable_flush_w2(void) { pcm_movs_word(4); }        /* 0x5527: 7c 02 */
void pcm_disable_flush_readback(void) { pcm_movl_byte_r6(); } /* 0x5529: 66 00 */
void pcm_enable_flush_w0(void) { pcm_movs_word(5); }         /* 0x5662: 7d 00 */
void pcm_enable_flush_w2(void) { pcm_movs_word(6); }         /* 0x5664: 7e 02 */
void pcm_enable_flush_readback(void) { pcm_movl_byte_r6(); }  /* 0x5666: 66 00 */

} /* namespace mk2c */

void MK2CPP_HandFillTables(void)
{
#if MK2CPP_HAND_PCM_ENABLE
    MK2CPP_HandRegister(0x00005525u, &mk2c::pcm_disable_flush_w0);
    MK2CPP_HandRegister(0x00005527u, &mk2c::pcm_disable_flush_w2);
    MK2CPP_HandRegister(0x00005529u, &mk2c::pcm_disable_flush_readback);
    MK2CPP_HandRegister(0x00005662u, &mk2c::pcm_enable_flush_w0);
    MK2CPP_HandRegister(0x00005664u, &mk2c::pcm_enable_flush_w2);
    MK2CPP_HandRegister(0x00005666u, &mk2c::pcm_enable_flush_readback);
#endif
    /* M4 slice-2 voice pool (native_pool.cpp); inert until Wave 2b registers
     * the pool-init L0 PCs there. */
    MK2CPP_PoolFillTables();
    void MK2CPP_VoiceMaterializeFillTables(void); MK2CPP_VoiceMaterializeFillTables(); /* voice_materialize.cpp (S1/S2) */
    void MK2CPP_MaskAccFillTables(void); MK2CPP_MaskAccFillTables(); /* mask_acc.cpp (B7) */
}

void MK2CPP_HandPostReset(void)
{
    mk2c::pool_post_reset();
}
