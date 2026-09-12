/*
 * HAND pcm_dispatch -- voice closure P0: IRQ0, C2 pcm_dispatcher, D1
 * voice_search, one host step per H8 instruction (M4 wave, out/m4/18 4.2 P0).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * Three P0 routines from mk2cpp/out/m4/18_closure_gap.md 4.2 were translated
 * instruction by instruction and semantically rewritten in M4 closure step 2
 * (2026-09-13): every instruction PC is now a named L0 entry registered with
 * MK2CPP_HandRegister, so the host keeps its per-instruction interrupt poll,
 * TIMER_Clock, trace and SM_Update cadence. A multi-instruction block would
 * defer the host tail and let -midiseq / real-time MIDI bytes queued inside
 * the block be consumed by the SM at the block end instead of at each
 * instruction boundary (same timing drift that made the earlier slices
 * per-PC). Coverage note: the IRQ0 fragment has no dynamic hit in any
 * recorded window (0x0522..0x053D are registration/build-only covered), while
 * voice_search and pcm_dispatcher are exercised by the regression song.
 *
 *   IRQ0          0x0522-0x053D, 10 PC (rom1; vector 32 -> 0x0522)
 *     STC r5 -> --r7; LDC #0 r5 (dp=0); stm #0x7f (push r0..r6);
 *     r0 = Read(0xe03e) (PCM status, dp page); r0 &= 0x1f; d15c[r0] = 0xff;
 *     r0=2, r1=1; LDC #0 r5; jmp 0x03db.  The shared tail 0x03db..0x0508 rte
 *     is a common event-queue epilogue used by several interrupt contexts and
 *     is outside the IRQ0 fragment (18 4.2 row 1; feasibility IRQ0 0x522-0x540),
 *     so it stays with gen/interpreter.
 *
 *   pcm_dispatcher 0x51CC/0x51D0-0x522A, 30 PC (rom1)
 *     0x51cc pjsr 0x04:1220 (A2 init_desc) pushes 0x51d0 then cp;
 *     0x51d0 trapa #0x10; on Z (trapa result != 0) scan d15c[0..0x1b] for a
 *     pending d15c, clear it and BRA 0x25f0 (C5 irq_service, out of scope);
 *     otherwise clear d154/d156, loop d0e0[0..0x1b], pop d0e0[r1] and dispatch
 *     through the 16-bit table at 0x522c (jmp r2 -> 0x5238 pcm_stop / 0x52cb
 *     pcm_play).  Gen's OVERLAP decodes 0x520a/0x521e are not instruction
 *     boundaries and are not registered.
 *
 *   voice_search  0x45CBE-0x45D99, 91 PC (rom2, cp=4, pc offsets 0x5cbe..)
 *     scans a368[0..0x1b] against r4=part, tracks max ad0e in r5 (0xff -> 0),
 *     scales it by dc76[part] (SHLL/MULXU/SWAP/SHLR), then the dc02/dc2a/dc2b
 *     gate arms update dc36[r0] mix fields; loop count r2 vs 0x10, BNE back to
 *     0x45c7c (outside this fragment).  The untraced fallthrough arms
 *     0x45cfe-0x45d2b and 0x45d69-0x45d70 are implemented from the ROM bytes
 *     via the same static decode as the generated table (confidence C).
 *
 * Baseline: tools/baselines/dasm_full.txt (IRQ0 372-381, dispatcher
 * 10630-10690, search 15901-16099); dasm_annotated.txt 6.4/6.5.  The gen map
 * (mk2cpp/src/gen/mk2c_r1.cpp, mk2c_r2.cpp) is the semantics reference for the
 * decoder-level details (flag sizes, sign-extension, odd-address traps); every
 * case below mirrors its per-PC body.  SRAM is the page-0 array the stock code
 * uses; device writes go through MCU_Write so the -pcmtrace routing is kept.
 *
 * No symbols from src/gen (a hand-only build must link), GT helpers only.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"
#include "mcu_interrupt.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as pcm_enable.cpp / native_allocfree.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* ---- effective addresses -------------------------------------------------- */

/* Page register for rN: dp (r0-r3), ep (r4/r5), tp (r6/r7). */
uint32_t page_of_reg(uint32_t reg)
{
    return (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
}

/* @rN+disp16: sum wraps inside 16 bits (src/mcu_opcodes.cpp general operand). */
uint32_t ind_addr(uint32_t reg, uint16_t disp)
{
    return ((uint32_t)page_of_reg(reg) << 16) | (uint16_t)(mcu.r[reg] + disp);
}

/* @rN with no displacement. */
uint32_t reg_addr(uint32_t reg)
{
    return ((uint32_t)page_of_reg(reg) << 16) | mcu.r[reg];
}

/* (dp,addr16): absolute through the DP page register. */
uint32_t dp_addr(uint16_t disp)
{
    return ((uint32_t)mcu.dp << 16) | disp;
}

/* ---- status / single-instruction operations (GT semantics) ---------------- */

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG2 @addr -> rN (byte, high byte preserved) */
void load8(uint16_t &reg, uint32_t addr)
{
    uint8_t value = MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

/* MOVG2 @addr -> rN (word; odd address traps before the read) */
void load16(uint16_t &reg, uint32_t addr)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t value = MCU_Read16(addr);
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* MOVG2 rS -> rD (byte register form) */
void mov8_reg(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)(src & 0xff);
    dst = (uint16_t)((dst & 0xff00u) | data);
    MCU_SetStatusCommon(data, 0);
}

/* MOVG3 rN -> @addr (byte) */
void store8(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* MOVG3 rN -> @addr (word; odd address traps before the write) is folded into
 * store16_predec / clr16_mem below; no plain word store is used by these PCs. */

/* CLR @addr (byte) */
void clr8_mem(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

/* CLR rN (byte: high byte preserved) */
void clr_reg_byte(uint16_t &reg)
{
    reg = (uint16_t)(reg & 0xff00u);
    flags_clr();
}

/* CLR rN (word) */
void clr_reg_word(uint16_t &reg)
{
    reg = 0;
    flags_clr();
}

/* CLR @addr (word; odd address traps before the write) */
void clr16_mem(uint32_t addr)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, 0);
    flags_clr();
}

/* TST @addr (byte): N/Z from the value, C cleared, V=0 */
void tst8_mem(uint32_t addr)
{
    MCU_SetStatusCommon(MCU_Read(addr), 0);
    MCU_SetStatus(0, STATUS_C);
}

/* TST rN (byte): N/Z from the low byte, C cleared, V=0 */
void tst8_reg(uint16_t reg)
{
    MCU_SetStatusCommon((uint32_t)(reg & 0xff), 0);
    MCU_SetStatus(0, STATUS_C);
}

/* BTSTI @addr #bit (byte): Z = (bit == 0) */
void btsti8_mem(uint32_t addr, uint8_t bit)
{
    uint32_t data = (uint32_t)MCU_Read(addr);
    MCU_SetStatus(((data >> bit) & 1u) == 0, STATUS_Z);
}

/* BTSTI @addr #bit (word; odd address traps before the read) */
void btsti16_mem(uint32_t addr, uint8_t bit)
{
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(addr);
    MCU_SetStatus(((data >> bit) & 1u) == 0, STATUS_Z);
}

/* SUB @addr #imm (byte): flags only, no write */
void sub8_nowrite(uint32_t addr, uint8_t imm)
{
    MCU_SUB_Common((int32_t)(uint32_t)MCU_Read(addr), (int32_t)(uint32_t)imm, 0, 0);
}

/* ADDQ #delta rN (word) */
void addq_word(uint16_t &reg, int delta)
{
    reg = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)reg, delta, 0, 1);
}

/* ADDQ #delta rN (byte: high byte preserved) */
void addq_byte_reg(uint16_t &reg, int delta)
{
    int32_t value = MCU_ADD_Common((int32_t)(uint32_t)(reg & 0xff), delta, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)value & 0xffu));
}

/* ADDQ #delta @addr (byte) */
void addq_byte_mem(uint32_t addr, int delta)
{
    int32_t value = MCU_ADD_Common((int32_t)(uint32_t)MCU_Read(addr), delta, 0, 0);
    MCU_Write(addr, (uint8_t)value);
}

/* cmp rD,b #imm: t1 = full rD, t2 = zero-extended imm8, byte flags */
void cmp_reg_imm8(uint16_t reg, uint8_t imm)
{
    MCU_SUB_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 0);
}

/* ADD #imm16 rD (word) */
void add16_imm(uint16_t &reg, uint16_t imm)
{
    int32_t value = MCU_ADD_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)imm, 0, 1);
    reg = (uint16_t)value;
}

/* ADD rD rD (byte form `a0 20`): t1 = full rD, t2 = its low byte */
void add8_self(uint16_t &reg)
{
    int32_t value = MCU_ADD_Common((int32_t)(uint32_t)reg,
                                   (int32_t)(uint32_t)(reg & 0xff), 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | ((uint32_t)value & 0xffu));
}

/* EXTU rN: zero-extend the low byte, N=0, Z=(value==0), V=0, C=0 */
void extu(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    reg = (uint16_t)data;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(data == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* SHLL rN (byte): C = old bit7, low byte shifted, N/Z/V from the result */
void shll_byte(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    uint32_t C = (data & 0x80u) != 0;
    data <<= 1;
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 0);
}

/* SHLR rN (byte): C = old bit0, low byte shifted, N/Z/V from the result */
void shlr_byte(uint16_t &reg)
{
    uint32_t data = (uint32_t)(reg & 0xff);
    uint32_t C = data & 1u;
    data >>= 1;
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 0);
}

/* MULXU rS rD (byte): rD = low(rS) * low(rD), word result */
void mulxu_byte(uint16_t src, uint16_t &dst)
{
    uint32_t t1 = (uint32_t)(src & 0xff);
    uint32_t t2 = (uint32_t)(dst & 0xff);
    t1 *= t2;
    t1 &= 0xffff;
    dst = (uint16_t)t1;
    MCU_SetStatus((t1 & 0x8000u) != 0, STATUS_N);
    MCU_SetStatus(t1 == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* SWAP rN: exchange the two bytes, word status */
void swap_word(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t data_h = data >> 8;
    uint32_t data_l = data & 0xff;
    data = (data_l << 8) | data_h;
    reg = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVI rN #imm16 / MOVE rN #imm8 */
void movi16(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

void move8(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

/* MOVG2 r7++ -> rN (word pop; odd address traps on the old r7) */
void load16_postinc(uint32_t reg, uint16_t &dst)
{
    uint32_t oea = (uint32_t)mcu.r[reg];
    mcu.r[reg] = (uint16_t)(mcu.r[reg] + 2);
    if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t data = MCU_Read16(((uint32_t)page_of_reg(reg) << 16) | (uint16_t)oea);
    dst = data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVG3 rN -> rM++ (byte store at rM, then rM++) */
void store8_postinc(uint32_t reg, uint16_t value)
{
    uint32_t addr = reg_addr(reg);
    mcu.r[reg] = (uint16_t)(mcu.r[reg] + 1);
    MCU_Write(addr, (uint8_t)value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* MOVG3 rN -> --rM (word push; odd address traps on the new rM) */
void store16_predec(uint32_t reg, uint16_t value)
{
    mcu.r[reg] = (uint16_t)(mcu.r[reg] - 2);
    uint32_t oea = (uint32_t)mcu.r[reg];
    if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(((uint32_t)page_of_reg(reg) << 16) | (uint16_t)oea, value);
    MCU_SetStatusCommon((uint32_t)value, 1);
}

/* ---- control / stack / trap instructions ---------------------------------- */

/* BSET_ORC #0x0700 r0: IML=7 window + ex_ignore (same as other hand slices) */
void bset_orc_iml7(void)
{
    mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
    mcu.ex_ignore = 1;
}

/* BCLR_ANDC #0xf8ff r0: IML=0 + ex_ignore */
void bclr_andc_iml0(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
}

/* LDC #0x00 r5: dp = 0, ex_ignore set (04 00 8d) */
void ldc_dp0(void)
{
    MCU_ControlRegisterWrite(5, 0, 0);
    mcu.ex_ignore = 1;
}

/* STC r5 -> --r7: push the low byte of dp, no flags (b7 9d) */
void stc_r5_predec_r7(void)
{
    mcu.r[7] = (uint16_t)(mcu.r[7] - 2);
    uint32_t oea = (uint32_t)mcu.r[7];
    uint32_t page = MCU_GetPageForRegister(7);
    uint32_t data = MCU_ControlRegisterRead(5, 0);
    MCU_Write(MCU_GetAddress((uint8_t)page, (uint16_t)oea), (uint8_t)data);
}

/* STM #rlist: push listed registers r7..r0 (r7 pushed as old r7 - 2) */
void stm_rlist(uint8_t rlist)
{
    for (int i = 7; i >= 0; i--)
    {
        if (rlist & (1 << i))
        {
            uint16_t data = mcu.r[i];
            if (i == 7)
                data = (uint16_t)(data - 2);
            MCU_PushStack(data);
        }
    }
}

/* Conditional branch selection (GT MCU_Jump_Bcc, src/mcu_opcodes.cpp:231). */
void bne(uint16_t taken, uint16_t fall)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? fall : taken;
}

void beq(uint16_t taken, uint16_t fall)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? taken : fall;
}

void bcc(uint16_t taken, uint16_t fall)
{
    mcu.pc = (mcu.sr & STATUS_C) ? fall : taken;
}

void bpl(uint16_t taken, uint16_t fall)
{
    mcu.pc = (mcu.sr & STATUS_N) ? fall : taken;
}

/* ---- IRQ0 0x0522..0x053D (rom1) ------------------------------------------- */

/* 0x0522 stc r5 -> --r7 [b7 9d] -- save dp on the IRQ stack. */
void step_irq0_save_dp(void)
{
    stc_r5_predec_r7();
    mcu.pc = 0x0524;
}

/* 0x0524 ldc #0,r5 [04 00 8d] -- dp = 0 for SRAM access. */
void step_irq0_dp0(void)
{
    ldc_dp0();
    mcu.pc = 0x0527;
}

/* 0x0527 stm #0x7f [12 7f] -- push r6..r0. */
void step_irq0_push_regs(void)
{
    stm_rlist(0x7f);
    mcu.pc = 0x0529;
}

/* 0x0529 mov.b (dp,0xe03e),r0 [15 e0 3e 80] -- PCM status. */
void step_irq0_read_status(void)
{
    load8(mcu.r[0], dp_addr(0xe03e));
    mcu.pc = 0x052d;
}

/* 0x052d and #0x001f,r0 [0c 00 1f 50] -- slot mask. */
void step_irq0_mask_status(void)
{
    mcu.r[0] = (uint16_t)(mcu.r[0] & 0x001fu);
    MCU_SetStatusCommon((uint32_t)mcu.r[0], 1);
    mcu.pc = 0x0531;
}

/* 0x0531 movg #0xff,@r0+0xd15c [f0 d1 5c 06 ff] -- mark the slot pending. */
void step_irq0_set_pending(void)
{
    store8(ind_addr(0, 0xd15c), 0xff);
    mcu.pc = 0x0536;
}

/* 0x0536 move r0,#0x02 [50 02] -- event queue arg 0. */
void step_irq0_arg0(void)
{
    move8(mcu.r[0], 0x02);
    mcu.pc = 0x0538;
}

/* 0x0538 move r1,#0x01 [51 01] -- event queue arg 1. */
void step_irq0_arg1(void)
{
    move8(mcu.r[1], 0x01);
    mcu.pc = 0x053a;
}

/* 0x053a ldc #0,r5 [04 00 8d] -- restore dp. */
void step_irq0_dp0_b(void)
{
    ldc_dp0();
    mcu.pc = 0x053d;
}

/* 0x053d jmp #0x03db [10 03 db] -- shared event epilogue (out of scope). */
void step_irq0_jump_epilogue(void)
{
    mcu.pc = 0x03db;
}

/* ---- C2 pcm_dispatcher 0x51cc..0x522a (rom1) ------------------------------- */

/* 0x51CC pjsr #0x04:1220 -- push 0x51d0,cp; A2 init_desc returns. */
void step_disp_call_init_desc(void)
{
    MCU_PushStack(0x51d0);
    MCU_PushStack(mcu.cp);
    mcu.cp = 0x04;
    mcu.pc = 0x1220; /* flat 0x041220 */
}

/* 0x51D0 move r0,#0x03 [50 03]. */
void step_disp_retry(void)
{
    move8(mcu.r[0], 0x03);
    mcu.pc = 0x51d2;
}

/* 0x51D2 trapa #0x10 [08 10] -- VM vector = imm & 0x0f = 0. */
void step_disp_trap(void)
{
    MCU_Interrupt_TRAPA(0x10 & 0x0f);
    mcu.pc = 0x51d4;
}

/* 0x51D4 cmp r0,b #0x00 [40 00]. */
void step_disp_test_trap(void)
{
    cmp_reg_imm8(mcu.r[0], 0x00);
    mcu.pc = 0x51d6;
}

/* 0x51D6 beq -> 0x51fc [37 00 23] -- trapa answered: scan the event queue. */
void step_disp_branch_play(void)
{
    beq(0x51fc, 0x51d9);
}

/* 0x51D9 movi r1,#0x001b [59 00 1b] -- pending-slot scan count. */
void step_disp_scan_pending(void)
{
    movi16(mcu.r[1], 0x001b);
    mcu.pc = 0x51dc;
}

/* 0x51DC bset_orc #0x0700,r0 [0c 07 00 48] -- raise IML, hold the poll. */
void step_disp_raise_iml(void)
{
    bset_orc_iml7();
    mcu.pc = 0x51e0;
}

/* 0x51E0 tst @r1+0xd15c [f1 d1 5c 16] -- pending slot? */
void step_disp_test_pending(void)
{
    tst8_mem(ind_addr(1, 0xd15c));
    mcu.pc = 0x51e4;
}

/* 0x51E4 bne -> 0x51ef [26 09] -- pending found: service it. */
void step_disp_branch_service(void)
{
    bne(0x51ef, 0x51e6);
}

/* 0x51E6 cntjmp r1,-9 -> 0x51e0 [01 b9 f7] -- next slot. */
void step_disp_loop_pending(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x51e0 : 0x51e9;
}

/* 0x51E9 bclr_andc #0xf8ff,r0 [0c f8 ff 58] -- lower IML. */
void step_disp_lower_iml(void)
{
    bclr_andc_iml0();
    mcu.pc = 0x51ed;
}

/* 0x51ED bra -> 0x51d0 [20 e1] -- retry the trap. */
void step_disp_retry_b(void)
{
    mcu.pc = 0x51d0;
}

/* 0x51EF clr @r1+0xd15c [f1 d1 5c 13] -- consume the pending slot. */
void step_disp_clear_pending(void)
{
    clr8_mem(ind_addr(1, 0xd15c));
    mcu.pc = 0x51f3;
}

/* 0x51F3 bra -> 0x25f0 [30 d3 fa] -- C5 irq_service (out of scope). */
void step_disp_branch_service2(void)
{
    mcu.pc = 0x25f0;
}

/* 0x51F6 bclr_andc #0xf8ff,r0 [0c f8 ff 58] -- C5 return path. */
void step_disp_return_iml(void)
{
    bclr_andc_iml0();
    mcu.pc = 0x51fa;
}

/* 0x51FA bra -> 0x51d9 [20 dd] -- rescan pending slots. */
void step_disp_rescan(void)
{
    mcu.pc = 0x51d9;
}

/* 0x51FC movi r1,#0x001b [59 00 1b] -- event scan count. */
void step_disp_scan_events(void)
{
    movi16(mcu.r[1], 0x001b);
    mcu.pc = 0x51ff;
}

/* 0x51FF clr (dp,0xd154) [1d d1 54 13]. */
void step_disp_clear_mask_a(void)
{
    clr16_mem(dp_addr(0xd154));
    mcu.pc = 0x5203;
}

/* 0x5203 clr (dp,0xd156) [1d d1 56 13]. */
void step_disp_clear_mask_b(void)
{
    clr16_mem(dp_addr(0xd156));
    mcu.pc = 0x5207;
}

/* 0x5207 bset_orc #0x0700,r0 [0c 07 00 48] -- IML=7 around the scan. */
void step_disp_raise_iml_b(void)
{
    bset_orc_iml7();
    mcu.pc = 0x520b;
}

/* 0x520B sub @r1+0xd0e0,#0x00 [f1 d0 e0 04 00] -- event pending? */
void step_disp_test_event(void)
{
    sub8_nowrite(ind_addr(1, 0xd0e0), 0x00);
    mcu.pc = 0x5210;
}

/* 0x5210 bne -> 0x521b [26 09] -- event found: dispatch it. */
void step_disp_branch_event(void)
{
    bne(0x521b, 0x5212);
}

/* 0x5212 cntjmp r1,-10 -> 0x520b [01 b9 f6] -- next event slot. */
void step_disp_loop_events(void)
{
    mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
    mcu.pc = (mcu.r[1] != 0xffff) ? 0x520b : 0x5215;
}

/* 0x5215 bclr_andc #0xf8ff,r0 [0c f8 ff 58] -- no event: lower IML. */
void step_disp_lower_iml_b(void)
{
    bclr_andc_iml0();
    mcu.pc = 0x5219;
}

/* 0x5219 bra -> 0x51d0 [20 b5] -- retry. */
void step_disp_retry_c(void)
{
    mcu.pc = 0x51d0;
}

/* 0x521B clr r2 [aa 13] -- high byte must be cleared or the 0x522c table
 * index keeps the previous jmp target page. */
void step_disp_clear_index(void)
{
    clr_reg_word(mcu.r[2]);
    mcu.pc = 0x521d;
}

/* 0x521D mov.b @r1+0xd0e0,r2 [f1 d0 e0 82] -- event code. */
void step_disp_load_event(void)
{
    load8(mcu.r[2], ind_addr(1, 0xd0e0));
    mcu.pc = 0x5221;
}

/* 0x5221 movg #0x00,@r1+0xd0e0 [f1 d0 e0 06 00] -- consume the event. */
void step_disp_clear_event(void)
{
    store8(ind_addr(1, 0xd0e0), 0x00);
    mcu.pc = 0x5226;
}

/* 0x5226 mov.w @r2+0x522c,r2 [fa 52 2c 82] -- handler address. */
void step_disp_load_handler(void)
{
    load16(mcu.r[2], ind_addr(2, 0x522c));
    mcu.pc = 0x522a;
}

/* 0x522A jmp r2 [11 d2] -> 0x5238 pcm_stop / 0x52cb pcm_play. */
void step_disp_jump_handler(void)
{
    mcu.pc = mcu.r[2];
}

/* ---- D1 voice_search 0x45cbe..0x45d99 (rom2; mcu.pc holds 0x5cbe..) -------- */

/* 0x5CBE clr r0 [a8 13] -- part cursor. */
void step_vs_clear_part(void)
{
    clr_reg_word(mcu.r[0]);
    mcu.pc = 0x5cc0;
}

/* 0x5CC0 clr r5 [a5 13] -- max activity. */
void step_vs_clear_max(void)
{
    clr_reg_byte(mcu.r[5]);
    mcu.pc = 0x5cc2;
}

/* 0x5CC2 mov.b @r0+0xa368,r6 [f0 a3 68 86] -- slot owner. */
void step_vs_load_owner(void)
{
    load8(mcu.r[6], ind_addr(0, 0xa368));
    mcu.pc = 0x5cc6;
}

/* 0x5CC6 cmp r6,r4 [a6 74] -- belongs to this part? */
void step_vs_compare_owner(void)
{
    MCU_SUB_Common((int32_t)(uint32_t)mcu.r[4],
                   (int32_t)(uint32_t)(mcu.r[6] & 0xff), 0, 0);
    mcu.pc = 0x5cc8;
}

/* 0x5CC8 bne -> 0x5cdc [26 12] -- other part: next slot. */
void step_vs_branch_other_part(void)
{
    bne(0x5cdc, 0x5cca);
}

/* 0x5CCA mov.b @r0+0xad0e,r6 [f0 ad 0e 86] -- slot activity. */
void step_vs_load_activity(void)
{
    load8(mcu.r[6], ind_addr(0, 0xad0e));
    mcu.pc = 0x5cce;
}

/* 0x5CCE cmp r6,b #0xff [46 ff]. */
void step_vs_compare_disabled(void)
{
    cmp_reg_imm8(mcu.r[6], 0xff);
    mcu.pc = 0x5cd0;
}

/* 0x5CD0 bne -> 0x5cd6 [26 04] -- active slot. */
void step_vs_branch_active(void)
{
    bne(0x5cd6, 0x5cd2);
}

/* 0x5CD2 clr r6 [a6 13] -- 0xff counts as 0. */
void step_vs_clear_disabled(void)
{
    clr_reg_byte(mcu.r[6]);
    mcu.pc = 0x5cd4;
}

/* 0x5CD4 bra -> 0x5cdc [20 06] -- next slot. */
void step_vs_branch_next(void)
{
    mcu.pc = 0x5cdc;
}

/* 0x5CD6 cmp r6,r5 [a6 75] -- activity > max? */
void step_vs_compare_max(void)
{
    MCU_SUB_Common((int32_t)(uint32_t)mcu.r[5],
                   (int32_t)(uint32_t)(mcu.r[6] & 0xff), 0, 0);
    mcu.pc = 0x5cd8;
}

/* 0x5CD8 bcc -> 0x5cdc [24 02] -- max unchanged. */
void step_vs_branch_not_max(void)
{
    bcc(0x5cdc, 0x5cda);
}

/* 0x5CDA mov.b r6,r5 [a6 85] -- new max. */
void step_vs_update_max(void)
{
    mov8_reg(mcu.r[5], mcu.r[6]);
    mcu.pc = 0x5cdc;
}

/* 0x5CDC addq #1,r0 [a8 08] -- next slot. */
void step_vs_next_slot(void)
{
    addq_word(mcu.r[0], 1);
    mcu.pc = 0x5cde;
}

/* 0x5CDE cmp r0,b #0x1c [40 1c] -- 28 slots scanned? */
void step_vs_compare_slots(void)
{
    cmp_reg_imm8(mcu.r[0], 0x1c);
    mcu.pc = 0x5ce0;
}

/* 0x5CE0 bne -> 0x5cc2 [26 e0] -- loop over slots. */
void step_vs_loop_slots(void)
{
    bne(0x5cc2, 0x5ce2);
}

/* 0x5CE2 mov.b r4,r0 [a4 80] -- part again. */
void step_vs_restore_part(void)
{
    mov8_reg(mcu.r[0], mcu.r[4]);
    mcu.pc = 0x5ce4;
}

/* 0x5CE4 extu r0 [a0 12]. */
void step_vs_zero_extend_part(void)
{
    extu(mcu.r[0]);
    mcu.pc = 0x5ce6;
}

/* 0x5CE6 mov.b @r0+0xdc76,r3 [f0 dc 76 83] -- part scale. */
void step_vs_load_scale(void)
{
    load8(mcu.r[3], ind_addr(0, 0xdc76));
    mcu.pc = 0x5cea;
}

/* 0x5CEA shll r3 [a3 1a]. */
void step_vs_double_scale(void)
{
    shll_byte(mcu.r[3]);
    mcu.pc = 0x5cec;
}

/* 0x5CEC mulxu r3,r5 [a3 ad] -- activity * scale. */
void step_vs_multiply_scale(void)
{
    mulxu_byte(mcu.r[3], mcu.r[5]);
    mcu.pc = 0x5cee;
}

/* 0x5CEE swap r5 [a5 10]. */
void step_vs_swap_product(void)
{
    swap_word(mcu.r[5]);
    mcu.pc = 0x5cf0;
}

/* 0x5CF0 shlr r5 [a5 1b]. */
void step_vs_shift_1(void)
{
    shlr_byte(mcu.r[5]);
    mcu.pc = 0x5cf2;
}

/* 0x5CF2 shlr r5 [a5 1b]. */
void step_vs_shift_2(void)
{
    shlr_byte(mcu.r[5]);
    mcu.pc = 0x5cf4;
}

/* 0x5CF4 shlr r5 [a5 1b]. */
void step_vs_shift_3(void)
{
    shlr_byte(mcu.r[5]);
    mcu.pc = 0x5cf6;
}

/* 0x5CF6 shlr r5 [a5 1b] -- scaled activity. */
void step_vs_shift_4(void)
{
    shlr_byte(mcu.r[5]);
    mcu.pc = 0x5cf8;
}

/* 0x5CF8 tst (dp,0xdcb4) [15 dc b4 16] -- mode gate. */
void step_vs_test_gate(void)
{
    tst8_mem(dp_addr(0xdcb4));
    mcu.pc = 0x5cfc;
}

/* 0x5CFC beq -> 0x5d2c [27 2e] -- store the value directly. */
void step_vs_branch_gate_off(void)
{
    beq(0x5d2c, 0x5cfe);
}

/* 0x5CFE btsti (dp,0xdc02),#3 [15 dc 02 f3]. */
void step_vs_test_bit3(void)
{
    btsti8_mem(dp_addr(0xdc02), 3);
    mcu.pc = 0x5d02;
}

/* 0x5D02 beq -> 0x5d14 [27 10] -- mix path. */
void step_vs_branch_bit3_clear(void)
{
    beq(0x5d14, 0x5d04);
}

/* 0x5D04 btsti (dp,0xdc02),#1 [15 dc 02 f1]. */
void step_vs_test_bit1(void)
{
    btsti8_mem(dp_addr(0xdc02), 1);
    mcu.pc = 0x5d08;
}

/* 0x5D08 bne -> 0x5d2c [26 22] -- no mix: store. */
void step_vs_branch_mix_done(void)
{
    bne(0x5d2c, 0x5d0a);
}

/* 0x5D0A mov.b (dp,0xdc2a),r6 [15 dc 2a 86] -- previous mix. */
void step_vs_load_prev(void)
{
    load8(mcu.r[6], dp_addr(0xdc2a));
    mcu.pc = 0x5d0e;
}

/* 0x5D0E cmp r6,r0 [a0 76]. */
void step_vs_compare_prev(void)
{
    MCU_SUB_Common((int32_t)(uint32_t)mcu.r[6],
                   (int32_t)(uint32_t)(mcu.r[0] & 0xff), 0, 0);
    mcu.pc = 0x5d10;
}

/* 0x5D10 beq -> 0x5d2c [27 1a] -- unchanged: store. */
void step_vs_branch_prev_same(void)
{
    beq(0x5d2c, 0x5d12);
}

/* 0x5D12 bra -> 0x5d26 [20 12] -- keep the value. */
void step_vs_branch_mix(void)
{
    mcu.pc = 0x5d26;
}

/* 0x5D14 add r0,r0 [a0 20] -- part * 2. */
void step_vs_double_index(void)
{
    add8_self(mcu.r[0]);
    mcu.pc = 0x5d16;
}

/* 0x5D16 extu r0 [a0 12]. */
void step_vs_zero_extend_index(void)
{
    extu(mcu.r[0]);
    mcu.pc = 0x5d18;
}

/* 0x5D18 add #0xdc36,r0 [0c dc 36 20] -- mix entry. */
void step_vs_index_mix(void)
{
    add16_imm(mcu.r[0], 0xdc36);
    mcu.pc = 0x5d1c;
}

/* 0x5D1C btsti @r0,#9 [d8 f9]. */
void step_vs_test_mix9(void)
{
    btsti16_mem(reg_addr(0), 9);
    mcu.pc = 0x5d1e;
}

/* 0x5D1E beq -> 0x5d26 [27 06]. */
void step_vs_branch_mix_off(void)
{
    beq(0x5d26, 0x5d20);
}

/* 0x5D20 btsti (dp,0xdc2b),#0 [15 dc 2b f0]. */
void step_vs_test_mixbit(void)
{
    btsti8_mem(dp_addr(0xdc2b), 0);
    mcu.pc = 0x5d24;
}

/* 0x5D24 beq -> 0x5d2c [27 06] -- keep the scaled value. */
void step_vs_branch_keep(void)
{
    beq(0x5d2c, 0x5d26);
}

/* 0x5D26 tst r5 [a5 16]. */
void step_vs_test_max(void)
{
    tst8_reg(mcu.r[5]);
    mcu.pc = 0x5d28;
}

/* 0x5D28 bne -> 0x5d2c [26 02]. */
void step_vs_branch_max_set(void)
{
    bne(0x5d2c, 0x5d2a);
}

/* 0x5D2A move r5,#0xff [55 ff] -- idle marker. */
void step_vs_set_max_ff(void)
{
    move8(mcu.r[5], 0xff);
    mcu.pc = 0x5d2c;
}

/* 0x5D2C mov.b r5,r1++ [c1 95] -- store per-part value. */
void step_vs_store_mix(void)
{
    store8_postinc(1, mcu.r[5]);
    mcu.pc = 0x5d2e;
}

/* 0x5D2E addq #1,r2 [a2 08] -- next part. */
void step_vs_next_entry(void)
{
    addq_byte_reg(mcu.r[2], 1);
    mcu.pc = 0x5d30;
}

/* 0x5D30 cmp r2,b #0x10 [42 10] -- 16 parts? */
void step_vs_compare_entries(void)
{
    cmp_reg_imm8(mcu.r[2], 0x10);
    mcu.pc = 0x5d32;
}

/* 0x5D32 bne -> 0x45c7c (16-bit disp; outside this fragment). */
void step_vs_loop_entries(void)
{
    bne(0x5c7c, 0x5d35);
}

/* 0x5D35 tst (dp,0xd6a7) [15 d6 a7 16] -- mix update gate. */
void step_vs_test_counter(void)
{
    tst8_mem(dp_addr(0xd6a7));
    mcu.pc = 0x5d39;
}

/* 0x5D39 beq -> 0x5d99 [27 5e] -- gate off: return. */
void step_vs_branch_counter_zero(void)
{
    beq(0x5d99, 0x5d3b);
}

/* 0x5D3B movi r0,#0xd2ee [58 d2 ee] -- mix table bases. */
void step_vs_init_ptr0(void)
{
    movi16(mcu.r[0], 0xd2ee);
    mcu.pc = 0x5d3e;
}

/* 0x5D3E movi r1,#0xd433 [59 d4 33]. */
void step_vs_init_ptr1(void)
{
    movi16(mcu.r[1], 0xd433);
    mcu.pc = 0x5d41;
}

/* 0x5D41 movi r2,#0xd378 [5a d3 78]. */
void step_vs_init_ptr2(void)
{
    movi16(mcu.r[2], 0xd378);
    mcu.pc = 0x5d44;
}

/* 0x5D44 movi r3,#0x000f [5b 00 0f] -- 16 mix entries. */
void step_vs_init_count(void)
{
    movi16(mcu.r[3], 0x000f);
    mcu.pc = 0x5d47;
}

/* 0x5D47 mov.w r3,--r7 [bf 93] -- save the count. */
void step_vs_push_count(void)
{
    store16_predec(7, mcu.r[3]);
    mcu.pc = 0x5d49;
}

/* 0x5D49 add #0xd2bf,r3 [0c d2 bf 23] -- entry pointer. */
void step_vs_base_ptr(void)
{
    add16_imm(mcu.r[3], 0xd2bf);
    mcu.pc = 0x5d4d;
}

/* 0x5D4D mov.b @r0,r4 [d0 84] -- left value. */
void step_vs_load_left(void)
{
    load8(mcu.r[4], reg_addr(0));
    mcu.pc = 0x5d4f;
}

/* 0x5D4F mov.b @r2,r6 [d2 86] -- right value. */
void step_vs_load_right(void)
{
    load8(mcu.r[6], reg_addr(2));
    mcu.pc = 0x5d51;
}

/* 0x5D51 cmp r4,r6 [a4 76]. */
void step_vs_compare_pair(void)
{
    MCU_SUB_Common((int32_t)(uint32_t)mcu.r[6],
                   (int32_t)(uint32_t)(mcu.r[4] & 0xff), 0, 0);
    mcu.pc = 0x5d53;
}

/* 0x5D53 bpl -> 0x5d5e [2a 09] -- right >= left: check the age. */
void step_vs_branch_pair_ge(void)
{
    bpl(0x5d5e, 0x5d55);
}

/* 0x5D55 mov.b r4,@r2 [d2 94] -- right = left. */
void step_vs_store_right(void)
{
    store8(reg_addr(2), (uint8_t)mcu.r[4]);
    mcu.pc = 0x5d57;
}

/* 0x5D57 mov.b r4,@r3 [d3 94] -- base = left. */
void step_vs_store_base(void)
{
    store8(reg_addr(3), (uint8_t)mcu.r[4]);
    mcu.pc = 0x5d59;
}

/* 0x5D59 movg #0x64,@r1 [d1 06 64] -- reset the age. */
void step_vs_store_age(void)
{
    store8(reg_addr(1), 0x64);
    mcu.pc = 0x5d5c;
}

/* 0x5D5C bra -> 0x5d8e [20 30]. */
void step_vs_branch_dec(void)
{
    mcu.pc = 0x5d8e;
}

/* 0x5D5E tst @r1 [d1 16] -- age. */
void step_vs_test_age(void)
{
    tst8_mem(reg_addr(1));
    mcu.pc = 0x5d60;
}

/* 0x5D60 bne -> 0x5d8e [26 2c] -- age still set. */
void step_vs_branch_age_set(void)
{
    bne(0x5d8e, 0x5d62);
}

/* 0x5D62 sub (dp,0xd6a7),#0x02 [15 d6 a7 04 02] -- gate == 2? */
void step_vs_compare_two(void)
{
    sub8_nowrite(dp_addr(0xd6a7), 0x02);
    mcu.pc = 0x5d67;
}

/* 0x5D67 bne -> 0x5d71 [26 08]. */
void step_vs_branch_not_two(void)
{
    bne(0x5d71, 0x5d69);
}

/* 0x5D69 movg #0x00,@r2 [d2 06 00] -- clear right. */
void step_vs_clear_right(void)
{
    store8(reg_addr(2), 0x00);
    mcu.pc = 0x5d6c;
}

/* 0x5D6C movg #0x00,@r3 [d3 06 00] -- clear base. */
void step_vs_clear_base(void)
{
    store8(reg_addr(3), 0x00);
    mcu.pc = 0x5d6f;
}

/* 0x5D6F bra -> 0x5d8e [20 1a]. */
void step_vs_branch_dec2(void)
{
    mcu.pc = 0x5d8e;
}

/* 0x5D71 tst r6 [a6 16]. */
void step_vs_test_right(void)
{
    tst8_reg(mcu.r[6]);
    mcu.pc = 0x5d73;
}

/* 0x5D73 beq -> 0x5d79 [27 04]. */
void step_vs_branch_right_zero(void)
{
    beq(0x5d79, 0x5d75);
}

/* 0x5D75 addq #-1,r6 (byte) [a6 0c] -- decay. */
void step_vs_dec_left(void)
{
    addq_byte_reg(mcu.r[6], -1);
    mcu.pc = 0x5d77;
}

/* 0x5D77 mov.b r6,@r2 [d2 96]. */
void step_vs_store_left(void)
{
    store8(reg_addr(2), (uint8_t)mcu.r[6]);
    mcu.pc = 0x5d79;
}

/* 0x5D79 sub (dp,0xd6a7),#0x03 [15 d6 a7 04 03] -- gate == 3? */
void step_vs_compare_three(void)
{
    sub8_nowrite(dp_addr(0xd6a7), 0x03);
    mcu.pc = 0x5d7e;
}

/* 0x5D7E bne -> 0x5d89 [26 09]. */
void step_vs_branch_not_three(void)
{
    bne(0x5d89, 0x5d80);
}

/* 0x5D80 sub @r3,#0x10 [d3 04 10] -- at the limit? */
void step_vs_compare_limit(void)
{
    sub8_nowrite(reg_addr(3), 0x10);
    mcu.pc = 0x5d83;
}

/* 0x5D83 beq -> 0x5d8b [27 06] -- saturated. */
void step_vs_branch_limit(void)
{
    beq(0x5d8b, 0x5d85);
}

/* 0x5D85 addq #1,@r3 [d3 08] -- ramp the base. */
void step_vs_inc_base(void)
{
    addq_byte_mem(reg_addr(3), 1);
    mcu.pc = 0x5d87;
}

/* 0x5D87 bra -> 0x5d8b [20 02]. */
void step_vs_branch_skip(void)
{
    mcu.pc = 0x5d8b;
}

/* 0x5D89 mov.b r6,@r3 [d3 96] -- base = right. */
void step_vs_store_base_r6(void)
{
    store8(reg_addr(3), (uint8_t)mcu.r[6]);
    mcu.pc = 0x5d8b;
}

/* 0x5D8B movg #0x14,@r1 [d1 06 14] -- reload the age. */
void step_vs_store_age2(void)
{
    store8(reg_addr(1), 0x14);
    mcu.pc = 0x5d8e;
}

/* 0x5D8E addq #-1,r0 [a8 0c] -- next entry. */
void step_vs_dec_ptr0(void)
{
    addq_word(mcu.r[0], -1);
    mcu.pc = 0x5d90;
}

/* 0x5D90 addq #-1,r1 [a9 0c]. */
void step_vs_dec_ptr1(void)
{
    addq_word(mcu.r[1], -1);
    mcu.pc = 0x5d92;
}

/* 0x5D92 addq #-1,r2 [aa 0c]. */
void step_vs_dec_ptr2(void)
{
    addq_word(mcu.r[2], -1);
    mcu.pc = 0x5d94;
}

/* 0x5D94 mov.w r7++,r3 [cf 83] -- pop the count. */
void step_vs_pop_count(void)
{
    load16_postinc(7, mcu.r[3]);
    mcu.pc = 0x5d96;
}

/* 0x5D96 cntjmp r3,-82 -> 0x5d47 [01 bb ae] -- loop the mix entries. */
void step_vs_loop_mix(void)
{
    mcu.r[3] = (uint16_t)(mcu.r[3] - 1);
    mcu.pc = (mcu.r[3] != 0xffff) ? 0x5d47 : 0x5d99;
}

/* 0x5D99 rts [19]. */
void step_vs_return(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- registration (one named L0 entry per PC) ------------------------------ */

/* Self-registration hook (hand_registry.h): called from
 * MK2CPP_HandFillTables after the built-in modules. */
void pcm_dispatch_fill(void)
{
    /* IRQ0 0x0522..0x053D (rom1). */
    MK2CPP_HandRegister(0x00000522u, &step_irq0_save_dp);
    MK2CPP_HandRegister(0x00000524u, &step_irq0_dp0);
    MK2CPP_HandRegister(0x00000527u, &step_irq0_push_regs);
    MK2CPP_HandRegister(0x00000529u, &step_irq0_read_status);
    MK2CPP_HandRegister(0x0000052du, &step_irq0_mask_status);
    MK2CPP_HandRegister(0x00000531u, &step_irq0_set_pending);
    MK2CPP_HandRegister(0x00000536u, &step_irq0_arg0);
    MK2CPP_HandRegister(0x00000538u, &step_irq0_arg1);
    MK2CPP_HandRegister(0x0000053au, &step_irq0_dp0_b);
    MK2CPP_HandRegister(0x0000053du, &step_irq0_jump_epilogue);

    /* C2 pcm_dispatcher 0x51CC..0x522A (rom1). */
    MK2CPP_HandRegister(0x000051ccu, &step_disp_call_init_desc);
    MK2CPP_HandRegister(0x000051d0u, &step_disp_retry);
    MK2CPP_HandRegister(0x000051d2u, &step_disp_trap);
    MK2CPP_HandRegister(0x000051d4u, &step_disp_test_trap);
    MK2CPP_HandRegister(0x000051d6u, &step_disp_branch_play);
    MK2CPP_HandRegister(0x000051d9u, &step_disp_scan_pending);
    MK2CPP_HandRegister(0x000051dcu, &step_disp_raise_iml);
    MK2CPP_HandRegister(0x000051e0u, &step_disp_test_pending);
    MK2CPP_HandRegister(0x000051e4u, &step_disp_branch_service);
    MK2CPP_HandRegister(0x000051e6u, &step_disp_loop_pending);
    MK2CPP_HandRegister(0x000051e9u, &step_disp_lower_iml);
    MK2CPP_HandRegister(0x000051edu, &step_disp_retry_b);
    MK2CPP_HandRegister(0x000051efu, &step_disp_clear_pending);
    MK2CPP_HandRegister(0x000051f3u, &step_disp_branch_service2);
    MK2CPP_HandRegister(0x000051f6u, &step_disp_return_iml);
    MK2CPP_HandRegister(0x000051fau, &step_disp_rescan);
    MK2CPP_HandRegister(0x000051fcu, &step_disp_scan_events);
    MK2CPP_HandRegister(0x000051ffu, &step_disp_clear_mask_a);
    MK2CPP_HandRegister(0x00005203u, &step_disp_clear_mask_b);
    MK2CPP_HandRegister(0x00005207u, &step_disp_raise_iml_b);
    MK2CPP_HandRegister(0x0000520bu, &step_disp_test_event);
    MK2CPP_HandRegister(0x00005210u, &step_disp_branch_event);
    MK2CPP_HandRegister(0x00005212u, &step_disp_loop_events);
    MK2CPP_HandRegister(0x00005215u, &step_disp_lower_iml_b);
    MK2CPP_HandRegister(0x00005219u, &step_disp_retry_c);
    MK2CPP_HandRegister(0x0000521bu, &step_disp_clear_index);
    MK2CPP_HandRegister(0x0000521du, &step_disp_load_event);
    MK2CPP_HandRegister(0x00005221u, &step_disp_clear_event);
    MK2CPP_HandRegister(0x00005226u, &step_disp_load_handler);
    MK2CPP_HandRegister(0x0000522au, &step_disp_jump_handler);

    /* D1 voice_search 0x45CBE..0x45D99 (rom2, cp=4). */
    MK2CPP_HandRegister(0x00045cbeu, &step_vs_clear_part);
    MK2CPP_HandRegister(0x00045cc0u, &step_vs_clear_max);
    MK2CPP_HandRegister(0x00045cc2u, &step_vs_load_owner);
    MK2CPP_HandRegister(0x00045cc6u, &step_vs_compare_owner);
    MK2CPP_HandRegister(0x00045cc8u, &step_vs_branch_other_part);
    MK2CPP_HandRegister(0x00045ccau, &step_vs_load_activity);
    MK2CPP_HandRegister(0x00045cceu, &step_vs_compare_disabled);
    MK2CPP_HandRegister(0x00045cd0u, &step_vs_branch_active);
    MK2CPP_HandRegister(0x00045cd2u, &step_vs_clear_disabled);
    MK2CPP_HandRegister(0x00045cd4u, &step_vs_branch_next);
    MK2CPP_HandRegister(0x00045cd6u, &step_vs_compare_max);
    MK2CPP_HandRegister(0x00045cd8u, &step_vs_branch_not_max);
    MK2CPP_HandRegister(0x00045cdau, &step_vs_update_max);
    MK2CPP_HandRegister(0x00045cdcu, &step_vs_next_slot);
    MK2CPP_HandRegister(0x00045cdeu, &step_vs_compare_slots);
    MK2CPP_HandRegister(0x00045ce0u, &step_vs_loop_slots);
    MK2CPP_HandRegister(0x00045ce2u, &step_vs_restore_part);
    MK2CPP_HandRegister(0x00045ce4u, &step_vs_zero_extend_part);
    MK2CPP_HandRegister(0x00045ce6u, &step_vs_load_scale);
    MK2CPP_HandRegister(0x00045ceau, &step_vs_double_scale);
    MK2CPP_HandRegister(0x00045cecu, &step_vs_multiply_scale);
    MK2CPP_HandRegister(0x00045ceeu, &step_vs_swap_product);
    MK2CPP_HandRegister(0x00045cf0u, &step_vs_shift_1);
    MK2CPP_HandRegister(0x00045cf2u, &step_vs_shift_2);
    MK2CPP_HandRegister(0x00045cf4u, &step_vs_shift_3);
    MK2CPP_HandRegister(0x00045cf6u, &step_vs_shift_4);
    MK2CPP_HandRegister(0x00045cf8u, &step_vs_test_gate);
    MK2CPP_HandRegister(0x00045cfcu, &step_vs_branch_gate_off);
    MK2CPP_HandRegister(0x00045cfeu, &step_vs_test_bit3);
    MK2CPP_HandRegister(0x00045d02u, &step_vs_branch_bit3_clear);
    MK2CPP_HandRegister(0x00045d04u, &step_vs_test_bit1);
    MK2CPP_HandRegister(0x00045d08u, &step_vs_branch_mix_done);
    MK2CPP_HandRegister(0x00045d0au, &step_vs_load_prev);
    MK2CPP_HandRegister(0x00045d0eu, &step_vs_compare_prev);
    MK2CPP_HandRegister(0x00045d10u, &step_vs_branch_prev_same);
    MK2CPP_HandRegister(0x00045d12u, &step_vs_branch_mix);
    MK2CPP_HandRegister(0x00045d14u, &step_vs_double_index);
    MK2CPP_HandRegister(0x00045d16u, &step_vs_zero_extend_index);
    MK2CPP_HandRegister(0x00045d18u, &step_vs_index_mix);
    MK2CPP_HandRegister(0x00045d1cu, &step_vs_test_mix9);
    MK2CPP_HandRegister(0x00045d1eu, &step_vs_branch_mix_off);
    MK2CPP_HandRegister(0x00045d20u, &step_vs_test_mixbit);
    MK2CPP_HandRegister(0x00045d24u, &step_vs_branch_keep);
    MK2CPP_HandRegister(0x00045d26u, &step_vs_test_max);
    MK2CPP_HandRegister(0x00045d28u, &step_vs_branch_max_set);
    MK2CPP_HandRegister(0x00045d2au, &step_vs_set_max_ff);
    MK2CPP_HandRegister(0x00045d2cu, &step_vs_store_mix);
    MK2CPP_HandRegister(0x00045d2eu, &step_vs_next_entry);
    MK2CPP_HandRegister(0x00045d30u, &step_vs_compare_entries);
    MK2CPP_HandRegister(0x00045d32u, &step_vs_loop_entries);
    MK2CPP_HandRegister(0x00045d35u, &step_vs_test_counter);
    MK2CPP_HandRegister(0x00045d39u, &step_vs_branch_counter_zero);
    MK2CPP_HandRegister(0x00045d3bu, &step_vs_init_ptr0);
    MK2CPP_HandRegister(0x00045d3eu, &step_vs_init_ptr1);
    MK2CPP_HandRegister(0x00045d41u, &step_vs_init_ptr2);
    MK2CPP_HandRegister(0x00045d44u, &step_vs_init_count);
    MK2CPP_HandRegister(0x00045d47u, &step_vs_push_count);
    MK2CPP_HandRegister(0x00045d49u, &step_vs_base_ptr);
    MK2CPP_HandRegister(0x00045d4du, &step_vs_load_left);
    MK2CPP_HandRegister(0x00045d4fu, &step_vs_load_right);
    MK2CPP_HandRegister(0x00045d51u, &step_vs_compare_pair);
    MK2CPP_HandRegister(0x00045d53u, &step_vs_branch_pair_ge);
    MK2CPP_HandRegister(0x00045d55u, &step_vs_store_right);
    MK2CPP_HandRegister(0x00045d57u, &step_vs_store_base);
    MK2CPP_HandRegister(0x00045d59u, &step_vs_store_age);
    MK2CPP_HandRegister(0x00045d5cu, &step_vs_branch_dec);
    MK2CPP_HandRegister(0x00045d5eu, &step_vs_test_age);
    MK2CPP_HandRegister(0x00045d60u, &step_vs_branch_age_set);
    MK2CPP_HandRegister(0x00045d62u, &step_vs_compare_two);
    MK2CPP_HandRegister(0x00045d67u, &step_vs_branch_not_two);
    MK2CPP_HandRegister(0x00045d69u, &step_vs_clear_right);
    MK2CPP_HandRegister(0x00045d6cu, &step_vs_clear_base);
    MK2CPP_HandRegister(0x00045d6fu, &step_vs_branch_dec2);
    MK2CPP_HandRegister(0x00045d71u, &step_vs_test_right);
    MK2CPP_HandRegister(0x00045d73u, &step_vs_branch_right_zero);
    MK2CPP_HandRegister(0x00045d75u, &step_vs_dec_left);
    MK2CPP_HandRegister(0x00045d77u, &step_vs_store_left);
    MK2CPP_HandRegister(0x00045d79u, &step_vs_compare_three);
    MK2CPP_HandRegister(0x00045d7eu, &step_vs_branch_not_three);
    MK2CPP_HandRegister(0x00045d80u, &step_vs_compare_limit);
    MK2CPP_HandRegister(0x00045d83u, &step_vs_branch_limit);
    MK2CPP_HandRegister(0x00045d85u, &step_vs_inc_base);
    MK2CPP_HandRegister(0x00045d87u, &step_vs_branch_skip);
    MK2CPP_HandRegister(0x00045d89u, &step_vs_store_base_r6);
    MK2CPP_HandRegister(0x00045d8bu, &step_vs_store_age2);
    MK2CPP_HandRegister(0x00045d8eu, &step_vs_dec_ptr0);
    MK2CPP_HandRegister(0x00045d90u, &step_vs_dec_ptr1);
    MK2CPP_HandRegister(0x00045d92u, &step_vs_dec_ptr2);
    MK2CPP_HandRegister(0x00045d94u, &step_vs_pop_count);
    MK2CPP_HandRegister(0x00045d96u, &step_vs_loop_mix);
    MK2CPP_HandRegister(0x00045d99u, &step_vs_return);
}

} /* anonymous namespace */
} /* namespace mk2c */

namespace {

struct PcmDispatchRegistration
{
    PcmDispatchRegistration()
    {
        mk2c::hand_register_module(&mk2c::pcm_dispatch_fill);
    }
};

PcmDispatchRegistration g_pcm_dispatch_registration;

} /* anonymous namespace */
