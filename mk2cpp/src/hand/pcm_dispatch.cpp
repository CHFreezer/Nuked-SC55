/*
 * HAND pcm_dispatch -- voice closure P0: IRQ0, C2 pcm_dispatcher, D1
 * voice_search, one host step per H8 instruction (M4 wave, out/m4/18 4.2 P0).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * Three P0 routines from mk2cpp/out/m4/18_closure_gap.md 4.2 are translated
 * instruction by instruction. Every instruction PC is one
 * MK2CPP_HandRegisterRoutine entry returning 1, so the host keeps its
 * per-instruction interrupt poll, TIMER_Clock, trace and SM_Update cadence.
 * A multi-instruction block would defer the host tail and let -midiseq /
 * real-time MIDI bytes queued inside the block be consumed by the SM at the
 * block end instead of at each instruction boundary (same timing drift that
 * made the earlier slices per-PC).
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

/* Safety net: a PC missing from the case tables is executed by the stock
 * interpreter for that one instruction (should never happen). */
void stock_instruction(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
}

/* ---- IRQ0 0x0522..0x053d (rom1) ------------------------------------------- */

uint32_t step_irq0(void)
{
    switch (mcu.pc)
    {
    case 0x0522: /* STC r5 -> --r7  [b7 9d] */
        stc_r5_predec_r7();
        mcu.pc = 0x0524;
        return 1;
    case 0x0524: /* LDC #0x00 r5  [04 00 8d] -> dp = 0 */
        ldc_dp0();
        mcu.pc = 0x0527;
        return 1;
    case 0x0527: /* stm #0x7f (push r6..r0)  [12 7f] */
        stm_rlist(0x7f);
        mcu.pc = 0x0529;
        return 1;
    case 0x0529: /* MOVG2 (dp,0xe03e) r0 -> PCM status  [15 e0 3e 80] */
        load8(mcu.r[0], dp_addr(0xe03e));
        mcu.pc = 0x052d;
        return 1;
    case 0x052d: /* AND #0x001f r0  [0c 00 1f 50] */
        mcu.r[0] = (uint16_t)(mcu.r[0] & 0x001fu);
        MCU_SetStatusCommon((uint32_t)mcu.r[0], 1);
        mcu.pc = 0x0531;
        return 1;
    case 0x0531: /* MOVG #0xff -> @r0+0xd15c  [f0 d1 5c 06 ff] */
        store8(ind_addr(0, 0xd15c), 0xff);
        mcu.pc = 0x0536;
        return 1;
    case 0x0536: /* move r0 #0x02  [50 02] */
        move8(mcu.r[0], 0x02);
        mcu.pc = 0x0538;
        return 1;
    case 0x0538: /* move r1 #0x01  [51 01] */
        move8(mcu.r[1], 0x01);
        mcu.pc = 0x053a;
        return 1;
    case 0x053a: /* LDC #0x00 r5  [04 00 8d] */
        ldc_dp0();
        mcu.pc = 0x053d;
        return 1;
    case 0x053d: /* jmp #0x03db  [10 03 db] (shared event epilogue, out of scope) */
        mcu.pc = 0x03db;
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- C2 pcm_dispatcher 0x51cc..0x522a (rom1) ------------------------------- */

uint32_t step_pcm_dispatcher(void)
{
    switch (mcu.pc)
    {
    case 0x51cc: /* pjsr #0x04:1220 -- push 0x51d0,cp; A2 init_desc returns */
        MCU_PushStack(0x51d0);
        MCU_PushStack(mcu.cp);
        mcu.cp = 0x04;
        mcu.pc = 0x1220; /* flat 0x041220 */
        return 1;
    case 0x51d0: /* move r0 #0x03  [50 03] */
        move8(mcu.r[0], 0x03);
        mcu.pc = 0x51d2;
        return 1;
    case 0x51d2: /* trapa #0x10  [08 10] (VM vector = imm & 0x0f = 0) */
        MCU_Interrupt_TRAPA(0x10 & 0x0f);
        mcu.pc = 0x51d4;
        return 1;
    case 0x51d4: /* cmp r0,b #0x00  [40 00] */
        cmp_reg_imm8(mcu.r[0], 0x00);
        mcu.pc = 0x51d6;
        return 1;
    case 0x51d6: /* BEQ 0x51fc (16-bit disp)  [37 00 23] */
        beq(0x51fc, 0x51d9);
        return 1;
    case 0x51d9: /* movi r1 #0x001b  [59 00 1b] */
        movi16(mcu.r[1], 0x001b);
        mcu.pc = 0x51dc;
        return 1;
    case 0x51dc: /* BSET_ORC #0x0700 r0  [0c 07 00 48] */
        bset_orc_iml7();
        mcu.pc = 0x51e0;
        return 1;
    case 0x51e0: /* TST @r1+0xd15c (byte)  [f1 d1 5c 16] */
        tst8_mem(ind_addr(1, 0xd15c));
        mcu.pc = 0x51e4;
        return 1;
    case 0x51e4: /* BNE 0x51ef  [26 09] */
        bne(0x51ef, 0x51e6);
        return 1;
    case 0x51e6: /* cntjmp r1 -9  [01 b9 f7] */
        mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
        mcu.pc = (mcu.r[1] != 0xffff) ? 0x51e0 : 0x51e9;
        return 1;
    case 0x51e9: /* BCLR_ANDC #0xf8ff r0  [0c f8 ff 58] */
        bclr_andc_iml0();
        mcu.pc = 0x51ed;
        return 1;
    case 0x51ed: /* BRA 0x51d0  [20 e1] */
        mcu.pc = 0x51d0;
        return 1;
    case 0x51ef: /* CLR @r1+0xd15c (byte)  [f1 d1 5c 13] */
        clr8_mem(ind_addr(1, 0xd15c));
        mcu.pc = 0x51f3;
        return 1;
    case 0x51f3: /* BRA 0x25f0 (C5 irq_service, out of scope)  [30 d3 fa] */
        mcu.pc = 0x25f0;
        return 1;
    case 0x51f6: /* BCLR_ANDC #0xf8ff r0  [0c f8 ff 58] */
        bclr_andc_iml0();
        mcu.pc = 0x51fa;
        return 1;
    case 0x51fa: /* BRA 0x51d9  [20 dd] */
        mcu.pc = 0x51d9;
        return 1;
    case 0x51fc: /* movi r1 #0x001b  [59 00 1b] */
        movi16(mcu.r[1], 0x001b);
        mcu.pc = 0x51ff;
        return 1;
    case 0x51ff: /* CLR (dp,0xd154) (word)  [1d d1 54 13] */
        clr16_mem(dp_addr(0xd154));
        mcu.pc = 0x5203;
        return 1;
    case 0x5203: /* CLR (dp,0xd156) (word)  [1d d1 56 13] */
        clr16_mem(dp_addr(0xd156));
        mcu.pc = 0x5207;
        return 1;
    case 0x5207: /* BSET_ORC #0x0700 r0  [0c 07 00 48] */
        bset_orc_iml7();
        mcu.pc = 0x520b;
        return 1;
    case 0x520b: /* SUB @r1+0xd0e0 #0x00 (flags only)  [f1 d0 e0 04 00] */
        sub8_nowrite(ind_addr(1, 0xd0e0), 0x00);
        mcu.pc = 0x5210;
        return 1;
    case 0x5210: /* BNE 0x521b  [26 09] */
        bne(0x521b, 0x5212);
        return 1;
    case 0x5212: /* cntjmp r1 -10  [01 b9 f6] */
        mcu.r[1] = (uint16_t)(mcu.r[1] - 1);
        mcu.pc = (mcu.r[1] != 0xffff) ? 0x520b : 0x5215;
        return 1;
    case 0x5215: /* BCLR_ANDC #0xf8ff r0  [0c f8 ff 58] */
        bclr_andc_iml0();
        mcu.pc = 0x5219;
        return 1;
    case 0x5219: /* BRA 0x51d0  [20 b5] */
        mcu.pc = 0x51d0;
        return 1;
    case 0x521b: /* CLR r2 (word)  [aa 13] -- high byte must be cleared or the
                  * 0x522c table index keeps the previous jmp target page */
        clr_reg_word(mcu.r[2]);
        mcu.pc = 0x521d;
        return 1;
    case 0x521d: /* MOVG2 @r1+0xd0e0 r2 (byte)  [f1 d0 e0 82] */
        load8(mcu.r[2], ind_addr(1, 0xd0e0));
        mcu.pc = 0x5221;
        return 1;
    case 0x5221: /* MOVG #0x00 -> @r1+0xd0e0  [f1 d0 e0 06 00] */
        store8(ind_addr(1, 0xd0e0), 0x00);
        mcu.pc = 0x5226;
        return 1;
    case 0x5226: /* MOVG2 @r2+0x522c r2 (word)  [fa 52 2c 82] */
        load16(mcu.r[2], ind_addr(2, 0x522c));
        mcu.pc = 0x522a;
        return 1;
    case 0x522a: /* jmp r2  [11 d2] -> 0x5238 pcm_stop / 0x52cb pcm_play */
        mcu.pc = mcu.r[2];
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- D1 voice_search 0x45cbe..0x45d99 (rom2; mcu.pc holds 0x5cbe..) -------- */

uint32_t step_voice_search(void)
{
    switch (mcu.pc)
    {
    case 0x5cbe: /* 00045cbe CLR r0 (word)  [a8 13] */
        clr_reg_word(mcu.r[0]);
        mcu.pc = 0x5cc0;
        return 1;
    case 0x5cc0: /* 00045cc0 CLR r5 (byte)  [a5 13] */
        clr_reg_byte(mcu.r[5]);
        mcu.pc = 0x5cc2;
        return 1;
    case 0x5cc2: /* 00045cc2 MOVG2 @r0+0xa368 r6 (byte)  [f0 a3 68 86] */
        load8(mcu.r[6], ind_addr(0, 0xa368));
        mcu.pc = 0x5cc6;
        return 1;
    case 0x5cc6: /* 00045cc6 CMP r6 r4  [a6 74] t1=r4, t2=r6.lo */
        MCU_SUB_Common((int32_t)(uint32_t)mcu.r[4],
                       (int32_t)(uint32_t)(mcu.r[6] & 0xff), 0, 0);
        mcu.pc = 0x5cc8;
        return 1;
    case 0x5cc8: /* 00045cc8 BNE 0x5cdc  [26 12] */
        bne(0x5cdc, 0x5cca);
        return 1;
    case 0x5cca: /* 00045cca MOVG2 @r0+0xad0e r6 (byte)  [f0 ad 0e 86] */
        load8(mcu.r[6], ind_addr(0, 0xad0e));
        mcu.pc = 0x5cce;
        return 1;
    case 0x5cce: /* 00045cce cmp r6,b #0xff  [46 ff] */
        cmp_reg_imm8(mcu.r[6], 0xff);
        mcu.pc = 0x5cd0;
        return 1;
    case 0x5cd0: /* 00045cd0 BNE 0x5cd6  [26 04] */
        bne(0x5cd6, 0x5cd2);
        return 1;
    case 0x5cd2: /* 00045cd2 CLR r6 (byte)  [a6 13] (0xff -> 0) */
        clr_reg_byte(mcu.r[6]);
        mcu.pc = 0x5cd4;
        return 1;
    case 0x5cd4: /* 00045cd4 BRA 0x5cdc  [20 06] */
        mcu.pc = 0x5cdc;
        return 1;
    case 0x5cd6: /* 00045cd6 CMP r6 r5  [a6 75] t1=r5, t2=r6.lo */
        MCU_SUB_Common((int32_t)(uint32_t)mcu.r[5],
                       (int32_t)(uint32_t)(mcu.r[6] & 0xff), 0, 0);
        mcu.pc = 0x5cd8;
        return 1;
    case 0x5cd8: /* 00045cd8 BCC 0x5cdc  [24 02] */
        bcc(0x5cdc, 0x5cda);
        return 1;
    case 0x5cda: /* 00045cda MOVG2 r6 r5 (byte)  [a6 85] (max) */
        mov8_reg(mcu.r[5], mcu.r[6]);
        mcu.pc = 0x5cdc;
        return 1;
    case 0x5cdc: /* 00045cdc ADDQ #1 r0 (word)  [a8 08] */
        addq_word(mcu.r[0], 1);
        mcu.pc = 0x5cde;
        return 1;
    case 0x5cde: /* 00045cde cmp r0,b #0x1c  [40 1c] */
        cmp_reg_imm8(mcu.r[0], 0x1c);
        mcu.pc = 0x5ce0;
        return 1;
    case 0x5ce0: /* 00045ce0 BNE 0x5cc2  [26 e0] */
        bne(0x5cc2, 0x5ce2);
        return 1;
    case 0x5ce2: /* 00045ce2 MOVG2 r4 r0 (byte)  [a4 80] */
        mov8_reg(mcu.r[0], mcu.r[4]);
        mcu.pc = 0x5ce4;
        return 1;
    case 0x5ce4: /* 00045ce4 EXTU r0  [a0 12] */
        extu(mcu.r[0]);
        mcu.pc = 0x5ce6;
        return 1;
    case 0x5ce6: /* 00045ce6 MOVG2 @r0+0xdc76 r3 (byte)  [f0 dc 76 83] */
        load8(mcu.r[3], ind_addr(0, 0xdc76));
        mcu.pc = 0x5cea;
        return 1;
    case 0x5cea: /* 00045cea SHLL r3 (byte)  [a3 1a] */
        shll_byte(mcu.r[3]);
        mcu.pc = 0x5cec;
        return 1;
    case 0x5cec: /* 00045cec MULXU r3 r5 (byte)  [a3 ad] */
        mulxu_byte(mcu.r[3], mcu.r[5]);
        mcu.pc = 0x5cee;
        return 1;
    case 0x5cee: /* 00045cee SWAP r5  [a5 10] */
        swap_word(mcu.r[5]);
        mcu.pc = 0x5cf0;
        return 1;
    case 0x5cf0: /* 00045cf0 SHLR r5 (byte)  [a5 1b] */
        shlr_byte(mcu.r[5]);
        mcu.pc = 0x5cf2;
        return 1;
    case 0x5cf2: /* 00045cf2 SHLR r5 (byte)  [a5 1b] */
        shlr_byte(mcu.r[5]);
        mcu.pc = 0x5cf4;
        return 1;
    case 0x5cf4: /* 00045cf4 SHLR r5 (byte)  [a5 1b] */
        shlr_byte(mcu.r[5]);
        mcu.pc = 0x5cf6;
        return 1;
    case 0x5cf6: /* 00045cf6 SHLR r5 (byte)  [a5 1b] */
        shlr_byte(mcu.r[5]);
        mcu.pc = 0x5cf8;
        return 1;
    case 0x5cf8: /* 00045cf8 TST (dp,0xdcb4) (byte)  [15 dc b4 16] */
        tst8_mem(dp_addr(0xdcb4));
        mcu.pc = 0x5cfc;
        return 1;
    case 0x5cfc: /* 00045cfc BEQ 0x5d2c  [27 2e] */
        beq(0x5d2c, 0x5cfe);
        return 1;
    case 0x5cfe: /* 00045cfe BTSTI (dp,0xdc02) #3  [15 dc 02 f3] */
        btsti8_mem(dp_addr(0xdc02), 3);
        mcu.pc = 0x5d02;
        return 1;
    case 0x5d02: /* 00045d02 BEQ 0x5d14  [27 10] */
        beq(0x5d14, 0x5d04);
        return 1;
    case 0x5d04: /* 00045d04 BTSTI (dp,0xdc02) #1  [15 dc 02 f1] */
        btsti8_mem(dp_addr(0xdc02), 1);
        mcu.pc = 0x5d08;
        return 1;
    case 0x5d08: /* 00045d08 BNE 0x5d2c  [26 22] */
        bne(0x5d2c, 0x5d0a);
        return 1;
    case 0x5d0a: /* 00045d0a MOVG2 (dp,0xdc2a) r6 (byte)  [15 dc 2a 86] */
        load8(mcu.r[6], dp_addr(0xdc2a));
        mcu.pc = 0x5d0e;
        return 1;
    case 0x5d0e: /* 00045d0e CMP r6 r0  [a0 76] t1=r6, t2=r0.lo */
        MCU_SUB_Common((int32_t)(uint32_t)mcu.r[6],
                       (int32_t)(uint32_t)(mcu.r[0] & 0xff), 0, 0);
        mcu.pc = 0x5d10;
        return 1;
    case 0x5d10: /* 00045d10 BEQ 0x5d2c  [27 1a] */
        beq(0x5d2c, 0x5d12);
        return 1;
    case 0x5d12: /* 00045d12 BRA 0x5d26  [20 12] */
        mcu.pc = 0x5d26;
        return 1;
    case 0x5d14: /* 00045d14 ADD r0 r0 (byte)  [a0 20] */
        add8_self(mcu.r[0]);
        mcu.pc = 0x5d16;
        return 1;
    case 0x5d16: /* 00045d16 EXTU r0  [a0 12] */
        extu(mcu.r[0]);
        mcu.pc = 0x5d18;
        return 1;
    case 0x5d18: /* 00045d18 ADD #0xdc36 r0 (word)  [0c dc 36 20] */
        add16_imm(mcu.r[0], 0xdc36);
        mcu.pc = 0x5d1c;
        return 1;
    case 0x5d1c: /* 00045d1c BTSTI @r0 #9 (word)  [d8 f9] */
        btsti16_mem(reg_addr(0), 9);
        mcu.pc = 0x5d1e;
        return 1;
    case 0x5d1e: /* 00045d1e BEQ 0x5d26  [27 06] */
        beq(0x5d26, 0x5d20);
        return 1;
    case 0x5d20: /* 00045d20 BTSTI (dp,0xdc2b) #0  [15 dc 2b f0] */
        btsti8_mem(dp_addr(0xdc2b), 0);
        mcu.pc = 0x5d24;
        return 1;
    case 0x5d24: /* 00045d24 BEQ 0x5d2c  [27 06] */
        beq(0x5d2c, 0x5d26);
        return 1;
    case 0x5d26: /* 00045d26 TST r5 (byte)  [a5 16] */
        tst8_reg(mcu.r[5]);
        mcu.pc = 0x5d28;
        return 1;
    case 0x5d28: /* 00045d28 BNE 0x5d2c  [26 02] */
        bne(0x5d2c, 0x5d2a);
        return 1;
    case 0x5d2a: /* 00045d2a move r5 #0xff (byte)  [55 ff] */
        move8(mcu.r[5], 0xff);
        mcu.pc = 0x5d2c;
        return 1;
    case 0x5d2c: /* 00045d2c MOVG3 r5 -> r1++ (byte)  [c1 95] */
        store8_postinc(1, mcu.r[5]);
        mcu.pc = 0x5d2e;
        return 1;
    case 0x5d2e: /* 00045d2e ADDQ #1 r2 (byte)  [a2 08] */
        addq_byte_reg(mcu.r[2], 1);
        mcu.pc = 0x5d30;
        return 1;
    case 0x5d30: /* 00045d30 cmp r2,b #0x10  [42 10] */
        cmp_reg_imm8(mcu.r[2], 0x10);
        mcu.pc = 0x5d32;
        return 1;
    case 0x5d32: /* 00045d32 BNE 0x45c7c (16-bit disp; outside this fragment) */
        bne(0x5c7c, 0x5d35);
        return 1;
    case 0x5d35: /* 00045d35 TST (dp,0xd6a7) (byte)  [15 d6 a7 16] */
        tst8_mem(dp_addr(0xd6a7));
        mcu.pc = 0x5d39;
        return 1;
    case 0x5d39: /* 00045d39 BEQ 0x5d99 (rts)  [27 5e] */
        beq(0x5d99, 0x5d3b);
        return 1;
    case 0x5d3b: /* 00045d3b movi r0 #0xd2ee  [58 d2 ee] */
        movi16(mcu.r[0], 0xd2ee);
        mcu.pc = 0x5d3e;
        return 1;
    case 0x5d3e: /* 00045d3e movi r1 #0xd433  [59 d4 33] */
        movi16(mcu.r[1], 0xd433);
        mcu.pc = 0x5d41;
        return 1;
    case 0x5d41: /* 00045d41 movi r2 #0xd378  [5a d3 78] */
        movi16(mcu.r[2], 0xd378);
        mcu.pc = 0x5d44;
        return 1;
    case 0x5d44: /* 00045d44 movi r3 #0x000f  [5b 00 0f] */
        movi16(mcu.r[3], 0x000f);
        mcu.pc = 0x5d47;
        return 1;
    case 0x5d47: /* 00045d47 MOVG3 r3 -> --r7 (word)  [bf 93] */
        store16_predec(7, mcu.r[3]);
        mcu.pc = 0x5d49;
        return 1;
    case 0x5d49: /* 00045d49 ADD #0xd2bf r3 (word)  [0c d2 bf 23] */
        add16_imm(mcu.r[3], 0xd2bf);
        mcu.pc = 0x5d4d;
        return 1;
    case 0x5d4d: /* 00045d4d MOVG2 @r0 r4 (byte)  [d0 84] */
        load8(mcu.r[4], reg_addr(0));
        mcu.pc = 0x5d4f;
        return 1;
    case 0x5d4f: /* 00045d4f MOVG2 @r2 r6 (byte)  [d2 86] */
        load8(mcu.r[6], reg_addr(2));
        mcu.pc = 0x5d51;
        return 1;
    case 0x5d51: /* 00045d51 CMP r4 r6  [a4 76] t1=r6, t2=r4.lo */
        MCU_SUB_Common((int32_t)(uint32_t)mcu.r[6],
                       (int32_t)(uint32_t)(mcu.r[4] & 0xff), 0, 0);
        mcu.pc = 0x5d53;
        return 1;
    case 0x5d53: /* 00045d53 BPL 0x5d5e  [2a 09] */
        bpl(0x5d5e, 0x5d55);
        return 1;
    case 0x5d55: /* 00045d55 MOVG3 r4 -> @r2 (byte)  [d2 94] */
        store8(reg_addr(2), (uint8_t)mcu.r[4]);
        mcu.pc = 0x5d57;
        return 1;
    case 0x5d57: /* 00045d57 MOVG3 r4 -> @r3 (byte)  [d3 94] */
        store8(reg_addr(3), (uint8_t)mcu.r[4]);
        mcu.pc = 0x5d59;
        return 1;
    case 0x5d59: /* 00045d59 MOVG #0x64 -> @r1 (byte)  [d1 06 64] */
        store8(reg_addr(1), 0x64);
        mcu.pc = 0x5d5c;
        return 1;
    case 0x5d5c: /* 00045d5c BRA 0x5d8e  [20 30] */
        mcu.pc = 0x5d8e;
        return 1;
    case 0x5d5e: /* 00045d5e TST @r1 (byte)  [d1 16] */
        tst8_mem(reg_addr(1));
        mcu.pc = 0x5d60;
        return 1;
    case 0x5d60: /* 00045d60 BNE 0x5d8e  [26 2c] */
        bne(0x5d8e, 0x5d62);
        return 1;
    case 0x5d62: /* 00045d62 SUB (dp,0xd6a7) #0x02 (flags only)  [15 d6 a7 04 02] */
        sub8_nowrite(dp_addr(0xd6a7), 0x02);
        mcu.pc = 0x5d67;
        return 1;
    case 0x5d67: /* 00045d67 BNE 0x5d71  [26 08] */
        bne(0x5d71, 0x5d69);
        return 1;
    case 0x5d69: /* 00045d69 MOVG #0x00 -> @r2 (byte)  [d2 06 00] */
        store8(reg_addr(2), 0x00);
        mcu.pc = 0x5d6c;
        return 1;
    case 0x5d6c: /* 00045d6c MOVG #0x00 -> @r3 (byte)  [d3 06 00] */
        store8(reg_addr(3), 0x00);
        mcu.pc = 0x5d6f;
        return 1;
    case 0x5d6f: /* 00045d6f BRA 0x5d8e  [20 1a] */
        mcu.pc = 0x5d8e;
        return 1;
    case 0x5d71: /* 00045d71 TST r6 (byte)  [a6 16] */
        tst8_reg(mcu.r[6]);
        mcu.pc = 0x5d73;
        return 1;
    case 0x5d73: /* 00045d73 BEQ 0x5d79  [27 04] */
        beq(0x5d79, 0x5d75);
        return 1;
    case 0x5d75: /* 00045d75 ADDQ #-1 r6 (byte)  [a6 0c] */
        addq_byte_reg(mcu.r[6], -1);
        mcu.pc = 0x5d77;
        return 1;
    case 0x5d77: /* 00045d77 MOVG3 r6 -> @r2 (byte)  [d2 96] */
        store8(reg_addr(2), (uint8_t)mcu.r[6]);
        mcu.pc = 0x5d79;
        return 1;
    case 0x5d79: /* 00045d79 SUB (dp,0xd6a7) #0x03 (flags only)  [15 d6 a7 04 03] */
        sub8_nowrite(dp_addr(0xd6a7), 0x03);
        mcu.pc = 0x5d7e;
        return 1;
    case 0x5d7e: /* 00045d7e BNE 0x5d89  [26 09] */
        bne(0x5d89, 0x5d80);
        return 1;
    case 0x5d80: /* 00045d80 SUB @r3 #0x10 (flags only)  [d3 04 10] */
        sub8_nowrite(reg_addr(3), 0x10);
        mcu.pc = 0x5d83;
        return 1;
    case 0x5d83: /* 00045d83 BEQ 0x5d8b  [27 06] */
        beq(0x5d8b, 0x5d85);
        return 1;
    case 0x5d85: /* 00045d85 ADDQ #1 @r3 (byte)  [d3 08] */
        addq_byte_mem(reg_addr(3), 1);
        mcu.pc = 0x5d87;
        return 1;
    case 0x5d87: /* 00045d87 BRA 0x5d8b  [20 02] */
        mcu.pc = 0x5d8b;
        return 1;
    case 0x5d89: /* 00045d89 MOVG3 r6 -> @r3 (byte)  [d3 96] */
        store8(reg_addr(3), (uint8_t)mcu.r[6]);
        mcu.pc = 0x5d8b;
        return 1;
    case 0x5d8b: /* 00045d8b MOVG #0x14 -> @r1 (byte)  [d1 06 14] */
        store8(reg_addr(1), 0x14);
        mcu.pc = 0x5d8e;
        return 1;
    case 0x5d8e: /* 00045d8e ADDQ #-1 r0 (word)  [a8 0c] */
        addq_word(mcu.r[0], -1);
        mcu.pc = 0x5d90;
        return 1;
    case 0x5d90: /* 00045d90 ADDQ #-1 r1 (word)  [a9 0c] */
        addq_word(mcu.r[1], -1);
        mcu.pc = 0x5d92;
        return 1;
    case 0x5d92: /* 00045d92 ADDQ #-1 r2 (word)  [aa 0c] */
        addq_word(mcu.r[2], -1);
        mcu.pc = 0x5d94;
        return 1;
    case 0x5d94: /* 00045d94 MOVG2 r7++ r3 (word)  [cf 83] */
        load16_postinc(7, mcu.r[3]);
        mcu.pc = 0x5d96;
        return 1;
    case 0x5d96: /* 00045d96 cntjmp r3 -82  [01 bb ae] */
        mcu.r[3] = (uint16_t)(mcu.r[3] - 1);
        mcu.pc = (mcu.r[3] != 0xffff) ? 0x5d47 : 0x5d99;
        return 1;
    case 0x5d99: /* 00045d99 rts  [19] */
        mcu.pc = MCU_PopStack();
        return 1;
    default:
        stock_instruction();
        return 1;
    }
}

/* ---- per-PC registration tables -------------------------------------------- */

const uint16_t kIrq0Pcs[] = {
    0x0522, 0x0524, 0x0527, 0x0529, 0x052d,
    0x0531, 0x0536, 0x0538, 0x053a, 0x053d,
};

const uint16_t kPcmDispatcherPcs[] = {
    0x51cc, 0x51d0, 0x51d2, 0x51d4, 0x51d6, 0x51d9, 0x51dc, 0x51e0,
    0x51e4, 0x51e6, 0x51e9, 0x51ed, 0x51ef, 0x51f3, 0x51f6, 0x51fa,
    0x51fc, 0x51ff, 0x5203, 0x5207, 0x520b, 0x5210, 0x5212, 0x5215,
    0x5219, 0x521b, 0x521d, 0x5221, 0x5226, 0x522a,
};

/* rom2 flat 0x45cbe.. = cp 4, pc offsets 0x5cbe.. */
const uint16_t kVoiceSearchPcs[] = {
    0x5cbe, 0x5cc0, 0x5cc2, 0x5cc6, 0x5cc8, 0x5cca, 0x5cce, 0x5cd0,
    0x5cd2, 0x5cd4, 0x5cd6, 0x5cd8, 0x5cda, 0x5cdc, 0x5cde, 0x5ce0,
    0x5ce2, 0x5ce4, 0x5ce6, 0x5cea, 0x5cec, 0x5cee, 0x5cf0, 0x5cf2,
    0x5cf4, 0x5cf6, 0x5cf8, 0x5cfc, 0x5cfe, 0x5d02, 0x5d04, 0x5d08,
    0x5d0a, 0x5d0e, 0x5d10, 0x5d12, 0x5d14, 0x5d16, 0x5d18, 0x5d1c,
    0x5d1e, 0x5d20, 0x5d24, 0x5d26, 0x5d28, 0x5d2a, 0x5d2c, 0x5d2e,
    0x5d30, 0x5d32, 0x5d35, 0x5d39, 0x5d3b, 0x5d3e, 0x5d41, 0x5d44,
    0x5d47, 0x5d49, 0x5d4d, 0x5d4f, 0x5d51, 0x5d53, 0x5d55, 0x5d57,
    0x5d59, 0x5d5c, 0x5d5e, 0x5d60, 0x5d62, 0x5d67, 0x5d69, 0x5d6c,
    0x5d6f, 0x5d71, 0x5d73, 0x5d75, 0x5d77, 0x5d79, 0x5d7e, 0x5d80,
    0x5d83, 0x5d85, 0x5d87, 0x5d89, 0x5d8b, 0x5d8e, 0x5d90, 0x5d92,
    0x5d94, 0x5d96, 0x5d99,
};

/* Self-registration hook (hand_registry.h): called from
 * MK2CPP_HandFillTables after the built-in modules. */
void pcm_dispatch_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kIrq0Pcs) / sizeof(kIrq0Pcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kIrq0Pcs[i], &step_irq0);

    for (uint32_t i = 0; i < sizeof(kPcmDispatcherPcs) / sizeof(kPcmDispatcherPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kPcmDispatcherPcs[i], &step_pcm_dispatcher);

    for (uint32_t i = 0; i < sizeof(kVoiceSearchPcs) / sizeof(kVoiceSearchPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | kVoiceSearchPcs[i], &step_voice_search);
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
