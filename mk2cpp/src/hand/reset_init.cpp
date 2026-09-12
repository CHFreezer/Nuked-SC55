/*
 * HAND reset_init -- M4 P3 init/reset family + d1ac bit helpers, one hand
 * entry per H8 instruction (L0: every entry executes exactly one H8
 * instruction, so the host keeps its per-instruction interrupt poll,
 * TIMER_Clock, midiseq/SM cadence, trace and PCM catch-up; a
 * multi-instruction block would defer the host tail and drift the timing
 * exactly like the earlier per-PC slices).
 *
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 * M4 closure step 2 (semantic rewrite): one named void step function per PC, registered flat = 0x00040000 | pc via MK2CPP_HandRegister.
 *
 * Scope (all rom2, cp=4; registration flat = 0x00040000 | local pc):
 *   A2 init_desc        0x41220-0x41263  20 PC   (out/m4/18 1.1 row A2)
 *   A3 init_params      0x41266-0x41324  54 PC   (row A3, calls 0x42B8A)
 *   A4 pcm_chan_init    0x41333-0x4142D  91 PC   (row A4, stub 0x4142F out)
 *   A5 maint_reset32    0x4329F-0x433C2  75 PC   (row A5, callees out)
 *   0x42B8A             0x42B8A-0x42B98   5 PC   (row 32, A3 loop callee)
 *   d1ac helper family  0x43695/A3/B5/C3/D5 (25 PC; 11 1.1 / R16 / 07 347)
 *
 * Boundaries come from the h8part static partition
 * (mk2cpp/out/h8part/map.csv, regenerated from pc_main + flow_main) and
 * tools/baselines/dasm_full.txt; every case below is a byte/PC transcription
 * of the generated per-PC reference mk2cpp/src/gen/mk2c_r2.cpp (same GT
 * helper calls, same flag size, same sign extension, same odd-address trap
 * order), with per-ROM constants folded in. Confidence:
 *   - A2/A4: C (ROM bytes + dasm:13219-13411).
 *   - A3: C bytes / S for the 0x42B8A call relation (18 1.1 row A3).
 *   - A5: C bytes (dasm:13839-13968); the routine's callees
 *     0x433C4/0x433D1/0x43641/0x434CB/0x4342A stay gen-owned, as does the
 *     shared 0x4142F nop;rts stub: they are outside the A5 fragment boundary.
 *   - 0x42B8A: C (dasm:13477-13482; 18 1.1 row 32).
 *   - d1ac family: 0x436B5 (clr bit2 single) is traced (pc_main:7277); the
 *     other four entry PCs are NOT in the recorded runs - their bodies were
 *     decoded from the raw ROM bytes (rom2 fileoff 0x3695..0x36db) and the
 *     loop displacement is checked against cntjmp -14 -> 0x36A6/0x36C6:
 *     C bytes / I dynamic. d1ac stays authoritative in page-0 SRAM and the
 *     index is the raw r0 byte, so 11 1.1's "fixed 32 entries, non-slot
 *     index" model is preserved byte for byte (r0=0x1f -> d1ac[31]).
 *
 * SRAM access via MCU_Read/MCU_Write (and the 16-bit forms where the ROM
 * instruction is 16-bit); every device-visible write goes through MCU_Write
 * so the -pcmtrace routing is kept. No symbols from src/gen are referenced:
 * a hand-only build still links (GT helpers only).
 *
 * Registration: explicit per-PC MK2CPP_HandRegister(flat, &step_...) calls in
 * MK2CPP_ResetInitFillTables, published from a file-static initializer through
 * mk2c::hand_register_module (hand_registry.h); no shared aggregator file is
 * edited. Duplicate registration is fatal in mk2cpp.cpp:96-118, so this module
 * was checked disjoint from every other hand modules range (grep of the whole
 * hand directory: no 0x412xx/0x413xx/0x432xx/0x433xx/0x436xx entries
 * elsewhere).
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"
#include "mcu_interrupt.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; declared locally like the other hand
 * modules (not exported through a header). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

/* Single-entry build switch (09 5.4): 0 makes this slice not register at all
 * without touching other files. */
#define MK2CPP_HAND_RESET_INIT 1

namespace mk2c {
namespace {

/* ---- effective addresses -------------------------------------------------- */

/* Page register for rN: dp (r0-r3), ep (r4/r5), tp (r6/r7). */
uint32_t page_of_reg(uint32_t reg)
{
    return (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
}

/* @rN+disp16: the rN+disp sum wraps inside 16 bits. */
uint32_t ind_addr(uint32_t reg, uint16_t disp)
{
    return ((uint32_t)page_of_reg(reg) << 16) | (uint16_t)(mcu.r[reg] + disp);
}

/* (dp,addr16) absolute through the DP page register. */
uint32_t dp_addr(uint16_t disp)
{
    return ((uint32_t)mcu.dp << 16) | disp;
}

/* (br,disp8): address = (br << 8) | disp, page prefix is zero. */
uint32_t br_addr(uint8_t disp)
{
    return (uint16_t)((uint16_t)((uint32_t)mcu.br << 8) | (uint16_t)disp);
}

/* ---- status --------------------------------------------------------------- */

void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* ---- control register load ------------------------------------------------ */

/* LDC #imm rN: control register write + ex_ignore (gen code does the same). */
void ldc_cr(uint32_t reg, uint8_t imm)
{
    MCU_ControlRegisterWrite(reg, 0, imm);
    mcu.ex_ignore = 1;
}

/* ---- byte moves ----------------------------------------------------------- */

/* MOVG2 @addr -> rN (byte; high byte preserved) */
void load8_via(uint16_t &reg, uint32_t addr)
{
    uint32_t data = (uint32_t)MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* MOVG2 @rBase+disp16 -> rDst (byte) */
void load8_ind(uint16_t &dst, uint32_t base, uint16_t disp)
{
    load8_via(dst, ind_addr(base, disp));
}

/* MOVG3 rN -> @addr (byte) */
void store8_via(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* MOVG3 rN -> @rBase+disp16 (byte) */
void store8_ind(uint32_t base, uint16_t disp, uint8_t value)
{
    store8_via(ind_addr(base, disp), value);
}

/* MOVG3 rN -> @rBase+disp8 (byte; no odd-address check) */
void store8_ind_disp8(uint32_t base, int8_t disp, uint8_t value)
{
    uint32_t oea = (uint16_t)(mcu.r[base] + (uint16_t)(int16_t)disp);
    MCU_Write(((uint32_t)page_of_reg(base) << 16) | (uint16_t)oea, value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* MOVG #imm -> @rN++ (byte store at the old rN, then rN++) */
void store8_postinc(uint32_t reg, uint8_t value)
{
    uint32_t addr = ((uint32_t)page_of_reg(reg) << 16) | (uint16_t)mcu.r[reg];
    mcu.r[reg] = (uint16_t)(mcu.r[reg] + 1);
    MCU_Write(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* (dp,disp16) byte/word stores, MOVG #imm forms. */
void store8_dp(uint16_t disp, uint8_t value)
{
    store8_via(dp_addr(disp), value);
}

void store16_dp(uint16_t disp, uint16_t value)
{
    uint32_t addr = dp_addr(disp);
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 1);
}

/* (br,disp8) stores: MOVG forms and movs/movsw */
void store8_br(uint8_t disp, uint8_t value)
{
    store8_via(br_addr(disp), value);
}

void store16_br(uint8_t disp, uint16_t value)
{
    uint16_t addr = (uint16_t)br_addr(disp);
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 1);
}

/* MOVG #imm16 -> @rN+disp8 (word; odd address traps before the write) */
void store16_ind_disp8(uint32_t base, int8_t disp, uint16_t value)
{
    uint32_t oea = (uint16_t)(mcu.r[base] + (uint16_t)(int16_t)disp);
    if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(((uint32_t)page_of_reg(base) << 16) | (uint16_t)oea, value);
    MCU_SetStatusCommon((uint32_t)value, 1);
}

/* MOVG2 @rBase+disp16 -> rDst (word) */
void load16_ind(uint16_t &dst, uint32_t base, uint16_t disp)
{
    uint32_t addr = ind_addr(base, disp);
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t data = MCU_Read16(addr);
    dst = data;
    MCU_SetStatusCommon((uint32_t)data, 1);
}

/* movs rN @(br,disp8): byte store, status byte */
void movs_br(uint8_t disp, uint8_t value)
{
    uint16_t addr = (uint16_t)br_addr(disp);
    MCU_Write(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* movsw rN @(br,disp8): word store, odd address traps before the write */
void movsw_br(uint8_t disp, uint16_t value)
{
    uint16_t addr = (uint16_t)br_addr(disp);
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, value);
    MCU_SetStatusCommon((uint32_t)value, 1);
}

/* movl rN @(br,disp8): byte load, status byte */
void movl_br(uint16_t &reg, uint8_t disp)
{
    uint32_t data = (uint32_t)MCU_Read(br_addr(disp));
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon(data, 0);
}

/* movlw rN @(br,disp8): word load, odd address traps before the read */
void movlw_br(uint16_t &reg, uint8_t disp)
{
    uint16_t addr = (uint16_t)br_addr(disp);
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t data = MCU_Read16(addr);
    reg = data;
    MCU_SetStatusCommon((uint32_t)data, 1);
}

/* ---- clears --------------------------------------------------------------- */

/* CLR @addr (byte) */
void clr8_via(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

void clr8_dp(uint16_t disp)
{
    clr8_via(dp_addr(disp));
}

/* CLR @rBase+disp16 (byte) */
void clr8_ind(uint32_t base, uint16_t disp)
{
    clr8_via(ind_addr(base, disp));
}

/* CLR @rBase+disp8 (word; odd address traps before the write) */
void clr16_ind_disp8(uint32_t base, int8_t disp)
{
    uint32_t oea = (uint16_t)(mcu.r[base] + (uint16_t)(int16_t)disp);
    if (oea & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(((uint32_t)page_of_reg(base) << 16) | (uint16_t)oea, 0);
    flags_clr();
}

/* CLR (dp,disp16) (word; odd address traps before the write) */
void clr16_dp(uint16_t disp)
{
    uint32_t addr = dp_addr(disp);
    if (addr & 1)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(addr, 0);
    flags_clr();
}

/* CLR rN++ (byte store at the old rN, then rN++) */
void clr8_postinc(uint32_t reg)
{
    uint32_t addr = ((uint32_t)page_of_reg(reg) << 16) | (uint16_t)mcu.r[reg];
    mcu.r[reg] = (uint16_t)(mcu.r[reg] + 1);
    MCU_Write(addr, 0);
    flags_clr();
}

/* CLR rN (word) */
void clr_reg_word(uint16_t &reg)
{
    reg = 0;
    flags_clr();
}

/* ---- register arithmetic -------------------------------------------------- */

/* MOVG2 rS -> rD (word) */
void mov16_reg(uint16_t &dst, uint16_t src)
{
    uint32_t data = (uint32_t)src;
    dst = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

/* MOVI rN #imm16 / MOVE rN #imm8 */
void movi16(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon((uint32_t)value, 1);
}

void move8(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon((uint32_t)value, 0);
}

/* ADD rD rD (word) */
void add16_self(uint16_t &reg)
{
    reg = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)reg, (int32_t)(uint32_t)reg, 0, 1);
}

/* ADDQ #delta rN (word) */
void addq_word(uint16_t &reg, int delta)
{
    reg = (uint16_t)MCU_ADD_Common((int32_t)(uint32_t)reg, delta, 0, 1);
}

/* ADDQ #delta rN (byte; high byte preserved) */
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

/* OR rS rD (word; dst = opcode register) */
void or_word_reg(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)(dst | src);
    MCU_SetStatusCommon((uint32_t)dst, 1);
}

/* OR #imm rN (byte flags; the whole register is ORed, only flags are byte) */
void or_imm8_reg(uint16_t &reg, uint8_t imm)
{
    reg = (uint16_t)(reg | imm);
    MCU_SetStatusCommon((uint32_t)reg, 0);
}

/* AND #imm rN (byte: only the low byte is written, flags from the result) */
void and_imm8_reg(uint16_t &reg, uint8_t imm)
{
    uint32_t data = (uint32_t)reg & imm;
    reg = (uint16_t)((reg & 0xff00u) | (data & 0xffu));
    MCU_SetStatusCommon((uint32_t)reg, 0);
}

/* TST rN (byte): N/Z from the low byte, C cleared, V=0 */
void tst8_reg(uint16_t reg)
{
    MCU_SetStatusCommon((uint32_t)(reg & 0xff), 0);
    MCU_SetStatus(0, STATUS_C);
}

/* CMP rS rD byte form: t1 = full rD, t2 = rS low byte, byte flags */
void cmp8_reg(uint16_t dst, uint16_t src)
{
    MCU_SUB_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)(src & 0xff), 0, 0);
}

/* CMP rS rD word form (dst = opcode register) */
void cmp16_reg(uint16_t dst, uint16_t src)
{
    MCU_SUB_Common((int32_t)(uint32_t)dst, (int32_t)(uint32_t)src, 0, 1);
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

/* SHLL rN (word): C = old bit15, N/Z/V from the result */
void shll_word(uint16_t &reg)
{
    uint32_t data = (uint32_t)reg;
    uint32_t C = (data & 0x8000u) != 0;
    data <<= 1;
    reg = (uint16_t)data;
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
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

/* BCLR @addr #bit (byte): Z = (bit == 0), then clear the bit */
void bclr8_mem(uint32_t addr, uint8_t bit)
{
    uint8_t data = MCU_Read(addr);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    MCU_Write(addr, (uint8_t)(data & ~(1u << bit)));
}

/* ---- control flow --------------------------------------------------------- */

void bne(uint16_t taken, uint16_t fall)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? fall : taken;
}

void beq(uint16_t taken, uint16_t fall)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? taken : fall;
}

/* cntjmp rN disp8: rN--; branch to taken while rN != 0xffff. */
void cntjmp(uint16_t &reg, uint16_t taken, uint16_t fall)
{
    reg = (uint16_t)(reg - 1);
    mcu.pc = (reg != 0xffff) ? taken : fall;
}

/* bsr/jsr: push the return PC, then jump. */
void bsr8(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

void bsr16(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

void jsr(uint16_t next, uint16_t target)
{
    MCU_PushStack(next);
    mcu.pc = target;
}

/* rts: pop pc only; ret (0x11 0x19): pop cp then pc. */
void rts(void)
{
    mcu.pc = MCU_PopStack();
}

void ret_cp(void)
{
    mcu.cp = (uint8_t)MCU_PopStack();
    mcu.pc = MCU_PopStack();
}

/* trapa #imm: pending vector = imm & 0x0f; the host takes it at the next
 * instruction boundary, when mcu.pc is already the following instruction. */
void trapa(uint8_t vector)
{
    MCU_Interrupt_TRAPA((uint32_t)(vector & 0x0f));
}

/* ======================================================================
 * A2 init_desc 0x1220..0x1263 (flat 0x41220-0x41263), 20 PC.
 * Clears d150/d152/d154/d156, then the 27..0 loop at 0x123c zeroes
 * d15c[i] and writes 0x0016 to the three descriptor words at
 * (d64d6[2i])+0/2/4, then r0=1 and trapa #0x11. Entry from pcm_dispatcher
 * 0x51cc (pjsr) which pushes 0x51d0/cp; exit 0x1263 ret (pop cp,pc).
 * Evidence: dasm_full.txt:13219-13242; 18 1.1 row A2 / fragment init.
 * ====================================================================== */

/* 0x1220 LDC #0x00 r5 [04 00 8d]. */
void step_a2_init_desc_ldc_0x00_r5(void)
{
    ldc_cr(5, 0x00);
    mcu.pc = 0x1223;
}

/* 0x1223 LDC #0x00 r4 [04 00 8c]. */
void step_a2_init_desc_ldc_0x00_r4(void)
{
    ldc_cr(4, 0x00);
    mcu.pc = 0x1226;
}

/* 0x1226 LDC #0xe0 r3 [04 e0 8b]. */
void step_a2_init_desc_ldc_0xe0_r3(void)
{
    ldc_cr(3, 0xe0);
    mcu.pc = 0x1229;
}

/* 0x1229 CLR (dp,0xd150) word [1d d1 50 13]. */
void step_a2_init_desc_clr_dp_0xd150_word(void)
{
    clr16_dp(0xd150);
    mcu.pc = 0x122d;
}

/* 0x122d CLR (dp,0xd152) word [1d d1 52 13]. */
void step_a2_init_desc_clr_dp_0xd152_word(void)
{
    clr16_dp(0xd152);
    mcu.pc = 0x1231;
}

/* 0x1231 CLR (dp,0xd154) word [1d d1 54 13]. */
void step_a2_init_desc_clr_dp_0xd154_word(void)
{
    clr16_dp(0xd154);
    mcu.pc = 0x1235;
}

/* 0x1235 CLR (dp,0xd156) word [1d d1 56 13]. */
void step_a2_init_desc_clr_dp_0xd156_word(void)
{
    clr16_dp(0xd156);
    mcu.pc = 0x1239;
}

/* 0x1239 movi r0 #0x001b [58 00 1b]. */
void step_a2_init_desc_movi_r0_0x001b(void)
{
    movi16(mcu.r[0], 0x001b);
    mcu.pc = 0x123c;
}

/* 0x123c MOVG2 r0 r2 word [a8 82]. */
void step_a2_init_desc_movg2_r0_r2_word(void)
{
    mov16_reg(mcu.r[2], mcu.r[0]);
    mcu.pc = 0x123e;
}

/* 0x123e ADD r2 r2 word [aa 22]. */
void step_a2_init_desc_add_r2_r2_word(void)
{
    add16_self(mcu.r[2]);
    mcu.pc = 0x1240;
}

/* 0x1240 MOVG2 @r2+0x64d6 r1 word [fa 64 d6 81]. */
void step_a2_init_desc_movg2_r2_0x64d6_r1_word(void)
{
    load16_ind(mcu.r[1], 2, 0x64d6);
    mcu.pc = 0x1244;
}

/* 0x1244 CLR @r0+0xad0e byte [f0 ad 0e 13]. */
void step_a2_init_desc_clr_r0_0xad0e_byte(void)
{
    clr8_ind(0, 0xad0e);
    mcu.pc = 0x1248;
}

/* 0x1248 MOVG #0x0016 -> @r1+0 word [e9 00 07 00 16]. */
void step_a2_init_desc_movg_0x0016_to_r1_0_word(void)
{
    store16_ind_disp8(1, 0, 0x0016);
    mcu.pc = 0x124d;
}

/* 0x124d MOVG #0x0016 -> @r1+2 word [e9 02 07 00 16]. */
void step_a2_init_desc_movg_0x0016_to_r1_2_word(void)
{
    store16_ind_disp8(1, 2, 0x0016);
    mcu.pc = 0x1252;
}

/* 0x1252 MOVG #0x0016 -> @r1+4 word [e9 04 07 00 16]. */
void step_a2_init_desc_movg_0x0016_to_r1_4_word(void)
{
    store16_ind_disp8(1, 4, 0x0016);
    mcu.pc = 0x1257;
}

/* 0x1257 CLR @r0+0xd15c byte [f0 d1 5c 13]. */
void step_a2_init_desc_clr_r0_0xd15c_byte(void)
{
    clr8_ind(0, 0xd15c);
    mcu.pc = 0x125b;
}

/* 0x125b cntjmp r0 -34 -> 0x123c [01 b8 de]. */
void step_a2_init_desc_cntjmp_r0_34_to_0x123c(void)
{
    cntjmp(mcu.r[0], 0x123c, 0x125e);
}

/* 0x125e movi r0 #0x0001 [58 00 01]. */
void step_a2_init_desc_movi_r0_0x0001(void)
{
    movi16(mcu.r[0], 0x0001);
    mcu.pc = 0x1261;
}

/* 0x1261 trapa #0x11 [08 11]. */
void step_a2_init_desc_trapa_0x11(void)
{
    trapa(0x11);
    mcu.pc = 0x1263;
}

/* 0x1263 ret (pop cp,pc) [11 19]. */
void step_a2_init_desc_ret(void)
{
    ret_cp();
}

/* ======================================================================
 * A3 init_params 0x1266..0x1324 (flat 0x41266-0x41324), 54 PC.
 * move r0 #2 / trapa #0x17, LDC dp=0/ep=0/br=0xe0, clear d1a0/d1a1/
 * d18e/d194-d196, set d19c/d19e=2, copy the 0x802b..0x8039 byte block into
 * d181..d18d (SHLR r6 before d186/d189/d18d), then the 27..0 loop at 0x1308
 * writes P-26=0xff and bsr16 0x42B8A; r0=2/trapa #0x18, r0=8/trapa #0x11,
 * ret. Entry 0x56b6 (C15 ts_scan pjsr); bsr16 pushes 0x1318.
 * Evidence: dasm_full.txt:13244-13304; 18 1.1 row A3.
 * ====================================================================== */

/* 0x1266 move r0 #0x02 [50 02]. */
void step_a3_init_params_move_r0_0x02(void)
{
    move8(mcu.r[0], 0x02);
    mcu.pc = 0x1268;
}

/* 0x1268 trapa #0x17 [08 17]. */
void step_a3_init_params_trapa_0x17(void)
{
    trapa(0x17);
    mcu.pc = 0x126a;
}

/* 0x126a LDC #0x00 r5 [04 00 8d]. */
void step_a3_init_params_ldc_0x00_r5(void)
{
    ldc_cr(5, 0x00);
    mcu.pc = 0x126d;
}

/* 0x126d LDC #0x00 r4 [04 00 8c]. */
void step_a3_init_params_ldc_0x00_r4(void)
{
    ldc_cr(4, 0x00);
    mcu.pc = 0x1270;
}

/* 0x1270 LDC #0xe0 r3 [04 e0 8b]. */
void step_a3_init_params_ldc_0xe0_r3(void)
{
    ldc_cr(3, 0xe0);
    mcu.pc = 0x1273;
}

/* 0x1273 CLR (dp,0xd1a0) byte [15 d1 a0 13]. */
void step_a3_init_params_clr_dp_0xd1a0_byte(void)
{
    clr8_dp(0xd1a0);
    mcu.pc = 0x1277;
}

/* 0x1277 CLR (dp,0xd1a1) byte [15 d1 a1 13]. */
void step_a3_init_params_clr_dp_0xd1a1_byte(void)
{
    clr8_dp(0xd1a1);
    mcu.pc = 0x127b;
}

/* 0x127b CLR (dp,0xd18e) word [1d d1 8e 13]. */
void step_a3_init_params_clr_dp_0xd18e_word(void)
{
    clr16_dp(0xd18e);
    mcu.pc = 0x127f;
}

/* 0x127f CLR (dp,0xd194) byte [15 d1 94 13]. */
void step_a3_init_params_clr_dp_0xd194_byte(void)
{
    clr8_dp(0xd194);
    mcu.pc = 0x1283;
}

/* 0x1283 CLR (dp,0xd195) byte [15 d1 95 13]. */
void step_a3_init_params_clr_dp_0xd195_byte(void)
{
    clr8_dp(0xd195);
    mcu.pc = 0x1287;
}

/* 0x1287 CLR (dp,0xd196) byte [15 d1 96 13]. */
void step_a3_init_params_clr_dp_0xd196_byte(void)
{
    clr8_dp(0xd196);
    mcu.pc = 0x128b;
}

/* 0x128b MOVG #0x0002 -> (dp,0xd19c) word [1d d1 9c 07 00 02]. */
void step_a3_init_params_movg_0x0002_to_dp_0xd19c_word(void)
{
    store16_dp(0xd19c, 0x0002);
    mcu.pc = 0x1291;
}

/* 0x1291 MOVG #0x0002 -> (dp,0xd19e) word [1d d1 9e 07 00 02]. */
void step_a3_init_params_movg_0x0002_to_dp_0xd19e_word(void)
{
    store16_dp(0xd19e, 0x0002);
    mcu.pc = 0x1297;
}

/* 0x1297 MOVG2 (dp,0x802b) r6 byte [15 80 2b 86]. */
void step_a3_init_params_movg2_dp_0x802b_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x802b));
    mcu.pc = 0x129b;
}

/* 0x129b MOVG3 r6 -> (dp,0xd181) byte [15 d1 81 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd181_byte(void)
{
    store8_dp(0xd181, (uint8_t)mcu.r[6]);
    mcu.pc = 0x129f;
}

/* 0x129f MOVG2 (dp,0x802c) r6 byte [15 80 2c 86]. */
void step_a3_init_params_movg2_dp_0x802c_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x802c));
    mcu.pc = 0x12a3;
}

/* 0x12a3 MOVG3 r6 -> (dp,0xd182) byte [15 d1 82 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd182_byte(void)
{
    store8_dp(0xd182, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12a7;
}

/* 0x12a7 MOVG2 (dp,0x802d) r6 byte [15 80 2d 86]. */
void step_a3_init_params_movg2_dp_0x802d_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x802d));
    mcu.pc = 0x12ab;
}

/* 0x12ab MOVG3 r6 -> (dp,0xd183) byte [15 d1 83 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd183_byte(void)
{
    store8_dp(0xd183, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12af;
}

/* 0x12af MOVG2 (dp,0x802e) r6 byte [15 80 2e 86]. */
void step_a3_init_params_movg2_dp_0x802e_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x802e));
    mcu.pc = 0x12b3;
}

/* 0x12b3 MOVG3 r6 -> (dp,0xd184) byte [15 d1 84 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd184_byte(void)
{
    store8_dp(0xd184, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12b7;
}

/* 0x12b7 MOVG2 (dp,0x802f) r6 byte [15 80 2f 86]. */
void step_a3_init_params_movg2_dp_0x802f_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x802f));
    mcu.pc = 0x12bb;
}

/* 0x12bb MOVG3 r6 -> (dp,0xd185) byte [15 d1 85 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd185_byte(void)
{
    store8_dp(0xd185, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12bf;
}

/* 0x12bf MOVG2 (dp,0x8030) r6 byte [15 80 30 86]. */
void step_a3_init_params_movg2_dp_0x8030_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x8030));
    mcu.pc = 0x12c3;
}

/* 0x12c3 SHLR r6 byte [a6 1b]. */
void step_a3_init_params_shlr_r6_byte(void)
{
    shlr_byte(mcu.r[6]);
    mcu.pc = 0x12c5;
}

/* 0x12c5 MOVG3 r6 -> (dp,0xd186) byte [15 d1 86 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd186_byte(void)
{
    store8_dp(0xd186, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12c9;
}

/* 0x12c9 MOVG2 (dp,0x8033) r6 byte [15 80 33 86]. */
void step_a3_init_params_movg2_dp_0x8033_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x8033));
    mcu.pc = 0x12cd;
}

/* 0x12cd MOVG3 r6 -> (dp,0xd187) byte [15 d1 87 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd187_byte(void)
{
    store8_dp(0xd187, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12d1;
}

/* 0x12d1 MOVG2 (dp,0x8034) r6 byte [15 80 34 86]. */
void step_a3_init_params_movg2_dp_0x8034_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x8034));
    mcu.pc = 0x12d5;
}

/* 0x12d5 MOVG3 r6 -> (dp,0xd188) byte [15 d1 88 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd188_byte(void)
{
    store8_dp(0xd188, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12d9;
}

/* 0x12d9 MOVG2 (dp,0x8035) r6 byte [15 80 35 86]. */
void step_a3_init_params_movg2_dp_0x8035_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x8035));
    mcu.pc = 0x12dd;
}

/* 0x12dd SHLR r6 byte [a6 1b]. */
void step_a3_init_params_shlr_r6_byte_a(void)
{
    shlr_byte(mcu.r[6]);
    mcu.pc = 0x12df;
}

/* 0x12df MOVG3 r6 -> (dp,0xd189) byte [15 d1 89 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd189_byte(void)
{
    store8_dp(0xd189, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12e3;
}

/* 0x12e3 MOVG2 (dp,0x8036) r6 byte [15 80 36 86]. */
void step_a3_init_params_movg2_dp_0x8036_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x8036));
    mcu.pc = 0x12e7;
}

/* 0x12e7 MOVG3 r6 -> (dp,0xd18a) byte [15 d1 8a 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd18a_byte(void)
{
    store8_dp(0xd18a, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12eb;
}

/* 0x12eb MOVG2 (dp,0x8037) r6 byte [15 80 37 86]. */
void step_a3_init_params_movg2_dp_0x8037_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x8037));
    mcu.pc = 0x12ef;
}

/* 0x12ef MOVG3 r6 -> (dp,0xd18b) byte [15 d1 8b 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd18b_byte(void)
{
    store8_dp(0xd18b, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12f3;
}

/* 0x12f3 MOVG2 (dp,0x8038) r6 byte [15 80 38 86]. */
void step_a3_init_params_movg2_dp_0x8038_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x8038));
    mcu.pc = 0x12f7;
}

/* 0x12f7 MOVG3 r6 -> (dp,0xd18c) byte [15 d1 8c 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd18c_byte(void)
{
    store8_dp(0xd18c, (uint8_t)mcu.r[6]);
    mcu.pc = 0x12fb;
}

/* 0x12fb MOVG2 (dp,0x8039) r6 byte [15 80 39 86]. */
void step_a3_init_params_movg2_dp_0x8039_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0x8039));
    mcu.pc = 0x12ff;
}

/* 0x12ff SHLR r6 byte [a6 1b]. */
void step_a3_init_params_shlr_r6_byte_b(void)
{
    shlr_byte(mcu.r[6]);
    mcu.pc = 0x1301;
}

/* 0x1301 MOVG3 r6 -> (dp,0xd18d) byte [15 d1 8d 96]. */
void step_a3_init_params_movg3_r6_to_dp_0xd18d_byte(void)
{
    store8_dp(0xd18d, (uint8_t)mcu.r[6]);
    mcu.pc = 0x1305;
}

/* 0x1305 movi r1 #0x001b [59 00 1b]. */
void step_a3_init_params_movi_r1_0x001b(void)
{
    movi16(mcu.r[1], 0x001b);
    mcu.pc = 0x1308;
}

/* 0x1308 MOVG2 r1 r2 word [a9 82]. */
void step_a3_init_params_movg2_r1_r2_word(void)
{
    mov16_reg(mcu.r[2], mcu.r[1]);
    mcu.pc = 0x130a;
}

/* 0x130a SHLL r2 word [aa 1a]. */
void step_a3_init_params_shll_r2_word(void)
{
    shll_word(mcu.r[2]);
    mcu.pc = 0x130c;
}

/* 0x130c MOVG2 @r2+0x64d6 r0 word [fa 64 d6 80]. */
void step_a3_init_params_movg2_r2_0x64d6_r0_word(void)
{
    load16_ind(mcu.r[0], 2, 0x64d6);
    mcu.pc = 0x1310;
}

/* 0x1310 MOVG #0x00ff -> @r0+-26 byte [e0 e6 07 00 ff]. */
void step_a3_init_params_movg_0x00ff_to_r0_m26_byte(void)
{
    store8_ind_disp8(0, (int8_t)0xe6, 0xff);
    mcu.pc = 0x1315;
}

/* 0x1315 bsr16 -> 0x2b8a (0x42B8A) [1e 18 72]. */
void step_a3_init_params_bsr16_to_0x2b8a(void)
{
    bsr16(0x1318, 0x2b8a);
}

/* 0x1318 cntjmp r1 -19 -> 0x1308 [01 b9 ed]. */
void step_a3_init_params_cntjmp_r1_19_to_0x1308(void)
{
    cntjmp(mcu.r[1], 0x1308, 0x131b);
}

/* 0x131b move r0 #0x02 [50 02]. */
void step_a3_init_params_move_r0_0x02_a(void)
{
    move8(mcu.r[0], 0x02);
    mcu.pc = 0x131d;
}

/* 0x131d trapa #0x18 [08 18]. */
void step_a3_init_params_trapa_0x18(void)
{
    trapa(0x18);
    mcu.pc = 0x131f;
}

/* 0x131f movi r0 #0x0008 [58 00 08]. */
void step_a3_init_params_movi_r0_0x0008(void)
{
    movi16(mcu.r[0], 0x0008);
    mcu.pc = 0x1322;
}

/* 0x1322 trapa #0x11 [08 11]. */
void step_a3_init_params_trapa_0x11(void)
{
    trapa(0x11);
    mcu.pc = 0x1324;
}

/* 0x1324 ret (pop cp,pc) [11 19]. */
void step_a3_init_params_ret(void)
{
    ret_cp();
}

/* ======================================================================
 * A4 pcm_chan_init 0x1333..0x142d (flat 0x41333-0x4142D), 91 PC.
 * LDC dp=0/ep=0/br=0xe0, r0=r1=0, LDC dp=4, r0 = r0 | word[0x1432],
 * LDC dp=0, store r0 to (br,0x3c), clear (br,0/2), copy (br,0x3e) into r1
 * and program the br window at 0x3e = 0x1b down to 0 (loop movs/movsw
 * 0x1362-0x1374), then the channel setup for br entries 0x1e/0x1f and the
 * mask-table walk with bsr 0x4142F nops; restore LDC 0/0/0xff and ret.
 * Entry boot `0x0217 pjsr`; the shared 0x4142F `nop; rts` stub stays
 * gen-owned (outside the fragment). Evidence: dasm:13306-13411; 18 1.1 A4.
 * ====================================================================== */

/* 0x1333 LDC #0x00 r5 [04 00 8d]. */
void step_a4_pcm_chan_init_ldc_0x00_r5(void)
{
    ldc_cr(5, 0x00);
    mcu.pc = 0x1336;
}

/* 0x1336 LDC #0x00 r4 [04 00 8c]. */
void step_a4_pcm_chan_init_ldc_0x00_r4(void)
{
    ldc_cr(4, 0x00);
    mcu.pc = 0x1339;
}

/* 0x1339 LDC #0xe0 r3 [04 e0 8b]. */
void step_a4_pcm_chan_init_ldc_0xe0_r3(void)
{
    ldc_cr(3, 0xe0);
    mcu.pc = 0x133c;
}

/* 0x133c CLR r0 word [a8 13]. */
void step_a4_pcm_chan_init_clr_r0_word(void)
{
    clr_reg_word(mcu.r[0]);
    mcu.pc = 0x133e;
}

/* 0x133e CLR r1 word [a9 13]. */
void step_a4_pcm_chan_init_clr_r1_word(void)
{
    clr_reg_word(mcu.r[1]);
    mcu.pc = 0x1340;
}

/* 0x1340 LDC #0x04 r5 [04 04 8d]. */
void step_a4_pcm_chan_init_ldc_0x04_r5(void)
{
    ldc_cr(5, 0x04);
    mcu.pc = 0x1343;
}

/* 0x1343 MOVG2 @r1+0x1432 r1 word [f9 14 32 81]. */
void step_a4_pcm_chan_init_movg2_r1_0x1432_r1_word(void)
{
    load16_ind(mcu.r[1], 1, 0x1432);
    mcu.pc = 0x1347;
}

/* 0x1347 OR r1 r0 word [a9 40]. */
void step_a4_pcm_chan_init_or_r1_r0_word(void)
{
    or_word_reg(mcu.r[0], mcu.r[1]);
    mcu.pc = 0x1349;
}

/* 0x1349 LDC #0x00 r5 [04 00 8d]. */
void step_a4_pcm_chan_init_ldc_0x00_r5_a(void)
{
    ldc_cr(5, 0x00);
    mcu.pc = 0x134c;
}

/* 0x134c MOVG3 r0 -> (br,$3c) word [0d 3c 90]. */
void step_a4_pcm_chan_init_movg3_r0_to_br_0x3c_word(void)
{
    store16_br(0x3c, mcu.r[0]);
    mcu.pc = 0x134f;
}

/* 0x134f MOVG #0x0000 -> (br,$00) word [0d 00 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x00_word(void)
{
    store16_br(0x00, 0x0000);
    mcu.pc = 0x1354;
}

/* 0x1354 MOVG #0x0000 -> (br,$02) word [0d 02 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x02_word(void)
{
    store16_br(0x02, 0x0000);
    mcu.pc = 0x1359;
}

/* 0x1359 MOVG2 (br,$3e) r1 byte [05 3e 81]. */
void step_a4_pcm_chan_init_movg2_br_0x3e_r1_byte(void)
{
    load8_via(mcu.r[1], br_addr(0x3e));
    mcu.pc = 0x135c;
}

/* 0x135c movi r0 #0x001b [58 00 1b]. */
void step_a4_pcm_chan_init_movi_r0_0x001b(void)
{
    movi16(mcu.r[0], 0x001b);
    mcu.pc = 0x135f;
}

/* 0x135f movi r1 #0x00ba [59 00 ba]. */
void step_a4_pcm_chan_init_movi_r1_0x00ba(void)
{
    movi16(mcu.r[1], 0x00ba);
    mcu.pc = 0x1362;
}

/* 0x1362 movs r0 @(br,$3e) [70 3e]. */
void step_a4_pcm_chan_init_movs_r0_br_0x3e(void)
{
    movs_br(0x3e, (uint8_t)mcu.r[0]);
    mcu.pc = 0x1364;
}

/* 0x1364 MOVG #0x0000 -> (br,$12) word [0d 12 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x12_word(void)
{
    store16_br(0x12, 0x0000);
    mcu.pc = 0x1369;
}

/* 0x1369 MOVG #0x0000 -> (br,$14) word [0d 14 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x14_word(void)
{
    store16_br(0x14, 0x0000);
    mcu.pc = 0x136e;
}

/* 0x136e movsw r1 @(br,$16) [79 16]. */
void step_a4_pcm_chan_init_movsw_r1_br_0x16(void)
{
    movsw_br(0x16, mcu.r[1]);
    mcu.pc = 0x1370;
}

/* 0x1370 movsw r1 @(br,$18) [79 18]. */
void step_a4_pcm_chan_init_movsw_r1_br_0x18(void)
{
    movsw_br(0x18, mcu.r[1]);
    mcu.pc = 0x1372;
}

/* 0x1372 movsw r1 @(br,$1a) [79 1a]. */
void step_a4_pcm_chan_init_movsw_r1_br_0x1a(void)
{
    movsw_br(0x1a, mcu.r[1]);
    mcu.pc = 0x1374;
}

/* 0x1374 cntjmp r0 -21 -> 0x1362 [01 b8 eb]. */
void step_a4_pcm_chan_init_cntjmp_r0_21_to_0x1362(void)
{
    cntjmp(mcu.r[0], 0x1362, 0x1377);
}

/* 0x1377 MOVG #0x1e -> (br,$3e) byte [05 3e 06 1e]. */
void step_a4_pcm_chan_init_movg_0x1e_to_br_0x3e_byte(void)
{
    store8_br(0x3e, 0x1e);
    mcu.pc = 0x137b;
}

/* 0x137b MOVG #0x0000 -> (br,$14) word [0d 14 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x14_word_a(void)
{
    store16_br(0x14, 0x0000);
    mcu.pc = 0x1380;
}

/* 0x1380 MOVG #0x0000 -> (br,$16) word [0d 16 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x16_word(void)
{
    store16_br(0x16, 0x0000);
    mcu.pc = 0x1385;
}

/* 0x1385 MOVG #0x1f -> (br,$3e) byte [05 3e 06 1f]. */
void step_a4_pcm_chan_init_movg_0x1f_to_br_0x3e_byte(void)
{
    store8_br(0x3e, 0x1f);
    mcu.pc = 0x1389;
}

/* 0x1389 MOVG #0x0000 -> (br,$14) word [0d 14 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x14_word_b(void)
{
    store16_br(0x14, 0x0000);
    mcu.pc = 0x138e;
}

/* 0x138e MOVG #0x0000 -> (br,$16) word [0d 16 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x16_word_a(void)
{
    store16_br(0x16, 0x0000);
    mcu.pc = 0x1393;
}

/* 0x1393 MOVG #0x0000 -> (br,$18) word [0d 18 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x18_word(void)
{
    store16_br(0x18, 0x0000);
    mcu.pc = 0x1398;
}

/* 0x1398 MOVG #0x0000 -> (br,$1a) word [0d 1a 07 00 00]. */
void step_a4_pcm_chan_init_movg_0x0000_to_br_0x1a_word(void)
{
    store16_br(0x1a, 0x0000);
    mcu.pc = 0x139d;
}

/* 0x139d MOVG #0xffff -> (br,$00) word [0d 00 07 ff ff]. */
void step_a4_pcm_chan_init_movg_0xffff_to_br_0x00_word(void)
{
    store16_br(0x00, 0xffff);
    mcu.pc = 0x13a2;
}

/* 0x13a2 MOVG #0xffff -> (br,$02) word [0d 02 07 ff ff]. */
void step_a4_pcm_chan_init_movg_0xffff_to_br_0x02_word(void)
{
    store16_br(0x02, 0xffff);
    mcu.pc = 0x13a7;
}

/* 0x13a7 movl r6 @(br,$00) byte [66 00]. */
void step_a4_pcm_chan_init_movl_r6_br_0x00_byte(void)
{
    movl_br(mcu.r[6], 0x00);
    mcu.pc = 0x13a9;
}

/* 0x13a9 MOVG #0x00 -> (br,$00) word (signed imm8) [0d 00 06 00]. */
void step_a4_pcm_chan_init_movg_0x00_signed_to_br_0x00_word(void)
{
    store16_br(0x00, (uint16_t)(int16_t)(int8_t)0x00);
    mcu.pc = 0x13ad;
}

/* 0x13ad MOVG #0x00 -> (br,$02) word (signed imm8) [0d 02 06 00]. */
void step_a4_pcm_chan_init_movg_0x00_signed_to_br_0x02_word(void)
{
    store16_br(0x02, (uint16_t)(int16_t)(int8_t)0x00);
    mcu.pc = 0x13b1;
}

/* 0x13b1 MOVG2 (br,$3e) r1 byte [05 3e 81]. */
void step_a4_pcm_chan_init_movg2_br_0x3e_r1_byte_a(void)
{
    load8_via(mcu.r[1], br_addr(0x3e));
    mcu.pc = 0x13b4;
}

/* 0x13b4 MOVG #0x1e -> (br,$3e) byte [05 3e 06 1e]. */
void step_a4_pcm_chan_init_movg_0x1e_to_br_0x3e_byte_a(void)
{
    store8_br(0x3e, 0x1e);
    mcu.pc = 0x13b8;
}

/* 0x13b8 MOVG #0xffff -> (br,$34) word [0d 34 07 ff ff]. */
void step_a4_pcm_chan_init_movg_0xffff_to_br_0x34_word(void)
{
    store16_br(0x34, 0xffff);
    mcu.pc = 0x13bd;
}

/* 0x13bd movl r5 @(br,$34) byte [65 34]. */
void step_a4_pcm_chan_init_movl_r5_br_0x34_byte(void)
{
    movl_br(mcu.r[5], 0x34);
    mcu.pc = 0x13bf;
}

/* 0x13bf movlw r5 @(br,$3a) word [6d 3a]. */
void step_a4_pcm_chan_init_movlw_r5_br_0x3a_word(void)
{
    movlw_br(mcu.r[5], 0x3a);
    mcu.pc = 0x13c1;
}

/* 0x13c1 BEQ -15 -> 0x13b4 [27 f1]. */
void step_a4_pcm_chan_init_beq_0x13b4(void)
{
    beq(0x13b4, 0x13c3);
}

/* 0x13c3 move r4 #0x1d [54 1d]. */
void step_a4_pcm_chan_init_move_r4_0x1d(void)
{
    move8(mcu.r[4], 0x1d);
    mcu.pc = 0x13c5;
}

/* 0x13c5 movi r6 #0x3800 [5e 38 00]. */
void step_a4_pcm_chan_init_movi_r6_0x3800(void)
{
    movi16(mcu.r[6], 0x3800);
    mcu.pc = 0x13c8;
}

/* 0x13c8 movs r4 @(br,$3e) [74 3e]. */
void step_a4_pcm_chan_init_movs_r4_br_0x3e(void)
{
    movs_br(0x3e, (uint8_t)mcu.r[4]);
    mcu.pc = 0x13ca;
}

/* 0x13ca movsw r6 @(br,$32) [7e 32]. */
void step_a4_pcm_chan_init_movsw_r6_br_0x32(void)
{
    movsw_br(0x32, mcu.r[6]);
    mcu.pc = 0x13cc;
}

/* 0x13cc movi r1 #0x007f [59 00 7f]. */
void step_a4_pcm_chan_init_movi_r1_0x007f(void)
{
    movi16(mcu.r[1], 0x007f);
    mcu.pc = 0x13cf;
}

/* 0x13cf move r6 #0x1f [56 1f]. */
void step_a4_pcm_chan_init_move_r6_0x1f(void)
{
    move8(mcu.r[6], 0x1f);
    mcu.pc = 0x13d1;
}

/* 0x13d1 movs r6 @(br,$3e) [76 3e]. */
void step_a4_pcm_chan_init_movs_r6_br_0x3e(void)
{
    movs_br(0x3e, (uint8_t)mcu.r[6]);
    mcu.pc = 0x13d3;
}

/* 0x13d3 movsw r1 @(br,$1e) [79 1e]. */
void step_a4_pcm_chan_init_movsw_r1_br_0x1e(void)
{
    movsw_br(0x1e, mcu.r[1]);
    mcu.pc = 0x13d5;
}

/* 0x13d5 bsr 88 -> 0x142f (shared nop;rts stub, gen-owned) [0e 58]. */
void step_a4_pcm_chan_init_bsr_to_0x142f(void)
{
    bsr8(0x13d7, 0x142f);
}

/* 0x13d7 bsr 86 -> 0x142f [0e 56]. */
void step_a4_pcm_chan_init_bsr_to_0x142f_a(void)
{
    bsr8(0x13d9, 0x142f);
}

/* 0x13d9 movsw r1 @(br,$1e) [79 1e]. */
void step_a4_pcm_chan_init_movsw_r1_br_0x1e_a(void)
{
    movsw_br(0x1e, mcu.r[1]);
    mcu.pc = 0x13db;
}

/* 0x13db movl r4 @(br,$1e) byte [64 1e]. */
void step_a4_pcm_chan_init_movl_r4_br_0x1e_byte(void)
{
    movl_br(mcu.r[4], 0x1e);
    mcu.pc = 0x13dd;
}

/* 0x13dd movlw r4 @(br,$3a) word [6c 3a]. */
void step_a4_pcm_chan_init_movlw_r4_br_0x3a_word(void)
{
    movlw_br(mcu.r[4], 0x3a);
    mcu.pc = 0x13df;
}

/* 0x13df CMP r4 r1 byte [a4 71]. */
void step_a4_pcm_chan_init_cmp_r4_r1_byte(void)
{
    cmp8_reg(mcu.r[1], mcu.r[4]);
    mcu.pc = 0x13e1;
}

/* 0x13e1 BNE -18 -> 0x13d1 [26 ee]. */
void step_a4_pcm_chan_init_bne_0x13d1(void)
{
    bne(0x13d1, 0x13e3);
}

/* 0x13e3 movi r1 #0x3800 [59 38 00]. */
void step_a4_pcm_chan_init_movi_r1_0x3800(void)
{
    movi16(mcu.r[1], 0x3800);
    mcu.pc = 0x13e6;
}

/* 0x13e6 MOVG2 r1 r2 word [a9 82]. */
void step_a4_pcm_chan_init_movg2_r1_r2_word(void)
{
    mov16_reg(mcu.r[2], mcu.r[1]);
    mcu.pc = 0x13e8;
}

/* 0x13e8 ADDQ #2 r2 word [aa 09]. */
void step_a4_pcm_chan_init_addq_2_r2_word(void)
{
    addq_word(mcu.r[2], 2);
    mcu.pc = 0x13ea;
}

/* 0x13ea CLR r3 word [ab 13]. */
void step_a4_pcm_chan_init_clr_r3_word(void)
{
    clr_reg_word(mcu.r[3]);
    mcu.pc = 0x13ec;
}

/* 0x13ec movs r3 @(br,$09) [73 09]. */
void step_a4_pcm_chan_init_movs_r3_br_0x09(void)
{
    movs_br(0x09, (uint8_t)mcu.r[3]);
    mcu.pc = 0x13ee;
}

/* 0x13ee movsw r1 @(br,$0a) [79 0a]. */
void step_a4_pcm_chan_init_movsw_r1_br_0x0a(void)
{
    movsw_br(0x0a, mcu.r[1]);
    mcu.pc = 0x13f0;
}

/* 0x13f0 MOVG3 r3 -> (br,$0d) byte [05 0d 93]. */
void step_a4_pcm_chan_init_movg3_r3_to_br_0x0d_byte(void)
{
    store8_br(0x0d, (uint8_t)mcu.r[3]);
    mcu.pc = 0x13f3;
}

/* 0x13f3 movsw r2 @(br,$0e) [7a 0e]. */
void step_a4_pcm_chan_init_movsw_r2_br_0x0e(void)
{
    movsw_br(0x0e, mcu.r[2]);
    mcu.pc = 0x13f5;
}

/* 0x13f5 MOVG3 r3 -> (br,$10) word [0d 10 93]. */
void step_a4_pcm_chan_init_movg3_r3_to_br_0x10_word(void)
{
    store16_br(0x10, mcu.r[3]);
    mcu.pc = 0x13f8;
}

/* 0x13f8 ADDQ #1 r1 word [a9 08]. */
void step_a4_pcm_chan_init_addq_1_r1_word(void)
{
    addq_word(mcu.r[1], 1);
    mcu.pc = 0x13fa;
}

/* 0x13fa movs r3 @(br,$05) [73 05]. */
void step_a4_pcm_chan_init_movs_r3_br_0x05(void)
{
    movs_br(0x05, (uint8_t)mcu.r[3]);
    mcu.pc = 0x13fc;
}

/* 0x13fc movsw r1 @(br,$06) [79 06]. */
void step_a4_pcm_chan_init_movsw_r1_br_0x06(void)
{
    movsw_br(0x06, mcu.r[1]);
    mcu.pc = 0x13fe;
}

/* 0x13fe bsr 47 -> 0x142f [0e 2f]. */
void step_a4_pcm_chan_init_bsr_to_0x142f_b(void)
{
    bsr8(0x1400, 0x142f);
}

/* 0x1400 bsr 45 -> 0x142f [0e 2d]. */
void step_a4_pcm_chan_init_bsr_to_0x142f_c(void)
{
    bsr8(0x1402, 0x142f);
}

/* 0x1402 movs r3 @(br,$05) [73 05]. */
void step_a4_pcm_chan_init_movs_r3_br_0x05_a(void)
{
    movs_br(0x05, (uint8_t)mcu.r[3]);
    mcu.pc = 0x1404;
}

/* 0x1404 movsw r1 @(br,$06) [79 06]. */
void step_a4_pcm_chan_init_movsw_r1_br_0x06_a(void)
{
    movsw_br(0x06, mcu.r[1]);
    mcu.pc = 0x1406;
}

/* 0x1406 movl r2 @(br,$05) byte [62 05]. */
void step_a4_pcm_chan_init_movl_r2_br_0x05_byte(void)
{
    movl_br(mcu.r[2], 0x05);
    mcu.pc = 0x1408;
}

/* 0x1408 movl r2 @(br,$39) byte [62 39]. */
void step_a4_pcm_chan_init_movl_r2_br_0x39_byte(void)
{
    movl_br(mcu.r[2], 0x39);
    mcu.pc = 0x140a;
}

/* 0x140a movlw r4 @(br,$3a) word [6c 3a]. */
void step_a4_pcm_chan_init_movlw_r4_br_0x3a_word_a(void)
{
    movlw_br(mcu.r[4], 0x3a);
    mcu.pc = 0x140c;
}

/* 0x140c TST r2 byte [a2 16]. */
void step_a4_pcm_chan_init_tst_r2_byte(void)
{
    tst8_reg(mcu.r[2]);
    mcu.pc = 0x140e;
}

/* 0x140e BNE -22 -> 0x13fa [26 ea]. */
void step_a4_pcm_chan_init_bne_0x13fa(void)
{
    bne(0x13fa, 0x1410);
}

/* 0x1410 CMP r1 r4 word [a9 74]. */
void step_a4_pcm_chan_init_cmp_r1_r4_word(void)
{
    cmp16_reg(mcu.r[4], mcu.r[1]);
    mcu.pc = 0x1412;
}

/* 0x1412 BNE -26 -> 0x13fa [26 e6]. */
void step_a4_pcm_chan_init_bne_0x13fa_a(void)
{
    bne(0x13fa, 0x1414);
}

/* 0x1414 movsw r3 @(br,$30) [7b 30]. */
void step_a4_pcm_chan_init_movsw_r3_br_0x30(void)
{
    movsw_br(0x30, mcu.r[3]);
    mcu.pc = 0x1416;
}

/* 0x1416 bsr 23 -> 0x142f [0e 17]. */
void step_a4_pcm_chan_init_bsr_to_0x142f_d(void)
{
    bsr8(0x1418, 0x142f);
}

/* 0x1418 bsr 21 -> 0x142f [0e 15]. */
void step_a4_pcm_chan_init_bsr_to_0x142f_e(void)
{
    bsr8(0x141a, 0x142f);
}

/* 0x141a movsw r3 @(br,$30) [7b 30]. */
void step_a4_pcm_chan_init_movsw_r3_br_0x30_a(void)
{
    movsw_br(0x30, mcu.r[3]);
    mcu.pc = 0x141c;
}

/* 0x141c movl r2 @(br,$30) byte [62 30]. */
void step_a4_pcm_chan_init_movl_r2_br_0x30_byte(void)
{
    movl_br(mcu.r[2], 0x30);
    mcu.pc = 0x141e;
}

/* 0x141e movlw r4 @(br,$3a) word [6c 3a]. */
void step_a4_pcm_chan_init_movlw_r4_br_0x3a_word_b(void)
{
    movlw_br(mcu.r[4], 0x3a);
    mcu.pc = 0x1420;
}

/* 0x1420 TST r4 byte [a4 16]. */
void step_a4_pcm_chan_init_tst_r4_byte(void)
{
    tst8_reg(mcu.r[4]);
    mcu.pc = 0x1422;
}

/* 0x1422 BNE -16 -> 0x1414 [26 f0]. */
void step_a4_pcm_chan_init_bne_0x1414(void)
{
    bne(0x1414, 0x1424);
}

/* 0x1424 LDC #0x00 r5 [04 00 8d]. */
void step_a4_pcm_chan_init_ldc_0x00_r5_b(void)
{
    ldc_cr(5, 0x00);
    mcu.pc = 0x1427;
}

/* 0x1427 LDC #0x00 r4 [04 00 8c]. */
void step_a4_pcm_chan_init_ldc_0x00_r4_a(void)
{
    ldc_cr(4, 0x00);
    mcu.pc = 0x142a;
}

/* 0x142a LDC #0xff r3 [04 ff 8b]. */
void step_a4_pcm_chan_init_ldc_0xff_r3(void)
{
    ldc_cr(3, 0xff);
    mcu.pc = 0x142d;
}

/* 0x142d ret (pop cp,pc) [11 19]. */
void step_a4_pcm_chan_init_ret(void)
{
    ret_cp();
}

/* ======================================================================
 * A5 maint_reset32 0x329f..0x33c2 (flat 0x4329F-0x433C2), 75 PC.
 * Program-wide reset: d1d6..d1dd=0xff (plus jsr 0x433C4 setting
 * d202/d204=0xd1de), d1ff/d22c/d1cd..d1cf/d206/ff8e=0xff, ff8c=0xfe,
 * clears d421/d200/d1cc/d22d; d435[32]=0 and d1ac[32]=4 (11 1.1); counters
 * d311/d6a4/d1d3/d1d4/d1d5=4/4/0x1e/3/4, ff f3/fff7/e402 bit twiddles,
 * then the trapa #0x11 / move r0 #0x80 / trapa #0x10 yield pair at 0x3370
 * and the decrement/refill chains, ending in bsr/jsr to the gen-owned
 * callees 0x433D1/0x43641/0x434CB/0x4342A and BRA back to 0x3370.
 * Entry via the 0x414A2 stub (`jmp 0x329f`). Evidence: 07 A5 row; dasm
 * 13839-13968; R12. Callees stay outside this fragment.
 * ====================================================================== */

/* 0x329f LDC #0x00 r5 [04 00 8d]. */
void step_a5_maint_reset32_ldc_0x00_r5(void)
{
    ldc_cr(5, 0x00);
    mcu.pc = 0x32a2;
}

/* 0x32a2 LDC #0x00 r4 [04 00 8c]. */
void step_a5_maint_reset32_ldc_0x00_r4(void)
{
    ldc_cr(4, 0x00);
    mcu.pc = 0x32a5;
}

/* 0x32a5 MOVG #0x00ff -> (dp,0xd1d6) byte [15 d1 d6 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1d6_byte(void)
{
    store8_dp(0xd1d6, 0xff);
    mcu.pc = 0x32ab;
}

/* 0x32ab MOVG #0x00ff -> (dp,0xd1d7) byte [15 d1 d7 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1d7_byte(void)
{
    store8_dp(0xd1d7, 0xff);
    mcu.pc = 0x32b1;
}

/* 0x32b1 MOVG #0x00ff -> (dp,0xd1d8) byte [15 d1 d8 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1d8_byte(void)
{
    store8_dp(0xd1d8, 0xff);
    mcu.pc = 0x32b7;
}

/* 0x32b7 MOVG #0x00ff -> (dp,0xd1d9) byte [15 d1 d9 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1d9_byte(void)
{
    store8_dp(0xd1d9, 0xff);
    mcu.pc = 0x32bd;
}

/* 0x32bd MOVG #0x00ff -> (dp,0xd1da) byte [15 d1 da 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1da_byte(void)
{
    store8_dp(0xd1da, 0xff);
    mcu.pc = 0x32c3;
}

/* 0x32c3 MOVG #0x00ff -> (dp,0xd1db) byte [15 d1 db 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1db_byte(void)
{
    store8_dp(0xd1db, 0xff);
    mcu.pc = 0x32c9;
}

/* 0x32c9 MOVG #0x00ff -> (dp,0xd1dc) byte [15 d1 dc 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1dc_byte(void)
{
    store8_dp(0xd1dc, 0xff);
    mcu.pc = 0x32cf;
}

/* 0x32cf MOVG #0x00ff -> (dp,0xd1dd) byte [15 d1 dd 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1dd_byte(void)
{
    store8_dp(0xd1dd, 0xff);
    mcu.pc = 0x32d5;
}

/* 0x32d5 jsr #0x33c4 (d202/d204 = 0xd1de, gen-owned) [18 33 c4]. */
void step_a5_maint_reset32_jsr_0x33c4(void)
{
    jsr(0x32d8, 0x33c4);
}

/* 0x32d8 MOVG #0x00ff -> (dp,0xd1ff) byte [15 d1 ff 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1ff_byte(void)
{
    store8_dp(0xd1ff, 0xff);
    mcu.pc = 0x32de;
}

/* 0x32de CLR (dp,0xd421) byte [15 d4 21 13]. */
void step_a5_maint_reset32_clr_dp_0xd421_byte(void)
{
    clr8_dp(0xd421);
    mcu.pc = 0x32e2;
}

/* 0x32e2 CLR (dp,0xd200) byte [15 d2 00 13]. */
void step_a5_maint_reset32_clr_dp_0xd200_byte(void)
{
    clr8_dp(0xd200);
    mcu.pc = 0x32e6;
}

/* 0x32e6 CLR (dp,0xd1cc) byte [15 d1 cc 13]. */
void step_a5_maint_reset32_clr_dp_0xd1cc_byte(void)
{
    clr8_dp(0xd1cc);
    mcu.pc = 0x32ea;
}

/* 0x32ea CLR (dp,0xd22d) byte [15 d2 2d 13]. */
void step_a5_maint_reset32_clr_dp_0xd22d_byte(void)
{
    clr8_dp(0xd22d);
    mcu.pc = 0x32ee;
}

/* 0x32ee MOVG #0x00ff -> (dp,0xd22c) byte [15 d2 2c 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd22c_byte(void)
{
    store8_dp(0xd22c, 0xff);
    mcu.pc = 0x32f4;
}

/* 0x32f4 MOVG #0x00ff -> (dp,0xd1cd) byte [15 d1 cd 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1cd_byte(void)
{
    store8_dp(0xd1cd, 0xff);
    mcu.pc = 0x32fa;
}

/* 0x32fa MOVG #0x00ff -> (dp,0xd1ce) byte [15 d1 ce 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1ce_byte(void)
{
    store8_dp(0xd1ce, 0xff);
    mcu.pc = 0x3300;
}

/* 0x3300 MOVG #0x00ff -> (dp,0xd1cf) byte [15 d1 cf 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1cf_byte(void)
{
    store8_dp(0xd1cf, 0xff);
    mcu.pc = 0x3306;
}

/* 0x3306 MOVG #0x00ff -> (dp,0xd206) byte [15 d2 06 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xd206_byte(void)
{
    store8_dp(0xd206, 0xff);
    mcu.pc = 0x330c;
}

/* 0x330c MOVG #0x00ff -> (dp,0xff8e) byte [15 ff 8e 07 00 ff]. */
void step_a5_maint_reset32_movg_0x00ff_to_dp_0xff8e_byte(void)
{
    store8_dp(0xff8e, 0xff);
    mcu.pc = 0x3312;
}

/* 0x3312 MOVG #0x00fe -> (dp,0xff8c) byte [15 ff 8c 07 00 fe]. */
void step_a5_maint_reset32_movg_0x00fe_to_dp_0xff8c_byte(void)
{
    store8_dp(0xff8c, 0xfe);
    mcu.pc = 0x3318;
}

/* 0x3318 movi r1 #0xd435 [59 d4 35]. */
void step_a5_maint_reset32_movi_r1_0xd435(void)
{
    movi16(mcu.r[1], 0xd435);
    mcu.pc = 0x331b;
}

/* 0x331b movi r0 #0x001f [58 00 1f]. */
void step_a5_maint_reset32_movi_r0_0x001f(void)
{
    movi16(mcu.r[0], 0x001f);
    mcu.pc = 0x331e;
}

/* 0x331e CLR r1++ byte [c1 13]. */
void step_a5_maint_reset32_clr_r1_postinc_byte(void)
{
    clr8_postinc(1);
    mcu.pc = 0x3320;
}

/* 0x3320 cntjmp r0 -5 -> 0x331e [01 b8 fb]. */
void step_a5_maint_reset32_cntjmp_r0_5_to_0x331e(void)
{
    cntjmp(mcu.r[0], 0x331e, 0x3323);
}

/* 0x3323 movi r1 #0xd1ac [59 d1 ac]. */
void step_a5_maint_reset32_movi_r1_0xd1ac(void)
{
    movi16(mcu.r[1], 0xd1ac);
    mcu.pc = 0x3326;
}

/* 0x3326 movi r0 #0x001f [58 00 1f]. */
void step_a5_maint_reset32_movi_r0_0x001f_a(void)
{
    movi16(mcu.r[0], 0x001f);
    mcu.pc = 0x3329;
}

/* 0x3329 MOVG #0x04 -> r1++ byte [c1 06 04]. */
void step_a5_maint_reset32_movg_0x04_to_r1_postinc_byte(void)
{
    store8_postinc(1, 0x04);
    mcu.pc = 0x332c;
}

/* 0x332c cntjmp r0 -6 -> 0x3329 [01 b8 fa]. */
void step_a5_maint_reset32_cntjmp_r0_6_to_0x3329(void)
{
    cntjmp(mcu.r[0], 0x3329, 0x332f);
}

/* 0x332f MOVG #0x04 -> (dp,0xd311) byte [15 d3 11 06 04]. */
void step_a5_maint_reset32_movg_0x04_to_dp_0xd311_byte(void)
{
    store8_dp(0xd311, 0x04);
    mcu.pc = 0x3334;
}

/* 0x3334 MOVG #0x04 -> (dp,0xd6a4) byte [15 d6 a4 06 04]. */
void step_a5_maint_reset32_movg_0x04_to_dp_0xd6a4_byte(void)
{
    store8_dp(0xd6a4, 0x04);
    mcu.pc = 0x3339;
}

/* 0x3339 MOVG #0x1e -> (dp,0xd1d3) byte [15 d1 d3 06 1e]. */
void step_a5_maint_reset32_movg_0x1e_to_dp_0xd1d3_byte(void)
{
    store8_dp(0xd1d3, 0x1e);
    mcu.pc = 0x333e;
}

/* 0x333e MOVG #0x03 -> (dp,0xd1d4) byte [15 d1 d4 06 03]. */
void step_a5_maint_reset32_movg_0x03_to_dp_0xd1d4_byte(void)
{
    store8_dp(0xd1d4, 0x03);
    mcu.pc = 0x3343;
}

/* 0x3343 MOVG #0x04 -> (dp,0xd1d5) byte [15 d1 d5 06 04]. */
void step_a5_maint_reset32_movg_0x04_to_dp_0xd1d5_byte(void)
{
    store8_dp(0xd1d5, 0x04);
    mcu.pc = 0x3348;
}

/* 0x3348 MOVG2 (dp,0xfff3) r0 byte [15 ff f3 80]. */
void step_a5_maint_reset32_movg2_dp_0xfff3_r0_byte(void)
{
    load8_via(mcu.r[0], dp_addr(0xfff3));
    mcu.pc = 0x334c;
}

/* 0x334c AND #0xf8 r0 byte [04 f8 50]. */
void step_a5_maint_reset32_and_0xf8_r0_byte(void)
{
    and_imm8_reg(mcu.r[0], 0xf8);
    mcu.pc = 0x334f;
}

/* 0x334f OR #0x07 r0 byte flags [04 07 40]. */
void step_a5_maint_reset32_or_0x07_r0_byte(void)
{
    or_imm8_reg(mcu.r[0], 0x07);
    mcu.pc = 0x3352;
}

/* 0x3352 MOVG3 r0 -> (dp,0xfff3) byte [15 ff f3 90]. */
void step_a5_maint_reset32_movg3_r0_to_dp_0xfff3_byte(void)
{
    store8_dp(0xfff3, (uint8_t)mcu.r[0]);
    mcu.pc = 0x3356;
}

/* 0x3356 MOVG2 (dp,0xfff7) r0 byte [15 ff f7 80]. */
void step_a5_maint_reset32_movg2_dp_0xfff7_r0_byte(void)
{
    load8_via(mcu.r[0], dp_addr(0xfff7));
    mcu.pc = 0x335a;
}

/* 0x335a AND #0xfe r0 byte [04 fe 50]. */
void step_a5_maint_reset32_and_0xfe_r0_byte(void)
{
    and_imm8_reg(mcu.r[0], 0xfe);
    mcu.pc = 0x335d;
}

/* 0x335d MOVG3 r0 -> (dp,0xfff7) byte [15 ff f7 90]. */
void step_a5_maint_reset32_movg3_r0_to_dp_0xfff7_byte(void)
{
    store8_dp(0xfff7, (uint8_t)mcu.r[0]);
    mcu.pc = 0x3361;
}

/* 0x3361 MOVG2 (dp,0xd45c) r6 byte [15 d4 5c 86]. */
void step_a5_maint_reset32_movg2_dp_0xd45c_r6_byte(void)
{
    load8_via(mcu.r[6], dp_addr(0xd45c));
    mcu.pc = 0x3365;
}

/* 0x3365 OR #0x02 r6 byte flags [04 02 46]. */
void step_a5_maint_reset32_or_0x02_r6_byte(void)
{
    or_imm8_reg(mcu.r[6], 0x02);
    mcu.pc = 0x3368;
}

/* 0x3368 MOVG3 r6 -> (dp,0xd45c) byte [15 d4 5c 96]. */
void step_a5_maint_reset32_movg3_r6_to_dp_0xd45c_byte(void)
{
    store8_dp(0xd45c, (uint8_t)mcu.r[6]);
    mcu.pc = 0x336c;
}

/* 0x336c MOVG3 r6 -> (dp,0xe402) byte [15 e4 02 96]. */
void step_a5_maint_reset32_movg3_r6_to_dp_0xe402_byte(void)
{
    store8_dp(0xe402, (uint8_t)mcu.r[6]);
    mcu.pc = 0x3370;
}

/* 0x3370 movi r0 #0x000a [58 00 0a]. */
void step_a5_maint_reset32_movi_r0_0x000a(void)
{
    movi16(mcu.r[0], 0x000a);
    mcu.pc = 0x3373;
}

/* 0x3373 trapa #0x11 [08 11]. */
void step_a5_maint_reset32_trapa_0x11(void)
{
    trapa(0x11);
    mcu.pc = 0x3375;
}

/* 0x3375 move r0 #0x80 [50 80]. */
void step_a5_maint_reset32_move_r0_0x80(void)
{
    move8(mcu.r[0], 0x80);
    mcu.pc = 0x3377;
}

/* 0x3377 trapa #0x10 [08 10]. */
void step_a5_maint_reset32_trapa_0x10(void)
{
    trapa(0x10);
    mcu.pc = 0x3379;
}

/* 0x3379 MOVG2 (dp,0xd1d3) r0 byte [15 d1 d3 80]. */
void step_a5_maint_reset32_movg2_dp_0xd1d3_r0_byte(void)
{
    load8_via(mcu.r[0], dp_addr(0xd1d3));
    mcu.pc = 0x337d;
}

/* 0x337d ADDQ #-1 r0 byte [a0 0c]. */
void step_a5_maint_reset32_addq_m1_r0_byte(void)
{
    addq_byte_reg(mcu.r[0], -1);
    mcu.pc = 0x337f;
}

/* 0x337f BEQ 6 -> 0x3387 [27 06]. */
void step_a5_maint_reset32_beq_0x3387(void)
{
    beq(0x3387, 0x3381);
}

/* 0x3381 MOVG3 r0 -> (dp,0xd1d3) byte [15 d1 d3 90]. */
void step_a5_maint_reset32_movg3_r0_to_dp_0xd1d3_byte(void)
{
    store8_dp(0xd1d3, (uint8_t)mcu.r[0]);
    mcu.pc = 0x3385;
}

/* 0x3385 BRA 29 -> 0x33a4 [20 1d]. */
void step_a5_maint_reset32_bra_0x33a4(void)
{
    mcu.pc = 0x33a4;
}

/* 0x3387 MOVG #0x1e -> (dp,0xd1d3) byte [15 d1 d3 06 1e]. */
void step_a5_maint_reset32_movg_0x1e_to_dp_0xd1d3_byte_a(void)
{
    store8_dp(0xd1d3, 0x1e);
    mcu.pc = 0x338c;
}

/* 0x338c bsr 67 -> 0x33d1 (gen-owned) [0e 43]. */
void step_a5_maint_reset32_bsr_to_0x33d1(void)
{
    bsr8(0x338e, 0x33d1);
}

/* 0x338e ADDQ #-1 (dp,0xd6a4) byte [15 d6 a4 0c]. */
void step_a5_maint_reset32_addq_m1_dp_0xd6a4_byte(void)
{
    addq_byte_mem(dp_addr(0xd6a4), -1);
    mcu.pc = 0x3392;
}

/* 0x3392 BNE 5 -> 0x3399 [26 05]. */
void step_a5_maint_reset32_bne_0x3399(void)
{
    bne(0x3399, 0x3394);
}

/* 0x3394 MOVG #0x03 -> (dp,0xd6a4) byte [15 d6 a4 06 03]. */
void step_a5_maint_reset32_movg_0x03_to_dp_0xd6a4_byte(void)
{
    store8_dp(0xd6a4, 0x03);
    mcu.pc = 0x3399;
}

/* 0x3399 ADDQ #-1 (dp,0xd311) byte [15 d3 11 0c]. */
void step_a5_maint_reset32_addq_m1_dp_0xd311_byte(void)
{
    addq_byte_mem(dp_addr(0xd311), -1);
    mcu.pc = 0x339d;
}

/* 0x339d BNE 5 -> 0x33a4 [26 05]. */
void step_a5_maint_reset32_bne_0x33a4(void)
{
    bne(0x33a4, 0x339f);
}

/* 0x339f MOVG #0x03 -> (dp,0xd311) byte [15 d3 11 06 03]. */
void step_a5_maint_reset32_movg_0x03_to_dp_0xd311_byte(void)
{
    store8_dp(0xd311, 0x03);
    mcu.pc = 0x33a4;
}

/* 0x33a4 MOVG2 (dp,0xd1d4) r0 byte [15 d1 d4 80]. */
void step_a5_maint_reset32_movg2_dp_0xd1d4_r0_byte(void)
{
    load8_via(mcu.r[0], dp_addr(0xd1d4));
    mcu.pc = 0x33a8;
}

/* 0x33a8 ADDQ #-1 r0 byte [a0 0c]. */
void step_a5_maint_reset32_addq_m1_r0_byte_a(void)
{
    addq_byte_reg(mcu.r[0], -1);
    mcu.pc = 0x33aa;
}

/* 0x33aa BEQ 6 -> 0x33b2 [27 06]. */
void step_a5_maint_reset32_beq_0x33b2(void)
{
    beq(0x33b2, 0x33ac);
}

/* 0x33ac MOVG3 r0 -> (dp,0xd1d4) byte [15 d1 d4 90]. */
void step_a5_maint_reset32_movg3_r0_to_dp_0xd1d4_byte(void)
{
    store8_dp(0xd1d4, (uint8_t)mcu.r[0]);
    mcu.pc = 0x33b0;
}

/* 0x33b0 BRA 7 -> 0x33b9 [20 07]. */
void step_a5_maint_reset32_bra_0x33b9(void)
{
    mcu.pc = 0x33b9;
}

/* 0x33b2 MOVG #0x03 -> (dp,0xd1d4) byte [15 d1 d4 06 03]. */
void step_a5_maint_reset32_movg_0x03_to_dp_0xd1d4_byte_a(void)
{
    store8_dp(0xd1d4, 0x03);
    mcu.pc = 0x33b7;
}

/* 0x33b7 bsr 49 -> 0x33ea (gen-owned) [0e 31]. */
void step_a5_maint_reset32_bsr_to_0x33ea(void)
{
    bsr8(0x33b9, 0x33ea);
}

/* 0x33b9 jsr #0x3641 (bit0 scan, gen-owned) [18 36 41]. */
void step_a5_maint_reset32_jsr_0x3641(void)
{
    jsr(0x33bc, 0x3641);
}

/* 0x33bc jsr #0x34cb (gen-owned) [18 34 cb]. */
void step_a5_maint_reset32_jsr_0x34cb(void)
{
    jsr(0x33bf, 0x34cb);
}

/* 0x33bf jsr #0x342a (gen-owned) [18 34 2a]. */
void step_a5_maint_reset32_jsr_0x342a(void)
{
    jsr(0x33c2, 0x342a);
}

/* 0x33c2 BRA -84 -> 0x3370 [20 ac]. */
void step_a5_maint_reset32_bra_0x3370(void)
{
    mcu.pc = 0x3370;
}

/* ======================================================================
 * 0x42B8A 0x2b8a..0x2b98 (flat 0x42B8A-0x42B98), 5 PC.
 * A3's per-voice loop body: clear the two AoS words P-108/P-74 and the
 * global ce36/ce38, then rts. Entry bsr16 from 0x41315 (next 0x41318).
 * Evidence: dasm_full.txt:13477-13482; 18 1.1 row 32 / 07:375.
 * ====================================================================== */

/* 0x2b8a CLR @r0+-108 word [e8 94 13]. */
void step_42b8a_clr_r0_m108_word(void)
{
    clr16_ind_disp8(0, (int8_t)0x94);
    mcu.pc = 0x2b8d;
}

/* 0x2b8d CLR @r0+-74 word [e8 b6 13]. */
void step_42b8a_clr_r0_m74_word(void)
{
    clr16_ind_disp8(0, (int8_t)0xb6);
    mcu.pc = 0x2b90;
}

/* 0x2b90 CLR (dp,0xce36) word [1d ce 36 13]. */
void step_42b8a_clr_dp_0xce36_word(void)
{
    clr16_dp(0xce36);
    mcu.pc = 0x2b94;
}

/* 0x2b94 CLR (dp,0xce38) word [1d ce 38 13]. */
void step_42b8a_clr_dp_0xce38_word(void)
{
    clr16_dp(0xce38);
    mcu.pc = 0x2b98;
}

/* 0x2b98 rts [19]. */
void step_42b8a_rts(void)
{
    rts();
}

/* ======================================================================
 * d1ac helper family 0x3695..0x36db (flat 0x43695-0x436DB), 25 PC.
 *
 *   0x3695   d1ac[r0] |= 0x04            (set bit2, single)
 *   0x36a3   d1ac[31..0] |= 0x04         (set bit2, all 32; r0=0x1f seed)
 *   0x36b5   d1ac[r0] &= 0xfb            (clear bit2, single)
 *   0x36c3   d1ac[31..0] &= 0xfb         (clear bit2, all 32)
 *   0x36d5   BCLR @r0+0xd1ac #1          (clear bit1, single)
 *
 * The array is a fixed 32-byte page-0 block and the index is the raw r0
 * byte (11 1.1: non-slot index; the all-32 loops start at r0=0x1f and end
 * when cntjmp wraps r0 to 0xffff). d1ac stays authoritative in SRAM.
 * 0x36b5 has been executed (pc_main:7277, callers 0x448de/e3/e8 with
 * r0=0/6/5); the other entries are not in the recorded runs and their
 * bodies come straight from the ROM bytes (rom2 fileoff 0x3695-0x36db),
 * cross-checked against the gen decoding of the shared loop bodies
 * (0x3697/0x36a6/0x36b7/0x36c6): C bytes / I dynamic.
 * ====================================================================== */

/* 0x3695 EXTU r0 [a0 12]. */
void step_d1ac_extu_r0(void)
{
    extu(mcu.r[0]);
    mcu.pc = 0x3697;
}

/* 0x3697 MOVG2 @r0+0xd1ac r1 byte [f0 d1 ac 81]. */
void step_d1ac_movg2_r0_0xd1ac_r1_byte(void)
{
    load8_ind(mcu.r[1], 0, 0xd1ac);
    mcu.pc = 0x369b;
}

/* 0x369b OR #0x04 r1 byte flags [04 04 41]. */
void step_d1ac_or_0x04_r1_byte(void)
{
    or_imm8_reg(mcu.r[1], 0x04);
    mcu.pc = 0x369e;
}

/* 0x369e MOVG3 r1 -> @r0+0xd1ac byte [f0 d1 ac 91]. */
void step_d1ac_movg3_r1_to_r0_0xd1ac_byte(void)
{
    store8_ind(0, 0xd1ac, (uint8_t)mcu.r[1]);
    mcu.pc = 0x36a2;
}

/* 0x36a2 rts [19]. */
void step_d1ac_rts(void)
{
    rts();
}

/* 0x36a3 movi r0 #0x001f [58 00 1f]. */
void step_d1ac_movi_r0_0x001f(void)
{
    movi16(mcu.r[0], 0x001f);
    mcu.pc = 0x36a6;
}

/* 0x36a6 MOVG2 @r0+0xd1ac r1 byte [f0 d1 ac 81]. */
void step_d1ac_movg2_r0_0xd1ac_r1_byte_a(void)
{
    load8_ind(mcu.r[1], 0, 0xd1ac);
    mcu.pc = 0x36aa;
}

/* 0x36aa OR #0x04 r1 byte flags [04 04 41]. */
void step_d1ac_or_0x04_r1_byte_a(void)
{
    or_imm8_reg(mcu.r[1], 0x04);
    mcu.pc = 0x36ad;
}

/* 0x36ad MOVG3 r1 -> @r0+0xd1ac byte [f0 d1 ac 91]. */
void step_d1ac_movg3_r1_to_r0_0xd1ac_byte_a(void)
{
    store8_ind(0, 0xd1ac, (uint8_t)mcu.r[1]);
    mcu.pc = 0x36b1;
}

/* 0x36b1 cntjmp r0 -14 -> 0x36a6 [01 b8 f2]. */
void step_d1ac_cntjmp_r0_14_to_0x36a6(void)
{
    cntjmp(mcu.r[0], 0x36a6, 0x36b4);
}

/* 0x36b4 rts [19]. */
void step_d1ac_rts_a(void)
{
    rts();
}

/* 0x36b5 EXTU r0 [a0 12]. */
void step_d1ac_extu_r0_a(void)
{
    extu(mcu.r[0]);
    mcu.pc = 0x36b7;
}

/* 0x36b7 MOVG2 @r0+0xd1ac r1 byte [f0 d1 ac 81]. */
void step_d1ac_movg2_r0_0xd1ac_r1_byte_b(void)
{
    load8_ind(mcu.r[1], 0, 0xd1ac);
    mcu.pc = 0x36bb;
}

/* 0x36bb AND #0xfb r1 byte [04 fb 51]. */
void step_d1ac_and_0xfb_r1_byte(void)
{
    and_imm8_reg(mcu.r[1], 0xfb);
    mcu.pc = 0x36be;
}

/* 0x36be MOVG3 r1 -> @r0+0xd1ac byte [f0 d1 ac 91]. */
void step_d1ac_movg3_r1_to_r0_0xd1ac_byte_b(void)
{
    store8_ind(0, 0xd1ac, (uint8_t)mcu.r[1]);
    mcu.pc = 0x36c2;
}

/* 0x36c2 rts [19]. */
void step_d1ac_rts_b(void)
{
    rts();
}

/* 0x36c3 movi r0 #0x001f [58 00 1f]. */
void step_d1ac_movi_r0_0x001f_a(void)
{
    movi16(mcu.r[0], 0x001f);
    mcu.pc = 0x36c6;
}

/* 0x36c6 MOVG2 @r0+0xd1ac r1 byte [f0 d1 ac 81]. */
void step_d1ac_movg2_r0_0xd1ac_r1_byte_c(void)
{
    load8_ind(mcu.r[1], 0, 0xd1ac);
    mcu.pc = 0x36ca;
}

/* 0x36ca AND #0xfb r1 byte [04 fb 51]. */
void step_d1ac_and_0xfb_r1_byte_a(void)
{
    and_imm8_reg(mcu.r[1], 0xfb);
    mcu.pc = 0x36cd;
}

/* 0x36cd MOVG3 r1 -> @r0+0xd1ac byte [f0 d1 ac 91]. */
void step_d1ac_movg3_r1_to_r0_0xd1ac_byte_c(void)
{
    store8_ind(0, 0xd1ac, (uint8_t)mcu.r[1]);
    mcu.pc = 0x36d1;
}

/* 0x36d1 cntjmp r0 -14 -> 0x36c6 [01 b8 f2]. */
void step_d1ac_cntjmp_r0_14_to_0x36c6(void)
{
    cntjmp(mcu.r[0], 0x36c6, 0x36d4);
}

/* 0x36d4 rts [19]. */
void step_d1ac_rts_c(void)
{
    rts();
}

/* 0x36d5 EXTU r0 [a0 12]. */
void step_d1ac_extu_r0_b(void)
{
    extu(mcu.r[0]);
    mcu.pc = 0x36d7;
}

/* 0x36d7 BCLR @r0+0xd1ac #1 [f0 d1 ac d1]. */
void step_d1ac_bclr_r0_0xd1ac_1(void)
{
    bclr8_mem(ind_addr(0, 0xd1ac), 1);
    mcu.pc = 0x36db;
}

/* 0x36db rts [19]. */
void step_d1ac_rts_d(void)
{
    rts();
}

} /* anonymous namespace */
} /* namespace mk2c */

void MK2CPP_ResetInitFillTables(void)
{
#if MK2CPP_HAND_RESET_INIT
    /* A2 init_desc 0x1220..0x1263, 20 PCs */
    MK2CPP_HandRegister(0x00041220u, &mk2c::step_a2_init_desc_ldc_0x00_r5);
    MK2CPP_HandRegister(0x00041223u, &mk2c::step_a2_init_desc_ldc_0x00_r4);
    MK2CPP_HandRegister(0x00041226u, &mk2c::step_a2_init_desc_ldc_0xe0_r3);
    MK2CPP_HandRegister(0x00041229u, &mk2c::step_a2_init_desc_clr_dp_0xd150_word);
    MK2CPP_HandRegister(0x0004122du, &mk2c::step_a2_init_desc_clr_dp_0xd152_word);
    MK2CPP_HandRegister(0x00041231u, &mk2c::step_a2_init_desc_clr_dp_0xd154_word);
    MK2CPP_HandRegister(0x00041235u, &mk2c::step_a2_init_desc_clr_dp_0xd156_word);
    MK2CPP_HandRegister(0x00041239u, &mk2c::step_a2_init_desc_movi_r0_0x001b);
    MK2CPP_HandRegister(0x0004123cu, &mk2c::step_a2_init_desc_movg2_r0_r2_word);
    MK2CPP_HandRegister(0x0004123eu, &mk2c::step_a2_init_desc_add_r2_r2_word);
    MK2CPP_HandRegister(0x00041240u, &mk2c::step_a2_init_desc_movg2_r2_0x64d6_r1_word);
    MK2CPP_HandRegister(0x00041244u, &mk2c::step_a2_init_desc_clr_r0_0xad0e_byte);
    MK2CPP_HandRegister(0x00041248u, &mk2c::step_a2_init_desc_movg_0x0016_to_r1_0_word);
    MK2CPP_HandRegister(0x0004124du, &mk2c::step_a2_init_desc_movg_0x0016_to_r1_2_word);
    MK2CPP_HandRegister(0x00041252u, &mk2c::step_a2_init_desc_movg_0x0016_to_r1_4_word);
    MK2CPP_HandRegister(0x00041257u, &mk2c::step_a2_init_desc_clr_r0_0xd15c_byte);
    MK2CPP_HandRegister(0x0004125bu, &mk2c::step_a2_init_desc_cntjmp_r0_34_to_0x123c);
    MK2CPP_HandRegister(0x0004125eu, &mk2c::step_a2_init_desc_movi_r0_0x0001);
    MK2CPP_HandRegister(0x00041261u, &mk2c::step_a2_init_desc_trapa_0x11);
    MK2CPP_HandRegister(0x00041263u, &mk2c::step_a2_init_desc_ret);

    /* A3 init_params 0x1266..0x1324, 54 PCs */
    MK2CPP_HandRegister(0x00041266u, &mk2c::step_a3_init_params_move_r0_0x02);
    MK2CPP_HandRegister(0x00041268u, &mk2c::step_a3_init_params_trapa_0x17);
    MK2CPP_HandRegister(0x0004126au, &mk2c::step_a3_init_params_ldc_0x00_r5);
    MK2CPP_HandRegister(0x0004126du, &mk2c::step_a3_init_params_ldc_0x00_r4);
    MK2CPP_HandRegister(0x00041270u, &mk2c::step_a3_init_params_ldc_0xe0_r3);
    MK2CPP_HandRegister(0x00041273u, &mk2c::step_a3_init_params_clr_dp_0xd1a0_byte);
    MK2CPP_HandRegister(0x00041277u, &mk2c::step_a3_init_params_clr_dp_0xd1a1_byte);
    MK2CPP_HandRegister(0x0004127bu, &mk2c::step_a3_init_params_clr_dp_0xd18e_word);
    MK2CPP_HandRegister(0x0004127fu, &mk2c::step_a3_init_params_clr_dp_0xd194_byte);
    MK2CPP_HandRegister(0x00041283u, &mk2c::step_a3_init_params_clr_dp_0xd195_byte);
    MK2CPP_HandRegister(0x00041287u, &mk2c::step_a3_init_params_clr_dp_0xd196_byte);
    MK2CPP_HandRegister(0x0004128bu, &mk2c::step_a3_init_params_movg_0x0002_to_dp_0xd19c_word);
    MK2CPP_HandRegister(0x00041291u, &mk2c::step_a3_init_params_movg_0x0002_to_dp_0xd19e_word);
    MK2CPP_HandRegister(0x00041297u, &mk2c::step_a3_init_params_movg2_dp_0x802b_r6_byte);
    MK2CPP_HandRegister(0x0004129bu, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd181_byte);
    MK2CPP_HandRegister(0x0004129fu, &mk2c::step_a3_init_params_movg2_dp_0x802c_r6_byte);
    MK2CPP_HandRegister(0x000412a3u, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd182_byte);
    MK2CPP_HandRegister(0x000412a7u, &mk2c::step_a3_init_params_movg2_dp_0x802d_r6_byte);
    MK2CPP_HandRegister(0x000412abu, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd183_byte);
    MK2CPP_HandRegister(0x000412afu, &mk2c::step_a3_init_params_movg2_dp_0x802e_r6_byte);
    MK2CPP_HandRegister(0x000412b3u, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd184_byte);
    MK2CPP_HandRegister(0x000412b7u, &mk2c::step_a3_init_params_movg2_dp_0x802f_r6_byte);
    MK2CPP_HandRegister(0x000412bbu, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd185_byte);
    MK2CPP_HandRegister(0x000412bfu, &mk2c::step_a3_init_params_movg2_dp_0x8030_r6_byte);
    MK2CPP_HandRegister(0x000412c3u, &mk2c::step_a3_init_params_shlr_r6_byte);
    MK2CPP_HandRegister(0x000412c5u, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd186_byte);
    MK2CPP_HandRegister(0x000412c9u, &mk2c::step_a3_init_params_movg2_dp_0x8033_r6_byte);
    MK2CPP_HandRegister(0x000412cdu, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd187_byte);
    MK2CPP_HandRegister(0x000412d1u, &mk2c::step_a3_init_params_movg2_dp_0x8034_r6_byte);
    MK2CPP_HandRegister(0x000412d5u, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd188_byte);
    MK2CPP_HandRegister(0x000412d9u, &mk2c::step_a3_init_params_movg2_dp_0x8035_r6_byte);
    MK2CPP_HandRegister(0x000412ddu, &mk2c::step_a3_init_params_shlr_r6_byte_a);
    MK2CPP_HandRegister(0x000412dfu, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd189_byte);
    MK2CPP_HandRegister(0x000412e3u, &mk2c::step_a3_init_params_movg2_dp_0x8036_r6_byte);
    MK2CPP_HandRegister(0x000412e7u, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd18a_byte);
    MK2CPP_HandRegister(0x000412ebu, &mk2c::step_a3_init_params_movg2_dp_0x8037_r6_byte);
    MK2CPP_HandRegister(0x000412efu, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd18b_byte);
    MK2CPP_HandRegister(0x000412f3u, &mk2c::step_a3_init_params_movg2_dp_0x8038_r6_byte);
    MK2CPP_HandRegister(0x000412f7u, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd18c_byte);
    MK2CPP_HandRegister(0x000412fbu, &mk2c::step_a3_init_params_movg2_dp_0x8039_r6_byte);
    MK2CPP_HandRegister(0x000412ffu, &mk2c::step_a3_init_params_shlr_r6_byte_b);
    MK2CPP_HandRegister(0x00041301u, &mk2c::step_a3_init_params_movg3_r6_to_dp_0xd18d_byte);
    MK2CPP_HandRegister(0x00041305u, &mk2c::step_a3_init_params_movi_r1_0x001b);
    MK2CPP_HandRegister(0x00041308u, &mk2c::step_a3_init_params_movg2_r1_r2_word);
    MK2CPP_HandRegister(0x0004130au, &mk2c::step_a3_init_params_shll_r2_word);
    MK2CPP_HandRegister(0x0004130cu, &mk2c::step_a3_init_params_movg2_r2_0x64d6_r0_word);
    MK2CPP_HandRegister(0x00041310u, &mk2c::step_a3_init_params_movg_0x00ff_to_r0_m26_byte);
    MK2CPP_HandRegister(0x00041315u, &mk2c::step_a3_init_params_bsr16_to_0x2b8a);
    MK2CPP_HandRegister(0x00041318u, &mk2c::step_a3_init_params_cntjmp_r1_19_to_0x1308);
    MK2CPP_HandRegister(0x0004131bu, &mk2c::step_a3_init_params_move_r0_0x02_a);
    MK2CPP_HandRegister(0x0004131du, &mk2c::step_a3_init_params_trapa_0x18);
    MK2CPP_HandRegister(0x0004131fu, &mk2c::step_a3_init_params_movi_r0_0x0008);
    MK2CPP_HandRegister(0x00041322u, &mk2c::step_a3_init_params_trapa_0x11);
    MK2CPP_HandRegister(0x00041324u, &mk2c::step_a3_init_params_ret);

    /* A4 pcm_chan_init 0x1333..0x142d, 91 PCs */
    MK2CPP_HandRegister(0x00041333u, &mk2c::step_a4_pcm_chan_init_ldc_0x00_r5);
    MK2CPP_HandRegister(0x00041336u, &mk2c::step_a4_pcm_chan_init_ldc_0x00_r4);
    MK2CPP_HandRegister(0x00041339u, &mk2c::step_a4_pcm_chan_init_ldc_0xe0_r3);
    MK2CPP_HandRegister(0x0004133cu, &mk2c::step_a4_pcm_chan_init_clr_r0_word);
    MK2CPP_HandRegister(0x0004133eu, &mk2c::step_a4_pcm_chan_init_clr_r1_word);
    MK2CPP_HandRegister(0x00041340u, &mk2c::step_a4_pcm_chan_init_ldc_0x04_r5);
    MK2CPP_HandRegister(0x00041343u, &mk2c::step_a4_pcm_chan_init_movg2_r1_0x1432_r1_word);
    MK2CPP_HandRegister(0x00041347u, &mk2c::step_a4_pcm_chan_init_or_r1_r0_word);
    MK2CPP_HandRegister(0x00041349u, &mk2c::step_a4_pcm_chan_init_ldc_0x00_r5_a);
    MK2CPP_HandRegister(0x0004134cu, &mk2c::step_a4_pcm_chan_init_movg3_r0_to_br_0x3c_word);
    MK2CPP_HandRegister(0x0004134fu, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x00_word);
    MK2CPP_HandRegister(0x00041354u, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x02_word);
    MK2CPP_HandRegister(0x00041359u, &mk2c::step_a4_pcm_chan_init_movg2_br_0x3e_r1_byte);
    MK2CPP_HandRegister(0x0004135cu, &mk2c::step_a4_pcm_chan_init_movi_r0_0x001b);
    MK2CPP_HandRegister(0x0004135fu, &mk2c::step_a4_pcm_chan_init_movi_r1_0x00ba);
    MK2CPP_HandRegister(0x00041362u, &mk2c::step_a4_pcm_chan_init_movs_r0_br_0x3e);
    MK2CPP_HandRegister(0x00041364u, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x12_word);
    MK2CPP_HandRegister(0x00041369u, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x14_word);
    MK2CPP_HandRegister(0x0004136eu, &mk2c::step_a4_pcm_chan_init_movsw_r1_br_0x16);
    MK2CPP_HandRegister(0x00041370u, &mk2c::step_a4_pcm_chan_init_movsw_r1_br_0x18);
    MK2CPP_HandRegister(0x00041372u, &mk2c::step_a4_pcm_chan_init_movsw_r1_br_0x1a);
    MK2CPP_HandRegister(0x00041374u, &mk2c::step_a4_pcm_chan_init_cntjmp_r0_21_to_0x1362);
    MK2CPP_HandRegister(0x00041377u, &mk2c::step_a4_pcm_chan_init_movg_0x1e_to_br_0x3e_byte);
    MK2CPP_HandRegister(0x0004137bu, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x14_word_a);
    MK2CPP_HandRegister(0x00041380u, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x16_word);
    MK2CPP_HandRegister(0x00041385u, &mk2c::step_a4_pcm_chan_init_movg_0x1f_to_br_0x3e_byte);
    MK2CPP_HandRegister(0x00041389u, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x14_word_b);
    MK2CPP_HandRegister(0x0004138eu, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x16_word_a);
    MK2CPP_HandRegister(0x00041393u, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x18_word);
    MK2CPP_HandRegister(0x00041398u, &mk2c::step_a4_pcm_chan_init_movg_0x0000_to_br_0x1a_word);
    MK2CPP_HandRegister(0x0004139du, &mk2c::step_a4_pcm_chan_init_movg_0xffff_to_br_0x00_word);
    MK2CPP_HandRegister(0x000413a2u, &mk2c::step_a4_pcm_chan_init_movg_0xffff_to_br_0x02_word);
    MK2CPP_HandRegister(0x000413a7u, &mk2c::step_a4_pcm_chan_init_movl_r6_br_0x00_byte);
    MK2CPP_HandRegister(0x000413a9u, &mk2c::step_a4_pcm_chan_init_movg_0x00_signed_to_br_0x00_word);
    MK2CPP_HandRegister(0x000413adu, &mk2c::step_a4_pcm_chan_init_movg_0x00_signed_to_br_0x02_word);
    MK2CPP_HandRegister(0x000413b1u, &mk2c::step_a4_pcm_chan_init_movg2_br_0x3e_r1_byte_a);
    MK2CPP_HandRegister(0x000413b4u, &mk2c::step_a4_pcm_chan_init_movg_0x1e_to_br_0x3e_byte_a);
    MK2CPP_HandRegister(0x000413b8u, &mk2c::step_a4_pcm_chan_init_movg_0xffff_to_br_0x34_word);
    MK2CPP_HandRegister(0x000413bdu, &mk2c::step_a4_pcm_chan_init_movl_r5_br_0x34_byte);
    MK2CPP_HandRegister(0x000413bfu, &mk2c::step_a4_pcm_chan_init_movlw_r5_br_0x3a_word);
    MK2CPP_HandRegister(0x000413c1u, &mk2c::step_a4_pcm_chan_init_beq_0x13b4);
    MK2CPP_HandRegister(0x000413c3u, &mk2c::step_a4_pcm_chan_init_move_r4_0x1d);
    MK2CPP_HandRegister(0x000413c5u, &mk2c::step_a4_pcm_chan_init_movi_r6_0x3800);
    MK2CPP_HandRegister(0x000413c8u, &mk2c::step_a4_pcm_chan_init_movs_r4_br_0x3e);
    MK2CPP_HandRegister(0x000413cau, &mk2c::step_a4_pcm_chan_init_movsw_r6_br_0x32);
    MK2CPP_HandRegister(0x000413ccu, &mk2c::step_a4_pcm_chan_init_movi_r1_0x007f);
    MK2CPP_HandRegister(0x000413cfu, &mk2c::step_a4_pcm_chan_init_move_r6_0x1f);
    MK2CPP_HandRegister(0x000413d1u, &mk2c::step_a4_pcm_chan_init_movs_r6_br_0x3e);
    MK2CPP_HandRegister(0x000413d3u, &mk2c::step_a4_pcm_chan_init_movsw_r1_br_0x1e);
    MK2CPP_HandRegister(0x000413d5u, &mk2c::step_a4_pcm_chan_init_bsr_to_0x142f);
    MK2CPP_HandRegister(0x000413d7u, &mk2c::step_a4_pcm_chan_init_bsr_to_0x142f_a);
    MK2CPP_HandRegister(0x000413d9u, &mk2c::step_a4_pcm_chan_init_movsw_r1_br_0x1e_a);
    MK2CPP_HandRegister(0x000413dbu, &mk2c::step_a4_pcm_chan_init_movl_r4_br_0x1e_byte);
    MK2CPP_HandRegister(0x000413ddu, &mk2c::step_a4_pcm_chan_init_movlw_r4_br_0x3a_word);
    MK2CPP_HandRegister(0x000413dfu, &mk2c::step_a4_pcm_chan_init_cmp_r4_r1_byte);
    MK2CPP_HandRegister(0x000413e1u, &mk2c::step_a4_pcm_chan_init_bne_0x13d1);
    MK2CPP_HandRegister(0x000413e3u, &mk2c::step_a4_pcm_chan_init_movi_r1_0x3800);
    MK2CPP_HandRegister(0x000413e6u, &mk2c::step_a4_pcm_chan_init_movg2_r1_r2_word);
    MK2CPP_HandRegister(0x000413e8u, &mk2c::step_a4_pcm_chan_init_addq_2_r2_word);
    MK2CPP_HandRegister(0x000413eau, &mk2c::step_a4_pcm_chan_init_clr_r3_word);
    MK2CPP_HandRegister(0x000413ecu, &mk2c::step_a4_pcm_chan_init_movs_r3_br_0x09);
    MK2CPP_HandRegister(0x000413eeu, &mk2c::step_a4_pcm_chan_init_movsw_r1_br_0x0a);
    MK2CPP_HandRegister(0x000413f0u, &mk2c::step_a4_pcm_chan_init_movg3_r3_to_br_0x0d_byte);
    MK2CPP_HandRegister(0x000413f3u, &mk2c::step_a4_pcm_chan_init_movsw_r2_br_0x0e);
    MK2CPP_HandRegister(0x000413f5u, &mk2c::step_a4_pcm_chan_init_movg3_r3_to_br_0x10_word);
    MK2CPP_HandRegister(0x000413f8u, &mk2c::step_a4_pcm_chan_init_addq_1_r1_word);
    MK2CPP_HandRegister(0x000413fau, &mk2c::step_a4_pcm_chan_init_movs_r3_br_0x05);
    MK2CPP_HandRegister(0x000413fcu, &mk2c::step_a4_pcm_chan_init_movsw_r1_br_0x06);
    MK2CPP_HandRegister(0x000413feu, &mk2c::step_a4_pcm_chan_init_bsr_to_0x142f_b);
    MK2CPP_HandRegister(0x00041400u, &mk2c::step_a4_pcm_chan_init_bsr_to_0x142f_c);
    MK2CPP_HandRegister(0x00041402u, &mk2c::step_a4_pcm_chan_init_movs_r3_br_0x05_a);
    MK2CPP_HandRegister(0x00041404u, &mk2c::step_a4_pcm_chan_init_movsw_r1_br_0x06_a);
    MK2CPP_HandRegister(0x00041406u, &mk2c::step_a4_pcm_chan_init_movl_r2_br_0x05_byte);
    MK2CPP_HandRegister(0x00041408u, &mk2c::step_a4_pcm_chan_init_movl_r2_br_0x39_byte);
    MK2CPP_HandRegister(0x0004140au, &mk2c::step_a4_pcm_chan_init_movlw_r4_br_0x3a_word_a);
    MK2CPP_HandRegister(0x0004140cu, &mk2c::step_a4_pcm_chan_init_tst_r2_byte);
    MK2CPP_HandRegister(0x0004140eu, &mk2c::step_a4_pcm_chan_init_bne_0x13fa);
    MK2CPP_HandRegister(0x00041410u, &mk2c::step_a4_pcm_chan_init_cmp_r1_r4_word);
    MK2CPP_HandRegister(0x00041412u, &mk2c::step_a4_pcm_chan_init_bne_0x13fa_a);
    MK2CPP_HandRegister(0x00041414u, &mk2c::step_a4_pcm_chan_init_movsw_r3_br_0x30);
    MK2CPP_HandRegister(0x00041416u, &mk2c::step_a4_pcm_chan_init_bsr_to_0x142f_d);
    MK2CPP_HandRegister(0x00041418u, &mk2c::step_a4_pcm_chan_init_bsr_to_0x142f_e);
    MK2CPP_HandRegister(0x0004141au, &mk2c::step_a4_pcm_chan_init_movsw_r3_br_0x30_a);
    MK2CPP_HandRegister(0x0004141cu, &mk2c::step_a4_pcm_chan_init_movl_r2_br_0x30_byte);
    MK2CPP_HandRegister(0x0004141eu, &mk2c::step_a4_pcm_chan_init_movlw_r4_br_0x3a_word_b);
    MK2CPP_HandRegister(0x00041420u, &mk2c::step_a4_pcm_chan_init_tst_r4_byte);
    MK2CPP_HandRegister(0x00041422u, &mk2c::step_a4_pcm_chan_init_bne_0x1414);
    MK2CPP_HandRegister(0x00041424u, &mk2c::step_a4_pcm_chan_init_ldc_0x00_r5_b);
    MK2CPP_HandRegister(0x00041427u, &mk2c::step_a4_pcm_chan_init_ldc_0x00_r4_a);
    MK2CPP_HandRegister(0x0004142au, &mk2c::step_a4_pcm_chan_init_ldc_0xff_r3);
    MK2CPP_HandRegister(0x0004142du, &mk2c::step_a4_pcm_chan_init_ret);

    /* A5 maint_reset32 0x329f..0x33c2, 75 PCs */
    MK2CPP_HandRegister(0x0004329fu, &mk2c::step_a5_maint_reset32_ldc_0x00_r5);
    MK2CPP_HandRegister(0x000432a2u, &mk2c::step_a5_maint_reset32_ldc_0x00_r4);
    MK2CPP_HandRegister(0x000432a5u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1d6_byte);
    MK2CPP_HandRegister(0x000432abu, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1d7_byte);
    MK2CPP_HandRegister(0x000432b1u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1d8_byte);
    MK2CPP_HandRegister(0x000432b7u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1d9_byte);
    MK2CPP_HandRegister(0x000432bdu, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1da_byte);
    MK2CPP_HandRegister(0x000432c3u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1db_byte);
    MK2CPP_HandRegister(0x000432c9u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1dc_byte);
    MK2CPP_HandRegister(0x000432cfu, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1dd_byte);
    MK2CPP_HandRegister(0x000432d5u, &mk2c::step_a5_maint_reset32_jsr_0x33c4);
    MK2CPP_HandRegister(0x000432d8u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1ff_byte);
    MK2CPP_HandRegister(0x000432deu, &mk2c::step_a5_maint_reset32_clr_dp_0xd421_byte);
    MK2CPP_HandRegister(0x000432e2u, &mk2c::step_a5_maint_reset32_clr_dp_0xd200_byte);
    MK2CPP_HandRegister(0x000432e6u, &mk2c::step_a5_maint_reset32_clr_dp_0xd1cc_byte);
    MK2CPP_HandRegister(0x000432eau, &mk2c::step_a5_maint_reset32_clr_dp_0xd22d_byte);
    MK2CPP_HandRegister(0x000432eeu, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd22c_byte);
    MK2CPP_HandRegister(0x000432f4u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1cd_byte);
    MK2CPP_HandRegister(0x000432fau, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1ce_byte);
    MK2CPP_HandRegister(0x00043300u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd1cf_byte);
    MK2CPP_HandRegister(0x00043306u, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xd206_byte);
    MK2CPP_HandRegister(0x0004330cu, &mk2c::step_a5_maint_reset32_movg_0x00ff_to_dp_0xff8e_byte);
    MK2CPP_HandRegister(0x00043312u, &mk2c::step_a5_maint_reset32_movg_0x00fe_to_dp_0xff8c_byte);
    MK2CPP_HandRegister(0x00043318u, &mk2c::step_a5_maint_reset32_movi_r1_0xd435);
    MK2CPP_HandRegister(0x0004331bu, &mk2c::step_a5_maint_reset32_movi_r0_0x001f);
    MK2CPP_HandRegister(0x0004331eu, &mk2c::step_a5_maint_reset32_clr_r1_postinc_byte);
    MK2CPP_HandRegister(0x00043320u, &mk2c::step_a5_maint_reset32_cntjmp_r0_5_to_0x331e);
    MK2CPP_HandRegister(0x00043323u, &mk2c::step_a5_maint_reset32_movi_r1_0xd1ac);
    MK2CPP_HandRegister(0x00043326u, &mk2c::step_a5_maint_reset32_movi_r0_0x001f_a);
    MK2CPP_HandRegister(0x00043329u, &mk2c::step_a5_maint_reset32_movg_0x04_to_r1_postinc_byte);
    MK2CPP_HandRegister(0x0004332cu, &mk2c::step_a5_maint_reset32_cntjmp_r0_6_to_0x3329);
    MK2CPP_HandRegister(0x0004332fu, &mk2c::step_a5_maint_reset32_movg_0x04_to_dp_0xd311_byte);
    MK2CPP_HandRegister(0x00043334u, &mk2c::step_a5_maint_reset32_movg_0x04_to_dp_0xd6a4_byte);
    MK2CPP_HandRegister(0x00043339u, &mk2c::step_a5_maint_reset32_movg_0x1e_to_dp_0xd1d3_byte);
    MK2CPP_HandRegister(0x0004333eu, &mk2c::step_a5_maint_reset32_movg_0x03_to_dp_0xd1d4_byte);
    MK2CPP_HandRegister(0x00043343u, &mk2c::step_a5_maint_reset32_movg_0x04_to_dp_0xd1d5_byte);
    MK2CPP_HandRegister(0x00043348u, &mk2c::step_a5_maint_reset32_movg2_dp_0xfff3_r0_byte);
    MK2CPP_HandRegister(0x0004334cu, &mk2c::step_a5_maint_reset32_and_0xf8_r0_byte);
    MK2CPP_HandRegister(0x0004334fu, &mk2c::step_a5_maint_reset32_or_0x07_r0_byte);
    MK2CPP_HandRegister(0x00043352u, &mk2c::step_a5_maint_reset32_movg3_r0_to_dp_0xfff3_byte);
    MK2CPP_HandRegister(0x00043356u, &mk2c::step_a5_maint_reset32_movg2_dp_0xfff7_r0_byte);
    MK2CPP_HandRegister(0x0004335au, &mk2c::step_a5_maint_reset32_and_0xfe_r0_byte);
    MK2CPP_HandRegister(0x0004335du, &mk2c::step_a5_maint_reset32_movg3_r0_to_dp_0xfff7_byte);
    MK2CPP_HandRegister(0x00043361u, &mk2c::step_a5_maint_reset32_movg2_dp_0xd45c_r6_byte);
    MK2CPP_HandRegister(0x00043365u, &mk2c::step_a5_maint_reset32_or_0x02_r6_byte);
    MK2CPP_HandRegister(0x00043368u, &mk2c::step_a5_maint_reset32_movg3_r6_to_dp_0xd45c_byte);
    MK2CPP_HandRegister(0x0004336cu, &mk2c::step_a5_maint_reset32_movg3_r6_to_dp_0xe402_byte);
    MK2CPP_HandRegister(0x00043370u, &mk2c::step_a5_maint_reset32_movi_r0_0x000a);
    MK2CPP_HandRegister(0x00043373u, &mk2c::step_a5_maint_reset32_trapa_0x11);
    MK2CPP_HandRegister(0x00043375u, &mk2c::step_a5_maint_reset32_move_r0_0x80);
    MK2CPP_HandRegister(0x00043377u, &mk2c::step_a5_maint_reset32_trapa_0x10);
    MK2CPP_HandRegister(0x00043379u, &mk2c::step_a5_maint_reset32_movg2_dp_0xd1d3_r0_byte);
    MK2CPP_HandRegister(0x0004337du, &mk2c::step_a5_maint_reset32_addq_m1_r0_byte);
    MK2CPP_HandRegister(0x0004337fu, &mk2c::step_a5_maint_reset32_beq_0x3387);
    MK2CPP_HandRegister(0x00043381u, &mk2c::step_a5_maint_reset32_movg3_r0_to_dp_0xd1d3_byte);
    MK2CPP_HandRegister(0x00043385u, &mk2c::step_a5_maint_reset32_bra_0x33a4);
    MK2CPP_HandRegister(0x00043387u, &mk2c::step_a5_maint_reset32_movg_0x1e_to_dp_0xd1d3_byte_a);
    MK2CPP_HandRegister(0x0004338cu, &mk2c::step_a5_maint_reset32_bsr_to_0x33d1);
    MK2CPP_HandRegister(0x0004338eu, &mk2c::step_a5_maint_reset32_addq_m1_dp_0xd6a4_byte);
    MK2CPP_HandRegister(0x00043392u, &mk2c::step_a5_maint_reset32_bne_0x3399);
    MK2CPP_HandRegister(0x00043394u, &mk2c::step_a5_maint_reset32_movg_0x03_to_dp_0xd6a4_byte);
    MK2CPP_HandRegister(0x00043399u, &mk2c::step_a5_maint_reset32_addq_m1_dp_0xd311_byte);
    MK2CPP_HandRegister(0x0004339du, &mk2c::step_a5_maint_reset32_bne_0x33a4);
    MK2CPP_HandRegister(0x0004339fu, &mk2c::step_a5_maint_reset32_movg_0x03_to_dp_0xd311_byte);
    MK2CPP_HandRegister(0x000433a4u, &mk2c::step_a5_maint_reset32_movg2_dp_0xd1d4_r0_byte);
    MK2CPP_HandRegister(0x000433a8u, &mk2c::step_a5_maint_reset32_addq_m1_r0_byte_a);
    MK2CPP_HandRegister(0x000433aau, &mk2c::step_a5_maint_reset32_beq_0x33b2);
    MK2CPP_HandRegister(0x000433acu, &mk2c::step_a5_maint_reset32_movg3_r0_to_dp_0xd1d4_byte);
    MK2CPP_HandRegister(0x000433b0u, &mk2c::step_a5_maint_reset32_bra_0x33b9);
    MK2CPP_HandRegister(0x000433b2u, &mk2c::step_a5_maint_reset32_movg_0x03_to_dp_0xd1d4_byte_a);
    MK2CPP_HandRegister(0x000433b7u, &mk2c::step_a5_maint_reset32_bsr_to_0x33ea);
    MK2CPP_HandRegister(0x000433b9u, &mk2c::step_a5_maint_reset32_jsr_0x3641);
    MK2CPP_HandRegister(0x000433bcu, &mk2c::step_a5_maint_reset32_jsr_0x34cb);
    MK2CPP_HandRegister(0x000433bfu, &mk2c::step_a5_maint_reset32_jsr_0x342a);
    MK2CPP_HandRegister(0x000433c2u, &mk2c::step_a5_maint_reset32_bra_0x3370);

    /* 0x42B8A 0x2b8a..0x2b98, 5 PCs */
    MK2CPP_HandRegister(0x00042b8au, &mk2c::step_42b8a_clr_r0_m108_word);
    MK2CPP_HandRegister(0x00042b8du, &mk2c::step_42b8a_clr_r0_m74_word);
    MK2CPP_HandRegister(0x00042b90u, &mk2c::step_42b8a_clr_dp_0xce36_word);
    MK2CPP_HandRegister(0x00042b94u, &mk2c::step_42b8a_clr_dp_0xce38_word);
    MK2CPP_HandRegister(0x00042b98u, &mk2c::step_42b8a_rts);

    /* d1ac helper family 0x3695..0x36db, 25 PCs */
    MK2CPP_HandRegister(0x00043695u, &mk2c::step_d1ac_extu_r0);
    MK2CPP_HandRegister(0x00043697u, &mk2c::step_d1ac_movg2_r0_0xd1ac_r1_byte);
    MK2CPP_HandRegister(0x0004369bu, &mk2c::step_d1ac_or_0x04_r1_byte);
    MK2CPP_HandRegister(0x0004369eu, &mk2c::step_d1ac_movg3_r1_to_r0_0xd1ac_byte);
    MK2CPP_HandRegister(0x000436a2u, &mk2c::step_d1ac_rts);
    MK2CPP_HandRegister(0x000436a3u, &mk2c::step_d1ac_movi_r0_0x001f);
    MK2CPP_HandRegister(0x000436a6u, &mk2c::step_d1ac_movg2_r0_0xd1ac_r1_byte_a);
    MK2CPP_HandRegister(0x000436aau, &mk2c::step_d1ac_or_0x04_r1_byte_a);
    MK2CPP_HandRegister(0x000436adu, &mk2c::step_d1ac_movg3_r1_to_r0_0xd1ac_byte_a);
    MK2CPP_HandRegister(0x000436b1u, &mk2c::step_d1ac_cntjmp_r0_14_to_0x36a6);
    MK2CPP_HandRegister(0x000436b4u, &mk2c::step_d1ac_rts_a);
    MK2CPP_HandRegister(0x000436b5u, &mk2c::step_d1ac_extu_r0_a);
    MK2CPP_HandRegister(0x000436b7u, &mk2c::step_d1ac_movg2_r0_0xd1ac_r1_byte_b);
    MK2CPP_HandRegister(0x000436bbu, &mk2c::step_d1ac_and_0xfb_r1_byte);
    MK2CPP_HandRegister(0x000436beu, &mk2c::step_d1ac_movg3_r1_to_r0_0xd1ac_byte_b);
    MK2CPP_HandRegister(0x000436c2u, &mk2c::step_d1ac_rts_b);
    MK2CPP_HandRegister(0x000436c3u, &mk2c::step_d1ac_movi_r0_0x001f_a);
    MK2CPP_HandRegister(0x000436c6u, &mk2c::step_d1ac_movg2_r0_0xd1ac_r1_byte_c);
    MK2CPP_HandRegister(0x000436cau, &mk2c::step_d1ac_and_0xfb_r1_byte_a);
    MK2CPP_HandRegister(0x000436cdu, &mk2c::step_d1ac_movg3_r1_to_r0_0xd1ac_byte_c);
    MK2CPP_HandRegister(0x000436d1u, &mk2c::step_d1ac_cntjmp_r0_14_to_0x36c6);
    MK2CPP_HandRegister(0x000436d4u, &mk2c::step_d1ac_rts_c);
    MK2CPP_HandRegister(0x000436d5u, &mk2c::step_d1ac_extu_r0_b);
    MK2CPP_HandRegister(0x000436d7u, &mk2c::step_d1ac_bclr_r0_0xd1ac_1);
    MK2CPP_HandRegister(0x000436dbu, &mk2c::step_d1ac_rts_d);
#endif
}

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
namespace
{
struct ResetInitSelfRegister
{
    ResetInitSelfRegister() { mk2c::hand_register_module(&MK2CPP_ResetInitFillTables); }
};

ResetInitSelfRegister g_reset_init_self_register;

} /* anonymous namespace */
