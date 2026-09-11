/*
 * HAND voice/native_pool -- voice pool initialization, one host step per H8
 * instruction (M4, round 7).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 5
 *
 * Replaces the rom2 pool-init routine 0x40462-0x4062a (pjsr from 0x565,
 * ret from 0x40586) with per-PC entries. Every registered PC executes exactly
 * one H8 instruction and returns 1, so the host keeps its per-instruction poll,
 * TIMER_Clock, trace and SM_Update cadence. The previous form aggregated the
 * routine into four L1 blocks (A 0x40462..0x404b0 = 22 instructions,
 * B 0x404b4..0x4061e = 103 PCs, C 0x0621, D 0x40624/27/2a); a multi-
 * instruction block defers the host tail, so -midiseq / real-time MIDI bytes
 * queued inside the block are consumed by the SM at the block end instead of
 * at each instruction boundary. That timing drift is why A and B are now
 * per-PC (the alloc/free and materialize slices were converted for the same
 * reason); C and D already were.
 *
 *   range           PCs  notes
 *   A 0x40462..0x404b0  22  seed + loop A first half
 *   B 0x404b4..0x4061e 103  loop A tail, free/desc/part tables, helpers
 *   C 0x40621            1  MOVG #0xff -> r2++ (16x16 fill store)
 *   D 0x40624/27/2a      3  cntjmp r1 -6 / cntjmp r3 -48 / rts
 *
 * 0x40586 (ret, pops cp+pc) stays with the generated code, exactly like
 * before; 0x4062a's rts pops to it and the next dispatch runs it per-PC.
 *
 * The stock boot executes 2866 instructions (2868 traced PCs minus the two
 * interrupt-poll lines); per-PC stepping reproduces the same sequence, and the
 * cycle count comes from the host's own +12 per entry.
 *
 * Instruction semantics come from GT (src/mcu_opcodes.cpp); flag effects are
 * mirrored with GT's own helpers. The ROM's LDC #1/#2 r4 (0x4059c/0x405a6)
 * writes ep and sets ex_ignore like the stock LDC (src/mcu_opcodes.cpp:996),
 * so the following host poll is skipped exactly as stock skips it. All state
 * lives in page-0 SRAM and is accessed with MCU_Read/MCU_Write, so the state
 * hashdump observes exactly the stock bytes.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"

#include "native_pool.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as native_allocfree.cpp / pcm_enable.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* ---- GT instruction effects (src/mcu_opcodes.cpp) ------------------------ */

/* CLR flags: N=0, Z=1, V=0, C=0. */
void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* MOVG3 / MOVG_Immediate -> mem: N/Z/V from the value, C preserved. */
void store8(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
    MCU_SetStatusCommon(value, 0);
}

void store16(uint32_t addr, uint16_t value)
{
    MCU_Write16(addr, value);
    MCU_SetStatusCommon(value, 1);
}

void mov_imm8(uint32_t addr, uint8_t value)
{
    MCU_Write(addr, value);
    MCU_SetStatusCommon(value, 0);
}

void mov_imm16(uint32_t addr, uint16_t value)
{
    MCU_Write16(addr, value);
    MCU_SetStatusCommon(value, 1);
}

/* CLR mem (byte/word): GT also clears C. */
void clear8(uint32_t addr)
{
    MCU_Write(addr, 0);
    flags_clr();
}

void clear16(uint32_t addr)
{
    MCU_Write16(addr, 0);
    flags_clr();
}

/* MOVG2 mem -> rN; byte loads keep the register's high byte. */
void load8(uint16_t &reg, uint32_t addr)
{
    uint8_t value = MCU_Read(addr);
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

void load16(uint16_t &reg, uint32_t addr)
{
    uint16_t value = MCU_Read16(addr);
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* movi rN #imm16 (Short_MOVI) / move rN #imm8 (Short_MOVE). */
void movi(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

void move8(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

/* SHLL/SHLR word: C = shifted-out bit, N/Z/V from the result. */
void shll16(uint16_t &reg)
{
    uint16_t data = reg;
    uint16_t carry = (uint16_t)((data & 0x8000u) != 0);
    data = (uint16_t)(data << 1);
    reg = data;
    MCU_SetStatus(carry, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

void shlr16(uint16_t &reg)
{
    uint16_t data = reg;
    uint16_t carry = (uint16_t)(data & 1u);
    data = (uint16_t)(data >> 1);
    reg = data;
    MCU_SetStatus(carry, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

/* SHLL/SHLR on the low byte only (operand byte has bit3 clear). */
void shll8(uint16_t &reg)
{
    uint16_t data = (uint16_t)(reg & 0xffu);
    uint16_t carry = (uint16_t)((data & 0x80u) != 0);
    data = (uint16_t)((data << 1) & 0xffu);
    reg = (uint16_t)((reg & 0xff00u) | data);
    MCU_SetStatus(carry, STATUS_C);
    MCU_SetStatusCommon(data, 0);
}

void shlr8(uint16_t &reg)
{
    uint16_t data = (uint16_t)(reg & 0xffu);
    uint16_t carry = (uint16_t)(data & 1u);
    data = (uint16_t)(data >> 1);
    reg = (uint16_t)((reg & 0xff00u) | data);
    MCU_SetStatus(carry, STATUS_C);
    MCU_SetStatusCommon(data, 0);
}

/* ADDQ on a register's low byte: high byte preserved, flags from the byte. */
void addq8_low(uint16_t &reg, int delta)
{
    int32_t value = MCU_ADD_Common((int32_t)(reg & 0xffu), delta, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | (uint8_t)value);
}

/* ADDQ #1 / #-1 on a memory byte: GT computes the flags from the byte. */
void addq8(uint32_t addr, int delta)
{
    int32_t value = MCU_ADD_Common((int32_t)MCU_Read(addr), delta, 0, 0);
    MCU_Write(addr, (uint8_t)value);
}

/* cmp r4,w #imm: flags only, no write (word). */
void sub16_nowrite(uint16_t value, uint16_t imm)
{
    (void)MCU_SUB_Common((int32_t)value, (int32_t)imm, 0, 1);
}

/* MULXU #0xd8 rH:rL: H = high word, L = low word, N/Z from the 32-bit product. */
void mulxu_d8(uint16_t &high, uint16_t &low, uint16_t value)
{
    uint32_t product = 0xd8u * (uint32_t)value;
    high = (uint16_t)(product >> 16);
    low = (uint16_t)product;
    MCU_SetStatus((product & 0x80000000u) != 0, STATUS_N);
    MCU_SetStatus(product == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* BCLR @addr #0: Z = (bit == 0), then clear bit 0; other flags untouched. */
void bclr0(uint32_t addr)
{
    uint8_t data = MCU_Read(addr);
    MCU_SetStatus((data & 1u) == 0, STATUS_Z);
    MCU_Write(addr, (uint8_t)(data & 0xfeu));
}

/* BTSTI @addr #0: Z = (bit == 0); other flags untouched. */
void btsti0(uint32_t addr)
{
    MCU_SetStatus((MCU_Read(addr) & 1u) == 0, STATUS_Z);
}

/* Conditional branch targets (GT MCU_Jump_Bcc, src/mcu_opcodes.cpp:231). */
uint16_t bpl_pool(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_N) ? fall : taken; }
uint16_t bcc_pool(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_C) ? fall : taken; }
uint16_t beq_pool(uint16_t taken, uint16_t fall) { return (mcu.sr & STATUS_Z) ? taken : fall; }

/* Effective addresses: @rN+disp through dp (r0-r3), ep (r4/r5), tp (r6/r7);
 * (dp,addr16) absolute. Same pages the interpreter uses. */
uint32_t ind_addr(uint32_t reg, uint16_t disp)
{
    uint8_t page = (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
    return ((uint32_t)page << 16) | (uint16_t)(mcu.r[reg] + disp);
}

uint32_t dp_addr(uint16_t disp) { return ((uint32_t)mcu.dp << 16) | disp; }

/* Safety net: a PC missing from the tables below is executed by the stock
 * interpreter for that one instruction (should never happen). */
void stock_instruction(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
}

/* ---- A 0x40462..0x404b0 (22 instructions) -------------------------------- */

uint32_t step_pool_init_a(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];
    uint16_t &r3 = mcu.r[3];

    switch (mcu.pc)
    {
    case 0x0462: movi(r1, 0x001b);                          mcu.pc = 0x0465; break;
    case 0x0465: r2 = 0; MCU_SetStatusCommon(0, 1);         mcu.pc = 0x0467; break;
    case 0x0467: move8(r3, 0xff);                           mcu.pc = 0x0469; break;
    case 0x0469: store8(ind_addr(1, 0xce5c), (uint8_t)r3);  mcu.pc = 0x046d; break;
    case 0x046d: store8(ind_addr(1, 0xce78), (uint8_t)r3);  mcu.pc = 0x0471; break;
    case 0x0471: store8(ind_addr(1, 0xce94), (uint8_t)r3);  mcu.pc = 0x0475; break;
    case 0x0475: store8(ind_addr(1, 0xd118), (uint8_t)r3);  mcu.pc = 0x0479; break;
    case 0x0479: store8(ind_addr(1, 0xd134), (uint8_t)r3);  mcu.pc = 0x047d; break;
    case 0x047d: store8(ind_addr(1, 0xceb0), (uint8_t)r3);  mcu.pc = 0x0481; break;
    case 0x0481: store8(ind_addr(1, 0xcf20), (uint8_t)r3);  mcu.pc = 0x0485; break;
    case 0x0485: mov_imm8(ind_addr(1, 0xcf3c), 0x3c);       mcu.pc = 0x048a; break;
    case 0x048a: store8(ind_addr(1, 0xcf90), (uint8_t)r3);  mcu.pc = 0x048e; break;
    case 0x048e: store8(ind_addr(1, 0xd0fc), (uint8_t)r3);  mcu.pc = 0x0492; break;
    case 0x0492: store8(ind_addr(1, 0xd0a8), (uint8_t)r3);  mcu.pc = 0x0496; break;
    case 0x0496: store8(ind_addr(1, 0xd0c4), (uint8_t)r3);  mcu.pc = 0x049a; break;
    case 0x049a: store8(ind_addr(1, 0xad0e), (uint8_t)r2);  mcu.pc = 0x049e; break;
    case 0x049e: store8(ind_addr(1, 0xce3f), (uint8_t)r2);  mcu.pc = 0x04a2; break;
    case 0x04a2: store8(ind_addr(1, 0xd0e0), (uint8_t)r2);  mcu.pc = 0x04a6; break;
    case 0x04a6: store8(ind_addr(1, 0xa4b4), (uint8_t)r2);  mcu.pc = 0x04aa; break;
    case 0x04aa: store8(ind_addr(1, 0xa34c), (uint8_t)r2);  mcu.pc = 0x04ae; break;
    case 0x04ae: shll16(r1);                                mcu.pc = 0x04b0; break;
    case 0x04b0: store16(ind_addr(1, 0xcfac), r2);          mcu.pc = 0x04b4; break;
    default: stock_instruction(); break;
    }
    return 1;
}

/* ---- B 0x404b4..0x4061e (103 instructions) ------------------------------- */

uint32_t step_pool_init_b(void)
{
    uint16_t &r0 = mcu.r[0];
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];
    uint16_t &r3 = mcu.r[3];

    switch (mcu.pc)
    {
    /* loop A tail: slot 27, then slots 26..0 via 0x404c2 */
    case 0x04b4: store16(ind_addr(1, 0xd000), r2);          mcu.pc = 0x04b8; break;
    case 0x04b8: store16(ind_addr(1, 0xd054), r2);          mcu.pc = 0x04bc; break;
    case 0x04bc: store16(ind_addr(1, 0xa46c), r2);          mcu.pc = 0x04c0; break;
    case 0x04c0: shlr16(r1);                                mcu.pc = 0x04c2; break;
    case 0x04c2: r1 = (uint16_t)(r1 - 1);
                  mcu.pc = (r1 != 0xffff) ? 0x0469u : 0x04c5u; break;

    /* seeds 0x404c5..0x404d9 */
    case 0x04c5: movi(r3, 0x000f);                          mcu.pc = 0x04c8; break;
    case 0x04c8: movi(r2, 0x001b);                          mcu.pc = 0x04cb; break;
    case 0x04cb: r1 = r2; MCU_SetStatusCommon(r2, 1);       mcu.pc = 0x04cd; break;
    case 0x04cd: store16(dp_addr(0xa432), r3);              mcu.pc = 0x04d1; break;
    case 0x04d1: store16(dp_addr(0xa434), r2);              mcu.pc = 0x04d5; break;
    case 0x04d5: store16(dp_addr(0xa436), r1);              mcu.pc = 0x04d9; break;
    case 0x04d9: movi(r1, 0x001b);                          mcu.pc = 0x04dc; break;

    /* loop B: chain sentinels, 28 iterations */
    case 0x04dc: mov_imm8(ind_addr(1, 0xa3f4), 0xff);       mcu.pc = 0x04e1; break;
    case 0x04e1: mov_imm8(ind_addr(1, 0xa410), 0xff);       mcu.pc = 0x04e6; break;
    case 0x04e6: clear8(ind_addr(1, 0xa3bc));               mcu.pc = 0x04ea; break;
    case 0x04ea: mov_imm8(ind_addr(1, 0xa3d8), 0xff);       mcu.pc = 0x04ef; break;
    case 0x04ef: r1 = (uint16_t)(r1 - 1);
                  mcu.pc = (r1 != 0xffff) ? 0x04dcu : 0x04f2u; break;

    /* free-list seed 0x404f2..0x40508 */
    case 0x04f2: clear16(dp_addr(0xa1f0));                  mcu.pc = 0x04f6; break;
    case 0x04f6: clear8(dp_addr(0xa1df));                   mcu.pc = 0x04fa; break;
    case 0x04fa: mov_imm8(dp_addr(0xa42f), 0xff);           mcu.pc = 0x04ff; break;
    case 0x04ff: mov_imm8(dp_addr(0xa430), 0xff);           mcu.pc = 0x0504; break;
    case 0x0504: clear8(dp_addr(0xa42d));                   mcu.pc = 0x0508; break;
    case 0x0508: movi(r1, 0x001b);                          mcu.pc = 0x050b; break;

    /* loop C: free list a42f = 27..0, a42d = 28 */
    case 0x050b: r0 = 0; flags_clr();                       mcu.pc = 0x050d; break;
    case 0x050d: load8(r0, dp_addr(0xa430));                mcu.pc = 0x0511; break;
    case 0x0511: mcu.pc = bpl_pool(0x051d, 0x0513); break;
    case 0x0513: store8(dp_addr(0xa42f), (uint8_t)r1);      mcu.pc = 0x0517; break;
    case 0x0517: store8(ind_addr(1, 0xa3d8), (uint8_t)r0);  mcu.pc = 0x051b; break;
    case 0x051b: mcu.pc = 0x0526; break;
    case 0x051d: store8(ind_addr(0, 0xa3d8), (uint8_t)r1);  mcu.pc = 0x0521; break;
    case 0x0521: mov_imm8(ind_addr(1, 0xa3d8), 0xff);       mcu.pc = 0x0526; break;
    case 0x0526: store8(dp_addr(0xa430), (uint8_t)r1);      mcu.pc = 0x052a; break;
    case 0x052a: mov_imm8(ind_addr(1, 0xa3a0), 0x94);       mcu.pc = 0x052f; break;
    case 0x052f: addq8(dp_addr(0xa42d), 1);                 mcu.pc = 0x0533; break;
    case 0x0533: r1 = (uint16_t)(r1 - 1);
                  mcu.pc = (r1 != 0xffff) ? 0x050du : 0x0536u; break;

    /* loop D: descriptor pool, r2 = 27..0 */
    case 0x0536: mov_imm8(dp_addr(0xa42e), 0xff);           mcu.pc = 0x053b; break;
    case 0x053b: load8(r0, dp_addr(0xa42e));                mcu.pc = 0x053f; break;
    case 0x053f: store8(ind_addr(2, 0xa250), (uint8_t)r0);  mcu.pc = 0x0543; break;
    case 0x0543: store8(dp_addr(0xa42e), (uint8_t)r2);      mcu.pc = 0x0547; break;
    case 0x0547: mov_imm8(ind_addr(2, 0xa288), 0x94);       mcu.pc = 0x054c; break;
    case 0x054c: mov_imm8(ind_addr(2, 0xa26c), 0xff);       mcu.pc = 0x0551; break;
    case 0x0551: mov_imm8(ind_addr(2, 0xa314), 0xff);       mcu.pc = 0x0556; break;
    case 0x0556: mov_imm8(ind_addr(2, 0xa2c0), 0xff);       mcu.pc = 0x055b; break;
    case 0x055b: mov_imm8(ind_addr(2, 0xa2dc), 0xff);       mcu.pc = 0x0560; break;
    case 0x0560: r2 = (uint16_t)(r2 - 1);
                  mcu.pc = (r2 != 0xffff) ? 0x053bu : 0x0563u; break;

    /* loop E: part table, r3 = 15..0 */
    case 0x0563: r2 = 0; flags_clr();                       mcu.pc = 0x0565; break;
    case 0x0565: clear8(ind_addr(3, 0xa210));               mcu.pc = 0x0569; break;
    case 0x0569: mov_imm8(ind_addr(3, 0xa220), 0xff);       mcu.pc = 0x056e; break;
    case 0x056e: mov_imm8(ind_addr(3, 0xa230), 0xff);       mcu.pc = 0x0573; break;
    case 0x0573: bclr0(ind_addr(3, 0xa240));                mcu.pc = 0x0577; break;
    case 0x0577: mov_imm8(ind_addr(3, 0xa200), 0xff);       mcu.pc = 0x057d; break;
    case 0x057d: r0 = (uint16_t)((r0 & 0xff00u) | (uint8_t)r3);
                  MCU_SetStatusCommon((uint8_t)r3, 0);       mcu.pc = 0x057f; break;
    case 0x057f: addq8_low(r3, -1);                         mcu.pc = 0x0581; break;
    case 0x0581: mcu.pc = bpl_pool(0x0565, 0x0583); break;

    /* bsr16 -> 0x40588; return 0x40586 */
    case 0x0583: MCU_PushStack(0x0586);                     mcu.pc = 0x0588; break;

    /* helper loop 1 0x40588..0x405f1: part record init, 16 parts */
    case 0x0588: movi(r3, 0x000f);                          mcu.pc = 0x058b; break;
    case 0x058b: shll8(r3);                                 mcu.pc = 0x058d; break;
    case 0x058d: load16(mcu.r[4], ind_addr(3, 0xabde));     mcu.pc = 0x0591; break;
    case 0x0591: store16(ind_addr(3, 0xa1b0), mcu.r[4]);    mcu.pc = 0x0595; break;
    case 0x0595: shlr8(r3);                                 mcu.pc = 0x0597; break;
    case 0x0597: sub16_nowrite(mcu.r[4], 0x00e0);           mcu.pc = 0x059a; break;
    case 0x059a: mcu.pc = bcc_pool(0x05a6, 0x059c); break;
    case 0x059c: mcu.ep = 1; mcu.ex_ignore = 1;             mcu.pc = 0x059f; break;
    case 0x059f: store8(dp_addr(0xa1f4), 1);                mcu.pc = 0x05a4; break;
    case 0x05a4: mcu.pc = 0x05b2; break;
    case 0x05a6: mcu.ep = 2; mcu.ex_ignore = 1;             mcu.pc = 0x05a9; break;
    case 0x05a9: store8(dp_addr(0xa1f4), 2);                mcu.pc = 0x05ae; break;
    case 0x05ae: mcu.r[4] = (uint16_t)MCU_SUB_Common(mcu.r[4], 0x00e0, 0, 1);
                                                             mcu.pc = 0x05b2; break;
    case 0x05b2: load8(r0, dp_addr(0xa1f4));                mcu.pc = 0x05b6; break;
    case 0x05b6: store8(ind_addr(3, 0xa43c), (uint8_t)r0);  mcu.pc = 0x05ba; break;
    case 0x05ba: r0 = mcu.r[4]; MCU_SetStatusCommon(mcu.r[4], 1);
                                                             mcu.pc = 0x05bc; break;
    case 0x05bc: mulxu_d8(r0, r1, r0);                      mcu.pc = 0x05c0; break;
    case 0x05c0: r1 = (uint16_t)MCU_ADD_Common(r1, 0, 0, 1); mcu.pc = 0x05c4; break;
    case 0x05c4: shll8(r3);                                 mcu.pc = 0x05c6; break;
    case 0x05c6: store16(ind_addr(3, 0xa44c), r1);          mcu.pc = 0x05ca; break;
    case 0x05ca: shlr8(r3);                                 mcu.pc = 0x05cc; break;
    case 0x05cc: mulxu_d8(mcu.r[4], mcu.r[5], mcu.r[4]);    mcu.pc = 0x05d0; break;
    case 0x05d0: mcu.r[5] = (uint16_t)MCU_ADD_Common(mcu.r[5], 0, 0, 1);
                                                             mcu.pc = 0x05d4; break;
    case 0x05d4: move8(r0, 0);                              mcu.pc = 0x05d6; break;
    case 0x05d6: btsti0(((uint32_t)mcu.ep << 16) | (uint16_t)(mcu.r[5] + 13));
                                                             mcu.pc = 0x05d9; break;
    case 0x05d9: mcu.pc = beq_pool(0x05de, 0x05dc); break;
    case 0x05dc: move8(r0, 2);                              mcu.pc = 0x05de; break;
    case 0x05de: store8(ind_addr(3, 0xa040), (uint8_t)r0);  mcu.pc = 0x05e2; break;
    case 0x05e2: mov_imm8(ind_addr(3, 0xa080), 0x40);       mcu.pc = 0x05e7; break;
    case 0x05e7: shll16(r3);                                mcu.pc = 0x05e9; break;
    case 0x05e9: mov_imm16(ind_addr(3, 0xa190), 0x3c3c);    mcu.pc = 0x05ef; break;
    case 0x05ef: shlr16(r3);                                mcu.pc = 0x05f1; break;
    case 0x05f1: r3 = (uint16_t)(r3 - 1);
                  mcu.pc = (r3 != 0xffff) ? 0x058bu : 0x05f4u; break;

    /* helper loop 2 seeds + per-part body 0x405fa..0x4061e */
    case 0x05f4: movi(r3, 0x000f);                          mcu.pc = 0x05f7; break;
    case 0x05f7: movi(r2, 0xa090);                          mcu.pc = 0x05fa; break;
    case 0x05fa: clear8(ind_addr(3, 0xa060));               mcu.pc = 0x05fe; break;
    case 0x05fe: mov_imm8(ind_addr(3, 0xa050), 0xff);       mcu.pc = 0x0603; break;
    case 0x0603: mov_imm8(ind_addr(3, 0xa070), 0x3c);       mcu.pc = 0x0608; break;
    case 0x0608: r0 = r3; MCU_SetStatusCommon(r3, 1);       mcu.pc = 0x060a; break;
    case 0x060a: shll16(r0);                                mcu.pc = 0x060c; break;
    case 0x060c: shll16(r0);                                mcu.pc = 0x060e; break;
    case 0x060e: shll16(r0);                                mcu.pc = 0x0610; break;
    case 0x0610: shll16(r0);                                mcu.pc = 0x0612; break;
    case 0x0612: r0 = (uint16_t)MCU_ADD_Common(r0, 0x9f50, 0, 1); mcu.pc = 0x0616; break;
    case 0x0616: movi(r1, 7);                               mcu.pc = 0x0619; break;
    case 0x0619: r0 = (uint16_t)(r0 - 2); clear16(dp_addr(mcu.r[0]));
                                                             mcu.pc = 0x061b; break;
    case 0x061b: r1 = (uint16_t)(r1 - 1);
                  mcu.pc = (r1 != 0xffff) ? 0x0619u : 0x061eu; break;
    case 0x061e: movi(r1, 0x000f);                          mcu.pc = 0x0621; break;
    default: stock_instruction(); break;
    }
    return 1;
}

/* ---- C 0x40621: MOVG #0xff -> r2++ (fill store) -------------------------- */

uint32_t step_pool_init_c(void)
{
    MCU_Write(((uint32_t)mcu.dp << 16) | mcu.r[2], 0xff);
    MCU_SetStatusCommon(0xff, 0);
    mcu.r[2] = (uint16_t)(mcu.r[2] + 1);
    mcu.pc = 0x0624u;
    return 1;
}

/* ---- D 0x40624/0x40627/0x4062a: counter tail of the shared helper --------- */

uint32_t step_pool_init_d(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r3 = mcu.r[3];

    switch (mcu.pc)
    {
    case 0x0624: r1 = (uint16_t)(r1 - 1);
                 mcu.pc = (r1 != 0xffff) ? 0x0621u : 0x0627u; break;
    case 0x0627: r3 = (uint16_t)(r3 - 1);
                 mcu.pc = (r3 != 0xffff) ? 0x05fau : 0x062au; break;
    case 0x062a: mcu.pc = MCU_PopStack(); break;  /* rts (0x0586 ret via gen) */
    default: break;
    }
    return 1;
}

/* ---- PC tables (rom2 offsets; the flat key adds cp 4) --------------------- */

const uint16_t kPoolInitAPcs[] = {
    0x0462, 0x0465, 0x0467, 0x0469, 0x046d, 0x0471, 0x0475, 0x0479,
    0x047d, 0x0481, 0x0485, 0x048a, 0x048e, 0x0492, 0x0496, 0x049a,
    0x049e, 0x04a2, 0x04a6, 0x04aa, 0x04ae, 0x04b0,
};

const uint16_t kPoolInitBPcs[] = {
    0x04b4, 0x04b8, 0x04bc, 0x04c0, 0x04c2, 0x04c5, 0x04c8, 0x04cb,
    0x04cd, 0x04d1, 0x04d5, 0x04d9, 0x04dc, 0x04e1, 0x04e6, 0x04ea,
    0x04ef, 0x04f2, 0x04f6, 0x04fa, 0x04ff, 0x0504, 0x0508, 0x050b,
    0x050d, 0x0511, 0x0513, 0x0517, 0x051b, 0x051d, 0x0521, 0x0526,
    0x052a, 0x052f, 0x0533, 0x0536, 0x053b, 0x053f, 0x0543, 0x0547,
    0x054c, 0x0551, 0x0556, 0x055b, 0x0560, 0x0563, 0x0565, 0x0569,
    0x056e, 0x0573, 0x0577, 0x057d, 0x057f, 0x0581, 0x0583, 0x0588,
    0x058b, 0x058d, 0x0591, 0x0595, 0x0597, 0x059a, 0x059c, 0x059f,
    0x05a4, 0x05a6, 0x05a9, 0x05ae, 0x05b2, 0x05b6, 0x05ba, 0x05bc,
    0x05c0, 0x05c4, 0x05c6, 0x05ca, 0x05cc, 0x05d0, 0x05d4, 0x05d6,
    0x05d9, 0x05dc, 0x05de, 0x05e2, 0x05e7, 0x05e9, 0x05ef, 0x05f1,
    0x05f4, 0x05f7, 0x05fa, 0x05fe, 0x0603, 0x0608, 0x060a, 0x060c,
    0x060e, 0x0610, 0x0612, 0x0616, 0x0619, 0x061b, 0x061e,
};

const uint16_t kPoolInitDPcs[] = {
    0x0624, 0x0627, 0x062a,
};

} /* anonymous namespace */

void pool_post_reset(void)
{
    /* Pool-init is translated from ROM and writes page-0 SRAM directly; there
     * is no native pool copy left to reset. Kept as the HandPostReset hook. */
}

} /* namespace mk2c */

/* ---- hand integration hooks ----------------------------------------------- */

static void register_pool_pcs(const uint16_t *pcs, uint32_t count,
                              mk2cpp_hand_routine_fn fn)
{
    for (uint32_t i = 0; i < count; i++)
        MK2CPP_HandRegisterRoutine(0x00040000u | pcs[i], fn);
}

void MK2CPP_PoolPostReset(void)
{
    mk2c::pool_post_reset();
}

void MK2CPP_PoolFillTables(void)
{
    register_pool_pcs(mk2c::kPoolInitAPcs,
                      (uint32_t)(sizeof(mk2c::kPoolInitAPcs) / sizeof(mk2c::kPoolInitAPcs[0])),
                      &mk2c::step_pool_init_a);
    register_pool_pcs(mk2c::kPoolInitBPcs,
                      (uint32_t)(sizeof(mk2c::kPoolInitBPcs) / sizeof(mk2c::kPoolInitBPcs[0])),
                      &mk2c::step_pool_init_b);
    MK2CPP_HandRegisterRoutine(0x00040621u, &mk2c::step_pool_init_c);
    register_pool_pcs(mk2c::kPoolInitDPcs,
                      (uint32_t)(sizeof(mk2c::kPoolInitDPcs) / sizeof(mk2c::kPoolInitDPcs[0])),
                      &mk2c::step_pool_init_d);

    /* Whole-routine alloc/free hooks (native_allocfree.cpp). */
    MK2CPP_AllocFreeFillTables();
}
