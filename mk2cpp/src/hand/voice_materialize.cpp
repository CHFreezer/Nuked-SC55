/*
 * HAND voice/voice_materialize -- PCM voice materialize / gate / param chain
 * (M4 S1..S10): materialize, coeff_copy, tone_fields, cross_copy, gate_setup,
 * coeff_calc, mask_set + shared tail, flush_off/flush_on, param_write,
 * loop_write, param_ack, pcm_play.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 7
 * M4 closure step 2 (semantic rewrite): one named `void step_<routine>_<sem>` per PC,
 * registered flat=pc (cp0) via MK2CPP_HandRegister (all 921 PCs).
 *
 * Replaces these rom1 routines:
 *
 *   0x53eb materialize   0x53eb..0x546c (+ 0x546d rts trampoline): per-PC
 *                        (round 6; see the S1 comment for the L1 retirement)
 *   0x5d6e coeff_copy    0x5d6e..0x5dc2: per-PC (round 6)
 *   0x3580 tone_fields   0x3580..0x3614: one entry per instruction (L0-style)
 *   0x3a9e cross_copy    0x3a9e..0x3bb1: one entry per instruction (0x3a9e
 *                        and 0x3ac8 entries both covered)
 *   0x3615 gate_setup    0x3615..0x3708: one entry per instruction; 0x36fa
 *                        (bsr 0x3709) is translated as the push+jump, the
 *                        callee keeps running interpreted (interp_entry,
 *                        out-of-scope deep water)
 *   0x546e mask_set      + shared pending-command tail 0x5492..0x54cb: per-PC
 *                        (IML=0, interrupted inside in the captured runs)
 *   0x54fb flush_off     0x54fb..0x5521: per-PC (round 6), then the six
 *                        existing per-PC PCM MOVS/MOVL handlers
 *   0x564a flush_on      0x564a..0x565e / 0x5668..0x5670: per-PC (round 6)
 *   0x5533 param_write   0x5533..0x5625: per-PC (round 6), both exits
 *   0x5626 loop_write    per-PC: its PCM status busy-wait needs one host step
 *                        per iteration (device updates), see comment below
 *   0x54cc param_ack    0x54cc..0x54fa: per-PC (IML=0, trapa #0x10 loop)
 *   0x52cb pcm_play     0x52cb..0x53e8 (both entries): per-PC; bsr targets
 *                        are the registered child routines, BRA 0x51fc exits
 *                        do not pop the stack
 *   0x5998 coeff_calc   0x5998..0x5d6d: per-PC (IML=0, dense resume PCs); the
 *                        fixed-point maths is transcribed as named operations
 *                        (mulxu8/mulxu16_imm, table sum, clamp, shift scaling),
 *                        no algebraic rewriting
 *
 * S0 data model: the firmware keeps the authoritative bytes in page-0 SRAM, so
 * the translation reads/writes the same addresses as the stock instructions do
 * (MCU_Read/MCU_Write) instead of keeping a native copy. `ind_addr` below is
 * the GT general-operand effective address `@rN+disp` (r0-r3 address through
 * dp, r4/r5 through ep, r6/r7 through tp; src/mcu_opcodes.cpp:646,
 * src/mcu.h:208), so a caller with a dirty page register behaves exactly like
 * the interpreter. That is the whole data model this slice needs: named SoA
 * bases + AoS offsets (below), all byte-exact against the ROM.
 *
 * Ownership / interrupt boundary evidence:
 *  - materialize and coeff_copy were L1 blocks until round 6. Interrupt
 *    deferral was ruled out (materialize runs at IML=7 between 0x5304/0x539c
 *    BCLR_ANDC; coeff_copy is a plain leaf), but an L1 step also batches the
 *    host's midiseq_poll/SM_Update: a byte posted while the block executes
 *    lands in the SM ring before the SM catch-up replays that span, so the SM
 *    consumes it earlier in its own time than stock. Caught at c212.565M (the
 *    [565264,565660) materialize step spanning the 565572 byte) where the SM
 *    state diverged; both routines are per-PC like S3-S10 now. The same rule
 *    applies to every remaining L1 block: the span must not cover a due
 *    -midiseq post.
 *  - tone_fields / gate_setup / cross_copy all run at IML=0 and the stock run
 *    is interrupted at many different instruction boundaries inside them
 *    (the captured runs alone resume at 12 tone_fields and 8 gate_setup PCs;
 *    the frozen union trace lists a resume PC at almost every instruction).
 *    A multi-instruction L1 block would defer an interrupt to the block end
 *    and change the handler's saved resume PC / extended-RAM residue, so
 *    these routines are stepped one instruction at a time: each instruction
 *    PC is one routine entry returning 1 (host adds 12*(1-1) plus its own
 *    +12). The host then polls, runs TIMER_Clock and traces per instruction
 *    exactly like stock. ex_ignore (LDC 0x3580/0x3615, ORC 0x36e8, ANDC
 *    0x36fc) is applied per instruction for the same reason.
 *  - cross_copy has two entries (0x3a9e from pcm_play, 0x3ac8 from ts_scan)
 *    and two exits (0x3b09 / 0x3ba1); neither is entered in the captured
 *    demo/mocknote windows (verified: 0 hits of 0x3a9e/0x3ac8 in
 *    [144,175) / [200,296) / [296,340)) but its union trace does have
 *    resume edges, so it uses the same per-instruction scheme.
 *
 * Materialize path counts (per-PC entries now; kept as the decode record; the
 * host adds one +12 per entry):
 *   common 53eb..542f            = 20
 *   + 5431 (cf90 bit7 set)       = 21
 *   5436..544f                   = +9
 *   d0fc == 0 : 545a,545d,546d + 545f,5461,5465,5467,5469,546c = +9
 *   d0fc <  0 : 5451 + 5469,546c                              = +3
 *   d0fc >  0 : 5453,5456,546d,5458 + ...                     = +10  (E3 arm,
 *               decoded from build/rom1.bin 0x5453 `5b 8b d4 0e 15 20 05`)
 *
 * coeff_copy counts: 11 word load/store pairs + rts = 23.
 *
 * The 0x546d trampoline is a real (empty) bsr target; its push/pop stack
 * writes are replayed with MCU_PushStack/PopStack so the stack-page SRAM
 * bytes stay byte-identical for the state hash.
 */

#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_interrupt.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as native_allocfree.cpp / pcm_enable.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* ---- S0: materialize / coeff_copy field map (page-0 SRAM) --------------- */

enum {
    /* SoA sources; materialize copies them into the AoS record P(slot). */
    kSoaCfe4 = 0xcfe4,      /* byte -> P+0x98 */
    kSoaD038 = 0xd038,      /* byte -> P+0x99 */
    kSoaD08c = 0xd08c,      /* byte -> P+0x9a */
    kSoaCfac = 0xcfac,      /* word -> P+0x9c */
    kSoaD000 = 0xd000,      /* word -> P+0x9e */
    kSoaD054 = 0xd054,      /* word -> P+0xa0 */
    kSoaCf90 = 0xcf90,      /* byte -> P-0x3b; bit7 => ad0e[slot]=0xff */
    kSoaCe78 = 0xce78,      /* byte -> P+0x9b, indexes the tone table */
    kSoaCecc = 0xcecc,      /* byte -> P+0x30 addend */
    kSoaD0fc = 0xd0fc,      /* byte selector for P+0x30 */
    kAd0e = 0xad0e,         /* per-slot PCM channel, 0xff = not assigned */
    kAosPtrTable = 0x64d6,  /* ROM word table: AoS record pointer per slot */
    kTonePtrTable = 0x7218, /* ROM word table: tone record pointer per index */

    /* AoS offsets relative to P (materialize). */
    kAosSlot = -2,          /* word: slot */
    kAosE1 = 0x98,          /* bytes 0x98..0x9a */
    kAosE2 = 0x99,
    kAosE3 = 0x9a,
    kAosP1 = 0x9c,          /* words 0x9c..0xa0 */
    kAosP2 = 0x9e,
    kAosP3 = 0xa0,
    kAosFlag45 = -59,       /* byte P-0x3b (cf90 copy, "S+0x45") */
    kAosToneIdx = 0x9b,     /* byte: tone index */
    kAosToneRec = 0x2e,     /* word: tone pointer */
    kAosToneVal = 0x30,     /* word: 0 / 0x8748+cecc / 0x8bd4+cecc */
    kAosBaseZero = 0x8748,  /* d0fc == 0 base */
    kAosBasePos = 0x8bd4,   /* d0fc > 0 base (E3 arm) */

    /* AoS word fields copied by coeff_copy (dst r0 <- src r2). */
    kCoef86 = 0x86, kCoef8a = 0x8a, kCoef88 = 0x88,
    kCoefP80 = -80, kCoefP114 = -114,
    kCoef96 = 0x96, kCoef8e = 0x8e, kCoef94 = 0x94,
    kCoef8c = 0x8c, kCoef92 = 0x92, kCoef90 = 0x90,
};

/* GT general-operand effective address @rN+disp (src/mcu_opcodes.cpp:631-647):
 * disp is 8/16-bit and the sum wraps mod 0x10000; the page comes from the
 * register file. reg is the base register index 0..7. */
uint32_t ind_addr(uint32_t reg, uint16_t disp)
{
    uint8_t page = (reg >= 6) ? mcu.tp : (reg >= 4) ? mcu.ep : mcu.dp;
    return ((uint32_t)page << 16) | (uint16_t)(mcu.r[reg] + disp);
}

/* MOVG2 @addr -> rN (GT MCU_Opcode_MOVG read path): byte form keeps the
 * register's high byte, word form replaces it; status = common(value, size). */
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

/* MOVG3 rN -> @addr (write path): store then status = common(value, size). */
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

/* CLR rN (word): N=0, Z=1, V=0, C=0. */
void clr16(uint16_t &reg)
{
    reg = 0;
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* movi rN #imm16 (Short_MOVI): status = common(imm, word). */
void movi16(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* CLR flags shared by CLR rN / CLR @addr / CLR byte variants. */
void flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* CLR byte/word memory (GT writes the operand then clears the flags). */
void clr8_mem(uint32_t addr) { MCU_Write(addr, 0); flags_clr(); }
void clr16_mem(uint32_t addr) { MCU_Write16(addr, 0); flags_clr(); }

/* CLR rN byte form: high byte preserved, flags cleared. */
void clr8_reg(uint16_t &reg) { reg = (uint16_t)(reg & 0xff00u); flags_clr(); }

/* MOVG2 rS -> rD byte form (direct register operand read). */
void move8(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

void move16(uint16_t &reg, uint16_t value)
{
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* SWAP rN (byte-form operand, GT swaps the full 16-bit register). */
void swap16(uint16_t &reg)
{
    uint16_t data = reg;
    uint16_t swapped = (uint16_t)((data << 8) | (data >> 8));
    reg = swapped;
    MCU_SetStatusCommon(swapped, 1);
}

/* NEG rN: byte form keeps the high byte, word form replaces the register. */
void neg8(uint16_t &reg)
{
    int32_t value = MCU_SUB_Common(0, (uint8_t)reg, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | (uint8_t)value);
}

void neg16(uint16_t &reg)
{
    reg = (uint16_t)MCU_SUB_Common(0, reg, 0, 1);
}

/* AND rN,#imm (immediate operand): low byte / full word, status common(). */
void and8_imm(uint16_t &reg, uint8_t imm)
{
    reg = (uint16_t)((reg & 0xff00u) | ((uint8_t)reg & imm));
    MCU_SetStatusCommon(reg, 0);
}

void and16_imm(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)(reg & imm);
    MCU_SetStatusCommon(reg, 1);
}

/* ADD/SUB rD,rS (register-direct operand): byte form keeps the high byte. */
void add8(uint16_t &dst, uint16_t src)
{
    int32_t value = MCU_ADD_Common((uint8_t)dst, (uint8_t)src, 0, 0);
    dst = (uint16_t)((dst & 0xff00u) | (uint8_t)value);
}

void add16(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)MCU_ADD_Common(dst, src, 0, 1);
}

void sub8(uint16_t &dst, uint16_t src)
{
    int32_t value = MCU_SUB_Common((uint8_t)dst, (uint8_t)src, 0, 0);
    dst = (uint16_t)((dst & 0xff00u) | (uint8_t)value);
}

void sub8_imm(uint16_t &reg, uint8_t imm)
{
    int32_t value = MCU_SUB_Common((uint8_t)reg, imm, 0, 0);
    reg = (uint16_t)((reg & 0xff00u) | (uint8_t)value);
}

/* TST rN (byte): N/Z from the value, C cleared, V=0. */
void tst8(uint16_t reg)
{
    MCU_SetStatusCommon((uint8_t)reg, 0);
    MCU_SetStatus(0, STATUS_C);
}

/* CMP rD,src: GT compares t1 = rD against t2 = operand. */
void cmp8(uint16_t a, uint16_t b) { MCU_SUB_Common((uint8_t)a, (uint8_t)b, 0, 0); }
void cmp16(uint16_t a, uint16_t b) { MCU_SUB_Common(a, b, 0, 1); }

/* STC r4 (byte): r4 low byte = ep (control register 4); no flag change. */
void stc_ep_to_r4(void)
{
    mcu.r[4] = (uint16_t)((mcu.r[4] & 0xff00u) | (mcu.ep & 0xffu));
}

/* MULXU rR, rN: rN:rN+1 = rR * rN (GT MCU_Opcode_MULXU word path). The
 * multiplier is the operand (r6 in cross_copy), the register pair holds the
 * multiplicand on entry and the 32-bit product on exit. */
void mulxu16(uint16_t multiplier, uint16_t &high, uint16_t &low)
{
    uint32_t product = (uint32_t)multiplier * (uint32_t)high;
    high = (uint16_t)(product >> 16);
    low = (uint16_t)product;
    MCU_SetStatus((product & 0x80000000u) != 0, STATUS_N);
    MCU_SetStatus(product == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* Absolute operand pages: @(br,disp8) and @(dp,addr16) (GT addrpage = 0 / dp). */
uint32_t br_addr(uint16_t disp) { return (uint32_t)(uint16_t)((mcu.br << 8) | disp); }
uint32_t dp_addr(uint16_t disp) { return ((uint32_t)mcu.dp << 16) | disp; }

/* ---- S7/S8 fixed helpers ------------------------------------------------- */

void not16(uint16_t &reg)
{
    reg = (uint16_t)~reg;
    MCU_SetStatusCommon(reg, 1);
}

void or16_mem(uint16_t &reg, uint32_t addr)
{
    reg = (uint16_t)(reg | MCU_Read16(addr));
    MCU_SetStatusCommon(reg, 1);
}

void and16(uint16_t &dst, uint16_t src)
{
    dst = (uint16_t)(dst & src);
    MCU_SetStatusCommon(dst, 1);
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

void sub_imm16(uint16_t &reg, uint16_t imm)
{
    reg = (uint16_t)MCU_SUB_Common(reg, imm, 0, 1);
}

void sub_mem_imm8(uint32_t addr, uint8_t imm)
{
    int32_t value = MCU_Read(addr);
    MCU_SUB_Common(value, imm, 0, 0);
}

void sub_mem_imm16(uint32_t addr, uint16_t imm)
{
    int32_t value = MCU_Read16(addr);
    MCU_SUB_Common(value, imm, 0, 1);
}

void addq_r7(int delta)
{
    mcu.r[7] = (uint16_t)MCU_ADD_Common(mcu.r[7], delta, 0, 1);
}

void bset16_mem(uint32_t addr, uint16_t bit_source)
{
    uint16_t data = MCU_Read16(addr);
    uint16_t bit = (uint16_t)(bit_source & 0x0fu);
    MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);
    MCU_Write16(addr, (uint16_t)(data | (1u << bit)));
}

void tst8_mem(uint32_t addr)
{
    MCU_SetStatusCommon(MCU_Read(addr), 0);
    MCU_SetStatus(0, STATUS_C);
}

void tst16_mem(uint32_t addr)
{
    MCU_SetStatusCommon(MCU_Read16(addr), 1);
    MCU_SetStatus(0, STATUS_C);
}

void btsti8_mem(uint32_t addr, uint8_t bit)
{
    MCU_SetStatus((MCU_Read(addr) & (1u << bit)) == 0, STATUS_Z);
}

void btsti16_reg(uint16_t reg, uint8_t bit)
{
    MCU_SetStatus((reg & (1u << bit)) == 0, STATUS_Z);
}

/* Short_MOVS / Short_MOVL @(br,disp8): keep GT device routing. */
void movs8(uint16_t reg, uint16_t disp)
{
    uint8_t value = (uint8_t)reg;
    MCU_Write(br_addr(disp), value);
    MCU_SetStatusCommon(value, 0);
}

void movs16(uint16_t reg, uint16_t disp)
{
    MCU_Write16(br_addr(disp), reg);
    MCU_SetStatusCommon(reg, 1);
}

void movl8(uint16_t &reg, uint16_t disp)
{
    uint8_t value = MCU_Read(br_addr(disp));
    reg = (uint16_t)((reg & 0xff00u) | value);
    MCU_SetStatusCommon(value, 0);
}

void movl16(uint16_t &reg, uint16_t disp)
{
    uint16_t value = MCU_Read16(br_addr(disp));
    reg = value;
    MCU_SetStatusCommon(value, 1);
}

/* MOVG #imm write forms used by these routines. */
void movg_imm8(uint32_t addr, uint8_t imm)
{
    MCU_Write(addr, imm);
    MCU_SetStatusCommon(imm, 0);
}

void movg_imm16(uint32_t addr, uint16_t imm)
{
    MCU_Write16(addr, imm);
    MCU_SetStatusCommon(imm, 1);
}

/* LDC #imm r5: control register 5 (dp). Stock also suppresses the very next
 * interrupt poll; the S8 blocks have no internal poll and run in the ROM's
 * IML=7 window, so ex_ignore is not set here (same rationale as gate_setup). */
void ldc_dp(uint8_t value)
{
    mcu.dp = value;
}


void tst16_reg(uint16_t reg)
{
    MCU_SetStatusCommon(reg, 1);
    MCU_SetStatus(0, STATUS_C);
}

void mulxu8_reg(uint16_t mult, uint16_t &acc)
{
    uint32_t product = (uint32_t)(mult & 0xffu) * (uint32_t)(acc & 0xffu);
    acc = (uint16_t)product;
    MCU_SetStatus((product & 0x8000u) != 0, STATUS_N);
    MCU_SetStatus(product == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void mulxu8_mem(uint32_t addr, uint16_t &acc)
{
    mulxu8_reg(MCU_Read(addr), acc);
}

void mulxu16_imm(uint16_t imm, uint16_t &hi, uint16_t &lo)
{
    uint32_t product = (uint32_t)imm * (uint32_t)hi;
    hi = (uint16_t)(product >> 16);
    lo = (uint16_t)product;
    MCU_SetStatus((product & 0x80000000u) != 0, STATUS_N);
    MCU_SetStatus(product == 0, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void add16_mem(uint16_t &reg, uint32_t addr)
{
    reg = (uint16_t)MCU_ADD_Common(reg, MCU_Read16(addr), 0, 1);
}



/* ---- S1: materialize 0x53eb-0x546c (per instruction) -------------------- */

/*
 * materialize runs one H8 instruction per MK2CPP_Step (each entry returns 1).
 * The round-6 L1 block was retired: `-midiseq` posts scheduled bytes only at
 * main-loop boundaries, so a multi-instruction step that spans a due post
 * defers it to the block end; the SM_Update that follows then catches the SM
 * up over the whole span with the byte already in the ring, i.e. the SM
 * consumes it before the main cycle it was posted at. Captured at c212.565M:
 * the [212565264,212565660) materialize step spanned the byte due at
 * 212565572; the SM delivered it 346 main cycles early and its RAM/register
 * state diverged (f138 -> f8d2 at sm 1062826368 instead of the stock f151).
 * Per-instruction stepping keeps midiseq_poll / SM_Update / PCM_Update
 * cadence identical to stock, like S3-S10. 0x546d is a real (empty) bsr
 * target; its push/pop stack writes are replayed for byte-exact SRAM.
 */


/* 0x53eb MOVG2 r1 r3 */
void step_materialize_movg2_r1_r3(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r3 = mcu.r[3];

    r3 = r1;
    mcu.pc = 0x53ed;
}

/* 0x53ed ADD r3 r3 */
void step_materialize_add_r3_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    add16(r3, r3);
    mcu.pc = 0x53ef;
}

/* 0x53ef MOVG2 @r3+0x64d6 r0 */
void step_materialize_movg2_at_r3_plus_0x64d6_r0(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, ind_addr(3, kAosPtrTable));
    mcu.pc = 0x53f3;
}

/* 0x53f3 MOVG3 r1 -> @r0+-2 */
void step_materialize_movg3_r1_to_at_r0_minus_2(void)
{
    uint16_t &r1 = mcu.r[1];

    store16(ind_addr(0, (uint16_t)kAosSlot), r1);
    mcu.pc = 0x53f6;
}

/* 0x53f6 MOVG2 @r1+0xcfe4 r6 */
void step_materialize_movg2_at_r1_plus_0xcfe4_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(1, kSoaCfe4));
    mcu.pc = 0x53fa;
}

/* 0x53fa MOVG3 r6 -> @r0+0x0098 */
void step_materialize_movg3_r6_to_at_r0_plus_0x98(void)
{
    uint16_t &r6 = mcu.r[6];

    store8(ind_addr(0, kAosE1), (uint8_t)r6);
    mcu.pc = 0x53fe;
}

/* 0x53fe MOVG2 @r1+0xd038 r6 */
void step_materialize_movg2_at_r1_plus_0xd038_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(1, kSoaD038));
    mcu.pc = 0x5402;
}

/* 0x5402 MOVG3 r6 -> @r0+0x0099 */
void step_materialize_movg3_r6_to_at_r0_plus_0x99(void)
{
    uint16_t &r6 = mcu.r[6];

    store8(ind_addr(0, kAosE2), (uint8_t)r6);
    mcu.pc = 0x5406;
}

/* 0x5406 MOVG2 @r1+0xd08c r6 */
void step_materialize_movg2_at_r1_plus_0xd08c_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(1, kSoaD08c));
    mcu.pc = 0x540a;
}

/* 0x540a MOVG3 r6 -> @r0+0x009a */
void step_materialize_movg3_r6_to_at_r0_plus_0x9a(void)
{
    uint16_t &r6 = mcu.r[6];

    store8(ind_addr(0, kAosE3), (uint8_t)r6);
    mcu.pc = 0x540e;
}

/* 0x540e MOVG2 @r3+0xcfac r6 */
void step_materialize_movg2_at_r3_plus_0xcfac_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(3, kSoaCfac));
    mcu.pc = 0x5412;
}

/* 0x5412 MOVG3 r6 -> @r0+0x009c */
void step_materialize_movg3_r6_to_at_r0_plus_0x9c(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, kAosP1), r6);
    mcu.pc = 0x5416;
}

/* 0x5416 MOVG2 @r3+0xd000 r6 */
void step_materialize_movg2_at_r3_plus_0xd000_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(3, kSoaD000));
    mcu.pc = 0x541a;
}

/* 0x541a MOVG3 r6 -> @r0+0x009e */
void step_materialize_movg3_r6_to_at_r0_plus_0x9e(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, kAosP2), r6);
    mcu.pc = 0x541e;
}

/* 0x541e MOVG2 @r3+0xd054 r6 */
void step_materialize_movg2_at_r3_plus_0xd054_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(3, kSoaD054));
    mcu.pc = 0x5422;
}

/* 0x5422 MOVG3 r6 -> @r0+0x00a0 */
void step_materialize_movg3_r6_to_at_r0_plus_0xa0(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, kAosP3), r6);
    mcu.pc = 0x5426;
}

/* 0x5426 MOVG2 @r1+0xcf90 r6 */
void step_materialize_movg2_at_r1_plus_0xcf90_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(1, kSoaCf90));
    mcu.pc = 0x542a;
}

/* 0x542a MOVG3 r6 -> @r0+-59 */
void step_materialize_movg3_r6_to_at_r0_minus_59(void)
{
    uint16_t &r6 = mcu.r[6];

    store8(ind_addr(0, (uint16_t)kAosFlag45), (uint8_t)r6);
    mcu.pc = 0x542d;
}

/* 0x542d BTSTI r6 #7 */
void step_materialize_btsti_r6_7(void)
{
    uint16_t &r6 = mcu.r[6];

    MCU_SetStatus((r6 & 0x80u) == 0, STATUS_Z);
    mcu.pc = 0x542f;
}

/* 0x542f BEQ 5 -> 0x5436 */
void step_materialize_beq_5_to_0x5436(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x5436 : 0x5431;
}

/* 0x5431 MOVG #0xff -> @r1+0xad0e */
void step_materialize_movg_0xff_to_at_r1_plus_0xad0e(void)
{
    store8(ind_addr(1, kAd0e), 0xff);
    mcu.pc = 0x5436;
}

/* 0x5436 CLR r3 */
void step_materialize_clr_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    clr16(r3);
    mcu.pc = 0x5438;
}

/* 0x5438 MOVG2 @r1+0xce78 r3 */
void step_materialize_movg2_at_r1_plus_0xce78_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load8(r3, ind_addr(1, kSoaCe78));
    mcu.pc = 0x543c;
}

/* 0x543c MOVG3 r3 -> @r0+0x009b */
void step_materialize_movg3_r3_to_at_r0_plus_0x9b(void)
{
    uint16_t &r3 = mcu.r[3];

    store8(ind_addr(0, kAosToneIdx), (uint8_t)r3);
    mcu.pc = 0x5440;
}

/* 0x5440 ADD r3 r3 */
void step_materialize_add_r3_r3_a(void)
{
    uint16_t &r3 = mcu.r[3];

    add16(r3, r3);
    mcu.pc = 0x5442;
}

/* 0x5442 MOVG2 @r3+0x7218 r3 */
void step_materialize_movg2_at_r3_plus_0x7218_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load16(r3, ind_addr(3, kTonePtrTable));
    mcu.pc = 0x5446;
}

/* 0x5446 MOVG3 r3 -> @r0+46 */
void step_materialize_movg3_r3_to_at_r0_plus_46(void)
{
    uint16_t &r3 = mcu.r[3];

    store16(ind_addr(0, kAosToneRec), r3);
    mcu.pc = 0x5449;
}

/* 0x5449 CLR r3 */
void step_materialize_clr_r3_a(void)
{
    uint16_t &r3 = mcu.r[3];

    clr16(r3);
    mcu.pc = 0x544b;
}

/* 0x544b TST @r1+0xd0fc */
void step_materialize_tst_at_r1_plus_0xd0fc(void)
{
    { uint8_t sel = MCU_Read(ind_addr(1, kSoaD0fc)); MCU_SetStatusCommon(sel, 0); MCU_SetStatus(0, STATUS_C); mcu.pc = 0x544f; }
}

/* 0x544f BEQ 9 -> 0x545a */
void step_materialize_beq_9_to_0x545a(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x545a : 0x5451;
}

/* 0x5451 BMI 22 -> 0x5469 */
void step_materialize_bmi_22_to_0x5469(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5469 : 0x5453;
}

/* 0x5453 movi r3 #0x8bd4 */
void step_materialize_movi_r3_0x8bd4(void)
{
    uint16_t &r3 = mcu.r[3];

    movi16(r3, kAosBasePos);
    mcu.pc = 0x5456;
}

/* 0x5456 bsr 21 -> 0x546d */
void step_materialize_bsr_21_to_0x546d(void)
{
    MCU_PushStack(0x5458);
    mcu.pc = 0x546d;
}

/* 0x5458 BRA 5 -> 0x545f */
void step_materialize_bra_5_to_0x545f(void)
{
    mcu.pc = 0x545f;
}

/* 0x545a movi r3 #0x8748 */
void step_materialize_movi_r3_0x8748(void)
{
    uint16_t &r3 = mcu.r[3];

    movi16(r3, kAosBaseZero);
    mcu.pc = 0x545d;
}

/* 0x545d bsr 14 -> 0x546d */
void step_materialize_bsr_14_to_0x546d(void)
{
    MCU_PushStack(0x545f);
    mcu.pc = 0x546d;
}

/* 0x545f CLR r6 */
void step_materialize_clr_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    clr16(r6);
    mcu.pc = 0x5461;
}

/* 0x5461 MOVG2 @r1+0xcecc r6 */
void step_materialize_movg2_at_r1_plus_0xcecc_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(1, kSoaCecc));
    mcu.pc = 0x5465;
}

/* 0x5465 ADD r6 r3 */
void step_materialize_add_r6_r3(void)
{
    uint16_t &r3 = mcu.r[3];
    uint16_t &r6 = mcu.r[6];

    add16(r3, r6);
    mcu.pc = 0x5467;
}

/* 0x5467 BRA 0 -> 0x5469 */
void step_materialize_bra_0_to_0x5469(void)
{
    mcu.pc = 0x5469;
}

/* 0x5469 MOVG3 r3 -> @r0+48 */
void step_materialize_movg3_r3_to_at_r0_plus_48(void)
{
    uint16_t &r3 = mcu.r[3];

    store16(ind_addr(0, kAosToneVal), r3);
    mcu.pc = 0x546c;
}

/* 0x546c rts */
void step_materialize_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x546d rts */
void step_materialize_rts_a(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- S2: coeff_copy 0x5d6e-0x5dc2 (per instruction) ---------------------- */

/* 0x5d6e MOVG2 @r2+0x0086 r6 */
void step_coeff_copy_movg2_at_r2_plus_0x86_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef86));
    mcu.pc = 0x5d72;
}

/* 0x5d72 MOVG3 r6 -> @r0+0x0086 */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x86(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef86), r6);
    mcu.pc = 0x5d76;
}

/* 0x5d76 MOVG2 @r2+0x008a r6 */
void step_coeff_copy_movg2_at_r2_plus_0x8a_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef8a));
    mcu.pc = 0x5d7a;
}

/* 0x5d7a MOVG3 r6 -> @r0+0x008a */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x8a(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef8a), r6);
    mcu.pc = 0x5d7e;
}

/* 0x5d7e MOVG2 @r2+0x0088 r6 */
void step_coeff_copy_movg2_at_r2_plus_0x88_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef88));
    mcu.pc = 0x5d82;
}

/* 0x5d82 MOVG3 r6 -> @r0+0x0088 */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x88(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef88), r6);
    mcu.pc = 0x5d86;
}

/* 0x5d86 MOVG2 @r2+-80 r6 */
void step_coeff_copy_movg2_at_r2_minus_80_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoefP80));
    mcu.pc = 0x5d89;
}

/* 0x5d89 MOVG3 r6 -> @r0+-80 */
void step_coeff_copy_movg3_r6_to_at_r0_minus_80(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoefP80), r6);
    mcu.pc = 0x5d8c;
}

/* 0x5d8c MOVG2 @r2+-114 r6 */
void step_coeff_copy_movg2_at_r2_minus_114_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoefP114));
    mcu.pc = 0x5d8f;
}

/* 0x5d8f MOVG3 r6 -> @r0+-114 */
void step_coeff_copy_movg3_r6_to_at_r0_minus_114(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoefP114), r6);
    mcu.pc = 0x5d92;
}

/* 0x5d92 MOVG2 @r2+0x0096 r6 */
void step_coeff_copy_movg2_at_r2_plus_0x96_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef96));
    mcu.pc = 0x5d96;
}

/* 0x5d96 MOVG3 r6 -> @r0+0x0096 */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x96(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef96), r6);
    mcu.pc = 0x5d9a;
}

/* 0x5d9a MOVG2 @r2+0x008e r6 */
void step_coeff_copy_movg2_at_r2_plus_0x8e_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef8e));
    mcu.pc = 0x5d9e;
}

/* 0x5d9e MOVG3 r6 -> @r0+0x008e */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x8e(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef8e), r6);
    mcu.pc = 0x5da2;
}

/* 0x5da2 MOVG2 @r2+0x0094 r6 */
void step_coeff_copy_movg2_at_r2_plus_0x94_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef94));
    mcu.pc = 0x5da6;
}

/* 0x5da6 MOVG3 r6 -> @r0+0x0094 */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x94(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef94), r6);
    mcu.pc = 0x5daa;
}

/* 0x5daa MOVG2 @r2+0x008c r6 */
void step_coeff_copy_movg2_at_r2_plus_0x8c_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef8c));
    mcu.pc = 0x5dae;
}

/* 0x5dae MOVG3 r6 -> @r0+0x008c */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x8c(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef8c), r6);
    mcu.pc = 0x5db2;
}

/* 0x5db2 MOVG2 @r2+0x0092 r6 */
void step_coeff_copy_movg2_at_r2_plus_0x92_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef92));
    mcu.pc = 0x5db6;
}

/* 0x5db6 MOVG3 r6 -> @r0+0x0092 */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x92(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef92), r6);
    mcu.pc = 0x5dba;
}

/* 0x5dba MOVG2 @r2+0x0090 r6 */
void step_coeff_copy_movg2_at_r2_plus_0x90_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)kCoef90));
    mcu.pc = 0x5dbe;
}

/* 0x5dbe MOVG3 r6 -> @r0+0x0090 */
void step_coeff_copy_movg3_r6_to_at_r0_plus_0x90(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)kCoef90), r6);
    mcu.pc = 0x5dc2;
}

/* 0x5dc2 rts */
void step_coeff_copy_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- S3/S4/S5: per-PC hand entries -------------------------------------- */

/*
 * tone_fields / cross_copy / gate_setup all run at IML=0 and the stock run is
 * interrupted inside them at many different instruction boundaries (the union
 * trace lists a resume PC at almost every instruction). A whole-routine block
 * would defer those interrupts and change the handler's saved resume PC / RAM
 * residue, so these three routines are stepped one H8 instruction per
 * MK2CPP_Step: each PC in the routine is registered as an entry that returns 1
 * (host adds 12*(1-1) plus its own +12) and executes exactly the stock
 * instruction, including ex_ignore and the branch targets. This is the L0
 * behaviour the slice spec (out/m4/15 2.3/2.5) requires for IML=0 code,
 * expressed through the routine API so the counts stay explicit.
 *
 * Resume PCs seen in the captured stock runs ([144,175) + [200,340)):
 *   tone_fields: 3588 3598 359a 359f 35a2 35cc 35d1 35d9 35df 35ee 35f5 3610
 *   gate_setup : 3626 3695 36bf 36c3 36cb 36d1 36e2 3704
 *   cross_copy : (none; registered per PC anyway, both entries covered)
 */


/* 0x3580 LDC @r0+0x0099 r4 */
void step_tone_fields_ldc_at_r0_plus_0x99_r4(void)
{
    mcu.ep = MCU_Read(ind_addr(0, 0x99));
    mcu.ex_ignore = 1;
    mcu.pc = 0x3584;
}

/* 0x3584 MOVG2 @r0+0x009e r5 */
void step_tone_fields_movg2_at_r0_plus_0x9e_r5(void)
{
    load16(mcu.r[5], ind_addr(0, 0x9e));
    mcu.pc = 0x3588;
}

/* 0x3588 CLR r2 */
void step_tone_fields_clr_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    clr16(r2);
    mcu.pc = 0x358a;
}

/* 0x358a MOVG2 @r5+73 r2 */
void step_tone_fields_movg2_at_r5_plus_73_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(5, 73));
    mcu.pc = 0x358d;
}

/* 0x358d BPL 9 -> 0x3598 */
void step_tone_fields_bpl_9_to_0x3598(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x358f : 0x3598;
}

/* 0x358f AND #0x7f r2 */
void step_tone_fields_and_0x7f_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    and8_imm(r2, 0x7f);
    mcu.pc = 0x3592;
}

/* 0x3592 SWAP r2 */
void step_tone_fields_swap_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    swap16(r2);
    mcu.pc = 0x3594;
}

/* 0x3594 NEG r2 */
void step_tone_fields_neg_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3596;
}

/* 0x3596 BRA 2 -> 0x359a */
void step_tone_fields_bra_2_to_0x359a(void)
{
    mcu.pc = 0x359a;
}

/* 0x3598 SWAP r2 */
void step_tone_fields_swap_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    swap16(r2);
    mcu.pc = 0x359a;
}

/* 0x359a MOVG3 r2 -> @r0+-94 */
void step_tone_fields_movg3_r2_to_at_r0_minus_94(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-94), r2);
    mcu.pc = 0x359d;
}

/* 0x359d CLR r2 */
void step_tone_fields_clr_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    clr16(r2);
    mcu.pc = 0x359f;
}

/* 0x359f MOVG2 @r5+72 r2 */
void step_tone_fields_movg2_at_r5_plus_72_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(5, 0x48));
    mcu.pc = 0x35a2;
}

/* 0x35a2 BPL 9 -> 0x35ad */
void step_tone_fields_bpl_9_to_0x35ad(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x35a4 : 0x35ad;
}

/* 0x35a4 AND #0x7f r2 */
void step_tone_fields_and_0x7f_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    and8_imm(r2, 0x7f);
    mcu.pc = 0x35a7;
}

/* 0x35a7 SWAP r2 */
void step_tone_fields_swap_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    swap16(r2);
    mcu.pc = 0x35a9;
}

/* 0x35a9 NEG r2 */
void step_tone_fields_neg_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x35ab;
}

/* 0x35ab BRA 2 -> 0x35af */
void step_tone_fields_bra_2_to_0x35af(void)
{
    mcu.pc = 0x35af;
}

/* 0x35ad SWAP r2 */
void step_tone_fields_swap_r2_c(void)
{
    uint16_t &r2 = mcu.r[2];

    swap16(r2);
    mcu.pc = 0x35af;
}

/* 0x35af MOVG3 r2 -> @r0+-128 */
void step_tone_fields_movg3_r2_to_at_r0_minus_128(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-128), r2);
    mcu.pc = 0x35b2;
}

/* 0x35b2 CLR r2 */
void step_tone_fields_clr_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    clr16(r2);
    mcu.pc = 0x35b4;
}

/* 0x35b4 MOVG2 @r5+43 r2 */
void step_tone_fields_movg2_at_r5_plus_43_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(5, 0x2b));
    mcu.pc = 0x35b7;
}

/* 0x35b7 BPL 13 -> 0x35c6 */
void step_tone_fields_bpl_13_to_0x35c6(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x35b9 : 0x35c6;
}

/* 0x35b9 AND #0x7f r2 */
void step_tone_fields_and_0x7f_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    and8_imm(r2, 0x7f);
    mcu.pc = 0x35bc;
}

/* 0x35bc ADD r2 r2 */
void step_tone_fields_add_r2_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x35be;
}

/* 0x35be MOVG2 @r2+0x6f86 r2 */
void step_tone_fields_movg2_at_r2_plus_0x6f86_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x6f86));
    mcu.pc = 0x35c2;
}

/* 0x35c2 NEG r2 */
void step_tone_fields_neg_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x35c4;
}

/* 0x35c4 BRA 6 -> 0x35cc */
void step_tone_fields_bra_6_to_0x35cc(void)
{
    mcu.pc = 0x35cc;
}

/* 0x35c6 ADD r2 r2 */
void step_tone_fields_add_r2_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x35c8;
}

/* 0x35c8 MOVG2 @r2+0x6f86 r2 */
void step_tone_fields_movg2_at_r2_plus_0x6f86_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x6f86));
    mcu.pc = 0x35cc;
}

/* 0x35cc MOVG3 r2 -> @r0+-92 */
void step_tone_fields_movg3_r2_to_at_r0_minus_92(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-92), r2);
    mcu.pc = 0x35cf;
}

/* 0x35cf CLR r2 */
void step_tone_fields_clr_r2_c(void)
{
    uint16_t &r2 = mcu.r[2];

    clr16(r2);
    mcu.pc = 0x35d1;
}

/* 0x35d1 MOVG2 @r5+42 r2 */
void step_tone_fields_movg2_at_r5_plus_42_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(5, 0x2a));
    mcu.pc = 0x35d4;
}

/* 0x35d4 BPL 13 -> 0x35e3 */
void step_tone_fields_bpl_13_to_0x35e3(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x35d6 : 0x35e3;
}

/* 0x35d6 AND #0x7f r2 */
void step_tone_fields_and_0x7f_r2_c(void)
{
    uint16_t &r2 = mcu.r[2];

    and8_imm(r2, 0x7f);
    mcu.pc = 0x35d9;
}

/* 0x35d9 ADD r2 r2 */
void step_tone_fields_add_r2_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x35db;
}

/* 0x35db MOVG2 @r2+0x6f86 r2 */
void step_tone_fields_movg2_at_r2_plus_0x6f86_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x6f86));
    mcu.pc = 0x35df;
}

/* 0x35df NEG r2 */
void step_tone_fields_neg_r2_c(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x35e1;
}

/* 0x35e1 BRA 6 -> 0x35e9 */
void step_tone_fields_bra_6_to_0x35e9(void)
{
    mcu.pc = 0x35e9;
}

/* 0x35e3 ADD r2 r2 */
void step_tone_fields_add_r2_r2_c(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x35e5;
}

/* 0x35e5 MOVG2 @r2+0x6f86 r2 */
void step_tone_fields_movg2_at_r2_plus_0x6f86_r2_c(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x6f86));
    mcu.pc = 0x35e9;
}

/* 0x35e9 MOVG3 r2 -> @r0+-126 */
void step_tone_fields_movg3_r2_to_at_r0_minus_126(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-126), r2);
    mcu.pc = 0x35ec;
}

/* 0x35ec CLR r2 */
void step_tone_fields_clr_r2_d(void)
{
    uint16_t &r2 = mcu.r[2];

    clr16(r2);
    mcu.pc = 0x35ee;
}

/* 0x35ee MOVG2 @r5+15 r2 */
void step_tone_fields_movg2_at_r5_plus_15_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(5, 0x0f));
    mcu.pc = 0x35f1;
}

/* 0x35f1 TST r2 */
void step_tone_fields_tst_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    tst8(r2);
    mcu.pc = 0x35f3;
}

/* 0x35f3 BPL 13 -> 0x3602 */
void step_tone_fields_bpl_13_to_0x3602(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x35f5 : 0x3602;
}

/* 0x35f5 AND #0x7f r2 */
void step_tone_fields_and_0x7f_r2_d(void)
{
    uint16_t &r2 = mcu.r[2];

    and8_imm(r2, 0x7f);
    mcu.pc = 0x35f8;
}

/* 0x35f8 ADD r2 r2 */
void step_tone_fields_add_r2_r2_d(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x35fa;
}

/* 0x35fa MOVG2 @r2+0x7086 r2 */
void step_tone_fields_movg2_at_r2_plus_0x7086_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x7086));
    mcu.pc = 0x35fe;
}

/* 0x35fe NEG r2 */
void step_tone_fields_neg_r2_d(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3600;
}

/* 0x3600 BRA 6 -> 0x3608 */
void step_tone_fields_bra_6_to_0x3608(void)
{
    mcu.pc = 0x3608;
}

/* 0x3602 ADD r2 r2 */
void step_tone_fields_add_r2_r2_e(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x3604;
}

/* 0x3604 MOVG2 @r2+0x7086 r2 */
void step_tone_fields_movg2_at_r2_plus_0x7086_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x7086));
    mcu.pc = 0x3608;
}

/* 0x3608 MOVG3 r2 -> @r0+-90 */
void step_tone_fields_movg3_r2_to_at_r0_minus_90(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-90), r2);
    mcu.pc = 0x360b;
}

/* 0x360b CLR r2 */
void step_tone_fields_clr_r2_e(void)
{
    uint16_t &r2 = mcu.r[2];

    clr16(r2);
    mcu.pc = 0x360d;
}

/* 0x360d MOVG2 @r5+14 r2 */
void step_tone_fields_movg2_at_r5_plus_14_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(5, 0x0e));
    mcu.pc = 0x3610;
}

/* 0x3610 MOVG3 r2 -> @r0+0x00a8 */
void step_tone_fields_movg3_r2_to_at_r0_plus_0xa8(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, 0xa8), r2);
    mcu.pc = 0x3614;
}

/* 0x3614 rts */
void step_tone_fields_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- S4: cross_copy 0x3a9e..0x3bb1 (per instruction, both entries) ---- */

/* 0x3a9e MOVG2 @r2+-2 r1 */
void step_cross_copy_movg2_at_r2_minus_2_r1(void)
{
    load16(mcu.r[1], ind_addr(2, (uint16_t)-2));
    mcu.pc = 0x3aa1;
}

/* 0x3aa1 ADD r1 r1 */
void step_cross_copy_add_r1_r1(void)
{
    add16(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x3aa3;
}

/* 0x3aa3 MOVG2 @r1+0xcdc6 r6 */
void step_cross_copy_movg2_at_r1_plus_0xcdc6_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(1, 0xcdc6));
    mcu.pc = 0x3aa7;
}

/* 0x3aa7 MOVG2 @r0+-2 r1 */
void step_cross_copy_movg2_at_r0_minus_2_r1(void)
{
    load16(mcu.r[1], ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x3aaa;
}

/* 0x3aaa ADD r1 r1 */
void step_cross_copy_add_r1_r1_a(void)
{
    add16(mcu.r[1], mcu.r[1]);
    mcu.pc = 0x3aac;
}

/* 0x3aac MOVG3 r6 -> @r1+0xcdc6 */
void step_cross_copy_movg3_r6_to_at_r1_plus_0xcdc6(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(1, 0xcdc6), r6);
    mcu.pc = 0x3ab0;
}

/* 0x3ab0 MOVG2 @r2+-25 r6 */
void step_cross_copy_movg2_at_r2_minus_25_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(2, (uint16_t)-25));
    mcu.pc = 0x3ab3;
}

/* 0x3ab3 MOVG3 r6 -> @r0+-25 */
void step_cross_copy_movg3_r6_to_at_r0_minus_25(void)
{
    uint16_t &r6 = mcu.r[6];

    store8(ind_addr(0, (uint16_t)-25), (uint8_t)r6);
    mcu.pc = 0x3ab6;
}

/* 0x3ab6 MOVG2 @r2+-112 r6 */
void step_cross_copy_movg2_at_r2_minus_112_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-112));
    mcu.pc = 0x3ab9;
}

/* 0x3ab9 MOVG3 r6 -> @r0+-112 */
void step_cross_copy_movg3_r6_to_at_r0_minus_112(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-112), r6);
    mcu.pc = 0x3abc;
}

/* 0x3abc MOVG2 @r2+-110 r6 */
void step_cross_copy_movg2_at_r2_minus_110_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-110));
    mcu.pc = 0x3abf;
}

/* 0x3abf MOVG3 r6 -> @r0+-110 */
void step_cross_copy_movg3_r6_to_at_r0_minus_110(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-110), r6);
    mcu.pc = 0x3ac2;
}

/* 0x3ac2 MOVG2 @r2+-108 r6 */
void step_cross_copy_movg2_at_r2_minus_108_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-108));
    mcu.pc = 0x3ac5;
}

/* 0x3ac5 MOVG3 r6 -> @r0+-108 */
void step_cross_copy_movg3_r6_to_at_r0_minus_108(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-108), r6);
    mcu.pc = 0x3ac8;
}

/* 0x3ac8 MOVG2 @r2+-106 r6 */
void step_cross_copy_movg2_at_r2_minus_106_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-106));
    mcu.pc = 0x3acb;
}

/* 0x3acb MOVG3 r6 -> @r0+-106 */
void step_cross_copy_movg3_r6_to_at_r0_minus_106(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-106), r6);
    mcu.pc = 0x3ace;
}

/* 0x3ace MOVG2 @r2+-104 r6 */
void step_cross_copy_movg2_at_r2_minus_104_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-104));
    mcu.pc = 0x3ad1;
}

/* 0x3ad1 MOVG3 r6 -> @r0+-104 */
void step_cross_copy_movg3_r6_to_at_r0_minus_104(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-104), r6);
    mcu.pc = 0x3ad4;
}

/* 0x3ad4 MOVG2 @r2+-102 r6 */
void step_cross_copy_movg2_at_r2_minus_102_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-102));
    mcu.pc = 0x3ad7;
}

/* 0x3ad7 MOVG3 r6 -> @r0+-102 */
void step_cross_copy_movg3_r6_to_at_r0_minus_102(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-102), r6);
    mcu.pc = 0x3ada;
}

/* 0x3ada MOVG2 @r2+-100 r6 */
void step_cross_copy_movg2_at_r2_minus_100_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-100));
    mcu.pc = 0x3add;
}

/* 0x3add MOVG3 r6 -> @r0+-100 */
void step_cross_copy_movg3_r6_to_at_r0_minus_100(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-100), r6);
    mcu.pc = 0x3ae0;
}

/* 0x3ae0 MOVG2 @r2+-98 r6 */
void step_cross_copy_movg2_at_r2_minus_98_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-98));
    mcu.pc = 0x3ae3;
}

/* 0x3ae3 MOVG3 r6 -> @r0+-98 */
void step_cross_copy_movg3_r6_to_at_r0_minus_98(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-98), r6);
    mcu.pc = 0x3ae6;
}

/* 0x3ae6 MOVG2 @r2+-96 r6 */
void step_cross_copy_movg2_at_r2_minus_96_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-96));
    mcu.pc = 0x3ae9;
}

/* 0x3ae9 MOVG3 r6 -> @r0+-96 */
void step_cross_copy_movg3_r6_to_at_r0_minus_96(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-96), r6);
    mcu.pc = 0x3aec;
}

/* 0x3aec MOVG2 @r2+-115 r6 */
void step_cross_copy_movg2_at_r2_minus_115_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(2, (uint16_t)-115));
    mcu.pc = 0x3aef;
}

/* 0x3aef MOVG3 r6 -> @r0+-115 */
void step_cross_copy_movg3_r6_to_at_r0_minus_115(void)
{
    uint16_t &r6 = mcu.r[6];

    store8(ind_addr(0, (uint16_t)-115), (uint8_t)r6);
    mcu.pc = 0x3af2;
}

/* 0x3af2 MOVG2 @r2+-114 r6 */
void step_cross_copy_movg2_at_r2_minus_114_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(2, (uint16_t)-114));
    mcu.pc = 0x3af5;
}

/* 0x3af5 MOVG3 r6 -> @r0+-114 */
void step_cross_copy_movg3_r6_to_at_r0_minus_114(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-114), r6);
    mcu.pc = 0x3af8;
}

/* 0x3af8 MOVG2 @r0+-104 r6 */
void step_cross_copy_movg2_at_r0_minus_104_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-104));
    mcu.pc = 0x3afb;
}

/* 0x3afb cmp r6,w #0xffff */
void step_cross_copy_cmp_r6_w_0xffff(void)
{
    uint16_t &r6 = mcu.r[6];

    cmp16(r6, 0xffff);
    mcu.pc = 0x3afe;
}

/* 0x3afe BEQ 10 -> 0x3b0a */
void step_cross_copy_beq_10_to_0x3b0a(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3b0a : 0x3b00;
}

/* 0x3b00 CLR @r0+-122 */
void step_cross_copy_clr_at_r0_minus_122(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-122));
    mcu.pc = 0x3b03;
}

/* 0x3b03 CLR @r0+-120 */
void step_cross_copy_clr_at_r0_minus_120(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-120));
    mcu.pc = 0x3b06;
}

/* 0x3b06 CLR @r0+-118 */
void step_cross_copy_clr_at_r0_minus_118(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-118));
    mcu.pc = 0x3b09;
}

/* 0x3b09 rts */
void step_cross_copy_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x3b0a MOVG2 @r0+-102 r6 */
void step_cross_copy_movg2_at_r0_minus_102_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-102));
    mcu.pc = 0x3b0d;
}

/* 0x3b0d cmp r6,w #0xffff */
void step_cross_copy_cmp_r6_w_0xffff_a(void)
{
    uint16_t &r6 = mcu.r[6];

    cmp16(r6, 0xffff);
    mcu.pc = 0x3b10;
}

/* 0x3b10 BEQ 0x008f -> 0x3ba2 */
void step_cross_copy_beq_0x8f_to_0x3ba2(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3ba2 : 0x3b13;
}

/* 0x3b13 MOVG2 @r0+-128 r2 */
void step_cross_copy_movg2_at_r0_minus_128_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(0, (uint16_t)-128));
    mcu.pc = 0x3b16;
}

/* 0x3b16 BPL 8 -> 0x3b20 */
void step_cross_copy_bpl_8_to_0x3b20(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x3b18 : 0x3b20;
}

/* 0x3b18 NEG r2 */
void step_cross_copy_neg_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3b1a;
}

/* 0x3b1a MULXU r6 r2:r3 */
void step_cross_copy_mulxu_r6_r2_r3(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r6 = mcu.r[6];

    mulxu16(r6, r2, mcu.r[3]);
    mcu.pc = 0x3b1c;
}

/* 0x3b1c NEG r2 */
void step_cross_copy_neg_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3b1e;
}

/* 0x3b1e BRA 2 -> 0x3b22 */
void step_cross_copy_bra_2_to_0x3b22(void)
{
    mcu.pc = 0x3b22;
}

/* 0x3b20 MULXU r6 r2:r3 */
void step_cross_copy_mulxu_r6_r2_r3_a(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r6 = mcu.r[6];

    mulxu16(r6, r2, mcu.r[3]);
    mcu.pc = 0x3b22;
}

/* 0x3b22 MOVG3 r2 -> @r0+-122 */
void step_cross_copy_movg3_r2_to_at_r0_minus_122(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-122), r2);
    mcu.pc = 0x3b25;
}

/* 0x3b25 MOVG2 @r0+-126 r2 */
void step_cross_copy_movg2_at_r0_minus_126_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(0, (uint16_t)-126));
    mcu.pc = 0x3b28;
}

/* 0x3b28 BPL 8 -> 0x3b32 */
void step_cross_copy_bpl_8_to_0x3b32(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x3b2a : 0x3b32;
}

/* 0x3b2a NEG r2 */
void step_cross_copy_neg_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3b2c;
}

/* 0x3b2c MULXU r6 r2:r3 */
void step_cross_copy_mulxu_r6_r2_r3_b(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r6 = mcu.r[6];

    mulxu16(r6, r2, mcu.r[3]);
    mcu.pc = 0x3b2e;
}

/* 0x3b2e NEG r2 */
void step_cross_copy_neg_r2_c(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3b30;
}

/* 0x3b30 BRA 2 -> 0x3b34 */
void step_cross_copy_bra_2_to_0x3b34(void)
{
    mcu.pc = 0x3b34;
}

/* 0x3b32 MULXU r6 r2:r3 */
void step_cross_copy_mulxu_r6_r2_r3_c(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r6 = mcu.r[6];

    mulxu16(r6, r2, mcu.r[3]);
    mcu.pc = 0x3b34;
}

/* 0x3b34 MOVG3 r2 -> @r0+-120 */
void step_cross_copy_movg3_r2_to_at_r0_minus_120(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-120), r2);
    mcu.pc = 0x3b37;
}

/* 0x3b37 CLR r3 */
void step_cross_copy_clr_r3(void)
{
    clr16(mcu.r[3]);
    mcu.pc = 0x3b39;
}

/* 0x3b39 MOVG2 @r0+46 r2 */
void step_cross_copy_movg2_at_r0_plus_46_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(0, 46));
    mcu.pc = 0x3b3c;
}

/* 0x3b3c MOVG2 @r2+17 r3 */
void step_cross_copy_movg2_at_r2_plus_17_r3(void)
{
    load8(mcu.r[3], ind_addr(2, 17));
    mcu.pc = 0x3b3f;
}

/* 0x3b3f MOVG2 @r0+0x00a8 r2 */
void step_cross_copy_movg2_at_r0_plus_0xa8_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(0, 0xa8));
    mcu.pc = 0x3b43;
}

/* 0x3b43 TST r2 */
void step_cross_copy_tst_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    tst8(r2);
    mcu.pc = 0x3b45;
}

/* 0x3b45 BPL 10 -> 0x3b51 */
void step_cross_copy_bpl_10_to_0x3b51(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x3b47 : 0x3b51;
}

/* 0x3b47 AND #0x7f r2 */
void step_cross_copy_and_0x7f_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    and8_imm(r2, 0x7f);
    mcu.pc = 0x3b4a;
}

/* 0x3b4a SUB #0x40 r3 */
void step_cross_copy_sub_0x40_r3(void)
{
    sub8_imm(mcu.r[3], 0x40);
    mcu.pc = 0x3b4d;
}

/* 0x3b4d BCC 41 -> 0x3b78 */
void step_cross_copy_bcc_41_to_0x3b78(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x3b4f : 0x3b78;
}

/* 0x3b4f BRA 27 -> 0x3b6c */
void step_cross_copy_bra_27_to_0x3b6c(void)
{
    mcu.pc = 0x3b6c;
}

/* 0x3b51 SUB #0x40 r3 */
void step_cross_copy_sub_0x40_r3_a(void)
{
    sub8_imm(mcu.r[3], 0x40);
    mcu.pc = 0x3b54;
}

/* 0x3b54 BCC 12 -> 0x3b62 */
void step_cross_copy_bcc_12_to_0x3b62(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x3b56 : 0x3b62;
}

/* 0x3b56 NEG r3 */
void step_cross_copy_neg_r3(void)
{
    neg8(mcu.r[3]);
    mcu.pc = 0x3b58;
}

/* 0x3b58 ADD r3 r3 */
void step_cross_copy_add_r3_r3(void)
{
    add8(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x3b5a;
}

/* 0x3b5a SUB r3 r2 */
void step_cross_copy_sub_r3_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    sub8(r2, mcu.r[3]);
    mcu.pc = 0x3b5c;
}

/* 0x3b5c BPL 53 -> 0x3b93 */
void step_cross_copy_bpl_53_to_0x3b93(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x3b5e : 0x3b93;
}

/* 0x3b5e CLR r2 */
void step_cross_copy_clr_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    clr8_reg(r2);
    mcu.pc = 0x3b60;
}

/* 0x3b60 BRA 49 -> 0x3b93 */
void step_cross_copy_bra_49_to_0x3b93(void)
{
    mcu.pc = 0x3b93;
}

/* 0x3b62 ADD r3 r3 */
void step_cross_copy_add_r3_r3_a(void)
{
    add8(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x3b64;
}

/* 0x3b64 ADD r3 r2 */
void step_cross_copy_add_r3_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    add8(r2, mcu.r[3]);
    mcu.pc = 0x3b66;
}

/* 0x3b66 BPL 43 -> 0x3b93 */
void step_cross_copy_bpl_43_to_0x3b93(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x3b68 : 0x3b93;
}

/* 0x3b68 move r2 #0x7f */
void step_cross_copy_move_r2_0x7f(void)
{
    uint16_t &r2 = mcu.r[2];

    move8(r2, 0x7f);
    mcu.pc = 0x3b6a;
}

/* 0x3b6a BRA 39 -> 0x3b93 */
void step_cross_copy_bra_39_to_0x3b93(void)
{
    mcu.pc = 0x3b93;
}

/* 0x3b6c NEG r3 */
void step_cross_copy_neg_r3_a(void)
{
    neg8(mcu.r[3]);
    mcu.pc = 0x3b6e;
}

/* 0x3b6e ADD r3 r3 */
void step_cross_copy_add_r3_r3_b(void)
{
    add8(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x3b70;
}

/* 0x3b70 ADD r3 r2 */
void step_cross_copy_add_r3_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    add8(r2, mcu.r[3]);
    mcu.pc = 0x3b72;
}

/* 0x3b72 BPL 12 -> 0x3b80 */
void step_cross_copy_bpl_12_to_0x3b80(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x3b74 : 0x3b80;
}

/* 0x3b74 move r2 #0x7f */
void step_cross_copy_move_r2_0x7f_a(void)
{
    uint16_t &r2 = mcu.r[2];

    move8(r2, 0x7f);
    mcu.pc = 0x3b76;
}

/* 0x3b76 BRA 8 -> 0x3b80 */
void step_cross_copy_bra_8_to_0x3b80(void)
{
    mcu.pc = 0x3b80;
}

/* 0x3b78 ADD r3 r3 */
void step_cross_copy_add_r3_r3_c(void)
{
    add8(mcu.r[3], mcu.r[3]);
    mcu.pc = 0x3b7a;
}

/* 0x3b7a SUB r3 r2 */
void step_cross_copy_sub_r3_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    sub8(r2, mcu.r[3]);
    mcu.pc = 0x3b7c;
}

/* 0x3b7c BPL 2 -> 0x3b80 */
void step_cross_copy_bpl_2_to_0x3b80(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x3b7e : 0x3b80;
}

/* 0x3b7e CLR r2 */
void step_cross_copy_clr_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    clr8_reg(r2);
    mcu.pc = 0x3b80;
}

/* 0x3b80 ADD r2 r2 */
void step_cross_copy_add_r2_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x3b82;
}

/* 0x3b82 MOVG2 @r2+0x7086 r2 */
void step_cross_copy_movg2_at_r2_plus_0x7086_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x7086));
    mcu.pc = 0x3b86;
}

/* 0x3b86 NEG r2 */
void step_cross_copy_neg_r2_d(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3b88;
}

/* 0x3b88 MOVG3 r2 -> @r0+-124 */
void step_cross_copy_movg3_r2_to_at_r0_minus_124(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-124), r2);
    mcu.pc = 0x3b8b;
}

/* 0x3b8b NEG r2 */
void step_cross_copy_neg_r2_e(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3b8d;
}

/* 0x3b8d MULXU r6 r2:r3 */
void step_cross_copy_mulxu_r6_r2_r3_d(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r6 = mcu.r[6];

    mulxu16(r6, r2, mcu.r[3]);
    mcu.pc = 0x3b8f;
}

/* 0x3b8f NEG r2 */
void step_cross_copy_neg_r2_f(void)
{
    uint16_t &r2 = mcu.r[2];

    neg16(r2);
    mcu.pc = 0x3b91;
}

/* 0x3b91 BRA 11 -> 0x3b9e */
void step_cross_copy_bra_11_to_0x3b9e(void)
{
    mcu.pc = 0x3b9e;
}

/* 0x3b93 ADD r2 r2 */
void step_cross_copy_add_r2_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x3b95;
}

/* 0x3b95 MOVG2 @r2+0x7086 r2 */
void step_cross_copy_movg2_at_r2_plus_0x7086_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x7086));
    mcu.pc = 0x3b99;
}

/* 0x3b99 MOVG3 r2 -> @r0+-124 */
void step_cross_copy_movg3_r2_to_at_r0_minus_124_a(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-124), r2);
    mcu.pc = 0x3b9c;
}

/* 0x3b9c MULXU r6 r2:r3 */
void step_cross_copy_mulxu_r6_r2_r3_e(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r6 = mcu.r[6];

    mulxu16(r6, r2, mcu.r[3]);
    mcu.pc = 0x3b9e;
}

/* 0x3b9e MOVG3 r2 -> @r0+-118 */
void step_cross_copy_movg3_r2_to_at_r0_minus_118(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(ind_addr(0, (uint16_t)-118), r2);
    mcu.pc = 0x3ba1;
}

/* 0x3ba1 rts */
void step_cross_copy_rts_a(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x3ba2 MOVG2 @r0+-128 r6 */
void step_cross_copy_movg2_at_r0_minus_128_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-128));
    mcu.pc = 0x3ba5;
}

/* 0x3ba5 MOVG3 r6 -> @r0+-122 */
void step_cross_copy_movg3_r6_to_at_r0_minus_122(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-122), r6);
    mcu.pc = 0x3ba8;
}

/* 0x3ba8 MOVG2 @r0+-126 r6 */
void step_cross_copy_movg2_at_r0_minus_126_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-126));
    mcu.pc = 0x3bab;
}

/* 0x3bab MOVG3 r6 -> @r0+-120 */
void step_cross_copy_movg3_r6_to_at_r0_minus_120(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, (uint16_t)-120), r6);
    mcu.pc = 0x3bae;
}

/* 0x3bae movi r6 #0xffff */
void step_cross_copy_movi_r6_0xffff(void)
{
    uint16_t &r6 = mcu.r[6];

    movi16(r6, 0xffff);
    mcu.pc = 0x3bb1;
}

/* 0x3bb1 BRA -124 -> 0x3b37 */
void step_cross_copy_bra_minus_124_to_0x3b37(void)
{
    mcu.pc = 0x3b37;
}

/* ---- S5: gate_setup 0x3615..0x3708 (per instruction) ------------------ */

/* 0x3615 LDC @r0+0x0098 r4 */
void step_gate_setup_ldc_at_r0_plus_0x98_r4(void)
{
    mcu.ep = MCU_Read(ind_addr(0, 0x98));
    mcu.ex_ignore = 1;
    mcu.pc = 0x3619;
}

/* 0x3619 MOVG2 @r0+0x009c r5 */
void step_gate_setup_movg2_at_r0_plus_0x9c_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load16(r5, ind_addr(0, 0x9c));
    mcu.pc = 0x361d;
}

/* 0x361d MOVG2 @r0+-2 r1 */
void step_gate_setup_movg2_at_r0_minus_2_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x3620;
}

/* 0x3620 ADD r1 r1 */
void step_gate_setup_add_r1_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    add16(r1, r1);
    mcu.pc = 0x3622;
}

/* 0x3622 CLR @r1+0xcdc6 */
void step_gate_setup_clr_at_r1_plus_0xcdc6(void)
{
    clr16_mem(ind_addr(1, 0xcdc6));
    mcu.pc = 0x3626;
}

/* 0x3626 CLR @r0+-115 */
void step_gate_setup_clr_at_r0_minus_115(void)
{
    clr8_mem(ind_addr(0, (uint16_t)-115));
    mcu.pc = 0x3629;
}

/* 0x3629 MOVG2 @r5+14 r6 */
void step_gate_setup_movg2_at_r5_plus_14_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(5, 14));
    mcu.pc = 0x362c;
}

/* 0x362c BTSTI r6 #4 */
void step_gate_setup_btsti_r6_4(void)
{
    uint16_t &r6 = mcu.r[6];

    MCU_SetStatus((r6 & 0x10u) == 0, STATUS_Z);
    mcu.pc = 0x362e;
}

/* 0x362e BEQ 54 -> 0x3666 */
void step_gate_setup_beq_54_to_0x3666(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3666 : 0x3630;
}

/* 0x3630 MOVG2 @r0+0x009b r3 */
void step_gate_setup_movg2_at_r0_plus_0x9b_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load8(r3, ind_addr(0, 0x9b));
    mcu.pc = 0x3634;
}

/* 0x3634 STC r4 -> r4 */
void step_gate_setup_stc_r4_to_r4(void)
{
    stc_ep_to_r4();
    mcu.pc = 0x3636;
}

/* 0x3636 movi r1 #0x001b */
void step_gate_setup_movi_r1_0x1b(void)
{
    uint16_t &r1 = mcu.r[1];

    movi16(r1, 0x001b);
    mcu.pc = 0x3639;
}

/* 0x3639 MOVG2 r1 r2 */
void step_gate_setup_movg2_r1_r2(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];

    move16(r2, r1);
    mcu.pc = 0x363b;
}

/* 0x363b ADD r2 r2 */
void step_gate_setup_add_r2_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    add16(r2, r2);
    mcu.pc = 0x363d;
}

/* 0x363d MOVG2 @r2+0x64d6 r2 */
void step_gate_setup_movg2_at_r2_plus_0x64d6_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(2, 0x64d6));
    mcu.pc = 0x3641;
}

/* 0x3641 CMP r0 r2 */
void step_gate_setup_cmp_r0_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    cmp16(r2, mcu.r[0]);
    mcu.pc = 0x3643;
}

/* 0x3643 BEQ 25 -> 0x365e */
void step_gate_setup_beq_25_to_0x365e(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x365e : 0x3645;
}

/* 0x3645 SUB @r2+0 #0x000c */
void step_gate_setup_sub_at_r2_plus_0_0xc(void)
{
    MCU_Write16(ind_addr(2, 0), 0x000c);
    MCU_SetStatusCommon(0x000c, 1);
    mcu.pc = 0x364a;
}

/* 0x364a BHI 18 -> 0x365e */
void step_gate_setup_bhi_18_to_0x365e(void)
{
    mcu.pc = ((mcu.sr & (STATUS_C | STATUS_Z)) == 0) ? 0x365e : 0x364c;
}

/* 0x364c CMP @r2+0x009b r3 */
void step_gate_setup_cmp_at_r2_plus_0x9b_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    cmp8(r3, MCU_Read(ind_addr(2, 0x9b)));
    mcu.pc = 0x3650;
}

/* 0x3650 BNE 12 -> 0x365e */
void step_gate_setup_bne_12_to_0x365e(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3652 : 0x365e;
}

/* 0x3652 CMP @r2+0x0098 r4 */
void step_gate_setup_cmp_at_r2_plus_0x98_r4(void)
{
    cmp8(mcu.r[4], MCU_Read(ind_addr(2, 0x98)));
    mcu.pc = 0x3656;
}

/* 0x3656 BNE 6 -> 0x365e */
void step_gate_setup_bne_6_to_0x365e(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3658 : 0x365e;
}

/* 0x3658 CMP @r2+0x009c r5 */
void step_gate_setup_cmp_at_r2_plus_0x9c_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    cmp16(r5, MCU_Read16(ind_addr(2, 0x9c)));
    mcu.pc = 0x365c;
}

/* 0x365c BEQ 5 -> 0x3663 */
void step_gate_setup_beq_5_to_0x3663(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x3663 : 0x365e;
}

/* 0x365e cntjmp r1 -40 -> 0x3639 */
void step_gate_setup_cntjmp_r1_minus_40_to_0x3639(void)
{
    uint16_t &r1 = mcu.r[1];

    r1 = (uint16_t)(r1 - 1);
    mcu.pc = (r1 != 0xffff) ? 0x3639 : 0x3661;
}

/* 0x3661 BRA 3 -> 0x3666 */
void step_gate_setup_bra_3_to_0x3666(void)
{
    mcu.pc = 0x3666;
}

/* 0x3663 BRA 0x0438 -> 0x3a9e */
void step_gate_setup_bra_0x438_to_0x3a9e(void)
{
    mcu.pc = 0x3a9e;
}

/* 0x3666 MOVG2 r6 r3 */
void step_gate_setup_movg2_r6_r3(void)
{
    uint16_t &r3 = mcu.r[3];
    uint16_t &r6 = mcu.r[6];

    move8(r3, (uint8_t)r6);
    mcu.pc = 0x3668;
}

/* 0x3668 AND #0x00c0 r3 */
void step_gate_setup_and_0xc0_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    and16_imm(r3, 0x00c0);
    mcu.pc = 0x366c;
}

/* 0x366c SWAP r3 */
void step_gate_setup_swap_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    swap16(r3);
    mcu.pc = 0x366e;
}

/* 0x366e MOVG3 r3 -> @r0+-106 */
void step_gate_setup_movg3_r3_to_at_r0_minus_106(void)
{
    uint16_t &r3 = mcu.r[3];

    store16(ind_addr(0, (uint16_t)-106), r3);
    mcu.pc = 0x3671;
}

/* 0x3671 MOVG2 r6 r3 */
void step_gate_setup_movg2_r6_r3_a(void)
{
    uint16_t &r3 = mcu.r[3];
    uint16_t &r6 = mcu.r[6];

    move8(r3, (uint8_t)r6);
    mcu.pc = 0x3673;
}

/* 0x3673 AND #0x000f r3 */
void step_gate_setup_and_0xf_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    and16_imm(r3, 0x000f);
    mcu.pc = 0x3677;
}

/* 0x3677 MOVG2 @r3+0x7207 r3 */
void step_gate_setup_movg2_at_r3_plus_0x7207_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load8(r3, ind_addr(3, 0x7207));
    mcu.pc = 0x367b;
}

/* 0x367b MOVG3 r3 -> @r0+-108 */
void step_gate_setup_movg3_r3_to_at_r0_minus_108(void)
{
    uint16_t &r3 = mcu.r[3];

    store16(ind_addr(0, (uint16_t)-108), r3);
    mcu.pc = 0x367e;
}

/* 0x367e CLR @r0+-98 */
void step_gate_setup_clr_at_r0_minus_98(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-98));
    mcu.pc = 0x3681;
}

/* 0x3681 CLR @r0+-104 */
void step_gate_setup_clr_at_r0_minus_104(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-104));
    mcu.pc = 0x3684;
}

/* 0x3684 CLR @r0+-102 */
void step_gate_setup_clr_at_r0_minus_102(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-102));
    mcu.pc = 0x3687;
}

/* 0x3687 CLR @r0+-96 */
void step_gate_setup_clr_at_r0_minus_96(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-96));
    mcu.pc = 0x368a;
}

/* 0x368a CLR @r0+-122 */
void step_gate_setup_clr_at_r0_minus_122(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-122));
    mcu.pc = 0x368d;
}

/* 0x368d CLR @r0+-120 */
void step_gate_setup_clr_at_r0_minus_120(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-120));
    mcu.pc = 0x3690;
}

/* 0x3690 CLR @r0+-118 */
void step_gate_setup_clr_at_r0_minus_118(void)
{
    clr16_mem(ind_addr(0, (uint16_t)-118));
    mcu.pc = 0x3693;
}

/* 0x3693 CLR r3 */
void step_gate_setup_clr_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    clr16(r3);
    mcu.pc = 0x3695;
}

/* 0x3695 MOVG2 @r5+16 r3 */
void step_gate_setup_movg2_at_r5_plus_16_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load8(r3, ind_addr(5, 16));
    mcu.pc = 0x3698;
}

/* 0x3698 BMI 39 -> 0x36c1 */
void step_gate_setup_bmi_39_to_0x36c1(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x36c1 : 0x369a;
}

/* 0x369a MOVG2 @r0+46 r2 */
void step_gate_setup_movg2_at_r0_plus_46_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(0, 46));
    mcu.pc = 0x369d;
}

/* 0x369d MOVG2 @r2+23 r2 */
void step_gate_setup_movg2_at_r2_plus_23_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(2, 23));
    mcu.pc = 0x36a0;
}

/* 0x36a0 SUB #0x40 r2 */
void step_gate_setup_sub_0x40_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    sub8_imm(r2, 0x40);
    mcu.pc = 0x36a3;
}

/* 0x36a3 BCC 12 -> 0x36b1 */
void step_gate_setup_bcc_12_to_0x36b1(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x36a5 : 0x36b1;
}

/* 0x36a5 NEG r2 */
void step_gate_setup_neg_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    neg8(r2);
    mcu.pc = 0x36a7;
}

/* 0x36a7 ADD r2 r2 */
void step_gate_setup_add_r2_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    add8(r2, r2);
    mcu.pc = 0x36a9;
}

/* 0x36a9 SUB r2 r3 */
void step_gate_setup_sub_r2_r3(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r3 = mcu.r[3];

    sub8(r3, r2);
    mcu.pc = 0x36ab;
}

/* 0x36ab BPL 12 -> 0x36b9 */
void step_gate_setup_bpl_12_to_0x36b9(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x36ad : 0x36b9;
}

/* 0x36ad CLR r3 */
void step_gate_setup_clr_r3_a(void)
{
    uint16_t &r3 = mcu.r[3];

    clr8_reg(r3);
    mcu.pc = 0x36af;
}

/* 0x36af BRA 8 -> 0x36b9 */
void step_gate_setup_bra_8_to_0x36b9(void)
{
    mcu.pc = 0x36b9;
}

/* 0x36b1 ADD r2 r2 */
void step_gate_setup_add_r2_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    add8(r2, r2);
    mcu.pc = 0x36b3;
}

/* 0x36b3 ADD r2 r3 */
void step_gate_setup_add_r2_r3(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r3 = mcu.r[3];

    add8(r3, r2);
    mcu.pc = 0x36b5;
}

/* 0x36b5 BPL 2 -> 0x36b9 */
void step_gate_setup_bpl_2_to_0x36b9(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x36b7 : 0x36b9;
}

/* 0x36b7 move r3 #0x7f */
void step_gate_setup_move_r3_0x7f(void)
{
    uint16_t &r3 = mcu.r[3];

    move8(r3, 0x7f);
    mcu.pc = 0x36b9;
}

/* 0x36b9 ADD r3 r3 */
void step_gate_setup_add_r3_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    add16(r3, r3);
    mcu.pc = 0x36bb;
}

/* 0x36bb MOVG2 @r3+0x6e86 r3 */
void step_gate_setup_movg2_at_r3_plus_0x6e86_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load16(r3, ind_addr(3, 0x6e86));
    mcu.pc = 0x36bf;
}

/* 0x36bf BRA 2 -> 0x36c3 */
void step_gate_setup_bra_2_to_0x36c3(void)
{
    mcu.pc = 0x36c3;
}

/* 0x36c1 CLR r3 */
void step_gate_setup_clr_r3_b(void)
{
    uint16_t &r3 = mcu.r[3];

    clr16(r3);
    mcu.pc = 0x36c3;
}

/* 0x36c3 MOVG3 r3 -> @r0+-112 */
void step_gate_setup_movg3_r3_to_at_r0_minus_112(void)
{
    uint16_t &r3 = mcu.r[3];

    store16(ind_addr(0, (uint16_t)-112), r3);
    mcu.pc = 0x36c6;
}

/* 0x36c6 CLR r3 */
void step_gate_setup_clr_r3_c(void)
{
    uint16_t &r3 = mcu.r[3];

    clr16(r3);
    mcu.pc = 0x36c8;
}

/* 0x36c8 MOVG2 @r5+17 r3 */
void step_gate_setup_movg2_at_r5_plus_17_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load8(r3, ind_addr(5, 17));
    mcu.pc = 0x36cb;
}

/* 0x36cb ADD r3 r3 */
void step_gate_setup_add_r3_r3_a(void)
{
    uint16_t &r3 = mcu.r[3];

    add16(r3, r3);
    mcu.pc = 0x36cd;
}

/* 0x36cd MOVG2 @r3+0x6e86 r3 */
void step_gate_setup_movg2_at_r3_plus_0x6e86_r3_a(void)
{
    uint16_t &r3 = mcu.r[3];

    load16(r3, ind_addr(3, 0x6e86));
    mcu.pc = 0x36d1;
}

/* 0x36d1 MOVG3 r3 -> @r0+-110 */
void step_gate_setup_movg3_r3_to_at_r0_minus_110(void)
{
    uint16_t &r3 = mcu.r[3];

    store16(ind_addr(0, (uint16_t)-110), r3);
    mcu.pc = 0x36d4;
}

/* 0x36d4 MOVG2 @r5+15 r3 */
void step_gate_setup_movg2_at_r5_plus_15_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load8(r3, ind_addr(5, 15));
    mcu.pc = 0x36d7;
}

/* 0x36d7 MOVG3 r3 -> @r0+-25 */
void step_gate_setup_movg3_r3_to_at_r0_minus_25(void)
{
    uint16_t &r3 = mcu.r[3];

    store8(ind_addr(0, (uint16_t)-25), (uint8_t)r3);
    mcu.pc = 0x36da;
}

/* 0x36da MOVG2 (dp,0xad2a) r6 */
void step_gate_setup_movg2_dp_0xad2a_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, dp_addr(0xad2a));
    mcu.pc = 0x36de;
}

/* 0x36de MOVG3 r6 -> (dp,0xad2c) */
void step_gate_setup_movg3_r6_to_dp_0xad2c(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xad2c), r6);
    mcu.pc = 0x36e2;
}

/* 0x36e2 MOVG #0x0001 -> (dp,0xad2a) */
void step_gate_setup_movg_0x1_to_dp_0xad2a(void)
{
    MCU_Write16(dp_addr(0xad2a), 0x0001);
    MCU_SetStatusCommon(0x0001, 1);
    mcu.pc = 0x36e8;
}

/* 0x36e8 BSET_ORC #0x0700 r0 */
void step_gate_setup_bset_orc_0x700_r0(void)
{
    mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x36ec;
}

/* 0x36ec MOVG #0x1e -> (br,$3e) */
void step_gate_setup_movg_0x1e_to_br_3e(void)
{
    MCU_Write(br_addr(0x3e), 0x1e);
    MCU_SetStatusCommon(0x1e, 0);
    mcu.pc = 0x36f0;
}

/* 0x36f0 movl r4 @(br,$34) */
void step_gate_setup_movl_r4_at_br_34(void)
{
    { uint8_t v = MCU_Read(br_addr(0x34)); move8(mcu.r[4], v); }
    mcu.pc = 0x36f2;
}

/* 0x36f2 movlw r4 @(br,$3a) */
void step_gate_setup_movlw_r4_at_br_3a(void)
{
    load16(mcu.r[4], br_addr(0x3a));
    mcu.pc = 0x36f4;
}

/* 0x36f4 MOVG3 r4 -> @r0+-100 */
void step_gate_setup_movg3_r4_to_at_r0_minus_100(void)
{
    store16(ind_addr(0, (uint16_t)-100), mcu.r[4]);
    mcu.pc = 0x36f7;
}

/* 0x36f7 MOVG3 r4 -> @r0+-98 */
void step_gate_setup_movg3_r4_to_at_r0_minus_98(void)
{
    store16(ind_addr(0, (uint16_t)-98), mcu.r[4]);
    mcu.pc = 0x36fa;
}

/* 0x36fa bsr 13 -> 0x3709 */
void step_gate_setup_bsr_13_to_0x3709(void)
{
    MCU_PushStack(0x36fc);
    mcu.pc = 0x3709;
}

/* 0x36fc BCLR_ANDC #0xf8ff r0 */
void step_gate_setup_bclr_andc_0xf8ff_r0(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x3700;
}

/* 0x3700 MOVG2 (dp,0xad2c) r6 */
void step_gate_setup_movg2_dp_0xad2c_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, dp_addr(0xad2c));
    mcu.pc = 0x3704;
}

/* 0x3704 MOVG3 r6 -> (dp,0xad2a) */
void step_gate_setup_movg3_r6_to_dp_0xad2a(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xad2a), r6);
    mcu.pc = 0x3708;
}

/* 0x3708 rts */
void step_gate_setup_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- S7: mask_set 0x546e-0x5491 + shared tail 0x5492-0x54cb ------------- */

/* mask_set is IML=0; stock is interrupted before 0x5474 and before the 0x5491
 * rts, so both ranges are per-PC entries (each returns 1). The tail at 0x5492
 * is shared with flush_off's pending-command exit: it pops the caller's return
 * address (ADDQ #2 r7) and re-joins the pcm_play epilogue at 0x51fc. */


/* 0x546e MOVG2 @r0+-2 r1 */
void step_mask_set_movg2_at_r0_minus_2_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x5471;
}

/* 0x5471 cmp r1,w #0x000f */
void step_mask_set_cmp_r1_w_0xf(void)
{
    uint16_t &r1 = mcu.r[1];

    cmp16(r1, 0x000f);
    mcu.pc = 0x5474;
}

/* 0x5474 BLS 10 -> 0x5480 */
void step_mask_set_bls_10_to_0x5480(void)
{
    mcu.pc = ((mcu.sr & (STATUS_C | STATUS_Z)) != 0) ? 0x5480 : 0x5476;
}

/* 0x5476 SUB #0x0010 r1 */
void step_mask_set_sub_0x10_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    sub_imm16(r1, 0x0010);
    mcu.pc = 0x547a;
}

/* 0x547a BSET (dp,0xd154) r1 */
void step_mask_set_bset_dp_0xd154_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    bset16_mem(dp_addr(0xd154), r1);
    mcu.pc = 0x547e;
}

/* 0x547e BRA 4 -> 0x5484 */
void step_mask_set_bra_4_to_0x5484(void)
{
    mcu.pc = 0x5484;
}

/* 0x5480 BSET (dp,0xd156) r1 */
void step_mask_set_bset_dp_0xd156_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    bset16_mem(dp_addr(0xd156), r1);
    mcu.pc = 0x5484;
}

/* 0x5484 MOVG2 @r0+-2 r1 */
void step_mask_set_movg2_at_r0_minus_2_r1_a(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x5487;
}

/* 0x5487 SUB @r1+0xd0e0 #0x00 */
void step_mask_set_sub_at_r1_plus_0xd0e0_0x0(void)
{
    sub_mem_imm8(ind_addr(1, 0xd0e0), 0x00);
    mcu.pc = 0x548c;
}

/* 0x548c BNE 4 -> 0x5492 */
void step_mask_set_bne_4_to_0x5492(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x548e : 0x5492;
}

/* 0x548e BRA 0xffffd29d -> 0x272e */
void step_mask_set_bra_0xffffd29d_to_0x272e(void)
{
    mcu.pc = 0x272e;
}

/* 0x5491 rts */
void step_mask_set_rts(void)
{
    mcu.pc = MCU_PopStack();
}


/* 0x5492 MOVG2 @r0+-2 r1 */
void step_mask_tail_movg2_at_r0_minus_2_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x5495;
}

/* 0x5495 ADDQ #2 r7 */
void step_mask_tail_addq_2_r7(void)
{
    addq_r7(2);
    mcu.pc = 0x5497;
}

/* 0x5497 MOVG2 (dp,0xd158) r0 */
void step_mask_tail_movg2_dp_0xd158_r0(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x549b;
}

/* 0x549b SUB @r0+0 #0x0018 */
void step_mask_tail_sub_at_r0_plus_0_0x18(void)
{
    sub_mem_imm16(ind_addr(0, 0), 0x0018);
    mcu.pc = 0x54a0;
}

/* 0x54a0 BNE 8 -> 0x54aa */
void step_mask_tail_bne_8_to_0x54aa(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x54a2 : 0x54aa;
}

/* 0x54a2 MOVG2 @r0+6 r6 */
void step_mask_tail_movg2_at_r0_plus_6_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, 6));
    mcu.pc = 0x54a5;
}

/* 0x54a5 MOVG3 r6 -> @r0+0 */
void step_mask_tail_movg3_r6_to_at_r0_plus_0(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, 0), r6);
    mcu.pc = 0x54a8;
}

/* 0x54a8 BRA 4 -> 0x54ae */
void step_mask_tail_bra_4_to_0x54ae(void)
{
    mcu.pc = 0x54ae;
}

/* 0x54aa CLR @r1+0xad0e */
void step_mask_tail_clr_at_r1_plus_0xad0e(void)
{
    clr8_mem(ind_addr(1, 0xad0e));
    mcu.pc = 0x54ae;
}

/* 0x54ae MOVG2 (dp,0xd15a) r0 */
void step_mask_tail_movg2_dp_0xd15a_r0(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd15a));
    mcu.pc = 0x54b2;
}

/* 0x54b2 BMI 0xfffffd47 -> 0x51fc */
void step_mask_tail_bmi_0xfffffd47_to_0x51fc(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x51fc : 0x54b5;
}

/* 0x54b5 SUB @r0+0 #0x0018 */
void step_mask_tail_sub_at_r0_plus_0_0x18_a(void)
{
    sub_mem_imm16(ind_addr(0, 0), 0x0018);
    mcu.pc = 0x54ba;
}

/* 0x54ba BNE 9 -> 0x54c5 */
void step_mask_tail_bne_9_to_0x54c5(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x54bc : 0x54c5;
}

/* 0x54bc MOVG2 @r0+6 r6 */
void step_mask_tail_movg2_at_r0_plus_6_r6_a(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, 6));
    mcu.pc = 0x54bf;
}

/* 0x54bf MOVG3 r6 -> @r0+0 */
void step_mask_tail_movg3_r6_to_at_r0_plus_0_a(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, 0), r6);
    mcu.pc = 0x54c2;
}

/* 0x54c2 BRA 0xfffffd37 -> 0x51fc */
void step_mask_tail_bra_0xfffffd37_to_0x51fc(void)
{
    mcu.pc = 0x51fc;
}

/* 0x54c5 CLR @r1+0xad0e */
void step_mask_tail_clr_at_r1_plus_0xad0e_a(void)
{
    clr8_mem(ind_addr(1, 0xad0e));
    mcu.pc = 0x54c9;
}

/* 0x54c9 BRA 0xfffffd30 -> 0x51fc */
void step_mask_tail_bra_0xfffffd30_to_0x51fc(void)
{
    mcu.pc = 0x51fc;
}

/* flush_off 0x54fb-0x5521 (IML=7): per-PC. Clear the pending mask bits from
 * the committed mask, then hand off to the existing per-PC PCM flush handlers
 * (0x5525/27/29 in pcm_enable.cpp) and the 0x552b rts. The pending-command
 * exit (0x552c, unobserved in the captured runs) is left interpreted; like
 * the rest of S1-S10 this is one H8 instruction per step so -midiseq posts
 * and SM_Update keep stock cadence (round-6 L1 finding). */

/* 0x54fb MOVG2 @r0+-2 r1 */
void step_flush_off_movg2_at_r0_minus_2_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x54fe;
}

/* 0x54fe SUB @r1+0xd0e0 #0x00 */
void step_flush_off_sub_at_r1_plus_0xd0e0_0x0(void)
{
    sub_mem_imm8(ind_addr(1, 0xd0e0), 0x00);
    mcu.pc = 0x5503;
}

/* 0x5503 BNE 39 -> 0x552c */
void step_flush_off_bne_39_to_0x552c(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x5505 : 0x552c;
}

/* 0x5505 MOVG2 (dp,0xd154) r5 */
void step_flush_off_movg2_dp_0xd154_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load16(r5, dp_addr(0xd154));
    mcu.pc = 0x5509;
}

/* 0x5509 MOVG2 (dp,0xd156) r6 */
void step_flush_off_movg2_dp_0xd156_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, dp_addr(0xd156));
    mcu.pc = 0x550d;
}

/* 0x550d NOT r5 */
void step_flush_off_not_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    not16(r5);
    mcu.pc = 0x550f;
}

/* 0x550f NOT r6 */
void step_flush_off_not_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    not16(r6);
    mcu.pc = 0x5511;
}

/* 0x5511 MOVG2 (dp,0xd150) r3 */
void step_flush_off_movg2_dp_0xd150_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load16(r3, dp_addr(0xd150));
    mcu.pc = 0x5515;
}

/* 0x5515 MOVG2 (dp,0xd152) r4 */
void step_flush_off_movg2_dp_0xd152_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load16(r4, dp_addr(0xd152));
    mcu.pc = 0x5519;
}

/* 0x5519 AND r5 r3 */
void step_flush_off_and_r5_r3(void)
{
    uint16_t &r3 = mcu.r[3];
    uint16_t &r5 = mcu.r[5];

    and16(r3, r5);
    mcu.pc = 0x551b;
}

/* 0x551b AND r6 r4 */
void step_flush_off_and_r6_r4(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r6 = mcu.r[6];

    and16(r4, r6);
    mcu.pc = 0x551d;
}

/* 0x551d MOVG3 r3 -> (dp,0xd150) */
void step_flush_off_movg3_r3_to_dp_0xd150(void)
{
    uint16_t &r3 = mcu.r[3];

    store16(dp_addr(0xd150), r3);
    mcu.pc = 0x5521;
}

/* 0x5521 MOVG3 r4 -> (dp,0xd152) */
void step_flush_off_movg3_r4_to_dp_0xd152(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(dp_addr(0xd152), r4);
    mcu.pc = 0x5525;
}

/* flush_on 0x564a-0x565e -> 0x5662 (IML=7), per-PC. */

/* 0x564a MOVG2 (dp,0xd150) r5 */
void step_flush_on_a_movg2_dp_0xd150_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load16(r5, dp_addr(0xd150));
    mcu.pc = 0x564e;
}

/* 0x564e MOVG2 (dp,0xd152) r6 */
void step_flush_on_a_movg2_dp_0xd152_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, dp_addr(0xd152));
    mcu.pc = 0x5652;
}

/* 0x5652 OR (dp,0xd154) r5 */
void step_flush_on_a_or_dp_0xd154_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    or16_mem(r5, dp_addr(0xd154));
    mcu.pc = 0x5656;
}

/* 0x5656 OR (dp,0xd156) r6 */
void step_flush_on_a_or_dp_0xd156_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    or16_mem(r6, dp_addr(0xd156));
    mcu.pc = 0x565a;
}

/* 0x565a MOVG3 r5 -> (dp,0xd150) */
void step_flush_on_a_movg3_r5_to_dp_0xd150(void)
{
    uint16_t &r5 = mcu.r[5];

    store16(dp_addr(0xd150), r5);
    mcu.pc = 0x565e;
}

/* 0x565e MOVG3 r6 -> (dp,0xd152) */
void step_flush_on_a_movg3_r6_to_dp_0xd152(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xd152), r6);
    mcu.pc = 0x5662;
}

/* flush_on 0x5668-0x5670 (after the 0x5662/64/66 PCM handlers), per-PC. */

/* 0x5668 CLR (dp,0xd154) */
void step_flush_on_b_clr_dp_0xd154(void)
{
    clr16_mem(dp_addr(0xd154));
    mcu.pc = 0x566c;
}

/* 0x566c CLR (dp,0xd156) */
void step_flush_on_b_clr_dp_0xd156(void)
{
    clr16_mem(dp_addr(0xd156));
    mcu.pc = 0x5670;
}

/* 0x5670 rts */
void step_flush_on_b_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- S8: param_write 0x5533-0x5625 (per instruction) --------------------- */

/* IML=7 (called between BSET_ORC 0x535e/0x53c7 and the exit BCLR_ANDC), no
 * observed internal interrupt. All PCM register traffic goes through
 * MCU_Write/Read (device routing + -pcmtrace). dp is cleared by each LDC as
 * in the ROM. Per-PC from round 6: the [229114164,229115004) span covered the
 * -midiseq byte due at 229114868, whose deferred post let the SM consume it
 * early (same class as the c212.565M materialize finding). */

/* 0x5533 MOVG2 @r0+-2 r3 */
void step_param_write_movg2_at_r0_minus_2_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load16(r3, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x5536;
}

/* 0x5536 CLR @r3+0xce3f */
void step_param_write_clr_at_r3_plus_0xce3f(void)
{
    clr8_mem(ind_addr(3, 0xce3f));
    mcu.pc = 0x553a;
}

/* 0x553a CLR @r3+0xd15c */
void step_param_write_clr_at_r3_plus_0xd15c(void)
{
    clr8_mem(ind_addr(3, 0xd15c));
    mcu.pc = 0x553e;
}

/* 0x553e movs r3 @(br,$3e) */
void step_param_write_movs_r3_at_br_3e(void)
{
    uint16_t &r3 = mcu.r[3];

    movs8(r3, 0x3e);
    mcu.pc = 0x5540;
}

/* 0x5540 MOVG2 @r0+-10 r6 */
void step_param_write_movg2_at_r0_minus_10_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-10));
    mcu.pc = 0x5543;
}

/* 0x5543 movsw r6 @(br,$1e) */
void step_param_write_movsw_r6_at_br_1e(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x1e);
    mcu.pc = 0x5545;
}

/* 0x5545 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5(void)
{
    ldc_dp(0);
    mcu.pc = 0x5548;
}

/* 0x5548 MOVG3 r3 -> (dp,0xfe6c) */
void step_param_write_movg3_r3_to_dp_0xfe6c(void)
{
    uint16_t &r3 = mcu.r[3];

    movg_imm8(dp_addr(0xfe6c), (uint8_t)r3);
    mcu.pc = 0x554c;
}

/* 0x554c MOVG3 r3 -> (dp,0xfe5e) */
void step_param_write_movg3_r3_to_dp_0xfe5e(void)
{
    uint16_t &r3 = mcu.r[3];

    store16(dp_addr(0xfe5e), r3);
    mcu.pc = 0x5550;
}

/* 0x5550 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_a(void)
{
    ldc_dp(0);
    mcu.pc = 0x5553;
}

/* 0x5553 MOVG2 @r0+-20 r5 */
void step_param_write_movg2_at_r0_minus_20_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load8(r5, ind_addr(0, (uint16_t)-20));
    mcu.pc = 0x5556;
}

/* 0x5556 MOVG2 @r0+-16 r6 */
void step_param_write_movg2_at_r0_minus_16_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-16));
    mcu.pc = 0x5559;
}

/* 0x5559 movs r5 @(br,$05) */
void step_param_write_movs_r5_at_br_05(void)
{
    uint16_t &r5 = mcu.r[5];

    movs8(r5, 0x05);
    mcu.pc = 0x555b;
}

/* 0x555b movsw r6 @(br,$06) */
void step_param_write_movsw_r6_at_br_06(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x06);
    mcu.pc = 0x555d;
}

/* 0x555d LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_b(void)
{
    ldc_dp(0);
    mcu.pc = 0x5560;
}

/* 0x5560 MOVG3 r5 -> (dp,0xfe6d) */
void step_param_write_movg3_r5_to_dp_0xfe6d(void)
{
    uint16_t &r5 = mcu.r[5];

    movg_imm8(dp_addr(0xfe6d), (uint8_t)r5);
    mcu.pc = 0x5564;
}

/* 0x5564 MOVG3 r6 -> (dp,0xfe60) */
void step_param_write_movg3_r6_to_dp_0xfe60(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xfe60), r6);
    mcu.pc = 0x5568;
}

/* 0x5568 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_c(void)
{
    ldc_dp(0);
    mcu.pc = 0x556b;
}

/* 0x556b MOVG2 @r0+-18 r5 */
void step_param_write_movg2_at_r0_minus_18_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load8(r5, ind_addr(0, (uint16_t)-18));
    mcu.pc = 0x556e;
}

/* 0x556e MOVG2 @r0+-12 r6 */
void step_param_write_movg2_at_r0_minus_12_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-12));
    mcu.pc = 0x5571;
}

/* 0x5571 movs r5 @(br,$09) */
void step_param_write_movs_r5_at_br_09(void)
{
    uint16_t &r5 = mcu.r[5];

    movs8(r5, 0x09);
    mcu.pc = 0x5573;
}

/* 0x5573 movsw r6 @(br,$0a) */
void step_param_write_movsw_r6_at_br_0a(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x0a);
    mcu.pc = 0x5575;
}

/* 0x5575 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_d(void)
{
    ldc_dp(0);
    mcu.pc = 0x5578;
}

/* 0x5578 MOVG3 r5 -> (dp,0xfe6e) */
void step_param_write_movg3_r5_to_dp_0xfe6e(void)
{
    uint16_t &r5 = mcu.r[5];

    movg_imm8(dp_addr(0xfe6e), (uint8_t)r5);
    mcu.pc = 0x557c;
}

/* 0x557c MOVG3 r6 -> (dp,0xfe62) */
void step_param_write_movg3_r6_to_dp_0xfe62(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xfe62), r6);
    mcu.pc = 0x5580;
}

/* 0x5580 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_e(void)
{
    ldc_dp(0);
    mcu.pc = 0x5583;
}

/* 0x5583 MOVG2 @r0+-19 r5 */
void step_param_write_movg2_at_r0_minus_19_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load8(r5, ind_addr(0, (uint16_t)-19));
    mcu.pc = 0x5586;
}

/* 0x5586 MOVG2 @r0+-14 r6 */
void step_param_write_movg2_at_r0_minus_14_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-14));
    mcu.pc = 0x5589;
}

/* 0x5589 movs r5 @(br,$0d) */
void step_param_write_movs_r5_at_br_0d(void)
{
    uint16_t &r5 = mcu.r[5];

    movs8(r5, 0x0d);
    mcu.pc = 0x558b;
}

/* 0x558b movsw r6 @(br,$0e) */
void step_param_write_movsw_r6_at_br_0e(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x0e);
    mcu.pc = 0x558d;
}

/* 0x558d LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_f(void)
{
    ldc_dp(0);
    mcu.pc = 0x5590;
}

/* 0x5590 MOVG3 r5 -> (dp,0xfe6f) */
void step_param_write_movg3_r5_to_dp_0xfe6f(void)
{
    uint16_t &r5 = mcu.r[5];

    movg_imm8(dp_addr(0xfe6f), (uint8_t)r5);
    mcu.pc = 0x5594;
}

/* 0x5594 MOVG3 r6 -> (dp,0xfe64) */
void step_param_write_movg3_r6_to_dp_0xfe64(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xfe64), r6);
    mcu.pc = 0x5598;
}

/* 0x5598 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_g(void)
{
    ldc_dp(0);
    mcu.pc = 0x559b;
}

/* 0x559b MOVG2 @r0+52 r6 */
void step_param_write_movg2_at_r0_plus_52_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, 52));
    mcu.pc = 0x559e;
}

/* 0x559e movsw r6 @(br,$12) */
void step_param_write_movsw_r6_at_br_12(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x12);
    mcu.pc = 0x55a0;
}

/* 0x55a0 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_h(void)
{
    ldc_dp(0);
    mcu.pc = 0x55a3;
}

/* 0x55a3 MOVG3 r6 -> (dp,0xfe66) */
void step_param_write_movg3_r6_to_dp_0xfe66(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xfe66), r6);
    mcu.pc = 0x55a7;
}

/* 0x55a7 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_i(void)
{
    ldc_dp(0);
    mcu.pc = 0x55aa;
}

/* 0x55aa MOVG2 @r0+58 r6 */
void step_param_write_movg2_at_r0_plus_58_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, 58));
    mcu.pc = 0x55ad;
}

/* 0x55ad movsw r6 @(br,$14) */
void step_param_write_movsw_r6_at_br_14(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x14);
    mcu.pc = 0x55af;
}

/* 0x55af LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_j(void)
{
    ldc_dp(0);
    mcu.pc = 0x55b2;
}

/* 0x55b2 MOVG3 r6 -> (dp,0xfe68) */
void step_param_write_movg3_r6_to_dp_0xfe68(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xfe68), r6);
    mcu.pc = 0x55b6;
}

/* 0x55b6 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_k(void)
{
    ldc_dp(0);
    mcu.pc = 0x55b9;
}

/* 0x55b9 MOVG2 @r0+104 r6 */
void step_param_write_movg2_at_r0_plus_104_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(0, 104));
    mcu.pc = 0x55bc;
}

/* 0x55bc SWAP r6 */
void step_param_write_swap_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    swap16(r6);
    mcu.pc = 0x55be;
}

/* 0x55be MOVG2 @r0+102 r6 */
void step_param_write_movg2_at_r0_plus_102_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(0, 102));
    mcu.pc = 0x55c1;
}

/* 0x55c1 movsw r6 @(br,$1c) */
void step_param_write_movsw_r6_at_br_1c(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x1c);
    mcu.pc = 0x55c3;
}

/* 0x55c3 MOVG2 @r0+-24 r6 */
void step_param_write_movg2_at_r0_minus_24_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, (uint16_t)-24));
    mcu.pc = 0x55c6;
}

/* 0x55c6 movsw r6 @(br,$1a) */
void step_param_write_movsw_r6_at_br_1a(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x1a);
    mcu.pc = 0x55c8;
}

/* 0x55c8 MOVG2 @r0+26 r6 */
void step_param_write_movg2_at_r0_plus_26_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, 26));
    mcu.pc = 0x55cb;
}

/* 0x55cb movsw r6 @(br,$16) */
void step_param_write_movsw_r6_at_br_16(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x16);
    mcu.pc = 0x55cd;
}

/* 0x55cd BTSTI @r0+-59 #7 */
void step_param_write_btsti_at_r0_minus_59_7(void)
{
    btsti8_mem(ind_addr(0, (uint16_t)-59), 7);
    mcu.pc = 0x55d0;
}

/* 0x55d0 BEQ 54 -> 0x5608 */
void step_param_write_beq_54_to_0x5608(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x5608 : 0x55d2;
}

/* 0x55d2 TST @r0+14 */
void step_param_write_tst_at_r0_plus_14(void)
{
    tst16_mem(ind_addr(0, 14));
    mcu.pc = 0x55d5;
}

/* 0x55d5 BEQ 11 -> 0x55e2 */
void step_param_write_beq_11_to_0x55e2(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x55e2 : 0x55d7;
}

/* 0x55d7 movi r4 #0x0002 */
void step_param_write_movi_r4_0x2(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0002);
    mcu.pc = 0x55da;
}

/* 0x55da MOVG2 @r0+30 r5 */
void step_param_write_movg2_at_r0_plus_30_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load16(r5, ind_addr(0, 30));
    mcu.pc = 0x55dd;
}

/* 0x55dd MOVG2 @r0+72 r6 */
void step_param_write_movg2_at_r0_plus_72_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, 72));
    mcu.pc = 0x55e0;
}

/* 0x55e0 BRA 11 -> 0x55ed */
void step_param_write_bra_11_to_0x55ed(void)
{
    mcu.pc = 0x55ed;
}

/* 0x55e2 movi r4 #0x0000 */
void step_param_write_movi_r4_0x0(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0000);
    mcu.pc = 0x55e5;
}

/* 0x55e5 movi r5 #0x00b5 */
void step_param_write_movi_r5_0xb5(void)
{
    uint16_t &r5 = mcu.r[5];

    movi16(r5, 0x00b5);
    mcu.pc = 0x55e8;
}

/* 0x55e8 CLR r6 */
void step_param_write_clr_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    clr16(r6);
    mcu.pc = 0x55ea;
}

/* 0x55ea CLR @r0+8 */
void step_param_write_clr_at_r0_plus_8(void)
{
    clr16_mem(ind_addr(0, 8));
    mcu.pc = 0x55ed;
}

/* 0x55ed MOVG3 r4 -> @r0+0 */
void step_param_write_movg3_r4_to_at_r0_plus_0(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0), r4);
    mcu.pc = 0x55f0;
}

/* 0x55f0 MOVG3 r4 -> @r0+2 */
void step_param_write_movg3_r4_to_at_r0_plus_2(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 2), r4);
    mcu.pc = 0x55f3;
}

/* 0x55f3 MOVG3 r4 -> @r0+4 */
void step_param_write_movg3_r4_to_at_r0_plus_4(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 4), r4);
    mcu.pc = 0x55f6;
}

/* 0x55f6 movsw r5 @(br,$18) */
void step_param_write_movsw_r5_at_br_18(void)
{
    uint16_t &r5 = mcu.r[5];

    movs16(r5, 0x18);
    mcu.pc = 0x55f8;
}

/* 0x55f8 MOVG3 r5 -> @r0+30 */
void step_param_write_movg3_r5_to_at_r0_plus_30(void)
{
    uint16_t &r5 = mcu.r[5];

    store16(ind_addr(0, 30), r5);
    mcu.pc = 0x55fb;
}

/* 0x55fb movsw r6 @(br,$10) */
void step_param_write_movsw_r6_at_br_10(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x10);
    mcu.pc = 0x55fd;
}

/* 0x55fd LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_l(void)
{
    ldc_dp(0);
    mcu.pc = 0x5600;
}

/* 0x5600 MOVG3 r6 -> (dp,0xfe6a) */
void step_param_write_movg3_r6_to_dp_0xfe6a(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xfe6a), r6);
    mcu.pc = 0x5604;
}

/* 0x5604 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_m(void)
{
    ldc_dp(0);
    mcu.pc = 0x5607;
}

/* 0x5607 rts */
void step_param_write_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x5608 MOVG2 @r0+30 r5 */
void step_param_write_movg2_at_r0_plus_30_r5_a(void)
{
    uint16_t &r5 = mcu.r[5];

    load16(r5, ind_addr(0, 30));
    mcu.pc = 0x560b;
}

/* 0x560b MOVG2 @r0+72 r6 */
void step_param_write_movg2_at_r0_plus_72_r6_a(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, 72));
    mcu.pc = 0x560e;
}

/* 0x560e movsw r5 @(br,$18) */
void step_param_write_movsw_r5_at_br_18_a(void)
{
    uint16_t &r5 = mcu.r[5];

    movs16(r5, 0x18);
    mcu.pc = 0x5610;
}

/* 0x5610 movsw r6 @(br,$10) */
void step_param_write_movsw_r6_at_br_10_a(void)
{
    uint16_t &r6 = mcu.r[6];

    movs16(r6, 0x10);
    mcu.pc = 0x5612;
}

/* 0x5612 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_n(void)
{
    ldc_dp(0);
    mcu.pc = 0x5615;
}

/* 0x5615 MOVG3 r6 -> (dp,0xfe6a) */
void step_param_write_movg3_r6_to_dp_0xfe6a_a(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(dp_addr(0xfe6a), r6);
    mcu.pc = 0x5619;
}

/* 0x5619 LDC #0x00 r5 */
void step_param_write_ldc_0x0_r5_o(void)
{
    ldc_dp(0);
    mcu.pc = 0x561c;
}

/* 0x561c MOVG2 @r0+6 r6 */
void step_param_write_movg2_at_r0_plus_6_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load16(r6, ind_addr(0, 6));
    mcu.pc = 0x561f;
}

/* 0x561f MOVG3 r6 -> @r0+0 */
void step_param_write_movg3_r6_to_at_r0_plus_0(void)
{
    uint16_t &r6 = mcu.r[6];

    store16(ind_addr(0, 0), r6);
    mcu.pc = 0x5622;
}

/* 0x5622 CLR @r0+6 */
void step_param_write_clr_at_r0_plus_6(void)
{
    clr16_mem(ind_addr(0, 6));
    mcu.pc = 0x5625;
}

/* 0x5625 rts */
void step_param_write_rts_a(void)
{
    mcu.pc = MCU_PopStack();
}

/* loop_write 0x5626-0x5649 (IML=7): waits for PCM status 0x1e bit5 before
 * programming the loop registers. The busy-wait needs one host iteration per
 * instruction (PCM_Update/SM_Update run between steps), so it must stay
 * per-PC: a whole-routine block would spin without the device ever updating. */

/* 0x5626 MOVG2 @r0+-2 r1 */
void step_loop_write_movg2_at_r0_minus_2_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x5629;
}

/* 0x5629 TST @r0+101 */
void step_loop_write_tst_at_r0_plus_101(void)
{
    tst8_mem(ind_addr(0, 101));
    mcu.pc = 0x562c;
}

/* 0x562c BNE 27 -> 0x5649 */
void step_loop_write_bne_27_to_0x5649(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x562e : 0x5649;
}

/* 0x562e movs r1 @(br,$3e) */
void step_loop_write_movs_r1_at_br_3e(void)
{
    uint16_t &r1 = mcu.r[1];

    movs8(r1, 0x3e);
    mcu.pc = 0x5630;
}

/* 0x5630 movl r5 @(br,$1e) */
void step_loop_write_movl_r5_at_br_1e(void)
{
    uint16_t &r5 = mcu.r[5];

    movl8(r5, 0x1e);
    mcu.pc = 0x5632;
}

/* 0x5632 movlw r5 @(br,$3a) */
void step_loop_write_movlw_r5_at_br_3a(void)
{
    uint16_t &r5 = mcu.r[5];

    movl16(r5, 0x3a);
    mcu.pc = 0x5634;
}

/* 0x5634 BTSTI r5 #5 */
void step_loop_write_btsti_r5_5(void)
{
    uint16_t &r5 = mcu.r[5];

    btsti16_reg(r5, 5);
    mcu.pc = 0x5636;
}

/* 0x5636 BEQ -8 -> 0x5630 */
void step_loop_write_beq_minus_8_to_0x5630(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x5630 : 0x5638;
}

/* 0x5638 MOVG #0xff00 -> (br,$1a) */
void step_loop_write_movg_0xff00_to_br_1a(void)
{
    movg_imm16(br_addr(0x1a), 0xff00);
    mcu.pc = 0x563d;
}

/* 0x563d MOVG2 @r0+-22 r5 */
void step_loop_write_movg2_at_r0_minus_22_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load16(r5, ind_addr(0, (uint16_t)-22));
    mcu.pc = 0x5640;
}

/* 0x5640 SHLR r5 */
void step_loop_write_shlr_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    shlr16(r5);
    mcu.pc = 0x5642;
}

/* 0x5642 movsw r5 @(br,$36) */
void step_loop_write_movsw_r5_at_br_36(void)
{
    uint16_t &r5 = mcu.r[5];

    movs16(r5, 0x36);
    mcu.pc = 0x5644;
}

/* 0x5644 MOVG2 @r0+38 r5 */
void step_loop_write_movg2_at_r0_plus_38_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load16(r5, ind_addr(0, 38));
    mcu.pc = 0x5647;
}

/* 0x5647 movsw r5 @(br,$1a) */
void step_loop_write_movsw_r5_at_br_1a(void)
{
    uint16_t &r5 = mcu.r[5];

    movs16(r5, 0x1a);
    mcu.pc = 0x5649;
}

/* 0x5649 rts */
void step_loop_write_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- S9: param_ack 0x54cc-0x54fa ---------------------------------------- */
/* IML=0 after the first ORC/ANDC pair and carries the trapa #0x10 handshake
 * loop; per-PC so the trap is taken by the host exactly where stock takes it
 * and the PCM latches are read one instruction at a time. */

/* 0x54cc BSET_ORC #0x0700 r0 */
void step_param_ack_bset_orc_0x700_r0(void)
{
    mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x54d0;
}

/* 0x54d0 MOVG2 @r1+-2 r2 */
void step_param_ack_movg2_at_r1_minus_2_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(1, (uint16_t)-2));
    mcu.pc = 0x54d3;
}

/* 0x54d3 movs r2 @(br,$3e) */
void step_param_ack_movs_r2_at_br_3e(void)
{
    uint16_t &r2 = mcu.r[2];

    movs8(r2, 0x3e);
    mcu.pc = 0x54d5;
}

/* 0x54d5 movl r5 @(br,$32) */
void step_param_ack_movl_r5_at_br_32(void)
{
    uint16_t &r5 = mcu.r[5];

    movl8(r5, 0x32);
    mcu.pc = 0x54d7;
}

/* 0x54d7 movlw r5 @(br,$3a) */
void step_param_ack_movlw_r5_at_br_3a(void)
{
    uint16_t &r5 = mcu.r[5];

    movl16(r5, 0x3a);
    mcu.pc = 0x54d9;
}

/* 0x54d9 movl r6 @(br,$34) */
void step_param_ack_movl_r6_at_br_34(void)
{
    uint16_t &r6 = mcu.r[6];

    movl8(r6, 0x34);
    mcu.pc = 0x54db;
}

/* 0x54db movlw r6 @(br,$3a) */
void step_param_ack_movlw_r6_at_br_3a(void)
{
    uint16_t &r6 = mcu.r[6];

    movl16(r6, 0x3a);
    mcu.pc = 0x54dd;
}

/* 0x54dd BCLR_ANDC #0xf8ff r0 */
void step_param_ack_bclr_andc_0xf8ff_r0(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x54e1;
}

/* 0x54e1 SUB @r2+0xd0e0 #0x00 */
void step_param_ack_sub_at_r2_plus_0xd0e0_0x0(void)
{
    sub_mem_imm8(ind_addr(2, 0xd0e0), 0x00);
    mcu.pc = 0x54e6;
}

/* 0x54e6 BEQ 4 -> 0x54ec */
void step_param_ack_beq_4_to_0x54ec(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x54ec : 0x54e8;
}

/* 0x54e8 MOVG2 r1 r0 */
void step_param_ack_movg2_r1_r0(void)
{
    uint16_t &r0 = mcu.r[0];

    move16(r0, mcu.r[1]);
    mcu.pc = 0x54ea;
}

/* 0x54ea BRA -90 -> 0x5492 */
void step_param_ack_bra_minus_90_to_0x5492(void)
{
    mcu.pc = 0x5492;
}

/* 0x54ec TST r5 */
void step_param_ack_tst_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    tst16_reg(r5);
    mcu.pc = 0x54ee;
}

/* 0x54ee BEQ 10 -> 0x54fa */
void step_param_ack_beq_10_to_0x54fa(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x54fa : 0x54f0;
}

/* 0x54f0 TST r6 */
void step_param_ack_tst_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    tst16_reg(r6);
    mcu.pc = 0x54f2;
}

/* 0x54f2 BEQ 6 -> 0x54fa */
void step_param_ack_beq_6_to_0x54fa(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x54fa : 0x54f4;
}

/* 0x54f4 move r0 #0x80 */
void step_param_ack_move_r0_0x80(void)
{
    uint16_t &r0 = mcu.r[0];

    move8(r0, 0x80);
    mcu.pc = 0x54f6;
}

/* 0x54f6 trapa #0x10 */
void step_param_ack_trapa_0x10(void)
{
    MCU_Interrupt_TRAPA(0x10u & 0x0fu);
    mcu.pc = 0x54f8;
}

/* 0x54f8 BRA -46 -> 0x54cc */
void step_param_ack_bra_minus_46_to_0x54cc(void)
{
    mcu.pc = 0x54cc;
}

/* 0x54fa rts */
void step_param_ack_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* ---- S10: pcm_play 0x52cb-0x53e8 ----------------------------------------- */

/* L0 per-PC: the routine orchestrates the already-hand child routines, so each
 * instruction runs as its own step and the host keeps dispatching (bsr targets
 * are the registered child entries; the two BRA 0x51fc exits do not pop the
 * stack). This keeps the IML=7 / IML=0 windows and the per-instruction device
 * updates exactly where stock has them. */


/* 0x52cb SUB @r1+0xd0a8 #0xff */
void step_pcm_play_sub_at_r1_plus_0xd0a8_0xff(void)
{
    sub_mem_imm8(ind_addr(1, 0xd0a8), 0xff);
    mcu.pc = 0x52d0;
}

/* 0x52d0 BNE 21 -> 0x52e7 */
void step_pcm_play_bne_21_to_0x52e7(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x52d2 : 0x52e7;
}

/* 0x52d2 SUB @r1+0xd0c4 #0xff */
void step_pcm_play_sub_at_r1_plus_0xd0c4_0xff(void)
{
    sub_mem_imm8(ind_addr(1, 0xd0c4), 0xff);
    mcu.pc = 0x52d7;
}

/* 0x52d7 BEQ 0x00b6 -> 0x5390 */
void step_pcm_play_beq_0xb6_to_0x5390(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x5390 : 0x52da;
}

/* 0x52da CLR r2 */
void step_pcm_play_clr_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    clr16(r2);
    mcu.pc = 0x52dc;
}

/* 0x52dc MOVG2 @r1+0xd0c4 r2 */
void step_pcm_play_movg2_at_r1_plus_0xd0c4_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(1, 0xd0c4));
    mcu.pc = 0x52e0;
}

/* 0x52e0 MOVG #0x00 -> @r2+0xd0e0 */
void step_pcm_play_movg_0x0_to_at_r2_plus_0xd0e0(void)
{
    movg_imm8(ind_addr(2, 0xd0e0), 0x00);
    mcu.pc = 0x52e5;
}

/* 0x52e5 BRA 13 -> 0x52f4 */
void step_pcm_play_bra_13_to_0x52f4(void)
{
    mcu.pc = 0x52f4;
}

/* 0x52e7 CLR r2 */
void step_pcm_play_clr_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    clr16(r2);
    mcu.pc = 0x52e9;
}

/* 0x52e9 MOVG2 @r1+0xd0a8 r2 */
void step_pcm_play_movg2_at_r1_plus_0xd0a8_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load8(r2, ind_addr(1, 0xd0a8));
    mcu.pc = 0x52ed;
}

/* 0x52ed MOVG #0x00 -> @r2+0xd0e0 */
void step_pcm_play_movg_0x0_to_at_r2_plus_0xd0e0_a(void)
{
    movg_imm8(ind_addr(2, 0xd0e0), 0x00);
    mcu.pc = 0x52f2;
}

/* 0x52f2 XCH r2 r1 */
void step_pcm_play_xch_r2_r1(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];

    { uint16_t t = r1; r1 = r2; r2 = t; }
    mcu.pc = 0x52f4;
}

/* 0x52f4 bsr16 -> 0x53eb */
void step_pcm_play_bsr16_to_0x53eb(void)
{
    MCU_PushStack(0x52f7);
    mcu.pc = 0x53eb;
}

/* 0x52f7 MOVG3 r0 -> (dp,0xd158) */
void step_pcm_play_movg3_r0_to_dp_0xd158(void)
{
    uint16_t &r0 = mcu.r[0];

    store16(dp_addr(0xd158), r0);
    mcu.pc = 0x52fb;
}

/* 0x52fb MOVG2 r2 r1 */
void step_pcm_play_movg2_r2_r1(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];

    move16(r1, r2);
    mcu.pc = 0x52fd;
}

/* 0x52fd bsr16 -> 0x53eb */
void step_pcm_play_bsr16_to_0x53eb_a(void)
{
    MCU_PushStack(0x5300);
    mcu.pc = 0x53eb;
}

/* 0x5300 MOVG3 r0 -> (dp,0xd15a) */
void step_pcm_play_movg3_r0_to_dp_0xd15a(void)
{
    uint16_t &r0 = mcu.r[0];

    store16(dp_addr(0xd15a), r0);
    mcu.pc = 0x5304;
}

/* 0x5304 BCLR_ANDC #0xf8ff r0 */
void step_pcm_play_bclr_andc_0xf8ff_r0(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x5308;
}

/* 0x5308 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x530c;
}

/* 0x530c bsr16 -> 0x5998 */
void step_pcm_play_bsr16_to_0x5998(void)
{
    MCU_PushStack(0x530f);
    mcu.pc = 0x5998;
}

/* 0x530f MOVG2 (dp,0xd15a) r0 */
void step_pcm_play_movg2_dp_0xd15a_r0(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd15a));
    mcu.pc = 0x5313;
}

/* 0x5313 MOVG2 (dp,0xd158) r2 */
void step_pcm_play_movg2_dp_0xd158_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd158));
    mcu.pc = 0x5317;
}

/* 0x5317 bsr16 -> 0x5d6e */
void step_pcm_play_bsr16_to_0x5d6e(void)
{
    MCU_PushStack(0x531a);
    mcu.pc = 0x5d6e;
}

/* 0x531a MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_a(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x531e;
}

/* 0x531e MOVG2 @r0+-2 r1 */
void step_pcm_play_movg2_at_r0_minus_2_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x5321;
}

/* 0x5321 BTSTI @r0+-59 #7 */
void step_pcm_play_btsti_at_r0_minus_59_7(void)
{
    btsti8_mem(ind_addr(0, (uint16_t)-59), 7);
    mcu.pc = 0x5324;
}

/* 0x5324 BEQ 28 -> 0x5342 */
void step_pcm_play_beq_28_to_0x5342(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x5342 : 0x5326;
}

/* 0x5326 bsr16 -> 0x3580 */
void step_pcm_play_bsr16_to_0x3580(void)
{
    MCU_PushStack(0x5329);
    mcu.pc = 0x3580;
}

/* 0x5329 MOVG2 (dp,0xd15a) r0 */
void step_pcm_play_movg2_dp_0xd15a_r0_a(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd15a));
    mcu.pc = 0x532d;
}

/* 0x532d bsr16 -> 0x3580 */
void step_pcm_play_bsr16_to_0x3580_a(void)
{
    MCU_PushStack(0x5330);
    mcu.pc = 0x3580;
}

/* 0x5330 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_b(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x5334;
}

/* 0x5334 bsr16 -> 0x3615 */
void step_pcm_play_bsr16_to_0x3615(void)
{
    MCU_PushStack(0x5337);
    mcu.pc = 0x3615;
}

/* 0x5337 MOVG2 (dp,0xd15a) r0 */
void step_pcm_play_movg2_dp_0xd15a_r0_b(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd15a));
    mcu.pc = 0x533b;
}

/* 0x533b MOVG2 (dp,0xd158) r2 */
void step_pcm_play_movg2_dp_0xd158_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd158));
    mcu.pc = 0x533f;
}

/* 0x533f bsr16 -> 0x3a9e */
void step_pcm_play_bsr16_to_0x3a9e(void)
{
    MCU_PushStack(0x5342);
    mcu.pc = 0x3a9e;
}

/* 0x5342 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_c(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x5346;
}

/* 0x5346 bsr16 -> 0x546e */
void step_pcm_play_bsr16_to_0x546e(void)
{
    MCU_PushStack(0x5349);
    mcu.pc = 0x546e;
}

/* 0x5349 MOVG2 (dp,0xd15a) r0 */
void step_pcm_play_movg2_dp_0xd15a_r0_c(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd15a));
    mcu.pc = 0x534d;
}

/* 0x534d bsr16 -> 0x546e */
void step_pcm_play_bsr16_to_0x546e_a(void)
{
    MCU_PushStack(0x5350);
    mcu.pc = 0x546e;
}

/* 0x5350 MOVG2 (dp,0xd158) r1 */
void step_pcm_play_movg2_dp_0xd158_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, dp_addr(0xd158));
    mcu.pc = 0x5354;
}

/* 0x5354 bsr16 -> 0x54cc */
void step_pcm_play_bsr16_to_0x54cc(void)
{
    MCU_PushStack(0x5357);
    mcu.pc = 0x54cc;
}

/* 0x5357 MOVG2 (dp,0xd15a) r1 */
void step_pcm_play_movg2_dp_0xd15a_r1(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, dp_addr(0xd15a));
    mcu.pc = 0x535b;
}

/* 0x535b bsr16 -> 0x54cc */
void step_pcm_play_bsr16_to_0x54cc_a(void)
{
    MCU_PushStack(0x535e);
    mcu.pc = 0x54cc;
}

/* 0x535e BSET_ORC #0x0700 r0 */
void step_pcm_play_bset_orc_0x700_r0(void)
{
    mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x5362;
}

/* 0x5362 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_d(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x5366;
}

/* 0x5366 bsr16 -> 0x54fb */
void step_pcm_play_bsr16_to_0x54fb(void)
{
    MCU_PushStack(0x5369);
    mcu.pc = 0x54fb;
}

/* 0x5369 bsr16 -> 0x5533 */
void step_pcm_play_bsr16_to_0x5533(void)
{
    MCU_PushStack(0x536c);
    mcu.pc = 0x5533;
}

/* 0x536c MOVG2 (dp,0xd15a) r0 */
void step_pcm_play_movg2_dp_0xd15a_r0_d(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd15a));
    mcu.pc = 0x5370;
}

/* 0x5370 bsr16 -> 0x5533 */
void step_pcm_play_bsr16_to_0x5533_a(void)
{
    MCU_PushStack(0x5373);
    mcu.pc = 0x5533;
}

/* 0x5373 bsr16 -> 0x564a */
void step_pcm_play_bsr16_to_0x564a(void)
{
    MCU_PushStack(0x5376);
    mcu.pc = 0x564a;
}

/* 0x5376 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_e(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x537a;
}

/* 0x537a BTSTI @r0+-59 #7 */
void step_pcm_play_btsti_at_r0_minus_59_7_a(void)
{
    btsti8_mem(ind_addr(0, (uint16_t)-59), 7);
    mcu.pc = 0x537d;
}

/* 0x537d BEQ 10 -> 0x5389 */
void step_pcm_play_beq_10_to_0x5389(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x5389 : 0x537f;
}

/* 0x537f bsr16 -> 0x5626 */
void step_pcm_play_bsr16_to_0x5626(void)
{
    MCU_PushStack(0x5382);
    mcu.pc = 0x5626;
}

/* 0x5382 MOVG2 (dp,0xd15a) r0 */
void step_pcm_play_movg2_dp_0xd15a_r0_e(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd15a));
    mcu.pc = 0x5386;
}

/* 0x5386 bsr16 -> 0x5626 */
void step_pcm_play_bsr16_to_0x5626_a(void)
{
    MCU_PushStack(0x5389);
    mcu.pc = 0x5626;
}

/* 0x5389 BCLR_ANDC #0xf8ff r0 */
void step_pcm_play_bclr_andc_0xf8ff_r0_a(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x538d;
}

/* 0x538d BRA 0xfffffe6c -> 0x51fc */
void step_pcm_play_bra_0xfffffe6c_to_0x51fc(void)
{
    mcu.pc = 0x51fc;
}

/* 0x5390 bsr 89 -> 0x53eb */
void step_pcm_play_bsr_89_to_0x53eb(void)
{
    MCU_PushStack(0x5392);
    mcu.pc = 0x53eb;
}

/* 0x5392 MOVG3 r0 -> (dp,0xd158) */
void step_pcm_play_movg3_r0_to_dp_0xd158_a(void)
{
    uint16_t &r0 = mcu.r[0];

    store16(dp_addr(0xd158), r0);
    mcu.pc = 0x5396;
}

/* 0x5396 MOVG #0xffff -> (dp,0xd15a) */
void step_pcm_play_movg_0xffff_to_dp_0xd15a(void)
{
    movg_imm16(dp_addr(0xd15a), 0xffff);
    mcu.pc = 0x539c;
}

/* 0x539c BCLR_ANDC #0xf8ff r0 */
void step_pcm_play_bclr_andc_0xf8ff_r0_b(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x53a0;
}

/* 0x53a0 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_f(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x53a4;
}

/* 0x53a4 bsr16 -> 0x5998 */
void step_pcm_play_bsr16_to_0x5998_a(void)
{
    MCU_PushStack(0x53a7);
    mcu.pc = 0x5998;
}

/* 0x53a7 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_g(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x53ab;
}

/* 0x53ab MOVG2 @r0+-2 r1 */
void step_pcm_play_movg2_at_r0_minus_2_r1_a(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, ind_addr(0, (uint16_t)-2));
    mcu.pc = 0x53ae;
}

/* 0x53ae BTSTI @r0+-59 #7 */
void step_pcm_play_btsti_at_r0_minus_59_7_b(void)
{
    btsti8_mem(ind_addr(0, (uint16_t)-59), 7);
    mcu.pc = 0x53b1;
}

/* 0x53b1 BEQ 10 -> 0x53bd */
void step_pcm_play_beq_10_to_0x53bd(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x53bd : 0x53b3;
}

/* 0x53b3 bsr16 -> 0x3580 */
void step_pcm_play_bsr16_to_0x3580_b(void)
{
    MCU_PushStack(0x53b6);
    mcu.pc = 0x3580;
}

/* 0x53b6 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_h(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x53ba;
}

/* 0x53ba bsr16 -> 0x3615 */
void step_pcm_play_bsr16_to_0x3615_a(void)
{
    MCU_PushStack(0x53bd);
    mcu.pc = 0x3615;
}

/* 0x53bd bsr16 -> 0x546e */
void step_pcm_play_bsr16_to_0x546e_b(void)
{
    MCU_PushStack(0x53c0);
    mcu.pc = 0x546e;
}

/* 0x53c0 MOVG2 (dp,0xd158) r1 */
void step_pcm_play_movg2_dp_0xd158_r1_a(void)
{
    uint16_t &r1 = mcu.r[1];

    load16(r1, dp_addr(0xd158));
    mcu.pc = 0x53c4;
}

/* 0x53c4 bsr16 -> 0x54cc */
void step_pcm_play_bsr16_to_0x54cc_b(void)
{
    MCU_PushStack(0x53c7);
    mcu.pc = 0x54cc;
}

/* 0x53c7 BSET_ORC #0x0700 r0 */
void step_pcm_play_bset_orc_0x700_r0_a(void)
{
    mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x53cb;
}

/* 0x53cb MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_i(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x53cf;
}

/* 0x53cf bsr16 -> 0x54fb */
void step_pcm_play_bsr16_to_0x54fb_a(void)
{
    MCU_PushStack(0x53d2);
    mcu.pc = 0x54fb;
}

/* 0x53d2 bsr16 -> 0x5533 */
void step_pcm_play_bsr16_to_0x5533_b(void)
{
    MCU_PushStack(0x53d5);
    mcu.pc = 0x5533;
}

/* 0x53d5 bsr16 -> 0x564a */
void step_pcm_play_bsr16_to_0x564a_a(void)
{
    MCU_PushStack(0x53d8);
    mcu.pc = 0x564a;
}

/* 0x53d8 MOVG2 (dp,0xd158) r0 */
void step_pcm_play_movg2_dp_0xd158_r0_j(void)
{
    uint16_t &r0 = mcu.r[0];

    load16(r0, dp_addr(0xd158));
    mcu.pc = 0x53dc;
}

/* 0x53dc BTSTI @r0+-59 #7 */
void step_pcm_play_btsti_at_r0_minus_59_7_c(void)
{
    btsti8_mem(ind_addr(0, (uint16_t)-59), 7);
    mcu.pc = 0x53df;
}

/* 0x53df BEQ 3 -> 0x53e4 */
void step_pcm_play_beq_3_to_0x53e4(void)
{
    mcu.pc = (mcu.sr & STATUS_Z) ? 0x53e4 : 0x53e1;
}

/* 0x53e1 bsr16 -> 0x5626 */
void step_pcm_play_bsr16_to_0x5626_b(void)
{
    MCU_PushStack(0x53e4);
    mcu.pc = 0x5626;
}

/* 0x53e4 BCLR_ANDC #0xf8ff r0 */
void step_pcm_play_bclr_andc_0xf8ff_r0_c(void)
{
    mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
    mcu.ex_ignore = 1;
    mcu.pc = 0x53e8;
}

/* 0x53e8 BRA 0xfffffe11 -> 0x51fc */
void step_pcm_play_bra_0xfffffe11_to_0x51fc(void)
{
    mcu.pc = 0x51fc;
}

/* ---- S6: coeff_calc 0x5998-0x5d6d ---------------------------------------- */

/* IML=0 with the stock run interrupted at ~50 different instruction labels
 * (41 unique resume PCs in the [225M,296M) trace alone), so the only exact
 * form is one entry per instruction (each returns 1): the host polls, runs
 * TIMER_Clock and traces between every instruction exactly like stock.
 *
 * The fixed-point maths is the ROM's, transcribed step by step as named
 * operations (mulxu8/mulxu16_imm, the table sum, the clamp and the shift
 * scaling); no algebraic rewriting or constant folding is applied. The routine
 * reads its parameters from the tone record (r2 = word[(dp,0xd17e)]) and the
 * per-slot coefficient base byte (dp,0xd180), then writes the nine DSP
 * coefficients P+0x86..90/92/94/96 and P-80/-114. */


/* 0x5998 CLR r6 */
void step_coeff_calc_clr_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    clr16(r6);
    mcu.pc = 0x599a;
}

/* 0x599a MOVG2 @r1+0xce78 r6 */
void step_coeff_calc_movg2_at_r1_plus_0xce78_r6(void)
{
    uint16_t &r6 = mcu.r[6];

    load8(r6, ind_addr(1, 0xce78));
    mcu.pc = 0x599e;
}

/* 0x599e MOVG2 r6 r3 */
void step_coeff_calc_movg2_r6_r3(void)
{
    uint16_t &r3 = mcu.r[3];
    uint16_t &r6 = mcu.r[6];

    move16(r3, r6);
    mcu.pc = 0x59a0;
}

/* 0x59a0 ADD r3 r3 */
void step_coeff_calc_add_r3_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    add16(r3, r3);
    mcu.pc = 0x59a2;
}

/* 0x59a2 MOVG2 @r3+0x7218 r2 */
void step_coeff_calc_movg2_at_r3_plus_0x7218_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, ind_addr(3, 0x7218));
    mcu.pc = 0x59a6;
}

/* 0x59a6 MOVG3 r2 -> (dp,0xd17e) */
void step_coeff_calc_movg3_r2_to_dp_0xd17e(void)
{
    uint16_t &r2 = mcu.r[2];

    store16(dp_addr(0xd17e), r2);
    mcu.pc = 0x59aa;
}

/* 0x59aa move r4 #0x80 */
void step_coeff_calc_move_r4_0x80(void)
{
    uint16_t &r4 = mcu.r[4];

    move8(r4, 0x80);
    mcu.pc = 0x59ac;
}

/* 0x59ac MULXU r6 r4 */
void step_coeff_calc_mulxu_r6_r4(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r6 = mcu.r[6];

    mulxu8_reg(r6, r4);
    mcu.pc = 0x59ae;
}

/* 0x59ae CLR r3 */
void step_coeff_calc_clr_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    clr16(r3);
    mcu.pc = 0x59b0;
}

/* 0x59b0 MOVG2 @r1+0xd134 r3 */
void step_coeff_calc_movg2_at_r1_plus_0xd134_r3(void)
{
    uint16_t &r3 = mcu.r[3];

    load8(r3, ind_addr(1, 0xd134));
    mcu.pc = 0x59b4;
}

/* 0x59b4 ADD r4 r3 */
void step_coeff_calc_add_r4_r3(void)
{
    uint16_t &r3 = mcu.r[3];
    uint16_t &r4 = mcu.r[4];

    add16(r3, r4);
    mcu.pc = 0x59b6;
}

/* 0x59b6 MOVG2 @r3+0x9740 r5 */
void step_coeff_calc_movg2_at_r3_plus_0x9740_r5(void)
{
    uint16_t &r5 = mcu.r[5];

    load8(r5, ind_addr(3, 0x9740));
    mcu.pc = 0x59ba;
}

/* 0x59ba MOVG3 r5 -> (dp,0xd180) */
void step_coeff_calc_movg3_r5_to_dp_0xd180(void)
{
    uint16_t &r5 = mcu.r[5];

    store8(dp_addr(0xd180), (uint8_t)r5);
    mcu.pc = 0x59be;
}

/* 0x59be MOVG2 @r2+76 r4 */
void step_coeff_calc_movg2_at_r2_plus_76_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 76));
    mcu.pc = 0x59c1;
}

/* 0x59c1 SUB #0x40 r4 */
void step_coeff_calc_sub_0x40_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    sub8_imm(r4, 0x40);
    mcu.pc = 0x59c4;
}

/* 0x59c4 BCC 9 -> 0x59cf */
void step_coeff_calc_bcc_9_to_0x59cf(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x59c6 : 0x59cf;
}

/* 0x59c6 NEG r4 */
void step_coeff_calc_neg_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    neg8(r4);
    mcu.pc = 0x59c8;
}

/* 0x59c8 MULXU r5 r4 */
void step_coeff_calc_mulxu_r5_r4(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu8_reg(r5, r4);
    mcu.pc = 0x59ca;
}

/* 0x59ca NEG r4 */
void step_coeff_calc_neg_r4_a(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x59cc;
}

/* 0x59cc BRA 3 -> 0x59d1 */
void step_coeff_calc_bra_3_to_0x59d1(void)
{
    mcu.pc = 0x59d1;
}

/* 0x59ce rts */
void step_coeff_calc_rts(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x59cf MULXU r5 r4 */
void step_coeff_calc_mulxu_r5_r4_a(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu8_reg(r5, r4);
    mcu.pc = 0x59d1;
}

/* 0x59d1 MOVG2 r6 r3 */
void step_coeff_calc_movg2_r6_r3_a(void)
{
    uint16_t &r3 = mcu.r[3];
    uint16_t &r6 = mcu.r[6];

    move16(r3, r6);
    mcu.pc = 0x59d3;
}

/* 0x59d3 ADD r3 r3 */
void step_coeff_calc_add_r3_r3_a(void)
{
    uint16_t &r3 = mcu.r[3];

    add16(r3, r3);
    mcu.pc = 0x59d5;
}

/* 0x59d5 ADD @r3+0x9060 r4 */
void step_coeff_calc_add_at_r3_plus_0x9060_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9060));
    mcu.pc = 0x59d9;
}

/* 0x59d9 ADD @r3+0x91c0 r4 */
void step_coeff_calc_add_at_r3_plus_0x91c0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x91c0));
    mcu.pc = 0x59dd;
}

/* 0x59dd ADD @r3+0x9320 r4 */
void step_coeff_calc_add_at_r3_plus_0x9320_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9320));
    mcu.pc = 0x59e1;
}

/* 0x59e1 ADD @r3+0x9480 r4 */
void step_coeff_calc_add_at_r3_plus_0x9480_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9480));
    mcu.pc = 0x59e5;
}

/* 0x59e5 ADD @r3+0x95e0 r4 */
void step_coeff_calc_add_at_r3_plus_0x95e0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x95e0));
    mcu.pc = 0x59e9;
}

/* 0x59e9 BPL 24 -> 0x5a03 */
void step_coeff_calc_bpl_24_to_0x5a03(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x59eb : 0x5a03;
}

/* 0x59eb NEG r4 */
void step_coeff_calc_neg_r4_b(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x59ed;
}

/* 0x59ed cmp r4,w #0x0be8 */
void step_coeff_calc_cmp_r4_w_0xbe8(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0be8);
    mcu.pc = 0x59f0;
}

/* 0x59f0 BCS 3 -> 0x59f5 */
void step_coeff_calc_bcs_3_to_0x59f5(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x59f5 : 0x59f2;
}

/* 0x59f2 movi r4 #0x0be8 */
void step_coeff_calc_movi_r4_0xbe8(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0be8);
    mcu.pc = 0x59f5;
}

/* 0x59f5 ADD r4 r4 */
void step_coeff_calc_add_r4_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x59f7;
}

/* 0x59f7 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_a(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x59f9;
}

/* 0x59f9 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_b(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x59fb;
}

/* 0x59fb MULXU #0xfbf8 r4:r5 */
void step_coeff_calc_mulxu_0xfbf8_r4_r5(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xfbf8, r4, r5);
    mcu.pc = 0x59ff;
}

/* 0x59ff NEG r4 */
void step_coeff_calc_neg_r4_c(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5a01;
}

/* 0x5a01 BRA 18 -> 0x5a15 */
void step_coeff_calc_bra_18_to_0x5a15(void)
{
    mcu.pc = 0x5a15;
}

/* 0x5a03 cmp r4,w #0x0be8 */
void step_coeff_calc_cmp_r4_w_0xbe8_a(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0be8);
    mcu.pc = 0x5a06;
}

/* 0x5a06 BCS 3 -> 0x5a0b */
void step_coeff_calc_bcs_3_to_0x5a0b(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5a0b : 0x5a08;
}

/* 0x5a08 movi r4 #0x0be8 */
void step_coeff_calc_movi_r4_0xbe8_a(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0be8);
    mcu.pc = 0x5a0b;
}

/* 0x5a0b ADD r4 r4 */
void step_coeff_calc_add_r4_r4_c(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a0d;
}

/* 0x5a0d ADD r4 r4 */
void step_coeff_calc_add_r4_r4_d(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a0f;
}

/* 0x5a0f ADD r4 r4 */
void step_coeff_calc_add_r4_r4_e(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a11;
}

/* 0x5a11 MULXU #0xfbf8 r4:r5 */
void step_coeff_calc_mulxu_0xfbf8_r4_r5_a(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xfbf8, r4, r5);
    mcu.pc = 0x5a15;
}

/* 0x5a15 MOVG3 r4 -> @r0+0x0086 */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x86(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x86), r4);
    mcu.pc = 0x5a19;
}

/* 0x5a19 MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5a1d;
}

/* 0x5a1d MOVG2 @r2+78 r4 */
void step_coeff_calc_movg2_at_r2_plus_78_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 78));
    mcu.pc = 0x5a20;
}

/* 0x5a20 SUB #0x40 r4 */
void step_coeff_calc_sub_0x40_r4_a(void)
{
    uint16_t &r4 = mcu.r[4];

    sub8_imm(r4, 0x40);
    mcu.pc = 0x5a23;
}

/* 0x5a23 BCC 12 -> 0x5a31 */
void step_coeff_calc_bcc_12_to_0x5a31(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5a25 : 0x5a31;
}

/* 0x5a25 NEG r4 */
void step_coeff_calc_neg_r4_d(void)
{
    uint16_t &r4 = mcu.r[4];

    neg8(r4);
    mcu.pc = 0x5a27;
}

/* 0x5a27 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5a2b;
}

/* 0x5a2b SHLR r4 */
void step_coeff_calc_shlr_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5a2d;
}

/* 0x5a2d NEG r4 */
void step_coeff_calc_neg_r4_e(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5a2f;
}

/* 0x5a2f BRA 6 -> 0x5a37 */
void step_coeff_calc_bra_6_to_0x5a37(void)
{
    mcu.pc = 0x5a37;
}

/* 0x5a31 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_a(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5a35;
}

/* 0x5a35 SHLR r4 */
void step_coeff_calc_shlr_r4_a(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5a37;
}

/* 0x5a37 ADD @r3+0x90a0 r4 */
void step_coeff_calc_add_at_r3_plus_0x90a0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x90a0));
    mcu.pc = 0x5a3b;
}

/* 0x5a3b ADD @r3+0x9200 r4 */
void step_coeff_calc_add_at_r3_plus_0x9200_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9200));
    mcu.pc = 0x5a3f;
}

/* 0x5a3f ADD @r3+0x9360 r4 */
void step_coeff_calc_add_at_r3_plus_0x9360_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9360));
    mcu.pc = 0x5a43;
}

/* 0x5a43 ADD @r3+0x94c0 r4 */
void step_coeff_calc_add_at_r3_plus_0x94c0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x94c0));
    mcu.pc = 0x5a47;
}

/* 0x5a47 ADD @r3+0x9620 r4 */
void step_coeff_calc_add_at_r3_plus_0x9620_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9620));
    mcu.pc = 0x5a4b;
}

/* 0x5a4b BPL 26 -> 0x5a67 */
void step_coeff_calc_bpl_26_to_0x5a67(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5a4d : 0x5a67;
}

/* 0x5a4d NEG r4 */
void step_coeff_calc_neg_r4_f(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5a4f;
}

/* 0x5a4f cmp r4,w #0x0fa0 */
void step_coeff_calc_cmp_r4_w_0xfa0(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fa0);
    mcu.pc = 0x5a52;
}

/* 0x5a52 BCS 3 -> 0x5a57 */
void step_coeff_calc_bcs_3_to_0x5a57(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5a57 : 0x5a54;
}

/* 0x5a54 movi r4 #0x0fa0 */
void step_coeff_calc_movi_r4_0xfa0(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fa0);
    mcu.pc = 0x5a57;
}

/* 0x5a57 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_f(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a59;
}

/* 0x5a59 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_g(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a5b;
}

/* 0x5a5b ADD r4 r4 */
void step_coeff_calc_add_r4_r4_h(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a5d;
}

/* 0x5a5d ADD r4 r4 */
void step_coeff_calc_add_r4_r4_i(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a5f;
}

/* 0x5a5f MULXU #0x820d r4:r5 */
void step_coeff_calc_mulxu_0x820d_r4_r5(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0x820d, r4, r5);
    mcu.pc = 0x5a63;
}

/* 0x5a63 NEG r4 */
void step_coeff_calc_neg_r4_g(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5a65;
}

/* 0x5a65 BRA 20 -> 0x5a7b */
void step_coeff_calc_bra_20_to_0x5a7b(void)
{
    mcu.pc = 0x5a7b;
}

/* 0x5a67 cmp r4,w #0x0fa0 */
void step_coeff_calc_cmp_r4_w_0xfa0_a(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fa0);
    mcu.pc = 0x5a6a;
}

/* 0x5a6a BCS 3 -> 0x5a6f */
void step_coeff_calc_bcs_3_to_0x5a6f(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5a6f : 0x5a6c;
}

/* 0x5a6c movi r4 #0x0fa0 */
void step_coeff_calc_movi_r4_0xfa0_a(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fa0);
    mcu.pc = 0x5a6f;
}

/* 0x5a6f ADD r4 r4 */
void step_coeff_calc_add_r4_r4_j(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a71;
}

/* 0x5a71 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_k(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a73;
}

/* 0x5a73 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_l(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a75;
}

/* 0x5a75 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_m(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5a77;
}

/* 0x5a77 MULXU #0x820d r4:r5 */
void step_coeff_calc_mulxu_0x820d_r4_r5_a(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0x820d, r4, r5);
    mcu.pc = 0x5a7b;
}

/* 0x5a7b MOVG3 r4 -> @r0+0x008a */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x8a(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x8a), r4);
    mcu.pc = 0x5a7f;
}

/* 0x5a7f MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_a(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5a83;
}

/* 0x5a83 MOVG2 @r2+77 r4 */
void step_coeff_calc_movg2_at_r2_plus_77_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 77));
    mcu.pc = 0x5a86;
}

/* 0x5a86 SUB #0x40 r4 */
void step_coeff_calc_sub_0x40_r4_b(void)
{
    uint16_t &r4 = mcu.r[4];

    sub8_imm(r4, 0x40);
    mcu.pc = 0x5a89;
}

/* 0x5a89 BCC 12 -> 0x5a97 */
void step_coeff_calc_bcc_12_to_0x5a97(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5a8b : 0x5a97;
}

/* 0x5a8b NEG r4 */
void step_coeff_calc_neg_r4_h(void)
{
    uint16_t &r4 = mcu.r[4];

    neg8(r4);
    mcu.pc = 0x5a8d;
}

/* 0x5a8d MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_b(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5a91;
}

/* 0x5a91 SHLR r4 */
void step_coeff_calc_shlr_r4_b(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5a93;
}

/* 0x5a93 NEG r4 */
void step_coeff_calc_neg_r4_i(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5a95;
}

/* 0x5a95 BRA 6 -> 0x5a9d */
void step_coeff_calc_bra_6_to_0x5a9d(void)
{
    mcu.pc = 0x5a9d;
}

/* 0x5a97 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_c(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5a9b;
}

/* 0x5a9b SHLR r4 */
void step_coeff_calc_shlr_r4_c(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5a9d;
}

/* 0x5a9d ADD @r3+0x9080 r4 */
void step_coeff_calc_add_at_r3_plus_0x9080_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9080));
    mcu.pc = 0x5aa1;
}

/* 0x5aa1 ADD @r3+0x91e0 r4 */
void step_coeff_calc_add_at_r3_plus_0x91e0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x91e0));
    mcu.pc = 0x5aa5;
}

/* 0x5aa5 ADD @r3+0x9340 r4 */
void step_coeff_calc_add_at_r3_plus_0x9340_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9340));
    mcu.pc = 0x5aa9;
}

/* 0x5aa9 ADD @r3+0x94a0 r4 */
void step_coeff_calc_add_at_r3_plus_0x94a0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x94a0));
    mcu.pc = 0x5aad;
}

/* 0x5aad ADD @r3+0x9600 r4 */
void step_coeff_calc_add_at_r3_plus_0x9600_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9600));
    mcu.pc = 0x5ab1;
}

/* 0x5ab1 BPL 24 -> 0x5acb */
void step_coeff_calc_bpl_24_to_0x5acb(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5ab3 : 0x5acb;
}

/* 0x5ab3 NEG r4 */
void step_coeff_calc_neg_r4_j(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5ab5;
}

/* 0x5ab5 cmp r4,w #0x0fa0 */
void step_coeff_calc_cmp_r4_w_0xfa0_b(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fa0);
    mcu.pc = 0x5ab8;
}

/* 0x5ab8 BCS 3 -> 0x5abd */
void step_coeff_calc_bcs_3_to_0x5abd(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5abd : 0x5aba;
}

/* 0x5aba movi r4 #0x0fa0 */
void step_coeff_calc_movi_r4_0xfa0_b(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fa0);
    mcu.pc = 0x5abd;
}

/* 0x5abd ADD r4 r4 */
void step_coeff_calc_add_r4_r4_n(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5abf;
}

/* 0x5abf ADD r4 r4 */
void step_coeff_calc_add_r4_r4_o(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5ac1;
}

/* 0x5ac1 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_p(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5ac3;
}

/* 0x5ac3 MULXU #0xc49c r4:r5 */
void step_coeff_calc_mulxu_0xc49c_r4_r5(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xc49c, r4, r5);
    mcu.pc = 0x5ac7;
}

/* 0x5ac7 NEG r4 */
void step_coeff_calc_neg_r4_k(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5ac9;
}

/* 0x5ac9 BRA 18 -> 0x5add */
void step_coeff_calc_bra_18_to_0x5add(void)
{
    mcu.pc = 0x5add;
}

/* 0x5acb cmp r4,w #0x0fa0 */
void step_coeff_calc_cmp_r4_w_0xfa0_c(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fa0);
    mcu.pc = 0x5ace;
}

/* 0x5ace BCS 3 -> 0x5ad3 */
void step_coeff_calc_bcs_3_to_0x5ad3(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5ad3 : 0x5ad0;
}

/* 0x5ad0 movi r4 #0x0fa0 */
void step_coeff_calc_movi_r4_0xfa0_c(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fa0);
    mcu.pc = 0x5ad3;
}

/* 0x5ad3 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_q(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5ad5;
}

/* 0x5ad5 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_r(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5ad7;
}

/* 0x5ad7 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_s(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5ad9;
}

/* 0x5ad9 MULXU #0xc49c r4:r5 */
void step_coeff_calc_mulxu_0xc49c_r4_r5_a(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xc49c, r4, r5);
    mcu.pc = 0x5add;
}

/* 0x5add MOVG3 r4 -> @r0+0x0088 */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x88(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x88), r4);
    mcu.pc = 0x5ae1;
}

/* 0x5ae1 MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_b(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5ae5;
}

/* 0x5ae5 MOVG2 @r2+84 r4 */
void step_coeff_calc_movg2_at_r2_plus_84_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 84));
    mcu.pc = 0x5ae8;
}

/* 0x5ae8 SUB #0x40 r4 */
void step_coeff_calc_sub_0x40_r4_c(void)
{
    uint16_t &r4 = mcu.r[4];

    sub8_imm(r4, 0x40);
    mcu.pc = 0x5aeb;
}

/* 0x5aeb BCC 12 -> 0x5af9 */
void step_coeff_calc_bcc_12_to_0x5af9(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5aed : 0x5af9;
}

/* 0x5aed NEG r4 */
void step_coeff_calc_neg_r4_l(void)
{
    uint16_t &r4 = mcu.r[4];

    neg8(r4);
    mcu.pc = 0x5aef;
}

/* 0x5aef MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_d(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5af3;
}

/* 0x5af3 SHLR r4 */
void step_coeff_calc_shlr_r4_d(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5af5;
}

/* 0x5af5 NEG r4 */
void step_coeff_calc_neg_r4_m(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5af7;
}

/* 0x5af7 BRA 6 -> 0x5aff */
void step_coeff_calc_bra_6_to_0x5aff(void)
{
    mcu.pc = 0x5aff;
}

/* 0x5af9 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_e(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5afd;
}

/* 0x5afd SHLR r4 */
void step_coeff_calc_shlr_r4_e(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5aff;
}

/* 0x5aff ADD @r3+0x9140 r4 */
void step_coeff_calc_add_at_r3_plus_0x9140_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9140));
    mcu.pc = 0x5b03;
}

/* 0x5b03 ADD @r3+0x92a0 r4 */
void step_coeff_calc_add_at_r3_plus_0x92a0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x92a0));
    mcu.pc = 0x5b07;
}

/* 0x5b07 ADD @r3+0x9400 r4 */
void step_coeff_calc_add_at_r3_plus_0x9400_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9400));
    mcu.pc = 0x5b0b;
}

/* 0x5b0b ADD @r3+0x9560 r4 */
void step_coeff_calc_add_at_r3_plus_0x9560_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9560));
    mcu.pc = 0x5b0f;
}

/* 0x5b0f ADD @r3+0x96c0 r4 */
void step_coeff_calc_add_at_r3_plus_0x96c0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x96c0));
    mcu.pc = 0x5b13;
}

/* 0x5b13 BPL 20 -> 0x5b29 */
void step_coeff_calc_bpl_20_to_0x5b29(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5b15 : 0x5b29;
}

/* 0x5b15 NEG r4 */
void step_coeff_calc_neg_r4_n(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5b17;
}

/* 0x5b17 cmp r4,w #0x0fa0 */
void step_coeff_calc_cmp_r4_w_0xfa0_d(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fa0);
    mcu.pc = 0x5b1a;
}

/* 0x5b1a BCS 3 -> 0x5b1f */
void step_coeff_calc_bcs_3_to_0x5b1f(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5b1f : 0x5b1c;
}

/* 0x5b1c movi r4 #0x0fa0 */
void step_coeff_calc_movi_r4_0xfa0_d(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fa0);
    mcu.pc = 0x5b1f;
}

/* 0x5b1f ADD r4 r4 */
void step_coeff_calc_add_r4_r4_t(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5b21;
}

/* 0x5b21 MULXU #0xa7c7 r4:r5 */
void step_coeff_calc_mulxu_0xa7c7_r4_r5(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xa7c7, r4, r5);
    mcu.pc = 0x5b25;
}

/* 0x5b25 NEG r4 */
void step_coeff_calc_neg_r4_o(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5b27;
}

/* 0x5b27 BRA 14 -> 0x5b37 */
void step_coeff_calc_bra_14_to_0x5b37(void)
{
    mcu.pc = 0x5b37;
}

/* 0x5b29 cmp r4,w #0x0fa0 */
void step_coeff_calc_cmp_r4_w_0xfa0_e(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fa0);
    mcu.pc = 0x5b2c;
}

/* 0x5b2c BCS 3 -> 0x5b31 */
void step_coeff_calc_bcs_3_to_0x5b31(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5b31 : 0x5b2e;
}

/* 0x5b2e movi r4 #0x0fa0 */
void step_coeff_calc_movi_r4_0xfa0_e(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fa0);
    mcu.pc = 0x5b31;
}

/* 0x5b31 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_u(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5b33;
}

/* 0x5b33 MULXU #0xa7c7 r4:r5 */
void step_coeff_calc_mulxu_0xa7c7_r4_r5_a(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xa7c7, r4, r5);
    mcu.pc = 0x5b37;
}

/* 0x5b37 MOVG3 r4 -> @r0+-80 */
void step_coeff_calc_movg3_r4_to_at_r0_minus_80(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, (uint16_t)-80), r4);
    mcu.pc = 0x5b3a;
}

/* 0x5b3a MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_c(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5b3e;
}

/* 0x5b3e MOVG2 @r2+80 r4 */
void step_coeff_calc_movg2_at_r2_plus_80_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 80));
    mcu.pc = 0x5b41;
}

/* 0x5b41 SUB #0x40 r4 */
void step_coeff_calc_sub_0x40_r4_d(void)
{
    uint16_t &r4 = mcu.r[4];

    sub8_imm(r4, 0x40);
    mcu.pc = 0x5b44;
}

/* 0x5b44 BCC 12 -> 0x5b52 */
void step_coeff_calc_bcc_12_to_0x5b52(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5b46 : 0x5b52;
}

/* 0x5b46 NEG r4 */
void step_coeff_calc_neg_r4_p(void)
{
    uint16_t &r4 = mcu.r[4];

    neg8(r4);
    mcu.pc = 0x5b48;
}

/* 0x5b48 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_f(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5b4c;
}

/* 0x5b4c SHLR r4 */
void step_coeff_calc_shlr_r4_f(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5b4e;
}

/* 0x5b4e NEG r4 */
void step_coeff_calc_neg_r4_q(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5b50;
}

/* 0x5b50 BRA 6 -> 0x5b58 */
void step_coeff_calc_bra_6_to_0x5b58(void)
{
    mcu.pc = 0x5b58;
}

/* 0x5b52 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_g(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5b56;
}

/* 0x5b56 SHLR r4 */
void step_coeff_calc_shlr_r4_g(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5b58;
}

/* 0x5b58 ADD @r3+0x90c0 r4 */
void step_coeff_calc_add_at_r3_plus_0x90c0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x90c0));
    mcu.pc = 0x5b5c;
}

/* 0x5b5c ADD @r3+0x9220 r4 */
void step_coeff_calc_add_at_r3_plus_0x9220_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9220));
    mcu.pc = 0x5b60;
}

/* 0x5b60 ADD @r3+0x9380 r4 */
void step_coeff_calc_add_at_r3_plus_0x9380_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9380));
    mcu.pc = 0x5b64;
}

/* 0x5b64 ADD @r3+0x94e0 r4 */
void step_coeff_calc_add_at_r3_plus_0x94e0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x94e0));
    mcu.pc = 0x5b68;
}

/* 0x5b68 ADD @r3+0x9640 r4 */
void step_coeff_calc_add_at_r3_plus_0x9640_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9640));
    mcu.pc = 0x5b6c;
}

/* 0x5b6c BPL 20 -> 0x5b82 */
void step_coeff_calc_bpl_20_to_0x5b82(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5b6e : 0x5b82;
}

/* 0x5b6e NEG r4 */
void step_coeff_calc_neg_r4_r(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5b70;
}

/* 0x5b70 cmp r4,w #0x0fa0 */
void step_coeff_calc_cmp_r4_w_0xfa0_f(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fa0);
    mcu.pc = 0x5b73;
}

/* 0x5b73 BCS 3 -> 0x5b78 */
void step_coeff_calc_bcs_3_to_0x5b78(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5b78 : 0x5b75;
}

/* 0x5b75 movi r4 #0x0fa0 */
void step_coeff_calc_movi_r4_0xfa0_f(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fa0);
    mcu.pc = 0x5b78;
}

/* 0x5b78 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_v(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5b7a;
}

/* 0x5b7a MULXU #0xa7c7 r4:r5 */
void step_coeff_calc_mulxu_0xa7c7_r4_r5_b(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xa7c7, r4, r5);
    mcu.pc = 0x5b7e;
}

/* 0x5b7e NEG r4 */
void step_coeff_calc_neg_r4_s(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5b80;
}

/* 0x5b80 BRA 14 -> 0x5b90 */
void step_coeff_calc_bra_14_to_0x5b90(void)
{
    mcu.pc = 0x5b90;
}

/* 0x5b82 cmp r4,w #0x0fa0 */
void step_coeff_calc_cmp_r4_w_0xfa0_g(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fa0);
    mcu.pc = 0x5b85;
}

/* 0x5b85 BCS 3 -> 0x5b8a */
void step_coeff_calc_bcs_3_to_0x5b8a(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5b8a : 0x5b87;
}

/* 0x5b87 movi r4 #0x0fa0 */
void step_coeff_calc_movi_r4_0xfa0_g(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fa0);
    mcu.pc = 0x5b8a;
}

/* 0x5b8a ADD r4 r4 */
void step_coeff_calc_add_r4_r4_w(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5b8c;
}

/* 0x5b8c MULXU #0xa7c7 r4:r5 */
void step_coeff_calc_mulxu_0xa7c7_r4_r5_c(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xa7c7, r4, r5);
    mcu.pc = 0x5b90;
}

/* 0x5b90 MOVG3 r4 -> @r0+-114 */
void step_coeff_calc_movg3_r4_to_at_r0_minus_114(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, (uint16_t)-114), r4);
    mcu.pc = 0x5b93;
}

/* 0x5b93 MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_d(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5b97;
}

/* 0x5b97 MOVG2 @r2+87 r4 */
void step_coeff_calc_movg2_at_r2_plus_87_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 87));
    mcu.pc = 0x5b9a;
}

/* 0x5b9a MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_h(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5b9e;
}

/* 0x5b9e SHLR r4 */
void step_coeff_calc_shlr_r4_h(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5ba0;
}

/* 0x5ba0 SHLR r4 */
void step_coeff_calc_shlr_r4_i(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5ba2;
}

/* 0x5ba2 ADD @r3+0x91a0 r4 */
void step_coeff_calc_add_at_r3_plus_0x91a0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x91a0));
    mcu.pc = 0x5ba6;
}

/* 0x5ba6 ADD @r3+0x9300 r4 */
void step_coeff_calc_add_at_r3_plus_0x9300_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9300));
    mcu.pc = 0x5baa;
}

/* 0x5baa ADD @r3+0x9460 r4 */
void step_coeff_calc_add_at_r3_plus_0x9460_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9460));
    mcu.pc = 0x5bae;
}

/* 0x5bae ADD @r3+0x95c0 r4 */
void step_coeff_calc_add_at_r3_plus_0x95c0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x95c0));
    mcu.pc = 0x5bb2;
}

/* 0x5bb2 ADD @r3+0x9720 r4 */
void step_coeff_calc_add_at_r3_plus_0x9720_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9720));
    mcu.pc = 0x5bb6;
}

/* 0x5bb6 BPL 26 -> 0x5bd2 */
void step_coeff_calc_bpl_26_to_0x5bd2(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5bb8 : 0x5bd2;
}

/* 0x5bb8 NEG r4 */
void step_coeff_calc_neg_r4_t(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5bba;
}

/* 0x5bba cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5bbd;
}

/* 0x5bbd BCS 3 -> 0x5bc2 */
void step_coeff_calc_bcs_3_to_0x5bc2(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5bc2 : 0x5bbf;
}

/* 0x5bbf movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5bc2;
}

/* 0x5bc2 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_x(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5bc4;
}

/* 0x5bc4 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_y(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5bc6;
}

/* 0x5bc6 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_z(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5bc8;
}

/* 0x5bc8 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_27(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5bca;
}

/* 0x5bca MULXU #0x8105 r4:r5 */
void step_coeff_calc_mulxu_0x8105_r4_r5(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0x8105, r4, r5);
    mcu.pc = 0x5bce;
}

/* 0x5bce NEG r4 */
void step_coeff_calc_neg_r4_u(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5bd0;
}

/* 0x5bd0 BRA 20 -> 0x5be6 */
void step_coeff_calc_bra_20_to_0x5be6(void)
{
    mcu.pc = 0x5be6;
}

/* 0x5bd2 cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_a(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5bd5;
}

/* 0x5bd5 BCS 3 -> 0x5bda */
void step_coeff_calc_bcs_3_to_0x5bda(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5bda : 0x5bd7;
}

/* 0x5bd7 movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_a(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5bda;
}

/* 0x5bda ADD r4 r4 */
void step_coeff_calc_add_r4_r4_28(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5bdc;
}

/* 0x5bdc ADD r4 r4 */
void step_coeff_calc_add_r4_r4_29(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5bde;
}

/* 0x5bde ADD r4 r4 */
void step_coeff_calc_add_r4_r4_30(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5be0;
}

/* 0x5be0 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_31(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5be2;
}

/* 0x5be2 MULXU #0x8105 r4:r5 */
void step_coeff_calc_mulxu_0x8105_r4_r5_a(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0x8105, r4, r5);
    mcu.pc = 0x5be6;
}

/* 0x5be6 MOVG3 r4 -> @r0+0x0096 */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x96(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x96), r4);
    mcu.pc = 0x5bea;
}

/* 0x5bea MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_e(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5bee;
}

/* 0x5bee MOVG2 @r2+83 r4 */
void step_coeff_calc_movg2_at_r2_plus_83_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 83));
    mcu.pc = 0x5bf1;
}

/* 0x5bf1 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_i(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5bf5;
}

/* 0x5bf5 SHLR r4 */
void step_coeff_calc_shlr_r4_j(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5bf7;
}

/* 0x5bf7 SHLR r4 */
void step_coeff_calc_shlr_r4_k(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5bf9;
}

/* 0x5bf9 ADD @r3+0x9120 r4 */
void step_coeff_calc_add_at_r3_plus_0x9120_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9120));
    mcu.pc = 0x5bfd;
}

/* 0x5bfd ADD @r3+0x9280 r4 */
void step_coeff_calc_add_at_r3_plus_0x9280_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9280));
    mcu.pc = 0x5c01;
}

/* 0x5c01 ADD @r3+0x93e0 r4 */
void step_coeff_calc_add_at_r3_plus_0x93e0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x93e0));
    mcu.pc = 0x5c05;
}

/* 0x5c05 ADD @r3+0x9540 r4 */
void step_coeff_calc_add_at_r3_plus_0x9540_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9540));
    mcu.pc = 0x5c09;
}

/* 0x5c09 ADD @r3+0x96a0 r4 */
void step_coeff_calc_add_at_r3_plus_0x96a0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x96a0));
    mcu.pc = 0x5c0d;
}

/* 0x5c0d BPL 26 -> 0x5c29 */
void step_coeff_calc_bpl_26_to_0x5c29(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5c0f : 0x5c29;
}

/* 0x5c0f NEG r4 */
void step_coeff_calc_neg_r4_v(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5c11;
}

/* 0x5c11 cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_b(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5c14;
}

/* 0x5c14 BCS 3 -> 0x5c19 */
void step_coeff_calc_bcs_3_to_0x5c19(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5c19 : 0x5c16;
}

/* 0x5c16 movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_b(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5c19;
}

/* 0x5c19 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_32(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c1b;
}

/* 0x5c1b ADD r4 r4 */
void step_coeff_calc_add_r4_r4_33(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c1d;
}

/* 0x5c1d ADD r4 r4 */
void step_coeff_calc_add_r4_r4_34(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c1f;
}

/* 0x5c1f ADD r4 r4 */
void step_coeff_calc_add_r4_r4_35(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c21;
}

/* 0x5c21 MULXU #0x8105 r4:r5 */
void step_coeff_calc_mulxu_0x8105_r4_r5_b(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0x8105, r4, r5);
    mcu.pc = 0x5c25;
}

/* 0x5c25 NEG r4 */
void step_coeff_calc_neg_r4_w(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5c27;
}

/* 0x5c27 BRA 20 -> 0x5c3d */
void step_coeff_calc_bra_20_to_0x5c3d(void)
{
    mcu.pc = 0x5c3d;
}

/* 0x5c29 cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_c(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5c2c;
}

/* 0x5c2c BCS 3 -> 0x5c31 */
void step_coeff_calc_bcs_3_to_0x5c31(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5c31 : 0x5c2e;
}

/* 0x5c2e movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_c(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5c31;
}

/* 0x5c31 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_36(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c33;
}

/* 0x5c33 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_37(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c35;
}

/* 0x5c35 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_38(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c37;
}

/* 0x5c37 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_39(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c39;
}

/* 0x5c39 MULXU #0x8105 r4:r5 */
void step_coeff_calc_mulxu_0x8105_r4_r5_c(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0x8105, r4, r5);
    mcu.pc = 0x5c3d;
}

/* 0x5c3d MOVG3 r4 -> @r0+0x008e */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x8e(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x8e), r4);
    mcu.pc = 0x5c41;
}

/* 0x5c41 MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_f(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5c45;
}

/* 0x5c45 MOVG2 @r2+86 r4 */
void step_coeff_calc_movg2_at_r2_plus_86_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 86));
    mcu.pc = 0x5c48;
}

/* 0x5c48 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_j(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5c4c;
}

/* 0x5c4c SHLR r4 */
void step_coeff_calc_shlr_r4_l(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5c4e;
}

/* 0x5c4e SHLR r4 */
void step_coeff_calc_shlr_r4_m(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5c50;
}

/* 0x5c50 ADD @r3+0x9180 r4 */
void step_coeff_calc_add_at_r3_plus_0x9180_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9180));
    mcu.pc = 0x5c54;
}

/* 0x5c54 ADD @r3+0x92e0 r4 */
void step_coeff_calc_add_at_r3_plus_0x92e0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x92e0));
    mcu.pc = 0x5c58;
}

/* 0x5c58 ADD @r3+0x9440 r4 */
void step_coeff_calc_add_at_r3_plus_0x9440_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9440));
    mcu.pc = 0x5c5c;
}

/* 0x5c5c ADD @r3+0x95a0 r4 */
void step_coeff_calc_add_at_r3_plus_0x95a0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x95a0));
    mcu.pc = 0x5c60;
}

/* 0x5c60 ADD @r3+0x9700 r4 */
void step_coeff_calc_add_at_r3_plus_0x9700_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9700));
    mcu.pc = 0x5c64;
}

/* 0x5c64 BPL 20 -> 0x5c7a */
void step_coeff_calc_bpl_20_to_0x5c7a(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5c66 : 0x5c7a;
}

/* 0x5c66 NEG r4 */
void step_coeff_calc_neg_r4_x(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5c68;
}

/* 0x5c68 cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_d(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5c6b;
}

/* 0x5c6b BCS 3 -> 0x5c70 */
void step_coeff_calc_bcs_3_to_0x5c70(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5c70 : 0x5c6d;
}

/* 0x5c6d movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_d(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5c70;
}

/* 0x5c70 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_40(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c72;
}

/* 0x5c72 MULXU #0xc30d r4:r5 */
void step_coeff_calc_mulxu_0xc30d_r4_r5(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xc30d, r4, r5);
    mcu.pc = 0x5c76;
}

/* 0x5c76 NEG r4 */
void step_coeff_calc_neg_r4_y(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5c78;
}

/* 0x5c78 BRA 14 -> 0x5c88 */
void step_coeff_calc_bra_14_to_0x5c88(void)
{
    mcu.pc = 0x5c88;
}

/* 0x5c7a cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_e(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5c7d;
}

/* 0x5c7d BCS 3 -> 0x5c82 */
void step_coeff_calc_bcs_3_to_0x5c82(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5c82 : 0x5c7f;
}

/* 0x5c7f movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_e(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5c82;
}

/* 0x5c82 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_41(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5c84;
}

/* 0x5c84 MULXU #0xc30d r4:r5 */
void step_coeff_calc_mulxu_0xc30d_r4_r5_a(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xc30d, r4, r5);
    mcu.pc = 0x5c88;
}

/* 0x5c88 MOVG3 r4 -> @r0+0x0094 */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x94(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x94), r4);
    mcu.pc = 0x5c8c;
}

/* 0x5c8c MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_g(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5c90;
}

/* 0x5c90 MOVG2 @r2+82 r4 */
void step_coeff_calc_movg2_at_r2_plus_82_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 82));
    mcu.pc = 0x5c93;
}

/* 0x5c93 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_k(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5c97;
}

/* 0x5c97 SHLR r4 */
void step_coeff_calc_shlr_r4_n(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5c99;
}

/* 0x5c99 SHLR r4 */
void step_coeff_calc_shlr_r4_o(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5c9b;
}

/* 0x5c9b ADD @r3+0x9100 r4 */
void step_coeff_calc_add_at_r3_plus_0x9100_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9100));
    mcu.pc = 0x5c9f;
}

/* 0x5c9f ADD @r3+0x9260 r4 */
void step_coeff_calc_add_at_r3_plus_0x9260_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9260));
    mcu.pc = 0x5ca3;
}

/* 0x5ca3 ADD @r3+0x93c0 r4 */
void step_coeff_calc_add_at_r3_plus_0x93c0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x93c0));
    mcu.pc = 0x5ca7;
}

/* 0x5ca7 ADD @r3+0x9520 r4 */
void step_coeff_calc_add_at_r3_plus_0x9520_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9520));
    mcu.pc = 0x5cab;
}

/* 0x5cab ADD @r3+0x9680 r4 */
void step_coeff_calc_add_at_r3_plus_0x9680_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9680));
    mcu.pc = 0x5caf;
}

/* 0x5caf BPL 20 -> 0x5cc5 */
void step_coeff_calc_bpl_20_to_0x5cc5(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5cb1 : 0x5cc5;
}

/* 0x5cb1 NEG r4 */
void step_coeff_calc_neg_r4_z(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5cb3;
}

/* 0x5cb3 cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_f(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5cb6;
}

/* 0x5cb6 BCS 3 -> 0x5cbb */
void step_coeff_calc_bcs_3_to_0x5cbb(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5cbb : 0x5cb8;
}

/* 0x5cb8 movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_f(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5cbb;
}

/* 0x5cbb ADD r4 r4 */
void step_coeff_calc_add_r4_r4_42(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5cbd;
}

/* 0x5cbd MULXU #0xc30d r4:r5 */
void step_coeff_calc_mulxu_0xc30d_r4_r5_b(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xc30d, r4, r5);
    mcu.pc = 0x5cc1;
}

/* 0x5cc1 NEG r4 */
void step_coeff_calc_neg_r4_27(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5cc3;
}

/* 0x5cc3 BRA 14 -> 0x5cd3 */
void step_coeff_calc_bra_14_to_0x5cd3(void)
{
    mcu.pc = 0x5cd3;
}

/* 0x5cc5 cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_g(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5cc8;
}

/* 0x5cc8 BCS 3 -> 0x5ccd */
void step_coeff_calc_bcs_3_to_0x5ccd(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5ccd : 0x5cca;
}

/* 0x5cca movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_g(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5ccd;
}

/* 0x5ccd ADD r4 r4 */
void step_coeff_calc_add_r4_r4_43(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5ccf;
}

/* 0x5ccf MULXU #0xc30d r4:r5 */
void step_coeff_calc_mulxu_0xc30d_r4_r5_c(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xc30d, r4, r5);
    mcu.pc = 0x5cd3;
}

/* 0x5cd3 MOVG3 r4 -> @r0+0x008c */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x8c(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x8c), r4);
    mcu.pc = 0x5cd7;
}

/* 0x5cd7 MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_h(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5cdb;
}

/* 0x5cdb MOVG2 @r2+85 r4 */
void step_coeff_calc_movg2_at_r2_plus_85_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 85));
    mcu.pc = 0x5cde;
}

/* 0x5cde MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_l(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5ce2;
}

/* 0x5ce2 SHLR r4 */
void step_coeff_calc_shlr_r4_p(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5ce4;
}

/* 0x5ce4 SHLR r4 */
void step_coeff_calc_shlr_r4_q(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5ce6;
}

/* 0x5ce6 ADD @r3+0x9160 r4 */
void step_coeff_calc_add_at_r3_plus_0x9160_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9160));
    mcu.pc = 0x5cea;
}

/* 0x5cea ADD @r3+0x92c0 r4 */
void step_coeff_calc_add_at_r3_plus_0x92c0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x92c0));
    mcu.pc = 0x5cee;
}

/* 0x5cee ADD @r3+0x9420 r4 */
void step_coeff_calc_add_at_r3_plus_0x9420_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9420));
    mcu.pc = 0x5cf2;
}

/* 0x5cf2 ADD @r3+0x9580 r4 */
void step_coeff_calc_add_at_r3_plus_0x9580_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9580));
    mcu.pc = 0x5cf6;
}

/* 0x5cf6 ADD @r3+0x96e0 r4 */
void step_coeff_calc_add_at_r3_plus_0x96e0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x96e0));
    mcu.pc = 0x5cfa;
}

/* 0x5cfa BPL 20 -> 0x5d10 */
void step_coeff_calc_bpl_20_to_0x5d10(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5cfc : 0x5d10;
}

/* 0x5cfc NEG r4 */
void step_coeff_calc_neg_r4_28(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5cfe;
}

/* 0x5cfe cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_h(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5d01;
}

/* 0x5d01 BCS 3 -> 0x5d06 */
void step_coeff_calc_bcs_3_to_0x5d06(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5d06 : 0x5d03;
}

/* 0x5d03 movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_h(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5d06;
}

/* 0x5d06 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_44(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5d08;
}

/* 0x5d08 MULXU #0xbe7a r4:r5 */
void step_coeff_calc_mulxu_0xbe7a_r4_r5(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xbe7a, r4, r5);
    mcu.pc = 0x5d0c;
}

/* 0x5d0c NEG r4 */
void step_coeff_calc_neg_r4_29(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5d0e;
}

/* 0x5d0e BRA 14 -> 0x5d1e */
void step_coeff_calc_bra_14_to_0x5d1e(void)
{
    mcu.pc = 0x5d1e;
}

/* 0x5d10 cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_i(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5d13;
}

/* 0x5d13 BCS 3 -> 0x5d18 */
void step_coeff_calc_bcs_3_to_0x5d18(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5d18 : 0x5d15;
}

/* 0x5d15 movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_i(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5d18;
}

/* 0x5d18 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_45(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5d1a;
}

/* 0x5d1a MULXU #0xbe7a r4:r5 */
void step_coeff_calc_mulxu_0xbe7a_r4_r5_a(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xbe7a, r4, r5);
    mcu.pc = 0x5d1e;
}

/* 0x5d1e MOVG3 r4 -> @r0+0x0092 */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x92(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x92), r4);
    mcu.pc = 0x5d22;
}

/* 0x5d22 MOVG2 (dp,0xd17e) r2 */
void step_coeff_calc_movg2_dp_0xd17e_r2_i(void)
{
    uint16_t &r2 = mcu.r[2];

    load16(r2, dp_addr(0xd17e));
    mcu.pc = 0x5d26;
}

/* 0x5d26 MOVG2 @r2+81 r4 */
void step_coeff_calc_movg2_at_r2_plus_81_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    load8(r4, ind_addr(2, 81));
    mcu.pc = 0x5d29;
}

/* 0x5d29 MULXU (dp,0xd180) r4 */
void step_coeff_calc_mulxu_dp_0xd180_r4_m(void)
{
    uint16_t &r4 = mcu.r[4];

    mulxu8_mem(dp_addr(0xd180), r4);
    mcu.pc = 0x5d2d;
}

/* 0x5d2d SHLR r4 */
void step_coeff_calc_shlr_r4_r(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5d2f;
}

/* 0x5d2f SHLR r4 */
void step_coeff_calc_shlr_r4_s(void)
{
    uint16_t &r4 = mcu.r[4];

    shlr16(r4);
    mcu.pc = 0x5d31;
}

/* 0x5d31 ADD @r3+0x90e0 r4 */
void step_coeff_calc_add_at_r3_plus_0x90e0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x90e0));
    mcu.pc = 0x5d35;
}

/* 0x5d35 ADD @r3+0x9240 r4 */
void step_coeff_calc_add_at_r3_plus_0x9240_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9240));
    mcu.pc = 0x5d39;
}

/* 0x5d39 ADD @r3+0x93a0 r4 */
void step_coeff_calc_add_at_r3_plus_0x93a0_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x93a0));
    mcu.pc = 0x5d3d;
}

/* 0x5d3d ADD @r3+0x9500 r4 */
void step_coeff_calc_add_at_r3_plus_0x9500_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9500));
    mcu.pc = 0x5d41;
}

/* 0x5d41 ADD @r3+0x9660 r4 */
void step_coeff_calc_add_at_r3_plus_0x9660_r4(void)
{
    uint16_t &r4 = mcu.r[4];

    add16_mem(r4, ind_addr(3, 0x9660));
    mcu.pc = 0x5d45;
}

/* 0x5d45 BPL 20 -> 0x5d5b */
void step_coeff_calc_bpl_20_to_0x5d5b(void)
{
    mcu.pc = (mcu.sr & STATUS_N) ? 0x5d47 : 0x5d5b;
}

/* 0x5d47 NEG r4 */
void step_coeff_calc_neg_r4_30(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5d49;
}

/* 0x5d49 cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_j(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5d4c;
}

/* 0x5d4c BCS 3 -> 0x5d51 */
void step_coeff_calc_bcs_3_to_0x5d51(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5d51 : 0x5d4e;
}

/* 0x5d4e movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_j(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5d51;
}

/* 0x5d51 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_46(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5d53;
}

/* 0x5d53 MULXU #0xbe7a r4:r5 */
void step_coeff_calc_mulxu_0xbe7a_r4_r5_b(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xbe7a, r4, r5);
    mcu.pc = 0x5d57;
}

/* 0x5d57 NEG r4 */
void step_coeff_calc_neg_r4_31(void)
{
    uint16_t &r4 = mcu.r[4];

    neg16(r4);
    mcu.pc = 0x5d59;
}

/* 0x5d59 BRA 14 -> 0x5d69 */
void step_coeff_calc_bra_14_to_0x5d69(void)
{
    mcu.pc = 0x5d69;
}

/* 0x5d5b cmp r4,w #0x0fc0 */
void step_coeff_calc_cmp_r4_w_0xfc0_k(void)
{
    uint16_t &r4 = mcu.r[4];

    cmp16(r4, 0x0fc0);
    mcu.pc = 0x5d5e;
}

/* 0x5d5e BCS 3 -> 0x5d63 */
void step_coeff_calc_bcs_3_to_0x5d63(void)
{
    mcu.pc = (mcu.sr & STATUS_C) ? 0x5d63 : 0x5d60;
}

/* 0x5d60 movi r4 #0x0fc0 */
void step_coeff_calc_movi_r4_0xfc0_k(void)
{
    uint16_t &r4 = mcu.r[4];

    movi16(r4, 0x0fc0);
    mcu.pc = 0x5d63;
}

/* 0x5d63 ADD r4 r4 */
void step_coeff_calc_add_r4_r4_47(void)
{
    uint16_t &r4 = mcu.r[4];

    add16(r4, r4);
    mcu.pc = 0x5d65;
}

/* 0x5d65 MULXU #0xbe7a r4:r5 */
void step_coeff_calc_mulxu_0xbe7a_r4_r5_c(void)
{
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];

    mulxu16_imm(0xbe7a, r4, r5);
    mcu.pc = 0x5d69;
}

/* 0x5d69 MOVG3 r4 -> @r0+0x0090 */
void step_coeff_calc_movg3_r4_to_at_r0_plus_0x90(void)
{
    uint16_t &r4 = mcu.r[4];

    store16(ind_addr(0, 0x90), r4);
    mcu.pc = 0x5d6d;
}

/* 0x5d6d rts */
void step_coeff_calc_rts_a(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */
} /* namespace mk2c */

void MK2CPP_VoiceMaterializeFillTables(void)
{
    /* pcm_enable.cpp calls this directly and the module self-registration at the
     * bottom calls it again through mk2c::hand_fill_modules(); register once, as
     * mk2cpp.cpp aborts on a duplicate flat. */
    static bool registered = false;
    if (registered)
        return;
    registered = true;

    /* materialize: 43 PCs */
    MK2CPP_HandRegister(0x000053ebu, &mk2c::step_materialize_movg2_r1_r3);
    MK2CPP_HandRegister(0x000053edu, &mk2c::step_materialize_add_r3_r3);
    MK2CPP_HandRegister(0x000053efu, &mk2c::step_materialize_movg2_at_r3_plus_0x64d6_r0);
    MK2CPP_HandRegister(0x000053f3u, &mk2c::step_materialize_movg3_r1_to_at_r0_minus_2);
    MK2CPP_HandRegister(0x000053f6u, &mk2c::step_materialize_movg2_at_r1_plus_0xcfe4_r6);
    MK2CPP_HandRegister(0x000053fau, &mk2c::step_materialize_movg3_r6_to_at_r0_plus_0x98);
    MK2CPP_HandRegister(0x000053feu, &mk2c::step_materialize_movg2_at_r1_plus_0xd038_r6);
    MK2CPP_HandRegister(0x00005402u, &mk2c::step_materialize_movg3_r6_to_at_r0_plus_0x99);
    MK2CPP_HandRegister(0x00005406u, &mk2c::step_materialize_movg2_at_r1_plus_0xd08c_r6);
    MK2CPP_HandRegister(0x0000540au, &mk2c::step_materialize_movg3_r6_to_at_r0_plus_0x9a);
    MK2CPP_HandRegister(0x0000540eu, &mk2c::step_materialize_movg2_at_r3_plus_0xcfac_r6);
    MK2CPP_HandRegister(0x00005412u, &mk2c::step_materialize_movg3_r6_to_at_r0_plus_0x9c);
    MK2CPP_HandRegister(0x00005416u, &mk2c::step_materialize_movg2_at_r3_plus_0xd000_r6);
    MK2CPP_HandRegister(0x0000541au, &mk2c::step_materialize_movg3_r6_to_at_r0_plus_0x9e);
    MK2CPP_HandRegister(0x0000541eu, &mk2c::step_materialize_movg2_at_r3_plus_0xd054_r6);
    MK2CPP_HandRegister(0x00005422u, &mk2c::step_materialize_movg3_r6_to_at_r0_plus_0xa0);
    MK2CPP_HandRegister(0x00005426u, &mk2c::step_materialize_movg2_at_r1_plus_0xcf90_r6);
    MK2CPP_HandRegister(0x0000542au, &mk2c::step_materialize_movg3_r6_to_at_r0_minus_59);
    MK2CPP_HandRegister(0x0000542du, &mk2c::step_materialize_btsti_r6_7);
    MK2CPP_HandRegister(0x0000542fu, &mk2c::step_materialize_beq_5_to_0x5436);
    MK2CPP_HandRegister(0x00005431u, &mk2c::step_materialize_movg_0xff_to_at_r1_plus_0xad0e);
    MK2CPP_HandRegister(0x00005436u, &mk2c::step_materialize_clr_r3);
    MK2CPP_HandRegister(0x00005438u, &mk2c::step_materialize_movg2_at_r1_plus_0xce78_r3);
    MK2CPP_HandRegister(0x0000543cu, &mk2c::step_materialize_movg3_r3_to_at_r0_plus_0x9b);
    MK2CPP_HandRegister(0x00005440u, &mk2c::step_materialize_add_r3_r3_a);
    MK2CPP_HandRegister(0x00005442u, &mk2c::step_materialize_movg2_at_r3_plus_0x7218_r3);
    MK2CPP_HandRegister(0x00005446u, &mk2c::step_materialize_movg3_r3_to_at_r0_plus_46);
    MK2CPP_HandRegister(0x00005449u, &mk2c::step_materialize_clr_r3_a);
    MK2CPP_HandRegister(0x0000544bu, &mk2c::step_materialize_tst_at_r1_plus_0xd0fc);
    MK2CPP_HandRegister(0x0000544fu, &mk2c::step_materialize_beq_9_to_0x545a);
    MK2CPP_HandRegister(0x00005451u, &mk2c::step_materialize_bmi_22_to_0x5469);
    MK2CPP_HandRegister(0x00005453u, &mk2c::step_materialize_movi_r3_0x8bd4);
    MK2CPP_HandRegister(0x00005456u, &mk2c::step_materialize_bsr_21_to_0x546d);
    MK2CPP_HandRegister(0x00005458u, &mk2c::step_materialize_bra_5_to_0x545f);
    MK2CPP_HandRegister(0x0000545au, &mk2c::step_materialize_movi_r3_0x8748);
    MK2CPP_HandRegister(0x0000545du, &mk2c::step_materialize_bsr_14_to_0x546d);
    MK2CPP_HandRegister(0x0000545fu, &mk2c::step_materialize_clr_r6);
    MK2CPP_HandRegister(0x00005461u, &mk2c::step_materialize_movg2_at_r1_plus_0xcecc_r6);
    MK2CPP_HandRegister(0x00005465u, &mk2c::step_materialize_add_r6_r3);
    MK2CPP_HandRegister(0x00005467u, &mk2c::step_materialize_bra_0_to_0x5469);
    MK2CPP_HandRegister(0x00005469u, &mk2c::step_materialize_movg3_r3_to_at_r0_plus_48);
    MK2CPP_HandRegister(0x0000546cu, &mk2c::step_materialize_rts);
    MK2CPP_HandRegister(0x0000546du, &mk2c::step_materialize_rts_a);

    /* coeff_copy: 23 PCs */
    MK2CPP_HandRegister(0x00005d6eu, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x86_r6);
    MK2CPP_HandRegister(0x00005d72u, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x86);
    MK2CPP_HandRegister(0x00005d76u, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x8a_r6);
    MK2CPP_HandRegister(0x00005d7au, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x8a);
    MK2CPP_HandRegister(0x00005d7eu, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x88_r6);
    MK2CPP_HandRegister(0x00005d82u, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x88);
    MK2CPP_HandRegister(0x00005d86u, &mk2c::step_coeff_copy_movg2_at_r2_minus_80_r6);
    MK2CPP_HandRegister(0x00005d89u, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_minus_80);
    MK2CPP_HandRegister(0x00005d8cu, &mk2c::step_coeff_copy_movg2_at_r2_minus_114_r6);
    MK2CPP_HandRegister(0x00005d8fu, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_minus_114);
    MK2CPP_HandRegister(0x00005d92u, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x96_r6);
    MK2CPP_HandRegister(0x00005d96u, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x96);
    MK2CPP_HandRegister(0x00005d9au, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x8e_r6);
    MK2CPP_HandRegister(0x00005d9eu, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x8e);
    MK2CPP_HandRegister(0x00005da2u, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x94_r6);
    MK2CPP_HandRegister(0x00005da6u, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x94);
    MK2CPP_HandRegister(0x00005daau, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x8c_r6);
    MK2CPP_HandRegister(0x00005daeu, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x8c);
    MK2CPP_HandRegister(0x00005db2u, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x92_r6);
    MK2CPP_HandRegister(0x00005db6u, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x92);
    MK2CPP_HandRegister(0x00005dbau, &mk2c::step_coeff_copy_movg2_at_r2_plus_0x90_r6);
    MK2CPP_HandRegister(0x00005dbeu, &mk2c::step_coeff_copy_movg3_r6_to_at_r0_plus_0x90);
    MK2CPP_HandRegister(0x00005dc2u, &mk2c::step_coeff_copy_rts);

    /* tone_fields: 58 PCs */
    MK2CPP_HandRegister(0x00003580u, &mk2c::step_tone_fields_ldc_at_r0_plus_0x99_r4);
    MK2CPP_HandRegister(0x00003584u, &mk2c::step_tone_fields_movg2_at_r0_plus_0x9e_r5);
    MK2CPP_HandRegister(0x00003588u, &mk2c::step_tone_fields_clr_r2);
    MK2CPP_HandRegister(0x0000358au, &mk2c::step_tone_fields_movg2_at_r5_plus_73_r2);
    MK2CPP_HandRegister(0x0000358du, &mk2c::step_tone_fields_bpl_9_to_0x3598);
    MK2CPP_HandRegister(0x0000358fu, &mk2c::step_tone_fields_and_0x7f_r2);
    MK2CPP_HandRegister(0x00003592u, &mk2c::step_tone_fields_swap_r2);
    MK2CPP_HandRegister(0x00003594u, &mk2c::step_tone_fields_neg_r2);
    MK2CPP_HandRegister(0x00003596u, &mk2c::step_tone_fields_bra_2_to_0x359a);
    MK2CPP_HandRegister(0x00003598u, &mk2c::step_tone_fields_swap_r2_a);
    MK2CPP_HandRegister(0x0000359au, &mk2c::step_tone_fields_movg3_r2_to_at_r0_minus_94);
    MK2CPP_HandRegister(0x0000359du, &mk2c::step_tone_fields_clr_r2_a);
    MK2CPP_HandRegister(0x0000359fu, &mk2c::step_tone_fields_movg2_at_r5_plus_72_r2);
    MK2CPP_HandRegister(0x000035a2u, &mk2c::step_tone_fields_bpl_9_to_0x35ad);
    MK2CPP_HandRegister(0x000035a4u, &mk2c::step_tone_fields_and_0x7f_r2_a);
    MK2CPP_HandRegister(0x000035a7u, &mk2c::step_tone_fields_swap_r2_b);
    MK2CPP_HandRegister(0x000035a9u, &mk2c::step_tone_fields_neg_r2_a);
    MK2CPP_HandRegister(0x000035abu, &mk2c::step_tone_fields_bra_2_to_0x35af);
    MK2CPP_HandRegister(0x000035adu, &mk2c::step_tone_fields_swap_r2_c);
    MK2CPP_HandRegister(0x000035afu, &mk2c::step_tone_fields_movg3_r2_to_at_r0_minus_128);
    MK2CPP_HandRegister(0x000035b2u, &mk2c::step_tone_fields_clr_r2_b);
    MK2CPP_HandRegister(0x000035b4u, &mk2c::step_tone_fields_movg2_at_r5_plus_43_r2);
    MK2CPP_HandRegister(0x000035b7u, &mk2c::step_tone_fields_bpl_13_to_0x35c6);
    MK2CPP_HandRegister(0x000035b9u, &mk2c::step_tone_fields_and_0x7f_r2_b);
    MK2CPP_HandRegister(0x000035bcu, &mk2c::step_tone_fields_add_r2_r2);
    MK2CPP_HandRegister(0x000035beu, &mk2c::step_tone_fields_movg2_at_r2_plus_0x6f86_r2);
    MK2CPP_HandRegister(0x000035c2u, &mk2c::step_tone_fields_neg_r2_b);
    MK2CPP_HandRegister(0x000035c4u, &mk2c::step_tone_fields_bra_6_to_0x35cc);
    MK2CPP_HandRegister(0x000035c6u, &mk2c::step_tone_fields_add_r2_r2_a);
    MK2CPP_HandRegister(0x000035c8u, &mk2c::step_tone_fields_movg2_at_r2_plus_0x6f86_r2_a);
    MK2CPP_HandRegister(0x000035ccu, &mk2c::step_tone_fields_movg3_r2_to_at_r0_minus_92);
    MK2CPP_HandRegister(0x000035cfu, &mk2c::step_tone_fields_clr_r2_c);
    MK2CPP_HandRegister(0x000035d1u, &mk2c::step_tone_fields_movg2_at_r5_plus_42_r2);
    MK2CPP_HandRegister(0x000035d4u, &mk2c::step_tone_fields_bpl_13_to_0x35e3);
    MK2CPP_HandRegister(0x000035d6u, &mk2c::step_tone_fields_and_0x7f_r2_c);
    MK2CPP_HandRegister(0x000035d9u, &mk2c::step_tone_fields_add_r2_r2_b);
    MK2CPP_HandRegister(0x000035dbu, &mk2c::step_tone_fields_movg2_at_r2_plus_0x6f86_r2_b);
    MK2CPP_HandRegister(0x000035dfu, &mk2c::step_tone_fields_neg_r2_c);
    MK2CPP_HandRegister(0x000035e1u, &mk2c::step_tone_fields_bra_6_to_0x35e9);
    MK2CPP_HandRegister(0x000035e3u, &mk2c::step_tone_fields_add_r2_r2_c);
    MK2CPP_HandRegister(0x000035e5u, &mk2c::step_tone_fields_movg2_at_r2_plus_0x6f86_r2_c);
    MK2CPP_HandRegister(0x000035e9u, &mk2c::step_tone_fields_movg3_r2_to_at_r0_minus_126);
    MK2CPP_HandRegister(0x000035ecu, &mk2c::step_tone_fields_clr_r2_d);
    MK2CPP_HandRegister(0x000035eeu, &mk2c::step_tone_fields_movg2_at_r5_plus_15_r2);
    MK2CPP_HandRegister(0x000035f1u, &mk2c::step_tone_fields_tst_r2);
    MK2CPP_HandRegister(0x000035f3u, &mk2c::step_tone_fields_bpl_13_to_0x3602);
    MK2CPP_HandRegister(0x000035f5u, &mk2c::step_tone_fields_and_0x7f_r2_d);
    MK2CPP_HandRegister(0x000035f8u, &mk2c::step_tone_fields_add_r2_r2_d);
    MK2CPP_HandRegister(0x000035fau, &mk2c::step_tone_fields_movg2_at_r2_plus_0x7086_r2);
    MK2CPP_HandRegister(0x000035feu, &mk2c::step_tone_fields_neg_r2_d);
    MK2CPP_HandRegister(0x00003600u, &mk2c::step_tone_fields_bra_6_to_0x3608);
    MK2CPP_HandRegister(0x00003602u, &mk2c::step_tone_fields_add_r2_r2_e);
    MK2CPP_HandRegister(0x00003604u, &mk2c::step_tone_fields_movg2_at_r2_plus_0x7086_r2_a);
    MK2CPP_HandRegister(0x00003608u, &mk2c::step_tone_fields_movg3_r2_to_at_r0_minus_90);
    MK2CPP_HandRegister(0x0000360bu, &mk2c::step_tone_fields_clr_r2_e);
    MK2CPP_HandRegister(0x0000360du, &mk2c::step_tone_fields_movg2_at_r5_plus_14_r2);
    MK2CPP_HandRegister(0x00003610u, &mk2c::step_tone_fields_movg3_r2_to_at_r0_plus_0xa8);
    MK2CPP_HandRegister(0x00003614u, &mk2c::step_tone_fields_rts);

    /* cross_copy: 109 PCs */
    MK2CPP_HandRegister(0x00003a9eu, &mk2c::step_cross_copy_movg2_at_r2_minus_2_r1);
    MK2CPP_HandRegister(0x00003aa1u, &mk2c::step_cross_copy_add_r1_r1);
    MK2CPP_HandRegister(0x00003aa3u, &mk2c::step_cross_copy_movg2_at_r1_plus_0xcdc6_r6);
    MK2CPP_HandRegister(0x00003aa7u, &mk2c::step_cross_copy_movg2_at_r0_minus_2_r1);
    MK2CPP_HandRegister(0x00003aaau, &mk2c::step_cross_copy_add_r1_r1_a);
    MK2CPP_HandRegister(0x00003aacu, &mk2c::step_cross_copy_movg3_r6_to_at_r1_plus_0xcdc6);
    MK2CPP_HandRegister(0x00003ab0u, &mk2c::step_cross_copy_movg2_at_r2_minus_25_r6);
    MK2CPP_HandRegister(0x00003ab3u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_25);
    MK2CPP_HandRegister(0x00003ab6u, &mk2c::step_cross_copy_movg2_at_r2_minus_112_r6);
    MK2CPP_HandRegister(0x00003ab9u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_112);
    MK2CPP_HandRegister(0x00003abcu, &mk2c::step_cross_copy_movg2_at_r2_minus_110_r6);
    MK2CPP_HandRegister(0x00003abfu, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_110);
    MK2CPP_HandRegister(0x00003ac2u, &mk2c::step_cross_copy_movg2_at_r2_minus_108_r6);
    MK2CPP_HandRegister(0x00003ac5u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_108);
    MK2CPP_HandRegister(0x00003ac8u, &mk2c::step_cross_copy_movg2_at_r2_minus_106_r6);
    MK2CPP_HandRegister(0x00003acbu, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_106);
    MK2CPP_HandRegister(0x00003aceu, &mk2c::step_cross_copy_movg2_at_r2_minus_104_r6);
    MK2CPP_HandRegister(0x00003ad1u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_104);
    MK2CPP_HandRegister(0x00003ad4u, &mk2c::step_cross_copy_movg2_at_r2_minus_102_r6);
    MK2CPP_HandRegister(0x00003ad7u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_102);
    MK2CPP_HandRegister(0x00003adau, &mk2c::step_cross_copy_movg2_at_r2_minus_100_r6);
    MK2CPP_HandRegister(0x00003addu, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_100);
    MK2CPP_HandRegister(0x00003ae0u, &mk2c::step_cross_copy_movg2_at_r2_minus_98_r6);
    MK2CPP_HandRegister(0x00003ae3u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_98);
    MK2CPP_HandRegister(0x00003ae6u, &mk2c::step_cross_copy_movg2_at_r2_minus_96_r6);
    MK2CPP_HandRegister(0x00003ae9u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_96);
    MK2CPP_HandRegister(0x00003aecu, &mk2c::step_cross_copy_movg2_at_r2_minus_115_r6);
    MK2CPP_HandRegister(0x00003aefu, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_115);
    MK2CPP_HandRegister(0x00003af2u, &mk2c::step_cross_copy_movg2_at_r2_minus_114_r6);
    MK2CPP_HandRegister(0x00003af5u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_114);
    MK2CPP_HandRegister(0x00003af8u, &mk2c::step_cross_copy_movg2_at_r0_minus_104_r6);
    MK2CPP_HandRegister(0x00003afbu, &mk2c::step_cross_copy_cmp_r6_w_0xffff);
    MK2CPP_HandRegister(0x00003afeu, &mk2c::step_cross_copy_beq_10_to_0x3b0a);
    MK2CPP_HandRegister(0x00003b00u, &mk2c::step_cross_copy_clr_at_r0_minus_122);
    MK2CPP_HandRegister(0x00003b03u, &mk2c::step_cross_copy_clr_at_r0_minus_120);
    MK2CPP_HandRegister(0x00003b06u, &mk2c::step_cross_copy_clr_at_r0_minus_118);
    MK2CPP_HandRegister(0x00003b09u, &mk2c::step_cross_copy_rts);
    MK2CPP_HandRegister(0x00003b0au, &mk2c::step_cross_copy_movg2_at_r0_minus_102_r6);
    MK2CPP_HandRegister(0x00003b0du, &mk2c::step_cross_copy_cmp_r6_w_0xffff_a);
    MK2CPP_HandRegister(0x00003b10u, &mk2c::step_cross_copy_beq_0x8f_to_0x3ba2);
    MK2CPP_HandRegister(0x00003b13u, &mk2c::step_cross_copy_movg2_at_r0_minus_128_r2);
    MK2CPP_HandRegister(0x00003b16u, &mk2c::step_cross_copy_bpl_8_to_0x3b20);
    MK2CPP_HandRegister(0x00003b18u, &mk2c::step_cross_copy_neg_r2);
    MK2CPP_HandRegister(0x00003b1au, &mk2c::step_cross_copy_mulxu_r6_r2_r3);
    MK2CPP_HandRegister(0x00003b1cu, &mk2c::step_cross_copy_neg_r2_a);
    MK2CPP_HandRegister(0x00003b1eu, &mk2c::step_cross_copy_bra_2_to_0x3b22);
    MK2CPP_HandRegister(0x00003b20u, &mk2c::step_cross_copy_mulxu_r6_r2_r3_a);
    MK2CPP_HandRegister(0x00003b22u, &mk2c::step_cross_copy_movg3_r2_to_at_r0_minus_122);
    MK2CPP_HandRegister(0x00003b25u, &mk2c::step_cross_copy_movg2_at_r0_minus_126_r2);
    MK2CPP_HandRegister(0x00003b28u, &mk2c::step_cross_copy_bpl_8_to_0x3b32);
    MK2CPP_HandRegister(0x00003b2au, &mk2c::step_cross_copy_neg_r2_b);
    MK2CPP_HandRegister(0x00003b2cu, &mk2c::step_cross_copy_mulxu_r6_r2_r3_b);
    MK2CPP_HandRegister(0x00003b2eu, &mk2c::step_cross_copy_neg_r2_c);
    MK2CPP_HandRegister(0x00003b30u, &mk2c::step_cross_copy_bra_2_to_0x3b34);
    MK2CPP_HandRegister(0x00003b32u, &mk2c::step_cross_copy_mulxu_r6_r2_r3_c);
    MK2CPP_HandRegister(0x00003b34u, &mk2c::step_cross_copy_movg3_r2_to_at_r0_minus_120);
    MK2CPP_HandRegister(0x00003b37u, &mk2c::step_cross_copy_clr_r3);
    MK2CPP_HandRegister(0x00003b39u, &mk2c::step_cross_copy_movg2_at_r0_plus_46_r2);
    MK2CPP_HandRegister(0x00003b3cu, &mk2c::step_cross_copy_movg2_at_r2_plus_17_r3);
    MK2CPP_HandRegister(0x00003b3fu, &mk2c::step_cross_copy_movg2_at_r0_plus_0xa8_r2);
    MK2CPP_HandRegister(0x00003b43u, &mk2c::step_cross_copy_tst_r2);
    MK2CPP_HandRegister(0x00003b45u, &mk2c::step_cross_copy_bpl_10_to_0x3b51);
    MK2CPP_HandRegister(0x00003b47u, &mk2c::step_cross_copy_and_0x7f_r2);
    MK2CPP_HandRegister(0x00003b4au, &mk2c::step_cross_copy_sub_0x40_r3);
    MK2CPP_HandRegister(0x00003b4du, &mk2c::step_cross_copy_bcc_41_to_0x3b78);
    MK2CPP_HandRegister(0x00003b4fu, &mk2c::step_cross_copy_bra_27_to_0x3b6c);
    MK2CPP_HandRegister(0x00003b51u, &mk2c::step_cross_copy_sub_0x40_r3_a);
    MK2CPP_HandRegister(0x00003b54u, &mk2c::step_cross_copy_bcc_12_to_0x3b62);
    MK2CPP_HandRegister(0x00003b56u, &mk2c::step_cross_copy_neg_r3);
    MK2CPP_HandRegister(0x00003b58u, &mk2c::step_cross_copy_add_r3_r3);
    MK2CPP_HandRegister(0x00003b5au, &mk2c::step_cross_copy_sub_r3_r2);
    MK2CPP_HandRegister(0x00003b5cu, &mk2c::step_cross_copy_bpl_53_to_0x3b93);
    MK2CPP_HandRegister(0x00003b5eu, &mk2c::step_cross_copy_clr_r2);
    MK2CPP_HandRegister(0x00003b60u, &mk2c::step_cross_copy_bra_49_to_0x3b93);
    MK2CPP_HandRegister(0x00003b62u, &mk2c::step_cross_copy_add_r3_r3_a);
    MK2CPP_HandRegister(0x00003b64u, &mk2c::step_cross_copy_add_r3_r2);
    MK2CPP_HandRegister(0x00003b66u, &mk2c::step_cross_copy_bpl_43_to_0x3b93);
    MK2CPP_HandRegister(0x00003b68u, &mk2c::step_cross_copy_move_r2_0x7f);
    MK2CPP_HandRegister(0x00003b6au, &mk2c::step_cross_copy_bra_39_to_0x3b93);
    MK2CPP_HandRegister(0x00003b6cu, &mk2c::step_cross_copy_neg_r3_a);
    MK2CPP_HandRegister(0x00003b6eu, &mk2c::step_cross_copy_add_r3_r3_b);
    MK2CPP_HandRegister(0x00003b70u, &mk2c::step_cross_copy_add_r3_r2_a);
    MK2CPP_HandRegister(0x00003b72u, &mk2c::step_cross_copy_bpl_12_to_0x3b80);
    MK2CPP_HandRegister(0x00003b74u, &mk2c::step_cross_copy_move_r2_0x7f_a);
    MK2CPP_HandRegister(0x00003b76u, &mk2c::step_cross_copy_bra_8_to_0x3b80);
    MK2CPP_HandRegister(0x00003b78u, &mk2c::step_cross_copy_add_r3_r3_c);
    MK2CPP_HandRegister(0x00003b7au, &mk2c::step_cross_copy_sub_r3_r2_a);
    MK2CPP_HandRegister(0x00003b7cu, &mk2c::step_cross_copy_bpl_2_to_0x3b80);
    MK2CPP_HandRegister(0x00003b7eu, &mk2c::step_cross_copy_clr_r2_a);
    MK2CPP_HandRegister(0x00003b80u, &mk2c::step_cross_copy_add_r2_r2);
    MK2CPP_HandRegister(0x00003b82u, &mk2c::step_cross_copy_movg2_at_r2_plus_0x7086_r2);
    MK2CPP_HandRegister(0x00003b86u, &mk2c::step_cross_copy_neg_r2_d);
    MK2CPP_HandRegister(0x00003b88u, &mk2c::step_cross_copy_movg3_r2_to_at_r0_minus_124);
    MK2CPP_HandRegister(0x00003b8bu, &mk2c::step_cross_copy_neg_r2_e);
    MK2CPP_HandRegister(0x00003b8du, &mk2c::step_cross_copy_mulxu_r6_r2_r3_d);
    MK2CPP_HandRegister(0x00003b8fu, &mk2c::step_cross_copy_neg_r2_f);
    MK2CPP_HandRegister(0x00003b91u, &mk2c::step_cross_copy_bra_11_to_0x3b9e);
    MK2CPP_HandRegister(0x00003b93u, &mk2c::step_cross_copy_add_r2_r2_a);
    MK2CPP_HandRegister(0x00003b95u, &mk2c::step_cross_copy_movg2_at_r2_plus_0x7086_r2_a);
    MK2CPP_HandRegister(0x00003b99u, &mk2c::step_cross_copy_movg3_r2_to_at_r0_minus_124_a);
    MK2CPP_HandRegister(0x00003b9cu, &mk2c::step_cross_copy_mulxu_r6_r2_r3_e);
    MK2CPP_HandRegister(0x00003b9eu, &mk2c::step_cross_copy_movg3_r2_to_at_r0_minus_118);
    MK2CPP_HandRegister(0x00003ba1u, &mk2c::step_cross_copy_rts_a);
    MK2CPP_HandRegister(0x00003ba2u, &mk2c::step_cross_copy_movg2_at_r0_minus_128_r6);
    MK2CPP_HandRegister(0x00003ba5u, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_122);
    MK2CPP_HandRegister(0x00003ba8u, &mk2c::step_cross_copy_movg2_at_r0_minus_126_r6);
    MK2CPP_HandRegister(0x00003babu, &mk2c::step_cross_copy_movg3_r6_to_at_r0_minus_120);
    MK2CPP_HandRegister(0x00003baeu, &mk2c::step_cross_copy_movi_r6_0xffff);
    MK2CPP_HandRegister(0x00003bb1u, &mk2c::step_cross_copy_bra_minus_124_to_0x3b37);

    /* gate_setup: 86 PCs */
    MK2CPP_HandRegister(0x00003615u, &mk2c::step_gate_setup_ldc_at_r0_plus_0x98_r4);
    MK2CPP_HandRegister(0x00003619u, &mk2c::step_gate_setup_movg2_at_r0_plus_0x9c_r5);
    MK2CPP_HandRegister(0x0000361du, &mk2c::step_gate_setup_movg2_at_r0_minus_2_r1);
    MK2CPP_HandRegister(0x00003620u, &mk2c::step_gate_setup_add_r1_r1);
    MK2CPP_HandRegister(0x00003622u, &mk2c::step_gate_setup_clr_at_r1_plus_0xcdc6);
    MK2CPP_HandRegister(0x00003626u, &mk2c::step_gate_setup_clr_at_r0_minus_115);
    MK2CPP_HandRegister(0x00003629u, &mk2c::step_gate_setup_movg2_at_r5_plus_14_r6);
    MK2CPP_HandRegister(0x0000362cu, &mk2c::step_gate_setup_btsti_r6_4);
    MK2CPP_HandRegister(0x0000362eu, &mk2c::step_gate_setup_beq_54_to_0x3666);
    MK2CPP_HandRegister(0x00003630u, &mk2c::step_gate_setup_movg2_at_r0_plus_0x9b_r3);
    MK2CPP_HandRegister(0x00003634u, &mk2c::step_gate_setup_stc_r4_to_r4);
    MK2CPP_HandRegister(0x00003636u, &mk2c::step_gate_setup_movi_r1_0x1b);
    MK2CPP_HandRegister(0x00003639u, &mk2c::step_gate_setup_movg2_r1_r2);
    MK2CPP_HandRegister(0x0000363bu, &mk2c::step_gate_setup_add_r2_r2);
    MK2CPP_HandRegister(0x0000363du, &mk2c::step_gate_setup_movg2_at_r2_plus_0x64d6_r2);
    MK2CPP_HandRegister(0x00003641u, &mk2c::step_gate_setup_cmp_r0_r2);
    MK2CPP_HandRegister(0x00003643u, &mk2c::step_gate_setup_beq_25_to_0x365e);
    MK2CPP_HandRegister(0x00003645u, &mk2c::step_gate_setup_sub_at_r2_plus_0_0xc);
    MK2CPP_HandRegister(0x0000364au, &mk2c::step_gate_setup_bhi_18_to_0x365e);
    MK2CPP_HandRegister(0x0000364cu, &mk2c::step_gate_setup_cmp_at_r2_plus_0x9b_r3);
    MK2CPP_HandRegister(0x00003650u, &mk2c::step_gate_setup_bne_12_to_0x365e);
    MK2CPP_HandRegister(0x00003652u, &mk2c::step_gate_setup_cmp_at_r2_plus_0x98_r4);
    MK2CPP_HandRegister(0x00003656u, &mk2c::step_gate_setup_bne_6_to_0x365e);
    MK2CPP_HandRegister(0x00003658u, &mk2c::step_gate_setup_cmp_at_r2_plus_0x9c_r5);
    MK2CPP_HandRegister(0x0000365cu, &mk2c::step_gate_setup_beq_5_to_0x3663);
    MK2CPP_HandRegister(0x0000365eu, &mk2c::step_gate_setup_cntjmp_r1_minus_40_to_0x3639);
    MK2CPP_HandRegister(0x00003661u, &mk2c::step_gate_setup_bra_3_to_0x3666);
    MK2CPP_HandRegister(0x00003663u, &mk2c::step_gate_setup_bra_0x438_to_0x3a9e);
    MK2CPP_HandRegister(0x00003666u, &mk2c::step_gate_setup_movg2_r6_r3);
    MK2CPP_HandRegister(0x00003668u, &mk2c::step_gate_setup_and_0xc0_r3);
    MK2CPP_HandRegister(0x0000366cu, &mk2c::step_gate_setup_swap_r3);
    MK2CPP_HandRegister(0x0000366eu, &mk2c::step_gate_setup_movg3_r3_to_at_r0_minus_106);
    MK2CPP_HandRegister(0x00003671u, &mk2c::step_gate_setup_movg2_r6_r3_a);
    MK2CPP_HandRegister(0x00003673u, &mk2c::step_gate_setup_and_0xf_r3);
    MK2CPP_HandRegister(0x00003677u, &mk2c::step_gate_setup_movg2_at_r3_plus_0x7207_r3);
    MK2CPP_HandRegister(0x0000367bu, &mk2c::step_gate_setup_movg3_r3_to_at_r0_minus_108);
    MK2CPP_HandRegister(0x0000367eu, &mk2c::step_gate_setup_clr_at_r0_minus_98);
    MK2CPP_HandRegister(0x00003681u, &mk2c::step_gate_setup_clr_at_r0_minus_104);
    MK2CPP_HandRegister(0x00003684u, &mk2c::step_gate_setup_clr_at_r0_minus_102);
    MK2CPP_HandRegister(0x00003687u, &mk2c::step_gate_setup_clr_at_r0_minus_96);
    MK2CPP_HandRegister(0x0000368au, &mk2c::step_gate_setup_clr_at_r0_minus_122);
    MK2CPP_HandRegister(0x0000368du, &mk2c::step_gate_setup_clr_at_r0_minus_120);
    MK2CPP_HandRegister(0x00003690u, &mk2c::step_gate_setup_clr_at_r0_minus_118);
    MK2CPP_HandRegister(0x00003693u, &mk2c::step_gate_setup_clr_r3);
    MK2CPP_HandRegister(0x00003695u, &mk2c::step_gate_setup_movg2_at_r5_plus_16_r3);
    MK2CPP_HandRegister(0x00003698u, &mk2c::step_gate_setup_bmi_39_to_0x36c1);
    MK2CPP_HandRegister(0x0000369au, &mk2c::step_gate_setup_movg2_at_r0_plus_46_r2);
    MK2CPP_HandRegister(0x0000369du, &mk2c::step_gate_setup_movg2_at_r2_plus_23_r2);
    MK2CPP_HandRegister(0x000036a0u, &mk2c::step_gate_setup_sub_0x40_r2);
    MK2CPP_HandRegister(0x000036a3u, &mk2c::step_gate_setup_bcc_12_to_0x36b1);
    MK2CPP_HandRegister(0x000036a5u, &mk2c::step_gate_setup_neg_r2);
    MK2CPP_HandRegister(0x000036a7u, &mk2c::step_gate_setup_add_r2_r2_a);
    MK2CPP_HandRegister(0x000036a9u, &mk2c::step_gate_setup_sub_r2_r3);
    MK2CPP_HandRegister(0x000036abu, &mk2c::step_gate_setup_bpl_12_to_0x36b9);
    MK2CPP_HandRegister(0x000036adu, &mk2c::step_gate_setup_clr_r3_a);
    MK2CPP_HandRegister(0x000036afu, &mk2c::step_gate_setup_bra_8_to_0x36b9);
    MK2CPP_HandRegister(0x000036b1u, &mk2c::step_gate_setup_add_r2_r2_b);
    MK2CPP_HandRegister(0x000036b3u, &mk2c::step_gate_setup_add_r2_r3);
    MK2CPP_HandRegister(0x000036b5u, &mk2c::step_gate_setup_bpl_2_to_0x36b9);
    MK2CPP_HandRegister(0x000036b7u, &mk2c::step_gate_setup_move_r3_0x7f);
    MK2CPP_HandRegister(0x000036b9u, &mk2c::step_gate_setup_add_r3_r3);
    MK2CPP_HandRegister(0x000036bbu, &mk2c::step_gate_setup_movg2_at_r3_plus_0x6e86_r3);
    MK2CPP_HandRegister(0x000036bfu, &mk2c::step_gate_setup_bra_2_to_0x36c3);
    MK2CPP_HandRegister(0x000036c1u, &mk2c::step_gate_setup_clr_r3_b);
    MK2CPP_HandRegister(0x000036c3u, &mk2c::step_gate_setup_movg3_r3_to_at_r0_minus_112);
    MK2CPP_HandRegister(0x000036c6u, &mk2c::step_gate_setup_clr_r3_c);
    MK2CPP_HandRegister(0x000036c8u, &mk2c::step_gate_setup_movg2_at_r5_plus_17_r3);
    MK2CPP_HandRegister(0x000036cbu, &mk2c::step_gate_setup_add_r3_r3_a);
    MK2CPP_HandRegister(0x000036cdu, &mk2c::step_gate_setup_movg2_at_r3_plus_0x6e86_r3_a);
    MK2CPP_HandRegister(0x000036d1u, &mk2c::step_gate_setup_movg3_r3_to_at_r0_minus_110);
    MK2CPP_HandRegister(0x000036d4u, &mk2c::step_gate_setup_movg2_at_r5_plus_15_r3);
    MK2CPP_HandRegister(0x000036d7u, &mk2c::step_gate_setup_movg3_r3_to_at_r0_minus_25);
    MK2CPP_HandRegister(0x000036dau, &mk2c::step_gate_setup_movg2_dp_0xad2a_r6);
    MK2CPP_HandRegister(0x000036deu, &mk2c::step_gate_setup_movg3_r6_to_dp_0xad2c);
    MK2CPP_HandRegister(0x000036e2u, &mk2c::step_gate_setup_movg_0x1_to_dp_0xad2a);
    MK2CPP_HandRegister(0x000036e8u, &mk2c::step_gate_setup_bset_orc_0x700_r0);
    MK2CPP_HandRegister(0x000036ecu, &mk2c::step_gate_setup_movg_0x1e_to_br_3e);
    MK2CPP_HandRegister(0x000036f0u, &mk2c::step_gate_setup_movl_r4_at_br_34);
    MK2CPP_HandRegister(0x000036f2u, &mk2c::step_gate_setup_movlw_r4_at_br_3a);
    MK2CPP_HandRegister(0x000036f4u, &mk2c::step_gate_setup_movg3_r4_to_at_r0_minus_100);
    MK2CPP_HandRegister(0x000036f7u, &mk2c::step_gate_setup_movg3_r4_to_at_r0_minus_98);
    MK2CPP_HandRegister(0x000036fau, &mk2c::step_gate_setup_bsr_13_to_0x3709);
    MK2CPP_HandRegister(0x000036fcu, &mk2c::step_gate_setup_bclr_andc_0xf8ff_r0);
    MK2CPP_HandRegister(0x00003700u, &mk2c::step_gate_setup_movg2_dp_0xad2c_r6);
    MK2CPP_HandRegister(0x00003704u, &mk2c::step_gate_setup_movg3_r6_to_dp_0xad2a);
    MK2CPP_HandRegister(0x00003708u, &mk2c::step_gate_setup_rts);

    /* mask_set: 12 PCs */
    MK2CPP_HandRegister(0x0000546eu, &mk2c::step_mask_set_movg2_at_r0_minus_2_r1);
    MK2CPP_HandRegister(0x00005471u, &mk2c::step_mask_set_cmp_r1_w_0xf);
    MK2CPP_HandRegister(0x00005474u, &mk2c::step_mask_set_bls_10_to_0x5480);
    MK2CPP_HandRegister(0x00005476u, &mk2c::step_mask_set_sub_0x10_r1);
    MK2CPP_HandRegister(0x0000547au, &mk2c::step_mask_set_bset_dp_0xd154_r1);
    MK2CPP_HandRegister(0x0000547eu, &mk2c::step_mask_set_bra_4_to_0x5484);
    MK2CPP_HandRegister(0x00005480u, &mk2c::step_mask_set_bset_dp_0xd156_r1);
    MK2CPP_HandRegister(0x00005484u, &mk2c::step_mask_set_movg2_at_r0_minus_2_r1_a);
    MK2CPP_HandRegister(0x00005487u, &mk2c::step_mask_set_sub_at_r1_plus_0xd0e0_0x0);
    MK2CPP_HandRegister(0x0000548cu, &mk2c::step_mask_set_bne_4_to_0x5492);
    MK2CPP_HandRegister(0x0000548eu, &mk2c::step_mask_set_bra_0xffffd29d_to_0x272e);
    MK2CPP_HandRegister(0x00005491u, &mk2c::step_mask_set_rts);

    /* mask_tail: 18 PCs */
    MK2CPP_HandRegister(0x00005492u, &mk2c::step_mask_tail_movg2_at_r0_minus_2_r1);
    MK2CPP_HandRegister(0x00005495u, &mk2c::step_mask_tail_addq_2_r7);
    MK2CPP_HandRegister(0x00005497u, &mk2c::step_mask_tail_movg2_dp_0xd158_r0);
    MK2CPP_HandRegister(0x0000549bu, &mk2c::step_mask_tail_sub_at_r0_plus_0_0x18);
    MK2CPP_HandRegister(0x000054a0u, &mk2c::step_mask_tail_bne_8_to_0x54aa);
    MK2CPP_HandRegister(0x000054a2u, &mk2c::step_mask_tail_movg2_at_r0_plus_6_r6);
    MK2CPP_HandRegister(0x000054a5u, &mk2c::step_mask_tail_movg3_r6_to_at_r0_plus_0);
    MK2CPP_HandRegister(0x000054a8u, &mk2c::step_mask_tail_bra_4_to_0x54ae);
    MK2CPP_HandRegister(0x000054aau, &mk2c::step_mask_tail_clr_at_r1_plus_0xad0e);
    MK2CPP_HandRegister(0x000054aeu, &mk2c::step_mask_tail_movg2_dp_0xd15a_r0);
    MK2CPP_HandRegister(0x000054b2u, &mk2c::step_mask_tail_bmi_0xfffffd47_to_0x51fc);
    MK2CPP_HandRegister(0x000054b5u, &mk2c::step_mask_tail_sub_at_r0_plus_0_0x18_a);
    MK2CPP_HandRegister(0x000054bau, &mk2c::step_mask_tail_bne_9_to_0x54c5);
    MK2CPP_HandRegister(0x000054bcu, &mk2c::step_mask_tail_movg2_at_r0_plus_6_r6_a);
    MK2CPP_HandRegister(0x000054bfu, &mk2c::step_mask_tail_movg3_r6_to_at_r0_plus_0_a);
    MK2CPP_HandRegister(0x000054c2u, &mk2c::step_mask_tail_bra_0xfffffd37_to_0x51fc);
    MK2CPP_HandRegister(0x000054c5u, &mk2c::step_mask_tail_clr_at_r1_plus_0xad0e_a);
    MK2CPP_HandRegister(0x000054c9u, &mk2c::step_mask_tail_bra_0xfffffd30_to_0x51fc);

    /* flush_off: 13 PCs */
    MK2CPP_HandRegister(0x000054fbu, &mk2c::step_flush_off_movg2_at_r0_minus_2_r1);
    MK2CPP_HandRegister(0x000054feu, &mk2c::step_flush_off_sub_at_r1_plus_0xd0e0_0x0);
    MK2CPP_HandRegister(0x00005503u, &mk2c::step_flush_off_bne_39_to_0x552c);
    MK2CPP_HandRegister(0x00005505u, &mk2c::step_flush_off_movg2_dp_0xd154_r5);
    MK2CPP_HandRegister(0x00005509u, &mk2c::step_flush_off_movg2_dp_0xd156_r6);
    MK2CPP_HandRegister(0x0000550du, &mk2c::step_flush_off_not_r5);
    MK2CPP_HandRegister(0x0000550fu, &mk2c::step_flush_off_not_r6);
    MK2CPP_HandRegister(0x00005511u, &mk2c::step_flush_off_movg2_dp_0xd150_r3);
    MK2CPP_HandRegister(0x00005515u, &mk2c::step_flush_off_movg2_dp_0xd152_r4);
    MK2CPP_HandRegister(0x00005519u, &mk2c::step_flush_off_and_r5_r3);
    MK2CPP_HandRegister(0x0000551bu, &mk2c::step_flush_off_and_r6_r4);
    MK2CPP_HandRegister(0x0000551du, &mk2c::step_flush_off_movg3_r3_to_dp_0xd150);
    MK2CPP_HandRegister(0x00005521u, &mk2c::step_flush_off_movg3_r4_to_dp_0xd152);

    /* flush_on_a: 6 PCs */
    MK2CPP_HandRegister(0x0000564au, &mk2c::step_flush_on_a_movg2_dp_0xd150_r5);
    MK2CPP_HandRegister(0x0000564eu, &mk2c::step_flush_on_a_movg2_dp_0xd152_r6);
    MK2CPP_HandRegister(0x00005652u, &mk2c::step_flush_on_a_or_dp_0xd154_r5);
    MK2CPP_HandRegister(0x00005656u, &mk2c::step_flush_on_a_or_dp_0xd156_r6);
    MK2CPP_HandRegister(0x0000565au, &mk2c::step_flush_on_a_movg3_r5_to_dp_0xd150);
    MK2CPP_HandRegister(0x0000565eu, &mk2c::step_flush_on_a_movg3_r6_to_dp_0xd152);

    /* flush_on_b: 3 PCs */
    MK2CPP_HandRegister(0x00005668u, &mk2c::step_flush_on_b_clr_dp_0xd154);
    MK2CPP_HandRegister(0x0000566cu, &mk2c::step_flush_on_b_clr_dp_0xd156);
    MK2CPP_HandRegister(0x00005670u, &mk2c::step_flush_on_b_rts);

    /* param_write: 85 PCs */
    MK2CPP_HandRegister(0x00005533u, &mk2c::step_param_write_movg2_at_r0_minus_2_r3);
    MK2CPP_HandRegister(0x00005536u, &mk2c::step_param_write_clr_at_r3_plus_0xce3f);
    MK2CPP_HandRegister(0x0000553au, &mk2c::step_param_write_clr_at_r3_plus_0xd15c);
    MK2CPP_HandRegister(0x0000553eu, &mk2c::step_param_write_movs_r3_at_br_3e);
    MK2CPP_HandRegister(0x00005540u, &mk2c::step_param_write_movg2_at_r0_minus_10_r6);
    MK2CPP_HandRegister(0x00005543u, &mk2c::step_param_write_movsw_r6_at_br_1e);
    MK2CPP_HandRegister(0x00005545u, &mk2c::step_param_write_ldc_0x0_r5);
    MK2CPP_HandRegister(0x00005548u, &mk2c::step_param_write_movg3_r3_to_dp_0xfe6c);
    MK2CPP_HandRegister(0x0000554cu, &mk2c::step_param_write_movg3_r3_to_dp_0xfe5e);
    MK2CPP_HandRegister(0x00005550u, &mk2c::step_param_write_ldc_0x0_r5_a);
    MK2CPP_HandRegister(0x00005553u, &mk2c::step_param_write_movg2_at_r0_minus_20_r5);
    MK2CPP_HandRegister(0x00005556u, &mk2c::step_param_write_movg2_at_r0_minus_16_r6);
    MK2CPP_HandRegister(0x00005559u, &mk2c::step_param_write_movs_r5_at_br_05);
    MK2CPP_HandRegister(0x0000555bu, &mk2c::step_param_write_movsw_r6_at_br_06);
    MK2CPP_HandRegister(0x0000555du, &mk2c::step_param_write_ldc_0x0_r5_b);
    MK2CPP_HandRegister(0x00005560u, &mk2c::step_param_write_movg3_r5_to_dp_0xfe6d);
    MK2CPP_HandRegister(0x00005564u, &mk2c::step_param_write_movg3_r6_to_dp_0xfe60);
    MK2CPP_HandRegister(0x00005568u, &mk2c::step_param_write_ldc_0x0_r5_c);
    MK2CPP_HandRegister(0x0000556bu, &mk2c::step_param_write_movg2_at_r0_minus_18_r5);
    MK2CPP_HandRegister(0x0000556eu, &mk2c::step_param_write_movg2_at_r0_minus_12_r6);
    MK2CPP_HandRegister(0x00005571u, &mk2c::step_param_write_movs_r5_at_br_09);
    MK2CPP_HandRegister(0x00005573u, &mk2c::step_param_write_movsw_r6_at_br_0a);
    MK2CPP_HandRegister(0x00005575u, &mk2c::step_param_write_ldc_0x0_r5_d);
    MK2CPP_HandRegister(0x00005578u, &mk2c::step_param_write_movg3_r5_to_dp_0xfe6e);
    MK2CPP_HandRegister(0x0000557cu, &mk2c::step_param_write_movg3_r6_to_dp_0xfe62);
    MK2CPP_HandRegister(0x00005580u, &mk2c::step_param_write_ldc_0x0_r5_e);
    MK2CPP_HandRegister(0x00005583u, &mk2c::step_param_write_movg2_at_r0_minus_19_r5);
    MK2CPP_HandRegister(0x00005586u, &mk2c::step_param_write_movg2_at_r0_minus_14_r6);
    MK2CPP_HandRegister(0x00005589u, &mk2c::step_param_write_movs_r5_at_br_0d);
    MK2CPP_HandRegister(0x0000558bu, &mk2c::step_param_write_movsw_r6_at_br_0e);
    MK2CPP_HandRegister(0x0000558du, &mk2c::step_param_write_ldc_0x0_r5_f);
    MK2CPP_HandRegister(0x00005590u, &mk2c::step_param_write_movg3_r5_to_dp_0xfe6f);
    MK2CPP_HandRegister(0x00005594u, &mk2c::step_param_write_movg3_r6_to_dp_0xfe64);
    MK2CPP_HandRegister(0x00005598u, &mk2c::step_param_write_ldc_0x0_r5_g);
    MK2CPP_HandRegister(0x0000559bu, &mk2c::step_param_write_movg2_at_r0_plus_52_r6);
    MK2CPP_HandRegister(0x0000559eu, &mk2c::step_param_write_movsw_r6_at_br_12);
    MK2CPP_HandRegister(0x000055a0u, &mk2c::step_param_write_ldc_0x0_r5_h);
    MK2CPP_HandRegister(0x000055a3u, &mk2c::step_param_write_movg3_r6_to_dp_0xfe66);
    MK2CPP_HandRegister(0x000055a7u, &mk2c::step_param_write_ldc_0x0_r5_i);
    MK2CPP_HandRegister(0x000055aau, &mk2c::step_param_write_movg2_at_r0_plus_58_r6);
    MK2CPP_HandRegister(0x000055adu, &mk2c::step_param_write_movsw_r6_at_br_14);
    MK2CPP_HandRegister(0x000055afu, &mk2c::step_param_write_ldc_0x0_r5_j);
    MK2CPP_HandRegister(0x000055b2u, &mk2c::step_param_write_movg3_r6_to_dp_0xfe68);
    MK2CPP_HandRegister(0x000055b6u, &mk2c::step_param_write_ldc_0x0_r5_k);
    MK2CPP_HandRegister(0x000055b9u, &mk2c::step_param_write_movg2_at_r0_plus_104_r6);
    MK2CPP_HandRegister(0x000055bcu, &mk2c::step_param_write_swap_r6);
    MK2CPP_HandRegister(0x000055beu, &mk2c::step_param_write_movg2_at_r0_plus_102_r6);
    MK2CPP_HandRegister(0x000055c1u, &mk2c::step_param_write_movsw_r6_at_br_1c);
    MK2CPP_HandRegister(0x000055c3u, &mk2c::step_param_write_movg2_at_r0_minus_24_r6);
    MK2CPP_HandRegister(0x000055c6u, &mk2c::step_param_write_movsw_r6_at_br_1a);
    MK2CPP_HandRegister(0x000055c8u, &mk2c::step_param_write_movg2_at_r0_plus_26_r6);
    MK2CPP_HandRegister(0x000055cbu, &mk2c::step_param_write_movsw_r6_at_br_16);
    MK2CPP_HandRegister(0x000055cdu, &mk2c::step_param_write_btsti_at_r0_minus_59_7);
    MK2CPP_HandRegister(0x000055d0u, &mk2c::step_param_write_beq_54_to_0x5608);
    MK2CPP_HandRegister(0x000055d2u, &mk2c::step_param_write_tst_at_r0_plus_14);
    MK2CPP_HandRegister(0x000055d5u, &mk2c::step_param_write_beq_11_to_0x55e2);
    MK2CPP_HandRegister(0x000055d7u, &mk2c::step_param_write_movi_r4_0x2);
    MK2CPP_HandRegister(0x000055dau, &mk2c::step_param_write_movg2_at_r0_plus_30_r5);
    MK2CPP_HandRegister(0x000055ddu, &mk2c::step_param_write_movg2_at_r0_plus_72_r6);
    MK2CPP_HandRegister(0x000055e0u, &mk2c::step_param_write_bra_11_to_0x55ed);
    MK2CPP_HandRegister(0x000055e2u, &mk2c::step_param_write_movi_r4_0x0);
    MK2CPP_HandRegister(0x000055e5u, &mk2c::step_param_write_movi_r5_0xb5);
    MK2CPP_HandRegister(0x000055e8u, &mk2c::step_param_write_clr_r6);
    MK2CPP_HandRegister(0x000055eau, &mk2c::step_param_write_clr_at_r0_plus_8);
    MK2CPP_HandRegister(0x000055edu, &mk2c::step_param_write_movg3_r4_to_at_r0_plus_0);
    MK2CPP_HandRegister(0x000055f0u, &mk2c::step_param_write_movg3_r4_to_at_r0_plus_2);
    MK2CPP_HandRegister(0x000055f3u, &mk2c::step_param_write_movg3_r4_to_at_r0_plus_4);
    MK2CPP_HandRegister(0x000055f6u, &mk2c::step_param_write_movsw_r5_at_br_18);
    MK2CPP_HandRegister(0x000055f8u, &mk2c::step_param_write_movg3_r5_to_at_r0_plus_30);
    MK2CPP_HandRegister(0x000055fbu, &mk2c::step_param_write_movsw_r6_at_br_10);
    MK2CPP_HandRegister(0x000055fdu, &mk2c::step_param_write_ldc_0x0_r5_l);
    MK2CPP_HandRegister(0x00005600u, &mk2c::step_param_write_movg3_r6_to_dp_0xfe6a);
    MK2CPP_HandRegister(0x00005604u, &mk2c::step_param_write_ldc_0x0_r5_m);
    MK2CPP_HandRegister(0x00005607u, &mk2c::step_param_write_rts);
    MK2CPP_HandRegister(0x00005608u, &mk2c::step_param_write_movg2_at_r0_plus_30_r5_a);
    MK2CPP_HandRegister(0x0000560bu, &mk2c::step_param_write_movg2_at_r0_plus_72_r6_a);
    MK2CPP_HandRegister(0x0000560eu, &mk2c::step_param_write_movsw_r5_at_br_18_a);
    MK2CPP_HandRegister(0x00005610u, &mk2c::step_param_write_movsw_r6_at_br_10_a);
    MK2CPP_HandRegister(0x00005612u, &mk2c::step_param_write_ldc_0x0_r5_n);
    MK2CPP_HandRegister(0x00005615u, &mk2c::step_param_write_movg3_r6_to_dp_0xfe6a_a);
    MK2CPP_HandRegister(0x00005619u, &mk2c::step_param_write_ldc_0x0_r5_o);
    MK2CPP_HandRegister(0x0000561cu, &mk2c::step_param_write_movg2_at_r0_plus_6_r6);
    MK2CPP_HandRegister(0x0000561fu, &mk2c::step_param_write_movg3_r6_to_at_r0_plus_0);
    MK2CPP_HandRegister(0x00005622u, &mk2c::step_param_write_clr_at_r0_plus_6);
    MK2CPP_HandRegister(0x00005625u, &mk2c::step_param_write_rts_a);

    /* loop_write: 15 PCs */
    MK2CPP_HandRegister(0x00005626u, &mk2c::step_loop_write_movg2_at_r0_minus_2_r1);
    MK2CPP_HandRegister(0x00005629u, &mk2c::step_loop_write_tst_at_r0_plus_101);
    MK2CPP_HandRegister(0x0000562cu, &mk2c::step_loop_write_bne_27_to_0x5649);
    MK2CPP_HandRegister(0x0000562eu, &mk2c::step_loop_write_movs_r1_at_br_3e);
    MK2CPP_HandRegister(0x00005630u, &mk2c::step_loop_write_movl_r5_at_br_1e);
    MK2CPP_HandRegister(0x00005632u, &mk2c::step_loop_write_movlw_r5_at_br_3a);
    MK2CPP_HandRegister(0x00005634u, &mk2c::step_loop_write_btsti_r5_5);
    MK2CPP_HandRegister(0x00005636u, &mk2c::step_loop_write_beq_minus_8_to_0x5630);
    MK2CPP_HandRegister(0x00005638u, &mk2c::step_loop_write_movg_0xff00_to_br_1a);
    MK2CPP_HandRegister(0x0000563du, &mk2c::step_loop_write_movg2_at_r0_minus_22_r5);
    MK2CPP_HandRegister(0x00005640u, &mk2c::step_loop_write_shlr_r5);
    MK2CPP_HandRegister(0x00005642u, &mk2c::step_loop_write_movsw_r5_at_br_36);
    MK2CPP_HandRegister(0x00005644u, &mk2c::step_loop_write_movg2_at_r0_plus_38_r5);
    MK2CPP_HandRegister(0x00005647u, &mk2c::step_loop_write_movsw_r5_at_br_1a);
    MK2CPP_HandRegister(0x00005649u, &mk2c::step_loop_write_rts);

    /* param_ack: 20 PCs */
    MK2CPP_HandRegister(0x000054ccu, &mk2c::step_param_ack_bset_orc_0x700_r0);
    MK2CPP_HandRegister(0x000054d0u, &mk2c::step_param_ack_movg2_at_r1_minus_2_r2);
    MK2CPP_HandRegister(0x000054d3u, &mk2c::step_param_ack_movs_r2_at_br_3e);
    MK2CPP_HandRegister(0x000054d5u, &mk2c::step_param_ack_movl_r5_at_br_32);
    MK2CPP_HandRegister(0x000054d7u, &mk2c::step_param_ack_movlw_r5_at_br_3a);
    MK2CPP_HandRegister(0x000054d9u, &mk2c::step_param_ack_movl_r6_at_br_34);
    MK2CPP_HandRegister(0x000054dbu, &mk2c::step_param_ack_movlw_r6_at_br_3a);
    MK2CPP_HandRegister(0x000054ddu, &mk2c::step_param_ack_bclr_andc_0xf8ff_r0);
    MK2CPP_HandRegister(0x000054e1u, &mk2c::step_param_ack_sub_at_r2_plus_0xd0e0_0x0);
    MK2CPP_HandRegister(0x000054e6u, &mk2c::step_param_ack_beq_4_to_0x54ec);
    MK2CPP_HandRegister(0x000054e8u, &mk2c::step_param_ack_movg2_r1_r0);
    MK2CPP_HandRegister(0x000054eau, &mk2c::step_param_ack_bra_minus_90_to_0x5492);
    MK2CPP_HandRegister(0x000054ecu, &mk2c::step_param_ack_tst_r5);
    MK2CPP_HandRegister(0x000054eeu, &mk2c::step_param_ack_beq_10_to_0x54fa);
    MK2CPP_HandRegister(0x000054f0u, &mk2c::step_param_ack_tst_r6);
    MK2CPP_HandRegister(0x000054f2u, &mk2c::step_param_ack_beq_6_to_0x54fa);
    MK2CPP_HandRegister(0x000054f4u, &mk2c::step_param_ack_move_r0_0x80);
    MK2CPP_HandRegister(0x000054f6u, &mk2c::step_param_ack_trapa_0x10);
    MK2CPP_HandRegister(0x000054f8u, &mk2c::step_param_ack_bra_minus_46_to_0x54cc);
    MK2CPP_HandRegister(0x000054fau, &mk2c::step_param_ack_rts);

    /* pcm_play: 85 PCs */
    MK2CPP_HandRegister(0x000052cbu, &mk2c::step_pcm_play_sub_at_r1_plus_0xd0a8_0xff);
    MK2CPP_HandRegister(0x000052d0u, &mk2c::step_pcm_play_bne_21_to_0x52e7);
    MK2CPP_HandRegister(0x000052d2u, &mk2c::step_pcm_play_sub_at_r1_plus_0xd0c4_0xff);
    MK2CPP_HandRegister(0x000052d7u, &mk2c::step_pcm_play_beq_0xb6_to_0x5390);
    MK2CPP_HandRegister(0x000052dau, &mk2c::step_pcm_play_clr_r2);
    MK2CPP_HandRegister(0x000052dcu, &mk2c::step_pcm_play_movg2_at_r1_plus_0xd0c4_r2);
    MK2CPP_HandRegister(0x000052e0u, &mk2c::step_pcm_play_movg_0x0_to_at_r2_plus_0xd0e0);
    MK2CPP_HandRegister(0x000052e5u, &mk2c::step_pcm_play_bra_13_to_0x52f4);
    MK2CPP_HandRegister(0x000052e7u, &mk2c::step_pcm_play_clr_r2_a);
    MK2CPP_HandRegister(0x000052e9u, &mk2c::step_pcm_play_movg2_at_r1_plus_0xd0a8_r2);
    MK2CPP_HandRegister(0x000052edu, &mk2c::step_pcm_play_movg_0x0_to_at_r2_plus_0xd0e0_a);
    MK2CPP_HandRegister(0x000052f2u, &mk2c::step_pcm_play_xch_r2_r1);
    MK2CPP_HandRegister(0x000052f4u, &mk2c::step_pcm_play_bsr16_to_0x53eb);
    MK2CPP_HandRegister(0x000052f7u, &mk2c::step_pcm_play_movg3_r0_to_dp_0xd158);
    MK2CPP_HandRegister(0x000052fbu, &mk2c::step_pcm_play_movg2_r2_r1);
    MK2CPP_HandRegister(0x000052fdu, &mk2c::step_pcm_play_bsr16_to_0x53eb_a);
    MK2CPP_HandRegister(0x00005300u, &mk2c::step_pcm_play_movg3_r0_to_dp_0xd15a);
    MK2CPP_HandRegister(0x00005304u, &mk2c::step_pcm_play_bclr_andc_0xf8ff_r0);
    MK2CPP_HandRegister(0x00005308u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0);
    MK2CPP_HandRegister(0x0000530cu, &mk2c::step_pcm_play_bsr16_to_0x5998);
    MK2CPP_HandRegister(0x0000530fu, &mk2c::step_pcm_play_movg2_dp_0xd15a_r0);
    MK2CPP_HandRegister(0x00005313u, &mk2c::step_pcm_play_movg2_dp_0xd158_r2);
    MK2CPP_HandRegister(0x00005317u, &mk2c::step_pcm_play_bsr16_to_0x5d6e);
    MK2CPP_HandRegister(0x0000531au, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_a);
    MK2CPP_HandRegister(0x0000531eu, &mk2c::step_pcm_play_movg2_at_r0_minus_2_r1);
    MK2CPP_HandRegister(0x00005321u, &mk2c::step_pcm_play_btsti_at_r0_minus_59_7);
    MK2CPP_HandRegister(0x00005324u, &mk2c::step_pcm_play_beq_28_to_0x5342);
    MK2CPP_HandRegister(0x00005326u, &mk2c::step_pcm_play_bsr16_to_0x3580);
    MK2CPP_HandRegister(0x00005329u, &mk2c::step_pcm_play_movg2_dp_0xd15a_r0_a);
    MK2CPP_HandRegister(0x0000532du, &mk2c::step_pcm_play_bsr16_to_0x3580_a);
    MK2CPP_HandRegister(0x00005330u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_b);
    MK2CPP_HandRegister(0x00005334u, &mk2c::step_pcm_play_bsr16_to_0x3615);
    MK2CPP_HandRegister(0x00005337u, &mk2c::step_pcm_play_movg2_dp_0xd15a_r0_b);
    MK2CPP_HandRegister(0x0000533bu, &mk2c::step_pcm_play_movg2_dp_0xd158_r2_a);
    MK2CPP_HandRegister(0x0000533fu, &mk2c::step_pcm_play_bsr16_to_0x3a9e);
    MK2CPP_HandRegister(0x00005342u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_c);
    MK2CPP_HandRegister(0x00005346u, &mk2c::step_pcm_play_bsr16_to_0x546e);
    MK2CPP_HandRegister(0x00005349u, &mk2c::step_pcm_play_movg2_dp_0xd15a_r0_c);
    MK2CPP_HandRegister(0x0000534du, &mk2c::step_pcm_play_bsr16_to_0x546e_a);
    MK2CPP_HandRegister(0x00005350u, &mk2c::step_pcm_play_movg2_dp_0xd158_r1);
    MK2CPP_HandRegister(0x00005354u, &mk2c::step_pcm_play_bsr16_to_0x54cc);
    MK2CPP_HandRegister(0x00005357u, &mk2c::step_pcm_play_movg2_dp_0xd15a_r1);
    MK2CPP_HandRegister(0x0000535bu, &mk2c::step_pcm_play_bsr16_to_0x54cc_a);
    MK2CPP_HandRegister(0x0000535eu, &mk2c::step_pcm_play_bset_orc_0x700_r0);
    MK2CPP_HandRegister(0x00005362u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_d);
    MK2CPP_HandRegister(0x00005366u, &mk2c::step_pcm_play_bsr16_to_0x54fb);
    MK2CPP_HandRegister(0x00005369u, &mk2c::step_pcm_play_bsr16_to_0x5533);
    MK2CPP_HandRegister(0x0000536cu, &mk2c::step_pcm_play_movg2_dp_0xd15a_r0_d);
    MK2CPP_HandRegister(0x00005370u, &mk2c::step_pcm_play_bsr16_to_0x5533_a);
    MK2CPP_HandRegister(0x00005373u, &mk2c::step_pcm_play_bsr16_to_0x564a);
    MK2CPP_HandRegister(0x00005376u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_e);
    MK2CPP_HandRegister(0x0000537au, &mk2c::step_pcm_play_btsti_at_r0_minus_59_7_a);
    MK2CPP_HandRegister(0x0000537du, &mk2c::step_pcm_play_beq_10_to_0x5389);
    MK2CPP_HandRegister(0x0000537fu, &mk2c::step_pcm_play_bsr16_to_0x5626);
    MK2CPP_HandRegister(0x00005382u, &mk2c::step_pcm_play_movg2_dp_0xd15a_r0_e);
    MK2CPP_HandRegister(0x00005386u, &mk2c::step_pcm_play_bsr16_to_0x5626_a);
    MK2CPP_HandRegister(0x00005389u, &mk2c::step_pcm_play_bclr_andc_0xf8ff_r0_a);
    MK2CPP_HandRegister(0x0000538du, &mk2c::step_pcm_play_bra_0xfffffe6c_to_0x51fc);
    MK2CPP_HandRegister(0x00005390u, &mk2c::step_pcm_play_bsr_89_to_0x53eb);
    MK2CPP_HandRegister(0x00005392u, &mk2c::step_pcm_play_movg3_r0_to_dp_0xd158_a);
    MK2CPP_HandRegister(0x00005396u, &mk2c::step_pcm_play_movg_0xffff_to_dp_0xd15a);
    MK2CPP_HandRegister(0x0000539cu, &mk2c::step_pcm_play_bclr_andc_0xf8ff_r0_b);
    MK2CPP_HandRegister(0x000053a0u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_f);
    MK2CPP_HandRegister(0x000053a4u, &mk2c::step_pcm_play_bsr16_to_0x5998_a);
    MK2CPP_HandRegister(0x000053a7u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_g);
    MK2CPP_HandRegister(0x000053abu, &mk2c::step_pcm_play_movg2_at_r0_minus_2_r1_a);
    MK2CPP_HandRegister(0x000053aeu, &mk2c::step_pcm_play_btsti_at_r0_minus_59_7_b);
    MK2CPP_HandRegister(0x000053b1u, &mk2c::step_pcm_play_beq_10_to_0x53bd);
    MK2CPP_HandRegister(0x000053b3u, &mk2c::step_pcm_play_bsr16_to_0x3580_b);
    MK2CPP_HandRegister(0x000053b6u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_h);
    MK2CPP_HandRegister(0x000053bau, &mk2c::step_pcm_play_bsr16_to_0x3615_a);
    MK2CPP_HandRegister(0x000053bdu, &mk2c::step_pcm_play_bsr16_to_0x546e_b);
    MK2CPP_HandRegister(0x000053c0u, &mk2c::step_pcm_play_movg2_dp_0xd158_r1_a);
    MK2CPP_HandRegister(0x000053c4u, &mk2c::step_pcm_play_bsr16_to_0x54cc_b);
    MK2CPP_HandRegister(0x000053c7u, &mk2c::step_pcm_play_bset_orc_0x700_r0_a);
    MK2CPP_HandRegister(0x000053cbu, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_i);
    MK2CPP_HandRegister(0x000053cfu, &mk2c::step_pcm_play_bsr16_to_0x54fb_a);
    MK2CPP_HandRegister(0x000053d2u, &mk2c::step_pcm_play_bsr16_to_0x5533_b);
    MK2CPP_HandRegister(0x000053d5u, &mk2c::step_pcm_play_bsr16_to_0x564a_a);
    MK2CPP_HandRegister(0x000053d8u, &mk2c::step_pcm_play_movg2_dp_0xd158_r0_j);
    MK2CPP_HandRegister(0x000053dcu, &mk2c::step_pcm_play_btsti_at_r0_minus_59_7_c);
    MK2CPP_HandRegister(0x000053dfu, &mk2c::step_pcm_play_beq_3_to_0x53e4);
    MK2CPP_HandRegister(0x000053e1u, &mk2c::step_pcm_play_bsr16_to_0x5626_b);
    MK2CPP_HandRegister(0x000053e4u, &mk2c::step_pcm_play_bclr_andc_0xf8ff_r0_c);
    MK2CPP_HandRegister(0x000053e8u, &mk2c::step_pcm_play_bra_0xfffffe11_to_0x51fc);

    /* coeff_calc: 345 PCs */
    MK2CPP_HandRegister(0x00005998u, &mk2c::step_coeff_calc_clr_r6);
    MK2CPP_HandRegister(0x0000599au, &mk2c::step_coeff_calc_movg2_at_r1_plus_0xce78_r6);
    MK2CPP_HandRegister(0x0000599eu, &mk2c::step_coeff_calc_movg2_r6_r3);
    MK2CPP_HandRegister(0x000059a0u, &mk2c::step_coeff_calc_add_r3_r3);
    MK2CPP_HandRegister(0x000059a2u, &mk2c::step_coeff_calc_movg2_at_r3_plus_0x7218_r2);
    MK2CPP_HandRegister(0x000059a6u, &mk2c::step_coeff_calc_movg3_r2_to_dp_0xd17e);
    MK2CPP_HandRegister(0x000059aau, &mk2c::step_coeff_calc_move_r4_0x80);
    MK2CPP_HandRegister(0x000059acu, &mk2c::step_coeff_calc_mulxu_r6_r4);
    MK2CPP_HandRegister(0x000059aeu, &mk2c::step_coeff_calc_clr_r3);
    MK2CPP_HandRegister(0x000059b0u, &mk2c::step_coeff_calc_movg2_at_r1_plus_0xd134_r3);
    MK2CPP_HandRegister(0x000059b4u, &mk2c::step_coeff_calc_add_r4_r3);
    MK2CPP_HandRegister(0x000059b6u, &mk2c::step_coeff_calc_movg2_at_r3_plus_0x9740_r5);
    MK2CPP_HandRegister(0x000059bau, &mk2c::step_coeff_calc_movg3_r5_to_dp_0xd180);
    MK2CPP_HandRegister(0x000059beu, &mk2c::step_coeff_calc_movg2_at_r2_plus_76_r4);
    MK2CPP_HandRegister(0x000059c1u, &mk2c::step_coeff_calc_sub_0x40_r4);
    MK2CPP_HandRegister(0x000059c4u, &mk2c::step_coeff_calc_bcc_9_to_0x59cf);
    MK2CPP_HandRegister(0x000059c6u, &mk2c::step_coeff_calc_neg_r4);
    MK2CPP_HandRegister(0x000059c8u, &mk2c::step_coeff_calc_mulxu_r5_r4);
    MK2CPP_HandRegister(0x000059cau, &mk2c::step_coeff_calc_neg_r4_a);
    MK2CPP_HandRegister(0x000059ccu, &mk2c::step_coeff_calc_bra_3_to_0x59d1);
    MK2CPP_HandRegister(0x000059ceu, &mk2c::step_coeff_calc_rts);
    MK2CPP_HandRegister(0x000059cfu, &mk2c::step_coeff_calc_mulxu_r5_r4_a);
    MK2CPP_HandRegister(0x000059d1u, &mk2c::step_coeff_calc_movg2_r6_r3_a);
    MK2CPP_HandRegister(0x000059d3u, &mk2c::step_coeff_calc_add_r3_r3_a);
    MK2CPP_HandRegister(0x000059d5u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9060_r4);
    MK2CPP_HandRegister(0x000059d9u, &mk2c::step_coeff_calc_add_at_r3_plus_0x91c0_r4);
    MK2CPP_HandRegister(0x000059ddu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9320_r4);
    MK2CPP_HandRegister(0x000059e1u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9480_r4);
    MK2CPP_HandRegister(0x000059e5u, &mk2c::step_coeff_calc_add_at_r3_plus_0x95e0_r4);
    MK2CPP_HandRegister(0x000059e9u, &mk2c::step_coeff_calc_bpl_24_to_0x5a03);
    MK2CPP_HandRegister(0x000059ebu, &mk2c::step_coeff_calc_neg_r4_b);
    MK2CPP_HandRegister(0x000059edu, &mk2c::step_coeff_calc_cmp_r4_w_0xbe8);
    MK2CPP_HandRegister(0x000059f0u, &mk2c::step_coeff_calc_bcs_3_to_0x59f5);
    MK2CPP_HandRegister(0x000059f2u, &mk2c::step_coeff_calc_movi_r4_0xbe8);
    MK2CPP_HandRegister(0x000059f5u, &mk2c::step_coeff_calc_add_r4_r4);
    MK2CPP_HandRegister(0x000059f7u, &mk2c::step_coeff_calc_add_r4_r4_a);
    MK2CPP_HandRegister(0x000059f9u, &mk2c::step_coeff_calc_add_r4_r4_b);
    MK2CPP_HandRegister(0x000059fbu, &mk2c::step_coeff_calc_mulxu_0xfbf8_r4_r5);
    MK2CPP_HandRegister(0x000059ffu, &mk2c::step_coeff_calc_neg_r4_c);
    MK2CPP_HandRegister(0x00005a01u, &mk2c::step_coeff_calc_bra_18_to_0x5a15);
    MK2CPP_HandRegister(0x00005a03u, &mk2c::step_coeff_calc_cmp_r4_w_0xbe8_a);
    MK2CPP_HandRegister(0x00005a06u, &mk2c::step_coeff_calc_bcs_3_to_0x5a0b);
    MK2CPP_HandRegister(0x00005a08u, &mk2c::step_coeff_calc_movi_r4_0xbe8_a);
    MK2CPP_HandRegister(0x00005a0bu, &mk2c::step_coeff_calc_add_r4_r4_c);
    MK2CPP_HandRegister(0x00005a0du, &mk2c::step_coeff_calc_add_r4_r4_d);
    MK2CPP_HandRegister(0x00005a0fu, &mk2c::step_coeff_calc_add_r4_r4_e);
    MK2CPP_HandRegister(0x00005a11u, &mk2c::step_coeff_calc_mulxu_0xfbf8_r4_r5_a);
    MK2CPP_HandRegister(0x00005a15u, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x86);
    MK2CPP_HandRegister(0x00005a19u, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2);
    MK2CPP_HandRegister(0x00005a1du, &mk2c::step_coeff_calc_movg2_at_r2_plus_78_r4);
    MK2CPP_HandRegister(0x00005a20u, &mk2c::step_coeff_calc_sub_0x40_r4_a);
    MK2CPP_HandRegister(0x00005a23u, &mk2c::step_coeff_calc_bcc_12_to_0x5a31);
    MK2CPP_HandRegister(0x00005a25u, &mk2c::step_coeff_calc_neg_r4_d);
    MK2CPP_HandRegister(0x00005a27u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4);
    MK2CPP_HandRegister(0x00005a2bu, &mk2c::step_coeff_calc_shlr_r4);
    MK2CPP_HandRegister(0x00005a2du, &mk2c::step_coeff_calc_neg_r4_e);
    MK2CPP_HandRegister(0x00005a2fu, &mk2c::step_coeff_calc_bra_6_to_0x5a37);
    MK2CPP_HandRegister(0x00005a31u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_a);
    MK2CPP_HandRegister(0x00005a35u, &mk2c::step_coeff_calc_shlr_r4_a);
    MK2CPP_HandRegister(0x00005a37u, &mk2c::step_coeff_calc_add_at_r3_plus_0x90a0_r4);
    MK2CPP_HandRegister(0x00005a3bu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9200_r4);
    MK2CPP_HandRegister(0x00005a3fu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9360_r4);
    MK2CPP_HandRegister(0x00005a43u, &mk2c::step_coeff_calc_add_at_r3_plus_0x94c0_r4);
    MK2CPP_HandRegister(0x00005a47u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9620_r4);
    MK2CPP_HandRegister(0x00005a4bu, &mk2c::step_coeff_calc_bpl_26_to_0x5a67);
    MK2CPP_HandRegister(0x00005a4du, &mk2c::step_coeff_calc_neg_r4_f);
    MK2CPP_HandRegister(0x00005a4fu, &mk2c::step_coeff_calc_cmp_r4_w_0xfa0);
    MK2CPP_HandRegister(0x00005a52u, &mk2c::step_coeff_calc_bcs_3_to_0x5a57);
    MK2CPP_HandRegister(0x00005a54u, &mk2c::step_coeff_calc_movi_r4_0xfa0);
    MK2CPP_HandRegister(0x00005a57u, &mk2c::step_coeff_calc_add_r4_r4_f);
    MK2CPP_HandRegister(0x00005a59u, &mk2c::step_coeff_calc_add_r4_r4_g);
    MK2CPP_HandRegister(0x00005a5bu, &mk2c::step_coeff_calc_add_r4_r4_h);
    MK2CPP_HandRegister(0x00005a5du, &mk2c::step_coeff_calc_add_r4_r4_i);
    MK2CPP_HandRegister(0x00005a5fu, &mk2c::step_coeff_calc_mulxu_0x820d_r4_r5);
    MK2CPP_HandRegister(0x00005a63u, &mk2c::step_coeff_calc_neg_r4_g);
    MK2CPP_HandRegister(0x00005a65u, &mk2c::step_coeff_calc_bra_20_to_0x5a7b);
    MK2CPP_HandRegister(0x00005a67u, &mk2c::step_coeff_calc_cmp_r4_w_0xfa0_a);
    MK2CPP_HandRegister(0x00005a6au, &mk2c::step_coeff_calc_bcs_3_to_0x5a6f);
    MK2CPP_HandRegister(0x00005a6cu, &mk2c::step_coeff_calc_movi_r4_0xfa0_a);
    MK2CPP_HandRegister(0x00005a6fu, &mk2c::step_coeff_calc_add_r4_r4_j);
    MK2CPP_HandRegister(0x00005a71u, &mk2c::step_coeff_calc_add_r4_r4_k);
    MK2CPP_HandRegister(0x00005a73u, &mk2c::step_coeff_calc_add_r4_r4_l);
    MK2CPP_HandRegister(0x00005a75u, &mk2c::step_coeff_calc_add_r4_r4_m);
    MK2CPP_HandRegister(0x00005a77u, &mk2c::step_coeff_calc_mulxu_0x820d_r4_r5_a);
    MK2CPP_HandRegister(0x00005a7bu, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x8a);
    MK2CPP_HandRegister(0x00005a7fu, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_a);
    MK2CPP_HandRegister(0x00005a83u, &mk2c::step_coeff_calc_movg2_at_r2_plus_77_r4);
    MK2CPP_HandRegister(0x00005a86u, &mk2c::step_coeff_calc_sub_0x40_r4_b);
    MK2CPP_HandRegister(0x00005a89u, &mk2c::step_coeff_calc_bcc_12_to_0x5a97);
    MK2CPP_HandRegister(0x00005a8bu, &mk2c::step_coeff_calc_neg_r4_h);
    MK2CPP_HandRegister(0x00005a8du, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_b);
    MK2CPP_HandRegister(0x00005a91u, &mk2c::step_coeff_calc_shlr_r4_b);
    MK2CPP_HandRegister(0x00005a93u, &mk2c::step_coeff_calc_neg_r4_i);
    MK2CPP_HandRegister(0x00005a95u, &mk2c::step_coeff_calc_bra_6_to_0x5a9d);
    MK2CPP_HandRegister(0x00005a97u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_c);
    MK2CPP_HandRegister(0x00005a9bu, &mk2c::step_coeff_calc_shlr_r4_c);
    MK2CPP_HandRegister(0x00005a9du, &mk2c::step_coeff_calc_add_at_r3_plus_0x9080_r4);
    MK2CPP_HandRegister(0x00005aa1u, &mk2c::step_coeff_calc_add_at_r3_plus_0x91e0_r4);
    MK2CPP_HandRegister(0x00005aa5u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9340_r4);
    MK2CPP_HandRegister(0x00005aa9u, &mk2c::step_coeff_calc_add_at_r3_plus_0x94a0_r4);
    MK2CPP_HandRegister(0x00005aadu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9600_r4);
    MK2CPP_HandRegister(0x00005ab1u, &mk2c::step_coeff_calc_bpl_24_to_0x5acb);
    MK2CPP_HandRegister(0x00005ab3u, &mk2c::step_coeff_calc_neg_r4_j);
    MK2CPP_HandRegister(0x00005ab5u, &mk2c::step_coeff_calc_cmp_r4_w_0xfa0_b);
    MK2CPP_HandRegister(0x00005ab8u, &mk2c::step_coeff_calc_bcs_3_to_0x5abd);
    MK2CPP_HandRegister(0x00005abau, &mk2c::step_coeff_calc_movi_r4_0xfa0_b);
    MK2CPP_HandRegister(0x00005abdu, &mk2c::step_coeff_calc_add_r4_r4_n);
    MK2CPP_HandRegister(0x00005abfu, &mk2c::step_coeff_calc_add_r4_r4_o);
    MK2CPP_HandRegister(0x00005ac1u, &mk2c::step_coeff_calc_add_r4_r4_p);
    MK2CPP_HandRegister(0x00005ac3u, &mk2c::step_coeff_calc_mulxu_0xc49c_r4_r5);
    MK2CPP_HandRegister(0x00005ac7u, &mk2c::step_coeff_calc_neg_r4_k);
    MK2CPP_HandRegister(0x00005ac9u, &mk2c::step_coeff_calc_bra_18_to_0x5add);
    MK2CPP_HandRegister(0x00005acbu, &mk2c::step_coeff_calc_cmp_r4_w_0xfa0_c);
    MK2CPP_HandRegister(0x00005aceu, &mk2c::step_coeff_calc_bcs_3_to_0x5ad3);
    MK2CPP_HandRegister(0x00005ad0u, &mk2c::step_coeff_calc_movi_r4_0xfa0_c);
    MK2CPP_HandRegister(0x00005ad3u, &mk2c::step_coeff_calc_add_r4_r4_q);
    MK2CPP_HandRegister(0x00005ad5u, &mk2c::step_coeff_calc_add_r4_r4_r);
    MK2CPP_HandRegister(0x00005ad7u, &mk2c::step_coeff_calc_add_r4_r4_s);
    MK2CPP_HandRegister(0x00005ad9u, &mk2c::step_coeff_calc_mulxu_0xc49c_r4_r5_a);
    MK2CPP_HandRegister(0x00005addu, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x88);
    MK2CPP_HandRegister(0x00005ae1u, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_b);
    MK2CPP_HandRegister(0x00005ae5u, &mk2c::step_coeff_calc_movg2_at_r2_plus_84_r4);
    MK2CPP_HandRegister(0x00005ae8u, &mk2c::step_coeff_calc_sub_0x40_r4_c);
    MK2CPP_HandRegister(0x00005aebu, &mk2c::step_coeff_calc_bcc_12_to_0x5af9);
    MK2CPP_HandRegister(0x00005aedu, &mk2c::step_coeff_calc_neg_r4_l);
    MK2CPP_HandRegister(0x00005aefu, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_d);
    MK2CPP_HandRegister(0x00005af3u, &mk2c::step_coeff_calc_shlr_r4_d);
    MK2CPP_HandRegister(0x00005af5u, &mk2c::step_coeff_calc_neg_r4_m);
    MK2CPP_HandRegister(0x00005af7u, &mk2c::step_coeff_calc_bra_6_to_0x5aff);
    MK2CPP_HandRegister(0x00005af9u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_e);
    MK2CPP_HandRegister(0x00005afdu, &mk2c::step_coeff_calc_shlr_r4_e);
    MK2CPP_HandRegister(0x00005affu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9140_r4);
    MK2CPP_HandRegister(0x00005b03u, &mk2c::step_coeff_calc_add_at_r3_plus_0x92a0_r4);
    MK2CPP_HandRegister(0x00005b07u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9400_r4);
    MK2CPP_HandRegister(0x00005b0bu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9560_r4);
    MK2CPP_HandRegister(0x00005b0fu, &mk2c::step_coeff_calc_add_at_r3_plus_0x96c0_r4);
    MK2CPP_HandRegister(0x00005b13u, &mk2c::step_coeff_calc_bpl_20_to_0x5b29);
    MK2CPP_HandRegister(0x00005b15u, &mk2c::step_coeff_calc_neg_r4_n);
    MK2CPP_HandRegister(0x00005b17u, &mk2c::step_coeff_calc_cmp_r4_w_0xfa0_d);
    MK2CPP_HandRegister(0x00005b1au, &mk2c::step_coeff_calc_bcs_3_to_0x5b1f);
    MK2CPP_HandRegister(0x00005b1cu, &mk2c::step_coeff_calc_movi_r4_0xfa0_d);
    MK2CPP_HandRegister(0x00005b1fu, &mk2c::step_coeff_calc_add_r4_r4_t);
    MK2CPP_HandRegister(0x00005b21u, &mk2c::step_coeff_calc_mulxu_0xa7c7_r4_r5);
    MK2CPP_HandRegister(0x00005b25u, &mk2c::step_coeff_calc_neg_r4_o);
    MK2CPP_HandRegister(0x00005b27u, &mk2c::step_coeff_calc_bra_14_to_0x5b37);
    MK2CPP_HandRegister(0x00005b29u, &mk2c::step_coeff_calc_cmp_r4_w_0xfa0_e);
    MK2CPP_HandRegister(0x00005b2cu, &mk2c::step_coeff_calc_bcs_3_to_0x5b31);
    MK2CPP_HandRegister(0x00005b2eu, &mk2c::step_coeff_calc_movi_r4_0xfa0_e);
    MK2CPP_HandRegister(0x00005b31u, &mk2c::step_coeff_calc_add_r4_r4_u);
    MK2CPP_HandRegister(0x00005b33u, &mk2c::step_coeff_calc_mulxu_0xa7c7_r4_r5_a);
    MK2CPP_HandRegister(0x00005b37u, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_minus_80);
    MK2CPP_HandRegister(0x00005b3au, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_c);
    MK2CPP_HandRegister(0x00005b3eu, &mk2c::step_coeff_calc_movg2_at_r2_plus_80_r4);
    MK2CPP_HandRegister(0x00005b41u, &mk2c::step_coeff_calc_sub_0x40_r4_d);
    MK2CPP_HandRegister(0x00005b44u, &mk2c::step_coeff_calc_bcc_12_to_0x5b52);
    MK2CPP_HandRegister(0x00005b46u, &mk2c::step_coeff_calc_neg_r4_p);
    MK2CPP_HandRegister(0x00005b48u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_f);
    MK2CPP_HandRegister(0x00005b4cu, &mk2c::step_coeff_calc_shlr_r4_f);
    MK2CPP_HandRegister(0x00005b4eu, &mk2c::step_coeff_calc_neg_r4_q);
    MK2CPP_HandRegister(0x00005b50u, &mk2c::step_coeff_calc_bra_6_to_0x5b58);
    MK2CPP_HandRegister(0x00005b52u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_g);
    MK2CPP_HandRegister(0x00005b56u, &mk2c::step_coeff_calc_shlr_r4_g);
    MK2CPP_HandRegister(0x00005b58u, &mk2c::step_coeff_calc_add_at_r3_plus_0x90c0_r4);
    MK2CPP_HandRegister(0x00005b5cu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9220_r4);
    MK2CPP_HandRegister(0x00005b60u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9380_r4);
    MK2CPP_HandRegister(0x00005b64u, &mk2c::step_coeff_calc_add_at_r3_plus_0x94e0_r4);
    MK2CPP_HandRegister(0x00005b68u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9640_r4);
    MK2CPP_HandRegister(0x00005b6cu, &mk2c::step_coeff_calc_bpl_20_to_0x5b82);
    MK2CPP_HandRegister(0x00005b6eu, &mk2c::step_coeff_calc_neg_r4_r);
    MK2CPP_HandRegister(0x00005b70u, &mk2c::step_coeff_calc_cmp_r4_w_0xfa0_f);
    MK2CPP_HandRegister(0x00005b73u, &mk2c::step_coeff_calc_bcs_3_to_0x5b78);
    MK2CPP_HandRegister(0x00005b75u, &mk2c::step_coeff_calc_movi_r4_0xfa0_f);
    MK2CPP_HandRegister(0x00005b78u, &mk2c::step_coeff_calc_add_r4_r4_v);
    MK2CPP_HandRegister(0x00005b7au, &mk2c::step_coeff_calc_mulxu_0xa7c7_r4_r5_b);
    MK2CPP_HandRegister(0x00005b7eu, &mk2c::step_coeff_calc_neg_r4_s);
    MK2CPP_HandRegister(0x00005b80u, &mk2c::step_coeff_calc_bra_14_to_0x5b90);
    MK2CPP_HandRegister(0x00005b82u, &mk2c::step_coeff_calc_cmp_r4_w_0xfa0_g);
    MK2CPP_HandRegister(0x00005b85u, &mk2c::step_coeff_calc_bcs_3_to_0x5b8a);
    MK2CPP_HandRegister(0x00005b87u, &mk2c::step_coeff_calc_movi_r4_0xfa0_g);
    MK2CPP_HandRegister(0x00005b8au, &mk2c::step_coeff_calc_add_r4_r4_w);
    MK2CPP_HandRegister(0x00005b8cu, &mk2c::step_coeff_calc_mulxu_0xa7c7_r4_r5_c);
    MK2CPP_HandRegister(0x00005b90u, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_minus_114);
    MK2CPP_HandRegister(0x00005b93u, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_d);
    MK2CPP_HandRegister(0x00005b97u, &mk2c::step_coeff_calc_movg2_at_r2_plus_87_r4);
    MK2CPP_HandRegister(0x00005b9au, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_h);
    MK2CPP_HandRegister(0x00005b9eu, &mk2c::step_coeff_calc_shlr_r4_h);
    MK2CPP_HandRegister(0x00005ba0u, &mk2c::step_coeff_calc_shlr_r4_i);
    MK2CPP_HandRegister(0x00005ba2u, &mk2c::step_coeff_calc_add_at_r3_plus_0x91a0_r4);
    MK2CPP_HandRegister(0x00005ba6u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9300_r4);
    MK2CPP_HandRegister(0x00005baau, &mk2c::step_coeff_calc_add_at_r3_plus_0x9460_r4);
    MK2CPP_HandRegister(0x00005baeu, &mk2c::step_coeff_calc_add_at_r3_plus_0x95c0_r4);
    MK2CPP_HandRegister(0x00005bb2u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9720_r4);
    MK2CPP_HandRegister(0x00005bb6u, &mk2c::step_coeff_calc_bpl_26_to_0x5bd2);
    MK2CPP_HandRegister(0x00005bb8u, &mk2c::step_coeff_calc_neg_r4_t);
    MK2CPP_HandRegister(0x00005bbau, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0);
    MK2CPP_HandRegister(0x00005bbdu, &mk2c::step_coeff_calc_bcs_3_to_0x5bc2);
    MK2CPP_HandRegister(0x00005bbfu, &mk2c::step_coeff_calc_movi_r4_0xfc0);
    MK2CPP_HandRegister(0x00005bc2u, &mk2c::step_coeff_calc_add_r4_r4_x);
    MK2CPP_HandRegister(0x00005bc4u, &mk2c::step_coeff_calc_add_r4_r4_y);
    MK2CPP_HandRegister(0x00005bc6u, &mk2c::step_coeff_calc_add_r4_r4_z);
    MK2CPP_HandRegister(0x00005bc8u, &mk2c::step_coeff_calc_add_r4_r4_27);
    MK2CPP_HandRegister(0x00005bcau, &mk2c::step_coeff_calc_mulxu_0x8105_r4_r5);
    MK2CPP_HandRegister(0x00005bceu, &mk2c::step_coeff_calc_neg_r4_u);
    MK2CPP_HandRegister(0x00005bd0u, &mk2c::step_coeff_calc_bra_20_to_0x5be6);
    MK2CPP_HandRegister(0x00005bd2u, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_a);
    MK2CPP_HandRegister(0x00005bd5u, &mk2c::step_coeff_calc_bcs_3_to_0x5bda);
    MK2CPP_HandRegister(0x00005bd7u, &mk2c::step_coeff_calc_movi_r4_0xfc0_a);
    MK2CPP_HandRegister(0x00005bdau, &mk2c::step_coeff_calc_add_r4_r4_28);
    MK2CPP_HandRegister(0x00005bdcu, &mk2c::step_coeff_calc_add_r4_r4_29);
    MK2CPP_HandRegister(0x00005bdeu, &mk2c::step_coeff_calc_add_r4_r4_30);
    MK2CPP_HandRegister(0x00005be0u, &mk2c::step_coeff_calc_add_r4_r4_31);
    MK2CPP_HandRegister(0x00005be2u, &mk2c::step_coeff_calc_mulxu_0x8105_r4_r5_a);
    MK2CPP_HandRegister(0x00005be6u, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x96);
    MK2CPP_HandRegister(0x00005beau, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_e);
    MK2CPP_HandRegister(0x00005beeu, &mk2c::step_coeff_calc_movg2_at_r2_plus_83_r4);
    MK2CPP_HandRegister(0x00005bf1u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_i);
    MK2CPP_HandRegister(0x00005bf5u, &mk2c::step_coeff_calc_shlr_r4_j);
    MK2CPP_HandRegister(0x00005bf7u, &mk2c::step_coeff_calc_shlr_r4_k);
    MK2CPP_HandRegister(0x00005bf9u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9120_r4);
    MK2CPP_HandRegister(0x00005bfdu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9280_r4);
    MK2CPP_HandRegister(0x00005c01u, &mk2c::step_coeff_calc_add_at_r3_plus_0x93e0_r4);
    MK2CPP_HandRegister(0x00005c05u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9540_r4);
    MK2CPP_HandRegister(0x00005c09u, &mk2c::step_coeff_calc_add_at_r3_plus_0x96a0_r4);
    MK2CPP_HandRegister(0x00005c0du, &mk2c::step_coeff_calc_bpl_26_to_0x5c29);
    MK2CPP_HandRegister(0x00005c0fu, &mk2c::step_coeff_calc_neg_r4_v);
    MK2CPP_HandRegister(0x00005c11u, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_b);
    MK2CPP_HandRegister(0x00005c14u, &mk2c::step_coeff_calc_bcs_3_to_0x5c19);
    MK2CPP_HandRegister(0x00005c16u, &mk2c::step_coeff_calc_movi_r4_0xfc0_b);
    MK2CPP_HandRegister(0x00005c19u, &mk2c::step_coeff_calc_add_r4_r4_32);
    MK2CPP_HandRegister(0x00005c1bu, &mk2c::step_coeff_calc_add_r4_r4_33);
    MK2CPP_HandRegister(0x00005c1du, &mk2c::step_coeff_calc_add_r4_r4_34);
    MK2CPP_HandRegister(0x00005c1fu, &mk2c::step_coeff_calc_add_r4_r4_35);
    MK2CPP_HandRegister(0x00005c21u, &mk2c::step_coeff_calc_mulxu_0x8105_r4_r5_b);
    MK2CPP_HandRegister(0x00005c25u, &mk2c::step_coeff_calc_neg_r4_w);
    MK2CPP_HandRegister(0x00005c27u, &mk2c::step_coeff_calc_bra_20_to_0x5c3d);
    MK2CPP_HandRegister(0x00005c29u, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_c);
    MK2CPP_HandRegister(0x00005c2cu, &mk2c::step_coeff_calc_bcs_3_to_0x5c31);
    MK2CPP_HandRegister(0x00005c2eu, &mk2c::step_coeff_calc_movi_r4_0xfc0_c);
    MK2CPP_HandRegister(0x00005c31u, &mk2c::step_coeff_calc_add_r4_r4_36);
    MK2CPP_HandRegister(0x00005c33u, &mk2c::step_coeff_calc_add_r4_r4_37);
    MK2CPP_HandRegister(0x00005c35u, &mk2c::step_coeff_calc_add_r4_r4_38);
    MK2CPP_HandRegister(0x00005c37u, &mk2c::step_coeff_calc_add_r4_r4_39);
    MK2CPP_HandRegister(0x00005c39u, &mk2c::step_coeff_calc_mulxu_0x8105_r4_r5_c);
    MK2CPP_HandRegister(0x00005c3du, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x8e);
    MK2CPP_HandRegister(0x00005c41u, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_f);
    MK2CPP_HandRegister(0x00005c45u, &mk2c::step_coeff_calc_movg2_at_r2_plus_86_r4);
    MK2CPP_HandRegister(0x00005c48u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_j);
    MK2CPP_HandRegister(0x00005c4cu, &mk2c::step_coeff_calc_shlr_r4_l);
    MK2CPP_HandRegister(0x00005c4eu, &mk2c::step_coeff_calc_shlr_r4_m);
    MK2CPP_HandRegister(0x00005c50u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9180_r4);
    MK2CPP_HandRegister(0x00005c54u, &mk2c::step_coeff_calc_add_at_r3_plus_0x92e0_r4);
    MK2CPP_HandRegister(0x00005c58u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9440_r4);
    MK2CPP_HandRegister(0x00005c5cu, &mk2c::step_coeff_calc_add_at_r3_plus_0x95a0_r4);
    MK2CPP_HandRegister(0x00005c60u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9700_r4);
    MK2CPP_HandRegister(0x00005c64u, &mk2c::step_coeff_calc_bpl_20_to_0x5c7a);
    MK2CPP_HandRegister(0x00005c66u, &mk2c::step_coeff_calc_neg_r4_x);
    MK2CPP_HandRegister(0x00005c68u, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_d);
    MK2CPP_HandRegister(0x00005c6bu, &mk2c::step_coeff_calc_bcs_3_to_0x5c70);
    MK2CPP_HandRegister(0x00005c6du, &mk2c::step_coeff_calc_movi_r4_0xfc0_d);
    MK2CPP_HandRegister(0x00005c70u, &mk2c::step_coeff_calc_add_r4_r4_40);
    MK2CPP_HandRegister(0x00005c72u, &mk2c::step_coeff_calc_mulxu_0xc30d_r4_r5);
    MK2CPP_HandRegister(0x00005c76u, &mk2c::step_coeff_calc_neg_r4_y);
    MK2CPP_HandRegister(0x00005c78u, &mk2c::step_coeff_calc_bra_14_to_0x5c88);
    MK2CPP_HandRegister(0x00005c7au, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_e);
    MK2CPP_HandRegister(0x00005c7du, &mk2c::step_coeff_calc_bcs_3_to_0x5c82);
    MK2CPP_HandRegister(0x00005c7fu, &mk2c::step_coeff_calc_movi_r4_0xfc0_e);
    MK2CPP_HandRegister(0x00005c82u, &mk2c::step_coeff_calc_add_r4_r4_41);
    MK2CPP_HandRegister(0x00005c84u, &mk2c::step_coeff_calc_mulxu_0xc30d_r4_r5_a);
    MK2CPP_HandRegister(0x00005c88u, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x94);
    MK2CPP_HandRegister(0x00005c8cu, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_g);
    MK2CPP_HandRegister(0x00005c90u, &mk2c::step_coeff_calc_movg2_at_r2_plus_82_r4);
    MK2CPP_HandRegister(0x00005c93u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_k);
    MK2CPP_HandRegister(0x00005c97u, &mk2c::step_coeff_calc_shlr_r4_n);
    MK2CPP_HandRegister(0x00005c99u, &mk2c::step_coeff_calc_shlr_r4_o);
    MK2CPP_HandRegister(0x00005c9bu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9100_r4);
    MK2CPP_HandRegister(0x00005c9fu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9260_r4);
    MK2CPP_HandRegister(0x00005ca3u, &mk2c::step_coeff_calc_add_at_r3_plus_0x93c0_r4);
    MK2CPP_HandRegister(0x00005ca7u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9520_r4);
    MK2CPP_HandRegister(0x00005cabu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9680_r4);
    MK2CPP_HandRegister(0x00005cafu, &mk2c::step_coeff_calc_bpl_20_to_0x5cc5);
    MK2CPP_HandRegister(0x00005cb1u, &mk2c::step_coeff_calc_neg_r4_z);
    MK2CPP_HandRegister(0x00005cb3u, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_f);
    MK2CPP_HandRegister(0x00005cb6u, &mk2c::step_coeff_calc_bcs_3_to_0x5cbb);
    MK2CPP_HandRegister(0x00005cb8u, &mk2c::step_coeff_calc_movi_r4_0xfc0_f);
    MK2CPP_HandRegister(0x00005cbbu, &mk2c::step_coeff_calc_add_r4_r4_42);
    MK2CPP_HandRegister(0x00005cbdu, &mk2c::step_coeff_calc_mulxu_0xc30d_r4_r5_b);
    MK2CPP_HandRegister(0x00005cc1u, &mk2c::step_coeff_calc_neg_r4_27);
    MK2CPP_HandRegister(0x00005cc3u, &mk2c::step_coeff_calc_bra_14_to_0x5cd3);
    MK2CPP_HandRegister(0x00005cc5u, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_g);
    MK2CPP_HandRegister(0x00005cc8u, &mk2c::step_coeff_calc_bcs_3_to_0x5ccd);
    MK2CPP_HandRegister(0x00005ccau, &mk2c::step_coeff_calc_movi_r4_0xfc0_g);
    MK2CPP_HandRegister(0x00005ccdu, &mk2c::step_coeff_calc_add_r4_r4_43);
    MK2CPP_HandRegister(0x00005ccfu, &mk2c::step_coeff_calc_mulxu_0xc30d_r4_r5_c);
    MK2CPP_HandRegister(0x00005cd3u, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x8c);
    MK2CPP_HandRegister(0x00005cd7u, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_h);
    MK2CPP_HandRegister(0x00005cdbu, &mk2c::step_coeff_calc_movg2_at_r2_plus_85_r4);
    MK2CPP_HandRegister(0x00005cdeu, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_l);
    MK2CPP_HandRegister(0x00005ce2u, &mk2c::step_coeff_calc_shlr_r4_p);
    MK2CPP_HandRegister(0x00005ce4u, &mk2c::step_coeff_calc_shlr_r4_q);
    MK2CPP_HandRegister(0x00005ce6u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9160_r4);
    MK2CPP_HandRegister(0x00005ceau, &mk2c::step_coeff_calc_add_at_r3_plus_0x92c0_r4);
    MK2CPP_HandRegister(0x00005ceeu, &mk2c::step_coeff_calc_add_at_r3_plus_0x9420_r4);
    MK2CPP_HandRegister(0x00005cf2u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9580_r4);
    MK2CPP_HandRegister(0x00005cf6u, &mk2c::step_coeff_calc_add_at_r3_plus_0x96e0_r4);
    MK2CPP_HandRegister(0x00005cfau, &mk2c::step_coeff_calc_bpl_20_to_0x5d10);
    MK2CPP_HandRegister(0x00005cfcu, &mk2c::step_coeff_calc_neg_r4_28);
    MK2CPP_HandRegister(0x00005cfeu, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_h);
    MK2CPP_HandRegister(0x00005d01u, &mk2c::step_coeff_calc_bcs_3_to_0x5d06);
    MK2CPP_HandRegister(0x00005d03u, &mk2c::step_coeff_calc_movi_r4_0xfc0_h);
    MK2CPP_HandRegister(0x00005d06u, &mk2c::step_coeff_calc_add_r4_r4_44);
    MK2CPP_HandRegister(0x00005d08u, &mk2c::step_coeff_calc_mulxu_0xbe7a_r4_r5);
    MK2CPP_HandRegister(0x00005d0cu, &mk2c::step_coeff_calc_neg_r4_29);
    MK2CPP_HandRegister(0x00005d0eu, &mk2c::step_coeff_calc_bra_14_to_0x5d1e);
    MK2CPP_HandRegister(0x00005d10u, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_i);
    MK2CPP_HandRegister(0x00005d13u, &mk2c::step_coeff_calc_bcs_3_to_0x5d18);
    MK2CPP_HandRegister(0x00005d15u, &mk2c::step_coeff_calc_movi_r4_0xfc0_i);
    MK2CPP_HandRegister(0x00005d18u, &mk2c::step_coeff_calc_add_r4_r4_45);
    MK2CPP_HandRegister(0x00005d1au, &mk2c::step_coeff_calc_mulxu_0xbe7a_r4_r5_a);
    MK2CPP_HandRegister(0x00005d1eu, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x92);
    MK2CPP_HandRegister(0x00005d22u, &mk2c::step_coeff_calc_movg2_dp_0xd17e_r2_i);
    MK2CPP_HandRegister(0x00005d26u, &mk2c::step_coeff_calc_movg2_at_r2_plus_81_r4);
    MK2CPP_HandRegister(0x00005d29u, &mk2c::step_coeff_calc_mulxu_dp_0xd180_r4_m);
    MK2CPP_HandRegister(0x00005d2du, &mk2c::step_coeff_calc_shlr_r4_r);
    MK2CPP_HandRegister(0x00005d2fu, &mk2c::step_coeff_calc_shlr_r4_s);
    MK2CPP_HandRegister(0x00005d31u, &mk2c::step_coeff_calc_add_at_r3_plus_0x90e0_r4);
    MK2CPP_HandRegister(0x00005d35u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9240_r4);
    MK2CPP_HandRegister(0x00005d39u, &mk2c::step_coeff_calc_add_at_r3_plus_0x93a0_r4);
    MK2CPP_HandRegister(0x00005d3du, &mk2c::step_coeff_calc_add_at_r3_plus_0x9500_r4);
    MK2CPP_HandRegister(0x00005d41u, &mk2c::step_coeff_calc_add_at_r3_plus_0x9660_r4);
    MK2CPP_HandRegister(0x00005d45u, &mk2c::step_coeff_calc_bpl_20_to_0x5d5b);
    MK2CPP_HandRegister(0x00005d47u, &mk2c::step_coeff_calc_neg_r4_30);
    MK2CPP_HandRegister(0x00005d49u, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_j);
    MK2CPP_HandRegister(0x00005d4cu, &mk2c::step_coeff_calc_bcs_3_to_0x5d51);
    MK2CPP_HandRegister(0x00005d4eu, &mk2c::step_coeff_calc_movi_r4_0xfc0_j);
    MK2CPP_HandRegister(0x00005d51u, &mk2c::step_coeff_calc_add_r4_r4_46);
    MK2CPP_HandRegister(0x00005d53u, &mk2c::step_coeff_calc_mulxu_0xbe7a_r4_r5_b);
    MK2CPP_HandRegister(0x00005d57u, &mk2c::step_coeff_calc_neg_r4_31);
    MK2CPP_HandRegister(0x00005d59u, &mk2c::step_coeff_calc_bra_14_to_0x5d69);
    MK2CPP_HandRegister(0x00005d5bu, &mk2c::step_coeff_calc_cmp_r4_w_0xfc0_k);
    MK2CPP_HandRegister(0x00005d5eu, &mk2c::step_coeff_calc_bcs_3_to_0x5d63);
    MK2CPP_HandRegister(0x00005d60u, &mk2c::step_coeff_calc_movi_r4_0xfc0_k);
    MK2CPP_HandRegister(0x00005d63u, &mk2c::step_coeff_calc_add_r4_r4_47);
    MK2CPP_HandRegister(0x00005d65u, &mk2c::step_coeff_calc_mulxu_0xbe7a_r4_r5_c);
    MK2CPP_HandRegister(0x00005d69u, &mk2c::step_coeff_calc_movg3_r4_to_at_r0_plus_0x90);
    MK2CPP_HandRegister(0x00005d6du, &mk2c::step_coeff_calc_rts_a);
}

/* Self-registration (hand_registry.h): pcm_enable.cpp's MK2CPP_HandFillTables
 * calls mk2c::hand_fill_modules() after the built-in modules; the fill above
 * is idempotent, so a direct call and this module registration coexist. */
namespace
{
struct VoiceMaterializeModule
{
    VoiceMaterializeModule() { mk2c::hand_register_module(&MK2CPP_VoiceMaterializeFillTables); }
};
VoiceMaterializeModule g_voice_materialize_module;
}
