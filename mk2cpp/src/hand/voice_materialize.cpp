/*
 * HAND voice/voice_materialize -- PCM voice materialize / gate / param chain
 * (M4 S1..S10): materialize, coeff_copy, tone_fields, cross_copy, gate_setup,
 * coeff_calc, mask_set + shared tail, flush_off/flush_on, param_write,
 * loop_write, param_ack, pcm_play.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 7
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

uint32_t routine_materialize_step(void)
{
    uint16_t &r0 = mcu.r[0];
    uint16_t &r1 = mcu.r[1];
    uint16_t &r3 = mcu.r[3];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x53eb: r3 = r1;                                       mcu.pc = 0x53ed; break;
    case 0x53ed: add16(r3, r3);                                 mcu.pc = 0x53ef; break;
    case 0x53ef: load16(r0, ind_addr(3, kAosPtrTable));         mcu.pc = 0x53f3; break;
    case 0x53f3: store16(ind_addr(0, (uint16_t)kAosSlot), r1);  mcu.pc = 0x53f6; break;
    case 0x53f6: load8(r6, ind_addr(1, kSoaCfe4));              mcu.pc = 0x53fa; break;
    case 0x53fa: store8(ind_addr(0, kAosE1), (uint8_t)r6);      mcu.pc = 0x53fe; break;
    case 0x53fe: load8(r6, ind_addr(1, kSoaD038));              mcu.pc = 0x5402; break;
    case 0x5402: store8(ind_addr(0, kAosE2), (uint8_t)r6);      mcu.pc = 0x5406; break;
    case 0x5406: load8(r6, ind_addr(1, kSoaD08c));              mcu.pc = 0x540a; break;
    case 0x540a: store8(ind_addr(0, kAosE3), (uint8_t)r6);      mcu.pc = 0x540e; break;
    case 0x540e: load16(r6, ind_addr(3, kSoaCfac));             mcu.pc = 0x5412; break;
    case 0x5412: store16(ind_addr(0, kAosP1), r6);              mcu.pc = 0x5416; break;
    case 0x5416: load16(r6, ind_addr(3, kSoaD000));             mcu.pc = 0x541a; break;
    case 0x541a: store16(ind_addr(0, kAosP2), r6);              mcu.pc = 0x541e; break;
    case 0x541e: load16(r6, ind_addr(3, kSoaD054));             mcu.pc = 0x5422; break;
    case 0x5422: store16(ind_addr(0, kAosP3), r6);              mcu.pc = 0x5426; break;
    case 0x5426: load8(r6, ind_addr(1, kSoaCf90));              mcu.pc = 0x542a; break;
    case 0x542a: store8(ind_addr(0, (uint16_t)kAosFlag45), (uint8_t)r6); mcu.pc = 0x542d; break;
    case 0x542d: MCU_SetStatus((r6 & 0x80u) == 0, STATUS_Z);    mcu.pc = 0x542f; break;
    case 0x542f: mcu.pc = (mcu.sr & STATUS_Z) ? 0x5436 : 0x5431; break;
    case 0x5431: store8(ind_addr(1, kAd0e), 0xff);              mcu.pc = 0x5436; break;
    case 0x5436: clr16(r3);                                     mcu.pc = 0x5438; break;
    case 0x5438: load8(r3, ind_addr(1, kSoaCe78));              mcu.pc = 0x543c; break;
    case 0x543c: store8(ind_addr(0, kAosToneIdx), (uint8_t)r3); mcu.pc = 0x5440; break;
    case 0x5440: add16(r3, r3);                                 mcu.pc = 0x5442; break;
    case 0x5442: load16(r3, ind_addr(3, kTonePtrTable));        mcu.pc = 0x5446; break;
    case 0x5446: store16(ind_addr(0, kAosToneRec), r3);         mcu.pc = 0x5449; break;
    case 0x5449: clr16(r3);                                     mcu.pc = 0x544b; break;
    case 0x544b:
    {
        uint8_t sel = MCU_Read(ind_addr(1, kSoaD0fc));
        MCU_SetStatusCommon(sel, 0);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = 0x544f;
        break;
    }
    case 0x544f: mcu.pc = (mcu.sr & STATUS_Z) ? 0x545a : 0x5451; break;
    case 0x5451: mcu.pc = (mcu.sr & STATUS_N) ? 0x5469 : 0x5453; break;
    case 0x5453: movi16(r3, kAosBasePos);                       mcu.pc = 0x5456; break;
    case 0x5456: MCU_PushStack(0x5458);                         mcu.pc = 0x546d; break;
    case 0x5458: mcu.pc = 0x545f; break;                        /* BRA +5 -> 545f */
    case 0x545a: movi16(r3, kAosBaseZero);                      mcu.pc = 0x545d; break;
    case 0x545d: MCU_PushStack(0x545f);                         mcu.pc = 0x546d; break;
    case 0x545f: clr16(r6);                                     mcu.pc = 0x5461; break;
    case 0x5461: load8(r6, ind_addr(1, kSoaCecc));              mcu.pc = 0x5465; break;
    case 0x5465: add16(r3, r6);                                 mcu.pc = 0x5467; break;
    case 0x5467: mcu.pc = 0x5469; break;                        /* BRA 0 -> 5469 */
    case 0x5469: store16(ind_addr(0, kAosToneVal), r3);         mcu.pc = 0x546c; break;
    case 0x546c: mcu.pc = MCU_PopStack(); break;                /* rts */
    case 0x546d: mcu.pc = MCU_PopStack(); break;                /* trampoline rts */
    default: break;
    }
    return 1;
}

/* ---- S2: coeff_copy 0x5d6e-0x5dc2 (per instruction) ---------------------- */

uint32_t routine_coeff_copy_step(void)
{
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x5d6e: load16(r6, ind_addr(2, (uint16_t)kCoef86));    mcu.pc = 0x5d72; break;
    case 0x5d72: store16(ind_addr(0, (uint16_t)kCoef86), r6);   mcu.pc = 0x5d76; break;
    case 0x5d76: load16(r6, ind_addr(2, (uint16_t)kCoef8a));    mcu.pc = 0x5d7a; break;
    case 0x5d7a: store16(ind_addr(0, (uint16_t)kCoef8a), r6);   mcu.pc = 0x5d7e; break;
    case 0x5d7e: load16(r6, ind_addr(2, (uint16_t)kCoef88));    mcu.pc = 0x5d82; break;
    case 0x5d82: store16(ind_addr(0, (uint16_t)kCoef88), r6);   mcu.pc = 0x5d86; break;
    case 0x5d86: load16(r6, ind_addr(2, (uint16_t)kCoefP80));   mcu.pc = 0x5d89; break;
    case 0x5d89: store16(ind_addr(0, (uint16_t)kCoefP80), r6);  mcu.pc = 0x5d8c; break;
    case 0x5d8c: load16(r6, ind_addr(2, (uint16_t)kCoefP114));  mcu.pc = 0x5d8f; break;
    case 0x5d8f: store16(ind_addr(0, (uint16_t)kCoefP114), r6); mcu.pc = 0x5d92; break;
    case 0x5d92: load16(r6, ind_addr(2, (uint16_t)kCoef96));    mcu.pc = 0x5d96; break;
    case 0x5d96: store16(ind_addr(0, (uint16_t)kCoef96), r6);   mcu.pc = 0x5d9a; break;
    case 0x5d9a: load16(r6, ind_addr(2, (uint16_t)kCoef8e));    mcu.pc = 0x5d9e; break;
    case 0x5d9e: store16(ind_addr(0, (uint16_t)kCoef8e), r6);   mcu.pc = 0x5da2; break;
    case 0x5da2: load16(r6, ind_addr(2, (uint16_t)kCoef94));    mcu.pc = 0x5da6; break;
    case 0x5da6: store16(ind_addr(0, (uint16_t)kCoef94), r6);   mcu.pc = 0x5daa; break;
    case 0x5daa: load16(r6, ind_addr(2, (uint16_t)kCoef8c));    mcu.pc = 0x5dae; break;
    case 0x5dae: store16(ind_addr(0, (uint16_t)kCoef8c), r6);   mcu.pc = 0x5db2; break;
    case 0x5db2: load16(r6, ind_addr(2, (uint16_t)kCoef92));    mcu.pc = 0x5db6; break;
    case 0x5db6: store16(ind_addr(0, (uint16_t)kCoef92), r6);   mcu.pc = 0x5dba; break;
    case 0x5dba: load16(r6, ind_addr(2, (uint16_t)kCoef90));    mcu.pc = 0x5dbe; break;
    case 0x5dbe: store16(ind_addr(0, (uint16_t)kCoef90), r6);   mcu.pc = 0x5dc2; break;
    case 0x5dc2: mcu.pc = MCU_PopStack(); break;                /* rts */
    default: break;
    }
    return 1;
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

uint32_t routine_tone_fields_step(void)
{
    uint16_t &r2 = mcu.r[2];

    switch (mcu.pc)
    {
    case 0x3580: mcu.ep = MCU_Read(ind_addr(0, 0x99));      mcu.ex_ignore = 1; mcu.pc = 0x3584; break;
    case 0x3584: load16(mcu.r[5], ind_addr(0, 0x9e));       mcu.pc = 0x3588; break;
    case 0x3588: clr16(r2);                                 mcu.pc = 0x358a; break;
    case 0x358a: load8(r2, ind_addr(5, 73));                mcu.pc = 0x358d; break;
    case 0x358d: mcu.pc = (mcu.sr & STATUS_N) ? 0x358f : 0x3598; break;
    case 0x358f: and8_imm(r2, 0x7f);                        mcu.pc = 0x3592; break;
    case 0x3592: swap16(r2);                                mcu.pc = 0x3594; break;
    case 0x3594: neg16(r2);                                 mcu.pc = 0x3596; break;
    case 0x3596: mcu.pc = 0x359a; break;
    case 0x3598: swap16(r2);                                mcu.pc = 0x359a; break;
    case 0x359a: store16(ind_addr(0, (uint16_t)-94), r2);   mcu.pc = 0x359d; break;
    case 0x359d: clr16(r2);                                 mcu.pc = 0x359f; break;

    case 0x359f: load8(r2, ind_addr(5, 0x48));              mcu.pc = 0x35a2; break;
    case 0x35a2: mcu.pc = (mcu.sr & STATUS_N) ? 0x35a4 : 0x35ad; break;
    case 0x35a4: and8_imm(r2, 0x7f);                        mcu.pc = 0x35a7; break;
    case 0x35a7: swap16(r2);                                mcu.pc = 0x35a9; break;
    case 0x35a9: neg16(r2);                                 mcu.pc = 0x35ab; break;
    case 0x35ab: mcu.pc = 0x35af; break;
    case 0x35ad: swap16(r2);                                mcu.pc = 0x35af; break;
    case 0x35af: store16(ind_addr(0, (uint16_t)-128), r2);  mcu.pc = 0x35b2; break;
    case 0x35b2: clr16(r2);                                 mcu.pc = 0x35b4; break;
    case 0x35b4: load8(r2, ind_addr(5, 0x2b));              mcu.pc = 0x35b7; break;
    case 0x35b7: mcu.pc = (mcu.sr & STATUS_N) ? 0x35b9 : 0x35c6; break;
    case 0x35b9: and8_imm(r2, 0x7f);                        mcu.pc = 0x35bc; break;
    case 0x35bc: add16(r2, r2);                             mcu.pc = 0x35be; break;
    case 0x35be: load16(r2, ind_addr(2, 0x6f86));           mcu.pc = 0x35c2; break;
    case 0x35c2: neg16(r2);                                 mcu.pc = 0x35c4; break;
    case 0x35c4: mcu.pc = 0x35cc; break;
    case 0x35c6: add16(r2, r2);                             mcu.pc = 0x35c8; break;
    case 0x35c8: load16(r2, ind_addr(2, 0x6f86));           mcu.pc = 0x35cc; break;
    case 0x35cc: store16(ind_addr(0, (uint16_t)-92), r2);   mcu.pc = 0x35cf; break;
    case 0x35cf: clr16(r2);                                 mcu.pc = 0x35d1; break;
    case 0x35d1: load8(r2, ind_addr(5, 0x2a));              mcu.pc = 0x35d4; break;
    case 0x35d4: mcu.pc = (mcu.sr & STATUS_N) ? 0x35d6 : 0x35e3; break;
    case 0x35d6: and8_imm(r2, 0x7f);                        mcu.pc = 0x35d9; break;
    case 0x35d9: add16(r2, r2);                             mcu.pc = 0x35db; break;
    case 0x35db: load16(r2, ind_addr(2, 0x6f86));           mcu.pc = 0x35df; break;
    case 0x35df: neg16(r2);                                 mcu.pc = 0x35e1; break;
    case 0x35e1: mcu.pc = 0x35e9; break;
    case 0x35e3: add16(r2, r2);                             mcu.pc = 0x35e5; break;
    case 0x35e5: load16(r2, ind_addr(2, 0x6f86));           mcu.pc = 0x35e9; break;
    case 0x35e9: store16(ind_addr(0, (uint16_t)-126), r2);  mcu.pc = 0x35ec; break;
    case 0x35ec: clr16(r2);                                 mcu.pc = 0x35ee; break;

    case 0x35ee: load8(r2, ind_addr(5, 0x0f));              mcu.pc = 0x35f1; break;
    case 0x35f1: tst8(r2);                                  mcu.pc = 0x35f3; break;
    case 0x35f3: mcu.pc = (mcu.sr & STATUS_N) ? 0x35f5 : 0x3602; break;
    case 0x35f5: and8_imm(r2, 0x7f);                        mcu.pc = 0x35f8; break;
    case 0x35f8: add16(r2, r2);                             mcu.pc = 0x35fa; break;
    case 0x35fa: load16(r2, ind_addr(2, 0x7086));           mcu.pc = 0x35fe; break;
    case 0x35fe: neg16(r2);                                 mcu.pc = 0x3600; break;
    case 0x3600: mcu.pc = 0x3608; break;
    case 0x3602: add16(r2, r2);                             mcu.pc = 0x3604; break;
    case 0x3604: load16(r2, ind_addr(2, 0x7086));           mcu.pc = 0x3608; break;
    case 0x3608: store16(ind_addr(0, (uint16_t)-90), r2);   mcu.pc = 0x360b; break;
    case 0x360b: clr16(r2);                                 mcu.pc = 0x360d; break;
    case 0x360d: load8(r2, ind_addr(5, 0x0e));              mcu.pc = 0x3610; break;
    case 0x3610: store16(ind_addr(0, 0xa8), r2);            mcu.pc = 0x3614; break;
    case 0x3614: mcu.pc = MCU_PopStack(); break;
    default: break;
    }
    return 1;
}

uint32_t routine_cross_copy_step(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x3a9e: load16(mcu.r[1], ind_addr(2, (uint16_t)-2));  mcu.pc = 0x3aa1; break;
    case 0x3aa1: add16(mcu.r[1], mcu.r[1]);                    mcu.pc = 0x3aa3; break;
    case 0x3aa3: load16(r6, ind_addr(1, 0xcdc6));               mcu.pc = 0x3aa7; break;
    case 0x3aa7: load16(mcu.r[1], ind_addr(0, (uint16_t)-2));  mcu.pc = 0x3aaa; break;
    case 0x3aaa: add16(mcu.r[1], mcu.r[1]);                    mcu.pc = 0x3aac; break;
    case 0x3aac: store16(ind_addr(1, 0xcdc6), r6);              mcu.pc = 0x3ab0; break;

    case 0x3ab0: load8(r6, ind_addr(2, (uint16_t)-25));         mcu.pc = 0x3ab3; break;
    case 0x3ab3: store8(ind_addr(0, (uint16_t)-25), (uint8_t)r6); mcu.pc = 0x3ab6; break;
    case 0x3ab6: load16(r6, ind_addr(2, (uint16_t)-112));       mcu.pc = 0x3ab9; break;
    case 0x3ab9: store16(ind_addr(0, (uint16_t)-112), r6);      mcu.pc = 0x3abc; break;
    case 0x3abc: load16(r6, ind_addr(2, (uint16_t)-110));       mcu.pc = 0x3abf; break;
    case 0x3abf: store16(ind_addr(0, (uint16_t)-110), r6);      mcu.pc = 0x3ac2; break;
    case 0x3ac2: load16(r6, ind_addr(2, (uint16_t)-108));       mcu.pc = 0x3ac5; break;
    case 0x3ac5: store16(ind_addr(0, (uint16_t)-108), r6);      mcu.pc = 0x3ac8; break;

    case 0x3ac8: load16(r6, ind_addr(2, (uint16_t)-106));       mcu.pc = 0x3acb; break;
    case 0x3acb: store16(ind_addr(0, (uint16_t)-106), r6);      mcu.pc = 0x3ace; break;
    case 0x3ace: load16(r6, ind_addr(2, (uint16_t)-104));       mcu.pc = 0x3ad1; break;
    case 0x3ad1: store16(ind_addr(0, (uint16_t)-104), r6);      mcu.pc = 0x3ad4; break;
    case 0x3ad4: load16(r6, ind_addr(2, (uint16_t)-102));       mcu.pc = 0x3ad7; break;
    case 0x3ad7: store16(ind_addr(0, (uint16_t)-102), r6);      mcu.pc = 0x3ada; break;
    case 0x3ada: load16(r6, ind_addr(2, (uint16_t)-100));       mcu.pc = 0x3add; break;
    case 0x3add: store16(ind_addr(0, (uint16_t)-100), r6);      mcu.pc = 0x3ae0; break;
    case 0x3ae0: load16(r6, ind_addr(2, (uint16_t)-98));        mcu.pc = 0x3ae3; break;
    case 0x3ae3: store16(ind_addr(0, (uint16_t)-98), r6);       mcu.pc = 0x3ae6; break;
    case 0x3ae6: load16(r6, ind_addr(2, (uint16_t)-96));        mcu.pc = 0x3ae9; break;
    case 0x3ae9: store16(ind_addr(0, (uint16_t)-96), r6);       mcu.pc = 0x3aec; break;
    case 0x3aec: load8(r6, ind_addr(2, (uint16_t)-115));        mcu.pc = 0x3aef; break;
    case 0x3aef: store8(ind_addr(0, (uint16_t)-115), (uint8_t)r6); mcu.pc = 0x3af2; break;
    case 0x3af2: load16(r6, ind_addr(2, (uint16_t)-114));       mcu.pc = 0x3af5; break;
    case 0x3af5: store16(ind_addr(0, (uint16_t)-114), r6);      mcu.pc = 0x3af8; break;

    case 0x3af8: load16(r6, ind_addr(0, (uint16_t)-104));       mcu.pc = 0x3afb; break;
    case 0x3afb: cmp16(r6, 0xffff);                             mcu.pc = 0x3afe; break;
    case 0x3afe: mcu.pc = (mcu.sr & STATUS_Z) ? 0x3b0a : 0x3b00; break;
    case 0x3b00: clr16_mem(ind_addr(0, (uint16_t)-122));        mcu.pc = 0x3b03; break;
    case 0x3b03: clr16_mem(ind_addr(0, (uint16_t)-120));        mcu.pc = 0x3b06; break;
    case 0x3b06: clr16_mem(ind_addr(0, (uint16_t)-118));        mcu.pc = 0x3b09; break;
    case 0x3b09: mcu.pc = MCU_PopStack(); break;

    case 0x3b0a: load16(r6, ind_addr(0, (uint16_t)-102));       mcu.pc = 0x3b0d; break;
    case 0x3b0d: cmp16(r6, 0xffff);                             mcu.pc = 0x3b10; break;
    case 0x3b10: mcu.pc = (mcu.sr & STATUS_Z) ? 0x3ba2 : 0x3b13; break;
    case 0x3b13: load16(r2, ind_addr(0, (uint16_t)-128));       mcu.pc = 0x3b16; break;
    case 0x3b16: mcu.pc = (mcu.sr & STATUS_N) ? 0x3b18 : 0x3b20; break;
    case 0x3b18: neg16(r2);                                     mcu.pc = 0x3b1a; break;
    case 0x3b1a: mulxu16(r6, r2, mcu.r[3]);                     mcu.pc = 0x3b1c; break;
    case 0x3b1c: neg16(r2);                                     mcu.pc = 0x3b1e; break;
    case 0x3b1e: mcu.pc = 0x3b22; break;
    case 0x3b20: mulxu16(r6, r2, mcu.r[3]);                     mcu.pc = 0x3b22; break;
    case 0x3b22: store16(ind_addr(0, (uint16_t)-122), r2);      mcu.pc = 0x3b25; break;
    case 0x3b25: load16(r2, ind_addr(0, (uint16_t)-126));       mcu.pc = 0x3b28; break;
    case 0x3b28: mcu.pc = (mcu.sr & STATUS_N) ? 0x3b2a : 0x3b32; break;
    case 0x3b2a: neg16(r2);                                     mcu.pc = 0x3b2c; break;
    case 0x3b2c: mulxu16(r6, r2, mcu.r[3]);                     mcu.pc = 0x3b2e; break;
    case 0x3b2e: neg16(r2);                                     mcu.pc = 0x3b30; break;
    case 0x3b30: mcu.pc = 0x3b34; break;
    case 0x3b32: mulxu16(r6, r2, mcu.r[3]);                     mcu.pc = 0x3b34; break;
    case 0x3b34: store16(ind_addr(0, (uint16_t)-120), r2);      mcu.pc = 0x3b37; break;

    case 0x3b37: clr16(mcu.r[3]);                               mcu.pc = 0x3b39; break;
    case 0x3b39: load16(r2, ind_addr(0, 46));                   mcu.pc = 0x3b3c; break;
    case 0x3b3c: load8(mcu.r[3], ind_addr(2, 17));              mcu.pc = 0x3b3f; break;
    case 0x3b3f: load16(r2, ind_addr(0, 0xa8));                 mcu.pc = 0x3b43; break;
    case 0x3b43: tst8(r2);                                      mcu.pc = 0x3b45; break;
    case 0x3b45: mcu.pc = (mcu.sr & STATUS_N) ? 0x3b47 : 0x3b51; break;
    case 0x3b47: and8_imm(r2, 0x7f);                            mcu.pc = 0x3b4a; break;
    case 0x3b4a: sub8_imm(mcu.r[3], 0x40);                      mcu.pc = 0x3b4d; break;
    case 0x3b4d: mcu.pc = (mcu.sr & STATUS_C) ? 0x3b4f : 0x3b78; break;
    case 0x3b4f: mcu.pc = 0x3b6c; break;
    case 0x3b51: sub8_imm(mcu.r[3], 0x40);                      mcu.pc = 0x3b54; break;
    case 0x3b54: mcu.pc = (mcu.sr & STATUS_C) ? 0x3b56 : 0x3b62; break;
    case 0x3b56: neg8(mcu.r[3]);                                mcu.pc = 0x3b58; break;
    case 0x3b58: add8(mcu.r[3], mcu.r[3]);                      mcu.pc = 0x3b5a; break;
    case 0x3b5a: sub8(r2, mcu.r[3]);                            mcu.pc = 0x3b5c; break;
    case 0x3b5c: mcu.pc = (mcu.sr & STATUS_N) ? 0x3b5e : 0x3b93; break;
    case 0x3b5e: clr8_reg(r2);                                  mcu.pc = 0x3b60; break;
    case 0x3b60: mcu.pc = 0x3b93; break;
    case 0x3b62: add8(mcu.r[3], mcu.r[3]);                      mcu.pc = 0x3b64; break;
    case 0x3b64: add8(r2, mcu.r[3]);                            mcu.pc = 0x3b66; break;
    case 0x3b66: mcu.pc = (mcu.sr & STATUS_N) ? 0x3b68 : 0x3b93; break;
    case 0x3b68: move8(r2, 0x7f);                               mcu.pc = 0x3b6a; break;
    case 0x3b6a: mcu.pc = 0x3b93; break;
    case 0x3b6c: neg8(mcu.r[3]);                                mcu.pc = 0x3b6e; break;
    case 0x3b6e: add8(mcu.r[3], mcu.r[3]);                      mcu.pc = 0x3b70; break;
    case 0x3b70: add8(r2, mcu.r[3]);                            mcu.pc = 0x3b72; break;
    case 0x3b72: mcu.pc = (mcu.sr & STATUS_N) ? 0x3b74 : 0x3b80; break;
    case 0x3b74: move8(r2, 0x7f);                               mcu.pc = 0x3b76; break;
    case 0x3b76: mcu.pc = 0x3b80; break;
    case 0x3b78: add8(mcu.r[3], mcu.r[3]);                      mcu.pc = 0x3b7a; break;
    case 0x3b7a: sub8(r2, mcu.r[3]);                            mcu.pc = 0x3b7c; break;
    case 0x3b7c: mcu.pc = (mcu.sr & STATUS_N) ? 0x3b7e : 0x3b80; break;
    case 0x3b7e: clr8_reg(r2);                                  mcu.pc = 0x3b80; break;

    case 0x3b80: add16(r2, r2);                                 mcu.pc = 0x3b82; break;
    case 0x3b82: load16(r2, ind_addr(2, 0x7086));               mcu.pc = 0x3b86; break;
    case 0x3b86: neg16(r2);                                     mcu.pc = 0x3b88; break;
    case 0x3b88: store16(ind_addr(0, (uint16_t)-124), r2);      mcu.pc = 0x3b8b; break;
    case 0x3b8b: neg16(r2);                                     mcu.pc = 0x3b8d; break;
    case 0x3b8d: mulxu16(r6, r2, mcu.r[3]);                     mcu.pc = 0x3b8f; break;
    case 0x3b8f: neg16(r2);                                     mcu.pc = 0x3b91; break;
    case 0x3b91: mcu.pc = 0x3b9e; break;
    case 0x3b93: add16(r2, r2);                                 mcu.pc = 0x3b95; break;
    case 0x3b95: load16(r2, ind_addr(2, 0x7086));               mcu.pc = 0x3b99; break;
    case 0x3b99: store16(ind_addr(0, (uint16_t)-124), r2);      mcu.pc = 0x3b9c; break;
    case 0x3b9c: mulxu16(r6, r2, mcu.r[3]);                     mcu.pc = 0x3b9e; break;
    case 0x3b9e: store16(ind_addr(0, (uint16_t)-118), r2);      mcu.pc = 0x3ba1; break;
    case 0x3ba1: mcu.pc = MCU_PopStack(); break;

    case 0x3ba2: load16(r6, ind_addr(0, (uint16_t)-128));       mcu.pc = 0x3ba5; break;
    case 0x3ba5: store16(ind_addr(0, (uint16_t)-122), r6);      mcu.pc = 0x3ba8; break;
    case 0x3ba8: load16(r6, ind_addr(0, (uint16_t)-126));       mcu.pc = 0x3bab; break;
    case 0x3bab: store16(ind_addr(0, (uint16_t)-120), r6);      mcu.pc = 0x3bae; break;
    case 0x3bae: movi16(r6, 0xffff);                            mcu.pc = 0x3bb1; break;
    case 0x3bb1: mcu.pc = 0x3b37; break;
    default: break;
    }
    return 1;
}

uint32_t routine_gate_setup_step(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];
    uint16_t &r3 = mcu.r[3];
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x3615: mcu.ep = MCU_Read(ind_addr(0, 0x98));         mcu.ex_ignore = 1; mcu.pc = 0x3619; break;
    case 0x3619: load16(r5, ind_addr(0, 0x9c));                mcu.pc = 0x361d; break;
    case 0x361d: load16(r1, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x3620; break;
    case 0x3620: add16(r1, r1);                                mcu.pc = 0x3622; break;
    case 0x3622: clr16_mem(ind_addr(1, 0xcdc6));               mcu.pc = 0x3626; break;
    case 0x3626: clr8_mem(ind_addr(0, (uint16_t)-115));        mcu.pc = 0x3629; break;
    case 0x3629: load8(r6, ind_addr(5, 14));                   mcu.pc = 0x362c; break;
    case 0x362c: MCU_SetStatus((r6 & 0x10u) == 0, STATUS_Z);   mcu.pc = 0x362e; break;
    case 0x362e: mcu.pc = (mcu.sr & STATUS_Z) ? 0x3666 : 0x3630; break;

    /* 0x3630..0x3663: slot scan (not entered in the captured runs). */
    case 0x3630: load8(r3, ind_addr(0, 0x9b));                 mcu.pc = 0x3634; break;
    case 0x3634: stc_ep_to_r4();                               mcu.pc = 0x3636; break;
    case 0x3636: movi16(r1, 0x001b);                           mcu.pc = 0x3639; break;
    case 0x3639: move16(r2, r1);                               mcu.pc = 0x363b; break;
    case 0x363b: add16(r2, r2);                                mcu.pc = 0x363d; break;
    case 0x363d: load16(r2, ind_addr(2, 0x64d6));              mcu.pc = 0x3641; break;
    case 0x3641: cmp16(r2, mcu.r[0]);                          mcu.pc = 0x3643; break;
    case 0x3643: mcu.pc = (mcu.sr & STATUS_Z) ? 0x365e : 0x3645; break;
    case 0x3645: MCU_Write16(ind_addr(2, 0), 0x000c);
                 MCU_SetStatusCommon(0x000c, 1);               mcu.pc = 0x364a; break;
    case 0x364a: mcu.pc = ((mcu.sr & (STATUS_C | STATUS_Z)) == 0) ? 0x365e : 0x364c; break;
    case 0x364c: cmp8(r3, MCU_Read(ind_addr(2, 0x9b)));        mcu.pc = 0x3650; break;
    case 0x3650: mcu.pc = (mcu.sr & STATUS_Z) ? 0x3652 : 0x365e; break;
    case 0x3652: cmp8(mcu.r[4], MCU_Read(ind_addr(2, 0x98)));  mcu.pc = 0x3656; break;
    case 0x3656: mcu.pc = (mcu.sr & STATUS_Z) ? 0x3658 : 0x365e; break;
    case 0x3658: cmp16(r5, MCU_Read16(ind_addr(2, 0x9c)));     mcu.pc = 0x365c; break;
    case 0x365c: mcu.pc = (mcu.sr & STATUS_Z) ? 0x3663 : 0x365e; break;
    case 0x365e: r1 = (uint16_t)(r1 - 1);
                 mcu.pc = (r1 != 0xffff) ? 0x3639 : 0x3661;    break;
    case 0x3661: mcu.pc = 0x3666; break;
    case 0x3663: mcu.pc = 0x3a9e; break;

    case 0x3666: move8(r3, (uint8_t)r6);                       mcu.pc = 0x3668; break;
    case 0x3668: and16_imm(r3, 0x00c0);                        mcu.pc = 0x366c; break;
    case 0x366c: swap16(r3);                                   mcu.pc = 0x366e; break;
    case 0x366e: store16(ind_addr(0, (uint16_t)-106), r3);     mcu.pc = 0x3671; break;
    case 0x3671: move8(r3, (uint8_t)r6);                       mcu.pc = 0x3673; break;
    case 0x3673: and16_imm(r3, 0x000f);                        mcu.pc = 0x3677; break;
    case 0x3677: load8(r3, ind_addr(3, 0x7207));               mcu.pc = 0x367b; break;
    case 0x367b: store16(ind_addr(0, (uint16_t)-108), r3);     mcu.pc = 0x367e; break;
    case 0x367e: clr16_mem(ind_addr(0, (uint16_t)-98));        mcu.pc = 0x3681; break;
    case 0x3681: clr16_mem(ind_addr(0, (uint16_t)-104));       mcu.pc = 0x3684; break;
    case 0x3684: clr16_mem(ind_addr(0, (uint16_t)-102));       mcu.pc = 0x3687; break;
    case 0x3687: clr16_mem(ind_addr(0, (uint16_t)-96));        mcu.pc = 0x368a; break;
    case 0x368a: clr16_mem(ind_addr(0, (uint16_t)-122));       mcu.pc = 0x368d; break;
    case 0x368d: clr16_mem(ind_addr(0, (uint16_t)-120));       mcu.pc = 0x3690; break;
    case 0x3690: clr16_mem(ind_addr(0, (uint16_t)-118));       mcu.pc = 0x3693; break;
    case 0x3693: clr16(r3);                                    mcu.pc = 0x3695; break;
    case 0x3695: load8(r3, ind_addr(5, 16));                   mcu.pc = 0x3698; break;
    case 0x3698: mcu.pc = (mcu.sr & STATUS_N) ? 0x36c1 : 0x369a; break;
    case 0x369a: load16(r2, ind_addr(0, 46));                  mcu.pc = 0x369d; break;
    case 0x369d: load8(r2, ind_addr(2, 23));                   mcu.pc = 0x36a0; break;
    case 0x36a0: sub8_imm(r2, 0x40);                           mcu.pc = 0x36a3; break;
    case 0x36a3: mcu.pc = (mcu.sr & STATUS_C) ? 0x36a5 : 0x36b1; break;
    case 0x36a5: neg8(r2);                                     mcu.pc = 0x36a7; break;
    case 0x36a7: add8(r2, r2);                                 mcu.pc = 0x36a9; break;
    case 0x36a9: sub8(r3, r2);                                 mcu.pc = 0x36ab; break;
    case 0x36ab: mcu.pc = (mcu.sr & STATUS_N) ? 0x36ad : 0x36b9; break;
    case 0x36ad: clr8_reg(r3);                                 mcu.pc = 0x36af; break;
    case 0x36af: mcu.pc = 0x36b9; break;
    case 0x36b1: add8(r2, r2);                                 mcu.pc = 0x36b3; break;
    case 0x36b3: add8(r3, r2);                                 mcu.pc = 0x36b5; break;
    case 0x36b5: mcu.pc = (mcu.sr & STATUS_N) ? 0x36b7 : 0x36b9; break;
    case 0x36b7: move8(r3, 0x7f);                              mcu.pc = 0x36b9; break;
    case 0x36b9: add16(r3, r3);                                mcu.pc = 0x36bb; break;
    case 0x36bb: load16(r3, ind_addr(3, 0x6e86));              mcu.pc = 0x36bf; break;
    case 0x36bf: mcu.pc = 0x36c3; break;
    case 0x36c1: clr16(r3);                                    mcu.pc = 0x36c3; break;
    case 0x36c3: store16(ind_addr(0, (uint16_t)-112), r3);     mcu.pc = 0x36c6; break;
    case 0x36c6: clr16(r3);                                    mcu.pc = 0x36c8; break;
    case 0x36c8: load8(r3, ind_addr(5, 17));                   mcu.pc = 0x36cb; break;
    case 0x36cb: add16(r3, r3);                                mcu.pc = 0x36cd; break;
    case 0x36cd: load16(r3, ind_addr(3, 0x6e86));              mcu.pc = 0x36d1; break;
    case 0x36d1: store16(ind_addr(0, (uint16_t)-110), r3);     mcu.pc = 0x36d4; break;
    case 0x36d4: load8(r3, ind_addr(5, 15));                   mcu.pc = 0x36d7; break;
    case 0x36d7: store8(ind_addr(0, (uint16_t)-25), (uint8_t)r3); mcu.pc = 0x36da; break;
    case 0x36da: load16(r6, dp_addr(0xad2a));                  mcu.pc = 0x36de; break;
    case 0x36de: store16(dp_addr(0xad2c), r6);                 mcu.pc = 0x36e2; break;
    case 0x36e2: MCU_Write16(dp_addr(0xad2a), 0x0001);
                 MCU_SetStatusCommon(0x0001, 1);               mcu.pc = 0x36e8; break;
    case 0x36e8: mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x36ec; break;
    case 0x36ec: MCU_Write(br_addr(0x3e), 0x1e);
                 MCU_SetStatusCommon(0x1e, 0);                 mcu.pc = 0x36f0; break;
    case 0x36f0: { uint8_t v = MCU_Read(br_addr(0x34)); move8(mcu.r[4], v); }
                                                               mcu.pc = 0x36f2; break;
    case 0x36f2: load16(mcu.r[4], br_addr(0x3a));              mcu.pc = 0x36f4; break;
    case 0x36f4: store16(ind_addr(0, (uint16_t)-100), mcu.r[4]); mcu.pc = 0x36f7; break;
    case 0x36f7: store16(ind_addr(0, (uint16_t)-98), mcu.r[4]); mcu.pc = 0x36fa; break;
    case 0x36fa: MCU_PushStack(0x36fc);                        mcu.pc = 0x3709; break;
    case 0x36fc: mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x3700; break;
    case 0x3700: load16(r6, dp_addr(0xad2c));                  mcu.pc = 0x3704; break;
    case 0x3704: store16(dp_addr(0xad2a), r6);                 mcu.pc = 0x3708; break;
    case 0x3708: mcu.pc = MCU_PopStack(); break;
    default: break;
    }
    return 1;
}

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

/* ---- S7: mask_set 0x546e-0x5491 + shared tail 0x5492-0x54cb ------------- */

/* mask_set is IML=0; stock is interrupted before 0x5474 and before the 0x5491
 * rts, so both ranges are per-PC entries (each returns 1). The tail at 0x5492
 * is shared with flush_off's pending-command exit: it pops the caller's return
 * address (ADDQ #2 r7) and re-joins the pcm_play epilogue at 0x51fc. */

uint32_t routine_mask_set_step(void)
{
    uint16_t &r1 = mcu.r[1];

    switch (mcu.pc)
    {
    case 0x546e: load16(r1, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x5471; break;
    case 0x5471: cmp16(r1, 0x000f);                            mcu.pc = 0x5474; break;
    case 0x5474: mcu.pc = ((mcu.sr & (STATUS_C | STATUS_Z)) != 0) ? 0x5480 : 0x5476; break;
    case 0x5476: sub_imm16(r1, 0x0010);                        mcu.pc = 0x547a; break;
    case 0x547a: bset16_mem(dp_addr(0xd154), r1);              mcu.pc = 0x547e; break;
    case 0x547e: mcu.pc = 0x5484; break;
    case 0x5480: bset16_mem(dp_addr(0xd156), r1);              mcu.pc = 0x5484; break;
    case 0x5484: load16(r1, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x5487; break;
    case 0x5487: sub_mem_imm8(ind_addr(1, 0xd0e0), 0x00);      mcu.pc = 0x548c; break;
    case 0x548c: mcu.pc = (mcu.sr & STATUS_Z) ? 0x548e : 0x5492; break;
    case 0x548e: mcu.pc = 0x272e; break;
    case 0x5491: mcu.pc = MCU_PopStack(); break;
    default: break;
    }
    return 1;
}

uint32_t routine_mask_tail_step(void)
{
    uint16_t &r0 = mcu.r[0];
    uint16_t &r1 = mcu.r[1];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x5492: load16(r1, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x5495; break;
    case 0x5495: addq_r7(2);                                   mcu.pc = 0x5497; break;
    case 0x5497: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x549b; break;
    case 0x549b: sub_mem_imm16(ind_addr(0, 0), 0x0018);        mcu.pc = 0x54a0; break;
    case 0x54a0: mcu.pc = (mcu.sr & STATUS_Z) ? 0x54a2 : 0x54aa; break;
    case 0x54a2: load16(r6, ind_addr(0, 6));                   mcu.pc = 0x54a5; break;
    case 0x54a5: store16(ind_addr(0, 0), r6);                  mcu.pc = 0x54a8; break;
    case 0x54a8: mcu.pc = 0x54ae; break;
    case 0x54aa: clr8_mem(ind_addr(1, 0xad0e));                mcu.pc = 0x54ae; break;
    case 0x54ae: load16(r0, dp_addr(0xd15a));                  mcu.pc = 0x54b2; break;
    case 0x54b2: mcu.pc = (mcu.sr & STATUS_N) ? 0x51fc : 0x54b5; break;
    case 0x54b5: sub_mem_imm16(ind_addr(0, 0), 0x0018);        mcu.pc = 0x54ba; break;
    case 0x54ba: mcu.pc = (mcu.sr & STATUS_Z) ? 0x54bc : 0x54c5; break;
    case 0x54bc: load16(r6, ind_addr(0, 6));                   mcu.pc = 0x54bf; break;
    case 0x54bf: store16(ind_addr(0, 0), r6);                  mcu.pc = 0x54c2; break;
    case 0x54c2: mcu.pc = 0x51fc; break;
    case 0x54c5: clr8_mem(ind_addr(1, 0xad0e));                mcu.pc = 0x54c9; break;
    case 0x54c9: mcu.pc = 0x51fc; break;
    default: break;
    }
    return 1;
}

/* flush_off 0x54fb-0x5521 (IML=7): per-PC. Clear the pending mask bits from
 * the committed mask, then hand off to the existing per-PC PCM flush handlers
 * (0x5525/27/29 in pcm_enable.cpp) and the 0x552b rts. The pending-command
 * exit (0x552c, unobserved in the captured runs) is left interpreted; like
 * the rest of S1-S10 this is one H8 instruction per step so -midiseq posts
 * and SM_Update keep stock cadence (round-6 L1 finding). */
uint32_t routine_flush_off_step(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r3 = mcu.r[3];
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x54fb: load16(r1, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x54fe; break;
    case 0x54fe: sub_mem_imm8(ind_addr(1, 0xd0e0), 0x00);      mcu.pc = 0x5503; break;
    case 0x5503: mcu.pc = (mcu.sr & STATUS_Z) ? 0x5505 : 0x552c; break;
    case 0x5505: load16(r5, dp_addr(0xd154));                  mcu.pc = 0x5509; break;
    case 0x5509: load16(r6, dp_addr(0xd156));                  mcu.pc = 0x550d; break;
    case 0x550d: not16(r5);                                    mcu.pc = 0x550f; break;
    case 0x550f: not16(r6);                                    mcu.pc = 0x5511; break;
    case 0x5511: load16(r3, dp_addr(0xd150));                  mcu.pc = 0x5515; break;
    case 0x5515: load16(r4, dp_addr(0xd152));                  mcu.pc = 0x5519; break;
    case 0x5519: and16(r3, r5);                                mcu.pc = 0x551b; break;
    case 0x551b: and16(r4, r6);                                mcu.pc = 0x551d; break;
    case 0x551d: store16(dp_addr(0xd150), r3);                 mcu.pc = 0x5521; break;
    case 0x5521: store16(dp_addr(0xd152), r4);                 mcu.pc = 0x5525; break;
    default: break;
    }
    return 1;
}

/* flush_on 0x564a-0x565e -> 0x5662 (IML=7), per-PC. */
uint32_t routine_flush_on_a_step(void)
{
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x564a: load16(r5, dp_addr(0xd150));                  mcu.pc = 0x564e; break;
    case 0x564e: load16(r6, dp_addr(0xd152));                  mcu.pc = 0x5652; break;
    case 0x5652: or16_mem(r5, dp_addr(0xd154));                mcu.pc = 0x5656; break;
    case 0x5656: or16_mem(r6, dp_addr(0xd156));                mcu.pc = 0x565a; break;
    case 0x565a: store16(dp_addr(0xd150), r5);                 mcu.pc = 0x565e; break;
    case 0x565e: store16(dp_addr(0xd152), r6);                 mcu.pc = 0x5662; break;
    default: break;
    }
    return 1;
}

/* flush_on 0x5668-0x5670 (after the 0x5662/64/66 PCM handlers), per-PC. */
uint32_t routine_flush_on_b_step(void)
{
    switch (mcu.pc)
    {
    case 0x5668: clr16_mem(dp_addr(0xd154));                   mcu.pc = 0x566c; break;
    case 0x566c: clr16_mem(dp_addr(0xd156));                   mcu.pc = 0x5670; break;
    case 0x5670: mcu.pc = MCU_PopStack(); break;               /* rts */
    default: break;
    }
    return 1;
}

/* ---- S8: param_write 0x5533-0x5625 (per instruction) --------------------- */

/* IML=7 (called between BSET_ORC 0x535e/0x53c7 and the exit BCLR_ANDC), no
 * observed internal interrupt. All PCM register traffic goes through
 * MCU_Write/Read (device routing + -pcmtrace). dp is cleared by each LDC as
 * in the ROM. Per-PC from round 6: the [229114164,229115004) span covered the
 * -midiseq byte due at 229114868, whose deferred post let the SM consume it
 * early (same class as the c212.565M materialize finding). */
uint32_t routine_param_write_step(void)
{
    uint16_t &r3 = mcu.r[3];
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x5533: load16(r3, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x5536; break;
    case 0x5536: clr8_mem(ind_addr(3, 0xce3f));                mcu.pc = 0x553a; break;
    case 0x553a: clr8_mem(ind_addr(3, 0xd15c));                mcu.pc = 0x553e; break;
    case 0x553e: movs8(r3, 0x3e);                              mcu.pc = 0x5540; break;
    case 0x5540: load16(r6, ind_addr(0, (uint16_t)-10));       mcu.pc = 0x5543; break;
    case 0x5543: movs16(r6, 0x1e);                             mcu.pc = 0x5545; break;
    case 0x5545: ldc_dp(0);                                    mcu.pc = 0x5548; break;
    case 0x5548: movg_imm8(dp_addr(0xfe6c), (uint8_t)r3);      mcu.pc = 0x554c; break;
    case 0x554c: store16(dp_addr(0xfe5e), r3);                 mcu.pc = 0x5550; break;
    case 0x5550: ldc_dp(0);                                    mcu.pc = 0x5553; break;
    case 0x5553: load8(r5, ind_addr(0, (uint16_t)-20));        mcu.pc = 0x5556; break;
    case 0x5556: load16(r6, ind_addr(0, (uint16_t)-16));       mcu.pc = 0x5559; break;
    case 0x5559: movs8(r5, 0x05);                              mcu.pc = 0x555b; break;
    case 0x555b: movs16(r6, 0x06);                             mcu.pc = 0x555d; break;
    case 0x555d: ldc_dp(0);                                    mcu.pc = 0x5560; break;
    case 0x5560: movg_imm8(dp_addr(0xfe6d), (uint8_t)r5);      mcu.pc = 0x5564; break;
    case 0x5564: store16(dp_addr(0xfe60), r6);                 mcu.pc = 0x5568; break;
    case 0x5568: ldc_dp(0);                                    mcu.pc = 0x556b; break;
    case 0x556b: load8(r5, ind_addr(0, (uint16_t)-18));        mcu.pc = 0x556e; break;
    case 0x556e: load16(r6, ind_addr(0, (uint16_t)-12));       mcu.pc = 0x5571; break;
    case 0x5571: movs8(r5, 0x09);                              mcu.pc = 0x5573; break;
    case 0x5573: movs16(r6, 0x0a);                             mcu.pc = 0x5575; break;
    case 0x5575: ldc_dp(0);                                    mcu.pc = 0x5578; break;
    case 0x5578: movg_imm8(dp_addr(0xfe6e), (uint8_t)r5);      mcu.pc = 0x557c; break;
    case 0x557c: store16(dp_addr(0xfe62), r6);                 mcu.pc = 0x5580; break;
    case 0x5580: ldc_dp(0);                                    mcu.pc = 0x5583; break;
    case 0x5583: load8(r5, ind_addr(0, (uint16_t)-19));        mcu.pc = 0x5586; break;
    case 0x5586: load16(r6, ind_addr(0, (uint16_t)-14));       mcu.pc = 0x5589; break;
    case 0x5589: movs8(r5, 0x0d);                              mcu.pc = 0x558b; break;
    case 0x558b: movs16(r6, 0x0e);                             mcu.pc = 0x558d; break;
    case 0x558d: ldc_dp(0);                                    mcu.pc = 0x5590; break;
    case 0x5590: movg_imm8(dp_addr(0xfe6f), (uint8_t)r5);      mcu.pc = 0x5594; break;
    case 0x5594: store16(dp_addr(0xfe64), r6);                 mcu.pc = 0x5598; break;
    case 0x5598: ldc_dp(0);                                    mcu.pc = 0x559b; break;
    case 0x559b: load16(r6, ind_addr(0, 52));                  mcu.pc = 0x559e; break;
    case 0x559e: movs16(r6, 0x12);                             mcu.pc = 0x55a0; break;
    case 0x55a0: ldc_dp(0);                                    mcu.pc = 0x55a3; break;
    case 0x55a3: store16(dp_addr(0xfe66), r6);                 mcu.pc = 0x55a7; break;
    case 0x55a7: ldc_dp(0);                                    mcu.pc = 0x55aa; break;
    case 0x55aa: load16(r6, ind_addr(0, 58));                  mcu.pc = 0x55ad; break;
    case 0x55ad: movs16(r6, 0x14);                             mcu.pc = 0x55af; break;
    case 0x55af: ldc_dp(0);                                    mcu.pc = 0x55b2; break;
    case 0x55b2: store16(dp_addr(0xfe68), r6);                 mcu.pc = 0x55b6; break;
    case 0x55b6: ldc_dp(0);                                    mcu.pc = 0x55b9; break;
    case 0x55b9: load8(r6, ind_addr(0, 104));                  mcu.pc = 0x55bc; break;
    case 0x55bc: swap16(r6);                                   mcu.pc = 0x55be; break;
    case 0x55be: load8(r6, ind_addr(0, 102));                  mcu.pc = 0x55c1; break;
    case 0x55c1: movs16(r6, 0x1c);                             mcu.pc = 0x55c3; break;
    case 0x55c3: load16(r6, ind_addr(0, (uint16_t)-24));       mcu.pc = 0x55c6; break;
    case 0x55c6: movs16(r6, 0x1a);                             mcu.pc = 0x55c8; break;
    case 0x55c8: load16(r6, ind_addr(0, 26));                  mcu.pc = 0x55cb; break;
    case 0x55cb: movs16(r6, 0x16);                             mcu.pc = 0x55cd; break;
    case 0x55cd: btsti8_mem(ind_addr(0, (uint16_t)-59), 7);    mcu.pc = 0x55d0; break;
    case 0x55d0: mcu.pc = (mcu.sr & STATUS_Z) ? 0x5608 : 0x55d2; break;
    case 0x55d2: tst16_mem(ind_addr(0, 14));                   mcu.pc = 0x55d5; break;
    case 0x55d5: mcu.pc = (mcu.sr & STATUS_Z) ? 0x55e2 : 0x55d7; break;
    case 0x55d7: movi16(r4, 0x0002);                           mcu.pc = 0x55da; break;
    case 0x55da: load16(r5, ind_addr(0, 30));                  mcu.pc = 0x55dd; break;
    case 0x55dd: load16(r6, ind_addr(0, 72));                  mcu.pc = 0x55e0; break;
    case 0x55e0: mcu.pc = 0x55ed; break;                       /* BRA 11 -> 55ed */
    case 0x55e2: movi16(r4, 0x0000);                           mcu.pc = 0x55e5; break;
    case 0x55e5: movi16(r5, 0x00b5);                           mcu.pc = 0x55e8; break;
    case 0x55e8: clr16(r6);                                    mcu.pc = 0x55ea; break;
    case 0x55ea: clr16_mem(ind_addr(0, 8));                    mcu.pc = 0x55ed; break;
    case 0x55ed: store16(ind_addr(0, 0), r4);                  mcu.pc = 0x55f0; break;
    case 0x55f0: store16(ind_addr(0, 2), r4);                  mcu.pc = 0x55f3; break;
    case 0x55f3: store16(ind_addr(0, 4), r4);                  mcu.pc = 0x55f6; break;
    case 0x55f6: movs16(r5, 0x18);                             mcu.pc = 0x55f8; break;
    case 0x55f8: store16(ind_addr(0, 30), r5);                 mcu.pc = 0x55fb; break;
    case 0x55fb: movs16(r6, 0x10);                             mcu.pc = 0x55fd; break;
    case 0x55fd: ldc_dp(0);                                    mcu.pc = 0x5600; break;
    case 0x5600: store16(dp_addr(0xfe6a), r6);                 mcu.pc = 0x5604; break;
    case 0x5604: ldc_dp(0);                                    mcu.pc = 0x5607; break;
    case 0x5607: mcu.pc = MCU_PopStack(); break;               /* rts */

    /* Alternate tail (cf90 bit7 clear), unobserved in the captured runs. */
    case 0x5608: load16(r5, ind_addr(0, 30));                  mcu.pc = 0x560b; break;
    case 0x560b: load16(r6, ind_addr(0, 72));                  mcu.pc = 0x560e; break;
    case 0x560e: movs16(r5, 0x18);                             mcu.pc = 0x5610; break;
    case 0x5610: movs16(r6, 0x10);                             mcu.pc = 0x5612; break;
    case 0x5612: ldc_dp(0);                                    mcu.pc = 0x5615; break;
    case 0x5615: store16(dp_addr(0xfe6a), r6);                 mcu.pc = 0x5619; break;
    case 0x5619: ldc_dp(0);                                    mcu.pc = 0x561c; break;
    case 0x561c: load16(r6, ind_addr(0, 6));                   mcu.pc = 0x561f; break;
    case 0x561f: store16(ind_addr(0, 0), r6);                  mcu.pc = 0x5622; break;
    case 0x5622: clr16_mem(ind_addr(0, 6));                    mcu.pc = 0x5625; break;
    case 0x5625: mcu.pc = MCU_PopStack(); break;               /* rts */
    default: break;
    }
    return 1;
}

/* loop_write 0x5626-0x5649 (IML=7): waits for PCM status 0x1e bit5 before
 * programming the loop registers. The busy-wait needs one host iteration per
 * instruction (PCM_Update/SM_Update run between steps), so it must stay
 * per-PC: a whole-routine block would spin without the device ever updating. */
uint32_t routine_loop_write_step(void)
{
    uint16_t &r1 = mcu.r[1];
    uint16_t &r5 = mcu.r[5];

    switch (mcu.pc)
    {
    case 0x5626: load16(r1, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x5629; break;
    case 0x5629: tst8_mem(ind_addr(0, 101));                   mcu.pc = 0x562c; break;
    case 0x562c: mcu.pc = (mcu.sr & STATUS_Z) ? 0x562e : 0x5649; break;
    case 0x562e: movs8(r1, 0x3e);                              mcu.pc = 0x5630; break;
    case 0x5630: movl8(r5, 0x1e);                              mcu.pc = 0x5632; break;
    case 0x5632: movl16(r5, 0x3a);                             mcu.pc = 0x5634; break;
    case 0x5634: btsti16_reg(r5, 5);                           mcu.pc = 0x5636; break;
    case 0x5636: mcu.pc = (mcu.sr & STATUS_Z) ? 0x5630 : 0x5638; break;
    case 0x5638: movg_imm16(br_addr(0x1a), 0xff00);            mcu.pc = 0x563d; break;
    case 0x563d: load16(r5, ind_addr(0, (uint16_t)-22));       mcu.pc = 0x5640; break;
    case 0x5640: shlr16(r5);                                   mcu.pc = 0x5642; break;
    case 0x5642: movs16(r5, 0x36);                             mcu.pc = 0x5644; break;
    case 0x5644: load16(r5, ind_addr(0, 38));                  mcu.pc = 0x5647; break;
    case 0x5647: movs16(r5, 0x1a);                             mcu.pc = 0x5649; break;
    case 0x5649: mcu.pc = MCU_PopStack(); break;
    default: break;
    }
    return 1;
}

/* ---- S9: param_ack 0x54cc-0x54fa ---------------------------------------- */

/* IML=0 after the first ORC/ANDC pair and carries the trapa #0x10 handshake
 * loop; per-PC so the trap is taken by the host exactly where stock takes it
 * and the PCM latches are read one instruction at a time. */

void tst16_reg(uint16_t reg)
{
    MCU_SetStatusCommon(reg, 1);
    MCU_SetStatus(0, STATUS_C);
}

uint32_t routine_param_ack_step(void)
{
    uint16_t &r0 = mcu.r[0];
    uint16_t &r2 = mcu.r[2];
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    case 0x54cc: mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x54d0; break;
    case 0x54d0: load16(r2, ind_addr(1, (uint16_t)-2));        mcu.pc = 0x54d3; break;
    case 0x54d3: movs8(r2, 0x3e);                              mcu.pc = 0x54d5; break;
    case 0x54d5: movl8(r5, 0x32);                              mcu.pc = 0x54d7; break;
    case 0x54d7: movl16(r5, 0x3a);                             mcu.pc = 0x54d9; break;
    case 0x54d9: movl8(r6, 0x34);                              mcu.pc = 0x54db; break;
    case 0x54db: movl16(r6, 0x3a);                             mcu.pc = 0x54dd; break;
    case 0x54dd: mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x54e1; break;
    case 0x54e1: sub_mem_imm8(ind_addr(2, 0xd0e0), 0x00);      mcu.pc = 0x54e6; break;
    case 0x54e6: mcu.pc = (mcu.sr & STATUS_Z) ? 0x54ec : 0x54e8; break;
    case 0x54e8: move16(r0, mcu.r[1]);                         mcu.pc = 0x54ea; break;
    case 0x54ea: mcu.pc = 0x5492; break;
    case 0x54ec: tst16_reg(r5);                                mcu.pc = 0x54ee; break;
    case 0x54ee: mcu.pc = (mcu.sr & STATUS_Z) ? 0x54fa : 0x54f0; break;
    case 0x54f0: tst16_reg(r6);                                mcu.pc = 0x54f2; break;
    case 0x54f2: mcu.pc = (mcu.sr & STATUS_Z) ? 0x54fa : 0x54f4; break;
    case 0x54f4: move8(r0, 0x80);                              mcu.pc = 0x54f6; break;
    case 0x54f6: MCU_Interrupt_TRAPA(0x10u & 0x0fu);           mcu.pc = 0x54f8; break;
    case 0x54f8: mcu.pc = 0x54cc; break;
    case 0x54fa: mcu.pc = MCU_PopStack(); break;
    default: break;
    }
    return 1;
}

/* ---- S10: pcm_play 0x52cb-0x53e8 ----------------------------------------- */

/* L0 per-PC: the routine orchestrates the already-hand child routines, so each
 * instruction runs as its own step and the host keeps dispatching (bsr targets
 * are the registered child entries; the two BRA 0x51fc exits do not pop the
 * stack). This keeps the IML=7 / IML=0 windows and the per-instruction device
 * updates exactly where stock has them. */

uint32_t routine_pcm_play_step(void)
{
    uint16_t &r0 = mcu.r[0];
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];

    switch (mcu.pc)
    {
    /* 52cb..: main (two-voice) path; d0a8/d0c4 neighbour check. */
    case 0x52cb: sub_mem_imm8(ind_addr(1, 0xd0a8), 0xff);      mcu.pc = 0x52d0; break;
    case 0x52d0: mcu.pc = (mcu.sr & STATUS_Z) ? 0x52d2 : 0x52e7; break;
    case 0x52d2: sub_mem_imm8(ind_addr(1, 0xd0c4), 0xff);      mcu.pc = 0x52d7; break;
    case 0x52d7: mcu.pc = (mcu.sr & STATUS_Z) ? 0x5390 : 0x52da; break;
    case 0x52da: clr16(r2);                                    mcu.pc = 0x52dc; break;
    case 0x52dc: load8(r2, ind_addr(1, 0xd0c4));               mcu.pc = 0x52e0; break;
    case 0x52e0: movg_imm8(ind_addr(2, 0xd0e0), 0x00);         mcu.pc = 0x52e5; break;
    case 0x52e5: mcu.pc = 0x52f4; break;
    case 0x52e7: clr16(r2);                                    mcu.pc = 0x52e9; break;
    case 0x52e9: load8(r2, ind_addr(1, 0xd0a8));               mcu.pc = 0x52ed; break;
    case 0x52ed: movg_imm8(ind_addr(2, 0xd0e0), 0x00);         mcu.pc = 0x52f2; break;
    case 0x52f2: { uint16_t t = r1; r1 = r2; r2 = t; }         mcu.pc = 0x52f4; break;
    case 0x52f4: MCU_PushStack(0x52f7);                        mcu.pc = 0x53eb; break;
    case 0x52f7: store16(dp_addr(0xd158), r0);                 mcu.pc = 0x52fb; break;
    case 0x52fb: move16(r1, r2);                               mcu.pc = 0x52fd; break;
    case 0x52fd: MCU_PushStack(0x5300);                        mcu.pc = 0x53eb; break;
    case 0x5300: store16(dp_addr(0xd15a), r0);                 mcu.pc = 0x5304; break;
    case 0x5304: mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x5308; break;
    case 0x5308: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x530c; break;
    case 0x530c: MCU_PushStack(0x530f);                        mcu.pc = 0x5998; break;
    case 0x530f: load16(r0, dp_addr(0xd15a));                  mcu.pc = 0x5313; break;
    case 0x5313: load16(r2, dp_addr(0xd158));                  mcu.pc = 0x5317; break;
    case 0x5317: MCU_PushStack(0x531a);                        mcu.pc = 0x5d6e; break;
    case 0x531a: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x531e; break;
    case 0x531e: load16(r1, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x5321; break;
    case 0x5321: btsti8_mem(ind_addr(0, (uint16_t)-59), 7);    mcu.pc = 0x5324; break;
    case 0x5324: mcu.pc = (mcu.sr & STATUS_Z) ? 0x5342 : 0x5326; break;
    case 0x5326: MCU_PushStack(0x5329);                        mcu.pc = 0x3580; break;
    case 0x5329: load16(r0, dp_addr(0xd15a));                  mcu.pc = 0x532d; break;
    case 0x532d: MCU_PushStack(0x5330);                        mcu.pc = 0x3580; break;
    case 0x5330: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x5334; break;
    case 0x5334: MCU_PushStack(0x5337);                        mcu.pc = 0x3615; break;
    case 0x5337: load16(r0, dp_addr(0xd15a));                  mcu.pc = 0x533b; break;
    case 0x533b: load16(r2, dp_addr(0xd158));                  mcu.pc = 0x533f; break;
    case 0x533f: MCU_PushStack(0x5342);                        mcu.pc = 0x3a9e; break;
    case 0x5342: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x5346; break;
    case 0x5346: MCU_PushStack(0x5349);                        mcu.pc = 0x546e; break;
    case 0x5349: load16(r0, dp_addr(0xd15a));                  mcu.pc = 0x534d; break;
    case 0x534d: MCU_PushStack(0x5350);                        mcu.pc = 0x546e; break;
    case 0x5350: load16(r1, dp_addr(0xd158));                  mcu.pc = 0x5354; break;
    case 0x5354: MCU_PushStack(0x5357);                        mcu.pc = 0x54cc; break;
    case 0x5357: load16(r1, dp_addr(0xd15a));                  mcu.pc = 0x535b; break;
    case 0x535b: MCU_PushStack(0x535e);                        mcu.pc = 0x54cc; break;
    case 0x535e: mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x5362; break;
    case 0x5362: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x5366; break;
    case 0x5366: MCU_PushStack(0x5369);                        mcu.pc = 0x54fb; break;
    case 0x5369: MCU_PushStack(0x536c);                        mcu.pc = 0x5533; break;
    case 0x536c: load16(r0, dp_addr(0xd15a));                  mcu.pc = 0x5370; break;
    case 0x5370: MCU_PushStack(0x5373);                        mcu.pc = 0x5533; break;
    case 0x5373: MCU_PushStack(0x5376);                        mcu.pc = 0x564a; break;
    case 0x5376: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x537a; break;
    case 0x537a: btsti8_mem(ind_addr(0, (uint16_t)-59), 7);    mcu.pc = 0x537d; break;
    case 0x537d: mcu.pc = (mcu.sr & STATUS_Z) ? 0x5389 : 0x537f; break;
    case 0x537f: MCU_PushStack(0x5382);                        mcu.pc = 0x5626; break;
    case 0x5382: load16(r0, dp_addr(0xd15a));                  mcu.pc = 0x5386; break;
    case 0x5386: MCU_PushStack(0x5389);                        mcu.pc = 0x5626; break;
    case 0x5389: mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x538d; break;
    case 0x538d: mcu.pc = 0x51fc; break;

    /* 5390..: single-neighbour branch. */
    case 0x5390: MCU_PushStack(0x5392);                        mcu.pc = 0x53eb; break;
    case 0x5392: store16(dp_addr(0xd158), r0);                 mcu.pc = 0x5396; break;
    case 0x5396: movg_imm16(dp_addr(0xd15a), 0xffff);          mcu.pc = 0x539c; break;
    case 0x539c: mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x53a0; break;
    case 0x53a0: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x53a4; break;
    case 0x53a4: MCU_PushStack(0x53a7);                        mcu.pc = 0x5998; break;
    case 0x53a7: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x53ab; break;
    case 0x53ab: load16(r1, ind_addr(0, (uint16_t)-2));        mcu.pc = 0x53ae; break;
    case 0x53ae: btsti8_mem(ind_addr(0, (uint16_t)-59), 7);    mcu.pc = 0x53b1; break;
    case 0x53b1: mcu.pc = (mcu.sr & STATUS_Z) ? 0x53bd : 0x53b3; break;
    case 0x53b3: MCU_PushStack(0x53b6);                        mcu.pc = 0x3580; break;
    case 0x53b6: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x53ba; break;
    case 0x53ba: MCU_PushStack(0x53bd);                        mcu.pc = 0x3615; break;
    case 0x53bd: MCU_PushStack(0x53c0);                        mcu.pc = 0x546e; break;
    case 0x53c0: load16(r1, dp_addr(0xd158));                  mcu.pc = 0x53c4; break;
    case 0x53c4: MCU_PushStack(0x53c7);                        mcu.pc = 0x54cc; break;
    case 0x53c7: mcu.sr = (uint16_t)((mcu.sr | 0x0700u) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x53cb; break;
    case 0x53cb: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x53cf; break;
    case 0x53cf: MCU_PushStack(0x53d2);                        mcu.pc = 0x54fb; break;
    case 0x53d2: MCU_PushStack(0x53d5);                        mcu.pc = 0x5533; break;
    case 0x53d5: MCU_PushStack(0x53d8);                        mcu.pc = 0x564a; break;
    case 0x53d8: load16(r0, dp_addr(0xd158));                  mcu.pc = 0x53dc; break;
    case 0x53dc: btsti8_mem(ind_addr(0, (uint16_t)-59), 7);    mcu.pc = 0x53df; break;
    case 0x53df: mcu.pc = (mcu.sr & STATUS_Z) ? 0x53e4 : 0x53e1; break;
    case 0x53e1: MCU_PushStack(0x53e4);                        mcu.pc = 0x5626; break;
    case 0x53e4: mcu.sr = (uint16_t)((mcu.sr & 0xf8ffu) & sr_mask);
                 mcu.ex_ignore = 1;                            mcu.pc = 0x53e8; break;
    case 0x53e8: mcu.pc = 0x51fc; break;
    default: break;
    }
    return 1;
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

uint32_t routine_coeff_calc_step(void)
{
    uint16_t &r2 = mcu.r[2];
    uint16_t &r3 = mcu.r[3];
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    switch (mcu.pc)
    {
    /* 5998..59ba prologue: tone pointer -> (dp,0xd17e), coeff base -> (dp,0xd180). */
    case 0x5998: clr16(r6);                                    mcu.pc = 0x599a; break;
    case 0x599a: load8(r6, ind_addr(1, 0xce78));               mcu.pc = 0x599e; break;
    case 0x599e: move16(r3, r6);                               mcu.pc = 0x59a0; break;
    case 0x59a0: add16(r3, r3);                                mcu.pc = 0x59a2; break;
    case 0x59a2: load16(r2, ind_addr(3, 0x7218));              mcu.pc = 0x59a6; break;
    case 0x59a6: store16(dp_addr(0xd17e), r2);                 mcu.pc = 0x59aa; break;
    case 0x59aa: move8(r4, 0x80);                              mcu.pc = 0x59ac; break;
    case 0x59ac: mulxu8_reg(r6, r4);                           mcu.pc = 0x59ae; break;
    case 0x59ae: clr16(r3);                                    mcu.pc = 0x59b0; break;
    case 0x59b0: load8(r3, ind_addr(1, 0xd134));               mcu.pc = 0x59b4; break;
    case 0x59b4: add16(r3, r4);                                mcu.pc = 0x59b6; break;
    case 0x59b6: load8(r5, ind_addr(3, 0x9740));               mcu.pc = 0x59ba; break;
    case 0x59ba: store8(dp_addr(0xd180), (uint8_t)r5);         mcu.pc = 0x59be; break;

    /* group P+0x86: tone[76], tables 9060/91c0/9320/9480/95e0, clamp 0xbe8,
     * <<3, *0xfbf8 (word). */
    case 0x59be: load8(r4, ind_addr(2, 76));                   mcu.pc = 0x59c1; break;
    case 0x59c1: sub8_imm(r4, 0x40);                           mcu.pc = 0x59c4; break;
    case 0x59c4: mcu.pc = (mcu.sr & STATUS_C) ? 0x59c6 : 0x59cf; break;
    case 0x59c6: neg8(r4);                                     mcu.pc = 0x59c8; break;
    case 0x59c8: mulxu8_reg(r5, r4);                           mcu.pc = 0x59ca; break;
    case 0x59ca: neg16(r4);                                    mcu.pc = 0x59cc; break;
    case 0x59cc: mcu.pc = 0x59d1; break;
    case 0x59ce: mcu.pc = MCU_PopStack(); break;
    case 0x59cf: mulxu8_reg(r5, r4);                           mcu.pc = 0x59d1; break;
    case 0x59d1: move16(r3, r6);                               mcu.pc = 0x59d3; break;
    case 0x59d3: add16(r3, r3);                                mcu.pc = 0x59d5; break;
    case 0x59d5: add16_mem(r4, ind_addr(3, 0x9060));           mcu.pc = 0x59d9; break;
    case 0x59d9: add16_mem(r4, ind_addr(3, 0x91c0));           mcu.pc = 0x59dd; break;
    case 0x59dd: add16_mem(r4, ind_addr(3, 0x9320));           mcu.pc = 0x59e1; break;
    case 0x59e1: add16_mem(r4, ind_addr(3, 0x9480));           mcu.pc = 0x59e5; break;
    case 0x59e5: add16_mem(r4, ind_addr(3, 0x95e0));           mcu.pc = 0x59e9; break;
    case 0x59e9: mcu.pc = (mcu.sr & STATUS_N) ? 0x59eb : 0x5a03; break;
    case 0x59eb: neg16(r4);                                    mcu.pc = 0x59ed; break;
    case 0x59ed: cmp16(r4, 0x0be8);                            mcu.pc = 0x59f0; break;
    case 0x59f0: mcu.pc = (mcu.sr & STATUS_C) ? 0x59f5 : 0x59f2; break;
    case 0x59f2: movi16(r4, 0x0be8);                           mcu.pc = 0x59f5; break;
    case 0x59f5: add16(r4, r4);                                mcu.pc = 0x59f7; break;
    case 0x59f7: add16(r4, r4);                                mcu.pc = 0x59f9; break;
    case 0x59f9: add16(r4, r4);                                mcu.pc = 0x59fb; break;
    case 0x59fb: mulxu16_imm(0xfbf8, r4, r5);                  mcu.pc = 0x59ff; break;
    case 0x59ff: neg16(r4);                                    mcu.pc = 0x5a01; break;
    case 0x5a01: mcu.pc = 0x5a15; break;
    case 0x5a03: cmp16(r4, 0x0be8);                            mcu.pc = 0x5a06; break;
    case 0x5a06: mcu.pc = (mcu.sr & STATUS_C) ? 0x5a0b : 0x5a08; break;
    case 0x5a08: movi16(r4, 0x0be8);                           mcu.pc = 0x5a0b; break;
    case 0x5a0b: add16(r4, r4);                                mcu.pc = 0x5a0d; break;
    case 0x5a0d: add16(r4, r4);                                mcu.pc = 0x5a0f; break;
    case 0x5a0f: add16(r4, r4);                                mcu.pc = 0x5a11; break;
    case 0x5a11: mulxu16_imm(0xfbf8, r4, r5);                  mcu.pc = 0x5a15; break;
    case 0x5a15: store16(ind_addr(0, 0x86), r4);               mcu.pc = 0x5a19; break;

    /* group P+0x8a: tone[78], tables 90a0/9200/9360/94c0/9620, clamp 0xfa0,
     * >>1, <<4, *0x820d. */
    case 0x5a19: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5a1d; break;
    case 0x5a1d: load8(r4, ind_addr(2, 78));                   mcu.pc = 0x5a20; break;
    case 0x5a20: sub8_imm(r4, 0x40);                           mcu.pc = 0x5a23; break;
    case 0x5a23: mcu.pc = (mcu.sr & STATUS_C) ? 0x5a25 : 0x5a31; break;
    case 0x5a25: neg8(r4);                                     mcu.pc = 0x5a27; break;
    case 0x5a27: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5a2b; break;
    case 0x5a2b: shlr16(r4);                                   mcu.pc = 0x5a2d; break;
    case 0x5a2d: neg16(r4);                                    mcu.pc = 0x5a2f; break;
    case 0x5a2f: mcu.pc = 0x5a37; break;
    case 0x5a31: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5a35; break;
    case 0x5a35: shlr16(r4);                                   mcu.pc = 0x5a37; break;
    case 0x5a37: add16_mem(r4, ind_addr(3, 0x90a0));           mcu.pc = 0x5a3b; break;
    case 0x5a3b: add16_mem(r4, ind_addr(3, 0x9200));           mcu.pc = 0x5a3f; break;
    case 0x5a3f: add16_mem(r4, ind_addr(3, 0x9360));           mcu.pc = 0x5a43; break;
    case 0x5a43: add16_mem(r4, ind_addr(3, 0x94c0));           mcu.pc = 0x5a47; break;
    case 0x5a47: add16_mem(r4, ind_addr(3, 0x9620));           mcu.pc = 0x5a4b; break;
    case 0x5a4b: mcu.pc = (mcu.sr & STATUS_N) ? 0x5a4d : 0x5a67; break;
    case 0x5a4d: neg16(r4);                                    mcu.pc = 0x5a4f; break;
    case 0x5a4f: cmp16(r4, 0x0fa0);                            mcu.pc = 0x5a52; break;
    case 0x5a52: mcu.pc = (mcu.sr & STATUS_C) ? 0x5a57 : 0x5a54; break;
    case 0x5a54: movi16(r4, 0x0fa0);                           mcu.pc = 0x5a57; break;
    case 0x5a57: add16(r4, r4);                                mcu.pc = 0x5a59; break;
    case 0x5a59: add16(r4, r4);                                mcu.pc = 0x5a5b; break;
    case 0x5a5b: add16(r4, r4);                                mcu.pc = 0x5a5d; break;
    case 0x5a5d: add16(r4, r4);                                mcu.pc = 0x5a5f; break;
    case 0x5a5f: mulxu16_imm(0x820d, r4, r5);                  mcu.pc = 0x5a63; break;
    case 0x5a63: neg16(r4);                                    mcu.pc = 0x5a65; break;
    case 0x5a65: mcu.pc = 0x5a7b; break;
    case 0x5a67: cmp16(r4, 0x0fa0);                            mcu.pc = 0x5a6a; break;
    case 0x5a6a: mcu.pc = (mcu.sr & STATUS_C) ? 0x5a6f : 0x5a6c; break;
    case 0x5a6c: movi16(r4, 0x0fa0);                           mcu.pc = 0x5a6f; break;
    case 0x5a6f: add16(r4, r4);                                mcu.pc = 0x5a71; break;
    case 0x5a71: add16(r4, r4);                                mcu.pc = 0x5a73; break;
    case 0x5a73: add16(r4, r4);                                mcu.pc = 0x5a75; break;
    case 0x5a75: add16(r4, r4);                                mcu.pc = 0x5a77; break;
    case 0x5a77: mulxu16_imm(0x820d, r4, r5);                  mcu.pc = 0x5a7b; break;
    case 0x5a7b: store16(ind_addr(0, 0x8a), r4);               mcu.pc = 0x5a7f; break;

    /* group P+0x88: tone[77], tables 9080/91e0/9340/94a0/9600, clamp 0xfa0,
     * >>1, <<3, *0xc49c. */
    case 0x5a7f: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5a83; break;
    case 0x5a83: load8(r4, ind_addr(2, 77));                   mcu.pc = 0x5a86; break;
    case 0x5a86: sub8_imm(r4, 0x40);                           mcu.pc = 0x5a89; break;
    case 0x5a89: mcu.pc = (mcu.sr & STATUS_C) ? 0x5a8b : 0x5a97; break;
    case 0x5a8b: neg8(r4);                                     mcu.pc = 0x5a8d; break;
    case 0x5a8d: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5a91; break;
    case 0x5a91: shlr16(r4);                                   mcu.pc = 0x5a93; break;
    case 0x5a93: neg16(r4);                                    mcu.pc = 0x5a95; break;
    case 0x5a95: mcu.pc = 0x5a9d; break;
    case 0x5a97: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5a9b; break;
    case 0x5a9b: shlr16(r4);                                   mcu.pc = 0x5a9d; break;
    case 0x5a9d: add16_mem(r4, ind_addr(3, 0x9080));           mcu.pc = 0x5aa1; break;
    case 0x5aa1: add16_mem(r4, ind_addr(3, 0x91e0));           mcu.pc = 0x5aa5; break;
    case 0x5aa5: add16_mem(r4, ind_addr(3, 0x9340));           mcu.pc = 0x5aa9; break;
    case 0x5aa9: add16_mem(r4, ind_addr(3, 0x94a0));           mcu.pc = 0x5aad; break;
    case 0x5aad: add16_mem(r4, ind_addr(3, 0x9600));           mcu.pc = 0x5ab1; break;
    case 0x5ab1: mcu.pc = (mcu.sr & STATUS_N) ? 0x5ab3 : 0x5acb; break;
    case 0x5ab3: neg16(r4);                                    mcu.pc = 0x5ab5; break;
    case 0x5ab5: cmp16(r4, 0x0fa0);                            mcu.pc = 0x5ab8; break;
    case 0x5ab8: mcu.pc = (mcu.sr & STATUS_C) ? 0x5abd : 0x5aba; break;
    case 0x5aba: movi16(r4, 0x0fa0);                           mcu.pc = 0x5abd; break;
    case 0x5abd: add16(r4, r4);                                mcu.pc = 0x5abf; break;
    case 0x5abf: add16(r4, r4);                                mcu.pc = 0x5ac1; break;
    case 0x5ac1: add16(r4, r4);                                mcu.pc = 0x5ac3; break;
    case 0x5ac3: mulxu16_imm(0xc49c, r4, r5);                  mcu.pc = 0x5ac7; break;
    case 0x5ac7: neg16(r4);                                    mcu.pc = 0x5ac9; break;
    case 0x5ac9: mcu.pc = 0x5add; break;
    case 0x5acb: cmp16(r4, 0x0fa0);                            mcu.pc = 0x5ace; break;
    case 0x5ace: mcu.pc = (mcu.sr & STATUS_C) ? 0x5ad3 : 0x5ad0; break;
    case 0x5ad0: movi16(r4, 0x0fa0);                           mcu.pc = 0x5ad3; break;
    case 0x5ad3: add16(r4, r4);                                mcu.pc = 0x5ad5; break;
    case 0x5ad5: add16(r4, r4);                                mcu.pc = 0x5ad7; break;
    case 0x5ad7: add16(r4, r4);                                mcu.pc = 0x5ad9; break;
    case 0x5ad9: mulxu16_imm(0xc49c, r4, r5);                  mcu.pc = 0x5add; break;
    case 0x5add: store16(ind_addr(0, 0x88), r4);               mcu.pc = 0x5ae1; break;

    /* group P-80: tone[84], tables 9140/92a0/9400/9560/96c0, clamp 0xfa0,
     * >>1, <<1, *0xa7c7. */
    case 0x5ae1: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5ae5; break;
    case 0x5ae5: load8(r4, ind_addr(2, 84));                   mcu.pc = 0x5ae8; break;
    case 0x5ae8: sub8_imm(r4, 0x40);                           mcu.pc = 0x5aeb; break;
    case 0x5aeb: mcu.pc = (mcu.sr & STATUS_C) ? 0x5aed : 0x5af9; break;
    case 0x5aed: neg8(r4);                                     mcu.pc = 0x5aef; break;
    case 0x5aef: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5af3; break;
    case 0x5af3: shlr16(r4);                                   mcu.pc = 0x5af5; break;
    case 0x5af5: neg16(r4);                                    mcu.pc = 0x5af7; break;
    case 0x5af7: mcu.pc = 0x5aff; break;
    case 0x5af9: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5afd; break;
    case 0x5afd: shlr16(r4);                                   mcu.pc = 0x5aff; break;
    case 0x5aff: add16_mem(r4, ind_addr(3, 0x9140));           mcu.pc = 0x5b03; break;
    case 0x5b03: add16_mem(r4, ind_addr(3, 0x92a0));           mcu.pc = 0x5b07; break;
    case 0x5b07: add16_mem(r4, ind_addr(3, 0x9400));           mcu.pc = 0x5b0b; break;
    case 0x5b0b: add16_mem(r4, ind_addr(3, 0x9560));           mcu.pc = 0x5b0f; break;
    case 0x5b0f: add16_mem(r4, ind_addr(3, 0x96c0));           mcu.pc = 0x5b13; break;
    case 0x5b13: mcu.pc = (mcu.sr & STATUS_N) ? 0x5b15 : 0x5b29; break;
    case 0x5b15: neg16(r4);                                    mcu.pc = 0x5b17; break;
    case 0x5b17: cmp16(r4, 0x0fa0);                            mcu.pc = 0x5b1a; break;
    case 0x5b1a: mcu.pc = (mcu.sr & STATUS_C) ? 0x5b1f : 0x5b1c; break;
    case 0x5b1c: movi16(r4, 0x0fa0);                           mcu.pc = 0x5b1f; break;
    case 0x5b1f: add16(r4, r4);                                mcu.pc = 0x5b21; break;
    case 0x5b21: mulxu16_imm(0xa7c7, r4, r5);                  mcu.pc = 0x5b25; break;
    case 0x5b25: neg16(r4);                                    mcu.pc = 0x5b27; break;
    case 0x5b27: mcu.pc = 0x5b37; break;
    case 0x5b29: cmp16(r4, 0x0fa0);                            mcu.pc = 0x5b2c; break;
    case 0x5b2c: mcu.pc = (mcu.sr & STATUS_C) ? 0x5b31 : 0x5b2e; break;
    case 0x5b2e: movi16(r4, 0x0fa0);                           mcu.pc = 0x5b31; break;
    case 0x5b31: add16(r4, r4);                                mcu.pc = 0x5b33; break;
    case 0x5b33: mulxu16_imm(0xa7c7, r4, r5);                  mcu.pc = 0x5b37; break;
    case 0x5b37: store16(ind_addr(0, (uint16_t)-80), r4);      mcu.pc = 0x5b3a; break;

    /* group P-114: tone[80], tables 90c0/9220/9380/94e0/9640, clamp 0xfa0,
     * >>1, <<1, *0xa7c7. */
    case 0x5b3a: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5b3e; break;
    case 0x5b3e: load8(r4, ind_addr(2, 80));                   mcu.pc = 0x5b41; break;
    case 0x5b41: sub8_imm(r4, 0x40);                           mcu.pc = 0x5b44; break;
    case 0x5b44: mcu.pc = (mcu.sr & STATUS_C) ? 0x5b46 : 0x5b52; break;
    case 0x5b46: neg8(r4);                                     mcu.pc = 0x5b48; break;
    case 0x5b48: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5b4c; break;
    case 0x5b4c: shlr16(r4);                                   mcu.pc = 0x5b4e; break;
    case 0x5b4e: neg16(r4);                                    mcu.pc = 0x5b50; break;
    case 0x5b50: mcu.pc = 0x5b58; break;
    case 0x5b52: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5b56; break;
    case 0x5b56: shlr16(r4);                                   mcu.pc = 0x5b58; break;
    case 0x5b58: add16_mem(r4, ind_addr(3, 0x90c0));           mcu.pc = 0x5b5c; break;
    case 0x5b5c: add16_mem(r4, ind_addr(3, 0x9220));           mcu.pc = 0x5b60; break;
    case 0x5b60: add16_mem(r4, ind_addr(3, 0x9380));           mcu.pc = 0x5b64; break;
    case 0x5b64: add16_mem(r4, ind_addr(3, 0x94e0));           mcu.pc = 0x5b68; break;
    case 0x5b68: add16_mem(r4, ind_addr(3, 0x9640));           mcu.pc = 0x5b6c; break;
    case 0x5b6c: mcu.pc = (mcu.sr & STATUS_N) ? 0x5b6e : 0x5b82; break;
    case 0x5b6e: neg16(r4);                                    mcu.pc = 0x5b70; break;
    case 0x5b70: cmp16(r4, 0x0fa0);                            mcu.pc = 0x5b73; break;
    case 0x5b73: mcu.pc = (mcu.sr & STATUS_C) ? 0x5b78 : 0x5b75; break;
    case 0x5b75: movi16(r4, 0x0fa0);                           mcu.pc = 0x5b78; break;
    case 0x5b78: add16(r4, r4);                                mcu.pc = 0x5b7a; break;
    case 0x5b7a: mulxu16_imm(0xa7c7, r4, r5);                  mcu.pc = 0x5b7e; break;
    case 0x5b7e: neg16(r4);                                    mcu.pc = 0x5b80; break;
    case 0x5b80: mcu.pc = 0x5b90; break;
    case 0x5b82: cmp16(r4, 0x0fa0);                            mcu.pc = 0x5b85; break;
    case 0x5b85: mcu.pc = (mcu.sr & STATUS_C) ? 0x5b8a : 0x5b87; break;
    case 0x5b87: movi16(r4, 0x0fa0);                           mcu.pc = 0x5b8a; break;
    case 0x5b8a: add16(r4, r4);                                mcu.pc = 0x5b8c; break;
    case 0x5b8c: mulxu16_imm(0xa7c7, r4, r5);                  mcu.pc = 0x5b90; break;
    case 0x5b90: store16(ind_addr(0, (uint16_t)-114), r4);     mcu.pc = 0x5b93; break;

    /* group P+0x96: tone[87] (no centre subtraction), tables
     * 91a0/9300/9460/95c0/9720, clamp 0xfc0, >>2, <<4, *0x8105. */
    case 0x5b93: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5b97; break;
    case 0x5b97: load8(r4, ind_addr(2, 87));                   mcu.pc = 0x5b9a; break;
    case 0x5b9a: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5b9e; break;
    case 0x5b9e: shlr16(r4);                                   mcu.pc = 0x5ba0; break;
    case 0x5ba0: shlr16(r4);                                   mcu.pc = 0x5ba2; break;
    case 0x5ba2: add16_mem(r4, ind_addr(3, 0x91a0));           mcu.pc = 0x5ba6; break;
    case 0x5ba6: add16_mem(r4, ind_addr(3, 0x9300));           mcu.pc = 0x5baa; break;
    case 0x5baa: add16_mem(r4, ind_addr(3, 0x9460));           mcu.pc = 0x5bae; break;
    case 0x5bae: add16_mem(r4, ind_addr(3, 0x95c0));           mcu.pc = 0x5bb2; break;
    case 0x5bb2: add16_mem(r4, ind_addr(3, 0x9720));           mcu.pc = 0x5bb6; break;
    case 0x5bb6: mcu.pc = (mcu.sr & STATUS_N) ? 0x5bb8 : 0x5bd2; break;
    case 0x5bb8: neg16(r4);                                    mcu.pc = 0x5bba; break;
    case 0x5bba: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5bbd; break;
    case 0x5bbd: mcu.pc = (mcu.sr & STATUS_C) ? 0x5bc2 : 0x5bbf; break;
    case 0x5bbf: movi16(r4, 0x0fc0);                           mcu.pc = 0x5bc2; break;
    case 0x5bc2: add16(r4, r4);                                mcu.pc = 0x5bc4; break;
    case 0x5bc4: add16(r4, r4);                                mcu.pc = 0x5bc6; break;
    case 0x5bc6: add16(r4, r4);                                mcu.pc = 0x5bc8; break;
    case 0x5bc8: add16(r4, r4);                                mcu.pc = 0x5bca; break;
    case 0x5bca: mulxu16_imm(0x8105, r4, r5);                  mcu.pc = 0x5bce; break;
    case 0x5bce: neg16(r4);                                    mcu.pc = 0x5bd0; break;
    case 0x5bd0: mcu.pc = 0x5be6; break;
    case 0x5bd2: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5bd5; break;
    case 0x5bd5: mcu.pc = (mcu.sr & STATUS_C) ? 0x5bda : 0x5bd7; break;
    case 0x5bd7: movi16(r4, 0x0fc0);                           mcu.pc = 0x5bda; break;
    case 0x5bda: add16(r4, r4);                                mcu.pc = 0x5bdc; break;
    case 0x5bdc: add16(r4, r4);                                mcu.pc = 0x5bde; break;
    case 0x5bde: add16(r4, r4);                                mcu.pc = 0x5be0; break;
    case 0x5be0: add16(r4, r4);                                mcu.pc = 0x5be2; break;
    case 0x5be2: mulxu16_imm(0x8105, r4, r5);                  mcu.pc = 0x5be6; break;
    case 0x5be6: store16(ind_addr(0, 0x96), r4);               mcu.pc = 0x5bea; break;

    /* group P+0x8e: tone[83], tables 9120/9280/93e0/9540/96a0, clamp 0xfc0,
     * >>2, <<4, *0x8105. */
    case 0x5bea: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5bee; break;
    case 0x5bee: load8(r4, ind_addr(2, 83));                   mcu.pc = 0x5bf1; break;
    case 0x5bf1: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5bf5; break;
    case 0x5bf5: shlr16(r4);                                   mcu.pc = 0x5bf7; break;
    case 0x5bf7: shlr16(r4);                                   mcu.pc = 0x5bf9; break;
    case 0x5bf9: add16_mem(r4, ind_addr(3, 0x9120));           mcu.pc = 0x5bfd; break;
    case 0x5bfd: add16_mem(r4, ind_addr(3, 0x9280));           mcu.pc = 0x5c01; break;
    case 0x5c01: add16_mem(r4, ind_addr(3, 0x93e0));           mcu.pc = 0x5c05; break;
    case 0x5c05: add16_mem(r4, ind_addr(3, 0x9540));           mcu.pc = 0x5c09; break;
    case 0x5c09: add16_mem(r4, ind_addr(3, 0x96a0));           mcu.pc = 0x5c0d; break;
    case 0x5c0d: mcu.pc = (mcu.sr & STATUS_N) ? 0x5c0f : 0x5c29; break;
    case 0x5c0f: neg16(r4);                                    mcu.pc = 0x5c11; break;
    case 0x5c11: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5c14; break;
    case 0x5c14: mcu.pc = (mcu.sr & STATUS_C) ? 0x5c19 : 0x5c16; break;
    case 0x5c16: movi16(r4, 0x0fc0);                           mcu.pc = 0x5c19; break;
    case 0x5c19: add16(r4, r4);                                mcu.pc = 0x5c1b; break;
    case 0x5c1b: add16(r4, r4);                                mcu.pc = 0x5c1d; break;
    case 0x5c1d: add16(r4, r4);                                mcu.pc = 0x5c1f; break;
    case 0x5c1f: add16(r4, r4);                                mcu.pc = 0x5c21; break;
    case 0x5c21: mulxu16_imm(0x8105, r4, r5);                  mcu.pc = 0x5c25; break;
    case 0x5c25: neg16(r4);                                    mcu.pc = 0x5c27; break;
    case 0x5c27: mcu.pc = 0x5c3d; break;
    case 0x5c29: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5c2c; break;
    case 0x5c2c: mcu.pc = (mcu.sr & STATUS_C) ? 0x5c31 : 0x5c2e; break;
    case 0x5c2e: movi16(r4, 0x0fc0);                           mcu.pc = 0x5c31; break;
    case 0x5c31: add16(r4, r4);                                mcu.pc = 0x5c33; break;
    case 0x5c33: add16(r4, r4);                                mcu.pc = 0x5c35; break;
    case 0x5c35: add16(r4, r4);                                mcu.pc = 0x5c37; break;
    case 0x5c37: add16(r4, r4);                                mcu.pc = 0x5c39; break;
    case 0x5c39: mulxu16_imm(0x8105, r4, r5);                  mcu.pc = 0x5c3d; break;
    case 0x5c3d: store16(ind_addr(0, 0x8e), r4);               mcu.pc = 0x5c41; break;

    /* group P+0x94: tone[86], tables 9180/92e0/9440/95a0/9700, clamp 0xfc0,
     * >>2, <<1, *0xc30d. */
    case 0x5c41: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5c45; break;
    case 0x5c45: load8(r4, ind_addr(2, 86));                   mcu.pc = 0x5c48; break;
    case 0x5c48: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5c4c; break;
    case 0x5c4c: shlr16(r4);                                   mcu.pc = 0x5c4e; break;
    case 0x5c4e: shlr16(r4);                                   mcu.pc = 0x5c50; break;
    case 0x5c50: add16_mem(r4, ind_addr(3, 0x9180));           mcu.pc = 0x5c54; break;
    case 0x5c54: add16_mem(r4, ind_addr(3, 0x92e0));           mcu.pc = 0x5c58; break;
    case 0x5c58: add16_mem(r4, ind_addr(3, 0x9440));           mcu.pc = 0x5c5c; break;
    case 0x5c5c: add16_mem(r4, ind_addr(3, 0x95a0));           mcu.pc = 0x5c60; break;
    case 0x5c60: add16_mem(r4, ind_addr(3, 0x9700));           mcu.pc = 0x5c64; break;
    case 0x5c64: mcu.pc = (mcu.sr & STATUS_N) ? 0x5c66 : 0x5c7a; break;
    case 0x5c66: neg16(r4);                                    mcu.pc = 0x5c68; break;
    case 0x5c68: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5c6b; break;
    case 0x5c6b: mcu.pc = (mcu.sr & STATUS_C) ? 0x5c70 : 0x5c6d; break;
    case 0x5c6d: movi16(r4, 0x0fc0);                           mcu.pc = 0x5c70; break;
    case 0x5c70: add16(r4, r4);                                mcu.pc = 0x5c72; break;
    case 0x5c72: mulxu16_imm(0xc30d, r4, r5);                  mcu.pc = 0x5c76; break;
    case 0x5c76: neg16(r4);                                    mcu.pc = 0x5c78; break;
    case 0x5c78: mcu.pc = 0x5c88; break;
    case 0x5c7a: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5c7d; break;
    case 0x5c7d: mcu.pc = (mcu.sr & STATUS_C) ? 0x5c82 : 0x5c7f; break;
    case 0x5c7f: movi16(r4, 0x0fc0);                           mcu.pc = 0x5c82; break;
    case 0x5c82: add16(r4, r4);                                mcu.pc = 0x5c84; break;
    case 0x5c84: mulxu16_imm(0xc30d, r4, r5);                  mcu.pc = 0x5c88; break;
    case 0x5c88: store16(ind_addr(0, 0x94), r4);               mcu.pc = 0x5c8c; break;

    /* group P+0x8c: tone[82], tables 9100/9260/93c0/9520/9680, clamp 0xfc0,
     * >>2, <<1, *0xc30d. */
    case 0x5c8c: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5c90; break;
    case 0x5c90: load8(r4, ind_addr(2, 82));                   mcu.pc = 0x5c93; break;
    case 0x5c93: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5c97; break;
    case 0x5c97: shlr16(r4);                                   mcu.pc = 0x5c99; break;
    case 0x5c99: shlr16(r4);                                   mcu.pc = 0x5c9b; break;
    case 0x5c9b: add16_mem(r4, ind_addr(3, 0x9100));           mcu.pc = 0x5c9f; break;
    case 0x5c9f: add16_mem(r4, ind_addr(3, 0x9260));           mcu.pc = 0x5ca3; break;
    case 0x5ca3: add16_mem(r4, ind_addr(3, 0x93c0));           mcu.pc = 0x5ca7; break;
    case 0x5ca7: add16_mem(r4, ind_addr(3, 0x9520));           mcu.pc = 0x5cab; break;
    case 0x5cab: add16_mem(r4, ind_addr(3, 0x9680));           mcu.pc = 0x5caf; break;
    case 0x5caf: mcu.pc = (mcu.sr & STATUS_N) ? 0x5cb1 : 0x5cc5; break;
    case 0x5cb1: neg16(r4);                                    mcu.pc = 0x5cb3; break;
    case 0x5cb3: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5cb6; break;
    case 0x5cb6: mcu.pc = (mcu.sr & STATUS_C) ? 0x5cbb : 0x5cb8; break;
    case 0x5cb8: movi16(r4, 0x0fc0);                           mcu.pc = 0x5cbb; break;
    case 0x5cbb: add16(r4, r4);                                mcu.pc = 0x5cbd; break;
    case 0x5cbd: mulxu16_imm(0xc30d, r4, r5);                  mcu.pc = 0x5cc1; break;
    case 0x5cc1: neg16(r4);                                    mcu.pc = 0x5cc3; break;
    case 0x5cc3: mcu.pc = 0x5cd3; break;
    case 0x5cc5: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5cc8; break;
    case 0x5cc8: mcu.pc = (mcu.sr & STATUS_C) ? 0x5ccd : 0x5cca; break;
    case 0x5cca: movi16(r4, 0x0fc0);                           mcu.pc = 0x5ccd; break;
    case 0x5ccd: add16(r4, r4);                                mcu.pc = 0x5ccf; break;
    case 0x5ccf: mulxu16_imm(0xc30d, r4, r5);                  mcu.pc = 0x5cd3; break;
    case 0x5cd3: store16(ind_addr(0, 0x8c), r4);               mcu.pc = 0x5cd7; break;

    /* group P+0x92: tone[85], tables 9160/92c0/9420/9580/96e0, clamp 0xfc0,
     * >>2, <<1, *0xbe7a. */
    case 0x5cd7: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5cdb; break;
    case 0x5cdb: load8(r4, ind_addr(2, 85));                   mcu.pc = 0x5cde; break;
    case 0x5cde: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5ce2; break;
    case 0x5ce2: shlr16(r4);                                   mcu.pc = 0x5ce4; break;
    case 0x5ce4: shlr16(r4);                                   mcu.pc = 0x5ce6; break;
    case 0x5ce6: add16_mem(r4, ind_addr(3, 0x9160));           mcu.pc = 0x5cea; break;
    case 0x5cea: add16_mem(r4, ind_addr(3, 0x92c0));           mcu.pc = 0x5cee; break;
    case 0x5cee: add16_mem(r4, ind_addr(3, 0x9420));           mcu.pc = 0x5cf2; break;
    case 0x5cf2: add16_mem(r4, ind_addr(3, 0x9580));           mcu.pc = 0x5cf6; break;
    case 0x5cf6: add16_mem(r4, ind_addr(3, 0x96e0));           mcu.pc = 0x5cfa; break;
    case 0x5cfa: mcu.pc = (mcu.sr & STATUS_N) ? 0x5cfc : 0x5d10; break;
    case 0x5cfc: neg16(r4);                                    mcu.pc = 0x5cfe; break;
    case 0x5cfe: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5d01; break;
    case 0x5d01: mcu.pc = (mcu.sr & STATUS_C) ? 0x5d06 : 0x5d03; break;
    case 0x5d03: movi16(r4, 0x0fc0);                           mcu.pc = 0x5d06; break;
    case 0x5d06: add16(r4, r4);                                mcu.pc = 0x5d08; break;
    case 0x5d08: mulxu16_imm(0xbe7a, r4, r5);                  mcu.pc = 0x5d0c; break;
    case 0x5d0c: neg16(r4);                                    mcu.pc = 0x5d0e; break;
    case 0x5d0e: mcu.pc = 0x5d1e; break;
    case 0x5d10: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5d13; break;
    case 0x5d13: mcu.pc = (mcu.sr & STATUS_C) ? 0x5d18 : 0x5d15; break;
    case 0x5d15: movi16(r4, 0x0fc0);                           mcu.pc = 0x5d18; break;
    case 0x5d18: add16(r4, r4);                                mcu.pc = 0x5d1a; break;
    case 0x5d1a: mulxu16_imm(0xbe7a, r4, r5);                  mcu.pc = 0x5d1e; break;
    case 0x5d1e: store16(ind_addr(0, 0x92), r4);               mcu.pc = 0x5d22; break;

    /* group P+0x90: tone[81], tables 90e0/9240/93a0/9500/9660, clamp 0xfc0,
     * >>2, <<1, *0xbe7a. */
    case 0x5d22: load16(r2, dp_addr(0xd17e));                  mcu.pc = 0x5d26; break;
    case 0x5d26: load8(r4, ind_addr(2, 81));                   mcu.pc = 0x5d29; break;
    case 0x5d29: mulxu8_mem(dp_addr(0xd180), r4);              mcu.pc = 0x5d2d; break;
    case 0x5d2d: shlr16(r4);                                   mcu.pc = 0x5d2f; break;
    case 0x5d2f: shlr16(r4);                                   mcu.pc = 0x5d31; break;
    case 0x5d31: add16_mem(r4, ind_addr(3, 0x90e0));           mcu.pc = 0x5d35; break;
    case 0x5d35: add16_mem(r4, ind_addr(3, 0x9240));           mcu.pc = 0x5d39; break;
    case 0x5d39: add16_mem(r4, ind_addr(3, 0x93a0));           mcu.pc = 0x5d3d; break;
    case 0x5d3d: add16_mem(r4, ind_addr(3, 0x9500));           mcu.pc = 0x5d41; break;
    case 0x5d41: add16_mem(r4, ind_addr(3, 0x9660));           mcu.pc = 0x5d45; break;
    case 0x5d45: mcu.pc = (mcu.sr & STATUS_N) ? 0x5d47 : 0x5d5b; break;
    case 0x5d47: neg16(r4);                                    mcu.pc = 0x5d49; break;
    case 0x5d49: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5d4c; break;
    case 0x5d4c: mcu.pc = (mcu.sr & STATUS_C) ? 0x5d51 : 0x5d4e; break;
    case 0x5d4e: movi16(r4, 0x0fc0);                           mcu.pc = 0x5d51; break;
    case 0x5d51: add16(r4, r4);                                mcu.pc = 0x5d53; break;
    case 0x5d53: mulxu16_imm(0xbe7a, r4, r5);                  mcu.pc = 0x5d57; break;
    case 0x5d57: neg16(r4);                                    mcu.pc = 0x5d59; break;
    case 0x5d59: mcu.pc = 0x5d69; break;
    case 0x5d5b: cmp16(r4, 0x0fc0);                            mcu.pc = 0x5d5e; break;
    case 0x5d5e: mcu.pc = (mcu.sr & STATUS_C) ? 0x5d63 : 0x5d60; break;
    case 0x5d60: movi16(r4, 0x0fc0);                           mcu.pc = 0x5d63; break;
    case 0x5d63: add16(r4, r4);                                mcu.pc = 0x5d65; break;
    case 0x5d65: mulxu16_imm(0xbe7a, r4, r5);                  mcu.pc = 0x5d69; break;
    case 0x5d69: store16(ind_addr(0, 0x90), r4);               mcu.pc = 0x5d6d; break;

    case 0x5d6d: mcu.pc = MCU_PopStack(); break;
    default: break;
    }
    return 1;
}

} /* anonymous namespace */

/* S1/S2 materialize + coeff_copy: one entry per instruction start PC (all
 * ran one H8 instruction per step from round 6 on; see the S1 comment). */
const uint16_t kMaterializePcs[] = {
    0x53eb, 0x53ed, 0x53ef, 0x53f3, 0x53f6, 0x53fa, 0x53fe, 0x5402,
    0x5406, 0x540a, 0x540e, 0x5412, 0x5416, 0x541a, 0x541e, 0x5422,
    0x5426, 0x542a, 0x542d, 0x542f, 0x5431, 0x5436, 0x5438, 0x543c,
    0x5440, 0x5442, 0x5446, 0x5449, 0x544b, 0x544f, 0x5451, 0x5453,
    0x5456, 0x5458, 0x545a, 0x545d, 0x545f, 0x5461, 0x5465, 0x5467,
    0x5469, 0x546c, 0x546d,
};

const uint16_t kCoeffCopyPcs[] = {
    0x5d6e, 0x5d72, 0x5d76, 0x5d7a, 0x5d7e, 0x5d82, 0x5d86, 0x5d89,
    0x5d8c, 0x5d8f, 0x5d92, 0x5d96, 0x5d9a, 0x5d9e, 0x5da2, 0x5da6,
    0x5daa, 0x5dae, 0x5db2, 0x5db6, 0x5dba, 0x5dbe, 0x5dc2,
};

/* Instruction start PCs of the three routines (registered one entry each). */
const uint16_t kToneFieldsPcs[] = {
    0x3580, 0x3584, 0x3588, 0x358a, 0x358d, 0x358f, 0x3592, 0x3594,
    0x3596, 0x3598, 0x359a, 0x359d, 0x359f, 0x35a2, 0x35a4, 0x35a7,
    0x35a9, 0x35ab, 0x35ad, 0x35af, 0x35b2, 0x35b4, 0x35b7, 0x35b9,
    0x35bc, 0x35be, 0x35c2, 0x35c4, 0x35c6, 0x35c8, 0x35cc, 0x35cf,
    0x35d1, 0x35d4, 0x35d6, 0x35d9, 0x35db, 0x35df, 0x35e1, 0x35e3,
    0x35e5, 0x35e9, 0x35ec, 0x35ee, 0x35f1, 0x35f3, 0x35f5, 0x35f8,
    0x35fa, 0x35fe, 0x3600, 0x3602, 0x3604, 0x3608, 0x360b, 0x360d,
    0x3610, 0x3614,
};

const uint16_t kCrossCopyPcs[] = {
    0x3a9e, 0x3aa1, 0x3aa3, 0x3aa7, 0x3aaa, 0x3aac, 0x3ab0, 0x3ab3,
    0x3ab6, 0x3ab9, 0x3abc, 0x3abf, 0x3ac2, 0x3ac5, 0x3ac8, 0x3acb,
    0x3ace, 0x3ad1, 0x3ad4, 0x3ad7, 0x3ada, 0x3add, 0x3ae0, 0x3ae3,
    0x3ae6, 0x3ae9, 0x3aec, 0x3aef, 0x3af2, 0x3af5, 0x3af8, 0x3afb,
    0x3afe, 0x3b00, 0x3b03, 0x3b06, 0x3b09, 0x3b0a, 0x3b0d, 0x3b10,
    0x3b13, 0x3b16, 0x3b18, 0x3b1a, 0x3b1c, 0x3b1e, 0x3b20, 0x3b22,
    0x3b25, 0x3b28, 0x3b2a, 0x3b2c, 0x3b2e, 0x3b30, 0x3b32, 0x3b34,
    0x3b37, 0x3b39, 0x3b3c, 0x3b3f, 0x3b43, 0x3b45, 0x3b47, 0x3b4a,
    0x3b4d, 0x3b4f, 0x3b51, 0x3b54, 0x3b56, 0x3b58, 0x3b5a, 0x3b5c,
    0x3b5e, 0x3b60, 0x3b62, 0x3b64, 0x3b66, 0x3b68, 0x3b6a, 0x3b6c,
    0x3b6e, 0x3b70, 0x3b72, 0x3b74, 0x3b76, 0x3b78, 0x3b7a, 0x3b7c,
    0x3b7e, 0x3b80, 0x3b82, 0x3b86, 0x3b88, 0x3b8b, 0x3b8d, 0x3b8f,
    0x3b91, 0x3b93, 0x3b95, 0x3b99, 0x3b9c, 0x3b9e, 0x3ba1, 0x3ba2,
    0x3ba5, 0x3ba8, 0x3bab, 0x3bae, 0x3bb1,
};

const uint16_t kGateSetupPcs[] = {
    0x3615, 0x3619, 0x361d, 0x3620, 0x3622, 0x3626, 0x3629, 0x362c,
    0x362e, 0x3630, 0x3634, 0x3636, 0x3639, 0x363b, 0x363d, 0x3641,
    0x3643, 0x3645, 0x364a, 0x364c, 0x3650, 0x3652, 0x3656, 0x3658,
    0x365c, 0x365e, 0x3661, 0x3663, 0x3666, 0x3668, 0x366c, 0x366e,
    0x3671, 0x3673, 0x3677, 0x367b, 0x367e, 0x3681, 0x3684, 0x3687,
    0x368a, 0x368d, 0x3690, 0x3693, 0x3695, 0x3698, 0x369a, 0x369d,
    0x36a0, 0x36a3, 0x36a5, 0x36a7, 0x36a9, 0x36ab, 0x36ad, 0x36af,
    0x36b1, 0x36b3, 0x36b5, 0x36b7, 0x36b9, 0x36bb, 0x36bf, 0x36c1,
    0x36c3, 0x36c6, 0x36c8, 0x36cb, 0x36cd, 0x36d1, 0x36d4, 0x36d7,
    0x36da, 0x36de, 0x36e2, 0x36e8, 0x36ec, 0x36f0, 0x36f2, 0x36f4,
    0x36f7, 0x36fa, 0x36fc, 0x3700, 0x3704, 0x3708,
};

/* S7 mask_set + shared tail: IML=0, per-PC entries. */
const uint16_t kMaskSetPcs[] = {
    0x546e, 0x5471, 0x5474, 0x5476, 0x547a, 0x547e, 0x5480,
    0x5484, 0x5487, 0x548c, 0x548e, 0x5491,
};

const uint16_t kMaskTailPcs[] = {
    0x5492, 0x5495, 0x5497, 0x549b, 0x54a0, 0x54a2, 0x54a5, 0x54a8,
    0x54aa, 0x54ae, 0x54b2, 0x54b5, 0x54ba, 0x54bc, 0x54bf, 0x54c2,
    0x54c5, 0x54c9,
};

/* S8 loop_write: per-PC because its status busy-wait needs host device
 * updates between iterations. */
const uint16_t kLoopWritePcs[] = {
    0x5626, 0x5629, 0x562c, 0x562e, 0x5630, 0x5632, 0x5634, 0x5636,
    0x5638, 0x563d, 0x5640, 0x5642, 0x5644, 0x5647, 0x5649,
};

/* S8 flush_off / param_write / flush_on: per-PC from round 6 (a span can
 * cover a due -midiseq post; see the S1 comment). */
const uint16_t kFlushOffPcs[] = {
    0x54fb, 0x54fe, 0x5503, 0x5505, 0x5509, 0x550d, 0x550f,
    0x5511, 0x5515, 0x5519, 0x551b, 0x551d, 0x5521,
};

const uint16_t kParamWritePcs[] = {
    0x5533, 0x5536, 0x553a, 0x553e, 0x5540, 0x5543, 0x5545, 0x5548,
    0x554c, 0x5550, 0x5553, 0x5556, 0x5559, 0x555b, 0x555d, 0x5560,
    0x5564, 0x5568, 0x556b, 0x556e, 0x5571, 0x5573, 0x5575, 0x5578,
    0x557c, 0x5580, 0x5583, 0x5586, 0x5589, 0x558b, 0x558d, 0x5590,
    0x5594, 0x5598, 0x559b, 0x559e, 0x55a0, 0x55a3, 0x55a7, 0x55aa,
    0x55ad, 0x55af, 0x55b2, 0x55b6, 0x55b9, 0x55bc, 0x55be, 0x55c1,
    0x55c3, 0x55c6, 0x55c8, 0x55cb, 0x55cd, 0x55d0, 0x55d2, 0x55d5,
    0x55d7, 0x55da, 0x55dd, 0x55e0, 0x55e2, 0x55e5, 0x55e8, 0x55ea,
    0x55ed, 0x55f0, 0x55f3, 0x55f6, 0x55f8, 0x55fb, 0x55fd, 0x5600,
    0x5604, 0x5607, 0x5608, 0x560b, 0x560e, 0x5610, 0x5612, 0x5615,
    0x5619, 0x561c, 0x561f, 0x5622, 0x5625,
};

const uint16_t kFlushOnAPcs[] = {
    0x564a, 0x564e, 0x5652, 0x5656, 0x565a, 0x565e,
};

const uint16_t kFlushOnBPcs[] = {
    0x5668, 0x566c, 0x5670,
};

/* S9 param_ack (IML=0, trapa handshake loop). */
const uint16_t kParamAckPcs[] = {
    0x54cc, 0x54d0, 0x54d3, 0x54d5, 0x54d7, 0x54d9, 0x54db, 0x54dd,
    0x54e1, 0x54e6, 0x54e8, 0x54ea, 0x54ec, 0x54ee, 0x54f0, 0x54f2,
    0x54f4, 0x54f6, 0x54f8, 0x54fa,
};

/* S10 pcm_play: L0 per-PC (orchestrates the registered child routines). */
const uint16_t kPcmPlayPcs[] = {
    0x52cb, 0x52d0, 0x52d2, 0x52d7, 0x52da, 0x52dc, 0x52e0, 0x52e5,
    0x52e7, 0x52e9, 0x52ed, 0x52f2, 0x52f4, 0x52f7, 0x52fb, 0x52fd,
    0x5300, 0x5304, 0x5308, 0x530c, 0x530f, 0x5313, 0x5317, 0x531a,
    0x531e, 0x5321, 0x5324, 0x5326, 0x5329, 0x532d, 0x5330, 0x5334,
    0x5337, 0x533b, 0x533f, 0x5342, 0x5346, 0x5349, 0x534d, 0x5350,
    0x5354, 0x5357, 0x535b, 0x535e, 0x5362, 0x5366, 0x5369, 0x536c,
    0x5370, 0x5373, 0x5376, 0x537a, 0x537d, 0x537f, 0x5382, 0x5386,
    0x5389, 0x538d,
    0x5390, 0x5392, 0x5396, 0x539c, 0x53a0, 0x53a4, 0x53a7, 0x53ab,
    0x53ae, 0x53b1, 0x53b3, 0x53b6, 0x53ba, 0x53bd, 0x53c0, 0x53c4,
    0x53c7, 0x53cb, 0x53cf, 0x53d2, 0x53d5, 0x53d8, 0x53dc, 0x53df,
    0x53e1, 0x53e4, 0x53e8,
};

/* S6 coeff_calc: per-PC entries (IML=0, dense resume PCs; fixed-point math is
 * transcribed as named operations, see routine_coeff_calc_step). */
const uint16_t kCoeffCalcPcs[] = {
    0x5998, 0x599a, 0x599e, 0x59a0, 0x59a2, 0x59a6, 0x59aa, 0x59ac, 0x59ae, 0x59b0,
    0x59b4, 0x59b6, 0x59ba, 0x59be, 0x59c1, 0x59c4, 0x59c6, 0x59c8, 0x59ca, 0x59cc,
    0x59ce, 0x59cf, 0x59d1, 0x59d3, 0x59d5, 0x59d9, 0x59dd, 0x59e1, 0x59e5, 0x59e9,
    0x59eb, 0x59ed, 0x59f0, 0x59f2, 0x59f5, 0x59f7, 0x59f9, 0x59fb, 0x59ff, 0x5a01,
    0x5a03, 0x5a06, 0x5a08, 0x5a0b, 0x5a0d, 0x5a0f, 0x5a11, 0x5a15, 0x5a19, 0x5a1d,
    0x5a20, 0x5a23, 0x5a25, 0x5a27, 0x5a2b, 0x5a2d, 0x5a2f, 0x5a31, 0x5a35, 0x5a37,
    0x5a3b, 0x5a3f, 0x5a43, 0x5a47, 0x5a4b, 0x5a4d, 0x5a4f, 0x5a52, 0x5a54, 0x5a57,
    0x5a59, 0x5a5b, 0x5a5d, 0x5a5f, 0x5a63, 0x5a65, 0x5a67, 0x5a6a, 0x5a6c, 0x5a6f,
    0x5a71, 0x5a73, 0x5a75, 0x5a77, 0x5a7b, 0x5a7f, 0x5a83, 0x5a86, 0x5a89, 0x5a8b,
    0x5a8d, 0x5a91, 0x5a93, 0x5a95, 0x5a97, 0x5a9b, 0x5a9d, 0x5aa1, 0x5aa5, 0x5aa9,
    0x5aad, 0x5ab1, 0x5ab3, 0x5ab5, 0x5ab8, 0x5aba, 0x5abd, 0x5abf, 0x5ac1, 0x5ac3,
    0x5ac7, 0x5ac9, 0x5acb, 0x5ace, 0x5ad0, 0x5ad3, 0x5ad5, 0x5ad7, 0x5ad9, 0x5add,
    0x5ae1, 0x5ae5, 0x5ae8, 0x5aeb, 0x5aed, 0x5aef, 0x5af3, 0x5af5, 0x5af7, 0x5af9,
    0x5afd, 0x5aff, 0x5b03, 0x5b07, 0x5b0b, 0x5b0f, 0x5b13, 0x5b15, 0x5b17, 0x5b1a,
    0x5b1c, 0x5b1f, 0x5b21, 0x5b25, 0x5b27, 0x5b29, 0x5b2c, 0x5b2e, 0x5b31, 0x5b33,
    0x5b37, 0x5b3a, 0x5b3e, 0x5b41, 0x5b44, 0x5b46, 0x5b48, 0x5b4c, 0x5b4e, 0x5b50,
    0x5b52, 0x5b56, 0x5b58, 0x5b5c, 0x5b60, 0x5b64, 0x5b68, 0x5b6c, 0x5b6e, 0x5b70,
    0x5b73, 0x5b75, 0x5b78, 0x5b7a, 0x5b7e, 0x5b80, 0x5b82, 0x5b85, 0x5b87, 0x5b8a,
    0x5b8c, 0x5b90, 0x5b93, 0x5b97, 0x5b9a, 0x5b9e, 0x5ba0, 0x5ba2, 0x5ba6, 0x5baa,
    0x5bae, 0x5bb2, 0x5bb6, 0x5bb8, 0x5bba, 0x5bbd, 0x5bbf, 0x5bc2, 0x5bc4, 0x5bc6,
    0x5bc8, 0x5bca, 0x5bce, 0x5bd0, 0x5bd2, 0x5bd5, 0x5bd7, 0x5bda, 0x5bdc, 0x5bde,
    0x5be0, 0x5be2, 0x5be6, 0x5bea, 0x5bee, 0x5bf1, 0x5bf5, 0x5bf7, 0x5bf9, 0x5bfd,
    0x5c01, 0x5c05, 0x5c09, 0x5c0d, 0x5c0f, 0x5c11, 0x5c14, 0x5c16, 0x5c19, 0x5c1b,
    0x5c1d, 0x5c1f, 0x5c21, 0x5c25, 0x5c27, 0x5c29, 0x5c2c, 0x5c2e, 0x5c31, 0x5c33,
    0x5c35, 0x5c37, 0x5c39, 0x5c3d, 0x5c41, 0x5c45, 0x5c48, 0x5c4c, 0x5c4e, 0x5c50,
    0x5c54, 0x5c58, 0x5c5c, 0x5c60, 0x5c64, 0x5c66, 0x5c68, 0x5c6b, 0x5c6d, 0x5c70,
    0x5c72, 0x5c76, 0x5c78, 0x5c7a, 0x5c7d, 0x5c7f, 0x5c82, 0x5c84, 0x5c88, 0x5c8c,
    0x5c90, 0x5c93, 0x5c97, 0x5c99, 0x5c9b, 0x5c9f, 0x5ca3, 0x5ca7, 0x5cab, 0x5caf,
    0x5cb1, 0x5cb3, 0x5cb6, 0x5cb8, 0x5cbb, 0x5cbd, 0x5cc1, 0x5cc3, 0x5cc5, 0x5cc8,
    0x5cca, 0x5ccd, 0x5ccf, 0x5cd3, 0x5cd7, 0x5cdb, 0x5cde, 0x5ce2, 0x5ce4, 0x5ce6,
    0x5cea, 0x5cee, 0x5cf2, 0x5cf6, 0x5cfa, 0x5cfc, 0x5cfe, 0x5d01, 0x5d03, 0x5d06,
    0x5d08, 0x5d0c, 0x5d0e, 0x5d10, 0x5d13, 0x5d15, 0x5d18, 0x5d1a, 0x5d1e, 0x5d22,
    0x5d26, 0x5d29, 0x5d2d, 0x5d2f, 0x5d31, 0x5d35, 0x5d39, 0x5d3d, 0x5d41, 0x5d45,
    0x5d47, 0x5d49, 0x5d4c, 0x5d4e, 0x5d51, 0x5d53, 0x5d57, 0x5d59, 0x5d5b, 0x5d5e,
    0x5d60, 0x5d63, 0x5d65, 0x5d69, 0x5d6d,
};
} /* namespace mk2c */

void MK2CPP_VoiceMaterializeFillTables(void)
{
    /* S1/S2 materialize + coeff_copy: per-PC (round 6; an L1 block spanning a
     * -midiseq post moved the SM's byte delivery and diverged at c212.565M). */
    for (uint32_t i = 0; i < sizeof(mk2c::kMaterializePcs) / sizeof(mk2c::kMaterializePcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kMaterializePcs[i], &mk2c::routine_materialize_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kCoeffCopyPcs) / sizeof(mk2c::kCoeffCopyPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kCoeffCopyPcs[i], &mk2c::routine_coeff_copy_step);

    /* S3/S4/S5: one entry per instruction (each returns 1, i.e. exactly one
     * H8 instruction; the host's poll/TIMER_Clock/trace per step then match
     * stock for these IML=0, densely interruptible routines). */
    for (uint32_t i = 0; i < sizeof(mk2c::kToneFieldsPcs) / sizeof(mk2c::kToneFieldsPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kToneFieldsPcs[i], &mk2c::routine_tone_fields_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kCrossCopyPcs) / sizeof(mk2c::kCrossCopyPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kCrossCopyPcs[i], &mk2c::routine_cross_copy_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kGateSetupPcs) / sizeof(mk2c::kGateSetupPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kGateSetupPcs[i], &mk2c::routine_gate_setup_step);

    /* S7 mask_set + shared tail: IML=0, per-PC entries (interrupted before
     * 0x5474 and before the 0x5491 rts in the captured runs). */
    for (uint32_t i = 0; i < sizeof(mk2c::kMaskSetPcs) / sizeof(mk2c::kMaskSetPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kMaskSetPcs[i], &mk2c::routine_mask_set_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kMaskTailPcs) / sizeof(mk2c::kMaskTailPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kMaskTailPcs[i], &mk2c::routine_mask_tail_step);

    /* S7/S8 flush/mask/param blocks: per-PC from round 6 (IML=7 does not
     * protect the host tail cadence; loop_write already per-PC). */
    for (uint32_t i = 0; i < sizeof(mk2c::kFlushOffPcs) / sizeof(mk2c::kFlushOffPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kFlushOffPcs[i], &mk2c::routine_flush_off_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kParamWritePcs) / sizeof(mk2c::kParamWritePcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kParamWritePcs[i], &mk2c::routine_param_write_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kLoopWritePcs) / sizeof(mk2c::kLoopWritePcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kLoopWritePcs[i], &mk2c::routine_loop_write_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kFlushOnAPcs) / sizeof(mk2c::kFlushOnAPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kFlushOnAPcs[i], &mk2c::routine_flush_on_a_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kFlushOnBPcs) / sizeof(mk2c::kFlushOnBPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kFlushOnBPcs[i], &mk2c::routine_flush_on_b_step);

    /* S9 param_ack and S10 pcm_play: per-PC entries; bsr targets are the
     * registered child routines, so the host keeps dispatching. */
    for (uint32_t i = 0; i < sizeof(mk2c::kParamAckPcs) / sizeof(mk2c::kParamAckPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kParamAckPcs[i], &mk2c::routine_param_ack_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kPcmPlayPcs) / sizeof(mk2c::kPcmPlayPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kPcmPlayPcs[i], &mk2c::routine_pcm_play_step);

    /* S6 coeff_calc: per-PC (IML=0, dense resume PCs; no deferral). */
    for (uint32_t i = 0; i < sizeof(mk2c::kCoeffCalcPcs) / sizeof(mk2c::kCoeffCalcPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kCoeffCalcPcs[i], &mk2c::routine_coeff_calc_step);
}
