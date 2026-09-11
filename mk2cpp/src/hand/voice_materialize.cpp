/*
 * HAND voice/voice_materialize -- PCM voice materialize / gate / param chain
 * (M4 S1..S5, S7..S10): materialize, coeff_copy, tone_fields, cross_copy,
 * gate_setup, mask_set + shared tail, flush_off/flush_on, param_write,
 * loop_write, param_ack, pcm_play.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 5
 *
 * Replaces these rom1 routines:
 *
 *   0x53eb materialize   0x53eb..0x546c (+ 0x546d rts trampoline): one L1
 *                        block (IML=7, stock cannot be interrupted inside)
 *   0x5d6e coeff_copy    0x5d6e..0x5dc2: one L1 block (leaf, 23 instructions)
 *   0x3580 tone_fields   0x3580..0x3614: one entry per instruction (L0-style)
 *   0x3a9e cross_copy    0x3a9e..0x3bb1: one entry per instruction (0x3a9e
 *                        and 0x3ac8 entries both covered)
 *   0x3615 gate_setup    0x3615..0x3708: one entry per instruction; 0x36fa
 *                        (bsr 0x3709) is translated as the push+jump, the
 *                        callee keeps running interpreted (interp_entry,
 *                        out-of-scope deep water)
 *   0x546e mask_set      + shared pending-command tail 0x5492..0x54cb: per-PC
 *                        (IML=0, interrupted inside in the captured runs)
 *   0x54fb flush_off     0x54fb..0x5521: one L1 block (IML=7), then the six
 *                        existing per-PC PCM MOVS/MOVL handlers
 *   0x564a flush_on      0x564a..0x565e / 0x5668..0x5670: two L1 blocks
 *   0x5533 param_write   0x5533..0x5625: one L1 block, both exits
 *   0x5626 loop_write    per-PC: its PCM status busy-wait needs one host step
 *                        per iteration (device updates), see comment below
 *   0x54cc param_ack    0x54cc..0x54fa: per-PC (IML=0, trapa #0x10 loop)
 *   0x52cb pcm_play     0x52cb..0x53e8 (both entries): per-PC; bsr targets
 *                        are the registered child routines, BRA 0x51fc exits
 *                        do not pop the stack
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
 *  - materialize is called from pcm_play before 0x5304/0x539c BCLR_ANDC, i.e.
 *    with IML=7 (dispatcher 0x5207 ORC). MCU_Interrupt_Handle only takes a
 *    maskable source when `mask < level` (src/mcu_interrupt.cpp:202), and
 *    neither trapa nor exception can originate inside the routine, so the
 *    stock run cannot be interrupted inside 0x53eb..0x546d. `-tracepc` over
 *    mock 144M / wide / demo296 / demo300 / demo312 confirms it (not one
 *    handler PC between 0x53eb and the 0x546c rts), so one L1 block matches
 *    both state and cycle timing. No split point is needed.
 *  - coeff_copy runs at IML=0 and is a plain leaf; no device writes, only
 *    word copies. It is registered below as one L1 block; the demo windows
 *    observed so far never enter it (the single-neighbour pcm_play branch
 *    0x5390 skips 0x5317), so the hash gate exercises it only when the main
 *    pcm_play path runs.
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
 * Materialize path counts (returns the exact stock instruction count; the
 * host adds 12*(n-1) on top of its own +12):
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

/* ---- S1: materialize 0x53eb-0x546c -------------------------------------- */

uint32_t routine_materialize(void)
{
    uint32_t n = 0;
    uint16_t &r0 = mcu.r[0];
    uint16_t &r1 = mcu.r[1];
    uint16_t &r3 = mcu.r[3];
    uint16_t &r6 = mcu.r[6];

    /* 53eb MOVG2 r1 r3 / 53ed ADD r3 r3 / 53ef MOVG2 @r3+0x64d6 r0:
     * r3 = 2*slot and r0 = AoS P(slot) from the ROM pointer table. */
    r3 = r1;
    MCU_SetStatusCommon(r1, 1);
    n++;
    r3 = (uint16_t)MCU_ADD_Common(r3, r3, 0, 1);
    n++;
    load16(r0, ind_addr(3, kAosPtrTable));
    n++;

    /* 53f3..5422: record header + pointer words. */
    store16(ind_addr(0, (uint16_t)kAosSlot), r1);      /* 53f3 P-2 = slot */
    n++;
    load8(r6, ind_addr(1, kSoaCfe4));                  /* 53f6 */
    n++;
    store8(ind_addr(0, kAosE1), (uint8_t)r6);          /* 53fa P+0x98 */
    n++;
    load8(r6, ind_addr(1, kSoaD038));                  /* 53fe */
    n++;
    store8(ind_addr(0, kAosE2), (uint8_t)r6);          /* 5402 P+0x99 */
    n++;
    load8(r6, ind_addr(1, kSoaD08c));                  /* 5406 */
    n++;
    store8(ind_addr(0, kAosE3), (uint8_t)r6);          /* 540a P+0x9a */
    n++;
    load16(r6, ind_addr(3, kSoaCfac));                 /* 540e */
    n++;
    store16(ind_addr(0, kAosP1), r6);                  /* 5412 P+0x9c */
    n++;
    load16(r6, ind_addr(3, kSoaD000));                 /* 5416 */
    n++;
    store16(ind_addr(0, kAosP2), r6);                  /* 541a P+0x9e */
    n++;
    load16(r6, ind_addr(3, kSoaD054));                 /* 541e */
    n++;
    store16(ind_addr(0, kAosP3), r6);                  /* 5422 P+0xa0 */
    n++;

    /* 5426..542f: cf90 byte -> P-0x3b; bit7 forces ad0e[slot] = 0xff. */
    load8(r6, ind_addr(1, kSoaCf90));                  /* 5426 */
    n++;
    store8(ind_addr(0, (uint16_t)kAosFlag45), (uint8_t)r6); /* 542a */
    n++;
    MCU_SetStatus((r6 & 0x80u) == 0, STATUS_Z);        /* 542d BTSTI r6 #7 */
    n++;
    n++;                                               /* 542f BEQ 5 */
    if ((mcu.sr & STATUS_Z) == 0)
    {
        store8(ind_addr(1, kAd0e), 0xff);              /* 5431 MOVG #0xff */
        n++;
    }

    /* 5436..5446: tone index -> P+0x9b, tone pointer -> P+0x2e. */
    clr16(r3);                                         /* 5436 */
    n++;
    load8(r3, ind_addr(1, kSoaCe78));                  /* 5438 */
    n++;
    store8(ind_addr(0, kAosToneIdx), (uint8_t)r3);     /* 543c P+0x9b */
    n++;
    r3 = (uint16_t)MCU_ADD_Common(r3, r3, 0, 1);       /* 5440 ADD r3 r3 */
    n++;
    load16(r3, ind_addr(3, kTonePtrTable));            /* 5442 @r3+0x7218 */
    n++;
    store16(ind_addr(0, kAosToneRec), r3);             /* 5446 P+0x2e */
    n++;

    /* 5449..544f: r3 = 0, then TST d0fc[slot] and BEQ/BMI. */
    clr16(r3);                                         /* 5449 */
    n++;
    {
        uint8_t sel = MCU_Read(ind_addr(1, kSoaD0fc)); /* 544b TST */
        MCU_SetStatusCommon(sel, 0);
        MCU_SetStatus(0, STATUS_C);
        n++;
        n++;                                           /* 544f BEQ */
        if (sel == 0)
        {
            /* 545a movi r3 #0x8748; 545d bsr -> 546d; 545f..5469; 546c rts */
            movi16(r3, kAosBaseZero);                  /* 545a */
            n++;
            MCU_PushStack(0x545fu);                    /* 545d bsr */
            n++;
            mcu.pc = MCU_PopStack();                   /* 546d rts */
            n++;
            goto common_tail;
        }
        if (sel & 0x80u)
        {
            /* 5451 BMI -> 5469: r3 stays 0, P+0x30 = 0. */
            n++;                                       /* 5451 BMI */
            store16(ind_addr(0, kAosToneVal), r3);     /* 5469 */
            n++;
            mcu.pc = MCU_PopStack();                   /* 546c rts */
            n++;
            return n;
        }
        /* E3 arm 0x5453-0x5458 (bytes 0x5453: 5b 8b d4 0e 15 20 05):
         * movi r3 #0x8bd4; 5456 bsr -> 546d; 5458 BRA +5 -> 545f. */
        movi16(r3, kAosBasePos);                       /* 5453 */
        n++;
        MCU_PushStack(0x5458u);                        /* 5456 bsr */
        n++;
        mcu.pc = MCU_PopStack();                       /* 546d rts */
        n++;
        n++;                                           /* 5458 BRA 0x545f */
    }
common_tail:
    clr16(r6);                                         /* 545f CLR r6 */
    n++;
    load8(r6, ind_addr(1, kSoaCecc));                  /* 5461 */
    n++;
    r3 = (uint16_t)MCU_ADD_Common(r3, r6, 0, 1);       /* 5465 ADD r6 r3 */
    n++;
    n++;                                               /* 5467 BRA 0x5469 */
    store16(ind_addr(0, kAosToneVal), r3);             /* 5469 P+0x30 */
    n++;
    mcu.pc = MCU_PopStack();                           /* 546c rts */
    n++;

    return n;
}

/* ---- S2: coeff_copy 0x5d6e-0x5dc2 ---------------------------------------- */

uint32_t routine_coeff_copy(void)
{
    /* In r0 = dst P, r2 = src P; 11 word copies in ROM order. */
    static const int16_t kOff[11] = {
        kCoef86, kCoef8a, kCoef88, kCoefP80, kCoefP114,
        kCoef96, kCoef8e, kCoef94, kCoef8c, kCoef92, kCoef90,
    };
    uint16_t &r6 = mcu.r[6];

    uint32_t n = 0;
    for (int i = 0; i < 11; i++)
    {
        uint16_t off = (uint16_t)kOff[i];
        load16(r6, ind_addr(2, off));                  /* 5d6e/76/7e/86/8c/92/9a/a2/aa/b2/ba */
        n++;
        store16(ind_addr(0, off), r6);                 /* 5d72/7a/82/89/8f/96/9e/a6/ae/b6/be */
        n++;
    }
    mcu.pc = MCU_PopStack();                           /* 5dc2 rts */
    n++;
    return n;
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
    case 0x549b: movg_imm16(ind_addr(0, 0), 0x0018);           mcu.pc = 0x54a0; break;
    case 0x54a0: mcu.pc = (mcu.sr & STATUS_Z) ? 0x54a2 : 0x54aa; break;
    case 0x54a2: load16(r6, ind_addr(0, 6));                   mcu.pc = 0x54a5; break;
    case 0x54a5: store16(ind_addr(0, 0), r6);                  mcu.pc = 0x54a8; break;
    case 0x54a8: mcu.pc = 0x54ae; break;
    case 0x54aa: clr8_mem(ind_addr(1, 0xad0e));                mcu.pc = 0x54ae; break;
    case 0x54ae: load16(r0, dp_addr(0xd15a));                  mcu.pc = 0x54b2; break;
    case 0x54b2: mcu.pc = (mcu.sr & STATUS_N) ? 0x51fc : 0x54b5; break;
    case 0x54b5: movg_imm16(ind_addr(0, 0), 0x0018);           mcu.pc = 0x54ba; break;
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

/* flush_off 0x54fb-0x5521 (IML=7): clear the pending mask bits from the
 * committed mask, then hand off to the six existing per-PC PCM flush handlers
 * (0x5525/27/29 in pcm_enable.cpp) and the 0x552b rts. The pending-command
 * exit (0x552c, unobserved in the captured runs) is left interpreted. */
uint32_t routine_flush_off(void)
{
    uint32_t n = 0;
    uint16_t &r1 = mcu.r[1];
    uint16_t &r3 = mcu.r[3];
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    load16(r1, ind_addr(0, (uint16_t)-2));                     /* 54fb */
    n++;
    sub_mem_imm8(ind_addr(1, 0xd0e0), 0x00);                   /* 54fe */
    n++;
    n++;                                                       /* 5503 BNE */
    if ((mcu.sr & STATUS_Z) == 0)
    {
        mcu.pc = 0x552cu;
        return n;
    }
    load16(r5, dp_addr(0xd154));                               /* 5505 */
    n++;
    load16(r6, dp_addr(0xd156));                               /* 5509 */
    n++;
    not16(r5);                                                 /* 550d */
    n++;
    not16(r6);                                                 /* 550f */
    n++;
    load16(r3, dp_addr(0xd150));                               /* 5511 */
    n++;
    load16(r4, dp_addr(0xd152));                               /* 5515 */
    n++;
    and16(r3, r5);                                             /* 5519 */
    n++;
    and16(r4, r6);                                             /* 551b */
    n++;
    store16(dp_addr(0xd150), r3);                              /* 551d */
    n++;
    store16(dp_addr(0xd152), r4);                              /* 5521 */
    n++;
    mcu.pc = 0x5525u;
    return n;
}

/* flush_on 0x564a-0x565e -> 0x5662 (IML=7). */
uint32_t routine_flush_on_a(void)
{
    uint32_t n = 0;
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    load16(r5, dp_addr(0xd150));                               /* 564a */
    n++;
    load16(r6, dp_addr(0xd152));                               /* 564e */
    n++;
    or16_mem(r5, dp_addr(0xd154));                             /* 5652 */
    n++;
    or16_mem(r6, dp_addr(0xd156));                             /* 5656 */
    n++;
    store16(dp_addr(0xd150), r5);                              /* 565a */
    n++;
    store16(dp_addr(0xd152), r6);                              /* 565e */
    n++;
    mcu.pc = 0x5662u;
    return n;
}

/* flush_on 0x5668-0x5670 (after the 0x5662/64/66 PCM handlers). */
uint32_t routine_flush_on_b(void)
{
    uint32_t n = 0;

    clr16_mem(dp_addr(0xd154));                                /* 5668 */
    n++;
    clr16_mem(dp_addr(0xd156));                                /* 566c */
    n++;
    mcu.pc = MCU_PopStack();                                   /* 5670 */
    n++;
    return n;
}

/* ---- S8: param_write 0x5533-0x5625 --------------------------------------- */

/* IML=7 (called between BSET_ORC 0x535e/0x53c7 and the exit BCLR_ANDC), no
 * observed internal interrupt, so one whole-routine block. All PCM register
 * traffic goes through MCU_Write/Read (device routing + -pcmtrace). dp is
 * cleared by each LDC as in the ROM; the block has no internal poll so the
 * host's boundary poll is the one stock would have taken. */
uint32_t routine_param_write(void)
{
    uint32_t n = 0;
    uint16_t &r3 = mcu.r[3];
    uint16_t &r4 = mcu.r[4];
    uint16_t &r5 = mcu.r[5];
    uint16_t &r6 = mcu.r[6];

    load16(r3, ind_addr(0, (uint16_t)-2));                     /* 5533 slot */
    n++;
    clr8_mem(ind_addr(3, 0xce3f));                             /* 5536 */
    n++;
    clr8_mem(ind_addr(3, 0xd15c));                             /* 553a */
    n++;
    movs8(r3, 0x3e);                                           /* 553e select */
    n++;
    load16(r6, ind_addr(0, (uint16_t)-10));                    /* 5540 P-0x0a */
    n++;
    movs16(r6, 0x1e);                                          /* 5543 */
    n++;
    ldc_dp(0);                                                 /* 5545 */
    n++;
    movg_imm8(dp_addr(0xfe6c), (uint8_t)r3);                   /* 5548 */
    n++;
    store16(dp_addr(0xfe5e), r3);                              /* 554c */
    n++;
    ldc_dp(0);                                                 /* 5550 */
    n++;
    load8(r5, ind_addr(0, (uint16_t)-20));                     /* 5553 P-0x14 */
    n++;
    load16(r6, ind_addr(0, (uint16_t)-16));                    /* 5556 P-0x10 */
    n++;
    movs8(r5, 0x05);                                           /* 5559 */
    n++;
    movs16(r6, 0x06);                                          /* 555b */
    n++;
    ldc_dp(0);                                                 /* 555d */
    n++;
    movg_imm8(dp_addr(0xfe6d), (uint8_t)r5);                   /* 5560 */
    n++;
    store16(dp_addr(0xfe60), r6);                              /* 5564 */
    n++;
    ldc_dp(0);                                                 /* 5568 */
    n++;
    load8(r5, ind_addr(0, (uint16_t)-18));                     /* 556b P-0x12 */
    n++;
    load16(r6, ind_addr(0, (uint16_t)-12));                    /* 556e P-0x0c */
    n++;
    movs8(r5, 0x09);                                           /* 5571 */
    n++;
    movs16(r6, 0x0a);                                          /* 5573 */
    n++;
    ldc_dp(0);                                                 /* 5575 */
    n++;
    movg_imm8(dp_addr(0xfe6e), (uint8_t)r5);                   /* 5578 */
    n++;
    store16(dp_addr(0xfe62), r6);                              /* 557c */
    n++;
    ldc_dp(0);                                                 /* 5580 */
    n++;
    load8(r5, ind_addr(0, (uint16_t)-19));                     /* 5583 P-0x13 */
    n++;
    load16(r6, ind_addr(0, (uint16_t)-14));                    /* 5586 P-0x0e */
    n++;
    movs8(r5, 0x0d);                                           /* 5589 */
    n++;
    movs16(r6, 0x0e);                                          /* 558b */
    n++;
    ldc_dp(0);                                                 /* 558d */
    n++;
    movg_imm8(dp_addr(0xfe6f), (uint8_t)r5);                   /* 5590 */
    n++;
    store16(dp_addr(0xfe64), r6);                              /* 5594 */
    n++;
    ldc_dp(0);                                                 /* 5598 */
    n++;
    load16(r6, ind_addr(0, 52));                               /* 559b P+0x34 */
    n++;
    movs16(r6, 0x12);                                          /* 559e */
    n++;
    ldc_dp(0);                                                 /* 55a0 */
    n++;
    store16(dp_addr(0xfe66), r6);                              /* 55a3 */
    n++;
    ldc_dp(0);                                                 /* 55a7 */
    n++;
    load16(r6, ind_addr(0, 58));                               /* 55aa P+0x3a */
    n++;
    movs16(r6, 0x14);                                          /* 55ad */
    n++;
    ldc_dp(0);                                                 /* 55af */
    n++;
    store16(dp_addr(0xfe68), r6);                              /* 55b2 */
    n++;
    ldc_dp(0);                                                 /* 55b6 */
    n++;
    load8(r6, ind_addr(0, 104));                               /* 55b9 P+0x68 */
    n++;
    swap16(r6);                                                /* 55bc */
    n++;
    load8(r6, ind_addr(0, 102));                               /* 55be P+0x66 */
    n++;
    movs16(r6, 0x1c);                                          /* 55c1 */
    n++;
    load16(r6, ind_addr(0, (uint16_t)-24));                    /* 55c3 P-0x18 */
    n++;
    movs16(r6, 0x1a);                                          /* 55c6 */
    n++;
    load16(r6, ind_addr(0, 26));                               /* 55c8 P+0x1a */
    n++;
    movs16(r6, 0x16);                                          /* 55cb */
    n++;
    btsti8_mem(ind_addr(0, (uint16_t)-59), 7);                 /* 55cd */
    n++;
    n++;                                                       /* 55d0 BEQ */
    if (mcu.sr & STATUS_Z)
    {
        /* 5608: alternate tail (cf90 bit7 clear), unobserved in captured runs. */
        load16(r5, ind_addr(0, 30));                           /* 5608 */
        n++;
        load16(r6, ind_addr(0, 72));                           /* 560b */
        n++;
        movs16(r5, 0x18);                                      /* 560e */
        n++;
        movs16(r6, 0x10);                                      /* 5610 */
        n++;
        ldc_dp(0);                                             /* 5612 */
        n++;
        store16(dp_addr(0xfe6a), r6);                          /* 5615 */
        n++;
        ldc_dp(0);                                             /* 5619 */
        n++;
        load16(r6, ind_addr(0, 6));                            /* 561c */
        n++;
        store16(ind_addr(0, 0), r6);                           /* 561f */
        n++;
        clr16_mem(ind_addr(0, 6));                             /* 5622 */
        n++;
        mcu.pc = MCU_PopStack();                               /* 5625 */
        n++;
        return n;
    }
    tst16_mem(ind_addr(0, 14));                                /* 55d2 */
    n++;
    n++;                                                       /* 55d5 BEQ */
    if (mcu.sr & STATUS_Z)
    {
        movi16(r4, 0x0000);                                    /* 55e2 */
        n++;
        movi16(r5, 0x00b5);                                    /* 55e5 */
        n++;
        clr16(r6);                                             /* 55e8 */
        n++;
        clr16_mem(ind_addr(0, 8));                             /* 55ea */
        n++;
    }
    else
    {
        movi16(r4, 0x0002);                                    /* 55d7 */
        n++;
        load16(r5, ind_addr(0, 30));                           /* 55da */
        n++;
        load16(r6, ind_addr(0, 72));                           /* 55dd */
        n++;
        n++;                                                   /* 55e0 BRA -> 55ed */
    }
    store16(ind_addr(0, 0), r4);                               /* 55ed */
    n++;
    store16(ind_addr(0, 2), r4);                               /* 55f0 */
    n++;
    store16(ind_addr(0, 4), r4);                               /* 55f3 */
    n++;
    movs16(r5, 0x18);                                          /* 55f6 */
    n++;
    store16(ind_addr(0, 30), r5);                              /* 55f8 */
    n++;
    movs16(r6, 0x10);                                          /* 55fb */
    n++;
    ldc_dp(0);                                                 /* 55fd */
    n++;
    store16(dp_addr(0xfe6a), r6);                              /* 5600 */
    n++;
    ldc_dp(0);                                                 /* 5604 */
    n++;
    mcu.pc = MCU_PopStack();                                   /* 5607 */
    n++;
    return n;
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

} /* anonymous namespace */

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

} /* namespace mk2c */

void MK2CPP_VoiceMaterializeFillTables(void)
{
    MK2CPP_HandRegisterRoutine(0x000053ebu, &mk2c::routine_materialize);
    MK2CPP_HandRegisterRoutine(0x00005d6eu, &mk2c::routine_coeff_copy);

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

    /* S7/S8 flush/mask/param blocks: IML=7, no internal interrupt observed.
     * loop_write stays per-PC (busy-wait needs host device updates). */
    MK2CPP_HandRegisterRoutine(0x000054fbu, &mk2c::routine_flush_off);
    MK2CPP_HandRegisterRoutine(0x00005533u, &mk2c::routine_param_write);
    for (uint32_t i = 0; i < sizeof(mk2c::kLoopWritePcs) / sizeof(mk2c::kLoopWritePcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kLoopWritePcs[i], &mk2c::routine_loop_write_step);
    MK2CPP_HandRegisterRoutine(0x0000564au, &mk2c::routine_flush_on_a);
    MK2CPP_HandRegisterRoutine(0x00005668u, &mk2c::routine_flush_on_b);

    /* S9 param_ack and S10 pcm_play: per-PC entries; bsr targets are the
     * registered child routines, so the host keeps dispatching. */
    for (uint32_t i = 0; i < sizeof(mk2c::kParamAckPcs) / sizeof(mk2c::kParamAckPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kParamAckPcs[i], &mk2c::routine_param_ack_step);
    for (uint32_t i = 0; i < sizeof(mk2c::kPcmPlayPcs) / sizeof(mk2c::kPcmPlayPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(mk2c::kPcmPlayPcs[i], &mk2c::routine_pcm_play_step);
}
