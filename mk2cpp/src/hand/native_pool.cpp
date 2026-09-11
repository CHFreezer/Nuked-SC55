/*
 * HAND voice/native_pool -- whole-routine voice pool initialization (M4).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 3
 *
 * Replaces the rom2 pool-init routine 0x40462-0x4062a (pjsr from 0x565,
 * ret from 0x40586) with four whole-routine L1 blocks. All state lives in
 * page-0 SRAM and is accessed with MCU_Read/MCU_Write, so untranslated ROM and
 * the state hashdump observe exactly the stock bytes and registers.
 *
 *   block           covers                  exit pc   why split there
 *   A 0x40462  0x40462..0x404b0             0x404b4  1st observed interrupt,
 *                                                      resume PC 0x404b4
 *   B 0x404b4  0x404b4..0x4061e             0x40621  2nd observed interrupt,
 *                                                      resume PC 0x40624
 *   C 0x40621  0x40621                      0x40624  poll point inside fill
 *   D 0x40624  0x40624 (+cntjmp r3 body)    per path  loop, same boundary
 *
 * Boot baseline (tools/baselines/trace_boot3m_base.txt) shows the FRT2 OCIA
 * handler 0x344 taken twice inside the routine: after 0x404b0 (c=322068) and
 * after 0x40621 (c=346128); the handler resumes at 0x404b4 and 0x40624. The
 * blocks end exactly there so the host polls at the same cycle and the
 * handler saves the same resume PC. 0x40624 is the helper's 16x16 r2 loop
 * counter, so block D returns to 0x40621/0x40624 after every inner iteration
 * and keeps a poll point at both instructions.
 *
 * Per-block instruction counts (host adds 12*(n-1) on top of its own +12):
 *   A = 3 (seed) + 19 (loop A first half)                    = 22
 *   B = 5 (loop A tail) + 27*24 (loop A) + 7 (seeds)
 *       + 140 (loop B) + 6 (free seed) + 226 (loop C) + 1
 *       + 252 (loop D) + 1 + 128 (loop E) + 1 (bsr)
 *       + 1 + 16*(28|29) (part helper 1) + 2
 *       + 27 (part helper 2 body, first part)                = 1909 boot
 *   C = 1 (0x40621 store)
 *   D = 1 (cntjmp r1), 29 (cntjmp r3 + 27-instruction body) or
 *       3/4 (rts, plus ret when the pool-init pjsr return PC 0x586 is on top)
 *
 * 0x40588 is a shared subroutine (jsr #0x0588 at 0x2f86/0x3115/0x5778a
 * besides pool-init's bsr16 at 0x40583), so block D only falls through to the
 * 0x40586 ret when rts returns to the pool-init return address; other callers
 * continue at their own PC.
 *
 * The stock boot executes 2866 instructions (2868 traced PCs minus the two
 * interrupt-poll lines); the block counts sum to the same on the baseline.
 * Instruction semantics come from GT (src/mcu_opcodes.cpp); flag effects are
 * mirrored with GT's own helpers where a block boundary can observe them.
 *
 * The ROM's LDC writes ep (control register 4) and sets ex_ignore; ep is kept
 * exact (the BTSTI below reads page ep), ex_ignore is left alone because the
 * virtual poll after each emulated LDC consumes it inside the block. There is
 * no IMASK window in this routine.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"

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

/* MOVG3 / MOVG #imm -> mem: N/Z/V from the value, C preserved. */
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

/* ADDQ on a register's low byte: high byte preserved. */
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

/* Page-0 bases (names from out/11 2.2; all literal, dp = 0):
 *   a3a0 free flag  ad0e pcm_ch  a3bc state  a4b4 active  d0e0 cmd
 *   a3f4/a410 voice prev/next, d0a8/d0c4 start chain
 *   a3d8 free_next, a42f free head, a430 free tail, a42d free count,
 *   a42e desc head, a250/a26c desc next/prev, a288 state, a2c0/a2dc chain,
 *   a314 age, a368/a384 part/desc, a34c scratch
 *   loop A scratch: ce5c/ce78/ce94/d118/d134/ceb0/cf20/cf3c/cf90/d0fc,
 *                   cfac/d000/d054/a46c (word)
 *   part table: a200/a210/a220/a230/a240; seeds a1f0/a1df/a432/a434/a436
 *   part helper: abde (word input), a1b0/a1f4/a43c/a44c/a040/a080/a190,
 *                a050/a060/a070, a090.. (r2 fill), a200.. */

/* ---- loop A 0x40469-0x404c2: 28 slots, r1 = 27..0 ------------------------ */

/* 0x40469-0x404b0, 19 instructions: byte constants for slot r1=i (r3=0xff,
 * r2=0, cf3c=0x3c), then SHLL and the cfac word at 0xcfac+2i. r1 becomes 2i. */
uint32_t loop_a_first(uint16_t &r1, uint16_t r2, uint16_t r3)
{
    uint16_t i = r1;
    store8(0xce5c + i, (uint8_t)r3);          /* 40469 */
    store8(0xce78 + i, (uint8_t)r3);          /* 4046d */
    store8(0xce94 + i, (uint8_t)r3);          /* 40471 */
    store8(0xd118 + i, (uint8_t)r3);          /* 40475 */
    store8(0xd134 + i, (uint8_t)r3);          /* 40479 */
    store8(0xceb0 + i, (uint8_t)r3);          /* 4047d */
    store8(0xcf20 + i, (uint8_t)r3);          /* 40481 */
    store8(0xcf3c + i, 0x3c);                 /* 40485 */
    store8(0xcf90 + i, (uint8_t)r3);          /* 4048a */
    store8(0xd0fc + i, (uint8_t)r3);          /* 4048e */
    store8(0xd0a8 + i, (uint8_t)r3);          /* 40492 */
    store8(0xd0c4 + i, (uint8_t)r3);          /* 40496 */
    store8(0xad0e + i, (uint8_t)r2);          /* 4049a */
    store8(0xce3f + i, (uint8_t)r2);          /* 4049e */
    store8(0xd0e0 + i, (uint8_t)r2);          /* 404a2 */
    store8(0xa4b4 + i, (uint8_t)r2);          /* 404a6 */
    store8(0xa34c + i, (uint8_t)r2);          /* 404aa */
    shll16(r1);                               /* 404ae */
    store16(0xcfac + r1, r2);                 /* 404b0 */
    return 19;
}

/* 0x404b4-0x404c2, 5 instructions: d000/d054/a46c words at +2i, SHLR back to
 * i, then cntjmp r1 -92 (r1 = i-1; `more` = the loop continues). */
uint32_t loop_a_tail(uint16_t &r1, uint16_t r2, bool &more)
{
    store16(0xd000 + r1, r2);                 /* 404b4 */
    store16(0xd054 + r1, r2);                 /* 404b8 */
    store16(0xa46c + r1, r2);                 /* 404bc */
    shlr16(r1);                               /* 404c0 */
    r1 = (uint16_t)(r1 - 1);                  /* 404c2 cntjmp */
    more = (r1 != 0xffff);
    return 5;
}

/* ---- part helper, second loop body 0x405fa-0x4061e (27) ------------------ */

/* One per-part body of helper loop 2. r3 = part (15..0), r2 = 0xa090 fill
 * pointer (advances 16 bytes per invocation). Leaves r1 = 0x000f so the
 * following 0x40621 fill byte is the next stock instruction. */
uint32_t helper_loop2_body(uint16_t part)
{
    uint32_t n = 0;
    uint16_t &r0 = mcu.r[0];
    uint16_t &r1 = mcu.r[1];

    clear8(0xa060 + part);                    /* 405fa */
    n++;
    store8(0xa050 + part, 0xff);              /* 405fe */
    n++;
    store8(0xa070 + part, 0x3c);              /* 40603 */
    n++;
    r0 = part;                                /* 40608 MOVG2 r3 r0 */
    MCU_SetStatusCommon(part, 1);
    n++;
    shll16(r0);                               /* 4060a */
    n++;
    shll16(r0);                               /* 4060c */
    n++;
    shll16(r0);                               /* 4060e */
    n++;
    shll16(r0);                               /* 40610 */
    n++;
    r0 = (uint16_t)MCU_ADD_Common(r0, 0x9f50, 0, 1); /* 40612 */
    n++;
    movi(r1, 7);                              /* 40616 */
    n++;
    for (int k = 0; k < 8; k++)               /* 40619 CLR --r0 x8 */
    {
        r0 = (uint16_t)(r0 - 2);
        clear16(r0);
        n++;
        r1 = (uint16_t)(r1 - 1);              /* 4061b cntjmp r1 -5 */
        n++;
    }
    movi(r1, 0x000f);                         /* 4061e */
    n++;
    return n;                                 /* 27 */
}

/* ---- the four L1 blocks -------------------------------------------------- */

/* 0x40462-0x404b0 (22): seed r1=27, r2=0, r3 low=0xff and slot 27's first
 * half. Ends at the first observed interrupt boundary. */
uint32_t routine_pool_init_a(void)
{
    uint32_t n = 0;
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];
    uint16_t &r3 = mcu.r[3];

    movi(r1, 0x001b);                         /* 40462 */
    n++;
    r2 = 0;                                   /* 40465 XOR r2 r2 */
    MCU_SetStatusCommon(0, 1);
    n++;
    move8(r3, 0xff);                          /* 40467 */
    n++;

    n += loop_a_first(r1, r2, r3);            /* 40469-404b0 */
    mcu.pc = 0x04b4u;                         /* host polls here (FRT2) */
    return n;
}

/* 0x404b4-0x4061e: finish loop A, build the free list and the descriptor
 * pool, run the part table and the part helper's first loop, then the first
 * part body. Ends at the 0x40621 poll point before the r2 fill loop. */
uint32_t routine_pool_init_b(void)
{
    uint32_t n = 0;
    uint16_t &r0 = mcu.r[0];
    uint16_t &r1 = mcu.r[1];
    uint16_t &r2 = mcu.r[2];
    uint16_t &r3 = mcu.r[3];
    bool more;

    n += loop_a_tail(r1, r2, more);           /* 404b4-404c2, slot 27 */
    while (more)                              /* slots 26..0 */
    {
        n += loop_a_first(r1, r2, r3);
        n += loop_a_tail(r1, r2, more);
    }

    movi(r3, 0x000f);                         /* 404c5 */
    n++;
    movi(r2, 0x001b);                         /* 404c8 */
    n++;
    r1 = r2;                                  /* 404cb MOVG2 r2 r1 */
    MCU_SetStatusCommon(r2, 1);
    n++;
    store16(0xa432, r3);                      /* 404cd word seed */
    n++;
    store16(0xa434, r2);                      /* 404d1 word seed */
    n++;
    store16(0xa436, r1);                      /* 404d5 word seed */
    n++;
    movi(r1, 0x001b);                         /* 404d9 */
    n++;

    /* Loop B 404dc-404ef: chain sentinels, 28 iterations. */
    for (;;)
    {
        store8(0xa3f4 + r1, 0xff);            /* 404dc */
        n++;
        store8(0xa410 + r1, 0xff);            /* 404e1 */
        n++;
        clear8(0xa3bc + r1);                  /* 404e6 */
        n++;
        store8(0xa3d8 + r1, 0xff);            /* 404ea */
        n++;
        r1 = (uint16_t)(r1 - 1);              /* 404ef cntjmp r1 -22 */
        n++;
        if (r1 == 0xffff)
            break;
    }

    /* Free-list seed 404f2-40508. */
    clear16(0xa1f0);                          /* 404f2 CLR word */
    n++;
    clear8(0xa1df);                           /* 404f6 */
    n++;
    store8(0xa42f, 0xff);                     /* 404fa */
    n++;
    store8(0xa430, 0xff);                     /* 404ff */
    n++;
    clear8(0xa42d);                           /* 40504 */
    n++;
    movi(r1, 0x001b);                         /* 40508 */
    n++;

    /* Loop C 4050b-40536: free list a42f = N-1 -> .. -> 0, a430 = 0,
     * a42d = N. Empty only on the first iteration (a430 = 0xff). */
    r0 = 0;                                   /* 4050b CLR r0 */
    flags_clr();
    n++;
    for (;;)
    {
        uint8_t tail = MCU_Read(0xa430);      /* 4050d MOVG2 a430 -> r0 */
        r0 = (uint16_t)((r0 & 0xff00u) | tail);
        MCU_SetStatusCommon(tail, 0);
        n++;
        n++;                                  /* 40511 BPL */
        if ((tail & 0x80u) == 0)
        {
            store8(0xa3d8 + tail, (uint8_t)r1);      /* 4051d a3d8[tail]=r1 */
            n++;
            store8(0xa3d8 + (uint8_t)r1, 0xff);      /* 40521 a3d8[r1]=ff */
            n++;
        }
        else
        {
            store8(0xa42f, (uint8_t)r1);             /* 40513 a42f=r1 */
            n++;
            store8(0xa3d8 + (uint8_t)r1, tail);      /* 40517 a3d8[r1]=tail */
            n++;
            n++;                                     /* 4051b BRA */
        }
        store8(0xa430, (uint8_t)r1);          /* 40526 */
        n++;
        store8(0xa3a0 + (uint8_t)r1, 0x94);   /* 4052a free */
        n++;
        addq8(0xa42d, 1);                     /* 4052f */
        n++;
        r1 = (uint16_t)(r1 - 1);              /* 40533 cntjmp r1 -41 */
        n++;
        if (r1 == 0xffff)
            break;
    }
    store8(0xa42e, 0xff);                     /* 40536 */
    n++;

    /* Loop D 4053b-40560: descriptor pool, r2 = 27..0 (seeded at 404c8). */
    for (;;)
    {
        uint8_t head = MCU_Read(0xa42e);      /* 4053b MOVG2 a42e -> r0 */
        r0 = (uint16_t)((r0 & 0xff00u) | head);
        MCU_SetStatusCommon(head, 0);
        n++;
        store8(0xa250 + r2, head);            /* 4053f a250[r2]=head */
        n++;
        store8(0xa42e, (uint8_t)r2);          /* 40543 a42e=r2 */
        n++;
        store8(0xa288 + r2, 0x94);            /* 40547 a288=0x94 */
        n++;
        store8(0xa26c + r2, 0xff);            /* 4054c a26c=0xff */
        n++;
        store8(0xa314 + r2, 0xff);            /* 40551 a314=0xff */
        n++;
        store8(0xa2c0 + r2, 0xff);            /* 40556 a2c0=0xff */
        n++;
        store8(0xa2dc + r2, 0xff);            /* 4055b a2dc=0xff */
        n++;
        r2 = (uint16_t)(r2 - 1);              /* 40560 cntjmp r2 -40 */
        n++;
        if (r2 == 0xffff)
            break;
    }
    r2 = 0;                                   /* 40563 CLR r2 */
    flags_clr();
    n++;

    /* Loop E 40565-40581: part table, r3 = 15..0 (seeded at 404c5). */
    for (;;)
    {
        clear8(0xa210 + r3);                  /* 40565 a210=0 */
        n++;
        store8(0xa220 + r3, 0xff);            /* 40569 */
        n++;
        store8(0xa230 + r3, 0xff);            /* 4056e */
        n++;
        {                                     /* 40573 BCLR a240[r3].0 */
            uint8_t data = MCU_Read(0xa240 + r3);
            MCU_SetStatus((data & 1u) == 0, STATUS_Z);
            MCU_Write(0xa240 + r3, (uint8_t)(data & 0xfeu));
        }
        n++;
        store8(0xa200 + r3, 0xff);            /* 40577 MOVG #0x00ff (byte) */
        n++;
        r0 = (uint16_t)((r0 & 0xff00u) | (uint8_t)r3); /* 4057d MOVG2 r3 r0 */
        MCU_SetStatusCommon((uint8_t)r3, 0);
        n++;
        addq8_low(r3, -1);                    /* 4057f ADDQ #-1 r3 */
        n++;
        n++;                                  /* 40581 BPL -> 40565 */
        if (r3 & 0x80u)
            break;
    }

    MCU_PushStack(0x0586);                    /* 40583 bsr16 -> 40588 */
    n++;

    /* Helper loop 1 40588-405f1: part record init, 16 parts. The 405a6 arm
     * (r4 >= 0xe0) is not in dasm_full; bytes build: LDC #2 r4, a1f4=2,
     * SUB #0xe0 r4 (build/rom2.bin 0x5a6). */
    movi(r3, 0x000f);                         /* 40588 */
    n++;
    for (;;)
    {
        shll8(r3);                            /* 4058b r3 = 2*part */
        n++;
        load16(mcu.r[4], 0xabde + r3);        /* 4058d r4 = word[abde+2p] */
        n++;
        store16(0xa1b0 + r3, mcu.r[4]);       /* 40591 a1b0[2p]=r4 */
        n++;
        shlr8(r3);                            /* 40595 */
        n++;
        (void)MCU_SUB_Common(mcu.r[4], 0x00e0, 0, 1); /* 40597 cmp r4,#0xe0 */
        n++;
        bool ge_e0 = (mcu.sr & STATUS_C) == 0; /* 4059a BCC: C==0 if r4>=0xe0 */
        n++;
        if (ge_e0)
        {
            mcu.ep = 2;                       /* 405a6 LDC #2 r4 (ep=2) */
            n++;
            store8(0xa1f4, 2);                /* 405a9 */
            n++;
            mcu.r[4] = (uint16_t)MCU_SUB_Common(mcu.r[4], 0x00e0, 0, 1); /* 405ae */
            n++;
        }
        else
        {
            mcu.ep = 1;                       /* 4059c LDC #1 r4 (ep=1) */
            n++;
            store8(0xa1f4, 1);                /* 4059f */
            n++;
            n++;                              /* 405a4 BRA -> 405b2 */
        }
        load8(r0, 0xa1f4);                    /* 405b2 MOVG2 a1f4 -> r0 */
        n++;
        store8(0xa43c + r3, (uint8_t)r0);     /* 405b6 */
        n++;
        r0 = mcu.r[4];                        /* 405ba MOVG2 r4 r0 */
        MCU_SetStatusCommon(mcu.r[4], 1);
        n++;
        mulxu_d8(r0, r1, r0);                 /* 405bc MULXU #0xd8 r0:r1 */
        n++;
        r1 = (uint16_t)MCU_ADD_Common(r1, 0, 0, 1); /* 405c0 ADD #0 r1 */
        n++;
        shll8(r3);                            /* 405c4 r3 = 2*part */
        n++;
        store16(0xa44c + r3, r1);             /* 405c6 a44c[2p]=r1 */
        n++;
        shlr8(r3);                            /* 405ca */
        n++;
        mulxu_d8(mcu.r[4], mcu.r[5], mcu.r[4]); /* 405cc MULXU #0xd8 r4:r5 */
        n++;
        mcu.r[5] = (uint16_t)MCU_ADD_Common(mcu.r[5], 0, 0, 1); /* 405d0 */
        n++;
        move8(r0, 0);                         /* 405d4 */
        n++;
        {                                     /* 405d6 BTSTI @r5+13, #0 (page ep) */
            uint32_t addr = ((uint32_t)mcu.ep << 16) | (uint16_t)(mcu.r[5] + 13);
            uint8_t data = MCU_Read(addr);
            MCU_SetStatus((data & 1u) == 0, STATUS_Z);
        }
        n++;
        bool bit_set = (mcu.sr & STATUS_Z) == 0;
        n++;                                  /* 405d9 BEQ -> 405de */
        if (bit_set)
        {
            move8(r0, 2);                     /* 405dc */
            n++;
        }
        store8(0xa040 + r3, (uint8_t)r0);     /* 405de a040[p]=r0 */
        n++;
        store8(0xa080 + r3, 0x40);            /* 405e2 */
        n++;
        shll16(r3);                           /* 405e7 r3 = 2*part */
        n++;
        store16(0xa190 + r3, 0x3c3c);         /* 405e9 a190[2p]=0x3c3c */
        n++;
        shlr16(r3);                           /* 405ef */
        n++;
        r3 = (uint16_t)(r3 - 1);              /* 405f1 cntjmp r3 -105 */
        n++;
        if (r3 == 0xffff)
            break;
    }

    movi(r3, 0x000f);                         /* 405f4 */
    n++;
    movi(r2, 0xa090);                         /* 405f7 */
    n++;
    n += helper_loop2_body(r3);               /* 405fa-4061e, first part */
    mcu.pc = 0x0621u;                         /* poll point before fill loop */
    return n;
}

/* 0x40621: MOVG #0xff -> @r2++ (page dp), one instruction. Its own block so
 * the host can poll at both 0x40624 and 0x40621 inside the 16x16 fill loop. */
uint32_t routine_pool_init_c(void)
{
    uint16_t &r2 = mcu.r[2];
    MCU_Write(((uint32_t)mcu.dp << 16) | r2, 0xff);
    MCU_SetStatusCommon(0xff, 0);
    r2 = (uint16_t)(r2 + 1);
    mcu.pc = 0x0624u;
    return 1;
}

/* 0x40624 cntjmp r1 -6: run the fill loop one step per host entry so the
 * interrupt poll points match stock; when r1 wraps, cntjmp r3 either starts
 * the next part body (29) or ends the helper with rts (3) plus the pool-init
 * ret (4 when the stack returns to 0x586). */
uint32_t routine_pool_init_d(void)
{
    uint32_t n = 1;                           /* 40624 cntjmp r1 -6 */
    uint16_t &r1 = mcu.r[1];
    uint16_t &r3 = mcu.r[3];

    r1 = (uint16_t)(r1 - 1);
    if (r1 != 0xffff)
    {
        mcu.pc = 0x0621u;
        return n;
    }

    n++;                                      /* 40627 cntjmp r3 -48 */
    r3 = (uint16_t)(r3 - 1);
    if (r3 != 0xffff)
    {
        n += helper_loop2_body(r3);           /* 405fa-4061e, next part */
        mcu.pc = 0x0621u;
        return n;
    }

    n++;                                      /* 4062a rts */
    mcu.pc = MCU_PopStack();
    if (mcu.pc == 0x0586u)
    {
        n++;                                  /* 40586 ret (pop cp,pc): only the
                                               * pjsr path (0x565/0x721) returns
                                               * to the bsr's 0x40586; other
                                               * callers (jsr #0x0588 at
                                               * 0x2f86/0x3115/0x5778a) resume at
                                               * their own PC */
        mcu.cp = (uint8_t)MCU_PopStack();
        mcu.pc = MCU_PopStack();
    }
    return n;
}

} /* anonymous namespace */

void pool_post_reset(void)
{
    /* Pool-init is translated from ROM and writes page-0 SRAM directly; there
     * is no native pool copy left to reset. Kept as the HandPostReset hook. */
}

} /* namespace mk2c */

/* ---- hand integration hooks ----------------------------------------------- */

void MK2CPP_PoolPostReset(void)
{
    mk2c::pool_post_reset();
}

void MK2CPP_PoolFillTables(void)
{
    MK2CPP_HandRegisterRoutine(0x00040462u, &mk2c::routine_pool_init_a);
    MK2CPP_HandRegisterRoutine(0x000404b4u, &mk2c::routine_pool_init_b);
    MK2CPP_HandRegisterRoutine(0x00040621u, &mk2c::routine_pool_init_c);
    MK2CPP_HandRegisterRoutine(0x00040624u, &mk2c::routine_pool_init_d);

    /* Whole-routine alloc/free hooks (native_allocfree.cpp). */
    MK2CPP_AllocFreeFillTables();
}
