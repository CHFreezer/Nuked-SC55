/*
 * HAND voice/native_allocfree -- whole-routine alloc/free (M4 Wave 2c).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * Replaces five rom1 routines as one MK2CPP_Step each (L1):
 *
 *   0x19ad pool_pop     pop the free-list head -> r1
 *   0x194c link         append slot to a descriptor's voice chain
 *   0x1823 release A    pcm_start + detach + push free-list tail
 *   0x187e release B    same, push free-list head
 *   0x19c4 free_voice   cleanup an assigned voice (two early exits)
 *
 * The internal helpers (H1 0x1b44, H2 0x1bad, H3 0x1b90, unlink 0x1b23,
 * pcm_start 0x516c) are translated below as named static functions. Each
 * routine returns the exact number of H8 instructions the stock path would
 * execute (own instructions plus inlined helper bodies); MK2CPP_Step then
 * adds 12*(n-1) on top of its own +12, so mcu.cycles match the stock run.
 *
 * Interop: every field lives at its real page-0 address and is accessed with
 * MCU_Read/MCU_Write, so un-translated ROM observes byte-identical SRAM. The
 * operations are written as ordinary list manipulation, not per-instruction
 * shadow updates. Semantics and path counts: out/m4/17_alloc_free_semantics.md.
 *
 * The ROM's IML=7 window around pcm_start is applied to mcu.sr directly; no
 * interrupt poll runs inside the block (the host polls once after it). This
 * is the documented L1 trade-off (17 5); pool-init stays L0 because it was
 * observed to be interrupted at boot.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as pcm_enable.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

const uint8_t kNone = 0xff;

/* Page-0 addresses of the SoA arrays / scalars the routines touch. Names keep
 * the firmware addresses so the translation is checkable against the dasm. */
enum {
    kA42C = 0xa42c, kA42D = 0xa42d, kA42E = 0xa42e, kA42F = 0xa42f, kA430 = 0xa430,
    /* per voice */
    kA3A0 = 0xa3a0, kA3BC = 0xa3bc, kA3D8 = 0xa3d8, kA3F4 = 0xa3f4,
    kA410 = 0xa410, kA4B4 = 0xa4b4, kAd0e = 0xad0e,
    kA368 = 0xa368, kA384 = 0xa384, kA34C = 0xa34c,
    kD0A8 = 0xd0a8, kD0C4 = 0xd0c4, kD0E0 = 0xd0e0, kD15C = 0xd15c,
    /* per descriptor */
    kA250 = 0xa250, kA26C = 0xa26c, kA288 = 0xa288, kA2A4 = 0xa2a4,
    kA2C0 = 0xa2c0, kA2DC = 0xa2dc, kA314 = 0xa314,
    /* per part */
    kA200 = 0xa200, kA210 = 0xa210, kA220 = 0xa220, kA230 = 0xa230,
};

/* AoS slot -> page-0 record pointer, ROM1 big-endian word table (28 entries). */
const uint32_t kAosPtrTable = 0x64d6;

uint8_t arr_r8(uint32_t base, uint8_t idx) { return MCU_Read(base + idx); }
void arr_w8(uint32_t base, uint8_t idx, uint8_t value) { MCU_Write(base + idx, value); }

/* Byte load into a 16-bit register, upper byte preserved (MOVG2 semantics). */
void set_reg_lo(uint16_t &reg, uint8_t value)
{
    reg = (uint16_t)((reg & 0xff00u) | value);
}

/* ADDQ #delta on a byte in memory: store result, set N/Z/V/C exactly like the
 * GT ADDQ path (MCU_ADD_Common). delta is +1 (0x186c, 0x18bc, 0x1a1c) or -1
 * (0x1873, 0x18c3, 0x19bf, 0x1bf9). */
uint8_t addq_byte(uint32_t addr, int delta)
{
    int32_t value = MCU_ADD_Common((int32_t)MCU_Read(addr), delta, 0, 0);
    MCU_Write(addr, (uint8_t)value);
    return (uint8_t)value;
}

/* CLR flags: N=0, Z=1, V=0, C=0. */
void set_flags_clr(void)
{
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

/* H1 0x1b44: detach `slot` from descriptor `desc`'s voice chain and from the
 * parallel d0a8/d0c4 start chain. The "prev" arm deliberately leaves a2dc
 * alone (stock branches past 0x1b89). Returns 11/12/13 instructions. */
uint32_t h1_unlink_voice(uint8_t slot, uint8_t desc)
{
    uint8_t prev = arr_r8(kA3F4, slot);
    if (prev < 0x80)
    {
        arr_w8(kA2C0, desc, prev);       /* 1b4e */
        arr_w8(kA410, prev, kNone);      /* 1b52 */
        arr_w8(kD0A8, prev, kNone);      /* 1b57 */
        arr_w8(kA3F4, slot, kNone);      /* 1b5c */
        arr_w8(kD0C4, slot, kNone);      /* 1b61 */
        return 12;
    }
    uint8_t next = arr_r8(kA410, slot);
    if (next >= 0x80)
    {
        arr_w8(kA2C0, desc, kNone);      /* 1b6e */
        arr_w8(kA2DC, desc, kNone);      /* 1b89 */
        return 11;
    }
    arr_w8(kA410, slot, kNone);          /* 1b75 */
    arr_w8(kD0A8, slot, kNone);          /* 1b7a */
    arr_w8(kA3F4, next, kNone);          /* 1b7f */
    arr_w8(kD0C4, next, kNone);          /* 1b84 */
    arr_w8(kA2DC, desc, next);           /* 1b89 */
    return 13;
}

/* unlink 0x1b23: remove `desc` from its part's active descriptor chain. The
 * second branch is taken on the flags left by the first MOVG3 store, i.e. it
 * tests `next`, not `prev`. Returns 7/8/8/9 instructions. */
uint32_t unlink_desc(uint8_t desc, uint8_t part)
{
    uint8_t next = arr_r8(kA250, desc);
    uint8_t prev = arr_r8(kA26C, desc);
    uint32_t n = 3;                              /* 1b23,1b27,1b2b */
    if (prev < 0x80)
    {
        arr_w8(kA250, prev, next);               /* 1b33 */
        n += 2;                                  /* 1b33,1b37 */
    }
    else
    {
        arr_w8(kA220, part, next);               /* 1b2d */
        n += 3;                                  /* 1b2d,1b31,1b37 */
    }
    if (next < 0x80)
    {
        arr_w8(kA26C, next, prev);               /* 1b3f */
        n += 1;
    }
    else
    {
        arr_w8(kA230, part, prev);               /* 1b39 */
        n += 2;                                  /* 1b39,1b3d */
    }
    return n + 1;                                /* 1b43 rts */
}

/* H3 0x1b90: reset a descriptor's state and keep a200[part] on the smallest
 * a314 (GT CMP: BHI skips while the existing a314 is larger). Returns 6/8/9. */
uint32_t h3_init_desc(uint8_t desc, uint8_t part)
{
    arr_w8(kA288, desc, 0);                      /* 1b90 */
    arr_w8(kA2A4, desc, 0);                      /* 1b94 */
    uint8_t current = arr_r8(kA200, part);       /* 1b98 */
    if (current < 0x80)
    {
        uint8_t age = arr_r8(kA314, desc);
        if (age > arr_r8(kA314, current))        /* 1b9e,1ba2,1ba6 BHI */
            return 8;
    }
    arr_w8(kA200, part, desc);                   /* 1ba8 */
    return (current < 0x80) ? 9u : 6u;
}

/* H2 0x1bad: finish one released voice. If the descriptor's voice chain is now
 * empty, unlink it, push it on the descriptor free-list and re-scan the part
 * chain for the smallest a314. a210[part] is decremented on every path. The
 * ROM's r0 scratch result is returned through `r0`. */
uint32_t h2_finish_desc(uint8_t desc, uint8_t part, uint16_t &r0)
{
    uint32_t n;
    if (arr_r8(kA2C0, desc) < 0x80)
    {
        n = 2;                                   /* 1bad,1bb1 */
    }
    else
    {
        n = 9 + unlink_desc(desc, part);         /* 1bb3 bsr + 1bb6..1bcb */
        uint8_t old_head = MCU_Read(kA42E);
        set_reg_lo(r0, old_head);                /* 1bb6 */
        arr_w8(kA250, desc, old_head);           /* 1bba */
        MCU_Write(kA42E, desc);                  /* 1bbe */
        arr_w8(kA288, desc, 0x94);               /* 1bc2 */
        uint8_t current = arr_r8(kA200, part);   /* 1bc7 */
        if (current < 0x80)
        {
            if (current != desc)
            {
                n += 2;                          /* 1bcd,1bd1 */
            }
            else
            {
                n += 6;                          /* 1bcd..1bdb */
                r0 = 0x7f;                       /* 1bd5 move r0 #0x7f */
                uint8_t d = arr_r8(kA220, part); /* 1bd7 */
                if (d >= 0x80)
                {
                    arr_w8(kA200, part, d);      /* 1bf3 */
                    n += 2;                      /* 1bf3,1bf7 */
                }
                else
                {
                    for (;;)
                    {
                        n += 2;                  /* 1bdd CMP, 1be1 BLS */
                        if (arr_r8(kA314, d) < (uint8_t)r0)
                        {
                            r0 = arr_r8(kA314, d);          /* 1be3 */
                            arr_w8(kA200, part, d);         /* 1be7 */
                            n += 2;
                        }
                        uint8_t next = arr_r8(kA250, d);    /* 1beb */
                        n += 2;                  /* 1beb,1bef BPL */
                        if (next >= 0x80)
                            break;
                        d = next;
                    }
                    n += 2;                      /* 1bf1,1bf7 */
                }
            }
        }
    }
    addq_byte(kA210 + part, -1);                 /* 1bf9 (all paths) */
    return n + 2;                                /* 1bf9,1bfd rts */
}

/* pcm_start 0x516c: select the slot's PCM channel, read back the routing
 * registers and program the three state words plus one port byte. `r1` is the
 * raw H8 register (release A has not EXTU'ed it yet); r6 becomes 0x12/0x14 and
 * the caller's ldm restores r1..r5. Returns 17/18 instructions. */
uint32_t pcm_start(uint16_t r1)
{
    MCU_Write((uint32_t)(uint16_t)(r1 + kD15C), 0);       /* 516c */
    MCU_Write(0xe03eu, (uint8_t)r1);                      /* 5170 select */

    /* 5174/5178: byte read of 0xe032 latches ram2[sel][9]; the following WORD
     * read of 0xe03a replaces r4 with the full read_latch value. Same for r5
     * via 0xe034 (ram2[sel][10]). */
    (void)MCU_Read(0xe032u);
    uint16_t r4 = MCU_Read16(0xe03au);
    (void)MCU_Read(0xe034u);
    uint16_t r5 = MCU_Read16(0xe03au);

    uint16_t index = (uint16_t)(r1 + r1);                 /* 5184 ADD r1 r1 */
    uint16_t aos = MCU_Read16((uint32_t)(uint16_t)(index + kAosPtrTable)); /* 5186 */

    /* 518a CMP r4 r5: GT subtracts r4 from r5; BCC (C==0) iff r5 >= r4. */
    if (r5 >= r4)
    {
        mcu.r[6] = 0x14;                                  /* 519e */
        MCU_Write16(0xe018u, 0x00b5u);                    /* 51a1 */
        MCU_Write16((uint32_t)(uint16_t)(aos + 30u), 0x00b5u);  /* 51a7 */
        MCU_Write16((uint32_t)aos, 0x0014u);              /* 51ac..51b2 */
        return 17;
    }
    mcu.r[6] = 0x12;                                      /* 518e */
    MCU_Write16(0xe016u, 0x00b5u);                        /* 5191 */
    MCU_Write16((uint32_t)(uint16_t)(aos + 26u), 0x00b5u);  /* 5197 */
    MCU_Write16((uint32_t)aos, 0x0012u);                  /* 51ac..51b2 */
    return 18;
}

/* Push `slot` on the free-list tail (release A 0x184a-0x1863, free_voice
 * 0x19fa-0x1a13). Loads a430 into r0 first (MOVG2 byte load, upper byte
 * preserved); the caller counts the BPL (already included in its own `n`). */
uint32_t free_push_tail(uint8_t slot, uint16_t &r0)
{
    uint8_t tail = arr_r8(kA430, 0);
    set_reg_lo(r0, tail);                    /* 184a / 19fa */
    if (tail < 0x80)
    {
        arr_w8(kA3D8, tail, slot);           /* 185a / 1a0a */
        arr_w8(kA3D8, slot, kNone);          /* 185e / 1a0e */
        arr_w8(kA430, 0, slot);              /* 1863 / 1a13 */
        return 3;
    }
    arr_w8(kA42F, 0, slot);                  /* 1850 / 1a00 */
    arr_w8(kA3D8, slot, tail);               /* 1854 / 1a04 */
    arr_w8(kA430, 0, slot);                  /* 1863 / 1a13 */
    return 4;                                /* + 1858 / 1a08 BRA */
}

/* pool_pop 0x19ad: pop the free-list head into r1, r0 = new head.
 *
 * Split at 0x19c3: the stock ROM is interrupted inside pool_pop in the demo
 * (FRT3 handler taken at the 0x19bf -> 0x19c3 boundary, traced at c337839336;
 * the handler resumes at 0x19c3). The head block runs 0x19ad..0x19bf and
 * leaves pc = 0x19c3 so the host polls before the rts, exactly like stock. */
uint32_t routine_pool_pop_head(void)
{
    uint8_t slot = arr_r8(kA42F, 0);         /* 19ad */
    uint8_t head = arr_r8(kA3D8, slot);      /* 19b1 */
    arr_w8(kA42F, 0, head);                  /* 19b5 */
    uint32_t n = 4;                          /* ..19b9 BPL */
    if (head >= 0x80)
    {
        arr_w8(kA430, 0, head);              /* 19bb list became empty */
        n += 1;
    }
    addq_byte(kA42D, -1);                    /* 19bf */
    n += 1;                                  /* 19bf */
    set_reg_lo(mcu.r[1], slot);
    set_reg_lo(mcu.r[0], head);
    mcu.pc = 0x19c3u;                        /* split point: host polls here */
    return n;
}

uint32_t routine_pool_pop_tail(void)
{
    mcu.pc = MCU_PopStack();                 /* 19c3 rts */
    return 1;
}

/* link 0x194c: append `slot` to descriptor `desc`'s voice chain (fixing the
 * parallel d0a8/d0c4 start chain too), then record slot->part/desc. Returns
 * 15..18 instructions; r0 = old chain tail (or the final d0c4 load). */
uint32_t routine_link(void)
{
    uint8_t slot = (uint8_t)mcu.r[1];
    uint8_t desc = (uint8_t)mcu.r[2];
    uint8_t part = (uint8_t)mcu.r[3];
    uint8_t tail = arr_r8(kA2DC, desc);      /* 194c */
    uint16_t r0 = mcu.r[0];
    uint32_t n;

    set_reg_lo(r0, tail);
    if (tail >= 0x80)
    {
        arr_w8(kA2C0, desc, slot);           /* 1952 */
        n = 16;
        set_reg_lo(r0, arr_r8(kD0A8, slot)); /* 1956 */
        if ((uint8_t)r0 < 0x80)
        {
            arr_w8(kD0C4, (uint8_t)r0, kNone);   /* 195c */
            n += 1;
        }
        set_reg_lo(r0, arr_r8(kD0C4, slot)); /* 1961 */
        if ((uint8_t)r0 < 0x80)
        {
            arr_w8(kD0A8, (uint8_t)r0, kNone);   /* 1967 */
            n += 1;
        }
        arr_w8(kA410, slot, kNone);          /* 196c */
        arr_w8(kD0A8, slot, kNone);          /* 1971 */
    }
    else
    {
        arr_w8(kA2C0, desc, tail);           /* 1978 */
        arr_w8(kA3F4, tail, slot);           /* 197c */
        arr_w8(kD0C4, tail, slot);           /* 1980 */
        arr_w8(kA410, slot, tail);           /* 1984 */
        arr_w8(kD0A8, slot, tail);           /* 1988 */
        arr_w8(kA410, tail, kNone);          /* 198c */
        arr_w8(kD0A8, tail, kNone);          /* 1991 */
        n = 15;
    }
    arr_w8(kA2DC, desc, slot);               /* 1996 */
    arr_w8(kA3F4, slot, kNone);              /* 199a */
    arr_w8(kD0C4, slot, kNone);              /* 199f */
    arr_w8(kA368, slot, part);               /* 19a4 */
    arr_w8(kA384, slot, desc);               /* 19a8 */
    MCU_SetStatusCommon(part, 0);            /* last MOVG3 store sets N/Z/V */
    mcu.r[0] = r0;
    mcu.pc = MCU_PopStack();
    return n;
}

/* release A 0x1823: stop the voice's PCM channel inside the IML=7 window,
 * detach it from its descriptor and push it on the free-list tail. */
uint32_t routine_release_a(void)
{
    uint32_t n = 7;                          /* 1823..1837 incl. jsr/ldm */
    mcu.sr |= 0x0700u;                       /* 1823 BSET_ORC: IML=7 */
    n += pcm_start(mcu.r[1]);                /* 1829 jsr 0x516c */
    MCU_Write((uint32_t)(uint16_t)(mcu.r[1] + kD0E0), 4);  /* 182e d0e0=4 */
    mcu.sr &= (uint16_t)0xf8ffu;             /* 1833 BCLR_ANDC: IML=0 */
    mcu.r[1] = (uint8_t)mcu.r[1];            /* 1837 EXTU r1 */

    uint8_t slot = (uint8_t)mcu.r[1];
    uint8_t desc = (uint8_t)mcu.r[2];
    uint8_t part = (uint8_t)mcu.r[3];

    arr_w8(kAd0e, slot, 0);                  /* 1839 */
    arr_w8(kA3BC, slot, 0);                  /* 183d */
    arr_w8(kA4B4, slot, 0);                  /* 1841 */
    n += 3;

    mcu.r[0] = 0;                            /* 1845 CLR r0 */
    n += 2;                                  /* 1845,1847 bsr H1 */
    n += h1_unlink_voice(slot, desc);

    n += 2;                                  /* 184a,184e BPL */
    n += free_push_tail(slot, mcu.r[0]);     /* 1850..1863 (r0 = tail) */
    arr_w8(kA3A0, slot, 0x94);               /* 1867 free */
    addq_byte(kA42D, 1);                     /* 186c */
    n += 2;                                  /* 1867,186c */

    n += 1 + h2_finish_desc(desc, part, mcu.r[0]); /* 1870 bsr H2 */

    uint8_t shortfall = addq_byte(kA42C, -1);/* 1873 */
    n += 2;                                  /* 1873,1877 BPL */
    if (shortfall & 0x80)
    {
        MCU_Write(kA42C, 0);                 /* 1879 CLR */
        set_flags_clr();
        n += 1;
    }
    n += 1;                                  /* 187d rts */

    set_reg_lo(mcu.r[1], slot);
    mcu.pc = MCU_PopStack();
    return n;
}

/* release B 0x187e: release A variant pushing the slot on the free-list head
 * (LIFO: the next pool_pop hands the just-released voice back first).
 *
 * Unlike release A, the stock ROM is interrupted inside this routine in the
 * demo (FRT3 handler 0x461ac taken at the 0x1894 -> 0x1898 boundary, traced at
 * c333016668; the handler resumes at 0x1898). A single L1 block would defer
 * that interrupt and leave a different return PC in the handler's stack frame,
 * which the state hash sees. The routine is therefore split at 0x1898: the
 * host polls between the two blocks exactly where the ROM did. */
uint32_t routine_release_b_head(void)
{
    mcu.r[1] = (uint8_t)mcu.r[1];            /* 187e EXTU r1 */
    mcu.sr |= 0x0700u;                       /* 1880 BSET_ORC: IML=7 */
    uint32_t n = 7;                          /* 187e..1890 incl. jsr/ldm */
    n += pcm_start(mcu.r[1]);                /* 1886 jsr 0x516c */
    MCU_Write((uint32_t)(uint16_t)(mcu.r[1] + kD0E0), 4);  /* 188b d0e0=4 */
    mcu.sr &= (uint16_t)0xf8ffu;             /* 1890 BCLR_ANDC: IML=0 */
    arr_w8(kAd0e, (uint8_t)mcu.r[1], 0);     /* 1894 */
    n += 1;
    mcu.pc = 0x1898u;                        /* split point: host polls here */
    return n;
}

uint32_t routine_release_b_tail(void)
{
    uint8_t slot = (uint8_t)mcu.r[1];
    uint8_t desc = (uint8_t)mcu.r[2];
    uint8_t part = (uint8_t)mcu.r[3];

    arr_w8(kA3BC, slot, 0);                  /* 1898 */
    arr_w8(kA4B4, slot, 0);                  /* 189c */
    uint32_t n = 3;                          /* 1898,189c,18a0 */
    mcu.r[0] = 0;                            /* 18a0 CLR r0 */
    n += 1;                                  /* 18a2 bsr H1 */
    n += h1_unlink_voice(slot, desc);

    set_reg_lo(mcu.r[0], arr_r8(kA42F, 0));  /* 18a5 r0 = head */
    uint8_t head = (uint8_t)mcu.r[0];
    n += 2;                                  /* 18a5,18a9 BPL */
    if (head >= 0x80)
    {
        arr_w8(kA430, 0, slot);              /* 18ab list was empty */
        n += 1;
    }
    arr_w8(kA3D8, slot, head);               /* 18af */
    arr_w8(kA42F, 0, slot);                  /* 18b3 */
    n += 2;
    arr_w8(kA3A0, slot, 0x94);               /* 18b7 free */
    addq_byte(kA42D, 1);                     /* 18bc */
    n += 2;                                  /* 18b7,18bc */

    n += 1 + h2_finish_desc(desc, part, mcu.r[0]); /* 18c0 bsr H2 */

    uint8_t shortfall = addq_byte(kA42C, -1);/* 18c3 */
    n += 2;                                  /* 18c3,18c7 BPL */
    if (shortfall & 0x80)
    {
        MCU_Write(kA42C, 0);                 /* 18c9 CLR */
        set_flags_clr();
        n += 1;
    }
    n += 1;                                  /* 18cd rts */

    set_reg_lo(mcu.r[1], slot);
    mcu.pc = MCU_PopStack();
    return n;
}

/* free_voice 0x19c4: cleanup an assigned voice. Exits early when the slot is
 * already free (a3a0 bit7) or has no PCM channel (ad0e == 0xff); otherwise
 * clears state, fixes the a34c bits, unlinks and pushes the free-list tail.
 * Unlike release A/B it does not touch a42c. */
uint32_t routine_free_voice(void)
{
    uint8_t slot = (uint8_t)mcu.r[1];
    if (arr_r8(kA3A0, slot) & 0x80)          /* 19c4 BTSTI #7 */
    {
        MCU_SetStatus(0, STATUS_Z);          /* bit set: BTSTI leaves Z=0 */
        mcu.pc = MCU_PopStack();
        return 3;                            /* 19c4,19c8,1a23 */
    }
    if (arr_r8(kAd0e, slot) == 0xff)         /* 19ca SUB/BEQ */
    {
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
        mcu.pc = MCU_PopStack();
        return 5;                            /* 19c4,19c8,19ca,19cf,1a23 */
    }

    set_reg_lo(mcu.r[2], arr_r8(kA384, slot));  /* 19d1 */
    set_reg_lo(mcu.r[3], arr_r8(kA368, slot));  /* 19d5 */
    uint8_t desc = (uint8_t)mcu.r[2];
    uint8_t part = (uint8_t)mcu.r[3];
    arr_w8(kA3BC, slot, 0);                  /* 19d9 */
    arr_w8(kA4B4, slot, 0);                  /* 19dd */
    uint32_t n = 8;                          /* 19c4..19dd */

    if (arr_r8(kA2C0, desc) == slot)         /* 19e1,19e5 */
    {
        arr_w8(kA34C, desc, (uint8_t)(arr_r8(kA34C, desc) & 0xfdu));  /* 19e7 */
        n += 3;
    }
    else
    {
        n += 2;
    }
    if (arr_r8(kA2DC, desc) == slot)         /* 19eb,19ef */
    {
        arr_w8(kA34C, desc, (uint8_t)(arr_r8(kA34C, desc) & 0xfeu));  /* 19f1 */
        n += 3;
    }
    else
    {
        n += 2;
    }

    mcu.r[0] = 0;                            /* 19f5 CLR r0 */
    n += 2;                                  /* 19f5,19f7 bsr H1 */
    n += h1_unlink_voice(slot, desc);

    n += 2;                                  /* 19fa,19fe BPL */
    n += free_push_tail(slot, mcu.r[0]);     /* 1a00..1a13 (r0 = tail) */
    arr_w8(kA3A0, slot, 0x94);               /* 1a17 free */
    addq_byte(kA42D, 1);                     /* 1a1c */
    n += 2;                                  /* 1a17,1a1c */

    n += 1 + h2_finish_desc(desc, part, mcu.r[0]); /* 1a20 bsr H2 */

    n += 1;                                  /* 1a23 rts */
    mcu.pc = MCU_PopStack();
    return n;
}

} /* anonymous namespace */
} /* namespace mk2c */

void MK2CPP_AllocFreeFillTables(void)
{
    MK2CPP_HandRegisterRoutine(0x000019adu, &mk2c::routine_pool_pop_head);
    MK2CPP_HandRegisterRoutine(0x000019c3u, &mk2c::routine_pool_pop_tail);
    MK2CPP_HandRegisterRoutine(0x0000194cu, &mk2c::routine_link);
    MK2CPP_HandRegisterRoutine(0x00001823u, &mk2c::routine_release_a);
    MK2CPP_HandRegisterRoutine(0x0000187eu, &mk2c::routine_release_b_head);
    MK2CPP_HandRegisterRoutine(0x00001898u, &mk2c::routine_release_b_tail);
    MK2CPP_HandRegisterRoutine(0x000019c4u, &mk2c::routine_free_voice);

    /* H3 is only called by desc_setup (ROM, not yet native); keep the
     * translation linked so the next slice can reuse it. */
    (void)mk2c::h3_init_desc;
    (void)mk2c::pcm_start;
}
