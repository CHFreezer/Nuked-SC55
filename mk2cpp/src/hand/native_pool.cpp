/*
 * HAND voice/native_pool -- native voice pool (M4 slice-2).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * Wave 2a: data model + SRAM mirror import/commit + post-reset configuration.
 * Wave 2b: L0 override of the pool-init routine (0x40462-0x4062a: seeds
 * 0x40462/0x404d9/0x40508/0x40563, loops A-E and the part-table helper).
 * The stock instruction stream still builds the SRAM mirror for the legacy
 * 28 slots; the native copy is filled for [0, voices). Slots >= kLegacy live in
 * separate native chains (free_head_ext/desc_head_ext) so no un-replaced ROM
 * consumer can ever see them (spec 2.1 / blocker B5); pool_check_legacy()
 * asserts this at the end of pool-init.
 *
 * Gate: these PCs are only reached under -mk2cpp; -mk2cpp-hand:0 skips the whole
 * hand table (mk2cpp.cpp MK2CPP_Step), so this file is inert then and the stock
 * interpreter runs. n=28 equivalence is verified by two_mode_check boot and
 * demo200 (trace + full hash), and the native/SRAM check prints
 * "pool-init self-check OK" with MK2CPP_POOL_VERBOSE=1.
 *
 * Access rule (spec 3): all SRAM traffic goes through MCU_Read/MCU_Write so the
 * GT address decode stays in charge; hand code never touches the (static) sram[]
 * array directly and never calls generated code.
 *
 * SRAM bases are page-0 flat addresses (dp=0); word arrays are scaled by the
 * firmware with SHLL (index*2), byte arrays with the slot index. Evidence:
 * tools/baselines/dasm_full.txt 0x40469-0x40560 and tools/docs/voice_memory_map.md.
 */
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_opcodes.h"

#include "native_pool.h"

namespace mk2c {

Pool g_pool;

int pool_configured(void)
{
    return g_pool.voices >= (uint8_t)kLegacy;
}

/* ---- field -> SRAM map (import/commit/check) ----------------------------- */

#define MAP_MIRROR 0x01u /* import/commit with the SRAM page-0 arrays */
#define MAP_INIT   0x02u /* pool-init (loops A-D) is responsible for it */

struct ByteMap {
    uint16_t off;   /* byte offset inside PoolSlot / PoolDesc */
    uint16_t base;  /* page-0 flat base of the H8 byte array */
    uint8_t  flags;
};

struct WordMap {
    uint16_t off;   /* byte offset of the uint16 inside PoolSlot */
    uint16_t base;  /* page-0 flat base of the H8 word array (index*2) */
    uint8_t  flags;
};

/* PoolSlot byte fields (spec 2.2/2.3). irq_pend/part/desc are native copies but
 * pool-init does not initialize d15c/a368/a384, so they are mirror-only. */
static const ByteMap kSlotBytes[] = {
    { offsetof(PoolSlot, flags),      0xa3a0u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, pcm_ch),     0xad0eu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, st_a3bc),    0xa3bcu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, active),     0xa4b4u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, cmd),        0xd0e0u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, irq_pend),   0xd15cu, MAP_MIRROR },
    { offsetof(PoolSlot, part),       0xa368u, MAP_MIRROR },
    { offsetof(PoolSlot, desc),       0xa384u, MAP_MIRROR },
    { offsetof(PoolSlot, prev),       0xa3f4u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, next),       0xa410u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, start_prev), 0xd0a8u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, start_next), 0xd0c4u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, ce5c),       0xce5cu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, ce78),       0xce78u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, ce94),       0xce94u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, d118),       0xd118u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, d134),       0xd134u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, ceb0),       0xceb0u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, cf20),       0xcf20u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, cf3c),       0xcf3cu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, cf90),       0xcf90u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, d0fc),       0xd0fcu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, ce3f),       0xce3fu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, f_a34c),     0xa34cu, MAP_MIRROR | MAP_INIT },
};

/* PoolSlot word fields (H8 arrays are indexed with index*2). */
static const WordMap kSlotWords[] = {
    { offsetof(PoolSlot, cfac), 0xcfacu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, d000), 0xd000u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, d054), 0xd054u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolSlot, a46c), 0xa46cu, MAP_MIRROR | MAP_INIT },
};

/* PoolDesc byte fields; a2f8 is written by desc_setup, not by pool-init. */
static const ByteMap kDescBytes[] = {
    { offsetof(PoolDesc, next),     0xa250u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolDesc, prev),     0xa26cu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolDesc, state),    0xa288u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolDesc, vhead),    0xa2c0u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolDesc, vtail),    0xa2dcu, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolDesc, part_age), 0xa314u, MAP_MIRROR | MAP_INIT },
    { offsetof(PoolDesc, f2f8),     0xa2f8u, MAP_MIRROR },
};

#define ARRAY_N(a) ((int)(sizeof(a) / sizeof((a)[0])))

static uint8_t *slot_at(int i)
{
    return (uint8_t *)(void *)&g_pool.v[i];
}

static uint8_t *desc_at(int i)
{
    return (uint8_t *)(void *)&g_pool.d[i];
}

static uint16_t *slot_word_at(int i, uint16_t off)
{
    return (uint16_t *)(void *)(slot_at(i) + off);
}

static uint8_t sram_r8(uint32_t flat)
{
    return MCU_Read(flat);
}

static uint16_t sram_r16(uint32_t flat)
{
    return MCU_Read16(flat);
}

static void sram_w8(uint32_t flat, uint8_t value)
{
    MCU_Write(flat, value);
}

static void sram_w16(uint32_t flat, uint16_t value)
{
    MCU_Write16(flat, value);
}

/* ---- import / commit ------------------------------------------------------ */

void pool_import_legacy(void)
{
    int i, k;
    for (i = 0; i < kLegacy; i++)
    {
        uint8_t *s = slot_at(i);
        uint8_t *d = desc_at(i);
        for (k = 0; k < ARRAY_N(kSlotBytes); k++)
        {
            if (kSlotBytes[k].flags & MAP_MIRROR)
                s[kSlotBytes[k].off] = sram_r8((uint32_t)kSlotBytes[k].base + (uint32_t)i);
        }
        for (k = 0; k < ARRAY_N(kSlotWords); k++)
        {
            if (kSlotWords[k].flags & MAP_MIRROR)
                *slot_word_at(i, kSlotWords[k].off) =
                    sram_r16((uint32_t)kSlotWords[k].base + 2u * (uint32_t)i);
        }
        for (k = 0; k < ARRAY_N(kDescBytes); k++)
        {
            if (kDescBytes[k].flags & MAP_MIRROR)
                d[kDescBytes[k].off] = sram_r8((uint32_t)kDescBytes[k].base + (uint32_t)i);
        }
    }
    g_pool.free_head = sram_r8(0xa42fu);
    g_pool.free_tail = sram_r8(0xa430u);
    g_pool.count     = sram_r8(0xa42du);
    g_pool.shortfall = (int8_t)sram_r8(0xa42cu);
    g_pool.desc_head = sram_r8(0xa42eu);
}

void pool_commit_legacy(void)
{
    int i, k;
    for (i = 0; i < kLegacy; i++)
    {
        uint8_t *s = slot_at(i);
        uint8_t *d = desc_at(i);
        for (k = 0; k < ARRAY_N(kSlotBytes); k++)
        {
            if (kSlotBytes[k].flags & MAP_MIRROR)
                sram_w8((uint32_t)kSlotBytes[k].base + (uint32_t)i, s[kSlotBytes[k].off]);
        }
        for (k = 0; k < ARRAY_N(kSlotWords); k++)
        {
            if (kSlotWords[k].flags & MAP_MIRROR)
                sram_w16((uint32_t)kSlotWords[k].base + 2u * (uint32_t)i,
                         *slot_word_at(i, kSlotWords[k].off));
        }
        for (k = 0; k < ARRAY_N(kDescBytes); k++)
        {
            if (kDescBytes[k].flags & MAP_MIRROR)
                sram_w8((uint32_t)kDescBytes[k].base + (uint32_t)i, d[kDescBytes[k].off]);
        }
    }
    sram_w8(0xa42fu, g_pool.free_head);
    sram_w8(0xa430u, g_pool.free_tail);
    sram_w8(0xa42du, g_pool.count);
    sram_w8(0xa42cu, (uint8_t)g_pool.shortfall);
    sram_w8(0xa42eu, g_pool.desc_head);
}

/* ---- consistency check ---------------------------------------------------- */

static int g_check_bad = 0;

static void check_u8(const char *name, int i, uint32_t base, uint8_t native)
{
    uint8_t mirror = sram_r8(base + (uint32_t)i);
    if (mirror != native)
    {
        if (g_check_bad < 8)
            fprintf(stderr, "native_pool: mismatch %s[%d] native=%02x sram=%02x\n",
                    name, i, (unsigned)native, (unsigned)mirror);
        g_check_bad++;
    }
}

static void check_u16(const char *name, int i, uint32_t base, uint16_t native)
{
    uint16_t mirror = sram_r16(base + 2u * (uint32_t)i);
    if (mirror != native)
    {
        if (g_check_bad < 8)
            fprintf(stderr, "native_pool: mismatch %s[%d] native=%04x sram=%04x\n",
                    name, i, (unsigned)native, (unsigned)mirror);
        g_check_bad++;
    }
}

static void check_true(const char *what, int ok)
{
    if (!ok)
    {
        if (g_check_bad < 8)
            fprintf(stderr, "native_pool: check FAILED: %s\n", what);
        g_check_bad++;
    }
}

static void check_scalar(const char *name, uint32_t base, uint8_t native)
{
    uint8_t mirror = sram_r8(base);
    if (mirror != native)
    {
        if (g_check_bad < 8)
            fprintf(stderr, "native_pool: mismatch %s native=%02x sram=%02x\n",
                    name, (unsigned)native, (unsigned)mirror);
        g_check_bad++;
    }
}

int pool_check_legacy(void)
{
    int i, k;
    int n = (int)g_pool.voices;
    int limit = g_pool.legacy_limit;
    g_check_bad = 0;
    if (limit > kLegacy)
        limit = kLegacy;
    /* 1) every pool-init-mapped array field in the legacy window must equal the
     * SRAM mirror. */
    for (i = 0; i < limit; i++)
    {
        uint8_t *s = slot_at(i);
        uint8_t *d = desc_at(i);
        for (k = 0; k < ARRAY_N(kSlotBytes); k++)
        {
            if (kSlotBytes[k].flags & MAP_INIT)
                check_u8("slot", i, kSlotBytes[k].base, s[kSlotBytes[k].off]);
        }
        for (k = 0; k < ARRAY_N(kSlotWords); k++)
        {
            if (kSlotWords[k].flags & MAP_INIT)
                check_u16("slot.w", i, kSlotWords[k].base, *slot_word_at(i, kSlotWords[k].off));
        }
        for (k = 0; k < ARRAY_N(kDescBytes); k++)
        {
            if (kDescBytes[k].flags & MAP_INIT)
                check_u8("desc", i, kDescBytes[k].base, d[kDescBytes[k].off]);
        }
    }
    /* 2) mirrored scalars. */
    check_scalar("free_head", 0xa42fu, g_pool.free_head);
    check_scalar("free_tail", 0xa430u, g_pool.free_tail);
    check_scalar("desc_head", 0xa42eu, g_pool.desc_head);
    if (n == kLegacy)
    {
        check_scalar("count", 0xa42du, g_pool.count);
        check_true("ext free head idle", g_pool.free_head_ext == 0xff);
        check_true("ext desc head idle", g_pool.desc_head_ext == 0xff);
    }
    else
    {
        /* 3) B5 no-leak: legacy chains walk only inside [0, kLegacy) and stop
         * after exactly kLegacy nodes; extension chains cover [kLegacy, N). */
        int seen;
        check_true("native count != voices", g_pool.count == (uint8_t)n);
        seen = 0;
        i = g_pool.free_head;
        while (i != 0xff && seen <= kVoicesMax)
        {
            if (i >= kLegacy) { check_true("legacy free chain leak", 0); break; }
            seen++;
            i = g_pool.free_next[i];
        }
        check_true("legacy free chain length", seen == kLegacy);
        seen = 0;
        i = g_pool.desc_head;
        while (i != 0xff && seen <= kVoicesMax)
        {
            if (i >= kLegacy) { check_true("legacy desc chain leak", 0); break; }
            seen++;
            i = g_pool.d[i].next;
        }
        check_true("legacy desc chain length", seen == kLegacy);
        seen = 0;
        i = g_pool.free_head_ext;
        while (i != 0xff && seen <= kVoicesMax)
        {
            if (i < kLegacy) { check_true("ext free chain below kLegacy", 0); break; }
            seen++;
            i = g_pool.free_next[i];
        }
        check_true("ext free chain length", seen == n - kLegacy);
        check_true("ext free tail", g_pool.free_tail_ext == (uint8_t)kLegacy);
        seen = 0;
        i = g_pool.desc_head_ext;
        while (i != 0xff && seen <= kVoicesMax)
        {
            if (i < kLegacy) { check_true("ext desc chain below kLegacy", 0); break; }
            seen++;
            i = g_pool.d[i].next;
        }
        check_true("ext desc chain length", seen == n - kLegacy);
    }
    return g_check_bad;
}

void pool_post_reset(void)
{
    int n = pcm_ext_voices;
    if (n < kLegacy)
        n = kLegacy;
    if (n > 255)
        n = 255;
    memset(&g_pool, 0, sizeof(g_pool));
    g_pool.voices = (uint8_t)n;
    g_pool.legacy_limit = (uint8_t)kLegacy;
}

/* ---- pool-init L0 dispatch (spec 3.1/4.4) --------------------------------- */

/* One registered PC executes exactly one H8 instruction. Instruction semantics
 * come from GT's own operand table (src/mcu_opcodes.cpp), the same table the
 * interpreter uses (src/mcu.cpp:1112-1122), so n=28 behavior is bit-exact by
 * construction; the T-state check stays in MK2CPP_Step (mk2cpp.cpp:158-162).
 * The native pool updates below run after the stock instruction and only touch
 * the C++ copy, never SRAM. */
static void pool_l0_stock(void)
{
    uint8_t op = MCU_ReadCodeAdvance();
    MCU_Operand_Table[op](op);
}

/* Loop A (0x40469-0x404c2): per-slot constants. Word arrays are index*2. */
static void pool_fill_loop_a(void)
{
    int i;
    for (i = 0; i < g_pool.voices; i++)
    {
        PoolSlot &s = g_pool.v[i];
        s.ce5c = s.ce78 = s.ce94 = s.d118 = s.d134 = s.ceb0 = s.cf20 = 0xff;
        s.cf3c = 0x3c;
        s.cf90 = s.d0fc = 0xff;
        s.start_prev = s.start_next = 0xff;      /* d0a8 / d0c4 */
        s.pcm_ch = s.ce3f = s.cmd = s.active = 0;  /* ad0e / ce3f / d0e0 / a4b4 */
        s.f_a34c = 0;
        s.cfac = s.d000 = s.d054 = s.a46c = 0;
    }
}

static void pool_l0_a_seed(void)
{
    pool_l0_stock();  /* 0x40462: movi r1 #0x1b */
    pool_fill_loop_a();
}

/* Loop B (0x404dc-0x404ef): chain sentinels. */
static void pool_fill_loop_b(void)
{
    int i;
    for (i = 0; i < g_pool.voices; i++)
    {
        PoolSlot &s = g_pool.v[i];
        s.prev = s.next = 0xff;      /* a3f4 / a410 */
        s.st_a3bc = 0;               /* a3bc */
        g_pool.free_next[i] = 0xff;  /* a3d8 (real chain built by loop C) */
    }
}

static void pool_l0_b_seed(void)
{
    pool_l0_stock();  /* 0x404d9: movi r1 #0x1b */
    pool_fill_loop_b();
}

/* Loop C (0x4050d-0x40533): free list build. The firmware links r1 = N-1..0
 * with a3d8[i] = i-1 (a3d8[0] = 0xff), a42f = N-1 (pop side), a430 = 0 (push
 * side) and a42d = N. The SRAM mirror keeps the stock 28-slot chain; slots
 * >= kLegacy go into a separate native chain (spec 2.1 / B5). */
static void pool_fill_free(void)
{
    int n = (int)g_pool.voices;
    int i;
    for (i = 0; i < kLegacy; i++)
    {
        g_pool.free_next[i] = (i == 0) ? 0xff : (uint8_t)(i - 1); /* a3d8 */
        g_pool.v[i].flags = 0x94;                                 /* a3a0: free */
    }
    if (n > kLegacy)
    {
        for (i = kLegacy; i < n; i++)
        {
            g_pool.free_next[i] = (i == kLegacy) ? 0xff : (uint8_t)(i - 1);
            g_pool.v[i].flags = 0x94;
        }
        g_pool.free_head_ext = (uint8_t)(n - 1);
        g_pool.free_tail_ext = (uint8_t)kLegacy;
    }
    else
    {
        g_pool.free_head_ext = 0xff;
        g_pool.free_tail_ext = 0xff;
    }
    g_pool.free_head = (uint8_t)(kLegacy - 1); /* a42f: mirror-visible head */
    g_pool.free_tail = 0;                      /* a430 */
    g_pool.count = (uint8_t)n;                 /* a42d: total free (native) */
}

static void pool_l0_c_seed(void)
{
    pool_l0_stock();  /* 0x40508: movi r1 #0x1b */
    pool_fill_free();
}

/* Loop D (0x4053b-0x40560): descriptor pool. Legacy window a250[i] = i+1 and
 * a250[27] = 0xff (stock); the [kLegacy, N) descriptors form a separate native
 * chain. a2f8 is not touched here (desc_setup writes it). */
static void pool_fill_desc(void)
{
    int n = (int)g_pool.voices;
    int i;
    for (i = 0; i < kLegacy; i++)
    {
        PoolDesc &d = g_pool.d[i];
        d.next = (uint8_t)((i + 1 < kLegacy) ? (i + 1) : 0xff); /* a250 */
        d.prev = 0xff;                                          /* a26c */
        d.state = 0x94;                                         /* a288 */
        d.vhead = 0xff;                                         /* a2c0 */
        d.vtail = 0xff;                                         /* a2dc */
        d.part_age = 0xff;                                      /* a314 */
    }
    if (n > kLegacy)
    {
        for (i = kLegacy; i < n; i++)
        {
            PoolDesc &d = g_pool.d[i];
            d.next = (uint8_t)((i + 1 < n) ? (i + 1) : 0xff);
            d.prev = 0xff;
            d.state = 0x94;
            d.vhead = 0xff;
            d.vtail = 0xff;
            d.part_age = 0xff;
        }
        g_pool.desc_head_ext = (uint8_t)kLegacy;
    }
    else
    {
        g_pool.desc_head_ext = 0xff;
    }
    g_pool.desc_head = 0; /* a42e */
}

/* One-shot native/SRAM consistency check after loops A-D; mismatches always
 * print, success is verbose-only (set MK2CPP_POOL_VERBOSE=1). */
static void pool_selfcheck(void)
{
    int bad = pool_check_legacy();
    if (bad)
        fprintf(stderr, "native_pool: pool-init self-check FAILED (%d mismatch(es))\n", bad);
    else if (getenv("MK2CPP_POOL_VERBOSE"))
        fprintf(stderr, "native_pool: pool-init self-check OK (voices=%u)\n",
                (unsigned)g_pool.voices);
}

/* ---- native pool operations (release/free/link/pop) ----------------------- */
/* Faithful models of dasm_full.txt:
 *   release A 0x1823-0x187d, release B 0x187e-0x18cd, free 0x19c4-0x1a23,
 *   pool_pop 0x19ad-0x19c3, link 0x194c-0x19ac, H1 0x1b44-0x1b8f,
 *   H2 0x1bad-0x1bfd, unlink 0x1b23-0x1b43.
 * The part-level arrays (a200/a210/a220/a230) and a34c stay SRAM-only in
 * slice-2 (spec 2.3), so the native models cover the pool/voice/descriptor
 * fields only. >= 0x80 is the 0xff sentinel convention used throughout. */

static void nat_h1(uint8_t slot, uint8_t desc)
{
    uint8_t r3 = g_pool.v[slot].prev;                    /* a3f4 */
    if (r3 < 0x80)
    {
        g_pool.d[desc].vhead = r3;                       /* a2c0 */
        g_pool.v[r3].next = 0xff;                        /* a410 */
        g_pool.v[r3].start_prev = 0xff;                  /* d0a8 */
        g_pool.v[slot].prev = 0xff;
        g_pool.v[slot].start_next = 0xff;                /* d0c4 */
        return;
    }
    r3 = g_pool.v[slot].next;                            /* a410 */
    if (r3 < 0x80)
    {
        g_pool.v[slot].next = 0xff;
        g_pool.v[slot].start_prev = 0xff;
        g_pool.v[r3].prev = 0xff;
        g_pool.v[r3].start_next = 0xff;
        g_pool.d[desc].vtail = r3;                       /* a2dc */
    }
    else
    {
        g_pool.d[desc].vhead = 0xff;
        g_pool.d[desc].vtail = 0xff;
    }
}

static void nat_h2(uint8_t desc)
{
    if (g_pool.d[desc].vhead < 0x80)
        return;
    /* unlink(desc) pool-level: a250/a26c rewiring; the part-level heads
     * a220/a230 and counters a200/a210 are SRAM-only (spec 2.3). */
    {
        uint8_t next = g_pool.d[desc].next;              /* a250 */
        uint8_t prev = g_pool.d[desc].prev;              /* a26c */
        if (prev < 0x80)
            g_pool.d[prev].next = next;
        if (next < 0x80)
            g_pool.d[next].prev = prev;
    }
    g_pool.d[desc].next = g_pool.desc_head;              /* a42e push */
    g_pool.desc_head = desc;
    g_pool.d[desc].state = 0x94;                         /* a288 */
}

/* Native allocation cap: slots >= kLegacy are only handed out once the ROM
 * consumers of a voice index are native (blocker B5). kLegacy = legacy only. */
static int g_alloc_limit = kLegacy;

/* release A / free push at the free-list tail (a430). Slots >= kLegacy use the
 * separate native extension chain and never touch the SRAM mirror. */
static void nat_push_tail_ext(uint8_t slot)
{
    if (g_pool.free_tail_ext < 0x80)
        g_pool.free_next[g_pool.free_tail_ext] = slot;
    else
        g_pool.free_head_ext = slot;
    g_pool.free_next[slot] = 0xff;
    g_pool.free_tail_ext = slot;
    g_pool.v[slot].flags = 0x94;                         /* a3a0 */
    g_pool.count++;
}

static void nat_push_head_ext(uint8_t slot)
{
    if (g_pool.free_head_ext >= 0x80)
        g_pool.free_tail_ext = slot;
    g_pool.free_next[slot] = g_pool.free_head_ext;
    g_pool.free_head_ext = slot;
    g_pool.v[slot].flags = 0x94;
    g_pool.count++;
}

static void nat_push_tail(uint8_t slot)
{
    if (slot >= kLegacy)
    {
        nat_push_tail_ext(slot);
        return;
    }
    if (g_pool.free_tail < 0x80)
        g_pool.free_next[g_pool.free_tail] = slot;       /* a3d8 */
    else
        g_pool.free_head = slot;                         /* a42f */
    g_pool.free_next[slot] = 0xff;
    g_pool.free_tail = slot;
    g_pool.v[slot].flags = 0x94;                         /* a3a0 */
    g_pool.count++;                                      /* a42d */
}

/* release B pushes at the free-list head. */
static void nat_push_head(uint8_t slot)
{
    if (slot >= kLegacy)
    {
        nat_push_head_ext(slot);
        return;
    }
    if (g_pool.free_head >= 0x80)
        g_pool.free_tail = slot;
    g_pool.free_next[slot] = g_pool.free_head;
    g_pool.free_head = slot;
    g_pool.v[slot].flags = 0x94;
    g_pool.count++;
}

/* Legacy chain first (mirrors a42f/a430/a42d); the extension chain is only
 * reachable when g_alloc_limit > kLegacy (dormant, blocker B5). */
static uint8_t nat_pop(void)
{
    uint8_t slot = g_pool.free_head;
    if (slot < 0x80)
    {
        g_pool.free_head = g_pool.free_next[slot];
        if (g_pool.free_head >= 0x80)
            g_pool.free_tail = g_pool.free_head;
        g_pool.count--;
        return slot;
    }
    if (g_alloc_limit > kLegacy)
    {
        slot = g_pool.free_head_ext;
        if (slot < 0x80)
        {
            g_pool.free_head_ext = g_pool.free_next[slot];
            if (g_pool.free_head_ext >= 0x80)
                g_pool.free_tail_ext = g_pool.free_head_ext;
            g_pool.count--;
            return slot;
        }
    }
    return 0xff;
}

static void nat_release(uint8_t slot, uint8_t desc, int push_head)
{
    g_pool.v[slot].cmd = 4;                              /* d0e0 */
    g_pool.v[slot].pcm_ch = 0;                           /* ad0e */
    g_pool.v[slot].st_a3bc = 0;                          /* a3bc */
    g_pool.v[slot].active = 0;                           /* a4b4 */
    nat_h1(slot, desc);
    if (push_head)
        nat_push_head(slot);
    else
        nat_push_tail(slot);
    nat_h2(desc);
    g_pool.shortfall--;                                  /* a42c */
    if (g_pool.shortfall < 0)
        g_pool.shortfall = 0;
}

static void nat_free(uint8_t slot)
{
    uint8_t desc;
    if (g_pool.v[slot].flags & 0x80)                     /* a3a0 bit7: already free */
        return;
    if (g_pool.v[slot].pcm_ch == 0xff)
        return;
    desc = g_pool.v[slot].desc;                          /* a384 */
    g_pool.v[slot].st_a3bc = 0;
    g_pool.v[slot].active = 0;
    /* a34c[desc] bits 1/0 are SRAM-only in slice-2 (spec 2.3) */
    nat_h1(slot, desc);
    nat_push_tail(slot);
    nat_h2(desc);
}

static void nat_link(uint8_t slot, uint8_t desc, uint8_t part)
{
    uint8_t r0 = g_pool.d[desc].vtail;                   /* a2dc */
    if (r0 >= 0x80)
    {
        g_pool.d[desc].vhead = slot;
        r0 = g_pool.v[slot].start_prev;                  /* d0a8 */
        if (r0 < 0x80)
            g_pool.v[r0].start_next = 0xff;              /* d0c4 */
        r0 = g_pool.v[slot].start_next;                  /* d0c4 */
        if (r0 < 0x80)
            g_pool.v[r0].start_prev = 0xff;              /* d0a8 */
        g_pool.v[slot].next = 0xff;
        g_pool.v[slot].start_prev = 0xff;
    }
    else
    {
        g_pool.d[desc].vhead = r0;
        g_pool.v[r0].prev = slot;
        g_pool.v[r0].start_next = slot;
        g_pool.v[slot].next = r0;
        g_pool.v[slot].start_prev = r0;
        g_pool.v[r0].next = 0xff;
        g_pool.v[r0].start_prev = 0xff;
    }
    g_pool.d[desc].vtail = slot;
    g_pool.v[slot].prev = 0xff;
    g_pool.v[slot].start_next = 0xff;
    g_pool.v[slot].part = part;                          /* a368 */
    g_pool.v[slot].desc = desc;                          /* a384 */
}

/* ---- op co-simulation (native model vs SRAM, every hooked invocation) ----- */

enum { OP_POOL_POP = 1, OP_LINK, OP_REL_A, OP_REL_B, OP_FREE };

static struct {
    int kind;
    uint8_t slot, desc, part;
    int active;
} g_op;

static int g_op_depth = 0;
static int g_op_nested = 0;
static int g_op_bad = 0;
static int g_ext_warned = 0;

static void ext_warn(const char *what)
{
    if (!g_ext_warned)
    {
        g_ext_warned = 1;
        fprintf(stderr, "native_pool: %s: index >= %d (native allocation stays capped at the legacy window until the consumers are native, blocker B5)\n",
                what, kLegacy);
    }
}

static void op_mismatch(const char *name, int idx, unsigned native, unsigned mirror)
{
    if (g_op_bad < 8)
        fprintf(stderr, "native_pool: op mismatch %s[%d] native=%02x sram=%02x\n",
                name, idx, native, mirror);
    g_op_bad++;
}

static void op_cmp8(const char *name, int idx, uint8_t native, uint32_t base)
{
    uint8_t mirror = sram_r8(base + (uint32_t)idx);
    if (mirror != native)
        op_mismatch(name, idx, (unsigned)native, (unsigned)mirror);
}

static void pool_check_ops(void)
{
    int i;
    for (i = 0; i < kLegacy; i++)
    {
        op_cmp8("flags", i, g_pool.v[i].flags, 0xa3a0u);
        op_cmp8("pcm_ch", i, g_pool.v[i].pcm_ch, 0xad0eu);
        op_cmp8("st_a3bc", i, g_pool.v[i].st_a3bc, 0xa3bcu);
        op_cmp8("active", i, g_pool.v[i].active, 0xa4b4u);
        op_cmp8("cmd", i, g_pool.v[i].cmd, 0xd0e0u);
        op_cmp8("part", i, g_pool.v[i].part, 0xa368u);
        op_cmp8("desc", i, g_pool.v[i].desc, 0xa384u);
        op_cmp8("prev", i, g_pool.v[i].prev, 0xa3f4u);
        op_cmp8("next", i, g_pool.v[i].next, 0xa410u);
        op_cmp8("start_prev", i, g_pool.v[i].start_prev, 0xd0a8u);
        op_cmp8("start_next", i, g_pool.v[i].start_next, 0xd0c4u);
        op_cmp8("free_next", i, g_pool.free_next[i], 0xa3d8u);
        op_cmp8("d.next", i, g_pool.d[i].next, 0xa250u);
        op_cmp8("d.prev", i, g_pool.d[i].prev, 0xa26cu);
        op_cmp8("d.state", i, g_pool.d[i].state, 0xa288u);
        op_cmp8("d.vhead", i, g_pool.d[i].vhead, 0xa2c0u);
        op_cmp8("d.vtail", i, g_pool.d[i].vtail, 0xa2dcu);
    }
    op_cmp8("free_head", 0, g_pool.free_head, 0xa42fu);
    op_cmp8("free_tail", 0, g_pool.free_tail, 0xa430u);
    if (g_pool.voices == kLegacy)
        op_cmp8("count", 0, g_pool.count, 0xa42du);
    op_cmp8("shortfall", 0, (uint8_t)g_pool.shortfall, 0xa42cu);
    op_cmp8("desc_head", 0, g_pool.desc_head, 0xa42eu);
}

static void op_begin(int kind, uint8_t slot, uint8_t desc, uint8_t part, int active)
{
    if (g_op_depth > 0)
    {
        g_op_nested = 1;
        g_op_depth++;
        return;
    }
    g_op.kind = kind;
    g_op.slot = slot;
    g_op.desc = desc;
    g_op.part = part;
    g_op.active = active;
    g_op_nested = 0;
    g_op_depth++;
}

static void op_end(void)
{
    if (g_op_depth == 0)
        return; /* entry guard took the native-only path: nothing to unfold */
    g_op_depth--;
    if (g_op_depth > 0)
        return;
    if (!g_op_nested && g_op.active)
    {
        switch (g_op.kind)
        {
        case OP_POOL_POP:
        {
            uint8_t ns = nat_pop();
            if ((uint8_t)mcu.r[1] != ns)
                op_mismatch("pool_pop.slot", 0, (unsigned)ns, (unsigned)(uint8_t)mcu.r[1]);
            break;
        }
        case OP_LINK:
            nat_link(g_op.slot, g_op.desc, g_op.part);
            break;
        case OP_REL_A:
            nat_release(g_op.slot, g_op.desc, 0);
            break;
        case OP_REL_B:
            nat_release(g_op.slot, g_op.desc, 1);
            break;
        case OP_FREE:
            nat_free(g_op.slot);
            break;
        }
        pool_check_ops();
    }
    g_op_nested = 0;
    g_op.active = 0;
    /* Re-import the SRAM truth: until the consumers are native the page-0
     * arrays stay authoritative (spec 2.1), and this also repairs a nested or
     * mismatching native op. */
    pool_import_legacy();
}

/* ---- L0 entry/exit hooks -------------------------------------------------- */

static void pool_l0_pop(void)
{
    pool_import_legacy();
    if (g_pool.free_head >= 0x80)
    {
        /* Stock would read a3d8[0xff] and corrupt a42f; the native extension
         * chain is not handed to un-replaced consumers yet (B5). */
        ext_warn("pool_pop with empty free list");
        mcu.r[1] = 0xff;
        mcu.pc = 0x19c3u;
        return;
    }
    op_begin(OP_POOL_POP, 0, 0, 0, 1);
    pool_l0_stock();
}

static void pool_l0_link(void)
{
    uint8_t slot = (uint8_t)mcu.r[1];
    uint8_t desc = (uint8_t)mcu.r[2];
    uint8_t part = (uint8_t)mcu.r[3];
    if (slot >= kLegacy || desc >= kLegacy)
    {
        if (slot < kVoicesMax && desc < kVoicesMax)
        {
            pool_import_legacy();
            nat_link(slot, desc, part);
            mcu.pc = 0x19acu;
        }
        else
        {
            ext_warn("link with sentinel index");
            mcu.pc = 0x19acu;
        }
        return;
    }
    pool_import_legacy();
    op_begin(OP_LINK, slot, desc, part, 1);
    pool_l0_stock();
}

static void pool_l0_rel_a(void)
{
    uint8_t slot = (uint8_t)mcu.r[1];
    uint8_t desc = (uint8_t)mcu.r[2];
    uint8_t part = (uint8_t)mcu.r[3];
    if (slot >= kLegacy && slot < kVoicesMax)
    {
        pool_import_legacy();
        nat_release(slot, desc, 0);
        mcu.pc = 0x187du;
        return;
    }
    pool_import_legacy();
    op_begin(OP_REL_A, slot, desc, part, slot < kLegacy && desc < kLegacy);
    pool_l0_stock();
}

static void pool_l0_rel_b(void)
{
    uint8_t slot = (uint8_t)mcu.r[1];
    uint8_t desc = (uint8_t)mcu.r[2];
    uint8_t part = (uint8_t)mcu.r[3];
    if (slot >= kLegacy && slot < kVoicesMax)
    {
        pool_import_legacy();
        nat_release(slot, desc, 1);
        mcu.pc = 0x18cdu;
        return;
    }
    pool_import_legacy();
    pool_l0_stock();                 /* 0x187e: EXTU r1 */
    slot = (uint8_t)mcu.r[1];
    op_begin(OP_REL_B, slot, desc, part, slot < kLegacy && desc < kLegacy);
}

static void pool_l0_free(void)
{
    uint8_t slot = (uint8_t)mcu.r[1];
    if (slot >= kLegacy && slot < kVoicesMax)
    {
        pool_import_legacy();
        nat_free(slot);
        mcu.pc = 0x1a23u;
        return;
    }
    pool_import_legacy();
    {
        int active = (slot < kLegacy) &&
                     !(g_pool.v[slot].flags & 0x80) &&
                     (g_pool.v[slot].pcm_ch != 0xff);
        op_begin(OP_FREE, slot, g_pool.v[slot].desc, g_pool.v[slot].part, active);
    }
    pool_l0_stock();
}

static void pool_l0_op_end(void)
{
    pool_l0_stock();
    op_end();
}

/* ---- native extension self-test (on a copy; verbose/one-shot) -------------- */

static int pool_ext_selftest(void)
{
    Pool saved = g_pool;
    int saved_limit = g_alloc_limit;
    int bad = 0;
    int i;
    memset(&g_pool, 0, sizeof(g_pool));
    g_pool.voices = 64;
    g_pool.legacy_limit = kLegacy;
    pool_fill_loop_a();
    pool_fill_loop_b();
    pool_fill_free();
    pool_fill_desc();
    g_alloc_limit = 64;
    /* legacy 27..0 first, then extension 63..28 */
    for (i = 0; i < kLegacy; i++)
    {
        if (nat_pop() != (uint8_t)(kLegacy - 1 - i))
            bad++;
    }
    if (g_pool.free_head != 0xff || g_pool.free_tail != 0xff)
        bad++;
    for (i = kLegacy; i < 64; i++)
    {
        if (nat_pop() != (uint8_t)(63 - (i - kLegacy)))
            bad++;
    }
    if (nat_pop() != 0xff)
        bad++;
    if (g_pool.count != 0)
        bad++;
    /* push the extension slots back at the tail: 28..63 */
    for (i = kLegacy; i < 64; i++)
        nat_push_tail((uint8_t)i);
    if (g_pool.count != 36)
        bad++;
    if (g_pool.free_head != 0xff || g_pool.free_tail != 0xff)
        bad++;
    if (g_pool.free_head_ext != kLegacy || g_pool.free_tail_ext != 63)
        bad++;
    /* link + release one extension voice/descriptor */
    nat_link(28, 28, 0);
    if (g_pool.v[28].desc != 28 || g_pool.v[28].part != 0)
        bad++;
    if (g_pool.d[28].vtail != 28 || g_pool.d[28].vhead != 28)
        bad++;
    nat_release(28, 28, 0);
    if (g_pool.v[28].flags != 0x94)
        bad++;
    if (g_pool.d[28].vhead != 0xff || g_pool.d[28].vtail != 0xff)
        bad++;
    if (g_pool.free_tail_ext != 28 || g_pool.count != 37)
        bad++;
    /* release B push-head path */
    nat_link(29, 29, 0);
    nat_release(29, 29, 1);
    if (g_pool.free_head_ext != 29 || g_pool.count != 38)
        bad++;
    /* free path: assigned extension voice linked to a descriptor */
    nat_link(30, 30, 0);
    g_pool.v[30].pcm_ch = 5;   /* ad0e assigned */
    g_pool.v[30].flags = 0x00; /* allocated */
    nat_free(30);
    if (g_pool.v[30].flags != 0x94)
        bad++;
    if (g_pool.d[30].vhead != 0xff)
        bad++;
    if (g_pool.free_tail_ext != 30 || g_pool.count != 39)
        bad++;
    g_alloc_limit = saved_limit;
    g_pool = saved;
    return bad;
}

static void pool_l0_d_seed(void)
{
    pool_l0_stock();  /* 0x40563: CLR r2 (loops A-D done) */
    pool_fill_desc();
    pool_selfcheck();
    {
        int bad = pool_ext_selftest();
        if (bad)
            fprintf(stderr, "native_pool: ext selftest FAILED (%d check(s))\n", bad);
        else if (getenv("MK2CPP_POOL_VERBOSE"))
            fprintf(stderr, "native_pool: ext selftest OK\n");
    }
}

} /* namespace mk2c */

/* ---- hand integration hooks ----------------------------------------------- */

void MK2CPP_PoolPostReset(void)
{
    mk2c::pool_post_reset();
}

/* ---- L0 registration ------------------------------------------------------ */

static void register_pcs(const uint32_t *pcs, int count, mk2cpp_fn fn)
{
    int i;
    for (i = 0; i < count; i++)
        MK2CPP_HandRegister(pcs[i], fn);
}

/* Loop A body PCs (0x40469-0x404c2); the seed 0x40462 is registered separately
 * because it also fills the native copy. Source: dasm_full.txt 0x40469-0x404c2. */
static const uint32_t kPcsLoopA[] = {
    0x00040465u, 0x00040467u, 0x00040469u, 0x0004046du, 0x00040471u,
    0x00040475u, 0x00040479u, 0x0004047du, 0x00040481u, 0x00040485u,
    0x0004048au, 0x0004048eu, 0x00040492u, 0x00040496u, 0x0004049au,
    0x0004049eu, 0x000404a2u, 0x000404a6u, 0x000404aau, 0x000404aeu,
    0x000404b0u, 0x000404b4u, 0x000404b8u, 0x000404bcu, 0x000404c0u,
    0x000404c2u,
};

/* Loop B body PCs (0x404dc-0x404ef); seed 0x404d9 fills the native copy. */
static const uint32_t kPcsLoopB[] = {
    0x000404dcu, 0x000404e1u, 0x000404e6u, 0x000404eau, 0x000404efu,
};

/* Loop C body PCs (0x4050b-0x40536); seed 0x40508 fills the native free list. */
static const uint32_t kPcsLoopC[] = {
    0x0004050bu, 0x0004050du, 0x00040511u, 0x00040513u, 0x00040517u,
    0x0004051bu, 0x0004051du, 0x00040521u, 0x00040526u, 0x0004052au,
    0x0004052fu, 0x00040533u, 0x00040536u,
};

/* Loop D body PCs (0x4053b-0x40560); seed 0x40563 fills the descriptor copy
 * and runs the native/SRAM consistency check. */
static const uint32_t kPcsLoopD[] = {
    0x0004053bu, 0x0004053fu, 0x00040543u, 0x00040547u, 0x0004054cu,
    0x00040551u, 0x00040556u, 0x0004055bu, 0x00040560u,
};

/* Loop E (0x40565-0x40581, per-part 16 entries) and the part-table helper
 * (0x40588-0x4062a, called by bsr16 at 0x40583). These touch only per-part
 * arrays (a010..a240 etc.), which spec 2.3 keeps in SRAM, so they are pure L0
 * passthrough with no native action. The 0x405a6/0x405a9/0x405ae arm is not in
 * dasm_full (unexecuted at boot) and was recovered from the ROM bytes
 * (build/rom2.bin, aligned 3/5/4-byte instructions). */
static const uint32_t kPcsLoopE[] = {
    0x00040565u, 0x00040569u, 0x0004056eu, 0x00040573u, 0x00040577u,
    0x0004057du, 0x0004057fu, 0x00040581u, 0x00040583u, 0x00040586u,
};

static const uint32_t kPcsHelper[] = {
    0x00040588u, 0x0004058bu, 0x0004058du, 0x00040591u, 0x00040595u,
    0x00040597u, 0x0004059au, 0x0004059cu, 0x0004059fu, 0x000405a4u,
    0x000405a6u, 0x000405a9u, 0x000405aeu, 0x000405b2u, 0x000405b6u,
    0x000405bau, 0x000405bcu, 0x000405c0u, 0x000405c4u, 0x000405c6u,
    0x000405cau, 0x000405ccu, 0x000405d0u, 0x000405d4u, 0x000405d6u,
    0x000405d9u, 0x000405dcu, 0x000405deu, 0x000405e2u, 0x000405e7u,
    0x000405e9u, 0x000405efu, 0x000405f1u, 0x000405f4u, 0x000405f7u,
    0x000405fau, 0x000405feu, 0x00040603u, 0x00040608u, 0x0004060au,
    0x0004060cu, 0x0004060eu, 0x00040610u, 0x00040612u, 0x00040616u,
    0x00040619u, 0x0004061bu, 0x0004061eu, 0x00040621u, 0x00040624u,
    0x00040627u, 0x0004062au,
};

void MK2CPP_PoolFillTables(void)
{
    /* Wave 2b: pool-init L0 override PCs (spec 3.1). The native copy is filled
     * for [0, voices) while the stock instruction sequence still initializes
     * the SRAM mirror for [0, kLegacy). Slots >= kLegacy stay native-only and
     * are never handed to un-replaced ROM consumers (legacy_limit, spec 2.1). */
    register_pcs(kPcsLoopA, (int)(sizeof(kPcsLoopA) / sizeof(kPcsLoopA[0])), &mk2c::pool_l0_stock);
    register_pcs(kPcsLoopB, (int)(sizeof(kPcsLoopB) / sizeof(kPcsLoopB[0])), &mk2c::pool_l0_stock);
    register_pcs(kPcsLoopC, (int)(sizeof(kPcsLoopC) / sizeof(kPcsLoopC[0])), &mk2c::pool_l0_stock);
    register_pcs(kPcsLoopD, (int)(sizeof(kPcsLoopD) / sizeof(kPcsLoopD[0])), &mk2c::pool_l0_stock);
    register_pcs(kPcsLoopE, (int)(sizeof(kPcsLoopE) / sizeof(kPcsLoopE[0])), &mk2c::pool_l0_stock);
    register_pcs(kPcsHelper, (int)(sizeof(kPcsHelper) / sizeof(kPcsHelper[0])), &mk2c::pool_l0_stock);
    MK2CPP_HandRegister(0x00040462u, &mk2c::pool_l0_a_seed);
    MK2CPP_HandRegister(0x000404d9u, &mk2c::pool_l0_b_seed);
    MK2CPP_HandRegister(0x00040508u, &mk2c::pool_l0_c_seed);
    MK2CPP_HandRegister(0x00040563u, &mk2c::pool_l0_d_seed);

    /* Wave 2 release/free allocator slice: entry + rts hooks for release A/B,
     * free, pool_pop and link (spec 3.2-3.4). The stock instructions still run;
     * the native models are applied at the rts and co-simulated against the SRAM
     * mirror (mismatches print as "native_pool: op mismatch"). */
    MK2CPP_HandRegister(0x00001823u, &mk2c::pool_l0_rel_a);
    MK2CPP_HandRegister(0x0000187du, &mk2c::pool_l0_op_end);   /* release A rts */
    MK2CPP_HandRegister(0x0000187eu, &mk2c::pool_l0_rel_b);
    MK2CPP_HandRegister(0x000018cdu, &mk2c::pool_l0_op_end);   /* release B rts */
    MK2CPP_HandRegister(0x0000194cu, &mk2c::pool_l0_link);
    MK2CPP_HandRegister(0x000019acu, &mk2c::pool_l0_op_end);   /* link rts */
    MK2CPP_HandRegister(0x000019adu, &mk2c::pool_l0_pop);
    MK2CPP_HandRegister(0x000019c3u, &mk2c::pool_l0_op_end);   /* pool_pop rts */
    MK2CPP_HandRegister(0x000019c4u, &mk2c::pool_l0_free);
    MK2CPP_HandRegister(0x00001a23u, &mk2c::pool_l0_op_end);   /* free rts */
}

