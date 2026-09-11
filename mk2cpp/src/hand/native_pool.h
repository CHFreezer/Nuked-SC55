/*
 * HAND voice/native_pool -- native voice pool (M4 slice-2).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 1
 *
 * Data model and SRAM mirror for the native voice pool, per
 * mk2cpp/out/m4/11_slice2_pool_spec.md section 2. Field names reuse the H8
 * SRAM names (a368, a3d8, ...) so the mapping is checkable against
 * tools/docs/voice_memory_map.md; unproven semantics keep the literal name with
 * a TODO. Two corrections against the older docs are applied here (spec 1.6):
 *   - a3a0 == 0x94 means FREE (bit7 set); note_fill clears it on allocation.
 *   - a368 is slot -> part; a384 is slot -> descriptor index.
 *
 * Dual-track policy (spec 2.1): during slice-2 the page-0 SRAM arrays remain
 * the interop truth for slots/descs [0, kLegacy); the native copy is filled
 * from the same constants and is byte-checked against SRAM (pool_check_legacy).
 * Slots >= kLegacy live in the native copy only and must not be handed to
 * un-replaced ROM consumers (legacy_limit).
 *
 * Used by:
 *   - MK2CPP_HandPostReset() (via pcm_enable.cpp)  -> pool_post_reset()
 *   - MK2CPP_HandFillTables() (via pcm_enable.cpp) -> MK2CPP_PoolFillTables()
 */
#pragma once
#include <stdint.h>

namespace mk2c {

/* Capacity: 256 slots; slot 255 is reserved as the 0xff sentinel (N <= 255). */
constexpr int kVoicesMax = 256;
/* Page-0 SRAM mirror window visible to the stock ROM (28 entries). */
constexpr int kLegacy = 28;

/* One 0x12a AoS voice struct, SoA view (spec 2.2). */
struct PoolSlot {
    /* identity / pool / chains */
    uint8_t flags;                    /* a3a0: 0x94 = free (bit7=1) */
    uint8_t pcm_ch;                   /* ad0e: 0xff = none */
    uint8_t st_a3bc;                  /* a3bc */
    uint8_t active;                   /* a4b4 */
    uint8_t cmd;                      /* d0e0: 0 idle / 2 play / 4 stop */
    uint8_t irq_pend;                 /* d15c */
    uint8_t part;                     /* a368 (slot -> part; corrected) */
    uint8_t desc;                     /* a384 (slot -> descriptor) */
    uint8_t prev, next;               /* a3f4 / a410 */
    uint8_t start_prev, start_next;   /* d0a8 / d0c4 */
    /* pool_init loop A payload (semantics mostly unproven: keep literal names) */
    uint8_t ce5c, ce78, ce94, d118, d134, ceb0, cf20, cf3c, cf90, d0fc, ce3f;
    uint16_t cfac, d000, d054, a46c;
    uint8_t f_a34c;                   /* a34c: slot/desc mixed index space TODO */
};

/* One note descriptor (spec 2.2); a250 chain head is Pool::desc_head (a42e). */
struct PoolDesc {
    uint8_t next, prev;               /* a250 / a26c */
    uint8_t state;                    /* a288 */
    uint8_t vhead, vtail;             /* a2c0 / a2dc */
    uint8_t part_age;                 /* a314 */
    uint8_t f2f8;                     /* a2f8 (desc_setup writes a4a5) TODO */
};

struct Pool {
    uint8_t voices;                   /* N from pcm_ext_voices (28..255) */
    uint8_t legacy_limit;             /* kLegacy: ROM-visible mirror cap */
    uint8_t free_next[kVoicesMax];    /* a3d8 */
    uint8_t free_head;                /* a42f (legacy mirror head, 0..27) */
    uint8_t free_tail;                /* a430 (push side) */
    uint8_t count;                    /* a42d (native free count = N) */
    int8_t  shortfall;                /* a42c */
    uint8_t desc_head;                /* a42e (legacy mirror head) */
    /* Extension chains over [kLegacy, N), held natively only so the SRAM
     * mirror can never hand a slot >= kLegacy to an un-replaced ROM consumer
     * (spec 2.1 / blocker B5). At N == kLegacy they are 0xff (unused). */
    uint8_t free_head_ext;            /* pop side of the extension free chain */
    uint8_t free_tail_ext;            /* push side of the extension free chain */
    uint8_t desc_head_ext;            /* head of the extension descriptor chain */
    PoolSlot v[kVoicesMax];
    PoolDesc d[kVoicesMax];
};

extern Pool g_pool;

/* Capture the post-reset configuration (N from pcm_ext_voices) and reset the
 * native copy. Called via MK2CPP_HandPostReset() after GT's reset chain. */
void pool_post_reset(void);

/* Non-zero once MK2CPP_PostReset captured a valid voice count. */
int pool_configured(void);

/* Import/commit the SRAM legacy window into/from the native copy.
 * Import is used at hand entry for fields an un-replaced ROM writer may have
 * touched; commit at block exit for fields with un-replaced ROM readers.
 * Both operate on slots/descs [0, kLegacy) only; the native extension is never
 * mirrored (legacy_limit safety, spec 2.1). */
void pool_import_legacy(void);
void pool_commit_legacy(void);

/* Compare the native copy against the SRAM mirror over the fields pool-init is
 * responsible for; returns the number of mismatches and prints the first few.
 * Read-only; a development/acceptance aid (no behavior change). */
int pool_check_legacy(void);

} /* namespace mk2c */

/* Registration/constructor hooks (see file header). */
void MK2CPP_PoolFillTables(void);
void MK2CPP_PoolPostReset(void);
