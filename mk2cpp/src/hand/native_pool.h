/*
 * HAND voice/native_pool -- whole-routine voice pool initialization.
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 3
 *
 * native_pool.cpp replaces the pool-init routine 0x40462-0x4062a as four
 * whole-routine L1 blocks (0x40462 / 0x404b4 / 0x40621 / 0x40624). The data
 * model lives in page-0 SRAM and is accessed with MCU_Read/MCU_Write, so the
 * untranslated ROM (and the state hashdump) observes exactly the stock bytes.
 * The old native Pool mirror, the per-PC L0 registration and the self-check
 * are gone; the alloc/free routines have their own hand module
 * (native_allocfree.cpp) and are aggregated in MK2CPP_PoolFillTables below.
 *
 * Header keeps only the hooks pcm_enable.cpp consumes:
 *   - MK2CPP_HandPostReset() (via pcm_enable.cpp) -> pool_post_reset()
 *   - MK2CPP_HandFillTables() (via pcm_enable.cpp) -> MK2CPP_PoolFillTables()
 */
#pragma once
#include <stdint.h>

namespace mk2c {

/* Post-reset hook (MK2CPP_HandPostReset). Pool-init is translated from ROM
 * and works directly on SRAM, so there is no native pool copy to reset. */
void pool_post_reset(void);

} /* namespace mk2c */

/* Registration/constructor hooks (see file header). */
void MK2CPP_PoolFillTables(void);
void MK2CPP_PoolPostReset(void);
/* Whole-routine alloc/free registration (native_allocfree.cpp), called from
 * MK2CPP_PoolFillTables so MK2CPP_HandFillTables remains the single entry. */
void MK2CPP_AllocFreeFillTables(void);
