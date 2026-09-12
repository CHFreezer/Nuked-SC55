/*
 * mk2cpp.cpp -- GT integration glue for the translated core.
 *
 * No behavior change unless -mk2cpp is given AND translated code was linked
 * (see MK2CPP_GEN_DIR in CMakeLists.txt). Generated code registers one
 * function per translated PC via MK2CPP_RegisterFn().
 */
#include "mk2cpp.h"
#include "mcu.h"
#include "submcu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Defined in src/mcu.cpp (GT's interpreter step). */
extern void MCU_ReadInstruction(void);
/* Defined in src/submcu.cpp (GT's SM interpreter step: fetch+dispatch). */
extern void SM_ExecuteOneInstruction(void);

int mk2cpp_enabled = 0;
int mk2cpp_mixed = 1;

static mk2cpp_fn mk2cpp_tab_cp0[0x10000];
static mk2cpp_fn mk2cpp_tab_cp4[0x10000];
static uint32_t mk2cpp_fallbacks = 0;
static uint32_t mk2cpp_translated = 0;

int mk2cpp_hand_enabled = 1;

/* Capacity rationale (M4 out/11_slice2_pool_spec.md 4.4): L0 override tables are
 * per-PC, not per-routine. The voice closure (mk2_polyphony_256 6.5 R11) needs
 * thousands of L0 entries once every per-voice PC is owned; the M4 translation
 * used ~3k entries by 2026-09-12 and the remaining fragments still add more.
 * 8192 entries keep a single sparse table without resizing; the entry is 32
 * bytes on x64, so the static array is ~256 KiB BSS. Registration still fails
 * fast on overflow/duplicates. */
#define MK2CPP_HAND_MAX 8192u

typedef struct {
    uint32_t  flat; /* (cp << 16) | pc */
    mk2cpp_fn fn;   /* L0: exactly one H8 instruction per call */
    mk2cpp_hand_routine_fn rfn; /* L1: whole routine, returns instruction count */
    uint32_t  hits; /* per-entry diagnostic counter */
    uint8_t   is_routine;
} mk2cpp_hand_ent;

/* Sparse table sorted by flat; hand registrations are few (<= MK2CPP_HAND_MAX),
 * so a binary search is cheaper than mirroring the 2x64K gen tables.
 * L0 and L1 entries share the table, the lookup and the HandCount/HandHitCount
 * accounting (out/09 2.3/4.3). */
static mk2cpp_hand_ent mk2cpp_hand[MK2CPP_HAND_MAX];
static uint32_t mk2cpp_hand_n = 0;
static uint32_t mk2cpp_hand_routines = 0;
static uint32_t mk2cpp_hand_hits = 0;
/* L1 protocol violations (returned n == 0): counted, warned once. */
static uint32_t mk2cpp_hand_bad_n = 0;
static int mk2cpp_hand_bad_n_warned = 0;

/* SM dispatch table indexed directly by sm.pc (16-bit). The firmware executes
 * in [0xf000,0xffff]; the table covers the full 16-bit space so any pc can be
 * looked up safely. */
static mk2cpp_sm_fn mk2cpp_sm_tab[0x10000];
static uint32_t mk2cpp_sm_fallbacks = 0;
static uint32_t mk2cpp_sm_translated = 0;

void MK2CPP_RegisterFn(uint32_t flat, mk2cpp_fn fn)
{
    uint32_t cp = (flat >> 16) & 0xffu;
    uint16_t off = (uint16_t)flat;
    if (cp == 0)
        mk2cpp_tab_cp0[off] = fn;
    else if (cp == 4)
        mk2cpp_tab_cp4[off] = fn;
    else
        fprintf(stderr, "mk2cpp: ignoring registration for unsupported page %02x at %08x\n", cp, flat);
}

static mk2cpp_hand_ent *mk2cpp_hand_lookup(uint32_t flat)
{
    uint32_t lo = 0, hi = mk2cpp_hand_n;
    while (lo < hi)
    {
        uint32_t mid = lo + (hi - lo) / 2;
        if (mk2cpp_hand[mid].flat < flat)
            lo = mid + 1;
        else if (mk2cpp_hand[mid].flat > flat)
            hi = mid;
        else
            return &mk2cpp_hand[mid];
    }
    return NULL;
}

static void mk2cpp_hand_insert(uint32_t flat, mk2cpp_fn fn, mk2cpp_hand_routine_fn rfn, int is_routine)
{
    uint32_t cp = (flat >> 16) & 0xffu;
    uint32_t lo = 0, hi = mk2cpp_hand_n;
    uint32_t i;
    if (cp != 0 && cp != 4)
    {
        fprintf(stderr, "mk2cpp: ignoring hand registration for unsupported page %02x at %08x\n", cp, flat);
        return;
    }
    while (lo < hi)
    {
        uint32_t mid = lo + (hi - lo) / 2;
        if (mk2cpp_hand[mid].flat < flat)
            lo = mid + 1;
        else
            hi = mid;
    }
    if (lo < mk2cpp_hand_n && mk2cpp_hand[lo].flat == flat)
    {
        fprintf(stderr, "mk2cpp: duplicate hand registration for %08x\n", flat);
        exit(1);
    }
    if (mk2cpp_hand_n >= MK2CPP_HAND_MAX)
    {
        fprintf(stderr, "mk2cpp: hand table full (%u entries), cannot register %08x\n",
                MK2CPP_HAND_MAX, flat);
        exit(1);
    }
    for (i = mk2cpp_hand_n; i > lo; i--)
        mk2cpp_hand[i] = mk2cpp_hand[i - 1];
    mk2cpp_hand[lo].flat = flat;
    mk2cpp_hand[lo].fn = fn;
    mk2cpp_hand[lo].rfn = rfn;
    mk2cpp_hand[lo].hits = 0;
    mk2cpp_hand[lo].is_routine = (uint8_t)(is_routine != 0);
    mk2cpp_hand_n++;
    if (is_routine)
        mk2cpp_hand_routines++;
}

void MK2CPP_HandRegister(uint32_t flat, mk2cpp_fn fn)
{
    mk2cpp_hand_insert(flat, fn, NULL, 0);
}

void MK2CPP_HandRegisterRoutine(uint32_t flat, mk2cpp_hand_routine_fn fn)
{
    mk2cpp_hand_insert(flat, NULL, fn, 1);
}

static mk2cpp_fn mk2cpp_lookup(uint32_t flat)
{
    uint32_t cp = (flat >> 16) & 0xffu;
    if (cp == 0)
        return mk2cpp_tab_cp0[(uint16_t)flat];
    if (cp == 4)
        return mk2cpp_tab_cp4[(uint16_t)flat];
    return NULL;
}

int MK2CPP_CanStep(uint32_t flat)
{
    if (mk2cpp_hand_enabled && mk2cpp_hand_lookup(flat) != NULL)
        return 1;
    return mk2cpp_lookup(flat) != NULL;
}

/* Mirrors the T-state trace check at the tail of MCU_ReadInstruction()
 * (src/mcu.cpp): translated and hand code must raise the same exception. */
static void mk2cpp_trace_check(void)
{
    if (mcu.sr & STATUS_T)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_TRACE);
}

void MK2CPP_Step(void)
{
    uint32_t flat = ((uint32_t)mcu.cp << 16) | mcu.pc;
    mk2cpp_hand_ent *he = mk2cpp_hand_enabled ? mk2cpp_hand_lookup(flat) : NULL;
    mk2cpp_fn fn;
    if (he)
    {
        if (he->is_routine)
        {
            uint32_t n = he->rfn();
            if (n == 0)
            {
                /* L1 protocol violation (n >= 1 required): treat as one
                 * instruction, i.e. add nothing here and keep the host's
                 * single +12; count it and warn once to stay diagnosable
                 * without spamming a hot path. */
                mk2cpp_hand_bad_n++;
                if (!mk2cpp_hand_bad_n_warned)
                {
                    mk2cpp_hand_bad_n_warned = 1;
                    fprintf(stderr, "mk2cpp: routine hand hook %08x returned 0 instructions, treating as 1\n", flat);
                }
                n = 1;
            }
            /* L1 cycle protocol (out/09 4.3): the host adds 12 once after this
             * step (src/mcu.cpp:1912), so complete n*12 with 12*(n-1) here. */
            mcu.cycles += (uint64_t)12u * (uint64_t)(n - 1u);
        }
        else
        {
            he->fn();
        }
        he->hits++;
        mk2cpp_hand_hits++;
        mk2cpp_trace_check();
        return;
    }
    fn = mk2cpp_lookup(flat);
    if (fn)
    {
        fn();
        mk2cpp_translated++;
        mk2cpp_trace_check();
        return;
    }
    mk2cpp_fallbacks++;
    if (mk2cpp_mixed)
    {
        MCU_ReadInstruction(); /* already performs the trace check itself */
        return;
    }
    fprintf(stderr, "mk2cpp: no translation for %08x and mixed mode is disabled\n", flat);
    exit(1);
}

void MK2CPP_SM_Register(uint16_t pc, mk2cpp_sm_fn fn)
{
    mk2cpp_sm_tab[pc] = fn;
}

int MK2CPP_SM_CanStep(uint16_t pc)
{
    return mk2cpp_sm_tab[pc] != NULL;
}

void MK2CPP_SM_Step(void)
{
    uint16_t pc = sm.pc;
    mk2cpp_sm_fn fn = mk2cpp_sm_tab[pc];
    if (fn)
    {
        fn();
        mk2cpp_sm_translated++;
        return;
    }
    mk2cpp_sm_fallbacks++;
    if (mk2cpp_mixed)
    {
        SM_ExecuteOneInstruction();
        return;
    }
    fprintf(stderr, "mk2cpp: no SM translation for %04x and mixed mode is disabled\n", pc);
    exit(1);
}

uint32_t MK2CPP_SM_FallbackCount(void)
{
    return mk2cpp_sm_fallbacks;
}

uint32_t MK2CPP_SM_TranslatedCount(void)
{
    return mk2cpp_sm_translated;
}

void MK2CPP_Init(void)
{
    uint32_t n_main = 0;
    uint32_t n_sm = 0;
    int i;
#ifdef MK2CPP_HAS_GEN
    extern void MK2CPP_FillTables(void);
    MK2CPP_FillTables();
#endif
#ifdef MK2CPP_HAS_SM_GEN
    extern void MK2CPP_SM_FillTables(void);
    MK2CPP_SM_FillTables();
#endif
#ifdef MK2CPP_HAS_HAND
    MK2CPP_HandFillTables();
#endif
    for (i = 0; i < 0x10000; i++)
    {
        if (mk2cpp_tab_cp0[i]) n_main++;
        if (mk2cpp_tab_cp4[i]) n_main++;
        if (mk2cpp_sm_tab[i]) n_sm++;
    }
    if (n_main || n_sm)
    {
        printf("mk2cpp: %u translated main PCs, %u translated SM PCs registered\n",
               n_main, n_sm);
        fflush(stdout);
    }
}

uint32_t MK2CPP_FallbackCount(void)
{
    return mk2cpp_fallbacks;
}

uint32_t MK2CPP_TranslatedCount(void)
{
    return mk2cpp_translated;
}

uint32_t MK2CPP_HandCount(void)
{
    return mk2cpp_hand_n;
}

uint32_t MK2CPP_HandHitCount(void)
{
    return mk2cpp_hand_hits;
}

uint32_t MK2CPP_HandRoutineCount(void)
{
    return mk2cpp_hand_routines;
}

uint32_t MK2CPP_HandCapacity(void)
{
    return (uint32_t)MK2CPP_HAND_MAX;
}

void MK2CPP_Configure(int argc, char **argv)
{
    int i;
    for (i = 1; i < argc; i++)
    {
        if (!strncmp(argv[i], "-mk2cpp-hand:", 13))
        {
            const char *value = argv[i] + 13;
            if (!strcmp(value, "0") || !strcmp(value, "1"))
                mk2cpp_hand_enabled = (value[0] == '1');
            else
                fprintf(stderr, "warning: -mk2cpp-hand:%s invalid (expected 0 or 1), ignored\n", value);
        }
    }
}

void MK2CPP_PostReset(void)
{
    /* Native engine configuration capture (M4 out/11 2.4): the voice count is
     * taken from pcm_ext_voices here, after GT's reset chain and after argv
     * parsing, so -mk2cpp without -voices keeps the stock 28. Placeholder for
     * pcm_ext_active activation once the native engine can drive it (out/09 3.2). */
#ifdef MK2CPP_HAS_HAND
    MK2CPP_HandPostReset();
#endif
}

const char *MK2CPP_Version(void)
{
#ifdef MK2CPP_HAS_GEN
    return "mk2cpp translated core";
#else
    return "mk2cpp: no translated code linked (stock interpreter only)";
#endif
}
