/*
 * mk2cpp.cpp -- GT integration glue for the translated core.
 *
 * No behavior change unless -mk2cpp is given AND translated code was linked
 * (see MK2CPP_GEN_DIR in CMakeLists.txt). Generated code registers one
 * function per translated PC via MK2CPP_RegisterFn().
 */
#include "mk2cpp.h"
#include "mcu.h"

#include <stdio.h>
#include <stdlib.h>

/* Defined in src/mcu.cpp (GT's interpreter step). */
extern void MCU_ReadInstruction(void);

int mk2cpp_enabled = 0;
int mk2cpp_mixed = 1;

static mk2cpp_fn mk2cpp_tab_cp0[0x10000];
static mk2cpp_fn mk2cpp_tab_cp4[0x10000];
static uint32_t mk2cpp_fallbacks = 0;
static uint32_t mk2cpp_translated = 0;

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
    return mk2cpp_lookup(flat) != NULL;
}

void MK2CPP_Step(void)
{
    uint32_t flat = ((uint32_t)mcu.cp << 16) | mcu.pc;
    mk2cpp_fn fn = mk2cpp_lookup(flat);
    if (fn)
    {
        fn();
        mk2cpp_translated++;
        return;
    }
    mk2cpp_fallbacks++;
    if (mk2cpp_mixed)
    {
        MCU_ReadInstruction();
        return;
    }
    fprintf(stderr, "mk2cpp: no translation for %08x and mixed mode is disabled\n", flat);
    exit(1);
}

void MK2CPP_Init(void)
{
#ifdef MK2CPP_HAS_GEN
    extern void MK2CPP_FillTables(void);
    MK2CPP_FillTables();
    {
        uint32_t n = 0;
        for (int i = 0; i < 0x10000; i++)
        {
            if (mk2cpp_tab_cp0[i]) n++;
            if (mk2cpp_tab_cp4[i]) n++;
        }
        printf("mk2cpp: %u translated PCs registered\n", n);
        fflush(stdout);
    }
#endif
}

uint32_t MK2CPP_FallbackCount(void)
{
    return mk2cpp_fallbacks;
}

uint32_t MK2CPP_TranslatedCount(void)
{
    return mk2cpp_translated;
}

const char *MK2CPP_Version(void)
{
#ifdef MK2CPP_HAS_GEN
    return "mk2cpp translated core";
#else
    return "mk2cpp: no translated code linked (stock interpreter only)";
#endif
}
