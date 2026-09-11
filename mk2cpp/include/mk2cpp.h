/*
 * mk2cpp.h -- SC-55mk2 ROM -> C++ translated core, integrated into GT.
 *
 * The translated core runs inside the GT emulator and replaces the H8
 * interpreter for the addresses it provides. Device models, scheduling and
 * timing stay in GT; translated code operates on the same global `mcu`
 * state and calls the same MCU_* helpers. Default (no -mk2cpp flag, or no
 * translated code linked) leaves GT behavior byte-identical.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Set by the -mk2cpp command line flag. */
extern int mk2cpp_enabled;
/* 1 (default): untranslated PCs fall back to the GT interpreter. */
extern int mk2cpp_mixed;

typedef void (*mk2cpp_fn)(void);

/* Called once at startup (before execution); registers generated code. */
void MK2CPP_Init(void);
/* Called by generated code to publish one translated PC. */
void MK2CPP_RegisterFn(uint32_t flat, mk2cpp_fn fn);
/* Non-zero if a translated implementation exists for flat = (cp<<16)|pc. */
int  MK2CPP_CanStep(uint32_t flat);
/* Execute exactly one instruction at the current mcu.cp/mcu.pc. */
void MK2CPP_Step(void);

/* Diagnostics. */
uint32_t MK2CPP_FallbackCount(void);
uint32_t MK2CPP_TranslatedCount(void);
const char *MK2CPP_Version(void);

#ifdef __cplusplus
}
#endif
