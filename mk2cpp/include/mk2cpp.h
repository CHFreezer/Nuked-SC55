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
/* L1 whole-routine hook (M4, out/09 4.3): runs a multi-instruction routine in
 * one MK2CPP_Step and returns the number of H8 instructions executed (n >= 1).
 * MK2CPP_Step adds 12*(n-1) cycles so the host's own +=12 completes n*12; the
 * routine must leave pc/cp/flags exactly as the stock routine would. */
typedef uint32_t (*mk2cpp_hand_routine_fn)(void);

/* Called once at startup (before execution); registers generated code. */
void MK2CPP_Init(void);
/* Called by generated code to publish one translated PC. */
void MK2CPP_RegisterFn(uint32_t flat, mk2cpp_fn fn);
/* Non-zero if a translated implementation exists for flat = (cp<<16)|pc. */
int  MK2CPP_CanStep(uint32_t flat);
/* Execute exactly one instruction at the current mcu.cp/mcu.pc. */
void MK2CPP_Step(void);

/* ---- hand overrides (M4) ---------------------------------------------- */
/* 1 (default): hand implementations take priority over generated code.
 * 0 (-mk2cpp-hand:0): the hand table is skipped (pure gen/interpreter A/B). */
extern int mk2cpp_hand_enabled;
/* Publish one hand implementation for flat = (cp<<16)|pc (cp 0 or 4 only).
 * Called by hand modules; duplicate flat is a fatal error. */
void MK2CPP_HandRegister(uint32_t flat, mk2cpp_fn fn);
/* Publish one L1 whole-routine hand implementation for flat = (cp<<16)|pc
 * (cp 0 or 4 only); shares the L0 sparse table and lookup. Duplicate flat is a
 * fatal error. The returned instruction count must be >= 1. */
void MK2CPP_HandRegisterRoutine(uint32_t flat, mk2cpp_hand_routine_fn fn);
/* Provided by hand modules; MK2CPP_Init calls it after the generated tables
 * are filled (guarded by MK2CPP_HAS_HAND) to publish all committed overrides. */
void MK2CPP_HandFillTables(void);
/* Provided by hand modules; called by MK2CPP_PostReset() (guarded by
 * MK2CPP_HAS_HAND) to capture post-reset native engine configuration. */
void MK2CPP_HandPostReset(void);
/* Called after the command line has been parsed; consumes -mk2cpp-hand:<0|1>. */
void MK2CPP_Configure(int argc, char **argv);
/* Called after PCM_Reset(); native extension-mode activation lands here. */
void MK2CPP_PostReset(void);

/* ---- SM (sub MCU, M37450) translated dispatch (M3) -------------------- */
typedef void (*mk2cpp_sm_fn)(void);
/* Called by generated code to publish one translated SM PC (0xf000..0xffff). */
void MK2CPP_SM_Register(uint16_t pc, mk2cpp_sm_fn fn);
/* Non-zero if a translated implementation exists for the given sm.pc. */
int  MK2CPP_SM_CanStep(uint16_t pc);
/* Execute exactly one instruction at the current sm.pc (fallback if absent). */
void MK2CPP_SM_Step(void);

/* Diagnostics. */
uint32_t MK2CPP_FallbackCount(void);
uint32_t MK2CPP_TranslatedCount(void);
uint32_t MK2CPP_HandCount(void);
uint32_t MK2CPP_HandHitCount(void);
uint32_t MK2CPP_HandRoutineCount(void);
/* Compile-time hand table capacity (entries); registration fails fast past it. */
uint32_t MK2CPP_HandCapacity(void);
uint32_t MK2CPP_SM_FallbackCount(void);
uint32_t MK2CPP_SM_TranslatedCount(void);
const char *MK2CPP_Version(void);

#ifdef __cplusplus
}
#endif
