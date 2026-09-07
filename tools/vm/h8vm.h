// Standalone H8/532 VM - self-contained header (ported from nukeykt mcu.h, no external deps).
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define STATUS_T 0x8000
#define STATUS_N 0x08
#define STATUS_Z 0x04
#define STATUS_V 0x02
#define STATUS_C 0x01
#define STATUS_INT_MASK 0x700
#define sr_mask 0x870f

enum { EXCEPTION_SOURCE_ADDRESS_ERROR = 0, EXCEPTION_SOURCE_INVALID_INSTRUCTION, EXCEPTION_SOURCE_TRACE };
enum {
    INTERRUPT_SOURCE_NMI = 0,
    INTERRUPT_SOURCE_IRQ0,
    INTERRUPT_SOURCE_IRQ1,
    INTERRUPT_SOURCE_FRT0_ICI,
    INTERRUPT_SOURCE_FRT0_OCIA,
    INTERRUPT_SOURCE_FRT0_OCIB,
    INTERRUPT_SOURCE_FRT0_FOVI,
    INTERRUPT_SOURCE_FRT1_ICI,
    INTERRUPT_SOURCE_FRT1_OCIA,
    INTERRUPT_SOURCE_FRT1_OCIB,
    INTERRUPT_SOURCE_FRT1_FOVI,
    INTERRUPT_SOURCE_FRT2_ICI,
    INTERRUPT_SOURCE_FRT2_OCIA,
    INTERRUPT_SOURCE_FRT2_OCIB,
    INTERRUPT_SOURCE_FRT2_FOVI,
    INTERRUPT_SOURCE_TIMER_CMIA,
    INTERRUPT_SOURCE_TIMER_CMIB,
    INTERRUPT_SOURCE_TIMER_OVI,
    INTERRUPT_SOURCE_ANALOG,
    INTERRUPT_SOURCE_UART_RX,
    INTERRUPT_SOURCE_UART_TX,
    INTERRUPT_SOURCE_MAX
};
// vector numbers (from mcu.h)
#define VECTOR_INVALID_INSTRUCTION 2
#define VECTOR_ADDRESS_ERROR 8
#define VECTOR_TRACE 9
#define VECTOR_NMI 11
#define VECTOR_TRAPA_0 16
#define VECTOR_IRQ0 32
#define VECTOR_IRQ1 33
#define VECTOR_INTERNAL_INTERRUPT_94 37
#define VECTOR_INTERNAL_INTERRUPT_98 38
#define VECTOR_INTERNAL_INTERRUPT_9C 39
#define VECTOR_INTERNAL_INTERRUPT_A4 41
#define VECTOR_INTERNAL_INTERRUPT_A8 42
#define VECTOR_INTERNAL_INTERRUPT_AC 43
#define VECTOR_INTERNAL_INTERRUPT_B4 45
#define VECTOR_INTERNAL_INTERRUPT_B8 46
#define VECTOR_INTERNAL_INTERRUPT_BC 47
#define VECTOR_INTERNAL_INTERRUPT_C0 48
#define VECTOR_INTERNAL_INTERRUPT_C4 49
#define VECTOR_INTERNAL_INTERRUPT_C8 50
#define VECTOR_INTERNAL_INTERRUPT_D4 53
#define VECTOR_INTERNAL_INTERRUPT_D8 54
#define VECTOR_INTERNAL_INTERRUPT_E0 56

struct mcu_t {
    uint16_t r[8];
    uint16_t pc;
    uint16_t sr;
    uint8_t cp, dp, ep, tp, br;
    uint8_t sleep;
    uint8_t ex_ignore;
    int32_t exception_pending;
    uint8_t interrupt_pending[INTERRUPT_SOURCE_MAX];
    uint8_t trapa_pending[16];
    uint64_t cycles;
};
typedef struct mcu_t mcu_t;

extern mcu_t mcu;

// Memory + helpers (implemented in h8vm_main.c)
uint8_t MCU_Read(uint32_t address);
uint16_t MCU_Read16(uint32_t address);
uint32_t MCU_Read32(uint32_t address);
void MCU_Write(uint32_t address, uint8_t value);
void MCU_Write16(uint32_t address, uint16_t value);
void MCU_Interrupt_Exception(uint32_t exception);
void MCU_Interrupt_SetRequest(uint32_t interrupt, uint32_t value);
void MCU_Interrupt_TRAPA(uint32_t vector);
void MCU_ErrorTrap(void);

// General-instruction opcode table (defined in h8vm_body.c)
typedef void (*mcu_opcode_fn)(uint8_t opcode, uint8_t opcode_reg);
extern mcu_opcode_fn MCU_Opcode_Table[32];

inline uint32_t MCU_GetAddress(uint8_t page, uint16_t address) { return (page << 16) + address; }
inline uint8_t MCU_ReadCode(void) { return MCU_Read(MCU_GetAddress(mcu.cp, mcu.pc)); }
inline uint8_t MCU_ReadCodeAdvance(void) { uint8_t ret = MCU_ReadCode(); mcu.pc++; return ret; }

inline void MCU_SetStatus(uint32_t condition, uint32_t mask) {
    if (condition) mcu.sr |= mask; else mcu.sr &= ~mask;
}

inline void MCU_PushStack(uint16_t data) {
    if (mcu.r[7] & 1) MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    mcu.r[7] -= 2;
    MCU_Write16(mcu.r[7], data);
}

inline uint16_t MCU_PopStack(void) {
    uint16_t ret;
    if (mcu.r[7] & 1) MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    ret = MCU_Read16(mcu.r[7]);
    mcu.r[7] += 2;
    return ret;
}

inline uint32_t MCU_GetVectorAddress(uint32_t vector) { return MCU_Read32(vector * 4); }

inline uint32_t MCU_GetPageForRegister(uint32_t reg) {
    if (reg >= 6) return mcu.tp;
    else if (reg >= 4) return mcu.ep;
    return mcu.dp;
}

inline void MCU_ControlRegisterWrite(uint32_t reg, uint32_t siz, uint32_t data) {
    if (siz) {
        if (reg == 0) { mcu.sr = data; mcu.sr &= sr_mask; }
        else if (reg == 5) mcu.dp = data & 0xff;
        else if (reg == 4) mcu.ep = data & 0xff;
        else if (reg == 3) mcu.br = data & 0xff;
        else MCU_ErrorTrap();
    } else {
        if (reg == 1) { mcu.sr &= ~0xff; mcu.sr |= data & 0xff; mcu.sr &= sr_mask; }
        else if (reg == 3) mcu.br = data;
        else if (reg == 4) mcu.ep = data;
        else if (reg == 5) mcu.dp = data;
        else if (reg == 7) mcu.tp = data;
        else MCU_ErrorTrap();
    }
}

inline uint32_t MCU_ControlRegisterRead(uint32_t reg, uint32_t siz) {
    uint32_t ret = 0;
    if (siz) {
        if (reg == 0) ret = mcu.sr & sr_mask;
        else if (reg == 5) ret = mcu.dp | (mcu.dp << 8);
        else if (reg == 4) ret = mcu.ep | (mcu.ep << 8);
        else if (reg == 3) ret = mcu.br | (mcu.br << 8);
        else MCU_ErrorTrap();
        ret &= 0xffff;
    } else {
        if (reg == 1) ret = mcu.sr & sr_mask;
        else if (reg == 3) ret = mcu.br;
        else if (reg == 4) ret = mcu.ep;
        else if (reg == 5) ret = mcu.dp;
        else if (reg == 7) ret = mcu.tp;
        else MCU_ErrorTrap();
        ret &= 0xff;
    }
    return ret;
}

// Main opcode dispatch table (defined in h8vm_body.c)
extern void (*MCU_Operand_Table[256])(uint8_t operand);

// INT16/INT8 limits (for the ADD/SUB common helpers)
#ifndef INT16_MIN
#define INT16_MIN (-32768)
#define INT16_MAX (32767)
#define INT8_MIN (-128)
#define INT8_MAX (127)
#endif
