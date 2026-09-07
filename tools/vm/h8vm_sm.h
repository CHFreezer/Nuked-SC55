// Standalone SM sub-CPU header (ported from nukeykt submcu.h).
#pragma once
#include <stdint.h>

enum {
    SM_STATUS_C = 1,
    SM_STATUS_Z = 2,
    SM_STATUS_I = 4,
    SM_STATUS_D = 8,
    SM_STATUS_B = 16,
    SM_STATUS_T = 32,
    SM_STATUS_V = 64,
    SM_STATUS_N = 128
};

struct submcu_t {
    uint16_t pc;
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t s;
    uint8_t sr;
    uint64_t cycles;
    uint8_t sleep;
};
typedef struct submcu_t submcu_t;

extern submcu_t sm;
extern uint8_t sm_rom[4096];
extern uint8_t sm_device_mode[32];
extern uint8_t sm_ram[128];

void SM_Reset(void);
void SM_Update(uint64_t cycles);
uint8_t SM_SysRead(uint32_t address);
void SM_SysWrite(uint32_t address, uint8_t data);

// Shared UART buffer: main MCU posts host/MIDI bytes here, SM consumes them.
#define SM_UART_BUFFER_SIZE 8192
extern uint8_t sm_uart_buffer[SM_UART_BUFFER_SIZE];
extern uint32_t sm_uart_read_ptr;
extern uint32_t sm_uart_write_ptr;

// Main-MCU interrupt sources (subset, from mcu_interrupt.h)
#define INTERRUPT_SOURCE_NMI 0
#define INTERRUPT_SOURCE_IRQ0 1
#define INTERRUPT_SOURCE_IRQ1 2
#define INTERRUPT_SOURCE_UART_RX 19
#define INTERRUPT_SOURCE_UART_TX 20
#define INTERRUPT_SOURCE_MAX 21

// Main-MCU vector numbers (from mcu.h)
#define VECTOR_IRQ0 32
#define VECTOR_IRQ1 33
#define VECTOR_INTERNAL_INTERRUPT_D4 53 // UART_RX

// Implemented in h8vm_main.c; called by the SM's GA-line coupling.
void MCU_Interrupt_SetRequest(uint32_t interrupt, uint32_t value);

// Get-and-clear the pending GA line (main MCU reads this via 0xe402).
int SM_GATrigger(void);
