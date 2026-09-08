// Standalone SM sub-CPU header (ported from nukeykt submcu.h).
#pragma once
#include <stdint.h>
#include <stdio.h>

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

// main-coupling latches + SM device state (exposed for the full-state snapshot)
extern uint8_t mcu_p0_data, mcu_p1_data;
extern int ga_int[8];
extern int ga_int_enable;
extern int ga_int_trigger;
extern uint8_t sm_shared_ram[192];
extern uint8_t sm_access[0x18];
extern uint8_t sm_p0_dir, sm_p1_dir, sm_cts;
extern uint64_t sm_timer_cycles;
extern uint8_t sm_timer_prescaler, sm_timer_counter;

void SM_Reset(void);
void SM_Update(uint64_t cycles);

// Unified PC trace (main + SM in one file), owned by the main MCU. The SM calls
// this for every non-sleep instruction: type 1 -> "s <sm_cycles> <pc>".
// (type 0 -> "m <mcu_cycles> <cp:pc>" is used by the main loop.)
void trace_write(uint8_t type, uint64_t cycle, uint32_t value);
// vm.log SM-instruction log (no-op when the plain vm.log is closed, i.e. when
// tracepc is on). Writes one "s <sm_cycles> <pc>" line per SM instruction so the
// complete SM PC set is captured in vm.log.
void log_sm_instr(uint64_t cycles, uint32_t pc);
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
