// Standalone H8/532 VM - main: memory map + fetch/execute loop + trace (ported from nukeykt mcu.cpp).
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "h8vm.h"
#include "h8vm_sm.h"
#include "h8vm_pcm.h"

static const int ROM1_SIZE = 0x8000;
static const int ROM2_SIZE = 0x80000;
static const int ROMSM_SIZE = 0x1000;
static const int SRAM_SIZE = 0x8000;

static uint8_t rom1[ROM1_SIZE];
static uint8_t rom2[ROM2_SIZE];
static uint8_t sram[SRAM_SIZE];
static uint8_t dev[0x100];
static uint8_t ram[0x400]; // nukeykt RAM_SIZE work RAM (page 0 0xfb80-0xfff7 when RAME set)
static const int BRAM_SIZE = 0x10000;
static uint8_t b_ram[BRAM_SIZE]; // 方案A: 64KB extended RAM backing page 6 (was unbacked: rd 0xff / write dropped)

// UART: shared buffer (defined in h8vm_sm.c). On SC-55 the SM sub-CPU consumes
// host/MIDI bytes from it; the main MCU's own UART RX is not used.
static uint8_t uart_rx_byte;

// ---- logging: single file written directly from C (no stdout). The per-instr
// PC line always goes here; detailed dumps (reads / frt state / sram writes /
// divergence) are gated by g_verbose (off by default so a long run stays fast).
// stdout is reserved for the leading '#' status lines only. ----
static FILE *g_log = NULL;
static char *g_log_buf = NULL;
static int g_verbose = 0;
static void vm_log(const char *fmt, ...)
{
    if (!g_log) return;
    char tbuf[600];
    va_list ap; va_start(ap, fmt); vsnprintf(tbuf, sizeof tbuf, fmt, ap); va_end(ap);
    fputs(tbuf, g_log);
}

// ---- unified PC trace (main + SM in ONE file), owned by the main MCU. ----
// type 0 (main): "m <mcu_cycles> <cp:pc>"; type 1 (SM): "s <sm_cycles> <pc>".
// The main loop opens/closes the file on the tracepc window; the SM calls
// trace_write() for every non-sleep instruction (no-ops when the file is closed).
static FILE *g_trace_f = NULL;
static char *g_trace_buf = NULL;
static int g_trace_closed = 0;
void trace_write(uint8_t type, uint64_t cycle, uint32_t value)
{
    if (!g_trace_f) return;
    if (type == 0)
        fprintf(g_trace_f, "m %llu %02x:%04x\n", (unsigned long long)cycle, (unsigned)((value >> 16) & 0xff), (unsigned)(value & 0xffff));
    else
        fprintf(g_trace_f, "s %llu %04x\n", (unsigned long long)cycle, (unsigned)(value & 0xffff));
}

// ---- vm.log SM-instruction log (plain vm.log path, i.e. tracepc off). ----
// The main loop logs one sampled sm.pc per instruction, which misses the SM
// instructions that run between samples (SM is 5x async). The SM calls this for
// every non-sleep instruction, writing the same "s <sm_cycles> <pc>" line as the
// trace file, so the complete SM PC set is recoverable from vm.log. No-op when
// g_log is closed (it is NULL while tracepc is on).
void log_sm_instr(uint64_t cycles, uint32_t pc)
{
    if (!g_log) return;
    fprintf(g_log, "s %llu %04x\n", (unsigned long long)cycles, (unsigned)(pc & 0xffff));
}

// device register indices (from mcu.h)
#define DEV_SCR 0x5a
#define DEV_SSR 0x5c
#define DEV_RDR 0x5d
#define DEV_IPRA 0x70
#define DEV_IPRB 0x71
#define DEV_IPRC 0x72
#define DEV_IPRD 0x73
#define DEV_P9DDR 0x7e
#define DEV_P9DR 0x7f
#define DEV_RAME 0x79
#define DEV_TMR_TCR 0x50
#define DEV_TMR_TCSR 0x51
#define DEV_TMR_TCORA 0x52
#define DEV_TMR_TCORB 0x53
#define DEV_TMR_TCNT 0x54
#define DEV_ADDRAH 0x60
#define DEV_ADDRAL 0x61
#define DEV_ADCSR 0x68

// ---- FRT / 8-bit timer state (ported from nukeykt mcu_timer.cpp) ----
static int mcu_mk1 = 0; // SC-55mk2
static uint64_t timer_cycles;
static uint8_t timer_tempreg;
struct frt_t {
    uint8_t tcr, tcsr;
    uint16_t frc, ocra, ocrb, icr;
    uint8_t status_rd;
} frt[3];
struct mcu_timer_t {
    uint8_t tcr, tcsr, tcora, tcorb, tcnt, status_rd;
} timer;

static void frt_write(uint32_t address, uint8_t data)
{
    uint32_t t = (address >> 4) - 1;
    if (t > 2) return;
    address &= 0x0f;
    struct frt_t *tm = &frt[t];
    switch (address)
    {
    case 0x00: tm->tcr = data; break;
    case 0x01:
        tm->tcsr &= ~0xf; tm->tcsr |= data & 0xf;
        if ((data & 0x10) == 0 && (tm->status_rd & 0x10) != 0) { tm->tcsr &= ~0x10; tm->status_rd &= ~0x10; MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_FRT0_FOVI + t*4, 0); }
        if ((data & 0x20) == 0 && (tm->status_rd & 0x20) != 0) { tm->tcsr &= ~0x20; tm->status_rd &= ~0x20; MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_FRT0_OCIA + t*4, 0); }
        if ((data & 0x40) == 0 && (tm->status_rd & 0x40) != 0) { tm->tcsr &= ~0x40; tm->status_rd &= ~0x40; MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_FRT0_OCIB + t*4, 0); }
        break;
    case 0x02: case 0x04: case 0x06: case 0x08: timer_tempreg = data; break;
    case 0x03: tm->frc = (timer_tempreg << 8) | data; break;
    case 0x05: tm->ocra = (timer_tempreg << 8) | data; break;
    case 0x07: tm->ocrb = (timer_tempreg << 8) | data; break;
    case 0x09: tm->icr = (timer_tempreg << 8) | data; break;
    }
}
static uint8_t frt_read(uint32_t address)
{
    uint32_t t = (address >> 4) - 1;
    if (t > 2) return 0xff;
    address &= 0x0f;
    struct frt_t *tm = &frt[t];
    switch (address)
    {
    case 0x00: return tm->tcr;
    case 0x01: { uint8_t ret = tm->tcsr; tm->status_rd |= tm->tcsr & 0xf0; return ret; }
    case 0x02: timer_tempreg = tm->frc & 0xff; return tm->frc >> 8;
    case 0x04: timer_tempreg = tm->ocra & 0xff; return tm->ocra >> 8;
    case 0x06: timer_tempreg = tm->ocrb & 0xff; return tm->ocrb >> 8;
    case 0x08: timer_tempreg = tm->icr & 0xff; return tm->icr >> 8;
    case 0x03: case 0x05: case 0x07: case 0x09: return timer_tempreg;
    }
    return 0xff;
}
static void frt2_write(uint32_t address, uint8_t data)
{
    switch (address)
    {
    case DEV_TMR_TCR: timer.tcr = data; break;
    case DEV_TMR_TCSR:
        timer.tcsr &= ~0xf; timer.tcsr |= data & 0xf;
        if ((data & 0x20) == 0 && (timer.status_rd & 0x20) != 0) { timer.tcsr &= ~0x20; timer.status_rd &= ~0x20; MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_TIMER_OVI, 0); }
        if ((data & 0x40) == 0 && (timer.status_rd & 0x40) != 0) { timer.tcsr &= ~0x40; timer.status_rd &= ~0x40; MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_TIMER_CMIA, 0); }
        if ((data & 0x80) == 0 && (timer.status_rd & 0x80) != 0) { timer.tcsr &= ~0x80; timer.status_rd &= ~0x80; MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_TIMER_CMIB, 0); }
        break;
    case DEV_TMR_TCORA: timer.tcora = data; break;
    case DEV_TMR_TCORB: timer.tcorb = data; break;
    case DEV_TMR_TCNT: timer.tcnt = data; break;
    }
}
static uint8_t frt2_read(uint32_t address)
{
    switch (address)
    {
    case DEV_TMR_TCR: return timer.tcr;
    case DEV_TMR_TCSR: { uint8_t ret = timer.tcsr; timer.status_rd |= timer.tcsr & 0xe0; return ret; }
    case DEV_TMR_TCORA: return timer.tcora;
    case DEV_TMR_TCORB: return timer.tcorb;
    case DEV_TMR_TCNT: return timer.tcnt;
    }
    return 0xff;
}
static void TIMER_Clock(uint64_t cycles)
{
    while (timer_cycles*2 < cycles)
    {
        for (int i = 0; i < 3; i++)
        {
            struct frt_t *tm = &frt[i];
            switch (tm->tcr & 3)
            {
            case 0: if (timer_cycles & 3) continue; break;
            case 1: if (timer_cycles & 7) continue; break;
            case 2: if (timer_cycles & 31) continue; break;
            case 3:
                if (mcu_mk1) { if (timer_cycles & 3) continue; }
                else { if (timer_cycles & 1) continue; }
                break;
            }
            uint32_t value = tm->frc;
            uint32_t matcha = value == tm->ocra;
            uint32_t matchb = value == tm->ocrb;
            if ((tm->tcsr & 1) != 0 && matcha) value = 0; else value++;
            uint32_t of = (value >> 16) & 1;
            value &= 0xffff;
            tm->frc = value;
            if (of) tm->tcsr |= 0x10;
            if (matcha) tm->tcsr |= 0x20;
            if (matchb) tm->tcsr |= 0x40;
            if ((tm->tcr & 0x10) != 0 && (tm->tcsr & 0x10) != 0) MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_FRT0_FOVI + i*4, 1);
            if ((tm->tcr & 0x20) != 0 && (tm->tcsr & 0x20) != 0) MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_FRT0_OCIA + i*4, 1);
            if ((tm->tcr & 0x40) != 0 && (tm->tcsr & 0x40) != 0) MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_FRT0_OCIB + i*4, 1);
        }
        int32_t timer_step = 0;
        switch (timer.tcr & 7)
        {
        case 0: case 4: break;
        case 1: if ((timer_cycles & 7) == 0) timer_step = 1; break;
        case 2: if ((timer_cycles & 63) == 0) timer_step = 1; break;
        case 3: if ((timer_cycles & 1023) == 0) timer_step = 1; break;
        case 5: case 6: case 7:
            if (mcu_mk1) { if ((timer_cycles & 3) == 0) timer_step = 1; }
            else { if ((timer_cycles & 1) == 0) timer_step = 1; }
            break;
        }
        if (timer_step)
        {
            uint32_t value = timer.tcnt;
            uint32_t matcha = value == timer.tcora;
            uint32_t matchb = value == timer.tcorb;
            if ((timer.tcr & 24) == 8 && matcha) value = 0;
            else if ((timer.tcr & 24) == 16 && matchb) value = 0;
            else value++;
            uint32_t of = (value >> 8) & 1;
            value &= 0xff;
            timer.tcnt = value;
            if (of) timer.tcsr |= 0x20;
            if (matcha) timer.tcsr |= 0x40;
            if (matchb) timer.tcsr |= 0x80;
            if ((timer.tcr & 0x20) != 0 && (timer.tcsr & 0x20) != 0) MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_TIMER_OVI, 1);
            if ((timer.tcr & 0x40) != 0 && (timer.tcsr & 0x40) != 0) MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_TIMER_CMIA, 1);
            if ((timer.tcr & 0x80) != 0 && (timer.tcsr & 0x80) != 0) MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_TIMER_CMIB, 1);
        }
        timer_cycles++;
    }
}

// ---- Analog/ADC engine (ported from nukeykt mcu.cpp MCU_UpdateAnalog) ----
static uint64_t analog_end_time = 0;
static int adf_rd = 0;
static uint8_t io_sd = 0x00;
static uint8_t sw_pos = 3;
static uint16_t ad_val[4]; // snapshot parity (unused by logic; ADC writes dev directly)
static uint8_t ad_nibble;
static int ga_lcd_counter = 0; // snapshot parity (decrements only on mk1; idle on mk2)

// LCD display state (ported from nukeykt lcd.cpp) — kept for snapshot parity.
static uint32_t LCD_DL, LCD_N, LCD_F, LCD_D, LCD_C, LCD_B, LCD_ID, LCD_S;
static uint32_t LCD_DD_RAM, LCD_AC, LCD_CG_RAM;
static uint32_t LCD_RAM_MODE = 0;
static uint8_t LCD_Data[80];
static uint8_t LCD_CG[64];
static uint8_t lcd_enable = 1;

static void LCD_Enable(uint32_t enable) { lcd_enable = enable; }
static void LCD_Write(uint32_t address, uint8_t data)
{
    if (address == 0)
    {
        if ((data & 0xe0) == 0x20)
        {
            LCD_DL = (data & 0x10) != 0;
            LCD_N = (data & 0x8) != 0;
            LCD_F = (data & 0x4) != 0;
        }
        else if ((data & 0xf8) == 0x8)
        {
            LCD_D = (data & 0x4) != 0;
            LCD_C = (data & 0x2) != 0;
            LCD_B = (data & 0x1) != 0;
        }
        else if ((data & 0xff) == 0x01)
        {
            LCD_DD_RAM = 0;
            LCD_ID = 1;
            memset(LCD_Data, 0x20, sizeof LCD_Data);
        }
        else if ((data & 0xff) == 0x02)
        {
            LCD_DD_RAM = 0;
        }
        else if ((data & 0xfc) == 0x04)
        {
            LCD_ID = (data & 0x2) != 0;
            LCD_S = (data & 0x1) != 0;
        }
        else if ((data & 0xc0) == 0x40)
        {
            LCD_CG_RAM = (data & 0x3f);
            LCD_RAM_MODE = 0;
        }
        else if ((data & 0x80) == 0x80)
        {
            LCD_DD_RAM = (data & 0x7f);
            LCD_RAM_MODE = 1;
        }
    }
    else
    {
        if (!LCD_RAM_MODE)
        {
            LCD_CG[LCD_CG_RAM] = data & 0x1f;
            if (LCD_ID) LCD_CG_RAM++;
            else LCD_CG_RAM--;
            LCD_CG_RAM &= 0x3f;
        }
        else
        {
            if (LCD_N)
            {
                if (LCD_DD_RAM & 0x40)
                {
                    if ((LCD_DD_RAM & 0x3f) < 40)
                        LCD_Data[(LCD_DD_RAM & 0x3f) + 40] = data;
                }
                else
                {
                    if ((LCD_DD_RAM & 0x3f) < 40)
                        LCD_Data[LCD_DD_RAM & 0x3f] = data;
                }
            }
            else
            {
                if (LCD_DD_RAM < 80)
                    LCD_Data[LCD_DD_RAM] = data;
            }
            if (LCD_ID) LCD_DD_RAM++;
            else LCD_DD_RAM--;
            LCD_DD_RAM &= 0x7f;
        }
    }
}

static uint16_t analog_read_pin(uint32_t pin)
{
    // SC-55mk2 (mk1=0, jv880=0, cm300=0, sc155=0)
    if (pin == 7)
    {
        switch ((io_sd >> 2) & 3)
        {
        case 0: return 0x2a0; // BATTERY
        case 1: return 0;     // NC
        case 2:               // SW
            switch (sw_pos)
            {
            case 1: return 0x155;
            case 2: return 0x2aa;
            case 3: return 0x3ff;
            default: return 0;
            }
        case 3: return 0;     // RCU
        }
    }
    return 0; // RCU_LOW
}

static void analog_sample(int channel)
{
    int value = analog_read_pin(channel);
    int dest = (channel << 1) & 6;
    dev[DEV_ADDRAH + dest] = value >> 2;
    dev[DEV_ADDRAL + dest] = (value << 6) & 0xc0;
}

static void MCU_UpdateAnalog(uint64_t cycles)
{
    int ctrl = dev[DEV_ADCSR];
    int isscan = (ctrl & 16) != 0;
    if (ctrl & 0x20)
    {
        if (analog_end_time == 0)
            analog_end_time = cycles + 200;
        else if (analog_end_time < cycles)
        {
            if (isscan)
            {
                int base = ctrl & 4;
                for (int i = 0; i <= (ctrl & 3); i++)
                    analog_sample(base + i);
                analog_end_time = cycles + 200;
            }
            else
            {
                analog_sample(ctrl & 7);
                dev[DEV_ADCSR] &= ~0x20;
                analog_end_time = 0;
            }
            dev[DEV_ADCSR] |= 0x80;
            if (ctrl & 0x40)
                MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_ANALOG, 1);
        }
    }
    else
        analog_end_time = 0;
}

// P9DR read (ported from nukeykt mcu.cpp MCU_DeviceRead DEV_P9DR):
// cfg bit1 = 1 for SC-55mk2; input bits (dir=0) return cfg, output bits (dir=1) return latched.
static uint8_t dev_p9dr_read(void)
{
    uint8_t cfg = 2; // SC-55mk2 (not mk1, not sc155)
    uint8_t dir = dev[DEV_P9DDR];
    int val = cfg & (dir ^ 0xff);
    val |= dev[DEV_P9DR] & dir;
    return (uint8_t)val;
}

static void uart_post(uint8_t data)
{
    sm_uart_buffer[sm_uart_write_ptr] = data;
    sm_uart_write_ptr = (sm_uart_write_ptr + 1) % SM_UART_BUFFER_SIZE;
}

struct mcu_t mcu;

static uint8_t MCU_Read_impl(uint32_t address)
{
    // rom2 addressing (ported from nukeykt mcu.cpp): 512KB mapped across pages 1-4, 8-9, 14-15.
    uint32_t address_rom = address & 0x3ffff;
    if (address & 0x80000)
        address_rom |= 0x40000;
    uint32_t page = (address >> 16) & 0xf;
    uint32_t off = address & 0xffff;
    switch (page)
    {
    case 0:
        if (off < 0x8000) return rom1[off];
        if (off >= 0xff80)
        {
            uint32_t d = off & 0x7f;
            if (d >= 0x10 && d < 0x40) return frt_read(d);
            if (d >= 0x50 && d < 0x55) return frt2_read(d);
            if (d == DEV_RDR) return uart_rx_byte;
            if (d == DEV_P9DR) return dev_p9dr_read();
            if (d == DEV_ADCSR) { adf_rd = (dev[d] & 0x80) != 0; return dev[d]; }
            return dev[d];
        }
        if (off >= 0xe000 && off < 0xe040) return PCMDev_Read(off & 0x3f);
        if (off >= 0xec00 && off < 0xf000) return SM_SysRead(off & 0xff);
        if (off == 0xe402)
        {
            uint8_t t = (uint8_t)SM_GATrigger();
            MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_IRQ1, 0);
            return t;
        }
        if (off >= 0xfb80 && off < 0xff80 && (dev[DEV_RAME] & 0x80) != 0)
            return ram[(off - 0xfb80) & 0x3ff];
        if (off >= 0x8000) return sram[off & 0x7fff];
        return 0xff;
    case 1:
    case 2:
    case 3:
    case 4:
    case 8:
    case 9:
    case 14:
    case 15:
        return rom2[address_rom & (ROM2_SIZE - 1)];
    case 6:
        /* 方案A: extended RAM page (was unbacked). */
        return b_ram[off];
    case 10:
    case 11:
        /* SC-55mk2 (mcu_mk1==0): pages 10/11 mirror sram. */
        return sram[off & 0x7fff];
    default:
        return 0xff;
    }
}

long g_read_count = 0;

uint8_t MCU_Read(uint32_t address)
{
    uint8_t v = MCU_Read_impl(address);
    if (g_verbose && g_read_count < 3000000)
        vm_log("rc%ld c%llu addr=%08x val=%02x pc=%04x:%04x\n", g_read_count, (unsigned long long)mcu.cycles, (unsigned)address, (int)v, (int)mcu.cp, (int)mcu.pc);
    g_read_count++;
    if (g_verbose)
    {
        static long ddump = 0;
        static uint32_t last_pc[24];
        static int lpi = 0;
        last_pc[lpi & 23] = ((uint32_t)mcu.cp << 16) | mcu.pc;
        lpi++;
        if (address == 0x0000fd48 && mcu.cp == 0 && mcu.pc == 0x7c3f && ddump < 3)
        {
            ddump++;
            vm_log("=== divergence read at 0xfd48 pc=0x7c3f ===\n");
            for (int i = 0; i < 8; i++) vm_log("r%d=%04x\n", i, (int)mcu.r[i]);
            vm_log("sr=%04x cp=%02x pc=%04x br=%04x dp=%02x ep=%02x tp=%02x\n",
                    (int)mcu.sr, (int)mcu.cp, (int)mcu.pc, (int)mcu.br, (int)mcu.dp, (int)mcu.ep, (int)mcu.tp);
            vm_log("last 24 PCs (oldest first):\n");
            int start = (lpi - 24) & 23;
            if (lpi < 24) start = 0;
            for (int i = 0; i < (lpi < 24 ? lpi : 24); i++)
                vm_log("  %08x\n", last_pc[(start + i) & 23]);
        }
    }
    return v;
}

uint16_t MCU_Read16(uint32_t address)
{
    address &= ~1u;
    return (MCU_Read(address) << 8) | MCU_Read(address + 1);
}

uint32_t MCU_Read32(uint32_t address)
{
    address &= ~3u;
    return ((uint32_t)MCU_Read(address) << 24) | ((uint32_t)MCU_Read(address + 1) << 16) |
           ((uint32_t)MCU_Read(address + 2) << 8) | (uint32_t)MCU_Read(address + 3);
}

void MCU_Write(uint32_t address, uint8_t value)
{
    uint32_t page = (address >> 16) & 0xf;
    uint32_t off = address & 0xffff;
    if (page == 0)
    {
        if (off >= 0xff80)
        {
            uint32_t d = off & 0x7f;
            if (d >= 0x10 && d < 0x40) { frt_write(d, value); return; }
            if (d >= 0x50 && d < 0x55) { frt2_write(d, value); return; }
            if (d == DEV_ADCSR)
            {
                dev[d] &= ~0x7f;
                dev[d] |= value & 0x7f;
                if ((value & 0x80) == 0 && adf_rd)
                {
                    dev[d] &= ~0x80;
                    MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_ANALOG, 0);
                }
                if ((value & 0x40) == 0)
                    MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_ANALOG, 0);
                return;
            }
            dev[d] = value;
            return;
        }
        if (off >= 0xe400 && off < 0xe800)
        {
            if (off == 0xe404 || off == 0xe405) LCD_Write(off & 1, value);
            else if (off == 0xe401) { io_sd = value; LCD_Enable((value & 1) == 0); }
            else if (off == 0xe402) ga_int_enable = (value << 1);
            return;
        }
        if (off >= 0xe000 && off < 0xe400) { PCMDev_Write(off & 0x3f, value); return; }
        if (off >= 0xec00 && off < 0xf000) { SM_SysWrite(off & 0xff, value); return; }
        if (off >= 0xfb80 && off < 0xff80 && (dev[DEV_RAME] & 0x80) != 0) { ram[(off - 0xfb80) & 0x3ff] = value; return; }
        if (off >= 0x8000) {
            uint32_t sa = off & 0x7fff;
            if (sa >= 0x5c00 && sa < 0x5d30 && g_verbose)
            {
                static long wc = 0;
                vm_log("sramw off=%04x val=%02x pc=%04x:%04x step%ld\n",
                        (int)sa, (int)value, (int)mcu.cp, (int)mcu.pc, wc++);
            }
            sram[sa] = value;
            return;
        }
        // rom1 (0x0000-0x7fff) is read-only; ignore writes.
    }
    else if (page == 10 || page == 11)
    {
        /* SC-55mk2 (mcu_mk1==0): pages 10/11 mirror sram. */
        sram[off & 0x7fff] = value;
    }
    else if (page == 6)
    {
        /* 方案A: extended RAM page (was unbacked). */
        b_ram[off] = value;
    }
    // rom2 is read-only; ignore writes there.
}

void MCU_Write16(uint32_t address, uint16_t value)
{
    address &= ~1u;
    MCU_Write(address, (value >> 8) & 0xff);
    MCU_Write(address + 1, value & 0xff);
}

void MCU_Interrupt_Exception(uint32_t exception)
{
    (void)exception;
    // Permissive: ignore exceptions (rare in the boot path).
}

static long irq1_req_count = 0;
void MCU_Interrupt_SetRequest(uint32_t interrupt, uint32_t value)
{
    mcu.interrupt_pending[interrupt] = value;
    if (interrupt == INTERRUPT_SOURCE_IRQ1 && value)
        irq1_req_count++;
}

static void MCU_Interrupt_Start(int32_t mask)
{
    MCU_PushStack(mcu.pc);
    MCU_PushStack(mcu.cp);
    MCU_PushStack(mcu.sr);
    mcu.sr &= ~STATUS_T;
    if (mask >= 0)
    {
        mcu.sr &= ~STATUS_INT_MASK;
        mcu.sr |= mask << 8;
    }
    mcu.sleep = 0;
    mcu.sleep = 0;
}

static void MCU_Interrupt_StartVector(uint32_t vector, int32_t mask)
{
    uint32_t address = MCU_GetVectorAddress(vector);
    MCU_Interrupt_Start(mask);
    mcu.cp = address >> 16;
    mcu.pc = address & 0xffff;
}

void PCMDev_OnIRQDeassert(void)
{
    // PCM voice-done IRQ (IRQ0, non-jv880) is deasserted by reading 0x3e.
    MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_IRQ0, 0);
}

void MCU_Interrupt_Handle(void)
{
    int32_t i;
    for (i = 0; i < 16; i++)
    {
        if (mcu.trapa_pending[i])
        {
            mcu.trapa_pending[i] = 0;
            MCU_Interrupt_StartVector(VECTOR_TRAPA_0 + i, -1);
            return;
        }
    }
    if (mcu.exception_pending >= 0)
    {
        switch (mcu.exception_pending)
        {
        case EXCEPTION_SOURCE_ADDRESS_ERROR:
            MCU_Interrupt_StartVector(VECTOR_ADDRESS_ERROR, -1);
            break;
        case EXCEPTION_SOURCE_INVALID_INSTRUCTION:
            MCU_Interrupt_StartVector(VECTOR_INVALID_INSTRUCTION, -1);
            break;
        case EXCEPTION_SOURCE_TRACE:
            MCU_Interrupt_StartVector(VECTOR_TRACE, -1);
            break;
        }
        mcu.exception_pending = -1;
        return;
    }
    if (mcu.interrupt_pending[INTERRUPT_SOURCE_NMI])
    {
        MCU_Interrupt_StartVector(VECTOR_NMI, 7);
        return;
    }
    int32_t mask = (mcu.sr >> 8) & 7;
    for (i = INTERRUPT_SOURCE_NMI + 1; i < INTERRUPT_SOURCE_MAX; i++)
    {
        int32_t vector = -1;
        int32_t level = 0;
        if (!mcu.interrupt_pending[i]) continue;
        switch (i)
        {
        case INTERRUPT_SOURCE_IRQ0: vector = VECTOR_IRQ0; level = (dev[DEV_IPRA] >> 4) & 7; break;
        case INTERRUPT_SOURCE_IRQ1: vector = VECTOR_IRQ1; level = (dev[DEV_IPRA] >> 0) & 7; break;
        case INTERRUPT_SOURCE_FRT0_OCIA: vector = VECTOR_INTERNAL_INTERRUPT_94; level = (dev[DEV_IPRB] >> 4) & 7; break;
        case INTERRUPT_SOURCE_FRT0_OCIB: vector = VECTOR_INTERNAL_INTERRUPT_98; level = (dev[DEV_IPRB] >> 4) & 7; break;
        case INTERRUPT_SOURCE_FRT0_FOVI: vector = VECTOR_INTERNAL_INTERRUPT_9C; level = (dev[DEV_IPRB] >> 4) & 7; break;
        case INTERRUPT_SOURCE_FRT1_OCIA: vector = VECTOR_INTERNAL_INTERRUPT_A4; level = (dev[DEV_IPRB] >> 0) & 7; break;
        case INTERRUPT_SOURCE_FRT1_OCIB: vector = VECTOR_INTERNAL_INTERRUPT_A8; level = (dev[DEV_IPRB] >> 0) & 7; break;
        case INTERRUPT_SOURCE_FRT1_FOVI: vector = VECTOR_INTERNAL_INTERRUPT_AC; level = (dev[DEV_IPRB] >> 0) & 7; break;
        case INTERRUPT_SOURCE_FRT2_OCIA: vector = VECTOR_INTERNAL_INTERRUPT_B4; level = (dev[DEV_IPRC] >> 4) & 7; break;
        case INTERRUPT_SOURCE_FRT2_OCIB: vector = VECTOR_INTERNAL_INTERRUPT_B8; level = (dev[DEV_IPRC] >> 4) & 7; break;
        case INTERRUPT_SOURCE_FRT2_FOVI: vector = VECTOR_INTERNAL_INTERRUPT_BC; level = (dev[DEV_IPRC] >> 4) & 7; break;
        case INTERRUPT_SOURCE_TIMER_CMIA: vector = VECTOR_INTERNAL_INTERRUPT_C0; level = (dev[DEV_IPRC] >> 0) & 7; break;
        case INTERRUPT_SOURCE_TIMER_CMIB: vector = VECTOR_INTERNAL_INTERRUPT_C4; level = (dev[DEV_IPRC] >> 0) & 7; break;
        case INTERRUPT_SOURCE_TIMER_OVI: vector = VECTOR_INTERNAL_INTERRUPT_C8; level = (dev[DEV_IPRC] >> 0) & 7; break;
        case INTERRUPT_SOURCE_ANALOG: vector = VECTOR_INTERNAL_INTERRUPT_E0; level = (dev[DEV_IPRD] >> 0) & 7; break;
        case INTERRUPT_SOURCE_UART_RX: vector = VECTOR_INTERNAL_INTERRUPT_D4; level = (dev[DEV_IPRD] >> 4) & 7; break;
        case INTERRUPT_SOURCE_UART_TX: vector = VECTOR_INTERNAL_INTERRUPT_D8; level = (dev[DEV_IPRD] >> 4) & 7; break;
        default: break;
        }
        if ((int32_t)mask < level)
        {
            MCU_Interrupt_StartVector(vector, level);
            return;
        }
    }
}

void MCU_Interrupt_TRAPA(uint32_t vector)
{
    mcu.trapa_pending[vector] = 1;
}

static char frt_state_prev[512];
static void frt_state_log(void)
{
    char buf[512];
    int pbits = 0;
    for (int i = 0; i < INTERRUPT_SOURCE_MAX; i++)
        if (mcu.interrupt_pending[i])
            pbits |= (1 << i);
    int n = snprintf(buf, sizeof buf, "c%llu f0=%02x%02x%04x%04x%04x f1=%02x%02x%04x%04x%04x f2=%02x%02x%04x%04x%04x t=%02x%02x%02x sr=%04x p=%05x\n",
        (unsigned long long)mcu.cycles,
        frt[0].tcr, frt[0].tcsr, frt[0].frc, frt[0].ocra, frt[0].ocrb,
        frt[1].tcr, frt[1].tcsr, frt[1].frc, frt[1].ocra, frt[1].ocrb,
        frt[2].tcr, frt[2].tcsr, frt[2].frc, frt[2].ocra, frt[2].ocrb,
        timer.tcr, timer.tcsr, timer.tcnt, mcu.sr, (unsigned)pbits);
    if (strcmp(buf, frt_state_prev) != 0)
    {
        strcpy(frt_state_prev, buf);
        if (g_verbose) fputs(buf, g_log);
    }
}

static int error_trap_count = 0;
void MCU_ErrorTrap(void)
{
    if (error_trap_count < 16)
    {
        error_trap_count++;
        printf("  ERROR_TRAP at cp=%02x pc=%04x\n", mcu.cp, mcu.pc);
    }
}

// ---- trace state ----
static long trace_count = 0;
static long trace_limit;
static int trace_regs = 1; // print r0-r7 + br/dp each instruction

static void trace_line(void)
{
    if (trace_count >= trace_limit) return;
    trace_count++;
    if (trace_regs)
        vm_log("m %llu %02x:%04x sr=%04x r0=%04x r1=%04x r2=%04x r3=%04x r4=%04x r5=%04x r6=%04x r7=%04x br=%02x dp=%02x ep=%02x tp=%02x\n",
            (unsigned long long)mcu.cycles, mcu.cp, mcu.pc, mcu.sr, mcu.r[0], mcu.r[1], mcu.r[2], mcu.r[3],
            mcu.r[4], mcu.r[5], mcu.r[6], mcu.r[7], mcu.br, mcu.dp, mcu.ep, mcu.tp);
    else
        vm_log("m %llu %02x:%04x\n", (unsigned long long)mcu.cycles, mcu.cp, mcu.pc);
}

static void MCU_ReadInstruction(void)
{
    {
        static long pn = 0;
        if (g_verbose && g_read_count >= 144000 && g_read_count < 145500)
            vm_log("instr step%ld rc%ld pc=%04x:%04x\n", pn, g_read_count, (int)mcu.cp, (int)mcu.pc);
        pn++;
    }
    uint8_t operand = MCU_ReadCodeAdvance();
    MCU_Operand_Table[operand](operand);
    if (mcu.sr & STATUS_T)
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_TRACE);
}

void MCU_Reset(void)
{
    memset(&mcu, 0, sizeof(mcu_t));
    mcu.sr = 0x700;
    uint32_t reset_address = MCU_GetVectorAddress(0); // VECTOR_RESET
    mcu.cp = (reset_address >> 16) & 0xff;
    mcu.pc = reset_address & 0xffff;
    mcu.exception_pending = -1;
}

// ---- Full machine-state snapshot (main MCU + SM sub-CPU + PCM device) ----
// Byte-for-byte serializable so the VM can resume from a saved point with the
// same per-cycle behavior as the original emulator.
// Byte-for-byte compatible with the reference emulator's state_save/state_load
// (src/mcu.cpp), so the VM can load demo_postW.bin directly with no conversion.
void VM_SaveState(FILE *f)
{
    fwrite(&mcu, sizeof mcu, 1, f);
    fwrite(ram, sizeof ram, 1, f);
    fwrite(sram, SRAM_SIZE, 1, f);
    fwrite(dev, 0x80, 1, f);
    fwrite(frt, sizeof frt, 1, f);
    fwrite(&timer, sizeof timer, 1, f);
    fwrite(&timer_cycles, sizeof timer_cycles, 1, f);
    fwrite(&timer_tempreg, 1, 1, f);
    fwrite(&mcu_p0_data, 1, 1, f);
    fwrite(&mcu_p1_data, 1, 1, f);
    fwrite(&io_sd, 1, 1, f);
    fwrite(&sw_pos, 1, 1, f);
    fwrite(ad_val, sizeof ad_val, 1, f);
    fwrite(&ad_nibble, 1, 1, f);
    fwrite(ga_int, sizeof ga_int, 1, f);
    fwrite(&ga_int_enable, 4, 1, f);
    fwrite(&ga_int_trigger, 4, 1, f);
    fwrite(&ga_lcd_counter, 4, 1, f);
    fwrite(&analog_end_time, sizeof analog_end_time, 1, f);
    fwrite(sm_uart_buffer, SM_UART_BUFFER_SIZE, 1, f);
    fwrite(&sm_uart_write_ptr, 4, 1, f);
    fwrite(&sm_uart_read_ptr, 4, 1, f);
    fwrite(&uart_rx_byte, 1, 1, f);
    fwrite(&sm, sizeof sm, 1, f);
    fwrite(sm_ram, sizeof sm_ram, 1, f);
    fwrite(sm_shared_ram, sizeof sm_shared_ram, 1, f);
    fwrite(sm_access, sizeof sm_access, 1, f);
    fwrite(&sm_p0_dir, 1, 1, f);
    fwrite(&sm_p1_dir, 1, 1, f);
    fwrite(sm_device_mode, sizeof sm_device_mode, 1, f);
    fwrite(&sm_cts, 1, 1, f);
    fwrite(&sm_timer_cycles, sizeof sm_timer_cycles, 1, f);
    fwrite(&sm_timer_prescaler, 1, 1, f);
    fwrite(&sm_timer_counter, 1, 1, f);
    fwrite(&pcmdev, sizeof pcmdev, 1, f);
    fwrite(&LCD_DL, 4, 1, f);
    fwrite(&LCD_N, 4, 1, f);
    fwrite(&LCD_F, 4, 1, f);
    fwrite(&LCD_D, 4, 1, f);
    fwrite(&LCD_C, 4, 1, f);
    fwrite(&LCD_B, 4, 1, f);
    fwrite(&LCD_ID, 4, 1, f);
    fwrite(&LCD_S, 4, 1, f);
    fwrite(&LCD_DD_RAM, 4, 1, f);
    fwrite(&LCD_AC, 4, 1, f);
    fwrite(&LCD_CG_RAM, 4, 1, f);
    fwrite(&LCD_RAM_MODE, 4, 1, f);
    fwrite(LCD_Data, sizeof LCD_Data, 1, f);
    fwrite(LCD_CG, sizeof LCD_CG, 1, f);
    fwrite(&lcd_enable, 1, 1, f);
}

void VM_LoadState(FILE *f)
{
    fread(&mcu, sizeof mcu, 1, f);
    fread(ram, sizeof ram, 1, f);
    fread(sram, SRAM_SIZE, 1, f);
    fread(dev, 0x80, 1, f);
    fread(frt, sizeof frt, 1, f);
    fread(&timer, sizeof timer, 1, f);
    fread(&timer_cycles, sizeof timer_cycles, 1, f);
    fread(&timer_tempreg, 1, 1, f);
    fread(&mcu_p0_data, 1, 1, f);
    fread(&mcu_p1_data, 1, 1, f);
    fread(&io_sd, 1, 1, f);
    fread(&sw_pos, 1, 1, f);
    fread(ad_val, sizeof ad_val, 1, f);
    fread(&ad_nibble, 1, 1, f);
    fread(ga_int, sizeof ga_int, 1, f);
    fread(&ga_int_enable, 4, 1, f);
    fread(&ga_int_trigger, 4, 1, f);
    fread(&ga_lcd_counter, 4, 1, f);
    fread(&analog_end_time, sizeof analog_end_time, 1, f);
    fread(sm_uart_buffer, SM_UART_BUFFER_SIZE, 1, f);
    fread(&sm_uart_write_ptr, 4, 1, f);
    fread(&sm_uart_read_ptr, 4, 1, f);
    fread(&uart_rx_byte, 1, 1, f);
    fread(&sm, sizeof sm, 1, f);
    fread(sm_ram, sizeof sm_ram, 1, f);
    fread(sm_shared_ram, sizeof sm_shared_ram, 1, f);
    fread(sm_access, sizeof sm_access, 1, f);
    fread(&sm_p0_dir, 1, 1, f);
    fread(&sm_p1_dir, 1, 1, f);
    fread(sm_device_mode, sizeof sm_device_mode, 1, f);
    fread(&sm_cts, 1, 1, f);
    fread(&sm_timer_cycles, sizeof sm_timer_cycles, 1, f);
    fread(&sm_timer_prescaler, 1, 1, f);
    fread(&sm_timer_counter, 1, 1, f);
    fread(&pcmdev, sizeof pcmdev, 1, f);
    fread(&LCD_DL, 4, 1, f);
    fread(&LCD_N, 4, 1, f);
    fread(&LCD_F, 4, 1, f);
    fread(&LCD_D, 4, 1, f);
    fread(&LCD_C, 4, 1, f);
    fread(&LCD_B, 4, 1, f);
    fread(&LCD_ID, 4, 1, f);
    fread(&LCD_S, 4, 1, f);
    fread(&LCD_DD_RAM, 4, 1, f);
    fread(&LCD_AC, 4, 1, f);
    fread(&LCD_CG_RAM, 4, 1, f);
    fread(&LCD_RAM_MODE, 4, 1, f);
    fread(LCD_Data, sizeof LCD_Data, 1, f);
    fread(LCD_CG, sizeof LCD_CG, 1, f);
    fread(&lcd_enable, 1, 1, f);
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printf("usage: h8vm.exe rom1.bin rom2.bin romsm.bin [limit] [noregs] [verbose] [waverom1 waverom2]\n");
        printf("       [savesnap file cycle] [loadsnap file]  (loadsnap = load full state at start)\n");
        printf("       [tracepc file [start end]]  (log 'cycles cp:pc' for every instr in [start,end), default 200M..210M)\n");
        printf("       [demo]  (apply the built-in Q/R/T/W key sequence to start the built-in demo;\n");
        printf("                off by default so a loadsnap of a demo run starts in the exact saved state)\n");
        printf("       [mocknote]  (post a MIDI note-on 0x90,60,100 into the shared UART at 144M cycles\n");
        printf("                (6s), aligned with the [demo] start, to exercise the SM UART RX path;\n");
        printf("                off by default; only effective in from-reset runs)\n");
        return 1;
    }
    FILE *f = fopen(argv[1], "rb");
    if (!f || fread(rom1, 1, ROM1_SIZE, f) != ROM1_SIZE) return 2;
    fclose(f);
    f = fopen(argv[2], "rb");
    if (!f || fread(rom2, 1, ROM2_SIZE, f) != ROM2_SIZE) return 3;
    fclose(f);
    f = fopen(argv[3], "rb");
    if (!f || fread(sm_rom, 1, ROMSM_SIZE, f) != ROMSM_SIZE) return 4;
    fclose(f);

    // Optional extra args: [waverom1] [waverom2] [limit] [noregs]
    trace_limit = 500000;
    trace_regs = 1;
    const char *wav1 = 0, *wav2 = 0;
    const char *save_file = 0;
    uint64_t save_at = 0;
    int save_done = 0;
    const char *load_file = 0;
    const char *tracepc_file = 0;
    int tracepc_enabled = 0;
    uint64_t tracepc_start = 200000000, tracepc_end = 210000000;
    int demo_keys_enabled = 0;
    int mocknote_enabled = 0;
    int mocknote_done = 0;
    for (int i = 4; i < argc; i++)
    {
        if (strcmp(argv[i], "noregs") == 0)
            trace_regs = 0;
        else if (strcmp(argv[i], "savesnap") == 0 && i + 2 < argc)
            { save_file = argv[++i]; save_at = strtoull(argv[++i], 0, 10); }
        else if (strcmp(argv[i], "loadsnap") == 0 && i + 1 < argc)
            { load_file = argv[++i]; }
        else if (strcmp(argv[i], "tracepc") == 0 && i + 1 < argc)
        {
            tracepc_file = argv[++i]; tracepc_enabled = 1;
            if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9')
                tracepc_start = strtoull(argv[++i], 0, 10);
            if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9')
                tracepc_end = strtoull(argv[++i], 0, 10);
        }
        else if (strcmp(argv[i], "demo") == 0)
            demo_keys_enabled = 1;
        else if (strcmp(argv[i], "mocknote") == 0)
            mocknote_enabled = 1;
        else if (strcmp(argv[i], "verbose") == 0)
            g_verbose = 1;
        else if (argv[i][0] >= '0' && argv[i][0] <= '9')
            trace_limit = strtol(argv[i], 0, 10);
        else if (!wav1)
            wav1 = argv[i];
        else
            wav2 = argv[i];
    }

    PCMDev_Reset();
    PCMDev_LoadWaveRoms(wav1, wav2);

    MCU_Reset();
    SM_Reset();

    // note-on is posted at 144M cycles (6s) in the run loop below (aligned with
    // the demo start; the LCD splash animation takes ~5s and no sound is output
    // until it finishes, so a reset-time note would be lost).

    // SC-55mk2 built-in demo, no external MIDI. Applied only when the [demo]
    // flag is given (off by default: a `loadsnap` of a mid-demo snapshot should
    // start in the exact saved state, key latches included, without re-driving
    // the boot key sequence). Key sequence (verified in the original emulator,
    // mirrored in machine time; 1 instruction = 12 cycles):
    //   machine auto-powers-on at boot -> Q (power off) -> hold R+T (Part< +
    //   Part>) -> Q again (power on with RT held = demo-mode standby) -> release
    //   all -> W (INST ALL) starts the demo. Each state is held 250ms.
    extern void VM_SetButtons(uint32_t mask);
    enum { VM_BTN_POWER = 1u << 0, VM_BTN_INST_ALL = 1u << 6,
           VM_BTN_PART_R = 1u << 14, VM_BTN_PART_L = 1u << 22 };
    typedef struct { long inst; uint32_t mask; } demo_step_t;
    static const demo_step_t demo_seq[] = {
        { 12000000, VM_BTN_POWER },
        { 12500000, VM_BTN_PART_L | VM_BTN_PART_R },
        { 13000000, VM_BTN_PART_L | VM_BTN_PART_R },
        { 13500000, VM_BTN_POWER | VM_BTN_PART_L | VM_BTN_PART_R },
        { 14000000, 0 },
        { 16000000, VM_BTN_INST_ALL },
        { 16500000, 0 },
    };
    int demo_step = 0;

    printf("# H8/532 VM trace, limit=%ld\n", trace_limit);
    printf("# reset -> cp=%02x pc=%04x\n", mcu.cp, mcu.pc);

    // loadsnap <file> = load the full saved state at the very start (before any
    // instruction), matching the ground truth's -loadsnap (ex- -demo2).
    if (load_file)
    {
        FILE *lf = fopen(load_file, "rb");
        if (lf) { VM_LoadState(lf); fclose(lf); printf("# loaded %s at start\n", load_file); }
        else printf("# load: cannot open %s\n", load_file);
    }

    // Per-instruction log written directly from C (buffered, no stdout). Skipped
    // when tracepc is on (that writes the unified main+SM trace instead).
    g_log = tracepc_enabled ? NULL : fopen("vm.log", "w");
    g_log_buf = malloc(1 << 20);
    if (g_log && g_log_buf) setvbuf(g_log, g_log_buf, _IOFBF, 1 << 20);

    long st_n = 0;
    for (long i = 0; i < trace_limit; i++)
    {
        if (demo_keys_enabled)
            while (demo_step < 7 && i >= demo_seq[demo_step].inst)
                VM_SetButtons(demo_seq[demo_step++].mask);
        if (g_verbose && mcu.cp == 0 && mcu.pc == 0x7c3f)
        {
            vm_log("burst%ld r7=%04x prepop: ", st_n++, mcu.r[7]);
            for (int s = 0; s < 16; s++)
                vm_log("%02x%02x ", sram[(mcu.r[7] + s * 2) & 0x7fff], sram[(mcu.r[7] + s * 2 + 1) & 0x7fff]);
            vm_log("\n");
            if (st_n <= 32)
            {
                vm_log("=== burst%ld sram ===\n", st_n - 1);
                for (int b = 0; b < 0x8000; b += 16)
                {
                    for (int k = 0; k < 16; k++)
                        vm_log("%02x", sram[b + k]);
                    vm_log("\n");
                }
            }
        }
        if (!mcu.ex_ignore)
            MCU_Interrupt_Handle();
        else
            mcu.ex_ignore = 0;
        if (!mcu.sleep)
            MCU_ReadInstruction();
        mcu.cycles += 12;
        // log after the increment so the m-line cycle matches the tracepc
        // m-line (both report the instruction's completion cycle, post-fetch pc).
        trace_line();
        if (tracepc_enabled)
        {
            if (mcu.cycles >= tracepc_start && mcu.cycles < tracepc_end)
            {
                if (!g_trace_f && !g_trace_closed)
                {
                    g_trace_f = fopen(tracepc_file, "w");
                    if (g_trace_f) { g_trace_buf = (char *)malloc(1 << 20); setvbuf(g_trace_f, g_trace_buf, _IOFBF, 1 << 20); }
                }
                trace_write(0, mcu.cycles, ((uint32_t)mcu.cp << 16) | mcu.pc);
            }
            else if (mcu.cycles >= tracepc_end && g_trace_f && !g_trace_closed)
            {
                fflush(g_trace_f);
                fclose(g_trace_f);
                g_trace_f = NULL;
                g_trace_closed = 1;
            }
        }
        if (save_file && !save_done && mcu.cycles >= save_at)
        {
            FILE *sf = fopen(save_file, "wb");
            if (sf) { VM_SaveState(sf); fclose(sf); printf("# saved %s at c%llu\n", save_file, (unsigned long long)mcu.cycles); }
            else printf("# save: cannot open %s\n", save_file);
            save_done = 1;
        }
        // mock note-on ([mocknote]): fire at 144M cycles (6s), aligned with the
        // demo start (the LCD splash animation takes ~5s and no sound is output
        // until it finishes, so a reset-time note would be lost).
        if (mocknote_enabled && !mocknote_done && mcu.cycles >= 144000000ull)
        {
            uart_post(0x90);
            uart_post(60);
            uart_post(100);
            mocknote_done = 1;
            printf("# mocknote: posted note-on at c%llu\n", (unsigned long long)mcu.cycles);
        }
        PCMDev_Update(mcu.cycles);
        TIMER_Clock(mcu.cycles);
        SM_Update(mcu.cycles);
        MCU_UpdateAnalog(mcu.cycles);
        if (g_verbose) frt_state_log();
    }
    if (g_log) { fflush(g_log); fclose(g_log); }
    free(g_log_buf);
    if (g_trace_f) { fflush(g_trace_f); fclose(g_trace_f); }
    free(g_trace_buf);
    printf("# done, %ld instructions\n", trace_count);
    printf("# SM: read_ptr=%u write_ptr=%u sem=%02x intreq=%02x uart1_ctrl=%02x sm_pc=%.4x sm_sleep=%d sm_sr=%02x\n",
        sm_uart_read_ptr, sm_uart_write_ptr,
        sm_device_mode[0x19], sm_device_mode[0x1c], sm_device_mode[0x06], sm.pc, sm.sleep, sm.sr);
    printf("# SM dev: timer_ctrl=%02x int_enable=%02x uart2_ctrl=%02x uart2_ms=%02x presc=%02x timer=%02x\n",
        sm_device_mode[0x1f], sm_device_mode[0x1b], sm_device_mode[0x0a],
        sm_device_mode[0x09], sm_device_mode[0x1d], sm_device_mode[0x1e]);
    printf("# SM ram[0..5]=%02x %02x %02x %02x %02x %02x  sm.cycles=%lu\n",
        sm_ram[0], sm_ram[1], sm_ram[2], sm_ram[3], sm_ram[4], sm_ram[5], sm.cycles);
    printf("# MAIN: sr=%04x irq1_pending=%d irq1_req_count=%ld\n", mcu.sr, mcu.interrupt_pending[2], irq1_req_count);
    return 0;
}
