/*
 * Copyright (C) 2021, 2024 nukeykt
 *
 * This file is part of Nuked-SC55.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define SDL_MAIN_HANDLED
#include "SDL.h"
#include "mcu.h"
#include "mk2cpp.h"
#include "mcu_opcodes.h"
#include "mcu_interrupt.h"
#include "mcu_timer.h"
#include "pcm.h"
#include "lcd.h"
#include "submcu.h"
#include "midi.h"
#include "utf8main.h"
#include "utils/files.h"

#if __linux__
#include <unistd.h>
#include <limits.h>
#endif

extern frt_t frt[3];
extern mcu_timer_t timer;
extern uint64_t timer_cycles;
extern uint8_t timer_tempreg;

// Set to 1 to enable trace/observation logging (slows the emulator down).
#define SC55_TRACE 0

#if SC55_TRACE
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
        static FILE *f = nullptr;
        if (!f) f = fopen("frtstate_orig.log", "a");
        if (f)
        {
            fwrite(buf, 1, (size_t)n, f);
            fflush(f);
        }
    }
}
#endif

const char* rs_name[ROM_SET_COUNT] = {
    "SC-55mk2",
    "SC-55st",
    "SC-55mk1",
    "CM-300/SCC-1",
    "JV-880",
    "SCB-55",
    "RLP-3237",
    "SC-155",
    "SC-155mk2"
};

static const int ROM_SET_N_FILES = 6;

const char* roms[ROM_SET_COUNT][ROM_SET_N_FILES] =
{
    "rom1.bin",
    "rom2.bin",
    "waverom1.bin",
    "waverom2.bin",
    "rom_sm.bin",
    "",

    "rom1.bin",
    "rom2_st.bin",
    "waverom1.bin",
    "waverom2.bin",
    "rom_sm.bin",
    "",

    "sc55_rom1.bin",
    "sc55_rom2.bin",
    "sc55_waverom1.bin",
    "sc55_waverom2.bin",
    "sc55_waverom3.bin",
    "",

    "cm300_rom1.bin",
    "cm300_rom2.bin",
    "cm300_waverom1.bin",
    "cm300_waverom2.bin",
    "cm300_waverom3.bin",
    "",

    "jv880_rom1.bin",
    "jv880_rom2.bin",
    "jv880_waverom1.bin",
    "jv880_waverom2.bin",
    "jv880_waverom_expansion.bin",
    "jv880_waverom_pcmcard.bin",

    "scb55_rom1.bin",
    "scb55_rom2.bin",
    "scb55_waverom1.bin",
    "scb55_waverom2.bin",
    "",
    "",

    "rlp3237_rom1.bin",
    "rlp3237_rom2.bin",
    "rlp3237_waverom1.bin",
    "",
    "",
    "",

    "sc155_rom1.bin",
    "sc155_rom2.bin",
    "sc155_waverom1.bin",
    "sc155_waverom2.bin",
    "sc155_waverom3.bin",
    "",

    "rom1.bin",
    "rom2.bin",
    "waverom1.bin",
    "waverom2.bin",
    "rom_sm.bin",
    "",
};

int romset = ROM_SET_MK2;

static const int ROM1_SIZE = 0x8000;
static const int ROM2_SIZE = 0x80000;
static const int RAM_SIZE = 0x400;
static const int SRAM_SIZE = 0x8000;
static const int NVRAM_SIZE = 0x8000; // JV880 only
static const int CARDRAM_SIZE = 0x8000; // JV880 only
static const int ROMSM_SIZE = 0x1000;


static int audio_buffer_size;
static int audio_page_size;
static short *sample_buffer;

static int sample_read_ptr;
static int sample_write_ptr;

static SDL_AudioDeviceID sdl_audio;

void MCU_ErrorTrap(void)
{
    printf("%.2x %.4x\n", mcu.cp, mcu.pc);
}

int mcu_mk1 = 0; // 0 - SC-55mkII, SC-55ST. 1 - SC-55, CM-300/SCC-1
int mcu_cm300 = 0; // 0 - SC-55, 1 - CM-300/SCC-1
int mcu_st = 0; // 0 - SC-55mk2, 1 - SC-55ST
int mcu_jv880 = 0; // 0 - SC-55, 1 - JV880
int mcu_scb55 = 0; // 0 - sub mcu (e.g SC-55mk2), 1 - no sub mcu (e.g SCB-55)
int mcu_sc155 = 0; // 0 - SC-55(MK2), 1 - SC-155(MK2)
int pcm_float = 0; // pure-float resonant filter, all chip selects
float master_gain = 1.0f; // overall output volume multiplier (linear), applied in MCU_PostSample before int16 clamp

static int ga_int[8];
static int ga_int_enable = 0;
static int ga_int_trigger = 0;
static int ga_lcd_counter = 0;


uint8_t dev_register[0x80];

static uint16_t ad_val[4];
static uint8_t ad_nibble = 0x00;
static uint8_t sw_pos = 3;
static uint8_t io_sd = 0x00;

SDL_atomic_t mcu_button_pressed = { 0 };

// SC-55mk2 built-in demo key sequence (-demo [cycles]):
// Q (power on) -> hold RT (Part< + Part>) -> Q again (power cycle with RT held)
// -> release all -> W (INST_ALL) starts the demo song.
static bool demo_seq_enabled = false;
static uint64_t demo_start = 144000000;  // first-event cycle (default 144M, min 144M)

// (-mocknote [cycles]): post a MIDI note-on (0x90, note 60, vel 100) into the
// shared UART so the SM's UART RX path runs without a MIDI device. Off by
// default. Default cycle 144M (6s) �?after the ~5s LCD splash, so a reset-time
// note is not lost. The SC-55mk2 rejects MIDI while in the demo phase, so to
// take effect the note must fire BEFORE the -demo start. Only effective in
// from-reset runs (a -loadsnap overwrites the buffer; the timer restarts from
// the loaded cycles).
static bool mocknote_enabled = false;
static bool mocknote_done = false;
static uint64_t mocknote_at = 144000000;  // note cycle (default 144M, min 144M)

// (-tracepc <file> [start end]): log a unified main+SM PC trace for every
// instruction in the [start,end) window (default 200M..210M), to verify
// per-cycle PC parity between runs. One file; main lines are "m <cycles>
// <cp:pc>", SM lines are "s <sm_cycles> <pc>".
static bool tracepc_enabled = false;
static const char *tracepc_file = nullptr;
static uint64_t tracepc_start = 200000000, tracepc_end = 210000000;

// (-pcmtrace): log PCM control-register writes (voice_enable 0x00-03, config
// 0x3c/3d, select_channel 0x3e) with the H8 PC, to pcm_trace.log.
static bool g_pcm_trace = false;

// ---- unified PC trace (main + SM in ONE file), owned by the main MCU. ----
// type 0 (main): "m <mcu_cycles> <cp:pc>"; type 1 (SM): "s <sm_cycles> <pc>".
// The work thread opens/closes the file on the tracepc window; the SM calls
// trace_write() for every non-sleep instruction (no-ops when the file is closed).
static FILE *g_trace_f = nullptr;
static char *g_trace_buf = nullptr;
static bool g_trace_closed = false;
void trace_write(uint8_t type, uint64_t cycle, uint32_t value)
{
    if (!g_trace_f) return;
    if (type == 0)
        fprintf(g_trace_f, "m %llu %02x:%04x\n", (unsigned long long)cycle, (unsigned)((value >> 16) & 0xff), (unsigned)(value & 0xffff));
    else
        fprintf(g_trace_f, "s %llu %04x\n", (unsigned long long)cycle, (unsigned)(value & 0xffff));
}

// (-demo [cycles]) / (-mocknote [cycles]): parse a cycle count from an argument.
// Accepts a full decimal ("144000000") or hex ("0x8a12c00") cycle count.
static uint64_t parse_cycles(const char *s)
{
    return strtoull(s, nullptr, 0);
}

// True if an argument looks like a cycle count (starts with a digit) rather than
// a flag (starts with '-').
static bool is_cycle_arg(const char *s)
{
    return s && s[0] >= '0' && s[0] <= '9';
}

uint8_t RCU_Read(void)
{
    return 0;
}

enum {
    ANALOG_LEVEL_RCU_LOW = 0,
    ANALOG_LEVEL_RCU_HIGH = 0,
    ANALOG_LEVEL_SW_0 = 0,
    ANALOG_LEVEL_SW_1 = 0x155,
    ANALOG_LEVEL_SW_2 = 0x2aa,
    ANALOG_LEVEL_SW_3 = 0x3ff,
    ANALOG_LEVEL_BATTERY = 0x2a0,
};

uint16_t MCU_SC155Sliders(uint32_t index)
{
    // 0 - 1/9
    // 1 - 2/10
    // 2 - 3/11
    // 3 - 4/12
    // 4 - 5/13
    // 5 - 6/14
    // 6 - 7/15
    // 7 - 8/16
    // 8 - ALL
    return 0x0;
}

uint16_t MCU_AnalogReadPin(uint32_t pin)
{
    if (mcu_cm300)
        return 0;
    if (mcu_jv880)
    {
        if (pin == 1)
            return ANALOG_LEVEL_BATTERY;
        return 0x3ff;
    }
    if (0)
    {
READ_RCU:
        uint8_t rcu = RCU_Read();
        if (rcu & (1 << pin))
            return ANALOG_LEVEL_RCU_HIGH;
        else
            return ANALOG_LEVEL_RCU_LOW;
    }
    if (mcu_mk1)
    {
        if (mcu_sc155 && (dev_register[DEV_P9DR] & 1) != 0)
        {
            return MCU_SC155Sliders(pin);
        }
        if (pin == 7)
        {
            if (mcu_sc155 && (dev_register[DEV_P9DR] & 2) != 0)
                return MCU_SC155Sliders(8);
            else
                return ANALOG_LEVEL_BATTERY;
        }
        else
            goto READ_RCU;
    }
    else
    {
        if (mcu_sc155 && (io_sd & 16) != 0)
        {
            return MCU_SC155Sliders(pin);
        }
        if (pin == 7)
        {
            if (mcu_mk1)
                return ANALOG_LEVEL_BATTERY;
            switch ((io_sd >> 2) & 3)
            {
            case 0: // Battery voltage
                return ANALOG_LEVEL_BATTERY;
            case 1: // NC
                if (mcu_sc155)
                    return MCU_SC155Sliders(8);
                return 0;
            case 2: // SW
                switch (sw_pos)
                {
                case 0:
                default:
                    return ANALOG_LEVEL_SW_0;
                case 1:
                    return ANALOG_LEVEL_SW_1;
                case 2:
                    return ANALOG_LEVEL_SW_2;
                case 3:
                    return ANALOG_LEVEL_SW_3;
                }
            case 3: // RCU
                goto READ_RCU;
            }
        }
        else
            goto READ_RCU;
    }
}

void MCU_AnalogSample(int channel)
{
    int value = MCU_AnalogReadPin(channel);
    int dest = (channel << 1) & 6;
#if SC55_TRACE
    {
        static FILE *af = nullptr;
        if (!af) af = fopen("analog_orig.log", "a");
        if (af)
        {
            fprintf(af, "c%llu ch%d dest%d value=%04x io_sd=%02x sw_pos=%d adcsr=%02x\n",
                (unsigned long long)mcu.cycles, channel, dest, value, io_sd, sw_pos, dev_register[DEV_ADCSR]);
            fflush(af);
        }
    }
#endif
    dev_register[DEV_ADDRAH + dest] = value >> 2;
    dev_register[DEV_ADDRAL + dest] = (value << 6) & 0xc0;
}

int adf_rd = 0;

uint64_t analog_end_time;

int ssr_rd = 0;

uint32_t uart_write_ptr;
uint32_t uart_read_ptr;
uint8_t uart_buffer[uart_buffer_size];

static uint8_t uart_rx_byte;
static uint64_t uart_rx_delay;
static uint64_t uart_tx_delay;

void MCU_DeviceWrite(uint32_t address, uint8_t data)
{
    address &= 0x7f;
    if (address >= 0x10 && address < 0x40)
    {
        TIMER_Write(address, data);
        return;
    }
    if (address >= 0x50 && address < 0x55)
    {
        TIMER2_Write(address, data);
        return;
    }
    switch (address)
    {
    case DEV_P1DDR: // P1DDR
        break;
    case DEV_P5DDR:
        break;
    case DEV_P6DDR:
        break;
    case DEV_P7DDR:
        break;
    case DEV_SCR:
        break;
    case DEV_WCR:
        break;
    case DEV_P9DDR:
        break;
    case DEV_RAME: // RAME
        break;
    case DEV_P1CR: // P1CR
        break;
    case DEV_DTEA:
        break;
    case DEV_DTEB:
        break;
    case DEV_DTEC:
        break;
    case DEV_DTED:
        break;
    case DEV_SMR:
        break;
    case DEV_BRR:
        break;
    case DEV_IPRA:
        break;
    case DEV_IPRB:
        break;
    case DEV_IPRC:
        break;
    case DEV_IPRD:
        break;
    case DEV_PWM1_DTR:
        break;
    case DEV_PWM1_TCR:
        break;
    case DEV_PWM2_DTR:
        break;
    case DEV_PWM2_TCR:
        break;
    case DEV_PWM3_DTR:
        break;
    case DEV_PWM3_TCR:
        break;
    case DEV_P7DR:
        break;
    case DEV_TMR_TCNT:
        break;
    case DEV_TMR_TCR:
        break;
    case DEV_TMR_TCSR:
        break;
    case DEV_TMR_TCORA:
        break;
    case DEV_TDR:
        break;
    case DEV_ADCSR:
    {
        dev_register[address] &= ~0x7f;
        dev_register[address] |= data & 0x7f;
        if ((data & 0x80) == 0 && adf_rd)
        {
            dev_register[address] &= ~0x80;
            MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_ANALOG, 0);
        }
        if ((data & 0x40) == 0)
            MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_ANALOG, 0);
        return;
    }
    case DEV_SSR:
    {
        if ((data & 0x80) == 0 && (ssr_rd & 0x80) != 0)
        {
            dev_register[address] &= ~0x80;
            uart_tx_delay = mcu.cycles + 3000;
            MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_UART_TX, 0);
        }
        if ((data & 0x40) == 0 && (ssr_rd & 0x40) != 0)
        {
            uart_rx_delay = mcu.cycles + 3000;
            dev_register[address] &= ~0x40;
            MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_UART_RX, 0);
        }
        if ((data & 0x20) == 0 && (ssr_rd & 0x20) != 0)
        {
            dev_register[address] &= ~0x20;
        }
        if ((data & 0x10) == 0 && (ssr_rd & 0x10) != 0)
        {
            dev_register[address] &= ~0x10;
        }
        break;
    }
    default:
        address += 0;
        break;
    }
    dev_register[address] = data;
}

uint8_t MCU_DeviceRead(uint32_t address)
{
    address &= 0x7f;
    if (address >= 0x10 && address < 0x40)
    {
        return TIMER_Read(address);
    }
    if (address >= 0x50 && address < 0x55)
    {
        return TIMER_Read2(address);
    }
    switch (address)
    {
    case DEV_ADDRAH:
    case DEV_ADDRAL:
    case DEV_ADDRBH:
    case DEV_ADDRBL:
    case DEV_ADDRCH:
    case DEV_ADDRCL:
    case DEV_ADDRDH:
    case DEV_ADDRDL:
        return dev_register[address];
    case DEV_ADCSR:
        adf_rd = (dev_register[address] & 0x80) != 0;
        return dev_register[address];
    case DEV_SSR:
        ssr_rd = dev_register[address];
        return dev_register[address];
    case DEV_RDR:
        return uart_rx_byte;
    case 0x00:
        return 0xff;
    case DEV_P7DR:
    {
        if (!mcu_jv880) return 0xff;

        uint8_t data = 0xff;
        uint32_t button_pressed = (uint32_t)SDL_AtomicGet(&mcu_button_pressed);

        if (io_sd == 0b11111011)
            data &= ((button_pressed >> 0) & 0b11111) ^ 0xFF;
        if (io_sd == 0b11110111)
            data &= ((button_pressed >> 5) & 0b11111) ^ 0xFF;
        if (io_sd == 0b11101111)
            data &= ((button_pressed >> 10) & 0b1111) ^ 0xFF;

        data |= 0b10000000;
        return data;
    }
    case DEV_P9DR:
    {
        int cfg = 0;
        if (!mcu_mk1)
            cfg = mcu_sc155 ? 0 : 2; // bit 1: 0 - SC-155mk2 (???), 1 - SC-55mk2

        int dir = dev_register[DEV_P9DDR];

        int val = cfg & (dir ^ 0xff);
        val |= dev_register[DEV_P9DR] & dir;
        return val;
    }
    case DEV_SCR:
    case DEV_TDR:
    case DEV_SMR:
        return dev_register[address];
    case DEV_IPRC:
    case DEV_IPRD:
    case DEV_DTEC:
    case DEV_DTED:
    case DEV_FRT2_TCSR:
    case DEV_FRT1_TCSR:
    case DEV_FRT1_TCR:
    case DEV_FRT1_FRCH:
    case DEV_FRT1_FRCL:
    case DEV_FRT3_TCSR:
    case DEV_FRT3_OCRAH:
    case DEV_FRT3_OCRAL:
        return dev_register[address];
    }
    return dev_register[address];
}

void MCU_DeviceReset(void)
{
    // dev_register[0x00] = 0x03;
    // dev_register[0x7c] = 0x87;
    dev_register[DEV_RAME] = 0x80;
    dev_register[DEV_SSR] = 0x80;
}

void MCU_UpdateAnalog(uint64_t cycles)
{
    int ctrl = dev_register[DEV_ADCSR];
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
                    MCU_AnalogSample(base + i);
                analog_end_time = cycles + 200;
            }
            else
            {
                MCU_AnalogSample(ctrl & 7);
                dev_register[DEV_ADCSR] &= ~0x20;
                analog_end_time = 0;
            }
            dev_register[DEV_ADCSR] |= 0x80;
            if (ctrl & 0x40)
                MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_ANALOG, 1);
        }
    }
    else
        analog_end_time = 0;
}

mcu_t mcu;

uint8_t rom1[ROM1_SIZE];
uint8_t rom2[ROM2_SIZE];
uint8_t ram[RAM_SIZE];
uint8_t sram[SRAM_SIZE];
uint8_t nvram[NVRAM_SIZE];
uint8_t cardram[CARDRAM_SIZE];
int rom2_mask = ROM2_SIZE - 1;

static FILE *g_pcm_trace_f = nullptr;

static void pcm_trace_log(uint32_t reg, uint8_t val)
{
    if (!g_pcm_trace_f)
    {
        g_pcm_trace_f = fopen("pcm_trace.log", "w");
        if (g_pcm_trace_f)
            setvbuf(g_pcm_trace_f, NULL, _IONBF, 0);
    }
    if (!g_pcm_trace_f)
        return;
    fprintf(g_pcm_trace_f, "pcm reg=%02x val=%02x pc=%02x:%04x cyc=%llu\n",
            (int)reg, (int)val, (int)mcu.cp, (int)mcu.pc,
            (unsigned long long)mcu.cycles);
}

static uint8_t MCU_Read_impl(uint32_t address)
{
    uint32_t address_rom = address & 0x3ffff;
    if (address & 0x80000 && !mcu_jv880)
        address_rom |= 0x40000;
    uint8_t page = (address >> 16) & 0xf;
    address &= 0xffff;
    uint8_t ret = 0xff;
    switch (page)
    {
    case 0:
        if (!(address & 0x8000))
            ret = rom1[address & 0x7fff];
        else
        {
            if (!mcu_mk1)
            {
                uint16_t base = mcu_jv880 ? 0xf000 : 0xe000;
                if (address >= base && address < (base | 0x400))
                {
                    ret = PCM_Read(address & 0x3f);
                }
                else if (!mcu_scb55 && address >= 0xec00 && address < 0xf000)
                {
                    ret = SM_SysRead(address & 0xff);
                }
                else if (address >= 0xff80)
                {
                    ret = MCU_DeviceRead(address & 0x7f);
                }
                else if (address >= 0xfb80 && address < 0xff80
                    && (dev_register[DEV_RAME] & 0x80) != 0)
                    ret = ram[(address - 0xfb80) & 0x3ff];
                else if (address >= 0x8000 && address < 0xe000)
                {
                    ret = sram[address & 0x7fff];
                }
                else if (address == (base | 0x402))
                {
                    ret = ga_int_trigger;
                    ga_int_trigger = 0;
                    MCU_Interrupt_SetRequest(mcu_jv880 ? INTERRUPT_SOURCE_IRQ0 : INTERRUPT_SOURCE_IRQ1, 0);
                }
                else
                {
                    printf("Unknown read %x\n", address);
                    ret = 0xff;
                }
                //
                // e402:2-0 irq source
                //
            }
            else
            {
                if (address >= 0xe000 && address < 0xe040)
                {
                    ret = PCM_Read(address & 0x3f);
                }
                else if (address >= 0xff80)
                {
                    ret = MCU_DeviceRead(address & 0x7f);
                }
                else if (address >= 0xfb80 && address < 0xff80
                    && (dev_register[DEV_RAME] & 0x80) != 0)
                {
                    ret = ram[(address - 0xfb80) & 0x3ff];
                }
                else if (address >= 0x8000 && address < 0xe000)
                {
                    ret = sram[address & 0x7fff];
                }
                else if (address >= 0xf000 && address < 0xf100)
                {
                    io_sd = address & 0xff;

                    if (mcu_cm300)
                        return 0xff;

                    LCD_Enable((io_sd & 8) != 0);

                    uint8_t data = 0xff;
                    uint32_t button_pressed = (uint32_t)SDL_AtomicGet(&mcu_button_pressed);

                    if ((io_sd & 1) == 0)
                        data &= ((button_pressed >> 0) & 255) ^ 255;
                    if ((io_sd & 2) == 0)
                        data &= ((button_pressed >> 8) & 255) ^ 255;
                    if ((io_sd & 4) == 0)
                        data &= ((button_pressed >> 16) & 255) ^ 255;
                    if ((io_sd & 8) == 0)
                        data &= ((button_pressed >> 24) & 255) ^ 255;
                    return data;
                }
                else if (address == 0xf106)
                {
                    ret = ga_int_trigger;
                    ga_int_trigger = 0;
                    MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_IRQ1, 0);
                }
                else
                {
                    printf("Unknown read %x\n", address);
                    ret = 0xff;
                }
                //
                // f106:2-0 irq source
                //
            }
        }
        break;
#if 0
    case 3:
        ret = rom2[address | 0x30000];
        break;
    case 4:
        ret = rom2[address];
        break;
    case 10:
        ret = rom2[address | 0x60000]; // FIXME
        break;
    case 1:
        ret = rom2[address | 0x10000];
        break;
#endif
    case 1:
        ret = rom2[address_rom & rom2_mask];
        break;
    case 2:
        ret = rom2[address_rom & rom2_mask];
        break;
    case 3:
        ret = rom2[address_rom & rom2_mask];
        break;
    case 4:
        ret = rom2[address_rom & rom2_mask];
        break;
    case 8:
        if (!mcu_jv880)
            ret = rom2[address_rom & rom2_mask];
        else
            ret = 0xff;
        break;
    case 9:
        if (!mcu_jv880)
            ret = rom2[address_rom & rom2_mask];
        else
            ret = 0xff;
        break;
    case 14:
    case 15:
        if (!mcu_jv880)
            ret = rom2[address_rom & rom2_mask];
        else
            ret = cardram[address & 0x7fff]; // FIXME
        break;
    case 10:
    case 11:
        if (!mcu_mk1)
            ret = sram[address & 0x7fff]; // FIXME
        else
            ret = 0xff;
        break;
    case 12:
    case 13:
        if (mcu_jv880)
            ret = nvram[address & 0x7fff]; // FIXME
        else
            ret = 0xff;
        break;
    case 5:
        if (mcu_mk1)
            ret = sram[address & 0x7fff]; // FIXME
        else
            ret = 0xff;
        break;
    default:
        ret = 0x00;
        break;
    }
    return ret;
}

uint8_t MCU_Read(uint32_t address)
{
    uint8_t v = MCU_Read_impl(address);
#if SC55_TRACE
    {
        static FILE *rf = nullptr;
        static long rc = 0;
        if (rc < 3000000)
        {
            if (!rf) rf = fopen("read_orig.log", "a");
            if (rf) fprintf(rf, "rc%ld c%llu addr=%08x val=%02x pc=%04x:%04x\n", rc, (unsigned long long)mcu.cycles, (unsigned)address, (int)v, (int)mcu.cp, (int)mcu.pc);
            rc++;
        }
    }
#endif
    return v;
}

uint16_t MCU_Read16(uint32_t address)
{
    address &= ~1;
    uint8_t b0, b1;
    b0 = MCU_Read(address);
    b1 = MCU_Read(address+1);
    return (b0 << 8) + b1;
}

uint32_t MCU_Read32(uint32_t address)
{
    address &= ~3;
    uint8_t b0, b1, b2, b3;
    b0 = MCU_Read(address);
    b1 = MCU_Read(address+1);
    b2 = MCU_Read(address+2);
    b3 = MCU_Read(address+3);
    return (b0 << 24) + (b1 << 16) + (b2 << 8) + b3;
}

void MCU_Write(uint32_t address, uint8_t value)
{
    uint8_t page = (address >> 16) & 0xf;
    address &= 0xffff;
    if (page == 0)
    {
        if (address & 0x8000)
        {
            if (!mcu_mk1)
            {
                uint16_t base = mcu_jv880 ? 0xf000 : 0xe000;
                if (address >= (base | 0x400) && address < (base | 0x800))
                {
                    if (address == (base | 0x404) || address == (base | 0x405))
                        LCD_Write(address & 1, value);
                    else if (address == (base | 0x401))
                    {
                        io_sd = value;
                        LCD_Enable((value & 1) == 0);
                    }
                    else if (address == (base | 0x402))
                        ga_int_enable = (value << 1);
                    else
                        printf("Unknown write %x %x\n", address, value);
                    //
                    // e400: always 4?
                    // e401: SC0-6?
                    // e402: enable/disable IRQ?
                    // e403: always 1?
                    // e404: LCD
                    // e405: LCD
                    // e406: 0 or 40
                    // e407: 0, e406 continuation?
                    //
                }
                else if (address >= (base | 0x000) && address < (base | 0x400))
                {
                    if (g_pcm_trace)
                    {
                        uint32_t reg = address & 0x3f;
                        if ((reg <= 3) || reg == 0x3c || reg == 0x3d || reg == 0x3e)
                            pcm_trace_log(reg, value);
                    }
                    PCM_Write(address & 0x3f, value);
                }
                else if (!mcu_scb55 && address >= 0xec00 && address < 0xf000)
                {
                    SM_SysWrite(address & 0xff, value);
                }
                else if (address >= 0xff80)
                {
                    MCU_DeviceWrite(address & 0x7f, value);
                }
                else if (address >= 0xfb80 && address < 0xff80
                    && (dev_register[DEV_RAME] & 0x80) != 0)
                {
                    ram[(address - 0xfb80) & 0x3ff] = value;
                }
                else if (address >= 0x8000 && address < 0xe000)
                {
                    uint32_t sa = address & 0x7fff;
#if SC55_TRACE
                    if (sa >= 0x5c00 && sa < 0x5d30)
                    {
                        static FILE *wf = nullptr;
                        static long wc = 0;
                        if (!wf) wf = fopen("sramw_orig.log", "a");
                        if (wf)
                        {
                            fprintf(wf, "off=%04x val=%02x pc=%04x:%04x step%ld\n",
                                    (int)sa, (int)value, (int)mcu.cp, (int)mcu.pc, wc++);
                        }
                    }
#endif
                    sram[sa] = value;
                }
                else
                {
                    printf("Unknown write %x %x\n", address, value);
                }
            }
            else
            {
                if (address >= 0xe000 && address < 0xe040)
                {
                    PCM_Write(address & 0x3f, value);
                }
                else if (address >= 0xff80)
                {
                    MCU_DeviceWrite(address & 0x7f, value);
                }
                else if (address >= 0xfb80 && address < 0xff80
                    && (dev_register[DEV_RAME] & 0x80) != 0)
                {
                    ram[(address - 0xfb80) & 0x3ff] = value;
                }
                else if (address >= 0x8000 && address < 0xe000)
                {
                    sram[address & 0x7fff] = value;
                }
                else if (address >= 0xf000 && address < 0xf100)
                {
                    io_sd = address & 0xff;
                    LCD_Enable((io_sd & 8) != 0);
                }
                else if (address == 0xf105)
                {
                    LCD_Write(0, value);
                    ga_lcd_counter = 500;
                }
                else if (address == 0xf104)
                {
                    LCD_Write(1, value);
                    ga_lcd_counter = 500;
                }
                else if (address == 0xf107)
                {
                    io_sd = value;
                }
                else
                {
                    printf("Unknown write %x %x\n", address, value);
                }
            }
        }
        else if (mcu_jv880 && address >= 0x6196 && address <= 0x6199)
        {
            // nop: the jv880 rom writes into the rom at 002E77-002E7D
        }
        else
        {
            printf("Unknown write %x %x\n", address, value);
        }
    }
    else if (page == 5 && mcu_mk1)
    {
        sram[address & 0x7fff] = value; // FIXME
    }
    else if (page == 10 && !mcu_mk1)
    {
        sram[address & 0x7fff] = value; // FIXME
    }
    else if (page == 12 && mcu_jv880)
    {
        nvram[address & 0x7fff] = value; // FIXME
    }
    else if (page == 14 && mcu_jv880)
    {
        cardram[address & 0x7fff] = value; // FIXME
    }
    else
    {
        printf("Unknown write %x %x\n", (page << 16) | address, value);
    }
}

void MCU_Write16(uint32_t address, uint16_t value)
{
    address &= ~1;
    MCU_Write(address, value >> 8);
    MCU_Write(address + 1, value & 0xff);
}

void MCU_ReadInstruction(void)
{
    uint8_t operand = MCU_ReadCodeAdvance();

    MCU_Operand_Table[operand](operand);

    if (mcu.sr & STATUS_T)
    {
        MCU_Interrupt_Exception(EXCEPTION_SOURCE_TRACE);
    }
}

void MCU_Init(void)
{
    memset(&mcu, 0, sizeof(mcu_t));
}

void MCU_Reset(void)
{
    mcu.r[0] = 0;
    mcu.r[1] = 0;
    mcu.r[2] = 0;
    mcu.r[3] = 0;
    mcu.r[4] = 0;
    mcu.r[5] = 0;
    mcu.r[6] = 0;
    mcu.r[7] = 0;

    mcu.pc = 0;

    mcu.sr = 0x700;

    mcu.cp = 0;
    mcu.dp = 0;
    mcu.ep = 0;
    mcu.tp = 0;
    mcu.br = 0;

    uint32_t reset_address = MCU_GetVectorAddress(VECTOR_RESET);
    mcu.cp = (reset_address >> 16) & 0xff;
    mcu.pc = reset_address & 0xffff;

    mcu.exception_pending = -1;

    MCU_DeviceReset();

    if (mcu_mk1)
    {
        ga_int_enable = 255;
    }
}

void MCU_PostUART(uint8_t data)
{
    uart_buffer[uart_write_ptr] = data;
    uart_write_ptr = (uart_write_ptr + 1) % uart_buffer_size;
}

void MCU_UpdateUART_RX(void)
{
    if ((dev_register[DEV_SCR] & 16) == 0) // RX disabled
        return;
    if (uart_write_ptr == uart_read_ptr) // no byte
        return;

    if (dev_register[DEV_SSR] & 0x40)
        return;

    if (mcu.cycles < uart_rx_delay)
        return;

    uart_rx_byte = uart_buffer[uart_read_ptr];
    uart_read_ptr = (uart_read_ptr + 1) % uart_buffer_size;
    dev_register[DEV_SSR] |= 0x40;
    MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_UART_RX, (dev_register[DEV_SCR] & 0x40) != 0);
}

// dummy TX
void MCU_UpdateUART_TX(void)
{
    if ((dev_register[DEV_SCR] & 32) == 0) // TX disabled
        return;

    if (dev_register[DEV_SSR] & 0x80)
        return;

    if (mcu.cycles < uart_tx_delay)
        return;

    dev_register[DEV_SSR] |= 0x80;
    MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_UART_TX, (dev_register[DEV_SCR] & 0x80) != 0);

    // printf("tx:%x\n", dev_register[DEV_TDR]);
}

static bool work_thread_run = false;

static SDL_mutex *work_thread_lock;

void MCU_WorkThread_Lock(void)
{
    SDL_LockMutex(work_thread_lock);
}

void MCU_WorkThread_Unlock(void)
{
    SDL_UnlockMutex(work_thread_lock);
}

// ---- TEMP OBSERVATION (remove later) ----
#if SC55_TRACE
extern uint8_t sm_device_mode[32];
static FILE* obs_f = nullptr;
static long obs_step = 0;
static uint32_t last_main_pc = 0;
static void obs_dump_state(const char* why)
{
    if (!obs_f) { obs_f = fopen("obs.log", "w"); if (!obs_f) return; }
    fprintf(obs_f, "%s main pc=%02x:%04x sr=%04x irq0=%d irq1=%d ga_trig=%d | sm pc=%04x sleep=%d sr=%02x intreq=%02x u1ctrl=%02x u2ctrl=%02x u1ms=%02x sem=%02x cycles=%llu\n",
        why, mcu.cp, mcu.pc, mcu.sr,
        mcu.interrupt_pending[1], mcu.interrupt_pending[2], ga_int_trigger,
        sm.pc, sm.sleep, sm.sr, sm_device_mode[0x1c],
        sm_device_mode[0x06], sm_device_mode[0x0a],
        sm_device_mode[0x05], sm_device_mode[0x19],
        (unsigned long long)mcu.cycles);
    fflush(obs_f);
}
#endif // SC55_TRACE
// ---- END TEMP OBSERVATION ----

// ---- full state snapshot: -savesnap <cycles> / -loadsnap <file> ----
// -savesnap dumps the full state to demo_snap.bin at <cycles>; -loadsnap loads
// a snapshot file at the start (ex- -demo2), so a keyless run starts from the
// exact captured state (e.g. demo_postW.bin, mid-demo, W released).
extern uint8_t mcu_p0_data, mcu_p1_data;
extern uint8_t sm_ram[128];
extern uint8_t sm_shared_ram[192];
extern uint8_t sm_access[0x18];
extern uint8_t sm_p0_dir, sm_p1_dir, sm_cts;
extern uint8_t sm_device_mode[32];
extern uint64_t sm_timer_cycles;
extern uint8_t sm_timer_prescaler, sm_timer_counter;

static uint64_t snap_dump_at = 0;
static const char *snap_load_file = nullptr;
static int snap_done = 0;

static void state_save(FILE *f)
{
    fwrite(&mcu, sizeof mcu, 1, f);
    fwrite(ram, RAM_SIZE, 1, f);
    fwrite(sram, SRAM_SIZE, 1, f);
    fwrite(dev_register, sizeof dev_register, 1, f);
    fwrite(frt, sizeof frt, 1, f);
    fwrite(&timer, sizeof timer, 1, f);
    fwrite(&timer_cycles, sizeof timer_cycles, 1, f);
    fwrite(&timer_tempreg, sizeof timer_tempreg, 1, f);
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
    fwrite(uart_buffer, uart_buffer_size, 1, f);
    fwrite(&uart_write_ptr, 4, 1, f);
    fwrite(&uart_read_ptr, 4, 1, f);
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
    fwrite(&pcm, sizeof pcm, 1, f);
    LCD_StateSave(f);
}

static void state_load(FILE *f)
{
    fread(&mcu, sizeof mcu, 1, f);
    fread(ram, RAM_SIZE, 1, f);
    fread(sram, SRAM_SIZE, 1, f);
    fread(dev_register, sizeof dev_register, 1, f);
    fread(frt, sizeof frt, 1, f);
    fread(&timer, sizeof timer, 1, f);
    fread(&timer_cycles, sizeof timer_cycles, 1, f);
    fread(&timer_tempreg, sizeof timer_tempreg, 1, f);
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
    fread(uart_buffer, uart_buffer_size, 1, f);
    fread(&uart_write_ptr, 4, 1, f);
    fread(&uart_read_ptr, 4, 1, f);
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
    fread(&pcm, sizeof pcm, 1, f);
    LCD_StateLoad(f);
}

// ---- deterministic state hash dump: -hashdump <cycles> <file> ----
static uint64_t hashdump_at = 0;
static const char *hashdump_file = nullptr;
static int hashdump_done = 0;

static const uint64_t fnv1a_basis = 0xcbf29ce484222325ULL;
static const uint64_t fnv1a_prime = 0x00000100000001b3ULL;

static uint64_t fnv1a64(uint64_t h, const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    while (len--)
    {
        h ^= (uint64_t)*p++;
        h *= fnv1a_prime;
    }
    return h;
}

static void hashdump_range(FILE *f, const char *name, const void *data, size_t len)
{
    fprintf(f, "hash.%s = %016llx\n", name,
            (unsigned long long)fnv1a64(fnv1a_basis, data, len));
}

static void hashdump_write(const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f)
    {
        printf("hashdump: cannot open %s\n", path);
        fflush(stdout);
        return;
    }

    fprintf(f, "version = hashdump v1\n");
    fprintf(f, "requested_cycles = %llu\n", (unsigned long long)hashdump_at);
    fprintf(f, "mcu.cp = %02x\n", (unsigned)mcu.cp);
    fprintf(f, "mcu.pc = %04x\n", (unsigned)mcu.pc);
    fprintf(f, "mcu.sr = %04x\n", (unsigned)mcu.sr);
    fprintf(f, "mcu.cycles = %llu\n", (unsigned long long)mcu.cycles);
    fprintf(f, "pcm.config_reg_3c = %02x\n", (unsigned)pcm.config_reg_3c);
    fprintf(f, "pcm.config_reg_3d = %02x\n", (unsigned)pcm.config_reg_3d);
    fprintf(f, "pcm.irq_assert = %u\n", (unsigned)pcm.irq_assert);
    fprintf(f, "pcm.cycles = %llu\n", (unsigned long long)pcm.cycles);
    fprintf(f, "sm.pc = %04x\n", (unsigned)sm.pc);
    fprintf(f, "sm.cycles = %llu\n", (unsigned long long)sm.cycles);

    hashdump_range(f, "mcu", &mcu, sizeof(mcu));
    hashdump_range(f, "ram", ram, RAM_SIZE);
    hashdump_range(f, "sram", sram, SRAM_SIZE);
    hashdump_range(f, "dev_register", dev_register, sizeof(dev_register));
    hashdump_range(f, "frt", frt, sizeof(frt));
    hashdump_range(f, "timer", &timer, sizeof(timer));
    hashdump_range(f, "timer_cycles", &timer_cycles, sizeof(timer_cycles));
    hashdump_range(f, "timer_tempreg", &timer_tempreg, sizeof(timer_tempreg));
    hashdump_range(f, "ad_val", ad_val, sizeof(ad_val));
    hashdump_range(f, "analog_end_time", &analog_end_time, sizeof(analog_end_time));
    hashdump_range(f, "ga_int", ga_int, sizeof(ga_int));
    hashdump_range(f, "io_sd", &io_sd, sizeof(io_sd));
    hashdump_range(f, "mcu_p0_data", &mcu_p0_data, sizeof(mcu_p0_data));
    hashdump_range(f, "mcu_p1_data", &mcu_p1_data, sizeof(mcu_p1_data));
    hashdump_range(f, "pcm", &pcm, sizeof(pcm));
    hashdump_range(f, "sm", &sm, sizeof(sm));
    hashdump_range(f, "sm_ram", sm_ram, sizeof(sm_ram));
    hashdump_range(f, "sm_shared_ram", sm_shared_ram, sizeof(sm_shared_ram));
    hashdump_range(f, "sm_access", sm_access, sizeof(sm_access));
    hashdump_range(f, "sm_device_mode", sm_device_mode, sizeof(sm_device_mode));
    hashdump_range(f, "sm_p0_dir", &sm_p0_dir, sizeof(sm_p0_dir));
    hashdump_range(f, "sm_p1_dir", &sm_p1_dir, sizeof(sm_p1_dir));
    hashdump_range(f, "sm_cts", &sm_cts, sizeof(sm_cts));
    hashdump_range(f, "sm_timer_cycles", &sm_timer_cycles, sizeof(sm_timer_cycles));
    hashdump_range(f, "sm_timer_prescaler", &sm_timer_prescaler, sizeof(sm_timer_prescaler));
    hashdump_range(f, "sm_timer_counter", &sm_timer_counter, sizeof(sm_timer_counter));
    hashdump_range(f, "uart_buffer", uart_buffer, uart_buffer_size);
    hashdump_range(f, "uart_rx_byte", &uart_rx_byte, sizeof(uart_rx_byte));

    FILE *lf = tmpfile();
    if (lf)
    {
        LCD_StateSave(lf);
        fflush(lf);
        fseek(lf, 0, SEEK_END);
        long lsz = ftell(lf);
        fseek(lf, 0, SEEK_SET);
        if (lsz > 0)
        {
            uint8_t *lbuf = (uint8_t *)malloc((size_t)lsz);
            if (lbuf && fread(lbuf, 1, (size_t)lsz, lf) == (size_t)lsz)
                hashdump_range(f, "lcd_state", lbuf, (size_t)lsz);
            free(lbuf);
        }
        fclose(lf);
    }

    printf("hashdump: dumped state at c%llu to %s\n", (unsigned long long)mcu.cycles, path);
    fflush(stdout);
    fclose(f);
}

// ============================================================================
// M4 oracle capture (all options default off; zero behavior change without
// them):
//   -wav:<file>                 producer-side int16 stereo tap in MCU_PostSample
//   -audiowin <start> <end>     capture only mcu.cycles in [start,end); at the
//                               end cycle backfill the RIFF/data sizes, fclose,
//                               then write <file>.meta (wavdump v1) last
//   -audiohash <cycles> <file>  FNV-1a64 of the PostSample stream (repeatable)
//   -midiseq <file> [start]     MIDI schedule via MCU_PostUART + backpressure
//   -snapinfo <cycles> <file>   text scalars (isr4fe/sleep/iml/pend/...)
//
// Threading: the tap and all checkpoint writes run on work_thread only (the
// PCM_Update caller); finalization on normal exit runs in main after the work
// thread has been joined (atexit is the backstop). WAV writes use stdio full
// buffering, so the real-time sample path is never flushed per sample.
// ============================================================================

static bool audio_capture_on = false; // any audio tap active (wav or hash)

static const char *g_wav_path = nullptr;
static FILE *g_wav = nullptr;
static bool g_wav_windowed = false;
static bool g_wav_window_closed = false;
static bool g_wav_finalized = false;
static uint64_t g_audio_win_start = 0;
static uint64_t g_audio_win_end = 0;
static int g_wav_rate = 66207;
static uint64_t g_wav_pairs = 0;
static uint64_t g_wav_first_cycle = 0;
static uint64_t g_wav_last_cycle = 0;
static uint64_t g_wav_fnv = fnv1a_basis;

static const int AUDIO_HASH_MAX = 16;
struct audio_hash_cp
{
    uint64_t cycles;
    const char *file;
    int done;
};
static audio_hash_cp g_audio_hash[AUDIO_HASH_MAX];
static int g_audio_hash_n = 0;
static uint64_t g_stream_pairs = 0;
static uint64_t g_stream_fnv = fnv1a_basis;

static bool g_snapinfo_on = false;
static bool g_snapinfo_done = false;
static uint64_t g_snapinfo_at = 0;
static const char *g_snapinfo_file = nullptr;
static uint64_t g_isr4fe = 0;

struct midiseq_ev
{
    uint64_t cycle; // absolute (start + schedule cycle)
    uint32_t off;
    uint32_t len;
};
static midiseq_ev *g_midiseq_ev = nullptr;
static uint32_t g_midiseq_n = 0;
static uint32_t g_midiseq_cap = 0;
static uint32_t g_midiseq_pos = 0;
static uint8_t *g_midiseq_bytes = nullptr;
static size_t g_midiseq_bytes_len = 0;
static size_t g_midiseq_bytes_cap = 0;
static const char *g_midiseq_file = nullptr;
static uint64_t g_midiseq_start = 0;
static bool g_midiseq_loaded = false;

static void wav_put_u16le(FILE *f, uint16_t v)
{
    uint8_t b[2];
    b[0] = (uint8_t)(v & 0xff);
    b[1] = (uint8_t)((v >> 8) & 0xff);
    fwrite(b, 1, 2, f);
}

static void wav_put_u32le(FILE *f, uint32_t v)
{
    uint8_t b[4];
    b[0] = (uint8_t)(v & 0xff);
    b[1] = (uint8_t)((v >> 8) & 0xff);
    b[2] = (uint8_t)((v >> 16) & 0xff);
    b[3] = (uint8_t)((v >> 24) & 0xff);
    fwrite(b, 1, 4, f);
}

static void audio_hash_pair(uint64_t *h, int l, int r)
{
    uint8_t b[4];
    b[0] = (uint8_t)(l & 0xff);
    b[1] = (uint8_t)((l >> 8) & 0xff);
    b[2] = (uint8_t)(r & 0xff);
    b[3] = (uint8_t)((r >> 8) & 0xff);
    *h = fnv1a64(*h, b, 4);
}

static void audio_capture_sample(int l, int r)
{
    g_stream_pairs++;
    audio_hash_pair(&g_stream_fnv, l, r);

    if (!g_wav)
        return;
    if (g_wav_windowed && (mcu.cycles < g_audio_win_start || mcu.cycles >= g_audio_win_end))
        return;

    uint8_t b[4];
    b[0] = (uint8_t)(l & 0xff);
    b[1] = (uint8_t)((l >> 8) & 0xff);
    b[2] = (uint8_t)(r & 0xff);
    b[3] = (uint8_t)((r >> 8) & 0xff);
    fwrite(b, 1, 4, g_wav);
    if (g_wav_pairs == 0)
        g_wav_first_cycle = mcu.cycles;
    g_wav_last_cycle = mcu.cycles;
    g_wav_pairs++;
    audio_hash_pair(&g_wav_fnv, l, r);
}

static void audio_dump_finalize(void);

static void audio_dump_open(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f)
    {
        printf("wavdump: cannot open %s\n", path);
        fflush(stdout);
        g_wav_path = nullptr;
        return;
    }
    g_wav = f;
    g_wav_pairs = 0;
    g_wav_first_cycle = 0;
    g_wav_last_cycle = 0;
    g_wav_fnv = fnv1a_basis;
    g_wav_window_closed = false;
    g_wav_finalized = false;

    // Canonical 44-byte PCM header; the RIFF size (offset 4) and data size
    // (offset 40) are placeholders backfilled by audio_dump_finalize().
    fwrite("RIFF", 1, 4, f);
    wav_put_u32le(f, 36);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    wav_put_u32le(f, 16);
    wav_put_u16le(f, 1);                          // PCM
    wav_put_u16le(f, 2);                          // channels
    wav_put_u32le(f, (uint32_t)g_wav_rate);
    wav_put_u32le(f, (uint32_t)g_wav_rate * 4u);  // byte rate = rate*2ch*2B
    wav_put_u16le(f, 4);                          // block align
    wav_put_u16le(f, 16);                         // bits per sample
    fwrite("data", 1, 4, f);
    wav_put_u32le(f, 0);

    atexit(audio_dump_finalize);
    printf("wavdump: opened %s (PCM s16le, 2ch, %d Hz)\n", path, g_wav_rate);
    fflush(stdout);
}

static void audio_dump_finalize(void)
{
    if (g_wav_finalized)
        return;
    g_wav_finalized = true;
    if (!g_wav)
        return;

    uint64_t data_bytes = g_wav_pairs * 4u;
    if (fseek(g_wav, 4, SEEK_SET) == 0)
        wav_put_u32le(g_wav, (uint32_t)(36u + data_bytes));
    if (fseek(g_wav, 40, SEEK_SET) == 0)
        wav_put_u32le(g_wav, (uint32_t)data_bytes);
    fflush(g_wav);
    fclose(g_wav);
    g_wav = nullptr;

    // <file>.meta (wavdump v1) is the deterministic completion marker and is
    // only written when the -audiowin end cycle was actually reached.
    if (!g_wav_windowed || !g_wav_window_closed)
        return;

    char meta_path[4096];
    snprintf(meta_path, sizeof meta_path, "%s.meta", g_wav_path);
    FILE *f = fopen(meta_path, "w");
    if (!f)
    {
        printf("wavdump: cannot write %s\n", meta_path);
        fflush(stdout);
        return;
    }
    fprintf(f, "version = wavdump v1\n");
    fprintf(f, "format = s16le\n");
    fprintf(f, "channels = 2\n");
    fprintf(f, "rate = %d\n", g_wav_rate);
    fprintf(f, "start_cycles = %llu\n", (unsigned long long)g_audio_win_start);
    fprintf(f, "end_cycles = %llu\n", (unsigned long long)g_audio_win_end);
    fprintf(f, "first_cycle = %llu\n", (unsigned long long)g_wav_first_cycle);
    fprintf(f, "last_cycle = %llu\n", (unsigned long long)g_wav_last_cycle);
    fprintf(f, "sample_pairs = %llu\n", (unsigned long long)g_wav_pairs);
    fprintf(f, "data_bytes = %llu\n", (unsigned long long)data_bytes);
    fprintf(f, "payload_fnv1a = %016llx\n", (unsigned long long)g_wav_fnv);
    fclose(f);
    printf("wavdump: finalized %s (%llu pairs) and wrote %s\n",
           g_wav_path, (unsigned long long)g_wav_pairs, meta_path);
    fflush(stdout);
}

static void audio_hash_write(const audio_hash_cp *cp)
{
    FILE *f = fopen(cp->file, "w");
    if (!f)
    {
        printf("audiohash: cannot open %s\n", cp->file);
        fflush(stdout);
        return;
    }
    fprintf(f, "version = audiohash v1\n");
    fprintf(f, "requested_cycles = %llu\n", (unsigned long long)cp->cycles);
    fprintf(f, "mcu.cycles = %llu\n", (unsigned long long)mcu.cycles);
    fprintf(f, "sample_pairs = %llu\n", (unsigned long long)g_stream_pairs);
    fprintf(f, "audio_fnv1a = %016llx\n", (unsigned long long)g_stream_fnv);
    fclose(f);
    printf("audiohash: c%llu pairs=%llu fnv1a=%016llx -> %s\n",
           (unsigned long long)mcu.cycles, (unsigned long long)g_stream_pairs,
           (unsigned long long)g_stream_fnv, cp->file);
    fflush(stdout);
}

static void snapinfo_write(void)
{
    FILE *f = fopen(g_snapinfo_file, "w");
    if (!f)
    {
        printf("snapinfo: cannot open %s\n", g_snapinfo_file);
        fflush(stdout);
        return;
    }
    uint32_t pend = 0;
    for (int i = 0; i < INTERRUPT_SOURCE_MAX; i++)
        if (mcu.interrupt_pending[i])
            pend |= 1u << i;
    uint32_t mask_pop = 0;
    {
        uint32_t v = pcm.voice_mask;
        while (v)
        {
            mask_pop += (v & 1u);
            v >>= 1;
        }
    }
    fprintf(f, "version = snapinfo v1\n");
    fprintf(f, "requested_cycles = %llu\n", (unsigned long long)g_snapinfo_at);
    fprintf(f, "mcu.cycles = %llu\n", (unsigned long long)mcu.cycles);
    fprintf(f, "mcu.cp = %02x\n", (unsigned)mcu.cp);
    fprintf(f, "mcu.pc = %04x\n", (unsigned)mcu.pc);
    fprintf(f, "mcu.sr = %04x\n", (unsigned)mcu.sr);
    fprintf(f, "isr4fe = %llu\n", (unsigned long long)g_isr4fe);
    fprintf(f, "sleep = %u\n", (unsigned)(mcu.sleep ? 1 : 0));
    fprintf(f, "iml = %u\n", (unsigned)((mcu.sr >> 8) & 7));
    fprintf(f, "pend = %u\n", (unsigned)pend);
    fprintf(f, "pcm.select_channel = %02x\n", (unsigned)pcm.select_channel);
    fprintf(f, "pcm.irq_channel = %02x\n", (unsigned)pcm.irq_channel);
    fprintf(f, "pcm.irq_assert = %u\n", (unsigned)pcm.irq_assert);
    fprintf(f, "pcm.config_reg_3c = %02x\n", (unsigned)pcm.config_reg_3c);
    fprintf(f, "pcm.config_reg_3d = %02x\n", (unsigned)pcm.config_reg_3d);
    fprintf(f, "voice_mask_popcount = %u\n", (unsigned)mask_pop);
    fclose(f);
    printf("snapinfo: wrote %s at c%llu (isr4fe=%llu)\n", g_snapinfo_file,
           (unsigned long long)mcu.cycles, (unsigned long long)g_isr4fe);
    fflush(stdout);
}

static void audio_capture_poll(void)
{
    for (int i = 0; i < g_audio_hash_n; i++)
    {
        if (!g_audio_hash[i].done && mcu.cycles >= g_audio_hash[i].cycles)
        {
            audio_hash_write(&g_audio_hash[i]);
            g_audio_hash[i].done = 1;
        }
    }
    // The .meta marker is written last so its appearance means the WAV is
    // already closed and complete.
    if (g_wav_windowed && g_wav && !g_wav_window_closed && mcu.cycles >= g_audio_win_end)
    {
        g_wav_window_closed = true;
        audio_dump_finalize();
    }
}

static uint32_t midiseq_free_bytes(void)
{
    uint32_t occupied = (uart_write_ptr + uart_buffer_size - uart_read_ptr) % uart_buffer_size;
    return uart_buffer_size - 1u - occupied;
}

static int midiseq_push_byte(uint8_t b)
{
    if (g_midiseq_bytes_len == g_midiseq_bytes_cap)
    {
        size_t ncap = g_midiseq_bytes_cap ? g_midiseq_bytes_cap * 2 : 4096;
        uint8_t *nb = (uint8_t *)realloc(g_midiseq_bytes, ncap);
        if (!nb)
        {
            fprintf(stderr, "midiseq: out of memory\n");
            return 0;
        }
        g_midiseq_bytes = nb;
        g_midiseq_bytes_cap = ncap;
    }
    g_midiseq_bytes[g_midiseq_bytes_len++] = b;
    return 1;
}

static void midiseq_push_ev(uint64_t cycle, uint32_t off, uint32_t len)
{
    if (g_midiseq_n == g_midiseq_cap)
    {
        uint32_t ncap = g_midiseq_cap ? g_midiseq_cap * 2 : 256;
        midiseq_ev *ne = (midiseq_ev *)realloc(g_midiseq_ev, (size_t)ncap * sizeof(midiseq_ev));
        if (!ne)
        {
            fprintf(stderr, "midiseq: out of memory\n");
            return;
        }
        g_midiseq_ev = ne;
        g_midiseq_cap = ncap;
    }
    g_midiseq_ev[g_midiseq_n].cycle = cycle;
    g_midiseq_ev[g_midiseq_n].off = off;
    g_midiseq_ev[g_midiseq_n].len = len;
    g_midiseq_n++;
}

// Parse the midisched text schedule: "<cycle> <hexbyte> [<hexbyte>...]" per
// line, '#' starts a comment, blank lines ignored. Cycles are relative to the
// -midiseq start argument.
static void midiseq_load(const char *path)
{
    if (!path)
        return;
    FILE *f = fopen(path, "r");
    if (!f)
    {
        fprintf(stderr, "midiseq: cannot open %s\n", path);
        fflush(stderr);
        return;
    }
    char line[1024];
    uint32_t events = 0, bad = 0;
    while (fgets(line, sizeof line, f))
    {
        char *p = line;
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '#' || *p == '\r' || *p == '\n' || *p == '\0')
            continue;
        char *end = nullptr;
        unsigned long long rel = strtoull(p, &end, 10);
        if (end == p)
        {
            bad++;
            continue;
        }
        uint32_t off = (uint32_t)g_midiseq_bytes_len;
        uint32_t cnt = 0;
        while (*end)
        {
            while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
                end++;
            if (*end == '\0' || *end == '#')
                break;
            char *bend = nullptr;
            unsigned long v = strtoul(end, &bend, 16);
            if (bend == end || v > 0xffu || !midiseq_push_byte((uint8_t)v))
            {
                bad++;
                cnt = 0;
                break;
            }
            cnt++;
            end = bend;
        }
        if (cnt > 0)
        {
            midiseq_push_ev(g_midiseq_start + rel, off, cnt);
            events++;
        }
    }
    fclose(f);
    g_midiseq_loaded = (g_midiseq_n > 0);
    printf("midiseq: loaded %u event(s) from %s (start=%llu, %u malformed line(s) skipped)\n",
           events, path, (unsigned long long)g_midiseq_start, bad);
    fflush(stdout);
}

// Post the due schedule events. If the 8192-byte UART ring cannot hold a whole
// event yet, postpone it and retry next instruction: unread bytes are never
// overwritten.
static void midiseq_poll(void)
{
    if (!g_midiseq_loaded)
        return;
    while (g_midiseq_pos < g_midiseq_n)
    {
        const midiseq_ev *e = &g_midiseq_ev[g_midiseq_pos];
        if (mcu.cycles < e->cycle)
            break;
        if (midiseq_free_bytes() < e->len)
            break;
        for (uint32_t i = 0; i < e->len; i++)
            MCU_PostUART(g_midiseq_bytes[e->off + i]);
        g_midiseq_pos++;
    }
}

int SDLCALL work_thread(void* data)
{
    work_thread_lock = SDL_CreateMutex();

    MCU_WorkThread_Lock();

    // ---- -loadsnap <file>: load the full machine state at the start (like the
    // old -demo2, but with a user-specified filename), so a keyless run starts
    // from the exact captured state (e.g. demo_postW.bin, mid-demo, W released). ----
    if (snap_load_file)
    {
        FILE *f = fopen(snap_load_file, "rb");
        if (f)
        {
            state_load(f);
            fclose(f);
            printf("loadsnap: loaded state from %s at c%llu\n", snap_load_file, (unsigned long long)mcu.cycles);
        }
        else
            printf("loadsnap: missing %s\n", snap_load_file);
    }

    while (work_thread_run)
    {
        if (pcm.config_reg_3c & 0x40)
            sample_write_ptr &= ~3;
        else
            sample_write_ptr &= ~1;
        if (sample_read_ptr == sample_write_ptr)
        {
            MCU_WorkThread_Unlock();
            while (sample_read_ptr == sample_write_ptr)
            {
                SDL_Delay(1);
            }
            MCU_WorkThread_Lock();
        }

        if (!mcu.ex_ignore)
            MCU_Interrupt_Handle();
        else
            mcu.ex_ignore = 0;

        if (!mcu.sleep)
        {
            // -snapinfo heartbeat: count instruction fetches at flat 00:04FE
            // (the main event dispatcher epilogue, i.e. completed dispatches).
            if (g_snapinfo_on && mcu.cp == 0x00 && mcu.pc == 0x04fe)
                g_isr4fe++;
            if (mk2cpp_enabled)
                MK2CPP_Step();
            else
                MCU_ReadInstruction();
        }

        mcu.cycles += 12; // FIXME: assume 12 cycles per instruction

        // ---- unified main+SM PC trace ([tracepc_start, tracepc_end) window) ----
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
                g_trace_f = nullptr;
                g_trace_closed = true;
            }
        }

        // ---- demo key sequence (SC-55mk2 built-in demo, -demo [cycles]) ----
        if (demo_seq_enabled)
        {
            // Key states fire at demo_start + relative offsets. The LCD splash
            // takes ~5s, so the default start is 144M cycles (6s). Each key state
            // is held 250ms (6M cycles @ 24MHz) so the firmware's debounce logic
            // sees real key presses; the second power-on has no boot animation,
            // so W (INST_ALL) can start the demo song at +48M.
            static const uint64_t demo_off[7] = { 0, 6000000, 12000000, 18000000, 24000000, 48000000, 54000000 };
            static const uint32_t demo_mask[7] = {
                1u << MCU_BUTTON_POWER,
                (1u << MCU_BUTTON_PART_L) | (1u << MCU_BUTTON_PART_R),
                (1u << MCU_BUTTON_PART_L) | (1u << MCU_BUTTON_PART_R),
                (1u << MCU_BUTTON_POWER) | (1u << MCU_BUTTON_PART_L) | (1u << MCU_BUTTON_PART_R),
                0,
                1u << MCU_BUTTON_INST_ALL,
                0,
            };
            static int demo_step = 0;
            while (demo_step < 7 && mcu.cycles >= demo_start + demo_off[demo_step])
            {
                SDL_AtomicSet(&mcu_button_pressed, (int)demo_mask[demo_step]);
                printf("demo: step%d mask=%08x at c%llu\n", demo_step, demo_mask[demo_step],
                       (unsigned long long)(demo_start + demo_off[demo_step]));
                demo_step++;
            }
        }

        // ---- mock note-on (-mocknote [cycles]) ----
        // The LCD splash animation takes ~5s and no sound is output until it
        // finishes, so the default fire cycle is 144M (6s). The SC-55mk2 rejects
        // MIDI during the demo phase, so to take effect the note must fire
        // BEFORE the -demo start (see the startup warning when both are enabled).
        if (mocknote_enabled && !mocknote_done && mcu.cycles >= mocknote_at)
        {
            SM_PostUART(0x90);
            SM_PostUART(60);
            SM_PostUART(100);
            mocknote_done = true;
            printf("mocknote: posted note-on at c%llu\n", (unsigned long long)mcu.cycles);
            fflush(stdout);
        }

        // ---- snapshot dump trigger (-savesnap <cycles>) ----
        if (!snap_done && snap_dump_at && mcu.cycles >= snap_dump_at)
        {
            FILE *f = fopen("demo_snap.bin", "wb");
            if (f)
            {
                state_save(f);
                printf("snap: dumped full state at c%llu\n", (unsigned long long)mcu.cycles);
                fflush(stdout);
                fclose(f);
            }
            snap_done = 1;
        }

        // ---- deterministic state hash dump trigger (-hashdump <cycles> <file>) ----
        if (!hashdump_done && hashdump_at && mcu.cycles >= hashdump_at)
        {
            if (hashdump_file)
                hashdump_write(hashdump_file);
            hashdump_done = 1;
        }

        // ---- M4 oracle injection/capture polls (no-ops by default) ----
        midiseq_poll();
        audio_capture_poll();
        if (g_snapinfo_on && !g_snapinfo_done && mcu.cycles >= g_snapinfo_at)
        {
            snapinfo_write();
            g_snapinfo_done = 1;
        }

        // if (mcu.cycles % 24000000 == 0)
        //     printf("seconds: %i\n", (int)(mcu.cycles / 24000000));

        PCM_Update(mcu.cycles);

        TIMER_Clock(mcu.cycles);

        if (!mcu_mk1 && !mcu_jv880 && !mcu_scb55)
            SM_Update(mcu.cycles);
        else
        {
            MCU_UpdateUART_RX();
            MCU_UpdateUART_TX();
        }

        MCU_UpdateAnalog(mcu.cycles);

#if SC55_TRACE
        frt_state_log();
#endif

        // ---- TEMP OBSERVATION ----
#if SC55_TRACE
        {
            uint32_t cur_pc = ((uint32_t)mcu.cp << 16) | mcu.pc;
            uint32_t prev_pc = last_main_pc;
            if (cur_pc == 0x007c3f && cur_pc != last_main_pc)
            {
                static long st_n2 = 0;
                if (st_n2 < 32)
                {
                    static FILE *rm2 = nullptr;
                    if (!rm2) rm2 = fopen("ram_orig.log", "a");
                    if (rm2)
                    {
                        fprintf(rm2, "=== burst%ld ===\n", st_n2);
                        for (int b = 0; b < 0x8000; b += 16)
                        {
                            for (int k = 0; k < 16; k++)
                                fprintf(rm2, "%02x", sram[b + k]);
                            fprintf(rm2, "\n");
                        }
                        fflush(rm2);
                    }
                    st_n2++;
                }
            }
            if (cur_pc != last_main_pc)
            {
                last_main_pc = cur_pc;
                obs_dump_state("pcchg");
                if (prev_pc == 0x007c3f)
                {
                    static FILE* st_f = nullptr;
                    static long st_n = 0;
                    if (!st_f) st_f = fopen("stack_orig.log", "w");
                    if (st_f)
                    {
                        fprintf(st_f, "burst%ld r7=%04x postpop: ", st_n++, mcu.r[7]);
                        for (int s = 0; s < 16; s++)
                            fprintf(st_f, "%02x%02x ", sram[(mcu.r[7] + s * 2) & 0x7fff], sram[(mcu.r[7] + s * 2 + 1) & 0x7fff]);
                        fprintf(st_f, "\n");
                        fflush(st_f);
                    }

                }
            }
        }
#endif // SC55_TRACE
        // ---- END TEMP ----

        if (mcu_mk1)
        {
            if (ga_lcd_counter)
            {
                ga_lcd_counter--;
                if (ga_lcd_counter == 0)
                {
                    MCU_GA_SetGAInt(1, 0);
                    MCU_GA_SetGAInt(1, 1);
                }
            }
        }
    }
    if (g_trace_f) { fflush(g_trace_f); fclose(g_trace_f); }
    free(g_trace_buf);
    MCU_WorkThread_Unlock();

    SDL_DestroyMutex(work_thread_lock);

    return 0;
}

static void MCU_Run()
{
    bool working = true;

    work_thread_run = true;
    SDL_Thread *thread = SDL_CreateThread(work_thread, "work thread", 0);

    while (working)
    {
        if(LCD_QuitRequested())
            working = false;

        LCD_Update();
        SDL_Delay(15);
    }

    work_thread_run = false;
    SDL_WaitThread(thread, 0);
}

void MCU_PatchROM(void)
{

    //rom2[0x1333] = 0x11;
    //rom2[0x1334] = 0x19;
    //rom1[0x622d] = 0x19;
}

uint8_t mcu_p0_data = 0x00;
uint8_t mcu_p1_data = 0x00;

uint8_t MCU_ReadP0(void)
{
    return 0xff;
}

uint8_t MCU_ReadP1(void)
{
    uint8_t data = 0xff;
    uint32_t button_pressed = (uint32_t)SDL_AtomicGet(&mcu_button_pressed);

    if ((mcu_p0_data & 1) == 0)
        data &= ((button_pressed >> 0) & 255) ^ 255;
    if ((mcu_p0_data & 2) == 0)
        data &= ((button_pressed >> 8) & 255) ^ 255;
    if ((mcu_p0_data & 4) == 0)
        data &= ((button_pressed >> 16) & 255) ^ 255;
    if ((mcu_p0_data & 8) == 0)
        data &= ((button_pressed >> 24) & 255) ^ 255;

    return data;
}

void MCU_WriteP0(uint8_t data)
{
    mcu_p0_data = data;
}

void MCU_WriteP1(uint8_t data)
{
    mcu_p1_data = data;
}

uint8_t tempbuf[0x800000];

void unscramble(uint8_t *src, uint8_t *dst, int len)
{
    for (int i = 0; i < len; i++)
    {
        int address = i & ~0xfffff;
        static const int aa[] = {
            2, 0, 3, 4, 1, 9, 13, 10, 18, 17, 6, 15, 11, 16, 8, 5, 12, 7, 14, 19
        };
        for (int j = 0; j < 20; j++)
        {
            if (i & (1 << j))
                address |= 1<<aa[j];
        }
        uint8_t srcdata = src[address];
        uint8_t data = 0;
        static const int dd[] = {
            2, 0, 4, 5, 7, 6, 3, 1
        };
        for (int j = 0; j < 8; j++)
        {
            if (srcdata & (1 << dd[j]))
                data |= 1<<j;
        }
        dst[i] = data;
    }
}

void audio_callback(void* /*userdata*/, Uint8* stream, int len)
{
    len /= 2;
    memcpy(stream, &sample_buffer[sample_read_ptr], len * 2);
    memset(&sample_buffer[sample_read_ptr], 0, len * 2);
    sample_read_ptr += len;
    sample_read_ptr %= audio_buffer_size;
}

static const char* audio_format_to_str(int format)
{
    switch(format)
    {
    case AUDIO_S8:
        return "S8";
    case AUDIO_U8:
        return "U8";
    case AUDIO_S16MSB:
        return "S16MSB";
    case AUDIO_S16LSB:
        return "S16LSB";
    case AUDIO_U16MSB:
        return "U16MSB";
    case AUDIO_U16LSB:
        return "U16LSB";
    case AUDIO_S32MSB:
        return "S32MSB";
    case AUDIO_S32LSB:
        return "S32LSB";
    case AUDIO_F32MSB:
        return "F32MSB";
    case AUDIO_F32LSB:
        return "F32LSB";
    }
    return "UNK";
}

int MCU_OpenAudio(int deviceIndex, int pageSize, int pageNum)
{
    SDL_AudioSpec spec = {};
    SDL_AudioSpec spec_actual = {};

    audio_page_size = (pageSize/2)*2; // must be even
    audio_buffer_size = audio_page_size*pageNum;
    
    spec.format = AUDIO_S16SYS;
    spec.freq = (mcu_mk1 || mcu_jv880) ? 64000 : 66207;
    spec.channels = 2;
    spec.callback = audio_callback;
    spec.samples = audio_page_size / 4;
    
    sample_buffer = (short*)calloc(audio_buffer_size, sizeof(short));
    if (!sample_buffer)
    {
        printf("Cannot allocate audio buffer.\n");
        return 0;
    }
    sample_read_ptr = 0;
    sample_write_ptr = 0;
    
    int num = SDL_GetNumAudioDevices(0);
    if (num == 0)
    {
        printf("No audio output device found.\n");
        return 0;
    }
    
    if (deviceIndex < -1 || deviceIndex >= num)
    {
        printf("Out of range audio device index is requested. Default audio output device is selected.\n");
        deviceIndex = -1;
    }
    
    const char* audioDevicename = deviceIndex == -1 ? "Default device" : SDL_GetAudioDeviceName(deviceIndex, 0);
    
    sdl_audio = SDL_OpenAudioDevice(deviceIndex == -1 ? NULL : audioDevicename, 0, &spec, &spec_actual, 0);
    if (!sdl_audio)
    {
        return 0;
    }

    printf("Audio device: %s\n", audioDevicename);

    printf("Audio Requested: F=%s, C=%d, R=%d, B=%d\n",
           audio_format_to_str(spec.format),
           spec.channels,
           spec.freq,
           spec.samples);

    printf("Audio Actual: F=%s, C=%d, R=%d, B=%d\n",
           audio_format_to_str(spec_actual.format),
           spec_actual.channels,
           spec_actual.freq,
           spec_actual.samples);
    fflush(stdout);

    SDL_PauseAudioDevice(sdl_audio, 0);

    return 1;
}

void MCU_CloseAudio(void)
{
    SDL_CloseAudio();
    if (sample_buffer) free(sample_buffer);
}

void MCU_PostSample(int *sample)
{
    if (master_gain != 1.0f)
    {
        /* Normalize raw (20-bit sample << 12) to +-1.0 (= 2^31), apply the
         * gain, clamp, then scale to the int16 rail (x 2^16). Avoids the int32
         * overflow of the old raw multiply. */
        float l = (float)sample[0] * (1.0f / 2147483648.0f) * master_gain;
        float r = (float)sample[1] * (1.0f / 2147483648.0f) * master_gain;
        if (l > 1.0f)
            l = 1.0f;
        else if (l < -1.0f)
            l = -1.0f;
        if (r > 1.0f)
            r = 1.0f;
        else if (r < -1.0f)
            r = -1.0f;
        sample[0] = (int)floorf(l * 65536.0f);
        sample[1] = (int)floorf(r * 65536.0f);
    }
    else
    {
        sample[0] >>= 15;
        sample[1] >>= 15;
    }

    if (sample[0] > INT16_MAX)
        sample[0] = INT16_MAX;
    else if (sample[0] < INT16_MIN)
        sample[0] = INT16_MIN;
    if (sample[1] > INT16_MAX)
        sample[1] = INT16_MAX;
    else if (sample[1] < INT16_MIN)
        sample[1] = INT16_MIN;

    sample_buffer[sample_write_ptr + 0] = sample[0];
    sample_buffer[sample_write_ptr + 1] = sample[1];
    sample_write_ptr = (sample_write_ptr + 2) % audio_buffer_size;

    // Producer-side tap: same post-gain/post-clamp int16 values that go into
    // the ring (including the oversampling second PostSample). Off by default.
    if (audio_capture_on)
        audio_capture_sample(sample[0], sample[1]);
}

void MCU_GA_SetGAInt(int line, int value)
{
    // guesswork
    if (value && !ga_int[line] && (ga_int_enable & (1 << line)) != 0)
        ga_int_trigger = line;
    ga_int[line] = value;

    if (mcu_jv880)
        MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_IRQ0, ga_int_trigger != 0);
    else
        MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_IRQ1, ga_int_trigger != 0);
}

void MCU_EncoderTrigger(int dir)
{
    if (!mcu_jv880) return;
    MCU_GA_SetGAInt(dir == 0 ? 3 : 4, 0);
    MCU_GA_SetGAInt(dir == 0 ? 3 : 4, 1);
}

static FILE *s_rf[ROM_SET_N_FILES] =
{
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr
};

static void closeAllR()
{
    for(size_t i = 0; i < ROM_SET_N_FILES; ++i)
    {
        if(s_rf[i])
            fclose(s_rf[i]);
        s_rf[i] = nullptr;
    }
}

enum class ResetType {
    NONE,
    GS_RESET,
    GM_RESET,
};

void MIDI_Reset(ResetType resetType)
{
    const unsigned char gmReset[] = { 0xF0, 0x7E, 0x7F, 0x09, 0x01, 0xF7 };
    const unsigned char gsReset[] = { 0xF0, 0x41, 0x10, 0x42, 0x12, 0x40, 0x00, 0x7F, 0x00, 0x41, 0xF7 };
    
    if (resetType == ResetType::GS_RESET)
    {
        for (size_t i = 0; i < sizeof(gsReset); i++)
        {
            MCU_PostUART(gsReset[i]);
        }
    }
    else  if (resetType == ResetType::GM_RESET)
    {
        for (size_t i = 0; i < sizeof(gmReset); i++)
        {
            MCU_PostUART(gmReset[i]);
        }
    }

}

int main(int argc, char *argv[])
{
    (void)argc;
    std::string basePath;

    int port = 0;
    int audioDeviceIndex = -1;
    int pageSize = 512;
    int pageNum = 32;
    bool autodetect = true;
    bool nomidi = false;
    ResetType resetType = ResetType::NONE;

    romset = ROM_SET_MK2;

    {
        for (int i = 1; i < argc; i++)
        {
            if (!strncmp(argv[i], "-p:", 3))
            {
                port = atoi(argv[i] + 3);
            }
            else if (!strcmp(argv[i], "-nomidi"))
            {
                nomidi = true;
            }
            else if (!strncmp(argv[i], "-a:", 3))
            {
                audioDeviceIndex = atoi(argv[i] + 3);
            }
            else if (!strncmp(argv[i], "-ab:", 4))
            {
                char* pColon = argv[i] + 3;
                
                if (pColon[1] != 0)
                {
                    pageSize = atoi(++pColon);
                    pColon = strchr(pColon, ':');
                    if (pColon && pColon[1] != 0)
                    {
                        pageNum = atoi(++pColon);
                    }
                }
                
                // reset both if either is invalid
                if (pageSize <= 0 || pageNum <= 0)
                {
                    pageSize = 512;
                    pageNum = 32;
                }
            }
            else if (!strcmp(argv[i], "-mk2cpp"))
            {
                mk2cpp_enabled = 1;
                MK2CPP_Init();
                printf("mk2cpp: translated core enabled (mixed=%d) -- %s\n", mk2cpp_mixed, MK2CPP_Version());
                fflush(stdout);
            }
            else if (!strcmp(argv[i], "-mk2"))
            {
                romset = ROM_SET_MK2;
                autodetect = false;
            }
            else if (!strcmp(argv[i], "-float"))
            {
                pcm_float = 1;
            }
            else if (!strcmp(argv[i], "-demo"))
            {
                demo_seq_enabled = true;
                if (i + 1 < argc && is_cycle_arg(argv[i + 1]))
                    demo_start = parse_cycles(argv[++i]);
            }
            else if (!strcmp(argv[i], "-mocknote"))
            {
                mocknote_enabled = true;
                if (i + 1 < argc && is_cycle_arg(argv[i + 1]))
                    mocknote_at = parse_cycles(argv[++i]);
            }
            else if (!strcmp(argv[i], "-tracepc") && i + 1 < argc)
            {
                tracepc_enabled = true;
                tracepc_file = argv[++i];
                if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9')
                    tracepc_start = strtoull(argv[++i], 0, 10);
                if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9')
                    tracepc_end = strtoull(argv[++i], 0, 10);
            }
            else if (!strcmp(argv[i], "-pcmtrace"))
            {
                g_pcm_trace = true;
            }
            else if (!strncmp(argv[i], "-wav:", 5))
            {
                g_wav_path = argv[i] + 5;
            }
            else if (!strcmp(argv[i], "-audiowin"))
            {
                if (i + 2 < argc)
                {
                    uint64_t win_start = strtoull(argv[++i], 0, 10);
                    uint64_t win_end = strtoull(argv[++i], 0, 10);
                    if (win_end <= win_start)
                    {
                        fprintf(stderr, "warning: -audiowin %llu %llu invalid (end <= start), ignored\n",
                                (unsigned long long)win_start, (unsigned long long)win_end);
                    }
                    else
                    {
                        g_audio_win_start = win_start;
                        g_audio_win_end = win_end;
                        g_wav_windowed = true;
                    }
                }
                else
                {
                    fprintf(stderr, "warning: -audiowin requires <start> <end>, ignored\n");
                }
            }
            else if (!strcmp(argv[i], "-audiohash"))
            {
                if (i + 2 < argc)
                {
                    if (g_audio_hash_n >= AUDIO_HASH_MAX)
                    {
                        fprintf(stderr, "warning: -audiohash limited to %d checkpoints, ignored\n", AUDIO_HASH_MAX);
                        i += 2;
                    }
                    else
                    {
                        audio_hash_cp *cp = &g_audio_hash[g_audio_hash_n];
                        cp->cycles = strtoull(argv[++i], 0, 10);
                        cp->file = argv[++i];
                        cp->done = 0;
                        g_audio_hash_n++;
                    }
                }
                else
                {
                    fprintf(stderr, "warning: -audiohash requires <cycles> <file>, ignored\n");
                }
            }
            else if (!strcmp(argv[i], "-midiseq"))
            {
                if (i + 1 < argc)
                {
                    g_midiseq_file = argv[++i];
                    if (i + 1 < argc && is_cycle_arg(argv[i + 1]))
                        g_midiseq_start = parse_cycles(argv[++i]);
                }
                else
                {
                    fprintf(stderr, "warning: -midiseq requires <file> [start], ignored\n");
                }
            }
            else if (!strcmp(argv[i], "-snapinfo"))
            {
                if (i + 2 < argc)
                {
                    g_snapinfo_at = strtoull(argv[++i], 0, 10);
                    g_snapinfo_file = argv[++i];
                    g_snapinfo_on = true;
                }
                else
                {
                    fprintf(stderr, "warning: -snapinfo requires <cycles> <file>, ignored\n");
                }
            }
            else if (!strcmp(argv[i], "-savesnap") && i + 1 < argc)
            {
                snap_dump_at = strtoull(argv[++i], 0, 10);
            }
            else if (!strcmp(argv[i], "-hashdump"))
            {
                if (i + 2 < argc)
                {
                    hashdump_at = strtoull(argv[++i], 0, 10);
                    hashdump_file = argv[++i];
                }
                else
                {
                    fprintf(stderr, "warning: -hashdump requires <cycles> <file>, ignored\n");
                }
            }
            else if (!strcmp(argv[i], "-loadsnap") && i + 1 < argc)
            {
                snap_load_file = argv[++i];
            }
            else if (!strncmp(argv[i], "-gain:", 6))
            {
                // "<number>" = linear multiplier; "<number>db" = decibels,
                // scale = pow(10, db / 20) (same convention as the standard frontend).
                const char* val = argv[i] + 6;
                size_t len = strlen(val);
                if (len >= 2 && (val[len - 2] == 'd' || val[len - 2] == 'D') &&
                    (val[len - 1] == 'b' || val[len - 1] == 'B'))
                {
                    char tmp[32];
                    size_t n = len - 2;
                    if (n > sizeof(tmp) - 1)
                        n = sizeof(tmp) - 1;
                    memcpy(tmp, val, n);
                    tmp[n] = 0;
                    master_gain = (float)pow(10.0, atof(tmp) / 20.0);
                }
                else
                {
                    master_gain = (float)atof(val);
                }
            }
            else if (!strcmp(argv[i], "-st"))
            {
                romset = ROM_SET_ST;
                autodetect = false;
            }
            else if (!strcmp(argv[i], "-mk1"))
            {
                romset = ROM_SET_MK1;
                autodetect = false;
            }
            else if (!strcmp(argv[i], "-cm300"))
            {
                romset = ROM_SET_CM300;
                autodetect = false;
            }
            else if (!strcmp(argv[i], "-jv880"))
            {
                romset = ROM_SET_JV880;
                autodetect = false;
            }
            else if (!strcmp(argv[i], "-scb55"))
            {
                romset = ROM_SET_SCB55;
                autodetect = false;
            }
            else if (!strcmp(argv[i], "-rlp3237"))
            {
                romset = ROM_SET_RLP3237;
                autodetect = false;
            }
            else if (!strcmp(argv[i], "-gs"))
            {
                resetType = ResetType::GS_RESET;
            }
            else if (!strcmp(argv[i], "-gm"))
            {
                resetType = ResetType::GM_RESET;
            }
            else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "-help") || !strcmp(argv[i], "--help"))
            {
                // TODO: Might want to try to find a way to print out the executable's actual name (without any full paths).
                printf("Usage: nuked-sc55 [options]\n");
                printf("\n");
                printf("  -h, -help, --help              Display this information.\n");
                printf("\n");
                printf("MIDI / audio:\n");
                printf("  -p:<port_number>               Set MIDI port.\n");
        printf("  -nomidi                        Do not open the host MIDI input; keeps oracle\n"
               "                                 runs deterministic (no external bytes injected).\n");
                printf("  -a:<device_number>             Set Audio Device index.\n");
                printf("  -ab:<page_size>:[page_count]   Set Audio Buffer size.\n");
                printf("  -gain:<amount>                 Set overall output volume: a linear multiplier\n"
                       "                                 (e.g. 2 = double, 0.5 = half) or decibels (e.g.\n"
                       "                                 6db = double, -6db = half). scale = 10^(db/20).\n");
                printf("  -float                         Pure-float resonant filter (normalized\n"
                       "                                 scalar domain, no per-add saturation),\n"
                       "                                 all ROM sets.\n");
                printf("\n");
                printf("ROM set:\n");
                printf("  -mk2                           Use SC-55mk2 ROM set.\n");
                printf("  -mk1                           Use SC-55mk1 ROM set.\n");
                printf("  -st                            Use SC-55st ROM set.\n");
                printf("  -cm300                         Use CM-300/SCC-1 ROM set.\n");
                printf("  -jv880                         Use JV-880 ROM set.\n");
                printf("  -scb55                         Use SCB-55 ROM set.\n");
                printf("  -rlp3237                       Use RLP-3237 ROM set.\n");
                printf("  -sc155                         Use SC-155 ROM set.\n");
                printf("  -sc155mk2                      Use SC-155mk2 ROM set.\n");
                printf("\n");
                printf("Reset mode:\n");
                printf("  -gs                            Reset system in GS mode.\n");
                printf("  -gm                            Reset system in GM mode.\n");
                printf("\n");
                printf("Test / verification (for VM cross-checks):\n");
                printf("  -demo [cycles]                 Apply the built-in demo key sequence starting at\n"
                        "                                 <cycles> (default 144000000 = 6s, min 144000000,\n"
                        "                                 after the LCD splash animation).\n");
                printf("  -mocknote [cycles]             Post a MIDI note-on (0x90, note 60, vel 100) at\n"
                        "                                 <cycles> (default 144000000, min 144000000) to\n"
                        "                                 exercise the SM UART RX path without a MIDI device.\n"
                        "                                 The mk2 rejects MIDI during the demo phase, so it\n"
                        "                                 takes effect only if it fires before -demo.\n");
                printf("  -tracepc <file> [start end]    Log a unified main+SM PC trace to <file> for the\n"
                       "                                 [start, end) cycle window (default 200M..210M).\n");
                printf("  -pcmtrace                      Log PCM control-register writes to pcm_trace.log:\n"
                       "                                 stock window regs 00-03/3c-3f, the 0xE800 extension\n"
                       "                                 writes and the 0xE820/0xE821 extension reads.\n");
                printf("  -savesnap <cycles>             Dump the full machine state to demo_snap.bin at\n"
                       "                                 <cycles>.\n");
                printf("  -hashdump <cycles> <file>      Write a deterministic text state dump with FNV-1a\n"
                       "                                 hashes to <file> at <cycles> (once, keeps running).\n");
                printf("  -loadsnap <file>               Load the full machine state from <file> at start\n"
                       "                                 (a keyless run starts from the exact saved state).\n");
                printf("  -wav:<file>                    Write the producer-side int16 stereo sample stream\n"
                       "                                 (after -gain and the int16 clamp) to a standard WAVE\n"
                       "                                 file (PCM=1, 2ch, 16-bit, emulated rate). Default off.\n");
                printf("  -audiowin <start> <end>        With -wav: capture only samples with mcu.cycles in\n"
                       "                                 [start,end). At <end> the WAV sizes are backfilled and\n"
                       "                                 the file closed, then <file>.meta (wavdump v1) is written\n"
                       "                                 last as the completion marker.\n");
                printf("  -audiohash <cycles> <file>     Write the FNV-1a64 of the int16 sample stream up to\n"
                       "                                 <cycles> to <file> (one audio_fnv1a line). May be\n"
                       "                                 repeated for multiple checkpoints.\n");
                printf("  -midiseq <file> [start]        Post a text MIDI schedule (\"<cycle> <hexbyte>...\"\n"
                       "                                 per line, '#' comments; cycles relative to <start>,\n"
                       "                                 default 0) through the normal UART input path; the\n"
                       "                                 8192-byte ring is never overwritten (backpressure).\n");
                printf("  -snapinfo <cycles> <file>      Write text scalars at <cycles> to <file>: isr4fe\n"
                       "                                 dispatch count, sleep/iml/pend, pcm.select_channel,\n"
                       "                                 pcm.irq_channel and the voice-mask popcount.\n");
                printf("\n");
                printf("Native core:\n");
                printf("  -mk2cpp                        Use the translated mk2cpp core when linked (mixed with\n"
                        "                                 the GT interpreter as needed).\n");
                printf("  -mk2cpp-hand:0|1               Enable (1, default) or skip (0) the M4 native hand\n"
                       "                                 table for the same-binary A/B gate.\n");
                return 0;
            }
            else if (!strcmp(argv[i], "-sc155"))
            {
                romset = ROM_SET_SC155;
                autodetect = false;
            }
            else if (!strcmp(argv[i], "-sc155mk2"))
            {
                romset = ROM_SET_SC155MK2;
                autodetect = false;
            }
        }
    }

    // ---- mk2cpp post-parse hooks (hand override flag) ----
    MK2CPP_Configure(argc, argv);

    // ---- M4 oracle setup (no-ops unless the matching options were given) ----
    audio_capture_on = (g_wav_path != nullptr) || (g_audio_hash_n > 0);
    if (g_midiseq_file)
        midiseq_load(g_midiseq_file);

    // ---- validate -demo / -mocknote start cycles (both must be >= 144M) ----
    if (demo_seq_enabled && demo_start < 144000000)
    {
        printf("warning: -demo start c%llu < 144M, clamped to 144M\n", (unsigned long long)demo_start);
        demo_start = 144000000;
    }
    if (mocknote_enabled && mocknote_at < 144000000)
    {
        printf("warning: -mocknote c%llu < 144M, clamped to 144M\n", (unsigned long long)mocknote_at);
        mocknote_at = 144000000;
    }

    // ---- warn when both -demo and -mocknote are enabled ----
    // The SC-55mk2 rejects MIDI while in the demo phase, so the note only takes
    // effect if it fires BEFORE the demo starts.
    if (demo_seq_enabled && mocknote_enabled)
    {
        printf("warning: -demo (start c%llu) and -mocknote (c%llu) both enabled.\n",
               (unsigned long long)demo_start, (unsigned long long)mocknote_at);
        printf("         The SC-55mk2 rejects MIDI during the demo phase, so the note\n"
               "         takes effect only if it fires before the demo (mocknote < demo start).\n");
        if (mocknote_at >= demo_start)
            printf("         NOTE: mocknote c%llu >= demo start c%llu -> the note will likely be rejected.\n",
                   (unsigned long long)mocknote_at, (unsigned long long)demo_start);
        fflush(stdout);
    }

#if __linux__
    char self_path[PATH_MAX];
    memset(&self_path[0], 0, PATH_MAX);

    if(readlink("/proc/self/exe", self_path, PATH_MAX) == -1)
        basePath = Files::real_dirname(argv[0]);
    else
        basePath = Files::dirname(self_path);
#else
    basePath = Files::real_dirname(argv[0]);
#endif

    printf("Base path is: %s\n", argv[0]);

    if(Files::dirExists(basePath + "/../share/nuked-sc55"))
        basePath += "/../share/nuked-sc55";

    if (autodetect)
    {
        for (size_t i = 0; i < ROM_SET_COUNT; i++)
        {
            bool good = true;
            for (size_t j = 0; j < 5; j++)
            {
                if (roms[i][j][0] == '\0')
                    continue;
                std::string path = basePath + "/" + roms[i][j];
                auto h = Files::utf8_fopen(path.c_str(), "rb");
                if (!h)
                {
                    good = false;
                    break;
                }
                fclose(h);
            }
            if (good)
            {
                romset = i;
                break;
            }
        }
        printf("ROM set autodetect: %s\n", rs_name[romset]);
    }

    if (pcm_float)
        printf("Filter: running in float precision (no per-add saturation).\n");

    if (master_gain != 1.0f)
        printf("Output gain: %.3f\n", master_gain);

    mcu_mk1 = false;
    mcu_cm300 = false;
    mcu_st = false;
    mcu_jv880 = false;
    mcu_scb55 = false;
    mcu_sc155 = false;
    switch (romset)
    {
        case ROM_SET_MK2:
        case ROM_SET_SC155MK2:
            if (romset == ROM_SET_SC155MK2)
                mcu_sc155 = true;
            break;
        case ROM_SET_ST:
            mcu_st = true;
            break;
        case ROM_SET_MK1:
        case ROM_SET_SC155:
            mcu_mk1 = true;
            mcu_st = false;
            if (romset == ROM_SET_SC155)
                mcu_sc155 = true;
            break;
        case ROM_SET_CM300:
            mcu_mk1 = true;
            mcu_cm300 = true;
            break;
        case ROM_SET_JV880:
            mcu_jv880 = true;
            rom2_mask /= 2; // rom is half the size
            lcd_width = 820;
            lcd_height = 100;
            lcd_col1 = 0x000000;
            lcd_col2 = 0x78b500;
            break;
        case ROM_SET_SCB55:
        case ROM_SET_RLP3237:
            mcu_scb55 = true;
            break;
    }

    // ---- M4 audio tap: open the WAV once the ROM set (rate) is known ----
    g_wav_rate = (mcu_mk1 || mcu_jv880) ? 64000 : 66207;
    if (g_wav_path)
        audio_dump_open(g_wav_path);

    std::string rpaths[ROM_SET_N_FILES];

    bool r_ok = true;
    std::string errors_list;

    for(size_t i = 0; i < ROM_SET_N_FILES; ++i)
    {
        if (roms[romset][i][0] == '\0')
        {
            rpaths[i] = "";
            continue;
        }
        rpaths[i] = basePath + "/" + roms[romset][i];
        s_rf[i] = Files::utf8_fopen(rpaths[i].c_str(), "rb");
        bool optional = mcu_jv880 && i >= 4;
        r_ok &= optional || (s_rf[i] != nullptr);
        if(!s_rf[i])
        {
            if(!errors_list.empty())
                errors_list.append(", ");

            errors_list.append(rpaths[i]);
        }
    }

    if (!r_ok)
    {
        fprintf(stderr, "FATAL ERROR: One of required data ROM files is missing: %s.\n", errors_list.c_str());
        fflush(stderr);
        closeAllR();
        return 1;
    }

    LCD_SetBackPath(basePath + "/back.data");

    memset(&mcu, 0, sizeof(mcu_t));


    if (fread(rom1, 1, ROM1_SIZE, s_rf[0]) != ROM1_SIZE)
    {
        fprintf(stderr, "FATAL ERROR: Failed to read the mcu ROM1.\n");
        fflush(stderr);
        closeAllR();
        return 1;
    }

    size_t rom2_read = fread(rom2, 1, ROM2_SIZE, s_rf[1]);

    if (rom2_read == ROM2_SIZE || rom2_read == ROM2_SIZE / 2)
    {
        rom2_mask = rom2_read - 1;
    }
    else
    {
        fprintf(stderr, "FATAL ERROR: Failed to read the mcu ROM2.\n");
        fflush(stderr);
        closeAllR();
        return 1;
    }

    if (mcu_mk1)
    {
        if (fread(tempbuf, 1, 0x100000, s_rf[2]) != 0x100000)
        {
            fprintf(stderr, "FATAL ERROR: Failed to read the WaveRom1.\n");
            fflush(stderr);
            closeAllR();
            return 1;
        }

        unscramble(tempbuf, waverom1, 0x100000);

        if (fread(tempbuf, 1, 0x100000, s_rf[3]) != 0x100000)
        {
            fprintf(stderr, "FATAL ERROR: Failed to read the WaveRom2.\n");
            fflush(stderr);
            closeAllR();
            return 1;
        }

        unscramble(tempbuf, waverom2, 0x100000);

        if (fread(tempbuf, 1, 0x100000, s_rf[4]) != 0x100000)
        {
            fprintf(stderr, "FATAL ERROR: Failed to read the WaveRom3.\n");
            fflush(stderr);
            closeAllR();
            return 1;
        }

        unscramble(tempbuf, waverom3, 0x100000);
    }
    else if (mcu_jv880)
    {
        if (fread(tempbuf, 1, 0x200000, s_rf[2]) != 0x200000)
        {
            fprintf(stderr, "FATAL ERROR: Failed to read the WaveRom1.\n");
            fflush(stderr);
            closeAllR();
            return 1;
        }

        unscramble(tempbuf, waverom1, 0x200000);

        if (fread(tempbuf, 1, 0x200000, s_rf[3]) != 0x200000)
        {
            fprintf(stderr, "FATAL ERROR: Failed to read the WaveRom2.\n");
            fflush(stderr);
            closeAllR();
            return 1;
        }

        unscramble(tempbuf, waverom2, 0x200000);
        
        if (s_rf[4] && fread(tempbuf, 1, 0x800000, s_rf[4]))
            unscramble(tempbuf, waverom_exp, 0x800000);
        else
            printf("WaveRom EXP not found, skipping it.\n");
        
        if (s_rf[5] && fread(tempbuf, 1, 0x200000, s_rf[5]))
            unscramble(tempbuf, waverom_card, 0x200000);
        else
            printf("WaveRom PCM not found, skipping it.\n");
    }
    else
    {
        if (fread(tempbuf, 1, 0x200000, s_rf[2]) != 0x200000)
        {
            fprintf(stderr, "FATAL ERROR: Failed to read the WaveRom1.\n");
            fflush(stderr);
            closeAllR();
            return 1;
        }

        unscramble(tempbuf, waverom1, 0x200000);

        if (s_rf[3])
        {
            if (fread(tempbuf, 1, 0x100000, s_rf[3]) != 0x100000)
            {
                fprintf(stderr, "FATAL ERROR: Failed to read the WaveRom2.\n");
                fflush(stderr);
                closeAllR();
                return 1;
            }

            unscramble(tempbuf, mcu_scb55 ? waverom3 : waverom2, 0x100000);
        }

        if (s_rf[4] && fread(sm_rom, 1, ROMSM_SIZE, s_rf[4]) != ROMSM_SIZE)
        {
            fprintf(stderr, "FATAL ERROR: Failed to read the sub mcu ROM.\n");
            fflush(stderr);
            closeAllR();
            return 1;
        }
    }

    // Close all files as they no longer needed being open
    closeAllR();

    if (SDL_Init(SDL_INIT_AUDIO | SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
    {
        fprintf(stderr, "FATAL ERROR: Failed to initialize the SDL2: %s.\n", SDL_GetError());
        fflush(stderr);
        return 2;
    }

    if (!MCU_OpenAudio(audioDeviceIndex, pageSize, pageNum))
    {
        fprintf(stderr, "FATAL ERROR: Failed to open the audio stream.\n");
        fflush(stderr);
        return 2;
    }

    if (nomidi)
    {
        printf("MIDI input disabled (-nomidi)\n");
    }
    else if(!MIDI_Init(port))
    {
        fprintf(stderr, "ERROR: Failed to initialize the MIDI Input.\nWARNING: Continuing without MIDI Input...\n");
        fflush(stderr);
    }

    LCD_Init();
    MCU_Init();
    MCU_PatchROM();
    MCU_Reset();
    SM_Reset();
    PCM_Reset();
    // ---- mk2cpp native engine post-reset hook (extension activation point) ----
    MK2CPP_PostReset();

    if (resetType != ResetType::NONE) MIDI_Reset(resetType);

    MCU_Run();

    // Normal exit (window closed): backfill and close any open WAV. The
    // -audiowin path already did this at the end cycle; atexit is the backstop.
    audio_dump_finalize();

    MCU_CloseAudio();
    MIDI_Quit();
    LCD_UnInit();
    SDL_Quit();

    return 0;
}

