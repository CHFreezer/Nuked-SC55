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
 *  Thanks:
 *      John McMaster (https://siliconprawn.org):
 *          PCM chip decap
 *
 */
#pragma once
#include <stdint.h>

// Polyphony extension constants (mkII, in-memory ROM patch, -voices:<n>):
//   voices 0..PCM_MAX_VOICE-1; effects live at PCM_EFF_BASE..PCM_EFF_BASE+3
//   (fixed slots 28..31 when the extension is disabled).
#define PCM_MAX_VOICE 255
#define PCM_EFF_BASE 256
#define PCM_SLOTS 260

struct pcm_t {
    uint32_t ram1[PCM_SLOTS][8];
    uint16_t ram2[PCM_SLOTS][16];
    uint32_t select_channel;
    // 256-bit voice enable mask, bit N = voice N; byte i = voices 8i..8i+7.
    uint8_t voice_mask[32];
    uint8_t voice_mask_pending[32];
    uint32_t voice_mask_updating;
    uint32_t write_latch;
    uint32_t wave_read_address;
    uint8_t wave_byte_latch;
    uint32_t read_latch;
    uint8_t config_reg_3c; // SC55:c3 JV880:c0
    uint8_t config_reg_3d;
    uint32_t irq_channel;
    uint32_t irq_assert;

    uint32_t nfs;

    uint32_t tv_counter;

    uint64_t cycles;

    uint16_t eram[0x4000];

    int accum_l;
    int accum_r;
    int rcsum[2];

    float fstate[PCM_SLOTS][2]; // S1: float filter state (state1/state2), normalized ±1.0
};

extern pcm_t pcm;
extern uint8_t waverom1[];
extern uint8_t waverom2[];
extern uint8_t waverom3[];
extern uint8_t waverom_card[];
extern uint8_t waverom_exp[];

void PCM_Write(uint32_t address, uint8_t data);
uint8_t PCM_Read(uint32_t address);
// Extended window (0xe800-0xe83f on the H8 side): high 224 bits of the voice
// enable mask (offsets 0x00..0x1b -> voices 32..255) + full IRQ channel (0x20,
// read only). Active only when the polyphony patch is applied.
void PCM_WriteExt(uint32_t address, uint8_t data);
uint8_t PCM_ReadExt(uint32_t address);
void PCM_Reset(void);
void PCM_Update(uint64_t cycles);
