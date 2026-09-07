// Standalone PCM device header.
#pragma once
#include <stdint.h>

struct pcmdev_t {
    uint32_t ram1[32][8];
    uint16_t ram2[32][16];
    uint32_t select_channel;
    uint32_t voice_mask;
    uint32_t voice_mask_pending;
    uint32_t voice_mask_updating;
    uint32_t write_latch;
    uint32_t wave_read_address;
    uint8_t wave_byte_latch;
    uint32_t read_latch;
    uint8_t config_reg_3c;
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
    float fstate[32][2];
};
typedef struct pcmdev_t pcmdev_t;

extern pcmdev_t pcmdev;

void PCMDev_Reset(void);
int PCMDev_LoadWaveRoms(const char *path1, const char *path2);
uint8_t PCMDev_ReadROM(uint32_t address);
void PCMDev_Write(uint32_t address, uint8_t data);
uint8_t PCMDev_Read(uint32_t address);
void PCMDev_OnIRQDeassert(void);
void PCMDev_Update(uint64_t cycles);
