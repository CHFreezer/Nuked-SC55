// Standalone PCM device register file (ported verbatim from nukeykt pcm.cpp PCM_Read/PCM_Write).
// The 32-bit voice mask is registers 0x00-0x03; 0x3c/0x3d config; 0x3e channel select + status;
// 0x3f wave byte latch; 0x39-0x3b read-latch output; per-channel ram1/ram2 latched writes.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "h8vm.h"
#include "h8vm_pcm.h"

pcmdev_t pcmdev;

static uint8_t waverom1[0x200000];
static uint8_t waverom2[0x100000];

void PCMDev_Reset(void)
{
    memset(&pcmdev, 0, sizeof(pcmdev));
}

static void wave_unscramble(const uint8_t *src, uint8_t *dst, int len)
{
    static const int aa[] = {
        2, 0, 3, 4, 1, 9, 13, 10, 18, 17, 6, 15, 11, 16, 8, 5, 12, 7, 14, 19
    };
    static const int dd[] = {
        2, 0, 4, 5, 7, 6, 3, 1
    };
    for (int i = 0; i < len; i++)
    {
        int address = i & ~0xfffff;
        for (int j = 0; j < 20; j++)
        {
            if (i & (1 << j))
                address |= 1 << aa[j];
        }
        uint8_t srcdata = src[address];
        uint8_t data = 0;
        for (int j = 0; j < 8; j++)
        {
            if (srcdata & (1 << dd[j]))
                data |= 1 << j;
        }
        dst[i] = data;
    }
}

int PCMDev_LoadWaveRoms(const char *path1, const char *path2)
{
    int ok = 1;
    uint8_t *tmp1 = (uint8_t *)malloc(sizeof(waverom1));
    uint8_t *tmp2 = (uint8_t *)malloc(sizeof(waverom2));
    if (path1)
    {
        FILE *f = fopen(path1, "rb");
        if (f)
        {
            size_t n = fread(tmp1, 1, sizeof(waverom1), f);
            if (n != sizeof(waverom1)) ok = 0;
            fclose(f);
            if (ok) wave_unscramble(tmp1, waverom1, sizeof(waverom1));
        }
        else ok = 0;
    }
    if (path2)
    {
        FILE *f = fopen(path2, "rb");
        if (f)
        {
            size_t n = fread(tmp2, 1, sizeof(waverom2), f);
            if (n != sizeof(waverom2)) ok = 0;
            fclose(f);
            if (ok) wave_unscramble(tmp2, waverom2, sizeof(waverom2));
        }
        else ok = 0;
    }
    free(tmp1);
    free(tmp2);
    return ok;
}

uint8_t PCMDev_ReadROM(uint32_t address)
{
    int bank;
    if (pcmdev.config_reg_3d & 0x20)
        bank = (address >> 21) & 7;
    else
        bank = (address >> 19) & 7;
    switch (bank)
    {
        case 0:
            return waverom1[address & 0x1fffff];
        case 1:
            return waverom2[address & 0xfffff];
        default:
            break;
    }
    return 0;
}

void PCMDev_Write(uint32_t address, uint8_t data)
{
    address &= 0x3f;
    if (address < 0x4) // voice enable
    {
        switch (address & 3)
        {
            case 0:
                pcmdev.voice_mask_pending &= ~0xf000000;
                pcmdev.voice_mask_pending |= (data & 0xf) << 24;
                break;
            case 1:
                pcmdev.voice_mask_pending &= ~0xff0000;
                pcmdev.voice_mask_pending |= (data & 0xff) << 16;
                break;
            case 2:
                pcmdev.voice_mask_pending &= ~0xff00;
                pcmdev.voice_mask_pending |= (data & 0xff) << 8;
                break;
            case 3:
                pcmdev.voice_mask_pending &= ~0xff;
                pcmdev.voice_mask_pending |= (data & 0xff) << 0;
                break;
        }
        pcmdev.voice_mask_updating = 1;
    }
    else if (address >= 0x20 && address < 0x24) // wave rom
    {
        switch (address & 3)
        {
            case 1:
                pcmdev.wave_read_address &= ~0xff0000;
                pcmdev.wave_read_address |= (data & 0xff) << 16;
                break;
            case 2:
                pcmdev.wave_read_address &= ~0xff00;
                pcmdev.wave_read_address |= (data & 0xff) << 8;
                break;
            case 3:
                pcmdev.wave_read_address &= ~0xff;
                pcmdev.wave_read_address |= (data & 0xff) << 0;
                pcmdev.wave_byte_latch = PCMDev_ReadROM(pcmdev.wave_read_address);
                break;
        }
    }
    else if (address == 0x3c)
    {
        pcmdev.config_reg_3c = data;
    }
    else if (address == 0x3d)
    {
        pcmdev.config_reg_3d = data;
    }
    else if (address == 0x3e)
    {
        pcmdev.select_channel = data & 0x1f;
    }
    else if ((address >= 0x4 && address < 0x10) || (address >= 0x24 && address < 0x30))
    {
        switch (address & 3)
        {
            case 1:
                pcmdev.write_latch &= ~0xf0000;
                pcmdev.write_latch |= (data & 0xf) << 16;
                break;
            case 2:
                pcmdev.write_latch &= ~0xff00;
                pcmdev.write_latch |= (data & 0xff) << 8;
                break;
            case 3:
                pcmdev.write_latch &= ~0xff;
                pcmdev.write_latch |= (data & 0xff) << 0;
                break;
        }
        if ((address & 3) == 3)
        {
            int ix = 0;
            if (address & 32)
                ix |= 1;
            if ((address & 8) == 0)
                ix |= 4;
            if ((address & 4) == 0)
                ix |= 2;

            pcmdev.ram1[pcmdev.select_channel][ix] = pcmdev.write_latch;
        }
    }
    else if ((address >= 0x10 && address < 0x20) || (address >= 0x30 && address < 0x38))
    {
        switch (address & 1)
        {
            case 0:
                pcmdev.write_latch &= ~0xff00;
                pcmdev.write_latch |= (data & 0xff) << 8;
                break;
            case 1:
                pcmdev.write_latch &= ~0xff;
                pcmdev.write_latch |= (data & 0xff) << 0;
                break;
        }
        if ((address & 1) == 1)
        {
            int ix = (address >> 1) & 7;
            if (address & 32)
                ix |= 8;

            pcmdev.ram2[pcmdev.select_channel][ix] = (uint16_t)pcmdev.write_latch;
        }
    }
}

uint8_t PCMDev_Read(uint32_t address)
{
    address &= 0x3f;

    if (address < 0x4)
    {
        if (pcmdev.voice_mask_updating)
            pcmdev.voice_mask = pcmdev.voice_mask_pending;
        pcmdev.voice_mask_updating = 0;
    }
    else if (address == 0x3c || address == 0x3e) // status
    {
        uint8_t status = 0;
        if (address == 0x3e && pcmdev.irq_assert)
        {
            pcmdev.irq_assert = 0;
            PCMDev_OnIRQDeassert();
        }

        status |= pcmdev.irq_channel;
        if (pcmdev.voice_mask_updating)
            status |= 32;

        return status;
    }
    else if (address == 0x3f)
    {
        return pcmdev.wave_byte_latch;
    }
    else if ((address >= 0x4 && address < 0x10) || (address >= 0x24 && address < 0x30))
    {
        if ((address & 3) == 1)
        {
            int ix = 0;
            if (address & 32)
                ix |= 1;
            if ((address & 8) == 0)
                ix |= 4;
            if ((address & 4) == 0)
                ix |= 2;

            pcmdev.read_latch = pcmdev.ram1[pcmdev.select_channel][ix];
        }
    }
    else if ((address >= 0x10 && address < 0x20) || (address >= 0x30 && address < 0x38))
    {
        if ((address & 1) == 0)
        {
            int ix = (address >> 1) & 7;
            if (address & 32)
                ix |= 8;

            pcmdev.read_latch = pcmdev.ram2[pcmdev.select_channel][ix];
        }
    }
    else if (address >= 0x39 && address <= 0x3b)
    {
        switch (address & 3)
        {
            case 1:
                return (pcmdev.read_latch >> 16) & 0xf;
            case 2:
                return (pcmdev.read_latch >> 8) & 0xff;
            case 3:
                return (pcmdev.read_latch >> 0) & 0xff;
        }
    }

    return 0;
}

// ---- PCM audio engine (ported verbatim from nukeykt pcm.cpp PCM_Update) ----

#define vm_mcu_mk1 0
#define vm_mcu_jv880 0
#define vm_mcu_scb55 0
static int pcm_float = 0;

static void MCU_PostSample(int *sample)
{
    (void)sample;
}

static void MCU_GA_SetGAInt(int idx, int value)
{
    (void)idx; (void)value;
}

static inline uint32_t addclip20(uint32_t add1, uint32_t add2, uint32_t cin)
{
    uint32_t sum = (add1 + add2 + cin) & 0xfffff;
    if ((add1 & 0x80000) != 0 && (add2 & 0x80000) != 0 && (sum & 0x80000) == 0)
        sum = 0x80000;
    else if ((add1 & 0x80000) == 0 && (add2 & 0x80000) == 0 && (sum & 0x80000) != 0)
        sum = 0x7ffff;
    return sum;
}

static inline int32_t multi(int32_t val1, int8_t val2)
{
    if (val1 & 0x80000)
        val1 |= ~0xfffff;
    else
        val1 &= 0x7ffff;

    val1 *= val2;
    if (val1 & 0x8000000)
        val1 |= ~0x1ffffff;
    else
        val1 &= 0x1ffffff;
    return val1;
}

static const int interp_lut[3][128] = {
    3385, 3401, 3417, 3432, 3448, 3463, 3478, 3492, 3506, 3521, 3535, 3548, 3562, 3575, 3588, 3601,
    3614, 3626, 3638, 3650, 3662, 3673, 3685, 3696, 3707, 3718, 3728, 3739, 3749, 3759, 3768, 3778,
    3787, 3796, 3805, 3814, 3823, 3831, 3839, 3847, 3855, 3863, 3870, 3878, 3885, 3892, 3899, 3905,
    3912, 3918, 3924, 3930, 3936, 3942, 3948, 3953, 3958, 3963, 3968, 3973, 3978, 3983, 3987, 3991,
    3995, 4000, 4004, 4007, 4011, 4015, 4018, 4022, 4025, 4028, 4031, 4034, 4037, 4040, 4042, 4045,
    4047, 4050, 4052, 4054, 4057, 4059, 4061, 4063, 4064, 4066, 4068, 4070, 4071, 4073, 4074, 4076,
    4077, 4078, 4079, 4081, 4082, 4083, 4084, 4085, 4086, 4086, 4087, 4088, 4089, 4089, 4090, 4091,
    4091, 4092, 4092, 4093, 4093, 4094, 4094, 4094, 4094, 4095, 4095, 4095, 4095, 4095, 4095, 4095,

    710, 726, 742, 758, 775, 792, 809, 826, 844, 861, 879, 897, 915, 933, 952, 971,
    990, 1009, 1028, 1047, 1067, 1087, 1106, 1126, 1147, 1167, 1188, 1208, 1229, 1250, 1271, 1292,
    1314, 1335, 1357, 1379, 1400, 1423, 1445, 1467, 1489, 1512, 1534, 1557, 1580, 1602, 1625, 1648,
    1671, 1695, 1718, 1741, 1764, 1788, 1811, 1835, 1858, 1882, 1906, 1929, 1953, 1977, 2000, 2024,
    2048, 2071, 2095, 2119, 2143, 2166, 2190, 2214, 2237, 2261, 2284, 2308, 2331, 2355, 2378, 2401,
    2425, 2448, 2471, 2494, 2517, 2539, 2562, 2585, 2607, 2630, 2652, 2674, 2696, 2718, 2740, 2762,
    2783, 2805, 2826, 2847, 2868, 2889, 2910, 2931, 2951, 2971, 2991, 3011, 3031, 3051, 3070, 3089,
    3108, 3127, 3146, 3164, 3182, 3200, 3218, 3236, 3253, 3271, 3288, 3304, 3321, 3338, 3354, 3370,

    0, 0, 0, 1, 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6,
    6, 7, 8, 8, 9, 10, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 22, 23, 24, 26, 27, 29, 30, 32, 34, 36, 38, 40, 42, 44, 46,
    49, 51, 53, 56, 59, 62, 65, 68, 71, 74, 77, 81, 84, 88, 92, 96,
    100, 104, 109, 113, 118, 122, 127, 132, 137, 143, 148, 154, 160, 165, 171, 178,
    184, 191, 197, 204, 211, 219, 226, 234, 241, 249, 257, 266, 274, 283, 292, 301,
    310, 319, 329, 339, 349, 359, 369, 380, 391, 402, 413, 424, 436, 448, 460, 472,
    484, 497, 510, 523, 536, 549, 563, 577, 591, 605, 619, 634, 648, 663, 679, 694,
};

static inline void calc_tv(int e, int adjust, uint16_t *levelcur, int active, int *volmul)
{
    *levelcur &= 0x7fff;
    int speed = adjust & 0xff;
    int target = (adjust >> 8) & 0xff;

    int w1 = (speed & 0xf0) == 0;
    int w2 = w1 || (speed & 0x10) != 0;
    int w3 = pcmdev.nfs &&
        ((speed & 0x80) == 0 || ((speed & 0x40) == 0 && (!w2 || (speed & 0x20) == 0)));

    int type = w2 | (w3 << 3);
    if (speed & 0x20)
        type |= 2;
    if ((speed & 0x80) == 0 || (speed & 0x40) == 0)
        type |= 4;

    int write = !active;
    int addlow = 0;
    if (type & 4)
    {
        if (pcmdev.tv_counter & 8)
            addlow |= 1;
        if (pcmdev.tv_counter & 4)
            addlow |= 2;
        if (pcmdev.tv_counter & 2)
            addlow |= 4;
        if (pcmdev.tv_counter & 1)
            addlow |= 8;
        write |= 1;
    }
    else
    {
        switch (type & 3)
        {
        case 0:
            if (pcmdev.tv_counter & 0x20)
                addlow |= 1;
            if (pcmdev.tv_counter & 0x10)
                addlow |= 2;
            if (pcmdev.tv_counter & 8)
                addlow |= 4;
            if (pcmdev.tv_counter & 4)
                addlow |= 8;
            write |= (pcmdev.tv_counter & 3) == 0;
            break;
        case 1:
            if (pcmdev.tv_counter & 0x80)
                addlow |= 1;
            if (pcmdev.tv_counter & 0x40)
                addlow |= 2;
            if (pcmdev.tv_counter & 0x20)
                addlow |= 4;
            if (pcmdev.tv_counter & 0x10)
                addlow |= 8;
            write |= (pcmdev.tv_counter & 15) == 0;
            break;
        case 2:
            if (pcmdev.tv_counter & 0x200)
                addlow |= 1;
            if (pcmdev.tv_counter & 0x100)
                addlow |= 2;
            if (pcmdev.tv_counter & 0x80)
                addlow |= 4;
            if (pcmdev.tv_counter & 0x40)
                addlow |= 8;
            write |= (pcmdev.tv_counter & 63) == 0;
            break;
        case 3:
            if (pcmdev.tv_counter & 0x800)
                addlow |= 1;
            if (pcmdev.tv_counter & 0x400)
                addlow |= 2;
            if (pcmdev.tv_counter & 0x200)
                addlow |= 4;
            if (pcmdev.tv_counter & 0x100)
                addlow |= 8;
            write |= (pcmdev.tv_counter & 127) == 0;
            break;
        }
    }

    if ((type & 8) == 0)
    {
        int shift = speed & 15;
        shift = (10 - shift) & 15;

        int sum1 = (target << 11);
        if (e != 2 || active)
            sum1 -= (*levelcur << 4);
        int neg = (sum1 & 0x80000) != 0;

        int preshift = sum1;

        int shifted = preshift >> shift;
        shifted -= sum1;

        int sum2 = (target << 11) + addlow + shifted;
        if (write && pcmdev.nfs)
            *levelcur = (sum2 >> 4) & 0x7fff;

        if (e == 0)
        {
            *volmul = (sum2 >> 4) & 0x7ffe;
        }
        else if (e == 1)
        {
            *volmul = (sum2 >> 4) & 0x7ffe;
        }
    }
    else
    {
        int shift = (speed >> 4) & 14;
        shift |= w2;
        shift = (10 - shift) & 15;

        int sum1 = target << 11;
        if (e != 2 || active)
            sum1 -= (*levelcur << 4);
        int neg = (sum1 & 0x80000) != 0;
        int preshift = (speed & 15) << 9;
        if (!w1)
            preshift |= 0x2000;
        if (neg)
            preshift ^= ~0x3f;

        int shifted = preshift >> shift;
        int sum2 = shifted;
        if (e != 2 || active)
            sum2 += (*levelcur << 4) | addlow;

        int sum2_l = (sum2 >> 4);

        int sum3 = (target << 11) - (sum2_l << 4);

        int neg2 = (sum3 & 0x80000) != 0;
        int xnor = !(neg2 ^ neg);

        if (write && pcmdev.nfs)
        {
            if (xnor)
                *levelcur = sum2_l & 0x7fff;
            else
                *levelcur = target << 7;
        }

        if (e == 0)
        {
            *volmul = sum2_l & 0x7ffe;
        }
        else if (e == 1)
        {
            if (xnor)
                *volmul = sum2_l & 0x7ffe;
            else
                *volmul = target << 7;
        }
    }
}

static inline int eram_unpack(int addr, int type)
{
    addr &= 0x3fff;
    int data = pcmdev.eram[addr];
    int val = data & 0x3fff;
    int sh = (data >> 14) & 3;

    val <<= 18;
    return val >> (18 - sh * 2 + type);
}

static inline void eram_pack(int addr, int val)
{
    addr &= 0x3fff;
    int sh = 0;
    int top = (val >> 13) & 0x7f;
    if (top & 0x40)
        top ^= 0x7f;
    if (top >= 16)
        sh = 3;
    else if (top >= 4)
        sh = 2;
    else if (top >= 1)
        sh = 1;
    else
        sh = 0;

    int data = (val >> (sh * 2)) & 0x3fff;
    data |= sh << 14;
    pcmdev.eram[addr] = data;
}

void PCMDev_Update(uint64_t cycles)
{
    int reg_slots = (pcmdev.config_reg_3d & 31) + 1;
    int voice_active = pcmdev.voice_mask & pcmdev.voice_mask_pending;
    while (pcmdev.cycles < cycles)
    {
        int tt[2] = {};

        { // final mixing
            int noise_mask = 0;
            int orval = 0;
            int write_mask = 0;
            int dac_mask = 0;
            if ((pcmdev.config_reg_3c & 0x30) != 0)
            {
                switch ((pcmdev.config_reg_3c >> 2) & 3)
                {
                    case 1:
                        noise_mask = 3;
                        break;
                    case 2:
                        noise_mask = 7;
                        break;
                    case 3:
                        noise_mask = 15;
                        break;
                }
                switch (pcmdev.config_reg_3c & 3)
                {
                    case 1:
                        orval |= 1 << 8;
                        break;
                    case 2:
                        orval |= 1 << 10;
                        break;
                }
                write_mask = 15;
                dac_mask = ~15;
            }
            else
            {
                switch ((pcmdev.config_reg_3c >> 2) & 3)
                {
                    case 2:
                        noise_mask = 1;
                        break;
                    case 3:
                        noise_mask = 3;
                        break;
                }
                switch (pcmdev.config_reg_3c & 3)
                {
                    case 1:
                        orval |= 1 << 6;
                        break;
                    case 2:
                        orval |= 1 << 8;
                        break;
                }
                write_mask = 3;
                dac_mask = ~3;
            }
            if ((pcmdev.config_reg_3c & 0x80) == 0)
                write_mask = 0;
            if ((pcmdev.config_reg_3c & 0x30) == 0x30)
                orval |= 1 << 12;


            int shifter = pcmdev.ram2[30][10];
            int xr = ((shifter >> 0) ^ (shifter >> 1) ^ (shifter >> 7) ^ (shifter >> 12)) & 1;
            shifter = (shifter >> 1) | (xr << 15);
            pcmdev.ram2[30][10] = shifter;

            pcmdev.accum_l = addclip20(pcmdev.accum_l, pcmdev.ram1[30][0], 0);
            pcmdev.accum_r = addclip20(pcmdev.accum_r, pcmdev.ram1[30][1], 0);

            pcmdev.ram1[30][2] = addclip20(pcmdev.accum_l,
                orval | (shifter & noise_mask), 0);

            pcmdev.ram1[30][4] = addclip20(pcmdev.accum_r,
                orval | (shifter & noise_mask), 0);

            pcmdev.ram1[30][0] = pcmdev.accum_l & write_mask;
            pcmdev.ram1[30][1] = pcmdev.accum_r & write_mask;

            tt[0] = (int)((pcmdev.ram1[30][2] & ~write_mask) << 12);
            tt[1] = (int)((pcmdev.ram1[30][4] & ~write_mask) << 12);

            MCU_PostSample(tt);

            xr = ((shifter >> 0) ^ (shifter >> 1) ^ (shifter >> 7) ^ (shifter >> 12)) & 1;
            shifter = (shifter >> 1) | (xr << 15);

            pcmdev.accum_l = addclip20(pcmdev.accum_l, pcmdev.ram1[30][0], 0);
            pcmdev.accum_r = addclip20(pcmdev.accum_r, pcmdev.ram1[30][1], 0);

            pcmdev.ram1[30][3] = addclip20(pcmdev.accum_l,
                orval | (shifter & noise_mask), 0);

            pcmdev.ram1[30][5] = addclip20(pcmdev.accum_r,
                orval | (shifter & noise_mask), 0);

            if (pcmdev.config_reg_3c & 0x40) // oversampling
            {
                pcmdev.ram2[30][10] = shifter;

                pcmdev.ram1[30][0] = pcmdev.accum_l & write_mask;
                pcmdev.ram1[30][1] = pcmdev.accum_r & write_mask;

                tt[0] = (int)((pcmdev.ram1[30][3] & ~write_mask) << 12);
                tt[1] = (int)((pcmdev.ram1[30][5] & ~write_mask) << 12);

                MCU_PostSample(tt);
            }
        }

        { // global counter for envelopes
            if (!pcmdev.nfs)
                pcmdev.tv_counter = pcmdev.ram2[31][8]; // fixme

            pcmdev.tv_counter -= 1;

            pcmdev.tv_counter &= 0x3fff;
        }

        // chorus/reverb

        { // fixme
            if (pcmdev.ram2[31][8] & 0x8000)
                pcmdev.ram2[31][9] = pcmdev.ram2[31][8] & 0x7fff;
            else
                pcmdev.ram2[31][10] = pcmdev.ram2[31][8] & 0x7fff;

            if ((0x4000 - pcmdev.ram2[31][8]) & 0x8000)
                pcmdev.ram2[31][10] = (0x4000 - pcmdev.ram2[31][8]) & 0x7fff;
            else
                pcmdev.ram2[31][9] = (0x4000 - pcmdev.ram2[31][8]) & 0x7fff;
        }

        {
            int v1 = pcmdev.ram2[31][1];

            int m1 = multi(pcmdev.ram1[29][1], v1 >> 8) >> 5; // 14
            int m2 = multi(pcmdev.rcsum[1], v1 & 255) >> 5; // 15

            pcmdev.ram1[29][1] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1); // 16
        }

        {
            int okey = (pcmdev.ram2[31][7] & 0x20) != 0;
            int key = 1;
            int active = okey && key;
            int u = 0;
            calc_tv(1, pcmdev.ram2[30][0], &pcmdev.ram2[30][9], active, &u);
        }

        {
            int v1 = pcmdev.ram2[30][1];
            int m1 = multi(pcmdev.ram1[29][0], v1 >> 8) >> 5; // 17
            int m2 = multi(pcmdev.rcsum[0], v1 & 255) >> 5; // 18

            pcmdev.ram1[29][0] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1); // 19
        }

        int rcadd[6] = {};
        int rcadd2[6] = {};

        {
            {
                // 1
                int v1 = pcmdev.ram2[30][4];
                int m1 = multi(pcmdev.ram1[29][0], (v1 >> 8)) >> 6;
                int v2 = 0;
                int s1 = eram_unpack(pcmdev.ram2[28][1] + pcmdev.tv_counter, 1);
                int s2 = eram_unpack(pcmdev.ram2[28][1] + pcmdev.tv_counter, 0);
                if ((v1 & 0x30) != 0)
                {
                    v2 = s1;
                }
                int v3 = addclip20(m1, v2 ^ 0xfffff, 1);
                pcmdev.ram1[29][4] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcmdev.ram1[29][5] = addclip20(m2 >> 1, s2, m2 & 1);
            }
            {
                // 2
                int v1 = pcmdev.ram2[30][4];
                int v2 = 0;
                int s1 = eram_unpack(pcmdev.ram2[28][2] + pcmdev.tv_counter, 1);
                int s2 = eram_unpack(pcmdev.ram2[28][2] + pcmdev.tv_counter, 0);
                if ((v1 & 0x30) != 0)
                {
                    v2 = s1;
                }
                int v3 = addclip20(pcmdev.ram1[29][5], v2 ^ 0xfffff, 1);
                pcmdev.ram1[29][5] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcmdev.ram1[28][0] = addclip20(m2 >> 1, s2, m2 & 1);
            }
            {
                // 3
                int v1 = pcmdev.ram2[30][4];
                int v2 = 0;
                int s1 = eram_unpack(pcmdev.ram2[28][3] + pcmdev.tv_counter, 1);
                int s2 = eram_unpack(pcmdev.ram2[28][3] + pcmdev.tv_counter, 0);
                if ((v1 & 0x30) != 0)
                {
                    v2 = s1;
                }
                int v3 = addclip20(pcmdev.ram1[28][0], v2 ^ 0xfffff, 1);
                pcmdev.ram1[28][0] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcmdev.ram1[28][1] = addclip20(m2 >> 1, s2, m2 & 1);


                pcmdev.ram1[28][2] = eram_unpack(pcmdev.ram2[28][5] + pcmdev.tv_counter, 0);
            }
            {
                // 4
                int v1 = pcmdev.ram2[30][5];
                int v2 = 0;
                int s1 = eram_unpack(pcmdev.ram2[28][4] + pcmdev.tv_counter, 1);
                int s2 = eram_unpack(pcmdev.ram2[28][4] + pcmdev.tv_counter, 0);
                if ((v1 & 0x30) != 0)
                {
                    v2 = s1;
                }
                int v3 = addclip20(pcmdev.ram1[28][1], v2 ^ 0xfffff, 1);
                pcmdev.ram1[28][1] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcmdev.ram1[28][3] = addclip20(m2 >> 1, s2, m2 & 1);


                pcmdev.ram1[28][4] = eram_unpack(pcmdev.ram2[29][1] + pcmdev.tv_counter, 0);
            }
            {
                // 5

                int v1 = pcmdev.ram2[30][7];
                int m1 = multi(pcmdev.ram1[29][2], (v1 >> 8)) >> 5;
                int s1 = eram_unpack(pcmdev.ram2[29][0] + pcmdev.tv_counter, 0);
                int m2 = multi(s1, v1 & 255) >> 5;
                pcmdev.ram1[29][2] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);

                eram_pack(pcmdev.ram2[28][0] + pcmdev.tv_counter, pcmdev.ram1[29][4]);
            }
            {
                // 6

                int v1 = pcmdev.ram2[30][8];
                int m1 = multi(pcmdev.ram1[29][3], (v1 >> 8)) >> 5;
                int s1 = eram_unpack(pcmdev.ram2[29][8] + pcmdev.tv_counter, 0);
                int m2 = multi(s1, v1 & 255) >> 5;
                pcmdev.ram1[29][3] = addclip20(m1 >> 1, m2 >> 1, (m1 | m2) & 1);

                eram_pack(pcmdev.ram2[28][1] + pcmdev.tv_counter, pcmdev.ram1[29][5]);

                eram_pack(pcmdev.ram2[28][2] + pcmdev.tv_counter, pcmdev.ram1[28][0]);
            }
            {
                // 7

                int v1 = pcmdev.ram2[30][9];
                int v2 = pcmdev.ram1[28][3];
                int m1 = multi(pcmdev.ram1[29][2], (v1 >> 8)) >> 5;
                int m2 = multi(pcmdev.ram1[29][3], (v1 >> 8)) >> 5;
                pcmdev.ram1[28][3] = addclip20(v2, m1 >> 1, m1 & 1);
                pcmdev.ram1[28][5] = addclip20(v2, m2 >> 1, m2 & 1);

                eram_pack(pcmdev.ram2[28][3] + pcmdev.tv_counter, pcmdev.ram1[28][1]);
            }
            {
                // 8

                int v1 = pcmdev.ram2[30][6];
                int m1 = multi(pcmdev.ram1[28][2], v1 >> 8) >> 5;

                int v2 = addclip20(pcmdev.ram1[28][3], m1 >> 1, m1 & 1);
                pcmdev.ram1[28][3] = v2;
                int m2 = multi(v2, v1 & 255) >> 5;
                pcmdev.ram1[28][2] = addclip20(pcmdev.ram1[28][2], m2 >> 1, m2 & 1);


                pcmdev.ram1[28][1] = eram_unpack(pcmdev.ram2[28][9] + pcmdev.tv_counter, 0);
            }
            {
                // 9

                int v1 = pcmdev.ram2[30][6];
                int m1 = multi(pcmdev.ram1[28][4], v1 >> 8) >> 5;

                int v2 = addclip20(pcmdev.ram1[28][5], m1 >> 1, m1 & 1);
                pcmdev.ram1[28][5] = v2;
                int m2 = multi(v2, v1 & 255) >> 5;
                pcmdev.ram1[28][4] = addclip20(pcmdev.ram1[28][4], m2 >> 1, m2 & 1);


                pcmdev.ram1[29][4] = eram_unpack(pcmdev.ram2[29][5] + pcmdev.tv_counter, 0);
            }
            {
                // 10

                int v1 = pcmdev.ram2[30][6];
                int v2 = pcmdev.ram1[28][1];
                int m1 = multi(v2, v1 >> 8) >> 5;
                int s1 = eram_unpack(pcmdev.ram2[28][8] + pcmdev.tv_counter, 0);
                int v3 = addclip20(m1 >> 1, s1, m1 & 1);
                pcmdev.ram1[28][1] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcmdev.ram1[29][5] = addclip20(m2 >> 1, v2, m2 & 1);

                eram_pack(pcmdev.ram2[28][4] + pcmdev.tv_counter, pcmdev.ram1[28][3]);
            }
            {
                // 11

                int v1 = pcmdev.ram2[30][6];
                int v2 = pcmdev.ram1[29][4];
                int m1 = multi(v2, v1 >> 8) >> 5;
                int s1 = eram_unpack(pcmdev.ram2[29][4] + pcmdev.tv_counter, 0);
                int v3 = addclip20(m1 >> 1, s1, m1 & 1);
                pcmdev.ram1[29][4] = v3;
                int m2 = multi(v3, v1 & 255) >> 5;
                pcmdev.ram1[28][0] = addclip20(m2 >> 1, v2, m2 & 1);


                eram_pack(pcmdev.ram2[28][5] + pcmdev.tv_counter, pcmdev.ram1[28][2]);

                eram_pack(pcmdev.ram2[29][0] + pcmdev.tv_counter, pcmdev.ram1[28][5]);
            }
            {
                // 12

                pcmdev.ram1[28][5] = eram_unpack(pcmdev.ram2[28][6] + pcmdev.tv_counter, 0);
            }

            {
                // 13

                int s1 = eram_unpack(pcmdev.ram2[28][10] + pcmdev.tv_counter, 0);
                pcmdev.ram1[28][5] = addclip20(pcmdev.ram1[28][5], s1, 0);

                pcmdev.ram1[28][2] = eram_unpack(pcmdev.ram2[29][2] + pcmdev.tv_counter, 0);
            }

            {
                // 14

                int s1 = eram_unpack(pcmdev.ram2[29][6] + pcmdev.tv_counter, 0);
                int t1 = addclip20(s1, pcmdev.ram1[28][2], 0); // 6

                pcmdev.ram1[28][5] = addclip20(t1, pcmdev.ram1[28][5], 0);

                pcmdev.ram1[28][2] = eram_unpack(pcmdev.ram2[28][7] + pcmdev.tv_counter, 0);
            }

            {
                // 15

                int s1 = eram_unpack(pcmdev.ram2[28][11] + pcmdev.tv_counter, 0);
                pcmdev.ram1[28][2] = addclip20(pcmdev.ram1[28][2], s1, 0);

                pcmdev.ram1[28][3] = eram_unpack(pcmdev.ram2[29][3] + pcmdev.tv_counter, 0);
            }

            {
                // 16

                int s1 = eram_unpack(pcmdev.ram2[29][7] + pcmdev.tv_counter, 0);
                int t1 = addclip20(s1, pcmdev.ram1[28][2], 0);
                pcmdev.ram1[28][2] = addclip20(t1, pcmdev.ram1[28][3], 0);


                eram_pack(pcmdev.ram2[29][1] + pcmdev.tv_counter, pcmdev.ram1[28][4]);

                eram_pack(pcmdev.ram2[28][8] + pcmdev.tv_counter, pcmdev.ram1[28][1]);
            }

            {
                // 17
                int v1 = pcmdev.ram2[30][2];
                int v2 = pcmdev.ram1[28][5];

                int m1 = multi(v2, v1 >> 8) >> 5;

                rcadd[0] = m1;

                rcadd2[0] = multi(v2, v1 & 255) >> 5;

                int t1 = eram_unpack(pcmdev.ram2[29][10] + pcmdev.tv_counter + 1, 0);
                eram_pack(pcmdev.ram2[28][9] + pcmdev.tv_counter, pcmdev.ram1[29][5]);
                pcmdev.ram1[29][5] = t1;
            }

            {
                // 18
                int v1 = pcmdev.ram2[30][3];
                int v2 = pcmdev.ram1[28][2];

                int m1 = multi(v2, v1 >> 8) >> 5;

                rcadd[1] = m1;

                rcadd2[1] = multi(v2, v1 & 255) >> 5;

                pcmdev.ram1[28][1] = eram_unpack(pcmdev.ram2[29][11] + pcmdev.tv_counter + 1, 0);
            }
            {
                // 19

                int v1 = pcmdev.ram2[31][9];

                int s1 = eram_unpack(pcmdev.ram2[29][10] + pcmdev.tv_counter, 0);

                eram_pack(pcmdev.ram2[29][4] + pcmdev.tv_counter, pcmdev.ram1[29][4]);

                int m1 = multi(s1, v1 >> 8) >> 5;
                int m2 = multi(pcmdev.ram1[29][5], v1 >> 8) >> 5;

                int t2 = addclip20(s1, (m1 >> 1) ^ 0xfffff, 1);

                pcmdev.ram1[29][5] = addclip20(t2, m2 >> 1, m2 & 1);
            }
            {
                // 20

                int v1 = pcmdev.ram2[31][10];

                int s1 = eram_unpack(pcmdev.ram2[29][11] + pcmdev.tv_counter, 0);

                eram_pack(pcmdev.ram2[29][5] + pcmdev.tv_counter, pcmdev.ram1[28][0]);

                int m1 = multi(s1, v1 >> 8) >> 5;
                int m2 = multi(pcmdev.ram1[28][1], v1 >> 8) >> 5;

                int t2 = addclip20(s1, (m1 >> 1) ^ 0xfffff, 1);

                pcmdev.ram1[28][1] = addclip20(t2, m2 >> 1, m2 & 1);

                eram_pack(pcmdev.ram2[29][9] + pcmdev.tv_counter, pcmdev.ram1[29][1]);
            }
            {
                // 21

                int v1 = pcmdev.ram2[31][2];
                int v2 = pcmdev.ram1[29][5];

                int m1 = multi(v2, v1 >> 8) >> 5;
                int m2 = multi(v2, v1 & 255) >> 5;

                rcadd[2] = m1;
                rcadd2[2] = m2;
            }
            {
                // 22

                int v1 = pcmdev.ram2[31][3];
                int v2 = pcmdev.ram1[29][5];

                int m1 = multi(v2, v1 >> 8) >> 5;
                int m2 = multi(v2, v1 & 255) >> 5;

                rcadd[3] = m1;
                rcadd2[3] = m2;
            }
            {
                // 23

                int v1 = pcmdev.ram2[31][4];
                int v2 = pcmdev.ram1[28][1];

                int m1 = multi(v2, v1 >> 8) >> 5;
                int m2 = multi(v2, v1 & 255) >> 5;

                rcadd[4] = m1;
                rcadd2[4] = m2;
            }
            {
                // 31

                int v1 = pcmdev.ram2[31][5];
                int v2 = pcmdev.ram1[28][1];

                int m1 = multi(v2, v1 >> 8) >> 5;
                int m2 = multi(v2, v1 & 255) >> 5;

                rcadd[5] = m1;
                rcadd2[5] = m2;

                {
                    // address generator

                    int key = 1;
                    int okey = (pcmdev.ram2[31][7] & 0x20) != 0;
                    int active = key && okey;
                    int kon = key && !okey;

                    int b15 = (pcmdev.ram2[31][8] & 0x8000) != 0; // 0
                    int b6 = (pcmdev.ram2[31][7] & 0x40) != 0; // 1
                    int b7 = (pcmdev.ram2[31][7] & 0x80) != 0; // 1
                    int old_nibble = (pcmdev.ram2[31][7] >> 12) & 15; // 1

                    int address = pcmdev.ram1[31][4]; // 0
                    int address_end = pcmdev.ram1[31][0]; // 1 or 2
                    int address_loop = pcmdev.ram1[31][2]; // 2 or 1

                    int sub_phase = (pcmdev.ram2[31][8] & 0x3fff); // 1
                    int interp_ratio = (sub_phase >> 7) & 127;
                    sub_phase += pcmdev.ram2[pcmdev.ram2[31][7] & 31][0]; // 5
                    int sub_phase_of = (sub_phase >> 14) & 7;
                    if (pcmdev.nfs)
                    {
                        pcmdev.ram2[31][8] &= ~0x3fff;
                        pcmdev.ram2[31][8] |= sub_phase & 0x3fff;
                    }


                    // address 0
                    int address_cnt = address;

                    int cmp1 = b15 ? address_loop : address_end;
                    int cmp2 = address_cnt;
                    int address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 9
                    int next_b15 = b15;

                    int next_address = address_cnt; // 11

                    cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
                    cmp2 = address_cnt;
                    int address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

                    int address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
                    int address_sub = !address_cmp && b6 && b15;
                    if (b7)
                        address_cnt2 -= address_add - address_sub;
                    else
                        address_cnt2 += address_add - address_sub;
                    address_cnt = address_cnt2 & 0xfffff; // 11
                    b15 = b6 && (b15 ^ address_cmp); // 11

                    cmp1 = b15 ? address_loop : address_end;
                    cmp2 = address_cnt;
                    address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 13

                    if (sub_phase_of >= 1)
                    {
                        next_address = address_cnt; // 13
                        next_b15 = b15;
                    }

                    if (active && pcmdev.nfs)
                        pcmdev.ram1[31][4] = next_address;

                    if (pcmdev.nfs)
                    {
                        pcmdev.ram2[31][8] &= ~0x8000;
                        pcmdev.ram2[31][8] |= next_b15 << 15;
                    }

                    int t1 = address_loop; // 18
                    int t2 = pcmdev.ram1[31][4] - t1; // 19
                    int t3 = address_end - t2; // 20
                    int t4 = pcmdev.ram1[31][4]; // 23

                    pcmdev.ram2[29][10] = t3;
                    pcmdev.ram2[29][11] = t4;
                }
            }
        }

        pcmdev.ram1[31][1] = 0;
        pcmdev.ram1[31][3] = 0;
        pcmdev.rcsum[0] = 0;
        pcmdev.rcsum[1] = 0;

        for (int slot = 0; slot < reg_slots; slot++)
        {
            uint32_t *ram1 = pcmdev.ram1[slot];
            uint16_t *ram2 = pcmdev.ram2[slot];
            int okey = (ram2[7] & 0x20) != 0;
            int key = (voice_active >> slot) & 1;

            int active = okey && key;
            int kon = key && !okey;

            // address generator

            int b15 = (ram2[8] & 0x8000) != 0; // 0
            int b6 = (ram2[7] & 0x40) != 0; // 1
            int b7 = (ram2[7] & 0x80) != 0; // 1
            int hiaddr = (ram2[7] >> 8) & 15; // 1
            int old_nibble = (ram2[7] >> 12) & 15; // 1

            int address = ram1[4]; // 0
            int address_end = ram1[0]; // 1 or 2
            int address_loop = ram1[2]; // 2 or 1

            int cmp1 = b15 ? address_loop : address_end;
            int cmp2 = address;
            int nibble_cmp1 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 2
            int irq_flag = 0;

            // fixme:
            if (kon)
                irq_flag = ((cmp1 + address_loop) & 0x100000) != 0;
            else
                irq_flag = ((address + ((-address_loop) & 0xfffff)) & 0x100000) != 0;
            irq_flag ^= b7;

            int nibble_address = (!b6 && nibble_cmp1) ? address_loop : address; // 3
            int address_b4 = (nibble_address & 0x10) != 0;
            int wave_address = nibble_address >> 5;
            int xor2 = (address_b4 ^ b7);
            int check1 = xor2 && active;
            int xor1 = (b15 ^ !nibble_cmp1);
            int nibble_add = b6 ? check1 && xor1 : (!nibble_cmp1 && check1);
            int nibble_subtract = b6 && !xor1 && active && !xor2;
            if (b7)
                wave_address -= nibble_add - nibble_subtract;
            else
                wave_address += nibble_add - nibble_subtract;
            wave_address &= 0xfffff;

            int newnibble = PCMDev_ReadROM((hiaddr << 20) | wave_address);
            int newnibble_sel = address_b4 ^ ((b6 || !nibble_cmp1) && okey);
            if (newnibble_sel)
                newnibble = (newnibble >> 4) & 15;
            else
                newnibble &= 15;

            int sub_phase = (ram2[8] & 0x3fff); // 1
            int interp_ratio = (sub_phase >> 7) & 127;
            sub_phase += pcmdev.ram2[ram2[7] & 31][0]; // 5
            int sub_phase_of = (sub_phase >> 14) & 7;
            if (pcmdev.nfs)
            {
                ram2[8] &= ~0x3fff;
                ram2[8] |= sub_phase & 0x3fff;
            }


            // address 0
            int address_cnt = address;
            int samp0 = (int8_t)PCMDev_ReadROM((hiaddr << 20) | address_cnt); // 18

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp2 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 8
            cmp1 = b15 ? address_loop : address_end;
            cmp2 = address_cnt;
            int address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 9

            int next_address = address_cnt; // 11
            int usenew = !nibble_cmp2;
            int next_b15 = b15;

            cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
            cmp2 = address_cnt;
            int address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

            int address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
            int address_sub = !address_cmp && b6 && b15;
            if (b7)
                address_cnt2 -= address_add - address_sub;
            else
                address_cnt2 += address_add - address_sub;
            address_cnt = address_cnt2 & 0xfffff; // 11
            b15 = b6 && (b15 ^ address_cmp); // 11

            int samp1 = (int8_t)PCMDev_ReadROM((hiaddr << 20) | address_cnt); // 20

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp3 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 12
            cmp1 = b15 ? address_loop : address_end;
            cmp2 = address_cnt;
            address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 13

            if (sub_phase_of >= 1)
            {
                next_address = address_cnt; // 13
                usenew = !nibble_cmp3;
                next_b15 = b15;
            }

            cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
            cmp2 = address_cnt;
            address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

            address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
            address_sub = !address_cmp && b6 && b15;
            if (b7)
                address_cnt2 -= address_add - address_sub;
            else
                address_cnt2 += address_add - address_sub;
            address_cnt = address_cnt2 & 0xfffff; // 15
            b15 = b6 && (b15 ^ address_cmp); // 15

            int samp2 = (int8_t)PCMDev_ReadROM((hiaddr << 20) | address_cnt); // 1

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp4 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 16
            cmp1 = b15 ? address_loop : address_end;
            cmp2 = address_cnt;
            address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 17

            if (sub_phase_of >= 2)
            {
                next_address = address_cnt; // 17
                usenew = !nibble_cmp4;
                next_b15 = b15;
            }

            cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
            cmp2 = address_cnt;
            address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

            address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
            address_sub = !address_cmp && b6 && b15;
            if (b7)
                address_cnt2 -= address_add - address_sub;
            else
                address_cnt2 += address_add - address_sub;
            address_cnt = address_cnt2 & 0xfffff; // 19
            b15 = b6 && (b15 ^ address_cmp); // 19

            int samp3 = (int8_t)PCMDev_ReadROM((hiaddr << 20) | address_cnt); // 5

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp5 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 20
            cmp1 = b15 ? address_loop : address_end;
            cmp2 = address_cnt;
            address_cmp = (cmp1 & 0xfffff) == (cmp2 & 0xfffff); // 21

            if (sub_phase_of >= 3)
            {
                next_address = address_cnt; // 21
                usenew = !nibble_cmp5;
                next_b15 = b15;
            }

            cmp1 = (!b6 && address_cmp) ? address_loop : address_cnt;
            cmp2 = address_cnt;
            address_cnt2 = (kon || (!b6 && address_cmp)) ? cmp1 : cmp2;

            address_add = (!address_cmp && b6 && !b15) || (!address_cmp && !b6);
            address_sub = !address_cmp && b6 && b15;
            if (b7)
                address_cnt2 -= address_add - address_sub;
            else
                address_cnt2 += address_add - address_sub;
            address_cnt = address_cnt2 & 0xfffff; // 23
            // b15 = b6 && (b15 ^ address_cmp); // 23

            cmp1 = address;
            cmp2 = address_cnt;
            int nibble_cmp6 = (cmp1 & 0xffff0) == (cmp2 & 0xffff0); // 24

            if (sub_phase_of >= 4)
            {
                next_address = address_cnt; // 1
                usenew = !nibble_cmp6;
                // b15 is not updated?
            }

            if (active && pcmdev.nfs)
                ram1[4] = next_address;

            if (pcmdev.nfs)
            {
                ram2[8] &= ~0x8000;
                ram2[8] |= next_b15 << 15;
            }

            // dpcm

            // 18
            int reference = ram1[5];

            // 19
            int preshift = samp0 << 10;
            int select_nibble = nibble_cmp2 ? old_nibble : newnibble;
            int shift = (10 - select_nibble) & 15;

            int shifted = (preshift << 1) >> shift;

            if (sub_phase_of >= 1)
                reference = addclip20(reference, shifted >> 1, shifted & 1);

            preshift = samp1 << 10;
            select_nibble = nibble_cmp3 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;

            shifted = (preshift << 1) >> shift;

            if (sub_phase_of >= 2)
                reference = addclip20(reference, shifted >> 1, shifted & 1);

            preshift = samp2 << 10;
            select_nibble = nibble_cmp4 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;

            shifted = (preshift << 1) >> shift;

            if (sub_phase_of >= 3)
                reference = addclip20(reference, shifted >> 1, shifted & 1);

            preshift = samp3 << 10;
            select_nibble = nibble_cmp5 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;

            shifted = (preshift << 1) >> shift;

            if (sub_phase_of >= 4)
                reference = addclip20(reference, shifted >> 1, shifted & 1);

            // interpolation

            int test = ram1[5];

            int step0 = multi(interp_lut[0][interp_ratio] << 6, samp0) >> 8;
            select_nibble = nibble_cmp2 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;
            step0 =  (step0 << 1) >> shift;

            test = addclip20(test, step0 >> 1, step0 & 1);


            int step1 = multi(interp_lut[1][interp_ratio] << 6, samp1) >> 8;
            select_nibble = nibble_cmp3 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;
            step1 = (step1 << 1) >> shift;

            test = addclip20(test, step1 >> 1, step1 & 1);

            int step2 = multi(interp_lut[2][interp_ratio] << 6, samp2) >> 8;
            select_nibble = nibble_cmp4 ? old_nibble : newnibble;
            shift = (10 - select_nibble) & 15;
            step2 = (step2 << 1) >> shift;

            int reg1 = ram1[1];
            int reg3 = ram1[3];
            int reg2_6 = (ram2[6] >> 8) & 127;

            test = addclip20(test, step2 >> 1, step2 & 1);

            int filter = ram2[11];
            int v3;

            if (pcm_float)
            {
                float A1 = (float)(int8_t)(filter >> 8);
                float A2 = (float)((filter >> 1) & 127);
                float Bc = (float)reg2_6;
                const float g1 = A1 / 64.0f + A2 / 8192.0f;
                const float g2 = Bc / 64.0f;

                int tests = test;
                tests <<= 12;
                tests >>= 12;
                float xf = (float)tests / 524288.0f;

                float f1 = pcmdev.fstate[slot][0];
                float f2 = pcmdev.fstate[slot][1];

                float state2_new = f2 + f1 * g1;
                state2_new = state2_new > 1.0f ? 1.0f : (state2_new < -1.0f ? -1.0f : state2_new);
                float subvar = state2_new + f1 * g2;
                subvar = subvar > 1.0f ? 1.0f : (subvar < -1.0f ? -1.0f : subvar);
                float out_v3 = xf - subvar;
                out_v3 = out_v3 > 1.0f ? 1.0f : (out_v3 < -1.0f ? -1.0f : out_v3);
                float state1_new = f1 + out_v3 * g1;
                state1_new = state1_new > 1.0f ? 1.0f : (state1_new < -1.0f ? -1.0f : state1_new);

                int c20;
                if (state2_new > 1.0f) state2_new = 1.0f; else if (state2_new < -1.0f) state2_new = -1.0f;
                c20 = (int)(state2_new * 524288.0f);
                ram1[3] = (uint32_t)c20;

                if (out_v3 > 1.0f) out_v3 = 1.0f; else if (out_v3 < -1.0f) out_v3 = -1.0f;
                v3 = (int)(out_v3 * 524288.0f);

                pcmdev.fstate[slot][0] = state1_new;
                pcmdev.fstate[slot][1] = state2_new;
            }
            else if (vm_mcu_mk1)
            {
                int mult1 = multi(reg1, filter >> 8); // 8
                int mult2 = multi(reg1, (filter >> 1) & 127); // 9
                int mult3 = multi(reg1, reg2_6); // 10

                int v2 = addclip20(reg3, mult1 >> 6, (mult1 >> 5) & 1); // 9
                int v1 = addclip20(v2, mult2 >> 13, (mult2 >> 12) & 1); // 10
                int subvar = addclip20(v1, (mult3 >> 6), (mult3 >> 5) & 1); // 11

                ram1[3] = v1;

                v3 = addclip20(test, subvar ^ 0xfffff, 1); // 12

                int mult4 = multi(v3, filter >> 8);
                int mult5 = multi(v3, (filter >> 1) & 127);
                int v4 = addclip20(reg1, mult4 >> 6, (mult4 >> 5) & 1); // 14
                int v5 = addclip20(v4, mult5 >> 13, (mult5 >> 12) & 1); // 15

                ram1[1] = v5;
            }
            else
            {
                // hack: use 32-bit math to avoid overflow
                int mult1 = reg1 * (int8_t)(filter >> 8); // 8
                int mult2 = reg1 * (int8_t)((filter >> 1) & 127); // 9
                int mult3 = reg1 * (int8_t)reg2_6; // 10

                int v2 = reg3 + (mult1 >> 6) + ((mult1 >> 5) & 1); // 9
                int v1 = v2 + (mult2 >> 13) + ((mult2 >> 12) & 1); // 10
                int subvar = v1 + (mult3 >> 6) + ((mult3 >> 5) & 1); // 11

                ram1[3] = v1;

                int tests = test;
                tests <<= 12;
                tests >>= 12;

                v3 = tests - subvar; // 12

                int mult4 = v3 * (int8_t)(filter >> 8);
                int mult5 = v3 * (int8_t)((filter >> 1) & 127);
                int v4 = reg1 + (mult4 >> 6) + ((mult4 >> 5) & 1); // 14
                int v5 = v4 + (mult5 >> 13) + ((mult5 >> 12) & 1); // 15

                ram1[1] = v5;
            }


            ram1[5] = reference;

            if (active && (ram2[6] & 1) != 0 && (ram2[8] & 0x4000) == 0 && !pcmdev.irq_assert && irq_flag)
            {
                if (pcmdev.nfs)
                    ram2[8] |= 0x4000;
                pcmdev.irq_assert = 1;
                pcmdev.irq_channel = slot;
                if (vm_mcu_jv880)
                    MCU_GA_SetGAInt(5, 1);
                else
                    MCU_Interrupt_SetRequest(INTERRUPT_SOURCE_IRQ0, 1);
            }

            int volmul1 = 0;
            int volmul2 = 0;

            calc_tv(0, ram2[3], &ram2[9], active, &volmul1);
            calc_tv(1, ram2[4], &ram2[10], active, &volmul2);
            calc_tv(2, ram2[5], &ram2[11], active, NULL);

            int sample = (ram2[6] & 2) == 0 ? ram1[3] : v3;

            int multiv1 = multi(sample, volmul1 >> 8);
            int multiv2 = multi(sample, (volmul1 >> 1) & 127);

            int sample2 = addclip20(multiv1 >> 6, multiv2 >> 13, ((multiv2 >> 12) | (multiv1 >> 5)) & 1);

            int multiv3 = multi(sample2, volmul2 >> 8);
            int multiv4 = multi(sample2, (volmul2 >> 1) & 127);

            int sample3 = addclip20(multiv3 >> 6, multiv4 >> 13, ((multiv4 >> 12) | (multiv3 >> 5)) & 1);

            int pan = active ? ram2[1] : 0;
            int rc = active ? ram2[2] : 0;

            int sampl = multi(sample3, (pan >> 8) & 255);
            int sampr = multi(sample3, (pan >> 0) & 255);

            int rc0 = multi(sample3, (rc >> 8) & 255) >> 5; // reverb
            int rc1 = multi(sample3, (rc >> 0) & 255) >> 5; // chorus

            // mix reverb/chorus?
            int slot2 = (slot == reg_slots - 1) ? 31 : slot + 1;
            switch (slot2)
            {
                // 17, 18 - reverb

                case 17:
                    pcmdev.ram1[31][1] = addclip20(pcmdev.ram1[31][1], rcadd[0] >> 1, rcadd[0] & 1);
                    break;
                case 18:
                    pcmdev.ram1[31][3] = addclip20(pcmdev.ram1[31][3], rcadd[1] >> 1, rcadd[1] & 1);
                    break;
                case 21:
                    pcmdev.ram1[31][1] = addclip20(pcmdev.ram1[31][1], rcadd[2] >> 1, rcadd[2] & 1);
                    break;
                case 22:
                    pcmdev.ram1[31][3] = addclip20(pcmdev.ram1[31][3], rcadd[3] >> 1, rcadd[3] & 1);
                    break;
                case 23:
                    pcmdev.ram1[31][1] = addclip20(pcmdev.ram1[31][1], rcadd[4] >> 1, rcadd[4] & 1);
                    break;
                case 31:
                    pcmdev.ram1[31][3] = addclip20(pcmdev.ram1[31][3], rcadd[5] >> 1, rcadd[5] & 1);
                    break;
            }

            int suml = addclip20(pcmdev.ram1[31][1], sampl >> 6, (sampl >> 5) & 1);
            int sumr = addclip20(pcmdev.ram1[31][3], sampr >> 6, (sampr >> 5) & 1);

            switch (slot2)
            {
                case 17:
                    pcmdev.rcsum[1] = addclip20(pcmdev.rcsum[1], rcadd2[0] >> 1, rcadd2[0] & 1);
                    break;
                case 18:
                    pcmdev.rcsum[1] = addclip20(pcmdev.rcsum[1], rcadd2[1] >> 1, rcadd2[1] & 1);
                    break;
                case 21:
                    pcmdev.rcsum[0] = addclip20(pcmdev.rcsum[0], rcadd2[2] >> 1, rcadd2[2] & 1);
                    break;
                case 22:
                    pcmdev.rcsum[1] = addclip20(pcmdev.rcsum[1], rcadd2[3] >> 1, rcadd2[3] & 1);
                    break;
                case 23:
                    pcmdev.rcsum[0] = addclip20(pcmdev.rcsum[0], rcadd2[4] >> 1, rcadd2[4] & 1);
                    break;
                case 31:
                    pcmdev.rcsum[1] = addclip20(pcmdev.rcsum[1], rcadd2[5] >> 1, rcadd2[5] & 1);
                    break;
            }

            pcmdev.rcsum[0] = addclip20(pcmdev.rcsum[0], rc0 >> 1, rc0 & 1);
            pcmdev.rcsum[1] = addclip20(pcmdev.rcsum[1], rc1 >> 1, rc1 & 1);

            if (slot != reg_slots - 1)
            {
                pcmdev.ram1[31][1] = suml;
                pcmdev.ram1[31][3] = sumr;
            }
            else
            {
                pcmdev.accum_l = suml;
                pcmdev.accum_r = sumr;
            }

            if (key && pcmdev.nfs)
            {
                ram2[7] &= ~0xf020;
                ram2[7] |= ((usenew || kon) ? newnibble : old_nibble) << 12;

                // update key
                ram2[7] |= key << 5;
            }

            if (!active)
            {
                if (pcmdev.nfs)
                {
                    ram1[1] = 0;
                    ram1[3] = 0;
                    ram1[5] = 0;
                    pcmdev.fstate[slot][0] = 0.0f;
                    pcmdev.fstate[slot][1] = 0.0f;
                }

                ram2[8] = 0;
                ram2[9] = 0;
                ram2[10] = 0;
            }
        }

        if (pcmdev.nfs)
        {
            pcmdev.ram2[31][7] |= 0x20;
        }

        pcmdev.nfs = 1;

        int cycles_step = (reg_slots + 1) * 25;

        pcmdev.cycles += vm_mcu_jv880 ? (cycles_step * 25) / 29 : cycles_step;
    }
}
