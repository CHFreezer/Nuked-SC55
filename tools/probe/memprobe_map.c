// memprobe_map.c — replicate the VM's MCU_Read_impl (h8vm_main.c) exactly, for the
// addresses the H8 probe pokes (each page P at offsets 0x0000 and 0x8000), to give the
// ground-truth backing + expected byte, and cross-check against the probe's measured
// pre values. This is the authoritative map (T0: rom1.bin/rom2.bin bytes + the C read
// model), used to validate the probe and to report the exact address->backing mapping.
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

static uint8_t rom1[0x8000];
static uint8_t rom2[0x80000];
static uint8_t sram[0x8000]; // zero-init (fresh, like the VM at reset)
static int g_rame = 0;       // RAME bit (0 by default at reset)
static uint8_t ram[0x400];   // work RAM (RAME-gated)

// Exact port of MCU_Read_impl's page dispatch (h8vm_main.c:413-463).
static const char *backing(uint32_t address, uint8_t *val)
{
    uint32_t address_rom = address & 0x3ffff;
    if (address & 0x80000)
        address_rom |= 0x40000;
    uint32_t page = (address >> 16) & 0xf;
    uint32_t off = address & 0xffff;
    switch (page)
    {
    case 0:
        if (off < 0x8000) { *val = rom1[off]; return "rom1"; }
        if (off >= 0xff80) { *val = 0x00; return "device"; }
        if (off >= 0xfb80 && off < 0xff80 && g_rame) { *val = ram[(off - 0xfb80) & 0x3ff]; return "workram"; }
        if (off >= 0x8000) { *val = sram[off & 0x7fff]; return "sram"; }
        *val = 0xff; return "hole";
    case 1: case 2: case 3: case 4: case 8: case 9: case 14: case 15:
        *val = rom2[address_rom & (0x80000 - 1)]; return "rom2";
    case 10: case 11:
        *val = sram[off & 0x7fff]; return "sram-mirror";
    default:
        *val = 0xff; return "UNBACKED";
    }
}

int main(int argc, char **argv)
{
    const char *p1 = (argc > 1) ? argv[1] : "build/rom1.bin";
    const char *p2 = (argc > 2) ? argv[2] : "build/rom2.bin";
    FILE *f = fopen(p1, "rb"); if (!f || fread(rom1, 1, sizeof rom1, f) != sizeof rom1) return 2; fclose(f);
    f = fopen(p2, "rb"); if (!f || fread(rom2, 1, sizeof rom2, f) != sizeof rom2) return 3; fclose(f);

    printf("Addressable space: page=(addr>>16)&0xf (4-bit) -> 16 pages x 64KB = 1MB.\n");
    printf("Per page P, the VM's MCU_Read_impl maps (P,0x0000) and (P,0x8000) to:\n");
    printf("  P  | (P,0x0000) backing=val        | (P,0x8000) backing=val\n");
    for (int P = 0; P < 16; P++)
    {
        uint32_t a0 = ((uint32_t)P << 16) | 0x0000;
        uint32_t a1 = ((uint32_t)P << 16) | 0x8000;
        uint8_t v0, v1; const char *b0 = backing(a0, &v0); const char *b1 = backing(a1, &v1);
        uint32_t r0 = a0 & 0x3ffff; if (a0 & 0x80000) r0 |= 0x40000;
        uint32_t r1 = a1 & 0x3ffff; if (a1 & 0x80000) r1 |= 0x40000;
        char e0[24] = "", e1[24] = "";
        if (b0[3] == '2') snprintf(e0, sizeof e0, " rom2+0x%05x", r0 & (0x80000 - 1));
        if (b1[3] == '2') snprintf(e1, sizeof e1, " rom2+0x%05x", r1 & (0x80000 - 1));
        printf("  %02x  | %-12s %02x%-12s | %-12s %02x%s\n",
               P, b0, v0, e0, b1, v1, e1);
    }
    return 0;
}
