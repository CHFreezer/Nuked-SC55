// smvec_check.c — replicate the VM's SM reset-vector math (h8vm_sm.c) on our
// generated idle ROM, to see exactly where the SM starts, WITHOUT running the VM
// (avoids the SM ERROR_TRAP stdout flood). Confirms SM_GetVectorAddress(9) and the
// first opcode the SM would fetch.
#include <stdio.h>
#include <stdint.h>

static uint8_t g_rom[0x1000];
static uint8_t sm_read(uint16_t a)
{
    a &= 0x1fff;
    if (a & 0x1000) return g_rom[a & 0xfff];
    return 0; // sm_ram/devices/etc -> 0 here (not backed in this check)
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "build/probe_smrom.bin";
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "open fail\n"); return 1; }
    size_t n = fread(g_rom, 1, sizeof g_rom, f);
    fclose(f);
    if (n < 0x1000) { fprintf(stderr, "short read %zu\n", n); return 1; }

    // SM_GetVectorAddress(SM_VECTOR_RESET=9)
    uint16_t vaddr_lo = 0x1fec + 9 * 2;   // 0x1ffe
    uint16_t vaddr_hi = 0x1fec + 9 * 2 + 1; // 0x1fff
    uint8_t lo = sm_read(vaddr_lo), hi = sm_read(vaddr_hi);
    uint16_t pc = (uint16_t)(lo | (hi << 8));
    printf("diag: vaddr_lo=0x%04x vaddr_hi=0x%04x  (0x1ffe&0xfff)=0x%03x\n",
           vaddr_lo, vaddr_hi, (0x1ffe & 0xfff));
    printf("diag: g_rom[0x7fe]=%02x g_rom[0x7ff]=%02x\n", g_rom[0x7fe], g_rom[0x7ff]);
    printf("diag: sm_read(0x1ffe)=%02x sm_read(0x1fff)=%02x\n", sm_read(0x1ffe), sm_read(0x1fff));
    printf("vector bytes lo=%02x hi=%02x\n", lo, hi);
    printf("SM reset PC = 0x%04x\n", pc);
    uint8_t first = sm_read(pc);
    printf("first opcode at 0x%04x = 0x%02x %s\n", pc, first,
           first == 0x42 ? "(STP -> sleeps, GOOD)" :
           first == 0xea ? "(NOP)" :
           first == 0x00 ? "(unbacked -> 0 -> ERROR_TRAP!)" : "(other)");
    // also show the whole vector table region for context
    printf("sm_rom[0x7f8..0x7ff] = ");
    for (int i = 0x7f8; i <= 0x7ff; i++) printf("%02x ", g_rom[i]);
    printf("\n");
    return 0;
}
