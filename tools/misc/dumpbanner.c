#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static long load(const char *p, unsigned char *buf, long sz) {
    FILE *f = fopen(p, "rb");
    if (!f) return -1;
    long n = fread(buf, 1, sz, f);
    fclose(f);
    return n;
}

int main(void) {
    unsigned char s[0x8000];
    /* read the orig ram dump (burst0 block) */
    FILE *f = fopen("..\\build\\ram_orig.log", "r");
    if (!f) { printf("no ram_orig.log\n"); return 1; }
    char line[300];
    int burst = -1;
    long off = 0;
    while (fgets(line, 300, f)) {
        line[strcspn(line, "\r\n")] = 0;
        if (line[0] == '=') { burst = atoi(line + 4); if (burst == 0) off = 0; continue; }
        if (burst == 0) {
            for (int k = 0; k + 1 < (int)strlen(line); k += 2) {
                int hi = line[k], lo = line[k+1];
                int v = (hi >= 'a' ? hi - 'a' + 10 : hi - '0') * 16 + (lo >= 'a' ? lo - 'a' + 10 : lo - '0');
                s[off++] = (unsigned char)v;
            }
        }
    }
    fclose(f);
    printf("orig sram 0x5c00-0x5d40:\n");
    for (int a = 0x5c00; a < 0x5d40; a += 16) {
        printf("%04x: ", a);
        for (int k = 0; k < 16; k++) printf("%02x ", s[a + k]);
        printf(" |");
        for (int k = 0; k < 16; k++) { char c = s[a+k]; if (c < 32 || c > 126) c = '.'; putchar(c); }
        printf("|\n");
    }

    unsigned char r2[512 * 1024];
    load("..\\roms\\SC-55mk2-v1.01\\rom2.bin", r2, 512 * 1024);
    /* print rom2 around 0x03ca00 and 0x0062b0 for reference */
    printf("\nrom2 0x03ca00-0x03ca40:\n");
    for (int a = 0x03ca00; a < 0x03ca40; a += 16) {
        printf("%06x: ", a);
        for (int k = 0; k < 16; k++) printf("%02x ", r2[a + k]);
        printf(" |");
        for (int k = 0; k < 16; k++) { char c = r2[a+k]; if (c < 32 || c > 126) c = '.'; putchar(c); }
        printf("|\n");
    }
    return 0;
}
