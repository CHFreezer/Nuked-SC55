#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Compare per-burst sram dump logs.
// Usage: ramdiff <vm_ram_log> <orig_ram_log>
int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: ramdiff <vm_ram_log> <orig_ram_log>\n"); return 1; }
    FILE *f1 = fopen(argv[1], "r");
    FILE *f2 = fopen(argv[2], "r");
    if (!f1 || !f2) return 2;
    char line1[200], line2[200];
    int cur = -1;
    long lndiff = 0, lincnt = 0;
    while (1) {
        int ea = 0, eb = 0;
        if (fgets(line1, 200, f1)) { line1[strcspn(line1, "\r\n")] = 0; ea = 1; }
        if (fgets(line2, 200, f2)) { line2[strcspn(line2, "\r\n")] = 0; eb = 1; }
        if (!ea && !eb) break;
        int ha = ea && line1[0] == '=';
        int hb = eb && line2[0] == '=';
        if (ha || hb) {
            if (lndiff) printf("burst%d: %ld diff lines\n", cur, lndiff);
            if (ha) { cur = atoi(line1 + 4); lndiff = 0; lincnt = 0; }
            continue;
        }
        if (ea && eb && strcmp(line1, line2) != 0) {
            lndiff++;
            if (lndiff <= 4) {
                long base = lincnt * 16;
                int bo = 0;
                for (int k = 0; k + 1 < (int)strlen(line1); k += 2)
                    if (line1[k] != line2[k] || line1[k+1] != line2[k+1]) { bo = k/2; break; }
                printf("  burst%d sram+%04x vm=%s\n  burst%d sram+%04x or=%s\n",
                       cur, (int)(base + bo), line1, cur, (int)(base + bo), line2);
            }
        }
        if (ea) lincnt++;
    }
    if (lndiff) printf("burst%d: %ld diff lines\n", cur, lndiff);
    return 0;
}
