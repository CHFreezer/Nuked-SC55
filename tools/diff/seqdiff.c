#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// First divergences between two deduped main-PC sequences, with resync search.
// Usage: seqdiff <orig_seq> <vm_seq>
int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: seqdiff <orig_seq> <vm_seq>\n"); return 1; }
    FILE *f1 = fopen(argv[1], "r");
    FILE *f2 = fopen(argv[2], "r");
    if (!f1 || !f2) return 2;
    char a[32], b[32];
    long i = 0, j = 0;
    int events = 0;
    while (fgets(a, 32, f1) && fgets(b, 32, f2) && events < 10) {
        a[strcspn(a, "\r\n")] = 0;
        b[strcspn(b, "\r\n")] = 0;
        i++; j++;
        if (strcmp(a, b) == 0) continue;
        // divergence: find next resync within 200 lines
        char *savea = strdup(a), *saveb = strdup(b);
        long sa = i, sb = j;
        int resync = 0;
        for (int k = 1; k < 200; k++) {
            if (!fgets(a, 32, f1) || !fgets(b, 32, f2)) break;
            a[strcspn(a, "\r\n")] = 0;
            b[strcspn(b, "\r\n")] = 0;
            i++; j++;
            if (strcmp(a, b) == 0) { resync = 1; break; }
        }
        printf("DIVERGE at orig#%ld vm#%ld: orig=%s vm=%s", sa, sb, savea, saveb);
        if (resync) printf(" (resync at +%ld lines)\n", i - sa);
        else printf(" (no resync in 200)\n");
        if (!resync) break;
        events++;
    }
    printf("done, events=%d at orig#%ld vm#%ld\n", events, i, j);
    return 0;
}
