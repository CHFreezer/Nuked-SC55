#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// Compare addr + pc columns of read logs to find first PC-path divergence.
// Usage: pcdiff <vm_read_log> <orig_read_log>
int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: pcdiff <vm_read_log> <orig_read_log>\n"); return 1; }
    FILE *f1 = fopen(argv[1], "r");
    FILE *f2 = fopen(argv[2], "r");
    if (!f1 || !f2) return 2;
    char l1[120], l2[120];
    long line = 0, first = -1;
    int shown = 0;
    while (1) {
        if (!fgets(l1, 120, f1)) break;
        if (!fgets(l2, 120, f2)) break;
        l1[strcspn(l1, "\r\n")] = 0;
        l2[strcspn(l2, "\r\n")] = 0;
        line++;
        // extract "addr=%08x" and "pc=%04x:%04x"
        char a1[16] = "", a2[16] = "", p1[16] = "", p2[16] = "";
        const char *s;
        if ((s = strstr(l1, "addr="))) { for (int i = 0; i < 8; i++) a1[i] = s[5 + i]; }
        if ((s = strstr(l2, "addr="))) { for (int i = 0; i < 8; i++) a2[i] = s[5 + i]; }
        if ((s = strstr(l1, "pc="))) { for (int i = 0; i < 9; i++) p1[i] = s[3 + i]; }
        if ((s = strstr(l2, "pc="))) { for (int i = 0; i < 9; i++) p2[i] = s[3 + i]; }
        if (strcmp(a1, a2) != 0 || strcmp(p1, p2) != 0) {
            if (first < 0) first = line;
            if (shown < 20) printf("PATH-DIVERGE line%ld\n  vm: %s\n  or: %s\n", line, l1, l2);
            shown++;
        }
    }
    printf("first path divergence at line: %ld\n", first);
    printf("total path divergences: %d\n", shown);
    return 0;
}
