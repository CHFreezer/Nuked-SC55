// Full-line diff of two read logs (addr + value + pc).
// Usage: readdiff <vm_read_log> <orig_read_log>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: readdiff <vm_read_log> <orig_read_log>\n"); return 1; }
    FILE *f1 = fopen(argv[1], "r");
    FILE *f2 = fopen(argv[2], "r");
    if (!f1 || !f2) return 2;
    char l1[120], l2[120];
    int shown = 0;
    long line = 0;
    while (1) {
        if (!fgets(l1, 120, f1)) l1[0] = 0;
        if (!fgets(l2, 120, f2)) l2[0] = 0;
        if (!l1[0] && !l2[0]) break;
        l1[strcspn(l1, "\r\n")] = 0;
        l2[strcspn(l2, "\r\n")] = 0;
        line++;
        if (strcmp(l1, l2) != 0) {
            if (shown < 30) printf("DIVERGE line%ld\n  vm: %s\n  or: %s\n", line, l1, l2);
            shown++;
        }
    }
    printf("total divergences: %d\n", shown);
    return 0;
}
