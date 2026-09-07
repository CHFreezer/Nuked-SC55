#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static long miss[6000];
static int nm = 0;

static int is_miss(long pc) {
    for (int i = 0; i < nm; i++) if (miss[i] == pc) return 1;
    return 0;
}

int main(void) {
    FILE *fm = fopen("miss_list.txt", "r");
    FILE *ff = fopen("flow_main.txt", "r");
    if (!fm || !ff) return 1;
    char buf[128];
    while (fgets(buf, 128, fm) && nm < 6000) miss[nm++] = strtol(buf, NULL, 16);
    // sort miss for speed
    for (int i = 0; i < nm; i++)
        for (int j = i + 1; j < nm; j++)
            if (miss[j] < miss[i]) { long t = miss[i]; miss[i] = miss[j]; miss[j] = t; }
    int nm2 = 0;
    for (int i = 0; i < nm; i++) if (i == 0 || miss[i] != miss[i-1]) miss[nm2++] = miss[i];
    nm = nm2;

    char a[64];
    long entry[4000]; int ne = 0;
    while (fgets(a, 64, ff)) {
        a[strcspn(a, "\r\n")] = 0;
        if (strlen(a) < 9) continue;
        long pa, pb;
        if (sscanf(a, "%lx %lx", &pa, &pb) != 2) continue;
        if (is_miss(pb) && !is_miss(pa)) {
            if (ne < 4000) entry[ne++] = pa;
        }
    }
    // dedup + sort entries, print
    for (int i = 0; i < ne; i++)
        for (int j = i + 1; j < ne; j++)
            if (entry[j] < entry[i]) { long t = entry[i]; entry[i] = entry[j]; entry[j] = t; }
    int ne2 = 0;
    for (int i = 0; i < ne; i++) if (i == 0 || entry[i] != entry[i-1]) entry[ne2++] = entry[i];
    printf("unique entry points into missing code: %d\n", ne2);
    for (int i = 0; i < ne2; i++) printf("%08lx\n", entry[i]);
    return 0;
}
