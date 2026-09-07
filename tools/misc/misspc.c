#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(int argc, char **argv) {
    FILE *f1 = fopen(argv[1], "r");
    FILE *f2 = fopen(argv[2], "r");
    if (!f1 || !f2) return 1;
    char buf[64];
    long orig[10000], vm[10000];
    long no = 0, nv = 0;
    while (fgets(buf, 64, f1)) {
        if (strlen(buf) < 3) continue;
        orig[no++] = strtol(buf, NULL, 16);
    }
    while (fgets(buf, 64, f2)) {
        if (strlen(buf) < 3) continue;
        vm[nv++] = strtol(buf, NULL, 16);
    }
    for (long i = 0; i < no; i++) {
        int found = 0;
        for (long j = 0; j < nv; j++) if (vm[j] == orig[i]) { found = 1; break; }
        if (!found) printf("%lx\n", orig[i]);
    }
    return 0;
}
