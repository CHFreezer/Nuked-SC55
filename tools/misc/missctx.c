#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define MMAX 6000
static long miss[MMAX];
static bool hit[MMAX];
static int nm = 0;

int main(void) {
    FILE *fm = fopen("miss_list.txt", "r");
    FILE *fs = fopen("seq_orig.txt", "r");
    if (!fm || !fs) return 1;
    char buf[64];
    while (fgets(buf, 64, fm) && nm < MMAX) miss[nm++] = strtol(buf, NULL, 16);
    int sample = 40, found = 0;
    for (int i = 0; i < nm; i++) hit[i] = false;
    char line[64], prev[64] = {0}, prev2[64] = {0};
    long idx = 0;
    while (fgets(line, 64, fs)) {
        line[strcspn(line, "\r\n")] = 0;
        if (strlen(line) < 4) {
            idx++;
            continue;
        }
        long pc = strtol(line, NULL, 16);
        for (int i = 0; i < nm; i += sample) {
            if (!hit[i] && miss[i] == pc) {
                printf("seq#%ld first hit %08lx prev=%s prev2=%s\n", idx, pc, prev, prev2);
                hit[i] = true;
                found++;
                break;
            }
        }
        strcpy(prev2, prev);
        strcpy(prev, line);
        idx++;
        if (found >= sample) break;
    }
    printf("total seq lines seen: %ld\n", idx);
    return 0;
}
