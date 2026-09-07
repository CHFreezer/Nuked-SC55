#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(void) {
    FILE *f = fopen("..\\build\\obs.log", "r");
    if (!f) { f = fopen("obs.log", "r"); }
    if (!f) return 1;
    char line[512], prev[512] = {0};
    long n = 0;
    while (fgets(line, 512, f)) {
        line[strcspn(line, "\r\n")] = 0;
        char *p = strstr(line, "main pc=");
        if (!p) continue;
        int cp, pc;
        sscanf(p + 8, "%x:%x", &cp, &pc);
        char *pp = strstr(prev, "main pc=");
        if (pp) {
            int c2, p2;
            sscanf(pp + 8, "%x:%x", &c2, &p2);
            if (c2 == 0 && p2 == 0x7c3f)
                printf("burst%ld: 7c3f -> %04x:%04x\n", n, cp, pc);
        }
        strcpy(prev, line);
        n++;
    }
    return 0;
}
