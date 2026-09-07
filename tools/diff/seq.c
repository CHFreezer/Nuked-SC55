// Emit ordered deduped main-PC sequence.
// mode "vm": reads vm_trace.txt lines "NNNN pc=XX:YYYY ..."
// mode "orig": reads obs.log lines "pcchg main pc=XX:YYYY ..."
// Usage: seq.exe mode infile outfile
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    if (argc < 4) return 1;
    int vm = strcmp(argv[1], "vm") == 0;
    FILE *f = fopen(argv[2], "rb");
    if (!f) return 2;
    FILE *o = fopen(argv[3], "w");
    static char line[512];
    long prev = -1, n = 0;
    while (fgets(line, sizeof line, f))
    {
        char *p;
        if (vm)
        {
            if (line[0] == '#') continue;
            p = strstr(line, " pc=");
            if (!p) continue;
            p += 4; // "XX:YYYY"
        }
        else
        {
            p = strstr(line, "main pc=");
            if (!p) continue;
            p += 8; // "XX:YYYY"
        }
        int pg = (int)strtol(p, 0, 16);
        int off = (int)strtol(p + 3, 0, 16);
        int a = (pg << 16) | off;
        if (a != prev)
        {
            prev = a;
            fprintf(o, "%08x\n", a);
            n++;
        }
    }
    fclose(f);
    fclose(o);
    printf("%ld unique seq entries\n", n);
    return 0;
}
