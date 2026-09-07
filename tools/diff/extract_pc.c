// Extract unique executed PCs from the original emulator's obs.log.
// Line format: "pcchg main pc=XX:YYYY ... | sm pc=ZZZZ ..."
// Usage: extract_pc.exe obs.log out_main.txt out_sm.txt
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAP (1 << 19) // up to 512K unique entries each

static int cmp_int(const void *a, const void *b)
{
    int x = *(const int *)a, y = *(const int *)b;
    return (x > y) - (x < y);
}

static int h2(const char *s) // 2 hex chars -> value
{
    int v = 0;
    for (int i = 0; i < 2; i++)
    {
        char c = s[i];
        v = v * 16 + (c >= 'a' ? (c - 'a' + 10) : (c - '0'));
    }
    return v;
}

static int h4(const char *s) // 4 hex chars -> value
{
    return h2(s) * 256 + h2(s + 2);
}

int main(int argc, char **argv)
{
    if (argc < 4) return 1;
    FILE *f = fopen(argv[1], "rb");
    if (!f) return 2;
    static unsigned char seen_main[0x10 << 16]; // page 0-15, off 16-bit
    static int mp[CAP];
    static int sp[CAP];
    static unsigned char seen_sm[0x10000];
    long mn = 0, sn = 0;
    static char buf[1 << 16];
    size_t r = 0;
    size_t carry = 0; // leftover bytes kept at buf[0..carry-1]
    for (;;)
    {
        if (carry) memmove(buf, buf + r - carry, carry);
        size_t want = sizeof(buf) - carry - 256;
        size_t got = fread(buf + carry, 1, want, f);
        if (got == 0 && carry == 0) break;
        r = got + carry;
        carry = 0;
        size_t last_nl = 0;
        for (size_t i = 0; i < r; i++) if (buf[i] == '\n') last_nl = i + 1;
        size_t complete = last_nl;
        if (got == want && r > complete) carry = r - complete;
        for (size_t i = 0; i < complete; )
        {
            size_t j = i;
            while (j < complete && buf[j] != '\n') j++;
            buf[j] = 0;
            char *s = buf + i;
            char *p = strstr(s, "main pc=");
            if (p)
            {
                int pg = h2(p + 8);
                int off = h4(p + 11);
                int key = (pg << 16) | off;
                if (pg < 0x10 && !seen_main[key]) { seen_main[key] = 1; if (mn < CAP) mp[mn++] = key; }
            }
            char *q = strstr(s, "sm pc=");
            if (q)
            {
                int key = h4(q + 6);
                if (!seen_sm[key]) { seen_sm[key] = 1; if (sn < CAP) sp[sn++] = key; }
            }
            i = j + 1;
        }
    }
    fclose(f);
    qsort(mp, mn, sizeof(int), cmp_int);
    qsort(sp, sn, sizeof(int), cmp_int);
    FILE *o1 = fopen(argv[2], "w");
    for (long i = 0; i < mn; i++) fprintf(o1, "%08x\n", mp[i]);
    fclose(o1);
    FILE *o2 = fopen(argv[3], "w");
    for (long i = 0; i < sn; i++) fprintf(o2, "%04x\n", sp[i]);
    fclose(o2);
    printf("main: %ld unique PCs, sm: %ld unique PCs\n", mn, sn);
    return 0;
}
