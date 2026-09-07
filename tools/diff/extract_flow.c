// Extract consecutive main-PC transitions (src -> dst) from obs.log, deduped.
// Output lines: "XXXXXXXX YYYYYYYY" (18-bit addrs, hex)
// Usage: extract_flow.exe obs.log out.txt
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static int cmp_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

static int h2(const char *s)
{
    int v = 0;
    for (int i = 0; i < 2; i++)
    {
        char c = s[i];
        v = v * 16 + (c >= 'a' ? (c - 'a' + 10) : (c - '0'));
    }
    return v;
}

static int h4(const char *s)
{
    return h2(s) * 256 + h2(s + 2);
}

int main(int argc, char **argv)
{
    if (argc < 3) return 1;
    FILE *f = fopen(argv[1], "rb");
    if (!f) return 2;
    static uint64_t *pairs;
    size_t cap = (1 << 22), n = 0;
    if (!pairs) pairs = malloc(cap * sizeof(uint64_t));
    static char buf[1 << 16];
    size_t r = 0, carry = 0;
    int prev = -1;
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
                if (prev >= 0 && prev != key)
                {
                    if (n >= cap) { cap *= 2; pairs = realloc(pairs, cap * sizeof(uint64_t)); }
                    pairs[n++] = ((uint64_t)(unsigned)prev << 18) | (unsigned)key;
                }
                prev = key;
            }
            else prev = -1;
            i = j + 1;
        }
    }
    fclose(f);
    qsort(pairs, n, sizeof(uint64_t), cmp_u64);
    FILE *o = fopen(argv[2], "w");
    uint64_t last = 0;
    for (size_t i = 0; i < n; i++)
    {
        if (i > 0 && pairs[i] == last) continue;
        unsigned src = (unsigned)(pairs[i] >> 18), dst = (unsigned)(pairs[i] & 0x3ffff);
        fprintf(o, "%06x %06x\n", src, dst);
        last = pairs[i];
    }
    fclose(o);
    printf("%zu transitions (deduped: see output)\n", n);
    return 0;
}
