/*
 * h8reach.c -- static reachability closure for the mk2cpp M2 pipeline.
 *
 * Usage:
 *   h8reach.exe <rom1.bin> <rom2.bin> <pc_main.txt> <pc_vec.txt> <r16_reach.txt> <outdir>
 *
 * Computes the set of flat addresses statically reachable from the roots
 * (executed set E, unique vector entries V, and the r16 static-scan entries
 * S) by following only the h8dec static-successor relation (doc 06 §1.5):
 *
 *   kind 0 none    : fall-through (flat + len)
 *   kind 1 call    : static target + fall-through (return continuation)
 *   kind 2 uncond  : static target
 *   kind 3 cond    : static target + fall-through
 *   kind 4 ret     : none (stack dynamic)
 *   kind 5 reg-ind : none (register dynamic)
 *   trapa #n       : vector table entry 0x10+n (rom1[0..255], 4-byte BE,
 *                    addr &= ~3; GT MCU_GetVectorAddress / MCU_Interrupt_Handle)
 *   sleep          : fall-through (wake continuation)
 *
 * A successor that does not decode, falls in a known data range, or leaves
 * the ROM pages (cp0 < 0x8000, cp4) is a stop: recorded, never emitted.
 * Roots that fail to decode abort the run (parity gate: E is decode-verified
 * by h8part, V/S are pre-validated by r16align; any failure means the
 * fixtures changed).
 *
 * Jump-table expansion (doc 06 §1.7 level 2): for every E instruction of the
 * form `MOVG2 @rN+disp16 rN` immediately followed by `jmp rN` (same N), the
 * 16-bit BE table at rom offset disp16 is read up to its 0x0000 terminator
 * and each non-zero entry is added as a candidate target (same page as the
 * site). Candidates that do not decode or fall in a data range are skipped
 * and logged.
 *
 * Known data ranges (doc 06 §1.6/§1.7, manually confirmed):
 *   rom1 0x7086-0x714a  curve table (interpolation, read via @r2, no jmp)
 *   rom1 0x7238-0x7248  jump table of the 0x396c site (data, not code)
 * An executed PC inside a data range is a fixture/range error and aborts.
 *
 * Outputs (deterministic: flat-ascending, LF, lowercase hex, no timestamps):
 *   pc_reach.txt    E ∪ V ∪ S ∪ closure ∪ table candidates (h8part input)
 *   reach_new.txt   output set minus E (the M2 addition)
 *   reach_vec.txt   unique vector entries (V)
 *   reach_stats.txt counters + input sha256
 *   reach_stops.txt every (flat, reason) stop, sorted
 *   reach_tables.txt jump-table sites and their entries (audit)
 *
 * Build (same chain as h8part):
 *   clang -O2 -D_CRT_SECURE_NO_WARNINGS -Dmain=h8dec_selftest_main -c h8dec.c -o h8dec_reach.o
 *   clang -O2 -D_CRT_SECURE_NO_WARNINGS -o h8reach.exe h8reach.c h8dec_reach.o
 */

#define _CRT_SECURE_NO_WARNINGS 1

#include "h8dec.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#define H8REACH_MKDIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define H8REACH_MKDIR(p) mkdir((p), 0755)
#endif

#define ADDR_SPACE (1u << 20)

static void die(const char *fmt, ...);

enum {
    STOP_INVALID = 0, /* successor does not decode */
    STOP_DATA    = 1, /* successor inside a data range */
    STOP_PAGE    = 2, /* successor outside cp0(<0x8000)/cp4 */
    STOP_RET     = 3, /* ret: stack dynamic */
    STOP_REGIND  = 4  /* register-indirect: register dynamic */
};
#define STOP_N 5

/* ------------------------------------------------------------------ */
/* SHA-256 (same compact implementation as h8emit.c)                   */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint8_t data[64];
    uint32_t datalen;
} sha256_ctx;

#define SHA_ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

static const uint32_t sha256_k[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu,
    0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u,
    0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u,
    0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
    0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u,
    0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
    0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
    0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u,
    0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u, 0x1e376c08u,
    0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu,
    0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
};

static void sha256_transform(sha256_ctx *ctx, const uint8_t *blk)
{
    uint32_t m[64];
    uint32_t a, b, c, d, e, f, g, h;
    uint32_t t1, t2;
    int i;
    for (i = 0; i < 16; i++)
        m[i] = ((uint32_t)blk[i * 4] << 24) | ((uint32_t)blk[i * 4 + 1] << 16) |
               ((uint32_t)blk[i * 4 + 2] << 8) | (uint32_t)blk[i * 4 + 3];
    for (i = 16; i < 64; i++) {
        uint32_t x = m[i - 15];
        uint32_t y = m[i - 2];
        uint32_t s0 = SHA_ROTR(x, 7) ^ SHA_ROTR(x, 18) ^ (x >> 3);
        uint32_t s1 = SHA_ROTR(y, 17) ^ SHA_ROTR(y, 19) ^ (y >> 10);
        m[i] = m[i - 16] + s0 + m[i - 7] + s1;
    }
    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];
    for (i = 0; i < 64; i++) {
        t1 = h + (SHA_ROTR(e, 6) ^ SHA_ROTR(e, 11) ^ SHA_ROTR(e, 25)) +
             ((e & f) ^ (~e & g)) + sha256_k[i] + m[i];
        t2 = (SHA_ROTR(a, 2) ^ SHA_ROTR(a, 13) ^ SHA_ROTR(a, 22)) +
             ((a & b) ^ (a & c) ^ (b & c));
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }
    ctx->state[0] += a; ctx->state[1] += b;
    ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f;
    ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_init(sha256_ctx *ctx)
{
    ctx->datalen = 0;
    ctx->bitlen = 0;
    ctx->state[0] = 0x6a09e667u; ctx->state[1] = 0xbb67ae85u;
    ctx->state[2] = 0x3c6ef372u; ctx->state[3] = 0xa54ff53au;
    ctx->state[4] = 0x510e527fu; ctx->state[5] = 0x9b05688cu;
    ctx->state[6] = 0x1f83d9abu; ctx->state[7] = 0x5be0cd19u;
}

static void sha256_update(sha256_ctx *ctx, const uint8_t *data, size_t len)
{
    size_t i;
    for (i = 0; i < len; i++) {
        ctx->data[ctx->datalen++] = data[i];
        if (ctx->datalen == 64) {
            sha256_transform(ctx, ctx->data);
            ctx->bitlen += 512;
            ctx->datalen = 0;
        }
    }
}

static void sha256_final(sha256_ctx *ctx, uint8_t out[32])
{
    uint32_t i = ctx->datalen;
    if (ctx->datalen < 56) {
        ctx->data[i++] = 0x80;
        while (i < 56)
            ctx->data[i++] = 0x00;
    } else {
        ctx->data[i++] = 0x80;
        while (i < 64)
            ctx->data[i++] = 0x00;
        sha256_transform(ctx, ctx->data);
        memset(ctx->data, 0, 56);
    }
    ctx->bitlen += (uint64_t)ctx->datalen * 8;
    ctx->data[63] = (uint8_t)(ctx->bitlen);
    ctx->data[62] = (uint8_t)(ctx->bitlen >> 8);
    ctx->data[61] = (uint8_t)(ctx->bitlen >> 16);
    ctx->data[60] = (uint8_t)(ctx->bitlen >> 24);
    ctx->data[59] = (uint8_t)(ctx->bitlen >> 32);
    ctx->data[58] = (uint8_t)(ctx->bitlen >> 40);
    ctx->data[57] = (uint8_t)(ctx->bitlen >> 48);
    ctx->data[56] = (uint8_t)(ctx->bitlen >> 56);
    sha256_transform(ctx, ctx->data);
    for (i = 0; i < 4; i++) {
        out[i]      = (uint8_t)(ctx->state[0] >> (24 - i * 8));
        out[i + 4]  = (uint8_t)(ctx->state[1] >> (24 - i * 8));
        out[i + 8]  = (uint8_t)(ctx->state[2] >> (24 - i * 8));
        out[i + 12] = (uint8_t)(ctx->state[3] >> (24 - i * 8));
        out[i + 16] = (uint8_t)(ctx->state[4] >> (24 - i * 8));
        out[i + 20] = (uint8_t)(ctx->state[5] >> (24 - i * 8));
        out[i + 24] = (uint8_t)(ctx->state[6] >> (24 - i * 8));
        out[i + 28] = (uint8_t)(ctx->state[7] >> (24 - i * 8));
    }
}

static void sha256_hex(const uint8_t *data, size_t len, char out[65])
{
    static const char hexd[] = "0123456789abcdef";
    uint8_t digest[32];
    sha256_ctx ctx;
    int i;
    sha256_init(&ctx);
    if (len)
        sha256_update(&ctx, data, len);
    sha256_final(&ctx, digest);
    for (i = 0; i < 32; i++) {
        out[i * 2] = hexd[digest[i] >> 4];
        out[i * 2 + 1] = hexd[digest[i] & 0xf];
    }
    out[64] = 0;
}

/* Known-answer self-test: the digest goes into lock metadata, so a broken
 * implementation must abort, not produce stable-but-wrong values. */
static void sha256_selftest(void)
{
    static const char *const EMPTY_EXPECT =
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
    static const char *const ABC_EXPECT =
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
    char h[65];
    sha256_hex(NULL, 0, h);
    if (strcmp(h, EMPTY_EXPECT) != 0)
        die("sha256 self-test failed (empty): got %s", h);
    sha256_hex((const uint8_t *)"abc", 3, h);
    if (strcmp(h, ABC_EXPECT) != 0)
        die("sha256 self-test failed (abc): got %s", h);
}

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
/* ------------------------------------------------------------------ */

static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fputs("h8reach: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    exit(2);
}

static void *xrealloc(void *p, size_t n)
{
    void *q = realloc(p, n ? n : 1);
    if (!q)
        die("out of memory");
    return q;
}

static void *xcalloc(size_t n, size_t sz)
{
    void *q = calloc(n ? n : 1, sz ? sz : 1);
    if (!q)
        die("out of memory");
    return q;
}

static uint8_t *load_file(const char *path, uint32_t *size)
{
    FILE *f = fopen(path, "rb");
    long n;
    uint8_t *buf;
    if (!f)
        die("cannot open '%s'", path);
    if (fseek(f, 0, SEEK_END) != 0)
        die("cannot seek '%s'", path);
    n = ftell(f);
    if (n < 0)
        die("cannot size '%s'", path);
    rewind(f);
    buf = (uint8_t *)xrealloc(NULL, (size_t)n ? (size_t)n : 1);
    if (fread(buf, 1, (size_t)n, f) != (size_t)n)
        die("cannot read '%s'", path);
    fclose(f);
    *size = (uint32_t)n;
    return buf;
}

static int parse_hex(const char *s, uint32_t *out)
{
    uint64_t v = 0;
    const char *p = s;
    int digits = 0;
    while (*p == ' ' || *p == '\t')
        p++;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X'))
        p += 2;
    while (*p) {
        int d;
        char c = *p;
        if (c >= '0' && c <= '9')
            d = c - '0';
        else if (c >= 'a' && c <= 'f')
            d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F')
            d = c - 'A' + 10;
        else
            break;
        v = (v << 4) | (uint64_t)d;
        if (v > 0xFFFFFFFFu)
            return 0;
        digits++;
        p++;
    }
    if (!digits)
        return 0;
    *out = (uint32_t)v;
    return 1;
}

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t')
        p++;
    return p;
}

/* flat -> rom byte with the same bank rules as h8dec/h8part */
static uint8_t *g_rom1;
static uint8_t *g_rom2;
static uint32_t g_rom1_size;
static uint32_t g_rom2_size;

static uint8_t rb(uint32_t flat)
{
    uint32_t page = (flat >> 16) & 0xf;
    uint32_t off = flat & 0xffff;
    uint32_t idx;
    if (page == 0 && off < g_rom1_size)
        return g_rom1[off];
    if (!g_rom2_size)
        return 0xff;
    idx = flat & 0x3ffff;
    if (flat & 0x80000)
        idx |= 0x40000;
    return g_rom2[idx % g_rom2_size];
}

/* ROM-page gate: only cp0 (< 0x8000, rom1) and cp4 (rom2) are code pages */
static int is_code_page(uint32_t flat)
{
    uint32_t page = (flat >> 16) & 0xf;
    uint32_t off = flat & 0xffff;
    if (page == 0)
        return off < 0x8000;
    if (page == 4)
        return 1;
    return 0;
}

/* known data ranges (doc 06 §1.6/§1.7): rom1 offsets, [start, end) */
static const uint32_t DATA_RANGES[][2] = {
    { 0x00007086u, 0x0000714a }, /* curve table */
    { 0x00007238u, 0x00007248 }  /* jump table of the 0x396c site */
};
#define DATA_RANGE_N (sizeof(DATA_RANGES) / sizeof(DATA_RANGES[0]))

static int in_data_range(uint32_t flat)
{
    uint32_t off;
    size_t i;
    if ((flat >> 16) & 0xf)
        return 0;
    off = flat & 0xffff;
    for (i = 0; i < DATA_RANGE_N; i++)
        if (off >= DATA_RANGES[i][0] && off < DATA_RANGES[i][1])
            return 1;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Flat set (bitmap over ADDR_SPACE, sorted iteration)                 */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t *m;
    uint32_t n;
} flatset_t;

static void set_add(flatset_t *s, uint32_t flat)
{
    if (flat < ADDR_SPACE && !s->m[flat]) {
        s->m[flat] = 1;
        s->n++;
    }
}

static int set_has(const flatset_t *s, uint32_t flat)
{
    return flat < ADDR_SPACE && s->m[flat] != 0;
}

/* ------------------------------------------------------------------ */
/* PC list files (first whitespace-separated hex token per line)       */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t *a;
    size_t n, cap;
} u32vec;

static void vec_push(u32vec *v, uint32_t x)
{
    if (v->n == v->cap) {
        v->cap = v->cap ? v->cap * 2 : 1024;
        v->a = (uint32_t *)xrealloc(v->a, v->cap * sizeof(uint32_t));
    }
    v->a[v->n++] = x;
}

static long read_pc_file(const char *path, u32vec *out)
{
    FILE *f = fopen(path, "rb");
    char line[512];
    long lines = 0;
    if (!f)
        die("cannot open pc file '%s'", path);
    while (fgets(line, sizeof line, f)) {
        const char *p = skip_ws(line);
        uint32_t v;
        if (!*p || *p == '\r' || *p == '\n' || *p == '#' ||
            (p[0] == '/' && p[1] == '/'))
            continue;
        if (parse_hex(p, &v)) {
            vec_push(out, v);
            lines++;
        }
    }
    fclose(f);
    return lines;
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    const char *rom1_path, *rom2_path, *pc_path, *vec_path, *s_path, *outdir;
    uint32_t rom1_size = 0, rom2_size = 0;
    uint8_t *rom1, *rom2;
    u32vec ev, vv, sv;
    long e_lines, v_lines, s_lines;
    flatset_t e, v, s, all, newset;
    uint8_t *stopmap[STOP_N];
    long stop_count[STOP_N] = {0};
    long n_new_vec = 0, n_new_static = 0;
    long table_sites = 0, table_entries = 0, table_added = 0, table_skipped = 0;
    long roots = 0;
    char sha1[65], sha2[65], shapc[65], shavec[65], shas[65];
    char path[1024];
    FILE *f, *ft;
    size_t i;
    uint32_t x;

    sha256_selftest();
    if (argc != 7)
        die("usage: h8reach.exe <rom1.bin> <rom2.bin> <pc_main.txt> "
            "<pc_vec.txt> <r16_reach.txt> <outdir>");
    rom1_path = argv[1];
    rom2_path = argv[2];
    pc_path = argv[3];
    vec_path = argv[4];
    s_path = argv[5];
    outdir = argv[6];

    memset(&ev, 0, sizeof ev);
    memset(&vv, 0, sizeof vv);
    memset(&sv, 0, sizeof sv);
    memset(&e, 0, sizeof e);
    memset(&v, 0, sizeof v);
    memset(&s, 0, sizeof s);
    memset(&all, 0, sizeof all);
    memset(&newset, 0, sizeof newset);

    rom1 = load_file(rom1_path, &rom1_size);
    rom2 = load_file(rom2_path, &rom2_size);
    g_rom1 = rom1;
    g_rom1_size = rom1_size;
    g_rom2 = rom2;
    g_rom2_size = rom2_size;
    h8dec_set_rom(rom1, rom1_size, rom2, rom2_size);
    fprintf(stderr, "h8reach: rom1=%s (%u) rom2=%s (%u)\n",
            rom1_path, rom1_size, rom2_path, rom2_size);

    e_lines = read_pc_file(pc_path, &ev);
    v_lines = read_pc_file(vec_path, &vv);
    s_lines = read_pc_file(s_path, &sv);
    fprintf(stderr, "h8reach: exec=%ld (unique %zu) vec=%ld (unique %zu) "
            "static-scan=%ld (unique %zu)\n",
            e_lines, ev.n, v_lines, vv.n, s_lines, sv.n);

    e.m = (uint8_t *)xcalloc(ADDR_SPACE, 1);
    v.m = (uint8_t *)xcalloc(ADDR_SPACE, 1);
    s.m = (uint8_t *)xcalloc(ADDR_SPACE, 1);
    all.m = (uint8_t *)xcalloc(ADDR_SPACE, 1);
    newset.m = (uint8_t *)xcalloc(ADDR_SPACE, 1);
    for (i = 0; i < STOP_N; i++)
        stopmap[i] = (uint8_t *)xcalloc(ADDR_SPACE, 1);

    /* roots: E (must all decode), V (unique code-page entries), S (r16) */
    for (i = 0; i < ev.n; i++) {
        uint32_t flat = ev.a[i];
        h8dec_t d;
        if (!is_code_page(flat))
            die("executed PC %08x outside code pages", flat);
        if (in_data_range(flat))
            die("executed PC %08x inside a data range (range wrong?)", flat);
        d = h8dec(flat);
        if (!d.valid || d.len <= 0)
            die("executed PC %08x does not decode", flat);
        set_add(&e, flat);
    }
    for (i = 0; i < vv.n; i++) {
        uint32_t flat = vv.a[i];
        h8dec_t d;
        if (!is_code_page(flat))
            continue; /* 0x00ffffff sentinels / foreign pages */
        d = h8dec(flat);
        if (!d.valid || d.len <= 0)
            die("vector entry %08x does not decode", flat);
        if (in_data_range(flat))
            die("vector entry %08x inside a data range (range wrong?)", flat);
        set_add(&v, flat);
    }
    for (i = 0; i < sv.n; i++) {
        uint32_t flat = sv.a[i];
        h8dec_t d;
        if (!is_code_page(flat))
            die("static-scan PC %08x outside code pages", flat);
        d = h8dec(flat);
        if (!d.valid || d.len <= 0)
            die("static-scan PC %08x does not decode", flat);
        if (in_data_range(flat))
            die("static-scan PC %08x inside a data range (range wrong?)", flat);
        set_add(&s, flat);
    }
    for (x = 0; x < ADDR_SPACE; x++)
        if (e.m[x] || v.m[x] || s.m[x])
            roots++;

    /* worklist over roots, following static successors only */
    {
        uint8_t *visited = (uint8_t *)xcalloc(ADDR_SPACE, 1);
        uint32_t *queue;
        uint32_t qhead = 0, qtail = 0, qcap = 1024;
        queue = (uint32_t *)xcalloc(qcap, sizeof(uint32_t));
        for (x = 0; x < ADDR_SPACE; x++) {
            if (!e.m[x] && !v.m[x] && !s.m[x])
                continue;
            visited[x] = 1;
            set_add(&all, x);
            if (qtail == qcap) {
                qcap *= 2;
                queue = (uint32_t *)xrealloc(queue, qcap * sizeof(uint32_t));
            }
            queue[qtail++] = x;
        }
        while (qhead < qtail) {
            uint32_t flat = queue[qhead++];
            h8dec_t d = h8dec(flat);
            uint8_t op0 = rb(flat);
            uint8_t op1 = rb(flat + 1);
            uint32_t succ[3];
            int ns = 0;

            if (!d.valid || d.len <= 0)
                die("closure PC %08x does not decode (internal error)", flat);

            if (op0 == 0x08 && (op1 & 0xf0) == 0x10) {
                /* trapa #n -> vector (0x10+n), 4-byte BE table in rom1 */
                uint32_t nvec = 0x10u + (op1 & 0x0f);
                uint32_t addr = ((uint32_t)rom1[nvec * 4] << 24) |
                                ((uint32_t)rom1[nvec * 4 + 1] << 16) |
                                ((uint32_t)rom1[nvec * 4 + 2] << 8) |
                                (uint32_t)rom1[nvec * 4 + 3];
                addr &= ~3u; /* MCU_Read32 alignment (mcu.cpp:885) */
                succ[ns++] = addr;
            } else if (op0 == 0x1a) { /* sleep: wake continuation */
                succ[ns++] = flat + (uint32_t)d.len;
            } else {
                switch (d.kind) {
                case H8K_NONE:
                    succ[ns++] = flat + (uint32_t)d.len;
                    break;
                case H8K_CALL:
                    succ[ns++] = d.target;
                    succ[ns++] = flat + (uint32_t)d.len;
                    break;
                case H8K_UNCOND:
                    succ[ns++] = d.target;
                    break;
                case H8K_COND:
                    succ[ns++] = d.target;
                    succ[ns++] = flat + (uint32_t)d.len;
                    break;
                case H8K_RET:
                    stopmap[STOP_RET][flat] = 1;
                    stop_count[STOP_RET]++;
                    break;
                case H8K_REGIND:
                    stopmap[STOP_REGIND][flat] = 1;
                    stop_count[STOP_REGIND]++;
                    break;
                default:
                    die("closure PC %08x: unknown kind %d", flat, d.kind);
                }
            }
            while (ns > 0) {
                uint32_t t = succ[--ns];
                if (t >= ADDR_SPACE || !is_code_page(t)) {
                    if (t < ADDR_SPACE) {
                        stopmap[STOP_PAGE][t] = 1;
                        stop_count[STOP_PAGE]++;
                    }
                    continue;
                }
                if (in_data_range(t)) {
                    stopmap[STOP_DATA][t] = 1;
                    stop_count[STOP_DATA]++;
                    continue;
                }
                if (visited[t])
                    continue;
                if (!h8dec(t).valid) {
                    stopmap[STOP_INVALID][t] = 1;
                    stop_count[STOP_INVALID]++;
                    continue;
                }
                visited[t] = 1;
                set_add(&all, t);
                if (qtail == qcap) {
                    qcap *= 2;
                    queue = (uint32_t *)xrealloc(queue, qcap * sizeof(uint32_t));
                }
                queue[qtail++] = t;
            }
        }
        free(queue);
        free(visited);
    }

    /* outdir (created before any output, incl. reach_tables.txt) */
    {
        size_t outdir_len = strlen(outdir);
        char tmp[1024];
        char *p;
        if (outdir_len == 0 || outdir_len >= sizeof tmp)
            die("bad outdir '%s'", outdir);
        memcpy(tmp, outdir, outdir_len + 1);
        for (p = tmp + 1; *p; p++) {
            if (*p == '/' || *p == '\\') {
                char c = *p;
                *p = '\0';
                if (tmp[0])
                    (void)H8REACH_MKDIR(tmp);
                *p = c;
            }
        }
        (void)H8REACH_MKDIR(tmp);
    }

    /* jump-table expansion (doc 06 §1.7 level 2): MOVG2 @rN+disp16 rN; jmp rN */
    ft = NULL;
    for (i = 0; i < ev.n; i++) {
        uint32_t flat = ev.a[i];
        h8dec_t d = h8dec(flat);
        uint32_t nxt, tpage, toff, base, k;
        uint8_t b2;
        if (flat >= ADDR_SPACE)
            continue;
        if (d.b0 != 0xf0 || d.b3 != 16 || d.b4 != d.b1 || d.len != 4)
            continue; /* MOVG2 @rN+disp16 rN (same register) */
        tpage = flat >> 16;
        toff = flat & 0xffff;
        nxt = flat + 4;
        b2 = rb(nxt + 1);
        if (rb(nxt) != 0x11 || (b2 >> 3) != 0x1a || (b2 & 7) != d.b1)
            continue; /* not immediately followed by jmp rN (same register) */
        /* disp16 = bytes flat+1..flat+2 ([fb][hi][lo][opcode]) */
        base = ((uint32_t)rb(flat + 1) << 8) | rb(flat + 2);
        {
            int added = 0, skipped = 0;
            table_sites++;
            for (k = 0; k < 256; k++) {
                uint32_t w = ((uint32_t)rb((tpage << 16) | (base + 2 * k)) << 8) |
                             rb((tpage << 16) | (base + 2 * k + 1));
                uint32_t cand = (tpage << 16) | w;
                if (w == 0)
                    break;
                table_entries++;
                if (cand >= ADDR_SPACE || !is_code_page(cand) ||
                    in_data_range(cand) || !h8dec(cand).valid) {
                    skipped++;
                    table_skipped++;
                    continue;
                }
                if (!set_has(&all, cand)) {
                    set_add(&all, cand);
                    added++;
                    table_added++;
                }
            }
            if (!ft) {
                snprintf(path, sizeof path, "%s/reach_tables.txt", outdir);
                ft = fopen(path, "wb");
                if (!ft)
                    die("cannot write '%s'", path);
                fputs("site register table_off entries added skipped\n", ft);
            }
            fprintf(ft, "%08x r%d %06x %u %d %d\n",
                    flat, d.b1, base, (unsigned)k, added, skipped);
        }
    }
    if (ft)
        fclose(ft);

    /* new = closure minus E (after table expansion) */
    for (x = 0; x < ADDR_SPACE; x++) {
        if (all.m[x] && !e.m[x]) {
            set_add(&newset, x);
            if (v.m[x])
                n_new_vec++;
            else
                n_new_static++;
        }
    }

    snprintf(path, sizeof path, "%s/pc_reach.txt", outdir);
    f = fopen(path, "wb");
    if (!f)
        die("cannot write '%s'", path);
    for (x = 0; x < ADDR_SPACE; x++)
        if (all.m[x])
            fprintf(f, "%08x\n", x);
    fclose(f);

    snprintf(path, sizeof path, "%s/reach_new.txt", outdir);
    f = fopen(path, "wb");
    if (!f)
        die("cannot write '%s'", path);
    for (x = 0; x < ADDR_SPACE; x++)
        if (newset.m[x])
            fprintf(f, "%08x\n", x);
    fclose(f);

    snprintf(path, sizeof path, "%s/reach_vec.txt", outdir);
    f = fopen(path, "wb");
    if (!f)
        die("cannot write '%s'", path);
    for (x = 0; x < ADDR_SPACE; x++)
        if (v.m[x])
            fprintf(f, "%08x\n", x);
    fclose(f);

    sha256_hex(rom1, rom1_size, sha1);
    sha256_hex(rom2, rom2_size, sha2);
    {
        uint32_t sz;
        uint8_t *b;
        b = load_file(pc_path, &sz);
        sha256_hex(b, sz, shapc);
        free(b);
        b = load_file(vec_path, &sz);
        sha256_hex(b, sz, shavec);
        free(b);
        b = load_file(s_path, &sz);
        sha256_hex(b, sz, shas);
        free(b);
    }

    {
        static const char *const STOP_NAME[STOP_N] = {
            "invalid-decode", "data-range", "outside-code-page",
            "ret-dynamic", "regind-dynamic"
        };
        snprintf(path, sizeof path, "%s/reach_stops.txt", outdir);
        f = fopen(path, "wb");
        if (!f)
            die("cannot write '%s'", path);
        for (i = 0; i < STOP_N; i++)
            for (x = 0; x < ADDR_SPACE; x++)
                if (stopmap[i][x])
                    fprintf(f, "%08x %s\n", x, STOP_NAME[i]);
        fclose(f);
    }

    snprintf(path, sizeof path, "%s/reach_stats.txt", outdir);
    f = fopen(path, "wb");
    if (!f)
        die("cannot write '%s'", path);
    fprintf(f, "h8reach stats (M2 static reachability)\n");
    fprintf(f, "========================================\n");
    fprintf(f, "rom1 = %s\n", rom1_path);
    fprintf(f, "rom2 = %s\n", rom2_path);
    fprintf(f, "rom1_sha256 = %s\n", sha1);
    fprintf(f, "rom2_sha256 = %s\n", sha2);
    fprintf(f, "pc_main = %s\n", pc_path);
    fprintf(f, "pc_main_sha256 = %s\n", shapc);
    fprintf(f, "pc_vec = %s\n", vec_path);
    fprintf(f, "pc_vec_sha256 = %s\n", shavec);
    fprintf(f, "r16_reach = %s\n", s_path);
    fprintf(f, "r16_reach_sha256 = %s\n", shas);
    fprintf(f, "exec lines = %ld, unique = %u\n", e_lines, (unsigned)e.n);
    fprintf(f, "vec lines = %ld, unique code-page entries = %u\n",
            v_lines, (unsigned)v.n);
    fprintf(f, "static-scan lines = %ld, unique = %u\n", s_lines, (unsigned)s.n);
    fprintf(f, "roots (E ∪ V ∪ S) = %ld\n", roots);
    fprintf(f, "closure set (all) = %u\n", (unsigned)all.n);
    fprintf(f, "new vs E = %u (vec-roots %ld, static %ld, incl. table cands)\n",
            (unsigned)newset.n, n_new_vec, n_new_static);
    fprintf(f, "stops: invalid-decode = %ld, data-range = %ld, outside-code-page = %ld, "
               "ret-dynamic = %ld, regind-dynamic = %ld\n",
            stop_count[STOP_INVALID], stop_count[STOP_DATA],
            stop_count[STOP_PAGE], stop_count[STOP_RET], stop_count[STOP_REGIND]);
    fprintf(f, "jump-table sites = %ld, entries read = %ld, added = %ld, skipped = %ld\n",
            table_sites, table_entries, table_added, table_skipped);
    fclose(f);

    printf("h8reach: all=%u new=%u (vec %ld, static %ld) tables: sites=%ld "
           "entries=%ld added=%ld -> %s\n",
           (unsigned)all.n, (unsigned)newset.n, n_new_vec, n_new_static,
           table_sites, table_entries, table_added, outdir);

    free(rom1);
    free(rom2);
    free(ev.a);
    free(vv.a);
    free(sv.a);
    free(e.m);
    free(v.m);
    free(s.m);
    free(all.m);
    free(newset.m);
    for (i = 0; i < STOP_N; i++)
        free(stopmap[i]);
    return 0;
}
