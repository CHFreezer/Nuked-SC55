/*
 * h8part.c -- deterministic basic-block partitioner for the mk2cpp ROM -> C++
 * translation pipeline.
 *
 * Usage:
 *   h8part.exe <rom1.bin> <rom2.bin> <pc_main.txt> <flow_main.txt> <outdir>
 *              [--vec <vec_pcs.txt>] [--static <static_pcs.txt>]
 *
 * Every positional argument may be omitted, in which case it falls back to:
 *   build/rom1.bin build/rom2.bin tools/baselines/pc_main.txt
 *   tools/baselines/flow_main.txt mk2cpp/out/h8part
 *
 * --vec / --static add supplemental PC sets (from h8reach) to the main list
 * before partitioning; all sets are merged, sorted, and deduplicated.
 *
 * Outputs in <outdir>:
 *   map.csv     flat,rom,fileoff,len,kind,cat,block_fn,block_start,rel
 *   blocks.csv  byte-identical copy of map.csv
 *   stats.txt   human-readable counters
 *
 * Partitioner model (mk2cpp/docs/06_h8lift_design.md sections 1.3-1.6):
 *   - static successors come from the h8dec decode semantics only
 *       kind 0 none   : fall-through
 *       kind 1 call   : target + fall-through
 *       kind 2 uncond : target
 *       kind 3 cond   : target + fall-through
 *       kind 4 ret    : dynamic (stack)
 *       kind 5 reg-ind: dynamic (register value)
 *       trapa / sleep : no successors, block terminates
 *   - flow edges never add successors (kind-0 trace edges are interrupt
 *     injection, not control flow); they discover trace entries (in_deg == 0)
 *     and feed the ret/register-indirect registry and coverage analysis, but
 *     dynamic targets do not cut blocks (06 section 1.4 item 5).
 *   - blocks are maximal linear runs from an entry point; a cut is forced by
 *     an entry, by a terminator, or by a non-contiguous instruction.
 *   - a start that falls inside another decoded instruction's byte span is an
 *     OVERLAP entry: it is marked but never used to cut a run.
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
#define H8PART_MKDIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define H8PART_MKDIR(p) mkdir((p), 0755)
#endif

#define ADDR_SPACE (1u << 20)

enum {
    ENTRY_TRACE  = 1,
    ENTRY_STATIC = 2,
    ENTRY_FALL   = 4,
    ENTRY_VEC    = 8
};

enum { CAT_EXEC = 0, CAT_VEC = 1, CAT_OVERLAP = 2 };

typedef struct {
    uint32_t flat;
    uint32_t target;
    int      len;
    int      kind;
    uint8_t  op0;
    uint8_t  term;
    uint8_t  has_fall;
    uint8_t  entry;
    uint8_t  overlap;
    uint8_t  cat;
    uint32_t block_start;
    int      rel;
} pinsn_t;

typedef struct {
    uint32_t *a;
    size_t    n;
    size_t    cap;
} u32vec;

static const uint8_t *g_rom1;
static const uint8_t *g_rom2;
static uint32_t g_rom1_size;
static uint32_t g_rom2_size;

static void die(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fputs("h8part: ", stderr);
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
    buf = (uint8_t *)xrealloc(NULL, (size_t)n);
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

static uint8_t rom_byte(uint32_t flat)
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

static void vec_push(u32vec *v, uint32_t x)
{
    if (v->n == v->cap) {
        v->cap = v->cap ? v->cap * 2 : 1024;
        v->a = (uint32_t *)xrealloc(v->a, v->cap * sizeof(uint32_t));
    }
    v->a[v->n++] = x;
}

static int cmp_u32(const void *a, const void *b)
{
    uint32_t x = *(const uint32_t *)a;
    uint32_t y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

static void vec_sort_unique(u32vec *v)
{
    size_t i, w;
    if (v->n < 2)
        return;
    qsort(v->a, v->n, sizeof(uint32_t), cmp_u32);
    w = 1;
    for (i = 1; i < v->n; i++)
        if (v->a[i] != v->a[w - 1])
            v->a[w++] = v->a[i];
    v->n = w;
}

static const char *skip_ws(const char *p)
{
    while (*p == ' ' || *p == '\t')
        p++;
    return p;
}

static const char *skip_token(const char *p)
{
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n')
        p++;
    return p;
}

static long read_pcs(const char *path, u32vec *out)
{
    FILE *f = fopen(path, "rb");
    char line[512];
    long lines = 0;
    if (!f)
        die("cannot open pc set '%s'", path);
    while (fgets(line, sizeof line, f)) {
        uint32_t v;
        const char *p = skip_ws(line);
        if (!*p || *p == '\r' || *p == '\n' || *p == '#' ||
            (p[0] == '/' && p[1] == '/'))
            continue;
        if (parse_hex(p, &v)) {
            vec_push(out, v);
            lines++;
        }
    }
    fclose(f);
    vec_sort_unique(out);
    return lines;
}

static long read_flow(const char *path, uint32_t **ps, uint32_t **pd, size_t *pn)
{
    FILE *f = fopen(path, "rb");
    char line[512];
    uint32_t *s = NULL, *d = NULL;
    size_t n = 0, cap = 0;
    if (!f)
        die("cannot open flow set '%s'", path);
    while (fgets(line, sizeof line, f)) {
        uint32_t a, b;
        const char *p = skip_ws(line);
        if (!*p || *p == '\r' || *p == '\n' || *p == '#' ||
            (p[0] == '/' && p[1] == '/'))
            continue;
        if (!parse_hex(p, &a))
            continue;
        p = skip_ws(skip_token(p));
        if (!parse_hex(p, &b))
            continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 4096;
            s = (uint32_t *)xrealloc(s, cap * sizeof(uint32_t));
            d = (uint32_t *)xrealloc(d, cap * sizeof(uint32_t));
        }
        s[n] = a;
        d[n] = b;
        n++;
    }
    fclose(f);
    *ps = s;
    *pd = d;
    *pn = n;
    return (long)n;
}

static void ensure_outdir(const char *dir)
{
    char tmp[1024];
    size_t n = strlen(dir);
    char *p;
    if (n == 0 || n >= sizeof tmp)
        die("bad outdir '%s'", dir);
    memcpy(tmp, dir, n + 1);
    for (p = tmp + 1; *p; p++) {
        if (*p == '/' || *p == '\\') {
            char c = *p;
            *p = '\0';
            if (tmp[0])
                (void)H8PART_MKDIR(tmp);
            *p = c;
        }
    }
    (void)H8PART_MKDIR(tmp);
}

static void mark_reason(uint8_t *reasons, uint32_t flat, int bit, int32_t *insn_of)
{
    if (flat < ADDR_SPACE && insn_of[flat] >= 0)
        reasons[flat] |= (uint8_t)bit;
}

int main(int argc, char **argv)
{
    const char *rom1_path = NULL;
    const char *rom2_path = NULL;
    const char *pc_path   = NULL;
    const char *flow_path = NULL;
    const char *outdir    = NULL;
    const char *vec_path  = NULL;
    const char *static_path = NULL;
    int pos = 0;
    int i;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--vec") == 0) {
            if (++i >= argc) die("--vec requires a file argument");
            vec_path = argv[i];
        } else if (strcmp(argv[i], "--static") == 0) {
            if (++i >= argc) die("--static requires a file argument");
            static_path = argv[i];
        } else {
            switch (pos++) {
            case 0: rom1_path = argv[i]; break;
            case 1: rom2_path = argv[i]; break;
            case 2: pc_path   = argv[i]; break;
            case 3: flow_path = argv[i]; break;
            case 4: outdir    = argv[i]; break;
            default:
                die("usage: h8part.exe [rom1.bin rom2.bin pc_main.txt flow_main.txt outdir] [--vec <file>] [--static <file>]");
            }
        }
    }
    if (!rom1_path) rom1_path = "build/rom1.bin";
    if (!rom2_path) rom2_path = "build/rom2.bin";
    if (!pc_path)   pc_path   = "tools/baselines/pc_main.txt";
    if (!flow_path) flow_path = "tools/baselines/flow_main.txt";
    if (!outdir)    outdir    = "mk2cpp/out/h8part";

    uint32_t rom1_size = 0, rom2_size = 0;
    uint8_t *rom1;
    uint8_t *rom2;
    u32vec pcs;
    uint32_t *es = NULL, *ed = NULL;
    size_t ne = 0;
    long pc_lines, vec_lines = 0, static_lines = 0, flow_lines;
    uint8_t *indeg;
    uint8_t *reasons;
    pinsn_t *ins;
    int32_t *insn_of;
    int32_t *owner;
    size_t root = 0;
    long entries = 0, blocks = 0, overlaps = 0;
    int maxrun = 0;
    long kind_count[6] = {0, 0, 0, 0, 0, 0};
    long cat_count[3] = {0, 0, 0};
    long r_trace = 0, r_static = 0, r_fall = 0, r_vec = 0;
    size_t e;
    FILE *fm, *fb, *st;
    char path[1024];
    static const char *const KIND_NAME[6] = {
        "none", "call", "uncond", "cond", "ret", "reg-indirect"
    };

    memset(&pcs, 0, sizeof pcs);

    rom1 = load_file(rom1_path, &rom1_size);
    rom2 = load_file(rom2_path, &rom2_size);
    g_rom1 = rom1;
    g_rom1_size = rom1_size;
    g_rom2 = rom2;
    g_rom2_size = rom2_size;
    h8dec_set_rom(rom1, rom1_size, rom2, rom2_size);
    fprintf(stderr, "h8part: rom1=%s (%u) rom2=%s (%u)\n",
            rom1_path, rom1_size, rom2_path, rom2_size);

    pc_lines = read_pcs(pc_path, &pcs);
    if (pcs.n == 0)
        die("no executed PCs in '%s'", pc_path);
    if (vec_path) {
        size_t before = pcs.n;
        vec_lines = read_pcs(vec_path, &pcs);
        fprintf(stderr, "h8part: vec=%s (%zu new, total %zu)\n",
                vec_path, pcs.n - before, pcs.n);
    }
    if (static_path) {
        size_t before = pcs.n;
        static_lines = read_pcs(static_path, &pcs);
        fprintf(stderr, "h8part: static=%s (%zu new, total %zu)\n",
                static_path, pcs.n - before, pcs.n);
    }
    vec_sort_unique(&pcs);
    flow_lines = read_flow(flow_path, &es, &ed, &ne);

    indeg = (uint8_t *)xcalloc(ADDR_SPACE, 1);
    for (e = 0; e < ne; e++)
        if (ed[e] < ADDR_SPACE && indeg[ed[e]] != 0xff)
            indeg[ed[e]]++;

    ins = (pinsn_t *)xcalloc(pcs.n, sizeof *ins);
    insn_of = (int32_t *)xrealloc(NULL, ADDR_SPACE * sizeof *insn_of);
    memset(insn_of, 0xff, ADDR_SPACE * sizeof *insn_of);
    for (i = 0; i < pcs.n; i++) {
        uint32_t f = pcs.a[i];
        h8dec_t d = h8dec(f);
        int is_trapa, is_sleep;
        if (!d.valid || d.len <= 0)
            die("executed PC %08x does not decode (GT trap/data byte)", f);
        ins[i].flat = f;
        ins[i].target = d.target;
        ins[i].len = d.len;
        ins[i].kind = d.kind;
        ins[i].op0 = rom_byte(f);
        is_trapa = (ins[i].op0 == 0x08 && (rom_byte(f + 1) & 0xf0) == 0x10);
        is_sleep = (ins[i].op0 == 0x1a);
        ins[i].term = (d.kind != H8K_NONE) || is_trapa || is_sleep;
        ins[i].has_fall = (d.kind == H8K_CALL) || (d.kind == H8K_COND) ||
                          is_trapa || is_sleep;
        if (f < ADDR_SPACE)
            insn_of[f] = (int32_t)i;
    }

    reasons = (uint8_t *)xcalloc(ADDR_SPACE, 1);

    /* trace entry: executed PC with no observed incoming flow edge */
    for (i = 0; i < pcs.n; i++)
        if (ins[i].flat < ADDR_SPACE && indeg[ins[i].flat] == 0)
            mark_reason(reasons, ins[i].flat, ENTRY_TRACE, insn_of);

    /* static targets of call / uncond / cond instructions */
    for (i = 0; i < pcs.n; i++) {
        if (ins[i].kind == H8K_CALL || ins[i].kind == H8K_UNCOND ||
            ins[i].kind == H8K_COND)
            mark_reason(reasons, ins[i].target, ENTRY_STATIC, insn_of);
    }

    /* continuation after a call/cond/trapa/sleep (doc 06 section 1.4 item 3);
     * ret / register-indirect flow targets are registry/coverage inputs only,
     * they never cut a block (section 1.4 item 5) */
    for (i = 0; i < pcs.n; i++)
        if (ins[i].has_fall)
            mark_reason(reasons, ins[i].flat + (uint32_t)ins[i].len,
                        ENTRY_FALL, insn_of);

    /* vector supplemental list: not provided for this baseline (cat VEC only) */

    for (i = 0; i < pcs.n; i++) {
        uint32_t f = ins[i].flat;
        if (f < ADDR_SPACE && reasons[f]) {
            ins[i].entry = 1;
            entries++;
        }
    }

    /* overlap: an executed start inside another decoded instruction's span */
    owner = (int32_t *)xrealloc(NULL, ADDR_SPACE * sizeof *owner);
    memset(owner, 0xff, ADDR_SPACE * sizeof *owner);
    for (i = 0; i < pcs.n; i++) {
        uint32_t b;
        uint32_t f = ins[i].flat;
        if (f < ADDR_SPACE && owner[f] != -1)
            ins[i].overlap = 1;
        for (b = f; b < f + (uint32_t)ins[i].len && b < ADDR_SPACE; b++)
            if (owner[b] == -1)
                owner[b] = (int32_t)i;
    }

    /* maximal linear runs cut at entries, terminators and address gaps */
    for (i = 0; i < pcs.n; i++) {
        int cont = 0;
        if (i > 0) {
            pinsn_t *p = &ins[i - 1];
            int cut = ins[i].entry && !ins[i].overlap;
            cont = (p->flat + (uint32_t)p->len == ins[i].flat) &&
                   !p->term && !cut;
        }
        if (!cont) {
            root = i;
            blocks++;
            ins[i].rel = 0;
        } else {
            ins[i].rel = ins[i - 1].rel + 1;
        }
        ins[i].block_start = ins[root].flat;
        if (ins[i].rel + 1 > maxrun)
            maxrun = ins[i].rel + 1;
        ins[i].cat = ins[i].overlap ? CAT_OVERLAP : CAT_EXEC;
    }

    for (i = 0; i < pcs.n; i++) {
        uint8_t r = ins[i].flat < ADDR_SPACE ? reasons[ins[i].flat] : 0;
        if (ins[i].kind >= 0 && ins[i].kind < 6)
            kind_count[ins[i].kind]++;
        if (ins[i].cat < 3)
            cat_count[ins[i].cat]++;
        if (ins[i].overlap)
            overlaps++;
        if (r & ENTRY_TRACE)
            r_trace++;
        if (r & ENTRY_STATIC)
            r_static++;
        if (r & ENTRY_FALL)
            r_fall++;
        if (r & ENTRY_VEC)
            r_vec++;
    }

    ensure_outdir(outdir);

    snprintf(path, sizeof path, "%s/map.csv", outdir);
    fm = fopen(path, "wb");
    if (!fm)
        die("cannot write '%s'", path);
    snprintf(path, sizeof path, "%s/blocks.csv", outdir);
    fb = fopen(path, "wb");
    if (!fb)
        die("cannot write '%s'", path);

    {
        static const char *const HDR =
            "flat,rom,fileoff,len,kind,cat,block_fn,block_start,rel\n";
        fputs(HDR, fm);
        fputs(HDR, fb);
    }

    for (i = 0; i < pcs.n; i++) {
        uint32_t f = ins[i].flat;
        h8dec_t chk = h8dec(f);
        const char *rom;
        uint32_t fileoff;
        const char *cat;
        char row[256];
        int n;
        if (chk.len != ins[i].len || chk.kind != ins[i].kind)
            die("re-decode mismatch at %08x (len %d/%d kind %d/%d)",
                f, chk.len, ins[i].len, chk.kind, ins[i].kind);
        if ((f >> 16) == 0 && (f & 0xffff) < 0x8000) {
            rom = "r1";
            fileoff = f;
        } else {
            uint32_t idx = f & 0x3ffff;
            if (f & 0x80000)
                idx |= 0x40000;
            rom = "r2";
            fileoff = idx;
        }
        cat = ins[i].cat == CAT_OVERLAP ? "OVERLAP" :
              (ins[i].cat == CAT_VEC ? "VEC" : "EXEC");
        n = snprintf(row, sizeof row,
                     "%08x,%s,%06x,%d,%d,%s,mk2c_%s_%08x,%08x,%d\n",
                     f, rom, fileoff, ins[i].len, ins[i].kind, cat,
                     rom, f, ins[i].block_start, ins[i].rel);
        if (n <= 0 || n >= (int)sizeof row)
            die("row formatting failed at %08x", f);
        fwrite(row, 1, (size_t)n, fm);
        fwrite(row, 1, (size_t)n, fb);
    }
    fclose(fm);
    fclose(fb);

    snprintf(path, sizeof path, "%s/stats.txt", outdir);
    st = fopen(path, "wb");
    if (!st)
        die("cannot write '%s'", path);
    fprintf(st, "h8part stats\n============\n");
    fprintf(st, "rom1 = %s\n", rom1_path);
    fprintf(st, "rom2 = %s\n", rom2_path);
    fprintf(st, "pc_main = %s\n", pc_path);
    fprintf(st, "pc_vec = %s\n", vec_path ? vec_path : "(none)");
    fprintf(st, "pc_static = %s\n", static_path ? static_path : "(none)");
    fprintf(st, "flow_main = %s\n", flow_path);
    fprintf(st, "outdir = %s\n", outdir);
    fprintf(st, "pc lines = %ld (main %ld + vec %ld + static %ld)\n",
            pc_lines + vec_lines + static_lines, pc_lines, vec_lines,
            static_lines);
    fprintf(st, "pc unique = %zu\n", pcs.n);
    fprintf(st, "pc duplicates = %ld\n",
            pc_lines + vec_lines + static_lines - (long)pcs.n);
    fprintf(st, "flow lines = %ld\n", flow_lines);
    fprintf(st, "flow edges = %zu\n", ne);
    fprintf(st, "instructions = %zu\n", pcs.n);
    fprintf(st, "blocks = %ld\n", blocks);
    fprintf(st, "entries = %ld\n", entries);
    fprintf(st, "overlaps = %ld\n", overlaps);
    fprintf(st, "max block (instrs) = %d\n", maxrun);
    fprintf(st, "entry reasons:\n");
    fprintf(st, "  trace = %ld\n", r_trace);
    fprintf(st, "  static-target = %ld\n", r_static);
    fprintf(st, "  fall-after-term = %ld\n", r_fall);
    fprintf(st, "  vec = %ld\n", r_vec);
    fprintf(st, "per-kind:\n");
    {
        int k;
        for (k = 0; k < 6; k++)
            fprintf(st, "  kind %d %s = %ld\n", k, KIND_NAME[k], kind_count[k]);
    }
    fprintf(st, "per-cat:\n");
    fprintf(st, "  EXEC = %ld\n", cat_count[CAT_EXEC]);
    fprintf(st, "  VEC = %ld\n", cat_count[CAT_VEC]);
    fprintf(st, "  OVERLAP = %ld\n", cat_count[CAT_OVERLAP]);
    fclose(st);

    printf("h8part: instrs=%zu blocks=%ld entries=%ld overlaps=%ld "
           "maxrun=%d -> %s/{map.csv,blocks.csv,stats.txt}\n",
           pcs.n, blocks, entries, overlaps, maxrun, outdir);
    return 0;
}
