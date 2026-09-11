/*
 * cover.c - coverage dashboard for the mk2cpp ROM -> C++ translation.
 *
 * Reads the transliterator's map.csv plus a set of executed (and optionally
 * statically reachable) flat PCs, and reports how much of each set is
 * translated.
 *
 *   cover --map <map.csv> --exec <pcset.txt> [--static <hits.txt>]
 *         [--out <coverage.md>] [--missing-limit N]
 *
 * map.csv schema (mk2cpp/docs/06_h8lift_design.md section 4.1):
 *   flat,rom,fileoff,len,kind,cat,block_fn,block_start,rel
 *   - flat: 8 hex digits, flat = (cp << 16) | pc
 *   - cat:  EXEC | VEC | STATIC | DATA | TRAP | OVERLAP | UNKNOWN
 *   - a translated instruction is a row with non-empty block_fn whose cat is
 *     neither DATA nor TRAP.
 *
 * Parsing is tolerant: the header row is optional, quoted CSV fields are
 * handled, extra columns are ignored, comment lines starting with '#' or
 * '//' are skipped, and rows whose flat does not parse are counted as
 * malformed and skipped.
 *
 * Exec input: one flat per line (first whitespace token is used).
 * Static input: auto-detected per line, either the r16 scan format
 *   `cat arr rom flat fileoff bytes hitsrc decode` (field 4 = flat) or a
 *   plain one-flat-per-line file.
 *
 * Output is Markdown, written to stdout and, with --out, also to a file
 * (LF line endings).
 */

#define _CRT_SECURE_NO_WARNINGS 1

#include <ctype.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* small helpers                                                       */
/* ------------------------------------------------------------------ */

typedef struct {
    char  *p;
    size_t len;
    size_t cap;
} Buf;

static void die(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    fputs("cover: ", stderr);
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
    exit(1);
}

static void *xrealloc(void *p, size_t n) {
    void *q = realloc(p, n);
    if (!q) die("out of memory");
    return q;
}

static void buf_reserve(Buf *b, size_t extra) {
    size_t nc;
    if (b->len + extra + 1 <= b->cap) return;
    nc = b->cap ? b->cap : 4096;
    while (nc < b->len + extra + 1) {
        if (nc > (size_t)-1 / 2) die("output too large");
        nc *= 2;
    }
    b->p = (char *)xrealloc(b->p, nc);
    b->cap = nc;
}

static void buf_puts(Buf *b, const char *s) {
    size_t n = strlen(s);
    buf_reserve(b, n);
    memcpy(b->p + b->len, s, n);
    b->len += n;
    b->p[b->len] = '\0';
}

static void buf_addf(Buf *b, const char *fmt, ...) {
    va_list ap, ap2;
    int n;
    va_start(ap, fmt);
    va_copy(ap2, ap);
    n = vsnprintf(NULL, 0, fmt, ap);
    va_end(ap);
    if (n < 0) {
        va_end(ap2);
        die("formatting failed");
    }
    buf_reserve(b, (size_t)n);
    vsnprintf(b->p + b->len, (size_t)n + 1, fmt, ap2);
    va_end(ap2);
    b->len += (size_t)n;
}

static int ieq(const char *a, const char *b) {
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return *a == '\0' && *b == '\0';
}

static void upcopy(char *dst, size_t cap, const char *src) {
    size_t i;
    for (i = 0; src[i] && i + 1 < cap; i++)
        dst[i] = (char)toupper((unsigned char)src[i]);
    dst[i] = '\0';
}

static void strip_eol(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) s[--n] = '\0';
}

static void strip_bom(char *s) {
    if ((unsigned char)s[0] == 0xEF && (unsigned char)s[1] == 0xBB &&
        (unsigned char)s[2] == 0xBF)
        memmove(s, s + 3, strlen(s + 3) + 1);
}

static char *trim_left(char *s) {
    while (*s == ' ' || *s == '\t') s++;
    return s;
}

static int is_comment(const char *s) {
    return s[0] == '#' || (s[0] == '/' && s[1] == '/');
}

/* strict hex parse (optional 0x/0X prefix); whole string must be hex */
static int parse_hex32(const char *s, uint32_t *out) {
    uint64_t v = 0;
    const char *p = s;
    if (!s || !*s) return 0;
    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
    if (!*p) return 0;
    for (; *p; p++) {
        int d;
        unsigned char c = (unsigned char)*p;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else return 0;
        v = (v << 4) | (uint64_t)d;
        if (v > 0xFFFFFFFFu) return 0;
    }
    *out = (uint32_t)v;
    return 1;
}

/*
 * Split a CSV line in place into fields. Handles double-quoted fields
 * (including "" escapes), trims unquoted spaces/tabs, and stops after
 * max fields. Returns the number of fields stored in out[].
 */
static int csv_split(char *line, char *out[], int max) {
    int n = 0;
    char *p = line;
    while (n < max) {
        char *dst, *comma, *end, *w;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') {
            out[n++] = p;
            break;
        }
        if (*p == '"') {
            p++;
            dst = p;
            w = p;
            while (*p) {
                if (*p == '"') {
                    if (p[1] == '"') {
                        *w++ = '"';
                        p += 2;
                    } else {
                        p++;
                        break;
                    }
                } else {
                    *w++ = *p++;
                }
            }
            *w = '\0';
            while (*p == ' ' || *p == '\t') p++;
            out[n++] = dst;
        } else {
            int has_sep;
            dst = p;
            while (*p && *p != ',') p++;
            comma = p;
            has_sep = (*comma == ',');
            end = p;
            while (end > dst && (end[-1] == ' ' || end[-1] == '\t')) end--;
            *end = '\0';
            out[n++] = dst;
            if (!has_sep) break;
            p = comma + 1;
            continue;
        }
        if (*p == ',') {
            p++;
            continue;
        }
        break;
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* flat-address vectors (sorted + unique)                              */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t *a;
    size_t    n;
    size_t    cap;
} U32Vec;

static void u32_push(U32Vec *v, uint32_t x) {
    if (v->n == v->cap) {
        v->cap = v->cap ? v->cap * 2 : 1024;
        v->a = (uint32_t *)xrealloc(v->a, v->cap * sizeof(uint32_t));
    }
    v->a[v->n++] = x;
}

static int cmp_u32(const void *pa, const void *pb) {
    uint32_t a = *(const uint32_t *)pa;
    uint32_t b = *(const uint32_t *)pb;
    return (a > b) - (a < b);
}

static void u32_sort_unique(U32Vec *v) {
    size_t i, w;
    if (v->n < 2) return;
    qsort(v->a, v->n, sizeof(uint32_t), cmp_u32);
    w = 1;
    for (i = 1; i < v->n; i++)
        if (v->a[i] != v->a[w - 1]) v->a[w++] = v->a[i];
    v->n = w;
}

static int u32_contains(const U32Vec *v, uint32_t x) {
    if (v->n == 0) return 0;
    return bsearch(&x, v->a, v->n, sizeof(uint32_t), cmp_u32) != NULL;
}

/* ------------------------------------------------------------------ */
/* map.csv                                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    char   cat[32];
    size_t count;
} CatCount;

typedef struct {
    CatCount *a;
    size_t    n;
    size_t    cap;
} CatVec;

typedef struct {
    size_t   rows;
    size_t   translated;
    size_t   malformed;
    CatVec   cats;
    U32Vec   translated_flat;
} MapData;

static void cat_add(CatVec *v, const char *cat) {
    size_t i;
    for (i = 0; i < v->n; i++) {
        if (ieq(v->a[i].cat, cat)) {
            v->a[i].count++;
            return;
        }
    }
    if (v->n == v->cap) {
        v->cap = v->cap ? v->cap * 2 : 8;
        v->a = (CatCount *)xrealloc(v->a, v->cap * sizeof(CatCount));
    }
    upcopy(v->a[v->n].cat, sizeof(v->a[v->n].cat), cat);
    v->a[v->n].count = 1;
    v->n++;
}

static void load_map(const char *path, MapData *m) {
    FILE *f = fopen(path, "rb");
    char line[4096];
    if (!f) die("cannot open map file '%s'", path);
    while (fgets(line, sizeof line, f)) {
        char *fields[16];
        char *p;
        int nf;
        uint32_t flat;
        const char *cat, *fn;
        strip_eol(line);
        strip_bom(line);
        p = trim_left(line);
        if (!*p || is_comment(p)) continue;
        nf = csv_split(p, fields, 16);
        if (nf == 0 || fields[0][0] == '\0') continue;
        if (ieq(fields[0], "flat")) continue; /* documented header row */
        if (!parse_hex32(fields[0], &flat)) {
            m->malformed++;
            continue;
        }
        m->rows++;
        cat = nf > 5 ? fields[5] : "";
        fn = nf > 6 ? fields[6] : "";
        cat_add(&m->cats, cat[0] ? cat : "(none)");
        if (fn[0] != '\0' && !ieq(cat, "DATA") && !ieq(cat, "TRAP")) {
            m->translated++;
            u32_push(&m->translated_flat, flat);
        }
    }
    fclose(f);
    u32_sort_unique(&m->translated_flat);
}

/* ------------------------------------------------------------------ */
/* PC inputs                                                           */
/* ------------------------------------------------------------------ */

/*
 * auto_field = 0: take token 0 (exec pcset format).
 * auto_field = 1: if the line looks like r16 (>= 7 fields, short first
 * token) take field 4 (token 3); otherwise take token 0.
 */
static void read_pcs(const char *path, U32Vec *out, int auto_field) {
    FILE *f = fopen(path, "rb");
    char line[4096];
    if (!f) die("cannot open PC file '%s'", path);
    while (fgets(line, sizeof line, f)) {
        char *tok[8];
        int nt = 0;
        char *p;
        uint32_t v;
        strip_eol(line);
        strip_bom(line);
        p = trim_left(line);
        if (!*p || is_comment(p)) continue;
        while (*p && nt < 8) {
            while (*p == ' ' || *p == '\t' || *p == ',') p++;
            if (!*p) break;
            tok[nt++] = p;
            while (*p && *p != ' ' && *p != '\t' && *p != ',') p++;
            if (*p) *p++ = '\0';
        }
        if (nt == 0) continue;
        if (auto_field && nt >= 7 && strlen(tok[0]) < 4 &&
            parse_hex32(tok[3], &v)) {
            u32_push(out, v); /* r16: cat arr rom flat ... */
        } else if (parse_hex32(tok[0], &v)) {
            u32_push(out, v); /* plain: flat first (also 8-hex-digit standard) */
        } else if (auto_field && nt >= 7 && parse_hex32(tok[3], &v)) {
            u32_push(out, v); /* r16 with an unrecognized category token */
        }
    }
    fclose(f);
    u32_sort_unique(out);
}

/* ------------------------------------------------------------------ */
/* report                                                              */
/* ------------------------------------------------------------------ */

static const char *const CAT_ORDER[] = {
    "EXEC", "VEC", "STATIC", "DATA", "TRAP", "OVERLAP", "UNKNOWN", "(none)"
};
#define CAT_ORDER_N (sizeof(CAT_ORDER) / sizeof(CAT_ORDER[0]))

static int cat_rank(const char *c) {
    size_t i;
    for (i = 0; i < CAT_ORDER_N; i++)
        if (ieq(c, CAT_ORDER[i])) return (int)i;
    return (int)CAT_ORDER_N;
}

static int cmp_cat(const void *pa, const void *pb) {
    const CatCount *a = (const CatCount *)pa;
    const CatCount *b = (const CatCount *)pb;
    int ra = cat_rank(a->cat);
    int rb = cat_rank(b->cat);
    if (ra != rb) return ra - rb;
    return strcmp(a->cat, b->cat);
}

static void fmt_pct(char *buf, size_t cap, size_t covered, size_t total) {
    if (total == 0) snprintf(buf, cap, "n/a");
    else snprintf(buf, cap, "%.2f%%", 100.0 * (double)covered / (double)total);
}

static void emit_missing(Buf *b, const char *title, const U32Vec *set,
                         const U32Vec *translated, size_t limit) {
    size_t i, missing = 0, shown = 0;
    for (i = 0; i < set->n; i++)
        if (!u32_contains(translated, set->a[i])) missing++;
    buf_addf(b, "## %s\n\n", title);
    if (set->n == 0) {
        buf_puts(b, "no input PCs.\n\n");
        return;
    }
    if (missing == 0) {
        buf_addf(b, "none - all %zu PCs translated.\n\n", set->n);
        return;
    }
    buf_addf(b, "%zu uncovered of %zu (showing first %zu):\n\n```\n",
             missing, set->n, missing < limit ? missing : limit);
    for (i = 0; i < set->n && shown < limit; i++) {
        if (!u32_contains(translated, set->a[i])) {
            buf_addf(b, "%08x\n", (unsigned)set->a[i]);
            shown++;
        }
    }
    buf_puts(b, "```\n\n");
}

/* ------------------------------------------------------------------ */
/* CLI                                                                 */
/* ------------------------------------------------------------------ */

static void usage(FILE *f) {
    fputs(
        "usage: cover --map <map.csv> --exec <pcset.txt> [--static <hits.txt>]\n"
        "             [--out <coverage.md>] [--missing-limit N]\n"
        "\n"
        "  --map            transliterator map.csv (required)\n"
        "  --exec           executed PC set, one flat hex per line (required)\n"
        "  --static         static scan hits, plain or r16 text format\n"
        "  --out            also write the Markdown report to this file\n"
        "  --missing-limit  max uncovered PCs listed per set (default 200)\n",
        f);
}

static const char *eq_value(const char *arg, const char *name) {
    size_t n = strlen(name);
    if (strncmp(arg, name, n) == 0 && arg[n] == '=') return arg + n + 1;
    return NULL;
}

static int parse_limit(const char *s, long *out) {
    char *end;
    long v;
    if (!s || !*s) return 0;
    v = strtol(s, &end, 10);
    if (*end != '\0' || v < 0) return 0;
    *out = v;
    return 1;
}

int main(int argc, char **argv) {
    const char *map_path = NULL;
    const char *exec_path = NULL;
    const char *static_path = NULL;
    const char *out_path = NULL;
    long missing_limit = 200;
    MapData map;
    U32Vec exec_set, static_set;
    Buf out;
    size_t exec_cov = 0, static_cov = 0;
    char exec_pct[32], static_pct[32];
    int i;

    memset(&map, 0, sizeof map);
    memset(&exec_set, 0, sizeof exec_set);
    memset(&static_set, 0, sizeof static_set);
    memset(&out, 0, sizeof out);

    for (i = 1; i < argc; i++) {
        const char *a = argv[i];
        const char *v;
        if ((v = eq_value(a, "--map")) != NULL) map_path = v;
        else if ((v = eq_value(a, "--exec")) != NULL) exec_path = v;
        else if ((v = eq_value(a, "--static")) != NULL) static_path = v;
        else if ((v = eq_value(a, "--out")) != NULL) out_path = v;
        else if ((v = eq_value(a, "--missing-limit")) != NULL) {
            if (!parse_limit(v, &missing_limit)) {
                fprintf(stderr, "cover: bad --missing-limit '%s'\n", v);
                return 2;
            }
        } else if (!strcmp(a, "--map") && i + 1 < argc) map_path = argv[++i];
        else if (!strcmp(a, "--exec") && i + 1 < argc) exec_path = argv[++i];
        else if (!strcmp(a, "--static") && i + 1 < argc) static_path = argv[++i];
        else if (!strcmp(a, "--out") && i + 1 < argc) out_path = argv[++i];
        else if (!strcmp(a, "--missing-limit") && i + 1 < argc) {
            if (!parse_limit(argv[++i], &missing_limit)) {
                fprintf(stderr, "cover: bad --missing-limit '%s'\n", argv[i]);
                return 2;
            }
        } else if (!strcmp(a, "-h") || !strcmp(a, "--help")) {
            usage(stdout);
            return 0;
        } else {
            fprintf(stderr, "cover: unrecognized argument '%s'\n", a);
            usage(stderr);
            return 2;
        }
    }
    if (!map_path || !exec_path) {
        usage(stderr);
        return 2;
    }

    load_map(map_path, &map);
    read_pcs(exec_path, &exec_set, 0);
    if (static_path) read_pcs(static_path, &static_set, 1);

    for (i = 0; i < (int)exec_set.n; i++)
        if (u32_contains(&map.translated_flat, exec_set.a[i])) exec_cov++;
    for (i = 0; i < (int)static_set.n; i++)
        if (u32_contains(&map.translated_flat, static_set.a[i])) static_cov++;
    fmt_pct(exec_pct, sizeof exec_pct, exec_cov, exec_set.n);
    fmt_pct(static_pct, sizeof static_pct, static_cov, static_set.n);

    buf_puts(&out, "# mk2cpp coverage report\n\n");
    buf_puts(&out, "| metric | value |\n|---|---:|\n");
    buf_addf(&out, "| map | `%s` |\n", map_path);
    buf_addf(&out, "| exec | `%s` |\n", exec_path);
    if (static_path) buf_addf(&out, "| static | `%s` |\n", static_path);
    buf_addf(&out, "| missing-limit | %ld |\n", missing_limit);
    buf_addf(&out, "| map rows | %zu |\n", map.rows);
    if (map.malformed)
        buf_addf(&out, "| map rows skipped (bad flat) | %zu |\n", map.malformed);
    buf_addf(&out, "| translated rows | %zu |\n", map.translated);
    buf_addf(&out, "| exec PCs | %zu |\n", exec_set.n);
    buf_addf(&out, "| exec covered | %zu |\n", exec_cov);
    buf_addf(&out, "| exec coverage | %s |\n", exec_pct);
    if (static_path) {
        buf_addf(&out, "| static PCs | %zu |\n", static_set.n);
        buf_addf(&out, "| static covered | %zu |\n", static_cov);
        buf_addf(&out, "| static coverage | %s |\n", static_pct);
    }

    buf_puts(&out, "\n## map categories\n\n| category | rows |\n|---|---:|\n");
    {
        CatVec sorted = map.cats;
        qsort(sorted.a, sorted.n, sizeof(CatCount), cmp_cat);
        for (i = 0; i < (int)sorted.n; i++)
            buf_addf(&out, "| %s | %zu |\n", sorted.a[i].cat, sorted.a[i].count);
    }
    buf_puts(&out, "\n");

    emit_missing(&out, "missing executed PCs", &exec_set,
                 &map.translated_flat, (size_t)missing_limit);
    if (static_path)
        emit_missing(&out, "missing static PCs", &static_set,
                     &map.translated_flat, (size_t)missing_limit);

    fwrite(out.p ? out.p : "", 1, out.len, stdout);
    if (out_path) {
        FILE *f = fopen(out_path, "wb");
        if (!f) die("cannot write report '%s'", out_path);
        if (fwrite(out.p, 1, out.len, f) != out.len) {
            fclose(f);
            die("short write to '%s'", out_path);
        }
        fclose(f);
    }
    return 0;
}
