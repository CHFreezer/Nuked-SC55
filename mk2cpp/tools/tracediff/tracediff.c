#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    RES_IDENTICAL = 0,
    RES_DIVERGENCE = 1,
    RES_PREFIX_REF = 2,
    RES_PREFIX_DUT = 3
};

typedef struct {
    char type;
    unsigned long long cycle;
    unsigned long cp;
    unsigned long pc;
    int valid;
} Event;

typedef struct {
    unsigned long long ref_line;
    unsigned long long dut_line;
    char *ref;
    char *dut;
} Pair;

typedef struct {
    int kind;
    const char *ref_path;
    const char *dut_path;
    unsigned long long ref_lines;
    unsigned long long dut_lines;
    unsigned long long ref_no;
    unsigned long long dut_no;
    char *ref_line;
    char *dut_line;
    const Pair *hist;
    unsigned long long hist_cap;
    unsigned long long hist_count;
    unsigned long long hist_head;
} Result;

static void usage(FILE *out, const char *prog)
{
    fprintf(out,
            "usage: %s --ref <file> --dut <file> [--history N] "
            "[--report out.md] [--divergence out.txt]\n",
            prog);
}

static void *xmalloc(size_t n)
{
    void *p = malloc(n);
    if (!p) {
        fprintf(stderr, "tracediff: out of memory\n");
        exit(2);
    }
    return p;
}

static char *xstrdup(const char *s)
{
    size_t n = strlen(s) + 1;
    char *p = (char *)xmalloc(n);
    memcpy(p, s, n);
    return p;
}

static void strip_bom(char *s)
{
    if ((unsigned char)s[0] == 0xEF && (unsigned char)s[1] == 0xBB &&
        (unsigned char)s[2] == 0xBF)
        memmove(s, s + 3, strlen(s + 3) + 1);
}

static char *read_line(FILE *f, char **buf, size_t *cap)
{
    size_t used = 0;

    for (;;) {
        if (used + 2 > *cap) {
            size_t ncap = *cap ? *cap * 2 : 256;
            char *nb = (char *)realloc(*buf, ncap);
            if (!nb) {
                fprintf(stderr, "tracediff: out of memory\n");
                exit(2);
            }
            *buf = nb;
            *cap = ncap;
        }
        if (!fgets(*buf + used, (int)(*cap - used), f)) {
            if (used == 0)
                return NULL;
            break;
        }
        used += strlen(*buf + used);
        if ((*buf)[used - 1] == '\n')
            break;
    }
    while (used > 0 && ((*buf)[used - 1] == '\n' || (*buf)[used - 1] == '\r'))
        used--;
    (*buf)[used] = '\0';
    return *buf;
}

static int is_blank(const char *s)
{
    while (*s) {
        if (!isspace((unsigned char)*s))
            return 0;
        s++;
    }
    return 1;
}

static int parse_event(const char *s, Event *ev)
{
    const char *p = s;
    char *end;
    unsigned long long cycle;
    unsigned long cp = 0;

    ev->type = 0;
    ev->cycle = 0;
    ev->cp = 0;
    ev->pc = 0;
    ev->valid = 0;

    while (isspace((unsigned char)*p))
        p++;
    if (*p == 'm' || *p == 'M')
        ev->type = 'm';
    else if (*p == 's' || *p == 'S')
        ev->type = 's';
    else
        return 0;
    p++;
    if (!isspace((unsigned char)*p))
        return 0;
    while (isspace((unsigned char)*p))
        p++;

    if (!isdigit((unsigned char)*p))
        return 0;
    errno = 0;
    cycle = strtoull(p, &end, 10);
    if (end == p || errno == ERANGE)
        return 0;
    p = end;
    if (!isspace((unsigned char)*p))
        return 0;
    while (isspace((unsigned char)*p))
        p++;

    if (ev->type == 'm') {
        if (!isxdigit((unsigned char)*p))
            return 0;
        errno = 0;
        cp = strtoul(p, &end, 16);
        if (end == p || errno == ERANGE || cp > 0xFFul)
            return 0;
        if (*end != ':')
            return 0;
        ev->cp = cp;
        p = end + 1;
    }
    if (!isxdigit((unsigned char)*p))
        return 0;
    errno = 0;
    ev->pc = strtoul(p, &end, 16);
    if (end == p || errno == ERANGE || ev->pc > 0xFFFFul)
        return 0;
    p = end;
    while (isspace((unsigned char)*p))
        p++;
    if (*p)
        return 0;

    ev->cycle = cycle;
    ev->valid = 1;
    return 1;
}

static int events_differ(const Event *a, const Event *b)
{
    return a->type != b->type || a->cycle != b->cycle || a->cp != b->cp ||
           a->pc != b->pc;
}

static void hist_push(Pair *hist, unsigned long long cap,
                      unsigned long long *count, unsigned long long *head,
                      unsigned long long ref_line, unsigned long long dut_line,
                      const char *ref, const char *dut)
{
    unsigned long long idx;
    if (cap == 0)
        return;
    idx = *head % cap;
    free(hist[idx].ref);
    free(hist[idx].dut);
    hist[idx].ref_line = ref_line;
    hist[idx].dut_line = dut_line;
    hist[idx].ref = xstrdup(ref);
    hist[idx].dut = xstrdup(dut);
    (*head)++;
    if (*count < cap)
        (*count)++;
}

static void print_history(FILE *out, const Result *r)
{
    unsigned long long hh, start, k;

    if (!r->hist || r->hist_cap == 0 || r->hist_count == 0)
        return;
    hh = r->hist_head % r->hist_cap;
    start = (hh + r->hist_cap - (r->hist_count % r->hist_cap)) % r->hist_cap;
    fprintf(out, "history (last %llu matching pair%s):\n", r->hist_count,
            r->hist_count == 1 ? "" : "s");
    for (k = 0; k < r->hist_count; k++) {
        unsigned long long idx = (start + k) % r->hist_cap;
        fprintf(out, "  ref[%llu]: %s\n", r->hist[idx].ref_line, r->hist[idx].ref);
        fprintf(out, "  dut[%llu]: %s\n", r->hist[idx].dut_line, r->hist[idx].dut);
    }
}

static void report_stdout(const Result *r)
{
    switch (r->kind) {
    case RES_IDENTICAL:
        printf("identical: ref %llu lines, dut %llu lines\n", r->ref_lines,
               r->dut_lines);
        break;
    case RES_DIVERGENCE:
        printf("first divergence at ref line %llu, dut line %llu\n", r->ref_no,
               r->dut_no);
        printf("  ref: %s\n", r->ref_line);
        printf("  dut: %s\n", r->dut_line);
        print_history(stdout, r);
        break;
    case RES_PREFIX_REF:
        printf("prefix: ref is a prefix of dut (ref %llu lines, dut %llu lines)\n",
               r->ref_lines, r->dut_lines);
        printf("  first extra dut line %llu: %s\n", r->dut_no, r->dut_line);
        print_history(stdout, r);
        break;
    case RES_PREFIX_DUT:
        printf("prefix: dut is a prefix of ref (ref %llu lines, dut %llu lines)\n",
               r->ref_lines, r->dut_lines);
        printf("  first extra ref line %llu: %s\n", r->ref_no, r->ref_line);
        print_history(stdout, r);
        break;
    }
}

static void report_markdown(FILE *f, const Result *r)
{
    fprintf(f, "# tracediff report\n\n");
    fprintf(f, "- ref: `%s`\n", r->ref_path);
    fprintf(f, "- dut: `%s`\n", r->dut_path);
    switch (r->kind) {
    case RES_IDENTICAL:
        fprintf(f, "- result: identical\n");
        break;
    case RES_DIVERGENCE:
        fprintf(f, "- result: divergence\n");
        fprintf(f, "- first divergence: ref line %llu / dut line %llu\n",
                r->ref_no, r->dut_no);
        break;
    case RES_PREFIX_REF:
        fprintf(f, "- result: prefix (ref is a prefix of dut)\n");
        fprintf(f, "- first unmatched: ref line %llu (EOF) / dut line %llu\n",
                r->ref_no + 1, r->dut_no);
        break;
    case RES_PREFIX_DUT:
        fprintf(f, "- result: prefix (dut is a prefix of ref)\n");
        fprintf(f, "- first unmatched: ref line %llu / dut line %llu (EOF)\n",
                r->ref_no, r->dut_no + 1);
        break;
    }
    fprintf(f, "- ref lines: %llu\n", r->ref_lines);
    fprintf(f, "- dut lines: %llu\n", r->dut_lines);
    if (r->kind != RES_IDENTICAL) {
        fprintf(f, "\n## First difference\n\n```\n");
        fprintf(f, "ref[%llu]: %s\n", r->ref_no,
                r->ref_line ? r->ref_line : "(EOF)");
        fprintf(f, "dut[%llu]: %s\n", r->dut_no,
                r->dut_line ? r->dut_line : "(EOF)");
        fprintf(f, "```\n");
    }
}

static void report_divergence(FILE *f, const Result *r)
{
    fprintf(f, "tracediff\n");
    fprintf(f, "ref: %s\n", r->ref_path);
    fprintf(f, "dut: %s\n", r->dut_path);
    switch (r->kind) {
    case RES_IDENTICAL:
        fprintf(f, "result: identical\n");
        break;
    case RES_DIVERGENCE:
        fprintf(f, "result: divergence at ref line %llu / dut line %llu\n",
                r->ref_no, r->dut_no);
        fprintf(f, "ref: %s\n", r->ref_line);
        fprintf(f, "dut: %s\n", r->dut_line);
        break;
    case RES_PREFIX_REF:
        fprintf(f, "result: prefix (ref is a prefix of dut)\n");
        fprintf(f, "first extra dut line %llu: %s\n", r->dut_no, r->dut_line);
        break;
    case RES_PREFIX_DUT:
        fprintf(f, "result: prefix (dut is a prefix of ref)\n");
        fprintf(f, "first extra ref line %llu: %s\n", r->ref_no, r->ref_line);
        break;
    }
    fprintf(f, "ref lines: %llu\n", r->ref_lines);
    fprintf(f, "dut lines: %llu\n", r->dut_lines);
    print_history(f, r);
}

int main(int argc, char **argv)
{
    const char *ref_path = NULL, *dut_path = NULL;
    const char *report_path = NULL, *div_path = NULL;
    unsigned long long history_n = 64;
    FILE *rf = NULL, *df = NULL;
    char *rbuf = NULL, *dbuf = NULL;
    size_t rcap = 0, dcap = 0;
    unsigned long long ref_no = 0, dut_no = 0;
    unsigned long long i;
    Pair *hist = NULL;
    unsigned long long hist_cap, hist_count = 0, hist_head = 0;
    char *diff_ref = NULL, *diff_dut = NULL;
    Result res;
    int status;

    for (i = 1; i < (unsigned long long)argc; i++) {
        if (!strcmp(argv[i], "--ref") && i + 1 < (unsigned long long)argc) {
            ref_path = argv[++i];
        } else if (!strcmp(argv[i], "--dut") && i + 1 < (unsigned long long)argc) {
            dut_path = argv[++i];
        } else if (!strcmp(argv[i], "--history") && i + 1 < (unsigned long long)argc) {
            char *end;
            unsigned long long n;
            const char *val = argv[++i];
            errno = 0;
            n = strtoull(val, &end, 10);
            if (end == val || *end || errno == ERANGE || n > 10000000ULL) {
                fprintf(stderr, "tracediff: invalid --history value '%s'\n", val);
                return 2;
            }
            history_n = n;
        } else if (!strcmp(argv[i], "--report") && i + 1 < (unsigned long long)argc) {
            report_path = argv[++i];
        } else if (!strcmp(argv[i], "--divergence") && i + 1 < (unsigned long long)argc) {
            div_path = argv[++i];
        } else {
            usage(stderr, argv[0]);
            return 2;
        }
    }
    if (!ref_path || !dut_path) {
        usage(stderr, argv[0]);
        return 2;
    }

    rf = fopen(ref_path, "rb");
    if (!rf) {
        fprintf(stderr, "tracediff: cannot open ref '%s': %s\n", ref_path,
                strerror(errno));
        return 2;
    }
    df = fopen(dut_path, "rb");
    if (!df) {
        fprintf(stderr, "tracediff: cannot open dut '%s': %s\n", dut_path,
                strerror(errno));
        fclose(rf);
        return 2;
    }

    hist_cap = history_n;
    if (hist_cap) {
        hist = (Pair *)calloc((size_t)hist_cap, sizeof(Pair));
        if (!hist) {
            fprintf(stderr, "tracediff: out of memory\n");
            fclose(rf);
            fclose(df);
            return 2;
        }
    }

    memset(&res, 0, sizeof(res));
    res.ref_path = ref_path;
    res.dut_path = dut_path;
    res.hist = hist;
    res.hist_cap = hist_cap;

    for (;;) {
        char *rl, *dl;
        Event re, de;
        int rv, dv, diff;

        do {
            rl = read_line(rf, &rbuf, &rcap);
            if (rl) {
                ref_no++;
                strip_bom(rl);
            }
        } while (rl && is_blank(rl));
        do {
            dl = read_line(df, &dbuf, &dcap);
            if (dl) {
                dut_no++;
                strip_bom(dl);
            }
        } while (dl && is_blank(dl));

        if (!rl && !dl) {
            res.kind = RES_IDENTICAL;
            break;
        }
        if (!rl) {
            res.kind = RES_PREFIX_REF;
            res.dut_no = dut_no;
            diff_dut = xstrdup(dl);
            break;
        }
        if (!dl) {
            res.kind = RES_PREFIX_DUT;
            res.ref_no = ref_no;
            diff_ref = xstrdup(rl);
            break;
        }

        rv = parse_event(rl, &re);
        dv = parse_event(dl, &de);
        diff = (rv && dv) ? events_differ(&re, &de) : (strcmp(rl, dl) != 0);
        if (diff) {
            res.kind = RES_DIVERGENCE;
            res.ref_no = ref_no;
            res.dut_no = dut_no;
            diff_ref = xstrdup(rl);
            diff_dut = xstrdup(dl);
            break;
        }
        hist_push(hist, hist_cap, &hist_count, &hist_head, ref_no, dut_no, rl, dl);
    }

    if (res.kind != RES_IDENTICAL) {
        char *t;
        while ((t = read_line(rf, &rbuf, &rcap)) != NULL)
            ref_no++;
        while ((t = read_line(df, &dbuf, &dcap)) != NULL)
            dut_no++;
    }

    if (ferror(rf) || ferror(df)) {
        fprintf(stderr, "tracediff: read error\n");
        status = 2;
        goto cleanup;
    }

    if (res.kind == RES_DIVERGENCE) {
        res.ref_line = diff_ref;
        res.dut_line = diff_dut;
    } else if (res.kind == RES_PREFIX_REF) {
        res.dut_line = diff_dut;
        res.ref_no = ref_no;
    } else if (res.kind == RES_PREFIX_DUT) {
        res.ref_line = diff_ref;
        res.dut_no = dut_no;
    }
    res.ref_lines = ref_no;
    res.dut_lines = dut_no;
    res.hist_count = hist_count;
    res.hist_head = hist_head;

    report_stdout(&res);

    if (div_path) {
        FILE *f = fopen(div_path, "wb");
        if (!f) {
            fprintf(stderr, "tracediff: cannot write '%s': %s\n", div_path,
                    strerror(errno));
            status = 2;
            goto cleanup;
        }
        report_divergence(f, &res);
        if (fclose(f) != 0) {
            fprintf(stderr, "tracediff: cannot write '%s': %s\n", div_path,
                    strerror(errno));
            status = 2;
            goto cleanup;
        }
    }
    if (report_path) {
        FILE *f = fopen(report_path, "wb");
        if (!f) {
            fprintf(stderr, "tracediff: cannot write '%s': %s\n", report_path,
                    strerror(errno));
            status = 2;
            goto cleanup;
        }
        report_markdown(f, &res);
        if (fclose(f) != 0) {
            fprintf(stderr, "tracediff: cannot write '%s': %s\n", report_path,
                    strerror(errno));
            status = 2;
            goto cleanup;
        }
    }

    status = (res.kind == RES_IDENTICAL) ? 0 : 1;

cleanup:
    free(rbuf);
    free(dbuf);
    if (hist) {
        for (i = 0; i < hist_cap; i++) {
            free(hist[i].ref);
            free(hist[i].dut);
        }
        free(hist);
    }
    free(diff_ref);
    free(diff_dut);
    fclose(rf);
    fclose(df);
    return status;
}
