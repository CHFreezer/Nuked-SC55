// verify_dasm.c - C port of tools/verify/verify_dasm.py + semantics.py.
//
// Cross-validate h8dasm decode against ground truth (VM trace == GT trace).
//
// Phase A (length/target): every executed PC's decoded length must agree with
//   the observed sequential transitions in the PC trace; every observed
//   non-sequential transition must match the decoded branch target.
// Phase B (semantics): with a full-register trace, verify register/SP/sr
//   effects of every executed instruction against GT semantics (as encoded in
//   tools/vm/h8vm_body.c / src/mcu_opcodes.cpp). Memory-operand values are
//   unknown, so only register-deterministic effects are checked.
//
// Build:  clang -O2 -o verify_dasm.exe verify_dasm.c
// Usage:  verify_dasm.exe --mach mach.txt --trace vm_pc.txt [--regtrace vm.log]
//                         --rom1 rom1.bin --rom2 rom2.bin
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define AKEY_MAX (0x10 << 16)

// ---------------- failure bookkeeping ----------------
static int fail_count = 0;
static int fails_printed = 0;

enum { TAG_REG, TAG_REGCHG, TAG_SR, TAG_JMP, TAG_CR };
static const char *tag_name(int t)
{
    switch (t) {
    case TAG_REG:    return "REG";
    case TAG_REGCHG: return "REGCHG";
    case TAG_SR:     return "SR";
    case TAG_JMP:    return "JMP";
    case TAG_CR:     return "CR";
    }
    return "?";
}

typedef struct { int tag, op, top, reg, siz, ocode, ore; int n; } pat_t;
static pat_t pats[512];
static int npat = 0;

// ---------------- mach table ----------------
typedef struct {
    int exists;
    int len, kind, tpage, toff, top, reg, siz, ocode, ore, ext;
} dec_t;
static dec_t *mach;
static int mach_total = 0;

static void die(const char *msg)
{
    fprintf(stderr, "%s\n", msg);
    exit(2);
}

// May be called multiple times (--mach a --mach b): later files only fill
// entries not already present (e.g. a supplemental vector-target decode).
static void load_mach(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) die("cannot open mach file");
    if (!mach) mach = (dec_t *)calloc(AKEY_MAX, sizeof(dec_t));
    char line[256];
    int n = 0;
    while (fgets(line, sizeof line, f)) {
        unsigned pc, tp, to, top;
        int len, kind, reg, siz, ocode, ore, ext;
        int m = sscanf(line, "%x %d %d %x %x %x %d %d %d %d %d",
                       &pc, &len, &kind, &tp, &to, &top, &reg, &siz, &ocode, &ore, &ext);
        if (m != 11) continue;
        if (pc >= AKEY_MAX) continue;
        n++;
        if (mach[pc].exists) continue;
        mach[pc].exists = 1;
        mach[pc].len = len;   mach[pc].kind = kind;
        mach[pc].tpage = tp;  mach[pc].toff = to;
        mach[pc].top = top;   mach[pc].reg = reg;   mach[pc].siz = siz;
        mach[pc].ocode = ocode; mach[pc].ore = ore; mach[pc].ext = ext;
        mach_total++;
    }
    fclose(f);
    printf("mach(%s): %d lines\n", path, n);
}

// ---------------- trace ----------------
typedef struct {
    unsigned long long cyc;
    int cp, pc;
    int has_regs;
    int sr, r[8], br, dp, ep, tp;
} tl_t;

static tl_t *load_trace(const char *path, int with_regs, long *out_n)
{
    FILE *f = fopen(path, "r");
    if (!f) die("cannot open trace file");
    long cap = 1 << 16, n = 0;
    tl_t *tr = (tl_t *)malloc(cap * sizeof(tl_t));
    char line[512];
    while (fgets(line, sizeof line, f)) {
        unsigned long long cyc;
        unsigned cp, pc;
        int m = sscanf(line, "m %llu %x:%x", &cyc, &cp, &pc);
        if (m != 3) continue;
        tl_t e;
        memset(&e, 0, sizeof e);
        e.cyc = cyc; e.cp = cp & 0xff; e.pc = pc & 0xffff;
        int sr, r0, r1, r2, r3, r4, r5, r6, r7, br, dp, ep, tp;
        int mr = sscanf(line,
            "m %llu %x:%x sr=%x r0=%x r1=%x r2=%x r3=%x r4=%x r5=%x r6=%x r7=%x br=%x dp=%x ep=%x tp=%x",
            &cyc, &cp, &pc, &sr, &r0, &r1, &r2, &r3, &r4, &r5, &r6, &r7, &br, &dp, &ep, &tp);
        if (mr == 16) {
            e.has_regs = 1;
            e.sr = sr & 0xffff;
            e.r[0] = r0 & 0xffff; e.r[1] = r1 & 0xffff;
            e.r[2] = r2 & 0xffff; e.r[3] = r3 & 0xffff;
            e.r[4] = r4 & 0xffff; e.r[5] = r5 & 0xffff;
            e.r[6] = r6 & 0xffff; e.r[7] = r7 & 0xffff;
            e.br = br & 0xff; e.dp = dp & 0xff; e.ep = ep & 0xff; e.tp = tp & 0xff;
        }
        if (with_regs && !e.has_regs) continue;
        if (n == cap) { cap *= 2; tr = (tl_t *)realloc(tr, cap * sizeof(tl_t)); }
        tr[n++] = e;
    }
    fclose(f);
    *out_n = n;
    return tr;
}

// ---------------- rom ----------------
static uint8_t *ROM1, *ROM2;

static int rom_byte(uint32_t addr)
{
    if (((addr >> 16) & 0xffff) == 0 && (addr & 0xffff) < 0x8000)
        return ROM1[addr & 0xffff];
    uint32_t idx = addr & 0x3ffff;
    if (addr & 0x80000) idx |= 0x40000;
    return ROM2[idx & 0x7ffff];
}
static int rom_word(uint32_t addr)
{
    return (rom_byte(addr) << 8) | rom_byte(addr + 1);
}

static uint8_t *load_rom(const char *path, long size)
{
    FILE *f = fopen(path, "rb");
    if (!f) die("cannot open rom");
    uint8_t *b = (uint8_t *)malloc(size);
    if (fread(b, 1, size, f) != (size_t)size) die("short rom read");
    fclose(f);
    return b;
}

// ---------------- machine state ----------------
typedef struct { int r[8]; int sr, br, dp, ep, tp; } st_t;

static void st_from_trace(st_t *s, const tl_t *t)
{
    for (int i = 0; i < 8; i++) s->r[i] = t->r[i];
    s->sr = t->sr; s->br = t->br; s->dp = t->dp; s->ep = t->ep; s->tp = t->tp;
}

// ---------------- helpers (port of semantics.py) ----------------
enum { N = 0x08, Z = 0x04, V = 0x02, C = 0x01 };

static int W(int v) { return v & 0xffff; }
static int B(int v) { return v & 0xff; }
static int sb(int v) { v &= 0xff; return v >= 128 ? v - 256 : v; }
static int sw(int v) { v &= 0xffff; return v >= 0x8000 ? v - 65536 : v; }
static int popcount(int v) { int n = 0; while (v) { n += v & 1; v >>= 1; } return n; }

static int status_common(int val, int siz)
{
    int m = siz ? 0xffff : 0xff;
    val &= m;
    int fl = 0;
    if (val & (siz ? 0x8000 : 0x80)) fl |= N;
    if (val == 0) fl |= Z;
    return fl;
}

static int merge_reg(int rval, int newval, int siz)
{
    return siz ? W(newval) : ((W(rval) & 0xff00) | (W(newval) & 0xff));
}

static int setcommon_sr(int sr, int val, int siz)
{
    return (sr & ~0x0e) | status_common(val, siz);
}

static int before16(int a, int b, int siz, int sign)
{
    int va, vb;
    if (siz) {
        va = a < 0x8000 ? a : a - 0x10000;
        vb = b < 0x8000 ? b : b - 0x10000;
    } else {
        va = a < 0x80 ? a : a - 0x100;
        vb = b < 0x80 ? b : b - 0x100;
    }
    return sign ? va + vb : va - vb;
}

static void flags_add(int t1, int t2, int c_bit, int siz, int *res_out, int *fl_out)
{
    int m = siz ? 0xffff : 0xff;
    t1 &= m; t2 &= m;
    int res = (t1 + t2 + c_bit) & m;
    int sres = before16(t1, t2, siz, 1) + c_bit;
    int ov = siz ? (sres < -0x8000 || sres > 0x7fff) : (sres < -0x80 || sres > 0x7f);
    int nn = siz ? (res & 0x8000) : (res & 0x80);
    int fl = 0;
    if (nn) fl |= N;
    if (res == 0) fl |= Z;
    if ((t1 + t2 + c_bit) > m) fl |= C;
    if (ov) fl |= V;
    *res_out = res; *fl_out = fl;
}

static void flags_sub(int t1, int t2, int c_bit, int siz, int *res_out, int *fl_out)
{
    int m = siz ? 0xffff : 0xff;
    t1 &= m; t2 &= m;
    int res = (t1 - t2 - c_bit) & m;
    int sres = before16(t1, t2, siz, 0) - c_bit;
    int ov = siz ? (sres < -0x8000 || sres > 0x7fff) : (sres < -0x80 || sres > 0x7f);
    int nn = siz ? (res & 0x8000) : (res & 0x80);
    int fl = 0;
    if (nn) fl |= N;
    if (res == 0) fl |= Z;
    if ((t1 - t2 - c_bit) < 0) fl |= C;
    if (ov) fl |= V;
    *res_out = res; *fl_out = fl;
}

static int bcc_taken(int cond, int sr)
{
    int n = sr & N ? 1 : 0, z = sr & Z ? 1 : 0;
    int v = sr & V ? 1 : 0, c = sr & C ? 1 : 0;
    switch (cond) {
    case 0x0: return 1;
    case 0x1: return 0;
    case 0x2: return (c | z) == 0;
    case 0x3: return (c | z) == 1;
    case 0x4: return c == 0;
    case 0x5: return c == 1;
    case 0x6: return z == 0;
    case 0x7: return z == 1;
    case 0x8: return v == 0;
    case 0x9: return v == 1;
    case 0xa: return n == 0;
    case 0xb: return n == 1;
    case 0xc: return (n ^ v) == 0;
    case 0xd: return (n ^ v) == 1;
    case 0xe: return (z | (n ^ v)) == 0;
    case 0xf: return (z | (n ^ v)) == 1;
    }
    return 0;
}

// set entry: mode 0 = unchanged, 1 = expected value, 2 = written/unknown
typedef struct { int mode; int val; } sset_t;

static void vfail(int tag, uint32_t s, const dec_t *d, const st_t *before, const st_t *after,
                  const char *extra)
{
    fail_count++;
    for (int i = 0; i < npat; i++) {
        if (pats[i].tag == tag && pats[i].op == rom_byte(s) && pats[i].top == d->top &&
            pats[i].reg == d->reg && pats[i].siz == d->siz && pats[i].ocode == d->ocode &&
            pats[i].ore == d->ore) {
            pats[i].n++;
            goto printed;
        }
    }
    if (npat < 512) {
        pat_t *p = &pats[npat++];
        p->tag = tag; p->op = rom_byte(s); p->top = d->top; p->reg = d->reg;
        p->siz = d->siz; p->ocode = d->ocode; p->ore = d->ore; p->n = 1;
    }
printed:
    if (fails_printed < 60) {
        char diff[256];
        int o = 0;
        for (int i = 0; i < 8 && o < (int)sizeof(diff) - 32; i++) {
            if (before->r[i] != after->r[i])
                o += snprintf(diff + o, sizeof(diff) - o, "%c:%04x->%04x ",
                              'A' + i, before->r[i], after->r[i]);
        }
        diff[o] = 0;
        printf("        %s pc=%08x op=%02x len=%d kind=%d top=%02x reg=%d siz=%d ocode=%d ore=%d ext=%d "
               "sr=%04x->%04x %s%s\n",
               tag_name(tag), s, rom_byte(s), d->len, d->kind, d->top, d->reg, d->siz,
               d->ocode, d->ore, d->ext, before->sr, after->sr, diff, extra);
        fails_printed++;
    }
}

static void expect_regs(uint32_t s, const dec_t *d, const st_t *before, const st_t *after,
                        sset_t *sets, int has_sr, int sr_exp)
{
    sset_t sd[8];
    memcpy(sd, sets, sizeof sd);
    int top = d->top, areg = d->reg, siz = d->siz;
    if ((top == 0xb0 || top == 0xc0) && sd[areg].mode == 0) {
        int delta = (siz || areg == 7) ? 2 : 1;
        if (top == 0xb0) delta = -delta;
        sd[areg].mode = 1;
        sd[areg].val = W(before->r[areg] + delta);
    }
    for (int i = 0; i < 8; i++) {
        if (sd[i].mode == 1) {
            int exp = W(sd[i].val);
            if (exp != after->r[i]) {
                char extra[64];
                snprintf(extra, sizeof extra, " r%d exp=%04x got=%04x", i, exp, after->r[i]);
                vfail(TAG_REG, s, d, before, after, extra);
            }
        } else if (sd[i].mode == 0) {
            if (before->r[i] != after->r[i]) {
                char extra[64];
                snprintf(extra, sizeof extra, " r%d %04x->%04x", i, before->r[i], after->r[i]);
                vfail(TAG_REGCHG, s, d, before, after, extra);
            }
        }
    }
    if (has_sr) {
        int got = B(after->sr) & 0x0f;
        if (got != (sr_exp & 0x0f)) {
            char extra[64];
            snprintf(extra, sizeof extra, " NVCZ exp=%04x got=%02x", sr_exp, B(after->sr));
            vfail(TAG_SR, s, d, before, after, extra);
        }
    }
}

static void expect_unchanged(uint32_t s, const dec_t *d, const st_t *before, const st_t *after)
{
    sset_t sets[8];
    memset(sets, 0, sizeof sets);
    expect_regs(s, d, before, after, sets, 0, 0);
}

// ---------------- check_short (port) ----------------
static void check_short(uint32_t s, const dec_t *d, const st_t *before, const st_t *after, uint32_t nxt)
{
    int op = rom_byte(s);
    const int *r = before->r;
    int sr = before->sr;
    sset_t sets[8];

    if (op == 0x00) { expect_unchanged(s, d, before, after); return; }

    if (op == 0x01 || op == 0x06 || op == 0x07) {
        int b = rom_byte(s + 1), reg = b & 7, h = b >> 3;
        if (h == 0x17) {
            int disp = sb(rom_byte(s + 2));
            int cond = (op == 0x01) ||
                       ((op == 0x06) && (sr & Z)) ||
                       ((op == 0x07) && !(sr & Z));
            if (cond) {
                int nr = W(r[reg] - 1);
                uint32_t exp_next = (nr != 0xffff) ? ((s + 3 + disp) & 0xffff) : ((s + 3) & 0xffff);
                memset(sets, 0, sizeof sets);
                sets[reg].mode = 1; sets[reg].val = nr;
                expect_regs(s, d, before, after, sets, 0, 0);
                if ((nxt & 0xffff) != (exp_next & 0xffff)) {
                    char extra[64];
                    snprintf(extra, sizeof extra, " exp=%04x got=%04x", exp_next, nxt & 0xffff);
                    vfail(TAG_JMP, s, d, before, after, extra);
                }
            } else {
                expect_unchanged(s, d, before, after);
                if ((nxt & 0xffff) != ((s + 3) & 0xffff)) {
                    char extra[64];
                    snprintf(extra, sizeof extra, " exp fall %04x got=%04x", (unsigned)(s + 3) & 0xffff, nxt & 0xffff);
                    vfail(TAG_JMP, s, d, before, after, extra);
                }
            }
        } else {
            expect_unchanged(s, d, before, after);
            if ((nxt & 0xffff) != ((s + 3) & 0xffff)) {
                char extra[64];
                snprintf(extra, sizeof extra, " trap exp %04x got=%04x", (unsigned)(s + 3) & 0xffff, nxt & 0xffff);
                vfail(TAG_JMP, s, d, before, after, extra);
            }
        }
        return;
    }
    if (op == 0x02) { // LDM
        int rlist = rom_byte(s + 1);
        int n = popcount(rlist);
        memset(sets, 0, sizeof sets);
        for (int i = 0; i < 8; i++)
            if (((rlist >> i) & 1) && i != 7) sets[i].mode = 2;
        sets[7].mode = 1; sets[7].val = r[7] + 2 * n;
        expect_regs(s, d, before, after, sets, 0, 0);
        return;
    }
    if (op == 0x12) { // STM
        int n = popcount(rom_byte(s + 1));
        memset(sets, 0, sizeof sets);
        sets[7].mode = 1; sets[7].val = r[7] - 2 * n;
        expect_regs(s, d, before, after, sets, 0, 0);
        return;
    }
    if (op == 0x03) { // PJSR
        int pg = rom_byte(s + 1), ad = rom_word(s + 2);
        memset(sets, 0, sizeof sets);
        sets[7].mode = 1; sets[7].val = r[7] - 4;
        expect_regs(s, d, before, after, sets, 0, 0);
        if (nxt != (uint32_t)((pg << 16) | ad)) {
            char extra[64];
            snprintf(extra, sizeof extra, " exp=%06x got=%06x", (pg << 16) | ad, nxt);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (op == 0x13) { // PJMP
        int pg = rom_byte(s + 1), ad = rom_word(s + 2);
        expect_unchanged(s, d, before, after);
        if (nxt != (uint32_t)((pg << 16) | ad)) {
            char extra[64];
            snprintf(extra, sizeof extra, " exp=%06x got=%06x", (pg << 16) | ad, nxt);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (op == 0x10) { // JMP
        int ad = rom_word(s + 1);
        expect_unchanged(s, d, before, after);
        if ((nxt & 0xffff) != ad) {
            char extra[64];
            snprintf(extra, sizeof extra, " exp=%04x got=%04x", ad, nxt & 0xffff);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (op == 0x18) { // JSR
        int ad = rom_word(s + 1);
        memset(sets, 0, sizeof sets);
        sets[7].mode = 1; sets[7].val = r[7] - 2;
        expect_regs(s, d, before, after, sets, 0, 0);
        if ((nxt & 0xffff) != ad) {
            char extra[64];
            snprintf(extra, sizeof extra, " exp=%04x got=%04x", ad, nxt & 0xffff);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (op == 0x0e) { // BSR
        int disp = sb(rom_byte(s + 1));
        memset(sets, 0, sizeof sets);
        sets[7].mode = 1; sets[7].val = r[7] - 2;
        expect_regs(s, d, before, after, sets, 0, 0);
        if ((nxt & 0xffff) != ((s + 2 + disp) & 0xffff)) {
            char extra[64];
            snprintf(extra, sizeof extra, " exp=%04x got=%04x", (unsigned)(s + 2 + disp) & 0xffff, nxt & 0xffff);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (op == 0x1e) { // BSR16
        int disp = sw(rom_word(s + 1));
        memset(sets, 0, sizeof sets);
        sets[7].mode = 1; sets[7].val = r[7] - 2;
        expect_regs(s, d, before, after, sets, 0, 0);
        if ((nxt & 0xffff) != ((s + 3 + disp) & 0xffff)) {
            char extra[64];
            snprintf(extra, sizeof extra, " exp=%04x got=%04x", (unsigned)(s + 3 + disp) & 0xffff, nxt & 0xffff);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (op == 0x0a) { // RTE
        memset(sets, 0, sizeof sets);
        sets[7].mode = 1; sets[7].val = r[7] + 6;
        expect_regs(s, d, before, after, sets, 0, 0);
        return;
    }
    if (op == 0x19) { // RTS
        memset(sets, 0, sizeof sets);
        sets[7].mode = 1; sets[7].val = r[7] + 2;
        expect_regs(s, d, before, after, sets, 0, 0);
        return;
    }
    if (op == 0x14) { // RTD
        int imm = sb(rom_byte(s + 1));
        memset(sets, 0, sizeof sets);
        sets[7].mode = 1; sets[7].val = r[7] + 2 + imm;
        expect_regs(s, d, before, after, sets, 0, 0);
        return;
    }
    if (op == 0x1c) { // RTD (trap)
        expect_unchanged(s, d, before, after);
        if ((nxt & 0xffff) != ((s + 2) & 0xffff)) {
            char extra[64];
            snprintf(extra, sizeof extra, " trap exp %04x got=%04x", (unsigned)(s + 2) & 0xffff, nxt & 0xffff);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (op == 0x1a) { // sleep
        expect_unchanged(s, d, before, after);
        return;
    }
    if (op == 0x08) { // TRAPA
        int b = rom_byte(s + 1);
        if ((b & 0xf0) == 0x10) {
            expect_unchanged(s, d, before, after);
        } else {
            expect_unchanged(s, d, before, after);
            if ((nxt & 0xffff) != ((s + 2) & 0xffff)) {
                char extra[64];
                snprintf(extra, sizeof extra, " trap exp %04x got=%04x", (unsigned)(s + 2) & 0xffff, nxt & 0xffff);
                vfail(TAG_JMP, s, d, before, after, extra);
            }
        }
        return;
    }
    if (op == 0x11) { // general jump family
        int b = rom_byte(s + 1), h = b >> 3, ol = b & 7;
        if (b == 0x19) {
            memset(sets, 0, sizeof sets);
            sets[7].mode = 1; sets[7].val = r[7] + 4;
            expect_regs(s, d, before, after, sets, 0, 0);
        } else if (h == 0x19) {
            int rr = ol & ~1;
            memset(sets, 0, sizeof sets);
            sets[7].mode = 1; sets[7].val = r[7] - 4;
            expect_regs(s, d, before, after, sets, 0, 0);
            uint32_t tgt = (uint32_t)((B(r[rr]) << 16) | r[rr + 1]);
            if (nxt != tgt) {
                char extra[64];
                snprintf(extra, sizeof extra, " exp=%06x got=%06x", tgt, nxt);
                vfail(TAG_JMP, s, d, before, after, extra);
            }
        } else if (h == 0x1a) {
            expect_unchanged(s, d, before, after);
            if ((nxt & 0xffff) != (uint32_t)r[ol]) {
                char extra[64];
                snprintf(extra, sizeof extra, " exp=%04x got=%04x", r[ol], nxt & 0xffff);
                vfail(TAG_JMP, s, d, before, after, extra);
            }
        } else if (h == 0x1b) {
            memset(sets, 0, sizeof sets);
            sets[7].mode = 1; sets[7].val = r[7] - 2;
            expect_regs(s, d, before, after, sets, 0, 0);
            if ((nxt & 0xffff) != (uint32_t)r[ol]) {
                char extra[64];
                snprintf(extra, sizeof extra, " exp=%04x got=%04x", r[ol], nxt & 0xffff);
                vfail(TAG_JMP, s, d, before, after, extra);
            }
        } else {
            expect_unchanged(s, d, before, after);
            if ((nxt & 0xffff) != ((s + 2) & 0xffff)) {
                char extra[64];
                snprintf(extra, sizeof extra, " trap exp %04x got=%04x", (unsigned)(s + 2) & 0xffff, nxt & 0xffff);
                vfail(TAG_JMP, s, d, before, after, extra);
            }
        }
        return;
    }
    if (op >= 0x20 && op <= 0x3f) { // BCC
        int c = op & 0xf;
        int disp = (op & 0x10) ? sw(rom_word(s + 1)) : sb(rom_byte(s + 1));
        int ln = d->len;
        int taken = bcc_taken(c, sr);
        uint32_t exp_next = taken ? ((s + ln + disp) & 0xffff) : ((s + ln) & 0xffff);
        expect_unchanged(s, d, before, after);
        if ((nxt & 0xffff) != exp_next) {
            char extra[80];
            snprintf(extra, sizeof extra, " cond=%x taken=%d exp=%04x got=%04x", c, taken, exp_next, nxt & 0xffff);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (op >= 0x40 && op <= 0x4f) { // CMP
        int reg = op & 7, siz = (op & 0x8) ? 1 : 0;
        int t2 = siz ? rom_word(s + 1) : rom_byte(s + 1);
        int res, fl;
        flags_sub(r[reg], t2, 0, siz, &res, &fl);
        memset(sets, 0, sizeof sets);
        expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
        return;
    }
    if (op >= 0x50 && op <= 0x57) { // MOVE
        int reg = op & 7, imm = rom_byte(s + 1);
        int nr = (r[reg] & 0xff00) | imm;
        memset(sets, 0, sizeof sets);
        sets[reg].mode = 1; sets[reg].val = nr;
        expect_regs(s, d, before, after, sets, 1, (sr & ~0x0e) | status_common(imm, 0));
        return;
    }
    if (op >= 0x58 && op <= 0x5f) { // MOVI
        int reg = op & 7, imm = rom_word(s + 1);
        memset(sets, 0, sizeof sets);
        sets[reg].mode = 1; sets[reg].val = imm;
        expect_regs(s, d, before, after, sets, 1, (sr & ~0x0e) | status_common(imm, 1));
        return;
    }
    if (op >= 0x60 && op <= 0x6f) { // MOVL
        return;
    }
    if (op >= 0x70 && op <= 0x7f) { // MOVS
        int reg = op & 7, siz = (op & 0x8) ? 1 : 0;
        int data = siz ? r[reg] : B(r[reg]);
        expect_unchanged(s, d, before, after);
        int fl = (B(sr) & 0x01) | status_common(data, siz);
        if ((B(after->sr) & 0x0f) != fl) {
            char extra[64];
            snprintf(extra, sizeof extra, " exp=%04x got=%02x", fl, B(after->sr));
            vfail(TAG_SR, s, d, before, after, extra);
        }
        return;
    }
    if (op >= 0x80 && op <= 0x9f) { // MOVF
        int reg = op & 7, siz = (op & 0x8) ? 1 : 0;
        if ((op & 0x10) == 0) return; // mem->reg, unknown
        int data, fl;
        if (siz) { data = B(r[reg]); fl = status_common(data, 0); }
        else     { data = r[reg];     fl = status_common(data, 1); }
        expect_unchanged(s, d, before, after);
        if ((B(after->sr) & 0x0f) != fl) {
            char extra[64];
            snprintf(extra, sizeof extra, " exp=%04x got=%02x", fl, B(after->sr));
            vfail(TAG_SR, s, d, before, after, extra);
        }
        return;
    }
}

// GT MCU_ControlRegisterRead (src/mcu.h:274)
static int cr_read(int reg, int siz, const st_t *x, int *ok)
{
    uint32_t ret = 0;
    *ok = 1;
    if (siz) {
        if (reg == 0) ret = x->sr & 0x870f;
        else if (reg == 5) ret = x->dp | (x->dp << 8);
        else if (reg == 4) ret = x->ep | (x->ep << 8);
        else if (reg == 3) ret = x->br | (x->br << 8);
        else *ok = 0;
        ret &= 0xffff;
    } else {
        if (reg == 1) ret = x->sr & 0x870f;
        else if (reg == 3) ret = x->br;
        else if (reg == 4) ret = x->ep;
        else if (reg == 5) ret = x->dp;
        else if (reg == 7) ret = x->tp;
        else *ok = 0;
        ret &= 0xff;
    }
    return ret;
}

// ---------------- check_general (port) ----------------
static void check_general(uint32_t s, const dec_t *d, const st_t *before, const st_t *after, uint32_t nxt)
{
    int top = d->top, reg = d->reg, siz = d->siz ? 1 : 0;
    int ocode = d->ocode, ore = d->ore;
    const int *r = before->r;
    int sr = before->sr;
    sset_t sets[8];
    int imm_operand = (top == 0x00 && reg == 4);
    int imm = imm_operand ? (siz ? rom_word(s + 1) : rom_byte(s + 1)) : 0;
    int direct = (top == 0xa0);
    int res, fl;

    if (ocode == 0) { // MOVG_Immediate: mem only / traps
        expect_unchanged(s, d, before, after);
        return;
    }
    if (ocode == 1) { // ADDQ
        if (!(ore == 0 || ore == 1 || ore == 4 || ore == 5)) {
            expect_unchanged(s, d, before, after);
            if ((nxt & 0xffff) != ((s + d->len) & 0xffff)) {
                char extra[64];
                snprintf(extra, sizeof extra, " trap exp s+len got=%04x", nxt & 0xffff);
                vfail(TAG_JMP, s, d, before, after, extra);
            }
            return;
        }
        int t2 = (ore == 0) ? 1 : (ore == 1) ? 2 : (ore == 4) ? -1 : -2;
        if (direct) {
            flags_add(r[reg], t2, 0, siz, &res, &fl);
            memset(sets, 0, sizeof sets);
            sets[reg].mode = 1; sets[reg].val = merge_reg(r[reg], res, siz);
            expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
        } else {
            expect_unchanged(s, d, before, after);
        }
        return;
    }
    if (ocode == 2) { // CLR family
        if (direct) {
            int data = r[reg];
            if (ore == 3) { // CLR
                memset(sets, 0, sizeof sets);
                sets[reg].mode = 1; sets[reg].val = merge_reg(data, 0, siz);
                expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | Z);
            } else if (ore == 6) { // TST
                expect_unchanged(s, d, before, after);
                if ((B(after->sr) & 0x0f) != (status_common(data, siz) | 0)) {
                    char extra[64];
                    snprintf(extra, sizeof extra, " exp=%04x got=%02x", status_common(data, siz), B(after->sr));
                    vfail(TAG_SR, s, d, before, after, extra);
                }
            } else if (ore == 2 && !siz) { // EXTU
                int nr = B(data);
                memset(sets, 0, sizeof sets);
                sets[reg].mode = 1; sets[reg].val = nr;
                expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | (nr == 0 ? Z : 0));
            } else if (ore == 0 && !siz) { // SWAP
                int nr = (B(data) << 8) | (data >> 8);
                memset(sets, 0, sizeof sets);
                sets[reg].mode = 1; sets[reg].val = nr;
                expect_regs(s, d, before, after, sets, 1, setcommon_sr(sr, nr, 1));
            } else if (ore == 5) { // NOT
                int m = siz ? 0xffff : 0xff;
                int nv = W(~data & m);
                memset(sets, 0, sizeof sets);
                sets[reg].mode = 1; sets[reg].val = merge_reg(data, nv, siz);
                expect_regs(s, d, before, after, sets, 1, setcommon_sr(sr, nv, siz));
            } else if (ore == 4) { // NEG
                flags_sub(0, data, 0, siz, &res, &fl);
                memset(sets, 0, sizeof sets);
                sets[reg].mode = 1; sets[reg].val = merge_reg(data, res, siz);
                expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
            } else if (ore == 1 && !siz) { // EXTS
                int nr = W(sb(B(data)) & 0xffff);
                memset(sets, 0, sizeof sets);
                sets[reg].mode = 1; sets[reg].val = nr;
                expect_regs(s, d, before, after, sets, 1, setcommon_sr(sr, nr, 1));
            } else { // trap
                expect_unchanged(s, d, before, after);
                if ((nxt & 0xffff) != ((s + d->len) & 0xffff)) {
                    char extra[64];
                    snprintf(extra, sizeof extra, " trap exp s+len got=%04x", nxt & 0xffff);
                    vfail(TAG_JMP, s, d, before, after, extra);
                }
            }
        } else {
            expect_unchanged(s, d, before, after);
        }
        return;
    }
    if (ocode == 3) { // SHLR family
        if (!direct) { expect_unchanged(s, d, before, after); return; }
        int data = r[reg];
        int c_in = (sr & C) ? 1 : 0;
        int m = siz ? 0xffff : 0xff;
        int dv = data & m;
        int c_out = 0, nv = 0, known = 1;
        if (ore == 3) { c_out = dv & 1; nv = W(dv >> 1); }
        else if (ore == 2) { c_out = (dv & (siz ? 0x8000 : 0x80)) ? 1 : 0; nv = W(dv << 1); }
        else if (ore == 6) { c_out = (dv & (siz ? 0x8000 : 0x80)) ? 1 : 0; nv = W((dv << 1) | c_in); }
        else if (ore == 4) { int c = (dv & (siz ? 0x8000 : 0x80)) ? 1 : 0; nv = W((dv << 1) | c); c_out = c; }
        else if (ore == 0) { c_out = (dv & (siz ? 0x8000 : 0x80)) ? 1 : 0; nv = W(dv << 1); }
        else if (ore == 1) {
            c_out = dv & 1;
            int msb = dv & (siz ? 0x8000 : 0x80);
            nv = W(((dv & m) >> 1) | msb);
        }
        else if (ore == 5) { c_out = dv & 1; nv = W((dv >> 1) | (c_out << (siz ? 15 : 7))); }
        else { known = 0; }
        if (!known) {
            expect_unchanged(s, d, before, after);
            if ((nxt & 0xffff) != ((s + d->len) & 0xffff)) {
                char extra[64];
                snprintf(extra, sizeof extra, " trap exp s+len got=%04x", nxt & 0xffff);
                vfail(TAG_JMP, s, d, before, after, extra);
            }
            return;
        }
        nv &= m;
        int nr = merge_reg(data, nv, siz);
        fl = status_common(nv, siz);
        if (c_out) fl |= C; else fl &= ~C;
        memset(sets, 0, sizeof sets);
        sets[reg].mode = 1; sets[reg].val = nr;
        expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
        return;
    }
    if (ocode == 4 || ocode == 5 || ocode == 6 || ocode == 7 ||
        ocode == 8 || ocode == 10 || ocode == 12) {
        int m = siz ? 0xffff : 0xff;
        if (ocode == 5 || ocode == 7) { // ADDS/SUBS
            int t2;
            if (direct) {
                t2 = siz ? r[reg] : sb(r[reg]);
            } else if (imm_operand) {
                t2 = siz ? imm : sb(imm);
            } else {
                memset(sets, 0, sizeof sets);
                sets[ore].mode = 2;
                expect_regs(s, d, before, after, sets, 0, 0);
                return;
            }
            int rr = (ocode == 5) ? W(r[ore] + t2) : W(r[ore] - t2);
            memset(sets, 0, sizeof sets);
            sets[ore].mode = 1; sets[ore].val = rr;
            expect_regs(s, d, before, after, sets, 0, 0);
            return;
        }
        int t2;
        if (direct) t2 = r[reg] & m;
        else if (imm_operand) t2 = imm & m;
        else {
            memset(sets, 0, sizeof sets);
            sets[ore].mode = 2;
            expect_regs(s, d, before, after, sets, 0, 0);
            return;
        }
        int nr, sr_exp;
        if (ocode == 4) {
            flags_add(r[ore], t2, 0, siz, &res, &fl);
            nr = merge_reg(r[ore], res, siz);
            sr_exp = (sr & ~0x0f) | fl;
        } else if (ocode == 6) {
            flags_sub(r[ore], t2, 0, siz, &res, &fl);
            nr = merge_reg(r[ore], res, siz);
            sr_exp = (sr & ~0x0f) | fl;
        } else {
            int rv;
            if (ocode == 8) rv = r[ore] | t2;
            else if (ocode == 10) rv = r[ore] & t2;
            else rv = r[ore] ^ t2;
            res = W(rv) & m;
            nr = merge_reg(r[ore], res, siz);
            sr_exp = setcommon_sr(sr, res, siz);
        }
        memset(sets, 0, sizeof sets);
        sets[ore].mode = 1; sets[ore].val = nr;
        expect_regs(s, d, before, after, sets, 1, sr_exp);
        return;
    }
    if (ocode == 9 || ocode == 11) {
        // GT MCU_Opcode_BSET_ORC (mcu_opcodes.cpp:875) / BCLR_ANDC (:899):
        // immediate operand -> ORC/ANDC on control register ore; otherwise
        // BSET/BCLR with bit = r[ore]&0xf.
        if (imm_operand) {
            int ok = 0;
            int cr = cr_read(ore, siz, before, &ok);
            if (!ok) { expect_unchanged(s, d, before, after); return; }
            int val = (ocode == 9) ? (cr | imm) : (cr & imm);
            if (siz) {
                if (ore == 0) { // sr word
                    expect_unchanged(s, d, before, after);
                    int exp = val & 0x870f;
                    if (after->sr != exp) {
                        char extra[64];
                        snprintf(extra, sizeof extra, " orc sr exp=%04x got=%04x", exp, after->sr);
                        vfail(TAG_SR, s, d, before, after, extra);
                    }
                } else if (ore == 3 || ore == 4 || ore == 5) { // br/ep/dp word
                    expect_unchanged(s, d, before, after);
                    int got = (ore == 3) ? after->br : (ore == 4) ? after->ep : after->dp;
                    int exp = val & 0xff;
                    if (got != exp) {
                        char extra[64];
                        snprintf(extra, sizeof extra, " orc cr%d exp=%02x got=%02x", ore, exp, got);
                        vfail(TAG_CR, s, d, before, after, extra);
                    }
                    int se = setcommon_sr(sr, val, siz);
                    if ((B(after->sr) & 0x0f) != (se & 0x0f)) {
                        char extra[64];
                        snprintf(extra, sizeof extra, " orc NVCZ exp=%04x got=%02x", se, B(after->sr));
                        vfail(TAG_SR, s, d, before, after, extra);
                    }
                } else { // ore 1/2/6/7 word: GT trap
                    expect_unchanged(s, d, before, after);
                }
            } else {
                if (ore == 1) { // sr low byte
                    expect_unchanged(s, d, before, after);
                    int exp = ((sr & ~0xff) | (val & 0xff)) & 0x870f;
                    if (after->sr != exp) {
                        char extra[64];
                        snprintf(extra, sizeof extra, " orc sr exp=%04x got=%04x", exp, after->sr);
                        vfail(TAG_SR, s, d, before, after, extra);
                    }
                } else if (ore == 3 || ore == 4 || ore == 5 || ore == 7) {
                    expect_unchanged(s, d, before, after);
                    int got = (ore == 3) ? after->br : (ore == 4) ? after->ep : (ore == 5) ? after->dp : after->tp;
                    int exp = val & 0xff;
                    if (got != exp) {
                        char extra[64];
                        snprintf(extra, sizeof extra, " orc cr%d exp=%02x got=%02x", ore, exp, got);
                        vfail(TAG_CR, s, d, before, after, extra);
                    }
                    int se = setcommon_sr(sr, val, siz);
                    if ((B(after->sr) & 0x0f) != (se & 0x0f)) {
                        char extra[64];
                        snprintf(extra, sizeof extra, " orc NVCZ exp=%04x got=%02x", se, B(after->sr));
                        vfail(TAG_SR, s, d, before, after, extra);
                    }
                } else { // ore 0/2/6 byte: GT trap
                    expect_unchanged(s, d, before, after);
                }
            }
        } else {
            int bit = r[ore] & 0x0f;
            if (direct) {
                int data = siz ? r[reg] : B(r[reg]);
                int zclear = ((data >> bit) & 1) == 0;
                int nd = (ocode == 9) ? (data | (1 << bit)) : (data & ~(1 << bit));
                memset(sets, 0, sizeof sets);
                sets[reg].mode = 1; sets[reg].val = merge_reg(r[reg], nd, siz);
                expect_regs(s, d, before, after, sets, 1, (sr & ~Z) | (zclear ? Z : 0));
            } else {
                expect_unchanged(s, d, before, after); // memory operand: Z unknown
            }
        }
        return;
    }
    if (ocode == 13) {
        expect_unchanged(s, d, before, after);
        if ((nxt & 0xffff) != ((s + d->len) & 0xffff)) {
            char extra[64];
            snprintf(extra, sizeof extra, " trap exp s+len got=%04x", nxt & 0xffff);
            vfail(TAG_JMP, s, d, before, after, extra);
        }
        return;
    }
    if (ocode == 14) { // CMP
        int known = 1, t2 = 0;
        if (direct) t2 = r[reg];
        else if (imm_operand) t2 = siz ? imm : (imm & 0xff);
        else known = 0;
        if (known) {
            flags_sub(r[ore], t2, 0, siz, &res, &fl);
            memset(sets, 0, sizeof sets);
            expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
        } else {
            expect_unchanged(s, d, before, after);
        }
        return;
    }
    if (ocode == 15) { // BTST
        expect_unchanged(s, d, before, after);
        return;
    }
    if (ocode == 16) { // MOVG read
        if (direct) {
            int nr = merge_reg(r[ore], r[reg], siz);
            memset(sets, 0, sizeof sets);
            sets[ore].mode = 1; sets[ore].val = nr;
            expect_regs(s, d, before, after, sets, 1, setcommon_sr(sr, r[reg], siz));
        } else if (imm_operand) {
            int nr = merge_reg(r[ore], imm, siz);
            memset(sets, 0, sizeof sets);
            sets[ore].mode = 1; sets[ore].val = nr;
            expect_regs(s, d, before, after, sets, 1, setcommon_sr(sr, imm, siz));
        } else {
            memset(sets, 0, sizeof sets);
            sets[ore].mode = 2;
            expect_regs(s, d, before, after, sets, 0, 0);
        }
        return;
    }
    if (ocode == 17) { // LDC
        if (imm_operand) {
            expect_unchanged(s, d, before, after);
            if (ore == 0) {
                int exp_sr = W(imm & 0x870f);
                if (after->sr != exp_sr) {
                    char extra[64];
                    snprintf(extra, sizeof extra, " ldc exp=%04x got=%04x", exp_sr, after->sr);
                    vfail(TAG_SR, s, d, before, after, extra);
                }
            } else if (ore == 3 || ore == 4 || ore == 5 || ore == 7) {
                int got = (ore == 3) ? after->br : (ore == 4) ? after->ep : (ore == 5) ? after->dp : after->tp;
                if (got != B(imm)) {
                    char extra[64];
                    snprintf(extra, sizeof extra, " cr%d exp=%02x got=%02x", ore, B(imm), got);
                    vfail(TAG_CR, s, d, before, after, extra);
                }
            }
        } else {
            expect_unchanged(s, d, before, after);
        }
        return;
    }
    if (ocode == 18) { // MOVG write / XCH
        if (direct) {
            if (siz) {
                memset(sets, 0, sizeof sets);
                sets[ore].mode = 1; sets[ore].val = r[reg];
                sets[reg].mode = 1; sets[reg].val = r[ore];
                expect_regs(s, d, before, after, sets, 0, 0);
            } else {
                expect_unchanged(s, d, before, after);
                if ((nxt & 0xffff) != ((s + d->len) & 0xffff)) {
                    char extra[64];
                    snprintf(extra, sizeof extra, " trap exp s+len got=%04x", nxt & 0xffff);
                    vfail(TAG_JMP, s, d, before, after, extra);
                }
            }
        } else {
            memset(sets, 0, sizeof sets);
            expect_regs(s, d, before, after, sets, 1, setcommon_sr(sr, r[ore], siz));
        }
        return;
    }
    if (ocode == 19) { // STC
        expect_unchanged(s, d, before, after);
        return;
    }
    if (ocode == 20) { // ADDX
        if (direct) {
            int c_bit = (sr & C) ? 1 : 0;
            flags_add(r[ore], r[reg], c_bit, siz, &res, &fl);
            if (!(sr & Z) || res != 0) fl &= ~Z;
            memset(sets, 0, sizeof sets);
            sets[ore].mode = 1; sets[ore].val = res;
            expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
        } else {
            expect_unchanged(s, d, before, after);
        }
        return;
    }
    if (ocode == 21) { // MULXU
        if (direct) {
            int t1 = r[reg], t2 = r[ore];
            memset(sets, 0, sizeof sets);
            int nn, zz;
            if (siz) {
                int prod = W(t1) * W(t2);
                int rr = ore & ~1;
                sets[rr].mode = 1; sets[rr].val = W(prod >> 16);
                sets[rr | 1].mode = 1; sets[rr | 1].val = W(prod);
                nn = (prod & 0x80000000) ? 1 : 0;
                zz = (prod == 0) ? 1 : 0;
            } else {
                int prod = B(t1) * B(t2);
                sets[ore].mode = 1; sets[ore].val = W(prod);
                nn = (prod & 0x8000) ? 1 : 0;
                zz = (prod == 0) ? 1 : 0;
            }
            fl = (nn ? N : 0) | (zz ? Z : 0);
            expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
        }
        return;
    }
    if (ocode == 22) { // SUBX
        if (direct) {
            int c_bit = (sr & C) ? 1 : 0;
            flags_sub(r[ore], r[reg], c_bit, siz, &res, &fl);
            memset(sets, 0, sizeof sets);
            sets[ore].mode = 1; sets[ore].val = res;
            expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
        } else {
            expect_unchanged(s, d, before, after);
        }
        return;
    }
    if (ocode == 23) { // DIVXU
        if (direct) {
            int t1 = r[reg], t2 = r[ore];
            int t1w, t2w;
            memset(sets, 0, sizeof sets);
            if (siz) {
                int rr = ore & ~1;
                t1w = W(t1);
                t2w = (r[rr] << 16) | r[rr | 1];
                if (t1w == 0) { expect_regs(s, d, before, after, sets, 0, 0); return; }
                int R = t2w % t1w, Q = t2w / t1w;
                if (Q > 0xffff) {
                    expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | V);
                    return;
                }
                sets[rr].mode = 1; sets[rr].val = W(R);
                sets[rr | 1].mode = 1; sets[rr | 1].val = W(Q);
                fl = status_common(Q, 1);
            } else {
                t1w = B(t1); t2w = B(t2);
                if (t1w == 0) { expect_regs(s, d, before, after, sets, 0, 0); return; }
                int R = t2w % t1w, Q = t2w / t1w;
                if (Q > 0xff) {
                    expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | V);
                    return;
                }
                sets[ore].mode = 1; sets[ore].val = W((R << 8) | Q);
                fl = status_common(Q, 0);
            }
            expect_regs(s, d, before, after, sets, 1, (sr & ~0x0f) | fl);
        }
        return;
    }
    if (ocode >= 24 && ocode <= 31) {
        // GT BSET/BCLR/BNOTI/BTSTI (mcu_opcodes.cpp:1009-1125): bit number =
        // ore | ((ocode&1)<<3); only Z is touched (Z=1 when the old bit was 0).
        if (imm_operand) { // GT trap
            expect_unchanged(s, d, before, after);
            return;
        }
        int bit = ore | ((ocode & 1) << 3);
        if (direct) {
            int data = siz ? r[reg] : B(r[reg]);
            int zclear = ((data >> bit) & 1) == 0;
            int nd = data;
            if (ocode == 24 || ocode == 25)      nd = data | (1 << bit);      // BSET
            else if (ocode == 26 || ocode == 27) nd = data & ~(1 << bit);     // BCLR
            else if (ocode == 28 || ocode == 29) nd = data ^ (1 << bit);      // BNOTI
            memset(sets, 0, sizeof sets);
            if (ocode < 30) { // BTSTI has no write
                sets[reg].mode = 1;
                sets[reg].val = merge_reg(r[reg], nd, siz);
            }
            expect_regs(s, d, before, after, sets, 1, (sr & ~Z) | (zclear ? Z : 0));
        } else {
            expect_unchanged(s, d, before, after); // memory operand: Z unknown
        }
        return;
    }
}

// ---------------- vector next set ----------------
static uint8_t *build_vec_next(void)
{
    uint8_t *out = (uint8_t *)calloc(AKEY_MAX, 1);
    for (int v = 0; v < 64; v++) {
        int b[4];
        for (int k = 0; k < 4; k++) b[k] = rom_byte((uint32_t)(v * 4 + k));
        uint32_t v32 = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
        uint32_t addr = (((v32 >> 16) & 0xff) << 16) | (v32 & 0xffff);
        if (addr >= AKEY_MAX || !mach[addr].exists) continue;
        uint32_t vn = ((addr >> 16) << 16) | (((addr & 0xffff) + mach[addr].len) & 0xffff);
        if (vn < AKEY_MAX) out[vn] = 1;
        int kind = mach[addr].kind;
        if (kind >= 1 && kind <= 5) {
            uint32_t t = ((uint32_t)mach[addr].tpage << 16) | (uint32_t)mach[addr].toff;
            if (t < AKEY_MAX) out[t] = 1;
        }
    }
    return out;
}

// ---------------- Phase A ----------------
typedef struct { uint32_t s; dec_t d; uint32_t u; } bad_t;
static bad_t bad_len[32], bad_tgt[32], unexp[32];
static int n_bad_len = 0, n_bad_tgt = 0, n_unexp = 0;
static long n_bad_len_all = 0, n_bad_tgt_all = 0, n_unexp_all = 0;

static void report(const char *name, bad_t *items, int n, long total)
{
    if (total == 0) { printf("[OK]   %s: 0\n", name); return; }
    printf("[FAIL] %s: %ld\n", name, total);
    for (int i = 0; i < n; i++) {
        const dec_t *d = &items[i].d;
        printf("        %08x len=%d kind=%d tgt=%02x:%04x top=%02x reg=%d siz=%d ocode=%d ore=%d ext=%d | %08x\n",
               items[i].s, d->len, d->kind, d->tpage, d->toff, d->top, d->reg, d->siz,
               d->ocode, d->ore, d->ext, items[i].u);
    }
}

static long phaseA(const tl_t *tr, long n, const uint8_t *vecnext)
{
    long missing = 0, sleeps = 0, interrupts = 0;
    for (long i = 0; i < n - 1; i++) {
        uint32_t s = (uint32_t)((tr[i].cp << 16) | tr[i].pc);
        uint32_t u = (uint32_t)((tr[i + 1].cp << 16) | tr[i + 1].pc);
        if (s >= AKEY_MAX || !mach[s].exists) { missing++; continue; }
        const dec_t *d = &mach[s];
        if (u == s) { sleeps++; continue; }
        uint32_t tgt = ((uint32_t)d->tpage << 16) | (uint32_t)d->toff;
        uint32_t seq = ((uint32_t)tr[i].cp << 16) | (((uint32_t)tr[i].pc + d->len) & 0xffff);
        if (u == seq || u == tgt) continue;
        if (u < AKEY_MAX && vecnext[u]) { interrupts++; continue; }
        if (d->kind == 3) {
            n_bad_len_all++;
            if (n_bad_len < 32) { bad_len[n_bad_len].s = s; bad_len[n_bad_len].d = *d; bad_len[n_bad_len].u = u; n_bad_len++; }
        } else if (d->kind == 1 || d->kind == 2) {
            n_bad_tgt_all++;
            if (n_bad_tgt < 32) { bad_tgt[n_bad_tgt].s = s; bad_tgt[n_bad_tgt].d = *d; bad_tgt[n_bad_tgt].u = u; n_bad_tgt++; }
        } else if (d->kind == 0) {
            n_unexp_all++;
            if (n_unexp < 32) { unexp[n_unexp].s = s; unexp[n_unexp].d = *d; unexp[n_unexp].u = u; n_unexp++; }
        }
    }
    printf("[INFO] trace transitions from PCs missing in mach set: %ld\n", missing);
    printf("[INFO] sleep no-op transitions: %ld\n", sleeps);
    printf("[INFO] interrupt/exception dispatch transitions: %ld\n", interrupts);
    report("BAD_FALLTHRU_OR_LEN (cond: next is neither s+len nor target; none: next != s+len)", bad_len, n_bad_len, n_bad_len_all);
    report("BAD_BRANCH_TARGET (uncond/call: observed dst != decoded target)", bad_tgt, n_bad_tgt, n_bad_tgt_all);
    report("UNEXPECTED_JMP from kind=0 instr (int/trap/undecoded transfer?)", unexp, n_unexp, n_unexp_all);
    return n_bad_len_all + n_bad_tgt_all + n_unexp_all;
}

// ---------------- Phase B ----------------
static long phaseB(const tl_t *rt, long rn, const uint8_t *vecnext)
{
    long sleeps = 0, interrupts = 0, checked = 0;
    for (long i = 0; i < rn - 1; i++) {
        uint32_t s = (uint32_t)((rt[i].cp << 16) | rt[i].pc);
        uint32_t u = (uint32_t)((rt[i + 1].cp << 16) | rt[i + 1].pc);
        if (s >= AKEY_MAX || !mach[s].exists) continue;
        const dec_t *d = &mach[s];
        uint32_t tgt = ((uint32_t)d->tpage << 16) | (uint32_t)d->toff;
        uint32_t seq = ((uint32_t)rt[i].cp << 16) | (((uint32_t)rt[i].pc + d->len) & 0xffff);
        if (u == s && u != tgt) { sleeps++; continue; }
        int executed;
        if (d->kind == 4 || d->kind == 5) executed = !(u < AKEY_MAX && vecnext[u]);
        else executed = (u == seq) || (u == tgt);
        if (!executed) { interrupts++; continue; }
        st_t before, after;
        st_from_trace(&before, &rt[i]);
        st_from_trace(&after, &rt[i + 1]);
        int op = rom_byte(s);
        if (op == 0x04 || op == 0x05 || op == 0x0c || op == 0x0d || op == 0x15 || op == 0x1d || op >= 0xa0)
            check_general(s, d, &before, &after, u);
        else
            check_short(s, d, &before, &after, u);
        checked++;
    }
    if (sleeps) printf("[INFO] sleeping transitions skipped: %ld\n", sleeps);
    if (interrupts) printf("[INFO] interrupt-dispatch transitions skipped: %ld\n", interrupts);
    printf("[INFO] semantic instructions checked: %ld\n", checked);
    if (fail_count) {
        printf("[FAIL] semantic failures: %d\n", fail_count);
        printf("  by (tag,op,top,reg,siz,ocode,ore):\n");
        for (int i = 0; i < npat; i++) {
            printf("    %-8s op=%02x top=%02x reg=%d siz=%d ocode=%2d ore=%d : %d\n",
                   tag_name(pats[i].tag), pats[i].op, pats[i].top, pats[i].reg,
                   pats[i].siz, pats[i].ocode, pats[i].ore, pats[i].n);
        }
    } else {
        printf("[OK]   semantic failures: 0\n");
    }
    return fail_count;
}

// ---------------- main ----------------
int main(int argc, char **argv)
{
    const char *mach_ps[8], *trace_p = NULL, *reg_p = NULL, *rom1_p = NULL, *rom2_p = NULL;
    int n_mach = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--mach") && i + 1 < argc && n_mach < 8) mach_ps[n_mach++] = argv[++i];
        else if (!strcmp(argv[i], "--trace") && i + 1 < argc) trace_p = argv[++i];
        else if (!strcmp(argv[i], "--regtrace") && i + 1 < argc) reg_p = argv[++i];
        else if (!strcmp(argv[i], "--rom1") && i + 1 < argc) rom1_p = argv[++i];
        else if (!strcmp(argv[i], "--rom2") && i + 1 < argc) rom2_p = argv[++i];
        else die("unknown arg");
    }
    if (!n_mach || !rom1_p || !rom2_p) die("usage: verify_dasm --mach mach.txt [--mach mach2.txt] [--trace t] [--regtrace r] --rom1 rom1 --rom2 rom2");

    for (int i = 0; i < n_mach; i++) load_mach(mach_ps[i]);
    printf("mach total: %d decoded PCs\n", mach_total);
    ROM1 = load_rom(rom1_p, 0x8000);
    ROM2 = load_rom(rom2_p, 0x80000);

    long total = 0;
    uint8_t *vecnext = build_vec_next();

    if (trace_p) {
        long n = 0;
        tl_t *tr = load_trace(trace_p, 0, &n);
        printf("trace: %ld main instr\n", n);
        total += phaseA(tr, n, vecnext);
        free(tr);
    }
    if (reg_p) {
        long n = 0;
        tl_t *rt = load_trace(reg_p, 1, &n);
        printf("regtrace: %ld main instr with regs\n", n);
        total += phaseB(rt, n, vecnext);
        free(rt);
    }
    printf("\nTOTAL FAILURES: %ld\n", total);
    return total ? 1 : 0;
}
