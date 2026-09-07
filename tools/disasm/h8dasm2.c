// Ground-truth H8/55 disassembler for Nuked-SC55 main MCU.
// Driven by executed-PC set + observed transition edges from the original emulator's
// PC-change log. Emits every executed linear run with observed branch annotations.
//
// Memory map (verified against src/mcu.cpp MCU_Read):
//   page 0, off < 0x8000        -> rom1[off]
//   else: idx = addr & 0x3ffff; if (addr & 0x80000) idx |= 0x40000; -> rom2[idx]
//
// Build:  clang -O2 -o h8dasm2.exe h8dasm2.c
// Usage:  h8dasm2.exe rom1.bin rom2.bin pc_main.txt flow_main.txt out.txt
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

static unsigned char *ROM1, *ROM2;
static const int ROM1_SIZE = 0x8000;
static const int ROM2_SIZE = 0x80000;

static unsigned char rbyte(int page, int off) {
    int addr = ((page & 0xf) << 16) | (off & 0xffff);
    int pg = (addr >> 16) & 0xf, o = addr & 0xffff;
    if (pg == 0 && o < ROM1_SIZE) return ROM1[o];
    int idx = addr & 0x3ffff;
    if (addr & 0x80000) idx |= 0x40000;
    return ROM2[idx & (ROM2_SIZE - 1)];
}
static int rword(int page, int off) { return (rbyte(page, off) << 8) | rbyte(page, off + 1); }
static int rword_s(int page, int off) { int v = rword(page, off); return (short)v; }
static int rbyte_s(int page, int off) { int v = rbyte(page, off); return (v >= 128) ? v - 256 : v; }

typedef struct {
    int len;
    int has_target, tpage, toff;
    int is_call, is_ret, is_uncond;
    char text[120];
} dec_t;

static const char *BCC_NAME[16] = {"BRA","BRN","BHI","BLS","BCC","BCS","BNE","BEQ",
                                   "BVC","BVS","BPL","BMI","BGE","BLT","BGT","BLE"};
static const char *OPC_NAME[] = {"MOVG","ADDQ","CLR","SHLR","ADD","ADDS","SUB","SUBS",
    "OR","BSET_ORC","AND","BCLR_ANDC","XOR","??","CMP","BTST","MOVG2","LDC","MOVG3","STC",
    "ADDX","MULXU","SUBX","DIVXU","BSET","BSET","BCLR","BCLR","BNOTI","BNOTI","BTSTI","BTSTI"};

static void fmt(dec_t *d, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); vsnprintf(d->text, sizeof d->text, fmt, ap); va_end(ap);
}

static void decode(int page, int off, dec_t *d) {
    int o = off;
    d->len = 1; d->has_target = 0; d->is_call = 0; d->is_ret = 0; d->is_uncond = 0;
    #define FINISH do { d->len = o - off; return; } while(0)
    int op = rbyte(page, off); o++;
    #define ADV(n) do { o += (n); } while(0)
    #define B1 (rbyte(page, o-1))
    #define W2 (rword(page, o-2))
    #define S1 (rbyte_s(page, o-1))
    #define SW2 (rword_s(page, o-2))

    if (op == 0x00) { fmt(d, "nop"); FINISH; }
    if (op == 0x01 || op == 0x06 || op == 0x07 || op == 0x11) { ADV(1); int b=B1;
        fmt(d, (b & 0xf0)==0x17 && (b&7) ? "jmp-reg r%d" : "jmp r%d", b & 7); FINISH; }
    if (op == 0x10) { ADV(2); fmt(d,"jmp #0x%04x",W2); d->has_target=1;d->tpage=page;d->toff=W2;d->is_uncond=1; FINISH; }
    if (op == 0x02) { ADV(1); fmt(d,"ldm #0x%02x",B1); FINISH; }
    if (op == 0x12) { ADV(1); fmt(d,"stm #0x%02x",B1); FINISH; }
    if (op == 0x03) { ADV(3); int pg=rbyte(page,o-3),ad=rword(page,o-2); fmt(d,"pjsr #0x%02x:%04x",pg,ad); d->has_target=1;d->tpage=pg;d->toff=ad;d->is_call=1; FINISH; }
    if (op == 0x13) { ADV(3); int pg=rbyte(page,o-3),ad=rword(page,o-2); fmt(d,"pjmp #0x%02x:%04x",pg,ad); d->has_target=1;d->tpage=pg;d->toff=ad;d->is_uncond=1; FINISH; }
    if (op == 0x08) { ADV(1); fmt(d,"trapa #0x%02x",B1); FINISH; }
    if (op == 0x18) { ADV(2); fmt(d,"jsr #0x%04x",W2); d->has_target=1;d->tpage=page;d->toff=W2;d->is_call=1; FINISH; }
    if (op == 0x0a) { fmt(d,"rte"); d->is_ret=1; FINISH; }
    if (op == 0x19) { fmt(d,"rts"); d->is_ret=1; FINISH; }
    if (op == 0x1a) { fmt(d,"sleep"); FINISH; }
    if (op == 0x0e) { ADV(1); int t=(off+2)+S1; fmt(d,"bsr %d -> 0x%04x",S1,t&0xffff); d->has_target=1;d->tpage=page;d->toff=t&0xffff; FINISH; }
    if (op == 0x1e) { ADV(2); int t=(off+3)+SW2; fmt(d,"bsr16 -> 0x%04x",t&0xffff); d->has_target=1;d->tpage=page;d->toff=t&0xffff; FINISH; }
    if (op == 0x14 || op == 0x1c) { ADV(1); fmt(d,"rtd %d",S1); FINISH; }
    if (op >= 0x20 && op <= 0x3f) { int c=op&0xf;
        if (op & 0x10) { ADV(2); int t=(off+3)+SW2; fmt(d,"%s 0x%04x -> 0x%04x",BCC_NAME[c],(unsigned)SW2,t&0xffff); d->has_target=1;d->tpage=page;d->toff=t&0xffff; }
        else { ADV(1); int t=(off+2)+S1; fmt(d,"%s %d -> 0x%04x",BCC_NAME[c],S1,t&0xffff); d->has_target=1;d->tpage=page;d->toff=t&0xffff; }
        FINISH; }
    if (op >= 0x40 && op <= 0x4f) { int reg=op&7,siz=op&8; if(siz){ADV(2);fmt(d,"cmp r%d,w #0x%04x",reg,W2);}else{ADV(1);fmt(d,"cmp r%d,b #0x%02x",reg,B1);} FINISH; }
    if (op >= 0x50 && op <= 0x57) { ADV(1); fmt(d,"move r%d #0x%02x",op&7,B1); FINISH; }
    if (op >= 0x58 && op <= 0x5f) { ADV(2); fmt(d,"movi r%d #0x%04x",op&7,W2); FINISH; }
    if (op >= 0x60 && op <= 0x6f) { ADV(1); fmt(d,"movl%s r%d @(br,$%02x)",(op&8)?"w":"",op&7,B1); FINISH; }
    if (op >= 0x70 && op <= 0x7f) { ADV(1); fmt(d,"movs%s r%d @(br,$%02x)",(op&8)?"w":"",op&7,B1); FINISH; }
    if (op >= 0x80 && op <= 0x9f) { ADV(1); fmt(d,"movf%s r%d @r6+%d",(op&8)?"w":"",op&7,S1); FINISH; }

    if ((op==0x04||op==0x05||op==0x0c||op==0x0d||op==0x15||op==0x1d) || op >= 0xa0) {
        int top=op&0xf0, reg=op&7, siz=op&8;
        int disp=0;
        if (top==0xe0){ disp=rbyte_s(page,o); ADV(1); }
        else if (top==0xf0){ disp=rword_s(page,o); ADV(2); }
        else if (top==0x00 && reg==5){ disp=rbyte(page,o); ADV(1); }
        else if (top==0x00 && reg==4){ if(siz){disp=rword(page,o);ADV(2);}else{disp=rbyte(page,o);ADV(1);} }
        else if (top==0x10 && reg==5){ disp=rword(page,o); ADV(2); }
        int opcode=rbyte(page,o); ADV(1);
        int ocode,ore,ext=0;
        if (opcode==0x00){ int e=rbyte(page,o); ADV(1); ore=e&7; ocode=e>>3; ext=1; }
        else { ocode=opcode>>3; ore=opcode&7; }
        char srcs[32];
        if (top==0xe0||top==0xf0) sprintf(srcs,"@r6+%d",disp);
        else if (top==0x00 && reg==5) sprintf(srcs,"(br,$%02x)",disp&0xff);
        else if (top==0x00 && reg==4) sprintf(srcs,"#%02x",disp&0xff);
        else if (top==0x10 && reg==5) sprintf(srcs,"(dp,$%02x)",disp&0xff);
        else sprintf(srcs,"?");
        fmt(d,"%s %s r%d%s",OPC_NAME[ocode&31],srcs,ore,ext?" x":"");
        FINISH;
    }
    fmt(d,"??0x%02x",op); FINISH;
}

// ---- ground-truth structures ----
#define AKEY_MAX (0x10 << 16)
static unsigned char *exec;      // executed PC bitmap
static unsigned char *in_deg;   // distinct src count per addr
static unsigned *dsts;         // flat list of dsts, bucketed by src (flow file is sorted by src)
static unsigned *src_off;      // start index per src
static unsigned *src_cnt;      // dst count per src
static unsigned edge_count;

static int has_edge(int src, int dst) {
    unsigned n = src_cnt[src];
    for (unsigned i = 0; i < n; i++) if (dsts[src_off[src] + i] == (unsigned)dst) return 1;
    return 0;
}

typedef struct { int addr; char line[512]; } line_t;
static int cmp_line(const void *a, const void *b) {
    const line_t *x = (const line_t*)a, *y = (const line_t*)b;
    return (x->addr > y->addr) - (x->addr < y->addr);
}
static line_t *lines; static int LN, LCAP;
static unsigned char *seen_line;
static void addline(int addr, const char *buf) {
    if (seen_line[addr]) return;
    seen_line[addr] = 1;
    if (LN == LCAP) { LCAP *= 2; lines = realloc(lines, LCAP * sizeof(line_t)); }
    lines[LN].addr = addr; strcpy(lines[LN].line, buf); LN++;
}

int main(int argc, char **argv) {
    if (argc < 6) { fprintf(stderr, "usage: h8dasm2 rom1 rom2 pcs flows out\n"); return 1; }
    FILE *f = fopen(argv[1], "rb"); ROM1 = malloc(ROM1_SIZE); if (fread(ROM1,1,ROM1_SIZE,f) != ROM1_SIZE) return 2; fclose(f);
    f = fopen(argv[2], "rb"); ROM2 = malloc(ROM2_SIZE); if (fread(ROM2,1,ROM2_SIZE,f) != ROM2_SIZE) return 3; fclose(f);
    FILE *out = fopen(argv[5], "w"); if (!out) return 4;

    exec = calloc(AKEY_MAX, 1);
    in_deg = calloc(AKEY_MAX, 1);
    src_off = calloc(AKEY_MAX + 1, sizeof(unsigned));
    src_cnt = calloc(AKEY_MAX, sizeof(unsigned));
    seen_line = calloc(AKEY_MAX, 1);
    lines = malloc((1<<14) * sizeof(line_t)); LCAP = 1<<14; LN = 0;

    // load executed PCs
    f = fopen(argv[3], "r");
    char lineb[64];
    long nexec = 0;
    while (fgets(lineb, sizeof lineb, f)) {
        int a = (int)strtol(lineb, 0, 16);
        exec[a] = 1; nexec++;
    }
    fclose(f);

    // load transitions (flow file is sorted by src)
    f = fopen(argv[4], "r");
    static unsigned *esrc, *edst;
    static size_t ESN = 16384;
    esrc = malloc(ESN * 4); edst = malloc(ESN * 4);
    unsigned nedges = 0;
    unsigned src, dst;
    while (fscanf(f, "%x %x", &src, &dst) == 2) {
        if (nedges == ESN) { ESN *= 2; esrc = realloc(esrc, ESN * 4); edst = realloc(edst, ESN * 4); }
        esrc[nedges] = src; edst[nedges] = dst; nedges++;
    }
    fclose(f);
    for (unsigned i = 0; i < nedges; i++) src_cnt[esrc[i]]++;
    for (unsigned a = 1; a < AKEY_MAX; a++) src_off[a] = src_off[a-1] + src_cnt[a-1];
    dsts = malloc(nedges * sizeof(unsigned));
    {
        unsigned *cursor = malloc(AKEY_MAX * sizeof(unsigned));
        memcpy(cursor, src_off, AKEY_MAX * sizeof(unsigned));
        for (unsigned i = 0; i < nedges; i++) dsts[cursor[esrc[i]]++] = edst[i];
        free(cursor);
    }
    for (unsigned i = 0; i < nedges; i++) in_deg[edst[i]]++;
    edge_count = nedges;
    fprintf(stderr, "exec=%ld lines=%lu edges=%lu\n", nexec, nedges, edge_count);
    fprintf(stderr, "phase: starts\n");

    // find run starts:
    //  1) executed PCs with in_deg == 0  (trace entry points)
    //  2) observed non-sequential edges: dst != src + len(src)
    static int is_start[AKEY_MAX];
    static int start_kind[AKEY_MAX]; // 0 none, 1 trace, 2 branch/call target, 3 ret target
    int starts = 0;
    for (int a = 0; a < AKEY_MAX; a++) {
        if (exec[a] && in_deg[a] == 0) { is_start[a] = 1; start_kind[a] = 1; starts++; }
    }
    for (int s = 0; s < AKEY_MAX; s++) {
        if (!exec[s]) continue;
        unsigned n = src_cnt[s];
        if (n == 0) continue;
        dec_t d; decode(s >> 16, s & 0xffff, &d);
        int seq = s + d.len;
        for (unsigned i = 0; i < n; i++) {
            int t = dsts[src_off[s] + i];
            if (t != seq && exec[t] && !is_start[t]) {
                is_start[t] = 1;
                if (d.is_call) start_kind[t] = 3; else start_kind[t] = 2;
                starts++;
            }
        }
    }
    fprintf(stderr, "run starts=%d\n", starts);

    // disassemble each run forward
    for (int st = 0; st < AKEY_MAX; st++) {
        if (!is_start[st]) continue;
        int pc = st;
        int guard = 0;
        while (pc < AKEY_MAX && guard++ < 16384) {
            if (!exec[pc]) break;
            if (seen_line[pc]) break; // tail already emitted by an earlier run
            int page = pc >> 16, off = pc & 0xffff;
            dec_t d; decode(page, off, &d);
            if (d.len < 1) break;
            char buf[512];
            snprintf(buf, sizeof buf, "%08x: %-40s [", pc, d.text);
            for (int b = 0; b < d.len && b < 8; b++) snprintf(buf+strlen(buf), 6, "%02x ", rbyte(page, off+b));
            char *nl = strrchr(buf, ' '); if (nl) *nl = 0; strcat(buf, "]");
            int seq = pc + d.len;
            // annotations
            int seq_obs = has_edge(pc, seq);
            if (seq_obs) strcat(buf, "  ;seq");
            unsigned nn = src_cnt[pc];
            int shown = 0;
            for (unsigned i = 0; i < nn; i++) {
                int t = dsts[src_off[pc] + i];
                if (t != seq) {
                    char tmp[32];
                    if (shown < 4) {
                        snprintf(tmp, sizeof tmp, "  ;-> %08x", t);
                        strcat(buf, tmp);
                        shown++;
                    } else if (shown == 4) {
                        snprintf(tmp, sizeof tmp, "  ;+%u more", nn - shown);
                        strcat(buf, tmp);
                        shown++;
                    }
                }
            }
            addline(pc, buf);
            if (seq_obs) pc = seq;
            else if (exec[seq]) { pc = seq; } // keep going: fall-through is executed code (branch not taken in this capture, or taken earlier)
            else break;
            if (!exec[pc]) break;
        }
    }

    fprintf(stderr, "phase: emit lines=%d\n", LN);
    qsort(lines, LN, sizeof(line_t), cmp_line);
    // emit with blank line + header when a run start begins
    int prev = -0x100;
    for (int i = 0; i < LN; i++) {
        int a = lines[i].addr;
        if (a - prev > 8 || is_start[a]) fprintf(out, "\n");
        if (is_start[a]) {
            if (start_kind[a] == 1) fprintf(out, "=== TRACE ENTRY ===\n");
            else if (start_kind[a] == 2) fprintf(out, "=== RUN START (branch target) ===\n");
            else if (start_kind[a] == 3) fprintf(out, "=== RUN START (call target) ===\n");
        }
        fprintf(out, "%s\n", lines[i].line);
        prev = a;
    }
    fprintf(stderr, "decoded %d instructions\n", LN);
    fclose(out);
    return 0;
}
