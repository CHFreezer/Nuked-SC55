// logproc.c - 解析 vm.log。
//   main 行: "m <mcu_cycles> CP:PC [state]" -> main addr = (CP<<16)|PC
//   SM 行:   "s <sm_cycles> <pc>"           -> 每条 SM 指令一行（完整 SM PC 日志）
// 功能：
//   1) 统计 main / SM 的 PC 执行频率，打印热点 top N
//   2) 打印前 5 行原始样本（stderr，验证格式）
//   3) 提取 main PC 去重集合 -> outdir/pc_main.txt（8 位 hex，升序）
//   4) 提取 SM PC 去重集合   -> outdir/pc_sm.txt（4 位 hex，升序；来自 s 行，完整）
//   5) 提取 main PC 变化边（src dst，去重排序）-> outdir/flow_main.txt
// 用法: logproc.exe vm.log outdir [topN]
// 注意: main 行需为 noregs 格式（"main pc=" 开头）；verbose 行以数字开头会被跳过。
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_BUCKETS (0x100000)
#define SM_BUCKETS   (0x10000)

static uint64_t main_cnt[MAIN_BUCKETS];
static uint64_t sm_cnt[SM_BUCKETS];

static uint64_t *edges = NULL; static uint64_t edges_n = 0, edges_cap = 0;
static void add_edge(uint32_t src, uint32_t dst) {
    if (!edges || edges_n == edges_cap) {
        edges_cap = edges_cap ? edges_cap * 2 : (1 << 20);
        edges = realloc(edges, edges_cap * sizeof(uint64_t));
    }
    edges[edges_n++] = ((uint64_t)src << 32) | dst;
}
static int cmp_u64(const void *a, const void *b) {
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

typedef struct { uint32_t addr; uint64_t cnt; } rec_t;
static rec_t mrec[MAIN_BUCKETS]; static uint32_t mn = 0;
static rec_t srec[SM_BUCKETS];   static uint32_t sn = 0;
static int cmp_rec(const void *a, const void *b) {
    const rec_t *x = a, *y = b;
    return (x->cnt < y->cnt) - (x->cnt > y->cnt); // 降序
}

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: logproc vm.log outdir [topN]\n"); return 1; }
    int topN = argc >= 4 ? atoi(argv[3]) : 30;
    FILE *f = fopen(argv[1], "rb");
    if (!f) { fprintf(stderr, "open fail: %s\n", argv[1]); return 2; }
    char line[160];
    uint64_t total_main = 0, total_sm = 0;
    uint32_t prev = 0; int have_prev = 0, sample = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == 's' && line[1] == ' ') {
            // "s <sm_cycles> <pc>" -> one line per SM instruction (complete log)
            char *sp = line + 2;
            while (*sp == ' ') sp++;       // skip leading spaces
            while (*sp && *sp != ' ') sp++; // skip <sm_cycles>
            while (*sp == ' ') sp++;        // skip space
            int spc = 0;
            if (sscanf(sp, "%x", &spc) == 1 && (unsigned)spc < SM_BUCKETS) { sm_cnt[spc]++; total_sm++; }
            continue;
        }
        if (line[0] == 'm' && line[1] == ' ') {
            // "m <mcu_cycles> <cp:pc> [state]" -> main instruction
            char *mp = line + 2;
            while (*mp && *mp != ' ') mp++;   // skip <mcu_cycles>
            while (*mp == ' ') mp++;
            if (sample < 5) { fprintf(stderr, "sample: %s", line); sample++; }
            int cpv = 0, pc = 0;
            if (sscanf(mp, "%x:%x", &cpv, &pc) == 2) {
                uint32_t addr = ((uint32_t)cpv << 16) | (uint32_t)pc;
            if (addr < MAIN_BUCKETS) {
                main_cnt[addr]++; total_main++;
                if (have_prev && addr != prev) add_edge(prev, addr);
                prev = addr; have_prev = 1;
            }
            continue;
            }
        }
    }
    fclose(f);
    char buf[512];
    snprintf(buf, sizeof buf, "%s/pc_main.txt", argv[2]);
    FILE *p = fopen(buf, "w");
    for (uint32_t a = 0; a < MAIN_BUCKETS; a++) if (main_cnt[a]) fprintf(p, "%08x\n", a);
    fclose(p);
    snprintf(buf, sizeof buf, "%s/pc_sm.txt", argv[2]);
    FILE *ps = fopen(buf, "w");
    for (uint32_t a = 0; a < SM_BUCKETS; a++) if (sm_cnt[a]) fprintf(ps, "%04x\n", a);
    fclose(ps);
    qsort(edges, edges_n, sizeof(uint64_t), cmp_u64);
    snprintf(buf, sizeof buf, "%s/flow_main.txt", argv[2]);
    FILE *fl = fopen(buf, "w");
    uint64_t last = ~0ULL; int uniq = 0;
    for (uint64_t i = 0; i < edges_n; i++) {
        if (edges[i] == last) continue;
        last = edges[i];
        fprintf(fl, "%08x %08x\n", (uint32_t)(edges[i] >> 32), (uint32_t)edges[i]);
        uniq++;
    }
    fclose(fl);
    for (uint32_t a = 0; a < MAIN_BUCKETS; a++) if (main_cnt[a]) { mrec[mn].addr = a; mrec[mn].cnt = main_cnt[a]; mn++; }
    for (uint32_t a = 0; a < SM_BUCKETS; a++) if (sm_cnt[a]) { srec[sn].addr = a; srec[sn].cnt = sm_cnt[a]; sn++; }
    qsort(mrec, mn, sizeof(rec_t), cmp_rec);
    qsort(srec, sn, sizeof(rec_t), cmp_rec);
    printf("main: %llu exec, %u unique | sm: %llu exec, %u unique | edges(raw)=%llu uniq=%d\n",
        (unsigned long long)total_main, mn, (unsigned long long)total_sm, sn, (unsigned long long)edges_n, uniq);
    printf("=== TOP %d main ===\n", topN);
    for (uint32_t i = 0; i < mn && i < (uint32_t)topN; i++) printf("  0x%08x x%llu\n", mrec[i].addr, (unsigned long long)mrec[i].cnt);
    printf("=== TOP %d sm ===\n", topN);
    for (uint32_t i = 0; i < sn && i < (uint32_t)topN; i++) printf("  0x%04x x%llu\n", srec[i].addr, (unsigned long long)srec[i].cnt);
    return 0;
}
