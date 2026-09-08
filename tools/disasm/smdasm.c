// smdasm.c - SC-55 SM (m740-family) 反汇编器。
// 用 tables.py 的完整 opcode 表（模板 + 寻址模式）解码 rom_sm，
// 由 logproc 提取的执行 PC 集（pc_sm.txt）驱动，输出反汇编 + 交叉引用。
//
// 地址映射：sm.pc = 0xfXXX  ->  sm_rom[0xXXX]   （SM_Read: addr &= 0x1fff, rom 0x1000-0x1fff）
//
// 用法: smdasm.exe rom_sm.bin pc_sm.txt out.txt [linear]
//   linear 模式: 全 ROM 线性反汇编（0xf000-0xffff），行首 "*" 标记已执行 PC。
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// addr_mode (与 tables.py AddressModes 一致)
// 0 Illegal 1 Immediate 2 Accumulator 3 ZeroPage 4 ZeroPageX 5 ZeroPageY 6 Absolute
// 7 AbsoluteX 8 AbsoluteY 9 Implied 10 Relative 11 IndirectX 12 IndirectY
// 13 IndirectAbsolute 14 ZeroPageIndirect 15 SpecialPage 16 ZeroPageBit
// 17 AccumulatorBit 18 AccumulatorBitRelative 19 ZeroPageBitRelative 20 ZeroPageImmediate
static const char *TMPL[256] = {
"brk","ora [{zp},x]","jsr [{zp}]","bbs 0,a,{rel}",".byte {opc}","ora {zp}","asl {zp}","bbs 0,{zp},{rel}",
"php","ora #{imm}","asl a","seb 0,a",".byte {opc}","ora {abs}","asl {abs}","seb 0,{zp}",
"bpl {rel}","ora [{zp}],y","clt","bbc 0,a,{rel}",".byte {opc}","ora {zp},x","asl {zp},x","bbc 0,{zp},{rel}",
"clc","ora {abs},y","dec a","clb 0,a",".byte {opc}","ora {abs},x","asl {abs},x","clb 0,{zp}",
"jsr {abs}","and [{zp},x]","jsr \\{sp}","bbs 1,a,{rel}","bit {zp}","and {zp}","rol {zp}","bbs 1,{zp},{rel}",
"plp","and #{imm}","rol a","seb 1,a","bit {abs}","and {abs}","rol {abs}","seb 1,{zp}",
"bmi {rel}","and [{zp}],y","set","bbc 1,a,{rel}",".byte {opc}","and {zp},x","rol {zp},x","bbc 1,{zp},{rel}",
"sec","and {abs},y","inc a","clb 1,a","ldm #{imm},{zp}","and {abs},x","rol {abs},x","clb 1,{zp}",
"rti","eor [{zp},x]","stp","bbs 2,a,{rel}","com {zp}","eor {zp}","lsr {zp}","bbs 2,{zp},{rel}",
"pha","eor #{imm}","lsr a","seb 2,a","jmp {abs}","eor {abs}","lsr {abs}","seb 2,{zp}",
"bvc {rel}","eor [{zp}],y",".byte {opc}","bbc 2,a,{rel}",".byte {opc}","eor {zp},x","lsr {zp},x","bbc 2,{zp},{rel}",
"cli","eor {abs},y",".byte {opc}","clb 2,a",".byte {opc}","eor {abs},x","lsr {abs},x","clb 2,{zp}",
"rts","adc [{zp},x]","mul {zp},x","bbs 3,a,{rel}","tst {zp}","adc {zp}","ror {zp}","bbs 3,{zp},{rel}",
"pla","adc #{imm}","ror a","seb 3,a","jmp [{abs}]","adc {abs}","ror {abs}","seb 3,{zp}",
"bvs {rel}","adc [{zp}],y",".byte {opc}","bbc 3,a,{rel}",".byte {opc}","adc {zp},x","ror {zp},x","bbc 3,{zp},{rel}",
"sei","adc {abs},y",".byte {opc}","clb 3,a",".byte {opc}","adc {abs},x","ror {abs},x","clb 3,{zp}",
"bra {rel}","sta [{zp},x]","rrf {zp}","bbs 4,a,{rel}","sty {zp}","sta {zp}","stx {zp}","bbs 4,{zp},{rel}",
"dey",".byte {opc}","txa","seb 4,a","sty {abs}","sta {abs}","stx {abs}","seb 4,{zp}",
"bcc {rel}","sta [{zp}],y",".byte {opc}","bbc 4,a,{rel}","sty {zp},x","sta {zp},x","stx {zp},y","bbc 4,{zp},{rel}",
"tya","sta {abs},y","txs","clb 4,a",".byte {opc}","sta {abs},x",".byte {opc}","clb 4,{zp}",
"ldy #{imm}","lda [{zp},x]","ldx #{imm}","bbs 5,a,{rel}","ldy {zp}","lda {zp}","ldx {zp}","bbs 5,{zp},{rel}",
"tay","lda #{imm}","tax","seb 5,a","ldy {abs}","lda {abs}","ldx {abs}","seb 5,{zp}",
"bcs {rel}","lda [{zp}],y","jmp [{zp}]","bbc 5,a,{rel}","ldy {zp},x","lda {zp},x","ldx {zp},y","bbc 5,{zp},{rel}",
"clv","lda {abs},y","tsx","clb 5,a","ldy {abs},x","lda {abs},x","ldx {abs},y","clb 5,{zp}",
"cpy #{imm}","cmp [{zp},x]","wit","bbs 6,a,{rel}","cpy {zp}","cmp {zp}","dec {zp}","bbs 6,{zp},{rel}",
"iny","cmp #{imm}","dex","seb 6,a","cpy {abs}","cmp {abs}","dec {abs}","seb 6,{zp}",
"bne {rel}","cmp [{zp}],y",".byte {opc}","bbc 6,a,{rel}",".byte {opc}","cmp {zp},x","dec {zp},x","bbc 6,{zp},{rel}",
"cld","cmp {abs},y",".byte {opc}","clb 6,a",".byte {opc}","cmp {abs},x","dec {abs},x","clb 6,{zp}",
"cpx #{imm}","sbc [{zp},x]","div {zp},x","bbs 7,a,{rel}","cpx {zp}","sbc {zp}","inc {zp}","bbs 7,{zp},{rel}",
"inx","sbc #{imm}","nop","seb 7,a","cpx {abs}","sbc {abs}","inc {abs}","seb 7,{zp}",
"beq {rel}","sbc [{zp}],y",".byte {opc}","bbc 7,a,{rel}",".byte {opc}","sbc {zp},x","inc {zp},x","bbc 7,{zp},{rel}",
"sed","sbc {abs},y",".byte {opc}","clb 7,a",".byte {opc}","sbc {abs},x","inc {abs},x","clb 7,{zp}"
};
static const unsigned char AM[256] = {
9,11,14,18,0,3,3,19, 9,1,2,17,0,6,6,16,
10,12,9,18,0,4,4,19, 9,8,2,17,0,7,7,16,
6,11,15,18,3,3,3,19, 9,1,2,17,6,6,6,16,
10,12,9,18,0,4,4,19, 9,8,2,17,20,7,7,16,
9,11,9,18,3,3,3,19, 9,1,2,17,6,6,6,16,
10,12,0,18,0,4,4,19, 9,8,0,17,0,7,7,16,
9,11,4,18,3,3,3,19, 9,1,2,17,13,6,6,16,
10,12,0,18,0,4,4,19, 9,8,0,17,0,7,7,16,
10,11,3,18,3,3,3,19, 9,0,9,17,6,6,6,16,
10,12,0,18,4,4,5,19, 9,8,9,17,0,7,0,16,
1,11,1,18,3,3,3,19, 9,1,9,17,6,6,6,16,
10,12,14,18,4,4,5,19, 9,8,9,17,7,7,8,16,
1,11,9,18,3,3,3,19, 9,1,9,17,6,6,6,16,
10,12,0,18,0,4,4,19, 9,8,0,17,0,7,7,16,
1,11,4,18,3,3,3,19, 9,1,9,17,6,6,6,16,
10,12,0,18,0,4,4,19, 9,8,0,17,0,7,7,16,
};

static uint8_t *ROM;          // sm_rom (4KB)
static int ROMSIZE = 0;
static uint8_t executed[0x10000]; // PC 执行集（sm.pc 16-bit）
static int exec_n = 0;
static int *exec_sorted;      // 排序后的 PC

static int cmp_i(const void *a, const void *b) {
    int x = *(const int *)a, y = *(const int *)b;
    return (x > y) - (x < y);
}

static int is_exec(int pc) { return executed[pc & 0xffff]; }

// 解码 po（sm_rom 偏移）/ p（sm.pc = 0xf000+po）处一条指令。返回指令长度，
// 填 opline（助记符+操作数）、xref（交叉引用后缀，可能为空）。
static int decode_instr(int po, int p, char *opline, int opls, char *xref, int xrls) {
    opline[0] = 0; xref[0] = 0;
    if (po >= ROMSIZE) { snprintf(opline, opls, ".byte 0x??"); return 1; }
    uint8_t op = ROM[po];
    int am = AM[op];
    int len;
    char opstr[64] = "", absstr[16] = "", imstr[16] = "", relstr[16] = "", spstr[16] = "", opcstr[8] = "";
    int op1 = (po + 1 < ROMSIZE) ? ROM[po + 1] : 0, op2 = (po + 2 < ROMSIZE) ? ROM[po + 2] : 0;
    int w = (op2 << 8) | op1; // 绝对地址 = op2(高):op1(低)
    switch (am) {
        case 0: len = 1; snprintf(opcstr, sizeof opcstr, "0x%02x", op); break;
        case 1: len = 2; snprintf(imstr, sizeof imstr, "0x%02x", op1); break;
        case 2: len = 1; break;
        case 3: len = 2; snprintf(opstr, sizeof opstr, "0x%02x", op1); break;
        case 4: len = 2; snprintf(opstr, sizeof opstr, "0x%02x", op1); break;
        case 5: len = 2; snprintf(opstr, sizeof opstr, "0x%02x", op1); break;
        case 6: len = 3; snprintf(absstr, sizeof absstr, "0x%04x", w); break;
        case 7: len = 3; snprintf(absstr, sizeof absstr, "0x%04x", w); break;
        case 8: len = 3; snprintf(absstr, sizeof absstr, "0x%04x", w); break;
        case 9: len = 1; break;
        case 10: { len = 2; int t = (p + 2 + (int)(int8_t)op1) & 0xffff; snprintf(relstr, sizeof relstr, "0x%04x", t); } break;
        case 11: len = 2; snprintf(opstr, sizeof opstr, "0x%02x", op1); break;
        case 12: len = 2; snprintf(opstr, sizeof opstr, "0x%02x", op1); break;
        case 13: len = 3; snprintf(absstr, sizeof absstr, "0x%04x", w); break;
        case 14: len = 2; snprintf(opstr, sizeof opstr, "0x%02x", op1); break;
        case 15: len = 2; snprintf(spstr, sizeof spstr, "0x%04x", 0xff00 | op1); break;
        case 16: len = 2; snprintf(opstr, sizeof opstr, "0x%02x", op1); break;
        case 17: len = 1; break;
        case 18: { len = 2; int t = (p + 2 + (int)(int8_t)op1) & 0xffff; snprintf(relstr, sizeof relstr, "0x%04x", t); } break;
        case 19: { len = 3; int t = (p + 3 + (int)(int8_t)op2) & 0xffff;
                    snprintf(opstr, sizeof opstr, "0x%02x", op1); snprintf(relstr, sizeof relstr, "0x%04x", t); } break;
        case 20: { len = 3; snprintf(imstr, sizeof imstr, "0x%02x", op1); snprintf(opstr, sizeof opstr, "0x%02x", op2); } break;
        default: len = 1; snprintf(opcstr, sizeof opcstr, "0x%02x", op); break;
    }
    char line[96]; const char *t = TMPL[op]; char *dst = line;
    while (*t) {
        if (*t == '{') {
            const char *e = strchr(t, '}');
            if (e && e - t > 1) {
                int n = e - t - 1;
                char ph[8] = {0};
                if (n < 8) { memcpy(ph, t + 1, n); ph[n] = 0; }
                const char *rep = (strcmp(ph, "zp") == 0) ? opstr :
                                 (strcmp(ph, "abs") == 0) ? absstr :
                                 (strcmp(ph, "imm") == 0) ? imstr :
                                 (strcmp(ph, "rel") == 0) ? relstr :
                                 (strcmp(ph, "sp") == 0) ? spstr :
                                 (strcmp(ph, "opc") == 0) ? opcstr : "?";
                while (*rep) *dst++ = *rep++;
                t = e + 1; continue;
            }
        }
        *dst++ = *t++;
    }
    *dst = 0;
    if (am == 10 || am == 18 || am == 19) {
        int tgt = (p + len + (int)(int8_t)(am == 19 ? op2 : op1)) & 0xffff;
        snprintf(xref, xrls, "  ;cond %04x [%s]", tgt, is_exec(tgt) ? "xref" : "miss");
    } else if (am == 6 && op == 0x4c) {
        snprintf(xref, xrls, "  ;jmp %04x [%s]", w, is_exec(w) ? "xref" : "miss");
    } else if (am == 6 && op == 0x20) {
        snprintf(xref, xrls, "  ;jsr %04x [%s]", w, is_exec(w) ? "xref" : "miss");
    } else if (am == 13) {
        snprintf(xref, xrls, "  ;jmp [abs]");
    }
    snprintf(opline, opls, "%s", line);
    return len;
}

int main(int argc, char **argv) {
    if (argc < 4) { fprintf(stderr, "usage: smdasm rom_sm.bin pc_sm.txt out.txt\n"); return 1; }
    FILE *f = fopen(argv[1], "rb");
    ROM = malloc(0x10000);
    ROMSIZE = (int)fread(ROM, 1, 0x10000, f);
    fclose(f);
    f = fopen(argv[2], "r");
    int pc;
    exec_sorted = malloc(0x10000 * sizeof(int));
    while (fscanf(f, "%x", &pc) == 1) {
        if (pc >= 0 && pc < 0x10000 && !executed[pc]) { executed[pc] = 1; exec_sorted[exec_n++] = pc; }
    }
    fclose(f);
    qsort(exec_sorted, exec_n, sizeof(int), cmp_i);

    int linear = (argc >= 5 && strcmp(argv[4], "linear") == 0);
    FILE *out = fopen(argv[3], "w");
    if (linear) {
        fprintf(out, "// SM (m740) 全 ROM 线性反汇编, rom %d bytes\n", ROMSIZE);
        fprintf(out, "// addr = sm.pc (0xf000+off -> sm_rom[off]); 行首 '*' = 已执行 PC（锚点，可信）\n");
        fprintf(out, "// 段头 [VM-verified] = 该段含已执行 PC；[linear/static, unverified] = 仅线性反汇编（未进执行集）\n");
        fprintf(out, "// 多入口：RESET(0xf003) + 9 中断向量；各段独立线性反汇编，数据区/缺口会漂移\n\n");
        static const char *vname[10] = {"UART3_TX","UART2_TX","UART1_TX","COLLISION","TIMER_X","IPCM0","UART3_RX","UART2_RX","UART1_RX","RESET"};
        int eaddr[10], eidx[10];
        for (int n = 0; n < 10; n++) {
            int off = 0x0fec + n*2;
            eaddr[n] = ROM[off] | (ROM[off+1] << 8);
            eidx[n] = n;
        }
        for (int i = 0; i < 9; i++) for (int j = i+1; j < 10; j++) { // 按地址排序
            if (eaddr[j] < eaddr[i]) { int t=eaddr[i]; eaddr[i]=eaddr[j]; eaddr[j]=t; int u=eidx[i]; eidx[i]=eidx[j]; eidx[j]=u; }
        }
        static uint8_t used[0x10000];
        int total = 0;
        for (int i = 0; i < 10; i++) {
            int start = eaddr[i];
            int stop = (i < 9) ? eaddr[i+1] : 0x10000; // 下一入口为止
            int verified = 0;
            for (int x = start; x < stop && x < 0x10000; x++) if (is_exec(x)) { verified = 1; break; }
            fprintf(out, "\n=== %s  @ 0x%04x  .. 0x%04x  [%s] ===\n", vname[eidx[i]], start, stop,
                    verified ? "VM-verified" : "linear/static, unverified");
            int p = start;
            while (p >= 0xf000 && p < stop && !used[p]) {
                used[p] = 1; total++;
                int po = p & 0xfff;
                char opline[96], xref[64];
                int len = decode_instr(po, p, opline, sizeof opline, xref, sizeof xref);
                char bytes[16] = "";
                for (int b = 0; b < len && b < 4; b++) { if (po + b < ROMSIZE) { strcat(bytes, " "); snprintf(bytes + strlen(bytes), sizeof bytes - strlen(bytes), "%02x", ROM[po + b]); } }
                const char *mark = is_exec(p) ? "*" : " ";
                fprintf(out, "%s%04x: %-28s %s%s\n", mark, p, opline, bytes, xref);
                p += len;
            }
        }
        fprintf(stderr, "smdasm: linear multi-entry, %d instr -> %s\n", total, argv[3]);
    } else {
        fprintf(out, "// SM (m740) disassembly - %d executed PCs, rom %d bytes\n", exec_n, ROMSIZE);
        fprintf(out, "// addr = sm.pc (0xf000+off -> sm_rom[off]); [xref] 标记目标是否在执行集内\n");
        fprintf(out, "// 只在 fall-through 落到下一个执行 PC 时继续（数据/缺口自然断开）\n\n");
        static uint8_t decoded[0x10000];
        for (int i = 0; i < exec_n; i++) {
            int pc = exec_sorted[i];
            if (decoded[pc]) continue; // 已作为上一个 run 的 fall-through 输出
            int p = pc;
            while (p <= 0xffff && is_exec(p)) {
                decoded[p] = 1;
                int po = p & 0xfff;
                if (po >= ROMSIZE) break;
                char opline[96], xref[64];
                int len = decode_instr(po, p, opline, sizeof opline, xref, sizeof xref);
                char bytes[16] = "";
                for (int b = 0; b < len && b < 4; b++) { if (po + b < ROMSIZE) { strcat(bytes, " "); snprintf(bytes + strlen(bytes), sizeof bytes - strlen(bytes), "%02x", ROM[po + b]); } }
                fprintf(out, "%04x: %-28s %s%s\n", p, opline, bytes, xref);
                p += len;
            }
        }
        fprintf(stderr, "smdasm: %d PCs -> %s\n", exec_n, argv[3]);
    }
    fclose(out);
    return 0;
}
