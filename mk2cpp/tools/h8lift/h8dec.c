/*
 * h8dec.c - standalone H8/532 decoder matching GT decode/semantics.
 *
 * Ground truth:
 *   src/mcu_opcodes.cpp  - operand families, 00 extended opcode, per-opcode traps,
 *                          MOVG_Immediate immediate tails (incl. the ore=4/5
 *                          size-mismatch FIXME cases at :825)
 *   src/mcu.h            - operand/control-register paths
 *   src/mcu.cpp:663-667  - rom1/rom2 bank rules (address_rom bit reshuffle)
 * Field layout mirrors tools/disasm/h8dasm.c's optional 6th-arg mach dump so
 * --check-decode can compare against tools/baselines/mach_*.txt.
 *
 * Build: clang -O2 -Wno-deprecated-declarations -o h8dec.exe h8dec.c
 * Usage: h8dec --check-decode <mach_main.txt> <mach_vec.txt>
 *              [--rom1 <rom1.bin> --rom2 <rom2.bin>]
 */
#define _CRT_SECURE_NO_WARNINGS
#include "h8dec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint8_t *g_rom1;
static const uint8_t *g_rom2;
static uint32_t g_rom1_size;
static uint32_t g_rom2_size;

void h8dec_set_rom(const uint8_t *rom1, uint32_t rom1_size,
                   const uint8_t *rom2, uint32_t rom2_size)
{
    g_rom1 = rom1;
    g_rom1_size = rom1_size;
    g_rom2 = rom2;
    g_rom2_size = rom2_size;
}

/* Bank rules: tools/disasm/h8dasm.c:21-28 == src/mcu.cpp MCU_Read_Impl rom
 * paths (:673-675 page0/rom1, :791-802/:803-814 rom2 with address_rom bit
 * reshuffle). Page = (address>>16)&0xf, off wraps at 16 bits. */
static uint8_t rbyte(uint32_t page, uint32_t off)
{
    uint32_t addr = ((page & 0xf) << 16) | (off & 0xffff);
    uint32_t pg = (addr >> 16) & 0xf;
    uint32_t o = addr & 0xffff;
    if (pg == 0 && g_rom1 && o < g_rom1_size)
        return g_rom1[o];
    if (!g_rom2 || g_rom2_size == 0)
        return 0xff;
    uint32_t idx = addr & 0x3ffff;
    if (addr & 0x80000)
        idx |= 0x40000;
    return g_rom2[idx % g_rom2_size];
}

static uint16_t rword(uint32_t page, uint32_t off)
{
    return (uint16_t)((rbyte(page, off) << 8) | rbyte(page, off + 1));
}

static int16_t rword_s(uint32_t page, uint32_t off)
{
    return (int16_t)rword(page, off);
}

static int8_t rbyte_s(uint32_t page, uint32_t off)
{
    return (int8_t)rbyte(page, off);
}

/* Operand classes from MCU_Operand_General (src/mcu_opcodes.cpp:543-653). */
enum { OP_DIRECT = 0, OP_INDIRECT, OP_ABSOLUTE, OP_IMMEDIATE };

/* Control register access sets (src/mcu.h:217-330). */
static int cr_ok(int siz, int reg)
{
    if (siz)
        return reg == 0 || reg == 3 || reg == 4 || reg == 5;
    return reg == 1 || reg == 3 || reg == 4 || reg == 5 || reg == 7;
}

/* Decode-time validity: 1 when GT reaches no MCU_ErrorTrap / invalid-instruction
 * exception for this form (src/mcu_opcodes.cpp opcode table, :1727-1760). */
static int general_valid(int ocode, int ore, int ext, int type, int siz)
{
    switch (ocode)
    {
    case 0: /* MOVG_Immediate :825 */
        return (type == OP_INDIRECT || type == OP_ABSOLUTE) && ore >= 4;
    case 1: /* ADDQ :1141 */
        return ore == 0 || ore == 1 || ore == 4 || ore == 5;
    case 2: /* CLR/SWAP/EXTS/EXTU/NEG/NOT/TST :937 */
        if (ore == 3 || ore == 4 || ore == 5 || ore == 6)
            return type != OP_IMMEDIATE;
        if (ore == 0 || ore == 1 || ore == 2)
            return type == OP_DIRECT && siz == 0;
        return 0;
    case 3: /* SHLR family :1219 */
        return type != OP_IMMEDIATE && ore <= 6;
    case 9:  /* BSET_ORC :875 */
    case 11: /* BCLR_ANDC :899 */
        return type != OP_IMMEDIATE || cr_ok(siz, ore);
    case 13: /* NotImplemented :820 */
        return 0;
    case 15: /* BTST :923 */
        return type != OP_IMMEDIATE;
    case 16: /* MOVG read :1041 (opcode_extended traps) */
        return !ext;
    case 17: /* LDC :996 */
        return cr_ok(siz, ore);
    case 18: /* MOVG write / XCH :1041 */
        if (ext)
            return 0;
        if (type == OP_DIRECT)
            return siz != 0;
        return 1;
    case 19: /* STC :1003 */
        return type != OP_IMMEDIATE && cr_ok(siz, ore);
    case 24: case 25: case 26: case 27:
    case 28: case 29: case 30: case 31: /* BSET/BCLR/BNOTI/BTSTI :1009-1125 */
        return type != OP_IMMEDIATE;
    default: /* ADD/ADDS/SUB/SUBS/OR/AND/XOR/CMP/ADDX/MULXU/SUBX/DIVXU */
        return 1;
    }
}

/* informational counters for GT-vs-h8dasm divergences seen by the checker */
static long g_bank_divergent;
static long g_fixme_ore4_word;
static long g_fixme_ore5_byte;

h8dec_t h8dec(uint32_t flat)
{
    h8dec_t d;
    uint32_t page = flat >> 16;
    uint32_t off = flat & 0xffff;
    uint32_t o = off;
    int valid = 0;

    memset(&d, 0, sizeof d);
    if (!g_rom1 && !g_rom2)
        return d;

    /* pages GT backs with something other than rom2 (src/mcu.cpp:663-854);
     * h8dasm.c rbyte maps them to rom2 regardless. Baseline PCs do not hit
     * these, but the transliterator must know when it leaves ROM. */
    {
        uint32_t pg = page & 0xf;
        if (pg == 5 || pg == 6 || pg == 7 || (pg >= 10 && pg <= 13) ||
            (pg == 0 && off >= 0x8000))
            g_bank_divergent++;
    }

    uint8_t op = rbyte(page, off);
    o++;

    if (op == 0x00) /* nop (operand table[0] = Nop) */
    {
        valid = 1;
    }
    else if (op == 0x01 || op == 0x06 || op == 0x07)
    {
        /* MCU_Jump_JMP :372-443; opcode2>>3 must be 0x17, else trap. */
        uint8_t b = rbyte(page, off + 1);
        if ((b >> 3) == 0x17)
        {
            int disp = (int8_t)rbyte(page, off + 2);
            uint32_t t = (uint32_t)((off + 3 + disp) & 0xffff);
            o += 2;
            d.kind = H8K_COND;
            d.tpage = (uint8_t)page;
            d.toff = (uint16_t)t;
            d.target = (page << 16) | t;
            valid = 1;
        }
        else
        {
            o += 1;
            d.kind = H8K_REGIND; /* h8dasm marks the trap path kind=5 */
        }
    }
    else if (op == 0x11)
    {
        /* MCU_Jump_JMP :340-370 register-indirect family. */
        uint8_t b = rbyte(page, off + 1);
        uint8_t h = b >> 3;
        o += 1;
        if (b == 0x19) /* ret (pop cp,pc) */
        {
            d.kind = H8K_RET;
            valid = 1;
        }
        else if (h == 0x19 || h == 0x1a || h == 0x1b)
        {
            /* jsr via rN:rN+1 (push pc/cp, call) / jmp rN / jsr rN */
            d.kind = H8K_REGIND;
            valid = 1;
        }
        else
        {
            d.kind = H8K_REGIND;
        }
    }
    else if (op == 0x10) /* MCU_Jump_JMP :391-397 jmp #abs16 */
    {
        o += 2;
        d.kind = H8K_UNCOND;
        d.tpage = (uint8_t)page;
        d.toff = rword(page, off + 1);
        d.target = (page << 16) | d.toff;
        valid = 1;
    }
    else if (op == 0x02 || op == 0x12) /* LDM :154 / STM :169 */
    {
        o += 1;
        valid = 1;
    }
    else if (op == 0x03 || op == 0x13) /* PJSR :198 / PJMP :466 */
    {
        o += 3;
        d.tpage = rbyte(page, off + 1);
        d.toff = rword(page, off + 2);
        d.kind = (op == 0x03) ? H8K_CALL : H8K_UNCOND;
        d.target = ((uint32_t)d.tpage << 16) | d.toff;
        valid = 1;
    }
    else if (op == 0x08) /* TRAPA :185: trap unless opcode&0xf0 == 0x10 */
    {
        uint8_t b = rbyte(page, off + 1);
        o += 1;
        valid = (b & 0xf0) == 0x10;
    }
    else if (op == 0x18) /* MCU_Jump_JSR :214 */
    {
        o += 2;
        d.kind = H8K_CALL;
        d.tpage = (uint8_t)page;
        d.toff = rword(page, off + 1);
        d.target = (page << 16) | d.toff;
        valid = 1;
    }
    else if (op == 0x0a || op == 0x19) /* RTE :223 / RTS :311 */
    {
        d.kind = H8K_RET;
        valid = 1;
    }
    else if (op == 0x1a) /* sleep */
    {
        valid = 1;
    }
    else if (op == 0x0e) /* MCU_Jump_BSR :450 signed disp8 */
    {
        int disp = (int8_t)rbyte(page, off + 1);
        uint32_t t = (uint32_t)((off + 2 + disp) & 0xffff);
        o += 1;
        d.kind = H8K_CALL;
        d.tpage = (uint8_t)page;
        d.toff = (uint16_t)t;
        d.target = (page << 16) | t;
        valid = 1;
    }
    else if (op == 0x1e) /* MCU_Jump_BSR :457-461 unsigned disp16 */
    {
        int disp = (int16_t)rword(page, off + 1);
        uint32_t t = (uint32_t)((off + 3 + disp) & 0xffff);
        o += 2;
        d.kind = H8K_CALL;
        d.tpage = (uint8_t)page;
        d.toff = (uint16_t)t;
        d.target = (page << 16) | t;
        valid = 1;
    }
    else if (op == 0x14 || op == 0x1c) /* RTD :316: 0x14 valid, 0x1c traps */
    {
        o += 1;
        d.kind = H8K_RET;
        valid = (op == 0x14);
    }
    else if (op >= 0x20 && op <= 0x3f) /* MCU_Jump_Bcc :231 */
    {
        if (op & 0x10)
        {
            int disp = (int16_t)rword(page, off + 1);
            uint32_t t = (uint32_t)((off + 3 + disp) & 0xffff);
            o += 2;
            d.toff = (uint16_t)t;
        }
        else
        {
            int disp = (int8_t)rbyte(page, off + 1);
            uint32_t t = (uint32_t)((off + 2 + disp) & 0xffff);
            o += 1;
            d.toff = (uint16_t)t;
        }
        d.kind = H8K_COND;
        d.tpage = (uint8_t)page;
        d.target = (page << 16) | d.toff;
        valid = 1;
    }
    else if (op >= 0x40 && op <= 0x4f) /* Short_CMP :802 */
    {
        o += (op & 8) ? 2 : 1;
        valid = 1;
    }
    else if (op >= 0x50 && op <= 0x57) /* Short_MOVE :694 */
    {
        o += 1;
        valid = 1;
    }
    else if (op >= 0x58 && op <= 0x5f) /* Short_MOVI :703 */
    {
        o += 2;
        valid = 1;
    }
    else if (op >= 0x60 && op <= 0x7f) /* Short_MOVL :755 / Short_MOVS :779 */
    {
        o += 1;
        valid = 1;
    }
    else if (op >= 0x80 && op <= 0x9f) /* Short_MOVF :713 */
    {
        o += 1;
        valid = 1;
    }
    else if (op == 0x04 || op == 0x05 || op == 0x0c || op == 0x0d ||
             op == 0x15 || op == 0x1d || op >= 0xa0)
    {
        /* MCU_Operand_General :543-673 */
        uint8_t top = op & 0xf0;
        uint8_t reg = op & 0x07;
        uint8_t siz = op & 0x08;
        int type = OP_DIRECT;

        if (top == 0xe0)
        {
            (void)rbyte_s(page, o);
            o += 1;
            type = OP_INDIRECT;
        }
        else if (top == 0xf0)
        {
            (void)rword_s(page, o);
            o += 2;
            type = OP_INDIRECT;
        }
        else if (top == 0x00 && reg == 5)
        {
            (void)rbyte(page, o);
            o += 1;
            type = OP_ABSOLUTE;
        }
        else if (top == 0x00 && reg == 4)
        {
            if (siz)
            {
                (void)rword(page, o);
                o += 2;
            }
            else
            {
                (void)rbyte(page, o);
                o += 1;
            }
            type = OP_IMMEDIATE;
        }
        else if (top == 0x10 && reg == 5)
        {
            (void)rword(page, o);
            o += 2;
            type = OP_ABSOLUTE;
        }
        else if (top == 0xb0 || top == 0xc0 || top == 0xd0)
        {
            type = OP_INDIRECT;
        }

        uint8_t opcode = rbyte(page, o);
        o += 1;
        int ext = 0, ocode, ore;
        if (opcode == 0x00) /* extended opcode (:656-660) */
        {
            uint8_t e = rbyte(page, o);
            o += 1;
            ext = 1;
            ore = e & 7;
            ocode = (e >> 3) & 31;
        }
        else
        {
            ore = opcode & 7;
            ocode = (opcode >> 3) & 31;
        }

        d.b0 = top;
        d.b1 = reg;
        d.b2 = siz;
        d.b3 = (uint8_t)ocode;
        d.b4 = (uint8_t)ore;
        d.b5 = (uint8_t)ext;

        /* MCU_Opcode_MOVG_Immediate (:825-873): indirect/absolute source plus a
         * trailing immediate. ore 6/7 write 1/2 bytes; ore 4/5 subtract 1/2.
         * GT FIXMEs: ore=4 word still reads 1 byte (:847), ore=5 byte still
         * reads 2 (:861) - reproduced verbatim. */
        if (ocode == 0 && (type == OP_INDIRECT || type == OP_ABSOLUTE))
        {
            if (ore == 4 || ore == 6)
                o += 1;
            else if (ore == 5 || ore == 7)
                o += 2;
            if (ore == 4 && siz)
                g_fixme_ore4_word++;
            if (ore == 5 && !siz)
                g_fixme_ore5_byte++;
        }

        valid = general_valid(ocode, ore, ext, type, siz);
    }
    /* opcodes 09/0b/0f/16/17/1b/1f: MCU_Operand_Table -> NotImplemented (:1478
     * etc.); not decoded within the known ISA. */

    d.valid = valid;
    d.len = valid ? (int)(o - off) : 0;
    return d;
}

/* ------------------------------ checker ------------------------------ */

static uint8_t *load_file(const char *path, uint32_t *size)
{
    FILE *f = fopen(path, "rb");
    if (!f)
    {
        fprintf(stderr, "h8dec: cannot open %s\n", path);
        exit(2);
    }
    if (fseek(f, 0, SEEK_END) != 0)
    {
        fprintf(stderr, "h8dec: cannot seek %s\n", path);
        exit(2);
    }
    long n = ftell(f);
    if (n < 0)
    {
        fprintf(stderr, "h8dec: cannot size %s\n", path);
        exit(2);
    }
    rewind(f);
    uint8_t *buf = (uint8_t *)malloc((size_t)n ? (size_t)n : 1);
    if (!buf || fread(buf, 1, (size_t)n, f) != (size_t)n)
    {
        fprintf(stderr, "h8dec: cannot read %s\n", path);
        exit(2);
    }
    fclose(f);
    *size = (uint32_t)n;
    return buf;
}

static long check_file(const char *path, long *checked, long *mismatches)
{
    FILE *f = fopen(path, "r");
    if (!f)
    {
        fprintf(stderr, "h8dec: cannot open %s\n", path);
        exit(2);
    }
    char line[256];
    long lines = 0;
    while (fgets(line, sizeof line, f))
    {
        unsigned pc, tpage_m, toff_m, top_m;
        int len_m, kind_m, reg_m, siz_m, ocode_m, ore_m, ext_m;
        if (sscanf(line, "%x %d %d %x %x %x %d %d %d %d %d",
                   &pc, &len_m, &kind_m, &tpage_m, &toff_m, &top_m,
                   &reg_m, &siz_m, &ocode_m, &ore_m, &ext_m) != 11)
            continue;
        lines++;

        h8dec_t d = h8dec(pc);
        (*checked)++;

        struct { const char *name; long m, g; } c[11];
        int n = 0;
        c[n].name = "len";   c[n].m = len_m;             c[n].g = d.len;  n++;
        c[n].name = "kind";  c[n].m = kind_m;            c[n].g = d.kind; n++;
        c[n].name = "tpage"; c[n].m = (long)tpage_m;     c[n].g = d.tpage; n++;
        c[n].name = "toff";  c[n].m = (long)toff_m;      c[n].g = d.toff; n++;
        c[n].name = "top";   c[n].m = (long)top_m;       c[n].g = d.b0;   n++;
        c[n].name = "reg";   c[n].m = reg_m;             c[n].g = d.b1;   n++;
        c[n].name = "siz";   c[n].m = siz_m;             c[n].g = d.b2;   n++;
        c[n].name = "ocode"; c[n].m = ocode_m;           c[n].g = d.b3;   n++;
        c[n].name = "ore";   c[n].m = ore_m;             c[n].g = d.b4;   n++;
        c[n].name = "ext";   c[n].m = ext_m;             c[n].g = d.b5;   n++;

        int bad = 0;
        for (int i = 0; i < n; i++)
        {
            if (c[i].m != c[i].g)
            {
                if (!bad)
                    printf("MISMATCH pc=%08x\n", pc);
                printf("  %-5s mach=%ld h8dec=%ld\n", c[i].name, c[i].m, c[i].g);
                bad = 1;
            }
        }
        if (bad)
            (*mismatches)++;
    }
    fclose(f);
    return lines;
}

int main(int argc, char **argv)
{
    const char *mach_main = NULL, *mach_vec = NULL;
    const char *rom1_path = "build/rom1.bin";
    const char *rom2_path = "build/rom2.bin";
    int have_check = 0;

    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "--check-decode"))
        {
            if (i + 2 >= argc)
            {
                fprintf(stderr, "h8dec: --check-decode needs <mach_main> <mach_vec>\n");
                return 2;
            }
            mach_main = argv[++i];
            mach_vec = argv[++i];
            have_check = 1;
        }
        else if (!strcmp(argv[i], "--rom1") && i + 1 < argc)
            rom1_path = argv[++i];
        else if (!strcmp(argv[i], "--rom2") && i + 1 < argc)
            rom2_path = argv[++i];
        else
        {
            fprintf(stderr, "usage: h8dec --check-decode <mach_main.txt> <mach_vec.txt> "
                            "[--rom1 <rom1.bin> --rom2 <rom2.bin>]\n");
            return 2;
        }
    }
    if (!have_check)
    {
        fprintf(stderr, "usage: h8dec --check-decode <mach_main.txt> <mach_vec.txt> "
                        "[--rom1 <rom1.bin> --rom2 <rom2.bin>]\n");
        return 2;
    }

    uint32_t rom1_size = 0, rom2_size = 0;
    uint8_t *rom1 = load_file(rom1_path, &rom1_size);
    uint8_t *rom2 = load_file(rom2_path, &rom2_size);
    h8dec_set_rom(rom1, rom1_size, rom2, rom2_size);
    fprintf(stderr, "h8dec: rom1=%s (%u bytes) rom2=%s (%u bytes)\n",
            rom1_path, rom1_size, rom2_path, rom2_size);

    long checked = 0, mismatches = 0;
    long n1 = check_file(mach_main, &checked, &mismatches);
    long n2 = check_file(mach_vec, &checked, &mismatches);
    printf("checked=%ld mismatches=%ld\n", checked, mismatches);

    fprintf(stderr, "h8dec: %s lines=%ld, %s lines=%ld\n", mach_main, n1, mach_vec, n2);
    if (g_bank_divergent)
        fprintf(stderr,
            "h8dec: note: %ld PCs on pages GT backs with SRAM/PCM/ext RAM "
            "(src/mcu.cpp:663-854), not rom2; h8dasm.c rbyte maps them to rom2\n",
            g_bank_divergent);
    if (g_fixme_ore4_word || g_fixme_ore5_byte)
        fprintf(stderr,
            "h8dec: note: GT FIXME tails decoded (src/mcu_opcodes.cpp:847/:861): "
            "ore=4 word=%ld (1 imm byte), ore=5 byte=%ld (2 imm bytes)\n",
            g_fixme_ore4_word, g_fixme_ore5_byte);

    free(rom1);
    free(rom2);
    return mismatches ? 1 : 0;
}
