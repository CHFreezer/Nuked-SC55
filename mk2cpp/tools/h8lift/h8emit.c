/*
 * h8emit.c -- H8/532 ROM -> C++ emitter for the mk2cpp project.
 *
 * Usage:
 *   h8emit.exe <rom1.bin> <rom2.bin> <map.csv> <outdir> [--only <pc-list.txt>]
 *
 * Input map.csv schema (see mk2cpp/out/h8part/map.csv):
 *   flat,rom,fileoff,len,kind,cat,block_fn,block_start,rel
 *
 * Outputs in <outdir>:
 *   mk2c_r1.cpp         one function per flat < 0x40000 (rom1/cp0 PCs)
 *   mk2c_r2.cpp         one function per flat >= 0x40000 (rom2/cp4 PCs)
 *   mk2cpp_gen_init.cpp void MK2CPP_FillTables(void) registering every fn
 *
 * Every emitted function runs exactly ONE instruction and is a 1:1
 * transcription of the ground-truth handler in src/mcu_opcodes.cpp /
 * src/mcu.h, through the GT helpers only.  Operand bytes are fetched at
 * runtime with MCU_ReadCodeAdvance() in the same order as GT so pc and any
 * IO side effects match.  Decode-time constants (addressing mode, size,
 * register numbers, opcode/ore) are folded by this emitter; the semantics
 * are never "improved" over GT.
 *
 * Row order: sorted by flat; output contains no timestamps, so reruns are
 * byte-identical for the same inputs.
 *
 * Emitted "ABI subset": the generated units carry a verbatim copy of the
 * needed declarations/inline helpers from src/mcu.h (SDL_atomic.h is not on
 * the generated units' include path: only -I src -I mk2cpp/include is used
 * for the syntax check).  The non-inline helpers (MCU_Read*, MCU_ADD/SUB/
 * SetStatusCommon, MCU_ErrorTrap, MCU_Interrupt_*) resolve to the real GT
 * symbols.  No memory array is touched directly.
 *
 * Build (h8dec.c self-test main must be renamed, as h8part does):
 *   clang -O2 -Wno-deprecated-declarations -Dmain=h8dec_selftest_main \
 *         -c mk2cpp/tools/h8lift/h8dec.c -o h8dec_emit.o
 *   clang -O2 -Wno-deprecated-declarations -o mk2cpp/tools/h8lift/h8emit.exe \
 *         mk2cpp/tools/h8lift/h8emit.c h8dec_emit.o
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
#define H8EMIT_MKDIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define H8EMIT_MKDIR(p) mkdir((p), 0755)
#endif

#define H8EMIT_VERSION "h8emit 0.1.0"

/* ====================================================================== */
/* SHA-256 (public-domain style compact implementation)                    */
/* ====================================================================== */

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
    uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, m[64];
    for (i = 0, j = 0; i < 16; i++, j += 4)
        m[i] = ((uint32_t)blk[j] << 24) | ((uint32_t)blk[j + 1] << 16) |
               ((uint32_t)blk[j + 2] << 8) | (uint32_t)blk[j + 3];
    for (; i < 64; i++)
        m[i] = (SHA_ROTR(m[i - 2], 17) ^ SHA_ROTR(m[i - 2], 19) ^
                (m[i - 2] >> 10)) + m[i - 7] +
               (SHA_ROTR(m[i - 15], 7) ^ SHA_ROTR(m[i - 15], 18) ^
                (m[i - 15] >> 3)) + m[i - 16];
    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2];
    d = ctx->state[3]; e = ctx->state[4]; f = ctx->state[5];
    g = ctx->state[6]; h = ctx->state[7];
    for (i = 0; i < 64; i++) {
        t1 = h + (SHA_ROTR(e, 6) ^ SHA_ROTR(e, 11) ^ SHA_ROTR(e, 25)) +
             ((e & f) ^ (~e & g)) + sha256_k[i] + m[i];
        t2 = (SHA_ROTR(a, 2) ^ SHA_ROTR(a, 13) ^ SHA_ROTR(a, 22)) +
             ((a & b) ^ (a & c) ^ (b & c));
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }
    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c;
    ctx->state[3] += d; ctx->state[4] += e; ctx->state[5] += f;
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

/* Known-answer self-test: the digest goes into the generated headers as
 * ROM/fixture binding metadata, so a broken implementation must abort. */
static void sha256_selftest(void)
{
    static const char *const EMPTY_EXPECT =
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
    static const char *const ABC_EXPECT =
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
    char h[65];
    sha256_hex(NULL, 0, h);
    if (strcmp(h, EMPTY_EXPECT) != 0) {
        fprintf(stderr, "h8emit: sha256 self-test failed (empty): got %s\n", h);
        exit(2);
    }
    sha256_hex((const uint8_t *)"abc", 3, h);
    if (strcmp(h, ABC_EXPECT) != 0) {
        fprintf(stderr, "h8emit: sha256 self-test failed (abc): got %s\n", h);
        exit(2);
    }
}

/* ====================================================================== */
/* ROM access (same bank rules as h8dec/h8part)                            */
/* ====================================================================== */

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

/* ====================================================================== */
/* Growable string buffer                                                  */
/* ====================================================================== */

typedef struct {
    char *p;
    size_t n, cap;
} sb_t;

static void sb_reserve(sb_t *b, size_t extra)
{
    if (b->n + extra + 1 > b->cap) {
        size_t nc = b->cap ? b->cap : 65536;
        while (nc < b->n + extra + 1)
            nc *= 2;
        b->p = (char *)realloc(b->p, nc);
        if (!b->p) {
            fprintf(stderr, "h8emit: out of memory\n");
            exit(2);
        }
        b->cap = nc;
    }
}

static void sb_putn(sb_t *b, const char *s, size_t n)
{
    if (!n)
        return;
    sb_reserve(b, n);
    memcpy(b->p + b->n, s, n);
    b->n += n;
    b->p[b->n] = 0;
}

static void sb_puts(sb_t *b, const char *s)
{
    sb_putn(b, s, strlen(s));
}

static void sb_printf(sb_t *b, const char *fmt, ...)
{
    char tmp[1024];
    va_list ap;
    int n;
    va_start(ap, fmt);
    n = vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (n < 0) {
        fprintf(stderr, "h8emit: vsnprintf failed\n");
        exit(2);
    }
    if ((size_t)n < sizeof tmp) {
        sb_putn(b, tmp, (size_t)n);
        return;
    }
    {
        char *big = (char *)malloc((size_t)n + 1);
        if (!big) {
            fprintf(stderr, "h8emit: out of memory\n");
            exit(2);
        }
        va_start(ap, fmt);
        vsnprintf(big, (size_t)n + 1, fmt, ap);
        va_end(ap);
        sb_putn(b, big, (size_t)n);
        free(big);
    }
}

/* ====================================================================== */
/* Emission context                                                        */
/* ====================================================================== */

enum {
    OT_DIRECT = 0,
    OT_INDIRECT = 1,
    OT_ABSOLUTE = 2,
    OT_IMMEDIATE = 3
};

typedef struct {
    sb_t *b;
    uint32_t flat;
    uint8_t op0;
    int b0, b1, b2, b3, b4, b5; /* h8dec decoded fields */
    int siz;                    /* 0 byte, 1 word */
    int reg;                    /* operand register (op0 & 7) */
    int ocode;                  /* op2 >> 3 */
    int ore;                    /* op2 & 7 */
    int ext;                    /* 0x00 extended prefix seen */
    int type;                   /* OT_* */
    int handled;
} ectx_t;

static long g_todo_count;
static long g_stub_count;
static long g_emitted_count;

static void emit(ectx_t *e, const char *fmt, ...)
{
    char tmp[1024];
    va_list ap;
    int n;
    va_start(ap, fmt);
    n = vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (n < 0) {
        fprintf(stderr, "h8emit: vsnprintf failed\n");
        exit(2);
    }
    sb_puts(e->b, "    ");
    if ((size_t)n < sizeof tmp) {
        sb_putn(e->b, tmp, (size_t)n);
    } else {
        char *big = (char *)malloc((size_t)n + 1);
        if (!big) {
            fprintf(stderr, "h8emit: out of memory\n");
            exit(2);
        }
        va_start(ap, fmt);
        vsnprintf(big, (size_t)n + 1, fmt, ap);
        va_end(ap);
        sb_putn(e->b, big, (size_t)n);
        free(big);
    }
    sb_puts(e->b, "\n");
}

static void emit_todo(ectx_t *e, const char *note)
{
    emit(e, "// TODO(gt): %s", note);
    g_todo_count++;
    printf("TODO(gt) flat=%08x note=%s\n", e->flat, note);
}

/* ====================================================================== */
/* Generated ABI preamble (verbatim subset of src/mcu.h)                   */
/* ====================================================================== */

static const char *ABI_PRELUDE =
"#include <stdint.h>\n"
"#include \"mcu.h\"\n"
"#include \"mcu_interrupt.h\"\n"
"#include \"mk2cpp.h\"\n"
"\n"
"int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);\n"
"int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);\n"
"void MCU_SetStatusCommon(uint32_t val, uint32_t siz);\n"
"extern uint32_t mk2c_emit_unhandled;\n";

/* ====================================================================== */
/* Operand read/write transcription (src/mcu_opcodes.cpp:486-541)          */
/* ====================================================================== */

static void emit_oread(ectx_t *e, const char *var)
{
    switch (e->type) {
    case OT_DIRECT:
        if (e->siz)
            emit(e, "uint32_t %s = (uint32_t)mcu.r[%d];", var, e->reg);
        else
            emit(e, "uint32_t %s = (uint32_t)(mcu.r[%d] & 0xff);", var, e->reg);
        break;
    case OT_INDIRECT:
    case OT_ABSOLUTE:
        if (e->siz) {
            emit(e, "if (oea & 1)");
            emit(e, "    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);");
            emit(e, "uint32_t %s = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));", var);
        } else {
            emit(e, "uint32_t %s = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));", var);
        }
        break;
    default: /* OT_IMMEDIATE */
        emit(e, "uint32_t %s = odata;", var);
        break;
    }
}

static void emit_owrite(ectx_t *e, const char *expr)
{
    switch (e->type) {
    case OT_DIRECT:
        if (e->siz)
            emit(e, "mcu.r[%d] = (uint16_t)(%s);", e->reg, expr);
        else
            emit(e, "mcu.r[%d] = (uint16_t)((mcu.r[%d] & 0xff00u) | ((uint32_t)(%s) & 0xffu));",
                 e->reg, e->reg, expr);
        break;
    case OT_INDIRECT:
    case OT_ABSOLUTE:
        if (e->siz) {
            emit(e, "if (oea & 1)");
            emit(e, "    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);");
            emit(e, "MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(%s));", expr);
        } else {
            emit(e, "MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(%s));", expr);
        }
        break;
    default: /* OT_IMMEDIATE */
        emit(e, "MCU_Interrupt_Exception(EXCEPTION_SOURCE_INVALID_INSTRUCTION);");
        break;
    }
}

static void emit_trap(ectx_t *e)
{
    emit(e, "MCU_ErrorTrap();");
}

/* ====================================================================== */
/* General opcode handler (MCU_Opcode_Table, src/mcu_opcodes.cpp)          */
/* ====================================================================== */

static void gen_handler(ectx_t *e)
{
    int S = e->siz;
    int R = e->b1;
    int ORE = e->ore;
    int OC = e->ocode;
    int mem = (e->type == OT_INDIRECT || e->type == OT_ABSOLUTE);

    switch (OC) {
    case 0: /* MOVG_Immediate :825 */
        if (ORE == 6 && mem) {
            emit(e, "uint32_t d = (uint32_t)(int8_t)MCU_ReadCodeAdvance();");
            emit_owrite(e, "d");
            emit(e, "MCU_SetStatusCommon(d, %d);", S);
        } else if (ORE == 7 && mem) {
            emit(e, "uint32_t d = (uint32_t)MCU_ReadCodeAdvance();");
            emit(e, "d = (d << 8) | (uint32_t)MCU_ReadCodeAdvance();");
            emit_owrite(e, "d");
            emit(e, "MCU_SetStatusCommon(d, %d);", S);
        } else if (ORE == 4 && mem && !S) {
            emit_oread(e, "t1");
            emit(e, "uint32_t t2 = (uint32_t)MCU_ReadCodeAdvance();");
            emit(e, "MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);");
        } else if (ORE == 4 && mem && S) {
            emit_todo(e, "MOVG_Immediate ore=4 word: GT reads one imm8 sign-extended (src/mcu_opcodes.cpp:847 FIXME)");
            emit_oread(e, "t1");
            emit(e, "uint32_t t2 = (uint16_t)(int8_t)MCU_ReadCodeAdvance();");
            emit(e, "MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);");
        } else if (ORE == 5 && mem && S) {
            emit_oread(e, "t1");
            emit(e, "uint32_t t2 = (uint32_t)MCU_ReadCodeAdvance();");
            emit(e, "t2 = (t2 << 8) | (uint32_t)MCU_ReadCodeAdvance();");
            emit(e, "MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);");
        } else if (ORE == 5 && mem && !S) {
            emit_todo(e, "MOVG_Immediate ore=5 byte: GT reads a two-byte imm16 (src/mcu_opcodes.cpp:861 FIXME)");
            emit_oread(e, "t1");
            emit(e, "uint32_t t2 = (uint32_t)MCU_ReadCodeAdvance();");
            emit(e, "t2 = (t2 << 8) | (uint32_t)MCU_ReadCodeAdvance();");
            emit(e, "MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 0);");
        } else {
            emit_trap(e);
        }
        break;

    case 1: { /* ADDQ :1141 */
        int val = 0;
        int ok = 1;
        if (ORE == 0) val = 1;
        else if (ORE == 1) val = 2;
        else if (ORE == 4) val = -1;
        else if (ORE == 5) val = -2;
        else ok = 0;
        emit_oread(e, "t1");
        emit(e, "int32_t t2 = %d;", val);
        if (!ok)
            emit_trap(e);
        emit(e, "t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, t2, 0, %d);", S);
        emit_owrite(e, "t1");
        break;
    }

    case 2: /* CLR/TST/EXTU/SWAP/NOT/NEG/EXTS :937 */
        if (ORE == 3 && e->type != OT_IMMEDIATE) {
            emit_owrite(e, "0");
            emit(e, "MCU_SetStatus(0, STATUS_N);");
            emit(e, "MCU_SetStatus(1, STATUS_Z);");
            emit(e, "MCU_SetStatus(0, STATUS_V);");
            emit(e, "MCU_SetStatus(0, STATUS_C);");
        } else if (ORE == 6 && e->type != OT_IMMEDIATE) {
            emit_oread(e, "data");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
            emit(e, "MCU_SetStatus(0, STATUS_C);");
        } else if (ORE == 2 && e->type == OT_DIRECT && !S) {
            emit(e, "uint32_t data = (uint32_t)(mcu.r[%d] & 0xff);", R);
            emit(e, "mcu.r[%d] = (uint16_t)data;", R);
            emit(e, "MCU_SetStatus(0, STATUS_N);");
            emit(e, "MCU_SetStatus(data == 0, STATUS_Z);");
            emit(e, "MCU_SetStatus(0, STATUS_V);");
            emit(e, "MCU_SetStatus(0, STATUS_C);");
        } else if (ORE == 0 && e->type == OT_DIRECT && !S) {
            emit(e, "uint32_t data = (uint32_t)mcu.r[%d];", R);
            emit(e, "uint32_t data_h = data >> 8;");
            emit(e, "uint32_t data_l = data & 0xff;");
            emit(e, "data = (data_l << 8) | data_h;");
            emit(e, "mcu.r[%d] = (uint16_t)data;", R);
            emit(e, "MCU_SetStatusCommon(data, 1);");
        } else if (ORE == 5 && e->type != OT_IMMEDIATE) {
            emit_oread(e, "data");
            emit(e, "data = ~data;");
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        } else if (ORE == 4 && e->type != OT_IMMEDIATE) {
            emit_oread(e, "data");
            emit(e, "data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, %d);", S);
            emit_owrite(e, "data");
        } else if (ORE == 1 && e->type == OT_DIRECT && !S) {
            emit(e, "uint32_t data = (uint32_t)mcu.r[%d];", R);
            emit(e, "mcu.r[%d] = (uint16_t)(int8_t)data;", R);
            emit(e, "MCU_SetStatusCommon(data, 1);");
        } else {
            emit_trap(e);
        }
        break;

    case 3: { /* SHLR family :1219 */
        if (e->type == OT_IMMEDIATE) {
            emit_trap(e);
            break;
        }
        emit_oread(e, "data");
        if (ORE == 3) { /* SHLR */
            emit(e, "uint32_t C = data & 1;");
            emit(e, "data >>= 1;");
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatus(C, STATUS_C);");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        } else if (ORE == 2) { /* SHLL */
            emit(e, "uint32_t C = (data & 0x%04xu) != 0;", S ? 0x8000 : 0x80);
            emit(e, "data <<= 1;");
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatus(C, STATUS_C);");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        } else if (ORE == 6) { /* ROTXL */
            emit(e, "uint32_t bit = (mcu.sr & STATUS_C) != 0;");
            emit(e, "uint32_t C = (data & 0x%04xu) != 0;", S ? 0x8000 : 0x80);
            emit(e, "data <<= 1;");
            emit(e, "data |= bit;");
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatus(C, STATUS_C);");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        } else if (ORE == 4) { /* ROTL */
            emit(e, "uint32_t C = (data & 0x%04xu) != 0;", S ? 0x8000 : 0x80);
            emit(e, "data <<= 1;");
            emit(e, "data |= C;");
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatus(C, STATUS_C);");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        } else if (ORE == 0) { /* SHAL (identical to SHLL in GT) */
            emit(e, "uint32_t C = (data & 0x%04xu) != 0;", S ? 0x8000 : 0x80);
            emit(e, "data <<= 1;");
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatus(C, STATUS_C);");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        } else if (ORE == 1) { /* SHAR */
            emit(e, "uint32_t C = data & 1;");
            emit(e, "uint32_t msb = data & 0x%04xu;", S ? 0x8000 : 0x80);
            if (S)
                emit(e, "data &= 0xffff;");
            else
                emit(e, "data &= 0xff;");
            emit(e, "data >>= 1;");
            emit(e, "data |= msb;");
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatus(C, STATUS_C);");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        } else if (ORE == 5) { /* ROTR */
            emit(e, "uint32_t C = (data & 1) != 0;");
            emit(e, "data >>= 1;");
            emit(e, "data |= C << %d;", S ? 15 : 7);
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatus(C, STATUS_C);");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        } else {
            emit_trap(e);
        }
        break;
    }

    case 4: /* ADD :1167 */
        emit(e, "int32_t t1 = (int32_t)mcu.r[%d];", ORE);
        emit_oread(e, "t2");
        emit(e, "t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, %d);", S);
        if (S)
            emit(e, "mcu.r[%d] = (uint16_t)t1;", ORE);
        else {
            emit(e, "mcu.r[%d] &= ~0xff;", ORE);
            emit(e, "mcu.r[%d] |= (uint16_t)(t1 & 0xff);", ORE);
        }
        break;

    case 5: /* ADDS :1419 (no flags) */
        emit_oread(e, "data");
        if (!S)
            emit(e, "data = (uint32_t)(int8_t)data;");
        emit(e, "mcu.r[%d] += (uint16_t)data;", ORE);
        break;

    case 6: /* SUB :1181 */
        emit(e, "int32_t t1 = (int32_t)mcu.r[%d];", ORE);
        emit_oread(e, "t2");
        emit(e, "t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, %d);", S);
        if (S)
            emit(e, "mcu.r[%d] = (uint16_t)t1;", ORE);
        else {
            emit(e, "mcu.r[%d] &= ~0xff;", ORE);
            emit(e, "mcu.r[%d] |= (uint16_t)(t1 & 0xff);", ORE);
        }
        break;

    case 7: /* SUBS :1195 (no flags) */
        emit(e, "int32_t t1 = (int32_t)mcu.r[%d];", ORE);
        emit_oread(e, "t2");
        if (S)
            emit(e, "mcu.r[%d] = (uint16_t)(t1 - (int32_t)t2);", ORE);
        else
            emit(e, "mcu.r[%d] = (uint16_t)(t1 - (int8_t)t2);", ORE);
        break;

    case 8: /* OR :1127 */
        emit_oread(e, "data");
        emit(e, "mcu.r[%d] |= (uint16_t)data;", ORE);
        emit(e, "MCU_SetStatusCommon(mcu.r[%d], %d);", ORE, S);
        break;

    case 9: /* BSET_ORC :875 */
        if (e->type == OT_IMMEDIATE) {
            emit_oread(e, "data");
            emit(e, "uint32_t val = MCU_ControlRegisterRead(%d, %d);", ORE, S);
            emit(e, "val |= data;");
            emit(e, "MCU_ControlRegisterWrite(%d, %d, val);", ORE, S);
            if (ORE >= 2)
                emit(e, "MCU_SetStatusCommon(val, %d);", S);
            emit(e, "mcu.ex_ignore = 1;");
        } else {
            emit_oread(e, "data");
            emit(e, "uint32_t bit = (uint32_t)(mcu.r[%d] & 0x0f);", ORE);
            emit(e, "MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);");
            emit(e, "data |= 1u << bit;");
            emit_owrite(e, "data");
        }
        break;

    case 10: /* AND :1205 */
        emit(e, "uint32_t data = (uint32_t)mcu.r[%d];", ORE);
        emit_oread(e, "t2");
        emit(e, "data &= t2;");
        if (S)
            emit(e, "mcu.r[%d] = (uint16_t)data;", ORE);
        else {
            emit(e, "mcu.r[%d] &= ~0xff;", ORE);
            emit(e, "mcu.r[%d] |= (uint16_t)(data & 0xff);", ORE);
        }
        emit(e, "MCU_SetStatusCommon(mcu.r[%d], %d);", ORE, S);
        break;

    case 11: /* BCLR_ANDC :899 */
        if (e->type == OT_IMMEDIATE) {
            emit_oread(e, "data");
            emit(e, "uint32_t val = MCU_ControlRegisterRead(%d, %d);", ORE, S);
            emit(e, "val &= data;");
            emit(e, "MCU_ControlRegisterWrite(%d, %d, val);", ORE, S);
            if (ORE >= 2)
                emit(e, "MCU_SetStatusCommon(val, %d);", S);
            emit(e, "mcu.ex_ignore = 1;");
        } else {
            emit_oread(e, "data");
            emit(e, "uint32_t bit = (uint32_t)(mcu.r[%d] & 0x0f);", ORE);
            emit(e, "MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);");
            emit(e, "data &= ~(1u << bit);");
            emit_owrite(e, "data");
        }
        break;

    case 12: /* XOR :1427 */
        emit_oread(e, "data");
        emit(e, "mcu.r[%d] ^= (uint16_t)data;", ORE);
        emit(e, "MCU_SetStatusCommon(mcu.r[%d], %d);", ORE, S);
        break;

    case 13: /* NotImplemented :820 */
        emit_trap(e);
        break;

    case 14: /* CMP :1134 */
        emit(e, "int32_t t1 = (int32_t)mcu.r[%d];", ORE);
        emit_oread(e, "t2");
        emit(e, "MCU_SUB_Common(t1, (int32_t)t2, 0, %d);", S);
        break;

    case 15: /* BTST :923 */
        if (e->type == OT_IMMEDIATE) {
            emit_trap(e);
        } else {
            emit_oread(e, "data");
            emit(e, "uint32_t bit = (uint32_t)(mcu.r[%d] & 0x0f);", ORE);
            emit(e, "MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);");
        }
        break;

    case 16: /* MOVG read :1041 */
        if (e->ext) {
            emit_todo(e, "MOVG (ocode 16) extended form: GT always traps (src/mcu_opcodes.cpp:1043-1054 FIXME)");
            emit_trap(e);
        } else {
            emit_oread(e, "data");
            if (S)
                emit(e, "mcu.r[%d] = (uint16_t)data;", ORE);
            else {
                emit(e, "mcu.r[%d] &= ~0xff;", ORE);
                emit(e, "mcu.r[%d] |= (uint16_t)(data & 0xff);", ORE);
            }
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        }
        break;

    case 17: /* LDC :996 */
        emit_oread(e, "data");
        emit(e, "MCU_ControlRegisterWrite(%d, %d, data);", ORE, S);
        emit(e, "mcu.ex_ignore = 1;");
        break;

    case 18: /* MOVG write / XCH :1041 */
        if (e->ext) {
            emit_todo(e, "MOVG (ocode 18) extended form: GT always traps (src/mcu_opcodes.cpp:1043-1054 FIXME)");
            emit_trap(e);
        } else if (e->type == OT_DIRECT) {
            if (S) {
                emit(e, "uint32_t r1 = (uint32_t)mcu.r[%d];", ORE);
                emit(e, "uint32_t r2 = (uint32_t)mcu.r[%d];", R);
                emit(e, "mcu.r[%d] = (uint16_t)r2;", ORE);
                emit(e, "mcu.r[%d] = (uint16_t)r1;", R);
            } else {
                emit_trap(e);
            }
        } else {
            emit(e, "uint32_t data = (uint32_t)mcu.r[%d];", ORE);
            emit_owrite(e, "data");
            emit(e, "MCU_SetStatusCommon(data, %d);", S);
        }
        break;

    case 19: /* STC :1003 */
        emit(e, "uint32_t data = MCU_ControlRegisterRead(%d, %d);", ORE, S);
        emit_owrite(e, "data");
        break;

    case 20: /* ADDX :1434 */
        emit(e, "int32_t t1 = (int32_t)mcu.r[%d];", ORE);
        emit_oread(e, "t2");
        emit(e, "int32_t C = (mcu.sr & STATUS_C) != 0;");
        emit(e, "int32_t Z = (mcu.sr & STATUS_Z) != 0;");
        emit(e, "t1 = MCU_ADD_Common(t1, (int32_t)t2, C, %d);", S);
        emit(e, "if (!Z)");
        emit(e, "    MCU_SetStatus(0, STATUS_Z);");
        if (S)
            emit(e, "mcu.r[%d] = (uint16_t)t1;", ORE);
        else {
            emit(e, "mcu.r[%d] &= ~0xff;", ORE);
            emit(e, "mcu.r[%d] |= (uint16_t)(t1 & 0xff);", ORE);
        }
        break;

    case 21: { /* MULXU :1325 */
        emit_oread(e, "t1");
        emit(e, "uint32_t t2 = (uint32_t)mcu.r[%d];", ORE);
        if (!S)
            emit(e, "t2 &= 0xff;");
        emit(e, "t1 *= t2;");
        if (S) {
            int lo = ORE & ~1;
            emit(e, "mcu.r[%d] = (uint16_t)(t1 >> 16);", lo);
            emit(e, "mcu.r[%d] = (uint16_t)t1;", lo | 1);
            emit(e, "uint32_t N = (t1 & 0x80000000u) != 0;");
        } else {
            emit(e, "t1 &= 0xffff;");
            emit(e, "mcu.r[%d] = (uint16_t)t1;", ORE);
            emit(e, "uint32_t N = (t1 & 0x8000u) != 0;");
        }
        emit(e, "uint32_t Z = (t1 == 0);");
        emit(e, "MCU_SetStatus(N, STATUS_N);");
        emit(e, "MCU_SetStatus(Z, STATUS_Z);");
        emit(e, "MCU_SetStatus(0, STATUS_V);");
        emit(e, "MCU_SetStatus(0, STATUS_C);");
        break;
    }

    case 22: /* SUBX :1453 */
        emit(e, "int32_t t1 = (int32_t)mcu.r[%d];", ORE);
        emit_oread(e, "t2");
        emit(e, "int32_t C = (mcu.sr & STATUS_C) != 0;");
        emit(e, "t1 = MCU_SUB_Common(t1, (int32_t)t2, C, %d);", S);
        if (S)
            emit(e, "mcu.r[%d] = (uint16_t)t1;", ORE);
        else {
            emit(e, "mcu.r[%d] &= ~0xff;", ORE);
            emit(e, "mcu.r[%d] |= (uint16_t)(t1 & 0xff);", ORE);
        }
        break;

    case 23: { /* DIVXU :1354 */
        emit_oread(e, "t1");
        emit(e, "uint32_t t2 = 0;");
        emit(e, "uint32_t R = 0, Q = 0;");
        emit(e, "if (t1 == 0)");
        emit(e, "{");
        emit(e, "    MCU_ErrorTrap();");
        emit(e, "    MCU_SetStatus(0, STATUS_N);");
        emit(e, "    MCU_SetStatus(1, STATUS_Z);");
        emit(e, "    MCU_SetStatus(0, STATUS_V);");
        emit(e, "    MCU_SetStatus(0, STATUS_C);");
        emit(e, "    return;");
        emit(e, "}");
        if (S) {
            int lo = ORE & ~1;
            emit(e, "t2 = ((uint32_t)mcu.r[%d] << 16) | (uint32_t)mcu.r[%d];", lo, lo | 1);
            emit(e, "R = t2 %% t1;");
            emit(e, "Q = t2 / t1;");
            emit(e, "if (Q > 0xffffu)");
            emit(e, "{");
            emit(e, "    MCU_SetStatus(0, STATUS_N);");
            emit(e, "    MCU_SetStatus(0, STATUS_Z);");
            emit(e, "    MCU_SetStatus(1, STATUS_V);");
            emit(e, "    MCU_SetStatus(0, STATUS_C);");
            emit(e, "}");
            emit(e, "else");
            emit(e, "{");
            emit(e, "    mcu.r[%d] = (uint16_t)R;", lo);
            emit(e, "    mcu.r[%d] = (uint16_t)Q;", lo | 1);
            emit(e, "    MCU_SetStatusCommon(Q, 1);");
            emit(e, "    MCU_SetStatus(0, STATUS_C);");
            emit(e, "}");
        } else {
            emit(e, "t2 = (uint32_t)mcu.r[%d];", ORE);
            emit(e, "R = t2 %% t1;");
            emit(e, "Q = t2 / t1;");
            emit(e, "if (Q > 0xffu)");
            emit(e, "{");
            emit(e, "    MCU_SetStatus(0, STATUS_N);");
            emit(e, "    MCU_SetStatus(0, STATUS_Z);");
            emit(e, "    MCU_SetStatus(1, STATUS_V);");
            emit(e, "    MCU_SetStatus(0, STATUS_C);");
            emit(e, "}");
            emit(e, "else");
            emit(e, "{");
            emit(e, "    R &= 0xff;");
            emit(e, "    Q &= 0xff;");
            emit(e, "    mcu.r[%d] = (uint16_t)((R << 8) | Q);", ORE);
            emit(e, "    MCU_SetStatusCommon(Q, 0);");
            emit(e, "    MCU_SetStatus(0, STATUS_C);");
            emit(e, "}");
        }
        break;
    }

    case 24: case 25: /* BSET :1009 (bit number not register) */
        if (e->type == OT_IMMEDIATE) {
            emit_trap(e);
        } else {
            emit_oread(e, "data");
            emit(e, "uint32_t bit = %d;", ORE | ((OC & 1) << 3));
            emit(e, "MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);");
            emit(e, "data |= 1u << bit;");
            emit_owrite(e, "data");
        }
        break;

    case 26: case 27: /* BCLR :1025 */
        if (e->type == OT_IMMEDIATE) {
            emit_trap(e);
        } else {
            emit_oread(e, "data");
            emit(e, "uint32_t bit = %d;", ORE | ((OC & 1) << 3));
            emit(e, "MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);");
            emit(e, "data &= ~(1u << bit);");
            emit_owrite(e, "data");
        }
        break;

    case 28: case 29: /* BNOTI :1111 */
        if (e->type == OT_IMMEDIATE) {
            emit_trap(e);
        } else {
            emit_oread(e, "data");
            emit(e, "uint32_t bit = %d;", ORE | ((OC & 1) << 3));
            emit(e, "MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);");
            emit(e, "data ^= 1u << bit;");
            emit_owrite(e, "data");
        }
        break;

    case 30: case 31: /* BTSTI :1097 */
        if (e->type == OT_IMMEDIATE) {
            emit_trap(e);
        } else {
            emit_oread(e, "data");
            emit(e, "uint32_t bit = %d;", ORE | ((OC & 1) << 3));
            emit(e, "MCU_SetStatus((data & (1u << bit)) == 0, STATUS_Z);");
        }
        break;

    default:
        emit_trap(e);
        break;
    }
}

/* ====================================================================== */
/* MCU_Operand_General (src/mcu_opcodes.cpp:543-673) + handler call        */
/* ====================================================================== */

static void gen_general(ectx_t *e)
{
    int top = e->b0;
    int r = e->b1;
    int S = e->siz;
    int step;

    if (top == 0xa0) {
        e->type = OT_DIRECT;
    } else if (top == 0xd0) {
        e->type = OT_INDIRECT;
        emit(e, "uint32_t oea = (uint32_t)mcu.r[%d];", r);
        emit(e, "oea &= 0xffff;");
        emit(e, "uint32_t oep = (uint32_t)(MCU_GetPageForRegister(%d) & 0xff);", r);
    } else if (top == 0xe0) {
        e->type = OT_INDIRECT;
        emit(e, "uint32_t oea = (uint32_t)mcu.r[%d] + (uint32_t)(int8_t)MCU_ReadCodeAdvance();", r);
        emit(e, "oea &= 0xffff;");
        emit(e, "uint32_t oep = (uint32_t)(MCU_GetPageForRegister(%d) & 0xff);", r);
    } else if (top == 0xf0) {
        e->type = OT_INDIRECT;
        emit(e, "uint32_t odisp = (uint32_t)MCU_ReadCodeAdvance();");
        emit(e, "odisp = (odisp << 8) | (uint32_t)MCU_ReadCodeAdvance();");
        emit(e, "uint32_t oea = (uint32_t)mcu.r[%d] + odisp;", r);
        emit(e, "oea &= 0xffff;");
        emit(e, "uint32_t oep = (uint32_t)(MCU_GetPageForRegister(%d) & 0xff);", r);
    } else if (top == 0xb0) {
        e->type = OT_INDIRECT;
        step = (S || r == 7) ? 2 : 1;
        emit(e, "mcu.r[%d] -= %d;", r, step);
        emit(e, "uint32_t oea = (uint32_t)mcu.r[%d];", r);
        emit(e, "oea &= 0xffff;");
        emit(e, "uint32_t oep = (uint32_t)(MCU_GetPageForRegister(%d) & 0xff);", r);
    } else if (top == 0xc0) {
        e->type = OT_INDIRECT;
        step = (S || r == 7) ? 2 : 1;
        emit(e, "uint32_t oea = (uint32_t)mcu.r[%d];", r);
        emit(e, "mcu.r[%d] += %d;", r, step);
        emit(e, "oea &= 0xffff;");
        emit(e, "uint32_t oep = (uint32_t)(MCU_GetPageForRegister(%d) & 0xff);", r);
    } else if (top == 0x00 && r == 5) {
        e->type = OT_ABSOLUTE;
        emit(e, "uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)MCU_ReadCodeAdvance();");
        emit(e, "oea &= 0xffff;");
        emit(e, "uint32_t oep = 0;");
    } else if (top == 0x00 && r == 4) {
        e->type = OT_IMMEDIATE;
        if (S) {
            emit(e, "uint32_t odata = (uint32_t)MCU_ReadCodeAdvance();");
            emit(e, "odata = (odata << 8) | (uint32_t)MCU_ReadCodeAdvance();");
        } else {
            emit(e, "uint32_t odata = (uint32_t)MCU_ReadCodeAdvance();");
        }
    } else if (top == 0x10 && r == 5) {
        e->type = OT_ABSOLUTE;
        emit(e, "uint32_t oea = (uint32_t)MCU_ReadCodeAdvance();");
        emit(e, "oea = (oea << 8) | (uint32_t)MCU_ReadCodeAdvance();");
        emit(e, "oea &= 0xffff;");
        emit(e, "uint32_t oep = (uint32_t)(mcu.dp & 0xff);");
    } else {
        e->handled = 0;
        return;
    }

    emit(e, "uint8_t op2 = MCU_ReadCodeAdvance();");
    emit(e, "if (op2 == 0x00)");
    emit(e, "    op2 = MCU_ReadCodeAdvance();");

    gen_handler(e);
    e->handled = 1;
}

/* ====================================================================== */
/* Short (first-byte dispatched) instruction families                      */
/* ====================================================================== */

static void gen_short(ectx_t *e, uint8_t op0)
{
    /* h8dec only fills b0..b5 for the MCU_Operand_General path; for the
     * first-byte-dispatched families the register/size come straight from
     * the opcode byte (GT: uint32_t reg = opcode & 7, siz = opcode & 8). */
    int S = (op0 & 0x08) != 0;
    int R = op0 & 0x07;

    if (op0 == 0x00) {
        emit(e, "/* nop */");
    } else if (op0 == 0x01 || op0 == 0x06 || op0 == 0x07) {
        emit(e, "uint8_t op2 = MCU_ReadCodeAdvance();");
        emit(e, "uint8_t reg = op2 & 0x07;");
        emit(e, "if ((op2 >> 3) == 0x17)");
        emit(e, "{");
        emit(e, "    uint16_t disp = (uint16_t)(int8_t)MCU_ReadCodeAdvance();");
        if (op0 == 0x01) {
            emit(e, "    mcu.r[reg]--;");
            emit(e, "    if (mcu.r[reg] != 0xffff)");
            emit(e, "        mcu.pc += disp;");
        } else if (op0 == 0x06) {
            emit(e, "    uint32_t Z = (mcu.sr & STATUS_Z) != 0;");
            emit(e, "    if (Z)");
            emit(e, "    {");
            emit(e, "        mcu.r[reg]--;");
            emit(e, "        if (mcu.r[reg] != 0xffff)");
            emit(e, "            mcu.pc += disp;");
            emit(e, "    }");
        } else {
            emit(e, "    uint32_t Z = (mcu.sr & STATUS_Z) != 0;");
            emit(e, "    if (!Z)");
            emit(e, "    {");
            emit(e, "        mcu.r[reg]--;");
            emit(e, "        if (mcu.r[reg] != 0xffff)");
            emit(e, "            mcu.pc += disp;");
            emit(e, "    }");
        }
        emit(e, "}");
        emit(e, "else");
        emit(e, "{");
        emit(e, "    MCU_ErrorTrap();");
        emit(e, "}");
    } else if (op0 == 0x02) { /* LDM */
        emit(e, "uint8_t rlist = MCU_ReadCodeAdvance();");
        emit(e, "for (int i = 0; i < 8; i++)");
        emit(e, "{");
        emit(e, "    if (rlist & (1 << i))");
        emit(e, "    {");
        emit(e, "        uint16_t data = MCU_PopStack();");
        emit(e, "        if (i != 7)");
        emit(e, "            mcu.r[i] = data;");
        emit(e, "    }");
        emit(e, "}");
    } else if (op0 == 0x12) { /* STM */
        emit(e, "uint8_t rlist = MCU_ReadCodeAdvance();");
        emit(e, "for (int i = 7; i >= 0; i--)");
        emit(e, "{");
        emit(e, "    if (rlist & (1 << i))");
        emit(e, "    {");
        emit(e, "        uint16_t data = mcu.r[i];");
        emit(e, "        if (i == 7)");
        emit(e, "            data -= 2;");
        emit(e, "        MCU_PushStack(data);");
        emit(e, "    }");
        emit(e, "}");
    } else if (op0 == 0x08) { /* TRAPA */
        emit(e, "uint8_t opcode = MCU_ReadCodeAdvance();");
        emit(e, "if ((opcode & 0xf0) == 0x10)");
        emit(e, "    MCU_Interrupt_TRAPA(opcode & 0x0f);");
        emit(e, "else");
        emit(e, "    MCU_ErrorTrap();");
    } else if (op0 == 0x03) { /* PJSR */
        emit(e, "uint8_t page = MCU_ReadCodeAdvance();");
        emit(e, "uint16_t address = (uint16_t)(MCU_ReadCodeAdvance() << 8);");
        emit(e, "address |= MCU_ReadCodeAdvance();");
        emit(e, "MCU_PushStack(mcu.pc);");
        emit(e, "MCU_PushStack(mcu.cp);");
        emit(e, "mcu.cp = page;");
        emit(e, "if (mcu.cp == 0x27)");
        emit(e, "    mcu.cp += 0; /* GT dead branch, src/mcu_opcodes.cpp:209 */");
        emit(e, "mcu.pc = address;");
    } else if (op0 == 0x13) { /* PJMP */
        emit(e, "uint8_t page = MCU_ReadCodeAdvance();");
        emit(e, "uint16_t address = (uint16_t)(MCU_ReadCodeAdvance() << 8);");
        emit(e, "address |= MCU_ReadCodeAdvance();");
        emit(e, "mcu.cp = page;");
        emit(e, "mcu.pc = address;");
    } else if (op0 == 0x10) { /* JMP #abs16 */
        emit(e, "uint16_t address = (uint16_t)(MCU_ReadCodeAdvance() << 8);");
        emit(e, "address |= MCU_ReadCodeAdvance();");
        emit(e, "mcu.pc = address;");
    } else if (op0 == 0x18) { /* JSR #abs16 */
        emit(e, "uint16_t address = (uint16_t)(MCU_ReadCodeAdvance() << 8);");
        emit(e, "address |= MCU_ReadCodeAdvance();");
        emit(e, "MCU_PushStack(mcu.pc);");
        emit(e, "mcu.pc = address;");
    } else if (op0 == 0x0e) { /* BSR disp8 */
        emit(e, "uint16_t disp = (uint16_t)(int8_t)MCU_ReadCodeAdvance();");
        emit(e, "MCU_PushStack(mcu.pc);");
        emit(e, "mcu.pc += disp;");
    } else if (op0 == 0x1e) { /* BSR disp16 */
        emit(e, "uint16_t disp = (uint16_t)(MCU_ReadCodeAdvance() << 8);");
        emit(e, "disp |= MCU_ReadCodeAdvance();");
        emit(e, "MCU_PushStack(mcu.pc);");
        emit(e, "mcu.pc += disp;");
    } else if (op0 == 0x19) { /* RTS */
        emit(e, "mcu.pc = MCU_PopStack();");
    } else if (op0 == 0x0a) { /* RTE */
        emit(e, "mcu.sr = MCU_PopStack();");
        emit(e, "mcu.cp = (uint8_t)MCU_PopStack();");
        emit(e, "mcu.pc = MCU_PopStack();");
        emit(e, "mcu.ex_ignore = 1;");
    } else if (op0 == 0x14) { /* RTD */
        emit(e, "int16_t imm = (int8_t)MCU_ReadCodeAdvance();");
        emit(e, "mcu.pc = MCU_PopStack();");
        emit(e, "mcu.r[7] += imm;");
        emit(e, "if (mcu.r[7] & 1)");
        emit(e, "    MCU_ErrorTrap();");
    } else if (op0 == 0x1c) { /* RTD 0x1c: GT TODO */
        emit_todo(e, "RTD 0x1c is unimplemented in GT: reads imm8, pops pc, then traps (src/mcu_opcodes.cpp:327)");
        emit(e, "int16_t imm = (int8_t)MCU_ReadCodeAdvance();");
        emit(e, "(void)imm;");
        emit(e, "mcu.pc = MCU_PopStack();");
        emit(e, "MCU_ErrorTrap();");
    } else if (op0 == 0x1a) { /* SLEEP */
        emit(e, "mcu.sleep = 1;");
    } else if (op0 == 0x11) { /* register-indirect family */
        emit(e, "uint8_t opcode = MCU_ReadCodeAdvance();");
        emit(e, "uint8_t opcode_h = opcode >> 3;");
        emit(e, "uint8_t opcode_l = opcode & 0x07;");
        emit(e, "if (opcode == 0x19)");
        emit(e, "{");
        emit(e, "    mcu.cp = (uint8_t)MCU_PopStack();");
        emit(e, "    mcu.pc = MCU_PopStack();");
        emit(e, "}");
        emit(e, "else if (opcode_h == 0x19)");
        emit(e, "{");
        emit(e, "    MCU_PushStack(mcu.pc);");
        emit(e, "    MCU_PushStack(mcu.cp);");
        emit(e, "    opcode_l &= ~1;");
        emit(e, "    mcu.cp = (uint8_t)(mcu.r[opcode_l] & 0xff);");
        emit(e, "    mcu.pc = mcu.r[opcode_l + 1];");
        emit(e, "}");
        emit(e, "else if (opcode_h == 0x1a)");
        emit(e, "{");
        emit(e, "    mcu.pc = mcu.r[opcode_l];");
        emit(e, "}");
        emit(e, "else if (opcode_h == 0x1b)");
        emit(e, "{");
        emit(e, "    MCU_PushStack(mcu.pc);");
        emit(e, "    mcu.pc = mcu.r[opcode_l];");
        emit(e, "}");
        emit(e, "else");
        emit(e, "{");
        emit(e, "    MCU_ErrorTrap();");
        emit(e, "}");
    } else if (op0 >= 0x20 && op0 <= 0x3f) { /* Bcc */
        int cond = op0 & 0x0f;
        if (op0 & 0x10) {
            emit(e, "uint16_t disp = (uint16_t)(MCU_ReadCodeAdvance() << 8);");
            emit(e, "disp |= MCU_ReadCodeAdvance();");
        } else {
            emit(e, "uint16_t disp = (uint16_t)(int8_t)MCU_ReadCodeAdvance();");
        }
        if (cond == 0 || cond == 1) {
            /* BRA / BRN: no flag reads in GT */
        } else if (cond == 2 || cond == 3) {
            emit(e, "uint32_t C = (mcu.sr & STATUS_C) != 0;");
            emit(e, "uint32_t Z = (mcu.sr & STATUS_Z) != 0;");
        } else if (cond == 4 || cond == 5) {
            emit(e, "uint32_t C = (mcu.sr & STATUS_C) != 0;");
        } else if (cond == 6 || cond == 7) {
            emit(e, "uint32_t Z = (mcu.sr & STATUS_Z) != 0;");
        } else if (cond == 8 || cond == 9) {
            emit(e, "uint32_t V = (mcu.sr & STATUS_V) != 0;");
        } else if (cond == 0xa || cond == 0xb) {
            emit(e, "uint32_t N = (mcu.sr & STATUS_N) != 0;");
        } else if (cond == 0xc || cond == 0xd) {
            emit(e, "uint32_t N = (mcu.sr & STATUS_N) != 0;");
            emit(e, "uint32_t V = (mcu.sr & STATUS_V) != 0;");
        } else {
            emit(e, "uint32_t N = (mcu.sr & STATUS_N) != 0;");
            emit(e, "uint32_t V = (mcu.sr & STATUS_V) != 0;");
            emit(e, "uint32_t Z = (mcu.sr & STATUS_Z) != 0;");
        }
        switch (cond) {
        case 0x0: emit(e, "uint32_t branch = 1;"); break;
        case 0x1: emit(e, "uint32_t branch = 0;"); break;
        case 0x2: emit(e, "uint32_t branch = (C | Z) == 0;"); break;
        case 0x3: emit(e, "uint32_t branch = (C | Z) == 1;"); break;
        case 0x4: emit(e, "uint32_t branch = C == 0;"); break;
        case 0x5: emit(e, "uint32_t branch = C == 1;"); break;
        case 0x6: emit(e, "uint32_t branch = Z == 0;"); break;
        case 0x7: emit(e, "uint32_t branch = Z == 1;"); break;
        case 0x8: emit(e, "uint32_t branch = V == 0;"); break;
        case 0x9: emit(e, "uint32_t branch = V == 1;"); break;
        case 0xa: emit(e, "uint32_t branch = N == 0;"); break;
        case 0xb: emit(e, "uint32_t branch = N == 1;"); break;
        case 0xc: emit(e, "uint32_t branch = (N ^ V) == 0;"); break;
        case 0xd: emit(e, "uint32_t branch = (N ^ V) == 1;"); break;
        case 0xe: emit(e, "uint32_t branch = (Z | (N ^ V)) == 0;"); break;
        default:  emit(e, "uint32_t branch = (Z | (N ^ V)) == 1;"); break;
        }
        emit(e, "if (branch)");
        emit(e, "    mcu.pc += disp;");
    } else if (op0 >= 0x40 && op0 <= 0x4f) { /* CMP r,#imm */
        if (op0 & 0x08) {
            emit(e, "int32_t t2 = (int32_t)MCU_ReadCodeAdvance();");
            emit(e, "t2 = (t2 << 8) | (int32_t)MCU_ReadCodeAdvance();");
        } else {
            emit(e, "int32_t t2 = (int32_t)MCU_ReadCodeAdvance();");
        }
        emit(e, "int32_t t1 = (int32_t)mcu.r[%d];", R);
        emit(e, "MCU_SUB_Common(t1, t2, 0, %d);", S);
    } else if (op0 >= 0x50 && op0 <= 0x57) { /* MOVE */
        emit(e, "uint8_t data = MCU_ReadCodeAdvance();");
        emit(e, "mcu.r[%d] &= ~0xff;", R);
        emit(e, "mcu.r[%d] |= data;", R);
        emit(e, "MCU_SetStatusCommon(data, 0);");
    } else if (op0 >= 0x58 && op0 <= 0x5f) { /* MOVI */
        emit(e, "uint16_t data = (uint16_t)(MCU_ReadCodeAdvance() << 8);");
        emit(e, "data |= MCU_ReadCodeAdvance();");
        emit(e, "mcu.r[%d] = data;", R);
        emit(e, "MCU_SetStatusCommon(data, 1);");
    } else if (op0 >= 0x60 && op0 <= 0x6f) { /* MOVL */
        emit(e, "uint16_t addr = (uint16_t)(mcu.br << 8);");
        emit(e, "addr |= MCU_ReadCodeAdvance();");
        if (S) {
            emit(e, "if (addr & 1)");
            emit(e, "    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);");
            emit(e, "uint16_t data = MCU_Read16(addr);");
            emit(e, "mcu.r[%d] = data;", R);
            emit(e, "MCU_SetStatusCommon(data, 1);");
        } else {
            emit(e, "uint32_t data = (uint32_t)MCU_Read(addr);");
            emit(e, "mcu.r[%d] &= ~0xff;", R);
            emit(e, "mcu.r[%d] |= (uint16_t)data;", R);
            emit(e, "MCU_SetStatusCommon(data, 0);");
        }
    } else if (op0 >= 0x70 && op0 <= 0x7f) { /* MOVS */
        emit(e, "uint16_t addr = (uint16_t)(mcu.br << 8);");
        emit(e, "addr |= MCU_ReadCodeAdvance();");
        if (S) {
            emit(e, "if (addr & 1)");
            emit(e, "    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);");
            emit(e, "uint32_t data = (uint32_t)mcu.r[%d];", R);
            emit(e, "MCU_Write16(addr, (uint16_t)data);");
            emit(e, "MCU_SetStatusCommon(data, 1);");
        } else {
            emit(e, "uint32_t data = (uint32_t)(mcu.r[%d] & 0xff);", R);
            emit(e, "MCU_Write(addr, (uint8_t)data);");
            emit(e, "MCU_SetStatusCommon(data, 0);");
        }
    } else if (op0 >= 0x80 && op0 <= 0x9f) { /* MOVF (GT asymmetric, G6) */
        emit(e, "int8_t disp = (int8_t)MCU_ReadCodeAdvance();");
        emit(e, "uint32_t addr = (uint32_t)((mcu.r[6] + disp) & 0xffff);");
        emit(e, "addr |= (uint32_t)mcu.tp << 16;");
        if ((op0 & 0x10) == 0) {
            if (S) {
                emit(e, "// GT quirk: word read uses r[reg] |= data (16-bit OR), status is byte");
                emit(e, "uint16_t data = MCU_Read16(addr);");
                emit(e, "mcu.r[%d] &= ~0xff;", R);
                emit(e, "mcu.r[%d] |= data;", R);
                emit(e, "MCU_SetStatusCommon(data, 0);");
            } else {
                emit(e, "// GT quirk: byte read overwrites the whole register, status is word");
                emit(e, "uint16_t data = MCU_Read(addr);");
                emit(e, "mcu.r[%d] = data;", R);
                emit(e, "MCU_SetStatusCommon(data, 1);");
            }
        } else {
            if (S) {
                emit(e, "uint16_t data = (uint16_t)(mcu.r[%d] & 0xff);", R);
                emit(e, "MCU_Write(addr, (uint8_t)data);");
                emit(e, "MCU_SetStatusCommon(data, 0);");
            } else {
                emit(e, "uint16_t data = mcu.r[%d];", R);
                emit(e, "MCU_Write16(addr, data);");
                emit(e, "MCU_SetStatusCommon(data, 1);");
            }
        }
    } else {
        e->handled = 0;
        return;
    }
    e->handled = 1;
}

/* ====================================================================== */
/* Map rows                                                                */
/* ====================================================================== */

typedef struct {
    uint32_t flat;
    int is_r1;
    uint32_t fileoff;
    int len;
    int kind;
    char cat[24];
    char fn[160];
} row_t;

static int row_cmp(const void *a, const void *b)
{
    const row_t *x = (const row_t *)a;
    const row_t *y = (const row_t *)b;
    if (x->flat != y->flat)
        return (x->flat > y->flat) - (x->flat < y->flat);
    return strcmp(x->fn, y->fn);
}

static uint8_t *load_file(const char *path, uint32_t *size)
{
    FILE *f = fopen(path, "rb");
    long n;
    uint8_t *buf;
    if (!f) {
        fprintf(stderr, "h8emit: cannot open '%s'\n", path);
        exit(2);
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "h8emit: cannot seek '%s'\n", path);
        exit(2);
    }
    n = ftell(f);
    if (n < 0) {
        fprintf(stderr, "h8emit: cannot size '%s'\n", path);
        exit(2);
    }
    rewind(f);
    buf = (uint8_t *)malloc((size_t)n ? (size_t)n : 1);
    if (!buf || fread(buf, 1, (size_t)n, f) != (size_t)n) {
        fprintf(stderr, "h8emit: cannot read '%s'\n", path);
        exit(2);
    }
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
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else break;
        v = (v << 4) | (uint64_t)d;
        if (v > 0xffffffffu)
            return 0;
        digits++;
        p++;
    }
    if (!digits)
        return 0;
    *out = (uint32_t)v;
    return 1;
}

static row_t *g_rows;
static size_t g_row_count;
static size_t g_row_cap;

static void add_row(const row_t *r)
{
    if (g_row_count == g_row_cap) {
        g_row_cap = g_row_cap ? g_row_cap * 2 : 8192;
        g_rows = (row_t *)realloc(g_rows, g_row_cap * sizeof(row_t));
        if (!g_rows) {
            fprintf(stderr, "h8emit: out of memory\n");
            exit(2);
        }
    }
    g_rows[g_row_count++] = *r;
}

static void read_map(const char *path)
{
    FILE *f = fopen(path, "rb");
    char line[512];
    if (!f) {
        fprintf(stderr, "h8emit: cannot open '%s'\n", path);
        exit(2);
    }
    while (fgets(line, sizeof line, f)) {
        char rom[8];
        char cat[24];
        char fn[160];
        unsigned flat, fileoff, bstart;
        int len, kind, rel;
        int n;
        row_t r;
        if (strncmp(line, "flat,", 5) == 0)
            continue;
        n = sscanf(line, "%x,%7[^,],%x,%d,%d,%23[^,],%159[^,],%x,%d",
                   &flat, rom, &fileoff, &len, &kind, cat, fn, &bstart, &rel);
        (void)bstart;
        (void)rel;
        if (n != 9)
            continue;
        memset(&r, 0, sizeof r);
        r.flat = flat;
        r.is_r1 = (strcmp(rom, "r1") == 0);
        r.fileoff = fileoff;
        r.len = len;
        r.kind = kind;
        strncpy(r.cat, cat, sizeof r.cat - 1);
        strncpy(r.fn, fn, sizeof r.fn - 1);
        add_row(&r);
    }
    fclose(f);
}

static uint32_t *read_pc_list(const char *path, size_t *count)
{
    FILE *f = fopen(path, "rb");
    char line[512];
    uint32_t *pcs = NULL;
    size_t n = 0, cap = 0;
    if (!f) {
        fprintf(stderr, "h8emit: cannot open '%s'\n", path);
        exit(2);
    }
    while (fgets(line, sizeof line, f)) {
        const char *p = line;
        uint32_t v;
        while (*p == ' ' || *p == '\t')
            p++;
        if (!*p || *p == '\r' || *p == '\n' || *p == '#')
            continue;
        if (!parse_hex(p, &v))
            continue;
        if (n == cap) {
            cap = cap ? cap * 2 : 64;
            pcs = (uint32_t *)realloc(pcs, cap * sizeof(uint32_t));
            if (!pcs) {
                fprintf(stderr, "h8emit: out of memory\n");
                exit(2);
            }
        }
        pcs[n++] = v;
    }
    fclose(f);
    *count = n;
    return pcs;
}

/* ====================================================================== */
/* Output helpers                                                          */
/* ====================================================================== */

static void ensure_outdir(const char *dir)
{
    char tmp[1024];
    size_t n = strlen(dir);
    char *p;
    if (n == 0 || n >= sizeof tmp) {
        fprintf(stderr, "h8emit: bad outdir '%s'\n", dir);
        exit(2);
    }
    memcpy(tmp, dir, n + 1);
    for (p = tmp + 1; *p; p++) {
        if (*p == '/' || *p == '\\') {
            char c = *p;
            *p = '\0';
            if (tmp[0])
                (void)H8EMIT_MKDIR(tmp);
            *p = c;
        }
    }
    (void)H8EMIT_MKDIR(tmp);
}

static void write_file(const char *path, const sb_t *b)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "h8emit: cannot write '%s'\n", path);
        exit(2);
    }
    if (b->n && fwrite(b->p, 1, b->n, f) != b->n) {
        fprintf(stderr, "h8emit: short write '%s'\n", path);
        fclose(f);
        exit(2);
    }
    fclose(f);
}

/* ====================================================================== */
/* Function emission                                                       */
/* ====================================================================== */

static void emit_function(sb_t *out, sb_t *init_decls, sb_t *init_regs,
                          const row_t *r)
{
    h8dec_t d = h8dec(r->flat);
    sb_t body;
    ectx_t e;
    int i;

    memset(&body, 0, sizeof body);
    memset(&e, 0, sizeof e);
    e.b = &body;
    e.flat = r->flat;
    e.op0 = rb(r->flat);
    e.b0 = d.b0;
    e.b1 = d.b1;
    e.b2 = d.b2;
    e.b3 = d.b3;
    e.b4 = d.b4;
    e.b5 = d.b5;
    e.siz = d.b2 ? 1 : 0;
    e.reg = d.b1;
    e.ocode = d.b3;
    e.ore = d.b4;
    e.ext = d.b5;

    /* per-function audit header */
    sb_printf(out, "/* %s: flat=%08x %s off=%06x len=%d kind=%d cat=%s\n",
              r->fn, r->flat, r->is_r1 ? "r1" : "r2", r->fileoff, r->len,
              r->kind, r->cat);
    sb_puts(out, " * decoded: top=");
    sb_printf(out, "%02x reg=%d siz=%d ocode=%d ore=%d ext=%d\n",
              d.b0, d.b1, d.b2, d.b3, d.b4, d.b5);
    sb_puts(out, " * bytes:");
    for (i = 0; i < r->len; i++)
        sb_printf(out, " %02x", rb(r->flat + (uint32_t)i));
    sb_puts(out, " */\n");

    if (!d.valid || d.len != r->len) {
        /* Should not happen for map rows (h8part aborts earlier). */
        emit(&e, "/* STUB: h8dec does not validate this row */");
        emit(&e, "mk2c_emit_unhandled++;");
        e.handled = 1;
        g_stub_count++;
        printf("STUB flat=%08x fn=%s reason=decode-invalid-or-len-mismatch\n",
               r->flat, r->fn);
    } else if (r->is_r1 && (r->flat >= 0x40000u)) {
        emit(&e, "/* STUB: emitter page rule violation (r1 flat >= 0x40000) */");
        emit(&e, "mk2c_emit_unhandled++;");
        e.handled = 1;
        g_stub_count++;
        printf("STUB flat=%08x fn=%s reason=page-rule\n", r->flat, r->fn);
    } else {
        /* The host dispatches on flat=(cp<<16)|pc and calls this function
         * INSTEAD of MCU_ReadInstruction(); nothing has fetched the first
         * opcode byte yet, so consume it here (GT mcu.cpp:1078-1080 does
         * MCU_ReadCodeAdvance() before MCU_Operand_Table[operand]). */
        emit(&e, "(void)MCU_ReadCodeAdvance(); /* consume first opcode byte */");
        if (e.op0 >= 0xa0 || e.op0 == 0x04 || e.op0 == 0x05 ||
            e.op0 == 0x0c || e.op0 == 0x0d || e.op0 == 0x15 ||
            e.op0 == 0x1d) {
            gen_general(&e);
        } else {
            gen_short(&e, e.op0);
        }
        if (!e.handled) {
            body.n = 0;
            if (body.p)
                body.p[0] = 0;
            emit(&e, "/* STUB: emitter has no transcription for this decode */");
            emit(&e, "mk2c_emit_unhandled++;");
            e.handled = 1;
            g_stub_count++;
            printf("STUB flat=%08x fn=%s reason=no-transcription op0=%02x\n",
                   r->flat, r->fn, e.op0);
        }
    }

    sb_printf(out, "void %s(void)\n{\n", r->fn);
    sb_putn(out, body.p ? body.p : "", body.n);
    sb_puts(out, "}\n\n");
    free(body.p);

    sb_printf(init_decls, "void %s(void);\n", r->fn);
    sb_printf(init_regs, "    MK2CPP_RegisterFn(0x%08xu, &%s);\n",
              r->flat, r->fn);

    g_emitted_count++;
}

/* ====================================================================== */
/* main                                                                    */
/* ====================================================================== */

static void usage(void)
{
    fprintf(stderr,
        "usage: h8emit.exe <rom1.bin> <rom2.bin> <map.csv> <outdir> "
        "[--only <pc-list.txt>]\n");
}

int main(int argc, char **argv)
{
    const char *rom1_path = NULL;
    const char *rom2_path = NULL;
    const char *map_path = NULL;
    const char *outdir = NULL;
    const char *only_path = NULL;
    const char *pos[4];
    sha256_selftest();
    int npos = 0;
    int i;
    uint32_t *only_pcs = NULL;
    size_t only_count = 0;
    size_t kept, a;
    char sha_rom1[65], sha_rom2[65], sha_map[65];
    char path[1024];
    sb_t r1, r2, init_decls, init_regs, init;
    char map_buf_cat[1024];

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--only") == 0) {
            if (i + 1 >= argc) {
                usage();
                return 2;
            }
            only_path = argv[++i];
        } else if (npos < 4) {
            pos[npos++] = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if (npos != 4) {
        usage();
        return 2;
    }
    rom1_path = pos[0];
    rom2_path = pos[1];
    map_path = pos[2];
    outdir = pos[3];

    g_rom1 = load_file(rom1_path, &g_rom1_size);
    g_rom2 = load_file(rom2_path, &g_rom2_size);
    h8dec_set_rom(g_rom1, g_rom1_size, g_rom2, g_rom2_size);

    read_map(map_path);
    if (g_row_count == 0) {
        fprintf(stderr, "h8emit: no rows in '%s'\n", map_path);
        return 2;
    }

    if (only_path) {
        size_t k;
        only_pcs = read_pc_list(only_path, &only_count);
        if (only_count == 0) {
            fprintf(stderr, "h8emit: no PCs in '%s'\n", only_path);
            return 2;
        }
        kept = 0;
        for (k = 0; k < g_row_count; k++) {
            size_t m;
            int hit = 0;
            for (m = 0; m < only_count; m++) {
                if (only_pcs[m] == g_rows[k].flat) {
                    hit = 1;
                    break;
                }
            }
            if (hit)
                g_rows[kept++] = g_rows[k];
        }
        g_row_count = kept;
        for (k = 0; k < only_count; k++) {
            size_t m;
            int found = 0;
            for (m = 0; m < g_row_count; m++) {
                if (g_rows[m].flat == only_pcs[k]) {
                    found = 1;
                    break;
                }
            }
            if (!found)
                fprintf(stderr,
                        "h8emit: warning: --only PC %08x has no map row\n",
                        only_pcs[k]);
        }
        printf("h8emit: --only kept %lu of %lu map rows\n",
               (unsigned long)g_row_count, (unsigned long)only_count);
    }

    qsort(g_rows, g_row_count, sizeof(row_t), row_cmp);

    sha256_hex(g_rom1, g_rom1_size, sha_rom1);
    sha256_hex(g_rom2, g_rom2_size, sha_rom2);
    {
        uint32_t map_size = 0;
        uint8_t *map_buf = load_file(map_path, &map_size);
        sha256_hex(map_buf, map_size, sha_map);
        free(map_buf);
    }

    memset(&r1, 0, sizeof r1);
    memset(&r2, 0, sizeof r2);
    memset(&init_decls, 0, sizeof init_decls);
    memset(&init_regs, 0, sizeof init_regs);
    memset(&init, 0, sizeof init);

    /* generated file headers */
    snprintf(map_buf_cat, sizeof map_buf_cat, "%s", map_path);
    sb_printf(&r1,
        "// GENERATED by " H8EMIT_VERSION " from ROM sha256 -- DO NOT EDIT\n"
        "// rom1 sha256 %s\n"
        "// rom2 sha256 %s\n"
        "// inputs map=%s map_sha256 %s rows=%lu emitted_at=BUILD (no timestamp)\n"
        "// one function per PC, exactly one GT instruction each; GT helpers only\n\n",
        sha_rom1, sha_rom2, map_buf_cat, sha_map, (unsigned long)g_row_count);
    sb_puts(&r1, ABI_PRELUDE);
    sb_printf(&r2,
        "// GENERATED by " H8EMIT_VERSION " from ROM sha256 -- DO NOT EDIT\n"
        "// rom1 sha256 %s\n"
        "// rom2 sha256 %s\n"
        "// inputs map=%s map_sha256 %s rows=%lu emitted_at=BUILD (no timestamp)\n"
        "// one function per PC, exactly one GT instruction each; GT helpers only\n\n",
        sha_rom1, sha_rom2, map_buf_cat, sha_map, (unsigned long)g_row_count);
    sb_puts(&r2, ABI_PRELUDE);

    for (a = 0; a < g_row_count; a++) {
        const row_t *r = &g_rows[a];
        sb_t *dst = r->is_r1 ? &r1 : &r2;
        emit_function(dst, &init_decls, &init_regs, r);
    }

    sb_printf(&init,
        "// GENERATED by " H8EMIT_VERSION " from ROM sha256 -- DO NOT EDIT\n"
        "// rom1 sha256 %s\n"
        "// rom2 sha256 %s\n"
        "// inputs map=%s map_sha256 %s rows=%lu emitted_at=BUILD (no timestamp)\n"
        "// registers every emitted PC; called by MK2CPP_Init (MK2CPP_HAS_GEN)\n\n"
        "#include \"mk2cpp.h\"\n\n"
        "uint32_t mk2c_emit_unhandled = 0;\n\n",
        sha_rom1, sha_rom2, map_buf_cat, sha_map, (unsigned long)g_row_count);
    sb_putn(&init, init_decls.p ? init_decls.p : "", init_decls.n);
    sb_puts(&init, "\nvoid MK2CPP_FillTables(void)\n{\n");
    sb_putn(&init, init_regs.p ? init_regs.p : "", init_regs.n);
    sb_puts(&init, "}\n");

    ensure_outdir(outdir);
    snprintf(path, sizeof path, "%s/mk2c_r1.cpp", outdir);
    write_file(path, &r1);
    snprintf(path, sizeof path, "%s/mk2c_r2.cpp", outdir);
    write_file(path, &r2);
    snprintf(path, sizeof path, "%s/mk2cpp_gen_init.cpp", outdir);
    write_file(path, &init);

    printf("h8emit: outdir=%s emitted=%ld r1_bytes=%lu r2_bytes=%lu init_bytes=%lu\n",
           outdir, g_emitted_count, (unsigned long)r1.n, (unsigned long)r2.n,
           (unsigned long)init.n);
    printf("h8emit: TODO(gt) instructions=%ld stubs=%ld\n",
           g_todo_count, g_stub_count);
    printf("h8emit: rom1 sha256 %s\n", sha_rom1);
    printf("h8emit: rom2 sha256 %s\n", sha_rom2);
    printf("h8emit: map  sha256 %s\n", sha_map);

    free(r1.p);
    free(r2.p);
    free(init.p);
    free(init_decls.p);
    free(init_regs.p);
    free(g_rows);
    free(only_pcs);
    free(g_rom1);
    free(g_rom2);
    return (g_stub_count != 0) ? 1 : 0;
}
