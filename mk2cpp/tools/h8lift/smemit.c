/*
 * smemit.c -- SM (sub MCU, 6502-like) ROM -> C++ emitter for the mk2cpp project.
 *
 * Usage:
 *   smemit.exe <rom_sm.bin> <outdir> [--extra <pc-list.txt>]
 *
 * Input:
 *   rom_sm.bin -- the 4096-byte sub-MCU firmware (ROM at 0x1000..0x1fff,
 *   vector table of 10 little-endian 16-bit entries at 0x1fec..0x1fff).
 *   --extra   -- optional file with one hex PC (XXXX) per line, unioned into
 *                the translated set (e.g. tools/baselines/pc_sm_251.txt).
 *
 * Outputs in <outdir>:
 *   mk2c_sm.cpp             one function per PC, exactly one GT SM instruction
 *   mk2cpp_gen_sm_init.cpp  void MK2CPP_SM_FillTables(void) registering every fn
 *   map_sm.csv              pc,op,len,fn
 *   sm_pcs.txt              the translated PC set, one XXXX per line, sorted
 *
 * PC set construction (static):
 *   - all 10 vector targets (SM_GetVectorAddress reads them little-endian);
 *   - linear scan from 0x1000 across the whole ROM region (every byte position
 *     reached by sequential flow becomes a candidate PC; instruction lengths
 *     come from the GT opcode table, so the scan never stalls);
 *   - resolved targets of the statically resolvable control transfers
 *     (Bcc/BRA/BBC/BBS disp8, JSR/JMP abs16, JSR page 0xff00|imm8).
 *     Indirect forms (JSR (zp), JMP (abs)/(zp)) depend on RAM at runtime and
 *     are NOT resolved statically; if firmware ever executes a PC the static
 *     set missed, the GT interpreter fallback keeps behavior correct (mixed
 *     mode) and the PC can be added via --extra.
 *
 * Every emitted function runs exactly ONE instruction and is a 1:1
 * transcription of the ground-truth handler in src/submcu.cpp (SM_Opcode_*),
 * through the GT helpers only (SM_ReadAdvance/SM_ReadAdvance16/SM_Read16/
 * SM_Read/SM_Write/SM_PushStack/SM_PopStack/SM_Update_NZ/SM_SetStatus/
 * SM_ErrorTrap).  Operand bytes are fetched at runtime in the same order as
 * GT so sm.pc and shared-RAM read side effects match.  Decode-time constants
 * (operand byte count, addressing mode, bit/test fields) are folded; the
 * semantics are never "improved" over GT, including its FIXME paths
 * (T-flag LDA/ORA/AND, IPCE read clearing).
 *
 * Output is sorted by PC and carries no timestamps, so reruns are
 * byte-identical for the same inputs.  The ROM sha256 goes into the file
 * headers as binding metadata.
 *
 * Build:
 *   clang-cl -O2 -Fe mk2cpp/tools/h8lift/smemit.exe mk2cpp/tools/h8lift/smemit.c
 */

#define _CRT_SECURE_NO_WARNINGS 1

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#define SMIEMT_MKDIR(p) _mkdir(p)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define SMIEMT_MKDIR(p) mkdir((p), 0755)
#endif

#define SMEMIT_VERSION "smemit 0.1.0"

/* ====================================================================== */
/* SHA-256 (same implementation as h8emit.c)                               */
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

static void sha256_selftest(void)
{
    static const char *const EMPTY_EXPECT =
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
    static const char *const ABC_EXPECT =
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
    char h[65];
    sha256_hex(NULL, 0, h);
    if (strcmp(h, EMPTY_EXPECT) != 0) {
        fprintf(stderr, "smemit: sha256 self-test failed (empty): got %s\n", h);
        exit(2);
    }
    sha256_hex((const uint8_t *)"abc", 3, h);
    if (strcmp(h, ABC_EXPECT) != 0) {
        fprintf(stderr, "smemit: sha256 self-test failed (abc): got %s\n", h);
        exit(2);
    }
}

/* ====================================================================== */
/* Growable string buffer (same as h8emit.c)                               */
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
            fprintf(stderr, "smemit: out of memory\n");
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
        fprintf(stderr, "smemit: vsnprintf failed\n");
        exit(2);
    }
    if ((size_t)n < sizeof tmp) {
        sb_putn(b, tmp, (size_t)n);
        return;
    }
    {
        char *big = (char *)malloc((size_t)n + 1);
        if (!big) {
            fprintf(stderr, "smemit: out of memory\n");
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
/* GT SM opcode model (src/submcu.cpp SM_Opcode_Table + handlers)          */
/* ====================================================================== */

static uint8_t g_rom[4096];

/* GENERATED from src/submcu.cpp SM_Opcode_Table by mk2cpp/tools/h8lift/sm_parse.py. DO NOT EDIT. */
static const uint8_t sm_op_impl[256] = {
    0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01,
    0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01,
};

static const uint8_t sm_op_len[256] = {
    0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01,
    0x02, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x02, 0x02, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x02, 0x02, 0x02, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x00, 0x00, 0x02, 0x02, 0x02, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x02, 0x02, 0x02, 0x01,
    0x01, 0x01, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x02, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01,
};
/* Operand byte count per opcode (instruction length = 1 + this).
 * sm_op_impl comes from src/submcu.cpp SM_Opcode_Table (GT implements only
 * 165 of 256 slots; the rest trap). sm_op_len counts the operand bytes each
 * GT handler path actually fetches, cross-checked 165/165 against the complete
 * M37450 addressing-mode table in tools/disasm/smdasm.c (GT alone cannot
 * describe the unimplemented slots). sm_op_len is 0 for NotImplemented slots
 * (never used, since sm_op_impl guards them). No hand-computed hex.
 * Regenerate / verify with:
 *   python mk2cpp/tools/h8lift/sm_parse.py --check mk2cpp/tools/h8lift/smemit.c */
static int sm_op_operand_bytes(uint8_t op)
{
    return sm_op_len[op];
}

/* 1 if the GT table slot is SM_Opcode_NotImplemented (GT: SM_ErrorTrap). */
static int sm_op_not_implemented(uint8_t op)
{
    return sm_op_impl[op] == 0;
}


/* ====================================================================== */
/* Body emission: 1:1 transcription of the GT handlers (src/submcu.cpp)    */
/* ====================================================================== */

static void emit(sb_t *b, const char *fmt, ...)
{
    char tmp[1024];
    va_list ap;
    int n;
    va_start(ap, fmt);
    n = vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (n < 0) {
        fprintf(stderr, "smemit: vsnprintf failed\n");
        exit(2);
    }
    sb_puts(b, "    ");
    if ((size_t)n < sizeof tmp) {
        sb_putn(b, tmp, (size_t)n);
    } else {
        char *big = (char *)malloc((size_t)n + 1);
        if (!big) {
            fprintf(stderr, "smemit: out of memory\n");
            exit(2);
        }
        va_start(ap, fmt);
        vsnprintf(big, (size_t)n + 1, fmt, ap);
        va_end(ap);
        sb_putn(b, big, (size_t)n);
        free(big);
    }
    sb_puts(b, "\n");
}

/* GT: SM_Opcode_LDA/LDX/LDY operand reads, per addressing mode.
 * Returns the C expression that yields the value (reads at runtime). */
static const char *sm_op_load_expr(uint8_t op)
{
    switch (op) {
    case 0xa9: case 0xa2: case 0xa0: return "SM_ReadAdvance()";
    case 0xa5: case 0xa6: case 0xa4: return "SM_Read(SM_ReadAdvance())";
    case 0xb5: return "SM_Read((SM_ReadAdvance() + sm.x) & 0xff)";
    case 0xb6: return "SM_Read((SM_ReadAdvance() + sm.y) & 0xff)";
    case 0xb4: return "SM_Read((SM_ReadAdvance() + sm.x) & 0xff)";
    case 0xad: case 0xae: case 0xac: return "SM_Read(SM_ReadAdvance16())";
    case 0xbd: return "SM_Read(SM_ReadAdvance16() + sm.x)";
    case 0xbe: return "SM_Read(SM_ReadAdvance16() + sm.y)";
    case 0xbc: return "SM_Read(SM_ReadAdvance16() + sm.x)";
    case 0xb9: return "SM_Read(SM_ReadAdvance16() + sm.y)";
    case 0xa1: return "SM_Read(SM_Read16((SM_ReadAdvance() + sm.x) & 0xff))";
    case 0xb1: return "SM_Read(SM_Read16(SM_ReadAdvance()) + sm.y)";
    case 0xc9: case 0xc0: case 0xe0: return "SM_ReadAdvance()";
    case 0xc5: case 0xc4: case 0xe4: return "SM_Read(SM_ReadAdvance())";
    case 0xd5: return "SM_Read((SM_ReadAdvance() + sm.x) & 0xff)";
    case 0xcd: case 0xcc: case 0xec: return "SM_Read(SM_ReadAdvance16())";
    case 0xdd: return "SM_Read(SM_ReadAdvance16() + sm.x)";
    case 0xd9: return "SM_Read(SM_ReadAdvance16() + sm.y)";
    case 0xc1: return "SM_Read(SM_Read16((SM_ReadAdvance() + sm.x) & 0xff))";
    case 0xd1: return "SM_Read(SM_Read16(SM_ReadAdvance()) + sm.y)";
    case 0x09: case 0x29: return "SM_ReadAdvance()";
    case 0x05: case 0x25: return "SM_Read(SM_ReadAdvance())";
    case 0x15: case 0x35: return "SM_Read((SM_ReadAdvance() + sm.x) & 0xff)";
    case 0x0d: case 0x2d: return "SM_Read(SM_ReadAdvance16())";
    case 0x1d: case 0x3d: return "SM_Read(SM_ReadAdvance16() + sm.x)";
    case 0x19: case 0x39: return "SM_Read(SM_ReadAdvance16() + sm.y)";
    case 0x01: case 0x21: return "SM_Read(SM_Read16((SM_ReadAdvance() + sm.x) & 0xff))";
    case 0x11: case 0x31: return "SM_Read(SM_Read16(SM_ReadAdvance()) + sm.y)";
    }
    fprintf(stderr, "smemit: internal: no load expr for op %02x\n", op);
    exit(3);
    return NULL; /* unreachable */
}

static const char *sm_op_store_dest(uint8_t op)
{
    switch (op) {
    /* STA / STX / STY */
    case 0x85: case 0x86: case 0x84: return "SM_ReadAdvance()";
    case 0x95: return "SM_ReadAdvance() + sm.x";
    case 0x96: return "SM_ReadAdvance() + sm.x";
    case 0x94: return "(SM_ReadAdvance() + sm.x) & 0xff";
    case 0x8d: case 0x8e: case 0x8c: return "SM_ReadAdvance16()";
    case 0x9d: return "SM_ReadAdvance16() + sm.x";
    case 0x99: return "SM_ReadAdvance16() + sm.y";
    case 0x81: return "SM_Read16((SM_ReadAdvance() + sm.x) & 0xff)";
    case 0x91: return "SM_Read16(SM_ReadAdvance()) + sm.y";
    /* DEC / INC memory forms (same addressing modes, different opcodes) */
    case 0xc6: case 0xe6: return "SM_ReadAdvance()";
    case 0xd6: case 0xf6: return "(SM_ReadAdvance() + sm.x) & 0xff";
    case 0xce: case 0xee: return "SM_ReadAdvance16()";
    case 0xde: case 0xfe: return "SM_ReadAdvance16() + sm.x";
    }
    fprintf(stderr, "smemit: internal: no store-dest expr for op %02x\n", op);
    exit(3);
    return NULL; /* unreachable */
}

static void emit_body(sb_t *b, uint8_t op)
{
    int zp = (op & 4) != 0;
    int bit = (op >> 5) & 7;
    int type = (op >> 4) & 1;
    const char *expr;

    if (sm_op_not_implemented(op)) {
        emit(b, "SM_ErrorTrap();");
        return;
    }

    switch (op) {
    case 0x78: emit(b, "SM_SetStatus(1, SM_STATUS_I);"); return; /* SEI */
    case 0x58: emit(b, "SM_SetStatus(0, SM_STATUS_I);"); return; /* CLI */
    case 0xd8: emit(b, "SM_SetStatus(0, SM_STATUS_D);"); return; /* CLD */
    case 0x12: emit(b, "SM_SetStatus(0, SM_STATUS_T);"); return; /* CLT */
    case 0x38: emit(b, "SM_SetStatus(1, SM_STATUS_C);"); return; /* SEC */
    case 0x18: emit(b, "SM_SetStatus(0, SM_STATUS_C);"); return; /* CLC */
    case 0xea: emit(b, "/* nop */"); return;
    case 0x42: emit(b, "sm.sleep = 1;"); return; /* STP */
    case 0xe8: emit(b, "sm.x++;"); emit(b, "SM_Update_NZ(sm.x);"); return; /* INX */
    case 0xc8: emit(b, "sm.y++;"); emit(b, "SM_Update_NZ(sm.y);"); return; /* INY */
    case 0x8a: emit(b, "sm.a = sm.x;"); emit(b, "SM_Update_NZ(sm.a);"); return; /* TXA */
    case 0xaa: emit(b, "sm.x = sm.a;"); emit(b, "SM_Update_NZ(sm.x);"); return; /* TAX */
    case 0x9a: emit(b, "sm.s = sm.x;"); return; /* TXS */
    case 0x48: emit(b, "SM_PushStack(sm.a);"); return; /* PHA */
    case 0x68: emit(b, "sm.a = SM_PopStack();"); emit(b, "SM_Update_NZ(sm.a);"); return; /* PLA */
    case 0x60: /* RTS */
        emit(b, "sm.pc = SM_PopStack();");
        emit(b, "sm.pc |= SM_PopStack() << 8;");
        return;
    case 0x40: /* RTI */
        emit(b, "sm.sr = SM_PopStack();");
        emit(b, "sm.pc = SM_PopStack();");
        emit(b, "sm.pc |= SM_PopStack() << 8;");
        return;

    case 0xa2: case 0xa6: case 0xb6: case 0xae: case 0xbe: /* LDX */
        expr = sm_op_load_expr(op);
        emit(b, "sm.x = %s;", expr);
        emit(b, "SM_Update_NZ(sm.x);");
        return;
    case 0xa0: case 0xa4: case 0xac: case 0xb4: case 0xbc: /* LDY */
        expr = sm_op_load_expr(op);
        emit(b, "sm.y = %s;", expr);
        emit(b, "SM_Update_NZ(sm.y);");
        return;

    case 0x85: case 0x95: case 0x8d: case 0x9d: case 0x99: /* STA */
    case 0x81: case 0x91:
        emit(b, "SM_Write(%s, sm.a);", sm_op_store_dest(op));
        return;
    case 0x86: case 0x96: case 0x8e: /* STX */
        emit(b, "SM_Write(%s, sm.x);", sm_op_store_dest(op));
        return;
    case 0x84: case 0x8c: case 0x94: /* STY */
        emit(b, "SM_Write(%s, sm.y);", sm_op_store_dest(op));
        return;

    case 0xe0: case 0xe4: case 0xec: /* CPX */
        emit(b, "uint8_t operand = %s;", sm_op_load_expr(op));
        emit(b, "int diff = sm.x - operand;");
        emit(b, "SM_SetStatus((diff & 0x100) == 0, SM_STATUS_C);");
        emit(b, "SM_Update_NZ(diff & 0xff);");
        return;
    case 0xc0: case 0xc4: case 0xcc: /* CPY */
        emit(b, "uint8_t operand = %s;", sm_op_load_expr(op));
        emit(b, "int diff = sm.y - operand;");
        emit(b, "SM_SetStatus((diff & 0x100) == 0, SM_STATUS_C);");
        emit(b, "SM_Update_NZ(diff & 0xff);");
        return;

    case 0xf0: /* BEQ */
        emit(b, "int8_t diff = SM_ReadAdvance();");
        emit(b, "if ((sm.sr & SM_STATUS_Z) != 0)");
        emit(b, "    sm.pc += diff;");
        return;
    case 0xd0: /* BNE */
        emit(b, "int8_t diff = SM_ReadAdvance();");
        emit(b, "if ((sm.sr & SM_STATUS_Z) == 0)");
        emit(b, "    sm.pc += diff;");
        return;
    case 0x90: /* BCC */
        emit(b, "int8_t diff = SM_ReadAdvance();");
        emit(b, "if ((sm.sr & SM_STATUS_C) == 0)");
        emit(b, "    sm.pc += diff;");
        return;
    case 0xb0: /* BCS */
        emit(b, "int8_t diff = SM_ReadAdvance();");
        emit(b, "if ((sm.sr & SM_STATUS_C) != 0)");
        emit(b, "    sm.pc += diff;");
        return;
    case 0x10: /* BPL */
        emit(b, "int8_t diff = SM_ReadAdvance();");
        emit(b, "if ((sm.sr & SM_STATUS_N) == 0)");
        emit(b, "    sm.pc += diff;");
        return;
    case 0x80: /* BRA */
        emit(b, "int8_t disp = SM_ReadAdvance();");
        emit(b, "sm.pc += disp;");
        return;

    case 0x3c: /* LDM */
        emit(b, "uint8_t val = SM_ReadAdvance();");
        emit(b, "SM_Write(SM_ReadAdvance(), val);");
        return;

    case 0xa9: case 0xa5: case 0xb5: case 0xad: case 0xbd: /* LDA */
    case 0xb9: case 0xa1: case 0xb1:
        emit(b, "uint8_t val = %s;", sm_op_load_expr(op));
        emit(b, "if ((sm.sr & SM_STATUS_T) == 0)");
        emit(b, "{");
        emit(b, "    sm.a = val;");
        emit(b, "    SM_Update_NZ(val);");
        emit(b, "}");
        emit(b, "else");
        emit(b, "    SM_Write(sm.x, val); /* GT FIXME path */");
        return;

    /* BBC/BBS: bit test of A or (zp); branch when bit != type. */
    case 0x03: case 0x07: case 0x13: case 0x17: case 0x23: case 0x27:
    case 0x33: case 0x37: case 0x43: case 0x47: case 0x53: case 0x57:
    case 0x63: case 0x67: case 0x73: case 0x77: case 0x83: case 0x87:
    case 0x93: case 0x97: case 0xa3: case 0xa7: case 0xb3: case 0xb7:
    case 0xc3: case 0xc7: case 0xd3: case 0xd7: case 0xe3: case 0xe7:
    case 0xf3: case 0xf7:
        if (zp)
            emit(b, "uint8_t val = SM_Read(SM_ReadAdvance());");
        else
            emit(b, "uint8_t val = sm.a;");
        emit(b, "int8_t diff = SM_ReadAdvance();");
        emit(b, "int32_t set = (val >> %d) & 1;", bit);
        if (type)
            emit(b, "if (set != 1)");
        else
            emit(b, "if (set != 0)");
        emit(b, "    sm.pc += diff;");
        return;

    case 0x20: /* JSR abs */
        emit(b, "uint16_t newpc = SM_ReadAdvance16();");
        emit(b, "SM_PushStack(sm.pc >> 8);");
        emit(b, "SM_PushStack(sm.pc & 0xff);");
        emit(b, "sm.pc = newpc;");
        return;
    case 0x02: /* JSR (zp) */
        emit(b, "uint16_t newpc = SM_Read16(SM_ReadAdvance());");
        emit(b, "SM_PushStack(sm.pc >> 8);");
        emit(b, "SM_PushStack(sm.pc & 0xff);");
        emit(b, "sm.pc = newpc;");
        return;
    case 0x22: /* JSR 0xff00|# */
        emit(b, "uint16_t newpc = 0xff00 | SM_ReadAdvance();");
        emit(b, "SM_PushStack(sm.pc >> 8);");
        emit(b, "SM_PushStack(sm.pc & 0xff);");
        emit(b, "sm.pc = newpc;");
        return;

    case 0xc9: case 0xc5: case 0xd5: case 0xcd: case 0xdd: /* CMP */
    case 0xd9: case 0xc1: case 0xd1:
        emit(b, "uint8_t operand = %s;", sm_op_load_expr(op));
        emit(b, "int diff = sm.a - operand;");
        emit(b, "SM_SetStatus((diff & 0x100) == 0, SM_STATUS_C);");
        emit(b, "SM_Update_NZ(diff & 0xff);");
        return;

    case 0x4c: /* JMP abs */
        emit(b, "sm.pc = SM_ReadAdvance16();");
        return;
    case 0x6c: /* JMP (abs) */
        emit(b, "sm.pc = SM_Read16(SM_ReadAdvance16());");
        return;
    case 0xb2: /* JMP (zp) */
        emit(b, "sm.pc = SM_Read16(SM_ReadAdvance());");
        return;

    case 0x09: case 0x05: case 0x15: case 0x0d: case 0x1d: /* ORA */
    case 0x19: case 0x01: case 0x11:
        emit(b, "uint8_t val = 0;");
        emit(b, "uint8_t val2 = 0;");
        emit(b, "if ((sm.sr & SM_STATUS_T) == 0)");
        emit(b, "    val = sm.a;");
        emit(b, "else");
        emit(b, "    val = SM_Read(sm.x); /* GT FIXME path */");
        emit(b, "val2 = %s;", sm_op_load_expr(op));
        emit(b, "val |= val2;");
        emit(b, "if ((sm.sr & SM_STATUS_T) == 0)");
        emit(b, "{");
        emit(b, "    sm.a = val;");
        emit(b, "    SM_Update_NZ(val);");
        emit(b, "}");
        emit(b, "else");
        emit(b, "    SM_Write(sm.x, val); /* GT FIXME path */");
        return;

    case 0x1a: /* DEC a */
        emit(b, "sm.a--;");
        emit(b, "SM_Update_NZ(sm.a);");
        return;
    case 0xc6: case 0xd6: case 0xce: case 0xde: /* DEC mem */
        emit(b, "uint16_t dest = %s;", sm_op_store_dest(op));
        emit(b, "uint8_t val = SM_Read(dest);");
        emit(b, "val--;");
        emit(b, "SM_Write(dest, val);");
        emit(b, "SM_Update_NZ(val);");
        return;

    case 0x29: case 0x25: case 0x35: case 0x2d: case 0x3d: /* AND */
    case 0x39: case 0x21: case 0x31:
        emit(b, "uint8_t val = 0;");
        emit(b, "uint8_t val2 = 0;");
        emit(b, "if ((sm.sr & SM_STATUS_T) == 0)");
        emit(b, "    val = sm.a;");
        emit(b, "else");
        emit(b, "    val = SM_Read(sm.x); /* GT FIXME path */");
        emit(b, "val2 = %s;", sm_op_load_expr(op));
        emit(b, "val &= val2;");
        emit(b, "if ((sm.sr & SM_STATUS_T) == 0)");
        emit(b, "{");
        emit(b, "    sm.a = val;");
        emit(b, "    SM_Update_NZ(val);");
        emit(b, "}");
        emit(b, "else");
        emit(b, "    SM_Write(sm.x, val); /* GT FIXME path */");
        return;

    case 0x3a: /* INC a */
        emit(b, "sm.a++;");
        emit(b, "SM_Update_NZ(sm.a);");
        return;
    case 0xe6: case 0xf6: case 0xee: case 0xfe: /* INC mem */
        emit(b, "uint16_t dest = %s;", sm_op_store_dest(op));
        emit(b, "uint8_t val = SM_Read(dest);");
        emit(b, "val++;");
        emit(b, "SM_Write(dest, val);");
        emit(b, "SM_Update_NZ(val);");
        return;

    /* SEB/CLB: set/clear bit of A or (zp). */
    case 0x0b: case 0x0f: case 0x1b: case 0x1f: case 0x2b: case 0x2f:
    case 0x3b: case 0x3f: case 0x4b: case 0x4f: case 0x5b: case 0x5f:
    case 0x6b: case 0x6f: case 0x7b: case 0x7f: case 0x8b: case 0x8f:
    case 0x9b: case 0x9f: case 0xab: case 0xaf: case 0xbb: case 0xbf:
    case 0xcb: case 0xcf: case 0xdb: case 0xdf: case 0xeb: case 0xef:
    case 0xfb: case 0xff:
        if (zp) {
            emit(b, "uint8_t dest = SM_ReadAdvance();");
            emit(b, "uint8_t val = SM_Read(dest);");
            if (type)
                emit(b, "val &= ~(1 << %d);", bit);
            else
                emit(b, "val |= 1 << %d;", bit);
            emit(b, "SM_Write(dest, val);");
        } else {
            emit(b, "uint8_t val = sm.a;");
            if (type)
                emit(b, "val &= ~(1 << %d);", bit);
            else
                emit(b, "val |= 1 << %d;", bit);
            emit(b, "sm.a = val;");
        }
        return;
    }

    /* Unreachable: every GT table slot is covered above. */
    fprintf(stderr, "smemit: internal error: no transcription for op %02x\n", op);
    exit(3);
}

/* ====================================================================== */
/* SM ROM region and PC set                                                */
/* ====================================================================== */
/*
 * The sub-MCU firmware executes at SM PCs in [0xf000, 0xffff] (14-bit form).
 * GT SM_Read maps address &= 0x1fff; bit 0x1000 selects ROM, indexed by
 * (address & 0xfff). So sm.pc P in this region reads ROM byte g_rom[P & 0xfff].
 * All 10 vector targets and the full VM-verified executed set (251 PCs) fall
 * here. We translate every position in the region ("rom_sm 4KB full
 * translation"): positions that are never executed simply become dead
 * transcriptions (or SM_ErrorTrap for NotImplemented opcodes) and are never
 * dispatched to. g_rom is used only to DECODE the opcode at each position; the
 * emitted bodies call the live GT helpers (SM_Read/Write/...), so behavior is
 * independent of which bytes happen to sit there at runtime.
 */
#define ROM_BASE  0xf000u
#define ROM_LIMIT 0x10000u

static uint8_t g_seen[0x10000];
static uint16_t g_pcs[0x1000 + 64];
static size_t g_pc_count;

static int sm_pc_in_rom(uint16_t pc)
{
    /* pc is 16-bit, so ">= ROM_BASE" already bounds it to [0xf000,0xffff]. */
    return pc >= ROM_BASE;
}

static int sm_pc_add(uint16_t pc)
{
    if (!sm_pc_in_rom(pc) || g_seen[pc])
        return 0;
    g_seen[pc] = 1;
    if (g_pc_count >= sizeof g_pcs / sizeof g_pcs[0]) {
        fprintf(stderr, "smemit: PC table overflow\n");
        exit(2);
    }
    g_pcs[g_pc_count++] = pc;
    return 1;
}

/* ====================================================================== */
/* Files / output helpers                                                  */
/* ====================================================================== */

static uint8_t *load_file(const char *path, uint32_t *size)
{
    FILE *f = fopen(path, "rb");
    long n;
    uint8_t *buf;
    if (!f) {
        fprintf(stderr, "smemit: cannot open '%s'\n", path);
        exit(2);
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "smemit: cannot seek '%s'\n", path);
        exit(2);
    }
    n = ftell(f);
    if (n < 0) {
        fprintf(stderr, "smemit: cannot size '%s'\n", path);
        exit(2);
    }
    rewind(f);
    buf = (uint8_t *)malloc((size_t)n ? (size_t)n : 1);
    if (!buf || fread(buf, 1, (size_t)n, f) != (size_t)n) {
        fprintf(stderr, "smemit: cannot read '%s'\n", path);
        exit(2);
    }
    fclose(f);
    *size = (uint32_t)n;
    return buf;
}

static int parse_hex16(const char *s, uint16_t *out)
{
    uint32_t v = 0;
    int digits = 0;
    while (*s == ' ' || *s == '\t')
        s++;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
        s += 2;
    while (*s) {
        int d;
        char c = *s;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else break;
        v = (v << 4) | (uint32_t)d;
        if (v > 0xffffu)
            return 0;
        digits++;
        s++;
    }
    if (!digits)
        return 0;
    *out = (uint16_t)v;
    return 1;
}

static void ensure_outdir(const char *dir)
{
    char tmp[1024];
    size_t n = strlen(dir);
    char *p;
    if (n == 0 || n >= sizeof tmp) {
        fprintf(stderr, "smemit: bad outdir '%s'\n", dir);
        exit(2);
    }
    memcpy(tmp, dir, n + 1);
    for (p = tmp + 1; *p; p++) {
        if (*p == '/' || *p == '\\') {
            char c = *p;
            *p = '\0';
            if (tmp[0])
                (void)SMIEMT_MKDIR(tmp);
            *p = c;
        }
    }
    (void)SMIEMT_MKDIR(tmp);
}

static void write_file(const char *path, const sb_t *b)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "smemit: cannot write '%s'\n", path);
        exit(2);
    }
    if (b->n && fwrite(b->p, 1, b->n, f) != b->n) {
        fprintf(stderr, "smemit: short write '%s'\n", path);
        fclose(f);
        exit(2);
    }
    fclose(f);
}

static int pc_cmp(const void *a, const void *b)
{
    uint16_t x = *(const uint16_t *)a;
    uint16_t y = *(const uint16_t *)b;
    return (x > y) - (x < y);
}

/* ====================================================================== */
/* main                                                                    */
/* ====================================================================== */

static void usage(void)
{
    fprintf(stderr, "usage: smemit.exe <rom_sm.bin> <outdir> [--extra <pc-list.txt>]\n");
}

int main(int argc, char **argv)
{
    const char *rom_path = NULL;
    const char *outdir = NULL;
    const char *extra_path = NULL;
    const char *pos[2];
    int npos = 0;
    int i;
    uint32_t rom_size = 0;
    char sha_rom[65];
    char path[1024];
    sb_t code, init_decls, init_regs, init, csv, pcset;
    size_t a;
    long n_notimpl = 0;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--extra") == 0) {
            if (i + 1 >= argc) {
                usage();
                return 2;
            }
            extra_path = argv[++i];
        } else if (npos < 2) {
            pos[npos++] = argv[i];
        } else {
            usage();
            return 2;
        }
    }
    if (npos != 2) {
        usage();
        return 2;
    }
    rom_path = pos[0];
    outdir = pos[1];

    sha256_selftest();

    {
        uint8_t *buf = load_file(rom_path, &rom_size);
        if (rom_size != 4096) {
            fprintf(stderr, "smemit: rom_sm must be 4096 bytes (got %u)\n", rom_size);
            return 2;
        }
        memcpy(g_rom, buf, 4096);
        free(buf);
    }
    sha256_hex(g_rom, 4096, sha_rom);

    /* vector table: 10 little-endian u16 at sm_rom[0x0fec..0x0fff]
     * (SM address 0x1fec; GT SM_Read maps it to ROM index 0x0fec). */
    for (i = 0; i < 10; i++) {
        uint16_t v = (uint16_t)(g_rom[0x0fec + 2 * i] |
                                (g_rom[0x0fec + 2 * i + 1] << 8));
        printf("smemit: vector[%d] = %04x%s\n", i, v,
               sm_pc_in_rom(v) ? "" : "  (outside ROM region)");
        sm_pc_add(v);
    }

    /* full-region translation: every SM PC in [0xf000, 0xffff] (4096 of them) */
    {
        int k;
        for (k = 0; k < 0x1000; k++)
            sm_pc_add((uint16_t)(ROM_BASE + (uint16_t)k));
    }

    if (extra_path) {
        FILE *f = fopen(extra_path, "rb");
        char line[128];
        if (!f) {
            fprintf(stderr, "smemit: cannot open '%s'\n", extra_path);
            return 2;
        }
        while (fgets(line, sizeof line, f)) {
            const char *p = line;
            uint16_t pc;
            while (*p == ' ' || *p == '\t')
                p++;
            if (!*p || *p == '\r' || *p == '\n' || *p == '#')
                continue;
            if (!parse_hex16(p, &pc))
                continue;
            if (!sm_pc_in_rom(pc)) {
                printf("smemit: note: --extra pc %04x outside ROM region; skipped\n", pc);
                continue;
            }
            sm_pc_add(pc);
        }
        fclose(f);
    }

    qsort(g_pcs, g_pc_count, sizeof(uint16_t), pc_cmp);

    memset(&code, 0, sizeof code);
    memset(&init_decls, 0, sizeof init_decls);
    memset(&init_regs, 0, sizeof init_regs);
    memset(&init, 0, sizeof init);
    memset(&csv, 0, sizeof csv);
    memset(&pcset, 0, sizeof pcset);

    sb_printf(&code,
        "// GENERATED by " SMEMIT_VERSION " from ROM sha256 -- DO NOT EDIT\n"
        "// rom_sm sha256 %s (4096 bytes)\n"
        "// PCs: %lu (full SM code region [0xf000,0xffff]%s)\n"
        "// one function per PC, exactly one GT SM instruction each; GT helpers only\n\n",
        sha_rom, (unsigned long)g_pc_count,
        extra_path ? " + --extra" : "");
    sb_puts(&code,
        "#include <stdint.h>\n"
        "#include \"submcu.h\"\n"
        "#include \"mk2cpp.h\"\n"
        "\n"
        "/* GT SM helpers defined in src/submcu.cpp (not declared in submcu.h). */\n"
        "uint8_t SM_Read(uint16_t address);\n"
        "uint8_t SM_ReadAdvance(void);\n"
        "uint16_t SM_ReadAdvance16(void);\n"
        "uint16_t SM_Read16(uint16_t address);\n"
        "void SM_Write(uint16_t address, uint8_t data);\n"
        "void SM_PushStack(uint8_t data);\n"
        "uint8_t SM_PopStack(void);\n"
        "void SM_Update_NZ(uint8_t val);\n"
        "void SM_SetStatus(uint32_t condition, uint32_t mask);\n"
        "void SM_ErrorTrap(void);\n\n");

    sb_puts(&csv, "pc,op,len,fn\n");

    for (a = 0; a < g_pc_count; a++) {
        uint16_t pc = g_pcs[a];
        uint8_t op = g_rom[pc & 0xfff];
        int len = 1 + sm_op_operand_bytes(op);
        sb_t body;
        int j;

        memset(&body, 0, sizeof body);
        sb_printf(&body, "/* smk2_pc%04x: pc=%04x op=%02x len=%d bytes:", pc, pc, op, len);
        for (j = 0; j < len; j++)
            sb_printf(&body, " %02x", g_rom[(pc + j) & 0xfff]);
        sb_puts(&body, " */\n");

        emit(&body, "(void)SM_ReadAdvance(); /* opcode 0x%02x */", op);
        emit_body(&body, op);
        if (sm_op_not_implemented(op))
            n_notimpl++;

        sb_printf(&code, "void smk2_pc%04x(void)\n{\n", pc);
        sb_putn(&code, body.p ? body.p : "", body.n);
        sb_puts(&code, "}\n\n");
        free(body.p);

        sb_printf(&init_decls, "void smk2_pc%04x(void);\n", pc);
        sb_printf(&init_regs, "    MK2CPP_SM_Register(0x%04x, &smk2_pc%04x);\n", pc, pc);
        sb_printf(&csv, "%04x,%02x,%d,smk2_pc%04x\n", pc, op, len, pc);
        sb_printf(&pcset, "%04x\n", pc);
    }

    sb_printf(&init,
        "// GENERATED by " SMEMIT_VERSION " from ROM sha256 -- DO NOT EDIT\n"
        "// rom_sm sha256 %s (4096 bytes)\n"
        "// registers every emitted SM PC; called by MK2CPP_Init (MK2CPP_HAS_SM_GEN)\n\n"
        "#include <stdint.h>\n"
        "#include \"mk2cpp.h\"\n\n",
        sha_rom);
    sb_putn(&init, init_decls.p ? init_decls.p : "", init_decls.n);
    sb_puts(&init, "\nvoid MK2CPP_SM_FillTables(void)\n{\n");
    sb_putn(&init, init_regs.p ? init_regs.p : "", init_regs.n);
    sb_puts(&init, "}\n");

    ensure_outdir(outdir);
    snprintf(path, sizeof path, "%s/mk2c_sm.cpp", outdir);
    write_file(path, &code);
    snprintf(path, sizeof path, "%s/mk2cpp_gen_sm_init.cpp", outdir);
    write_file(path, &init);
    snprintf(path, sizeof path, "%s/map_sm.csv", outdir);
    write_file(path, &csv);
    snprintf(path, sizeof path, "%s/sm_pcs.txt", outdir);
    write_file(path, &pcset);

    printf("smemit: outdir=%s emitted=%lu notimplemented=%ld\n",
           outdir, (unsigned long)g_pc_count, n_notimpl);
    printf("smemit: rom_sm sha256 %s\n", sha_rom);

    free(code.p);
    free(init.p);
    free(init_decls.p);
    free(init_regs.p);
    free(csv.p);
    free(pcset.p);
    return 0;
}
