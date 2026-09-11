#pragma once
#include <stdint.h>
enum { H8K_NONE=0, H8K_CALL=1, H8K_UNCOND=2, H8K_COND=3, H8K_RET=4, H8K_REGIND=5 };
typedef struct {
    int valid;          /* decoded within known ISA */
    int len;            /* byte length (0 if invalid) */
    int kind;           /* one of the H8K_* above */
    uint32_t target;    /* static target flat addr when kind==CALL/UNCOND/COND, else 0 */
    uint8_t tpage;      /* target page for pjmp/pjsr/abs forms (0..255) */
    uint16_t toff;      /* target offset */
    /* decoded operand fields, mirroring tools/disasm/h8dasm.c mach output:
       b0=top, b1=reg, b2=siz, b3=ocode, b4=ore, b5=ext */
    uint8_t b0,b1,b2,b3,b4,b5;
} h8dec_t;
void h8dec_set_rom(const uint8_t *rom1, uint32_t rom1_size, const uint8_t *rom2, uint32_t rom2_size);
h8dec_t h8dec(uint32_t flat);
