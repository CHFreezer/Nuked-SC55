/*
 * HAND pcm/pcm_interp -- M4 P2 REC0/REC1 fixed-point integration + output
 * (C12 interp_entry 0x3709.., C13 interp 0x38b0..), one host step per H8
 * instruction (M4 out/m4/18_closure_gap.md 4.2 rows 25-26; docs 07 C12/C13).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 2
 * M4 closure step 2 (semantic rewrite): one named void step function per PC, registered flat = pc via MK2CPP_HandRegister.
 *
 * Scope (all rom1, cp=0):
 *   0x3709..0x37fb C12 interp_entry. TST P-0x73 gate; gate==0 -> 0x377a:
 *     read tone r5 fields, pick the 16-bit input from the ROM table 0x7086
 *     (0x37e4/0x37ee, sign via NEG), store it at P-0x7c and fall into 0x38b0
 *     with r1=P-0x80 (REC0). gate!=0 -> 0x370e..0x3779 per-slot sync arm:
 *     r1=word[P-2]*2 indexes cdc6; word[r2] -= 0x000c; if word<=0xc and the
 *     r2+0x9b/r2+0x98/r2+0x9c mirror bytes all match, tail-call cross_copy
 *     0x3ac8; otherwise clear cdc6[P-2], clear the P-0x73 gate, scrub every
 *     cdc6[i] equal to r2 in an r1=0x1b..0 loop, then the IML window
 *     (0xf8ff / 4 nops / 0x0700), CMP word[P], r6 and either rts or fall
 *     into 0x377a.
 *   0x37fe..0x38ac C13 entry used by bsr16 0x2865/0x3001: TST P-0x51; ==0
 *     -> 0x38aa sets r1=P-0x5e (REC1) and falls into 0x38b0; !=0 -> the
 *     0x3804..0x38a9 mirror of the C12 sync arm (cdc6/cdfe variant) with the
 *     same IML window and tail merge at 0x38aa.
 *   0x38b0..0x39b4 C13 body, shared by both records (r1 = record base):
 *     integrate @r1+24 and @r1+26 with MULXU (dp,0xad2a) (the per-voice rate
 *     latch written by C15/ts_scan EXTU r0), latch 0xffff overflow sentinels,
 *     scale @r1+0/+2/+4 by the integrated factor through signed MULXU, pick
 *     the curve at tone r5[12] (word table 0x6d86), clamp |r4| <= 0x28f6,
 *     then jump through the 16-bit selector table 0x7238: 0x3972
 *     17-point interpolation via (dp,0x7186), r4 -> @r1+32 (REC0+0x20 =
 *     P-0x60); rts at 0x39b4. The selector entry PCs 0x39b5/0x39cc/0x39e0,
 *     the index-4 body 0x3a30..0x3a53 and the entries 0x3a54/0x3a57 are all
 *     registered by gen (out/m4/33 sec.1), so only the dynamically reached
 *     tail is added here:
 *   0x3a5a..0x3a9d selector-5/6 tail (entered at 0x3a54/0x3a57 movi r6):
 *     32-bit r4:r5 += @r1+22; if r4 != 0 write (br,0x3e)=0x1e and read the
 *     PCM (br,0x34) byte / (br,0x3a) word back into @r1+28, else skip; then
 *     slew @r1+28 toward the previous @r1+30 by +/-r6 with V/overshoot clamps
 *     and latch the result to @r1+30 and @r1+32; rts at 0x3a9d. The other
 *     unregistered arms 0x39b8/0x39cf/0x39e3/0x3a00/0x3a0e/0x3a2c have no
 *     dynamic hits (out/m4/34) and stay with the stock interpreter.
 *
 * Translation form: one named void step function per instruction PC (L0,
 * always one H8 instruction) so the host keeps its per-instruction interrupt
 * poll, TIMER_Clock, trace and SM cadence; a multi-instruction block defers
 * the host tail and shifts -midiseq/SM posts at the block end (the drift
 * that made the earlier slices per-PC). Every case is a byte-exact
 * transcription of the validated one-instruction-per-PC emitter
 * (src/gen/mk2c_r1.cpp) with operand bytes folded to constants and the next
 * PC made explicit; no semantic change. SRAM is the page-0 array the stock
 * code uses; device writes go through MCU_Write so -pcmtrace routing is
 * kept. No symbols from src/gen (a hand-only build must link).
 *
 * Confidence (tools/docs/evidence_protocol.md 12): traced PCs are C (dasm +
 * three-run union + gen decode); the never-executed arms 0x370e..0x3779,
 * 0x3804..0x38a9, 0x3950..0x395a and the 0x7238 selector tails other than
 * 0x3972/0x3a30..0x3a53 are C on the ROM bytes / gen decode and I on dynamic
 * reachability (absent from the baseline union); 0x3a5a..0x3a9d is C on the
 * ROM bytes and on the stress_hold trace (first hit cycle 206873088, out/m4/34).
 *
 * Registration: self-registration via hand_registry.h from this file's
 * static object; no shared aggregator is edited. Duplicate registration is
 * fatal (mk2cpp.cpp:113-117), this module is disjoint from the other
 * hand/*.cpp PC ranges.
 */
#include <stdint.h>

#include "mk2cpp.h"
#include "mcu.h"
#include "mcu_interrupt.h"
#include "mcu_opcodes.h"

#include "hand_registry.h"

/* Defined in src/mcu_opcodes.cpp; not exported through a header (same local
 * declaration pattern as pcm_enable.cpp / the split pcm_* modules). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

void step_pc_3709(void)
{
    mcu.pc = 0x370c;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x8d;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x16;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SetStatusCommon(data, 0);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_370c(void)
{
    mcu.pc = 0x370e;
    uint16_t disp = (uint16_t)(int8_t)0x6c;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_370e(void)
{
    mcu.pc = 0x3711;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x81;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[1] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3711(void)
{
    mcu.pc = 0x3713;
    uint8_t op2 = 0x21;
    int32_t t1 = (int32_t)mcu.r[1];
    uint32_t t2 = (uint32_t)mcu.r[1];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[1] = (uint16_t)t1;
}

void step_pc_3713(void)
{
    mcu.pc = 0x3717;
    uint32_t odisp = (uint32_t)0xcd;
    odisp = (odisp << 8) | (uint32_t)0xc6;
    uint32_t oea = (uint32_t)mcu.r[1] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3717(void)
{
    mcu.pc = 0x371b;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x9b;
    uint32_t oea = (uint32_t)mcu.r[0] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x83;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_371b(void)
{
    mcu.pc = 0x371f;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x98;
    uint32_t oea = (uint32_t)mcu.r[0] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x84;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[4] &= ~0xff;
    mcu.r[4] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_371f(void)
{
    mcu.pc = 0x3723;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x9c;
    uint32_t oea = (uint32_t)mcu.r[0] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x85;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[5] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3723(void)
{
    mcu.pc = 0x3728;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0x00;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x05;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    uint32_t t2 = (uint32_t)0x00;
    t2 = (t2 << 8) | (uint32_t)0x0c;
    MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
}

void step_pc_3728(void)
{
    mcu.pc = 0x372a;
    uint16_t disp = (uint16_t)(int8_t)0x15;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = (C | Z) == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_372a(void)
{
    mcu.pc = 0x372e;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x9b;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x73;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
}

void step_pc_372e(void)
{
    mcu.pc = 0x3730;
    uint16_t disp = (uint16_t)(int8_t)0x0f;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3730(void)
{
    mcu.pc = 0x3734;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x98;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x74;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
}

void step_pc_3734(void)
{
    mcu.pc = 0x3736;
    uint16_t disp = (uint16_t)(int8_t)0x09;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3736(void)
{
    mcu.pc = 0x373a;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x9c;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x75;
    int32_t t1 = (int32_t)mcu.r[5];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_373a(void)
{
    mcu.pc = 0x373c;
    uint16_t disp = (uint16_t)(int8_t)0x03;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_373c(void)
{
    mcu.pc = 0x373f;
    uint16_t disp = (uint16_t)(0x03 << 8);
    disp |= 0x89;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_373f(void)
{
    mcu.pc = 0x3742;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x81;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[1] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3742(void)
{
    mcu.pc = 0x3744;
    uint8_t op2 = 0x86;
    uint32_t data = (uint32_t)mcu.r[1];
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3744(void)
{
    mcu.pc = 0x3746;
    uint8_t op2 = 0x21;
    int32_t t1 = (int32_t)mcu.r[1];
    uint32_t t2 = (uint32_t)mcu.r[1];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[1] = (uint16_t)t1;
}

void step_pc_3746(void)
{
    mcu.pc = 0x374a;
    uint32_t odisp = (uint32_t)0xcd;
    odisp = (odisp << 8) | (uint32_t)0xc6;
    uint32_t oea = (uint32_t)mcu.r[1] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x13;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_374a(void)
{
    mcu.pc = 0x374d;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x8d;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x13;
    MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(0));
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_374d(void)
{
    mcu.pc = 0x3750;
    uint16_t data = (uint16_t)(0x00 << 8);
    data |= 0x1b;
    mcu.r[1] = data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3750(void)
{
    mcu.pc = 0x3752;
    uint8_t op2 = 0x76;
    int32_t t1 = (int32_t)mcu.r[6];
    uint32_t t2 = (uint32_t)mcu.r[1];
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_3752(void)
{
    mcu.pc = 0x3754;
    uint16_t disp = (uint16_t)(int8_t)0x0e;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3754(void)
{
    mcu.pc = 0x3756;
    uint8_t op2 = 0x83;
    uint32_t data = (uint32_t)mcu.r[1];
    mcu.r[3] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3756(void)
{
    mcu.pc = 0x3758;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)mcu.r[3];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[3] = (uint16_t)t1;
}

void step_pc_3758(void)
{
    mcu.pc = 0x375c;
    uint32_t odisp = (uint32_t)0xcd;
    odisp = (odisp << 8) | (uint32_t)0xc6;
    uint32_t oea = (uint32_t)mcu.r[3] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
    uint8_t op2 = 0x72;
    int32_t t1 = (int32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_375c(void)
{
    mcu.pc = 0x375e;
    uint16_t disp = (uint16_t)(int8_t)0x04;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_375e(void)
{
    mcu.pc = 0x3762;
    uint32_t odisp = (uint32_t)0xcd;
    odisp = (odisp << 8) | (uint32_t)0xc6;
    uint32_t oea = (uint32_t)mcu.r[3] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
    uint8_t op2 = 0x90;
    uint32_t data = (uint32_t)mcu.r[0];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3762(void)
{
    mcu.pc = 0x3765;
    uint8_t op2 = 0xb9;
    uint8_t reg = op2 & 0x07;
    if ((op2 >> 3) == 0x17)
    {
    uint16_t disp = (uint16_t)(int8_t)0xeb;
    mcu.r[reg]--;
    if (mcu.r[reg] != 0xffff)
    mcu.pc += disp;
    }
    else
    {
    MCU_ErrorTrap();
    }
}

void step_pc_3765(void)
{
    mcu.pc = 0x3768;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3768(void)
{
    mcu.pc = 0x376c;
    uint32_t odata = (uint32_t)0xf8;
    odata = (odata << 8) | (uint32_t)0xff;
    uint8_t op2 = 0x58;
    uint32_t data = odata;
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val &= data;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

void step_pc_376c(void)
{
    mcu.pc = 0x376d;
    /* nop */
}

void step_pc_376d(void)
{
    mcu.pc = 0x376e;
    /* nop */
}

void step_pc_376e(void)
{
    mcu.pc = 0x376f;
    /* nop */
}

void step_pc_376f(void)
{
    mcu.pc = 0x3770;
    /* nop */
}

void step_pc_3770(void)
{
    mcu.pc = 0x3774;
    uint32_t odata = (uint32_t)0x07;
    odata = (odata << 8) | (uint32_t)0x00;
    uint8_t op2 = 0x48;
    uint32_t data = odata;
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val |= data;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

void step_pc_3774(void)
{
    mcu.pc = 0x3777;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x76;
    int32_t t1 = (int32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_3777(void)
{
    mcu.pc = 0x3779;
    uint16_t disp = (uint16_t)(int8_t)0x01;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3779(void)
{
    mcu.pc = MCU_PopStack();
}

void step_pc_377a(void)
{
    mcu.pc = 0x377d;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x2e;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x83;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[3] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_377d(void)
{
    mcu.pc = 0x3780;
    uint32_t oea = (uint32_t)mcu.r[3] + (uint32_t)(int8_t)0x10;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
    uint8_t op2 = 0x84;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[4] &= ~0xff;
    mcu.r[4] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3780(void)
{
    mcu.pc = 0x3783;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xe7;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x83;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3783(void)
{
    mcu.pc = 0x3786;
    uint32_t odata = (uint32_t)0x40;
    uint8_t op2 = 0x34;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = odata;
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[4] &= ~0xff;
    mcu.r[4] |= (uint16_t)(t1 & 0xff);
}

void step_pc_3786(void)
{
    mcu.pc = 0x3788;
    uint16_t disp = (uint16_t)(int8_t)0x08;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3788(void)
{
    mcu.pc = 0x378a;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(t1 & 0xff);
}

void step_pc_378a(void)
{
    mcu.pc = 0x378c;
    uint16_t disp = (uint16_t)(int8_t)0x0a;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_378c(void)
{
    mcu.pc = 0x378e;
    uint8_t op2 = 0x13;
    mcu.r[3] = (uint16_t)(0);
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_378e(void)
{
    mcu.pc = 0x3790;
    uint16_t disp = (uint16_t)(int8_t)0x06;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3790(void)
{
    mcu.pc = 0x3792;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(t1 & 0xff);
}

void step_pc_3792(void)
{
    mcu.pc = 0x3794;
    uint16_t disp = (uint16_t)(int8_t)0x02;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3794(void)
{
    mcu.pc = 0x3796;
    uint8_t data = 0x7f;
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= data;
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3796(void)
{
    mcu.pc = 0x3799;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x8c;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x93;
    uint32_t data = (uint32_t)mcu.r[3];
    MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(data));
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3799(void)
{
    mcu.pc = 0x379b;
    uint8_t op2 = 0x13;
    mcu.r[3] = (uint16_t)(0);
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_379b(void)
{
    mcu.pc = 0x379e;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x2e;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_379e(void)
{
    mcu.pc = 0x37a1;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0x11;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x83;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_37a1(void)
{
    mcu.pc = 0x37a5;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0xa8;
    uint32_t oea = (uint32_t)mcu.r[0] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_37a5(void)
{
    mcu.pc = 0x37a7;
    uint8_t op2 = 0x16;
    uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
    MCU_SetStatusCommon(data, 0);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_37a7(void)
{
    mcu.pc = 0x37a9;
    uint16_t disp = (uint16_t)(int8_t)0x0a;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37a9(void)
{
    mcu.pc = 0x37ac;
    uint32_t odata = (uint32_t)0x7f;
    uint8_t op2 = 0x52;
    uint32_t data = (uint32_t)mcu.r[2];
    uint32_t t2 = odata;
    data &= t2;
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(mcu.r[2], 0);
}

void step_pc_37ac(void)
{
    mcu.pc = 0x37af;
    uint32_t odata = (uint32_t)0x40;
    uint8_t op2 = 0x33;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = odata;
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37af(void)
{
    mcu.pc = 0x37b1;
    uint16_t disp = (uint16_t)(int8_t)0x29;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t branch = C == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37b1(void)
{
    mcu.pc = 0x37b3;
    uint16_t disp = (uint16_t)(int8_t)0x1b;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37b3(void)
{
    mcu.pc = 0x37b6;
    uint32_t odata = (uint32_t)0x40;
    uint8_t op2 = 0x33;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = odata;
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37b6(void)
{
    mcu.pc = 0x37b8;
    uint16_t disp = (uint16_t)(int8_t)0x0c;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t branch = C == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37b8(void)
{
    mcu.pc = 0x37ba;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
    mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
}

void step_pc_37ba(void)
{
    mcu.pc = 0x37bc;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37bc(void)
{
    mcu.pc = 0x37be;
    uint8_t op2 = 0x32;
    int32_t t1 = (int32_t)mcu.r[2];
    uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37be(void)
{
    mcu.pc = 0x37c0;
    uint16_t disp = (uint16_t)(int8_t)0x2c;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37c0(void)
{
    mcu.pc = 0x37c2;
    uint8_t op2 = 0x13;
    mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(0) & 0xffu));
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_37c2(void)
{
    mcu.pc = 0x37c4;
    uint16_t disp = (uint16_t)(int8_t)0x28;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37c4(void)
{
    mcu.pc = 0x37c6;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37c6(void)
{
    mcu.pc = 0x37c8;
    uint8_t op2 = 0x22;
    int32_t t1 = (int32_t)mcu.r[2];
    uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37c8(void)
{
    mcu.pc = 0x37ca;
    uint16_t disp = (uint16_t)(int8_t)0x22;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37ca(void)
{
    mcu.pc = 0x37cc;
    uint8_t data = 0x7f;
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= data;
    MCU_SetStatusCommon(data, 0);
}

void step_pc_37cc(void)
{
    mcu.pc = 0x37ce;
    uint16_t disp = (uint16_t)(int8_t)0x1e;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37ce(void)
{
    mcu.pc = 0x37d0;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
    mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
}

void step_pc_37d0(void)
{
    mcu.pc = 0x37d2;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37d2(void)
{
    mcu.pc = 0x37d4;
    uint8_t op2 = 0x22;
    int32_t t1 = (int32_t)mcu.r[2];
    uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37d4(void)
{
    mcu.pc = 0x37d6;
    uint16_t disp = (uint16_t)(int8_t)0x0c;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37d6(void)
{
    mcu.pc = 0x37d8;
    uint8_t data = 0x7f;
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= data;
    MCU_SetStatusCommon(data, 0);
}

void step_pc_37d8(void)
{
    mcu.pc = 0x37da;
    uint16_t disp = (uint16_t)(int8_t)0x08;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37da(void)
{
    mcu.pc = 0x37dc;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37dc(void)
{
    mcu.pc = 0x37de;
    uint8_t op2 = 0x32;
    int32_t t1 = (int32_t)mcu.r[2];
    uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= (uint16_t)(t1 & 0xff);
}

void step_pc_37de(void)
{
    mcu.pc = 0x37e0;
    uint16_t disp = (uint16_t)(int8_t)0x02;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37e0(void)
{
    mcu.pc = 0x37e2;
    uint8_t op2 = 0x13;
    mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(0) & 0xffu));
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_37e2(void)
{
    mcu.pc = 0x37e4;
    uint8_t op2 = 0x22;
    int32_t t1 = (int32_t)mcu.r[2];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[2] = (uint16_t)t1;
}

void step_pc_37e4(void)
{
    mcu.pc = 0x37e8;
    uint32_t odisp = (uint32_t)0x70;
    odisp = (odisp << 8) | (uint32_t)0x86;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_37e8(void)
{
    mcu.pc = 0x37ea;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[2];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[2] = (uint16_t)(data);
}

void step_pc_37ea(void)
{
    mcu.pc = 0x37ec;
    uint16_t disp = (uint16_t)(int8_t)0x06;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37ec(void)
{
    mcu.pc = 0x37ee;
    uint8_t op2 = 0x22;
    int32_t t1 = (int32_t)mcu.r[2];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[2] = (uint16_t)t1;
}

void step_pc_37ee(void)
{
    mcu.pc = 0x37f2;
    uint32_t odisp = (uint32_t)0x70;
    odisp = (odisp << 8) | (uint32_t)0x86;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_37f2(void)
{
    mcu.pc = 0x37f5;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x84;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x92;
    uint32_t data = (uint32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_37f5(void)
{
    mcu.pc = 0x37f7;
    uint8_t op2 = 0x81;
    uint32_t data = (uint32_t)mcu.r[0];
    mcu.r[1] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_37f7(void)
{
    mcu.pc = 0x37fb;
    uint32_t odata = (uint32_t)0xff;
    odata = (odata << 8) | (uint32_t)0x80;
    uint8_t op2 = 0x21;
    int32_t t1 = (int32_t)mcu.r[1];
    uint32_t t2 = odata;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[1] = (uint16_t)t1;
}

void step_pc_37fb(void)
{
    mcu.pc = 0x37fe;
    uint16_t disp = (uint16_t)(0x00 << 8);
    disp |= 0xb2;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_37fe(void)
{
    mcu.pc = 0x3801;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xaf;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x16;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SetStatusCommon(data, 0);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3801(void)
{
    mcu.pc = 0x3804;
    uint16_t disp = (uint16_t)(0x00 << 8);
    disp |= 0xa6;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3804(void)
{
    mcu.pc = 0x3807;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x81;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[1] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3807(void)
{
    mcu.pc = 0x3809;
    uint8_t op2 = 0x21;
    int32_t t1 = (int32_t)mcu.r[1];
    uint32_t t2 = (uint32_t)mcu.r[1];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[1] = (uint16_t)t1;
}

void step_pc_3809(void)
{
    mcu.pc = 0x380d;
    uint32_t odisp = (uint32_t)0xcd;
    odisp = (odisp << 8) | (uint32_t)0xfe;
    uint32_t oea = (uint32_t)mcu.r[1] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_380d(void)
{
    mcu.pc = 0x3811;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x9b;
    uint32_t oea = (uint32_t)mcu.r[0] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x83;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3811(void)
{
    mcu.pc = 0x3815;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x99;
    uint32_t oea = (uint32_t)mcu.r[0] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x84;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[4] &= ~0xff;
    mcu.r[4] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3815(void)
{
    mcu.pc = 0x3819;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x9e;
    uint32_t oea = (uint32_t)mcu.r[0] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x85;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[5] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3819(void)
{
    mcu.pc = 0x381e;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0x00;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x05;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    uint32_t t2 = (uint32_t)0x00;
    t2 = (t2 << 8) | (uint32_t)0x0c;
    MCU_SUB_Common((int32_t)t1, (int32_t)t2, 0, 1);
}

void step_pc_381e(void)
{
    mcu.pc = 0x3820;
    uint16_t disp = (uint16_t)(int8_t)0x4f;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = (C | Z) == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3820(void)
{
    mcu.pc = 0x3824;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x9b;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x73;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
}

void step_pc_3824(void)
{
    mcu.pc = 0x3826;
    uint16_t disp = (uint16_t)(int8_t)0x49;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3826(void)
{
    mcu.pc = 0x382a;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x99;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x74;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
}

void step_pc_382a(void)
{
    mcu.pc = 0x382c;
    uint16_t disp = (uint16_t)(int8_t)0x43;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_382c(void)
{
    mcu.pc = 0x3830;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x9e;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x75;
    int32_t t1 = (int32_t)mcu.r[5];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_3830(void)
{
    mcu.pc = 0x3832;
    uint16_t disp = (uint16_t)(int8_t)0x3d;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3832(void)
{
    mcu.pc = 0x3835;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xa8;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3835(void)
{
    mcu.pc = 0x3838;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xa8;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3838(void)
{
    mcu.pc = 0x383b;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xaa;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_383b(void)
{
    mcu.pc = 0x383e;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xaa;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_383e(void)
{
    mcu.pc = 0x3841;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xac;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3841(void)
{
    mcu.pc = 0x3844;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xac;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3844(void)
{
    mcu.pc = 0x3847;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xb0;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3847(void)
{
    mcu.pc = 0x384a;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb0;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_384a(void)
{
    mcu.pc = 0x384d;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xb8;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_384d(void)
{
    mcu.pc = 0x3850;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xb8;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3850(void)
{
    mcu.pc = 0x3853;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xba;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3853(void)
{
    mcu.pc = 0x3856;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xba;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3856(void)
{
    mcu.pc = 0x3859;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xbc;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3859(void)
{
    mcu.pc = 0x385c;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xbc;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_385c(void)
{
    mcu.pc = 0x385f;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xbe;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_385f(void)
{
    mcu.pc = 0x3862;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xbe;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3862(void)
{
    mcu.pc = 0x3865;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xc0;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3865(void)
{
    mcu.pc = 0x3868;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc0;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3868(void)
{
    mcu.pc = 0x386b;
    uint32_t oea = (uint32_t)mcu.r[2] + (uint32_t)(int8_t)0xc2;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_386b(void)
{
    mcu.pc = 0x386e;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xc2;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x96;
    uint32_t data = (uint32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_386e(void)
{
    mcu.pc = MCU_PopStack();
}

void step_pc_386f(void)
{
    mcu.pc = 0x3872;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xfe;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x81;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[1] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3872(void)
{
    mcu.pc = 0x3874;
    uint8_t op2 = 0x86;
    uint32_t data = (uint32_t)mcu.r[1];
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3874(void)
{
    mcu.pc = 0x3876;
    uint8_t op2 = 0x21;
    int32_t t1 = (int32_t)mcu.r[1];
    uint32_t t2 = (uint32_t)mcu.r[1];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[1] = (uint16_t)t1;
}

void step_pc_3876(void)
{
    mcu.pc = 0x387a;
    uint32_t odisp = (uint32_t)0xcd;
    odisp = (odisp << 8) | (uint32_t)0xfe;
    uint32_t oea = (uint32_t)mcu.r[1] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x13;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(0));
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_387a(void)
{
    mcu.pc = 0x387d;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0xaf;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x13;
    MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(0));
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_387d(void)
{
    mcu.pc = 0x3880;
    uint16_t data = (uint16_t)(0x00 << 8);
    data |= 0x1b;
    mcu.r[1] = data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3880(void)
{
    mcu.pc = 0x3882;
    uint8_t op2 = 0x76;
    int32_t t1 = (int32_t)mcu.r[6];
    uint32_t t2 = (uint32_t)mcu.r[1];
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_3882(void)
{
    mcu.pc = 0x3884;
    uint16_t disp = (uint16_t)(int8_t)0x0e;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3884(void)
{
    mcu.pc = 0x3886;
    uint8_t op2 = 0x83;
    uint32_t data = (uint32_t)mcu.r[1];
    mcu.r[3] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3886(void)
{
    mcu.pc = 0x3888;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)mcu.r[3];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[3] = (uint16_t)t1;
}

void step_pc_3888(void)
{
    mcu.pc = 0x388c;
    uint32_t odisp = (uint32_t)0xcd;
    odisp = (odisp << 8) | (uint32_t)0xfe;
    uint32_t oea = (uint32_t)mcu.r[3] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
    uint8_t op2 = 0x72;
    int32_t t1 = (int32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_388c(void)
{
    mcu.pc = 0x388e;
    uint16_t disp = (uint16_t)(int8_t)0x04;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_388e(void)
{
    mcu.pc = 0x3892;
    uint32_t odisp = (uint32_t)0xcd;
    odisp = (odisp << 8) | (uint32_t)0xfe;
    uint32_t oea = (uint32_t)mcu.r[3] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
    uint8_t op2 = 0x90;
    uint32_t data = (uint32_t)mcu.r[0];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3892(void)
{
    mcu.pc = 0x3895;
    uint8_t op2 = 0xb9;
    uint8_t reg = op2 & 0x07;
    if ((op2 >> 3) == 0x17)
    {
    uint16_t disp = (uint16_t)(int8_t)0xeb;
    mcu.r[reg]--;
    if (mcu.r[reg] != 0xffff)
    mcu.pc += disp;
    }
    else
    {
    MCU_ErrorTrap();
    }
}

void step_pc_3895(void)
{
    mcu.pc = 0x3898;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3898(void)
{
    mcu.pc = 0x389c;
    uint32_t odata = (uint32_t)0xf8;
    odata = (odata << 8) | (uint32_t)0xff;
    uint8_t op2 = 0x58;
    uint32_t data = odata;
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val &= data;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

void step_pc_389c(void)
{
    mcu.pc = 0x389d;
    /* nop */
}

void step_pc_389d(void)
{
    mcu.pc = 0x389e;
    /* nop */
}

void step_pc_389e(void)
{
    mcu.pc = 0x389f;
    /* nop */
}

void step_pc_389f(void)
{
    mcu.pc = 0x38a0;
    /* nop */
}

void step_pc_38a0(void)
{
    mcu.pc = 0x38a4;
    uint32_t odata = (uint32_t)0x07;
    odata = (odata << 8) | (uint32_t)0x00;
    uint8_t op2 = 0x48;
    uint32_t data = odata;
    uint32_t val = MCU_ControlRegisterRead(0, 1);
    val |= data;
    MCU_ControlRegisterWrite(0, 1, val);
    mcu.ex_ignore = 1;
}

void step_pc_38a4(void)
{
    mcu.pc = 0x38a7;
    uint32_t oea = (uint32_t)mcu.r[0] + (uint32_t)(int8_t)0x00;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(0) & 0xff);
    uint8_t op2 = 0x76;
    int32_t t1 = (int32_t)mcu.r[6];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_38a7(void)
{
    mcu.pc = 0x38a9;
    uint16_t disp = (uint16_t)(int8_t)0x01;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_38a9(void)
{
    mcu.pc = MCU_PopStack();
}

void step_pc_38aa(void)
{
    mcu.pc = 0x38ac;
    uint8_t op2 = 0x81;
    uint32_t data = (uint32_t)mcu.r[0];
    mcu.r[1] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38ac(void)
{
    mcu.pc = 0x38b0;
    uint32_t odata = (uint32_t)0xff;
    odata = (odata << 8) | (uint32_t)0xa2;
    uint8_t op2 = 0x21;
    int32_t t1 = (int32_t)mcu.r[1];
    uint32_t t2 = odata;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[1] = (uint16_t)t1;
}

void step_pc_38b0(void)
{
    mcu.pc = 0x38b3;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x18;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38b3(void)
{
    mcu.pc = 0x38b6;
    int32_t t2 = (int32_t)0xff;
    t2 = (t2 << 8) | (int32_t)0xff;
    int32_t t1 = (int32_t)mcu.r[6];
    MCU_SUB_Common(t1, t2, 0, 1);
}

void step_pc_38b6(void)
{
    mcu.pc = 0x38b8;
    uint16_t disp = (uint16_t)(int8_t)0x1e;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_38b8(void)
{
    mcu.pc = 0x38bb;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x10;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x84;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[4] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38bb(void)
{
    mcu.pc = 0x38bf;
    uint32_t oea = (uint32_t)0xad;
    oea = (oea << 8) | (uint32_t)0x2a;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(mcu.dp & 0xff);
    uint8_t op2 = 0xac;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    uint32_t t2 = (uint32_t)mcu.r[4];
    t1 *= t2;
    mcu.r[4] = (uint16_t)(t1 >> 16);
    mcu.r[5] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_38bf(void)
{
    mcu.pc = 0x38c1;
    uint8_t op2 = 0x25;
    int32_t t1 = (int32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[6];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[5] = (uint16_t)t1;
}

void step_pc_38c1(void)
{
    mcu.pc = 0x38c5;
    uint32_t odata = (uint32_t)0x00;
    odata = (odata << 8) | (uint32_t)0x00;
    uint8_t op2 = 0xa4;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = odata;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
    if (!Z)
    MCU_SetStatus(0, STATUS_Z);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_38c5(void)
{
    mcu.pc = 0x38c7;
    uint8_t op2 = 0x16;
    uint32_t data = (uint32_t)mcu.r[4];
    MCU_SetStatusCommon(data, 1);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_38c7(void)
{
    mcu.pc = 0x38c9;
    uint16_t disp = (uint16_t)(int8_t)0x08;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_38c9(void)
{
    mcu.pc = 0x38cc;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x18;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x95;
    uint32_t data = (uint32_t)mcu.r[5];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38cc(void)
{
    mcu.pc = 0x38cf;
    int32_t t2 = (int32_t)0xff;
    t2 = (t2 << 8) | (int32_t)0xff;
    int32_t t1 = (int32_t)mcu.r[5];
    MCU_SUB_Common(t1, t2, 0, 1);
}

void step_pc_38cf(void)
{
    mcu.pc = 0x38d1;
    uint16_t disp = (uint16_t)(int8_t)0x6f;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_38d1(void)
{
    mcu.pc = 0x38d6;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x18;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x07;
    uint32_t d = (uint32_t)0xff;
    d = (d << 8) | (uint32_t)0xff;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(d));
    MCU_SetStatusCommon(d, 1);
}

void step_pc_38d6(void)
{
    mcu.pc = 0x38d9;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x1a;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x86;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38d9(void)
{
    mcu.pc = 0x38dc;
    int32_t t2 = (int32_t)0xff;
    t2 = (t2 << 8) | (int32_t)0xff;
    int32_t t1 = (int32_t)mcu.r[6];
    MCU_SUB_Common(t1, t2, 0, 1);
}

void step_pc_38dc(void)
{
    mcu.pc = 0x38de;
    uint16_t disp = (uint16_t)(int8_t)0x50;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_38de(void)
{
    mcu.pc = 0x38e1;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x12;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x84;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[4] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38e1(void)
{
    mcu.pc = 0x38e5;
    uint32_t oea = (uint32_t)0xad;
    oea = (oea << 8) | (uint32_t)0x2a;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(mcu.dp & 0xff);
    uint8_t op2 = 0xac;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    uint32_t t2 = (uint32_t)mcu.r[4];
    t1 *= t2;
    mcu.r[4] = (uint16_t)(t1 >> 16);
    mcu.r[5] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_38e5(void)
{
    mcu.pc = 0x38e7;
    uint8_t op2 = 0x25;
    int32_t t1 = (int32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[6];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[5] = (uint16_t)t1;
}

void step_pc_38e7(void)
{
    mcu.pc = 0x38eb;
    uint32_t odata = (uint32_t)0x00;
    odata = (odata << 8) | (uint32_t)0x00;
    uint8_t op2 = 0xa4;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = odata;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
    if (!Z)
    MCU_SetStatus(0, STATUS_Z);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_38eb(void)
{
    mcu.pc = 0x38ed;
    uint8_t op2 = 0x16;
    uint32_t data = (uint32_t)mcu.r[4];
    MCU_SetStatusCommon(data, 1);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_38ed(void)
{
    mcu.pc = 0x38ef;
    uint16_t disp = (uint16_t)(int8_t)0x03;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_38ef(void)
{
    mcu.pc = 0x38f2;
    uint16_t data = (uint16_t)(0xff << 8);
    data |= 0xff;
    mcu.r[5] = data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38f2(void)
{
    mcu.pc = 0x38f5;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x1a;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x95;
    uint32_t data = (uint32_t)mcu.r[5];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38f5(void)
{
    mcu.pc = 0x38f8;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x00;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_38f8(void)
{
    mcu.pc = 0x38fa;
    uint16_t disp = (uint16_t)(int8_t)0x08;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_38fa(void)
{
    mcu.pc = 0x38fc;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[2];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[2] = (uint16_t)(data);
}

void step_pc_38fc(void)
{
    mcu.pc = 0x38fe;
    uint8_t op2 = 0xaa;
    uint32_t t1 = (uint32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 *= t2;
    mcu.r[2] = (uint16_t)(t1 >> 16);
    mcu.r[3] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_38fe(void)
{
    mcu.pc = 0x3900;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[2];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[2] = (uint16_t)(data);
}

void step_pc_3900(void)
{
    mcu.pc = 0x3902;
    uint16_t disp = (uint16_t)(int8_t)0x02;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3902(void)
{
    mcu.pc = 0x3904;
    uint8_t op2 = 0xaa;
    uint32_t t1 = (uint32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 *= t2;
    mcu.r[2] = (uint16_t)(t1 >> 16);
    mcu.r[3] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3904(void)
{
    mcu.pc = 0x3907;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x06;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x92;
    uint32_t data = (uint32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3907(void)
{
    mcu.pc = 0x390a;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x02;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_390a(void)
{
    mcu.pc = 0x390c;
    uint16_t disp = (uint16_t)(int8_t)0x08;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_390c(void)
{
    mcu.pc = 0x390e;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[2];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[2] = (uint16_t)(data);
}

void step_pc_390e(void)
{
    mcu.pc = 0x3910;
    uint8_t op2 = 0xaa;
    uint32_t t1 = (uint32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 *= t2;
    mcu.r[2] = (uint16_t)(t1 >> 16);
    mcu.r[3] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3910(void)
{
    mcu.pc = 0x3912;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[2];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[2] = (uint16_t)(data);
}

void step_pc_3912(void)
{
    mcu.pc = 0x3914;
    uint16_t disp = (uint16_t)(int8_t)0x02;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3914(void)
{
    mcu.pc = 0x3916;
    uint8_t op2 = 0xaa;
    uint32_t t1 = (uint32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 *= t2;
    mcu.r[2] = (uint16_t)(t1 >> 16);
    mcu.r[3] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3916(void)
{
    mcu.pc = 0x3919;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x08;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x92;
    uint32_t data = (uint32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3919(void)
{
    mcu.pc = 0x391d;
    uint32_t odisp = (uint32_t)0x00;
    odisp = (odisp << 8) | (uint32_t)0x04;
    uint32_t oea = (uint32_t)mcu.r[1] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_391d(void)
{
    mcu.pc = 0x391f;
    uint16_t disp = (uint16_t)(int8_t)0x08;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_391f(void)
{
    mcu.pc = 0x3921;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[2];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[2] = (uint16_t)(data);
}

void step_pc_3921(void)
{
    mcu.pc = 0x3923;
    uint8_t op2 = 0xaa;
    uint32_t t1 = (uint32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 *= t2;
    mcu.r[2] = (uint16_t)(t1 >> 16);
    mcu.r[3] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3923(void)
{
    mcu.pc = 0x3925;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[2];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[2] = (uint16_t)(data);
}

void step_pc_3925(void)
{
    mcu.pc = 0x3927;
    uint16_t disp = (uint16_t)(int8_t)0x02;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3927(void)
{
    mcu.pc = 0x3929;
    uint8_t op2 = 0xaa;
    uint32_t t1 = (uint32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 *= t2;
    mcu.r[2] = (uint16_t)(t1 >> 16);
    mcu.r[3] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3929(void)
{
    mcu.pc = 0x392c;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x0a;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x92;
    uint32_t data = (uint32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_392c(void)
{
    mcu.pc = 0x392e;
    uint16_t disp = (uint16_t)(int8_t)0x12;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_392e(void)
{
    mcu.pc = 0x3931;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x00;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3931(void)
{
    mcu.pc = 0x3934;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x06;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x92;
    uint32_t data = (uint32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3934(void)
{
    mcu.pc = 0x3937;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x02;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3937(void)
{
    mcu.pc = 0x393a;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x08;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x92;
    uint32_t data = (uint32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_393a(void)
{
    mcu.pc = 0x393d;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x04;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_393d(void)
{
    mcu.pc = 0x3940;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x0a;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x92;
    uint32_t data = (uint32_t)mcu.r[2];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3940(void)
{
    mcu.pc = 0x3942;
    uint8_t op2 = 0x13;
    mcu.r[3] = (uint16_t)(0);
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3942(void)
{
    mcu.pc = 0x3945;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x0c;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x83;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[3] &= ~0xff;
    mcu.r[3] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3945(void)
{
    mcu.pc = 0x3947;
    uint8_t op2 = 0x23;
    int32_t t1 = (int32_t)mcu.r[3];
    uint32_t t2 = (uint32_t)mcu.r[3];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[3] = (uint16_t)t1;
}

void step_pc_3947(void)
{
    mcu.pc = 0x394b;
    uint32_t odisp = (uint32_t)0x6d;
    odisp = (odisp << 8) | (uint32_t)0x86;
    uint32_t oea = (uint32_t)mcu.r[3] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(3) & 0xff);
    uint8_t op2 = 0x83;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[3] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_394b(void)
{
    mcu.pc = 0x394e;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x0e;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x84;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[4] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_394e(void)
{
    mcu.pc = 0x3950;
    uint16_t disp = (uint16_t)(int8_t)0x0b;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t branch = N == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3950(void)
{
    mcu.pc = 0x3952;
    uint8_t op2 = 0x24;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)mcu.r[3];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_3952(void)
{
    mcu.pc = 0x3955;
    int32_t t2 = (int32_t)0x28;
    t2 = (t2 << 8) | (int32_t)0xf6;
    int32_t t1 = (int32_t)mcu.r[4];
    MCU_SUB_Common(t1, t2, 0, 1);
}

void step_pc_3955(void)
{
    mcu.pc = 0x3957;
    uint16_t disp = (uint16_t)(int8_t)0x0e;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = (C | Z) == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3957(void)
{
    mcu.pc = 0x3959;
    uint8_t op2 = 0x13;
    mcu.r[4] = (uint16_t)(0);
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3959(void)
{
    mcu.pc = 0x395b;
    uint16_t disp = (uint16_t)(int8_t)0x0a;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_395b(void)
{
    mcu.pc = 0x395d;
    uint8_t op2 = 0x24;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)mcu.r[3];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_395d(void)
{
    mcu.pc = 0x3960;
    int32_t t2 = (int32_t)0x28;
    t2 = (t2 << 8) | (int32_t)0xf6;
    int32_t t1 = (int32_t)mcu.r[4];
    MCU_SUB_Common(t1, t2, 0, 1);
}

void step_pc_3960(void)
{
    mcu.pc = 0x3962;
    uint16_t disp = (uint16_t)(int8_t)0x03;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = (C | Z) == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3962(void)
{
    mcu.pc = 0x3965;
    uint16_t data = (uint16_t)(0x28 << 8);
    data |= 0xf6;
    mcu.r[4] = data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3965(void)
{
    mcu.pc = 0x3969;
    uint32_t oea = (uint32_t)0xad;
    oea = (oea << 8) | (uint32_t)0x2a;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(mcu.dp & 0xff);
    uint8_t op2 = 0xac;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t1 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    uint32_t t2 = (uint32_t)mcu.r[4];
    t1 *= t2;
    mcu.r[4] = (uint16_t)(t1 >> 16);
    mcu.r[5] = (uint16_t)t1;
    uint32_t N = (t1 & 0x80000000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3969(void)
{
    mcu.pc = 0x396c;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x14;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_396c(void)
{
    mcu.pc = 0x3970;
    uint32_t odisp = (uint32_t)0x72;
    odisp = (odisp << 8) | (uint32_t)0x38;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x82;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3970(void)
{
    uint8_t opcode = 0xd2;
    uint8_t opcode_h = opcode >> 3;
    uint8_t opcode_l = opcode & 0x07;
    if (opcode == 0x19)
    {
    mcu.cp = (uint8_t)MCU_PopStack();
    mcu.pc = MCU_PopStack();
    }
    else if (opcode_h == 0x19)
    {
    MCU_PushStack(mcu.pc);
    MCU_PushStack(mcu.cp);
    opcode_l &= ~1;
    mcu.cp = (uint8_t)(mcu.r[opcode_l] & 0xff);
    mcu.pc = mcu.r[opcode_l + 1];
    }
    else if (opcode_h == 0x1a)
    {
    mcu.pc = mcu.r[opcode_l];
    }
    else if (opcode_h == 0x1b)
    {
    MCU_PushStack(mcu.pc);
    mcu.pc = mcu.r[opcode_l];
    }
    else
    {
    MCU_ErrorTrap();
    }
}

void step_pc_3972(void)
{
    mcu.pc = 0x3975;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x16;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x25;
    int32_t t1 = (int32_t)mcu.r[5];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[5] = (uint16_t)t1;
}

void step_pc_3975(void)
{
    mcu.pc = 0x3978;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x16;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x95;
    uint32_t data = (uint32_t)mcu.r[5];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3978(void)
{
    mcu.pc = 0x397a;
    uint8_t op2 = 0x86;
    uint32_t data = (uint32_t)mcu.r[5];
    mcu.r[6] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_397a(void)
{
    mcu.pc = 0x397e;
    uint32_t odata = (uint32_t)0x80;
    odata = (odata << 8) | (uint32_t)0x00;
    uint8_t op2 = 0x35;
    int32_t t1 = (int32_t)mcu.r[5];
    uint32_t t2 = odata;
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[5] = (uint16_t)t1;
}

void step_pc_397e(void)
{
    mcu.pc = 0x3980;
    uint16_t disp = (uint16_t)(int8_t)0x02;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t branch = C == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3980(void)
{
    mcu.pc = 0x3982;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[5];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[5] = (uint16_t)(data);
}

void step_pc_3982(void)
{
    mcu.pc = 0x3984;
    uint8_t op2 = 0x82;
    uint32_t data = (uint32_t)mcu.r[5];
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3984(void)
{
    mcu.pc = 0x3986;
    uint8_t op2 = 0x13;
    mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(0) & 0xffu));
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3986(void)
{
    mcu.pc = 0x3988;
    uint8_t op2 = 0x10;
    uint32_t data = (uint32_t)mcu.r[2];
    uint32_t data_h = data >> 8;
    uint32_t data_l = data & 0xff;
    data = (data_l << 8) | data_h;
    mcu.r[2] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3988(void)
{
    mcu.pc = 0x398a;
    uint8_t op2 = 0x13;
    mcu.r[4] = (uint16_t)(0);
    MCU_SetStatus(0, STATUS_N);
    MCU_SetStatus(1, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_398a(void)
{
    mcu.pc = 0x398e;
    uint32_t odisp = (uint32_t)0x71;
    odisp = (odisp << 8) | (uint32_t)0x86;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x84;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[4] &= ~0xff;
    mcu.r[4] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_398e(void)
{
    mcu.pc = 0x3990;
    uint8_t op2 = 0x08;
    uint32_t t1 = (uint32_t)mcu.r[2];
    int32_t t2 = 1;
    t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, t2, 0, 1);
    mcu.r[2] = (uint16_t)(t1);
}

void step_pc_3990(void)
{
    mcu.pc = 0x3994;
    uint32_t odisp = (uint32_t)0x71;
    odisp = (odisp << 8) | (uint32_t)0x86;
    uint32_t oea = (uint32_t)mcu.r[2] + odisp;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(2) & 0xff);
    uint8_t op2 = 0x82;
    uint32_t data = (uint32_t)MCU_Read(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= (uint16_t)(data & 0xff);
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3994(void)
{
    mcu.pc = 0x3996;
    uint8_t op2 = 0x32;
    int32_t t1 = (int32_t)mcu.r[2];
    uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
    mcu.r[2] &= ~0xff;
    mcu.r[2] |= (uint16_t)(t1 & 0xff);
}

void step_pc_3996(void)
{
    mcu.pc = 0x3998;
    uint16_t disp = (uint16_t)(int8_t)0x0a;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t branch = C == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3998(void)
{
    mcu.pc = 0x399a;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
    mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(data) & 0xffu));
}

void step_pc_399a(void)
{
    mcu.pc = 0x399c;
    uint8_t op2 = 0xaa;
    uint32_t t1 = (uint32_t)(mcu.r[5] & 0xff);
    uint32_t t2 = (uint32_t)mcu.r[2];
    t2 &= 0xff;
    t1 *= t2;
    t1 &= 0xffff;
    mcu.r[2] = (uint16_t)t1;
    uint32_t N = (t1 & 0x8000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_399c(void)
{
    mcu.pc = 0x399e;
    uint8_t op2 = 0x10;
    uint32_t data = (uint32_t)mcu.r[4];
    uint32_t data_h = data >> 8;
    uint32_t data_l = data & 0xff;
    data = (data_l << 8) | data_h;
    mcu.r[4] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_399e(void)
{
    mcu.pc = 0x39a0;
    uint8_t op2 = 0x34;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_39a0(void)
{
    mcu.pc = 0x39a2;
    uint16_t disp = (uint16_t)(int8_t)0x06;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_39a2(void)
{
    mcu.pc = 0x39a4;
    uint8_t op2 = 0xaa;
    uint32_t t1 = (uint32_t)(mcu.r[5] & 0xff);
    uint32_t t2 = (uint32_t)mcu.r[2];
    t2 &= 0xff;
    t1 *= t2;
    t1 &= 0xffff;
    mcu.r[2] = (uint16_t)t1;
    uint32_t N = (t1 & 0x8000u) != 0;
    uint32_t Z = (t1 == 0);
    MCU_SetStatus(N, STATUS_N);
    MCU_SetStatus(Z, STATUS_Z);
    MCU_SetStatus(0, STATUS_V);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_39a4(void)
{
    mcu.pc = 0x39a6;
    uint8_t op2 = 0x10;
    uint32_t data = (uint32_t)mcu.r[4];
    uint32_t data_h = data >> 8;
    uint32_t data_l = data & 0xff;
    data = (data_l << 8) | data_h;
    mcu.r[4] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_39a6(void)
{
    mcu.pc = 0x39a8;
    uint8_t op2 = 0x24;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)mcu.r[2];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_39a8(void)
{
    mcu.pc = 0x39aa;
    uint8_t op2 = 0x1b;
    uint32_t data = (uint32_t)mcu.r[4];
    uint32_t C = data & 1;
    data >>= 1;
    mcu.r[4] = (uint16_t)(data);
    MCU_SetStatus(C, STATUS_C);
    MCU_SetStatusCommon(data, 1);
}

void step_pc_39aa(void)
{
    mcu.pc = 0x39ad;
    int32_t t2 = (int32_t)0x80;
    t2 = (t2 << 8) | (int32_t)0x00;
    int32_t t1 = (int32_t)mcu.r[6];
    MCU_SUB_Common(t1, t2, 0, 1);
}

void step_pc_39ad(void)
{
    mcu.pc = 0x39af;
    uint16_t disp = (uint16_t)(int8_t)0x02;
    uint32_t C = (mcu.sr & STATUS_C) != 0;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = (C | Z) == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_39af(void)
{
    mcu.pc = 0x39b1;
    uint8_t op2 = 0x14;
    uint32_t data = (uint32_t)mcu.r[4];
    data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
    mcu.r[4] = (uint16_t)(data);
}

void step_pc_39b1(void)
{
    mcu.pc = 0x39b4;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x20;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x94;
    uint32_t data = (uint32_t)mcu.r[4];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_39b4(void)
{
    mcu.pc = MCU_PopStack();
}

/* 0x7238 selector 5/6 tail 0x3a5a..0x3a9d (entered from the gen-owned
 * 0x3a54/0x3a57 movi r6). Byte-identical to the gen index-4 body
 * 0x3a30..0x3a53 up to 0x3a74; the tail adds the +/-r6 slew clamp. */
void step_pc_3a5a(void)
{
    mcu.pc = 0x3a5c;
    uint8_t op2 = 0x25;
    int32_t t1 = (int32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[5];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[5] = (uint16_t)t1;
}

void step_pc_3a5c(void)
{
    mcu.pc = 0x3a5e;
    uint8_t op2 = 0xa4;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)mcu.r[4];
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
    if (!Z)
    MCU_SetStatus(0, STATUS_Z);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_3a5e(void)
{
    mcu.pc = 0x3a61;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x16;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x25;
    int32_t t1 = (int32_t)mcu.r[5];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t t2 = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[5] = (uint16_t)t1;
}

void step_pc_3a61(void)
{
    mcu.pc = 0x3a65;
    uint32_t odata = (uint32_t)0x00;
    odata = (odata << 8) | (uint32_t)0x00;
    uint8_t op2 = 0xa4;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = odata;
    int32_t C = (mcu.sr & STATUS_C) != 0;
    int32_t Z = (mcu.sr & STATUS_Z) != 0;
    t1 = MCU_ADD_Common(t1, (int32_t)t2, C, 1);
    if (!Z)
    MCU_SetStatus(0, STATUS_Z);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_3a65(void)
{
    mcu.pc = 0x3a67;
    uint8_t op2 = 0x16;
    uint32_t data = (uint32_t)mcu.r[4];
    MCU_SetStatusCommon(data, 1);
    MCU_SetStatus(0, STATUS_C);
}

void step_pc_3a67(void)
{
    mcu.pc = 0x3a69;
    uint16_t disp = (uint16_t)(int8_t)0x0b;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3a69(void)
{
    mcu.pc = 0x3a6d;
    uint32_t oea = ((uint32_t)mcu.br << 8) | (uint32_t)0x3e;
    oea &= 0xffff;
    uint32_t oep = 0;
    uint8_t op2 = 0x06;
    uint32_t d = (uint32_t)(int8_t)0x1e;
    MCU_Write(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint8_t)(d));
    MCU_SetStatusCommon(d, 0);
}

void step_pc_3a6d(void)
{
    mcu.pc = 0x3a6f;
    uint16_t addr = (uint16_t)(mcu.br << 8);
    addr |= 0x34;
    uint32_t data = (uint32_t)MCU_Read(addr);
    mcu.r[4] &= ~0xff;
    mcu.r[4] |= (uint16_t)data;
    MCU_SetStatusCommon(data, 0);
}

void step_pc_3a6f(void)
{
    mcu.pc = 0x3a71;
    uint16_t addr = (uint16_t)(mcu.br << 8);
    addr |= 0x3a;
    if (addr & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint16_t data = MCU_Read16(addr);
    mcu.r[4] = data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3a71(void)
{
    mcu.pc = 0x3a74;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x1c;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x94;
    uint32_t data = (uint32_t)mcu.r[4];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3a74(void)
{
    mcu.pc = 0x3a77;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x16;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x95;
    uint32_t data = (uint32_t)mcu.r[5];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3a77(void)
{
    mcu.pc = 0x3a7a;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x1e;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x84;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[4] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3a7a(void)
{
    mcu.pc = 0x3a7d;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x1c;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x85;
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    uint32_t data = (uint32_t)MCU_Read16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea));
    mcu.r[5] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3a7d(void)
{
    mcu.pc = 0x3a7f;
    uint8_t op2 = 0x75;
    int32_t t1 = (int32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[4];
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_3a7f(void)
{
    mcu.pc = 0x3a81;
    uint16_t disp = (uint16_t)(int8_t)0x16;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = Z == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3a81(void)
{
    mcu.pc = 0x3a83;
    uint16_t disp = (uint16_t)(int8_t)0x0a;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t V = (mcu.sr & STATUS_V) != 0;
    uint32_t branch = (N ^ V) == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3a83(void)
{
    mcu.pc = 0x3a85;
    uint8_t op2 = 0x34;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)mcu.r[6];
    t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_3a85(void)
{
    mcu.pc = 0x3a87;
    uint16_t disp = (uint16_t)(int8_t)0x0e;
    uint32_t V = (mcu.sr & STATUS_V) != 0;
    uint32_t branch = V == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3a87(void)
{
    mcu.pc = 0x3a89;
    uint8_t op2 = 0x75;
    int32_t t1 = (int32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[4];
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_3a89(void)
{
    mcu.pc = 0x3a8b;
    uint16_t disp = (uint16_t)(int8_t)0x0a;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t V = (mcu.sr & STATUS_V) != 0;
    uint32_t Z = (mcu.sr & STATUS_Z) != 0;
    uint32_t branch = (Z | (N ^ V)) == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3a8b(void)
{
    mcu.pc = 0x3a8d;
    uint16_t disp = (uint16_t)(int8_t)0x0a;
    uint32_t branch = 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3a8d(void)
{
    mcu.pc = 0x3a8f;
    uint8_t op2 = 0x24;
    int32_t t1 = (int32_t)mcu.r[4];
    uint32_t t2 = (uint32_t)mcu.r[6];
    t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
    mcu.r[4] = (uint16_t)t1;
}

void step_pc_3a8f(void)
{
    mcu.pc = 0x3a91;
    uint16_t disp = (uint16_t)(int8_t)0x04;
    uint32_t V = (mcu.sr & STATUS_V) != 0;
    uint32_t branch = V == 1;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3a91(void)
{
    mcu.pc = 0x3a93;
    uint8_t op2 = 0x75;
    int32_t t1 = (int32_t)mcu.r[5];
    uint32_t t2 = (uint32_t)mcu.r[4];
    MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
}

void step_pc_3a93(void)
{
    mcu.pc = 0x3a95;
    uint16_t disp = (uint16_t)(int8_t)0x02;
    uint32_t N = (mcu.sr & STATUS_N) != 0;
    uint32_t V = (mcu.sr & STATUS_V) != 0;
    uint32_t branch = (N ^ V) == 0;
    if (branch)
    mcu.pc += disp;
}

void step_pc_3a95(void)
{
    mcu.pc = 0x3a97;
    uint8_t op2 = 0x84;
    uint32_t data = (uint32_t)mcu.r[5];
    mcu.r[4] = (uint16_t)data;
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3a97(void)
{
    mcu.pc = 0x3a9a;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x1e;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x94;
    uint32_t data = (uint32_t)mcu.r[4];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3a9a(void)
{
    mcu.pc = 0x3a9d;
    uint32_t oea = (uint32_t)mcu.r[1] + (uint32_t)(int8_t)0x20;
    oea &= 0xffff;
    uint32_t oep = (uint32_t)(MCU_GetPageForRegister(1) & 0xff);
    uint8_t op2 = 0x94;
    uint32_t data = (uint32_t)mcu.r[4];
    if (oea & 1)
    MCU_Interrupt_Exception(EXCEPTION_SOURCE_ADDRESS_ERROR);
    MCU_Write16(MCU_GetAddress((uint8_t)oep, (uint16_t)oea), (uint16_t)(data));
    MCU_SetStatusCommon(data, 1);
}

void step_pc_3a9d(void)
{
    mcu.pc = MCU_PopStack();
}

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from
 * MK2CPP_HandFillTables after the built-in modules. */
void pcm_interp_fill(void)
{
    MK2CPP_HandRegister(0x00003709u, &mk2c::step_pc_3709);
    MK2CPP_HandRegister(0x0000370cu, &mk2c::step_pc_370c);
    MK2CPP_HandRegister(0x0000370eu, &mk2c::step_pc_370e);
    MK2CPP_HandRegister(0x00003711u, &mk2c::step_pc_3711);
    MK2CPP_HandRegister(0x00003713u, &mk2c::step_pc_3713);
    MK2CPP_HandRegister(0x00003717u, &mk2c::step_pc_3717);
    MK2CPP_HandRegister(0x0000371bu, &mk2c::step_pc_371b);
    MK2CPP_HandRegister(0x0000371fu, &mk2c::step_pc_371f);
    MK2CPP_HandRegister(0x00003723u, &mk2c::step_pc_3723);
    MK2CPP_HandRegister(0x00003728u, &mk2c::step_pc_3728);
    MK2CPP_HandRegister(0x0000372au, &mk2c::step_pc_372a);
    MK2CPP_HandRegister(0x0000372eu, &mk2c::step_pc_372e);
    MK2CPP_HandRegister(0x00003730u, &mk2c::step_pc_3730);
    MK2CPP_HandRegister(0x00003734u, &mk2c::step_pc_3734);
    MK2CPP_HandRegister(0x00003736u, &mk2c::step_pc_3736);
    MK2CPP_HandRegister(0x0000373au, &mk2c::step_pc_373a);
    MK2CPP_HandRegister(0x0000373cu, &mk2c::step_pc_373c);
    MK2CPP_HandRegister(0x0000373fu, &mk2c::step_pc_373f);
    MK2CPP_HandRegister(0x00003742u, &mk2c::step_pc_3742);
    MK2CPP_HandRegister(0x00003744u, &mk2c::step_pc_3744);
    MK2CPP_HandRegister(0x00003746u, &mk2c::step_pc_3746);
    MK2CPP_HandRegister(0x0000374au, &mk2c::step_pc_374a);
    MK2CPP_HandRegister(0x0000374du, &mk2c::step_pc_374d);
    MK2CPP_HandRegister(0x00003750u, &mk2c::step_pc_3750);
    MK2CPP_HandRegister(0x00003752u, &mk2c::step_pc_3752);
    MK2CPP_HandRegister(0x00003754u, &mk2c::step_pc_3754);
    MK2CPP_HandRegister(0x00003756u, &mk2c::step_pc_3756);
    MK2CPP_HandRegister(0x00003758u, &mk2c::step_pc_3758);
    MK2CPP_HandRegister(0x0000375cu, &mk2c::step_pc_375c);
    MK2CPP_HandRegister(0x0000375eu, &mk2c::step_pc_375e);
    MK2CPP_HandRegister(0x00003762u, &mk2c::step_pc_3762);
    MK2CPP_HandRegister(0x00003765u, &mk2c::step_pc_3765);
    MK2CPP_HandRegister(0x00003768u, &mk2c::step_pc_3768);
    MK2CPP_HandRegister(0x0000376cu, &mk2c::step_pc_376c);
    MK2CPP_HandRegister(0x0000376du, &mk2c::step_pc_376d);
    MK2CPP_HandRegister(0x0000376eu, &mk2c::step_pc_376e);
    MK2CPP_HandRegister(0x0000376fu, &mk2c::step_pc_376f);
    MK2CPP_HandRegister(0x00003770u, &mk2c::step_pc_3770);
    MK2CPP_HandRegister(0x00003774u, &mk2c::step_pc_3774);
    MK2CPP_HandRegister(0x00003777u, &mk2c::step_pc_3777);
    MK2CPP_HandRegister(0x00003779u, &mk2c::step_pc_3779);
    MK2CPP_HandRegister(0x0000377au, &mk2c::step_pc_377a);
    MK2CPP_HandRegister(0x0000377du, &mk2c::step_pc_377d);
    MK2CPP_HandRegister(0x00003780u, &mk2c::step_pc_3780);
    MK2CPP_HandRegister(0x00003783u, &mk2c::step_pc_3783);
    MK2CPP_HandRegister(0x00003786u, &mk2c::step_pc_3786);
    MK2CPP_HandRegister(0x00003788u, &mk2c::step_pc_3788);
    MK2CPP_HandRegister(0x0000378au, &mk2c::step_pc_378a);
    MK2CPP_HandRegister(0x0000378cu, &mk2c::step_pc_378c);
    MK2CPP_HandRegister(0x0000378eu, &mk2c::step_pc_378e);
    MK2CPP_HandRegister(0x00003790u, &mk2c::step_pc_3790);
    MK2CPP_HandRegister(0x00003792u, &mk2c::step_pc_3792);
    MK2CPP_HandRegister(0x00003794u, &mk2c::step_pc_3794);
    MK2CPP_HandRegister(0x00003796u, &mk2c::step_pc_3796);
    MK2CPP_HandRegister(0x00003799u, &mk2c::step_pc_3799);
    MK2CPP_HandRegister(0x0000379bu, &mk2c::step_pc_379b);
    MK2CPP_HandRegister(0x0000379eu, &mk2c::step_pc_379e);
    MK2CPP_HandRegister(0x000037a1u, &mk2c::step_pc_37a1);
    MK2CPP_HandRegister(0x000037a5u, &mk2c::step_pc_37a5);
    MK2CPP_HandRegister(0x000037a7u, &mk2c::step_pc_37a7);
    MK2CPP_HandRegister(0x000037a9u, &mk2c::step_pc_37a9);
    MK2CPP_HandRegister(0x000037acu, &mk2c::step_pc_37ac);
    MK2CPP_HandRegister(0x000037afu, &mk2c::step_pc_37af);
    MK2CPP_HandRegister(0x000037b1u, &mk2c::step_pc_37b1);
    MK2CPP_HandRegister(0x000037b3u, &mk2c::step_pc_37b3);
    MK2CPP_HandRegister(0x000037b6u, &mk2c::step_pc_37b6);
    MK2CPP_HandRegister(0x000037b8u, &mk2c::step_pc_37b8);
    MK2CPP_HandRegister(0x000037bau, &mk2c::step_pc_37ba);
    MK2CPP_HandRegister(0x000037bcu, &mk2c::step_pc_37bc);
    MK2CPP_HandRegister(0x000037beu, &mk2c::step_pc_37be);
    MK2CPP_HandRegister(0x000037c0u, &mk2c::step_pc_37c0);
    MK2CPP_HandRegister(0x000037c2u, &mk2c::step_pc_37c2);
    MK2CPP_HandRegister(0x000037c4u, &mk2c::step_pc_37c4);
    MK2CPP_HandRegister(0x000037c6u, &mk2c::step_pc_37c6);
    MK2CPP_HandRegister(0x000037c8u, &mk2c::step_pc_37c8);
    MK2CPP_HandRegister(0x000037cau, &mk2c::step_pc_37ca);
    MK2CPP_HandRegister(0x000037ccu, &mk2c::step_pc_37cc);
    MK2CPP_HandRegister(0x000037ceu, &mk2c::step_pc_37ce);
    MK2CPP_HandRegister(0x000037d0u, &mk2c::step_pc_37d0);
    MK2CPP_HandRegister(0x000037d2u, &mk2c::step_pc_37d2);
    MK2CPP_HandRegister(0x000037d4u, &mk2c::step_pc_37d4);
    MK2CPP_HandRegister(0x000037d6u, &mk2c::step_pc_37d6);
    MK2CPP_HandRegister(0x000037d8u, &mk2c::step_pc_37d8);
    MK2CPP_HandRegister(0x000037dau, &mk2c::step_pc_37da);
    MK2CPP_HandRegister(0x000037dcu, &mk2c::step_pc_37dc);
    MK2CPP_HandRegister(0x000037deu, &mk2c::step_pc_37de);
    MK2CPP_HandRegister(0x000037e0u, &mk2c::step_pc_37e0);
    MK2CPP_HandRegister(0x000037e2u, &mk2c::step_pc_37e2);
    MK2CPP_HandRegister(0x000037e4u, &mk2c::step_pc_37e4);
    MK2CPP_HandRegister(0x000037e8u, &mk2c::step_pc_37e8);
    MK2CPP_HandRegister(0x000037eau, &mk2c::step_pc_37ea);
    MK2CPP_HandRegister(0x000037ecu, &mk2c::step_pc_37ec);
    MK2CPP_HandRegister(0x000037eeu, &mk2c::step_pc_37ee);
    MK2CPP_HandRegister(0x000037f2u, &mk2c::step_pc_37f2);
    MK2CPP_HandRegister(0x000037f5u, &mk2c::step_pc_37f5);
    MK2CPP_HandRegister(0x000037f7u, &mk2c::step_pc_37f7);
    MK2CPP_HandRegister(0x000037fbu, &mk2c::step_pc_37fb);
    MK2CPP_HandRegister(0x000037feu, &mk2c::step_pc_37fe);
    MK2CPP_HandRegister(0x00003801u, &mk2c::step_pc_3801);
    MK2CPP_HandRegister(0x00003804u, &mk2c::step_pc_3804);
    MK2CPP_HandRegister(0x00003807u, &mk2c::step_pc_3807);
    MK2CPP_HandRegister(0x00003809u, &mk2c::step_pc_3809);
    MK2CPP_HandRegister(0x0000380du, &mk2c::step_pc_380d);
    MK2CPP_HandRegister(0x00003811u, &mk2c::step_pc_3811);
    MK2CPP_HandRegister(0x00003815u, &mk2c::step_pc_3815);
    MK2CPP_HandRegister(0x00003819u, &mk2c::step_pc_3819);
    MK2CPP_HandRegister(0x0000381eu, &mk2c::step_pc_381e);
    MK2CPP_HandRegister(0x00003820u, &mk2c::step_pc_3820);
    MK2CPP_HandRegister(0x00003824u, &mk2c::step_pc_3824);
    MK2CPP_HandRegister(0x00003826u, &mk2c::step_pc_3826);
    MK2CPP_HandRegister(0x0000382au, &mk2c::step_pc_382a);
    MK2CPP_HandRegister(0x0000382cu, &mk2c::step_pc_382c);
    MK2CPP_HandRegister(0x00003830u, &mk2c::step_pc_3830);
    MK2CPP_HandRegister(0x00003832u, &mk2c::step_pc_3832);
    MK2CPP_HandRegister(0x00003835u, &mk2c::step_pc_3835);
    MK2CPP_HandRegister(0x00003838u, &mk2c::step_pc_3838);
    MK2CPP_HandRegister(0x0000383bu, &mk2c::step_pc_383b);
    MK2CPP_HandRegister(0x0000383eu, &mk2c::step_pc_383e);
    MK2CPP_HandRegister(0x00003841u, &mk2c::step_pc_3841);
    MK2CPP_HandRegister(0x00003844u, &mk2c::step_pc_3844);
    MK2CPP_HandRegister(0x00003847u, &mk2c::step_pc_3847);
    MK2CPP_HandRegister(0x0000384au, &mk2c::step_pc_384a);
    MK2CPP_HandRegister(0x0000384du, &mk2c::step_pc_384d);
    MK2CPP_HandRegister(0x00003850u, &mk2c::step_pc_3850);
    MK2CPP_HandRegister(0x00003853u, &mk2c::step_pc_3853);
    MK2CPP_HandRegister(0x00003856u, &mk2c::step_pc_3856);
    MK2CPP_HandRegister(0x00003859u, &mk2c::step_pc_3859);
    MK2CPP_HandRegister(0x0000385cu, &mk2c::step_pc_385c);
    MK2CPP_HandRegister(0x0000385fu, &mk2c::step_pc_385f);
    MK2CPP_HandRegister(0x00003862u, &mk2c::step_pc_3862);
    MK2CPP_HandRegister(0x00003865u, &mk2c::step_pc_3865);
    MK2CPP_HandRegister(0x00003868u, &mk2c::step_pc_3868);
    MK2CPP_HandRegister(0x0000386bu, &mk2c::step_pc_386b);
    MK2CPP_HandRegister(0x0000386eu, &mk2c::step_pc_386e);
    MK2CPP_HandRegister(0x0000386fu, &mk2c::step_pc_386f);
    MK2CPP_HandRegister(0x00003872u, &mk2c::step_pc_3872);
    MK2CPP_HandRegister(0x00003874u, &mk2c::step_pc_3874);
    MK2CPP_HandRegister(0x00003876u, &mk2c::step_pc_3876);
    MK2CPP_HandRegister(0x0000387au, &mk2c::step_pc_387a);
    MK2CPP_HandRegister(0x0000387du, &mk2c::step_pc_387d);
    MK2CPP_HandRegister(0x00003880u, &mk2c::step_pc_3880);
    MK2CPP_HandRegister(0x00003882u, &mk2c::step_pc_3882);
    MK2CPP_HandRegister(0x00003884u, &mk2c::step_pc_3884);
    MK2CPP_HandRegister(0x00003886u, &mk2c::step_pc_3886);
    MK2CPP_HandRegister(0x00003888u, &mk2c::step_pc_3888);
    MK2CPP_HandRegister(0x0000388cu, &mk2c::step_pc_388c);
    MK2CPP_HandRegister(0x0000388eu, &mk2c::step_pc_388e);
    MK2CPP_HandRegister(0x00003892u, &mk2c::step_pc_3892);
    MK2CPP_HandRegister(0x00003895u, &mk2c::step_pc_3895);
    MK2CPP_HandRegister(0x00003898u, &mk2c::step_pc_3898);
    MK2CPP_HandRegister(0x0000389cu, &mk2c::step_pc_389c);
    MK2CPP_HandRegister(0x0000389du, &mk2c::step_pc_389d);
    MK2CPP_HandRegister(0x0000389eu, &mk2c::step_pc_389e);
    MK2CPP_HandRegister(0x0000389fu, &mk2c::step_pc_389f);
    MK2CPP_HandRegister(0x000038a0u, &mk2c::step_pc_38a0);
    MK2CPP_HandRegister(0x000038a4u, &mk2c::step_pc_38a4);
    MK2CPP_HandRegister(0x000038a7u, &mk2c::step_pc_38a7);
    MK2CPP_HandRegister(0x000038a9u, &mk2c::step_pc_38a9);
    MK2CPP_HandRegister(0x000038aau, &mk2c::step_pc_38aa);
    MK2CPP_HandRegister(0x000038acu, &mk2c::step_pc_38ac);
    MK2CPP_HandRegister(0x000038b0u, &mk2c::step_pc_38b0);
    MK2CPP_HandRegister(0x000038b3u, &mk2c::step_pc_38b3);
    MK2CPP_HandRegister(0x000038b6u, &mk2c::step_pc_38b6);
    MK2CPP_HandRegister(0x000038b8u, &mk2c::step_pc_38b8);
    MK2CPP_HandRegister(0x000038bbu, &mk2c::step_pc_38bb);
    MK2CPP_HandRegister(0x000038bfu, &mk2c::step_pc_38bf);
    MK2CPP_HandRegister(0x000038c1u, &mk2c::step_pc_38c1);
    MK2CPP_HandRegister(0x000038c5u, &mk2c::step_pc_38c5);
    MK2CPP_HandRegister(0x000038c7u, &mk2c::step_pc_38c7);
    MK2CPP_HandRegister(0x000038c9u, &mk2c::step_pc_38c9);
    MK2CPP_HandRegister(0x000038ccu, &mk2c::step_pc_38cc);
    MK2CPP_HandRegister(0x000038cfu, &mk2c::step_pc_38cf);
    MK2CPP_HandRegister(0x000038d1u, &mk2c::step_pc_38d1);
    MK2CPP_HandRegister(0x000038d6u, &mk2c::step_pc_38d6);
    MK2CPP_HandRegister(0x000038d9u, &mk2c::step_pc_38d9);
    MK2CPP_HandRegister(0x000038dcu, &mk2c::step_pc_38dc);
    MK2CPP_HandRegister(0x000038deu, &mk2c::step_pc_38de);
    MK2CPP_HandRegister(0x000038e1u, &mk2c::step_pc_38e1);
    MK2CPP_HandRegister(0x000038e5u, &mk2c::step_pc_38e5);
    MK2CPP_HandRegister(0x000038e7u, &mk2c::step_pc_38e7);
    MK2CPP_HandRegister(0x000038ebu, &mk2c::step_pc_38eb);
    MK2CPP_HandRegister(0x000038edu, &mk2c::step_pc_38ed);
    MK2CPP_HandRegister(0x000038efu, &mk2c::step_pc_38ef);
    MK2CPP_HandRegister(0x000038f2u, &mk2c::step_pc_38f2);
    MK2CPP_HandRegister(0x000038f5u, &mk2c::step_pc_38f5);
    MK2CPP_HandRegister(0x000038f8u, &mk2c::step_pc_38f8);
    MK2CPP_HandRegister(0x000038fau, &mk2c::step_pc_38fa);
    MK2CPP_HandRegister(0x000038fcu, &mk2c::step_pc_38fc);
    MK2CPP_HandRegister(0x000038feu, &mk2c::step_pc_38fe);
    MK2CPP_HandRegister(0x00003900u, &mk2c::step_pc_3900);
    MK2CPP_HandRegister(0x00003902u, &mk2c::step_pc_3902);
    MK2CPP_HandRegister(0x00003904u, &mk2c::step_pc_3904);
    MK2CPP_HandRegister(0x00003907u, &mk2c::step_pc_3907);
    MK2CPP_HandRegister(0x0000390au, &mk2c::step_pc_390a);
    MK2CPP_HandRegister(0x0000390cu, &mk2c::step_pc_390c);
    MK2CPP_HandRegister(0x0000390eu, &mk2c::step_pc_390e);
    MK2CPP_HandRegister(0x00003910u, &mk2c::step_pc_3910);
    MK2CPP_HandRegister(0x00003912u, &mk2c::step_pc_3912);
    MK2CPP_HandRegister(0x00003914u, &mk2c::step_pc_3914);
    MK2CPP_HandRegister(0x00003916u, &mk2c::step_pc_3916);
    MK2CPP_HandRegister(0x00003919u, &mk2c::step_pc_3919);
    MK2CPP_HandRegister(0x0000391du, &mk2c::step_pc_391d);
    MK2CPP_HandRegister(0x0000391fu, &mk2c::step_pc_391f);
    MK2CPP_HandRegister(0x00003921u, &mk2c::step_pc_3921);
    MK2CPP_HandRegister(0x00003923u, &mk2c::step_pc_3923);
    MK2CPP_HandRegister(0x00003925u, &mk2c::step_pc_3925);
    MK2CPP_HandRegister(0x00003927u, &mk2c::step_pc_3927);
    MK2CPP_HandRegister(0x00003929u, &mk2c::step_pc_3929);
    MK2CPP_HandRegister(0x0000392cu, &mk2c::step_pc_392c);
    MK2CPP_HandRegister(0x0000392eu, &mk2c::step_pc_392e);
    MK2CPP_HandRegister(0x00003931u, &mk2c::step_pc_3931);
    MK2CPP_HandRegister(0x00003934u, &mk2c::step_pc_3934);
    MK2CPP_HandRegister(0x00003937u, &mk2c::step_pc_3937);
    MK2CPP_HandRegister(0x0000393au, &mk2c::step_pc_393a);
    MK2CPP_HandRegister(0x0000393du, &mk2c::step_pc_393d);
    MK2CPP_HandRegister(0x00003940u, &mk2c::step_pc_3940);
    MK2CPP_HandRegister(0x00003942u, &mk2c::step_pc_3942);
    MK2CPP_HandRegister(0x00003945u, &mk2c::step_pc_3945);
    MK2CPP_HandRegister(0x00003947u, &mk2c::step_pc_3947);
    MK2CPP_HandRegister(0x0000394bu, &mk2c::step_pc_394b);
    MK2CPP_HandRegister(0x0000394eu, &mk2c::step_pc_394e);
    MK2CPP_HandRegister(0x00003950u, &mk2c::step_pc_3950);
    MK2CPP_HandRegister(0x00003952u, &mk2c::step_pc_3952);
    MK2CPP_HandRegister(0x00003955u, &mk2c::step_pc_3955);
    MK2CPP_HandRegister(0x00003957u, &mk2c::step_pc_3957);
    MK2CPP_HandRegister(0x00003959u, &mk2c::step_pc_3959);
    MK2CPP_HandRegister(0x0000395bu, &mk2c::step_pc_395b);
    MK2CPP_HandRegister(0x0000395du, &mk2c::step_pc_395d);
    MK2CPP_HandRegister(0x00003960u, &mk2c::step_pc_3960);
    MK2CPP_HandRegister(0x00003962u, &mk2c::step_pc_3962);
    MK2CPP_HandRegister(0x00003965u, &mk2c::step_pc_3965);
    MK2CPP_HandRegister(0x00003969u, &mk2c::step_pc_3969);
    MK2CPP_HandRegister(0x0000396cu, &mk2c::step_pc_396c);
    MK2CPP_HandRegister(0x00003970u, &mk2c::step_pc_3970);
    MK2CPP_HandRegister(0x00003972u, &mk2c::step_pc_3972);
    MK2CPP_HandRegister(0x00003975u, &mk2c::step_pc_3975);
    MK2CPP_HandRegister(0x00003978u, &mk2c::step_pc_3978);
    MK2CPP_HandRegister(0x0000397au, &mk2c::step_pc_397a);
    MK2CPP_HandRegister(0x0000397eu, &mk2c::step_pc_397e);
    MK2CPP_HandRegister(0x00003980u, &mk2c::step_pc_3980);
    MK2CPP_HandRegister(0x00003982u, &mk2c::step_pc_3982);
    MK2CPP_HandRegister(0x00003984u, &mk2c::step_pc_3984);
    MK2CPP_HandRegister(0x00003986u, &mk2c::step_pc_3986);
    MK2CPP_HandRegister(0x00003988u, &mk2c::step_pc_3988);
    MK2CPP_HandRegister(0x0000398au, &mk2c::step_pc_398a);
    MK2CPP_HandRegister(0x0000398eu, &mk2c::step_pc_398e);
    MK2CPP_HandRegister(0x00003990u, &mk2c::step_pc_3990);
    MK2CPP_HandRegister(0x00003994u, &mk2c::step_pc_3994);
    MK2CPP_HandRegister(0x00003996u, &mk2c::step_pc_3996);
    MK2CPP_HandRegister(0x00003998u, &mk2c::step_pc_3998);
    MK2CPP_HandRegister(0x0000399au, &mk2c::step_pc_399a);
    MK2CPP_HandRegister(0x0000399cu, &mk2c::step_pc_399c);
    MK2CPP_HandRegister(0x0000399eu, &mk2c::step_pc_399e);
    MK2CPP_HandRegister(0x000039a0u, &mk2c::step_pc_39a0);
    MK2CPP_HandRegister(0x000039a2u, &mk2c::step_pc_39a2);
    MK2CPP_HandRegister(0x000039a4u, &mk2c::step_pc_39a4);
    MK2CPP_HandRegister(0x000039a6u, &mk2c::step_pc_39a6);
    MK2CPP_HandRegister(0x000039a8u, &mk2c::step_pc_39a8);
    MK2CPP_HandRegister(0x000039aau, &mk2c::step_pc_39aa);
    MK2CPP_HandRegister(0x000039adu, &mk2c::step_pc_39ad);
    MK2CPP_HandRegister(0x000039afu, &mk2c::step_pc_39af);
    MK2CPP_HandRegister(0x000039b1u, &mk2c::step_pc_39b1);
    MK2CPP_HandRegister(0x000039b4u, &mk2c::step_pc_39b4);
    MK2CPP_HandRegister(0x00003a5au, &mk2c::step_pc_3a5a);
    MK2CPP_HandRegister(0x00003a5cu, &mk2c::step_pc_3a5c);
    MK2CPP_HandRegister(0x00003a5eu, &mk2c::step_pc_3a5e);
    MK2CPP_HandRegister(0x00003a61u, &mk2c::step_pc_3a61);
    MK2CPP_HandRegister(0x00003a65u, &mk2c::step_pc_3a65);
    MK2CPP_HandRegister(0x00003a67u, &mk2c::step_pc_3a67);
    MK2CPP_HandRegister(0x00003a69u, &mk2c::step_pc_3a69);
    MK2CPP_HandRegister(0x00003a6du, &mk2c::step_pc_3a6d);
    MK2CPP_HandRegister(0x00003a6fu, &mk2c::step_pc_3a6f);
    MK2CPP_HandRegister(0x00003a71u, &mk2c::step_pc_3a71);
    MK2CPP_HandRegister(0x00003a74u, &mk2c::step_pc_3a74);
    MK2CPP_HandRegister(0x00003a77u, &mk2c::step_pc_3a77);
    MK2CPP_HandRegister(0x00003a7au, &mk2c::step_pc_3a7a);
    MK2CPP_HandRegister(0x00003a7du, &mk2c::step_pc_3a7d);
    MK2CPP_HandRegister(0x00003a7fu, &mk2c::step_pc_3a7f);
    MK2CPP_HandRegister(0x00003a81u, &mk2c::step_pc_3a81);
    MK2CPP_HandRegister(0x00003a83u, &mk2c::step_pc_3a83);
    MK2CPP_HandRegister(0x00003a85u, &mk2c::step_pc_3a85);
    MK2CPP_HandRegister(0x00003a87u, &mk2c::step_pc_3a87);
    MK2CPP_HandRegister(0x00003a89u, &mk2c::step_pc_3a89);
    MK2CPP_HandRegister(0x00003a8bu, &mk2c::step_pc_3a8b);
    MK2CPP_HandRegister(0x00003a8du, &mk2c::step_pc_3a8d);
    MK2CPP_HandRegister(0x00003a8fu, &mk2c::step_pc_3a8f);
    MK2CPP_HandRegister(0x00003a91u, &mk2c::step_pc_3a91);
    MK2CPP_HandRegister(0x00003a93u, &mk2c::step_pc_3a93);
    MK2CPP_HandRegister(0x00003a95u, &mk2c::step_pc_3a95);
    MK2CPP_HandRegister(0x00003a97u, &mk2c::step_pc_3a97);
    MK2CPP_HandRegister(0x00003a9au, &mk2c::step_pc_3a9a);
    MK2CPP_HandRegister(0x00003a9du, &mk2c::step_pc_3a9d);
}

namespace {

/* Self-registration (parallel-safe): no shared aggregator file is edited. */
struct PcmInterpSelfRegister
{
    PcmInterpSelfRegister() { hand_register_module(&pcm_interp_fill); }
};

PcmInterpSelfRegister g_pcm_interp_self_register;

} /* anonymous namespace */
} /* namespace mk2c */
