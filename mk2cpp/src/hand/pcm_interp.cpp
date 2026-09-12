/*
 * HAND pcm/pcm_interp -- M4 P2 REC0/REC1 fixed-point integration + output
 * (C12 interp_entry 0x3709.., C13 interp 0x38b0..), one host step per H8
 * instruction (M4 out/m4/18_closure_gap.md 4.2 rows 25-26; docs 07 C12/C13).
 * rom1 sha256 8a1eb33c7599b746c0c50283e4349a1bb1773b5c0ec0e9661219bf6c067d2042
 * rom2 sha256 a4c9fd821059054c7e7681d61f49ce6f42ed2fe407a7ec1ba0dfdc9722582ce0
 * hand_rev 2
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
 * Translation form: one MK2CPP_HandRegisterRoutine entry per instruction PC
 * (L0, always returns 1) so the host keeps its per-instruction interrupt
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
 * declaration pattern as pcm_enable.cpp / pcm_misc.cpp). */
int32_t MCU_ADD_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
int32_t MCU_SUB_Common(int32_t t1, int32_t t2, int32_t c_bit, uint32_t siz);
void MCU_SetStatusCommon(uint32_t val, uint32_t siz);

namespace mk2c {
namespace {

/* One host step: dispatch on the PC the host is at. Each entry executes
 * exactly one H8 instruction and returns 1. Bodies are the gen per-PC
 * decode with the operand bytes folded in and mcu.pc advanced explicitly. */
uint32_t step_interp(void)
{
    switch (mcu.pc)
    {
    case 0x3709:
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
    break;

    case 0x370c:
    {
        mcu.pc = 0x370e;
        uint16_t disp = (uint16_t)(int8_t)0x6c;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x370e:
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
    break;

    case 0x3711:
    {
        mcu.pc = 0x3713;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;

    case 0x3713:
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
    break;

    case 0x3717:
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
    break;

    case 0x371b:
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
    break;

    case 0x371f:
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
    break;

    case 0x3723:
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
    break;

    case 0x3728:
    {
        mcu.pc = 0x372a;
        uint16_t disp = (uint16_t)(int8_t)0x15;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x372a:
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
    break;

    case 0x372e:
    {
        mcu.pc = 0x3730;
        uint16_t disp = (uint16_t)(int8_t)0x0f;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3730:
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
    break;

    case 0x3734:
    {
        mcu.pc = 0x3736;
        uint16_t disp = (uint16_t)(int8_t)0x09;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3736:
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
    break;

    case 0x373a:
    {
        mcu.pc = 0x373c;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x373c:
    {
        mcu.pc = 0x373f;
        uint16_t disp = (uint16_t)(0x03 << 8);
        disp |= 0x89;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x373f:
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
    break;

    case 0x3742:
    {
        mcu.pc = 0x3744;
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3744:
    {
        mcu.pc = 0x3746;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;

    case 0x3746:
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
    break;

    case 0x374a:
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
    break;

    case 0x374d:
    {
        mcu.pc = 0x3750;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x1b;
        mcu.r[1] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3750:
    {
        mcu.pc = 0x3752;
        uint8_t op2 = 0x76;
        int32_t t1 = (int32_t)mcu.r[6];
        uint32_t t2 = (uint32_t)mcu.r[1];
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;

    case 0x3752:
    {
        mcu.pc = 0x3754;
        uint16_t disp = (uint16_t)(int8_t)0x0e;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3754:
    {
        mcu.pc = 0x3756;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3756:
    {
        mcu.pc = 0x3758;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;

    case 0x3758:
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
    break;

    case 0x375c:
    {
        mcu.pc = 0x375e;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x375e:
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
    break;

    case 0x3762:
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
    break;

    case 0x3765:
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
    break;

    case 0x3768:
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
    break;

    case 0x376c:
    {
        mcu.pc = 0x376d;
        /* nop */
    }
    break;

    case 0x376d:
    {
        mcu.pc = 0x376e;
        /* nop */
    }
    break;

    case 0x376e:
    {
        mcu.pc = 0x376f;
        /* nop */
    }
    break;

    case 0x376f:
    {
        mcu.pc = 0x3770;
        /* nop */
    }
    break;

    case 0x3770:
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
    break;

    case 0x3774:
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
    break;

    case 0x3777:
    {
        mcu.pc = 0x3779;
        uint16_t disp = (uint16_t)(int8_t)0x01;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3779:
    {
        mcu.pc = MCU_PopStack();
    }
    break;

    case 0x377a:
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
    break;

    case 0x377d:
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
    break;

    case 0x3780:
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
    break;

    case 0x3783:
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
    break;

    case 0x3786:
    {
        mcu.pc = 0x3788;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3788:
    {
        mcu.pc = 0x378a;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x378a:
    {
        mcu.pc = 0x378c;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x378c:
    {
        mcu.pc = 0x378e;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x378e:
    {
        mcu.pc = 0x3790;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3790:
    {
        mcu.pc = 0x3792;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x3792:
    {
        mcu.pc = 0x3794;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3794:
    {
        mcu.pc = 0x3796;
        uint8_t data = 0x7f;
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;

    case 0x3796:
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
    break;

    case 0x3799:
    {
        mcu.pc = 0x379b;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x379b:
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
    break;

    case 0x379e:
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
    break;

    case 0x37a1:
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
    break;

    case 0x37a5:
    {
        mcu.pc = 0x37a7;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        MCU_SetStatusCommon(data, 0);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x37a7:
    {
        mcu.pc = 0x37a9;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37a9:
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
    break;

    case 0x37ac:
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
    break;

    case 0x37af:
    {
        mcu.pc = 0x37b1;
        uint16_t disp = (uint16_t)(int8_t)0x29;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37b1:
    {
        mcu.pc = 0x37b3;
        uint16_t disp = (uint16_t)(int8_t)0x1b;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37b3:
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
    break;

    case 0x37b6:
    {
        mcu.pc = 0x37b8;
        uint16_t disp = (uint16_t)(int8_t)0x0c;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37b8:
    {
        mcu.pc = 0x37ba;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;

    case 0x37ba:
    {
        mcu.pc = 0x37bc;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x37bc:
    {
        mcu.pc = 0x37be;
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x37be:
    {
        mcu.pc = 0x37c0;
        uint16_t disp = (uint16_t)(int8_t)0x2c;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37c0:
    {
        mcu.pc = 0x37c2;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(0) & 0xffu));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x37c2:
    {
        mcu.pc = 0x37c4;
        uint16_t disp = (uint16_t)(int8_t)0x28;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37c4:
    {
        mcu.pc = 0x37c6;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x37c6:
    {
        mcu.pc = 0x37c8;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x37c8:
    {
        mcu.pc = 0x37ca;
        uint16_t disp = (uint16_t)(int8_t)0x22;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37ca:
    {
        mcu.pc = 0x37cc;
        uint8_t data = 0x7f;
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;

    case 0x37cc:
    {
        mcu.pc = 0x37ce;
        uint16_t disp = (uint16_t)(int8_t)0x1e;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37ce:
    {
        mcu.pc = 0x37d0;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[3] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[3] = (uint16_t)((mcu.r[3] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;

    case 0x37d0:
    {
        mcu.pc = 0x37d2;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x37d2:
    {
        mcu.pc = 0x37d4;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x37d4:
    {
        mcu.pc = 0x37d6;
        uint16_t disp = (uint16_t)(int8_t)0x0c;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37d6:
    {
        mcu.pc = 0x37d8;
        uint8_t data = 0x7f;
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= data;
        MCU_SetStatusCommon(data, 0);
    }
    break;

    case 0x37d8:
    {
        mcu.pc = 0x37da;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37da:
    {
        mcu.pc = 0x37dc;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[3] &= ~0xff;
        mcu.r[3] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x37dc:
    {
        mcu.pc = 0x37de;
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[3] & 0xff);
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x37de:
    {
        mcu.pc = 0x37e0;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37e0:
    {
        mcu.pc = 0x37e2;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(0) & 0xffu));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x37e2:
    {
        mcu.pc = 0x37e4;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;

    case 0x37e4:
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
    break;

    case 0x37e8:
    {
        mcu.pc = 0x37ea;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;

    case 0x37ea:
    {
        mcu.pc = 0x37ec;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37ec:
    {
        mcu.pc = 0x37ee;
        uint8_t op2 = 0x22;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[2] = (uint16_t)t1;
    }
    break;

    case 0x37ee:
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
    break;

    case 0x37f2:
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
    break;

    case 0x37f5:
    {
        mcu.pc = 0x37f7;
        uint8_t op2 = 0x81;
        uint32_t data = (uint32_t)mcu.r[0];
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x37f7:
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
    break;

    case 0x37fb:
    {
        mcu.pc = 0x37fe;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0xb2;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x37fe:
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
    break;

    case 0x3801:
    {
        mcu.pc = 0x3804;
        uint16_t disp = (uint16_t)(0x00 << 8);
        disp |= 0xa6;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3804:
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
    break;

    case 0x3807:
    {
        mcu.pc = 0x3809;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;

    case 0x3809:
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
    break;

    case 0x380d:
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
    break;

    case 0x3811:
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
    break;

    case 0x3815:
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
    break;

    case 0x3819:
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
    break;

    case 0x381e:
    {
        mcu.pc = 0x3820;
        uint16_t disp = (uint16_t)(int8_t)0x4f;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3820:
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
    break;

    case 0x3824:
    {
        mcu.pc = 0x3826;
        uint16_t disp = (uint16_t)(int8_t)0x49;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3826:
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
    break;

    case 0x382a:
    {
        mcu.pc = 0x382c;
        uint16_t disp = (uint16_t)(int8_t)0x43;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x382c:
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
    break;

    case 0x3830:
    {
        mcu.pc = 0x3832;
        uint16_t disp = (uint16_t)(int8_t)0x3d;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3832:
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
    break;

    case 0x3835:
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
    break;

    case 0x3838:
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
    break;

    case 0x383b:
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
    break;

    case 0x383e:
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
    break;

    case 0x3841:
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
    break;

    case 0x3844:
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
    break;

    case 0x3847:
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
    break;

    case 0x384a:
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
    break;

    case 0x384d:
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
    break;

    case 0x3850:
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
    break;

    case 0x3853:
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
    break;

    case 0x3856:
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
    break;

    case 0x3859:
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
    break;

    case 0x385c:
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
    break;

    case 0x385f:
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
    break;

    case 0x3862:
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
    break;

    case 0x3865:
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
    break;

    case 0x3868:
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
    break;

    case 0x386b:
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
    break;

    case 0x386e:
    {
        mcu.pc = MCU_PopStack();
    }
    break;

    case 0x386f:
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
    break;

    case 0x3872:
    {
        mcu.pc = 0x3874;
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3874:
    {
        mcu.pc = 0x3876;
        uint8_t op2 = 0x21;
        int32_t t1 = (int32_t)mcu.r[1];
        uint32_t t2 = (uint32_t)mcu.r[1];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[1] = (uint16_t)t1;
    }
    break;

    case 0x3876:
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
    break;

    case 0x387a:
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
    break;

    case 0x387d:
    {
        mcu.pc = 0x3880;
        uint16_t data = (uint16_t)(0x00 << 8);
        data |= 0x1b;
        mcu.r[1] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3880:
    {
        mcu.pc = 0x3882;
        uint8_t op2 = 0x76;
        int32_t t1 = (int32_t)mcu.r[6];
        uint32_t t2 = (uint32_t)mcu.r[1];
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;

    case 0x3882:
    {
        mcu.pc = 0x3884;
        uint16_t disp = (uint16_t)(int8_t)0x0e;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3884:
    {
        mcu.pc = 0x3886;
        uint8_t op2 = 0x83;
        uint32_t data = (uint32_t)mcu.r[1];
        mcu.r[3] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3886:
    {
        mcu.pc = 0x3888;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;

    case 0x3888:
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
    break;

    case 0x388c:
    {
        mcu.pc = 0x388e;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x388e:
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
    break;

    case 0x3892:
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
    break;

    case 0x3895:
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
    break;

    case 0x3898:
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
    break;

    case 0x389c:
    {
        mcu.pc = 0x389d;
        /* nop */
    }
    break;

    case 0x389d:
    {
        mcu.pc = 0x389e;
        /* nop */
    }
    break;

    case 0x389e:
    {
        mcu.pc = 0x389f;
        /* nop */
    }
    break;

    case 0x389f:
    {
        mcu.pc = 0x38a0;
        /* nop */
    }
    break;

    case 0x38a0:
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
    break;

    case 0x38a4:
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
    break;

    case 0x38a7:
    {
        mcu.pc = 0x38a9;
        uint16_t disp = (uint16_t)(int8_t)0x01;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x38a9:
    {
        mcu.pc = MCU_PopStack();
    }
    break;

    case 0x38aa:
    {
        mcu.pc = 0x38ac;
        uint8_t op2 = 0x81;
        uint32_t data = (uint32_t)mcu.r[0];
        mcu.r[1] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x38ac:
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
    break;

    case 0x38b0:
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
    break;

    case 0x38b3:
    {
        mcu.pc = 0x38b6;
        int32_t t2 = (int32_t)0xff;
        t2 = (t2 << 8) | (int32_t)0xff;
        int32_t t1 = (int32_t)mcu.r[6];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;

    case 0x38b6:
    {
        mcu.pc = 0x38b8;
        uint16_t disp = (uint16_t)(int8_t)0x1e;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x38b8:
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
    break;

    case 0x38bb:
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
    break;

    case 0x38bf:
    {
        mcu.pc = 0x38c1;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[6];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;

    case 0x38c1:
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
    break;

    case 0x38c5:
    {
        mcu.pc = 0x38c7;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[4];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x38c7:
    {
        mcu.pc = 0x38c9;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x38c9:
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
    break;

    case 0x38cc:
    {
        mcu.pc = 0x38cf;
        int32_t t2 = (int32_t)0xff;
        t2 = (t2 << 8) | (int32_t)0xff;
        int32_t t1 = (int32_t)mcu.r[5];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;

    case 0x38cf:
    {
        mcu.pc = 0x38d1;
        uint16_t disp = (uint16_t)(int8_t)0x6f;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x38d1:
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
    break;

    case 0x38d6:
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
    break;

    case 0x38d9:
    {
        mcu.pc = 0x38dc;
        int32_t t2 = (int32_t)0xff;
        t2 = (t2 << 8) | (int32_t)0xff;
        int32_t t1 = (int32_t)mcu.r[6];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;

    case 0x38dc:
    {
        mcu.pc = 0x38de;
        uint16_t disp = (uint16_t)(int8_t)0x50;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x38de:
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
    break;

    case 0x38e1:
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
    break;

    case 0x38e5:
    {
        mcu.pc = 0x38e7;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[6];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;

    case 0x38e7:
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
    break;

    case 0x38eb:
    {
        mcu.pc = 0x38ed;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[4];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x38ed:
    {
        mcu.pc = 0x38ef;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x38ef:
    {
        mcu.pc = 0x38f2;
        uint16_t data = (uint16_t)(0xff << 8);
        data |= 0xff;
        mcu.r[5] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x38f2:
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
    break;

    case 0x38f5:
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
    break;

    case 0x38f8:
    {
        mcu.pc = 0x38fa;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x38fa:
    {
        mcu.pc = 0x38fc;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;

    case 0x38fc:
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
    break;

    case 0x38fe:
    {
        mcu.pc = 0x3900;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;

    case 0x3900:
    {
        mcu.pc = 0x3902;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3902:
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
    break;

    case 0x3904:
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
    break;

    case 0x3907:
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
    break;

    case 0x390a:
    {
        mcu.pc = 0x390c;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x390c:
    {
        mcu.pc = 0x390e;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;

    case 0x390e:
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
    break;

    case 0x3910:
    {
        mcu.pc = 0x3912;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;

    case 0x3912:
    {
        mcu.pc = 0x3914;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3914:
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
    break;

    case 0x3916:
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
    break;

    case 0x3919:
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
    break;

    case 0x391d:
    {
        mcu.pc = 0x391f;
        uint16_t disp = (uint16_t)(int8_t)0x08;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x391f:
    {
        mcu.pc = 0x3921;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;

    case 0x3921:
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
    break;

    case 0x3923:
    {
        mcu.pc = 0x3925;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[2];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[2] = (uint16_t)(data);
    }
    break;

    case 0x3925:
    {
        mcu.pc = 0x3927;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3927:
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
    break;

    case 0x3929:
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
    break;

    case 0x392c:
    {
        mcu.pc = 0x392e;
        uint16_t disp = (uint16_t)(int8_t)0x12;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x392e:
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
    break;

    case 0x3931:
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
    break;

    case 0x3934:
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
    break;

    case 0x3937:
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
    break;

    case 0x393a:
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
    break;

    case 0x393d:
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
    break;

    case 0x3940:
    {
        mcu.pc = 0x3942;
        uint8_t op2 = 0x13;
        mcu.r[3] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x3942:
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
    break;

    case 0x3945:
    {
        mcu.pc = 0x3947;
        uint8_t op2 = 0x23;
        int32_t t1 = (int32_t)mcu.r[3];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[3] = (uint16_t)t1;
    }
    break;

    case 0x3947:
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
    break;

    case 0x394b:
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
    break;

    case 0x394e:
    {
        mcu.pc = 0x3950;
        uint16_t disp = (uint16_t)(int8_t)0x0b;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t branch = N == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3950:
    {
        mcu.pc = 0x3952;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;

    case 0x3952:
    {
        mcu.pc = 0x3955;
        int32_t t2 = (int32_t)0x28;
        t2 = (t2 << 8) | (int32_t)0xf6;
        int32_t t1 = (int32_t)mcu.r[4];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;

    case 0x3955:
    {
        mcu.pc = 0x3957;
        uint16_t disp = (uint16_t)(int8_t)0x0e;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3957:
    {
        mcu.pc = 0x3959;
        uint8_t op2 = 0x13;
        mcu.r[4] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x3959:
    {
        mcu.pc = 0x395b;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x395b:
    {
        mcu.pc = 0x395d;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[3];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;

    case 0x395d:
    {
        mcu.pc = 0x3960;
        int32_t t2 = (int32_t)0x28;
        t2 = (t2 << 8) | (int32_t)0xf6;
        int32_t t1 = (int32_t)mcu.r[4];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;

    case 0x3960:
    {
        mcu.pc = 0x3962;
        uint16_t disp = (uint16_t)(int8_t)0x03;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3962:
    {
        mcu.pc = 0x3965;
        uint16_t data = (uint16_t)(0x28 << 8);
        data |= 0xf6;
        mcu.r[4] = data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3965:
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
    break;

    case 0x3969:
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
    break;

    case 0x396c:
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
    break;

    case 0x3970:
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
    break;

    case 0x3972:
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
    break;

    case 0x3975:
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
    break;

    case 0x3978:
    {
        mcu.pc = 0x397a;
        uint8_t op2 = 0x86;
        uint32_t data = (uint32_t)mcu.r[5];
        mcu.r[6] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x397a:
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
    break;

    case 0x397e:
    {
        mcu.pc = 0x3980;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3980:
    {
        mcu.pc = 0x3982;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[5];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[5] = (uint16_t)(data);
    }
    break;

    case 0x3982:
    {
        mcu.pc = 0x3984;
        uint8_t op2 = 0x82;
        uint32_t data = (uint32_t)mcu.r[5];
        mcu.r[2] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3984:
    {
        mcu.pc = 0x3986;
        uint8_t op2 = 0x13;
        mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(0) & 0xffu));
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x3986:
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
    break;

    case 0x3988:
    {
        mcu.pc = 0x398a;
        uint8_t op2 = 0x13;
        mcu.r[4] = (uint16_t)(0);
        MCU_SetStatus(0, STATUS_N);
        MCU_SetStatus(1, STATUS_Z);
        MCU_SetStatus(0, STATUS_V);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x398a:
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
    break;

    case 0x398e:
    {
        mcu.pc = 0x3990;
        uint8_t op2 = 0x08;
        uint32_t t1 = (uint32_t)mcu.r[2];
        int32_t t2 = 1;
        t1 = (uint32_t)MCU_ADD_Common((int32_t)t1, t2, 0, 1);
        mcu.r[2] = (uint16_t)(t1);
    }
    break;

    case 0x3990:
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
    break;

    case 0x3994:
    {
        mcu.pc = 0x3996;
        uint8_t op2 = 0x32;
        int32_t t1 = (int32_t)mcu.r[2];
        uint32_t t2 = (uint32_t)(mcu.r[4] & 0xff);
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 0);
        mcu.r[2] &= ~0xff;
        mcu.r[2] |= (uint16_t)(t1 & 0xff);
    }
    break;

    case 0x3996:
    {
        mcu.pc = 0x3998;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t branch = C == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3998:
    {
        mcu.pc = 0x399a;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)(mcu.r[2] & 0xff);
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 0);
        mcu.r[2] = (uint16_t)((mcu.r[2] & 0xff00u) | ((uint32_t)(data) & 0xffu));
    }
    break;

    case 0x399a:
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
    break;

    case 0x399c:
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
    break;

    case 0x399e:
    {
        mcu.pc = 0x39a0;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;

    case 0x39a0:
    {
        mcu.pc = 0x39a2;
        uint16_t disp = (uint16_t)(int8_t)0x06;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x39a2:
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
    break;

    case 0x39a4:
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
    break;

    case 0x39a6:
    {
        mcu.pc = 0x39a8;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[2];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;

    case 0x39a8:
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
    break;

    case 0x39aa:
    {
        mcu.pc = 0x39ad;
        int32_t t2 = (int32_t)0x80;
        t2 = (t2 << 8) | (int32_t)0x00;
        int32_t t1 = (int32_t)mcu.r[6];
        MCU_SUB_Common(t1, t2, 0, 1);
    }
    break;

    case 0x39ad:
    {
        mcu.pc = 0x39af;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t C = (mcu.sr & STATUS_C) != 0;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = (C | Z) == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x39af:
    {
        mcu.pc = 0x39b1;
        uint8_t op2 = 0x14;
        uint32_t data = (uint32_t)mcu.r[4];
        data = (uint32_t)MCU_SUB_Common(0, (int32_t)data, 0, 1);
        mcu.r[4] = (uint16_t)(data);
    }
    break;

    case 0x39b1:
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
    break;

    case 0x39b4:
    {
        mcu.pc = MCU_PopStack();
    }
    break;

    /* 0x7238 selector 5/6 tail 0x3a5a..0x3a9d (entered from the gen-owned
     * 0x3a54/0x3a57 movi r6). Byte-identical to the gen index-4 body
     * 0x3a30..0x3a53 up to 0x3a74; the tail adds the +/-r6 slew clamp. */
    case 0x3a5a:
    {
        mcu.pc = 0x3a5c;
        uint8_t op2 = 0x25;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[5];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[5] = (uint16_t)t1;
    }
    break;

    case 0x3a5c:
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
    break;

    case 0x3a5e:
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
    break;

    case 0x3a61:
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
    break;

    case 0x3a65:
    {
        mcu.pc = 0x3a67;
        uint8_t op2 = 0x16;
        uint32_t data = (uint32_t)mcu.r[4];
        MCU_SetStatusCommon(data, 1);
        MCU_SetStatus(0, STATUS_C);
    }
    break;

    case 0x3a67:
    {
        mcu.pc = 0x3a69;
        uint16_t disp = (uint16_t)(int8_t)0x0b;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3a69:
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
    break;

    case 0x3a6d:
    {
        mcu.pc = 0x3a6f;
        uint16_t addr = (uint16_t)(mcu.br << 8);
        addr |= 0x34;
        uint32_t data = (uint32_t)MCU_Read(addr);
        mcu.r[4] &= ~0xff;
        mcu.r[4] |= (uint16_t)data;
        MCU_SetStatusCommon(data, 0);
    }
    break;

    case 0x3a6f:
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
    break;

    case 0x3a71:
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
    break;

    case 0x3a74:
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
    break;

    case 0x3a77:
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
    break;

    case 0x3a7a:
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
    break;

    case 0x3a7d:
    {
        mcu.pc = 0x3a7f;
        uint8_t op2 = 0x75;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[4];
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;

    case 0x3a7f:
    {
        mcu.pc = 0x3a81;
        uint16_t disp = (uint16_t)(int8_t)0x16;
        uint32_t Z = (mcu.sr & STATUS_Z) != 0;
        uint32_t branch = Z == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3a81:
    {
        mcu.pc = 0x3a83;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t V = (mcu.sr & STATUS_V) != 0;
        uint32_t branch = (N ^ V) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3a83:
    {
        mcu.pc = 0x3a85;
        uint8_t op2 = 0x34;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[6];
        t1 = MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;

    case 0x3a85:
    {
        mcu.pc = 0x3a87;
        uint16_t disp = (uint16_t)(int8_t)0x0e;
        uint32_t V = (mcu.sr & STATUS_V) != 0;
        uint32_t branch = V == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3a87:
    {
        mcu.pc = 0x3a89;
        uint8_t op2 = 0x75;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[4];
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;

    case 0x3a89:
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
    break;

    case 0x3a8b:
    {
        mcu.pc = 0x3a8d;
        uint16_t disp = (uint16_t)(int8_t)0x0a;
        uint32_t branch = 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3a8d:
    {
        mcu.pc = 0x3a8f;
        uint8_t op2 = 0x24;
        int32_t t1 = (int32_t)mcu.r[4];
        uint32_t t2 = (uint32_t)mcu.r[6];
        t1 = MCU_ADD_Common(t1, (int32_t)t2, 0, 1);
        mcu.r[4] = (uint16_t)t1;
    }
    break;

    case 0x3a8f:
    {
        mcu.pc = 0x3a91;
        uint16_t disp = (uint16_t)(int8_t)0x04;
        uint32_t V = (mcu.sr & STATUS_V) != 0;
        uint32_t branch = V == 1;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3a91:
    {
        mcu.pc = 0x3a93;
        uint8_t op2 = 0x75;
        int32_t t1 = (int32_t)mcu.r[5];
        uint32_t t2 = (uint32_t)mcu.r[4];
        MCU_SUB_Common(t1, (int32_t)t2, 0, 1);
    }
    break;

    case 0x3a93:
    {
        mcu.pc = 0x3a95;
        uint16_t disp = (uint16_t)(int8_t)0x02;
        uint32_t N = (mcu.sr & STATUS_N) != 0;
        uint32_t V = (mcu.sr & STATUS_V) != 0;
        uint32_t branch = (N ^ V) == 0;
        if (branch)
        mcu.pc += disp;
    }
    break;

    case 0x3a95:
    {
        mcu.pc = 0x3a97;
        uint8_t op2 = 0x84;
        uint32_t data = (uint32_t)mcu.r[5];
        mcu.r[4] = (uint16_t)data;
        MCU_SetStatusCommon(data, 1);
    }
    break;

    case 0x3a97:
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
    break;

    case 0x3a9a:
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
    break;

    case 0x3a9d:
    {
        mcu.pc = MCU_PopStack();
    }
    break;

    default: /* not in this block: execute the stock instruction once */
    {
        uint8_t op = MCU_ReadCodeAdvance();
        MCU_Operand_Table[op](op);
        break;
    }
    }
    return 1;
}

/* One entry per instruction PC of the closure (0x3709..0x39b4 plus the
 * 0x7238 selector-5/6 tail 0x3a5a..0x3a9d). */
const uint16_t kInterpPcs[] = {
    0x3709, 0x370c, 0x370e, 0x3711, 0x3713, 0x3717, 0x371b, 0x371f, 0x3723, 0x3728,
    0x372a, 0x372e, 0x3730, 0x3734, 0x3736, 0x373a, 0x373c, 0x373f, 0x3742, 0x3744,
    0x3746, 0x374a, 0x374d, 0x3750, 0x3752, 0x3754, 0x3756, 0x3758, 0x375c, 0x375e,
    0x3762, 0x3765, 0x3768, 0x376c, 0x376d, 0x376e, 0x376f, 0x3770, 0x3774, 0x3777,
    0x3779, 0x377a, 0x377d, 0x3780, 0x3783, 0x3786, 0x3788, 0x378a, 0x378c, 0x378e,
    0x3790, 0x3792, 0x3794, 0x3796, 0x3799, 0x379b, 0x379e, 0x37a1, 0x37a5, 0x37a7,
    0x37a9, 0x37ac, 0x37af, 0x37b1, 0x37b3, 0x37b6, 0x37b8, 0x37ba, 0x37bc, 0x37be,
    0x37c0, 0x37c2, 0x37c4, 0x37c6, 0x37c8, 0x37ca, 0x37cc, 0x37ce, 0x37d0, 0x37d2,
    0x37d4, 0x37d6, 0x37d8, 0x37da, 0x37dc, 0x37de, 0x37e0, 0x37e2, 0x37e4, 0x37e8,
    0x37ea, 0x37ec, 0x37ee, 0x37f2, 0x37f5, 0x37f7, 0x37fb, 0x37fe, 0x3801, 0x3804,
    0x3807, 0x3809, 0x380d, 0x3811, 0x3815, 0x3819, 0x381e, 0x3820, 0x3824, 0x3826,
    0x382a, 0x382c, 0x3830, 0x3832, 0x3835, 0x3838, 0x383b, 0x383e, 0x3841, 0x3844,
    0x3847, 0x384a, 0x384d, 0x3850, 0x3853, 0x3856, 0x3859, 0x385c, 0x385f, 0x3862,
    0x3865, 0x3868, 0x386b, 0x386e, 0x386f, 0x3872, 0x3874, 0x3876, 0x387a, 0x387d,
    0x3880, 0x3882, 0x3884, 0x3886, 0x3888, 0x388c, 0x388e, 0x3892, 0x3895, 0x3898,
    0x389c, 0x389d, 0x389e, 0x389f, 0x38a0, 0x38a4, 0x38a7, 0x38a9, 0x38aa, 0x38ac,
    0x38b0, 0x38b3, 0x38b6, 0x38b8, 0x38bb, 0x38bf, 0x38c1, 0x38c5, 0x38c7, 0x38c9,
    0x38cc, 0x38cf, 0x38d1, 0x38d6, 0x38d9, 0x38dc, 0x38de, 0x38e1, 0x38e5, 0x38e7,
    0x38eb, 0x38ed, 0x38ef, 0x38f2, 0x38f5, 0x38f8, 0x38fa, 0x38fc, 0x38fe, 0x3900,
    0x3902, 0x3904, 0x3907, 0x390a, 0x390c, 0x390e, 0x3910, 0x3912, 0x3914, 0x3916,
    0x3919, 0x391d, 0x391f, 0x3921, 0x3923, 0x3925, 0x3927, 0x3929, 0x392c, 0x392e,
    0x3931, 0x3934, 0x3937, 0x393a, 0x393d, 0x3940, 0x3942, 0x3945, 0x3947, 0x394b,
    0x394e, 0x3950, 0x3952, 0x3955, 0x3957, 0x3959, 0x395b, 0x395d, 0x3960, 0x3962,
    0x3965, 0x3969, 0x396c, 0x3970, 0x3972, 0x3975, 0x3978, 0x397a, 0x397e, 0x3980,
    0x3982, 0x3984, 0x3986, 0x3988, 0x398a, 0x398e, 0x3990, 0x3994, 0x3996, 0x3998,
    0x399a, 0x399c, 0x399e, 0x39a0, 0x39a2, 0x39a4, 0x39a6, 0x39a8, 0x39aa, 0x39ad,
    0x39af, 0x39b1, 0x39b4, 0x3a5a, 0x3a5c, 0x3a5e, 0x3a61, 0x3a65, 0x3a67,
    0x3a69, 0x3a6d, 0x3a6f, 0x3a71, 0x3a74, 0x3a77, 0x3a7a, 0x3a7d, 0x3a7f,
    0x3a81, 0x3a83, 0x3a85, 0x3a87, 0x3a89, 0x3a8b, 0x3a8d, 0x3a8f, 0x3a91,
    0x3a93, 0x3a95, 0x3a97, 0x3a9a, 0x3a9d,
};

} /* anonymous namespace */

/* Hand module fill: called by mk2c::hand_fill_modules() from
 * MK2CPP_HandFillTables after the built-in modules. */
void pcm_interp_fill(void)
{
    for (uint32_t i = 0; i < sizeof(kInterpPcs) / sizeof(kInterpPcs[0]); i++)
        MK2CPP_HandRegisterRoutine(kInterpPcs[i], &step_interp);
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
