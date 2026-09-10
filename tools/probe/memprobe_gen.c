// memprobe_gen.c — build the H8/532 memory-probe ROM and a 6502 SM idle ROM,
// both as files under build/. Run the H8 probe through tools/vm/h8vm.exe and
// decode the snapshot with memprobe_read.c.
//
// Purpose: empirically establish the simulated main-MCU address space — which
// (page, offset) regions are readable/writable — and (after the 方案A patch)
// prove page 6 is a real, full-window, non-aliasing writable RAM.
//
// Memory model probed (from h8vm_main.c, T0):
//   page = (addr>>16)&0xf  (4-bit) -> 16 pages x 64KB = 1MB addressable.
//   page0 off>=0x8000 -> sram[&0x7fff]; page0 0xfb80-0xff80 (RAME) -> ram[&0x3ff];
//   page6 -> b_ram[off] (64KB extended RAM, 方案A patch; previously unbacked);
//   page10/11 -> sram mirror; rom1/rom2 read-only (writes dropped);
//   pages 5/7/12/13 -> unbacked (write dropped, read 0xff).
//
// MOVF addressing is (tp<<16) | (r6+disp), disp = int8. Reset clears r6=0.
//
// INSTRUCTION SEMANTICS (byte-for-byte verified against h8vm_body.c AND a
// g_verbose read log of an actual run — protocol §3/§4):
//   0x88/0x90 are NOT byte ops: bit3 (0x08) = siz. 0x88 = WORD read
//   (r0 = Read16: b[off]<<8|b[off+1]); 0x90 = word-dir write = Write16
//   (hi byte -> off, lo byte -> off+1). Byte-precise ops are:
//     0x80 = byte read  (r0 = Read(off), full register)
//     0x98 = single-byte write (Write(off, r0 & 0xff))
//   The main 16-page matrix below uses the legacy 0x88/0x90 pattern (its
//   WRITABLE verdict = readback low byte 0xA5, i.e. the Write16 lo byte at
//   off+1; classification WRITABLE/UNBACKED/ROM is unaffected). The page6
//   detail block uses the byte-precise 0x80/0x98 ops so every checked byte
//   is exactly the probed address.
//
// Other encodings (verified):
//   LDC <ctrl>,#imm8  = 04 imm (0x11<<3|ctrl)   ctrl: br=3 dp=5 tp=7
//   MOVI rN,#imm8     = 50|N imm ; MOVI rN,#imm16 = 58|N hi lo
//   MOVS [br+d8],rN   = 70|N d (byte store, addr=(br<<8)|d)
//   SLEEP             = 1A
//
// Results land in sram via br=0x81 (page0 0x8100+ = sram[0x0100+]):
//   main matrix (per page P, off O in {0x0000,0x8000}):
//     pre0[P]@sram[0x0100+P]  rb0[P]@sram[0x0110+P]   (P, 0x0000)
//     pre1[P]@sram[0x0140+P]  rb1[P]@sram[0x0150+P]   (P, 0x8000)
//   page6 detail (byte-precise 0x80/0x98, expected values in comments):
//     d0@sram[0x0160] pre (6,0x0000)      informational: the matrix's legacy
//                                         0x90 Write16 leaks the r0 high byte
//                                         (word read = r &= ~0xff; r |= data,
//                                         mcu_opcodes.cpp:724) -> b_ram[0]=0xff
//                                         from the page5 pre-read; not a p6 prop
//     d1@sram[0x0161] rb  (6,0x0000) wr A5 expect A5
//     d2@sram[0x0162] rb  (6,0x8000) wr 5A expect 5A
//     d3@sram[0x0163] re-read (6,0x0000)  expect A5   (no alias w/ 0x8000)
//     d4@sram[0x0164] pre (6,0xC000)      expect 00
//     d5@sram[0x0165] rb  (6,0xC000) wr 3C expect 3C
//     d6@sram[0x0166] pre (6,0xFFFE)      expect 00
//     d7@sram[0x0167] rb  (6,0xFFFE) wr C3 expect C3
//
// 6502 SM idle ROM: reset vector lives at sm_rom[0xffe..0xfff]. SM_GetVectorAddress
// (SM_VECTOR_RESET=9) reads SM addr 0x1ffe/0x1fff, and SM addr A maps to sm_rom[A & 0xfff]
// for A in [0x1000,0x1fff]. So 0x1ffe -> sm_rom[0xffe] (low), 0x1fff -> sm_rom[0xfff] (high).
// Point it at sm_rom[0] (=SM addr 0x1000) holding STP(0x42) so the sub-CPU sleeps on
// reset. Rest = NOP(0xea).

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

static uint8_t rom[0x8000];
static int p = 0;

static int emit(uint8_t b) { rom[p++] = b; return p - 1; }
static int emit2(uint8_t a, uint8_t b) { emit(a); return emit(b); }
static int emit3(uint8_t a, uint8_t b, uint8_t c) { emit2(a, b); return emit(c); }

static FILE *gmap = NULL;
static void mapnote(const uint8_t *bytes, int n, const char *mnem)
{
    if (!gmap) return;
    fprintf(gmap, "  %04x: ", p - n);
    for (int i = 0; i < n; i++) fprintf(gmap, "%02x ", bytes[i]);
    fprintf(gmap, "  %s\n", mnem);
}

// legacy matrix probe of one (tp, r6-based) offset using the 0x88/0x90 pattern
static void probe(int disp, int pre_off, int rb_off, const char *tag)
{
    uint8_t b[8]; int at; char m[64];
    at = p; emit2(0x88, (uint8_t)disp); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVFW r0,[tp+%02x] ; %s pre-read", disp, tag); mapnote(b, 2, m);
    at = p; emit2(0x70, (uint8_t)pre_off); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVS [br+%02x],r0 ; %s pre", pre_off, tag); mapnote(b, 2, m);
    at = p; emit2(0x50, 0xA5); memcpy(b, rom + at, 2); mapnote(b, 2, "MOVE r0,#0xA5");
    at = p; emit2(0x90, (uint8_t)disp); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVFW [tp+%02x],r0 ; %s Write16 00A5", disp, tag); mapnote(b, 2, m);
    at = p; emit2(0x88, (uint8_t)disp); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVFW r0,[tp+%02x] ; %s readback", disp, tag); mapnote(b, 2, m);
    at = p; emit2(0x70, (uint8_t)rb_off); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVS [br+%02x],r0 ; %s rb", rb_off, tag); mapnote(b, 2, m);
}

// byte-precise page6 helpers
static void p6_read(uint16_t off, int scratch, const char *tag)
{
    uint8_t b[8]; int at; char m[96];
    at = p; emit3(0x5E, (uint8_t)(off >> 8), (uint8_t)(off & 0xff)); memcpy(b, rom + at, 3);
    snprintf(m, sizeof m, "MOVI r6,#%04x ; target (6,%04x)", off, off); mapnote(b, 3, m);
    at = p; emit2(0x80, 0x00); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVFB r0,[tp+00] ; %s", tag); mapnote(b, 2, m);
    at = p; emit2(0x70, (uint8_t)scratch); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVS [br+%02x],r0 ; d%d @sram[0x01%02x] %s", scratch, scratch - 0x60, scratch, tag); mapnote(b, 2, m);
}
static void p6_write(uint16_t off, uint8_t val, const char *tag)
{
    uint8_t b[8]; int at; char m[96];
    at = p; emit3(0x5E, (uint8_t)(off >> 8), (uint8_t)(off & 0xff)); memcpy(b, rom + at, 3);
    snprintf(m, sizeof m, "MOVI r6,#%04x ; target (6,%04x)", off, off); mapnote(b, 3, m);
    at = p; emit2(0x50, val); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVE r0,#%02x", val); mapnote(b, 2, m);
    at = p; emit2(0x98, 0x00); memcpy(b, rom + at, 2);
    snprintf(m, sizeof m, "MOVFW [tp+00],r0 ; %s wr %02x -> (6,%04x)", tag, val, off); mapnote(b, 2, m);
}

static int gen_h8_probe(const char *out_rom1, const char *out_map)
{
    memset(rom, 0xff, sizeof rom);
    rom[0] = 0x00; rom[1] = 0x00; rom[2] = 0x00; rom[3] = 0x10; // reset vector -> cp=00 pc=0010
    p = 0x10;

    gmap = fopen(out_map, "w");
    if (!gmap) return 1;
    fprintf(gmap, "H8/532 memory probe ROM  (br=0x81 scratch -> sram[0x0100+]; r6=0 at reset)\n");
    fprintf(gmap, "  main matrix: per page P, off in {0x0000,0x8000}: pre=pre-read, rb=readback after Write16 00A5\n");
    fprintf(gmap, "    WRITABLE iff rb==0xa5 ; UNBACKED iff pre==0xff && rb==0xff ; else READ-ONLY ROM\n");
    fprintf(gmap, "    (0x88/0x90 are word ops: pre/rb record the low byte = value at off+1)\n");
    fprintf(gmap, "  page6 detail: byte-precise (0x80/0x98) at (6, 0x0000/0x8000/0xC000/0xFFFE) @ sram[0x0160..0x0167]\n");

    uint8_t b[8]; int at; char m[96];

    // setup: br = 0x81 -> scratch at page0 0x8100+ = sram[0x0100+]. br must be >= 0x80
    // so the offset (br<<8) lands in sram (page0 0x8000-0xffff); br=0x80 would alias the
    // probe targets sram[0x0000]/sram[0x8000], so use 0x81 -> sram[0x0100+] (no alias).
    at = p; emit3(0x04, 0x81, 0x8B); memcpy(b, rom + at, 3); mapnote(b, 3, "LDC br,#0x81   ; scratch base -> sram[0x0100+]");

    // offset 0: r6 stays 0 -> (page, 0x0000)
    for (int P = 0; P < 16; P++)
    {
        at = p; emit3(0x04, (uint8_t)P, 0x8F); memcpy(b, rom + at, 3);
        snprintf(m, sizeof m, "LDC tp,#%02x   ; page %d", P, P); mapnote(b, 3, m);
        probe(0x00, 0x00 + P, 0x10 + P, "off0");
    }
    // offset 0x8000: set r6=0x8000 (MOVI r6,#0x8000 = 5E 80 00) -> (page, 0x8000)
    at = p; emit3(0x5E, 0x80, 0x00); memcpy(b, rom + at, 3); mapnote(b, 3, "MOVI r6,#0x8000 ; probe base off 0x8000");
    for (int P = 0; P < 16; P++)
    {
        at = p; emit3(0x04, (uint8_t)P, 0x8F); memcpy(b, rom + at, 3);
        snprintf(m, sizeof m, "LDC tp,#%02x   ; page %d", P, P); mapnote(b, 3, m);
        probe(0x00, 0x40 + P, 0x50 + P, "off1");
    }

    // == page6 detail (byte-precise): prove full 64KB window, no aliasing ==
    at = p; emit3(0x04, 0x06, 0x8F); memcpy(b, rom + at, 3); mapnote(b, 3, "LDC tp,#06   ; page6 detail");
    p6_read (0x0000, 0x60, "d0 pre (6,0x0000) expect 00");
    p6_write(0x0000, 0xA5, "d1 target");
    p6_read (0x0000, 0x61, "d1 rb (6,0x0000) expect A5");
    p6_write(0x8000, 0x5A, "d2 target");
    p6_read (0x8000, 0x62, "d2 rb (6,0x8000) expect 5A");
    p6_read (0x0000, 0x63, "d3 re-read (6,0x0000) expect A5 (no alias)");
    p6_read (0xC000, 0x64, "d4 pre (6,0xC000) expect 00");
    p6_write(0xC000, 0x3C, "d5 target");
    p6_read (0xC000, 0x65, "d5 rb (6,0xC000) expect 3C");
    p6_read (0xFFFE, 0x66, "d6 pre (6,0xFFFE) expect 00");
    p6_write(0xFFFE, 0xC3, "d7 target");
    p6_read (0xFFFE, 0x67, "d7 rb (6,0xFFFE) expect C3");

    // halt
    at = p; emit(0x1A); memcpy(b, rom + at, 1); mapnote(b, 1, "SLEEP   ; halt");
    fclose(gmap);

    FILE *f = fopen(out_rom1, "wb");
    if (!f) return 2;
    fwrite(rom, 1, sizeof rom, f);
    fclose(f);
    printf("wrote %s (%d code bytes, %dKB image)\n", out_rom1, p, (int)(sizeof rom / 1024));
    return 0;
}

static int gen_sm_idle(const char *out_smrom)
{
    static uint8_t sm[4096];
    for (int i = 0; i < 4096; i++) sm[i] = 0xea; // NOP
    sm[0x0000] = 0x42; // STP: sleep on first fetch at SM addr 0x1000
    sm[0x0ffe] = 0x00; // reset vector low  (SM addr 0x1ffe) -> 0x1000
    sm[0x0fff] = 0x10; // reset vector high
    FILE *f = fopen(out_smrom, "wb");
    if (!f) return 2;
    fwrite(sm, 1, sizeof sm, f);
    fclose(f);
    printf("wrote %s (4KB idle: sm[0]=STP, reset vec @sm[0xffe]=0x1000)\n", out_smrom);
    return 0;
}

int main(int argc, char **argv)
{
    const char *out_rom1 = (argc > 1) ? argv[1] : "build/probe_rom1.bin";
    const char *out_map  = (argc > 2) ? argv[2] : "build/probe_map.txt";
    const char *out_sm   = (argc > 3) ? argv[3] : "build/probe_smrom.bin";
    int rc = gen_h8_probe(out_rom1, out_map);
    if (rc) return rc;
    return gen_sm_idle(out_sm);
}
