// memprobe_read.c — decode the H8 probe results out of a VM snapshot.
// Snapshot layout (VM_SaveState in h8vm_main.c): [mcu_t][ram 0x400][sram 0x8000]...
// sram therefore starts at file offset 0x450 (= sizeof(mcu_t) 0x50 + ram 0x400).
//
// The H8 probe (memprobe_gen.c) sets br=0x81 and probes each page P at offsets
// 0x0000 and 0x8000, recording into the scratch at sram[0x0100+]:
//   pre0[P]@sram[0x0100+P]  rb0[P]@sram[0x0110+P]   (page P, 0x0000)
//   pre1[P]@sram[0x0140+P]  rb1[P]@sram[0x0150+P]   (page P, 0x8000)
// where pre = read before any write, rb = readback after writing 0xA5.
// (0x88/0x90 are WORD ops in this VM/GT: pre/rb record the low byte = value at off+1.)
//   WRITABLE  iff rb == 0xA5   (the 0xA5 write stuck)
//   UNBACKED  iff pre == 0xff && rb == 0xff   (always 0xff, write dropped)
//   READ-ONLY otherwise (write dropped; rb == pre, a ROM byte)
//
// Page6 detail block (byte-precise 0x80/0x98 ops), sram[0x0160..0x0167]:
//   d0 pre(6,0x0000)     informational (matrix's legacy Write16 leaks the r0
//                        high byte: word read does r &= ~0xff; r |= data
//                        (mcu_opcodes.cpp:724), so b_ram[0] carries 0xFF from
//                        the page5 unbacked pre-read; NOT a page6 property)
//   d1 rb (6,0x0000=A5)  expect A5
//   d2 rb (6,0x8000=5A)  expect 5A
//   d3 re-read (6,0x0000) expect A5   (no alias with 0x8000)
//   d4 pre(6,0xC000)     expect 00
//   d5 rb (6,0xC000=3C)  expect 3C
//   d6 pre(6,0xFFFE)     expect 00
//   d7 rb (6,0xFFFE=C3)  expect C3
// d1..d7 all match => page6 is a real 64KB writable window, byte-precise,
// no aliasing (5 distinct offsets 0x0000/0x8000/0xC000/0xFFFE, distinct
// sentinels, all retained independently).

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

static const char *verdict(uint8_t pre, uint8_t rb)
{
    if (rb == 0xa5) return "WRITABLE (RAM)";
    if (pre == 0xff && rb == 0xff) return "UNBACKED (rd 0xff, write dropped)";
    return "READ-ONLY (ROM, write dropped)";
}

int main(int argc, char **argv)
{
    const char *path = (argc > 1) ? argv[1] : "build/probe.snap";
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", path); return 1; }
    long n = 0;
    fseek(f, 0, SEEK_END); n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *buf = malloc(n);
    if (!buf || fread(buf, 1, n, f) != (size_t)n) { fprintf(stderr, "read failed\n"); return 1; }
    fclose(f);
    if (n < 0x450 + 0x0160) { fprintf(stderr, "snapshot too small (%ld)\n", n); return 1; }

    uint8_t *sram = buf + 0x450;
    // mcu_t (h8vm.h): r[8]@0x00(16B), pc@0x10, sr@0x12, cp@0x14, dp@0x15, ep@0x16,
    // tp@0x17, br@0x18, sleep@0x19  (little-endian)
    uint16_t mr0 = buf[0x00] | (buf[0x01] << 8);
    uint16_t mr6 = buf[0x0c] | (buf[0x0d] << 8);
    uint16_t mpc = buf[0x10] | (buf[0x11] << 8);
    uint16_t msr = buf[0x12] | (buf[0x13] << 8);
    printf("H8 probe results  (snapshot %s, %ld bytes)\n", path, n);
    printf("  H8 final state: cp=%02x pc=%04x sr=%04x r0=%04x r6=%04x br=%02x dp=%02x ep=%02x tp=%02x sleep=%d\n",
           buf[0x14], mpc, msr, mr0, mr6, buf[0x18], buf[0x15], buf[0x16], buf[0x17], buf[0x19]);
    printf("  Per page P, two offsets: 0x0000 (off0) and 0x8000 (off1).\n");
    printf("  pre = pre-write read, rb = readback after writing 0xA5.\n");
    printf("  WRITABLE iff rb==0xa5 ; UNBACKED iff pre==rb==0xff\n");
    printf("\n  %-3s | %-7s %-7s %-22s | %-7s %-7s %-22s\n",
           "P", "off0 pre", "off0 rb", "off0 verdict", "off1 pre", "off1 rb", "off1 verdict");
    printf("  %s\n", "--------------------------------------------------------------"
                      "--------------------------------------------------------");
    int wr_pages = 0, un_pages = 0, rom_pages = 0;
    for (int P = 0; P < 16; P++)
    {
        uint8_t pre0 = sram[0x0100 + P], rb0 = sram[0x0110 + P];
        uint8_t pre1 = sram[0x0140 + P], rb1 = sram[0x0150 + P];
        const char *v0 = verdict(pre0, rb0), *v1 = verdict(pre1, rb1);
        // page-level class: writable if either offset sticky; unbacked if both unbacked;
        // else ROM (a page is uniformly one backing).
        int w0 = rb0 == 0xa5, w1 = rb1 == 0xa5;
        int u0 = pre0 == 0xff && rb0 == 0xff, u1 = pre1 == 0xff && rb1 == 0xff;
        if (w0 || w1) wr_pages++;
        else if (u0 && u1) un_pages++;
        else rom_pages++;
        printf("  %02x    |  %02x     %02x     %-22s |  %02x     %02x     %-22s\n",
               P, pre0, rb0, v0, pre1, rb1, v1);
    }
    printf("\n  summary: %d/16 pages have a sticky-writable offset, %d/16 fully unbacked, "
           "%d/16 read-only ROM\n", wr_pages, un_pages, rom_pages);

    // ---- page6 detail block (byte-precise) ----
    // sram[0x0160..0x0167] = d0..d7. d0 is informational (see header);
    // d1..d7 carry the deterministic expectations.
    static const uint8_t exp[8] = { 0xFF, 0xA5, 0x5A, 0xA5, 0x00, 0x3C, 0x00, 0xC3 };
    // d0=0xFF: matrix's prior 0x90 Write16 to (6,0x0000) left r0's high byte 0xFF
    // (word-read keeps the old high byte). d4/d6 are the pristine 0x00 checks.
    static const char *dtag[8] = {
        "d0 pre (6,0x0000) [info]", "d1 rb (6,0x0000)=A5", "d2 rb (6,0x8000)=5A",
        "d3 re-read (6,0x0000)", "d4 pre (6,0xC000)", "d5 rb (6,0xC000)=3C",
        "d6 pre (6,0xFFFE)", "d7 rb (6,0xFFFE)=C3" };
    printf("\n  page6 detail (byte-precise 0x80/0x98; 方案A extended RAM):\n");
    int p6ok = 1;
    for (int i = 0; i < 8; i++)
    {
        uint8_t v = sram[0x0160 + i];
        if (i == 0)
        {
            printf("    %s : %02x (matrix Write16 high-byte leak; not a page6 property)\n", dtag[i], v);
            continue;
        }
        int ok = (v == exp[i]);
        if (!ok) p6ok = 0;
        printf("    %s : %02x (expect %02x) %s\n", dtag[i], v, exp[i], ok ? "OK" : "FAIL");
    }
    printf("    page6 64KB window: %s\n",
           p6ok ? "WRITABLE, full window, no aliasing (PASS)"
                : "NOT as expected (FAIL)");
    free(buf);
    return p6ok ? 0 : 3;
}
