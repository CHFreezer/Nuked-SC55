#include <stdio.h>
#include <stdlib.h>
int main(void) {
    FILE *f = fopen("..\\roms\\SC-55mk2-v1.01\\rom2.bin", "rb");
    unsigned char buf[512 * 1024];
    long n = fread(buf, 1, sizeof buf, f);
    fclose(f);
    for (int a = 0x00ff80; a < 0x010020; a += 16) {
        printf("%06x: ", a);
        for (int k = 0; k < 16; k++) printf("%02x ", buf[a + k]);
        printf(" |");
        for (int k = 0; k < 16; k++) { char c = buf[a+k]; if (c < 32 || c > 126) c = '.'; putchar(c); }
        printf("|\n");
    }
    return 0;
}
