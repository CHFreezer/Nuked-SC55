#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(int argc, char **argv) {
    FILE *f = fopen(argv[1], "r");
    char line[32], prev[32] = {0};
    int start = 0;
    int target = atoi(argv[2]); // which 7c3f->33f0 occurrence (1-based)
    int cnt = 0;
    while (fgets(line, 32, f)) {
        line[strcspn(line, "\r\n")] = 0;
        if (start) {
            printf("%s\n", line);
            if (strcmp(line, "00007c3f") == 0) break;
        }
        if (!start && strcmp(line, "000433f0") == 0 && strcmp(prev, "00007c3f") == 0) {
            cnt++;
            if (cnt == target) { start = 1; printf("%s\n", line); continue; }
        }
        strcpy(prev, line);
    }
    return 0;
}
