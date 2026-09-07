// First structural divergence between two segment dumps.
// Usage: segdiff <orig_seg> <vm_seg>
#include <stdio.h>
#include <string.h>
int main(int argc, char **argv){
    if (argc < 3) { fprintf(stderr, "usage: segdiff <orig_seg> <vm_seg>\n"); return 1; }
    FILE *f1=fopen(argv[1],"r"), *f2=fopen(argv[2],"r");
    if (!f1 || !f2) return 2;
    char a[32], b[32]; long i=0, last=0;
    char pa[32]={0}, pb[32]={0};
    while(fgets(a,32,f1)&&fgets(b,32,f2)){
        i++;
        a[strcspn(a,"\r\n")]=0; b[strcspn(b,"\r\n")]=0;
        /* collapse consecutive identical lines */
        if (i>1 && strcmp(a,pa)==0 && strcmp(b,pb)==0) continue;
        if (strcmp(a,b)!=0){
            printf("structural divergence at file pair line ~%ld: orig=%s vm=%s\n", i, a, b);
            for (int k = 0; k < 30; k++) {
                char x[32], y[32];
                if (!fgets(x, 32, f1) || !fgets(y, 32, f2)) break;
                x[strcspn(x,"\r\n")]=0; y[strcspn(y,"\r\n")]=0;
                if (k>0 && strcmp(x,pa)==0 && strcmp(y,pb)==0) continue;
                printf("+  orig=%s vm=%s\n", x, y);
                pa[0]=0; pb[0]=0;
            }
            break;
        }
        strcpy(pa,a); strcpy(pb,b);
    }
    return 0;
}
