#include <stdio.h>
#include <string.h>
int main(int argc, char **argv){
    FILE *f=fopen(argv[1],"r"), *o=fopen(argv[2],"w");
    char a[32], prev[32] = {0};
    while(fgets(a,32,f)){
        a[strcspn(a,"\r\n")]=0;
        if (strcmp(a,prev)!=0) fprintf(o, "%s\n", a);
        strcpy(prev,a);
    }
    return 0;
}
