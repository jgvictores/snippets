
#include <stdio.h>

int main(int argc, char **argv) {
    FILE *file;
    file = fopen("file.txt","w");
    int i;
    for(i=0;i<argc;i++) {
        printf("%s\n",argv[i]);
        fprintf(file,"%s\n",argv[i]);
    }
    fclose(file);
    return 0;
}
