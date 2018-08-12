#include <stdio.h>
#include  <stdlib.h>

int main(int argc, char** argv) {
    float *data= (float*)malloc((argc-1)*sizeof(float));
    for(int i=1; i<argc; i++)
        data[i-1] = atof(argv[i]);
    
    int center[100];
    int depth=0, l=0, r=argc-2;
    while(1) {
        if (r-l<2){
            if (depth-1 < 0 ) {
                break;
            }
            if (center[depth-1]<0) {
                r = l;
                l = -center[depth-1];
                depth--;
            } else {
                l = r;
                r = center[depth-1];
                depth--;
            }
        } else {
            int m = (r+l)/2;
            if (data[m]>data[m-1] && data[m]>data[m+1]) {
                printf("%d", m);
                break;
            } else if (data[m+1] >= data[m-1]) {
                center[depth++] = l;
                l = m;
            } else {
                center[depth++] = -r;
                r = m;
            }
        }
    }
    free(data);
    return 0;
}


