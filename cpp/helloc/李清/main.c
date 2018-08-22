#include <stdio.h>
#include <stdlib.h>

int findmax(float* data, int start, int end)
{
    if (start == end)
        return -1;

    int id = (start + end) /2;
    printf("%d ", id);

    if (id +1 >= end || id -1 < 0) return -1;

    if (data[id]>data[id+1] && data[id]>data[id-1])
    {
        return id;
    }

    int p =  findmax(data, start, id);
    if (p>0)
        return p;
    int q =  findmax(data, id, end);
     if (q>0)
         return q;
}

int main(int argc, char** argv) {
    printf("%d ", argc);
    float* data = (float*)malloc((argc-1) * sizeof(float));
    for (int i = 1; i < argc; i++) {
        data[i - 1] = atof(argv[i]);
        printf("%f ", data[i-1]);
    }

    int result = findmax(data, 0, argc-2);
    printf("%d", result);


    return 0;
}