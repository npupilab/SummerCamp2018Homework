#include <stdio.h>
#include <stdlib.h>

int findPeak(int* list, int start, int end)
{
    // tips: do not re-value start = start + center!!
    if (start==end) return start;
    int center = (start + end) / 2;

    if(list[center] < list[center+1]){
        start = center + 1;
        return findPeak(list, start, end);
    } 

    if(list[center] >= list[center+1]){
        end = center;
        return findPeak(list, start, end);
    }
}

int main(int argc, char* argv[])
{
    int *list = (int*)malloc(argc*sizeof(int));
    int i = 0;
    //printf("%d\n",argc);
    for( ; i < argc-1; i++)
    {
        list[i] = atoi(argv[i+1]);
        //printf("%d\n",list[i]);
    }

    int peak_id = findPeak(list, 0, argc-2);
    printf("%d\n",peak_id);
    return 0;
}
