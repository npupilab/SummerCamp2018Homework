#include <stdio.h>
#include<stdlib.h>
int findPeak(int left, int right,float *argv)
{
	int mid = (left + right) / 2;
	if (argv[mid] > argv[mid+1] && argv[mid] > argv[mid-1]) return mid;
	else if (argv[mid] > argv[mid+1]) return findPeak(left,mid-1,argv);
	else return findPeak(mid+1, right,argv);
}

int main(int argc, char const *argv[])
{
    float* arr = (float*)malloc(argc * sizeof(float));
    for(int i = 1; i < argc; ++i)
        arr[i-1] = atof ( argv[i]);
    printf("%d",findPeak(0,argc-1, arr));
	return 0;
}