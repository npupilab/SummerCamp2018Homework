// C99 standard headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>

int findLocalMax(double* p,int from,int to)
{
    int nextID=(from+to)/2;
    if(p[nextID]<p[nextID+1]) {from=nextID;return findLocalMax(p,from,to);}
    if(p[nextID]<p[nextID-1]) {to=nextID;return findLocalMax(p,from,to);}
    return nextID;
}

int main(int argc,char** argv){
    // Input is several int number, such as 1 2 3 4 2 3 1
    // Please output the index of one local maximum as soon as possible
    // Please do not iterate every number, since the number could be huge

    double* p=(double*)malloc( argc * sizeof(double) );
    for(int i=1;i<argc;i++)
        sscanf(argv[i],"%lf",p+i-1);

    int idx=findLocalMax(p,0,argc-1);
    printf("%d",idx);

    return 0;
}

