#pragma once

// C99 standard headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>

// C99 macros
#if defined(INT_MIN)
#ifdef LOG
#undef LOG
#endif
#define LOG(msg) printf("%s %s %s %d %d %s\n",(char *)__DATE__,(char *)__TIME__,\
                        (char *)__FILE__,__LINE__,__STDC__,msg);

#elif !defined(INT_MAX)
#define max(a,b) (a)>(b)?(a):(b)
#else
#error "this should not happen!"
#endif

// C99 buildin types and struct
struct CTypes
{
    char   c;//1
    short  s;//2
    int    i;//4
    float  f;//4
    double         d;//8
    unsigned char  uc;//1
    unsigned short us;//2
    unsigned int   ui;//4
    char           str[16];//16
    void*          p;//4?8

    union Union{
        char c;
        float f;
    }u;//4

    struct BitInt{
        unsigned int bi1 : 1;
        unsigned int bi234 : 3;
        unsigned int bi4 : 1;
    }bi;//4

    enum Day{Sunday=0,Monday,};

    // Do not use long types as possible
    // since it has different size on different systems!
    long           l;//4?8
    long long      ll;//8
    long double    ld;//12?16
    unsigned long  ul;//4?8
} ctypes;

double average(int num,...)
{

    va_list valist;
    double sum = 0.0;
    int i;

    /* 为 num 个参数初始化 valist */
    va_start(valist, num);

    /* 访问所有赋给 valist 的参数 */
    for (i = 0; i < num; i++)
    {
       sum += va_arg(valist, int);
    }
    /* 清理为 valist 保留的内存 */
    va_end(valist);

    return sum/num;
}

void runtest(){
    printf("sizeof ctypes:%d, void*: %d, BitInt:%d, "
           "long:%d, long long: %d, long double:%d\n",
           sizeof(ctypes),sizeof(ctypes.p), sizeof(ctypes.bi),
           sizeof(ctypes.l),sizeof(ctypes.ll),sizeof(ctypes.ld));
    LOG("This is a log");
    printf("Average of 5, 10, 15 = %f\n", average(3, 5,10,15));
}

void funcPointerTest()
{
    void(*testFunc)() =runtest;
    testFunc();
}

int findLocalMax(double* p,int from,int to)
{
    int nextID=(from+to)/2;
    if(p[nextID]<=p[nextID+1]) {from=nextID;return findLocalMax(p,from,to);}
    if(p[nextID]<=p[nextID-1]) {to=nextID;return findLocalMax(p,from,to);}
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

