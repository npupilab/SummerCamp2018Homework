#pragma once

// C99 standard headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>
#include <iostream>
#include <eigen3/Eigen/Core>

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
class CTypes// SIMD, Little,Greater, float:
{
public:
    //EIGEN_ALIGN16// SSE,128bit,4float,16char AVX:256bit
    // NEON,128bit
    char   c;//1 p
    unsigned char  uc;//p+1
    short  s;//2 p+2
    int    i;//4 p+4
    float  f;//4 p+8
    unsigned int   ui;//4 p+12
    double         d;//8 p+16
    unsigned short us;//2 p+24
    char           str[6];// p+26
    void*          p;//4?8 p+32

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
    char           haha[8];

    void say(){printf("Hello");}
    virtual void hello(){}
    char           hehe[1];
} ctypes;

class TestVirtual
{
public:
    TestVirtual(){
        var=10;
    }
    virtual ~TestVirtual(){}

    double var;
};

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

int main(int argc,char** argv){
    // Input is several int number, such as 1 2 3 4 2 3 1
    // Please output the index of one local maximum as soon as possible
    // Please do not iterate every number, since the number could be huge
//    CTypes ctypes;
    void* p=&ctypes;
    //void* ps=&ctypes.s==++(char*)p;
    printf("p:%p,pc:%p,ps:%p,pp:%p,sizeof:%d",
           p,&ctypes.c,&ctypes.s,&ctypes.p,sizeof(TestVirtual));

    TestVirtual virt;
    return 0;
}
