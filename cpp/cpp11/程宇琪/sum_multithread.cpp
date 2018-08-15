
#include "sum_multithread.h"
#include <thread>
#include <vector>
#include <iostream>
#include <pthread.h>

int partsum(int num, int no, const std::vector<int>& nums) //
{
    int start=(no/num);
    int end = (no+1)/num;
    int partsums=0;
    for(int i = start; i < end; i++)
                    partsums+=nums[i];
    return partsums;
}

int mergesum(int n,const std::vector<int>& nums)
{
    int sum=0;
    for(auto i : nums)
        sum+=i;
    return sum;

}
int sum_multithread(const std::vector<int>& nums)
{
    int thread_num=4;
    pthread_t threads[thread_num];
    int number = nums.size();
    int sum=0;
    int rc=0;
    std::vector<int> parts_sum;
    for(int i = 0; i < thread_num; i++)
    {
        std::cout << "main() : 创建线程, " << i << std::endl;
              // 传入的时候必须强制转换为void* 类型，即无类型指针
              rc = pthread_create(&threads[i], NULL,
                                  partsum, thread_num,i,nums);
              if (rc){
                 cout << "Error:无法创建线程," << rc << endl;

    }


   return sum;
}

