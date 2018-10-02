#include "sum_multithread.h"
#include <thread>
#include <mutex>
#include <iostream>
#include <atomic>


using namespace std;

mutex m;//声明互斥锁m
long sum = 0;
//atomic<int> sum(0); //使用原子操作



void sumPart(const std::vector<int>& nums, int i, int thread_num, int len)
{
    if(i != thread_num)
    {
        m.lock();
        for(int j = (i-1)*len; j < i*len; j++)
            sum += nums[j];
        m.unlock();
    }
    else
    {
        m.lock();
        for(int j = (i-1)*len; j < int(nums.size()); j++)
            sum += nums[j];
        m.unlock();
    }
}

int sum_multithread(const std::vector<int>& nums)
{
    int len = nums.size();
    int thread_num = std::thread::hardware_concurrency();
    int num_per_thread = len / thread_num;
//
    thread th[thread_num];

    for(int i = 0; i < thread_num; i++)
    {
        th[i] = thread(sumPart, std::ref(nums), i+1, thread_num, num_per_thread);
    }

    for(int i = 0; i < thread_num; i++)
    {
        th[i].join();
    }

   return sum;
}

