
#include "sum_multithread.h"
#include <thread>
#include <vector>
#include <iostream>
void partsum( int no,int thread_nums, const std::vector<int>& nums, int& part_sum) //
{
    int size = nums.size();
    int start=size*no/thread_nums;
    int end =size*(no+1)/thread_nums;
    if(no == thread_nums-1)
        end = nums.size();
    for(int i = start; i < end; i++)
        part_sum+=nums[i];
}

int mergesum(int n,const int* nums)
{
    int sum=0;
    for(int i =0; i<n; i++)
        sum+=nums[i];
    return sum;

}
int sum_multithread(const std::vector<int>& nums)
{
    int thread_num=std::thread::hardware_concurrency();
    int sum=0;//总和
    int part_sum[thread_num] = {0};
    std::vector<std::thread> workers;
    for(int i = 0; i < thread_num; i++)
    {
       workers.emplace_back(std::thread(partsum, i,thread_num, std::ref(nums), std::ref(part_sum[i])));
    }
    for(auto &k : workers) k.join();
    sum = mergesum(thread_num, part_sum);

   return sum;
}

