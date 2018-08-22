#include "sum_multithread.h"
#include <thread>
#include <iostream>

struct ThreadArg{
    int start;
    int end;
    int sum;
};

int run(ThreadArg& th, const std::vector<int>& nums)
{
   int sum = 0;
    for (int i = th.start; i <= th.end; i++)
    {
       sum += nums[i];
    }

    th.sum = sum;
}

int sum_multithread(const std::vector<int>& nums)
{
   int thread_num = std::thread::hardware_concurrency();
   int num_size = nums.size();
   int num_per_thread = num_size / thread_num;

   ThreadArg args[thread_num];
   for (int i = 0; i < (thread_num-1); i++)
   {
      args[i].start = i * num_per_thread;
      args[i].end = (i+1)* num_per_thread - 1;
   }

   args[thread_num-1].start = (thread_num-1)*num_per_thread;
   args[thread_num-1].end = num_size - 1;

   std::vector<std::thread> workers;
   for (int i = 0; i < thread_num; i++){
      workers.emplace_back(std::thread(run, std::ref(args[i]), std::ref(nums)));
   }


   for (int i = 0; i < thread_num; i++){
      workers[i].join();
   }
   int sum = 0;
   for (int i = 0; i < thread_num; i++){
      sum += args[i].sum;
   }
   return sum;
}

