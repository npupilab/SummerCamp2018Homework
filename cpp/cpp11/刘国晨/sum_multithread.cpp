#include "sum_multithread.h"
#include <thread>
#include <algorithm>
#include <iostream>
#include <vector>
#include <iterator>

using namespace std;


int sum_multithread(const std::vector<int>& nums)
{
    int thread_num = std::thread::hardware_concurrency();
    int num_size = nums.size();
    int task_num = num_size / thread_num;
    int num_sum = 0;
    vector<int> start;
    vector<int> end;

    for(int i = 0; i < thread_num - 1; ++i)
    {
        start.push_back(i * task_num);
        end.push_back((i + 1) * task_num - 1);
    }

    start.push_back((thread_num - 1) * task_num);
    end.push_back(num_size - 1);

    vector<int> sum(thread_num, 0);
    vector<thread> workers;

    auto run_func = [&](const int &id){
        auto s = nums.begin() + start[id];
        auto e = nums.begin() + end[id] + 1;
        sum[id] = std::accumulate(s, e, 0);
    };
    for(int i = 0; i < thread_num; ++i)
    {
        workers.emplace_back(std::thread(run_func, i));
    }

    for(auto &k : workers) k.join();
    num_sum = std::accumulate(sum.begin(), sum.end(), 0);
    return num_sum;
}

