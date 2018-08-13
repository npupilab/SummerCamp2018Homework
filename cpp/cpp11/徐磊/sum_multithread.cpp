#include "sum_multithread.h"
#include <functional>
#include <iostream>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>


int sum_multithread(const std::vector<int>& nums) {
  int thread_num = std::thread::hardware_concurrency();
  int num_size = nums.size();
  int task_num = num_size / thread_num;
  int sum_sum = 0;
  std::vector<int> start;
  std::vector<int> end;

  for (int i = 0; i < thread_num - 1; i++) {
    start.push_back(i * task_num);
    end.push_back((i + 1) * task_num - 1);
  }
  start.push_back((thread_num - 1) * task_num);
  end.push_back((num_size - 1));

  std::vector<int> sum(thread_num, 0);
  std::vector<std::thread> workers;

  auto run_fuc = [&](const int& id) {
    auto s = nums.begin() + start[id];
    auto e = nums.begin() + end[id] + 1;
    sum[id] = std::accumulate(s, e, 0);
  };
  for (int i = 0; i < thread_num; i++) {
    workers.emplace_back(std::thread(run_fuc, i));
  }

  // i was not pass in thread,Why?
  //  for (int i = 0; i < thread_num; i++) {
  //    workers.emplace_back(std::thread([&]() {
  //      int id = i;
  //      auto s = nums.begin() + start[id];
  //      auto e = nums.begin() + end[id] + 1;
  //      sum[id] = std::accumulate(s, e, 0);
  //    }));
  //  }

  for (auto& k : workers) k.join();
  sum_sum = std::accumulate(sum.begin(), sum.end(), 0);
  return sum_sum;
}
