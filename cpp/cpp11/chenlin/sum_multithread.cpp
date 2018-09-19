#include <thread>
#include <vector>
#include <numeric>
#include <iostream>

#include "sum_multithread.h"

int sum_multithread(const std::vector<int>& nums)
{
	int thread_num = std::thread::hardware_concurrency();
	int interval = nums.size() / thread_num;
	int parts[thread_num] = {0};
	std::vector<int> sum(thread_num, 0);
	std::vector<std::thread> workers;

	for (int i = 0; i < thread_num; ++i) {parts[i] = i * interval;}

	auto run_fuc = [&](const int& id) {
		auto s = nums.begin() + parts[id];
		auto e = nums.end();
		if (id != thread_num - 1) e = s + interval;
		sum[id] = std::accumulate(s, e, 0);
	};
	for (int i = 0; i < thread_num; i++)
	{
		workers.emplace_back(std::thread(run_fuc, i));
	}
	for (auto &k : workers) k.join();

	return std::accumulate(sum.begin(), sum.end(), 0);
}