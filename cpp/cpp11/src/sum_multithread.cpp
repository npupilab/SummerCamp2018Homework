#include "sum_multithread.h"
#include <thread>
#include <future>
#include <queue>
#include <functional>
#include <iostream>

// A simple threadpool implementation.
class ThreadPool {
 public:
  // All the threads are created upon construction.
  explicit ThreadPool(size_t num_threads): stop(false) {
        if(num_threads< 1) num_threads=1;
        for (size_t i = 0; i < num_threads; ++i) {
          workers.emplace_back([this] {
            for (;;) {
              std::function<void()> task;

              {
                std::unique_lock<std::mutex> lock(this->queue_mutex);
                this->condition.wait(lock, [this] {
                  return this->stop || !this->tasks.empty();
                });
                if (this->stop && this->tasks.empty()) return;
                task = std::move(this->tasks.front());
                this->tasks.pop();
              }

              task();
            }
            });
        }
      }
  ~ThreadPool(){
        {
          std::unique_lock<std::mutex> lock(queue_mutex);
          stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers)
          worker.join();
      }


  // Adds a task to the threadpool.
  template <class F, class... Args>
  auto Add(F&& f, Args&& ... args)
      ->std::future<typename std::result_of<F(Args...)>::type>;

 private:
  // Keep track of threads so we can join them
  std::vector<std::thread> workers;
  // The task queue
  std::queue<std::function<void()> > tasks;

  // Synchronization
  std::mutex queue_mutex;
  std::condition_variable condition;
  bool stop;

};

// add new work item to the pool
template <class F, class... Args>
auto ThreadPool::Add(F&& f, Args&& ... args)
    ->std::future<typename std::result_of<F(Args...)>::type> {
  using return_type = typename std::result_of<F(Args...)>::type;

  auto task = std::make_shared<std::packaged_task<return_type()> >(
      std::bind(std::forward<F>(f), std::forward<Args>(args)...));

  std::future<return_type> res = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queue_mutex);

    // don't allow enqueueing after stopping the pool
    if(stop)
        std::cerr<< "The ThreadPool object has been destroyed! Cannot add more "
                    "tasks to the ThreadPool!";

    tasks.emplace([task]() {
      (*task)();
    });
  }
  condition.notify_one();
  return res;
}

ThreadPool threads(8);

int sum_multithread(const std::vector<int>& nums)
{
    int threadNum=8;
    std::vector<std::future<int> > futures;
    int packageSize=nums.size()/threadNum;
    if(packageSize<1) packageSize++;
    for(size_t startIdx=0;startIdx<nums.size();startIdx+=packageSize)
    {
        size_t endIdx  =startIdx+packageSize-1;
        endIdx=endIdx<nums.size()?endIdx:(nums.size()-1);
        futures.push_back(threads.Add([](std::vector<int>::const_iterator begin,std::vector<int>::const_iterator end){
                                         int sum=0;
                                         for(auto it=begin;it<=end;it++)
                                              sum+=*it;
                                          return sum;
                                      },nums.begin()+startIdx,nums.begin()+endIdx));
    }
    for(auto& f:futures) f.wait();

    int sumThreads=0;
    for(auto& f:futures)
        sumThreads+=f.get();
    return sumThreads;
}

