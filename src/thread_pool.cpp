#include "thread_pool.h"

#include <algorithm>

namespace rr {

ThreadPool::ThreadPool(int threads) {
  int extra = std::max(threads, 1) - 1;
  for (int i = 0; i < extra; ++i) workers_.emplace_back([this, i] { Loop(i + 1); });
}

ThreadPool::~ThreadPool() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  for (std::thread& worker : workers_) worker.join();
}

int ThreadPool::DefaultThreads() {
  unsigned hw = std::thread::hardware_concurrency();
  return hw == 0 ? 1 : static_cast<int>(hw);
}

void ThreadPool::Run(std::size_t count, std::size_t grain, const Job& job) {
  if (count == 0) return;
  grain = std::max<std::size_t>(grain, 1);
  if (workers_.empty() || count <= grain) {
    for (std::size_t begin = 0; begin < count; begin += grain) {
      job(begin, std::min(begin + grain, count), 0);
    }
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    job_ = &job;
    count_ = count;
    grain_ = grain;
    next_.store(0);
    ++generation_;
    ++active_;
  }
  wake_.notify_all();
  Work(0);
  std::unique_lock<std::mutex> lock(mutex_);
  --active_;
  done_.wait(lock, [this] { return active_ == 0; });
  job_ = nullptr;
}

void ThreadPool::Loop(int worker) {
  std::uint64_t seen = 0;
  while (true) {
    std::unique_lock<std::mutex> lock(mutex_);
    wake_.wait(lock, [&] { return stop_ || generation_ != seen; });
    if (stop_) return;
    seen = generation_;
    if (job_ == nullptr) continue;
    ++active_;
    lock.unlock();
    Work(worker);
    lock.lock();
    if (--active_ == 0) done_.notify_all();
  }
}

void ThreadPool::Work(int worker) {
  while (true) {
    std::size_t begin = next_.fetch_add(grain_);
    if (begin >= count_) return;
    (*job_)(begin, std::min(begin + grain_, count_), worker);
  }
}

}
