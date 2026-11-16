#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace rr {

class ThreadPool {
 public:
  using Job = std::function<void(std::size_t begin, std::size_t end, int worker)>;

  explicit ThreadPool(int threads);
  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;
  ~ThreadPool();

  int size() const { return static_cast<int>(workers_.size()) + 1; }
  void Run(std::size_t count, std::size_t grain, const Job& job);

  static int DefaultThreads();

 private:
  void Loop(int worker);
  void Work(int worker);

  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable done_;
  const Job* job_ = nullptr;
  std::size_t count_ = 0;
  std::size_t grain_ = 1;
  std::atomic<std::size_t> next_{0};
  std::uint64_t generation_ = 0;
  int active_ = 0;
  bool stop_ = false;
};

}
