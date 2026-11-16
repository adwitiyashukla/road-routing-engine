#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace rr {

using NodeId = std::uint32_t;
using Weight = std::uint32_t;

inline constexpr NodeId kNoNode = std::numeric_limits<NodeId>::max();
inline constexpr Weight kInf = std::numeric_limits<Weight>::max();
inline constexpr Weight kMaxArcWeight = (Weight{1} << 30) - 1;
inline constexpr std::uint32_t kMaxNodes = (std::uint32_t{1} << 30) - 1;

struct LatLon {
  std::int32_t lat = 0;
  std::int32_t lon = 0;
};

class Status {
 public:
  Status() = default;

  static Status Error(std::string message) {
    Status status;
    status.ok_ = false;
    status.message_ = std::move(message);
    return status;
  }

  bool ok() const { return ok_; }
  const std::string& message() const { return message_; }

 private:
  bool ok_ = true;
  std::string message_;
};

template <typename T>
class Result {
 public:
  Result(T value) : value_(std::move(value)) {}
  Result(Status status) : status_(std::move(status)) {}

  bool ok() const { return status_.ok(); }
  const Status& status() const { return status_; }
  T& value() { return *value_; }
  const T& value() const { return *value_; }
  T& operator*() { return *value_; }
  const T& operator*() const { return *value_; }
  T* operator->() { return &*value_; }
  const T* operator->() const { return &*value_; }

 private:
  Status status_;
  std::optional<T> value_;
};

#define RR_RETURN_IF_ERROR(expr)             \
  do {                                       \
    ::rr::Status rr_status_ = (expr);        \
    if (!rr_status_.ok()) return rr_status_; \
  } while (false)

class Timer {
 public:
  Timer() : start_(Clock::now()) {}

  void Reset() { start_ = Clock::now(); }

  double Seconds() const { return std::chrono::duration<double>(Clock::now() - start_).count(); }

 private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point start_;
};

inline std::uint64_t Mix64(std::uint64_t x) {
  x ^= x >> 30;
  x *= 0xbf58476d1ce4e5b9ULL;
  x ^= x >> 27;
  x *= 0x94d049bb133111ebULL;
  x ^= x >> 31;
  return x;
}

class Rng {
 public:
  explicit Rng(std::uint64_t seed) : state_(seed) {}

  std::uint64_t Next() {
    state_ += 0x9e3779b97f4a7c15ULL;
    return Mix64(state_);
  }

  std::uint32_t Below(std::uint32_t bound) {
    return static_cast<std::uint32_t>(((Next() >> 32) * bound) >> 32);
  }

  double Uniform() { return static_cast<double>(Next() >> 11) * 0x1.0p-53; }

 private:
  std::uint64_t state_;
};

}
