#pragma once

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace rr::testing {

struct Case {
  const char* name;
  void (*body)();
};

inline std::vector<Case>& Cases() {
  static std::vector<Case> cases;
  return cases;
}

inline int& Failures() {
  static int failures = 0;
  return failures;
}

inline void Report(const char* file, int line, const std::string& message) {
  std::printf("  %s:%d: %s\n", file, line, message.c_str());
  ++Failures();
}

struct Register {
  Register(const char* name, void (*body)()) { Cases().push_back({name, body}); }
};

inline int RunAll() {
  int failed = 0;
  for (const Case& test : Cases()) {
    int before = Failures();
    std::printf("%s\n", test.name);
    test.body();
    if (Failures() != before) ++failed;
  }
  std::printf("%zu cases, %d failed, %d checks failed\n", Cases().size(), failed, Failures());
  return Failures() == 0 ? 0 : 1;
}

template <typename T>
std::string Show(const T& value) {
  return std::to_string(value);
}

inline std::string Show(const std::string& value) { return value; }
inline std::string Show(bool value) { return value ? "true" : "false"; }

}

#define RR_TEST(name)                                              \
  static void name();                                              \
  static ::rr::testing::Register rr_register_##name(#name, &name); \
  static void name()

#define RR_CHECK(condition)                                             \
  do {                                                                  \
    if (!(condition)) {                                                 \
      ::rr::testing::Report(__FILE__, __LINE__, "failed: " #condition); \
      return;                                                           \
    }                                                                   \
  } while (false)

#define RR_CHECK_EQ(left, right)                                         \
  do {                                                                   \
    auto rr_left = (left);                                               \
    auto rr_right = (right);                                             \
    if (!(rr_left == rr_right)) {                                        \
      ::rr::testing::Report(__FILE__, __LINE__,                          \
                            std::string(#left " == " #right " gave ") +  \
                                ::rr::testing::Show(rr_left) + " and " + \
                                ::rr::testing::Show(rr_right));          \
      return;                                                            \
    }                                                                    \
  } while (false)

#define RR_CHECK_OK(result)                                                                  \
  do {                                                                                       \
    if (!(result).ok()) {                                                                    \
      ::rr::testing::Report(__FILE__, __LINE__,                                              \
                            std::string(#result " failed: ") + (result).status().message()); \
      return;                                                                                \
    }                                                                                        \
  } while (false)

#define RR_CHECK_STATUS(status)                                                      \
  do {                                                                               \
    ::rr::Status rr_status = (status);                                               \
    if (!rr_status.ok()) {                                                           \
      ::rr::testing::Report(__FILE__, __LINE__,                                      \
                            std::string(#status " failed: ") + rr_status.message()); \
      return;                                                                        \
    }                                                                                \
  } while (false)
