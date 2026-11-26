#pragma once

// Real-time test harness. See doc/rt_safety.md.
//
// CHECK_RT_SAFE(stmt)   stmt must not allocate or free (counted by the hook in
//                       allocation_hook.cpp). An Eigen allocation in code
//                       compiled with EIGEN_RUNTIME_NO_MALLOC aborts instead.
// CHECK_ALLOCATES(stmt) stmt must allocate or free (negative tests).
// CHECK_ABORTS(stmt)    stmt must abort (Eigen asserts); runs in a fork. The
//                       statement cannot use the test's local variables.
//
// Never put doctest assertions inside these statements: doctest allocates.
// AGIMUS_RT_ABORT_ON_ALLOC=1 aborts on the first counted allocation, to get a
// backtrace under gdb.

#include <doctest/doctest.h>

#include <cstddef>

namespace agimus_controller::test {

struct AllocationCount {
  std::size_t calls = 0;
  std::size_t bytes = 0;
};

// Implemented in allocation_hook.cpp. Counting is per thread.
void start_counting() noexcept;
AllocationCount stop_counting() noexcept;
AllocationCount current_count() noexcept;

/// Counts allocations of this thread and forbids Eigen allocations while
/// alive.
class ScopedRtGuard {
 public:
  ScopedRtGuard() noexcept;
  ~ScopedRtGuard();
  ScopedRtGuard(const ScopedRtGuard&) = delete;
  ScopedRtGuard& operator=(const ScopedRtGuard&) = delete;

  std::size_t allocations() const noexcept { return current_count().calls; }
  std::size_t bytes() const noexcept { return current_count().bytes; }

 private:
  bool eigen_malloc_was_allowed_ = true;
};

/// Runs `fn` in a forked child with stderr silenced; true if the child was
/// killed by SIGABRT.
bool aborts(void (*fn)(void*), void* context);

}  // namespace agimus_controller::test

#define AGIMUS_RT_COUNT_(count_var, ...)                       \
  ::agimus_controller::test::AllocationCount count_var;        \
  {                                                            \
    ::agimus_controller::test::ScopedRtGuard agimus_rt_guard_; \
    __VA_ARGS__;                                               \
    count_var = ::agimus_controller::test::current_count();    \
  }

#define CHECK_RT_SAFE(...)                                                 \
  do {                                                                     \
    AGIMUS_RT_COUNT_(agimus_rt_count_, __VA_ARGS__)                        \
    CHECK_MESSAGE(agimus_rt_count_.calls == 0u,                            \
                  "RT violation in `" #__VA_ARGS__ "`: "                   \
                      << agimus_rt_count_.calls << " allocation call(s), " \
                      << agimus_rt_count_.bytes << " bytes");              \
  } while (0)

#define CHECK_ALLOCATES(...)                                       \
  do {                                                             \
    AGIMUS_RT_COUNT_(agimus_rt_count_, __VA_ARGS__)                \
    CHECK_MESSAGE(agimus_rt_count_.calls > 0u,                     \
                  "expected an allocation in `" #__VA_ARGS__ "`"); \
  } while (0)

#define CHECK_ABORTS(...)                                                    \
  do {                                                                       \
    auto agimus_rt_fn_ = [](void*) { __VA_ARGS__; };                         \
    CHECK_MESSAGE(::agimus_controller::test::aborts(agimus_rt_fn_, nullptr), \
                  "expected `" #__VA_ARGS__ "` to abort");                   \
  } while (0)
