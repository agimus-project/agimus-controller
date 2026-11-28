// Tests of the real-time harness itself: the positive controls must pass, and
// every forbidden operation must be detected (doc/rt_safety.md).

#include <Eigen/Core>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "agimus_controller/trajectory_buffer.hpp"
#include "rt_guard.hpp"

namespace {
// Results and pointers are written here so that the compiler cannot elide the
// allocations (GCC removes new/delete pairs whose pointer does not escape).
volatile std::size_t g_sink = 0;
void* volatile g_escape = nullptr;
}  // namespace

TEST_CASE("RtHarness: fixed-size Eigen work is RT-safe") {
  const Eigen::Matrix3d a = Eigen::Matrix3d::Identity() * 2.0;
  const Eigen::Vector3d x(1.0, 2.0, 3.0);
  Eigen::Vector3d y;
  CHECK_RT_SAFE(y.noalias() = a * x);
  CHECK(y == 2.0 * x);
}

TEST_CASE(
    "RtHarness: dynamic Eigen work into preallocated storage is RT-safe") {
  const Eigen::MatrixXd a = Eigen::MatrixXd::Random(30, 30);
  const Eigen::VectorXd x = Eigen::VectorXd::Random(30);
  Eigen::VectorXd y(30);
  CHECK_RT_SAFE(y.noalias() = a * x);
  CHECK(y.isApprox(a * x));
}

TEST_CASE("RtHarness: vector growth is detected") {
  std::vector<double> v;
  v.reserve(1);
  v.push_back(1.0);
  CHECK_ALLOCATES(v.push_back(2.0));
  g_sink = v.size();
}

TEST_CASE(
    "RtHarness: a string longer than the small-string buffer is detected") {
  std::string s;
  CHECK_ALLOCATES(s.assign(100, 'x'));
  g_sink = s.size();
}

TEST_CASE("RtHarness: make_shared is detected") {
  std::shared_ptr<int> p;
  CHECK_ALLOCATES(p = std::make_shared<int>(1));
  g_sink = static_cast<std::size_t>(*p);
}

TEST_CASE("RtHarness: throw and catch is detected") {
  CHECK_ALLOCATES(
      try { throw std::runtime_error("x"); } catch (const std::exception& e) {
        g_sink = e.what()[0];
      });
}

TEST_CASE("RtHarness: freeing memory allocated earlier is detected") {
  int* p = new int(1);
  g_escape = p;
  CHECK_ALLOCATES(delete static_cast<int*>(g_escape));
}

TEST_CASE("RtHarness: C malloc is detected") {
  void* p = nullptr;
  CHECK_ALLOCATES(p = std::malloc(64));
  g_sink = reinterpret_cast<std::size_t>(p);
  std::free(p);
}

TEST_CASE("RtHarness: allocations inside a shared library are detected") {
  // compute_horizon_indexes lives in libagimus_controller_cpp and fills a
  // std::vector: proves the hook interposes calls from shared libraries.
  const agimus_controller::DtFactorsNSeq seq{{1, 2}, {3, 3}};
  std::vector<std::size_t> indexes;
  CHECK_ALLOCATES(indexes = agimus_controller::compute_horizon_indexes(seq));
  g_sink = indexes.size();
}

#ifdef EIGEN_RUNTIME_NO_MALLOC
TEST_CASE("RtHarness: a dynamic Eigen allocation aborts under the guard") {
  CHECK_ABORTS(agimus_controller::test::ScopedRtGuard guard;
               Eigen::VectorXd v(10); g_sink = v.size());
}

TEST_CASE("RtHarness: a dynamic Eigen temporary aborts under the guard") {
  CHECK_ABORTS(const Eigen::MatrixXd a = Eigen::MatrixXd::Identity(4, 4);
               const Eigen::VectorXd x = Eigen::VectorXd::Ones(4);
               agimus_controller::test::ScopedRtGuard guard;
               const Eigen::VectorXd y = a * x; g_sink = y.size());
}

TEST_CASE("RtHarness: no abort without the guard") {
  // Control for the two tests above: CHECK_ABORTS must not pass vacuously.
  CHECK_FALSE(agimus_controller::test::aborts(
      [](void*) {
        Eigen::VectorXd v(10);
        g_sink = v.size();
      },
      nullptr));
}
TEST_CASE("RtHarness: a resizing assignment aborts under the guard") {
  // Eigen 3.4 resizes on assignment even with EIGEN_NO_AUTOMATIC_RESIZING
  // (only PlainObjectBase::_resize_to_match checks it), so a size mismatch
  // is caught through the reallocation instead.
  CHECK_ABORTS(Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
               const Eigen::VectorXd b = Eigen::VectorXd::Zero(4);
               agimus_controller::test::ScopedRtGuard guard; a = b;
               g_sink = a.size());
}
#endif
