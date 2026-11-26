// Port of agimus_controller/tests/test_buffer.py (TrajectoryBuffer and
// ConstantTrajectoryBuffer), plus capacity, wrap-around and RT tests.

#include <chrono>
#include <stdexcept>

#include "agimus_controller/trajectory_buffer.hpp"
#include "rt_guard.hpp"

using agimus_controller::BufferMode;
using agimus_controller::compute_horizon_indexes;
using agimus_controller::DtFactorsNSeq;
using agimus_controller::make_trajectory_point;
using agimus_controller::Status;
using agimus_controller::TrajectoryBuffer;
using agimus_controller::WeightedTrajectoryPoint;

namespace {

constexpr Eigen::Index kNv = 23;
constexpr Eigen::Index kNq = kNv + 1;
constexpr int kNControls = 10;
constexpr std::chrono::nanoseconds kDt = std::chrono::milliseconds(10);
constexpr std::size_t kTrajectorySize = 3000;  // Python: 30 * 100 points.

const DtFactorsNSeq kSimpleSeq{{1}, {kNControls}};
const DtFactorsNSeq kComplexSeq{{1, 2, 3, 4, 5}, {2, 2, 2, 2, 2}};

WeightedTrajectoryPoint prototype() { return make_trajectory_point(kNq, kNv); }

// A random point at time step * kDt, with id = step.
WeightedTrajectoryPoint random_point(int step) {
  auto p = prototype();
  p.time = step * kDt;
  p.id = step;
  p.q.setRandom();
  p.qdot.setRandom();
  p.qddot.setRandom();
  p.robot_effort.setRandom();
  p.w_q.setRandom();
  p.w_qdot.setRandom();
  p.w_qddot.setRandom();
  p.w_robot_effort.setRandom();
  return p;
}

TrajectoryBuffer filled_sliding(const DtFactorsNSeq& seq, std::size_t n) {
  TrajectoryBuffer buffer(seq, n, BufferMode::kSliding, prototype());
  for (std::size_t i = 0; i < n; ++i) {
    REQUIRE(buffer.push_back(random_point(static_cast<int>(i))) == Status::kOk);
  }
  return buffer;
}

TrajectoryBuffer constant_buffer(const DtFactorsNSeq& seq = kSimpleSeq) {
  return TrajectoryBuffer(seq, 0, BufferMode::kConstant, prototype());
}

}  // namespace

// ---- Sliding mode (Python TrajectoryBuffer) --------------------------------

TEST_CASE("TrajectoryBuffer: append data") {
  const auto buffer = filled_sliding(kSimpleSeq, kTrajectorySize);
  CHECK(buffer.size() == kTrajectorySize);
}

TEST_CASE("TrajectoryBuffer: clear past") {
  auto buffer = filled_sliding(kSimpleSeq, kTrajectorySize);
  const auto second = buffer[1];
  buffer.clear_past();
  CHECK(buffer.size() == kTrajectorySize - 1);
  CHECK(buffer[0] == second);
  buffer.clear_past();
  CHECK(buffer.size() == kTrajectorySize - 2);
  buffer.clear_past();
  CHECK(buffer.size() == kTrajectorySize - 3);
}

TEST_CASE("TrajectoryBuffer: compute horizon indexes") {
  const std::vector<std::size_t> expected{0, 1, 2, 4, 6, 9, 12, 16, 20, 25, 30};
  CHECK(compute_horizon_indexes(kComplexSeq) == expected);
  const TrajectoryBuffer buffer(kComplexSeq, 31, BufferMode::kSliding,
                                prototype());
  CHECK(buffer.horizon_indexes() == expected);
  CHECK_THROWS_AS(compute_horizon_indexes(DtFactorsNSeq{{1}, {1, 2}}),
                  std::invalid_argument);
}

TEST_CASE("TrajectoryBuffer: horizon") {
  auto buffer = filled_sliding(kSimpleSeq, kTrajectorySize);
  const auto& horizon = buffer.horizon();
  REQUIRE(horizon.size() == kNControls + 1);
  for (std::size_t k = 0; k < horizon.size(); ++k) {
    CHECK(*horizon[k] == buffer[k]);
  }
}

TEST_CASE("TrajectoryBuffer: horizon with a more complex dt_factor_n_seq") {
  auto buffer = filled_sliding(kComplexSeq, kTrajectorySize);
  const auto& indexes = buffer.horizon_indexes();
  const auto& horizon = buffer.horizon();
  REQUIRE(horizon.size() == kNControls + 1);
  for (std::size_t k = 0; k < horizon.size(); ++k) {
    CHECK(*horizon[k] == buffer[indexes[k]]);
  }
}

TEST_CASE("TrajectoryBuffer: horizon is available once past the last index") {
  // B2: Python only checked len >= n_controls + 1, but the horizon needs
  // len > horizon_indexes[-1] (30 here, while n_controls + 1 = 11).
  TrajectoryBuffer buffer(kComplexSeq, 64, BufferMode::kSliding, prototype());
  for (int i = 0; i < 30; ++i) {
    REQUIRE(buffer.push_back(random_point(i)) == Status::kOk);
  }
  CHECK(buffer.size() >= kNControls + 1);
  CHECK_FALSE(buffer.has_horizon());
  REQUIRE(buffer.push_back(random_point(30)) == Status::kOk);
  CHECK(buffer.has_horizon());
}

TEST_CASE("TrajectoryBuffer: capacity, full buffer and wrap-around (B9)") {
  CHECK_THROWS_AS(TrajectoryBuffer(kSimpleSeq, kNControls, BufferMode::kSliding,
                                   prototype()),
                  std::invalid_argument);

  const std::size_t capacity = 15;
  TrajectoryBuffer buffer(kSimpleSeq, capacity, BufferMode::kSliding,
                          prototype());
  for (std::size_t i = 0; i < capacity; ++i) {
    REQUIRE(buffer.push_back(random_point(static_cast<int>(i))) == Status::kOk);
  }
  CHECK(buffer.push_back(random_point(0)) == Status::kBufferFull);

  // Consume 5, add 5: the ring wraps and the order is kept.
  for (int i = 0; i < 5; ++i) buffer.clear_past();
  for (int i = 15; i < 20; ++i) {
    REQUIRE(buffer.push_back(random_point(i)) == Status::kOk);
  }
  REQUIRE(buffer.size() == capacity);
  for (std::size_t i = 0; i < capacity; ++i) {
    CHECK(buffer[i].time == static_cast<int>(i + 5) * kDt);
  }
  CHECK(buffer.back().time == 19 * kDt);
  const auto& horizon = buffer.horizon();
  CHECK(horizon.back()->time == 15 * kDt);
}

TEST_CASE("TrajectoryBuffer: a point of other sizes is rejected") {
  TrajectoryBuffer buffer(kSimpleSeq, 20, BufferMode::kSliding, prototype());
  CHECK(buffer.push_back(make_trajectory_point(kNq, kNv + 1)) ==
        Status::kSizeMismatch);
  CHECK(buffer.size() == 0);
}

// ---- Constant mode (Python ConstantTrajectoryBuffer) -----------------------

TEST_CASE("ConstantBuffer: empty buffer has no horizon") {
  // Python raised AssertionError on .horizon; here has_horizon() is false.
  const auto buffer = constant_buffer();
  CHECK_FALSE(buffer.has_horizon());
}

TEST_CASE("ConstantBuffer: empty buffer size is zero") {
  CHECK(constant_buffer().size() == 0);
}

TEST_CASE("ConstantBuffer: size is sufficient after one point") {
  auto buffer = constant_buffer();
  buffer.push_back(random_point(0));
  CHECK(buffer.size() > buffer.horizon_indexes().back());
}

TEST_CASE("ConstantBuffer: horizon length") {
  auto buffer = constant_buffer();
  buffer.push_back(random_point(0));
  CHECK(buffer.horizon().size() == kNControls + 1);
}

TEST_CASE("ConstantBuffer: horizon repeats one stored point") {
  auto buffer = constant_buffer();
  const auto point = random_point(0);
  buffer.push_back(point);
  const auto& horizon = buffer.horizon();
  for (const auto* h : horizon) {
    CHECK(h == horizon[0]);  // Same object, no copies.
    CHECK(*h == point);
  }
}

TEST_CASE("ConstantBuffer: push_back replaces the latest point") {
  auto buffer = constant_buffer();
  for (int i = 0; i < 5; ++i) buffer.push_back(random_point(i));
  const auto last = random_point(99);
  buffer.push_back(last);
  for (const auto* h : buffer.horizon()) CHECK(*h == last);
}

TEST_CASE("ConstantBuffer: clear_past is a no-op") {
  auto buffer = constant_buffer();
  const auto point = random_point(0);
  buffer.push_back(point);
  const auto size_before = buffer.size();
  buffer.clear_past();
  CHECK(buffer.size() == size_before);
  CHECK(*buffer.horizon()[0] == point);
}

TEST_CASE("ConstantBuffer: operator[] always returns the latest point") {
  auto buffer = constant_buffer();
  const auto point = random_point(0);
  buffer.push_back(point);
  for (std::size_t i : {0u, 5u, 100u}) CHECK(buffer[i] == point);
}

TEST_CASE("ConstantBuffer: writing through operator[] updates the point") {
  auto buffer = constant_buffer();
  buffer.push_back(random_point(0));
  const auto new_point = random_point(42);
  buffer[0] = new_point;
  for (const auto* h : buffer.horizon()) CHECK(*h == new_point);
}

TEST_CASE("ConstantBuffer: back returns the latest point (Python pop)") {
  auto buffer = constant_buffer();
  const auto point = random_point(0);
  buffer.push_back(point);
  CHECK(buffer.back() == point);
  CHECK(*buffer.horizon()[0] == point);  // Still there.
}

TEST_CASE("ConstantBuffer: complex dt_factor_n_seq horizon length") {
  auto buffer = constant_buffer(DtFactorsNSeq{{1, 2, 3}, {3, 3, 3}});
  buffer.push_back(random_point(0));
  CHECK(buffer.horizon().size() == 3 + 3 + 3 + 1);
}

// ---- Real-time tests --------------------------------------------------------

TEST_CASE("RtTrajectoryBuffer.push_back: sliding, full, wrong size, constant") {
  TrajectoryBuffer buffer(kSimpleSeq, 12, BufferMode::kSliding, prototype());
  const auto point = random_point(0);
  const auto wrong = make_trajectory_point(kNq, kNv + 1);
  Status status = Status::kOk;
  CHECK_RT_SAFE(for (int i = 0; i < 12; ++i) status = buffer.push_back(point));
  CHECK(status == Status::kOk);
  CHECK_RT_SAFE(status = buffer.push_back(point));
  CHECK(status == Status::kBufferFull);
  CHECK_RT_SAFE(status = buffer.push_back(wrong));
  CHECK(status == Status::kSizeMismatch);
  // Re-appending a point of the buffer itself (ROS node fallback).
  buffer.clear_past();
  CHECK_RT_SAFE(status = buffer.push_back(buffer.back()));
  CHECK(status == Status::kOk);

  auto constant = constant_buffer();
  CHECK_RT_SAFE(status = constant.push_back(point));
  CHECK(status == Status::kOk);
}

TEST_CASE("RtTrajectoryBuffer.clear_past: sliding, empty, constant") {
  auto buffer = filled_sliding(kSimpleSeq, 20);
  TrajectoryBuffer empty(kSimpleSeq, 20, BufferMode::kSliding, prototype());
  auto constant = constant_buffer();
  constant.push_back(random_point(0));
  CHECK_RT_SAFE(buffer.clear_past(); empty.clear_past(); constant.clear_past());
  CHECK(buffer.size() == 19);
  CHECK(empty.size() == 0);
}

TEST_CASE("RtTrajectoryBuffer.has_horizon: both modes") {
  const auto full = filled_sliding(kSimpleSeq, kNControls + 1);
  TrajectoryBuffer short_buffer(kSimpleSeq, 20, BufferMode::kSliding,
                                prototype());
  for (int i = 0; i < kNControls; ++i) short_buffer.push_back(random_point(0));
  const auto constant = constant_buffer();
  bool a = false, b = true, c = true;
  CHECK_RT_SAFE(a = full.has_horizon(); b = constant.has_horizon();
                c = short_buffer.has_horizon());
  CHECK(a);
  CHECK_FALSE(b);
  CHECK_FALSE(c);
}

TEST_CASE("RtTrajectoryBuffer.horizon: sliding and constant, repeated calls") {
  auto buffer = filled_sliding(kComplexSeq, 64);
  auto constant = constant_buffer(kComplexSeq);
  constant.push_back(random_point(0));
  std::size_t n = 0;
  CHECK_RT_SAFE(for (int tick = 0; tick < 10; ++tick) {
    n += buffer.horizon().size();
    n += constant.horizon().size();
    buffer.clear_past();
  });
  CHECK(n == 10 * 2 * (kNControls + 1));
}

TEST_CASE("RtTrajectoryBuffer.size: both modes") {
  const auto buffer = filled_sliding(kSimpleSeq, 20);
  auto constant = constant_buffer();
  constant.push_back(random_point(0));
  std::size_t a = 0, b = 0;
  CHECK_RT_SAFE(a = buffer.size(); b = constant.size());
  CHECK(a == 20);
  CHECK(b == 3 * (kNControls + 1));
}

TEST_CASE("RtTrajectoryBuffer.operator[]: read and write") {
  auto buffer = filled_sliding(kSimpleSeq, 20);
  const auto point = random_point(7);
  std::chrono::nanoseconds t{0};
  CHECK_RT_SAFE(buffer[3] = point; t = buffer[3].time);
  CHECK(t == 7 * kDt);
}

TEST_CASE("RtTrajectoryBuffer.back: newest point") {
  const auto buffer = filled_sliding(kSimpleSeq, 20);
  std::chrono::nanoseconds t{0};
  CHECK_RT_SAFE(t = buffer.back().time);
  CHECK(t == 19 * kDt);
}
