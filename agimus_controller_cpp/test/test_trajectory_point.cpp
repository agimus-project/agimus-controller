#include "agimus_controller/trajectory_point.hpp"
#include "rt_guard.hpp"

using agimus_controller::EndEffectorInput;
using agimus_controller::find_ee_input;
using agimus_controller::FrameName;
using agimus_controller::interpolate_weights;
using agimus_controller::kMaxEndEffectors;
using agimus_controller::make_robot_state;
using agimus_controller::make_trajectory_point;
using agimus_controller::Status;
using agimus_controller::Vector6d;
using agimus_controller::WeightedTrajectoryPoint;

namespace {

constexpr Eigen::Index kNq = 8;  // Not equal to nv on purpose (free flyer).
constexpr Eigen::Index kNv = 7;

EndEffectorInput make_ee(std::string_view name, double w) {
  EndEffectorInput ee;
  ee.frame_name = FrameName(name);
  ee.pose = pinocchio::SE3::Random();
  ee.w_pose = Vector6d::Constant(w);
  ee.twist = pinocchio::Motion::Random();
  ee.w_twist = Vector6d::Constant(2 * w);
  ee.force = pinocchio::Force::Random();
  ee.w_force = Vector6d::Constant(3 * w);
  return ee;
}

WeightedTrajectoryPoint make_weighted(double w) {
  auto p = make_trajectory_point(kNq, kNv);
  p.q.setRandom();
  p.w_q.setConstant(w);
  p.w_qdot.setConstant(2 * w);
  p.w_qddot.setConstant(3 * w);
  p.w_robot_effort.setConstant(4 * w);
  p.w_collision_avoidance = 5 * w;
  return p;
}

}  // namespace

TEST_CASE("make_trajectory_point: sizes and zeros") {
  const auto p = make_trajectory_point(kNq, kNv);
  CHECK(p.q.size() == kNq);
  CHECK(p.w_q.size() == kNv);  // Tangent space, like the state residual.
  for (const auto* v : {&p.qdot, &p.w_qdot, &p.qddot, &p.w_qddot,
                        &p.robot_effort, &p.w_robot_effort}) {
    CHECK(v->size() == kNv);
    CHECK(v->isZero());
  }
  CHECK(p.ee_inputs.empty());
  CHECK(p.id == -1);

  const auto s = make_robot_state(kNq, kNv);
  CHECK(s.q.size() == kNq);
  CHECK(s.v.size() == kNv);
  CHECK(s.a.size() == kNv);
  CHECK(s.contact_force.isZero());
}

TEST_CASE("EndEffectorInput: defaults") {
  const EndEffectorInput ee;
  CHECK(ee.frame_name.empty());
  CHECK(ee.pose.isIdentity());
  CHECK(ee.w_pose.isZero());
  CHECK(ee.twist.isZero());
  CHECK(ee.force.isZero());
}

TEST_CASE("WeightedTrajectoryPoint: equality") {
  auto a = make_weighted(1.0);
  a.ee_inputs.push_back(make_ee("tool", 1.0));
  auto b = a;
  CHECK(a == b);
  b.ee_inputs[0].w_force[0] += 1.0;
  CHECK_FALSE(a == b);
  b = a;
  b.q[0] += 1.0;
  CHECK_FALSE(a == b);
  // Different sizes compare unequal instead of asserting.
  CHECK_FALSE(a == make_trajectory_point(kNq + 1, kNv));
}

TEST_CASE("RtFree.find_ee_input: hit and miss") {
  auto p = make_trajectory_point(kNq, kNv);
  p.ee_inputs.push_back(make_ee("left", 1.0));
  p.ee_inputs.push_back(make_ee("right", 2.0));
  const EndEffectorInput* hit = nullptr;
  const EndEffectorInput* miss = &p.ee_inputs[0];
  const auto& cp = p;
  CHECK_RT_SAFE(hit = find_ee_input(cp, "right");
                miss = find_ee_input(cp, "head"));
  REQUIRE(hit != nullptr);
  CHECK(hit->frame_name == "right");
  CHECK(miss == nullptr);
  EndEffectorInput* writable = find_ee_input(p, "left");
  REQUIRE(writable != nullptr);
  writable->w_pose.setOnes();
  CHECK(p.ee_inputs[0].w_pose.isOnes());
}

TEST_CASE("RtFree.same_sizes: compares every vector size") {
  const auto a = make_trajectory_point(kNq, kNv);
  const auto b = make_trajectory_point(kNq, kNv);
  const auto c = make_trajectory_point(kNq, kNv + 1);
  bool same = false, different = true;
  CHECK_RT_SAFE(same = agimus_controller::same_sizes(a, b);
                different = agimus_controller::same_sizes(a, c));
  CHECK(same);
  CHECK_FALSE(different);
}

TEST_CASE("RtFree.interpolate_weights: joint weights, alpha and clamping") {
  const auto a = make_weighted(1.0);
  const auto b = make_weighted(3.0);
  auto out = make_trajectory_point(kNq, kNv);
  Status status = Status::kSizeMismatch;

  CHECK_RT_SAFE(status = interpolate_weights(a, b, 0.5, out));
  CHECK(status == Status::kOk);
  CHECK(out.w_q.isApprox(Eigen::VectorXd::Constant(kNv, 2.0)));
  CHECK(out.w_qdot.isApprox(Eigen::VectorXd::Constant(kNv, 4.0)));
  CHECK(out.w_qddot.isApprox(Eigen::VectorXd::Constant(kNv, 6.0)));
  CHECK(out.w_robot_effort.isApprox(Eigen::VectorXd::Constant(kNv, 8.0)));
  CHECK(out.w_collision_avoidance == doctest::Approx(10.0));
  CHECK(out.q.isZero());  // Only weights are written.

  CHECK_RT_SAFE(status = interpolate_weights(a, b, 0.0, out));
  CHECK(out.w_q == a.w_q);
  CHECK_RT_SAFE(status = interpolate_weights(a, b, 1.0, out));
  CHECK(out.w_q == b.w_q);
  CHECK_RT_SAFE(status = interpolate_weights(a, b, 2.0, out));  // Clamped.
  CHECK(out.w_q == b.w_q);
  CHECK_RT_SAFE(status = interpolate_weights(a, b, -1.0, out));  // Clamped.
  CHECK(out.w_q == a.w_q);
}

TEST_CASE("RtFree.interpolate_weights: end-effector union (behaviour B3)") {
  auto a = make_weighted(1.0);
  auto b = make_weighted(1.0);
  a.ee_inputs.push_back(make_ee("both", 1.0));
  a.ee_inputs.push_back(make_ee("only_a", 1.0));
  b.ee_inputs.push_back(make_ee("only_b", 1.0));
  b.ee_inputs.push_back(make_ee("both", 3.0));
  auto out = make_trajectory_point(kNq, kNv);
  out.ee_inputs.push_back(make_ee("only_out", 7.0));
  const pinocchio::SE3 out_pose = out.ee_inputs[0].pose;

  Status status = Status::kSizeMismatch;
  CHECK_RT_SAFE(status = interpolate_weights(a, b, 0.25, out));
  REQUIRE(status == Status::kOk);
  REQUIRE(out.ee_inputs.size() == 4);

  const auto* both = find_ee_input(out, "both");
  REQUIRE(both != nullptr);
  CHECK(both->w_pose.isApprox(Vector6d::Constant(0.75 * 1.0 + 0.25 * 3.0)));
  CHECK(both->w_twist.isApprox(Vector6d::Constant(0.75 * 2.0 + 0.25 * 6.0)));
  CHECK(both->w_force.isApprox(Vector6d::Constant(0.75 * 3.0 + 0.25 * 9.0)));
  // A side without the frame counts as zero weights (Python: KeyError).
  CHECK(
      find_ee_input(out, "only_a")->w_pose.isApprox(Vector6d::Constant(0.75)));
  CHECK(
      find_ee_input(out, "only_b")->w_pose.isApprox(Vector6d::Constant(0.25)));
  // Appended frames get an identity pose; frames in neither input get zero
  // weights and keep their pose.
  CHECK(find_ee_input(out, "only_a")->pose.isIdentity());
  const auto* only_out = find_ee_input(out, "only_out");
  CHECK(only_out->w_pose.isZero());
  CHECK(only_out->w_force.isZero());
  CHECK(only_out->pose == out_pose);
}

TEST_CASE("RtFree.interpolate_weights: errors leave out unchanged") {
  const auto a = make_weighted(1.0);
  auto out = make_weighted(9.0);
  const auto out_before = out;
  Status status = Status::kOk;

  const auto wrong = make_trajectory_point(kNq, kNv + 1);
  CHECK_RT_SAFE(status = interpolate_weights(a, wrong, 0.5, out));
  CHECK(status == Status::kSizeMismatch);
  CHECK(out == out_before);

  // More than kMaxEndEffectors frames in the union.
  auto many_a = make_weighted(1.0);
  auto many_b = make_weighted(1.0);
  for (std::size_t i = 0; i < kMaxEndEffectors; ++i) {
    many_a.ee_inputs.push_back(make_ee("a" + std::to_string(i), 1.0));
    many_b.ee_inputs.push_back(make_ee("b" + std::to_string(i), 1.0));
  }
  CHECK_RT_SAFE(status = interpolate_weights(many_a, many_b, 0.5, out));
  CHECK(status == Status::kBufferFull);
  CHECK(out == out_before);
}
