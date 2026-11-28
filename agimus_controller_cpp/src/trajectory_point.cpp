#include "agimus_controller/trajectory_point.hpp"

#include <algorithm>
#include <cassert>

namespace agimus_controller {

namespace {

bool equal(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  return a.size() == b.size() && a == b;
}

bool same_weight_sizes(const WeightedTrajectoryPoint& a,
                       const WeightedTrajectoryPoint& b) noexcept {
  return a.w_q.size() == b.w_q.size() && a.w_qdot.size() == b.w_qdot.size() &&
         a.w_qddot.size() == b.w_qddot.size() &&
         a.w_robot_effort.size() == b.w_robot_effort.size();
}

template <typename Point>
auto find_ee(Point& point, std::string_view frame_name) noexcept
    -> decltype(&point.ee_inputs[0]) {
  for (auto& ee : point.ee_inputs) {
    if (ee.frame_name == frame_name) {
      return &ee;
    }
  }
  return nullptr;
}

}  // namespace

bool operator==(const EndEffectorInput& a, const EndEffectorInput& b) {
  return a.frame_name == b.frame_name && a.pose == b.pose &&
         a.w_pose == b.w_pose && a.twist == b.twist && a.w_twist == b.w_twist &&
         a.force == b.force && a.w_force == b.w_force;
}

bool operator==(const WeightedTrajectoryPoint& a,
                const WeightedTrajectoryPoint& b) {
  return a.id == b.id && a.time == b.time && equal(a.q, b.q) &&
         equal(a.w_q, b.w_q) && equal(a.qdot, b.qdot) &&
         equal(a.w_qdot, b.w_qdot) && equal(a.qddot, b.qddot) &&
         equal(a.w_qddot, b.w_qddot) && equal(a.robot_effort, b.robot_effort) &&
         equal(a.w_robot_effort, b.w_robot_effort) &&
         a.w_collision_avoidance == b.w_collision_avoidance &&
         a.ee_inputs == b.ee_inputs;
}

bool operator==(const RobotState& a, const RobotState& b) {
  return a.time == b.time && equal(a.q, b.q) && equal(a.v, b.v) &&
         equal(a.a, b.a) && a.contact_force == b.contact_force;
}

WeightedTrajectoryPoint make_trajectory_point(Eigen::Index nq,
                                              Eigen::Index nv) {
  WeightedTrajectoryPoint p;
  p.q = Eigen::VectorXd::Zero(nq);
  p.w_q = Eigen::VectorXd::Zero(nv);
  p.qdot = Eigen::VectorXd::Zero(nv);
  p.w_qdot = Eigen::VectorXd::Zero(nv);
  p.qddot = Eigen::VectorXd::Zero(nv);
  p.w_qddot = Eigen::VectorXd::Zero(nv);
  p.robot_effort = Eigen::VectorXd::Zero(nv);
  p.w_robot_effort = Eigen::VectorXd::Zero(nv);
  return p;
}

RobotState make_robot_state(Eigen::Index nq, Eigen::Index nv) {
  RobotState s;
  s.q = Eigen::VectorXd::Zero(nq);
  s.v = Eigen::VectorXd::Zero(nv);
  s.a = Eigen::VectorXd::Zero(nv);
  return s;
}

bool same_sizes(const WeightedTrajectoryPoint& a,
                const WeightedTrajectoryPoint& b) noexcept {
  return a.q.size() == b.q.size() && a.qdot.size() == b.qdot.size() &&
         a.qddot.size() == b.qddot.size() &&
         a.robot_effort.size() == b.robot_effort.size() &&
         same_weight_sizes(a, b);
}

const EndEffectorInput* find_ee_input(const WeightedTrajectoryPoint& point,
                                      std::string_view frame_name) noexcept {
  return find_ee(point, frame_name);
}

EndEffectorInput* find_ee_input(WeightedTrajectoryPoint& point,
                                std::string_view frame_name) noexcept {
  return find_ee(point, frame_name);
}

Status interpolate_weights(const WeightedTrajectoryPoint& a,
                           const WeightedTrajectoryPoint& b, double alpha,
                           WeightedTrajectoryPoint& out) noexcept {
  assert(&out != &a && &out != &b && "out must not alias an input");
  if (!same_weight_sizes(a, b) || !same_weight_sizes(a, out)) {
    return Status::kSizeMismatch;
  }

  // Count the frames `out` must hold before touching it, so that a failure
  // leaves `out` unchanged.
  std::size_t n_out = out.ee_inputs.size();
  for (const auto& ea : a.ee_inputs) {
    n_out += find_ee_input(out, ea.frame_name.view()) == nullptr;
  }
  for (const auto& eb : b.ee_inputs) {
    const auto name = eb.frame_name.view();
    n_out += find_ee_input(a, name) == nullptr &&
             find_ee_input(out, name) == nullptr;
  }
  if (n_out > kMaxEndEffectors) {
    return Status::kBufferFull;
  }

  alpha = std::clamp(alpha, 0.0, 1.0);
  const double wa = 1.0 - alpha;
  const double wb = alpha;
  out.w_q = wa * a.w_q + wb * b.w_q;
  out.w_qdot = wa * a.w_qdot + wb * b.w_qdot;
  out.w_qddot = wa * a.w_qddot + wb * b.w_qddot;
  out.w_robot_effort = wa * a.w_robot_effort + wb * b.w_robot_effort;
  out.w_collision_avoidance =
      wa * a.w_collision_avoidance + wb * b.w_collision_avoidance;

  for (auto& eo : out.ee_inputs) {
    eo.w_pose.setZero();
    eo.w_twist.setZero();
    eo.w_force.setZero();
  }
  auto target = [&out](const FrameName& name) -> EndEffectorInput& {
    if (EndEffectorInput* e = find_ee_input(out, name.view())) {
      return *e;
    }
    EndEffectorInput fresh;
    fresh.frame_name = name;
    out.ee_inputs.push_back(fresh);  // Capacity checked above.
    return out.ee_inputs[out.ee_inputs.size() - 1];
  };
  // A missing side (nullptr) counts as zero weights (Python raised KeyError).
  auto blend = [wa, wb](const EndEffectorInput* ea, const EndEffectorInput* eb,
                        EndEffectorInput& eo) {
    eo.w_pose.setZero();
    eo.w_twist.setZero();
    eo.w_force.setZero();
    if (ea != nullptr) {
      eo.w_pose += wa * ea->w_pose;
      eo.w_twist += wa * ea->w_twist;
      eo.w_force += wa * ea->w_force;
    }
    if (eb != nullptr) {
      eo.w_pose += wb * eb->w_pose;
      eo.w_twist += wb * eb->w_twist;
      eo.w_force += wb * eb->w_force;
    }
  };
  for (const auto& ea : a.ee_inputs) {
    blend(&ea, find_ee_input(b, ea.frame_name.view()), target(ea.frame_name));
  }
  for (const auto& eb : b.ee_inputs) {
    if (find_ee_input(a, eb.frame_name.view()) == nullptr) {
      blend(nullptr, &eb, target(eb.frame_name));
    }
  }
  return Status::kOk;
}

}  // namespace agimus_controller
