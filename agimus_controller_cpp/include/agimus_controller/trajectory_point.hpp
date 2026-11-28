#pragma once

#include <Eigen/Core>
#include <chrono>
#include <string_view>

#if __has_include(<pinocchio/spatial.hpp>)
#include <pinocchio/spatial.hpp>  // pinocchio >= 4
#else
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>
#endif

#include "agimus_controller/fixed_capacity.hpp"
#include "agimus_controller/status.hpp"

namespace agimus_controller {

inline constexpr std::size_t kMaxEndEffectors = 8;
inline constexpr std::size_t kMaxFrameNameLength = 63;
using FrameName = FixedString<kMaxFrameNameLength>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

/// Target of one end-effector frame. Mirrors agimus_msgs/MpcEEInput.
struct EndEffectorInput {
  FrameName frame_name;  // frame_id
  pinocchio::SE3 pose = pinocchio::SE3::Identity();
  Vector6d w_pose = Vector6d::Zero();
  pinocchio::Motion twist = pinocchio::Motion::Zero();
  Vector6d w_twist = Vector6d::Zero();
  pinocchio::Force force = pinocchio::Force::Zero();
  Vector6d w_force = Vector6d::Zero();

  friend bool operator==(const EndEffectorInput& a, const EndEffectorInput& b);
};

using EndEffectorInputs = FixedVector<EndEffectorInput, kMaxEndEffectors>;

/// Reference point of the MPC with its cost weights. Mirrors
/// agimus_msgs/MpcInput. Create it with make_trajectory_point() so that every
/// vector is sized.
struct WeightedTrajectoryPoint {
  int id = -1;
  std::chrono::nanoseconds time{0};
  Eigen::VectorXd q;               // nq
  Eigen::VectorXd w_q;             // nv (tangent space, like the residual)
  Eigen::VectorXd qdot, w_qdot;    // nv
  Eigen::VectorXd qddot, w_qddot;  // nv
  Eigen::VectorXd robot_effort, w_robot_effort;  // nv
  double w_collision_avoidance = 0.0;
  EndEffectorInputs ee_inputs;

  friend bool operator==(const WeightedTrajectoryPoint& a,
                         const WeightedTrajectoryPoint& b);
};

/// Measured robot state given to the MPC.
struct RobotState {
  std::chrono::nanoseconds time{0};
  Eigen::VectorXd q;  // nq
  Eigen::VectorXd v;  // nv
  Eigen::VectorXd a;  // nv, zero by default as in the ROS node
  Eigen::Vector3d contact_force = Eigen::Vector3d::Zero();  // force feedback

  friend bool operator==(const RobotState& a, const RobotState& b);
};

/// Setup: a point with every vector sized and zero, and no end effector.
WeightedTrajectoryPoint make_trajectory_point(Eigen::Index nq, Eigen::Index nv);

/// Setup: a state with every vector sized and zero.
RobotState make_robot_state(Eigen::Index nq, Eigen::Index nv);

/// True if every vector of `a` has the size of the same vector of `b`.
/// @rt same_sizes
bool same_sizes(const WeightedTrajectoryPoint& a,
                const WeightedTrajectoryPoint& b) noexcept;

/// Linear search by frame name; nullptr if absent.
/// @rt find_ee_input
const EndEffectorInput* find_ee_input(const WeightedTrajectoryPoint& point,
                                      std::string_view frame_name) noexcept;
EndEffectorInput* find_ee_input(WeightedTrajectoryPoint& point,
                                std::string_view frame_name) noexcept;

/// Interpolates the weights of `a` and `b` into `out`:
/// out.w = (1 - alpha) * a.w + alpha * b.w, with alpha clamped to [0, 1].
/// Only weights are written. End effectors are the union by name of `a` and
/// `b`; a frame missing on one side has zero weights there. A union frame
/// missing from `out` is appended (identity pose); an `out` frame in neither
/// input gets zero weights.
/// Returns kSizeMismatch if the weight vectors differ in size, kBufferFull if
/// the union holds more than kMaxEndEffectors frames.
/// @rt interpolate_weights
Status interpolate_weights(const WeightedTrajectoryPoint& a,
                           const WeightedTrajectoryPoint& b, double alpha,
                           WeightedTrajectoryPoint& out) noexcept;

}  // namespace agimus_controller
