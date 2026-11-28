#pragma once

#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <vector>

#include "agimus_controller/mpc_data.hpp"
#include "agimus_controller/ocp.hpp"
#include "agimus_controller/status.hpp"
#include "agimus_controller/trajectory_buffer.hpp"
#include "agimus_controller/trajectory_point.hpp"
#include "agimus_controller/warm_start.hpp"

namespace agimus_controller {

/// The MPC loop (Python MPC, PY/mpc.py): at each tick, the OCP is solved over
/// the horizon of the buffer from the measured state, warm-started by the
/// warm start, which then receives the solution.
///
/// Start-up (setup, as the ROS node's setup_mpc_initial_guess): once the
/// buffer holds a horizon, set the OCP's references, solve with
/// Ocp::solve_without_limits from WarmStartReference's guess, and seed the
/// warm start with WarmStart::update_previous_solution. This first solve is
/// also the warm-up of the RT path (rt_safety.md, rule 7).
class Mpc {
 public:
  /// Setup: sizes the warm-start storage from the OCP. Throws
  /// std::invalid_argument for a null pointer, or if the buffer's horizon
  /// does not have n_controls + 1 nodes.
  Mpc(std::shared_ptr<Ocp> ocp, std::shared_ptr<WarmStart> warm_start,
      std::shared_ptr<TrajectoryBuffer> buffer);

  /// One MPC tick (PY/mpc.py:32-66): sets the references of the buffer's
  /// horizon, generates the warm start, solves, hands the solution to the
  /// warm start and drops the oldest buffer point. Results are read with
  /// ocp().results(); a solver that does not converge is not an error (see
  /// debug_data().ocp.problem_solved).
  ///
  /// Returns, without changing anything:
  /// - kNotEnoughData unless buffer().has_horizon() (B2);
  /// - the status of Ocp::set_reference_weighted_trajectory or of
  ///   WarmStart::generate when not kOk (e.g. kNoPreviousSolution before the
  ///   start-up). The references may then be partly updated, as documented
  ///   by Ocp.
  ///
  /// With OcpParams::use_debug_data, fills debug_data() and its phase timers
  /// (std::chrono::steady_clock). Without it, no debug work and no clock
  /// read.
  ///
  /// `now` is unused, as Python's current_time_ns (audit A3); it is kept for
  /// a later wall-time playback of the trajectory.
  /// @rt Mpc::run
  Status run(const RobotState& state, std::chrono::nanoseconds now) noexcept;

  /// Advances `state` by one step of running node 0 from control `u`
  /// (Ocp::integrate), writing q and v in place, and advances `state.time` by
  /// that step, OcpParams::timesteps[0], rounded to nanoseconds (B1, A12).
  /// `state.a` is unchanged. Returns kSizeMismatch, and changes nothing,
  /// unless q has the state's nq, q and v together have its nx, and u has
  /// nu entries.
  /// @rt Mpc::integrate
  Status integrate(RobotState& state, const Eigen::VectorXd& u) noexcept;

  /// Appends a reference point to the buffer (TrajectoryBuffer::push_back).
  /// @rt Mpc::push_back
  Status push_back(const WeightedTrajectoryPoint& point) noexcept;

  const MpcDebugData& debug_data() const { return debug_data_; }
  const Ocp& ocp() const { return *ocp_; }
  Ocp& ocp() { return *ocp_; }
  const TrajectoryBuffer& buffer() const { return *buffer_; }
  TrajectoryBuffer& buffer() { return *buffer_; }

 private:
  std::shared_ptr<Ocp> ocp_;
  std::shared_ptr<WarmStart> warm_start_;
  std::shared_ptr<TrajectoryBuffer> buffer_;
  bool use_debug_data_;
  std::chrono::nanoseconds step_;  // timesteps[0], converted at setup
  Eigen::Index nq_;                // configuration size of the OCP state
  // Warm start, passed to Ocp::solve.
  Eigen::VectorXd x0_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;
  // integrate(): [q, v] and the next state.
  Eigen::VectorXd x_;
  Eigen::VectorXd x_next_;
  MpcDebugData debug_data_;
};

}  // namespace agimus_controller
