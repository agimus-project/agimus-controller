#pragma once

// Fixtures of test_mpc.cpp: golden set G10a, the goal-reaching MPC of G10a
// with its start-up, and the hand-built unicycle OCP of
// agimus_controller/tests/test_mpc_unicycle.py. Defined in mpc_fixtures.cpp
// (compile RAM, R6).

#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <vector>

#include "agimus_controller/mpc.hpp"
#include "agimus_controller/ocp.hpp"
#include "agimus_controller/ocp_params.hpp"
#include "agimus_controller/trajectory_buffer.hpp"
#include "agimus_controller/trajectory_point.hpp"
#include "agimus_controller/warm_start.hpp"

namespace agimus_controller::test {

// ---- Golden set G10a (test/golden/README.md) --------------------------------

struct G10aTick {
  std::chrono::nanoseconds time{0};  // Python's, always 0 (B1)
  Eigen::VectorXd q, v;              // the state given to MPC.run
  int reference_id = -1;
  bool problem_solved = false;
  int nb_iter = 0;
  double kkt = 0.0;
  Eigen::VectorXd u0;
  Eigen::MatrixXd K0;
  Eigen::VectorXd x1;
  Eigen::VectorXd q_next, v_next;  // after MPC.integrate(state, u0)
};

struct G10a {
  OcpParams params;  // finalized
  std::chrono::nanoseconds dt_ns{0};
  RobotState initial_state;
  std::vector<WeightedTrajectoryPoint> reference;
  // Start-up: the solve without limits warm-started by WarmStartReference.
  bool startup_solved = false;
  int startup_iterations = 0;
  double startup_kkt = 0.0;
  std::vector<Eigen::VectorXd> startup_xs, startup_us;
  std::vector<G10aTick> ticks;
};

/// test/golden/g10a_mpc_goal_reaching.toml, loaded once. Throws
/// std::runtime_error on a missing key.
const G10a& g10a();

// ---- Goal-reaching MPC on the panda (panda_ocp.hpp) -------------------------

/// G10a's setup: build_goal_reaching_models on the panda, the shift warm
/// start and a sliding buffer of `capacity` points.
struct GoalReachingMpc {
  std::shared_ptr<Ocp> ocp;
  std::shared_ptr<WarmStartShiftPreviousSolution> warm_start;
  std::shared_ptr<TrajectoryBuffer> buffer;
  std::unique_ptr<Mpc> mpc;
};
GoalReachingMpc make_goal_reaching_mpc(const OcpParams& params,
                                       std::size_t capacity);

/// Point k of a G10a-like reference: the panda at the "ready" configuration,
/// the hand target moving 5 mm along y per point, id k, time k * dt.
WeightedTrajectoryPoint moving_target_point(int k, std::chrono::nanoseconds dt);

/// The start-up of the ROS node (setup_mpc_initial_guess) and of the G10a
/// generator: references of the buffer's first horizon, WarmStartReference
/// from `state`, Ocp::solve_without_limits, then
/// `warm_start.update_previous_solution`. No clear_past. Returns the
/// convergence flag. Throws std::runtime_error if the buffer has no horizon
/// or a step fails.
bool start_up(Mpc& mpc, WarmStart& warm_start, const RobotState& state);

// ---- Unicycle (test_mpc_unicycle.py) ----------------------------------------

/// OCPUnicycle(horizon, dt): horizon - 1 running crocoddyl
/// ActionModelUnicycle nodes (cost weights 1, 1) and a terminal one (100,
/// 100), all with step `dt`; no updaters.
OcpModels unicycle_models(int horizon, double dt);

/// Finalized parameters of OCPUnicycle: `n_controls` nodes of `dt`.
OcpParams unicycle_params(int n_controls, double dt);

/// Python WarmStartUnicycle. x0 = state.q. Without a previous solution: xs =
/// the q of each horizon point, us = 0. Otherwise: xs = the previous states
/// shifted by one with the last horizon q appended, us = the previous
/// controls (not shifted, as Python). Records horizon[0]->q[0] for the port
/// of the reference-trajectory callback.
class WarmStartUnicycle : public WarmStart {
 public:
  WarmStartUnicycle(int n_controls, Eigen::Index nx, Eigen::Index nu);
  Status generate(const RobotState& x0_state,
                  const std::vector<const WeightedTrajectoryPoint*>& horizon,
                  Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& xs,
                  std::vector<Eigen::VectorXd>& us) override;
  void update_previous_solution(const OcpResults& previous) override;

  double first_reference_q0() const { return first_reference_q0_; }

 private:
  OcpResults previous_;
  bool has_previous_ = false;
  double first_reference_q0_ = -1.0;
};

/// Python unicycle_integrate(q, dq, dt).
Eigen::Vector3d unicycle_integrate(const Eigen::Vector3d& q,
                                   const Eigen::Vector2d& u, double dt);

}  // namespace agimus_controller::test
