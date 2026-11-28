#pragma once

#include <Eigen/Core>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <memory>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <vector>

#include "agimus_controller/mpc_data.hpp"
#include "agimus_controller/ocp_params.hpp"
#include "agimus_controller/robot_models.hpp"
#include "agimus_controller/status.hpp"
#include "agimus_controller/trajectory_point.hpp"

namespace agimus_controller {

/// Initial guess of the OCP (Python WarmStartBase). `generate` is real-time:
/// it writes into `x0`, `xs` and `us`, which the caller sized at setup, and
/// never allocates. Each override carries its own @rt tag.
class WarmStart {
 public:
  virtual ~WarmStart() = default;
  virtual Status generate(
      const RobotState& x0_state,
      const std::vector<const WeightedTrajectoryPoint*>& horizon,
      Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& xs,
      std::vector<Eigen::VectorXd>& us) = 0;
  /// Real-time. Default: ignores the solution (only the shift warm start
  /// uses it).
  virtual void update_previous_solution(const OcpResults& previous);
};

/// Port of PY/warm_start_reference.py: the reference trajectory, with the
/// controls from inverse dynamics (RNEA).
class WarmStartReference : public WarmStart {
 public:
  /// Setup: copies the model and creates its data.
  explicit WarmStartReference(const pinocchio::Model& model);

  /// xs[0] = x0 = [x0_state.q, x0_state.v]; xs[i] = [q, qdot] of horizon[i];
  /// us[0] = rnea(x0_state.q, v, a); us[i] = rnea(horizon[i]) for
  /// 1 <= i < n - 1, with n = horizon.size(). `horizon[0]` and the
  /// acceleration of the last point are unused.
  /// Returns kSizeMismatch (and writes nothing) unless the state vectors have
  /// size nq/nv, the horizon holds n >= 1 non-null points of the right
  /// sizes, `x0` has size nq + nv, `xs` holds n states and `us` n - 1
  /// controls (sizes nq + nv and nv).
  /// @rt WarmStartReference::generate
  Status generate(const RobotState& x0_state,
                  const std::vector<const WeightedTrajectoryPoint*>& horizon,
                  Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& xs,
                  std::vector<Eigen::VectorXd>& us) override;

 private:
  pinocchio::Model model_;
  pinocchio::Data data_;
};

/// Port of PY/warm_start_shift_previous_solution.py: shifts the previous
/// solution by one dt. A step longer than dt is advanced by one dt with the
/// integrator, keeping its control.
///
/// Behaviour change A13 (B18): after the loop, if the last step is longer
/// than dt, the terminal state xs[N] is advanced by one dt step with
/// us.back() as well. Python leaves it unshifted.
class WarmStartShiftPreviousSolution : public WarmStart {
 public:
  /// Setup: builds the integrator and sizes the copy of the previous
  /// solution, then calls the integrator once (warm-up). `params` must be
  /// finalized. Throws std::invalid_argument unless
  /// params.dt == params.timesteps[0] and every step is >= dt.
  WarmStartShiftPreviousSolution(const RobotModels& robot_models,
                                 const OcpParams& params);
  ~WarmStartShiftPreviousSolution() override;

  /// Copies `previous` into the own storage. A solution of other sizes is
  /// ignored (the previous one, if any, is kept): the setup is wrong, and
  /// this function cannot report it.
  /// @rt WarmStartShiftPreviousSolution::update_previous_solution
  void update_previous_solution(const OcpResults& previous) override;

  /// Shifts the own copy of the previous solution in place, then copies it
  /// into `xs` and `us` and writes `x0 = [x0_state.q, x0_state.v]`. The
  /// horizon is ignored, as in Python.
  /// Returns kNoPreviousSolution before the first update_previous_solution,
  /// and kSizeMismatch (before shifting) unless the state vectors have size
  /// nq/nv, `x0` has size nx, `xs` holds n_controls + 1 states of size nx
  /// and `us` n_controls controls of size nv.
  /// @rt WarmStartShiftPreviousSolution::generate
  Status generate(const RobotState& x0_state,
                  const std::vector<const WeightedTrajectoryPoint*>& horizon,
                  Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& xs,
                  std::vector<Eigen::VectorXd>& us) override;

 private:
  void shift();

  std::vector<double> timesteps_;
  double dt_;
  std::shared_ptr<crocoddyl::StateMultibody> state_;
  std::shared_ptr<crocoddyl::ActionModelAbstract> integrator_;
  std::shared_ptr<crocoddyl::ActionDataAbstract> integrator_data_;
  OcpResults previous_;
  bool has_previous_ = false;
};

}  // namespace agimus_controller
