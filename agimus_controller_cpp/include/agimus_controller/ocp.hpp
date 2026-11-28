#pragma once

#include <Eigen/Core>
#include <crocoddyl/core/fwd.hpp>
#include <cstddef>
#include <memory>
#include <pinocchio/multibody/geometry.hpp>
#include <string>
#include <string_view>
#include <vector>

#include "agimus_controller/cost_updater.hpp"
#include "agimus_controller/mpc_data.hpp"
#include "agimus_controller/ocp_params.hpp"
#include "agimus_controller/status.hpp"
#include "agimus_controller/trajectory_point.hpp"

namespace crocoddyl {
class SolverAbstract;  // not in crocoddyl/core/fwd.hpp
}  // namespace crocoddyl

namespace mim_solvers {
class SolverCSQP;
}  // namespace mim_solvers

namespace agimus_controller {

/// What an OCP builder produces (design.md, "OcpModels").
struct OcpModels {
  /// crocoddyl or colmpc StateMultibody; any state in hand-built tests.
  std::shared_ptr<crocoddyl::StateAbstract> state;
  /// The object the collision residuals use; nullptr without collisions.
  std::shared_ptr<pinocchio::GeometryModel> geometry;
  /// n_controls models, one object per node (never shared between nodes).
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> running;
  std::shared_ptr<crocoddyl::ActionModelAbstract> terminal;
  /// One per running node, or empty for an OCP without references.
  std::vector<NodeUpdater> running_updaters;
  NodeUpdater terminal_updater;
  /// Debug: costs with update rules (not collision); their references at
  /// node 0 go to OcpDebugData::references.
  std::vector<std::string> reference_costs;
  /// Debug: costs whose residuals over the running nodes are published.
  std::vector<std::string> residual_costs;
  /// Visual servoing: (parent, child) frame names.
  std::vector<TransformInput> input_transforms;
  std::size_t force_dim = 0;    // force feedback: nc, else 0
  bool rolling_buffer = false;  // [agimus] rolling_buffer (B7), from T8.3
};

/// Setup: creates the solver of `problem` and applies the settings of
/// `params` (PY/ocp_base_croco.py:64-81). Today it is always
/// mim_solvers::SolverCSQP. With `params.callbacks`, the verbose callback is
/// added: it prints every iteration, so a solver with callbacks is not RT.
std::shared_ptr<crocoddyl::SolverAbstract> make_solver(
    const std::shared_ptr<crocoddyl::ShootingProblem>& problem,
    const OcpParams& params);

/// The optimal control problem solved at each MPC tick (Python OCPBaseCroco
/// and OCPCrocoGeneric). Built once from an OcpModels; the RT methods then
/// update references, solve and read results without allocating.
///
/// Warm-up (rt_safety.md, rule 7): call solve_without_limits(), or solve(),
/// once before the RT loop; the first solve allocates lazily.
class Ocp {
 public:
  /// Setup. `params` must be finalized. Throws std::invalid_argument if the
  /// models do not match the parameters (number of nodes, updaters, a node
  /// shared between two nodes, a transform index out of range, a debug cost
  /// that is not found) and while the rolling buffer is not implemented
  /// (T8.3).
  Ocp(OcpModels models, const OcpParams& params);
  ~Ocp();
  Ocp(const Ocp&) = delete;
  Ocp& operator=(const Ocp&) = delete;

  /// Applies running_updaters[i] with horizon[i] and terminal_updater with
  /// horizon.back(). kSizeMismatch (nothing written) unless
  /// horizon.size() == n_controls() + 1; otherwise the first non-kOk status
  /// of apply(), nodes before it being updated.
  /// @rt Ocp::set_reference_weighted_trajectory
  Status set_reference_weighted_trajectory(
      const std::vector<const WeightedTrajectoryPoint*>& horizon) noexcept;

  /// Solves from x0 with the warm start (xs, us), with solver_iters
  /// iterations and max_solve_time. Copies xs, us and K into results() and,
  /// with use_debug_data, fills debug_data(). Returns the solver's
  /// convergence flag; false without solving if a size is wrong.
  /// @rt Ocp::solve
  bool solve(const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& xs,
             const std::vector<Eigen::VectorXd>& us) noexcept;

  /// solve() with 1000 iterations and no timeout: the initial solve. Setup.
  bool solve_without_limits(const Eigen::VectorXd& x0,
                            const std::vector<Eigen::VectorXd>& xs,
                            const std::vector<Eigen::VectorXd>& us);

  /// One step of running node 0 from (x, u): calc on its model and data,
  /// then x_next = xnext. `x_next` must have size nx().
  /// @rt Ocp::integrate
  void integrate(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                 Eigen::VectorXd& x_next) noexcept;

  const OcpResults& results() const { return results_; }
  const OcpDebugData& debug_data() const { return debug_data_; }
  int n_controls() const { return params_.n_controls; }
  double dt() const { return params_.dt; }
  /// The finalized parameters given at setup (Mpc reads timesteps[0] and
  /// use_debug_data).
  const OcpParams& params() const { return params_; }
  Eigen::Index nx() const;
  Eigen::Index nu() const;
  const std::shared_ptr<crocoddyl::ShootingProblem>& problem() const {
    return problem_;
  }
  const std::shared_ptr<crocoddyl::SolverAbstract>& solver() const {
    return solver_;
  }

  /// Setup: index of a collision geometry. Throws std::invalid_argument for
  /// an unknown name, for a model without geometry, and for VISUAL (the OCP
  /// holds no visual model).
  pinocchio::GeomIndex geometry_index(std::string_view name,
                                      pinocchio::GeometryType type) const;
  /// Sets the placement of a collision geometry of OcpModels::geometry
  /// (Python update_geometry_placement). `index` comes from
  /// geometry_index(); only COLLISION is valid.
  /// @rt Ocp::set_geometry_placement
  void set_geometry_placement(pinocchio::GeomIndex index,
                              const pinocchio::SE3& placement,
                              pinocchio::GeometryType type) noexcept;

  /// Setup: the (parent, child) pairs, in the order of the indexes below.
  const std::vector<TransformInput>& input_transforms() const {
    return input_transforms_;
  }
  /// @rt Ocp::set_input_transform
  void set_input_transform(std::size_t i,
                           const pinocchio::SE3& transform) noexcept;
  /// Python None: visual servoing then needs zero weights.
  /// @rt Ocp::clear_input_transform
  void clear_input_transform(std::size_t i) noexcept;

 private:
  bool solve_impl(const Eigen::VectorXd& x0,
                  const std::vector<Eigen::VectorXd>& xs,
                  const std::vector<Eigen::VectorXd>& us, std::size_t max_iter,
                  double max_solve_time) noexcept;
  void fill_debug_data(bool solved) noexcept;

  OcpParams params_;
  std::shared_ptr<crocoddyl::StateAbstract> state_;
  std::shared_ptr<pinocchio::GeometryModel> geometry_;
  std::vector<NodeUpdater> running_updaters_;
  NodeUpdater terminal_updater_;
  std::vector<TransformInput> input_transforms_;
  InputTransforms transforms_;
  std::shared_ptr<crocoddyl::ShootingProblem> problem_;
  std::shared_ptr<crocoddyl::SolverAbstract> solver_;
  mim_solvers::SolverCSQP* csqp_ = nullptr;  // solver_, for CSQP-only reads
  const std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                  Eigen::RowMajor>>* gains_ = nullptr;  // K
  OcpResults results_;
  OcpDebugData debug_data_;
  // Node 0's updater of each reference cost (points into running_updaters_).
  std::vector<const CostUpdater*> debug_references_;
  // residual_vectors_[i][node]: residual of residual_costs[i] at a node.
  std::vector<std::vector<const Eigen::VectorXd*>> residual_vectors_;
};

}  // namespace agimus_controller
