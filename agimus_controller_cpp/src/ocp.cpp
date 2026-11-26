#include "agimus_controller/ocp.hpp"

#include <cassert>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/state-base.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <limits>
#include <mim_solvers/csqp.hpp>
#include <mim_solvers/utils/callbacks.hpp>
#include <set>
#include <stdexcept>

namespace agimus_controller {

namespace {

constexpr std::size_t kIterationsWithoutLimits = 1000;  // Python solve()

const CostUpdater* find_updater(const NodeUpdater& node,
                                const std::string& cost) {
  for (const CostUpdater& u : node.costs) {
    if (u.item->name == cost) {
      return &u;
    }
  }
  return nullptr;
}

// Rows and columns of a debug reference (design.md, Parameters and results).
std::pair<Eigen::Index, Eigen::Index> reference_shape(const CostUpdater& u) {
  switch (u.rule) {
    case UpdateRule::kState:
      return {static_cast<Eigen::Index>(u.residual->get_state()->get_nx()), 1};
    case UpdateRule::kControl:
      return {static_cast<Eigen::Index>(u.residual->get_nu()), 1};
    case UpdateRule::kControlGrav:
      return {0, 1};  // B4: no reference.
    case UpdateRule::kFramePlacement:
    case UpdateRule::kVisualServoing:
      return {7, 1};  // XYZQUAT
    case UpdateRule::kFrameTranslation:
      return {3, 1};
    case UpdateRule::kFrameRotation:
      return {3, 3};
    case UpdateRule::kFrameVelocity:
      return {6, 1};
    default:
      throw std::invalid_argument("unsupported reference");
  }
}

void write_reference(const CostUpdater& u, Eigen::MatrixXd& out) noexcept {
  switch (u.rule) {
    case UpdateRule::kState:
      out.col(0) = static_cast<const crocoddyl::ResidualModelState*>(u.residual)
                       ->get_reference();
      return;
    case UpdateRule::kControl:
      out.col(0) =
          static_cast<const crocoddyl::ResidualModelControl*>(u.residual)
              ->get_reference();
      return;
    case UpdateRule::kFramePlacement:
    case UpdateRule::kVisualServoing: {
      const pinocchio::SE3& m =
          static_cast<const crocoddyl::ResidualModelFramePlacement*>(u.residual)
              ->get_reference();
      const Eigen::Quaterniond q(m.rotation());
      out.col(0).head<3>() = m.translation();
      out.col(0).tail<4>() = q.coeffs();  // x, y, z, w as SE3ToXYZQUAT
      return;
    }
    case UpdateRule::kFrameTranslation:
      out.col(0) = static_cast<const crocoddyl::ResidualModelFrameTranslation*>(
                       u.residual)
                       ->get_reference();
      return;
    case UpdateRule::kFrameRotation:
      out =
          static_cast<const crocoddyl::ResidualModelFrameRotation*>(u.residual)
              ->get_reference();
      return;
    case UpdateRule::kFrameVelocity:
      out.col(0) =
          static_cast<const crocoddyl::ResidualModelFrameVelocity*>(u.residual)
              ->get_reference()
              .toVector();
      return;
    default:
      return;
  }
}

// The residual vector of cost `name` in the data of a running node. Setup.
const Eigen::VectorXd* residual_vector(
    const std::shared_ptr<crocoddyl::ActionDataAbstract>& data,
    const std::string& name) {
  const auto* euler =
      dynamic_cast<const crocoddyl::IntegratedActionDataEuler*>(data.get());
  const auto* dam =
      euler == nullptr
          ? nullptr
          : dynamic_cast<
                const crocoddyl::DifferentialActionDataFreeFwdDynamics*>(
                euler->differential.get());
  if (dam == nullptr) {
    throw std::invalid_argument(
        "Ocp: residual_costs: '" + name +
        "' needs Euler nodes over a free forward dynamics model.");
  }
  const auto it = dam->costs->costs.find(name);
  if (it == dam->costs->costs.end()) {
    throw std::invalid_argument("Ocp: residual_costs: no cost '" + name +
                                "' in a running node.");
  }
  return &it->second->residual->r;
}

void check_transform_indexes(const NodeUpdater& node, std::size_t n) {
  for (const CostUpdater& u : node.costs) {
    if (u.rule == UpdateRule::kVisualServoing && u.transform_index >= n) {
      throw std::invalid_argument(
          "Ocp: cost '" + u.item->name + "' uses input transform " +
          std::to_string(u.transform_index) + ", but there are only " +
          std::to_string(n) + ".");
    }
  }
}

}  // namespace

std::shared_ptr<crocoddyl::SolverAbstract> make_solver(
    const std::shared_ptr<crocoddyl::ShootingProblem>& problem,
    const OcpParams& params) {
  auto solver = std::make_shared<mim_solvers::SolverCSQP>(problem);
  solver->set_use_filter_line_search(params.use_filter_line_search);
  if (params.max_solve_time.has_value()) {
    solver->set_max_solve_time(*params.max_solve_time);
  }
  solver->set_termination_tolerance(params.termination_tolerance);
  solver->set_max_qp_iters(params.qp_iters);
  solver->set_eps_abs(params.eps_abs);
  solver->set_eps_rel(params.eps_rel);
  if (params.callbacks) {
    // Python also adds CallbackLogger, which exists only in mim_solvers'
    // Python bindings.
    solver->setCallbacks({std::make_shared<mim_solvers::CallbackVerbose>()});
  }
  return solver;
}

Ocp::Ocp(OcpModels models, const OcpParams& params) : params_(params) {
  const std::size_t n = static_cast<std::size_t>(params_.n_controls);
  if (params_.n_controls <= 0 || params_.timesteps.size() != n) {
    throw std::invalid_argument(
        "Ocp: OcpParams is not finalized (call finalize()).");
  }
  if (models.rolling_buffer) {
    throw std::invalid_argument(
        "Ocp: rolling_buffer is not implemented until T8.3.");
  }
  if (!models.state || !models.terminal) {
    throw std::invalid_argument(
        "Ocp: OcpModels::state and terminal are "
        "required.");
  }
  if (models.running.size() != n) {
    throw std::invalid_argument(
        "Ocp: OcpModels::running has " + std::to_string(models.running.size()) +
        " nodes, OcpParams::n_controls is " + std::to_string(n) + ".");
  }
  std::set<const crocoddyl::ActionModelAbstract*> nodes{models.terminal.get()};
  for (const auto& model : models.running) {
    if (!model || !nodes.insert(model.get()).second) {
      throw std::invalid_argument(
          "Ocp: every node needs its own action model (null or shared "
          "model in OcpModels::running).");
    }
  }
  if (!models.running_updaters.empty() && models.running_updaters.size() != n) {
    throw std::invalid_argument(
        "Ocp: OcpModels::running_updaters must be empty or hold one entry "
        "per running node.");
  }
  const std::size_t n_transforms = models.input_transforms.size();
  for (const NodeUpdater& node : models.running_updaters) {
    check_transform_indexes(node, n_transforms);
  }
  check_transform_indexes(models.terminal_updater, n_transforms);

  state_ = std::move(models.state);
  geometry_ = std::move(models.geometry);
  running_updaters_ = std::move(models.running_updaters);
  terminal_updater_ = std::move(models.terminal_updater);
  input_transforms_ = std::move(models.input_transforms);
  transforms_.assign(n_transforms, std::nullopt);

  const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(state_->get_nx() + models.force_dim));
  problem_ = std::make_shared<crocoddyl::ShootingProblem>(x0, models.running,
                                                          models.terminal);
  problem_->set_nthreads(params_.n_threads);
  solver_ = make_solver(problem_, params_);
  csqp_ = dynamic_cast<mim_solvers::SolverCSQP*>(solver_.get());
  if (csqp_ == nullptr) {
    throw std::invalid_argument("Ocp: make_solver did not return CSQP.");
  }
  gains_ = &csqp_->get_K();

  const auto nx_ = static_cast<Eigen::Index>(problem_->get_nx());
  const auto ndx_ = static_cast<Eigen::Index>(problem_->get_ndx());
  const auto nu_ = nu();
  results_.resize(n, nx_, ndx_, nu_);
  debug_data_.result.resize(n, nx_, ndx_, nu_);

  for (const std::string& name : models.reference_costs) {
    const CostUpdater* u = running_updaters_.empty()
                               ? nullptr
                               : find_updater(running_updaters_[0], name);
    if (u == nullptr || u->rule == UpdateRule::kCollisionWeight) {
      throw std::invalid_argument(
          "Ocp: reference_costs: '" + name +
          "' has no reference updater in running node 0.");
    }
    const auto [rows, cols] = reference_shape(*u);
    debug_references_.push_back(u);
    debug_data_.references.push_back({name, Eigen::MatrixXd::Zero(rows, cols)});
  }
  const auto& datas = problem_->get_runningDatas();
  for (const std::string& name : models.residual_costs) {
    std::vector<const Eigen::VectorXd*> vectors;
    for (const auto& data : datas) {
      vectors.push_back(residual_vector(data, name));
    }
    debug_data_.residuals.push_back(
        {name, Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(n),
                                     vectors.front()->size())});
    residual_vectors_.push_back(std::move(vectors));
  }
}

Ocp::~Ocp() = default;

Eigen::Index Ocp::nx() const {
  return static_cast<Eigen::Index>(problem_->get_nx());
}

Eigen::Index Ocp::nu() const {
  return static_cast<Eigen::Index>(problem_->get_runningModels()[0]->get_nu());
}

Status Ocp::set_reference_weighted_trajectory(
    const std::vector<const WeightedTrajectoryPoint*>& horizon) noexcept {
  const std::size_t n = static_cast<std::size_t>(params_.n_controls);
  if (horizon.size() != n + 1) {
    return Status::kSizeMismatch;
  }
  for (std::size_t i = 0; i < running_updaters_.size(); ++i) {
    const Status status = apply(running_updaters_[i], *horizon[i], transforms_);
    if (status != Status::kOk) {
      return status;
    }
  }
  return apply(terminal_updater_, *horizon.back(), transforms_);
}

bool Ocp::solve(const Eigen::VectorXd& x0,
                const std::vector<Eigen::VectorXd>& xs,
                const std::vector<Eigen::VectorXd>& us) noexcept {
  return solve_impl(x0, xs, us, static_cast<std::size_t>(params_.solver_iters),
                    params_.max_solve_time.value_or(0.0));
}

bool Ocp::solve_without_limits(const Eigen::VectorXd& x0,
                               const std::vector<Eigen::VectorXd>& xs,
                               const std::vector<Eigen::VectorXd>& us) {
  return solve_impl(x0, xs, us, kIterationsWithoutLimits,
                    std::numeric_limits<double>::infinity());
}

bool Ocp::solve_impl(const Eigen::VectorXd& x0,
                     const std::vector<Eigen::VectorXd>& xs,
                     const std::vector<Eigen::VectorXd>& us,
                     std::size_t max_iter, double max_solve_time) noexcept {
  // crocoddyl throws on a wrong size; refuse instead.
  const Eigen::Index nx_ = nx();
  const std::size_t n = static_cast<std::size_t>(params_.n_controls);
  if (x0.size() != nx_ || xs.size() != n + 1 || us.size() != n) {
    return false;
  }
  const auto& models = problem_->get_runningModels();
  for (std::size_t i = 0; i < n; ++i) {
    if (xs[i].size() != nx_ ||
        us[i].size() != static_cast<Eigen::Index>(models[i]->get_nu())) {
      return false;
    }
  }
  if (xs[n].size() != nx_) {
    return false;
  }

  problem_->set_x0(x0);
  // As Python: max_solve_time is touched only when the parameters set it.
  if (params_.max_solve_time.has_value()) {
    csqp_->set_max_solve_time(max_solve_time);
  }
  const bool solved = solver_->solve(xs, us, max_iter);

  const auto& sxs = solver_->get_xs();
  const auto& sus = solver_->get_us();
  const auto& gains = *gains_;
  for (std::size_t i = 0; i < n; ++i) {
    results_.states[i] = sxs[i];
    results_.feed_forward_terms[i] = sus[i];
    results_.riccati_gains[i] = gains[i];
  }
  results_.states[n] = sxs[n];

  if (params_.use_debug_data) {
    fill_debug_data(solved);
  }
  return solved;
}

void Ocp::fill_debug_data(bool solved) noexcept {
  debug_data_.problem_solved = solved;
  copy_results(results_, debug_data_.result);
  debug_data_.kkt_norm = csqp_->get_KKT();
  debug_data_.nb_iter = static_cast<int>(solver_->get_iter());
  debug_data_.nb_qp_iter = static_cast<int>(csqp_->get_qp_iters());
  // References from node 0 only, as Python.
  for (std::size_t i = 0; i < debug_references_.size(); ++i) {
    write_reference(*debug_references_[i], debug_data_.references[i].data);
  }
  for (std::size_t i = 0; i < residual_vectors_.size(); ++i) {
    Eigen::MatrixXd& out = debug_data_.residuals[i].data;
    const auto& vectors = residual_vectors_[i];
    for (std::size_t node = 0; node < vectors.size(); ++node) {
      out.row(static_cast<Eigen::Index>(node)) = vectors[node]->transpose();
    }
  }
}

void Ocp::integrate(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x_next) noexcept {
  const auto& model = problem_->get_runningModels()[0];
  const auto& data = problem_->get_runningDatas()[0];
  assert(x.size() == nx() && x_next.size() == nx());
  assert(u.size() == static_cast<Eigen::Index>(model->get_nu()));
  model->calc(data, x, u);
  x_next = data->xnext;
}

pinocchio::GeomIndex Ocp::geometry_index(std::string_view name,
                                         pinocchio::GeometryType type) const {
  if (type != pinocchio::COLLISION) {
    throw std::invalid_argument(
        "Ocp::geometry_index: only COLLISION geometries are in the OCP ('" +
        std::string(name) + "').");
  }
  if (!geometry_) {
    throw std::invalid_argument(
        "Ocp::geometry_index: the OCP has no "
        "geometry model ('" +
        std::string(name) + "').");
  }
  const std::string key(name);
  if (!geometry_->existGeometryName(key)) {
    throw std::invalid_argument("Ocp::geometry_index: unknown geometry name '" +
                                key + "' in collision model.");
  }
  return geometry_->getGeometryId(key);
}

void Ocp::set_geometry_placement(pinocchio::GeomIndex index,
                                 const pinocchio::SE3& placement,
                                 pinocchio::GeometryType type) noexcept {
  // VISUAL: Python updates RobotModels' visual model, which the OCP never
  // reads; there is nothing to update here.
  if (type != pinocchio::COLLISION || !geometry_) {
    return;
  }
  assert(index < geometry_->geometryObjects.size());
  geometry_->geometryObjects[index].placement = placement;
}

void Ocp::set_input_transform(std::size_t i,
                              const pinocchio::SE3& transform) noexcept {
  assert(i < transforms_.size());
  transforms_[i] = transform;
}

void Ocp::clear_input_transform(std::size_t i) noexcept {
  assert(i < transforms_.size());
  transforms_[i].reset();
}

}  // namespace agimus_controller
