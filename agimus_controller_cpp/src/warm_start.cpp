#include "agimus_controller/warm_start.hpp"

#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <stdexcept>

namespace agimus_controller {

void WarmStart::update_previous_solution(const OcpResults& /*previous*/) {}

// ---- WarmStartReference ----------------------------------------------------

WarmStartReference::WarmStartReference(const pinocchio::Model& model)
    : model_(model), data_(model_) {}

Status WarmStartReference::generate(
    const RobotState& x0_state,
    const std::vector<const WeightedTrajectoryPoint*>& horizon,
    Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& xs,
    std::vector<Eigen::VectorXd>& us) {
  const Eigen::Index nq = model_.nq;
  const Eigen::Index nv = model_.nv;
  const std::size_t n = horizon.size();
  if (n < 1 || xs.size() != n || us.size() != n - 1 || x0.size() != nq + nv ||
      x0_state.q.size() != nq || x0_state.v.size() != nv ||
      x0_state.a.size() != nv) {
    return Status::kSizeMismatch;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const WeightedTrajectoryPoint* p = horizon[i];
    if (xs[i].size() != nq + nv || (i + 1 < n && us[i].size() != nv)) {
      return Status::kSizeMismatch;
    }
    // horizon[0] is replaced by the state, and the last acceleration is
    // unused.
    if (i > 0 && (p == nullptr || p->q.size() != nq || p->qdot.size() != nv ||
                  (i + 1 < n && p->qddot.size() != nv))) {
      return Status::kSizeMismatch;
    }
  }

  x0.head(nq) = x0_state.q;
  x0.tail(nv) = x0_state.v;
  xs[0] = x0;
  for (std::size_t i = 1; i < n; ++i) {
    xs[i].head(nq) = horizon[i]->q;
    xs[i].tail(nv) = horizon[i]->qdot;
  }
  if (n > 1) {
    us[0] = pinocchio::rnea(model_, data_, x0_state.q, x0_state.v, x0_state.a);
  }
  for (std::size_t i = 1; i + 1 < n; ++i) {
    us[i] = pinocchio::rnea(model_, data_, horizon[i]->q, horizon[i]->qdot,
                            horizon[i]->qddot);
  }
  return Status::kOk;
}

// ---- WarmStartShiftPreviousSolution ---------------------------------------

WarmStartShiftPreviousSolution::WarmStartShiftPreviousSolution(
    const RobotModels& robot_models, const OcpParams& params)
    : timesteps_(params.timesteps) {
  if (timesteps_.empty()) {
    throw std::invalid_argument(
        "WarmStartShiftPreviousSolution: params are not finalized");
  }
  dt_ = timesteps_[0];
  if (params.dt != dt_) {
    throw std::invalid_argument(
        "WarmStartShiftPreviousSolution: params.dt != timesteps[0]");
  }
  for (const double step : timesteps_) {
    if (!(step >= dt_)) {
      throw std::invalid_argument(
          "WarmStartShiftPreviousSolution: a time step is shorter than dt");
    }
  }

  state_ = std::make_shared<crocoddyl::StateMultibody>(
      std::make_shared<pinocchio::Model>(robot_models.robot_model()));
  auto actuation = std::make_shared<crocoddyl::ActuationModelFull>(state_);
  auto costs = std::make_shared<crocoddyl::CostModelSum>(state_);
  auto differential =
      std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
          state_, actuation, costs);
  const Eigen::VectorXd& armature = robot_models.armature();
  if (armature.size() > 0) {
    differential->set_armature(armature);
  }
  integrator_ = std::make_shared<crocoddyl::IntegratedActionModelEuler>(
      differential, dt_);
  integrator_data_ = integrator_->createData();

  previous_.resize(timesteps_.size(),
                   static_cast<Eigen::Index>(state_->get_nx()),
                   static_cast<Eigen::Index>(state_->get_ndx()),
                   static_cast<Eigen::Index>(actuation->get_nu()));
  // Warm-up: some libraries allocate on their first call (rt_safety.md, 7).
  integrator_->calc(integrator_data_, previous_.states[0],
                    previous_.feed_forward_terms[0]);
}

WarmStartShiftPreviousSolution::~WarmStartShiftPreviousSolution() = default;

void WarmStartShiftPreviousSolution::update_previous_solution(
    const OcpResults& previous) {
  if (copy_results(previous, previous_) == Status::kOk) {
    has_previous_ = true;
  }
}

void WarmStartShiftPreviousSolution::shift() {
  std::vector<Eigen::VectorXd>& xs = previous_.states;
  std::vector<Eigen::VectorXd>& us = previous_.feed_forward_terms;
  const std::size_t n = timesteps_.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (timesteps_[i] == dt_) {
      xs[i] = xs[i + 1];
      // For the last running model, i + 1 is the terminal model, which has
      // no control: the two last controls end up equal.
      if (i + 1 < n) {
        us[i] = us[i + 1];
      }
    } else {
      integrator_->calc(integrator_data_, xs[i], us[i]);
      xs[i] = integrator_data_->xnext;
      // The control is kept: we are still in the segment where us[i] applies.
    }
  }
  // A13 (B18): the terminal state of a long last step moves by one dt too.
  if (timesteps_.back() > dt_) {
    integrator_->calc(integrator_data_, xs[n], us[n - 1]);
    xs[n] = integrator_data_->xnext;
  }
}

Status WarmStartShiftPreviousSolution::generate(
    const RobotState& x0_state,
    const std::vector<const WeightedTrajectoryPoint*>& /*horizon*/,
    Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& xs,
    std::vector<Eigen::VectorXd>& us) {
  if (!has_previous_) {
    return Status::kNoPreviousSolution;
  }
  const std::size_t n = timesteps_.size();
  const Eigen::Index nq = state_->get_nq();
  const Eigen::Index nv = state_->get_nv();
  const Eigen::Index nx = state_->get_nx();
  if (x0_state.q.size() != nq || x0_state.v.size() != nv || x0.size() != nx ||
      xs.size() != n + 1 || us.size() != n) {
    return Status::kSizeMismatch;
  }
  for (std::size_t i = 0; i <= n; ++i) {
    if (xs[i].size() != nx || (i < n && us[i].size() != nv)) {
      return Status::kSizeMismatch;
    }
  }

  shift();
  x0.head(nq) = x0_state.q;
  x0.tail(nv) = x0_state.v;
  for (std::size_t i = 0; i <= n; ++i) {
    xs[i] = previous_.states[i];
  }
  for (std::size_t i = 0; i < n; ++i) {
    us[i] = previous_.feed_forward_terms[i];
  }
  return Status::kOk;
}

}  // namespace agimus_controller
