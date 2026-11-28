#include "agimus_controller/mpc.hpp"

#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/state-base.hpp>
#include <stdexcept>
#include <string>

namespace agimus_controller {

namespace {

using Clock = std::chrono::steady_clock;

// Copies the values of `src` into `dst`, whose names and sizes were copied
// from `src` at setup.
void copy_debug_data(const OcpDebugData& src, OcpDebugData& dst) noexcept {
  copy_results(src.result, dst.result);
  for (std::size_t i = 0; i < src.references.size(); ++i) {
    dst.references[i].data = src.references[i].data;
  }
  for (std::size_t i = 0; i < src.residuals.size(); ++i) {
    dst.residuals[i].data = src.residuals[i].data;
  }
  dst.kkt_norm = src.kkt_norm;
  dst.nb_iter = src.nb_iter;
  dst.nb_qp_iter = src.nb_qp_iter;
  dst.problem_solved = src.problem_solved;
}

}  // namespace

Mpc::Mpc(std::shared_ptr<Ocp> ocp, std::shared_ptr<WarmStart> warm_start,
         std::shared_ptr<TrajectoryBuffer> buffer)
    : ocp_(std::move(ocp)),
      warm_start_(std::move(warm_start)),
      buffer_(std::move(buffer)) {
  if (!ocp_ || !warm_start_ || !buffer_) {
    throw std::invalid_argument(
        "Mpc: the ocp, warm_start and buffer pointers must not be null.");
  }
  const std::size_t n = static_cast<std::size_t>(ocp_->n_controls());
  if (buffer_->horizon_indexes().size() != n + 1) {
    throw std::invalid_argument(
        "Mpc: the buffer's horizon has " +
        std::to_string(buffer_->horizon_indexes().size()) +
        " nodes, the OCP needs n_controls + 1 = " + std::to_string(n + 1) +
        ".");
  }
  const OcpParams& params = ocp_->params();
  use_debug_data_ = params.use_debug_data;
  step_ = std::chrono::round<std::chrono::nanoseconds>(
      std::chrono::duration<double>(params.timesteps[0]));

  const auto& models = ocp_->problem()->get_runningModels();
  nq_ = static_cast<Eigen::Index>(models[0]->get_state()->get_nq());
  const Eigen::Index nx = ocp_->nx();
  x0_ = Eigen::VectorXd::Zero(nx);
  xs_.assign(n + 1, Eigen::VectorXd::Zero(nx));
  us_.clear();
  for (const auto& model : models) {
    us_.push_back(
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(model->get_nu())));
  }
  x_ = Eigen::VectorXd::Zero(nx);
  x_next_ = Eigen::VectorXd::Zero(nx);
  debug_data_.ocp = ocp_->debug_data();  // names and sizes
}

Status Mpc::run(const RobotState& state,
                std::chrono::nanoseconds /*now*/) noexcept {
  if (!buffer_->has_horizon()) {
    return Status::kNotEnoughData;  // B2
  }
  Clock::time_point t1;
  if (use_debug_data_) {
    t1 = Clock::now();
  }
  const std::vector<const WeightedTrajectoryPoint*>& horizon =
      buffer_->horizon();
  Status status = ocp_->set_reference_weighted_trajectory(horizon);
  if (status != Status::kOk) {
    return status;
  }
  Clock::time_point t2;
  if (use_debug_data_) {
    t2 = Clock::now();
  }
  status = warm_start_->generate(state, horizon, x0_, xs_, us_);
  if (status != Status::kOk) {
    return status;
  }
  Clock::time_point t3;
  if (use_debug_data_) {
    t3 = Clock::now();
  }
  ocp_->solve(x0_, xs_, us_);  // convergence: debug_data().ocp.problem_solved
  warm_start_->update_previous_solution(ocp_->results());
  const int reference_id = horizon[0]->id;  // before clear_past
  buffer_->clear_past();
  if (use_debug_data_) {
    const Clock::time_point t4 = Clock::now();
    copy_debug_data(ocp_->debug_data(), debug_data_.ocp);
    debug_data_.reference_id = reference_id;
    debug_data_.duration_iteration = t4 - t1;
    debug_data_.duration_horizon_update = t2 - t1;
    debug_data_.duration_generate_warm_start = t3 - t2;
    debug_data_.duration_ocp_solve = t4 - t3;
  }
  return Status::kOk;
}

Status Mpc::integrate(RobotState& state, const Eigen::VectorXd& u) noexcept {
  const Eigen::Index nx = x_.size();
  if (state.q.size() != nq_ || state.q.size() + state.v.size() != nx ||
      u.size() != us_[0].size()) {
    return Status::kSizeMismatch;
  }
  const Eigen::Index nv = state.v.size();
  x_.head(nq_) = state.q;
  x_.tail(nv) = state.v;
  ocp_->integrate(x_, u, x_next_);
  state.q = x_next_.head(nq_);
  state.v = x_next_.tail(nv);
  state.time += step_;  // B1, A12: the step just integrated
  return Status::kOk;
}

Status Mpc::push_back(const WeightedTrajectoryPoint& point) noexcept {
  return buffer_->push_back(point);
}

}  // namespace agimus_controller
