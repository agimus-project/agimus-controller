#include "agimus_controller/ocp_builders.hpp"

#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <memory>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <stdexcept>
#include <utility>

namespace agimus_controller {
namespace {

void add_weighted_cost(
    crocoddyl::CostModelSum& costs, const char* name,
    const std::shared_ptr<crocoddyl::StateMultibody>& state,
    const std::shared_ptr<crocoddyl::ResidualModelAbstract>& residual) {
  auto activation = std::make_shared<crocoddyl::ActivationModelWeightedQuad>(
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(residual->get_nr())));
  costs.addCost(name,
                std::make_shared<crocoddyl::CostModelResidual>(
                    state, activation, residual),
                1.0);
}

}  // namespace

OcpModels build_goal_reaching_models(const RobotModels& robot_models,
                                     const OcpParams& params) {
  if (params.n_controls <= 0 ||
      params.timesteps.size() != static_cast<std::size_t>(params.n_controls)) {
    throw std::invalid_argument(
        "build_goal_reaching_models: OcpParams must be finalized.");
  }
  const pinocchio::Model& robot = robot_models.robot_model();
  if (robot_models.armature().size() != robot.nv) {
    throw std::invalid_argument(
        "build_goal_reaching_models: armature has size " +
        std::to_string(robot_models.armature().size()) +
        ", the robot model has nv = " + std::to_string(robot.nv) + ".");
  }

  auto state = std::make_shared<crocoddyl::StateMultibody>(
      std::make_shared<pinocchio::Model>(robot));
  auto actuation = std::make_shared<crocoddyl::ActuationModelFull>(state);
  const pinocchio::Model& model = *state->get_pinocchio();

  Eigen::VectorXd x_reference = Eigen::VectorXd::Zero(state->get_nx());
  x_reference.head(model.nq) = pinocchio::neutral(model);

  // One node: the costs share nothing between nodes (references differ).
  auto make_node = [&](bool terminal, double dt, NodeUpdater& updater) {
    auto costs = std::make_shared<crocoddyl::CostModelSum>(state);
    add_weighted_cost(
        *costs, "stateReg", state,
        std::make_shared<crocoddyl::ResidualModelState>(state, x_reference));
    if (!terminal) {
      add_weighted_cost(
          *costs, "ctrlReg", state,
          std::make_shared<crocoddyl::ResidualModelControl>(state));
    }
    add_weighted_cost(*costs, "goalTracking", state,
                      std::make_shared<crocoddyl::ResidualModelFramePlacement>(
                          state, 0, pinocchio::SE3::Identity()));

    updater.costs.push_back(
        make_cost_updater(UpdateRule::kState, *costs, "stateReg", model));
    if (!terminal) {
      updater.costs.push_back(
          make_cost_updater(UpdateRule::kControl, *costs, "ctrlReg", model));
    }
    updater.costs.push_back(make_cost_updater(
        UpdateRule::kFramePlacement, *costs, "goalTracking", model, {}, true));

    auto dam =
        std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
            state, actuation, costs);
    dam->set_armature(robot_models.armature());
    return std::make_shared<crocoddyl::IntegratedActionModelEuler>(dam, dt);
  };

  OcpModels models;
  models.state = state;
  models.running.reserve(params.timesteps.size());
  models.running_updaters.reserve(params.timesteps.size());
  for (const double dt : params.timesteps) {
    NodeUpdater updater;
    models.running.push_back(make_node(false, dt, updater));
    models.running_updaters.push_back(std::move(updater));
  }
  models.terminal = make_node(true, 0.0, models.terminal_updater);
  return models;
}

}  // namespace agimus_controller
