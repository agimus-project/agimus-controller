// Test fixtures shared by test_ocp.cpp and test_cost_updater.cpp. A separate
// translation unit, so that each test file stays well under the 2 GB compile
// limit (R6).

#include "panda_ocp.hpp"

#include <crocoddyl/core/activations/quadratic.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/residuals/control-gravity.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <filesystem>
#include <string_view>

namespace agimus_controller::test {

/// example-robot-data's panda as in the golden files: the 7 arm joints,
/// armature 0.1, capsules, no self collision. Loaded once.
const RobotModels& panda() {
  static const RobotModels models = [] {
    namespace fs = std::filesystem;
    const fs::path dir = AGIMUS_EXAMPLE_ROBOT_DATA_DIR;
    RobotModelParameters p;
    p.q0 = Eigen::VectorXd::Zero(9);
    for (int i = 1; i <= 7; ++i) {
      p.moving_joint_names.push_back("panda_joint" + std::to_string(i));
    }
    p.robot_urdf_path =
        (dir / "robots" / "panda_description" / "urdf" / "panda.urdf").string();
    p.srdf_path =
        (dir / "robots" / "panda_description" / "srdf" / "panda.srdf").string();
    p.urdf_meshes_dir = dir.parent_path().string();
    p.collision_as_capsule = true;
    p.self_collision = false;
    p.armature = Eigen::VectorXd::Constant(7, 0.1);
    return RobotModels(p);
  }();
  return models;
}

/// A new crocoddyl state over a copy of the panda's reduced model.
std::shared_ptr<crocoddyl::StateMultibody> panda_state() {
  return std::make_shared<crocoddyl::StateMultibody>(
      std::make_shared<pinocchio::Model>(panda().robot_model()));
}

/// Adds a residual cost, with a weighted-quad activation (weights 1) or the
/// default quadratic one.
void add_cost(crocoddyl::CostModelSum& costs, const std::string& name,
              std::shared_ptr<crocoddyl::ResidualModelAbstract> residual,
              double weight, bool weighted_quad, bool active) {
  const auto state = costs.get_state();
  std::shared_ptr<crocoddyl::CostModelAbstract> cost;
  if (weighted_quad) {
    auto activation = std::make_shared<crocoddyl::ActivationModelWeightedQuad>(
        Eigen::VectorXd::Ones(static_cast<Eigen::Index>(residual->get_nr())));
    cost = std::make_shared<crocoddyl::CostModelResidual>(state, activation,
                                                          residual);
  } else {
    cost = std::make_shared<crocoddyl::CostModelResidual>(state, residual);
  }
  costs.addCost(name, cost, weight, active);
}

/// An Euler node over free forward dynamics with full actuation and the
/// panda's armature.
std::shared_ptr<crocoddyl::ActionModelAbstract> make_node(
    const std::shared_ptr<crocoddyl::StateMultibody>& state,
    const std::shared_ptr<crocoddyl::CostModelSum>& costs, double dt) {
  auto actuation = std::make_shared<crocoddyl::ActuationModelFull>(state);
  auto dam =
      std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(
          state, actuation, costs);
  dam->set_armature(panda().armature());
  return std::make_shared<crocoddyl::IntegratedActionModelEuler>(dam, dt);
}

/// The cost sum of a node built by make_node().
crocoddyl::CostModelSum& costs_of(const crocoddyl::ActionModelAbstract& node) {
  const auto& euler =
      dynamic_cast<const crocoddyl::IntegratedActionModelEuler&>(node);
  const auto& dam =
      dynamic_cast<const crocoddyl::DifferentialActionModelFreeFwdDynamics&>(
          *euler.get_differential());
  return *dam.get_costs();
}

// ---- A realistic OCP: panda, 20 nodes, every rule --------------------------

OcpParams make_params(int n, int solver_iters) {
  OcpParams p;
  p.dt = 0.01;
  p.solver_iters = solver_iters;
  p.dt_factor_n_seq = {{1}, {n}};
  p.horizon_size = n;
  p.n_threads = 1;
  p.finalize();
  return p;
}

// Running nodes: every rule. With Frames::kFromTrajectory, the four frame
// costs follow the point's only end effector and there is no visual
// servoing. Terminal node: state, placement and visual servoing.
Built build_models(const OcpParams& params, Frames frames) {
  Built b;
  b.state = panda_state();
  const auto& state = b.state;
  const pinocchio::Model& model = *state->get_pinocchio();
  const pinocchio::FrameIndex tcp = model.getFrameId(kTcp);
  const bool fixed = frames == Frames::kFixed;
  auto placement = [&](pinocchio::FrameIndex id) {
    return std::make_shared<crocoddyl::ResidualModelFramePlacement>(
        state, id, pinocchio::SE3::Identity());
  };
  auto frame_updater = [&](UpdateRule rule, crocoddyl::CostModelSum& costs,
                           const char* name) {
    return fixed ? make_cost_updater(rule, costs, name, model, kTcp)
                 : make_cost_updater(rule, costs, name, model, {}, true);
  };
  b.models.state = state;
  b.models.geometry =
      std::make_shared<pinocchio::GeometryModel>(panda().collision_model());
  b.models.input_transforms.push_back(
      TransformInput{FrameName("world"), FrameName("object")});

  for (int i = 0; i <= params.n_controls; ++i) {
    const bool terminal = i == params.n_controls;
    auto costs = std::make_shared<crocoddyl::CostModelSum>(state);
    NodeUpdater node;
    add_cost(*costs, "state_reg",
             std::make_shared<crocoddyl::ResidualModelState>(state), 0.1);
    add_cost(*costs, "ee_pose", placement(tcp), 1.0);
    if (fixed) add_cost(*costs, "vs", placement(tcp), 1.0);
    node.costs.push_back(
        make_cost_updater(UpdateRule::kState, *costs, "state_reg", model));
    node.costs.push_back(
        frame_updater(UpdateRule::kFramePlacement, *costs, "ee_pose"));
    if (!terminal) {
      add_cost(*costs, "ctrl_reg",
               std::make_shared<crocoddyl::ResidualModelControl>(state), 1e-3);
      add_cost(*costs, "ctrl_grav",
               std::make_shared<crocoddyl::ResidualModelControlGrav>(state),
               1e-3);
      add_cost(*costs, "ee_translation",
               std::make_shared<crocoddyl::ResidualModelFrameTranslation>(
                   state, tcp, Eigen::Vector3d::Zero()),
               1.0);
      add_cost(*costs, "ee_rotation",
               std::make_shared<crocoddyl::ResidualModelFrameRotation>(
                   state, tcp, Eigen::Matrix3d::Identity()),
               1.0);
      add_cost(*costs, "ee_velocity",
               std::make_shared<crocoddyl::ResidualModelFrameVelocity>(
                   state, tcp, pinocchio::Motion::Zero(), pinocchio::WORLD),
               0.1);
      // A stand-in for colmpc's distance residual: only the weight changes.
      add_cost(*costs, "collision",
               std::make_shared<crocoddyl::ResidualModelState>(state), 1e-3,
               false);
      node.costs.push_back(
          make_cost_updater(UpdateRule::kControl, *costs, "ctrl_reg", model));
      node.costs.push_back(make_cost_updater(UpdateRule::kControlGrav, *costs,
                                             "ctrl_grav", model));
      node.costs.push_back(frame_updater(UpdateRule::kFrameTranslation, *costs,
                                         "ee_translation"));
      node.costs.push_back(
          frame_updater(UpdateRule::kFrameRotation, *costs, "ee_rotation"));
      node.costs.push_back(
          frame_updater(UpdateRule::kFrameVelocity, *costs, "ee_velocity"));
      node.costs.push_back(make_cost_updater(UpdateRule::kCollisionWeight,
                                             *costs, "collision", model));
    }
    if (fixed) {
      node.costs.push_back(make_cost_updater(
          UpdateRule::kVisualServoing, *costs, "vs", model, kTcp, false, 0));
    }
    const double dt = terminal ? 0.0 : params.timesteps[i];
    auto action = make_node(state, costs, dt);
    if (terminal) {
      b.models.terminal = action;
      b.models.terminal_updater = std::move(node);
    } else {
      b.models.running.push_back(action);
      b.models.running_updaters.push_back(std::move(node));
    }
  }
  b.models.reference_costs = {"state_reg",  "ctrl_reg",       "ctrl_grav",
                              "ee_pose",    "ee_translation", "ee_rotation",
                              "ee_velocity"};
  b.models.residual_costs = {"ee_pose", "state_reg"};
  return b;
}

Eigen::VectorXd neutral(const pinocchio::Model& model) {
  return Eigen::VectorXd::Zero(model.nq);
}

// A target near the panda's tcp (or link7) at q = 0.
EndEffectorInput ee_input(const char* frame, const char* name, double shift) {
  EndEffectorInput ee;
  ee.frame_name = FrameName(name);
  const bool tcp = std::string_view(frame) == kTcp;
  ee.pose = pinocchio::SE3(
      tcp ? Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()))
          : Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(0.13, shift, tcp ? 0.82 : 0.93));
  ee.w_pose << 10, 10, 10, 1, 1, 1;
  ee.w_twist.setConstant(0.1);
  return ee;
}

// Points for the fixed-frame OCP: tcp target, a link7 entry (ignored) and
// the visual-servoing target.
std::vector<WeightedTrajectoryPoint> make_points(const pinocchio::Model& model,
                                                 int n, Frames frames,
                                                 const char* frame) {
  std::vector<WeightedTrajectoryPoint> points;
  for (int i = 0; i <= n; ++i) {
    auto p = make_trajectory_point(model.nq, model.nv);
    p.id = i;
    p.q = neutral(model);
    p.w_q.setConstant(1.0);
    p.w_qdot.setConstant(0.1);
    p.w_robot_effort.setConstant(1e-2);
    p.w_collision_avoidance = 1.0;
    const double shift = 0.001 * i;
    if (frames == Frames::kFixed) {
      p.ee_inputs.push_back(ee_input(kTcp, kTcp, shift));
      p.ee_inputs.push_back(ee_input(kLink7, kLink7, shift));
      p.ee_inputs.push_back(ee_input(kTcp, kTcpVs, shift));
    } else {
      p.ee_inputs.push_back(ee_input(frame, frame, shift));
    }
    points.push_back(std::move(p));
  }
  return points;
}

std::vector<const WeightedTrajectoryPoint*> pointers(
    const std::vector<WeightedTrajectoryPoint>& points) {
  std::vector<const WeightedTrajectoryPoint*> out;
  for (const auto& p : points) out.push_back(&p);
  return out;
}

// ---- B10 -------------------------------------------------------------------

// The B10 OCP: state, control, placement and an "extra" translation cost on
// panda_link7, active (to be toggled) or disabled at setup.
Built build_b10_models(const OcpParams& params, bool extra_active) {
  Built b;
  b.state = panda_state();
  const auto& state = b.state;
  const pinocchio::Model& model = *state->get_pinocchio();
  b.models.state = state;
  for (int i = 0; i <= params.n_controls; ++i) {
    const bool terminal = i == params.n_controls;
    auto costs = std::make_shared<crocoddyl::CostModelSum>(state);
    NodeUpdater node;
    add_cost(*costs, "state_reg",
             std::make_shared<crocoddyl::ResidualModelState>(state), 0.1);
    add_cost(*costs, "ee_pose",
             std::make_shared<crocoddyl::ResidualModelFramePlacement>(
                 state, model.getFrameId(kTcp), pinocchio::SE3::Identity()),
             1.0);
    // Disabled at setup through addCost: crocoddyl's bookkeeping (nr_, the
    // name sets, the action model's nr) is consistent from the start.
    add_cost(*costs, "extra",
             std::make_shared<crocoddyl::ResidualModelFrameTranslation>(
                 state, model.getFrameId(kLink7), Eigen::Vector3d::Zero()),
             10.0, true, extra_active);
    node.costs.push_back(
        make_cost_updater(UpdateRule::kState, *costs, "state_reg", model));
    node.costs.push_back(make_cost_updater(UpdateRule::kFramePlacement, *costs,
                                           "ee_pose", model, kTcp));
    node.costs.push_back(make_cost_updater(UpdateRule::kFrameTranslation,
                                           *costs, "extra", model, kLink7));
    if (!terminal) {
      add_cost(*costs, "ctrl_reg",
               std::make_shared<crocoddyl::ResidualModelControl>(state), 1e-3);
      node.costs.push_back(
          make_cost_updater(UpdateRule::kControl, *costs, "ctrl_reg", model));
    }
    auto action = make_node(state, costs, terminal ? 0.0 : params.timesteps[i]);
    if (terminal) {
      b.models.terminal = action;
      b.models.terminal_updater = std::move(node);
    } else {
      b.models.running.push_back(action);
      b.models.running_updaters.push_back(std::move(node));
    }
  }
  return b;
}

// Points with the extra cost's weights on or off.
std::vector<WeightedTrajectoryPoint> b10_points(const pinocchio::Model& model,
                                                int n, bool extra_on) {
  auto points = make_points(model, n, Frames::kFixed);
  for (auto& p : points) {
    p.ee_inputs[1].w_pose.head<3>().setConstant(extra_on ? 5.0 : 0.0);
    p.ee_inputs[1].pose.translation() += Eigen::Vector3d(0.0, 0.1, 0.0);
  }
  return points;
}

const Eigen::VectorXd& node_residual(const Ocp& ocp, std::size_t node,
                                     const std::string& cost) {
  const auto& euler = dynamic_cast<const crocoddyl::IntegratedActionDataEuler&>(
      *ocp.problem()->get_runningDatas()[node]);
  const auto& dam =
      dynamic_cast<const crocoddyl::DifferentialActionDataFreeFwdDynamics&>(
          *euler.differential);
  return dam.costs->costs.at(cost)->residual->r;
}

const pinocchio::SE3& placement_reference(
    const crocoddyl::ActionModelAbstract& node, const std::string& cost) {
  return dynamic_cast<const crocoddyl::ResidualModelFramePlacement&>(
             *costs_of(node).get_costs().at(cost)->cost->get_residual())
      .get_reference();
}

}  // namespace agimus_controller::test
