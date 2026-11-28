#pragma once

// Test fixtures shared by test_ocp.cpp and test_cost_updater.cpp: the panda of
// the golden files, hand-built crocoddyl nodes and the OCPs of test_ocp.cpp.
// Defined in panda_ocp.cpp (compile RAM, R6).

#include <Eigen/Core>
#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/fwd.hpp>
#include <memory>
#include <string>
#include <vector>

#include "agimus_controller/ocp.hpp"
#include "agimus_controller/robot_models.hpp"

namespace agimus_controller::test {

inline constexpr const char* kTcp = "panda_hand_tcp";
inline constexpr const char* kLink7 = "panda_link7";
inline constexpr const char* kTcpVs = "panda_hand_tcp_vs";
inline constexpr int kN = 20;  // realistic horizon for the RT tests

/// example-robot-data's panda as in the golden files: the 7 arm joints,
/// armature 0.1, capsules, no self collision. Loaded once.
const RobotModels& panda();

/// A new crocoddyl state over a copy of the panda's reduced model.
std::shared_ptr<crocoddyl::StateMultibody> panda_state();

/// Adds a residual cost, with a weighted-quad activation (weights 1) or the
/// default quadratic one.
void add_cost(crocoddyl::CostModelSum& costs, const std::string& name,
              std::shared_ptr<crocoddyl::ResidualModelAbstract> residual,
              double weight, bool weighted_quad = true, bool active = true);

/// An Euler node over free forward dynamics with full actuation and the
/// panda's armature.
std::shared_ptr<crocoddyl::ActionModelAbstract> make_node(
    const std::shared_ptr<crocoddyl::StateMultibody>& state,
    const std::shared_ptr<crocoddyl::CostModelSum>& costs, double dt);

/// The cost sum of a node built by make_node().
crocoddyl::CostModelSum& costs_of(const crocoddyl::ActionModelAbstract& node);

/// Finalized parameters: dt 0.01, n equal steps.
OcpParams make_params(int n = kN, int solver_iters = 10);

enum class Frames { kFixed, kFromTrajectory };

struct Built {
  OcpModels models;
  std::shared_ptr<crocoddyl::StateMultibody> state;
};

/// Running nodes: every rule. With Frames::kFromTrajectory, the four frame
/// costs follow the point's only end effector and there is no visual
/// servoing. Terminal node: state, placement and visual servoing.
Built build_models(const OcpParams& params, Frames frames = Frames::kFixed);

/// The B10 OCP: state, control, placement and an "extra" translation cost on
/// panda_link7, active (to be toggled) or disabled at setup.
Built build_b10_models(const OcpParams& params, bool extra_active);

/// The panda's neutral configuration: zero (every arm joint is revolute).
Eigen::VectorXd neutral(const pinocchio::Model& model);

/// A target near the panda's tcp (or link7) at q = 0.
EndEffectorInput ee_input(const char* frame, const char* name, double shift);

/// n + 1 points. kFixed: tcp target, a link7 entry and the visual-servoing
/// target. kFromTrajectory: one end effector, `frame`.
std::vector<WeightedTrajectoryPoint> make_points(const pinocchio::Model& model,
                                                 int n, Frames frames,
                                                 const char* frame = kTcp);

/// Points with the B10 "extra" cost's weights on or off.
std::vector<WeightedTrajectoryPoint> b10_points(const pinocchio::Model& model,
                                                int n, bool extra_on);

std::vector<const WeightedTrajectoryPoint*> pointers(
    const std::vector<WeightedTrajectoryPoint>& points);

/// The residual vector of `cost` in the data of a running node.
const Eigen::VectorXd& node_residual(const Ocp& ocp, std::size_t node,
                                     const std::string& cost);

/// The reference of a frame-placement cost of a node built by make_node().
const pinocchio::SE3& placement_reference(
    const crocoddyl::ActionModelAbstract& node, const std::string& cost);

}  // namespace agimus_controller::test
