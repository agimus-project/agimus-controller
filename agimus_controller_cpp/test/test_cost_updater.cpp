// Cost updaters (design.md, "Cost updaters"): the update rules of
// PY/ocp/ocp_croco_generic.py:153-476,713-721 and the zero-weight rule B10.

#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/multibody/residuals/control-gravity.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <stdexcept>

#include "agimus_controller/cost_updater.hpp"
#include "panda_ocp.hpp"
#include "rt_guard.hpp"

using agimus_controller::apply;
using agimus_controller::CostUpdater;
using agimus_controller::EndEffectorInput;
using agimus_controller::FrameName;
using agimus_controller::InputTransforms;
using agimus_controller::make_cost_updater;
using agimus_controller::make_trajectory_point;
using agimus_controller::NodeUpdater;
using agimus_controller::Status;
using agimus_controller::UpdateRule;
using agimus_controller::Vector6d;
using agimus_controller::WeightedTrajectoryPoint;
using agimus_controller::test::add_cost;
using agimus_controller::test::panda_state;

namespace {

constexpr const char* kTcp = "panda_hand_tcp";
constexpr const char* kLink7 = "panda_link7";

// One node's costs on the panda, one per residual type, with weighted-quad
// activations, plus a default-quadratic and a "collision" cost.
struct Costs {
  std::shared_ptr<crocoddyl::StateMultibody> state = panda_state();
  std::shared_ptr<crocoddyl::CostModelSum> sum =
      std::make_shared<crocoddyl::CostModelSum>(state);
  const pinocchio::Model& model = *state->get_pinocchio();
  pinocchio::FrameIndex tcp = model.getFrameId(kTcp);
  pinocchio::FrameIndex link7 = model.getFrameId(kLink7);

  Costs() {
    add_cost(*sum, "state",
             std::make_shared<crocoddyl::ResidualModelState>(state), 1.0);
    add_cost(*sum, "control",
             std::make_shared<crocoddyl::ResidualModelControl>(state), 1.0);
    add_cost(*sum, "control_grav",
             std::make_shared<crocoddyl::ResidualModelControlGrav>(state), 1.0);
    add_cost(*sum, "placement", placement(tcp), 1.0);
    add_cost(*sum, "translation",
             std::make_shared<crocoddyl::ResidualModelFrameTranslation>(
                 state, tcp, Eigen::Vector3d::Zero()),
             1.0);
    add_cost(*sum, "rotation",
             std::make_shared<crocoddyl::ResidualModelFrameRotation>(
                 state, tcp, Eigen::Matrix3d::Identity()),
             1.0);
    add_cost(*sum, "velocity",
             std::make_shared<crocoddyl::ResidualModelFrameVelocity>(
                 state, tcp, pinocchio::Motion::Zero(), pinocchio::WORLD),
             1.0);
    add_cost(*sum, "placement_quad", placement(tcp), 1.0, false);
    add_cost(*sum, "vs", placement(tcp), 1.0);
    add_cost(*sum, "collision",
             std::make_shared<crocoddyl::ResidualModelState>(state), 1.0,
             false);
  }

  std::shared_ptr<crocoddyl::ResidualModelFramePlacement> placement(
      pinocchio::FrameIndex id) const {
    return std::make_shared<crocoddyl::ResidualModelFramePlacement>(
        state, id, pinocchio::SE3::Identity());
  }

  crocoddyl::CostItem& item(const std::string& name) const {
    return *sum->get_costs().at(name);
  }
  template <typename Residual>
  Residual& residual(const std::string& name) const {
    return dynamic_cast<Residual&>(*item(name).cost->get_residual());
  }
  const Eigen::VectorXd& weights(const std::string& name) const {
    return dynamic_cast<const crocoddyl::ActivationModelWeightedQuad&>(
               *item(name).cost->get_activation())
        .get_weights();
  }
  CostUpdater updater(UpdateRule rule, const std::string& name,
                      std::string_view frame = {}, bool from_trajectory = false,
                      std::size_t transform = 0) {
    return make_cost_updater(rule, *sum, name, model, frame, from_trajectory,
                             transform);
  }
};

EndEffectorInput ee_input(const char* frame, double seed) {
  EndEffectorInput ee;
  ee.frame_name = FrameName(frame);
  ee.pose = pinocchio::SE3(
      Eigen::AngleAxisd(seed, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d(seed, 2 * seed, 3 * seed));
  ee.w_pose << 1 + seed, 2, 3, 4, 5, 6;
  ee.twist = pinocchio::Motion(Vector6d::Constant(seed));
  ee.w_twist << 6, 5, 4, 3, 2, 1 + seed;
  return ee;
}

WeightedTrajectoryPoint point(const crocoddyl::StateMultibody& state,
                              double seed) {
  auto p = make_trajectory_point(state.get_nq(), state.get_nv());
  p.q.setConstant(seed);
  p.qdot.setConstant(2 * seed);
  p.robot_effort.setConstant(3 * seed);
  p.w_q.setConstant(1.0);
  p.w_qdot.setConstant(2.0);
  p.w_robot_effort.setConstant(3.0);
  p.w_collision_avoidance = 7.0;
  return p;
}

NodeUpdater node_of(CostUpdater u) {
  NodeUpdater node;
  node.costs.push_back(std::move(u));
  return node;
}

const InputTransforms kNoTransform(1);

}  // namespace

// ---- Rules -----------------------------------------------------------------

TEST_CASE("CostUpdater: kState writes [q; qdot] and [w_q; w_qdot]") {
  Costs c;
  auto node = node_of(c.updater(UpdateRule::kState, "state"));
  const auto p = point(*c.state, 0.5);
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
  Eigen::VectorXd x(14), w(14);
  x << p.q, p.qdot;
  w << p.w_q, p.w_qdot;
  CHECK(c.residual<crocoddyl::ResidualModelState>("state").get_reference() ==
        x);
  CHECK(c.weights("state") == w);
  CHECK(c.item("state").active);
}

TEST_CASE("CostUpdater: kControl and kControlGrav") {
  Costs c;
  NodeUpdater node;
  node.costs.push_back(c.updater(UpdateRule::kControl, "control"));
  node.costs.push_back(c.updater(UpdateRule::kControlGrav, "control_grav"));
  const auto p = point(*c.state, 0.5);
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
  CHECK(
      c.residual<crocoddyl::ResidualModelControl>("control").get_reference() ==
      p.robot_effort);
  CHECK(c.weights("control") == p.w_robot_effort);
  // B4: ControlGrav has no reference, only weights.
  CHECK(c.weights("control_grav") == p.w_robot_effort);
}

TEST_CASE("CostUpdater: frame rules with a fixed frame") {
  Costs c;
  NodeUpdater node;
  node.costs.push_back(
      c.updater(UpdateRule::kFramePlacement, "placement", kTcp));
  node.costs.push_back(
      c.updater(UpdateRule::kFrameTranslation, "translation", kTcp));
  node.costs.push_back(c.updater(UpdateRule::kFrameRotation, "rotation", kTcp));
  node.costs.push_back(c.updater(UpdateRule::kFrameVelocity, "velocity", kTcp));
  auto p = point(*c.state, 0.5);
  // Several end effectors: a fixed frame is looked up by name.
  p.ee_inputs.push_back(ee_input(kLink7, 0.1));
  p.ee_inputs.push_back(ee_input(kTcp, 0.3));
  const EndEffectorInput& ee = p.ee_inputs[1];
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);

  CHECK(c.residual<crocoddyl::ResidualModelFramePlacement>("placement")
            .get_reference()
            .isApprox(ee.pose));
  CHECK(c.weights("placement") == Eigen::VectorXd(ee.w_pose));
  CHECK(c.residual<crocoddyl::ResidualModelFrameTranslation>("translation")
            .get_reference() == ee.pose.translation());
  CHECK(c.weights("translation") == Eigen::VectorXd(ee.w_pose.head<3>()));
  CHECK(c.residual<crocoddyl::ResidualModelFrameRotation>("rotation")
            .get_reference() == ee.pose.rotation());
  CHECK(c.weights("rotation") == Eigen::VectorXd(ee.w_pose.tail<3>()));
  CHECK(c.residual<crocoddyl::ResidualModelFrameVelocity>("velocity")
            .get_reference()
            .isApprox(ee.twist));
  CHECK(c.weights("velocity") == Eigen::VectorXd(ee.w_twist));
  // The residual keeps its frame.
  CHECK(c.residual<crocoddyl::ResidualModelFramePlacement>("placement")
            .get_id() == c.tcp);
}

TEST_CASE("CostUpdater: a missing fixed frame is kUnknownFrame") {
  Costs c;
  auto node =
      node_of(c.updater(UpdateRule::kFramePlacement, "placement", kTcp));
  auto p = point(*c.state, 0.5);
  p.ee_inputs.push_back(ee_input(kLink7, 0.1));
  const auto before = c.weights("placement");
  CHECK(apply(node, p, kNoTransform) == Status::kUnknownFrame);
  CHECK(c.weights("placement") == before);  // unchanged
}

TEST_CASE("CostUpdater: frame from the trajectory follows the point's frame") {
  Costs c;
  NodeUpdater node;
  node.costs.push_back(
      c.updater(UpdateRule::kFramePlacement, "placement", {}, true));
  node.costs.push_back(
      c.updater(UpdateRule::kFrameTranslation, "translation", {}, true));
  node.costs.push_back(
      c.updater(UpdateRule::kFrameRotation, "rotation", {}, true));
  node.costs.push_back(
      c.updater(UpdateRule::kFrameVelocity, "velocity", {}, true));
  // The cache starts at the residual's frame.
  CHECK(node.costs[0].frame_name == std::string_view(kTcp));
  CHECK(node.costs[0].frame_id == c.tcp);

  auto p = point(*c.state, 0.5);
  p.ee_inputs.push_back(ee_input(kLink7, 0.2));
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
  CHECK(c.residual<crocoddyl::ResidualModelFramePlacement>("placement")
            .get_id() == c.link7);
  CHECK(c.residual<crocoddyl::ResidualModelFrameTranslation>("translation")
            .get_id() == c.link7);
  CHECK(
      c.residual<crocoddyl::ResidualModelFrameRotation>("rotation").get_id() ==
      c.link7);
  CHECK(
      c.residual<crocoddyl::ResidualModelFrameVelocity>("velocity").get_id() ==
      c.link7);
  CHECK(c.residual<crocoddyl::ResidualModelFramePlacement>("placement")
            .get_reference()
            .isApprox(p.ee_inputs[0].pose));
  CHECK(node.costs[0].frame_name == std::string_view(kLink7));
  CHECK(node.costs[0].frame_id == c.link7);

  SUBCASE("the cache is the key: a changed residual id is not re-read") {
    // Same name again: no lookup, so an id changed behind the updater's back
    // stays (the cache, not the residual, is the source of truth).
    c.residual<crocoddyl::ResidualModelFramePlacement>("placement")
        .set_id(c.tcp);
    REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
    CHECK(c.residual<crocoddyl::ResidualModelFramePlacement>("placement")
              .get_id() == c.tcp);
  }
  SUBCASE("back to the first frame") {
    p.ee_inputs[0] = ee_input(kTcp, 0.3);
    REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
    CHECK(c.residual<crocoddyl::ResidualModelFrameVelocity>("velocity")
              .get_id() == c.tcp);
    CHECK(node.costs[3].frame_id == c.tcp);
  }
  SUBCASE("an unknown frame is kUnknownFrame and keeps the cache") {
    p.ee_inputs[0] = ee_input("no_such_frame", 0.3);
    CHECK(apply(node, p, kNoTransform) == Status::kUnknownFrame);
    CHECK(node.costs[0].frame_name == std::string_view(kLink7));
    CHECK(c.residual<crocoddyl::ResidualModelFramePlacement>("placement")
              .get_id() == c.link7);
  }
  SUBCASE("not exactly one end effector is kSizeMismatch") {
    p.ee_inputs.push_back(ee_input(kTcp, 0.3));
    CHECK(apply(node, p, kNoTransform) == Status::kSizeMismatch);
    p.ee_inputs.clear();
    CHECK(apply(node, p, kNoTransform) == Status::kSizeMismatch);
  }
}

TEST_CASE("CostUpdater: a non-weighted-quad activation keeps its weights") {
  Costs c;
  auto node =
      node_of(c.updater(UpdateRule::kFramePlacement, "placement_quad", kTcp));
  CHECK(node.costs[0].activation == nullptr);
  auto p = point(*c.state, 0.5);
  p.ee_inputs.push_back(ee_input(kTcp, 0.3));
  p.ee_inputs[0].w_pose.setZero();  // ignored: no weights to write
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
  CHECK(c.residual<crocoddyl::ResidualModelFramePlacement>("placement_quad")
            .get_reference()
            .isApprox(p.ee_inputs[0].pose));
  // Python only updates WeightedQuad weights, so B10 does not apply either.
  CHECK(c.item("placement_quad").active);
}

TEST_CASE("CostUpdater: kVisualServoing") {
  Costs c;
  auto node =
      node_of(c.updater(UpdateRule::kVisualServoing, "vs", kTcp, false, 1));
  CHECK(node.costs[0].frame_name == std::string_view("panda_hand_tcp_vs"));
  auto p = point(*c.state, 0.5);
  p.ee_inputs.push_back(ee_input("panda_hand_tcp_vs", 0.3));
  const pinocchio::SE3& target = p.ee_inputs[0].pose;
  InputTransforms transforms(2);
  auto& residual = c.residual<crocoddyl::ResidualModelFramePlacement>("vs");

  SUBCASE("cleared transform with weights: kMissingTransform") {
    const auto before = residual.get_reference();
    CHECK(apply(node, p, transforms) == Status::kMissingTransform);
    CHECK(residual.get_reference().isApprox(before));
  }
  SUBCASE("cleared transform with zero weights: the target as is") {
    p.ee_inputs[0].w_pose.setZero();
    REQUIRE(apply(node, p, transforms) == Status::kOk);
    CHECK(residual.get_reference().isApprox(target));
    CHECK_FALSE(c.item("vs").active);  // B10
  }
  SUBCASE("with a transform: T * target") {
    const pinocchio::SE3 t(Eigen::Matrix3d::Identity(),
                           Eigen::Vector3d(1, 0, 0));
    transforms[1] = t;
    REQUIRE(apply(node, p, transforms) == Status::kOk);
    CHECK(residual.get_reference().isApprox(t * target));
    CHECK(c.weights("vs") == Eigen::VectorXd(p.ee_inputs[0].w_pose));
  }
  SUBCASE("no robot_frame_vs input: kUnknownFrame") {
    p.ee_inputs[0].frame_name = FrameName(kTcp);
    transforms[1] = pinocchio::SE3::Identity();
    CHECK(apply(node, p, transforms) == Status::kUnknownFrame);
  }
  SUBCASE("transform index out of range: kMissingTransform") {
    CHECK(apply(node, p, InputTransforms(1)) == Status::kMissingTransform);
  }
}

TEST_CASE("CostUpdater: kCollisionWeight writes the item's weight") {
  Costs c;
  auto node = node_of(c.updater(UpdateRule::kCollisionWeight, "collision"));
  auto p = point(*c.state, 0.5);
  p.w_collision_avoidance = 3.5;
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
  CHECK(c.item("collision").weight == 3.5);
  CHECK(c.item("collision").active);
}

TEST_CASE("CostUpdater: a point of the wrong size is kSizeMismatch") {
  Costs c;
  NodeUpdater node;
  node.costs.push_back(c.updater(UpdateRule::kState, "state"));
  node.costs.push_back(c.updater(UpdateRule::kControl, "control"));
  auto wrong = make_trajectory_point(8, 8);
  CHECK(apply(node, wrong, kNoTransform) == Status::kSizeMismatch);
  node.costs.erase(node.costs.begin());
  CHECK(apply(node, wrong, kNoTransform) == Status::kSizeMismatch);
}

TEST_CASE("make_cost_updater: setup errors name the cost") {
  Costs c;
  CHECK_THROWS_WITH_AS(c.updater(UpdateRule::kState, "nope"),
                       doctest::Contains("'nope'"), std::invalid_argument);
  CHECK_THROWS_WITH_AS(c.updater(UpdateRule::kState, "control"),
                       doctest::Contains("'control'"), std::invalid_argument);
  CHECK_THROWS_WITH_AS(
      c.updater(UpdateRule::kFramePlacement, "placement", kLink7),
      doctest::Contains("'placement'"), std::invalid_argument);
  CHECK_THROWS_WITH_AS(c.updater(UpdateRule::kState, "state", {}, true),
                       doctest::Contains("'state'"), std::invalid_argument);
  CHECK_THROWS_WITH_AS(
      c.updater(UpdateRule::kFramePlacement, "placement", kTcp, true),
      doctest::Contains("'placement'"), std::invalid_argument);
  CHECK_THROWS_WITH_AS(c.updater(UpdateRule::kState, "state", kTcp),
                       doctest::Contains("'state'"), std::invalid_argument);
  CHECK_THROWS_WITH_AS(c.updater(UpdateRule::kSoftContactForce, "state"),
                       doctest::Contains("P8"), std::invalid_argument);
}

// ---- B10: zero weight disables the cost -----------------------------------

TEST_CASE("CostUpdater B10: zero weights deactivate, non-zero reactivate") {
  Costs c;
  NodeUpdater node;
  node.costs.push_back(c.updater(UpdateRule::kState, "state"));
  node.costs.push_back(c.updater(UpdateRule::kFrameRotation, "rotation", kTcp));
  node.costs.push_back(c.updater(UpdateRule::kCollisionWeight, "collision"));
  auto p = point(*c.state, 0.5);
  p.ee_inputs.push_back(ee_input(kTcp, 0.3));
  const std::size_t nr = c.sum->get_nr();

  p.w_q.setZero();
  p.w_qdot.setZero();
  p.ee_inputs[0].w_pose.tail<3>().setZero();  // rotation weights only
  p.w_collision_avoidance = 0.0;
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
  CHECK_FALSE(c.item("state").active);
  CHECK_FALSE(c.item("rotation").active);
  CHECK_FALSE(c.item("collision").active);
  // Written through CostItem::active: crocoddyl's bookkeeping is not updated.
  CHECK(c.sum->get_nr() == nr);

  p.w_qdot[3] = 1.0;
  p.ee_inputs[0].w_pose[5] = 1.0;
  p.w_collision_avoidance = 1.0;
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
  CHECK(c.item("state").active);
  CHECK(c.item("rotation").active);
  CHECK(c.item("collision").active);
}

TEST_CASE("CostUpdater B10: a zero cost at setup starts inactive") {
  Costs c;
  auto& activation = dynamic_cast<crocoddyl::ActivationModelWeightedQuad&>(
      *c.item("state").cost->get_activation());
  activation.set_weights(Eigen::VectorXd::Zero(14));
  c.item("collision").weight = 0.0;
  const auto state = c.updater(UpdateRule::kState, "state");
  const auto collision = c.updater(UpdateRule::kCollisionWeight, "collision");
  CHECK_FALSE(c.item("state").active);
  CHECK_FALSE(c.item("collision").active);
  CHECK(state.toggles_active);
  // At setup crocoddyl's bookkeeping is kept: changeCostStatus is used.
  CHECK(c.sum->get_inactive_set().count("state") == 1);
  CHECK(c.sum->get_inactive_set().count("collision") == 1);

  auto node = node_of(state);
  REQUIRE(apply(node, point(*c.state, 0.5), kNoTransform) == Status::kOk);
  CHECK(c.item("state").active);
}

TEST_CASE("CostUpdater B10: a cost inactive by choice is never activated") {
  Costs c;
  c.sum->changeCostStatus("control", false);
  auto node = node_of(c.updater(UpdateRule::kControl, "control"));
  CHECK_FALSE(node.costs[0].toggles_active);
  const auto p = point(*c.state, 0.5);
  REQUIRE(apply(node, p, kNoTransform) == Status::kOk);
  CHECK_FALSE(c.item("control").active);
  CHECK(c.weights("control") == p.w_robot_effort);  // still updated
}

// ---- RT --------------------------------------------------------------------

TEST_CASE("RtFree.apply: every rule, frame switches and early returns") {
  Costs c;
  Costs traj;  // frames from the trajectory
  NodeUpdater node, from_traj;
  node.costs.push_back(c.updater(UpdateRule::kState, "state"));
  node.costs.push_back(c.updater(UpdateRule::kControl, "control"));
  node.costs.push_back(c.updater(UpdateRule::kControlGrav, "control_grav"));
  node.costs.push_back(
      c.updater(UpdateRule::kFramePlacement, "placement", kTcp));
  node.costs.push_back(
      c.updater(UpdateRule::kFrameTranslation, "translation", kTcp));
  node.costs.push_back(c.updater(UpdateRule::kFrameRotation, "rotation", kTcp));
  node.costs.push_back(c.updater(UpdateRule::kFrameVelocity, "velocity", kTcp));
  node.costs.push_back(
      c.updater(UpdateRule::kFramePlacement, "placement_quad", kTcp));
  node.costs.push_back(
      c.updater(UpdateRule::kVisualServoing, "vs", kTcp, false, 0));
  node.costs.push_back(c.updater(UpdateRule::kCollisionWeight, "collision"));
  from_traj.costs.push_back(
      traj.updater(UpdateRule::kFramePlacement, "placement", {}, true));
  from_traj.costs.push_back(
      traj.updater(UpdateRule::kFrameTranslation, "translation", {}, true));
  from_traj.costs.push_back(
      traj.updater(UpdateRule::kFrameRotation, "rotation", {}, true));
  from_traj.costs.push_back(
      traj.updater(UpdateRule::kFrameVelocity, "velocity", {}, true));

  auto p = point(*c.state, 0.5);
  p.ee_inputs.push_back(ee_input(kTcp, 0.3));
  p.ee_inputs.push_back(ee_input("panda_hand_tcp_vs", 0.4));
  auto zero = p;  // B10: every weight zero
  zero.w_q.setZero();
  zero.w_qdot.setZero();
  zero.w_robot_effort.setZero();
  zero.w_collision_avoidance = 0.0;
  for (auto& ee : zero.ee_inputs) {
    ee.w_pose.setZero();
    ee.w_twist.setZero();
  }
  auto tcp = point(*c.state, 0.5);
  tcp.ee_inputs.push_back(ee_input(kTcp, 0.3));
  auto link7 = tcp;
  link7.ee_inputs[0] = ee_input(kLink7, 0.2);
  auto unknown = tcp;
  unknown.ee_inputs[0] = ee_input("no_such_frame", 0.2);
  auto missing = p;
  missing.ee_inputs[0].frame_name = FrameName(kLink7);
  const auto wrong_size = make_trajectory_point(8, 8);
  InputTransforms transforms(1, pinocchio::SE3::Identity());
  const InputTransforms cleared(1);

  REQUIRE(apply(node, p, transforms) == Status::kOk);  // warm-up
  Status s[10];
  // 11 ticks: the last one is even (p, tcp).
  CHECK_RT_SAFE(for (int tick = 0; tick < 11; ++tick) {
    s[0] = apply(node, tick % 2 ? zero : p, transforms);          // B10 toggles
    s[1] = apply(from_traj, tick % 2 ? link7 : tcp, transforms);  // frame scan
  });
  CHECK(s[0] == Status::kOk);
  CHECK(s[1] == Status::kOk);
  CHECK(c.item("state").active);
  CHECK(traj.residual<crocoddyl::ResidualModelFramePlacement>("placement")
            .get_id() == traj.tcp);
  CHECK_RT_SAFE(s[2] = apply(node, missing, transforms));
  CHECK_RT_SAFE(s[3] = apply(node, p, cleared));
  CHECK_RT_SAFE(s[4] = apply(node, wrong_size, transforms));
  CHECK_RT_SAFE(s[5] = apply(from_traj, unknown, transforms));
  CHECK_RT_SAFE(s[6] = apply(from_traj, p, transforms));
  CHECK(s[2] == Status::kUnknownFrame);
  CHECK(s[3] == Status::kMissingTransform);
  CHECK(s[4] == Status::kSizeMismatch);
  CHECK(s[5] == Status::kUnknownFrame);
  CHECK(s[6] == Status::kSizeMismatch);
}
