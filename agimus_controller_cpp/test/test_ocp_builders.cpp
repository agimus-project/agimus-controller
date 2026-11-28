// build_goal_reaching_models (design.md, "Builders"): port of
// agimus_controller/tests/test_ocp_croco_goal_reaching.py against golden set
// G5, plus the two behaviour changes the builder carries: A4/B12 (the
// terminal cost resolves its own frame) and A6/B14 (state weights of size
// ndx).

#include <algorithm>
#include <cmath>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/solver-base.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <mim_solvers/csqp.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>
#include <vector>

#include "agimus_controller/ocp_builders.hpp"
#include "panda_ocp.hpp"
#include "robot_models_fixtures.hpp"
#include "rt_guard.hpp"

using agimus_controller::build_goal_reaching_models;
using agimus_controller::EndEffectorInput;
using agimus_controller::FrameName;
using agimus_controller::make_trajectory_point;
using agimus_controller::Ocp;
using agimus_controller::OcpModels;
using agimus_controller::OcpParams;
using agimus_controller::RobotModels;
using agimus_controller::Status;
using agimus_controller::WeightedTrajectoryPoint;
using namespace agimus_controller::test;  // fixtures (panda_ocp.hpp)

namespace {

// ---- TOML helpers (golden files) -------------------------------------------

Eigen::VectorXd to_vector(const toml::node& node) {
  const toml::array* a = node.as_array();
  REQUIRE(a != nullptr);
  Eigen::VectorXd v(static_cast<Eigen::Index>(a->size()));
  for (std::size_t i = 0; i < a->size(); ++i) {
    const std::optional<double> x = (*a)[i].value<double>();
    REQUIRE(x.has_value());
    v[static_cast<Eigen::Index>(i)] = *x;
  }
  return v;
}

Eigen::MatrixXd to_matrix(const toml::node& node) {
  const toml::array* rows = node.as_array();
  REQUIRE(rows != nullptr);
  REQUIRE(!rows->empty());
  const Eigen::VectorXd first = to_vector((*rows)[0]);
  Eigen::MatrixXd m(static_cast<Eigen::Index>(rows->size()), first.size());
  for (std::size_t i = 0; i < rows->size(); ++i) {
    m.row(static_cast<Eigen::Index>(i)) = to_vector((*rows)[i]).transpose();
  }
  return m;
}

std::vector<Eigen::VectorXd> to_vectors(const toml::node& node) {
  std::vector<Eigen::VectorXd> out;
  for (const toml::node& row : *node.as_array()) out.push_back(to_vector(row));
  return out;
}

template <typename T>
T value(const toml::table& t, const char* key) {
  const std::optional<T> v = t[key].value<T>();
  REQUIRE_MESSAGE(v.has_value(), "missing value " << key);
  return *v;
}

double max_abs_diff(const std::vector<Eigen::VectorXd>& a,
                    const std::vector<Eigen::VectorXd>& b) {
  REQUIRE(a.size() == b.size());
  double d = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    REQUIRE(a[i].size() == b[i].size());
    d = std::max(d, (a[i] - b[i]).cwiseAbs().maxCoeff());
  }
  return d;
}

// ---- Access to the built nodes ---------------------------------------------

const crocoddyl::CostItem& cost_item(const crocoddyl::ActionModelAbstract& node,
                                     const std::string& name) {
  return *costs_of(node).get_costs().at(name);
}

const crocoddyl::ResidualModelFramePlacement& placement_residual(
    const crocoddyl::ActionModelAbstract& node) {
  return dynamic_cast<const crocoddyl::ResidualModelFramePlacement&>(
      *cost_item(node, "goalTracking").cost->get_residual());
}

const Eigen::VectorXd& weights_of(const crocoddyl::ActionModelAbstract& node,
                                  const std::string& name) {
  return dynamic_cast<const crocoddyl::ActivationModelWeightedQuad&>(
             *cost_item(node, name).cost->get_activation())
      .get_weights();
}

WeightedTrajectoryPoint goal_point(const pinocchio::Model& model,
                                   const std::string& frame, double x) {
  auto p = make_trajectory_point(model.nq, model.nv);
  p.q = pinocchio::neutral(model);
  p.w_q.setConstant(0.01);
  p.w_qdot.setConstant(0.01);
  p.w_robot_effort.setConstant(1e-4);
  EndEffectorInput ee;
  ee.frame_name = FrameName(frame);
  ee.pose =
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, 0.2, 0.5));
  ee.w_pose.setConstant(1e3);
  p.ee_inputs.push_back(ee);
  return p;
}

}  // namespace

// ---- G5: test_ocp_croco_goal_reaching.py -----------------------------------

TEST_CASE("build_goal_reaching_models: golden set G5") {
  const toml::table golden =
      toml::parse_file(AGIMUS_GOLDEN_DIR "/g5_ocp_goal_reaching.toml");
  const toml::table& tp = *golden["ocp_params"].as_table();
  const toml::table& inputs = *golden["inputs"].as_table();
  const toml::table& ref = *inputs["reference"].as_table();
  const toml::table& outputs = *golden["outputs"].as_table();

  OcpParams params;
  params.dt = value<double>(tp, "dt");
  params.solver_iters = static_cast<int>(value<int64_t>(tp, "solver_iters"));
  params.dt_factor_n_seq.factors = {
      static_cast<int>(*(*tp["factors"].as_array())[0].value<int64_t>())};
  params.dt_factor_n_seq.n_steps = {
      static_cast<int>(*(*tp["n_steps"].as_array())[0].value<int64_t>())};
  params.horizon_size = static_cast<int>(value<int64_t>(tp, "horizon_size"));
  params.qp_iters = static_cast<int>(value<int64_t>(tp, "qp_iters"));
  params.termination_tolerance = value<double>(tp, "termination_tolerance");
  params.eps_abs = value<double>(tp, "eps_abs");
  params.eps_rel = value<double>(tp, "eps_rel");
  params.callbacks = value<bool>(tp, "callbacks");
  params.use_debug_data = value<bool>(tp, "use_debug_data");
  params.use_filter_line_search = value<bool>(tp, "use_filter_line_search");
  params.n_threads = static_cast<int>(value<int64_t>(tp, "n_threads"));
  params.finalize();
  REQUIRE(params.n_controls == value<int64_t>(tp, "n_controls"));

  const pinocchio::Model& model = panda().robot_model();
  auto point = make_trajectory_point(model.nq, model.nv);
  point.q = to_vector(*ref["q"].node());
  point.qdot = to_vector(*ref["v"].node());
  point.robot_effort = to_vector(*ref["tau"].node());
  point.w_q = to_vector(*ref["w_q"].node());
  point.w_qdot = to_vector(*ref["w_v"].node());
  point.w_robot_effort = to_vector(*ref["w_tau"].node());
  EndEffectorInput ee;
  ee.frame_name = FrameName(value<std::string>(ref, "ee_frame"));
  const toml::table& pose = *ref["ee_pose"].as_table();
  ee.pose = pinocchio::SE3(to_matrix(*pose["rotation"].node()),
                           to_vector(*pose["translation"].node()));
  ee.w_pose = to_vector(*ref["w_ee"].node());
  point.ee_inputs.push_back(ee);
  const std::vector<WeightedTrajectoryPoint> points(
      static_cast<std::size_t>(value<int64_t>(ref, "n_points")), point);

  Ocp ocp(build_goal_reaching_models(panda(), params), params);
  CHECK(ocp.set_reference_weighted_trajectory(pointers(points)) == Status::kOk);

  const std::size_t n = static_cast<std::size_t>(params.n_controls);
  const Eigen::VectorXd x0 = to_vector(*inputs["x0"].node());
  const std::vector<Eigen::VectorXd> xs(
      n + 1, to_vector(*inputs["warm_start_x"].node()));
  const std::vector<Eigen::VectorXd> us(
      n, to_vector(*inputs["warm_start_u"].node()));
  const bool solved = ocp.solve(x0, xs, us);

  const auto& results = ocp.results();
  const double tol = 1e-6;  // design.md, Golden files: solver outputs
  const double d_xs =
      max_abs_diff(results.states, to_vectors(*outputs["xs"].node()));
  const double d_us = max_abs_diff(results.feed_forward_terms,
                                   to_vectors(*outputs["us"].node()));
  const double d_k =
      (results.riccati_gains.at(0) - to_matrix(*outputs["K0"].node()))
          .cwiseAbs()
          .maxCoeff();
  const auto& debug = ocp.debug_data();
  const double d_kkt = std::abs(debug.kkt_norm - value<double>(outputs, "kkt"));
  MESSAGE("G5 max |C++ - Python|: xs " << d_xs << ", us " << d_us << ", K0 "
                                       << d_k << ", kkt " << d_kkt);
  CHECK(d_xs <= tol);
  CHECK(d_us <= tol);
  CHECK(d_k <= tol);
  CHECK(d_kkt <= tol);
  CHECK(solved == value<bool>(outputs, "problem_solved"));
  CHECK(debug.nb_iter == value<int64_t>(outputs, "iterations"));
  CHECK(debug.nb_qp_iter == value<int64_t>(outputs, "qp_iterations"));

  // test_ocp_solution: the last end-effector position is near the target.
  pinocchio::Data data(model);
  pinocchio::framesForwardKinematics(model, data,
                                     results.states.back().head(model.nq));
  CHECK((data.oMf[model.getFrameId(kTcp)].translation() - ee.pose.translation())
            .norm() < 0.1);
}

// ---- Structure --------------------------------------------------------------

TEST_CASE("build_goal_reaching_models: structure") {
  const auto params = make_params(5);
  const OcpModels models = build_goal_reaching_models(panda(), params);
  REQUIRE(models.running.size() == 5);
  REQUIRE(models.running_updaters.size() == 5);
  CHECK(models.running_updaters[0].costs.size() == 3);
  CHECK(models.terminal_updater.costs.size() == 2);
  CHECK(models.geometry == nullptr);
  CHECK(models.input_transforms.empty());

  const auto step = [](const crocoddyl::ActionModelAbstract& node) {
    return dynamic_cast<const crocoddyl::IntegratedActionModelEuler&>(node)
        .get_dt();
  };
  const auto armature = [](const crocoddyl::ActionModelAbstract& node) {
    const auto& euler =
        dynamic_cast<const crocoddyl::IntegratedActionModelEuler&>(node);
    return dynamic_cast<
               const crocoddyl::DifferentialActionModelFreeFwdDynamics&>(
               *euler.get_differential())
        .get_armature();
  };
  for (std::size_t i = 0; i < 5; ++i) {
    if (i > 0) CHECK(models.running[i] != models.running[0]);
    CHECK(step(*models.running[i]) == params.timesteps[i]);
    CHECK(armature(*models.running[i]) == panda().armature());
    const auto& costs = costs_of(*models.running[i]).get_costs();
    CHECK(costs.size() == 3);
    CHECK(costs.count("stateReg") == 1);
    CHECK(costs.count("ctrlReg") == 1);
    CHECK(costs.count("goalTracking") == 1);
  }
  CHECK(step(*models.terminal) == 0.0);
  CHECK(armature(*models.terminal) == panda().armature());
  const auto& terminal_costs = costs_of(*models.terminal).get_costs();
  CHECK(terminal_costs.size() == 2);
  CHECK(terminal_costs.count("ctrlReg") == 0);
}

TEST_CASE("build_goal_reaching_models: setup errors") {
  OcpParams raw;
  CHECK_THROWS_WITH_AS(build_goal_reaching_models(panda(), raw),
                       doctest::Contains("finalized"), std::invalid_argument);
}

// ---- Ticks ------------------------------------------------------------------

TEST_CASE("build_goal_reaching_models: 20 ticks of reference and solve") {
  const auto params = make_params(kN, 5);
  Ocp ocp(build_goal_reaching_models(panda(), params), params);
  const pinocchio::Model& model = panda().robot_model();
  const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  const std::vector<Eigen::VectorXd> xs(kN + 1, x0);
  const std::vector<Eigen::VectorXd> us(kN, Eigen::VectorXd::Zero(model.nv));
  auto points = [&](int tick) {
    std::vector<WeightedTrajectoryPoint> pts;
    for (int i = 0; i <= kN; ++i) {
      pts.push_back(goal_point(model, kTcp, 0.3 + 0.001 * (tick + i)));
    }
    return pts;
  };
  REQUIRE(ocp.set_reference_weighted_trajectory(pointers(points(0))) ==
          Status::kOk);
  ocp.solve_without_limits(x0, xs, us);  // warm-up (not RT: third party)
  for (int tick = 0; tick < 20; ++tick) {
    const auto pts = points(tick);
    const auto ptrs = pointers(pts);
    CHECK_RT_SAFE((void)ocp.set_reference_weighted_trajectory(ptrs));
    ocp.solve(x0, xs, us);
    const auto& r = ocp.results();
    REQUIRE(r.states.size() == kN + 1);
    for (const auto& u : r.feed_forward_terms) REQUIRE(u.allFinite());
    for (const auto& x : r.states) REQUIRE(x.allFinite());
  }
  // The last reference reached the nodes.
  const auto& node = *ocp.problem()->get_runningModels()[3];
  CHECK(placement_reference(node, "goalTracking").translation().x() ==
        doctest::Approx(0.3 + 0.001 * (19 + 3)));
}

// ---- A4 / B12: the terminal node resolves its own frame ---------------------

TEST_CASE("build_goal_reaching_models: A4, terminal frame (B12)") {
  const auto params = make_params(4);
  Ocp ocp(build_goal_reaching_models(panda(), params), params);
  const pinocchio::Model& model = panda().robot_model();
  auto pts = std::vector<WeightedTrajectoryPoint>();
  for (int i = 0; i <= 4; ++i) pts.push_back(goal_point(model, kTcp, 0.3));
  pts.back() = goal_point(model, kLink7, 0.4);  // another frame at the end

  REQUIRE(ocp.set_reference_weighted_trajectory(pointers(pts)) == Status::kOk);
  const auto& problem = *ocp.problem();
  const auto tcp = model.getFrameId(kTcp);
  const auto link7 = model.getFrameId(kLink7);
  REQUIRE(tcp != link7);
  CHECK(placement_residual(*problem.get_runningModels()[3]).get_id() == tcp);
  // C++: the terminal node follows its own point. Python would keep the id
  // of the last running node (tcp, ocp_croco_goal_reaching.py:206).
  CHECK(placement_residual(*problem.get_terminalModel()).get_id() == link7);
  CHECK(placement_residual(*problem.get_terminalModel())
            .get_reference()
            .translation()
            .x() == doctest::Approx(0.4));
}

// ---- A6 / B14: state weights of size ndx ------------------------------------

TEST_CASE("build_goal_reaching_models: A6, state weights of size ndx (B14)") {
  // A free flyer has nq = nv + 1. Python builds stateReg with nq + nv weights
  // while the residual has ndx = 2 nv rows: crocoddyl refuses the cost
  // (activation and residual sizes differ), so Python cannot build the OCP.
  using namespace robot_models_test;
  auto rp = tiny_params();
  rp.free_flyer = true;
  rp.moving_joint_names = {"root_joint", "j1", "j2"};
  rp.armature = Eigen::VectorXd::Constant(8, 0.1);
  const RobotModels robot(rp);
  const pinocchio::Model& model = robot.robot_model();
  REQUIRE(model.nq == 7 + 2);
  REQUIRE(model.nv == 6 + 2);

  const auto params = make_params(4, 3);
  OcpModels models = build_goal_reaching_models(robot, params);
  CHECK(weights_of(*models.running[0], "stateReg").size() == 2 * model.nv);
  CHECK(weights_of(*models.terminal, "stateReg").size() == 2 * model.nv);
  CHECK(weights_of(*models.running[0], "ctrlReg").size() == model.nv);

  Ocp ocp(std::move(models), params);
  std::vector<WeightedTrajectoryPoint> pts;
  for (int i = 0; i <= 4; ++i) {
    auto p = goal_point(model, "link2", 0.0);
    p.w_q.setConstant(0.5);  // size nv = 8, not nq = 9
    p.w_qdot.setConstant(0.25);
    pts.push_back(p);
  }
  REQUIRE(ocp.set_reference_weighted_trajectory(pointers(pts)) == Status::kOk);
  const Eigen::VectorXd w =
      weights_of(*ocp.problem()->get_runningModels()[0], "stateReg");
  CHECK(w.head(model.nv) == Eigen::VectorXd::Constant(model.nv, 0.5));
  CHECK(w.tail(model.nv) == Eigen::VectorXd::Constant(model.nv, 0.25));

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  x0.head(model.nq) = pinocchio::neutral(model);
  const std::vector<Eigen::VectorXd> xs(5, x0);
  const std::vector<Eigen::VectorXd> us(4, Eigen::VectorXd::Zero(model.nv));
  ocp.solve(x0, xs, us);
  for (const auto& x : ocp.results().states) CHECK(x.allFinite());
}
