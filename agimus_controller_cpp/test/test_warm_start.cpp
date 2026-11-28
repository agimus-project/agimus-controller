// WarmStart, WarmStartReference and WarmStartShiftPreviousSolution
// (design.md, "Warm starts"): ports of test_warm_start_base.py,
// test_warm_start_reference.py and test_warm_start_shift_previous_reference.py,
// golden sets G2 and G3, audit A13/B18, and the RT tests.

#include <algorithm>
#include <cmath>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>
#include <type_traits>
#include <vector>

#include "agimus_controller/warm_start.hpp"
#include "panda_ocp.hpp"
#include "robot_models_fixtures.hpp"
#include "rt_guard.hpp"

using agimus_controller::make_robot_state;
using agimus_controller::make_trajectory_point;
using agimus_controller::OcpParams;
using agimus_controller::OcpResults;
using agimus_controller::RobotState;
using agimus_controller::Status;
using agimus_controller::WarmStart;
using agimus_controller::WarmStartReference;
using agimus_controller::WarmStartShiftPreviousSolution;
using agimus_controller::WeightedTrajectoryPoint;
using namespace agimus_controller::test;  // fixtures (panda_ocp.hpp)
using robot_models_test::ur5_model;

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

std::vector<Eigen::VectorXd> to_vectors(const toml::node& node) {
  const toml::array* rows = node.as_array();
  REQUIRE(rows != nullptr);
  std::vector<Eigen::VectorXd> out;
  for (const toml::node& row : *rows) out.push_back(to_vector(row));
  return out;
}

template <typename T>
T value(const toml::table& t, const char* key) {
  const std::optional<T> v = t[key].value<T>();
  REQUIRE_MESSAGE(v.has_value(), "missing value " << key);
  return *v;
}

std::vector<int> to_ints(const toml::node& node) {
  std::vector<int> out;
  for (const toml::node& n : *node.as_array()) {
    out.push_back(static_cast<int>(*n.value<int64_t>()));
  }
  return out;
}

// ---- Helpers ---------------------------------------------------------------

std::vector<Eigen::VectorXd> zeros(std::size_t n, Eigen::Index size) {
  return std::vector<Eigen::VectorXd>(n, Eigen::VectorXd::Zero(size));
}

double max_diff(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  REQUIRE(a.size() == b.size());
  return (a - b).cwiseAbs().maxCoeff();
}

OcpParams make_shift_params(double dt, std::vector<int> factors,
                            std::vector<int> n_steps) {
  OcpParams p;
  p.dt = dt;
  p.solver_iters = 10;
  p.dt_factor_n_seq = {std::move(factors), std::move(n_steps)};
  int horizon = 0;
  for (const int n : p.dt_factor_n_seq.n_steps) horizon += n;
  p.horizon_size = horizon;
  p.finalize();
  return p;
}

/// A free-forward-dynamics Euler node with no costs and the panda's armature,
/// as the OCP's running models.
std::shared_ptr<crocoddyl::ActionModelAbstract> bare_node(double dt) {
  return make_node(panda_state(),
                   std::make_shared<crocoddyl::CostModelSum>(panda_state()),
                   dt);
}

/// One step of a bare node: the xnext of `calc(x, u)`.
Eigen::VectorXd step(double dt, const Eigen::VectorXd& x,
                     const Eigen::VectorXd& u) {
  const auto node = bare_node(dt);
  const auto data = node->createData();
  node->calc(data, x, u);
  return data->xnext;
}

/// A previous solution: the rollout of `us` from x = 0 through nodes of the
/// given steps; gains are zeros.
OcpResults rollout(const OcpParams& params, std::vector<Eigen::VectorXd> us) {
  const auto& model = panda().robot_model();
  OcpResults r;
  r.resize(params.timesteps.size(), model.nq + model.nv, 2 * model.nv,
           model.nv);
  r.states[0].setZero();
  for (std::size_t i = 0; i < params.timesteps.size(); ++i) {
    r.states[i + 1] = step(params.timesteps[i], r.states[i], us[i]);
    r.feed_forward_terms[i] = us[i];
  }
  return r;
}

std::vector<Eigen::VectorXd> random_vectors(std::size_t n, Eigen::Index size) {
  std::vector<Eigen::VectorXd> out;
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back(Eigen::VectorXd::Random(size));
  }
  return out;
}

RobotState state_of(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                    const Eigen::VectorXd& a) {
  RobotState s = make_robot_state(q.size(), v.size());
  s.q = q;
  s.v = v;
  s.a = a;
  return s;
}

}  // namespace

// ---- test_warm_start_base.py -----------------------------------------------

TEST_CASE("WarmStart is abstract") {
  static_assert(std::is_abstract_v<WarmStart>);
  static_assert(!std::is_abstract_v<WarmStartReference>);
  static_assert(!std::is_abstract_v<WarmStartShiftPreviousSolution>);
}

TEST_CASE("WarmStart.update_previous_solution: default does nothing") {
  WarmStartReference ws(panda().robot_model());
  OcpResults r;
  ws.update_previous_solution(r);
  WarmStart& base = ws;
  base.update_previous_solution(r);
}

// ---- test_warm_start_reference.py ------------------------------------------

TEST_CASE("WarmStartReference: initialization") {
  WarmStartReference ws(ur5_model());
  CHECK(true);
}

TEST_CASE("WarmStartReference.generate: random reference on the ur5") {
  const pinocchio::Model& model = ur5_model();
  WarmStartReference ws(model);
  pinocchio::Data data(model);
  constexpr int kPoints = 10;
  const RobotState initial = state_of(Eigen::VectorXd::Random(model.nq),
                                      Eigen::VectorXd::Random(model.nv),
                                      Eigen::VectorXd::Random(model.nv));
  std::vector<WeightedTrajectoryPoint> points;
  for (int i = 0; i < kPoints; ++i) {
    auto p = make_trajectory_point(model.nq, model.nv);
    p.q = Eigen::VectorXd::Random(model.nq);
    p.qdot = Eigen::VectorXd::Random(model.nv);
    p.qddot = Eigen::VectorXd::Random(model.nv);
    points.push_back(p);
  }
  const auto horizon = pointers(points);

  Eigen::VectorXd expected_x0(model.nq + model.nv);
  expected_x0 << initial.q, initial.v;
  std::vector<Eigen::VectorXd> expected_xs{expected_x0};
  for (int i = 1; i < kPoints; ++i) {
    Eigen::VectorXd x(model.nq + model.nv);
    x << points[i].q, points[i].qdot;
    expected_xs.push_back(x);
  }
  std::vector<Eigen::VectorXd> expected_us{
      pinocchio::rnea(model, data, initial.q, initial.v, initial.a)};
  for (int i = 1; i < kPoints - 1; ++i) {
    expected_us.push_back(pinocchio::rnea(model, data, points[i].q,
                                          points[i].qdot, points[i].qddot));
  }

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  auto xs = zeros(kPoints, model.nq + model.nv);
  auto us = zeros(kPoints - 1, model.nv);
  REQUIRE(ws.generate(initial, horizon, x0, xs, us) == Status::kOk);

  REQUIRE(xs.size() == expected_xs.size());
  REQUIRE(us.size() == expected_us.size());
  CHECK(x0 == expected_x0);
  for (std::size_t i = 0; i < xs.size(); ++i) CHECK(xs[i] == expected_xs[i]);
  for (std::size_t i = 0; i < us.size(); ++i) CHECK(us[i] == expected_us[i]);
}

TEST_CASE("WarmStartReference.generate: refuses wrong sizes, writes nothing") {
  const pinocchio::Model& model = panda().robot_model();
  WarmStartReference ws(model);
  const auto points = make_points(model, 4, Frames::kFixed);
  const auto horizon = pointers(points);  // 5 points
  const RobotState state = make_robot_state(model.nq, model.nv);
  const Eigen::Index nx = model.nq + model.nv;
  Eigen::VectorXd x0 = Eigen::VectorXd::Constant(nx, 7.0);
  auto xs = std::vector<Eigen::VectorXd>(5, Eigen::VectorXd::Constant(nx, 7.0));
  auto us =
      std::vector<Eigen::VectorXd>(4, Eigen::VectorXd::Constant(model.nv, 7.0));
  const auto untouched = [&] {
    return x0.isApproxToConstant(7.0) && xs[0].isApproxToConstant(7.0) &&
           us[0].isApproxToConstant(7.0);
  };

  auto short_xs = xs;
  short_xs.pop_back();
  CHECK(ws.generate(state, horizon, x0, short_xs, us) == Status::kSizeMismatch);
  auto short_us = us;
  short_us.pop_back();
  CHECK(ws.generate(state, horizon, x0, xs, short_us) == Status::kSizeMismatch);
  auto bad_x = xs;
  bad_x[2] = Eigen::VectorXd::Zero(3);
  CHECK(ws.generate(state, horizon, x0, bad_x, us) == Status::kSizeMismatch);
  auto bad_u = us;
  bad_u[1] = Eigen::VectorXd::Zero(3);
  CHECK(ws.generate(state, horizon, x0, xs, bad_u) == Status::kSizeMismatch);
  Eigen::VectorXd bad_x0 = Eigen::VectorXd::Zero(3);
  CHECK(ws.generate(state, horizon, bad_x0, xs, us) == Status::kSizeMismatch);
  RobotState bad_state = state;
  bad_state.q = Eigen::VectorXd::Zero(3);
  CHECK(ws.generate(bad_state, horizon, x0, xs, us) == Status::kSizeMismatch);
  CHECK(ws.generate(state, {}, x0, xs, us) == Status::kSizeMismatch);
  auto with_null = horizon;
  with_null[2] = nullptr;
  CHECK(ws.generate(state, with_null, x0, xs, us) == Status::kSizeMismatch);
  auto bad_points = points;
  bad_points[3].qdot = Eigen::VectorXd::Zero(3);
  CHECK(ws.generate(state, pointers(bad_points), x0, xs, us) ==
        Status::kSizeMismatch);
  CHECK(untouched());
}

TEST_CASE("WarmStartReference.generate: horizon of one point has no control") {
  const pinocchio::Model& model = panda().robot_model();
  WarmStartReference ws(model);
  const auto points = make_points(model, 0, Frames::kFixed);
  RobotState state = make_robot_state(model.nq, model.nv);
  state.q.setConstant(0.3);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  auto xs = zeros(1, model.nq + model.nv);
  std::vector<Eigen::VectorXd> us;
  REQUIRE(ws.generate(state, pointers(points), x0, xs, us) == Status::kOk);
  CHECK(xs[0] == x0);
  CHECK(x0.head(model.nq) == state.q);
}

// ---- G2
// ----------------------------------------------------------------------

TEST_CASE("WarmStartReference.generate: golden set G2") {
  const toml::table golden =
      toml::parse_file(AGIMUS_GOLDEN_DIR "/g2_warm_start_reference.toml");
  const toml::table& inputs = *golden["inputs"].as_table();
  const toml::table& init = *inputs["initial_state"].as_table();
  const toml::table& ref = *inputs["reference"].as_table();
  const toml::table& out = *golden["outputs"].as_table();

  const pinocchio::Model& model = ur5_model();
  REQUIRE(model.nq == value<int64_t>(*golden["robot"].as_table(), "nq"));
  const RobotState state =
      state_of(to_vector(*init["q"].node()), to_vector(*init["v"].node()),
               to_vector(*init["a"].node()));
  const auto qs = to_vectors(*ref["q"].node());
  const auto vs = to_vectors(*ref["v"].node());
  const auto as = to_vectors(*ref["a"].node());
  std::vector<WeightedTrajectoryPoint> points;
  for (std::size_t i = 0; i < qs.size(); ++i) {
    auto p = make_trajectory_point(model.nq, model.nv);
    p.q = qs[i];
    p.qdot = vs[i];
    p.qddot = as[i];
    points.push_back(p);
  }
  const auto want_x0 = to_vector(*out["x0"].node());
  const auto want_xs = to_vectors(*out["xs"].node());
  const auto want_us = to_vectors(*out["us"].node());
  REQUIRE(want_xs.size() == points.size());
  REQUIRE(want_us.size() == points.size() - 1);

  WarmStartReference ws(model);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  auto xs = zeros(want_xs.size(), model.nq + model.nv);
  auto us = zeros(want_us.size(), model.nv);
  REQUIRE(ws.generate(state, pointers(points), x0, xs, us) == Status::kOk);

  double worst = max_diff(x0, want_x0);
  for (std::size_t i = 0; i < xs.size(); ++i) {
    worst = std::max(worst, max_diff(xs[i], want_xs[i]));
  }
  for (std::size_t i = 0; i < us.size(); ++i) {
    worst = std::max(worst, max_diff(us[i], want_us[i]));
  }
  MESSAGE("G2 max abs error: " << worst);
  CHECK(worst < 1e-10);
}

// ---- test_warm_start_shift_previous_reference.py ----------------------------

TEST_CASE("WarmStartShiftPreviousSolution.generate: no previous solution") {
  const auto params = make_shift_params(0.1, {1, 2}, {2, 1});
  WarmStartShiftPreviousSolution ws(panda(), params);
  const auto& model = panda().robot_model();
  const RobotState state = make_robot_state(model.nq, model.nv);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  auto xs = zeros(4, model.nq + model.nv);
  auto us = zeros(3, model.nv);
  CHECK(ws.generate(state, {}, x0, xs, us) == Status::kNoPreviousSolution);
  CHECK(x0.isZero());
}

TEST_CASE("WarmStartShiftPreviousSolution.generate: shifts the rollout") {
  const auto params = make_shift_params(0.1, {1, 2}, {2, 1});
  REQUIRE(params.n_controls == 3);
  REQUIRE(params.timesteps == std::vector<double>{0.1, 0.1, 0.2});
  const auto& model = panda().robot_model();
  constexpr std::size_t nu = 3;
  const Eigen::Index nx = model.nq + model.nv;

  WarmStartShiftPreviousSolution ws(panda(), params);
  const auto controls = random_vectors(nu, model.nv);
  const OcpResults prev = rollout(params, controls);
  // Sanity check: the rollout is the node's xnext.
  for (std::size_t i = 0; i < nu; ++i) {
    CHECK(prev.states[i + 1] ==
          step(params.timesteps[i], prev.states[i], controls[i]));
  }
  ws.update_previous_solution(prev);

  const RobotState state = state_of(Eigen::VectorXd::Constant(model.nq, -10.0),
                                    Eigen::VectorXd::Constant(model.nv, 100.0),
                                    Eigen::VectorXd::Zero(model.nv));
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);
  auto xs = zeros(nu + 1, nx);
  auto us = zeros(nu, model.nv);
  REQUIRE(ws.generate(state, {}, x0, xs, us) == Status::kOk);

  CHECK(x0.head(model.nq) == state.q);
  CHECK(x0.tail(model.nv) == state.v);
  // Integrate each point and compare. The first control is not in the
  // result (discarded).
  for (std::size_t i = 1; i < nu; ++i) {
    CHECK(us[i - 1] == controls[i]);
    CHECK(xs[i] == step(params.dt, prev.states[i], controls[i]));
  }
  CHECK(us[nu - 1] == controls[nu - 1]);
  CHECK(xs[0] == prev.states[1]);
}

TEST_CASE("WarmStartShiftPreviousSolution.generate: size errors") {
  const auto params = make_shift_params(0.1, {1, 2}, {2, 1});
  WarmStartShiftPreviousSolution ws(panda(), params);
  const auto& model = panda().robot_model();
  const Eigen::Index nx = model.nq + model.nv;
  ws.update_previous_solution(rollout(params, random_vectors(3, model.nv)));
  const RobotState state = make_robot_state(model.nq, model.nv);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);
  auto xs = zeros(4, nx);
  auto us = zeros(3, model.nv);

  auto short_xs = xs;
  short_xs.pop_back();
  CHECK(ws.generate(state, {}, x0, short_xs, us) == Status::kSizeMismatch);
  auto long_us = us;
  long_us.push_back(Eigen::VectorXd::Zero(model.nv));
  CHECK(ws.generate(state, {}, x0, xs, long_us) == Status::kSizeMismatch);
  auto bad_x = xs;
  bad_x[3] = Eigen::VectorXd::Zero(2);
  CHECK(ws.generate(state, {}, x0, bad_x, us) == Status::kSizeMismatch);
  auto bad_u = us;
  bad_u[0] = Eigen::VectorXd::Zero(2);
  CHECK(ws.generate(state, {}, x0, xs, bad_u) == Status::kSizeMismatch);
  Eigen::VectorXd bad_x0 = Eigen::VectorXd::Zero(2);
  CHECK(ws.generate(state, {}, bad_x0, xs, us) == Status::kSizeMismatch);
  RobotState bad_state = state;
  bad_state.v = Eigen::VectorXd::Zero(2);
  CHECK(ws.generate(bad_state, {}, x0, xs, us) == Status::kSizeMismatch);

  CHECK(ws.generate(state, {}, x0, xs, us) == Status::kOk);
}

TEST_CASE("WarmStartShiftPreviousSolution: constructor and update errors") {
  const auto& model = panda().robot_model();
  OcpParams not_finalized;
  CHECK_THROWS_AS(WarmStartShiftPreviousSolution(panda(), not_finalized),
                  std::invalid_argument);
  auto bad_dt = make_shift_params(0.1, {1, 2}, {2, 1});
  bad_dt.dt = 0.05;
  CHECK_THROWS_AS(WarmStartShiftPreviousSolution(panda(), bad_dt),
                  std::invalid_argument);
  auto shorter = make_shift_params(0.1, {1, 2}, {2, 1});
  shorter.timesteps[2] = 0.05;
  CHECK_THROWS_AS(WarmStartShiftPreviousSolution(panda(), shorter),
                  std::invalid_argument);

  // A solution of other sizes is ignored.
  const auto params = make_shift_params(0.1, {1}, {3});
  WarmStartShiftPreviousSolution ws(panda(), params);
  OcpResults wrong;
  wrong.resize(2, model.nq + model.nv, 2 * model.nv, model.nv);
  ws.update_previous_solution(wrong);
  const RobotState state = make_robot_state(model.nq, model.nv);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  auto xs = zeros(4, model.nq + model.nv);
  auto us = zeros(3, model.nv);
  CHECK(ws.generate(state, {}, x0, xs, us) == Status::kNoPreviousSolution);
}

TEST_CASE("WarmStartShiftPreviousSolution: keeps its own copy") {
  const auto params = make_shift_params(0.1, {1}, {3});
  const auto& model = panda().robot_model();
  WarmStartShiftPreviousSolution ws(panda(), params);
  OcpResults prev = rollout(params, random_vectors(3, model.nv));
  const OcpResults saved = prev;
  ws.update_previous_solution(prev);
  for (auto& x : prev.states) x.setConstant(1e6);  // caller reuses its storage
  const RobotState state = make_robot_state(model.nq, model.nv);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  auto xs = zeros(4, model.nq + model.nv);
  auto us = zeros(3, model.nv);
  REQUIRE(ws.generate(state, {}, x0, xs, us) == Status::kOk);
  // All steps equal dt: a plain shift, terminal state unchanged.
  CHECK(xs[0] == saved.states[1]);
  CHECK(xs[1] == saved.states[2]);
  CHECK(xs[2] == saved.states[3]);
  CHECK(xs[3] == saved.states[3]);
  CHECK(us[2] == saved.feed_forward_terms[2]);
  CHECK(us[1] == saved.feed_forward_terms[2]);
  CHECK(us[0] == saved.feed_forward_terms[1]);
}

// ---- G3 and A13
// --------------------------------------------------------------

namespace {

struct G3 {
  toml::table golden =
      toml::parse_file(AGIMUS_GOLDEN_DIR "/g3_warm_start_shift.toml");
  OcpParams params;
  RobotState state;
  OcpResults prev;
  Eigen::VectorXd want_x0;
  std::vector<Eigen::VectorXd> want_xs, want_us;

  G3() {
    const toml::table& tp = *golden["ocp_params"].as_table();
    const toml::table& inputs = *golden["inputs"].as_table();
    const toml::table& init = *inputs["initial_state"].as_table();
    const toml::table& out = *golden["outputs"].as_table();
    params = make_shift_params(value<double>(tp, "dt"),
                               to_ints(*tp["factors"].node()),
                               to_ints(*tp["n_steps"].node()));
    REQUIRE(params.n_controls == value<int64_t>(tp, "n_controls"));
    const auto& model = panda().robot_model();
    state = state_of(to_vector(*init["q"].node()), to_vector(*init["v"].node()),
                     Eigen::VectorXd::Zero(model.nv));
    prev.resize(params.timesteps.size(), model.nq + model.nv, 2 * model.nv,
                model.nv);
    prev.states = to_vectors(*inputs["previous_xs"].node());
    prev.feed_forward_terms = to_vectors(*inputs["previous_us"].node());
    want_x0 = to_vector(*out["x0"].node());
    want_xs = to_vectors(*out["xs"].node());
    want_us = to_vectors(*out["us"].node());
  }
};

}  // namespace

TEST_CASE("WarmStartShiftPreviousSolution.generate: golden set G3") {
  G3 g;
  const auto& model = panda().robot_model();
  const Eigen::Index nx = model.nq + model.nv;
  const std::size_t n = g.params.timesteps.size();
  REQUIRE(g.want_xs.size() == n + 1);
  REQUIRE(g.want_us.size() == n);

  WarmStartShiftPreviousSolution ws(panda(), g.params);
  ws.update_previous_solution(g.prev);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);
  auto xs = zeros(n + 1, nx);
  auto us = zeros(n, model.nv);
  REQUIRE(ws.generate(g.state, {}, x0, xs, us) == Status::kOk);

  // Python records A13: it never shifts xs[N], so that one is compared in
  // the A13 test below.
  double worst = max_diff(x0, g.want_x0);
  for (std::size_t i = 0; i < n; ++i) {
    worst = std::max(worst, max_diff(xs[i], g.want_xs[i]));
    worst = std::max(worst, max_diff(us[i], g.want_us[i]));
  }
  MESSAGE("G3 max abs error (all but xs[N]): " << worst);
  CHECK(worst < 1e-10);
}

TEST_CASE("WarmStartShiftPreviousSolution.generate: A13, terminal state") {
  G3 g;
  const auto& model = panda().robot_model();
  const Eigen::Index nx = model.nq + model.nv;
  const std::size_t n = g.params.timesteps.size();
  REQUIRE(g.params.timesteps.back() > g.params.dt);

  WarmStartShiftPreviousSolution ws(panda(), g.params);
  ws.update_previous_solution(g.prev);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);
  auto xs = zeros(n + 1, nx);
  auto us = zeros(n, model.nv);
  REQUIRE(ws.generate(g.state, {}, x0, xs, us) == Status::kOk);

  // Python leaves the terminal state where it was.
  CHECK(max_diff(g.want_xs[n], g.prev.states[n]) == 0.0);
  // C++ advances it by one dt with the last control.
  const Eigen::VectorXd expected =
      step(g.params.dt, g.prev.states[n], g.prev.feed_forward_terms[n - 1]);
  MESSAGE("A13: |xs[N] C++ - Python| = " << max_diff(xs[n], g.want_xs[n]));
  CHECK(max_diff(xs[n], expected) < 1e-10);
  CHECK(max_diff(xs[n], g.want_xs[n]) > 1e-6);
}

TEST_CASE("WarmStartShiftPreviousSolution.generate: A13, long last step only") {
  // Steps 0.1, 0.2, 0.1: the last step is dt, so xs[N] is not integrated.
  auto params = make_shift_params(0.1, {1, 2, 1}, {1, 1, 1});
  const auto& model = panda().robot_model();
  WarmStartShiftPreviousSolution ws(panda(), params);
  const OcpResults prev = rollout(params, random_vectors(3, model.nv));
  ws.update_previous_solution(prev);
  const RobotState state = make_robot_state(model.nq, model.nv);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(model.nq + model.nv);
  auto xs = zeros(4, model.nq + model.nv);
  auto us = zeros(3, model.nv);
  REQUIRE(ws.generate(state, {}, x0, xs, us) == Status::kOk);
  CHECK(xs[3] == prev.states[3]);
  // The long middle step advances by one dt and keeps its control.
  CHECK(xs[1] == step(0.1, prev.states[1], prev.feed_forward_terms[1]));
  CHECK(us[1] == prev.feed_forward_terms[1]);
}

// ---- RT tests
// ----------------------------------------------------------------

namespace {

struct RtState {
  const pinocchio::Model& model = panda().robot_model();
  Eigen::Index nx = model.nq + model.nv;
  std::vector<WeightedTrajectoryPoint> points =
      make_points(model, kN, Frames::kFromTrajectory);
  std::vector<const WeightedTrajectoryPoint*> horizon = pointers(points);
  RobotState state = make_robot_state(model.nq, model.nv);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);
  std::vector<Eigen::VectorXd> xs = zeros(kN + 1, nx);
  std::vector<Eigen::VectorXd> us = zeros(kN, model.nv);

  RtState() {
    state.q.setConstant(0.1);
    state.v.setConstant(0.2);
    state.a.setConstant(0.3);
    for (auto& p : points) {
      p.q.setConstant(0.4);
      p.qdot.setConstant(0.5);
      p.qddot.setConstant(0.6);
    }
  }
};

}  // namespace

TEST_CASE("RtWarmStartReference.generate: ticks and early returns") {
  RtState f;
  WarmStartReference ws(f.model);
  REQUIRE(ws.generate(f.state, f.horizon, f.x0, f.xs, f.us) == Status::kOk);

  auto short_xs = f.xs;
  short_xs.pop_back();
  auto bad_us = f.us;
  bad_us[3] = Eigen::VectorXd::Zero(2);
  const std::vector<const WeightedTrajectoryPoint*> empty;
  auto with_null = f.horizon;
  with_null[4] = nullptr;
  Status s[5];
  CHECK_RT_SAFE(for (int tick = 0; tick < 5; ++tick) {
    s[0] = ws.generate(f.state, f.horizon, f.x0, f.xs, f.us);
  });
  CHECK(s[0] == Status::kOk);
  CHECK_RT_SAFE(s[1] = ws.generate(f.state, f.horizon, f.x0, short_xs, f.us));
  CHECK_RT_SAFE(s[2] = ws.generate(f.state, f.horizon, f.x0, f.xs, bad_us));
  CHECK_RT_SAFE(s[3] = ws.generate(f.state, empty, f.x0, f.xs, f.us));
  CHECK_RT_SAFE(s[4] = ws.generate(f.state, with_null, f.x0, f.xs, f.us));
  for (int i = 1; i < 5; ++i) CHECK(s[i] == Status::kSizeMismatch);
}

TEST_CASE(
    "RtWarmStartShiftPreviousSolution.generate: ticks and early returns") {
  RtState f;
  // Equal steps, then a long tail (the integrator branch and A13).
  const auto equal = make_shift_params(0.01, {1}, {kN});
  const auto mixed = make_shift_params(0.01, {1, 2, 4}, {10, 6, 4});
  for (const OcpParams* params : {&equal, &mixed}) {
    WarmStartShiftPreviousSolution ws(panda(), *params);
    Status s[6];
    // Before any solution.
    CHECK_RT_SAFE(s[0] = ws.generate(f.state, f.horizon, f.x0, f.xs, f.us));
    CHECK(s[0] == Status::kNoPreviousSolution);

    ws.update_previous_solution(
        rollout(*params, random_vectors(kN, f.model.nv)));
    REQUIRE(ws.generate(f.state, f.horizon, f.x0, f.xs, f.us) ==
            Status::kOk);  // warm-up

    auto short_xs = f.xs;
    short_xs.pop_back();
    auto bad_us = f.us;
    bad_us[3] = Eigen::VectorXd::Zero(2);
    CHECK_RT_SAFE(for (int tick = 0; tick < 5; ++tick) {
      s[1] = ws.generate(f.state, f.horizon, f.x0, f.xs, f.us);
    });
    CHECK(s[1] == Status::kOk);
    CHECK_RT_SAFE(s[2] = ws.generate(f.state, f.horizon, f.x0, short_xs, f.us));
    CHECK_RT_SAFE(s[3] = ws.generate(f.state, f.horizon, f.x0, f.xs, bad_us));
    CHECK(s[2] == Status::kSizeMismatch);
    CHECK(s[3] == Status::kSizeMismatch);
  }
}

TEST_CASE("RtWarmStartShiftPreviousSolution.update_previous_solution: ticks") {
  RtState f;
  const auto params = make_shift_params(0.01, {1, 2}, {15, 3});
  WarmStartShiftPreviousSolution ws(panda(), params);
  OcpResults good = rollout(params, random_vectors(18, f.model.nv));
  OcpResults wrong;
  wrong.resize(5, f.nx, 2 * f.model.nv, f.model.nv);
  ws.update_previous_solution(good);  // warm-up
  CHECK_RT_SAFE(for (int tick = 0; tick < 5;
                     ++tick) { ws.update_previous_solution(good); });
  CHECK_RT_SAFE(ws.update_previous_solution(wrong));  // ignored
  auto xs = zeros(19, f.nx);
  auto us = zeros(18, f.model.nv);
  CHECK(ws.generate(f.state, f.horizon, f.x0, xs, us) == Status::kOk);
}
