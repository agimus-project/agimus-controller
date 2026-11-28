// Fixtures of test_mpc.cpp. A separate translation unit, so that the test
// file stays under the 2 GB compile limit (R6), and so that only this file
// parses TOML.

#include "mpc_fixtures.hpp"

#include <cmath>
#include <crocoddyl/core/actions/unicycle.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>

#include "agimus_controller/ocp_builders.hpp"
#include "panda_ocp.hpp"

namespace agimus_controller::test {

namespace {

// ---- TOML helpers (golden files) -------------------------------------------

[[noreturn]] void missing(const std::string& what) {
  throw std::runtime_error("G10a: missing or malformed '" + what + "'");
}

const toml::table& table(const toml::table& t, const char* key) {
  const toml::table* sub = t[key].as_table();
  if (sub == nullptr) missing(key);
  return *sub;
}

const toml::array& array(const toml::table& t, const char* key) {
  const toml::array* a = t[key].as_array();
  if (a == nullptr) missing(key);
  return *a;
}

template <typename T>
T value(const toml::table& t, const char* key) {
  const std::optional<T> v = t[key].value<T>();
  if (!v.has_value()) missing(key);
  return *v;
}

Eigen::VectorXd to_vector(const toml::node& node) {
  const toml::array* a = node.as_array();
  if (a == nullptr) missing("vector");
  Eigen::VectorXd v(static_cast<Eigen::Index>(a->size()));
  for (std::size_t i = 0; i < a->size(); ++i) {
    const std::optional<double> x = (*a)[i].value<double>();
    if (!x.has_value()) missing("vector entry");
    v[static_cast<Eigen::Index>(i)] = *x;
  }
  return v;
}

Eigen::MatrixXd to_matrix(const toml::node& node) {
  const toml::array* rows = node.as_array();
  if (rows == nullptr || rows->empty()) missing("matrix");
  const Eigen::VectorXd first = to_vector((*rows)[0]);
  Eigen::MatrixXd m(static_cast<Eigen::Index>(rows->size()), first.size());
  for (std::size_t i = 0; i < rows->size(); ++i) {
    m.row(static_cast<Eigen::Index>(i)) = to_vector((*rows)[i]).transpose();
  }
  return m;
}

std::vector<Eigen::VectorXd> to_vectors(const toml::array& a) {
  std::vector<Eigen::VectorXd> out;
  for (const toml::node& row : a) out.push_back(to_vector(row));
  return out;
}

std::vector<int> to_ints(const toml::array& a) {
  std::vector<int> out;
  for (const toml::node& x : a) {
    const std::optional<int64_t> v = x.value<int64_t>();
    if (!v.has_value()) missing("integer");
    out.push_back(static_cast<int>(*v));
  }
  return out;
}

OcpParams read_params(const toml::table& tp) {
  OcpParams p;
  p.dt = value<double>(tp, "dt");
  p.solver_iters = static_cast<int>(value<int64_t>(tp, "solver_iters"));
  p.dt_factor_n_seq.factors = to_ints(array(tp, "factors"));
  p.dt_factor_n_seq.n_steps = to_ints(array(tp, "n_steps"));
  p.horizon_size = static_cast<int>(value<int64_t>(tp, "horizon_size"));
  p.qp_iters = static_cast<int>(value<int64_t>(tp, "qp_iters"));
  p.termination_tolerance = value<double>(tp, "termination_tolerance");
  p.eps_abs = value<double>(tp, "eps_abs");
  p.eps_rel = value<double>(tp, "eps_rel");
  p.callbacks = value<bool>(tp, "callbacks");
  p.use_debug_data = value<bool>(tp, "use_debug_data");
  p.use_filter_line_search = value<bool>(tp, "use_filter_line_search");
  p.n_threads = static_cast<int>(value<int64_t>(tp, "n_threads"));
  p.finalize();
  if (p.n_controls != value<int64_t>(tp, "n_controls")) missing("n_controls");
  return p;
}

G10a load_g10a() {
  const toml::table golden =
      toml::parse_file(AGIMUS_GOLDEN_DIR "/g10a_mpc_goal_reaching.toml");
  G10a g;
  g.params = read_params(table(golden, "ocp_params"));
  const toml::table& inputs = table(golden, "inputs");
  g.dt_ns = std::chrono::nanoseconds(value<int64_t>(inputs, "dt_ns"));

  const toml::table& s0 = table(inputs, "initial_state");
  g.initial_state.time =
      std::chrono::nanoseconds(value<int64_t>(s0, "time_ns"));
  g.initial_state.q = to_vector(*s0["q"].node());
  g.initial_state.v = to_vector(*s0["v"].node());
  g.initial_state.a = to_vector(*s0["a"].node());

  const toml::table& ref = table(inputs, "reference");
  const auto n_points =
      static_cast<std::size_t>(value<int64_t>(ref, "n_points"));
  const std::vector<int> ids = to_ints(array(ref, "id"));
  const toml::array& times = array(ref, "time_ns");
  const std::vector<Eigen::VectorXd> translations =
      to_vectors(array(ref, "ee_translation"));
  const Eigen::Matrix3d rotation = to_matrix(*ref["ee_rotation"].node());
  const Eigen::VectorXd q = to_vector(*ref["q"].node());
  auto point = make_trajectory_point(q.size(), q.size());
  point.q = q;
  point.qdot = to_vector(*ref["v"].node());
  point.qddot = to_vector(*ref["a"].node());
  point.robot_effort = to_vector(*ref["tau"].node());
  point.w_q = to_vector(*ref["w_q"].node());
  point.w_qdot = to_vector(*ref["w_v"].node());
  point.w_robot_effort = to_vector(*ref["w_tau"].node());
  EndEffectorInput ee;
  ee.frame_name = FrameName(value<std::string>(ref, "ee_frame"));
  ee.w_pose = to_vector(*ref["w_ee"].node());
  point.ee_inputs.push_back(ee);
  if (ids.size() != n_points || times.size() != n_points ||
      translations.size() != n_points) {
    missing("reference size");
  }
  for (std::size_t k = 0; k < n_points; ++k) {
    point.id = ids[k];
    point.time = std::chrono::nanoseconds(*times[k].value<int64_t>());
    point.ee_inputs[0].pose = pinocchio::SE3(rotation, translations[k]);
    g.reference.push_back(point);
  }

  const toml::table& startup = table(golden, "startup");
  g.startup_solved = value<bool>(startup, "problem_solved");
  g.startup_iterations =
      static_cast<int>(value<int64_t>(startup, "iterations"));
  g.startup_kkt = value<double>(startup, "kkt");
  g.startup_xs = to_vectors(array(startup, "xs"));
  g.startup_us = to_vectors(array(startup, "us"));

  const toml::table& ticks = table(golden, "ticks");
  const auto n_ticks =
      static_cast<std::size_t>(value<int64_t>(inputs, "n_ticks"));
  const toml::array& t_time = array(ticks, "time_ns");
  const toml::array& t_q = array(ticks, "q");
  const toml::array& t_v = array(ticks, "v");
  const std::vector<int> t_ref = to_ints(array(ticks, "reference_id"));
  const toml::array& t_solved = array(ticks, "problem_solved");
  const std::vector<int> t_iter = to_ints(array(ticks, "nb_iter"));
  const toml::array& t_kkt = array(ticks, "kkt");
  const toml::array& t_u0 = array(ticks, "u0");
  const toml::array& t_k0 = array(ticks, "K0");
  const toml::array& t_x1 = array(ticks, "x1");
  const toml::array& t_qn = array(ticks, "q_next");
  const toml::array& t_vn = array(ticks, "v_next");
  for (std::size_t k = 0; k < n_ticks; ++k) {
    G10aTick t;
    t.time = std::chrono::nanoseconds(*t_time[k].value<int64_t>());
    t.q = to_vector(t_q[k]);
    t.v = to_vector(t_v[k]);
    t.reference_id = t_ref.at(k);
    t.problem_solved = *t_solved[k].value<bool>();
    t.nb_iter = t_iter.at(k);
    t.kkt = *t_kkt[k].value<double>();
    t.u0 = to_vector(t_u0[k]);
    t.K0 = to_matrix(t_k0[k]);
    t.x1 = to_vector(t_x1[k]);
    t.q_next = to_vector(t_qn[k]);
    t.v_next = to_vector(t_vn[k]);
    g.ticks.push_back(std::move(t));
  }
  return g;
}

}  // namespace

const G10a& g10a() {
  static const G10a golden = load_g10a();
  return golden;
}

// ---- Goal-reaching MPC ------------------------------------------------------

GoalReachingMpc make_goal_reaching_mpc(const OcpParams& params,
                                       std::size_t capacity) {
  const pinocchio::Model& model = panda().robot_model();
  GoalReachingMpc m;
  m.ocp = std::make_shared<Ocp>(build_goal_reaching_models(panda(), params),
                                params);
  m.warm_start =
      std::make_shared<WarmStartShiftPreviousSolution>(panda(), params);
  m.buffer = std::make_shared<TrajectoryBuffer>(
      params.dt_factor_n_seq, capacity, BufferMode::kSliding,
      make_trajectory_point(model.nq, model.nv));
  m.mpc = std::make_unique<Mpc>(m.ocp, m.warm_start, m.buffer);
  return m;
}

WeightedTrajectoryPoint moving_target_point(int k,
                                            std::chrono::nanoseconds dt) {
  const pinocchio::Model& model = panda().robot_model();
  auto p = make_trajectory_point(model.nq, model.nv);
  p.id = k;
  p.time = k * dt;
  p.q << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
  p.w_q.setConstant(0.01);
  p.w_qdot.setConstant(0.01);
  p.w_robot_effort.setConstant(1e-4);
  EndEffectorInput ee;
  ee.frame_name = FrameName(kTcp);
  const Eigen::Matrix3d down = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  ee.pose =
      pinocchio::SE3(down, Eigen::Vector3d(0.45, -0.05 + 0.005 * k, 0.45));
  ee.w_pose.setConstant(1e3);
  p.ee_inputs.push_back(ee);
  return p;
}

bool start_up(Mpc& mpc, WarmStart& warm_start, const RobotState& state) {
  if (!mpc.buffer().has_horizon()) {
    throw std::runtime_error("start_up: the buffer has no horizon.");
  }
  const auto& horizon = mpc.buffer().horizon();
  Ocp& ocp = mpc.ocp();
  if (ocp.set_reference_weighted_trajectory(horizon) != Status::kOk) {
    throw std::runtime_error("start_up: set_reference_weighted_trajectory.");
  }
  WarmStartReference reference(panda().robot_model());
  const auto n = static_cast<std::size_t>(ocp.n_controls());
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(ocp.nx());
  std::vector<Eigen::VectorXd> xs(n + 1, x0);
  std::vector<Eigen::VectorXd> us(n, Eigen::VectorXd::Zero(ocp.nu()));
  if (reference.generate(state, horizon, x0, xs, us) != Status::kOk) {
    throw std::runtime_error("start_up: WarmStartReference::generate.");
  }
  const bool solved = ocp.solve_without_limits(x0, xs, us);
  warm_start.update_previous_solution(ocp.results());
  return solved;
}

// ---- Unicycle ---------------------------------------------------------------

OcpModels unicycle_models(int horizon, double dt) {
  OcpModels m;
  for (int i = 0; i < horizon; ++i) {
    auto model = std::make_shared<crocoddyl::ActionModelUnicycle>();
    const bool terminal = i == horizon - 1;
    model->set_cost_weights(terminal ? Eigen::Vector2d(100.0, 100.0)
                                     : Eigen::Vector2d(1.0, 1.0));
    model->set_dt(dt);
    if (terminal) {
      m.terminal = model;
    } else {
      m.running.push_back(model);
    }
  }
  m.state = m.terminal->get_state();
  return m;
}

OcpParams unicycle_params(int n_controls, double dt) {
  OcpParams p;
  p.dt = dt;
  p.solver_iters = 100;  // Python's SolverDDP.solve() default
  // Python solves with SolverDDP; Ocp always uses CSQP. With the merit line
  // search (agimus' default, B8) CSQP stops after one iteration on this
  // problem (KKT 4934, also in Python mim_solvers 0.3.0), so its xs are not
  // a rollout of its us. The filter line search converges (14 iterations).
  p.use_filter_line_search = true;
  p.dt_factor_n_seq = {{1}, {n_controls}};
  p.horizon_size = n_controls;
  p.n_threads = 1;
  p.finalize();
  return p;
}

WarmStartUnicycle::WarmStartUnicycle(int n_controls, Eigen::Index nx,
                                     Eigen::Index nu) {
  previous_.resize(static_cast<std::size_t>(n_controls), nx, nx, nu);
}

Status WarmStartUnicycle::generate(
    const RobotState& x0_state,
    const std::vector<const WeightedTrajectoryPoint*>& horizon,
    Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& xs,
    std::vector<Eigen::VectorXd>& us) {
  first_reference_q0_ = horizon[0]->q[0];
  x0 = x0_state.q;
  const std::size_t n = us.size();
  if (!has_previous_) {
    for (std::size_t i = 0; i <= n; ++i) xs[i] = horizon[i]->q;
    for (auto& u : us) u.setZero();
  } else {
    for (std::size_t i = 0; i < n; ++i) xs[i] = previous_.states[i + 1];
    xs[n] = horizon.back()->q;
    for (std::size_t i = 0; i < n; ++i) {
      us[i] = previous_.feed_forward_terms[i];
    }
  }
  return Status::kOk;
}

void WarmStartUnicycle::update_previous_solution(const OcpResults& previous) {
  has_previous_ = copy_results(previous, previous_) == Status::kOk;
}

Eigen::Vector3d unicycle_integrate(const Eigen::Vector3d& q,
                                   const Eigen::Vector2d& u, double dt) {
  return q +
         Eigen::Vector3d(u[0] * std::cos(q[2]), u[0] * std::sin(q[2]), u[1]) *
             dt;
}

}  // namespace agimus_controller::test
