// Mpc (design.md, "MPC"): port of agimus_controller/tests/test_mpc_unicycle.py,
// the closed loop of golden set G10a, behaviour changes B1/A12 (time advance)
// and B2 (kNotEnoughData), debug data, and the RT tests.

#include <omp.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <crocoddyl/core/integ-action-base.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <vector>

#include "agimus_controller/mpc.hpp"
#include "agimus_controller/ocp_builders.hpp"
#include "mpc_fixtures.hpp"
#include "panda_ocp.hpp"
#include "rt_guard.hpp"

using agimus_controller::BufferMode;
using agimus_controller::make_robot_state;
using agimus_controller::make_trajectory_point;
using agimus_controller::Mpc;
using agimus_controller::Ocp;
using agimus_controller::OcpParams;
using agimus_controller::RobotState;
using agimus_controller::Status;
using agimus_controller::TrajectoryBuffer;
using agimus_controller::WarmStartReference;
using agimus_controller::WeightedTrajectoryPoint;
using namespace agimus_controller::test;  // fixtures
using namespace std::chrono_literals;
using std::chrono::nanoseconds;

// ---- Clock reads ------------------------------------------------------------
// std::chrono::steady_clock::now() calls clock_gettime in libstdc++; the
// executable's definition is the one the dynamic linker binds (as the
// allocation hook does for malloc), so reads can be counted.

namespace {
std::atomic<bool> g_count_clock{false};
std::atomic<int> g_clock_reads{0};
}  // namespace

extern "C" int clock_gettime(clockid_t clock, struct timespec* tp) noexcept {
  if (g_count_clock.load(std::memory_order_relaxed)) {
    g_clock_reads.fetch_add(1, std::memory_order_relaxed);
  }
  return static_cast<int>(syscall(SYS_clock_gettime, clock, tp));
}

namespace {

template <typename F>
int clock_reads(F&& f) {
  g_clock_reads = 0;
  g_count_clock = true;
  f();
  g_count_clock = false;
  return g_clock_reads.load();
}

// numpy.testing.assert_allclose(actual, desired) with its default rtol.
bool allclose(const Eigen::VectorXd& actual, const Eigen::VectorXd& desired,
              double rtol = 1e-7) {
  return actual.size() == desired.size() &&
         ((actual - desired).cwiseAbs().array() <=
          rtol * desired.cwiseAbs().array())
             .all();
}

double max_abs(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  REQUIRE(a.rows() == b.rows());
  REQUIRE(a.cols() == b.cols());
  return (a - b).cwiseAbs().maxCoeff();
}

RobotState panda_state(const Eigen::VectorXd& q) {
  RobotState s = make_robot_state(q.size(), q.size());
  s.q = q;
  return s;
}

const Eigen::VectorXd& q_init() {
  static const Eigen::VectorXd q =
      (Eigen::VectorXd(7) << 0.1, -0.7, -0.1, -2.2, 0.05, 1.5, 0.7).finished();
  return q;
}

/// A goal-reaching MPC with `n_points` moving-target points and, if
/// `started`, the start-up done from q_init().
GoalReachingMpc started_mpc(const OcpParams& params, int n_points,
                            std::size_t capacity, bool started = true) {
  GoalReachingMpc m = make_goal_reaching_mpc(params, capacity);
  for (int k = 0; k < n_points; ++k) {
    REQUIRE(m.mpc->push_back(moving_target_point(k, 10ms)) == Status::kOk);
  }
  if (started) start_up(*m.mpc, *m.warm_start, panda_state(q_init()));
  return m;
}

OcpParams params_with(double dt, std::vector<int> factors,
                      std::vector<int> n_steps, int solver_iters = 10) {
  OcpParams p;
  p.dt = dt;
  p.solver_iters = solver_iters;
  p.dt_factor_n_seq = {std::move(factors), std::move(n_steps)};
  for (int n : p.dt_factor_n_seq.n_steps) p.horizon_size += n;
  p.n_threads = 1;
  p.finalize();
  return p;
}

}  // namespace

// ---- Setup ------------------------------------------------------------------

TEST_CASE("Mpc: setup errors") {
  const auto params = make_params(5);
  auto m = make_goal_reaching_mpc(params, 20);
  CHECK_THROWS_WITH_AS(Mpc(nullptr, m.warm_start, m.buffer),
                       doctest::Contains("null"), std::invalid_argument);
  CHECK_THROWS_WITH_AS(Mpc(m.ocp, nullptr, m.buffer), doctest::Contains("null"),
                       std::invalid_argument);
  CHECK_THROWS_WITH_AS(Mpc(m.ocp, m.warm_start, nullptr),
                       doctest::Contains("null"), std::invalid_argument);
  const pinocchio::Model& model = panda().robot_model();
  auto other = std::make_shared<TrajectoryBuffer>(
      make_params(6).dt_factor_n_seq, 20, BufferMode::kSliding,
      make_trajectory_point(model.nq, model.nv));
  CHECK_THROWS_WITH_AS(Mpc(m.ocp, m.warm_start, other),
                       doctest::Contains("n_controls + 1 = 6"),
                       std::invalid_argument);
}

// ---- test_mpc_unicycle.py ---------------------------------------------------

namespace {

struct UnicycleMpc {
  std::shared_ptr<Ocp> ocp;
  std::shared_ptr<WarmStartUnicycle> warm_start;
  std::shared_ptr<TrajectoryBuffer> buffer;
  std::unique_ptr<Mpc> mpc;

  UnicycleMpc(int horizon, double dt, std::size_t capacity) {
    const auto params = unicycle_params(horizon - 1, dt);
    ocp = std::make_shared<Ocp>(unicycle_models(horizon, dt), params);
    warm_start = std::make_shared<WarmStartUnicycle>(horizon - 1, 3, 2);
    buffer = std::make_shared<TrajectoryBuffer>(params.dt_factor_n_seq,
                                                capacity, BufferMode::kSliding,
                                                make_trajectory_point(3, 3));
    mpc = std::make_unique<Mpc>(ocp, warm_start, buffer);
  }
};

WeightedTrajectoryPoint unicycle_point(int k, double x, nanoseconds time) {
  auto p = make_trajectory_point(3, 3);
  p.id = k;
  p.time = time;
  p.q << x, 0.0, 0.0;
  p.w_q.setOnes();
  p.w_qdot.setOnes();
  return p;
}

RobotState unicycle_state() {
  RobotState s;
  s.q = Eigen::Vector3d(0.0, 1.0, 1.57);
  s.v = Eigen::Vector2d::Zero();
  s.a = Eigen::Vector2d::Zero();
  return s;
}

constexpr int kUnicycleIters = 500;  // N_iter

}  // namespace

TEST_CASE("Mpc unicycle: horizon and dt (test_hoziron_and_dt)") {
  UnicycleMpc u(100, 0.01, 200);
  CHECK(u.mpc->ocp().n_controls() == 99);
  CHECK(u.mpc->ocp().dt() == 0.01);
}

// The Python callback asserts that node 0's reference is point `iteration`;
// here the warm start records horizon[0] and the debug data its id.
TEST_CASE("Mpc unicycle: reference trajectory (test_reference_trajectory)") {
  UnicycleMpc u(100, 0.01, kUnicycleIters + 100);
  const auto dt_ns = nanoseconds(10'000'000);
  for (int k = 0; k < kUnicycleIters + 99 + 1; ++k) {
    REQUIRE(u.mpc->push_back(unicycle_point(k, k, (k - 1) * dt_ns)) ==
            Status::kOk);
  }
  const RobotState state = unicycle_state();
  nanoseconds time{0};
  for (int iteration = 0; iteration < kUnicycleIters; ++iteration) {
    REQUIRE(u.mpc->run(state, time) == Status::kOk);
    REQUIRE(u.warm_start->first_reference_q0() == iteration);
    REQUIRE(u.mpc->debug_data().reference_id == iteration);
    time += dt_ns;
  }
  CHECK(u.mpc->buffer().size() == 100);
}

TEST_CASE("Mpc unicycle: solve (test_solve)") {
  UnicycleMpc u(100, 0.01, kUnicycleIters + 100);
  const auto dt_ns = nanoseconds(10'000'000);
  for (int k = 0; k < kUnicycleIters + 99 + 1; ++k) {
    REQUIRE(u.mpc->push_back(unicycle_point(k, 0.0, k * dt_ns)) == Status::kOk);
  }
  RobotState state = unicycle_state();
  nanoseconds time{0};
  for (int iteration = 0; iteration < kUnicycleIters; ++iteration) {
    REQUIRE(u.mpc->run(state, time) == Status::kOk);
    const auto& res = u.mpc->ocp().results();
    INFO("iteration " << iteration);
    REQUIRE(allclose(res.states[0], state.q));
    const Eigen::Vector2d control = res.feed_forward_terms[0];
    const Eigen::Vector3d next_q =
        unicycle_integrate(state.q, control, u.mpc->ocp().dt());
    REQUIRE(allclose(res.states[1], next_q));
    state.q = next_q;
    state.v = control;
    time += dt_ns;
  }
  // The unicycle is driven towards the origin (reference and costs).
  CHECK(state.q.head<2>().norm() < 1.0);
}

// ---- G10a: MPC.run closed loop ----------------------------------------------

TEST_CASE("Mpc: golden set G10a, 20 ticks of closed loop") {
  const G10a& g = g10a();
  auto m = make_goal_reaching_mpc(g.params, 2 * g.reference.size());
  for (const auto& p : g.reference) {
    REQUIRE(m.mpc->push_back(p) == Status::kOk);
  }
  RobotState state = g.initial_state;

  // Start-up, as the generator (ROS setup_mpc_initial_guess).
  const bool startup_solved = start_up(*m.mpc, *m.warm_start, state);
  const auto& r0 = m.ocp->results();
  const double tol = 1e-6;  // design.md, Golden files: solver outputs
  double d_startup = 0.0;
  for (std::size_t i = 0; i < r0.states.size(); ++i) {
    d_startup = std::max(d_startup, max_abs(r0.states[i], g.startup_xs.at(i)));
  }
  for (std::size_t i = 0; i < r0.feed_forward_terms.size(); ++i) {
    d_startup = std::max(d_startup,
                         max_abs(r0.feed_forward_terms[i], g.startup_us.at(i)));
  }
  CHECK(startup_solved == g.startup_solved);
  CHECK(m.ocp->debug_data().nb_iter == g.startup_iterations);
  CHECK(std::abs(m.ocp->debug_data().kkt_norm - g.startup_kkt) <= tol);
  CHECK(d_startup <= tol);

  double d_state = 0.0, d_u0 = 0.0, d_k0 = 0.0, d_x1 = 0.0, d_kkt = 0.0,
         d_next = 0.0;
  int iter_mismatch = 0, solved_mismatch = 0, id_mismatch = 0;
  const nanoseconds step = g.dt_ns;  // timesteps[0] = dt (A12 not reached)
  for (std::size_t k = 0; k < g.ticks.size(); ++k) {
    const G10aTick& t = g.ticks[k];
    d_state = std::max({d_state, max_abs(state.q, t.q), max_abs(state.v, t.v)});
    REQUIRE(m.mpc->run(state, state.time) == Status::kOk);
    const auto& res = m.ocp->results();
    const auto& dbg = m.mpc->debug_data();
    d_u0 = std::max(d_u0, max_abs(res.feed_forward_terms[0], t.u0));
    d_k0 = std::max(d_k0, max_abs(res.riccati_gains[0], t.K0));
    d_x1 = std::max(d_x1, max_abs(res.states[1], t.x1));
    d_kkt = std::max(d_kkt, std::abs(dbg.ocp.kkt_norm - t.kkt));
    iter_mismatch += dbg.ocp.nb_iter != t.nb_iter;
    solved_mismatch += dbg.ocp.problem_solved != t.problem_solved;
    id_mismatch += dbg.reference_id != t.reference_id;

    const Eigen::VectorXd u0 = res.feed_forward_terms[0];
    REQUIRE(m.mpc->integrate(state, u0) == Status::kOk);
    d_next = std::max(
        {d_next, max_abs(state.q, t.q_next), max_abs(state.v, t.v_next)});
    // B1: Python's time stays at t.time (0); C++ advances by one step.
    CHECK(state.time == t.time + static_cast<int>(k + 1) * step);
  }
  MESSAGE("G10a max |C++ - Python|: startup "
          << d_startup << ", state " << d_state << ", u0 " << d_u0 << ", K0 "
          << d_k0 << ", x1 " << d_x1 << ", kkt " << d_kkt << ", q/v next "
          << d_next);
  CHECK(d_state <= tol);
  CHECK(d_u0 <= tol);
  CHECK(d_k0 <= tol);
  CHECK(d_x1 <= tol);
  CHECK(d_kkt <= tol);
  CHECK(d_next <= tol);
  CHECK(iter_mismatch == 0);
  CHECK(solved_mismatch == 0);
  CHECK(id_mismatch == 0);
}

// ---- B2: kNotEnoughData -----------------------------------------------------

TEST_CASE("Mpc.run: B2, kNotEnoughData until the buffer holds the horizon") {
  SUBCASE("factors {1}") {
    const auto params = make_params(5);
    auto m = started_mpc(params, 0, 20, false);
    for (int k = 0; k < 5; ++k) {
      REQUIRE(m.mpc->push_back(moving_target_point(k, 10ms)) == Status::kOk);
      CHECK(m.mpc->run(panda_state(q_init()), 0ns) == Status::kNotEnoughData);
    }
    CHECK(m.mpc->buffer().size() == 5);  // nothing consumed
  }
  SUBCASE("factors {1, 2}: Python's n_controls + 1 check passes too early") {
    // n_controls 4, horizon indexes {0, 1, 2, 4, 6}: 7 points are needed.
    const auto params = params_with(0.01, {1, 2}, {2, 2});
    REQUIRE(params.n_controls == 4);
    auto m = started_mpc(params, 6, 20, false);
    // Python: len(buffer) = 6 >= n_controls + 1 = 5, then horizon asserts.
    CHECK(m.mpc->buffer().size() >= static_cast<std::size_t>(4 + 1));
    const RobotState state = panda_state(q_init());
    CHECK(m.mpc->run(state, 0ns) == Status::kNotEnoughData);
    CHECK(m.mpc->buffer().size() == 6);
    REQUIRE(m.mpc->push_back(moving_target_point(6, 10ms)) == Status::kOk);
    start_up(*m.mpc, *m.warm_start, state);
    CHECK(m.mpc->run(state, 0ns) == Status::kOk);
    CHECK(m.mpc->buffer().size() == 6);
    CHECK(m.mpc->run(state, 0ns) == Status::kNotEnoughData);
  }
}

TEST_CASE("Mpc.run: errors of the references and of the warm start") {
  const auto params = make_params(5);
  auto m = started_mpc(params, 6, 20, false);
  const RobotState state = panda_state(q_init());
  // The shift warm start has no previous solution before the start-up.
  CHECK(m.mpc->run(state, 0ns) == Status::kNoPreviousSolution);
  CHECK(m.mpc->buffer().size() == 6);
  start_up(*m.mpc, *m.warm_start, state);
  // An unknown frame in the horizon.
  auto bad = moving_target_point(6, 10ms);
  bad.ee_inputs[0].frame_name = agimus_controller::FrameName("no_such_frame");
  auto m2 = started_mpc(params, 0, 20, false);
  for (int k = 0; k < 6; ++k) {
    REQUIRE(m2.mpc->push_back(k == 0 ? bad : moving_target_point(k, 10ms)) ==
            Status::kOk);
  }
  CHECK(m2.mpc->run(state, 0ns) == Status::kUnknownFrame);
  CHECK(m2.mpc->buffer().size() == 6);
  // A state of the wrong size.
  CHECK(m.mpc->run(panda_state(Eigen::VectorXd::Zero(3)), 0ns) ==
        Status::kSizeMismatch);
  CHECK(m.mpc->run(state, 0ns) == Status::kOk);
}

// ---- integrate: B1 and A12 --------------------------------------------------

TEST_CASE("Mpc.integrate: one step of node 0, time advanced by timesteps[0]") {
  struct Case {
    double dt;
    std::vector<int> factors, n_steps;
    nanoseconds expected;
  };
  // timesteps[0] * 1e9 lies just below an integer in binary: 0.00013 s
  // (129999.99999999999), 0.00026 s (259999.99999999997), 2.1 s from
  // 3 * 0.7 (2099999999.9999998). Truncation would lose a nanosecond.
  const Case cases[] = {
      {0.01, {1}, {5}, 10'000'000ns},
      {0.00013, {1}, {5}, 130'000ns},
      {0.00013, {2, 1}, {2, 3}, 260'000ns},  // A12: first factor 2
      {0.7, {3}, {5}, 2'100'000'000ns},
  };
  for (const Case& c : cases) {
    const auto params = params_with(c.dt, c.factors, c.n_steps);
    CAPTURE(params.timesteps[0]);
    auto ocp = std::make_shared<Ocp>(
        build_goal_reaching_models(panda(), params), params);
    const pinocchio::Model& model = panda().robot_model();
    auto ws = std::make_shared<WarmStartReference>(model);
    auto buffer = std::make_shared<TrajectoryBuffer>(
        params.dt_factor_n_seq, 40, BufferMode::kSliding,
        make_trajectory_point(model.nq, model.nv));
    Mpc mpc(ocp, ws, buffer);
    // Node 0 integrates timesteps[0].
    const auto* node0 =
        dynamic_cast<const crocoddyl::IntegratedActionModelAbstract*>(
            ocp->problem()->get_runningModels()[0].get());
    REQUIRE(node0 != nullptr);
    CHECK(node0->get_dt() == params.timesteps[0]);

    RobotState state = panda_state(q_init());
    state.v.setConstant(0.2);
    state.a.setConstant(7.0);
    state.time = 5ns;
    const Eigen::VectorXd u = Eigen::VectorXd::Constant(7, 0.3);
    Eigen::VectorXd x(14), expected(14);
    x << state.q, state.v;
    ocp->integrate(x, u, expected);
    REQUIRE(mpc.integrate(state, u) == Status::kOk);
    CHECK(state.q == expected.head(7));
    CHECK(state.v == expected.tail(7));
    CHECK(state.a == Eigen::VectorXd::Constant(7, 7.0));
    CHECK(state.time == 5ns + c.expected);
    // Python: int(dt * 1e-9) = 0 (B1).
    CHECK(static_cast<long long>(params.timesteps[0] * 1e-9) == 0);
  }
}

TEST_CASE("Mpc.integrate: wrong sizes change nothing") {
  const auto params = make_params(5);
  auto m = make_goal_reaching_mpc(params, 20);
  const RobotState start = panda_state(q_init());
  const Eigen::VectorXd u = Eigen::VectorXd::Zero(7);
  RobotState s = start;
  s.q = Eigen::VectorXd::Zero(6);
  CHECK(m.mpc->integrate(s, u) == Status::kSizeMismatch);
  CHECK(s.time == 0ns);
  s = start;
  s.v = Eigen::VectorXd::Zero(8);
  CHECK(m.mpc->integrate(s, u) == Status::kSizeMismatch);
  CHECK(s == [&] {
    RobotState e = start;
    e.v = Eigen::VectorXd::Zero(8);
    return e;
  }());
  s = start;
  CHECK(m.mpc->integrate(s, Eigen::VectorXd::Zero(6)) == Status::kSizeMismatch);
  CHECK(s == start);
}

// ---- Debug data -------------------------------------------------------------

TEST_CASE("Mpc.run: debug data and phase timers with use_debug_data") {
  const auto params = make_params(5);
  REQUIRE(params.use_debug_data);
  auto m = started_mpc(params, 12, 20);
  const RobotState state = panda_state(q_init());
  REQUIRE(m.mpc->run(state, 0ns) == Status::kOk);
  REQUIRE(m.mpc->run(state, 0ns) == Status::kOk);
  const auto& d = m.mpc->debug_data();
  const auto& o = m.ocp->debug_data();
  CHECK(d.reference_id == 1);  // second tick: point 1 at node 0
  CHECK(d.ocp.nb_iter == o.nb_iter);
  CHECK(d.ocp.nb_iter > 0);
  CHECK(d.ocp.nb_qp_iter == o.nb_qp_iter);
  CHECK(d.ocp.kkt_norm == o.kkt_norm);
  CHECK(d.ocp.problem_solved == o.problem_solved);
  REQUIRE(d.ocp.result.states.size() == 6);
  for (std::size_t i = 0; i < 6; ++i) {
    CHECK(d.ocp.result.states[i] == m.ocp->results().states[i]);
  }
  CHECK(d.ocp.result.riccati_gains[0] == m.ocp->results().riccati_gains[0]);
  CHECK(d.duration_ocp_solve > 0ns);
  CHECK(d.duration_horizon_update >= 0ns);
  CHECK(d.duration_generate_warm_start >= 0ns);
  CHECK(d.duration_iteration == d.duration_horizon_update +
                                    d.duration_generate_warm_start +
                                    d.duration_ocp_solve);
}

TEST_CASE("Mpc.run: without use_debug_data, no debug work and no clock read") {
  // Control: the counter sees steady_clock.
  CHECK(clock_reads([] { (void)std::chrono::steady_clock::now(); }) == 1);

  auto with = make_params(5);
  auto without = make_params(5);
  without.use_debug_data = false;
  auto on = started_mpc(with, 12, 20);
  auto off = started_mpc(without, 12, 20);
  const RobotState state = panda_state(q_init());
  // Warm-up tick, then compare one tick of each: the debug data adds four
  // clock reads (the solver may read the clock itself, equally in both).
  REQUIRE(on.mpc->run(state, 0ns) == Status::kOk);
  REQUIRE(off.mpc->run(state, 0ns) == Status::kOk);
  Status s_on = Status::kSizeMismatch, s_off = Status::kSizeMismatch;
  const int reads_on = clock_reads([&] { s_on = on.mpc->run(state, 0ns); });
  const int reads_off = clock_reads([&] { s_off = off.mpc->run(state, 0ns); });
  CHECK(s_on == Status::kOk);
  CHECK(s_off == Status::kOk);
  MESSAGE("clock reads per tick: debug on " << reads_on << ", off "
                                            << reads_off);
  CHECK(reads_on - reads_off == 4);
  CHECK(on.mpc->ocp().results().states[1] ==
        off.mpc->ocp().results().states[1]);

  const auto& d = off.mpc->debug_data();
  CHECK(d.reference_id == -1);
  CHECK(d.ocp.nb_iter == 0);
  CHECK(d.duration_iteration == 0ns);
  CHECK(d.duration_horizon_update == 0ns);
  CHECK(d.duration_generate_warm_start == 0ns);
  CHECK(d.duration_ocp_solve == 0ns);

  // An early return before the solve: with debug data the timers are read,
  // without it not at all.
  auto failing_on = started_mpc(with, 6, 20, false);
  auto failing_off = started_mpc(without, 6, 20, false);
  CHECK(clock_reads([&] { s_on = failing_on.mpc->run(state, 0ns); }) == 2);
  CHECK(clock_reads([&] { s_off = failing_off.mpc->run(state, 0ns); }) == 0);
  CHECK(s_on == Status::kNoPreviousSolution);
  CHECK(s_off == Status::kNoPreviousSolution);
}

// ---- RT ---------------------------------------------------------------------

TEST_CASE("RtMpc.push_back: appends, full buffer, wrong size") {
  const auto params = make_params(kN);
  auto m = make_goal_reaching_mpc(params, kN + 2);
  std::vector<WeightedTrajectoryPoint> points;
  for (int k = 0; k < kN + 2; ++k) {
    points.push_back(moving_target_point(k, 10ms));
  }
  const auto wrong = make_trajectory_point(3, 3);
  Status s[kN + 4] = {};
  CHECK_RT_SAFE(for (int k = 0; k < kN + 2; ++k) {
    s[k] = m.mpc->push_back(points[static_cast<std::size_t>(k)]);
  });
  CHECK_RT_SAFE(s[kN + 2] = m.mpc->push_back(points[0]));
  CHECK_RT_SAFE(s[kN + 3] = m.mpc->push_back(wrong));
  for (int k = 0; k < kN + 2; ++k) CHECK(s[k] == Status::kOk);
  CHECK(s[kN + 2] == Status::kBufferFull);
  CHECK(s[kN + 3] == Status::kSizeMismatch);
  CHECK(m.mpc->buffer().size() == kN + 2);
  CHECK(m.mpc->buffer().back().id == kN + 1);
}

TEST_CASE("RtMpc.integrate: 100 steps and wrong sizes") {
  const auto params = make_params(kN);
  auto m = make_goal_reaching_mpc(params, kN + 2);
  RobotState state = panda_state(q_init());
  RobotState wrong = make_robot_state(6, 7);
  const Eigen::VectorXd u = Eigen::VectorXd::Constant(7, 0.1);
  const Eigen::VectorXd bad_u = Eigen::VectorXd::Zero(3);
  Status s[3] = {Status::kSizeMismatch, Status::kOk, Status::kOk};
  REQUIRE(m.mpc->integrate(state, u) == Status::kOk);  // warm-up
  CHECK_RT_SAFE(
      for (int i = 0; i < 100; ++i) { s[0] = m.mpc->integrate(state, u); });
  CHECK_RT_SAFE(s[1] = m.mpc->integrate(wrong, u));
  CHECK_RT_SAFE(s[2] = m.mpc->integrate(state, bad_u));
  CHECK(s[0] == Status::kOk);
  CHECK(s[1] == Status::kSizeMismatch);
  CHECK(s[2] == Status::kSizeMismatch);
  CHECK(state.time == 101 * 10ms);
}

TEST_CASE("RtMpc.run: early returns") {
  const auto params = make_params(kN);
  auto m = started_mpc(params, kN, 2 * (kN + 1), false);
  const RobotState state = panda_state(q_init());
  Status s[2] = {};
  // Warm-up of the error paths (nothing to warm: no solve).
  (void)m.mpc->run(state, 0ns);
  CHECK_RT_SAFE(s[0] = m.mpc->run(state, 0ns));
  CHECK(s[0] == Status::kNotEnoughData);
  REQUIRE(m.mpc->push_back(moving_target_point(kN, 10ms)) == Status::kOk);
  CHECK_RT_SAFE(s[1] = m.mpc->run(state, 0ns));
  CHECK(s[1] == Status::kNoPreviousSolution);
}

// R3: third-party allocation, see rt_safety.md Known violations. All
// 231,400 allocation calls of the 100 ticks (T4.5, gdb aggregation) happen
// inside Ocp::solve's SolverCSQP::solve: crocoddyl
// ResidualModelState::calcCostDiff, crocoddyl Stopwatch strings and Eigen
// resizes in SolverCSQP (forwardPass, tryStep, update_lagrangian_parameters,
// forwardPass_without_constraints), and libgomp teams in ShootingProblem
// calc/calcDiff. Mpc, the warm start, the buffer and the updaters allocate
// nothing.
namespace {

// The 100 ticks of RtMpc.run, with OcpParams::n_threads = `n_threads`.
void rt_run_100_ticks(int n_threads) {
  auto params = make_params(kN);
  params.n_threads = n_threads;
  constexpr int kTicks = 100;
  const int n_initial = 2 * (kN + 1);  // the ROS node starts at two horizons
  auto m = started_mpc(params, n_initial, static_cast<std::size_t>(n_initial));
  std::vector<WeightedTrajectoryPoint> points;
  for (int k = 0; k < kTicks + 1; ++k) {
    points.push_back(moving_target_point(n_initial + k, 10ms));
  }
  RobotState state = panda_state(q_init());
  // Warm-up: one full tick of the RT path.
  REQUIRE(m.mpc->run(state, state.time) == Status::kOk);
  REQUIRE(m.mpc->integrate(state, m.ocp->results().feed_forward_terms[0]) ==
          Status::kOk);
  REQUIRE(m.mpc->push_back(points[0]) == Status::kOk);

  Status s[kTicks][3] = {};
  CHECK_RT_SAFE(for (int k = 0; k < kTicks; ++k) {
    s[k][0] = m.mpc->run(state, state.time);
    s[k][1] = m.mpc->integrate(state, m.ocp->results().feed_forward_terms[0]);
    s[k][2] = m.mpc->push_back(points[static_cast<std::size_t>(k) + 1]);
  });
  int not_ok = 0;
  for (const auto& tick : s) {
    for (Status st : tick) not_ok += st != Status::kOk;
  }
  CHECK(not_ok == 0);
  CHECK(m.mpc->debug_data().reference_id == kTicks);
  CHECK(state.time == (kTicks + 1) * 10ms);
  for (const auto& x : m.ocp->results().states) CHECK(x.allFinite());
}

}  // namespace

// R3: third-party allocation, see rt_safety.md Known violations. All
// 231,400 allocation calls of the 100 ticks (T4.5, gdb aggregation) happen
// inside Ocp::solve's SolverCSQP::solve: crocoddyl
// ResidualModelState::calcCostDiff, crocoddyl Stopwatch strings and Eigen
// resizes in SolverCSQP (forwardPass, tryStep, update_lagrangian_parameters,
// forwardPass_without_constraints), and libgomp teams in ShootingProblem
// calc/calcDiff. Mpc, the warm start, the buffer and the updaters allocate
// nothing.
TEST_CASE("RtMpc.run: 100 ticks after warm-up" * doctest::may_fail()) {
  rt_run_100_ticks(1);
}

// R3: third-party allocation, see rt_safety.md Known violations. The same
// sites as the 1-thread test; the allocation hook counts per thread, so only
// the calling thread's allocations are counted (not the OpenMP workers').
TEST_CASE("RtMpc.run: n_threads 4" * doctest::may_fail()) {
  // The thread count must not be capped by OMP_THREAD_LIMIT.
  REQUIRE(omp_get_thread_limit() >= 4);
  rt_run_100_ticks(4);
}

TEST_CASE("Mpc.run: n_threads 4 gives the solution of n_threads 1") {
  REQUIRE(omp_get_thread_limit() >= 4);
  constexpr int kTicks = 100;
  const int n_initial = 2 * (kN + 1);
  auto run = [&](int n_threads) {
    auto params = make_params(kN);
    params.n_threads = n_threads;
    auto m =
        started_mpc(params, n_initial, static_cast<std::size_t>(n_initial));
    RobotState state = panda_state(q_init());
    std::vector<std::vector<Eigen::VectorXd>> xs, us;
    for (int k = 0; k < kTicks; ++k) {
      REQUIRE(m.mpc->run(state, state.time) == Status::kOk);
      const auto& r = m.ocp->results();
      xs.push_back(r.states);
      us.push_back(r.feed_forward_terms);
      REQUIRE(m.mpc->integrate(state, r.feed_forward_terms[0]) == Status::kOk);
      REQUIRE(m.mpc->push_back(moving_target_point(n_initial + k, 10ms)) ==
              Status::kOk);
    }
    return std::make_pair(xs, us);
  };
  const auto one = run(1);
  const auto four = run(4);
  double worst = 0.0;
  for (int k = 0; k < kTicks; ++k) {
    for (std::size_t i = 0; i < one.first[k].size(); ++i) {
      worst = std::max(worst, max_abs(one.first[k][i], four.first[k][i]));
    }
    for (std::size_t i = 0; i < one.second[k].size(); ++i) {
      worst = std::max(worst, max_abs(one.second[k][i], four.second[k][i]));
    }
  }
  MESSAGE("max |xs, us| difference over " << kTicks << " ticks: " << worst);
  CHECK(worst <= 1e-10);
}
