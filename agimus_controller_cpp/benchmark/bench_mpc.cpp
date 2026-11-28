// Benchmark of Mpc::run on the goal-reaching OCP of golden set G10a
// (design.md, "Benchmark"). A plain executable, not a test: it never fails on
// time. See benchmark/CMakeLists.txt for how to build it.
//
//   bench_mpc [--threads N] [--mode agimus|croco] [--ticks N] [--warmup N]
//
// agimus: Mpc::run closed by Mpc::integrate, with the MpcDebugData phase
//         timers.
// croco:  the bare-crocoddyl baseline: the same models (the OCP's), a
//         ShootingProblem and the same solver called directly, warm-started
//         by shifting the previous solution. The references are set outside
//         the timed region (they are the agimus layer).
//
// The reference is G10a's: the hand target moves 5 mm along y per point
// (points 0..39 are G10a's). The measured run is longer than G10a's 20 ticks,
// so after 40 points the target comes back (a triangle wave of period 80
// points), which keeps it reachable.

#include <omp.h>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mim_solvers/csqp.hpp>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "agimus_controller/mpc.hpp"
#include "agimus_controller/ocp.hpp"
#include "mpc_fixtures.hpp"
#include "panda_ocp.hpp"

#ifndef AGIMUS_BENCH_COMMIT
#define AGIMUS_BENCH_COMMIT "unknown"
#endif
#ifndef AGIMUS_BENCH_BUILD_TYPE
#define AGIMUS_BENCH_BUILD_TYPE "unknown"
#endif
#ifndef AGIMUS_BENCH_COMPILER
#define AGIMUS_BENCH_COMPILER "unknown"
#endif

using namespace agimus_controller;
using namespace agimus_controller::test;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using std::chrono::nanoseconds;

namespace {

constexpr int kTriangleHalfPeriod = 40;  // points

// G10a's moving target, bouncing after point 40 (see the header).
WeightedTrajectoryPoint bench_point(int k) {
  WeightedTrajectoryPoint p = moving_target_point(0, 10ms);
  const int m = k % (2 * kTriangleHalfPeriod);
  const int steps = m <= kTriangleHalfPeriod ? m : 2 * kTriangleHalfPeriod - m;
  p.id = k;
  p.time = k * 10ms;
  p.ee_inputs[0].pose.translation() << 0.45, -0.05 + 0.005 * steps, 0.45;
  return p;
}

RobotState initial_state() {
  RobotState s = make_robot_state(7, 7);
  s.q << 0.1, -0.7, -0.1, -2.2, 0.05, 1.5, 0.7;  // G10a's initial state
  return s;
}

struct Tick {
  nanoseconds total{0};
  nanoseconds references{0}, warm_start{0}, solve{0};  // agimus only
  int iterations = 0, qp_iterations = 0;
};

nanoseconds percentile(std::vector<nanoseconds> v, double q) {
  std::sort(v.begin(), v.end());
  const auto rank =
      static_cast<std::size_t>(std::ceil(q * static_cast<double>(v.size())));
  return v[std::max<std::size_t>(rank, 1) - 1];
}

template <typename F>
std::vector<nanoseconds> column(const std::vector<Tick>& ticks, F&& f) {
  std::vector<nanoseconds> v;
  v.reserve(ticks.size());
  for (const Tick& t : ticks) v.push_back(f(t));
  return v;
}

double us(nanoseconds ns) { return static_cast<double>(ns.count()) / 1e3; }

double mean(const std::vector<Tick>& ticks, int Tick::* field) {
  double sum = 0.0;
  for (const Tick& t : ticks) sum += t.*field;
  return sum / static_cast<double>(ticks.size());
}

std::string read_first_line(const char* path) {
  std::ifstream f(path);
  std::string line;
  if (f) std::getline(f, line);
  return line.empty() ? "n/a" : line;
}

std::string cpu_model() {
  std::ifstream f("/proc/cpuinfo");
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("model name", 0) == 0) {
      const auto colon = line.find(':');
      return colon == std::string::npos ? line : line.substr(colon + 2);
    }
  }
  return "n/a";
}

std::string env(const char* name) {
  const char* v = std::getenv(name);
  return v == nullptr ? "(unset)" : v;
}

void print_conditions(int n_threads, int ticks, int warmup) {
  cpu_set_t set;
  CPU_ZERO(&set);
  sched_getaffinity(0, sizeof(set), &set);
  const char* policy = "other";
  switch (sched_getscheduler(0)) {
    case SCHED_FIFO:
      policy = "SCHED_FIFO";
      break;
    case SCHED_RR:
      policy = "SCHED_RR";
      break;
    case SCHED_OTHER:
      policy = "SCHED_OTHER";
      break;
    default:
      break;
  }
  double load[3] = {0, 0, 0};
  getloadavg(load, 3);
  std::printf("## Conditions\n\n");
  std::printf("- commit: %s\n- build type: %s\n- compiler: %s\n",
              AGIMUS_BENCH_COMMIT, AGIMUS_BENCH_BUILD_TYPE,
              AGIMUS_BENCH_COMPILER);
  std::printf("- CPU: %s (%ld online cores)\n", cpu_model().c_str(),
              sysconf(_SC_NPROCESSORS_ONLN));
  std::printf(
      "- n_threads: %d (omp_get_max_threads %d, omp_get_thread_limit %d)\n",
      n_threads, omp_get_max_threads(), omp_get_thread_limit());
  std::printf(
      "- OMP_NUM_THREADS=%s OMP_THREAD_LIMIT=%s OMP_PROC_BIND=%s "
      "OMP_PLACES=%s OMP_WAIT_POLICY=%s GOMP_SPINCOUNT=%s\n",
      env("OMP_NUM_THREADS").c_str(), env("OMP_THREAD_LIMIT").c_str(),
      env("OMP_PROC_BIND").c_str(), env("OMP_PLACES").c_str(),
      env("OMP_WAIT_POLICY").c_str(), env("GOMP_SPINCOUNT").c_str());
  std::printf(
      "- AC online: %s; governor (cpu0): %s; intel_pstate no_turbo: %s\n",
      read_first_line("/sys/class/power_supply/AC/online").c_str(),
      read_first_line("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
          .c_str(),
      read_first_line("/sys/devices/system/cpu/intel_pstate/no_turbo").c_str());
  std::printf("- affinity: %d cpu(s); nice %d; scheduler %s\n", CPU_COUNT(&set),
              getpriority(PRIO_PROCESS, 0), policy);
  std::printf("- load average at start: %.2f %.2f %.2f\n", load[0], load[1],
              load[2]);
  std::printf("- warm-up ticks %d, measured ticks %d\n\n", warmup, ticks);
}

void print_row(const char* label, int n_threads, const std::vector<Tick>& t,
               bool phases) {
  const auto total = column(t, [](const Tick& k) { return k.total; });
  const auto med = [&](auto f) { return us(percentile(column(t, f), 0.5)); };
  std::printf("| %s | %d | %.1f | %.1f | %.1f | %.1f |", label, n_threads,
              us(percentile(total, 0.5)), us(percentile(total, 0.9)),
              us(percentile(total, 0.99)), us(percentile(total, 1.0)));
  if (phases) {
    std::printf(" %.1f | %.1f | %.1f |",
                med([](const Tick& k) { return k.references; }),
                med([](const Tick& k) { return k.warm_start; }),
                med([](const Tick& k) { return k.solve; }));
  } else {
    std::printf(" - | %.1f | %.1f |",
                med([](const Tick& k) { return k.warm_start; }),
                med([](const Tick& k) { return k.solve; }));
  }
  std::printf(" %.2f | %.2f |\n", mean(t, &Tick::iterations),
              mean(t, &Tick::qp_iterations));
}

void print_header() {
  std::printf(
      "| run | n_threads | median us | p90 us | p99 us | max us | "
      "references us | warm start us | solve us | mean iter | mean QP iter "
      "|\n|---|---|---|---|---|---|---|---|---|---|---|\n");
}

// ---- Mpc::run closed by Mpc::integrate -------------------------------------

std::vector<Tick> run_agimus(const OcpParams& params, int n_ticks, int warmup) {
  const int n_initial = 2 * (params.n_controls + 1);  // as the ROS node
  const int total = warmup + n_ticks;
  GoalReachingMpc m =
      make_goal_reaching_mpc(params, static_cast<std::size_t>(n_initial));
  std::vector<WeightedTrajectoryPoint> points;
  for (int k = 0; k < n_initial + total + 1; ++k)
    points.push_back(bench_point(k));
  for (int k = 0; k < n_initial; ++k) {
    if (m.mpc->push_back(points[static_cast<std::size_t>(k)]) != Status::kOk) {
      throw std::runtime_error("push_back");
    }
  }
  RobotState state = initial_state();
  start_up(*m.mpc, *m.warm_start, state);  // the initial solve, no limits
  std::vector<Tick> ticks;
  ticks.reserve(static_cast<std::size_t>(n_ticks));
  for (int k = 0; k < total; ++k) {
    const auto t0 = Clock::now();
    const Status s1 = m.mpc->run(state, state.time);
    const Status s2 =
        m.mpc->integrate(state, m.ocp->results().feed_forward_terms[0]);
    const auto t1 = Clock::now();
    if (s1 != Status::kOk || s2 != Status::kOk) {
      throw std::runtime_error("Mpc::run or integrate failed");
    }
    m.mpc->push_back(points[static_cast<std::size_t>(n_initial + k)]);
    if (k >= warmup) {
      const MpcDebugData& d = m.mpc->debug_data();
      Tick t;
      t.total = t1 - t0;
      t.references = d.duration_horizon_update;
      t.warm_start = d.duration_generate_warm_start;
      t.solve = d.duration_ocp_solve;
      t.iterations = d.ocp.nb_iter;
      t.qp_iterations = d.ocp.nb_qp_iter;
      ticks.push_back(t);
    }
  }
  return ticks;
}

// ---- Bare crocoddyl ---------------------------------------------------------

std::vector<Tick> run_croco(const OcpParams& params, int n_ticks, int warmup) {
  const int n_initial = 2 * (params.n_controls + 1);
  const int total = warmup + n_ticks;
  GoalReachingMpc m =
      make_goal_reaching_mpc(params, static_cast<std::size_t>(n_initial));
  std::vector<WeightedTrajectoryPoint> points;
  for (int k = 0; k < n_initial + total + 1; ++k)
    points.push_back(bench_point(k));
  for (int k = 0; k < n_initial; ++k) {
    m.mpc->push_back(points[static_cast<std::size_t>(k)]);
  }
  RobotState state = initial_state();
  start_up(*m.mpc, *m.warm_start, state);  // the Ocp's initial solve

  // The bare problem: the OCP's models (their references are the ones the
  // Ocp updates), own datas, own solver of the same settings.
  const auto& running = m.ocp->problem()->get_runningModels();
  const auto terminal = m.ocp->problem()->get_terminalModel();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(m.ocp->nx());
  x0.head(7) = state.q;
  auto problem =
      std::make_shared<crocoddyl::ShootingProblem>(x0, running, terminal);
  problem->set_nthreads(params.n_threads);
  auto solver = make_solver(problem, params);
  auto* csqp = dynamic_cast<mim_solvers::SolverCSQP*>(solver.get());
  const std::size_t n = running.size();
  std::vector<Eigen::VectorXd> xs = m.ocp->results().states;
  std::vector<Eigen::VectorXd> us(n);
  for (std::size_t i = 0; i < n; ++i) {
    us[i] = m.ocp->results().feed_forward_terms[i];
  }
  Eigen::VectorXd x = Eigen::VectorXd::Zero(m.ocp->nx());
  Eigen::VectorXd x_next = x;
  const auto step_data = running[0]->createData();

  std::vector<Tick> ticks;
  for (int k = 0; k < total; ++k) {
    // Outside the timed region: the agimus layer.
    m.ocp->set_reference_weighted_trajectory(m.buffer->horizon());
    m.buffer->clear_past();
    x.head(7) = state.q;
    x.tail(7) = state.v;

    const auto t0 = Clock::now();
    // Warm start: shift the previous solution, repeat the last element.
    for (std::size_t i = 0; i + 1 < xs.size(); ++i) xs[i] = xs[i + 1];
    for (std::size_t i = 0; i + 1 < n; ++i) us[i] = us[i + 1];
    xs[0] = x;
    problem->set_x0(x);
    const auto t1 = Clock::now();
    solver->solve(xs, us, static_cast<std::size_t>(params.solver_iters));
    const auto t2 = Clock::now();
    const auto& sxs = solver->get_xs();
    const auto& sus = solver->get_us();
    for (std::size_t i = 0; i <= n; ++i) xs[i] = sxs[i];
    for (std::size_t i = 0; i < n; ++i) us[i] = sus[i];
    running[0]->calc(step_data, x, us[0]);  // integrate
    x_next = step_data->xnext;
    const auto t3 = Clock::now();
    state.q = x_next.head(7);
    state.v = x_next.tail(7);
    m.buffer->push_back(points[static_cast<std::size_t>(n_initial + k)]);
    if (k >= warmup) {
      Tick t;
      t.total = t3 - t0;
      t.warm_start = t1 - t0;
      t.solve = t2 - t1;
      t.iterations = static_cast<int>(solver->get_iter());
      t.qp_iterations = static_cast<int>(csqp->get_qp_iters());
      ticks.push_back(t);
    }
  }
  return ticks;
}

}  // namespace

int main(int argc, char** argv) {
  int n_threads = 1, n_ticks = 1000, warmup = 50;
  std::string mode = "agimus";
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string a = argv[i];
    if (a == "--threads")
      n_threads = std::atoi(argv[i + 1]);
    else if (a == "--mode")
      mode = argv[i + 1];
    else if (a == "--ticks")
      n_ticks = std::atoi(argv[i + 1]);
    else if (a == "--warmup")
      warmup = std::atoi(argv[i + 1]);
    else {
      std::fprintf(stderr, "unknown option %s\n", a.c_str());
      return 2;
    }
  }
  // G10a's parameters (make_params: dt 0.01, 20 nodes, 10 SQP iterations,
  // debug data on, as G10a).
  OcpParams params = make_params(kN);
  params.n_threads = n_threads;
  if (omp_get_thread_limit() < n_threads) {
    std::fprintf(stderr, "OMP_THREAD_LIMIT caps the threads below %d\n",
                 n_threads);
    return 1;
  }
  omp_set_num_threads(n_threads);
  print_conditions(n_threads, n_ticks, warmup);
  print_header();
  if (mode == "agimus") {
    print_row("agimus Mpc::run + integrate", n_threads,
              run_agimus(params, n_ticks, warmup), true);
  } else if (mode == "croco") {
    print_row("bare crocoddyl", n_threads, run_croco(params, n_ticks, warmup),
              false);
  } else {
    std::fprintf(stderr, "unknown mode %s\n", mode.c_str());
    return 2;
  }
  return 0;
}
