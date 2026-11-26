#pragma once

#include <chrono>
#include <optional>
#include <vector>

namespace agimus_controller {

/// Time-varying steps of the OCP: `n_steps[i]` nodes of length
/// `factors[i] * dt`, in order. Example: dt = 0.1, factors = {2, 1},
/// n_steps = {2, 2} gives the time steps {0.2, 0.2, 0.1, 0.1}.
struct DtFactorsNSeq {
  std::vector<int> factors;
  std::vector<int> n_steps;
};

/// Parameters of the OCP and of its CSQP solver (Python OCPParamsBaseCroco).
/// Fill the inputs, then call finalize().
struct OcpParams {
  // Inputs.
  std::chrono::nanoseconds dt{0};  // Base integration step.
  int solver_iters = 0;            // SQP iterations per solve.
  DtFactorsNSeq dt_factor_n_seq;
  int horizon_size = 0;  // Must equal the sum of n_steps (sanity check).
  int qp_iters = 200;    // QP iterations (multiple of 25).
  double termination_tolerance = 1e-3;                     // KKT norm.
  std::optional<std::chrono::nanoseconds> max_solve_time;  // Empty: none.
  double eps_abs = 1e-6;
  double eps_rel = 0.0;
  bool callbacks = false;  // Verbose solver callbacks (prints: not RT).
  bool use_debug_data = true;
  bool use_filter_line_search = false;
  int n_threads = 1;

  // Derived by finalize().
  int n_controls = 0;
  std::vector<std::chrono::nanoseconds> timesteps;  // n_controls entries.
  std::chrono::nanoseconds total_time{0};           // Exact sum.

  /// Setup: validates the inputs and computes the derived fields. Throws
  /// std::invalid_argument naming the offending field.
  void finalize();
};

}  // namespace agimus_controller
