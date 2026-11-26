#pragma once

#include <Eigen/Core>
#include <chrono>
#include <string>
#include <vector>

#include "agimus_controller/status.hpp"

namespace agimus_controller {

/// Solution of one OCP solve (Python OCPResults).
struct OcpResults {
  std::vector<Eigen::VectorXd> states;              // n_controls + 1, size nx
  std::vector<Eigen::MatrixXd> riccati_gains;       // n_controls, nu x ndx
  std::vector<Eigen::VectorXd> feed_forward_terms;  // n_controls, size nu

  /// Setup: sizes every entry (zeros).
  void resize(std::size_t n_controls, Eigen::Index nx, Eigen::Index ndx,
              Eigen::Index nu);
};

/// Copies `src` into `dst` element by element. Returns kSizeMismatch (and
/// leaves `dst` unchanged) unless both have the same sizes.
/// @rt copy_results
Status copy_results(const OcpResults& src, OcpResults& dst) noexcept;

/// A named matrix of debug output. The name and size are set at setup.
struct NamedMatrix {
  std::string name;
  Eigen::MatrixXd data;
};

/// Debug output of the OCP (Python OCPDebugData).
struct OcpDebugData {
  OcpResults result;
  /// Cost references at node 0: a vector is n x 1, an SE3 is 7 x 1 (XYZQUAT),
  /// a rotation is 3 x 3.
  std::vector<NamedMatrix> references;
  /// Published residuals, n_controls x nr each.
  std::vector<NamedMatrix> residuals;
  double kkt_norm = 0.0;
  int nb_iter = 0;
  int nb_qp_iter = 0;
  bool problem_solved = false;
};

/// Debug output of one MPC iteration (Python MPCDebugData).
struct MpcDebugData {
  OcpDebugData ocp;
  int reference_id = -1;
  // Measured with std::chrono::steady_clock.
  std::chrono::nanoseconds duration_iteration{0};
  std::chrono::nanoseconds duration_horizon_update{0};
  std::chrono::nanoseconds duration_generate_warm_start{0};
  std::chrono::nanoseconds duration_ocp_solve{0};
};

}  // namespace agimus_controller
