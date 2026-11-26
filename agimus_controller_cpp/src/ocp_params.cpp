#include "agimus_controller/ocp_params.hpp"

#include <stdexcept>
#include <string>

namespace agimus_controller {

void OcpParams::finalize() {
  const auto& factors = dt_factor_n_seq.factors;
  const auto& n_steps = dt_factor_n_seq.n_steps;
  if (dt <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("OcpParams: dt must be > 0, got " +
                                std::to_string(dt.count()) + " ns.");
  }
  if (factors.empty() || factors.size() != n_steps.size()) {
    throw std::invalid_argument(
        "OcpParams: dt_factor_n_seq.factors and n_steps must be non-empty and "
        "of equal size, got " +
        std::to_string(factors.size()) + " and " +
        std::to_string(n_steps.size()) + ".");
  }
  for (std::size_t i = 0; i < factors.size(); ++i) {
    if (factors[i] < 0 || n_steps[i] < 0) {
      throw std::invalid_argument(
          "OcpParams: dt_factor_n_seq entries must be >= 0 (index " +
          std::to_string(i) + ").");
    }
  }
  if (solver_iters < 0 || qp_iters < 0 || n_threads < 1) {
    throw std::invalid_argument(
        "OcpParams: solver_iters and qp_iters must be >= 0 and n_threads >= "
        "1.");
  }

  timesteps.clear();
  for (std::size_t i = 0; i < factors.size(); ++i) {
    timesteps.insert(timesteps.end(), n_steps[i], dt * factors[i]);
  }
  n_controls = static_cast<int>(timesteps.size());
  total_time = std::chrono::nanoseconds::zero();
  for (const auto t : timesteps) {
    total_time += t;
  }
  if (horizon_size != n_controls) {
    throw std::invalid_argument(
        "OcpParams: horizon_size (" + std::to_string(horizon_size) +
        ") must equal the sum of dt_factor_n_seq.n_steps (" +
        std::to_string(n_controls) + ").");
  }
}

}  // namespace agimus_controller
