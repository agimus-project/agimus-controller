// Port of agimus_controller/tests/test_ocp_param_base.py, plus validation.

#include <doctest/doctest.h>

#include <chrono>
#include <stdexcept>

#include "agimus_controller/ocp_params.hpp"

using agimus_controller::DtFactorsNSeq;
using agimus_controller::OcpParams;
using namespace std::chrono_literals;

namespace {

OcpParams make_params() {
  OcpParams p;
  p.dt = 10ms;
  p.horizon_size = 100;
  p.dt_factor_n_seq = DtFactorsNSeq{{1}, {100}};
  p.solver_iters = 50;
  p.qp_iters = 200;
  p.termination_tolerance = 1e-3;
  p.eps_abs = 1e-6;
  p.eps_rel = 0.0;
  p.callbacks = false;
  return p;
}

}  // namespace

TEST_CASE("OcpParams: initialization (test_ocp_param_base.py)") {
  auto p = make_params();
  p.finalize();
  CHECK(p.dt == 10ms);
  CHECK(p.horizon_size == 100);
  CHECK(p.solver_iters == 50);
  CHECK(p.qp_iters == 200);
  CHECK(p.termination_tolerance == 1e-3);
  CHECK(p.eps_abs == 1e-6);
  CHECK(p.eps_rel == 0.0);
  CHECK_FALSE(p.callbacks);
  CHECK(p.n_controls == p.horizon_size);
  REQUIRE(p.timesteps.size() == 100);
  for (const auto t : p.timesteps) {
    CHECK(t == 10ms);
  }
  CHECK(p.total_time == 1s);  // Exact: no floating-point sum.
}

TEST_CASE("OcpParams: defaults match the Python dataclass") {
  const OcpParams p;
  CHECK(p.qp_iters == 200);
  CHECK(p.termination_tolerance == 1e-3);
  CHECK_FALSE(p.max_solve_time.has_value());
  CHECK(p.eps_abs == 1e-6);
  CHECK(p.eps_rel == 0.0);
  CHECK_FALSE(p.callbacks);
  CHECK(p.use_debug_data);
  CHECK_FALSE(p.use_filter_line_search);  // B8: a settable field.
  CHECK(p.n_threads == 1);
}

TEST_CASE("OcpParams: time-varying steps keep the sequence order") {
  OcpParams p;
  p.dt = 100ms;
  p.dt_factor_n_seq = DtFactorsNSeq{{2, 1}, {2, 2}};
  p.horizon_size = 4;
  p.finalize();
  REQUIRE(p.timesteps.size() == 4);
  CHECK(p.timesteps[0] == 200ms);
  CHECK(p.timesteps[1] == 200ms);
  CHECK(p.timesteps[2] == 100ms);
  CHECK(p.timesteps[3] == 100ms);
  CHECK(p.total_time == 600ms);
}

TEST_CASE("OcpParams: finalize can be called again after a change") {
  auto p = make_params();
  p.finalize();
  p.dt_factor_n_seq = DtFactorsNSeq{{1, 2}, {20, 10}};
  p.horizon_size = 30;
  p.finalize();
  CHECK(p.n_controls == 30);
  CHECK(p.timesteps.size() == 30);
  CHECK(p.total_time == 400ms);
}

TEST_CASE("OcpParams: invalid inputs throw") {
  auto p = make_params();
  p.horizon_size = 99;
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.dt = 0ms;
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.dt = -1ms;
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.dt_factor_n_seq = DtFactorsNSeq{{1, 2}, {100}};
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.dt_factor_n_seq = DtFactorsNSeq{{}, {}};
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.dt_factor_n_seq = DtFactorsNSeq{{-1}, {100}};
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.n_threads = 0;
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);
}
