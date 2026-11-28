// Port of agimus_controller/tests/test_ocp_param_base.py, plus validation and
// the golden set G1 (test/golden/g1_ocp_params.toml).

#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>
#include <vector>

#include "agimus_controller/ocp_params.hpp"

using agimus_controller::DtFactorsNSeq;
using agimus_controller::OcpParams;

namespace {

template <typename T>
std::vector<T> read_array(const toml::table& table, const char* key) {
  const toml::array* array = table[key].as_array();
  REQUIRE_MESSAGE(array != nullptr, "missing array " << key);
  std::vector<T> out;
  for (const toml::node& node : *array) {
    const std::optional<T> value = node.value<T>();
    REQUIRE_MESSAGE(value.has_value(), "bad element in " << key);
    out.push_back(*value);
  }
  return out;
}

OcpParams make_params() {
  OcpParams p;
  p.dt = 0.01;
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
  CHECK(p.dt == 0.01);
  CHECK(p.horizon_size == 100);
  CHECK(p.solver_iters == 50);
  CHECK(p.qp_iters == 200);
  CHECK(p.termination_tolerance == 1e-3);
  CHECK(p.eps_abs == 1e-6);
  CHECK(p.eps_rel == 0.0);
  CHECK_FALSE(p.callbacks);
  CHECK(p.n_controls == p.horizon_size);
  REQUIRE(p.timesteps.size() == 100);
  for (const double t : p.timesteps) {
    CHECK(t == 0.01);  // dt * 1 is exact.
  }
  CHECK(p.total_time == doctest::Approx(1.0));
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
  p.dt = 0.1;
  p.dt_factor_n_seq = DtFactorsNSeq{{2, 1}, {2, 2}};
  p.horizon_size = 4;
  p.finalize();
  REQUIRE(p.timesteps.size() == 4);
  CHECK(p.timesteps[0] == doctest::Approx(0.2));
  CHECK(p.timesteps[1] == doctest::Approx(0.2));
  CHECK(p.timesteps[2] == doctest::Approx(0.1));
  CHECK(p.timesteps[3] == doctest::Approx(0.1));
  CHECK(p.total_time == doctest::Approx(0.6));
}

TEST_CASE("OcpParams: finalize can be called again after a change") {
  auto p = make_params();
  p.finalize();
  p.dt_factor_n_seq = DtFactorsNSeq{{1, 2}, {20, 10}};
  p.horizon_size = 30;
  p.finalize();
  CHECK(p.n_controls == 30);
  CHECK(p.timesteps.size() == 30);
  CHECK(p.total_time == doctest::Approx(0.4));
}

TEST_CASE("OcpParams: invalid inputs throw") {
  auto p = make_params();
  p.horizon_size = 99;
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.dt = 0.0;
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.dt = -0.001;
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.dt = std::nan("");
  CHECK_THROWS_AS(p.finalize(), std::invalid_argument);

  p = make_params();
  p.max_solve_time = 0.0;
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

namespace {

/// One G1 case, finalized like Python's OCPParamsBaseCroco.
struct G1Case {
  OcpParams params;
  const toml::table* golden;
};

std::vector<G1Case> load_g1() {
  static const toml::table golden =
      toml::parse_file(std::string(AGIMUS_GOLDEN_DIR) + "/g1_ocp_params.toml");
  REQUIRE(golden["meta"]["set"].value<std::string>() == "G1");
  const std::optional<double> dt = golden["inputs"]["dt"].value<double>();
  REQUIRE(dt.has_value());
  const toml::array* cases = golden["cases"].as_array();
  REQUIRE(cases != nullptr);
  REQUIRE(cases->size() == 3);

  std::vector<G1Case> out;
  for (const toml::node& node : *cases) {
    const toml::table* c = node.as_table();
    REQUIRE(c != nullptr);
    OcpParams p;
    p.dt = *dt;
    for (const std::int64_t f : read_array<std::int64_t>(*c, "factors")) {
      p.dt_factor_n_seq.factors.push_back(static_cast<int>(f));
    }
    for (const std::int64_t n : read_array<std::int64_t>(*c, "n_steps")) {
      p.dt_factor_n_seq.n_steps.push_back(static_cast<int>(n));
    }
    p.horizon_size =
        static_cast<int>(*(*c)["horizon_size"].value<std::int64_t>());
    p.finalize();
    out.push_back({p, c});
  }
  return out;
}

}  // namespace

TEST_CASE("OcpParams: golden set G1, n_controls and timesteps") {
  for (const G1Case& g : load_g1()) {
    const OcpParams& p = g.params;
    const toml::table& c = *g.golden;
    INFO("number of factors: " << p.dt_factor_n_seq.factors.size());
    CHECK(p.n_controls == *c["n_controls"].value<std::int64_t>());
    // Same IEEE operation as Python (dt * factor): exact comparison.
    const std::vector<double> timesteps = read_array<double>(c, "timesteps");
    REQUIRE(p.timesteps.size() == timesteps.size());
    for (std::size_t i = 0; i < timesteps.size(); ++i) {
      CHECK(p.timesteps[i] == timesteps[i]);
    }
  }
}

// finalize() reproduces CPython >= 3.12's compensated float sum, so the
// comparison is exact (ledger, T4.1 ruling).
TEST_CASE("OcpParams: golden set G1, total_time") {
  for (const G1Case& g : load_g1()) {
    INFO("number of factors: " << g.params.dt_factor_n_seq.factors.size());
    CHECK(g.params.total_time == *(*g.golden)["total_time"].value<double>());
  }
}
