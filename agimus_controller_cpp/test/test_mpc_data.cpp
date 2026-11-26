// Port of agimus_controller/tests/test_mpc_data.py, plus copy_results.

#include <chrono>

#include "agimus_controller/mpc_data.hpp"
#include "rt_guard.hpp"

using agimus_controller::copy_results;
using agimus_controller::MpcDebugData;
using agimus_controller::OcpDebugData;
using agimus_controller::OcpResults;
using agimus_controller::Status;

namespace {

constexpr std::size_t kN = 17;
constexpr Eigen::Index kNu = 12;
constexpr Eigen::Index kNx = 2 * kNu;

OcpResults make_random_results() {
  OcpResults r;
  r.resize(kN, kNx, kNx, kNu);
  for (auto& x : r.states) x.setRandom();
  for (auto& k : r.riccati_gains) k.setRandom();
  for (auto& u : r.feed_forward_terms) u.setRandom();
  return r;
}

}  // namespace

TEST_CASE("OcpResults: resize sizes every entry (test_ocp_results)") {
  OcpResults r;
  r.resize(kN, kNx, kNx, kNu);
  REQUIRE(r.states.size() == kN + 1);
  REQUIRE(r.riccati_gains.size() == kN);
  REQUIRE(r.feed_forward_terms.size() == kN);
  CHECK(r.states[0].size() == kNx);
  CHECK(r.riccati_gains[0].rows() == kNu);
  CHECK(r.riccati_gains[0].cols() == kNx);
  CHECK(r.feed_forward_terms[0].size() == kNu);
  CHECK(r.states[kN].isZero());

  // Resizing again to other sizes (setup) works.
  r.resize(3, 5, 4, 2);
  CHECK(r.states.size() == 4);
  CHECK(r.states[0].size() == 5);
  CHECK(r.riccati_gains[0].cols() == 4);
}

TEST_CASE("OcpDebugData and MpcDebugData: defaults and fields") {
  const OcpDebugData ocp;
  CHECK(ocp.result.states.empty());
  CHECK(ocp.references.empty());
  CHECK(ocp.residuals.empty());
  CHECK(ocp.kkt_norm == 0.0);
  CHECK(ocp.nb_iter == 0);
  CHECK(ocp.nb_qp_iter == 0);
  CHECK_FALSE(ocp.problem_solved);

  MpcDebugData mpc;
  CHECK(mpc.reference_id == -1);
  CHECK(mpc.duration_iteration == std::chrono::nanoseconds::zero());
  mpc.duration_iteration = std::chrono::nanoseconds(4);
  mpc.duration_horizon_update = std::chrono::nanoseconds(1);
  mpc.duration_generate_warm_start = std::chrono::nanoseconds(1);
  mpc.duration_ocp_solve = std::chrono::nanoseconds(2);
  CHECK(mpc.duration_iteration.count() == 4);
}

TEST_CASE("RtFree.copy_results: copies values of equal sizes") {
  const OcpResults src = make_random_results();
  OcpResults dst;
  dst.resize(kN, kNx, kNx, kNu);
  Status status = Status::kSizeMismatch;
  CHECK_RT_SAFE(status = copy_results(src, dst));
  CHECK(status == Status::kOk);
  CHECK(dst.states == src.states);
  CHECK(dst.riccati_gains == src.riccati_gains);
  CHECK(dst.feed_forward_terms == src.feed_forward_terms);
}

TEST_CASE("RtFree.copy_results: size mismatch leaves dst unchanged") {
  const OcpResults src = make_random_results();
  OcpResults dst;
  dst.resize(kN, kNx, kNx, kNu + 1);
  const OcpResults before = dst;
  Status status = Status::kOk;
  CHECK_RT_SAFE(status = copy_results(src, dst));
  CHECK(status == Status::kSizeMismatch);
  CHECK(dst.feed_forward_terms == before.feed_forward_terms);

  OcpResults shorter;
  shorter.resize(kN - 1, kNx, kNx, kNu);
  CHECK_RT_SAFE(status = copy_results(src, shorter));
  CHECK(status == Status::kSizeMismatch);
}
