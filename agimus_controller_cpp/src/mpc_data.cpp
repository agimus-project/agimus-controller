#include "agimus_controller/mpc_data.hpp"

namespace agimus_controller {

namespace {

template <typename T>
bool same_shapes(const std::vector<T>& a, const std::vector<T>& b) noexcept {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].rows() != b[i].rows() || a[i].cols() != b[i].cols()) {
      return false;
    }
  }
  return true;
}

template <typename T>
void copy_elements(const std::vector<T>& src, std::vector<T>& dst) noexcept {
  for (std::size_t i = 0; i < src.size(); ++i) {
    dst[i] = src[i];
  }
}

}  // namespace

void OcpResults::resize(std::size_t n_controls, Eigen::Index nx,
                        Eigen::Index ndx, Eigen::Index nu) {
  states.assign(n_controls + 1, Eigen::VectorXd::Zero(nx));
  riccati_gains.assign(n_controls, Eigen::MatrixXd::Zero(nu, ndx));
  feed_forward_terms.assign(n_controls, Eigen::VectorXd::Zero(nu));
}

Status copy_results(const OcpResults& src, OcpResults& dst) noexcept {
  if (!same_shapes(src.states, dst.states) ||
      !same_shapes(src.riccati_gains, dst.riccati_gains) ||
      !same_shapes(src.feed_forward_terms, dst.feed_forward_terms)) {
    return Status::kSizeMismatch;
  }
  copy_elements(src.states, dst.states);
  copy_elements(src.riccati_gains, dst.riccati_gains);
  copy_elements(src.feed_forward_terms, dst.feed_forward_terms);
  return Status::kOk;
}

}  // namespace agimus_controller
