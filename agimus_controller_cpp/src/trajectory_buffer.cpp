#include "agimus_controller/trajectory_buffer.hpp"

#include <cassert>
#include <stdexcept>
#include <string>

namespace agimus_controller {

std::vector<std::size_t> compute_horizon_indexes(
    const DtFactorsNSeq& dt_factor_n_seq) {
  const auto& factors = dt_factor_n_seq.factors;
  const auto& n_steps = dt_factor_n_seq.n_steps;
  if (factors.size() != n_steps.size()) {
    throw std::invalid_argument(
        "compute_horizon_indexes: factors and n_steps differ in size.");
  }
  std::vector<std::size_t> indexes{0};
  for (std::size_t i = 0; i < factors.size(); ++i) {
    if (factors[i] < 0 || n_steps[i] < 0) {
      throw std::invalid_argument(
          "compute_horizon_indexes: entries must be >= 0 (index " +
          std::to_string(i) + ").");
    }
    for (int k = 0; k < n_steps[i]; ++k) {
      indexes.push_back(indexes.back() + static_cast<std::size_t>(factors[i]));
    }
  }
  return indexes;
}

TrajectoryBuffer::TrajectoryBuffer(const DtFactorsNSeq& dt_factor_n_seq,
                                   std::size_t capacity, BufferMode mode,
                                   const WeightedTrajectoryPoint& prototype)
    : mode_(mode), horizon_indexes_(compute_horizon_indexes(dt_factor_n_seq)) {
  if (mode_ == BufferMode::kConstant) {
    capacity = 1;
  } else if (capacity <= horizon_indexes_.back()) {
    throw std::invalid_argument("TrajectoryBuffer: capacity (" +
                                std::to_string(capacity) +
                                ") must exceed the last horizon index (" +
                                std::to_string(horizon_indexes_.back()) + ").");
  }
  slots_.assign(capacity, prototype);
  horizon_.assign(horizon_indexes_.size(), nullptr);
}

std::size_t TrajectoryBuffer::slot_index(std::size_t i) const noexcept {
  if (mode_ == BufferMode::kConstant) {
    return 0;
  }
  const std::size_t s = head_ + i;
  return s < slots_.size() ? s : s - slots_.size();
}

Status TrajectoryBuffer::push_back(
    const WeightedTrajectoryPoint& point) noexcept {
  if (!same_sizes(point, slots_[0])) {
    return Status::kSizeMismatch;
  }
  if (mode_ == BufferMode::kConstant) {
    slots_[0] = point;
    count_ = 1;
    return Status::kOk;
  }
  if (count_ == slots_.size()) {
    return Status::kBufferFull;
  }
  slots_[slot_index(count_)] = point;
  ++count_;
  return Status::kOk;
}

void TrajectoryBuffer::clear_past() noexcept {
  if (mode_ == BufferMode::kConstant || count_ == 0) {
    return;
  }
  head_ = slot_index(1);
  --count_;
}

bool TrajectoryBuffer::has_horizon() const noexcept {
  if (mode_ == BufferMode::kConstant) {
    return count_ == 1;
  }
  return count_ > horizon_indexes_.back();
}

const std::vector<const WeightedTrajectoryPoint*>&
TrajectoryBuffer::horizon() noexcept {
  assert(has_horizon());
  for (std::size_t k = 0; k < horizon_indexes_.size(); ++k) {
    horizon_[k] = &slots_[slot_index(horizon_indexes_[k])];
  }
  return horizon_;
}

std::size_t TrajectoryBuffer::size() const noexcept {
  if (mode_ == BufferMode::kConstant) {
    return count_ == 0 ? 0 : 3 * (horizon_indexes_.back() + 1);
  }
  return count_;
}

const WeightedTrajectoryPoint& TrajectoryBuffer::operator[](
    std::size_t i) const noexcept {
  assert(mode_ == BufferMode::kConstant ? count_ == 1 : i < count_);
  return slots_[slot_index(i)];
}

WeightedTrajectoryPoint& TrajectoryBuffer::operator[](std::size_t i) noexcept {
  assert(mode_ == BufferMode::kConstant ? count_ == 1 : i < count_);
  return slots_[slot_index(i)];
}

const WeightedTrajectoryPoint& TrajectoryBuffer::back() const noexcept {
  assert(count_ > 0);
  return slots_[slot_index(count_ - 1)];
}

}  // namespace agimus_controller
