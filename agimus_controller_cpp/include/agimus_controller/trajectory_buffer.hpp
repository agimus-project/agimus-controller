#pragma once

#include <cstddef>
#include <vector>

#include "agimus_controller/ocp_params.hpp"
#include "agimus_controller/status.hpp"
#include "agimus_controller/trajectory_point.hpp"

namespace agimus_controller {

/// Node index of each OCP node in the buffer: 0, then cumulative factors.
/// Example: factors {1, 2}, n_steps {2, 2} gives {0, 1, 2, 4, 6}.
/// Setup; throws std::invalid_argument on malformed input.
std::vector<std::size_t> compute_horizon_indexes(
    const DtFactorsNSeq& dt_factor_n_seq);

enum class BufferMode {
  /// FIFO of reference points: the MPC consumes one point per tick
  /// (Python TrajectoryBuffer).
  kSliding,
  /// Only the latest point is kept and repeated over the whole horizon, for
  /// teleoperation (Python ConstantTrajectoryBuffer).
  kConstant,
};

/// Reference points waiting to be used by the MPC. All storage is allocated
/// at construction; push_back copies into a preallocated slot.
class TrajectoryBuffer {
 public:
  /// Setup. `prototype` gives the vector sizes of every slot (see
  /// make_trajectory_point). In sliding mode `capacity` must exceed the last
  /// horizon index, otherwise the horizon could never be filled.
  TrajectoryBuffer(const DtFactorsNSeq& dt_factor_n_seq, std::size_t capacity,
                   BufferMode mode, const WeightedTrajectoryPoint& prototype);

  /// Sliding: appends a copy, kBufferFull when full. Constant: replaces the
  /// latest point. kSizeMismatch if `point` is not sized like the prototype.
  /// @rt TrajectoryBuffer::push_back
  Status push_back(const WeightedTrajectoryPoint& point) noexcept;

  /// Sliding: drops the oldest point (no-op when empty). Constant: no-op.
  /// @rt TrajectoryBuffer::clear_past
  void clear_past() noexcept;

  /// True when horizon() can be called.
  /// @rt TrajectoryBuffer::has_horizon
  bool has_horizon() const noexcept;

  /// The n_controls + 1 points of the OCP horizon (pointers into the buffer,
  /// valid until the next push_back or clear_past). Precondition:
  /// has_horizon(). Constant mode returns the latest point n_controls + 1
  /// times.
  /// @rt TrajectoryBuffer::horizon
  const std::vector<const WeightedTrajectoryPoint*>& horizon() noexcept;

  /// Sliding: number of stored points. Constant: 0 before the first point,
  /// then 3 * (horizon_indexes().back() + 1) so that the ROS node's
  /// "enough data" checks pass (as Python ConstantTrajectoryBuffer.__len__).
  /// @rt TrajectoryBuffer::size
  std::size_t size() const noexcept;

  /// i-th oldest point. Constant: always the latest point. Precondition:
  /// i < size() (sliding) or a point was received (constant).
  /// @rt TrajectoryBuffer::operator[]
  const WeightedTrajectoryPoint& operator[](std::size_t i) const noexcept;
  WeightedTrajectoryPoint& operator[](std::size_t i) noexcept;

  /// Newest point. Precondition: not empty.
  /// @rt TrajectoryBuffer::back
  const WeightedTrajectoryPoint& back() const noexcept;

  const std::vector<std::size_t>& horizon_indexes() const noexcept {
    return horizon_indexes_;
  }
  std::size_t capacity() const noexcept { return slots_.size(); }
  BufferMode mode() const noexcept { return mode_; }

 private:
  std::size_t slot_index(std::size_t i) const noexcept;

  BufferMode mode_;
  std::vector<std::size_t> horizon_indexes_;
  std::vector<WeightedTrajectoryPoint> slots_;
  std::vector<const WeightedTrajectoryPoint*> horizon_;
  std::size_t head_ = 0;   // Slot of the oldest point.
  std::size_t count_ = 0;  // Stored points (constant mode: 0 or 1).
};

}  // namespace agimus_controller
