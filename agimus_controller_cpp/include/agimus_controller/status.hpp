#pragma once

namespace agimus_controller {

/// Result of a real-time (@rt) function. RT functions never throw.
enum class Status {
  kOk,
  kNotEnoughData,
  kBufferFull,
  kUnknownFrame,
  kMissingTransform,
  kSizeMismatch,
  kNoPreviousSolution,
};

/// @rt to_string
const char* to_string(Status status) noexcept;

}  // namespace agimus_controller
