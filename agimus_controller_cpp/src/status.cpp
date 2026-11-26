#include "agimus_controller/status.hpp"

namespace agimus_controller {

const char* to_string(Status status) noexcept {
  switch (status) {
    case Status::kOk:
      return "ok";
    case Status::kNotEnoughData:
      return "not enough data";
    case Status::kBufferFull:
      return "buffer full";
    case Status::kUnknownFrame:
      return "unknown frame";
    case Status::kMissingTransform:
      return "missing transform";
    case Status::kSizeMismatch:
      return "size mismatch";
    case Status::kNoPreviousSolution:
      return "no previous solution";
  }
  return "unknown status";
}

}  // namespace agimus_controller
