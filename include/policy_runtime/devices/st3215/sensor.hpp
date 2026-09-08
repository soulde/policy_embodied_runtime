#pragma once

#include <cstdint>
#include <utility>

#include "policy_runtime/devices/st3215/servo.hpp"
#include "policy_runtime/devices/st3215/protocol.hpp"

namespace policy_runtime {

class St3215Sensor final {
 public:
  explicit St3215Sensor(St3215ServoConfig config) : config_(std::move(config)) {}

  Result<St3215ServoFeedback> decode(const St3215Status& status,
                                     std::int64_t timestamp_ns,
                                     std::uint64_t command_sequence) const {
    St3215ServoFeedback feedback{};
    feedback.feedback_sequence = 1U;
    feedback.command_sequence = command_sequence;
    feedback.timestamp_ns = timestamp_ns;
    feedback.status_error = status.error;
    feedback.flags = status.error == 0U ? kSt3215FeedbackValid
                                        : kSt3215FeedbackDeviceError;
    if (status.device_id != config_.device_id) {
      return Result<St3215ServoFeedback>::failure(
          {ErrorCode::protocol, "ST3215 response device ID does not match sensor"});
    }
    if (status.parameters.size() >= 2U) {
      auto raw = read_st3215_u16_le(status.parameters);
      if (!raw.has_value()) return Result<St3215ServoFeedback>::failure(raw.error());
      auto radians = position_units_to_radians(raw.value(), config_.max_position_units);
      if (!radians.has_value()) return Result<St3215ServoFeedback>::failure(radians.error());
      feedback.raw_position = raw.value();
      feedback.position_rad = radians.value();
    }
    return Result<St3215ServoFeedback>::success(feedback);
  }

 private:
  St3215ServoConfig config_;
};

}  // namespace policy_runtime
