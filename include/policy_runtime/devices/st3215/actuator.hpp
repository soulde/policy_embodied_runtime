#pragma once

#include <utility>

#include "policy_runtime/devices/st3215/servo.hpp"
#include "policy_runtime/protocol/st3215/protocol.hpp"

namespace policy_runtime {

class St3215Actuator final {
 public:
  explicit St3215Actuator(St3215ServoConfig config) : config_(std::move(config)) {}

  Result<Bytes> encode(const St3215ServoCommand& command) const {
    if (!command.enabled || command.emergency_stop) {
      return Result<Bytes>::success(St3215Protocol::read_present_position_command(
          config_.device_id));
    }
    auto position = radians_to_position_units(command.target_position_rad,
                                               config_.max_position_units);
    if (!position.has_value()) return Result<Bytes>::failure(position.error());
    return Result<Bytes>::success(St3215Protocol::goal_position_command(
        config_.device_id, position.value(), config_.time_units,
        config_.speed_units));
  }

 private:
  St3215ServoConfig config_;
};

}  // namespace policy_runtime
