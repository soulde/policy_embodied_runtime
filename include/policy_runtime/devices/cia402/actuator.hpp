#pragma once

#include <cmath>
#include <cstdint>
#include <utility>
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/cia402/pdo.hpp"
#include "policy_runtime/protocol/cia402/units.hpp"
#include "policy_runtime/robot_io/messages.hpp"

namespace policy_runtime {

class Cia402Actuator final {
 public:
  explicit Cia402Actuator(profiles::AxisConfig config) : config_(std::move(config)) {}

  Result<void> encode_setpoint(const AxisCommand& command,
                               Cia402PdoView& pdo) const noexcept {
    pdo.target_position = 0;
    pdo.target_velocity = 0;
    pdo.target_torque = 0;
    if (!std::isfinite(command.target) || config_.scale <= 0.0) {
      return Result<void>::failure({ErrorCode::invalid_argument,
                                    "invalid CiA 402 actuator target"});
    }
    switch (config_.mode) {
      case profiles::Cia402Mode::csp: {
        auto value = position_to_device_units(command.target, config_.scale);
        if (!value) return Result<void>::failure({ErrorCode::invalid_argument, "invalid CSP target"});
        pdo.target_position = *value;
        break;
      }
      case profiles::Cia402Mode::csv: {
        auto value = velocity_to_device_units(command.target, config_.scale);
        if (!value) return Result<void>::failure({ErrorCode::invalid_argument, "invalid CSV target"});
        pdo.target_velocity = *value;
        break;
      }
      case profiles::Cia402Mode::cst: {
        auto value = torque_to_device_units(command.target, config_.scale);
        if (!value) return Result<void>::failure({ErrorCode::invalid_argument, "invalid CST target"});
        pdo.target_torque = *value;
        break;
      }
    }
    return Result<void>::success();
  }

 private:
  profiles::AxisConfig config_;
};

}  // namespace policy_runtime
