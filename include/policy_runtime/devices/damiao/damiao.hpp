#pragma once

#include <cstdint>

#include "policy_runtime/protocol/damiao/protocol.hpp"

namespace policy_runtime {

// Shared static identity and conversion limits for the sensor and actuator
// halves of one Damiao motor.
struct DamiaoConfig {
  std::uint8_t motor_id{};
  DamiaoLimits limits{};
};

}  // namespace policy_runtime
