#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace policy_runtime {

// Shared static configuration used by the ST3215 sensor and actuator codecs
// and by the compatibility servo runtime.
struct St3215ServoConfig {
  std::string name;
  std::uint8_t device_id{};
  std::uint16_t servo_id{};
  std::uint16_t max_position_units{4095U};
  std::uint16_t speed_units{};
  std::uint16_t time_units{};
  std::chrono::milliseconds feedback_timeout{250};
  std::chrono::nanoseconds command_timeout{};
  std::chrono::nanoseconds maximum_command_future{};
};

}  // namespace policy_runtime
