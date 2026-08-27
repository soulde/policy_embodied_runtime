#pragma once

#include <cstdint>

namespace policy_runtime {

// Typed protocol-side staging values. EtherCAT owns the process image and
// copies these values to and from its registered fields during Transport::cycle().
struct Cia402PdoView {
  std::uint16_t status_word{};
  std::int8_t mode_display{};
  std::int32_t actual_position{};
  std::int32_t actual_velocity{};
  std::int16_t actual_torque{};

  std::uint16_t control_word{};
  std::int32_t target_position{};
  std::int32_t target_velocity{};
  std::int16_t target_torque{};
};

}  // namespace policy_runtime
