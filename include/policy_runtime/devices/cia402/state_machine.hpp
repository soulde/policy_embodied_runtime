#pragma once

#include <cstdint>

namespace policy_runtime {

enum class DriveState : std::uint8_t {
  not_ready_to_switch_on,
  switch_on_disabled,
  ready_to_switch_on,
  switched_on,
  operation_enabled,
  quick_stop_active,
  fault_reaction_active,
  fault,
  unknown,
};

enum class AxisRequest : std::uint8_t {
  disable,
  enable,
  quick_stop,
  fault_reset,
};

struct Cia402StateOutput {
  DriveState state{DriveState::unknown};
  std::uint16_t control_word{};
};

class Cia402StateMachine {
 public:
  static DriveState decode(std::uint16_t status_word) noexcept;

  Cia402StateOutput update(std::uint16_t status_word, AxisRequest request) noexcept;

 private:
  bool fault_reset_latched_{false};
};

}  // namespace policy_runtime
