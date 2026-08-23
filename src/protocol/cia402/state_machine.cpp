#include "policy_runtime/protocol/cia402/state_machine.hpp"

namespace policy_runtime {

DriveState Cia402StateMachine::decode(std::uint16_t status_word) noexcept {
  switch (status_word & 0x004FU) {
    case 0x0000:
      return DriveState::not_ready_to_switch_on;
    case 0x0040:
      return DriveState::switch_on_disabled;
    case 0x000F:
      return DriveState::fault_reaction_active;
    case 0x0008:
      return DriveState::fault;
    default:
      break;
  }

  switch (status_word & 0x006FU) {
    case 0x0021:
      return DriveState::ready_to_switch_on;
    case 0x0023:
      return DriveState::switched_on;
    case 0x0027:
      return DriveState::operation_enabled;
    case 0x0007:
      return DriveState::quick_stop_active;
    default:
      return DriveState::unknown;
  }
}

Cia402StateOutput Cia402StateMachine::update(std::uint16_t status_word,
                                             AxisRequest request) noexcept {
  const auto state = decode(status_word);
  std::uint16_t control_word{};

  if (request != AxisRequest::fault_reset) {
    fault_reset_latched_ = false;
  }

  switch (request) {
    case AxisRequest::disable:
      break;
    case AxisRequest::quick_stop:
      if (state == DriveState::ready_to_switch_on || state == DriveState::switched_on ||
          state == DriveState::operation_enabled || state == DriveState::quick_stop_active) {
        control_word = 0x0002;
      }
      break;
    case AxisRequest::fault_reset:
      if (!fault_reset_latched_ && state == DriveState::fault) {
        control_word = 0x0080;
      }
      fault_reset_latched_ = true;
      break;
    case AxisRequest::enable:
      switch (state) {
        case DriveState::switch_on_disabled:
          control_word = 0x0006;
          break;
        case DriveState::ready_to_switch_on:
          control_word = 0x0007;
          break;
        case DriveState::switched_on:
        case DriveState::operation_enabled:
        case DriveState::quick_stop_active:
          control_word = 0x000F;
          break;
        case DriveState::not_ready_to_switch_on:
        case DriveState::fault_reaction_active:
        case DriveState::fault:
        case DriveState::unknown:
          break;
      }
      break;
  }

  return {state, control_word};
}

}  // namespace policy_runtime
