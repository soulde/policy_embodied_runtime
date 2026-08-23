#include "policy_runtime/robot/devices/cia402/axis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "policy_runtime/protocol/cia402/units.hpp"

namespace policy_runtime {
namespace {

constexpr std::uint32_t kKnownCommandFlags = kAxisCommandEnable | kAxisCommandDisable |
                                               kAxisCommandQuickStop |
                                               kAxisCommandFaultReset;

unsigned flag_count(std::uint32_t flags) noexcept {
  unsigned count{};
  while (flags != 0U) {
    count += flags & 1U;
    flags >>= 1U;
  }
  return count;
}

AxisRequest requested_action(std::uint32_t flags, bool& invalid) noexcept {
  invalid = (flags & ~kKnownCommandFlags) != 0U || flag_count(flags & kKnownCommandFlags) > 1U;
  if (invalid) {
    return AxisRequest::quick_stop;
  }
  if ((flags & kAxisCommandQuickStop) != 0U) {
    return AxisRequest::quick_stop;
  }
  if ((flags & kAxisCommandFaultReset) != 0U) {
    return AxisRequest::fault_reset;
  }
  if ((flags & kAxisCommandEnable) != 0U) {
    return AxisRequest::enable;
  }
  return AxisRequest::disable;
}

bool valid_positive_limit(double value) noexcept {
  return (std::isfinite(value) && value > 0.0) ||
         value == std::numeric_limits<double>::infinity();
}

}  // namespace

Cia402Axis::Cia402Axis(profiles::AxisConfig config) : config_(std::move(config)) {}

Result<void> Cia402Axis::verify_mode(std::int8_t mode_display) const {
  if (mode_display != static_cast<std::int8_t>(config_.mode)) {
    return Result<void>::failure(
        {ErrorCode::protocol, "CiA 402 mode display differs from static configuration"});
  }
  return Result<void>::success();
}

double Cia402Axis::actual_setpoint(const Cia402PdoView& pdo) const noexcept {
  switch (config_.mode) {
    case profiles::Cia402Mode::csp:
      return device_units_to_engineering(pdo.actual_position, config_.scale);
    case profiles::Cia402Mode::csv:
      return device_units_to_engineering(pdo.actual_velocity, config_.scale);
    case profiles::Cia402Mode::cst:
      return device_units_to_engineering(pdo.actual_torque, config_.scale);
  }
  return 0.0;
}

bool Cia402Axis::stage_setpoint(double value, Cia402PdoView& pdo) const noexcept {
  pdo.target_position = 0;
  pdo.target_velocity = 0;
  pdo.target_torque = 0;
  switch (config_.mode) {
    case profiles::Cia402Mode::csp: {
      const auto converted = position_to_device_units(value, config_.scale);
      if (!converted) {
        return false;
      }
      pdo.target_position = *converted;
      return true;
    }
    case profiles::Cia402Mode::csv: {
      const auto converted = velocity_to_device_units(value, config_.scale);
      if (!converted) {
        return false;
      }
      pdo.target_velocity = *converted;
      return true;
    }
    case profiles::Cia402Mode::cst: {
      const auto converted = torque_to_device_units(value, config_.scale);
      if (!converted) {
        return false;
      }
      pdo.target_torque = *converted;
      return true;
    }
  }
  return false;
}

AxisFeedback Cia402Axis::cycle(const AxisCommand& command, Cia402PdoView& pdo) noexcept {
  AxisFeedback feedback{};
  feedback.sequence = command.sequence;
  feedback.timestamp_ns = command.timestamp_ns;
  feedback.position = device_units_to_engineering(pdo.actual_position, config_.scale);
  feedback.velocity = device_units_to_engineering(pdo.actual_velocity, config_.scale);
  feedback.effort = device_units_to_engineering(pdo.actual_torque, config_.scale);
  feedback.status_word = pdo.status_word;
  feedback.mode_display = static_cast<std::uint8_t>(pdo.mode_display);

  bool invalid_flags{};
  auto request = requested_action(command.flags, invalid_flags);
  if (invalid_flags) {
    feedback.flags |= kAxisFeedbackInvalidCommand;
  }

  const double actual = actual_setpoint(pdo);
  if (!has_last_target_) {
    last_target_ = actual;
    has_last_target_ = true;
  }
  double staged_target = actual;

  if (request == AxisRequest::enable) {
    if (!std::isfinite(command.target) || !std::isfinite(config_.scale) ||
        config_.scale <= 0.0 || !std::isfinite(config_.minimum) ||
        !std::isfinite(config_.maximum) || config_.minimum > config_.maximum ||
        !valid_positive_limit(config_.slew_limit) ||
        !valid_positive_limit(config_.following_error_limit)) {
      feedback.flags |= kAxisFeedbackInvalidCommand;
      request = AxisRequest::quick_stop;
    } else {
      staged_target = std::clamp(command.target, config_.minimum, config_.maximum);
      if (staged_target != command.target) {
        feedback.flags |= kAxisFeedbackTargetClamped;
      }
      if (std::isfinite(config_.slew_limit)) {
        const double lower = last_target_ - config_.slew_limit;
        const double upper = last_target_ + config_.slew_limit;
        const double slew_target = std::clamp(staged_target, lower, upper);
        if (slew_target != staged_target) {
          feedback.flags |= kAxisFeedbackSlewLimited;
        }
        staged_target = slew_target;
      }
      if (Cia402StateMachine::decode(pdo.status_word) == DriveState::operation_enabled &&
          std::isfinite(config_.following_error_limit) &&
          std::abs(staged_target - actual) > config_.following_error_limit) {
        feedback.flags |= kAxisFeedbackFollowingError;
        request = AxisRequest::quick_stop;
        staged_target = actual;
      }
    }
  }

  if (pdo.mode_display != static_cast<std::int8_t>(config_.mode)) {
    feedback.flags |= kAxisFeedbackModeMismatch;
    if (request == AxisRequest::enable) {
      request = AxisRequest::quick_stop;
      staged_target = actual;
    }
  }

  if (!stage_setpoint(staged_target, pdo)) {
    feedback.flags |= kAxisFeedbackInvalidCommand;
    request = AxisRequest::quick_stop;
    static_cast<void>(stage_setpoint(0.0, pdo));
  }

  if (request == AxisRequest::enable) {
    last_target_ = staged_target;
  } else {
    last_target_ = actual;
  }

  pdo.control_word = state_machine_.update(pdo.status_word, request).control_word;
  return feedback;
}

}  // namespace policy_runtime
