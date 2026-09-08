#pragma once

#include <cstdint>
#include <utility>
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/cia402/pdo.hpp"
#include "policy_runtime/protocol/cia402/units.hpp"
#include "policy_runtime/robot_io/messages.hpp"

namespace policy_runtime {

class Cia402Sensor final {
 public:
  explicit Cia402Sensor(profiles::AxisConfig config) : config_(std::move(config)) {}

  AxisFeedback decode(const Cia402PdoView& pdo, std::uint64_t sequence,
                      std::int64_t timestamp_ns) const noexcept {
    AxisFeedback feedback{};
    feedback.sequence = sequence;
    feedback.timestamp_ns = timestamp_ns;
    feedback.position = device_units_to_engineering(pdo.actual_position, config_.scale).value_or(0.0);
    feedback.velocity = device_units_to_engineering(pdo.actual_velocity, config_.scale).value_or(0.0);
    feedback.effort = device_units_to_engineering(pdo.actual_torque, config_.scale).value_or(0.0);
    feedback.status_word = pdo.status_word;
    feedback.mode_display = static_cast<std::uint8_t>(pdo.mode_display);
    return feedback;
  }

 private:
  profiles::AxisConfig config_;
};

}  // namespace policy_runtime
