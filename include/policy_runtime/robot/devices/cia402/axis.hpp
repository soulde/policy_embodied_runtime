#pragma once

#include <cstdint>
#include <optional>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/cia402/pdo.hpp"
#include "policy_runtime/protocol/cia402/state_machine.hpp"
#include "policy_runtime/robot_io/ipc_protocol.hpp"

namespace policy_runtime {

inline constexpr std::uint32_t kAxisCommandEnable = 1U << 0U;
inline constexpr std::uint32_t kAxisCommandDisable = 1U << 1U;
inline constexpr std::uint32_t kAxisCommandQuickStop = 1U << 2U;
inline constexpr std::uint32_t kAxisCommandFaultReset = 1U << 3U;

inline constexpr std::uint32_t kAxisFeedbackTargetClamped = 1U << 0U;
inline constexpr std::uint32_t kAxisFeedbackSlewLimited = 1U << 1U;
inline constexpr std::uint32_t kAxisFeedbackFollowingError = 1U << 2U;
inline constexpr std::uint32_t kAxisFeedbackModeMismatch = 1U << 3U;
inline constexpr std::uint32_t kAxisFeedbackInvalidCommand = 1U << 4U;

class Cia402Axis {
 public:
  // Construction occurs before the realtime loop and rejects unsafe limits.
  explicit Cia402Axis(profiles::AxisConfig config);

  AxisFeedback cycle(const AxisCommand& command, Cia402PdoView& pdo) noexcept;
  Result<void> verify_mode(std::int8_t mode_display) const;

 private:
  std::optional<double> actual_setpoint(const Cia402PdoView& pdo) const noexcept;
  bool stage_setpoint(double value, Cia402PdoView& pdo) const noexcept;

  profiles::AxisConfig config_;
  Cia402StateMachine state_machine_;
  double last_target_{};
  bool has_last_target_{false};
};

}  // namespace policy_runtime
