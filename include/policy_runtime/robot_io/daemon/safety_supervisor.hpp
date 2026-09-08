#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/cia402/pdo.hpp"
#include "policy_runtime/protocol/cia402/state_machine.hpp"
#include "policy_runtime/robot_io/daemon/messages.hpp"
#include "policy_runtime/robot_io/daemon/value_snapshot.hpp"

namespace policy_runtime {

inline constexpr std::uint32_t kAxisFeedbackSafetyTimeout = 1U << 16U;
inline constexpr std::uint32_t kAxisFeedbackSafetyWkc = 1U << 17U;
inline constexpr std::uint32_t kAxisFeedbackSafetyLink = 1U << 18U;
inline constexpr std::uint32_t kAxisFeedbackSafetySlave = 1U << 19U;
inline constexpr std::uint32_t kAxisFeedbackSafetyModeMismatch = 1U << 20U;
inline constexpr std::uint32_t kAxisFeedbackSafetyFollowingError = 1U << 21U;
inline constexpr std::uint32_t kAxisFeedbackSafetyInvalidCommand = 1U << 22U;
inline constexpr std::uint32_t kAxisFeedbackSafetyFaultResetLimit = 1U << 23U;
inline constexpr std::uint32_t kAxisFeedbackSafetyGroup = 1U << 24U;
inline constexpr std::uint32_t kAxisFeedbackSafetyShutdown = 1U << 25U;
inline constexpr std::uint32_t kAxisFeedbackSafetyProcessData = 1U << 26U;
inline constexpr std::uint32_t kAxisFeedbackSafetyDriveFault = 1U << 27U;
inline constexpr std::uint32_t kAxisFeedbackSafetySerial = 1U << 28U;

struct SafetyOptions {
  std::uint32_t consecutive_wkc_limit{3U};
  std::uint32_t maximum_fault_resets{3U};
  std::chrono::nanoseconds safe_stop_timeout{250'000'000};
  std::int32_t zero_velocity_threshold_counts{1};
  std::uint32_t zero_velocity_confirmation_cycles{1U};
};

enum class CommandAcceptance : std::uint8_t {
  accepted,
  duplicate,
  rejected,
};

struct SafetyBusState {
  std::uint32_t working_counter{};
  std::uint32_t expected_working_counter{};
  bool expected_working_counter_known{};
  bool working_counter_complete{};
  bool link_up{};
  bool all_slaves_operational{};
  bool process_data_valid{};
  std::uint16_t operational_axes_mask{};
  std::uint16_t external_stop_axes_mask{};
  std::int64_t dc_deviation_ns{};
};

struct SafetyEvaluation {
  std::array<AxisCommand, kRobotIoMaximumAxes> commands{};
  std::array<AxisRequest, kRobotIoMaximumAxes> requests{};
  std::array<std::uint32_t, kRobotIoMaximumAxes> feedback_flags{};
  std::uint32_t axis_count{};
  bool shutdown_complete{};
};

class SafetySupervisor {
 public:
  explicit SafetySupervisor(SafetyOptions options = {}) noexcept;

  Result<void> configure(std::span<const profiles::AxisConfig> axes);
  CommandAcceptance accept_commands(const Snapshot<AxisCommand>& snapshot,
                                    std::int64_t now_ns) noexcept;
  void request_shutdown(std::int64_t now_ns) noexcept;
  SafetyEvaluation evaluate(const SafetyBusState& bus,
                            std::span<const Cia402PdoView> pdos,
                            std::int64_t now_ns) noexcept;

  AxisRequest axis_request(std::size_t axis_index) const noexcept;
  std::uint64_t last_command_sequence() const noexcept;
  std::int64_t last_command_timestamp_ns() const noexcept;
  std::uint32_t consecutive_wkc_failures() const noexcept;
  bool shutdown_complete() const noexcept;

 private:
  struct AxisSafetyConfig {
    std::int64_t command_timeout_ns{};
    profiles::Cia402Mode mode{};
    double scale{};
    double minimum{};
    double maximum{};
    double slew_limit{};
    double following_error_limit{};
    std::uint8_t group{};
  };

  enum class StopPhase : std::uint8_t { running, quick_stop, disabled };

  struct AxisSafetyState {
    StopPhase phase{StopPhase::running};
    std::int64_t stop_started_ns{};
    std::uint32_t zero_velocity_cycles{};
    std::uint32_t fault_reset_edges{};
    std::uint64_t stopped_command_sequence{};
    double last_target{};
    bool has_last_target{};
    bool fault_episode_active{};
    bool fault_reset_output_active{};
  };

  struct TargetEvaluation {
    double actual{};
    double staged{};
    bool following_error{};
  };

  static bool command_equals(const AxisCommand& lhs,
                             const AxisCommand& rhs) noexcept;
  static AxisRequest request_from_flags(std::uint32_t flags) noexcept;
  static std::uint32_t flags_for_request(AxisRequest request) noexcept;
  bool valid_snapshot(const Snapshot<AxisCommand>& snapshot,
                      std::int64_t now_ns) const noexcept;
  bool axis_operational(const SafetyBusState& bus,
                        std::size_t axis_index) const noexcept;
  static TargetEvaluation evaluate_target(const AxisSafetyConfig& config,
                                          AxisSafetyState& state,
                                          const AxisCommand& command,
                                          const Cia402PdoView& pdo) noexcept;

  SafetyOptions options_{};
  std::array<AxisSafetyConfig, kRobotIoMaximumAxes> configs_{};
  std::array<AxisSafetyState, kRobotIoMaximumAxes> states_{};
  std::array<AxisCommand, kRobotIoMaximumAxes> commands_{};
  std::array<AxisRequest, kRobotIoMaximumAxes> last_requests_{};
  std::uint32_t axis_count_{};
  std::uint32_t consecutive_wkc_failures_{};
  std::uint64_t last_command_sequence_{};
  std::int64_t last_command_timestamp_ns_{};
  bool configured_{};
  bool has_command_{};
  bool commands_valid_{};
  bool shutdown_requested_{};
  bool shutdown_complete_{};
};

}  // namespace policy_runtime
