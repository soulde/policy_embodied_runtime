#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace policy_runtime {

inline constexpr std::uint32_t kRobotIoMaximumAxes = 12;
inline constexpr std::uint32_t kRobotIoMaximumServos = 32;

struct AxisCommand {
  std::uint64_t sequence{};
  std::int64_t timestamp_ns{};
  double target{};
  std::uint32_t flags{};
  std::uint32_t reserved{};
};

struct AxisFeedback {
  std::uint64_t sequence{};
  std::int64_t timestamp_ns{};
  double position{};
  double velocity{};
  double effort{};
  std::uint32_t status_word{};
  std::uint32_t flags{};
  std::uint32_t mode_display{};
  std::uint32_t reserved0{};
  std::uint64_t reserved1{};
};

struct St3215ServoCommand {
  std::uint64_t sequence{};
  std::int64_t timestamp_ns{};
  double target_position_rad{};
  bool enabled{};
  bool emergency_stop{};
  std::array<std::byte, 6> reserved{};
};

struct St3215ServoFeedback {
  std::uint64_t feedback_sequence{};
  std::uint64_t command_sequence{};
  std::int64_t timestamp_ns{};
  double position_rad{};
  std::uint32_t raw_position{};
  std::uint32_t flags{1U << 1U};
  std::uint32_t transport_health{};
  std::uint32_t status_error{};
  std::uint64_t timeout_count{};
  std::uint64_t io_error_count{};
};

struct BusHealth {
  std::uint64_t sequence{};
  std::int64_t timestamp_ns{};
  std::uint32_t state{};
  std::uint32_t working_counter{};
  std::uint32_t expected_working_counter{};
  std::uint32_t flags{};
};

static_assert(std::is_trivially_copyable_v<AxisCommand>);
static_assert(std::is_trivially_copyable_v<AxisFeedback>);
static_assert(std::is_trivially_copyable_v<St3215ServoCommand>);
static_assert(std::is_trivially_copyable_v<St3215ServoFeedback>);

}  // namespace policy_runtime
