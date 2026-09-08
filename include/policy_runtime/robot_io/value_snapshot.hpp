#pragma once

#include <array>
#include <cstdint>

#include "policy_runtime/robot_io/messages.hpp"
#include "policy_runtime/protocol/damiao/protocol.hpp"

namespace policy_runtime {

template <class T>
inline constexpr std::uint32_t kValueSnapshotMaximumItems =
    kRobotIoMaximumAxes;

template <>
inline constexpr std::uint32_t
    kValueSnapshotMaximumItems<St3215ServoCommand> = kRobotIoMaximumServos;

template <>
inline constexpr std::uint32_t
    kValueSnapshotMaximumItems<St3215ServoFeedback> = kRobotIoMaximumServos;

template <>
inline constexpr std::uint32_t
    kValueSnapshotMaximumItems<DamiaoMitCommand> = kRobotIoMaximumServos;

template <>
inline constexpr std::uint32_t
    kValueSnapshotMaximumItems<DamiaoFeedback> = kRobotIoMaximumServos;

template <class T>
struct Snapshot {
  std::uint64_t sequence{};
  std::int64_t timestamp_ns{};
  std::uint32_t axis_count{};
  std::array<T, kValueSnapshotMaximumItems<T>> axes{};
};

}  // namespace policy_runtime
