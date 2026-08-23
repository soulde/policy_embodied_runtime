#pragma once

#if !defined(__linux__)
#error "Robot I/O shared-memory IPC requires Linux"
#endif

#if !defined(__x86_64__) && !defined(__aarch64__)
#error "Robot I/O shared-memory IPC supports only x86_64 and ARM64"
#endif

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime {

inline constexpr std::uint32_t kRobotIoIpcAbiVersion = 1;
inline constexpr std::uint32_t kRobotIoMaximumAxes = 12;
inline constexpr std::array<char, 8> kRobotIoIpcMagic{'R', 'O', 'B', 'O', 'T', 'I', 'O', '1'};
inline constexpr std::array<char, 8> kRobotIoSetupMagic{'R', 'I', 'O', 'F', 'D', '0', '0', '1'};

enum class IpcRegionKind : std::uint32_t {
  command = 1,
  feedback = 2,
};

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

struct BusHealth {
  std::uint64_t sequence{};
  std::int64_t timestamp_ns{};
  std::uint32_t state{};
  std::uint32_t working_counter{};
  std::uint32_t expected_working_counter{};
  std::uint32_t flags{};
};

struct alignas(64) IpcHeader {
  std::array<char, 8> magic{'R', 'O', 'B', 'O', 'T', 'I', 'O', '1'};
  std::uint32_t abi_version{kRobotIoIpcAbiVersion};
  std::uint32_t generation{};
  std::uint32_t axis_count{};
  std::uint32_t axis_stride{};
  std::uint32_t region_kind{};
  std::uint32_t header_size{64};
  std::uint64_t slot_stride{};
  std::uint64_t mapping_size{};
  std::uint64_t reserved0{};
  std::uint64_t reserved1{};
};

struct IpcSetupMessage {
  std::array<char, 8> magic{'R', 'I', 'O', 'F', 'D', '0', '0', '1'};
  std::uint32_t abi_version{kRobotIoIpcAbiVersion};
  std::uint32_t generation{};
  std::uint32_t axis_count{};
  std::uint32_t descriptor_count{2};
  std::uint64_t command_mapping_size{};
  std::uint64_t feedback_mapping_size{};
  std::uint32_t command_axis_stride{sizeof(AxisCommand)};
  std::uint32_t feedback_axis_stride{sizeof(AxisFeedback)};
  std::array<std::uint64_t, 2> reserved{};
};

static_assert(std::endian::native == std::endian::little,
              "Robot I/O IPC ABI requires little-endian Linux");
static_assert(sizeof(void*) == 8, "Robot I/O IPC ABI requires a 64-bit process");
static_assert(sizeof(AxisCommand) == 32);
static_assert(sizeof(AxisFeedback) == 64);
static_assert(sizeof(BusHealth) == 32);
static_assert(sizeof(IpcHeader) == 64);
static_assert(alignof(IpcHeader) == 64);
static_assert(sizeof(IpcSetupMessage) == 64);
static_assert(offsetof(IpcHeader, magic) == 0);
static_assert(offsetof(IpcHeader, abi_version) == 8);
static_assert(offsetof(IpcHeader, generation) == 12);
static_assert(offsetof(IpcHeader, axis_count) == 16);
static_assert(offsetof(IpcHeader, axis_stride) == 20);
static_assert(offsetof(IpcHeader, region_kind) == 24);
static_assert(offsetof(IpcHeader, header_size) == 28);
static_assert(offsetof(IpcHeader, slot_stride) == 32);
static_assert(offsetof(IpcHeader, mapping_size) == 40);
static_assert(offsetof(AxisCommand, sequence) == 0);
static_assert(offsetof(AxisCommand, timestamp_ns) == 8);
static_assert(offsetof(AxisCommand, target) == 16);
static_assert(offsetof(AxisCommand, flags) == 24);
static_assert(offsetof(AxisFeedback, sequence) == 0);
static_assert(offsetof(AxisFeedback, position) == 16);
static_assert(offsetof(AxisFeedback, status_word) == 40);
static_assert(std::is_trivially_copyable_v<AxisCommand>);
static_assert(std::is_trivially_copyable_v<AxisFeedback>);
static_assert(std::is_trivially_copyable_v<BusHealth>);
static_assert(std::is_trivially_copyable_v<IpcHeader>);
static_assert(std::is_trivially_copyable_v<IpcSetupMessage>);
static_assert(std::is_standard_layout_v<AxisCommand>);
static_assert(std::is_standard_layout_v<AxisFeedback>);
static_assert(std::is_standard_layout_v<BusHealth>);
static_assert(std::is_standard_layout_v<IpcHeader>);

inline Result<void> validate_header_fields(const IpcHeader& header,
                                           std::uint32_t expected_generation,
                                           std::uint32_t expected_axis_count,
                                           std::uint32_t expected_axis_stride,
                                           IpcRegionKind expected_kind,
                                           std::size_t expected_mapping_size,
                                           std::size_t expected_slot_stride) {
  if (header.magic != kRobotIoIpcMagic) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC mapping magic"});
  }
  if (header.abi_version != kRobotIoIpcAbiVersion) {
    return Result<void>::failure({ErrorCode::protocol, "unsupported IPC ABI version"});
  }
  if (header.generation != expected_generation) {
    return Result<void>::failure({ErrorCode::unavailable, "stale daemon generation"});
  }
  if (header.axis_count > kRobotIoMaximumAxes ||
      header.axis_count != expected_axis_count) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC axis count"});
  }
  if (header.axis_stride != expected_axis_stride) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC axis stride"});
  }
  if (header.region_kind != static_cast<std::uint32_t>(expected_kind)) {
    return Result<void>::failure({ErrorCode::protocol, "unexpected IPC region kind"});
  }
  if (header.header_size != sizeof(IpcHeader) ||
      header.mapping_size != expected_mapping_size ||
      header.slot_stride != expected_slot_stride) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC mapping layout"});
  }
  if (header.reserved0 != 0 || header.reserved1 != 0) {
    return Result<void>::failure({ErrorCode::protocol, "nonzero IPC header reserved field"});
  }
  return Result<void>::success();
}

}  // namespace policy_runtime
