#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime {

using Bytes = std::vector<std::byte>;

inline constexpr std::uint8_t kSt3215BroadcastId = 0xfeU;
inline constexpr std::size_t kSt3215MaximumParameters = 253U;
inline constexpr std::size_t kSt3215MaximumFrameSize = 259U;

enum class St3215Instruction : std::uint8_t {
  ping = 0x01,
  read_data = 0x02,
  write_data = 0x03,
  reg_write = 0x04,
  action = 0x05,
  recovery = 0x06,
  reset = 0x0a,
  sync_read = 0x82,
  sync_write = 0x83,
};

enum class St3215Register : std::uint8_t {
  goal_position = 0x2a,
  present_position = 0x38,
};

struct St3215Packet {
  std::uint8_t device_id{};
  std::uint8_t instruction_or_status{};
  Bytes parameters;
};

struct St3215Status {
  std::uint8_t device_id{};
  std::uint8_t error{};
  Bytes parameters;
};

class St3215Protocol {
 public:
  static Bytes ping_command(std::uint8_t device_id);
  static Bytes read_data_command(std::uint8_t device_id,
                                 St3215Register address,
                                 std::uint8_t length);
  static Bytes write_data_command(std::uint8_t device_id,
                                  St3215Register address,
                                  std::span<const std::byte> data);
  static Bytes goal_position_command(std::uint8_t device_id,
                                     std::uint16_t position_units,
                                     std::uint16_t time_units,
                                     std::uint16_t speed_units);
  static Bytes read_present_position_command(std::uint8_t device_id);
  static Bytes status_packet(std::uint8_t device_id, std::uint8_t error,
                             std::span<const std::byte> parameters);

  static Result<Bytes> encode_packet(
      std::uint8_t device_id, std::uint8_t instruction_or_status,
      std::span<const std::byte> parameters = {});
  static Result<St3215Packet> decode_packet(
      std::span<const std::byte> packet);
  static Result<St3215Status> parse_status(
      std::span<const std::byte> packet);
  static std::uint8_t checksum(
      std::uint8_t device_id, std::uint8_t length,
      std::uint8_t instruction_or_status,
      std::span<const std::byte> parameters) noexcept;
};

// Bounded incremental parser for a serial byte stream. It discards bytes before
// the next 0xff 0xff header and consumes one complete malformed frame when
// reporting an error, allowing the following call to recover at the next frame.
class St3215StreamParser {
 public:
  Result<void> append(std::span<const std::byte> data);
  Result<std::optional<St3215Packet>> next();
  void reset() noexcept;
  std::size_t buffered_size() const noexcept;

 private:
  static constexpr std::size_t kCapacity = 2U * kSt3215MaximumFrameSize;

  void discard_prefix(std::size_t count) noexcept;

  std::array<std::byte, kCapacity> buffer_{};
  std::size_t size_{};
};

Result<std::uint16_t> read_st3215_u16_le(std::span<const std::byte> data);
Result<std::uint16_t> radians_to_position_units(
    double position_rad, std::uint16_t max_position_units);
Result<double> position_units_to_radians(
    std::uint16_t position_units, std::uint16_t max_position_units);

}  // namespace policy_runtime
