#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/transport/can_frame.hpp"

namespace policy_runtime {

// USB-CAN adapter exposed as a virtual serial port. Frames use a fixed
// little-endian wire format:
//   [0]      0xAA header
//   [1..4]   CAN id (32-bit little-endian)
//   [5]      DLC (<= 8)
//   [6..6+n] payload bytes
//   [6+n]    modulo-256 sum of bytes [0..6+n)
// The descriptor is pre-opened and nonblocking. Each call performs at most
// one write(2)/read(2) with no retry or recovery; partial frames from earlier
// reads are retained in a bounded buffer until complete.
class VirtualCanTransport {
 public:
  explicit VirtualCanTransport(int fd) noexcept : fd_(fd) {}

  bool valid() const noexcept { return fd_ >= 0; }

  Result<void> send_frame(const CanFrame& frame) noexcept;
  Result<std::optional<CanFrame>> receive_frame() noexcept;

  // Number of bytes of a partially received frame still pending.
  std::size_t pending_bytes() const noexcept { return buffer_length_; }

 private:
  static constexpr std::size_t kEncodedFrameSize = 7U + 8U;

  int fd_;
  std::byte buffer_[kEncodedFrameSize]{};
  std::size_t buffer_length_{0};
};

}  // namespace policy_runtime
