#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime {

// Canonical CAN data frame shared by every CAN-capable transport.
struct CanFrame {
  std::uint32_t id{};
  std::array<std::byte, 8> data{};
  std::uint8_t dlc{8};
};

// Transport over a pre-opened, nonblocking SocketCAN descriptor. One
// send_frame() performs exactly one write(2); one receive_frame() performs at
// most one read(2). Errors, including EAGAIN, are surfaced as faults; there is
// no retry or recovery.
class SocketCanTransport {
 public:
  explicit SocketCanTransport(int fd) noexcept : fd_(fd) {}

  bool valid() const noexcept { return fd_ >= 0; }

  Result<void> send_frame(const CanFrame& frame) noexcept;
  Result<std::optional<CanFrame>> receive_frame() noexcept;

 private:
  int fd_;
};

}  // namespace policy_runtime
