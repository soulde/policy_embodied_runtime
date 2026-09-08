#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/transport/can_frame.hpp"

namespace policy_runtime {

// Transport over a pre-opened, nonblocking SocketCAN descriptor. One
// send_frame() performs exactly one write(2); one receive_frame() performs at
// most one read(2). Errors, including EAGAIN, are surfaced as faults; there is
// no retry or recovery.
class SocketCanTransport {
 public:
  explicit SocketCanTransport(int fd) noexcept : fd_(fd) {}
  ~SocketCanTransport();

  SocketCanTransport(const SocketCanTransport&) = delete;
  SocketCanTransport& operator=(const SocketCanTransport&) = delete;
  SocketCanTransport(SocketCanTransport&& other) noexcept;
  SocketCanTransport& operator=(SocketCanTransport&& other) noexcept;

  static Result<SocketCanTransport> open(std::string_view interface_name);

  bool valid() const noexcept { return fd_ >= 0; }

  Result<void> send_frame(const CanFrame& frame) noexcept;
  Result<std::optional<CanFrame>> receive_frame() noexcept;

 private:
  int fd_;
  bool owns_fd_{};
};

}  // namespace policy_runtime
