#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <functional>
#include <string_view>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/transport/can_frame.hpp"
#include "policy_runtime/transport/asynchronous_transport.hpp"

namespace policy_runtime {

// Transport over a pre-opened, nonblocking SocketCAN descriptor. One
// send_frame() performs exactly one write(2); one receive_frame() performs at
// most one read(2). Errors, including EAGAIN, are surfaced as faults; there is
// no retry or recovery.
class SocketCanTransport final : public AsynchronousTransport {
 public:
  using ReceiveHandler = std::function<void(const CanFrame&)>;
  explicit SocketCanTransport(int fd) noexcept : fd_(fd) {}
  ~SocketCanTransport();

  SocketCanTransport(const SocketCanTransport&) = delete;
  SocketCanTransport& operator=(const SocketCanTransport&) = delete;
  SocketCanTransport(SocketCanTransport&& other) noexcept;
  SocketCanTransport& operator=(SocketCanTransport&& other) noexcept;

  static Result<SocketCanTransport> open(std::string_view interface_name);

  Result<void> open() override;
  void close() noexcept override;
  TransportHealth health() const noexcept override;
  SchedulingClass scheduling_class() const noexcept override;
  void cycle(const CycleContext&) noexcept override;
  void receive_once() noexcept override;
  void set_receive_handler(ReceiveHandler handler);

  bool valid() const noexcept { return fd_ >= 0; }

  Result<void> send_frame(const CanFrame& frame) noexcept;
  Result<std::optional<CanFrame>> receive_frame() noexcept;

 private:
  int fd_;
  bool owns_fd_{};
  ReceiveHandler receive_handler_;
};

}  // namespace policy_runtime
