#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <thread>
#include <vector>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/devices/device.hpp"
#include "policy_runtime/robot_io/dds/realtime_mailbox.hpp"
#include "policy_runtime/transport/socketcan/socketcan_transport.hpp"
#include "policy_runtime/transport/asynchronous_transport.hpp"
#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime::robot_io {

// Unified runtime. Cyclic transports use one cycle worker. Asynchronous
// transports use one receive worker plus the same cycle worker; cycle() sends
// directly and there is intentionally no separate transmit worker.
class TransportRuntime final {
 public:
  using ReceiveCallback =
      std::function<void(const DeviceFrame&, std::uint64_t sequence)>;

  static Result<TransportRuntime> create(
      Transport& transport,
      std::chrono::nanoseconds period = std::chrono::milliseconds{1});
  static Result<TransportRuntime> create(SocketCanTransport transport,
                                         ReceiveCallback callback,
                                         std::size_t actuator_slots = 32U);

  TransportRuntime(TransportRuntime&&) noexcept;
  TransportRuntime& operator=(TransportRuntime&&) noexcept;
  TransportRuntime(const TransportRuntime&) = delete;
  TransportRuntime& operator=(const TransportRuntime&) = delete;
  ~TransportRuntime();

  Result<void> start();
  void stop() noexcept;
  Result<void> stage(std::size_t slot, const DeviceFrame& frame) noexcept;
  bool running() const noexcept;

 private:
  struct Slot {
    dds::CommandMailbox<DeviceFrame> mailbox;
    std::uint64_t consumed_sequence{};
  };
  enum class Mode : std::uint8_t { generic, can };

  TransportRuntime(Transport& transport, std::chrono::nanoseconds period) noexcept;
  TransportRuntime(SocketCanTransport transport, ReceiveCallback callback,
                   std::vector<std::unique_ptr<Slot>> slots);
  void cycle_loop(std::stop_token stop);

  Mode mode_{Mode::generic};
  Transport* transport_{};
  std::unique_ptr<SocketCanTransport> can_transport_;
  AsynchronousTransport* asynchronous_transport_{};
  std::chrono::nanoseconds period_{};
  ReceiveCallback callback_;
  std::vector<std::unique_ptr<Slot>> slots_;
  std::atomic<bool> running_{};
  std::jthread worker_;
  std::jthread receive_worker_;
  std::chrono::microseconds poll_period_{100};
  std::uint64_t receive_sequence_{};
};

}  // namespace policy_runtime::robot_io
