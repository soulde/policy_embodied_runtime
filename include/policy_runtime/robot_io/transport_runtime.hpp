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

namespace policy_runtime::robot_io {

class TransportRuntime {
 public:
  using ReceiveCallback =
      std::function<void(const DeviceFrame&, std::uint64_t sequence)>;

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

  TransportRuntime(SocketCanTransport transport, ReceiveCallback callback,
                   std::vector<std::unique_ptr<Slot>> slots);
  void receive_loop(std::stop_token stop);
  void send_loop(std::stop_token stop);

  SocketCanTransport transport_;
  ReceiveCallback callback_;
  std::vector<std::unique_ptr<Slot>> slots_;
  std::atomic<bool> running_{};
  std::jthread receive_worker_;
  std::jthread send_worker_;
  std::chrono::microseconds poll_period_{100};
  std::uint64_t receive_sequence_{};
};

}  // namespace policy_runtime::robot_io
