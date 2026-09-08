#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime::robot_io {

// Generic executor for any physical Transport. It knows only the Transport
// lifecycle and cycle contract; CAN frame queues remain in CanTransportRuntime.
class TransportRuntime final {
 public:
  static Result<TransportRuntime> create(
      Transport& transport,
      std::chrono::nanoseconds period = std::chrono::milliseconds{1});

  TransportRuntime(TransportRuntime&&) noexcept;
  TransportRuntime& operator=(TransportRuntime&&) noexcept;
  TransportRuntime(const TransportRuntime&) = delete;
  TransportRuntime& operator=(const TransportRuntime&) = delete;
  ~TransportRuntime();

  Result<void> start();
  void stop() noexcept;
  bool running() const noexcept;

 private:
  TransportRuntime(Transport& transport, std::chrono::nanoseconds period) noexcept;
  void cycle_loop(std::stop_token stop);

  Transport* transport_{};
  std::chrono::nanoseconds period_{};
  std::atomic<bool> running_{};
  std::jthread worker_;
};

}  // namespace policy_runtime::robot_io
