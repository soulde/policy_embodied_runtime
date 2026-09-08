#pragma once

#include <chrono>
#include <cstdint>
#include <cstring>

#include "policy_runtime/protocol/damiao/protocol.hpp"
#include "policy_runtime/robot/devices/mit_motor_device.hpp"
#include "policy_runtime/transport/socketcan/socketcan_transport.hpp"

namespace policy_runtime {

enum class DamiaoMode : std::uint8_t {
  mit,
};

struct DamiaoMotorConfig {
  std::uint8_t motor_id{};
  DamiaoMode mode{DamiaoMode::mit};
  DamiaoLimits limits;
  std::chrono::milliseconds feedback_timeout{100};
};

// Realtime Damiao motor device. One cycle() performs exactly one nonblocking
// send and at most one bounded receive against a CAN-capable transport; any
// error latches a fault with no recovery. Feedback that has not been refreshed
// within the configured timeout is reported stale, which also latches a fault.
class DamiaoMotorDevice final : public MITMotorDevice {
 public:
  explicit DamiaoMotorDevice(DamiaoMotorConfig config)
      : MITMotorDevice(config.limits), config_(config) {}

  const DamiaoMotorConfig& config() const noexcept { return config_; }
  DamiaoMode mode() const noexcept { return config_.mode; }
  std::uint8_t motor_id() const noexcept { return config_.motor_id; }
  std::uint64_t feedback_sequence() const noexcept { return feedback_sequence_; }

  bool feedback_stale(std::int64_t now_ns) const noexcept {
    return !has_feedback_ ||
           now_ns - last_feedback_ns_ >
               std::chrono::duration_cast<std::chrono::nanoseconds>(
                   config_.feedback_timeout)
                   .count();
  }

  Result<DamiaoProtocol::Frame> encode_command(
      const DamiaoMitCommand& command) const noexcept {
    return DamiaoProtocol::encode_mit(command, limits());
  }

  Result<void> consume_feedback(std::uint32_t can_id,
                                const DamiaoProtocol::Frame& frame,
                                std::int64_t now_ns) noexcept {
    auto decoded = DamiaoProtocol::decode_feedback(can_id, frame.data(),
                                                   frame.size(), limits());
    if (!decoded.has_value()) {
      latch_fault();
      return Result<void>::failure(decoded.error());
    }
    if (decoded.value().motor_id != motor_id()) {
      latch_fault();
      return Result<void>::failure(
          {ErrorCode::protocol, "feedback motor ID does not match device"});
    }
    update_feedback(decoded.value());
    ++feedback_sequence_;
    has_feedback_ = true;
    last_feedback_ns_ = now_ns;
    return Result<void>::success();
  }

  // One-shot realtime cycle. `transport` must be a CAN-capable transport with
  // send_frame(const CanFrame&) and receive_frame() members.
  template <typename CanTransport>
  Result<void> cycle(const DamiaoMitCommand& command, CanTransport& transport,
                     std::uint32_t feedback_can_id, std::int64_t now_ns) {
    auto encoded = encode_command(command);
    if (!encoded.has_value()) {
      latch_fault();
      return Result<void>::failure(encoded.error());
    }
    CanFrame frame{};
    frame.id = command_can_id();
    frame.dlc = encoded.value().size();
    std::memcpy(frame.data.data(), encoded.value().data(),
                encoded.value().size());
    auto sent = transport.send_frame(frame);
    if (!sent.has_value()) {
      latch_fault();
      return sent;
    }
    auto received = transport.receive_frame();
    if (!received.has_value()) {
      latch_fault();
      return Result<void>::failure(received.error());
    }
    if (received.value().has_value() &&
        received.value().value().id == feedback_can_id) {
      DamiaoProtocol::Frame payload{};
      std::memcpy(payload.data(), received.value().value().data.data(),
                  received.value().value().dlc);
      return consume_feedback(feedback_can_id, payload, now_ns);
    }
    if (feedback_stale(now_ns)) {
      latch_fault();
      return Result<void>::failure(
          {ErrorCode::timeout, "Damiao feedback is stale"});
    }
    return Result<void>::success();
  }

  // CAN identifier the motor listens on for MIT commands.
  std::uint32_t command_can_id() const noexcept { return motor_id(); }

 private:
  DamiaoMotorConfig config_{};
  std::int64_t last_feedback_ns_{};
  bool has_feedback_{false};
  std::uint64_t feedback_sequence_{};
};

}  // namespace policy_runtime
