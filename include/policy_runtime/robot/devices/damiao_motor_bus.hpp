#pragma once

#include <variant>

#include "policy_runtime/robot/devices/damiao_motor_device.hpp"
#include "policy_runtime/transport/serial/virtual_can_transport.hpp"
#include "policy_runtime/transport/socketcan/socketcan_transport.hpp"
#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

// One realtime bus entry per Damiao motor. Owns the device and its CAN
// transport; cycle() performs exactly one nonblocking send and at most one
// bounded receive. Faults latch and are cleared only by process restart.
class DamiaoMotorBus final : public Transport {
 public:
  using CanTransport =
      std::variant<SocketCanTransport, VirtualCanTransport>;

  DamiaoMotorBus(DamiaoMotorConfig config, CanTransport transport)
      : device_(config), transport_(std::move(transport)) {}

  DamiaoMotorDevice& device() noexcept { return device_; }
  const DamiaoMotorDevice& device() const noexcept { return device_; }

  // Stages the next command. Realtime-safe: no allocation, no I/O.
  void stage_command(const DamiaoMitCommand& command) noexcept {
    staged_ = command;
  }
  void set_enabled(bool enabled) noexcept { requested_enabled_ = enabled; }
  void request_zero_position() noexcept { zero_requested_ = true; }

  const DamiaoMitCommand& staged_command() const noexcept { return staged_; }

  Result<void> last_cycle_result() const noexcept { return last_cycle_; }

  // Transport interface. Configuration is static; open only validates the
  // pre-opened descriptor and never reopens.
  Result<void> open() override {
    const bool valid = std::visit(
        [](const auto& transport) { return transport.valid(); }, transport_);
    if (!valid) {
      return Result<void>::failure(
          {ErrorCode::unavailable, "damiao transport descriptor is not open"});
    }
    return Result<void>::success();
  }

  void close() noexcept override {}

  TransportHealth health() const noexcept override {
    if (device_.fault_latched() || !last_cycle_.has_value()) {
      return TransportHealth::failed;
    }
    return TransportHealth::healthy;
  }

  SchedulingClass scheduling_class() const noexcept override {
    return SchedulingClass::hard_realtime_periodic;
  }

  void cycle(const CycleContext& context) noexcept override {
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         context.scheduled_start.time_since_epoch())
                         .count();
    if (zero_requested_ || requested_enabled_ != enabled_) {
      const auto control = zero_requested_
                               ? DamiaoControl::zero_position
                               : (requested_enabled_ ? DamiaoControl::enable
                                                     : DamiaoControl::disable);
      CanFrame frame;
      frame.id = device_.command_can_id();
      frame.data = DamiaoProtocol::encode_control(control);
      frame.dlc = 8U;
      last_cycle_ = std::visit(
          [&](auto& transport) { return transport.send_frame(frame); },
          transport_);
      if (last_cycle_.has_value()) {
        enabled_ = requested_enabled_;
        zero_requested_ = false;
      } else {
        device_.latch_fault();
      }
      return;
    }
    if (!enabled_) {
      return;
    }
    last_cycle_ = std::visit(
        [&](auto& transport) {
          return device_.cycle(staged_, transport, device_.motor_id(), now);
        }, transport_);
  }

 private:
  DamiaoMotorDevice device_;
  CanTransport transport_;
  DamiaoMitCommand staged_{};
  bool requested_enabled_{};
  bool enabled_{};
  bool zero_requested_{};
  Result<void> last_cycle_{Result<void>::success()};
};

}  // namespace policy_runtime
