#pragma once

#include <algorithm>

#include "policy_runtime/devices/damiao/damiao.hpp"
#include "policy_runtime/protocol/damiao/protocol.hpp"
#include "policy_runtime/devices/device.hpp"

namespace policy_runtime {

class DamiaoActuator final : public ActuatorDevice {
 public:
  explicit DamiaoActuator(DamiaoConfig config) noexcept
      : config_(config) {}

  DamiaoActuator(std::uint8_t motor_id, DamiaoLimits limits) noexcept
      : DamiaoActuator(DamiaoConfig{motor_id, limits}) {}

  void set_enable() noexcept { enabled_ = true; }
  void set_disable() noexcept { enabled_ = false; }
  void request_zero_position() noexcept { zero_requested_ = true; }
  void clear_zero_position() noexcept { zero_requested_ = false; }
  void set_command(const DamiaoMitCommand& command) noexcept { command_ = command; }

  Result<DeviceFrame> encode_frame() const noexcept override {
    DeviceFrame frame;
    frame.address = config_.motor_id;
    frame.size = 8U;
    if (zero_requested_) {
      const auto payload = DamiaoProtocol::encode_control(DamiaoControl::zero_position);
      std::copy(payload.begin(), payload.end(), frame.bytes.begin());
      return Result<DeviceFrame>::success(frame);
    }
    if (!enabled_) {
      const auto payload = DamiaoProtocol::encode_control(DamiaoControl::disable);
      std::copy(payload.begin(), payload.end(), frame.bytes.begin());
      return Result<DeviceFrame>::success(frame);
    }
    auto encoded = DamiaoProtocol::encode_mit(command_, config_.limits);
    if (!encoded.has_value()) return Result<DeviceFrame>::failure(encoded.error());
    std::copy(encoded.value().begin(), encoded.value().end(), frame.bytes.begin());
    return Result<DeviceFrame>::success(frame);
  }

  Result<DeviceFrame> encode_enable() const noexcept {
    DeviceFrame frame;
    frame.address = config_.motor_id;
    frame.size = 8U;
    const auto payload = DamiaoProtocol::encode_control(DamiaoControl::enable);
    std::copy(payload.begin(), payload.end(), frame.bytes.begin());
    return Result<DeviceFrame>::success(frame);
  }

 private:
  DamiaoConfig config_{};
  DamiaoMitCommand command_{};
  bool enabled_{};
  bool zero_requested_{};
};

}  // namespace policy_runtime
