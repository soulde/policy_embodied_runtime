#pragma once

#include <algorithm>

#include "policy_runtime/protocol/damiao/protocol.hpp"
#include "policy_runtime/robot/devices/device.hpp"

namespace policy_runtime {

class DamiaoActuator final : public ActuatorDevice {
 public:
  DamiaoActuator(std::uint8_t motor_id, DamiaoLimits limits) noexcept
      : motor_id_(motor_id), limits_(limits) {}

  void set_enabled(bool enabled) noexcept { enabled_ = enabled; }
  void request_zero_position() noexcept { zero_requested_ = true; }
  void clear_zero_position() noexcept { zero_requested_ = false; }
  void set_command(const DamiaoMitCommand& command) noexcept { command_ = command; }

  Result<DeviceFrame> encode_frame() const noexcept override {
    DeviceFrame frame;
    frame.address = motor_id_;
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
    auto encoded = DamiaoProtocol::encode_mit(command_, limits_);
    if (!encoded.has_value()) return Result<DeviceFrame>::failure(encoded.error());
    std::copy(encoded.value().begin(), encoded.value().end(), frame.bytes.begin());
    return Result<DeviceFrame>::success(frame);
  }

  Result<DeviceFrame> encode_enable() const noexcept {
    DeviceFrame frame;
    frame.address = motor_id_;
    frame.size = 8U;
    const auto payload = DamiaoProtocol::encode_control(DamiaoControl::enable);
    std::copy(payload.begin(), payload.end(), frame.bytes.begin());
    return Result<DeviceFrame>::success(frame);
  }

 private:
  std::uint8_t motor_id_{};
  DamiaoLimits limits_{};
  DamiaoMitCommand command_{};
  bool enabled_{};
  bool zero_requested_{};
};

}  // namespace policy_runtime
