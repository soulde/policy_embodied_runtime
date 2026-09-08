#pragma once

#include "policy_runtime/devices/damiao/damiao.hpp"
#include "policy_runtime/protocol/damiao/protocol.hpp"
#include "policy_runtime/devices/device.hpp"

namespace policy_runtime {

class DamiaoSensor final : public SensorDevice {
 public:
  explicit DamiaoSensor(DamiaoConfig config) noexcept : config_(config) {}

  DamiaoSensor(std::uint8_t motor_id, DamiaoLimits limits) noexcept
      : DamiaoSensor(DamiaoConfig{motor_id, limits}) {}

  bool accepts(const DeviceFrame& frame) const noexcept override {
    return frame.address == config_.motor_id && frame.size == 8U;
  }

  Result<DamiaoFeedback> decode(const DeviceFrame& frame) const noexcept {
    if (!accepts(frame)) {
      return Result<DamiaoFeedback>::failure(
          {ErrorCode::protocol, "Damiao sensor frame does not match motor"});
    }
    return DamiaoProtocol::decode_feedback(frame.address, frame.bytes.data(),
                                           frame.size, config_.limits);
  }

 private:
  DamiaoConfig config_{};
};

}  // namespace policy_runtime
