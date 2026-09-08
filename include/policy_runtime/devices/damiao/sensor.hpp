#pragma once

#include "policy_runtime/protocol/damiao/protocol.hpp"
#include "policy_runtime/devices/device.hpp"

namespace policy_runtime {

class DamiaoSensor final : public SensorDevice {
 public:
  DamiaoSensor(std::uint8_t motor_id, DamiaoLimits limits) noexcept
      : motor_id_(motor_id), limits_(limits) {}

  bool accepts(const DeviceFrame& frame) const noexcept override {
    return frame.address == motor_id_ && frame.size == 8U;
  }

  Result<DamiaoFeedback> decode(const DeviceFrame& frame) const noexcept {
    if (!accepts(frame)) {
      return Result<DamiaoFeedback>::failure(
          {ErrorCode::protocol, "Damiao sensor frame does not match motor"});
    }
    return DamiaoProtocol::decode_feedback(frame.address, frame.bytes.data(),
                                           frame.size, limits_);
  }

 private:
  std::uint8_t motor_id_{};
  DamiaoLimits limits_{};
};

}  // namespace policy_runtime
