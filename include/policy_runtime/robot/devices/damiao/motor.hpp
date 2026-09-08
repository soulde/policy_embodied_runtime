#pragma once

#include "policy_runtime/robot/devices/damiao/actuator.hpp"
#include "policy_runtime/robot/devices/damiao/sensor.hpp"

namespace policy_runtime {

// A physical Damiao motor is represented by two directional devices.
class DamiaoMotor final {
 public:
  DamiaoMotor(std::uint8_t motor_id, DamiaoLimits limits) noexcept
      : sensor_(motor_id, limits), actuator_(motor_id, limits) {}

  DamiaoSensor& sensor() noexcept { return sensor_; }
  const DamiaoSensor& sensor() const noexcept { return sensor_; }
  DamiaoActuator& actuator() noexcept { return actuator_; }
  const DamiaoActuator& actuator() const noexcept { return actuator_; }

 private:
  DamiaoSensor sensor_;
  DamiaoActuator actuator_;
};

}  // namespace policy_runtime
