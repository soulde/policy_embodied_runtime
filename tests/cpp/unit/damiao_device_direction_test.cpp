#include <vector>

#include <gtest/gtest.h>

#include "policy_runtime/robot/devices/damiao.hpp"
#include "policy_runtime/robot/devices/damiao/motor.hpp"

namespace {

TEST(DamiaoDeviceDirectionTest, SensorAcceptsOnlyItsMotorFeedbackAddress) {
  policy_runtime::DamiaoSensor sensor(3U, {12.5F, 30.0F, 10.0F});
  policy_runtime::DeviceFrame frame;
  frame.address = 3U;
  frame.size = 8U;
  EXPECT_TRUE(sensor.accepts(frame));
  frame.address = 4U;
  EXPECT_FALSE(sensor.accepts(frame));
}

TEST(DamiaoDeviceDirectionTest, ActuatorEmitsControlAndMitFrames) {
  policy_runtime::DamiaoActuator actuator(1U, {12.5F, 30.0F, 10.0F});
  actuator.set_enabled(false);
  auto disabled = actuator.encode_frame();
  ASSERT_TRUE(disabled.has_value());
  EXPECT_EQ(std::to_integer<unsigned>(disabled.value().bytes[7]), 0xFD);

  actuator.set_enabled(true);
  actuator.set_command({1.0F, 0.0F, 10.0F, 0.5F, 0.0F});
  auto mit = actuator.encode_frame();
  ASSERT_TRUE(mit.has_value());
  EXPECT_EQ(mit.value().size, 8U);
  EXPECT_NE(mit.value().bytes, disabled.value().bytes);
}

TEST(DamiaoDeviceDirectionTest, MotorExposesIndependentSensorAndActuatorDevices) {
  policy_runtime::DamiaoMotor motor(5U, {12.5F, 30.0F, 10.0F});
  policy_runtime::DeviceFrame feedback;
  feedback.address = 5U;
  feedback.size = 8U;
  EXPECT_TRUE(motor.sensor().accepts(feedback));
  motor.actuator().set_enabled(false);
  auto command = motor.actuator().encode_frame();
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command.value().address, 5U);
}

TEST(DamiaoDeviceDirectionTest, SensorAndActuatorCanBeManagedAsSeparateCollections) {
  std::vector<policy_runtime::DamiaoSensor> sensors;
  std::vector<policy_runtime::DamiaoActuator> actuators;
  sensors.emplace_back(7U, policy_runtime::DamiaoLimits{12.5F, 30.0F, 10.0F});
  actuators.emplace_back(7U, policy_runtime::DamiaoLimits{12.5F, 30.0F, 10.0F});
  policy_runtime::DeviceFrame feedback;
  feedback.address = 7U;
  feedback.size = 8U;
  EXPECT_TRUE(sensors.front().accepts(feedback));
  actuators.front().set_enabled(false);
  EXPECT_TRUE(actuators.front().encode_frame().has_value());
}

}  // namespace
