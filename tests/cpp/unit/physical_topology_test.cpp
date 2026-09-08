#include <gtest/gtest.h>

#include "policy_runtime/robot_io/topology.hpp"

namespace {

TEST(PhysicalTopologyTest, DeduplicatesPairedDevicesAndSharedCanBus) {
  policy_runtime::profiles::RobotProfile profile;
  profile.sensors.push_back(
      {"motor_1_state", {"damiao_motor", "vcan0"},
       {{"transport", "socketcan"}}});
  profile.actuators.push_back(
      {"motor_1_command", {"damiao_motor", "vcan0"},
       {{"transport", "socketcan"}}});
  profile.actuators.push_back(
      {"motor_2_command", {"damiao_motor", "vcan0"},
       {{"transport", "socketcan"}}});

  auto topology = policy_runtime::robot_io::compile_physical_topology(profile);
  ASSERT_TRUE(topology.has_value()) << topology.error().message;
  ASSERT_EQ(topology.value().buses.size(), 1U);
  EXPECT_EQ(topology.value().sensors.size(), 1U);
  EXPECT_EQ(topology.value().actuators.size(), 2U);
  EXPECT_EQ(topology.value().buses.front().path, "vcan0");
  EXPECT_EQ(topology.value().sensors[0].transport_slot, 0U);
  EXPECT_EQ(topology.value().actuators[0].transport_slot, 1U);
  EXPECT_EQ(topology.value().actuators[1].transport_slot, 2U);
}

TEST(PhysicalTopologyTest, KeepsDifferentPhysicalConnectionsSeparate) {
  policy_runtime::profiles::RobotProfile profile;
  profile.sensors.push_back(
      {"can", {"damiao_motor", "can0"}, {{"transport", "socketcan"}}});
  profile.sensors.push_back(
      {"usb", {"damiao_motor", "/dev/ttyUSB0"},
       {{"transport", "usb_can"}}});

  auto topology = policy_runtime::robot_io::compile_physical_topology(profile);
  ASSERT_TRUE(topology.has_value()) << topology.error().message;
  EXPECT_EQ(topology.value().buses.size(), 2U);
}

}  // namespace
