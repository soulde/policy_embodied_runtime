#include <algorithm>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/robot_io/dds/topic_registry.hpp"

namespace {

using policy_runtime::ErrorCode;
using policy_runtime::profiles::DeviceConfig;
using policy_runtime::profiles::DeviceLink;
using policy_runtime::profiles::RobotProfile;
using policy_runtime::robot_io::dds::DdsTopicDescriptor;
using policy_runtime::robot_io::dds::DdsDeviceKind;
using policy_runtime::robot_io::dds::DdsTopicKind;
using policy_runtime::robot_io::dds::build_dds_topic_registry;

DeviceConfig damiao_device(std::string name) {
  return DeviceConfig{std::move(name), DeviceLink{"damiao_motor", "can0"}, {}};
}

RobotProfile profile_with_one_motor() {
  RobotProfile profile;
  profile.sensors.push_back(damiao_device("joint_position"));
  profile.actuators.push_back(damiao_device("joint_torque"));
  return profile;
}

RobotProfile profile_with_names(std::string first, std::string second) {
  RobotProfile profile;
  profile.sensors.push_back(damiao_device(std::move(first)));
  profile.sensors.push_back(damiao_device(std::move(second)));
  return profile;
}

std::size_t count_kind(const std::vector<DdsTopicDescriptor>& descriptors,
                       DdsTopicKind kind) {
  return static_cast<std::size_t>(std::count_if(
      descriptors.begin(), descriptors.end(),
      [kind](const DdsTopicDescriptor& descriptor) {
        return descriptor.kind == kind;
      }));
}

const DdsTopicDescriptor* find_descriptor(
    const std::vector<DdsTopicDescriptor>& descriptors, DdsTopicKind kind,
    std::string_view device_id) {
  const auto found = std::find_if(
      descriptors.begin(), descriptors.end(), [kind, device_id](
                                                const DdsTopicDescriptor& descriptor) {
        return descriptor.kind == kind && descriptor.device_id == device_id;
      });
  return found == descriptors.end() ? nullptr : &*found;
}

}  // namespace

TEST(DdsTopicRegistryTest, GivesEachSensorAndActuatorExactlyOneTopic) {
  auto registry = build_dds_topic_registry(profile_with_one_motor(), "arm");
  ASSERT_TRUE(registry.has_value()) << registry.error().message;
  EXPECT_EQ(count_kind(registry.value(), DdsTopicKind::command), 1U);
  EXPECT_EQ(count_kind(registry.value(), DdsTopicKind::state), 1U);
}

TEST(DdsTopicRegistryTest, RejectsNormalizedNameCollisions) {
  auto profile = profile_with_names("joint-1", "joint_1");
  const auto registry = build_dds_topic_registry(profile, "arm");
  ASSERT_FALSE(registry.has_value());
  EXPECT_EQ(registry.error().code, ErrorCode::invalid_argument);
}

TEST(DdsTopicRegistryTest,
     UsesExplicitRobotNamespaceAndNormalizedTopicComponents) {
  RobotProfile profile;
  profile.sensors.push_back(damiao_device("Joint--1"));

  auto registry = build_dds_topic_registry(profile, "Arm A");
  ASSERT_TRUE(registry.has_value()) << registry.error().message;

  const auto* state = find_descriptor(registry.value(), DdsTopicKind::state,
                                      "Joint--1");
  ASSERT_NE(state, nullptr);
  EXPECT_EQ(state->robot_id, "Arm A");
  EXPECT_EQ(state->topic_name, "robot_io_arm_a_sensor_joint_1_state");

  const auto* health =
      find_descriptor(registry.value(), DdsTopicKind::health, "health");
  ASSERT_NE(health, nullptr);
  EXPECT_EQ(health->topic_name, "robot_io_arm_a_health");
}

TEST(DdsTopicRegistryTest, RejectsRobotIdentifierWithoutAsciiLetterOrDigit) {
  const auto registry = build_dds_topic_registry(profile_with_one_motor(), "---");
  ASSERT_FALSE(registry.has_value());
  EXPECT_EQ(registry.error().code, ErrorCode::invalid_argument);
}

TEST(DdsTopicRegistryTest, AddsHealthAndOneCommitPerEthercatGroup) {
  RobotProfile profile;
  profile.sensors.push_back(DeviceConfig{
      "shoulder_position", DeviceLink{"cia402", "/dev/EtherCAT0"},
      {{"safety_group", "arm-a"}}});
  profile.actuators.push_back(DeviceConfig{
      "shoulder_target", DeviceLink{"cia402", "/dev/EtherCAT0"},
      {{"safety_group", "arm-a"}}});
  profile.actuators.push_back(damiao_device("wrist_torque"));

  auto registry = build_dds_topic_registry(profile, "Arm A");
  ASSERT_TRUE(registry.has_value()) << registry.error().message;

  const auto commit_count = count_kind(registry.value(), DdsTopicKind::commit);
  const auto health_count = count_kind(registry.value(), DdsTopicKind::health);
  EXPECT_EQ(commit_count, 1U);
  EXPECT_EQ(health_count, 1U);

  const auto commit = std::find_if(
      registry.value().begin(), registry.value().end(),
      [](const DdsTopicDescriptor& descriptor) {
        return descriptor.kind == DdsTopicKind::commit;
      });
  ASSERT_NE(commit, registry.value().end());
  EXPECT_EQ(commit->device_kind, DdsDeviceKind::ethercat);
  EXPECT_EQ(commit->execution_group, "arm-a");
  EXPECT_EQ(commit->topic_name,
            "robot_io_arm_a_ethercat_arm_a_command_commit");
}

TEST(DdsTopicRegistryTest, AddsOneCommitForEachDistinctEthercatGroup) {
  RobotProfile profile;
  profile.actuators.push_back(DeviceConfig{
      "shoulder_target", DeviceLink{"cia402", "/dev/EtherCAT0"},
      {{"safety_group", "shoulder"}}});
  profile.actuators.push_back(DeviceConfig{
      "wrist_target", DeviceLink{"cia402", "/dev/EtherCAT0"},
      {{"safety_group", "wrist"}}});

  auto registry = build_dds_topic_registry(profile, "arm");
  ASSERT_TRUE(registry.has_value()) << registry.error().message;
  EXPECT_EQ(count_kind(registry.value(), DdsTopicKind::commit), 2U);

  const auto* shoulder = find_descriptor(
      registry.value(), DdsTopicKind::commit, "shoulder");
  const auto* wrist = find_descriptor(registry.value(), DdsTopicKind::commit,
                                      "wrist");
  ASSERT_NE(shoulder, nullptr);
  ASSERT_NE(wrist, nullptr);
  EXPECT_EQ(shoulder->topic_name,
            "robot_io_arm_ethercat_shoulder_command_commit");
  EXPECT_EQ(wrist->topic_name,
            "robot_io_arm_ethercat_wrist_command_commit");
}

TEST(DdsTopicRegistryTest, RejectsNormalizedEthercatGroupCollisions) {
  RobotProfile profile;
  profile.actuators.push_back(DeviceConfig{
      "shoulder_target", DeviceLink{"cia402", "/dev/EtherCAT0"},
      {{"safety_group", "arm-a"}}});
  profile.actuators.push_back(DeviceConfig{
      "wrist_target", DeviceLink{"cia402", "/dev/EtherCAT0"},
      {{"safety_group", "arm_a"}}});

  const auto registry = build_dds_topic_registry(profile, "arm");
  ASSERT_FALSE(registry.has_value());
  EXPECT_EQ(registry.error().code, ErrorCode::invalid_argument);
}
