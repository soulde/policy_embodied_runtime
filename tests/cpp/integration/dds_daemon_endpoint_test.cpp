#include <chrono>
#include <array>
#include <atomic>
#include <thread>
#include <type_traits>

#include <gtest/gtest.h>
#include <dds/dds.hpp>

#include "policy_runtime/robot_io/dds/daemon_endpoint.hpp"

namespace {

using policy_runtime::robot_io::dds::DdsDaemonCallbacks;
using policy_runtime::robot_io::dds::DdsDaemonEndpoint;

static_assert(!std::is_copy_constructible_v<DdsDaemonEndpoint>);
static_assert(std::is_move_constructible_v<DdsDaemonEndpoint>);

TEST(DdsDaemonEndpointTest, RejectsProfileWithoutDdsBackend) {
  policy_runtime::profiles::RobotProfile profile;
  auto endpoint = DdsDaemonEndpoint::create(profile, DdsDaemonCallbacks{});
  ASSERT_FALSE(endpoint.has_value());
  EXPECT_EQ(static_cast<int>(endpoint.error().code),
            static_cast<int>(policy_runtime::ErrorCode::invalid_argument));
}

TEST(DdsDaemonEndpointTest, CreatesParticipantForConfiguredDomain) {
  policy_runtime::profiles::RobotProfile profile;
  profile.dds.backend = policy_runtime::profiles::RobotIoBackend::dds;
  profile.dds.robot_id = "test_robot";
  profile.dds.local_domain_id = 219U;

  auto endpoint = DdsDaemonEndpoint::create(profile, DdsDaemonCallbacks{});
  ASSERT_TRUE(endpoint.has_value()) << endpoint.error().message;
  EXPECT_TRUE(endpoint.value().start().has_value());
  EXPECT_TRUE(endpoint.value().poll_health().has_value());
  endpoint.value().stop();
}

TEST(DdsDaemonEndpointTest, PublishesOneStateForPhysicalReceiveEvent) {
  constexpr std::uint32_t domain_id = 220U;
  policy_runtime::profiles::RobotProfile profile;
  profile.dds.backend = policy_runtime::profiles::RobotIoBackend::dds;
  profile.dds.robot_id = "test_robot";
  profile.dds.local_domain_id = domain_id;
  profile.sensors.push_back({"joint_1_position",
                             {"cia402", "ethercat://joint_1"},
                             {{"safety_group", "arm"}}});

  ::dds::domain::DomainParticipant subscriber_participant(domain_id);
  ::dds::topic::Topic<policy_runtime::robot_io::Cia402SensorState> topic(
      subscriber_participant,
      "robot_io_test_robot_sensor_joint_1_position_state");
  ::dds::sub::Subscriber subscriber(subscriber_participant);
  ::dds::sub::DataReader<policy_runtime::robot_io::Cia402SensorState> reader(
      subscriber, topic);

  auto endpoint = DdsDaemonEndpoint::create(profile, DdsDaemonCallbacks{});
  ASSERT_TRUE(endpoint.has_value()) << endpoint.error().message;
  ASSERT_TRUE(endpoint.value().start().has_value());
  policy_runtime::robot_io::dds::Cia402SensorEvent event;
  event.position = 1.25;
  event.bus_cycle = 17U;
  ASSERT_TRUE(endpoint.value().enqueue_cia402_sensor_realtime(0U, event));

  for (unsigned attempt = 0; attempt < 50U; ++attempt) {
    auto samples = reader.take();
    for (const auto& sample : samples) {
      if (sample.info().valid()) {
        EXPECT_DOUBLE_EQ(sample.data().position(), 1.25);
        EXPECT_EQ(sample.data().header().bus_cycle(), 17U);
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  FAIL() << "timed out waiting for CiA402 sensor state";
}

TEST(DdsDaemonEndpointTest, CountsDroppedPhysicalReceiveEvents) {
  policy_runtime::profiles::RobotProfile profile;
  profile.dds.backend = policy_runtime::profiles::RobotIoBackend::dds;
  profile.dds.robot_id = "overflow_robot";
  profile.dds.local_domain_id = 221U;
  profile.dds.sensor_queue_capacity = 16U;
  profile.sensors.push_back({"joint_1_position",
                             {"cia402", "ethercat://joint_1"},
                             {{"safety_group", "arm"}}});
  auto endpoint = DdsDaemonEndpoint::create(profile, DdsDaemonCallbacks{});
  ASSERT_TRUE(endpoint.has_value()) << endpoint.error().message;
  for (std::uint64_t sequence = 1U; sequence <= 20U; ++sequence) {
    policy_runtime::robot_io::dds::Cia402SensorEvent event;
    event.sequence = sequence;
    ASSERT_TRUE(endpoint.value().enqueue_cia402_sensor_realtime(0U, event));
  }
  EXPECT_EQ(endpoint.value().dropped_cia402_sensor_samples(0U), 4U);
}

TEST(DdsDaemonEndpointTest, RejectsIncompleteEthercatCommit) {
  constexpr std::uint32_t domain_id = 222U;
  policy_runtime::profiles::RobotProfile profile;
  profile.dds.backend = policy_runtime::profiles::RobotIoBackend::dds;
  profile.dds.robot_id = "command_robot";
  profile.dds.local_domain_id = domain_id;
  profile.actuators.push_back({"joint_1_target", {"cia402", "ethercat://1"},
                               {{"safety_group", "arm"}}});
  profile.actuators.push_back({"joint_2_target", {"cia402", "ethercat://2"},
                               {{"safety_group", "arm"}}});
  std::atomic<unsigned> accepted{};
  std::array<double, 2> targets{};
  DdsDaemonCallbacks callbacks;
  callbacks.stage_ethercat_epoch =
      [&](std::string_view group, std::uint64_t epoch,
          std::span<const policy_runtime::robot_io::dds::Cia402CommandValue>
              commands) {
        if (group == "arm" && epoch == 11U && commands.size() == 2U) {
          targets[0] = commands[0].target;
          targets[1] = commands[1].target;
          accepted.fetch_add(1U, std::memory_order_release);
        }
      };
  auto endpoint = DdsDaemonEndpoint::create(profile, std::move(callbacks));
  ASSERT_TRUE(endpoint.has_value()) << endpoint.error().message;
  ASSERT_TRUE(endpoint.value().start().has_value());

  ::dds::domain::DomainParticipant participant(domain_id);
  ::dds::pub::Publisher publisher(participant);
  using Command = policy_runtime::robot_io::Cia402ActuatorCommand;
  ::dds::topic::Topic<Command> topic_1(
      participant, "robot_io_command_robot_actuator_joint_1_target_command");
  ::dds::topic::Topic<Command> topic_2(
      participant, "robot_io_command_robot_actuator_joint_2_target_command");
  ::dds::topic::Topic<policy_runtime::robot_io::EthercatCommandCommit>
      commit_topic(participant,
                   "robot_io_command_robot_ethercat_arm_command_commit");
  ::dds::pub::DataWriter<Command> writer_1(publisher, topic_1);
  ::dds::pub::DataWriter<Command> writer_2(publisher, topic_2);
  ::dds::pub::DataWriter<policy_runtime::robot_io::EthercatCommandCommit>
      commit_writer(publisher, commit_topic);
  for (unsigned attempt = 0; attempt < 100U &&
       (writer_1.publication_matched_status().current_count() == 0 ||
        writer_2.publication_matched_status().current_count() == 0 ||
        commit_writer.publication_matched_status().current_count() == 0);
       ++attempt)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  Command command_1;
  command_1.header().schema_version(policy_runtime::robot_io::DDS_SCHEMA_VERSION);
  command_1.header().robot_id("command_robot");
  command_1.header().device_id("joint_1_target");
  command_1.command_epoch(11U);
  command_1.target(1.0);
  policy_runtime::robot_io::EthercatCommandCommit commit;
  commit.header().schema_version(policy_runtime::robot_io::DDS_SCHEMA_VERSION);
  commit.header().robot_id("command_robot");
  commit.execution_group("arm");
  commit.command_epoch(11U);
  writer_1.write(command_1);
  commit_writer.write(commit);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(accepted.load(std::memory_order_acquire), 0U);

  Command command_2;
  command_2.header().schema_version(policy_runtime::robot_io::DDS_SCHEMA_VERSION);
  command_2.header().robot_id("command_robot");
  command_2.header().device_id("joint_2_target");
  command_2.command_epoch(11U);
  command_2.target(2.0);
  writer_2.write(command_2);
  commit_writer.write(commit);
  for (unsigned attempt = 0; attempt < 100U &&
       accepted.load(std::memory_order_acquire) == 0U; ++attempt)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_EQ(accepted.load(std::memory_order_acquire), 1U);
  EXPECT_DOUBLE_EQ(targets[0], 1.0);
  EXPECT_DOUBLE_EQ(targets[1], 2.0);
}

}  // namespace
