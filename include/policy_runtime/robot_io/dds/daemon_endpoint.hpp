#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string_view>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "robot_io.hpp"

namespace policy_runtime::robot_io::dds {

struct Cia402CommandValue {
  double target{};
  std::int32_t mode{};
  std::uint32_t flags{};
  std::uint64_t sequence{};
  std::int64_t source_timestamp_ns{};
};

struct DamiaoCommandValue {
  bool enabled{};
  bool fault_reset{};
  float position{};
  float velocity{};
  float kp{};
  float kd{};
  float torque{};
  std::uint64_t sequence{};
  std::int64_t source_timestamp_ns{};
};

struct St3215CommandValue {
  double target_radians{};
  bool enabled{};
  bool fault_reset{};
  std::uint64_t sequence{};
  std::int64_t source_timestamp_ns{};
};

struct DdsDaemonCallbacks {
  std::function<void(std::string_view execution_group, std::uint64_t epoch,
                     std::span<const Cia402CommandValue> commands)>
      stage_ethercat_epoch;
  std::function<void(std::size_t actuator_index,
                     const DamiaoCommandValue& command)>
      stage_damiao_command;
  std::function<void(std::size_t actuator_index,
                     const St3215CommandValue& command)>
      stage_st3215_command;
};

struct Cia402SensorEvent {
  std::uint64_t sequence{};
  std::int64_t source_timestamp_ns{};
  std::uint64_t bus_cycle{};
  std::uint32_t flags{};
  double position{};
  double velocity{};
  double torque{};
  std::uint16_t status_word{};
  std::int32_t mode_display{};
};

struct DamiaoSensorEvent {
  std::uint64_t sequence{};
  std::int64_t source_timestamp_ns{};
  std::uint64_t bus_cycle{};
  std::uint32_t flags{};
  float position{};
  float velocity{};
  float torque{};
  std::uint8_t motor_id{};
  std::uint8_t error_code{};
  std::int16_t mos_temperature_c{};
  std::int16_t rotor_temperature_c{};
};

struct St3215SensorEvent {
  std::uint64_t sequence{};
  std::int64_t source_timestamp_ns{};
  std::uint64_t bus_cycle{};
  std::uint32_t flags{};
  double position_radians{};
  std::uint16_t position_units{};
  std::uint16_t error_flags{};
};

struct RobotIoHealthEvent {
  std::uint64_t sequence{};
  std::int64_t source_timestamp_ns{};
  std::uint32_t health_flags{};
  std::uint32_t dropped_samples{};
};

class DdsDaemonEndpoint {
 public:
  static Result<DdsDaemonEndpoint> create(
      const profiles::RobotProfile& profile, DdsDaemonCallbacks callbacks);

  DdsDaemonEndpoint(DdsDaemonEndpoint&&) noexcept;
  DdsDaemonEndpoint& operator=(DdsDaemonEndpoint&&) noexcept;
  DdsDaemonEndpoint(const DdsDaemonEndpoint&) = delete;
  DdsDaemonEndpoint& operator=(const DdsDaemonEndpoint&) = delete;
  ~DdsDaemonEndpoint();

  Result<void> start();
  void stop() noexcept;
  Result<void> poll_health();
  bool enqueue_cia402_sensor_realtime(
      std::size_t sensor_index, const Cia402SensorEvent& event) noexcept;
  bool enqueue_damiao_sensor_realtime(
      std::size_t sensor_index, const DamiaoSensorEvent& event) noexcept;
  bool enqueue_st3215_sensor_realtime(
      std::size_t sensor_index, const St3215SensorEvent& event) noexcept;
  bool enqueue_health_realtime(const RobotIoHealthEvent& event) noexcept;
  std::uint64_t dropped_cia402_sensor_samples(
      std::size_t sensor_index) const noexcept;

 private:
  struct Impl;
  explicit DdsDaemonEndpoint(std::unique_ptr<Impl> implementation) noexcept;
  std::unique_ptr<Impl> implementation_;
};

}  // namespace policy_runtime::robot_io::dds
