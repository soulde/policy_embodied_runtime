#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace policy_runtime::profiles {

struct DeviceLink {
  std::string type;
  std::string path;
};

struct DeviceConfig {
  std::string name;
  DeviceLink device;
  std::map<std::string, std::string> args;
};

enum class Cia402Mode : std::int8_t { csp = 8, csv = 9, cst = 10 };

struct AxisConfig {
  std::string name;
  std::uint16_t alias{};
  std::uint16_t position{};
  std::uint32_t vendor_id{};
  std::uint32_t product_code{};
  std::uint32_t revision{};
  Cia402Mode mode{};
  // Device counts per engineering unit for the configured cyclic mode.
  double scale{};
  double minimum{};
  double maximum{};
  std::chrono::milliseconds command_timeout{};
  std::string safety_group;
  // Maximum engineering-unit target change per daemon cycle.
  double slew_limit{};
  // Maximum target-to-actual error in engineering units.
  double following_error_limit{};
};

struct SerialPortConfig {
  std::string path;
  std::uint32_t baud_rate{};
  std::size_t read_buffer_size{256U};
  std::size_t maximum_frame_size{259U};
  std::chrono::milliseconds read_timeout{20};
  std::chrono::milliseconds write_timeout{20};
  std::chrono::nanoseconds service_period{std::chrono::milliseconds{1}};
};

struct St3215ServoProfile {
  std::string sensor_name;
  std::string actuator_name;
  SerialPortConfig serial;
  std::uint8_t device_id{};
  std::uint16_t servo_id{};
  std::uint16_t max_position_units{4095U};
  std::uint16_t speed_units{};
  std::uint16_t time_units{};
  std::chrono::milliseconds feedback_timeout{250};
  std::string safety_group;
  std::chrono::milliseconds command_timeout{250};
  std::chrono::milliseconds maximum_command_future{50};
};

struct RobotProfile {
  std::vector<DeviceConfig> sensors;
  std::vector<DeviceConfig> actuators;
  std::vector<AxisConfig> axes;
  std::vector<St3215ServoProfile> st3215_servos;
};

}  // namespace policy_runtime::profiles
