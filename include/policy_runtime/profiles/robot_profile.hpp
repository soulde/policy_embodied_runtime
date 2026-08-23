#pragma once

#include <chrono>
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
  double scale{};
  double minimum{};
  double maximum{};
  std::chrono::milliseconds command_timeout{};
  std::string safety_group;
};

struct RobotProfile {
  std::vector<DeviceConfig> sensors;
  std::vector<DeviceConfig> actuators;
  std::vector<AxisConfig> axes;
};

}  // namespace policy_runtime::profiles
