#pragma once

#include <cstddef>
#include <compare>
#include <string>
#include <vector>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"

namespace policy_runtime::robot_io {

enum class DeviceDirection { sensor, actuator };
enum class PhysicalTransportKind { ethercat, socketcan, usb_can, serial };

struct PhysicalBusKey {
  PhysicalTransportKind kind{};
  std::string path;

  auto operator<=>(const PhysicalBusKey&) const = default;
};

struct DeviceBinding {
  DeviceDirection direction{};
  std::size_t profile_index{};
  std::size_t bus_index{};
};

struct CompiledTopology {
  std::vector<PhysicalBusKey> buses;
  std::vector<DeviceBinding> devices;
};

Result<CompiledTopology> compile_physical_topology(
    const profiles::RobotProfile& profile);

}  // namespace policy_runtime::robot_io
