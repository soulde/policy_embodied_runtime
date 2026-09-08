#pragma once

#include <cstddef>
#include <compare>
#include <string>
#include <vector>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"

namespace policy_runtime::robot_io {

enum class PhysicalTransportKind { ethercat, socketcan, usb_can, serial };

struct PhysicalTransportKey {
  PhysicalTransportKind kind{};
  std::string path;

  auto operator<=>(const PhysicalTransportKey&) const = default;
};

struct SensorBinding {
  std::size_t profile_index{};
  std::size_t transport_index{};
  std::size_t transport_slot{};
};

using ActuatorBinding = SensorBinding;

struct CompiledTopology {
  std::vector<PhysicalTransportKey> transports;
  std::vector<SensorBinding> sensors;
  std::vector<ActuatorBinding> actuators;
};

Result<CompiledTopology> compile_physical_topology(
    const profiles::RobotProfile& profile);

}  // namespace policy_runtime::robot_io
