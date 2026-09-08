#pragma once

#include <cstddef>
#include <compare>
#include <string>
#include <optional>
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

struct DeviceBinding {
  std::optional<std::size_t> sensor_profile_index;
  std::optional<std::size_t> actuator_profile_index;
  std::size_t transport_index{};
  std::size_t receive_slot{};
  std::size_t transmit_slot{};
};

struct CompiledTopology {
  std::vector<PhysicalTransportKey> transports;
  std::vector<DeviceBinding> devices;
};

Result<CompiledTopology> compile_physical_topology(
    const profiles::RobotProfile& profile);

}  // namespace policy_runtime::robot_io
