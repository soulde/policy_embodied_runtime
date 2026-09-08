#include "policy_runtime/robot_io/topology.hpp"

#include <algorithm>
#include <filesystem>
#include <map>
#include <string>
#include <utility>

namespace policy_runtime::robot_io {
namespace {

Result<PhysicalTransportKind> transport_kind(
    const profiles::DeviceConfig& device) {
  if (device.device.type == "cia402") {
    return Result<PhysicalTransportKind>::success(
        PhysicalTransportKind::ethercat);
  }
  if (device.device.type == "st3215") {
    return Result<PhysicalTransportKind>::success(
        PhysicalTransportKind::serial);
  }
  if (device.device.type == "damiao_motor") {
    const auto selected = device.args.find("transport");
    const auto value = selected == device.args.end() ? "socketcan"
                                                     : selected->second;
    if (value == "socketcan") {
      return Result<PhysicalTransportKind>::success(
          PhysicalTransportKind::socketcan);
    }
    if (value == "virtual_serial" || value == "usb_can") {
      return Result<PhysicalTransportKind>::success(
          PhysicalTransportKind::usb_can);
    }
  }
  return Result<PhysicalTransportKind>::failure(
      {ErrorCode::invalid_argument,
       "device '" + device.name + "' has no compatible transport"});
}

std::string canonical_path(PhysicalTransportKind kind, std::string path) {
  if (kind == PhysicalTransportKind::socketcan) {
    return path;
  }
  return std::filesystem::path(std::move(path)).lexically_normal().string();
}

Result<void> append_devices(
    const std::vector<profiles::DeviceConfig>& devices,
    bool sensor, CompiledTopology& topology,
    std::map<PhysicalTransportKey, std::size_t>& transport_indices) {
  for (std::size_t index = 0; index < devices.size(); ++index) {
    const auto& device = devices[index];
    auto kind = transport_kind(device);
    if (!kind.has_value()) {
      return Result<void>::failure(kind.error());
    }
    if (device.device.path.empty()) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument,
           "device '" + device.name + "' requires a transport path"});
    }
    PhysicalTransportKey key{kind.value(),
                             canonical_path(kind.value(), device.device.path)};
    const auto [entry, inserted] =
        transport_indices.emplace(key, topology.transports.size());
    if (inserted) {
      topology.transports.push_back(std::move(key));
    }
    std::size_t slot = 0U;
    for (const auto& binding : topology.sensors) {
      if (binding.transport_index == entry->second) {
        slot = std::max(slot, binding.transport_slot + 1U);
      }
    }
    for (const auto& binding : topology.actuators) {
      if (binding.transport_index == entry->second) {
        slot = std::max(slot, binding.transport_slot + 1U);
      }
    }
    if (sensor) {
      topology.sensors.push_back({index, entry->second, slot});
    } else {
      topology.actuators.push_back({index, entry->second, slot});
    }
  }
  return Result<void>::success();
}

}  // namespace

Result<CompiledTopology> compile_physical_topology(
    const profiles::RobotProfile& profile) {
  CompiledTopology topology;
  topology.transports.reserve(profile.sensors.size() + profile.actuators.size());
  topology.sensors.reserve(profile.sensors.size());
  topology.actuators.reserve(profile.actuators.size());
  std::map<PhysicalTransportKey, std::size_t> transport_indices;
  auto appended = append_devices(profile.sensors, true,
                                 topology, transport_indices);
  if (!appended.has_value()) {
    return Result<CompiledTopology>::failure(appended.error());
  }
  appended = append_devices(profile.actuators, false,
                            topology, transport_indices);
  if (!appended.has_value()) {
    return Result<CompiledTopology>::failure(appended.error());
  }
  return Result<CompiledTopology>::success(std::move(topology));
}

}  // namespace policy_runtime::robot_io
