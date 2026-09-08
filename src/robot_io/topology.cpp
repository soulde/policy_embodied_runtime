#include "policy_runtime/robot_io/topology.hpp"

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
    DeviceDirection direction, CompiledTopology& topology,
    std::map<PhysicalBusKey, std::size_t>& bus_indices) {
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
    PhysicalBusKey key{kind.value(),
                       canonical_path(kind.value(), device.device.path)};
    const auto [entry, inserted] =
        bus_indices.emplace(key, topology.buses.size());
    if (inserted) {
      topology.buses.push_back(std::move(key));
    }
    topology.devices.push_back({direction, index, entry->second});
  }
  return Result<void>::success();
}

}  // namespace

Result<CompiledTopology> compile_physical_topology(
    const profiles::RobotProfile& profile) {
  CompiledTopology topology;
  topology.buses.reserve(profile.sensors.size() + profile.actuators.size());
  topology.devices.reserve(profile.sensors.size() + profile.actuators.size());
  std::map<PhysicalBusKey, std::size_t> bus_indices;
  auto appended = append_devices(profile.sensors, DeviceDirection::sensor,
                                 topology, bus_indices);
  if (!appended.has_value()) {
    return Result<CompiledTopology>::failure(appended.error());
  }
  appended = append_devices(profile.actuators, DeviceDirection::actuator,
                            topology, bus_indices);
  if (!appended.has_value()) {
    return Result<CompiledTopology>::failure(appended.error());
  }
  return Result<CompiledTopology>::success(std::move(topology));
}

}  // namespace policy_runtime::robot_io
