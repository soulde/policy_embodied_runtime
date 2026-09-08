#include "policy_runtime/robot_io/daemon/topology.hpp"

#include <filesystem>
#include <map>
#include <string>
#include <utility>

namespace policy_runtime::robot_io {
namespace {

Result<PhysicalTransportKind> transport_kind(
    const profiles::DeviceConfig& device) {
  if (device.device.type == "cia402") {
    return Result<PhysicalTransportKind>::success(PhysicalTransportKind::ethercat);
  }
  if (device.device.type == "st3215") {
    return Result<PhysicalTransportKind>::success(PhysicalTransportKind::serial);
  }
  if (device.device.type == "damiao_motor") {
    const auto selected = device.args.find("transport");
    const auto value = selected == device.args.end() ? "socketcan"
                                                     : selected->second;
    if (value == "socketcan") {
      return Result<PhysicalTransportKind>::success(PhysicalTransportKind::socketcan);
    }
    if (value == "virtual_serial" || value == "usb_can") {
      return Result<PhysicalTransportKind>::success(PhysicalTransportKind::usb_can);
    }
  }
  return Result<PhysicalTransportKind>::failure(
      {ErrorCode::invalid_argument,
       "device '" + device.name + "' has no compatible transport"});
}

std::string canonical_path(PhysicalTransportKind kind, std::string path) {
  if (kind == PhysicalTransportKind::socketcan) return path;
  return std::filesystem::path(std::move(path)).lexically_normal().string();
}

std::string physical_identity(const profiles::DeviceConfig& device) {
  std::string identity = device.device.type + "\n" + device.device.path;
  const auto motor_id = device.args.find("motor_id");
  if (motor_id != device.args.end()) identity += "\n" + motor_id->second;
  return identity;
}

Result<void> append_devices(
    const std::vector<profiles::DeviceConfig>& devices, bool sensor,
    CompiledTopology& topology,
    std::map<PhysicalTransportKey, std::size_t>& transport_indices,
    std::map<std::string, std::size_t>& identities,
    std::vector<std::size_t>& next_slots) {
  for (std::size_t index = 0; index < devices.size(); ++index) {
    const auto& device = devices[index];
    auto kind = transport_kind(device);
    if (!kind.has_value()) return Result<void>::failure(kind.error());
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
      next_slots.push_back(0U);
    }
    const auto transport_index = entry->second;
    const auto slot = next_slots[transport_index]++;
    const auto identity = physical_identity(device);
    if (sensor) {
      auto binding = identities.find(identity);
      auto binding_inserted = binding == identities.end();
      if (!binding_inserted &&
          topology.devices[binding->second].sensor_profile_index.has_value()) {
        const auto unique_identity = identity + "\nsensor\n" +
                                     std::to_string(index);
        binding = identities.emplace(unique_identity, topology.devices.size()).first;
        binding_inserted = true;
      }
      if (binding_inserted) {
        identities.emplace(identity, topology.devices.size());
        topology.devices.push_back(
            {index, std::nullopt, transport_index, slot, slot});
      } else {
        auto& existing = topology.devices[binding->second];
        existing.sensor_profile_index = index;
        existing.receive_slot = slot;
      }
    } else {
      auto found = identities.find(identity);
      if (found != identities.end() &&
          topology.devices[found->second].actuator_profile_index.has_value()) {
        const auto unique_identity = identity + "\nactuator\n" +
                                     std::to_string(index);
        identities.emplace(unique_identity, topology.devices.size());
        topology.devices.push_back(
            {std::nullopt, index, transport_index, slot, slot});
        continue;
      }
      if (found == identities.end()) {
        identities.emplace(identity, topology.devices.size());
        topology.devices.push_back(
            {std::nullopt, index, transport_index, slot, slot});
      } else {
        auto& existing = topology.devices[found->second];
        existing.actuator_profile_index = index;
        existing.transmit_slot = slot;
      }
    }
  }
  return Result<void>::success();
}

}  // namespace

Result<CompiledTopology> compile_physical_topology(
    const profiles::RobotProfile& profile) {
  CompiledTopology topology;
  topology.transports.reserve(profile.sensors.size() + profile.actuators.size());
  topology.devices.reserve(profile.sensors.size() + profile.actuators.size());
  std::map<PhysicalTransportKey, std::size_t> transport_indices;
  std::map<std::string, std::size_t> identities;
  std::vector<std::size_t> next_slots;
  auto appended = append_devices(profile.sensors, true, topology,
                                 transport_indices, identities, next_slots);
  if (!appended.has_value()) return Result<CompiledTopology>::failure(appended.error());
  appended = append_devices(profile.actuators, false, topology,
                            transport_indices, identities, next_slots);
  if (!appended.has_value()) return Result<CompiledTopology>::failure(appended.error());
  return Result<CompiledTopology>::success(std::move(topology));
}

}  // namespace policy_runtime::robot_io
