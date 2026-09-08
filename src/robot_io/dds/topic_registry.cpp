#include "policy_runtime/robot_io/dds/topic_registry.hpp"

#include <map>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace policy_runtime::robot_io::dds {
namespace {

Result<std::string> normalize_identifier(std::string_view value,
                                         std::string_view label) {
  std::string normalized;
  bool separator_pending = false;
  for (const auto character : value) {
    const auto byte = static_cast<unsigned char>(character);
    if ((byte >= 'a' && byte <= 'z') || (byte >= 'A' && byte <= 'Z') ||
        (byte >= '0' && byte <= '9')) {
      if (separator_pending && !normalized.empty()) {
        normalized.push_back('_');
      }
      separator_pending = false;
      normalized.push_back(
          static_cast<char>(byte >= 'A' && byte <= 'Z' ? byte + ('a' - 'A')
                                                        : byte));
    } else {
      separator_pending = true;
    }
  }
  if (normalized.empty()) {
    return Result<std::string>::failure(
        {ErrorCode::invalid_argument,
         std::string(label) + " must contain an ASCII letter or digit"});
  }
  return Result<std::string>::success(std::move(normalized));
}

Result<DdsDeviceKind> device_kind_for(const profiles::DeviceConfig& device) {
  if (device.device.type == "cia402") {
    return Result<DdsDeviceKind>::success(DdsDeviceKind::cia402);
  }
  if (device.device.type == "damiao_motor") {
    return Result<DdsDeviceKind>::success(DdsDeviceKind::damiao);
  }
  if (device.device.type == "st3215") {
    return Result<DdsDeviceKind>::success(DdsDeviceKind::st3215);
  }
  return Result<DdsDeviceKind>::failure(
      {ErrorCode::invalid_argument,
       "DDS does not support device type '" + device.device.type + "'"});
}

Result<std::string> ethercat_group_for(const profiles::DeviceConfig& device) {
  const auto group = device.args.find("safety_group");
  if (group == device.args.end()) {
    return Result<std::string>::failure(
        {ErrorCode::invalid_argument,
         "cia402 device '" + device.name + "' requires safety_group"});
  }
  return normalize_identifier(group->second, "cia402 safety_group");
}

Result<void> append_descriptor(std::vector<DdsTopicDescriptor>& descriptors,
                               std::set<std::string>& topic_names,
                               DdsTopicDescriptor descriptor) {
  if (!topic_names.insert(descriptor.topic_name).second) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "DDS topic name collision after normalization: '" +
             descriptor.topic_name + "'"});
  }
  descriptors.push_back(std::move(descriptor));
  return Result<void>::success();
}

Result<void> append_device_descriptors(
    const std::vector<profiles::DeviceConfig>& devices, DdsTopicKind kind,
    std::string_view direction, std::string_view suffix,
    const std::string& robot_id, const std::string& normalized_robot_id,
    std::vector<DdsTopicDescriptor>& descriptors, std::set<std::string>& topic_names,
    std::map<std::string, std::string>& ethercat_groups) {
  for (const auto& device : devices) {
    auto device_kind = device_kind_for(device);
    if (!device_kind.has_value()) {
      return Result<void>::failure(device_kind.error());
    }
    auto normalized_device_id = normalize_identifier(device.name, "device_id");
    if (!normalized_device_id.has_value()) {
      return Result<void>::failure(normalized_device_id.error());
    }

    std::string execution_group;
    if (device_kind.value() == DdsDeviceKind::cia402) {
      auto group = ethercat_group_for(device);
      if (!group.has_value()) {
        return Result<void>::failure(group.error());
      }
      execution_group = device.args.at("safety_group");
      if (kind == DdsTopicKind::command) {
        const auto [existing, inserted] =
            ethercat_groups.emplace(group.value(), execution_group);
        if (!inserted && existing->second != execution_group) {
          return Result<void>::failure(
              {ErrorCode::invalid_argument,
               "EtherCAT execution-group collision after normalization: '" +
                   group.value() + "'"});
        }
      }
    }

    auto appended = append_descriptor(
        descriptors, topic_names,
        DdsTopicDescriptor{kind,
                           device_kind.value(),
                           robot_id,
                           device.name,
                           "robot_io_" + normalized_robot_id + "_" +
                               std::string(direction) + "_" +
                               normalized_device_id.value() + "_" +
                               std::string(suffix),
                           std::move(execution_group)});
    if (!appended.has_value()) {
      return appended;
    }
  }
  return Result<void>::success();
}

}  // namespace

Result<std::vector<DdsTopicDescriptor>> build_dds_topic_registry(
    const profiles::RobotProfile& profile, std::string_view robot_id) {
  auto normalized_robot_id = normalize_identifier(robot_id, "robot_id");
  if (!normalized_robot_id.has_value()) {
    return Result<std::vector<DdsTopicDescriptor>>::failure(
        normalized_robot_id.error());
  }

  std::vector<DdsTopicDescriptor> descriptors;
  descriptors.reserve(profile.sensors.size() + profile.actuators.size() + 2U);
  std::set<std::string> topic_names;
  std::map<std::string, std::string> ethercat_groups;
  const std::string robot_id_string(robot_id);

  auto appended = append_device_descriptors(
      profile.sensors, DdsTopicKind::state, "sensor", "state", robot_id_string,
      normalized_robot_id.value(), descriptors, topic_names, ethercat_groups);
  if (!appended.has_value()) {
    return Result<std::vector<DdsTopicDescriptor>>::failure(appended.error());
  }
  appended = append_device_descriptors(
      profile.actuators, DdsTopicKind::command, "actuator", "command",
      robot_id_string, normalized_robot_id.value(), descriptors, topic_names,
      ethercat_groups);
  if (!appended.has_value()) {
    return Result<std::vector<DdsTopicDescriptor>>::failure(appended.error());
  }

  for (const auto& [normalized_group, group] : ethercat_groups) {
    appended = append_descriptor(
        descriptors, topic_names,
        DdsTopicDescriptor{DdsTopicKind::commit,
                           DdsDeviceKind::ethercat,
                           robot_id_string,
                           group,
                           "robot_io_" + normalized_robot_id.value() +
                               "_ethercat_" + normalized_group +
                               "_command_commit",
                           group});
    if (!appended.has_value()) {
      return Result<std::vector<DdsTopicDescriptor>>::failure(appended.error());
    }
  }

  appended = append_descriptor(
      descriptors, topic_names,
      DdsTopicDescriptor{DdsTopicKind::health,
                         DdsDeviceKind::robot_io,
                         robot_id_string,
                         "health",
                         "robot_io_" + normalized_robot_id.value() + "_health",
                         {}});
  if (!appended.has_value()) {
    return Result<std::vector<DdsTopicDescriptor>>::failure(appended.error());
  }
  return Result<std::vector<DdsTopicDescriptor>>::success(std::move(descriptors));
}

}  // namespace policy_runtime::robot_io::dds
