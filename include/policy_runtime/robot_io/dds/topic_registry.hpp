#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"

namespace policy_runtime::robot_io::dds {

enum class DdsTopicKind {
  command,
  state,
  commit,
  health,
};

enum class DdsDeviceKind {
  cia402,
  damiao,
  st3215,
  ethercat,
  robot_io,
};

struct DdsTopicDescriptor {
  DdsTopicKind kind;
  DdsDeviceKind device_kind;
  std::string robot_id;
  std::string device_id;
  std::string topic_name;
  std::string execution_group;
};

Result<std::vector<DdsTopicDescriptor>> build_dds_topic_registry(
    const profiles::RobotProfile& profile, std::string_view robot_id);

}  // namespace policy_runtime::robot_io::dds
