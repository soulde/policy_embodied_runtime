#pragma once

#include <memory>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"

namespace policy_runtime {

class RuntimeRobotIo;

Result<std::unique_ptr<RuntimeRobotIo>> make_dds_runtime_robot_io(
    const profiles::RobotProfile& profile);

}  // namespace policy_runtime
