#pragma once

#include <filesystem>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/policy_profile.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"

namespace policy_runtime::profiles {

Result<RobotProfile> load_robot_profile(const std::filesystem::path& path);
Result<PolicyProfile> load_policy_profile(const std::filesystem::path& path);

}  // namespace policy_runtime::profiles
