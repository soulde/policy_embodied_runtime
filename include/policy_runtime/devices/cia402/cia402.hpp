#pragma once

#include "policy_runtime/profiles/robot_profile.hpp"

namespace policy_runtime {

// CiA 402 uses the profile axis configuration as its static device
// configuration. The alias gives the device family a stable config name while
// preserving the existing profile ABI.
using Cia402Config = profiles::AxisConfig;

}  // namespace policy_runtime
