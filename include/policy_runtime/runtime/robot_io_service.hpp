#pragma once

#include <string_view>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/runtime/robot_io_client.hpp"

namespace policy_runtime {

// Opens an owner-only daemon generation file, connects its owner-only Unix
// SOCK_SEQPACKET listener, and consumes the connected descriptor through the
// normal generation-validating RobotIoClient handshake.
Result<RobotIoClient> connect_robot_io_service(
    std::string_view socket_path, std::string_view generation_path,
    int setup_timeout_ms);

}  // namespace policy_runtime
