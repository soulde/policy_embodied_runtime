#pragma once

#include <sys/socket.h>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime::robot_io_service_detail {

using ConnectOperation = int (*)(int, const sockaddr*, socklen_t);

Result<void> connect_with_deadline(int descriptor, const sockaddr* address,
                                   socklen_t address_size, int timeout_ms,
                                   ConnectOperation connect_operation);

}  // namespace policy_runtime::robot_io_service_detail
