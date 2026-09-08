#pragma once

#include <string>

namespace policy_runtime {

enum class RpcTransportKind : unsigned char { zmq, dds };

// Static endpoint configuration for the software RPC device.
struct RpcConfig {
  std::string name;
  RpcTransportKind transport{RpcTransportKind::zmq};
  std::string endpoint{"ipc:///tmp/policy-runtime.sock"};
};

}  // namespace policy_runtime
