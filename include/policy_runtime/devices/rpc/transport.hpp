#pragma once

#include <string_view>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/devices/rpc/rpc.hpp"

namespace policy_runtime {

// Transport boundary for the RPC software device. Concrete ZMQ/DDS adapters
// own sockets or participants; the RPC codec remains transport-independent.
class RpcTransport {
 public:
  virtual ~RpcTransport() = default;
  virtual Result<void> open() = 0;
  virtual void close() noexcept = 0;
  virtual RpcTransportKind kind() const noexcept = 0;
  virtual std::string_view endpoint() const noexcept = 0;
};

}  // namespace policy_runtime
