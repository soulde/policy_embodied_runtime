#pragma once

#include <memory>
#include <string_view>
#include <vector>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/robot_io/daemon/topology.hpp"
#include "policy_runtime/robot_io/daemon/transport_runtime.hpp"
#include "policy_runtime/transport/ethercat/backend.hpp"
#include "policy_runtime/transport/ethercat/master.hpp"

namespace policy_runtime::robot_io {

// Owns construction of physical transports. Device codecs and routing remain
// outside this class; the factory only opens/configures physical I/O.
class TransportFactory final {
 public:
  static Result<void> validate(const PhysicalTransportKey& key);

  static Result<std::unique_ptr<TransportRuntime>> create_socketcan(
      const PhysicalTransportKey& key, TransportRuntime::ReceiveCallback callback,
      std::size_t actuator_slots = 32U);

  static Result<std::unique_ptr<EthercatMaster>> create_ethercat(
      std::shared_ptr<EthercatBackend> backend,
      std::vector<EthercatAxisConfiguration> configurations);
};

}  // namespace policy_runtime::robot_io
