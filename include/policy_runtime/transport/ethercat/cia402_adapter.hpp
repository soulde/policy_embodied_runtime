#pragma once

#include <cstdint>
#include <span>

#include "policy_runtime/protocol/cia402/pdo.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/transport/ethercat/backend.hpp"
#include "policy_runtime/transport/ethercat/master.hpp"

namespace policy_runtime {

// CiA402-specific EtherCAT PDO binding. EthercatMaster owns the physical
// transport; this adapter owns the CiA402 object dictionary and typed fields.
class Cia402PdoAdapter final {
 public:
  static Result<Cia402PdoHandles> configure_axis(
      EthercatBackend& backend, const EthercatAxisConfiguration& configuration);

  static bool read_inputs(const Cia402PdoHandles& handles,
                          std::span<const std::byte> image,
                          Cia402PdoView& pdo) noexcept;

  static bool write_outputs(const Cia402PdoHandles& handles,
                            std::span<std::byte> image,
                            const Cia402PdoView& pdo,
                            profiles::Cia402Mode mode,
                            bool enabled) noexcept;
};

}  // namespace policy_runtime
