#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/transport/ethercat/backend.hpp"

namespace policy_runtime {

class ElmoGoldDeviceDescription {
 public:
  static constexpr std::uint32_t vendor_id() noexcept { return 0x0000009AU; }
  static constexpr std::uint32_t product_code() noexcept { return 0x00030924U; }
  static constexpr std::uint32_t revision() noexcept { return 0x00010420U; }
  static constexpr std::uint16_t assign_activate() noexcept { return 0x0300U; }
  static constexpr std::uint32_t sync0_cycle_ns() noexcept { return 1'000'000U; }

  static std::uint16_t rx_pdo(profiles::Cia402Mode mode) noexcept;
  static const PdoDefinition& rx_pdo_definition(profiles::Cia402Mode mode) noexcept;
  static std::span<const PdoDefinition> feedback_pdos() noexcept;
  static std::span<const SyncManagerDescription> sync_managers() noexcept;
  static std::array<SdoDownloadRequest, 2> startup_sdos(profiles::Cia402Mode mode);
};

}  // namespace policy_runtime
