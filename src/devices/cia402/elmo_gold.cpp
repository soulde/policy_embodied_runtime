#include "policy_runtime/devices/cia402/elmo_gold.hpp"

#include <array>

namespace policy_runtime {
namespace {

constexpr std::array kCspEntries{
    PdoEntryDefinition{ObjectAddress{0x607AU, 0U}, 32U},
    PdoEntryDefinition{ObjectAddress{0x60FEU, 1U}, 32U},
    PdoEntryDefinition{ObjectAddress{0x6040U, 0U}, 16U},
};
constexpr std::array kCsvEntries{
    PdoEntryDefinition{ObjectAddress{0x60FFU, 0U}, 32U},
    PdoEntryDefinition{ObjectAddress{0x6040U, 0U}, 16U},
};
constexpr std::array kCstEntries{
    PdoEntryDefinition{ObjectAddress{0x6071U, 0U}, 16U},
    PdoEntryDefinition{ObjectAddress{0x6040U, 0U}, 16U},
};
constexpr PdoDefinition kCspPdo{0x1600U, kCspEntries};
constexpr PdoDefinition kCsvPdo{0x1601U, kCsvEntries};
constexpr PdoDefinition kCstPdo{0x1602U, kCstEntries};

constexpr std::array kPrimaryFeedbackEntries{
    PdoEntryDefinition{ObjectAddress{0x6064U, 0U}, 32U},
    PdoEntryDefinition{ObjectAddress{0x6077U, 0U}, 16U},
    PdoEntryDefinition{ObjectAddress{0x6041U, 0U}, 16U},
    PdoEntryDefinition{ObjectAddress{0x6061U, 0U}, 8U},
    PdoEntryDefinition{ObjectAddress{0x0000U, 0U}, 8U},
};
constexpr std::array kVelocityFeedbackEntries{
    PdoEntryDefinition{ObjectAddress{0x606CU, 0U}, 32U},
};
constexpr std::array kFeedbackPdos{
    PdoDefinition{0x1A02U, kPrimaryFeedbackEntries},
    PdoDefinition{0x1A11U, kVelocityFeedbackEntries},
};

constexpr std::array kSyncManagers{
    SyncManagerDescription{0U, 0x008CU, 0x008CU, 0x008CU, 0x1800U, 0x26U,
                           true, WatchdogMode::device_default,
                           PdoDirection::output},
    SyncManagerDescription{1U, 0x008CU, 0x008CU, 0x008CU, 0x1900U, 0x22U,
                           true, WatchdogMode::device_default,
                           PdoDirection::input},
    SyncManagerDescription{2U, 0U, 0x0020U, 0x0020U, 0x1100U, 0x64U, true,
                           WatchdogMode::device_default, PdoDirection::output},
    SyncManagerDescription{3U, 0U, 0x0020U, 0x0020U, 0x1180U, 0x20U, true,
                           WatchdogMode::device_default, PdoDirection::input},
};

}  // namespace

std::uint16_t ElmoGoldDeviceDescription::rx_pdo(profiles::Cia402Mode mode) noexcept {
  switch (mode) {
    case profiles::Cia402Mode::csp:
      return 0x1600U;
    case profiles::Cia402Mode::csv:
      return 0x1601U;
    case profiles::Cia402Mode::cst:
      return 0x1602U;
  }
  return 0U;
}

const PdoDefinition& ElmoGoldDeviceDescription::rx_pdo_definition(
    profiles::Cia402Mode mode) noexcept {
  switch (mode) {
    case profiles::Cia402Mode::csp:
      return kCspPdo;
    case profiles::Cia402Mode::csv:
      return kCsvPdo;
    case profiles::Cia402Mode::cst:
      return kCstPdo;
  }
  return kCspPdo;
}

std::span<const PdoDefinition> ElmoGoldDeviceDescription::feedback_pdos() noexcept {
  return kFeedbackPdos;
}

std::span<const SyncManagerDescription>
ElmoGoldDeviceDescription::sync_managers() noexcept {
  return kSyncManagers;
}

std::array<SdoDownloadRequest, 2> ElmoGoldDeviceDescription::startup_sdos(
    profiles::Cia402Mode mode) {
  return {
      SdoDownloadRequest{ObjectAddress{0x6060U, 0U},
                         {std::byte{static_cast<unsigned char>(mode)}}},
      SdoDownloadRequest{ObjectAddress{0x60C2U, 1U}, {std::byte{0x02U}}},
  };
}

}  // namespace policy_runtime
