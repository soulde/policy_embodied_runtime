#include <array>
#include <cstdint>
#include <span>

#include <gtest/gtest.h>

#include "policy_runtime/devices/cia402/elmo_gold.hpp"

namespace {

using policy_runtime::ElmoGoldDeviceDescription;
using policy_runtime::ObjectAddress;
using policy_runtime::PdoEntryDefinition;
using policy_runtime::TypedPdoField;
using policy_runtime::WatchdogMode;
using policy_runtime::profiles::Cia402Mode;

void expect_entries(std::span<const PdoEntryDefinition> actual,
                    std::span<const PdoEntryDefinition> expected) {
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t index = 0; index < expected.size(); ++index) {
    EXPECT_EQ(actual[index].address.index, expected[index].address.index);
    EXPECT_EQ(actual[index].address.subindex, expected[index].address.subindex);
    EXPECT_EQ(actual[index].bit_length, expected[index].bit_length);
  }
}

TEST(ElmoPdoTest, SelectsEsiIdentityModeSpecificRxPdoAndDc) {
  EXPECT_EQ(ElmoGoldDeviceDescription::vendor_id(), 0x0000009AU);
  EXPECT_EQ(ElmoGoldDeviceDescription::product_code(), 0x00030924U);
  EXPECT_EQ(ElmoGoldDeviceDescription::revision(), 0x00010420U);
  EXPECT_EQ(ElmoGoldDeviceDescription::rx_pdo(Cia402Mode::csp), 0x1600U);
  EXPECT_EQ(ElmoGoldDeviceDescription::rx_pdo(Cia402Mode::csv), 0x1601U);
  EXPECT_EQ(ElmoGoldDeviceDescription::rx_pdo(Cia402Mode::cst), 0x1602U);
  EXPECT_EQ(ElmoGoldDeviceDescription::assign_activate(), 0x0300U);
  EXPECT_EQ(ElmoGoldDeviceDescription::sync0_cycle_ns(), 1'000'000U);
}

TEST(ElmoPdoTest, MatchesEsiSyncManagersAndModeSpecificOutputEntries) {
  const auto sync_managers = ElmoGoldDeviceDescription::sync_managers();
  ASSERT_EQ(sync_managers.size(), 4U);
  EXPECT_EQ(sync_managers[0].index, 0U);
  EXPECT_EQ(sync_managers[0].start_address, 0x1800U);
  EXPECT_EQ(sync_managers[0].default_size, 0x008CU);
  EXPECT_EQ(sync_managers[0].control_byte, 0x26U);
  EXPECT_EQ(sync_managers[2].index, 2U);
  EXPECT_EQ(sync_managers[2].start_address, 0x1100U);
  EXPECT_EQ(sync_managers[2].default_size, 0x0020U);
  EXPECT_EQ(sync_managers[2].maximum_size, 0x0020U);
  EXPECT_EQ(sync_managers[2].control_byte, 0x64U);
  EXPECT_EQ(sync_managers[2].watchdog, WatchdogMode::device_default);
  EXPECT_EQ(sync_managers[3].index, 3U);
  EXPECT_EQ(sync_managers[3].start_address, 0x1180U);
  EXPECT_EQ(sync_managers[3].control_byte, 0x20U);
  EXPECT_EQ(sync_managers[3].watchdog, WatchdogMode::device_default);

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
  expect_entries(ElmoGoldDeviceDescription::rx_pdo_definition(Cia402Mode::csp).entries,
                 kCspEntries);
  expect_entries(ElmoGoldDeviceDescription::rx_pdo_definition(Cia402Mode::csv).entries,
                 kCsvEntries);
  expect_entries(ElmoGoldDeviceDescription::rx_pdo_definition(Cia402Mode::cst).entries,
                 kCstEntries);
}

TEST(ElmoPdoTest, SelectsMinimalEsiFeedbackPdosCoveringEveryAxisField) {
  const auto feedback_pdos = ElmoGoldDeviceDescription::feedback_pdos();
  ASSERT_EQ(feedback_pdos.size(), 2U);
  EXPECT_EQ(feedback_pdos[0].index, 0x1A02U);
  EXPECT_EQ(feedback_pdos[1].index, 0x1A11U);

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
  expect_entries(feedback_pdos[0].entries, kPrimaryFeedbackEntries);
  expect_entries(feedback_pdos[1].entries, kVelocityFeedbackEntries);
}

TEST(ElmoPdoTest, BuildsStaticModeAndEsiCycleTimeStartupSdos) {
  for (const auto& [mode, mode_value] :
       std::array{std::pair{Cia402Mode::csp, std::byte{8U}},
                  std::pair{Cia402Mode::csv, std::byte{9U}},
                  std::pair{Cia402Mode::cst, std::byte{10U}}}) {
    const auto requests = ElmoGoldDeviceDescription::startup_sdos(mode);
    ASSERT_EQ(requests.size(), 2U);
    EXPECT_EQ(requests[0].address.index, 0x6060U);
    EXPECT_EQ(requests[0].address.subindex, 0U);
    EXPECT_EQ(requests[0].data, (std::vector<std::byte>{mode_value}));
    EXPECT_EQ(requests[1].address.index, 0x60C2U);
    EXPECT_EQ(requests[1].address.subindex, 1U);
    EXPECT_EQ(requests[1].data, (std::vector<std::byte>{std::byte{0x02U}}));
  }
}

TEST(PdoFieldTest, BindsTypedByteAlignedFieldsAndUsesEthercatLittleEndian) {
  TypedPdoField<std::int32_t> field;
  std::array<std::byte, 6> image{};
  EXPECT_FALSE(field.read(image).has_value());
  EXPECT_FALSE(field.write(image, -0x01020304));

  ASSERT_TRUE(field.bind({1U, 0U, 32U}).has_value());
  ASSERT_TRUE(field.write(image, -0x01020304));
  EXPECT_EQ(image[1], std::byte{0xFCU});
  EXPECT_EQ(image[2], std::byte{0xFCU});
  EXPECT_EQ(image[3], std::byte{0xFDU});
  EXPECT_EQ(image[4], std::byte{0xFEU});
  EXPECT_EQ(field.read(image), -0x01020304);

  std::array<std::byte, 4> short_image{};
  EXPECT_FALSE(field.read(short_image).has_value());
  EXPECT_FALSE(field.write(short_image, 1));
}

TEST(PdoFieldTest, RejectsRebindingBitFieldsAndMismatchedWidths) {
  TypedPdoField<std::uint16_t> field;
  const auto bit_field = field.bind({0U, 1U, 16U});
  ASSERT_FALSE(bit_field.has_value());
  EXPECT_EQ(bit_field.error().code, policy_runtime::ErrorCode::invalid_argument);

  const auto wrong_width = field.bind({0U, 0U, 8U});
  ASSERT_FALSE(wrong_width.has_value());
  EXPECT_EQ(wrong_width.error().code, policy_runtime::ErrorCode::invalid_argument);

  ASSERT_TRUE(field.bind({0U, 0U, 16U}).has_value());
  const auto rebound = field.bind({2U, 0U, 16U});
  ASSERT_FALSE(rebound.has_value());
  EXPECT_EQ(rebound.error().code, policy_runtime::ErrorCode::invalid_argument);
}

}  // namespace
