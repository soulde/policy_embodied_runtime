#include <array>
#include <cstddef>

#include <gtest/gtest.h>

#include "policy_runtime/protocol/damiao/protocol.hpp"

namespace {

using policy_runtime::DamiaoFeedback;
using policy_runtime::DamiaoLimits;
using policy_runtime::DamiaoMitCommand;
using policy_runtime::DamiaoProtocol;
using Frame = DamiaoProtocol::Frame;

TEST(DamiaoProtocolTest, EncodesMitCommandUsingDocumentedBitLayout) {
  const DamiaoLimits limits{12.5F, 30.0F, 10.0F};
  auto result = DamiaoProtocol::encode_mit(
      DamiaoMitCommand{1.0F, 2.0F, 100.0F, 1.0F, -3.0F}, limits);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value(),
            (std::array<std::byte, 8>{std::byte{0x8A}, std::byte{0x3D},
                                       std::byte{0x88}, std::byte{0x83},
                                       std::byte{0x33}, std::byte{0x33},
                                       std::byte{0x35}, std::byte{0x99}}));
}

TEST(DamiaoProtocolTest, DecodesFeedbackAndFaultNibble) {
  const Frame kFeedback{std::byte{0x0AU}, std::byte{0x80U}, std::byte{0x00U},
                        std::byte{0x80U}, std::byte{0x08U}, std::byte{0x00U},
                        std::byte{40U},   std::byte{41U}};
  auto result = DamiaoProtocol::decode_feedback(
      0x01U, kFeedback.data(), kFeedback.size(),
      DamiaoLimits{12.5F, 30.0F, 10.0F});
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result.value().motor_id, 0x0AU);
  EXPECT_EQ(result.value().error_code, 0x0U);
  // Mid-scale code 0x8000 maps to half an LSB above zero; allow one LSB.
  EXPECT_NEAR(result.value().position, 0.0F, 4e-4F);
  EXPECT_EQ(result.value().mos_temperature_c, 40);
}

TEST(DamiaoProtocolTest, RejectsOutOfRangeAndShortFrames) {
  EXPECT_FALSE(DamiaoProtocol::encode_mit(
                   DamiaoMitCommand{20.0F, 0.0F, 0.0F, 0.0F, 0.0F},
                   DamiaoLimits{12.5F, 30.0F, 10.0F})
                   .has_value());
  const Frame kShortFrame{};
  EXPECT_FALSE(DamiaoProtocol::decode_feedback(
                   0x01U, kShortFrame.data(), 7U,
                   DamiaoLimits{12.5F, 30.0F, 10.0F})
                   .has_value());
  EXPECT_FALSE(DamiaoProtocol::decode_feedback(
                   0x800U, kShortFrame.data(), kShortFrame.size(),
                   DamiaoLimits{12.5F, 30.0F, 10.0F})
                   .has_value());
}

}  // namespace
