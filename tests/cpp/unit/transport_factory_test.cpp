#include <gtest/gtest.h>

#include "policy_runtime/robot_io/daemon/transport_factory.hpp"

namespace {

using policy_runtime::ErrorCode;
using policy_runtime::robot_io::TransportFactory;

TEST(TransportFactoryTest, RejectsUnsupportedPhysicalTransportKind) {
  const policy_runtime::robot_io::PhysicalTransportKey key{
      policy_runtime::robot_io::PhysicalTransportKind::usb_can, "/dev/ttyUSB0"};

  auto result = TransportFactory::validate(key);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ErrorCode::invalid_argument);
}

TEST(TransportFactoryTest, RejectsEmptyPhysicalTransportPath) {
  const policy_runtime::robot_io::PhysicalTransportKey key{
      policy_runtime::robot_io::PhysicalTransportKind::socketcan, ""};

  auto result = TransportFactory::validate(key);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ErrorCode::invalid_argument);
}

}  // namespace
