#include <chrono>

#include <gtest/gtest.h>

#include "policy_runtime/robot_io/daemon/transport_factory.hpp"
#include "policy_runtime/transport/serial/serial_transport.hpp"

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

TEST(TransportFactoryTest, CreatesClosedSerialTransportFromStaticConfiguration) {
  const policy_runtime::SerialConfig config{
      "/dev/ttyFAKE", 1'000'000U, 64U, 259U,
      std::chrono::milliseconds{0}, std::chrono::milliseconds{0}};

  auto result = TransportFactory::create_serial(config);

  ASSERT_TRUE(result.has_value()) << result.error().message;
  ASSERT_NE(result.value(), nullptr);
  EXPECT_FALSE(result.value()->is_open());
}

}  // namespace
