#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <optional>
#include <unistd.h>
#include <vector>

#include <gtest/gtest.h>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/robot/devices/damiao_motor_bus.hpp"
#include "policy_runtime/robot/devices/damiao_motor_device.hpp"
#include "policy_runtime/robot_io/transport_scheduler.hpp"
#include "policy_runtime/transport/socketcan/socketcan_transport.hpp"

namespace {

using policy_runtime::CanFrame;
using policy_runtime::DamiaoLimits;
using policy_runtime::DamiaoMitCommand;
using policy_runtime::DamiaoMode;
using policy_runtime::DamiaoMotorConfig;
using policy_runtime::DamiaoMotorDevice;
using policy_runtime::DamiaoProtocol;
using policy_runtime::Error;
using policy_runtime::ErrorCode;
using policy_runtime::Result;

// Scriptable stand-in for a CAN transport used to observe one-shot behavior.
class FakeCanTransport {
 public:
  Result<void> send_frame(const CanFrame& frame) {
    if (fail_send) {
      return Result<void>::failure({ErrorCode::io, "injected send failure"});
    }
    sent.push_back(frame);
    return Result<void>::success();
  }

  Result<std::optional<CanFrame>> receive_frame() {
    if (fail_receive) {
      return Result<std::optional<CanFrame>>::failure(
          {ErrorCode::io, "injected receive failure"});
    }
    if (next_receive >= incoming.size()) {
      return Result<std::optional<CanFrame>>::failure(
          {ErrorCode::io, "EAGAIN"});
    }
    return Result<std::optional<CanFrame>>::success(
        incoming[next_receive++]);
  }

  bool fail_send{false};
  bool fail_receive{false};
  std::vector<CanFrame> sent;
  std::vector<CanFrame> incoming;

 private:
  std::size_t next_receive{0};
};

DamiaoMotorConfig sample_config() {
  DamiaoMotorConfig config;
  config.motor_id = 0x01U;
  config.mode = DamiaoMode::mit;
  config.limits = DamiaoLimits{12.5F, 30.0F, 10.0F};
  return config;
}

CanFrame feedback_frame(std::uint32_t id) {
  CanFrame frame{};
  frame.id = id;
  frame.dlc = 8U;
  frame.data = {std::byte{0x01}, std::byte{0x80}, std::byte{0x00},
                std::byte{0x80}, std::byte{0x08}, std::byte{0x00},
                std::byte{40},   std::byte{41}};
  return frame;
}

TEST(DamiaoMotorDeviceTest, CycleSendsOneCommandAndConsumesFeedback) {
  DamiaoMotorDevice device(sample_config());
  FakeCanTransport transport;
  transport.incoming.push_back(feedback_frame(0x01U));

  auto result =
      device.cycle(DamiaoMitCommand{1.0F, 0.0F, 0.0F, 0.0F, 0.0F}, transport,
                   0x01U, 1'000'000'000LL);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(transport.sent.size(), 1U);
  EXPECT_EQ(transport.sent.front().id, 0x01U);
  EXPECT_FALSE(device.fault_latched());
  EXPECT_NEAR(device.feedback().position, 0.0F, 4e-4F);
  EXPECT_FALSE(device.feedback_stale(1'000'000'000LL));
}

TEST(DamiaoMotorDeviceTest, SendFailureLatchesFaultWithoutRetry) {
  DamiaoMotorDevice device(sample_config());
  FakeCanTransport transport;
  transport.fail_send = true;

  auto result = device.cycle(DamiaoMitCommand{}, transport, 0x01U, 0LL);
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(device.fault_latched());
  EXPECT_TRUE(transport.sent.empty());
}

TEST(DamiaoMotorDeviceTest, StaleFeedbackLatchesFault) {
  DamiaoMotorConfig config = sample_config();
  config.feedback_timeout = std::chrono::milliseconds{100};
  DamiaoMotorDevice device(config);
  FakeCanTransport transport;
  transport.incoming.push_back(feedback_frame(0x01U));

  const std::int64_t kNsPerMs = 1'000'000LL;
  ASSERT_TRUE(device
                  .cycle(DamiaoMitCommand{}, transport, 0x01U, 0LL)
                  .has_value());
  // Next cycle receives no new frame and the previous feedback has aged out.
  auto stale = device.cycle(DamiaoMitCommand{}, transport, 0x01U,
                            200 * kNsPerMs);
  EXPECT_FALSE(stale.has_value());
  EXPECT_TRUE(device.fault_latched());
}

TEST(DamiaoMotorDeviceTest, MismatchedMotorIdIsRejected) {
  DamiaoMotorDevice device(sample_config());
  EXPECT_FALSE(device
                   .consume_feedback(0x02U, DamiaoProtocol::Frame{}, 0LL)
                   .has_value());
  EXPECT_TRUE(device.fault_latched());
}

TEST(DamiaoMotorDeviceTest, OutOfRangeCommandIsRejectedBeforeSending) {
  DamiaoMotorDevice device(sample_config());
  FakeCanTransport transport;
  auto result = device.cycle(DamiaoMitCommand{100.0F, 0.0F, 0.0F, 0.0F, 0.0F},
                             transport, 0x01U, 0LL);
  EXPECT_FALSE(result.has_value());
  EXPECT_TRUE(device.fault_latched());
  EXPECT_TRUE(transport.sent.empty());
}

}  // namespace

namespace {

using policy_runtime::DamiaoMotorBus;
using policy_runtime::SocketCanTransport;
using policy_runtime::TransportHealth;
using policy_runtime::TransportScheduler;
using policy_runtime::VirtualCanTransport;

TEST(DamiaoMotorBusTest, RegistersWithSchedulerAndCyclesOneShot) {
  DamiaoMotorConfig config;
  config.motor_id = 0x01U;
  config.limits = DamiaoLimits{12.5F, 30.0F, 10.0F};

  DamiaoMotorBus bus(config, DamiaoMotorBus::CanTransport{
                                  SocketCanTransport{-1}});
  EXPECT_FALSE(bus.open().has_value());

  // A pre-opened PTY pair provides a real descriptor for the serial variant.
  int master = ::open("/dev/ptmx", O_RDWR | O_NOCTTY);
  ASSERT_GE(master, 0);

  DamiaoMotorBus serial_bus(config, DamiaoMotorBus::CanTransport{
                                        VirtualCanTransport{master}});
  ASSERT_TRUE(serial_bus.open().has_value());

  TransportScheduler scheduler;
  ASSERT_TRUE(scheduler.add(serial_bus).has_value());
  EXPECT_TRUE(scheduler.executor_id(serial_bus).has_value());

  serial_bus.stage_command(DamiaoMitCommand{});
  policy_runtime::CycleContext context;
  context.scheduled_start = std::chrono::steady_clock::now();
  serial_bus.cycle(context);
  // No feedback ever arrives, so the cycle faults and health degrades.
  EXPECT_FALSE(serial_bus.last_cycle_result().has_value());
  EXPECT_TRUE(serial_bus.device().fault_latched());
  EXPECT_EQ(serial_bus.health(), TransportHealth::failed);

  ASSERT_TRUE(scheduler.remove(serial_bus).has_value());
  close(master);
}

}  // namespace
