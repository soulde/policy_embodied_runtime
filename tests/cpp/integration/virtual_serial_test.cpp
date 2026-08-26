#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "policy_runtime/protocol/st3215/protocol.hpp"
#include "policy_runtime/robot/devices/st3215/servo.hpp"
#include "policy_runtime/robot_io/daemon.hpp"
#include "policy_runtime/robot_io/ipc_server.hpp"
#include "policy_runtime/robot_io/transport_scheduler.hpp"
#include "policy_runtime/runtime/robot_io_client.hpp"
#include "policy_runtime/transport/serial/serial_transport.hpp"

namespace {

using namespace std::chrono_literals;
using policy_runtime::Bytes;
using policy_runtime::ErrorCode;
using policy_runtime::SerialConfig;
using policy_runtime::SerialTransport;
using policy_runtime::St3215Protocol;

class PtyEndpoint {
 public:
  PtyEndpoint() {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (master_ < 0 || ::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
      throw std::runtime_error("unable to create PTY");
    }
    std::array<char, 256> path{};
    if (::ptsname_r(master_, path.data(), path.size()) != 0) {
      throw std::runtime_error("unable to resolve PTY path");
    }
    path_ = path.data();
  }

  ~PtyEndpoint() {
    if (master_ >= 0) {
      ::close(master_);
    }
  }

  PtyEndpoint(const PtyEndpoint&) = delete;
  PtyEndpoint& operator=(const PtyEndpoint&) = delete;

  const std::string& path() const noexcept { return path_; }

  Bytes read_frame(std::chrono::milliseconds timeout) const {
    Bytes data;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      struct pollfd descriptor { master_, POLLIN, 0 };
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
      const int ready = ::poll(&descriptor, 1, std::max(1, static_cast<int>(remaining.count())));
      if (ready <= 0) {
        continue;
      }
      std::array<std::byte, 32> chunk{};
      const auto count = ::read(master_, chunk.data(), chunk.size());
      if (count > 0) {
        data.insert(data.end(), chunk.begin(), chunk.begin() + count);
        if (data.size() >= 4U) {
          const auto expected = std::to_integer<std::uint8_t>(data[3]) + 4U;
          if (data.size() >= expected) {
            data.resize(expected);
            return data;
          }
        }
      }
    }
    return data;
  }

  void write_chunks(std::span<const std::byte> data,
                    std::size_t chunk_size = 1U) const {
    std::size_t offset{};
    while (offset < data.size()) {
      const auto count = std::min(chunk_size, data.size() - offset);
      const auto written = ::write(master_, data.data() + offset, count);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
      }
    }
  }

 private:
  int master_{-1};
  std::string path_;
};

SerialConfig config_for(const PtyEndpoint& endpoint) {
  return SerialConfig{endpoint.path(), 1'000'000U, 64U, 259U, 40ms, 40ms};
}

}  // namespace

TEST(VirtualSerialTest, ReassemblesPartialPtyStatusAndPreservesFrameBoundary) {
  PtyEndpoint endpoint;
  SerialTransport transport{config_for(endpoint)};
  ASSERT_TRUE(transport.open().has_value());
  const auto request = St3215Protocol::read_present_position_command(1);
  ASSERT_TRUE(transport.write(0, request).has_value());

  std::thread device([&] {
    const auto received = endpoint.read_frame(200ms);
    EXPECT_EQ(received, request);
    const auto response = St3215Protocol::status_packet(
        1, 0, std::array<std::byte, 2>{std::byte{0x00}, std::byte{0x08}});
    endpoint.write_chunks(response, 1U);
  });
  transport.cycle({});
  device.join();

  std::array<std::byte, 259> response{};
  auto read = transport.read(0, response);
  ASSERT_TRUE(read.has_value()) << read.error().message;
  EXPECT_EQ(Bytes(response.begin(), response.begin() + read.value()),
            St3215Protocol::status_packet(
                1, 0, std::array<std::byte, 2>{std::byte{0x00},
                                               std::byte{0x08}}));
  transport.close();
}

TEST(VirtualSerialTest, BoundsTimeoutAndRejectsDuplicatePhysicalPortOpen) {
  PtyEndpoint endpoint;
  SerialTransport first{config_for(endpoint)};
  SerialTransport duplicate{config_for(endpoint)};
  ASSERT_TRUE(first.open().has_value());
  auto duplicate_open = duplicate.open();
  ASSERT_FALSE(duplicate_open.has_value());
  EXPECT_EQ(duplicate_open.error().code, ErrorCode::unavailable);

  ASSERT_TRUE(first.write(0, St3215Protocol::ping_command(1)).has_value());
  const auto started = std::chrono::steady_clock::now();
  first.cycle({});
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(elapsed, 250ms);
  EXPECT_EQ(first.last_error(), policy_runtime::SerialError::timeout);
  EXPECT_EQ(first.health(), policy_runtime::TransportHealth::degraded);

  std::array<std::byte, 32> response{};
  auto read = first.read(0, response);
  ASSERT_FALSE(read.has_value());
  EXPECT_EQ(read.error().code, ErrorCode::timeout);
  first.close();

  ASSERT_TRUE(duplicate.open().has_value());
  duplicate.close();
}

TEST(VirtualSerialTest, RejectsUnboundedIoTimeoutConfiguration) {
  PtyEndpoint endpoint;
  auto config = config_for(endpoint);
  config.read_timeout = 101ms;
  SerialTransport transport{config};
  auto opened = transport.open();
  ASSERT_FALSE(opened.has_value());
  EXPECT_EQ(opened.error().code, ErrorCode::invalid_argument);
}

TEST(VirtualSerialTest, ReportsQueueSaturationAsADegradedPreciseError) {
  PtyEndpoint endpoint;
  SerialTransport transport{config_for(endpoint)};
  ASSERT_TRUE(transport.open().has_value());
  const auto request = St3215Protocol::ping_command(1);
  for (unsigned index = 0; index < 64U; ++index) {
    ASSERT_TRUE(transport.write(0, request).has_value());
  }

  auto saturated = transport.write(0, request);
  ASSERT_FALSE(saturated.has_value());
  EXPECT_EQ(saturated.error().code, ErrorCode::unavailable);
  EXPECT_EQ(saturated.error().message, "serial transmit queue is full");
  EXPECT_EQ(transport.last_error(), policy_runtime::SerialError::queue_full);
  EXPECT_EQ(transport.health(), policy_runtime::TransportHealth::degraded);
  transport.close();
}

TEST(VirtualSerialTest, StopCancelsAnInFlightSerialReceive) {
  PtyEndpoint endpoint;
  auto config = config_for(endpoint);
  config.read_timeout = 100ms;
  auto transport = std::make_shared<SerialTransport>(config);
  auto servo = std::make_shared<policy_runtime::St3215Servo>(
      policy_runtime::St3215ServoConfig{
          "cancel", 1, 1, 4095, 0, 0, 250ms});
  policy_runtime::St3215Bus bus{
      transport, policy_runtime::St3215BusOptions{1ms}};
  policy_runtime::TransportScheduler scheduler;
  ASSERT_TRUE(bus.add_servo(servo).has_value());
  ASSERT_TRUE(bus.start(scheduler).has_value());

  // The executor has issued its present-position read and is blocked waiting
  // for a response. Stopping must interrupt that wait rather than inherit the
  // configured receive deadline.
  ASSERT_FALSE(endpoint.read_frame(200ms).empty());
  const auto started = std::chrono::steady_clock::now();
  bus.stop(scheduler);
  EXPECT_LT(std::chrono::steady_clock::now() - started, 50ms);
}

TEST(VirtualSerialTest,
     MissingGoalWriteAckDoesNotPreventPresentPositionFeedback) {
  PtyEndpoint endpoint;
  auto config = config_for(endpoint);
  config.read_timeout = 10ms;
  auto transport = std::make_shared<SerialTransport>(config);
  auto servo = std::make_shared<policy_runtime::St3215Servo>(
      policy_runtime::St3215ServoConfig{
          "optional-write-ack", 1, 1, 4095, 0, 0, 250ms, 250ms, 5ms});
  policy_runtime::St3215Bus bus{
      transport, policy_runtime::St3215BusOptions{1ms}};
  policy_runtime::TransportScheduler scheduler;
  ASSERT_TRUE(bus.add_servo(servo).has_value());

  const auto timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  ASSERT_EQ(servo->stage_command(policy_runtime::St3215ServoCommand{
                17U, timestamp_ns, 1.0, true, false}),
            policy_runtime::St3215CommandAcceptance::accepted);

  std::atomic<bool> saw_goal{};
  std::atomic<bool> saw_position_read{};
  std::jthread device([&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      const auto request = endpoint.read_frame(50ms);
      if (request.empty()) {
        continue;
      }
      auto decoded = St3215Protocol::decode_packet(request);
      if (!decoded.has_value()) {
        continue;
      }
      if (decoded.value().instruction_or_status == 0x03U) {
        saw_goal.store(true, std::memory_order_release);
        // A write status packet is optional on deployed ST3215 devices.
        continue;
      }
      if (decoded.value().instruction_or_status == 0x02U) {
        saw_position_read.store(true, std::memory_order_release);
        endpoint.write_chunks(St3215Protocol::status_packet(
            1, 0,
            std::array<std::byte, 2>{std::byte{0x34}, std::byte{0x02}}));
      }
    }
  });

  ASSERT_TRUE(bus.start(scheduler).has_value());
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline &&
         (servo->feedback().command_sequence != 17U ||
          (servo->feedback().flags & policy_runtime::kSt3215FeedbackValid) ==
              0U)) {
    std::this_thread::yield();
  }

  EXPECT_TRUE(saw_goal.load(std::memory_order_acquire));
  EXPECT_TRUE(saw_position_read.load(std::memory_order_acquire));
  EXPECT_EQ(servo->feedback().command_sequence, 17U);
  EXPECT_EQ(servo->feedback().raw_position, 0x0234U);
  EXPECT_NE(servo->feedback().flags & policy_runtime::kSt3215FeedbackValid,
            0U);
  bus.stop(scheduler);
  device.request_stop();
}

TEST(VirtualSerialTest, ExclusiveClaimRejectsASeparateProcess) {
  PtyEndpoint endpoint;
  SerialTransport owner{config_for(endpoint)};
  ASSERT_TRUE(owner.open().has_value());
  const auto child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    // Do not construct another SerialTransport here: after fork, its
    // process-local duplicate-open registry is copied from the parent and
    // would make this pass without exercising the kernel TIOCEXCL claim.
    const int contender = ::open(endpoint.path().c_str(),
                                 O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    const int open_error = errno;
    if (contender >= 0) {
      ::close(contender);
    }
    ::_exit(contender < 0 && open_error == EBUSY ? 0 : 1);
  }
  int status{};
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  owner.close();

  const auto after_close = ::fork();
  ASSERT_GE(after_close, 0);
  if (after_close == 0) {
    const int contender = ::open(endpoint.path().c_str(),
                                 O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (contender >= 0) {
      ::close(contender);
    }
    ::_exit(contender >= 0 ? 0 : 1);
  }
  ASSERT_EQ(::waitpid(after_close, &status, 0), after_close);
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
}

TEST(VirtualSerialTest, QuarantinesDelayedSameIdResponseAfterTimeout) {
  PtyEndpoint endpoint;
  auto config = config_for(endpoint);
  config.read_timeout = 30ms;
  SerialTransport transport{config};
  ASSERT_TRUE(transport.open().has_value());
  const auto request = St3215Protocol::read_present_position_command(1);
  ASSERT_TRUE(transport.write(0, request).has_value());
  std::thread first_device([&] {
    EXPECT_EQ(endpoint.read_frame(200ms), request);
  });
  transport.cycle({});
  first_device.join();
  EXPECT_EQ(transport.last_error(), policy_runtime::SerialError::timeout);

  ASSERT_TRUE(transport.write(0, request).has_value());
  const auto late = St3215Protocol::status_packet(
      1, 0, std::array<std::byte, 2>{std::byte{0x6f}, std::byte{0x00}});
  const auto current = St3215Protocol::status_packet(
      1, 0, std::array<std::byte, 2>{std::byte{0xde}, std::byte{0x00}});
  std::thread second_device([&] {
    std::this_thread::sleep_for(5ms);
    endpoint.write_chunks(late);
    EXPECT_EQ(endpoint.read_frame(200ms), request);
    endpoint.write_chunks(current);
  });
  transport.cycle({});
  second_device.join();

  std::array<std::byte, 259> response{};
  auto read = transport.read(0, response);
  ASSERT_TRUE(read.has_value()) << read.error().message;
  EXPECT_EQ(Bytes(response.begin(), response.begin() + read.value()), current);
  transport.close();
}

TEST(VirtualSerialTest, RestoresTermiosAndSurvivesNoiseBeforeAValidFrame) {
  PtyEndpoint endpoint;
  const int observer = ::open(endpoint.path().c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  ASSERT_GE(observer, 0);
  struct termios before {};
  ASSERT_EQ(::tcgetattr(observer, &before), 0);

  SerialTransport transport{config_for(endpoint)};
  ASSERT_TRUE(transport.open().has_value());
  ASSERT_TRUE(transport.write(0, St3215Protocol::ping_command(2)).has_value());
  std::thread device([&] {
    ASSERT_FALSE(endpoint.read_frame(200ms).empty());
    const std::array<std::byte, 3> noise{
        std::byte{0x00}, std::byte{0xff}, std::byte{0x01}};
    endpoint.write_chunks(noise);
    endpoint.write_chunks(St3215Protocol::status_packet(2, 0, {}), 2U);
  });
  transport.cycle({});
  device.join();

  std::array<std::byte, 259> response{};
  auto read = transport.read(0, response);
  ASSERT_TRUE(read.has_value()) << read.error().message;
  EXPECT_EQ(Bytes(response.begin(), response.begin() + read.value()),
            St3215Protocol::status_packet(2, 0, {}));
  transport.close();

  struct termios after {};
  ASSERT_EQ(::tcgetattr(observer, &after), 0);
  EXPECT_EQ(std::memcmp(&before, &after, sizeof(before)), 0);
  ::close(observer);
}

TEST(VirtualSerialTest, SerialTimeoutFeedsSafetyWithoutBlockingDaemonCycles) {
  PtyEndpoint endpoint;
  policy_runtime::profiles::RobotProfile profile;
  profile.axes.push_back(policy_runtime::profiles::AxisConfig{
      "axis", 0, 0, 0x9a, 0x00030924, 0x00010420,
      policy_runtime::profiles::Cia402Mode::csp, 1000.0, -10.0, 10.0,
      100ms, "arm", 1.0, 10.0});
  profile.st3215_servos.push_back(
      policy_runtime::profiles::St3215ServoProfile{
          "servo_position", "servo_target",
          policy_runtime::profiles::SerialPortConfig{
              endpoint.path(), 1'000'000U, 64U, 259U, 40ms, 40ms, 1ms},
          1, 1, 4095, 0, 0, 100ms, "arm"});

  policy_runtime::RobotIoDaemon daemon;
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.start().has_value());
  const auto timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  policy_runtime::Snapshot<policy_runtime::AxisCommand> axes;
  axes.sequence = 1;
  axes.timestamp_ns = timestamp_ns;
  axes.axis_count = 1;
  axes.axes[0] = policy_runtime::AxisCommand{
      1, timestamp_ns, 0.25, policy_runtime::kAxisCommandEnable, 0};
  ASSERT_EQ(daemon.stage_commands(axes),
            policy_runtime::CommandAcceptance::accepted);
  EXPECT_EQ(daemon.stage_servo_command(
                0, policy_runtime::St3215ServoCommand{
                       1, timestamp_ns, 0.5, true, false}),
            policy_runtime::St3215CommandAcceptance::accepted);

  // Consume the executor's request but deliberately do not answer. The serial
  // owner now waits on its bounded 40 ms deadline while the daemon cycle must
  // remain independent.
  ASSERT_FALSE(endpoint.read_frame(200ms).empty());
  const auto cycle_started = std::chrono::steady_clock::now();
  for (unsigned cycle = 0; cycle < 1000U; ++cycle) {
    daemon.cycle();
  }
  EXPECT_LT(std::chrono::steady_clock::now() - cycle_started, 30ms);

  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while ((daemon.servo_feedback(0).flags &
          policy_runtime::kSt3215FeedbackTimeout) == 0U &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  daemon.cycle();

  const auto serial_feedback = daemon.servo_feedback(0);
  EXPECT_NE(serial_feedback.flags & policy_runtime::kSt3215FeedbackTimeout,
            0U);
  EXPECT_NE(serial_feedback.flags & policy_runtime::kSt3215FeedbackStale,
            0U);
  // The serial fault entered the supervisor before evaluation. By the time
  // this asynchronous timeout is observed the latched stop may already have
  // confirmed zero velocity and advanced to its terminal disabled phase.
  EXPECT_EQ(daemon.axis_request(0), policy_runtime::AxisRequest::disable);
  EXPECT_NE(daemon.feedback(0).flags &
                policy_runtime::kAxisFeedbackSafetySerial,
            0U);
  const auto health = daemon.health();
  EXPECT_EQ(health.serial_servo_count, 1U);
  EXPECT_EQ(health.serial_fault_count, 1U);
  EXPECT_NE(health.serial_safety_flags, 0U);
  ASSERT_TRUE(daemon.request_stop().has_value());
}

TEST(VirtualSerialTest,
     ServoHeartbeatTimeoutLatchesSafetyUntilANewerCommandSequence) {
  PtyEndpoint endpoint;
  policy_runtime::profiles::RobotProfile profile;
  profile.axes.push_back(policy_runtime::profiles::AxisConfig{
      "axis", 0, 0, 0x9a, 0x00030924, 0x00010420,
      policy_runtime::profiles::Cia402Mode::csp, 1000.0, -10.0, 10.0,
      2s, "arm", 1.0, 10.0});
  profile.st3215_servos.push_back(
      policy_runtime::profiles::St3215ServoProfile{
          "servo_position", "servo_target",
          policy_runtime::profiles::SerialPortConfig{
              endpoint.path(), 1'000'000U, 64U, 259U, 10ms, 10ms, 1ms},
          1, 1, 4095, 0, 0, 250ms, "arm", 30ms, 5ms});

  policy_runtime::RobotIoDaemon daemon;
  ASSERT_TRUE(daemon.configure(profile).has_value());
  std::jthread device([&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      const auto request = endpoint.read_frame(20ms);
      if (request.empty()) {
        continue;
      }
      auto decoded = St3215Protocol::decode_packet(request);
      if (!decoded.has_value()) {
        continue;
      }
      if (decoded.value().instruction_or_status == 0x03U) {
        endpoint.write_chunks(St3215Protocol::status_packet(1, 0, {}));
      } else if (decoded.value().instruction_or_status == 0x02U) {
        endpoint.write_chunks(St3215Protocol::status_packet(
            1, 0,
            std::array<std::byte, 2>{std::byte{0x00}, std::byte{0x08}}));
      }
    }
  });
  ASSERT_TRUE(daemon.start().has_value());

  const auto initial_deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < initial_deadline &&
         (daemon.serial_safety_snapshot().fault_count != 0U ||
          (daemon.servo_feedback(0).flags &
           policy_runtime::kSt3215FeedbackValid) == 0U)) {
    daemon.cycle();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(daemon.serial_safety_snapshot().fault_count, 0U);

  const auto first_timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  policy_runtime::Snapshot<policy_runtime::AxisCommand> axes{};
  axes.sequence = 1U;
  axes.timestamp_ns = first_timestamp_ns;
  axes.axis_count = 1U;
  axes.axes[0] = policy_runtime::AxisCommand{
      1U, first_timestamp_ns, 0.25, policy_runtime::kAxisCommandEnable, 0U};
  const policy_runtime::St3215ServoCommand first_servo{
      1U, first_timestamp_ns, 0.5, true, false};
  ASSERT_EQ(daemon.stage_commands(axes),
            policy_runtime::CommandAcceptance::accepted);
  ASSERT_EQ(daemon.stage_servo_command(0, first_servo),
            policy_runtime::St3215CommandAcceptance::accepted);

  const auto enabled_deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < enabled_deadline &&
         (daemon.axis_request(0) != policy_runtime::AxisRequest::enable ||
          daemon.servo_feedback(0).command_sequence != 1U ||
          (daemon.servo_feedback(0).flags &
           policy_runtime::kSt3215FeedbackValid) == 0U)) {
    daemon.cycle();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(daemon.axis_request(0), policy_runtime::AxisRequest::enable);

  const auto timeout_deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < timeout_deadline &&
         (daemon.servo_feedback(0).flags &
          policy_runtime::kSt3215FeedbackTimeout) == 0U) {
    daemon.cycle();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_NE(daemon.servo_feedback(0).flags &
                policy_runtime::kSt3215FeedbackTimeout,
            0U);

  daemon.cycle();
  EXPECT_EQ(daemon.axis_request(0), policy_runtime::AxisRequest::quick_stop);
  EXPECT_NE(daemon.feedback(0).flags &
                policy_runtime::kAxisFeedbackSafetySerial,
            0U);
  daemon.cycle();
  EXPECT_EQ(daemon.axis_request(0), policy_runtime::AxisRequest::disable);
  EXPECT_EQ(daemon.serial_safety_snapshot().fault_count, 1U);

  EXPECT_NE(daemon.stage_servo_command(0, first_servo),
            policy_runtime::St3215CommandAcceptance::accepted);
  daemon.cycle();
  EXPECT_EQ(daemon.axis_request(0), policy_runtime::AxisRequest::disable);

  const auto second_timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  axes.sequence = 2U;
  axes.timestamp_ns = second_timestamp_ns;
  axes.axes[0].sequence = 2U;
  axes.axes[0].timestamp_ns = second_timestamp_ns;
  ASSERT_EQ(daemon.stage_commands(axes),
            policy_runtime::CommandAcceptance::accepted);
  ASSERT_EQ(daemon.stage_servo_command(
                0, policy_runtime::St3215ServoCommand{
                       2U, second_timestamp_ns, 0.75, true, false}),
            policy_runtime::St3215CommandAcceptance::accepted);

  const auto recovered_deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < recovered_deadline &&
         (daemon.axis_request(0) != policy_runtime::AxisRequest::enable ||
          daemon.servo_feedback(0).command_sequence != 2U ||
          daemon.serial_safety_snapshot().fault_count != 0U)) {
    daemon.cycle();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(daemon.servo_feedback(0).command_sequence, 2U);
  EXPECT_EQ(daemon.serial_safety_snapshot().fault_count, 0U);
  EXPECT_EQ(daemon.axis_request(0), policy_runtime::AxisRequest::enable);
  ASSERT_TRUE(daemon.request_stop().has_value());
  device.request_stop();
}

TEST(VirtualSerialTest, St3215OnlyDaemonRelaysV2IpcCommandsAndFeedback) {
  PtyEndpoint endpoint;
  std::array<int, 2> sockets{-1, -1};
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0,
                       sockets.data()),
            0);
  auto server = policy_runtime::RobotIoIpcServer::create(
      sockets[0], 0U, 1U, 93U);
  ASSERT_TRUE(server.has_value()) << server.error().message;

  policy_runtime::profiles::RobotProfile profile;
  profile.st3215_servos.push_back(
      policy_runtime::profiles::St3215ServoProfile{
          "servo_position", "servo_target",
          policy_runtime::profiles::SerialPortConfig{
              endpoint.path(), 1'000'000U, 64U, 259U, 20ms, 20ms, 1ms},
          1, 1, 4095, 0, 0, 250ms, "arm"});
  policy_runtime::RobotIoDaemon daemon;
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.attach_ipc(std::move(server.value())).has_value());
  auto client = policy_runtime::RobotIoClient::connect(sockets[1], 93U);
  ASSERT_TRUE(client.has_value()) << client.error().message;
  ASSERT_EQ(client.value().axis_count(), 0U);
  ASSERT_EQ(client.value().servo_count(), 1U);
  ASSERT_TRUE(daemon.start().has_value());

  std::jthread device([&](std::stop_token stop) {
    while (!stop.stop_requested()) {
      const auto request = endpoint.read_frame(20ms);
      if (request.empty()) {
        continue;
      }
      auto decoded = St3215Protocol::decode_packet(request);
      if (!decoded.has_value()) {
        continue;
      }
      if (decoded.value().instruction_or_status == 0x03U) {
        endpoint.write_chunks(St3215Protocol::status_packet(1, 0, {}));
      } else if (decoded.value().instruction_or_status == 0x02U) {
        endpoint.write_chunks(St3215Protocol::status_packet(
            1, 0, std::array<std::byte, 2>{std::byte{0x00}, std::byte{0x08}}));
      }
    }
  });

  const auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();
  const std::array command{policy_runtime::St3215ServoCommand{
      7U, timestamp_ns, std::numbers::pi, true, false}};
  ASSERT_TRUE(client.value().publish_servo_commands(command, 7U, timestamp_ns)
                  .has_value());

  const auto uncommitted_deadline = std::chrono::steady_clock::now() + 100ms;
  while (std::chrono::steady_clock::now() < uncommitted_deadline) {
    daemon.cycle();
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_NE(daemon.servo_feedback(0).command_sequence, 7U)
      << "a servo payload is not committed until its matching axis epoch";

  const std::array<policy_runtime::AxisCommand, 0> no_axes{};
  ASSERT_TRUE(client.value()
                  .publish_commands(no_axes, command, 7U, timestamp_ns)
                  .has_value());

  policy_runtime::St3215ServoFeedback observed{};
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    daemon.cycle();
    auto feedback = client.value().read_servo_feedback();
    if (feedback.has_value() && feedback.value().axis_count == 1U) {
      observed = feedback.value().axes[0];
      if (observed.command_sequence == 7U &&
          (observed.flags & policy_runtime::kSt3215FeedbackValid) != 0U) {
        break;
      }
    }
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_EQ(observed.command_sequence, 7U);
  EXPECT_EQ(observed.raw_position, 2048U);
  EXPECT_NE(observed.flags & policy_runtime::kSt3215FeedbackValid, 0U);
  ASSERT_TRUE(daemon.request_stop().has_value());
  daemon.run();
  device.request_stop();
}

TEST(VirtualSerialTest, SharedBusRecoversFromMalformedFrameAndConcurrentCommands) {
  PtyEndpoint endpoint;
  auto transport = std::make_shared<SerialTransport>(config_for(endpoint));
  auto first = std::make_shared<policy_runtime::St3215Servo>(
      policy_runtime::St3215ServoConfig{
          "first", 1, 1, 4095, 0, 0, 250ms});
  auto second = std::make_shared<policy_runtime::St3215Servo>(
      policy_runtime::St3215ServoConfig{
          "second", 2, 2, 4095, 0, 0, 250ms});
  policy_runtime::St3215Bus bus{
      transport, policy_runtime::St3215BusOptions{1ms}};
  policy_runtime::TransportScheduler scheduler;
  ASSERT_TRUE(bus.add_servo(first).has_value());
  ASSERT_TRUE(bus.add_servo(second).has_value());

  std::atomic<bool> device_ok{true};
  std::atomic<unsigned> frames_seen{};
  std::array<std::uint16_t, 3> positions{0, 100, 200};
  std::jthread device([&](std::stop_token stop) {
    bool inject_bad_checksum = true;
    while (!stop.stop_requested()) {
      const auto raw = endpoint.read_frame(50ms);
      if (raw.empty()) {
        continue;
      }
      auto decoded = St3215Protocol::decode_packet(raw);
      if (!decoded.has_value() || decoded.value().device_id > 2U) {
        device_ok.store(false, std::memory_order_release);
        continue;
      }
      ++frames_seen;
      const auto device_id = decoded.value().device_id;
      Bytes response;
      if (decoded.value().instruction_or_status == 0x03U &&
          decoded.value().parameters.size() >= 3U) {
        positions[device_id] = static_cast<std::uint16_t>(
            std::to_integer<std::uint8_t>(decoded.value().parameters[1]) |
            (std::to_integer<std::uint8_t>(decoded.value().parameters[2])
             << 8U));
        response = St3215Protocol::status_packet(device_id, 0, {});
      } else if (decoded.value().instruction_or_status == 0x02U) {
        const std::array<std::byte, 2> parameters{
            static_cast<std::byte>(positions[device_id] & 0xffU),
            static_cast<std::byte>((positions[device_id] >> 8U) & 0xffU)};
        response =
            St3215Protocol::status_packet(device_id, 0, parameters);
      } else {
        device_ok.store(false, std::memory_order_release);
        continue;
      }
      if (inject_bad_checksum) {
        auto malformed = response;
        malformed.back() ^= std::byte{0x01};
        endpoint.write_chunks(malformed, 1U);
        inject_bad_checksum = false;
      }
      endpoint.write_chunks(response, 1U);
    }
  });

  ASSERT_TRUE(bus.start(scheduler).has_value());
  const auto now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  std::thread first_writer([&] {
    EXPECT_EQ(first->stage_command(policy_runtime::St3215ServoCommand{
                  1, now_ns, std::numbers::pi, true, false}),
              policy_runtime::St3215CommandAcceptance::accepted);
  });
  std::thread second_writer([&] {
    EXPECT_EQ(second->stage_command(policy_runtime::St3215ServoCommand{
                  1, now_ns, std::numbers::pi / 2.0, true, false}),
              policy_runtime::St3215CommandAcceptance::accepted);
  });
  first_writer.join();
  second_writer.join();

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline &&
         (((first->feedback().flags & policy_runtime::kSt3215FeedbackValid) ==
           0U) ||
          ((second->feedback().flags & policy_runtime::kSt3215FeedbackValid) ==
           0U) ||
          first->feedback().command_sequence != 1U ||
          second->feedback().command_sequence != 1U)) {
    std::this_thread::yield();
  }

  EXPECT_TRUE(device_ok.load(std::memory_order_acquire));
  EXPECT_GE(frames_seen.load(std::memory_order_acquire), 4U);
  EXPECT_NE(first->feedback().flags & policy_runtime::kSt3215FeedbackValid,
            0U);
  EXPECT_NE(second->feedback().flags & policy_runtime::kSt3215FeedbackValid,
            0U);
  EXPECT_EQ(first->feedback().raw_position, 2048U);
  EXPECT_EQ(second->feedback().raw_position, 1024U);

  const auto stop_started = std::chrono::steady_clock::now();
  bus.stop(scheduler);
  EXPECT_LT(std::chrono::steady_clock::now() - stop_started, 250ms);
  device.request_stop();
}
