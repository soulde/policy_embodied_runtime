#include <array>
#include <atomic>
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
#include <unistd.h>

#include <gtest/gtest.h>

#include "policy_runtime/protocol/st3215/protocol.hpp"
#include "policy_runtime/robot_io/daemon.hpp"
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
  EXPECT_EQ(daemon.axis_request(0), policy_runtime::AxisRequest::quick_stop);
  EXPECT_NE(daemon.feedback(0).flags &
                policy_runtime::kAxisFeedbackSafetySerial,
            0U);
  const auto health = daemon.health();
  EXPECT_EQ(health.serial_servo_count, 1U);
  EXPECT_EQ(health.serial_fault_count, 1U);
  EXPECT_NE(health.serial_safety_flags, 0U);
  ASSERT_TRUE(daemon.request_stop().has_value());
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
