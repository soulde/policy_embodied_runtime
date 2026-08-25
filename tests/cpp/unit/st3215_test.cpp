#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/st3215/protocol.hpp"
#include "policy_runtime/robot/devices/st3215/servo.hpp"
#include "policy_runtime/robot_io/transport_scheduler.hpp"

namespace {

using namespace std::chrono_literals;
using policy_runtime::Bytes;
using policy_runtime::ErrorCode;
using policy_runtime::St3215Bus;
using policy_runtime::St3215BusOptions;
using policy_runtime::St3215CommandAcceptance;
using policy_runtime::St3215Protocol;
using policy_runtime::St3215Servo;
using policy_runtime::St3215ServoCommand;
using policy_runtime::St3215ServoConfig;

Bytes bytes(std::initializer_list<std::uint8_t> values) {
  Bytes output;
  output.reserve(values.size());
  for (const auto value : values) {
    output.push_back(static_cast<std::byte>(value));
  }
  return output;
}

class RecordingFrameTransport final : public policy_runtime::FrameTransport {
 public:
  policy_runtime::Result<void> open() override {
    ++open_calls;
    open_.store(true, std::memory_order_release);
    return policy_runtime::Result<void>::success();
  }

  void close() noexcept override {
    ++close_calls;
    open_.store(false, std::memory_order_release);
  }

  policy_runtime::TransportHealth health() const noexcept override {
    return open_.load(std::memory_order_acquire)
               ? policy_runtime::TransportHealth::healthy
               : policy_runtime::TransportHealth::failed;
  }

  policy_runtime::SchedulingClass scheduling_class() const noexcept override {
    return policy_runtime::SchedulingClass::blocking_event_driven;
  }

  policy_runtime::Result<void> write(
      policy_runtime::ChannelId, std::span<const std::byte> data) override {
    if (throw_on_write.load(std::memory_order_acquire)) {
      throw std::runtime_error("injected frame transport exception");
    }
    std::scoped_lock lock(mutex_);
    physical_threads.push_back(std::this_thread::get_id());
    pending.assign(data.begin(), data.end());
    writes.emplace_back(data.begin(), data.end());
    return policy_runtime::Result<void>::success();
  }

  policy_runtime::Result<std::size_t> read(
      policy_runtime::ChannelId, std::span<std::byte> buffer) override {
    std::scoped_lock lock(mutex_);
    physical_threads.push_back(std::this_thread::get_id());
    if (response.empty()) {
      return policy_runtime::Result<std::size_t>::failure(
          {ErrorCode::timeout, "no fake response"});
    }
    const auto count = std::min(buffer.size(), response.size());
    std::copy_n(response.begin(), count, buffer.begin());
    response.clear();
    return policy_runtime::Result<std::size_t>::success(count);
  }

  void cycle(const policy_runtime::CycleContext&) noexcept override {
    std::scoped_lock lock(mutex_);
    physical_threads.push_back(std::this_thread::get_id());
    if (pending.size() < 6U) {
      return;
    }
    const auto device_id = std::to_integer<std::uint8_t>(pending[2]);
    const auto instruction = std::to_integer<std::uint8_t>(pending[4]);
    if (instruction == 0x03U && pending.size() >= 9U) {
      positions[device_id] = static_cast<std::uint16_t>(
          std::to_integer<std::uint8_t>(pending[6]) |
          (std::to_integer<std::uint8_t>(pending[7]) << 8U));
      response = St3215Protocol::status_packet(
          device_id, status_error.load(std::memory_order_acquire), {});
    } else if (instruction == 0x02U) {
      const auto raw = positions[device_id];
      const std::array<std::byte, 2> parameters{
          static_cast<std::byte>(raw & 0xffU),
          static_cast<std::byte>((raw >> 8U) & 0xffU)};
      response = St3215Protocol::status_packet(device_id, 0U, parameters);
    }
    pending.clear();
  }

  std::vector<std::thread::id> physical_thread_ids() const {
    std::scoped_lock lock(mutex_);
    return physical_threads;
  }

  std::vector<Bytes> written_frames() const {
    std::scoped_lock lock(mutex_);
    return writes;
  }

  std::atomic<unsigned> open_calls{};
  std::atomic<unsigned> close_calls{};
  std::atomic<bool> throw_on_write{};
  std::atomic<std::uint8_t> status_error{};
  std::array<std::uint16_t, 254> positions{};

 private:
  mutable std::mutex mutex_;
  std::atomic<bool> open_{};
  Bytes pending;
  Bytes response;
  std::vector<Bytes> writes;
  std::vector<std::thread::id> physical_threads;
};

bool wait_for_feedback(St3215Servo& servo, std::uint64_t sequence) {
  const auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    if (servo.feedback().command_sequence == sequence &&
        (servo.feedback().flags & policy_runtime::kSt3215FeedbackValid) != 0U) {
      return true;
    }
    std::this_thread::yield();
  }
  return false;
}

}  // namespace

TEST(St3215Test, MatchesPythonCommandAndStatusGoldenVectors) {
  EXPECT_EQ(St3215Protocol::ping_command(1),
            bytes({0xff, 0xff, 0x01, 0x02, 0x01, 0xfb}));
  EXPECT_EQ(St3215Protocol::read_present_position_command(1),
            bytes({0xff, 0xff, 0x01, 0x04, 0x02, 0x38, 0x02, 0xbe}));
  EXPECT_EQ(St3215Protocol::goal_position_command(1, 2048, 0, 0),
            bytes({0xff, 0xff, 0x01, 0x09, 0x03, 0x2a, 0x00,
                   0x08, 0x00, 0x00, 0x00, 0x00, 0xc0}));
  EXPECT_EQ(St3215Protocol::goal_position_command(6, 4095, 5, 1000),
            bytes({0xff, 0xff, 0x06, 0x09, 0x03, 0x2a, 0xff,
                   0x0f, 0x05, 0x00, 0xe8, 0x03, 0xc5}));
  EXPECT_EQ(St3215Protocol::status_packet(
                1, 0, bytes({0x00, 0x08})),
            bytes({0xff, 0xff, 0x01, 0x04, 0x00, 0x00, 0x08, 0xf2}));
}

TEST(St3215Test, RejectsMalformedFramesAndParsesStatusPayload) {
  auto short_packet = St3215Protocol::decode_packet(bytes({0xff, 0xff, 1}));
  ASSERT_FALSE(short_packet.has_value());
  EXPECT_EQ(short_packet.error().code, ErrorCode::protocol);

  auto wrong_header = St3215Protocol::decode_packet(
      bytes({0xfe, 0xff, 0x01, 0x02, 0x01, 0xfb}));
  ASSERT_FALSE(wrong_header.has_value());
  EXPECT_EQ(wrong_header.error().code, ErrorCode::protocol);

  auto wrong_length = St3215Protocol::decode_packet(
      bytes({0xff, 0xff, 0x01, 0x03, 0x01, 0xfb}));
  ASSERT_FALSE(wrong_length.has_value());
  EXPECT_EQ(wrong_length.error().code, ErrorCode::protocol);

  auto bad_checksum = St3215Protocol::decode_packet(
      bytes({0xff, 0xff, 0x01, 0x04, 0x00, 0x00, 0x08, 0xf3}));
  ASSERT_FALSE(bad_checksum.has_value());
  EXPECT_EQ(bad_checksum.error().code, ErrorCode::protocol);

  auto decoded = St3215Protocol::decode_packet(
      bytes({0xff, 0xff, 0x01, 0x04, 0x02, 0x38, 0x02, 0xbe}));
  ASSERT_TRUE(decoded.has_value()) << decoded.error().message;
  EXPECT_EQ(decoded.value().device_id, 1U);
  EXPECT_EQ(decoded.value().instruction_or_status, 2U);
  EXPECT_EQ(decoded.value().parameters, bytes({0x38, 0x02}));

  auto status = St3215Protocol::parse_status(St3215Protocol::status_packet(
      1, 4, bytes({0x00, 0x08})));
  ASSERT_TRUE(status.has_value()) << status.error().message;
  EXPECT_EQ(status.value().device_id, 1U);
  EXPECT_EQ(status.value().error, 4U);
  EXPECT_EQ(status.value().parameters, bytes({0x00, 0x08}));
}

TEST(St3215Test, IncrementalParserResynchronizesAfterNoiseAndBadChecksum) {
  policy_runtime::St3215StreamParser parser;
  auto first = bytes({0x01, 0x02, 0xff, 0xff, 0x01, 0x04, 0x00});
  ASSERT_TRUE(parser.append(first).has_value());
  auto incomplete = parser.next();
  ASSERT_TRUE(incomplete.has_value());
  EXPECT_FALSE(incomplete.value().has_value());

  auto remainder = bytes({0x00, 0x08, 0xf3});
  const auto valid = St3215Protocol::status_packet(2, 0, bytes({0x34, 0x12}));
  remainder.insert(remainder.end(), valid.begin(), valid.end());
  ASSERT_TRUE(parser.append(remainder).has_value());

  auto malformed = parser.next();
  ASSERT_FALSE(malformed.has_value());
  EXPECT_EQ(malformed.error().code, ErrorCode::protocol);

  auto recovered = parser.next();
  ASSERT_TRUE(recovered.has_value()) << recovered.error().message;
  ASSERT_TRUE(recovered.value().has_value());
  EXPECT_EQ(recovered.value()->device_id, 2U);
  EXPECT_EQ(recovered.value()->parameters, bytes({0x34, 0x12}));
}

TEST(St3215Test, MatchesPythonModuloRoundingAndRadiansConversion) {
  auto half_turn = policy_runtime::radians_to_position_units(
      std::numbers::pi, 4095);
  ASSERT_TRUE(half_turn.has_value());
  EXPECT_EQ(half_turn.value(), 2048U);

  auto negative_half_turn = policy_runtime::radians_to_position_units(
      -std::numbers::pi, 4095);
  ASSERT_TRUE(negative_half_turn.has_value());
  EXPECT_EQ(negative_half_turn.value(), 2048U);

  auto full_turn = policy_runtime::radians_to_position_units(
      2.0 * std::numbers::pi, 4095);
  ASSERT_TRUE(full_turn.has_value());
  EXPECT_EQ(full_turn.value(), 0U);

  auto radians = policy_runtime::position_units_to_radians(2048, 4095);
  ASSERT_TRUE(radians.has_value());
  EXPECT_NEAR(radians.value(), 2048.0 / 4095.0 * 2.0 * std::numbers::pi,
              1e-12);

  EXPECT_FALSE(policy_runtime::radians_to_position_units(
                   std::numeric_limits<double>::infinity(), 4095)
                   .has_value());
  EXPECT_FALSE(policy_runtime::position_units_to_radians(1, 0).has_value());

  constexpr double tau = 2.0 * std::numbers::pi;
  ASSERT_TRUE(policy_runtime::radians_to_position_units(
                  tau * (0.5 / 4095.0), 4095)
                  .has_value());
  EXPECT_EQ(policy_runtime::radians_to_position_units(
                tau * (0.5 / 4095.0), 4095)
                .value(),
            0U);
  EXPECT_EQ(policy_runtime::radians_to_position_units(
                tau * (1.5 / 4095.0), 4095)
                .value(),
            2U);
  EXPECT_EQ(policy_runtime::radians_to_position_units(
                tau * (2.5 / 4095.0), 4095)
                .value(),
            2U);
}

TEST(St3215Test, SharedBusUsesOneNonRealtimeOwnerForAllPhysicalIo) {
  auto transport = std::make_shared<RecordingFrameTransport>();
  transport->positions[1] = 100;
  transport->positions[2] = 200;
  auto first = std::make_shared<St3215Servo>(St3215ServoConfig{
      "first", 1, 11, 4095, 0, 0, 250ms});
  auto second = std::make_shared<St3215Servo>(St3215ServoConfig{
      "second", 2, 12, 4095, 0, 0, 250ms});
  St3215Bus bus{transport, St3215BusOptions{1ms}};
  policy_runtime::TransportScheduler scheduler;
  ASSERT_TRUE(bus.add_servo(first).has_value());
  ASSERT_TRUE(bus.add_servo(second).has_value());
  ASSERT_TRUE(bus.start(scheduler).has_value());

  const auto caller = std::this_thread::get_id();
  EXPECT_EQ(first->stage_command(
                St3215ServoCommand{7, 100, std::numbers::pi, true, false}),
            St3215CommandAcceptance::accepted);
  EXPECT_EQ(second->stage_command(
                St3215ServoCommand{8, 101, std::numbers::pi / 2.0, true, false}),
            St3215CommandAcceptance::accepted);
  ASSERT_TRUE(wait_for_feedback(*first, 7));
  ASSERT_TRUE(wait_for_feedback(*second, 8));
  bus.stop(scheduler);

  EXPECT_EQ(transport->open_calls.load(), 1U);
  EXPECT_EQ(transport->close_calls.load(), 1U);
  const auto physical_threads = transport->physical_thread_ids();
  ASSERT_FALSE(physical_threads.empty());
  for (const auto thread : physical_threads) {
    EXPECT_NE(thread, caller);
    EXPECT_EQ(thread, physical_threads.front());
  }
  const auto frames = transport->written_frames();
  ASSERT_GE(frames.size(), 4U);
  for (const auto& frame : frames) {
    EXPECT_TRUE(St3215Protocol::decode_packet(frame).has_value());
  }
  EXPECT_FALSE(scheduler.executor_id(*transport).has_value());
}

TEST(St3215Test, RejectsStaleAndFutureCommandsAndStopsResendingExpiredEnable) {
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
  auto transport = std::make_shared<RecordingFrameTransport>();
  auto servo = std::make_shared<St3215Servo>(St3215ServoConfig{
      "freshness", 1, 1, 4095, 0, 0, 250ms, 20ms, 5ms});
  EXPECT_EQ(servo->stage_command(
                St3215ServoCommand{1, now_ns - 21'000'000, 0.5, true, false}),
            St3215CommandAcceptance::rejected);
  EXPECT_EQ(servo->stage_command(
                St3215ServoCommand{1, now_ns + 6'000'000, 0.5, true, false}),
            St3215CommandAcceptance::rejected);

  St3215Bus bus{transport, St3215BusOptions{1ms}};
  policy_runtime::TransportScheduler scheduler;
  ASSERT_TRUE(bus.add_servo(servo).has_value());
  ASSERT_TRUE(bus.start(scheduler).has_value());
  const auto fresh_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();
  ASSERT_EQ(servo->stage_command(
                St3215ServoCommand{2, fresh_ns, 0.5, true, false}),
            St3215CommandAcceptance::accepted);
  ASSERT_TRUE(wait_for_feedback(*servo, 2));
  std::this_thread::sleep_for(40ms);
  const auto first = transport->written_frames();
  const auto goals_before = std::count_if(first.begin(), first.end(), [](const auto& frame) {
    return frame.size() > 4U && std::to_integer<std::uint8_t>(frame[4]) == 0x03U;
  });
  std::this_thread::sleep_for(20ms);
  const auto second = transport->written_frames();
  const auto goals_after = std::count_if(second.begin(), second.end(), [](const auto& frame) {
    return frame.size() > 4U && std::to_integer<std::uint8_t>(frame[4]) == 0x03U;
  });
  EXPECT_GT(goals_before, 0);
  EXPECT_EQ(goals_after, goals_before);
  EXPECT_NE(servo->feedback().flags & policy_runtime::kSt3215FeedbackDisabled,
            0U);
  bus.stop(scheduler);
}

TEST(St3215Test, PublishesDeviceErrorBeforeSuccessShapeValidation) {
  auto transport = std::make_shared<RecordingFrameTransport>();
  transport->status_error.store(4U, std::memory_order_release);
  auto servo = std::make_shared<St3215Servo>(St3215ServoConfig{
      "device-error", 1, 1, 4095, 0, 0, 250ms});
  St3215Bus bus{transport, St3215BusOptions{1ms}};
  policy_runtime::TransportScheduler scheduler;
  ASSERT_TRUE(bus.add_servo(servo).has_value());
  ASSERT_TRUE(bus.start(scheduler).has_value());
  ASSERT_EQ(servo->stage_command(
                St3215ServoCommand{1, 1, 0.5, true, false}),
            St3215CommandAcceptance::accepted);
  const auto deadline = std::chrono::steady_clock::now() + 200ms;
  while ((servo->feedback().flags & policy_runtime::kSt3215FeedbackDeviceError) ==
             0U &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  EXPECT_EQ(servo->feedback().status_error, 4U);
  EXPECT_NE(servo->feedback().flags & policy_runtime::kSt3215FeedbackDeviceError,
            0U);
  bus.stop(scheduler);
}

TEST(St3215Test, RegistryFreezesMultipleServosOntoOneConfiguredPort) {
  const policy_runtime::profiles::SerialPortConfig serial{
      "/tmp/registry-only-st3215", 1'000'000U, 64U, 259U,
      20ms, 20ms, 1ms};
  std::array<policy_runtime::profiles::St3215ServoProfile, 2> profiles{
      policy_runtime::profiles::St3215ServoProfile{
          "joint_1_position", "joint_1_target", serial, 1, 1, 4095,
          0, 0, 250ms, "arm"},
      policy_runtime::profiles::St3215ServoProfile{
          "joint_2_position", "joint_2_target", serial, 2, 2, 4095,
          0, 0, 250ms, "arm"}};
  policy_runtime::St3215DeviceRegistry registry;

  ASSERT_TRUE(registry.configure(profiles).has_value());
  EXPECT_EQ(registry.bus_count(), 1U);
  EXPECT_EQ(registry.servo_count(), 2U);
  ASSERT_NE(registry.servo(0), nullptr);
  ASSERT_NE(registry.servo(1), nullptr);
  EXPECT_NE(registry.servo(0), registry.servo(1));
  EXPECT_TRUE(registry.atomics_are_lock_free());

  auto duplicate_configure = registry.configure(profiles);
  ASSERT_FALSE(duplicate_configure.has_value());
  EXPECT_EQ(duplicate_configure.error().code, ErrorCode::invalid_argument);
}

TEST(St3215Test, ExecutorContainsTransportExceptionsAndPublishesFailure) {
  auto transport = std::make_shared<RecordingFrameTransport>();
  transport->throw_on_write.store(true, std::memory_order_release);
  auto servo = std::make_shared<St3215Servo>(St3215ServoConfig{
      "servo", 1, 1, 4095, 0, 0, 250ms});
  St3215Bus bus{transport, St3215BusOptions{1ms}};
  policy_runtime::TransportScheduler scheduler;
  ASSERT_TRUE(bus.add_servo(servo).has_value());
  ASSERT_TRUE(bus.start(scheduler).has_value());
  ASSERT_EQ(servo->stage_command(
                St3215ServoCommand{1, 1, 0.5, true, false}),
            St3215CommandAcceptance::accepted);

  const auto deadline = std::chrono::steady_clock::now() + 250ms;
  while ((servo->feedback().flags & policy_runtime::kSt3215FeedbackIo) == 0U &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }

  EXPECT_TRUE(bus.running());
  EXPECT_NE(servo->feedback().flags & policy_runtime::kSt3215FeedbackIo, 0U);
  EXPECT_NE(servo->feedback().flags & policy_runtime::kSt3215FeedbackStale,
            0U);
  bus.stop(scheduler);
}

TEST(St3215Test, StopWakesAOneSecondServicePeriodWithoutWaitingForDeadline) {
  auto transport = std::make_shared<RecordingFrameTransport>();
  auto servo = std::make_shared<St3215Servo>(St3215ServoConfig{
      "bounded-stop", 1, 1, 4095, 0, 0, 250ms});
  St3215Bus bus{transport, St3215BusOptions{1s}};
  policy_runtime::TransportScheduler scheduler;
  ASSERT_TRUE(bus.add_servo(servo).has_value());
  ASSERT_TRUE(bus.start(scheduler).has_value());
  std::this_thread::sleep_for(5ms);
  const auto started = std::chrono::steady_clock::now();
  bus.stop(scheduler);
  EXPECT_LT(std::chrono::steady_clock::now() - started, 100ms);
}
