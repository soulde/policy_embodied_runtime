#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <sys/socket.h>
#include <unistd.h>

#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/cia402/pdo.hpp"
#include "policy_runtime/protocol/cia402/state_machine.hpp"
#include "policy_runtime/robot/devices/cia402/axis.hpp"
#include "policy_runtime/robot_io/daemon.hpp"
#include "policy_runtime/robot_io/ipc_server.hpp"
#include "policy_runtime/robot_io/realtime_loop.hpp"
#include "policy_runtime/runtime/robot_io_client.hpp"
#include "policy_runtime/transport/ethercat/backend.hpp"
#include "policy_runtime/transport/ethercat/elmo_gold.hpp"
#include "policy_runtime/transport/ethercat/master.hpp"

namespace allocation_probe {

std::atomic<bool> enabled{};
std::atomic<std::size_t> count{};

void record() noexcept {
  if (enabled.load(std::memory_order_relaxed)) {
    count.fetch_add(1U, std::memory_order_relaxed);
  }
}

void begin() noexcept {
  count.store(0U, std::memory_order_relaxed);
  enabled.store(true, std::memory_order_release);
}

std::size_t end() noexcept {
  enabled.store(false, std::memory_order_release);
  return count.load(std::memory_order_relaxed);
}

}  // namespace allocation_probe

void* operator new(std::size_t size) {
  allocation_probe::record();
  if (auto* allocation = std::malloc(size == 0U ? 1U : size)) {
    return allocation;
  }
  throw std::bad_alloc{};
}

void* operator new[](std::size_t size) { return ::operator new(size); }
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
  try {
    return ::operator new(size);
  } catch (...) {
    return nullptr;
  }
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
  return ::operator new(size, std::nothrow);
}
void* operator new(std::size_t size, std::align_val_t alignment) {
  allocation_probe::record();
  void* allocation{};
  const auto aligned = static_cast<std::size_t>(alignment);
  if (posix_memalign(&allocation, aligned, size == 0U ? aligned : size) != 0) {
    throw std::bad_alloc{};
  }
  return allocation;
}
void* operator new[](std::size_t size, std::align_val_t alignment) {
  return ::operator new(size, alignment);
}
void* operator new(std::size_t size, std::align_val_t alignment,
                   const std::nothrow_t&) noexcept {
  try {
    return ::operator new(size, alignment);
  } catch (...) {
    return nullptr;
  }
}
void* operator new[](std::size_t size, std::align_val_t alignment,
                     const std::nothrow_t&) noexcept {
  return ::operator new(size, alignment, std::nothrow);
}
void operator delete(void* allocation) noexcept { std::free(allocation); }
void operator delete[](void* allocation) noexcept { std::free(allocation); }
void operator delete(void* allocation, std::size_t) noexcept { std::free(allocation); }
void operator delete[](void* allocation, std::size_t) noexcept {
  std::free(allocation);
}
void operator delete(void* allocation, const std::nothrow_t&) noexcept {
  std::free(allocation);
}
void operator delete[](void* allocation, const std::nothrow_t&) noexcept {
  std::free(allocation);
}
void operator delete(void* allocation, std::align_val_t) noexcept {
  std::free(allocation);
}
void operator delete[](void* allocation, std::align_val_t) noexcept {
  std::free(allocation);
}
void operator delete(void* allocation, std::size_t, std::align_val_t) noexcept {
  std::free(allocation);
}
void operator delete[](void* allocation, std::size_t, std::align_val_t) noexcept {
  std::free(allocation);
}
void operator delete(void* allocation, std::align_val_t,
                     const std::nothrow_t&) noexcept {
  std::free(allocation);
}
void operator delete[](void* allocation, std::align_val_t,
                       const std::nothrow_t&) noexcept {
  std::free(allocation);
}

namespace {

using namespace std::chrono_literals;
using policy_runtime::AxisCommand;
using policy_runtime::AxisRequest;
using policy_runtime::Cia402PdoView;
using policy_runtime::CommandAcceptance;
using policy_runtime::DaemonSignalLatch;
using policy_runtime::DomainHealth;
using policy_runtime::ElmoGoldDeviceDescription;
using policy_runtime::LinuxRealtimeSystem;
using policy_runtime::MonotonicClock;
using policy_runtime::RealtimeLoop;
using policy_runtime::RealtimeLoopConfig;
using policy_runtime::RealtimeSystem;
using policy_runtime::RobotIoClient;
using policy_runtime::RobotIoDaemon;
using policy_runtime::RobotIoIpcServer;
using policy_runtime::SafetyOptions;
using policy_runtime::Snapshot;
using policy_runtime::kAxisCommandEnable;
using policy_runtime::kAxisCommandFaultReset;
using policy_runtime::kAxisFeedbackSafetyFollowingError;
using policy_runtime::kAxisFeedbackSafetyDriveFault;
using policy_runtime::kAxisFeedbackSafetyGroup;
using policy_runtime::kAxisFeedbackSafetyModeMismatch;
using policy_runtime::kAxisFeedbackSafetyWkc;
using policy_runtime::profiles::AxisConfig;
using policy_runtime::profiles::Cia402Mode;
using policy_runtime::profiles::RobotProfile;
using policy_runtime::EthercatAxisConfiguration;
using policy_runtime::EthercatBackend;
using policy_runtime::EthercatMaster;
using policy_runtime::EthercatSlaveAddress;
using policy_runtime::EthercatSlaveConfiguration;
using policy_runtime::ObjectAddress;
using policy_runtime::PdoFieldLocation;
using policy_runtime::Result;
using policy_runtime::SdoDownloadRequest;
using policy_runtime::SdoTransferProgress;
using policy_runtime::sdo_completed;

class FakeClock final : public MonotonicClock {
 public:
  std::int64_t now_ns() const noexcept override {
    return now_ns_.load(std::memory_order_acquire);
  }

  int sleep_until_ns(std::int64_t absolute_ns) noexcept override {
    sleep_calls_.fetch_add(1U, std::memory_order_acq_rel);
    last_deadline_ns_.store(absolute_ns, std::memory_order_release);
    const auto error_index = next_sleep_error_.fetch_add(1U);
    if (error_index < sleep_errors_.size() && sleep_errors_[error_index] != 0) {
      return sleep_errors_[error_index];
    }
    auto now = now_ns_.load(std::memory_order_acquire);
    const auto wake = absolute_ns + wake_delay_ns_.load(std::memory_order_acquire);
    while (wake > now && !now_ns_.compare_exchange_weak(
                             now, wake, std::memory_order_acq_rel,
                             std::memory_order_acquire)) {
    }
    return 0;
  }

  void advance(std::chrono::nanoseconds duration) noexcept {
    now_ns_.fetch_add(duration.count(), std::memory_order_acq_rel);
  }

  void set_sleep_errors(std::vector<int> errors) {
    sleep_errors_ = std::move(errors);
    next_sleep_error_.store(0U, std::memory_order_release);
  }
  void set_wake_delay(std::chrono::nanoseconds delay) noexcept {
    wake_delay_ns_.store(delay.count(), std::memory_order_release);
  }

  unsigned sleep_calls() const noexcept {
    return sleep_calls_.load(std::memory_order_acquire);
  }
  std::int64_t last_deadline_ns() const noexcept {
    return last_deadline_ns_.load(std::memory_order_acquire);
  }

 private:
  std::atomic<std::int64_t> now_ns_{};
  std::atomic<std::int64_t> last_deadline_ns_{};
  std::atomic<std::int64_t> wake_delay_ns_{};
  std::atomic<unsigned> sleep_calls_{};
  std::vector<int> sleep_errors_;
  std::atomic<std::size_t> next_sleep_error_{};
};

class FakeRealtimeSystem final : public RealtimeSystem {
 public:
  bool capture_current_thread_state() noexcept override {
    ++capture_calls;
    setup_thread = std::this_thread::get_id();
    return capture_ok;
  }
  bool preempt_rt_kernel() const noexcept override { return preempt_rt; }
  bool lock_process_memory() noexcept override {
    ++lock_calls;
    return lock_ok;
  }
  bool pin_current_thread(int cpu) noexcept override {
    ++pin_calls;
    pinned_cpu = cpu;
    return pin_ok;
  }
  bool set_current_thread_fifo(int priority) noexcept override {
    ++fifo_calls;
    fifo_priority = priority;
    return fifo_ok;
  }
  void restore_current_thread_state() noexcept override {
    ++restore_calls;
    restore_thread = std::this_thread::get_id();
  }
  void unlock_process_memory() noexcept override { ++unlock_calls; }

  bool capture_ok{true};
  bool preempt_rt{true};
  bool lock_ok{true};
  bool pin_ok{true};
  bool fifo_ok{true};
  unsigned capture_calls{};
  unsigned lock_calls{};
  unsigned pin_calls{};
  unsigned fifo_calls{};
  unsigned restore_calls{};
  unsigned unlock_calls{};
  int pinned_cpu{-1};
  int fifo_priority{};
  std::thread::id setup_thread;
  std::thread::id restore_thread;
};

class FakeEthercatBackend final : public EthercatBackend {
 public:
  void set_health(DomainHealth health) noexcept { health_ = health; }
  std::span<std::byte> image() noexcept { return image_; }
  void fail_next_activation() noexcept { fail_activation_ = true; }
  void observe_control_word(std::size_t byte_offset) noexcept {
    control_word_offset_ = byte_offset;
  }
  std::uint16_t last_sent_control_word() const noexcept {
    return last_sent_control_word_.load(std::memory_order_acquire);
  }
  int final_disable_send_order() const noexcept {
    return final_disable_send_order_.load(std::memory_order_acquire);
  }
  int deactivate_order() const noexcept {
    return deactivate_order_.load(std::memory_order_acquire);
  }

 protected:
  Result<void> initialize() override {
    next_offset_ = 0U;
    return Result<void>::success();
  }
  Result<void> configure_slave(const EthercatSlaveConfiguration&) override {
    return Result<void>::success();
  }
  Result<PdoFieldLocation> bind_pdo_entry(EthercatSlaveAddress, ObjectAddress,
                                           std::uint8_t bit_length) override {
    const PdoFieldLocation location{next_offset_, 0U, bit_length};
    next_offset_ += (bit_length + 7U) / 8U;
    return Result<PdoFieldLocation>::success(location);
  }
  Result<void> activate() override {
    if (fail_activation_) {
      fail_activation_ = false;
      return Result<void>::failure(
          {policy_runtime::ErrorCode::io, "injected activation failure"});
    }
    active_ = true;
    return Result<void>::success();
  }
  void deactivate() noexcept override {
    active_ = false;
    deactivate_order_.store(event_order_.fetch_add(1) + 1,
                            std::memory_order_release);
  }
  void receive() noexcept override {}
  void process_domain() noexcept override {}
  std::span<std::byte> process_image() noexcept override { return image_; }
  void queue_domain() noexcept override {}
  void send() noexcept override {
    if (!control_word_offset_.has_value() ||
        *control_word_offset_ + sizeof(std::uint16_t) > image_.size()) {
      return;
    }
    const auto offset = *control_word_offset_;
    const auto control_word = static_cast<std::uint16_t>(
        std::to_integer<unsigned char>(image_[offset]) |
        (std::to_integer<unsigned char>(image_[offset + 1U]) << 8U));
    last_sent_control_word_.store(control_word, std::memory_order_release);
    const auto order = event_order_.fetch_add(1) + 1;
    if (control_word == 0U) {
      final_disable_send_order_.store(order, std::memory_order_release);
    }
  }
  DomainHealth domain_health() const noexcept override {
    return active_ ? health_ : DomainHealth{};
  }
  SdoTransferProgress progress_download_sdo(
      EthercatSlaveAddress, const SdoDownloadRequest&) override {
    return sdo_completed();
  }
  SdoTransferProgress progress_upload_sdo(
      EthercatSlaveAddress, ObjectAddress,
      std::span<std::byte>) override {
    return sdo_completed();
  }
  SdoTransferProgress progress_cancel_sdo(EthercatSlaveAddress) override {
    return sdo_completed();
  }

 private:
  std::array<std::byte, 512> image_{};
  std::size_t next_offset_{};
  bool active_{};
  bool fail_activation_{};
  std::optional<std::size_t> control_word_offset_;
  std::atomic<std::uint16_t> last_sent_control_word_{};
  std::atomic<int> event_order_{};
  std::atomic<int> final_disable_send_order_{};
  std::atomic<int> deactivate_order_{};
  DomainHealth health_{1U, 1U, true, true, true, 0, 1U};
};

AxisConfig axis_config(std::string name = "axis", std::string group = "arm",
                       std::chrono::milliseconds timeout = 20ms) {
  return AxisConfig{std::move(name), 0U, 0U,
                    ElmoGoldDeviceDescription::vendor_id(),
                    ElmoGoldDeviceDescription::product_code(),
                    ElmoGoldDeviceDescription::revision(), Cia402Mode::csp,
                    1000.0, -2.0, 2.0, timeout, std::move(group), 0.1, 0.5};
}

RobotProfile profile_with_axes(std::size_t count, bool shared_group = false) {
  RobotProfile profile;
  profile.axes.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    auto config = axis_config("axis_" + std::to_string(index),
                              shared_group ? "arm" : "axis_" + std::to_string(index));
    config.position = static_cast<std::uint16_t>(index);
    profile.axes.push_back(std::move(config));
  }
  return profile;
}

Snapshot<AxisCommand> commands(std::size_t count, std::uint64_t sequence,
                               std::int64_t timestamp_ns, double target = 0.0,
                               std::uint32_t flags = kAxisCommandEnable) {
  Snapshot<AxisCommand> snapshot{};
  snapshot.axis_count = static_cast<std::uint32_t>(count);
  snapshot.sequence = sequence;
  snapshot.timestamp_ns = timestamp_ns;
  for (std::size_t axis = 0; axis < count; ++axis) {
    snapshot.axes[axis].sequence = sequence;
    snapshot.axes[axis].timestamp_ns = timestamp_ns;
    snapshot.axes[axis].target = target;
    snapshot.axes[axis].flags = flags;
  }
  return snapshot;
}

DomainHealth healthy_domain(std::size_t axis_count) {
  const auto mask = static_cast<std::uint16_t>((1U << axis_count) - 1U);
  return DomainHealth{1U, 1U, true, true, true, 0, mask};
}

std::vector<Cia402PdoView> enabled_pdos(std::size_t count) {
  std::vector<Cia402PdoView> pdos(count);
  for (auto& pdo : pdos) {
    pdo.status_word = 0x0027U;
    pdo.mode_display = static_cast<std::int8_t>(Cia402Mode::csp);
  }
  return pdos;
}

class DaemonHarness {
 public:
  explicit DaemonHarness(RobotProfile profile, SafetyOptions options = {})
      : daemon(clock, realtime, options), profile_(std::move(profile)) {
    EXPECT_TRUE(daemon.configure(profile_).has_value());
    EXPECT_TRUE(daemon.start().has_value());
    pdos = enabled_pdos(profile_.axes.size());
  }

  FakeClock clock;
  FakeRealtimeSystem realtime;
  RobotIoDaemon daemon;
  RobotProfile profile_;
  std::vector<Cia402PdoView> pdos;
};

TEST(RealtimeLoopTest, UsesAbsoluteMonotonicDeadlinesAndReportsSetup) {
  FakeClock clock;
  FakeRealtimeSystem system;
  RealtimeLoop loop{clock, system, RealtimeLoopConfig{1ms, 3, 91}};

  auto setup = loop.prepare();
  ASSERT_TRUE(setup.has_value());
  EXPECT_TRUE(setup.value().realtime_guarantee);
  EXPECT_EQ(system.pinned_cpu, 3);
  EXPECT_EQ(system.fifo_priority, 91);

  const auto first = loop.wait_next();
  const auto second = loop.wait_next();
  ASSERT_EQ(first.sleep_error, 0);
  ASSERT_EQ(second.sleep_error, 0);
  EXPECT_EQ(first.context.sequence, 1U);
  EXPECT_EQ(second.context.sequence, 2U);
  EXPECT_EQ(clock.sleep_calls(), 2U);
  EXPECT_EQ(clock.last_deadline_ns(), 2'000'000);
  EXPECT_EQ(second.context.period, 1ms);
  loop.release();
  EXPECT_EQ(system.restore_calls, 1U);
  EXPECT_EQ(system.unlock_calls, 1U);
}

TEST(RealtimeLoopTest, ResynchronizesToFirstFutureReleaseAfterLongOverrun) {
  FakeClock clock;
  FakeRealtimeSystem system;
  RealtimeLoop loop{clock, system, RealtimeLoopConfig{1ms, 0, 80}};
  ASSERT_TRUE(loop.prepare().has_value());
  ASSERT_EQ(loop.wait_next().sleep_error, 0);

  clock.advance(5ms);
  const auto next = loop.wait_next();

  ASSERT_EQ(next.sleep_error, 0);
  EXPECT_EQ(std::chrono::duration_cast<std::chrono::nanoseconds>(
                next.context.scheduled_start.time_since_epoch())
                .count(),
            7'000'000);
  EXPECT_EQ(clock.last_deadline_ns(), 7'000'000);
  EXPECT_EQ(loop.metrics().skipped_releases, 5U);
}

TEST(RealtimeLoopTest, RetriesEintrAndReportsPersistentSleepFailure) {
  FakeClock clock;
  FakeRealtimeSystem system;
  RealtimeLoop loop{clock, system, RealtimeLoopConfig{1ms, 0, 80}};
  ASSERT_TRUE(loop.prepare().has_value());
  clock.set_sleep_errors({EINTR, EINTR, EIO});

  const auto result = loop.wait_next();

  EXPECT_EQ(result.sleep_error, EIO);
  EXPECT_EQ(clock.sleep_calls(), 3U);
  EXPECT_EQ(loop.metrics().sleep_failures, 1U);
}

TEST(RealtimeLoopTest, SeparatesWakeLatencyFromCycleExecutionTime) {
  FakeClock clock;
  FakeRealtimeSystem system;
  RealtimeLoop loop{clock, system, RealtimeLoopConfig{1ms, 0, 80}};
  ASSERT_TRUE(loop.prepare().has_value());
  clock.set_wake_delay(100us);
  const auto release = loop.wait_next();
  ASSERT_EQ(release.sleep_error, 0);

  clock.advance(500us);
  loop.observe_finish(clock.now_ns());
  const auto metrics = loop.metrics();

  EXPECT_EQ(metrics.maximum_wake_latency_ns, 100'000);
  EXPECT_EQ(metrics.maximum_execution_time_ns, 500'000);
}

TEST(RobotIoDaemonTest, QuickStopsThenDisablesOnExpiredCommand) {
  DaemonHarness harness{profile_with_axes(1)};
  ASSERT_EQ(harness.daemon.stage_commands(commands(1, 1U, 0)),
            CommandAcceptance::accepted);

  harness.pdos[0].actual_velocity = 100;
  harness.clock.advance(21ms);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);

  harness.pdos[0].actual_velocity = 0;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::disable);
}

TEST(RobotIoDaemonTest, AppliesConsecutiveWkcThresholdInTheCurrentCycle) {
  SafetyOptions options{};
  options.consecutive_wkc_limit = 3U;
  DaemonHarness harness{profile_with_axes(1), options};
  ASSERT_EQ(harness.daemon.stage_commands(commands(1, 1U, 0)),
            CommandAcceptance::accepted);
  harness.pdos[0].actual_velocity = 100;
  auto unhealthy = healthy_domain(1);
  unhealthy.working_counter = 0U;
  unhealthy.working_counter_complete = false;

  harness.daemon.process_device_cycle(harness.pdos, unhealthy, false);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::enable);
  harness.daemon.process_device_cycle(harness.pdos, unhealthy, false);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::enable);
  harness.daemon.process_device_cycle(harness.pdos, unhealthy, false);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);
  EXPECT_NE(harness.daemon.feedback(0).flags & kAxisFeedbackSafetyWkc, 0U);
}

TEST(RobotIoDaemonTest, PropagatesSlaveLossAcrossItsSafetyGroup) {
  DaemonHarness harness{profile_with_axes(2, true)};
  ASSERT_EQ(harness.daemon.stage_commands(commands(2, 1U, 0)),
            CommandAcceptance::accepted);
  harness.pdos[0].actual_velocity = 100;
  harness.pdos[1].actual_velocity = 100;
  auto health = healthy_domain(2);
  health.all_slaves_operational = false;
  health.operational_axes_mask = 0x0001U;

  harness.daemon.process_device_cycle(harness.pdos, health, true);

  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);
  EXPECT_EQ(harness.daemon.axis_request(1), AxisRequest::quick_stop);
  EXPECT_NE(harness.daemon.feedback(0).flags & kAxisFeedbackSafetyGroup, 0U);
}

TEST(RobotIoDaemonTest, BlocksAGroupOnModeMismatchAndFollowingError) {
  DaemonHarness harness{profile_with_axes(2, true)};
  ASSERT_EQ(harness.daemon.stage_commands(commands(2, 1U, 0, 1.0)),
            CommandAcceptance::accepted);
  harness.pdos[0].mode_display = static_cast<std::int8_t>(Cia402Mode::csv);
  harness.pdos[0].actual_velocity = 100;
  harness.pdos[1].actual_velocity = 100;

  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(2), true);

  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);
  EXPECT_EQ(harness.daemon.axis_request(1), AxisRequest::quick_stop);
  EXPECT_NE(harness.daemon.feedback(0).flags & kAxisFeedbackSafetyModeMismatch, 0U);
  EXPECT_NE(harness.daemon.feedback(1).flags & kAxisFeedbackSafetyGroup, 0U);

  auto following_profile = profile_with_axes(1);
  following_profile.axes[0].following_error_limit = 0.05;
  DaemonHarness following{std::move(following_profile)};
  ASSERT_EQ(following.daemon.stage_commands(commands(1, 1U, 0, 1.0)),
            CommandAcceptance::accepted);
  following.pdos[0].actual_velocity = 100;
  following.daemon.process_device_cycle(following.pdos, healthy_domain(1), true);
  EXPECT_EQ(following.daemon.axis_request(0), AxisRequest::quick_stop);
  EXPECT_NE(following.daemon.feedback(0).flags & kAxisFeedbackSafetyFollowingError,
            0U);
}

TEST(RobotIoDaemonTest, SlewLimitedTargetDoesNotFalseTripFollowingError) {
  DaemonHarness harness{profile_with_axes(1)};
  ASSERT_EQ(harness.daemon.stage_commands(commands(1, 1U, 0, 1.0)),
            CommandAcceptance::accepted);

  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);

  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::enable);
  EXPECT_EQ(harness.pdos[0].target_position, 100);
  EXPECT_EQ(harness.daemon.feedback(0).flags &
                kAxisFeedbackSafetyFollowingError,
            0U);
}

TEST(RobotIoDaemonTest, PropagatesDriveFaultAcrossItsSafetyGroup) {
  DaemonHarness harness{profile_with_axes(2, true)};
  ASSERT_EQ(harness.daemon.stage_commands(commands(2, 1U, 0)),
            CommandAcceptance::accepted);
  harness.pdos[0].status_word = 0x0008U;
  harness.pdos[0].actual_velocity = 100;
  harness.pdos[1].actual_velocity = 100;

  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(2), true);

  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);
  EXPECT_EQ(harness.daemon.axis_request(1), AxisRequest::quick_stop);
  EXPECT_NE(harness.daemon.feedback(0).flags & kAxisFeedbackSafetyDriveFault, 0U);
  EXPECT_NE(harness.daemon.feedback(1).flags & kAxisFeedbackSafetyGroup, 0U);
}

TEST(RobotIoDaemonTest, EthercatOwnerAppliesWkcThresholdBeforeSending) {
  FakeClock clock;
  FakeRealtimeSystem realtime;
  SafetyOptions options{};
  options.consecutive_wkc_limit = 3U;
  auto profile = profile_with_axes(1);
  profile.axes[0].command_timeout = 1s;
  profile.axes[0].following_error_limit = 10.0;
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{
      backend, {EthercatAxisConfiguration{profile.axes[0], {}}}};
  RobotIoDaemon daemon{clock, realtime, options};
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.attach_ethercat(master).has_value());
  ASSERT_TRUE(daemon.start().has_value());
  const auto& handles = master.pdo_handles()[0];
  ASSERT_TRUE(handles.status_word.write(backend->image(), 0x0027U));
  ASSERT_TRUE(handles.mode_display.write(backend->image(), 8));
  ASSERT_TRUE(handles.actual_position.write(backend->image(), 0));
  ASSERT_TRUE(handles.actual_velocity.write(backend->image(), 100));
  ASSERT_TRUE(handles.actual_torque.write(backend->image(), 0));
  ASSERT_EQ(daemon.stage_commands(commands(1, 1U, 0)),
            CommandAcceptance::accepted);
  backend->set_health(DomainHealth{0U, 1U, false, true, true, 0, 1U});

  daemon.cycle();
  EXPECT_EQ(handles.control_word.read(backend->image()), 0x000FU);
  daemon.cycle();
  EXPECT_EQ(handles.control_word.read(backend->image()), 0x000FU);
  daemon.cycle();
  EXPECT_EQ(handles.control_word.read(backend->image()), 0x0002U);
  EXPECT_EQ(daemon.axis_request(0), AxisRequest::quick_stop);
}

TEST(RobotIoDaemonTest, EthercatOwnerKeepsIndependentOperationalGroupRunning) {
  FakeClock clock;
  FakeRealtimeSystem realtime;
  auto profile = profile_with_axes(2, false);
  for (auto& axis : profile.axes) {
    axis.command_timeout = 1s;
    axis.following_error_limit = 10.0;
  }
  auto backend = std::make_shared<FakeEthercatBackend>();
  std::vector<EthercatAxisConfiguration> axes;
  for (const auto& axis : profile.axes) {
    axes.push_back(EthercatAxisConfiguration{axis, {}});
  }
  EthercatMaster master{backend, std::move(axes)};
  RobotIoDaemon daemon{clock, realtime};
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.attach_ethercat(master).has_value());
  ASSERT_TRUE(daemon.start().has_value());
  for (const auto& handles : master.pdo_handles()) {
    ASSERT_TRUE(handles.status_word.write(backend->image(), 0x0027U));
    ASSERT_TRUE(handles.mode_display.write(backend->image(), 8));
    ASSERT_TRUE(handles.actual_position.write(backend->image(), 0));
    ASSERT_TRUE(handles.actual_velocity.write(backend->image(), 100));
    ASSERT_TRUE(handles.actual_torque.write(backend->image(), 0));
  }
  ASSERT_EQ(daemon.stage_commands(commands(2, 1U, 0)),
            CommandAcceptance::accepted);
  backend->set_health(DomainHealth{0U, 2U, false, true, false, 0, 0x0002U});

  daemon.cycle();

  EXPECT_EQ(master.pdo_handles()[0].control_word.read(backend->image()), 0x0002U);
  EXPECT_EQ(master.pdo_handles()[1].control_word.read(backend->image()), 0x000FU);
  EXPECT_EQ(daemon.axis_request(0), AxisRequest::quick_stop);
  EXPECT_EQ(daemon.axis_request(1), AxisRequest::enable);
}

TEST(RobotIoDaemonTest, RejectsOutOfOrderAndMutatedDuplicateCommands) {
  DaemonHarness harness{profile_with_axes(1)};
  const auto accepted = commands(1, 5U, 0);
  ASSERT_EQ(harness.daemon.stage_commands(accepted), CommandAcceptance::accepted);
  EXPECT_EQ(harness.daemon.stage_commands(accepted), CommandAcceptance::duplicate);

  auto changed_duplicate = accepted;
  changed_duplicate.axes[0].target = 0.25;
  EXPECT_EQ(harness.daemon.stage_commands(changed_duplicate),
            CommandAcceptance::rejected);
  EXPECT_EQ(harness.daemon.stage_commands(commands(1, 4U, 0)),
            CommandAcceptance::rejected);

  harness.pdos[0].actual_velocity = 100;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);
}

TEST(RobotIoDaemonTest, BoundsActualFaultResetEdgesPerPersistentFaultEpisode) {
  SafetyOptions options{};
  options.maximum_fault_resets = 2U;
  DaemonHarness harness{profile_with_axes(1), options};
  harness.pdos[0].status_word = 0x0008U;
  harness.pdos[0].actual_velocity = 100;

  for (std::uint64_t sequence = 1; sequence <= 2; ++sequence) {
    harness.clock.advance(1ms);
    ASSERT_EQ(harness.daemon.stage_commands(
                  commands(1, sequence * 2U - 1U, harness.clock.now_ns(), 0.0,
                           kAxisCommandFaultReset)),
              CommandAcceptance::accepted);
    harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
    EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::fault_reset);
    EXPECT_EQ(harness.pdos[0].control_word, 0x0080U);

    // A held request is not another edge and must not spend the episode budget.
    harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
    EXPECT_EQ(harness.pdos[0].control_word, 0x0000U);

    harness.clock.advance(1ms);
    ASSERT_EQ(harness.daemon.stage_commands(
                  commands(1, sequence * 2U, harness.clock.now_ns(), 0.0,
                           policy_runtime::kAxisCommandDisable)),
              CommandAcceptance::accepted);
    harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
    EXPECT_NE(harness.pdos[0].control_word, 0x0080U);
  }

  harness.clock.advance(1ms);
  ASSERT_EQ(harness.daemon.stage_commands(
                commands(1, 5U, harness.clock.now_ns(), 0.0,
                         kAxisCommandFaultReset)),
            CommandAcceptance::accepted);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);
  EXPECT_NE(harness.pdos[0].control_word, 0x0080U);

  // An unknown status is not evidence that the persistent fault cleared.
  harness.pdos[0].status_word = 0xFFFFU;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  harness.clock.advance(1ms);
  ASSERT_EQ(harness.daemon.stage_commands(
                commands(1, 6U, harness.clock.now_ns(), 0.0,
                         policy_runtime::kAxisCommandDisable)),
            CommandAcceptance::accepted);
  harness.pdos[0].status_word = 0x0008U;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  harness.clock.advance(1ms);
  ASSERT_EQ(harness.daemon.stage_commands(
                commands(1, 7U, harness.clock.now_ns(), 0.0,
                         kAxisCommandFaultReset)),
            CommandAcceptance::accepted);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_NE(harness.pdos[0].control_word, 0x0080U);

  // Only an observed non-fault CiA 402 state starts a new reset episode.
  harness.pdos[0].status_word = 0x0040U;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  harness.clock.advance(1ms);
  ASSERT_EQ(harness.daemon.stage_commands(
                commands(1, 8U, harness.clock.now_ns(), 0.0,
                         policy_runtime::kAxisCommandDisable)),
            CommandAcceptance::accepted);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  harness.pdos[0].status_word = 0x0008U;
  harness.clock.advance(1ms);
  ASSERT_EQ(harness.daemon.stage_commands(
                commands(1, 9U, harness.clock.now_ns(), 0.0,
                         kAxisCommandFaultReset)),
            CommandAcceptance::accepted);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.pdos[0].control_word, 0x0080U);
}

TEST(RobotIoDaemonTest, KeepsSafetyGroupPeersStoppedWhileFaultedAxisResets) {
  DaemonHarness harness{profile_with_axes(2, true)};
  ASSERT_EQ(harness.daemon.stage_commands(commands(2, 1U, 0)),
            CommandAcceptance::accepted);
  harness.pdos[0].status_word = 0x0008U;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(2), true);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(2), true);
  ASSERT_EQ(harness.daemon.axis_request(0), AxisRequest::disable);
  ASSERT_EQ(harness.daemon.axis_request(1), AxisRequest::disable);

  auto reset = commands(2, 2U, 0);
  reset.axes[0].flags = kAxisCommandFaultReset;
  reset.axes[1].flags = kAxisCommandEnable;
  ASSERT_EQ(harness.daemon.stage_commands(reset), CommandAcceptance::accepted);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(2), true);

  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::fault_reset);
  EXPECT_EQ(harness.pdos[0].control_word, 0x0080U);
  EXPECT_NE(harness.daemon.axis_request(1), AxisRequest::enable);
  EXPECT_NE(harness.pdos[1].control_word, 0x000FU);
}

TEST(RobotIoDaemonTest, SignalShutdownQuickStopsThenDisablesAtZeroSpeed) {
  DaemonSignalLatch signal_latch;
  ASSERT_TRUE(signal_latch.install().has_value());
  ASSERT_EQ(std::raise(SIGTERM), 0);
  EXPECT_TRUE(signal_latch.stop_requested());

  DaemonHarness harness{profile_with_axes(1)};
  ASSERT_EQ(harness.daemon.stage_commands(commands(1, 1U, 0)),
            CommandAcceptance::accepted);
  ASSERT_TRUE(harness.daemon.request_stop().has_value());
  harness.pdos[0].actual_velocity = 100;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);

  harness.pdos[0].actual_velocity = 0;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::disable);
  EXPECT_TRUE(harness.daemon.health().shutdown_complete);
}

TEST(RobotIoDaemonTest, SignalDrivenRunSendsFinalDisableBeforeMasterDeactivation) {
  FakeClock clock;
  FakeRealtimeSystem realtime;
  auto profile = profile_with_axes(1);
  profile.axes[0].command_timeout = 10s;
  profile.axes[0].following_error_limit = 10.0;
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{
      backend, {EthercatAxisConfiguration{profile.axes[0], {}}}};
  RobotIoDaemon daemon{clock, realtime};
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.attach_ethercat(master).has_value());
  ASSERT_TRUE(daemon.start().has_value());
  const auto& handles = master.pdo_handles()[0];
  ASSERT_TRUE(handles.status_word.write(backend->image(), 0x0027U));
  ASSERT_TRUE(handles.mode_display.write(backend->image(), 8));
  ASSERT_TRUE(handles.actual_position.write(backend->image(), 0));
  ASSERT_TRUE(handles.actual_velocity.write(backend->image(), 0));
  ASSERT_TRUE(handles.actual_torque.write(backend->image(), 0));
  backend->observe_control_word(handles.control_word.location().byte_offset);
  ASSERT_EQ(daemon.stage_commands(commands(1, 1U, 0)),
            CommandAcceptance::accepted);

  DaemonSignalLatch signal_latch;
  ASSERT_TRUE(signal_latch.install().has_value());
  ASSERT_EQ(std::raise(SIGTERM), 0);
  daemon.run();

  EXPECT_TRUE(daemon.health().shutdown_complete);
  EXPECT_EQ(backend->last_sent_control_word(), 0U);
  ASSERT_GT(backend->final_disable_send_order(), 0);
  ASSERT_GT(backend->deactivate_order(), 0);
  EXPECT_LT(backend->final_disable_send_order(), backend->deactivate_order());
}

TEST(RobotIoDaemonTest, ShutdownTimeoutFallsBackToDisable) {
  SafetyOptions options{};
  options.safe_stop_timeout = 10ms;
  DaemonHarness harness{profile_with_axes(1), options};
  ASSERT_TRUE(harness.daemon.request_stop().has_value());
  harness.pdos[0].actual_velocity = 100;
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);

  harness.clock.advance(11ms);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::disable);
  EXPECT_TRUE(harness.daemon.health().shutdown_complete);
}

TEST(RobotIoDaemonTest, StartsWithoutRealtimeGuaranteeWhenSetupDegrades) {
  FakeClock clock;
  FakeRealtimeSystem realtime;
  realtime.preempt_rt = false;
  realtime.lock_ok = true;
  realtime.pin_ok = true;
  realtime.fifo_ok = false;
  RobotIoDaemon daemon{clock, realtime};
  auto profile = profile_with_axes(1);

  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.start().has_value());
  ASSERT_TRUE(daemon.request_stop().has_value());
  daemon.run();
  const auto health = daemon.health();
  EXPECT_FALSE(health.running);
  EXPECT_FALSE(health.realtime_guarantee);
  EXPECT_EQ(realtime.lock_calls, 1U);
  EXPECT_EQ(realtime.pin_calls, 1U);
  EXPECT_EQ(realtime.fifo_calls, 1U);
  EXPECT_EQ(realtime.restore_calls, 1U);
  EXPECT_EQ(realtime.unlock_calls, 1U);
}

TEST(RobotIoDaemonTest, AppliesAndRestoresRealtimeSetupOnTheRunThread) {
  FakeClock clock;
  FakeRealtimeSystem realtime;
  RobotIoDaemon daemon{clock, realtime};
  auto profile = profile_with_axes(1);
  profile.axes[0].command_timeout = 10s;
  ASSERT_TRUE(daemon.configure(profile).has_value());
  const auto start_thread = std::this_thread::get_id();
  ASSERT_TRUE(daemon.start().has_value());
  EXPECT_EQ(realtime.capture_calls, 0U);
  ASSERT_TRUE(daemon.request_stop().has_value());

  std::thread::id run_thread;
  std::thread runner([&] {
    run_thread = std::this_thread::get_id();
    daemon.run();
  });
  runner.join();

  EXPECT_NE(run_thread, start_thread);
  EXPECT_EQ(realtime.setup_thread, run_thread);
  EXPECT_EQ(realtime.restore_thread, run_thread);
  EXPECT_EQ(realtime.capture_calls, 1U);
  EXPECT_EQ(realtime.restore_calls, 1U);
  EXPECT_EQ(realtime.unlock_calls, 1U);
}

TEST(RobotIoDaemonTest, RollsBackMasterRegistrationAfterFailedStart) {
  FakeClock clock;
  FakeRealtimeSystem realtime;
  auto profile = profile_with_axes(1);
  auto backend = std::make_shared<FakeEthercatBackend>();
  backend->fail_next_activation();
  EthercatMaster master{
      backend, {EthercatAxisConfiguration{profile.axes[0], {}}}};
  RobotIoDaemon daemon{clock, realtime};
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.attach_ethercat(master).has_value());

  EXPECT_FALSE(daemon.start().has_value());
  EXPECT_EQ(realtime.capture_calls, 0U);
  EXPECT_TRUE(daemon.start().has_value());
  ASSERT_TRUE(daemon.request_stop().has_value());
  daemon.run();
  EXPECT_TRUE(daemon.health().shutdown_complete);
}

TEST(RobotIoDaemonTest, SleepFailureDegradesTimingAndCompletesSafeStop) {
  FakeClock clock;
  clock.set_sleep_errors({EIO});
  FakeRealtimeSystem realtime;
  RobotIoDaemon daemon{clock, realtime};
  auto profile = profile_with_axes(1);
  profile.axes[0].command_timeout = 10s;
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.start().has_value());

  daemon.run();

  const auto health = daemon.health();
  EXPECT_TRUE(health.timing_fault);
  EXPECT_EQ(health.sleep_error, EIO);
  EXPECT_EQ(health.sleep_failures, 1U);
  EXPECT_FALSE(health.realtime_guarantee);
  EXPECT_TRUE(health.shutdown_complete);
}

TEST(RobotIoDaemonTest, IntegratesVersionedIpcSequencesAndTimestamps) {
  std::array<int, 2> sockets{-1, -1};
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, sockets.data()), 0);
  auto server = RobotIoIpcServer::create(sockets[0], 1U, 44U);
  ASSERT_TRUE(server.has_value()) << server.error().message;

  FakeClock clock;
  FakeRealtimeSystem realtime;
  RobotIoDaemon daemon{clock, realtime};
  auto profile = profile_with_axes(1);
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.attach_ipc(std::move(server.value())).has_value());
  auto client = RobotIoClient::connect(sockets[1], 44U);
  ASSERT_TRUE(client.has_value()) << client.error().message;
  ASSERT_TRUE(daemon.start().has_value());

  auto command = commands(1, 7U, 0, 0.25);
  ASSERT_TRUE(client.value()
                  .publish_commands(std::span<const AxisCommand>(command.axes.data(), 1U),
                                    7U, 0)
                  .has_value());
  auto pdos = enabled_pdos(1);
  allocation_probe::begin();
  const auto acceptance = daemon.refresh_commands();
  daemon.process_device_cycle(pdos, healthy_domain(1), true);
  const auto allocations = allocation_probe::end();
  EXPECT_EQ(acceptance, CommandAcceptance::accepted);
  EXPECT_EQ(allocations, 0U);

  auto feedback = client.value().read_feedback();
  ASSERT_TRUE(feedback.has_value()) << feedback.error().message;
  EXPECT_EQ(feedback.value().sequence, 7U);
  EXPECT_EQ(feedback.value().timestamp_ns, clock.now_ns());
  EXPECT_EQ(feedback.value().axes[0].sequence, 7U);
  EXPECT_EQ(daemon.health().last_command_sequence, 7U);
}

TEST(RobotIoDaemonTest, PublishesRaceFreeAxisSnapshotsDuringCommandHandoff) {
  auto profile = profile_with_axes(2);
  for (auto& axis : profile.axes) {
    axis.command_timeout = 10s;
    axis.following_error_limit = 10.0;
  }
  DaemonHarness harness{std::move(profile)};
  std::atomic<bool> publisher_done{};
  std::atomic<bool> reader_done{};
  std::atomic<bool> coherent{true};

  std::thread publisher([&] {
    for (std::uint64_t sequence = 1U; sequence <= 10'000U; ++sequence) {
      const auto acceptance = harness.daemon.stage_commands(
          commands(2, sequence, 0, static_cast<double>(sequence % 10U) / 100.0));
      if (acceptance != CommandAcceptance::accepted) {
        coherent.store(false, std::memory_order_release);
        break;
      }
    }
    publisher_done.store(true, std::memory_order_release);
  });
  std::thread reader([&] {
    while (!publisher_done.load(std::memory_order_acquire) ||
           !reader_done.load(std::memory_order_acquire)) {
      const auto snapshot = harness.daemon.axis_snapshot();
      if (snapshot.axis_count != 2U ||
          snapshot.feedback[0].sequence != snapshot.feedback[1].sequence) {
        coherent.store(false, std::memory_order_release);
        break;
      }
      static_cast<void>(harness.daemon.feedback(0));
      static_cast<void>(harness.daemon.axis_request(0));
      static_cast<void>(harness.daemon.health());
    }
  });

  while (!publisher_done.load(std::memory_order_acquire)) {
    harness.clock.advance(1us);
    harness.daemon.process_device_cycle(harness.pdos, healthy_domain(2), true);
  }
  publisher.join();
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(2), true);
  reader_done.store(true, std::memory_order_release);
  reader.join();

  EXPECT_TRUE(coherent.load(std::memory_order_acquire));
  EXPECT_EQ(harness.daemon.health().last_command_sequence, 10'000U);
  const auto final_snapshot = harness.daemon.axis_snapshot();
  EXPECT_EQ(final_snapshot.feedback[0].sequence, 10'000U);
  EXPECT_EQ(final_snapshot.feedback[1].sequence, 10'000U);
}

TEST(RobotIoDaemonTest, FreezesAtTwelveAxesAndRejectsThirteen) {
  FakeClock clock;
  FakeRealtimeSystem realtime;
  RobotIoDaemon daemon{clock, realtime};
  auto twelve = profile_with_axes(12);
  ASSERT_TRUE(daemon.configure(twelve).has_value());
  ASSERT_TRUE(daemon.start().has_value());

  RobotIoDaemon too_many{clock, realtime};
  auto thirteen = profile_with_axes(13);
  auto configured = too_many.configure(thirteen);
  ASSERT_FALSE(configured.has_value());
  EXPECT_EQ(configured.error().code, policy_runtime::ErrorCode::invalid_argument);
}

TEST(RobotIoDaemonTest, DeviceCycleDoesNotAllocateAfterStart) {
  auto profile = profile_with_axes(12);
  for (auto& axis : profile.axes) {
    axis.command_timeout = 10s;
    axis.following_error_limit = 10.0;
  }
  DaemonHarness harness{std::move(profile)};
  ASSERT_EQ(harness.daemon.stage_commands(commands(12, 1U, 0)),
            CommandAcceptance::accepted);

  allocation_probe::begin();
  for (unsigned cycle = 0; cycle < 100U; ++cycle) {
    harness.clock.advance(1ms);
    harness.daemon.process_device_cycle(harness.pdos, healthy_domain(12), true);
  }
  const auto allocations = allocation_probe::end();

  EXPECT_EQ(allocations, 0U);
}

TEST(RobotIoDaemonTest, FullIpcEthercatCycleDoesNotAllocateAfterStart) {
  std::array<int, 2> sockets{-1, -1};
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, sockets.data()), 0);
  auto server = RobotIoIpcServer::create(sockets[0], 12U, 71U);
  ASSERT_TRUE(server.has_value()) << server.error().message;

  FakeClock clock;
  FakeRealtimeSystem realtime;
  auto profile = profile_with_axes(12);
  for (auto& axis : profile.axes) {
    axis.command_timeout = 10s;
    axis.following_error_limit = 10.0;
  }
  auto backend = std::make_shared<FakeEthercatBackend>();
  std::vector<EthercatAxisConfiguration> axes;
  for (const auto& axis : profile.axes) {
    axes.push_back(EthercatAxisConfiguration{axis, {}});
  }
  EthercatMaster master{backend, std::move(axes)};
  RobotIoDaemon daemon{clock, realtime};
  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.attach_ethercat(master).has_value());
  ASSERT_TRUE(daemon.attach_ipc(std::move(server.value())).has_value());
  auto client = RobotIoClient::connect(sockets[1], 71U);
  ASSERT_TRUE(client.has_value()) << client.error().message;
  ASSERT_TRUE(daemon.start().has_value());
  for (const auto& handles : master.pdo_handles()) {
    ASSERT_TRUE(handles.status_word.write(backend->image(), 0x0027U));
    ASSERT_TRUE(handles.mode_display.write(backend->image(), 8));
    ASSERT_TRUE(handles.actual_position.write(backend->image(), 0));
    ASSERT_TRUE(handles.actual_velocity.write(backend->image(), 0));
    ASSERT_TRUE(handles.actual_torque.write(backend->image(), 0));
  }
  auto command = commands(12, 1U, 0);
  ASSERT_TRUE(client.value()
                  .publish_commands(
                      std::span<const AxisCommand>(command.axes.data(), 12U), 1U, 0)
                  .has_value());

  allocation_probe::begin();
  for (unsigned cycle = 0; cycle < 100U; ++cycle) {
    clock.advance(1ms);
    daemon.cycle();
  }
  const auto allocations = allocation_probe::end();

  EXPECT_EQ(allocations, 0U);
}

static_assert(noexcept(std::declval<RobotIoDaemon&>().process_device_cycle(
    std::declval<std::span<Cia402PdoView>>(), std::declval<DomainHealth>(), true)));
static_assert(noexcept(std::declval<RobotIoDaemon&>().cycle()));
static_assert(noexcept(std::declval<LinuxRealtimeSystem&>().lock_process_memory()));

}  // namespace
