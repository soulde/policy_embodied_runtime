#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <span>
#include <string>
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
void operator delete(void* allocation) noexcept { std::free(allocation); }
void operator delete[](void* allocation) noexcept { std::free(allocation); }
void operator delete(void* allocation, std::size_t) noexcept { std::free(allocation); }
void operator delete[](void* allocation, std::size_t) noexcept {
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
  std::int64_t now_ns() const noexcept override { return now_ns_; }

  int sleep_until_ns(std::int64_t absolute_ns) noexcept override {
    ++sleep_calls_;
    last_deadline_ns_ = absolute_ns;
    if (absolute_ns > now_ns_) {
      now_ns_ = absolute_ns;
    }
    return 0;
  }

  void advance(std::chrono::nanoseconds duration) noexcept {
    now_ns_ += duration.count();
  }

  unsigned sleep_calls() const noexcept { return sleep_calls_; }
  std::int64_t last_deadline_ns() const noexcept { return last_deadline_ns_; }

 private:
  std::int64_t now_ns_{};
  std::int64_t last_deadline_ns_{};
  unsigned sleep_calls_{};
};

class FakeRealtimeSystem final : public RealtimeSystem {
 public:
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

  bool preempt_rt{true};
  bool lock_ok{true};
  bool pin_ok{true};
  bool fifo_ok{true};
  unsigned lock_calls{};
  unsigned pin_calls{};
  unsigned fifo_calls{};
  int pinned_cpu{-1};
  int fifo_priority{};
};

class FakeEthercatBackend final : public EthercatBackend {
 public:
  void set_health(DomainHealth health) noexcept { health_ = health; }
  std::span<std::byte> image() noexcept { return image_; }

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
    active_ = true;
    return Result<void>::success();
  }
  void deactivate() noexcept override { active_ = false; }
  void receive() noexcept override {}
  void process_domain() noexcept override {}
  std::span<std::byte> process_image() noexcept override { return image_; }
  void queue_domain() noexcept override {}
  void send() noexcept override {}
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
  std::array<std::byte, 128> image_{};
  std::size_t next_offset_{};
  bool active_{};
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
  EXPECT_EQ(first.sequence, 1U);
  EXPECT_EQ(second.sequence, 2U);
  EXPECT_EQ(clock.sleep_calls(), 2U);
  EXPECT_EQ(clock.last_deadline_ns(), 2'000'000);
  EXPECT_EQ(second.period, 1ms);
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

TEST(RobotIoDaemonTest, BoundsFaultResetRequestsPerFaultEpisode) {
  SafetyOptions options{};
  options.maximum_fault_resets = 3U;
  DaemonHarness harness{profile_with_axes(1), options};
  harness.pdos[0].status_word = 0x0008U;
  harness.pdos[0].actual_velocity = 100;

  for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
    harness.clock.advance(1ms);
    ASSERT_EQ(harness.daemon.stage_commands(
                  commands(1, sequence, harness.clock.now_ns(), 0.0,
                           kAxisCommandFaultReset)),
              CommandAcceptance::accepted);
    harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
    EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::fault_reset);
  }

  harness.clock.advance(1ms);
  ASSERT_EQ(harness.daemon.stage_commands(
                commands(1, 4U, harness.clock.now_ns(), 0.0,
                         kAxisCommandFaultReset)),
            CommandAcceptance::accepted);
  harness.daemon.process_device_cycle(harness.pdos, healthy_domain(1), true);
  EXPECT_EQ(harness.daemon.axis_request(0), AxisRequest::quick_stop);
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
  realtime.lock_ok = false;
  realtime.pin_ok = false;
  realtime.fifo_ok = false;
  RobotIoDaemon daemon{clock, realtime};
  auto profile = profile_with_axes(1);

  ASSERT_TRUE(daemon.configure(profile).has_value());
  ASSERT_TRUE(daemon.start().has_value());
  const auto health = daemon.health();
  EXPECT_TRUE(health.running);
  EXPECT_FALSE(health.realtime_guarantee);
  EXPECT_EQ(realtime.lock_calls, 1U);
  EXPECT_EQ(realtime.pin_calls, 1U);
  EXPECT_EQ(realtime.fifo_calls, 1U);
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

static_assert(noexcept(std::declval<RobotIoDaemon&>().process_device_cycle(
    std::declval<std::span<Cia402PdoView>>(), std::declval<DomainHealth>(), true)));
static_assert(noexcept(std::declval<RobotIoDaemon&>().cycle()));
static_assert(noexcept(std::declval<LinuxRealtimeSystem&>().lock_process_memory()));

}  // namespace
