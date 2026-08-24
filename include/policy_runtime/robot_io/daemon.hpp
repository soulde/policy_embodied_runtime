#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include <signal.h>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/cia402/pdo.hpp"
#include "policy_runtime/protocol/cia402/state_machine.hpp"
#include "policy_runtime/robot/devices/cia402/axis.hpp"
#include "policy_runtime/robot_io/ipc_server.hpp"
#include "policy_runtime/robot_io/realtime_loop.hpp"
#include "policy_runtime/robot_io/safety_supervisor.hpp"
#include "policy_runtime/robot_io/transport_scheduler.hpp"
#include "policy_runtime/transport/ethercat/backend.hpp"

namespace policy_runtime {

class EthercatMaster;

struct DaemonHealth {
  bool configured{};
  bool running{};
  bool accepting_commands{};
  bool realtime_guarantee{};
  bool shutdown_requested{};
  bool shutdown_complete{};
  bool process_data_valid{};
  std::uint32_t axis_count{};
  std::uint32_t working_counter{};
  std::uint32_t expected_working_counter{};
  std::uint32_t consecutive_wkc_failures{};
  std::uint32_t safety_flags{};
  std::uint64_t cycle_sequence{};
  std::uint64_t last_command_sequence{};
  std::int64_t last_command_timestamp_ns{};
  std::int64_t feedback_timestamp_ns{};
  std::int64_t dc_deviation_ns{};
  std::uint64_t deadline_misses{};
  std::int64_t actual_period_ns{};
  std::int64_t maximum_jitter_ns{};
};

class DaemonSignalLatch {
 public:
  DaemonSignalLatch() = default;
  DaemonSignalLatch(const DaemonSignalLatch&) = delete;
  DaemonSignalLatch& operator=(const DaemonSignalLatch&) = delete;
  ~DaemonSignalLatch();

  Result<void> install();
  bool stop_requested() const noexcept;

 private:
  struct sigaction previous_ {};
  bool installed_{};
};

class RobotIoDaemon {
 public:
  RobotIoDaemon();
  RobotIoDaemon(MonotonicClock& clock, RealtimeSystem& realtime_system,
                SafetyOptions safety_options = {},
                RealtimeLoopConfig loop_config = {}) noexcept;
  ~RobotIoDaemon();

  RobotIoDaemon(const RobotIoDaemon&) = delete;
  RobotIoDaemon& operator=(const RobotIoDaemon&) = delete;

  Result<void> configure(const profiles::RobotProfile& profile);
  // The attached master remains owned by the caller and must outlive the daemon.
  Result<void> attach_ethercat(EthercatMaster& master);
  Result<void> attach_ipc(RobotIoIpcServer server);
  Result<void> start();
  void run() noexcept;
  void cycle() noexcept;
  Result<void> request_stop();
  Result<void> poll_control();

  CommandAcceptance stage_commands(
      const Snapshot<AxisCommand>& snapshot) noexcept;
  CommandAcceptance refresh_commands() noexcept;
  void process_device_cycle(std::span<Cia402PdoView> pdos,
                            const DomainHealth& domain,
                            bool process_data_valid) noexcept;

  DaemonHealth health() const noexcept;
  AxisRequest axis_request(std::size_t axis_index) const noexcept;
  AxisFeedback feedback(std::size_t axis_index) const noexcept;

 private:
  static void ethercat_cycle_handler(void* context,
                                     std::span<Cia402PdoView> pdos,
                                     const DomainHealth& domain,
                                     bool process_data_valid) noexcept;
  void cycle(const CycleContext& context) noexcept;
  void stop_transports() noexcept;
  bool health_atomics_are_lock_free() const noexcept;

  MonotonicClock* clock_{};
  RealtimeLoop loop_;
  SafetySupervisor safety_;
  TransportScheduler scheduler_;
  EthercatMaster* ethercat_master_{};
  std::optional<RobotIoIpcServer> ipc_;
  std::array<std::optional<Cia402Axis>, kRobotIoMaximumAxes> axes_{};
  std::array<Cia402PdoView, kRobotIoMaximumAxes> standalone_pdos_{};
  std::array<AxisFeedback, kRobotIoMaximumAxes> feedback_{};
  std::uint32_t axis_count_{};
  bool configured_{};
  bool master_registered_{};
  bool master_handler_bound_{};

  std::atomic<bool> running_{};
  std::atomic<bool> accepting_commands_{};
  std::atomic<bool> realtime_guarantee_{};
  std::atomic<bool> stop_requested_{};
  std::atomic<bool> shutdown_complete_{};
  std::atomic<bool> process_data_valid_{};
  std::atomic<std::uint32_t> working_counter_{};
  std::atomic<std::uint32_t> expected_working_counter_{};
  std::atomic<std::uint32_t> consecutive_wkc_failures_{};
  std::atomic<std::uint32_t> safety_flags_{};
  std::atomic<std::uint64_t> cycle_sequence_{};
  std::atomic<std::uint64_t> last_command_sequence_{};
  std::atomic<std::int64_t> last_command_timestamp_ns_{};
  std::atomic<std::int64_t> feedback_timestamp_ns_{};
  std::atomic<std::int64_t> dc_deviation_ns_{};
};

}  // namespace policy_runtime
