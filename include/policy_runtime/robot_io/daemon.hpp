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
#include "policy_runtime/robot/devices/damiao.hpp"
#include "policy_runtime/robot_io/transport_runtime.hpp"
#include "policy_runtime/robot/devices/st3215/servo.hpp"
#include "policy_runtime/robot_io/local_snapshot.hpp"
#include "policy_runtime/robot_io/realtime_loop.hpp"
#include "policy_runtime/robot_io/safety_supervisor.hpp"
#include "policy_runtime/robot_io/transport_scheduler.hpp"
#include "policy_runtime/robot_io/dds/daemon_endpoint.hpp"
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
  std::uint64_t skipped_releases{};
  std::uint64_t sleep_failures{};
  std::int64_t actual_period_ns{};
  std::int64_t wake_latency_ns{};
  std::int64_t maximum_wake_latency_ns{};
  std::int64_t execution_time_ns{};
  std::int64_t maximum_execution_time_ns{};
  bool timing_fault{};
  int sleep_error{};
  std::uint32_t serial_servo_count{};
  std::uint32_t serial_fault_count{};
  std::uint32_t serial_safety_flags{};
};

struct DaemonAxisSnapshot {
  std::uint64_t cycle_sequence{};
  std::int64_t timestamp_ns{};
  std::uint32_t axis_count{};
  std::array<AxisRequest, kRobotIoMaximumAxes> requests{};
  std::array<AxisFeedback, kRobotIoMaximumAxes> feedback{};
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
  Result<void> attach_dds(const profiles::RobotProfile& profile);
  Result<void> start();
  Result<void> run();
  void cycle() noexcept;
  Result<void> request_stop();

  CommandAcceptance stage_commands(
      const Snapshot<AxisCommand>& snapshot) noexcept;
  CommandAcceptance refresh_commands() noexcept;
  void process_device_cycle(std::span<Cia402PdoView> pdos,
                            const DomainHealth& domain,
                            bool process_data_valid) noexcept;

  DaemonHealth health() const noexcept;
  DaemonAxisSnapshot axis_snapshot() const noexcept;
  AxisRequest axis_request(std::size_t axis_index) const noexcept;
  AxisFeedback feedback(std::size_t axis_index) const noexcept;
  St3215CommandAcceptance stage_servo_command(
      std::size_t servo_index,
      const St3215ServoCommand& command) noexcept;
  St3215ServoFeedback servo_feedback(
      std::size_t servo_index) const noexcept;
  St3215RegistrySafetySnapshot serial_safety_snapshot() const noexcept;

 private:
  static void ethercat_cycle_handler(void* context,
                                     std::span<Cia402PdoView> pdos,
                                     const DomainHealth& domain,
                                     bool process_data_valid) noexcept;
  void cycle_owned(const CycleContext& context) noexcept;
  CommandAcceptance refresh_commands_owned() noexcept;
  void process_device_cycle_owned(std::span<Cia402PdoView> pdos,
                                  const DomainHealth& domain,
                                  bool process_data_valid) noexcept;
  bool owns_cycle() const noexcept;
  CommandAcceptance consume_staged_commands() noexcept;
  void stop_transports() noexcept;
  bool health_atomics_are_lock_free() const noexcept;

  MonotonicClock* clock_{};
  RealtimeLoop loop_;
  SafetySupervisor safety_;
  SafetySupervisor command_ingress_;
  TransportScheduler scheduler_;
  St3215DeviceRegistry serial_devices_;
  struct DamiaoBusRoute {
    std::size_t bus_index{};
    std::size_t local_index{};
  };
  std::vector<std::unique_ptr<robot_io::TransportRuntime>> transport_runtimes_;
  std::vector<DamiaoSensor> damiao_sensors_;
  std::vector<DamiaoActuator> damiao_actuators_;
  std::array<std::optional<DamiaoBusRoute>, kRobotIoMaximumServos>
      damiao_routes_{};
  EthercatMaster* ethercat_master_{};
  std::optional<robot_io::dds::DdsDaemonEndpoint> dds_;
  std::array<std::optional<Cia402Axis>, kRobotIoMaximumAxes> axes_{};
  std::array<Cia402PdoView, kRobotIoMaximumAxes> standalone_pdos_{};
  std::array<AxisFeedback, kRobotIoMaximumAxes> cycle_feedback_{};
  std::uint32_t axis_count_{};
  bool configured_{};
  bool master_registered_{};
  bool master_handler_bound_{};
  std::array<std::uint16_t, kMaximumSt3215Servos>
      serial_axis_stop_masks_{};

  struct StagedCommandEvent {
    std::uint64_t publication{};
    Snapshot<AxisCommand> snapshot{};
  };

  LocalSnapshot<StagedCommandEvent> command_handoff_;
  LocalSnapshot<DaemonAxisSnapshot> axis_publication_;
  std::atomic_flag command_ingress_gate_ = ATOMIC_FLAG_INIT;
  std::uint64_t command_publication_{};
  std::uint64_t consumed_command_publication_{};
  std::uint64_t acknowledged_rejection_publication_{};
  std::atomic<std::uint64_t> rejected_command_publication_{};

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
  std::atomic<bool> timing_fault_{};
  std::atomic<int> sleep_error_{};
  std::atomic<std::uint32_t> serial_servo_count_{};
  std::atomic<std::uint32_t> serial_fault_count_{};
  std::atomic<std::uint32_t> serial_safety_flags_{};
  std::atomic<std::uint64_t> cycle_owner_token_{};
};

}  // namespace policy_runtime
