#include "policy_runtime/robot_io/daemon.hpp"

#include <cerrno>
#include <exception>
#include <string>
#include <system_error>
#include <utility>

#include <sys/syscall.h>
#include <unistd.h>

#include "policy_runtime/transport/ethercat/master.hpp"

namespace policy_runtime {
namespace {

volatile sig_atomic_t signal_stop_requested = 0;
volatile sig_atomic_t signal_latch_installed = 0;

void daemon_signal_handler(int) noexcept { signal_stop_requested = 1; }

SystemMonotonicClock& default_clock() {
  static SystemMonotonicClock clock;
  return clock;
}

LinuxRealtimeSystem& default_realtime_system() {
  static LinuxRealtimeSystem system;
  return system;
}

Error system_error(ErrorCode code, const char* operation) {
  return {code, std::string(operation) + ": " +
                    std::error_code(errno, std::generic_category()).message()};
}

SafetyBusState safety_bus_state(const DomainHealth& domain,
                                bool process_data_valid) noexcept {
  return SafetyBusState{
      domain.working_counter,
      domain.expected_working_counter.value_or(0U),
      domain.expected_working_counter.has_value(),
      domain.working_counter_complete,
      domain.link_up,
      domain.all_slaves_operational,
      process_data_valid,
      domain.operational_axes_mask,
      0U,
      domain.dc_deviation_ns};
}

std::uint64_t current_thread_token() noexcept {
  return static_cast<std::uint64_t>(::syscall(SYS_gettid));
}

class CycleOwnerGuard {
 public:
  explicit CycleOwnerGuard(std::atomic<std::uint64_t>& owner) noexcept
      : owner_(&owner), token_(current_thread_token()) {
    std::uint64_t expected{};
    acquired_ = owner_->compare_exchange_strong(
        expected, token_, std::memory_order_acq_rel, std::memory_order_acquire);
  }

  CycleOwnerGuard(const CycleOwnerGuard&) = delete;
  CycleOwnerGuard& operator=(const CycleOwnerGuard&) = delete;

  ~CycleOwnerGuard() {
    if (acquired_) {
      owner_->store(0U, std::memory_order_release);
    }
  }

  explicit operator bool() const noexcept { return acquired_; }

 private:
  std::atomic<std::uint64_t>* owner_{};
  std::uint64_t token_{};
  bool acquired_{};
};

}  // namespace

DaemonSignalLatch::~DaemonSignalLatch() {
  if (installed_) {
    static_cast<void>(sigaction(SIGTERM, &previous_, nullptr));
    signal_stop_requested = 0;
    signal_latch_installed = 0;
  }
}

Result<void> DaemonSignalLatch::install() {
  if (installed_ || signal_latch_installed != 0) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "SIGTERM latch is already installed"});
  }
  struct sigaction action {};
  action.sa_handler = daemon_signal_handler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  signal_stop_requested = 0;
  if (sigaction(SIGTERM, &action, &previous_) != 0) {
    return Result<void>::failure(system_error(ErrorCode::io, "sigaction(SIGTERM)"));
  }
  installed_ = true;
  signal_latch_installed = 1;
  return Result<void>::success();
}

bool DaemonSignalLatch::stop_requested() const noexcept {
  return signal_stop_requested != 0;
}

RobotIoDaemon::RobotIoDaemon()
    : RobotIoDaemon(default_clock(), default_realtime_system()) {}

RobotIoDaemon::RobotIoDaemon(MonotonicClock& clock,
                             RealtimeSystem& realtime_system,
                             SafetyOptions safety_options,
                             RealtimeLoopConfig loop_config) noexcept
    : clock_(&clock),
      loop_(clock, realtime_system, loop_config),
      safety_(safety_options),
      command_ingress_(safety_options) {}

RobotIoDaemon::~RobotIoDaemon() { stop_transports(); }

Result<void> RobotIoDaemon::configure(const profiles::RobotProfile& profile) {
  if (configured_ || running_.load(std::memory_order_acquire)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "robot I/O daemon is already configured"});
  }
  if (profile.axes.size() > kRobotIoMaximumAxes) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "robot I/O daemon supports at most 12 axes"});
  }
  // Validate every independently stateful subsystem before mutating the
  // daemon. This keeps a failed configure attempt retryable.
  SafetySupervisor safety_preflight{};
  if (auto checked = safety_preflight.configure(profile.axes);
      !checked.has_value()) {
    return checked;
  }
  St3215DeviceRegistry serial_preflight;
  if (auto checked = serial_preflight.configure(profile.st3215_servos);
      !checked.has_value()) {
    return checked;
  }

  std::array<std::optional<Cia402Axis>, kRobotIoMaximumAxes> configured_axes;
  try {
    for (std::size_t axis = 0; axis < profile.axes.size(); ++axis) {
      configured_axes[axis].emplace(profile.axes[axis]);
      standalone_pdos_[axis].mode_display =
          static_cast<std::int8_t>(profile.axes[axis].mode);
    }
  } catch (const std::exception& error) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         std::string("failed to construct configured CiA 402 axis: ") + error.what()});
  }
  auto safety_configured = safety_.configure(profile.axes);
  if (!safety_configured.has_value()) {
    return safety_configured;
  }
  auto ingress_configured = command_ingress_.configure(profile.axes);
  if (!ingress_configured.has_value()) {
    return ingress_configured;
  }
  auto serial_configured = serial_devices_.configure(profile.st3215_servos);
  if (!serial_configured.has_value()) {
    return serial_configured;
  }
  for (std::size_t servo = 0; servo < profile.st3215_servos.size(); ++servo) {
    for (std::size_t axis = 0; axis < profile.axes.size(); ++axis) {
      if (profile.st3215_servos[servo].safety_group ==
          profile.axes[axis].safety_group) {
        serial_axis_stop_masks_[servo] |=
            static_cast<std::uint16_t>(std::uint16_t{1U} << axis);
      }
    }
  }
  axes_ = std::move(configured_axes);
  axis_count_ = static_cast<std::uint32_t>(profile.axes.size());
  DaemonAxisSnapshot initial{};
  initial.axis_count = axis_count_;
  axis_publication_.publish(initial);
  configured_ = true;
  return Result<void>::success();
}

Result<void> RobotIoDaemon::attach_ethercat(EthercatMaster& master) {
  if (!configured_ || running_.load(std::memory_order_acquire) ||
      ethercat_master_ != nullptr || master.pdo_views().size() != axis_count_) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "EtherCAT master must match the configured inactive daemon"});
  }
  ethercat_master_ = &master;
  return Result<void>::success();
}

Result<void> RobotIoDaemon::attach_ipc(RobotIoIpcServer server) {
  if (!configured_ || running_.load(std::memory_order_acquire) || ipc_.has_value() ||
      server.axis_count() != axis_count_ ||
      server.servo_count() != serial_devices_.servo_count()) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "IPC endpoint must match the configured inactive daemon"});
  }
  auto setup = server.send_setup();
  if (!setup.has_value()) {
    return setup;
  }
  ipc_.emplace(std::move(server));
  return Result<void>::success();
}

bool RobotIoDaemon::health_atomics_are_lock_free() const noexcept {
  return running_.is_lock_free() && accepting_commands_.is_lock_free() &&
         realtime_guarantee_.is_lock_free() && stop_requested_.is_lock_free() &&
         shutdown_complete_.is_lock_free() && process_data_valid_.is_lock_free() &&
         working_counter_.is_lock_free() && expected_working_counter_.is_lock_free() &&
         consecutive_wkc_failures_.is_lock_free() && safety_flags_.is_lock_free() &&
         cycle_sequence_.is_lock_free() &&
         last_command_sequence_.is_lock_free() &&
         last_command_timestamp_ns_.is_lock_free() &&
         feedback_timestamp_ns_.is_lock_free() && dc_deviation_ns_.is_lock_free() &&
         timing_fault_.is_lock_free() && sleep_error_.is_lock_free() &&
         serial_servo_count_.is_lock_free() &&
         serial_fault_count_.is_lock_free() &&
         serial_safety_flags_.is_lock_free() &&
         cycle_owner_token_.is_lock_free() &&
         rejected_command_publication_.is_lock_free() &&
         loop_.atomics_are_lock_free() &&
         command_handoff_.atomics_are_lock_free() &&
         axis_publication_.atomics_are_lock_free() &&
         serial_devices_.atomics_are_lock_free();
}

Result<void> RobotIoDaemon::start() {
  if (!configured_ || running_.load(std::memory_order_acquire) ||
      stop_requested_.load(std::memory_order_acquire) ||
      shutdown_complete_.load(std::memory_order_acquire)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "robot I/O daemon is not startable"});
  }
  if (!health_atomics_are_lock_free()) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "daemon health atomics are not lock-free"});
  }
  auto loop_config = loop_.validate_config();
  if (!loop_config.has_value()) {
    return loop_config;
  }

  if (ethercat_master_ != nullptr) {
    auto scheduled = scheduler_.add(*ethercat_master_);
    if (!scheduled.has_value()) {
      return scheduled;
    }
    master_registered_ = true;
    auto handler = ethercat_master_->set_supervised_cycle_handler(
        &RobotIoDaemon::ethercat_cycle_handler, this);
    if (!handler.has_value()) {
      static_cast<void>(scheduler_.remove(*ethercat_master_));
      master_registered_ = false;
      return handler;
    }
    master_handler_bound_ = true;
    auto opened = ethercat_master_->open();
    if (!opened.has_value()) {
      static_cast<void>(ethercat_master_->clear_supervised_cycle_handler(this));
      master_handler_bound_ = false;
      static_cast<void>(scheduler_.remove(*ethercat_master_));
      master_registered_ = false;
      return opened;
    }
  }

  auto serial_started = serial_devices_.start(scheduler_);
  if (!serial_started.has_value()) {
    if (ethercat_master_ != nullptr && master_handler_bound_) {
      ethercat_master_->close();
      static_cast<void>(
          ethercat_master_->clear_supervised_cycle_handler(this));
      master_handler_bound_ = false;
    }
    if (ethercat_master_ != nullptr && master_registered_) {
      static_cast<void>(scheduler_.remove(*ethercat_master_));
      master_registered_ = false;
    }
    return serial_started;
  }

  accepting_commands_.store(true, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  return Result<void>::success();
}

CommandAcceptance RobotIoDaemon::stage_commands(
    const Snapshot<AxisCommand>& snapshot) noexcept {
  if (!accepting_commands_.load(std::memory_order_acquire)) {
    return CommandAcceptance::rejected;
  }
  while (command_ingress_gate_.test_and_set(std::memory_order_acquire)) {
  }
  if (!accepting_commands_.load(std::memory_order_acquire)) {
    command_ingress_gate_.clear(std::memory_order_release);
    return CommandAcceptance::rejected;
  }
  const auto accepted =
      command_ingress_.accept_commands(snapshot, clock_->now_ns());
  if (accepted == CommandAcceptance::accepted) {
    ++command_publication_;
    command_handoff_.publish({command_publication_, snapshot});
  } else if (accepted == CommandAcceptance::rejected) {
    ++command_publication_;
    rejected_command_publication_.store(command_publication_,
                                        std::memory_order_release);
  }
  command_ingress_gate_.clear(std::memory_order_release);
  if (accepted == CommandAcceptance::accepted) {
    last_command_sequence_.store(snapshot.sequence, std::memory_order_release);
    last_command_timestamp_ns_.store(snapshot.timestamp_ns,
                                     std::memory_order_release);
  }
  return accepted;
}

CommandAcceptance RobotIoDaemon::consume_staged_commands() noexcept {
  const auto rejected_publication =
      rejected_command_publication_.load(std::memory_order_acquire);
  auto event = command_handoff_.read();
  if (rejected_publication != acknowledged_rejection_publication_) {
    acknowledged_rejection_publication_ = rejected_publication;
    if (consumed_command_publication_ > rejected_publication) {
      // A strictly newer valid command raced ahead of this rejection. Replay
      // it after the required safe-stop cycle; publications at or below the
      // rejection barrier are permanently stale.
      consumed_command_publication_ = rejected_publication;
    }
    Snapshot<AxisCommand> invalid{};
    invalid.axis_count = kRobotIoMaximumAxes + 1U;
    return safety_.accept_commands(invalid, clock_->now_ns());
  }
  if (event.has_value() &&
      event->publication <= acknowledged_rejection_publication_) {
    return CommandAcceptance::duplicate;
  }
  if (!event.has_value() ||
      event->publication == consumed_command_publication_) {
    return CommandAcceptance::duplicate;
  }
  consumed_command_publication_ = event->publication;
  return safety_.accept_commands(event->snapshot, clock_->now_ns());
}

CommandAcceptance RobotIoDaemon::refresh_commands() noexcept {
  CycleOwnerGuard owner{cycle_owner_token_};
  if (!owner) {
    return CommandAcceptance::rejected;
  }
  return refresh_commands_owned();
}

CommandAcceptance RobotIoDaemon::refresh_commands_owned() noexcept {
  auto acceptance = consume_staged_commands();
  if (acceptance == CommandAcceptance::rejected) {
    // Preserve one complete safe-stop output cycle. IPC (or a newer local
    // publication) remains available for the following owner cycle.
    return acceptance;
  }
  if (!ipc_.has_value()) {
    return acceptance;
  }
  auto snapshot = ipc_->read_commands_realtime();
  if (!snapshot.has_value()) {
    if (snapshot.error().code == ErrorCode::protocol) {
      Snapshot<AxisCommand> invalid{};
      invalid.axis_count = kRobotIoMaximumAxes + 1U;
      static_cast<void>(safety_.accept_commands(invalid, clock_->now_ns()));
      return CommandAcceptance::rejected;
    }
    return acceptance;
  }
  const auto ipc_acceptance =
      safety_.accept_commands(snapshot.value(), clock_->now_ns());
  if (ipc_acceptance != CommandAcceptance::rejected &&
      ipc_->servo_count() != 0U) {
    ipc_commit_sequence_ = snapshot.value().sequence;
    ipc_commit_timestamp_ns_ = snapshot.value().timestamp_ns;
    has_ipc_commit_ = true;
  }
  if (ipc_acceptance == CommandAcceptance::accepted) {
    last_command_sequence_.store(snapshot.value().sequence,
                                 std::memory_order_release);
    last_command_timestamp_ns_.store(snapshot.value().timestamp_ns,
                                     std::memory_order_release);
  }
  return ipc_acceptance;
}

void RobotIoDaemon::process_device_cycle(std::span<Cia402PdoView> pdos,
                                         const DomainHealth& domain,
                                         bool process_data_valid) noexcept {
  CycleOwnerGuard owner{cycle_owner_token_};
  if (!owner) {
    return;
  }
  process_device_cycle_owned(pdos, domain, process_data_valid);
}

void RobotIoDaemon::process_device_cycle_owned(
    std::span<Cia402PdoView> pdos, const DomainHealth& domain,
    bool process_data_valid) noexcept {
  if (!running_.load(std::memory_order_acquire) || pdos.size() != axis_count_) {
    process_data_valid_.store(false, std::memory_order_release);
    return;
  }
  static_cast<void>(consume_staged_commands());
  const auto now = clock_->now_ns();
  if (ipc_.has_value() && ipc_->servo_count() != 0U) {
    auto commands = ipc_->read_servo_commands_realtime();
    if (commands.has_value() && has_ipc_commit_ &&
        commands.value().sequence == ipc_commit_sequence_ &&
        commands.value().timestamp_ns == ipc_commit_timestamp_ns_) {
      static_cast<void>(serial_devices_.stage_commands(commands.value(), now));
    }
  }
  if (stop_requested_.load(std::memory_order_acquire)) {
    accepting_commands_.store(false, std::memory_order_release);
    safety_.request_shutdown(now);
  }
  const auto serial_safety = serial_devices_.safety_snapshot(now);
  serial_servo_count_.store(serial_safety.servo_count,
                            std::memory_order_release);
  serial_fault_count_.store(serial_safety.fault_count,
                            std::memory_order_release);
  serial_safety_flags_.store(serial_safety.aggregate_flags,
                             std::memory_order_release);
  std::uint16_t serial_stop_mask{};
  for (std::size_t servo = 0; servo < serial_safety.servo_count; ++servo) {
    if ((serial_safety.fault_mask & (std::uint32_t{1U} << servo)) != 0U) {
      serial_stop_mask |= serial_axis_stop_masks_[servo];
    }
  }
  auto bus_state = safety_bus_state(domain, process_data_valid);
  bus_state.external_stop_axes_mask = serial_stop_mask;
  auto decisions = safety_.evaluate(bus_state, pdos, now);

  std::uint32_t safety_flags{};
  for (std::size_t axis = 0; axis < axis_count_; ++axis) {
    auto value = axes_[axis]->cycle(decisions.commands[axis], pdos[axis]);
    value.sequence = safety_.last_command_sequence();
    value.timestamp_ns = now;
    value.flags |= decisions.feedback_flags[axis];
    cycle_feedback_[axis] = value;
    safety_flags |= decisions.feedback_flags[axis];
  }

  const auto sequence = cycle_sequence_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
  static_cast<void>(sequence);
  working_counter_.store(domain.working_counter, std::memory_order_release);
  expected_working_counter_.store(
      domain.expected_working_counter.value_or(0U), std::memory_order_release);
  consecutive_wkc_failures_.store(safety_.consecutive_wkc_failures(),
                                  std::memory_order_release);
  process_data_valid_.store(process_data_valid, std::memory_order_release);
  dc_deviation_ns_.store(domain.dc_deviation_ns, std::memory_order_release);
  feedback_timestamp_ns_.store(now, std::memory_order_release);
  safety_flags_.store(safety_flags, std::memory_order_release);
  shutdown_complete_.store(decisions.shutdown_complete,
                           std::memory_order_release);

  DaemonAxisSnapshot public_axes{};
  public_axes.cycle_sequence = sequence;
  public_axes.timestamp_ns = now;
  public_axes.axis_count = axis_count_;
  for (std::size_t axis = 0; axis < axis_count_; ++axis) {
    public_axes.requests[axis] = decisions.requests[axis];
    public_axes.feedback[axis] = cycle_feedback_[axis];
  }
  axis_publication_.publish(public_axes);

  if (ipc_.has_value()) {
    static_cast<void>(ipc_->publish_feedback_realtime(
        std::span<const AxisFeedback>(cycle_feedback_.data(), axis_count_),
        safety_.last_command_sequence(), now));
    if (ipc_->servo_count() != 0U) {
      std::array<St3215ServoFeedback, kMaximumSt3215Servos> feedback{};
      for (std::size_t index = 0; index < ipc_->servo_count(); ++index) {
        feedback[index] = serial_devices_.feedback(index, now);
      }
      static_cast<void>(ipc_->publish_servo_feedback_realtime(
          std::span<const St3215ServoFeedback>(feedback.data(),
                                               ipc_->servo_count()),
          sequence, now));
    }
  }
}

void RobotIoDaemon::ethercat_cycle_handler(
    void* context, std::span<Cia402PdoView> pdos, const DomainHealth& domain,
    bool process_data_valid) noexcept {
  auto& daemon = *static_cast<RobotIoDaemon*>(context);
  if (daemon.owns_cycle()) {
    daemon.process_device_cycle_owned(pdos, domain, process_data_valid);
  }
}

bool RobotIoDaemon::owns_cycle() const noexcept {
  return cycle_owner_token_.load(std::memory_order_acquire) ==
         current_thread_token();
}

void RobotIoDaemon::cycle_owned(const CycleContext& context) noexcept {
  static_cast<void>(refresh_commands_owned());
  if (ethercat_master_ != nullptr) {
    ethercat_master_->cycle(context);
  } else {
    const auto mask = axis_count_ == 0U
                          ? std::uint16_t{0U}
                          : static_cast<std::uint16_t>((1U << axis_count_) - 1U);
    process_device_cycle_owned(
        std::span<Cia402PdoView>(standalone_pdos_.data(), axis_count_),
        DomainHealth{1U, 1U, true, true, true, 0, mask}, true);
  }
  loop_.observe_finish(clock_->now_ns());
}

void RobotIoDaemon::cycle() noexcept {
  CycleOwnerGuard owner{cycle_owner_token_};
  if (!owner) {
    return;
  }
  const auto now = clock_->now_ns();
  cycle_owned(CycleContext{
      cycle_sequence_.load(std::memory_order_acquire) + 1U,
      std::chrono::steady_clock::time_point{std::chrono::nanoseconds{now}},
      loop_.config().period});
}

void RobotIoDaemon::run() noexcept {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }
  CycleOwnerGuard owner{cycle_owner_token_};
  if (!owner || !running_.load(std::memory_order_acquire)) {
    return;
  }
  auto realtime = loop_.prepare();
  if (!realtime.has_value()) {
    realtime_guarantee_.store(false, std::memory_order_release);
    timing_fault_.store(true, std::memory_order_release);
    static_cast<void>(request_stop());
  } else {
    realtime_guarantee_.store(realtime.value().realtime_guarantee,
                              std::memory_order_release);
  }
  bool sleep_failed = !realtime.has_value();
  while (running_.load(std::memory_order_acquire) &&
         !shutdown_complete_.load(std::memory_order_acquire)) {
    if (signal_stop_requested != 0) {
      stop_requested_.store(true, std::memory_order_release);
      accepting_commands_.store(false, std::memory_order_release);
    }
    if (!sleep_failed) {
      const auto release = loop_.wait_next();
      if (release.sleep_error == 0) {
        cycle_owned(release.context);
        continue;
      }
      sleep_failed = true;
      timing_fault_.store(true, std::memory_order_release);
      sleep_error_.store(release.sleep_error, std::memory_order_release);
      realtime_guarantee_.store(false, std::memory_order_release);
      static_cast<void>(request_stop());
    }
    const auto now = clock_->now_ns();
    cycle_owned(CycleContext{
        cycle_sequence_.load(std::memory_order_acquire) + 1U,
        std::chrono::steady_clock::time_point{std::chrono::nanoseconds{now}},
        loop_.config().period});
  }
  loop_.release();
  stop_transports();
}

Result<void> RobotIoDaemon::request_stop() {
  if (!configured_) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "robot I/O daemon is not configured"});
  }
  stop_requested_.store(true, std::memory_order_release);
  accepting_commands_.store(false, std::memory_order_release);
  return Result<void>::success();
}

Result<void> RobotIoDaemon::poll_control() {
  if (!ipc_.has_value()) {
    return Result<void>::success();
  }
  auto peer = ipc_->check_peer();
  if (!peer.has_value()) {
    static_cast<void>(request_stop());
  }
  return peer;
}

void RobotIoDaemon::stop_transports() noexcept {
  serial_devices_.stop(scheduler_);
  if (ethercat_master_ != nullptr && master_handler_bound_) {
    ethercat_master_->close();
    static_cast<void>(ethercat_master_->clear_supervised_cycle_handler(this));
    master_handler_bound_ = false;
  }
  if (ethercat_master_ != nullptr) {
    if (master_registered_) {
      static_cast<void>(scheduler_.remove(*ethercat_master_));
      master_registered_ = false;
    }
  }
  running_.store(false, std::memory_order_release);
}

DaemonHealth RobotIoDaemon::health() const noexcept {
  const auto metrics = loop_.metrics();
  return DaemonHealth{
      configured_,
      running_.load(std::memory_order_acquire),
      accepting_commands_.load(std::memory_order_acquire),
      realtime_guarantee_.load(std::memory_order_acquire),
      stop_requested_.load(std::memory_order_acquire),
      shutdown_complete_.load(std::memory_order_acquire),
      process_data_valid_.load(std::memory_order_acquire),
      axis_count_,
      working_counter_.load(std::memory_order_acquire),
      expected_working_counter_.load(std::memory_order_acquire),
      consecutive_wkc_failures_.load(std::memory_order_acquire),
      safety_flags_.load(std::memory_order_acquire),
      cycle_sequence_.load(std::memory_order_acquire),
      last_command_sequence_.load(std::memory_order_acquire),
      last_command_timestamp_ns_.load(std::memory_order_acquire),
      feedback_timestamp_ns_.load(std::memory_order_acquire),
      dc_deviation_ns_.load(std::memory_order_acquire),
      metrics.deadline_misses,
      metrics.skipped_releases,
      metrics.sleep_failures,
      metrics.actual_period_ns,
      metrics.wake_latency_ns,
      metrics.maximum_wake_latency_ns,
      metrics.execution_time_ns,
      metrics.maximum_execution_time_ns,
      timing_fault_.load(std::memory_order_acquire),
      sleep_error_.load(std::memory_order_acquire),
      serial_servo_count_.load(std::memory_order_acquire),
      serial_fault_count_.load(std::memory_order_acquire),
      serial_safety_flags_.load(std::memory_order_acquire)};
}

DaemonAxisSnapshot RobotIoDaemon::axis_snapshot() const noexcept {
  auto snapshot = axis_publication_.read();
  if (snapshot.has_value()) {
    return *snapshot;
  }
  DaemonAxisSnapshot fallback{};
  fallback.axis_count = axis_count_;
  return fallback;
}

AxisRequest RobotIoDaemon::axis_request(std::size_t axis_index) const noexcept {
  const auto snapshot = axis_snapshot();
  return axis_index < snapshot.axis_count ? snapshot.requests[axis_index]
                                          : AxisRequest::disable;
}

AxisFeedback RobotIoDaemon::feedback(std::size_t axis_index) const noexcept {
  const auto snapshot = axis_snapshot();
  return axis_index < snapshot.axis_count ? snapshot.feedback[axis_index]
                                          : AxisFeedback{};
}

St3215CommandAcceptance RobotIoDaemon::stage_servo_command(
    std::size_t servo_index,
    const St3215ServoCommand& command) noexcept {
  if (!accepting_commands_.load(std::memory_order_acquire)) {
    return St3215CommandAcceptance::rejected;
  }
  return serial_devices_.stage_command(servo_index, command);
}

St3215ServoFeedback RobotIoDaemon::servo_feedback(
    std::size_t servo_index) const noexcept {
  return serial_devices_.feedback(servo_index, clock_->now_ns());
}

St3215RegistrySafetySnapshot RobotIoDaemon::serial_safety_snapshot()
    const noexcept {
  return serial_devices_.safety_snapshot(clock_->now_ns());
}

}  // namespace policy_runtime
