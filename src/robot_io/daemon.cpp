#include "policy_runtime/robot_io/daemon.hpp"

#include <cerrno>
#include <exception>
#include <string>
#include <system_error>
#include <utility>

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
      domain.dc_deviation_ns};
}

}  // namespace

DaemonSignalLatch::~DaemonSignalLatch() {
  if (installed_) {
    static_cast<void>(sigaction(SIGTERM, &previous_, nullptr));
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
      safety_(safety_options) {}

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
  axes_ = std::move(configured_axes);
  axis_count_ = static_cast<std::uint32_t>(profile.axes.size());
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
      server.axis_count() != axis_count_) {
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
         feedback_timestamp_ns_.is_lock_free() && dc_deviation_ns_.is_lock_free();
}

Result<void> RobotIoDaemon::start() {
  if (!configured_ || running_.load(std::memory_order_acquire)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "robot I/O daemon is not startable"});
  }
  if (!health_atomics_are_lock_free()) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "daemon health atomics are not lock-free"});
  }
  auto realtime = loop_.prepare();
  if (!realtime.has_value()) {
    return Result<void>::failure(realtime.error());
  }
  realtime_guarantee_.store(realtime.value().realtime_guarantee,
                            std::memory_order_release);

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

  accepting_commands_.store(true, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  return Result<void>::success();
}

CommandAcceptance RobotIoDaemon::stage_commands(
    const Snapshot<AxisCommand>& snapshot) noexcept {
  if (!accepting_commands_.load(std::memory_order_acquire)) {
    return CommandAcceptance::rejected;
  }
  const auto accepted = safety_.accept_commands(snapshot, clock_->now_ns());
  if (accepted == CommandAcceptance::accepted) {
    last_command_sequence_.store(snapshot.sequence, std::memory_order_release);
    last_command_timestamp_ns_.store(snapshot.timestamp_ns,
                                     std::memory_order_release);
  }
  return accepted;
}

CommandAcceptance RobotIoDaemon::refresh_commands() noexcept {
  if (!ipc_.has_value()) {
    return CommandAcceptance::duplicate;
  }
  auto snapshot = ipc_->read_commands_realtime();
  if (!snapshot.has_value()) {
    if (snapshot.error().code == ErrorCode::protocol) {
      Snapshot<AxisCommand> invalid{};
      invalid.axis_count = kRobotIoMaximumAxes + 1U;
      static_cast<void>(safety_.accept_commands(invalid, clock_->now_ns()));
      return CommandAcceptance::rejected;
    }
    return CommandAcceptance::duplicate;
  }
  return stage_commands(snapshot.value());
}

void RobotIoDaemon::process_device_cycle(std::span<Cia402PdoView> pdos,
                                         const DomainHealth& domain,
                                         bool process_data_valid) noexcept {
  if (!running_.load(std::memory_order_acquire) || pdos.size() != axis_count_) {
    process_data_valid_.store(false, std::memory_order_release);
    return;
  }
  const auto now = clock_->now_ns();
  if (stop_requested_.load(std::memory_order_acquire)) {
    accepting_commands_.store(false, std::memory_order_release);
    safety_.request_shutdown(now);
  }
  const auto decisions =
      safety_.evaluate(safety_bus_state(domain, process_data_valid), pdos, now);

  std::uint32_t safety_flags{};
  for (std::size_t axis = 0; axis < axis_count_; ++axis) {
    auto value = axes_[axis]->cycle(decisions.commands[axis], pdos[axis]);
    value.sequence = safety_.last_command_sequence();
    value.timestamp_ns = now;
    value.flags |= decisions.feedback_flags[axis];
    feedback_[axis] = value;
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

  if (ipc_.has_value()) {
    static_cast<void>(ipc_->publish_feedback_realtime(
        std::span<const AxisFeedback>(feedback_.data(), axis_count_),
        safety_.last_command_sequence(), now));
  }
}

void RobotIoDaemon::ethercat_cycle_handler(
    void* context, std::span<Cia402PdoView> pdos, const DomainHealth& domain,
    bool process_data_valid) noexcept {
  static_cast<RobotIoDaemon*>(context)->process_device_cycle(
      pdos, domain, process_data_valid);
}

void RobotIoDaemon::cycle(const CycleContext& context) noexcept {
  static_cast<void>(refresh_commands());
  if (ethercat_master_ != nullptr) {
    ethercat_master_->cycle(context);
  } else {
    const auto mask = axis_count_ == 0U
                          ? std::uint16_t{0U}
                          : static_cast<std::uint16_t>((1U << axis_count_) - 1U);
    process_device_cycle(
        std::span<Cia402PdoView>(standalone_pdos_.data(), axis_count_),
        DomainHealth{1U, 1U, true, true, true, 0, mask}, true);
  }
  loop_.observe_finish(clock_->now_ns());
}

void RobotIoDaemon::cycle() noexcept {
  const auto now = clock_->now_ns();
  cycle(CycleContext{
      cycle_sequence_.load(std::memory_order_acquire) + 1U,
      std::chrono::steady_clock::time_point{std::chrono::nanoseconds{now}},
      loop_.config().period});
}

void RobotIoDaemon::run() noexcept {
  while (running_.load(std::memory_order_acquire) &&
         !shutdown_complete_.load(std::memory_order_acquire)) {
    if (signal_stop_requested != 0) {
      stop_requested_.store(true, std::memory_order_release);
      accepting_commands_.store(false, std::memory_order_release);
    }
    cycle(loop_.wait_next());
  }
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
      metrics.actual_period_ns,
      metrics.maximum_jitter_ns};
}

AxisRequest RobotIoDaemon::axis_request(std::size_t axis_index) const noexcept {
  return safety_.axis_request(axis_index);
}

AxisFeedback RobotIoDaemon::feedback(std::size_t axis_index) const noexcept {
  return axis_index < axis_count_ ? feedback_[axis_index] : AxisFeedback{};
}

}  // namespace policy_runtime
