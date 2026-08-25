#include "policy_runtime/robot_io/safety_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "policy_runtime/robot/devices/cia402/axis.hpp"
#include "policy_runtime/protocol/cia402/units.hpp"

namespace policy_runtime {
namespace {

constexpr std::uint32_t kKnownCommandFlags =
    kAxisCommandEnable | kAxisCommandDisable | kAxisCommandQuickStop |
    kAxisCommandFaultReset;

unsigned flag_count(std::uint32_t flags) noexcept {
  unsigned count{};
  while (flags != 0U) {
    count += flags & 1U;
    flags >>= 1U;
  }
  return count;
}

bool valid_options(const SafetyOptions& options) noexcept {
  return options.consecutive_wkc_limit > 0U &&
         options.maximum_fault_resets > 0U &&
         options.safe_stop_timeout.count() > 0 &&
         options.zero_velocity_threshold_counts >= 0 &&
         options.zero_velocity_confirmation_cycles > 0U;
}

}  // namespace

SafetySupervisor::SafetySupervisor(SafetyOptions options) noexcept
    : options_(options) {}

Result<void> SafetySupervisor::configure(
    std::span<const profiles::AxisConfig> axes) {
  if (configured_) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "safety supervisor is already configured"});
  }
  if (axes.size() > kRobotIoMaximumAxes || !valid_options(options_)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "invalid axis count or safety options"});
  }

  std::array<std::uint8_t, kRobotIoMaximumAxes> group_ids{};
  std::uint8_t next_group{};
  for (std::size_t axis = 0; axis < axes.size(); ++axis) {
    if (axes[axis].command_timeout.count() <= 0 ||
        axes[axis].command_timeout.count() >
            std::numeric_limits<std::int64_t>::max() / 1'000'000LL ||
        !std::isfinite(axes[axis].scale) || axes[axis].scale <= 0.0 ||
        !std::isfinite(axes[axis].minimum) ||
        !std::isfinite(axes[axis].maximum) ||
        axes[axis].minimum > axes[axis].maximum ||
        !std::isfinite(axes[axis].slew_limit) ||
        axes[axis].slew_limit <= 0.0 ||
        !std::isfinite(axes[axis].following_error_limit) ||
        axes[axis].following_error_limit <= 0.0) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "axis has invalid daemon safety limits"});
    }
    if (axes[axis].mode != profiles::Cia402Mode::csp &&
        axes[axis].mode != profiles::Cia402Mode::csv &&
        axes[axis].mode != profiles::Cia402Mode::cst) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "axis has invalid static CiA 402 mode"});
    }

    std::uint8_t group = next_group++;
    if (!axes[axis].safety_group.empty()) {
      for (std::size_t previous = 0; previous < axis; ++previous) {
        if (axes[previous].safety_group == axes[axis].safety_group) {
          group = group_ids[previous];
          --next_group;
          break;
        }
      }
    }
    group_ids[axis] = group;
    configs_[axis] = AxisSafetyConfig{
        axes[axis].command_timeout.count() * 1'000'000LL,
        axes[axis].mode,
        axes[axis].scale,
        axes[axis].minimum,
        axes[axis].maximum,
        axes[axis].slew_limit,
        axes[axis].following_error_limit,
        group};
    last_requests_[axis] = AxisRequest::disable;
  }
  axis_count_ = static_cast<std::uint32_t>(axes.size());
  configured_ = true;
  return Result<void>::success();
}

bool SafetySupervisor::command_equals(const AxisCommand& lhs,
                                      const AxisCommand& rhs) noexcept {
  return lhs.sequence == rhs.sequence &&
         lhs.timestamp_ns == rhs.timestamp_ns && lhs.target == rhs.target &&
         lhs.flags == rhs.flags && lhs.reserved == rhs.reserved;
}

bool SafetySupervisor::valid_snapshot(const Snapshot<AxisCommand>& snapshot,
                                      std::int64_t now_ns) const noexcept {
  if (!configured_ || snapshot.axis_count != axis_count_ ||
      snapshot.timestamp_ns < 0 || snapshot.timestamp_ns > now_ns) {
    return false;
  }
  for (std::size_t axis = 0; axis < axis_count_; ++axis) {
    const auto& command = snapshot.axes[axis];
    if (command.sequence != snapshot.sequence ||
        command.timestamp_ns != snapshot.timestamp_ns ||
        command.reserved != 0U || !std::isfinite(command.target) ||
        (command.flags & ~kKnownCommandFlags) != 0U ||
        flag_count(command.flags & kKnownCommandFlags) > 1U) {
      return false;
    }
  }
  return true;
}

CommandAcceptance SafetySupervisor::accept_commands(
    const Snapshot<AxisCommand>& snapshot, std::int64_t now_ns) noexcept {
  if (!valid_snapshot(snapshot, now_ns)) {
    commands_valid_ = false;
    return CommandAcceptance::rejected;
  }
  if (has_command_) {
    if (snapshot.sequence < last_command_sequence_ ||
        snapshot.timestamp_ns < last_command_timestamp_ns_) {
      commands_valid_ = false;
      return CommandAcceptance::rejected;
    }
    if (snapshot.sequence == last_command_sequence_) {
      bool same = snapshot.timestamp_ns == last_command_timestamp_ns_;
      for (std::size_t axis = 0; axis < axis_count_; ++axis) {
        same = same && command_equals(snapshot.axes[axis], commands_[axis]);
      }
      if (same) {
        return CommandAcceptance::duplicate;
      }
      commands_valid_ = false;
      return CommandAcceptance::rejected;
    }
  }

  for (std::size_t axis = 0; axis < axis_count_; ++axis) {
    commands_[axis] = snapshot.axes[axis];
  }
  last_command_sequence_ = snapshot.sequence;
  last_command_timestamp_ns_ = snapshot.timestamp_ns;
  has_command_ = true;
  commands_valid_ = true;
  return CommandAcceptance::accepted;
}

void SafetySupervisor::request_shutdown(std::int64_t now_ns) noexcept {
  if (!shutdown_requested_) {
    shutdown_requested_ = true;
    static_cast<void>(now_ns);
  }
}

AxisRequest SafetySupervisor::request_from_flags(std::uint32_t flags) noexcept {
  if ((flags & kAxisCommandQuickStop) != 0U) {
    return AxisRequest::quick_stop;
  }
  if ((flags & kAxisCommandFaultReset) != 0U) {
    return AxisRequest::fault_reset;
  }
  if ((flags & kAxisCommandEnable) != 0U) {
    return AxisRequest::enable;
  }
  return AxisRequest::disable;
}

std::uint32_t SafetySupervisor::flags_for_request(AxisRequest request) noexcept {
  switch (request) {
    case AxisRequest::enable:
      return kAxisCommandEnable;
    case AxisRequest::disable:
      return kAxisCommandDisable;
    case AxisRequest::quick_stop:
      return kAxisCommandQuickStop;
    case AxisRequest::fault_reset:
      return kAxisCommandFaultReset;
  }
  return kAxisCommandQuickStop;
}

bool SafetySupervisor::axis_operational(const SafetyBusState& bus,
                                        std::size_t axis_index) const noexcept {
  if (bus.all_slaves_operational) {
    return true;
  }
  return axis_index < 16U &&
         (bus.operational_axes_mask & (std::uint16_t{1U} << axis_index)) != 0U;
}

SafetySupervisor::TargetEvaluation SafetySupervisor::evaluate_target(
    const AxisSafetyConfig& config, AxisSafetyState& state,
    const AxisCommand& command, const Cia402PdoView& pdo) noexcept {
  TargetEvaluation evaluation{};
  std::optional<double> actual;
  switch (config.mode) {
    case profiles::Cia402Mode::csp:
      actual = device_units_to_engineering(pdo.actual_position, config.scale);
      break;
    case profiles::Cia402Mode::csv:
      actual = device_units_to_engineering(pdo.actual_velocity, config.scale);
      break;
    case profiles::Cia402Mode::cst:
      actual = device_units_to_engineering(pdo.actual_torque, config.scale);
      break;
  }
  evaluation.actual = actual.value_or(0.0);
  evaluation.staged = evaluation.actual;
  if (!state.has_last_target) {
    state.last_target = evaluation.actual;
    state.has_last_target = true;
  }
  if (request_from_flags(command.flags) != AxisRequest::enable || !actual) {
    return evaluation;
  }

  evaluation.staged =
      std::clamp(command.target, config.minimum, config.maximum);
  evaluation.staged =
      std::clamp(evaluation.staged, state.last_target - config.slew_limit,
                 state.last_target + config.slew_limit);
  const auto drive_state = Cia402StateMachine::decode(pdo.status_word);
  const bool following_error_active =
      drive_state == DriveState::switched_on ||
      drive_state == DriveState::operation_enabled ||
      drive_state == DriveState::quick_stop_active;
  evaluation.following_error =
      following_error_active &&
      std::abs(evaluation.staged - evaluation.actual) >
          config.following_error_limit;
  return evaluation;
}

SafetyEvaluation SafetySupervisor::evaluate(
    const SafetyBusState& bus, std::span<const Cia402PdoView> pdos,
    std::int64_t now_ns) noexcept {
  SafetyEvaluation output{};
  output.axis_count = axis_count_;
  if (!configured_ || pdos.size() != axis_count_) {
    for (std::size_t axis = 0; axis < axis_count_; ++axis) {
      output.requests[axis] = AxisRequest::quick_stop;
      output.commands[axis].flags = kAxisCommandQuickStop;
      output.feedback_flags[axis] = kAxisFeedbackSafetyProcessData;
      last_requests_[axis] = AxisRequest::quick_stop;
    }
    return output;
  }

  const bool counter_bad =
      !bus.working_counter_complete ||
      (bus.expected_working_counter_known &&
       bus.working_counter != bus.expected_working_counter);
  if (counter_bad) {
    if (consecutive_wkc_failures_ < std::numeric_limits<std::uint32_t>::max()) {
      ++consecutive_wkc_failures_;
    }
  } else {
    consecutive_wkc_failures_ = 0U;
  }
  const bool wkc_limit_reached =
      consecutive_wkc_failures_ >= options_.consecutive_wkc_limit;
  const bool invalid_process_data =
      !bus.process_data_valid && !counter_bad && bus.link_up;

  std::array<std::uint32_t, kRobotIoMaximumAxes> reasons{};
  std::array<bool, kRobotIoMaximumAxes> triggers{};
  std::array<bool, kRobotIoMaximumAxes> fault_reset_candidates{};
  std::array<TargetEvaluation, kRobotIoMaximumAxes> targets{};
  for (std::size_t axis = 0; axis < axis_count_; ++axis) {
    const auto& config = configs_[axis];
    const auto request =
        has_command_ ? request_from_flags(commands_[axis].flags)
                     : AxisRequest::disable;
    if (!has_command_ || now_ns < last_command_timestamp_ns_ ||
        now_ns - last_command_timestamp_ns_ > config.command_timeout_ns) {
      reasons[axis] |= kAxisFeedbackSafetyTimeout;
    }
    if (!commands_valid_) {
      reasons[axis] |= kAxisFeedbackSafetyInvalidCommand;
    }
    if ((bus.external_stop_axes_mask &
         (std::uint16_t{1U} << axis)) != 0U) {
      reasons[axis] |= kAxisFeedbackSafetySerial;
    }
    if (!bus.link_up) {
      reasons[axis] |= kAxisFeedbackSafetyLink;
    }
    if (wkc_limit_reached) {
      reasons[axis] |= kAxisFeedbackSafetyWkc;
    }
    if (invalid_process_data) {
      reasons[axis] |= kAxisFeedbackSafetyProcessData;
    }
    if (!axis_operational(bus, axis)) {
      reasons[axis] |= kAxisFeedbackSafetySlave;
    }
    if (pdos[axis].mode_display != static_cast<std::int8_t>(config.mode)) {
      reasons[axis] |= kAxisFeedbackSafetyModeMismatch;
    }
    targets[axis] =
        evaluate_target(config, states_[axis], commands_[axis], pdos[axis]);
    if (targets[axis].following_error) {
      reasons[axis] |= kAxisFeedbackSafetyFollowingError;
    }
    const auto drive_state = Cia402StateMachine::decode(pdos[axis].status_word);
    const bool drive_fault = drive_state == DriveState::fault ||
                             drive_state == DriveState::fault_reaction_active;
    const bool resettable_fault = drive_state == DriveState::fault;
    const bool fault_clear_observed =
        !drive_fault && drive_state != DriveState::unknown && bus.link_up &&
        bus.process_data_valid && axis_operational(bus, axis);
    if (fault_clear_observed) {
      states_[axis].fault_episode_active = false;
      states_[axis].fault_reset_edges = 0U;
      states_[axis].fault_reset_output_active = false;
    } else if (!states_[axis].fault_episode_active) {
      states_[axis].fault_episode_active = true;
      states_[axis].fault_reset_edges = 0U;
      states_[axis].fault_reset_output_active = false;
    }
    const bool new_reset_edge =
        resettable_fault && request == AxisRequest::fault_reset &&
        !states_[axis].fault_reset_output_active;
    const bool reset_budget_available =
        !new_reset_edge ||
        states_[axis].fault_reset_edges < options_.maximum_fault_resets;
    if (drive_fault &&
        (!resettable_fault || request != AxisRequest::fault_reset ||
         !reset_budget_available)) {
      reasons[axis] |= kAxisFeedbackSafetyDriveFault;
      if (request == AxisRequest::fault_reset && !reset_budget_available) {
        reasons[axis] |= kAxisFeedbackSafetyFaultResetLimit;
      }
    }
    if (shutdown_requested_) {
      reasons[axis] |= kAxisFeedbackSafetyShutdown;
    }
    fault_reset_candidates[axis] =
        resettable_fault && request == AxisRequest::fault_reset &&
        reset_budget_available && reasons[axis] == 0U;
    if (request == AxisRequest::quick_stop) {
      triggers[axis] = true;
    }
    triggers[axis] = triggers[axis] || reasons[axis] != 0U;
  }

  for (std::size_t source = 0; source < axis_count_; ++source) {
    if (!triggers[source]) {
      continue;
    }
    for (std::size_t member = 0; member < axis_count_; ++member) {
      if (member != source && configs_[member].group == configs_[source].group) {
        triggers[member] = true;
        reasons[member] |= kAxisFeedbackSafetyGroup;
      }
    }
  }

  // A reset pulse is permitted on the faulted member itself, including from a
  // stopped phase, but it does not release healthy peers in that safety group.
  // Multiple faulted members may reset together when each independently has
  // budget and no other safety reason.
  for (std::size_t source = 0; source < axis_count_; ++source) {
    if (!fault_reset_candidates[source]) {
      continue;
    }
    for (std::size_t member = 0; member < axis_count_; ++member) {
      if (member != source && !fault_reset_candidates[member] &&
          configs_[member].group == configs_[source].group) {
        triggers[member] = true;
        reasons[member] |= kAxisFeedbackSafetyGroup;
      }
    }
  }

  bool all_shutdown_axes_disabled = true;
  for (std::size_t axis = 0; axis < axis_count_; ++axis) {
    auto& state = states_[axis];
    const auto requested =
        has_command_ ? request_from_flags(commands_[axis].flags)
                     : AxisRequest::disable;
    const auto drive_state = Cia402StateMachine::decode(pdos[axis].status_word);
    const bool resettable_fault = drive_state == DriveState::fault;
    const bool reset_edge =
        resettable_fault && requested == AxisRequest::fault_reset &&
        !state.fault_reset_output_active;
    const bool fault_reset_allowed =
        resettable_fault && requested == AxisRequest::fault_reset &&
        reasons[axis] == 0U &&
        (!reset_edge ||
         state.fault_reset_edges < options_.maximum_fault_resets);
    bool entered_quick_stop = false;
    if (triggers[axis] && state.phase == StopPhase::running) {
      state.phase = StopPhase::quick_stop;
      state.stop_started_ns = now_ns;
      state.zero_velocity_cycles = 0U;
      state.stopped_command_sequence = last_command_sequence_;
      entered_quick_stop = true;
    }
    if (!triggers[axis] && state.phase == StopPhase::disabled && has_command_ &&
        last_command_sequence_ > state.stopped_command_sequence &&
        requested == AxisRequest::enable) {
      state.phase = StopPhase::running;
    }

    AxisRequest decision = requested;
    if (fault_reset_allowed) {
      decision = AxisRequest::fault_reset;
      if (reset_edge &&
          state.fault_reset_edges < std::numeric_limits<std::uint32_t>::max()) {
        ++state.fault_reset_edges;
      }
      state.fault_reset_output_active = true;
    } else if (state.phase == StopPhase::quick_stop) {
      state.fault_reset_output_active = false;
      decision = AxisRequest::quick_stop;
      if (!entered_quick_stop) {
        const auto magnitude = std::abs(static_cast<std::int64_t>(pdos[axis].actual_velocity));
        if (magnitude <= options_.zero_velocity_threshold_counts) {
          ++state.zero_velocity_cycles;
        } else {
          state.zero_velocity_cycles = 0U;
        }
        const bool confirmed_zero =
            state.zero_velocity_cycles >= options_.zero_velocity_confirmation_cycles;
        const bool timed_out =
            now_ns >= state.stop_started_ns &&
            now_ns - state.stop_started_ns >= options_.safe_stop_timeout.count();
        if (confirmed_zero || timed_out) {
          state.phase = StopPhase::disabled;
          decision = AxisRequest::disable;
        }
      }
    } else if (state.phase == StopPhase::disabled) {
      state.fault_reset_output_active = false;
      decision = AxisRequest::disable;
    } else {
      state.fault_reset_output_active = false;
      if (requested == AxisRequest::fault_reset) {
        // A reset request is meaningful only in Fault. Passing it through in
        // any other state would arm the CiA 402 edge latch without emitting
        // 0x0080, poisoning the first real reset opportunity.
        decision = AxisRequest::disable;
      }
    }

    output.commands[axis] = commands_[axis];
    output.commands[axis].sequence = last_command_sequence_;
    output.commands[axis].timestamp_ns = last_command_timestamp_ns_;
    output.commands[axis].flags = flags_for_request(decision);
    output.commands[axis].reserved = 0U;
    output.requests[axis] = decision;
    output.feedback_flags[axis] = reasons[axis];
    last_requests_[axis] = decision;
    state.last_target = decision == AxisRequest::enable
                            ? targets[axis].staged
                            : targets[axis].actual;
    all_shutdown_axes_disabled =
        all_shutdown_axes_disabled && decision == AxisRequest::disable;
  }

  shutdown_complete_ = shutdown_requested_ && all_shutdown_axes_disabled;
  output.shutdown_complete = shutdown_complete_;
  return output;
}

AxisRequest SafetySupervisor::axis_request(std::size_t axis_index) const noexcept {
  return axis_index < axis_count_ ? last_requests_[axis_index]
                                  : AxisRequest::disable;
}

std::uint64_t SafetySupervisor::last_command_sequence() const noexcept {
  return last_command_sequence_;
}

std::int64_t SafetySupervisor::last_command_timestamp_ns() const noexcept {
  return last_command_timestamp_ns_;
}

std::uint32_t SafetySupervisor::consecutive_wkc_failures() const noexcept {
  return consecutive_wkc_failures_;
}

bool SafetySupervisor::shutdown_complete() const noexcept {
  return shutdown_complete_;
}

}  // namespace policy_runtime
