#include "policy_runtime/robot/devices/st3215/servo.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include "policy_runtime/protocol/st3215/protocol.hpp"
#include "policy_runtime/transport/serial/serial_transport.hpp"

namespace policy_runtime {
namespace {

std::int64_t steady_now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool same_command(const St3215ServoCommand& left,
                  const St3215ServoCommand& right) noexcept {
  return left.sequence == right.sequence &&
         left.timestamp_ns == right.timestamp_ns &&
         left.target_position_rad == right.target_position_rad &&
         left.enabled == right.enabled &&
         left.emergency_stop == right.emergency_stop &&
         left.reserved == right.reserved;
}

std::uint32_t error_flag(ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::timeout:
      return kSt3215FeedbackTimeout;
    case ErrorCode::protocol:
    case ErrorCode::invalid_argument:
      return kSt3215FeedbackProtocol;
    case ErrorCode::io:
    case ErrorCode::unavailable:
    case ErrorCode::internal:
      return kSt3215FeedbackIo;
  }
  return kSt3215FeedbackIo;
}

}  // namespace

St3215Servo::St3215Servo(St3215ServoConfig config)
    : config_(std::move(config)) {
  feedback_.publish(St3215ServoFeedback{});
}

St3215CommandAcceptance St3215Servo::stage_command(
    const St3215ServoCommand& command) noexcept {
  if (command_gate_.test_and_set(std::memory_order_acquire)) {
    return St3215CommandAcceptance::rejected;
  }
  const auto release_gate = [this] {
    command_gate_.clear(std::memory_order_release);
  };
  if (command.timestamp_ns < 0 || !std::isfinite(command.target_position_rad) ||
      std::any_of(command.reserved.begin(), command.reserved.end(),
                  [](std::byte value) { return value != std::byte{}; })) {
    release_gate();
    return St3215CommandAcceptance::rejected;
  }
  const auto now_ns = steady_now_ns();
  if (config_.command_timeout.count() > 0 &&
      (command.timestamp_ns > now_ns + config_.maximum_command_future.count() ||
       now_ns - command.timestamp_ns > config_.command_timeout.count())) {
    release_gate();
    return St3215CommandAcceptance::rejected;
  }
  if (has_command_ &&
      (command.sequence < last_command_sequence_ ||
       command.timestamp_ns < last_command_timestamp_ns_)) {
    release_gate();
    return St3215CommandAcceptance::rejected;
  }
  if (has_command_ && command.sequence == last_command_sequence_) {
    const auto result = same_command(command, last_command_)
                            ? St3215CommandAcceptance::duplicate
                            : St3215CommandAcceptance::rejected;
    release_gate();
    return result;
  }
  last_command_ = command;
  last_command_sequence_ = command.sequence;
  last_command_timestamp_ns_ = command.timestamp_ns;
  has_command_ = true;
  commands_.publish(StagedCommand{++next_publication_, command});
  release_gate();
  return St3215CommandAcceptance::accepted;
}

St3215ServoFeedback St3215Servo::feedback(std::int64_t now_ns) const noexcept {
  auto value = feedback_.read().value_or(St3215ServoFeedback{});
  if (now_ns > 0 && config_.feedback_timeout.count() > 0 &&
      (value.timestamp_ns <= 0 || now_ns < value.timestamp_ns ||
       now_ns - value.timestamp_ns >
           config_.feedback_timeout.count() * 1'000'000LL)) {
    value.flags |= kSt3215FeedbackStale;
    value.flags &= ~kSt3215FeedbackValid;
  }
  return value;
}

const St3215ServoConfig& St3215Servo::config() const noexcept {
  return config_;
}

bool St3215Servo::atomics_are_lock_free() const noexcept {
  return commands_.atomics_are_lock_free() &&
         feedback_.atomics_are_lock_free();
}

bool St3215Servo::read_staged(std::uint64_t consumed_publication,
                              StagedCommand& output) const noexcept {
  const auto staged = commands_.read();
  if (!staged.has_value() || staged->publication == consumed_publication) {
    return false;
  }
  output = *staged;
  return true;
}

void St3215Servo::publish_feedback(
    const St3215ServoFeedback& feedback) noexcept {
  feedback_.publish(feedback);
}

St3215Bus::St3215Bus(std::shared_ptr<FrameTransport> transport,
                     St3215BusOptions options)
    : transport_(std::move(transport)), options_(options) {}

St3215Bus::~St3215Bus() { stop(); }

Result<void> St3215Bus::add_servo(std::shared_ptr<St3215Servo> servo) {
  std::scoped_lock lock(lifecycle_mutex_);
  if (running_.load(std::memory_order_acquire) || servo == nullptr ||
      servo->config().name.empty() || servo->config().device_id == kSt3215BroadcastId ||
      servo->config().max_position_units == 0U ||
      servo->config().feedback_timeout.count() <= 0 ||
      servo->config().feedback_timeout.count() >
          std::numeric_limits<std::int64_t>::max() / 1'000'000LL) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "invalid ST3215 servo configuration"});
  }
  const auto duplicate = std::find_if(
      servos_.begin(), servos_.end(), [&](const ServoRuntime& existing) {
        return existing.servo->config().device_id == servo->config().device_id ||
               existing.servo->config().name == servo->config().name;
      });
  if (duplicate != servos_.end()) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "duplicate ST3215 servo on shared bus"});
  }
  servos_.push_back(ServoRuntime{std::move(servo)});
  return Result<void>::success();
}

Result<void> St3215Bus::start(TransportScheduler& scheduler) {
  std::scoped_lock lock(lifecycle_mutex_);
  if (running_.load(std::memory_order_acquire) || transport_ == nullptr ||
      servos_.empty() || options_.service_period.count() <= 0 ||
      options_.service_period > std::chrono::seconds{1}) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "ST3215 bus is not startable"});
  }
  for (const auto& runtime : servos_) {
    if (!runtime.servo->atomics_are_lock_free()) {
      return Result<void>::failure(
          {ErrorCode::unavailable, "ST3215 snapshots are not lock-free"});
    }
  }
  auto scheduled = scheduler.add(*transport_);
  if (!scheduled.has_value()) {
    return scheduled;
  }
  auto opened = transport_->open();
  if (!opened.has_value()) {
    static_cast<void>(scheduler.remove(*transport_));
    return opened;
  }
  scheduler_ = &scheduler;
  health_.store(TransportHealth::healthy, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  try {
    executor_ = std::jthread(
        [this](std::stop_token stop_token) { executor_main(stop_token); });
  } catch (const std::exception& error) {
    running_.store(false, std::memory_order_release);
    transport_->close();
    static_cast<void>(scheduler.remove(*transport_));
    scheduler_ = nullptr;
    return Result<void>::failure(
        {ErrorCode::unavailable,
         std::string("failed to start ST3215 serial executor: ") + error.what()});
  }
  return Result<void>::success();
}

void St3215Bus::stop(TransportScheduler& scheduler) noexcept {
  {
    std::scoped_lock lock(lifecycle_mutex_);
    if (scheduler_ != nullptr && scheduler_ != &scheduler) {
      return;
    }
  }
  stop();
}

void St3215Bus::stop() noexcept {
  TransportScheduler* scheduler{};
  {
    std::scoped_lock lock(lifecycle_mutex_);
    if (!running_.exchange(false, std::memory_order_acq_rel) &&
        scheduler_ == nullptr) {
      return;
    }
    scheduler = scheduler_;
  }
  if (executor_.joinable()) {
    executor_.request_stop();
    transport_->request_stop();
    executor_wake_.notify_all();
    executor_.join();
  }
  std::scoped_lock lock(lifecycle_mutex_);
  transport_->close();
  if (scheduler != nullptr) {
    static_cast<void>(scheduler->remove(*transport_));
  }
  scheduler_ = nullptr;
  health_.store(TransportHealth::failed, std::memory_order_release);
}

bool St3215Bus::running() const noexcept {
  return running_.load(std::memory_order_acquire);
}

std::size_t St3215Bus::servo_count() const noexcept { return servos_.size(); }

TransportHealth St3215Bus::health() const noexcept {
  return health_.load(std::memory_order_acquire);
}

std::shared_ptr<FrameTransport> St3215Bus::transport() const noexcept {
  return transport_;
}

void St3215Bus::executor_main(std::stop_token stop_token) noexcept {
  auto next_release = std::chrono::steady_clock::now();
  std::size_t cursor{};
  while (!stop_token.stop_requested() &&
         running_.load(std::memory_order_acquire)) {
    next_release += options_.service_period;
    const CycleContext context{
        ++cycle_sequence_, next_release, options_.service_period};
    try {
      service_one(servos_[cursor], context);
    } catch (...) {
      auto feedback = failed_feedback(
          servos_[cursor], kSt3215FeedbackIo, steady_now_ns());
      servos_[cursor].servo->publish_feedback(feedback);
      health_.store(TransportHealth::failed, std::memory_order_release);
    }
    cursor = (cursor + 1U) % servos_.size();
    std::unique_lock wait_lock(executor_wait_mutex_);
    static_cast<void>(executor_wake_.wait_until(
        wait_lock, stop_token, next_release,
        [this] { return !running_.load(std::memory_order_acquire); }));
  }
}

void St3215Bus::service_one(ServoRuntime& runtime,
                            const CycleContext& context) {
  const auto now_ns = steady_now_ns();
  St3215Servo::StagedCommand staged;
  if (runtime.servo->read_staged(runtime.consumed_publication, staged)) {
    runtime.consumed_publication = staged.publication;
    runtime.command = staged.command;
    runtime.has_command = true;
  }

  if (runtime.has_command && runtime.servo->config().command_timeout.count() > 0 &&
      (now_ns < runtime.command.timestamp_ns ||
       now_ns - runtime.command.timestamp_ns >
           runtime.servo->config().command_timeout.count())) {
    runtime.has_command = false;
    runtime.command.enabled = false;
  }

  if (runtime.has_command && runtime.command.enabled &&
      !runtime.command.emergency_stop) {
    auto position = radians_to_position_units(
        runtime.command.target_position_rad,
        runtime.servo->config().max_position_units);
    if (!position.has_value()) {
      runtime.servo->publish_feedback(failed_feedback(
          runtime, kSt3215FeedbackProtocol, now_ns));
      return;
    }
    const auto goal = St3215Protocol::goal_position_command(
        runtime.servo->config().device_id, position.value(),
        runtime.servo->config().time_units,
        runtime.servo->config().speed_units);
    auto acknowledged = transact(runtime, goal, false, context);
    if (!acknowledged.has_value()) {
      runtime.servo->publish_feedback(failed_feedback(
          runtime, error_flag(acknowledged.error().code), now_ns));
      return;
    }
    if ((acknowledged.value().flags & kSt3215FeedbackDeviceError) != 0U) {
      runtime.servo->publish_feedback(acknowledged.value());
      return;
    }
  }

  auto present = transact(
      runtime,
      St3215Protocol::read_present_position_command(
          runtime.servo->config().device_id),
      true, context);
  if (!present.has_value()) {
    runtime.servo->publish_feedback(failed_feedback(
        runtime, error_flag(present.error().code), now_ns));
    return;
  }
  auto feedback = present.value();
  if (!runtime.has_command || !runtime.command.enabled ||
      runtime.command.emergency_stop) {
    feedback.flags |= kSt3215FeedbackDisabled;
  }
  runtime.servo->publish_feedback(feedback);
  health_.store(transport_->health(), std::memory_order_release);
}

Result<St3215ServoFeedback> St3215Bus::transact(
    ServoRuntime& runtime, std::span<const std::byte> frame,
    bool position_response, const CycleContext& context) {
  auto staged = transport_->write(0U, frame);
  if (!staged.has_value()) {
    return Result<St3215ServoFeedback>::failure(staged.error());
  }
  transport_->cycle(context);
  Error last_error{ErrorCode::timeout, "ST3215 response is unavailable"};
  for (unsigned attempt = 0; attempt < 8U; ++attempt) {
    std::array<std::byte, kSt3215MaximumFrameSize> response{};
    auto received = transport_->read(0U, response);
    if (!received.has_value()) {
      last_error = received.error();
      break;
    }
    auto status = St3215Protocol::parse_status(
        std::span<const std::byte>(response.data(), received.value()));
    if (!status.has_value()) {
      last_error = status.error();
      transport_->cycle(context);
      continue;
    }
    if (status.value().device_id != runtime.servo->config().device_id) {
      last_error = {ErrorCode::protocol,
                    "ST3215 response device ID does not match request"};
      transport_->cycle(context);
      continue;
    }
    if (status.value().error == 0U &&
        ((position_response && status.value().parameters.size() < 2U) ||
         (!position_response && !status.value().parameters.empty()))) {
      last_error = {ErrorCode::protocol,
                    "ST3215 response shape does not match request"};
      transport_->cycle(context);
      continue;
    }
    St3215ServoFeedback feedback = runtime.servo->feedback();
    feedback.feedback_sequence = ++runtime.feedback_sequence;
    feedback.command_sequence =
        runtime.has_command ? runtime.command.sequence : 0U;
    feedback.timestamp_ns = steady_now_ns();
    feedback.status_error = status.value().error;
    feedback.transport_health =
        static_cast<std::uint32_t>(transport_->health());
    feedback.flags = status.value().error == 0U
                         ? kSt3215FeedbackValid
                         : kSt3215FeedbackStale |
                               kSt3215FeedbackDeviceError;
    if (const auto serial =
            std::dynamic_pointer_cast<SerialTransport>(transport_)) {
      feedback.timeout_count = serial->timeout_count();
      feedback.io_error_count = serial->io_error_count();
    }
    if (position_response && status.value().error == 0U) {
      auto raw = read_st3215_u16_le(status.value().parameters);
      if (!raw.has_value()) {
        return Result<St3215ServoFeedback>::failure(raw.error());
      }
      auto radians = position_units_to_radians(
          raw.value(), runtime.servo->config().max_position_units);
      if (!radians.has_value()) {
        return Result<St3215ServoFeedback>::failure(radians.error());
      }
      feedback.raw_position = raw.value();
      feedback.position_rad = radians.value();
    }
    return Result<St3215ServoFeedback>::success(feedback);
  }
  return Result<St3215ServoFeedback>::failure(std::move(last_error));
}

St3215ServoFeedback St3215Bus::failed_feedback(
    ServoRuntime& runtime, std::uint32_t flags,
    std::int64_t now_ns) noexcept {
  auto feedback = runtime.servo->feedback();
  feedback.feedback_sequence = ++runtime.feedback_sequence;
  feedback.command_sequence =
      runtime.has_command ? runtime.command.sequence : 0U;
  feedback.timestamp_ns = now_ns;
  feedback.flags &= ~kSt3215FeedbackValid;
  feedback.flags |= kSt3215FeedbackStale | flags;
  feedback.transport_health =
      static_cast<std::uint32_t>(transport_->health());
  if (const auto serial = std::dynamic_pointer_cast<SerialTransport>(transport_)) {
    feedback.timeout_count = serial->timeout_count();
    feedback.io_error_count = serial->io_error_count();
  }
  health_.store(transport_->health(), std::memory_order_release);
  return feedback;
}

St3215DeviceRegistry::~St3215DeviceRegistry() { stop(); }

Result<void> St3215DeviceRegistry::configure(
    std::span<const profiles::St3215ServoProfile> profiles) {
  if (configured_ || running_.load(std::memory_order_acquire) ||
      profiles.size() > kMaximumSt3215Servos) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "ST3215 device registry is already configured or oversized"});
  }
  std::vector<BusEntry> buses;
  std::vector<std::shared_ptr<St3215Servo>> servos;
  std::vector<std::string> safety_groups;
  buses.reserve(profiles.size());
  servos.reserve(profiles.size());
  safety_groups.reserve(profiles.size());
  for (const auto& profile : profiles) {
    auto bus = std::find_if(
        buses.begin(), buses.end(), [&](const BusEntry& candidate) {
          return candidate.serial.path == profile.serial.path;
        });
    if (bus == buses.end()) {
      auto transport = std::make_shared<SerialTransport>(SerialConfig{
          profile.serial.path,
          profile.serial.baud_rate,
          profile.serial.read_buffer_size,
          profile.serial.maximum_frame_size,
          profile.serial.read_timeout,
          profile.serial.write_timeout});
      buses.push_back(BusEntry{
          profile.serial,
          std::make_unique<St3215Bus>(
              std::move(transport),
              St3215BusOptions{profile.serial.service_period})});
      bus = std::prev(buses.end());
    } else if (bus->serial.baud_rate != profile.serial.baud_rate ||
               bus->serial.read_buffer_size !=
                   profile.serial.read_buffer_size ||
               bus->serial.maximum_frame_size !=
                   profile.serial.maximum_frame_size ||
               bus->serial.read_timeout != profile.serial.read_timeout ||
               bus->serial.write_timeout != profile.serial.write_timeout ||
               bus->serial.service_period != profile.serial.service_period) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument,
           "inconsistent ST3215 registry serial configuration"});
    }
    auto servo = std::make_shared<St3215Servo>(St3215ServoConfig{
        profile.sensor_name,
        profile.device_id,
        profile.servo_id,
        profile.max_position_units,
        profile.speed_units,
        profile.time_units,
        profile.feedback_timeout,
        profile.command_timeout,
        profile.maximum_command_future});
    auto added = bus->bus->add_servo(servo);
    if (!added.has_value()) {
      return added;
    }
    servos.push_back(std::move(servo));
    safety_groups.push_back(profile.safety_group);
  }
  buses_ = std::move(buses);
  servos_ = std::move(servos);
  safety_groups_ = std::move(safety_groups);
  configured_ = true;
  return Result<void>::success();
}

Result<void> St3215DeviceRegistry::start(TransportScheduler& scheduler) {
  if (!configured_ || running_.load(std::memory_order_acquire) ||
      scheduler_ != nullptr) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "ST3215 registry is not startable"});
  }
  std::size_t started{};
  for (; started < buses_.size(); ++started) {
    auto result = buses_[started].bus->start(scheduler);
    if (!result.has_value()) {
      while (started > 0U) {
        buses_[--started].bus->stop(scheduler);
      }
      return result;
    }
  }
  scheduler_ = &scheduler;
  running_.store(true, std::memory_order_release);
  return Result<void>::success();
}

void St3215DeviceRegistry::stop(TransportScheduler& scheduler) noexcept {
  if (scheduler_ != nullptr && scheduler_ != &scheduler) {
    return;
  }
  stop();
}

void St3215DeviceRegistry::stop() noexcept {
  running_.store(false, std::memory_order_release);
  auto* scheduler = scheduler_;
  if (scheduler != nullptr) {
    for (auto bus = buses_.rbegin(); bus != buses_.rend(); ++bus) {
      bus->bus->stop(*scheduler);
    }
  } else {
    for (auto bus = buses_.rbegin(); bus != buses_.rend(); ++bus) {
      bus->bus->stop();
    }
  }
  scheduler_ = nullptr;
}

St3215CommandAcceptance St3215DeviceRegistry::stage_command(
    std::size_t servo_index, const St3215ServoCommand& command) noexcept {
  if (servo_index >= servos_.size()) {
    return St3215CommandAcceptance::rejected;
  }
  return servos_[servo_index]->stage_command(command);
}

St3215ServoFeedback St3215DeviceRegistry::feedback(
    std::size_t servo_index, std::int64_t now_ns) const noexcept {
  if (servo_index >= servos_.size()) {
    return St3215ServoFeedback{};
  }
  return servos_[servo_index]->feedback(now_ns);
}

St3215RegistrySafetySnapshot St3215DeviceRegistry::safety_snapshot(
    std::int64_t now_ns) const noexcept {
  St3215RegistrySafetySnapshot snapshot;
  snapshot.servo_count = static_cast<std::uint32_t>(servos_.size());
  constexpr std::uint32_t kFaultFlags =
      kSt3215FeedbackStale | kSt3215FeedbackTimeout |
      kSt3215FeedbackIo | kSt3215FeedbackProtocol |
      kSt3215FeedbackDeviceError;
  for (std::size_t index = 0; index < servos_.size(); ++index) {
    const auto value = servos_[index]->feedback(now_ns);
    snapshot.flags[index] = value.flags;
    snapshot.aggregate_flags |= value.flags;
    if ((value.flags & kFaultFlags) != 0U) {
      ++snapshot.fault_count;
      snapshot.fault_mask |= std::uint32_t{1U} << index;
    }
  }
  return snapshot;
}

St3215Servo* St3215DeviceRegistry::servo(
    std::size_t servo_index) const noexcept {
  return servo_index < servos_.size() ? servos_[servo_index].get() : nullptr;
}

std::size_t St3215DeviceRegistry::servo_count() const noexcept {
  return servos_.size();
}

std::size_t St3215DeviceRegistry::bus_count() const noexcept {
  return buses_.size();
}

bool St3215DeviceRegistry::configured() const noexcept { return configured_; }

bool St3215DeviceRegistry::running() const noexcept {
  return running_.load(std::memory_order_acquire);
}

bool St3215DeviceRegistry::atomics_are_lock_free() const noexcept {
  if (!running_.is_lock_free()) {
    return false;
  }
  return std::all_of(servos_.begin(), servos_.end(), [](const auto& servo) {
    return servo->atomics_are_lock_free();
  });
}

}  // namespace policy_runtime
