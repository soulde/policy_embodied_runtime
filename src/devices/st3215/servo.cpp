#include "policy_runtime/devices/st3215/servo.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include "policy_runtime/protocol/st3215/protocol.hpp"
#include "policy_runtime/robot_io/daemon/transport_factory.hpp"
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
  if (!try_lock_command()) {
    return St3215CommandAcceptance::rejected;
  }
  const auto acceptance = inspect_command(command, steady_now_ns());
  if (acceptance == St3215CommandAcceptance::accepted) {
    commit_command(command);
  }
  unlock_command();
  return acceptance;
}

bool St3215Servo::try_lock_command() noexcept {
  return !command_gate_.test_and_set(std::memory_order_acquire);
}

void St3215Servo::unlock_command() noexcept {
  command_gate_.clear(std::memory_order_release);
}

St3215CommandAcceptance St3215Servo::inspect_command(
    const St3215ServoCommand& command, std::int64_t now_ns) const noexcept {
  if (command.timestamp_ns < 0 || !std::isfinite(command.target_position_rad) ||
      std::any_of(command.reserved.begin(), command.reserved.end(),
                  [](std::byte value) { return value != std::byte{}; })) {
    return St3215CommandAcceptance::rejected;
  }
  if (config_.command_timeout.count() > 0 &&
      (command.timestamp_ns > now_ns + config_.maximum_command_future.count() ||
       now_ns - command.timestamp_ns > config_.command_timeout.count())) {
    return St3215CommandAcceptance::rejected;
  }
  if (has_command_ &&
      (command.sequence < last_command_sequence_ ||
       command.timestamp_ns < last_command_timestamp_ns_)) {
    return St3215CommandAcceptance::rejected;
  }
  if (has_command_ && command.sequence == last_command_sequence_) {
    return same_command(command, last_command_)
               ? St3215CommandAcceptance::duplicate
               : St3215CommandAcceptance::rejected;
  }
  return St3215CommandAcceptance::accepted;
}

void St3215Servo::commit_command(
    const St3215ServoCommand& command) noexcept {
  last_command_ = command;
  last_command_sequence_ = command.sequence;
  last_command_timestamp_ns_ = command.timestamp_ns;
  has_command_ = true;
  commands_.publish(StagedCommand{++next_publication_, command});
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
  started_ns_ = steady_now_ns();
  running_.store(true, std::memory_order_release);
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
  if (fault_latched_) {
    return TransportHealth::failed;
  }
  return health_.load(std::memory_order_acquire);
}

std::shared_ptr<FrameTransport> St3215Bus::transport() const noexcept {
  return transport_;
}

void St3215Bus::cycle(const CycleContext& context) noexcept {
  if (!running_.load(std::memory_order_acquire) || servos_.empty()) {
    return;
  }
  const auto now_ns = steady_now_ns();
  ++cycle_sequence_;
  static_cast<void>(context);
  try {
    if (fault_latched_) {
      servos_[cursor_].servo->publish_feedback(failed_feedback(
          servos_[cursor_], kSt3215FeedbackIo, now_ns));
    } else {
      if (dispatched_) {
        poll_response(servos_[pending_index_], now_ns);
      }
      if (!fault_latched_) {
        cursor_ = dispatched_ ? (pending_index_ + 1U) % servos_.size() : 0U;
        refresh_staged(servos_[cursor_], now_ns);
        dispatch_request(servos_[cursor_], now_ns);
        pending_index_ = cursor_;
        dispatched_ = true;
      }
    }
  } catch (...) {
    fault_latched_ = true;
    health_.store(TransportHealth::failed, std::memory_order_release);
    servos_[cursor_].servo->publish_feedback(failed_feedback(
        servos_[cursor_], kSt3215FeedbackIo, now_ns));
  }
}

void St3215Bus::refresh_staged(ServoRuntime& runtime,
                               std::int64_t now_ns) noexcept {
  St3215Servo::StagedCommand staged;
  if (runtime.servo->read_staged(runtime.consumed_publication, staged)) {
    runtime.consumed_publication = staged.publication;
    runtime.command = staged.command;
    runtime.has_command = true;
    runtime.command_timed_out = false;
  }
  if (runtime.has_command && runtime.servo->config().command_timeout.count() > 0 &&
      (now_ns < runtime.command.timestamp_ns ||
       now_ns - runtime.command.timestamp_ns >
           runtime.servo->config().command_timeout.count())) {
    runtime.has_command = false;
    runtime.command.enabled = false;
    runtime.command_timed_out = true;
  }
}

void St3215Bus::poll_response(ServoRuntime& runtime,
                              std::int64_t now_ns) {
  refresh_staged(runtime, now_ns);
  auto response = consume_response(runtime, now_ns);
  if (response.has_value()) {
    auto feedback = response.value();
    if (runtime.command_timed_out) {
      feedback.command_sequence = runtime.command.sequence;
      feedback.flags &= ~kSt3215FeedbackValid;
      feedback.flags |= kSt3215FeedbackStale | kSt3215FeedbackTimeout;
    }
    if (!runtime.has_command || !runtime.command.enabled ||
        runtime.command.emergency_stop) {
      feedback.flags |= kSt3215FeedbackDisabled;
    }
    runtime.last_response_ns = now_ns;
    runtime.servo->publish_feedback(feedback);
  } else if (response.error().code != ErrorCode::timeout) {
    fault_latched_ = true;
    health_.store(TransportHealth::failed, std::memory_order_release);
    runtime.servo->publish_feedback(failed_feedback(
        runtime, error_flag(response.error().code), now_ns));
    return;
  } else {
    // No response bytes yet. Staleness is measured from the last accepted
    // response (or bus start) so a slow first answer is not misjudged, and
    // latches the bus once the servo feedback timeout expires.
    const auto reference =
        std::max(runtime.last_response_ns, started_ns_);
    const auto timeout_ns =
        runtime.servo->config().feedback_timeout.count() * 1'000'000LL;
    if (now_ns - reference > timeout_ns) {
      fault_latched_ = true;
      health_.store(TransportHealth::failed, std::memory_order_release);
      runtime.servo->publish_feedback(failed_feedback(
          runtime, kSt3215FeedbackTimeout, now_ns));
    }
  }
}

void St3215Bus::dispatch_request(ServoRuntime& runtime,
                                 std::int64_t now_ns) {
  auto sent = send_request(runtime);
  if (!sent.has_value()) {
    fault_latched_ = true;
    health_.store(TransportHealth::failed, std::memory_order_release);
    runtime.servo->publish_feedback(failed_feedback(
        runtime, error_flag(sent.error().code), now_ns));
    return;
  }
}

Result<St3215ServoFeedback> St3215Bus::consume_response(
    ServoRuntime& runtime, std::int64_t now_ns) {
  std::array<std::byte, kSt3215MaximumFrameSize> response{};
  auto received = transport_->read(0U, response);
  if (!received.has_value()) {
    return Result<St3215ServoFeedback>::failure(received.error());
  }
  auto status = St3215Protocol::parse_status(
      std::span<const std::byte>(response.data(), received.value()));
  if (!status.has_value()) {
    return Result<St3215ServoFeedback>::failure(status.error());
  }
  if (status.value().device_id != runtime.servo->config().device_id) {
    return Result<St3215ServoFeedback>::failure(
        {ErrorCode::protocol,
         "ST3215 response device ID does not match request"});
  }
  // Response latency varies between zero and one service period, so the
  // request a response answers cannot be inferred reliably; frames are
  // already checksum- and device-id-validated. A frame carrying parameters
  // is a position read answer, an empty one is a write acknowledgement.
  St3215ServoFeedback feedback = runtime.servo->feedback();
  feedback.feedback_sequence = ++runtime.feedback_sequence;
  feedback.command_sequence =
      runtime.has_command ? runtime.command.sequence : 0U;
  feedback.timestamp_ns = now_ns;
  feedback.status_error = status.value().error;
  feedback.transport_health = static_cast<std::uint32_t>(transport_->health());
  feedback.flags = status.value().error == 0U
                       ? kSt3215FeedbackValid
                       : kSt3215FeedbackStale | kSt3215FeedbackDeviceError;
  if (const auto serial =
          std::dynamic_pointer_cast<SerialTransport>(transport_)) {
    feedback.timeout_count = serial->timeout_count();
    feedback.io_error_count = serial->io_error_count();
  }
  if (status.value().error == 0U && status.value().parameters.size() >= 2U) {
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

Result<void> St3215Bus::send_request(ServoRuntime& runtime) {
  std::array<std::byte, kSt3215MaximumFrameSize> frame{};
  std::span<const std::byte> request;
  const bool command_active = runtime.has_command && runtime.command.enabled &&
                              !runtime.command.emergency_stop;
  // A command sequence is written to the device exactly once; every later
  // visit polls present position so feedback keeps flowing while moving.
  const bool send_goal =
      command_active && (!runtime.goal_dispatched ||
                         runtime.goal_sequence != runtime.command.sequence);
  if (send_goal) {
    auto position = radians_to_position_units(
        runtime.command.target_position_rad,
        runtime.servo->config().max_position_units);
    if (!position.has_value()) {
      return Result<void>::failure(position.error());
    }
    const auto encoded = St3215Protocol::goal_position_command(
        runtime.servo->config().device_id, position.value(),
        runtime.servo->config().time_units,
        runtime.servo->config().speed_units);
    std::copy(encoded.begin(), encoded.end(), frame.begin());
    request = std::span<const std::byte>(frame.data(), encoded.size());
    runtime.goal_dispatched = true;
    runtime.goal_sequence = runtime.command.sequence;
  } else {
    const auto encoded = St3215Protocol::read_present_position_command(
        runtime.servo->config().device_id);
    std::copy(encoded.begin(), encoded.end(), frame.begin());
    request = std::span<const std::byte>(frame.data(), encoded.size());
  }
  auto staged = transport_->write(0U, request);
  if (!staged.has_value()) {
    return staged;
  }
  transport_->cycle(CycleContext{cycle_sequence_, std::chrono::steady_clock::now(),
                                 options_.service_period});
  return Result<void>::success();
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
    // Daemon-driven hard-realtime cycles require zero-wait I/O: any
    // configured receive/transmit deadline would block the realtime loop.
    // Profile timeouts remain informational for the compatibility layer.
    auto serial = profile.serial;
    serial.read_timeout = std::chrono::milliseconds{0};
    serial.write_timeout = std::chrono::milliseconds{0};
    auto bus = std::find_if(
        buses.begin(), buses.end(), [&](const BusEntry& candidate) {
          return candidate.serial.path == profile.serial.path;
        });
    if (bus == buses.end()) {
      auto transport = robot_io::TransportFactory::create_serial(SerialConfig{
          serial.path,
          serial.baud_rate,
          serial.read_buffer_size,
          serial.maximum_frame_size,
          serial.read_timeout,
          serial.write_timeout});
      if (!transport.has_value()) {
        return Result<void>::failure(transport.error());
      }
      buses.push_back(BusEntry{
          serial,
          std::make_unique<St3215Bus>(
              std::move(transport.value()),
              St3215BusOptions{profile.serial.service_period})});
      bus = std::prev(buses.end());
    } else if (bus->serial.baud_rate != serial.baud_rate ||
               bus->serial.read_buffer_size != serial.read_buffer_size ||
               bus->serial.maximum_frame_size != serial.maximum_frame_size ||
               bus->serial.service_period != serial.service_period) {
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

void St3215DeviceRegistry::cycle(const CycleContext& context) noexcept {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }
  for (auto& bus : buses_) {
    bus.bus->cycle(context);
  }
}

St3215CommandAcceptance St3215DeviceRegistry::stage_command(
    std::size_t servo_index, const St3215ServoCommand& command) noexcept {
  if (servo_index >= servos_.size()) {
    return St3215CommandAcceptance::rejected;
  }
  return servos_[servo_index]->stage_command(command);
}

St3215CommandAcceptance St3215DeviceRegistry::stage_commands(
    const Snapshot<St3215ServoCommand>& snapshot,
    std::int64_t now_ns) noexcept {
  if (snapshot.axis_count != servos_.size() || servos_.empty() ||
      snapshot.timestamp_ns < 0 || snapshot.timestamp_ns > now_ns) {
    return St3215CommandAcceptance::rejected;
  }
  for (std::size_t index = 0; index < servos_.size(); ++index) {
    const auto& command = snapshot.axes[index];
    if (command.sequence != snapshot.sequence ||
        command.timestamp_ns != snapshot.timestamp_ns) {
      return St3215CommandAcceptance::rejected;
    }
  }

  std::size_t locked{};
  for (; locked < servos_.size(); ++locked) {
    if (!servos_[locked]->try_lock_command()) {
      for (std::size_t index = 0; index < locked; ++index) {
        servos_[index]->unlock_command();
      }
      return St3215CommandAcceptance::rejected;
    }
  }

  const auto first = servos_[0]->inspect_command(snapshot.axes[0], now_ns);
  bool uniform = first != St3215CommandAcceptance::rejected;
  for (std::size_t index = 1; index < servos_.size(); ++index) {
    uniform = uniform &&
              servos_[index]->inspect_command(snapshot.axes[index], now_ns) ==
                  first;
  }
  if (uniform && first == St3215CommandAcceptance::accepted) {
    for (std::size_t index = 0; index < servos_.size(); ++index) {
      servos_[index]->commit_command(snapshot.axes[index]);
    }
  }
  for (const auto& servo : servos_) {
    servo->unlock_command();
  }
  return uniform ? first : St3215CommandAcceptance::rejected;
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
