#include "policy_runtime/robot_io/daemon/realtime_loop.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <limits>

#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <time.h>
#include <unistd.h>

namespace policy_runtime {
namespace {

timespec to_timespec(std::int64_t absolute_ns) noexcept {
  if (absolute_ns < 0) {
    absolute_ns = 0;
  }
  constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;
  return timespec{static_cast<time_t>(absolute_ns / kNanosecondsPerSecond),
                  static_cast<long>(absolute_ns % kNanosecondsPerSecond)};
}

bool contains_preempt_rt(const char* text) noexcept {
  return text != nullptr &&
         (std::strstr(text, "PREEMPT_RT") != nullptr ||
          std::strstr(text, "PREEMPT RT") != nullptr);
}

std::int64_t saturating_add(std::int64_t value, std::int64_t increment) noexcept {
  if (increment > 0 && value > std::numeric_limits<std::int64_t>::max() - increment) {
    return std::numeric_limits<std::int64_t>::max();
  }
  return value + increment;
}

void update_maximum(std::atomic<std::int64_t>& maximum,
                    std::int64_t value) noexcept {
  auto observed = maximum.load(std::memory_order_relaxed);
  while (value > observed &&
         !maximum.compare_exchange_weak(observed, value,
                                        std::memory_order_relaxed,
                                        std::memory_order_relaxed)) {
  }
}

}  // namespace

std::int64_t SystemMonotonicClock::now_ns() const noexcept {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    return 0;
  }
  return static_cast<std::int64_t>(value.tv_sec) * 1'000'000'000LL +
         value.tv_nsec;
}

int SystemMonotonicClock::sleep_until_ns(std::int64_t absolute_ns) noexcept {
  auto deadline = to_timespec(absolute_ns);
  int result{};
  do {
    result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
  } while (result == EINTR);
  return result;
}

bool LinuxRealtimeSystem::preempt_rt_kernel() const noexcept {
  const int descriptor = ::open("/sys/kernel/realtime", O_RDONLY | O_CLOEXEC);
  if (descriptor >= 0) {
    std::array<char, 8> value{};
    const auto size = ::read(descriptor, value.data(), value.size() - 1U);
    static_cast<void>(::close(descriptor));
    if (size > 0) {
      return value[0] == '1';
    }
  }
  utsname release{};
  return uname(&release) == 0 &&
         (contains_preempt_rt(release.release) ||
          contains_preempt_rt(release.version));
}

bool LinuxRealtimeSystem::capture_current_thread_state() noexcept {
  if (thread_state_captured_) {
    return false;
  }
  if (pthread_getaffinity_np(pthread_self(), sizeof(original_affinity_),
                             &original_affinity_) != 0) {
    return false;
  }
  if (pthread_getschedparam(pthread_self(), &original_sched_policy_,
                            &original_sched_parameter_) != 0) {
    return false;
  }
  thread_state_captured_ = true;
  return true;
}

bool LinuxRealtimeSystem::lock_process_memory() noexcept {
  memory_locked_ = mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
  return memory_locked_;
}

bool LinuxRealtimeSystem::pin_current_thread(int cpu) noexcept {
  if (cpu < 0 || cpu >= CPU_SETSIZE) {
    return false;
  }
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
}

bool LinuxRealtimeSystem::set_current_thread_fifo(int priority) noexcept {
  if (priority < sched_get_priority_min(SCHED_FIFO) ||
      priority > sched_get_priority_max(SCHED_FIFO)) {
    return false;
  }
  sched_param parameter{};
  parameter.sched_priority = priority;
  return pthread_setschedparam(pthread_self(), SCHED_FIFO, &parameter) == 0;
}

void LinuxRealtimeSystem::restore_current_thread_state() noexcept {
  if (!thread_state_captured_) {
    return;
  }
  static_cast<void>(pthread_setschedparam(
      pthread_self(), original_sched_policy_, &original_sched_parameter_));
  static_cast<void>(pthread_setaffinity_np(
      pthread_self(), sizeof(original_affinity_), &original_affinity_));
  thread_state_captured_ = false;
}

void LinuxRealtimeSystem::unlock_process_memory() noexcept {
  if (memory_locked_) {
    static_cast<void>(munlockall());
    memory_locked_ = false;
  }
}

RealtimeLoop::RealtimeLoop(MonotonicClock& clock, RealtimeSystem& system,
                           RealtimeLoopConfig config) noexcept
    : clock_(&clock), system_(&system), config_(config) {}

RealtimeLoop::~RealtimeLoop() { release(); }

Result<void> RealtimeLoop::validate_config() const {
  if (config_.period.count() <= 0 || config_.cpu < 0 ||
      config_.priority < sched_get_priority_min(SCHED_FIFO) ||
      config_.priority > sched_get_priority_max(SCHED_FIFO)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "invalid real-time loop configuration"});
  }
  return Result<void>::success();
}

Result<RealtimeSetupStatus> RealtimeLoop::prepare() {
  auto valid = validate_config();
  if (!valid.has_value()) {
    return Result<RealtimeSetupStatus>::failure(
        valid.error());
  }
  if (setup_active_ || captured_thread_state_ || locked_memory_) {
    return Result<RealtimeSetupStatus>::failure(
        {ErrorCode::invalid_argument, "real-time loop setup is already active"});
  }
  RealtimeSetupStatus status{};
  status.thread_state_captured = system_->capture_current_thread_state();
  status.preempt_rt = system_->preempt_rt_kernel();
  captured_thread_state_ = status.thread_state_captured;
  if (status.thread_state_captured) {
    status.memory_locked = system_->lock_process_memory();
    locked_memory_ = status.memory_locked;
    status.affinity_set = system_->pin_current_thread(config_.cpu);
    status.sched_fifo = system_->set_current_thread_fifo(config_.priority);
  }
  status.realtime_guarantee =
      status.thread_state_captured && status.preempt_rt &&
      status.memory_locked && status.affinity_set && status.sched_fifo;
  setup_active_ = status.realtime_guarantee;
  reset_schedule();
  if (!status.realtime_guarantee) {
    release();
  }
  return Result<RealtimeSetupStatus>::success(status);
}

void RealtimeLoop::release() noexcept {
  if (captured_thread_state_) {
    system_->restore_current_thread_state();
    captured_thread_state_ = false;
  }
  if (locked_memory_) {
    system_->unlock_process_memory();
    locked_memory_ = false;
  }
  setup_active_ = false;
}

RealtimeWaitResult RealtimeLoop::wait_next() noexcept {
  const auto now = clock_->now_ns();
  if (!schedule_started_) {
    next_deadline_ns_ = saturating_add(now, config_.period.count());
    schedule_started_ = true;
  } else {
    next_deadline_ns_ =
        saturating_add(next_deadline_ns_, config_.period.count());
  }
  if (next_deadline_ns_ <= now) {
    const auto behind = now - next_deadline_ns_;
    const auto skipped =
        static_cast<std::uint64_t>(behind / config_.period.count()) + 1U;
    const auto maximum_increment =
        (std::numeric_limits<std::int64_t>::max() - next_deadline_ns_) /
        config_.period.count();
    const auto increment_periods = std::min<std::uint64_t>(
        skipped, static_cast<std::uint64_t>(maximum_increment));
    next_deadline_ns_ += static_cast<std::int64_t>(increment_periods) *
                         config_.period.count();
    skipped_releases_.fetch_add(skipped, std::memory_order_relaxed);
  }
  int sleep_error{};
  do {
    sleep_error = clock_->sleep_until_ns(next_deadline_ns_);
  } while (sleep_error == EINTR);
  const auto next_cycle = cycle_count_.load(std::memory_order_relaxed) + 1U;
  const CycleContext context{
      next_cycle,
      std::chrono::steady_clock::time_point{
          std::chrono::nanoseconds{next_deadline_ns_}},
      config_.period};
  if (sleep_error != 0) {
    sleep_failures_.fetch_add(1U, std::memory_order_relaxed);
    return {context, sleep_error};
  }
  const auto started = clock_->now_ns();
  const auto previous_cycles = cycle_count_.load(std::memory_order_relaxed);
  if (previous_cycles != 0U) {
    actual_period_ns_.store(started - previous_start_ns_,
                            std::memory_order_relaxed);
  }
  previous_start_ns_ = started;
  current_start_ns_ = started;
  const auto wake_latency =
      started > next_deadline_ns_ ? started - next_deadline_ns_ : 0;
  wake_latency_ns_.store(wake_latency, std::memory_order_relaxed);
  update_maximum(maximum_wake_latency_ns_, wake_latency);
  const auto cycle = cycle_count_.fetch_add(1U, std::memory_order_relaxed) + 1U;
  auto released_context = context;
  released_context.sequence = cycle;
  return {released_context, 0};
}

void RealtimeLoop::observe_finish(std::int64_t finish_ns) noexcept {
  if (!schedule_started_) {
    return;
  }
  const auto execution_time =
      finish_ns > current_start_ns_ ? finish_ns - current_start_ns_ : 0;
  execution_time_ns_.store(execution_time, std::memory_order_relaxed);
  update_maximum(maximum_execution_time_ns_, execution_time);
  const auto next_release =
      saturating_add(next_deadline_ns_, config_.period.count());
  if (finish_ns > next_release) {
    const auto misses =
        static_cast<std::uint64_t>((finish_ns - next_release) /
                                   config_.period.count()) +
        1U;
    deadline_misses_.fetch_add(misses, std::memory_order_relaxed);
  }
}

void RealtimeLoop::reset_schedule() noexcept {
  cycle_count_.store(0U, std::memory_order_relaxed);
  deadline_misses_.store(0U, std::memory_order_relaxed);
  skipped_releases_.store(0U, std::memory_order_relaxed);
  sleep_failures_.store(0U, std::memory_order_relaxed);
  actual_period_ns_.store(0, std::memory_order_relaxed);
  wake_latency_ns_.store(0, std::memory_order_relaxed);
  maximum_wake_latency_ns_.store(0, std::memory_order_relaxed);
  execution_time_ns_.store(0, std::memory_order_relaxed);
  maximum_execution_time_ns_.store(0, std::memory_order_relaxed);
  next_deadline_ns_ = 0;
  previous_start_ns_ = 0;
  current_start_ns_ = 0;
  schedule_started_ = false;
}

RealtimeLoopMetrics RealtimeLoop::metrics() const noexcept {
  return RealtimeLoopMetrics{
      cycle_count_.load(std::memory_order_relaxed),
      deadline_misses_.load(std::memory_order_relaxed),
      skipped_releases_.load(std::memory_order_relaxed),
      sleep_failures_.load(std::memory_order_relaxed),
      actual_period_ns_.load(std::memory_order_relaxed),
      wake_latency_ns_.load(std::memory_order_relaxed),
      maximum_wake_latency_ns_.load(std::memory_order_relaxed),
      execution_time_ns_.load(std::memory_order_relaxed),
      maximum_execution_time_ns_.load(std::memory_order_relaxed)};
}

bool RealtimeLoop::atomics_are_lock_free() const noexcept {
  return cycle_count_.is_lock_free() && deadline_misses_.is_lock_free() &&
         skipped_releases_.is_lock_free() && sleep_failures_.is_lock_free() &&
         actual_period_ns_.is_lock_free() && wake_latency_ns_.is_lock_free() &&
         maximum_wake_latency_ns_.is_lock_free() &&
         execution_time_ns_.is_lock_free() &&
         maximum_execution_time_ns_.is_lock_free();
}

const RealtimeLoopConfig& RealtimeLoop::config() const noexcept { return config_; }

}  // namespace policy_runtime
