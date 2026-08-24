#include "policy_runtime/robot_io/realtime_loop.hpp"

#include <array>
#include <cerrno>
#include <cstring>

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

bool LinuxRealtimeSystem::lock_process_memory() noexcept {
  return mlockall(MCL_CURRENT | MCL_FUTURE) == 0;
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

RealtimeLoop::RealtimeLoop(MonotonicClock& clock, RealtimeSystem& system,
                           RealtimeLoopConfig config) noexcept
    : clock_(&clock), system_(&system), config_(config) {}

Result<RealtimeSetupStatus> RealtimeLoop::prepare() {
  if (config_.period.count() <= 0 || config_.cpu < 0 ||
      config_.priority < sched_get_priority_min(SCHED_FIFO) ||
      config_.priority > sched_get_priority_max(SCHED_FIFO)) {
    return Result<RealtimeSetupStatus>::failure(
        {ErrorCode::invalid_argument, "invalid real-time loop configuration"});
  }
  RealtimeSetupStatus status{};
  status.preempt_rt = system_->preempt_rt_kernel();
  status.memory_locked = system_->lock_process_memory();
  status.affinity_set = system_->pin_current_thread(config_.cpu);
  status.sched_fifo = system_->set_current_thread_fifo(config_.priority);
  status.realtime_guarantee = status.preempt_rt && status.memory_locked &&
                              status.affinity_set && status.sched_fifo;
  reset_schedule();
  return Result<RealtimeSetupStatus>::success(status);
}

CycleContext RealtimeLoop::wait_next() noexcept {
  const auto now = clock_->now_ns();
  if (!schedule_started_) {
    next_deadline_ns_ = now + config_.period.count();
    schedule_started_ = true;
  } else {
    next_deadline_ns_ += config_.period.count();
  }
  static_cast<void>(clock_->sleep_until_ns(next_deadline_ns_));
  const auto started = clock_->now_ns();
  const auto previous_cycles = cycle_count_.load(std::memory_order_relaxed);
  if (previous_cycles != 0U) {
    actual_period_ns_.store(started - previous_start_ns_,
                            std::memory_order_relaxed);
  }
  previous_start_ns_ = started;
  const auto cycle = cycle_count_.fetch_add(1U, std::memory_order_relaxed) + 1U;
  return CycleContext{
      cycle,
      std::chrono::steady_clock::time_point{std::chrono::nanoseconds{next_deadline_ns_}},
      config_.period};
}

void RealtimeLoop::observe_finish(std::int64_t finish_ns) noexcept {
  if (!schedule_started_) {
    return;
  }
  const auto jitter = finish_ns - next_deadline_ns_;
  const auto magnitude = jitter < 0 ? -jitter : jitter;
  auto maximum = maximum_jitter_ns_.load(std::memory_order_relaxed);
  while (magnitude > maximum &&
         !maximum_jitter_ns_.compare_exchange_weak(
             maximum, magnitude, std::memory_order_relaxed,
             std::memory_order_relaxed)) {
  }
  if (finish_ns > next_deadline_ns_ + config_.period.count()) {
    deadline_misses_.fetch_add(1U, std::memory_order_relaxed);
  }
}

void RealtimeLoop::reset_schedule() noexcept {
  cycle_count_.store(0U, std::memory_order_relaxed);
  deadline_misses_.store(0U, std::memory_order_relaxed);
  actual_period_ns_.store(0, std::memory_order_relaxed);
  maximum_jitter_ns_.store(0, std::memory_order_relaxed);
  next_deadline_ns_ = 0;
  previous_start_ns_ = 0;
  schedule_started_ = false;
}

RealtimeLoopMetrics RealtimeLoop::metrics() const noexcept {
  return RealtimeLoopMetrics{
      cycle_count_.load(std::memory_order_relaxed),
      deadline_misses_.load(std::memory_order_relaxed),
      actual_period_ns_.load(std::memory_order_relaxed),
      maximum_jitter_ns_.load(std::memory_order_relaxed)};
}

const RealtimeLoopConfig& RealtimeLoop::config() const noexcept { return config_; }

}  // namespace policy_runtime
