#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

#include <pthread.h>
#include <sched.h>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

class MonotonicClock {
 public:
  virtual ~MonotonicClock() = default;
  virtual std::int64_t now_ns() const noexcept = 0;
  virtual int sleep_until_ns(std::int64_t absolute_ns) noexcept = 0;
};

class RealtimeSystem {
 public:
  virtual ~RealtimeSystem() = default;
  virtual bool capture_current_thread_state() noexcept = 0;
  virtual bool preempt_rt_kernel() const noexcept = 0;
  virtual bool lock_process_memory() noexcept = 0;
  virtual bool pin_current_thread(int cpu) noexcept = 0;
  virtual bool set_current_thread_fifo(int priority) noexcept = 0;
  virtual void restore_current_thread_state() noexcept = 0;
  virtual void unlock_process_memory() noexcept = 0;
};

class SystemMonotonicClock final : public MonotonicClock {
 public:
  std::int64_t now_ns() const noexcept override;
  int sleep_until_ns(std::int64_t absolute_ns) noexcept override;
};

class LinuxRealtimeSystem final : public RealtimeSystem {
 public:
  bool capture_current_thread_state() noexcept override;
  bool preempt_rt_kernel() const noexcept override;
  bool lock_process_memory() noexcept override;
  bool pin_current_thread(int cpu) noexcept override;
  bool set_current_thread_fifo(int priority) noexcept override;
  void restore_current_thread_state() noexcept override;
  void unlock_process_memory() noexcept override;

 private:
  cpu_set_t original_affinity_{};
  sched_param original_sched_parameter_{};
  int original_sched_policy_{};
  bool thread_state_captured_{};
  bool memory_locked_{};
};

struct RealtimeLoopConfig {
  std::chrono::nanoseconds period{1'000'000};
  int cpu{0};
  int priority{80};
};

struct RealtimeSetupStatus {
  bool thread_state_captured{};
  bool preempt_rt{};
  bool memory_locked{};
  bool affinity_set{};
  bool sched_fifo{};
  bool realtime_guarantee{};
};

struct RealtimeLoopMetrics {
  std::uint64_t cycle_count{};
  std::uint64_t deadline_misses{};
  std::uint64_t skipped_releases{};
  std::uint64_t sleep_failures{};
  std::int64_t actual_period_ns{};
  std::int64_t wake_latency_ns{};
  std::int64_t maximum_wake_latency_ns{};
  std::int64_t execution_time_ns{};
  std::int64_t maximum_execution_time_ns{};
};

struct RealtimeWaitResult {
  CycleContext context{};
  int sleep_error{};
};

class RealtimeLoop {
 public:
  RealtimeLoop(MonotonicClock& clock, RealtimeSystem& system,
               RealtimeLoopConfig config = {}) noexcept;
  ~RealtimeLoop();

  Result<void> validate_config() const;
  Result<RealtimeSetupStatus> prepare();
  void release() noexcept;
  RealtimeWaitResult wait_next() noexcept;
  void observe_finish(std::int64_t finish_ns) noexcept;
  void reset_schedule() noexcept;
  RealtimeLoopMetrics metrics() const noexcept;
  bool atomics_are_lock_free() const noexcept;
  const RealtimeLoopConfig& config() const noexcept;

 private:
  MonotonicClock* clock_{};
  RealtimeSystem* system_{};
  RealtimeLoopConfig config_{};
  std::atomic<std::uint64_t> cycle_count_{};
  std::atomic<std::uint64_t> deadline_misses_{};
  std::atomic<std::uint64_t> skipped_releases_{};
  std::atomic<std::uint64_t> sleep_failures_{};
  std::atomic<std::int64_t> actual_period_ns_{};
  std::atomic<std::int64_t> wake_latency_ns_{};
  std::atomic<std::int64_t> maximum_wake_latency_ns_{};
  std::atomic<std::int64_t> execution_time_ns_{};
  std::atomic<std::int64_t> maximum_execution_time_ns_{};
  std::int64_t next_deadline_ns_{};
  std::int64_t previous_start_ns_{};
  std::int64_t current_start_ns_{};
  bool schedule_started_{};
  bool setup_active_{};
  bool captured_thread_state_{};
  bool locked_memory_{};
};

}  // namespace policy_runtime
