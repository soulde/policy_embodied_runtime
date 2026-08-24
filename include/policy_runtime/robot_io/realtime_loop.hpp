#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

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
  virtual bool preempt_rt_kernel() const noexcept = 0;
  virtual bool lock_process_memory() noexcept = 0;
  virtual bool pin_current_thread(int cpu) noexcept = 0;
  virtual bool set_current_thread_fifo(int priority) noexcept = 0;
};

class SystemMonotonicClock final : public MonotonicClock {
 public:
  std::int64_t now_ns() const noexcept override;
  int sleep_until_ns(std::int64_t absolute_ns) noexcept override;
};

class LinuxRealtimeSystem final : public RealtimeSystem {
 public:
  bool preempt_rt_kernel() const noexcept override;
  bool lock_process_memory() noexcept override;
  bool pin_current_thread(int cpu) noexcept override;
  bool set_current_thread_fifo(int priority) noexcept override;
};

struct RealtimeLoopConfig {
  std::chrono::nanoseconds period{1'000'000};
  int cpu{0};
  int priority{80};
};

struct RealtimeSetupStatus {
  bool preempt_rt{};
  bool memory_locked{};
  bool affinity_set{};
  bool sched_fifo{};
  bool realtime_guarantee{};
};

struct RealtimeLoopMetrics {
  std::uint64_t cycle_count{};
  std::uint64_t deadline_misses{};
  std::int64_t actual_period_ns{};
  std::int64_t maximum_jitter_ns{};
};

class RealtimeLoop {
 public:
  RealtimeLoop(MonotonicClock& clock, RealtimeSystem& system,
               RealtimeLoopConfig config = {}) noexcept;

  Result<RealtimeSetupStatus> prepare();
  CycleContext wait_next() noexcept;
  void observe_finish(std::int64_t finish_ns) noexcept;
  void reset_schedule() noexcept;
  RealtimeLoopMetrics metrics() const noexcept;
  const RealtimeLoopConfig& config() const noexcept;

 private:
  MonotonicClock* clock_{};
  RealtimeSystem* system_{};
  RealtimeLoopConfig config_{};
  std::atomic<std::uint64_t> cycle_count_{};
  std::atomic<std::uint64_t> deadline_misses_{};
  std::atomic<std::int64_t> actual_period_ns_{};
  std::atomic<std::int64_t> maximum_jitter_ns_{};
  std::int64_t next_deadline_ns_{};
  std::int64_t previous_start_ns_{};
  bool schedule_started_{};
};

}  // namespace policy_runtime
