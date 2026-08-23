#pragma once

#include <chrono>
#include <cstdint>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime {

enum class TransportHealth { healthy, degraded, failed };

enum class SchedulingClass {
  hard_realtime_periodic,
  soft_realtime_periodic,
  blocking_event_driven,
  asynchronous,
};

struct CycleContext {
  std::uint64_t sequence{};
  std::chrono::steady_clock::time_point scheduled_start{};
  std::chrono::nanoseconds period{};
};

class Transport {
 public:
  virtual ~Transport() = default;

  virtual Result<void> open() = 0;
  virtual void close() noexcept = 0;
  virtual TransportHealth health() const noexcept = 0;
  virtual SchedulingClass scheduling_class() const noexcept = 0;

  // This is the only transport operation permitted to perform physical I/O.
  virtual void cycle(const CycleContext& context) noexcept = 0;
};

}  // namespace policy_runtime
