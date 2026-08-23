#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

enum class ExecutorId : std::uint8_t {
  hard_realtime,
  soft_realtime,
  blocking_event,
  asynchronous,
};

class TransportScheduler {
 public:
  // Registers a non-owning transport assignment. This models executor isolation only;
  // it does not create or run threads.
  Result<void> add(Transport& transport);

  std::optional<ExecutorId> executor_id(const Transport& transport) const noexcept;

 private:
  std::unordered_map<const Transport*, ExecutorId> assignments_;
};

}  // namespace policy_runtime
