#include "policy_runtime/robot_io/transport_scheduler.hpp"

namespace policy_runtime {

namespace {

ExecutorId executor_for(SchedulingClass scheduling_class) {
  switch (scheduling_class) {
    case SchedulingClass::hard_realtime_periodic:
      return ExecutorId::hard_realtime;
    case SchedulingClass::soft_realtime_periodic:
      return ExecutorId::soft_realtime;
    case SchedulingClass::blocking_event_driven:
      return ExecutorId::blocking_event;
    case SchedulingClass::asynchronous:
      return ExecutorId::asynchronous;
  }
  return ExecutorId::asynchronous;
}

}  // namespace

Result<void> TransportScheduler::add(Transport& transport) {
  assignments_.insert_or_assign(&transport, executor_for(transport.scheduling_class()));
  return Result<void>::success();
}

std::optional<ExecutorId> TransportScheduler::executor_id(
    const Transport& transport) const noexcept {
  const auto assignment = assignments_.find(&transport);
  if (assignment == assignments_.end()) {
    return std::nullopt;
  }
  return assignment->second;
}

}  // namespace policy_runtime
