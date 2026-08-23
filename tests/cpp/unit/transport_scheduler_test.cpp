#include <gtest/gtest.h>

#include "policy_runtime/robot_io/transport_scheduler.hpp"

namespace {

class FakeTransport final : public policy_runtime::Transport {
 public:
  explicit FakeTransport(policy_runtime::SchedulingClass scheduling_class)
      : scheduling_class_(scheduling_class) {}

  policy_runtime::Result<void> open() override {
    return policy_runtime::Result<void>::success();
  }

  void close() noexcept override {}

  policy_runtime::TransportHealth health() const noexcept override {
    return policy_runtime::TransportHealth::healthy;
  }

  policy_runtime::SchedulingClass scheduling_class() const noexcept override {
    return scheduling_class_;
  }

  void cycle(const policy_runtime::CycleContext&) noexcept override {}

 private:
  policy_runtime::SchedulingClass scheduling_class_;
};

}  // namespace

TEST(TransportSchedulerTest, SeparatesRealtimeAndBlockingTransports) {
  FakeTransport ethercat{policy_runtime::SchedulingClass::hard_realtime_periodic};
  FakeTransport serial{policy_runtime::SchedulingClass::blocking_event_driven};
  policy_runtime::TransportScheduler scheduler;

  ASSERT_TRUE(scheduler.add(ethercat).has_value());
  ASSERT_TRUE(scheduler.add(serial).has_value());

  EXPECT_NE(scheduler.executor_id(ethercat), scheduler.executor_id(serial));
}
