#include <array>
#include <cstddef>
#include <new>
#include <unordered_map>

#include <gtest/gtest.h>

#include "policy_runtime/robot_io/transport_scheduler.hpp"
#include "policy_runtime/transport/object_dictionary_transport.hpp"

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

  void set_scheduling_class(policy_runtime::SchedulingClass scheduling_class) {
    scheduling_class_ = scheduling_class;
  }

  void cycle(const policy_runtime::CycleContext&) noexcept override {}

 private:
  policy_runtime::SchedulingClass scheduling_class_;
};

class FakeObjectDictionaryTransport final : public policy_runtime::ObjectDictionaryTransport {
 public:
  policy_runtime::Result<void> open() override {
    return policy_runtime::Result<void>::success();
  }

  void close() noexcept override {}

  policy_runtime::TransportHealth health() const noexcept override {
    return policy_runtime::TransportHealth::healthy;
  }

  policy_runtime::SchedulingClass scheduling_class() const noexcept override {
    return policy_runtime::SchedulingClass::hard_realtime_periodic;
  }

  void cycle(const policy_runtime::CycleContext&) noexcept override {}

  policy_runtime::MailboxSchedulingClass mailbox_scheduling_class() const noexcept override {
    return policy_runtime::MailboxSchedulingClass::blocking_event_driven;
  }

  policy_runtime::Result<policy_runtime::MailboxRequestId> queue_download(
      policy_runtime::ObjectAddress, std::span<const std::byte>) override {
    return enqueue();
  }

  policy_runtime::Result<policy_runtime::MailboxRequestId> queue_upload(
      policy_runtime::ObjectAddress) override {
    return enqueue();
  }

  std::optional<policy_runtime::MailboxRequestStatus> mailbox_status(
      policy_runtime::MailboxRequestId request_id) const override {
    const auto status = statuses_.find(request_id);
    return status == statuses_.end() ? std::nullopt
                                     : std::optional<policy_runtime::MailboxRequestStatus>{
                                           status->second};
  }

  void service_mailbox() noexcept override {
    for (auto& [request_id, status] : statuses_) {
      (void)request_id;
      if (status.state != policy_runtime::MailboxRequestState::queued) {
        continue;
      }
      if (fail_next_request_) {
        status = policy_runtime::MailboxRequestStatus{
            policy_runtime::MailboxRequestState::failed,
            policy_runtime::Error{policy_runtime::ErrorCode::io, "mailbox failure"}};
        fail_next_request_ = false;
      } else {
        status = {policy_runtime::MailboxRequestState::completed, std::nullopt};
      }
    }
  }

  void fail_next_request() { fail_next_request_ = true; }

 private:
  policy_runtime::Result<policy_runtime::MailboxRequestId> enqueue() {
    const auto request_id = next_request_id_++;
    statuses_.emplace(request_id,
                      policy_runtime::MailboxRequestStatus{
                          policy_runtime::MailboxRequestState::queued, std::nullopt});
    return policy_runtime::Result<policy_runtime::MailboxRequestId>::success(request_id);
  }

  policy_runtime::MailboxRequestId next_request_id_{1};
  std::unordered_map<policy_runtime::MailboxRequestId,
                     policy_runtime::MailboxRequestStatus>
      statuses_;
  bool fail_next_request_{false};
};

}  // namespace

TEST(TransportSchedulerTest, SeparatesRealtimeAndBlockingTransports) {
  FakeTransport ethercat{policy_runtime::SchedulingClass::hard_realtime_periodic};
  FakeTransport serial{policy_runtime::SchedulingClass::blocking_event_driven};
  policy_runtime::TransportScheduler scheduler;

  ASSERT_TRUE(scheduler.add(ethercat).has_value());
  ASSERT_TRUE(scheduler.add(serial).has_value());

  const auto ethercat_executor = scheduler.executor_id(ethercat);
  const auto serial_executor = scheduler.executor_id(serial);
  ASSERT_TRUE(ethercat_executor.has_value());
  ASSERT_TRUE(serial_executor.has_value());
  EXPECT_EQ(*ethercat_executor, policy_runtime::ExecutorId::hard_realtime);
  EXPECT_EQ(*serial_executor, policy_runtime::ExecutorId::blocking_event);
  EXPECT_NE(*ethercat_executor, *serial_executor);
}

TEST(TransportSchedulerTest, RejectsDuplicateRegistrationAndKeepsAssignmentStable) {
  FakeTransport transport{policy_runtime::SchedulingClass::soft_realtime_periodic};
  policy_runtime::TransportScheduler scheduler;

  ASSERT_TRUE(scheduler.add(transport).has_value());
  const auto executor = scheduler.executor_id(transport);
  ASSERT_TRUE(executor.has_value());
  EXPECT_EQ(*executor, policy_runtime::ExecutorId::soft_realtime);

  transport.set_scheduling_class(policy_runtime::SchedulingClass::blocking_event_driven);
  const auto duplicate = scheduler.add(transport);

  ASSERT_FALSE(duplicate.has_value());
  EXPECT_EQ(duplicate.error().code, policy_runtime::ErrorCode::invalid_argument);
  ASSERT_TRUE(scheduler.executor_id(transport).has_value());
  EXPECT_EQ(*scheduler.executor_id(transport), *executor);
}

TEST(TransportSchedulerTest, RemoveClearsAssignmentBeforeAddressReuse) {
  alignas(FakeTransport) std::array<std::byte, sizeof(FakeTransport)> storage{};
  auto* first = new (storage.data())
      FakeTransport{policy_runtime::SchedulingClass::hard_realtime_periodic};
  policy_runtime::TransportScheduler scheduler;

  ASSERT_TRUE(scheduler.add(*first).has_value());
  ASSERT_TRUE(scheduler.remove(*first).has_value());
  EXPECT_FALSE(scheduler.executor_id(*first).has_value());
  first->~FakeTransport();

  auto* replacement = new (storage.data())
      FakeTransport{policy_runtime::SchedulingClass::blocking_event_driven};
  ASSERT_EQ(replacement, first);
  EXPECT_FALSE(scheduler.executor_id(*replacement).has_value());
  ASSERT_TRUE(scheduler.add(*replacement).has_value());
  ASSERT_TRUE(scheduler.executor_id(*replacement).has_value());
  EXPECT_EQ(*scheduler.executor_id(*replacement), policy_runtime::ExecutorId::blocking_event);
  replacement->~FakeTransport();
}

TEST(ObjectDictionaryTransportTest, QueuesMailboxRequestsAndReportsCompletionOrError) {
  FakeObjectDictionaryTransport transport;
  const std::array<std::byte, 1> value{std::byte{0x01}};

  EXPECT_EQ(transport.scheduling_class(),
            policy_runtime::SchedulingClass::hard_realtime_periodic);
  EXPECT_EQ(transport.mailbox_scheduling_class(),
            policy_runtime::MailboxSchedulingClass::blocking_event_driven);

  auto download = transport.queue_download({0x6040, 0}, value);
  ASSERT_TRUE(download.has_value());
  const auto queued = transport.mailbox_status(download.value());
  ASSERT_TRUE(queued.has_value());
  EXPECT_EQ(queued->state, policy_runtime::MailboxRequestState::queued);
  EXPECT_FALSE(queued->error.has_value());

  transport.service_mailbox();
  const auto complete = transport.mailbox_status(download.value());
  ASSERT_TRUE(complete.has_value());
  EXPECT_EQ(complete->state, policy_runtime::MailboxRequestState::completed);
  EXPECT_FALSE(complete->error.has_value());

  transport.fail_next_request();
  auto upload = transport.queue_upload({0x6064, 0});
  ASSERT_TRUE(upload.has_value());
  transport.service_mailbox();
  const auto failed = transport.mailbox_status(upload.value());
  ASSERT_TRUE(failed.has_value());
  EXPECT_EQ(failed->state, policy_runtime::MailboxRequestState::failed);
  ASSERT_TRUE(failed->error.has_value());
  EXPECT_EQ(failed->error->code, policy_runtime::ErrorCode::io);
}
