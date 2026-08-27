#include <array>
#include <cstddef>
#include <new>
#include <unordered_map>
#include <vector>

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
    return policy_runtime::SchedulingClass::blocking_event_driven;
  }

  void cycle(const policy_runtime::CycleContext&) noexcept override {
    for (auto& [request_id, request] : requests_) {
      (void)request_id;
      auto& status = request.status;
      if (status.state != policy_runtime::MailboxRequestState::queued) {
        continue;
      }
      if (fail_next_request_) {
        status = policy_runtime::MailboxRequestStatus{
            policy_runtime::MailboxRequestState::failed,
            policy_runtime::Error{policy_runtime::ErrorCode::io, "mailbox failure"}, {}};
        fail_next_request_ = false;
      } else {
        status.state = policy_runtime::MailboxRequestState::completed;
        if (request.is_upload) {
          status.uploaded_bytes = {std::byte{0x12}, std::byte{0x34}};
        }
      }
    }
  }

  policy_runtime::Result<policy_runtime::MailboxRequestId> queue_download(
      policy_runtime::ObjectAddress, std::span<const std::byte>) override {
    return enqueue(false);
  }

  policy_runtime::Result<policy_runtime::MailboxRequestId> queue_upload(
      policy_runtime::ObjectAddress) override {
    return enqueue(true);
  }

  std::optional<policy_runtime::MailboxRequestStatus> mailbox_status(
      policy_runtime::MailboxRequestId request_id) const override {
    const auto status = requests_.find(request_id);
    return status == requests_.end() ? std::nullopt
                                     : std::optional<policy_runtime::MailboxRequestStatus>{
                                           status->second.status};
  }

  void fail_next_request() { fail_next_request_ = true; }

 private:
  policy_runtime::Result<policy_runtime::MailboxRequestId> enqueue(bool is_upload) {
    const auto request_id = next_request_id_++;
    requests_.emplace(request_id, Request{is_upload,
                                          {policy_runtime::MailboxRequestState::queued,
                                           std::nullopt, {}}});
    return policy_runtime::Result<policy_runtime::MailboxRequestId>::success(request_id);
  }

  struct Request {
    bool is_upload;
    policy_runtime::MailboxRequestStatus status;
  };

  policy_runtime::MailboxRequestId next_request_id_{1};
  std::unordered_map<policy_runtime::MailboxRequestId, Request> requests_;
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
  ASSERT_TRUE(scheduler.remove(*replacement).has_value());
  EXPECT_FALSE(scheduler.executor_id(*replacement).has_value());
  replacement->~FakeTransport();
}

TEST(ObjectDictionaryTransportTest, QueuesMailboxRequestsAndReportsCompletionOrError) {
  FakeObjectDictionaryTransport transport;
  const std::array<std::byte, 1> value{std::byte{0x01}};

  EXPECT_EQ(transport.scheduling_class(),
            policy_runtime::SchedulingClass::blocking_event_driven);

  auto download = transport.queue_download({0x6040, 0}, value);
  ASSERT_TRUE(download.has_value());
  const auto queued = transport.mailbox_status(download.value());
  ASSERT_TRUE(queued.has_value());
  EXPECT_EQ(queued->state, policy_runtime::MailboxRequestState::queued);
  EXPECT_FALSE(queued->error.has_value());

  transport.cycle({});
  const auto complete = transport.mailbox_status(download.value());
  ASSERT_TRUE(complete.has_value());
  EXPECT_EQ(complete->state, policy_runtime::MailboxRequestState::completed);
  EXPECT_FALSE(complete->error.has_value());
  EXPECT_TRUE(complete->uploaded_bytes.empty());

  auto upload = transport.queue_upload({0x6064, 0});
  ASSERT_TRUE(upload.has_value());
  transport.cycle({});
  const auto uploaded = transport.mailbox_status(upload.value());
  ASSERT_TRUE(uploaded.has_value());
  EXPECT_EQ(uploaded->state, policy_runtime::MailboxRequestState::completed);
  EXPECT_EQ(uploaded->uploaded_bytes,
            (std::vector<std::byte>{std::byte{0x12}, std::byte{0x34}}));

  transport.fail_next_request();
  auto failed_upload = transport.queue_upload({0x6064, 0});
  ASSERT_TRUE(failed_upload.has_value());
  transport.cycle({});
  const auto failed = transport.mailbox_status(failed_upload.value());
  ASSERT_TRUE(failed.has_value());
  EXPECT_EQ(failed->state, policy_runtime::MailboxRequestState::failed);
  ASSERT_TRUE(failed->error.has_value());
  EXPECT_EQ(failed->error->code, policy_runtime::ErrorCode::io);
}
