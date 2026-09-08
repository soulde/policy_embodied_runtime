#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>

#include <gtest/gtest.h>

#include "policy_runtime/robot_io/dds/realtime_mailbox.hpp"

namespace allocation_probe {
std::atomic<bool> enabled{};
std::atomic<std::size_t> count{};

void begin() noexcept {
  count.store(0U, std::memory_order_relaxed);
  enabled.store(true, std::memory_order_release);
}

std::size_t end() noexcept {
  enabled.store(false, std::memory_order_release);
  return count.load(std::memory_order_relaxed);
}
}  // namespace allocation_probe

void* operator new(std::size_t size) {
  if (allocation_probe::enabled.load(std::memory_order_acquire)) {
    allocation_probe::count.fetch_add(1U, std::memory_order_relaxed);
  }
  if (auto* value = std::malloc(size == 0U ? 1U : size)) {
    return value;
  }
  throw std::bad_alloc();
}

void operator delete(void* value) noexcept { std::free(value); }
void operator delete(void* value, std::size_t) noexcept { std::free(value); }

namespace {

using policy_runtime::robot_io::dds::CommandMailbox;
using policy_runtime::robot_io::dds::EpochCommand;
using policy_runtime::robot_io::dds::EthercatEpochMailbox;
using policy_runtime::robot_io::dds::SensorEventQueue;

struct Command {
  std::uint64_t sequence{};
  double target{};
};

TEST(DdsRealtimeMailboxTest, FullSensorQueueDropsOldestAndCountsIt) {
  SensorEventQueue<int, 2> queue;
  EXPECT_TRUE(queue.push_realtime(1));
  EXPECT_TRUE(queue.push_realtime(2));
  EXPECT_TRUE(queue.push_realtime(3));
  EXPECT_EQ(queue.dropped(), 1U);
  EXPECT_EQ(queue.pop(), 2);
  EXPECT_EQ(queue.pop(), 3);
  EXPECT_FALSE(queue.pop().has_value());
}

TEST(DdsRealtimeMailboxTest, CommandReaderOnlyReturnsANewerPublication) {
  CommandMailbox<Command> mailbox;
  mailbox.publish(Command{4U, 1.5});
  const auto first = mailbox.read_latest(3U);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->sequence, 4U);
  EXPECT_DOUBLE_EQ(first->target, 1.5);
  EXPECT_FALSE(mailbox.read_latest(4U).has_value());
}

TEST(DdsRealtimeMailboxTest, EthercatEpochIsInvisibleUntilCompleteCommit) {
  EthercatEpochMailbox<Command, 4> mailbox(2U);
  EXPECT_TRUE(mailbox.stage(0U, EpochCommand<Command>{9U, {1U, 1.0}}));
  EXPECT_FALSE(mailbox.commit(9U));
  EXPECT_FALSE(mailbox.consume_complete(0U).has_value());
}

TEST(DdsRealtimeMailboxTest, EthercatEpochPublishesOnlyCompleteMatchingGroup) {
  EthercatEpochMailbox<Command, 4> mailbox(2U);
  EXPECT_TRUE(mailbox.stage(0U, EpochCommand<Command>{9U, {1U, 1.0}}));
  EXPECT_TRUE(mailbox.stage(1U, EpochCommand<Command>{9U, {2U, 2.0}}));
  EXPECT_TRUE(mailbox.commit(9U));
  const auto batch = mailbox.consume_complete(0U);
  ASSERT_TRUE(batch.has_value());
  EXPECT_EQ(batch->epoch, 9U);
  EXPECT_EQ(batch->count, 2U);
  EXPECT_DOUBLE_EQ(batch->commands[0].target, 1.0);
  EXPECT_DOUBLE_EQ(batch->commands[1].target, 2.0);
}

TEST(DdsRealtimeMailboxTest, HotPathsDoNotAllocate) {
  SensorEventQueue<Command, 8> queue;
  CommandMailbox<Command> commands;
  EthercatEpochMailbox<Command, 4> epochs(2U);

  allocation_probe::begin();
  for (std::uint64_t index = 1U; index <= 1'000'000U; ++index) {
    const Command command{index, static_cast<double>(index)};
    queue.push_realtime(command);
    static_cast<void>(queue.pop());
    commands.publish(command);
    static_cast<void>(commands.read_latest(index - 1U));
    epochs.stage(0U, EpochCommand<Command>{index, command});
    epochs.stage(1U, EpochCommand<Command>{index, command});
    epochs.commit(index);
    static_cast<void>(epochs.consume_complete(index - 1U));
  }
  const auto allocations = allocation_probe::end();
  EXPECT_EQ(allocations, 0U);
}

}  // namespace
