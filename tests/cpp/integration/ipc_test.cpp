#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <limits>
#include <span>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include "policy_runtime/robot_io/ipc_protocol.hpp"
#include "policy_runtime/robot_io/ipc_server.hpp"
#include "policy_runtime/robot_io/snapshot.hpp"
#include "policy_runtime/runtime/robot_io_client.hpp"

namespace {

using policy_runtime::AxisCommand;
using policy_runtime::AxisFeedback;
using policy_runtime::ErrorCode;
using policy_runtime::IpcRegionKind;
using policy_runtime::RobotIoClient;
using policy_runtime::RobotIoIpcServer;
using policy_runtime::SnapshotRegion;

constexpr std::uint32_t kGeneration = 73;

AxisCommand command(double target, std::uint64_t sequence = 0) {
  AxisCommand value{};
  value.sequence = sequence;
  value.timestamp_ns = static_cast<std::int64_t>(sequence) * 10;
  value.target = target;
  return value;
}

AxisFeedback feedback(double position, std::uint64_t sequence = 0) {
  AxisFeedback value{};
  value.sequence = sequence;
  value.timestamp_ns = static_cast<std::int64_t>(sequence) * 10;
  value.position = position;
  return value;
}

struct Mapping {
  Mapping() = default;
  Mapping(void* mapped_address, std::size_t mapped_size) noexcept
      : address(mapped_address), size(mapped_size) {}
  Mapping(const Mapping&) = delete;
  Mapping& operator=(const Mapping&) = delete;
  Mapping(Mapping&& other) noexcept
      : address(std::exchange(other.address, MAP_FAILED)), size(other.size) {}
  Mapping& operator=(Mapping&& other) noexcept {
    if (this != &other) {
      if (address != MAP_FAILED) {
        munmap(address, size);
      }
      address = std::exchange(other.address, MAP_FAILED);
      size = other.size;
    }
    return *this;
  }
  ~Mapping() {
    if (address != MAP_FAILED) {
      munmap(address, size);
    }
  }

  void* address{MAP_FAILED};
  std::size_t size{};
};

template <class T>
std::pair<Mapping, SnapshotRegion<T>> make_test_region(std::uint32_t axis_count,
                                                       IpcRegionKind kind) {
  auto size = SnapshotRegion<T>::mapping_size(axis_count);
  EXPECT_TRUE(size.has_value());
  Mapping mapping{mmap(nullptr, size.value(), PROT_READ | PROT_WRITE,
                       MAP_SHARED | MAP_ANONYMOUS, -1, 0),
                  size.value()};
  EXPECT_NE(mapping.address, MAP_FAILED);
  auto region = SnapshotRegion<T>::initialize(mapping.address, mapping.size, axis_count,
                                               kGeneration, kind);
  EXPECT_TRUE(region.has_value());
  return {std::move(mapping), std::move(region.value())};
}

std::array<int, 2> make_socket_pair(int type = SOCK_SEQPACKET) {
  std::array<int, 2> sockets{-1, -1};
  EXPECT_EQ(socketpair(AF_UNIX, type | SOCK_CLOEXEC, 0, sockets.data()), 0);
  return sockets;
}

TEST(IpcTest, PublishesOnlyCompleteSnapshots) {
  auto [mapping, region] = make_test_region<AxisCommand>(2, IpcRegionKind::command);
  const std::array commands{command(1.0), command(2.0)};

  ASSERT_TRUE(region.publish(commands, 7, 1000).has_value());
  auto snapshot = region.read_latest();

  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot.value().sequence, 7U);
  EXPECT_EQ(snapshot.value().timestamp_ns, 1000);
  EXPECT_EQ(snapshot.value().axis_count, 2U);
  EXPECT_DOUBLE_EQ(snapshot.value().axes[1].target, 2.0);
}

TEST(IpcTest, SnapshotStressNeverReturnsMixedAxes) {
  auto [mapping, region] = make_test_region<AxisCommand>(12, IpcRegionKind::command);
  std::array<AxisCommand, 12> initial{};
  for (std::size_t axis = 0; axis < initial.size(); ++axis) {
    initial[axis] = command(static_cast<double>(axis), 0);
  }
  ASSERT_TRUE(region.publish(initial, 0, 0).has_value());

  constexpr std::uint64_t kIterations = 25000;
  std::atomic<bool> writer_done{false};
  std::atomic<bool> torn{false};
  std::thread writer([&] {
    std::array<AxisCommand, 12> commands{};
    for (std::uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
      for (std::size_t axis = 0; axis < commands.size(); ++axis) {
        commands[axis] = command(static_cast<double>(sequence * 100 + axis), sequence);
      }
      if (!region.publish(commands, sequence, static_cast<std::int64_t>(sequence * 10))
               .has_value()) {
        torn.store(true);
        break;
      }
    }
    writer_done.store(true);
  });

  while (!writer_done.load()) {
    auto snapshot = region.read_latest();
    if (!snapshot.has_value()) {
      continue;
    }
    const auto sequence = snapshot.value().sequence;
    for (std::size_t axis = 0; axis < snapshot.value().axis_count; ++axis) {
      const auto& value = snapshot.value().axes[axis];
      if (value.sequence != sequence ||
          value.target != static_cast<double>(sequence * 100 + axis)) {
        torn.store(true);
      }
    }
  }
  writer.join();
  EXPECT_FALSE(torn.load());
}

TEST(IpcTest, RejectsStaleDaemonGenerationAndMalformedHeaders) {
  auto [mapping, region] = make_test_region<AxisCommand>(2, IpcRegionKind::command);
  (void)region;
  const auto* header = static_cast<const policy_runtime::IpcHeader*>(mapping.address);

  const auto stale = policy_runtime::validate_header<AxisCommand>(
      *header, kGeneration + 1, 2, IpcRegionKind::command, mapping.size);
  ASSERT_FALSE(stale.has_value());
  EXPECT_EQ(stale.error().code, ErrorCode::unavailable);

  auto malformed = *header;
  malformed.magic[0] = 'X';
  const auto bad_magic = policy_runtime::validate_header<AxisCommand>(
      malformed, kGeneration, 2, IpcRegionKind::command, mapping.size);
  ASSERT_FALSE(bad_magic.has_value());
  EXPECT_EQ(bad_magic.error().code, ErrorCode::protocol);

  malformed = *header;
  malformed.axis_stride += 8;
  const auto bad_stride = policy_runtime::validate_header<AxisCommand>(
      malformed, kGeneration, 2, IpcRegionKind::command, mapping.size);
  ASSERT_FALSE(bad_stride.has_value());
  EXPECT_EQ(bad_stride.error().code, ErrorCode::protocol);

  malformed = *header;
  ++malformed.abi_version;
  EXPECT_EQ(policy_runtime::validate_header<AxisCommand>(
                malformed, kGeneration, 2, IpcRegionKind::command, mapping.size)
                .error()
                .code,
            ErrorCode::protocol);

  malformed = *header;
  malformed.axis_count = 3;
  EXPECT_EQ(policy_runtime::validate_header<AxisCommand>(
                malformed, kGeneration, 2, IpcRegionKind::command, mapping.size)
                .error()
                .code,
            ErrorCode::protocol);

  malformed = *header;
  malformed.region_kind = static_cast<std::uint32_t>(IpcRegionKind::feedback);
  EXPECT_EQ(policy_runtime::validate_header<AxisCommand>(
                malformed, kGeneration, 2, IpcRegionKind::command, mapping.size)
                .error()
                .code,
            ErrorCode::protocol);
}

TEST(IpcTest, MappingSizeAcceptsFrozenAxisRangeOnly) {
  EXPECT_TRUE(SnapshotRegion<AxisCommand>::mapping_size(0).has_value());
  EXPECT_TRUE(SnapshotRegion<AxisCommand>::mapping_size(12).has_value());
  const auto too_many = SnapshotRegion<AxisCommand>::mapping_size(13);
  ASSERT_FALSE(too_many.has_value());
  EXPECT_EQ(too_many.error().code, ErrorCode::invalid_argument);
  const auto absurd = SnapshotRegion<AxisCommand>::mapping_size(
      std::numeric_limits<std::uint32_t>::max());
  ASSERT_FALSE(absurd.has_value());
  EXPECT_EQ(absurd.error().code, ErrorCode::invalid_argument);
}

TEST(IpcTest, TransfersSealedCloexecMemfdsAndSnapshotsBothDirections) {
  auto sockets = make_socket_pair(SOCK_STREAM);
  auto server_result = RobotIoIpcServer::create(sockets[0], 2, kGeneration);
  ASSERT_TRUE(server_result.has_value()) << server_result.error().message;
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());

  auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
  ASSERT_TRUE(client_result.has_value()) << client_result.error().message;
  auto client = std::move(client_result.value());

  EXPECT_EQ(client.axis_count(), 2U);
  EXPECT_NE(client.command_fd(), server.command_fd());
  EXPECT_NE(client.feedback_fd(), server.feedback_fd());
  EXPECT_NE(fcntl(client.command_fd(), F_GETFD) & FD_CLOEXEC, 0);
  EXPECT_NE(fcntl(client.feedback_fd(), F_GETFD) & FD_CLOEXEC, 0);
  EXPECT_EQ(fcntl(client.command_fd(), F_GET_SEALS) &
                (F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL),
            F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL);

  const std::array commands{command(3.5, 11), command(-4.5, 11)};
  ASSERT_TRUE(client.publish_commands(commands, 11, 110).has_value());
  auto daemon_commands = server.read_commands();
  ASSERT_TRUE(daemon_commands.has_value());
  EXPECT_EQ(daemon_commands.value().sequence, 11U);
  EXPECT_DOUBLE_EQ(daemon_commands.value().axes[0].target, 3.5);
  EXPECT_DOUBLE_EQ(daemon_commands.value().axes[1].target, -4.5);

  const std::array feedback_values{feedback(1.25, 12), feedback(2.5, 12)};
  ASSERT_TRUE(server.publish_feedback(feedback_values, 12, 120).has_value());
  auto host_feedback = client.read_feedback();
  ASSERT_TRUE(host_feedback.has_value());
  EXPECT_EQ(host_feedback.value().sequence, 12U);
  EXPECT_DOUBLE_EQ(host_feedback.value().axes[1].position, 2.5);
}

TEST(IpcTest, ClientRejectsUnexpectedGenerationFromRealHandshake) {
  auto sockets = make_socket_pair();
  auto server_result = RobotIoIpcServer::create(sockets[0], 1, kGeneration);
  ASSERT_TRUE(server_result.has_value());
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());

  const auto client = RobotIoClient::connect(sockets[1], kGeneration + 1);
  ASSERT_FALSE(client.has_value());
  EXPECT_EQ(client.error().code, ErrorCode::unavailable);
}

TEST(IpcTest, SharesSnapshotAtomicsAcrossProcesses) {
  auto sockets = make_socket_pair();
  auto server_result = RobotIoIpcServer::create(sockets[0], 2, kGeneration);
  ASSERT_TRUE(server_result.has_value());
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());

  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    (void)::close(sockets[0]);
    auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
    if (!client_result.has_value()) {
      _exit(10);
    }
    auto client = std::move(client_result.value());
    const std::array values{command(8.0, 21), command(9.0, 21)};
    if (!client.publish_commands(values, 21, 210).has_value()) {
      _exit(11);
    }
    if (!client.close().has_value()) {
      _exit(12);
    }
    _exit(0);
  }

  ASSERT_EQ(::close(sockets[1]), 0);
  int status{};
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0);
  auto snapshot = server.read_commands();
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot.value().sequence, 21U);
  EXPECT_DOUBLE_EQ(snapshot.value().axes[0].target, 8.0);
  EXPECT_DOUBLE_EQ(snapshot.value().axes[1].target, 9.0);
}

TEST(IpcTest, ReportsPeerDisconnectWithoutBlocking) {
  auto sockets = make_socket_pair();
  auto server_result = RobotIoIpcServer::create(sockets[0], 0, kGeneration);
  ASSERT_TRUE(server_result.has_value());
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());
  auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
  ASSERT_TRUE(client_result.has_value());
  auto client = std::move(client_result.value());

  ASSERT_TRUE(client.check_peer().has_value());
  ASSERT_TRUE(server.close().has_value());
  const auto disconnected = client.check_peer();
  ASSERT_FALSE(disconnected.has_value());
  EXPECT_EQ(disconnected.error().code, ErrorCode::unavailable);
  const std::array<AxisCommand, 0> no_commands{};
  const auto rejected_publish = client.publish_commands(no_commands, 1, 10);
  ASSERT_FALSE(rejected_publish.has_value());
  EXPECT_EQ(rejected_publish.error().code, ErrorCode::unavailable);
}

}  // namespace
