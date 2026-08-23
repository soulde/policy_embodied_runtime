#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <limits>
#include <span>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <sched.h>
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
using policy_runtime::SnapshotMappingAccess;
using policy_runtime::SnapshotRegion;

constexpr std::uint32_t kGeneration = 73;

AxisCommand command(double target, std::uint64_t sequence = 0) {
  AxisCommand value{};
  value.sequence = sequence;
  value.timestamp_ns = static_cast<std::int64_t>(sequence) * 10;
  value.target = target;
  value.flags = static_cast<std::uint32_t>(sequence);
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

std::size_t open_descriptor_count() {
  return static_cast<std::size_t>(
      std::distance(std::filesystem::directory_iterator("/proc/self/fd"),
                    std::filesystem::directory_iterator{}));
}

void send_raw_setup(int socket_fd, std::size_t payload_size,
                    std::span<const int> descriptors) {
  policy_runtime::IpcSetupMessage setup{};
  alignas(cmsghdr) std::array<std::byte, CMSG_SPACE(3 * sizeof(int))> control{};
  ASSERT_LE(descriptors.size(), 3U);
  iovec vector{&setup, payload_size};
  msghdr message{};
  message.msg_iov = &vector;
  message.msg_iovlen = 1;
  message.msg_control = control.data();
  message.msg_controllen = CMSG_SPACE(descriptors.size_bytes());
  auto* item = CMSG_FIRSTHDR(&message);
  ASSERT_NE(item, nullptr);
  item->cmsg_level = SOL_SOCKET;
  item->cmsg_type = SCM_RIGHTS;
  item->cmsg_len = CMSG_LEN(descriptors.size_bytes());
  std::memcpy(CMSG_DATA(item), descriptors.data(), descriptors.size_bytes());
  ASSERT_EQ(sendmsg(socket_fd, &message, MSG_NOSIGNAL | MSG_EOR),
            static_cast<ssize_t>(payload_size));
}

TEST(IpcTest, PublishesOnlyCompleteSnapshots) {
  auto [mapping, region] = make_test_region<AxisCommand>(2, IpcRegionKind::command);
  auto writer = region.writer();
  auto reader = region.reader();
  std::array commands{command(1.0, 7), command(2.0, 7)};
  for (auto& value : commands) {
    value.timestamp_ns = 1000;
  }

  ASSERT_TRUE(writer.publish(commands, 7, 1000).has_value());
  auto snapshot = reader.read_latest();

  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot.value().sequence, 7U);
  EXPECT_EQ(snapshot.value().timestamp_ns, 1000);
  EXPECT_EQ(snapshot.value().axis_count, 2U);
  EXPECT_DOUBLE_EQ(snapshot.value().axes[1].target, 2.0);
}

TEST(IpcTest, SnapshotStressNeverReturnsMixedAxes) {
  auto [mapping, region] = make_test_region<AxisCommand>(12, IpcRegionKind::command);
  auto writer_view = region.writer();
  auto reader_view = region.reader();
  std::array<AxisCommand, 12> initial{};
  for (std::size_t axis = 0; axis < initial.size(); ++axis) {
    initial[axis] = command(static_cast<double>(axis), 0);
    initial[axis].flags = static_cast<std::uint32_t>(axis);
  }
  ASSERT_TRUE(writer_view.publish(initial, 0, 0).has_value());

  constexpr std::uint64_t kIterations = 25000;
  std::atomic<bool> writer_done{false};
  std::atomic<bool> torn{false};
  std::uint64_t successful_reads = 0;
  std::thread writer([&] {
    std::array<AxisCommand, 12> commands{};
    for (std::uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
      for (std::size_t axis = 0; axis < commands.size(); ++axis) {
        commands[axis] = command(static_cast<double>(sequence * 100 + axis), sequence);
        commands[axis].flags = static_cast<std::uint32_t>(sequence + axis);
      }
      if (!writer_view
               .publish(commands, sequence, static_cast<std::int64_t>(sequence * 10))
               .has_value()) {
        torn.store(true);
        break;
      }
      if ((sequence & 63U) == 0) {
        std::this_thread::yield();
      }
    }
    writer_done.store(true);
  });

  while (!writer_done.load()) {
    auto snapshot = reader_view.read_latest();
    if (!snapshot.has_value()) {
      continue;
    }
    ++successful_reads;
    const auto sequence = snapshot.value().sequence;
    if (snapshot.value().timestamp_ns != static_cast<std::int64_t>(sequence * 10)) {
      torn.store(true);
    }
    for (std::size_t axis = 0; axis < snapshot.value().axis_count; ++axis) {
      const auto& value = snapshot.value().axes[axis];
      if (value.sequence != sequence || value.timestamp_ns != snapshot.value().timestamp_ns ||
          value.target != static_cast<double>(sequence * 100 + axis) ||
          value.flags != static_cast<std::uint32_t>(sequence + axis) ||
          value.reserved != 0) {
        torn.store(true);
      }
    }
  }
  writer.join();
  EXPECT_GT(successful_reads, 0U);
  EXPECT_FALSE(torn.load());
}

TEST(IpcTest, RejectsDecreasingPublicationMetadataAndInconsistentControlMetadata) {
  auto [mapping, region] = make_test_region<AxisCommand>(1, IpcRegionKind::command);
  auto writer = region.writer();
  auto reader = region.reader();
  std::array values{command(1.0, 0)};
  ASSERT_TRUE(writer.publish(values, 0, 0).has_value());
  values[0] = command(2.0, 7);
  ASSERT_TRUE(writer.publish(values, 7, 70).has_value());

  values[0] = command(3.0, 6);
  auto decreasing_sequence = writer.publish(values, 6, 80);
  ASSERT_FALSE(decreasing_sequence.has_value());
  EXPECT_EQ(decreasing_sequence.error().code, ErrorCode::invalid_argument);
  values[0] = command(3.0, 8);
  auto decreasing_timestamp = writer.publish(values, 8, 60);
  ASSERT_FALSE(decreasing_timestamp.has_value());
  EXPECT_EQ(decreasing_timestamp.error().code, ErrorCode::invalid_argument);

  auto* publication = reinterpret_cast<policy_runtime::detail::PublicationControl*>(
      static_cast<std::byte*>(mapping.address) + sizeof(policy_runtime::IpcHeader));
  publication->published_sequence.store(99, std::memory_order_seq_cst);
  auto inconsistent = reader.read_latest();
  ASSERT_FALSE(inconsistent.has_value());
  EXPECT_EQ(inconsistent.error().code, ErrorCode::unavailable);
}

TEST(IpcTest, PublicViewsRejectMutatedTopologyBeforeAccessingSlots) {
  auto [mapping, region] = make_test_region<AxisCommand>(1, IpcRegionKind::command);
  auto writer = region.writer();
  auto reader = region.reader();
  std::array values{command(1.0, 1)};
  auto* header = static_cast<policy_runtime::IpcHeader*>(mapping.address);
  header->axis_stride += 8;

  auto write = writer.publish(values, 1, 10);
  ASSERT_FALSE(write.has_value());
  EXPECT_EQ(write.error().code, ErrorCode::protocol);
  auto read = reader.read_latest();
  ASSERT_FALSE(read.has_value());
  EXPECT_EQ(read.error().code, ErrorCode::protocol);
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
  auto sockets = make_socket_pair();
  auto server_result = RobotIoIpcServer::create(sockets[0], 2, kGeneration);
  ASSERT_TRUE(server_result.has_value()) << server_result.error().message;
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());

  auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
  ASSERT_TRUE(client_result.has_value()) << client_result.error().message;
  auto client = std::move(client_result.value());

  EXPECT_EQ(client.axis_count(), 2U);
  EXPECT_EQ(client.command_writer().mapping_access(), SnapshotMappingAccess::read_write);
  EXPECT_EQ(client.feedback_reader().mapping_access(), SnapshotMappingAccess::read_only);
  EXPECT_EQ(server.command_reader().mapping_access(), SnapshotMappingAccess::read_only);
  EXPECT_EQ(server.feedback_writer().mapping_access(), SnapshotMappingAccess::read_write);

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

TEST(IpcTest, RejectsUnixStreamSockets) {
  auto sockets = make_socket_pair(SOCK_STREAM);
  auto server = RobotIoIpcServer::create(sockets[0], 1, kGeneration);
  ASSERT_FALSE(server.has_value());
  EXPECT_EQ(server.error().code, ErrorCode::invalid_argument);
  EXPECT_EQ(::close(sockets[1]), 0);
}

TEST(IpcTest, RejectsMalformedAncillaryAndClosesReceivedDescriptors) {
  for (const auto& [payload_size, descriptor_count] :
       {std::pair{sizeof(policy_runtime::IpcSetupMessage) - 1, std::size_t{2}},
        std::pair{sizeof(policy_runtime::IpcSetupMessage), std::size_t{3}}}) {
    auto sockets = make_socket_pair();
    std::array<int, 3> sent_descriptors{
        ::open("/dev/null", O_RDONLY | O_CLOEXEC),
        ::open("/dev/null", O_RDONLY | O_CLOEXEC),
        ::open("/dev/null", O_RDONLY | O_CLOEXEC)};
    ASSERT_GE(sent_descriptors[0], 0);
    ASSERT_GE(sent_descriptors[1], 0);
    ASSERT_GE(sent_descriptors[2], 0);
    send_raw_setup(sockets[0], payload_size,
                   std::span<const int>(sent_descriptors).first(descriptor_count));
    const auto before = open_descriptor_count();

    auto client = RobotIoClient::connect(sockets[1], kGeneration);

    ASSERT_FALSE(client.has_value());
    EXPECT_EQ(client.error().code, ErrorCode::protocol);
    EXPECT_EQ(open_descriptor_count(), before - 1);
    EXPECT_EQ(::close(sockets[0]), 0);
    for (const auto descriptor : sent_descriptors) {
      EXPECT_EQ(::close(descriptor), 0);
    }
  }
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

TEST(IpcTest, ReusesSlotsWithoutTearingAcrossProcesses) {
  auto sockets = make_socket_pair();
  auto server_result = RobotIoIpcServer::create(sockets[0], 2, kGeneration);
  ASSERT_TRUE(server_result.has_value());
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());

  constexpr std::uint64_t kIterations = 20000;
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    (void)::close(sockets[0]);
    auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
    if (!client_result.has_value()) {
      _exit(10);
    }
    auto client = std::move(client_result.value());
    std::array<AxisCommand, 2> values{};
    for (std::uint64_t sequence = 1; sequence <= kIterations; ++sequence) {
      for (std::size_t axis = 0; axis < values.size(); ++axis) {
        values[axis] = command(static_cast<double>(sequence * 100 + axis), sequence);
        values[axis].flags = static_cast<std::uint32_t>(sequence + axis);
      }
      if (!client.publish_commands(values, sequence,
                                   static_cast<std::int64_t>(sequence * 10))
               .has_value()) {
        _exit(11);
      }
      if ((sequence & 63U) == 0) {
        sched_yield();
      }
    }
    if (!client.close().has_value()) {
      _exit(12);
    }
    _exit(0);
  }

  ASSERT_EQ(::close(sockets[1]), 0);
  int status{};
  std::uint64_t successful_reads = 0;
  bool torn = false;
  for (;;) {
    const auto wait_result = waitpid(child, &status, WNOHANG);
    ASSERT_GE(wait_result, 0);
    if (wait_result == child) {
      break;
    }
    auto snapshot = server.read_commands();
    if (!snapshot.has_value()) {
      continue;
    }
    ++successful_reads;
    const auto sequence = snapshot.value().sequence;
    torn = torn || snapshot.value().timestamp_ns != static_cast<std::int64_t>(sequence * 10);
    for (std::size_t axis = 0; axis < snapshot.value().axis_count; ++axis) {
      const auto& value = snapshot.value().axes[axis];
      torn = torn || value.sequence != sequence ||
             value.timestamp_ns != snapshot.value().timestamp_ns ||
             value.target != static_cast<double>(sequence * 100 + axis) ||
             value.flags != static_cast<std::uint32_t>(sequence + axis) ||
             value.reserved != 0;
    }
  }
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0);
  EXPECT_GT(successful_reads, 0U);
  EXPECT_FALSE(torn);
}

TEST(IpcTest, MovedFromAndDisconnectedServerOperationsAreUnavailable) {
  auto sockets = make_socket_pair();
  auto server_result = RobotIoIpcServer::create(sockets[0], 1, kGeneration);
  ASSERT_TRUE(server_result.has_value());
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());
  auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
  ASSERT_TRUE(client_result.has_value());
  auto client = std::move(client_result.value());

  auto moved_server = std::move(server);
  EXPECT_FALSE(server.read_commands().has_value());
  const std::array feedback_values{feedback(1.0, 1)};
  EXPECT_FALSE(server.publish_feedback(feedback_values, 1, 10).has_value());
  EXPECT_EQ(server.axis_count(), 0U);
  EXPECT_EQ(server.generation(), 0U);

  ASSERT_TRUE(client.close().has_value());
  auto disconnected_read = moved_server.read_commands();
  ASSERT_FALSE(disconnected_read.has_value());
  EXPECT_EQ(disconnected_read.error().code, ErrorCode::unavailable);
  auto disconnected_write = moved_server.publish_feedback(feedback_values, 1, 10);
  ASSERT_FALSE(disconnected_write.has_value());
  EXPECT_EQ(disconnected_write.error().code, ErrorCode::unavailable);

  ASSERT_TRUE(moved_server.close().has_value());
  auto closed_server = std::move(moved_server);
  EXPECT_FALSE(moved_server.read_commands().has_value());
  EXPECT_FALSE(closed_server.read_commands().has_value());
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
