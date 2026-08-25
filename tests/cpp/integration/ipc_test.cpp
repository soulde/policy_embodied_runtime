#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <limits>
#include <span>
#include <string>
#include <thread>
#include <type_traits>
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
using policy_runtime::SnapshotReader;
using policy_runtime::SnapshotRegion;
using policy_runtime::SnapshotWriter;
using policy_runtime::St3215ServoCommand;
using policy_runtime::St3215ServoFeedback;

constexpr std::uint32_t kGeneration = 73;

template <class T>
concept HasCommandWriterAccessor = requires(T& endpoint) { endpoint.command_writer(); };

template <class T>
concept HasFeedbackReaderAccessor = requires(T& endpoint) { endpoint.feedback_reader(); };

template <class T>
concept HasCommandReaderAccessor = requires(T& endpoint) { endpoint.command_reader(); };

template <class T>
concept HasFeedbackWriterAccessor = requires(T& endpoint) { endpoint.feedback_writer(); };

static_assert(!HasCommandWriterAccessor<RobotIoClient>);
static_assert(!HasFeedbackReaderAccessor<RobotIoClient>);
static_assert(!HasCommandReaderAccessor<RobotIoIpcServer>);
static_assert(!HasFeedbackWriterAccessor<RobotIoIpcServer>);
static_assert(!std::is_default_constructible_v<SnapshotReader<AxisCommand>>);
static_assert(!std::is_copy_constructible_v<SnapshotReader<AxisCommand>>);
static_assert(!std::is_copy_assignable_v<SnapshotReader<AxisCommand>>);
static_assert(std::is_move_constructible_v<SnapshotReader<AxisCommand>>);
static_assert(!std::is_default_constructible_v<SnapshotWriter<AxisCommand>>);
static_assert(!std::is_copy_constructible_v<SnapshotWriter<AxisCommand>>);
static_assert(!std::is_copy_assignable_v<SnapshotWriter<AxisCommand>>);
static_assert(std::is_move_constructible_v<SnapshotWriter<AxisCommand>>);

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

struct DescriptorModes {
  std::size_t read_only{};
  std::size_t read_write{};
};

DescriptorModes memfd_modes(const std::string& name) {
  DescriptorModes modes{};
  for (const auto& entry : std::filesystem::directory_iterator("/proc/self/fd")) {
    std::error_code error;
    const auto target = std::filesystem::read_symlink(entry.path(), error).string();
    if (error || target.find("memfd:" + name) == std::string::npos) {
      continue;
    }
    const int fd = std::stoi(entry.path().filename().string());
    const int flags = fcntl(fd, F_GETFL);
    if (flags < 0) {
      continue;
    }
    if ((flags & O_ACCMODE) == O_RDONLY) {
      ++modes.read_only;
    } else if ((flags & O_ACCMODE) == O_RDWR) {
      ++modes.read_write;
    }
  }
  return modes;
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
  auto writer_result = region.writer();
  auto reader_result = region.reader();
  ASSERT_TRUE(writer_result.has_value());
  ASSERT_TRUE(reader_result.has_value());
  auto writer = std::move(writer_result.value());
  auto reader = std::move(reader_result.value());
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
  auto writer_result = region.writer();
  auto reader_result = region.reader();
  ASSERT_TRUE(writer_result.has_value());
  ASSERT_TRUE(reader_result.has_value());
  auto writer_view = std::move(writer_result.value());
  auto reader_view = std::move(reader_result.value());
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
  auto writer_result = region.writer();
  auto reader_result = region.reader();
  ASSERT_TRUE(writer_result.has_value());
  ASSERT_TRUE(reader_result.has_value());
  auto writer = std::move(writer_result.value());
  auto reader = std::move(reader_result.value());
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

TEST(IpcTest, AllowsFirstSequenceZeroWithRealMonotonicTimestamp) {
  auto [mapping, region] = make_test_region<AxisCommand>(1, IpcRegionKind::command);
  auto writer_result = region.writer();
  auto reader_result = region.reader();
  ASSERT_TRUE(writer_result.has_value());
  ASSERT_TRUE(reader_result.has_value());
  auto writer = std::move(writer_result.value());
  auto reader = std::move(reader_result.value());
  std::array values{command(4.0, 0)};
  values[0].timestamp_ns = 987654321;

  ASSERT_TRUE(writer.publish(values, 0, 987654321).has_value());
  auto snapshot = reader.read_latest();
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot.value().sequence, 0U);
  EXPECT_EQ(snapshot.value().timestamp_ns, 987654321);
}

TEST(IpcTest, MappingAccessPreventsReadOnlyRegionFromMintingWriter) {
  auto [mapping, initialized] =
      make_test_region<AxisCommand>(1, IpcRegionKind::command);
  ASSERT_EQ(mprotect(mapping.address, mapping.size, PROT_READ), 0);
  auto read_only = SnapshotRegion<AxisCommand>::attach(
      mapping.address, mapping.size, kGeneration, 1, IpcRegionKind::command,
      SnapshotMappingAccess::read_only);
  ASSERT_TRUE(read_only.has_value());

  auto reader = read_only.value().reader();
  EXPECT_TRUE(reader.has_value());
  auto writer = read_only.value().writer();
  ASSERT_FALSE(writer.has_value());
  EXPECT_EQ(writer.error().code, ErrorCode::invalid_argument);
}

TEST(IpcTest, PublicViewsRejectMutatedTopologyBeforeAccessingSlots) {
  auto [mapping, region] = make_test_region<AxisCommand>(1, IpcRegionKind::command);
  auto writer_result = region.writer();
  auto reader_result = region.reader();
  ASSERT_TRUE(writer_result.has_value());
  ASSERT_TRUE(reader_result.has_value());
  auto writer = std::move(writer_result.value());
  auto reader = std::move(reader_result.value());
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

TEST(IpcTest, ServoMappingsUseTheirIndependentFrozenBound) {
  EXPECT_TRUE(SnapshotRegion<St3215ServoCommand>::mapping_size(0).has_value());
  EXPECT_TRUE(SnapshotRegion<St3215ServoCommand>::mapping_size(32).has_value());
  EXPECT_TRUE(SnapshotRegion<St3215ServoFeedback>::mapping_size(32).has_value());
  EXPECT_FALSE(SnapshotRegion<St3215ServoCommand>::mapping_size(33).has_value());
  EXPECT_FALSE(SnapshotRegion<St3215ServoFeedback>::mapping_size(33).has_value());
}

TEST(IpcTest, VersionTwoTransfersSelfDescribingServoMappingsBothDirections) {
  auto sockets = make_socket_pair();
  auto server_result = RobotIoIpcServer::create(sockets[0], 0U, 2U, kGeneration);
  ASSERT_TRUE(server_result.has_value()) << server_result.error().message;
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());

  auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
  ASSERT_TRUE(client_result.has_value()) << client_result.error().message;
  auto client = std::move(client_result.value());
  EXPECT_EQ(client.axis_count(), 0U);
  EXPECT_EQ(client.servo_count(), 2U);

  std::array<St3215ServoCommand, 2> commands{};
  commands[0] = St3215ServoCommand{4U, 400, 1.25, true, false};
  commands[1] = St3215ServoCommand{4U, 400, -0.5, true, false};
  ASSERT_TRUE(client.publish_servo_commands(commands, 4U, 400).has_value());
  auto daemon_commands = server.read_servo_commands();
  ASSERT_TRUE(daemon_commands.has_value());
  EXPECT_EQ(daemon_commands.value().axis_count, 2U);
  EXPECT_DOUBLE_EQ(daemon_commands.value().axes[1].target_position_rad, -0.5);

  std::array<St3215ServoFeedback, 2> feedback_values{};
  feedback_values[0].feedback_sequence = 7U;
  feedback_values[0].command_sequence = 4U;
  feedback_values[0].timestamp_ns = 700;
  feedback_values[0].position_rad = 1.5;
  feedback_values[0].flags = 1U;
  feedback_values[1] = feedback_values[0];
  feedback_values[1].position_rad = -1.5;
  ASSERT_TRUE(server.publish_servo_feedback(feedback_values, 7U, 700).has_value());
  auto host_feedback = client.read_servo_feedback();
  ASSERT_TRUE(host_feedback.has_value());
  EXPECT_EQ(host_feedback.value().axis_count, 2U);
  EXPECT_DOUBLE_EQ(host_feedback.value().axes[0].position_rad, 1.5);
}

TEST(IpcTest, TransfersSealedCloexecMemfdsAndSnapshotsBothDirections) {
  const auto initial_descriptor_count = open_descriptor_count();
  auto sockets = make_socket_pair();
  EXPECT_EQ(open_descriptor_count(), initial_descriptor_count + 2);
  auto server_result = RobotIoIpcServer::create(sockets[0], 2, kGeneration);
  ASSERT_TRUE(server_result.has_value()) << server_result.error().message;
  auto server = std::move(server_result.value());
  EXPECT_EQ(open_descriptor_count(), initial_descriptor_count + 6);
  EXPECT_EQ(memfd_modes("robot-io-command").read_only, 1U);
  EXPECT_EQ(memfd_modes("robot-io-command").read_write, 1U);
  EXPECT_EQ(memfd_modes("robot-io-feedback").read_only, 1U);
  EXPECT_EQ(memfd_modes("robot-io-feedback").read_write, 1U);
  ASSERT_TRUE(server.send_setup().has_value());
  EXPECT_EQ(open_descriptor_count(), initial_descriptor_count + 4);
  EXPECT_EQ(memfd_modes("robot-io-command").read_only, 1U);
  EXPECT_EQ(memfd_modes("robot-io-command").read_write, 0U);
  EXPECT_EQ(memfd_modes("robot-io-feedback").read_only, 0U);
  EXPECT_EQ(memfd_modes("robot-io-feedback").read_write, 1U);

  auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
  ASSERT_TRUE(client_result.has_value()) << client_result.error().message;
  auto client = std::move(client_result.value());
  EXPECT_EQ(open_descriptor_count(), initial_descriptor_count + 6);

  EXPECT_EQ(client.axis_count(), 2U);
  EXPECT_EQ(memfd_modes("robot-io-command").read_only, 1U);
  EXPECT_EQ(memfd_modes("robot-io-command").read_write, 1U);
  EXPECT_EQ(memfd_modes("robot-io-feedback").read_only, 1U);
  EXPECT_EQ(memfd_modes("robot-io-feedback").read_write, 1U);

  std::array commands{command(3.5, 0), command(-4.5, 0)};
  for (auto& value : commands) {
    value.timestamp_ns = 110;
  }
  ASSERT_TRUE(client.publish_commands(commands, 0, 110).has_value());
  auto daemon_commands = server.read_commands();
  ASSERT_TRUE(daemon_commands.has_value());
  EXPECT_EQ(daemon_commands.value().sequence, 0U);
  EXPECT_EQ(daemon_commands.value().timestamp_ns, 110);
  EXPECT_DOUBLE_EQ(daemon_commands.value().axes[0].target, 3.5);
  EXPECT_DOUBLE_EQ(daemon_commands.value().axes[1].target, -4.5);

  const std::array feedback_values{feedback(1.25, 12), feedback(2.5, 12)};
  ASSERT_TRUE(server.publish_feedback(feedback_values, 12, 120).has_value());
  auto host_feedback = client.read_feedback();
  ASSERT_TRUE(host_feedback.has_value());
  EXPECT_EQ(host_feedback.value().sequence, 12U);
  EXPECT_DOUBLE_EQ(host_feedback.value().axes[1].position, 2.5);

  ASSERT_TRUE(client.close().has_value());
  EXPECT_EQ(open_descriptor_count(), initial_descriptor_count + 3);
  ASSERT_TRUE(server.close().has_value());
  EXPECT_EQ(open_descriptor_count(), initial_descriptor_count);
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
  auto moved_client = std::move(client);
  EXPECT_FALSE(server.read_commands().has_value());
  const std::array feedback_values{feedback(1.0, 1)};
  EXPECT_FALSE(server.publish_feedback(feedback_values, 1, 10).has_value());
  EXPECT_EQ(server.axis_count(), 0U);
  EXPECT_EQ(server.generation(), 0U);
  const std::array commands{command(2.0, 0)};
  EXPECT_FALSE(client.publish_commands(commands, 0, 123).has_value());
  EXPECT_FALSE(client.read_feedback().has_value());
  EXPECT_EQ(client.axis_count(), 0U);
  EXPECT_EQ(client.generation(), 0U);

  ASSERT_TRUE(moved_client.close().has_value());
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
  auto closed_client = std::move(moved_client);
  EXPECT_FALSE(moved_client.publish_commands(commands, 0, 123).has_value());
  EXPECT_FALSE(closed_client.publish_commands(commands, 0, 123).has_value());
}

TEST(IpcTest, UnexpectedPacketThenDisconnectCannotMaskEndpointFailure) {
  {
    auto sockets = make_socket_pair();
    const int peer = dup(sockets[1]);
    ASSERT_GE(peer, 0);
    auto server_result = RobotIoIpcServer::create(sockets[0], 1, kGeneration);
    ASSERT_TRUE(server_result.has_value());
    auto server = std::move(server_result.value());
    ASSERT_TRUE(server.send_setup().has_value());
    auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
    ASSERT_TRUE(client_result.has_value());
    auto client = std::move(client_result.value());
    const std::byte unexpected{0x5a};
    ASSERT_EQ(send(peer, &unexpected, sizeof(unexpected), MSG_NOSIGNAL), 1);
    ASSERT_TRUE(client.close().has_value());
    ASSERT_EQ(close(peer), 0);

    auto health = server.check_peer();
    ASSERT_FALSE(health.has_value());
    EXPECT_TRUE(health.error().code == ErrorCode::protocol ||
                health.error().code == ErrorCode::unavailable);
    auto read = server.read_commands();
    ASSERT_FALSE(read.has_value());
    EXPECT_TRUE(read.error().code == ErrorCode::protocol ||
                read.error().code == ErrorCode::unavailable);
    const std::array values{feedback(1.0, 0)};
    auto publish = server.publish_feedback(values, 0, 123);
    ASSERT_FALSE(publish.has_value());
    EXPECT_TRUE(publish.error().code == ErrorCode::protocol ||
                publish.error().code == ErrorCode::unavailable);
  }

  {
    auto sockets = make_socket_pair();
    const int peer = dup(sockets[0]);
    ASSERT_GE(peer, 0);
    auto server_result = RobotIoIpcServer::create(sockets[0], 1, kGeneration);
    ASSERT_TRUE(server_result.has_value());
    auto server = std::move(server_result.value());
    ASSERT_TRUE(server.send_setup().has_value());
    auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
    ASSERT_TRUE(client_result.has_value());
    auto client = std::move(client_result.value());
    const std::byte unexpected{0x2a};
    ASSERT_EQ(send(peer, &unexpected, sizeof(unexpected), MSG_NOSIGNAL), 1);
    ASSERT_TRUE(server.close().has_value());
    ASSERT_EQ(close(peer), 0);

    auto health = client.check_peer();
    ASSERT_FALSE(health.has_value());
    EXPECT_TRUE(health.error().code == ErrorCode::protocol ||
                health.error().code == ErrorCode::unavailable);
    auto read = client.read_feedback();
    ASSERT_FALSE(read.has_value());
    EXPECT_TRUE(read.error().code == ErrorCode::protocol ||
                read.error().code == ErrorCode::unavailable);
    const std::array values{command(1.0, 0)};
    auto publish = client.publish_commands(values, 0, 123);
    ASSERT_FALSE(publish.has_value());
    EXPECT_TRUE(publish.error().code == ErrorCode::protocol ||
                publish.error().code == ErrorCode::unavailable);
  }
}

TEST(IpcTest, PeerCheckConsumesOnlyOneCompleteUnexpectedPacket) {
  {
    auto sockets = make_socket_pair();
    const int inspection_fd = dup(sockets[0]);
    const int peer = dup(sockets[1]);
    ASSERT_GE(inspection_fd, 0);
    ASSERT_GE(peer, 0);
    auto server_result = RobotIoIpcServer::create(sockets[0], 1, kGeneration);
    ASSERT_TRUE(server_result.has_value());
    auto server = std::move(server_result.value());
    ASSERT_TRUE(server.send_setup().has_value());
    auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
    ASSERT_TRUE(client_result.has_value());
    auto client = std::move(client_result.value());

    std::array<std::byte, 4096> oversized{};
    oversized.fill(std::byte{0xa5});
    const std::byte marker{0x3c};
    ASSERT_EQ(send(peer, oversized.data(), oversized.size(), MSG_NOSIGNAL),
              static_cast<ssize_t>(oversized.size()));
    ASSERT_EQ(send(peer, &marker, sizeof(marker), MSG_NOSIGNAL), 1);

    auto health = server.check_peer();
    ASSERT_FALSE(health.has_value());
    EXPECT_EQ(health.error().code, ErrorCode::protocol);
    std::byte received{};
    ASSERT_EQ(recv(inspection_fd, &received, sizeof(received), MSG_DONTWAIT), 1);
    EXPECT_EQ(received, marker);
    EXPECT_EQ(close(inspection_fd), 0);
    EXPECT_EQ(close(peer), 0);
  }

  {
    auto sockets = make_socket_pair();
    const int peer = dup(sockets[0]);
    const int inspection_fd = dup(sockets[1]);
    ASSERT_GE(peer, 0);
    ASSERT_GE(inspection_fd, 0);
    auto server_result = RobotIoIpcServer::create(sockets[0], 1, kGeneration);
    ASSERT_TRUE(server_result.has_value());
    auto server = std::move(server_result.value());
    ASSERT_TRUE(server.send_setup().has_value());
    auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
    ASSERT_TRUE(client_result.has_value());
    auto client = std::move(client_result.value());

    std::array<std::byte, 4096> oversized{};
    oversized.fill(std::byte{0x5a});
    const std::byte marker{0xc3};
    ASSERT_EQ(send(peer, oversized.data(), oversized.size(), MSG_NOSIGNAL),
              static_cast<ssize_t>(oversized.size()));
    ASSERT_EQ(send(peer, &marker, sizeof(marker), MSG_NOSIGNAL), 1);

    auto health = client.check_peer();
    ASSERT_FALSE(health.has_value());
    EXPECT_EQ(health.error().code, ErrorCode::protocol);
    std::byte received{};
    ASSERT_EQ(recv(inspection_fd, &received, sizeof(received), MSG_DONTWAIT), 1);
    EXPECT_EQ(received, marker);
    EXPECT_EQ(close(peer), 0);
    EXPECT_EQ(close(inspection_fd), 0);
  }
}

TEST(IpcTest, FloodingPeerCannotDelayEndpointOperation) {
  auto sockets = make_socket_pair();
  const int peer = dup(sockets[1]);
  ASSERT_GE(peer, 0);
  auto server_result = RobotIoIpcServer::create(sockets[0], 1, kGeneration);
  ASSERT_TRUE(server_result.has_value());
  auto server = std::move(server_result.value());
  ASSERT_TRUE(server.send_setup().has_value());
  auto client_result = RobotIoClient::connect(sockets[1], kGeneration);
  ASSERT_TRUE(client_result.has_value());
  auto client = std::move(client_result.value());

  const int flags = fcntl(peer, F_GETFL);
  ASSERT_GE(flags, 0);
  ASSERT_EQ(fcntl(peer, F_SETFL, flags | O_NONBLOCK), 0);
  const std::byte unexpected{0x7e};
  std::size_t queued = 0;
  while (send(peer, &unexpected, sizeof(unexpected), MSG_NOSIGNAL) == 1) {
    ++queued;
  }
  ASSERT_GT(queued, 0U);
  ASSERT_TRUE(errno == EAGAIN || errno == EWOULDBLOCK);

  std::atomic<bool> stop{false};
  std::vector<std::thread> flooders;
  for (int index = 0; index < 8; ++index) {
    flooders.emplace_back([&] {
      while (!stop.load(std::memory_order_relaxed)) {
        if (send(peer, &unexpected, sizeof(unexpected), MSG_NOSIGNAL) < 0 &&
            errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          stop.store(true, std::memory_order_relaxed);
        }
      }
    });
  }
  std::thread deadline([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    stop.store(true, std::memory_order_relaxed);
  });

  const auto started = std::chrono::steady_clock::now();
  auto read = server.read_commands();
  const auto elapsed = std::chrono::steady_clock::now() - started;
  stop.store(true, std::memory_order_relaxed);
  for (auto& flooder : flooders) {
    flooder.join();
  }
  deadline.join();

  ASSERT_FALSE(read.has_value());
  EXPECT_EQ(read.error().code, ErrorCode::protocol);
  EXPECT_LT(elapsed, std::chrono::milliseconds(50));
  EXPECT_EQ(close(peer), 0);
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
