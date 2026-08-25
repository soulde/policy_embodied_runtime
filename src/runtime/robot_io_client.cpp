#include "policy_runtime/runtime/robot_io_client.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <string>
#include <system_error>
#include <utility>

#include <sys/mman.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

namespace policy_runtime {
namespace {

Error system_error(ErrorCode code, const char* operation, int error_number = errno) {
  return {code, std::string(operation) + ": " +
                    std::error_code(error_number, std::generic_category()).message()};
}

class ScopedFd {
 public:
  ScopedFd() noexcept = default;
  explicit ScopedFd(int fd) noexcept : fd_(fd) {}
  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;
  ScopedFd(ScopedFd&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
  ScopedFd& operator=(ScopedFd&& other) noexcept {
    if (this != &other) {
      if (fd_ >= 0) {
        ::close(fd_);
      }
      fd_ = std::exchange(other.fd_, -1);
    }
    return *this;
  }
  ~ScopedFd() {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }
  int get() const noexcept { return fd_; }
  int release() noexcept { return std::exchange(fd_, -1); }

 private:
  int fd_{-1};
};

class ScopedMapping {
 public:
  ScopedMapping(void* address = nullptr, std::size_t size = 0) noexcept
      : address_(address), size_(size) {}
  ScopedMapping(const ScopedMapping&) = delete;
  ScopedMapping& operator=(const ScopedMapping&) = delete;
  ScopedMapping(ScopedMapping&& other) noexcept
      : address_(std::exchange(other.address_, nullptr)), size_(other.size_) {}
  ScopedMapping& operator=(ScopedMapping&& other) noexcept {
    if (this != &other) {
      if (address_ != nullptr) {
        munmap(address_, size_);
      }
      address_ = std::exchange(other.address_, nullptr);
      size_ = other.size_;
    }
    return *this;
  }
  ~ScopedMapping() {
    if (address_ != nullptr) {
      munmap(address_, size_);
    }
  }
  void* release() noexcept { return std::exchange(address_, nullptr); }

 private:
  void* address_{};
  std::size_t size_{};
};

template <std::size_t DescriptorCount>
struct AncillaryStorage {
  alignas(cmsghdr) std::array<std::byte,
                              CMSG_SPACE(DescriptorCount * sizeof(int))>
      bytes{};
};

static_assert(alignof(AncillaryStorage<2>) >= alignof(cmsghdr));
static_assert(alignof(AncillaryStorage<2>) >= alignof(int));
static_assert(sizeof(AncillaryStorage<2>) == CMSG_SPACE(2 * sizeof(int)));
static_assert(sizeof(AncillaryStorage<4>) == CMSG_SPACE(4 * sizeof(int)));

Result<void> prepare_socket(int socket_fd) {
  if (socket_fd < 0) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "invalid IPC socket descriptor"});
  }
  int domain{};
  socklen_t domain_size = sizeof(domain);
  if (getsockopt(socket_fd, SOL_SOCKET, SO_DOMAIN, &domain, &domain_size) != 0) {
    return Result<void>::failure(system_error(ErrorCode::io, "getsockopt(SO_DOMAIN)"));
  }
  int type{};
  socklen_t type_size = sizeof(type);
  if (getsockopt(socket_fd, SOL_SOCKET, SO_TYPE, &type, &type_size) != 0) {
    return Result<void>::failure(system_error(ErrorCode::io, "getsockopt(SO_TYPE)"));
  }
  if (domain != AF_UNIX || type != SOCK_SEQPACKET) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "IPC socket must be AF_UNIX SOCK_SEQPACKET"});
  }
  const int descriptor_flags = fcntl(socket_fd, F_GETFD);
  if (descriptor_flags < 0 ||
      fcntl(socket_fd, F_SETFD, descriptor_flags | FD_CLOEXEC) != 0) {
    return Result<void>::failure(system_error(ErrorCode::io, "fcntl(FD_CLOEXEC)"));
  }
  return Result<void>::success();
}

Result<void> check_peer_fd(int socket_fd) {
  if (socket_fd < 0) {
    return Result<void>::failure({ErrorCode::unavailable, "IPC socket is closed"});
  }
  pollfd descriptor{socket_fd, static_cast<short>(POLLIN | POLLRDHUP), 0};
  int polled{};
  for (;;) {
    polled = poll(&descriptor, 1, 0);
    if (polled < 0 && errno == EINTR) {
      continue;
    }
    break;
  }
  if (polled < 0) {
    return Result<void>::failure(system_error(ErrorCode::io, "poll(IPC peer)"));
  }
  if (polled == 0) {
    return Result<void>::success();
  }

  if ((descriptor.revents & POLLIN) != 0) {
    std::array<std::byte, 64> discarded{};
    iovec vector{discarded.data(), discarded.size()};
    msghdr message{};
    message.msg_iov = &vector;
    message.msg_iovlen = 1;
    ssize_t received{};
    do {
      // MSG_TRUNC returns the original packet length while SOCK_SEQPACKET
      // atomically consumes the whole packet, even when it exceeds discarded.
      received = recvmsg(socket_fd, &message, MSG_DONTWAIT | MSG_TRUNC);
    } while (received < 0 && errno == EINTR);
    if (received > 0 ||
        (received == 0 &&
         (descriptor.revents & (POLLHUP | POLLRDHUP)) == 0)) {
      return Result<void>::failure(
          {ErrorCode::protocol, "unexpected packet after IPC setup"});
    }
    if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
        errno != ECONNRESET && errno != ENOTCONN) {
      return Result<void>::failure(
          system_error(ErrorCode::io, "recvmsg(unexpected IPC packet)"));
    }
  }
  if ((descriptor.revents & (POLLRDHUP | POLLHUP | POLLERR | POLLNVAL)) != 0) {
    return Result<void>::failure({ErrorCode::unavailable, "IPC peer disconnected"});
  }
  return Result<void>::success();
}

Result<void> validate_setup(const IpcSetupMessage& setup,
                            std::uint32_t expected_generation) {
  if (setup.magic != kRobotIoSetupMagic ||
      setup.abi_version != kRobotIoIpcAbiVersion) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC setup version"});
  }
  if (setup.generation != expected_generation) {
    return Result<void>::failure({ErrorCode::unavailable, "stale daemon generation"});
  }
  if (setup.axis_count > kRobotIoMaximumAxes || setup.descriptor_count != 2 ||
      setup.command_axis_stride != sizeof(AxisCommand) ||
      setup.feedback_axis_stride != sizeof(AxisFeedback) || setup.reserved[0] != 0 ||
      setup.reserved[1] != 0) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC setup layout"});
  }
  auto command_size = SnapshotRegion<AxisCommand>::mapping_size(setup.axis_count);
  auto feedback_size = SnapshotRegion<AxisFeedback>::mapping_size(setup.axis_count);
  if (!command_size.has_value() || !feedback_size.has_value() ||
      setup.command_mapping_size != command_size.value() ||
      setup.feedback_mapping_size != feedback_size.value()) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC setup mapping size"});
  }
  return Result<void>::success();
}

Result<void> validate_setup_v2(const IpcSetupMessageV2& setup,
                               std::uint32_t expected_generation) {
  constexpr std::array<char, 8> kMagic{'R', 'I', 'O', 'F', 'D', '0', '0', '2'};
  if (setup.magic != kMagic || setup.abi_version != kRobotIoIpcAbiVersion2) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC v2 setup version"});
  }
  if (setup.generation != expected_generation) {
    return Result<void>::failure({ErrorCode::unavailable, "stale daemon generation"});
  }
  if (setup.descriptor_count != 4U || setup.reserved0 != 0U ||
      setup.reserved1 != 0U) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC v2 setup header"});
  }
  const std::array<std::uint32_t, 4> kinds{
      static_cast<std::uint32_t>(IpcRegionKind::command),
      static_cast<std::uint32_t>(IpcRegionKind::feedback),
      static_cast<std::uint32_t>(IpcRegionKind::servo_command),
      static_cast<std::uint32_t>(IpcRegionKind::servo_feedback)};
  const std::array<std::uint32_t, 4> strides{
      sizeof(AxisCommand), sizeof(AxisFeedback), sizeof(St3215ServoCommand),
      sizeof(St3215ServoFeedback)};
  if (setup.mappings[0].item_count > kRobotIoMaximumAxes ||
      setup.mappings[1].item_count != setup.mappings[0].item_count ||
      setup.mappings[2].item_count == 0U ||
      setup.mappings[2].item_count > kRobotIoMaximumServos ||
      setup.mappings[3].item_count != setup.mappings[2].item_count) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC v2 topology"});
  }
  for (std::size_t index = 0; index < setup.mappings.size(); ++index) {
    const auto& mapping = setup.mappings[index];
    if (mapping.region_kind != kinds[index] ||
        mapping.item_stride != strides[index] || mapping.reserved != 0U) {
      return Result<void>::failure({ErrorCode::protocol, "invalid IPC v2 mapping role"});
    }
  }
  auto axis_command = SnapshotRegion<AxisCommand>::mapping_size(
      setup.mappings[0].item_count);
  auto axis_feedback = SnapshotRegion<AxisFeedback>::mapping_size(
      setup.mappings[1].item_count);
  auto servo_command = SnapshotRegion<St3215ServoCommand>::mapping_size(
      setup.mappings[2].item_count);
  auto servo_feedback = SnapshotRegion<St3215ServoFeedback>::mapping_size(
      setup.mappings[3].item_count);
  if (!axis_command.has_value() || !axis_feedback.has_value() ||
      !servo_command.has_value() || !servo_feedback.has_value() ||
      setup.mappings[0].mapping_size != axis_command.value() ||
      setup.mappings[1].mapping_size != axis_feedback.value() ||
      setup.mappings[2].mapping_size != servo_command.value() ||
      setup.mappings[3].mapping_size != servo_feedback.value()) {
    return Result<void>::failure({ErrorCode::protocol, "invalid IPC v2 mapping size"});
  }
  return Result<void>::success();
}

Result<void> validate_received_fd(int fd, std::uint64_t expected_size,
                                  SnapshotMappingAccess expected_access) {
  if (expected_size > static_cast<std::uint64_t>(std::numeric_limits<off_t>::max())) {
    return Result<void>::failure({ErrorCode::protocol, "IPC mapping size exceeds off_t"});
  }
  struct stat status {};
  if (fstat(fd, &status) != 0) {
    return Result<void>::failure(system_error(ErrorCode::io, "fstat(memfd)"));
  }
  if (!S_ISREG(status.st_mode) || status.st_size != static_cast<off_t>(expected_size)) {
    return Result<void>::failure({ErrorCode::protocol, "received descriptor size mismatch"});
  }
  const int seals = fcntl(fd, F_GET_SEALS);
  constexpr int kRequiredSeals = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
  if (seals < 0 || (seals & kRequiredSeals) != kRequiredSeals) {
    return Result<void>::failure({ErrorCode::protocol, "received descriptor is not sealed"});
  }
  const int status_flags = fcntl(fd, F_GETFL);
  const int required_mode = expected_access == SnapshotMappingAccess::read_write
                                ? O_RDWR
                                : O_RDONLY;
  if (status_flags < 0 || (status_flags & O_ACCMODE) != required_mode) {
    return Result<void>::failure(
        {ErrorCode::protocol, "received descriptor has incorrect access mode"});
  }
  const int descriptor_flags = fcntl(fd, F_GETFD);
  if (descriptor_flags < 0 ||
      fcntl(fd, F_SETFD, descriptor_flags | FD_CLOEXEC) != 0) {
    return Result<void>::failure(system_error(ErrorCode::io, "fcntl(FD_CLOEXEC)"));
  }
  return Result<void>::success();
}

}  // namespace

Result<RobotIoClient> RobotIoClient::connect(int connected_socket,
                                             std::uint32_t expected_generation) {
  ScopedFd socket(connected_socket);
  const auto socket_result = prepare_socket(socket.get());
  if (!socket_result.has_value()) {
    return Result<RobotIoClient>::failure(socket_result.error());
  }

  alignas(IpcSetupMessageV2)
      std::array<std::byte, sizeof(IpcSetupMessageV2)> setup_storage{};
  AncillaryStorage<4> control{};
  iovec vector{setup_storage.data(), setup_storage.size()};
  msghdr message{};
  message.msg_iov = &vector;
  message.msg_iovlen = 1;
  message.msg_control = control.bytes.data();
  message.msg_controllen = control.bytes.size();
  ssize_t received{};
  do {
    received = recvmsg(socket.get(), &message, MSG_CMSG_CLOEXEC);
  } while (received < 0 && errno == EINTR);
  if (received == 0) {
    return Result<RobotIoClient>::failure(
        {ErrorCode::unavailable, "IPC peer disconnected during setup"});
  }
  if (received < 0) {
    const auto code = errno == ECONNRESET || errno == ENOTCONN
                          ? ErrorCode::unavailable
                          : ErrorCode::io;
    return Result<RobotIoClient>::failure(system_error(code, "recvmsg(SCM_RIGHTS)"));
  }

  std::array<ScopedFd, 4> descriptors{};
  std::size_t descriptor_count = 0;
  std::size_t rights_messages = 0;
  bool ancillary_invalid = false;
  for (auto* item = CMSG_FIRSTHDR(&message); item != nullptr;
       item = CMSG_NXTHDR(&message, item)) {
    if (item->cmsg_level != SOL_SOCKET || item->cmsg_type != SCM_RIGHTS ||
        item->cmsg_len < CMSG_LEN(0)) {
      ancillary_invalid = true;
      continue;
    }
    ++rights_messages;
    const auto payload_size = item->cmsg_len - CMSG_LEN(0);
    if (payload_size % sizeof(int) != 0) {
      ancillary_invalid = true;
      continue;
    }
    const auto item_descriptor_count = payload_size / sizeof(int);
    if ((item_descriptor_count != 2 && item_descriptor_count != 4) ||
        rights_messages != 1) {
      ancillary_invalid = true;
    }
    const auto* received_descriptors =
        reinterpret_cast<const int*>(CMSG_DATA(item));
    for (std::size_t index = 0; index < item_descriptor_count; ++index) {
      if (descriptor_count < descriptors.size()) {
        descriptors[descriptor_count++] = ScopedFd(received_descriptors[index]);
      } else {
        (void)::close(received_descriptors[index]);
      }
    }
  }
  const auto received_size = static_cast<std::size_t>(received);
  const bool is_v1 = received_size == sizeof(IpcSetupMessage);
  const bool is_v2 = received_size == sizeof(IpcSetupMessageV2);
  const std::size_t expected_descriptors = is_v2 ? 4U : 2U;
  if ((!is_v1 && !is_v2) ||
      (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0 ||
      descriptor_count != expected_descriptors || rights_messages != 1 ||
      ancillary_invalid) {
    return Result<RobotIoClient>::failure(
        {ErrorCode::protocol, "malformed IPC setup packet or ancillary data"});
  }

  IpcSetupMessage setup{};
  IpcSetupMessageV2 setup_v2{};
  if (is_v1) {
    std::memcpy(&setup, setup_storage.data(), sizeof(setup));
  } else {
    std::memcpy(&setup_v2, setup_storage.data(), sizeof(setup_v2));
  }
  const auto setup_result = is_v1
                                ? validate_setup(setup, expected_generation)
                                : validate_setup_v2(setup_v2,
                                                    expected_generation);
  if (!setup_result.has_value()) {
    return Result<RobotIoClient>::failure(setup_result.error());
  }
  const auto axis_count =
      is_v1 ? setup.axis_count : setup_v2.mappings[0].item_count;
  const auto servo_count = is_v1 ? 0U : setup_v2.mappings[2].item_count;
  const auto command_mapping_size = is_v1
                                        ? setup.command_mapping_size
                                        : setup_v2.mappings[0].mapping_size;
  const auto feedback_mapping_size = is_v1
                                         ? setup.feedback_mapping_size
                                         : setup_v2.mappings[1].mapping_size;
  const auto command_fd_result =
      validate_received_fd(descriptors[0].get(), command_mapping_size,
                           SnapshotMappingAccess::read_write);
  if (!command_fd_result.has_value()) {
    return Result<RobotIoClient>::failure(command_fd_result.error());
  }
  const auto feedback_fd_result =
      validate_received_fd(descriptors[1].get(), feedback_mapping_size,
                           SnapshotMappingAccess::read_only);
  if (!feedback_fd_result.has_value()) {
    return Result<RobotIoClient>::failure(feedback_fd_result.error());
  }
  const auto servo_command_mapping_size =
      is_v2 ? setup_v2.mappings[2].mapping_size : 0U;
  const auto servo_feedback_mapping_size =
      is_v2 ? setup_v2.mappings[3].mapping_size : 0U;
  if (is_v2) {
    const auto servo_command_fd_result = validate_received_fd(
        descriptors[2].get(), servo_command_mapping_size,
        SnapshotMappingAccess::read_write);
    if (!servo_command_fd_result.has_value()) {
      return Result<RobotIoClient>::failure(servo_command_fd_result.error());
    }
    const auto servo_feedback_fd_result = validate_received_fd(
        descriptors[3].get(), servo_feedback_mapping_size,
        SnapshotMappingAccess::read_only);
    if (!servo_feedback_fd_result.has_value()) {
      return Result<RobotIoClient>::failure(servo_feedback_fd_result.error());
    }
  }

  void* command_address = mmap(nullptr, command_mapping_size,
                               PROT_READ | PROT_WRITE, MAP_SHARED,
                               descriptors[0].get(), 0);
  if (command_address == MAP_FAILED) {
    return Result<RobotIoClient>::failure(system_error(ErrorCode::io, "mmap(command)"));
  }
  ScopedMapping command_mapping(command_address, command_mapping_size);
  void* feedback_address = mmap(nullptr, feedback_mapping_size, PROT_READ,
                                MAP_SHARED, descriptors[1].get(), 0);
  if (feedback_address == MAP_FAILED) {
    return Result<RobotIoClient>::failure(system_error(ErrorCode::io, "mmap(feedback)"));
  }
  ScopedMapping feedback_mapping(feedback_address, feedback_mapping_size);

  auto command_region = SnapshotRegion<AxisCommand>::attach(
      command_address, command_mapping_size, expected_generation, axis_count,
      IpcRegionKind::command, SnapshotMappingAccess::read_write);
  if (!command_region.has_value()) {
    return Result<RobotIoClient>::failure(command_region.error());
  }
  auto feedback_region = SnapshotRegion<AxisFeedback>::attach(
      feedback_address, feedback_mapping_size, expected_generation, axis_count,
      IpcRegionKind::feedback, SnapshotMappingAccess::read_only);
  if (!feedback_region.has_value()) {
    return Result<RobotIoClient>::failure(feedback_region.error());
  }

  auto command_writer = command_region.value().writer();
  if (!command_writer.has_value()) {
    return Result<RobotIoClient>::failure(command_writer.error());
  }
  auto feedback_reader = feedback_region.value().reader();
  if (!feedback_reader.has_value()) {
    return Result<RobotIoClient>::failure(feedback_reader.error());
  }

  ScopedMapping servo_command_mapping;
  ScopedMapping servo_feedback_mapping;
  std::optional<SnapshotWriter<St3215ServoCommand>> servo_command_writer;
  std::optional<SnapshotReader<St3215ServoFeedback>> servo_feedback_reader;
  if (is_v2) {
    void* servo_command_address =
        mmap(nullptr, servo_command_mapping_size, PROT_READ | PROT_WRITE,
             MAP_SHARED, descriptors[2].get(), 0);
    if (servo_command_address == MAP_FAILED) {
      return Result<RobotIoClient>::failure(
          system_error(ErrorCode::io, "mmap(servo command)"));
    }
    servo_command_mapping =
        ScopedMapping(servo_command_address, servo_command_mapping_size);
    void* servo_feedback_address =
        mmap(nullptr, servo_feedback_mapping_size, PROT_READ, MAP_SHARED,
             descriptors[3].get(), 0);
    if (servo_feedback_address == MAP_FAILED) {
      return Result<RobotIoClient>::failure(
          system_error(ErrorCode::io, "mmap(servo feedback)"));
    }
    servo_feedback_mapping =
        ScopedMapping(servo_feedback_address, servo_feedback_mapping_size);
    auto servo_command_region = SnapshotRegion<St3215ServoCommand>::attach(
        servo_command_address, servo_command_mapping_size,
        expected_generation, servo_count, IpcRegionKind::servo_command,
        SnapshotMappingAccess::read_write);
    if (!servo_command_region.has_value()) {
      return Result<RobotIoClient>::failure(servo_command_region.error());
    }
    auto servo_feedback_region = SnapshotRegion<St3215ServoFeedback>::attach(
        servo_feedback_address, servo_feedback_mapping_size,
        expected_generation, servo_count, IpcRegionKind::servo_feedback,
        SnapshotMappingAccess::read_only);
    if (!servo_feedback_region.has_value()) {
      return Result<RobotIoClient>::failure(servo_feedback_region.error());
    }
    auto writer = servo_command_region.value().writer();
    auto reader = servo_feedback_region.value().reader();
    if (!writer.has_value()) {
      return Result<RobotIoClient>::failure(writer.error());
    }
    if (!reader.has_value()) {
      return Result<RobotIoClient>::failure(reader.error());
    }
    servo_command_writer.emplace(std::move(writer.value()));
    servo_feedback_reader.emplace(std::move(reader.value()));
  }

  RobotIoClient client(
      socket.release(), descriptors[0].release(), descriptors[1].release(),
      command_mapping.release(), command_mapping_size,
      feedback_mapping.release(), feedback_mapping_size,
      is_v2 ? descriptors[2].release() : -1,
      is_v2 ? descriptors[3].release() : -1,
      is_v2 ? servo_command_mapping.release() : nullptr,
      servo_command_mapping_size,
      is_v2 ? servo_feedback_mapping.release() : nullptr,
      servo_feedback_mapping_size, axis_count, expected_generation,
      std::move(command_writer.value()), std::move(feedback_reader.value()),
      servo_count, std::move(servo_command_writer),
      std::move(servo_feedback_reader));
  return Result<RobotIoClient>::success(std::move(client));
}

RobotIoClient::RobotIoClient(int socket_fd, int command_writer_fd,
                             int feedback_reader_fd, void* command_mapping,
                             std::size_t command_mapping_size, void* feedback_mapping,
                             std::size_t feedback_mapping_size,
                             int servo_command_writer_fd,
                             int servo_feedback_reader_fd,
                             void* servo_command_mapping,
                             std::size_t servo_command_mapping_size,
                             void* servo_feedback_mapping,
                             std::size_t servo_feedback_mapping_size,
                             std::uint32_t axis_count, std::uint32_t generation,
                             SnapshotWriter<AxisCommand> command_writer,
                             SnapshotReader<AxisFeedback> feedback_reader,
                             std::uint32_t servo_count,
                             std::optional<SnapshotWriter<St3215ServoCommand>>
                                 servo_command_writer,
                             std::optional<SnapshotReader<St3215ServoFeedback>>
                                 servo_feedback_reader) noexcept
    : socket_fd_(socket_fd),
      command_writer_fd_(command_writer_fd),
      feedback_reader_fd_(feedback_reader_fd),
      servo_command_writer_fd_(servo_command_writer_fd),
      servo_feedback_reader_fd_(servo_feedback_reader_fd),
      command_mapping_(command_mapping),
      command_mapping_size_(command_mapping_size),
      feedback_mapping_(feedback_mapping),
      feedback_mapping_size_(feedback_mapping_size),
      servo_command_mapping_(servo_command_mapping),
      servo_command_mapping_size_(servo_command_mapping_size),
      servo_feedback_mapping_(servo_feedback_mapping),
      servo_feedback_mapping_size_(servo_feedback_mapping_size),
      axis_count_(axis_count),
      servo_count_(servo_count),
      generation_(generation),
      command_writer_(std::move(command_writer)),
      feedback_reader_(std::move(feedback_reader)),
      servo_command_writer_(std::move(servo_command_writer)),
      servo_feedback_reader_(std::move(servo_feedback_reader)) {}

RobotIoClient::RobotIoClient(RobotIoClient&& other) noexcept
    : socket_fd_(std::exchange(other.socket_fd_, -1)),
      command_writer_fd_(std::exchange(other.command_writer_fd_, -1)),
      feedback_reader_fd_(std::exchange(other.feedback_reader_fd_, -1)),
      servo_command_writer_fd_(
          std::exchange(other.servo_command_writer_fd_, -1)),
      servo_feedback_reader_fd_(
          std::exchange(other.servo_feedback_reader_fd_, -1)),
      command_mapping_(std::exchange(other.command_mapping_, nullptr)),
      command_mapping_size_(std::exchange(other.command_mapping_size_, 0)),
      feedback_mapping_(std::exchange(other.feedback_mapping_, nullptr)),
      feedback_mapping_size_(std::exchange(other.feedback_mapping_size_, 0)),
      servo_command_mapping_(
          std::exchange(other.servo_command_mapping_, nullptr)),
      servo_command_mapping_size_(
          std::exchange(other.servo_command_mapping_size_, 0)),
      servo_feedback_mapping_(
          std::exchange(other.servo_feedback_mapping_, nullptr)),
      servo_feedback_mapping_size_(
          std::exchange(other.servo_feedback_mapping_size_, 0)),
      axis_count_(std::exchange(other.axis_count_, 0)),
      servo_count_(std::exchange(other.servo_count_, 0)),
      generation_(std::exchange(other.generation_, 0)),
      command_writer_(std::move(other.command_writer_)),
      feedback_reader_(std::move(other.feedback_reader_)),
      servo_command_writer_(std::move(other.servo_command_writer_)),
      servo_feedback_reader_(std::move(other.servo_feedback_reader_)) {
  other.command_writer_.reset();
  other.feedback_reader_.reset();
  other.servo_command_writer_.reset();
  other.servo_feedback_reader_.reset();
}

RobotIoClient& RobotIoClient::operator=(RobotIoClient&& other) noexcept {
  if (this != &other) {
    release_noexcept();
    socket_fd_ = std::exchange(other.socket_fd_, -1);
    command_writer_fd_ = std::exchange(other.command_writer_fd_, -1);
    feedback_reader_fd_ = std::exchange(other.feedback_reader_fd_, -1);
    servo_command_writer_fd_ =
        std::exchange(other.servo_command_writer_fd_, -1);
    servo_feedback_reader_fd_ =
        std::exchange(other.servo_feedback_reader_fd_, -1);
    command_mapping_ = std::exchange(other.command_mapping_, nullptr);
    command_mapping_size_ = std::exchange(other.command_mapping_size_, 0);
    feedback_mapping_ = std::exchange(other.feedback_mapping_, nullptr);
    feedback_mapping_size_ = std::exchange(other.feedback_mapping_size_, 0);
    servo_command_mapping_ =
        std::exchange(other.servo_command_mapping_, nullptr);
    servo_command_mapping_size_ =
        std::exchange(other.servo_command_mapping_size_, 0);
    servo_feedback_mapping_ =
        std::exchange(other.servo_feedback_mapping_, nullptr);
    servo_feedback_mapping_size_ =
        std::exchange(other.servo_feedback_mapping_size_, 0);
    axis_count_ = std::exchange(other.axis_count_, 0);
    servo_count_ = std::exchange(other.servo_count_, 0);
    generation_ = std::exchange(other.generation_, 0);
    command_writer_ = std::move(other.command_writer_);
    feedback_reader_ = std::move(other.feedback_reader_);
    servo_command_writer_ = std::move(other.servo_command_writer_);
    servo_feedback_reader_ = std::move(other.servo_feedback_reader_);
    other.command_writer_.reset();
    other.feedback_reader_.reset();
    other.servo_command_writer_.reset();
    other.servo_feedback_reader_.reset();
  }
  return *this;
}

RobotIoClient::~RobotIoClient() { release_noexcept(); }

Result<void> RobotIoClient::publish_commands(std::span<const AxisCommand> axes,
                                             std::uint64_t sequence,
                                             std::int64_t timestamp_ns) {
  auto peer = check_peer();
  if (!peer.has_value()) {
    return peer;
  }
  if (!command_writer_.has_value()) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "IPC command writer is closed"});
  }
  return command_writer_->publish(axes, sequence, timestamp_ns);
}

Result<Snapshot<AxisFeedback>> RobotIoClient::read_feedback() const {
  auto peer = check_peer();
  if (!peer.has_value()) {
    return Result<Snapshot<AxisFeedback>>::failure(peer.error());
  }
  if (!feedback_reader_.has_value()) {
    return Result<Snapshot<AxisFeedback>>::failure(
        {ErrorCode::unavailable, "IPC feedback reader is closed"});
  }
  return feedback_reader_->read_latest();
}

Result<void> RobotIoClient::publish_servo_commands(
    std::span<const St3215ServoCommand> servos, std::uint64_t sequence,
    std::int64_t timestamp_ns) {
  auto peer = check_peer();
  if (!peer.has_value()) {
    return peer;
  }
  if (!servo_command_writer_.has_value()) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "IPC servo command writer is closed"});
  }
  return servo_command_writer_->publish(servos, sequence, timestamp_ns);
}

Result<Snapshot<St3215ServoFeedback>>
RobotIoClient::read_servo_feedback() const {
  auto peer = check_peer();
  if (!peer.has_value()) {
    return Result<Snapshot<St3215ServoFeedback>>::failure(peer.error());
  }
  if (!servo_feedback_reader_.has_value()) {
    return Result<Snapshot<St3215ServoFeedback>>::failure(
        {ErrorCode::unavailable, "IPC servo feedback reader is closed"});
  }
  return servo_feedback_reader_->read_latest();
}

Result<void> RobotIoClient::check_peer() const { return check_peer_fd(socket_fd_); }

Result<void> RobotIoClient::close() {
  Error first_error{};
  bool failed = false;
  const auto remember = [&](const char* operation) {
    if (!failed) {
      first_error = system_error(ErrorCode::io, operation);
      failed = true;
    }
  };
  command_writer_.reset();
  feedback_reader_.reset();
  servo_command_writer_.reset();
  servo_feedback_reader_.reset();
  if (command_mapping_ != nullptr) {
    if (munmap(command_mapping_, command_mapping_size_) != 0) {
      remember("munmap(command)");
    }
    command_mapping_ = nullptr;
  }
  if (feedback_mapping_ != nullptr) {
    if (munmap(feedback_mapping_, feedback_mapping_size_) != 0) {
      remember("munmap(feedback)");
    }
    feedback_mapping_ = nullptr;
  }
  if (servo_command_mapping_ != nullptr) {
    if (munmap(servo_command_mapping_, servo_command_mapping_size_) != 0) {
      remember("munmap(servo command)");
    }
    servo_command_mapping_ = nullptr;
  }
  if (servo_feedback_mapping_ != nullptr) {
    if (munmap(servo_feedback_mapping_, servo_feedback_mapping_size_) != 0) {
      remember("munmap(servo feedback)");
    }
    servo_feedback_mapping_ = nullptr;
  }
  for (auto* descriptor :
       {&command_writer_fd_, &feedback_reader_fd_,
        &servo_command_writer_fd_, &servo_feedback_reader_fd_, &socket_fd_}) {
    if (*descriptor >= 0) {
      if (::close(*descriptor) != 0) {
        remember("close");
      }
      *descriptor = -1;
    }
  }
  command_mapping_size_ = 0;
  feedback_mapping_size_ = 0;
  servo_command_mapping_size_ = 0;
  servo_feedback_mapping_size_ = 0;
  axis_count_ = 0;
  servo_count_ = 0;
  generation_ = 0;
  return failed ? Result<void>::failure(std::move(first_error))
                : Result<void>::success();
}

void RobotIoClient::release_noexcept() noexcept {
  command_writer_.reset();
  feedback_reader_.reset();
  servo_command_writer_.reset();
  servo_feedback_reader_.reset();
  if (command_mapping_ != nullptr) {
    (void)munmap(command_mapping_, command_mapping_size_);
    command_mapping_ = nullptr;
  }
  if (feedback_mapping_ != nullptr) {
    (void)munmap(feedback_mapping_, feedback_mapping_size_);
    feedback_mapping_ = nullptr;
  }
  if (servo_command_mapping_ != nullptr) {
    (void)munmap(servo_command_mapping_, servo_command_mapping_size_);
    servo_command_mapping_ = nullptr;
  }
  if (servo_feedback_mapping_ != nullptr) {
    (void)munmap(servo_feedback_mapping_, servo_feedback_mapping_size_);
    servo_feedback_mapping_ = nullptr;
  }
  for (auto* descriptor :
       {&command_writer_fd_, &feedback_reader_fd_,
        &servo_command_writer_fd_, &servo_feedback_reader_fd_, &socket_fd_}) {
    if (*descriptor >= 0) {
      (void)::close(*descriptor);
      *descriptor = -1;
    }
  }
  command_mapping_size_ = 0;
  feedback_mapping_size_ = 0;
  servo_command_mapping_size_ = 0;
  servo_feedback_mapping_size_ = 0;
  axis_count_ = 0;
  servo_count_ = 0;
  generation_ = 0;
}

}  // namespace policy_runtime
