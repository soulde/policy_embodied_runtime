#include "policy_runtime/robot_io/ipc_server.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/memfd.h>
#include <limits>
#include <string>
#include <system_error>
#include <utility>

#include <sys/mman.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace policy_runtime {
namespace {

Error system_error(ErrorCode code, const char* operation, int error_number = errno) {
  return {code, std::string(operation) + ": " +
                    std::error_code(error_number, std::generic_category()).message()};
}

class ScopedFd {
 public:
  explicit ScopedFd(int fd = -1) noexcept : fd_(fd) {}
  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;
  ScopedFd(ScopedFd&& other) noexcept : fd_(std::exchange(other.fd_, -1)) {}
  ~ScopedFd() {
    if (fd_ >= 0) {
      (void)::close(fd_);
    }
  }
  int get() const noexcept { return fd_; }
  int release() noexcept { return std::exchange(fd_, -1); }

 private:
  int fd_;
};

class ScopedMapping {
 public:
  ScopedMapping(void* address = nullptr, std::size_t size = 0) noexcept
      : address_(address), size_(size) {}
  ScopedMapping(const ScopedMapping&) = delete;
  ScopedMapping& operator=(const ScopedMapping&) = delete;
  ScopedMapping(ScopedMapping&& other) noexcept
      : address_(std::exchange(other.address_, nullptr)), size_(other.size_) {}
  ~ScopedMapping() {
    if (address_ != nullptr) {
      (void)munmap(address_, size_);
    }
  }
  void* get() const noexcept { return address_; }
  void* release() noexcept { return std::exchange(address_, nullptr); }

 private:
  void* address_;
  std::size_t size_;
};

template <std::size_t DescriptorCount>
struct alignas(cmsghdr) AncillaryStorage {
  std::array<std::byte, CMSG_SPACE(DescriptorCount * sizeof(int))> bytes{};
};

static_assert(alignof(AncillaryStorage<2>) >= alignof(cmsghdr));
static_assert(alignof(AncillaryStorage<2>) >= alignof(int));
static_assert(sizeof(AncillaryStorage<2>) == CMSG_SPACE(2 * sizeof(int)));

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
  polled = poll(&descriptor, 1, 0);
  if (polled < 0 && errno == EINTR) {
    // The non-real-time monitor will retry on its next bounded iteration.
    return Result<void>::success();
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
    // MSG_TRUNC returns the original packet length while SOCK_SEQPACKET
    // atomically consumes the whole packet, even when it exceeds discarded.
    const ssize_t received =
        recvmsg(socket_fd, &message, MSG_DONTWAIT | MSG_TRUNC);
    if (received < 0 && errno == EINTR) {
      return Result<void>::success();
    }
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

Result<ScopedFd> reopen_read_only(int fd) {
  std::array<char, 64> path{};
  const int length = std::snprintf(path.data(), path.size(), "/proc/self/fd/%d", fd);
  if (length < 0 || static_cast<std::size_t>(length) >= path.size()) {
    return Result<ScopedFd>::failure(
        {ErrorCode::internal, "failed to format memfd reopen path"});
  }
  ScopedFd read_only_fd(::open(path.data(), O_RDONLY | O_CLOEXEC));
  if (read_only_fd.get() < 0) {
    return Result<ScopedFd>::failure(system_error(ErrorCode::io, "open(memfd read-only)"));
  }
  return Result<ScopedFd>::success(std::move(read_only_fd));
}

template <class T>
struct CreatedBacking {
  ScopedFd read_write_fd;
  ScopedFd read_only_fd;
  std::size_t size{};
};

template <class T>
Result<CreatedBacking<T>> create_backing(const char* name, std::uint32_t axis_count,
                                         std::uint32_t generation, IpcRegionKind kind) {
  auto size = SnapshotRegion<T>::mapping_size(axis_count);
  if (!size.has_value()) {
    return Result<CreatedBacking<T>>::failure(size.error());
  }
  if (size.value() > static_cast<std::size_t>(std::numeric_limits<off_t>::max())) {
    return Result<CreatedBacking<T>>::failure(
        {ErrorCode::invalid_argument, "IPC mapping is too large for off_t"});
  }
  ScopedFd read_write_fd(static_cast<int>(
      syscall(SYS_memfd_create, name, MFD_CLOEXEC | MFD_ALLOW_SEALING)));
  if (read_write_fd.get() < 0) {
    return Result<CreatedBacking<T>>::failure(system_error(ErrorCode::io, "memfd_create"));
  }
  if (ftruncate(read_write_fd.get(), static_cast<off_t>(size.value())) != 0) {
    return Result<CreatedBacking<T>>::failure(system_error(ErrorCode::io, "ftruncate"));
  }
  void* address = mmap(nullptr, size.value(), PROT_READ | PROT_WRITE, MAP_SHARED,
                       read_write_fd.get(), 0);
  if (address == MAP_FAILED) {
    return Result<CreatedBacking<T>>::failure(system_error(ErrorCode::io, "mmap(init)"));
  }
  ScopedMapping initialization_mapping(address, size.value());
  auto initialized = SnapshotRegion<T>::initialize(address, size.value(), axis_count,
                                                    generation, kind);
  if (!initialized.has_value()) {
    return Result<CreatedBacking<T>>::failure(initialized.error());
  }
  constexpr int kRequiredSeals = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
  if (fcntl(read_write_fd.get(), F_ADD_SEALS, kRequiredSeals) != 0) {
    return Result<CreatedBacking<T>>::failure(
        system_error(ErrorCode::io, "fcntl(F_ADD_SEALS)"));
  }
  auto read_only = reopen_read_only(read_write_fd.get());
  if (!read_only.has_value()) {
    return Result<CreatedBacking<T>>::failure(read_only.error());
  }
  return Result<CreatedBacking<T>>::success(
      {std::move(read_write_fd), std::move(read_only.value()), size.value()});
}

template <class T>
struct AttachedMapping {
  ScopedMapping mapping;
  SnapshotRegion<T> region;
};

template <class T>
Result<AttachedMapping<T>> attach_mapping(int fd, std::size_t size,
                                          SnapshotMappingAccess mapping_access,
                                          std::uint32_t generation,
                                          std::uint32_t axis_count, IpcRegionKind kind) {
  const int protection = mapping_access == SnapshotMappingAccess::read_write
                             ? PROT_READ | PROT_WRITE
                             : PROT_READ;
  void* address = mmap(nullptr, size, protection, MAP_SHARED, fd, 0);
  if (address == MAP_FAILED) {
    return Result<AttachedMapping<T>>::failure(system_error(ErrorCode::io, "mmap(role)"));
  }
  ScopedMapping mapping(address, size);
  auto region = SnapshotRegion<T>::attach(address, size, generation, axis_count, kind,
                                           mapping_access);
  if (!region.has_value()) {
    return Result<AttachedMapping<T>>::failure(region.error());
  }
  return Result<AttachedMapping<T>>::success(
      {std::move(mapping), std::move(region.value())});
}

}  // namespace

Result<RobotIoIpcServer> RobotIoIpcServer::create(int connected_socket,
                                                  std::uint32_t axis_count,
                                                  std::uint32_t generation) {
  ScopedFd socket(connected_socket);
  auto socket_result = prepare_socket(socket.get());
  if (!socket_result.has_value()) {
    return Result<RobotIoIpcServer>::failure(socket_result.error());
  }
  auto command = create_backing<AxisCommand>("robot-io-command", axis_count, generation,
                                              IpcRegionKind::command);
  if (!command.has_value()) {
    return Result<RobotIoIpcServer>::failure(command.error());
  }
  auto feedback = create_backing<AxisFeedback>("robot-io-feedback", axis_count, generation,
                                                IpcRegionKind::feedback);
  if (!feedback.has_value()) {
    return Result<RobotIoIpcServer>::failure(feedback.error());
  }
  auto command_mapping = attach_mapping<AxisCommand>(
      command.value().read_only_fd.get(), command.value().size,
      SnapshotMappingAccess::read_only, generation, axis_count, IpcRegionKind::command);
  if (!command_mapping.has_value()) {
    return Result<RobotIoIpcServer>::failure(command_mapping.error());
  }
  auto feedback_mapping = attach_mapping<AxisFeedback>(
      feedback.value().read_write_fd.get(), feedback.value().size,
      SnapshotMappingAccess::read_write, generation, axis_count,
      IpcRegionKind::feedback);
  if (!feedback_mapping.has_value()) {
    return Result<RobotIoIpcServer>::failure(feedback_mapping.error());
  }

  auto command_reader = command_mapping.value().region.reader();
  if (!command_reader.has_value()) {
    return Result<RobotIoIpcServer>::failure(command_reader.error());
  }
  auto feedback_writer = feedback_mapping.value().region.writer();
  if (!feedback_writer.has_value()) {
    return Result<RobotIoIpcServer>::failure(feedback_writer.error());
  }

  RobotIoIpcServer server(
      socket.release(), command.value().read_write_fd.release(),
      command.value().read_only_fd.release(), feedback.value().read_write_fd.release(),
      feedback.value().read_only_fd.release(), command_mapping.value().mapping.release(),
      command.value().size, feedback_mapping.value().mapping.release(),
      feedback.value().size, -1, -1, -1, -1, nullptr, 0U, nullptr, 0U,
      axis_count, generation, std::move(command_reader.value()),
      std::move(feedback_writer.value()), 0U, std::nullopt, std::nullopt);
  return Result<RobotIoIpcServer>::success(std::move(server));
}

Result<RobotIoIpcServer> RobotIoIpcServer::create(
    int connected_socket, std::uint32_t axis_count,
    std::uint32_t servo_count, std::uint32_t generation) {
  if (servo_count == 0U) {
    return create(connected_socket, axis_count, generation);
  }
  ScopedFd socket(connected_socket);
  auto socket_result = prepare_socket(socket.get());
  if (!socket_result.has_value()) {
    return Result<RobotIoIpcServer>::failure(socket_result.error());
  }
  auto command = create_backing<AxisCommand>(
      "robot-io-command", axis_count, generation, IpcRegionKind::command);
  auto feedback = create_backing<AxisFeedback>(
      "robot-io-feedback", axis_count, generation, IpcRegionKind::feedback);
  auto servo_command = create_backing<St3215ServoCommand>(
      "robot-io-servo-command", servo_count, generation,
      IpcRegionKind::servo_command);
  auto servo_feedback = create_backing<St3215ServoFeedback>(
      "robot-io-servo-feedback", servo_count, generation,
      IpcRegionKind::servo_feedback);
  if (!command.has_value()) return Result<RobotIoIpcServer>::failure(command.error());
  if (!feedback.has_value()) return Result<RobotIoIpcServer>::failure(feedback.error());
  if (!servo_command.has_value()) return Result<RobotIoIpcServer>::failure(servo_command.error());
  if (!servo_feedback.has_value()) return Result<RobotIoIpcServer>::failure(servo_feedback.error());

  auto command_mapping = attach_mapping<AxisCommand>(
      command.value().read_only_fd.get(), command.value().size,
      SnapshotMappingAccess::read_only, generation, axis_count,
      IpcRegionKind::command);
  auto feedback_mapping = attach_mapping<AxisFeedback>(
      feedback.value().read_write_fd.get(), feedback.value().size,
      SnapshotMappingAccess::read_write, generation, axis_count,
      IpcRegionKind::feedback);
  auto servo_command_mapping = attach_mapping<St3215ServoCommand>(
      servo_command.value().read_only_fd.get(), servo_command.value().size,
      SnapshotMappingAccess::read_only, generation, servo_count,
      IpcRegionKind::servo_command);
  auto servo_feedback_mapping = attach_mapping<St3215ServoFeedback>(
      servo_feedback.value().read_write_fd.get(), servo_feedback.value().size,
      SnapshotMappingAccess::read_write, generation, servo_count,
      IpcRegionKind::servo_feedback);
  if (!command_mapping.has_value()) return Result<RobotIoIpcServer>::failure(command_mapping.error());
  if (!feedback_mapping.has_value()) return Result<RobotIoIpcServer>::failure(feedback_mapping.error());
  if (!servo_command_mapping.has_value()) return Result<RobotIoIpcServer>::failure(servo_command_mapping.error());
  if (!servo_feedback_mapping.has_value()) return Result<RobotIoIpcServer>::failure(servo_feedback_mapping.error());

  auto command_reader = command_mapping.value().region.reader();
  auto feedback_writer = feedback_mapping.value().region.writer();
  auto servo_command_reader = servo_command_mapping.value().region.reader();
  auto servo_feedback_writer = servo_feedback_mapping.value().region.writer();
  if (!command_reader.has_value()) return Result<RobotIoIpcServer>::failure(command_reader.error());
  if (!feedback_writer.has_value()) return Result<RobotIoIpcServer>::failure(feedback_writer.error());
  if (!servo_command_reader.has_value()) return Result<RobotIoIpcServer>::failure(servo_command_reader.error());
  if (!servo_feedback_writer.has_value()) return Result<RobotIoIpcServer>::failure(servo_feedback_writer.error());

  RobotIoIpcServer server(
      socket.release(), command.value().read_write_fd.release(),
      command.value().read_only_fd.release(),
      feedback.value().read_write_fd.release(),
      feedback.value().read_only_fd.release(),
      command_mapping.value().mapping.release(), command.value().size,
      feedback_mapping.value().mapping.release(), feedback.value().size,
      servo_command.value().read_write_fd.release(),
      servo_command.value().read_only_fd.release(),
      servo_feedback.value().read_write_fd.release(),
      servo_feedback.value().read_only_fd.release(),
      servo_command_mapping.value().mapping.release(),
      servo_command.value().size,
      servo_feedback_mapping.value().mapping.release(),
      servo_feedback.value().size, axis_count, generation,
      std::move(command_reader.value()), std::move(feedback_writer.value()),
      servo_count, std::move(servo_command_reader.value()),
      std::move(servo_feedback_writer.value()));
  return Result<RobotIoIpcServer>::success(std::move(server));
}

RobotIoIpcServer::RobotIoIpcServer(
    int socket_fd, int command_transfer_fd, int command_reader_fd,
    int feedback_writer_fd, int feedback_transfer_fd, void* command_mapping,
    std::size_t command_mapping_size, void* feedback_mapping,
    std::size_t feedback_mapping_size, int servo_command_transfer_fd,
    int servo_command_reader_fd, int servo_feedback_writer_fd,
    int servo_feedback_transfer_fd, void* servo_command_mapping,
    std::size_t servo_command_mapping_size, void* servo_feedback_mapping,
    std::size_t servo_feedback_mapping_size, std::uint32_t axis_count,
    std::uint32_t generation, SnapshotReader<AxisCommand> command_reader,
    SnapshotWriter<AxisFeedback> feedback_writer, std::uint32_t servo_count,
    std::optional<SnapshotReader<St3215ServoCommand>> servo_command_reader,
    std::optional<SnapshotWriter<St3215ServoFeedback>> servo_feedback_writer) noexcept
    : socket_fd_(socket_fd),
      command_transfer_fd_(command_transfer_fd),
      command_reader_fd_(command_reader_fd),
      feedback_writer_fd_(feedback_writer_fd),
      feedback_transfer_fd_(feedback_transfer_fd),
      servo_command_transfer_fd_(servo_command_transfer_fd),
      servo_command_reader_fd_(servo_command_reader_fd),
      servo_feedback_writer_fd_(servo_feedback_writer_fd),
      servo_feedback_transfer_fd_(servo_feedback_transfer_fd),
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
      command_reader_(std::move(command_reader)),
      feedback_writer_(std::move(feedback_writer)),
      servo_command_reader_(std::move(servo_command_reader)),
      servo_feedback_writer_(std::move(servo_feedback_writer)) {}

RobotIoIpcServer::RobotIoIpcServer(RobotIoIpcServer&& other) noexcept
    : socket_fd_(std::exchange(other.socket_fd_, -1)),
      command_transfer_fd_(std::exchange(other.command_transfer_fd_, -1)),
      command_reader_fd_(std::exchange(other.command_reader_fd_, -1)),
      feedback_writer_fd_(std::exchange(other.feedback_writer_fd_, -1)),
      feedback_transfer_fd_(std::exchange(other.feedback_transfer_fd_, -1)),
      servo_command_transfer_fd_(
          std::exchange(other.servo_command_transfer_fd_, -1)),
      servo_command_reader_fd_(
          std::exchange(other.servo_command_reader_fd_, -1)),
      servo_feedback_writer_fd_(
          std::exchange(other.servo_feedback_writer_fd_, -1)),
      servo_feedback_transfer_fd_(
          std::exchange(other.servo_feedback_transfer_fd_, -1)),
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
      command_reader_(std::move(other.command_reader_)),
      feedback_writer_(std::move(other.feedback_writer_)),
      servo_command_reader_(std::move(other.servo_command_reader_)),
      servo_feedback_writer_(std::move(other.servo_feedback_writer_)),
      setup_sent_(std::exchange(other.setup_sent_, false)) {
  other.command_reader_.reset();
  other.feedback_writer_.reset();
  other.servo_command_reader_.reset();
  other.servo_feedback_writer_.reset();
}

RobotIoIpcServer& RobotIoIpcServer::operator=(RobotIoIpcServer&& other) noexcept {
  if (this != &other) {
    release_noexcept();
    socket_fd_ = std::exchange(other.socket_fd_, -1);
    command_transfer_fd_ = std::exchange(other.command_transfer_fd_, -1);
    command_reader_fd_ = std::exchange(other.command_reader_fd_, -1);
    feedback_writer_fd_ = std::exchange(other.feedback_writer_fd_, -1);
    feedback_transfer_fd_ = std::exchange(other.feedback_transfer_fd_, -1);
    servo_command_transfer_fd_ =
        std::exchange(other.servo_command_transfer_fd_, -1);
    servo_command_reader_fd_ =
        std::exchange(other.servo_command_reader_fd_, -1);
    servo_feedback_writer_fd_ =
        std::exchange(other.servo_feedback_writer_fd_, -1);
    servo_feedback_transfer_fd_ =
        std::exchange(other.servo_feedback_transfer_fd_, -1);
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
    command_reader_ = std::move(other.command_reader_);
    feedback_writer_ = std::move(other.feedback_writer_);
    servo_command_reader_ = std::move(other.servo_command_reader_);
    servo_feedback_writer_ = std::move(other.servo_feedback_writer_);
    setup_sent_ = std::exchange(other.setup_sent_, false);
    other.command_reader_.reset();
    other.feedback_writer_.reset();
    other.servo_command_reader_.reset();
    other.servo_feedback_writer_.reset();
  }
  return *this;
}

RobotIoIpcServer::~RobotIoIpcServer() { release_noexcept(); }

Result<void> RobotIoIpcServer::send_setup() {
  if (socket_fd_ < 0) {
    return Result<void>::failure({ErrorCode::unavailable, "IPC server is closed"});
  }
  if (setup_sent_) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "IPC setup descriptors were already sent"});
  }
  if (servo_count_ > 0U) {
    IpcSetupMessageV2 setup{};
    setup.generation = generation_;
    setup.mappings = {
        IpcMappingDescription{
            static_cast<std::uint32_t>(IpcRegionKind::command), axis_count_,
            sizeof(AxisCommand), 0U, command_mapping_size_},
        IpcMappingDescription{
            static_cast<std::uint32_t>(IpcRegionKind::feedback), axis_count_,
            sizeof(AxisFeedback), 0U, feedback_mapping_size_},
        IpcMappingDescription{
            static_cast<std::uint32_t>(IpcRegionKind::servo_command),
            servo_count_, sizeof(St3215ServoCommand), 0U,
            servo_command_mapping_size_},
        IpcMappingDescription{
            static_cast<std::uint32_t>(IpcRegionKind::servo_feedback),
            servo_count_, sizeof(St3215ServoFeedback), 0U,
            servo_feedback_mapping_size_}};
    std::array<int, 4> descriptors{
        command_transfer_fd_, feedback_transfer_fd_,
        servo_command_transfer_fd_, servo_feedback_transfer_fd_};
    AncillaryStorage<4> control{};
    iovec vector{&setup, sizeof(setup)};
    msghdr message{};
    message.msg_iov = &vector;
    message.msg_iovlen = 1;
    message.msg_control = control.bytes.data();
    message.msg_controllen = control.bytes.size();
    auto* item = CMSG_FIRSTHDR(&message);
    item->cmsg_level = SOL_SOCKET;
    item->cmsg_type = SCM_RIGHTS;
    item->cmsg_len = CMSG_LEN(sizeof(descriptors));
    std::memcpy(CMSG_DATA(item), descriptors.data(), sizeof(descriptors));
    ssize_t sent{};
    do {
      sent = sendmsg(socket_fd_, &message, MSG_NOSIGNAL | MSG_EOR);
    } while (sent < 0 && errno == EINTR);
    if (sent < 0) {
      const auto code = errno == EPIPE || errno == ECONNRESET
                            ? ErrorCode::unavailable
                            : ErrorCode::io;
      return Result<void>::failure(
          system_error(code, "sendmsg(v2 SCM_RIGHTS)"));
    }
    if (static_cast<std::size_t>(sent) != sizeof(setup)) {
      return Result<void>::failure({ErrorCode::io, "short IPC v2 setup packet"});
    }
    setup_sent_ = true;
    Error close_error{};
    bool close_failed = false;
    for (auto* descriptor : {&command_transfer_fd_, &feedback_transfer_fd_,
                             &servo_command_transfer_fd_,
                             &servo_feedback_transfer_fd_}) {
      if (*descriptor >= 0) {
        if (::close(*descriptor) != 0 && !close_failed) {
          close_error = system_error(
              ErrorCode::io, "close(v2 setup transfer descriptor)");
          close_failed = true;
        }
        *descriptor = -1;
      }
    }
    return close_failed ? Result<void>::failure(std::move(close_error))
                        : Result<void>::success();
  }
  IpcSetupMessage setup{};
  setup.generation = generation_;
  setup.axis_count = axis_count_;
  setup.command_mapping_size = command_mapping_size_;
  setup.feedback_mapping_size = feedback_mapping_size_;

  std::array<int, 2> descriptors{command_transfer_fd_, feedback_transfer_fd_};
  AncillaryStorage<2> control{};
  iovec vector{&setup, sizeof(setup)};
  msghdr message{};
  message.msg_iov = &vector;
  message.msg_iovlen = 1;
  message.msg_control = control.bytes.data();
  message.msg_controllen = control.bytes.size();
  auto* item = CMSG_FIRSTHDR(&message);
  item->cmsg_level = SOL_SOCKET;
  item->cmsg_type = SCM_RIGHTS;
  item->cmsg_len = CMSG_LEN(sizeof(descriptors));
  std::memcpy(CMSG_DATA(item), descriptors.data(), sizeof(descriptors));

  ssize_t sent{};
  do {
    sent = sendmsg(socket_fd_, &message, MSG_NOSIGNAL | MSG_EOR);
  } while (sent < 0 && errno == EINTR);
  if (sent < 0) {
    const auto code = errno == EPIPE || errno == ECONNRESET ? ErrorCode::unavailable
                                                            : ErrorCode::io;
    return Result<void>::failure(system_error(code, "sendmsg(SCM_RIGHTS)"));
  }
  if (static_cast<std::size_t>(sent) != sizeof(setup)) {
    return Result<void>::failure({ErrorCode::io, "short IPC setup packet"});
  }
  setup_sent_ = true;
  Error close_error{};
  bool close_failed = false;
  for (auto* descriptor : {&command_transfer_fd_, &feedback_transfer_fd_}) {
    if (*descriptor >= 0) {
      if (::close(*descriptor) != 0 && !close_failed) {
        close_error = system_error(ErrorCode::io, "close(setup transfer descriptor)");
        close_failed = true;
      }
      *descriptor = -1;
    }
  }
  if (close_failed) {
    return Result<void>::failure(std::move(close_error));
  }
  return Result<void>::success();
}

Result<Snapshot<AxisCommand>> RobotIoIpcServer::read_commands() const {
  auto peer = check_peer_fd(socket_fd_);
  if (!peer.has_value()) {
    return Result<Snapshot<AxisCommand>>::failure(peer.error());
  }
  if (!command_reader_.has_value()) {
    return Result<Snapshot<AxisCommand>>::failure(
        {ErrorCode::unavailable, "IPC command reader is closed"});
  }
  return command_reader_->read_latest();
}

Result<void> RobotIoIpcServer::publish_feedback(std::span<const AxisFeedback> axes,
                                                std::uint64_t sequence,
                                                std::int64_t timestamp_ns) {
  auto peer = check_peer_fd(socket_fd_);
  if (!peer.has_value()) {
    return peer;
  }
  if (!feedback_writer_.has_value()) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "IPC feedback writer is closed"});
  }
  return feedback_writer_->publish(axes, sequence, timestamp_ns);
}

Result<Snapshot<St3215ServoCommand>>
RobotIoIpcServer::read_servo_commands() const {
  auto peer = check_peer_fd(socket_fd_);
  if (!peer.has_value()) {
    return Result<Snapshot<St3215ServoCommand>>::failure(peer.error());
  }
  if (!servo_command_reader_.has_value()) {
    return Result<Snapshot<St3215ServoCommand>>::failure(
        {ErrorCode::unavailable, "IPC servo command reader is closed"});
  }
  return servo_command_reader_->read_latest();
}

Result<void> RobotIoIpcServer::publish_servo_feedback(
    std::span<const St3215ServoFeedback> servos, std::uint64_t sequence,
    std::int64_t timestamp_ns) {
  auto peer = check_peer_fd(socket_fd_);
  if (!peer.has_value()) {
    return peer;
  }
  if (!servo_feedback_writer_.has_value()) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "IPC servo feedback writer is closed"});
  }
  return servo_feedback_writer_->publish(servos, sequence, timestamp_ns);
}

Result<Snapshot<AxisCommand>>
RobotIoIpcServer::read_commands_realtime() const noexcept {
  if (!command_reader_.has_value()) {
    return Result<Snapshot<AxisCommand>>::failure({ErrorCode::unavailable, {}});
  }
  return command_reader_->read_latest();
}

Result<void> RobotIoIpcServer::publish_feedback_realtime(
    std::span<const AxisFeedback> axes, std::uint64_t sequence,
    std::int64_t timestamp_ns) noexcept {
  if (!feedback_writer_.has_value()) {
    return Result<void>::failure({ErrorCode::unavailable, {}});
  }
  return feedback_writer_->publish(axes, sequence, timestamp_ns);
}

Result<Snapshot<St3215ServoCommand>>
RobotIoIpcServer::read_servo_commands_realtime() const noexcept {
  if (!servo_command_reader_.has_value()) {
    return Result<Snapshot<St3215ServoCommand>>::failure(
        {ErrorCode::unavailable, {}});
  }
  return servo_command_reader_->read_latest();
}

Result<void> RobotIoIpcServer::publish_servo_feedback_realtime(
    std::span<const St3215ServoFeedback> servos, std::uint64_t sequence,
    std::int64_t timestamp_ns) noexcept {
  if (!servo_feedback_writer_.has_value()) {
    return Result<void>::failure({ErrorCode::unavailable, {}});
  }
  return servo_feedback_writer_->publish(servos, sequence, timestamp_ns);
}

Result<void> RobotIoIpcServer::check_peer() const { return check_peer_fd(socket_fd_); }

Result<void> RobotIoIpcServer::close() {
  Error first_error{};
  bool failed = false;
  const auto remember = [&](const char* operation) {
    if (!failed) {
      first_error = system_error(ErrorCode::io, operation);
      failed = true;
    }
  };
  command_reader_.reset();
  feedback_writer_.reset();
  servo_command_reader_.reset();
  servo_feedback_writer_.reset();
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
  for (auto* descriptor : {&command_transfer_fd_, &command_reader_fd_,
                           &feedback_writer_fd_, &feedback_transfer_fd_,
                           &servo_command_transfer_fd_,
                           &servo_command_reader_fd_,
                           &servo_feedback_writer_fd_,
                           &servo_feedback_transfer_fd_, &socket_fd_}) {
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
  setup_sent_ = false;
  return failed ? Result<void>::failure(std::move(first_error))
                : Result<void>::success();
}

void RobotIoIpcServer::release_noexcept() noexcept {
  command_reader_.reset();
  feedback_writer_.reset();
  servo_command_reader_.reset();
  servo_feedback_writer_.reset();
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
  for (auto* descriptor : {&command_transfer_fd_, &command_reader_fd_,
                           &feedback_writer_fd_, &feedback_transfer_fd_,
                           &servo_command_transfer_fd_,
                           &servo_command_reader_fd_,
                           &servo_feedback_writer_fd_,
                           &servo_feedback_transfer_fd_, &socket_fd_}) {
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
  setup_sent_ = false;
}

}  // namespace policy_runtime
