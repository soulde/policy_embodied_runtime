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
  void* get() const noexcept { return address_; }
  void* release() noexcept { return std::exchange(address_, nullptr); }

 private:
  void* address_;
  std::size_t size_;
};

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
  if (domain != AF_UNIX || (type != SOCK_STREAM && type != SOCK_SEQPACKET)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "IPC socket must be AF_UNIX stream or seqpacket"});
  }
  const int descriptor_flags = fcntl(socket_fd, F_GETFD);
  if (descriptor_flags < 0 || fcntl(socket_fd, F_SETFD, descriptor_flags | FD_CLOEXEC) != 0) {
    return Result<void>::failure(system_error(ErrorCode::io, "fcntl(FD_CLOEXEC)"));
  }
  return Result<void>::success();
}

Result<void> check_peer_fd(int socket_fd) {
  if (socket_fd < 0) {
    return Result<void>::failure({ErrorCode::unavailable, "IPC socket is closed"});
  }
  std::byte byte{};
  for (;;) {
    const auto received = recv(socket_fd, &byte, sizeof(byte), MSG_PEEK | MSG_DONTWAIT);
    if (received > 0) {
      return Result<void>::success();
    }
    if (received == 0) {
      return Result<void>::failure({ErrorCode::unavailable, "IPC peer disconnected"});
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return Result<void>::success();
    }
    if (errno == ECONNRESET || errno == ENOTCONN) {
      return Result<void>::failure({ErrorCode::unavailable, "IPC peer disconnected"});
    }
    return Result<void>::failure(system_error(ErrorCode::io, "recv(MSG_PEEK)"));
  }
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

Result<void> validate_received_fd(int fd, std::uint64_t expected_size) {
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
  const int flags = fcntl(fd, F_GETFD);
  if (flags < 0 || fcntl(fd, F_SETFD, flags | FD_CLOEXEC) != 0) {
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

  IpcSetupMessage setup{};
  std::array<std::byte, CMSG_SPACE(2 * sizeof(int))> control{};
  iovec vector{&setup, sizeof(setup)};
  msghdr message{};
  message.msg_iov = &vector;
  message.msg_iovlen = 1;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  ssize_t received{};
  do {
    received = recvmsg(socket.get(), &message, MSG_CMSG_CLOEXEC | MSG_WAITALL);
  } while (received < 0 && errno == EINTR);
  if (received == 0) {
    return Result<RobotIoClient>::failure(
        {ErrorCode::unavailable, "IPC peer disconnected during setup"});
  }
  if (received < 0) {
    const auto code = errno == ECONNRESET || errno == ENOTCONN ? ErrorCode::unavailable
                                                               : ErrorCode::io;
    return Result<RobotIoClient>::failure(system_error(code, "recvmsg(SCM_RIGHTS)"));
  }

  std::array<ScopedFd, 2> descriptors{};
  std::size_t descriptor_count = 0;
  bool ancillary_invalid = false;
  for (auto* item = CMSG_FIRSTHDR(&message); item != nullptr;
       item = CMSG_NXTHDR(&message, item)) {
    if (item->cmsg_level != SOL_SOCKET || item->cmsg_type != SCM_RIGHTS ||
        item->cmsg_len < CMSG_LEN(0)) {
      ancillary_invalid = true;
      continue;
    }
    const auto payload_size = item->cmsg_len - CMSG_LEN(0);
    const auto item_descriptor_count = payload_size / sizeof(int);
    ancillary_invalid = ancillary_invalid || payload_size % sizeof(int) != 0 ||
                        item_descriptor_count != 2 || descriptor_count != 0;
    const auto* received_descriptors = reinterpret_cast<const int*>(CMSG_DATA(item));
    for (std::size_t index = 0; index < item_descriptor_count; ++index) {
      if (descriptor_count < descriptors.size()) {
        descriptors[descriptor_count++] = ScopedFd(received_descriptors[index]);
      } else {
        (void)::close(received_descriptors[index]);
      }
    }
  }
  if (static_cast<std::size_t>(received) != sizeof(setup) ||
      (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0 || descriptor_count != 2 ||
      ancillary_invalid) {
    return Result<RobotIoClient>::failure(
        {ErrorCode::protocol, "truncated IPC setup message"});
  }
  const auto setup_result = validate_setup(setup, expected_generation);
  if (!setup_result.has_value()) {
    return Result<RobotIoClient>::failure(setup_result.error());
  }
  const auto command_fd_result =
      validate_received_fd(descriptors[0].get(), setup.command_mapping_size);
  if (!command_fd_result.has_value()) {
    return Result<RobotIoClient>::failure(command_fd_result.error());
  }
  const auto feedback_fd_result =
      validate_received_fd(descriptors[1].get(), setup.feedback_mapping_size);
  if (!feedback_fd_result.has_value()) {
    return Result<RobotIoClient>::failure(feedback_fd_result.error());
  }

  void* command_address = mmap(nullptr, setup.command_mapping_size, PROT_READ | PROT_WRITE,
                               MAP_SHARED, descriptors[0].get(), 0);
  if (command_address == MAP_FAILED) {
    return Result<RobotIoClient>::failure(system_error(ErrorCode::io, "mmap(command)"));
  }
  ScopedMapping command_mapping(command_address, setup.command_mapping_size);
  void* feedback_address = mmap(nullptr, setup.feedback_mapping_size,
                                PROT_READ | PROT_WRITE, MAP_SHARED,
                                descriptors[1].get(), 0);
  if (feedback_address == MAP_FAILED) {
    return Result<RobotIoClient>::failure(system_error(ErrorCode::io, "mmap(feedback)"));
  }
  ScopedMapping feedback_mapping(feedback_address, setup.feedback_mapping_size);

  auto command_region = SnapshotRegion<AxisCommand>::attach(
      command_address, setup.command_mapping_size, setup.generation, setup.axis_count,
      IpcRegionKind::command);
  if (!command_region.has_value()) {
    return Result<RobotIoClient>::failure(command_region.error());
  }
  auto feedback_region = SnapshotRegion<AxisFeedback>::attach(
      feedback_address, setup.feedback_mapping_size, setup.generation, setup.axis_count,
      IpcRegionKind::feedback);
  if (!feedback_region.has_value()) {
    return Result<RobotIoClient>::failure(feedback_region.error());
  }

  RobotIoClient client(
      socket.release(), descriptors[0].release(), descriptors[1].release(),
      command_mapping.release(), setup.command_mapping_size, feedback_mapping.release(),
      setup.feedback_mapping_size, setup.axis_count, setup.generation,
      command_region.value(), feedback_region.value());
  return Result<RobotIoClient>::success(std::move(client));
}

RobotIoClient::RobotIoClient(int socket_fd, int command_fd, int feedback_fd,
                             void* command_mapping, std::size_t command_mapping_size,
                             void* feedback_mapping, std::size_t feedback_mapping_size,
                             std::uint32_t axis_count, std::uint32_t generation,
                             SnapshotRegion<AxisCommand> command_region,
                             SnapshotRegion<AxisFeedback> feedback_region) noexcept
    : socket_fd_(socket_fd),
      command_fd_(command_fd),
      feedback_fd_(feedback_fd),
      command_mapping_(command_mapping),
      command_mapping_size_(command_mapping_size),
      feedback_mapping_(feedback_mapping),
      feedback_mapping_size_(feedback_mapping_size),
      axis_count_(axis_count),
      generation_(generation),
      command_region_(command_region),
      feedback_region_(feedback_region) {}

RobotIoClient::RobotIoClient(RobotIoClient&& other) noexcept
    : socket_fd_(std::exchange(other.socket_fd_, -1)),
      command_fd_(std::exchange(other.command_fd_, -1)),
      feedback_fd_(std::exchange(other.feedback_fd_, -1)),
      command_mapping_(std::exchange(other.command_mapping_, nullptr)),
      command_mapping_size_(other.command_mapping_size_),
      feedback_mapping_(std::exchange(other.feedback_mapping_, nullptr)),
      feedback_mapping_size_(other.feedback_mapping_size_),
      axis_count_(other.axis_count_),
      generation_(other.generation_),
      command_region_(other.command_region_),
      feedback_region_(other.feedback_region_) {}

RobotIoClient& RobotIoClient::operator=(RobotIoClient&& other) noexcept {
  if (this != &other) {
    release_noexcept();
    socket_fd_ = std::exchange(other.socket_fd_, -1);
    command_fd_ = std::exchange(other.command_fd_, -1);
    feedback_fd_ = std::exchange(other.feedback_fd_, -1);
    command_mapping_ = std::exchange(other.command_mapping_, nullptr);
    command_mapping_size_ = other.command_mapping_size_;
    feedback_mapping_ = std::exchange(other.feedback_mapping_, nullptr);
    feedback_mapping_size_ = other.feedback_mapping_size_;
    axis_count_ = other.axis_count_;
    generation_ = other.generation_;
    command_region_ = other.command_region_;
    feedback_region_ = other.feedback_region_;
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
  return command_region_.publish(axes, sequence, timestamp_ns);
}

Result<Snapshot<AxisFeedback>> RobotIoClient::read_feedback() const {
  auto peer = check_peer();
  if (!peer.has_value()) {
    return Result<Snapshot<AxisFeedback>>::failure(peer.error());
  }
  return feedback_region_.read_latest();
}

Result<void> RobotIoClient::check_peer() const {
  return check_peer_fd(socket_fd_);
}

Result<void> RobotIoClient::close() {
  Error first_error{};
  bool failed = false;
  const auto remember = [&](const char* operation) {
    if (!failed) {
      first_error = system_error(ErrorCode::io, operation);
      failed = true;
    }
  };
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
  for (auto* descriptor : {&command_fd_, &feedback_fd_, &socket_fd_}) {
    if (*descriptor >= 0) {
      if (::close(*descriptor) != 0) {
        remember("close");
      }
      *descriptor = -1;
    }
  }
  command_region_ = {};
  feedback_region_ = {};
  return failed ? Result<void>::failure(std::move(first_error))
                : Result<void>::success();
}

void RobotIoClient::release_noexcept() noexcept {
  if (command_mapping_ != nullptr) {
    (void)munmap(command_mapping_, command_mapping_size_);
    command_mapping_ = nullptr;
  }
  if (feedback_mapping_ != nullptr) {
    (void)munmap(feedback_mapping_, feedback_mapping_size_);
    feedback_mapping_ = nullptr;
  }
  for (auto* descriptor : {&command_fd_, &feedback_fd_, &socket_fd_}) {
    if (*descriptor >= 0) {
      (void)::close(*descriptor);
      *descriptor = -1;
    }
  }
  command_region_ = {};
  feedback_region_ = {};
}

}  // namespace policy_runtime
