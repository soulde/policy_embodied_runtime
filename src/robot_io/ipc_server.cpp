#include "policy_runtime/robot_io/ipc_server.hpp"

#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/memfd.h>
#include <limits>
#include <string>
#include <system_error>
#include <utility>

#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
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
      ::close(fd_);
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

template <class T>
struct CreatedMapping {
  ScopedFd fd;
  ScopedMapping mapping;
  std::size_t size{};
  SnapshotRegion<T> region;
};

template <class T>
Result<CreatedMapping<T>> create_mapping(const char* name, std::uint32_t axis_count,
                                         std::uint32_t generation, IpcRegionKind kind) {
  auto size = SnapshotRegion<T>::mapping_size(axis_count);
  if (!size.has_value()) {
    return Result<CreatedMapping<T>>::failure(size.error());
  }
  if (size.value() > static_cast<std::size_t>(std::numeric_limits<off_t>::max())) {
    return Result<CreatedMapping<T>>::failure(
        {ErrorCode::invalid_argument, "IPC mapping is too large for off_t"});
  }

  ScopedFd fd(static_cast<int>(syscall(SYS_memfd_create, name,
                                       MFD_CLOEXEC | MFD_ALLOW_SEALING)));
  if (fd.get() < 0) {
    return Result<CreatedMapping<T>>::failure(system_error(ErrorCode::io, "memfd_create"));
  }
  if (ftruncate(fd.get(), static_cast<off_t>(size.value())) != 0) {
    return Result<CreatedMapping<T>>::failure(system_error(ErrorCode::io, "ftruncate"));
  }
  void* address = mmap(nullptr, size.value(), PROT_READ | PROT_WRITE, MAP_SHARED, fd.get(), 0);
  if (address == MAP_FAILED) {
    return Result<CreatedMapping<T>>::failure(system_error(ErrorCode::io, "mmap"));
  }
  ScopedMapping mapping(address, size.value());
  auto region = SnapshotRegion<T>::initialize(address, size.value(), axis_count, generation,
                                               kind);
  if (!region.has_value()) {
    return Result<CreatedMapping<T>>::failure(region.error());
  }
  constexpr int kRequiredSeals = F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL;
  if (fcntl(fd.get(), F_ADD_SEALS, kRequiredSeals) != 0) {
    return Result<CreatedMapping<T>>::failure(
        system_error(ErrorCode::io, "fcntl(F_ADD_SEALS)"));
  }
  return Result<CreatedMapping<T>>::success(
      {std::move(fd), std::move(mapping), size.value(), std::move(region.value())});
}

}  // namespace

Result<RobotIoIpcServer> RobotIoIpcServer::create(int connected_socket,
                                                  std::uint32_t axis_count,
                                                  std::uint32_t generation) {
  ScopedFd socket(connected_socket);
  const auto socket_result = prepare_socket(socket.get());
  if (!socket_result.has_value()) {
    return Result<RobotIoIpcServer>::failure(socket_result.error());
  }
  auto command = create_mapping<AxisCommand>("robot-io-command", axis_count, generation,
                                              IpcRegionKind::command);
  if (!command.has_value()) {
    return Result<RobotIoIpcServer>::failure(command.error());
  }
  auto feedback = create_mapping<AxisFeedback>("robot-io-feedback", axis_count, generation,
                                                IpcRegionKind::feedback);
  if (!feedback.has_value()) {
    return Result<RobotIoIpcServer>::failure(feedback.error());
  }

  auto& command_value = command.value();
  auto& feedback_value = feedback.value();
  RobotIoIpcServer server(
      socket.release(), command_value.fd.release(), feedback_value.fd.release(),
      command_value.mapping.release(), command_value.size, feedback_value.mapping.release(),
      feedback_value.size, axis_count, generation, command_value.region,
      feedback_value.region);
  return Result<RobotIoIpcServer>::success(std::move(server));
}

RobotIoIpcServer::RobotIoIpcServer(
    int socket_fd, int command_fd, int feedback_fd, void* command_mapping,
    std::size_t command_mapping_size, void* feedback_mapping,
    std::size_t feedback_mapping_size, std::uint32_t axis_count,
    std::uint32_t generation, SnapshotRegion<AxisCommand> command_region,
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

RobotIoIpcServer::RobotIoIpcServer(RobotIoIpcServer&& other) noexcept
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
      feedback_region_(other.feedback_region_),
      setup_sent_(other.setup_sent_) {}

RobotIoIpcServer& RobotIoIpcServer::operator=(RobotIoIpcServer&& other) noexcept {
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
    setup_sent_ = other.setup_sent_;
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
  IpcSetupMessage setup{};
  setup.generation = generation_;
  setup.axis_count = axis_count_;
  setup.command_mapping_size = command_mapping_size_;
  setup.feedback_mapping_size = feedback_mapping_size_;

  std::array<int, 2> descriptors{command_fd_, feedback_fd_};
  std::array<std::byte, CMSG_SPACE(sizeof(descriptors))> control{};
  iovec vector{&setup, sizeof(setup)};
  msghdr message{};
  message.msg_iov = &vector;
  message.msg_iovlen = 1;
  message.msg_control = control.data();
  message.msg_controllen = control.size();
  auto* control_message = CMSG_FIRSTHDR(&message);
  control_message->cmsg_level = SOL_SOCKET;
  control_message->cmsg_type = SCM_RIGHTS;
  control_message->cmsg_len = CMSG_LEN(sizeof(descriptors));
  std::memcpy(CMSG_DATA(control_message), descriptors.data(), sizeof(descriptors));
  message.msg_controllen = control.size();

  ssize_t sent{};
  do {
    sent = sendmsg(socket_fd_, &message, MSG_NOSIGNAL | MSG_EOR);
  } while (sent < 0 && errno == EINTR);
  if (sent < 0) {
    const auto code = errno == EPIPE || errno == ECONNRESET ? ErrorCode::unavailable
                                                            : ErrorCode::io;
    return Result<void>::failure(system_error(code, "sendmsg(SCM_RIGHTS)"));
  }
  auto total_sent = static_cast<std::size_t>(sent);
  const auto* setup_bytes = reinterpret_cast<const std::byte*>(&setup);
  while (total_sent < sizeof(setup)) {
    const auto remainder = send(socket_fd_, setup_bytes + total_sent,
                                sizeof(setup) - total_sent, MSG_NOSIGNAL);
    if (remainder > 0) {
      total_sent += static_cast<std::size_t>(remainder);
      continue;
    }
    if (remainder < 0 && errno == EINTR) {
      continue;
    }
    const auto code = remainder == 0 || errno == EPIPE || errno == ECONNRESET
                          ? ErrorCode::unavailable
                          : ErrorCode::io;
    return Result<void>::failure(system_error(code, "send(IPC setup)"));
  }
  setup_sent_ = true;
  return Result<void>::success();
}

Result<Snapshot<AxisCommand>> RobotIoIpcServer::read_commands() const noexcept {
  return command_region_.read_latest();
}

Result<void> RobotIoIpcServer::publish_feedback(std::span<const AxisFeedback> axes,
                                                std::uint64_t sequence,
                                                std::int64_t timestamp_ns) noexcept {
  return feedback_region_.publish(axes, sequence, timestamp_ns);
}

Result<void> RobotIoIpcServer::check_peer() const {
  return check_peer_fd(socket_fd_);
}

Result<void> RobotIoIpcServer::close() {
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

void RobotIoIpcServer::release_noexcept() noexcept {
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
