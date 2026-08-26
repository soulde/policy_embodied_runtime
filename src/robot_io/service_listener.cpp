#include "policy_runtime/robot_io/service_listener.hpp"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <system_error>
#include <utility>

#include <poll.h>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "policy_runtime/robot_io/daemon.hpp"

namespace policy_runtime {
namespace {

Error system_error(ErrorCode code, const char* operation,
                   int error_number = errno) {
  return {code, std::string(operation) + ": " +
                    std::error_code(error_number, std::generic_category())
                        .message()};
}

class ScopedFd {
 public:
  explicit ScopedFd(int fd = -1) noexcept : fd_(fd) {}
  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;
  ~ScopedFd() {
    if (fd_ >= 0) {
      static_cast<void>(close(fd_));
    }
  }

  int get() const noexcept { return fd_; }
  int release() noexcept { return std::exchange(fd_, -1); }

 private:
  int fd_{-1};
};

Result<std::uint32_t> fresh_generation() {
  for (unsigned attempt = 0; attempt < 8U; ++attempt) {
    std::uint32_t generation{};
    std::size_t offset{};
    while (offset < sizeof(generation)) {
      const auto received = getrandom(
          reinterpret_cast<std::byte*>(&generation) + offset,
          sizeof(generation) - offset, 0);
      if (received < 0 && errno == EINTR) {
        continue;
      }
      if (received <= 0) {
        return Result<std::uint32_t>::failure(
            system_error(ErrorCode::io, "getrandom(daemon generation)"));
      }
      offset += static_cast<std::size_t>(received);
    }
    if (generation != 0U) {
      return Result<std::uint32_t>::success(generation);
    }
  }
  return Result<std::uint32_t>::failure(
      {ErrorCode::unavailable, "could not generate a nonzero daemon generation"});
}

Result<void> write_generation(int descriptor, std::uint32_t generation) {
  const auto text = std::to_string(generation) + "\n";
  if (ftruncate(descriptor, 0) != 0) {
    return Result<void>::failure(
        system_error(ErrorCode::io, "ftruncate(generation file)"));
  }
  std::size_t written{};
  while (written < text.size()) {
    const auto count = pwrite(descriptor, text.data() + written,
                              text.size() - written,
                              static_cast<off_t>(written));
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count <= 0) {
      return Result<void>::failure(
          system_error(ErrorCode::io, "pwrite(generation file)"));
    }
    written += static_cast<std::size_t>(count);
  }
  if (fdatasync(descriptor) != 0) {
    return Result<void>::failure(
        system_error(ErrorCode::io, "fdatasync(generation file)"));
  }
  return Result<void>::success();
}

void unlink_if_same(const std::string& path, std::uint64_t device,
                    std::uint64_t inode) noexcept {
  if (path.empty() || inode == 0U) {
    return;
  }
  struct stat status {};
  if (lstat(path.c_str(), &status) == 0 &&
      static_cast<std::uint64_t>(status.st_dev) == device &&
      static_cast<std::uint64_t>(status.st_ino) == inode) {
    static_cast<void>(unlink(path.c_str()));
  }
}

}  // namespace

RobotIoServiceListener::RobotIoServiceListener(
    int generation_fd, std::string socket_path, std::string generation_path,
    std::uint32_t generation, uid_t allowed_peer_uid) noexcept
    : generation_fd_(generation_fd),
      socket_path_(std::move(socket_path)),
      generation_path_(std::move(generation_path)),
      generation_(generation),
      allowed_peer_uid_(allowed_peer_uid) {}

RobotIoServiceListener::RobotIoServiceListener(
    RobotIoServiceListener&& other) noexcept
    : listen_fd_(std::exchange(other.listen_fd_, -1)),
      generation_fd_(std::exchange(other.generation_fd_, -1)),
      socket_path_(std::move(other.socket_path_)),
      generation_path_(std::move(other.generation_path_)),
      socket_device_(std::exchange(other.socket_device_, 0U)),
      socket_inode_(std::exchange(other.socket_inode_, 0U)),
      generation_device_(std::exchange(other.generation_device_, 0U)),
      generation_inode_(std::exchange(other.generation_inode_, 0U)),
      generation_(std::exchange(other.generation_, 0U)),
      allowed_peer_uid_(other.allowed_peer_uid_) {}

RobotIoServiceListener& RobotIoServiceListener::operator=(
    RobotIoServiceListener&& other) noexcept {
  if (this != &other) {
    release_noexcept();
    listen_fd_ = std::exchange(other.listen_fd_, -1);
    generation_fd_ = std::exchange(other.generation_fd_, -1);
    socket_path_ = std::move(other.socket_path_);
    generation_path_ = std::move(other.generation_path_);
    socket_device_ = std::exchange(other.socket_device_, 0U);
    socket_inode_ = std::exchange(other.socket_inode_, 0U);
    generation_device_ = std::exchange(other.generation_device_, 0U);
    generation_inode_ = std::exchange(other.generation_inode_, 0U);
    generation_ = std::exchange(other.generation_, 0U);
    allowed_peer_uid_ = other.allowed_peer_uid_;
  }
  return *this;
}

RobotIoServiceListener::~RobotIoServiceListener() { release_noexcept(); }

Result<RobotIoServiceListener> RobotIoServiceListener::create(
    std::string socket_path, std::string generation_path,
    uid_t allowed_peer_uid) {
  if (socket_path.empty() || generation_path.empty() ||
      socket_path == generation_path ||
      socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
    return Result<RobotIoServiceListener>::failure(
        {ErrorCode::invalid_argument, "invalid service IPC runtime paths"});
  }

  ScopedFd generation_fd(open(generation_path.c_str(),
                              O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW,
                              S_IRUSR | S_IWUSR));
  if (generation_fd.get() < 0) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::io, "open(generation file)"));
  }
  struct stat generation_status {};
  if (fstat(generation_fd.get(), &generation_status) != 0) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::io, "fstat(generation file)"));
  }
  if (!S_ISREG(generation_status.st_mode) ||
      generation_status.st_uid != geteuid()) {
    return Result<RobotIoServiceListener>::failure(
        {ErrorCode::invalid_argument,
         "generation file must be a regular file owned by the daemon user"});
  }
  if (fchmod(generation_fd.get(), S_IRUSR | S_IWUSR) != 0) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::io, "fchmod(generation file)"));
  }
  if (flock(generation_fd.get(), LOCK_EX | LOCK_NB) != 0) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::unavailable, "flock(generation file)"));
  }
  RobotIoServiceListener listener(
      generation_fd.release(), std::move(socket_path),
      std::move(generation_path), 0U, allowed_peer_uid);
  listener.generation_device_ =
      static_cast<std::uint64_t>(generation_status.st_dev);
  listener.generation_inode_ =
      static_cast<std::uint64_t>(generation_status.st_ino);
  if (ftruncate(listener.generation_fd_, 0) != 0) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::io, "ftruncate(stale generation file)"));
  }

  auto generated = fresh_generation();
  if (!generated.has_value()) {
    return Result<RobotIoServiceListener>::failure(generated.error());
  }
  listener.generation_ = generated.value();

  struct stat stale_status {};
  if (lstat(listener.socket_path_.c_str(), &stale_status) == 0) {
    if (!S_ISSOCK(stale_status.st_mode) || stale_status.st_uid != geteuid() ||
        unlink(listener.socket_path_.c_str()) != 0) {
      return Result<RobotIoServiceListener>::failure(
          {ErrorCode::unavailable,
           "service IPC socket path is not a removable stale socket"});
    }
  } else if (errno != ENOENT) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::io, "lstat(service IPC socket)"));
  }

  ScopedFd socket_fd(socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0));
  if (socket_fd.get() < 0) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::io, "socket(service IPC)"));
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, listener.socket_path_.c_str(),
              listener.socket_path_.size() + 1U);
  const auto previous_umask = umask(0077);
  const auto bound = bind(socket_fd.get(),
                          reinterpret_cast<const sockaddr*>(&address),
                          static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                                                 listener.socket_path_.size() +
                                                 1U));
  static_cast<void>(umask(previous_umask));
  if (bound != 0) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::io, "bind(service IPC socket)"));
  }
  struct stat socket_status {};
  if (lstat(listener.socket_path_.c_str(), &socket_status) != 0 ||
      !S_ISSOCK(socket_status.st_mode) || socket_status.st_uid != geteuid()) {
    return Result<RobotIoServiceListener>::failure(
        {ErrorCode::io, "service IPC listener has unexpected ownership or type"});
  }
  listener.listen_fd_ = socket_fd.release();
  listener.socket_device_ = static_cast<std::uint64_t>(socket_status.st_dev);
  listener.socket_inode_ = static_cast<std::uint64_t>(socket_status.st_ino);
  if (chmod(listener.socket_path_.c_str(), S_IRUSR | S_IWUSR) != 0 ||
      listen(listener.listen_fd_, 1) != 0) {
    return Result<RobotIoServiceListener>::failure(
        system_error(ErrorCode::io, "prepare(service IPC listener)"));
  }

  auto published = write_generation(listener.generation_fd_,
                                    listener.generation_);
  if (!published.has_value()) {
    return Result<RobotIoServiceListener>::failure(published.error());
  }
  return Result<RobotIoServiceListener>::success(std::move(listener));
}

Result<int> RobotIoServiceListener::accept_authenticated(
    const DaemonSignalLatch& signals) {
  if (listen_fd_ < 0) {
    return Result<int>::failure(
        {ErrorCode::invalid_argument, "service IPC listener is not accepting"});
  }
  for (;;) {
    if (signals.stop_requested()) {
      return Result<int>::failure(
          {ErrorCode::unavailable,
           "stop requested while waiting for the service IPC peer"});
    }
    pollfd descriptor{listen_fd_, POLLIN, 0};
    const auto polled = poll(&descriptor, 1, 100);
    if (polled < 0 && errno == EINTR) {
      continue;
    }
    if (polled < 0) {
      return Result<int>::failure(
          system_error(ErrorCode::io, "poll(service IPC listener)"));
    }
    if (polled == 0) {
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      return Result<int>::failure(
          {ErrorCode::io, "service IPC listener became unavailable"});
    }
    if ((descriptor.revents & POLLIN) == 0) {
      continue;
    }

    int accepted{};
    do {
      accepted = accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC);
    } while (accepted < 0 && errno == EINTR && !signals.stop_requested());
    if (accepted < 0) {
      if (signals.stop_requested()) {
        return Result<int>::failure(
            {ErrorCode::unavailable,
             "stop requested while accepting the service IPC peer"});
      }
      return Result<int>::failure(
          system_error(ErrorCode::io, "accept4(service IPC peer)"));
    }
    ScopedFd peer(accepted);
    ucred credentials{};
    socklen_t credential_size = sizeof(credentials);
    if (getsockopt(peer.get(), SOL_SOCKET, SO_PEERCRED, &credentials,
                   &credential_size) != 0) {
      return Result<int>::failure(
          system_error(ErrorCode::io, "getsockopt(SO_PEERCRED)"));
    }
    if (credential_size != sizeof(credentials) || credentials.pid <= 0) {
      return Result<int>::failure(
          {ErrorCode::protocol, "service IPC peer credentials are malformed"});
    }
    if (credentials.uid != allowed_peer_uid_) {
      return Result<int>::failure(
          {ErrorCode::unavailable,
           "service IPC peer effective user is not authorized"});
    }
    static_cast<void>(close(std::exchange(listen_fd_, -1)));
    return Result<int>::success(peer.release());
  }
}

void RobotIoServiceListener::release_noexcept() noexcept {
  if (listen_fd_ >= 0) {
    static_cast<void>(close(std::exchange(listen_fd_, -1)));
  }
  if (generation_fd_ >= 0) {
    static_cast<void>(close(std::exchange(generation_fd_, -1)));
  }
  unlink_if_same(socket_path_, socket_device_, socket_inode_);
  unlink_if_same(generation_path_, generation_device_, generation_inode_);
  socket_device_ = 0U;
  socket_inode_ = 0U;
  generation_device_ = 0U;
  generation_inode_ = 0U;
  generation_ = 0U;
}

}  // namespace policy_runtime
