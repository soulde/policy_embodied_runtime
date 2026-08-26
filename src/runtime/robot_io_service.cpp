#include "policy_runtime/runtime/robot_io_service.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <string>
#include <system_error>
#include <utility>

#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

namespace policy_runtime {
namespace {

constexpr off_t kMaximumGenerationFileSize = 11;

Error system_error(ErrorCode code, const char* operation,
                   int error_number = errno) {
  return {code, std::string(operation) + ": " +
                    std::error_code(error_number, std::generic_category())
                        .message()};
}

class ScopedFd {
 public:
  explicit ScopedFd(int descriptor = -1) noexcept : descriptor_(descriptor) {}
  ScopedFd(const ScopedFd&) = delete;
  ScopedFd& operator=(const ScopedFd&) = delete;
  ~ScopedFd() {
    if (descriptor_ >= 0) {
      static_cast<void>(close(descriptor_));
    }
  }

  int get() const noexcept { return descriptor_; }
  int release() noexcept { return std::exchange(descriptor_, -1); }

 private:
  int descriptor_{-1};
};

Result<std::uint32_t> read_generation(int descriptor) {
  std::array<char, kMaximumGenerationFileSize + 1U> bytes{};
  std::size_t size{};
  while (size < bytes.size()) {
    const auto count = pread(descriptor, bytes.data() + size,
                             bytes.size() - size,
                             static_cast<off_t>(size));
    if (count < 0 && errno == EINTR) {
      continue;
    }
    if (count < 0) {
      return Result<std::uint32_t>::failure(
          system_error(ErrorCode::io, "pread(generation file)"));
    }
    if (count == 0) {
      break;
    }
    size += static_cast<std::size_t>(count);
  }
  if (size == bytes.size()) {
    return Result<std::uint32_t>::failure(
        {ErrorCode::invalid_argument, "generation file is too long"});
  }
  if (size != 0U && bytes[size - 1U] == '\n') {
    --size;
  }
  std::uint64_t generation{};
  const auto parsed =
      std::from_chars(bytes.data(), bytes.data() + size, generation);
  if (size == 0U || parsed.ec != std::errc{} ||
      parsed.ptr != bytes.data() + size || generation == 0U ||
      generation > std::numeric_limits<std::uint32_t>::max()) {
    return Result<std::uint32_t>::failure(
        {ErrorCode::invalid_argument,
         "generation file must contain one positive 32-bit integer"});
  }
  return Result<std::uint32_t>::success(
      static_cast<std::uint32_t>(generation));
}

bool same_inode(const struct stat& left, const struct stat& right) noexcept {
  return left.st_dev == right.st_dev && left.st_ino == right.st_ino;
}

Result<void> validate_runtime_file(const struct stat& status,
                                   mode_t expected_type,
                                   const char* description) {
  if ((status.st_mode & S_IFMT) != expected_type ||
      status.st_uid != geteuid() || (status.st_mode & 0077) != 0) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         std::string(description) +
             " must be owner-only and owned by the runtime host user"});
  }
  return Result<void>::success();
}

Result<void> connect_with_deadline(int descriptor,
                                   const sockaddr_un& address,
                                   socklen_t address_size,
                                   int timeout_ms) {
  const int original_flags = fcntl(descriptor, F_GETFL);
  if (original_flags < 0 ||
      fcntl(descriptor, F_SETFL, original_flags | O_NONBLOCK) != 0) {
    return Result<void>::failure(
        system_error(ErrorCode::io, "fcntl(nonblocking service socket)"));
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  int connected_result{};
  do {
    connected_result =
        connect(descriptor, reinterpret_cast<const sockaddr*>(&address),
                address_size);
  } while (connected_result != 0 && errno == EINTR &&
           std::chrono::steady_clock::now() < deadline);

  if (connected_result != 0 && errno != EINPROGRESS && errno != EALREADY &&
      errno != EAGAIN && errno != EWOULDBLOCK && errno != EISCONN) {
    return Result<void>::failure(
        system_error(ErrorCode::unavailable, "connect(robot I/O service)"));
  }
  bool connected = connected_result == 0 || errno == EISCONN;
  while (!connected) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return Result<void>::failure(
          {ErrorCode::timeout, "connect(robot I/O service) timed out"});
    }
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    const int poll_timeout =
        static_cast<int>(std::max<std::int64_t>(1, remaining.count()));
    pollfd pending{descriptor, POLLOUT, 0};
    const int polled = poll(&pending, 1, poll_timeout);
    if (polled < 0 && errno == EINTR) {
      continue;
    }
    if (polled < 0) {
      return Result<void>::failure(
          system_error(ErrorCode::io, "poll(robot I/O service connect)"));
    }
    if (polled == 0) {
      continue;
    }
    int socket_error{};
    socklen_t socket_error_size = sizeof(socket_error);
    if (getsockopt(descriptor, SOL_SOCKET, SO_ERROR, &socket_error,
                   &socket_error_size) != 0 ||
        socket_error_size != sizeof(socket_error)) {
      return Result<void>::failure(
          system_error(ErrorCode::io, "getsockopt(service connect error)"));
    }
    if (socket_error != 0) {
      return Result<void>::failure(system_error(
          ErrorCode::unavailable, "connect(robot I/O service)", socket_error));
    }
    connected = true;
  }

  if (fcntl(descriptor, F_SETFL, original_flags) != 0) {
    return Result<void>::failure(
        system_error(ErrorCode::io, "fcntl(restore service socket flags)"));
  }
  return Result<void>::success();
}

}  // namespace

Result<RobotIoClient> connect_robot_io_service(
    std::string_view socket_path, std::string_view generation_path,
    int setup_timeout_ms) {
  if (socket_path.empty() || generation_path.empty() ||
      socket_path == generation_path ||
      socket_path.size() >= sizeof(sockaddr_un::sun_path) ||
      socket_path.find('\0') != std::string_view::npos ||
      generation_path.find('\0') != std::string_view::npos ||
      setup_timeout_ms <= 0) {
    return Result<RobotIoClient>::failure(
        {ErrorCode::invalid_argument, "invalid robot I/O service connection"});
  }

  const std::string generation_name(generation_path);
  ScopedFd generation_fd(open(generation_name.c_str(),
                              O_RDONLY | O_NONBLOCK | O_CLOEXEC | O_NOFOLLOW));
  if (generation_fd.get() < 0) {
    const auto code = errno == ELOOP ? ErrorCode::invalid_argument
                                    : ErrorCode::unavailable;
    return Result<RobotIoClient>::failure(
        system_error(code, "open(robot I/O generation file)"));
  }
  struct stat generation_status {};
  if (fstat(generation_fd.get(), &generation_status) != 0) {
    return Result<RobotIoClient>::failure(
        system_error(ErrorCode::io, "fstat(robot I/O generation file)"));
  }
  auto valid_generation_file =
      validate_runtime_file(generation_status, S_IFREG, "generation file");
  if (!valid_generation_file.has_value()) {
    return Result<RobotIoClient>::failure(valid_generation_file.error());
  }
  if (generation_status.st_size <= 0 ||
      generation_status.st_size > kMaximumGenerationFileSize) {
    return Result<RobotIoClient>::failure(
        {ErrorCode::invalid_argument,
         "generation file size is outside the accepted bound"});
  }
  auto generation = read_generation(generation_fd.get());
  if (!generation.has_value()) {
    return Result<RobotIoClient>::failure(generation.error());
  }

  const std::string socket_name(socket_path);
  struct stat socket_status {};
  if (lstat(socket_name.c_str(), &socket_status) != 0) {
    return Result<RobotIoClient>::failure(
        system_error(ErrorCode::unavailable, "lstat(robot I/O socket)"));
  }
  auto valid_socket =
      validate_runtime_file(socket_status, S_IFSOCK, "robot I/O socket");
  if (!valid_socket.has_value()) {
    return Result<RobotIoClient>::failure(valid_socket.error());
  }

  ScopedFd connected(socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0));
  if (connected.get() < 0) {
    return Result<RobotIoClient>::failure(
        system_error(ErrorCode::io, "socket(robot I/O service)"));
  }
  timeval timeout{setup_timeout_ms / 1000,
                  (setup_timeout_ms % 1000) * 1000};
  if (setsockopt(connected.get(), SOL_SOCKET, SO_RCVTIMEO, &timeout,
                 sizeof(timeout)) != 0) {
    return Result<RobotIoClient>::failure(
        system_error(ErrorCode::io, "setsockopt(robot I/O setup timeout)"));
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::memcpy(address.sun_path, socket_name.c_str(), socket_name.size() + 1U);
  auto connected_service = connect_with_deadline(
      connected.get(), address,
      static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) +
                             socket_name.size() + 1U),
      setup_timeout_ms);
  if (!connected_service.has_value()) {
    return Result<RobotIoClient>::failure(connected_service.error());
  }

  ucred peer{};
  socklen_t peer_size = sizeof(peer);
  if (getsockopt(connected.get(), SOL_SOCKET, SO_PEERCRED, &peer, &peer_size) !=
          0 ||
      peer_size != sizeof(peer) || peer.pid <= 0 ||
      peer.uid != generation_status.st_uid) {
    return Result<RobotIoClient>::failure(
        {ErrorCode::unavailable,
         "robot I/O service peer identity does not match generation owner"});
  }

  struct stat published_generation_status {};
  if (lstat(generation_name.c_str(), &published_generation_status) != 0 ||
      !same_inode(generation_status, published_generation_status)) {
    return Result<RobotIoClient>::failure(
        {ErrorCode::unavailable,
         "robot I/O daemon generation changed while connecting"});
  }
  return RobotIoClient::connect(connected.release(), generation.value());
}

}  // namespace policy_runtime
