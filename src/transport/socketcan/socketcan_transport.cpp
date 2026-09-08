#include "policy_runtime/transport/socketcan/socketcan_transport.hpp"

#include <cerrno>
#include <cstring>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace policy_runtime {
namespace {

Error system_error(const char* operation) {
  return {ErrorCode::io,
          std::string(operation) + ": " +
              std::error_code(errno, std::generic_category()).message()};
}

bool valid_dlc(std::uint8_t dlc) { return dlc <= CAN_MAX_DLEN; }

}  // namespace

SocketCanTransport::~SocketCanTransport() {
  if (owns_fd_ && fd_ >= 0) {
    ::close(fd_);
  }
}

SocketCanTransport::SocketCanTransport(SocketCanTransport&& other) noexcept
    : fd_(std::exchange(other.fd_, -1)),
      owns_fd_(std::exchange(other.owns_fd_, false)),
      receive_handler_(std::move(other.receive_handler_)) {}

SocketCanTransport& SocketCanTransport::operator=(
    SocketCanTransport&& other) noexcept {
  if (this != &other) {
    if (owns_fd_ && fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = std::exchange(other.fd_, -1);
    owns_fd_ = std::exchange(other.owns_fd_, false);
    receive_handler_ = std::move(other.receive_handler_);
  }
  return *this;
}

Result<void> SocketCanTransport::open() {
  return valid() ? Result<void>::success()
                 : Result<void>::failure(
                       {ErrorCode::unavailable,
                        "SocketCAN descriptor is not open"});
}

void SocketCanTransport::close() noexcept {
  if (owns_fd_ && fd_ >= 0) {
    ::close(fd_);
  }
  fd_ = -1;
  owns_fd_ = false;
}

TransportHealth SocketCanTransport::health() const noexcept {
  return valid() ? TransportHealth::healthy : TransportHealth::failed;
}

SchedulingClass SocketCanTransport::scheduling_class() const noexcept {
  return SchedulingClass::asynchronous;
}

void SocketCanTransport::cycle(const CycleContext&) noexcept {}

void SocketCanTransport::receive_once() noexcept {
  auto received = receive_frame();
  if (received.has_value() && received.value().has_value() &&
      receive_handler_) {
    try {
      receive_handler_(received.value().value());
    } catch (...) {
      // Transport callbacks are owned by the runtime; a callback failure must
      // not terminate the physical receive worker.
    }
  }
}

void SocketCanTransport::set_receive_handler(ReceiveHandler handler) {
  receive_handler_ = std::move(handler);
}

Result<SocketCanTransport> SocketCanTransport::open(
    std::string_view interface_name) {
  if (interface_name.empty() || interface_name.size() >= IFNAMSIZ) {
    return Result<SocketCanTransport>::failure(
        {ErrorCode::invalid_argument, "invalid SocketCAN interface name"});
  }
  const int fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC,
                          CAN_RAW);
  if (fd < 0) {
    return Result<SocketCanTransport>::failure(system_error("socket(PF_CAN)"));
  }
  ::ifreq request{};
  std::memcpy(request.ifr_name, interface_name.data(), interface_name.size());
  if (::ioctl(fd, SIOCGIFINDEX, &request) != 0) {
    const auto error = system_error("ioctl(SIOCGIFINDEX)");
    ::close(fd);
    return Result<SocketCanTransport>::failure(error);
  }
  ::sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<const ::sockaddr*>(&address),
             sizeof(address)) != 0) {
    const auto error = system_error("bind(SocketCAN)");
    ::close(fd);
    return Result<SocketCanTransport>::failure(error);
  }
  SocketCanTransport transport(fd);
  transport.owns_fd_ = true;
  return Result<SocketCanTransport>::success(std::move(transport));
}

Result<void> SocketCanTransport::send_frame(const CanFrame& frame) noexcept {
  if (fd_ < 0 || frame.id > CAN_SFF_MASK || !valid_dlc(frame.dlc)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "invalid CAN frame"});
  }
  ::can_frame raw{};
  raw.can_id = frame.id;
  raw.can_dlc = frame.dlc;
  std::memcpy(raw.data, frame.data.data(), frame.dlc);
  const auto written = ::write(fd_, &raw, sizeof(raw));
  if (written < 0) {
    return Result<void>::failure(system_error("socketcan write"));
  }
  if (static_cast<std::size_t>(written) != sizeof(raw)) {
    return Result<void>::failure(
        {ErrorCode::io, "socketcan short write"});
  }
  return Result<void>::success();
}

Result<std::optional<CanFrame>> SocketCanTransport::receive_frame() noexcept {
  if (fd_ < 0) {
    return Result<std::optional<CanFrame>>::failure(
        {ErrorCode::invalid_argument, "invalid socketcan descriptor"});
  }
  ::can_frame raw{};
  const auto received = ::read(fd_, &raw, sizeof(raw));
  if (received < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return Result<std::optional<CanFrame>>::success(std::nullopt);
    }
    return Result<std::optional<CanFrame>>::failure(
        system_error("socketcan read"));
  }
  if (static_cast<std::size_t>(received) < sizeof(raw)) {
    return Result<std::optional<CanFrame>>::failure(
        {ErrorCode::io, "socketcan short read"});
  }
  if ((raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U) {
    // Non-data frames are skipped by the caller as absent feedback.
    return Result<std::optional<CanFrame>>::success(std::nullopt);
  }
  if (!valid_dlc(raw.can_dlc)) {
    return Result<std::optional<CanFrame>>::failure(
        {ErrorCode::protocol, "invalid CAN DLC"});
  }
  CanFrame frame{};
  frame.id = raw.can_id & CAN_SFF_MASK;
  frame.dlc = raw.can_dlc;
  std::memcpy(frame.data.data(), raw.data, raw.can_dlc);
  return Result<std::optional<CanFrame>>::success(frame);
}

}  // namespace policy_runtime
