#include "policy_runtime/transport/socketcan/socketcan_transport.hpp"

#include <cerrno>
#include <cstring>
#include <string>
#include <system_error>

#include <linux/can.h>
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
