#include "policy_runtime/transport/serial/virtual_can_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <string>
#include <system_error>

#include <unistd.h>

namespace policy_runtime {
namespace {

constexpr std::byte kHeader = std::byte{0xAA};
constexpr std::size_t kHeaderSize = 6U;

Error system_error(const char* operation) {
  return {ErrorCode::io,
          std::string(operation) + ": " +
              std::error_code(errno, std::generic_category()).message()};
}

std::byte checksum(const std::byte* data, std::size_t length) noexcept {
  unsigned sum = 0U;
  for (std::size_t i = 0U; i < length; ++i) {
    sum += std::to_integer<unsigned>(data[i]);
  }
  return static_cast<std::byte>(sum & 0xFFU);
}

}  // namespace

Result<void> VirtualCanTransport::send_frame(const CanFrame& frame) noexcept {
  if (fd_ < 0 || frame.id > 0x1FFFFFFFU || frame.dlc > 8U) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "invalid CAN frame"});
  }
  std::byte encoded[kEncodedFrameSize]{};
  encoded[0] = kHeader;
  const std::uint32_t id = frame.id;
  for (unsigned i = 0U; i < 4U; ++i) {
    encoded[1U + i] = static_cast<std::byte>((id >> (8U * i)) & 0xFFU);
  }
  encoded[5] = static_cast<std::byte>(frame.dlc);
  std::memcpy(encoded + kHeaderSize, frame.data.data(), frame.dlc);
  const std::size_t length = kHeaderSize + frame.dlc;
  encoded[length] = checksum(encoded, length);

  const auto written = ::write(fd_, encoded, length + 1U);
  if (written < 0) {
    return Result<void>::failure(system_error("virtual CAN write"));
  }
  if (static_cast<std::size_t>(written) != length + 1U) {
    return Result<void>::failure({ErrorCode::io, "virtual CAN short write"});
  }
  return Result<void>::success();
}

Result<std::optional<CanFrame>> VirtualCanTransport::receive_frame() noexcept {
  if (fd_ < 0) {
    return Result<std::optional<CanFrame>>::failure(
        {ErrorCode::invalid_argument, "invalid virtual CAN descriptor"});
  }
  if (buffer_length_ == sizeof(buffer_)) {
    return Result<std::optional<CanFrame>>::failure(
        {ErrorCode::protocol, "virtual CAN frame overflow"});
  }
  const auto received =
      ::read(fd_, buffer_ + buffer_length_, sizeof(buffer_) - buffer_length_);
  if (received < 0) {
    return Result<std::optional<CanFrame>>::failure(
        system_error("virtual CAN read"));
  }
  buffer_length_ += static_cast<std::size_t>(received);

  std::optional<CanFrame> frame;
  std::size_t consumed = 0U;
  while (buffer_length_ - consumed >= kHeaderSize + 1U) {
    bool complete = false;
    // Parse one candidate frame starting at `consumed`.
    const auto dlc = std::to_integer<std::uint8_t>(buffer_[consumed + 5U]);
    if (buffer_[consumed] != kHeader || dlc > 8U) {
      return Result<std::optional<CanFrame>>::failure(
          {ErrorCode::protocol, "malformed virtual CAN frame"});
    }
    const std::size_t length = kHeaderSize + dlc + 1U;
    if (buffer_length_ - consumed >= length) {
      if (checksum(buffer_ + consumed, length - 1U) !=
          buffer_[consumed + length - 1U]) {
        return Result<std::optional<CanFrame>>::failure(
            {ErrorCode::protocol, "virtual CAN checksum mismatch"});
      }
      if (!frame.has_value()) {
        CanFrame parsed{};
        for (unsigned i = 0U; i < 4U; ++i) {
          parsed.id |= static_cast<std::uint32_t>(
                           std::to_integer<std::uint8_t>(
                               buffer_[consumed + 1U + i]))
                       << (8U * i);
        }
        parsed.dlc = dlc;
        std::memcpy(parsed.data.data(), buffer_ + consumed + kHeaderSize, dlc);
        frame = parsed;
      }
      consumed += length;
      complete = true;
    }
    if (!complete) {
      break;
    }
  }
  // Retain any trailing partial frame for the next cycle.
  std::memmove(buffer_, buffer_ + consumed, buffer_length_ - consumed);
  buffer_length_ -= consumed;
  return Result<std::optional<CanFrame>>::success(frame);
}

}  // namespace policy_runtime
