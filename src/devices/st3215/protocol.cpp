#include "policy_runtime/devices/st3215/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>

namespace policy_runtime {
namespace {

constexpr std::byte kHeaderByte{0xff};

std::uint8_t byte_value(std::byte value) noexcept {
  return std::to_integer<std::uint8_t>(value);
}

void append_u16(Bytes& output, std::uint16_t value) {
  output.push_back(static_cast<std::byte>(value & 0xffU));
  output.push_back(static_cast<std::byte>((value >> 8U) & 0xffU));
}

Bytes require_encoded(std::uint8_t device_id,
                      std::uint8_t instruction_or_status,
                      std::span<const std::byte> parameters) {
  auto encoded =
      St3215Protocol::encode_packet(device_id, instruction_or_status, parameters);
  return encoded.has_value() ? std::move(encoded.value()) : Bytes{};
}

}  // namespace

Bytes St3215Protocol::ping_command(std::uint8_t device_id) {
  return require_encoded(device_id,
                         static_cast<std::uint8_t>(St3215Instruction::ping), {});
}

Bytes St3215Protocol::read_data_command(std::uint8_t device_id,
                                        St3215Register address,
                                        std::uint8_t length) {
  const std::array parameters{static_cast<std::byte>(address),
                              static_cast<std::byte>(length)};
  return require_encoded(
      device_id, static_cast<std::uint8_t>(St3215Instruction::read_data),
      parameters);
}

Bytes St3215Protocol::write_data_command(
    std::uint8_t device_id, St3215Register address,
    std::span<const std::byte> data) {
  if (data.size() + 1U > kSt3215MaximumParameters) {
    return {};
  }
  Bytes parameters;
  parameters.reserve(data.size() + 1U);
  parameters.push_back(static_cast<std::byte>(address));
  parameters.insert(parameters.end(), data.begin(), data.end());
  return require_encoded(
      device_id, static_cast<std::uint8_t>(St3215Instruction::write_data),
      parameters);
}

Bytes St3215Protocol::goal_position_command(std::uint8_t device_id,
                                            std::uint16_t position_units,
                                            std::uint16_t time_units,
                                            std::uint16_t speed_units) {
  Bytes data;
  data.reserve(6U);
  append_u16(data, position_units);
  append_u16(data, time_units);
  append_u16(data, speed_units);
  return write_data_command(device_id, St3215Register::goal_position, data);
}

Bytes St3215Protocol::read_present_position_command(std::uint8_t device_id) {
  return read_data_command(device_id, St3215Register::present_position, 2U);
}

Bytes St3215Protocol::status_packet(
    std::uint8_t device_id, std::uint8_t error,
    std::span<const std::byte> parameters) {
  return require_encoded(device_id, error, parameters);
}

Result<Bytes> St3215Protocol::encode_packet(
    std::uint8_t device_id, std::uint8_t instruction_or_status,
    std::span<const std::byte> parameters) {
  if (parameters.size() > kSt3215MaximumParameters) {
    return Result<Bytes>::failure(
        {ErrorCode::invalid_argument, "too many ST3215 parameters"});
  }
  const auto length = static_cast<std::uint8_t>(parameters.size() + 2U);
  Bytes output;
  output.reserve(parameters.size() + 6U);
  output.push_back(kHeaderByte);
  output.push_back(kHeaderByte);
  output.push_back(static_cast<std::byte>(device_id));
  output.push_back(static_cast<std::byte>(length));
  output.push_back(static_cast<std::byte>(instruction_or_status));
  output.insert(output.end(), parameters.begin(), parameters.end());
  output.push_back(static_cast<std::byte>(
      checksum(device_id, length, instruction_or_status, parameters)));
  return Result<Bytes>::success(std::move(output));
}

Result<St3215Packet> St3215Protocol::decode_packet(
    std::span<const std::byte> packet) {
  if (packet.size() < 6U) {
    return Result<St3215Packet>::failure(
        {ErrorCode::protocol, "ST3215 packet is too short"});
  }
  if (packet[0] != kHeaderByte || packet[1] != kHeaderByte) {
    return Result<St3215Packet>::failure(
        {ErrorCode::protocol, "invalid ST3215 packet header"});
  }
  const auto length = byte_value(packet[3]);
  if (length < 2U || static_cast<std::size_t>(length) + 4U != packet.size()) {
    return Result<St3215Packet>::failure(
        {ErrorCode::protocol, "invalid ST3215 packet length"});
  }
  const auto device_id = byte_value(packet[2]);
  const auto instruction_or_status = byte_value(packet[4]);
  const auto parameters = packet.subspan(5U, packet.size() - 6U);
  const auto expected =
      checksum(device_id, length, instruction_or_status, parameters);
  if (byte_value(packet.back()) != expected) {
    return Result<St3215Packet>::failure(
        {ErrorCode::protocol, "invalid ST3215 checksum"});
  }
  St3215Packet decoded;
  decoded.device_id = device_id;
  decoded.instruction_or_status = instruction_or_status;
  decoded.parameters.assign(parameters.begin(), parameters.end());
  return Result<St3215Packet>::success(std::move(decoded));
}

Result<St3215Status> St3215Protocol::parse_status(
    std::span<const std::byte> packet) {
  auto decoded = decode_packet(packet);
  if (!decoded.has_value()) {
    return Result<St3215Status>::failure(decoded.error());
  }
  St3215Status status;
  status.device_id = decoded.value().device_id;
  status.error = decoded.value().instruction_or_status;
  status.parameters = std::move(decoded.value().parameters);
  return Result<St3215Status>::success(std::move(status));
}

std::uint8_t St3215Protocol::checksum(
    std::uint8_t device_id, std::uint8_t length,
    std::uint8_t instruction_or_status,
    std::span<const std::byte> parameters) noexcept {
  std::uint32_t total = static_cast<std::uint32_t>(device_id) + length +
                        instruction_or_status;
  for (const auto parameter : parameters) {
    total += byte_value(parameter);
  }
  return static_cast<std::uint8_t>(~total & 0xffU);
}

Result<void> St3215StreamParser::append(std::span<const std::byte> data) {
  if (data.size() > buffer_.size() - size_) {
    reset();
    return Result<void>::failure(
        {ErrorCode::protocol, "ST3215 stream buffer capacity exceeded"});
  }
  std::copy(data.begin(), data.end(), buffer_.begin() + size_);
  size_ += data.size();
  return Result<void>::success();
}

Result<std::optional<St3215Packet>> St3215StreamParser::next() {
  std::size_t header{};
  while (header + 1U < size_ &&
         (buffer_[header] != kHeaderByte ||
          buffer_[header + 1U] != kHeaderByte)) {
    ++header;
  }
  if (header > 0U) {
    discard_prefix(header);
  }
  if (size_ < 2U) {
    return Result<std::optional<St3215Packet>>::success(std::nullopt);
  }
  if (size_ < 4U) {
    return Result<std::optional<St3215Packet>>::success(std::nullopt);
  }
  const auto length = byte_value(buffer_[3]);
  if (length < 2U) {
    discard_prefix(2U);
    return Result<std::optional<St3215Packet>>::failure(
        {ErrorCode::protocol, "invalid ST3215 packet length"});
  }
  const auto frame_size = static_cast<std::size_t>(length) + 4U;
  if (frame_size > kSt3215MaximumFrameSize) {
    discard_prefix(2U);
    return Result<std::optional<St3215Packet>>::failure(
        {ErrorCode::protocol, "ST3215 frame exceeds maximum size"});
  }
  if (size_ < frame_size) {
    return Result<std::optional<St3215Packet>>::success(std::nullopt);
  }
  auto decoded = St3215Protocol::decode_packet(
      std::span<const std::byte>(buffer_.data(), frame_size));
  discard_prefix(frame_size);
  if (!decoded.has_value()) {
    return Result<std::optional<St3215Packet>>::failure(decoded.error());
  }
  return Result<std::optional<St3215Packet>>::success(
      std::optional<St3215Packet>{std::move(decoded.value())});
}

void St3215StreamParser::reset() noexcept { size_ = 0U; }

std::size_t St3215StreamParser::buffered_size() const noexcept { return size_; }

void St3215StreamParser::discard_prefix(std::size_t count) noexcept {
  count = std::min(count, size_);
  std::move(buffer_.begin() + count, buffer_.begin() + size_, buffer_.begin());
  size_ -= count;
}

Result<std::uint16_t> read_st3215_u16_le(
    std::span<const std::byte> data) {
  if (data.size() < 2U) {
    return Result<std::uint16_t>::failure(
        {ErrorCode::protocol, "expected at least two ST3215 bytes"});
  }
  return Result<std::uint16_t>::success(static_cast<std::uint16_t>(
      byte_value(data[0]) | (static_cast<std::uint16_t>(byte_value(data[1]))
                             << 8U)));
}

Result<std::uint16_t> radians_to_position_units(
    double position_rad, std::uint16_t max_position_units) {
  if (!std::isfinite(position_rad) || max_position_units == 0U) {
    return Result<std::uint16_t>::failure(
        {ErrorCode::invalid_argument, "invalid ST3215 position conversion"});
  }
  constexpr double tau = 2.0 * std::numbers::pi;
  double wrapped = std::fmod(position_rad, tau);
  if (wrapped < 0.0) {
    wrapped += tau;
  }
  const double scaled = wrapped / tau * max_position_units;
  const double integral = std::floor(scaled);
  const double fractional = scaled - integral;
  double rounded = integral;
  if (fractional > 0.5 ||
      (fractional == 0.5 &&
       static_cast<std::uint64_t>(integral) % 2U != 0U)) {
    rounded += 1.0;
  }
  rounded = std::clamp(rounded, 0.0,
                       static_cast<double>(max_position_units));
  return Result<std::uint16_t>::success(
      static_cast<std::uint16_t>(rounded));
}

Result<double> position_units_to_radians(
    std::uint16_t position_units, std::uint16_t max_position_units) {
  if (max_position_units == 0U) {
    return Result<double>::failure(
        {ErrorCode::invalid_argument, "invalid ST3215 position conversion"});
  }
  return Result<double>::success(
      static_cast<double>(position_units) /
      static_cast<double>(max_position_units) * 2.0 * std::numbers::pi);
}

}  // namespace policy_runtime
