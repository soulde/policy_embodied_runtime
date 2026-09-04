#include "policy_runtime/protocol/damiao/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace policy_runtime {
namespace {

template <typename T>
T clamp_code(float value, float min_value, float max_value, T max_code) {
  const float ratio = (value - min_value) / (max_value - min_value);
  const float scaled = std::clamp(ratio, 0.0F, 1.0F) * static_cast<float>(max_code);
  return static_cast<T>(std::lround(scaled));
}

float decode_code(std::uint32_t code, std::uint32_t max_code, float min_value,
                  float max_value) {
  return min_value + (static_cast<float>(code) / static_cast<float>(max_code)) *
                         (max_value - min_value);
}

bool finite_and_in_range(float value, float min_value, float max_value) {
  return std::isfinite(value) && value >= min_value && value <= max_value;
}

Error invalid(const char* message) {
  return {ErrorCode::invalid_argument, message};
}

}  // namespace

Result<DamiaoProtocol::Frame> DamiaoProtocol::encode_mit(
    const DamiaoMitCommand& command, const DamiaoLimits& limits) noexcept {
  if (!(limits.position_max > 0.0F && limits.velocity_max > 0.0F &&
        limits.torque_max > 0.0F) ||
      !finite_and_in_range(command.position, -limits.position_max,
                           limits.position_max) ||
      !finite_and_in_range(command.velocity, -limits.velocity_max,
                           limits.velocity_max) ||
      !finite_and_in_range(command.torque, -limits.torque_max,
                           limits.torque_max) ||
      !finite_and_in_range(command.kp, 0.0F, 500.0F) ||
      !finite_and_in_range(command.kd, 0.0F, 5.0F)) {
    return Result<Frame>::failure(invalid("MIT command is outside configured limits"));
  }
  const auto p = clamp_code<std::uint16_t>(command.position, -limits.position_max,
                                            limits.position_max, 0xFFFFU);
  const auto v = clamp_code<std::uint16_t>(command.velocity, -limits.velocity_max,
                                            limits.velocity_max, 0x0FFFU);
  const auto kp = clamp_code<std::uint16_t>(command.kp, 0.0F, 500.0F, 0x0FFFU);
  const auto kd = clamp_code<std::uint16_t>(command.kd, 0.0F, 5.0F, 0x0FFFU);
  const auto t = clamp_code<std::uint16_t>(command.torque, -limits.torque_max,
                                            limits.torque_max, 0x0FFFU);
  Frame frame{};
  frame[0] = static_cast<std::byte>(p >> 8U);
  frame[1] = static_cast<std::byte>(p & 0xFFU);
  frame[2] = static_cast<std::byte>(v >> 4U);
  frame[3] = static_cast<std::byte>(((v & 0x0FU) << 4U) | (kp >> 8U));
  frame[4] = static_cast<std::byte>(kp & 0xFFU);
  frame[5] = static_cast<std::byte>(kd >> 4U);
  frame[6] = static_cast<std::byte>(((kd & 0x0FU) << 4U) | (t >> 8U));
  frame[7] = static_cast<std::byte>(t & 0xFFU);
  return Result<Frame>::success(frame);
}

Result<DamiaoFeedback> DamiaoProtocol::decode_feedback(
    std::uint32_t can_id, const std::byte* frame, std::size_t length,
    const DamiaoLimits& limits) noexcept {
  if (can_id > 0x7FFU || frame == nullptr || length != 8U ||
      !(limits.position_max > 0.0F &&
                           limits.velocity_max > 0.0F && limits.torque_max > 0.0F)) {
    return Result<DamiaoFeedback>::failure(invalid("invalid CAN feedback frame"));
  }
  const auto byte = [](std::byte value) { return std::to_integer<std::uint8_t>(value); };
  const auto id_error = byte(frame[0]);
  const auto position = (static_cast<std::uint16_t>(byte(frame[1])) << 8U) | byte(frame[2]);
  const auto velocity = (static_cast<std::uint16_t>(byte(frame[3])) << 4U) | (byte(frame[4]) >> 4U);
  const auto torque = (static_cast<std::uint16_t>(byte(frame[4]) & 0x0FU) << 8U) | byte(frame[5]);
  return Result<DamiaoFeedback>::success(DamiaoFeedback{
      static_cast<std::uint8_t>(id_error & 0x0FU),
      static_cast<std::uint8_t>(id_error >> 4U),
      decode_code(position, 0xFFFFU, -limits.position_max, limits.position_max),
      decode_code(velocity, 0x0FFFU, -limits.velocity_max, limits.velocity_max),
      decode_code(torque, 0x0FFFU, -limits.torque_max, limits.torque_max),
      static_cast<std::int8_t>(byte(frame[6])), static_cast<std::int8_t>(byte(frame[7]))});
}

}  // namespace policy_runtime
