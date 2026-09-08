#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime {

struct DamiaoLimits {
  float position_max{};
  float velocity_max{};
  float torque_max{};
};

struct DamiaoMitCommand {
  float position{};
  float velocity{};
  float kp{};
  float kd{};
  float torque{};
};

struct DamiaoFeedback {
  std::uint8_t motor_id{};
  std::uint8_t error_code{};
  float position{};
  float velocity{};
  float torque{};
  std::int8_t mos_temperature_c{};
  std::int8_t rotor_temperature_c{};
};

enum class DamiaoControl : std::uint8_t {
  enable = 0xFC,
  disable = 0xFD,
  zero_position = 0xFE,
};

class DamiaoProtocol {
 public:
  using Frame = std::array<std::byte, 8>;

  static Result<Frame> encode_mit(const DamiaoMitCommand& command,
                                  const DamiaoLimits& limits) noexcept;
  static Frame encode_control(DamiaoControl control) noexcept;

  static Result<DamiaoFeedback> decode_feedback(
      std::uint32_t can_id, const std::byte* frame, std::size_t length,
      const DamiaoLimits& limits) noexcept;
};

}  // namespace policy_runtime
