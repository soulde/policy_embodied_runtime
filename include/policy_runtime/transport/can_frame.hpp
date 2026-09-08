#pragma once

#include <array>
#include <cstdint>

namespace policy_runtime {

// Canonical CAN data frame shared by SocketCAN, USB-CAN and device codecs.
struct CanFrame {
  std::uint32_t id{};
  std::array<std::byte, 8> data{};
  std::uint8_t dlc{8};
};

}  // namespace policy_runtime
