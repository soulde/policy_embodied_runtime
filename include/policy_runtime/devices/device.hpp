#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime {

// Transport-neutral bounded frame. A physical bus owns the address namespace;
// devices only decode/encode their own payload bytes.
struct DeviceFrame {
  std::uint64_t sequence{};
  std::uint32_t address{};
  std::uint16_t size{};
  std::array<std::byte, 256> bytes{};
};

class SensorDevice {
 public:
  virtual ~SensorDevice() = default;
  virtual bool accepts(const DeviceFrame& frame) const noexcept = 0;
};

class ActuatorDevice {
 public:
  virtual ~ActuatorDevice() = default;
  virtual Result<DeviceFrame> encode_frame() const noexcept = 0;
};

}  // namespace policy_runtime
