#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

struct ObjectAddress {
  std::uint16_t index{};
  std::uint8_t subindex{};
};

class ObjectDictionaryTransport : public virtual Transport {
 public:
  // Stages an object-dictionary write for physical transmission by cycle().
  virtual Result<void> download(ObjectAddress address, std::span<const std::byte> data) = 0;

  // Retrieves data received by cycle(); implementations must not perform physical I/O here.
  virtual Result<std::size_t> upload(ObjectAddress address, std::span<std::byte> buffer) = 0;
};

}  // namespace policy_runtime
