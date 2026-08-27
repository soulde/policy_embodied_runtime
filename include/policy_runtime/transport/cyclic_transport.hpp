#pragma once

#include <cstddef>
#include <cstdint>

#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

using CyclicFieldId = std::uint32_t;

struct CyclicField {
  CyclicFieldId id{};
  std::size_t size_bytes{};
};

class CyclicTransport : public virtual Transport {
 public:
  virtual Result<void> register_cyclic_input(CyclicField field) = 0;
  virtual Result<void> register_cyclic_output(CyclicField field) = 0;
};

}  // namespace policy_runtime
