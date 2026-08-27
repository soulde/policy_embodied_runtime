#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

using ChannelId = std::uint32_t;

class FrameTransport : public virtual Transport {
 public:
  // Stages a frame for physical transmission by a later cycle().
  virtual Result<void> write(ChannelId channel, std::span<const std::byte> data) = 0;

  // Returns bytes already received by cycle(); this call does not perform physical I/O.
  virtual Result<std::size_t> read(ChannelId channel, std::span<std::byte> buffer) = 0;

  // Interrupt a blocking cycle so an owning executor can stop promptly. Most
  // transports are cycle-bounded already; transports with blocking I/O may
  // override this with a non-blocking wakeup.
  virtual void request_stop() noexcept {}
};

}  // namespace policy_runtime
