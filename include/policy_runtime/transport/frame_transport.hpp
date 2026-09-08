#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "policy_runtime/transport/transport.hpp"
#include "policy_runtime/transport/asynchronous_transport.hpp"

namespace policy_runtime {

using ChannelId = std::uint32_t;

class FrameTransport : public virtual AsynchronousTransport {
 public:
  // Stages a frame for physical transmission by the next cycle(). For an
  // asynchronous transport, cycle() owns the bounded physical send while
  // receive_once() is driven by the independent receive worker.
  virtual Result<void> write(ChannelId channel, std::span<const std::byte> data) = 0;

  // Returns bytes already received by cycle(); this call does not perform physical I/O.
  virtual Result<std::size_t> read(ChannelId channel, std::span<std::byte> buffer) = 0;

  // Interrupt a blocking cycle so an owning executor can stop promptly. Most
  // transports are cycle-bounded already; transports with blocking I/O may
  // override this with a non-blocking wakeup.
  void request_stop() noexcept override {}
};

}  // namespace policy_runtime
