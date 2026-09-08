#pragma once

#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

// A transport whose receive path is independent from the control cycle. The
// owning runtime invokes receive_once() from its receive worker while cycle()
// is invoked at the configured control rate and performs the send operation.
class AsynchronousTransport : public virtual Transport {
 public:
  ~AsynchronousTransport() override = default;

  // Compatibility default for frame transports that still expose a combined
  // cycle. Native asynchronous transports override this with one bounded,
  // non-blocking receive operation.
  virtual void receive_once() noexcept {}
  virtual void request_stop() noexcept {}
};

}  // namespace policy_runtime
