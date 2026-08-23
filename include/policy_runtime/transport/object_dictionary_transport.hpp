#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

struct ObjectAddress {
  std::uint16_t index{};
  std::uint8_t subindex{};
};

using MailboxRequestId = std::uint64_t;

enum class MailboxRequestState { queued, completed, failed };

struct MailboxRequestStatus {
  MailboxRequestState state{};
  std::optional<Error> error;
  std::vector<std::byte> uploaded_bytes;
};

// An OD endpoint must use blocking_event_driven or asynchronous scheduling and
// must not also implement CyclicTransport. EtherCAT PDO and mailbox endpoints
// may share a backend below this interface; that composition must serialize
// backend access outside the hard-real-time PDO critical window.
class ObjectDictionaryTransport : public virtual Transport {
 public:
  // Must return blocking_event_driven or asynchronous, never a real-time class.
  virtual SchedulingClass scheduling_class() const noexcept override = 0;

  // Enqueues mailbox transfers; neither method may perform physical I/O.
  virtual Result<MailboxRequestId> queue_download(
      ObjectAddress address, std::span<const std::byte> data) = 0;
  virtual Result<MailboxRequestId> queue_upload(ObjectAddress address) = 0;

  // Returns a completion-state snapshot, or std::nullopt for an unknown request ID.
  virtual std::optional<MailboxRequestStatus> mailbox_status(
      MailboxRequestId request_id) const = 0;

  // cycle() is inherited as the only physical-I/O entry point and services queued requests.
};

}  // namespace policy_runtime
