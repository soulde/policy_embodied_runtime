#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "policy_runtime/transport/transport.hpp"

namespace policy_runtime {

struct ObjectAddress {
  std::uint16_t index{};
  std::uint8_t subindex{};
};

using MailboxRequestId = std::uint64_t;

enum class MailboxSchedulingClass { blocking_event_driven, asynchronous };

enum class MailboxRequestState { queued, completed, failed };

struct MailboxRequestStatus {
  MailboxRequestState state{};
  std::optional<Error> error;
};

class ObjectDictionaryTransport : public virtual Transport {
 public:
  // Mailbox I/O must run on this non-real-time service class, never on a cyclic executor.
  virtual MailboxSchedulingClass mailbox_scheduling_class() const noexcept = 0;

  // Enqueues mailbox transfers; neither method may perform physical I/O.
  virtual Result<MailboxRequestId> queue_download(
      ObjectAddress address, std::span<const std::byte> data) = 0;
  virtual Result<MailboxRequestId> queue_upload(ObjectAddress address) = 0;

  // Returns a completion-state snapshot, or std::nullopt for an unknown request ID.
  virtual std::optional<MailboxRequestStatus> mailbox_status(
      MailboxRequestId request_id) const = 0;

  // Performs mailbox physical I/O and may block; schedule it independently of cycle().
  virtual void service_mailbox() noexcept = 0;
};

}  // namespace policy_runtime
