#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/cia402/pdo.hpp"
#include "policy_runtime/transport/cyclic_transport.hpp"
#include "policy_runtime/transport/ethercat/backend.hpp"
#include "policy_runtime/transport/object_dictionary_transport.hpp"

namespace policy_runtime {

struct EthercatAxisConfiguration {
  profiles::AxisConfig axis;
  std::vector<SdoDownloadRequest> startup_parameters;
};

struct Cia402PdoHandles {
  TypedPdoField<std::uint16_t> status_word;
  TypedPdoField<std::int8_t> mode_display;
  TypedPdoField<std::int32_t> actual_position;
  TypedPdoField<std::int32_t> actual_velocity;
  TypedPdoField<std::int16_t> actual_torque;

  TypedPdoField<std::uint16_t> control_word;
  TypedPdoField<std::int32_t> target_position;
  TypedPdoField<std::int32_t> target_velocity;
  TypedPdoField<std::int16_t> target_torque;
};

class EthercatMailbox final : public ObjectDictionaryTransport {
 public:
  Result<void> open() override;
  void close() noexcept override;
  TransportHealth health() const noexcept override;
  SchedulingClass scheduling_class() const noexcept override;
  void cycle(const CycleContext& context) noexcept override;

  Result<MailboxRequestId> queue_download(
      ObjectAddress address, std::span<const std::byte> data) override;
  Result<MailboxRequestId> queue_upload(ObjectAddress address) override;
  std::optional<MailboxRequestStatus> mailbox_status(
      MailboxRequestId request_id) const override;

 private:
  struct OwnerLifecycle {
    std::atomic<std::uint64_t> active_generation{};
  };

  enum class RequestPhase {
    queued,
    staged,
    active,
    terminalizing,
    completed,
    failed,
  };

  enum class FailureKind { none, closed, backend };

  struct Request {
    std::uint64_t generation{};
    bool upload{};
    SdoDownloadRequest transfer;
    std::atomic<RequestPhase> phase{RequestPhase::queued};
    FailureKind failure_kind{FailureKind::none};
    ErrorCode backend_error_code{ErrorCode::io};
    SdoFailureReason backend_failure_reason{SdoFailureReason::none};
    std::size_t uploaded_size{};
  };

  static_assert(std::atomic<Request*>::is_always_lock_free,
                "EtherCAT mailbox handoff requires lock-free pointer atomics");
  static_assert(std::atomic<RequestPhase>::is_always_lock_free &&
                    std::atomic<std::uint64_t>::is_always_lock_free,
                "EtherCAT mailbox state requires lock-free atomics");

  EthercatMailbox(std::shared_ptr<EthercatBackend> backend,
                  EthercatSlaveAddress slave,
                  std::shared_ptr<OwnerLifecycle> owner_lifecycle);

  Result<MailboxRequestId> enqueue(bool upload, ObjectAddress address,
                                    std::span<const std::byte> data);
  void stage_one();
  bool owner_step(std::uint64_t parent_generation) noexcept;
  Result<void> open_for_parent(std::uint64_t parent_generation);
  void close_for_parent(std::uint64_t parent_generation) noexcept;
  void reset_after_parent_deactivate() noexcept;
  void close_generation(std::uint64_t generation, bool request_cancel) noexcept;
  static void fail_request(Request& request, FailureKind kind) noexcept;

  std::shared_ptr<EthercatBackend> backend_;
  std::shared_ptr<OwnerLifecycle> owner_lifecycle_;
  EthercatSlaveAddress slave_{};
  std::mutex lifecycle_mutex_;
  mutable std::mutex requests_mutex_;
  std::map<MailboxRequestId, Request> requests_;
  std::atomic<Request*> staged_request_{};
  MailboxRequestId next_request_id_{1U};
  std::atomic<std::uint64_t> open_generation_{};
  std::atomic<std::uint64_t> cancel_requested_generation_{};
  std::atomic<TransportHealth> health_{TransportHealth::failed};

  friend class EthercatMaster;
  friend class EthercatMailboxTestPeer;
};

class EthercatMaster final : public CyclicTransport {
 public:
  using CycleHandler = void (*)(void*, std::span<Cia402PdoView>) noexcept;

  EthercatMaster(std::shared_ptr<EthercatBackend> backend,
                 std::vector<EthercatAxisConfiguration> axes);
  ~EthercatMaster() override;

  EthercatMaster(const EthercatMaster&) = delete;
  EthercatMaster& operator=(const EthercatMaster&) = delete;

  Result<void> open() override;
  void close() noexcept override;
  TransportHealth health() const noexcept override;
  SchedulingClass scheduling_class() const noexcept override;
  void cycle(const CycleContext& context) noexcept override;

  Result<void> register_cyclic_input(CyclicField field) override;
  Result<void> register_cyclic_output(CyclicField field) override;

  Result<void> set_cycle_handler(CycleHandler handler, void* context);
  std::span<Cia402PdoView> pdo_views() noexcept;
  std::span<const Cia402PdoHandles> pdo_handles() const noexcept;
  ObjectDictionaryTransport& mailbox(std::size_t axis_index);

 private:
  static constexpr std::uint64_t kCycleOpenBit = std::uint64_t{1U} << 63U;
  static constexpr std::uint64_t kCycleCountMask = kCycleOpenBit - 1U;

  Result<void> register_field(CyclicField field, bool input);
  Result<void> validate_configuration() const;
  static bool admission_successor(std::uint64_t observed,
                                  std::uint64_t& desired) noexcept;
  bool admission_is_open() const noexcept;
  bool try_enter_cycle() noexcept;
  void leave_cycle() noexcept;

  std::shared_ptr<EthercatBackend> backend_;
  std::shared_ptr<EthercatMailbox::OwnerLifecycle> owner_lifecycle_;
  std::vector<EthercatAxisConfiguration> axes_;
  std::vector<Cia402PdoView> pdo_views_;
  std::vector<Cia402PdoHandles> pdo_handles_;
  std::vector<std::unique_ptr<EthercatMailbox>> mailboxes_;
  std::vector<CyclicField> registered_inputs_;
  std::vector<CyclicField> registered_outputs_;
  CycleHandler cycle_handler_{};
  void* cycle_handler_context_{};
  std::mutex lifecycle_mutex_;
  // Open/closing and in-flight count share one modification order. The high bit
  // gates admission and the remaining bits count admitted cycles.
  std::atomic<std::uint64_t> cycle_admission_{};
  std::atomic<TransportHealth> health_{TransportHealth::failed};
  std::uint64_t generation_{};
  std::uint64_t next_generation_{1U};
  std::size_t mailbox_cursor_{};

  static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
                "EtherCAT cycle admission requires a lock-free 64-bit atomic");

  friend class EthercatMasterTestPeer;
};

}  // namespace policy_runtime
