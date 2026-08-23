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
  EthercatMailbox(std::shared_ptr<EthercatBackend> backend,
                  EthercatSlaveAddress slave);

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
  struct Request {
    bool upload{};
    bool executing{};
    ObjectAddress address{};
    std::vector<std::byte> data;
    MailboxRequestStatus status{MailboxRequestState::queued, std::nullopt, {}};
  };

  Result<MailboxRequestId> enqueue(bool upload, ObjectAddress address,
                                    std::span<const std::byte> data);

  std::shared_ptr<EthercatBackend> backend_;
  EthercatSlaveAddress slave_{};
  mutable std::mutex requests_mutex_;
  std::map<MailboxRequestId, Request> requests_;
  MailboxRequestId next_request_id_{1U};
  std::atomic<bool> open_{false};
  std::atomic<TransportHealth> health_{TransportHealth::failed};
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
  Result<void> register_field(CyclicField field, bool input);
  Result<void> validate_configuration() const;

  std::shared_ptr<EthercatBackend> backend_;
  std::vector<EthercatAxisConfiguration> axes_;
  std::vector<Cia402PdoView> pdo_views_;
  std::vector<Cia402PdoHandles> pdo_handles_;
  std::vector<std::unique_ptr<EthercatMailbox>> mailboxes_;
  std::vector<CyclicField> registered_inputs_;
  std::vector<CyclicField> registered_outputs_;
  CycleHandler cycle_handler_{};
  void* cycle_handler_context_{};
  bool open_{};
  std::atomic<TransportHealth> health_{TransportHealth::failed};
};

}  // namespace policy_runtime
