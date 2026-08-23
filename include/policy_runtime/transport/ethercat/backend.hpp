#pragma once

#include <bit>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <memory>
#include <span>
#include <type_traits>
#include <vector>

#include "policy_runtime/transport/object_dictionary_transport.hpp"

namespace policy_runtime {

struct EthercatSlaveAddress {
  std::uint16_t alias{};
  std::uint16_t position{};
};

struct EthercatDeviceIdentity {
  std::uint32_t vendor_id{};
  std::uint32_t product_code{};
  std::uint32_t revision{};
};

enum class PdoDirection { output, input };
enum class WatchdogMode { device_default, disabled, enabled };

struct PdoEntryDefinition {
  ObjectAddress address{};
  std::uint8_t bit_length{};
};

struct PdoDefinition {
  std::uint16_t index{};
  std::span<const PdoEntryDefinition> entries;
};

struct SyncManagerDescription {
  std::uint8_t index{};
  std::uint16_t minimum_size{};
  std::uint16_t maximum_size{};
  std::uint16_t default_size{};
  std::uint16_t start_address{};
  std::uint8_t control_byte{};
  bool enabled{};
  WatchdogMode watchdog{WatchdogMode::device_default};
  PdoDirection direction{PdoDirection::output};
};

struct DistributedClockConfiguration {
  std::uint16_t assign_activate{};
  std::uint32_t sync0_cycle_ns{};
  std::int32_t sync0_shift_ns{};
};

struct SdoDownloadRequest {
  ObjectAddress address{};
  std::vector<std::byte> data;
};

enum class SdoTransferState { pending, completed, failed };

struct SdoTransferProgress {
  SdoTransferState state{SdoTransferState::pending};
  std::optional<Error> error;
  std::vector<std::byte> uploaded_bytes;
};

struct EthercatSlaveConfiguration {
  EthercatSlaveAddress address{};
  EthercatDeviceIdentity identity{};
  PdoDefinition rx_pdo{};
  std::span<const PdoDefinition> tx_pdos;
  DistributedClockConfiguration dc{};
  std::span<const SdoDownloadRequest> startup_sdos;
};

struct DomainHealth {
  std::uint32_t working_counter{};
  std::optional<std::uint32_t> expected_working_counter;
  bool working_counter_complete{};
  bool link_up{};
  bool all_slaves_operational{};
  std::int64_t dc_deviation_ns{};
};

struct PdoFieldLocation {
  std::size_t byte_offset{};
  std::uint8_t bit_position{};
  std::uint8_t bit_length{};
};

template <class T>
class TypedPdoField {
  static_assert(std::is_integral_v<T> && !std::is_same_v<T, bool>);
  static_assert(sizeof(T) == 1U || sizeof(T) == 2U || sizeof(T) == 4U);

 public:
  Result<void> bind(PdoFieldLocation location) {
    if (bound_) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "PDO field is already bound"});
    }
    if (location.bit_position != 0U ||
        location.bit_length != static_cast<std::uint8_t>(sizeof(T) * 8U)) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "PDO field alignment or width is incompatible"});
    }
    location_ = location;
    bound_ = true;
    return Result<void>::success();
  }

  bool is_bound() const noexcept { return bound_; }

  std::optional<T> read(std::span<const std::byte> process_image) const noexcept {
    if (!contains(process_image.size())) {
      return std::nullopt;
    }
    using Unsigned = std::make_unsigned_t<T>;
    Unsigned wire_value{};
    for (std::size_t index = 0; index < sizeof(T); ++index) {
      wire_value |= static_cast<Unsigned>(
                        std::to_integer<unsigned char>(
                            process_image[location_.byte_offset + index]))
                    << (index * 8U);
    }
    return std::bit_cast<T>(wire_value);
  }

  bool write(std::span<std::byte> process_image, T value) const noexcept {
    if (!contains(process_image.size())) {
      return false;
    }
    using Unsigned = std::make_unsigned_t<T>;
    const auto wire_value = std::bit_cast<Unsigned>(value);
    for (std::size_t index = 0; index < sizeof(T); ++index) {
      process_image[location_.byte_offset + index] = std::byte{static_cast<unsigned char>(
          wire_value >> (index * 8U))};
    }
    return true;
  }

  PdoFieldLocation location() const noexcept { return location_; }

 private:
  bool contains(std::size_t process_image_size) const noexcept {
    return bound_ && location_.byte_offset <= process_image_size &&
           sizeof(T) <= process_image_size - location_.byte_offset;
  }

  PdoFieldLocation location_{};
  bool bound_{false};
};

class EthercatMaster;
class EthercatMailbox;

class EthercatBackend {
 public:
  virtual ~EthercatBackend() = default;

 protected:
  virtual Result<void> initialize() = 0;
  virtual Result<void> configure_slave(
      const EthercatSlaveConfiguration& configuration) = 0;
  virtual Result<PdoFieldLocation> bind_pdo_entry(EthercatSlaveAddress slave,
                                                   ObjectAddress address,
                                                   std::uint8_t bit_length) = 0;
  virtual Result<void> activate() = 0;
  virtual void deactivate() noexcept = 0;

  virtual void receive() noexcept = 0;
  virtual void process_domain() noexcept = 0;
  virtual std::span<std::byte> process_image() noexcept = 0;
  virtual void queue_domain() noexcept = 0;
  virtual void send() noexcept = 0;
  virtual DomainHealth domain_health() const noexcept = 0;

  // These polling calls share the master userspace context with the PDO path.
  // Implementations must be bounded and nonblocking so an already-entered mailbox
  // call cannot create unbounded priority inversion for the next PDO cycle.
  virtual SdoTransferProgress progress_download_sdo(
      EthercatSlaveAddress slave, const SdoDownloadRequest& request) = 0;
  virtual SdoTransferProgress progress_upload_sdo(EthercatSlaveAddress slave,
                                                   ObjectAddress address,
                                                   std::size_t maximum_size) = 0;

 private:
  void acquire_pdo() noexcept {
    pdo_waiters_.fetch_add(1U, std::memory_order_acq_rel);
    while (access_busy_.test_and_set(std::memory_order_acquire)) {
    }
    pdo_waiters_.fetch_sub(1U, std::memory_order_release);
  }

  bool try_acquire_mailbox() noexcept {
    if (pdo_waiters_.load(std::memory_order_acquire) != 0U ||
        access_busy_.test_and_set(std::memory_order_acquire)) {
      return false;
    }
    if (pdo_waiters_.load(std::memory_order_acquire) != 0U) {
      access_busy_.clear(std::memory_order_release);
      return false;
    }
    return true;
  }

  void release_access() noexcept { access_busy_.clear(std::memory_order_release); }

  std::atomic_flag access_busy_ = ATOMIC_FLAG_INIT;
  std::atomic<unsigned int> pdo_waiters_{};

  friend class EthercatMaster;
  friend class EthercatMailbox;
};

#if POLICY_RUNTIME_WITH_IGH
// IgH request buffers are fixed before master activation. Runtime downloads are
// therefore limited to exact-size 1, 2, 4, or 8 byte scalar requests; other sizes
// fail with invalid_argument. Uploads remain bounded by the mailbox capacity.
class IghBackend final : public EthercatBackend {
 public:
  explicit IghBackend(unsigned int master_index = 0U);
  ~IghBackend() override;

  IghBackend(const IghBackend&) = delete;
  IghBackend& operator=(const IghBackend&) = delete;

 protected:
  Result<void> initialize() override;
  Result<void> configure_slave(
      const EthercatSlaveConfiguration& configuration) override;
  Result<PdoFieldLocation> bind_pdo_entry(EthercatSlaveAddress slave,
                                           ObjectAddress address,
                                           std::uint8_t bit_length) override;
  Result<void> activate() override;
  void deactivate() noexcept override;

  void receive() noexcept override;
  void process_domain() noexcept override;
  std::span<std::byte> process_image() noexcept override;
  void queue_domain() noexcept override;
  void send() noexcept override;
  DomainHealth domain_health() const noexcept override;

  SdoTransferProgress progress_download_sdo(
      EthercatSlaveAddress slave, const SdoDownloadRequest& request) override;
  SdoTransferProgress progress_upload_sdo(EthercatSlaveAddress slave,
                                           ObjectAddress address,
                                           std::size_t maximum_size) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif

}  // namespace policy_runtime
