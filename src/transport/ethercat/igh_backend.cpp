#include "policy_runtime/transport/ethercat/backend.hpp"

#if POLICY_RUNTIME_WITH_IGH

#include <ecrt.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

namespace policy_runtime {
namespace {

constexpr std::size_t kMailboxCapacity = 4096U;
constexpr std::uint32_t kMailboxTimeoutMs = 1000U;
constexpr std::array<std::size_t, 4U> kDownloadRequestSizes{1U, 2U, 4U, 8U};

enum class ActiveSdo { none, download, upload };

SdoTransferProgress sdo_failure(ErrorCode code, std::string message) {
  return SdoTransferProgress{SdoTransferState::failed,
                             Error{code, std::move(message)}, {}};
}

template <class Callable>
bool igh_call_succeeded(Callable&& callable) {
  if constexpr (std::is_void_v<std::invoke_result_t<Callable>>) {
    std::forward<Callable>(callable)();
    return true;
  } else {
    return std::forward<Callable>(callable)() >= 0;
  }
}

std::vector<ec_pdo_entry_info_t> igh_entries(const PdoDefinition& pdo) {
  std::vector<ec_pdo_entry_info_t> entries;
  entries.reserve(pdo.entries.size());
  for (const auto& entry : pdo.entries) {
    entries.push_back(
        ec_pdo_entry_info_t{entry.address.index, entry.address.subindex,
                            entry.bit_length});
  }
  return entries;
}

}  // namespace

struct IghBackend::Impl {
  struct Slave {
    EthercatSlaveAddress address{};
    std::uint16_t ring_position{};
    ec_slave_config_t* configuration{};
    std::array<ec_sdo_request_t*, kDownloadRequestSizes.size()> download_requests{};
    ec_sdo_request_t* upload_request{};
    ec_sdo_request_t* active_request{};
    ActiveSdo active_sdo{ActiveSdo::none};
    ObjectAddress active_address{};
    std::size_t active_upload_limit{};
  };

  explicit Impl(unsigned int index) : master_index(index) {}

  Slave* find_slave(EthercatSlaveAddress address) noexcept {
    const auto slave = std::find_if(slaves.begin(), slaves.end(),
                                    [address](const auto& candidate) {
                                      return candidate.address.alias == address.alias &&
                                             candidate.address.position == address.position;
                                    });
    return slave == slaves.end() ? nullptr : &*slave;
  }

  const Slave* find_slave(EthercatSlaveAddress address) const noexcept {
    const auto slave = std::find_if(slaves.begin(), slaves.end(),
                                    [address](const auto& candidate) {
                                      return candidate.address.alias == address.alias &&
                                             candidate.address.position == address.position;
                                    });
    return slave == slaves.end() ? nullptr : &*slave;
  }

  unsigned int master_index{};
  ec_master_t* master{};
  ec_domain_t* domain{};
  std::uint8_t* domain_data{};
  std::size_t domain_size{};
  bool activated{};
  std::vector<Slave> slaves;
};

IghBackend::IghBackend(unsigned int master_index)
    : impl_(std::make_unique<Impl>(master_index)) {}

IghBackend::~IghBackend() { deactivate(); }

Result<void> IghBackend::initialize() {
  if (impl_->master != nullptr) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "IgH master is already initialized"});
  }
  impl_->master = ecrt_request_master(impl_->master_index);
  if (impl_->master == nullptr) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "ecrt_request_master failed"});
  }
  impl_->domain = ecrt_master_create_domain(impl_->master);
  if (impl_->domain == nullptr) {
    ecrt_release_master(impl_->master);
    impl_->master = nullptr;
    return Result<void>::failure(
        {ErrorCode::unavailable, "ecrt_master_create_domain failed"});
  }
  return Result<void>::success();
}

Result<void> IghBackend::configure_slave(
    const EthercatSlaveConfiguration& configuration) {
  if (impl_->master == nullptr || impl_->domain == nullptr || impl_->activated) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "IgH slave configuration requires an inactive initialized master"});
  }
  if (impl_->find_slave(configuration.address) != nullptr) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "IgH slave address is already configured"});
  }

  ec_master_info_t master_info{};
  if (ecrt_master(impl_->master, &master_info) < 0) {
    return Result<void>::failure(
        {ErrorCode::io, "ecrt_master failed while checking the bus"});
  }
  if (master_info.scan_busy != 0U) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "IgH master bus scan is still in progress"});
  }

  std::uint16_t ring_position = configuration.address.position;
  if (configuration.address.alias != 0U) {
    bool alias_found = false;
    for (std::uint16_t index = 0U; index < master_info.slave_count; ++index) {
      ec_slave_info_t candidate{};
      if (ecrt_master_get_slave(impl_->master, index, &candidate) == 0 &&
          candidate.alias == configuration.address.alias) {
        const auto resolved = static_cast<std::uint32_t>(candidate.position) +
                              configuration.address.position;
        if (resolved > std::numeric_limits<std::uint16_t>::max()) {
          return Result<void>::failure(
              {ErrorCode::invalid_argument, "resolved EtherCAT ring position overflows"});
        }
        ring_position = static_cast<std::uint16_t>(resolved);
        alias_found = true;
        break;
      }
    }
    if (!alias_found) {
      return Result<void>::failure(
          {ErrorCode::unavailable, "configured EtherCAT slave alias was not found"});
    }
  }

  ec_slave_info_t slave_info{};
  if (ecrt_master_get_slave(impl_->master, ring_position, &slave_info) < 0) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "configured EtherCAT slave position was not found"});
  }
  if (slave_info.vendor_id != configuration.identity.vendor_id ||
      slave_info.product_code != configuration.identity.product_code ||
      slave_info.revision_number != configuration.identity.revision) {
    return Result<void>::failure(
        {ErrorCode::protocol, "EtherCAT slave identity or revision differs from the profile"});
  }

  auto* slave_configuration = ecrt_master_slave_config(
      impl_->master, configuration.address.alias, configuration.address.position,
      configuration.identity.vendor_id, configuration.identity.product_code);
  if (slave_configuration == nullptr) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "ecrt_master_slave_config rejected the Elmo slave"});
  }

  auto rx_entries = igh_entries(configuration.rx_pdo);
  ec_pdo_info_t rx_pdo{configuration.rx_pdo.index,
                       static_cast<unsigned int>(rx_entries.size()), rx_entries.data()};

  std::vector<std::vector<ec_pdo_entry_info_t>> tx_entry_storage;
  tx_entry_storage.reserve(configuration.tx_pdos.size());
  std::vector<ec_pdo_info_t> tx_pdos;
  tx_pdos.reserve(configuration.tx_pdos.size());
  for (const auto& pdo : configuration.tx_pdos) {
    tx_entry_storage.push_back(igh_entries(pdo));
    auto& entries = tx_entry_storage.back();
    tx_pdos.push_back(ec_pdo_info_t{pdo.index,
                                    static_cast<unsigned int>(entries.size()),
                                    entries.data()});
  }

  const ec_sync_info_t syncs[]{
      ec_sync_info_t{2U, EC_DIR_OUTPUT, 1U, &rx_pdo, EC_WD_DEFAULT},
      ec_sync_info_t{3U, EC_DIR_INPUT, static_cast<unsigned int>(tx_pdos.size()),
                     tx_pdos.data(), EC_WD_DEFAULT},
  };
  if (ecrt_slave_config_pdos(slave_configuration, 2U, syncs) < 0) {
    return Result<void>::failure(
        {ErrorCode::protocol, "ecrt_slave_config_pdos rejected the ESI mapping"});
  }

  for (const auto& request : configuration.startup_sdos) {
    if (request.data.empty() ||
        ecrt_slave_config_sdo(
            slave_configuration, request.address.index, request.address.subindex,
            reinterpret_cast<const std::uint8_t*>(request.data.data()),
            request.data.size()) < 0) {
      return Result<void>::failure(
          {ErrorCode::protocol, "ecrt_slave_config_sdo rejected a startup parameter"});
    }
  }
  if (!igh_call_succeeded([&] {
        return ecrt_slave_config_dc(slave_configuration,
                                    configuration.dc.assign_activate,
                                    configuration.dc.sync0_cycle_ns,
                                    configuration.dc.sync0_shift_ns, 0U, 0);
      })) {
    return Result<void>::failure(
        {ErrorCode::protocol, "ecrt_slave_config_dc rejected the 1 kHz DC setup"});
  }

  std::array<ec_sdo_request_t*, kDownloadRequestSizes.size()> download_requests{};
  for (std::size_t index = 0U; index < kDownloadRequestSizes.size(); ++index) {
    download_requests[index] = ecrt_slave_config_create_sdo_request(
        slave_configuration, 0x6060U, 0U, kDownloadRequestSizes[index]);
    if (download_requests[index] == nullptr ||
        !igh_call_succeeded([&] {
          return ecrt_sdo_request_timeout(download_requests[index], kMailboxTimeoutMs);
        })) {
      return Result<void>::failure(
          {ErrorCode::unavailable,
           "failed to create an exact-size IgH SDO download request"});
    }
  }
  auto* upload_request = ecrt_slave_config_create_sdo_request(
      slave_configuration, 0x6060U, 0U, kMailboxCapacity);
  if (upload_request == nullptr ||
      !igh_call_succeeded(
          [&] { return ecrt_sdo_request_timeout(upload_request, kMailboxTimeoutMs); })) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "failed to create the IgH SDO upload request"});
  }
  if (impl_->slaves.empty() && !igh_call_succeeded([&] {
        return ecrt_master_select_reference_clock(impl_->master,
                                                  slave_configuration);
      })) {
    return Result<void>::failure(
        {ErrorCode::protocol, "failed to select the EtherCAT DC reference clock"});
  }

  impl_->slaves.push_back(Impl::Slave{configuration.address, ring_position,
                                      slave_configuration, download_requests,
                                      upload_request});
  return Result<void>::success();
}

Result<PdoFieldLocation> IghBackend::bind_pdo_entry(
    EthercatSlaveAddress slave, ObjectAddress address, std::uint8_t bit_length) {
  if (impl_->activated || impl_->domain == nullptr) {
    return Result<PdoFieldLocation>::failure(
        {ErrorCode::invalid_argument, "PDO entries must be bound before IgH activation"});
  }
  auto* configured_slave = impl_->find_slave(slave);
  if (configured_slave == nullptr) {
    return Result<PdoFieldLocation>::failure(
        {ErrorCode::invalid_argument, "PDO entry references an unknown IgH slave"});
  }
  unsigned int bit_position{};
  const int offset = ecrt_slave_config_reg_pdo_entry(
      configured_slave->configuration, address.index, address.subindex, impl_->domain,
      &bit_position);
  if (offset < 0 || bit_position > std::numeric_limits<std::uint8_t>::max()) {
    return Result<PdoFieldLocation>::failure(
        {ErrorCode::protocol, "ecrt_slave_config_reg_pdo_entry failed"});
  }
  return Result<PdoFieldLocation>::success(
      PdoFieldLocation{static_cast<std::size_t>(offset),
                       static_cast<std::uint8_t>(bit_position), bit_length});
}

Result<void> IghBackend::activate() {
  if (impl_->master == nullptr || impl_->domain == nullptr || impl_->activated) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "IgH activation requires one inactive master"});
  }
  if (ecrt_master_activate(impl_->master) < 0) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "ecrt_master_activate failed"});
  }
  impl_->domain_size = ecrt_domain_size(impl_->domain);
  impl_->domain_data = ecrt_domain_data(impl_->domain);
  if (impl_->domain_size == 0U || impl_->domain_data == nullptr) {
    ecrt_master_deactivate(impl_->master);
    impl_->domain_data = nullptr;
    impl_->domain_size = 0U;
    return Result<void>::failure(
        {ErrorCode::unavailable, "IgH domain process image is unavailable"});
  }
  impl_->activated = true;
  return Result<void>::success();
}

void IghBackend::deactivate() noexcept {
  if (impl_ == nullptr || impl_->master == nullptr) {
    return;
  }
  if (impl_->activated) {
    ecrt_master_deactivate(impl_->master);
  }
  ecrt_release_master(impl_->master);
  impl_->master = nullptr;
  impl_->domain = nullptr;
  impl_->domain_data = nullptr;
  impl_->domain_size = 0U;
  impl_->activated = false;
  impl_->slaves.clear();
}

void IghBackend::receive() noexcept {
  if (impl_->activated) {
    static_cast<void>(ecrt_master_receive(impl_->master));
  }
}

void IghBackend::process_domain() noexcept {
  if (impl_->activated) {
    static_cast<void>(ecrt_domain_process(impl_->domain));
  }
}

std::span<std::byte> IghBackend::process_image() noexcept {
  return {reinterpret_cast<std::byte*>(impl_->domain_data), impl_->domain_size};
}

void IghBackend::queue_domain() noexcept {
  if (impl_->activated) {
    static_cast<void>(ecrt_domain_queue(impl_->domain));
  }
}

void IghBackend::send() noexcept {
  if (impl_->activated) {
    static_cast<void>(ecrt_master_send(impl_->master));
  }
}

DomainHealth IghBackend::domain_health() const noexcept {
  if (!impl_->activated) {
    return {};
  }
  ec_domain_state_t domain_state{};
  ec_master_state_t master_state{};
  static_cast<void>(ecrt_domain_state(impl_->domain, &domain_state));
  static_cast<void>(ecrt_master_state(impl_->master, &master_state));
  bool slaves_operational = true;
  for (const auto& slave : impl_->slaves) {
    ec_slave_config_state_t state{};
    static_cast<void>(ecrt_slave_config_state(slave.configuration, &state));
    slaves_operational = slaves_operational && state.online != 0U &&
                         state.operational != 0U;
  }
  const bool complete = domain_state.wc_state == EC_WC_COMPLETE;
  return DomainHealth{domain_state.working_counter, std::nullopt,
                      complete, master_state.link_up != 0U, slaves_operational, 0};
}

SdoTransferProgress IghBackend::progress_download_sdo(
    EthercatSlaveAddress slave, const SdoDownloadRequest& request) {
  auto* configured_slave = impl_->find_slave(slave);
  if (!impl_->activated || configured_slave == nullptr) {
    return sdo_failure(ErrorCode::unavailable,
                       "IgH asynchronous SDO endpoint is unavailable");
  }
  const auto size = std::find(kDownloadRequestSizes.begin(), kDownloadRequestSizes.end(),
                              request.data.size());
  if (size == kDownloadRequestSizes.end()) {
    return sdo_failure(ErrorCode::invalid_argument,
                       "IgH SDO download requires a preconfigured 1, 2, 4, or 8 byte size");
  }

  if (configured_slave->active_sdo == ActiveSdo::none) {
    auto* download_request = configured_slave->download_requests[static_cast<std::size_t>(
        std::distance(kDownloadRequestSizes.begin(), size))];
    if (download_request == nullptr ||
        ecrt_sdo_request_data_size(download_request) != request.data.size()) {
      return sdo_failure(ErrorCode::protocol,
                         "IgH exact-size SDO download request is unavailable");
    }
    if (!igh_call_succeeded([&] {
          return ecrt_sdo_request_index(download_request, request.address.index,
                                        request.address.subindex);
        })) {
      return sdo_failure(ErrorCode::protocol, "failed to select the IgH SDO object");
    }
    std::memcpy(ecrt_sdo_request_data(download_request), request.data.data(),
                request.data.size());
    if (!igh_call_succeeded(
            [&] { return ecrt_sdo_request_write(download_request); })) {
      return sdo_failure(ErrorCode::io, "failed to schedule the IgH SDO download");
    }
    configured_slave->active_sdo = ActiveSdo::download;
    configured_slave->active_request = download_request;
    configured_slave->active_address = request.address;
    return SdoTransferProgress{};
  }
  if (configured_slave->active_sdo != ActiveSdo::download ||
      configured_slave->active_address.index != request.address.index ||
      configured_slave->active_address.subindex != request.address.subindex) {
    return sdo_failure(ErrorCode::protocol,
                       "a different IgH SDO request is already active for this slave");
  }

  switch (ecrt_sdo_request_state(configured_slave->active_request)) {
    case EC_REQUEST_UNUSED:
    case EC_REQUEST_BUSY:
      return SdoTransferProgress{};
    case EC_REQUEST_SUCCESS:
      configured_slave->active_sdo = ActiveSdo::none;
      configured_slave->active_request = nullptr;
      return SdoTransferProgress{SdoTransferState::completed, std::nullopt, {}};
    case EC_REQUEST_ERROR:
      configured_slave->active_sdo = ActiveSdo::none;
      configured_slave->active_request = nullptr;
      return sdo_failure(ErrorCode::io, "IgH SDO download completed with an error");
  }
  configured_slave->active_sdo = ActiveSdo::none;
  configured_slave->active_request = nullptr;
  return sdo_failure(ErrorCode::internal, "unknown IgH SDO request state");
}

SdoTransferProgress IghBackend::progress_upload_sdo(
    EthercatSlaveAddress slave, ObjectAddress address,
    std::span<std::byte> destination) {
  auto* configured_slave = impl_->find_slave(slave);
  if (!impl_->activated || configured_slave == nullptr ||
      configured_slave->upload_request == nullptr) {
    return sdo_failure(ErrorCode::unavailable,
                       "IgH asynchronous SDO endpoint is unavailable");
  }
  if (destination.empty() || destination.size() > kMailboxCapacity) {
    return sdo_failure(ErrorCode::invalid_argument,
                       "IgH SDO upload size is outside the configured capacity");
  }

  if (configured_slave->active_sdo == ActiveSdo::none) {
    if (!igh_call_succeeded([&] {
          return ecrt_sdo_request_index(configured_slave->upload_request,
                                        address.index, address.subindex);
        }) ||
        !igh_call_succeeded(
            [&] { return ecrt_sdo_request_read(configured_slave->upload_request); })) {
      return sdo_failure(ErrorCode::io, "failed to schedule the IgH SDO upload");
    }
    configured_slave->active_sdo = ActiveSdo::upload;
    configured_slave->active_request = configured_slave->upload_request;
    configured_slave->active_address = address;
    configured_slave->active_upload_limit = destination.size();
    return SdoTransferProgress{};
  }
  if (configured_slave->active_sdo != ActiveSdo::upload ||
      configured_slave->active_address.index != address.index ||
      configured_slave->active_address.subindex != address.subindex) {
    return sdo_failure(ErrorCode::protocol,
                       "a different IgH SDO request is already active for this slave");
  }

  switch (ecrt_sdo_request_state(configured_slave->active_request)) {
    case EC_REQUEST_UNUSED:
    case EC_REQUEST_BUSY:
      return SdoTransferProgress{};
    case EC_REQUEST_SUCCESS: {
      const auto size = ecrt_sdo_request_data_size(configured_slave->active_request);
      if (size > configured_slave->active_upload_limit || size > kMailboxCapacity) {
        configured_slave->active_sdo = ActiveSdo::none;
        configured_slave->active_request = nullptr;
        return sdo_failure(ErrorCode::protocol,
                           "IgH SDO upload exceeded the requested capacity");
      }
      const auto* data = reinterpret_cast<const std::byte*>(
          ecrt_sdo_request_data(configured_slave->active_request));
      std::copy_n(data, size, destination.begin());
      configured_slave->active_sdo = ActiveSdo::none;
      configured_slave->active_request = nullptr;
      return SdoTransferProgress{SdoTransferState::completed, std::nullopt, size};
    }
    case EC_REQUEST_ERROR:
      configured_slave->active_sdo = ActiveSdo::none;
      configured_slave->active_request = nullptr;
      return sdo_failure(ErrorCode::io, "IgH SDO upload completed with an error");
  }
  configured_slave->active_sdo = ActiveSdo::none;
  configured_slave->active_request = nullptr;
  return sdo_failure(ErrorCode::internal, "unknown IgH SDO request state");
}

SdoTransferProgress IghBackend::progress_cancel_sdo(
    EthercatSlaveAddress slave) {
  auto* configured_slave = impl_->find_slave(slave);
  if (!impl_->activated || configured_slave == nullptr) {
    return sdo_failure(ErrorCode::unavailable,
                       "IgH asynchronous SDO endpoint is unavailable");
  }
  if (configured_slave->active_sdo == ActiveSdo::none ||
      configured_slave->active_request == nullptr) {
    return SdoTransferProgress{SdoTransferState::completed, std::nullopt, 0U};
  }

  // IgH's public asynchronous request API has no abort operation. The sole
  // master owner therefore drains one request-state observation per cycle and
  // releases the logical request only after IgH reports a terminal state.
  switch (ecrt_sdo_request_state(configured_slave->active_request)) {
    case EC_REQUEST_UNUSED:
    case EC_REQUEST_BUSY:
      return SdoTransferProgress{};
    case EC_REQUEST_SUCCESS:
    case EC_REQUEST_ERROR:
      configured_slave->active_sdo = ActiveSdo::none;
      configured_slave->active_request = nullptr;
      configured_slave->active_address = {};
      configured_slave->active_upload_limit = 0U;
      return SdoTransferProgress{SdoTransferState::completed, std::nullopt, 0U};
  }
  return sdo_failure(ErrorCode::internal, "unknown IgH SDO request state");
}

}  // namespace policy_runtime

#endif
