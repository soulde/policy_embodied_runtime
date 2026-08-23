#include "policy_runtime/transport/ethercat/master.hpp"

#include <algorithm>
#include <array>
#include <exception>
#include <set>
#include <utility>

#include "policy_runtime/transport/ethercat/elmo_gold.hpp"

namespace policy_runtime {
namespace {

constexpr std::size_t kMaximumAxes = 12U;
constexpr std::size_t kMaximumSdoUploadBytes = 4096U;

bool same_address(ObjectAddress left, ObjectAddress right) noexcept {
  return left.index == right.index && left.subindex == right.subindex;
}

bool healthy_domain(const DomainHealth& health) noexcept {
  return health.link_up && health.working_counter_complete &&
         health.all_slaves_operational &&
         health.working_counter == health.expected_working_counter;
}

bool valid_static_mode(profiles::Cia402Mode mode) noexcept {
  switch (mode) {
    case profiles::Cia402Mode::csp:
    case profiles::Cia402Mode::csv:
    case profiles::Cia402Mode::cst:
      return true;
  }
  return false;
}

}  // namespace

EthercatMailbox::EthercatMailbox(std::shared_ptr<EthercatBackend> backend,
                                 EthercatSlaveAddress slave)
    : backend_(std::move(backend)), slave_(slave) {}

Result<void> EthercatMailbox::open() {
  if (backend_ == nullptr) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "EtherCAT mailbox has no backend"});
  }
  bool expected = false;
  if (!open_.compare_exchange_strong(expected, true)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "EtherCAT mailbox is already open"});
  }
  health_.store(TransportHealth::healthy, std::memory_order_release);
  return Result<void>::success();
}

void EthercatMailbox::close() noexcept {
  open_.store(false, std::memory_order_release);
  health_.store(TransportHealth::failed, std::memory_order_release);
}

TransportHealth EthercatMailbox::health() const noexcept {
  return health_.load(std::memory_order_acquire);
}

SchedulingClass EthercatMailbox::scheduling_class() const noexcept {
  return SchedulingClass::blocking_event_driven;
}

Result<MailboxRequestId> EthercatMailbox::enqueue(
    bool upload, ObjectAddress address, std::span<const std::byte> data) {
  if (!open_.load(std::memory_order_acquire)) {
    return Result<MailboxRequestId>::failure(
        {ErrorCode::unavailable, "EtherCAT mailbox is closed"});
  }
  if (address.index == 0U || (!upload && data.empty())) {
    return Result<MailboxRequestId>::failure(
        {ErrorCode::invalid_argument, "invalid EtherCAT mailbox request"});
  }
  std::scoped_lock lock(requests_mutex_);
  const auto request_id = next_request_id_++;
  Request request{};
  request.upload = upload;
  request.address = address;
  request.data.assign(data.begin(), data.end());
  requests_.emplace(request_id, std::move(request));
  return Result<MailboxRequestId>::success(request_id);
}

Result<MailboxRequestId> EthercatMailbox::queue_download(
    ObjectAddress address, std::span<const std::byte> data) {
  return enqueue(false, address, data);
}

Result<MailboxRequestId> EthercatMailbox::queue_upload(ObjectAddress address) {
  return enqueue(true, address, {});
}

std::optional<MailboxRequestStatus> EthercatMailbox::mailbox_status(
    MailboxRequestId request_id) const {
  std::scoped_lock lock(requests_mutex_);
  const auto request = requests_.find(request_id);
  if (request == requests_.end()) {
    return std::nullopt;
  }
  return request->second.status;
}

void EthercatMailbox::cycle(const CycleContext&) noexcept {
  if (!open_.load(std::memory_order_acquire) || backend_ == nullptr) {
    health_.store(TransportHealth::failed, std::memory_order_release);
    return;
  }

  MailboxRequestId request_id{};
  Request execution;
  {
    std::scoped_lock lock(requests_mutex_);
    const auto request = std::find_if(
        requests_.begin(), requests_.end(), [](const auto& entry) {
          return entry.second.status.state == MailboxRequestState::queued &&
                 !entry.second.executing;
        });
    if (request == requests_.end()) {
      return;
    }
    request->second.executing = true;
    request_id = request->first;
    execution = request->second;
  }

  if (!backend_->try_acquire()) {
    std::scoped_lock lock(requests_mutex_);
    requests_.at(request_id).executing = false;
    health_.store(TransportHealth::degraded, std::memory_order_release);
    return;
  }

  SdoTransferProgress progress;
  try {
    if (execution.upload) {
      progress = backend_->progress_upload_sdo(slave_, execution.address,
                                               kMaximumSdoUploadBytes);
    } else {
      progress = backend_->progress_download_sdo(
          slave_, SdoDownloadRequest{execution.address, execution.data});
    }
  } catch (const std::exception& exception) {
    progress = SdoTransferProgress{
        SdoTransferState::failed,
        Error{ErrorCode::internal, exception.what()}, {}};
  } catch (...) {
    progress = SdoTransferProgress{
        SdoTransferState::failed,
        Error{ErrorCode::internal, "unknown EtherCAT mailbox failure"}, {}};
  }
  backend_->release();

  {
    std::scoped_lock lock(requests_mutex_);
    auto& request = requests_.at(request_id);
    request.executing = false;
    switch (progress.state) {
      case SdoTransferState::pending:
        health_.store(TransportHealth::healthy, std::memory_order_release);
        break;
      case SdoTransferState::completed:
        request.status = MailboxRequestStatus{MailboxRequestState::completed, std::nullopt,
                                              std::move(progress.uploaded_bytes)};
        health_.store(TransportHealth::healthy, std::memory_order_release);
        break;
      case SdoTransferState::failed:
        if (!progress.error.has_value()) {
          progress.error = Error{ErrorCode::io, "EtherCAT SDO transfer failed"};
        }
        request.status = MailboxRequestStatus{MailboxRequestState::failed,
                                              std::move(progress.error), {}};
        health_.store(TransportHealth::degraded, std::memory_order_release);
        break;
    }
  }
}

EthercatMaster::EthercatMaster(std::shared_ptr<EthercatBackend> backend,
                               std::vector<EthercatAxisConfiguration> axes)
    : backend_(std::move(backend)),
      axes_(std::move(axes)),
      pdo_views_(axes_.size()) {
  mailboxes_.reserve(axes_.size());
  for (const auto& configuration : axes_) {
    mailboxes_.push_back(std::make_unique<EthercatMailbox>(
        backend_, EthercatSlaveAddress{configuration.axis.alias,
                                       configuration.axis.position}));
  }
}

EthercatMaster::~EthercatMaster() { close(); }

Result<void> EthercatMaster::validate_configuration() const {
  if (backend_ == nullptr) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "EtherCAT master has no backend"});
  }
  if (axes_.empty() || axes_.size() > kMaximumAxes) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "EtherCAT master requires between one and 12 axes"});
  }

  std::set<std::pair<std::uint16_t, std::uint16_t>> addresses;
  for (const auto& configuration : axes_) {
    const auto& axis = configuration.axis;
    if (axis.vendor_id != ElmoGoldDeviceDescription::vendor_id() ||
        axis.product_code != ElmoGoldDeviceDescription::product_code() ||
        axis.revision != ElmoGoldDeviceDescription::revision()) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "axis identity is not the supported Elmo Gold ESI"});
    }
    if (!valid_static_mode(axis.mode)) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "axis has an unknown static CiA 402 mode"});
    }
    if (!addresses.emplace(axis.alias, axis.position).second) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "duplicate EtherCAT slave address"});
    }
    for (const auto& parameter : configuration.startup_parameters) {
      if (parameter.data.empty() || same_address(parameter.address, {0x6060U, 0U}) ||
          same_address(parameter.address, {0x60C2U, 1U})) {
        return Result<void>::failure(
            {ErrorCode::invalid_argument,
             "startup parameter is empty or overrides the static mode/cycle time"});
      }
    }
  }
  return Result<void>::success();
}

Result<void> EthercatMaster::open() {
  if (open_) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "EtherCAT master is already open"});
  }
  auto valid = validate_configuration();
  if (!valid.has_value()) {
    return valid;
  }

  auto initialized = backend_->initialize();
  if (!initialized.has_value()) {
    health_.store(TransportHealth::failed, std::memory_order_release);
    return initialized;
  }

  std::vector<Cia402PdoHandles> handles(axes_.size());
  for (std::size_t axis_index = 0; axis_index < axes_.size(); ++axis_index) {
    const auto& configuration = axes_[axis_index];
    const auto slave = EthercatSlaveAddress{configuration.axis.alias,
                                            configuration.axis.position};
    auto startup_array =
        ElmoGoldDeviceDescription::startup_sdos(configuration.axis.mode);
    std::vector<SdoDownloadRequest> startup(startup_array.begin(), startup_array.end());
    startup.insert(startup.end(), configuration.startup_parameters.begin(),
                   configuration.startup_parameters.end());
    const auto slave_configuration = EthercatSlaveConfiguration{
        slave,
        EthercatDeviceIdentity{configuration.axis.vendor_id,
                               configuration.axis.product_code,
                               configuration.axis.revision},
        ElmoGoldDeviceDescription::rx_pdo_definition(configuration.axis.mode),
        ElmoGoldDeviceDescription::feedback_pdos(),
        DistributedClockConfiguration{ElmoGoldDeviceDescription::assign_activate(),
                                      ElmoGoldDeviceDescription::sync0_cycle_ns(), 0},
        startup};
    auto configured = backend_->configure_slave(slave_configuration);
    if (!configured.has_value()) {
      backend_->deactivate();
      return configured;
    }

    auto& axis_handles = handles[axis_index];
    const auto bind_input = [&]<class T>(ObjectAddress address, std::uint8_t bits,
                                         TypedPdoField<T>& field) {
      auto location = backend_->bind_pdo_entry(slave, address, bits);
      if (!location.has_value()) {
        return Result<void>::failure(location.error());
      }
      return field.bind(location.value());
    };
    auto result =
        bind_input(ObjectAddress{0x6041U, 0U}, 16U, axis_handles.status_word);
    if (result.has_value()) {
      result = bind_input(ObjectAddress{0x6061U, 0U}, 8U,
                          axis_handles.mode_display);
    }
    if (result.has_value()) {
      result = bind_input(ObjectAddress{0x6064U, 0U}, 32U,
                          axis_handles.actual_position);
    }
    if (result.has_value()) {
      result = bind_input(ObjectAddress{0x606CU, 0U}, 32U,
                          axis_handles.actual_velocity);
    }
    if (result.has_value()) {
      result = bind_input(ObjectAddress{0x6077U, 0U}, 16U,
                          axis_handles.actual_torque);
    }
    if (result.has_value()) {
      result = bind_input(ObjectAddress{0x6040U, 0U}, 16U,
                          axis_handles.control_word);
    }
    if (!result.has_value()) {
      backend_->deactivate();
      return result;
    }

    Result<void> target = Result<void>::failure(
        {ErrorCode::internal, "unreachable static CiA 402 mode"});
    switch (configuration.axis.mode) {
      case profiles::Cia402Mode::csp:
        target = bind_input({0x607AU, 0U}, 32U, axis_handles.target_position);
        break;
      case profiles::Cia402Mode::csv:
        target = bind_input({0x60FFU, 0U}, 32U, axis_handles.target_velocity);
        break;
      case profiles::Cia402Mode::cst:
        target = bind_input({0x6071U, 0U}, 16U, axis_handles.target_torque);
        break;
    }
    if (!target.has_value()) {
      backend_->deactivate();
      return target;
    }
  }

  auto activated = backend_->activate();
  if (!activated.has_value()) {
    backend_->deactivate();
    return activated;
  }
  for (auto& mailbox : mailboxes_) {
    auto opened = mailbox->open();
    if (!opened.has_value()) {
      for (auto& rollback : mailboxes_) {
        rollback->close();
      }
      backend_->deactivate();
      return opened;
    }
  }

  pdo_handles_ = std::move(handles);
  std::fill(pdo_views_.begin(), pdo_views_.end(), Cia402PdoView{});
  open_ = true;
  health_.store(TransportHealth::healthy, std::memory_order_release);
  return Result<void>::success();
}

void EthercatMaster::close() noexcept {
  if (!open_) {
    return;
  }
  for (auto& mailbox : mailboxes_) {
    mailbox->close();
  }
  backend_->deactivate();
  open_ = false;
  health_.store(TransportHealth::failed, std::memory_order_release);
}

TransportHealth EthercatMaster::health() const noexcept {
  return health_.load(std::memory_order_acquire);
}

SchedulingClass EthercatMaster::scheduling_class() const noexcept {
  return SchedulingClass::hard_realtime_periodic;
}

void EthercatMaster::cycle(const CycleContext&) noexcept {
  if (!open_ || backend_ == nullptr) {
    health_.store(TransportHealth::failed, std::memory_order_release);
    return;
  }
  if (!backend_->try_acquire()) {
    health_.store(TransportHealth::degraded, std::memory_order_release);
    return;
  }

  backend_->receive();
  backend_->process_domain();
  auto image = backend_->process_image();
  bool valid_image = true;
  for (std::size_t axis_index = 0; axis_index < pdo_handles_.size(); ++axis_index) {
    const auto& handles = pdo_handles_[axis_index];
    auto& pdo = pdo_views_[axis_index];
    const auto status_word = handles.status_word.read(image);
    const auto mode_display = handles.mode_display.read(image);
    const auto actual_position = handles.actual_position.read(image);
    const auto actual_velocity = handles.actual_velocity.read(image);
    const auto actual_torque = handles.actual_torque.read(image);
    valid_image = valid_image && status_word.has_value() && mode_display.has_value() &&
                  actual_position.has_value() && actual_velocity.has_value() &&
                  actual_torque.has_value();
    if (status_word && mode_display && actual_position && actual_velocity && actual_torque) {
      pdo.status_word = *status_word;
      pdo.mode_display = *mode_display;
      pdo.actual_position = *actual_position;
      pdo.actual_velocity = *actual_velocity;
      pdo.actual_torque = *actual_torque;
    }
  }

  if (valid_image && cycle_handler_ != nullptr) {
    cycle_handler_(cycle_handler_context_, pdo_views_);
  }

  bool outputs_written = valid_image;
  for (std::size_t axis_index = 0; axis_index < pdo_handles_.size(); ++axis_index) {
    const auto& handles = pdo_handles_[axis_index];
    const auto& pdo = pdo_views_[axis_index];
    outputs_written = handles.control_word.write(image, pdo.control_word) && outputs_written;
    switch (axes_[axis_index].axis.mode) {
      case profiles::Cia402Mode::csp:
        outputs_written =
            handles.target_position.write(image, pdo.target_position) && outputs_written;
        break;
      case profiles::Cia402Mode::csv:
        outputs_written =
            handles.target_velocity.write(image, pdo.target_velocity) && outputs_written;
        break;
      case profiles::Cia402Mode::cst:
        outputs_written =
            handles.target_torque.write(image, pdo.target_torque) && outputs_written;
        break;
    }
  }

  const auto domain = backend_->domain_health();
  backend_->queue_domain();
  backend_->send();
  backend_->release();

  health_.store(valid_image && outputs_written && healthy_domain(domain)
                    ? TransportHealth::healthy
                    : TransportHealth::degraded,
                std::memory_order_release);
}

Result<void> EthercatMaster::register_field(CyclicField field, bool input) {
  if (open_ || field.size_bytes == 0U) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "cyclic fields must be registered before open"});
  }
  const auto contains = [field](const auto& fields) {
    return std::any_of(fields.begin(), fields.end(),
                       [field](const auto& candidate) { return candidate.id == field.id; });
  };
  if (contains(registered_inputs_) || contains(registered_outputs_)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "cyclic field ID is already registered"});
  }
  (input ? registered_inputs_ : registered_outputs_).push_back(field);
  return Result<void>::success();
}

Result<void> EthercatMaster::register_cyclic_input(CyclicField field) {
  return register_field(field, true);
}

Result<void> EthercatMaster::register_cyclic_output(CyclicField field) {
  return register_field(field, false);
}

Result<void> EthercatMaster::set_cycle_handler(CycleHandler handler, void* context) {
  if (open_ || handler == nullptr) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "cycle handler must be bound before open"});
  }
  cycle_handler_ = handler;
  cycle_handler_context_ = context;
  return Result<void>::success();
}

std::span<Cia402PdoView> EthercatMaster::pdo_views() noexcept { return pdo_views_; }

std::span<const Cia402PdoHandles> EthercatMaster::pdo_handles() const noexcept {
  return pdo_handles_;
}

ObjectDictionaryTransport& EthercatMaster::mailbox(std::size_t axis_index) {
  return *mailboxes_.at(axis_index);
}

}  // namespace policy_runtime
