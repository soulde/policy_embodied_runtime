#include "policy_runtime/transport/ethercat/master.hpp"

#include <algorithm>
#include <array>
#include <set>
#include <utility>

#include "policy_runtime/transport/ethercat/elmo_gold.hpp"

namespace policy_runtime {
namespace {

constexpr std::size_t kMaximumAxes = 12U;
constexpr std::size_t kMaximumSdoUploadBytes = 4096U;

bool static_mode_object(ObjectAddress address) noexcept {
  return address.index == 0x6060U || address.index == 0x60C2U;
}

bool healthy_domain(const DomainHealth& health) noexcept {
  const bool expected_matches = !health.expected_working_counter.has_value() ||
                                health.working_counter == *health.expected_working_counter;
  return health.link_up && health.working_counter_complete &&
         health.all_slaves_operational && expected_matches;
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
                                 EthercatSlaveAddress slave,
                                 std::shared_ptr<OwnerLifecycle> owner_lifecycle)
    : backend_(std::move(backend)),
      owner_lifecycle_(std::move(owner_lifecycle)),
      slave_(slave) {}

Result<void> EthercatMailbox::open() {
  std::scoped_lock lifecycle_lock(lifecycle_mutex_);
  if (backend_ == nullptr || owner_lifecycle_ == nullptr) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "EtherCAT mailbox has no backend"});
  }
  const auto parent_generation =
      owner_lifecycle_->active_generation.load(std::memory_order_acquire);
  if (parent_generation == 0U) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "EtherCAT mailbox parent is closed"});
  }
  if (open_generation_.load(std::memory_order_acquire) != 0U) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "EtherCAT mailbox is already open"});
  }
  if (cancel_requested_generation_.load(std::memory_order_acquire) != 0U ||
      staged_request_.load(std::memory_order_acquire) != nullptr) {
    return Result<void>::failure(
        {ErrorCode::unavailable,
         "EtherCAT mailbox cancellation is awaiting an owner cycle"});
  }
  if (owner_lifecycle_->active_generation.load(std::memory_order_acquire) !=
      parent_generation) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "EtherCAT mailbox parent generation changed"});
  }
  open_generation_.store(parent_generation, std::memory_order_release);
  health_.store(TransportHealth::healthy, std::memory_order_release);
  return Result<void>::success();
}

void EthercatMailbox::close() noexcept {
  std::scoped_lock lifecycle_lock(lifecycle_mutex_);
  const auto generation = open_generation_.exchange(0U, std::memory_order_acq_rel);
  if (generation != 0U) {
    close_generation(generation, true);
  }
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
  const auto generation = open_generation_.load(std::memory_order_acquire);
  if (generation == 0U || owner_lifecycle_ == nullptr ||
      owner_lifecycle_->active_generation.load(std::memory_order_acquire) !=
          generation) {
    return Result<MailboxRequestId>::failure(
        {ErrorCode::unavailable, "EtherCAT mailbox is closed"});
  }
  if (address.index == 0U || (!upload && data.empty())) {
    return Result<MailboxRequestId>::failure(
        {ErrorCode::invalid_argument, "invalid EtherCAT mailbox request"});
  }
  if (!upload && static_mode_object(address)) {
    return Result<MailboxRequestId>::failure(
        {ErrorCode::invalid_argument,
         "runtime writes to static CiA 402 mode objects are forbidden"});
  }
  std::scoped_lock lock(requests_mutex_);
  if (open_generation_.load(std::memory_order_acquire) != generation ||
      owner_lifecycle_->active_generation.load(std::memory_order_acquire) !=
          generation) {
    return Result<MailboxRequestId>::failure(
        {ErrorCode::unavailable, "EtherCAT mailbox is closed"});
  }
  const auto request_id = next_request_id_++;
  auto [request, inserted] = requests_.try_emplace(request_id);
  if (!inserted) {
    return Result<MailboxRequestId>::failure(
        {ErrorCode::internal, "EtherCAT mailbox request ID collision"});
  }
  try {
    request->second.generation = generation;
    request->second.upload = upload;
    request->second.transfer.address = address;
    if (upload) {
      request->second.transfer.data.resize(kMaximumSdoUploadBytes);
    } else {
      request->second.transfer.data.assign(data.begin(), data.end());
    }
  } catch (...) {
    requests_.erase(request);
    throw;
  }
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
  const auto phase = request->second.phase.load(std::memory_order_acquire);
  if (phase == RequestPhase::completed) {
    const auto size = std::min(request->second.uploaded_size,
                               request->second.transfer.data.size());
    return MailboxRequestStatus{
        MailboxRequestState::completed, std::nullopt,
        request->second.upload
            ? std::vector<std::byte>(request->second.transfer.data.begin(),
                                     request->second.transfer.data.begin() + size)
            : std::vector<std::byte>{}};
  }
  if (phase == RequestPhase::failed ||
      (phase == RequestPhase::terminalizing &&
       open_generation_.load(std::memory_order_acquire) !=
           request->second.generation)) {
    Error error = request->second.failure_kind == FailureKind::backend
                      ? Error{ErrorCode::io, "EtherCAT SDO transfer failed"}
                      : Error{ErrorCode::unavailable,
                              "EtherCAT mailbox closed before completion"};
    if (request->second.failure_kind == FailureKind::backend &&
        request->second.backend_error.has_value()) {
      error = *request->second.backend_error;
    }
    return MailboxRequestStatus{MailboxRequestState::failed, std::move(error), {}};
  }
  return MailboxRequestStatus{MailboxRequestState::queued, std::nullopt, {}};
}

void EthercatMailbox::cycle(const CycleContext&) noexcept {
  const auto generation = open_generation_.load(std::memory_order_acquire);
  if (generation == 0U || owner_lifecycle_ == nullptr ||
      owner_lifecycle_->active_generation.load(std::memory_order_acquire) !=
          generation) {
    health_.store(TransportHealth::failed, std::memory_order_release);
    return;
  }
  try {
    stage_one();
  } catch (...) {
    health_.store(TransportHealth::degraded, std::memory_order_release);
  }
}

void EthercatMailbox::stage_one() {
  if (staged_request_.load(std::memory_order_acquire) != nullptr) {
    return;
  }
  std::scoped_lock lock(requests_mutex_);
  const auto generation = open_generation_.load(std::memory_order_acquire);
  if (generation == 0U || owner_lifecycle_ == nullptr ||
      owner_lifecycle_->active_generation.load(std::memory_order_acquire) !=
          generation ||
      staged_request_.load(std::memory_order_acquire) != nullptr) {
    return;
  }
  const auto candidate = std::find_if(
      requests_.begin(), requests_.end(), [generation](const auto& entry) {
        return entry.second.generation == generation &&
               entry.second.phase.load(std::memory_order_acquire) ==
                   RequestPhase::queued;
      });
  if (candidate == requests_.end()) {
    return;
  }
  auto expected_phase = RequestPhase::queued;
  if (!candidate->second.phase.compare_exchange_strong(
          expected_phase, RequestPhase::staged, std::memory_order_acq_rel)) {
    return;
  }
  Request* expected_request = nullptr;
  if (!staged_request_.compare_exchange_strong(
          expected_request, &candidate->second, std::memory_order_release,
          std::memory_order_relaxed)) {
    expected_phase = RequestPhase::staged;
    static_cast<void>(candidate->second.phase.compare_exchange_strong(
        expected_phase, RequestPhase::queued, std::memory_order_release,
        std::memory_order_relaxed));
  }
}

void EthercatMailbox::fail_request(Request& request, FailureKind kind) noexcept {
  auto phase = request.phase.load(std::memory_order_acquire);
  if (phase == RequestPhase::completed || phase == RequestPhase::failed) {
    return;
  }
  for (;;) {
    if (phase == RequestPhase::terminalizing) {
      request.phase.wait(RequestPhase::terminalizing, std::memory_order_acquire);
      phase = request.phase.load(std::memory_order_acquire);
      if (phase == RequestPhase::failed) {
        return;
      }
    }
    if (phase == RequestPhase::failed) {
      return;
    }
    if (request.phase.compare_exchange_weak(
            phase, RequestPhase::terminalizing, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      request.failure_kind = kind;
      request.phase.store(RequestPhase::failed, std::memory_order_release);
      request.phase.notify_all();
      return;
    }
  }
}

void EthercatMailbox::close_generation(std::uint64_t generation,
                                       bool request_cancel) noexcept {
  {
    std::scoped_lock lock(requests_mutex_);
    for (auto& [request_id, request] : requests_) {
      static_cast<void>(request_id);
      if (request.generation == generation) {
        fail_request(request, FailureKind::closed);
      }
    }
  }
  if (request_cancel) {
    cancel_requested_generation_.store(generation, std::memory_order_release);
  }
}

Result<void> EthercatMailbox::open_for_parent(std::uint64_t parent_generation) {
  std::scoped_lock lifecycle_lock(lifecycle_mutex_);
  if (parent_generation == 0U ||
      open_generation_.load(std::memory_order_acquire) != 0U ||
      cancel_requested_generation_.load(std::memory_order_acquire) != 0U ||
      staged_request_.load(std::memory_order_acquire) != nullptr) {
    return Result<void>::failure(
        {ErrorCode::unavailable, "EtherCAT mailbox has stale lifecycle state"});
  }
  open_generation_.store(parent_generation, std::memory_order_release);
  health_.store(TransportHealth::healthy, std::memory_order_release);
  return Result<void>::success();
}

void EthercatMailbox::close_for_parent(std::uint64_t parent_generation) noexcept {
  std::scoped_lock lifecycle_lock(lifecycle_mutex_);
  if (open_generation_.load(std::memory_order_acquire) == parent_generation) {
    open_generation_.store(0U, std::memory_order_release);
  }
  close_generation(parent_generation, false);
  health_.store(TransportHealth::failed, std::memory_order_release);
}

void EthercatMailbox::reset_after_parent_deactivate() noexcept {
  std::scoped_lock lifecycle_lock(lifecycle_mutex_);
  staged_request_.store(nullptr, std::memory_order_release);
  cancel_requested_generation_.store(0U, std::memory_order_release);
  open_generation_.store(0U, std::memory_order_release);
  health_.store(TransportHealth::failed, std::memory_order_release);
}

bool EthercatMailbox::owner_step(std::uint64_t parent_generation) noexcept {
  const auto cancel_generation =
      cancel_requested_generation_.load(std::memory_order_acquire);
  if (cancel_generation != 0U) {
    SdoTransferProgress cancellation;
    try {
      cancellation = backend_->progress_cancel_sdo(slave_);
    } catch (...) {
      health_.store(TransportHealth::degraded, std::memory_order_release);
      return true;
    }
    if (cancellation.state == SdoTransferState::completed) {
      staged_request_.store(nullptr, std::memory_order_release);
      cancel_requested_generation_.store(0U, std::memory_order_release);
    }
    if (cancellation.state == SdoTransferState::failed) {
      health_.store(TransportHealth::failed, std::memory_order_release);
    }
    return true;
  }

  auto* request = staged_request_.load(std::memory_order_acquire);
  if (request == nullptr) {
    return false;
  }
  if (request->generation != parent_generation || owner_lifecycle_ == nullptr ||
      owner_lifecycle_->active_generation.load(std::memory_order_acquire) !=
          parent_generation ||
      open_generation_.load(std::memory_order_acquire) != parent_generation) {
    return false;
  }

  auto phase = request->phase.load(std::memory_order_acquire);
  if (phase == RequestPhase::staged) {
    if (!request->phase.compare_exchange_strong(
            phase, RequestPhase::active, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      return false;
    }
    phase = RequestPhase::active;
  }
  if (phase != RequestPhase::active) {
    return false;
  }

  SdoTransferProgress progress;
  try {
    if (request->upload) {
      progress = backend_->progress_upload_sdo(
          slave_, request->transfer.address, request->transfer.data);
    } else {
      progress = backend_->progress_download_sdo(slave_, request->transfer);
    }
  } catch (...) {
    progress.state = SdoTransferState::failed;
    progress.error.reset();
    progress.uploaded_size = 0U;
  }
  if (progress.state == SdoTransferState::pending) {
    return true;
  }

  auto expected_phase = RequestPhase::active;
  if (!request->phase.compare_exchange_strong(
          expected_phase, RequestPhase::terminalizing, std::memory_order_acq_rel,
          std::memory_order_acquire)) {
    return true;
  }
  const bool still_current =
      owner_lifecycle_->active_generation.load(std::memory_order_acquire) ==
          parent_generation &&
      open_generation_.load(std::memory_order_acquire) == parent_generation;
  if (progress.state == SdoTransferState::completed && still_current &&
      (!request->upload || progress.uploaded_size <= request->transfer.data.size())) {
    request->uploaded_size = progress.uploaded_size;
    request->phase.store(RequestPhase::completed, std::memory_order_release);
    health_.store(TransportHealth::healthy, std::memory_order_release);
  } else {
    request->failure_kind = still_current ? FailureKind::backend : FailureKind::closed;
    if (still_current) {
      request->backend_error = std::move(progress.error);
    }
    request->phase.store(RequestPhase::failed, std::memory_order_release);
    health_.store(still_current ? TransportHealth::degraded
                                : TransportHealth::failed,
                  std::memory_order_release);
  }
  request->phase.notify_all();
  Request* expected_request = request;
  static_cast<void>(staged_request_.compare_exchange_strong(
      expected_request, nullptr, std::memory_order_release,
      std::memory_order_relaxed));
  return true;
}

EthercatMaster::EthercatMaster(std::shared_ptr<EthercatBackend> backend,
                               std::vector<EthercatAxisConfiguration> axes)
    : backend_(std::move(backend)),
      owner_lifecycle_(std::make_shared<EthercatMailbox::OwnerLifecycle>()),
      axes_(std::move(axes)),
      pdo_views_(axes_.size()) {
  mailboxes_.reserve(axes_.size());
  for (const auto& configuration : axes_) {
    mailboxes_.push_back(std::unique_ptr<EthercatMailbox>(new EthercatMailbox(
        backend_, EthercatSlaveAddress{configuration.axis.alias,
                                       configuration.axis.position},
        owner_lifecycle_)));
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
      if (parameter.data.empty() || static_mode_object(parameter.address)) {
        return Result<void>::failure(
            {ErrorCode::invalid_argument,
             "startup parameter is empty or overrides the static mode/cycle time"});
      }
    }
  }
  return Result<void>::success();
}

Result<void> EthercatMaster::open() {
  std::scoped_lock lifecycle_lock(lifecycle_mutex_);
  if (open_.load(std::memory_order_acquire)) {
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
  auto parent_generation = next_generation_++;
  if (parent_generation == 0U) {
    parent_generation = next_generation_++;
  }
  std::size_t opened_mailboxes{};
  for (auto& mailbox : mailboxes_) {
    auto opened = mailbox->open_for_parent(parent_generation);
    if (!opened.has_value()) {
      for (std::size_t index = 0U; index < opened_mailboxes; ++index) {
        mailboxes_[index]->close_for_parent(parent_generation);
      }
      backend_->deactivate();
      for (auto& rollback : mailboxes_) {
        rollback->reset_after_parent_deactivate();
      }
      return opened;
    }
    ++opened_mailboxes;
  }

  pdo_handles_ = std::move(handles);
  std::fill(pdo_views_.begin(), pdo_views_.end(), Cia402PdoView{});
  generation_ = parent_generation;
  mailbox_cursor_ = 0U;
  owner_lifecycle_->active_generation.store(parent_generation,
                                             std::memory_order_release);
  open_.store(true, std::memory_order_release);
  health_.store(TransportHealth::healthy, std::memory_order_release);
  return Result<void>::success();
}

void EthercatMaster::close() noexcept {
  std::scoped_lock lifecycle_lock(lifecycle_mutex_);
  if (!open_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  owner_lifecycle_->active_generation.store(0U, std::memory_order_release);
  for (auto& mailbox : mailboxes_) {
    mailbox->close_for_parent(generation_);
  }
  auto active = cycles_in_flight_.load(std::memory_order_acquire);
  while (active != 0U) {
    cycles_in_flight_.wait(active, std::memory_order_acquire);
    active = cycles_in_flight_.load(std::memory_order_acquire);
  }
  backend_->deactivate();
  for (auto& mailbox : mailboxes_) {
    mailbox->reset_after_parent_deactivate();
  }
  generation_ = 0U;
  health_.store(TransportHealth::failed, std::memory_order_release);
}

TransportHealth EthercatMaster::health() const noexcept {
  return health_.load(std::memory_order_acquire);
}

SchedulingClass EthercatMaster::scheduling_class() const noexcept {
  return SchedulingClass::hard_realtime_periodic;
}

bool EthercatMaster::try_enter_cycle() noexcept {
  cycles_in_flight_.fetch_add(1U, std::memory_order_acq_rel);
  if (open_.load(std::memory_order_acquire)) {
    return true;
  }
  leave_cycle();
  return false;
}

void EthercatMaster::leave_cycle() noexcept {
  if (cycles_in_flight_.fetch_sub(1U, std::memory_order_acq_rel) == 1U) {
    cycles_in_flight_.notify_all();
  }
}

void EthercatMaster::cycle(const CycleContext&) noexcept {
  if (!try_enter_cycle()) {
    health_.store(TransportHealth::failed, std::memory_order_release);
    return;
  }
  if (backend_ == nullptr) {
    health_.store(TransportHealth::failed, std::memory_order_release);
    leave_cycle();
    return;
  }
  backend_->receive();
  backend_->process_domain();
  const auto domain = backend_->domain_health();
  const bool domain_is_healthy = healthy_domain(domain);
  auto image = backend_->process_image();
  bool valid_image = domain_is_healthy;
  if (domain_is_healthy) {
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
  }

  if (valid_image && cycle_handler_ != nullptr) {
    cycle_handler_(cycle_handler_context_, pdo_views_);
  }

  bool outputs_written = true;
  for (std::size_t axis_index = 0; axis_index < pdo_handles_.size(); ++axis_index) {
    const auto& handles = pdo_handles_[axis_index];
    const auto& pdo = pdo_views_[axis_index];
    const auto control_word = domain_is_healthy && valid_image ? pdo.control_word : 0U;
    outputs_written = handles.control_word.write(image, control_word) && outputs_written;
    switch (axes_[axis_index].axis.mode) {
      case profiles::Cia402Mode::csp:
        outputs_written = handles.target_position.write(
                              image, domain_is_healthy && valid_image
                                         ? pdo.target_position
                                         : std::int32_t{0}) &&
                          outputs_written;
        break;
      case profiles::Cia402Mode::csv:
        outputs_written = handles.target_velocity.write(
                              image, domain_is_healthy && valid_image
                                         ? pdo.target_velocity
                                         : std::int32_t{0}) &&
                          outputs_written;
        break;
      case profiles::Cia402Mode::cst:
        outputs_written = handles.target_torque.write(
                              image, domain_is_healthy && valid_image
                                         ? pdo.target_torque
                                         : std::int16_t{0}) &&
                          outputs_written;
        break;
    }
  }

  if (!mailboxes_.empty()) {
    for (std::size_t offset = 0U; offset < mailboxes_.size(); ++offset) {
      const auto mailbox_index = (mailbox_cursor_ + offset) % mailboxes_.size();
      if (mailboxes_[mailbox_index]->owner_step(generation_)) {
        mailbox_cursor_ = (mailbox_index + 1U) % mailboxes_.size();
        break;
      }
    }
  }

  backend_->queue_domain();
  backend_->send();

  health_.store(domain_is_healthy && valid_image && outputs_written
                    ? TransportHealth::healthy
                    : TransportHealth::degraded,
                std::memory_order_release);
  leave_cycle();
}

Result<void> EthercatMaster::register_field(CyclicField field, bool input) {
  if (open_.load(std::memory_order_acquire) || field.size_bytes == 0U) {
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
  if (open_.load(std::memory_order_acquire) || handler == nullptr) {
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
