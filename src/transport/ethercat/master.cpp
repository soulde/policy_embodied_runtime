#include "policy_runtime/transport/ethercat/master.hpp"

#include <algorithm>
#include <array>
#include <set>
#include <utility>

#include "policy_runtime/devices/cia402/ethercat_binding.hpp"
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

const char* sdo_failure_message(SdoFailureReason reason) noexcept {
  switch (reason) {
    case SdoFailureReason::none:
      return "EtherCAT SDO transfer failed";
    case SdoFailureReason::backend_exception:
      return "EtherCAT SDO backend raised an exception";
    case SdoFailureReason::endpoint_unavailable:
      return "EtherCAT SDO endpoint is unavailable";
    case SdoFailureReason::unsupported_download_size:
      return "EtherCAT SDO download size is unsupported";
    case SdoFailureReason::exact_size_request_unavailable:
      return "EtherCAT exact-size SDO request is unavailable";
    case SdoFailureReason::object_selection_failed:
      return "EtherCAT SDO object selection failed";
    case SdoFailureReason::download_schedule_failed:
      return "EtherCAT SDO download scheduling failed";
    case SdoFailureReason::conflicting_request:
      return "A different EtherCAT SDO request is active";
    case SdoFailureReason::invalid_upload_capacity:
      return "EtherCAT SDO upload capacity is invalid";
    case SdoFailureReason::upload_schedule_failed:
      return "EtherCAT SDO upload scheduling failed";
    case SdoFailureReason::upload_capacity_exceeded:
      return "EtherCAT SDO upload exceeded its capacity";
    case SdoFailureReason::transfer_failed:
      return "EtherCAT SDO transfer failed";
    case SdoFailureReason::unknown_request_state:
      return "EtherCAT SDO request entered an unknown state";
  }
  return "EtherCAT SDO transfer failed";
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
    Error error =
        request->second.failure_kind == FailureKind::backend
            ? Error{request->second.backend_error_code,
                    sdo_failure_message(
                        request->second.backend_failure_reason)}
            : Error{ErrorCode::unavailable,
                    "EtherCAT mailbox closed before completion"};
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
    progress = sdo_failed(ErrorCode::internal,
                          SdoFailureReason::backend_exception);
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
      request->backend_error_code = progress.error_code;
      request->backend_failure_reason = progress.failure_reason;
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
  if (admission_is_open()) {
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

  std::vector<Cia402PdoHandles> handles;
  handles.reserve(axes_.size());
  for (std::size_t axis_index = 0; axis_index < axes_.size(); ++axis_index) {
    const auto& configuration = axes_[axis_index];
    auto configured = Cia402EthercatBinding::configure_axis(*backend_, configuration);
    if (!configured.has_value()) {
      backend_->deactivate();
      return Result<void>::failure(configured.error());
    }
    handles.push_back(std::move(configured.value()));
  }

  for (auto& field : process_image_fields_) {
    auto location = backend_->bind_pdo_entry(
        field.slave, field.address, field.bit_length);
    if (!location.has_value()) {
      backend_->deactivate();
      return Result<void>::failure(location.error());
    }
    field.location = location.value();
    field.bound = true;
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
  cycle_admission_.store(kCycleOpenBit, std::memory_order_release);
  health_.store(TransportHealth::healthy, std::memory_order_release);
  return Result<void>::success();
}

void EthercatMaster::close() noexcept {
  std::scoped_lock lifecycle_lock(lifecycle_mutex_);
  // This RMW and admission's CAS share one atomic modification order. A CAS
  // ordered first contributes to the count below; a CAS ordered later observes
  // the cleared gate and cannot admit a cycle.
  const auto previous =
      cycle_admission_.fetch_and(kCycleCountMask, std::memory_order_acq_rel);
  if ((previous & kCycleOpenBit) == 0U) {
    return;
  }
  owner_lifecycle_->active_generation.store(0U, std::memory_order_release);
  for (auto& mailbox : mailboxes_) {
    mailbox->close_for_parent(generation_);
  }
  auto admission = cycle_admission_.load(std::memory_order_acquire);
  while ((admission & kCycleCountMask) != 0U) {
    cycle_admission_.wait(admission, std::memory_order_acquire);
    admission = cycle_admission_.load(std::memory_order_acquire);
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

bool EthercatMaster::admission_successor(std::uint64_t observed,
                                         std::uint64_t& desired) noexcept {
  const auto count = observed & kCycleCountMask;
  if ((observed & kCycleOpenBit) == 0U || count == kCycleCountMask) {
    return false;
  }
  desired = observed + 1U;
  return true;
}

bool EthercatMaster::admission_is_open() const noexcept {
  return (cycle_admission_.load(std::memory_order_acquire) & kCycleOpenBit) !=
         0U;
}

bool EthercatMaster::try_enter_cycle() noexcept {
  auto observed = cycle_admission_.load(std::memory_order_acquire);
  for (;;) {
    std::uint64_t desired{};
    if (!admission_successor(observed, desired)) {
      return false;
    }
    if (cycle_admission_.compare_exchange_weak(
            observed, desired, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      return true;
    }
  }
}

void EthercatMaster::leave_cycle() noexcept {
  const auto previous =
      cycle_admission_.fetch_sub(1U, std::memory_order_release);
  if ((previous & kCycleOpenBit) == 0U &&
      (previous & kCycleCountMask) == 1U) {
    cycle_admission_.notify_all();
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
  const bool supervised_input_available =
      supervised_cycle_handler_ != nullptr && domain.link_up;
  const bool input_available = domain_is_healthy || supervised_input_available;
  bool valid_image = input_available;
  if (input_available) {
    for (std::size_t axis_index = 0; axis_index < pdo_handles_.size(); ++axis_index) {
      const auto& handles = pdo_handles_[axis_index];
      auto& pdo = pdo_views_[axis_index];
      valid_image = Cia402EthercatBinding::read_inputs(handles, image, pdo) &&
                    valid_image;
    }
  }

  if (valid_image && cycle_handler_ != nullptr) {
    cycle_handler_(cycle_handler_context_, pdo_views_);
  }
  if (supervised_cycle_handler_ != nullptr) {
    supervised_cycle_handler_(cycle_handler_context_, pdo_views_, domain,
                              valid_image);
  }

  const bool supervised_outputs =
      supervised_cycle_handler_ != nullptr && valid_image;
  const bool outputs_enabled =
      (domain_is_healthy && valid_image) || supervised_outputs;

  bool outputs_written = true;
  for (std::size_t axis_index = 0; axis_index < pdo_handles_.size(); ++axis_index) {
    const auto& handles = pdo_handles_[axis_index];
    const auto& pdo = pdo_views_[axis_index];
    outputs_written = Cia402EthercatBinding::write_outputs(
                          handles, image, pdo, axes_[axis_index].axis.mode,
                          outputs_enabled) &&
                      outputs_written;
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
  if (admission_is_open() || field.size_bytes == 0U) {
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

Result<void> EthercatMaster::register_process_image_field(
    EthercatSlaveAddress slave, ObjectAddress address, PdoDirection direction,
    std::uint8_t bit_length, CyclicFieldId id) {
  if (admission_is_open() || id == 0U ||
      (bit_length != 8U && bit_length != 16U && bit_length != 32U)) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "process-image fields require a unique id and 8/16/32-bit width"});
  }
  const auto duplicate = std::any_of(
      process_image_fields_.begin(), process_image_fields_.end(),
      [id](const auto& field) { return field.id == id; });
  if (duplicate) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "process-image field id is already registered"});
  }
  process_image_fields_.push_back(
      EthercatProcessImageField{slave, address, direction, bit_length, id});
  return Result<void>::success();
}

std::optional<std::uint32_t> EthercatMaster::read_process_image_field(
    CyclicFieldId id) const noexcept {
  const auto found = std::find_if(
      process_image_fields_.begin(), process_image_fields_.end(),
      [id](const auto& field) { return field.id == id; });
  if (found == process_image_fields_.end() || !found->bound ||
      found->direction != PdoDirection::input) {
    return std::nullopt;
  }
  const auto image = backend_->process_image();
  if (found->location.byte_offset > image.size() ||
      found->bit_length / 8U > image.size() - found->location.byte_offset) {
    return std::nullopt;
  }
  std::uint32_t value{};
  for (std::size_t index = 0U; index < found->bit_length / 8U; ++index) {
    value |= static_cast<std::uint32_t>(std::to_integer<unsigned char>(
                  image[found->location.byte_offset + index]))
             << (index * 8U);
  }
  return value;
}

bool EthercatMaster::write_process_image_field(CyclicFieldId id,
                                                std::uint32_t value) noexcept {
  const auto found = std::find_if(
      process_image_fields_.begin(), process_image_fields_.end(),
      [id](const auto& field) { return field.id == id; });
  if (found == process_image_fields_.end() || !found->bound ||
      found->direction != PdoDirection::output) {
    return false;
  }
  auto image = backend_->process_image();
  if (found->location.byte_offset > image.size() ||
      found->bit_length / 8U > image.size() - found->location.byte_offset) {
    return false;
  }
  for (std::size_t index = 0U; index < found->bit_length / 8U; ++index) {
    image[found->location.byte_offset + index] = std::byte{
        static_cast<unsigned char>(value >> (index * 8U))};
  }
  return true;
}

std::size_t EthercatMaster::process_image_field_count() const noexcept {
  return process_image_fields_.size();
}

Result<void> EthercatMaster::set_cycle_handler(CycleHandler handler, void* context) {
  if (admission_is_open() || handler == nullptr ||
      supervised_cycle_handler_ != nullptr) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "cycle handler must be bound before open"});
  }
  cycle_handler_ = handler;
  cycle_handler_context_ = context;
  return Result<void>::success();
}

Result<void> EthercatMaster::set_supervised_cycle_handler(
    SupervisedCycleHandler handler, void* context) {
  if (admission_is_open() || handler == nullptr || cycle_handler_ != nullptr ||
      supervised_cycle_handler_ != nullptr) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "supervised cycle handler must be exclusively bound before open"});
  }
  supervised_cycle_handler_ = handler;
  cycle_handler_context_ = context;
  return Result<void>::success();
}

Result<void> EthercatMaster::clear_supervised_cycle_handler(void* context) {
  if (admission_is_open() || supervised_cycle_handler_ == nullptr ||
      cycle_handler_context_ != context) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "supervised cycle handler can only be cleared by its inactive owner"});
  }
  supervised_cycle_handler_ = nullptr;
  cycle_handler_context_ = nullptr;
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
