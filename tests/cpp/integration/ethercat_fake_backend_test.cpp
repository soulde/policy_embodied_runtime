#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "policy_runtime/transport/ethercat/elmo_gold.hpp"
#include "policy_runtime/transport/ethercat/master.hpp"

namespace {

using namespace std::chrono_literals;
using policy_runtime::Cia402PdoView;
using policy_runtime::CycleContext;
using policy_runtime::DistributedClockConfiguration;
using policy_runtime::DomainHealth;
using policy_runtime::ElmoGoldDeviceDescription;
using policy_runtime::Error;
using policy_runtime::ErrorCode;
using policy_runtime::EthercatAxisConfiguration;
using policy_runtime::EthercatBackend;
using policy_runtime::EthercatDeviceIdentity;
using policy_runtime::EthercatMaster;
using policy_runtime::EthercatSlaveAddress;
using policy_runtime::EthercatSlaveConfiguration;
using policy_runtime::MailboxRequestState;
using policy_runtime::ObjectAddress;
using policy_runtime::PdoFieldLocation;
using policy_runtime::Result;
using policy_runtime::SdoDownloadRequest;
using policy_runtime::SdoTransferProgress;
using policy_runtime::SdoTransferState;
using policy_runtime::TransportHealth;
using policy_runtime::profiles::AxisConfig;
using policy_runtime::profiles::Cia402Mode;

struct ConfigSnapshot {
  EthercatSlaveAddress address{};
  EthercatDeviceIdentity identity{};
  std::uint16_t rx_pdo{};
  std::vector<std::uint16_t> tx_pdos;
  DistributedClockConfiguration dc{};
  std::vector<SdoDownloadRequest> startup_sdos;
};

struct EntryKey {
  std::uint16_t alias{};
  std::uint16_t position{};
  std::uint16_t index{};
  std::uint8_t subindex{};

  bool operator==(const EntryKey&) const = default;
};

struct EntryKeyHash {
  std::size_t operator()(const EntryKey& key) const noexcept {
    return (static_cast<std::size_t>(key.alias) << 40U) |
           (static_cast<std::size_t>(key.position) << 24U) |
           (static_cast<std::size_t>(key.index) << 8U) | key.subindex;
  }
};

class FakeEthercatBackend final : public EthercatBackend {
 public:
  void clear_cycle_events() {
    std::scoped_lock lock(mutex_);
    events_.clear();
  }

  void set_domain_health(DomainHealth health) {
    std::scoped_lock lock(mutex_);
    domain_health_ = health;
  }

  void record(std::string event) {
    std::scoped_lock lock(mutex_);
    events_.push_back(std::move(event));
  }

  std::vector<std::string> events() const {
    std::scoped_lock lock(mutex_);
    return events_;
  }

  std::span<std::byte> image() { return image_; }
  const std::vector<ConfigSnapshot>& configurations() const { return configurations_; }
  bool activated() const noexcept { return activated_.load(); }
  unsigned deactivate_count() const noexcept { return deactivate_count_.load(); }
  unsigned receive_count() const noexcept { return receive_count_.load(); }
  unsigned download_count() const noexcept { return download_count_.load(); }
  unsigned bind_count() const noexcept { return bind_count_.load(); }

  void make_download_pending_once() { pending_downloads_.store(1U); }
  void fail_bind_at(unsigned call) { fail_bind_at_.store(call); }

  void block_download() {
    std::scoped_lock lock(block_mutex_);
    block_download_ = true;
    download_entered_ = false;
    release_download_ = false;
  }

  bool wait_for_download(std::chrono::milliseconds timeout) {
    std::unique_lock lock(block_mutex_);
    return block_cv_.wait_for(lock, timeout, [this] { return download_entered_; });
  }

  void release_download() {
    {
      std::scoped_lock lock(block_mutex_);
      release_download_ = true;
    }
    block_cv_.notify_all();
  }

 protected:
  Result<void> initialize() override {
    locations_.clear();
    next_offset_ = 0U;
    record("initialize");
    return Result<void>::success();
  }

  Result<void> configure_slave(const EthercatSlaveConfiguration& configuration) override {
    ConfigSnapshot snapshot{};
    snapshot.address = configuration.address;
    snapshot.identity = configuration.identity;
    snapshot.rx_pdo = configuration.rx_pdo.index;
    for (const auto& pdo : configuration.tx_pdos) {
      snapshot.tx_pdos.push_back(pdo.index);
    }
    snapshot.dc = configuration.dc;
    snapshot.startup_sdos.assign(configuration.startup_sdos.begin(),
                                 configuration.startup_sdos.end());
    configurations_.push_back(std::move(snapshot));
    record("configure");
    return Result<void>::success();
  }

  Result<PdoFieldLocation> bind_pdo_entry(EthercatSlaveAddress slave,
                                           ObjectAddress address,
                                           std::uint8_t bit_length) override {
    const auto call = bind_count_.fetch_add(1U) + 1U;
    record("bind");
    if (call == fail_bind_at_.load()) {
      return Result<PdoFieldLocation>::failure(
          {ErrorCode::protocol, "requested fake PDO bind failure"});
    }
    const EntryKey key{slave.alias, slave.position, address.index, address.subindex};
    if (locations_.contains(key)) {
      return Result<PdoFieldLocation>::failure(
          {ErrorCode::invalid_argument, "duplicate fake PDO registration"});
    }
    const PdoFieldLocation location{next_offset_, 0U, bit_length};
    locations_.emplace(key, location);
    next_offset_ += (bit_length + 7U) / 8U;
    return Result<PdoFieldLocation>::success(location);
  }

  Result<void> activate() override {
    activated_.store(true);
    record("activate");
    return Result<void>::success();
  }

  void deactivate() noexcept override {
    activated_.store(false);
    deactivate_count_.fetch_add(1U);
    record("deactivate");
  }

  void receive() noexcept override {
    receive_count_.fetch_add(1U);
    record("receive");
  }

  void process_domain() noexcept override { record("process"); }

  std::span<std::byte> process_image() noexcept override {
    record("image");
    return image_;
  }

  void queue_domain() noexcept override { record("queue"); }
  void send() noexcept override { record("send"); }

  DomainHealth domain_health() const noexcept override {
    std::scoped_lock lock(mutex_);
    events_.push_back("health");
    return domain_health_;
  }

  SdoTransferProgress progress_download_sdo(
      EthercatSlaveAddress, const SdoDownloadRequest&) override {
    download_count_.fetch_add(1U);
    record("download");
    std::unique_lock lock(block_mutex_);
    if (block_download_) {
      download_entered_ = true;
      block_cv_.notify_all();
      block_cv_.wait(lock, [this] { return release_download_; });
      block_download_ = false;
    }
    if (pending_downloads_.load() != 0U) {
      pending_downloads_.fetch_sub(1U);
      return SdoTransferProgress{SdoTransferState::pending, std::nullopt, {}};
    }
    return SdoTransferProgress{SdoTransferState::completed, std::nullopt, {}};
  }

  SdoTransferProgress progress_upload_sdo(EthercatSlaveAddress, ObjectAddress,
                                           std::size_t) override {
    record("upload");
    return SdoTransferProgress{SdoTransferState::completed, std::nullopt,
                               {std::byte{0x12U}, std::byte{0x34U}}};
  }

 private:
  mutable std::mutex mutex_;
  mutable std::vector<std::string> events_;
  std::vector<std::byte> image_{256U};
  std::vector<ConfigSnapshot> configurations_;
  std::unordered_map<EntryKey, PdoFieldLocation, EntryKeyHash> locations_;
  std::size_t next_offset_{};
  std::atomic<bool> activated_{};
  std::atomic<unsigned> deactivate_count_{};
  std::atomic<unsigned> receive_count_{};
  std::atomic<unsigned> download_count_{};
  std::atomic<unsigned> pending_downloads_{};
  std::atomic<unsigned> bind_count_{};
  std::atomic<unsigned> fail_bind_at_{};
  DomainHealth domain_health_{1U, 1U, true, true, true, 0};

  std::mutex block_mutex_;
  std::condition_variable block_cv_;
  bool block_download_{};
  bool download_entered_{};
  bool release_download_{};
};

AxisConfig axis_config(Cia402Mode mode = Cia402Mode::csp) {
  return AxisConfig{"axis", 0U, 0U, ElmoGoldDeviceDescription::vendor_id(),
                    ElmoGoldDeviceDescription::product_code(),
                    ElmoGoldDeviceDescription::revision(), mode, 1000.0, -1.0, 1.0,
                    100ms, "arm", 0.1, 0.5};
}

struct StageContext {
  FakeEthercatBackend* backend{};
  bool saw_feedback{};
  unsigned calls{};
};

void stage_axis(void* opaque, std::span<Cia402PdoView> axes) noexcept {
  auto& context = *static_cast<StageContext*>(opaque);
  context.backend->record("handler");
  ++context.calls;
  context.saw_feedback = axes.size() == 1U && axes[0].status_word == 0x0027U &&
                         axes[0].mode_display == 8 && axes[0].actual_position == 12345 &&
                         axes[0].actual_velocity == -77 && axes[0].actual_torque == 22;
  axes[0].control_word = 0x000FU;
  axes[0].target_position = -654321;
}

void stage_two_axes(void* opaque, std::span<Cia402PdoView> axes) noexcept {
  auto& calls = *static_cast<unsigned*>(opaque);
  ++calls;
  if (axes.size() != 2U) {
    return;
  }
  axes[0].control_word = 0x0006U;
  axes[0].target_position = axes[0].actual_position + 100;
  axes[1].control_word = 0x000FU;
  axes[1].target_position = axes[1].actual_position - 200;
}

TEST(EthercatFakeBackendTest, ConfiguresAndBindsBeforeActivation) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};

  ASSERT_TRUE(master.open().has_value());
  ASSERT_TRUE(backend->activated());
  ASSERT_EQ(backend->configurations().size(), 1U);
  const auto& configured = backend->configurations()[0];
  EXPECT_EQ(configured.identity.vendor_id, 0x0000009AU);
  EXPECT_EQ(configured.identity.product_code, 0x00030924U);
  EXPECT_EQ(configured.identity.revision, 0x00010420U);
  EXPECT_EQ(configured.rx_pdo, 0x1600U);
  EXPECT_EQ(configured.tx_pdos, (std::vector<std::uint16_t>{0x1A02U, 0x1A11U}));
  EXPECT_EQ(configured.dc.assign_activate, 0x0300U);
  EXPECT_EQ(configured.dc.sync0_cycle_ns, 1'000'000U);
  ASSERT_EQ(configured.startup_sdos.size(), 2U);
  EXPECT_EQ(configured.startup_sdos[0].address.index, 0x6060U);
  EXPECT_EQ(configured.startup_sdos[0].data,
            (std::vector<std::byte>{std::byte{8U}}));

  const auto events = backend->events();
  const auto activate = std::find(events.begin(), events.end(), "activate");
  ASSERT_NE(activate, events.end());
  EXPECT_EQ(std::count(events.begin(), activate, "bind"), 7);
  EXPECT_EQ(master.pdo_handles().size(), 1U);
  EXPECT_TRUE(master.pdo_handles()[0].status_word.is_bound());
  EXPECT_TRUE(master.pdo_handles()[0].target_position.is_bound());
  EXPECT_FALSE(master.pdo_handles()[0].target_velocity.is_bound());
  EXPECT_FALSE(master.pdo_handles()[0].target_torque.is_bound());
}

TEST(EthercatFakeBackendTest, StagesTwoAliasesWithTheSameRelativePositionIndependently) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  auto first = axis_config();
  first.name = "axis_1";
  first.alias = 1U;
  auto second = axis_config();
  second.name = "axis_2";
  second.alias = 2U;
  unsigned handler_calls{};
  EthercatMaster master{backend,
                        {EthercatAxisConfiguration{first, {}},
                         EthercatAxisConfiguration{second, {}}}};
  ASSERT_TRUE(master.set_cycle_handler(&stage_two_axes, &handler_calls).has_value());

  ASSERT_TRUE(master.open().has_value());
  ASSERT_EQ(backend->configurations().size(), 2U);
  EXPECT_EQ(backend->configurations()[0].address.alias, 1U);
  EXPECT_EQ(backend->configurations()[1].address.alias, 2U);
  EXPECT_EQ(backend->bind_count(), 14U);
  ASSERT_EQ(master.pdo_handles().size(), 2U);
  ASSERT_TRUE(master.pdo_handles()[0].actual_position.write(backend->image(), 1000));
  ASSERT_TRUE(master.pdo_handles()[1].actual_position.write(backend->image(), 5000));

  master.cycle({});

  EXPECT_EQ(handler_calls, 1U);
  EXPECT_EQ(master.pdo_handles()[0].control_word.read(backend->image()), 0x0006U);
  EXPECT_EQ(master.pdo_handles()[0].target_position.read(backend->image()), 1100);
  EXPECT_EQ(master.pdo_handles()[1].control_word.read(backend->image()), 0x000FU);
  EXPECT_EQ(master.pdo_handles()[1].target_position.read(backend->image()), 4800);
}

TEST(EthercatFakeBackendTest, RunsExactCycleAroundTypedStaging) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  StageContext context{backend.get(), false, 0U};
  ASSERT_TRUE(master.set_cycle_handler(&stage_axis, &context).has_value());
  ASSERT_TRUE(master.open().has_value());

  const auto& handles = master.pdo_handles()[0];
  ASSERT_TRUE(handles.status_word.write(backend->image(), 0x0027U));
  ASSERT_TRUE(handles.mode_display.write(backend->image(), 8));
  ASSERT_TRUE(handles.actual_position.write(backend->image(), 12345));
  ASSERT_TRUE(handles.actual_velocity.write(backend->image(), -77));
  ASSERT_TRUE(handles.actual_torque.write(backend->image(), 22));
  backend->clear_cycle_events();

  master.cycle(CycleContext{1U, std::chrono::steady_clock::now(), 1ms});

  EXPECT_TRUE(context.saw_feedback);
  EXPECT_EQ(backend->events(),
            (std::vector<std::string>{"receive", "process", "health", "image", "handler",
                                      "queue", "send"}));
  EXPECT_EQ(handles.control_word.read(backend->image()), 0x000FU);
  EXPECT_EQ(handles.target_position.read(backend->image()), -654321);
  EXPECT_EQ(master.health(), TransportHealth::healthy);
}

TEST(EthercatFakeBackendTest, RejectsUnhealthyInputBeforeHandlerAndSendsSafeOutputs) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  StageContext context{backend.get(), false, 0U};
  ASSERT_TRUE(master.set_cycle_handler(&stage_axis, &context).has_value());
  ASSERT_TRUE(master.open().has_value());
  auto& pdo = master.pdo_views()[0];
  pdo.control_word = 0x000FU;
  pdo.target_position = 123456;
  backend->set_domain_health(DomainHealth{0U, 1U, false, true, false, 0});
  backend->clear_cycle_events();

  master.cycle({});

  EXPECT_EQ(context.calls, 0U);
  const auto events = backend->events();
  EXPECT_EQ(events,
            (std::vector<std::string>{"receive", "process", "health", "image", "queue",
                                      "send"}));
  const auto& handles = master.pdo_handles()[0];
  EXPECT_EQ(handles.control_word.read(backend->image()), 0U);
  EXPECT_EQ(handles.target_position.read(backend->image()), 0);
  EXPECT_EQ(master.health(), TransportHealth::degraded);
}

TEST(EthercatFakeBackendTest, StopsAtFirstPdoBindingFailureBeforeActivation) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  backend->fail_bind_at(1U);
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};

  const auto opened = master.open();

  ASSERT_FALSE(opened.has_value());
  EXPECT_EQ(opened.error().code, ErrorCode::protocol);
  EXPECT_EQ(backend->bind_count(), 1U);
  EXPECT_FALSE(backend->activated());
  const auto events = backend->events();
  EXPECT_EQ(std::count(events.begin(), events.end(), "activate"), 0);
  EXPECT_EQ(std::count(events.begin(), events.end(), "deactivate"), 1);
}

TEST(EthercatFakeBackendTest, RejectsUnknownStaticModeBeforeBackendInitialization) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  auto invalid = axis_config();
  invalid.mode = static_cast<Cia402Mode>(127);
  EthercatMaster master{backend, {EthercatAxisConfiguration{invalid, {}}}};

  const auto opened = master.open();

  ASSERT_FALSE(opened.has_value());
  EXPECT_EQ(opened.error().code, ErrorCode::invalid_argument);
  EXPECT_TRUE(backend->events().empty());
}

struct BlockingStageContext {
  std::mutex mutex;
  std::condition_variable cv;
  bool entered{};
  bool release{};
};

void block_in_pdo_window(void* opaque, std::span<Cia402PdoView>) noexcept {
  auto& context = *static_cast<BlockingStageContext*>(opaque);
  std::unique_lock lock(context.mutex);
  context.entered = true;
  context.cv.notify_all();
  context.cv.wait(lock, [&context] { return context.release; });
}

TEST(EthercatFakeBackendTest, KeepsMailboxOutsidePdoCriticalWindow) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  BlockingStageContext stage;
  ASSERT_TRUE(master.set_cycle_handler(&block_in_pdo_window, &stage).has_value());
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);
  const std::vector<std::byte> value{std::byte{8U}};
  auto request = mailbox.queue_download({0x2000U, 0U}, value);
  ASSERT_TRUE(request.has_value());

  std::thread pdo_thread([&] { master.cycle({}); });
  {
    std::unique_lock lock(stage.mutex);
    ASSERT_TRUE(stage.cv.wait_for(lock, 1s, [&stage] { return stage.entered; }));
  }
  mailbox.cycle({});
  EXPECT_EQ(backend->download_count(), 0U);
  ASSERT_TRUE(mailbox.mailbox_status(request.value()).has_value());
  EXPECT_EQ(mailbox.mailbox_status(request.value())->state, MailboxRequestState::queued);

  {
    std::scoped_lock lock(stage.mutex);
    stage.release = true;
  }
  stage.cv.notify_all();
  pdo_thread.join();

  mailbox.cycle({});
  EXPECT_EQ(backend->download_count(), 1U);
  EXPECT_EQ(mailbox.mailbox_status(request.value())->state,
            MailboxRequestState::completed);
}

TEST(EthercatFakeBackendTest, AdvancesAsynchronousSdoAcrossUnifiedMasterCycles) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);
  const std::vector<std::byte> value{std::byte{8U}};
  auto request = mailbox.queue_download({0x2000U, 0U}, value);
  ASSERT_TRUE(request.has_value());
  backend->make_download_pending_once();

  mailbox.cycle({});
  EXPECT_EQ(mailbox.mailbox_status(request.value())->state, MailboxRequestState::queued);
  EXPECT_EQ(backend->download_count(), 1U);

  master.cycle({});
  mailbox.cycle({});
  EXPECT_EQ(mailbox.mailbox_status(request.value())->state,
            MailboxRequestState::completed);
  EXPECT_EQ(backend->download_count(), 2U);
}

TEST(EthercatFakeBackendTest, PdoCycleIsNotDiscardedByAnInFlightMailboxTransfer) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);
  const std::vector<std::byte> value{std::byte{8U}};
  auto request = mailbox.queue_download({0x2000U, 0U}, value);
  ASSERT_TRUE(request.has_value());
  backend->block_download();
  std::thread mailbox_thread([&] { mailbox.cycle({}); });
  ASSERT_TRUE(backend->wait_for_download(1s));

  const auto receives_before = backend->receive_count();
  std::thread pdo_thread([&] { master.cycle({}); });
  std::this_thread::sleep_for(10ms);
  EXPECT_EQ(backend->receive_count(), receives_before);

  backend->release_download();
  mailbox_thread.join();
  pdo_thread.join();
  EXPECT_EQ(backend->receive_count(), receives_before + 1U);
  EXPECT_EQ(master.health(), TransportHealth::healthy);
  EXPECT_EQ(mailbox.mailbox_status(request.value())->state,
            MailboxRequestState::completed);
}

TEST(EthercatFakeBackendTest, PdoWaiterRunsBeforeAnotherMailboxRequest) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);
  const std::vector<std::byte> first_value{std::byte{1U}};
  const std::vector<std::byte> second_value{std::byte{2U}};
  auto first = mailbox.queue_download({0x2000U, 0U}, first_value);
  auto second = mailbox.queue_download({0x2001U, 0U}, second_value);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  backend->block_download();
  std::thread first_mailbox([&] { mailbox.cycle({}); });
  ASSERT_TRUE(backend->wait_for_download(1s));

  backend->clear_cycle_events();
  std::thread pdo_thread([&] { master.cycle({}); });
  std::this_thread::sleep_for(10ms);
  std::thread competing_mailbox([&] { mailbox.cycle({}); });
  competing_mailbox.join();
  backend->release_download();
  first_mailbox.join();
  pdo_thread.join();

  mailbox.cycle({});
  const auto events = backend->events();
  const auto receive = std::find(events.begin(), events.end(), "receive");
  const auto second_download = std::find(events.begin(), events.end(), "download");
  ASSERT_NE(receive, events.end());
  ASSERT_NE(second_download, events.end());
  EXPECT_LT(receive, second_download);
  EXPECT_EQ(mailbox.mailbox_status(second.value())->state,
            MailboxRequestState::completed);
}

TEST(EthercatFakeBackendTest, RejectsRuntimeWritesToEveryStaticModeSubindex) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);
  const std::vector<std::byte> value{std::byte{8U}};
  constexpr std::array<ObjectAddress, 4U> protected_objects{
      ObjectAddress{0x6060U, 0U}, ObjectAddress{0x6060U, 3U},
      ObjectAddress{0x60C2U, 1U}, ObjectAddress{0x60C2U, 2U}};

  for (const auto address : protected_objects) {
    auto queued = mailbox.queue_download(address, value);
    ASSERT_FALSE(queued.has_value());
    EXPECT_EQ(queued.error().code, ErrorCode::invalid_argument);
  }
  EXPECT_EQ(backend->download_count(), 0U);
}

TEST(EthercatFakeBackendTest, RejectsUserStartupOverridesForStaticObjectFamilies) {
  const std::vector<std::byte> value{std::byte{8U}};
  constexpr std::array<ObjectAddress, 4U> protected_objects{
      ObjectAddress{0x6060U, 0U}, ObjectAddress{0x6060U, 3U},
      ObjectAddress{0x60C2U, 1U}, ObjectAddress{0x60C2U, 2U}};

  for (const auto address : protected_objects) {
    auto backend = std::make_shared<FakeEthercatBackend>();
    EthercatMaster master{
        backend,
        {EthercatAxisConfiguration{axis_config(), {SdoDownloadRequest{address, value}}}}};
    auto opened = master.open();
    ASSERT_FALSE(opened.has_value());
    EXPECT_EQ(opened.error().code, ErrorCode::invalid_argument);
    EXPECT_TRUE(backend->events().empty());
  }
}

TEST(EthercatFakeBackendTest, CloseWaitsForInFlightPdoBeforeDeactivation) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  BlockingStageContext stage;
  ASSERT_TRUE(master.set_cycle_handler(&block_in_pdo_window, &stage).has_value());
  ASSERT_TRUE(master.open().has_value());

  std::thread pdo_thread([&] { master.cycle({}); });
  {
    std::unique_lock lock(stage.mutex);
    ASSERT_TRUE(stage.cv.wait_for(lock, 1s, [&stage] { return stage.entered; }));
  }
  std::atomic<bool> close_completed{};
  std::thread close_thread([&] {
    master.close();
    close_completed.store(true);
  });
  std::this_thread::sleep_for(10ms);
  EXPECT_FALSE(close_completed.load());
  EXPECT_TRUE(backend->activated());
  EXPECT_EQ(backend->deactivate_count(), 0U);

  {
    std::scoped_lock lock(stage.mutex);
    stage.release = true;
  }
  stage.cv.notify_all();
  pdo_thread.join();
  close_thread.join();
  EXPECT_TRUE(close_completed.load());
  EXPECT_FALSE(backend->activated());
  EXPECT_EQ(backend->deactivate_count(), 1U);
}

TEST(EthercatFakeBackendTest, CloseFailsMailboxWorkAndReopenHasNoStaleRequest) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);
  const std::vector<std::byte> value{std::byte{1U}};
  auto active = mailbox.queue_download({0x2000U, 0U}, value);
  auto queued = mailbox.queue_download({0x2001U, 0U}, value);
  ASSERT_TRUE(active.has_value());
  ASSERT_TRUE(queued.has_value());
  backend->block_download();
  std::thread mailbox_thread([&] { mailbox.cycle({}); });
  ASSERT_TRUE(backend->wait_for_download(1s));

  std::atomic<bool> close_completed{};
  std::thread close_thread([&] {
    master.close();
    close_completed.store(true);
  });
  std::this_thread::sleep_for(10ms);
  EXPECT_FALSE(close_completed.load());
  EXPECT_TRUE(backend->activated());

  backend->release_download();
  mailbox_thread.join();
  close_thread.join();
  ASSERT_TRUE(mailbox.mailbox_status(active.value()).has_value());
  ASSERT_TRUE(mailbox.mailbox_status(queued.value()).has_value());
  EXPECT_EQ(mailbox.mailbox_status(active.value())->state, MailboxRequestState::failed);
  EXPECT_EQ(mailbox.mailbox_status(queued.value())->state, MailboxRequestState::failed);

  const auto downloads_before_reopen = backend->download_count();
  ASSERT_TRUE(master.open().has_value());
  master.mailbox(0U).cycle({});
  EXPECT_EQ(backend->download_count(), downloads_before_reopen);
  EXPECT_EQ(mailbox.mailbox_status(active.value())->state, MailboxRequestState::failed);
  EXPECT_EQ(mailbox.mailbox_status(queued.value())->state, MailboxRequestState::failed);

  auto fresh = mailbox.queue_download({0x2002U, 0U}, value);
  ASSERT_TRUE(fresh.has_value());
  mailbox.cycle({});
  EXPECT_EQ(mailbox.mailbox_status(fresh.value())->state,
            MailboxRequestState::completed);
  EXPECT_EQ(backend->download_count(), downloads_before_reopen + 1U);
}

}  // namespace
