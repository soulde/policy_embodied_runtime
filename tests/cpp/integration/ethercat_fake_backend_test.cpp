#include <atomic>
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
  std::uint16_t position{};
  std::uint16_t index{};
  std::uint8_t subindex{};

  bool operator==(const EntryKey&) const = default;
};

struct EntryKeyHash {
  std::size_t operator()(const EntryKey& key) const noexcept {
    return (static_cast<std::size_t>(key.position) << 24U) |
           (static_cast<std::size_t>(key.index) << 8U) | key.subindex;
  }
};

class FakeEthercatBackend final : public EthercatBackend {
 public:
  void clear_cycle_events() {
    std::scoped_lock lock(mutex_);
    events_.clear();
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
  bool activated() const noexcept { return activated_; }
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
    const EntryKey key{slave.position, address.index, address.subindex};
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
    activated_ = true;
    record("activate");
    return Result<void>::success();
  }

  void deactivate() noexcept override {
    activated_ = false;
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
    return DomainHealth{1U, 1U, true, true, true, 0};
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
  std::vector<std::string> events_;
  std::vector<std::byte> image_{256U};
  std::vector<ConfigSnapshot> configurations_;
  std::unordered_map<EntryKey, PdoFieldLocation, EntryKeyHash> locations_;
  std::size_t next_offset_{};
  bool activated_{};
  std::atomic<unsigned> receive_count_{};
  std::atomic<unsigned> download_count_{};
  std::atomic<unsigned> pending_downloads_{};
  std::atomic<unsigned> bind_count_{};
  std::atomic<unsigned> fail_bind_at_{};

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
};

void stage_axis(void* opaque, std::span<Cia402PdoView> axes) noexcept {
  auto& context = *static_cast<StageContext*>(opaque);
  context.backend->record("handler");
  context.saw_feedback = axes.size() == 1U && axes[0].status_word == 0x0027U &&
                         axes[0].mode_display == 8 && axes[0].actual_position == 12345 &&
                         axes[0].actual_velocity == -77 && axes[0].actual_torque == 22;
  axes[0].control_word = 0x000FU;
  axes[0].target_position = -654321;
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

TEST(EthercatFakeBackendTest, RunsExactCycleAroundTypedStaging) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  StageContext context{backend.get(), false};
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
            (std::vector<std::string>{"receive", "process", "image", "handler",
                                      "queue", "send"}));
  EXPECT_EQ(handles.control_word.read(backend->image()), 0x000FU);
  EXPECT_EQ(handles.target_position.read(backend->image()), -654321);
  EXPECT_EQ(master.health(), TransportHealth::healthy);
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
  auto request = mailbox.queue_download({0x6060U, 0U}, value);
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
  auto request = mailbox.queue_download({0x6060U, 0U}, value);
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

TEST(EthercatFakeBackendTest, PdoCycleNeverBlocksBehindMailboxTransfer) {
  auto backend = std::make_shared<FakeEthercatBackend>();
  EthercatMaster master{backend, {EthercatAxisConfiguration{axis_config(), {}}}};
  ASSERT_TRUE(master.open().has_value());
  auto& mailbox = master.mailbox(0U);
  const std::vector<std::byte> value{std::byte{8U}};
  auto request = mailbox.queue_download({0x6060U, 0U}, value);
  ASSERT_TRUE(request.has_value());
  backend->block_download();
  std::thread mailbox_thread([&] { mailbox.cycle({}); });
  ASSERT_TRUE(backend->wait_for_download(1s));

  const auto receives_before = backend->receive_count();
  const auto started = std::chrono::steady_clock::now();
  master.cycle({});
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(elapsed, 100ms);
  EXPECT_EQ(backend->receive_count(), receives_before);
  EXPECT_EQ(master.health(), TransportHealth::degraded);

  backend->release_download();
  mailbox_thread.join();
  EXPECT_EQ(mailbox.mailbox_status(request.value())->state,
            MailboxRequestState::completed);
}

}  // namespace
