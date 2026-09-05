#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <type_traits>
#include <vector>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/robot_io/ipc_protocol.hpp"
#include "policy_runtime/robot_io/local_snapshot.hpp"
#include "policy_runtime/robot_io/snapshot.hpp"
#include "policy_runtime/robot_io/transport_scheduler.hpp"
#include "policy_runtime/transport/frame_transport.hpp"

namespace policy_runtime {

inline constexpr std::uint32_t kSt3215FeedbackValid = 1U << 0U;
inline constexpr std::uint32_t kSt3215FeedbackStale = 1U << 1U;
inline constexpr std::uint32_t kSt3215FeedbackTimeout = 1U << 2U;
inline constexpr std::uint32_t kSt3215FeedbackIo = 1U << 3U;
inline constexpr std::uint32_t kSt3215FeedbackProtocol = 1U << 4U;
inline constexpr std::uint32_t kSt3215FeedbackDeviceError = 1U << 5U;
inline constexpr std::uint32_t kSt3215FeedbackDisabled = 1U << 6U;

struct St3215ServoConfig {
  std::string name;
  std::uint8_t device_id{};
  std::uint16_t servo_id{};
  std::uint16_t max_position_units{4095U};
  std::uint16_t speed_units{};
  std::uint16_t time_units{};
  std::chrono::milliseconds feedback_timeout{250};
  std::chrono::nanoseconds command_timeout{};
  std::chrono::nanoseconds maximum_command_future{};
};

static_assert(std::is_trivially_copyable_v<St3215ServoCommand>);
static_assert(std::is_trivially_copyable_v<St3215ServoFeedback>);
static_assert(sizeof(St3215ServoCommand) % sizeof(std::uint64_t) == 0U);
static_assert(sizeof(St3215ServoFeedback) % sizeof(std::uint64_t) == 0U);

enum class St3215CommandAcceptance : std::uint8_t {
  accepted,
  duplicate,
  rejected,
};

class St3215Servo {
 public:
  explicit St3215Servo(St3215ServoConfig config);

  St3215CommandAcceptance stage_command(
      const St3215ServoCommand& command) noexcept;
  St3215ServoFeedback feedback(std::int64_t now_ns = 0) const noexcept;
  const St3215ServoConfig& config() const noexcept;
  bool atomics_are_lock_free() const noexcept;

 private:
  struct StagedCommand {
    std::uint64_t publication{};
    St3215ServoCommand command{};
  };

  bool read_staged(std::uint64_t consumed_publication,
                   StagedCommand& output) const noexcept;
  void publish_feedback(const St3215ServoFeedback& feedback) noexcept;
  bool try_lock_command() noexcept;
  void unlock_command() noexcept;
  St3215CommandAcceptance inspect_command(
      const St3215ServoCommand& command, std::int64_t now_ns) const noexcept;
  void commit_command(const St3215ServoCommand& command) noexcept;

  St3215ServoConfig config_;
  LocalSnapshot<StagedCommand> commands_;
  LocalSnapshot<St3215ServoFeedback> feedback_;
  std::atomic_flag command_gate_ = ATOMIC_FLAG_INIT;
  std::uint64_t next_publication_{};
  std::uint64_t last_command_sequence_{};
  std::int64_t last_command_timestamp_ns_{};
  St3215ServoCommand last_command_{};
  bool has_command_{};

  friend class St3215Bus;
  friend class St3215DeviceRegistry;
};

struct St3215BusOptions {
  std::chrono::nanoseconds service_period{std::chrono::milliseconds{1}};
};

// Hard-realtime shared ST3215 serial bus with the same one-shot contract as
// the CAN paths: the owner's realtime loop calls cycle() once per period.
// Each cycle performs at most one bounded receive (the response to the
// previous cycle's request) and stages exactly one write, then performs one
// transport cycle. Write errors, malformed frames, and device-ID mismatches
// latch a permanent fault with no retry or recovery; a response that stays
// absent past the servo feedback timeout also latches. Recovery is process
// restart.
class St3215Bus {
 public:
  St3215Bus(std::shared_ptr<FrameTransport> transport,
            St3215BusOptions options = {});
  ~St3215Bus();

  St3215Bus(const St3215Bus&) = delete;
  St3215Bus& operator=(const St3215Bus&) = delete;

  Result<void> add_servo(std::shared_ptr<St3215Servo> servo);
  Result<void> start(TransportScheduler& scheduler);
  void stop(TransportScheduler& scheduler) noexcept;
  void stop() noexcept;

  // One realtime service step. Must be called from a single thread while
  // running(); performs no allocation, blocking, or retry.
  void cycle(const CycleContext& context) noexcept;

  bool running() const noexcept;
  bool fault_latched() const noexcept { return fault_latched_; }
  std::size_t servo_count() const noexcept;
  TransportHealth health() const noexcept;
  std::shared_ptr<FrameTransport> transport() const noexcept;

 private:
  struct ServoRuntime {
    std::shared_ptr<St3215Servo> servo;
    std::uint64_t consumed_publication{};
    std::uint64_t feedback_sequence{};
    std::uint64_t goal_sequence{};
    bool goal_dispatched{};
    std::int64_t last_response_ns{};
    St3215ServoCommand command{};
    bool has_command{};
    bool command_timed_out{};
  };

  void refresh_staged(ServoRuntime& runtime, std::int64_t now_ns) noexcept;
  // Non-noexcept: transport I/O may throw; cycle() contains the fault latch.
  void poll_response(ServoRuntime& runtime, std::int64_t now_ns);
  void dispatch_request(ServoRuntime& runtime, std::int64_t now_ns);
  Result<St3215ServoFeedback> consume_response(ServoRuntime& runtime,
                                               std::int64_t now_ns);
  Result<void> send_request(ServoRuntime& runtime);
  St3215ServoFeedback failed_feedback(ServoRuntime& runtime,
                                      std::uint32_t flags,
                                      std::int64_t now_ns) noexcept;

  std::shared_ptr<FrameTransport> transport_;
  St3215BusOptions options_;
  mutable std::mutex lifecycle_mutex_;
  std::vector<ServoRuntime> servos_;
  TransportScheduler* scheduler_{};
  std::atomic<bool> running_{};
  std::atomic<TransportHealth> health_{TransportHealth::failed};
  std::uint64_t cycle_sequence_{};
  std::size_t cursor_{};
  std::size_t pending_index_{};
  bool dispatched_{false};
  std::int64_t started_ns_{};
  bool fault_latched_{false};
};

inline constexpr std::size_t kMaximumSt3215Servos = 32U;

struct St3215RegistrySafetySnapshot {
  std::uint32_t servo_count{};
  std::uint32_t fault_count{};
  std::uint32_t fault_mask{};
  std::uint32_t aggregate_flags{};
  std::array<std::uint32_t, kMaximumSt3215Servos> flags{};
};

class St3215DeviceRegistry {
 public:
  St3215DeviceRegistry() = default;
  ~St3215DeviceRegistry();

  St3215DeviceRegistry(const St3215DeviceRegistry&) = delete;
  St3215DeviceRegistry& operator=(const St3215DeviceRegistry&) = delete;

  Result<void> configure(
      std::span<const profiles::St3215ServoProfile> profiles);
  Result<void> start(TransportScheduler& scheduler);
  void stop(TransportScheduler& scheduler) noexcept;
  void stop() noexcept;

  // One realtime step across every bus; called from the daemon cycle thread.
  void cycle(const CycleContext& context) noexcept;

  St3215CommandAcceptance stage_command(
      std::size_t servo_index,
      const St3215ServoCommand& command) noexcept;
  St3215CommandAcceptance stage_commands(
      const Snapshot<St3215ServoCommand>& snapshot,
      std::int64_t now_ns) noexcept;
  St3215ServoFeedback feedback(std::size_t servo_index,
                               std::int64_t now_ns = 0) const noexcept;
  St3215RegistrySafetySnapshot safety_snapshot(
      std::int64_t now_ns) const noexcept;
  St3215Servo* servo(std::size_t servo_index) const noexcept;
  std::size_t servo_count() const noexcept;
  std::size_t bus_count() const noexcept;
  bool configured() const noexcept;
  bool running() const noexcept;
  bool atomics_are_lock_free() const noexcept;

 private:
  struct BusEntry {
    profiles::SerialPortConfig serial;
    std::unique_ptr<St3215Bus> bus;
  };

  std::vector<BusEntry> buses_;
  std::vector<std::shared_ptr<St3215Servo>> servos_;
  std::vector<std::string> safety_groups_;
  TransportScheduler* scheduler_{};
  bool configured_{};
  std::atomic<bool> running_{};
};

}  // namespace policy_runtime
