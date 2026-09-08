# C++ Robot I/O Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Python runtime with a C++20 policy host and robot I/O daemon, then add IgH EtherCAT and Elmo Gold CiA 402 CSP/CSV/CST control without changing the public ZMQ/JSON contract.

**Architecture:** `policy-runtime-host` owns policy RPC and inference; `robot-io-daemon` owns device transports, protocols, scheduling, and safety. A Unix socket carries control messages, while daemon-created `memfd` double-slot snapshots carry commands and feedback. EtherCAT is a transport, CiA 402 is a protocol, and each axis is a shared Sensor/Actuator device.

**Tech Stack:** C++20, CMake 3.24+, GoogleTest, nlohmann/json, ZeroMQ/cppzmq, Linux Unix sockets/memfd, IgH `libethercat`, PREEMPT_RT for production 1 kHz operation.

**Spec:** `docs/superpowers/specs/2026-08-23-cpp-robot-io-runtime-design.md`

## Global Constraints

- Support ARM64 and x86_64 Linux; require PREEMPT_RT only for production 1 kHz guarantees.
- Configure 0–12 axes at startup; freeze topology and all real-time memory after activation.
- Configure each axis as CSP, CSV, or CST; reject runtime mode changes.
- Preserve `embodied-policy-runtime/v1alpha1`, existing ZMQ/JSON envelopes, robot profiles, and policy profiles.
- Keep JSON, ZMQ, logging, allocation, exceptions, file I/O, and blocking locks out of real-time cycles.
- Keep the simulator independent; do not migrate it into either runtime process.
- Use short imperative commit subjects and run `git diff --check` before every commit.

## File Map

```text
CMakeLists.txt                         root build and feature options
cmake/FindEtherCAT.cmake               IgH header/library discovery
include/policy_runtime/common/         Result, errors, time, IDs
include/policy_runtime/protocol/rpc/   compatible JSON envelope types
include/policy_runtime/profiles/       robot/policy configuration types
include/policy_runtime/transport/      lifecycle and scheduling interfaces
include/policy_runtime/protocol/cia402 CiA 402 state and PDO codecs
include/policy_runtime/robot_io/       snapshots, IPC, daemon, scheduler, safety
include/policy_runtime/devices/  CiA402Axis and later ST3215 devices
include/policy_runtime/runtime/        policy host and RobotIoClient
src/                                   one implementation file per interface
apps/                                  two executable entry points
tests/cpp/unit/                        hardware-free unit tests
tests/cpp/integration/                 process and compatibility tests
tests/golden/                          stable JSON/profile fixtures
```

---

### Task 1: C++ Build Skeleton and Error Model

**Files:**
- Create: `CMakeLists.txt`
- Create: `cmake/ProjectOptions.cmake`
- Create: `include/policy_runtime/common/result.hpp`
- Create: `src/common/result.cpp`
- Create: `tests/cpp/unit/result_test.cpp`

**Interfaces:**
- Produces: `ErrorCode`, `Error`, and `template<class T> Result<T>` used by every later task.

- [ ] **Step 1: Write the failing smoke test**

```cpp
#include <gtest/gtest.h>
#include "policy_runtime/common/result.hpp"

TEST(ResultTest, CarriesTypedError) {
  auto result = policy_runtime::Result<int>::failure(
      {policy_runtime::ErrorCode::invalid_argument, "bad value"});
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, policy_runtime::ErrorCode::invalid_argument);
}
```

- [ ] **Step 2: Add CMake targets and verify the test fails to compile**

Create `policy_runtime_core`, `policy_runtime_unit_tests`, enable C++20 and warnings, then run:

```bash
cmake -S . -B build -DPOLICY_RUNTIME_BUILD_TESTS=ON
cmake --build build
```

Expected: compilation fails because `result.hpp` has no declarations.

- [ ] **Step 3: Implement the minimal error API**

```cpp
enum class ErrorCode { invalid_argument, io, timeout, protocol, unavailable, internal };
struct Error { ErrorCode code; std::string message; };

template<class T>
class Result {
 public:
  static Result success(T value);
  static Result failure(Error error);
  bool has_value() const noexcept;
  T& value();
  const Error& error() const;
 private:
  std::variant<T, Error> storage_;
};
```

- [ ] **Step 4: Build and run the unit test**

```bash
cmake --build build
ctest --test-dir build -R result_test --output-on-failure
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt cmake/ include/policy_runtime/common src/common tests/cpp/unit/result_test.cpp
git commit -m "Add C++ runtime build skeleton"
```

### Task 2: Compatible RPC Types and JSON Codec

**Files:**
- Create: `include/policy_runtime/protocol/rpc/messages.hpp`
- Create: `include/policy_runtime/protocol/rpc/codec.hpp`
- Create: `src/protocol/rpc/codec.cpp`
- Create: `tests/golden/observation_request.json`
- Create: `tests/cpp/unit/rpc_codec_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `Result<T>` from Task 1.
- Produces: `MessageEnvelope`, `ObservationPayload`, `ActionPayload`, `decode_envelope(std::string_view)`, and `encode_envelope(const MessageEnvelope&)`.

- [ ] **Step 1: Copy one canonical fixture from the Python tests and write a failing round-trip test**

```cpp
TEST(RpcCodecTest, PreservesV1Alpha1Envelope) {
  const auto text = read_fixture("tests/golden/observation_request.json");
  auto decoded = decode_envelope(text);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded.value().schema, "embodied-policy-runtime/v1alpha1");
  EXPECT_EQ(decoded.value().type, "observation_request");
  EXPECT_EQ(nlohmann::json::parse(encode_envelope(decoded.value())),
            nlohmann::json::parse(text));
}
```

- [ ] **Step 2: Run the test and confirm missing codec symbols**

```bash
cmake --build build
ctest --test-dir build -R rpc_codec_test --output-on-failure
```

Expected: FAIL to compile with `decode_envelope` undefined.

- [ ] **Step 3: Implement strict envelope validation**

```cpp
struct MessageEnvelope {
  std::string schema;
  std::string type;
  std::string request_id;
  std::string session_id;
  std::uint64_t step_id{};
  std::int64_t timestamp_ns{};
  nlohmann::json payload = nlohmann::json::object();
  std::optional<ErrorPayload> error;
};

Result<MessageEnvelope> decode_envelope(std::string_view text);
std::string encode_envelope(const MessageEnvelope& envelope);
```

Reject unknown top-level keys, empty IDs/types, negative timestamps, mismatched joint vector/name lengths, and empty action payloads using the same messages asserted by `policy_embodied_runtime/tests/test_messages.py`.

- [ ] **Step 4: Add malformed-envelope cases and run both implementations**

```bash
ctest --test-dir build -R rpc_codec_test --output-on-failure
pytest policy_embodied_runtime/tests/test_messages.py -q
```

Expected: both suites PASS.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt include/policy_runtime/protocol src/protocol tests/golden tests/cpp/unit/rpc_codec_test.cpp
git commit -m "Port RPC message validation to C++"
```

### Task 3: Compatible Robot and Policy Profiles

**Files:**
- Create: `include/policy_runtime/profiles/robot_profile.hpp`
- Create: `include/policy_runtime/profiles/policy_profile.hpp`
- Create: `include/policy_runtime/profiles/loader.hpp`
- Create: `src/profiles/loader.cpp`
- Create: `tests/cpp/unit/profile_loader_test.cpp`
- Create: `tests/golden/elmo_robot_profile.json`

**Interfaces:**
- Produces: `DeviceLink`, `DeviceConfig`, `RobotProfile`, `PolicyProfile`, `load_robot_profile(path)`, and `load_policy_profile(path)`.

- [ ] **Step 1: Write failing compatibility and CiA 402 validation tests**

```cpp
TEST(ProfileLoaderTest, LoadsExistingRobotProfile) {
  auto profile = load_robot_profile(
      "policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json");
  ASSERT_TRUE(profile.has_value());
  EXPECT_FALSE(profile.value().sensors.empty());
}

TEST(ProfileLoaderTest, ParsesStaticCia402Mode) {
  auto profile = load_robot_profile("tests/golden/elmo_robot_profile.json");
  ASSERT_TRUE(profile.has_value());
  EXPECT_EQ(profile.value().axes.at(0).mode, Cia402Mode::csp);
  EXPECT_EQ(profile.value().axes.size(), 2U);
}
```

- [ ] **Step 2: Run and verify the loader is missing**

```bash
cmake --build build
ctest --test-dir build -R profile_loader_test --output-on-failure
```

- [ ] **Step 3: Implement parsing and exact validation rules**

```cpp
enum class Cia402Mode : std::int8_t { csp = 8, csv = 9, cst = 10 };
struct AxisConfig {
  std::string name;
  std::uint16_t alias;
  std::uint16_t position;
  std::uint32_t vendor_id;
  std::uint32_t product_code;
  std::uint32_t revision;
  Cia402Mode mode;
  double scale;
  double minimum;
  double maximum;
  std::chrono::milliseconds command_timeout;
  std::string safety_group;
};
```

Accept existing string-valued `args`, parse decimal/`0x` integers, reject duplicate names/links, reject more than 12 axes, and require mode-specific scale/limits.

- [ ] **Step 4: Run C++ and Python profile suites**

```bash
ctest --test-dir build -R profile_loader_test --output-on-failure
pytest policy_embodied_runtime/tests/test_robot_profile.py policy_embodied_runtime/tests/test_policy_profile.py -q
```

- [ ] **Step 5: Commit**

```bash
git add include/policy_runtime/profiles src/profiles tests/cpp/unit/profile_loader_test.cpp tests/golden/elmo_robot_profile.json
git commit -m "Port runtime profile loading to C++"
```

### Task 4: Transport Capabilities and Scheduling

**Files:**
- Create: `include/policy_runtime/transport/transport.hpp`
- Create: `include/policy_runtime/transport/frame_transport.hpp`
- Create: `include/policy_runtime/transport/cyclic_transport.hpp`
- Create: `include/policy_runtime/transport/object_dictionary_transport.hpp`
- Create: `include/policy_runtime/robot_io/daemon/transport_scheduler.hpp`
- Create: `src/robot_io/daemon/transport_scheduler.cpp`
- Create: `tests/cpp/unit/transport_scheduler_test.cpp`

**Interfaces:**
- Produces: `Transport`, `FrameTransport`, `CyclicTransport`, `ObjectDictionaryTransport`, `SchedulingClass`, and `TransportScheduler`.

- [ ] **Step 1: Write a failing scheduler-isolation test**

```cpp
TEST(TransportSchedulerTest, SeparatesRealtimeAndBlockingTransports) {
  FakeTransport ethercat{SchedulingClass::hard_realtime_periodic};
  FakeTransport serial{SchedulingClass::blocking_event_driven};
  TransportScheduler scheduler;
  ASSERT_TRUE(scheduler.add(ethercat).has_value());
  ASSERT_TRUE(scheduler.add(serial).has_value());
  EXPECT_NE(scheduler.executor_id(ethercat), scheduler.executor_id(serial));
}
```

- [ ] **Step 2: Run and verify failure**

```bash
cmake --build build
ctest --test-dir build -R transport_scheduler_test --output-on-failure
```

- [ ] **Step 3: Implement capability interfaces**

```cpp
class Transport {
 public:
  virtual ~Transport() = default;
  virtual Result<void> open() = 0;
  virtual void close() noexcept = 0;
  virtual TransportHealth health() const noexcept = 0;
  virtual SchedulingClass scheduling_class() const noexcept = 0;
  virtual void cycle(const CycleContext&) noexcept = 0;
};

class FrameTransport : public virtual Transport {
 public:
  virtual Result<void> write(ChannelId, std::span<const std::byte>) = 0;
  virtual Result<std::size_t> read(ChannelId, std::span<std::byte>) = 0;
};
```

Define cyclic field registration and OD upload/download on the other two interfaces. `write()` stages data; only `cycle()` performs physical I/O.

- [ ] **Step 4: Run scheduler tests under ThreadSanitizer preset**

```bash
cmake --preset tsan
cmake --build --preset tsan
ctest --preset tsan -R transport_scheduler_test --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt CMakePresets.json include/policy_runtime/transport include/policy_runtime/robot_io src/robot_io tests/cpp/unit/transport_scheduler_test.cpp
git commit -m "Define scheduled transport capabilities"
```

### Task 5: Versioned Unix Socket and memfd IPC

**Files:**
- Create: `include/policy_runtime/robot_io/snapshot.hpp`
- Create: `include/policy_runtime/robot_io/ipc_protocol.hpp`
- Create: `include/policy_runtime/robot_io/ipc_server.hpp`
- Create: `include/policy_runtime/runtime/robot_io_client.hpp`
- Create: `src/robot_io/ipc_server.cpp`
- Create: `src/runtime/robot_io_client.cpp`
- Create: `tests/cpp/integration/ipc_test.cpp`

**Interfaces:**
- Produces: `AxisCommand`, `AxisFeedback`, `BusHealth`, `SnapshotRegion<T>`, `RobotIoIpcServer`, and `RobotIoClient`.

- [ ] **Step 1: Write failing snapshot and restart tests**

```cpp
TEST(IpcTest, PublishesOnlyCompleteSnapshots) {
  SnapshotRegion<AxisCommand> region = make_test_region(2);
  region.publish({command(1.0), command(2.0)}, 7, 1000);
  auto snapshot = region.read_latest();
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->sequence, 7U);
  EXPECT_DOUBLE_EQ(snapshot->axes[1].target, 2.0);
}

TEST(IpcTest, RejectsStaleDaemonGeneration) {
  EXPECT_EQ(validate_header(header(1), expected_generation(2)).error().code,
            ErrorCode::unavailable);
}
```

- [ ] **Step 2: Run and verify missing IPC types**

```bash
cmake --build build
ctest --test-dir build -R ipc_test --output-on-failure
```

- [ ] **Step 3: Implement the fixed ABI**

```cpp
struct alignas(64) IpcHeader {
  std::array<char, 8> magic{'R','O','B','O','T','I','O','1'};
  std::uint32_t abi_version{1};
  std::uint32_t generation{};
  std::uint32_t axis_count{};
  std::uint32_t axis_stride{};
};

struct AxisCommand {
  std::uint64_t sequence{};
  std::int64_t timestamp_ns{};
  double target{};
  std::uint32_t flags{};
};
```

Create two memfds in the daemon, size each for the configured axis count, pass them with `SCM_RIGHTS`, and use two slots plus lock-free `std::atomic<std::uint32_t>` active index. Abort startup if the shared atomics are not lock-free.

- [ ] **Step 4: Run normal and stress tests**

```bash
ctest --test-dir build -R ipc_test --output-on-failure
ctest --preset tsan -R ipc_test --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add include/policy_runtime/robot_io include/policy_runtime/runtime src/robot_io src/runtime tests/cpp/integration/ipc_test.cpp
git commit -m "Add versioned robot I/O IPC"
```

### Task 6: CiA 402 State Machine and Axis Device

**Files:**
- Create: `include/policy_runtime/protocol/cia402/state_machine.hpp`
- Create: `include/policy_runtime/protocol/cia402/pdo.hpp`
- Create: `include/policy_runtime/protocol/cia402/units.hpp`
- Create: `include/policy_runtime/devices/cia402/axis.hpp`
- Create: `src/protocol/cia402/state_machine.cpp`
- Create: `src/devices/cia402/axis.cpp`
- Create: `tests/cpp/unit/cia402_state_machine_test.cpp`
- Create: `tests/cpp/unit/cia402_axis_test.cpp`

**Interfaces:**
- Consumes: `AxisConfig`, `AxisCommand`, `AxisFeedback`.
- Produces: `DriveState`, `Cia402StateMachine::update(statusword, request)`, and `Cia402Axis::cycle(command, pdo)`.

- [ ] **Step 1: Write the full table-driven transition test**

```cpp
struct Case { std::uint16_t status; AxisRequest request; std::uint16_t control; DriveState state; };
const Case cases[] = {
  {0x0040, AxisRequest::enable, 0x0006, DriveState::switch_on_disabled},
  {0x0021, AxisRequest::enable, 0x0007, DriveState::ready_to_switch_on},
  {0x0023, AxisRequest::enable, 0x000F, DriveState::switched_on},
  {0x0027, AxisRequest::quick_stop, 0x0002, DriveState::operation_enabled},
  {0x0008, AxisRequest::fault_reset, 0x0080, DriveState::fault},
};
```

- [ ] **Step 2: Run and confirm failures**

```bash
cmake --build build
ctest --test-dir build -R 'cia402_(state_machine|axis)_test' --output-on-failure
```

- [ ] **Step 3: Implement state decoding, controlword generation, units, and static modes**

```cpp
class Cia402Axis {
 public:
  explicit Cia402Axis(AxisConfig config);
  AxisFeedback cycle(const AxisCommand&, Cia402PdoView&) noexcept;
  Result<void> verify_mode(std::int8_t mode_display) const;
 private:
  AxisConfig config_;
  Cia402StateMachine state_machine_;
};
```

Map CSP/CSV/CST to 8/9/10. Apply configured scale, min/max, slew limit, following-error limit, and reject operation-enabled output when `0x6061` differs from the configured mode.

- [ ] **Step 4: Run unit tests**

```bash
ctest --test-dir build -R 'cia402_(state_machine|axis)_test' --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add include/policy_runtime/protocol/cia402 include/policy_runtime/devices/cia402 src/protocol/cia402 src/devices/cia402 tests/cpp/unit/cia402_*
git commit -m "Implement CiA 402 axis control"
```

### Task 7: IgH EtherCAT Transport and Elmo Gold Mapping

**Files:**
- Create: `cmake/FindEtherCAT.cmake`
- Create: `include/policy_runtime/transport/ethercat/master.hpp`
- Create: `include/policy_runtime/transport/ethercat/backend.hpp`
- Create: `include/policy_runtime/transport/ethercat/elmo_gold.hpp`
- Create: `src/transport/ethercat/master.cpp`
- Create: `src/transport/ethercat/igh_backend.cpp`
- Create: `src/transport/ethercat/elmo_gold.cpp`
- Create: `tests/cpp/unit/elmo_pdo_test.cpp`
- Create: `tests/cpp/integration/ethercat_fake_backend_test.cpp`

**Interfaces:**
- Produces: `EthercatBackend`, `IghBackend`, `EthercatMaster`, `ElmoGoldDeviceDescription`, typed PDO handles, and SDO requests.

- [ ] **Step 1: Write failing ESI-derived PDO golden tests**

```cpp
TEST(ElmoPdoTest, SelectsModeSpecificRxPdo) {
  EXPECT_EQ(ElmoGoldDeviceDescription::rx_pdo(Cia402Mode::csp), 0x1600);
  EXPECT_EQ(ElmoGoldDeviceDescription::rx_pdo(Cia402Mode::csv), 0x1601);
  EXPECT_EQ(ElmoGoldDeviceDescription::rx_pdo(Cia402Mode::cst), 0x1602);
  EXPECT_EQ(ElmoGoldDeviceDescription::assign_activate(), 0x0300);
}
```

- [ ] **Step 2: Configure without IgH and verify fake tests fail**

```bash
cmake -S . -B build -DPOLICY_RUNTIME_WITH_IGH=OFF
cmake --build build
ctest --test-dir build -R 'elmo_pdo|ethercat_fake' --output-on-failure
```

- [ ] **Step 3: Implement backend separation and the exact cycle**

```cpp
class EthercatBackend {
 public:
  virtual void receive() noexcept = 0;
  virtual void process_domain() noexcept = 0;
  virtual std::span<std::byte> process_image() noexcept = 0;
  virtual void queue_domain() noexcept = 0;
  virtual void send() noexcept = 0;
  virtual DomainHealth domain_health() const noexcept = 0;
};
```

Register vendor `0x0000009A`, product `0x00030924`, revision `0x00010420`; configure DC at 1,000,000 ns; bind PDO offsets before activation; and implement SDO writes for `0x6060` plus startup parameters. Compile `IghBackend` only when `POLICY_RUNTIME_WITH_IGH=ON`.

- [ ] **Step 4: Run fake backend and optional IgH build checks**

```bash
ctest --test-dir build -R 'elmo_pdo|ethercat_fake' --output-on-failure
cmake -S . -B build-igh -DPOLICY_RUNTIME_WITH_IGH=ON -DEtherCAT_ROOT=/usr
cmake --build build-igh
```

Expected: fake tests PASS; IgH build PASS on an IgH development host, otherwise configuration fails with a precise missing-library message.

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt cmake/FindEtherCAT.cmake include/policy_runtime/transport/ethercat src/transport/ethercat tests/cpp/unit/elmo_pdo_test.cpp tests/cpp/integration/ethercat_fake_backend_test.cpp
git commit -m "Add IgH EtherCAT transport"
```

### Task 8: Robot I/O Daemon, Real-Time Loop, and Safety

**Files:**
- Create: `include/policy_runtime/robot_io/daemon/daemon.hpp`
- Create: `include/policy_runtime/robot_io/daemon/realtime_loop.hpp`
- Create: `include/policy_runtime/robot_io/daemon/safety_supervisor.hpp`
- Create: `src/robot_io/daemon/daemon.cpp`
- Create: `src/robot_io/daemon/realtime_loop.cpp`
- Create: `src/robot_io/daemon/safety_supervisor.cpp`
- Create: `apps/robot_io_daemon_main.cpp`
- Create: `tests/cpp/integration/robot_io_daemon_test.cpp`

**Interfaces:**
- Consumes: profiles, IPC, `TransportScheduler`, `EthercatMaster`, and `Cia402Axis`.
- Produces: `robot-io-daemon` and health/safe-shutdown behavior.

- [ ] **Step 1: Write failing timeout, WKC, and SIGTERM tests**

```cpp
TEST(RobotIoDaemonTest, QuickStopsThenDisablesOnExpiredCommand) {
  FakeClock clock;
  DaemonHarness daemon{clock, one_axis_profile()};
  daemon.publish(command_at(0));
  clock.advance(21ms);
  daemon.cycle();
  EXPECT_EQ(daemon.axis_request(0), AxisRequest::quick_stop);
  daemon.set_velocity(0, 0.0);
  daemon.cycle();
  EXPECT_EQ(daemon.axis_request(0), AxisRequest::disable);
}
```

- [ ] **Step 2: Run and verify failures**

```bash
cmake --build build
ctest --test-dir build -R robot_io_daemon_test --output-on-failure
```

- [ ] **Step 3: Implement startup and cycle ownership**

```cpp
class RobotIoDaemon {
 public:
  Result<void> configure(const RobotProfile&);
  Result<void> start();
  Result<void> request_stop();
  DaemonHealth health() const noexcept;
};
```

Allocate before activation, verify lock-free shared atomics, call `mlockall`, set affinity and `SCHED_FIFO`, use `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`, and expose `realtime_guarantee=false` when the host is not PREEMPT_RT. Implement heartbeat timeout, consecutive WKC threshold, slave-OP loss, mode mismatch, safety groups, bounded fault reset, and SIGTERM Quick Stop → zero-speed confirmation → Disable.

- [ ] **Step 4: Run integration, sanitizer, and allocation-guard tests**

```bash
ctest --test-dir build -R robot_io_daemon_test --output-on-failure
ctest --preset tsan -R robot_io_daemon_test --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add include/policy_runtime/robot_io src/robot_io apps/robot_io_daemon_main.cpp tests/cpp/integration/robot_io_daemon_test.cpp
git commit -m "Add real-time robot I/O daemon"
```

### Task 9: C++ Policy Runtime Host and ZMQ Compatibility

**Files:**
- Create: `include/policy_runtime/runtime/runtime_host.hpp`
- Create: `include/policy_runtime/policy/policy.hpp`
- Create: `include/policy_runtime/policy/pipeline.hpp`
- Create: `src/runtime/runtime_host.cpp`
- Create: `src/policy/dummy_policy.cpp`
- Create: `src/policy/pipeline.cpp`
- Create: `apps/policy_runtime_host_main.cpp`
- Create: `tests/cpp/integration/runtime_host_test.cpp`

**Interfaces:**
- Consumes: RPC codec, policy profiles, and `RobotIoClient`.
- Produces: C++ `policy-runtime-host` with the existing CLI and ZMQ/JSON behavior.

- [ ] **Step 1: Write a failing golden RPC round-trip test**

```cpp
TEST(RuntimeHostTest, MatchesDummyPolicyResponseContract) {
  RuntimeHostHarness host{"policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"};
  auto response = host.handle(read_fixture("tests/golden/observation_request.json"));
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->type, "action_response");
  EXPECT_TRUE(response->payload.at("ok").get<bool>());
}
```

- [ ] **Step 2: Run and verify missing host symbols**

```bash
cmake --build build
ctest --test-dir build -R runtime_host_test --output-on-failure
```

- [ ] **Step 3: Port the current runtime flow**

Implement decode/validate → canonical binding → preprocess → policy infer → postprocess → RobotIoClient command publish → feedback response. Port health, server info, reset, error envelopes, dummy policy, noop processors, session step IDs, and current CLI flags before adding new features.

```cpp
class RuntimeHost {
 public:
  Result<MessageEnvelope> handle(const MessageEnvelope&);
  Result<void> open();
  void close() noexcept;
};
```

- [ ] **Step 4: Run C++/Python contract and TCP smoke tests**

```bash
ctest --test-dir build -R runtime_host_test --output-on-failure
pytest policy_embodied_runtime/tests/test_runtime.py policy_embodied_runtime/tests/test_messages.py -q
```

- [ ] **Step 5: Commit**

```bash
git add include/policy_runtime/runtime include/policy_runtime/policy src/runtime src/policy apps/policy_runtime_host_main.cpp tests/cpp/integration/runtime_host_test.cpp
git commit -m "Port policy runtime host to C++"
```

### Task 10: Move Serial and ST3215 into the Daemon

**Files:**
- Create: `include/policy_runtime/transport/serial/serial_transport.hpp`
- Create: `include/policy_runtime/protocol/st3215/protocol.hpp`
- Create: `include/policy_runtime/devices/st3215/servo.hpp`
- Create: `src/transport/serial/serial_transport.cpp`
- Create: `src/protocol/st3215/protocol.cpp`
- Create: `src/devices/st3215/servo.cpp`
- Create: `tests/cpp/unit/st3215_test.cpp`
- Create: `tests/cpp/integration/virtual_serial_test.cpp`

**Interfaces:**
- Consumes: `FrameTransport`, scheduler, daemon device registry, and current ST3215 wire contract.
- Produces: daemon-owned Serial/ST3215 Sensor and Actuator support.

- [ ] **Step 1: Port the Python protocol vectors as failing tests**

```cpp
TEST(St3215Test, EncodesGoalPositionPacket) {
  const auto bytes = St3215Protocol::goal_position_command(1, 2048, 0, 0);
  EXPECT_EQ(bytes, (Bytes{0xFF, 0xFF, 0x01, 0x09, 0x03, 0x2A,
                          0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xC0}));
}
```

- [ ] **Step 2: Run and verify failure**

```bash
cmake --build build
ctest --test-dir build -R 'st3215|virtual_serial' --output-on-failure
```

- [ ] **Step 3: Implement staged writes and daemon scheduling**

Port checksum, parser, radians conversion, goal writes, position reads, device sharing, and error reporting. `St3215Servo` stages requests; the serial executor alone performs blocking I/O and publishes feedback snapshots.

- [ ] **Step 4: Run C++ and Python simulator tests**

```bash
ctest --test-dir build -R 'st3215|virtual_serial' --output-on-failure
pytest policy_embodied_runtime/tests/test_st3215.py policy_embodied_runtime/tests/test_robot_io.py -q
```

- [ ] **Step 5: Commit**

```bash
git add include/policy_runtime/transport/serial include/policy_runtime/protocol/st3215 include/policy_runtime/devices/st3215 src/transport/serial src/protocol/st3215 src/devices/st3215 tests/cpp/unit/st3215_test.cpp tests/cpp/integration/virtual_serial_test.cpp
git commit -m "Move serial robot I/O into daemon"
```

### Task 11: Packaging, Service Definition, and Hardware Qualification

**Files:**
- Create: `packaging/systemd/robot-io-daemon.service`
- Create: `configs/robot_profiles/elmo_gold_example.json`
- Create: `tools/robot_io_health.cpp`
- Create: `tools/ethercat_cycle_stats.cpp`
- Create: `docs/robot-io-daemon.md`
- Create: `tests/cpp/integration/service_config_test.cpp`
- Modify: `README.md`

**Interfaces:**
- Produces: installable binaries, service hardening, health CLI, example configuration, and qualification procedure.

- [ ] **Step 1: Write failing install-layout and config tests**

```cpp
TEST(ServiceConfigTest, ExampleProfileHasStaticModesAndTimeouts) {
  auto profile = load_robot_profile("configs/robot_profiles/elmo_gold_example.json");
  ASSERT_TRUE(profile.has_value());
  for (const auto& axis : profile->axes) {
    EXPECT_GT(axis.command_timeout.count(), 0);
  }
}
```

- [ ] **Step 2: Run the test and CMake install check**

```bash
cmake --build build
ctest --test-dir build -R service_config_test --output-on-failure
cmake --install build --prefix /tmp/policy-runtime-install
```

- [ ] **Step 3: Add service and qualification behavior**

Set `LimitRTPRIO=95`, `LimitMEMLOCK=infinity`, `Restart=on-failure`, a dedicated runtime directory, and no shell/network privileges beyond those needed by IgH and the IPC socket. Document NIC IRQ affinity, performance governor, CPU isolation, `cyclictest`, DC/WKC metrics, command-timeout test, cable-disconnect test, and CSP/CSV/CST hardware checks.

- [ ] **Step 4: Verify packaging and docs commands**

```bash
ctest --test-dir build -R service_config_test --output-on-failure
cmake --install build --prefix /tmp/policy-runtime-install
/tmp/policy-runtime-install/bin/robot-io-health --help
/tmp/policy-runtime-install/bin/ethercat-cycle-stats --help
```

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt packaging configs tools docs/robot-io-daemon.md tests/cpp/integration/service_config_test.cpp README.md
git commit -m "Package robot I/O daemon"
```

### Task 12: Prove Parity and Retire the Python Runtime

**Files:**
- Create: `tests/compat/run_runtime_parity.py`
- Create: `docs/migration/cpp-runtime.md`
- Modify: `pyproject.toml`
- Modify/Delete after parity: Python runtime files under `policy_embodied_runtime/`, excluding `sim/` and simulator assets
- Modify: `README.md`

**Interfaces:**
- Consumes: all production binaries and golden fixtures.
- Produces: a C++-only runtime with the independent Python simulator retained.

- [ ] **Step 1: Write the parity harness before deleting Python**

```python
def test_cpp_matches_python(golden_request, python_host, cpp_host):
    py = normalize(python_host.request(golden_request))
    cpp = normalize(cpp_host.request(golden_request))
    assert cpp == py
```

Normalize only nondeterministic timestamps and process IDs; compare schemas, errors, policy IDs, payloads, step IDs, reset behavior, and profile rejection exactly.

- [ ] **Step 2: Run full parity and retain Python on any difference**

```bash
pytest tests/compat/run_runtime_parity.py -q
ctest --test-dir build --output-on-failure
pytest policy_embodied_runtime/tests -q
```

Expected: all suites PASS before removal.

- [ ] **Step 3: Remove only parity-proven Python runtime modules**

Keep `policy_embodied_runtime/sim/soarm101/` and `policy_embodied_runtime/sim/soarm101_assets/`. Update simulator packaging so `policy-soarm101-sim` remains installable independently. Remove Python host, protocol, robot, transport, model, profile, pre/postprocess, and their superseded tests only after each behavior is covered by C++ or parity tests.

- [ ] **Step 4: Run the final clean build and simulator smoke test**

```bash
cmake -S . -B build-clean -DPOLICY_RUNTIME_BUILD_TESTS=ON -DPOLICY_RUNTIME_WITH_IGH=OFF
cmake --build build-clean
ctest --test-dir build-clean --output-on-failure
uv run policy-soarm101-sim --help
git diff --check
```

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "Complete C++ runtime migration"
```

## Final Acceptance

- [ ] Build and test on both ARM64 and x86_64.
- [ ] Run ASan/UBSan and TSan presets with zero reported defects.
- [ ] Verify existing ZMQ/JSON clients and profiles without modification.
- [ ] Run 12 configured fake axes for at least one hour with no torn IPC snapshots.
- [ ] On PREEMPT_RT hardware, run Elmo CSP, CSV, and CST qualification, cable loss, host crash, daemon shutdown, WKC fault, command timeout, and drive fault recovery.
- [ ] Record maximum cycle time, deadline misses, DC deviation, WKC failures, and safe-stop latency in the release report.
