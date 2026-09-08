# DDS Robot I/O IPC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace daemon-to-host Unix socket/shared-memory IPC with Cyclone DDS while exposing one command topic per actuator and one state topic per sensor.

**Architecture:** Physical bus cycles exchange data only with preallocated lock-free ingress snapshots and sensor-event queues. Non-realtime DDS bridge threads own Cyclone DDS entities, publish each physical receive event, and validate per-actuator commands; only EtherCAT commands use a group commit epoch. A temporary backend seam keeps Unix IPC available until DDS contract and end-to-end tests pass, after which Unix IPC is removed.

**Tech Stack:** C++20, CMake 3.24+, Eclipse Cyclone DDS C API, Cyclone DDS C++ binding, OMG IDL, GoogleTest, nlohmann-json.

**Spec:** `docs/superpowers/specs/2026-09-07-dds-robot-io-ipc-design.md`

## Global Constraints

- Cyclone DDS and Cyclone DDS C++ are system dependencies; CMake must perform no network download.
- Hard-realtime EtherCAT, Damiao CAN, ST3215 serial, and USB-CAN paths must not allocate, block, retry, reopen, reinitialize, or call DDS.
- Topic topology and transport modes are static after startup.
- Maximum topology remains 12 generic axes and 32 ST3215 servos.
- Every configured sensor owns one state topic and every configured actuator owns one command topic.
- Only EtherCAT actuator groups use command commit; no cross-bus synchronization is claimed.
- Local control DDS binds to loopback by default; the optional external domain publishes only allowlisted sensor and health topics.
- Existing untracked `Elmo+ECAT+00010420+V12.xml` and `build-damiao/` must never be staged.

---

### Task 1: Offline Cyclone DDS Build Integration

**Files:**
- Create: `cmake/PolicyRuntimeDds.cmake`
- Create: `tests/cmake/cyclonedds_probe/CMakeLists.txt`
- Create: `tests/cmake/cyclonedds_probe/fake/CycloneDDSConfig.cmake`
- Create: `tests/cmake/cyclonedds_probe/fake/CycloneDDS-CXXConfig.cmake`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: CMake option `POLICY_RUNTIME_WITH_DDS`, boolean `POLICY_RUNTIME_HAS_DDS`, target variables `POLICY_RUNTIME_DDS_C_TARGET` and `POLICY_RUNTIME_DDS_CXX_TARGET`.
- Produces: generated IDL target helper `policy_runtime_add_dds_types(TARGET <target> IDL <file>)`.

- [ ] **Step 1: Add a configure-time test that enables DDS against fake imported targets**

```cmake
add_test(NAME cyclonedds_package_probe
  COMMAND ${CMAKE_COMMAND}
    -S ${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/cyclonedds_probe
    -B ${CMAKE_CURRENT_BINARY_DIR}/cyclonedds-probe
    -DPOLICY_RUNTIME_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_SOURCE_DIR}/tests/cmake/cyclonedds_probe/fake)
```

- [ ] **Step 2: Run the probe and verify it fails because DDS discovery is absent**

Run: `ctest --test-dir build-damiao -R cyclonedds_package_probe --output-on-failure`

Expected: FAIL because the test or `PolicyRuntimeDds.cmake` does not exist.

- [ ] **Step 3: Implement strict system-package discovery**

```cmake
option(POLICY_RUNTIME_WITH_DDS "Build Cyclone DDS robot I/O IPC" OFF)
if(POLICY_RUNTIME_WITH_DDS)
  find_package(CycloneDDS CONFIG REQUIRED)
  find_package(CycloneDDS-CXX CONFIG REQUIRED)
  set(POLICY_RUNTIME_HAS_DDS ON)
endif()
```

The helper must accept the package targets exported by supported Cyclone DDS installations and invoke their IDL generator without `FetchContent`, `ExternalProject`, URLs, or package-manager commands.

- [ ] **Step 4: Run configure probes in DDS-on and DDS-off modes**

Run: `ctest --test-dir build-damiao -R 'cyclonedds_package_probe|cppzmq_.*_api_probe' --output-on-failure`

Expected: all selected tests PASS.

- [ ] **Step 5: Commit the build seam**

```bash
git add CMakeLists.txt cmake/PolicyRuntimeDds.cmake tests/cmake/cyclonedds_probe
git commit -m "Add offline Cyclone DDS build integration"
```

---

### Task 2: Bounded IDL and Topic Registry

**Files:**
- Create: `idl/policy_runtime/robot_io.idl`
- Create: `include/policy_runtime/robot_io/dds/topic_registry.hpp`
- Create: `src/robot_io/dds/topic_registry.cpp`
- Create: `tests/cpp/unit/dds_topic_registry_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `DdsTopicKind`, `DdsDeviceKind`, `DdsTopicDescriptor`.
- Produces: `Result<std::vector<DdsTopicDescriptor>> build_dds_topic_registry(const profiles::RobotProfile&)`.
- Produces: bounded IDL types `Cia402ActuatorCommand`, `Cia402SensorState`, `DamiaoActuatorCommand`, `DamiaoSensorState`, `St3215ActuatorCommand`, `St3215SensorState`, `EthercatCommandCommit`, and `RobotIoHealth`.

- [ ] **Step 1: Write registry tests for independent sensor/actuator topics and collisions**

```cpp
TEST(DdsTopicRegistryTest, GivesEachSensorAndActuatorExactlyOneTopic) {
  const auto registry = build_dds_topic_registry(profile_with_one_motor());
  ASSERT_TRUE(registry.has_value());
  EXPECT_EQ(count_kind(registry.value(), DdsTopicKind::command), 1U);
  EXPECT_EQ(count_kind(registry.value(), DdsTopicKind::state), 1U);
}

TEST(DdsTopicRegistryTest, RejectsNormalizedNameCollisions) {
  auto profile = profile_with_names("joint-1", "joint_1");
  EXPECT_EQ(build_dds_topic_registry(profile).error().code,
            ErrorCode::invalid_argument);
}
```

- [ ] **Step 2: Run the new unit test and verify it fails**

Run: `cmake --build build-damiao --target dds_topic_registry_test && ctest --test-dir build-damiao -R dds_topic_registry_test --output-on-failure`

Expected: build FAIL because the registry interfaces do not exist.

- [ ] **Step 3: Define bounded IDL and registry interfaces**

```cpp
struct DdsTopicDescriptor {
  DdsTopicKind kind;
  DdsDeviceKind device_kind;
  std::string robot_id;
  std::string device_id;
  std::string topic_name;
  std::string execution_group;
};
```

Use bounded IDL strings, explicit `schema_version`, `session_id`, `sequence`, `source_timestamp_ns`, `bus_cycle`, and `device_id`. Do not encode native C++ struct bytes.

- [ ] **Step 4: Implement normalization and static topology validation**

Normalize ASCII letters and digits, replace separators with one underscore, reject empty results, and reject any collision after normalization. Add one commit descriptor per nonempty EtherCAT group and one health descriptor per robot.

- [ ] **Step 5: Run registry and profile tests**

Run: `ctest --test-dir build-damiao -R 'dds_topic_registry_test|profile_loader_test' --output-on-failure`

Expected: PASS.

- [ ] **Step 6: Commit IDL and registry**

```bash
git add CMakeLists.txt idl include/policy_runtime/robot_io/dds src/robot_io/dds tests/cpp/unit/dds_topic_registry_test.cpp
git commit -m "Define per-device DDS topic contracts"
```

---

### Task 3: Realtime-Safe Command and Sensor Mailboxes

**Files:**
- Create: `include/policy_runtime/robot_io/dds/realtime_mailbox.hpp`
- Create: `tests/cpp/unit/dds_realtime_mailbox_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `template<class T, std::size_t Capacity> class SensorEventQueue` with `bool push_realtime(const T&) noexcept`, `std::optional<T> pop() noexcept`, and `std::uint64_t dropped() const noexcept`.
- Produces: `template<class T> class CommandMailbox` with `publish(const T&) noexcept` and `read_latest(std::uint64_t after_sequence) const noexcept`.
- Produces: `EthercatEpochMailbox::stage(index, command)`, `commit(epoch)`, and `consume_complete()`.

- [ ] **Step 1: Write tests for ordering, overflow, and EtherCAT atomicity**

```cpp
TEST(DdsRealtimeMailboxTest, FullSensorQueueDropsOldestAndCountsIt) {
  SensorEventQueue<int, 2> queue;
  EXPECT_TRUE(queue.push_realtime(1));
  EXPECT_TRUE(queue.push_realtime(2));
  EXPECT_TRUE(queue.push_realtime(3));
  EXPECT_EQ(queue.dropped(), 1U);
  EXPECT_EQ(queue.pop(), 2);
  EXPECT_EQ(queue.pop(), 3);
}

TEST(DdsRealtimeMailboxTest, EthercatEpochIsInvisibleUntilCompleteCommit) {
  EthercatEpochMailbox mailbox(2);
  mailbox.stage(0, command_for_epoch(9));
  mailbox.commit(9);
  EXPECT_FALSE(mailbox.consume_complete().has_value());
}
```

- [ ] **Step 2: Run the test and verify missing interfaces fail the build**

Run: `cmake --build build-damiao --target dds_realtime_mailbox_test`

Expected: FAIL with missing `realtime_mailbox.hpp`.

- [ ] **Step 3: Implement fixed-storage SPSC queues and snapshots**

Use `std::array`, lock-free atomics, acquire/release publication, no mutex, and no heap allocation after construction. Overflow advances the consumer-visible oldest sequence and preserves the newest event.

- [ ] **Step 4: Add allocation-guard stress tests**

Run one million producer/consumer operations after construction under the repository's allocation counter and assert zero allocations in `push_realtime`, command reads, epoch staging, commit, and consume.

- [ ] **Step 5: Run mailbox tests under repetition**

Run: `ctest --test-dir build-damiao -R dds_realtime_mailbox_test --repeat until-fail:20 --output-on-failure`

Expected: PASS for all repetitions.

- [ ] **Step 6: Commit realtime mailboxes**

```bash
git add CMakeLists.txt include/policy_runtime/robot_io/dds/realtime_mailbox.hpp tests/cpp/unit/dds_realtime_mailbox_test.cpp
git commit -m "Add realtime-safe DDS bridge mailboxes"
```

---

### Task 4: DDS Configuration in Robot Profiles

**Files:**
- Modify: `include/policy_runtime/profiles/robot_profile.hpp`
- Modify: `src/profiles/loader.cpp`
- Modify: `tests/cpp/unit/profile_loader_test.cpp`
- Modify: `configs/robot_profiles/elmo_gold_example.json`
- Modify: `policy_embodied_runtime/examples/robot_profiles/damiao_can_robot_profile.json`

**Interfaces:**
- Produces: `RobotIoBackend { unix_shm, dds }`, `DdsExternalConfig`, and `DdsConfig` stored in `RobotProfile`.
- `DdsConfig` contains `robot_id`, `local_domain_id`, `cyclone_config`, `sensor_queue_capacity`, and external-domain allowlist settings.

- [ ] **Step 1: Add failing schema tests**

```cpp
TEST(ProfileLoaderTest, ParsesStaticDdsTopologyAndExternalAllowlist) {
  const auto profile = load_profile(dds_profile_json());
  ASSERT_TRUE(profile.has_value());
  EXPECT_EQ(profile.value().dds.robot_id, "arm_a");
  EXPECT_FALSE(profile.value().dds.external.enabled);
}

TEST(ProfileLoaderTest, RejectsCommandTopicInExternalAllowlist) {
  EXPECT_FALSE(load_profile(profile_exposing_actuator_command()).has_value());
}
```

- [ ] **Step 2: Run profile tests and verify failure**

Run: `ctest --test-dir build-damiao -R profile_loader_test --output-on-failure`

Expected: FAIL because `RobotProfile::dds` is absent.

- [ ] **Step 3: Add strict profile structures and parsing**

```cpp
struct DdsExternalConfig {
  bool enabled{};
  std::uint32_t domain_id{};
  std::vector<std::string> topics;
};

struct DdsConfig {
  RobotIoBackend backend{RobotIoBackend::unix_shm};
  std::string robot_id;
  std::uint32_t local_domain_id{};
  std::string cyclone_config;
  std::size_t sensor_queue_capacity{64U};
  DdsExternalConfig external;
};
```

Validate domain ranges, nonempty robot ID, supported fixed queue capacities, topic existence, external-domain inequality, and sensor/health-only allowlists.

- [ ] **Step 4: Update examples and run tests**

Run: `ctest --test-dir build-damiao -R 'profile_loader_test|service_config_test' --output-on-failure`

Expected: PASS while profiles without `dds` continue to select the temporary Unix backend.

- [ ] **Step 5: Commit profile support**

```bash
git add include/policy_runtime/profiles/robot_profile.hpp src/profiles/loader.cpp tests/cpp/unit/profile_loader_test.cpp configs/robot_profiles/elmo_gold_example.json policy_embodied_runtime/examples/robot_profiles/damiao_can_robot_profile.json
git commit -m "Add static DDS robot profile configuration"
```

---

### Task 5: Daemon DDS Command Ingress and Sensor Egress

**Files:**
- Create: `include/policy_runtime/robot_io/dds/daemon_endpoint.hpp`
- Create: `src/robot_io/dds/daemon_endpoint.cpp`
- Create: `tests/cpp/integration/dds_daemon_endpoint_test.cpp`
- Modify: `include/policy_runtime/robot_io/daemon/daemon.hpp`
- Modify: `src/robot_io/daemon/daemon.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `Result<DdsDaemonEndpoint> DdsDaemonEndpoint::create(const profiles::RobotProfile&, DdsDaemonCallbacks)`.
- Produces: `start()`, `stop() noexcept`, `poll_health()`, and non-realtime worker ownership.
- `DdsDaemonCallbacks` stages validated independent commands, complete EtherCAT epochs, and consumes per-device sensor events.

- [ ] **Step 1: Write DDS daemon tests using two loopback participants**

```cpp
TEST(DdsDaemonEndpointTest, PublishesOneStateForEachPhysicalReceiveEvent) {
  harness.receive_cia402(position_sample(1.25));
  const auto state = subscriber.take_one<Cia402SensorState>();
  EXPECT_DOUBLE_EQ(state.position(), 1.25);
  EXPECT_EQ(state.bus_cycle(), harness.bus_cycle());
}

TEST(DdsDaemonEndpointTest, RejectsIncompleteEthercatCommit) {
  publisher.write(axis_command(0, 11));
  publisher.write(commit_for_epoch(11));
  EXPECT_FALSE(harness.consume_ethercat_epoch().has_value());
}
```

- [ ] **Step 2: Run DDS endpoint tests and verify failure**

Run: `cmake --build build-dds --target dds_daemon_endpoint_test && ctest --test-dir build-dds -R dds_daemon_endpoint_test --output-on-failure`

Expected: build FAIL because `DdsDaemonEndpoint` is absent.

- [ ] **Step 3: Implement entity creation and command validation outside realtime**

Create one typed reader per actuator, one typed writer per sensor, one EtherCAT commit reader per group, and one reliable health writer. Apply Reliable/KeepLast(1)/Volatile/Deadline/Lifespan/Exclusive Ownership to commands and BestEffort/KeepLast(1)/Volatile to sensor states.

- [ ] **Step 4: Connect daemon cycles only to preallocated mailboxes**

Replace direct DDS interaction with callbacks equivalent to:

```cpp
void on_sensor_sample(std::size_t sensor_index,
                      const SensorEvent& event) noexcept {
  sensor_queues_[sensor_index].push_realtime(event);
}
```

Assert through the allocation guard and a fake DDS endpoint that `cycle_owned` performs no DDS call and no allocation.

- [ ] **Step 5: Test sequence, timestamp, session, deadline, liveliness, and queue overflow**

Run: `ctest --test-dir build-dds -R 'dds_daemon_endpoint_test|robot_io_daemon_test|virtual_serial_test|damiao_.*_test' --output-on-failure`

Expected: PASS.

- [ ] **Step 6: Commit daemon DDS bridge**

```bash
git add CMakeLists.txt include/policy_runtime/robot_io/dds src/robot_io/dds include/policy_runtime/robot_io/daemon/daemon.hpp src/robot_io/daemon/daemon.cpp tests/cpp/integration/dds_daemon_endpoint_test.cpp
git commit -m "Bridge robot I/O daemon to Cyclone DDS"
```

---

### Task 6: Host DDS Runtime Adapter

**Files:**
- Create: `include/policy_runtime/runtime/dds_robot_io.hpp`
- Create: `src/runtime/dds_robot_io.cpp`
- Create: `tests/cpp/integration/dds_runtime_robot_io_test.cpp`
- Modify: `include/policy_runtime/runtime/runtime_host.hpp`
- Modify: `src/runtime/runtime_host.cpp`
- Modify: `apps/policy_runtime_host_main.cpp`
- Modify: `src/runtime/runtime_host_cli.cpp`
- Modify: `include/policy_runtime/runtime/runtime_host_cli.hpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `Result<std::unique_ptr<RuntimeRobotIo>> make_dds_runtime_robot_io(const profiles::RobotProfile&)`.
- Preserves the existing `RuntimeRobotIo` methods used by `RuntimeHost`.

- [ ] **Step 1: Write a contract test for per-device publishing and feedback assembly**

```cpp
TEST(DdsRuntimeRobotIoTest, PublishesIndependentCommandsAndEthercatCommit) {
  auto io = make_test_dds_runtime_robot_io(profile_with_two_ethercat_axes());
  ASSERT_TRUE(io->publish_commands(commands, {}, 7, now_ns()).has_value());
  EXPECT_EQ(reader(0).take().sequence(), 7U);
  EXPECT_EQ(reader(1).take().sequence(), 7U);
  EXPECT_EQ(commit_reader().take().epoch(), 7U);
}
```

- [ ] **Step 2: Run the test and verify failure**

Run: `cmake --build build-dds --target dds_runtime_robot_io_test`

Expected: FAIL because the DDS adapter does not exist.

- [ ] **Step 3: Implement the adapter and startup session handshake**

Create one writer per actuator and one reader per sensor. Generate a fresh nonzero session ID at host startup, publish commands with per-device sequence metadata, write EtherCAT commit only after all member writes succeed, and assemble the latest independent sensor samples into the existing bounded snapshots.

- [ ] **Step 4: Select backend from the loaded robot profile**

```cpp
if (profile.dds.backend == profiles::RobotIoBackend::dds) {
  robot_io = make_dds_runtime_robot_io(profile).value();
} else {
  robot_io = make_runtime_robot_io(std::move(unix_client));
}
```

Reject a requested DDS backend at runtime when `POLICY_RUNTIME_HAS_DDS` is false.

- [ ] **Step 5: Run host contract and DDS round-trip tests**

Run: `ctest --test-dir build-dds -R 'dds_runtime_robot_io_test|runtime_host_test|runtime_contract' --output-on-failure`

Expected: PASS.

- [ ] **Step 6: Commit the host adapter**

```bash
git add CMakeLists.txt include/policy_runtime/runtime src/runtime apps/policy_runtime_host_main.cpp tests/cpp/integration/dds_runtime_robot_io_test.cpp
git commit -m "Connect runtime host through Cyclone DDS"
```

---

### Task 7: Local-Only Defaults and External Telemetry Bridge

**Files:**
- Create: `include/policy_runtime/robot_io/dds/telemetry_bridge.hpp`
- Create: `src/robot_io/dds/telemetry_bridge.cpp`
- Create: `configs/cyclonedds-loopback.xml`
- Create: `tests/cpp/integration/dds_telemetry_bridge_test.cpp`
- Modify: `apps/robot_io_daemon_main.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces: `Result<TelemetryBridge> TelemetryBridge::create(const DdsConfig&, std::span<const DdsTopicDescriptor>)`.
- Produces: `start()` and `stop() noexcept`; no actuator or commit writer can be constructed through this interface.

- [ ] **Step 1: Write allowlist and two-domain tests**

```cpp
TEST(DdsTelemetryBridgeTest, RepublishesOnlyAllowlistedSensorAndHealthTopics) {
  bridge.publish_local(sensor_event("joint_1"));
  bridge.publish_local(sensor_event("joint_2"));
  EXPECT_TRUE(external_reader("joint_1").wait_for_sample());
  EXPECT_FALSE(external_reader("joint_2").has_sample());
}
```

- [ ] **Step 2: Run the test and verify failure**

Run: `cmake --build build-dds --target dds_telemetry_bridge_test`

Expected: FAIL because the bridge is absent.

- [ ] **Step 3: Add loopback-only default configuration**

The packaged XML must select only the loopback interface for the local domain and disable multicast beyond what same-host discovery requires. CLI/profile overrides must be explicit paths and must not mutate process-global DDS configuration after entity creation.

- [ ] **Step 4: Implement typed republishing on an external domain**

Build readers and writers only for registry descriptors whose kind is `state` or `health` and whose topic name is allowlisted. Return `ErrorCode::invalid_argument` before entity creation for command, commit, unknown, duplicate, or local-domain-equal entries.

- [ ] **Step 5: Run bridge and daemon tests**

Run: `ctest --test-dir build-dds -R 'dds_telemetry_bridge_test|dds_daemon_endpoint_test' --output-on-failure`

Expected: PASS.

- [ ] **Step 6: Commit selective telemetry**

```bash
git add CMakeLists.txt configs/cyclonedds-loopback.xml include/policy_runtime/robot_io/dds/telemetry_bridge.hpp src/robot_io/dds/telemetry_bridge.cpp apps/robot_io_daemon_main.cpp tests/cpp/integration/dds_telemetry_bridge_test.cpp
git commit -m "Add selective external DDS telemetry"
```

---

### Task 8: DDS Daemon/Host End-to-End Service

**Files:**
- Create: `tests/cpp/integration/dds_service_test.cpp`
- Create: `tests/compat/test_runtime_dds_smoke.py`
- Modify: `apps/robot_io_daemon_main.cpp`
- Modify: `packaging/systemd/robot-io-daemon.service`
- Modify: `docs/robot-io-daemon.md`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `DdsDaemonEndpoint::create`, `make_dds_runtime_robot_io`, static DDS profile topology.
- Produces: packaged daemon and host processes that discover each other without socket paths or generation files when `backend` is `dds`.

- [ ] **Step 1: Add a subprocess test with fake physical transports**

```cpp
TEST(DdsServiceTest, HostRestartCreatesNewSessionAndOldCommandsStayRejected) {
  auto daemon = start_daemon(dds_profile);
  auto first = start_host(dds_profile);
  const auto old_session = first.session_id();
  first.stop();
  auto second = start_host(dds_profile);
  EXPECT_NE(second.session_id(), old_session);
  EXPECT_TRUE(daemon.rejects(command_from(old_session)));
}
```

- [ ] **Step 2: Run the subprocess test and verify failure**

Run: `cmake --build build-dds --target dds_service_test && ctest --test-dir build-dds -R dds_service_test --output-on-failure`

Expected: FAIL until both executables select DDS directly.

- [ ] **Step 3: Complete daemon CLI and systemd startup for DDS**

For DDS profiles, daemon startup must create all participants, readers, writers, mailboxes, and bridge threads before activating physical transports. Socket and generation arguments remain accepted only for temporary `unix_shm` profiles.

- [ ] **Step 4: Add external ZeroMQ-to-DDS smoke coverage**

Run the Python compatibility client against `policy-runtime-host`, verify that a policy request produces per-actuator DDS commands, inject per-sensor DDS state, and verify the next host observation uses it.

- [ ] **Step 5: Run all DDS service tests repeatedly**

Run: `ctest --test-dir build-dds -R 'dds_.*|runtime_zmq_tcp_smoke' --repeat until-fail:10 --output-on-failure`

Expected: PASS with no socket or generation file created by DDS-mode processes.

- [ ] **Step 6: Commit end-to-end service support**

```bash
git add CMakeLists.txt apps/robot_io_daemon_main.cpp packaging/systemd/robot-io-daemon.service docs/robot-io-daemon.md tests/cpp/integration/dds_service_test.cpp tests/compat/test_runtime_dds_smoke.py
git commit -m "Run robot I/O service over Cyclone DDS"
```

---

### Task 9: Remove Unix Socket and Shared-Memory IPC

**Files:**
- Delete: `include/policy_runtime/robot_io/ipc_protocol.hpp`
- Delete: `include/policy_runtime/robot_io/ipc_server.hpp`
- Delete: `include/policy_runtime/robot_io/service_listener.hpp`
- Delete: `include/policy_runtime/runtime/robot_io_client.hpp`
- Delete: `include/policy_runtime/runtime/robot_io_service.hpp`
- Delete: `include/policy_runtime/runtime/robot_io_service_detail.hpp`
- Delete: `src/robot_io/ipc_server.cpp`
- Delete: `src/robot_io/service_listener.cpp`
- Delete: `src/runtime/robot_io_client.cpp`
- Delete: `src/runtime/robot_io_service.cpp`
- Delete: `tests/cpp/integration/ipc_test.cpp`
- Modify: `include/policy_runtime/robot_io/daemon/daemon.hpp`
- Modify: `src/robot_io/daemon/daemon.cpp`
- Modify: `include/policy_runtime/runtime/runtime_host.hpp`
- Modify: `src/runtime/runtime_host.cpp`
- Modify: `include/policy_runtime/runtime/runtime_host_cli.hpp`
- Modify: `src/runtime/runtime_host_cli.cpp`
- Modify: `apps/robot_io_daemon_main.cpp`
- Modify: `apps/policy_runtime_host_main.cpp`
- Modify: `tests/cpp/integration/service_config_test.cpp`
- Modify: `tests/cpp/integration/robot_io_daemon_test.cpp`
- Modify: `tests/cpp/integration/virtual_serial_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Removes: `RobotIoIpcServer`, `RobotIoClient`, `RobotIoServiceListener`, `connect_robot_io_service`, socket/generation CLI options, shared-memory ABI.
- Retains: transport-neutral command/feedback value types in a new `include/policy_runtime/robot_io/daemon/messages.hpp` included by DDS and runtime code.

- [ ] **Step 1: Move semantic value types out of the IPC ABI header**

```cpp
// include/policy_runtime/robot_io/daemon/messages.hpp
struct AxisCommand { /* existing semantic fields */ };
struct AxisFeedback { /* existing semantic fields */ };
struct St3215ServoCommand { /* existing semantic fields */ };
struct St3215ServoFeedback { /* existing semantic fields */ };
struct BusHealth { /* existing semantic fields */ };
```

Update includes and verify all non-IPC targets build before deleting the ABI implementation.

- [ ] **Step 2: Delete Unix IPC sources and remove CMake registrations**

Run: `rg -n 'RobotIoIpc|RobotIoClient|ServiceListener|ipc-socket|generation-file|SOCK_SEQPACKET|SCM_RIGHTS|memfd' include src apps tests packaging CMakeLists.txt`

Expected before edits: matches identify every removal site. Expected after edits: no matches except migration history documentation.

- [ ] **Step 3: Convert remaining tests to transport-neutral mailboxes or DDS fixtures**

Daemon safety tests must inject commands through `CommandMailbox`/`EthercatEpochMailbox`; virtual serial tests must observe sensor events through `SensorEventQueue`; service tests must launch DDS-mode processes.

- [ ] **Step 4: Make DDS required for daemon and host robot I/O**

Remove `RobotIoBackend::unix_shm`, socket/generation CLI options, direct descriptor startup, systemd socket paths, generation-file validation, and fallback messages. Preserve ZeroMQ options because they serve the separate external policy RPC.

- [ ] **Step 5: Run focused removal verification**

Run: `cmake --build build-dds && ctest --test-dir build-dds -R 'dds_.*|robot_io_daemon_test|virtual_serial_test|service_config_test|runtime_host_test' --output-on-failure`

Expected: PASS.

- [ ] **Step 6: Commit Unix IPC removal**

```bash
git add -A include/policy_runtime src apps tests/cpp packaging CMakeLists.txt
git commit -m "Remove Unix shared-memory robot I/O IPC"
```

---

### Task 10: Documentation, Packaging, and Full Verification

**Files:**
- Modify: `README.md`
- Modify: `docs/migration/cpp-runtime.md`
- Modify: `docs/qualification/cpp-runtime-final.md`
- Modify: `docs/robot-io-daemon.md`
- Modify: `packaging/systemd/robot-io-daemon.service`
- Modify: `configs/robot_profiles/elmo_gold_example.json`
- Modify: `policy_embodied_runtime/examples/robot_profiles/damiao_can_robot_profile.json`

**Interfaces:**
- Documents: system Cyclone DDS prerequisites, local-only default, profile schema, per-device topics, EtherCAT-only commit, monitoring, startup, and external allowlist.

- [ ] **Step 1: Update operator and migration documentation**

Document exact CMake flags, required package config names, Cyclone XML selection, topic naming, `robot_id`, QoS, queue-drop health counters, session restart behavior, and confirmation that external ZeroMQ remains unchanged.

- [ ] **Step 2: Configure clean DDS and non-hardware builds**

Run: `env UV_CACHE_DIR=/tmp/uv-cache uv run --with 'cmake>=3.24' cmake -S . -B build-dds -DPOLICY_RUNTIME_WITH_IGH=OFF -DPOLICY_RUNTIME_WITH_DDS=ON`

Expected: PASS on a system with Cyclone DDS C and C++ packages installed; otherwise fail with a direct missing-package diagnostic.

- [ ] **Step 3: Build and run the complete C++ suite**

Run: `env UV_CACHE_DIR=/tmp/uv-cache uv run --with 'cmake>=3.24' cmake --build build-dds`

Run: `ctest --test-dir build-dds --output-on-failure`

Expected: all enabled tests PASS. Compare any failure against the two pre-existing service timing failures recorded before DDS implementation rather than suppressing it.

- [ ] **Step 4: Run the complete Python compatibility and simulator suite**

Run: `env UV_CACHE_DIR=/tmp/uv-cache uv run --extra dev pytest`

Expected: all tests PASS; Python simulation remains functional.

- [ ] **Step 5: Verify source and package hygiene**

Run: `git diff --check && rg -n 'SOCK_SEQPACKET|SCM_RIGHTS|memfd_create|robot-io.sock|generation-file' include src apps packaging CMakeLists.txt`

Expected: no source/package matches. Confirm `git status --short` lists no generated DDS output, build artifacts, XML, or unrelated files staged for commit.

- [ ] **Step 6: Commit final documentation**

```bash
git add README.md docs/migration/cpp-runtime.md docs/qualification/cpp-runtime-final.md docs/robot-io-daemon.md packaging/systemd/robot-io-daemon.service configs/robot_profiles/elmo_gold_example.json policy_embodied_runtime/examples/robot_profiles/damiao_can_robot_profile.json
git commit -m "Document Cyclone DDS robot I/O deployment"
```

- [ ] **Step 7: Request final code review**

Use `superpowers:requesting-code-review` against the complete `feature/dds-ipc` diff, address verified findings, rerun the complete verification commands, and then use `superpowers:finishing-a-development-branch` for integration.
