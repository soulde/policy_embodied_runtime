# Task 10 Fix Round 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete daemon-owned ST3215 support across versioned IPC and RuntimeHost while making command, safety, serial transaction, exclusivity, and shutdown behavior fail-safe.

**Architecture:** Axis-only profiles retain the exact ABI-v1 two-region setup. Profiles with ST3215 servos use a self-describing ABI-v2 setup carrying four typed shared-memory regions; the new client accepts both versions and validates every descriptor role, size, count, seal, and access capability. Servo faults enter `SafetySupervisor` before evaluation, while a single cancelable serial owner enforces command freshness and response generations without adding blocking, allocation, or serial calls to the EtherCAT cycle.

**Tech Stack:** C++20, Linux memfd/SCM_RIGHTS shared memory, termios/poll/eventfd/ioctl, GoogleTest, pytest, CMake/CTest.

**Spec:** Approved Task 10 fix-round design in the parent task message and `docs/superpowers/specs/2026-08-23-cpp-robot-io-runtime-design.md`.

## Global Constraints

- Preserve exact ABI-v1 setup and mapping behavior for axis-only profiles.
- ABI-v2 setup is self-describing and rejects malformed descriptor roles, counts, sizes, capabilities, and versions without leaking descriptors.
- Bounds remain fixed at 12 CiA402 axes and 32 ST3215 servos; realtime paths allocate nothing and perform no serial operations.
- Authoritative servo ordering is derived stably from validated robot-profile sensor/actuator order and rejects duplicate or ambiguous physical/name mappings.
- Every behavior change follows RED → observed expected failure → minimal GREEN.
- Do not modify or stage `Elmo+ECAT+00010420+V12.xml` or `uv.lock`.

---

### Task 1: Versioned Axis and Servo IPC

**Files:**
- Modify: `include/policy_runtime/robot_io/ipc_protocol.hpp`
- Modify: `include/policy_runtime/robot_io/snapshot.hpp`
- Modify: `include/policy_runtime/robot_io/ipc_server.hpp`
- Modify: `src/robot_io/ipc_server.cpp`
- Modify: `include/policy_runtime/runtime/robot_io_client.hpp`
- Modify: `src/runtime/robot_io_client.cpp`
- Test: `tests/cpp/unit/ipc_test.cpp`

**Interfaces:**
- Produces ABI-v2 setup metadata for command/feedback axis and servo regions, plus v1/v2 `RobotIoClient` APIs.
- Keeps existing `RobotIoIpcServer::create(socket, axis_count, generation)` and v1 behavior; adds an explicit servo-count creation path.

- [ ] Add tests proving v1 setup remains byte/layout compatible, v2 advertises four typed mappings, exact four-FD rights and access modes are enforced, malformed/unknown setup closes all received FDs, and the new client connects to both versions.
- [ ] Run `ctest --test-dir build -R 'ipc_test' --output-on-failure` and observe compile/assertion failures caused by missing v2 APIs.
- [ ] Add fixed servo IPC wire records, per-record snapshot bounds, self-describing v2 setup entries, four backing mappings, exact ancillary parsing, and v1/v2 client/server ownership.
- [ ] Re-run the focused IPC tests until GREEN without weakening existing v1 tests.

### Task 2: RuntimeHost Servo Bindings and Daemon Data Plane

**Files:**
- Modify: `include/policy_runtime/runtime/runtime_host.hpp`
- Modify: `src/runtime/runtime_host.cpp`
- Modify: `include/policy_runtime/robot_io/daemon.hpp`
- Modify: `src/robot_io/daemon.cpp`
- Modify: `apps/robot_io_daemon_main.cpp`
- Test: `tests/cpp/integration/runtime_host_test.cpp`
- Test: `tests/cpp/integration/robot_io_daemon_test.cpp`

**Interfaces:**
- Extends `RuntimeRobotIo` with servo topology, feedback reads, and command publication.
- Maps validated `St3215ServoProfile.sensor_name` and `.actuator_name` in stable profile order.

- [ ] Add fake-channel tests for ST3215-only and mixed profiles, canonical input/output bindings, duplicate/ambiguous names, monotonic sequence/timestamps, and servo topology mismatch.
- [ ] Add a real socket/memfd v2 test that publishes servo feedback through the daemon side and observes the host-published servo command; mark it as controller-runnable if the sandbox rejects socket inspection.
- [ ] Run focused runtime/daemon tests and observe missing servo interface/layout failures.
- [ ] Implement daemon realtime shared-memory servo snapshot ingestion/publication and RuntimeHost servo mapping/command extraction using pre-sized fixed topology.
- [ ] Re-run focused tests to GREEN and confirm axis-only behavior stays unchanged.

### Task 3: Servo Command Freshness and Integrated Safety Episodes

**Files:**
- Modify: `include/policy_runtime/profiles/robot_profile.hpp`
- Modify: `src/profiles/loader.cpp`
- Modify: `include/policy_runtime/robot/devices/st3215/servo.hpp`
- Modify: `src/robot/devices/st3215/servo.cpp`
- Modify: `include/policy_runtime/robot_io/safety_supervisor.hpp`
- Modify: `src/robot_io/safety_supervisor.cpp`
- Modify: `src/robot_io/daemon.cpp`
- Test: `tests/cpp/unit/profile_loader_test.cpp`
- Test: `tests/cpp/unit/st3215_test.cpp`
- Test: `tests/cpp/integration/virtual_serial_test.cpp`
- Test: `tests/cpp/integration/robot_io_daemon_test.cpp`

**Interfaces:**
- Adds static per-servo `command_timeout` and serial-fault axis masks to the safety input.
- Requires a sequence newer than the sequence recorded when a serial fault episode stopped a group.

- [ ] Add tests that reject future timestamps, stop resending expired enabled goals, mark command timeout, drive QuickStop then Disable through `SafetySupervisor`, hold Disable after fault clearance, and recover only on a newer valid sequence.
- [ ] Run focused tests and observe old resend and post-hoc one-cycle QuickStop failures.
- [ ] Enforce command freshness in the serial owner and inject serial reasons before `SafetySupervisor::evaluate`; remove daemon post-processing overrides.
- [ ] Re-run focused safety and serial tests to GREEN, including the no-allocation daemon cycle test.

### Task 4: Transaction Isolation, Status Errors, and ACK Policy

**Files:**
- Modify: `include/policy_runtime/robot/devices/st3215/servo.hpp`
- Modify: `src/robot/devices/st3215/servo.cpp`
- Modify: `src/transport/serial/serial_transport.cpp`
- Test: `tests/cpp/unit/st3215_test.cpp`
- Test: `tests/cpp/integration/virtual_serial_test.cpp`

**Interfaces:**
- Produces a bounded resynchronization generation after timeout and an optional write-ACK policy matching Python's non-required ACK behavior.

- [ ] Add deterministic PTY tests where a timed-out same-ID position response arrives during the next transaction and must never refresh feedback.
- [ ] Add tests that a nonzero status with zero parameters publishes `DeviceError` and exact `status_error` before any success-shape requirement; verify a missing goal ACK does not block the subsequent position read.
- [ ] Run focused tests and observe stale-frame acceptance, wrong shape-first classification, and required-ACK failures.
- [ ] Implement bounded drain/quiet generation isolation, status-error-first parsing, and optional goal ACK handling.
- [ ] Re-run focused tests to GREEN.

### Task 5: Cancelable Shutdown, Port Exclusivity, and Compatibility Edges

**Files:**
- Modify: `include/policy_runtime/transport/serial/serial_transport.hpp`
- Modify: `src/transport/serial/serial_transport.cpp`
- Modify: `src/profiles/loader.cpp`
- Modify: `src/robot/devices/st3215/servo.cpp`
- Test: `tests/cpp/integration/virtual_serial_test.cpp`
- Test: `tests/cpp/unit/profile_loader_test.cpp`
- Test: `tests/cpp/unit/st3215_test.cpp`

**Interfaces:**
- Adds a stop wakeup consumed by serial poll and kernel-enforced physical-port exclusivity.
- Preserves Python `timeout_s=0` as nonblocking and bounds positive read/write/service/command timeouts.

- [ ] Add tests for shutdown during a very large requested wait, rejection of unsafe timeout bounds, `TIOCEXCL` exclusion from a second process, release after close, explicit/default timeout zero, queue-full degraded health/precise errors, configure retry, and ties-to-even/full-turn conversion.
- [ ] Run focused tests and observe unbounded join, cross-process open success, timeout-default mismatch, and missing health/config behavior.
- [ ] Add eventfd cancellation to every blocking poll, upper-bound validation, `TIOCEXCL` lifecycle, nonblocking zero-timeout behavior, transactional configure, and precise queue health.
- [ ] Re-run focused tests to GREEN.

### Task 6: Verification, Report, and Commit

**Files:**
- Modify: `.superpowers/sdd/2026-08-23-cpp-robot-io-runtime/task-10-report.md`
- Modify: `.superpowers/sdd/2026-08-23-cpp-robot-io-runtime/progress.md`

- [ ] Run a fresh all-target build and all focused IPC/runtime/daemon/ST3215/PTY tests.
- [ ] Run full CTest, recording only the established sandbox socket exclusions when applicable.
- [ ] Run Python ST3215/robot-I/O/profile compatibility tests, ASan/UBSan focused coverage, and 25x focused repeats.
- [ ] Run `git diff --check`, inspect staged names, preserve the two prohibited untracked files, append the fix-round report, and commit with an imperative Task 10 fix message.
