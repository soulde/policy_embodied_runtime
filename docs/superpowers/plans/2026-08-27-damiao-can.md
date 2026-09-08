# Generic Damiao CAN Motor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add a model-agnostic Damiao private motor protocol over SocketCAN and virtual serial transports.

**Architecture:** Canonical Damiao messages are encoded by a protocol module; transport adapters provide nonblocking frame I/O. A device adapter exposes Sensor/Actuator data to the realtime scheduler and latches any cycle error without recovery.

**Tech Stack:** C++20, existing Result/Transport interfaces, Linux SocketCAN, POSIX termios, GoogleTest, PTY fixtures.

**Spec:** `docs/superpowers/specs/2026-08-27-damiao-can-design.md`

## Global Constraints

- No product model names in protocol or device identifiers.
- Runtime configuration is static; no mode switching or reconnect.
- Realtime paths perform one nonblocking send and bounded receive per cycle.
- No retry, timeout wait, reopen, reinitialization, or dynamic allocation in realtime code.

### Task 1: Protocol codec

**Files:** create `include/policy_runtime/protocol/damiao/protocol.hpp`, `src/protocol/damiao/protocol.cpp`, and `tests/cpp/unit/damiao_protocol_test.cpp`; modify `CMakeLists.txt`.

- [ ] Write failing tests for canonical command/feedback packing, numeric bounds, invalid frame lengths, and invalid identifiers.
- [ ] Run the focused GoogleTest and confirm the missing header/implementation failure.
- [ ] Implement fixed-size encode/decode functions using the documented field layout and explicit endian conversions.
- [ ] Run the focused test until green, then run the protocol unit suite.
- [ ] Commit as `Add generic Damiao protocol codec`.

### Task 2: Transport adapters

**Files:** create `include/policy_runtime/transport/socketcan/socketcan_transport.hpp`, `src/transport/socketcan/socketcan_transport.cpp`, `include/policy_runtime/transport/serial/virtual_can_transport.hpp`, `src/transport/serial/virtual_can_transport.cpp`, and `tests/cpp/integration/damiao_transport_test.cpp`; modify `CMakeLists.txt`.

- [ ] Write failing tests for SocketCAN frame mapping and PTY serial framing, including EAGAIN and short-write fault results.
- [ ] Implement pre-opened nonblocking descriptors and bounded one-shot read/write operations without recovery logic.
- [ ] Run transport tests and verify no blocking calls or retries are present.
- [ ] Commit as `Add Damiao CAN transport adapters`.

### Task 3: Device adapter and configuration

**Files:** create `include/policy_runtime/devices/damiao/motor.hpp`, `src/devices/damiao/motor.cpp`, and `tests/cpp/integration/damiao_motor_test.cpp`; modify profile loader and CMake registration.

- [ ] Write failing tests for Sensor/Actuator conversion, static mode selection, stale feedback faulting, and one-shot send failure propagation.
- [ ] Implement the device adapter with preallocated state and latched fault semantics.
- [ ] Add generic `damiao_motor` profile parsing and reject unsupported transport/protocol combinations.
- [ ] Run device, profile, and existing C++ tests.
- [ ] Commit as `Add Damiao realtime motor device`.

### Task 4: Runtime integration and documentation

**Files:** modify runtime transport registry, example profiles, README, and `docs/migration/cpp-runtime.md`; add integration coverage for scheduler registration.

- [ ] Write failing registration and profile examples.
- [ ] Register both transport variants and document static configuration and restart-only recovery.
- [ ] Run clean CMake build, CTest, Python compatibility tests, and simulator help checks.
- [ ] Commit as `Integrate Damiao motor transports`.
