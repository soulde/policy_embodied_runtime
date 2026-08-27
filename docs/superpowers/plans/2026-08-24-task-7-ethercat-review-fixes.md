# Task 7 EtherCAT Review Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct IgH SDO sizing, enforce PDO-priority backend arbitration, move health gating before command staging, protect static mode objects, and make close/reopen race-free.

**Architecture:** Keep one shared EtherCAT backend and distinct hard-real-time PDO and blocking mailbox endpoints. Replace the symmetric try-lock with priority arbitration: PDO callers register as waiters and are never dropped, while mailbox callers enter only when no PDO is active or waiting. Lifecycle counters make non-real-time close wait for in-flight cycles before deactivation. IgH uses pre-activation, exact-size download request pools separate from its upload request.

**Tech Stack:** C++20, CMake 3.24+, GoogleTest, IgH EtherCAT stable-1.5/stable-1.6 userspace API, ASan/UBSan.

**Spec:** Review instructions for Task 7 fix round 1/5, based on commit `6d2ef0b`.

## Global Constraints

- Use RED→GREEN tests for every behavioral correction.
- Preserve 1 kHz PDO priority and serialized backend access.
- Runtime mode remains static; runtime writes to `0x6060` and `0x60C2` are forbidden.
- Do not modify or commit `Elmo+ECAT+00010420+V12.xml` or `uv.lock`.
- Do not spawn subagents.

---

### Task 1: Exact-Size IgH Download Requests

**Files:**
- Modify: `src/transport/ethercat/igh_backend.cpp`
- Modify: `CMakeLists.txt`
- Create: `tests/cpp/support/ecrt.h`
- Create: `tests/cpp/support/igh_shim.hpp`
- Create: `tests/cpp/support/igh_shim.cpp`
- Create: `tests/cpp/integration/igh_sdo_semantics_test.cpp`

**Interfaces:**
- Consumes: IgH pre-activation `ecrt_slave_config_create_sdo_request(sc, index, subindex, size)` lifecycle.
- Produces: separate upload request and exact-size download request selection for supported scalar sizes `1`, `2`, `4`, and `8` bytes.

- [ ] Write an IgH-shim integration test that completes an upload with a non-download data size, then schedules a two-byte download and asserts the shim receives exactly two bytes with the caller payload.
- [ ] Run only `igh_sdo_semantics_test`; verify the current shared 4096-byte request reports the stale upload/reserved size.
- [ ] Precreate independent 1/2/4/8-byte download requests plus one 4096-byte upload request before activation; reject empty, oversized, and unsupported download lengths.
- [ ] Run the shim test and existing EtherCAT tests; verify the semantic size test passes.

### Task 2: PDO-Priority Arbitration

**Files:**
- Modify: `include/policy_runtime/transport/ethercat/backend.hpp`
- Modify: `src/transport/ethercat/master.cpp`
- Modify: `tests/cpp/integration/ethercat_fake_backend_test.cpp`

**Interfaces:**
- Produces: `acquire_pdo()`, `try_acquire_mailbox()`, and `release_access()` backend arbitration, with an atomic PDO-waiter count.

- [ ] Replace the old “PDO returns degraded when mailbox owns the backend” assertion with a failing test proving one PDO receive/send still occurs after active mailbox contention clears.
- [ ] Add a test that queues repeated mailbox cycles while a PDO waiter exists and proves the mailbox defers until the PDO completes, then makes progress.
- [ ] Implement priority admission: PDO registers before waiting for the serialized backend; mailbox performs two waiter checks around its nonblocking acquisition and yields when PDO is pending.
- [ ] Repeat the contention/order tests 100 times.

### Task 3: Pre-Staging Health Gate and Safe Output

**Files:**
- Modify: `include/policy_runtime/transport/ethercat/backend.hpp`
- Modify: `src/transport/ethercat/master.cpp`
- Modify: `src/transport/ethercat/igh_backend.cpp`
- Modify: `tests/cpp/integration/ethercat_fake_backend_test.cpp`

**Interfaces:**
- Produces: domain health sampled immediately after receive/process; unhealthy cycles bypass the handler and write control word/active target zero before queue/send.
- Produces: optional expected-WKC representation; IgH reports it unavailable and relies on `EC_WC_COMPLETE` rather than copying actual WKC into expected WKC.

- [ ] Add a failing event-order test with unhealthy WKC that proves health is sampled before process-image access, the normal handler is not called, and transmitted PDO outputs are safe zeros.
- [ ] Move health sampling before image/feedback/handler staging and add the safe-output branch.
- [ ] Change expected WKC to optional, make equality conditional on availability, and make IgH return no fabricated expected value.
- [ ] Run exact-cycle, unhealthy-cycle, and full fake-backend tests.

### Task 4: Static Object Runtime Protection

**Files:**
- Modify: `src/transport/ethercat/master.cpp`
- Modify: `tests/cpp/integration/ethercat_fake_backend_test.cpp`

**Interfaces:**
- Produces: runtime download rejection for every subindex of indexes `0x6060` and `0x60C2`; internal Elmo startup SDO generation remains unchanged.

- [ ] Add failing table-driven tests for `0x6060:00`, another `0x6060` subindex, `0x60C2:01`, and another `0x60C2` subindex.
- [ ] Replace existing runtime download fixtures using `0x6060` with a non-static manufacturer object.
- [ ] Reject protected index families in mailbox enqueue and user startup overrides while leaving internal startup generation intact.
- [ ] Run the focused static-object and startup configuration tests.

### Task 5: Close/Reopen Synchronization

**Files:**
- Modify: `include/policy_runtime/transport/ethercat/master.hpp`
- Modify: `src/transport/ethercat/master.cpp`
- Modify: `tests/cpp/integration/ethercat_fake_backend_test.cpp`

**Interfaces:**
- Produces: atomic open state, in-flight cycle counters with close-side waits, terminal failure status for queued/in-progress requests, and fresh reopen state.

- [ ] Add a failing PDO-close test proving deactivate does not run until a blocked PDO handler exits.
- [ ] Add a failing mailbox-close test proving deactivate waits for in-flight mailbox access, the request becomes failed, and no request survives reopen.
- [ ] Implement cycle entry/exit counters with a post-increment open recheck; close first rejects new work, waits for counters, then deactivates.
- [ ] Mark every queued/in-progress mailbox request failed during close and retain terminal status snapshots; ensure reopen cannot execute them.
- [ ] Run close tests under ASan/UBSan and repeat them 100 times.

### Task 6: Multi-Axis/Alias Coverage and Verification

**Files:**
- Modify: `tests/cpp/integration/ethercat_fake_backend_test.cpp`
- Modify: `.superpowers/sdd/2026-08-23-cpp-robot-io-runtime/task-7-report.md`

**Interfaces:**
- Produces: practical two-axis coverage with distinct aliases and the same relative position.

- [ ] Add a two-axis/alias test that verifies two configurations, fourteen unique PDO bindings, and independent per-axis staging.
- [ ] Run IgH-disabled configure/build, all CTest, Python pytest, full ASan/UBSan, and contention/close repetitions.
- [ ] Compile the IgH backend against official stable-1.5 and stable-1.6 headers and verify missing dependency diagnostics.
- [ ] Run `git diff --check`, confirm XML checksum/`uv.lock` status, append the task report, stage only intended files, and commit.
