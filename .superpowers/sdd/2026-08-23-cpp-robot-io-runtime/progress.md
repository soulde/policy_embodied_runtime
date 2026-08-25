# SDD ledger — plan: docs/superpowers/plans/2026-08-23-cpp-robot-io-runtime.md

Baseline: `env UV_CACHE_DIR=/tmp/uv-cache uv run --extra dev pytest -q` — 28 passed.
Branch start: `c979473` on `feature/cpp-robot-io-runtime`.

## Pre-flight scan

| Tasks | Producer / consumer or internal check | Finding |
|---|---|---|
| 1 | Files, test, `Result<T>` agree | `Result<T>` is templated but plan also creates `src/common/result.cpp`; keep the translation unit as a header include/anchor. |
| 2 | RPC fixture, strict codec, tests agree | Clean. |
| 3 | Existing profile compatibility plus static CiA 402 mode | Clean. |
| 4 | Capability interfaces and scheduler isolation | Clean. |
| 5 | IPC test, fixed ABI, memfd, double-slot snapshots | Clean; lock-free atomic check is mandatory at daemon startup. |
| 6 | State table, axis interface, static mode | Clean. |
| 7 | Fake backend, optional IgH backend, ESI-derived values | Clean; untracked ESI is test input only and must not be committed. |
| 8 | Daemon tests and real-time/safety implementation | Clean. |
| 9 | RPC host parity and runtime flow | Clean. |
| 10 | ST3215 packet/vector and daemon scheduling | Clean; exact example checksum is `0xC0`. |
| 11 | Packaging test, service, qualification tools | Clean. |
| 12 | Parity gate before Python deletion and simulator retention | Clean. |
| 1 → 2,3,4,5 | `Result<T>` consumed throughout | Signatures agree. |
| 1 → 2,4,7 | Root `CMakeLists.txt` extended by later tasks | Additive edits; no conflict. |
| 2 → 9 → 12 | RPC types feed host, then parity gate | Contract remains `embodied-policy-runtime/v1alpha1`. |
| 3 → 6,8,9,11 | Profiles feed axis, daemon, host, example config | Static mode and 0–12 axes agree. |
| 4 → 7,8,10 | Transport capabilities feed EtherCAT, daemon scheduler, Serial | `write()` stages; `cycle()` owns physical I/O in every consumer. |
| 5 → 8,9 | IPC server/client feed daemon and host | Single-writer directions and generation checks agree. |
| 6 → 7,8 | CiA 402 axis consumes typed PDO and runs in daemon | Layering agrees: EtherCAT transport, CiA 402 protocol, axis device. |
| 7 → 8 | EtherCAT backend feeds real-time loop | Receive/process/state/update/queue/send order agrees. |
| 8 → 9,10,11 | Daemon hosts client IPC, Serial devices, service packaging | No conflicting ownership. |
| 9,10,11 → 12 | Host/device/service behavior gates Python retirement | Python removal occurs only after parity and clean build. |

Ruling: Keep CMake minimum 3.24 as specified; local CMake is 3.22.1, so task commands use a pinned CMake supplied through `uv` rather than weakening the requirement — cost if wrong: build commands may need adjustment on hosts without `uv`.
Ruling: Use GoogleTest from an installed package when available and a version-pinned CMake FetchContent fallback otherwise — cost if wrong: first configure requires network access.
Ruling: Keep `src/common/result.cpp` as a minimal translation-unit anchor while template definitions remain in the header — cost if wrong: one intentionally small source file remains.

## Task status

Task 1: implementer `/root/task_1_implementer`, base `c979473`.
Task 1: complete (commits `c979473..6153b1d`, review clean).
Task 2: implementer `/root/task_2_implementer`, base `6153b1d`.
Task 2: Ruling: keep the plan-mandated `std::uint64_t step_id` and strict JSON numeric types instead of reproducing Pydantic coercion/arbitrary-precision acceptance — wire values produced by supported clients remain compatible; cost if wrong: legacy clients relying on negative/coerced/greater-than-64-bit identifiers will be rejected.
Task 2: minor (deferred): byte-oriented blank-string checking differs from Python Unicode `strip()` for non-ASCII whitespace.
Task 2: fix round 1/5 started from reviewer head `70c33f1` — nested validation, typed payload dispatch, CMake fallback, and malformed-case coverage open.
Task 2: fix round 1/5 (4 addressed, 0 open; commits `70c33f1..6416517`).
Task 2: complete (commits `6153b1d..6416517`, review clean after 1 fix round).
Task 3: Ruling: preserve `sensors[]`/`actuators[]` as the robot-profile source of truth and expose `RobotProfile.axes` only as a derived, validated grouping of `cia402` device entries — cost if wrong: clients expecting a new top-level axes schema must instead use the compatible device-entry form.
Task 3: implementer `/root/task_3_implementer`, base `6416517`.
Task 3: Ruling: reject duplicate device names both within and across sensor/actuator lists as the plan explicitly requires, rather than preserve Python's same-side `setdefault()` loophole — cost if wrong: a legacy profile containing same-side duplicate names must be corrected before migration.
Task 3: minor (deferred): structured robot `args` values stringify as compact JSON rather than Python container repr; supported device args are scalar strings/numbers/bools.
Task 3: fix round 1/5 started from reviewer head `937a7dc` — Pydantic-compatible policy coercions, robot identifier trimming, and positive compatibility tests open.
Task 3: fix round 1/5 (1 addressed, 2 open; commits `937a7dc..8ef2c19`) — common coercions/trimming fixed, edge grammar and non-scalar identifier conversion remain.
Task 3: Ruling: temporal horizons must fit `std::uint64_t` even though Python integers are unbounded, because the values drive finite C++ runtime storage and loops — cost if wrong: profiles with horizons above `18446744073709551615` are rejected by C++.
Task 3: fix round 2/5 started from reviewer head `8ef2c19` — exact numeric-string grammar/non-finite bounds and non-scalar identifier acceptance open.
Task 3: fix round 2/5 (3 addressed, 0 open; commits `8ef2c19..b6fea64`).
Task 3: complete (commits `6416517..b6fea64`, review clean after 2 fix rounds).
Task 4: implementer `/root/task_4_implementer`, base `b6fea64`.
Task 4: Ruling: add the minimal `Result<void>` specialization required by `Transport::open()` and later IPC/daemon interfaces — cost if wrong: Task 4 touches the Task 1 error header and expands its review surface.
Task 4: minor (deferred): staged span APIs should explicitly state implementations copy caller storage before return.
Task 4: minor (deferred): add successful `Result<void>::value()` and typed-success regression coverage.
Task 4: fix round 1/5 started from reviewer head `4d0ee12` — non-owning registration lifetime, duplicate identity stability, OD mailbox execution/completion semantics, and weak isolation test open.
Task 4: fix round 1/5 (3 addressed, 1 open; commits `4d0ee12..cf56356`) — OD cycle-only I/O and upload payload path remain.
Task 4: Ruling: model EtherCAT PDO and mailbox as separate scheduled Transport endpoints sharing a later backend, rather than one object with two physical-I/O methods — cost if wrong: later EtherCAT implementation needs two small façade objects instead of one combined transport instance.
Task 4: fix round 2/5 started from reviewer head `cf56356` — OD cycle contract, upload payload, concurrency boundary, and invariant-clean test open.
Task 4: fix round 2/5 (1 addressed, 0 open; commits `cf56356..cb75e42`).
Task 4: complete (commits `b6fea64..cb75e42`, review clean after 2 fix rounds).
Task 7 carry-forward: concrete EtherCAT shared backend must enforce/test serialization between PDO and mailbox endpoint cycles outside the PDO hard-real-time critical window.
Task 5: implementer `/root/task_5_implementer`, base `cb75e42`.
Task 5: Ruling: support Unix `SOCK_SEQPACKET` only for the fixed setup record and SCM_RIGHTS transfer; remove the unrequired stream fallback instead of maintaining a partial stream protocol — cost if wrong: deployments using `SOCK_STREAM` must switch socket type.
Task 5: minor (deferred): 64-bit publication epoch has a theoretical wraparound ABA window.
Task 5: minor (deferred): `munmap()` failure discards the pointer and cannot retry cleanup.
Task 5: fix round 1/5 started from reviewer head `2a0e44e` — ancillary alignment, socket framing, header publication metadata, move/liveness safety, role capabilities/topology protection, and concurrent test strength open.
Task 5: fix round 1/5 (4 addressed, 2 open, 1 new; commits `2a0e44e..8ced12a`) — exposed view lifetime/liveness, wrong-role FD capability, and sequence-zero timestamp restriction remain.
Task 5: fix round 2/5 started from reviewer head `8ced12a` — endpoint capability closure and robust disconnect detection open.
Task 5: fix round 2/5 (4 addressed, 0 original open, 1 new; commits `8ced12a..2575089`) — unexpected-packet drain loop is unbounded under a flooding peer.
Task 5: fix round 3/5 started from reviewer head `2575089` — bounded unexpected-packet rejection open.
Task 5: fix round 3/5 (1 addressed, 0 open; commits `2575089..6ac2a13`).
Task 5: complete (commits `cb75e42..6ac2a13`, review clean after 3 fix rounds).
Task 6: started from `6ac2a13` — CiA402 state machine, scaled axis, mode enforcement, and fault-safe command handling.
Task 6: fix round 1/5 started from reviewer head `6059fea` — reverse-conversion finiteness, finite safety limits, and pre-enable following-error protection open.
Task 6: fix round 1/5 (3 addressed, 0 open; commits `6059fea..38c9393`).
Task 6: complete (commits `6ac2a13..38c9393`, review clean after 1 fix round).
Task 7: started from `38c9393` — IgH EtherCAT backend/master and ESI-derived Elmo Gold PDO mapping.
Task 7: fix round 1/5 started from reviewer head `6d2ef0b` — exact SDO sizing, PDO priority, pre-handler bus-health gating, static-mode OD immutability, and synchronized close/reopen open.
Task 7: minor (deferred to fix if naturally adjacent): real expected-WKC reporting and 12-axis/alias fake coverage.
Task 7: fix round 1/5 (4 addressed, 1 partial, 1 new Critical; commits `6d2ef0b..67e0bc4`) — PDO busy-spin priority inversion and mailbox-only reopen stale backend SDO remain.
Task 7: Ruling: mailbox callers only stage/observe requests while the sole cyclic PDO owner advances a bounded SDO state machine; the RT owner must never wait or spin on a lower-priority owner — cost if wrong: mailbox completion latency is quantized to the 1 kHz cycle but cyclic deadlines remain bounded.
Task 7: fix round 2/5 started from reviewer head `67e0bc4` — single-owner bounded SDO advancement and generation-gated mailbox lifecycle open.
Task 7: fix round 2/5 (2 original addressed, 2 new open; commits `67e0bc4..4587de2`) — close admission lacks one atomic linearization point and RT SDO failure creates owning strings.
Task 7: fix round 3/5 started from reviewer head `4587de2` — combined close/refcount state and allocation-free failure progress open.
Task 7: fix round 3/5 (2 addressed, 0 open; commits `4587de2..e8fdc50`).
Task 7: complete (commits `38c9393..e8fdc50`, review clean after 3 fix rounds).
Task 7: qualification deferred: real IgH linkage/runtime, arm64 execution, TSan, and 12-axis HIL require target hardware/environment.
Task 8: started from `e8fdc50` — real-time daemon scheduling, safety state, watchdog, and IPC integration.
Task 8: fix round 1/5 started from reviewer head `e7d1803` — fault-episode reset edges, overrun resynchronization/sleep failure, cycle-owner RT setup rollback, and race-free public snapshots open.
Task 8: minor adjacent fixes requested: separate wake jitter from execution time and prove signal-driven final Disable before deactivation.
Task 8: fix round 1/5 (overrun/metrics and signal/allocation addressed; 4 Important open/new; commits `e7d1803..175bc83`) — reset/CiA402 latch mismatch, owner TOCTOU, lossy rejection publication, and terminal restart remain.
Task 8: fix round 2/5 started from reviewer head `175bc83` — actual reset-pulse accounting, atomic exclusive cycle ownership, sticky rejection acknowledgement, and one-shot terminal lifecycle open.
Task 8: fix round 2/5 (3 addressed, 1 partial; commits `175bc83..11d767d`) — rejection barrier does not discard older valid snapshots or short-circuit same-cycle IPC input.
Task 8: fix round 3/5 started from reviewer head `11d767d` — monotonic rejection barrier and mandatory one-cycle safe-stop open.
Task 8: fix round 3/5 (1 addressed, 0 open; commits `11d767d..f556fee`).
Task 8: complete (commits `e8fdc50..f556fee`, review clean after 3 fix rounds).
Task 8: minor (deferred): uint64 command publication wrap affects raw ordering only after roughly 5.8e8 years at 1 kHz.
Task 9: started from `f556fee` — C++ runtime host, ZeroMQ RPC compatibility, and daemon IPC client integration.
Task 9: fix round 1/5 started from reviewer head `9c06e43` — exclude-none null semantics, correlated typed-payload errors, schema numeric normalization/type-sensitive parity, and cppzmq >=4.7 capability gate open.
Task 9: minor adjacent fixes requested: Python-style Pi0 setting stringification and argparse `--flag=value` compatibility.
Task 9: fix round 1/5 (4 Important and 2 Minor addressed, 0 serious open; commits `9c06e43..7cecfec`).
Task 9: complete (commits `f556fee..7cecfec`, review clean after 1 fix round).
Task 9: minor (deferred): Pi0 container metadata repr differs from Python for unusual non-printable Unicode strings; diagnostics are checked semantically but not byte-for-byte.
Task 10: started from `7cecfec` — Serial/ST3215 C++ transport/device and independent daemon executor integration.
Task 10: fix round 1/5 started from reviewer head `cb09c68` — versioned servo IPC/host mapping, command heartbeat, supervisor-latched serial faults, late-response isolation, device-error precedence, bounded cancellable stop, cross-process tty exclusion, and timeout_s=0 compatibility open.
Task 10: minor adjacent fixes requested: queue health/error fidelity, transactional configure retry, and complete rounding boundary vectors.
Task 10: implementation complete in `cb09c68` — serial/ST3215 port sharing, PTY recovery, daemon safety snapshots, full non-socket regression, sanitizer coverage, and 25x focused repeats passed; review pending.
Task 10: fix round 1 completed — ABI-v2 ST3215 IPC/RuntimeHost coverage,
dedicated ST-only E2E, direct forked TIOCEXCL verification, timeout-zero
parity, queue-error health fidelity, transactional configure retry, command
window bounds, and rounding-boundary coverage verified. Pinned build passed;
CTest 175/175 passed with exactly 15 sandbox-forbidden socket tests excluded;
ASan/UBSan and 25x focused non-socket repeats passed.
