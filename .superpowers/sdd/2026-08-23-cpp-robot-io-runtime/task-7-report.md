# Task 7: IgH EtherCAT and Elmo Gold Mapping

## Result

Implemented an optional IgH EtherCAT backend, exact Elmo Gold ESI-derived PDO/DC/startup-SDO configuration, typed process-image fields, and a master-owned PDO/SDO cycle with a separate staging-only mailbox frontend.

The cycle is fixed as `receive -> domain process -> domain health -> safe/normal staging -> at most one SDO/cancel step -> domain queue -> send`. SDO requests are published by a separate mailbox endpoint and advanced asynchronously by the same sole owner.

## ESI Evidence

- Source file: `Elmo+ECAT+00010420+V12.xml` (kept untracked and unmodified)
- SHA-256: `42286fa8219f6b98bd28532d9a1868443c1cd178b1bc35027643f81c57afe0d9`
- Identity: vendor `0x0000009A`, product `0x00030924`, revision `0x00010420`
- RxPDOs: CSP `0x1600`, CSV `0x1601`, CST `0x1602`
- Feedback PDOs: `0x1A02` plus velocity PDO `0x1A11`
- DC: assign activate `0x0300`, Sync0 period `1,000,000 ns`
- Startup SDOs: static `0x6060:00` mode and `0x60C2:01 = 2`

## Verification

- IgH disabled build: configured and built successfully.
- C++ CTest: 89/89 passed.
- Python pytest: 28/28 passed.
- ASan/UBSan CTest: 89/89 passed; focused EtherCAT suite: 13/13 passed.
- Mailbox/PDO concurrency tests: each passed 50 consecutive repetitions.
- IgH discovery without local development files: failed as intended with a precise `ecrt.h`/`libethercat` diagnostic.
- IgH source compile compatibility was checked against official stable-1.5 and stable-1.6 headers. The local environment lacks a real IgH library and EtherCAT hardware, so link/runtime/HIL behavior was not claimed.
- `git diff --check`: clean.

## Carry-Forward

Daemon integration should provide application-time/DC clock synchronization calls and expose a configured expected-WKC policy. Real IgH linkage and 12-axis Elmo hardware-in-the-loop verification remain environment-dependent follow-up work.

## Review Fix Round 1

- Replaced the shared 4096-byte IgH SDO request with separate upload storage and pre-activation 1/2/4/8-byte download requests. Runtime downloads outside those exact scalar sizes fail with `invalid_argument`; they are never truncated or padded.
- Added a compiled IgH API shim test that first changes upload `data_size` to seven bytes, then proves a two-byte download transmits exactly the caller's two bytes. A three-byte request is explicitly rejected.
- Changed backend arbitration so PDO callers register priority and wait for the bounded current owner; new mailbox admission yields while PDO is active or pending. PDO cycles are no longer discarded on contention.
- Samples domain/WKC/slave health immediately after receive/process. Unhealthy cycles bypass the handler and transmit zero control word plus zero active-mode target.
- Rejects runtime downloads and user startup overrides for every subindex of `0x6060` and `0x60C2`; internally generated static startup SDOs remain enabled.
- Close atomically rejects new cycles, fails queued/in-progress mailbox requests, waits for PDO and mailbox cycles, then deactivates. Reopen retains terminal status only and cannot execute stale work.
- IgH reports expected WKC as unavailable instead of copying actual WKC. Public IgH userspace exposes `working_counter` and `wc_state`, not the numeric domain expected WKC.
- Added independent two-axis coverage for distinct aliases sharing relative position zero.

### Review Verification

- IgH-disabled CTest: 98/98 passed.
- Python pytest: 28/28 passed.
- ASan/UBSan CTest: 98/98 passed.
- Two contention and two close/reopen tests: each passed 100 consecutive repetitions.
- IgH backend compile-only checks passed with official stable-1.5 and stable-1.6 headers. These use placeholder archives and do not claim real linking or hardware operation.
- Missing local IgH development files still produce the precise required dependency diagnostic.

## Review Fix Round 2

- Made `EthercatMaster::cycle()` the sole backend/ecrt owner. Mailbox public calls now only enqueue, publish one stable request pointer through a lock-free atomic slot, report status, and manage frontend lifecycle.
- Removed the PDO/mailbox arbitration spin lock. The 1 kHz owner never takes a mailbox mutex: it completes PDO receive/process/health/protocol/output staging, advances at most one round-robin SDO or cancellation step globally, then queues and sends the domain.
- Preallocates the 4096-byte upload destination on the frontend. IgH copies successful upload bytes into that storage and reports only the completed size, avoiding normal-path vector allocation in the cyclic owner.
- Added owner-only SDO cancellation. Because public IgH has no asynchronous abort call, close drains one request-state observation per owner cycle and refuses child reopen until the request reaches a terminal state. Parent shutdown instead waits for its admitted cycle, deactivates the backend, and resets all child owner state.
- Added parent/child lifecycle generations. A child cannot open or enqueue while the parent is closed; old request IDs retain terminal failure, and same- or different-object requests after reopen cannot inherit an earlier backend request.
- Added deterministic coverage for a paused frontend lock, exact PDO/SDO/queue/send ordering, one global SDO step across two axes, multi-cycle cancellation, mailbox-only same/different-object reopen, and parent close/reopen generation changes.

### Review Fix Round 2 Verification

- IgH-disabled CTest: 102/102 passed.
- Python pytest: 28/28 passed.
- ASan/UBSan CTest: 102/102 passed with leak detection enabled.
- Seven ownership, bounded-work, cancellation, and generation tests: 100 repetitions passed in both normal and ASan/UBSan builds.
- Compile-only compatibility passed with official IgH 1.5.4 and 1.6.12 headers. Placeholder archives were used, so this does not claim real IgH linkage or hardware execution.
- Missing IgH development files still fail configuration with the explicit `ecrt.h`/`libethercat` dependency diagnostic.
- A TSan binary built, but this container could not start TSan (`unexpected memory mapping`); ASan/UBSan and deterministic concurrency repetitions are the sanitizer evidence claimed here.

### Remaining Hardware Boundary

IgH cannot forcibly abort an active asynchronous SDO through its public API. If a request never leaves `BUSY`, mailbox-only reopen intentionally remains unavailable until the parent is stopped and the backend is deactivated. Real IgH linkage and 12-axis Elmo HIL remain follow-up validation.
