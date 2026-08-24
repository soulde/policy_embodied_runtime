# Task 8: Robot I/O Daemon, Real-Time Loop, and Safety

## Result

Implemented `robot-io-daemon` as the sole cyclic EtherCAT owner with a fixed-capacity, allocation-free device cycle for up to twelve statically configured CiA 402 axes. The daemon consumes versioned shared-memory command snapshots, stages protocol outputs, publishes feedback, and exposes lock-free health metrics.

The 1 kHz loop uses absolute `CLOCK_MONOTONIC` deadlines. Startup checks PREEMPT_RT, calls `mlockall(MCL_CURRENT | MCL_FUTURE)`, pins the owner thread, and requests `SCHED_FIFO`; an unsupported or unprivileged host starts in a degraded state with `realtime_guarantee=false`.

## Safety Behavior

- Heartbeat expiry, malformed/out-of-order commands, link/process-image loss, consecutive WKC failures, slave OP loss, static-mode mismatch, drive faults, and following error enter the bounded safe-stop path.
- Safety groups propagate only the relevant failure. On a partial slave loss, an independent operational group can continue until its own condition or the configured WKC threshold stops it.
- Target validation matches CiA 402 order: engineering limit, per-cycle slew limit, then following-error comparison.
- Fault-reset requests are bounded per fault episode.
- Stop/SIGTERM rejects new commands, commands Quick Stop, confirms zero velocity for a bounded number of cycles (or reaches a timeout), then commands Disable before closing the master.

## TDD and Review Evidence

Initial tests failed because the daemon interfaces did not exist. Later red tests reproduced a false following-error trip caused by checking the unslewed target, and whole-bus output suppression after one slave left OP. Both regressions now pass at the daemon/master boundary. The cycle is `noexcept`, uses preallocated axis/PDO/feedback arrays, avoids socket polling, locks, logging, parsing, and dynamic allocation, and has an explicit 100-cycle allocation guard across twelve axes.

## Verification

- Normal build and complete CTest: 122/122 passed.
- Python compatibility suite: 28/28 passed.
- Focused daemon suite: 17/17 passed and all seventeen tests passed 50 consecutive repetitions.
- ASan/UBSan build and focused suite: 17/17 passed.
- TSan-instrumented build: completed successfully. Test discovery could not start any instrumented binary in this container and reported `ThreadSanitizer: unexpected memory mapping`; no TSan runtime result is claimed.
- `git diff --check`: clean.

## Carry-Forward

Real-time guarantees still require qualification on the target ARM64/x86-64 PREEMPT_RT host with a real IgH master and Elmo drives. Separate executor threads for future blocking/event transports remain service-integration work; Task 8 assigns and owns the EtherCAT hard-real-time cycle without introducing those transports.

## Review Fix Round 1

- Fault-reset limits now count actual authorized `0x0080` edges for the entire persistent fault episode. Disable/unknown states do not replenish the budget, an observed valid non-fault state does, and peer axes remain stopped while a faulted group member resets.
- Late releases resynchronize to the first future absolute deadline and record skipped releases. EINTR is retried; other sleep errors are published as a timing fault and drive the daemon through safe stop. Wake latency and cycle execution time are reported separately.
- Real-time setup is performed by the thread that owns `run()`. Partial affinity/FIFO/mlock setup is rolled back, transport startup failures clean their scheduler/handler state, and teardown keeps cycle ownership until after master deactivation.
- Commands cross from non-RT callers through a fixed atomic double-slot handoff. Feedback and axis requests are published as one lock-free snapshot, removing races between policy/control threads and the EtherCAT owner without adding RT locks or allocation.
- The signal test now has `run()` consume SIGTERM and observes a final Disable PDO send before backend deactivation. Allocation interception covers regular, nothrow, array, and aligned C++ allocation; the combined IPC/EtherCAT 12-axis cycle guard exercises the complete RT data path.

### Fix Verification

- Fresh normal build of all targets: passed.
- CTest without sandbox-forbidden socket cases: 112/112 passed. The restricted sandbox blocks all 18 `ipc_test` cases plus the two daemon IPC cases at socket/`SO_DOMAIN` operations; no result is claimed for those 20 cases in this run.
- Python without its two socket-dependent ZMQ cases: 26/26 passed. Both excluded cases fail at socket creation with `EPERM` in this sandbox.
- ASan/UBSan focused daemon suite without the two IPC cases: 25/25 passed with leak detection disabled because LeakSanitizer cannot operate under the container's ptrace policy.
- Controller follow-up outside the socket-restricted sandbox used the pinned CTest 4.4.2 and passed both daemon IPC cases, 2/2 (`IntegratesVersionedIpcSequencesAndTimestamps` and `FullIpcEthercatCycleDoesNotAllocateAfterStart`).
- Seven timing, fault-reset, SIGTERM, owner-thread, and concurrent-handoff regressions: 350/350 passed across 50 repetitions.
- TSan-instrumented focused target: built. Runtime remains unavailable in this container (`ThreadSanitizer: unexpected memory mapping`), so no TSan execution result is claimed.
- `git diff --check`: clean.

## Review Fix Round 2

- Fault reset is never forwarded to the CiA 402 state machine outside the
  resettable Fault state. A request made before a fault therefore cannot arm
  the edge latch, and the configured budget is spent only when the axis can
  emit the `0x0080` control-word pulse.
- `run()`, `cycle()`, `process_device_cycle()`, and command refresh now share
  one CAS-acquired RAII cycle-owner token. Competing public entries return
  without touching the backend, process image, axes, or safety state; the
  EtherCAT callback runs only on the admitted owner thread.
- Rejected command publications carry a sticky monotonic publication number.
  The RT owner acknowledges the rejection before any later valid snapshot. If
  a racing valid snapshot was consumed first, it is replayed on the following
  cycle, preserving both the required safe-stop cycle and bounded recovery.
- Shutdown is one-shot. Once stop or terminal shutdown is observed, `start()`
  rejects permanently and cannot reactivate the EtherCAT backend.

### Round 2 Verification

- Five initial regression tests reproduced all four findings and passed 0/5
  before the fixes. A concurrent producer/RT-owner test then exposed and drove
  the valid-snapshot replay fix.
- Fresh normal build of all targets: passed.
- CTest without the 20 sandbox-forbidden socket cases: 118/118 passed.
- Python without the two socket-dependent ZMQ cases: 26/26 passed.
- ASan/UBSan daemon suite without the two IPC cases: 31/31 passed with leak
  detection disabled for the container ptrace restriction.
- Six fault-reset, admission, sticky-rejection, concurrency, and lifecycle
  regressions: 300/300 passed across 50 repetitions.
- TSan-instrumented target: built. Runtime remains unavailable in this
  container (`ThreadSanitizer: unexpected memory mapping`), so no TSan runtime
  result is claimed.
- `git diff --check`: clean.
