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
