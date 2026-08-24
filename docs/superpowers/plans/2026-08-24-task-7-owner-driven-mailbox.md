# Task 7 Owner-Driven EtherCAT Mailbox Plan

**Goal:** Make the EtherCAT master cycle the sole backend/ecrt owner while preserving a separate mailbox transport frontend, bounded SDO progress, and race-free lifecycle generations.

**Architecture:** Public mailbox calls only stage requests, report status, and manage frontend lifecycle. The frontend publishes one stable request pointer through a lock-free atomic slot. Each master cycle performs PDO receive/process/health/protocol/output work, observes those slots without taking a frontend lock, and advances at most one SDO or cancellation step before queue/send. Parent and child generations reject stale work across close/reopen. IgH cancellation drains an active public-API request from the owner thread because IgH exposes no asynchronous abort primitive.

**Constraints:** Preserve static-mode protection, exact-size IgH downloads, safe unhealthy outputs, and all prior tests. Do not modify or commit the Elmo XML or `uv.lock`.

## 1. Specify the ownership boundary with tests

- Replace tests that expect a mailbox thread to enter the backend with tests proving mailbox `cycle()` performs no backend I/O.
- Add a deterministic paused-frontend-lock test proving the master still completes receive/queue/send and simply defers unstaged SDO work.
- Assert each master cycle advances no more than one SDO/cancel step and that the step occurs after PDO staging but before queue/send.

## 2. Add lifecycle and cancellation tests

- Test mailbox-only close while an SDO is pending, owner-driven cancellation acknowledgement, and clean reopen for both the same and a different object.
- Test that a child cannot reopen while its parent is closed and can reopen only under the parent's new active generation.
- Retain terminal status for old request IDs and prove stale completions cannot affect new requests.

## 3. Implement owner-driven SDO progress

- Remove backend arbitration and every mailbox-thread backend call.
- Add a shared parent-generation state and lock-free atomic request slot from each mailbox to the master, with round-robin selection across axes.
- Preallocate upload storage on the frontend and let backend upload progress copy into caller-provided storage to avoid normal-path allocation in the cyclic owner.
- Add one-step backend cancellation progress; IgH polls active request completion before clearing its per-slave state.

## 4. Verify and report

- Run focused RED/GREEN tests, all CTest, Python pytest, ASan/UBSan, concurrency repetitions, and official IgH header compile checks.
- Run `git diff --check`, verify the XML checksum and untracked `uv.lock`, update the task report, stage only intended files, and commit.
