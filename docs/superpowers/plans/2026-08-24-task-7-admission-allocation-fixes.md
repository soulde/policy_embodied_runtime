# Task 7 Admission and Allocation Fix Plan

**Goal:** Close the cycle-admission race on weakly ordered CPUs and guarantee that every EtherCAT master-cycle SDO failure path remains allocation-free.

**Architecture:** Represent the master's open/closing state and in-flight cycle count in one lock-free 64-bit atomic word. Cycle admission and close then share a single modification order: admission either increments the count while the open bit is set, or observes closing and fails. Replace owning SDO errors with a trivial fixed error code/reason pair; only the non-real-time mailbox status path renders that pair as an owning `Error` string.

**Constraints:** Preserve the master-only backend ownership model, lifecycle generations, bounded round-robin mailbox work, existing public APIs, and IgH 1.5/1.6 compatibility. Do not modify or commit the Elmo XML or `uv.lock`.

## 1. Add deterministic RED coverage

- Add a test peer that holds a synthetic admitted-cycle reference, observes close atomically clear the open bit, and proves a later cycle cannot enter before deactivation.
- Reopen after that boundary test and prove the new lifecycle generation admits cycles normally.
- Model the combined state transitions directly, including admission-before-close, close-before-admission, and refcount-overflow rejection.

## 2. Make SDO progress structurally allocation-free

- Replace `std::optional<Error>` in `SdoTransferProgress` with fixed enums for error code and reason, and assert the progress value is trivially copyable/destructible.
- Store only fixed failure metadata in owner-side mailbox requests. Render human-readable messages when `mailbox_status()` is called outside the master cycle.
- Update the fake and IgH backends so download, upload, cancellation, and exception fallback paths return only fixed progress values.

## 3. Implement single-word admission

- Use one lock-free `std::atomic<std::uint64_t>` containing an open bit and an overflow-checked in-flight count.
- Admit with compare/exchange only while open; close clears the open bit in the same atomic modification order and waits for the count to drain before backend deactivation.
- Keep cycle exit `noexcept`, bounded, nonallocating, and notify close only when the final closing-state reference leaves.

## 4. Verify and report

- Run focused RED/GREEN tests, full CTest and Python suites, ASan/UBSan, repeated concurrency tests, and official IgH header compile checks.
- Run `git diff --check`, verify the protected XML checksum and untracked `uv.lock`, update the task report, stage only intended files, and commit.
