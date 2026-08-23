# Task 4 Report: Transport Capabilities and Scheduling

## Implementation summary

- Added a small `Transport` base interface with lifecycle, health, scheduling,
  and cycle operations. Its contract states that only `cycle()` may perform
  physical I/O.
- Added independently mockable `FrameTransport`, `CyclicTransport`, and
  `ObjectDictionaryTransport` capabilities. Frame writes and OD downloads are
  staged; reads/uploads expose data received during a cycle.
- Added a non-owning `TransportScheduler` that maps each scheduling class to a
  distinct executor identity without creating production threads. This keeps
  blocking work from sharing the hard-real-time execution context.
- Added default and ThreadSanitizer CMake presets. GoogleTest discovery is
  deferred to CTest so a successful sanitizer build is not reported as a build
  failure solely because the environment cannot execute TSan binaries.

## Scope extension: `Result<void>`

The approved `Transport::open()` interface requires `Result<void>`, but the
existing generic template cannot instantiate `std::variant<void, Error>`. I
added a specialization backed by `std::variant<std::monostate, Error>` without
changing `Result<T>` semantics. It has focused success and failure/error tests
and makes no allocation beyond `Error.message` storage.

## RED and GREEN evidence

The scheduler-isolation test was added before its interface. The focused build
failed for the intended missing production header:

```text
fatal error: policy_runtime/robot_io/transport_scheduler.hpp: No such file or directory
```

The approved `Result<void>` tests were then added first. They failed because
the generic template formed `std::variant<void, Error>`, including:

```text
error: invalid parameter type 'void'
static assertion failed: variant must have no void alternative
```

After the minimal specialization and transport/scheduler implementation, the
focused checks passed:

```bash
uv run --with cmake cmake --build build --target policy_runtime_unit_tests
uv run --with cmake ctest --test-dir build -R 'result_test\\.' --output-on-failure
# 3/3 passed

uv run --with cmake cmake --build build --target transport_scheduler_test
uv run --with cmake ctest --test-dir build -R transport_scheduler_test --output-on-failure
# 1/1 passed
```

## Verification

```bash
uv run --with cmake cmake --preset default
uv run --with cmake cmake --build --preset default
uv run --with cmake ctest --preset default --output-on-failure
uv run pytest
git diff --check
```

Results: default CMake configure/build succeeded; CTest passed 42/42; Python
pytest passed 28/28; `git diff --check` produced no output.

The system `cmake` is 3.22.1, below the project minimum 3.24, and failed at
`cmake_minimum_required`. Verification therefore uses
`uv run --with cmake cmake` (CMake 4.4.2); the presets declare the same 3.24
minimum as the project.

## ThreadSanitizer

```bash
uv run --with cmake cmake --preset tsan
uv run --with cmake cmake --build --preset tsan
uv run --with cmake ctest --preset tsan -R transport_scheduler_test --output-on-failure
```

TSan configuration and compilation/linking completed successfully, including
`transport_scheduler_test`. CTest could not run the binaries because this
container reports `FATAL: ThreadSanitizer: unexpected memory mapping`; this is
an environment runtime limitation, not a configure or build defect. Deferring
GoogleTest discovery to CTest preserves that distinction.

## Self-review and concerns

- The transport capability headers have no dependency on policy/robot runtime
  logic; an EtherCAT implementation can combine cyclic and object-dictionary
  capabilities while serial can expose only frame capability.
- The scheduler owns neither transports nor threads, avoiding lifecycle and
  blocking-I/O behavior outside this task's scope.
- The pre-existing untracked `Elmo+ECAT+00010420+V12.xml` ESI file was left
  untouched and is not staged. No Python or simulator files were changed.
- TSan execution should be re-run on a host whose address-space layout supports
  the GNU ThreadSanitizer runtime.

## Review Fix Round 1

### Root cause and RED

`TransportScheduler` stored non-owning raw addresses but used
`insert_or_assign`, so a duplicate `add()` could silently replace its executor
assignment and no API removed stale addresses before placement-address reuse.
The OD capability also described its work as cycle-driven, which cannot express
potentially blocking EtherCAT mailbox operations independently of a hard
real-time PDO cycle.

Focused tests were added first for exact executor assignments, duplicate
rejection, removal-before-address-reuse, and queued mailbox completion/error
states. The scheduler/transport test target then failed because the required
API did not yet exist, including:

```text
error: 'MailboxSchedulingClass' in namespace 'policy_runtime' does not name a type
error: 'class policy_runtime::TransportScheduler' has no member named 'remove'
```

### Green implementation

- `add()` now rejects every duplicate registration with
  `ErrorCode::invalid_argument`, preserving the original executor identity.
- `remove()` erases the non-owning assignment and rejects an unregistered
  transport. The public contract requires a transport to outlive its
  registration and be removed before destruction, making clean address reuse
  explicit without transferring ownership to the scheduler.
- `ObjectDictionaryTransport` now has a non-real-time
  `MailboxSchedulingClass` restricted to blocking-event-driven or asynchronous
  execution, queued upload/download request IDs, queryable snapshot states for
  queued/completed/failed requests, and a distinct `service_mailbox()` method. PDO I/O
  remains in `cycle()`; only the separately scheduled mailbox service may do
  mailbox physical I/O.

### Verification

```bash
uv run --with cmake cmake --build --preset default --target transport_scheduler_test
uv run --with cmake ctest --preset default -R 'transport_scheduler_test|ObjectDictionaryTransportTest' --output-on-failure
uv run --with cmake cmake --build --preset default --target policy_runtime_unit_tests
uv run --with cmake ctest --preset default -R 'result_test\\.' --output-on-failure
uv run --with cmake cmake --build --preset default
uv run --with cmake ctest --preset default --output-on-failure
uv run pytest
git diff --check
```

Results: focused scheduler/transport CTest passed 4/4; focused result CTest
passed 3/3; full CTest passed 45/45; Python pytest passed 28/28; `git diff
--check` produced no output.

TSan configure and build passed with the updated sources. Its CTest invocation
again stopped during GoogleTest discovery with `FATAL: ThreadSanitizer:
unexpected memory mapping`, which is the known container runtime limitation,
not a compile or link failure.
