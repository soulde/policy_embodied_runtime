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
