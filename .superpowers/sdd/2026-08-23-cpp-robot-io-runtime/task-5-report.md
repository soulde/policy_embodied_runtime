# Task 5 Report: Versioned Unix Socket and memfd IPC

## Implementation summary

- Added fixed-layout, version-1 Linux IPC records for `AxisCommand`,
  `AxisFeedback`, `BusHealth`, `IpcHeader`, and the two-descriptor setup
  message. Compile-time size, alignment, offset, standard-layout, trivial-copy,
  64-bit, little-endian, and x86_64/AArch64 checks make the supported ABI
  explicit.
- Added dynamic 0–12-axis `SnapshotRegion<T>` mappings. Mapping arithmetic uses
  checked multiplication, addition, and cache-line rounding; topology and file
  size are frozen after setup.
- Implemented two cache-line-aligned snapshot slots. The single writer marks
  the inactive slot with an odd guard, stores metadata and payload through
  lock-free atomic words, closes the slot with an even guard, then publishes
  its active index. All participating operations use `memory_order_seq_cst`,
  which gives the guard and atomic payload words one total order and prevents a
  reader that observes an old even guard from accepting newer payload stores.
  Reads retry at most eight times and return a fixed-capacity 12-axis value;
  successful reads and publishes allocate nothing.
- Runtime startup rejects mappings when the shared 32-bit or 64-bit atomics are
  not lock-free. No pointer, `bool`, `std::string`, `std::vector`, owning dynamic
  type, or non-trivially-copyable record is stored in shared memory.
- Added `RobotIoIpcServer`, which consumes a connected Unix stream or seqpacket
  socket, creates two `MFD_CLOEXEC | MFD_ALLOW_SEALING` memfds, sizes/maps and
  initializes them, freezes their sizes with seals, and sends both descriptors
  with `SCM_RIGHTS` and a versioned setup record.
- Added `RobotIoClient`, which receives descriptors with `MSG_CMSG_CLOEXEC`,
  validates ancillary framing, closes all received descriptors on every error
  path, checks file type/size/seals, maps both regions, and validates magic,
  version, generation, axis count, record stride, region kind, slot stride, and
  total mapping size before exposing them.
- Both endpoints are move-only and own their socket, memfds, and mappings.
  Explicit `close()` continues through `munmap()`/`close()` failures and reports
  the first error; destructors perform non-throwing best-effort cleanup. Client
  operations reject a disconnected daemon instead of continuing on an orphaned
  generation.
- Committed the scoped implementation as `2a0e44e Add versioned robot I/O IPC`.

## RED and GREEN evidence

The integration target and tests were added before the new IPC headers. The
first build failed for the intended missing production interface:

```text
fatal error: policy_runtime/robot_io/ipc_protocol.hpp: No such file or directory
```

The first concurrent stress run exposed a bad sequence-0 test fixture: its
targets were all zero while the independently derived invariant expected axis
values 0–11. Correcting the fixture made the test pass without changing the
publication protocol.

Self-review then added a real `SOCK_STREAM` descriptor-transfer case before
stream support. It failed with the intended restriction:

```text
IPC socket must be AF_UNIX SOCK_SEQPACKET
```

After adding stream-safe setup framing (`MSG_WAITALL` receive and partial-send
completion), both stream and seqpacket tests pass.

## Test coverage

`tests/cpp/integration/ipc_test.cpp` exercises:

- complete snapshot publication and an allocation-free fixed-capacity read;
- a 25,000-publication, 12-axis writer/reader stress invariant;
- magic, ABI version, generation, axis count, stride, and region-kind rejection;
- 0/12-axis bounds and oversized-axis rejection;
- real Unix stream `SCM_RIGHTS` exchange, CLOEXEC, seals, and command/feedback
  round trips;
- stale generation rejection during a real setup handshake;
- a forked cross-process seqpacket/memfd publication;
- peer disconnect detection and rejection of post-disconnect commands.

The 12-axis stress test also passed 20 consecutive direct repetitions.

## Verification

```bash
uv run --with cmake cmake --build --preset default
uv run --with cmake ctest --preset default --output-on-failure
.venv/bin/pytest -q
```

Results: CTest passed 53/53; Python pytest passed 28/28.

```bash
uv run --with cmake cmake -S . -B build/asan-ubsan \
  -DCMAKE_BUILD_TYPE=Debug -DPOLICY_RUNTIME_BUILD_TESTS=ON \
  '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
uv run --with cmake cmake --build build/asan-ubsan
ASAN_OPTIONS=detect_leaks=1 uv run --with cmake ctest --test-dir build/asan-ubsan \
  -R ipc_test --output-on-failure
```

Results: ASan/UBSan IPC suite passed 8/8 with leak detection enabled.

```bash
uv run --with cmake cmake --preset tsan
uv run --with cmake cmake --build --preset tsan
build/tsan/ipc_test --gtest_filter=IpcTest.SnapshotStressNeverReturnsMixedAxes
```

TSan configuration and compilation/linking succeeded. The instrumented binary
cannot start in this container and exits before GoogleTest or IPC code runs:

```text
FATAL: ThreadSanitizer: unexpected memory mapping
```

## Self-review and concerns

- Reviewed all create/receive/mapping failure paths for ownership transfer and
  cleanup, including malformed/truncated ancillary messages and excess received
  descriptors.
- Confirmed command and feedback directionality is exposed only through the
  intended endpoint operations: host publishes commands and reads feedback;
  daemon reads commands and publishes feedback.
- Confirmed no Python, simulator, or ESI file was modified. The pre-existing
  untracked `Elmo+ECAT+00010420+V12.xml` remains untouched and unstaged.
- This ABI intentionally rejects non-Linux, non-little-endian, non-64-bit,
  non-x86_64/AArch64 builds. TSan still needs execution on a compatible host;
  its runtime failure here is the only sanitizer limitation.

## Review Fix Round 1

### RED evidence

Tests were first changed to require strict seqpacket framing, publication
sequence/timestamp control metadata, reader/writer capability views, immutable
topology checks on every public operation, malformed ancillary cleanup,
cross-process slot reuse, and moved/disconnected server invalidation. The
focused target then failed at compile time on the deliberately missing API:

```text
error: 'SnapshotMappingAccess' has not been declared
error: 'class policy_runtime::SnapshotRegion<...>' has no member named 'writer'
error: 'struct policy_runtime::detail::PublicationControl' has no member named
       'published_sequence'
```

### Fix implementation

- Both setup endpoints now accept only connected `AF_UNIX SOCK_SEQPACKET`
  sockets. Setup is one fixed 64-byte `sendmsg`/`recvmsg` record carrying
  exactly one `SCM_RIGHTS` item with two descriptors. Ancillary storage is
  explicitly `cmsghdr`-aligned with compile-time alignment and
  `CMSG_SPACE(2 * sizeof(int))` size checks. The receiver rejects payload or
  control truncation, excess rights, non-rights control messages, and duplicate
  rights messages while closing every descriptor delivered on failure.
- `PublicationControl` now records the sequence and nonnegative monotonic
  timestamp associated with the active slot. A writer completes the inactive
  slot, closes its guard, publishes sequence/timestamp metadata, and finally
  changes the active index. A reader accepts only stable guard, active index,
  and control/slot metadata observations. `(0, 0)` is explicitly valid for the
  initial publication; later sequence and timestamp values are independently
  nondecreasing. All new shared atomics participate in the lock-free startup
  check.
- `SnapshotReader<T>` and `SnapshotWriter<T>` expose only the operation valid
  for their capability. Public raw endpoint FD accessors were removed. Backing
  regions are initialized through a temporary read/write mapping, then the
  daemon maps commands read-only and feedback read/write. The host receives a
  command `O_RDWR` description and feedback `O_RDONLY` description, validates
  those kernel access modes, and maps feedback with `PROT_READ` only.
- Every snapshot read/publish validates the immutable header topology before
  deriving or accessing slots. Every server read/publish checks socket
  liveness first. Server and client moves exchange sizes/identity fields and
  clear all source descriptors, mappings, views, dimensions, generations, and
  setup state; close does the same invalidation.
- Stress assertions now require at least one successful read and validate
  control sequence/timestamp plus every axis record's sequence, timestamp,
  target, flags, and reserved fields. A forked writer performs 20,000
  publications while its peer concurrently reads and validates both reused
  slots. Tests also cover truncated payloads, excess rights and FD cleanup,
  stream rejection, topology mutation, decreasing metadata, moved-from state,
  disconnect liveness, and move-after-close.

### Verification

```bash
uv run --with cmake cmake --preset default
uv run --with cmake cmake --build --preset default --target ipc_test
uv run --with cmake ctest --preset default -R '^ipc_test\.' --output-on-failure
uv run --with cmake cmake --build --preset default
uv run --with cmake ctest --preset default --output-on-failure
.venv/bin/pytest -q
./build/ipc_test \
  --gtest_filter='IpcTest.SnapshotStressNeverReturnsMixedAxes:IpcTest.ReusesSlotsWithoutTearingAcrossProcesses' \
  --gtest_repeat=20 --gtest_break_on_failure
git diff --check
```

Results: focused IPC CTest passed 13/13; full CTest passed 58/58;
Python pytest passed 28/28; both stress cases passed all 20 repetitions (40
stress executions total); `git diff --check` produced no output.

```bash
uv run --with cmake cmake -S . -B build/asan-ubsan \
  -DCMAKE_BUILD_TYPE=Debug -DPOLICY_RUNTIME_BUILD_TESTS=ON \
  '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
uv run --with cmake cmake --build build/asan-ubsan --target ipc_test -j2
ASAN_OPTIONS=detect_leaks=1 uv run --with cmake ctest \
  --test-dir build/asan-ubsan -R '^ipc_test\.' --output-on-failure
```

ASan/UBSan passed all 13 IPC tests with leak detection enabled.

```bash
uv run --with cmake cmake --preset tsan
uv run --with cmake cmake --build --preset tsan --target ipc_test -j2
./build/tsan/ipc_test \
  --gtest_filter='IpcTest.SnapshotStressNeverReturnsMixedAxes:IpcTest.ReusesSlotsWithoutTearingAcrossProcesses'
```

TSan configuration and compilation/linking passed. Execution again stopped
before GoogleTest or IPC code ran with the container limitation:

```text
FATAL: ThreadSanitizer: unexpected memory mapping
```

The deferred theoretical 64-bit epoch wraparound ABA and non-retryable
`munmap()` cleanup concerns remain unchanged. The pre-existing untracked
`Elmo+ECAT+00010420+V12.xml` remains untouched and unstaged.

## Review Fix Round 2

### RED evidence

Tests were changed first to require endpoint-only read/publish access,
non-default-constructible and noncopyable snapshot views, fallible
access-gated view creation, a first sequence-zero command with a real nonzero
monotonic timestamp, daemon transfer-capability closure, and unexpected socket
packet rejection before any mapping access. The focused target failed at the
intended missing contracts:

```text
error: static assertion failed
  static_assert(!HasCommandWriterAccessor<RobotIoClient>);
error: static assertion failed
  static_assert(!std::is_copy_constructible_v<SnapshotReader<AxisCommand>>);
error: 'class SnapshotWriter<AxisCommand>' has no member named 'has_value'
error: no matching function for call to 'SnapshotRegion<AxisCommand>::attach(...,
       SnapshotMappingAccess)'
```

### Fix implementation

- Removed all public server/client snapshot-view accessors. Endpoint users can
  only read or publish through methods that first check socket health and then
  invoke view-level topology/generation validation. `SnapshotReader<T>` and
  `SnapshotWriter<T>` now have private region constructors, no default
  constructor, deleted copy operations, and invalidating move operations.
  Endpoint-owned views are private optionals, moved together with their mmap
  ownership, reset in moved-from objects, and reset before unmapping on close or
  destruction. Compile-time concepts and type-trait assertions protect those
  contracts; runtime tests cover server and client move, close, and
  move-after-close behavior.
- `SnapshotRegion::attach()` now requires the mapping access role. `reader()`
  and `writer()` are fallible factories that validate immutable topology;
  `writer()` additionally rejects every region not attached/initialized as
  read/write. The daemon mapping helper derives `mmap()` protection from the
  same access value passed to `attach()`, preventing role/protection drift. A
  real `PROT_READ` test mapping cannot mint a writer.
- Replaced `MSG_PEEK` liveness with zero-timeout `poll()` using `POLLIN`,
  `POLLRDHUP`, `POLLHUP`, `POLLERR`, and `POLLNVAL`, retrying `EINTR`.
  Any post-setup `POLLIN` packet is consumed and rejected as a protocol error;
  queued packets are drained before returning so a packet followed by peer
  closure cannot appear healthy. Both server and client tests send an
  unexpected packet, close the peer, and verify check/read/publish operations
  return only protocol/unavailable errors.
- After a successful atomic setup send, the daemon closes both transfer-only
  file descriptions. It retains only the command `O_RDONLY` reader description
  and feedback `O_RDWR` writer description. `/proc/self/fd` plus `F_GETFL`
  assertions verify exact descriptor counts and access modes before setup,
  after setup, after client receipt, and after both endpoint closes.
- Removed the sequence-zero/nonzero-timestamp restriction. The inactive-index
  sentinel already identifies an uninitialized region; monotonicity checks
  still apply independently to sequence and timestamp after the first
  publication. Both a direct snapshot test and the real endpoint command round
  trip publish sequence zero with a nonzero monotonic timestamp.

### Verification

```bash
uv run --with cmake cmake --preset default
uv run --with cmake cmake --build --preset default --target ipc_test
uv run --with cmake ctest --preset default -R '^ipc_test\.' --output-on-failure
uv run --with cmake cmake --build --preset default
uv run --with cmake ctest --preset default --output-on-failure
.venv/bin/pytest -q
./build/ipc_test \
  --gtest_filter='IpcTest.SnapshotStressNeverReturnsMixedAxes:IpcTest.ReusesSlotsWithoutTearingAcrossProcesses:IpcTest.UnexpectedPacketThenDisconnectCannotMaskEndpointFailure' \
  --gtest_repeat=20 --gtest_break_on_failure
git diff --check
```

Results: focused IPC CTest passed 16/16; full CTest passed 61/61; Python
pytest passed 28/28; both concurrency tests and the packet/disconnect test
passed all 20 repetitions (60 repeated executions); `git diff --check`
produced no output.

```bash
uv run --with cmake cmake -S . -B build/asan-ubsan \
  -DCMAKE_BUILD_TYPE=Debug -DPOLICY_RUNTIME_BUILD_TESTS=ON \
  '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
uv run --with cmake cmake --build build/asan-ubsan --target ipc_test -j2
ASAN_OPTIONS=detect_leaks=1 uv run --with cmake ctest \
  --test-dir build/asan-ubsan -R '^ipc_test\.' --output-on-failure
```

ASan/UBSan passed all 16 IPC tests with leak detection enabled.

```bash
uv run --with cmake cmake --preset tsan
uv run --with cmake cmake --build --preset tsan --target ipc_test -j2
./build/tsan/ipc_test \
  --gtest_filter='IpcTest.SnapshotStressNeverReturnsMixedAxes:IpcTest.ReusesSlotsWithoutTearingAcrossProcesses:IpcTest.UnexpectedPacketThenDisconnectCannotMaskEndpointFailure'
```

TSan configuration and compilation/linking passed. Execution stopped before
GoogleTest or IPC code ran with the unchanged container limitation:

```text
FATAL: ThreadSanitizer: unexpected memory mapping
```

The deferred theoretical 64-bit epoch wraparound ABA and non-retryable
`munmap()` cleanup concerns remain unchanged. The pre-existing untracked ESI
XML remains untouched and unstaged.

## Review Fix Round 3

### RED evidence

Two regressions were added before production changes. The first queued an
oversized seqpacket followed by a one-byte marker in each direction and
required one peer check to consume only the first complete packet. The second
prefilled the receive queue, ran eight nonblocking flooding senders for a
bounded 100 ms window, and required `read_commands()` to return within 50 ms.
Both failed against `2575089` for the intended drain-loop behavior:

```text
[  FAILED  ] IpcTest.PeerCheckConsumesOnlyOneCompleteUnexpectedPacket
Expected: recv(..., MSG_DONTWAIT) == 1
  Actual: -1

[  FAILED  ] IpcTest.FloodingPeerCannotDelayEndpointOperation
Expected: elapsed < 50 ms
  Actual: approximately 100 ms
```

### Fix implementation

- Daemon and client peer checks still use zero-timeout `poll()` and retry
  interrupted syscalls, but no longer drain until `EAGAIN`.
- On `POLLIN`, each check performs at most one nonblocking `recvmsg()` and
  immediately returns `ErrorCode::protocol` after consuming that one packet.
- `MSG_TRUNC` is supplied to `recvmsg()`: Linux returns the original packet
  length while `SOCK_SEQPACKET` consumes the entire packet atomically even when
  it is larger than the fixed discard buffer. The 4096-byte regression packet
  is therefore discarded completely, and the second marker packet remains
  queued for the next check.
- Normal `POLLRDHUP`/`POLLHUP`/`POLLERR`/`POLLNVAL` disconnect handling and
  unexpected-packet-followed-by-disconnect behavior remain intact.

### Verification

```bash
uv run --with cmake cmake --preset default
uv run --with cmake cmake --build --preset default --target ipc_test
uv run --with cmake ctest --preset default -R '^ipc_test\.' --output-on-failure
uv run --with cmake cmake --build --preset default
uv run --with cmake ctest --preset default --output-on-failure
.venv/bin/pytest -q
./build/ipc_test \
  --gtest_filter='IpcTest.SnapshotStressNeverReturnsMixedAxes:IpcTest.ReusesSlotsWithoutTearingAcrossProcesses:IpcTest.UnexpectedPacketThenDisconnectCannotMaskEndpointFailure:IpcTest.PeerCheckConsumesOnlyOneCompleteUnexpectedPacket:IpcTest.FloodingPeerCannotDelayEndpointOperation' \
  --gtest_repeat=20 --gtest_break_on_failure
git diff --check
```

Results: focused IPC CTest passed 18/18; full CTest passed 63/63; Python
pytest passed 28/28; the two concurrency and three protocol/liveness cases
passed all 20 repetitions (100 repeated executions); `git diff --check`
produced no output.

```bash
uv run --with cmake cmake -S . -B build/asan-ubsan \
  -DCMAKE_BUILD_TYPE=Debug -DPOLICY_RUNTIME_BUILD_TESTS=ON \
  '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
uv run --with cmake cmake --build build/asan-ubsan --target ipc_test -j2
ASAN_OPTIONS=detect_leaks=1 uv run --with cmake ctest \
  --test-dir build/asan-ubsan -R '^ipc_test\.' --output-on-failure
```

ASan/UBSan passed all 18 IPC tests with leak detection enabled.

```bash
uv run --with cmake cmake --preset tsan
uv run --with cmake cmake --build --preset tsan --target ipc_test -j2
./build/tsan/ipc_test \
  --gtest_filter='IpcTest.SnapshotStressNeverReturnsMixedAxes:IpcTest.ReusesSlotsWithoutTearingAcrossProcesses:IpcTest.FloodingPeerCannotDelayEndpointOperation'
```

TSan configuration and compilation/linking passed. Execution stopped before
GoogleTest or IPC code ran with the unchanged container limitation:

```text
FATAL: ThreadSanitizer: unexpected memory mapping
```

The deferred theoretical 64-bit epoch wraparound ABA and non-retryable
`munmap()` cleanup concerns remain unchanged. The pre-existing untracked ESI
XML remains untouched and unstaged.
