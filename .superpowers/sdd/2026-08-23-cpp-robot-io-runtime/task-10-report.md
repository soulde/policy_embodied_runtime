# Task 10: Serial and ST3215 Daemon Ownership

## Result

Implemented the native C++ serial transport, ST3215 wire codec and stream
parser, daemon-owned servo abstraction, and static multi-device registry. One
blocking executor owns each physical serial bus, so multiple servos share one
open port and all byte-stream I/O is serialized outside the EtherCAT realtime
cycle. The realtime daemon path only reads fixed-size lock-free feedback and
safety snapshots.

The serial transport uses raw `termios`, restores the original port settings,
guards against duplicate opens of the same physical device, and bounds partial
writes and reads with EINTR/EAGAIN-aware polling. Its fixed-capacity transmit,
receive, and stream buffers reject oversized frames and expose timeout,
framing, queue, I/O, and lifecycle health without allocating in a cycle.

The ST3215 implementation matches the Python checksum, little-endian command,
status, and radians conversion contract. It supports exact goal-position and
present-position exchanges, malformed-frame recovery, command sequencing,
stale feedback, device errors, and bounded shutdown. Profile loading pairs one
sensor and actuator per physical servo, freezes port configuration, and rejects
duplicate IDs, inconsistent bus settings, missing identities, unsafe limits,
and more than 32 servos.

Daemon integration starts and rolls back serial buses with the other static
transports. Serial timeout/protocol/I/O/stale state propagates through fixed
safety-group masks to Quick Stop associated CiA402 axes and appears in daemon
health, without moving any serial syscall onto the 1 kHz owner.

Committed as `cb09c68` (`Move serial robot I/O into daemon`).

## TDD Evidence

The initial protocol and PTY targets failed to compile because the ST3215 and
serial APIs did not exist. Red tests then drove the Python golden frames,
malformed length/header/checksum handling, incremental resynchronization,
radians parity, physical-port sharing, profile derivation and rejection,
daemon safety propagation, and PTY partial reads/timeouts/termios restoration.

Self-review added a transport-exception regression: the first version let an
exception escape the serial executor and terminate the process. The executor
now contains all transport/protocol exceptions, publishes stale I/O feedback,
and remains stoppable. A final real-PTY test exercises two concurrent servo
publishers on one bus, injects a bad checksum before a valid response, verifies
recovery without frame interleaving, and bounds shutdown.

## Verification

- Fresh all-target normal build: passed.
- Full CTest with exactly the 13 sandbox-forbidden pre-existing IPC socket
  cases excluded: 162/162 passed. Those exclusions are unchanged from the Task
  9 baseline (`getsockopt(SO_DOMAIN)` is denied by this sandbox).
- Focused ST3215 and virtual-serial suite: 12/12 passed; all 12 tests passed 25
  consecutive repetitions (300/300 executions).
- Python ST3215, robot I/O, and robot-profile compatibility tests: 9/9 passed.
- ASan/UBSan ST3215, PTY/daemon, and profile coverage: 14/14 passed with leak
  detection disabled because LeakSanitizer cannot run under the container's
  ptrace policy. The sanitizer build reused the already-fetched nlohmann-json
  source because outbound downloads are blocked.
- `git diff --check`: clean.

## Fix Round 3

Closed the remaining mixed-command partial-commit window. The daemon now reads
and validates axis and servo snapshots in one owner cycle, previews axis safety
on a stack copy, stages the complete servo batch, and commits the axis state,
heartbeat, and servo batch only after every validation succeeds. A malformed or
stale servo record leaves both sides uncommitted and enters the invalid-command
safety path; a newer epoch recovers atomically on the following cycle. The old
delayed servo commit in the process cycle was removed.

### Fix-round verification

- Full pinned CMake/CTest: 199/199 passed.
- Python: 28/28 passed.
- ASan/UBSan focused suite: 86/86 passed.
- Mixed transaction, heartbeat, and ST-only IPC regressions: 25 repetitions
  passed.
- Final review: no Critical, Important, or Minor findings.

## Carry-Forward

Real USB serial adapters, mixed EtherCAT/ST3215 hardware, disconnect/reconnect
behavior, electrical bus turnaround, and target-kernel scheduling remain HIL
qualification. The PTY suite covers the daemon/transport ownership boundary,
wire framing, timeout isolation, shared-port concurrency, error recovery, and
shutdown deterministically in this environment.

## Fix Round 1

Completed the servo IPC/RuntimeHost and serial-safety review pass. The daemon
keeps ABI-v1 for axis-only profiles and uses the four-descriptor ABI-v2 layout
when ST3215 servos are present. The runtime host now binds ST3215 feedback and
commands in frozen profile order; the dedicated ST3215-only daemon/IPC test
exercises that zero-axis topology on environments allowed to create Unix
sockets.

The serial shutdown wakeup, bounded timeout quarantine, fault-latched safety
input, TIOCEXCL lifecycle, and command-freshness behavior were retained and
verified. The exclusion regression now forks a child that opens the PTY
directly, proving the kernel TIOCEXCL claim rather than inheriting the
parent's process-local duplicate-open registry; it also proves that close
releases the claim. Queue saturation reports the exact transmit-queue error
and degraded health. Loader coverage proves Python-compatible omitted and
explicit `timeout_s=0` behavior, and now rejects ST3215 command/future windows
above one second before any duration conversion can overflow.

### Fix-round verification

- Pinned CMake 4.4.2 all-target build: passed.
- Full CTest with the exact 15 sandbox-forbidden Unix-socket tests excluded:
  175/175 passed. Those tests fail before their handshakes because the sandbox
  denies `getsockopt(SO_DOMAIN)`; run them on a controller with Unix socket
  permissions.
- ST3215 plus non-socket PTY focused tests: 25 consecutive repetitions passed.
- Python suite: 26 passed with two AF_INET ZMQ tests deselected; the two tests
  cannot create sockets in this sandbox. The focused ST3215/robot-I/O/profile
  compatibility set passed 9/9.
- ASan/UBSan profile, ST3215, PTY, and non-socket daemon coverage passed.
  LeakSanitizer cannot run under the sandbox ptrace policy, so leak detection
  was disabled for that sanitizer run.
- `git diff --check`: clean.

## Fix Round 2

Completed the follow-up safety and IPC compatibility pass. A missing ST3215
goal-write acknowledgement is now treated as optional so the executor
continues to the present-position read, while non-timeout acknowledgement
errors still stop the transaction. A timed-out enabled servo command now
publishes stale/timeout feedback, which the daemon feeds into its existing
serial safety-group latch until a newer command sequence recovers the group.

Mixed RuntimeHost outputs use one combined axis-and-servo client publication.
The servo snapshot is written before the axis snapshot, and the daemon treats
the matching axis sequence/timestamp as the commit barrier. The registry locks
all affected servos, validates every snapshot record against the shared epoch,
and only then stages the whole batch; a malformed or non-uniform batch leaves
all previous servo commands intact.

ABI-v2 setup validation now has raw-handshake coverage for a wrong advertised
descriptor count, a wrong SCM_RIGHTS count, and read-only descriptors supplied
for each writable role. The v1 receiver regression verifies that a v2 packet
is explicitly truncated rather than accepted as a valid v1 prefix, closes any
received descriptors, and leaves the server lifecycle clean.

### Fix-round verification

- Pinned CMake 4.4.2 all-target build: passed.
- Full non-socket CTest selection: 160/160 passed; the remaining IPC,
  virtual-serial, and cross-language socket entries are controller-runnable.
- Python suite: 28/28 passed.
- ASan/UBSan focused ST3215, RuntimeHost, IPC, and PTY coverage: passed with
  `detect_leaks=1` and `halt_on_error=1`.
- Focused ST3215 and RuntimeHost regressions: 25 consecutive repetitions each
  passed.
- `git diff --check`: clean.
