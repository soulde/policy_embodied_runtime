# Task 11: Packaging, Service Definition, and Hardware Qualification

## Result

Added install rules for the daemon, runtime host, and two safe diagnostic
tools. The package also installs a static three-axis Elmo Gold profile and a
hardened systemd service definition. The profile deliberately covers CSP, CSV,
and CST as a documented subset; the loader continues to enforce the 12-axis
limit.

The service constrains capabilities, address families, writable paths, device
access, and process privileges while retaining the realtime memory and FIFO
limits required by the daemon. It now invokes a direct listener mode instead
of naming a nonexistent inherited FD 3. That mode creates an owner-only Unix
`SOCK_SEQPACKET` socket and generation file in `RuntimeDirectory`, locks the
generation file against concurrent incarnations, publishes a fresh nonzero
generation, accepts one peer with bounded signal-aware polling, authenticates
the peer with `SO_PEERCRED`, and cleans runtime files by inode on exit. The
original positional connected-FD form remains supported.

Fix round 2 keeps the generation-file flock held while the listener checks and
unlinks both published inodes, and closes the generation descriptor only after
the generation path is removed. A concurrent handoff regression repeatedly
starts a successor while the old listener is being destroyed and verifies the
successor's generation remains published.

Fix round 3 bounds the remaining runtime-host handoff operations. The host
opens the generation path with `O_NONBLOCK`, rejects anything except a
nonempty regular file no larger than the 11-byte maximum 32-bit generation
representation, and performs the service-socket connect in nonblocking mode
against the configured steady-clock deadline. Completion is checked with
`poll()` and `SO_ERROR`, and the original descriptor flags are restored before
the existing setup handshake. A FIFO regression proves generation-path open
cannot wait for a writer, while a saturated-listener regression requires the
connector to return the specific timeout result within a bounded parent
deadline.

The daemon run loop now checks the accepted sole host on its non-realtime
control path. A closed peer or unexpected post-setup packet requests the normal
safe shutdown, leaves cyclic processing active until shutdown completes,
stops transports, and then returns the peer error to `robot-io-daemon`'s main
function. The process exits with failure so `Restart=on-failure` can start a
new listener with a fresh generation. Regression coverage checks the daemon's
shutdown health, packaged-host generation mismatch, accepted-host crash,
runtime-file cleanup, successor startup, and fresh-generation connection. The
packaged-host lifecycle test also waits for daemon failure after the fresh host
closes instead of racing that failure with an expected successful SIGTERM.

`policy-runtime-host` now accepts paired `--robot-io-socket` and
`--robot-io-generation-file` options in addition to the existing paired FD and
generation options. The service connector uses `O_NOFOLLOW`, strict generation
parsing, owner/type/mode checks, `SO_PEERCRED`, a post-connect inode recheck,
and the existing `RobotIoClient` generation handshake. The service integration
coverage starts a no-hardware daemon and packaged host, rejects a deliberately
mismatched generation, and exercises a fresh connection after daemon restart
when the host is built with ZeroMQ.

The systemd unit installs through the dedicated
`POLICY_RUNTIME_SYSTEMD_UNIT_DIR` default `lib/systemd/system`, independently
of GNUInstallDirs' multiarch library directory. The qualification guide now
distinguishes the CPU scaling governor from NIC settings and explicitly
requires checking/disabling EEE and latency-inducing interrupt coalescing.

`robot-io-health` and `ethercat-cycle-stats` provide `--help` and otherwise
report an explicit unavailable diagnostic without acquiring EtherCAT hardware.
They are not hardware-validation tools. `docs/robot-io-daemon.md` documents
the PREEMPT_RT, NIC IRQ, governor, CPU isolation, cyclictest, DC/WKC,
timeout, cable-disconnect, and CSP/CSV/CST HIL qualification procedure.

## TDD evidence

The fix-round tests first observed that daemon `--help` exited 2, listener
arguments were parsed as an invalid descriptor/generation, the packaged unit
still named FD 3 and generation 1, and a fresh `/usr`-prefix configuration
installed the unit below `lib/x86_64-linux-gnu/systemd/system`. The completed
tests exercise daemon help, direct listener handoff, socket/file modes,
generation freshness, peer-uid rejection, signal-time cleanup, unit listener
arguments, and an actual `/usr` plus `DESTDIR` install.

## Verification

- Pinned CMake 4.4.2 `/usr`-prefix configure and full all-target build with IgH
  and ZeroMQ disabled: passed.
- Non-socket `service_config_test` selection: 4/4 passed.
- `/usr` plus `DESTDIR` install CTest: 1/1 passed; the unit was present at
  `/usr/lib/systemd/system` and absent from the multiarch library directory.
- `robot-io-daemon`, `robot-io-health`, and `ethercat-cycle-stats` help paths:
  passed; daemon help exits zero without loading a profile or hardware.
- Python suite: 26 passed; the two remaining ZeroMQ tests were blocked by the
  managed sandbox at `socket(AF_INET, SOCK_STREAM)` with `EPERM`.
- The three new AF_UNIX listener tests compiled but were likewise blocked by
  the managed sandbox at `bind(AF_UNIX, SOCK_SEQPACKET)` with `EPERM`; no
  listener assertion failed before that boundary.
- `systemd-analyze verify` could not complete in the sandbox because it could
  neither connect to systemd nor resolve the not-yet-system-installed
  `/usr/bin/robot-io-daemon`; the unit's listener semantics are covered by the
  passing packaged-unit test and installed-artifact check.
- `git diff --check`: clean.

Fix-round-2 evidence before the controller-directed commit:

- Pinned CMake 4.4.2 rebuilt `policy-runtime-host`, `service_config_test`, and
  `runtime_host_test`: passed.
- Runtime-host CLI selection: 4/4 passed.
- Non-socket service/config selection, including unsafe generation-file
  rejection: 5/5 passed.
- Full service/config execution: 5 passed, 1 skipped because compatible
  cppzmq was unavailable, and 4 listener tests reached only the managed
  sandbox's `bind(AF_UNIX, SOCK_SEQPACKET)` `EPERM` boundary.
- The requested unsandboxed listener run was canceled before producing test
  output. Full build, install, Python, and sanitizer reruns were not performed
  after that cancellation because the controller directed an immediate commit
  with no more tests.

Fix-round-3 evidence:

- Cached pinned CMake 4.4.2 configured the `/usr`-prefix, IgH-off, ZeroMQ-off
  build and completed the full all-target build: passed. The `uv --with`
  resolver itself could not reach PyPI because outbound networking is blocked,
  so the already-cached verified 4.4.2 executable was invoked directly.
- Exact non-socket CTest selection: 187/187 passed.
- Full CTest: 187 passed, 24 socket-dependent tests failed only at the managed
  sandbox's `getsockopt(SO_DOMAIN)` or `bind(AF_UNIX, SOCK_SEQPACKET)` `EPERM`
  boundary, and four tests were explicitly skipped (three at the same AF_UNIX
  boundary and the packaged-host mismatch case because ZeroMQ was unavailable).
- Focused service coverage: the generation FIFO test passed in 10 ms; the
  full-backlog timeout and accepted-host crash/restart cases skipped at AF_UNIX
  bind `EPERM`; the packaged mismatch/restart case skipped because this build
  has no compatible ZeroMQ. The focused daemon peer-close safety test skipped
  at sandbox-forbidden `SO_DOMAIN` inspection.
- A `/usr` plus temporary `DESTDIR` install passed. The four installed
  `robot-io-daemon`, `policy-runtime-host`, `robot-io-health`, and
  `ethercat-cycle-stats` help paths all exited zero, and the unit installed at
  `/usr/lib/systemd/system/robot-io-daemon.service`.
- Full Python execution: 26 passed and the two ZeroMQ transport tests failed at
  sandbox-forbidden `socket(AF_INET, SOCK_STREAM)` with `EPERM`. The explicit
  non-socket Python selection passed 26/26.
- `git diff --check`: clean.

Hardware, real systemd activation, unsandboxed socket execution, and target
PREEMPT_RT/IgH/Elmo operation remain explicitly deferred to the documented
qualification procedure.
