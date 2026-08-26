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

Hardware, real systemd activation, unsandboxed socket execution, and target
PREEMPT_RT/IgH/Elmo operation remain explicitly deferred to the documented
qualification procedure.
