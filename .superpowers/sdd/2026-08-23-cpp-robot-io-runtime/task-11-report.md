# Task 11: Packaging, Service Definition, and Hardware Qualification

## Result

Added install rules for the daemon, runtime host, and two safe diagnostic
tools. The package also installs a static three-axis Elmo Gold profile and a
hardened systemd service definition. The profile deliberately covers CSP, CSV,
and CST as a documented subset; the loader continues to enforce the 12-axis
limit.

The service constrains capabilities, address families, writable paths, device
access, and process privileges while retaining the realtime memory and FIFO
limits required by the daemon. Its connected IPC descriptor is intentionally
supplied by a supervisor: the service does not invent a listener or shell
wrapper around the daemon's existing IPC contract.

`robot-io-health` and `ethercat-cycle-stats` provide `--help` and otherwise
report an explicit unavailable diagnostic without acquiring EtherCAT hardware.
They are not hardware-validation tools. `docs/robot-io-daemon.md` documents
the PREEMPT_RT, NIC IRQ, governor, CPU isolation, cyclictest, DC/WKC,
timeout, cable-disconnect, and CSP/CSV/CST HIL qualification procedure.

## TDD evidence

`service_config_test` first failed because the Elmo profile was absent. After
the profile was added, it passed static-mode, timeout, and representative-mode
coverage. The diagnostic test then failed because neither tool binary existed;
the tools were added and the test now exercises actual `--help` output plus
their safe unavailable result.

## Verification

- Pinned CMake 4.4.2 configure and full all-target build with IgH and ZeroMQ
  disabled: passed.
- Focused `service_config_test`: 2/2 passed.
- Full CTest non-TCP selection: 201/201 passed.
- Temporary-prefix install contained both runtime binaries, both tools, the
  profile, and the systemd unit; both installed tools accepted `--help`.
- Python suite: 28 passed.
- `git diff --check`: clean.

Hardware, systemd activation with a real supervisor-provided IPC descriptor,
and target PREEMPT_RT/IgH/Elmo operation remain explicitly deferred to the
documented HIL qualification procedure.
