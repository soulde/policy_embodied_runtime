# Robot I/O daemon deployment and qualification

The C++ daemon separates device I/O from `policy-runtime-host`. It is intended
for a target that has been qualified with a real IgH EtherCAT master and Elmo
Gold drives. Building, installing, starting the service, and invoking the
diagnostic tools do **not** validate hardware.

## Install layout

Build without optional integrations for development, or enable IgH only on a
machine that has the matching development package and master device:

```bash
cmake -S . -B build -DPOLICY_RUNTIME_WITH_IGH=OFF -DPOLICY_RUNTIME_WITH_ZMQ=OFF
cmake --build build
cmake --install build --prefix /tmp/policy-runtime-install
```

The install places `robot-io-daemon`, `policy-runtime-host`,
`robot-io-health`, and `ethercat-cycle-stats` in `bin/`; the example profile
in `share/policy-runtime/robot_profiles/`; and the unit in
`lib/systemd/system/`, independently of the platform's multiarch library
directory. Distribution packages should use `/usr` as the install prefix, or
set `POLICY_RUNTIME_SYSTEMD_UNIT_DIR` and provide a systemd drop-in that
changes `ROBOT_IO_PROFILE` and `ExecStart` for a nonstandard prefix.

`elmo_gold_example.json` intentionally models three axes: CSP, CSV, and CST.
It is a documented representative subset, not a 12-axis machine definition.
Copy each matching sensor/actuator pair to extend it; the loader enforces at
most 12 static CiA 402 axes. Review every PDO mapping, scaling value, safety
limit, timeout, alias, and position against the actual ESI and commissioning
record before use.

## Service boundary and hardening

Create a `robot-io` service account, then grant it access to `/dev/EtherCAT0`
through a narrowly scoped udev rule. The packaged unit starts the daemon's
direct service mode. The daemon creates
`/run/policy-runtime/robot-io.sock` as an owner-only Unix `SOCK_SEQPACKET`
listener, publishes the current nonzero incarnation in the owner-only
`robot-io.generation` file, accepts one connection, and authenticates its peer
with `SO_PEERCRED`. The connecting runtime host or supervisor must therefore
run as `robot-io`, read the generation file, connect the socket, and pass that
generation to `RobotIoClient`. Every service restart publishes a fresh
generation, so mappings from an earlier incarnation are rejected.

The positional `ROBOT_PROFILE CONNECTED_SOCKET_FD GENERATION` daemon form
remains available for supervisors that already create and pass a connected
descriptor. The packaged unit uses the listener form directly and does not
assume that an arbitrary descriptor such as FD 3 is connected.

The supplied unit has `LimitRTPRIO=95`, `LimitMEMLOCK=infinity`,
`Restart=on-failure`, and a private `/run/policy-runtime` directory. It keeps
only `CAP_IPC_LOCK` and `CAP_SYS_NICE`, permits only `AF_UNIX`, and allows only
`/dev/EtherCAT0`. `PrivateDevices` remains disabled because the IgH character
device must be visible; do not broaden `DeviceAllow` unless the revised profile
requires another device. Test the unit with the exact target systemd release:

```bash
systemd-analyze verify /usr/lib/systemd/system/robot-io-daemon.service
sudo systemctl daemon-reload
sudo systemctl start robot-io-daemon
sudo systemctl status robot-io-daemon
```

Do not enable the unit at boot until socket ownership, generation discovery,
the IPC handshake, and restart with a newly generated incarnation have been
exercised on the target.

`robot-io-health` and `ethercat-cycle-stats` currently have a deliberately
safe failure mode: `--help` works, while a normal invocation reports
`unavailable` and opens neither the daemon nor EtherCAT hardware. They are
install/operational-discovery tools, not a substitute for the qualification
steps below.

## PREEMPT_RT qualification procedure

Perform these steps with an E-stop, current/velocity limits, and a trained
operator. Record kernel, NIC firmware/driver, IgH version, drive firmware,
profile checksum, and every result. A failure requires a safe stop and an
investigation; it is not an instruction to relax safety limits.

1. Boot the intended PREEMPT_RT kernel and record `uname -a`. Reserve the
   cyclic CPU with kernel `isolcpus`, `nohz_full`, and `rcu_nocbs` parameters;
   keep housekeeping and IRQ work off that CPU. Set the cyclic CPU's scaling
   governor to `performance`. On the dedicated EtherCAT NIC, inspect and
   disable Energy Efficient Ethernet (EEE) with `ethtool --show-eee` and
   `ethtool --set-eee ... eee off`. Inspect interrupt coalescing with
   `ethtool --show-coalesce`; disable adaptive and fixed coalescing settings
   that defer frames, using only options supported by that NIC driver. Disable
   other NIC power-saving features that add latency, and pin the NIC IRQs from
   `/proc/interrupts` to a non-cyclic CPU. Configure `irqbalance` so it does
   not undo the affinity. Reboot and re-check the CPU governor, EEE/coalescing
   settings, effective CPU lists, and IRQ affinities.
2. Before connecting motion hardware, run `cyclictest` with the intended
   1 kHz period, FIFO priority, memory lock, CPU affinity, and representative
   system load. Archive the full histogram and worst latency. Set an
   application-specific acceptance limit from the machine safety analysis;
   this document supplies no universal latency threshold.
3. With drives disabled, verify the physical topology and the expected Elmo
   vendor/product/revision. Bring the master to OP only after static PDO and
   DC settings match the reviewed ESI. Log distributed-clock offset/drift,
   working counter (WKC), slave state, daemon cycle period, wake latency,
   execution time, deadline misses, and skipped releases for an idle soak and
   a representative policy-load soak. Investigate every WKC/DC anomaly before
   enabling motion.
4. Exercise command-timeout behavior at low energy: stop command publication,
   verify the configured timeout causes Quick Stop followed by Disable, and
   verify that fresh commands cannot bypass the supervisor latch without the
   documented recovery path. Repeat for host exit and controlled daemon
   shutdown.
5. Exercise cable disconnect/reconnect with motion disabled or mechanically
   restrained. Confirm WKC/slave-state loss produces the configured safety
   response, no automatic unsafe re-enable occurs, and operator recovery is
   required as documented. Do not pull a live cable during unrestricted
   motion.
6. Qualify CSP, CSV, and CST separately with one reviewed axis at a time.
   Confirm the static `0x6060` mode and `0x6061` feedback agree, validate
   units/limits/following error, then test enable, disable, quick stop, fault
   reset, and a drive fault. CST begins with a conservative torque limit and
   no unexpected load. Expand the three-axis example to each target axis only
   after its individual record passes.

Only a reviewed record from this procedure on the actual target constitutes
hardware qualification.
