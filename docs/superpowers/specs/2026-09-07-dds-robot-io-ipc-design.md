# DDS Robot I/O IPC Design

## 1. Scope

Replace the Unix `SOCK_SEQPACKET` and shared-memory IPC between
`robot-io-daemon` and `policy-runtime-host` with Eclipse Cyclone DDS. The
physical EtherCAT, SocketCAN, USB-CAN virtual serial, and ST3215 serial
transports remain unchanged. The host's external ZeroMQ JSON policy RPC also
remains unchanged.

The migration temporarily supports selectable `unix_shm` and `dds` robot I/O
backends. Once DDS behavior and end-to-end tests match the required runtime
semantics, the branch removes the Unix socket listener, shared-memory mappings,
generation files, obsolete service arguments, and their tests.

Cyclone DDS and Cyclone DDS C++ are system dependencies discovered with
`find_package`. CMake must not download them. `POLICY_RUNTIME_WITH_DDS` controls
the integration; explicitly enabling DDS without compatible dependencies is a
configuration error rather than a silent fallback.

## 2. Device and Topic Model

Sensors and actuators are independent endpoints. A physical motor is modeled
as an actuator and a sensor associated by the robot profile, not as one combined
DDS endpoint. A configured logical sensor has one state topic. A configured
logical actuator has one command topic.

Topic topology is static after startup and is derived from the same robot
profile in both processes. Default names are:

```text
robot_io_<robot_id>_actuator_<device_id>_command
robot_io_<robot_id>_sensor_<device_id>_state
robot_io_<robot_id>_ethercat_command_commit
robot_io_<robot_id>_health
```

Profile validation normalizes identifiers, rejects invalid DDS names and name
collisions, and permits an explicit topic-name override. `robot_id` isolates
multiple robots in the same DDS domain. Device identity and topic direction
remain explicit in every message so a configuration mismatch fails closed.

## 3. IDL Data Contracts

IDL types are bounded and device-class-specific rather than a generic property
map or JSON envelope. Initial pairs are:

- `Cia402ActuatorCommand` and `Cia402SensorState`
- `DamiaoActuatorCommand` and `DamiaoSensorState`
- `St3215ActuatorCommand` and `St3215SensorState`

Each message includes a common, bounded header containing:

- schema version;
- robot and device identifiers;
- publisher session identifier;
- monotonically increasing sequence;
- source monotonic timestamp;
- physical bus cycle or receive sequence;
- validity and diagnostic flags.

The IDL represents the existing typed command and feedback fields without
depending on C++ object layout, host endianness, struct padding, file
descriptors, or shared-memory ABI versions. New device classes add new typed IDL
contracts instead of extending an unbounded union.

## 4. Command Semantics

Every actuator validates robot identity, device identity, schema version,
session, sequence, timestamp, static operating mode, finite values, limits, and
deadline before accepting a command.

Only EtherCAT actuators use `CommandCommit`. The host publishes each member's
command with a common epoch and then publishes the commit keyed to the static
EtherCAT execution group. The daemon activates the epoch only when every group
member has a valid command for it. A missing member, inconsistent epoch, or
commit timeout rejects the entire group, so no partial EtherCAT update reaches
the process image. Accepted commands take effect in the same EtherCAT cycle.

CAN, USB, virtual-serial, and serial actuators do not use commit. Each accepted
command independently becomes eligible for its transport's next operation.
These transports do not claim physical simultaneity, and no atomicity or timing
guarantee is provided across different buses.

## 5. Sensor Publication Semantics

Sensor publication is driven by physical receive events, not a DDS timer:

- after a valid EtherCAT process image is received, each mapped sensor produces
  one sample carrying the common bus-cycle value;
- after a valid CAN, USB, or serial response is received, the corresponding
  sensor produces one sample;
- without new physical data, old sensor values are not republished periodically;
- health and fault transitions produce their own immediate health events.

The hard-realtime transport loop never calls DDS. It writes each event into a
preallocated bounded single-producer/single-consumer queue. A non-realtime DDS
publisher drains the queue. If publishing cannot keep up, the producer evicts
the oldest pending sensor sample and retains the newest one. It increments a
per-sensor dropped-sample counter and reports degraded communication health,
without blocking or reinitializing the physical bus.

## 6. Threads and Realtime Boundary

The daemon owns two DDS bridge directions outside its transport cycles:

1. A DDS subscription path validates actuator commands and stages them in
   preallocated, lock-free command snapshots. Realtime bus owners consume at
   most one complete eligible update per device or EtherCAT group per cycle.
2. Realtime bus owners enqueue sensor and health events into bounded queues. A
   DDS publication path serializes and writes them outside the realtime thread.

The host uses a DDS-backed `RuntimeRobotIo` adapter. Its writer publishes typed
actuator commands, including EtherCAT epochs and commits. Its reader updates
bounded latest-value sensor snapshots consumed by the existing runtime host.
Policy, preprocess, postprocess, and external ZMQ request handling do not depend
on Cyclone DDS types.

No realtime path may allocate, block, wait for discovery, retry DDS operations,
create entities, reopen a transport, or rebuild DDS state. DDS entities and all
queue storage are created and validated before bus cycles start. Runtime DDS
failure is reported and fed to safety supervision; recovery remains
restart-only.

## 7. DDS QoS and Ownership

Actuator command topics use:

- Reliable reliability;
- KeepLast depth 1;
- Volatile durability;
- configured Deadline and Lifespan;
- Exclusive Ownership so only the selected host controls an actuator;
- liveliness suitable for detecting host loss within the configured safety
  deadline.

Sensor state topics use BestEffort reliability, KeepLast depth 1, and Volatile
durability. Sequence counters expose samples lost in DDS or in the local
publisher queue. Health topics use Reliable delivery with bounded history.

The daemon selects a newly valid host session only through an explicit startup
handshake/state transition. Commands from an old session remain invalid after a
host restart even if delayed DDS samples arrive later.

## 8. Discovery and Selective External Exposure

The local control domain binds to loopback by default. Network-interface
selection applies at the Cyclone DDS domain/participant level, so DDS partitions
are not treated as a network security boundary.

Optional external telemetry uses a second DDS domain and a non-realtime bridge.
The bridge republishes only statically configured sensor and health topics. Its
allowlist is validated against the robot topic registry; actuator command and
EtherCAT commit topics are forbidden. The external domain binds only to
explicitly configured interfaces. Cross-machine deployment therefore requires
an intentional Cyclone DDS XML and profile change.

Representative configuration:

```json
{
  "dds": {
    "backend": "dds",
    "local_domain_id": 0,
    "cyclone_config": "/etc/policy-runtime/cyclonedds.xml",
    "sensor_queue_capacity": 64,
    "external": {
      "enabled": false,
      "domain_id": 10,
      "topics": ["motor_1_state", "bus_health"]
    }
  }
}
```

The final profile schema will retain existing static safety limits and add DDS
fields without permitting runtime mode, transport, topology, or topic changes.

## 9. Failure and Safety Behavior

DDS initialization, topic creation, discovery prerequisites, type mismatch, or
profile topology mismatch at startup prevents the affected process from
starting. There is no automatic fallback from requested DDS to Unix IPC.

At runtime:

- stale, duplicated, reordered, future-dated, expired, wrong-session, and
  wrong-device commands are rejected;
- command deadline or host liveliness loss enters the existing safe-stop path;
- EtherCAT commit failure stops or rejects the complete configured EtherCAT
  safety group;
- independent transports stop according to their existing safety-group rules;
- sensor queue overflow degrades communication health but never blocks the bus;
- physical transport failures retain their existing latched, restart-only
  behavior;
- DDS failures do not cause participant or transport recreation from a realtime
  cycle.

## 10. Migration Sequence

1. Add optional system Cyclone DDS discovery and an offline compile fixture.
2. Add bounded IDL and generated-code integration.
3. Extract transport-neutral realtime command snapshots and sensor event queues
   from the current Unix IPC implementation.
4. Add DDS endpoint unit tests and the daemon-side bridge.
5. Add the host-side `RuntimeRobotIo` DDS adapter.
6. Add static profile, CLI, systemd, and local-domain configuration.
7. Add the optional external telemetry bridge and allowlist validation.
8. Run Unix IPC and DDS backends against common behavioral contract tests.
9. Make DDS the sole daemon-host backend and remove Unix sockets, shared memory,
   generation files, compatibility arguments, and obsolete tests.
10. Update packaging, examples, and architecture documentation.

## 11. Verification

Tests must cover:

- IDL serialization round trips and numeric boundaries;
- profile-derived topic names, normalization, collisions, and topology match;
- one independent topic per sensor and actuator;
- one publication event per valid physical receive event;
- bounded queue ordering, overflow, drop counters, and latest-sample retention;
- stale, duplicate, out-of-order, expired, future, and wrong-session commands;
- exclusive command ownership and liveliness loss;
- EtherCAT complete-epoch activation and incomplete-epoch rejection;
- absence of commit semantics for CAN, USB, virtual serial, and serial devices;
- daemon and host restart behavior;
- local-only discovery defaults;
- external-domain telemetry allowlisting and command-topic rejection;
- common behavioral parity while both backends exist;
- end-to-end daemon/host operation using hardware-free fake transports;
- zero allocation, blocking, DDS calls, retries, or entity creation in hard
  realtime cycles.

The full Python compatibility/simulator tests and C++ unit/integration tests run
throughout migration. Existing unrelated test failures must be recorded rather
than hidden by the new backend.

## 12. Rejected Alternatives

- A single batch topic for all sensors and actuators conflicts with the required
  per-device topic model.
- Cross-bus `CommandCommit` would imply synchronization that CAN, USB, and serial
  transports cannot provide.
- Calling DDS directly from transport cycles violates hard-realtime rules.
- DDS partitions alone do not provide network isolation for selectively exposed
  telemetry.
- Vendoring or downloading Cyclone DDS during CMake configuration conflicts with
  the repository's offline build policy.
