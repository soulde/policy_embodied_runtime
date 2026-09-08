# Device-owned codecs and physical-bus daemon refactor plan

**Goal:** Replace the current protocol/transport/daemon coupling with
directional devices, deduplicated physical buses, bus-owned workers, DDS
device topics, and a real SocketCAN OpenArm/Damiao integration simulator.

**Required build:** IgH EtherCAT, Cyclone DDS, and Cyclone DDS CXX are mandatory
system dependencies. The native runtime has no Unix shared-memory fallback and
no feature macros for IgH or DDS.

## Phase 0 — Stabilize the migration branch

- [ ] Inventory every modified/deleted/untracked file and preserve the existing
  DDS IDL, topic registry, realtime queues, Cyclone/Iceoryx configuration, and
  OpenArm assets that are still valid.
- [ ] Remove or rewrite interrupted partial code (especially daemon and host
  Damiao wiring) until the branch builds with mandatory dependencies.
- [ ] Vendor OpenArm assets as normal repository files (remove nested `.git`),
  retain its Apache-2.0 license, and record the upstream revision.
- [ ] Create a temporary IgH test prefix/shim only for CI compilation; production
  configure must fail if the real IgH package is absent.
- [ ] Run the clean native and Python baselines and record any unrelated failures.

## Phase 1 — Collapse protocol into directional devices

Create these transport-independent device interfaces:

```cpp
struct DeviceFrame {
  std::uint32_t address{};
  std::uint16_t size{};
  std::array<std::byte, 256> bytes{};
};

class SensorDevice {
 public:
  virtual bool accepts(const DeviceFrame&) const noexcept = 0;
  virtual Result<SensorEvent> decode(const DeviceFrame&) noexcept = 0;
};

class ActuatorDevice {
 public:
  virtual Result<DeviceFrame> encode(const ActuatorCommand&) noexcept = 0;
};
```

- [ ] Move CiA402 state-machine/PDO codec behavior under
  `robot/devices/cia402/`; remove runtime dependency on `protocol/cia402/`.
- [ ] Split Damiao into `DamiaoSensor` and `DamiaoActuator`. The sensor owns
  feedback decoding; the actuator owns enable, disable, zero and MIT encoding.
- [ ] Split ST3215 into `St3215Sensor` and `St3215Actuator`; each owns its packet
  parsing/encoding and device-specific validation.
- [ ] Keep message/value definitions transport-neutral under `robot_io/messages`.
- [ ] Delete obsolete `protocol/cia402`, `protocol/damiao`, and
  `protocol/st3215` files after all callers move.
- [ ] Add golden byte tests for every device direction, boundary, malformed
  frame, device-ID mismatch, enable/disable and fault behavior.

## Phase 2 — Compile the profile into a physical topology

- [ ] Define `PhysicalBusKey { transport_kind, canonical_path }` and canonical
  transport kinds: `ethercat`, `socketcan`, `usb_can`, and `serial`.
- [ ] Parse each sensor and actuator independently. A device declares compatible
  transport kinds; the profile chooses one kind and path.
- [ ] Deduplicate all devices by `PhysicalBusKey`. A sensor and actuator for the
  same motor share one bus; all motors on `can0` also share that one bus.
- [ ] Validate duplicate addresses, incompatible device/transport pairs,
  mismatched physical motor pairs, topic collisions and EtherCAT group membership.
- [ ] Produce an immutable `CompiledRobotTopology` containing buses, device
  bindings, DDS descriptors and EtherCAT execution groups.
- [ ] Add tests proving N logical devices on M physical connections create
  exactly M bus runtimes.

## Phase 3 — Transport factory and bus-owned loops

Create a factory that opens one transport per unique bus:

```cpp
Result<std::unique_ptr<BusRuntime>> make_bus_runtime(
    const PhysicalBusSpec&, std::span<DeviceBinding>);
```

- [ ] SocketCAN factory: create, set nonblocking/CLOEXEC, resolve interface,
  bind once, apply CAN filters, and own/close the descriptor.
- [ ] USB-CAN factory: open/configure the serial descriptor once and wrap the
  existing bounded virtual-CAN framing.
- [ ] ST3215 serial factory: open/configure once and preserve its request/response
  arbitration for multiple servos on one serial bus.
- [ ] EtherCAT factory: create one IgH master/domain runtime for the physical
  master and bind all PDO devices before activation.
- [ ] Each framed bus owns a receive worker and send worker. They communicate
  through preallocated bounded queues/mailboxes; no worker allocates, retries,
  reopens or changes topology after start.
- [ ] RX worker: one bounded receive, address routing, sensor decode, one event
  enqueue per valid physical frame.
- [ ] TX worker: consume latest actuator mailboxes, schedule fairly per bus, and
  perform one bounded send operation at a time.
- [ ] Bus faults latch and propagate to daemon safety/health; recovery is process
  restart only.
- [ ] EtherCAT keeps its single cyclic owner instead of pretending its process
  image is a framed byte stream; command commit remains group-atomic.

## Phase 4 — Make daemon a lifecycle/topology owner

- [ ] Reduce `RobotIoDaemon::configure` to topology compilation, device factory,
  transport factory, DDS endpoint creation and safety wiring.
- [ ] Start order: validate topology → open all buses → create all DDS entities →
  start RX workers → start TX/cyclic workers → accept commands.
- [ ] Stop order: reject commands → safe-stop actuators → stop workers → join →
  close each physical transport once → close DDS.
- [ ] Remove transport-specific branches and per-device polling from daemon.
- [ ] Remove the obsolete scheduler if bus runtimes fully own scheduling; otherwise
  retain it only as immutable assignment metadata.
- [ ] Publish health on transitions, including queue overflow and worker faults.

## Phase 5 — DDS at the device message boundary

- [ ] Generate exactly one command topic per actuator and one state topic per
  sensor from `CompiledRobotTopology`.
- [ ] DDS actuator readers validate schema, robot/device/session, sequence,
  timestamp, mode and limits, then call that actuator's encode/mailbox path.
- [ ] Sensor devices publish only after a physical RX event successfully decodes;
  no timer republishes cached state.
- [ ] Damiao and ST3215 commands are independent and never use commit.
- [ ] EtherCAT actuator commands stage a shared epoch; only a complete valid
  execution-group commit becomes visible to the cyclic owner.
- [ ] Add Deadline/Lifespan/Exclusive Ownership/Liveliness command QoS and
  BestEffort KeepLast(1) sensor QoS.
- [ ] Implement explicit host-session takeover and reject stale sessions.
- [ ] Implement the optional external-domain bridge with a validated sensor/health
  allowlist; commands and commits must be impossible to expose.

## Phase 6 — Host and policy runtime adapter

- [ ] Replace axis/servo-specialized `RuntimeRobotIo` with a profile-derived typed
  device registry so Damiao, ST3215 and CiA402 use the same sensor/action binding
  mechanism.
- [ ] Read latest typed sensor values by logical sensor name.
- [ ] Publish typed actuator values by logical actuator name.
- [ ] Generate EtherCAT epochs/commits only for EtherCAT groups.
- [ ] Remove all remaining daemon-host Unix socket, shared-memory, generation-file
  and compatibility CLI code/tests.
- [ ] Keep external policy RPC (ZeroMQ) separate; its IPC/TCP endpoint is not the
  removed robot-I/O IPC.

## Phase 7 — OpenArm virtual SocketCAN simulator

- [ ] Package the OpenArm v1 MuJoCo model/assets and upstream license.
- [ ] Implement a simulator process binding a real Linux SocketCAN interface,
  default `vcan0`; it must not bypass the kernel CAN device.
- [ ] Map CAN motor IDs 1–7 to OpenArm joints/actuators 1–7.
- [ ] Decode Damiao enable (`FF..FC`), disable (`FF..FD`), zero (`FF..FE`) and MIT
  frames. Ignore MIT actuation while disabled.
- [ ] Apply MIT `kp*(p_target-p) + kd*(v_target-v) + torque` to MuJoCo within
  actuator limits.
- [ ] Emit canonical eight-byte Damiao feedback frames after received control/MIT
  traffic, including motor ID, error state, position, velocity, torque and
  temperatures.
- [ ] Provide `policy-openarm-sim --interface vcan0 [--headless]`.
- [ ] Provide setup commands:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 up
```

- [ ] Add a seven-motor DDS robot profile using one shared `vcan0` bus.
- [ ] Add a DDS command utility/test fixture that explicitly exercises enable,
  MIT movement and disable.

## Phase 8 — End-to-end joint test

- [ ] Start `iox-roudi` and Cyclone DDS with the local PSMX/Iceoryx config.
- [ ] Create and bring up `vcan0`.
- [ ] Start headless OpenArm simulator.
- [ ] Start `robot-io-daemon` with the seven-motor OpenArm profile.
- [ ] Start the DDS host/test driver and wait for discovery.
- [ ] Enable all motors; verify feedback state changes to enabled.
- [ ] Send bounded MIT position targets; verify MuJoCo joints move and state topics
  report physical receive events.
- [ ] Disable all motors; verify outgoing actuator effort becomes zero and further
  MIT frames do not move the model.
- [ ] Kill the simulator or daemon and verify the transport/safety fault latches.
- [ ] Assert exactly one SocketCAN bus runtime and two workers exist for seven
  sensor/actuator pairs.

## Phase 9 — Mandatory dependencies and cleanup

- [ ] Remove `POLICY_RUNTIME_WITH_DDS`, `POLICY_RUNTIME_HAS_DDS`, and
  `POLICY_RUNTIME_WITH_IGH` conditionals/options everywhere.
- [ ] Use unconditional `find_package(CycloneDDS CONFIG REQUIRED)`,
  `find_package(CycloneDDS-CXX CONFIG REQUIRED)`, and
  `find_package(EtherCAT REQUIRED)`.
- [ ] Build DDS IDL and IgH backend unconditionally.
- [ ] Update install docs and systemd units for Cyclone config, RouDi and required
  libraries.
- [ ] Delete primary Python host/runtime/device implementations; retain simulation,
  its protocol fixtures, and test/client utilities required for joint testing.
- [ ] Remove dead tests rather than silently excluding them; replace removed IPC
  coverage with DDS/device/bus contract coverage.

## Verification gates

1. Device codec unit tests.
2. Topology and factory unit tests.
3. Per-transport PTY/socket/vcan integration tests.
4. DDS endpoint and EtherCAT commit tests with real Cyclone DDS CXX.
5. OpenArm simulator unit tests.
6. Headless `vcan0` end-to-end test.
7. Full C++ `ctest --output-on-failure`.
8. Simulation-only Python `pytest` suite.
9. Clean mandatory-dependency configure/install test.
10. `git status` audit proving build directories, nested repository metadata and
    unrelated files are not committed.
