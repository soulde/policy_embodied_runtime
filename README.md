# Embodied Policy Runtime

`policy_embodied_runtime` is an embodied policy runtime for VLA, imitation, and other policy inference workloads. It provides a policy runtime, robot input/output abstractions, policy implementations, JSON protocol validation, ZMQ transport, and SO-ARM101 simulation assets.

## Scope Boundary

The repository now contains two cooperating C++ processes as well as the
Python-compatible policy and simulator layers:

- `policy-runtime-host` owns policy inference, canonical observation/action
  binding, session state, and the optional ZeroMQ JSON interface.
- `robot-io-daemon` owns every physical transport, protocol, device registry,
  scheduling decision, watchdog, and safety transition.
- Python policy, profile, protocol, and SO-ARM101 simulator packages remain
  supported for compatibility and deterministic regression tests.

ROS2 control, camera drivers, real IgH hardware qualification, and HIL are
integration concerns rather than hidden runtime dependencies.

## Current Architecture

The runtime follows `Transport -> Protocol/Device -> Sensor` on input and
`Actuator -> Protocol/Device -> Transport` on output. A transport performs
physical I/O; a protocol translates fields in a process image or frame; a
device exposes typed sensor/actuator values.

The C++ tree mirrors this layering one-to-one under `include/policy_runtime/`
and `src/` (both consumed by the CMake build):

| Module | Responsibility |
| --- | --- |
| `common/` | `Result<T>` error type shared by every layer |
| `transport/` | EtherCAT/IgH backend + Elmo Gold PDO mapping, SocketCAN, USB serial, USB-CAN virtual serial framing |
| `protocol/` | CiA402 PDO/state machine, ST3215, Damiao MIT CAN codec, RPC JSON envelope codec |
| `devices/` | Directional `SensorDevice`/`ActuatorDevice` implementations, including Damiao sensor and actuator codecs |
| `robot_io/` | daemon, versioned IPC, snapshot exchange, safety supervisor, transport scheduler |
| `runtime/` | `policy-runtime-host` pipeline, host CLI, daemon client |
| `profiles/` | robot/policy JSON profile loader with static validation |
| `policy/` | preprocess, policy, postprocess pipeline stages |

Third-party C++ dependencies are vendored under `third_party/` (currently
`nlohmann-json` 3.11.3, header-only, trimmed to build inputs), so a clean
checkout configures and builds fully offline; IgH EtherCAT and ZeroMQ remain
optional configure-time features.

```text
policy-runtime-host (non-RT)                 robot-io-daemon
┌──────────────────────────────┐       ┌──────────────────────────────┐
│ ZMQ/JSON RPC (optional)      │       │ 1 kHz EtherCAT owner          │
│ profile + policy pipeline    │       │ IgH backend / Elmo Gold PDO  │
│ canonical bindings           │       │ CiA402 Axis (CSP/CSV/CST)    │
└──────────────┬───────────────┘       └──────────────┬───────────────┘
               │ Unix SOCK_SEQPACKET + memfd snapshots │
               └───────────────────────────────────────┘
                                           ┌───────────┴───────────┐
                                           │ daemon RT serial cycles│
                                           │ ST3215 shared bus     │
                                           └───────────────────────┘
```

EtherCAT is the transport layer. Its PDO and mailbox endpoints share one
backend, but the cyclic master is the sole ecrt owner: each cycle performs
bounded PDO exchange and at most one mailbox/SDO step, without waiting for a
lower-priority thread. CiA402 is the protocol layer; `Cia402Axis` is both a
typed `Sensor` and `Actuator`. CSP/CSV/CST (8/9/10) are static profile choices
and cannot be switched at runtime.

Serial/ST3215 now follows the same hard-realtime contract as the CAN paths:
the daemon cycle drives one `St3215Bus::cycle()` per period, which performs at
most one bounded receive (the response to the previous cycle's request) and
stages exactly one write on the shared half-duplex bus. The serial transport
runs with zero read/write timeouts so cycles never block, write errors,
malformed frames, and device-ID mismatches latch a permanent fault, and a
response absent past the servo feedback timeout also latches. Recovery is
restart-only. `St3215Servo` still only stages commands and consumes lock-free
snapshots.

Damiao CAN motors use the vendor MIT-mode protocol over SocketCAN or a
USB-CAN virtual serial port, configured statically in the robot profile
(`damiao_motor` devices; see
`policy_embodied_runtime/examples/robot_profiles/damiao_can_robot_profile.json`).
SocketCAN is represented by one physical `TransportRuntime` per interface.
The runtime performs raw frame I/O only; Damiao sensors decode received frames
and Damiao actuators encode outgoing frames independently. Multiple protocols
may share one physical transport.

The daemon/host IPC is versioned. Axis-only profiles use the compact v1
layout; profiles containing ST3215 devices use v2 with independent bounded
axis and servo snapshots. Setup transfers role-checked file descriptors and a
fresh generation. Commands carry sequence/timestamp metadata, stale or future
records are rejected, and mixed axis+servo publications commit only after all
records validate. A failed command or transport health event follows
QuickStop -> zero-speed confirmation -> Disable; recovery requires a newer
command sequence.

## Policy request flow

1. Decode and validate the JSON envelope (including correlated runtime errors).
2. Normalize nullable and numeric fields using the Python-compatible schema.
3. Bind observations from daemon snapshots to canonical policy inputs.
4. Run preprocess, policy inference, and postprocess/action validation.
5. Validate all axis and ST3215 targets, then publish one sequence atomically.
6. Daemon devices translate the snapshot into PDO fields or serial frames;
   only transports perform physical I/O.

## Development

This repository uses `uv venv` for the local virtual environment.

```bash
uv venv .venv
source .venv/bin/activate
env UV_CACHE_DIR=/tmp/uv-cache uv pip install -e ".[dev]"
pytest
```

## C++ robot I/O daemon

The C++ implementation packages `robot-io-daemon`, `policy-runtime-host`, safe
diagnostic CLIs, an Elmo Gold static-mode example, and a hardened systemd
listener. IgH EtherCAT and ZeroMQ are optional at configure time, while the
fake backends and PTY tests keep development hardware-free. Use CMake >=3.24
from the documented `uv` environment, for example:

```bash
env UV_CACHE_DIR=/tmp/uv-cache uv run --with 'cmake>=3.24' cmake -S . -B build \
  -DPOLICY_RUNTIME_WITH_IGH=OFF
env UV_CACHE_DIR=/tmp/uv-cache uv run --with 'cmake>=3.24' cmake --build build
ctest --test-dir build
```

nlohmann/json is vendored in `third_party/nlohmann-json` and compiled from
source when no system package is found, so the build needs no network access.

Target installation, service setup, and the PREEMPT_RT/EtherCAT qualification
procedure are in [docs/robot-io-daemon.md](docs/robot-io-daemon.md). Passing
fake/PTY tests, packaging checks, or CLI help does not validate connected
hardware; real IgH/Elmo and 12-axis HIL remain deployment qualification steps.
Migration, parity-gate, and Python compatibility-layer details are in
[docs/migration/cpp-runtime.md](docs/migration/cpp-runtime.md); the outstanding
target matrix is tracked in
[docs/qualification/cpp-runtime-final.md](docs/qualification/cpp-runtime-final.md).

## Quickstart

Validate schemas, runtime, transport, and robot I/O abstractions:

```bash
source .venv/bin/activate
pytest policy_embodied_runtime/tests
```

The basic runtime flow has three roles:

1. `robot-io-daemon` loads the robot profile, owns EtherCAT/serial devices,
   and publishes versioned snapshots over its Unix socket.
2. `policy-runtime-host` loads the policy profile, reads snapshots, runs
   policy inference, and publishes validated axis/servo commands.
3. An upstream RPC client or simulator supplies policy observations. For
   SO-ARM101 simulation this is `policy-soarm101-command-publisher`.

Run a minimal policy host (direct handler mode):

```bash
policy-runtime-host \
  --policy-profile policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json
```

For a packaged daemon listener, pass the same socket and generation-file paths
to the host (`--robot-io-socket` and `--robot-io-generation-file`); the service
unit documents the corresponding `/run/policy-runtime` locations.

Run the SO-ARM101 flow with simulation as the environment:

```bash
uv pip install -e ".[dev]"
policy-soarm101-sim --gui
policy-runtime-host \
  --policy-profile policy_embodied_runtime/examples/policy_profiles/soarm101_sim_policy_profile.json \
  --robot-profile policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json
policy-soarm101-command-publisher
```

The simulator exposes a stable virtual serial path at `/tmp/rusty_robot_soarm101`, matching the SO-ARM101 robot profile. In a real deployment, keep the same runtime host flow and replace the simulator/profile path, policy profile, and input publisher with the real robot and policy.

## Config Files

- `policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json`: SO-ARM101 sensors, actuators, serial transport, and ST3215 device IDs
- `policy_embodied_runtime/examples/robot_profiles/default_rpc_robot_profile.json`: default RPC sensor/actuator robot I/O profile
- `policy_embodied_runtime/examples/robot_profiles/damiao_can_robot_profile.json`: Damiao CAN motors over SocketCAN and a USB-CAN virtual serial port
- `policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json`: MVP single-step dummy policy
- `policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json`: placeholder chunked policy shape
- `policy_embodied_runtime/examples/policy_profiles/soarm101_sim_policy_profile.json`: policy bindings from SO-ARM101 robot data names to canonical model fields

Robot profiles describe robot data interfaces and hardware links only. Policy selection, policy choice, canonical input/output schemas, and preprocess/postprocess bindings belong to policy profiles.

Robot profile entries use the same shape for sensors and actuators:

```json
{
  "name": "shoulder_pan_position",
  "device": {"type": "st3215", "path": "/tmp/rusty_robot_soarm101"},
  "args": {"baud_rate": 1000000, "device_id": 1, "servo_id": 1}
}
```

`name` is the unique robot data field. `device.type` selects the hardware/network device model, and `device.path` is the physical endpoint, such as a serial path or RPC address. Sensors or actuators with the same device type and path reuse the same underlying transport. Device-specific options live under `args`.

## Protocol Contract

All messages use a JSON envelope with:

- `schema`
- `type`
- `request_id`
- `session_id`
- `step_id`
- `timestamp_ns`
- `payload`
- `error`

Observation and action payloads must use named semantic fields. Bare arrays are not permitted at protocol level.

Policy RPC observations and actions use the canonical fields declared by the active policy profile.

## Status

Current implementation is staged:

- Python policy/protocol/runtime modules remain as a deliberate compatibility
  layer because published wheel entry points and the parity oracle still use
  them; new hardware deployments should select the CMake-installed native
  `policy-runtime-host` explicitly.
- C++ runtime host, versioned daemon IPC, CiA402/Elmo EtherCAT, and ST3215
  serial execution paths are implemented and covered by fake/PTY tests.
- PREEMPT_RT scheduling, real IgH linkage, Elmo commissioning, and 12-axis HIL
  are target-environment qualification work; see the daemon guide.
