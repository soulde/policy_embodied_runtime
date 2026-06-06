# Embodied Policy Runtime

`policy_embodied_runtime` is an embodied policy runtime for VLA, imitation, and other policy inference workloads. It provides a policy runtime, robot input/output abstractions, policy implementations, JSON protocol validation, ZMQ transport, and SO-ARM101 simulation assets.

## Scope Boundary

This project does:

- robot-owned policy runtime and session lifecycle
- policy implementations
- policy profile validation
- ZMQ transport for policy RPC
- preprocess, postprocess, and validation placeholders
- robot `Sensor` and `Actuator` interfaces
- SO-ARM101 MuJoCo simulator

This project does not do:

- robot drivers
- ROS2 control
- realtime servo loops
- camera drivers

## Architecture

- `robot/`: robot layer inspired by `soulde/rustyRobot`; owns `RobotData`, `Sensor`, `Actuator`, `Policy`, session state, registry, and policy runtime orchestration
- `transport/`: low-level communication implementations in the `rustyRobot` sense; examples include ZMQ, serial, CAN, USB2CAN, and virtual serial
- `protocol/`: JSON policy RPC envelope and payload contracts plus protocol codec/errors
- `apps/`: runnable entrypoints such as the ZMQ policy RPC robot host
- `models/`: policy implementations consumed by the robot layer
- `sim/`: simulators, including the migrated SO-ARM101 MuJoCo + virtual ST3215 serial simulator
- `integrations/`: external usage examples only; ROS2 remains optional

The project uses the same layer vocabulary as `rustyRobot`:

```text
Robot Layer -> Sensor input, Actuator output, RobotData, policy runtime
Protocol    -> schema-validated policy RPC envelopes and payloads
Transport   -> low-level communication channel implementation
```

Anything that enters the robot is a `Sensor`, including physical sensors, remote controls, network links, simulator state, and policy RPC requests. Anything the robot outputs is an `Actuator`, including motors, grippers, serial/CAN commands, telemetry, logs, simulator commands, and policy RPC responses. `transport` is only the bottom communication mechanism used inside a sensor or actuator; for policy RPC the transport implementation is ZMQ.

Server flow in MVP:

1. Decode and validate request envelope.
2. Validate policy input.
3. Map incoming observation into canonical observation.
4. Run preprocess pipeline.
5. Call policy inference.
6. Run postprocess pipeline such as action clipping.
7. Validate and package the response.
8. Return policy action.
9. Encode response envelope.

## Development

This repository uses `uv venv` for the local virtual environment.

```bash
uv venv .venv
source .venv/bin/activate
env UV_CACHE_DIR=/tmp/uv-cache uv pip install -e ".[dev]"
pytest
```

## Quickstart

Validate schemas, runtime, transport, and robot I/O abstractions:

```bash
source .venv/bin/activate
pytest policy_embodied_runtime/tests
```

Run the robot runtime host:

```bash
policy-runtime-host \
  --policy-profile policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json
```

Run the SO-ARM101 MuJoCo simulator:

```bash
uv pip install -e ".[dev]"
policy-soarm101-sim
```

It exposes a stable virtual serial path at `/tmp/rusty_robot_soarm101`.

## Config Files

- `policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json`: SO-ARM101 sensors, actuators, serial transport, and ST3215 device IDs
- `policy_embodied_runtime/examples/robot_profiles/default_rpc_robot_profile.json`: default RPC sensor/actuator robot I/O profile
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

- Phase 1-2: scaffold, schemas, config loader, examples, validation tests
- Phase 3-5: policy registry, dummy policy, robot runtime, ZMQ transport
- Phase 6-9: robot I/O abstractions, example policies, integration tests, documentation refinement
