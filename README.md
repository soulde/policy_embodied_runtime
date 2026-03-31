# Embodied Policy Runtime

`policy_embodied_runtime` is an embodied policy runtime for VLA, imitation, and other policy inference workloads. The MVP is intentionally narrow: it provides a policy server runtime, model adapters, embodiment adapters, JSON schema/config validation, ZMQ transport, and a Python client SDK.

## Scope Boundary

This project does:

- `policy_server` runtime and session lifecycle
- model adapters
- embodiment adapters
- policy profile and embodiment profile validation
- ZMQ + JSON transport
- preprocess, postprocess, and validation placeholders
- Python client SDK for launching and talking to the server

This project does not do:

- robot drivers
- ROS2 control
- hardware integration
- realtime servo loops
- camera drivers
- actuator interfaces

## Architecture

- `server/`: runtime, adapters, models, schemas, transport, safety
- `sdk/`: Python client API that hides ZMQ and process launch details
- `integrations/`: non-core usage examples only; ROS2 remains optional

Server flow in MVP:

1. Decode and validate request envelope.
2. Validate embodiment-specific input.
3. Map incoming observation into canonical observation.
4. Run preprocess pipeline.
5. Call model adapter inference.
6. Run postprocess pipeline such as action clipping.
7. Validate and package the response.
8. Map canonical action into embodiment action.
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

Validate schemas, server runtime, transport, and SDK:

```bash
source .venv/bin/activate
pytest policy_embodied_runtime/server/tests policy_embodied_runtime/sdk/tests
```

Run the dummy SDK example:

```bash
python -m policy_embodied_runtime.examples.sdk_dummy_roundtrip
```

The example launches a local server, loads the dummy policy profile and embodiment profile, resets a session, and performs one inference step.

## Config Files

- `policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json`: MVP single-step dummy adapter
- `policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json`: placeholder chunked adapter shape
- `policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json`: direct mapping profile
- `policy_embodied_runtime/examples/embodiment_profiles/franka_like_profile.json`: Franka-like semantic field mapping

## Transport Contract

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

For embodiment adapters that declare nested `source_field` paths such as `arm.joint_position` or `camera.front_rgb`, the ZMQ observation payload may include those JSON object paths directly under `observation`. The runtime will map them into canonical fields before model inference.

## Status

Current implementation is staged:

- Phase 1-2: scaffold, schemas, config loader, examples, validation tests
- Phase 3-5: adapter registries, dummy adapters, runtime core, ZMQ transport
- Phase 6-9: SDK, example adapters, integration tests, documentation refinement
