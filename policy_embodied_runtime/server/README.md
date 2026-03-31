# Server Runtime

`policy_embodied_runtime/server/` contains the embodied policy runtime implementation. It owns request validation, adapter/model execution, canonical observation and action flow, transport wiring, and the minimal runtime registries used by the SDK and tests.

## Layout

- `apps/`: runnable entrypoints such as the ZMQ server
- `core/`: runtime orchestration, sessions, errors, and adapter registries
- `adapters/`: embodiment adapters that map external observations/actions to canonical fields
- `models/`: model adapters that infer canonical actions from canonical observations
- `schemas/`: Pydantic contracts for messages, policy profiles, and embodiment profiles
- `transport/`: envelope codec, endpoint normalization, and ZMQ socket wrappers
- `preprocess/`: preprocess interfaces and pipeline
- `postprocess/`: postprocess interfaces and pipeline
- `safety/`: validation checks that do not mutate actions
- `tests/`: runtime and transport coverage

## Request Flow

1. Decode the incoming message envelope.
2. Validate the payload against the message schema.
3. Route the request to the runtime API.
4. Validate embodiment-specific input fields.
5. Run embodiment preprocessors.
   The default embodiment preprocess pipeline includes observation mapping, and if the policy profile marks a canonical observation field as normalized, automatic normalization from robot limits into model space.
6. Run preprocessors.
7. Run model inference.
8. Run postprocessors.
9. Run embodiment postprocessors.
   The default embodiment postprocess pipeline includes automatic denormalization for normalized model outputs, robot limits, and action mapping back into embodiment semantics.
10. Encode the response envelope.

## Endpoint Rules

The server accepts a single `endpoint` and binds one ZMQ REP socket.

- `embodied-policy-runtime` resolves to `ipc:///tmp/embodied-policy-runtime.sock`
- `tcp://127.0.0.1:5555` is used as-is

Request routing does not happen through transport subpaths. The runtime reuses `MessageEnvelope.type` to dispatch:

- `health_request`
- `server_info_request`
- `reset_request`
- `observation_request`

## Server Construction

`ZmqPolicyServer` accepts required `policy_profile` and `embodiment_profile` initialization parameters as JSON file paths.

- The runtime is constructed eagerly during server initialization.
- One server instance corresponds to one fixed policy profile and embodiment profile.
- Reconfiguration happens by starting a new server, not by sending a runtime load request.
