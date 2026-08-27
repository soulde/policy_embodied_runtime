# Migrating to the C++ runtime

The C++ `policy-runtime-host` and `robot-io-daemon` are the primary runtime
processes for new deployments. They preserve the
`embodied-policy-runtime/v1alpha1` JSON contract, existing policy/robot
profiles, session behavior, and the static SO-ARM101 simulator interface.

## Install and launch

Build and install with CMake 3.24 or newer:

```bash
cmake -S . -B build -DPOLICY_RUNTIME_BUILD_TESTS=ON \
  -DPOLICY_RUNTIME_WITH_IGH=OFF
cmake --build build
cmake --install build --prefix /opt/policy-runtime
```

The CMake install supplies these native executables:

- `/opt/policy-runtime/bin/policy-runtime-host`
- `/opt/policy-runtime/bin/robot-io-daemon`
- `/opt/policy-runtime/bin/robot-io-health`
- `/opt/policy-runtime/bin/ethercat-cycle-stats`

Use the native host by absolute path, or put its install directory before the
Python environment on `PATH`. The Python package currently publishes a legacy
console script with the same `policy-runtime-host` name.

For the packaged daemon listener, follow
[the robot I/O daemon guide](../robot-io-daemon.md). A host using daemon-owned
devices must receive the same robot profile and connect with
`--robot-io-socket` and `--robot-io-generation-file`. A host using only RPC
robot devices does not require daemon IPC.

## Compatibility gate

`tests/compat/run_runtime_parity.py` is the retirement gate. It runs one
long-lived host per profile so session state is observable and compares the
Python and C++ responses after normalizing only response timestamps. JSON
integers and floats receive distinct tags before comparison. The gate covers:

- health, server information, observation, and action happy paths;
- unknown request and malformed JSON error envelopes;
- explicit null observations and required-field rejection;
- integer inputs whose known action fields must be floating-point on the wire;
- independent sessions and reset-to-step-zero behavior;
- summaries of every golden policy and robot profile;
- shared policy and robot profile rejection cases.

Run it after building the test contract driver:

```bash
pytest tests/compat/run_runtime_parity.py -q
python tests/compat/run_runtime_parity.py \
  --driver build/runtime_contract_driver
```

CTest runs the same gate as `runtime_cross_language_test` when the configured
Python interpreter has the compatibility dependencies.

## Python compatibility-layer disposition

Task 12 did **not** delete the Python runtime modules. Retention is deliberate,
not an assertion that the migration is complete:

- the Python wheel still publishes `policy-runtime-host` and
  `policy-soarm101-command-publisher`;
- the command publisher imports the Python profile, protocol, and ZeroMQ
  packages;
- the independently packaged simulator and its serial verification path are
  still validated against Python ST3215/transport behavior;
- the Python implementation is the executable oracle used by the parity gate;
- the current setuptools wheel does not build or install the CMake runtime
  binaries, so removing the legacy host would silently remove a published
  entry point for wheel-only users.

Consequently `pyproject.toml` and the Python package remain intact. Deletion is
safe only after native binaries are included in, or explicitly separated from,
the distribution; the Python console-script collision is resolved; the
command publisher/simulator dependency graph is narrowed; and this parity gate
passes from an installed artifact rather than the source tree.

The retained layer is compatibility-only for deployed robot control. New
hardware deployments should use the native host and daemon; Python remains
appropriate for the SO-ARM101 simulator, the command publisher, and contract
regression tests.

## Known compatibility boundaries

The C++ codec intentionally bounds `step_id` to unsigned 64-bit and
`timestamp_ns` to signed 64-bit values. Robot profile topology is frozen at
startup, supports at most 12 CiA 402 axes, and rejects duplicate device names.
CiA 402 modes are static CSP, CSV, or CST selections; changing a mode requires
a safe stop, profile edit, and restart.

Passing fake, PTY, parity, or sanitizer tests does not qualify real EtherCAT
hardware. The final environment and hardware matrix is recorded in
[the C++ runtime qualification report](../qualification/cpp-runtime-final.md).
