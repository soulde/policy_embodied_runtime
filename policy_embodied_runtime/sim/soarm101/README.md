# SO-ARM101 MuJoCo Simulator

This simulator was migrated from `soulde/rustyRobot/python/soarm101_sim`.
It runs a MuJoCo SO-ARM101 model behind a virtual ST3215 serial bus.

Install project dependencies:

```bash
uv pip install -e ".[dev]"
```

Run the simulator:

```bash
policy-soarm101-sim
```

The simulator creates `/tmp/rusty_robot_soarm101` as a stable symlink to the
current `/dev/pts/N`. A robot-side ST3215 config can point at:

```cfg
path = /tmp/rusty_robot_soarm101
```

Verify the virtual serial loop:

```bash
policy-soarm101-verify-serial
```

Supported ST3215 subset:

- `PING`
- `READ DATA` for present position at address `0x38`
- `WRITE DATA` for goal position at address `0x2A`

Servo IDs `1..6` map to:

1. `shoulder_pan`
2. `shoulder_lift`
3. `elbow_flex`
4. `wrist_flex`
5. `wrist_roll`
6. `gripper`

MJCF assets are copied from `google-deepmind/mujoco_menagerie/robotstudio_so101`
and keep their upstream Apache-2.0 license in
`policy_embodied_runtime/sim/soarm101_assets/robotstudio_so101/LICENSE`.
