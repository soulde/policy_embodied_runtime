# Task 2: Bounded IDL and Topic Registry

## Outcome

Added the DDS IDL contract and static topic registry.  The registry creates
independent command/state topics for configured actuator/sensor endpoints,
normalizes topic components, rejects invalid or colliding normalized names,
creates one EtherCAT commit topic per actuator safety group, and creates one
health topic per robot.

`robot_io.idl` defines bounded, device-specific CiA402, Damiao, and ST3215
command/state messages plus bounded EtherCAT commit and health messages.  The
common header carries schema version, robot/device/session identity, sequence,
source timestamp, bus cycle, and flags; no native C++ layout is serialized.

## Robot-ID decision for Task 4

`RobotProfile` does not yet have `dds.robot_id`, so
`build_dds_topic_registry` requires an explicit `std::string_view robot_id`.
There is deliberately no default or one-argument overload: a silent shared
`"robot"` namespace could make two configured robots use the same DDS topics
and weaken collision protection.  Task 4 should call this API with
`profile.dds.robot_id` after it adds that field.

## Verification

- DDS-off configure/build: `dds_topic_registry_test` builds successfully.
- Focused DDS-off regression: `dds_topic_registry_test` and
  `profile_loader_test` pass (31 tests).
- Fake Cyclone DDS configuration and generated-type target build pass; this
  verifies the package-gated IDL wiring without requiring locally unavailable
  Cyclone DDS packages.
- The explicit robot-namespace test was mutation-checked by temporarily
  replacing the supplied namespace with a fixed value; it failed with the
  expected topic-name mismatch, then passed after restoration.

## Limitation

No real Cyclone DDS installation is available in this workspace, so generated
IDL code was probe-tested with the existing fake package rather than compiled
against the real generator.
