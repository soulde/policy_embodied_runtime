# Task 1: Offline Cyclone DDS Build Integration

## Result

Completed the inherited Cyclone DDS CMake seam. The native build now exposes
the `POLICY_RUNTIME_WITH_DDS` option, reports `POLICY_RUNTIME_HAS_DDS`, exports
the selected `CycloneDDS::ddsc` and `CycloneDDS-CXX::ddscxx` target names, and
provides `policy_runtime_add_dds_types(TARGET ... IDL ...)` through the system
Cyclone DDS C++ IDL generator. DDS discovery is strict and package-only when
enabled; no Cyclone DDS source, URL, package-manager command, FetchContent, or
ExternalProject path was added.

## Inherited-state assessment

The previous agent had already added the root CMake include and configure-time
probe registration, plus the new module and fake package fixtures. The
inherited implementation matched the task interfaces and passed review for
target names and `idlcxx_generate(TARGET ... FILES ...)` usage, so no unrelated
changes were discarded. The generated `uv.lock` and local `build-task1/`
directory were test artifacts; both were removed. No Task 2 IDL types were
added.

## Files

- `CMakeLists.txt`: includes the DDS seam and registers the fake-package
  `cyclonedds_package_probe` CTest.
- `cmake/PolicyRuntimeDds.cmake`: strict optional package discovery, target
  variables, and IDL-generation helper.
- `tests/cmake/cyclonedds_probe/CMakeLists.txt`: enables DDS against the fake
  packages and asserts the exported interface.
- `tests/cmake/cyclonedds_probe/fake/CycloneDDSConfig.cmake`: fake C target.
- `tests/cmake/cyclonedds_probe/fake/CycloneDDS-CXXConfig.cmake`: fake C++
  target and IDL generator command.
- `tests/cmake/cyclonedds_probe/probe.idl`: minimal generator input fixture.

## Verification

All commands below were run in the `dds-ipc` worktree with the installed
CMake 4.1.3 toolchain:

- `cmake -S tests/cmake/cyclonedds_probe -B /tmp/policy-runtime-dds-probe-final -DPOLICY_RUNTIME_SOURCE_DIR=$PWD -DCMAKE_PREFIX_PATH=$PWD/tests/cmake/cyclonedds_probe/fake`: configure passed with fake DDS packages and generated target assertion.
- `cmake -S . -B /tmp/policy-runtime-dds-task1-build -DPOLICY_RUNTIME_WITH_IGH=OFF -DPOLICY_RUNTIME_WITH_DDS=OFF -DPOLICY_RUNTIME_BUILD_TESTS=ON -DGTest_DIR=/usr/lib/x86_64-linux-gnu/cmake/GTest`: configure passed in DDS-off mode.
- `ctest --test-dir /tmp/policy-runtime-dds-task1-build -R 'cyclonedds_package_probe|cppzmq_.*_api_probe' --output-on-failure`: **5/5 passed** (Cyclone DDS probe plus four existing cppzmq probes).
- `cmake --build /tmp/policy-runtime-dds-task1-build -j2`: completed successfully; all configured targets built. The existing `st3215_test.cpp` unused-function warning remains unrelated.
- DDS-on configure without a package: failed as required with the direct
  missing `CycloneDDSConfig.cmake` diagnostic.
- `git diff --check`: clean.

## Self-review

The helper rejects malformed arguments and use while DDS is disabled, and the
probe verifies both imported target names and that the IDL generator receives
the requested input. The root test runs from a normal DDS-off project
configuration but configures its child probe with the checked-in fake package,
so it is deterministic and offline. No production runtime or Task 2 IDL code
was touched.

## Concerns

An actual DDS-enabled build remains dependent on system-installed Cyclone DDS C
and Cyclone DDS C++ development packages exporting the standard targets and
`idlcxx_generate` command. None are installed in this environment, so only
the fake-package enablement path could be exercised here. The two known Unix
IPC timing tests from the baseline were not changed or reclassified.
