# C++ runtime final qualification report

Date: 2026-08-26

This report separates reproducible source-tree checks from target-specific
qualification. A passing development-host matrix is not evidence of a
production 1 kHz guarantee.

## Development-host evidence

| Check | Result | Evidence |
|---|---|---|
| Python/C++ runtime parity | PASS | `pytest tests/compat/run_runtime_parity.py -q`: 4 passed |
| Direct parity driver | PASS | 4 runtime/profile scenarios passed |
| x86_64 clean CMake build | Pending final run | IgH disabled; CMake pinned to >=3.24 |
| Full CTest | Pending final run | AF_UNIX restrictions, if observed, are reported rather than treated as product passes |
| Full Python regression | Pending final run | Socket and non-socket results reported separately |
| ASan + UBSan | Pending final run | Hardware-free suite |
| TSan | Pending final run | Runtime availability is environment-dependent |
| Simulator CLI and install layout | Pending final run | Help must not touch hardware |

## Not verified in this environment

| Acceptance item | Status | Required evidence |
|---|---|---|
| ARM64 build and execution | NOT VERIFIED | Native ARM64 clean build, CTest, parity, and sanitizer results |
| PREEMPT_RT 1 kHz behavior | NOT VERIFIED | RT kernel, affinity/priority setup, deadline and safe-stop measurements |
| Real IgH linkage and master operation | NOT VERIFIED | Build with `POLICY_RUNTIME_WITH_IGH=ON` against the deployment IgH release |
| Elmo Gold CSP qualification | NOT VERIFIED | Enable/move/stop/fault recovery on the specified drive revision |
| Elmo Gold CSV qualification | NOT VERIFIED | Velocity-mode functional and safety results |
| Elmo Gold CST qualification | NOT VERIFIED | Torque-mode functional and safety results |
| Cable loss and WKC fault recovery | NOT VERIFIED | Measured WKC sequence, group response, QuickStop, and Disable behavior |
| Host crash, daemon shutdown, command timeout, drive fault | NOT VERIFIED | HIL fault-injection logs and recovery acceptance |
| Twelve-axis hardware-in-the-loop | NOT VERIFIED | Frozen 12-axis topology on representative hardware |
| One-hour 12-axis fake/HIL soak | NOT VERIFIED | No torn snapshots plus complete cycle/IPC statistics for at least one hour |
| Maximum cycle time | NOT MEASURED | PREEMPT_RT qualification log |
| Deadline misses | NOT MEASURED | PREEMPT_RT qualification log |
| DC deviation | NOT MEASURED | IgH/Elmo qualification log |
| WKC failures | NOT MEASURED | IgH/Elmo qualification log |
| Safe-stop latency | NOT MEASURED | Hardware fault-injection timestamps |

Release approval for production EtherCAT control remains blocked until every
target-specific item above is accompanied by captured configuration, logs,
metrics, and operator sign-off.
