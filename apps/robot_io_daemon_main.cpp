#include <iostream>
#include <string_view>

#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/robot_io/daemon/daemon.hpp"

namespace {
constexpr std::string_view kUsage = "Usage: robot-io-daemon ROBOT_PROFILE\n";
}  // namespace

int main(int argc, char** argv) {
  // The executable is intentionally a thin composition root. Physical
  // transports and directional devices are created by RobotIoDaemon from the
  // validated profile; this file must not construct EtherCAT/CAN objects.
  if (argc == 2 && (std::string_view(argv[1]) == "--help" ||
                    std::string_view(argv[1]) == "-h")) {
    std::cout << "Run the DDS robot I/O daemon.\n" << kUsage;
    return 0;
  }
  if (argc != 2) {
    std::cerr << kUsage;
    return 2;
  }
  // Load and validate the static robot description before opening hardware.
  auto profile = policy_runtime::profiles::load_robot_profile(argv[1]);
  if (!profile.has_value()) {
    std::cerr << profile.error().message << '\n';
    return 1;
  }
  if (profile.value().dds.backend !=
      policy_runtime::profiles::RobotIoBackend::dds) {
    std::cerr << "robot profile must select the DDS backend\n";
    return 1;
  }
  // SIGTERM is converted into a daemon shutdown request. The signal handler
  // itself only records the request; shutdown runs in the normal control flow.
  policy_runtime::DaemonSignalLatch signals;
  auto installed = signals.install();
  if (!installed.has_value()) {
    std::cerr << installed.error().message << '\n';
    return 1;
  }
  // Configure compiles the physical topology and prepares all device state,
  // but does not start realtime workers or accept commands yet.
  policy_runtime::RobotIoDaemon daemon;
  auto configured = daemon.configure(profile.value());
  if (!configured.has_value()) {
    std::cerr << configured.error().message << '\n';
    return 1;
  }
  // DDS is the external host/daemon boundary. Topic endpoints are created
  // after configuration and before transport workers start.
  auto attached_dds = daemon.attach_dds(profile.value());
  if (!attached_dds.has_value()) {
    std::cerr << attached_dds.error().message << '\n';
    return 1;
  }
  // start() creates the configured Transport instances (including IgH
  // EtherCAT), opens them, and starts their workers/scheduler.
  auto started = daemon.start();
  if (!started.has_value()) {
    std::cerr << started.error().message << '\n';
    return 1;
  }
  // run() owns the daemon lifecycle until shutdown is requested or a safety
  // condition completes the stop sequence.
  auto ran = daemon.run();
  if (!ran.has_value()) {
    std::cerr << ran.error().message << '\n';
    return 1;
  }
  return 0;
}
