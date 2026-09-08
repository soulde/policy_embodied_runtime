#include <iostream>
#include <string_view>

#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/robot_io/daemon.hpp"

namespace {
constexpr std::string_view kUsage = "Usage: robot-io-daemon ROBOT_PROFILE\n";
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && (std::string_view(argv[1]) == "--help" ||
                    std::string_view(argv[1]) == "-h")) {
    std::cout << "Run the DDS robot I/O daemon.\n" << kUsage;
    return 0;
  }
  if (argc != 2) {
    std::cerr << kUsage;
    return 2;
  }
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
  policy_runtime::DaemonSignalLatch signals;
  auto installed = signals.install();
  if (!installed.has_value()) {
    std::cerr << installed.error().message << '\n';
    return 1;
  }
  policy_runtime::RobotIoDaemon daemon;
  auto configured = daemon.configure(profile.value());
  if (!configured.has_value()) {
    std::cerr << configured.error().message << '\n';
    return 1;
  }
  auto attached_dds = daemon.attach_dds(profile.value());
  if (!attached_dds.has_value()) {
    std::cerr << attached_dds.error().message << '\n';
    return 1;
  }
  auto started = daemon.start();
  if (!started.has_value()) {
    std::cerr << started.error().message << '\n';
    return 1;
  }
  auto ran = daemon.run();
  if (!ran.has_value()) {
    std::cerr << ran.error().message << '\n';
    return 1;
  }
  return 0;
}
