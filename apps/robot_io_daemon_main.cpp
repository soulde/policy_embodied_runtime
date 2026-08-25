#include <cerrno>
#include <charconv>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string_view>
#include <vector>

#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/robot_io/daemon.hpp"
#include "policy_runtime/robot_io/ipc_server.hpp"
#include "policy_runtime/transport/ethercat/backend.hpp"
#include "policy_runtime/transport/ethercat/master.hpp"

namespace {

int parse_descriptor(std::string_view text) {
  int descriptor{-1};
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), descriptor);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size()
             ? descriptor
             : -1;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: robot-io-daemon ROBOT_PROFILE CONNECTED_SOCKET_FD GENERATION\n";
    return 2;
  }
  const auto socket_fd = parse_descriptor(argv[2]);
  const auto generation_value = parse_descriptor(argv[3]);
  if (socket_fd < 0 || generation_value <= 0) {
    std::cerr << "invalid socket descriptor or generation\n";
    return 2;
  }
  auto profile = policy_runtime::profiles::load_robot_profile(argv[1]);
  if (!profile.has_value()) {
    std::cerr << profile.error().message << '\n';
    return 1;
  }
  const auto axis_count =
      static_cast<std::uint32_t>(profile.value().axes.size());
  const auto servo_count =
      static_cast<std::uint32_t>(profile.value().st3215_servos.size());
  auto ipc = servo_count == 0U
                 ? policy_runtime::RobotIoIpcServer::create(
                       socket_fd, axis_count,
                       static_cast<std::uint32_t>(generation_value))
                 : policy_runtime::RobotIoIpcServer::create(
                       socket_fd, axis_count, servo_count,
                       static_cast<std::uint32_t>(generation_value));
  if (!ipc.has_value()) {
    std::cerr << ipc.error().message << '\n';
    return 1;
  }

#if POLICY_RUNTIME_WITH_IGH
  auto backend = std::make_shared<policy_runtime::IghBackend>();
  std::vector<policy_runtime::EthercatAxisConfiguration> configurations;
  configurations.reserve(profile.value().axes.size());
  for (const auto& axis : profile.value().axes) {
    configurations.push_back({axis, {}});
  }
  policy_runtime::EthercatMaster master{backend, std::move(configurations)};
#endif

  policy_runtime::RobotIoDaemon daemon;
  auto configured = daemon.configure(profile.value());
  if (!configured.has_value()) {
    std::cerr << configured.error().message << '\n';
    return 1;
  }
#if POLICY_RUNTIME_WITH_IGH
  auto attached_master = daemon.attach_ethercat(master);
  if (!attached_master.has_value()) {
    std::cerr << attached_master.error().message << '\n';
    return 1;
  }
#else
  if (!profile.value().axes.empty()) {
    std::cerr << "robot-io-daemon was built without IgH EtherCAT support\n";
    return 1;
  }
#endif
  auto attached_ipc = daemon.attach_ipc(std::move(ipc.value()));
  if (!attached_ipc.has_value()) {
    std::cerr << attached_ipc.error().message << '\n';
    return 1;
  }
  policy_runtime::DaemonSignalLatch signals;
  auto installed = signals.install();
  if (!installed.has_value()) {
    std::cerr << installed.error().message << '\n';
    return 1;
  }
  auto started = daemon.start();
  if (!started.has_value()) {
    std::cerr << started.error().message << '\n';
    return 1;
  }

  daemon.run();
  return 0;
}
