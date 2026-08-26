#include <charconv>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <unistd.h>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/robot_io/daemon.hpp"
#include "policy_runtime/robot_io/ipc_server.hpp"
#include "policy_runtime/robot_io/service_listener.hpp"
#include "policy_runtime/transport/ethercat/backend.hpp"
#include "policy_runtime/transport/ethercat/master.hpp"

namespace {

struct DaemonCliOptions {
  bool show_help{};
  std::string profile_path;
  std::optional<int> connected_socket_fd;
  std::optional<std::uint32_t> generation;
  std::optional<std::string> ipc_socket_path;
  std::optional<std::string> generation_file_path;
};

template <class Integer>
bool parse_integer(std::string_view text, Integer& value) {
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
}

std::string daemon_usage() {
  return "Usage:\n"
         "  robot-io-daemon ROBOT_PROFILE CONNECTED_SOCKET_FD GENERATION\n"
         "  robot-io-daemon ROBOT_PROFILE --ipc-socket PATH "
         "--generation-file PATH\n";
}

policy_runtime::Result<DaemonCliOptions> parse_cli(int argc, char** argv) {
  if (argc == 2 &&
      (std::string_view(argv[1]) == "--help" ||
       std::string_view(argv[1]) == "-h")) {
    DaemonCliOptions options;
    options.show_help = true;
    return policy_runtime::Result<DaemonCliOptions>::success(
        std::move(options));
  }
  if (argc < 2) {
    return policy_runtime::Result<DaemonCliOptions>::failure(
        {policy_runtime::ErrorCode::invalid_argument,
         "robot profile is required"});
  }

  DaemonCliOptions options;
  options.profile_path = argv[1];
  if (argc == 4 && !std::string_view(argv[2]).starts_with("--")) {
    int descriptor{-1};
    std::uint32_t generation{};
    if (!parse_integer(argv[2], descriptor) || descriptor < 0 ||
        !parse_integer(argv[3], generation) || generation == 0U) {
      return policy_runtime::Result<DaemonCliOptions>::failure(
          {policy_runtime::ErrorCode::invalid_argument,
           "invalid socket descriptor or generation"});
    }
    options.connected_socket_fd = descriptor;
    options.generation = generation;
    return policy_runtime::Result<DaemonCliOptions>::success(
        std::move(options));
  }

  for (int index = 2; index < argc; ++index) {
    std::string_view flag = argv[index];
    std::string_view value;
    if (const auto equals = flag.find('='); equals != std::string_view::npos) {
      value = flag.substr(equals + 1U);
      flag = flag.substr(0U, equals);
    } else {
      if (index + 1 >= argc) {
        return policy_runtime::Result<DaemonCliOptions>::failure(
            {policy_runtime::ErrorCode::invalid_argument,
             "argument requires a value: " + std::string(flag)});
      }
      value = argv[++index];
    }
    if (value.empty()) {
      return policy_runtime::Result<DaemonCliOptions>::failure(
          {policy_runtime::ErrorCode::invalid_argument,
           "argument requires a nonempty value: " + std::string(flag)});
    }
    if (flag == "--ipc-socket") {
      if (options.ipc_socket_path.has_value()) {
        return policy_runtime::Result<DaemonCliOptions>::failure(
            {policy_runtime::ErrorCode::invalid_argument,
             "--ipc-socket may be provided only once"});
      }
      options.ipc_socket_path = std::string(value);
    } else if (flag == "--generation-file") {
      if (options.generation_file_path.has_value()) {
        return policy_runtime::Result<DaemonCliOptions>::failure(
            {policy_runtime::ErrorCode::invalid_argument,
             "--generation-file may be provided only once"});
      }
      options.generation_file_path = std::string(value);
    } else {
      return policy_runtime::Result<DaemonCliOptions>::failure(
          {policy_runtime::ErrorCode::invalid_argument,
           "unknown argument: " + std::string(flag)});
    }
  }
  if (!options.ipc_socket_path.has_value() ||
      !options.generation_file_path.has_value()) {
    return policy_runtime::Result<DaemonCliOptions>::failure(
        {policy_runtime::ErrorCode::invalid_argument,
         "--ipc-socket and --generation-file must be provided together"});
  }
  return policy_runtime::Result<DaemonCliOptions>::success(std::move(options));
}

}  // namespace

int main(int argc, char** argv) {
  auto options = parse_cli(argc, argv);
  if (!options.has_value()) {
    std::cerr << options.error().message << '\n' << daemon_usage();
    return 2;
  }
  if (options.value().show_help) {
    std::cout << "Run the robot I/O daemon.\n" << daemon_usage();
    return 0;
  }

  auto profile =
      policy_runtime::profiles::load_robot_profile(options.value().profile_path);
  if (!profile.has_value()) {
    std::cerr << profile.error().message << '\n';
    return 1;
  }

  policy_runtime::DaemonSignalLatch signals;
  auto installed = signals.install();
  if (!installed.has_value()) {
    std::cerr << installed.error().message << '\n';
    return 1;
  }

  std::optional<policy_runtime::RobotIoServiceListener> listener;
  int socket_fd{-1};
  std::uint32_t generation{};
  if (options.value().connected_socket_fd.has_value()) {
    socket_fd = *options.value().connected_socket_fd;
    generation = *options.value().generation;
  } else {
    auto created = policy_runtime::RobotIoServiceListener::create(
        *options.value().ipc_socket_path,
        *options.value().generation_file_path, geteuid());
    if (!created.has_value()) {
      std::cerr << created.error().message << '\n';
      return 1;
    }
    listener.emplace(std::move(created.value()));
    generation = listener->generation();
    auto accepted = listener->accept_authenticated(signals);
    if (!accepted.has_value()) {
      if (signals.stop_requested()) {
        return 0;
      }
      std::cerr << accepted.error().message << '\n';
      return 1;
    }
    socket_fd = accepted.value();
  }

  const auto axis_count =
      static_cast<std::uint32_t>(profile.value().axes.size());
  const auto servo_count =
      static_cast<std::uint32_t>(profile.value().st3215_servos.size());
  auto ipc = servo_count == 0U
                 ? policy_runtime::RobotIoIpcServer::create(
                       socket_fd, axis_count, generation)
                 : policy_runtime::RobotIoIpcServer::create(
                       socket_fd, axis_count, servo_count, generation);
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
  auto started = daemon.start();
  if (!started.has_value()) {
    std::cerr << started.error().message << '\n';
    return 1;
  }

  daemon.run();
  return 0;
}
