#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <sys/socket.h>
#include <sys/time.h>

#include "policy_runtime/runtime/robot_io_client.hpp"
#include "policy_runtime/runtime/robot_io_service.hpp"
#include "policy_runtime/runtime/runtime_host.hpp"
#include "policy_runtime/runtime/runtime_host_cli.hpp"

#if POLICY_RUNTIME_HAS_ZMQ
#include <zmq.hpp>
#endif

namespace {

volatile std::sig_atomic_t stop_requested = 0;

extern "C" void request_stop(int) { stop_requested = 1; }

#if POLICY_RUNTIME_HAS_ZMQ

std::optional<std::filesystem::path> ipc_path(std::string_view endpoint) {
  constexpr std::string_view prefix = "ipc://";
  if (!endpoint.starts_with(prefix)) {
    return std::nullopt;
  }
  return std::filesystem::path(endpoint.substr(prefix.size()));
}

class ZmqRepServer {
 public:
  ZmqRepServer(std::string endpoint, int timeout_ms)
      : endpoint_(std::move(endpoint)), timeout_ms_(timeout_ms), context_(1) {
    reopen();
  }

  ~ZmqRepServer() { cleanup_ipc_path(); }

  bool serve_one(policy_runtime::RuntimeHost& host) {
    try {
      zmq::message_t request;
      auto received = socket_->recv(request, zmq::recv_flags::none);
      if (!received.has_value()) {
        return false;
      }
      const std::string_view request_text(
          static_cast<const char*>(request.data()), request.size());
      const auto response = host.handle_text(request_text);
      auto sent = socket_->send(zmq::buffer(response), zmq::send_flags::none);
      if (!sent.has_value()) {
        reopen();
      }
      return true;
    } catch (const zmq::error_t& error) {
      if (error.num() != EAGAIN && error.num() != EINTR) {
        reopen();
      }
      return false;
    }
  }

 private:
  void reopen() {
    socket_.reset();
    cleanup_ipc_path();
    if (const auto path = ipc_path(endpoint_); path.has_value()) {
      std::error_code error;
      std::filesystem::create_directories(path->parent_path(), error);
      if (error) {
        throw std::runtime_error("unable to create ZeroMQ IPC directory: " +
                                 error.message());
      }
    }
    socket_.emplace(context_, zmq::socket_type::rep);
    socket_->set(zmq::sockopt::linger, 0);
    socket_->set(zmq::sockopt::rcvtimeo, timeout_ms_);
    socket_->set(zmq::sockopt::sndtimeo, timeout_ms_);
    socket_->bind(endpoint_);
  }

  void cleanup_ipc_path() noexcept {
    if (const auto path = ipc_path(endpoint_); path.has_value()) {
      std::error_code ignored;
      std::filesystem::remove(*path, ignored);
    }
  }

  std::string endpoint_;
  int timeout_ms_{};
  zmq::context_t context_;
  std::optional<zmq::socket_t> socket_;
};

#endif

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string_view> arguments;
  arguments.reserve(static_cast<std::size_t>(argc));
  for (int index = 0; index < argc; ++index) {
    arguments.emplace_back(argv[index]);
  }
  auto options = policy_runtime::parse_runtime_host_cli(arguments);
  if (!options.has_value()) {
    std::cerr << options.error().message << '\n'
              << policy_runtime::runtime_host_usage();
    return 2;
  }
  if (options.value().show_help) {
    std::cout << "Run the robot runtime host.\n"
              << policy_runtime::runtime_host_usage();
    return 0;
  }
  if (options.value().timeout_ms <= 0) {
    std::cerr << "--timeout-ms must be positive\n";
    return 2;
  }

#if !POLICY_RUNTIME_HAS_ZMQ
  std::cerr << "policy-runtime-host was built without ZeroMQ/cppzmq support\n";
  return 1;
#else
  std::unique_ptr<policy_runtime::RuntimeRobotIo> robot_io;
  if (options.value().robot_io_fd.has_value()) {
    timeval timeout{options.value().timeout_ms / 1000,
                    (options.value().timeout_ms % 1000) * 1000};
    if (setsockopt(*options.value().robot_io_fd, SOL_SOCKET, SO_RCVTIMEO,
                   &timeout, sizeof(timeout)) != 0) {
      std::cerr << "unable to set robot I/O setup timeout\n";
      return 1;
    }
    auto client = policy_runtime::RobotIoClient::connect(
        *options.value().robot_io_fd,
        *options.value().robot_io_generation);
    if (!client.has_value()) {
      std::cerr << client.error().message << '\n';
      return 1;
    }
    robot_io = policy_runtime::make_runtime_robot_io(
        std::move(client.value()));
  } else if (options.value().robot_io_socket.has_value()) {
    auto client = policy_runtime::connect_robot_io_service(
        *options.value().robot_io_socket,
        *options.value().robot_io_generation_file,
        options.value().timeout_ms);
    if (!client.has_value()) {
      std::cerr << client.error().message << '\n';
      return 1;
    }
    robot_io = policy_runtime::make_runtime_robot_io(
        std::move(client.value()));
  }

  const std::filesystem::path robot_profile =
      options.value().robot_profile.value_or(
          "policy_embodied_runtime/examples/robot_profiles/"
          "default_rpc_robot_profile.json");
  auto host = policy_runtime::RuntimeHost::from_profiles(
      options.value().policy_profile, robot_profile, std::move(robot_io));
  if (!host.has_value()) {
    std::cerr << host.error().message << '\n';
    return 1;
  }
  auto opened = host.value().open();
  if (!opened.has_value()) {
    std::cerr << opened.error().message << '\n';
    return 1;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  try {
    ZmqRepServer server(
        policy_runtime::resolve_policy_endpoint(options.value().endpoint),
        options.value().timeout_ms);
    while (stop_requested == 0) {
      static_cast<void>(server.serve_one(host.value()));
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    host.value().close();
    return 1;
  }
  host.value().close();
  return 0;
#endif
}
