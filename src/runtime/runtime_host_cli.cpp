#include "policy_runtime/runtime/runtime_host_cli.hpp"

#include <charconv>
#include <limits>
#include <string>

namespace policy_runtime {
namespace {

template <class Integer>
bool parse_integer(std::string_view text, Integer& value) {
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value);
  return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
}

}  // namespace

Result<RuntimeHostCliOptions> parse_runtime_host_cli(
    std::span<const std::string_view> arguments) {
  RuntimeHostCliOptions options;
  for (std::size_t index = 1; index < arguments.size(); ++index) {
    auto flag = arguments[index];
    if (flag == "--help" || flag == "-h") {
      options.show_help = true;
      return Result<RuntimeHostCliOptions>::success(std::move(options));
    }
    std::string_view value;
    if (const auto equals = flag.find('=');
        equals != std::string_view::npos) {
      value = flag.substr(equals + 1);
      flag = flag.substr(0, equals);
    } else {
      if (index + 1 >= arguments.size()) {
        return Result<RuntimeHostCliOptions>::failure(
            {ErrorCode::invalid_argument,
             "argument requires a value: " + std::string(flag)});
      }
      value = arguments[++index];
    }
    if (flag == "--endpoint") {
      options.endpoint = value;
    } else if (flag == "--timeout-ms") {
      if (!parse_integer(value, options.timeout_ms)) {
        return Result<RuntimeHostCliOptions>::failure(
            {ErrorCode::invalid_argument, "--timeout-ms requires an integer"});
      }
    } else if (flag == "--policy-profile") {
      options.policy_profile = value;
    } else if (flag == "--robot-profile") {
      options.robot_profile = std::string(value);
    } else if (flag == "--robot-io-fd") {
      int descriptor{};
      if (!parse_integer(value, descriptor) || descriptor < 0) {
        return Result<RuntimeHostCliOptions>::failure(
            {ErrorCode::invalid_argument,
             "--robot-io-fd requires a non-negative integer"});
      }
      options.robot_io_fd = descriptor;
    } else if (flag == "--robot-io-generation") {
      std::uint64_t generation{};
      if (!parse_integer(value, generation) || generation == 0 ||
          generation > std::numeric_limits<std::uint32_t>::max()) {
        return Result<RuntimeHostCliOptions>::failure(
            {ErrorCode::invalid_argument,
             "--robot-io-generation requires a positive 32-bit integer"});
      }
      options.robot_io_generation = static_cast<std::uint32_t>(generation);
    } else if (flag == "--robot-io-socket") {
      if (value.empty()) {
        return Result<RuntimeHostCliOptions>::failure(
            {ErrorCode::invalid_argument,
             "--robot-io-socket requires a nonempty path"});
      }
      options.robot_io_socket = std::string(value);
    } else if (flag == "--robot-io-generation-file") {
      if (value.empty()) {
        return Result<RuntimeHostCliOptions>::failure(
            {ErrorCode::invalid_argument,
             "--robot-io-generation-file requires a nonempty path"});
      }
      options.robot_io_generation_file = std::string(value);
    } else {
      return Result<RuntimeHostCliOptions>::failure(
          {ErrorCode::invalid_argument, "unknown argument: " + std::string(flag)});
    }
  }
  if (options.policy_profile.empty()) {
    return Result<RuntimeHostCliOptions>::failure(
        {ErrorCode::invalid_argument, "--policy-profile is required"});
  }
  if (options.robot_io_fd.has_value() !=
      options.robot_io_generation.has_value()) {
    return Result<RuntimeHostCliOptions>::failure(
        {ErrorCode::invalid_argument,
         "--robot-io-fd and --robot-io-generation must be provided together"});
  }
  if (options.robot_io_socket.has_value() !=
      options.robot_io_generation_file.has_value()) {
    return Result<RuntimeHostCliOptions>::failure(
        {ErrorCode::invalid_argument,
         "--robot-io-socket and --robot-io-generation-file must be provided "
         "together"});
  }
  if (options.robot_io_fd.has_value() &&
      options.robot_io_socket.has_value()) {
    return Result<RuntimeHostCliOptions>::failure(
        {ErrorCode::invalid_argument,
         "robot I/O descriptor and service-path modes are mutually exclusive"});
  }
  return Result<RuntimeHostCliOptions>::success(std::move(options));
}

std::string resolve_policy_endpoint(std::string_view endpoint) {
  if (endpoint.find("://") != std::string_view::npos) {
    return std::string(endpoint);
  }
  return "ipc:///tmp/" + std::string(endpoint) + ".sock";
}

std::string runtime_host_usage() {
  return "usage: policy-runtime-host [--endpoint ENDPOINT] "
         "[--timeout-ms TIMEOUT_MS] --policy-profile POLICY_PROFILE "
         "[--robot-profile ROBOT_PROFILE] "
         "[--robot-io-fd FD --robot-io-generation GENERATION | "
         "--robot-io-socket PATH --robot-io-generation-file PATH]\n";
}

}  // namespace policy_runtime
