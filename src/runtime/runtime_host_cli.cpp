#include "policy_runtime/runtime/runtime_host_cli.hpp"

#include <charconv>
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
    } else {
      return Result<RuntimeHostCliOptions>::failure(
          {ErrorCode::invalid_argument, "unknown argument: " + std::string(flag)});
    }
  }
  if (options.policy_profile.empty()) {
    return Result<RuntimeHostCliOptions>::failure(
        {ErrorCode::invalid_argument, "--policy-profile is required"});
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
         "[--robot-profile ROBOT_PROFILE]\n";
}

}  // namespace policy_runtime
