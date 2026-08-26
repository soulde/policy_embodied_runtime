#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime {

struct RuntimeHostCliOptions {
  std::string endpoint = "embodied-policy-runtime";
  int timeout_ms = 100;
  std::string policy_profile;
  std::optional<std::string> robot_profile;
  std::optional<int> robot_io_fd;
  std::optional<std::uint32_t> robot_io_generation;
  std::optional<std::string> robot_io_socket;
  std::optional<std::string> robot_io_generation_file;
  bool show_help{};
};

Result<RuntimeHostCliOptions> parse_runtime_host_cli(
    std::span<const std::string_view> arguments);
std::string resolve_policy_endpoint(std::string_view endpoint);
std::string runtime_host_usage();

}  // namespace policy_runtime
