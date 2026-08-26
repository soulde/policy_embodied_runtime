#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"

namespace {

std::filesystem::path source_path(const char* relative_path) {
  return std::filesystem::path(POLICY_RUNTIME_SOURCE_DIR) / relative_path;
}

std::string invoke_tool(const char* name, const char* argument, int& status) {
  const auto output = std::filesystem::temp_directory_path() /
                      (std::string("policy-runtime-") + name + ".out");
  const auto tool = std::filesystem::path(POLICY_RUNTIME_TOOL_DIR) / name;
  const auto command = "\"" + tool.string() + "\" " + argument +
                       " > \"" + output.string() + "\" 2>&1";
  status = std::system(command.c_str());
  std::ifstream input(output);
  const std::string text{std::istreambuf_iterator<char>(input),
                         std::istreambuf_iterator<char>()};
  std::filesystem::remove(output);
  return text;
}

TEST(ServiceConfigTest, ExampleProfileHasStaticModesAndTimeouts) {
  auto profile = policy_runtime::profiles::load_robot_profile(
      source_path("configs/robot_profiles/elmo_gold_example.json"));

  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  ASSERT_EQ(profile.value().axes.size(), 3U);

  std::array<bool, 3> modes{};
  for (const auto& axis : profile.value().axes) {
    EXPECT_GT(axis.command_timeout, std::chrono::milliseconds::zero());
    EXPECT_FALSE(axis.name.empty());
    switch (axis.mode) {
      case policy_runtime::profiles::Cia402Mode::csp:
        modes[0] = true;
        break;
      case policy_runtime::profiles::Cia402Mode::csv:
        modes[1] = true;
        break;
      case policy_runtime::profiles::Cia402Mode::cst:
        modes[2] = true;
        break;
    }
  }
  EXPECT_EQ(modes, (std::array<bool, 3>{true, true, true}));
}

TEST(ServiceConfigTest, DiagnosticToolsAreSafeWhenNoDaemonIsAvailable) {
  for (const char* tool : {"robot-io-health", "ethercat-cycle-stats"}) {
    int help_status{};
    const auto help = invoke_tool(tool, "--help", help_status);
    EXPECT_EQ(help_status, 0) << tool << ": " << help;
    EXPECT_NE(help.find("Usage:"), std::string::npos) << tool << ": " << help;

    int diagnostic_status{};
    const auto diagnostic = invoke_tool(tool, "", diagnostic_status);
    EXPECT_NE(diagnostic_status, 0) << tool;
    EXPECT_NE(diagnostic.find("unavailable"), std::string::npos)
        << tool << ": " << diagnostic;
  }
}

}  // namespace
