#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/runtime/runtime_host.hpp"

namespace {

nlohmann::json summarize_devices(
    const std::vector<policy_runtime::profiles::DeviceConfig>& devices) {
  auto result = nlohmann::json::array();
  for (const auto& device : devices) {
    result.push_back({{"name", device.name},
                      {"type", device.device.type},
                      {"path", device.device.path},
                      {"args", device.args}});
  }
  return result;
}

int check_policy_profile(const char* path) {
  auto profile = policy_runtime::profiles::load_policy_profile(path);
  if (!profile.has_value()) {
    std::cout << nlohmann::json{{"accepted", false}} << '\n';
    return 0;
  }
  auto observation_fields = nlohmann::json::array();
  for (const auto& field : profile.value().canonical_observation_schema) {
    observation_fields.push_back(field.name);
  }
  auto action_fields = nlohmann::json::array();
  for (const auto& field : profile.value().canonical_action_schema) {
    action_fields.push_back(field.name);
  }
  std::cout << nlohmann::json{
                   {"accepted", true},
                   {"id", profile.value().id},
                   {"version", profile.value().version},
                   {"policy", profile.value().policy},
                   {"observation_fields", std::move(observation_fields)},
                   {"action_fields", std::move(action_fields)},
                   {"input_count", profile.value().inputs.size()},
                   {"output_count", profile.value().outputs.size()},
                   {"action_horizon", profile.value().temporal.action_horizon},
                   {"observation_history",
                    profile.value().temporal.observation_history},
               }
            << '\n';
  return 0;
}

int check_robot_profile(const char* path) {
  auto profile = policy_runtime::profiles::load_robot_profile(path);
  if (!profile.has_value()) {
    std::cout << nlohmann::json{{"accepted", false}} << '\n';
    return 0;
  }
  std::cout << nlohmann::json{
                   {"accepted", true},
                   {"sensors", summarize_devices(profile.value().sensors)},
                   {"actuators", summarize_devices(profile.value().actuators)},
               }
            << '\n';
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 3 && std::string_view(argv[1]) == "--check-policy-profile") {
    return check_policy_profile(argv[2]);
  }
  if (argc == 3 && std::string_view(argv[1]) == "--check-robot-profile") {
    return check_robot_profile(argv[2]);
  }
  if (argc != 2) {
    std::cerr << "usage: runtime_contract_driver POLICY_PROFILE\n"
                 "       runtime_contract_driver --check-policy-profile PATH\n"
                 "       runtime_contract_driver --check-robot-profile PATH\n";
    return 2;
  }
  auto host = policy_runtime::RuntimeHost::from_profiles(argv[1]);
  if (!host.has_value()) {
    std::cerr << host.error().message << '\n';
    return 1;
  }
  auto opened = host.value().open();
  if (!opened.has_value()) {
    std::cerr << opened.error().message << '\n';
    return 1;
  }
  std::string request;
  while (std::getline(std::cin, request)) {
    std::cout << host.value().handle_text(request) << '\n';
    std::cout.flush();
  }
  host.value().close();
  return 0;
}
