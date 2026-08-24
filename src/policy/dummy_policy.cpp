#include "policy_runtime/policy/policy.hpp"

#include <memory>
#include <utility>

namespace policy_runtime {
namespace {

class DummyPolicy final : public Policy {
 public:
  explicit DummyPolicy(profiles::PolicyProfile profile)
      : profile_(std::move(profile)) {}

  Result<nlohmann::json> infer(const nlohmann::json& observation,
                               const PolicySession&) override {
    nlohmann::json action = nlohmann::json::object();
    if (!profile_.inputs.empty()) {
      for (std::size_t index = 0; index < profile_.inputs.size(); ++index) {
        const auto& input = profile_.inputs[index];
        const auto& output = profile_.outputs[index];
        const auto found = observation.find(input.canonical_field);
        action[output.canonical_field] =
            found == observation.end() ? nlohmann::json(nullptr) : *found;
      }
      return Result<nlohmann::json>::success(std::move(action));
    }
    if (const auto joint = observation.find("joint_position");
        joint != observation.end()) {
      action["joint_position_delta"] = *joint;
    }
    if (const auto gripper = observation.find("gripper_width");
        gripper != observation.end()) {
      action["gripper_command"] = *gripper;
    }
    return Result<nlohmann::json>::success(std::move(action));
  }

  void reset(const std::string&) noexcept override {}

  nlohmann::json capabilities() const override {
    return {{"backend", "dummy"},
            {"modalities",
             {"joint_position", "gripper_width", "task_text"}},
            {"temporal_modes", {"single_step"}}};
  }

 private:
  profiles::PolicyProfile profile_;
};

class Pi0LikePolicy final : public Policy {
 public:
  explicit Pi0LikePolicy(const profiles::PolicyProfile& profile)
      : checkpoint_(string_config(profile.model, "checkpoint", "")),
        device_(string_config(profile.model, "device", "cpu")),
        precision_(string_config(profile.model, "precision", "fp16")) {}

  Result<nlohmann::json> infer(const nlohmann::json& observation,
                               const PolicySession& session) override {
    const auto joint = observation.find("joint_position");
    const auto joint_value =
        joint != observation.end() && joint->is_object()
            ? *joint
            : nlohmann::json::object();
    const auto values = joint_value.value(
        "values", nlohmann::json::array());
    const auto names = joint_value.value(
        "joint_names", nlohmann::json::array());
    const auto unit = joint_value.value("unit", std::string("rad"));
    nlohmann::json deltas = nlohmann::json::array();
    for (std::size_t index = 0; index < values.size(); ++index) {
      deltas.push_back(index % 2 == 0 ? 0.02 : -0.02);
    }

    double gripper_value = 0.04;
    if (const auto gripper = observation.find("gripper_width");
        gripper != observation.end() && gripper->is_object()) {
      gripper_value = gripper->value("value", 0.04);
    }
    nlohmann::json chunk = nlohmann::json::array();
    for (std::size_t index = 0; index < 4; ++index) {
      chunk.push_back(
          {{"joint_position_delta",
            {{"values", deltas}, {"joint_names", names}, {"unit", unit}}},
           {"gripper_command", {{"value", gripper_value}, {"unit", "m"}}},
           {"meta", {{"chunk_index", index}, {"step_id", session.step_id}}}});
    }
    return Result<nlohmann::json>::success(
        {{"action_chunk", std::move(chunk)},
         {"meta",
          {{"backend", "pi0_like_placeholder"},
           {"checkpoint", checkpoint_},
           {"device", device_},
           {"precision", precision_}}}});
  }

  void reset(const std::string&) noexcept override {}

  nlohmann::json capabilities() const override {
    return {{"backend", "pytorch_placeholder"},
            {"modalities", {"image", "joint_position", "task_text"}},
            {"temporal_modes", {"single_step", "chunked"}}};
  }

 private:
  static std::string string_config(const nlohmann::json& config,
                                   std::string_view key,
                                   std::string fallback) {
    const auto found = config.find(std::string(key));
    if (found == config.end()) {
      return fallback;
    }
    return found->is_string() ? found->get<std::string>() : found->dump();
  }

  std::string checkpoint_;
  std::string device_;
  std::string precision_;
};

}  // namespace

Result<std::unique_ptr<Policy>> make_policy(
    const profiles::PolicyProfile& profile) {
  if (profile.policy != "dummy") {
    if (profile.policy == "pi0_like") {
      return Result<std::unique_ptr<Policy>>::success(
          std::make_unique<Pi0LikePolicy>(profile));
    }
    return Result<std::unique_ptr<Policy>>::failure(
        {ErrorCode::invalid_argument, "unknown policy: " + profile.policy});
  }
  return Result<std::unique_ptr<Policy>>::success(
      std::make_unique<DummyPolicy>(profile));
}

}  // namespace policy_runtime
