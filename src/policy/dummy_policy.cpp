#include "policy_runtime/policy/policy.hpp"

#include <memory>
#include <utility>

namespace policy_runtime {
namespace {

std::string python_string_repr(std::string_view value) {
  const bool has_single_quote = value.find('\'') != std::string_view::npos;
  const bool has_double_quote = value.find('"') != std::string_view::npos;
  const char quote = has_single_quote && !has_double_quote ? '"' : '\'';
  std::string result(1, quote);
  constexpr char hex_digits[] = "0123456789abcdef";
  for (const auto character : value) {
    const auto byte = static_cast<unsigned char>(character);
    if (character == quote || character == '\\') {
      result.push_back('\\');
      result.push_back(character);
    } else if (character == '\n') {
      result += "\\n";
    } else if (character == '\r') {
      result += "\\r";
    } else if (character == '\t') {
      result += "\\t";
    } else if (byte < 0x20 || byte == 0x7f) {
      result += "\\x";
      result.push_back(hex_digits[(byte >> 4U) & 0x0fU]);
      result.push_back(hex_digits[byte & 0x0fU]);
    } else {
      result.push_back(character);
    }
  }
  result.push_back(quote);
  return result;
}

std::string python_repr(const nlohmann::ordered_json& value) {
  if (value.is_null()) {
    return "None";
  }
  if (value.is_boolean()) {
    return value.get<bool>() ? "True" : "False";
  }
  if (value.is_string()) {
    return python_string_repr(value.get_ref<const std::string&>());
  }
  if (value.is_array()) {
    std::string result = "[";
    for (std::size_t index = 0; index < value.size(); ++index) {
      if (index != 0) {
        result += ", ";
      }
      result += python_repr(value[index]);
    }
    return result + "]";
  }
  if (value.is_object()) {
    std::string result = "{";
    bool first = true;
    for (auto item = value.begin(); item != value.end(); ++item) {
      if (!first) {
        result += ", ";
      }
      first = false;
      result += python_string_repr(item.key()) + ": " +
                python_repr(item.value());
    }
    return result + "}";
  }
  return value.dump();
}

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
  static std::string string_config(const nlohmann::ordered_json& config,
                                   std::string_view key,
                                   std::string fallback) {
    const auto found = config.find(std::string(key));
    if (found == config.end()) {
      return fallback;
    }
    return found->is_string() ? found->get<std::string>()
                              : python_repr(*found);
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
