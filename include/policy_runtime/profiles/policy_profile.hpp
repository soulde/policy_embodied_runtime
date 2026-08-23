#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace policy_runtime::profiles {

enum class SemanticKind { scalar, vector, text, image, object };

struct SemanticBounds {
  std::optional<double> lower;
  std::optional<double> upper;
};

struct SemanticField {
  std::string name;
  std::string semantic_type;
  SemanticKind kind = SemanticKind::scalar;
  std::optional<std::string> unit;
  std::vector<std::string> ordering;
  std::optional<SemanticBounds> bounds;
  std::optional<std::string> frame;
  bool normalized = false;
  bool optional = false;
  std::optional<std::string> description;
};

enum class TemporalMode { single_step, chunked };

struct TemporalSpec {
  TemporalMode mode = TemporalMode::single_step;
  std::uint64_t action_horizon = 1;
  std::uint64_t observation_history = 1;
};

struct ProcessorSpec {
  std::string name;
  nlohmann::json params = nlohmann::json::object();
};

struct RobotPolicyBinding {
  std::string name;
  std::string robot_data;
  std::string canonical_field;
  std::optional<std::string> source_path;
  std::optional<std::string> target_path;
  bool optional = false;
  nlohmann::json metadata = nlohmann::json::object();
};

struct PolicyProfile {
  std::string id;
  std::string version;
  std::string policy;
  nlohmann::json model = nlohmann::json::object();
  std::vector<RobotPolicyBinding> inputs;
  std::vector<RobotPolicyBinding> outputs;
  std::vector<SemanticField> canonical_observation_schema;
  std::vector<SemanticField> canonical_action_schema;
  TemporalSpec temporal;
  std::vector<ProcessorSpec> preprocess;
  std::vector<ProcessorSpec> postprocess;
  nlohmann::json safety = nlohmann::json::object();
};

}  // namespace policy_runtime::profiles
