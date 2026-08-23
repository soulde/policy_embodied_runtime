#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace policy_runtime::rpc {

inline constexpr char kSchemaVersion[] = "embodied-policy-runtime/v1alpha1";

struct ErrorPayload {
  std::string code;
  std::string message;
  nlohmann::json details = nlohmann::json::object();
};

struct JointStateValue {
  std::vector<double> values;
  std::vector<std::string> joint_names;
  std::string unit;
};

struct GripperValue {
  double value{};
  std::string unit;
};

struct TaskTextValue {
  std::string text;
};

struct ImageValue {
  std::string encoding = "base64";
  std::string data;
  std::string mime_type = "image/jpeg";
};

struct ObservationPayload {
  std::optional<JointStateValue> joint_position;
  std::optional<GripperValue> gripper_width;
  std::optional<TaskTextValue> task_text;
  std::optional<ImageValue> image;
  nlohmann::json meta = nlohmann::json::object();
  nlohmann::json extra = nlohmann::json::object();
};

struct ActionPayload {
  std::optional<JointStateValue> joint_position_delta;
  std::optional<GripperValue> gripper_command;
  nlohmann::json action_chunk = nlohmann::json::array();
  nlohmann::json meta = nlohmann::json::object();
  nlohmann::json extra = nlohmann::json::object();
};

struct MessageEnvelope {
  std::string schema = kSchemaVersion;
  std::string type;
  std::string request_id;
  std::string session_id;
  std::uint64_t step_id{};
  std::int64_t timestamp_ns{};
  nlohmann::json payload = nlohmann::json::object();
  std::optional<ErrorPayload> error;
};

}  // namespace policy_runtime::rpc
