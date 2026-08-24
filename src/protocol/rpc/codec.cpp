#include "policy_runtime/protocol/rpc/codec.hpp"

#include <cctype>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>

namespace policy_runtime::rpc {
namespace {

using Json = nlohmann::json;

Result<MessageEnvelope> failure(std::string message) {
  return Result<MessageEnvelope>::failure({ErrorCode::protocol, std::move(message)});
}

bool is_blank(std::string_view value) {
  for (const auto character : value) {
    if (!std::isspace(static_cast<unsigned char>(character))) {
      return false;
    }
  }
  return true;
}

const Json* member(const Json& object, std::string_view name) {
  const auto found = object.find(std::string(name));
  return found == object.end() ? nullptr : &*found;
}

bool is_strict_integer(const Json& value) {
  return value.type() == Json::value_t::number_integer ||
         value.type() == Json::value_t::number_unsigned;
}

bool is_strict_number(const Json& value) {
  return is_strict_integer(value) || value.type() == Json::value_t::number_float;
}

bool only_keys(const Json& object, std::initializer_list<std::string_view> allowed,
               std::string_view name, std::string& error) {
  for (auto it = object.begin(); it != object.end(); ++it) {
    bool is_allowed = false;
    for (const auto key : allowed) {
      if (it.key() == key) {
        is_allowed = true;
        break;
      }
    }
    if (!is_allowed) {
      error = std::string(name) + " contains unknown key: " + it.key();
      return false;
    }
  }
  return true;
}

bool require_string(const Json& object, std::string_view key, std::string_view name,
                    std::string& error) {
  const auto* value = member(object, key);
  if (value == nullptr || !value->is_string()) {
    error = std::string(name) + " requires string " + std::string(key);
    return false;
  }
  return true;
}

bool validate_joint_state(const Json& value, std::string& error) {
  if (!value.is_object()) {
    error = "joint state must be an object";
    return false;
  }
  if (!only_keys(value, {"values", "joint_names", "unit"}, "joint state", error)) {
    return false;
  }
  const auto* values = member(value, "values");
  const auto* joint_names = member(value, "joint_names");
  if (values == nullptr || !values->is_array() || joint_names == nullptr ||
      !joint_names->is_array() || !require_string(value, "unit", "joint state", error)) {
    if (error.empty()) {
      error = "joint state requires values, joint_names, and unit";
    }
    return false;
  }
  for (const auto& joint_value : *values) {
    if (!is_strict_number(joint_value)) {
      error = "joint state values must be numbers";
      return false;
    }
  }
  for (const auto& joint_name : *joint_names) {
    if (!joint_name.is_string()) {
      error = "joint state joint_names must contain strings";
      return false;
    }
  }
  if (values->size() != joint_names->size()) {
    error = "values and joint_names must have the same length";
    return false;
  }
  return true;
}

bool validate_gripper(const Json& value, std::string& error) {
  if (!value.is_object()) {
    error = "gripper value must be an object";
    return false;
  }
  if (!only_keys(value, {"value", "unit"}, "gripper value", error)) {
    return false;
  }
  const auto* scalar = member(value, "value");
  if (scalar == nullptr || !is_strict_number(*scalar) ||
      !require_string(value, "unit", "gripper value", error)) {
    if (error.empty()) {
      error = "gripper value requires numeric value and string unit";
    }
    return false;
  }
  return true;
}

bool validate_task_text(const Json& value, std::string& error) {
  if (!value.is_object() || !only_keys(value, {"text"}, "task text", error) ||
      !require_string(value, "text", "task text", error)) {
    if (error.empty()) {
      error = "task text requires string text";
    }
    return false;
  }
  return true;
}

bool validate_image(const Json& value, std::string& error) {
  if (!value.is_object() ||
      !only_keys(value, {"encoding", "data", "mime_type"}, "image", error) ||
      !require_string(value, "data", "image", error)) {
    if (error.empty()) {
      error = "image requires string data";
    }
    return false;
  }
  const auto* encoding = member(value, "encoding");
  if (encoding != nullptr &&
      (!encoding->is_string() || (encoding->get<std::string>() != "base64" &&
                                  encoding->get<std::string>() != "uri" &&
                                  encoding->get<std::string>() != "inline_json"))) {
    error = "image encoding must be base64, uri, or inline_json";
    return false;
  }
  const auto* mime_type = member(value, "mime_type");
  if (mime_type != nullptr && !mime_type->is_string()) {
    error = "image mime_type must be a string";
    return false;
  }
  return true;
}

bool validate_observation(const Json& value, std::string& error) {
  if (!value.is_object()) {
    error = "observation must be an object";
    return false;
  }
  const auto* joint_position = member(value, "joint_position");
  if (joint_position != nullptr && !joint_position->is_null() &&
      !validate_joint_state(*joint_position, error)) {
    return false;
  }
  const auto* gripper_width = member(value, "gripper_width");
  if (gripper_width != nullptr && !gripper_width->is_null() &&
      !validate_gripper(*gripper_width, error)) {
    return false;
  }
  const auto* task_text = member(value, "task_text");
  if (task_text != nullptr && !task_text->is_null() && !validate_task_text(*task_text, error)) {
    return false;
  }
  const auto* image = member(value, "image");
  if (image != nullptr && !image->is_null() && !validate_image(*image, error)) {
    return false;
  }
  const auto* meta = member(value, "meta");
  if (meta != nullptr && !meta->is_object()) {
    error = "observation meta must be an object";
    return false;
  }
  return true;
}

bool validate_action(const Json& action, std::string& error) {
  if (!action.is_object()) {
    error = "action must be an object";
    return false;
  }
  const auto* joint_delta = member(action, "joint_position_delta");
  if (joint_delta != nullptr && !joint_delta->is_null() &&
      !validate_joint_state(*joint_delta, error)) {
    return false;
  }
  const auto* gripper_command = member(action, "gripper_command");
  if (gripper_command != nullptr && !gripper_command->is_null() &&
      !validate_gripper(*gripper_command, error)) {
    return false;
  }
  const auto* action_chunk = member(action, "action_chunk");
  if (action_chunk != nullptr) {
    if (!action_chunk->is_array()) {
      error = "action_chunk must be an array";
      return false;
    }
    for (const auto& item : *action_chunk) {
      if (!item.is_object()) {
        error = "action_chunk must contain objects";
        return false;
      }
    }
  }
  const auto* meta = member(action, "meta");
  if (meta != nullptr && !meta->is_object()) {
    error = "action meta must be an object";
    return false;
  }
  const bool has_single_step =
      (joint_delta != nullptr && !joint_delta->is_null()) ||
      (gripper_command != nullptr && !gripper_command->is_null());
  const bool has_chunk = action_chunk != nullptr && !action_chunk->empty();
  bool has_extra = false;
  for (auto it = action.begin(); it != action.end(); ++it) {
    if (it.key() != "joint_position_delta" && it.key() != "gripper_command" &&
        it.key() != "action_chunk" && it.key() != "meta") {
      has_extra = true;
      break;
    }
  }
  if (!has_single_step && !has_chunk && !has_extra) {
    error = "action payload must contain a single-step action or an action_chunk";
    return false;
  }
  return true;
}

bool validate_error_payload(const Json& value, ErrorPayload& error_payload,
                            std::string& error) {
  if (!value.is_object() || !only_keys(value, {"code", "message", "details"}, "error", error) ||
      !require_string(value, "code", "error", error) ||
      !require_string(value, "message", "error", error)) {
    if (error.empty()) {
      error = "error requires string code and message";
    }
    return false;
  }
  const auto* details = member(value, "details");
  if (details != nullptr && !details->is_object()) {
    error = "error details must be an object";
    return false;
  }
  error_payload = {member(value, "code")->get<std::string>(),
                   member(value, "message")->get<std::string>(),
                   details == nullptr ? Json::object() : *details};
  return true;
}

bool validate_observation_request(const Json& payload, std::string& error) {
  if (!only_keys(payload, {"observation"}, "observation request", error)) {
    return false;
  }
  const auto* observation = member(payload, "observation");
  if (observation == nullptr || observation->is_null()) {
    error = "observation request requires observation";
    return false;
  }
  return validate_observation(*observation, error);
}

bool validate_action_response(const Json& payload, std::string& error) {
  if (!only_keys(payload, {"ok", "action"}, "action response", error)) {
    return false;
  }
  const auto* ok = member(payload, "ok");
  const auto* action = member(payload, "action");
  if (ok == nullptr || !ok->is_boolean() || action == nullptr || action->is_null()) {
    error = "action response requires boolean ok and action";
    return false;
  }
  return validate_action(*action, error);
}

bool validate_health_request(const Json& payload, std::string& error) {
  if (!only_keys(payload, {"verbose"}, "health request", error)) {
    return false;
  }
  const auto* verbose = member(payload, "verbose");
  if (verbose != nullptr && !verbose->is_boolean()) {
    error = "health request verbose must be a boolean";
    return false;
  }
  return true;
}

bool validate_health_response(const Json& payload, std::string& error) {
  if (!only_keys(payload, {"ok", "policy_id"}, "health response", error)) {
    return false;
  }
  const auto* ok = member(payload, "ok");
  if (ok == nullptr || !ok->is_boolean() ||
      !require_string(payload, "policy_id", "health response", error)) {
    if (error.empty()) {
      error = "health response requires boolean ok and string policy_id";
    }
    return false;
  }
  return true;
}

bool validate_reset_request(const Json& payload, std::string& error) {
  if (!only_keys(payload, {"hard"}, "reset request", error)) {
    return false;
  }
  const auto* hard = member(payload, "hard");
  if (hard != nullptr && !hard->is_boolean()) {
    error = "reset request hard must be a boolean";
    return false;
  }
  return true;
}

bool validate_server_info_response(const Json& payload, std::string& error) {
  if (!only_keys(payload, {"server_name", "server_version", "supported_schema", "transports"},
                 "server info response", error) ||
      !require_string(payload, "server_name", "server info response", error) ||
      !require_string(payload, "server_version", "server info response", error)) {
    if (error.empty()) {
      error = "server info response requires server_name and server_version";
    }
    return false;
  }
  const auto* supported_schema = member(payload, "supported_schema");
  if (supported_schema != nullptr && !supported_schema->is_string()) {
    error = "server info response supported_schema must be a string";
    return false;
  }
  const auto* transports = member(payload, "transports");
  if (transports != nullptr) {
    if (!transports->is_array()) {
      error = "server info response transports must be an array";
      return false;
    }
    for (const auto& transport : *transports) {
      if (!transport.is_string()) {
        error = "server info response transports must contain strings";
        return false;
      }
    }
  }
  return true;
}

bool validate_payload_for_type(std::string_view type, const Json& payload, std::string& error) {
  if (!payload.is_object()) {
    error = "payload must be an object";
    return false;
  }
  if (type == "observation_request") {
    return validate_observation_request(payload, error);
  }
  if (type == "action_response") {
    return validate_action_response(payload, error);
  }
  if (type == "health_request") {
    return validate_health_request(payload, error);
  }
  if (type == "health_response") {
    return validate_health_response(payload, error);
  }
  if (type == "reset_request") {
    return validate_reset_request(payload, error);
  }
  if (type == "server_info_response") {
    return validate_server_info_response(payload, error);
  }
  return true;
}

}  // namespace

Result<void> validate_action_payload(const nlohmann::json& action) {
  std::string error;
  if (!validate_action(action, error)) {
    return Result<void>::failure({ErrorCode::protocol, std::move(error)});
  }
  return Result<void>::success();
}

Result<MessageEnvelope> decode_envelope(std::string_view text) {
  Json message = Json::parse(text.begin(), text.end(), nullptr, false);
  if (message.is_discarded()) {
    return failure("invalid JSON message");
  }
  if (!message.is_object()) {
    return failure("message envelope must be an object");
  }
  for (auto it = message.begin(); it != message.end(); ++it) {
    if (it.key() != "schema" && it.key() != "type" && it.key() != "request_id" &&
        it.key() != "session_id" && it.key() != "step_id" &&
        it.key() != "timestamp_ns" && it.key() != "payload" && it.key() != "error") {
      return failure("unknown top-level key: " + it.key());
    }
  }

  MessageEnvelope envelope;
  std::string validation_error;
  const auto* schema = member(message, "schema");
  if (schema != nullptr) {
    if (!schema->is_string()) {
      return failure("schema must be a string");
    }
    envelope.schema = schema->get<std::string>();
  }
  const auto* type = member(message, "type");
  const auto* request_id = member(message, "request_id");
  const auto* session_id = member(message, "session_id");
  const auto* timestamp_ns = member(message, "timestamp_ns");
  if (type == nullptr || !type->is_string()) {
    return failure("type must be a string");
  }
  if (request_id == nullptr || !request_id->is_string()) {
    return failure("request_id must be a string");
  }
  if (session_id == nullptr || !session_id->is_string()) {
    return failure("session_id must be a string");
  }
  if (timestamp_ns == nullptr || !is_strict_integer(*timestamp_ns)) {
    return failure("timestamp_ns must be an integer");
  }

  envelope.type = type->get<std::string>();
  envelope.request_id = request_id->get<std::string>();
  envelope.session_id = session_id->get<std::string>();
  if (is_blank(envelope.type)) {
    return failure("type must be non-empty");
  }
  if (is_blank(envelope.request_id)) {
    return failure("request_id must be non-empty");
  }
  if (is_blank(envelope.session_id)) {
    return failure("session_id must be non-empty");
  }
  if (timestamp_ns->type() == Json::value_t::number_integer) {
    const auto timestamp = timestamp_ns->get<std::int64_t>();
    if (timestamp < 0) {
      return failure("timestamp_ns must be >= 0");
    }
    envelope.timestamp_ns = timestamp;
  } else {
    const auto timestamp = timestamp_ns->get<std::uint64_t>();
    if (timestamp > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      return failure("timestamp_ns is out of range");
    }
    envelope.timestamp_ns = static_cast<std::int64_t>(timestamp);
  }

  const auto* step_id = member(message, "step_id");
  if (step_id != nullptr) {
    if (!is_strict_integer(*step_id)) {
      return failure("step_id must be a non-negative integer");
    }
    if (step_id->type() == Json::value_t::number_integer) {
      const auto step = step_id->get<std::int64_t>();
      if (step < 0) {
        return failure("step_id must be a non-negative integer");
      }
      envelope.step_id = static_cast<std::uint64_t>(step);
    } else {
      envelope.step_id = step_id->get<std::uint64_t>();
    }
  }

  const auto* payload = member(message, "payload");
  if (payload != nullptr) {
    envelope.payload = *payload;
  }
  if (!validate_payload_for_type(envelope.type, envelope.payload, validation_error)) {
    return failure(validation_error);
  }
  const auto* error = member(message, "error");
  if (error != nullptr && !error->is_null()) {
    ErrorPayload error_payload;
    if (!validate_error_payload(*error, error_payload, validation_error)) {
      return failure(validation_error);
    }
    envelope.error = std::move(error_payload);
  }
  return Result<MessageEnvelope>::success(std::move(envelope));
}

std::string encode_envelope(const MessageEnvelope& envelope) {
  Json message = {
      {"schema", envelope.schema},
      {"type", envelope.type},
      {"request_id", envelope.request_id},
      {"session_id", envelope.session_id},
      {"step_id", envelope.step_id},
      {"timestamp_ns", envelope.timestamp_ns},
      {"payload", envelope.payload},
  };
  if (envelope.error.has_value()) {
    message["error"] = {
        {"code", envelope.error->code},
        {"message", envelope.error->message},
        {"details", envelope.error->details},
    };
  } else {
    // Python's current Pydantic encoder includes the nullable error field on
    // every successful request and response. Keep that exact wire shape.
    message["error"] = nullptr;
  }
  return message.dump();
}

}  // namespace policy_runtime::rpc
