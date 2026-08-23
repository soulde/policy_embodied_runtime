#include "policy_runtime/protocol/rpc/codec.hpp"

#include <cctype>
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

bool validate_joint_state(const Json& value, std::string& error) {
  if (!value.is_object()) {
    error = "joint state must be an object";
    return false;
  }
  const auto* values = member(value, "values");
  const auto* joint_names = member(value, "joint_names");
  const auto* unit = member(value, "unit");
  if (values == nullptr || !values->is_array() || joint_names == nullptr ||
      !joint_names->is_array() || unit == nullptr || !unit->is_string()) {
    error = "joint state requires values, joint_names, and unit";
    return false;
  }
  if (values->size() != joint_names->size()) {
    error = "values and joint_names must have the same length";
    return false;
  }
  return true;
}

bool validate_observation(const Json& payload, std::string& error) {
  const auto* observation = member(payload, "observation");
  if (observation == nullptr) {
    return true;
  }
  if (!observation->is_object()) {
    error = "observation must be an object";
    return false;
  }
  const auto* joint_position = member(*observation, "joint_position");
  return joint_position == nullptr || joint_position->is_null() ||
         validate_joint_state(*joint_position, error);
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

  const auto* action_chunk = member(action, "action_chunk");
  if (action_chunk != nullptr && !action_chunk->is_array()) {
    error = "action_chunk must be an array";
    return false;
  }

  const auto* gripper_command = member(action, "gripper_command");
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

bool validate_payload(const Json& payload, std::string& error) {
  if (!payload.is_object()) {
    error = "payload must be an object";
    return false;
  }
  if (!validate_observation(payload, error)) {
    return false;
  }
  const auto* action = member(payload, "action");
  return action == nullptr || action->is_null() || validate_action(*action, error);
}

bool validate_error_payload(const Json& value, ErrorPayload& error_payload,
                            std::string& error) {
  if (!value.is_object()) {
    error = "error must be an object";
    return false;
  }
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (it.key() != "code" && it.key() != "message" && it.key() != "details") {
      error = "unknown error key: " + it.key();
      return false;
    }
  }
  const auto* code = member(value, "code");
  const auto* message = member(value, "message");
  if (code == nullptr || !code->is_string() || message == nullptr || !message->is_string()) {
    error = "error requires string code and message";
    return false;
  }
  const auto* details = member(value, "details");
  if (details != nullptr && !details->is_object()) {
    error = "error details must be an object";
    return false;
  }
  error_payload = {code->get<std::string>(), message->get<std::string>(),
                   details == nullptr ? Json::object() : *details};
  return true;
}

}  // namespace

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
  if (timestamp_ns == nullptr ||
      (!timestamp_ns->is_number_integer() && !timestamp_ns->is_number_unsigned())) {
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
  if (timestamp_ns->is_number_integer() && timestamp_ns->get<std::int64_t>() < 0) {
    return failure("timestamp_ns must be >= 0");
  }
  if (timestamp_ns->is_number_unsigned() &&
      timestamp_ns->get<std::uint64_t>() >
          static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    return failure("timestamp_ns is out of range");
  }
  envelope.timestamp_ns = timestamp_ns->get<std::int64_t>();

  const auto* step_id = member(message, "step_id");
  if (step_id != nullptr) {
    if (!step_id->is_number_unsigned() && !step_id->is_number_integer()) {
      return failure("step_id must be a non-negative integer");
    }
    if (step_id->is_number_integer() && step_id->get<std::int64_t>() < 0) {
      return failure("step_id must be a non-negative integer");
    }
    envelope.step_id = step_id->get<std::uint64_t>();
  }

  const auto* payload = member(message, "payload");
  if (payload != nullptr) {
    if (!validate_payload(*payload, validation_error)) {
      return failure(validation_error);
    }
    envelope.payload = *payload;
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
  }
  return message.dump();
}

}  // namespace policy_runtime::rpc
