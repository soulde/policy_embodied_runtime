#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "policy_runtime/protocol/rpc/codec.hpp"

namespace {

std::string read_fixture(const std::string& relative_path) {
  std::ifstream input(std::string(POLICY_RUNTIME_SOURCE_DIR) + "/" + relative_path);
  if (!input) {
    throw std::runtime_error("unable to read fixture: " + relative_path);
  }
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

}  // namespace

void expect_protocol_error(const nlohmann::json& message, std::string_view expected) {
  const auto decoded = policy_runtime::rpc::decode_envelope(message.dump());
  ASSERT_FALSE(decoded.has_value());
  EXPECT_EQ(decoded.error().code, policy_runtime::ErrorCode::protocol);
  EXPECT_EQ(decoded.error().message, expected);
}

void expect_rejected(const nlohmann::json& message) {
  const auto decoded = policy_runtime::rpc::decode_envelope(message.dump());
  EXPECT_FALSE(decoded.has_value());
}

void expect_accepted(const nlohmann::json& message) {
  auto decoded = policy_runtime::rpc::decode_envelope(message.dump());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(nlohmann::json::parse(policy_runtime::rpc::encode_envelope(decoded.value())),
            message);
}

nlohmann::json observation_request() {
  return nlohmann::json::parse(read_fixture("tests/golden/observation_request.json"));
}

TEST(RpcCodecTest, PreservesV1Alpha1Envelope) {
  const auto text = read_fixture("tests/golden/observation_request.json");
  auto decoded = policy_runtime::rpc::decode_envelope(text);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded.value().schema, "embodied-policy-runtime/v1alpha1");
  EXPECT_EQ(decoded.value().type, "observation_request");
  EXPECT_EQ(nlohmann::json::parse(policy_runtime::rpc::encode_envelope(decoded.value())),
            nlohmann::json::parse(text));
}

TEST(RpcCodecTest, RejectsUnknownTopLevelKey) {
  auto message = nlohmann::json::parse(read_fixture("tests/golden/observation_request.json"));
  message["unexpected"] = true;
  expect_protocol_error(message, "unknown top-level key: unexpected");
}

TEST(RpcCodecTest, RejectsEmptyRequestId) {
  auto message = nlohmann::json::parse(read_fixture("tests/golden/observation_request.json"));
  message["request_id"] = "  ";
  expect_protocol_error(message, "request_id must be non-empty");
}

TEST(RpcCodecTest, RejectsEmptyType) {
  auto message = nlohmann::json::parse(read_fixture("tests/golden/observation_request.json"));
  message["type"] = "\t";
  expect_protocol_error(message, "type must be non-empty");
}

TEST(RpcCodecTest, RejectsEmptySessionId) {
  auto message = nlohmann::json::parse(read_fixture("tests/golden/observation_request.json"));
  message["session_id"] = "";
  expect_protocol_error(message, "session_id must be non-empty");
}

TEST(RpcCodecTest, RejectsNegativeTimestamp) {
  auto message = nlohmann::json::parse(read_fixture("tests/golden/observation_request.json"));
  message["timestamp_ns"] = -1;
  expect_protocol_error(message, "timestamp_ns must be >= 0");
}

TEST(RpcCodecTest, RejectsMismatchedJointState) {
  auto message = nlohmann::json::parse(read_fixture("tests/golden/observation_request.json"));
  message["payload"]["observation"]["joint_position"]["joint_names"] =
      nlohmann::json::array({"joint_1"});
  expect_protocol_error(message, "values and joint_names must have the same length");
}

TEST(RpcCodecTest, RejectsEmptyActionPayload) {
  auto message = nlohmann::json::parse(read_fixture("tests/golden/observation_request.json"));
  message["type"] = "action_response";
  message["payload"] = {{"ok", true}, {"action", nlohmann::json::object()}};
  expect_protocol_error(message,
                        "action payload must contain a single-step action or an action_chunk");
}

TEST(RpcCodecTest, RejectsInvalidJsonAndNonObjectRoots) {
  const auto invalid_json = policy_runtime::rpc::decode_envelope("{");
  ASSERT_FALSE(invalid_json.has_value());
  EXPECT_EQ(invalid_json.error().message, "invalid JSON message");

  const auto array_root = policy_runtime::rpc::decode_envelope("[]");
  ASSERT_FALSE(array_root.has_value());
  EXPECT_EQ(array_root.error().message, "message envelope must be an object");
}

TEST(RpcCodecTest, RejectsMissingAndMistypedEnvelopeFields) {
  auto missing_timestamp = observation_request();
  missing_timestamp.erase("timestamp_ns");
  expect_rejected(missing_timestamp);

  auto mistyped_step = observation_request();
  mistyped_step["step_id"] = "0";
  expect_rejected(mistyped_step);

  auto non_object_payload = observation_request();
  non_object_payload["payload"] = false;
  expect_rejected(non_object_payload);
}

TEST(RpcCodecTest, RequiresObservationRequestAndForbidsWrapperExtras) {
  auto missing_observation = observation_request();
  missing_observation["payload"] = nlohmann::json::object();
  expect_rejected(missing_observation);

  auto unexpected_wrapper_key = observation_request();
  unexpected_wrapper_key["payload"]["unexpected"] = true;
  expect_rejected(unexpected_wrapper_key);
}

TEST(RpcCodecTest, DispatchDecodePreservesEnvelopeBeforePayloadValidation) {
  auto invalid_health = observation_request();
  invalid_health["type"] = "health_request";
  invalid_health["request_id"] = "correlated-request";
  invalid_health["session_id"] = "correlated-session";
  invalid_health["step_id"] = 17;
  invalid_health["payload"] = {{"unexpected", true}};

  EXPECT_FALSE(
      policy_runtime::rpc::decode_envelope(invalid_health.dump()).has_value());
  auto dispatch = policy_runtime::rpc::decode_envelope_for_dispatch(
      invalid_health.dump());
  ASSERT_TRUE(dispatch.has_value()) << dispatch.error().message;
  EXPECT_EQ(dispatch.value().type, "health_request");
  EXPECT_EQ(dispatch.value().request_id, "correlated-request");
  EXPECT_EQ(dispatch.value().session_id, "correlated-session");
  EXPECT_EQ(dispatch.value().step_id, 17U);
  EXPECT_EQ(dispatch.value().payload, (nlohmann::json{{"unexpected", true}}));
}

TEST(RpcCodecTest, AllowsObservationExtensionsButForbidsNestedValueExtras) {
  auto extension = observation_request();
  extension["payload"]["observation"]["policy_camera"] =
      {{"uri", "memory://front"}, {"vendor", "custom"}};
  expect_accepted(extension);

  auto invalid_joint = observation_request();
  invalid_joint["payload"]["observation"]["joint_position"]["unexpected"] = true;
  expect_rejected(invalid_joint);
}

TEST(RpcCodecTest, ValidatesJointGripperTaskAndImageStructures) {
  auto string_joint_value = observation_request();
  string_joint_value["payload"]["observation"]["joint_position"]["values"][0] = "0";
  expect_rejected(string_joint_value);

  auto missing_gripper_unit = observation_request();
  missing_gripper_unit["payload"]["observation"]["gripper_width"] = {{"value", 0.04}};
  expect_rejected(missing_gripper_unit);

  auto non_string_task = observation_request();
  non_string_task["payload"]["observation"]["task_text"] = {{"text", 1}};
  expect_rejected(non_string_task);

  auto invalid_encoding = observation_request();
  invalid_encoding["payload"]["observation"]["image"] =
      {{"encoding", "binary"}, {"data", "payload"}};
  expect_rejected(invalid_encoding);
}

TEST(RpcCodecTest, RequiresActionResponseFieldsAndObjectChunkElements) {
  auto response = observation_request();
  response["type"] = "action_response";
  response["payload"] = {
      {"action", {{"action_chunk", nlohmann::json::array({{{"delta", 0.1}}})}}},
  };
  expect_rejected(response);

  response["payload"]["ok"] = true;
  response["payload"]["action"]["action_chunk"] = nlohmann::json::array({1});
  expect_rejected(response);
}

TEST(RpcCodecTest, PreservesActionChunkAndAllowsActionExtensions) {
  auto response = observation_request();
  response["type"] = "action_response";
  response["payload"] = {
      {"ok", true},
      {"action", {{"action_chunk", nlohmann::json::array({{{"delta", 0.1}}})},
                  {"policy_extension", {{"confidence", 0.9}}}}},
  };
  expect_accepted(response);
}

TEST(RpcCodecTest, ValidatesKnownHealthPayloadAndResponses) {
  auto health_request = observation_request();
  health_request["type"] = "health_request";
  health_request["payload"] = {{"verbose", "true"}};
  expect_rejected(health_request);

  health_request["payload"] = {{"verbose", true}, {"unexpected", false}};
  expect_rejected(health_request);

  auto health_response = observation_request();
  health_response["type"] = "health_response";
  health_response["payload"] = {{"ok", true}, {"policy_id", "dummy-policy"}};
  expect_accepted(health_response);
}

TEST(RpcCodecTest, ValidatesKnownResetAndServerInfoPayloads) {
  auto reset_request = observation_request();
  reset_request["type"] = "reset_request";
  reset_request["payload"] = {{"hard", "true"}};
  expect_rejected(reset_request);

  reset_request["payload"] = {{"hard", true}};
  expect_accepted(reset_request);

  auto server_info = observation_request();
  server_info["type"] = "server_info_response";
  server_info["payload"] = {{"server_name", "host"}, {"transports", {"zmq+json"}}};
  expect_rejected(server_info);

  server_info["payload"] = {{"server_name", "host"},
                              {"server_version", "0.1.0"},
                              {"supported_schema", "embodied-policy-runtime/v1alpha1"},
                              {"transports", {"zmq+json"}}};
  expect_accepted(server_info);
}

TEST(RpcCodecTest, ValidatesErrorPayloadStructure) {
  auto message = observation_request();
  message["error"] = {{"code", "runtime_error"}, {"message", "bad"}, {"details", true}};
  expect_rejected(message);

  message["error"] = {{"code", "runtime_error"}, {"message", "bad"}, {"extra", 1}};
  expect_rejected(message);

  message["error"] = {{"code", "runtime_error"}, {"message", "bad"},
                      {"details", {{"field", "payload"}}}};
  expect_accepted(message);
}

TEST(RpcCodecTest, KeepsUnknownRequestPayloadOpaqueForRuntimeDispatch) {
  auto message = observation_request();
  message["type"] = "future_request";
  message["payload"] = {{"action", nlohmann::json::object()}, {"custom", {{"v", 1}}}};
  expect_accepted(message);
}

TEST(RpcCodecTest, EnforcesCxxIntegerBoundaries) {
  auto largest_supported = observation_request();
  largest_supported["step_id"] = std::numeric_limits<std::uint64_t>::max();
  largest_supported["timestamp_ns"] = std::numeric_limits<std::int64_t>::max();
  expect_accepted(largest_supported);

  auto floating_step = observation_request();
  floating_step["step_id"] = 1.0;
  expect_rejected(floating_step);

  auto boolean_timestamp = observation_request();
  boolean_timestamp["timestamp_ns"] = true;
  expect_rejected(boolean_timestamp);

  auto out_of_range_timestamp = observation_request();
  out_of_range_timestamp["timestamp_ns"] =
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) + 1U;
  expect_rejected(out_of_range_timestamp);
}
