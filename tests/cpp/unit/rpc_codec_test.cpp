#include <fstream>
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
