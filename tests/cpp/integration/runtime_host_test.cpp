#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "policy_runtime/protocol/rpc/codec.hpp"
#include "policy_runtime/robot/devices/cia402/axis.hpp"
#include "policy_runtime/runtime/runtime_host.hpp"
#include "policy_runtime/runtime/runtime_host_cli.hpp"

namespace {

std::filesystem::path source_path(std::string_view relative_path) {
  return std::filesystem::path(POLICY_RUNTIME_SOURCE_DIR) / relative_path;
}

std::string read_fixture(std::string_view relative_path) {
  std::ifstream input(source_path(relative_path));
  if (!input) {
    throw std::runtime_error("unable to read fixture");
  }
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

policy_runtime::rpc::MessageEnvelope request(
    std::string type, nlohmann::json payload = nlohmann::json::object(),
    std::string session_id = "session-1", std::uint64_t step_id = 0) {
  policy_runtime::rpc::MessageEnvelope value;
  value.type = std::move(type);
  value.request_id = "request-1";
  value.session_id = std::move(session_id);
  value.step_id = step_id;
  value.timestamp_ns = 1;
  value.payload = std::move(payload);
  return value;
}

policy_runtime::RuntimeHost open_host(std::string_view profile) {
  auto host = policy_runtime::RuntimeHost::from_profiles(source_path(profile));
  if (!host.has_value()) {
    throw std::runtime_error(host.error().message);
  }
  auto opened = host.value().open();
  if (!opened.has_value()) {
    throw std::runtime_error(opened.error().message);
  }
  return std::move(host.value());
}

class FakeRobotIoChannel final : public policy_runtime::RuntimeRobotIo {
 public:
  std::uint32_t axis_count() const noexcept override { return configured_axis_count; }

  policy_runtime::Result<policy_runtime::Snapshot<policy_runtime::AxisFeedback>>
  read_feedback() override {
    if (fail_feedback) {
      return policy_runtime::Result<
          policy_runtime::Snapshot<policy_runtime::AxisFeedback>>::failure(
          {policy_runtime::ErrorCode::unavailable, "daemon feedback unavailable"});
    }
    return policy_runtime::Result<
        policy_runtime::Snapshot<policy_runtime::AxisFeedback>>::success(
        feedback);
  }

  policy_runtime::Result<void> publish_commands(
      std::span<const policy_runtime::AxisCommand> commands,
      std::uint64_t sequence, std::int64_t timestamp_ns) override {
    if (fail_publish) {
      return policy_runtime::Result<void>::failure(
          {policy_runtime::ErrorCode::io, "daemon command publish failed"});
    }
    published.assign(commands.begin(), commands.end());
    published_sequence = sequence;
    published_timestamp_ns = timestamp_ns;
    return policy_runtime::Result<void>::success();
  }

  void close() noexcept override { closed = true; }

  policy_runtime::Snapshot<policy_runtime::AxisFeedback> feedback{};
  std::uint32_t configured_axis_count{2};
  std::vector<policy_runtime::AxisCommand> published;
  std::uint64_t published_sequence{};
  std::int64_t published_timestamp_ns{};
  bool fail_feedback{};
  bool fail_publish{};
  bool closed{};
};

}  // namespace

TEST(RuntimeHostTest, MatchesDummyPolicyResponseContract) {
  auto host = policy_runtime::RuntimeHost::from_profiles(source_path(
      "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"));
  ASSERT_TRUE(host.has_value()) << host.error().message;
  ASSERT_TRUE(host.value().open().has_value());

  auto request = policy_runtime::rpc::decode_envelope(
      read_fixture("tests/golden/observation_request.json"));
  ASSERT_TRUE(request.has_value()) << request.error().message;
  auto response = host.value().handle(request.value());

  ASSERT_TRUE(response.has_value()) << response.error().message;
  EXPECT_EQ(response.value().type, "action_response");
  ASSERT_TRUE(response.value().payload.at("ok").get<bool>());
  const auto& action = response.value().payload.at("action");
  EXPECT_EQ(action.at("joint_position_delta"),
            request.value().payload.at("observation").at("joint_position"));
  EXPECT_EQ(action.at("gripper_command"),
            request.value().payload.at("observation").at("gripper_width"));
  EXPECT_EQ(action.at("action_chunk"), nlohmann::json::array());
  EXPECT_EQ(action.at("meta"), nlohmann::json::object());
}

TEST(RuntimeHostTest, MatchesHealthServerInfoResetAndRuntimeErrorEnvelopes) {
  auto host = open_host(
      "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json");

  auto health = host.handle(request("health_request", {{"verbose", true}},
                                    "health-session", 7));
  ASSERT_TRUE(health.has_value()) << health.error().message;
  EXPECT_EQ(health.value().type, "health_response");
  EXPECT_EQ(health.value().request_id, "request-1");
  EXPECT_EQ(health.value().session_id, "health-session");
  EXPECT_EQ(health.value().step_id, 7U);
  EXPECT_EQ(health.value().payload,
            (nlohmann::json{{"ok", true}, {"policy_id", "dummy-policy"}}));

  auto info = host.handle(request("server_info_request"));
  ASSERT_TRUE(info.has_value()) << info.error().message;
  EXPECT_EQ(info.value().type, "server_info_response");
  EXPECT_EQ(info.value().payload,
            (nlohmann::json{{"server_name", "policy_embodied_runtime"},
                            {"server_version", "0.1.0"},
                            {"supported_schema",
                             "embodied-policy-runtime/v1alpha1"},
                            {"transports", {"zmq+json"}}}));

  auto reset = host.handle(
      request("reset_request", {{"hard", true}}, "reset-session", 9));
  ASSERT_TRUE(reset.has_value()) << reset.error().message;
  EXPECT_EQ(reset.value().type, "reset_response");
  EXPECT_EQ(reset.value().payload,
            (nlohmann::json{{"ok", true}, {"hard", true}}));

  auto unknown = host.handle(request("future_request"));
  ASSERT_TRUE(unknown.has_value()) << unknown.error().message;
  EXPECT_EQ(unknown.value().type, "future_request_error");
  ASSERT_TRUE(unknown.value().error.has_value());
  EXPECT_EQ(unknown.value().error->code, "runtime_error");
  EXPECT_EQ(unknown.value().error->message,
            "unknown request type: future_request");
  EXPECT_EQ(unknown.value().payload, nlohmann::json::object());
}

TEST(RuntimeHostTest, MalformedRawMessageDoesNotPoisonTheNextRequest) {
  auto host = open_host(
      "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json");

  auto malformed = policy_runtime::rpc::decode_envelope(
      host.handle_text("{not-json"));
  ASSERT_TRUE(malformed.has_value()) << malformed.error().message;
  EXPECT_EQ(malformed.value().type, "protocol_error");
  EXPECT_EQ(malformed.value().request_id, "unknown");
  ASSERT_TRUE(malformed.value().error.has_value());
  EXPECT_EQ(malformed.value().error->code, "protocol_error");
  EXPECT_EQ(malformed.value().error->message, "invalid JSON message");

  auto valid = policy_runtime::rpc::decode_envelope(host.handle_text(
      policy_runtime::rpc::encode_envelope(request("health_request"))));
  ASSERT_TRUE(valid.has_value()) << valid.error().message;
  EXPECT_EQ(valid.value().type, "health_response");

  const auto encoded = nlohmann::json::parse(
      host.handle_text(policy_runtime::rpc::encode_envelope(
          request("health_request"))));
  ASSERT_TRUE(encoded.contains("error"));
  EXPECT_TRUE(encoded.at("error").is_null());
}

TEST(RuntimeHostTest, Pi0LikeSessionStepAdvancesPerSessionAndResetRestartsIt) {
  auto host = open_host(
      "policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json");
  const auto observation = nlohmann::json{
      {"observation",
       {{"joint_position",
         {{"values", {0.0, 0.1}},
          {"joint_names", {"joint_a", "joint_b"}},
          {"unit", "rad"}}},
        {"task_text", {{"text", "move"}}},
        {"image",
         {{"encoding", "uri"},
          {"data", "memory://rgb/front"},
          {"mime_type", "image/jpeg"}}}}}};

  auto first = host.handle(
      request("observation_request", observation, "session-a", 42));
  auto second = host.handle(
      request("observation_request", observation, "session-a", 42));
  ASSERT_TRUE(first.has_value()) << first.error().message;
  ASSERT_TRUE(second.has_value()) << second.error().message;
  EXPECT_EQ(first.value().payload["action"]["action_chunk"][0]["meta"]
                                ["step_id"],
            0);
  EXPECT_EQ(second.value().payload["action"]["action_chunk"][0]["meta"]
                                 ["step_id"],
            1);
  EXPECT_EQ(first.value().step_id, 42U);

  auto reset = host.handle(request("reset_request", nlohmann::json::object(),
                                   "session-a"));
  ASSERT_TRUE(reset.has_value()) << reset.error().message;
  auto after_reset = host.handle(
      request("observation_request", observation, "session-a", 42));
  ASSERT_TRUE(after_reset.has_value()) << after_reset.error().message;
  EXPECT_EQ(after_reset.value().payload["action"]["action_chunk"][0]["meta"]
                                      ["step_id"],
            0);
}

TEST(RuntimeHostTest, MissingCanonicalInputReturnsRuntimeErrorEnvelope) {
  auto host = open_host(
      "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json");
  auto response = host.handle(request(
      "observation_request",
      {{"observation", {{"joint_position", nlohmann::json::object()}}}}));

  ASSERT_TRUE(response.has_value()) << response.error().message;
  EXPECT_EQ(response.value().type, "observation_request_error");
  ASSERT_TRUE(response.value().error.has_value());
  EXPECT_EQ(response.value().error->code, "runtime_error");
  EXPECT_NE(response.value().error->message.find(
                "missing required canonical observation fields"),
            std::string::npos);
}

TEST(RuntimeHostTest, InvalidPolicyActionReturnsRuntimeErrorEnvelope) {
  auto host = open_host("tests/golden/invalid_action_chunk_policy_profile.json");
  auto response = host.handle(request(
      "observation_request",
      {{"observation",
        {{"joint_position",
          {{"values", {0.1}}, {"joint_names", {"joint_a"}}, {"unit", "rad"}}}}}}));

  ASSERT_TRUE(response.has_value()) << response.error().message;
  EXPECT_EQ(response.value().type, "observation_request_error");
  ASSERT_TRUE(response.value().error.has_value());
  EXPECT_EQ(response.value().error->code, "runtime_error");
  EXPECT_EQ(response.value().error->message, "action_chunk must be an array");
}

TEST(RuntimeHostTest, BindsDaemonFeedbackAndPublishesCanonicalAxisCommands) {
  auto channel = std::make_unique<FakeRobotIoChannel>();
  auto* channel_view = channel.get();
  channel->feedback.axis_count = 2;
  channel->feedback.sequence = 11;
  channel->feedback.timestamp_ns = 1234;
  channel->feedback.axes[0].position = 1.25;
  channel->feedback.axes[0].velocity = 8.0;
  channel->feedback.axes[1].position = 9.0;
  channel->feedback.axes[1].velocity = -0.5;

  auto host = policy_runtime::RuntimeHost::from_profiles(
      source_path("tests/golden/elmo_dummy_policy_profile.json"),
      source_path("tests/golden/elmo_robot_profile.json"), std::move(channel));
  ASSERT_TRUE(host.has_value()) << host.error().message;
  ASSERT_TRUE(host.value().open().has_value());

  auto response = host.value().handle(request(
      "observation_request",
      {{"observation", {{"shoulder_position", 99.0},
                         {"elbow_velocity", 99.0}}}},
      "robot-session", 5));

  ASSERT_TRUE(response.has_value()) << response.error().message;
  EXPECT_EQ(response.value().type, "action_response");
  EXPECT_DOUBLE_EQ(response.value().payload["action"]["shoulder_target"],
                   1.25);
  EXPECT_DOUBLE_EQ(response.value().payload["action"]["elbow_target"], -0.5);
  ASSERT_EQ(channel_view->published.size(), 2U);
  EXPECT_DOUBLE_EQ(channel_view->published[0].target, 1.25);
  EXPECT_DOUBLE_EQ(channel_view->published[1].target, -0.5);
  EXPECT_EQ(channel_view->published[0].flags,
            policy_runtime::kAxisCommandEnable);
  EXPECT_EQ(channel_view->published[1].flags,
            policy_runtime::kAxisCommandEnable);
  EXPECT_EQ(channel_view->published_sequence, 1U);
  EXPECT_GT(channel_view->published_timestamp_ns, 0);
  EXPECT_EQ(channel_view->published[0].sequence, 1U);
  EXPECT_EQ(channel_view->published[0].timestamp_ns,
            channel_view->published_timestamp_ns);

  auto second = host.value().handle(request(
      "observation_request",
      {{"observation", {{"shoulder_position", 0.0},
                         {"elbow_velocity", 0.0}}}},
      "robot-session", 6));
  ASSERT_TRUE(second.has_value()) << second.error().message;
  EXPECT_EQ(channel_view->published_sequence, 2U);

  host.value().close();
  EXPECT_TRUE(channel_view->closed);
}

TEST(RuntimeHostCliTest, PreservesCurrentFlagsDefaultsAndEndpointResolution) {
  const std::vector<std::string_view> arguments{
      "policy-runtime-host", "--policy-profile", "policy.json",
      "--robot-profile", "robot.json", "--endpoint", "rpc-test",
      "--timeout-ms", "250"};
  auto parsed = policy_runtime::parse_runtime_host_cli(arguments);

  ASSERT_TRUE(parsed.has_value()) << parsed.error().message;
  EXPECT_EQ(parsed.value().policy_profile, "policy.json");
  ASSERT_TRUE(parsed.value().robot_profile.has_value());
  EXPECT_EQ(*parsed.value().robot_profile, "robot.json");
  EXPECT_EQ(parsed.value().endpoint, "rpc-test");
  EXPECT_EQ(parsed.value().timeout_ms, 250);
  EXPECT_EQ(policy_runtime::resolve_policy_endpoint("rpc-test"),
            "ipc:///tmp/rpc-test.sock");
  EXPECT_EQ(policy_runtime::resolve_policy_endpoint("tcp://127.0.0.1:5555"),
            "tcp://127.0.0.1:5555");

  const std::vector<std::string_view> defaults{
      "policy-runtime-host", "--policy-profile", "policy.json"};
  parsed = policy_runtime::parse_runtime_host_cli(defaults);
  ASSERT_TRUE(parsed.has_value()) << parsed.error().message;
  EXPECT_EQ(parsed.value().endpoint, "embodied-policy-runtime");
  EXPECT_EQ(parsed.value().timeout_ms, 100);
  EXPECT_FALSE(parsed.value().robot_profile.has_value());
}

TEST(RuntimeHostCliTest, RejectsMissingValuesUnknownFlagsAndIncompleteRobotIo) {
  for (const auto& arguments : {
           std::vector<std::string_view>{"host"},
           {"host", "--policy-profile"},
           {"host", "--policy-profile", "p.json", "--unknown", "x"},
           {"host", "--policy-profile", "p.json", "--timeout-ms", "abc"},
           {"host", "--policy-profile", "p.json", "--robot-io-fd", "4"},
           {"host", "--policy-profile", "p.json", "--robot-io-generation",
            "2"},
       }) {
    EXPECT_FALSE(policy_runtime::parse_runtime_host_cli(arguments).has_value());
  }

  const std::vector<std::string_view> help{"host", "--help"};
  auto parsed = policy_runtime::parse_runtime_host_cli(help);
  ASSERT_TRUE(parsed.has_value()) << parsed.error().message;
  EXPECT_TRUE(parsed.value().show_help);
}

TEST(RuntimeHostTest, RejectsMissingOrMismatchedDaemonAndClosedLifecycleUse) {
  auto missing = policy_runtime::RuntimeHost::from_profiles(
      source_path("tests/golden/elmo_dummy_policy_profile.json"),
      source_path("tests/golden/elmo_robot_profile.json"));
  ASSERT_TRUE(missing.has_value()) << missing.error().message;
  auto opened = missing.value().open();
  ASSERT_FALSE(opened.has_value());
  EXPECT_EQ(opened.error().code, policy_runtime::ErrorCode::unavailable);

  auto channel = std::make_unique<FakeRobotIoChannel>();
  channel->configured_axis_count = 1;
  auto mismatched = policy_runtime::RuntimeHost::from_profiles(
      source_path("tests/golden/elmo_dummy_policy_profile.json"),
      source_path("tests/golden/elmo_robot_profile.json"), std::move(channel));
  ASSERT_TRUE(mismatched.has_value()) << mismatched.error().message;
  opened = mismatched.value().open();
  ASSERT_FALSE(opened.has_value());
  EXPECT_EQ(opened.error().code, policy_runtime::ErrorCode::invalid_argument);

  auto host = open_host(
      "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json");
  host.close();
  auto response = host.handle(request("health_request"));
  ASSERT_FALSE(response.has_value());
  EXPECT_EQ(response.error().code, policy_runtime::ErrorCode::unavailable);
}

TEST(RuntimeHostTest, DaemonFailuresReturnErrorsWithoutAdvancingPublication) {
  auto channel = std::make_unique<FakeRobotIoChannel>();
  auto* channel_view = channel.get();
  channel->feedback.axis_count = 2;
  channel->feedback.axes[0].position = 0.25;
  channel->feedback.axes[1].velocity = -0.25;
  auto host = policy_runtime::RuntimeHost::from_profiles(
      source_path("tests/golden/elmo_dummy_policy_profile.json"),
      source_path("tests/golden/elmo_robot_profile.json"), std::move(channel));
  ASSERT_TRUE(host.has_value()) << host.error().message;
  ASSERT_TRUE(host.value().open().has_value());
  const auto observation = nlohmann::json{
      {"observation", {{"shoulder_position", 0.0},
                       {"elbow_velocity", 0.0}}}};

  channel_view->fail_feedback = true;
  auto feedback_error = host.value().handle(
      request("observation_request", observation));
  ASSERT_TRUE(feedback_error.has_value()) << feedback_error.error().message;
  EXPECT_EQ(feedback_error.value().type, "observation_request_error");
  ASSERT_TRUE(feedback_error.value().error.has_value());
  EXPECT_EQ(feedback_error.value().error->message,
            "daemon feedback unavailable");
  EXPECT_TRUE(channel_view->published.empty());

  channel_view->fail_feedback = false;
  channel_view->fail_publish = true;
  auto publish_error = host.value().handle(
      request("observation_request", observation));
  ASSERT_TRUE(publish_error.has_value()) << publish_error.error().message;
  EXPECT_EQ(publish_error.value().type, "observation_request_error");
  ASSERT_TRUE(publish_error.value().error.has_value());
  EXPECT_EQ(publish_error.value().error->message,
            "daemon command publish failed");
  EXPECT_EQ(channel_view->published_sequence, 0U);

  channel_view->fail_publish = false;
  auto recovered = host.value().handle(
      request("observation_request", observation));
  ASSERT_TRUE(recovered.has_value()) << recovered.error().message;
  EXPECT_EQ(recovered.value().type, "action_response");
  EXPECT_EQ(channel_view->published_sequence, 1U);
}
