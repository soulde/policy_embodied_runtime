#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include "policy_runtime/profiles/loader.hpp"

namespace {

using Json = nlohmann::json;
using policy_runtime::ErrorCode;
using policy_runtime::profiles::Cia402Mode;
using policy_runtime::profiles::load_policy_profile;
using policy_runtime::profiles::load_robot_profile;

std::filesystem::path source_path(std::string_view relative_path) {
  return std::filesystem::path(POLICY_RUNTIME_SOURCE_DIR) / relative_path;
}

Json read_json(std::string_view relative_path) {
  std::ifstream input(source_path(relative_path));
  if (!input) {
    throw std::runtime_error("unable to read fixture");
  }
  return Json::parse(input);
}

class TemporaryJsonFile {
 public:
  explicit TemporaryJsonFile(const Json& value) {
    static std::atomic<unsigned long> sequence{};
    path_ = std::filesystem::temp_directory_path() /
            ("policy-runtime-profile-test-" + std::to_string(::getpid()) + "-" +
             std::to_string(sequence++) + ".json");
    std::ofstream output(path_);
    output << value;
    if (!output) {
      throw std::runtime_error("unable to write temporary fixture");
    }
  }

  ~TemporaryJsonFile() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

void expect_robot_rejected(const Json& value) {
  const TemporaryJsonFile file(value);
  const auto profile = load_robot_profile(file.path());
  EXPECT_FALSE(profile.has_value());
  if (!profile.has_value()) {
    EXPECT_EQ(profile.error().code, ErrorCode::invalid_argument);
  }
}

void expect_policy_rejected(const Json& value) {
  const TemporaryJsonFile file(value);
  const auto profile = load_policy_profile(file.path());
  EXPECT_FALSE(profile.has_value());
  if (!profile.has_value()) {
    EXPECT_EQ(profile.error().code, ErrorCode::invalid_argument);
  }
}

Json elmo_profile() { return read_json("tests/golden/elmo_robot_profile.json"); }

Json dummy_policy() {
  return read_json(
      "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json");
}

}  // namespace

TEST(ProfileLoaderTest, LoadsExistingRobotProfileAndStringifiesArgs) {
  auto profile = load_robot_profile(source_path(
      "policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json"));
  ASSERT_TRUE(profile.has_value());
  ASSERT_FALSE(profile.value().sensors.empty());
  EXPECT_EQ(profile.value().sensors.at(6).args.at("baud_rate"), "1000000");
  EXPECT_TRUE(profile.value().axes.empty());
  ASSERT_EQ(profile.value().st3215_servos.size(), 6U);
  EXPECT_EQ(profile.value().st3215_servos.at(0).sensor_name,
            "shoulder_pan_position");
  EXPECT_EQ(profile.value().st3215_servos.at(0).actuator_name,
            "shoulder_pan_target");
  EXPECT_EQ(profile.value().st3215_servos.at(0).serial.path,
            "/tmp/rusty_robot_soarm101");
  EXPECT_EQ(profile.value().st3215_servos.at(0).serial.baud_rate,
            1'000'000U);
  EXPECT_EQ(profile.value().st3215_servos.at(0).device_id, 1U);
  EXPECT_EQ(profile.value().st3215_servos.at(5).device_id, 6U);
}

TEST(ProfileLoaderTest, RejectsInconsistentOrUnsafeSt3215StaticConfiguration) {
  auto profile = read_json(
      "policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json");
  profile["actuators"][1]["args"]["baud_rate"] = 115200;
  expect_robot_rejected(profile);

  profile = read_json(
      "policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json");
  profile["sensors"][7]["args"]["device_id"] = 1;
  expect_robot_rejected(profile);

  profile = read_json(
      "policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json");
  profile["sensors"][6]["args"].erase("device_id");
  profile["sensors"][6]["args"].erase("servo_id");
  expect_robot_rejected(profile);

  profile = read_json(
      "policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json");
  profile["sensors"][6]["args"]["feedback_timeout_ms"] = 0;
  expect_robot_rejected(profile);
}

TEST(ProfileLoaderTest, NormalizesScalarRobotIdentityValuesLikePython) {
  const Json value = {
      {"sensors",
       Json::array({{{"name", 42},
                     {"device", {{"type", " rpc \t"}, {"path", true}}},
                     {"args", Json::object()}}})},
      {"actuators", Json::array()}};
  const TemporaryJsonFile file(value);

  auto profile = load_robot_profile(file.path());

  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  ASSERT_EQ(profile.value().sensors.size(), 1U);
  EXPECT_EQ(profile.value().sensors.at(0).name, "42");
  EXPECT_EQ(profile.value().sensors.at(0).device.type, "rpc");
  EXPECT_EQ(profile.value().sensors.at(0).device.path, "True");
}

TEST(ProfileLoaderTest, NormalizesStructuredRobotIdentityValuesLikePython) {
  const Json value = {
      {"sensors",
       Json::array({{{"name", Json::array({"joint", 1})},
                     {"device",
                      {{"type", Json{{"kind", "rpc"}}},
                       {"path", Json::array({true, nullptr, 2.5})}}},
                     {"args", Json::object()}}})},
      {"actuators", Json::array()}};
  const TemporaryJsonFile file(value);

  auto profile = load_robot_profile(file.path());

  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  ASSERT_EQ(profile.value().sensors.size(), 1U);
  EXPECT_EQ(profile.value().sensors.at(0).name, "['joint', 1]");
  EXPECT_EQ(profile.value().sensors.at(0).device.type, "{'kind': 'rpc'}");
  EXPECT_EQ(profile.value().sensors.at(0).device.path, "[True, None, 2.5]");
}

TEST(ProfileLoaderTest, DerivesCia402AxisFromTrimmedDeviceIdentity) {
  auto value = elmo_profile();
  value["sensors"][0]["name"] = " shoulder_position ";
  value["sensors"][0]["device"]["type"] = " cia402 ";
  value["sensors"][0]["device"]["path"] = " /dev/EtherCAT0 ";
  value["actuators"][0]["device"]["type"] = "\tcia402";
  value["actuators"][0]["device"]["path"] = "/dev/EtherCAT0\n";
  const TemporaryJsonFile file(value);

  auto profile = load_robot_profile(file.path());

  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  ASSERT_EQ(profile.value().axes.size(), 2U);
  EXPECT_EQ(profile.value().axes.at(0).name, "shoulder_position");
  EXPECT_EQ(profile.value().sensors.at(0).device.type, "cia402");
  EXPECT_EQ(profile.value().sensors.at(0).device.path, "/dev/EtherCAT0");
}

TEST(ProfileLoaderTest, ParsesStaticCia402ModesAndHexadecimalIdentity) {
  auto profile = load_robot_profile(source_path("tests/golden/elmo_robot_profile.json"));
  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  ASSERT_EQ(profile.value().axes.size(), 2U);
  EXPECT_EQ(profile.value().axes.at(0).name, "shoulder_position");
  EXPECT_EQ(profile.value().axes.at(0).mode, Cia402Mode::csp);
  EXPECT_EQ(profile.value().axes.at(0).vendor_id, 0x0000009aU);
  EXPECT_EQ(profile.value().axes.at(0).product_code, 0x00030924U);
  EXPECT_EQ(profile.value().axes.at(0).revision, 0x00010420U);
  EXPECT_EQ(profile.value().axes.at(0).command_timeout.count(), 100);
  EXPECT_EQ(profile.value().axes.at(1).mode, Cia402Mode::csv);
  EXPECT_DOUBLE_EQ(profile.value().axes.at(0).slew_limit, 0.05);
  EXPECT_DOUBLE_EQ(profile.value().axes.at(0).following_error_limit, 0.2);
}

TEST(ProfileLoaderTest, RequiresPositiveCia402SlewAndFollowingErrorLimits) {
  auto value = elmo_profile();
  value["sensors"][0]["args"]["slew_limit"] = "0";
  expect_robot_rejected(value);

  value = elmo_profile();
  for (auto* side : {"sensors", "actuators"}) {
    value[side][0]["args"].erase("following_error_limit");
  }
  expect_robot_rejected(value);
}

TEST(ProfileLoaderTest, MapsCstToTheStaticCia402ModeValue) {
  auto value = elmo_profile();
  value["sensors"][1]["args"]["mode"] = "cst";
  value["actuators"][1]["args"]["mode"] = "cst";
  const TemporaryJsonFile file(value);

  auto profile = load_robot_profile(file.path());

  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  EXPECT_EQ(profile.value().axes.at(1).mode, Cia402Mode::cst);
  EXPECT_EQ(static_cast<std::int8_t>(profile.value().axes.at(1).mode), 10);
}

TEST(ProfileLoaderTest, RejectsDuplicateDeviceNamesAndAxisLinks) {
  auto duplicate_name = elmo_profile();
  duplicate_name["sensors"][1]["name"] = "shoulder_position";
  expect_robot_rejected(duplicate_name);

  auto duplicate_sensor_link = elmo_profile();
  duplicate_sensor_link["sensors"][1]["args"]["position"] = "0";
  expect_robot_rejected(duplicate_sensor_link);

  auto normalized_cross_side_name = elmo_profile();
  normalized_cross_side_name["actuators"][0]["name"] = " shoulder_position\t";
  expect_robot_rejected(normalized_cross_side_name);
}

TEST(ProfileLoaderTest, RejectsMoreThanTwelveDerivedAxes) {
  auto profile = elmo_profile();
  profile["sensors"] = Json::array();
  profile["actuators"] = Json::array();
  const auto prototype = elmo_profile()["sensors"][0];
  for (int position = 0; position < 13; ++position) {
    auto sensor = prototype;
    sensor["name"] = "axis_" + std::to_string(position) + "_position";
    sensor["args"]["position"] = std::to_string(position);
    auto actuator = sensor;
    actuator["name"] = "axis_" + std::to_string(position) + "_target";
    profile["sensors"].push_back(std::move(sensor));
    profile["actuators"].push_back(std::move(actuator));
  }
  expect_robot_rejected(profile);
}

TEST(ProfileLoaderTest, RequiresConsistentStaticAxisConfiguration) {
  for (const std::string key : {"vendor_id", "product_code", "revision", "mode",
                                "scale", "minimum", "maximum", "command_timeout_ms",
                                "safety_group", "slew_limit", "following_error_limit"}) {
    auto profile = elmo_profile();
    profile["actuators"][0]["args"][key] = key == "mode" ? Json("cst") : Json("17");
    expect_robot_rejected(profile);
  }
}

TEST(ProfileLoaderTest, RejectsRuntimeModeSwitchAndInvalidModeLimits) {
  auto runtime_switch = elmo_profile();
  runtime_switch["actuators"][0]["args"]["runtime_switch"] = true;
  expect_robot_rejected(runtime_switch);

  auto invalid_mode = elmo_profile();
  invalid_mode["sensors"][0]["args"]["mode"] = "profile_position";
  expect_robot_rejected(invalid_mode);

  auto zero_scale = elmo_profile();
  zero_scale["sensors"][0]["args"]["scale"] = "0";
  expect_robot_rejected(zero_scale);

  auto reversed_limits = elmo_profile();
  reversed_limits["sensors"][0]["args"]["minimum"] = "4";
  expect_robot_rejected(reversed_limits);

  auto missing_limit = elmo_profile();
  missing_limit["sensors"][0]["args"].erase("maximum");
  expect_robot_rejected(missing_limit);
}

TEST(ProfileLoaderTest, LoadsExistingPolicyProfiles) {
  auto profile = load_policy_profile(source_path(
      "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"));
  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  EXPECT_EQ(profile.value().id, "dummy-policy");
  EXPECT_EQ(profile.value().temporal.mode,
            policy_runtime::profiles::TemporalMode::single_step);
  ASSERT_EQ(profile.value().canonical_observation_schema.size(), 3U);
  EXPECT_EQ(profile.value().canonical_observation_schema.at(0).ordering.size(), 7U);

  auto paired = load_policy_profile(source_path(
      "policy_embodied_runtime/examples/policy_profiles/soarm101_sim_policy_profile.json"));
  ASSERT_TRUE(paired.has_value()) << paired.error().message;
  EXPECT_EQ(paired.value().inputs.size(), 6U);
  EXPECT_EQ(paired.value().outputs.size(), 6U);
}

TEST(ProfileLoaderTest, CoercesPolicyBoundsAndBooleanFieldsLikePydantic) {
  auto value = dummy_policy();
  value["canonical_observation_schema"][0]["bounds"] =
      {{"lower", "0.25"}, {"upper", true}};
  value["canonical_observation_schema"][0]["normalized"] = "true";
  value["canonical_observation_schema"][1]["normalized"] = "false";
  value["canonical_observation_schema"][2]["normalized"] = 1;
  value["canonical_action_schema"][0]["normalized"] = 0;
  const TemporaryJsonFile file(value);

  auto profile = load_policy_profile(file.path());

  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  const auto& observations = profile.value().canonical_observation_schema;
  ASSERT_TRUE(observations.at(0).bounds.has_value());
  EXPECT_DOUBLE_EQ(observations.at(0).bounds->lower.value(), 0.25);
  EXPECT_DOUBLE_EQ(observations.at(0).bounds->upper.value(), 1.0);
  EXPECT_TRUE(observations.at(0).normalized);
  EXPECT_FALSE(observations.at(1).normalized);
  EXPECT_TRUE(observations.at(2).normalized);
  EXPECT_FALSE(profile.value().canonical_action_schema.at(0).normalized);
}

TEST(ProfileLoaderTest, MatchesPydanticBoundsNumericStringGrammar) {
  struct AcceptedCase {
    const char* input;
    double expected;
  };
  for (const auto& test_case :
       {AcceptedCase{"1_0", 10.0}, AcceptedCase{"1.", 1.0}, AcceptedCase{"2.0", 2.0}}) {
    auto value = dummy_policy();
    value["canonical_observation_schema"][0]["bounds"] = {{"lower", test_case.input}};
    const TemporaryJsonFile file(value);
    auto profile = load_policy_profile(file.path());
    ASSERT_TRUE(profile.has_value()) << test_case.input << ": " << profile.error().message;
    EXPECT_DOUBLE_EQ(profile.value().canonical_observation_schema.at(0).bounds->lower.value(),
                     test_case.expected);
  }

  for (const char* input : {"nan", "inf"}) {
    auto value = dummy_policy();
    value["canonical_observation_schema"][0]["bounds"] = {{"lower", input}};
    const TemporaryJsonFile file(value);
    auto profile = load_policy_profile(file.path());
    ASSERT_TRUE(profile.has_value()) << input << ": " << profile.error().message;
    const auto parsed =
        profile.value().canonical_observation_schema.at(0).bounds->lower.value();
    if (std::string_view(input) == "nan") {
      EXPECT_TRUE(std::isnan(parsed));
    } else {
      EXPECT_TRUE(std::isinf(parsed));
    }
  }

  for (const char* input : {"1_", "_1"}) {
    auto value = dummy_policy();
    value["canonical_observation_schema"][0]["bounds"] = {{"lower", input}};
    expect_policy_rejected(value);
  }
}

TEST(ProfileLoaderTest, CoercesTemporalIntegersAndBindingBooleansLikePydantic) {
  auto value = read_json(
      "policy_embodied_runtime/examples/policy_profiles/soarm101_sim_policy_profile.json");
  value["temporal"]["action_horizon"] = "4";
  value["temporal"]["observation_history"] = 1.0;
  value["inputs"][0]["optional"] = "false";
  value["inputs"][1]["optional"] = "true";
  value["outputs"][0]["optional"] = 0;
  value["outputs"][1]["optional"] = 1;
  const TemporaryJsonFile file(value);

  auto profile = load_policy_profile(file.path());

  ASSERT_TRUE(profile.has_value()) << profile.error().message;
  EXPECT_EQ(profile.value().temporal.action_horizon, 4U);
  EXPECT_EQ(profile.value().temporal.observation_history, 1U);
  EXPECT_FALSE(profile.value().inputs.at(0).optional);
  EXPECT_TRUE(profile.value().inputs.at(1).optional);
  EXPECT_FALSE(profile.value().outputs.at(0).optional);
  EXPECT_TRUE(profile.value().outputs.at(1).optional);

  value["temporal"]["action_horizon"] = true;
  const TemporaryJsonFile bool_file(value);
  auto bool_profile = load_policy_profile(bool_file.path());
  ASSERT_TRUE(bool_profile.has_value()) << bool_profile.error().message;
  EXPECT_EQ(bool_profile.value().temporal.action_horizon, 1U);

  value["temporal"]["action_horizon"] = "2147483648";
  const TemporaryJsonFile wide_file(value);
  auto wide_profile = load_policy_profile(wide_file.path());
  ASSERT_TRUE(wide_profile.has_value()) << wide_profile.error().message;
  EXPECT_EQ(wide_profile.value().temporal.action_horizon, 2147483648ULL);
}

TEST(ProfileLoaderTest, MatchesPydanticTemporalIntegerStringGrammar) {
  for (const auto& [input, expected] :
       {std::pair{"1_0", 10ULL}, std::pair{"2.0", 2ULL}}) {
    auto value = dummy_policy();
    value["temporal"]["action_horizon"] = input;
    const TemporaryJsonFile file(value);
    auto profile = load_policy_profile(file.path());
    ASSERT_TRUE(profile.has_value()) << input << ": " << profile.error().message;
    EXPECT_EQ(profile.value().temporal.action_horizon, expected);
  }

  for (const char* input : {"nan", "inf", "1_", "_1", "1."}) {
    auto value = dummy_policy();
    value["temporal"]["action_horizon"] = input;
    expect_policy_rejected(value);
  }
}

TEST(ProfileLoaderTest, RejectsPolicyTopLevelAndSchemaViolations) {
  auto unknown = dummy_policy();
  unknown["unexpected"] = true;
  expect_policy_rejected(unknown);

  auto empty_id = dummy_policy();
  empty_id["id"] = " \t";
  expect_policy_rejected(empty_id);

  auto empty_policy = dummy_policy();
  empty_policy["policy"] = "";
  expect_policy_rejected(empty_policy);

  auto duplicate = dummy_policy();
  duplicate["canonical_observation_schema"].push_back(
      duplicate["canonical_observation_schema"][0]);
  expect_policy_rejected(duplicate);
}

TEST(ProfileLoaderTest, RejectsInvalidSemanticFieldsBoundsAndTemporalValues) {
  auto empty_name = dummy_policy();
  empty_name["canonical_observation_schema"][0]["name"] = "";
  expect_policy_rejected(empty_name);

  auto empty_semantic_type = dummy_policy();
  empty_semantic_type["canonical_observation_schema"][0]["semantic_type"] = "  ";
  expect_policy_rejected(empty_semantic_type);

  auto vector_without_ordering = dummy_policy();
  vector_without_ordering["canonical_observation_schema"][0]["ordering"] = Json::array();
  expect_policy_rejected(vector_without_ordering);

  auto reversed_bounds = dummy_policy();
  reversed_bounds["canonical_action_schema"][1]["bounds"] =
      {{"lower", 2.0}, {"upper", 1.0}};
  expect_policy_rejected(reversed_bounds);

  for (const auto& invalid_temporal :
       {Json{{"mode", "streaming"}, {"action_horizon", 1}, {"observation_history", 1}},
        Json{{"mode", "single_step"}, {"action_horizon", 0}, {"observation_history", 1}},
        Json{{"mode", "single_step"}, {"action_horizon", 1}, {"observation_history", 0}},
        Json{{"mode", "single_step"}, {"action_horizon", 1},
             {"observation_history", 1}, {"unexpected", true}}}) {
    auto profile = dummy_policy();
    profile["temporal"] = invalid_temporal;
    expect_policy_rejected(profile);
  }
}

TEST(ProfileLoaderTest, RejectsInvalidProcessorsAndBindings) {
  auto invalid_processor = dummy_policy();
  invalid_processor["preprocess"] = Json::array({{{"name", "  "}}});
  expect_policy_rejected(invalid_processor);

  auto profile = read_json(
      "policy_embodied_runtime/examples/policy_profiles/soarm101_sim_policy_profile.json");
  auto unknown_field = profile;
  unknown_field["inputs"][0]["canonical_field"] = "missing";
  expect_policy_rejected(unknown_field);

  auto duplicate_name = profile;
  duplicate_name["inputs"][1]["name"] = duplicate_name["inputs"][0]["name"];
  expect_policy_rejected(duplicate_name);

  auto duplicate_robot_data = profile;
  duplicate_robot_data["outputs"][1]["robot_data"] =
      duplicate_robot_data["outputs"][0]["robot_data"];
  expect_policy_rejected(duplicate_robot_data);

  auto unequal_pairs = profile;
  unequal_pairs["outputs"].erase(unequal_pairs["outputs"].begin());
  expect_policy_rejected(unequal_pairs);

  auto unexpected_binding_key = profile;
  unexpected_binding_key["inputs"][0]["unexpected"] = true;
  expect_policy_rejected(unexpected_binding_key);
}
