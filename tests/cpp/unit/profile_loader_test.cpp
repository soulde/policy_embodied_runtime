#include <atomic>
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
                                "safety_group"}) {
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
