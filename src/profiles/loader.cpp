#include "policy_runtime/profiles/loader.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace policy_runtime::profiles {
namespace {

using Json = nlohmann::json;

template <class T>
Result<T> invalid(std::string message) {
  return Result<T>::failure({ErrorCode::invalid_argument, std::move(message)});
}

bool is_blank(std::string_view value) {
  return std::all_of(value.begin(), value.end(), [](char character) {
    return std::isspace(static_cast<unsigned char>(character)) != 0;
  });
}

std::string trim(std::string_view value) {
  const auto first = std::find_if_not(value.begin(), value.end(), [](char character) {
    return std::isspace(static_cast<unsigned char>(character)) != 0;
  });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](char character) {
                      return std::isspace(static_cast<unsigned char>(character)) != 0;
                    }).base();
  return first < last ? std::string(first, last) : std::string();
}

const Json* member(const Json& object, std::string_view key) {
  const auto found = object.find(std::string(key));
  return found == object.end() ? nullptr : &*found;
}

bool only_keys(const Json& object, std::initializer_list<std::string_view> allowed,
               std::string_view section, std::string& error) {
  for (auto it = object.begin(); it != object.end(); ++it) {
    const bool found = std::any_of(allowed.begin(), allowed.end(), [&](std::string_view key) {
      return it.key() == key;
    });
    if (!found) {
      error = std::string(section) + " contains unknown key: " + it.key();
      return false;
    }
  }
  return true;
}

std::optional<Json> read_json(const std::filesystem::path& path, Error& error) {
  std::ifstream input(path);
  if (!input) {
    error = {ErrorCode::io, "unable to read profile: " + path.string()};
    return std::nullopt;
  }
  try {
    Json value;
    input >> value;
    return value;
  } catch (const Json::exception&) {
    error = {ErrorCode::invalid_argument, "invalid JSON profile"};
    return std::nullopt;
  }
}

std::optional<std::string> required_string(const Json& object, std::string_view key,
                                           std::string_view section, std::string& error,
                                           bool non_empty = false) {
  const auto* value = member(object, key);
  if (value == nullptr || !value->is_string()) {
    error = std::string(section) + " requires string " + std::string(key);
    return std::nullopt;
  }
  auto result = value->get<std::string>();
  if (non_empty && is_blank(result)) {
    error = std::string(section) + "." + std::string(key) + " must be non-empty";
    return std::nullopt;
  }
  return result;
}

bool optional_string(const Json& object, std::string_view key, std::optional<std::string>& out,
                     std::string_view section, std::string& error) {
  const auto* value = member(object, key);
  if (value == nullptr || value->is_null()) {
    return true;
  }
  if (!value->is_string()) {
    error = std::string(section) + "." + std::string(key) + " must be a string or null";
    return false;
  }
  out = value->get<std::string>();
  return true;
}

std::string json_arg_value(const Json& value) {
  if (value.is_string()) {
    return value.get<std::string>();
  }
  if (value.is_boolean()) {
    return value.get<bool>() ? "true" : "false";
  }
  if (value.is_null()) {
    return "None";
  }
  return value.dump();
}

template <class UInt>
std::optional<UInt> parse_unsigned(std::string_view input) {
  const auto value = trim(input);
  if (value.empty() || value.front() == '-') {
    return std::nullopt;
  }
  const bool hexadecimal = value.size() > 2 && value[0] == '0' &&
                           (value[1] == 'x' || value[1] == 'X');
  const std::string_view digits = hexadecimal ? std::string_view(value).substr(2) : value;
  if (digits.empty()) {
    return std::nullopt;
  }
  std::uint64_t parsed{};
  const auto result =
      std::from_chars(digits.data(), digits.data() + digits.size(), parsed, hexadecimal ? 16 : 10);
  if (result.ec != std::errc() || result.ptr != digits.data() + digits.size() ||
      parsed > std::numeric_limits<UInt>::max()) {
    return std::nullopt;
  }
  return static_cast<UInt>(parsed);
}

std::optional<double> parse_double(std::string_view input) {
  const auto value = trim(input);
  if (value.empty()) {
    return std::nullopt;
  }
  char* end = nullptr;
  const double result = std::strtod(value.c_str(), &end);
  if (end != value.c_str() + value.size() || !std::isfinite(result)) {
    return std::nullopt;
  }
  return result;
}

bool parse_device_list(const Json& root, std::string_view plural, std::string_view singular,
                       std::vector<DeviceConfig>& output, std::string& error) {
  const auto* values = member(root, plural);
  if (values == nullptr) {
    return true;
  }
  if (!values->is_array()) {
    error = std::string(plural) + " must be an array";
    return false;
  }
  for (const auto& value : *values) {
    if (!value.is_object()) {
      error = std::string(singular) + " entry must be an object";
      return false;
    }
    const auto name = required_string(value, "name", singular, error, true);
    if (!name) {
      return false;
    }
    const auto* device = member(value, "device");
    if (device == nullptr || !device->is_object()) {
      error = std::string(singular) + "." + *name + ".device must be an object";
      return false;
    }
    const auto type = required_string(*device, "type", "device", error, true);
    const auto path = required_string(*device, "path", "device", error, true);
    if (!type || !path) {
      return false;
    }
    DeviceConfig config{*name, {*type, *path}, {}};
    const auto* args = member(value, "args");
    if (args != nullptr && !args->is_null()) {
      if (!args->is_object()) {
        error = std::string(singular) + "." + *name + ".args must be an object";
        return false;
      }
      for (auto it = args->begin(); it != args->end(); ++it) {
        config.args.emplace(it.key(), json_arg_value(*it));
      }
    }
    output.push_back(std::move(config));
  }
  return true;
}

const std::string* arg(const DeviceConfig& config, std::string_view key, std::string& error) {
  const auto found = config.args.find(std::string(key));
  if (found == config.args.end()) {
    error = config.name + ".args requires " + std::string(key);
    return nullptr;
  }
  return &found->second;
}

std::optional<AxisConfig> parse_axis(const DeviceConfig& config, std::string& error) {
  for (const auto& [key, value] : config.args) {
    static_cast<void>(value);
    if (key == "runtime_mode" || key.find("mode_switch") != std::string::npos ||
        key.find("runtime_switch") != std::string::npos) {
      error = config.name + ".args must not configure runtime mode switching";
      return std::nullopt;
    }
  }

  const auto* alias_text = arg(config, "alias", error);
  const auto* position_text = arg(config, "position", error);
  const auto* vendor_text = arg(config, "vendor_id", error);
  const auto* product_text = arg(config, "product_code", error);
  const auto* revision_text = arg(config, "revision", error);
  const auto* mode_text = arg(config, "mode", error);
  const auto* scale_text = arg(config, "scale", error);
  const auto* minimum_text = arg(config, "minimum", error);
  const auto* maximum_text = arg(config, "maximum", error);
  const auto* timeout_text = arg(config, "command_timeout_ms", error);
  const auto* safety_group = arg(config, "safety_group", error);
  if (!alias_text || !position_text || !vendor_text || !product_text || !revision_text ||
      !mode_text || !scale_text || !minimum_text || !maximum_text || !timeout_text ||
      !safety_group) {
    return std::nullopt;
  }

  const auto alias = parse_unsigned<std::uint16_t>(*alias_text);
  const auto position = parse_unsigned<std::uint16_t>(*position_text);
  const auto vendor = parse_unsigned<std::uint32_t>(*vendor_text);
  const auto product = parse_unsigned<std::uint32_t>(*product_text);
  const auto revision = parse_unsigned<std::uint32_t>(*revision_text);
  const auto timeout = parse_unsigned<std::uint64_t>(*timeout_text);
  const auto scale = parse_double(*scale_text);
  const auto minimum = parse_double(*minimum_text);
  const auto maximum = parse_double(*maximum_text);
  if (!alias || !position || !vendor || !product || !revision || !timeout || !scale ||
      !minimum || !maximum || *timeout > static_cast<std::uint64_t>(
                                     std::numeric_limits<std::int64_t>::max())) {
    error = config.name + ".args contains an invalid numeric value";
    return std::nullopt;
  }
  Cia402Mode mode{};
  if (*mode_text == "csp") {
    mode = Cia402Mode::csp;
  } else if (*mode_text == "csv") {
    mode = Cia402Mode::csv;
  } else if (*mode_text == "cst") {
    mode = Cia402Mode::cst;
  } else {
    error = config.name + ".args.mode must be csp, csv, or cst";
    return std::nullopt;
  }
  if (*scale <= 0.0 || *minimum > *maximum) {
    error = config.name + ".args requires positive scale and minimum <= maximum";
    return std::nullopt;
  }
  if (*timeout == 0 || is_blank(*safety_group)) {
    error = config.name + ".args requires positive timeout and non-empty safety_group";
    return std::nullopt;
  }
  return AxisConfig{config.name,
                    *alias,
                    *position,
                    *vendor,
                    *product,
                    *revision,
                    mode,
                    *scale,
                    *minimum,
                    *maximum,
                    std::chrono::milliseconds(static_cast<std::int64_t>(*timeout)),
                    *safety_group};
}

bool same_static_config(const AxisConfig& left, const AxisConfig& right) {
  return left.alias == right.alias && left.position == right.position &&
         left.vendor_id == right.vendor_id && left.product_code == right.product_code &&
         left.revision == right.revision && left.mode == right.mode && left.scale == right.scale &&
         left.minimum == right.minimum && left.maximum == right.maximum &&
         left.command_timeout == right.command_timeout && left.safety_group == right.safety_group;
}

struct DerivedAxis {
  AxisConfig config;
  bool has_sensor = false;
  bool has_actuator = false;
};

bool derive_axes(const std::vector<DeviceConfig>& sensors,
                 const std::vector<DeviceConfig>& actuators, std::vector<AxisConfig>& axes,
                 std::string& error) {
  std::map<std::string, std::size_t> indices;
  std::vector<DerivedAxis> derived;
  const auto add = [&](const DeviceConfig& device, bool sensor) -> bool {
    if (device.device.type != "cia402") {
      return true;
    }
    const auto parsed = parse_axis(device, error);
    if (!parsed) {
      return false;
    }
    const auto identity = device.device.path + "\n" + std::to_string(parsed->alias) + "\n" +
                          std::to_string(parsed->position);
    const auto found = indices.find(identity);
    if (found == indices.end()) {
      indices.emplace(identity, derived.size());
      derived.push_back({*parsed, sensor, !sensor});
      return true;
    }
    auto& existing = derived.at(found->second);
    if ((sensor && existing.has_sensor) || (!sensor && existing.has_actuator)) {
      error = "duplicate cia402 physical link: " + device.device.path;
      return false;
    }
    if (!same_static_config(existing.config, *parsed)) {
      error = "inconsistent cia402 static configuration for: " + device.device.path;
      return false;
    }
    existing.has_sensor = existing.has_sensor || sensor;
    existing.has_actuator = existing.has_actuator || !sensor;
    if (sensor) {
      existing.config.name = parsed->name;
    }
    return true;
  };
  for (const auto& sensor : sensors) {
    if (!add(sensor, true)) {
      return false;
    }
  }
  for (const auto& actuator : actuators) {
    if (!add(actuator, false)) {
      return false;
    }
  }
  if (derived.size() > 12) {
    error = "robot profile supports at most 12 cia402 axes";
    return false;
  }
  for (const auto& axis : derived) {
    if (!axis.has_sensor || !axis.has_actuator) {
      error = "cia402 physical link requires one sensor and one actuator";
      return false;
    }
    axes.push_back(axis.config);
  }
  return true;
}

bool parse_bounds(const Json& value, SemanticBounds& bounds, std::string& error) {
  if (!value.is_object()) {
    error = "semantic field bounds must be an object";
    return false;
  }
  for (const auto& [key, destination] :
       {std::pair{"lower", &bounds.lower}, std::pair{"upper", &bounds.upper}}) {
    const auto* item = member(value, key);
    if (item == nullptr || item->is_null()) {
      continue;
    }
    if (!item->is_number()) {
      error = std::string("semantic field bounds.") + key + " must be numeric";
      return false;
    }
    *destination = item->get<double>();
  }
  if (bounds.lower && bounds.upper && *bounds.lower > *bounds.upper) {
    error = "semantic field bounds lower must be <= upper";
    return false;
  }
  return true;
}

bool parse_semantic_fields(const Json& root, std::string_view key,
                           std::vector<SemanticField>& fields, std::string& error) {
  const auto* values = member(root, key);
  if (values == nullptr || !values->is_array()) {
    error = std::string(key) + " must be an array";
    return false;
  }
  for (const auto& value : *values) {
    if (!value.is_object() ||
        !only_keys(value,
                   {"name", "semantic_type", "kind", "unit", "ordering", "bounds", "frame",
                    "normalized", "optional", "description"},
                   "semantic field", error)) {
      if (error.empty()) {
        error = "semantic field must be an object";
      }
      return false;
    }
    const auto name = required_string(value, "name", "semantic field", error, true);
    const auto semantic_type =
        required_string(value, "semantic_type", "semantic field", error, true);
    if (!name || !semantic_type) {
      return false;
    }
    SemanticField field;
    field.name = *name;
    field.semantic_type = *semantic_type;
    const auto* kind = member(value, "kind");
    if (kind != nullptr) {
      if (!kind->is_string()) {
        error = "semantic field kind must be a string";
        return false;
      }
      const auto text = kind->get<std::string>();
      if (text == "scalar") field.kind = SemanticKind::scalar;
      else if (text == "vector") field.kind = SemanticKind::vector;
      else if (text == "text") field.kind = SemanticKind::text;
      else if (text == "image") field.kind = SemanticKind::image;
      else if (text == "object") field.kind = SemanticKind::object;
      else {
        error = "semantic field kind is invalid";
        return false;
      }
    }
    if (!optional_string(value, "unit", field.unit, "semantic field", error) ||
        !optional_string(value, "frame", field.frame, "semantic field", error) ||
        !optional_string(value, "description", field.description, "semantic field", error)) {
      return false;
    }
    const auto* ordering = member(value, "ordering");
    if (ordering != nullptr) {
      if (!ordering->is_array()) {
        error = "semantic field ordering must be an array";
        return false;
      }
      for (const auto& item : *ordering) {
        if (!item.is_string()) {
          error = "semantic field ordering must contain strings";
          return false;
        }
        field.ordering.push_back(item.get<std::string>());
      }
    }
    if (field.kind == SemanticKind::vector && field.ordering.empty()) {
      error = "vector fields must declare ordering";
      return false;
    }
    const auto* bounds = member(value, "bounds");
    if (bounds != nullptr && !bounds->is_null()) {
      SemanticBounds parsed;
      if (!parse_bounds(*bounds, parsed, error)) {
        return false;
      }
      field.bounds = parsed;
    }
    for (const auto& [boolean_key, destination] :
         {std::pair{"normalized", &field.normalized}, std::pair{"optional", &field.optional}}) {
      const auto* item = member(value, boolean_key);
      if (item != nullptr) {
        if (!item->is_boolean()) {
          error = std::string("semantic field ") + boolean_key + " must be a boolean";
          return false;
        }
        *destination = item->get<bool>();
      }
    }
    fields.push_back(std::move(field));
  }
  return true;
}

bool parse_temporal(const Json& root, TemporalSpec& temporal, std::string& error) {
  const auto* value = member(root, "temporal");
  if (value == nullptr) {
    return true;
  }
  if (!value->is_object() ||
      !only_keys(*value, {"mode", "action_horizon", "observation_history"}, "temporal", error)) {
    if (error.empty()) error = "temporal must be an object";
    return false;
  }
  const auto* mode = member(*value, "mode");
  if (mode != nullptr) {
    if (!mode->is_string()) {
      error = "temporal.mode must be a string";
      return false;
    }
    if (*mode == "single_step") temporal.mode = TemporalMode::single_step;
    else if (*mode == "chunked") temporal.mode = TemporalMode::chunked;
    else {
      error = "temporal.mode must be single_step or chunked";
      return false;
    }
  }
  for (const auto& [key, destination] :
       {std::pair{"action_horizon", &temporal.action_horizon},
        std::pair{"observation_history", &temporal.observation_history}}) {
    const auto* item = member(*value, key);
    if (item != nullptr) {
      if (!item->is_number_integer() || item->get<std::int64_t>() < 1 ||
          item->get<std::int64_t>() > std::numeric_limits<int>::max()) {
        error = std::string("temporal.") + key + " must be >= 1";
        return false;
      }
      *destination = item->get<int>();
    }
  }
  return true;
}

bool parse_processors(const Json& root, std::string_view key,
                      std::vector<ProcessorSpec>& output, std::string& error) {
  const auto* values = member(root, key);
  if (values == nullptr) return true;
  if (!values->is_array()) {
    error = std::string(key) + " must be an array";
    return false;
  }
  for (const auto& value : *values) {
    if (!value.is_object() ||
        !only_keys(value, {"name", "params"}, "processor", error)) {
      if (error.empty()) error = "processor must be an object";
      return false;
    }
    const auto name = required_string(value, "name", "processor", error, true);
    if (!name) return false;
    const auto* params = member(value, "params");
    if (params != nullptr && !params->is_object()) {
      error = "processor.params must be an object";
      return false;
    }
    output.push_back({*name, params == nullptr ? Json::object() : *params});
  }
  return true;
}

bool parse_bindings(const Json& root, std::string_view key,
                    std::vector<RobotPolicyBinding>& output, std::string& error) {
  const auto* values = member(root, key);
  if (values == nullptr) return true;
  if (!values->is_array()) {
    error = std::string(key) + " must be an array";
    return false;
  }
  for (const auto& value : *values) {
    if (!value.is_object() ||
        !only_keys(value,
                   {"name", "robot_data", "canonical_field", "source_path", "target_path",
                    "optional", "metadata"},
                   "binding", error)) {
      if (error.empty()) error = "binding must be an object";
      return false;
    }
    const auto name = required_string(value, "name", "binding", error, true);
    const auto robot_data = required_string(value, "robot_data", "binding", error, true);
    const auto canonical =
        required_string(value, "canonical_field", "binding", error, true);
    if (!name || !robot_data || !canonical) return false;
    RobotPolicyBinding binding;
    binding.name = *name;
    binding.robot_data = *robot_data;
    binding.canonical_field = *canonical;
    if (!optional_string(value, "source_path", binding.source_path, "binding", error) ||
        !optional_string(value, "target_path", binding.target_path, "binding", error)) {
      return false;
    }
    const auto* optional = member(value, "optional");
    if (optional != nullptr) {
      if (!optional->is_boolean()) {
        error = "binding.optional must be a boolean";
        return false;
      }
      binding.optional = optional->get<bool>();
    }
    const auto* metadata = member(value, "metadata");
    if (metadata != nullptr) {
      if (!metadata->is_object()) {
        error = "binding.metadata must be an object";
        return false;
      }
      binding.metadata = *metadata;
    }
    output.push_back(std::move(binding));
  }
  return true;
}

bool validate_unique_fields(const std::vector<SemanticField>& fields, std::string_view section,
                            std::set<std::string>& names, std::string& error) {
  for (const auto& field : fields) {
    if (!names.insert(field.name).second) {
      error = std::string(section) + " contains duplicate field names";
      return false;
    }
  }
  return true;
}

bool validate_bindings(const std::vector<RobotPolicyBinding>& bindings,
                       const std::set<std::string>& canonical_fields, std::string_view section,
                       std::string& error) {
  std::set<std::string> names;
  std::set<std::string> robot_data;
  for (const auto& binding : bindings) {
    if (!names.insert(binding.name).second) {
      error = std::string(section) + " contains duplicate binding names";
      return false;
    }
    if (!robot_data.insert(binding.robot_data).second) {
      error = std::string(section) + " contains duplicate robot_data names";
      return false;
    }
    if (!canonical_fields.contains(binding.canonical_field)) {
      error = std::string(section) + "." + binding.name +
              " references unknown canonical field '" + binding.canonical_field + "'";
      return false;
    }
  }
  return true;
}

}  // namespace

Result<RobotProfile> load_robot_profile(const std::filesystem::path& path) {
  Error load_error;
  const auto root = read_json(path, load_error);
  if (!root) {
    return Result<RobotProfile>::failure(std::move(load_error));
  }
  if (!root->is_object()) {
    return invalid<RobotProfile>("robot profile must be an object");
  }
  RobotProfile profile;
  std::string error;
  if (!parse_device_list(*root, "sensors", "sensor", profile.sensors, error) ||
      !parse_device_list(*root, "actuators", "actuator", profile.actuators, error)) {
    return invalid<RobotProfile>(std::move(error));
  }
  std::set<std::string> names;
  for (const auto& device : profile.sensors) {
    if (!names.insert(device.name).second) {
      return invalid<RobotProfile>("duplicate device name '" + device.name + "'");
    }
  }
  for (const auto& device : profile.actuators) {
    if (!names.insert(device.name).second) {
      return invalid<RobotProfile>("duplicate device name '" + device.name + "'");
    }
  }
  if (!derive_axes(profile.sensors, profile.actuators, profile.axes, error)) {
    return invalid<RobotProfile>(std::move(error));
  }
  return Result<RobotProfile>::success(std::move(profile));
}

Result<PolicyProfile> load_policy_profile(const std::filesystem::path& path) {
  Error load_error;
  const auto root = read_json(path, load_error);
  if (!root) {
    return Result<PolicyProfile>::failure(std::move(load_error));
  }
  if (!root->is_object()) {
    return invalid<PolicyProfile>("policy profile must be an object");
  }
  std::string error;
  if (!only_keys(*root,
                 {"id", "version", "policy", "model", "inputs", "outputs",
                  "canonical_observation_schema", "canonical_action_schema", "temporal",
                  "preprocess", "postprocess", "safety"},
                 "policy profile", error)) {
    return invalid<PolicyProfile>(std::move(error));
  }
  const auto id = required_string(*root, "id", "policy profile", error, true);
  const auto version = required_string(*root, "version", "policy profile", error);
  const auto policy = required_string(*root, "policy", "policy profile", error, true);
  if (!id || !version || !policy) {
    return invalid<PolicyProfile>(std::move(error));
  }
  PolicyProfile profile;
  profile.id = *id;
  profile.version = *version;
  profile.policy = *policy;
  for (const auto& [key, destination] :
       {std::pair{"model", &profile.model}, std::pair{"safety", &profile.safety}}) {
    const auto* value = member(*root, key);
    if (value != nullptr) {
      if (!value->is_object()) {
        return invalid<PolicyProfile>(std::string("policy profile.") + key +
                                      " must be an object");
      }
      *destination = *value;
    }
  }
  if (!parse_bindings(*root, "inputs", profile.inputs, error) ||
      !parse_bindings(*root, "outputs", profile.outputs, error) ||
      !parse_semantic_fields(*root, "canonical_observation_schema",
                             profile.canonical_observation_schema, error) ||
      !parse_semantic_fields(*root, "canonical_action_schema",
                             profile.canonical_action_schema, error) ||
      !parse_temporal(*root, profile.temporal, error) ||
      !parse_processors(*root, "preprocess", profile.preprocess, error) ||
      !parse_processors(*root, "postprocess", profile.postprocess, error)) {
    return invalid<PolicyProfile>(std::move(error));
  }
  std::set<std::string> observation_names;
  std::set<std::string> action_names;
  if (!validate_unique_fields(profile.canonical_observation_schema,
                              "canonical_observation_schema", observation_names, error) ||
      !validate_unique_fields(profile.canonical_action_schema, "canonical_action_schema",
                              action_names, error) ||
      !validate_bindings(profile.inputs, observation_names, "inputs", error) ||
      !validate_bindings(profile.outputs, action_names, "outputs", error)) {
    return invalid<PolicyProfile>(std::move(error));
  }
  if (profile.inputs.empty() != profile.outputs.empty() ||
      profile.inputs.size() != profile.outputs.size()) {
    return invalid<PolicyProfile>(
        "policy inputs and outputs must be declared as equal-length pairs");
  }
  return Result<PolicyProfile>::success(std::move(profile));
}

}  // namespace policy_runtime::profiles
