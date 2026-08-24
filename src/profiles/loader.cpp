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

using Json = nlohmann::ordered_json;

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

std::string python_string_repr(std::string_view value) {
  std::string result{"'"};
  for (const char character : value) {
    if (character == '\\' || character == '\'') {
      result.push_back('\\');
      result.push_back(character);
    } else if (character == '\n') {
      result += "\\n";
    } else if (character == '\r') {
      result += "\\r";
    } else if (character == '\t') {
      result += "\\t";
    } else {
      result.push_back(character);
    }
  }
  result.push_back('\'');
  return result;
}

std::string python_repr(const Json& value) {
  if (value.is_string()) {
    return python_string_repr(value.get_ref<const std::string&>());
  }
  if (value.is_boolean()) {
    return value.get<bool>() ? "True" : "False";
  }
  if (value.is_null()) {
    return "None";
  }
  if (value.is_number()) {
    return value.dump();
  }
  if (value.is_array()) {
    std::string result{"["};
    for (std::size_t index = 0; index < value.size(); ++index) {
      if (index != 0) {
        result += ", ";
      }
      result += python_repr(value.at(index));
    }
    result.push_back(']');
    return result;
  }
  if (value.is_object()) {
    std::string result{"{"};
    bool first = true;
    for (auto it = value.begin(); it != value.end(); ++it) {
      if (!first) {
        result += ", ";
      }
      first = false;
      result += python_string_repr(it.key()) + ": " + python_repr(*it);
    }
    result.push_back('}');
    return result;
  }
  return value.dump();
}

std::string python_value_string(const Json& value) {
  return value.is_string() ? value.get<std::string>() : python_repr(value);
}

std::optional<std::string> required_robot_string(const Json& object, std::string_view key,
                                                 std::string_view section,
                                                 std::string& error) {
  const auto* value = member(object, key);
  if (value == nullptr) {
    error = std::string(section) + " is missing " + std::string(key);
    return std::nullopt;
  }
  auto normalized = trim(python_value_string(*value));
  if (normalized.empty()) {
    error = std::string(section) + " is missing " + std::string(key);
    return std::nullopt;
  }
  return normalized;
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

std::optional<double> parse_pydantic_double_string(std::string_view input) {
  const auto value = trim(input);
  if (value.empty()) {
    return std::nullopt;
  }
  std::string normalized;
  normalized.reserve(value.size());
  for (std::size_t index = 0; index < value.size(); ++index) {
    const char character = value[index];
    if (character != '_') {
      normalized.push_back(character);
      continue;
    }
    if (index == 0 || index + 1 == value.size() ||
        !std::isdigit(static_cast<unsigned char>(value[index - 1])) ||
        !std::isdigit(static_cast<unsigned char>(value[index + 1]))) {
      return std::nullopt;
    }
  }
  char* end = nullptr;
  const double result = std::strtod(normalized.c_str(), &end);
  if (end != normalized.c_str() + normalized.size()) {
    return std::nullopt;
  }
  return result;
}

std::optional<double> coerce_pydantic_double(const Json& value) {
  if (value.is_boolean()) {
    return value.get<bool>() ? 1.0 : 0.0;
  }
  if (value.is_number()) {
    return value.get<double>();
  }
  if (value.is_string()) {
    return parse_pydantic_double_string(value.get_ref<const std::string&>());
  }
  return std::nullopt;
}

std::optional<bool> coerce_pydantic_bool(const Json& value) {
  if (value.is_boolean()) {
    return value.get<bool>();
  }
  if (value.type() == Json::value_t::number_integer) {
    const auto integer = value.get<std::int64_t>();
    if (integer == 0 || integer == 1) {
      return integer == 1;
    }
    return std::nullopt;
  }
  if (value.type() == Json::value_t::number_unsigned) {
    const auto integer = value.get<std::uint64_t>();
    if (integer == 0 || integer == 1) {
      return integer == 1;
    }
    return std::nullopt;
  }
  if (value.is_number_float()) {
    const auto number = value.get<double>();
    if (number == 0.0 || number == 1.0) {
      return number == 1.0;
    }
    return std::nullopt;
  }
  if (!value.is_string()) {
    return std::nullopt;
  }
  auto text = value.get<std::string>();
  std::transform(text.begin(), text.end(), text.begin(), [](char character) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  });
  if (text == "1" || text == "true" || text == "t" || text == "yes" || text == "y" ||
      text == "on") {
    return true;
  }
  if (text == "0" || text == "false" || text == "f" || text == "no" || text == "n" ||
      text == "off") {
    return false;
  }
  return std::nullopt;
}

std::optional<std::uint64_t> parse_pydantic_unsigned_integer_string(
    std::string_view input) {
  const auto text = trim(input);
  if (text.empty()) {
    return std::nullopt;
  }
  std::size_t offset = 0;
  bool negative = false;
  if (text[offset] == '+' || text[offset] == '-') {
    negative = text[offset] == '-';
    ++offset;
  }
  if (offset == text.size()) {
    return std::nullopt;
  }

  std::uint64_t result = 0;
  bool saw_digit = false;
  bool decimal = false;
  bool saw_fractional_digit = false;
  for (; offset < text.size(); ++offset) {
    const char character = text[offset];
    if (character == '_') {
      if (offset == 0 || offset + 1 == text.size() ||
          !std::isdigit(static_cast<unsigned char>(text[offset - 1])) ||
          !std::isdigit(static_cast<unsigned char>(text[offset + 1]))) {
        return std::nullopt;
      }
      continue;
    }
    if (character == '.') {
      if (decimal || !saw_digit) {
        return std::nullopt;
      }
      decimal = true;
      continue;
    }
    if (character < '0' || character > '9') {
      return std::nullopt;
    }
    saw_digit = true;
    const auto digit = static_cast<std::uint64_t>(character - '0');
    if (decimal) {
      saw_fractional_digit = true;
      if (digit != 0) {
        return std::nullopt;
      }
      continue;
    }
    if (result > (std::numeric_limits<std::uint64_t>::max() - digit) / 10U) {
      return std::nullopt;
    }
    result = result * 10U + digit;
  }
  if (!saw_digit || (decimal && !saw_fractional_digit) || (negative && result != 0)) {
    return std::nullopt;
  }
  return result;
}

std::optional<std::uint64_t> coerce_pydantic_positive_integer(const Json& value) {
  if (value.is_boolean()) {
    return value.get<bool>() ? 1U : 0U;
  }
  if (value.type() == Json::value_t::number_unsigned) {
    return value.get<std::uint64_t>();
  }
  if (value.type() == Json::value_t::number_integer) {
    const auto integer = value.get<std::int64_t>();
    return integer >= 0 ? std::optional<std::uint64_t>(static_cast<std::uint64_t>(integer))
                        : std::nullopt;
  }

  long double number{};
  if (value.is_number_float()) {
    number = static_cast<long double>(value.get<double>());
  } else if (value.is_string()) {
    return parse_pydantic_unsigned_integer_string(value.get_ref<const std::string&>());
  } else {
    return std::nullopt;
  }

  const auto upper_bound = std::ldexp(1.0L, 64);
  if (!std::isfinite(number) || number < 0.0L || number >= upper_bound ||
      std::trunc(number) != number) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(number);
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
    const auto name = required_robot_string(value, "name", singular, error);
    if (!name) {
      return false;
    }
    const auto* device = member(value, "device");
    if (device == nullptr || !device->is_object()) {
      error = std::string(singular) + "." + *name + ".device must be an object";
      return false;
    }
    const auto type = required_robot_string(*device, "type", "device", error);
    const auto path = required_robot_string(*device, "path", "device", error);
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
  const auto* slew_text = arg(config, "slew_limit", error);
  const auto* following_error_text = arg(config, "following_error_limit", error);
  const auto* timeout_text = arg(config, "command_timeout_ms", error);
  const auto* safety_group = arg(config, "safety_group", error);
  if (!alias_text || !position_text || !vendor_text || !product_text || !revision_text ||
      !mode_text || !scale_text || !minimum_text || !maximum_text || !slew_text ||
      !following_error_text || !timeout_text || !safety_group) {
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
  const auto slew_limit = parse_double(*slew_text);
  const auto following_error_limit = parse_double(*following_error_text);
  if (!alias || !position || !vendor || !product || !revision || !timeout || !scale ||
      !minimum || !maximum || !slew_limit || !following_error_limit ||
      *timeout > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
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
  if (*scale <= 0.0 || *minimum > *maximum || *slew_limit <= 0.0 ||
      *following_error_limit <= 0.0) {
    error = config.name +
            ".args requires positive scale/safety limits and minimum <= maximum";
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
                    *safety_group,
                    *slew_limit,
                    *following_error_limit};
}

bool same_static_config(const AxisConfig& left, const AxisConfig& right) {
  return left.alias == right.alias && left.position == right.position &&
         left.vendor_id == right.vendor_id && left.product_code == right.product_code &&
         left.revision == right.revision && left.mode == right.mode && left.scale == right.scale &&
         left.minimum == right.minimum && left.maximum == right.maximum &&
         left.command_timeout == right.command_timeout && left.safety_group == right.safety_group &&
         left.slew_limit == right.slew_limit &&
         left.following_error_limit == right.following_error_limit;
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

const std::string* optional_arg(const DeviceConfig& config,
                                std::string_view key) {
  const auto found = config.args.find(std::string(key));
  return found == config.args.end() ? nullptr : &found->second;
}

template <class T>
std::optional<T> unsigned_arg(const DeviceConfig& config,
                              std::string_view key, T default_value,
                              std::string& error) {
  const auto* text = optional_arg(config, key);
  if (text == nullptr) {
    return default_value;
  }
  const auto parsed = parse_unsigned<T>(*text);
  if (!parsed) {
    error = config.name + ".args." + std::string(key) +
            " must be an unsigned integer";
  }
  return parsed;
}

std::optional<std::chrono::milliseconds> milliseconds_from_seconds_arg(
    const DeviceConfig& config, std::string_view key,
    std::chrono::milliseconds default_value, std::string& error) {
  const auto* text = optional_arg(config, key);
  if (text == nullptr) {
    return default_value;
  }
  const auto seconds = parse_double(*text);
  if (!seconds || *seconds <= 0.0 ||
      *seconds > static_cast<double>(std::numeric_limits<std::int64_t>::max()) /
                     1000.0) {
    error = config.name + ".args." + std::string(key) +
            " must be a positive finite duration";
    return std::nullopt;
  }
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(*seconds));
  if (milliseconds.count() <= 0) {
    error = config.name + ".args." + std::string(key) +
            " is below one millisecond";
    return std::nullopt;
  }
  return milliseconds;
}

std::optional<std::uint16_t> st3215_identity_arg(
    const DeviceConfig& config, std::string_view primary,
    std::string_view fallback, std::string& error) {
  const auto* text = optional_arg(config, primary);
  if (text == nullptr) {
    text = optional_arg(config, fallback);
  }
  if (text == nullptr) {
    error = config.name + ".args requires " + std::string(primary);
    return std::nullopt;
  }
  const auto value = parse_unsigned<std::uint16_t>(*text);
  if (!value) {
    error = config.name + ".args." + std::string(primary) +
            " must be an unsigned integer";
  }
  return value;
}

std::optional<St3215ServoProfile> parse_st3215(
    const DeviceConfig& config, bool sensor, std::string& error) {
  const auto* baud_text = arg(config, "baud_rate", error);
  const auto device_id =
      st3215_identity_arg(config, "device_id", "servo_id", error);
  const auto servo_id =
      st3215_identity_arg(config, "servo_id", "device_id", error);
  if (baud_text == nullptr || !device_id || !servo_id) {
    return std::nullopt;
  }
  const auto baud_rate = parse_unsigned<std::uint32_t>(*baud_text);
  const auto read_buffer =
      unsigned_arg<std::size_t>(config, "read_buffer_len", 256U, error);
  const auto maximum_frame =
      unsigned_arg<std::size_t>(config, "maximum_frame_size", 259U, error);
  const auto max_position = unsigned_arg<std::uint16_t>(
      config, "max_position_units", 4095U, error);
  const auto speed =
      unsigned_arg<std::uint16_t>(config, "speed_units", 0U, error);
  const auto time =
      unsigned_arg<std::uint16_t>(config, "time_units", 0U, error);
  const auto feedback_timeout = unsigned_arg<std::uint64_t>(
      config, "feedback_timeout_ms", 250U, error);
  const auto service_period = unsigned_arg<std::uint64_t>(
      config, "service_period_us", 1000U, error);
  const auto* timeout_key = optional_arg(config, "timeout_s") != nullptr
                                ? "timeout_s"
                                : (optional_arg(config, "timeout") != nullptr
                                       ? "timeout"
                                       : "timeout_s");
  const auto read_timeout = milliseconds_from_seconds_arg(
      config, timeout_key, std::chrono::milliseconds{20}, error);
  auto write_timeout = read_timeout;
  if (optional_arg(config, "write_timeout_s") != nullptr) {
    write_timeout = milliseconds_from_seconds_arg(
        config, "write_timeout_s", std::chrono::milliseconds{20}, error);
  }
  if (!baud_rate || !read_buffer || !maximum_frame || !max_position ||
      !speed || !time || !feedback_timeout || !service_period ||
      !read_timeout || !write_timeout) {
    if (error.empty()) {
      error = config.name + ".args contains invalid ST3215 configuration";
    }
    return std::nullopt;
  }
  if (*baud_rate == 0U || *device_id > 0xfdU || *max_position == 0U ||
      *read_buffer == 0U || *read_buffer > 4096U || *maximum_frame < 6U ||
      *maximum_frame > 259U || *feedback_timeout == 0U ||
      *feedback_timeout >
          static_cast<std::uint64_t>(
              std::numeric_limits<std::int64_t>::max() / 1'000'000LL) ||
      *service_period == 0U ||
      *service_period >
          static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    error = config.name + ".args contains unsafe ST3215 limits";
    return std::nullopt;
  }

  St3215ServoProfile result;
  if (sensor) {
    result.sensor_name = config.name;
  } else {
    result.actuator_name = config.name;
  }
  result.serial = SerialPortConfig{
      config.device.path,
      *baud_rate,
      *read_buffer,
      *maximum_frame,
      *read_timeout,
      *write_timeout,
      std::chrono::microseconds(static_cast<std::int64_t>(*service_period))};
  result.device_id = static_cast<std::uint8_t>(*device_id);
  result.servo_id = *servo_id;
  result.max_position_units = *max_position;
  result.speed_units = *speed;
  result.time_units = *time;
  result.feedback_timeout = std::chrono::milliseconds(
      static_cast<std::int64_t>(*feedback_timeout));
  const auto* safety_group = optional_arg(config, "safety_group");
  result.safety_group =
      safety_group == nullptr || is_blank(*safety_group)
          ? config.device.path
          : *safety_group;
  return result;
}

bool same_serial(const SerialPortConfig& left,
                 const SerialPortConfig& right) {
  return left.path == right.path && left.baud_rate == right.baud_rate &&
         left.read_buffer_size == right.read_buffer_size &&
         left.maximum_frame_size == right.maximum_frame_size &&
         left.read_timeout == right.read_timeout &&
         left.write_timeout == right.write_timeout &&
         left.service_period == right.service_period;
}

bool same_st3215_physical_config(const St3215ServoProfile& left,
                                 const St3215ServoProfile& right) {
  return same_serial(left.serial, right.serial) &&
         left.device_id == right.device_id &&
         left.servo_id == right.servo_id &&
         left.max_position_units == right.max_position_units &&
         left.feedback_timeout == right.feedback_timeout &&
         left.safety_group == right.safety_group;
}

bool derive_st3215(const std::vector<DeviceConfig>& sensors,
                   const std::vector<DeviceConfig>& actuators,
                   std::vector<St3215ServoProfile>& servos,
                   std::string& error) {
  struct Derived {
    St3215ServoProfile profile;
    bool has_sensor{};
    bool has_actuator{};
  };
  std::vector<Derived> derived;
  std::map<std::string, std::size_t> identities;
  std::map<std::string, SerialPortConfig> buses;
  const auto add = [&](const DeviceConfig& device, bool sensor) -> bool {
    if (device.device.type != "st3215") {
      return true;
    }
    auto parsed = parse_st3215(device, sensor, error);
    if (!parsed) {
      return false;
    }
    const auto bus = buses.find(parsed->serial.path);
    if (bus == buses.end()) {
      buses.emplace(parsed->serial.path, parsed->serial);
    } else if (!same_serial(bus->second, parsed->serial)) {
      error = "inconsistent serial configuration for: " + parsed->serial.path;
      return false;
    }
    const auto identity = parsed->serial.path + "\n" +
                          std::to_string(parsed->device_id);
    const auto existing = identities.find(identity);
    if (existing == identities.end()) {
      identities.emplace(identity, derived.size());
      derived.push_back({std::move(*parsed), sensor, !sensor});
      return true;
    }
    auto& physical = derived.at(existing->second);
    if ((sensor && physical.has_sensor) ||
        (!sensor && physical.has_actuator)) {
      error = "duplicate ST3215 physical link: " + device.device.path;
      return false;
    }
    if (!same_st3215_physical_config(physical.profile, *parsed)) {
      error = "inconsistent ST3215 static configuration for: " +
              device.device.path;
      return false;
    }
    if (sensor) {
      physical.profile.sensor_name = parsed->sensor_name;
    } else {
      physical.profile.actuator_name = parsed->actuator_name;
      physical.profile.speed_units = parsed->speed_units;
      physical.profile.time_units = parsed->time_units;
    }
    physical.has_sensor = physical.has_sensor || sensor;
    physical.has_actuator = physical.has_actuator || !sensor;
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
  if (derived.size() > 32U) {
    error = "robot profile supports at most 32 ST3215 servos";
    return false;
  }
  for (auto& physical : derived) {
    if (!physical.has_sensor || !physical.has_actuator) {
      error = "ST3215 physical link requires one sensor and one actuator";
      return false;
    }
    servos.push_back(std::move(physical.profile));
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
    const auto converted = coerce_pydantic_double(*item);
    if (!converted) {
      error = std::string("semantic field bounds.") + key + " must be numeric";
      return false;
    }
    *destination = *converted;
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
        const auto converted = coerce_pydantic_bool(*item);
        if (!converted) {
          error = std::string("semantic field ") + boolean_key + " must be a boolean";
          return false;
        }
        *destination = *converted;
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
      const auto converted = coerce_pydantic_positive_integer(*item);
      if (!converted || *converted < 1) {
        error = std::string("temporal.") + key + " must be >= 1";
        return false;
      }
      *destination = *converted;
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
      const auto converted = coerce_pydantic_bool(*optional);
      if (!converted) {
        error = "binding.optional must be a boolean";
        return false;
      }
      binding.optional = *converted;
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
  if (!derive_st3215(profile.sensors, profile.actuators,
                     profile.st3215_servos, error)) {
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
  const auto* model = member(*root, "model");
  if (model != nullptr) {
    if (!model->is_object()) {
      return invalid<PolicyProfile>("policy profile.model must be an object");
    }
    profile.model = *model;
  }
  const auto* safety = member(*root, "safety");
  if (safety != nullptr) {
    if (!safety->is_object()) {
      return invalid<PolicyProfile>("policy profile.safety must be an object");
    }
    profile.safety = *safety;
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
