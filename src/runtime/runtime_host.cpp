#include "policy_runtime/runtime/runtime_host.hpp"

#include <algorithm>
#include <chrono>
#include <charconv>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "policy_runtime/profiles/loader.hpp"
#include "policy_runtime/protocol/rpc/codec.hpp"
#include "policy_runtime/devices/cia402/axis.hpp"

namespace policy_runtime {
namespace {

std::int64_t realtime_now_ns() noexcept {
  const auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  return value < 0 ? 0 : value;
}

std::int64_t monotonic_now_ns() noexcept {
  const auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  return value < 0 ? 0 : value;
}

std::optional<std::uint16_t> parse_bus_position(std::string_view text) {
  int base = 10;
  if (text.size() > 2 && text[0] == '0' &&
      (text[1] == 'x' || text[1] == 'X')) {
    text.remove_prefix(2);
    base = 16;
  }
  std::uint32_t value{};
  const auto parsed =
      std::from_chars(text.data(), text.data() + text.size(), value, base);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
      value > std::numeric_limits<std::uint16_t>::max()) {
    return std::nullopt;
  }
  return static_cast<std::uint16_t>(value);
}

bool matches_axis(const profiles::DeviceConfig& device,
                  const profiles::AxisConfig& axis,
                  std::string_view path) {
  const auto alias = device.args.find("alias");
  const auto position = device.args.find("position");
  return device.device.type == "cia402" && device.device.path == path &&
         alias != device.args.end() && position != device.args.end() &&
         parse_bus_position(alias->second) == axis.alias &&
         parse_bus_position(position->second) == axis.position;
}

Result<std::vector<std::string>> axis_actuator_names(
    const profiles::RobotProfile& profile) {
  std::vector<std::string> names;
  names.reserve(profile.axes.size());
  for (const auto& axis : profile.axes) {
    const auto sensor = std::find_if(
        profile.sensors.begin(), profile.sensors.end(),
        [&](const auto& candidate) {
          return candidate.device.type == "cia402" &&
                 candidate.name == axis.name;
        });
    if (sensor == profile.sensors.end()) {
      return Result<std::vector<std::string>>::failure(
          {ErrorCode::invalid_argument,
           "cia402 axis has no matching sensor: " + axis.name});
    }
    const auto actuator = std::find_if(
        profile.actuators.begin(), profile.actuators.end(),
        [&](const auto& candidate) {
          return matches_axis(candidate, axis, sensor->device.path);
        });
    if (actuator == profile.actuators.end()) {
      return Result<std::vector<std::string>>::failure(
          {ErrorCode::invalid_argument,
           "cia402 axis has no matching actuator: " + axis.name});
    }
    names.push_back(actuator->name);
  }
  return Result<std::vector<std::string>>::success(std::move(names));
}

void normalize_joint_state_numbers(nlohmann::json& value) {
  if (!value.is_object()) {
    return;
  }
  const auto values = value.find("values");
  if (values == value.end() || !values->is_array()) {
    return;
  }
  for (auto& item : *values) {
    if (item.is_number()) {
      item = item.get<double>();
    }
  }
}

void normalize_gripper_number(nlohmann::json& value) {
  if (!value.is_object()) {
    return;
  }
  const auto scalar = value.find("value");
  if (scalar != value.end() && scalar->is_number()) {
    *scalar = scalar->get<double>();
  }
}

nlohmann::json normalized_observation(const nlohmann::json& input) {
  auto observation = input;
  for (const auto field : {"joint_position", "gripper_width", "task_text",
                           "image"}) {
    if (const auto value = observation.find(field);
        value != observation.end() && value->is_null()) {
      observation.erase(value);
    }
  }
  if (!observation.contains("meta")) {
    observation["meta"] = nlohmann::json::object();
  }
  if (auto joint = observation.find("joint_position");
      joint != observation.end()) {
    normalize_joint_state_numbers(*joint);
  }
  if (auto gripper = observation.find("gripper_width");
      gripper != observation.end()) {
    normalize_gripper_number(*gripper);
  }
  if (auto image = observation.find("image");
      image != observation.end() && image->is_object()) {
    if (!image->contains("encoding")) {
      (*image)["encoding"] = "base64";
    }
    if (!image->contains("mime_type")) {
      (*image)["mime_type"] = "image/jpeg";
    }
  }
  return observation;
}

nlohmann::json normalized_action(const nlohmann::json& input) {
  nlohmann::json action = {{"joint_position_delta", nullptr},
                           {"gripper_command", nullptr},
                           {"action_chunk", nlohmann::json::array()},
                           {"meta", nlohmann::json::object()}};
  for (auto item = input.begin(); item != input.end(); ++item) {
    action[item.key()] = item.value();
  }
  normalize_joint_state_numbers(action["joint_position_delta"]);
  normalize_gripper_number(action["gripper_command"]);
  return action;
}

std::string missing_fields_message(const std::vector<std::string>& fields) {
  std::string message = "missing required canonical observation fields: [";
  for (std::size_t index = 0; index < fields.size(); ++index) {
    if (index != 0) {
      message += ", ";
    }
    message += "'" + fields[index] + "'";
  }
  return message + "]";
}

double feedback_value(const profiles::AxisConfig& axis,
                      const AxisFeedback& feedback) noexcept {
  switch (axis.mode) {
    case profiles::Cia402Mode::csp:
      return feedback.position;
    case profiles::Cia402Mode::csv:
      return feedback.velocity;
    case profiles::Cia402Mode::cst:
      return feedback.effort;
  }
  return feedback.position;
}

std::optional<double> action_target(const nlohmann::json& value,
                                    profiles::Cia402Mode mode) {
  if (value.is_number()) {
    return value.get<double>();
  }
  if (!value.is_object()) {
    return std::nullopt;
  }
  std::vector<std::string_view> keys{"target", "value"};
  switch (mode) {
    case profiles::Cia402Mode::csp:
      keys.insert(keys.begin(), {"target_position", "target_position_rad",
                                 "position"});
      break;
    case profiles::Cia402Mode::csv:
      keys.insert(keys.begin(), {"target_velocity", "velocity"});
      break;
    case profiles::Cia402Mode::cst:
      keys.insert(keys.begin(), {"target_effort", "target_torque", "effort",
                                 "torque"});
      break;
  }
  for (const auto key : keys) {
    const auto found = value.find(std::string(key));
    if (found != value.end() && found->is_number()) {
      return found->get<double>();
    }
  }
  return std::nullopt;
}

}  // namespace

Result<RuntimeHost> RuntimeHost::from_profiles(
    const std::filesystem::path& policy_profile,
    std::optional<std::filesystem::path> robot_profile,
    std::unique_ptr<RuntimeRobotIo> robot_io) {
  auto profile = profiles::load_policy_profile(policy_profile);
  if (!profile.has_value()) {
    return Result<RuntimeHost>::failure(profile.error());
  }
  auto policy = make_policy(profile.value());
  if (!policy.has_value()) {
    return Result<RuntimeHost>::failure(policy.error());
  }
  auto preprocess =
      ProcessorPipeline::create(profile.value().preprocess, "preprocess");
  if (!preprocess.has_value()) {
    return Result<RuntimeHost>::failure(preprocess.error());
  }
  auto postprocess =
      ProcessorPipeline::create(profile.value().postprocess, "postprocess");
  if (!postprocess.has_value()) {
    return Result<RuntimeHost>::failure(postprocess.error());
  }
  profiles::RobotProfile loaded_robot;
  if (robot_profile.has_value()) {
    auto loaded = profiles::load_robot_profile(*robot_profile);
    if (!loaded.has_value()) {
      return Result<RuntimeHost>::failure(loaded.error());
    }
    loaded_robot = std::move(loaded.value());
  }
  auto actuator_names = axis_actuator_names(loaded_robot);
  if (!actuator_names.has_value()) {
    return Result<RuntimeHost>::failure(actuator_names.error());
  }
  return Result<RuntimeHost>::success(RuntimeHost(
      std::move(profile.value()), std::move(loaded_robot),
      std::move(actuator_names.value()), std::move(policy.value()),
      std::move(preprocess.value()), std::move(postprocess.value()),
      std::move(robot_io)));
}

RuntimeHost::RuntimeHost(profiles::PolicyProfile profile,
                         profiles::RobotProfile robot_profile,
                         std::vector<std::string> axis_actuator_names,
                         std::unique_ptr<Policy> policy,
                         ProcessorPipeline preprocess,
                         ProcessorPipeline postprocess,
                         std::unique_ptr<RuntimeRobotIo> robot_io)
    : profile_(std::move(profile)),
      robot_profile_(std::move(robot_profile)),
      axis_actuator_names_(std::move(axis_actuator_names)),
      policy_(std::move(policy)),
      preprocess_(std::move(preprocess)),
      postprocess_(std::move(postprocess)),
      robot_io_(std::move(robot_io)) {}

Result<void> RuntimeHost::open() {
  if ((!robot_profile_.axes.empty() || !robot_profile_.st3215_servos.empty()) &&
      !robot_io_) {
    return Result<void>::failure(
        {ErrorCode::unavailable,
         "robot profile requires a robot I/O daemon connection"});
  }
  if (robot_io_ && robot_io_->axis_count() != robot_profile_.axes.size()) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "robot I/O axis count does not match robot profile"});
  }
  if (robot_io_ &&
      robot_io_->servo_count() != robot_profile_.st3215_servos.size()) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "robot I/O servo count does not match robot profile"});
  }
  if (robot_io_ &&
      robot_io_->damiao_count() != robot_profile_.damiao_motors.size()) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument,
         "robot I/O Damiao count does not match robot profile"});
  }
  open_ = true;
  return Result<void>::success();
}

void RuntimeHost::close() noexcept {
  open_ = false;
  if (robot_io_) {
    robot_io_->close();
  }
}

Result<rpc::MessageEnvelope> RuntimeHost::handle(
    const rpc::MessageEnvelope& request) {
  if (!open_) {
    return Result<rpc::MessageEnvelope>::failure(
        {ErrorCode::unavailable, "runtime host is not open"});
  }
  try {
    rpc::MessageEnvelope response;
    response.request_id = request.request_id;
    response.session_id = request.session_id;
    response.step_id = request.step_id;
    response.timestamp_ns = realtime_now_ns();

    if (request.type == "health_request" || request.type == "reset_request" ||
        request.type == "observation_request") {
      auto valid_payload = rpc::validate_payload(request.type, request.payload);
      if (!valid_payload.has_value()) {
        return Result<rpc::MessageEnvelope>::success(error_envelope(
            request, "runtime_error", valid_payload.error().message));
      }
    }

    if (request.type == "health_request") {
      response.type = "health_response";
      response.payload = {{"ok", true}, {"policy_id", profile_.id}};
      return Result<rpc::MessageEnvelope>::success(std::move(response));
    }
    if (request.type == "server_info_request") {
      response.type = "server_info_response";
      response.payload =
          {{"server_name", "policy_embodied_runtime"},
           {"server_version", "0.1.0"},
           {"supported_schema", rpc::kSchemaVersion},
           {"transports", {"zmq+json"}}};
      return Result<rpc::MessageEnvelope>::success(std::move(response));
    }
    if (request.type == "reset_request") {
      const bool hard = request.payload.value("hard", false);
      sessions_[request.session_id] =
          PolicySession{request.session_id, 0, nlohmann::json::object()};
      policy_->reset(request.session_id);
      response.type = "reset_response";
      response.payload = {{"ok", true}, {"hard", hard}};
      return Result<rpc::MessageEnvelope>::success(std::move(response));
    }
    if (request.type != "observation_request") {
      return Result<rpc::MessageEnvelope>::success(error_envelope(
          request, "runtime_error", "unknown request type: " + request.type));
    }

    auto request_observation =
        normalized_observation(request.payload.at("observation"));
    auto sensor_fields = request_observation;
    if (robot_io_ && !robot_profile_.axes.empty()) {
      auto feedback = robot_io_->read_feedback();
      if (!feedback.has_value()) {
        return Result<rpc::MessageEnvelope>::success(error_envelope(
            request, "runtime_error", feedback.error().message.empty()
                                                  ? "robot feedback is unavailable"
                                                  : feedback.error().message));
      }
      if (feedback.value().axis_count != robot_profile_.axes.size()) {
        return Result<rpc::MessageEnvelope>::success(error_envelope(
            request, "runtime_error",
            "robot feedback axis count does not match robot profile"));
      }
      for (std::size_t axis = 0; axis < robot_profile_.axes.size(); ++axis) {
        sensor_fields[robot_profile_.axes[axis].name] =
            feedback_value(robot_profile_.axes[axis],
                           feedback.value().axes[axis]);
      }
    }
    if (robot_io_ && !robot_profile_.st3215_servos.empty()) {
      auto feedback = robot_io_->read_servo_feedback();
      if (!feedback.has_value()) {
        return Result<rpc::MessageEnvelope>::success(error_envelope(
            request, "runtime_error", feedback.error().message.empty()
                                          ? "robot servo feedback is unavailable"
                                          : feedback.error().message));
      }
      if (feedback.value().axis_count != robot_profile_.st3215_servos.size()) {
        return Result<rpc::MessageEnvelope>::success(error_envelope(
            request, "runtime_error",
            "robot feedback servo count does not match robot profile"));
      }
      for (std::size_t index = 0; index < robot_profile_.st3215_servos.size();
           ++index) {
        const auto& profile = robot_profile_.st3215_servos[index];
        const auto& value = feedback.value().axes[index];
        sensor_fields[profile.sensor_name] = {
            {"servo_id", profile.servo_id},
            {"position_rad", value.position_rad},
            {"raw_position", value.raw_position},
            {"status_error", value.status_error},
        };
      }
    }
    if (robot_io_ && !robot_profile_.damiao_motors.empty()) {
      auto feedback = robot_io_->read_damiao_feedback();
      if (!feedback.has_value()) {
        return Result<rpc::MessageEnvelope>::success(error_envelope(
            request, "runtime_error", "Damiao feedback is unavailable"));
      }
      for (std::size_t index = 0; index < robot_profile_.damiao_motors.size();
           ++index) {
        const auto& motor = robot_profile_.damiao_motors[index];
        const auto& value = feedback.value().axes[index];
        sensor_fields[motor.sensor_name] = {
            {"motor_id", value.motor_id}, {"position", value.position},
            {"velocity", value.velocity}, {"torque", value.torque},
            {"error_code", value.error_code}};
      }
    }

    nlohmann::json observation = nlohmann::json::object();
    for (const auto& binding : profile_.inputs) {
      if (const auto field = sensor_fields.find(binding.robot_data);
          field != sensor_fields.end()) {
        observation[binding.canonical_field] = *field;
      } else if (const auto canonical =
                     sensor_fields.find(binding.canonical_field);
                 canonical != sensor_fields.end()) {
        observation[binding.canonical_field] = *canonical;
      }
    }
    if (observation.empty()) {
      observation = std::move(request_observation);
    }
    std::vector<std::string> missing;
    for (const auto& field : profile_.canonical_observation_schema) {
      if (!field.optional && !observation.contains(field.name)) {
        missing.push_back(field.name);
      }
    }
    if (!missing.empty()) {
      return Result<rpc::MessageEnvelope>::success(error_envelope(
          request, "runtime_error", missing_fields_message(missing)));
    }
    auto processed_observation = preprocess_.run(observation);
    if (!processed_observation.has_value()) {
      return Result<rpc::MessageEnvelope>::success(error_envelope(
          request, "runtime_error", processed_observation.error().message));
    }
    auto [session, inserted] = sessions_.try_emplace(
        request.session_id,
        PolicySession{request.session_id, 0, nlohmann::json::object()});
    (void)inserted;
    auto action = policy_->infer(processed_observation.value(), session->second);
    if (!action.has_value()) {
      return Result<rpc::MessageEnvelope>::success(error_envelope(
          request, "runtime_error", action.error().message));
    }
    auto processed_action = postprocess_.run(action.value());
    if (!processed_action.has_value()) {
      return Result<rpc::MessageEnvelope>::success(error_envelope(
          request, "runtime_error", processed_action.error().message));
    }
    auto output = normalized_action(processed_action.value());
    auto valid_action = rpc::validate_action_payload(output);
    if (!valid_action.has_value()) {
      return Result<rpc::MessageEnvelope>::success(error_envelope(
          request, "runtime_error", valid_action.error().message));
    }

    if (robot_io_) {
      const auto sequence = command_sequence_ + 1;
      auto timestamp_ns = monotonic_now_ns();
      if (timestamp_ns <= last_command_timestamp_ns_) {
        timestamp_ns = last_command_timestamp_ns_ + 1;
      }
      std::array<AxisCommand, kRobotIoMaximumAxes> commands{};
      for (std::size_t axis = 0; axis < robot_profile_.axes.size(); ++axis) {
        const auto& actuator_name = axis_actuator_names_[axis];
        std::string action_field = actuator_name;
        for (const auto& binding : profile_.outputs) {
          if (binding.robot_data == actuator_name) {
            action_field = binding.canonical_field;
            break;
          }
        }
        const auto value = output.find(action_field);
        const auto target =
            value == output.end()
                ? std::optional<double>{}
                : action_target(*value, robot_profile_.axes[axis].mode);
        if (!target.has_value()) {
          return Result<rpc::MessageEnvelope>::success(error_envelope(
              request, "runtime_error",
              "action does not contain a numeric target for robot field: " +
                  actuator_name));
        }
        commands[axis].sequence = sequence;
        commands[axis].timestamp_ns = timestamp_ns;
        commands[axis].target = *target;
        commands[axis].flags = kAxisCommandEnable;
      }
      std::array<St3215ServoCommand, kRobotIoMaximumServos> servo_commands{};
      for (std::size_t index = 0; index < robot_profile_.st3215_servos.size();
           ++index) {
        const auto& servo = robot_profile_.st3215_servos[index];
        std::string action_field = servo.actuator_name;
        for (const auto& binding : profile_.outputs) {
          if (binding.robot_data == servo.actuator_name) {
            action_field = binding.canonical_field;
            break;
          }
        }
        const auto value = output.find(action_field);
        std::optional<double> target;
        if (value != output.end()) {
          if (value->is_number()) {
            target = value->get<double>();
          } else if (value->is_object()) {
            const auto position = value->find("position_rad");
            if (position != value->end() && position->is_number()) {
              target = position->get<double>();
            } else {
              const auto nested = value->find("target_position_rad");
              if (nested != value->end() && nested->is_number()) {
                target = nested->get<double>();
              }
            }
          }
        }
        if (!target.has_value()) {
          return Result<rpc::MessageEnvelope>::success(error_envelope(
              request, "runtime_error",
              "action does not contain a numeric target for robot field: " +
                  servo.actuator_name));
        }
        auto& command = servo_commands[index];
        command.sequence = sequence;
        command.timestamp_ns = timestamp_ns;
        command.target_position_rad = *target;
        command.enabled = true;
      }
      const auto axis_span = std::span<const AxisCommand>(
          commands.data(), robot_profile_.axes.size());
      const auto servo_span = std::span<const St3215ServoCommand>(
          servo_commands.data(), robot_profile_.st3215_servos.size());
      auto published = robot_io_->publish_commands(
          axis_span, servo_span, sequence, timestamp_ns);
      if (!published.has_value()) {
        return Result<rpc::MessageEnvelope>::success(error_envelope(
            request, "runtime_error", published.error().message.empty()
                                          ? "robot command publication failed"
                                          : published.error().message));
      }
      std::array<DamiaoMitCommand, kRobotIoMaximumServos> damiao_commands{};
      std::array<bool, kRobotIoMaximumServos> damiao_enabled{};
      for (std::size_t index = 0; index < robot_profile_.damiao_motors.size();
           ++index) {
        const auto& motor = robot_profile_.damiao_motors[index];
        std::string action_field = motor.actuator_name;
        for (const auto& binding : profile_.outputs) {
          if (binding.robot_data == motor.actuator_name) {
            action_field = binding.canonical_field;
            break;
          }
        }
        const auto value = output.find(action_field);
        if (value != output.end() && value->is_object()) {
          auto number = [&](const char* key, float fallback) {
            const auto field = value->find(key);
            return field != value->end() && field->is_number()
                       ? field->get<float>()
                       : fallback;
          };
          damiao_commands[index] = {
              number("position", 0.0F), number("velocity", 0.0F),
              number("kp", 0.0F), number("kd", 0.0F),
              number("torque", 0.0F)};
          const auto enabled = value->find("enabled");
          damiao_enabled[index] =
              enabled == value->end() || !enabled->is_boolean() ||
              enabled->get<bool>();
        }
      }
      published = robot_io_->publish_damiao_commands(
          std::span<const DamiaoMitCommand>(damiao_commands.data(),
                                            robot_profile_.damiao_motors.size()),
          std::span<const bool>(damiao_enabled.data(),
                                robot_profile_.damiao_motors.size()),
          sequence, timestamp_ns);
      if (!published.has_value()) {
        return Result<rpc::MessageEnvelope>::success(error_envelope(
            request, "runtime_error", published.error().message));
      }
      command_sequence_ = sequence;
      last_command_timestamp_ns_ = timestamp_ns;
    }
    ++session->second.step_id;

    response.type = "action_response";
    response.payload = {{"ok", true}, {"action", std::move(output)}};
    return Result<rpc::MessageEnvelope>::success(std::move(response));
  } catch (const std::exception& error) {
    return Result<rpc::MessageEnvelope>::success(
        error_envelope(request, "internal_error", error.what()));
  }
}

rpc::MessageEnvelope RuntimeHost::error_envelope(
    const rpc::MessageEnvelope& request, std::string code,
    std::string message) const {
  rpc::MessageEnvelope response;
  response.type = request.type + "_error";
  response.request_id = request.request_id;
  response.session_id = request.session_id;
  response.step_id = request.step_id;
  response.timestamp_ns = realtime_now_ns();
  response.error = rpc::ErrorPayload{std::move(code), std::move(message),
                                     nlohmann::json::object()};
  return response;
}

std::string RuntimeHost::handle_text(std::string_view request_text) {
  auto decoded = rpc::decode_envelope_for_dispatch(request_text);
  if (!decoded.has_value()) {
    rpc::MessageEnvelope request;
    request.type = "protocol";
    request.request_id = "unknown";
    request.session_id = "unknown";
    return rpc::encode_envelope(
        error_envelope(request, "protocol_error", decoded.error().message));
  }
  auto response = handle(decoded.value());
  if (!response.has_value()) {
    return rpc::encode_envelope(error_envelope(
        decoded.value(), "internal_error", response.error().message));
  }
  return rpc::encode_envelope(response.value());
}

}  // namespace policy_runtime
