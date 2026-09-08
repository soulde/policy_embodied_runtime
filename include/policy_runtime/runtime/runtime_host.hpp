#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/policy/pipeline.hpp"
#include "policy_runtime/policy/policy.hpp"
#include "policy_runtime/profiles/policy_profile.hpp"
#include "policy_runtime/profiles/robot_profile.hpp"
#include "policy_runtime/protocol/rpc/messages.hpp"
#include "policy_runtime/robot_io/value_snapshot.hpp"

namespace policy_runtime {

class RuntimeRobotIo {
 public:
  virtual ~RuntimeRobotIo() = default;
  virtual std::uint32_t axis_count() const noexcept = 0;
  virtual std::uint32_t servo_count() const noexcept = 0;
  virtual std::uint32_t damiao_count() const noexcept { return 0U; }
  virtual Result<Snapshot<AxisFeedback>> read_feedback() = 0;
  virtual Result<Snapshot<St3215ServoFeedback>> read_servo_feedback() = 0;
  virtual Result<Snapshot<DamiaoFeedback>> read_damiao_feedback() {
    return Result<Snapshot<DamiaoFeedback>>::failure(
        {ErrorCode::unavailable, "Damiao feedback is unavailable"});
  }
  virtual Result<void> publish_commands(
      std::span<const AxisCommand> axis_commands,
      std::span<const St3215ServoCommand> servo_commands,
      std::uint64_t sequence, std::int64_t timestamp_ns) = 0;
  virtual Result<void> publish_damiao_commands(
      std::span<const DamiaoMitCommand>, std::span<const bool>,
      std::uint64_t, std::int64_t) {
    return Result<void>::success();
  }
  virtual void close() noexcept = 0;
};

class RuntimeHost {
 public:
  static Result<RuntimeHost> from_profiles(
      const std::filesystem::path& policy_profile,
      std::optional<std::filesystem::path> robot_profile = std::nullopt,
      std::unique_ptr<RuntimeRobotIo> robot_io = {});

  RuntimeHost(const RuntimeHost&) = delete;
  RuntimeHost& operator=(const RuntimeHost&) = delete;
  RuntimeHost(RuntimeHost&&) noexcept = default;
  RuntimeHost& operator=(RuntimeHost&&) noexcept = default;

  Result<void> open();
  void close() noexcept;
  Result<rpc::MessageEnvelope> handle(const rpc::MessageEnvelope& request);
  std::string handle_text(std::string_view request);

 private:
  RuntimeHost(profiles::PolicyProfile profile,
              profiles::RobotProfile robot_profile,
              std::vector<std::string> axis_actuator_names,
              std::unique_ptr<Policy> policy,
              ProcessorPipeline preprocess, ProcessorPipeline postprocess,
              std::unique_ptr<RuntimeRobotIo> robot_io);

  rpc::MessageEnvelope error_envelope(const rpc::MessageEnvelope& request,
                                      std::string code,
                                      std::string message) const;

  profiles::PolicyProfile profile_;
  profiles::RobotProfile robot_profile_;
  std::vector<std::string> axis_actuator_names_;
  std::unique_ptr<Policy> policy_;
  ProcessorPipeline preprocess_;
  ProcessorPipeline postprocess_;
  std::unique_ptr<RuntimeRobotIo> robot_io_;
  std::unordered_map<std::string, PolicySession> sessions_;
  std::uint64_t command_sequence_{};
  std::int64_t last_command_timestamp_ns_{};
  bool open_{};
};

}  // namespace policy_runtime
