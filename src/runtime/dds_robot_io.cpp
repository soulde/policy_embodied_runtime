#include "policy_runtime/runtime/dds_robot_io.hpp"

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <dds/dds.hpp>

#include "policy_runtime/robot_io/dds/topic_registry.hpp"
#include "policy_runtime/runtime/runtime_host.hpp"
#include "robot_io.hpp"

namespace policy_runtime {
namespace {

using robot_io::dds::DdsDeviceKind;
using robot_io::dds::DdsTopicDescriptor;
using robot_io::dds::DdsTopicKind;

template <typename T>
struct ReaderChannel {
  ReaderChannel(::dds::sub::Subscriber& subscriber,
                ::dds::domain::DomainParticipant& participant,
                const DdsTopicDescriptor& descriptor,
                const ::dds::sub::qos::DataReaderQos& qos)
      : device_id(descriptor.device_id),
        reader(subscriber, ::dds::topic::Topic<T>(participant,
                                                  descriptor.topic_name),
               qos) {}
  std::string device_id;
  ::dds::sub::DataReader<T> reader;
};

template <typename T>
struct WriterChannel {
  WriterChannel(::dds::pub::Publisher& publisher,
                ::dds::domain::DomainParticipant& participant,
                const DdsTopicDescriptor& descriptor,
                const ::dds::pub::qos::DataWriterQos& qos)
      : device_id(descriptor.device_id),
        execution_group(descriptor.execution_group),
        writer(publisher, ::dds::topic::Topic<T>(participant,
                                                 descriptor.topic_name),
               qos) {}
  std::string device_id;
  std::string execution_group;
  ::dds::pub::DataWriter<T> writer;
};

class DdsRuntimeRobotIo final : public RuntimeRobotIo {
 public:
  DdsRuntimeRobotIo(const profiles::RobotProfile& profile,
                    const std::vector<DdsTopicDescriptor>& descriptors)
      : participant_(profile.dds.local_domain_id),
        publisher_(participant_),
        subscriber_(participant_),
        robot_id_(profile.dds.robot_id),
        axis_count_(static_cast<std::uint32_t>(profile.axes.size())),
        servo_count_(static_cast<std::uint32_t>(profile.st3215_servos.size())) {
    damiao_count_ = static_cast<std::uint32_t>(profile.damiao_motors.size());
    auto reader_qos = subscriber_.default_datareader_qos();
    reader_qos << ::dds::core::policy::Reliability::BestEffort()
               << ::dds::core::policy::History::KeepLast(1)
               << ::dds::core::policy::Durability::Volatile();
    auto writer_qos = publisher_.default_datawriter_qos();
    writer_qos << ::dds::core::policy::Reliability::Reliable()
               << ::dds::core::policy::History::KeepLast(1)
               << ::dds::core::policy::Durability::Volatile();
    for (const auto& descriptor : descriptors) {
      if (descriptor.kind == DdsTopicKind::state &&
          descriptor.device_kind == DdsDeviceKind::cia402) {
        cia402_readers_.push_back(std::make_unique<ReaderChannel<
            robot_io::Cia402SensorState>>(subscriber_, participant_, descriptor,
                                         reader_qos));
      } else if (descriptor.kind == DdsTopicKind::state &&
                 descriptor.device_kind == DdsDeviceKind::st3215) {
        st3215_readers_.push_back(std::make_unique<ReaderChannel<
            robot_io::St3215SensorState>>(subscriber_, participant_, descriptor,
                                         reader_qos));
      } else if (descriptor.kind == DdsTopicKind::state &&
                 descriptor.device_kind == DdsDeviceKind::damiao) {
        damiao_readers_.push_back(std::make_unique<ReaderChannel<
            robot_io::DamiaoSensorState>>(subscriber_, participant_, descriptor,
                                         reader_qos));
      } else if (descriptor.kind == DdsTopicKind::command &&
                 descriptor.device_kind == DdsDeviceKind::cia402) {
        cia402_writers_.push_back(std::make_unique<WriterChannel<
            robot_io::Cia402ActuatorCommand>>(publisher_, participant_,
                                              descriptor, writer_qos));
      } else if (descriptor.kind == DdsTopicKind::command &&
                 descriptor.device_kind == DdsDeviceKind::st3215) {
        st3215_writers_.push_back(std::make_unique<WriterChannel<
            robot_io::St3215ActuatorCommand>>(publisher_, participant_,
                                              descriptor, writer_qos));
      } else if (descriptor.kind == DdsTopicKind::command &&
                 descriptor.device_kind == DdsDeviceKind::damiao) {
        damiao_writers_.push_back(std::make_unique<WriterChannel<
            robot_io::DamiaoActuatorCommand>>(publisher_, participant_,
                                              descriptor, writer_qos));
      } else if (descriptor.kind == DdsTopicKind::commit) {
        commit_writers_.push_back(std::make_unique<WriterChannel<
            robot_io::EthercatCommandCommit>>(publisher_, participant_,
                                              descriptor, writer_qos));
      }
    }
    axis_feedback_.axis_count = axis_count_;
    servo_feedback_.axis_count = servo_count_;
    damiao_feedback_.axis_count = damiao_count_;
  }

  std::uint32_t axis_count() const noexcept override { return axis_count_; }
  std::uint32_t servo_count() const noexcept override { return servo_count_; }
  std::uint32_t damiao_count() const noexcept override { return damiao_count_; }

  Result<Snapshot<AxisFeedback>> read_feedback() override {
    for (std::size_t index = 0; index < cia402_readers_.size(); ++index) {
      for (const auto& sample : cia402_readers_[index]->reader.take()) {
        if (!sample.info().valid() || index >= axis_feedback_.axis_count) {
          continue;
        }
        const auto& state = sample.data();
        if (!valid_header(state.header(), cia402_readers_[index]->device_id)) {
          continue;
        }
        auto& value = axis_feedback_.axes[index];
        value.sequence = state.header().sequence_number();
        value.timestamp_ns = state.header().source_timestamp_ns();
        value.position = state.position();
        value.velocity = state.velocity();
        value.effort = state.torque();
        value.status_word = state.status_word();
        value.flags = state.header().flags();
        value.mode_display = static_cast<std::uint32_t>(state.mode_display());
        axis_feedback_.sequence = std::max(axis_feedback_.sequence,
                                           value.sequence);
        axis_feedback_.timestamp_ns = std::max(axis_feedback_.timestamp_ns,
                                               value.timestamp_ns);
      }
    }
    return Result<Snapshot<AxisFeedback>>::success(axis_feedback_);
  }

  Result<Snapshot<St3215ServoFeedback>> read_servo_feedback() override {
    for (std::size_t index = 0; index < st3215_readers_.size(); ++index) {
      for (const auto& sample : st3215_readers_[index]->reader.take()) {
        if (!sample.info().valid() || index >= servo_feedback_.axis_count) {
          continue;
        }
        const auto& state = sample.data();
        if (!valid_header(state.header(), st3215_readers_[index]->device_id)) {
          continue;
        }
        auto& value = servo_feedback_.axes[index];
        value.feedback_sequence = state.header().sequence_number();
        value.timestamp_ns = state.header().source_timestamp_ns();
        value.position_rad = state.position_radians();
        value.raw_position = state.position_units();
        value.flags = state.header().flags();
        value.status_error = state.error_flags();
        servo_feedback_.sequence = std::max(servo_feedback_.sequence,
                                            value.feedback_sequence);
        servo_feedback_.timestamp_ns = std::max(servo_feedback_.timestamp_ns,
                                                value.timestamp_ns);
      }
    }
    return Result<Snapshot<St3215ServoFeedback>>::success(servo_feedback_);
  }

  Result<Snapshot<DamiaoFeedback>> read_damiao_feedback() override {
    for (std::size_t index = 0; index < damiao_readers_.size(); ++index) {
      for (const auto& sample : damiao_readers_[index]->reader.take()) {
        if (!sample.info().valid() || index >= damiao_feedback_.axis_count) {
          continue;
        }
        const auto& state = sample.data();
        if (!valid_header(state.header(), damiao_readers_[index]->device_id)) {
          continue;
        }
        damiao_feedback_.axes[index] = {
            state.motor_id(), state.error_code(), state.position(),
            state.velocity(), state.torque(),
            static_cast<std::int8_t>(state.mos_temperature_c()),
            static_cast<std::int8_t>(state.rotor_temperature_c())};
        damiao_feedback_.sequence = std::max(
            damiao_feedback_.sequence, state.header().sequence_number());
        damiao_feedback_.timestamp_ns = std::max(
            damiao_feedback_.timestamp_ns,
            state.header().source_timestamp_ns());
      }
    }
    return Result<Snapshot<DamiaoFeedback>>::success(damiao_feedback_);
  }

  Result<void> publish_commands(
      std::span<const AxisCommand> axis_commands,
      std::span<const St3215ServoCommand> servo_commands,
      std::uint64_t sequence, std::int64_t timestamp_ns) override {
    if (axis_commands.size() != cia402_writers_.size() ||
        servo_commands.size() != st3215_writers_.size()) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "DDS command count mismatch"});
    }
    try {
      for (std::size_t index = 0; index < axis_commands.size(); ++index) {
        robot_io::Cia402ActuatorCommand command;
        fill_header(command.header(), cia402_writers_[index]->device_id,
                    sequence, timestamp_ns);
        command.target(axis_commands[index].target);
        command.mode(0);
        command.command_flags(axis_commands[index].flags);
        command.command_epoch(sequence);
        cia402_writers_[index]->writer.write(command);
      }
      for (auto& channel : commit_writers_) {
        robot_io::EthercatCommandCommit commit;
        fill_header(commit.header(), channel->device_id, sequence,
                    timestamp_ns);
        commit.execution_group(channel->execution_group);
        commit.command_epoch(sequence);
        channel->writer.write(commit);
      }
      for (std::size_t index = 0; index < servo_commands.size(); ++index) {
        robot_io::St3215ActuatorCommand command;
        fill_header(command.header(), st3215_writers_[index]->device_id,
                    sequence, timestamp_ns);
        command.target_radians(servo_commands[index].target_position_rad);
        command.enabled(servo_commands[index].enabled);
        command.fault_reset(servo_commands[index].emergency_stop);
        st3215_writers_[index]->writer.write(command);
      }
      return Result<void>::success();
    } catch (const ::dds::core::Exception& error) {
      return Result<void>::failure(
          {ErrorCode::io, std::string("DDS command publish failed: ") +
                              error.what()});
    }
  }

  Result<void> publish_damiao_commands(
      std::span<const DamiaoMitCommand> commands,
      std::span<const bool> enabled, std::uint64_t sequence,
      std::int64_t timestamp_ns) override {
    if (commands.size() != damiao_writers_.size() ||
        enabled.size() != commands.size()) {
      return Result<void>::failure(
          {ErrorCode::invalid_argument, "DDS Damiao command count mismatch"});
    }
    try {
      for (std::size_t index = 0; index < commands.size(); ++index) {
        robot_io::DamiaoActuatorCommand command;
        fill_header(command.header(), damiao_writers_[index]->device_id,
                    sequence, timestamp_ns);
        command.enabled(enabled[index]);
        command.fault_reset(false);
        command.position(commands[index].position);
        command.velocity(commands[index].velocity);
        command.kp(commands[index].kp);
        command.kd(commands[index].kd);
        command.torque(commands[index].torque);
        damiao_writers_[index]->writer.write(command);
      }
      return Result<void>::success();
    } catch (const ::dds::core::Exception& error) {
      return Result<void>::failure(
          {ErrorCode::io, std::string("DDS Damiao publish failed: ") +
                              error.what()});
    }
  }

  void close() noexcept override {}

 private:
  bool valid_header(const robot_io::MessageHeader& header,
                    const std::string& device_id) const noexcept {
    return header.schema_version() == robot_io::DDS_SCHEMA_VERSION &&
           header.robot_id() == robot_id_ && header.device_id() == device_id;
  }

  void fill_header(robot_io::MessageHeader& header,
                   const std::string& device_id, std::uint64_t sequence,
                   std::int64_t timestamp_ns) {
    header.schema_version(robot_io::DDS_SCHEMA_VERSION);
    header.robot_id(robot_id_);
    header.device_id(device_id);
    header.session_id(session_id_);
    header.sequence_number(sequence);
    header.source_timestamp_ns(timestamp_ns);
  }

  ::dds::domain::DomainParticipant participant_;
  ::dds::pub::Publisher publisher_;
  ::dds::sub::Subscriber subscriber_;
  std::string robot_id_;
  std::string session_id_{"policy-runtime-host"};
  std::uint32_t axis_count_{};
  std::uint32_t servo_count_{};
  std::uint32_t damiao_count_{};
  Snapshot<AxisFeedback> axis_feedback_;
  Snapshot<St3215ServoFeedback> servo_feedback_;
  Snapshot<DamiaoFeedback> damiao_feedback_;
  std::vector<std::unique_ptr<ReaderChannel<robot_io::Cia402SensorState>>>
      cia402_readers_;
  std::vector<std::unique_ptr<ReaderChannel<robot_io::St3215SensorState>>>
      st3215_readers_;
  std::vector<std::unique_ptr<ReaderChannel<robot_io::DamiaoSensorState>>>
      damiao_readers_;
  std::vector<std::unique_ptr<WriterChannel<robot_io::Cia402ActuatorCommand>>>
      cia402_writers_;
  std::vector<std::unique_ptr<WriterChannel<robot_io::St3215ActuatorCommand>>>
      st3215_writers_;
  std::vector<std::unique_ptr<WriterChannel<robot_io::DamiaoActuatorCommand>>>
      damiao_writers_;
  std::vector<std::unique_ptr<WriterChannel<robot_io::EthercatCommandCommit>>>
      commit_writers_;
};

}  // namespace

Result<std::unique_ptr<RuntimeRobotIo>> make_dds_runtime_robot_io(
    const profiles::RobotProfile& profile) {
  if (profile.dds.backend != profiles::RobotIoBackend::dds) {
    return Result<std::unique_ptr<RuntimeRobotIo>>::failure(
        {ErrorCode::invalid_argument, "robot profile does not select DDS"});
  }
  auto descriptors = robot_io::dds::build_dds_topic_registry(
      profile, profile.dds.robot_id);
  if (!descriptors.has_value()) {
    return Result<std::unique_ptr<RuntimeRobotIo>>::failure(
        descriptors.error());
  }
  try {
    return Result<std::unique_ptr<RuntimeRobotIo>>::success(
        std::make_unique<DdsRuntimeRobotIo>(profile, descriptors.value()));
  } catch (const ::dds::core::Exception& error) {
    return Result<std::unique_ptr<RuntimeRobotIo>>::failure(
        {ErrorCode::io, std::string("failed to create DDS host endpoint: ") +
                            error.what()});
  }
}

}  // namespace policy_runtime
