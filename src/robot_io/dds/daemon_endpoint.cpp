#include "policy_runtime/robot_io/dds/daemon_endpoint.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <thread>
#include <variant>
#include <vector>
#include <utility>

#include <dds/dds.hpp>

#include "policy_runtime/robot_io/dds/topic_registry.hpp"
#include "policy_runtime/robot_io/dds/realtime_mailbox.hpp"

namespace policy_runtime::robot_io::dds {

namespace {

Result<std::string> load_cyclone_config(const std::string& configured_path) {
  if (configured_path.empty()) {
    return Result<std::string>::success({});
  }
  std::filesystem::path path(configured_path);
  if (!path.is_absolute()) {
    auto candidate = std::filesystem::current_path();
    for (unsigned depth = 0U; depth < 6U; ++depth) {
      const auto resolved = candidate / configured_path;
      if (std::filesystem::exists(resolved)) {
        path = resolved;
        break;
      }
      if (candidate == candidate.root_path()) break;
      candidate = candidate.parent_path();
    }
  }
  if (!std::filesystem::exists(path)) {
    return Result<std::string>::failure(
        {ErrorCode::unavailable,
         "CycloneDDS configuration file does not exist: " +
             configured_path});
  }
  std::ifstream input(path);
  const std::string xml((std::istreambuf_iterator<char>(input)),
                        std::istreambuf_iterator<char>());
  if (xml.empty()) {
    return Result<std::string>::failure(
        {ErrorCode::io,
         "CycloneDDS configuration file is empty: " + path.string()});
  }
  return Result<std::string>::success(xml);
}

}  // namespace

template <typename Event>
using EventQueue = std::variant<SensorEventQueue<Event, 16U>,
                                SensorEventQueue<Event, 32U>,
                                SensorEventQueue<Event, 64U>,
                                SensorEventQueue<Event, 128U>,
                                SensorEventQueue<Event, 256U>>;

template <typename Event>
void select_queue_capacity(EventQueue<Event>& queue, std::size_t capacity) {
  if (capacity == 32U) {
    queue.template emplace<1>();
  } else if (capacity == 64U) {
    queue.template emplace<2>();
  } else if (capacity == 128U) {
    queue.template emplace<3>();
  } else if (capacity == 256U) {
    queue.template emplace<4>();
  }
}

struct DdsDaemonEndpoint::Impl {
  struct Cia402Channel {
    Cia402Channel(::dds::pub::Publisher& publisher,
                  ::dds::domain::DomainParticipant& participant,
                  const DdsTopicDescriptor& descriptor,
                  const ::dds::pub::qos::DataWriterQos& qos,
                  std::size_t capacity)
        : device_id(descriptor.device_id),
          writer(publisher,
                 ::dds::topic::Topic<
                     ::policy_runtime::robot_io::Cia402SensorState>(
                     participant, descriptor.topic_name),
                 qos) {
      select_queue_capacity(queue, capacity);
    }

    std::string device_id;
    EventQueue<Cia402SensorEvent> queue;
    ::dds::pub::DataWriter<::policy_runtime::robot_io::Cia402SensorState>
        writer;
  };

  struct DamiaoChannel {
    DamiaoChannel(::dds::pub::Publisher& publisher,
                  ::dds::domain::DomainParticipant& participant,
                  const DdsTopicDescriptor& descriptor,
                  const ::dds::pub::qos::DataWriterQos& qos,
                  std::size_t capacity)
        : device_id(descriptor.device_id),
          writer(publisher,
                 ::dds::topic::Topic<
                     ::policy_runtime::robot_io::DamiaoSensorState>(
                     participant, descriptor.topic_name),
                 qos) {
      select_queue_capacity(queue, capacity);
    }
    std::string device_id;
    EventQueue<DamiaoSensorEvent> queue;
    ::dds::pub::DataWriter<::policy_runtime::robot_io::DamiaoSensorState>
        writer;
  };

  struct St3215Channel {
    St3215Channel(::dds::pub::Publisher& publisher,
                  ::dds::domain::DomainParticipant& participant,
                  const DdsTopicDescriptor& descriptor,
                  const ::dds::pub::qos::DataWriterQos& qos,
                  std::size_t capacity)
        : device_id(descriptor.device_id),
          writer(publisher,
                 ::dds::topic::Topic<
                     ::policy_runtime::robot_io::St3215SensorState>(
                     participant, descriptor.topic_name),
                 qos) {
      select_queue_capacity(queue, capacity);
    }
    std::string device_id;
    EventQueue<St3215SensorEvent> queue;
    ::dds::pub::DataWriter<::policy_runtime::robot_io::St3215SensorState>
        writer;
  };

  struct HealthChannel {
    HealthChannel(::dds::pub::Publisher& publisher,
                  ::dds::domain::DomainParticipant& participant,
                  const DdsTopicDescriptor& descriptor,
                  const ::dds::pub::qos::DataWriterQos& qos)
        : writer(publisher,
                 ::dds::topic::Topic<
                     ::policy_runtime::robot_io::RobotIoHealth>(
                     participant, descriptor.topic_name),
                 qos) {}
    ::dds::pub::DataWriter<::policy_runtime::robot_io::RobotIoHealth> writer;
  };

  struct EthercatGroup {
    std::string name;
    std::uint64_t epoch{};
    std::vector<Cia402CommandValue> commands;
    std::vector<bool> staged;
  };

  struct Cia402CommandChannel {
    std::size_t group_index{};
    std::size_t member_index{};
    std::string device_id;
    ::dds::sub::DataReader<::policy_runtime::robot_io::Cia402ActuatorCommand>
        reader;
  };

  struct CommitChannel {
    std::size_t group_index{};
    ::dds::sub::DataReader<::policy_runtime::robot_io::EthercatCommandCommit>
        reader;
  };

  struct DamiaoCommandChannel {
    std::size_t actuator_index{};
    std::string device_id;
    ::dds::sub::DataReader<::policy_runtime::robot_io::DamiaoActuatorCommand>
        reader;
  };

  struct St3215CommandChannel {
    std::size_t actuator_index{};
    std::string device_id;
    ::dds::sub::DataReader<::policy_runtime::robot_io::St3215ActuatorCommand>
        reader;
  };

  Impl(std::uint32_t domain_id,
       std::string robot_id_value, std::size_t queue_capacity,
       const std::vector<DdsTopicDescriptor>& descriptors,
       DdsDaemonCallbacks callback_set)
      : participant(domain_id), publisher(participant), subscriber(participant),
        robot_id(std::move(robot_id_value)), callbacks(std::move(callback_set)) {
    auto qos = publisher.default_datawriter_qos();
    qos << ::dds::core::policy::Reliability::BestEffort()
        << ::dds::core::policy::History::KeepLast(1)
        << ::dds::core::policy::Durability::Volatile();
    for (const auto& descriptor : descriptors) {
      if (descriptor.kind == DdsTopicKind::state &&
          descriptor.device_kind == DdsDeviceKind::cia402) {
        cia402_channels.push_back(std::make_unique<Cia402Channel>(
            publisher, participant, descriptor, qos, queue_capacity));
      } else if (descriptor.kind == DdsTopicKind::state &&
                 descriptor.device_kind == DdsDeviceKind::damiao) {
        damiao_channels.push_back(std::make_unique<DamiaoChannel>(
            publisher, participant, descriptor, qos, queue_capacity));
      } else if (descriptor.kind == DdsTopicKind::state &&
                 descriptor.device_kind == DdsDeviceKind::st3215) {
        st3215_channels.push_back(std::make_unique<St3215Channel>(
            publisher, participant, descriptor, qos, queue_capacity));
      } else if (descriptor.kind == DdsTopicKind::health) {
        health_channel = std::make_unique<HealthChannel>(
            publisher, participant, descriptor, qos);
      }
    }
    select_queue_capacity(health_queue, queue_capacity);
    auto command_qos = subscriber.default_datareader_qos();
    command_qos << ::dds::core::policy::Reliability::Reliable()
                << ::dds::core::policy::History::KeepLast(1)
                << ::dds::core::policy::Durability::Volatile();
    std::map<std::string, std::size_t> group_indices;
    for (const auto& descriptor : descriptors) {
      if (descriptor.kind != DdsTopicKind::command ||
          descriptor.device_kind != DdsDeviceKind::cia402) continue;
      auto [entry, inserted] = group_indices.emplace(
          descriptor.execution_group, ethercat_groups.size());
      if (inserted) {
        EthercatGroup group;
        group.name = descriptor.execution_group;
        ethercat_groups.push_back(std::move(group));
      }
      auto& group = ethercat_groups[entry->second];
      const auto member = group.commands.size();
      group.commands.emplace_back();
      group.staged.push_back(false);
      ::dds::topic::Topic<::policy_runtime::robot_io::Cia402ActuatorCommand>
          topic(participant, descriptor.topic_name);
      cia402_command_channels.push_back(std::make_unique<Cia402CommandChannel>(
          Cia402CommandChannel{entry->second, member, descriptor.device_id,
                               {subscriber, topic, command_qos}}));
    }
    std::size_t damiao_index{};
    std::size_t st3215_index{};
    for (const auto& descriptor : descriptors) {
      if (descriptor.kind != DdsTopicKind::command) {
        continue;
      }
      if (descriptor.device_kind == DdsDeviceKind::damiao) {
        ::dds::topic::Topic<::policy_runtime::robot_io::DamiaoActuatorCommand>
            topic(participant, descriptor.topic_name);
        damiao_command_channels.push_back(
            std::make_unique<DamiaoCommandChannel>(DamiaoCommandChannel{
                damiao_index++, descriptor.device_id,
                {subscriber, topic, command_qos}}));
      } else if (descriptor.device_kind == DdsDeviceKind::st3215) {
        ::dds::topic::Topic<::policy_runtime::robot_io::St3215ActuatorCommand>
            topic(participant, descriptor.topic_name);
        st3215_command_channels.push_back(
            std::make_unique<St3215CommandChannel>(St3215CommandChannel{
                st3215_index++, descriptor.device_id,
                {subscriber, topic, command_qos}}));
      }
    }
    for (const auto& descriptor : descriptors) {
      if (descriptor.kind != DdsTopicKind::commit) continue;
      const auto group = group_indices.find(descriptor.execution_group);
      if (group == group_indices.end()) continue;
      ::dds::topic::Topic<::policy_runtime::robot_io::EthercatCommandCommit>
          topic(participant, descriptor.topic_name);
      commit_channels.push_back(std::make_unique<CommitChannel>(
          CommitChannel{group->second, {subscriber, topic, command_qos}}));
    }
  }

  ::dds::domain::DomainParticipant participant;
  ::dds::pub::Publisher publisher;
  ::dds::sub::Subscriber subscriber;
  std::string robot_id;
  std::vector<std::unique_ptr<Cia402Channel>> cia402_channels;
  std::vector<std::unique_ptr<DamiaoChannel>> damiao_channels;
  std::vector<std::unique_ptr<St3215Channel>> st3215_channels;
  std::vector<EthercatGroup> ethercat_groups;
  std::vector<std::unique_ptr<Cia402CommandChannel>>
      cia402_command_channels;
  std::vector<std::unique_ptr<CommitChannel>> commit_channels;
  std::vector<std::unique_ptr<DamiaoCommandChannel>> damiao_command_channels;
  std::vector<std::unique_ptr<St3215CommandChannel>> st3215_command_channels;
  EventQueue<RobotIoHealthEvent> health_queue;
  std::unique_ptr<HealthChannel> health_channel;
  DdsDaemonCallbacks callbacks;
  std::atomic<bool> running{};
  std::atomic<bool> worker_failed{};
  std::jthread worker;
};

DdsDaemonEndpoint::DdsDaemonEndpoint(
    std::unique_ptr<Impl> implementation) noexcept
    : implementation_(std::move(implementation)) {}

DdsDaemonEndpoint::DdsDaemonEndpoint(DdsDaemonEndpoint&&) noexcept = default;
DdsDaemonEndpoint& DdsDaemonEndpoint::operator=(DdsDaemonEndpoint&&) noexcept =
    default;
DdsDaemonEndpoint::~DdsDaemonEndpoint() = default;

Result<DdsDaemonEndpoint> DdsDaemonEndpoint::create(
    const profiles::RobotProfile& profile, DdsDaemonCallbacks callbacks) {
  if (profile.dds.backend != profiles::RobotIoBackend::dds) {
    return Result<DdsDaemonEndpoint>::failure(
        {ErrorCode::invalid_argument, "DDS daemon endpoint requires DDS backend"});
  }
  try {
    auto configuration = load_cyclone_config(profile.dds.cyclone_config);
    if (!configuration.has_value()) {
      return Result<DdsDaemonEndpoint>::failure(configuration.error());
    }
    dds_entity_t configured_domain = DDS_CYCLONEDDS_HANDLE;
    bool owns_configured_domain = false;
    if (!configuration.value().empty()) {
      configured_domain = dds_create_domain(
          profile.dds.local_domain_id, configuration.value().c_str());
      if (configured_domain > 0) {
        owns_configured_domain = true;
      } else if (configured_domain != DDS_RETCODE_PRECONDITION_NOT_MET) {
        return Result<DdsDaemonEndpoint>::failure(
            {ErrorCode::io, "failed to create configured CycloneDDS domain"});
      }
    }
    auto registry = build_dds_topic_registry(profile, profile.dds.robot_id);
    if (!registry.has_value()) {
      return Result<DdsDaemonEndpoint>::failure(registry.error());
    }
    try {
      return Result<DdsDaemonEndpoint>::success(DdsDaemonEndpoint(
          std::make_unique<Impl>(profile.dds.local_domain_id,
                                 profile.dds.robot_id,
                                 profile.dds.sensor_queue_capacity,
                                 registry.value(), std::move(callbacks))));
    } catch (...) {
      if (owns_configured_domain) {
        static_cast<void>(dds_delete(configured_domain));
      }
      throw;
    }
  } catch (const ::dds::core::Exception& error) {
    return Result<DdsDaemonEndpoint>::failure(
        {ErrorCode::io, std::string("failed to create DDS participant: ") +
                            error.what()});
  }
}

Result<void> DdsDaemonEndpoint::start() {
  if (!implementation_ || implementation_->running) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "DDS daemon endpoint is not startable"});
  }
  implementation_->running.store(true, std::memory_order_release);
  auto* implementation = implementation_.get();
  implementation_->worker = std::jthread([implementation](std::stop_token stop) {
    try {
      while (!stop.stop_requested()) {
        bool consumed = false;
        for (auto& channel : implementation->cia402_command_channels) {
          for (const auto& sample : channel->reader.take()) {
            if (!sample.info().valid()) continue;
            const auto& value = sample.data();
            const auto& header = value.header();
            if (header.schema_version() !=
                    ::policy_runtime::robot_io::DDS_SCHEMA_VERSION ||
                header.robot_id() != implementation->robot_id ||
                header.device_id() != channel->device_id ||
                value.command_epoch() == 0U || !std::isfinite(value.target()))
              continue;
            auto& group = implementation->ethercat_groups[channel->group_index];
            if (group.epoch != value.command_epoch()) {
              group.epoch = value.command_epoch();
              std::fill(group.staged.begin(), group.staged.end(), false);
            }
            group.commands[channel->member_index] = {
                value.target(), value.mode(), value.command_flags(),
                header.sequence_number(), header.source_timestamp_ns()};
            group.staged[channel->member_index] = true;
            consumed = true;
          }
        }
        for (auto& channel : implementation->commit_channels) {
          for (const auto& sample : channel->reader.take()) {
            if (!sample.info().valid()) continue;
            const auto& value = sample.data();
            const auto& header = value.header();
            auto& group = implementation->ethercat_groups[channel->group_index];
            if (header.schema_version() !=
                    ::policy_runtime::robot_io::DDS_SCHEMA_VERSION ||
                header.robot_id() != implementation->robot_id ||
                value.execution_group() != group.name ||
                value.command_epoch() == 0U ||
                value.command_epoch() != group.epoch ||
                !std::all_of(group.staged.begin(), group.staged.end(),
                             [](bool staged) { return staged; }))
              continue;
            if (implementation->callbacks.stage_ethercat_epoch) {
              implementation->callbacks.stage_ethercat_epoch(
                  group.name, group.epoch, group.commands);
            }
            std::fill(group.staged.begin(), group.staged.end(), false);
            consumed = true;
          }
        }
        for (auto& channel : implementation->damiao_command_channels) {
          for (const auto& sample : channel->reader.take()) {
            if (!sample.info().valid()) {
              continue;
            }
            const auto& value = sample.data();
            const auto& header = value.header();
            if (header.schema_version() !=
                    ::policy_runtime::robot_io::DDS_SCHEMA_VERSION ||
                header.robot_id() != implementation->robot_id ||
                header.device_id() != channel->device_id ||
                !std::isfinite(value.position()) ||
                !std::isfinite(value.velocity()) || !std::isfinite(value.kp()) ||
                !std::isfinite(value.kd()) || !std::isfinite(value.torque())) {
              continue;
            }
            if (implementation->callbacks.stage_damiao_command) {
              implementation->callbacks.stage_damiao_command(
                  channel->actuator_index,
                  {value.enabled(), value.fault_reset(), value.position(),
                   value.velocity(), value.kp(), value.kd(), value.torque(),
                   header.sequence_number(),
                   header.source_timestamp_ns()});
            }
            consumed = true;
          }
        }
        for (auto& channel : implementation->st3215_command_channels) {
          for (const auto& sample : channel->reader.take()) {
            if (!sample.info().valid()) {
              continue;
            }
            const auto& value = sample.data();
            const auto& header = value.header();
            if (header.schema_version() !=
                    ::policy_runtime::robot_io::DDS_SCHEMA_VERSION ||
                header.robot_id() != implementation->robot_id ||
                header.device_id() != channel->device_id ||
                !std::isfinite(value.target_radians())) {
              continue;
            }
            if (implementation->callbacks.stage_st3215_command) {
              implementation->callbacks.stage_st3215_command(
                  channel->actuator_index,
                  {value.target_radians(), value.enabled(), value.fault_reset(),
                   header.sequence_number(), header.source_timestamp_ns()});
            }
            consumed = true;
          }
        }
        for (auto& channel : implementation->cia402_channels) {
          auto event = std::visit([](auto& queue) { return queue.pop(); },
                                  channel->queue);
          if (!event.has_value()) continue;
          consumed = true;
          ::policy_runtime::robot_io::Cia402SensorState state;
          state.header().schema_version(
              ::policy_runtime::robot_io::DDS_SCHEMA_VERSION);
          state.header().robot_id(implementation->robot_id);
          state.header().device_id(channel->device_id);
          state.header().sequence_number(event->sequence);
          state.header().source_timestamp_ns(event->source_timestamp_ns);
          state.header().bus_cycle(event->bus_cycle);
          state.header().flags(event->flags);
          state.position(event->position);
          state.velocity(event->velocity);
          state.torque(event->torque);
          state.status_word(event->status_word);
          state.mode_display(event->mode_display);
          channel->writer.write(state);
        }
        for (auto& channel : implementation->damiao_channels) {
          auto event = std::visit([](auto& queue) { return queue.pop(); },
                                  channel->queue);
          if (!event.has_value()) {
            continue;
          }
          consumed = true;
          ::policy_runtime::robot_io::DamiaoSensorState state;
          state.header().schema_version(
              ::policy_runtime::robot_io::DDS_SCHEMA_VERSION);
          state.header().robot_id(implementation->robot_id);
          state.header().device_id(channel->device_id);
          state.header().sequence_number(event->sequence);
          state.header().source_timestamp_ns(event->source_timestamp_ns);
          state.header().bus_cycle(event->bus_cycle);
          state.header().flags(event->flags);
          state.position(event->position);
          state.velocity(event->velocity);
          state.torque(event->torque);
          state.motor_id(event->motor_id);
          state.error_code(event->error_code);
          state.mos_temperature_c(event->mos_temperature_c);
          state.rotor_temperature_c(event->rotor_temperature_c);
          channel->writer.write(state);
        }
        for (auto& channel : implementation->st3215_channels) {
          auto event = std::visit([](auto& queue) { return queue.pop(); },
                                  channel->queue);
          if (!event.has_value()) {
            continue;
          }
          consumed = true;
          ::policy_runtime::robot_io::St3215SensorState state;
          state.header().schema_version(
              ::policy_runtime::robot_io::DDS_SCHEMA_VERSION);
          state.header().robot_id(implementation->robot_id);
          state.header().device_id(channel->device_id);
          state.header().sequence_number(event->sequence);
          state.header().source_timestamp_ns(event->source_timestamp_ns);
          state.header().bus_cycle(event->bus_cycle);
          state.header().flags(event->flags);
          state.position_radians(event->position_radians);
          state.position_units(event->position_units);
          state.error_flags(event->error_flags);
          channel->writer.write(state);
        }
        auto health = std::visit([](auto& queue) { return queue.pop(); },
                                 implementation->health_queue);
        if (health.has_value() && implementation->health_channel) {
          consumed = true;
          ::policy_runtime::robot_io::RobotIoHealth state;
          state.header().schema_version(
              ::policy_runtime::robot_io::DDS_SCHEMA_VERSION);
          state.header().robot_id(implementation->robot_id);
          state.header().device_id("health");
          state.header().sequence_number(health->sequence);
          state.header().source_timestamp_ns(health->source_timestamp_ns);
          state.health_flags(health->health_flags);
          state.dropped_samples(health->dropped_samples);
          state.diagnostic("");
          implementation->health_channel->writer.write(state);
        }
        if (!consumed)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (...) {
      implementation->worker_failed.store(true, std::memory_order_release);
    }
  });
  return Result<void>::success();
}

void DdsDaemonEndpoint::stop() noexcept {
  if (implementation_) {
    implementation_->worker.request_stop();
    if (implementation_->worker.joinable()) implementation_->worker.join();
    implementation_->running.store(false, std::memory_order_release);
  }
}

Result<void> DdsDaemonEndpoint::poll_health() {
  if (!implementation_) {
    return Result<void>::failure(
        {ErrorCode::invalid_argument, "DDS daemon endpoint has been moved"});
  }
  if (implementation_->worker_failed.load(std::memory_order_acquire)) {
    return Result<void>::failure(
        {ErrorCode::io, "DDS daemon worker failed"});
  }
  return Result<void>::success();
}

bool DdsDaemonEndpoint::enqueue_cia402_sensor_realtime(
    std::size_t sensor_index, const Cia402SensorEvent& event) noexcept {
  if (!implementation_ || sensor_index >= implementation_->cia402_channels.size())
    return false;
  return std::visit([&event](auto& queue) {
    return queue.push_realtime(event);
  }, implementation_->cia402_channels[sensor_index]->queue);
}

bool DdsDaemonEndpoint::enqueue_damiao_sensor_realtime(
    std::size_t sensor_index, const DamiaoSensorEvent& event) noexcept {
  if (!implementation_ || sensor_index >= implementation_->damiao_channels.size()) {
    return false;
  }
  return std::visit(
      [&event](auto& queue) { return queue.push_realtime(event); },
      implementation_->damiao_channels[sensor_index]->queue);
}

bool DdsDaemonEndpoint::enqueue_st3215_sensor_realtime(
    std::size_t sensor_index, const St3215SensorEvent& event) noexcept {
  if (!implementation_ ||
      sensor_index >= implementation_->st3215_channels.size()) {
    return false;
  }
  return std::visit(
      [&event](auto& queue) { return queue.push_realtime(event); },
      implementation_->st3215_channels[sensor_index]->queue);
}

bool DdsDaemonEndpoint::enqueue_health_realtime(
    const RobotIoHealthEvent& event) noexcept {
  if (!implementation_) {
    return false;
  }
  return std::visit(
      [&event](auto& queue) { return queue.push_realtime(event); },
      implementation_->health_queue);
}

std::uint64_t DdsDaemonEndpoint::dropped_cia402_sensor_samples(
    std::size_t sensor_index) const noexcept {
  if (!implementation_ || sensor_index >= implementation_->cia402_channels.size())
    return 0U;
  return std::visit([](const auto& queue) { return queue.dropped(); },
                    implementation_->cia402_channels[sensor_index]->queue);
}

}  // namespace policy_runtime::robot_io::dds
