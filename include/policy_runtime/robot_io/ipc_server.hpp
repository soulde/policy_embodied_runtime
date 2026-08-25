#pragma once

#include <cstdint>
#include <optional>
#include <span>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/robot_io/ipc_protocol.hpp"
#include "policy_runtime/robot_io/snapshot.hpp"

namespace policy_runtime {

class RobotIoIpcServer {
 public:
  // Consumes connected_socket on every return path. The socket must be a connected
  // AF_UNIX SOCK_SEQPACKET descriptor. The returned object owns daemon-created
  // role-specific memfd descriptions, their mappings, and the socket.
  static Result<RobotIoIpcServer> create(int connected_socket, std::uint32_t axis_count,
                                         std::uint32_t generation);
  static Result<RobotIoIpcServer> create(int connected_socket,
                                         std::uint32_t axis_count,
                                         std::uint32_t servo_count,
                                         std::uint32_t generation);

  RobotIoIpcServer(const RobotIoIpcServer&) = delete;
  RobotIoIpcServer& operator=(const RobotIoIpcServer&) = delete;
  RobotIoIpcServer(RobotIoIpcServer&& other) noexcept;
  RobotIoIpcServer& operator=(RobotIoIpcServer&& other) noexcept;
  ~RobotIoIpcServer();

  Result<void> send_setup();
  Result<Snapshot<AxisCommand>> read_commands() const;
  Result<void> publish_feedback(std::span<const AxisFeedback> axes,
                                std::uint64_t sequence,
                                std::int64_t timestamp_ns);
  Result<Snapshot<St3215ServoCommand>> read_servo_commands() const;
  Result<void> publish_servo_feedback(
      std::span<const St3215ServoFeedback> servos,
      std::uint64_t sequence, std::int64_t timestamp_ns);
  // Shared-memory-only data-plane operations for the real-time owner. Peer
  // liveness remains on check_peer() in a non-real-time control path.
  Result<Snapshot<AxisCommand>> read_commands_realtime() const noexcept;
  Result<void> publish_feedback_realtime(std::span<const AxisFeedback> axes,
                                         std::uint64_t sequence,
                                         std::int64_t timestamp_ns) noexcept;
  Result<Snapshot<St3215ServoCommand>> read_servo_commands_realtime()
      const noexcept;
  Result<void> publish_servo_feedback_realtime(
      std::span<const St3215ServoFeedback> servos,
      std::uint64_t sequence, std::int64_t timestamp_ns) noexcept;
  Result<void> check_peer() const;
  Result<void> close();

  std::uint32_t axis_count() const noexcept { return axis_count_; }
  std::uint32_t servo_count() const noexcept { return servo_count_; }
  std::uint32_t generation() const noexcept { return generation_; }

 private:
  RobotIoIpcServer(int socket_fd, int command_transfer_fd, int command_reader_fd,
                   int feedback_writer_fd, int feedback_transfer_fd,
                   void* command_mapping, std::size_t command_mapping_size,
                   void* feedback_mapping, std::size_t feedback_mapping_size,
                   int servo_command_transfer_fd,
                   int servo_command_reader_fd,
                   int servo_feedback_writer_fd,
                   int servo_feedback_transfer_fd,
                   void* servo_command_mapping,
                   std::size_t servo_command_mapping_size,
                   void* servo_feedback_mapping,
                   std::size_t servo_feedback_mapping_size,
                   std::uint32_t axis_count, std::uint32_t generation,
                   SnapshotReader<AxisCommand> command_reader,
                   SnapshotWriter<AxisFeedback> feedback_writer,
                   std::uint32_t servo_count,
                   std::optional<SnapshotReader<St3215ServoCommand>>
                       servo_command_reader,
                   std::optional<SnapshotWriter<St3215ServoFeedback>>
                       servo_feedback_writer) noexcept;

  void release_noexcept() noexcept;

  int socket_fd_{-1};
  int command_transfer_fd_{-1};
  int command_reader_fd_{-1};
  int feedback_writer_fd_{-1};
  int feedback_transfer_fd_{-1};
  int servo_command_transfer_fd_{-1};
  int servo_command_reader_fd_{-1};
  int servo_feedback_writer_fd_{-1};
  int servo_feedback_transfer_fd_{-1};
  void* command_mapping_{};
  std::size_t command_mapping_size_{};
  void* feedback_mapping_{};
  std::size_t feedback_mapping_size_{};
  void* servo_command_mapping_{};
  std::size_t servo_command_mapping_size_{};
  void* servo_feedback_mapping_{};
  std::size_t servo_feedback_mapping_size_{};
  std::uint32_t axis_count_{};
  std::uint32_t servo_count_{};
  std::uint32_t generation_{};
  std::optional<SnapshotReader<AxisCommand>> command_reader_;
  std::optional<SnapshotWriter<AxisFeedback>> feedback_writer_;
  std::optional<SnapshotReader<St3215ServoCommand>> servo_command_reader_;
  std::optional<SnapshotWriter<St3215ServoFeedback>> servo_feedback_writer_;
  bool setup_sent_{false};
};

}  // namespace policy_runtime
