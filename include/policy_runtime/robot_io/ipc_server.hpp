#pragma once

#include <cstdint>
#include <span>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/robot_io/ipc_protocol.hpp"
#include "policy_runtime/robot_io/snapshot.hpp"

namespace policy_runtime {

class RobotIoIpcServer {
 public:
  // Consumes connected_socket on every return path. The socket must be a connected
  // AF_UNIX SOCK_STREAM or SOCK_SEQPACKET descriptor. The returned object owns both
  // daemon-created memfds, their mappings, and the socket.
  static Result<RobotIoIpcServer> create(int connected_socket, std::uint32_t axis_count,
                                         std::uint32_t generation);

  RobotIoIpcServer(const RobotIoIpcServer&) = delete;
  RobotIoIpcServer& operator=(const RobotIoIpcServer&) = delete;
  RobotIoIpcServer(RobotIoIpcServer&& other) noexcept;
  RobotIoIpcServer& operator=(RobotIoIpcServer&& other) noexcept;
  ~RobotIoIpcServer();

  Result<void> send_setup();
  Result<Snapshot<AxisCommand>> read_commands() const noexcept;
  Result<void> publish_feedback(std::span<const AxisFeedback> axes,
                                std::uint64_t sequence,
                                std::int64_t timestamp_ns) noexcept;
  Result<void> check_peer() const;
  Result<void> close();

  std::uint32_t axis_count() const noexcept { return axis_count_; }
  std::uint32_t generation() const noexcept { return generation_; }
  int command_fd() const noexcept { return command_fd_; }
  int feedback_fd() const noexcept { return feedback_fd_; }

 private:
  RobotIoIpcServer(int socket_fd, int command_fd, int feedback_fd,
                   void* command_mapping, std::size_t command_mapping_size,
                   void* feedback_mapping, std::size_t feedback_mapping_size,
                   std::uint32_t axis_count, std::uint32_t generation,
                   SnapshotRegion<AxisCommand> command_region,
                   SnapshotRegion<AxisFeedback> feedback_region) noexcept;

  void release_noexcept() noexcept;

  int socket_fd_{-1};
  int command_fd_{-1};
  int feedback_fd_{-1};
  void* command_mapping_{};
  std::size_t command_mapping_size_{};
  void* feedback_mapping_{};
  std::size_t feedback_mapping_size_{};
  std::uint32_t axis_count_{};
  std::uint32_t generation_{};
  SnapshotRegion<AxisCommand> command_region_{};
  SnapshotRegion<AxisFeedback> feedback_region_{};
  bool setup_sent_{false};
};

}  // namespace policy_runtime
