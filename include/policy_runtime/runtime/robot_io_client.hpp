#pragma once

#include <cstdint>
#include <optional>
#include <span>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/robot_io/ipc_protocol.hpp"
#include "policy_runtime/robot_io/snapshot.hpp"

namespace policy_runtime {

class RobotIoClient {
 public:
  // Consumes connected_socket on every return path and receives ownership of two
  // CLOEXEC memfd duplicates from the daemon. Stale daemon generations are rejected.
  static Result<RobotIoClient> connect(int connected_socket,
                                       std::uint32_t expected_generation);

  RobotIoClient(const RobotIoClient&) = delete;
  RobotIoClient& operator=(const RobotIoClient&) = delete;
  RobotIoClient(RobotIoClient&& other) noexcept;
  RobotIoClient& operator=(RobotIoClient&& other) noexcept;
  ~RobotIoClient();

  Result<void> publish_commands(std::span<const AxisCommand> axes,
                                std::uint64_t sequence,
                                std::int64_t timestamp_ns);
  Result<Snapshot<AxisFeedback>> read_feedback() const;
  Result<void> check_peer() const;
  Result<void> close();

  std::uint32_t axis_count() const noexcept { return axis_count_; }
  std::uint32_t generation() const noexcept { return generation_; }

 private:
  RobotIoClient(int socket_fd, int command_writer_fd, int feedback_reader_fd,
                void* command_mapping, std::size_t command_mapping_size,
                void* feedback_mapping, std::size_t feedback_mapping_size,
                std::uint32_t axis_count, std::uint32_t generation,
                SnapshotWriter<AxisCommand> command_writer,
                SnapshotReader<AxisFeedback> feedback_reader) noexcept;

  void release_noexcept() noexcept;

  int socket_fd_{-1};
  int command_writer_fd_{-1};
  int feedback_reader_fd_{-1};
  void* command_mapping_{};
  std::size_t command_mapping_size_{};
  void* feedback_mapping_{};
  std::size_t feedback_mapping_size_{};
  std::uint32_t axis_count_{};
  std::uint32_t generation_{};
  std::optional<SnapshotWriter<AxisCommand>> command_writer_;
  std::optional<SnapshotReader<AxisFeedback>> feedback_reader_;
};

}  // namespace policy_runtime
