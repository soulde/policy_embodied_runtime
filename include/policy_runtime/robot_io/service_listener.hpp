#pragma once

#include <cstdint>
#include <string>

#include <sys/types.h>

#include "policy_runtime/common/result.hpp"

namespace policy_runtime {

class DaemonSignalLatch;

// Owns the one-client service-mode IPC rendezvous. The generation file is
// also held under an exclusive lock so concurrent daemon incarnations cannot
// replace each other's socket.
class RobotIoServiceListener {
 public:
  static Result<RobotIoServiceListener> create(
      std::string socket_path, std::string generation_path,
      uid_t allowed_peer_uid);

  RobotIoServiceListener(const RobotIoServiceListener&) = delete;
  RobotIoServiceListener& operator=(const RobotIoServiceListener&) = delete;
  RobotIoServiceListener(RobotIoServiceListener&& other) noexcept;
  RobotIoServiceListener& operator=(RobotIoServiceListener&& other) noexcept;
  ~RobotIoServiceListener();

  Result<int> accept_authenticated(const DaemonSignalLatch& signals);
  std::uint32_t generation() const noexcept { return generation_; }

 private:
  RobotIoServiceListener(int generation_fd, std::string socket_path,
                         std::string generation_path,
                         std::uint32_t generation,
                         uid_t allowed_peer_uid) noexcept;

  void release_noexcept() noexcept;

  int listen_fd_{-1};
  int generation_fd_{-1};
  std::string socket_path_;
  std::string generation_path_;
  std::uint64_t socket_device_{};
  std::uint64_t socket_inode_{};
  std::uint64_t generation_device_{};
  std::uint64_t generation_inode_{};
  std::uint32_t generation_{};
  uid_t allowed_peer_uid_{};
};

}  // namespace policy_runtime
